//! The OFT Protocol (spec §6) over picoserve: `/oft/v1/{info,config,status,time,crossings,
//! samples,stream}`. JSON is built with serde-json-core into fixed buffers.

use crate::board::OFT_PORT;
use crate::state::{Config, Event, SeatConfig, ThresholdConfig, Timer, MAX_SEATS, TRACE_DT_US};
use core::fmt::Write as _;
use embassy_net::Stack;
use heapless::{String, Vec};
use picoserve::extract::{Query, State};
use picoserve::response::ws;
use picoserve::response::{Content, IntoResponse, Json, Response, StatusCode};
use picoserve::routing::{get, get_service, post};
use serde::{Deserialize, Serialize};

pub const WEB_TASK_POOL_SIZE: usize = 4;

#[derive(Clone, Copy)]
pub struct AppState(pub &'static Timer);

// ------------------------------------------------------------------ JSON shapes

#[derive(Serialize)]
struct CrossingJson {
    #[serde(rename = "type")]
    kind: &'static str,
    seq: u64,
    seat: u8,
    t_enter: u64,
    t_exit: u64,
    t_mid: u64,
    peak: i16,
    snr: i16,
    flags: Vec<&'static str, 4>,
}

fn crossing_json(seq: u64, c: &oft_core::Crossing) -> CrossingJson {
    use oft_core::CrossingFlags as F;
    let mut flags = Vec::new();
    for (bit, name) in [(F::CLIPPED, "clipped"), (F::LOW_SNR, "low_snr"), (F::HOP_ARTIFACT, "hop_artifact"), (F::TRUNCATED, "truncated")] {
        if c.flags.contains(bit) {
            let _ = flags.push(name);
        }
    }
    CrossingJson { kind: "crossing", seq, seat: c.seat, t_enter: c.t_enter, t_exit: c.t_exit, t_mid: c.t_mid, peak: c.peak, snr: c.snr, flags }
}

#[derive(Serialize)]
struct SeatStatusJson {
    seat: u8,
    rate_hz: u32,
    rssi: Option<i16>,
    floor: Option<i16>,
    enter: i16,
    exit: i16,
    crossing: bool,
}

/// Health of the Node link, so lost samples can be traced to their cause (counts since boot).
#[derive(Serialize)]
struct LinkJson {
    gap_samples: u32,
    crc_errors: u32,
    uart_errors: u32,
    node_overruns: Option<u16>,
    node_hop_errors: Option<u16>,
    node_gain: Option<u8>,
}

#[derive(Serialize)]
struct StatusJson {
    #[serde(rename = "type")]
    kind: &'static str,
    uptime_s: u64,
    time_us: u64,
    temp_c: Option<f32>,
    dropped: u32,
    clock_err_us: Option<f64>,
    clock_synced: bool,
    clock_source: &'static str,
    drift_ppm: Option<f64>,
    node_connected: bool,
    link: LinkJson,
    seats: Vec<SeatStatusJson, MAX_SEATS>,
}

#[derive(Serialize, Deserialize)]
struct SeatConfigJson {
    seat: u8,
    freq_mhz: u16,
    enabled: bool,
}

/// `{"mode":"auto","snr_enter":..,"hysteresis":..}` or `{"mode":"manual","enter":..,"exit":..}`.
/// Flat rather than an internally tagged enum: serde's tagged enums need `alloc`.
#[derive(Serialize, Deserialize)]
struct ThresholdJson {
    mode: String<8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    snr_enter: Option<i16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    hysteresis: Option<i16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    enter: Option<i16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    exit: Option<i16>,
}

#[derive(Serialize, Deserialize)]
struct ConfigJson {
    seats: Vec<SeatConfigJson, MAX_SEATS>,
    seats_per_node: u8,
    threshold: ThresholdJson,
    min_lap_ms: u32,
    /// Absent (older clients) means the default.
    #[serde(default)]
    exit_hold_ms: Option<u32>,
}

impl From<&Config> for ConfigJson {
    fn from(c: &Config) -> Self {
        let mut seats = Vec::new();
        for s in c.seats.iter().flatten() {
            let _ = seats.push(SeatConfigJson { seat: s.seat, freq_mhz: s.freq_mhz, enabled: s.enabled });
        }
        let threshold = match c.threshold {
            ThresholdConfig::Auto { snr_enter, hysteresis } => ThresholdJson { mode: String::try_from("auto").unwrap(), snr_enter: Some(snr_enter), hysteresis: Some(hysteresis), enter: None, exit: None },
            ThresholdConfig::Manual { enter, exit } => ThresholdJson { mode: String::try_from("manual").unwrap(), snr_enter: None, hysteresis: None, enter: Some(enter), exit: Some(exit) },
        };
        Self { seats, seats_per_node: c.seats_per_node, threshold, min_lap_ms: c.min_lap_ms, exit_hold_ms: Some(c.exit_hold_ms) }
    }
}

impl ConfigJson {
    fn into_config(self) -> Result<Config, &'static str> {
        let mut seats = [None; MAX_SEATS];
        for s in self.seats {
            if s.seat as usize >= MAX_SEATS {
                return Err("seat must be 0..7");
            }
            if !(5000..=6200).contains(&s.freq_mhz) {
                return Err("freq_mhz out of range");
            }
            seats[s.seat as usize] = Some(SeatConfig { seat: s.seat, freq_mhz: s.freq_mhz, enabled: s.enabled });
        }
        let t = &self.threshold;
        let threshold = match t.mode.as_str() {
            "auto" => ThresholdConfig::Auto { snr_enter: t.snr_enter.ok_or("snr_enter required")?, hysteresis: t.hysteresis.ok_or("hysteresis required")? },
            "manual" => ThresholdConfig::Manual { enter: t.enter.ok_or("enter required")?, exit: t.exit.ok_or("exit required")? },
            _ => return Err("threshold.mode must be auto or manual"),
        };
        Ok(Config { seats, seats_per_node: self.seats_per_node.clamp(1, 8), threshold, min_lap_ms: self.min_lap_ms, exit_hold_ms: self.exit_hold_ms.unwrap_or(Config::default().exit_hold_ms).min(1000) })
    }
}

#[derive(Deserialize)]
struct SinceQ {
    #[serde(default)]
    since: u64,
}

#[derive(Deserialize)]
struct SamplesQ {
    seat: u8,
    from: u64,
    to: u64,
}

// ------------------------------------------------------------------ builders

fn status_json(t: &Timer) -> StatusJson {
    t.with(|i| {
        let clock = i.host.clock();
        let mut seats = Vec::new();
        for s in i.config.enabled_seats() {
            let det = i.host.detector(s.seat);
            let (enter, exit) = det.map(|d| d.levels()).unwrap_or((0, 0));
            let tr = i.traces.get(s.seat as usize).and_then(|t| t.as_ref());
            let _ = seats.push(SeatStatusJson {
                seat: s.seat,
                rate_hz: tr.map(|t| t.rate_hz).unwrap_or(0),
                rssi: tr.map(|t| t.last_rssi),
                floor: det.and_then(|d| d.floor()),
                enter,
                exit,
                crossing: det.is_some_and(|d| d.is_crossing()),
            });
        }
        let now = crate::state::now_us();
        StatusJson {
            kind: "status",
            uptime_s: now / 1_000_000,
            time_us: now,
            temp_c: i.node_temp_dc.map(|t| t as f32 / 10.0),
            dropped: i.host.dropped_samples.wrapping_add(i.dropped_frames),
            clock_err_us: clock.is_synced().then(|| clock.residual_us()),
            clock_synced: clock.is_synced(),
            clock_source: i.host.sync_source(),
            drift_ppm: clock.is_synced().then(|| clock.drift_ppm()),
            node_connected: i.node_connected,
            link: LinkJson {
                gap_samples: i.host.dropped_samples,
                crc_errors: i.dropped_frames,
                uart_errors: i.uart_errors,
                node_overruns: i.node_status.map(|s| s.overruns),
                node_hop_errors: i.node_status.map(|s| s.hop_errors),
                node_gain: i.node_status.map(|s| s.gain_index),
            },
            seats,
        }
    })
}

/// `/info` and the WebSocket `hello` share this body; `hello` adds a few fields.
fn info_json(t: &Timer, hello: bool, out: &mut String<1024>) {
    let h = t.hello();
    let (seats, rate) = t.with(|i| {
        let mut v: Vec<(u8, u32), MAX_SEATS> = Vec::new();
        for s in i.config.enabled_seats() {
            let r = i.traces.get(s.seat as usize).and_then(|t| t.as_ref()).map(|t| t.rate_hz).unwrap_or(0);
            let _ = v.push((s.seat, r));
        }
        (v, 1000u32)
    });
    out.clear();
    let _ = write!(out, "{{");
    if hello {
        let _ = write!(out, "\"type\":\"hello\",\"timer\":\"oft-{:02x}{:02x}\",\"time_us\":{},\"encodings\":[\"json\"],", t.mac[4], t.mac[5], crate::state::now_us());
    }
    let _ = write!(
        out,
        "\"id\":\"oft-{:02x}{:02x}\",\"name\":\"OFT Timer\",\"boot_id\":\"{:08x}\",\"proto\":1,\"host\":\"rp2350\",\"fw\":\"{}\",\"level\":\"L2\",\"calibrated\":false,",
        t.mac[4],
        t.mac[5],
        t.boot_id,
        env!("CARGO_PKG_VERSION")
    );
    let _ = write!(out, "\"nodes\":[");
    if let Some(h) = h {
        let _ = write!(
            out,
            "{{\"id\":\"{:012x}\",\"chip\":\"esp32c5\",\"fw\":\"{}.{}.{}\",\"range_mhz\":[{},{}],\"max_seats\":{},\"link\":\"uart\"}}",
            h.chip_id, h.fw[0], h.fw[1], h.fw[2], h.freq_min_mhz, h.freq_max_mhz, h.max_seats
        );
    }
    let _ = write!(out, "],\"seats\":[");
    for (k, (seat, r)) in seats.iter().enumerate() {
        let _ = write!(out, "{}{{\"seat\":{},\"node\":0,\"rate_hz\":{}}}", if k > 0 { "," } else { "" }, seat, r);
    }
    let _ = write!(
        out,
        "],\"capabilities\":{{\"crossings\":{{\"algorithms\":[\"edge_mid\"],\"replay\":{},\"fields\":[\"t_enter\",\"t_exit\",\"peak\",\"snr\"]}},\
         \"samples.stream\":{{\"rate_hz\":{},\"decimate\":[1,2,5,10,20,50]}},\
         \"samples.history\":{{\"full_rate_s\":0,\"reduced_rate_s\":{},\"reduced_hz\":{}}},\
         \"config\":{{\"persist\":false,\"threshold_modes\":[\"auto\",\"manual\"],\"min_lap_ms\":true,\"seats_per_node\":true,\"exit_hold_ms\":true}},\
         \"time\":{{\"sync\":[\"sync\",\"ping\"]}}}}}}",
        crate::state::CROSSING_REPLAY,
        rate,
        crate::state::TRACE_LEN as u64 * TRACE_DT_US / 1_000_000,
        1_000_000 / TRACE_DT_US
    );
}

/// A raw, pre-built JSON body with a status code.
struct RawJson(StatusCode, String<1024>);

impl RawJson {
    fn ok(s: String<1024>) -> Self {
        RawJson(StatusCode::OK, s)
    }
    fn error(code: u16, msg: &str) -> Self {
        let mut s = String::new();
        let _ = write!(s, "{{\"error\":\"{}\"}}", msg);
        RawJson(StatusCode::new(code), s)
    }
    fn of<T: Serialize>(code: u16, v: &T) -> Self {
        let mut buf = [0u8; 1024];
        let mut s = String::new();
        if let Ok(n) = serde_json_core::to_slice(v, &mut buf) {
            let _ = s.push_str(core::str::from_utf8(&buf[..n]).unwrap_or("{}"));
        }
        RawJson(StatusCode::new(code), s)
    }
}

struct JsonBody(String<1024>);

impl Content for JsonBody {
    fn content_type(&self) -> &'static str {
        "application/json"
    }
    fn content_length(&self) -> usize {
        self.0.len()
    }
    async fn write_content<W: picoserve::io::Write>(self, mut writer: W) -> Result<(), W::Error> {
        writer.write_all(self.0.as_bytes()).await
    }
}

impl IntoResponse for RawJson {
    async fn write_to<R: picoserve::io::Read, W: picoserve::response::ResponseWriter<Error = R::Error>>(
        self,
        connection: picoserve::response::Connection<'_, R>,
        response_writer: W,
    ) -> Result<picoserve::ResponseSent, W::Error> {
        Response::new(self.0, JsonBody(self.1)).write_to(connection, response_writer).await
    }
}

// ------------------------------------------------------------------ handlers

async fn info(State(AppState(t)): State<AppState>) -> RawJson {
    let mut s = String::new();
    info_json(t, false, &mut s);
    RawJson::ok(s)
}

async fn status(State(AppState(t)): State<AppState>) -> Json<StatusJson> {
    Json(status_json(t))
}

async fn time(State(AppState(t)): State<AppState>) -> RawJson {
    let mut s = String::new();
    let _ = write!(s, "{{\"time_us\":{},\"boot_id\":\"{:08x}\"}}", crate::state::now_us(), t.boot_id);
    RawJson::ok(s)
}

async fn get_config(State(AppState(t)): State<AppState>) -> Json<ConfigJson> {
    Json(t.with(|i| ConfigJson::from(&i.config)))
}

async fn put_config(State(AppState(t)): State<AppState>, picoserve::extract::Json(c): picoserve::extract::Json<ConfigJson>) -> RawJson {
    match c.into_config() {
        Ok(cfg) => {
            t.set_config(cfg);
            RawJson::of(200, &ConfigJson::from(&cfg))
        }
        Err(e) => RawJson::error(400, e),
    }
}

async fn crossings(State(AppState(t)): State<AppState>, Query(q): Query<SinceQ>) -> RawJson {
    let mut s: String<1024> = String::new();
    let _ = write!(s, "{{\"boot_id\":\"{:08x}\",\"crossings\":[", t.boot_id);
    let items: Vec<(u64, oft_core::Crossing), 8> = t.with(|i| i.crossings.iter().filter(|(seq, _)| *seq > q.since).take(8).copied().collect());
    for (k, (seq, c)) in items.iter().enumerate() {
        let mut buf = [0u8; 256];
        let Ok(n) = serde_json_core::to_slice(&crossing_json(*seq, c), &mut buf) else { continue };
        // Leave room for the comma and the closing "]}"; the client asks again with `since`.
        if s.len() + n + 3 > s.capacity() {
            break;
        }
        if k > 0 {
            let _ = s.push(',');
        }
        let _ = s.push_str(core::str::from_utf8(&buf[..n]).unwrap_or(""));
    }
    let _ = s.push_str("]}");
    RawJson::ok(s)
}

async fn samples(State(AppState(t)): State<AppState>, Query(q): Query<SamplesQ>) -> RawJson {
    let mut s: String<1024> = String::new();
    let ok = t.with(|i| {
        let Some(tr) = i.traces.get(q.seat as usize).and_then(|t| t.as_ref()) else { return false };
        let t_first = tr.t_first();
        let floor = i.host.detector(q.seat).and_then(|d| d.floor()).unwrap_or(0);
        let mut first_t = None;
        let mut body: String<800> = String::new();
        for (k, &p) in tr.samples.iter().enumerate() {
            let tk = t_first + k as u64 * TRACE_DT_US;
            if tk < q.from || tk > q.to {
                continue;
            }
            if first_t.is_none() {
                first_t = Some(tk);
            }
            // Stop while a comma and the longest value ("-32768") still fit, so the list
            // never ends in a dangling comma or a cut-off number.
            if body.len() + 7 > body.capacity() {
                break;
            }
            if !body.is_empty() {
                let _ = body.push(',');
            }
            let _ = write!(body, "{}", p);
        }
        let Some(t0) = first_t else { return false };
        let _ = write!(s, "{{\"type\":\"samples\",\"seat\":{},\"t0\":{},\"dt_us\":{},\"p\":[{}],\"nf\":{}}}", q.seat, t0, TRACE_DT_US, body, floor);
        true
    });
    if ok {
        RawJson::ok(s)
    } else {
        RawJson::error(404, "no samples for that seat and window")
    }
}

// ------------------------------------------------------------------ WebSocket

#[derive(Deserialize, Default)]
struct SubMsg<'a> {
    #[serde(rename = "type")]
    kind: &'a str,
    #[serde(default)]
    crossings: Option<SubCrossings>,
    #[serde(default)]
    samples: Option<SubSamples>,
    #[serde(default)]
    status: bool,
}

#[derive(Deserialize, Default, Clone, Copy)]
struct SubCrossings {
    since: Option<u64>,
}

#[derive(Deserialize, Clone, Copy)]
struct SubSamples {
    #[serde(default = "one")]
    decimate: u32,
}

fn one() -> u32 {
    1
}

struct Stream(&'static Timer);

impl ws::WebSocketCallback for Stream {
    async fn run<R: picoserve::io::Read, W: picoserve::io::Write<Error = R::Error>>(self, mut rx: ws::SocketRx<R>, mut tx: ws::SocketTx<W>) -> Result<(), W::Error> {
        let t = self.0;
        let mut sub = t.events.subscriber().unwrap();
        let mut want_crossings = false;
        let mut want_samples: Option<SubSamples> = None;
        let mut want_status = false;
        let mut decim: [u32; MAX_SEATS] = [0; MAX_SEATS];
        let mut in_buf = [0u8; 512];
        let mut body: String<1024> = String::new();

        info_json(t, true, &mut body);
        tx.send_text(body.as_str()).await?;

        let close = loop {
            match rx.next_message(&mut in_buf, sub.next_message_pure()).await? {
                picoserve::futures::Either::First(Ok(ws::Message::Text(txt))) => {
                    match serde_json_core::from_str::<SubMsg>(txt) {
                        Ok((m, _)) if m.kind == "sub" => {
                            if let Some(since) = m.crossings.and_then(|c| c.since) {
                                let items: Vec<(u64, oft_core::Crossing), 16> = t.with(|i| i.crossings.iter().filter(|(s, _)| *s > since).take(16).copied().collect());
                                for (seq, c) in items {
                                    let mut buf = [0u8; 256];
                                    if let Ok(n) = serde_json_core::to_slice(&crossing_json(seq, &c), &mut buf) {
                                        tx.send_text(core::str::from_utf8(&buf[..n]).unwrap_or("{}")).await?;
                                    }
                                }
                            }
                            want_crossings = m.crossings.is_some();
                            want_samples = m.samples;
                            want_status = m.status;
                            tx.send_text("{\"type\":\"ack\",\"for\":\"sub\"}").await?;
                        }
                        Ok((m, _)) if m.kind == "ping" => {
                            body.clear();
                            let _ = write!(body, "{{\"type\":\"pong\",\"time_us\":{}}}", crate::state::now_us());
                            tx.send_text(body.as_str()).await?;
                        }
                        Ok(_) => tx.send_text("{\"type\":\"error\",\"code\":\"unknown_type\"}").await?,
                        Err(_) => tx.send_text("{\"type\":\"error\",\"code\":\"bad_json\"}").await?,
                    }
                }
                picoserve::futures::Either::First(Ok(ws::Message::Close(_))) => break None,
                picoserve::futures::Either::First(Ok(ws::Message::Ping(d))) => tx.send_pong(d).await?,
                picoserve::futures::Either::First(Ok(_)) => {}
                picoserve::futures::Either::First(Err(e)) => break Some((e.code(), "websocket error")),
                picoserve::futures::Either::Second(ev) => match ev {
                    Event::Crossing { seq, c } if want_crossings => {
                        let mut buf = [0u8; 256];
                        if let Ok(n) = serde_json_core::to_slice(&crossing_json(seq, &c), &mut buf) {
                            tx.send_text(core::str::from_utf8(&buf[..n]).unwrap_or("{}")).await?;
                        }
                    }
                    Event::Samples { seat, t0, dt_us, n, p, nf } => {
                        let Some(sel) = want_samples else { continue };
                        let d = sel.decimate.max(1);
                        let idx = seat as usize % MAX_SEATS;
                        body.clear();
                        let mut first_t = None;
                        let mut count = 0;
                        let mut inner: String<256> = String::new();
                        for (k, &v) in p[..n as usize].iter().enumerate() {
                            if decim[idx] == 0 {
                                if first_t.is_none() {
                                    first_t = Some(t0 + k as u64 * dt_us as u64);
                                }
                                if count > 0 {
                                    let _ = inner.push(',');
                                }
                                let _ = write!(inner, "{}", v);
                                count += 1;
                            }
                            decim[idx] = (decim[idx] + 1) % d;
                        }
                        if let Some(t0) = first_t {
                            let _ = write!(body, "{{\"type\":\"samples\",\"seat\":{},\"t0\":{},\"dt_us\":{},\"p\":[{}],\"nf\":{}}}", seat, t0, dt_us as u32 * d, inner, nf);
                            tx.send_text(body.as_str()).await?;
                        }
                    }
                    Event::Status if want_status => {
                        let mut buf = [0u8; 1024];
                        if let Ok(n) = serde_json_core::to_slice(&status_json(t), &mut buf) {
                            tx.send_text(core::str::from_utf8(&buf[..n]).unwrap_or("{}")).await?;
                        }
                    }
                    _ => {}
                },
            }
        };
        tx.close(close).await
    }
}

fn opt_bool(v: Option<bool>) -> &'static str {
    match v {
        Some(true) => "true",
        Some(false) => "false",
        None => "null",
    }
}

/// Progress and result of the Node wiring test.
async fn get_wiretest(State(AppState(t)): State<AppState>) -> RawJson {
    let w = t.with(|i| i.wire_test);
    let mut s = String::new();
    let _ = write!(
        s,
        "{{\"running\":{},\"done\":{},\"step\":\"{}\",\"en_ok\":{},\"en_ms\":{},\"boot_ok\":{},\"recovered\":{},\"recover_ms\":{}}}",
        w.running,
        w.done,
        w.step,
        opt_bool(w.en_ok),
        w.en_ms,
        opt_bool(w.boot_ok),
        opt_bool(w.recovered),
        w.recover_ms
    );
    RawJson::ok(s)
}

/// Start the Node wiring test (EN and BOOT lines). Bench diagnostic, not part of the OFT
/// Protocol: it resets the Node three times and stops timing while it runs.
async fn post_wiretest(State(AppState(t)): State<AppState>) -> RawJson {
    if t.with(|i| i.wire_test.running) {
        return RawJson::error(409, "the wiring test is already running");
    }
    t.with(|i| {
        i.wire_test = crate::state::WireTest { running: true, step: "starting", ..Default::default() };
    });
    crate::node::WIRE_TEST.signal(());
    get_wiretest(State(AppState(t))).await
}

/// Set by `POST /oft/v1/fw/bootsel`; the status task restarts the board once the reply is out.
static BOOTSEL_REQUESTED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// Restart into the ROM's USB bootloader (the `RP2350` drive), as holding BOOT while
/// plugging in does, so a new UF2 can be copied on without reaching the button.
/// Bench convenience, not part of the OFT Protocol: anyone on the network can call it.
async fn fw_bootsel() -> RawJson {
    BOOTSEL_REQUESTED.store(true, core::sync::atomic::Ordering::Relaxed);
    let mut s = String::new();
    let _ = s.push_str("{\"ok\":true,\"msg\":\"restarting into USB boot mode\"}");
    RawJson::ok(s)
}

async fn stream(State(AppState(t)): State<AppState>, upgrade: ws::WebSocketUpgrade) -> impl IntoResponse {
    upgrade.on_upgrade(Stream(t))
}

// ------------------------------------------------------------------ server

static CONFIG: picoserve::Config = picoserve::Config::const_default().keep_connection_alive();

/// One HTTP connection at a time per task; `WEB_TASK_POOL_SIZE` tasks run.
#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
pub async fn web_task(task_id: usize, stack: Stack<'static>, timer: &'static Timer) -> ! {
    let app = picoserve::Router::new()
        // The built-in diagnostics page: the same file every Timer host serves.
        .route("/", get_service(picoserve::response::File::html(include_str!("../../oft/web/timer.html"))))
        .route("/oft/v1/info", get(info))
        .route("/oft/v1/status", get(status))
        .route("/oft/v1/time", get(time))
        .route("/oft/v1/config", get(get_config).put(put_config))
        .route("/oft/v1/crossings", get(crossings))
        .route("/oft/v1/samples", get(samples))
        .route("/oft/v1/stream", get(stream))
        .route("/oft/v1/fw/bootsel", post(fw_bootsel))
        .route("/oft/v1/node/wiretest", get(get_wiretest).post(post_wiretest))
        .with_state(AppState(timer));
    let mut tcp_rx = [0u8; 1024];
    let mut tcp_tx = [0u8; 2048];
    let mut http = [0u8; 2048];
    picoserve::Server::new(&app, &CONFIG, &mut http).listen_and_serve(task_id, stack, OFT_PORT, &mut tcp_rx, &mut tcp_tx).await.into_never()
}

/// Publish a `Status` event once a second for WebSocket subscribers.
#[embassy_executor::task]
pub async fn status_tick_task(timer: &'static Timer) {
    let publisher = timer.events.publisher().unwrap();
    loop {
        embassy_time::Timer::after_secs(1).await;
        if BOOTSEL_REQUESTED.load(core::sync::atomic::Ordering::Relaxed) {
            embassy_rp::rom_data::reset_to_usb_boot(0, 0);
        }
        publisher.publish_immediate(Event::Status);
    }
}
