//! The OFT Protocol (spec §6) over picoserve: `/oft/v1/{info,config,status,time,crossings,
//! samples,stream}`. JSON is built with serde-json-core into fixed buffers.

use crate::board::{NodePins, Pins, MAX_NODES, OFT_PORT, OUT_PINS};
use crate::fw::{Fw, Other};
use crate::state::{Config, Event, NetConfig, NetNow, Saved, SeatConfig, ThresholdConfig, Timer, MAX_SEATS, TRACE_DT_US};
use core::fmt::Write as _;
use embassy_net::Stack;
use heapless::{String, Vec};
use picoserve::extract::{Query, State};
use picoserve::response::ws;
use picoserve::response::{Content, IntoResponse, Json, Response, StatusCode};
use picoserve::routing::{get, get_service, post};
use serde::{Deserialize, Serialize};

/// Connections served at once. A WebSocket holds one for as long as it is open, and a browser
/// showing the Timer's page opens several more in a burst; when every task is busy a new
/// connection is refused outright. Four was too few for the page plus one other client.
pub const WEB_TASK_POOL_SIZE: usize = 8;

/// Capacity of a JSON body built in place. Every connection task holds several of these, so
/// it is kept to what the ordinary replies need.
const BODY: usize = 1280;
/// Capacity for the two bodies that grow with the number of Nodes: `info` (the WebSocket
/// `hello`) and `status`. Eight seats and four Nodes come to about 2.4 KiB.
const BIG: usize = 2560;

#[derive(Clone, Copy)]
pub struct AppState(pub &'static Timer, pub &'static Fw);

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
    /// Id of the Node that serves the seat; null until a Node has introduced itself.
    node: Option<String<16>>,
    rate_hz: u32,
    rssi: Option<i16>,
    floor: Option<i16>,
    enter: i16,
    exit: i16,
    crossing: bool,
}

/// One Node's part of `status`.
#[derive(Serialize)]
struct NodeStatusJson {
    /// Which of the Timer's Node links it is on, from 0.
    slot: u8,
    /// Null until the Node has introduced itself.
    id: Option<String<16>>,
    connected: bool,
    temp_c: Option<f32>,
    clock_synced: bool,
    clock_source: &'static str,
    clock_err_us: Option<f64>,
    drift_ppm: Option<f64>,
    link: LinkJson,
}

/// Health of a Node link, so lost samples can be traced to their cause (counts since boot).
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
struct CrashJson {
    count: u32,
    text: String<200>,
    safe_pins: bool,
}

#[derive(Serialize)]
struct StatusJson {
    #[serde(rename = "type")]
    kind: &'static str,
    /// The last run ended in a crash: where and why, and how many in a row. This run is
    /// then on the default Node pins, whatever is saved.
    #[serde(skip_serializing_if = "Option::is_none")]
    crash: Option<CrashJson>,
    uptime_s: u64,
    time_us: u64,
    temp_c: Option<f32>,
    timer_temp_c: Option<f32>,
    dropped: u32,
    clock_err_us: Option<f64>,
    clock_synced: bool,
    clock_source: &'static str,
    drift_ppm: Option<f64>,
    node_connected: bool,
    /// What is wrong with the Timer, for race software (spec 6.3): empty when nothing is.
    problems: Vec<&'static str, 4>,
    link: LinkJson,
    /// Every wired Node. The Node fields above sum these up: the worst clock, the highest
    /// temperature, the counts added together, `node_connected` only if all are.
    nodes: Vec<NodeStatusJson, MAX_NODES>,
    seats: Vec<SeatStatusJson, MAX_SEATS>,
    /// WebSocket streams open now.
    clients: u32,
    config_rev: u32,
    /// The claim that holds control.
    control: Option<HolderJson>,
}

#[derive(Serialize)]
struct HolderJson {
    id: u32,
    name: String<{ oft_core::control::NAME_MAX }>,
}

#[derive(Serialize, Deserialize)]
struct SeatConfigJson {
    seat: u8,
    freq_mhz: u16,
    enabled: bool,
    /// This seat's own threshold; absent = the shared one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    threshold: Option<ThresholdJson>,
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

impl From<ThresholdConfig> for ThresholdJson {
    fn from(t: ThresholdConfig) -> Self {
        match t {
            ThresholdConfig::Auto { snr_enter, hysteresis } => ThresholdJson { mode: String::try_from("auto").unwrap(), snr_enter: Some(snr_enter), hysteresis: Some(hysteresis), enter: None, exit: None },
            ThresholdConfig::Manual { enter, exit } => ThresholdJson { mode: String::try_from("manual").unwrap(), snr_enter: None, hysteresis: None, enter: Some(enter), exit: Some(exit) },
        }
    }
}

impl ThresholdJson {
    fn into_threshold(&self) -> Result<ThresholdConfig, &'static str> {
        match self.mode.as_str() {
            "auto" => Ok(ThresholdConfig::Auto { snr_enter: self.snr_enter.ok_or("snr_enter required")?, hysteresis: self.hysteresis.ok_or("hysteresis required")? }),
            "manual" => Ok(ThresholdConfig::Manual { enter: self.enter.ok_or("enter required")?, exit: self.exit.ok_or("exit required")? }),
            _ => Err("threshold.mode must be auto or manual"),
        }
    }
}

#[derive(Serialize, Deserialize)]
pub(crate) struct ConfigJson {
    seats: Vec<SeatConfigJson, MAX_SEATS>,
    /// Dropped in spec v0.5. Still accepted (older clients, settings saved by older
    /// firmware) and ignored; never sent.
    #[serde(default, skip_serializing)]
    #[allow(dead_code)]
    seats_per_node: Option<u8>,
    threshold: ThresholdJson,
    min_lap_ms: u32,
    /// Absent (older clients) means the default.
    #[serde(default)]
    exit_hold_ms: Option<u32>,
    /// Absent (older clients, settings saved by older firmware) means off.
    #[serde(default)]
    pass_window_ms: u32,
}

impl From<&Config> for ConfigJson {
    fn from(c: &Config) -> Self {
        let mut seats = Vec::new();
        for s in c.seats.iter().flatten() {
            let _ = seats.push(SeatConfigJson { seat: s.seat, freq_mhz: s.freq_mhz, enabled: s.enabled, threshold: s.threshold.map(Into::into) });
        }
        Self { seats, seats_per_node: None, threshold: c.threshold.into(), min_lap_ms: c.min_lap_ms, exit_hold_ms: Some(c.exit_hold_ms), pass_window_ms: c.pass_window_ms }
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
            let threshold = match &s.threshold {
                Some(t) => Some(t.into_threshold()?),
                None => None,
            };
            seats[s.seat as usize] = Some(SeatConfig { seat: s.seat, freq_mhz: s.freq_mhz, enabled: s.enabled, threshold });
        }
        let threshold = self.threshold.into_threshold()?;
        if self.pass_window_ms > 10_000 {
            return Err("pass_window_ms must be 0..10000");
        }
        Ok(Config { seats, threshold, min_lap_ms: self.min_lap_ms, exit_hold_ms: self.exit_hold_ms.unwrap_or(Config::default().exit_hold_ms).min(1000), pass_window_ms: self.pass_window_ms })
    }
}

/// Network settings: `{"mode":"dhcp"}` or
/// `{"mode":"static","address":"192.168.1.50","prefix":24,"gateway":"192.168.1.1"}`.
#[derive(Serialize, Deserialize)]
pub(crate) struct NetJson {
    mode: String<8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    address: Option<String<15>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    prefix: Option<u8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    gateway: Option<String<15>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    dns: Option<String<15>>,
}

pub(crate) fn ip_text(a: [u8; 4]) -> String<15> {
    let mut s = String::new();
    let _ = write!(s, "{}.{}.{}.{}", a[0], a[1], a[2], a[3]);
    s
}

fn ip_parse(s: &str) -> Option<[u8; 4]> {
    s.parse::<core::net::Ipv4Addr>().ok().map(|a| a.octets())
}

impl From<NetConfig> for NetJson {
    fn from(n: NetConfig) -> Self {
        match n {
            NetConfig::Dhcp => NetJson { mode: String::try_from("dhcp").unwrap(), address: None, prefix: None, gateway: None, dns: None },
            NetConfig::Static { address, prefix, gateway, dns } => {
                NetJson { mode: String::try_from("static").unwrap(), address: Some(ip_text(address)), prefix: Some(prefix), gateway: gateway.map(ip_text), dns: dns.map(ip_text) }
            }
        }
    }
}

impl NetJson {
    fn into_net(&self) -> Result<NetConfig, &'static str> {
        match self.mode.as_str() {
            "dhcp" => Ok(NetConfig::Dhcp),
            "static" => {
                let address = self.address.as_deref().and_then(ip_parse).ok_or("address must be an IPv4 address like 192.168.1.50")?;
                let prefix = self.prefix.ok_or("prefix required (24 for a 255.255.255.0 mask)")?;
                if !(8..=30).contains(&prefix) {
                    return Err("prefix must be 8..30");
                }
                let host_bits = u32::from_be_bytes(address) & (u32::MAX >> prefix);
                if address[0] == 0 || address[0] >= 224 || address[0] == 127 || host_bits == 0 || host_bits == u32::MAX >> prefix {
                    return Err("address is not usable as a host address on that network");
                }
                let opt = |s: &Option<String<15>>, what: &'static str| match s.as_deref() {
                    None | Some("") => Ok(None),
                    Some(t) => ip_parse(t).map(Some).ok_or(what),
                };
                Ok(NetConfig::Static { address, prefix, gateway: opt(&self.gateway, "gateway must be an IPv4 address")?, dns: opt(&self.dns, "dns must be an IPv4 address")? })
            }
            _ => Err("mode must be dhcp or static"),
        }
    }
}

/// Validate a fixed address given as text (the USB console's way in).
#[cfg_attr(not(feature = "usb-console"), allow(dead_code))]
pub(crate) fn parse_net(address: &str, prefix: u8, gateway: Option<&str>) -> Result<NetConfig, &'static str> {
    let text = |s: &str| String::<15>::try_from(s).map_err(|_| "address must be an IPv4 address like 192.168.1.50");
    NetJson { mode: String::try_from("static").unwrap(), address: Some(text(address)?), prefix: Some(prefix), gateway: gateway.map(text).transpose()?, dns: None }.into_net()
}

#[derive(Serialize, Deserialize, Clone, Copy)]
struct NodePinsJson {
    tx: u8,
    rx: u8,
}

/// The Node wiring as GPIO numbers: the body of `PUT /pins`, and how it is stored. One
/// entry in `nodes` per Node, Node 1 first.
#[derive(Serialize, Deserialize, Clone)]
pub(crate) struct PinsJson {
    #[serde(default)]
    nodes: Vec<NodePinsJson, MAX_NODES>,
    /// Firmware that drove one Node stored its link here; still read, never written.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    tx: Option<u8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    rx: Option<u8>,
    sync: u8,
    /// `null` or left out: not wired.
    #[serde(default)]
    en: Option<u8>,
    #[serde(default)]
    boot: Option<u8>,
}

impl From<Pins> for PinsJson {
    fn from(p: Pins) -> Self {
        PinsJson { nodes: p.nodes.iter().flatten().map(|n| NodePinsJson { tx: n.tx, rx: n.rx }).collect(), tx: None, rx: None, sync: p.sync, en: p.en, boot: p.boot }
    }
}

impl PinsJson {
    fn into_pins(self) -> Result<Pins, &'static str> {
        let mut nodes = [None; MAX_NODES];
        for (slot, n) in nodes.iter_mut().zip(&self.nodes) {
            *slot = Some(NodePins { tx: n.tx, rx: n.rx });
        }
        if let (true, Some(tx), Some(rx)) = (self.nodes.is_empty(), self.tx, self.rx) {
            nodes[0] = Some(NodePins { tx, rx });
        }
        let p = Pins { nodes, sync: self.sync, en: self.en, boot: self.boot };
        p.check().map(|()| p)
    }
}

/// What the settings sectors hold.
#[derive(Serialize, Deserialize)]
pub(crate) struct SavedJson {
    #[serde(default)]
    config: Option<ConfigJson>,
    #[serde(default)]
    network: Option<NetJson>,
    /// Absent (settings saved by older firmware, or never changed) means the defaults.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pins: Option<PinsJson>,
}

impl From<&Saved> for SavedJson {
    fn from(s: &Saved) -> Self {
        SavedJson { config: s.config.as_ref().map(ConfigJson::from), network: Some(s.net.into()), pins: (s.pins != Pins::default()).then(|| s.pins.into()) }
    }
}

impl SavedJson {
    /// Parts that do not validate fall back to their defaults, one by one.
    pub(crate) fn into_saved(self) -> Saved {
        Saved {
            config: self.config.and_then(|c| c.into_config().ok()),
            net: self.network.and_then(|n| n.into_net().ok()).unwrap_or_default(),
            pins: self.pins.and_then(|p| p.into_pins().ok()).unwrap_or_default(),
        }
    }
}

#[derive(Serialize)]
struct NodePinChoicesJson {
    tx: &'static [u8],
    rx: &'static [u8],
}

#[derive(Serialize)]
struct PinChoicesJson {
    /// What each Node's link may use, Node 1 first: as many entries as the Timer can drive.
    nodes: [NodePinChoicesJson; MAX_NODES],
    /// For `sync`, and for `en` and `boot` where they are wired.
    out: &'static [u8],
}

/// `GET /pins`: the saved pins, the ones this boot is using, and what each may be.
#[derive(Serialize)]
struct PinsStateJson {
    saved: PinsJson,
    using: PinsJson,
    defaults: PinsJson,
    choices: PinChoicesJson,
    /// The saved pins differ from what this boot started with.
    restart_needed: bool,
}

#[derive(Serialize)]
struct NetNowJson {
    address: String<15>,
    prefix: u8,
    #[serde(skip_serializing_if = "Option::is_none")]
    gateway: Option<String<15>>,
}

/// `GET /network`: the saved settings, what this boot is using, and the address right now.
#[derive(Serialize)]
struct NetStateJson {
    saved: NetJson,
    /// "dhcp", "static", or "dhcp (recovery jumper)".
    using: &'static str,
    now: Option<NetNowJson>,
    /// The saved settings differ from what this boot started with.
    restart_needed: bool,
}

#[derive(Serialize)]
struct SaveStateJson {
    /// A power-on config is stored.
    saved: bool,
    /// The config in use is the stored one.
    current_is_saved: bool,
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

fn node_id(h: &oft_nodelink::msg::Hello) -> String<16> {
    let mut id = String::new();
    let _ = write!(id, "{:012x}", h.chip_id);
    id
}

fn status_json(t: &Timer) -> StatusJson {
    t.with(|i| {
        let count = i.node_count();
        let mut seats = Vec::new();
        for s in i.config.enabled_seats() {
            let det = i.hosts.detector(s.seat);
            let (enter, exit) = det.map(|d| d.levels()).unwrap_or((0, 0));
            let tr = i.traces.get(s.seat as usize).and_then(|t| t.as_ref());
            let _ = seats.push(SeatStatusJson {
                seat: s.seat,
                node: i.hosts.seat_node(s.seat).and_then(|n| i.hosts.hello(n as usize)).map(node_id),
                rate_hz: tr.map(|t| t.rate_hz).unwrap_or(0),
                rssi: tr.map(|t| t.last_rssi),
                floor: det.and_then(|d| d.floor()),
                enter,
                exit,
                crossing: det.is_some_and(|d| d.is_crossing()),
            });
        }
        let mut nodes: Vec<NodeStatusJson, MAX_NODES> = Vec::new();
        // The Node whose clock says most about the Timer's: the first connected one that is
        // not synced, else the connected one with the largest error.
        let mut worst = 0;
        let mut worst_err = -1.0;
        for n in 0..count {
            let (link, st) = (i.hosts.link(n), &i.nodes[n]);
            let clock = link.clock();
            if st.connected {
                let err = if clock.is_synced() { clock.residual_us() } else { f64::INFINITY };
                if err > worst_err {
                    (worst, worst_err) = (n, err);
                }
            }
            let _ = nodes.push(NodeStatusJson {
                slot: n as u8,
                id: link.hello().map(node_id),
                connected: st.connected,
                temp_c: st.temp_dc.map(|t| t as f32 / 10.0),
                clock_synced: clock.is_synced(),
                clock_source: link.sync_source(),
                clock_err_us: clock.is_synced().then(|| clock.residual_us()),
                drift_ppm: clock.is_synced().then(|| clock.drift_ppm()),
                link: LinkJson {
                    gap_samples: link.dropped_samples,
                    crc_errors: st.dropped_frames,
                    uart_errors: st.uart_errors,
                    node_overruns: st.status.map(|s| s.overruns),
                    node_hop_errors: st.status.map(|s| s.hop_errors),
                    node_gain: st.status.map(|s| s.gain_index),
                },
            });
        }
        let sum = |f: fn(&LinkJson) -> u32| nodes.iter().fold(0u32, |a, n| a.wrapping_add(f(&n.link)));
        let sum_opt = |f: fn(&LinkJson) -> Option<u16>| nodes.iter().filter_map(|n| f(&n.link)).reduce(|a, b| a.saturating_add(b));
        let link = LinkJson {
            gap_samples: sum(|l| l.gap_samples),
            crc_errors: sum(|l| l.crc_errors),
            uart_errors: sum(|l| l.uart_errors),
            node_overruns: sum_opt(|l| l.node_overruns),
            node_hop_errors: sum_opt(|l| l.node_hop_errors),
            node_gain: nodes.first().and_then(|n| n.link.node_gain),
        };
        let now = crate::state::now_us();
        let w = nodes.get(worst);
        StatusJson {
            kind: "status",
            crash: i.crash.as_ref().map(|c| CrashJson { count: c.count, text: c.text.clone(), safe_pins: c.safe_pins }),
            uptime_s: now / 1_000_000,
            time_us: now,
            temp_c: nodes.iter().filter_map(|n| n.temp_c).reduce(f32::max),
            timer_temp_c: i.timer_temp_dc.map(|t| t as f32 / 10.0),
            dropped: link.gap_samples.wrapping_add(link.crc_errors),
            clock_err_us: w.and_then(|n| n.clock_err_us),
            clock_synced: w.is_some_and(|n| n.clock_synced),
            clock_source: w.map_or("none", |n| n.clock_source),
            drift_ppm: w.and_then(|n| n.drift_ppm),
            node_connected: !nodes.is_empty() && nodes.iter().all(|n| n.connected),
            problems: {
                let mut p = Vec::new();
                if nodes.iter().any(|n| !n.connected) {
                    let _ = p.push("node_missing");
                }
                if i.crash.as_ref().is_some_and(|c| c.safe_pins) {
                    let _ = p.push("safe_mode");
                }
                if i.crash.is_some() {
                    let _ = p.push("crashed");
                }
                p
            },
            link,
            nodes,
            seats,
            clients: WS_CLIENTS.load(portable_atomic::Ordering::Relaxed),
            config_rev: i.config_rev,
            control: i.control.holder().map(|h| HolderJson { id: h.id, name: String::try_from(h.name.as_str()).unwrap_or_default() }),
        }
    })
}

/// `/info` and the WebSocket `hello` share this body; `hello` adds a few fields.
fn info_json(t: &Timer, self_update: bool, hello: bool, out: &mut String<BIG>) {
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
    let rate = 1000u32;
    t.with(|i| {
        // `nodes` lists the Nodes that have introduced themselves, in link order; a seat's
        // `node` is its Node's place in that list.
        let mut place = [None; MAX_NODES];
        let _ = write!(out, "\"nodes\":[");
        let mut listed = 0;
        for n in 0..i.node_count() {
            let Some(h) = i.hosts.hello(n) else { continue };
            let _ = write!(
                out,
                "{}{{\"id\":\"{:012x}\",\"chip\":\"{}\",\"fw\":\"{}.{}.{}\",\"range_mhz\":[{},{}],\"max_seats\":{},\"level\":{{\"unit\":\"{}\",\"scale\":{}}},\"link\":\"uart\",\"slot\":{},\"connected\":{}}}",
                if listed > 0 { "," } else { "" },
                h.chip_id,
                h.chip_name(),
                h.fw[0],
                h.fw[1],
                h.fw[2],
                h.freq_min_mhz,
                h.freq_max_mhz,
                h.max_seats,
                h.level().unit(),
                h.level().scale(),
                n,
                i.nodes[n].connected
            );
            place[n] = Some(listed);
            listed += 1;
        }
        let _ = write!(out, "],\"seats\":[");
        for (k, s) in i.config.enabled_seats().enumerate() {
            let r = i.traces.get(s.seat as usize).and_then(|t| t.as_ref()).map(|t| t.rate_hz).unwrap_or(0);
            let _ = write!(out, "{}{{\"seat\":{},\"node\":", if k > 0 { "," } else { "" }, s.seat);
            match i.hosts.seat_node(s.seat).and_then(|n| place[n as usize]) {
                Some(p) => {
                    let _ = write!(out, "{}", p);
                }
                // No Node is serving it (none answering, or none with room).
                None => {
                    let _ = out.push_str("null");
                }
            }
            let _ = write!(out, ",\"rate_hz\":{}}}", r);
        }
    });
    let _ = write!(
        out,
        "],\"capabilities\":{{\"crossings\":{{\"algorithms\":[\"edge_mid\"],\"replay\":{},\"fields\":[\"t_enter\",\"t_exit\",\"peak\",\"snr\"]}},\
         \"samples.stream\":{{\"rate_hz\":{},\"decimate\":[1,2,5,10,20,50]}},\
         \"samples.history\":{{\"full_rate_s\":0,\"reduced_rate_s\":{},\"reduced_hz\":{}}},\
         \"config\":{{\"persist\":true,\"threshold_modes\":[\"auto\",\"manual\"],\"min_lap_ms\":true,\"seat_thresholds\":true,\"exit_hold_ms\":true,\"pass_window_ms\":true}},\
         \"control\":{{}},\
         \"time\":{{\"sync\":[\"sync\",\"ping\"]}}}},\
         \"manage_url\":\"/\",\
         \"manage\":{{\"network\":{{\"static\":true}},\
         \"pins\":{{\"max_nodes\":{}}},{}\
         \"fw.update\":{{\"nodes\":[\"uart\"],\"self\":{}}}}}}}",
        crate::state::CROSSING_REPLAY,
        rate,
        crate::state::TRACE_LEN as u64 * TRACE_DT_US / 1_000_000,
        1_000_000 / TRACE_DT_US,
        MAX_NODES,
        // Present when this build has the serial console on the USB-C port.
        if cfg!(feature = "usb-console") { "\"usb.console\":{}," } else { "" },
        self_update
    );
}

/// A raw, pre-built JSON body with a status code.
struct RawJson(StatusCode, String<BODY>);

impl RawJson {
    fn ok(s: String<BODY>) -> Self {
        RawJson(StatusCode::OK, s)
    }
    fn error(code: u16, msg: &str) -> Self {
        let mut s = String::new();
        let _ = write!(s, "{{\"error\":\"{}\"}}", msg);
        RawJson(StatusCode::new(code), s)
    }
    fn of<T: Serialize>(code: u16, v: &T) -> Self {
        let mut buf = [0u8; BODY];
        let mut s = String::new();
        if let Ok(n) = serde_json_core::to_slice(v, &mut buf) {
            let _ = s.push_str(core::str::from_utf8(&buf[..n]).unwrap_or("{}"));
        }
        RawJson(StatusCode::new(code), s)
    }
}

struct JsonBody(String<BODY>);

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

/// The `/info` body. It outgrows `RawJson`, and a body that size carried in the reply would
/// be copied into every connection task several times over. Sent chunked instead: it is
/// built once, where it is written.
struct InfoChunks(&'static Timer, bool);

impl picoserve::response::chunked::Chunks for InfoChunks {
    fn content_type(&self) -> &'static str {
        "application/json"
    }
    async fn write_chunks<W: picoserve::io::Write>(self, mut chunk_writer: picoserve::response::chunked::ChunkWriter<W>) -> Result<picoserve::response::chunked::ChunksWritten, W::Error> {
        let mut s: String<BIG> = String::new();
        info_json(self.0, self.1, false, &mut s);
        chunk_writer.write_chunk(s.as_bytes()).await?;
        chunk_writer.finalize().await
    }
}

// ------------------------------------------------------------------ control (spec §6.6)

fn control_changed(t: &Timer) {
    t.events.immediate_publisher().publish_immediate(Event::Control);
}

/// Whether a request may change the Timer: while a claim holds control, only with its key
/// in the `OFT-Control` header. The refusal is the reply to send.
fn may_write(t: &Timer, parts: &picoserve::request::RequestParts<'_>) -> Result<(), RawJson> {
    let key = parts.headers().get("OFT-Control").and_then(|v| v.as_str().ok());
    let (expired, holder) = t.with(|i| {
        let expired = i.control.expire(crate::state::now_us());
        (expired, if i.control.allows(key) { None } else { i.control.holder().map(|h| h.name) })
    });
    if expired {
        control_changed(t);
    }
    match holder {
        None => Ok(()),
        Some(name) => {
            let mut msg: String<64> = String::new();
            let _ = write!(msg, "controlled by {}", name.as_str());
            Err(RawJson::error(423, &msg))
        }
    }
}

/// An argument of every handler that changes the Timer: the request is refused before the
/// handler runs unless it may write.
struct Writer;

impl<'r> picoserve::extract::FromRequestParts<'r, AppState> for Writer {
    type Rejection = RawJson;
    async fn from_request_parts(state: &'r AppState, parts: &picoserve::request::RequestParts<'r>) -> Result<Self, RawJson> {
        may_write(state.0, parts).map(|()| Writer)
    }
}

/// The reply of `/control`: the standing claims, oldest first.
fn control_json(t: &Timer) -> RawJson {
    let mut s: String<BODY> = String::new();
    t.with(|i| {
        let holder = i.control.holder().map(|h| h.id);
        let _ = s.push_str("{\"holder\":");
        opt_u32(holder, &mut s);
        let _ = s.push_str(",\"claims\":[");
        for (k, c) in i.control.claims().enumerate() {
            let _ = write!(
                s,
                "{}{{\"id\":{},\"name\":\"{}\",\"holds\":{},\"connected\":{}}}",
                if k > 0 { "," } else { "" },
                c.id,
                c.name.as_str(),
                Some(c.id) == holder,
                c.away_until.is_none()
            );
        }
        let _ = s.push_str("]}");
    });
    RawJson::ok(s)
}

async fn get_control(State(AppState(t, _)): State<AppState>) -> RawJson {
    control_json(t)
}

/// `{"grant": <claim id>}` or `{"grant": null}`. The field must be there: an empty body
/// must not free control by accident.
#[derive(Deserialize)]
struct GrantJson {
    #[serde(default, deserialize_with = "present")]
    grant: Option<Option<u32>>,
}

fn present<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Option<Option<u32>>, D::Error> {
    Option::<u32>::deserialize(d).map(Some)
}

/// An argument of a handler that only the Timer's own page may call: a browser sends
/// `Origin` with every POST, and for a page this Timer served it names the host the
/// request went to. Pages from elsewhere cannot fake it, and other clients do not send it.
struct OwnPage;

impl<'r> picoserve::extract::FromRequestParts<'r, AppState> for OwnPage {
    type Rejection = RawJson;
    async fn from_request_parts(_: &'r AppState, parts: &picoserve::request::RequestParts<'r>) -> Result<Self, RawJson> {
        let get = |name| parts.headers().get(name).and_then(|v| v.as_str().ok());
        match (get("Origin"), get("Host")) {
            (Some(origin), Some(host)) if oft_core::control::same_origin(origin, host) => Ok(OwnPage),
            _ => Err(RawJson::error(403, oft_core::control::OWN_PAGE_ONLY)),
        }
    }
}

/// Hand control to a standing claim, or to nobody. Needs no key: it is the way back in
/// when the holder is the wrong one. Only for the Timer's own page, so that no other
/// client can take control for itself.
async fn post_control(State(AppState(t, _)): State<AppState>, _: OwnPage, picoserve::extract::Json(g): picoserve::extract::Json<GrantJson>) -> RawJson {
    let Some(grant) = g.grant else { return RawJson::error(400, "expected grant: a claim id or null") };
    if t.with(|i| i.control.grant(grant)).is_err() {
        return RawJson::error(404, "no such claim");
    }
    control_changed(t);
    control_json(t)
}

/// Where control stands, for one stream: the holder, the stream's own claim, and the key
/// if that claim is the holder.
fn control_msg(t: &Timer, mine: Option<(u32, u32)>, out: &mut String<BODY>) {
    out.clear();
    t.with(|i| {
        // Another stream may have taken the claim up since (it came back with the key).
        let mine = mine.filter(|(id, owner)| i.control.get(*id).is_some_and(|c| c.owner == *owner)).map(|(id, _)| id);
        let _ = out.push_str("{\"type\":\"control\",\"holder\":");
        let _ = match i.control.holder() {
            Some(h) => write!(out, "{{\"id\":{},\"name\":\"{}\"}}", h.id, h.name.as_str()),
            None => write!(out, "null"),
        };
        let _ = out.push_str(",\"claim\":");
        opt_u32(mine, out);
        if let Some(h) = i.control.holder().filter(|h| Some(h.id) == mine) {
            let _ = write!(out, ",\"key\":\"{}\"", core::str::from_utf8(&h.key_text()).unwrap_or(""));
        }
        let _ = out.push('}');
    });
}

/// A stream's claim, ended when the stream is, however that happens. A holder keeps
/// control for a while after its stream drops, to come back to.
struct StreamClaim(&'static Timer, Option<(u32, u32)>);

impl Drop for StreamClaim {
    fn drop(&mut self) {
        if let Some(mine) = self.1 {
            self.0.with(|i| i.control.leave(mine, true, crate::state::now_us()));
            control_changed(self.0);
        }
    }
}

// ------------------------------------------------------------------ handlers

async fn info(State(AppState(t, fw)): State<AppState>) -> picoserve::response::chunked::ChunkedResponse<InfoChunks> {
    fw.note_http();
    picoserve::response::chunked::ChunkedResponse::new(InfoChunks(t, fw.self_update()))
}

async fn status(State(AppState(t, _)): State<AppState>) -> Json<StatusJson> {
    Json(status_json(t))
}

async fn time(State(AppState(t, _)): State<AppState>) -> RawJson {
    let mut s = String::new();
    let _ = write!(s, "{{\"time_us\":{},\"boot_id\":\"{:08x}\"}}", crate::state::now_us(), t.boot_id);
    RawJson::ok(s)
}

async fn get_config(State(AppState(t, _)): State<AppState>) -> Json<ConfigJson> {
    Json(t.with(|i| ConfigJson::from(&i.config)))
}

async fn put_config(State(AppState(t, _)): State<AppState>, _: Writer, picoserve::extract::Json(c): picoserve::extract::Json<ConfigJson>) -> RawJson {
    match c.into_config() {
        Ok(cfg) => {
            t.set_config(cfg);
            RawJson::of(200, &ConfigJson::from(&cfg))
        }
        Err(e) => RawJson::error(400, e),
    }
}

fn save_state(t: &Timer) -> RawJson {
    RawJson::of(200, &t.with(|i| SaveStateJson { saved: i.saved.config.is_some(), current_is_saved: i.saved.config == Some(i.config) }))
}

async fn get_config_save(State(AppState(t, _)): State<AppState>) -> RawJson {
    save_state(t)
}

/// Keep the config in use as the power-on config.
async fn post_config_save(State(AppState(t, fw)): State<AppState>, _: Writer) -> RawJson {
    let saved = t.with(|i| Saved { config: Some(i.config), ..i.saved });
    match crate::settings::save(t, fw, saved).await {
        Ok(()) => save_state(t),
        Err(e) => RawJson::error(500, e),
    }
}

/// Forget the power-on config: the next start uses the built-in defaults.
async fn delete_config_save(State(AppState(t, fw)): State<AppState>, _: Writer) -> RawJson {
    let saved = t.with(|i| Saved { config: None, ..i.saved });
    match crate::settings::save(t, fw, saved).await {
        Ok(()) => save_state(t),
        Err(e) => RawJson::error(500, e),
    }
}

fn net_state(t: &Timer) -> RawJson {
    RawJson::of(
        200,
        &t.with(|i| NetStateJson {
            saved: i.saved.net.into(),
            using: match (i.net_recovery, i.net_boot) {
                (true, _) => "dhcp (recovery jumper)",
                (false, NetConfig::Dhcp) => "dhcp",
                (false, NetConfig::Static { .. }) => "static",
            },
            now: i.net_now.map(|n: NetNow| NetNowJson { address: ip_text(n.address), prefix: n.prefix, gateway: n.gateway.map(ip_text) }),
            restart_needed: i.net_recovery || i.saved.net != i.net_boot,
        }),
    )
}

async fn get_network(State(AppState(t, _)): State<AppState>) -> RawJson {
    net_state(t)
}

/// Save new network settings. They take effect at the next restart.
async fn put_network(State(AppState(t, fw)): State<AppState>, _: Writer, picoserve::extract::Json(n): picoserve::extract::Json<NetJson>) -> RawJson {
    let net = match n.into_net() {
        Ok(net) => net,
        Err(e) => return RawJson::error(400, e),
    };
    let saved = t.with(|i| Saved { net, ..i.saved });
    match crate::settings::save(t, fw, saved).await {
        Ok(()) => net_state(t),
        Err(e) => RawJson::error(500, e),
    }
}

fn pins_state(t: &Timer) -> RawJson {
    RawJson::of(
        200,
        &t.with(|i| PinsStateJson {
            saved: i.saved.pins.into(),
            using: i.pins_boot.into(),
            defaults: Pins::default().into(),
            choices: PinChoicesJson { nodes: core::array::from_fn(|n| NodePinChoicesJson { tx: crate::board::tx_pins(n), rx: crate::board::rx_pins(n) }), out: &OUT_PINS },
            restart_needed: i.saved.pins != i.pins_boot,
        }),
    )
}

async fn get_pins(State(AppState(t, _)): State<AppState>) -> RawJson {
    pins_state(t)
}

/// Save new Node pins. They take effect at the next restart.
async fn put_pins(State(AppState(t, fw)): State<AppState>, _: Writer, picoserve::extract::Json(p): picoserve::extract::Json<PinsJson>) -> RawJson {
    let pins = match p.into_pins() {
        Ok(pins) => pins,
        Err(e) => return RawJson::error(400, e),
    };
    let saved = t.with(|i| Saved { pins, ..i.saved });
    match crate::settings::save(t, fw, saved).await {
        Ok(()) => pins_state(t),
        Err(e) => RawJson::error(500, e),
    }
}

async fn post_restart(State(AppState(_, fw)): State<AppState>, _: Writer) -> RawJson {
    if fw.on_trial() {
        return RawJson::error(409, "this firmware is still on trial after an update; wait for it to confirm");
    }
    fw.restart_soon();
    let mut s = String::new();
    let _ = s.push_str("{\"restarting\":true}");
    RawJson(StatusCode::ACCEPTED, s)
}

async fn crossings(State(AppState(t, _)): State<AppState>, Query(q): Query<SinceQ>) -> RawJson {
    let mut s: String<BODY> = String::new();
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

async fn samples(State(AppState(t, _)): State<AppState>, Query(q): Query<SamplesQ>) -> RawJson {
    let mut s: String<BODY> = String::new();
    let ok = t.with(|i| {
        let Some(tr) = i.traces.get(q.seat as usize).and_then(|t| t.as_ref()) else { return false };
        let t_first = tr.t_first();
        let floor = i.hosts.detector(q.seat).and_then(|d| d.floor()).unwrap_or(0);
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
    // `control` messages (spec §6.6).
    #[serde(default)]
    claim: Option<&'a str>,
    #[serde(default)]
    key: Option<&'a str>,
    #[serde(default)]
    release: bool,
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

struct Stream(&'static Timer, &'static Fw);

/// WebSocket streams open now (`clients` in `/status`).
static WS_CLIENTS: portable_atomic::AtomicU32 = portable_atomic::AtomicU32::new(0);

/// Counts one stream for as long as it lives, however it ends.
struct ClientCount;
impl ClientCount {
    fn new() -> Self {
        WS_CLIENTS.fetch_add(1, portable_atomic::Ordering::Relaxed);
        Self
    }
}
impl Drop for ClientCount {
    fn drop(&mut self) {
        WS_CLIENTS.fetch_sub(1, portable_atomic::Ordering::Relaxed);
    }
}

impl ws::WebSocketCallback for Stream {
    async fn run<R: picoserve::io::Read, W: picoserve::io::Write<Error = R::Error>>(self, mut rx: ws::SocketRx<R>, mut tx: ws::SocketTx<W>) -> Result<(), W::Error> {
        let t = self.0;
        let _counted = ClientCount::new();
        // One event subscription per stream; with all of them taken, say so and go.
        let Ok(mut sub) = t.events.subscriber() else { return tx.close(Some((1013, "too many streams are open"))).await };
        let mut claim = StreamClaim(t, None);
        let mut want_crossings = false;
        let mut want_samples: Option<SubSamples> = None;
        let mut want_status = false;
        let mut decim: [u32; MAX_SEATS] = [0; MAX_SEATS];
        let mut in_buf = [0u8; 512];
        let mut body: String<BODY> = String::new();
        // `hello` and `status` are built here; both outgrow `body`.
        let mut big: String<BIG> = String::new();

        info_json(t, self.1.self_update(), true, &mut big);
        tx.send_text(big.as_str()).await?;
        control_msg(t, claim.1, &mut body);
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
                        // Every stream, this one included, hears the outcome as an event.
                        Ok((m, _)) if m.kind == "control" && m.release => {
                            if let Some(mine) = claim.1.take() {
                                t.with(|i| i.control.leave(mine, false, crate::state::now_us()));
                            }
                            control_changed(t);
                        }
                        Ok((m, _)) if m.kind == "control" => match m.claim {
                            Some(name) => match t.with(|i| i.control.claim(claim.1, name, m.key, crate::state::now_us() as u32)) {
                                Ok(mine) => {
                                    claim.1 = Some(mine);
                                    control_changed(t);
                                }
                                Err(_) => tx.send_text("{\"type\":\"error\",\"code\":\"control_full\",\"msg\":\"this Timer has no room for another claim\"}").await?,
                            },
                            None => tx.send_text("{\"type\":\"error\",\"code\":\"bad_control\",\"msg\":\"expected claim or release\"}").await?,
                        },
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
                    Event::Control => {
                        control_msg(t, claim.1, &mut body);
                        tx.send_text(body.as_str()).await?;
                    }
                    Event::Status if want_status => {
                        // Safety: `to_slice` writes valid UTF-8, and `n` is how much of it.
                        let buf = unsafe { big.as_mut_vec() };
                        let _ = buf.resize(BIG, 0);
                        let n = serde_json_core::to_slice(&status_json(t), buf).unwrap_or(0);
                        buf.truncate(n);
                        if n > 0 {
                            tx.send_text(big.as_str()).await?;
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

/// Progress and result of the Node restart test. `via` says how the Nodes were restarted:
/// `wires` (EN and BOOT) or `link` (messages on each Node's link). The top-level verdicts are for all the
/// Nodes together (the slowest time, and a pass only if every Node passed); `nodes` has
/// each one's.
async fn get_wiretest(State(AppState(t, _)): State<AppState>) -> RawJson {
    let (w, count) = t.with(|i| (i.wire_test, i.node_count()));
    let nodes = &w.nodes[..count];
    let all = |f: fn(&crate::state::NodeWireTest) -> Option<bool>| nodes.iter().map(f).try_fold(true, |a, v| v.map(|v| a && v));
    let mut s = String::new();
    let _ = write!(
        s,
        "{{\"running\":{},\"done\":{},\"step\":\"{}\",\"via\":\"{}\",\"en_ok\":{},\"en_ms\":{},\"boot_ok\":{},\"recovered\":{},\"recover_ms\":{},\"nodes\":[",
        w.running,
        w.done,
        w.step,
        if w.by_wire { "wires" } else { "link" },
        opt_bool(all(|n| n.en_ok)),
        nodes.iter().map(|n| n.en_ms).max().unwrap_or(0),
        opt_bool(all(|n| n.boot_ok)),
        opt_bool(all(|n| n.recovered)),
        nodes.iter().map(|n| n.recover_ms).max().unwrap_or(0)
    );
    for (k, n) in nodes.iter().enumerate() {
        let _ = write!(
            s,
            "{}{{\"en_ok\":{},\"en_ms\":{},\"boot_ok\":{},\"recovered\":{},\"recover_ms\":{}}}",
            if k > 0 { "," } else { "" },
            opt_bool(n.en_ok),
            n.en_ms,
            opt_bool(n.boot_ok),
            opt_bool(n.recovered),
            n.recover_ms
        );
    }
    let _ = s.push_str("]}");
    RawJson::ok(s)
}

/// Start the Node restart test (over the Node link, or the EN and BOOT wires if fitted). Bench diagnostic, not part of the OFT
/// Protocol: it resets the Nodes three times and stops timing while it runs.
async fn post_wiretest(State(state): State<AppState>, _: Writer) -> RawJson {
    let t = state.0;
    // Check and claim in one step, so the test and a Node firmware write cannot both start.
    let claimed = t.with(|i| {
        if i.wire_test.running || i.node_flash.running {
            return false;
        }
        i.wire_test = crate::state::WireTest { running: true, step: "starting", ..Default::default() };
        true
    });
    if !claimed {
        return RawJson::error(409, "the Node is already being tested or written; wait for that to finish");
    }
    crate::node::WIRE_TEST.signal(());
    get_wiretest(State(state)).await
}

/// Set by `POST /oft/v1/fw/bootsel`; the status task restarts the board once the reply is out.
pub(crate) static BOOTSEL_REQUESTED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// Restart into the ROM's USB bootloader (the `RP2350` drive), as holding BOOT while
/// plugging in does, so a new UF2 can be copied on without reaching the button.
/// Bench convenience, not part of the OFT Protocol: anyone on the network can call it.
async fn fw_bootsel(_: Writer) -> RawJson {
    BOOTSEL_REQUESTED.store(true, core::sync::atomic::Ordering::Relaxed);
    let mut s = String::new();
    let _ = s.push_str("{\"ok\":true,\"msg\":\"restarting into USB boot mode\"}");
    RawJson::ok(s)
}

// ------------------------------------------------------------------ firmware update

fn other_json(o: Other, out: &mut String<BODY>) {
    let _ = match o {
        Other::Empty => write!(out, "null"),
        Other::Confirmed(build) => write!(out, "{{\"build\":{},\"confirmed\":true}}", build),
        Other::Unconfirmed(build) => write!(out, "{{\"build\":{},\"confirmed\":false}}", build),
    };
}

fn opt_u32(v: Option<u32>, out: &mut String<BODY>) {
    let _ = match v {
        Some(v) => write!(out, "{}", v),
        None => write!(out, "null"),
    };
}

/// State of the Timer's own firmware update: which slot runs, its build number, whether it
/// is still on trial, and what the other slot holds. Not part of the OFT Protocol.
///
/// A new firmware on trial counts this request (and `/info`) as proof that it can be
/// reached over Ethernet, and confirms itself shortly after answering one.
async fn get_fw_timer(State(AppState(_, fw)): State<AppState>) -> RawJson {
    fw.note_http();
    let mut s: String<BODY> = String::new();
    let supported = fw.slot.is_ok() && !fw.seal_ignored;
    let _ = write!(s, "{{\"supported\":{},", supported);
    if let Ok(slot) = fw.slot {
        let _ = write!(s, "\"slot\":\"{}\",", slot.name());
    }
    if !supported {
        let reason = match fw.slot {
            Err(e) => e.as_str(),
            Ok(_) => crate::fw::NotReady::SealIgnored.as_str(),
        };
        let _ = write!(s, "\"reason\":\"{}\",", reason);
    }
    let state = if fw.restarting() {
        "restarting"
    } else if fw.on_trial() {
        "trial"
    } else {
        "confirmed"
    };
    let _ = write!(s, "\"state\":\"{}\",\"build\":", state);
    opt_u32(fw.build, &mut s);
    let _ = write!(s, ",\"other\":");
    other_json(fw.other(), &mut s);
    let _ = write!(s, ",\"busy\":{},\"max_bytes\":{},\"confirm_error\":{},", fw.store.try_lock().is_err(), oft_fwimage::layout::MAX_IMAGE_BYTES, fw.buy_error());
    // For diagnosis: what the boot ROM said about this boot, and the flash mapping.
    let _ = write!(s, "\"seal_ignored\":{},\"rom\":{{\"atrans0\":\"{:#010x}\"", fw.seal_ignored, fw.atrans0);
    if let Some(b) = fw.boot {
        let _ = write!(s, ",\"boot_type\":{},\"partition\":{},\"tbyb\":{},\"diagnostic\":\"{:#010x}\",\"update_base\":\"{:#010x}\"", b.boot_type, b.partition, b.tbyb_and_update, b.diagnostic, b.reboot_params[0]);
    }
    let _ = s.push_str("}}");
    RawJson::ok(s)
}

/// `POST /oft/v1/fw/timer`: the body is a firmware image (`oft-timer.bin` from
/// `tools/uf2.py`), `Content-Length` its size. It is written to the slot that is not
/// running, verified there, and the Timer restarts to try it. See `fw.rs`.
/// Bench convenience, not part of the OFT Protocol: anyone on the network can call it.
struct FwUpload;

/// Read the rest of the body and throw it away, so the reply reaches a client that is still
/// sending.
async fn drain<R: picoserve::io::Read>(reader: &mut R) -> Result<(), R::Error> {
    let mut chunk = [0u8; 256];
    while reader.read(&mut chunk).await? != 0 {}
    Ok(())
}

async fn fw_receive<R: picoserve::io::Read>(fw: &'static Fw, request: &mut picoserve::request::Request<'_, R>) -> Result<RawJson, R::Error> {
    use oft_fwimage::receiver::Receiver;
    use picoserve::io::Read as _;
    let len = request.body_connection.content_length();
    if len > oft_fwimage::layout::MAX_IMAGE_BYTES as usize {
        // Not worth reading: the connection closes after the reply.
        return Ok(RawJson::error(413, oft_fwimage::image::ImageError::TooLarge.as_str()));
    }
    let mut reader = request.body_connection.body().reader().with_different_timeout(embassy_time::Duration::from_secs(120));
    let running = match fw.ready() {
        Ok(slot) => slot,
        Err(e) => {
            drain(&mut reader).await?;
            return Ok(RawJson::error(409, e.as_str()));
        }
    };
    let Ok(mut store) = fw.store.try_lock() else {
        drain(&mut reader).await?;
        return Ok(RawJson::error(409, "another firmware upload is in progress"));
    };
    let (mut spare, bufs) = store.spare(running);
    let mut rx = Receiver::new(len as u32);
    let mut failure: Option<&'static str> = None;
    let mut chunk = [0u8; 1024];
    loop {
        // A read error (the client went away, or the 120 s limit) ends the request here: the
        // spare slot is left without its first sector, which the boot ROM treats as empty.
        let n = match reader.read(&mut chunk).await {
            Ok(n) => n,
            Err(e) => {
                if rx.started() {
                    fw.spare_spoiled();
                }
                return Err(e);
            }
        };
        if n == 0 {
            break;
        }
        if failure.is_none() {
            // Erases and programs flash a sector at a time, with interrupts off while it does.
            if let Err(e) = rx.push(bufs, &mut spare, &chunk[..n]) {
                failure = Some(e.as_str());
            }
        }
    }
    let result = match failure {
        Some(m) => Err(m),
        None => rx.finish(bufs, &mut spare).map_err(|e| e.as_str()),
    };
    match result {
        Ok(info) => {
            let target = running.other();
            fw.switch_to(target, &info);
            let mut s: String<BODY> = String::new();
            let _ = write!(
                s,
                "{{\"ok\":true,\"slot\":\"{}\",\"build\":{},\"bytes\":{},\"msg\":\"stored and verified; restarting to try it\"}}",
                target.name(),
                info.build,
                info.len
            );
            Ok(RawJson::ok(s))
        }
        Err(m) => {
            if rx.started() {
                fw.spare_spoiled();
            }
            Ok(RawJson::error(422, m))
        }
    }
}

/// `POST /oft/v1/fw/node?offset=65536&baud=115200`: firmware for the Node, written to it over
/// the Node link's UART through the C5's ROM bootloader (reached over the Node link; the EN
/// and BOOT wires are only a fallback). The body
/// is the raw file for that offset: `firmware.bin` at 0x10000 (the default),
/// `bootloader.bin` at 0x2000, `partitions.bin` at 0x8000. It is stored in the Timer's flash
/// first and checked; the transfer then runs on its own, with progress at `GET`.
/// Bench convenience, not part of the OFT Protocol: anyone on the network can call it.
struct NodeUpload;

fn node_flash_json(t: &Timer) -> RawJson {
    let (f, connected) = t.with(|i| (i.node_flash, (0..i.node_count()).all(|n| i.nodes[n].connected)));
    let mut s: String<BODY> = String::new();
    let _ = write!(
        s,
        "{{\"supported\":true,\"running\":{},\"done\":{},\"ok\":{},\"step\":\"{}\",\"error\":\"{}\",\"offset\":{},\"bytes\":{},\"written\":{},\"baud\":{},\"seconds\":{},\"node\":{},\"nodes\":{},\"nodes_done\":{},\"node_connected\":{},\"in_bootloader\":{},\"max_bytes\":{}",
        f.running,
        f.done,
        f.ok,
        f.step,
        f.error,
        f.offset,
        f.total,
        f.written,
        f.baud,
        f.seconds,
        f.node,
        f.nodes,
        f.nodes_done,
        connected,
        f.in_bootloader,
        oft_fwimage::layout::NODE_STORE_BYTES
    );
    let _ = s.push_str(",\"diag\":\"");
    crate::node::FLASH_DIAG.lock(|d| {
        let _ = s.push_str(d.borrow().as_str());
    });
    match f.returned {
        Some(r) => {
            let _ = write!(s, "\",\"returned\":{}}}", r);
        }
        None => {
            let _ = s.push_str("\",\"returned\":null}");
        }
    }
    RawJson::ok(s)
}

async fn get_fw_node(State(AppState(t, _)): State<AppState>) -> RawJson {
    node_flash_json(t)
}

/// `offset`, `baud` and `node` from the query string, with their defaults. `node` is the
/// Node to write, from 0; without it every wired Node is written, one after the other.
fn node_query(query: Option<&str>, nodes: usize) -> Result<(u32, u32, Option<u8>), &'static str> {
    let (mut offset, mut baud, mut node) = (oft_fwimage::esprom::C5_APP_OFFSET, 115_200u32, None);
    for pair in query.unwrap_or("").split('&').filter(|p| !p.is_empty()) {
        let (k, v) = pair.split_once('=').unwrap_or((pair, ""));
        let n = match v.strip_prefix("0x") {
            Some(h) => u32::from_str_radix(h, 16),
            None => v.parse::<u32>(),
        };
        match (k, n) {
            ("offset", Ok(n)) => offset = n,
            ("baud", Ok(n)) => baud = n,
            ("node", Ok(n)) if (n as usize) < nodes => node = Some(n as u8),
            ("node", Ok(_)) => return Err("there is no Node with that number"),
            ("offset" | "baud" | "node", Err(_)) => return Err("offset, baud and node must be numbers"),
            _ => {}
        }
    }
    if ![115_200, 230_400, 460_800, 921_600].contains(&baud) {
        return Err("baud must be 115200, 230400, 460800 or 921600");
    }
    Ok((offset, baud, node))
}

async fn node_receive<R: picoserve::io::Read>(t: &'static Timer, fw: &'static Fw, request: &mut picoserve::request::Request<'_, R>) -> Result<RawJson, R::Error> {
    use md5::Digest as _;
    use oft_fwimage::esprom::check_image;
    use oft_fwimage::layout::{NODE_STORE_BYTES, SECTOR_BYTES};
    use picoserve::io::Read as _;
    const SECTOR: usize = SECTOR_BYTES as usize;
    let len = request.body_connection.content_length();
    let query = node_query(request.parts.query().map(|q| q.0), t.with(|i| i.node_count()));
    let mut reader = request.body_connection.body().reader().with_different_timeout(embassy_time::Duration::from_secs(60));
    let refuse = |code: u16, m: &'static str| RawJson::error(code, m);
    let (offset, baud, node) = match query {
        Ok(q) => q,
        Err(m) => {
            drain(&mut reader).await?;
            return Ok(refuse(400, m));
        }
    };
    if len > NODE_STORE_BYTES as usize {
        // Not worth reading: the connection closes after the reply.
        return Ok(refuse(413, "the file is too large for the Timer to hold"));
    }
    if t.with(|i| i.node_flash.running || i.wire_test.running) {
        drain(&mut reader).await?;
        return Ok(refuse(409, "the Node is already being written or tested; wait for that to finish"));
    }
    if fw.restarting() {
        drain(&mut reader).await?;
        return Ok(refuse(409, "the Timer is about to restart"));
    }
    let Ok(mut store) = fw.store.try_lock() else {
        drain(&mut reader).await?;
        return Ok(refuse(409, "a firmware upload to the Timer is in progress"));
    };
    let mut md5 = md5::Md5::new();
    let (mut got, mut fill, mut sector) = (0usize, 0usize, 0u32);
    let mut checked = false;
    let mut failure: Option<&'static str> = None;
    let mut chunk = [0u8; 512];
    loop {
        let n = reader.read(&mut chunk).await?;
        if n == 0 {
            break;
        }
        if failure.is_some() {
            continue;
        }
        md5.update(&chunk[..n]);
        got += n;
        let mut rest = &chunk[..n];
        while !rest.is_empty() && failure.is_none() {
            let take = rest.len().min(SECTOR - fill);
            store.sector_buf()[fill..fill + take].copy_from_slice(&rest[..take]);
            fill += take;
            rest = &rest[take..];
            // Look at the start of the file before anything is written anywhere.
            if !checked && (fill >= 16 || got == len) {
                checked = true;
                if let Err(e) = check_image(offset, len as u32, &store.sector_buf()[..fill], NODE_STORE_BYTES) {
                    failure = Some(e.as_str());
                    break;
                }
            }
            if fill == SECTOR {
                if let Err(e) = store.write_node_sector(sector) {
                    failure = Some(e);
                }
                sector += 1;
                fill = 0;
            }
        }
    }
    if failure.is_none() && got != len {
        failure = Some("the upload ended early");
    }
    if failure.is_none() && !checked {
        failure = Some(oft_fwimage::esprom::ImageError::Empty.as_str());
    }
    if failure.is_none() && fill > 0 {
        store.sector_buf()[fill..].fill(0xff);
        if let Err(e) = store.write_node_sector(sector) {
            failure = Some(e);
        }
    }
    drop(store);
    if let Some(m) = failure {
        return Ok(refuse(400, m));
    }
    let mut digest = [0u8; 16];
    digest.copy_from_slice(&md5.finalize());
    // Claim the flasher. Two uploads cannot both get here: the store lock was held until now
    // and this check-and-set does not yield.
    let claimed = t.with(|i| {
        if i.node_flash.running || i.wire_test.running {
            return false;
        }
        i.node_flash = crate::state::NodeFlash { running: true, step: "queued", offset, total: len as u32, baud, ..Default::default() };
        true
    });
    if !claimed {
        return Ok(refuse(409, "the Node is already being written or tested; wait for that to finish"));
    }
    crate::node::NODE_FLASH.signal(crate::node::NodeJob { offset, len: len as u32, md5: digest, baud, node });
    let mut reply = node_flash_json(t);
    reply.0 = StatusCode::ACCEPTED;
    Ok(reply)
}

impl picoserve::routing::RequestHandlerService<AppState> for NodeUpload {
    async fn call_request_handler_service<R: picoserve::io::Read, W: picoserve::response::ResponseWriter<Error = R::Error>>(
        &self,
        state: &AppState,
        (): (),
        mut request: picoserve::request::Request<'_, R>,
        response_writer: W,
    ) -> Result<picoserve::ResponseSent, W::Error> {
        let reply = match may_write(state.0, &request.parts) {
            Ok(()) => node_receive(state.0, state.1, &mut request).await?,
            Err(refused) => refused,
        };
        let connection = request.body_connection.finalize().await?;
        reply.write_to(connection, response_writer).await
    }
}

impl picoserve::routing::RequestHandlerService<AppState> for FwUpload {
    async fn call_request_handler_service<R: picoserve::io::Read, W: picoserve::response::ResponseWriter<Error = R::Error>>(
        &self,
        state: &AppState,
        (): (),
        mut request: picoserve::request::Request<'_, R>,
        response_writer: W,
    ) -> Result<picoserve::ResponseSent, W::Error> {
        let reply = match may_write(state.0, &request.parts) {
            Ok(()) => fw_receive(state.1, &mut request).await?,
            Err(refused) => refused,
        };
        let connection = request.body_connection.finalize().await?;
        reply.write_to(connection, response_writer).await
    }
}

async fn stream(State(AppState(t, fw)): State<AppState>, upgrade: ws::WebSocketUpgrade) -> impl IntoResponse {
    upgrade.on_upgrade(Stream(t, fw))
}

// ------------------------------------------------------------------ server

static CONFIG: picoserve::Config = picoserve::Config::const_default().keep_connection_alive();

/// One HTTP connection at a time per task; `WEB_TASK_POOL_SIZE` tasks run.
#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
pub async fn web_task(task_id: usize, stack: Stack<'static>, timer: &'static Timer, fw: &'static Fw) -> ! {
    let app = picoserve::Router::new()
        // The built-in diagnostics page: the same file every Timer host serves.
        .route("/", get_service(picoserve::response::File::html(include_str!("../../oft/web/timer.html"))))
        .route("/oft/v1/info", get(info))
        .route("/oft/v1/status", get(status))
        .route("/oft/v1/time", get(time))
        .route("/oft/v1/config", get(get_config).put(put_config))
        .route("/oft/v1/config/save", get(get_config_save).post(post_config_save).delete(delete_config_save))
        .route("/oft/v1/network", get(get_network).put(put_network))
        .route("/oft/v1/pins", get(get_pins).put(put_pins))
        .route("/oft/v1/control", get(get_control).post(post_control))
        .route("/oft/v1/restart", post(post_restart))
        .route("/oft/v1/crossings", get(crossings))
        .route("/oft/v1/samples", get(samples))
        .route("/oft/v1/stream", get(stream))
        .route("/oft/v1/fw/bootsel", post(fw_bootsel))
        .route("/oft/v1/fw/timer", get(get_fw_timer).post_service(FwUpload))
        .route("/oft/v1/fw/node", get(get_fw_node).post_service(NodeUpload))
        .route("/oft/v1/node/wiretest", get(get_wiretest).post(post_wiretest))
        .with_state(AppState(timer, fw));
    let mut tcp_rx = [0u8; 1024];
    let mut tcp_tx = [0u8; 2048];
    let mut http = [0u8; 2048];
    picoserve::Server::new(&app, &CONFIG, &mut http).listen_and_serve(task_id, stack, OFT_PORT, &mut tcp_rx, &mut tcp_tx).await.into_never()
}

/// Read the RP2350's die temperature every two seconds (average of 8 conversions). The
/// sensor is uncalibrated: good to a few degrees, which is enough to see a case run hot.
#[embassy_executor::task]
pub async fn temp_task(timer: &'static Timer, mut adc: embassy_rp::adc::Adc<'static, embassy_rp::adc::Async>, mut ch: embassy_rp::adc::Channel<'static>) {
    loop {
        let mut sum = 0u32;
        let mut n = 0u32;
        for _ in 0..8 {
            if let Ok(v) = adc.read(&mut ch).await {
                sum += v as u32;
                n += 1;
            }
        }
        if n > 0 {
            // 12-bit reading against 3.3 V; 0.706 V at 27 °C, -1.721 mV per °C (datasheet).
            let volts = (sum as f32 / n as f32) * 3.3 / 4096.0;
            let dc = (27.0 - (volts - 0.706) / 0.001721) * 10.0;
            timer.with(|i| i.timer_temp_dc = Some(dc as i16));
        }
        embassy_time::Timer::after_secs(2).await;
    }
}

/// Keep `net_now` in step with the address the network stack holds.
#[embassy_executor::task]
pub async fn net_watch_task(timer: &'static Timer, stack: Stack<'static>) {
    loop {
        let now = stack.config_v4().map(|c| NetNow { address: c.address.address().octets(), prefix: c.address.prefix_len(), gateway: c.gateway.map(|g| g.octets()) });
        timer.with(|i| i.net_now = now);
        embassy_time::Timer::after_secs(1).await;
    }
}

/// Publish a `Status` event once a second for WebSocket subscribers.
#[embassy_executor::task]
pub async fn status_tick_task(timer: &'static Timer) {
    let publisher = timer.events.immediate_publisher();
    loop {
        embassy_time::Timer::after_secs(1).await;
        if BOOTSEL_REQUESTED.load(core::sync::atomic::Ordering::Relaxed) {
            crate::crash::stopping();
            embassy_rp::rom_data::reset_to_usb_boot(0, 0);
        }
        if timer.with(|i| i.control.expire(crate::state::now_us())) {
            publisher.publish_immediate(Event::Control);
        }
        publisher.publish_immediate(Event::Status);
    }
}
