//! The Node links: decode each Node's frames into the hosts, send plans and pings, and pulse
//! SYNC. Also restarts the Nodes and writes their firmware.
//!
//! Each Node has its own serial link and its own pair of tasks. SYNC is one wire shared by
//! every Node.
//!
//! A Node is restarted, and sent to its ROM bootloader, with `Reset` and `Dfu` messages on
//! its own link, so no wire beyond the link is needed and the other Nodes keep running.
//! EN and BOOT wires (`Wires`) are optional: one of each to every Node, so they restart
//! all Nodes at once. Where they are fitted they are the fallback for a Node whose
//! firmware does not answer.

use crate::board::MAX_NODES;
use crate::state::{Event, NodeFlash, NodeWireTest, Timer, Trace, WireTest};
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio_programs::uart::PioUartTx;
use embassy_rp::uart::{BufferedUartRx, BufferedUartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer as Delay};
use embedded_io_async::{Read, Write};
use oft_core::Event as HostEvent;
use oft_fwimage::esprom;
use oft_nodelink::msg::Message;
use oft_nodelink::{encode_frame, FrameDecoder, MAX_FRAME};
use portable_atomic::{AtomicU32, AtomicU8, Ordering};

/// The receiving half of a Node link.
pub enum LinkRx {
    /// A hardware UART (Nodes 1 and 2).
    Uart(BufferedUartRx),
    /// A PIO state machine filling a ring by DMA (Nodes 3 and 4).
    Ring(crate::piolink::RingRx),
}

impl LinkRx {
    /// Wait for bytes. `Err` is a receive error the UART reported; reading goes on.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        match self {
            LinkRx::Uart(rx) => rx.read(buf).await.map_err(|_| ()),
            LinkRx::Ring(rx) => Ok(rx.read(buf).await),
        }
    }
}

/// The sending half of a Node link.
pub enum LinkTx {
    Uart(BufferedUartTx),
    Pio0(PioUartTx<'static, PIO0, 0>),
    Pio2(PioUartTx<'static, PIO0, 2>),
}

impl LinkTx {
    async fn write_all(&mut self, buf: &[u8]) {
        match self {
            LinkTx::Uart(tx) => {
                let _ = tx.write_all(buf).await;
            }
            LinkTx::Pio0(tx) => {
                let _ = tx.write_all(buf).await;
            }
            LinkTx::Pio2(tx) => {
                let _ = tx.write_all(buf).await;
            }
        }
    }
}

/// Who each link is talking to.
/// `LINK`: the Node firmware, over the Node link.
/// `ROM_PROBE` (wiring test): the TX task sends ROM sync packets in place of pings, and the
/// RX task counts the ROM's answers.
/// `ROM_LOADER` (flashing): the RX task passes every byte to `ROM_RX`, and the TX task sends
/// what arrives in `ROM_TX`. One link at a time.
/// `QUIET`: nothing is sent and what arrives is thrown away (a Node sitting in its
/// bootloader while another is written).
static ROM_MODE: [AtomicU8; MAX_NODES] = [const { AtomicU8::new(LINK) }; MAX_NODES];
const LINK: u8 = 0;
const ROM_PROBE: u8 = 1;
const ROM_LOADER: u8 = 2;
const QUIET: u8 = 3;
/// Bytes from the ROM bootloader to the flasher, and requests from the flasher to the wire.
static ROM_RX: Pipe<CriticalSectionRawMutex, 512> = Pipe::new();
static ROM_TX: Pipe<CriticalSectionRawMutex, { esprom::MAX_FRAME }> = Pipe::new();

/// A message for Node `n`'s TX task to send over the Node link (`CMD_*`).
static NODE_CMD: [AtomicU8; MAX_NODES] = [const { AtomicU8::new(CMD_NONE) }; MAX_NODES];
const CMD_NONE: u8 = 0;
/// Restart into the Node firmware.
const CMD_RESET: u8 = 1;
/// Restart into the ROM bootloader.
const CMD_DFU: u8 = 2;

/// The baud rate at which Node `n`'s ROM bootloader last answered, while a firmware write is
/// under way.
static LOADER_BAUD: [AtomicU32; MAX_NODES] = [const { AtomicU32::new(0) }; MAX_NODES];
/// Non-zero: a write to Node `n` failed and the Node was left sitting in its ROM bootloader
/// at this baud rate, watchdogs off, so the next attempt can carry straight on. Restarting
/// it instead would start firmware that is now partly written, and the Timer could not get
/// it back into its bootloader without a hand on the board. Cleared when the Node is heard
/// on its link again, and at the next attempt.
static PARKED: [AtomicU32; MAX_NODES] = [const { AtomicU32::new(0) }; MAX_NODES];

/// Nodes left in their bootloaders by a failed write (`PARKED`).
pub fn parked(n: usize) -> bool {
    PARKED[n].load(Ordering::Relaxed) != 0
}

/// The optional reset and boot-strap wires, one of each to every Node.
pub struct Wires {
    /// C5 EN, active low.
    pub en: Output<'static>,
    /// C5 BOOT strap: low through a reset starts the ROM bootloader.
    pub boot: Output<'static>,
}

fn set_modes(mode: u8) {
    for m in &ROM_MODE {
        m.store(mode, Ordering::Relaxed);
    }
}

/// A Node firmware file sitting in the Timer's scratch area (`fw::node_store`), to be
/// written to a Node's flash.
#[derive(Clone, Copy)]
pub struct NodeJob {
    /// Offset in the Node's flash.
    pub offset: u32,
    pub len: u32,
    pub md5: [u8; 16],
    /// Baud rate to talk to the ROM bootloader at (it measures the rate from the sync).
    pub baud: u32,
    /// The Node to write, from 0; `None` = every wired Node, one after the other.
    pub node: Option<u8>,
}
/// Raised by `POST /oft/v1/fw/node` once the file is stored.
pub static NODE_FLASH: Signal<CriticalSectionRawMutex, NodeJob> = Signal::new();
/// Size the Node's flash is described as to its bootloader. Every part written lies below it.
const NODE_FLASH_BYTES: u32 = 4 * 1024 * 1024;
/// ROM sync answers seen, per Node.
static ROM_SYNC_SEEN: [AtomicU32; MAX_NODES] = [const { AtomicU32::new(0) }; MAX_NODES];
/// Times each Node was seen to restart: a new Hello, or its clock running backwards.
static NODE_REBOOTS: [AtomicU32; MAX_NODES] = [const { AtomicU32::new(0) }; MAX_NODES];
/// Raised by `POST /oft/v1/node/wiretest`.
pub static WIRE_TEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The ESP ROM loader's SYNC command as one SLIP frame: direction 0, command 0x08, 36 data
/// bytes (07 07 12 20, then 32 x 55). The ROM measures the baud rate from it and answers
/// with a frame that starts C0 01 08.
const ROM_SYNC: [u8; 46] = {
    let mut p = [0x55u8; 46];
    p[0] = 0xC0;
    p[1] = 0x00;
    p[2] = 0x08;
    p[3] = 0x24;
    p[4] = 0x00;
    p[5] = 0;
    p[6] = 0;
    p[7] = 0;
    p[8] = 0;
    p[9] = 0x07;
    p[10] = 0x07;
    p[11] = 0x12;
    p[12] = 0x20;
    p[45] = 0xC0;
    p
};
/// Baud rate the ROM loader is addressed at (its conventional rate; it autobauds on SYNC).
const ROM_BAUD: u32 = 115_200;

/// Change a hardware UART's baud rate after the driver has been split into halves (the
/// driver only offers this on the unsplit port). Same arithmetic as embassy-rp's
/// `set_baudrate`.
fn set_uart_baud(r: embassy_rp::pac::uart::Uart, baud: u32) {
    let div = (8 * embassy_rp::clocks::clk_peri_freq()) / baud;
    let (mut ibrd, mut fbrd) = (div >> 7, ((div & 0x7f) + 1) / 2);
    if ibrd == 0 {
        ibrd = 1;
        fbrd = 0;
    } else if ibrd >= 65535 {
        ibrd = 65535;
        fbrd = 0;
    }
    r.uartibrd().write_value(embassy_rp::pac::uart::regs::Uartibrd(ibrd));
    r.uartfbrd().write_value(embassy_rp::pac::uart::regs::Uartfbrd(fbrd));
    // The divisors take effect on a write to LCR_H.
    r.uartlcr_h().modify(|_| {});
}

/// Change the baud rate of Node `n`'s link.
fn set_baud(n: usize, baud: u32) {
    match n {
        0 => set_uart_baud(embassy_rp::pac::UART1, baud),
        1 => set_uart_baud(embassy_rp::pac::UART0, baud),
        _ => crate::piolink::set_baud(n, baud),
    }
}

async fn send(tx: &mut LinkTx, m: &Message) {
    let mut buf = [0u8; MAX_FRAME];
    if let Ok(n) = encode_frame(m, &mut buf) {
        tx.write_all(&buf[..n]).await;
    }
}

/// Read Node `n`'s bytes forever.
#[embassy_executor::task(pool_size = MAX_NODES)]
pub async fn node_rx_task(timer: &'static Timer, n: usize, mut rx: LinkRx) {
    let mut dec = FrameDecoder::new();
    let mut buf = [0u8; 256];
    // Not `publisher()`: the channel allows two of those, and there is one of these tasks per Node.
    // The first start with four Nodes stopped here (2026-11-27).
    let publisher = timer.events.immediate_publisher();
    let mut rom_match = 0u8;
    let mut last_pong_t_node = 0u64;
    loop {
        let len = match rx.read(&mut buf).await {
            Ok(0) => {
                Delay::after_millis(5).await;
                continue;
            }
            Err(()) => {
                // Overrun, framing, noise or break on the wire. Count it and keep reading:
                // the frame decoder resynchronises at the next frame boundary.
                timer.with(|i| i.nodes[n].uart_errors = i.nodes[n].uart_errors.wrapping_add(1));
                continue;
            }
            Ok(len) => len,
        };
        let t = crate::state::now_us();
        let mode = ROM_MODE[n].load(Ordering::Relaxed);
        if mode == QUIET {
            continue;
        }
        if mode == ROM_LOADER {
            // The flasher reads these. `try_write` takes only what fits before the end of
            // the pipe's ring, so it is called until everything is in: taking its first
            // answer as the whole job dropped the tail of a reply each time the 512-byte
            // ring wrapped, about every 35 replies. That was the lost answer "every few
            // seconds" during a Node write (found 2026-11-27). If the pipe is really full
            // the rest is dropped, and that request times out and is retried.
            let mut rest = &buf[..len];
            while !rest.is_empty() {
                match ROM_RX.try_write(rest) {
                    Ok(k) if k > 0 => rest = &rest[k..],
                    _ => break,
                }
            }
            continue;
        }
        if mode == ROM_PROBE {
            // Talking to the ROM loader at another baud rate: these are not Node-link frames.
            for &b in &buf[..len] {
                rom_match = match (rom_match, b) {
                    (0, 0xC0) => 1,
                    (1, 0x01) => 2,
                    (2, 0x08) => {
                        ROM_SYNC_SEEN[n].fetch_add(1, Ordering::Relaxed);
                        0
                    }
                    (_, 0xC0) => 1,
                    _ => 0,
                };
            }
            continue;
        }
        for &b in &buf[..len] {
            let msg = match dec.push(b) {
                Some(Ok(m)) => m,
                Some(Err(_)) => {
                    timer.with(|i| i.nodes[n].dropped_frames = i.nodes[n].dropped_frames.wrapping_add(1));
                    continue;
                }
                None => continue,
            };
            let mut restarted = false;
            match &msg {
                Message::Hello(_) => {
                    // Its firmware is running: it is no longer waiting in its bootloader.
                    PARKED[n].store(0, Ordering::Relaxed);
                    NODE_REBOOTS[n].fetch_add(1, Ordering::Relaxed);
                }
                Message::Pong(p) => {
                    if p.t_node < last_pong_t_node {
                        NODE_REBOOTS[n].fetch_add(1, Ordering::Relaxed);
                        // The Node restarted and its Hello did not arrive (or has not yet):
                        // it has no plan, so send ours again.
                        restarted = true;
                    }
                    last_pong_t_node = p.t_node;
                }
                _ => {}
            }
            // Everything below is short and non-blocking; events are queued for the WS tasks.
            let mut out: heapless::Vec<Event, 8> = heapless::Vec::new();
            timer.with(|i| {
                i.node_heard(n, t);
                if restarted {
                    i.nodes[n].plan_dirty = true;
                    // Its Hello is about the run that just ended.
                    i.nodes[n].hello_fresh = false;
                }
                if matches!(msg, Message::Hello(_)) {
                    i.nodes[n].hello_fresh = true;
                }
                if let Message::Samples(s) = &msg {
                    // Only from the Node the seat is on (as `Hosts::feed` does for detection).
                    if i.hosts.seat_node(s.seat) == Some(n as u8) {
                        let clock = *i.hosts.clock(n);
                        let t0 = clock.to_timer(s.t0);
                        if let Some(slot) = i.traces.get_mut(s.seat as usize) {
                            let tr = slot.get_or_insert_with(Trace::new);
                            if s.dt_us > 0 {
                                tr.rate_hz = 1_000_000 / s.dt_us as u32;
                            }
                            for (k, &p) in s.power().iter().enumerate() {
                                tr.push(t0 + k as u64 * s.dt_us as u64, p);
                            }
                        }
                        // Fan out in 32-sample pieces so one event stays small.
                        for (k, chunk) in s.power().chunks(32).enumerate() {
                            let mut p = [0i16; 32];
                            p[..chunk.len()].copy_from_slice(chunk);
                            let _ = out.push(Event::Samples {
                                seat: s.seat,
                                t0: t0 + (k * 32) as u64 * s.dt_us as u64,
                                dt_us: s.dt_us,
                                n: chunk.len() as u8,
                                p,
                                nf: s.noise_floor,
                            });
                        }
                    }
                }
                let before = i.seat_nodes();
                let crate::state::Inner { hosts, crossings, next_seq, nodes, .. } = i;
                let changed = hosts.feed(n, &msg, t, |e| match e {
                    // `feed` says whose plan to send.
                    HostEvent::Hello(_) => {}
                    HostEvent::Crossing(c) => {
                        let seq = *next_seq;
                        *next_seq += 1;
                        if crossings.is_full() {
                            crossings.pop_front();
                        }
                        let _ = crossings.push_back((seq, c));
                        let _ = out.push(Event::Crossing { seq, c });
                    }
                    HostEvent::Status(s) => {
                        // i16::MIN = the Node has no reading; 0 = Node firmware before it had a sensor.
                        nodes[n].temp_dc = (s.temp_dc != i16::MIN && s.temp_dc != 0).then_some(s.temp_dc);
                        nodes[n].status = Some(s);
                    }
                    HostEvent::Synced => {}
                });
                i.rescheduled(before, changed);
            });
            for e in out {
                publisher.publish_immediate(e);
            }
        }
    }
}

/// For Node `n`: send its plan when it changes, ping once a second, and watch the link for
/// silence.
#[embassy_executor::task(pool_size = MAX_NODES)]
pub async fn node_tx_task(timer: &'static Timer, n: usize, mut tx: LinkTx) {
    let mut next_ping = Instant::now();
    let mut unnamed_since: Option<Instant> = None;
    loop {
        match ROM_MODE[n].load(Ordering::Relaxed) {
            ROM_PROBE => {
                tx.write_all(&ROM_SYNC).await;
                Delay::after_millis(100).await;
                continue;
            }
            ROM_LOADER => {
                // Send whatever the flasher queued; look at the mode again at least every 20 ms.
                let mut chunk = [0u8; 256];
                if let Ok(len) = embassy_time::with_timeout(Duration::from_millis(20), ROM_TX.read(&mut chunk)).await {
                    tx.write_all(&chunk[..len]).await;
                }
                continue;
            }
            QUIET => {
                Delay::after_millis(20).await;
                continue;
            }
            _ => {}
        }
        match NODE_CMD[n].swap(CMD_NONE, Ordering::Relaxed) {
            CMD_RESET => send(&mut tx, &Message::Reset).await,
            CMD_DFU => send(&mut tx, &Message::Dfu).await,
            _ => {}
        }
        // A plan goes only to a Node that has said Hello since it started. A Node repeats
        // its Hello until a plan arrives, and its first one is often lost in its start-up
        // text; a plan sent on the strength of any other frame stopped the Hellos for good,
        // and the Timer was left with a Node it could not name or give a seat to, or with
        // the id and abilities of the firmware it ran before an update (both seen
        // 2026-11-27).
        let (plan, unnamed) = timer.with(|i| {
            let known = i.nodes[n].hello_fresh;
            let plan = if known && i.nodes[n].plan_dirty {
                i.nodes[n].plan_dirty = false;
                Some(i.hosts.plan(n))
            } else {
                None
            };
            (plan, i.nodes[n].connected && !known)
        });
        if let Some(p) = plan {
            send(&mut tx, &Message::Plan(p)).await;
        }
        // A Node that is talking but has not said Hello within a few seconds already has a
        // plan, from before this Timer started (the Timer no longer resets its Nodes with
        // a wire when it starts). Restart it over the link: it then says Hello.
        match (unnamed, unnamed_since) {
            (false, _) => unnamed_since = None,
            (true, None) => unnamed_since = Some(Instant::now()),
            // Compared forwards: `t0` is put in the future below, and `elapsed` panics on that.
            (true, Some(t0)) if Instant::now() >= t0 + Duration::from_secs(4) => {
                send(&mut tx, &Message::Reset).await;
                // Not again until it has had time to restart and be heard.
                unnamed_since = Some(Instant::now() + Duration::from_secs(8));
            }
            _ => {}
        }
        if Instant::now() >= next_ping {
            let t = crate::state::now_us();
            let ping = timer.with(|i| i.hosts.make_ping(n, t));
            send(&mut tx, &ping).await;
            next_ping = Instant::now() + Duration::from_secs(1);
            // Node silence check: after 3 s the other Nodes take its seats.
            timer.with(|i| {
                if i.nodes[n].last_rx_us.is_some_and(|last| t.saturating_sub(last) > 3_000_000) {
                    i.node_lost(n);
                }
            });
        }
        Delay::after_millis(20).await;
    }
}

/// Pulse SYNC at 20 Hz. The pulse time is recorded in the hosts *before* the edge so each
/// Node's report always finds its pulse.
#[embassy_executor::task]
pub async fn sync_task(timer: &'static Timer, mut sync: Output<'static>) {
    let mut next = Instant::now();
    loop {
        next += Duration::from_millis(50);
        Delay::at(next).await;
        let t = crate::state::now_us();
        timer.with(|i| i.hosts.sync_pulsed(t));
        sync.set_high();
        Delay::after_micros(200).await;
        sync.set_low();
    }
}

/// Hold the C5s in reset, then release them, into the firmware or (`download_mode`) the
/// ROM bootloader. Every Node is on these two wires.
pub async fn reset_nodes(w: &mut Wires, download_mode: bool) {
    if download_mode {
        w.boot.set_low();
    } else {
        w.boot.set_high();
    }
    w.en.set_low();
    Delay::after_millis(20).await;
    w.en.set_high();
    Delay::after_millis(50).await;
    w.boot.set_high();
}

/// How often each Node has restarted so far.
fn reboots() -> [u32; MAX_NODES] {
    core::array::from_fn(|n| NODE_REBOOTS[n].load(Ordering::Relaxed))
}

/// Wait for evidence that each of Nodes `first..last` restarted after `base` was read.
/// Returns each one's wait in ms, `None` for those not back within `limit`.
async fn wait_reboots(first: usize, last: usize, base: [u32; MAX_NODES], limit: Duration) -> [Option<u32>; MAX_NODES] {
    let t0 = Instant::now();
    let mut back = [None; MAX_NODES];
    loop {
        for n in first..last {
            if back[n].is_none() && NODE_REBOOTS[n].load(Ordering::Relaxed) != base[n] {
                back[n] = Some(t0.elapsed().as_millis() as u32);
            }
        }
        if back[first..last].iter().all(|b| b.is_some()) || t0.elapsed() >= limit {
            return back;
        }
        Delay::after_millis(100).await;
    }
}

/// What happened around a failed Node firmware write, for `GET /oft/v1/fw/node`: when, which
/// request, and the last bytes the Node sent (a restarted chip prints its reset reason).
pub static FLASH_DIAG: embassy_sync::blocking_mutex::Mutex<CriticalSectionRawMutex, core::cell::RefCell<heapless::String<720>>> =
    embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(heapless::String::new()));

fn diag(args: core::fmt::Arguments) {
    use core::fmt::Write as _;
    FLASH_DIAG.lock(|d| {
        let _ = d.borrow_mut().write_fmt(args);
    });
}

/// What arrived while sync calls went unanswered: the byte count, and the bytes of the last
/// call that received any.
static SYNC_SEEN: embassy_sync::blocking_mutex::Mutex<CriticalSectionRawMutex, core::cell::RefCell<(u32, heapless::Vec<u8, 120>)>> =
    embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new((0, heapless::Vec::new())));

/// Bytes for the diagnosis: as text if they read as text (a restarted chip prints why), else hex.
fn diag_bytes(bytes: &[u8]) {
    let len = bytes.len();
    let text = len > 24 && bytes.iter().filter(|b| matches!(b, 32..=126 | b'\r' | b'\n')).count() * 10 >= len * 8;
    for &b in bytes.iter().take(if text { 120 } else { 24 }) {
        match (text, b) {
            (true, b'\\' | b'"') => diag(format_args!("'")),
            (true, 32..=126) => diag(format_args!("{}", b as char)),
            (true, b'\n') => diag(format_args!("|")),
            (true, _) => {}
            (false, b) => diag(format_args!("{:02x}", b)),
        }
    }
}

struct RomReply {
    value: u32,
    data: heapless::Vec<u8, { esprom::MD5_HEX }>,
}

/// One request to the ROM bootloader and its reply. Replies to other requests still in
/// flight are skipped.
async fn rom_cmd(req: &esprom::Request, timeout: Duration, frame: &mut [u8; esprom::MAX_FRAME]) -> Result<RomReply, &'static str> {
    let mut junk = [0u8; 64];
    while ROM_RX.try_read(&mut junk).is_ok() {}
    let n = req.frame(frame);
    ROM_TX.write_all(&frame[..n]).await;
    let deadline = Instant::now() + timeout;
    let mut de = esprom::Deframer::<64>::new();
    let mut buf = [0u8; 64];
    // The last bytes received, kept for the diagnosis if no reply comes.
    let mut seen: heapless::Deque<u8, 120> = heapless::Deque::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        let Ok(n) = embassy_time::with_timeout(left, ROM_RX.read(&mut buf)).await else {
            if req.op() != esprom::OP_SYNC {
                diag(format_args!("[op {:#04x}: no reply in {} ms; got {} bytes: ", req.op(), timeout.as_millis(), seen.len()));
                let all: heapless::Vec<u8, 120> = seen.iter().copied().collect();
                diag_bytes(&all);
                diag(format_args!("] "));
            } else {
                // Kept for `rom_sync`, which reports only if every call goes unanswered.
                SYNC_SEEN.lock(|s| {
                    let mut s = s.borrow_mut();
                    s.0 = s.0.wrapping_add(seen.len() as u32);
                    if !seen.is_empty() {
                        s.1.clear();
                        s.1.extend(seen.iter().copied());
                    }
                });
            }
            return Err("the Node's bootloader did not answer");
        };
        for &b in &buf[..n] {
            if seen.is_full() {
                seen.pop_front();
            }
            let _ = seen.push_back(b);
            if !de.push(b) {
                continue;
            }
            let Some(reply) = esprom::parse_reply(de.frame()) else { continue };
            if reply.op != req.op() {
                continue;
            }
            if reply.status != 0 {
                diag(format_args!("[op {:#04x}: status {} error {:#04x}] ", req.op(), reply.status, reply.error));
                return Err(esprom::error_text(reply.error));
            }
            let data = heapless::Vec::from_slice(reply.data).map_err(|_| "the Node's bootloader sent an oversized reply")?;
            return Ok(RomReply { value: reply.value, data });
        }
    }
}

/// The buffer ROM requests are framed in. It is 2 KiB: kept out of the task's future, made
/// once, and handed back with `frame_done` at the end of each use.
static FRAME: static_cell::StaticCell<[u8; esprom::MAX_FRAME]> = static_cell::StaticCell::new();
static FRAME_REF: Signal<CriticalSectionRawMutex, &'static mut [u8; esprom::MAX_FRAME]> = Signal::new();

fn frame_take() -> Option<&'static mut [u8; esprom::MAX_FRAME]> {
    FRAME_REF.try_take().or_else(|| FRAME.try_init([0; esprom::MAX_FRAME]))
}

fn frame_done(frame: &'static mut [u8; esprom::MAX_FRAME]) {
    FRAME_REF.signal(frame);
}

/// Call the ROM bootloader on the link in `ROM_LOADER` mode until it answers or `limit` runs out.
async fn rom_sync(limit: Duration, frame: &mut [u8; esprom::MAX_FRAME]) -> bool {
    let t0 = Instant::now();
    SYNC_SEEN.lock(|s| *s.borrow_mut() = (0, heapless::Vec::new()));
    let mut calls = 0u32;
    while t0.elapsed() < limit {
        calls += 1;
        if rom_cmd(&esprom::sync(), Duration::from_millis(150), frame).await.is_ok() {
            // The ROM answers a sync several times over. Nothing waits for the rest: the
            // next request skips them, and the caller is in a hurry (`disable_watchdogs`).
            diag(format_args!("[sync answered on call {} after {} ms] ", calls, t0.elapsed().as_millis()));
            return true;
        }
    }
    let (total, last) = SYNC_SEEN.lock(|s| s.borrow().clone());
    diag(format_args!("[{} sync calls unanswered in {} ms; {} bytes came back, last: ", calls, limit.as_millis(), total));
    diag_bytes(&last);
    diag(format_args!("] "));
    false
}

/// Stop the chip's watchdogs while it sits in its bootloader, as esptool does for this chip
/// family: switch the RTC watchdog off and let the "super watchdog" feed itself. Done as
/// soon as the bootloader answers. Whether a C5 in its ROM bootloader needs this is not
/// known: Nodes that seemed to die there on 2026-11-27 turned out to be alive and
/// listening at the wrong rate (see `enter_bootloader`), and one watched over USB sat at
/// "waiting for download" for minutes with its watchdogs untouched and never restarted.
async fn disable_watchdogs(frame: &mut [u8; esprom::MAX_FRAME]) -> bool {
    use esprom::c5_wdt as w;
    let t = Duration::from_millis(300);
    let mut ok = true;
    // The RTC watchdog first: it is the one with a second to live.
    for (addr, value) in [(w::RWDT_WPROTECT, w::WKEY), (w::RWDT_CONFIG0, 0), (w::RWDT_WPROTECT, 0), (w::SWD_WPROTECT, w::WKEY)] {
        ok &= rom_cmd(&esprom::write_reg(addr, value), t, frame).await.is_ok();
    }
    match rom_cmd(&esprom::read_reg(w::SWD_CONFIG), t, frame).await {
        Ok(r) => ok &= rom_cmd(&esprom::write_reg(w::SWD_CONFIG, r.value | w::SWD_AUTO_FEED_EN), t, frame).await.is_ok(),
        Err(_) => ok = false,
    }
    ok &= rom_cmd(&esprom::write_reg(w::SWD_WPROTECT, 0), t, frame).await.is_ok();
    diag(format_args!("[watchdogs {}] ", if ok { "off" } else { "NOT switched off" }));
    ok
}

/// Move the bootloader on the `ROM_LOADER` link, and the link, to `baud`. Asked for with a
/// command so that the rate is exact. If the bootloader does not take the command, both
/// stay where they are, which still works, only slower.
async fn switch_baud(n: usize, baud: u32, frame: &mut [u8; esprom::MAX_FRAME]) -> Result<(), &'static str> {
    if rom_cmd(&esprom::change_baud(baud), Duration::from_millis(500), frame).await.is_err() {
        diag(format_args!("[speed change not acknowledged: staying at {}] ", ROM_BAUD));
        return Ok(());
    }
    // The answer has left at the old rate by now.
    Delay::after_millis(30).await;
    set_baud(n, baud);
    LOADER_BAUD[n].store(baud, Ordering::Relaxed);
    Delay::after_millis(20).await;
    for _ in 0..3 {
        if rom_cmd(&esprom::read_reg(esprom::c5_boot::SYS_CFG), Duration::from_millis(300), frame).await.is_ok() {
            diag(format_args!("[now at {} baud] ", baud));
            return Ok(());
        }
    }
    Err("the Node's bootloader took a faster speed and does not answer at it; it is still in its bootloader, so try again at 115200")
}

/// Put Node `n` into its ROM bootloader and leave its link in `ROM_LOADER` mode at `baud`.
/// First contact is always at `ROM_BAUD`: the bootloader works its rate out from the first
/// call, and at 921600 it gets that wrong often enough to matter (a Node on a PIO link was
/// lost that way 2026-11-27: called, never understood, and so never had its watchdog
/// switched off). The move to `baud` follows, by command.
/// Returns whether the EN and BOOT wires had to be used, which puts every Node there.
///
/// Over the Node link first (`Dfu`): only this Node restarts. A Node already sitting in
/// its bootloader (put there with its own buttons) answers the same way. The wires, where
/// fitted, are for a Node whose firmware does not answer.
async fn enter_bootloader(n: usize, baud: u32, wires: &mut Option<Wires>, frame: &mut [u8; esprom::MAX_FRAME]) -> Result<bool, &'static str> {
    // Left in its bootloader by a failed write: it is still there unless someone reset it,
    // and it listens at the rate it was first called at.
    let parked = PARKED[n].swap(0, Ordering::Relaxed);
    if parked != 0 {
        ROM_MODE[n].store(ROM_LOADER, Ordering::Relaxed);
        Delay::after_millis(40).await;
        set_baud(n, parked);
        if rom_sync(Duration::from_millis(1500), frame).await {
            disable_watchdogs(frame).await;
            LOADER_BAUD[n].store(parked, Ordering::Relaxed);
            diag(format_args!("[still in its bootloader from the last attempt] "));
            return Ok(false);
        }
        diag(format_args!("[no longer in its bootloader] "));
    }
    for _ in 0..2 {
        ROM_MODE[n].store(LINK, Ordering::Relaxed);
        set_baud(n, crate::board::node::BAUD);
        NODE_CMD[n].store(CMD_DFU, Ordering::Relaxed);
        // Sent within 20 ms; the rest lets the frame leave the wire before the baud changes.
        // No longer than needed: the Node's watchdog is running from the moment it restarts.
        Delay::after_millis(60).await;
        ROM_MODE[n].store(ROM_LOADER, Ordering::Relaxed);
        NODE_CMD[n].store(CMD_NONE, Ordering::Relaxed);
        Delay::after_millis(40).await;
        set_baud(n, ROM_BAUD);
        if rom_sync(Duration::from_secs(2), frame).await {
            disable_watchdogs(frame).await;
            LOADER_BAUD[n].store(ROM_BAUD, Ordering::Relaxed);
            diag(format_args!("[bootloader reached over the Node link] "));
            if baud != ROM_BAUD {
                switch_baud(n, baud, frame).await?;
            }
            return Ok(false);
        }
    }
    // No answer at the ROM's rate. It may be in its bootloader all the same, listening at
    // about the link's rate: the ROM bootloader takes its rate from the first traffic it
    // hears, and if that was this Timer's own pings and plans (a Node put there by its
    // buttons, or sent there and then not called in time), it settles near 921600, not quite
    // on it. It hears requests at that rate well enough, though its replies come back
    // garbled. So: tell it, unheard, to move to the ROM's rate, which a command sets
    // exactly, and call it there. Found 2026-11-27 with a Node on USB showing "waiting for
    // download" while it answered nothing; until then such a Node looked dead.
    ROM_MODE[n].store(ROM_LOADER, Ordering::Relaxed);
    Delay::after_millis(40).await;
    set_baud(n, crate::board::node::BAUD);
    for _ in 0..3 {
        let _ = rom_cmd(&esprom::change_baud(ROM_BAUD), Duration::from_millis(120), frame).await;
    }
    Delay::after_millis(40).await;
    set_baud(n, ROM_BAUD);
    if rom_sync(Duration::from_millis(1500), frame).await {
        disable_watchdogs(frame).await;
        LOADER_BAUD[n].store(ROM_BAUD, Ordering::Relaxed);
        diag(format_args!("[bootloader was listening at the link's rate: moved to {}] ", ROM_BAUD));
        if baud != ROM_BAUD {
            switch_baud(n, baud, frame).await?;
        }
        return Ok(false);
    }
    let Some(w) = wires else {
        return Err("the Node did not go into its bootloader when asked over the Node link. Put it there by hand (hold its BOOT button, tap RESET, release BOOT) and upload again, or flash it over its own USB port");
    };
    // EN and BOOT reach every Node, so they all restart into their bootloaders. Only this
    // one is spoken to; the others are left alone until the restart at the end.
    set_modes(QUIET);
    ROM_MODE[n].store(ROM_LOADER, Ordering::Relaxed);
    Delay::after_millis(150).await;
    set_baud(n, ROM_BAUD);
    reset_nodes(w, true).await;
    if rom_sync(Duration::from_secs(6), frame).await {
        disable_watchdogs(frame).await;
        LOADER_BAUD[n].store(ROM_BAUD, Ordering::Relaxed);
        diag(format_args!("[bootloader reached with EN and BOOT] "));
        if baud != ROM_BAUD {
            switch_baud(n, baud, frame).await?;
        }
        return Ok(true);
    }
    Err("the Node's bootloader did not answer, over the Node link or after EN and BOOT (check those wires with the Node restart test)")
}

/// Start the firmware of the Node whose bootloader is on the `ROM_LOADER` link, without
/// the EN wire: withdraw the request to start in the bootloader (set by the Node's `Dfu`
/// handler, and harmless to clear if the BOOT button was used), then let the chip's own
/// watchdog restart it.
async fn leave_bootloader(frame: &mut [u8; esprom::MAX_FRAME]) {
    use esprom::{c5_boot as b, c5_wdt as w};
    let t = Duration::from_millis(500);
    let mut ok = match rom_cmd(&esprom::read_reg(b::SYS_CFG), t, frame).await {
        Ok(r) => rom_cmd(&esprom::write_reg(b::SYS_CFG, r.value & !(b::FORCE_DOWNLOAD | b::SW_RESET)), t, frame).await.is_ok(),
        Err(_) => false,
    };
    for (addr, value) in [(w::RWDT_WPROTECT, w::WKEY), (w::RWDT_CONFIG1, w::RWDT_RESET_TICKS), (w::RWDT_CONFIG0, w::RWDT_SYSTEM_RESET)] {
        ok &= rom_cmd(&esprom::write_reg(addr, value), t, frame).await.is_ok();
    }
    diag(format_args!("[restart from the bootloader {}] ", if ok { "requested" } else { "NOT acknowledged" }));
}

/// Write the stored file to Node `n` through its ROM bootloader, check it, and start the
/// Node again if that can be done over its link. Also returns whether EN and BOOT were
/// used: then every Node is in its bootloader and the caller restarts them with EN.
async fn rom_flash(timer: &'static Timer, job: &NodeJob, n: usize, wires: &mut Option<Wires>) -> (Result<(), &'static str>, bool) {
    use md5::Digest as _;
    let Some(frame) = frame_take() else { return (Err("the flasher is busy"), false) };
    let by_wire = core::cell::Cell::new(None::<bool>);
    let result = async {
        let step = |text: &'static str| timer.with(|i| i.node_flash.step = text);
        let image = &crate::fw::node_store()[..job.len as usize];
        // The copy in the Timer's flash must still be what was uploaded.
        if md5::Md5::digest(image)[..] != job.md5 {
            return Err("the Timer's copy of the file is damaged; upload it again");
        }

        step("restarting the Node into its bootloader");
        by_wire.set(Some(enter_bootloader(n, job.baud, wires, frame).await?));

        step("preparing the Node's flash");
        rom_cmd(&esprom::spi_attach(), Duration::from_secs(3), frame).await?;
        rom_cmd(&esprom::spi_set_params(NODE_FLASH_BYTES), Duration::from_secs(3), frame).await?;

        // Written in sessions. If a block gets no clean answer, the Timer cannot know whether
        // the bootloader wrote it, and the bootloader refuses a block number it has already
        // counted. So after any failure: call the bootloader again (sync), and start a new
        // session at the 4 KiB flash sector the failed block lies in. Starting a session
        // erases from there on, so what is rewritten always lands on erased flash.
        const SECTOR_BLOCKS: usize = 4096 / esprom::BLOCK;
        const MAX_RESTARTS: u32 = 80;
        let total_blocks = image.len().div_ceil(esprom::BLOCK);
        let t_write = Instant::now();
        let mut start = 0usize;
        let mut restarts = 0u32;
        loop {
            let from = start * esprom::BLOCK;
            let left = job.len - from as u32;
            step(if start == 0 { "erasing" } else { "writing" });
            let mut ok = rom_cmd(&esprom::flash_begin(left, job.offset + from as u32), Duration::from_millis(esprom::erase_timeout_ms(left) as u64), frame).await.is_ok();
            if ok {
                step("writing");
                for seq in start..total_blocks {
                    let block = &image[seq * esprom::BLOCK..((seq + 1) * esprom::BLOCK).min(image.len())];
                    if rom_cmd(&esprom::flash_data((seq - start) as u32, block), Duration::from_secs(1), frame).await.is_err() {
                        if restarts < 8 {
                            diag(format_args!("[block {} failed at {} ms] ", seq, t_write.elapsed().as_millis()));
                        }
                        start = seq - seq % SECTOR_BLOCKS;
                        ok = false;
                        break;
                    }
                    timer.with(|i| i.node_flash.written = ((seq + 1) * esprom::BLOCK).min(image.len()) as u32);
                }
            }
            if ok {
                break;
            }
            restarts += 1;
            if restarts > MAX_RESTARTS {
                diag(format_args!("[gave up after {} restarts] ", restarts));
                return Err("the link to the Node's bootloader is too unreliable to finish the write");
            }
            // Usually the bootloader is still listening and only its answer was lost: go
            // straight to a new session. Call it again (sync) only if two sessions in a row
            // fail to start, since each sync has been followed by one more lost answer.
            Delay::after_millis(50).await;
            if restarts % 2 == 1 {
                continue;
            }
            Delay::after_millis(100).await;
            let mut synced = false;
            for _ in 0..20 {
                if rom_cmd(&esprom::sync(), Duration::from_millis(150), frame).await.is_ok() {
                    synced = true;
                    break;
                }
            }
            if !synced {
                diag(format_args!("[no answer to sync after block {}] ", start));
                return Err("the Node's bootloader stopped answering part way through");
            }
            Delay::after_millis(250).await;
        }
        diag(format_args!("[written in {} ms, {} restarts] ", t_write.elapsed().as_millis(), restarts));

        step("checking what was written");
        // Asked again if the answer is lost: the question changes nothing on the Node.
        let mut got = Err("the Node's bootloader did not answer");
        for attempt in 0..3 {
            got = rom_cmd(&esprom::spi_flash_md5(job.offset, job.len), Duration::from_millis(esprom::md5_timeout_ms(job.len) as u64), frame).await.map(|r| r.data);
            if got.is_ok() {
                break;
            }
            diag(format_args!("[checksum not answered, attempt {}] ", attempt + 1));
        }
        let got = got?;
        let mut want = [0u8; esprom::MD5_HEX];
        for (k, b) in job.md5.iter().enumerate() {
            want[2 * k] = b"0123456789abcdef"[(b >> 4) as usize];
            want[2 * k + 1] = b"0123456789abcdef"[(b & 15) as usize];
        }
        if !got.eq_ignore_ascii_case(&want) {
            return Err("the Node's flash does not match the file after writing");
        }
        Ok(())
    }
    .await;
    // Reached over the link. Written and checked: start it the same way, now. Failed: its
    // firmware is partly written, so leave it in its bootloader, where the next attempt
    // finds it (`PARKED`). Starting it would lose the only way back that needs no hands.
    if by_wire.get() == Some(false) {
        if result.is_ok() {
            timer.with(|i| i.node_flash.step = "starting the Node's firmware");
            leave_bootloader(frame).await;
        } else {
            PARKED[n].store(LOADER_BAUD[n].load(Ordering::Relaxed), Ordering::Relaxed);
            diag(format_args!("[left in its bootloader for the next attempt] "));
        }
    }
    frame_done(frame);
    (result, by_wire.get() == Some(true))
}

/// Restarts the Nodes after start-up, and owns the EN and BOOT wires where they are fitted.
/// Writes Node firmware on request, and runs the Node restart test:
///
/// 1. **Restart**: restart each Node and wait for it to come back (a new Hello, or its
///    clock starting again from zero).
/// 2. **Bootloader**: restart each Node into its ROM bootloader and send that SYNC packets
///    until it answers.
/// 3. Start the Nodes normally again and wait for them to return.
///
/// With EN and BOOT wired the test uses them, so it tests those wires; otherwise it uses
/// the `Reset` and `Dfu` messages on each Node's link. Timing stops for the duration (up
/// to a few minutes if a Node is slow to start).
#[embassy_executor::task]
pub async fn node_ctl_task(timer: &'static Timer, mut wires: Option<Wires>) {
    const RETURN_LIMIT: Duration = Duration::from_secs(90);
    let count = timer.with(|i| i.node_count());
    loop {
        let job = match embassy_futures::select::select(WIRE_TEST.wait(), NODE_FLASH.wait()).await {
            embassy_futures::select::Either::First(()) => None,
            embassy_futures::select::Either::Second(job) => Some(job),
        };
        if let Some(job) = job {
            let t0 = Instant::now();
            FLASH_DIAG.lock(|d| d.borrow_mut().clear());
            let (first, last) = match job.node {
                Some(n) => (n as usize, n as usize + 1),
                None => (0, count),
            };
            timer.with(|i| i.node_flash.nodes = (last - first) as u8);
            let mut result = Ok(());
            // Read before any Node is restarted: one reached over its link starts again as
            // soon as it is written.
            let base = reboots();
            let mut hard = false;
            for n in first..last {
                timer.with(|i| {
                    i.node_flash.node = n as u8;
                    i.node_flash.written = 0;
                });
                if last - first > 1 {
                    diag(format_args!("[Node {}] ", n + 1));
                }
                let (r, by_wire) = rom_flash(timer, &job, n, &mut wires).await;
                result = r;
                hard |= by_wire;
                Delay::after_millis(150).await;
                set_baud(n, crate::board::node::BAUD);
                // Free the bootloader pipes for the next Node. After EN and BOOT every Node
                // sits in its bootloader until the restart below.
                ROM_MODE[n].store(if hard { QUIET } else { LINK }, Ordering::Relaxed);
                if result.is_err() {
                    break;
                }
                timer.with(|i| i.node_flash.nodes_done += 1);
            }
            // Whatever happened, go back to the Node link and start the Nodes normally: after
            // a failed write the old firmware is gone, and that Node then simply stays silent.
            timer.with(|i| i.node_flash.step = "restarting the Nodes");
            if let (true, Some(w)) = (hard, &mut wires) {
                reset_nodes(w, false).await;
            }
            set_modes(LINK);
            // A Node left in its bootloader is not coming back until it is written.
            let waiting = (first..last).any(parked);
            let limit = if waiting {
                Duration::from_secs(0)
            } else if result.is_err() {
                Duration::from_secs(10)
            } else if wires.is_some() && !hard {
                Duration::from_secs(20)
            } else {
                RETURN_LIMIT
            };
            let mut back = wait_reboots(first, last, base, limit).await;
            // Asked over the link to start and not back: use EN if it is there.
            if let (false, true, Some(w)) = (hard, result.is_ok() && back[first..last].iter().any(|b| b.is_none()), &mut wires) {
                diag(format_args!("[not back after the restart over the link: EN used] "));
                reset_nodes(w, false).await;
                back = wait_reboots(first, last, base, RETURN_LIMIT).await;
            }
            timer.with(|i| {
                i.node_flash = NodeFlash {
                    running: false,
                    done: true,
                    ok: result.is_ok(),
                    step: "finished",
                    error: result.err().unwrap_or(""),
                    returned: Some(back[first..last].iter().all(|b| b.is_some())),
                    in_bootloader: waiting,
                    seconds: t0.elapsed().as_secs() as u32,
                    ..i.node_flash
                };
            });
            continue;
        }
        let by_wire = wires.is_some();
        FLASH_DIAG.lock(|d| d.borrow_mut().clear());
        timer.with(|i| i.wire_test = WireTest { running: true, by_wire, step: "restarting the Nodes and waiting for them to return", ..WireTest::default() });

        let base = reboots();
        match &mut wires {
            Some(w) => reset_nodes(w, false).await,
            None => (0..count).for_each(|n| NODE_CMD[n].store(CMD_RESET, Ordering::Relaxed)),
        }
        let back = wait_reboots(0, count, base, RETURN_LIMIT).await;
        timer.with(|i| {
            for n in 0..count {
                i.wire_test.nodes[n] = NodeWireTest { en_ok: Some(back[n].is_some()), en_ms: back[n].unwrap_or(0), ..NodeWireTest::default() };
            }
            i.wire_test.step = "restarting the Nodes into their bootloaders and calling those";
        });

        let mut answered = [false; MAX_NODES];
        let base;
        match &mut wires {
            Some(w) => {
                // Stop Node-link traffic, let the last frame drain, then move to the ROM's baud rate.
                for seen in &ROM_SYNC_SEEN {
                    seen.store(0, Ordering::Relaxed);
                }
                set_modes(ROM_PROBE);
                Delay::after_millis(150).await;
                for n in 0..count {
                    set_baud(n, ROM_BAUD);
                }
                reset_nodes(w, true).await;
                let t0 = Instant::now();
                let seen = |n: usize| ROM_SYNC_SEEN[n].load(Ordering::Relaxed) > 0;
                while t0.elapsed() < Duration::from_secs(5) && !(0..count).all(seen) {
                    Delay::after_millis(100).await;
                }
                for n in 0..count {
                    answered[n] = seen(n);
                }
                timer.with(|i| {
                    for n in 0..count {
                        i.wire_test.nodes[n].boot_ok = Some(answered[n]);
                    }
                    i.wire_test.step = "restarting the Nodes normally";
                });
                // Back to the Node link. Reset while still quiet, then resume pings.
                Delay::after_millis(150).await;
                for n in 0..count {
                    set_baud(n, crate::board::node::BAUD);
                }
                base = reboots();
                reset_nodes(w, false).await;
                set_modes(LINK);
            }
            None => {
                // Over the link: one Node at a time, by the path a firmware write takes
                // (into the bootloader, watchdogs off, then started again), with nothing
                // written. The other Nodes keep running.
                base = reboots();
                if let Some(frame) = frame_take() {
                    let mut none = None;
                    for n in 0..count {
                        // A Node waiting in its bootloader after a failed write has no
                        // firmware to go back to: find it there, and leave it there.
                        let was_parked = parked(n);
                        answered[n] = enter_bootloader(n, ROM_BAUD, &mut none, frame).await.is_ok();
                        timer.with(|i| i.wire_test.nodes[n].boot_ok = Some(answered[n]));
                        if answered[n] && was_parked {
                            PARKED[n].store(LOADER_BAUD[n].load(Ordering::Relaxed), Ordering::Relaxed);
                            diag(format_args!("[Node {} left in its bootloader: its last write failed] ", n + 1));
                        } else if answered[n] {
                            leave_bootloader(frame).await;
                        }
                        Delay::after_millis(100).await;
                        set_baud(n, crate::board::node::BAUD);
                        ROM_MODE[n].store(LINK, Ordering::Relaxed);
                        if !answered[n] {
                            // A Node that never left its firmware has nothing to return
                            // from: restart it so that it reports in like the others.
                            NODE_CMD[n].store(CMD_RESET, Ordering::Relaxed);
                        }
                    }
                    frame_done(frame);
                }
                timer.with(|i| i.wire_test.step = "waiting for the Nodes to return");
            }
        }
        let back = wait_reboots(0, count, base, RETURN_LIMIT).await;
        timer.with(|i| {
            for n in 0..count {
                i.wire_test.nodes[n].recovered = Some(back[n].is_some());
                i.wire_test.nodes[n].recover_ms = back[n].unwrap_or(0);
            }
            i.wire_test.step = "finished";
            i.wire_test.running = false;
            i.wire_test.done = true;
        });
    }
}
