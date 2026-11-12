//! The Node link over UART1: decode frames into the host, send plans and pings, and pulse
//! SYNC. Also drives the C5's EN/BOOT lines for reset and (later) flashing.

use crate::state::{Event, Timer, Trace, WireTest};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use portable_atomic::{AtomicBool, AtomicU32, Ordering};
use embassy_rp::gpio::Output;
use embassy_rp::uart::BufferedUartRx;
use embassy_rp::uart::BufferedUartTx;
use embassy_time::{Duration, Instant, Timer as Delay};
use embedded_io_async::{Read, Write};
use oft_core::Event as HostEvent;
use oft_nodelink::msg::Message;
use oft_nodelink::{encode_frame, FrameDecoder, MAX_FRAME};

/// Set while the wiring test talks to the C5's ROM bootloader: the TX task sends ROM sync
/// packets in place of pings, and the RX task looks for the ROM's answer.
static ROM_MODE: AtomicBool = AtomicBool::new(false);
/// ROM sync answers seen.
static ROM_SYNC_SEEN: AtomicU32 = AtomicU32::new(0);
/// Times the Node was seen to restart: a new Hello, or its clock running backwards.
static NODE_REBOOTS: AtomicU32 = AtomicU32::new(0);
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

/// Change UART1's baud rate after the driver has been split into halves (the driver only
/// offers this on the unsplit port). Same arithmetic as embassy-rp's `set_baudrate`.
fn set_node_baud(baud: u32) {
    let r = embassy_rp::pac::UART1;
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

async fn send(tx: &mut BufferedUartTx, m: &Message) {
    let mut buf = [0u8; MAX_FRAME];
    if let Ok(n) = encode_frame(m, &mut buf) {
        let _ = tx.write_all(&buf[..n]).await;
    }
}

/// Read the Node's bytes forever.
#[embassy_executor::task]
pub async fn node_rx_task(timer: &'static Timer, mut rx: BufferedUartRx) {
    let mut dec = FrameDecoder::new();
    let mut buf = [0u8; 256];
    let publisher = timer.events.publisher().unwrap();
    let mut rom_match = 0u8;
    let mut last_pong_t_node = 0u64;
    loop {
        let n = match rx.read(&mut buf).await {
            Ok(0) => {
                Delay::after_millis(5).await;
                continue;
            }
            Err(_) => {
                // Overrun, framing, noise or break on the wire. Count it and keep reading:
                // the frame decoder resynchronises at the next frame boundary.
                timer.with(|i| i.uart_errors = i.uart_errors.wrapping_add(1));
                continue;
            }
            Ok(n) => n,
        };
        let t = crate::state::now_us();
        if ROM_MODE.load(Ordering::Relaxed) {
            // Talking to the ROM loader at another baud rate: these are not Node-link frames.
            for &b in &buf[..n] {
                rom_match = match (rom_match, b) {
                    (0, 0xC0) => 1,
                    (1, 0x01) => 2,
                    (2, 0x08) => {
                        ROM_SYNC_SEEN.fetch_add(1, Ordering::Relaxed);
                        0
                    }
                    (_, 0xC0) => 1,
                    _ => 0,
                };
            }
            continue;
        }
        for &b in &buf[..n] {
            let msg = match dec.push(b) {
                Some(Ok(m)) => m,
                Some(Err(_)) => {
                    timer.with(|i| i.dropped_frames = i.dropped_frames.wrapping_add(1));
                    continue;
                }
                None => continue,
            };
            match &msg {
                Message::Hello(_) => {
                    NODE_REBOOTS.fetch_add(1, Ordering::Relaxed);
                }
                Message::Pong(p) => {
                    if p.t_node < last_pong_t_node {
                        NODE_REBOOTS.fetch_add(1, Ordering::Relaxed);
                    }
                    last_pong_t_node = p.t_node;
                }
                _ => {}
            }
            // Everything below is short and non-blocking; events are queued for the WS tasks.
            let mut out: heapless::Vec<Event, 8> = heapless::Vec::new();
            timer.with(|i| {
                i.last_node_rx_us = Some(t);
                i.node_connected = true;
                if let Message::Samples(s) = &msg {
                    let clock = *i.host.clock();
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
                let crate::state::Inner { host, crossings, next_seq, node_temp_dc, node_status, plan_dirty, .. } = i;
                host.feed(&msg, t, |e| match e {
                    HostEvent::Hello(_) => *plan_dirty = true,
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
                                *node_temp_dc = (s.temp_dc != i16::MIN && s.temp_dc != 0).then_some(s.temp_dc);
                        *node_status = Some(s);
                    }
                    HostEvent::Synced => {}
                });
            });
            for e in out {
                publisher.publish_immediate(e);
            }
        }
    }
}

/// Send plans when dirty, ping once a second, and watch the link for silence.
#[embassy_executor::task]
pub async fn node_tx_task(timer: &'static Timer, mut tx: BufferedUartTx) {
    let mut next_ping = Instant::now();
    loop {
        if ROM_MODE.load(Ordering::Relaxed) {
            let _ = tx.write_all(&ROM_SYNC).await;
            Delay::after_millis(100).await;
            continue;
        }
        let plan = timer.with(|i| {
            if i.plan_dirty {
                i.plan_dirty = false;
                Some(i.config.plan())
            } else {
                None
            }
        });
        if let Some(p) = plan {
            send(&mut tx, &Message::Plan(p)).await;
        }
        if Instant::now() >= next_ping {
            let t = crate::state::now_us();
            let ping = timer.with(|i| i.host.make_ping(t));
            send(&mut tx, &ping).await;
            next_ping = Instant::now() + Duration::from_secs(1);
            // Node silence check.
            timer.with(|i| {
                if let Some(last) = i.last_node_rx_us {
                    if t.saturating_sub(last) > 3_000_000 {
                        i.node_connected = false;
                    }
                }
            });
        }
        Delay::after_millis(20).await;
    }
}

/// Pulse SYNC at 20 Hz. The pulse time is recorded in the host *before* the edge so the
/// Node's report always finds its pulse.
#[embassy_executor::task]
pub async fn sync_task(timer: &'static Timer, mut sync: Output<'static>) {
    let mut next = Instant::now();
    loop {
        next += Duration::from_millis(50);
        Delay::at(next).await;
        let t = crate::state::now_us();
        timer.with(|i| {
            i.host.sync_pulsed(t);
        });
        sync.set_high();
        Delay::after_micros(200).await;
        sync.set_low();
    }
}

/// Hold the C5 in reset, then release it (normal boot). Used at power-up so the Node and
/// Timer start together, and before flashing.
pub async fn reset_node(en: &mut Output<'static>, boot: &mut Output<'static>, download_mode: bool) {
    if download_mode {
        boot.set_low();
    } else {
        boot.set_high();
    }
    en.set_low();
    Delay::after_millis(20).await;
    en.set_high();
    Delay::after_millis(50).await;
    boot.set_high();
}

/// Wait for evidence that the Node restarted after `base` was read. Returns the wait in ms.
async fn wait_reboot(base: u32, limit: Duration) -> Option<u32> {
    let t0 = Instant::now();
    while t0.elapsed() < limit {
        if NODE_REBOOTS.load(Ordering::Relaxed) != base {
            return Some(t0.elapsed().as_millis() as u32);
        }
        Delay::after_millis(100).await;
    }
    None
}

/// Owns the C5's EN and BOOT lines after start-up and runs the wiring test on request:
///
/// 1. **EN**: pulse reset and wait for the Node to come back (a new Hello, or its clock
///    starting again from zero).
/// 2. **BOOT**: hold BOOT low through a reset, which should start the C5's ROM bootloader on
///    the same serial pins, and send it SYNC packets until it answers.
/// 3. Reset normally again and wait for the Node to return.
///
/// Timing stops for the duration (up to a few minutes if the Node is slow to start).
#[embassy_executor::task]
pub async fn node_ctl_task(timer: &'static Timer, mut en: Output<'static>, mut boot: Output<'static>) {
    const RETURN_LIMIT: Duration = Duration::from_secs(90);
    loop {
        WIRE_TEST.wait().await;
        timer.with(|i| i.wire_test = WireTest { running: true, step: "EN: resetting the Node and waiting for it to return", ..WireTest::default() });

        let base = NODE_REBOOTS.load(Ordering::Relaxed);
        reset_node(&mut en, &mut boot, false).await;
        let back = wait_reboot(base, RETURN_LIMIT).await;
        timer.with(|i| {
            i.wire_test.en_ok = Some(back.is_some());
            i.wire_test.en_ms = back.unwrap_or(0);
            i.wire_test.step = "BOOT: resetting with BOOT held low and calling the ROM bootloader";
        });

        // Stop Node-link traffic, let the last frame drain, then move to the ROM's baud rate.
        ROM_SYNC_SEEN.store(0, Ordering::Relaxed);
        ROM_MODE.store(true, Ordering::Relaxed);
        Delay::after_millis(150).await;
        set_node_baud(ROM_BAUD);
        reset_node(&mut en, &mut boot, true).await;
        let t0 = Instant::now();
        while t0.elapsed() < Duration::from_secs(5) && ROM_SYNC_SEEN.load(Ordering::Relaxed) == 0 {
            Delay::after_millis(100).await;
        }
        let boot_ok = ROM_SYNC_SEEN.load(Ordering::Relaxed) > 0;
        timer.with(|i| {
            i.wire_test.boot_ok = Some(boot_ok);
            i.wire_test.step = "restarting the Node normally";
        });

        // Back to the Node link. Reset while still quiet, then resume pings.
        Delay::after_millis(150).await;
        set_node_baud(crate::board::node::BAUD);
        let base = NODE_REBOOTS.load(Ordering::Relaxed);
        reset_node(&mut en, &mut boot, false).await;
        ROM_MODE.store(false, Ordering::Relaxed);
        let back = wait_reboot(base, RETURN_LIMIT).await;
        timer.with(|i| {
            i.wire_test.recovered = Some(back.is_some());
            i.wire_test.recover_ms = back.unwrap_or(0);
            i.wire_test.step = "finished";
            i.wire_test.running = false;
            i.wire_test.done = true;
        });
    }
}
