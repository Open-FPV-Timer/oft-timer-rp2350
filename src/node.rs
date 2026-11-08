//! The Node link over UART1: decode frames into the host, send plans and pings, and pulse
//! SYNC. Also drives the C5's EN/BOOT lines for reset and (later) flashing.

use crate::state::{Event, Timer, Trace};
use embassy_rp::gpio::Output;
use embassy_rp::uart::BufferedUartRx;
use embassy_rp::uart::BufferedUartTx;
use embassy_time::{Duration, Instant, Timer as Delay};
use embedded_io_async::{Read, Write};
use oft_core::Event as HostEvent;
use oft_nodelink::msg::Message;
use oft_nodelink::{encode_frame, FrameDecoder, MAX_FRAME};

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
    loop {
        let n = match rx.read(&mut buf).await {
            Ok(0) | Err(_) => {
                Delay::after_millis(5).await;
                continue;
            }
            Ok(n) => n,
        };
        let t = crate::state::now_us();
        for &b in &buf[..n] {
            let msg = match dec.push(b) {
                Some(Ok(m)) => m,
                Some(Err(_)) => {
                    timer.with(|i| i.dropped_frames = i.dropped_frames.wrapping_add(1));
                    continue;
                }
                None => continue,
            };
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
                let crate::state::Inner { host, crossings, next_seq, node_temp_dc, plan_dirty, .. } = i;
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
                    HostEvent::Status(s) => *node_temp_dc = Some(s.temp_dc),
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
