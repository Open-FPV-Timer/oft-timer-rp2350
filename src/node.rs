//! The Node link over UART1: decode frames into the host, send plans and pings, and pulse
//! SYNC. Also drives the C5's EN/BOOT lines for reset and (later) flashing.

use crate::state::{Event, NodeFlash, Timer, Trace, WireTest};
use embassy_sync::pipe::Pipe;
use oft_fwimage::esprom;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use portable_atomic::{AtomicU32, AtomicU8, Ordering};
use embassy_rp::gpio::Output;
use embassy_rp::uart::BufferedUartRx;
use embassy_rp::uart::BufferedUartTx;
use embassy_time::{Duration, Instant, Timer as Delay};
use embedded_io_async::{Read, Write};
use oft_core::Event as HostEvent;
use oft_nodelink::msg::Message;
use oft_nodelink::{encode_frame, FrameDecoder, MAX_FRAME};

/// Who the UART is talking to.
/// `LINK`: the Node firmware, over the Node link.
/// `ROM_PROBE` (wiring test): the TX task sends ROM sync packets in place of pings, and the
/// RX task counts the ROM's answers.
/// `ROM_LOADER` (flashing): the RX task passes every byte to `ROM_RX`, and the TX task sends
/// what arrives in `ROM_TX`.
static ROM_MODE: AtomicU8 = AtomicU8::new(LINK);
const LINK: u8 = 0;
const ROM_PROBE: u8 = 1;
const ROM_LOADER: u8 = 2;
/// Bytes from the ROM bootloader to the flasher, and requests from the flasher to the wire.
static ROM_RX: Pipe<CriticalSectionRawMutex, 512> = Pipe::new();
static ROM_TX: Pipe<CriticalSectionRawMutex, { esprom::MAX_FRAME }> = Pipe::new();

/// A Node firmware file sitting in the Timer's scratch area (`fw::node_store`), to be
/// written to the Node's flash.
#[derive(Clone, Copy)]
pub struct NodeJob {
    /// Offset in the Node's flash.
    pub offset: u32,
    pub len: u32,
    pub md5: [u8; 16],
    /// Baud rate to talk to the ROM bootloader at (it measures the rate from the sync).
    pub baud: u32,
}
/// Raised by `POST /oft/v1/fw/node` once the file is stored.
pub static NODE_FLASH: Signal<CriticalSectionRawMutex, NodeJob> = Signal::new();
/// Size the Node's flash is described as to its bootloader. Every part written lies below it.
const NODE_FLASH_BYTES: u32 = 4 * 1024 * 1024;
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
        let mode = ROM_MODE.load(Ordering::Relaxed);
        if mode == ROM_LOADER {
            // The flasher reads these. Its replies are tens of bytes; if the pipe is ever
            // full the rest is dropped and that request times out and is retried.
            let _ = ROM_RX.try_write(&buf[..n]);
            continue;
        }
        if mode == ROM_PROBE {
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
                        // The Node restarted and its Hello did not arrive (or has not yet):
                        // it has no plan, so send ours again.
                        timer.with(|i| i.plan_dirty = true);
                    }
                    last_pong_t_node = p.t_node;
                }
                _ => {}
            }
            // Everything below is short and non-blocking; events are queued for the WS tasks.
            let mut out: heapless::Vec<Event, 8> = heapless::Vec::new();
            timer.with(|i| {
                i.last_node_rx_us = Some(t);
                if !i.node_connected {
                    // Back after a silence (a restart, a flash, a cable): it may have no plan.
                    i.plan_dirty = true;
                }
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
        match ROM_MODE.load(Ordering::Relaxed) {
            ROM_PROBE => {
                let _ = tx.write_all(&ROM_SYNC).await;
                Delay::after_millis(100).await;
                continue;
            }
            ROM_LOADER => {
                // Send whatever the flasher queued; look at the mode again at least every 20 ms.
                let mut chunk = [0u8; 256];
                if let Ok(n) = embassy_time::with_timeout(Duration::from_millis(20), ROM_TX.read(&mut chunk)).await {
                    let _ = tx.write_all(&chunk[..n]).await;
                }
                continue;
            }
            _ => {}
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

/// What happened around a failed Node firmware write, for `GET /oft/v1/fw/node`: when, which
/// request, and the last bytes the Node sent (a restarted chip prints its reset reason).
pub static FLASH_DIAG: embassy_sync::blocking_mutex::Mutex<CriticalSectionRawMutex, core::cell::RefCell<heapless::String<360>>> =
    embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(heapless::String::new()));

fn diag(args: core::fmt::Arguments) {
    use core::fmt::Write as _;
    FLASH_DIAG.lock(|d| {
        let _ = d.borrow_mut().write_fmt(args);
    });
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
                // Text if it reads as text (a restarted chip prints why), else hex.
                let text = seen.len() > 24 && seen.iter().filter(|b| matches!(b, 32..=126 | b'\r' | b'\n')).count() * 10 >= seen.len() * 8;
                diag(format_args!("[op {:#04x}: no reply in {} ms; got {} bytes: ", req.op(), timeout.as_millis(), seen.len()));
                for &b in seen.iter().take(if text { 120 } else { 24 }) {
                    match (text, b) {
                        (true, b'\\' | b'"') => diag(format_args!("'")),
                        (true, 32..=126) => diag(format_args!("{}", b as char)),
                        (true, b'\n') => diag(format_args!("|")),
                        (true, _) => {}
                        (false, b) => diag(format_args!("{:02x}", b)),
                    }
                }
                diag(format_args!("] "));
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

/// Write the stored file to the Node through its ROM bootloader and check it. The Node is
/// in download mode when this returns, whatever the outcome; the caller restarts it.
async fn rom_flash(timer: &'static Timer, job: &NodeJob, en: &mut Output<'static>, boot: &mut Output<'static>) -> Result<(), &'static str> {
    use md5::Digest as _;
    static FRAME: static_cell::StaticCell<[u8; esprom::MAX_FRAME]> = static_cell::StaticCell::new();
    static FRAME_REF: Signal<CriticalSectionRawMutex, &'static mut [u8; esprom::MAX_FRAME]> = Signal::new();
    // The frame buffer is 2 KiB: keep it out of the task's future. Made once, handed back
    // through the signal at the end of each run.
    let frame: &'static mut [u8; esprom::MAX_FRAME] = match FRAME_REF.try_take() {
        Some(f) => f,
        None => FRAME.try_init([0; esprom::MAX_FRAME]).ok_or("the flasher is busy")?,
    };
    let result = async {
        let step = |text: &'static str| timer.with(|i| i.node_flash.step = text);
        let image = &crate::fw::node_store()[..job.len as usize];
        // The copy in the Timer's flash must still be what was uploaded.
        if md5::Md5::digest(image)[..] != job.md5 {
            return Err("the Timer's copy of the file is damaged; upload it again");
        }

        step("restarting the Node into its bootloader");
        ROM_MODE.store(ROM_LOADER, Ordering::Relaxed);
        Delay::after_millis(150).await;
        set_node_baud(job.baud);
        reset_node(en, boot, true).await;
        let t0 = Instant::now();
        loop {
            if rom_cmd(&esprom::sync(), Duration::from_millis(150), frame).await.is_ok() {
                break;
            }
            if t0.elapsed() > Duration::from_secs(6) {
                return Err("the Node's bootloader did not answer (check the EN and BOOT wires with the wiring test)");
            }
        }
        // The ROM answers a sync several times over; let the rest arrive and be discarded.
        Delay::after_millis(250).await;

        // Stop the chip's watchdogs from restarting it part way, as esptool does for this
        // chip family: switch the RTC watchdog off and let the "super watchdog" feed itself.
        // A failure here is recorded but not fatal.
        {
            use esprom::c5_wdt as w;
            let t = Duration::from_secs(1);
            let mut ok = true;
            for (addr, value) in [(w::RWDT_WPROTECT, w::WKEY), (w::RWDT_CONFIG0, 0), (w::RWDT_WPROTECT, 0), (w::SWD_WPROTECT, w::WKEY)] {
                ok &= rom_cmd(&esprom::write_reg(addr, value), t, frame).await.is_ok();
            }
            match rom_cmd(&esprom::read_reg(w::SWD_CONFIG), t, frame).await {
                Ok(r) => ok &= rom_cmd(&esprom::write_reg(w::SWD_CONFIG, r.value | w::SWD_AUTO_FEED_EN), t, frame).await.is_ok(),
                Err(_) => ok = false,
            }
            ok &= rom_cmd(&esprom::write_reg(w::SWD_WPROTECT, 0), t, frame).await.is_ok();
            diag(format_args!("[watchdogs {}] ", if ok { "off" } else { "NOT switched off" }));
        }

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
        let got = rom_cmd(&esprom::spi_flash_md5(job.offset, job.len), Duration::from_millis(esprom::md5_timeout_ms(job.len) as u64), frame).await?.data;
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
    FRAME_REF.signal(frame);
    result
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
        let job = match embassy_futures::select::select(WIRE_TEST.wait(), NODE_FLASH.wait()).await {
            embassy_futures::select::Either::First(()) => None,
            embassy_futures::select::Either::Second(job) => Some(job),
        };
        if let Some(job) = job {
            let t0 = Instant::now();
            FLASH_DIAG.lock(|d| d.borrow_mut().clear());
            let result = rom_flash(timer, &job, &mut en, &mut boot).await;
            // Whatever happened, go back to the Node link and start the Node normally: after
            // a failed write the old firmware is gone, and the Node then simply stays silent.
            timer.with(|i| i.node_flash.step = "restarting the Node");
            Delay::after_millis(150).await;
            set_node_baud(crate::board::node::BAUD);
            let base = NODE_REBOOTS.load(Ordering::Relaxed);
            reset_node(&mut en, &mut boot, false).await;
            ROM_MODE.store(LINK, Ordering::Relaxed);
            let back = wait_reboot(base, if result.is_ok() { RETURN_LIMIT } else { Duration::from_secs(10) }).await;
            timer.with(|i| {
                i.node_flash = NodeFlash {
                    running: false,
                    done: true,
                    ok: result.is_ok(),
                    step: "finished",
                    error: result.err().unwrap_or(""),
                    returned: Some(back.is_some()),
                    seconds: t0.elapsed().as_secs() as u32,
                    ..i.node_flash
                };
            });
            continue;
        }
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
        ROM_MODE.store(ROM_PROBE, Ordering::Relaxed);
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
        ROM_MODE.store(LINK, Ordering::Relaxed);
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
