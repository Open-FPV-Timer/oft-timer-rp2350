//! What a crash or a hang leaves behind, and the start that follows one.
//!
//! A panic used to halt the Timer until its power was cycled, with nothing to say why, and
//! a hang did the same. Now:
//!
//! - A **panic** writes where and why into a corner of RAM that a restart does not clear,
//!   and restarts.
//! - A **hang** (the tasks stop running) is ended by the hardware watchdog, which `main`
//!   starts and `fw::fw_task` feeds. The next start can tell: the watchdog says it fired,
//!   and the record still says "running", which every orderly restart clears.
//!
//! The next start reports either as `crash` in `/status`. If it happened in the first
//! minute of the run, that start also ignores the saved Node pins (one Node on the default
//! pins): pins the board cannot start with are the likeliest way to fail before Ethernet
//! is up, and a Timer that keeps failing at start must still reach the network. A failure
//! after a minute of running restarts on the saved pins, so a race carries on.
//!
//! First needed 2026-11-27, when the first start with four Nodes stopped the Timer before
//! it reached the network, and again that night, when it stopped answering with its
//! Ethernet lights on and no panic to report.

use core::fmt::Write;
use core::mem::MaybeUninit;
use embassy_rp::pac;

const MAGIC: u32 = 0x4F46_5444; // "OFTD"
const TEXT: usize = 200;
/// **Off for now.** With the watchdog running, two firmware uploads in a row stopped the
/// Timer part way (2026-11-28), and a deliberate hang was not followed by a restart in
/// three tries, though the first two of those ran with the watchdog switched off by the
/// boot ROM (it does that when it confirms an update; `feed` now undoes it). Not understood,
/// and each test costs a walk to the RESET button, so it waits for a debug probe. Until
/// then a hang still needs that button; a panic is recorded and restarts as described.
pub const WATCHDOG_ENABLED: bool = false;

/// The watchdog's reach. Longer than anything the firmware does with interrupts off or
/// without yielding (writing a flash sector: about 50 ms; checking a Node image: under a
/// second).
pub const WATCHDOG_US: u32 = 8_000_000;

#[repr(C)]
struct Record {
    magic: u32,
    /// Failures since the Timer last ran for a minute.
    count: u32,
    /// Set by a failure, cleared by the start that reports it: a restart for any other
    /// reason within the minute (an update, the Restart button) is not reported again.
    fresh: u32,
    /// The run has not yet lasted a minute.
    early: u32,
    /// Set at start, cleared by a panic and by every orderly restart.
    running: u32,
    len: u32,
    text: [u8; TEXT],
}

/// In `.uninit`: start-up code leaves it alone, and RAM keeps its contents over a restart.
#[link_section = ".uninit.OFT_CRASH"]
static mut RECORD: MaybeUninit<Record> = MaybeUninit::uninit();

fn record() -> &'static mut Record {
    // Safety: used from `main` and `fw::fw_task` (one executor, never at once), and by the
    // panic handler with interrupts off. Every bit pattern is a valid `Record`.
    unsafe { &mut *core::ptr::addr_of_mut!(RECORD).cast::<Record>() }
}

fn set_text(r: &mut Record, args: core::fmt::Arguments) {
    let mut w = Cursor { buf: &mut r.text, len: 0 };
    let _ = w.write_fmt(args);
    r.len = w.len as u32;
}

/// The failure that ended the last run, if one did.
#[derive(Clone)]
pub struct Crash {
    pub count: u32,
    pub text: heapless::String<TEXT>,
    /// It came in the first minute of that run: this run is on the default Node pins.
    pub safe_pins: bool,
}

/// Call once, first thing, before the watchdog is started for this run.
pub fn take() -> Option<Crash> {
    let r = record();
    let valid = r.magic == MAGIC && r.len as usize <= TEXT;
    if !valid {
        r.count = 0;
        r.fresh = 0;
    } else if WATCHDOG_ENABLED && r.running == 1 && pac::WATCHDOG.reason().read().timer() {
        // Still "running" when the watchdog fired: nothing stopped it on purpose.
        set_text(r, format_args!("watchdog: the firmware stopped running its tasks (a hang, not a panic)"));
        r.count = r.count.saturating_add(1);
        r.fresh = 1;
    }
    let crash = (valid && r.fresh == 1 && r.count > 0).then(|| {
        let mut text = heapless::String::new();
        for &b in &r.text[..r.len as usize] {
            // Printable ASCII only, and nothing that needs escaping in JSON.
            let c = match b {
                b'"' | b'\\' => '\'',
                b'\n' | b'\r' => ' ',
                32..=126 => b as char,
                _ => '?',
            };
            let _ = text.push(c);
        }
        Crash { count: r.count, text, safe_pins: r.early == 1 }
    });
    r.magic = MAGIC;
    r.fresh = 0;
    r.early = 1;
    r.running = 1;
    crash
}

/// The Timer has run for a minute: a failure from here on is not a start-up failure.
pub fn healthy() {
    let r = record();
    r.early = 0;
    r.count = 0;
}

/// The Timer is about to restart on purpose.
pub fn stopping() {
    record().running = 0;
}

/// Start the hardware watchdog. Not `embassy_rp::watchdog::Watchdog::start`: in embassy-rp
/// 0.10 that chooses what the watchdog resets with the RP2040's list (17 stages), which on
/// the RP2350 (25 stages, in another order) restarts the oscillators, the clocks and part
/// of the memory but not the processors. A hang then ended with the chip half reset and
/// stopped for good (2026-11-28). This is the pico-sdk's choice for the RP2350: every
/// stage but the two oscillators.
pub fn watchdog_start() {
    if !WATCHDOG_ENABLED {
        return;
    }
    const ALL_STAGES: u32 = 0x01ff_ffff;
    const ROSC: u32 = 1 << 2;
    const XOSC: u32 = 1 << 3;
    pac::WATCHDOG.ctrl().modify(|w| w.set_enable(false));
    pac::PSM.wdsel().write_value(pac::psm::regs::Wdsel(ALL_STAGES & !ROSC & !XOSC));
    feed();
    pac::WATCHDOG.ctrl().modify(|w| w.set_enable(true));
}

/// The hardware watchdog has that long again. Called by `fw::fw_task` while the tasks run.
pub fn feed() {
    if !WATCHDOG_ENABLED {
        return;
    }
    pac::WATCHDOG.load().write_value(pac::watchdog::regs::Load(WATCHDOG_US));
    // Switched on again if something switched it off: the boot ROM does, when it confirms
    // an update (`explicit_buy`). Every Timer that had just been updated was then running
    // with no watchdog at all, which is how the first two tests of it failed (2026-11-28).
    if !pac::WATCHDOG.ctrl().read().enable() {
        watchdog_start();
    }
}

struct Cursor<'a> {
    buf: &'a mut [u8; TEXT],
    len: usize,
}

impl Write for Cursor<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.len < TEXT {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
        Ok(())
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    let r = record();
    r.count = if r.magic == MAGIC { r.count.saturating_add(1) } else { 1 };
    set_text(r, format_args!("{}", info));
    r.fresh = 1;
    r.running = 0;
    r.magic = MAGIC;
    // The restart the page's Restart button uses, known to work on this board.
    crate::fw::restart(0)
}
