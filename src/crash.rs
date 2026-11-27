//! What a crash leaves behind, and the safe start that follows one.
//!
//! A panic used to halt the Timer until its power was cycled, with nothing to say why. Now
//! the panic handler writes where and why into a corner of RAM that a restart does not
//! clear, and restarts. The next start reads it: `/status` shows it, and that start ignores
//! the saved Node pins (one Node on the default pins), since pins the board cannot start
//! with are the likeliest way to crash before Ethernet is up. First needed 2026-11-27,
//! when the first start with four Nodes stopped the Timer before it reached the network.
//!
//! Five crashes in a row without a minute of running in between and the Timer halts as it
//! used to, so a crash that safe pins do not cure does not restart for ever.

use core::fmt::Write;
use core::mem::MaybeUninit;

const MAGIC: u32 = 0x4F46_5443; // "OFTC"
const TEXT: usize = 200;
/// Crashes in a row before giving up.
const GIVE_UP: u32 = 5;

#[repr(C)]
struct Record {
    magic: u32,
    /// Crashes since the Timer last ran for a minute.
    count: u32,
    /// Set by the crash, cleared by the start that reports it: a restart for any other
    /// reason within the minute (an update, the Restart button) is not another safe start.
    fresh: u32,
    len: u32,
    text: [u8; TEXT],
}

/// In `.uninit`: start-up code leaves it alone, and RAM keeps its contents over a restart.
#[link_section = ".uninit.OFT_CRASH"]
static mut RECORD: MaybeUninit<Record> = MaybeUninit::uninit();

fn record() -> &'static mut Record {
    // Safety: read at the start of `main`, written by `healthy` from `main`, and by the
    // panic handler with interrupts off. Every bit pattern is a valid `Record`.
    unsafe { &mut *core::ptr::addr_of_mut!(RECORD).cast::<Record>() }
}

/// The crash that ended the last run, if one did.
#[derive(Clone)]
pub struct Crash {
    pub count: u32,
    pub text: heapless::String<TEXT>,
}

/// Call once, first thing. The record stays armed (so a crash in this run counts as one
/// more in a row) until `healthy`.
pub fn take() -> Option<Crash> {
    let r = record();
    if r.magic != MAGIC || r.count == 0 || r.len as usize > TEXT {
        r.magic = 0;
        r.count = 0;
        return None;
    }
    if r.fresh != 1 {
        return None;
    }
    r.fresh = 0;
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
    Some(Crash { count: r.count, text })
}

/// The Timer has run long enough that the next crash is a new story.
pub fn healthy() {
    let r = record();
    r.magic = 0;
    r.count = 0;
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
    let count = if r.magic == MAGIC { r.count.saturating_add(1) } else { 1 };
    let mut w = Cursor { buf: &mut r.text, len: 0 };
    let _ = write!(w, "{}", info);
    r.len = w.len as u32;
    r.count = count;
    r.fresh = 1;
    r.magic = MAGIC;
    if count >= GIVE_UP {
        loop {
            cortex_m::asm::wfi();
        }
    }
    // The restart the page's Restart button uses, known to work on this board.
    crate::fw::restart(0)
}
