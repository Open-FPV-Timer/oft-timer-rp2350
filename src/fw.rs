//! Firmware update over Ethernet, using the RP2350 boot ROM's A/B partitions.
//!
//! Flash holds a partition table and two firmware slots (`oft_fwimage::layout`). The boot ROM
//! maps whichever slot it starts to `0x1000_0000`, so one image runs from either. An update:
//!
//! 1. `POST /oft/v1/fw/timer` streams the new image into the slot that is *not* running
//!    (`oft_fwimage::receiver`), marked "try before you buy", and verifies it as stored.
//! 2. The Timer asks the boot ROM for a "flash update" restart naming that slot. The ROM
//!    checks the image's SHA-256 and starts it once, on trial, with a watchdog running.
//! 3. The new firmware confirms itself (`explicit_buy`) once Ethernet has an address and an
//!    HTTP request has been answered. If that does not happen the watchdog restarts the
//!    board, and the boot ROM, on a normal start, ignores unconfirmed images: the old
//!    firmware runs again.
//!
//! The running slot is never written, and there is no bootloader of ours to get wrong: the
//! choice between slots is made by mask ROM. What this file adds is the glue to hardware.
//! The decisions that matter are in `oft_fwimage`, tested on the host.
//!
//! Erasing and programming flash runs with interrupts off (flash cannot serve code while it
//! is being written): about 50 ms per 4 KiB sector. The Node link loses data during an
//! upload and for a moment at confirmation. Timing is not expected to work mid-update.

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use embassy_net::Stack;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_rp::{pac, rom_data, Peri};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant};
use oft_fwimage::image::{self, ImageInfo, ImageSource};
use oft_fwimage::layout::{self, BootInfo, LayoutError, Slot, FLASH_BYTES, SECTOR_BYTES, SLOT_BYTES, XIP_BASE, XIP_NOTRANSLATE_BASE};
use oft_fwimage::receiver::{Buffers, SlotFlash};
use static_cell::StaticCell;

const SECTOR: usize = SECTOR_BYTES as usize;

/// The tag `oft_fwimage::image::check_head` looks for, in the first 4 KiB of the image next
/// to the boot ROM's IMAGE_DEF (`memory.x` keeps `.boot_info` there).
#[repr(C, align(4))]
pub struct Tag([u8; 32]);

#[link_section = ".boot_info"]
#[used]
pub static FW_TAG: Tag = Tag(image::TAG);

// Boot ROM `reboot()` flags (pico-sdk `boot/picoboot_constants.h`).
const REBOOT_TYPE_NORMAL: u32 = 0x0;
const REBOOT_TYPE_FLASH_UPDATE: u32 = 0x4;
const REBOOT_NO_RETURN_ON_SUCCESS: u32 = 0x100;

/// How long after boot the trial watchdog is kept from expiring. The boot ROM gives a trial
/// 16.7 s; a switch port that waits out spanning tree before forwarding needs longer than
/// that to get a DHCP answer. The watchdog is reloaded (to 16.7 s) until this point and then
/// left to run out, so an unconfirmed trial ends at most `TRIAL_FEED + 16.7 s` after boot,
/// and at once if the firmware stops running its tasks.
const TRIAL_FEED: Duration = Duration::from_secs(30);
const WATCHDOG_MAX_US: u32 = 0x00ff_ffff;
/// Time for the HTTP reply to leave before the board restarts into the new image.
const SWITCH_DELAY: Duration = Duration::from_millis(1500);

/// The flash driver and the working buffers: one user at a time.
pub struct Store {
    flash: Flash<'static, FLASH, Blocking, { FLASH_BYTES as usize }>,
    pub bufs: Buffers,
}

/// What is known about the slot that is not running.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Other {
    /// No image the boot ROM would look at.
    Empty,
    /// A complete image the boot ROM can fall back to.
    Confirmed(u32),
    /// An image that was never confirmed: uploaded and not yet tried, or tried and dropped.
    Unconfirmed(u32),
}

pub struct Fw {
    /// The running slot, or why updates are not possible.
    pub slot: Result<Slot, LayoutError>,
    /// Build number of the running image, if it carries one.
    pub build: Option<u32>,
    /// The running image is still marked try-before-you-buy in flash, yet the boot ROM did
    /// not start it on trial: the ROM is not treating the image's hash block the way this
    /// scheme assumes. Updates are refused, because their safety rests on that.
    pub seal_ignored: bool,
    /// What the boot ROM reported about this boot, and the flash mapping, for diagnosis.
    pub boot: Option<BootInfo>,
    pub atrans0: u32,
    pub store: Mutex<ThreadModeRawMutex, Store>,
    /// The boot ROM started this image on trial and it has not confirmed itself yet.
    trial: AtomicBool,
    /// An HTTP request was answered since boot.
    http_seen: AtomicBool,
    /// Uptime in ms at which to restart into `switch_to`; 0 = no restart pending.
    switch_at_ms: AtomicU32,
    /// 0 = normal restart (give up a trial), else 1 + partition index for a flash update.
    switch_kind: AtomicU8,
    other_state: AtomicU8,
    other_build: AtomicU32,
    /// Boot ROM return code of a failed confirmation, 0 if none.
    buy_error: AtomicU32,
}

static FW: StaticCell<Fw> = StaticCell::new();

/// Read the boot ROM's account of this boot.
fn boot_info() -> Option<BootInfo> {
    let mut words = [0u32; 5];
    // Safety: a buffer of five words, as `rom_get_boot_info` in the pico-sdk passes.
    let n = unsafe { rom_data::get_sys_info(words.as_mut_ptr(), words.len(), layout::SYS_INFO_BOOT_INFO) };
    BootInfo::from_sys_info(&words, n)
}

/// Memory-mapped flash as an image source. `limit` bounds every read.
struct Mapped {
    base: u32,
    limit: u32,
}

impl ImageSource for Mapped {
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), ()> {
        let end = offset.checked_add(buf.len() as u32).ok_or(())?;
        if end > self.limit {
            return Err(());
        }
        // Safety: inside flash that the boot ROM mapped (the running slot, `base` = XIP_BASE)
        // or inside the 16 MB untranslated window; reads of flash have no side effects.
        unsafe { core::ptr::copy_nonoverlapping((self.base + offset) as *const u8, buf.as_mut_ptr(), buf.len()) };
        Ok(())
    }
}

/// A slot through the untranslated, uncached flash window: what is really stored there.
fn stored(slot: Slot) -> Mapped {
    Mapped { base: XIP_NOTRANSLATE_BASE + slot.offset(), limit: SLOT_BYTES }
}

fn look(slot: Slot, scratch: &mut [u8; SECTOR]) -> Other {
    match image::probe(&mut stored(slot), scratch) {
        Ok(ImageInfo { build, tbyb: false, .. }) => Other::Confirmed(build),
        Ok(ImageInfo { build, tbyb: true, .. }) => Other::Unconfirmed(build),
        Err(_) => Other::Empty,
    }
}

/// The spare slot as the receiver sees it. Writes go through the flash driver (interrupts
/// off, flash offline for the duration); reads come straight from the untranslated window.
pub struct SpareSlot<'a> {
    flash: &'a mut Flash<'static, FLASH, Blocking, { FLASH_BYTES as usize }>,
    slot: Slot,
}

impl SlotFlash for SpareSlot<'_> {
    type Error = embassy_rp::flash::Error;

    fn erase_sector(&mut self, offset: u32) -> Result<(), Self::Error> {
        if offset % SECTOR_BYTES != 0 || offset >= SLOT_BYTES {
            return Err(embassy_rp::flash::Error::OutOfBounds);
        }
        let at = self.slot.offset() + offset;
        self.flash.blocking_erase(at, at + SECTOR_BYTES)
    }

    fn program_sector(&mut self, offset: u32, data: &[u8; SECTOR]) -> Result<(), Self::Error> {
        if offset % SECTOR_BYTES != 0 || offset >= SLOT_BYTES {
            return Err(embassy_rp::flash::Error::OutOfBounds);
        }
        self.flash.blocking_write(self.slot.offset() + offset, data)
    }

    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        stored(self.slot).read(offset, buf).map_err(|_| embassy_rp::flash::Error::OutOfBounds)
    }
}

/// One of the two saved-settings sectors, read straight from flash (uncached window).
pub fn settings_sector(copy: usize) -> &'static [u8] {
    let at = XIP_NOTRANSLATE_BASE + oft_fwimage::settings::copy_offset(copy);
    // Safety: 4 KiB of flash inside the 16 MB chip; reads of flash have no side effects, and
    // the only writer (`Store::write_settings`) runs with everything else stopped.
    unsafe { core::slice::from_raw_parts(at as *const u8, SECTOR) }
}

/// The Node firmware scratch area, read straight from flash (uncached window).
pub fn node_store() -> &'static [u8] {
    let at = XIP_NOTRANSLATE_BASE + layout::NODE_STORE_OFFSET;
    // Safety: as `settings_sector`; the area lies inside the 16 MB chip.
    unsafe { core::slice::from_raw_parts(at as *const u8, layout::NODE_STORE_BYTES as usize) }
}

impl Store {
    /// The 4 KiB working buffer, for filling a sector before `write_node_sector`.
    pub fn sector_buf(&mut self) -> &mut [u8; SECTOR] {
        &mut self.bufs.sector
    }

    /// Erase sector `index` of the Node firmware scratch area and write the working buffer
    /// to it.
    pub fn write_node_sector(&mut self, index: u32) -> Result<(), &'static str> {
        let at = layout::NODE_STORE_OFFSET + index * SECTOR_BYTES;
        if at + SECTOR_BYTES > layout::NODE_STORE_OFFSET + layout::NODE_STORE_BYTES {
            return Err("the file is too large for the Timer to hold");
        }
        self.flash.blocking_erase(at, at + SECTOR_BYTES).map_err(|_| "flash erase failed")?;
        self.flash.blocking_write(at, &self.bufs.sector).map_err(|_| "flash write failed")
    }

    /// Save `payload` as the newest settings record, in the copy that is not the current
    /// newest, so a power cut leaves the previous record readable.
    pub fn write_settings(&mut self, payload: &[u8]) -> Result<(), &'static str> {
        use oft_fwimage::settings::{choose, copy_offset, decode, encode, COPIES};
        let seqs: [Option<u32>; COPIES] = core::array::from_fn(|c| decode(settings_sector(c)).map(|r| r.0));
        let (_, target, seq) = choose(seqs);
        encode(seq, payload, &mut self.bufs.sector).map_err(|_| "settings too large to save")?;
        let at = copy_offset(target);
        self.flash.blocking_erase(at, at + SECTOR_BYTES).map_err(|_| "flash erase failed")?;
        self.flash.blocking_write(at, &self.bufs.sector).map_err(|_| "flash write failed")?;
        match decode(settings_sector(target)) {
            Some((s, p)) if s == seq && p == payload => Ok(()),
            _ => Err("saved settings did not read back"),
        }
    }

    /// The slot to write an upload into. `running` must be the slot this firmware runs from.
    pub fn spare(&mut self, running: Slot) -> (SpareSlot<'_>, &mut Buffers) {
        (SpareSlot { flash: &mut self.flash, slot: running.other() }, &mut self.bufs)
    }
}

/// Why an upload cannot start, if it cannot.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum NotReady {
    Layout(LayoutError),
    SealIgnored,
    OnTrial,
    Restarting,
}

impl NotReady {
    pub fn as_str(self) -> &'static str {
        match self {
            NotReady::Layout(e) => e.as_str(),
            NotReady::SealIgnored => "the boot ROM started this firmware without the trial it was marked for; network updates are disabled, flash oft-timer.uf2 over USB",
            NotReady::OnTrial => "this firmware is still on trial after an update; wait for it to confirm",
            NotReady::Restarting => "the Timer is about to restart into new firmware",
        }
    }
}

impl Fw {
    pub fn init(flash: Peri<'static, FLASH>) -> &'static Fw {
        let boot = boot_info();
        // QMI ATRANS0: the mapping of the first 4 MB of the flash window, set by the boot ROM.
        let atrans0 = pac::QMI.atrans(0).read().0;
        let slot = match boot {
            Some(b) => layout::running_slot(b.partition, atrans0),
            None => Err(LayoutError::NoPartitionTable),
        };
        let mut bufs = Buffers::new();
        // The running image, as mapped. `check_head` bounds the end block inside one slot's
        // worth of flash, which is mapped in either layout.
        let running = image::probe(&mut Mapped { base: XIP_BASE, limit: SLOT_BYTES }, &mut bufs.sector).ok();
        let trial = boot.is_some_and(|b| b.buy_pending());
        let other = match slot {
            Ok(s) => look(s.other(), &mut bufs.sector),
            Err(_) => Other::Empty,
        };
        let fw = FW.init(Fw {
            slot,
            build: running.map(|i| i.build),
            seal_ignored: running.is_some_and(|i| i.tbyb) && !trial,
            boot,
            atrans0,
            store: Mutex::new(Store { flash: Flash::new_blocking(flash), bufs }),
            trial: AtomicBool::new(trial),
            http_seen: AtomicBool::new(false),
            switch_at_ms: AtomicU32::new(0),
            switch_kind: AtomicU8::new(0),
            other_state: AtomicU8::new(0),
            other_build: AtomicU32::new(0),
            buy_error: AtomicU32::new(0),
        });
        fw.set_other(other);
        fw
    }

    fn set_other(&self, other: Other) {
        let (state, build) = match other {
            Other::Empty => (0, 0),
            Other::Confirmed(b) => (1, b),
            Other::Unconfirmed(b) => (2, b),
        };
        self.other_build.store(build, Ordering::Relaxed);
        self.other_state.store(state, Ordering::Relaxed);
    }

    pub fn other(&self) -> Other {
        let build = self.other_build.load(Ordering::Relaxed);
        match self.other_state.load(Ordering::Relaxed) {
            1 => Other::Confirmed(build),
            2 => Other::Unconfirmed(build),
            _ => Other::Empty,
        }
    }

    /// Whether this Timer can update itself over the network (the `fw.update` capability).
    pub fn self_update(&self) -> bool {
        self.slot.is_ok() && !self.seal_ignored
    }

    pub fn on_trial(&self) -> bool {
        self.trial.load(Ordering::Relaxed)
    }

    pub fn restarting(&self) -> bool {
        self.switch_at_ms.load(Ordering::Relaxed) != 0
    }

    pub fn buy_error(&self) -> i32 {
        self.buy_error.load(Ordering::Relaxed) as i32
    }

    /// Called by the HTTP handlers a browser polls: proof that the API is reachable.
    pub fn note_http(&self) {
        self.http_seen.store(true, Ordering::Relaxed);
    }

    /// The running slot if an upload may start now.
    pub fn ready(&self) -> Result<Slot, NotReady> {
        let slot = self.slot.map_err(NotReady::Layout)?;
        if self.seal_ignored {
            return Err(NotReady::SealIgnored);
        }
        if self.on_trial() {
            return Err(NotReady::OnTrial);
        }
        if self.restarting() {
            return Err(NotReady::Restarting);
        }
        Ok(slot)
    }

    /// An upload was stored and verified in `target`: restart into it shortly.
    pub fn switch_to(&self, target: Slot, info: &ImageInfo) {
        self.set_other(Other::Unconfirmed(info.build));
        self.schedule(1 + target.partition() as u8, SWITCH_DELAY);
    }

    /// Restart the Timer shortly (after the HTTP reply has left), into the same firmware.
    pub fn restart_soon(&self) {
        if !self.restarting() {
            self.schedule(0, SWITCH_DELAY);
        }
    }

    /// An upload failed after the spare slot was touched: whatever it held is gone.
    pub fn spare_spoiled(&self) {
        self.set_other(Other::Empty);
    }

    fn schedule(&self, kind: u8, after: Duration) {
        self.switch_kind.store(kind, Ordering::Relaxed);
        let at = (Instant::now() + after).as_millis() as u32;
        self.switch_at_ms.store(at.max(1), Ordering::Relaxed);
    }

    /// Confirm the running image with the boot ROM: clears its try-before-you-buy bit and,
    /// when the old image would otherwise win the next normal start, erases that image's
    /// first sector. After this the boot ROM starts this image on every boot.
    async fn buy(&self) {
        let mut store = self.store.lock().await;
        let buf = &mut store.bufs.sector;
        let rc = critical_section::with(|_| {
            // As `embassy_rp::flash` does before taking flash offline: let any DMA transfer
            // that reads from flash finish.
            for n in 0..16 {
                let ch = pac::DMA.ch(n);
                while ch.read_addr().read() < 0x2000_0000 && ch.ctrl_trig().read().busy() {}
            }
            // Safety: a word-aligned 4 KiB scratch buffer in RAM; interrupts are off and the
            // second core is not running, so nothing else touches flash while the ROM rewrites
            // one sector (the pico-sdk calls this the same way, under `flash_safe_execute`).
            unsafe { rom_data::explicit_buy(buf.as_mut_ptr(), SECTOR as u32) }
        });
        if rc == 0 {
            self.trial.store(false, Ordering::Relaxed);
            if let Ok(slot) = self.slot {
                self.set_other(look(slot.other(), &mut store.bufs.sector));
            }
        } else {
            // The ROM stopped the trial watchdog on entry. Do not stay in an unconfirmed
            // image: a normal restart goes back to the previous firmware.
            self.buy_error.store(rc as u32, Ordering::Relaxed);
            self.schedule(0, Duration::from_millis(500));
        }
    }
}

pub(crate) fn restart(kind: u8) -> ! {
    crate::crash::stopping();
    let (flags, p0) = match kind {
        0 => (REBOOT_TYPE_NORMAL, 0),
        k => {
            let slot = if k == 1 { Slot::A } else { Slot::B };
            // p0: where flash was updated. When it is the start of a partition, the boot ROM
            // prefers that partition for this one boot, whatever the versions say.
            (REBOOT_TYPE_FLASH_UPDATE, XIP_BASE + slot.offset())
        }
    };
    // 10 ms, not 0: the ROM does not restart with a zero delay (erratum RP2350-E30).
    // Ends this program: the ROM arms the watchdog and waits for it.
    rom_data::reboot(flags | REBOOT_NO_RETURN_ON_SUCCESS, 10, p0, 0);
    loop {
        cortex_m::asm::wfi();
    }
}

/// Runs the trial after an update and performs restarts requested by `switch_to`.
#[embassy_executor::task]
pub async fn fw_task(fw: &'static Fw, stack: Stack<'static>) -> ! {
    loop {
        embassy_time::Timer::after_millis(250).await;
        let now = Instant::now();
        // The tasks are running: tell the hardware watchdog (`crash.rs`). Not during a
        // trial, which has its own rule below and must be allowed to run out.
        if !fw.on_trial() && !fw.restarting() {
            crate::crash::feed();
        }
        if fw.on_trial() && !fw.restarting() {
            if now.as_micros() < TRIAL_FEED.as_micros() {
                // The watchdog is running (`main`); push it out while the trial may still
                // succeed, then let it run down.
                pac::WATCHDOG.load().write_value(pac::watchdog::regs::Load(WATCHDOG_MAX_US));
            }
            if stack.is_config_up() && fw.http_seen.load(Ordering::Relaxed) {
                // Healthy: Ethernet has an address and a client got an answer.
                embassy_time::Timer::after_millis(250).await;
                fw.buy().await;
            }
        }
        let at = fw.switch_at_ms.load(Ordering::Relaxed);
        if at != 0 && now.as_millis() as u32 >= at {
            restart(fw.switch_kind.load(Ordering::Relaxed));
        }
    }
}
