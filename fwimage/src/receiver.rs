//! Writing an uploaded image into the spare slot.
//!
//! The order of flash operations is the safety argument, so it lives here where it can be
//! tested against a fake flash:
//!
//! 1. Nothing is erased until the first 4 KiB of the upload has passed [`check_upload_head`]
//!    (right chip, right board, length consistent). A wrong file leaves both slots as they were.
//! 2. The spare slot's first sector is erased first. From here until step 5 the slot has no
//!    block loop, so the boot ROM cannot start it whatever else it contains.
//! 3. Sectors 1.. are erased, written and read back as they arrive. The try-before-you-buy
//!    bit is set on the way through, whatever the file said.
//! 4. When the last byte is in, the whole image is verified *as stored*: sectors 1.. read
//!    back from flash, sector 0 from RAM, SHA-256 compared with the end block.
//! 5. Only then is sector 0 written and read back.
//!
//! The running slot is never written. An image that reaches step 5 is still only a
//! candidate: the boot ROM starts it once when asked, and keeps it only if it confirms.

use crate::image::{check_upload_head, verify, ImageError, ImageInfo, ImageSource, END_BLOCK_BYTES, END_BLOCK_TBYB_BYTE};
use crate::layout::{SECTOR_BYTES, SLOT_BYTES};

const SECTOR: usize = SECTOR_BYTES as usize;

/// The spare slot. Offsets are from the start of the slot.
pub trait SlotFlash {
    type Error;
    /// Erase the 4 KiB sector at `offset`.
    fn erase_sector(&mut self, offset: u32) -> Result<(), Self::Error>;
    /// Program the erased sector at `offset`.
    fn program_sector(&mut self, offset: u32, data: &[u8; SECTOR]) -> Result<(), Self::Error>;
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error>;
}

/// Two word-aligned sectors of working space.
#[repr(C, align(4))]
pub struct Buffers {
    /// Sector 0 of the upload, held until everything else is verified.
    pub head: [u8; SECTOR],
    /// The sector being filled, then scratch for verification.
    pub sector: [u8; SECTOR],
}

impl Buffers {
    pub const fn new() -> Self {
        Self { head: [0; SECTOR], sector: [0; SECTOR] }
    }
}

impl Default for Buffers {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum UpdateError<E> {
    /// The upload is not an acceptable image.
    Image(ImageError),
    /// The flash driver reported an error.
    Flash(E),
    /// A sector did not read back as written.
    Readback,
    /// More bytes arrived than announced.
    TooMuchData,
    /// Fewer bytes arrived than announced.
    Incomplete,
}

impl<E> UpdateError<E> {
    pub fn as_str(&self) -> &'static str {
        match self {
            UpdateError::Image(e) => e.as_str(),
            UpdateError::Flash(_) => "flash driver error while storing the image",
            UpdateError::Readback => "flash did not read back as written",
            UpdateError::TooMuchData => "more data than the announced length",
            UpdateError::Incomplete => "upload ended early",
        }
    }
}

impl<E> From<ImageError> for UpdateError<E> {
    fn from(e: ImageError) -> Self {
        UpdateError::Image(e)
    }
}

/// Receives one upload of `len` bytes.
pub struct Receiver {
    len: u32,
    received: u32,
    /// Offset in the image of the byte holding the try-before-you-buy bit.
    tbyb_at: u32,
    /// Set once the slot's first sector has been erased: flash has been changed.
    started: bool,
    failed: bool,
}

impl Receiver {
    /// `len` is the announced length (HTTP `Content-Length`).
    pub fn new(len: u32) -> Self {
        Self { len, received: 0, tbyb_at: len.wrapping_sub(END_BLOCK_BYTES).wrapping_add(END_BLOCK_TBYB_BYTE), started: false, failed: false }
    }

    pub fn received(&self) -> u32 {
        self.received
    }

    /// Whether the spare slot has been touched (its previous contents are gone).
    pub fn started(&self) -> bool {
        self.started
    }

    /// Feed the next bytes of the upload, in order.
    pub fn push<F: SlotFlash>(&mut self, bufs: &mut Buffers, flash: &mut F, data: &[u8]) -> Result<(), UpdateError<F::Error>> {
        if self.failed {
            return Err(UpdateError::Incomplete);
        }
        let r = self.push_inner(bufs, flash, data);
        self.failed = r.is_err();
        r
    }

    fn push_inner<F: SlotFlash>(&mut self, bufs: &mut Buffers, flash: &mut F, mut data: &[u8]) -> Result<(), UpdateError<F::Error>> {
        while !data.is_empty() {
            if self.received >= self.len {
                return Err(UpdateError::TooMuchData);
            }
            let fill = (self.received % SECTOR_BYTES) as usize;
            let n = data.len().min(SECTOR - fill).min((self.len - self.received) as usize);
            let in_head = self.received < SECTOR_BYTES;
            let buf = if in_head { &mut bufs.head } else { &mut bufs.sector };
            buf[fill..fill + n].copy_from_slice(&data[..n]);
            data = &data[n..];
            self.received += n as u32;
            if fill + n == SECTOR {
                if in_head {
                    // The announced length may still be anything: check it with the head.
                    check_upload_head(&bufs.head, self.len)?;
                    self.started = true;
                    flash.erase_sector(0).map_err(UpdateError::Flash)?;
                } else {
                    self.store(bufs, flash, self.received - SECTOR_BYTES)?;
                }
            }
        }
        Ok(())
    }

    /// Write `bufs.sector` to the sector at `offset` (never sector 0) and read it back.
    fn store<F: SlotFlash>(&mut self, bufs: &mut Buffers, flash: &mut F, offset: u32) -> Result<(), UpdateError<F::Error>> {
        debug_assert!(offset >= SECTOR_BYTES && offset % SECTOR_BYTES == 0 && offset + SECTOR_BYTES <= SLOT_BYTES);
        if (offset..offset + SECTOR_BYTES).contains(&self.tbyb_at) {
            bufs.sector[(self.tbyb_at - offset) as usize] |= 0x80;
        }
        flash.erase_sector(offset).map_err(UpdateError::Flash)?;
        flash.program_sector(offset, &bufs.sector).map_err(UpdateError::Flash)?;
        readback(flash, offset, &bufs.sector)
    }

    /// Call when the upload has ended. On success the spare slot holds a complete, verified
    /// image marked try-before-you-buy, and the caller may ask the boot ROM to try it.
    pub fn finish<F: SlotFlash>(&mut self, bufs: &mut Buffers, flash: &mut F) -> Result<ImageInfo, UpdateError<F::Error>> {
        if self.failed || !self.started || self.received != self.len {
            self.failed = true;
            return Err(UpdateError::Incomplete);
        }
        self.failed = true; // one way or the other this receiver is done
        let tail = (self.len % SECTOR_BYTES) as usize;
        if tail != 0 {
            bufs.sector[tail..].fill(0xff);
            self.store(bufs, flash, self.len - tail as u32)?;
        }
        let Buffers { head, sector } = bufs;
        let info = {
            let mut stored = Stored { head, flash: &mut *flash, flash_error: None };
            match verify(&mut stored, self.len, sector) {
                Ok(info) => info,
                Err(e) => {
                    return Err(match (e, stored.flash_error.take()) {
                        (ImageError::Read, Some(fe)) => UpdateError::Flash(fe),
                        (e, _) => UpdateError::Image(e),
                    })
                }
            }
        };
        if !info.tbyb {
            // `store` sets the bit; if it is not there the end block is not where it was assumed.
            return Err(UpdateError::Image(ImageError::BadEndBlock));
        }
        flash.program_sector(0, head).map_err(UpdateError::Flash)?;
        readback(flash, 0, head)?;
        Ok(info)
    }
}

fn readback<F: SlotFlash>(flash: &mut F, offset: u32, expect: &[u8; SECTOR]) -> Result<(), UpdateError<F::Error>> {
    let mut chunk = [0u8; 256];
    for (k, want) in expect.chunks(chunk.len()).enumerate() {
        flash.read(offset + (k * chunk.len()) as u32, &mut chunk).map_err(UpdateError::Flash)?;
        if chunk != want {
            return Err(UpdateError::Readback);
        }
    }
    Ok(())
}

/// The image as it will be once sector 0 is written: sector 0 from RAM, the rest from flash.
struct Stored<'a, F: SlotFlash> {
    head: &'a [u8; SECTOR],
    flash: &'a mut F,
    flash_error: Option<F::Error>,
}

impl<F: SlotFlash> ImageSource for Stored<'_, F> {
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), ()> {
        let off = offset as usize;
        let from_head = SECTOR.saturating_sub(off).min(buf.len());
        buf[..from_head].copy_from_slice(&self.head[off.min(SECTOR)..off.min(SECTOR) + from_head]);
        if from_head < buf.len() {
            self.flash.read(offset + from_head as u32, &mut buf[from_head..]).map_err(|e| self.flash_error = Some(e))?;
        }
        Ok(())
    }
}
