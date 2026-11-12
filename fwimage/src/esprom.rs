//! The Espressif ROM bootloader's serial protocol, as far as the Timer needs it to write a
//! Node's flash over the Node link's UART: packet building, SLIP framing, reply parsing and
//! a check of the image before anything is sent. No I/O here.
//!
//! Reference: Espressif's "Serial Protocol" document and esptool (`esptool/loader.py`). This
//! talks to the ROM itself, not to esptool's RAM stub: 1 KiB blocks, four status bytes in a
//! reply, and no `FLASH_END` (it would make the ROM leave download mode; the Timer resets
//! the Node with its EN line instead).
//!
//! ```text
//! request:  C0 | 00 op len:u16 checksum:u32 data[len] | C0      (SLIP-escaped inside)
//! reply:    C0 | 01 op len:u16 value:u32 data[len]    | C0      data ends with the status
//! ```

pub const OP_FLASH_BEGIN: u8 = 0x02;
pub const OP_FLASH_DATA: u8 = 0x03;
pub const OP_SYNC: u8 = 0x08;
pub const OP_WRITE_REG: u8 = 0x09;
pub const OP_READ_REG: u8 = 0x0a;
pub const OP_SPI_SET_PARAMS: u8 = 0x0b;
pub const OP_SPI_ATTACH: u8 = 0x0d;
pub const OP_SPI_FLASH_MD5: u8 = 0x13;

/// Data bytes per `FLASH_DATA` block when talking to the ROM.
pub const BLOCK: usize = 0x400;
/// Largest request before SLIP framing: header, the block header, one block.
pub const MAX_REQUEST: usize = 8 + 16 + BLOCK;
/// Largest SLIP frame: every byte escaped, plus the two end markers.
pub const MAX_FRAME: usize = 2 * MAX_REQUEST + 2;
/// The MD5 reply from the ROM: 32 ASCII hex digits.
pub const MD5_HEX: usize = 32;

const SLIP_END: u8 = 0xc0;
const SLIP_ESC: u8 = 0xdb;
const SLIP_ESC_END: u8 = 0xdc;
const SLIP_ESC_ESC: u8 = 0xdd;
const CHECKSUM_SEED: u8 = 0xef;

/// Where the parts of a Node firmware go in the C5's flash.
pub const C5_BOOTLOADER_OFFSET: u32 = 0x2000;
pub const C5_PARTITIONS_OFFSET: u32 = 0x8000;
pub const C5_APP_OFFSET: u32 = 0x1_0000;
/// `chip_id` in an ESP32-C5 image header.
pub const C5_CHIP_ID: u16 = 23;

/// A request being built: raw bytes, then `frame()` wraps them for the wire.
pub struct Request {
    buf: [u8; MAX_REQUEST],
    len: usize,
}

impl Request {
    fn new(op: u8, checksum: u32) -> Self {
        let mut buf = [0u8; MAX_REQUEST];
        buf[1] = op;
        buf[4..8].copy_from_slice(&checksum.to_le_bytes());
        Self { buf, len: 8 }
    }

    fn word(mut self, w: u32) -> Self {
        self.buf[self.len..self.len + 4].copy_from_slice(&w.to_le_bytes());
        self.len += 4;
        self
    }

    fn done(mut self) -> Self {
        let n = (self.len - 8) as u16;
        self.buf[2..4].copy_from_slice(&n.to_le_bytes());
        self
    }

    pub fn op(&self) -> u8 {
        self.buf[1]
    }

    /// The request before framing.
    pub fn bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    /// SLIP-frame the request into `out`; returns the frame length.
    pub fn frame(&self, out: &mut [u8; MAX_FRAME]) -> usize {
        let mut n = 0;
        out[n] = SLIP_END;
        n += 1;
        for &b in self.bytes() {
            match b {
                SLIP_END => {
                    out[n] = SLIP_ESC;
                    out[n + 1] = SLIP_ESC_END;
                    n += 2;
                }
                SLIP_ESC => {
                    out[n] = SLIP_ESC;
                    out[n + 1] = SLIP_ESC_ESC;
                    n += 2;
                }
                b => {
                    out[n] = b;
                    n += 1;
                }
            }
        }
        out[n] = SLIP_END;
        n + 1
    }
}

/// `SYNC`: 07 07 12 20, then 32 x 55. The ROM measures the baud rate from it.
pub fn sync() -> Request {
    let mut r = Request::new(OP_SYNC, 0);
    r.buf[8..12].copy_from_slice(&[0x07, 0x07, 0x12, 0x20]);
    r.buf[12..44].fill(0x55);
    r.len = 44;
    r.done()
}

/// `SPI_ATTACH` with the default flash pins. The ROM takes a second word ("is legacy" = 0).
pub fn spi_attach() -> Request {
    Request::new(OP_SPI_ATTACH, 0).word(0).word(0).done()
}

/// `SPI_SET_PARAMS`: flash id 0, total size, 64 KiB blocks, 4 KiB sectors, 256-byte pages.
pub fn spi_set_params(flash_bytes: u32) -> Request {
    Request::new(OP_SPI_SET_PARAMS, 0).word(0).word(flash_bytes).word(0x1_0000).word(0x1000).word(0x100).word(0xffff).done()
}

/// `FLASH_BEGIN`: erases `size` bytes at `offset` and expects `blocks(size)` blocks. The last
/// word (0 = not encrypted) is taken by the ROMs of chips with flash encryption, the C5 among
/// them. The ROM erases before it answers, so allow `erase_timeout_ms(size)`.
pub fn flash_begin(size: u32, offset: u32) -> Request {
    Request::new(OP_FLASH_BEGIN, 0).word(size).word(blocks(size)).word(BLOCK as u32).word(offset).word(0).done()
}

/// `FLASH_DATA` block `seq`. `data` is at most one block; a short last block is padded with
/// 0xff, as flash reads when erased.
pub fn flash_data(seq: u32, data: &[u8]) -> Request {
    assert!(!data.is_empty() && data.len() <= BLOCK);
    let mut block = [0xffu8; BLOCK];
    block[..data.len()].copy_from_slice(data);
    let checksum = block.iter().fold(CHECKSUM_SEED, |a, &b| a ^ b);
    let mut r = Request::new(OP_FLASH_DATA, checksum as u32).word(BLOCK as u32).word(seq).word(0).word(0);
    r.buf[r.len..r.len + BLOCK].copy_from_slice(&block);
    r.len += BLOCK;
    r.done()
}

/// `SPI_FLASH_MD5` over `size` bytes at `offset`. The ROM answers with 32 hex digits.
pub fn spi_flash_md5(offset: u32, size: u32) -> Request {
    Request::new(OP_SPI_FLASH_MD5, 0).word(offset).word(size).word(0).word(0).done()
}

/// `WRITE_REG`: write `value` to the 32-bit register at `addr` (all bits, no delay).
pub fn write_reg(addr: u32, value: u32) -> Request {
    Request::new(OP_WRITE_REG, 0).word(addr).word(value).word(0xffff_ffff).word(0).done()
}

/// `READ_REG`: the register's value comes back in the reply's `value` field.
pub fn read_reg(addr: u32) -> Request {
    Request::new(OP_READ_REG, 0).word(addr).done()
}

/// The C5's low-power watchdog block (the same registers as the C6; esptool's names).
pub mod c5_wdt {
    const BASE: u32 = 0x600b_1c00;
    pub const RWDT_CONFIG0: u32 = BASE;
    pub const RWDT_WPROTECT: u32 = BASE + 0x18;
    pub const SWD_CONFIG: u32 = BASE + 0x1c;
    pub const SWD_WPROTECT: u32 = BASE + 0x20;
    /// Unlocks either write-protect register.
    pub const WKEY: u32 = 0x50d8_3aa1;
    pub const SWD_AUTO_FEED_EN: u32 = 1 << 18;
}

pub const fn blocks(size: u32) -> u32 {
    size.div_ceil(BLOCK as u32)
}

/// esptool's allowance for the erase in `FLASH_BEGIN`: 30 s per megabyte, at least 10 s here.
pub const fn erase_timeout_ms(size: u32) -> u32 {
    let ms = (size as u64 * 30_000 / 1_000_000) as u32;
    if ms < 10_000 {
        10_000
    } else {
        ms
    }
}

/// esptool's allowance for the MD5: 8 s per megabyte, at least 3 s.
pub const fn md5_timeout_ms(size: u32) -> u32 {
    let ms = (size as u64 * 8_000 / 1_000_000) as u32;
    if ms < 3_000 {
        3_000
    } else {
        ms
    }
}

/// Collects one SLIP frame from the wire, a byte at a time.
pub struct Deframer<const N: usize> {
    buf: [u8; N],
    len: usize,
    in_frame: bool,
    escape: bool,
    overflow: bool,
}

impl<const N: usize> Deframer<N> {
    pub const fn new() -> Self {
        Self { buf: [0; N], len: 0, in_frame: false, escape: false, overflow: false }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.in_frame = false;
        self.escape = false;
        self.overflow = false;
    }

    /// Feed one byte. `true` when a complete, non-empty frame is ready in `frame()`; the next
    /// `push` starts a new one.
    pub fn push(&mut self, b: u8) -> bool {
        if !self.in_frame {
            if b == SLIP_END {
                self.in_frame = true;
                self.len = 0;
                self.escape = false;
                self.overflow = false;
            }
            return false;
        }
        if b == SLIP_END {
            if self.len == 0 {
                // Two end markers in a row: the second one opens the frame.
                return false;
            }
            self.in_frame = false;
            return !self.overflow;
        }
        let b = if self.escape {
            self.escape = false;
            match b {
                SLIP_ESC_END => SLIP_END,
                SLIP_ESC_ESC => SLIP_ESC,
                other => other,
            }
        } else if b == SLIP_ESC {
            self.escape = true;
            return false;
        } else {
            b
        };
        if self.len < N {
            self.buf[self.len] = b;
            self.len += 1;
        } else {
            self.overflow = true;
        }
        false
    }

    pub fn frame(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

impl<const N: usize> Default for Deframer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// A parsed reply.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Reply<'a> {
    pub op: u8,
    pub value: u32,
    /// Reply data without the status bytes.
    pub data: &'a [u8],
    /// 0 = success.
    pub status: u8,
    /// The ROM's error code when `status` is not 0.
    pub error: u8,
}

/// Parse a deframed reply. `None` if it is not a reply (wrong direction byte, too short, or
/// the length field does not match). The ROM ends the data with four status bytes; the
/// first two are status and error.
pub fn parse_reply(frame: &[u8]) -> Option<Reply<'_>> {
    if frame.len() < 8 || frame[0] != 0x01 {
        return None;
    }
    let len = u16::from_le_bytes([frame[2], frame[3]]) as usize;
    let data = frame.get(8..8 + len)?;
    if frame.len() != 8 + len || data.len() < 2 {
        return None;
    }
    // Four status bytes from the ROM; tolerate a two-byte status (esptool's stub) as well.
    let status_len = if data.len() >= 4 { 4 } else { 2 };
    let (body, status) = data.split_at(data.len() - status_len);
    Some(Reply { op: frame[1], value: u32::from_le_bytes([frame[4], frame[5], frame[6], frame[7]]), data: body, status: status[0], error: status[1] })
}

/// What the ROM's error byte means (esptool's list).
pub fn error_text(code: u8) -> &'static str {
    match code {
        0x05 => "the Node's bootloader did not understand the request",
        0x06 => "the Node's bootloader could not act on the request",
        0x07 => "a block arrived damaged (checksum)",
        0x08 => "the Node's flash did not take the write",
        0x09 => "the Node's flash could not be read back",
        0x0a => "the Node's bootloader got a block of the wrong length",
        0x0b => "the Node's bootloader could not unpack the data",
        _ => "the Node's bootloader reported an error",
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ImageError {
    Offset,
    Empty,
    TooLarge,
    NotAnImage,
    WrongChip,
    NotAPartitionTable,
}

impl ImageError {
    pub fn as_str(self) -> &'static str {
        match self {
            ImageError::Offset => "offset must be 0x2000 (bootloader), 0x8000 (partition table) or 0x10000 (firmware)",
            ImageError::Empty => "the file is empty",
            ImageError::TooLarge => "the file is too large for that part of the Node's flash",
            ImageError::NotAnImage => "this is not an ESP firmware image (it does not start with 0xE9)",
            ImageError::WrongChip => "this firmware is for another chip, not the ESP32-C5",
            ImageError::NotAPartitionTable => "this is not a partition table (it does not start with AA 50)",
        }
    }
}

/// Largest file accepted at `offset`, or `None` for an offset that is not one of the three
/// parts. The firmware limit is what the Timer has room to hold, not the partition size.
pub const fn max_len(offset: u32, store_bytes: u32) -> Option<u32> {
    match offset {
        C5_BOOTLOADER_OFFSET => Some(C5_PARTITIONS_OFFSET - C5_BOOTLOADER_OFFSET),
        C5_PARTITIONS_OFFSET => Some(0x1000),
        C5_APP_OFFSET => Some(store_bytes),
        _ => None,
    }
}

/// Check what is about to be written, from its first bytes (at least 16, or the whole file
/// if shorter): the offset is one of the three known parts, the size fits, and the content
/// is the kind of thing that belongs there, built for the C5.
pub fn check_image(offset: u32, len: u32, head: &[u8], store_bytes: u32) -> Result<(), ImageError> {
    let max = max_len(offset, store_bytes).ok_or(ImageError::Offset)?;
    if len == 0 {
        return Err(ImageError::Empty);
    }
    if len > max {
        return Err(ImageError::TooLarge);
    }
    if offset == C5_PARTITIONS_OFFSET {
        return if head.len() >= 2 && head[0] == 0xaa && head[1] == 0x50 { Ok(()) } else { Err(ImageError::NotAPartitionTable) };
    }
    // esp_image_header_t: magic E9 at 0, chip_id (u16) at 12.
    if head.len() < 16 || head[0] != 0xe9 {
        return Err(ImageError::NotAnImage);
    }
    if u16::from_le_bytes([head[12], head[13]]) != C5_CHIP_ID {
        return Err(ImageError::WrongChip);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn framed(r: &Request) -> ([u8; MAX_FRAME], usize) {
        let mut out = [0u8; MAX_FRAME];
        let n = r.frame(&mut out);
        (out, n)
    }

    #[test]
    fn sync_matches_the_packet_proven_on_hardware() {
        // The 46-byte frame the wiring test sent, which the C5's ROM answered.
        let mut want = [0x55u8; 46];
        want[..13].copy_from_slice(&[0xc0, 0x00, 0x08, 0x24, 0x00, 0, 0, 0, 0, 0x07, 0x07, 0x12, 0x20]);
        want[45] = 0xc0;
        let (out, n) = framed(&sync());
        assert_eq!(&out[..n], &want[..]);
    }

    #[test]
    fn requests_match_esptool() {
        // struct.pack("<II", 0, 0) for the ROM
        assert_eq!(spi_attach().bytes(), &[0, 0x0d, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let p = spi_set_params(4 * 1024 * 1024);
        assert_eq!(p.bytes().len(), 8 + 24);
        assert_eq!(&p.bytes()[8..], &[0, 0, 0, 0, 0, 0, 0x40, 0, 0, 0, 1, 0, 0, 0x10, 0, 0, 0, 1, 0, 0, 0xff, 0xff, 0, 0]);
        // 700 000 bytes at 0x10000: 684 blocks of 1024, and the fifth word for the ROM
        let b = flash_begin(700_000, 0x1_0000);
        assert_eq!(b.bytes()[..4], [0, 0x02, 20, 0]);
        let w = |i: usize| u32::from_le_bytes(b.bytes()[8 + 4 * i..12 + 4 * i].try_into().unwrap());
        assert_eq!((w(0), w(1), w(2), w(3), w(4)), (700_000, 684, 1024, 0x1_0000, 0));
        let m = spi_flash_md5(0x1_0000, 700_000);
        assert_eq!(m.bytes()[..4], [0, 0x13, 16, 0]);
    }

    #[test]
    fn register_requests_match_esptool() {
        // struct.pack("<IIII", addr, value, 0xFFFFFFFF, 0)
        let w = write_reg(c5_wdt::RWDT_WPROTECT, c5_wdt::WKEY);
        assert_eq!(w.bytes(), &[0, 0x09, 16, 0, 0, 0, 0, 0, 0x18, 0x1c, 0x0b, 0x60, 0xa1, 0x3a, 0xd8, 0x50, 0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0]);
        assert_eq!(read_reg(c5_wdt::SWD_CONFIG).bytes(), &[0, 0x0a, 4, 0, 0, 0, 0, 0, 0x1c, 0x1c, 0x0b, 0x60]);
    }

    #[test]
    fn data_block_is_padded_and_checksummed() {
        let r = flash_data(3, &[1, 2, 3]);
        let b = r.bytes();
        assert_eq!(b.len(), 8 + 16 + BLOCK);
        assert_eq!(u16::from_le_bytes([b[2], b[3]]) as usize, 16 + BLOCK);
        let w = |i: usize| u32::from_le_bytes(b[8 + 4 * i..12 + 4 * i].try_into().unwrap());
        assert_eq!((w(0), w(1), w(2), w(3)), (BLOCK as u32, 3, 0, 0));
        assert_eq!(&b[24..27], &[1, 2, 3]);
        assert!(b[27..].iter().all(|&x| x == 0xff));
        // 0xef ^ 1 ^ 2 ^ 3 ^ (1021 x 0xff): an odd count of 0xff flips every bit once
        let want = 0xefu8 ^ 1 ^ 2 ^ 3 ^ 0xff;
        assert_eq!(u32::from_le_bytes(b[4..8].try_into().unwrap()), want as u32);
        assert_eq!(blocks(1), 1);
        assert_eq!(blocks(1024), 1);
        assert_eq!(blocks(1025), 2);
    }

    #[test]
    fn slip_round_trip_with_every_awkward_byte() {
        let mut data = [0u8; BLOCK];
        for (i, b) in data.iter_mut().enumerate() {
            *b = [0xc0, 0xdb, 0xdc, 0xdd, 0x00, 0xff][i % 6];
        }
        let r = flash_data(7, &data);
        let (out, n) = framed(&r);
        assert_eq!(out[0], 0xc0);
        assert_eq!(out[n - 1], 0xc0);
        assert!(!out[1..n - 1].contains(&0xc0), "an end marker inside the frame");
        let mut d = Deframer::<{ MAX_REQUEST }>::new();
        let mut got = 0;
        for &b in &out[..n] {
            if d.push(b) {
                got += 1;
                assert_eq!(d.frame(), r.bytes());
            }
        }
        assert_eq!(got, 1);
        // worst case fits the frame buffer
        let all = flash_data(0, &[0xc0; BLOCK]);
        let (_, n) = framed(&all);
        assert!(n <= MAX_FRAME);
    }

    #[test]
    fn deframer_handles_noise_back_to_back_frames_and_overflow() {
        let mut d = Deframer::<16>::new();
        let mut frames: [[u8; 16]; 4] = [[0; 16]; 4];
        let mut lens = [0usize; 4];
        let mut k = 0;
        // boot noise, a frame, a frame sharing no gap, an over-long frame, a frame
        let wire: &[u8] = &[0x12, 0x34, 0xc0, 1, 8, 0xc0, 0xc0, 2, 0xdb, 0xdc, 0xdb, 0xdd, 0xc0];
        for &b in wire {
            if d.push(b) {
                frames[k][..d.frame().len()].copy_from_slice(d.frame());
                lens[k] = d.frame().len();
                k += 1;
            }
        }
        assert_eq!(k, 2);
        assert_eq!(&frames[0][..lens[0]], &[1, 8]);
        assert_eq!(&frames[1][..lens[1]], &[2, 0xc0, 0xdb]);
        let mut n = 0;
        d.push(0xc0);
        for _ in 0..40 {
            assert!(!d.push(0x55));
        }
        if d.push(0xc0) {
            n += 1;
        }
        assert_eq!(n, 0, "an over-long frame is dropped, not truncated");
        d.push(0xc0);
        d.push(9);
        assert!(d.push(0xc0));
        assert_eq!(d.frame(), &[9]);
    }

    #[test]
    fn replies() {
        // SYNC reply from a ROM: value, then 4 status bytes
        let ok = [0x01, 0x08, 4, 0, 0x20, 0x12, 0x07, 0x07, 0, 0, 0, 0];
        assert_eq!(parse_reply(&ok), Some(Reply { op: 8, value: 0x0707_1220, data: &[], status: 0, error: 0 }));
        // MD5 reply: 32 hex digits, then status
        let mut md5 = [0u8; 8 + 36];
        md5[..4].copy_from_slice(&[0x01, 0x13, 36, 0]);
        md5[8..40].copy_from_slice(b"d41d8cd98f00b204e9800998ecf8427e");
        let r = parse_reply(&md5).unwrap();
        assert_eq!((r.op, r.status, r.data), (0x13, 0, &b"d41d8cd98f00b204e9800998ecf8427e"[..]));
        // failure: status 1, error 7 (checksum)
        let bad = [0x01, 0x03, 4, 0, 0, 0, 0, 0, 1, 7, 0, 0];
        let r = parse_reply(&bad).unwrap();
        assert_eq!((r.status, r.error), (1, 7));
        assert_eq!(error_text(7), "a block arrived damaged (checksum)");
        // not replies
        assert_eq!(parse_reply(&[0x00, 0x08, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0]), None);
        assert_eq!(parse_reply(&[0x01, 0x08, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0]), None);
        assert_eq!(parse_reply(&[0x01, 0x08]), None);
    }

    #[test]
    fn timeouts_scale_with_size() {
        assert_eq!(erase_timeout_ms(100_000), 10_000);
        assert_eq!(erase_timeout_ms(1_000_000), 30_000);
        assert_eq!(md5_timeout_ms(100_000), 3_000);
        assert_eq!(md5_timeout_ms(1_000_000), 8_000);
    }

    #[test]
    fn image_checks() {
        const STORE: u32 = 2 * 1024 * 1024;
        let mut app = [0u8; 24];
        app[0] = 0xe9;
        app[12] = 23;
        assert_eq!(check_image(C5_APP_OFFSET, 700_000, &app, STORE), Ok(()));
        assert_eq!(check_image(C5_BOOTLOADER_OFFSET, 14_720, &app, STORE), Ok(()));
        assert_eq!(check_image(C5_BOOTLOADER_OFFSET, 0x6001, &app, STORE), Err(ImageError::TooLarge));
        assert_eq!(check_image(C5_APP_OFFSET, STORE + 1, &app, STORE), Err(ImageError::TooLarge));
        assert_eq!(check_image(C5_APP_OFFSET, 0, &app, STORE), Err(ImageError::Empty));
        assert_eq!(check_image(0x1000, 100, &app, STORE), Err(ImageError::Offset));
        let mut s3 = app;
        s3[12] = 9;
        assert_eq!(check_image(C5_APP_OFFSET, 700_000, &s3, STORE), Err(ImageError::WrongChip));
        // the Timer's own firmware, picked by mistake
        assert_eq!(check_image(C5_APP_OFFSET, 300_000, &[0x00, 0x00, 0x08, 0x20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], STORE), Err(ImageError::NotAnImage));
        assert_eq!(check_image(C5_PARTITIONS_OFFSET, 3072, &[0xaa, 0x50, 1, 2], STORE), Ok(()));
        assert_eq!(check_image(C5_PARTITIONS_OFFSET, 3072, &app, STORE), Err(ImageError::NotAPartitionTable));
        assert_eq!(check_image(C5_PARTITIONS_OFFSET, 0x1001, &[0xaa, 0x50], STORE), Err(ImageError::TooLarge));
    }
}
