//! What a Timer firmware image is, and the checks made before one is trusted.
//!
//! An image is the flat binary linked at `0x1000_0000`, with two boot ROM "blocks" forming
//! a loop (RP2350 datasheet 5.9):
//!
//! - the first block, inside the first 4 KiB, emitted by the firmware itself
//!   (`embassy_rp::block::ImageDef`); `tools/uf2.py` points its link at the end block;
//! - the end block, the last 88 bytes of the file, added by `tools/uf2.py`: image type,
//!   version (the build number), a load map covering everything before it, and a SHA-256 of
//!   that range plus the start of the block itself. This is the layout `picotool seal --hash`
//!   produces, and the boot ROM checks the hash every time it considers the image.
//!
//! The first 4 KiB also carries [`TAG`], which names the board and flash layout.
//!
//! The checks here are deliberately narrower than the boot ROM's: only the exact shape
//! `tools/uf2.py` writes is accepted. Anything else is refused before the switch.

use crate::layout::{LAYOUT_REV, MAX_IMAGE_BYTES, MIN_IMAGE_BYTES, SECTOR_BYTES, XIP_BASE};
use sha2::{Digest, Sha256};

pub const BLOCK_MARKER_START: u32 = 0xffff_ded3;
pub const BLOCK_MARKER_END: u32 = 0xab12_3579;

const ITEM_IMAGE_TYPE: u8 = 0x42;
const ITEM_VERSION: u8 = 0x48;
const ITEM_LOAD_MAP: u8 = 0x06;
const ITEM_HASH_DEF: u8 = 0x47;
const ITEM_HASH_VALUE: u8 = 0x4b;
const ITEM_LAST: u8 = 0xff;
const HASH_SHA256: u32 = 1;
const _: () = assert!(ITEM_VERSION == 0x48 && ITEM_LOAD_MAP == 0x06 && ITEM_HASH_DEF == 0x47 && ITEM_HASH_VALUE == 0x4b);

/// Executable, Secure, Arm, RP2350 (`IMAGE_TYPE_EXE | EXE_SECURITY_S | EXE_CPU_ARM | EXE_CHIP_RP2350`).
pub const IMAGE_FLAGS: u16 = 0x1021;
/// "Try before you buy": the boot ROM starts such an image only when told it was just
/// written, and drops it again unless the image confirms itself.
pub const IMAGE_FLAG_TBYB: u16 = 0x8000;

/// The boot ROM ignores IMAGE_DEF blocks longer than this.
const MAX_BLOCK_WORDS: usize = 0x180 / 4;

/// Marker, image type, version (2), load map (4), hash def (2), hash value (9), last, link, marker.
pub const END_BLOCK_WORDS: usize = 22;
pub const END_BLOCK_BYTES: u32 = (END_BLOCK_WORDS * 4) as u32;
/// Byte of the end block that holds the try-before-you-buy bit (0x80): the top byte of its
/// image type word. The hash is computed with the bit clear, so it may be set or cleared
/// without invalidating the image; the boot ROM clears it when the image confirms itself.
pub const END_BLOCK_TBYB_BYTE: u32 = 7;

/// 32 bytes every Timer image carries, word aligned, in its first 4 KiB: magic, board,
/// flash layout revision, reserved.
pub const TAG_MAGIC: [u8; 8] = *b"OFTFWTAG";
pub const TAG_BOARD: [u8; 16] = *b"rp2350-poe-eth\0\0";
pub const TAG: [u8; 32] = tag();

const fn tag() -> [u8; 32] {
    let mut t = [0u8; 32];
    let mut i = 0;
    while i < 8 {
        t[i] = TAG_MAGIC[i];
        i += 1;
    }
    let mut i = 0;
    while i < 16 {
        t[8 + i] = TAG_BOARD[i];
        i += 1;
    }
    let rev = LAYOUT_REV.to_le_bytes();
    let mut i = 0;
    while i < 4 {
        t[24 + i] = rev[i];
        i += 1;
    }
    t
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ImageError {
    TooSmall,
    TooLarge,
    LengthNotWordMultiple,
    /// No boot ROM block in the first 4 KiB.
    NoFirstBlock,
    /// The first block is not an executable image definition for the RP2350 (Arm, Secure).
    WrongChip,
    /// No Timer tag: some other RP2350 program.
    NotTimerImage,
    /// A Timer image for another board or flash layout.
    WrongBoard,
    /// The first block does not point at an end block at the end of the upload: the file is
    /// truncated, padded, or was not made by `tools/uf2.py`.
    LengthMismatch,
    /// The end block is missing or not the shape `tools/uf2.py` writes.
    BadEndBlock,
    /// SHA-256 over the image does not match the value in its end block.
    HashMismatch,
    /// Reading the image back failed.
    Read,
}

impl ImageError {
    pub const fn as_str(self) -> &'static str {
        match self {
            ImageError::TooSmall => "file too small to be Timer firmware",
            ImageError::TooLarge => "file too large for a firmware slot",
            ImageError::LengthNotWordMultiple => "file length is not a multiple of 4: not a Timer firmware .bin",
            ImageError::NoFirstBlock => "not an RP2350 firmware image (no image definition found)",
            ImageError::WrongChip => "not an RP2350 Arm firmware image; upload oft-timer.bin, not the .uf2",
            ImageError::NotTimerImage => "image is not OFT Timer firmware",
            ImageError::WrongBoard => "firmware is for a different board or flash layout",
            ImageError::LengthMismatch => "file is truncated or not a sealed image: upload the oft-timer.bin written by tools/uf2.py",
            ImageError::BadEndBlock => "image has no valid hash block: upload the oft-timer.bin written by tools/uf2.py",
            ImageError::HashMismatch => "image is corrupt (SHA-256 mismatch)",
            ImageError::Read => "could not read the image back from flash",
        }
    }
}

/// Random access to the bytes of an image (a slot in flash, a file, RAM).
pub trait ImageSource {
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), ()>;
}

impl ImageSource for &[u8] {
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), ()> {
        let start = offset as usize;
        let end = start.checked_add(buf.len()).ok_or(())?;
        buf.copy_from_slice(self.get(start..end).ok_or(())?);
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ImageInfo {
    /// Length in bytes, end block included.
    pub len: u32,
    /// Version from the end block: `major << 16 | minor`. `tools/uf2.py` stamps the build
    /// time here (seconds since 2020-01-01 UTC), so a later build has a higher number.
    pub build: u32,
    /// The try-before-you-buy bit is set.
    pub tbyb: bool,
    /// SHA-256 stored in the end block.
    pub hash: [u8; 32],
}

fn word(buf: &[u8], index: usize) -> u32 {
    let b = &buf[index * 4..index * 4 + 4];
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

fn item_size(header: u32) -> usize {
    if header & 0x80 != 0 {
        ((header >> 8) & 0xffff) as usize
    } else {
        ((header >> 8) & 0xff) as usize
    }
}

/// A block's frame: where its LAST item is and what it links to.
#[derive(Clone, Copy, Debug)]
struct Frame {
    last: usize,
    link: i32,
}

impl Frame {
    fn words(&self) -> usize {
        self.last + 3
    }
}

/// The boot ROM's framing rules (`s_varm_crit_next_block`): start marker, items that tile
/// the block exactly, a LAST item whose size is the word count of the items before it, the
/// link, and the end marker. `buf` starts at the candidate block and may be longer.
fn frame(buf: &[u8]) -> Option<Frame> {
    let n = (buf.len() / 4).min(MAX_BLOCK_WORDS);
    if n < 5 || word(buf, 0) != BLOCK_MARKER_START {
        return None;
    }
    let mut pos = 1;
    while pos + 2 < n {
        let header = word(buf, pos);
        let size = item_size(header);
        if size == 0 {
            return None;
        }
        if header as u8 == ITEM_LAST {
            if size != pos - 1 || word(buf, pos + 2) != BLOCK_MARKER_END {
                return None;
            }
            return Some(Frame { last: pos, link: word(buf, pos + 1) as i32 });
        }
        pos += size;
    }
    None
}

/// Image type flags if the block's first item is an IMAGE_DEF.
fn image_def_flags(buf: &[u8]) -> Option<u16> {
    let w = word(buf, 1);
    (w & 0xffff == 0x0100 | ITEM_IMAGE_TYPE as u32).then_some((w >> 16) as u16)
}

/// What the first 4 KiB says about the image.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Head {
    /// Offset of the first block.
    pub first_block: u32,
    /// Offset the first block links to: where the end block must be.
    pub end_block: u32,
}

impl Head {
    /// Image length implied by the first block's link.
    pub fn image_len(&self) -> u32 {
        self.end_block + END_BLOCK_BYTES
    }
}

/// Check the first 4 KiB of an image: a first block that is an RP2350 Arm Secure executable
/// and links forward to a word-aligned end block, and the Timer tag for this board.
/// Needs nothing but those 4 KiB, so an upload can be refused before anything is erased.
pub fn check_head(head: &[u8]) -> Result<Head, ImageError> {
    let head = &head[..head.len().min(SECTOR_BYTES as usize)];
    let mut found = None;
    let mut off = 0;
    while off + 20 <= head.len() {
        if word(head, off / 4) == BLOCK_MARKER_START {
            if let Some(f) = frame(&head[off..]) {
                found = Some((off, f));
                break;
            }
        }
        off += 4;
    }
    let (off, f) = found.ok_or(ImageError::NoFirstBlock)?;
    let block = &head[off..];
    match image_def_flags(block) {
        Some(IMAGE_FLAGS) => {}
        _ => return Err(ImageError::WrongChip),
    }
    // The link must go forward, past this block and the first sector, to a word boundary.
    let end = (off as i64) + f.link as i64;
    if f.link <= 0 || f.link % 4 != 0 || end < SECTOR_BYTES as i64 || end > (MAX_IMAGE_BYTES - END_BLOCK_BYTES) as i64 {
        return Err(ImageError::LengthMismatch);
    }
    check_tag(head)?;
    Ok(Head { first_block: off as u32, end_block: end as u32 })
}

fn check_tag(head: &[u8]) -> Result<(), ImageError> {
    let mut off = 0;
    while off + TAG.len() <= head.len() {
        if head[off..off + 8] == TAG_MAGIC {
            return if head[off..off + TAG.len()] == TAG { Ok(()) } else { Err(ImageError::WrongBoard) };
        }
        off += 4;
    }
    Err(ImageError::NotTimerImage)
}

fn check_len(len: u32) -> Result<(), ImageError> {
    if len < MIN_IMAGE_BYTES {
        return Err(ImageError::TooSmall);
    }
    if len > MAX_IMAGE_BYTES {
        return Err(ImageError::TooLarge);
    }
    if len % 4 != 0 {
        return Err(ImageError::LengthNotWordMultiple);
    }
    Ok(())
}

/// Length checks and [`check_head`] for an upload of `len` bytes whose first 4 KiB is `head`.
pub fn check_upload_head(head: &[u8], len: u32) -> Result<Head, ImageError> {
    check_len(len)?;
    let h = check_head(head)?;
    if h.image_len() != len {
        return Err(ImageError::LengthMismatch);
    }
    Ok(h)
}

struct EndBlock {
    build: u32,
    tbyb: bool,
    hash: [u8; 32],
    hashed_words: usize,
}

/// Parse the end block at `end_off`, whose bytes are `b`, for an image whose first block is
/// at `first_off`. Accepts exactly: IMAGE_DEF with the expected flags (try-before-you-buy
/// either way), VERSION without rollback, LOAD_MAP with one relative entry covering
/// `[0, end_off)` in place, HASH_DEF (SHA-256, everything up to and including itself),
/// HASH_VALUE of 8 words, and a link back to the first block.
fn parse_end_block(b: &[u8; END_BLOCK_BYTES as usize], first_off: u32, end_off: u32) -> Result<EndBlock, ImageError> {
    let bad = ImageError::BadEndBlock;
    let f = frame(b).ok_or(bad)?;
    if f.words() != END_BLOCK_WORDS || end_off as i64 + f.link as i64 != first_off as i64 {
        return Err(bad);
    }
    // Padding bytes the boot ROM ignores must be zero too: every bit of the file is pinned
    // down except the try-before-you-buy bit.
    if word(b, f.last) != ITEM_LAST as u32 | ((f.last as u32 - 1) << 8) || word(b, 1) & 0xffff != 0x0142 {
        return Err(bad);
    }
    let flags = image_def_flags(b).ok_or(bad)?;
    if flags & !IMAGE_FLAG_TBYB != IMAGE_FLAGS {
        return Err(bad);
    }
    let (mut build, mut load_map, mut hashed_words, mut hash) = (None, false, None, None);
    let mut i = 2;
    while i < f.last {
        let header = word(b, i);
        let size = item_size(header);
        if size == 0 || i + size > f.last {
            return Err(bad);
        }
        match header as u8 {
            ITEM_VERSION if build.is_none() && header == 0x0000_0248 => build = Some(word(b, i + 1)),
            ITEM_LOAD_MAP if !load_map && header == 0x0100_0406 => {
                // One entry, relative: storage start is given relative to this item's address.
                let storage = (end_off as i64) + (i as i64) * 4 + (word(b, i + 1) as i32) as i64;
                if storage != 0 || word(b, i + 2) != XIP_BASE || word(b, i + 3) != end_off {
                    return Err(bad);
                }
                load_map = true;
            }
            ITEM_HASH_DEF if hashed_words.is_none() && header == 0x0000_0247 | HASH_SHA256 << 24 => {
                // Everything from the start marker up to and including this item is hashed.
                if word(b, i + 1) as usize != i + 2 {
                    return Err(bad);
                }
                hashed_words = Some(i + 2);
            }
            ITEM_HASH_VALUE if hash.is_none() && header == 0x0000_094b && hashed_words.is_some() => {
                let mut h = [0u8; 32];
                h.copy_from_slice(&b[(i + 1) * 4..(i + 9) * 4]);
                hash = Some(h);
            }
            _ => return Err(bad),
        }
        i += size;
    }
    match (build, load_map, hashed_words, hash) {
        (Some(build), true, Some(hashed_words), Some(hash)) => Ok(EndBlock { build, tbyb: flags & IMAGE_FLAG_TBYB != 0, hash, hashed_words }),
        _ => Err(bad),
    }
}

fn inspect<S: ImageSource>(src: &mut S, len: Option<u32>, scratch: &mut [u8; SECTOR_BYTES as usize], verify_hash: bool) -> Result<ImageInfo, ImageError> {
    src.read(0, scratch).map_err(|_| ImageError::Read)?;
    let head = match len {
        Some(len) => check_upload_head(scratch, len)?,
        None => check_head(scratch)?,
    };
    let mut end = [0u8; END_BLOCK_BYTES as usize];
    src.read(head.end_block, &mut end).map_err(|_| ImageError::Read)?;
    let eb = parse_end_block(&end, head.first_block, head.end_block)?;
    if verify_hash {
        // What the boot ROM hashes: the load map range, then the block up to its HASH_DEF
        // with the try-before-you-buy bit clear.
        let mut sha = Sha256::new();
        let mut off = 0;
        while off < head.end_block {
            let n = (head.end_block - off).min(SECTOR_BYTES) as usize;
            src.read(off, &mut scratch[..n]).map_err(|_| ImageError::Read)?;
            sha.update(&scratch[..n]);
            off += n as u32;
        }
        end[END_BLOCK_TBYB_BYTE as usize] &= 0x7f;
        sha.update(&end[..eb.hashed_words * 4]);
        if sha.finalize().as_slice() != eb.hash {
            return Err(ImageError::HashMismatch);
        }
    }
    Ok(ImageInfo { len: head.image_len(), build: eb.build, tbyb: eb.tbyb, hash: eb.hash })
}

/// Full check of an image of known length: head, end block and SHA-256.
/// `scratch` is working space; its contents are overwritten.
pub fn verify<S: ImageSource>(src: &mut S, len: u32, scratch: &mut [u8; SECTOR_BYTES as usize]) -> Result<ImageInfo, ImageError> {
    inspect(src, Some(len), scratch, true)
}

/// Full check of whatever image starts at offset 0 of `src` (a whole slot): the length comes
/// from the image itself.
pub fn verify_slot<S: ImageSource>(src: &mut S, scratch: &mut [u8; SECTOR_BYTES as usize]) -> Result<ImageInfo, ImageError> {
    inspect(src, None, scratch, true)
}

/// Structure check without the hash: cheap, for reporting what is in a slot.
pub fn probe<S: ImageSource>(src: &mut S, scratch: &mut [u8; SECTOR_BYTES as usize]) -> Result<ImageInfo, ImageError> {
    inspect(src, None, scratch, false)
}
