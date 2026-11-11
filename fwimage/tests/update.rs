//! Host tests for image checks and the update sequence.
//!
//! `seal` below builds images the way `tools/uf2.py` does, independently of it. Set
//! `OFT_FW_IMAGE` to a `.bin` written by the real tool to run the same checks on it.
//!
//! The power-cut tests use `rom_boot`, a model of the boot ROM's choice between the two
//! slots transcribed from `s_varm_crit_choose_by_tbyb_flash_update_boot_and_version` and
//! `s_varm_crit_ram_trash_verify_and_launch_flash_image` in pico-bootrom-rp2350. The model's
//! idea of a valid image is this crate's (stricter than the ROM's), which is sound here
//! because every intermediate state either has no closed block loop at all or ends in the
//! hashed end block, which the ROM checks the same way.

use oft_fwimage::image::*;
use oft_fwimage::layout::*;
use oft_fwimage::receiver::*;
use sha2::{Digest, Sha256};

const SECTOR: usize = 4096;
const FIRST_BLOCK: usize = 0x110;

fn put(v: &mut [u8], off: usize, w: u32) {
    v[off..off + 4].copy_from_slice(&w.to_le_bytes());
}

/// An unsealed image of `body` bytes: filler, a first block at 0x110 looping to itself, the tag.
fn raw(body: usize, seed: u32) -> Vec<u8> {
    assert!(body % 4 == 0);
    let mut x = seed.wrapping_mul(2654435761).wrapping_add(12345);
    let mut v: Vec<u8> = (0..body)
        .map(|_| {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            // Never emit 0xff or 0xd3 so the filler cannot contain a block start marker.
            (x as u8) & 0x7f
        })
        .collect();
    put(&mut v, FIRST_BLOCK, BLOCK_MARKER_START);
    put(&mut v, FIRST_BLOCK + 4, 0x0142 | (IMAGE_FLAGS as u32) << 16);
    put(&mut v, FIRST_BLOCK + 8, 0x0000_01ff);
    put(&mut v, FIRST_BLOCK + 12, 0);
    put(&mut v, FIRST_BLOCK + 16, BLOCK_MARKER_END);
    v[FIRST_BLOCK + 20..FIRST_BLOCK + 20 + TAG.len()].copy_from_slice(&TAG);
    v
}

/// Append the end block: what `picotool seal --hash` and `tools/uf2.py` do.
fn seal_raw(mut v: Vec<u8>, build: u32, tbyb: bool) -> Vec<u8> {
    let end = v.len();
    put(&mut v, FIRST_BLOCK + 12, (end - FIRST_BLOCK) as u32);
    let flags = IMAGE_FLAGS | if tbyb { IMAGE_FLAG_TBYB } else { 0 };
    let mut w: Vec<u32> = vec![
        BLOCK_MARKER_START,
        0x0142 | (flags as u32) << 16,
        0x0248,
        build,
        0x0100_0406,
        (0i64 - (end as i64 + 4 * 4)) as u32,
        XIP_BASE,
        end as u32,
        0x0100_0247,
        10,
    ];
    let mut sha = Sha256::new();
    sha.update(&v);
    let mut hashed: Vec<u8> = w.iter().flat_map(|x| x.to_le_bytes()).collect();
    hashed[7] &= 0x7f;
    sha.update(&hashed);
    let digest = sha.finalize();
    w.push(0x094b);
    w.extend(digest.chunks(4).map(|c| u32::from_le_bytes(c.try_into().unwrap())));
    w.push(0xff | ((w.len() as u32 - 1) << 8));
    w.push((FIRST_BLOCK as i64 - end as i64) as u32);
    w.push(BLOCK_MARKER_END);
    assert_eq!(w.len(), END_BLOCK_WORDS);
    v.extend(w.iter().flat_map(|x| x.to_le_bytes()));
    v
}

fn seal(body: usize, build: u32, tbyb: bool) -> Vec<u8> {
    seal_raw(raw(body, build), build, tbyb)
}

fn check(img: &[u8]) -> Result<ImageInfo, ImageError> {
    let mut scratch = [0u8; SECTOR];
    let mut src = img;
    verify(&mut src, img.len() as u32, &mut scratch)
}

// ------------------------------------------------------------------ image checks

#[test]
fn a_sealed_image_verifies_with_the_tbyb_bit_either_way() {
    for body in [2 * SECTOR, 2 * SECTOR + 4, 3 * SECTOR - 88, 3 * SECTOR - 84, 60 * SECTOR + 1236] {
        let img = seal(body, 0x0001_0002, false);
        let info = check(&img).unwrap();
        assert_eq!(info, ImageInfo { len: img.len() as u32, build: 0x0001_0002, tbyb: false, hash: info.hash });
        let mut t = img.clone();
        let at = t.len() - END_BLOCK_BYTES as usize + END_BLOCK_TBYB_BYTE as usize;
        t[at] |= 0x80;
        assert_eq!(t, seal(body, 0x0001_0002, true), "the bit is all that differs");
        assert_eq!(check(&t).unwrap(), ImageInfo { tbyb: true, ..info });
        // A slot holds the image followed by whatever was there before.
        let mut slot = t.clone();
        slot.resize(slot.len() + 3 * SECTOR, 0x5a);
        let mut scratch = [0u8; SECTOR];
        assert_eq!(verify_slot(&mut &slot[..], &mut scratch).unwrap(), ImageInfo { tbyb: true, ..info });
        assert_eq!(probe(&mut &slot[..], &mut scratch).unwrap(), ImageInfo { tbyb: true, ..info });
    }
}

#[test]
fn any_single_bit_flip_is_caught() {
    let img = seal(2 * SECTOR + 256, 7, false);
    let tbyb_at = img.len() - END_BLOCK_BYTES as usize + END_BLOCK_TBYB_BYTE as usize;
    for byte in 0..img.len() {
        // Every bit of the head and the end block, a sample of the bits elsewhere.
        let bits: &[u8] = if byte < 0x200 || byte >= img.len() - 100 { &[0, 1, 2, 3, 4, 5, 6, 7] } else { &[(byte % 8) as u8] };
        for &bit in bits {
            if byte == tbyb_at && bit == 7 {
                continue; // the one bit that is allowed to change
            }
            let mut bad = img.clone();
            bad[byte] ^= 1 << bit;
            assert!(check(&bad).is_err(), "flip of bit {bit} in byte {byte:#x} went unnoticed");
        }
    }
}

#[test]
fn truncated_or_padded_files_are_refused() {
    let img = seal(3 * SECTOR, 7, false);
    for cut in [1, 4, 88, 92, SECTOR, img.len() - 2 * SECTOR, img.len() - 2 * SECTOR + 4] {
        assert!(check(&img[..img.len() - cut]).is_err(), "cut of {cut} accepted");
    }
    let mut padded = img.clone();
    padded.extend([0u8; 4]);
    assert_eq!(check(&padded), Err(ImageError::LengthMismatch));
    padded.extend([0xffu8; 252]);
    assert_eq!(check(&padded), Err(ImageError::LengthMismatch));
    assert_eq!(check(&img[..SECTOR]), Err(ImageError::TooSmall));
    assert_eq!(check(&img[..img.len() - 1]), Err(ImageError::LengthNotWordMultiple));
}

#[test]
fn images_for_something_else_are_refused() {
    let body = 2 * SECTOR;
    // An unsealed image (what `objcopy -O binary` gives): the first block loops to itself.
    assert_eq!(check(&raw(body, 1)), Err(ImageError::LengthMismatch));
    // Not an RP2350 image at all (an RP2040 or ESP32 binary, a UF2, a text file).
    let junk = vec![0x55u8; body];
    assert_eq!(check(&junk), Err(ImageError::NoFirstBlock));
    let mut uf2 = vec![0u8; body];
    uf2[..8].copy_from_slice(&[0x55, 0x46, 0x32, 0x0a, 0x57, 0x51, 0x5d, 0x9e]);
    assert_eq!(check(&uf2), Err(ImageError::NoFirstBlock));
    // Wrong chip or architecture in the first block: RP2040, RISC-V, Non-secure, data image.
    for flags in [0x0021u32, 0x1121, 0x1011, 0x1022, 0x2021] {
        let mut r = raw(body, 1);
        put(&mut r, FIRST_BLOCK + 4, 0x0142 | flags << 16);
        assert_eq!(check(&seal_raw(r, 1, false)), Err(ImageError::WrongChip), "flags {flags:#x}");
    }
    // A sealed RP2350 Arm image without the tag (say, a Pico 2 program).
    let mut r = raw(body, 1);
    r[FIRST_BLOCK + 20..FIRST_BLOCK + 52].fill(0);
    assert_eq!(check(&seal_raw(r, 1, false)), Err(ImageError::NotTimerImage));
    // Timer firmware for another board, or another flash layout.
    let mut r = raw(body, 1);
    r[FIRST_BLOCK + 20 + 8] ^= 0x01;
    assert_eq!(check(&seal_raw(r, 1, false)), Err(ImageError::WrongBoard));
    let mut r = raw(body, 1);
    r[FIRST_BLOCK + 20 + 24] = (LAYOUT_REV + 1) as u8;
    assert_eq!(check(&seal_raw(r, 1, false)), Err(ImageError::WrongBoard));
    // Too large for a slot.
    let big = seal((MAX_IMAGE_BYTES + 4 - END_BLOCK_BYTES) as usize, 1, false);
    assert_eq!(check(&big), Err(ImageError::TooLarge));
    assert!(check(&seal((MAX_IMAGE_BYTES - END_BLOCK_BYTES) as usize, 1, false)).is_ok());
}

#[test]
fn end_blocks_of_any_other_shape_are_refused() {
    let img = seal(2 * SECTOR, 9, false);
    let end = img.len() - END_BLOCK_BYTES as usize;
    // A correct hash does not help if the load map does not cover exactly [0, end).
    for (word, value) in [(5usize, 4u32), (6, XIP_BASE + 4), (7, end as u32 - 4), (9, 9), (3, 8), (20, 0)] {
        let mut bad = img.clone();
        put(&mut bad, end + word * 4, value);
        let e = check(&bad).unwrap_err();
        assert!(matches!(e, ImageError::BadEndBlock | ImageError::HashMismatch), "word {word}: {e:?}");
    }
    // An unhashed end block (a plain second IMAGE_DEF) is not enough.
    let mut r = raw(2 * SECTOR, 9);
    let e = r.len();
    put(&mut r, FIRST_BLOCK + 12, (e - FIRST_BLOCK) as u32);
    for w in [BLOCK_MARKER_START, 0x0142 | (IMAGE_FLAGS as u32) << 16, 0x01ff, (FIRST_BLOCK as i64 - e as i64) as u32, BLOCK_MARKER_END] {
        r.extend(w.to_le_bytes());
    }
    r.resize(e + END_BLOCK_BYTES as usize, 0);
    assert_eq!(check(&r), Err(ImageError::BadEndBlock));
}

#[test]
fn the_real_tool_output_verifies() {
    let Ok(path) = std::env::var("OFT_FW_IMAGE") else {
        eprintln!("OFT_FW_IMAGE not set: skipping the check of a real tools/uf2.py image");
        return;
    };
    let img = std::fs::read(&path).unwrap();
    let info = check(&img).unwrap_or_else(|e| panic!("{path}: {e:?} ({})", e.as_str()));
    assert!(!info.tbyb, "tools/uf2.py must not set the try-before-you-buy bit");
    println!("{path}: {} bytes, build {}, sha256 {:02x?}", info.len, info.build, info.hash);
    // The USB file made alongside it is not an image and must be refused untouched.
    let uf2 = std::fs::read(std::path::Path::new(&path).with_extension("uf2")).unwrap();
    let mut flash = Fake::new();
    assert!(matches!(upload(&mut flash, &uf2[..uf2.len().min(MAX_IMAGE_BYTES as usize)], 1024), Err(UpdateError::Image(_))));
    assert!(flash.log.is_empty());
    // And it goes through the receiver.
    let mut flash = Fake::new();
    let got = upload(&mut flash, &img, 1024).unwrap();
    assert_eq!(got, ImageInfo { tbyb: true, ..info });
}

// ------------------------------------------------------------------ receiver

#[derive(Clone, Debug, PartialEq)]
enum Op {
    Erase(u32),
    Program(u32),
}

/// A slot of NOR flash: erase sets a sector to 0xff, programming can only clear bits.
#[derive(Clone)]
struct Fake {
    mem: Vec<u8>,
    log: Vec<Op>,
    /// Stop (as a power cut would) after this many page-sized steps; erase counts as one.
    budget: Option<usize>,
    dead: bool,
    /// Flip a bit in this byte when it is programmed.
    corrupt_at: Option<u32>,
}

impl Fake {
    fn new() -> Self {
        Self { mem: vec![0xa5; SLOT_BYTES as usize], log: vec![], budget: None, dead: false, corrupt_at: None }
    }

    fn step(&mut self) -> bool {
        match &mut self.budget {
            Some(0) => {
                self.dead = true;
                false
            }
            Some(n) => {
                *n -= 1;
                true
            }
            None => true,
        }
    }
}

impl SlotFlash for Fake {
    type Error = &'static str;

    fn erase_sector(&mut self, offset: u32) -> Result<(), Self::Error> {
        assert!(offset % 4096 == 0 && offset + 4096 <= SLOT_BYTES);
        if self.dead || !self.step() {
            return Err("power cut");
        }
        self.log.push(Op::Erase(offset));
        self.mem[offset as usize..offset as usize + SECTOR].fill(0xff);
        Ok(())
    }

    fn program_sector(&mut self, offset: u32, data: &[u8; SECTOR]) -> Result<(), Self::Error> {
        assert!(offset % 4096 == 0 && offset + 4096 <= SLOT_BYTES);
        if self.dead {
            return Err("power cut");
        }
        self.log.push(Op::Program(offset));
        for (p, page) in data.chunks(256).enumerate() {
            if !self.step() {
                return Err("power cut");
            }
            for (i, b) in page.iter().enumerate() {
                let at = offset as usize + p * 256 + i;
                self.mem[at] &= b;
                if self.corrupt_at == Some(at as u32) {
                    self.mem[at] ^= 0x10;
                }
            }
        }
        Ok(())
    }

    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        if self.dead {
            return Err("power cut");
        }
        buf.copy_from_slice(&self.mem[offset as usize..offset as usize + buf.len()]);
        Ok(())
    }
}

fn upload(flash: &mut Fake, file: &[u8], chunk: usize) -> Result<ImageInfo, UpdateError<&'static str>> {
    let mut bufs = Box::new(Buffers::new());
    let mut rx = Receiver::new(file.len() as u32);
    for c in file.chunks(chunk) {
        rx.push(&mut bufs, flash, c)?;
    }
    rx.finish(&mut bufs, flash)
}

#[test]
fn an_upload_lands_in_the_slot_marked_try_before_you_buy() {
    for body in [2 * SECTOR, 2 * SECTOR + 400, 3 * SECTOR - 88, 3 * SECTOR - 84, 3 * SECTOR - 4, 59 * SECTOR + 268] {
        for chunk in [1usize, 7, 536, 1024, 4096, 5000, 1 << 20] {
            if chunk == 1 && body > 4 * SECTOR {
                continue;
            }
            for tbyb_in_file in [false, true] {
                let file = seal(body, 42, tbyb_in_file);
                let mut flash = Fake::new();
                let info = upload(&mut flash, &file, chunk).unwrap();
                assert_eq!((info.len, info.build, info.tbyb), (file.len() as u32, 42, true));
                // The slot holds the file with the bit set, and passes as the boot ROM reads it.
                assert_eq!(flash.mem[..file.len()], seal(body, 42, true)[..]);
                let mut scratch = [0u8; SECTOR];
                assert_eq!(verify_slot(&mut &flash.mem[..], &mut scratch), Ok(info));
                // Sector 0 is erased before anything else and written after everything else.
                assert_eq!(flash.log.first(), Some(&Op::Erase(0)));
                assert_eq!(flash.log.last(), Some(&Op::Program(0)));
                assert_eq!(flash.log.iter().filter(|o| matches!(o, Op::Erase(0) | Op::Program(0))).count(), 2);
                // Nothing past the image's last sector is touched.
                let last = (file.len() as u32 - 1) / 4096 * 4096;
                assert!(flash.log.iter().all(|o| matches!(o, Op::Erase(a) | Op::Program(a) if *a <= last)));
                assert!(flash.mem[(last as usize + SECTOR)..].iter().all(|&b| b == 0xa5));
            }
        }
    }
}

#[test]
fn a_wrong_file_changes_nothing() {
    let good = seal(3 * SECTOR, 5, false);
    let mut cases: Vec<(Vec<u8>, &str)> = vec![];
    cases.push((vec![0x55; 3 * SECTOR], "junk"));
    cases.push((raw(3 * SECTOR, 5), "unsealed"));
    cases.push((good[..good.len() - SECTOR].to_vec(), "truncated"));
    let mut longer = good.clone();
    longer.extend([0u8; 512]);
    cases.push((longer, "padded"));
    let mut other_board = raw(3 * SECTOR, 5);
    other_board[FIRST_BLOCK + 20 + 9] = b'X';
    cases.push((seal_raw(other_board, 5, false), "other board"));
    let mut rp2040 = raw(3 * SECTOR, 5);
    put(&mut rp2040, FIRST_BLOCK + 4, 0x0142 | 0x0021 << 16);
    cases.push((seal_raw(rp2040, 5, false), "rp2040"));
    cases.push((good[..SECTOR + 4].to_vec(), "tiny"));
    for (file, name) in cases {
        let mut flash = Fake::new();
        let r = upload(&mut flash, &file, 1000);
        assert!(matches!(r, Err(UpdateError::Image(_))), "{name}: {r:?}");
        assert!(flash.log.is_empty(), "{name}: flash was touched: {:?}", flash.log);
    }
}

#[test]
fn a_file_damaged_past_the_first_sector_is_stored_but_never_completed() {
    let mut file = seal(4 * SECTOR, 5, false);
    file[2 * SECTOR + 17] ^= 0x04;
    let mut flash = Fake::new();
    assert_eq!(upload(&mut flash, &file, 1024), Err(UpdateError::Image(ImageError::HashMismatch)));
    assert!(!flash.log.contains(&Op::Program(0)), "sector 0 must not be written");
    assert!(flash.mem[..SECTOR].iter().all(|&b| b == 0xff), "the slot is left without a first sector");
}

#[test]
fn a_flash_fault_is_caught_by_reading_back() {
    let file = seal(4 * SECTOR, 5, false);
    for at in [SECTOR as u32 + 3, 3 * SECTOR as u32 + 999, 4 * SECTOR as u32 + 50, 100] {
        let mut flash = Fake::new();
        flash.corrupt_at = Some(at);
        assert_eq!(upload(&mut flash, &file, 1024), Err(UpdateError::Readback), "fault at {at:#x}");
    }
}

#[test]
fn the_announced_length_is_binding() {
    let file = seal(3 * SECTOR, 5, false);
    let mut bufs = Box::new(Buffers::new());
    // Stops early.
    let mut flash = Fake::new();
    let mut rx = Receiver::new(file.len() as u32);
    rx.push(&mut bufs, &mut flash, &file[..file.len() - 10]).unwrap();
    assert_eq!(rx.finish(&mut bufs, &mut flash), Err(UpdateError::Incomplete));
    assert!(!flash.log.contains(&Op::Program(0)));
    // Sends more.
    let mut flash = Fake::new();
    let mut rx = Receiver::new(file.len() as u32);
    rx.push(&mut bufs, &mut flash, &file).unwrap();
    assert_eq!(rx.push(&mut bufs, &mut flash, &[0]), Err(UpdateError::TooMuchData));
    assert_eq!(rx.finish(&mut bufs, &mut flash), Err(UpdateError::Incomplete));
    assert!(!flash.log.contains(&Op::Program(0)));
    // Nothing at all.
    let mut flash = Fake::new();
    assert_eq!(Receiver::new(file.len() as u32).finish(&mut bufs, &mut flash), Err(UpdateError::Incomplete));
    assert!(flash.log.is_empty());
}

// ------------------------------------------------------------------ boot ROM model, power cuts

#[derive(Clone, Copy)]
struct Seen {
    build: u32,
    tbyb: bool,
    verified: bool,
}

/// What the boot ROM finds in a slot: nothing, or a block loop that may or may not verify.
fn look(slot: &[u8]) -> Option<Seen> {
    let mut scratch = [0u8; SECTOR];
    let p = probe(&mut &slot[..], &mut scratch).ok()?;
    let verified = verify_slot(&mut &slot[..], &mut scratch).is_ok();
    Some(Seen { build: p.build, tbyb: p.tbyb, verified })
}

/// Which slot the boot ROM starts. `update` is the slot named in a flash-update reboot.
fn rom_boot(a: &[u8], b: &[u8], update: Option<Slot>) -> Option<Slot> {
    let (x, y) = (look(a), look(b));
    let mut chosen = Slot::A;
    if let Some(y) = y {
        let greater_y = x.map_or(true, |x| y.build > x.build);
        let consider_y;
        match x {
            Some(x) if update == Some(Slot::A) => consider_y = !x.verified,
            _ => {
                let mut use_y = update == Some(Slot::B) || x.is_some_and(|x| x.tbyb);
                if !use_y {
                    if greater_y && !y.tbyb {
                        use_y = true;
                    } else if let Some(x) = x {
                        use_y = !x.verified;
                    }
                }
                consider_y = use_y;
            }
        }
        if consider_y && y.verified {
            chosen = Slot::B;
        }
    }
    // Launch: the chosen image must verify, and a try-before-you-buy image only starts when
    // it is the flash update.
    let c = match chosen {
        Slot::A => x,
        Slot::B => y,
    }?;
    (c.verified && (!c.tbyb || update == Some(chosen))).then_some(chosen)
}

/// The boot ROM's `explicit_buy`, with a step budget: rewrite the sector holding the
/// try-before-you-buy bit with the bit clear, then (if the ROM marked it at boot) erase the
/// first sector of the other slot. Returns false if power was cut.
fn rom_buy(new: &mut Fake, old: &mut Fake, erase_other: bool) -> bool {
    let mut scratch = [0u8; SECTOR];
    let info = verify_slot(&mut &new.mem[..], &mut scratch).unwrap();
    let at = info.len - END_BLOCK_BYTES + END_BLOCK_TBYB_BYTE;
    let sector = at / 4096 * 4096;
    let mut data = [0u8; SECTOR];
    data.copy_from_slice(&new.mem[sector as usize..sector as usize + SECTOR]);
    data[(at - sector) as usize] &= 0x7f;
    if new.erase_sector(sector).is_err() || new.program_sector(sector, &data).is_err() {
        return false;
    }
    if erase_other {
        old.budget = new.budget;
        if old.erase_sector(0).is_err() {
            return false;
        }
    }
    true
}

fn slot_with(img: &[u8]) -> Fake {
    let mut f = Fake::new();
    f.mem[..img.len()].copy_from_slice(img);
    f
}

#[test]
fn the_model_agrees_with_the_documented_rules() {
    let old = slot_with(&seal(3 * SECTOR, 10, false)).mem;
    let newer = slot_with(&seal(3 * SECTOR, 11, false)).mem;
    let trial = slot_with(&seal(3 * SECTOR, 11, true)).mem;
    let blank = vec![0xffu8; SLOT_BYTES as usize];
    // Higher version wins; A wins a tie; a blank or broken slot loses.
    assert_eq!(rom_boot(&old, &newer, None), Some(Slot::B));
    assert_eq!(rom_boot(&newer, &old, None), Some(Slot::A));
    assert_eq!(rom_boot(&old, &old, None), Some(Slot::A));
    assert_eq!(rom_boot(&blank, &old, None), Some(Slot::B));
    assert_eq!(rom_boot(&old, &blank, None), Some(Slot::A));
    assert_eq!(rom_boot(&blank, &blank, None), None);
    // A try-before-you-buy image is ignored on a normal boot and tried on a flash update.
    assert_eq!(rom_boot(&old, &trial, None), Some(Slot::A));
    assert_eq!(rom_boot(&trial, &old, None), Some(Slot::B));
    assert_eq!(rom_boot(&old, &trial, Some(Slot::B)), Some(Slot::B));
    assert_eq!(rom_boot(&trial, &old, Some(Slot::A)), Some(Slot::A));
    assert_eq!(rom_boot(&trial, &blank, None), None);
    // A flash update of a lower version still starts it.
    assert_eq!(rom_boot(&newer, &slot_with(&seal(3 * SECTOR, 3, true)).mem, Some(Slot::B)), Some(Slot::B));
}

/// Cut power after every flash step of an update, in both directions and for an upgrade, a
/// downgrade and a re-install of the same build: the running firmware must still be what
/// the boot ROM starts. Then cut power at every step of the confirmation: some complete
/// firmware must start, and once confirmed the new one must.
#[test]
fn power_cut_at_any_step_leaves_a_bootable_board() {
    let body = 3 * SECTOR + 1000;
    for running in [Slot::A, Slot::B] {
        for (old_build, new_build) in [(10u32, 11u32), (11, 10), (10, 10)] {
            for spare_before in ["blank", "older image", "unbought image"] {
                let old_img = seal(body, old_build, false);
                let file = seal(body + 512, new_build, false);
                let run = slot_with(&old_img);
                let spare = match spare_before {
                    "blank" => Fake::new(),
                    "older image" => slot_with(&seal(body, 4, false)),
                    _ => slot_with(&seal(body, 99, true)),
                };
                let boot = |run: &Fake, spare: &Fake, update: Option<Slot>| match running {
                    Slot::A => rom_boot(&run.mem, &spare.mem, update),
                    Slot::B => rom_boot(&spare.mem, &run.mem, update),
                };
                let ctx = format!("running {running:?}, {old_build} -> {new_build}, spare {spare_before}");

                // How many steps does a complete upload take?
                let mut full = spare.clone();
                full.budget = Some(usize::MAX);
                upload(&mut full, &file, 1460).unwrap();
                let steps = usize::MAX - full.budget.unwrap();
                assert!(steps > 60);

                for cut in 0..steps {
                    let mut s = spare.clone();
                    s.budget = Some(cut);
                    assert!(upload(&mut s, &file, 1460).is_err(), "{ctx}: cut {cut}");
                    assert_eq!(boot(&run, &s, None), Some(running), "{ctx}: power cut after {cut} flash steps of the upload");
                    // Even a flash-update reboot aimed at the spare slot (which the firmware
                    // never requests for a failed upload) cannot start a partial image: once
                    // the slot has been touched it falls back to the running firmware. Before
                    // that the slot still holds whatever complete image it held.
                    let aimed = boot(&run, &s, Some(running.other()));
                    if s.log.is_empty() {
                        assert_eq!(aimed, boot(&run, &spare, Some(running.other())), "{ctx}: untouched spare, cut {cut}");
                    } else {
                        assert_eq!(aimed, Some(running), "{ctx}: update boot after cut {cut}");
                    }
                }

                // Upload complete, power lost before or during the switch: a normal boot.
                full.budget = None;
                assert_eq!(boot(&run, &full, None), Some(running), "{ctx}: power cut before the switch");
                // The switch: the boot ROM tries the new image.
                assert_eq!(boot(&run, &full, Some(running.other())), Some(running.other()), "{ctx}: the trial boot");
                // The trial is never confirmed (no Ethernet, crash, power cut): watchdog, normal boot.
                assert_eq!(boot(&run, &full, None), Some(running), "{ctx}: after an unconfirmed trial");

                // The ROM marks the old slot for erasure when the new image would not win a
                // normal boot by version: new in A loses only to a strictly greater B; new in
                // B loses to an equal or greater A (see the choose function).
                let erase_other = match running {
                    Slot::B => old_build > new_build,
                    Slot::A => new_build <= old_build,
                };
                let buy_steps = 1 + 16 + erase_other as usize;
                for cut in 0..=buy_steps {
                    let (mut n, mut o) = (full.clone(), run.clone());
                    n.budget = Some(cut);
                    let done = rom_buy(&mut n, &mut o, erase_other);
                    assert_eq!(done, cut == buy_steps, "{ctx}: buy cut {cut}");
                    let booted = boot(&o, &n, None);
                    assert!(booted.is_some(), "{ctx}: power cut after {cut} steps of the confirmation leaves nothing bootable");
                    if done {
                        assert_eq!(booted, Some(running.other()), "{ctx}: the confirmed image must be the one that starts");
                    }
                }
            }
        }
    }
}
