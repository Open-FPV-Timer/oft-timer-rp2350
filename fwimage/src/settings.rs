//! Where the Timer keeps its saved settings, and the record format.
//!
//! Two flash sectors straight after firmware slot B hold one record each. A save goes to the
//! copy that is *not* the newest, so a power cut during a save leaves the previous record
//! intact. Both firmware slots read the same sectors, so settings survive an update.
//!
//! ```text
//! record:  "OFTS"  seq:u32  len:u32  crc32:u32  payload[len]   (little endian, rest 0xff)
//! ```
//!
//! The CRC covers `seq`, `len` and the payload. The payload is opaque here (the firmware
//! stores JSON).

use crate::layout::{SECTOR_BYTES, SLOT_B_OFFSET, SLOT_BYTES};

pub const SECTOR: usize = SECTOR_BYTES as usize;
pub const COPIES: usize = 2;
/// Flash offset of the first copy; the second follows it.
pub const SETTINGS_OFFSET: u32 = SLOT_B_OFFSET + SLOT_BYTES;
const MAGIC: [u8; 4] = *b"OFTS";
const HEADER: usize = 16;
pub const MAX_PAYLOAD: usize = SECTOR - HEADER;

pub const fn copy_offset(copy: usize) -> u32 {
    SETTINGS_OFFSET + (copy as u32) * SECTOR_BYTES
}

/// CRC-32 (IEEE, reflected), bit by bit: a few kilobytes at most, once per save or boot.
pub fn crc32(parts: &[&[u8]]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    for part in parts {
        for &b in *part {
            crc ^= b as u32;
            for _ in 0..8 {
                crc = if crc & 1 != 0 { (crc >> 1) ^ 0xedb8_8320 } else { crc >> 1 };
            }
        }
    }
    !crc
}

/// Fill `sector` with a record. `Err` if the payload does not fit.
pub fn encode(seq: u32, payload: &[u8], sector: &mut [u8; SECTOR]) -> Result<(), ()> {
    if payload.len() > MAX_PAYLOAD {
        return Err(());
    }
    sector.fill(0xff);
    let len = (payload.len() as u32).to_le_bytes();
    let seq = seq.to_le_bytes();
    sector[0..4].copy_from_slice(&MAGIC);
    sector[4..8].copy_from_slice(&seq);
    sector[8..12].copy_from_slice(&len);
    sector[12..16].copy_from_slice(&crc32(&[&seq, &len, payload]).to_le_bytes());
    sector[HEADER..HEADER + payload.len()].copy_from_slice(payload);
    Ok(())
}

/// The record in `sector`, if it holds a complete one: `(seq, payload)`.
pub fn decode(sector: &[u8]) -> Option<(u32, &[u8])> {
    if sector.len() < HEADER || sector[0..4] != MAGIC {
        return None;
    }
    let word = |at: usize| u32::from_le_bytes([sector[at], sector[at + 1], sector[at + 2], sector[at + 3]]);
    let (seq, len, crc) = (word(4), word(8) as usize, word(12));
    if len > MAX_PAYLOAD || HEADER + len > sector.len() {
        return None;
    }
    let payload = &sector[HEADER..HEADER + len];
    (crc32(&[&sector[4..8], &sector[8..12], payload]) == crc).then_some((seq, payload))
}

/// Given the sequence number of each copy (if valid): which copy is the newest, which copy
/// the next save goes to, and the sequence number to give it.
pub fn choose(seqs: [Option<u32>; COPIES]) -> (Option<usize>, usize, u32) {
    match seqs {
        [None, None] => (None, 0, 1),
        [Some(a), None] => (Some(0), 1, a.wrapping_add(1)),
        [None, Some(b)] => (Some(1), 0, b.wrapping_add(1)),
        // Wrapping compare, so the order survives the counter rolling over.
        [Some(a), Some(b)] => {
            if (a.wrapping_sub(b) as i32) >= 0 {
                (Some(0), 1, a.wrapping_add(1))
            } else {
                (Some(1), 0, b.wrapping_add(1))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_the_standard_check_value() {
        assert_eq!(crc32(&[b"123456789"]), 0xcbf4_3926);
        assert_eq!(crc32(&[b"1234", b"56789"]), 0xcbf4_3926);
    }

    #[test]
    fn round_trip() {
        let mut s = [0u8; SECTOR];
        encode(7, b"{\"a\":1}", &mut s).unwrap();
        assert_eq!(decode(&s), Some((7, &b"{\"a\":1}"[..])));
        encode(8, &[], &mut s).unwrap();
        assert_eq!(decode(&s), Some((8, &[][..])));
        let big = [0x55u8; MAX_PAYLOAD];
        encode(9, &big, &mut s).unwrap();
        assert_eq!(decode(&s).unwrap().1.len(), MAX_PAYLOAD);
        assert!(encode(1, &[0u8; MAX_PAYLOAD + 1], &mut s).is_err());
    }

    #[test]
    fn erased_torn_and_corrupt_sectors_are_rejected() {
        assert_eq!(decode(&[0xffu8; SECTOR]), None);
        assert_eq!(decode(&[0u8; SECTOR]), None);
        let mut s = [0u8; SECTOR];
        encode(3, b"hello settings", &mut s).unwrap();
        // every single-byte change in the record is caught
        for at in 0..HEADER + 14 {
            let mut t = s;
            t[at] ^= 0x01;
            assert_eq!(decode(&t), None, "byte {at}");
        }
        // a write that stopped part way (flash programs in order, the rest stays 0xff)
        for cut in 1..HEADER + 14 {
            let mut t = [0xffu8; SECTOR];
            t[..cut].copy_from_slice(&s[..cut]);
            assert_eq!(decode(&t), None, "cut {cut}");
        }
        // a length field pointing past the sector
        let mut t = s;
        t[8..12].copy_from_slice(&(SECTOR as u32).to_le_bytes());
        assert_eq!(decode(&t), None);
    }

    #[test]
    fn saves_alternate_and_survive_a_power_cut() {
        assert_eq!(choose([None, None]), (None, 0, 1));
        assert_eq!(choose([Some(1), None]), (Some(0), 1, 2));
        assert_eq!(choose([Some(1), Some(2)]), (Some(1), 0, 3));
        assert_eq!(choose([Some(3), Some(2)]), (Some(0), 1, 4));
        assert_eq!(choose([None, Some(9)]), (Some(1), 0, 10));
        // across the counter rolling over
        assert_eq!(choose([Some(u32::MAX), Some(0)]), (Some(1), 0, 1));

        // Simulate saves with a cut during the erase or the write of each one: the newest
        // complete record is always still readable.
        let mut flash = [[0xffu8; SECTOR]; COPIES];
        let mut last_good: Option<u32> = None;
        for round in 0..6u32 {
            let seqs = [decode(&flash[0]).map(|r| r.0), decode(&flash[1]).map(|r| r.0)];
            let (newest, target, seq) = choose(seqs);
            assert_eq!(newest.map(|n| seqs[n].unwrap()), last_good);
            let mut rec = [0u8; SECTOR];
            encode(seq, &round.to_le_bytes(), &mut rec).unwrap();
            // cut after the erase
            flash[target] = [0xff; SECTOR];
            let after = [decode(&flash[0]).map(|r| r.0), decode(&flash[1]).map(|r| r.0)];
            assert_eq!(choose(after).0.map(|n| after[n].unwrap()), last_good);
            // cut half way through the write
            flash[target][..10].copy_from_slice(&rec[..10]);
            let after = [decode(&flash[0]).map(|r| r.0), decode(&flash[1]).map(|r| r.0)];
            assert_eq!(choose(after).0.map(|n| after[n].unwrap()), last_good);
            // complete
            flash[target] = rec;
            last_good = Some(seq);
        }
        assert_eq!(last_good, Some(6));
    }

    #[test]
    fn settings_sit_after_slot_b() {
        assert_eq!(SETTINGS_OFFSET, 0x0040_2000);
        assert_eq!(copy_offset(1), 0x0040_3000);
        assert!(copy_offset(COPIES) <= crate::layout::FLASH_BYTES);
    }
}
