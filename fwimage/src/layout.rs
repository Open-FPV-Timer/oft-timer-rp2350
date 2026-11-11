//! Flash layout of the Timer and the running-slot check.
//!
//! ```text
//! 0x000000  4 K    partition table (boot ROM "slot 0")
//! 0x001000  4 K    left erased (boot ROM "slot 1", a second partition table would go here)
//! 0x002000  2 M    partition 0 = firmware slot A
//! 0x202000  2 M    partition 1 = firmware slot B (linked to A as its "B" partition)
//! 0x402000  ...    not partitioned, unused
//! ```
//!
//! `tools/uf2.py` writes the same numbers into the partition table; `LAYOUT_REV` is in the
//! tag every image carries, so an image built for another layout is refused.

/// Where flash appears when the boot ROM has mapped the running slot to the start.
pub const XIP_BASE: u32 = 0x1000_0000;
/// The same flash without address translation (and without the cache): offset = flash offset.
pub const XIP_NOTRANSLATE_BASE: u32 = 0x1C00_0000;

pub const FLASH_BYTES: u32 = 16 * 1024 * 1024;
pub const SECTOR_BYTES: u32 = 4096;

pub const SLOT_A_OFFSET: u32 = 0x0000_2000;
pub const SLOT_BYTES: u32 = 0x0020_0000;
pub const SLOT_B_OFFSET: u32 = SLOT_A_OFFSET + SLOT_BYTES;

/// Largest image accepted. One sector short of the slot: the boot ROM refuses a hashed range
/// that reaches the end of its partition, and `memory.x` gives the linker the same limit.
pub const MAX_IMAGE_BYTES: u32 = SLOT_BYTES - SECTOR_BYTES;
/// Smallest image accepted: the first sector is handled separately from the rest.
pub const MIN_IMAGE_BYTES: u32 = 2 * SECTOR_BYTES;

/// Bumped when the numbers above change.
pub const LAYOUT_REV: u32 = 1;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Slot {
    A,
    B,
}

impl Slot {
    /// Offset of the slot from the start of flash.
    pub const fn offset(self) -> u32 {
        match self {
            Slot::A => SLOT_A_OFFSET,
            Slot::B => SLOT_B_OFFSET,
        }
    }

    pub const fn other(self) -> Slot {
        match self {
            Slot::A => Slot::B,
            Slot::B => Slot::A,
        }
    }

    /// Index in the partition table.
    pub const fn partition(self) -> i8 {
        match self {
            Slot::A => 0,
            Slot::B => 1,
        }
    }

    pub const fn name(self) -> &'static str {
        match self {
            Slot::A => "A",
            Slot::B => "B",
        }
    }

    /// The value the boot ROM leaves in QMI `ATRANS0` after starting this slot: bits 11:0
    /// are the first sector, bits 26:16 the mapped size in sectors (at most 4 MB per window).
    pub const fn atrans0(self) -> u32 {
        ((SLOT_BYTES / SECTOR_BYTES) << 16) | (self.offset() / SECTOR_BYTES)
    }
}

/// `ATRANS0` fields: where the first 4 MB of the flash window really starts, and how much
/// of it is mapped.
const ATRANS_BASE: u32 = 0x0000_0fff;
const ATRANS_SIZE: u32 = 0x07ff_0000;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LayoutError {
    /// The boot ROM did not start the firmware from a partition.
    NoPartitionTable,
    /// Started from a partition that is not slot A or B.
    UnknownPartition,
    /// The flash mapping does not match the slot the boot ROM reported.
    MappingMismatch,
}

impl LayoutError {
    pub const fn as_str(self) -> &'static str {
        match self {
            LayoutError::NoPartitionTable => "this board has no A/B partition table yet: flash oft-timer.uf2 over USB once",
            LayoutError::UnknownPartition => "started from an unknown partition: flash oft-timer.uf2 over USB",
            LayoutError::MappingMismatch => "flash mapping does not match the expected A/B layout: flash oft-timer.uf2 over USB",
        }
    }
}

/// Which slot the firmware is running from, from two independent sources that must agree:
/// the partition index the boot ROM reports and the address translation it set up.
pub fn running_slot(boot_partition: i8, atrans0: u32) -> Result<Slot, LayoutError> {
    let slot = match boot_partition {
        0 => Slot::A,
        1 => Slot::B,
        p if p < 0 => return Err(LayoutError::NoPartitionTable),
        _ => return Err(LayoutError::UnknownPartition),
    };
    // The base is what matters: it is where the running code physically is. The size is
    // only required to be non-zero (something is mapped).
    if atrans0 & ATRANS_BASE != slot.atrans0() & ATRANS_BASE || atrans0 & ATRANS_SIZE == 0 {
        return Err(LayoutError::MappingMismatch);
    }
    Ok(slot)
}

/// Boot ROM `get_sys_info(SYS_INFO_BOOT_INFO)` result (pico-sdk `boot_info_t`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct BootInfo {
    /// The ROM's diagnostic word for this boot (`BOOT_DIAGNOSTIC_*` in the pico-sdk), raw.
    pub diagnostic: u32,
    pub boot_type: u8,
    /// Partition the firmware was started from, negative if none.
    pub partition: i8,
    pub tbyb_and_update: u8,
    pub reboot_params: [u32; 2],
}

pub const SYS_INFO_BOOT_INFO: u32 = 0x0040;
/// `tbyb_and_update` bit: started on trial, `explicit_buy` has not been called yet.
pub const TBYB_BUY_PENDING: u8 = 0x01;
pub const BOOT_TYPE_FLASH_UPDATE: u8 = 4;

impl BootInfo {
    /// `words` is the buffer `get_sys_info` filled and `returned` its return value.
    /// Word 0 echoes the flags; word 1 packs diagnostic partition, boot type, partition and
    /// the try-before-you-buy flags, one byte each from the low end.
    pub fn from_sys_info(words: &[u32; 5], returned: i32) -> Option<BootInfo> {
        if returned != 5 || words[0] != SYS_INFO_BOOT_INFO {
            return None;
        }
        let w = words[1];
        Some(BootInfo { diagnostic: words[2], boot_type: (w >> 8) as u8, partition: (w >> 16) as u8 as i8, tbyb_and_update: (w >> 24) as u8, reboot_params: [words[3], words[4]] })
    }

    pub fn buy_pending(&self) -> bool {
        self.tbyb_and_update & TBYB_BUY_PENDING != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slots_are_sector_aligned_inside_flash_and_do_not_overlap() {
        for s in [Slot::A, Slot::B] {
            assert_eq!(s.offset() % SECTOR_BYTES, 0);
            assert!(s.offset() >= 2 * SECTOR_BYTES, "the first two sectors belong to the partition table");
            assert!(s.offset() + SLOT_BYTES <= FLASH_BYTES);
        }
        assert_eq!(SLOT_BYTES % SECTOR_BYTES, 0);
        assert!(Slot::A.offset() + SLOT_BYTES <= Slot::B.offset());
        assert_eq!(Slot::A.other(), Slot::B);
        assert_eq!(Slot::B.other(), Slot::A);
        // One address-translation window covers at most 4 MB; the check reads only window 0.
        assert!(SLOT_BYTES <= 4 * 1024 * 1024);
    }

    #[test]
    fn atrans_values() {
        // size 0x200 sectors (2 MB), base sector 2 / 0x202.
        assert_eq!(Slot::A.atrans0(), 0x0200_0002);
        assert_eq!(Slot::B.atrans0(), 0x0200_0202);
    }

    #[test]
    fn running_slot_needs_both_sources_to_agree() {
        assert_eq!(running_slot(0, 0x0200_0002), Ok(Slot::A));
        assert_eq!(running_slot(1, 0x0200_0202), Ok(Slot::B));
        // Reserved bits set by hardware do not matter.
        assert_eq!(running_slot(1, 0xf800_f000 | 0x0200_0202), Ok(Slot::B));
        // No partition table: the ROM reports -1 and leaves the reset mapping (4 MB at 0).
        assert_eq!(running_slot(-1, 0x0400_0000), Err(LayoutError::NoPartitionTable));
        assert_eq!(running_slot(0, 0x0400_0000), Err(LayoutError::MappingMismatch));
        // A different mapped size is tolerated, an empty mapping is not.
        assert_eq!(running_slot(0, 0x0040_0002), Ok(Slot::A));
        assert_eq!(running_slot(0, 0x0000_0002), Err(LayoutError::MappingMismatch));
        assert_eq!(running_slot(0, 0x0200_0202), Err(LayoutError::MappingMismatch));
        assert_eq!(running_slot(1, 0x0200_0002), Err(LayoutError::MappingMismatch));
        assert_eq!(running_slot(2, 0x0200_0002), Err(LayoutError::UnknownPartition));
    }

    #[test]
    fn boot_info_bytes() {
        // diagnostic partition 0xfe, boot type 4 (flash update), partition 1, buy pending.
        let words = [0x40, 0x0101_04fe, 0x1234, 0x1020_2000, 0];
        let b = BootInfo::from_sys_info(&words, 5).unwrap();
        assert_eq!(b.boot_type, BOOT_TYPE_FLASH_UPDATE);
        assert_eq!(b.partition, 1);
        assert!(b.buy_pending());
        assert_eq!(b.reboot_params, [0x1020_2000, 0]);
        assert_eq!(b.diagnostic, 0x1234);
        let none = BootInfo::from_sys_info(&[0x40, 0x00ff_0000, 0, 0, 0], 5).unwrap();
        assert_eq!(none.partition, -1);
        assert!(!none.buy_pending());
        assert_eq!(BootInfo::from_sys_info(&words, 4), None);
        assert_eq!(BootInfo::from_sys_info(&[0, 0, 0, 0, 0], 5), None);
    }
}
