#!/usr/bin/env python3
"""ELF -> UF2 for the RP2350 (ARM secure family), no external tools beyond llvm-objcopy.

Usage: tools/uf2.py target/thumbv8m.main-none-eabihf/release/oft-timer-rp2350 oft-timer.uf2
"""
import struct
import subprocess
import sys
from pathlib import Path

FAMILY_RP2350_ARM_S = 0xE48BFF59
FLASH_BASE = 0x10000000
MAGIC0, MAGIC1, MAGIC_END = 0x0A324655, 0x9E5D5157, 0x0AB16F30
FLAG_FAMILY_ID = 0x00002000


def objcopy():
    sysroot = subprocess.check_output(["rustc", "--print", "sysroot"], text=True).strip()
    for p in Path(sysroot).glob("lib/rustlib/*/bin/llvm-objcopy"):
        return str(p)
    sys.exit("llvm-objcopy not found: rustup component add llvm-tools")


def main(elf, out):
    binpath = Path(out).with_suffix(".bin")
    subprocess.check_call([objcopy(), "-O", "binary", elf, str(binpath)])
    data = binpath.read_bytes()
    blocks = (len(data) + 255) // 256
    with open(out, "wb") as f:
        for i in range(blocks):
            chunk = data[i * 256 : (i + 1) * 256].ljust(256, b"\0")
            hdr = struct.pack("<IIIIIIII", MAGIC0, MAGIC1, FLAG_FAMILY_ID, FLASH_BASE + i * 256, 256, i, blocks, FAMILY_RP2350_ARM_S)
            f.write(hdr + chunk.ljust(476, b"\0") + struct.pack("<I", MAGIC_END))
    print(f"{out}: {len(data)} bytes of flash image in {blocks} blocks")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
