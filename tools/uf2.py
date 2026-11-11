#!/usr/bin/env python3
"""Package the Timer firmware for the RP2350: no external tools beyond llvm-objcopy.

  tools/uf2.py ELF OUT.uf2 [--build N]    write OUT.uf2 (USB install) and OUT.bin (network update)
  tools/uf2.py ELF OUT.uf2 --flat         write a plain single-image UF2 (the pre-A/B layout);
                                          ELF may also be a raw .bin from an earlier build
  tools/uf2.py --inspect FILE             describe a .bin or .uf2 written by this tool

OUT.bin is the "sealed" firmware image: the linked binary plus an 88-byte block at the end
holding a build number and a SHA-256 of everything before it, in the format the RP2350 boot
ROM checks (what `picotool seal --hash` produces). Upload it to POST /oft/v1/fw/timer.

OUT.uf2 installs the A/B layout over USB in one file: a partition table at the start of
flash, the same sealed image in slot A, and slot B's first sector erased so that A is the
image that boots. It uses the UF2 "absolute" family, so every block lands at the address it
names whether or not the board already has a partition table.

Flash layout (keep in step with fwimage/src/layout.rs):
  0x000000  4 K   partition table
  0x001000  4 K   erased (the boot ROM's second partition-table slot)
  0x002000  2 M   partition 0, firmware slot A
  0x202000  2 M   partition 1, firmware slot B (the "B" partner of partition 0)
"""
import hashlib
import struct
import subprocess
import sys
import time
from pathlib import Path

FLASH_BASE = 0x10000000
SECTOR = 4096
PAGE = 256
SLOT_A = 0x002000
SLOT_BYTES = 0x200000
SLOT_B = SLOT_A + SLOT_BYTES
MAX_IMAGE = SLOT_BYTES - SECTOR
MIN_IMAGE = 2 * SECTOR
LAYOUT_REV = 1

# UF2 (https://github.com/microsoft/uf2) with the RP2350 boot ROM's family IDs.
UF2_MAGIC0, UF2_MAGIC1, UF2_MAGIC_END = 0x0A324655, 0x9E5D5157, 0x0AB16F30
UF2_FLAG_FAMILY_ID = 0x00002000
FAMILY_ABSOLUTE = 0xE48BFF57
FAMILY_NAMES = {0xE48BFF56: "rp2040", 0xE48BFF57: "absolute", 0xE48BFF58: "data", 0xE48BFF59: "rp2350-arm-s", 0xE48BFF5A: "rp2350-riscv", 0xE48BFF5B: "rp2350-arm-ns"}

# Boot ROM metadata blocks (RP2350 datasheet 5.9, pico-sdk boot/picobin.h).
BLOCK_START, BLOCK_END = 0xFFFFDED3, 0xAB123579
ITEM_IMAGE_TYPE, ITEM_VERSION, ITEM_LOAD_MAP, ITEM_HASH_DEF, ITEM_HASH_VALUE = 0x42, 0x48, 0x06, 0x47, 0x4B
ITEM_PARTITION_TABLE, ITEM_LAST = 0x0A, 0xFF
IMAGE_FLAGS = 0x1021  # executable, Secure, Arm, RP2350
IMAGE_FLAG_TBYB = 0x8000
END_BLOCK_WORDS = 22
MAX_IMAGE_DEF_WORDS = 0x180 // 4

# Partition table words (datasheet 5.9.4.2).
PERM_ALL = 0xFC000000  # read/write for Secure, Non-secure and the bootloader
ACCEPTS_ABSOLUTE = 0x00008000
ACCEPTS_RP2350_ARM_S = 0x00020000
LINK_TO_A_PARTITION = 1 << 1  # this is the "B" of the partition named in bits 6:3

TAG = b"OFTFWTAG" + b"rp2350-poe-eth\0\0" + struct.pack("<II", LAYOUT_REV, 0)
BUILD_EPOCH = 1577836800  # 2020-01-01 00:00:00 UTC


def words(data):
    return list(struct.unpack(f"<{len(data) // 4}I", data[: len(data) // 4 * 4]))


def to_bytes(ws):
    return struct.pack(f"<{len(ws)}I", *ws)


def item_size(header):
    return (header >> 8) & (0xFFFF if header & 0x80 else 0xFF)


def frame(ws):
    """The boot ROM's framing check. `ws` starts at a candidate block. Returns (index of the
    LAST item, link) or None."""
    ws = ws[:MAX_IMAGE_DEF_WORDS]
    if len(ws) < 5 or ws[0] != BLOCK_START:
        return None
    pos = 1
    while pos + 2 < len(ws):
        size = item_size(ws[pos])
        if size == 0:
            return None
        if ws[pos] & 0xFF == ITEM_LAST:
            if size != pos - 1 or ws[pos + 2] != BLOCK_END:
                return None
            link = ws[pos + 1]
            return pos, link - (1 << 32) if link & 0x80000000 else link
        pos += size
    return None


def find_first_block(image):
    """Offset, LAST index and link of the first valid block in the first 4 KiB."""
    head = words(image[:SECTOR])
    for i, w in enumerate(head):
        if w == BLOCK_START:
            f = frame(head[i:])
            if f:
                return i * 4, f[0], f[1]
    return None


def objcopy():
    sysroot = subprocess.check_output(["rustc", "--print", "sysroot"], text=True).strip()
    for p in Path(sysroot).glob("lib/rustlib/*/bin/llvm-objcopy"):
        return str(p)
    sys.exit("llvm-objcopy not found: rustup component add llvm-tools")


def flat_binary(elf, need_tag=True):
    """The ELF as a flat image starting at 0x10000000, checked for the things this tool relies on.
    A path ending in .bin is taken to be that flat image already."""
    if elf.endswith(".bin"):
        data = Path(elf).read_bytes()
    else:
        tmp = Path(elf).with_suffix(".objcopy.tmp")
        try:
            subprocess.check_call([objcopy(), "-O", "binary", elf, str(tmp)])
            data = tmp.read_bytes()
        finally:
            tmp.unlink(missing_ok=True)
    if len(data) < MIN_IMAGE:
        sys.exit(f"{elf}: only {len(data)} bytes of flash image")
    sp, reset = struct.unpack_from("<II", data)
    if not 0x20000000 < sp <= 0x20082000 or not FLASH_BASE <= reset & ~1 < FLASH_BASE + len(data) or not reset & 1:
        sys.exit(f"{elf}: does not start with a vector table linked at {FLASH_BASE:#x} (sp {sp:#x}, reset {reset:#x})")
    first = find_first_block(data)
    if not first:
        sys.exit(f"{elf}: no boot ROM IMAGE_DEF block in the first 4 KiB")
    off, last, link = first
    w = words(data[off : off + 8])
    if w[1] != (ITEM_IMAGE_TYPE | 1 << 8 | IMAGE_FLAGS << 16):
        sys.exit(f"{elf}: first block is not an RP2350 Arm Secure executable IMAGE_DEF ({w[1]:#010x})")
    if not need_tag:
        return data
    if link != 0:
        sys.exit(f"{elf}: the IMAGE_DEF already links to another block; this tool expects a freshly linked ELF")
    tag = data[:SECTOR].find(TAG[:8])
    if tag < 0 or tag % 4 or data[tag : tag + len(TAG)] != TAG:
        sys.exit(f"{elf}: the OFT firmware tag for this board and flash layout is not in the first 4 KiB")
    return data


def seal(image, build):
    """Append the end block and close the block loop, as `picotool seal --hash` does for a
    binary: IMAGE_DEF, VERSION, a relative LOAD_MAP over the whole image, HASH_DEF, HASH_VALUE."""
    image = bytearray(image) + b"\0" * (-len(image) % 4)
    off, last, _ = find_first_block(image)
    end = len(image)
    struct.pack_into("<i", image, off + (last + 1) * 4, end - off)  # first block -> end block
    block = [
        BLOCK_START,
        ITEM_IMAGE_TYPE | 1 << 8 | IMAGE_FLAGS << 16,
        ITEM_VERSION | 2 << 8,
        build,  # major << 16 | minor
        ITEM_LOAD_MAP | 4 << 8 | 1 << 24,  # one entry, relative
        (0 - (end + 4 * 4)) & 0xFFFFFFFF,  # storage start, relative to this item (word 4 of the block)
        FLASH_BASE,  # runtime start
        end,  # size
        ITEM_HASH_DEF | 2 << 8 | 1 << 24,  # SHA-256
        10,  # block words hashed: everything up to and including this item
    ]
    digest = hashlib.sha256(bytes(image) + to_bytes(block)).digest()
    block += [ITEM_HASH_VALUE | 9 << 8] + words(digest)
    block += [ITEM_LAST | (len(block) - 1) << 8, (off - end) & 0xFFFFFFFF, BLOCK_END]
    assert len(block) == END_BLOCK_WORDS
    return bytes(image) + to_bytes(block)


def verify_image(image):
    """Check a sealed image the way the boot ROM will, plus what the Timer insists on.
    Returns a dict of facts or raises ValueError. Written separately from seal()."""
    if len(image) % 4 or not MIN_IMAGE <= len(image) <= MAX_IMAGE:
        raise ValueError(f"length {len(image)} is not a multiple of 4 between {MIN_IMAGE} and {MAX_IMAGE}")
    first = find_first_block(image)
    if not first:
        raise ValueError("no block in the first 4 KiB")
    off, last, link = first
    w = words(image)
    if w[off // 4 + 1] != (ITEM_IMAGE_TYPE | 1 << 8 | IMAGE_FLAGS << 16):
        raise ValueError("first block is not an RP2350 Arm Secure executable")
    end = off + link
    if link <= 0 or end % 4 or end + END_BLOCK_WORDS * 4 != len(image):
        raise ValueError(f"first block links to {end:#x}, which is not {END_BLOCK_WORDS * 4} bytes before the end ({len(image):#x})")
    b = w[end // 4 :]
    f = frame(b)
    if not f or f[0] + 3 != END_BLOCK_WORDS or end + f[1] != off:
        raise ValueError("end block is malformed or does not link back to the first block")
    flags = b[1] >> 16
    if b[1] & 0xFFFF != (ITEM_IMAGE_TYPE | 1 << 8) or flags & ~IMAGE_FLAG_TBYB != IMAGE_FLAGS:
        raise ValueError(f"end block image type {b[1]:#010x}")
    if b[2] != (ITEM_VERSION | 2 << 8) or b[4] != (ITEM_LOAD_MAP | 4 << 8 | 1 << 24) or b[8] != (ITEM_HASH_DEF | 2 << 8 | 1 << 24) or b[10] != (ITEM_HASH_VALUE | 9 << 8):
        raise ValueError("end block items are not VERSION, LOAD_MAP, HASH_DEF, HASH_VALUE")
    storage = (end + 4 * 4 + b[5]) & 0xFFFFFFFF
    if storage != 0 or b[6] != FLASH_BASE or b[7] != end or b[9] != 10:
        raise ValueError("load map does not cover the image in place")
    hashed = bytearray(image[:end] + to_bytes(b[: b[9]]))
    hashed[end + 7] &= 0x7F  # the boot ROM hashes with the try-before-you-buy bit clear
    digest = hashlib.sha256(hashed).digest()
    if digest != to_bytes(b[11:19]):
        raise ValueError("SHA-256 mismatch")
    tag = image[:SECTOR].find(TAG[:8])
    if tag < 0 or tag % 4 or image[tag : tag + len(TAG)] != TAG:
        raise ValueError("no OFT tag for this board and layout")
    sp, reset = struct.unpack_from("<II", image)
    return {"len": len(image), "first_block": off, "end_block": end, "build": b[3], "tbyb": bool(flags & IMAGE_FLAG_TBYB), "sha256": digest.hex(), "tag": tag, "sp": sp, "reset": reset}


def partition_table():
    """The partition table block, as `picotool partition create` writes it for:
    unpartitioned space rw, accepts "absolute" UF2s; A and B: 2 MB each, rw, accept
    "rp2350-arm-s", B linked to A; version 1.0; hashed."""
    first_a, first_b = SLOT_A // SECTOR, SLOT_B // SECTOR
    last_a, last_b = first_a + SLOT_BYTES // SECTOR - 1, first_b + SLOT_BYTES // SECTOR - 1
    parts = [
        PERM_ALL | last_a << 13 | first_a,
        PERM_ALL | ACCEPTS_RP2350_ARM_S,
        PERM_ALL | last_b << 13 | first_b,
        PERM_ALL | ACCEPTS_RP2350_ARM_S | LINK_TO_A_PARTITION | 0 << 3,
    ]
    block = [BLOCK_START, ITEM_PARTITION_TABLE | (2 + len(parts)) << 8 | 2 << 24, PERM_ALL | ACCEPTS_ABSOLUTE] + parts
    block += [ITEM_VERSION | 2 << 8, 1 << 16 | 0]
    block += [ITEM_HASH_DEF | 2 << 8 | 1 << 24, len(block) + 2]
    block += [ITEM_HASH_VALUE | 9 << 8] + words(hashlib.sha256(to_bytes(block)).digest())
    block += [ITEM_LAST | (len(block) - 1) << 8, 0, BLOCK_END]
    return to_bytes(block)


def describe_partition_table(data):
    """Decode a partition table block; returns lines of text or raises ValueError."""
    w = words(data[:0x280])
    f = frame_any(w)
    if not f or w[1] & 0x7F != ITEM_PARTITION_TABLE:
        raise ValueError("no partition table block at the start")
    last, link = f
    count, size = (w[1] >> 24) & 0xF, item_size(w[1])
    out = [f"partition table: {count} partitions, block of {(last + 3) * 4} bytes, link {link}, unpartitioned flags {w[2]:#010x}" + (" (accepts absolute UF2s)" if w[2] & ACCEPTS_ABSOLUTE else "")]
    for k in range(count):
        loc, flags = w[3 + 2 * k], w[4 + 2 * k]
        first, last_s = loc & 0x1FFF, (loc >> 13) & 0x1FFF
        link_txt = f", B of partition {(flags >> 3) & 0xF}" if (flags >> 1) & 3 == 1 else ""
        out.append(f"  partition {k}: flash {first * SECTOR:#08x}..{(last_s + 1) * SECTOR:#08x} ({(last_s - first + 1) * SECTOR // 1024} K), flags {flags:#010x}" + (", accepts rp2350-arm-s" if flags & ACCEPTS_RP2350_ARM_S else "") + link_txt)
    i = 1 + size
    hashed, digest = None, None
    while i < last:
        t = w[i] & 0xFF
        if t == ITEM_VERSION:
            out.append(f"  version {w[i + 1] >> 16}.{w[i + 1] & 0xFFFF}")
        elif t == ITEM_HASH_DEF:
            hashed = w[i + 1]
        elif t == ITEM_HASH_VALUE:
            digest = to_bytes(w[i + 1 : i + 9])
        i += item_size(w[i])
    if hashed is None or digest != hashlib.sha256(to_bytes(w[:hashed])).digest():
        raise ValueError("partition table hash missing or wrong")
    out.append(f"  sha256 over its first {hashed} words: ok")
    return out


def frame_any(ws):
    """frame() without the IMAGE_DEF size limit (a partition table may be up to 0x280 bytes)."""
    global MAX_IMAGE_DEF_WORDS
    keep, MAX_IMAGE_DEF_WORDS = MAX_IMAGE_DEF_WORDS, 0x280 // 4
    try:
        return frame(ws)
    finally:
        MAX_IMAGE_DEF_WORDS = keep


def uf2_blocks(chunks, family):
    """`chunks` is a list of (flash offset, bytes); each is cut into 256-byte pages. Pages are
    written in the order given, which is the order the boot ROM programs them."""
    pages = []
    for offset, data in chunks:
        assert offset % PAGE == 0
        for i in range(0, len(data), PAGE):
            pages.append((FLASH_BASE + offset + i, data[i : i + PAGE].ljust(PAGE, b"\xff")))
    out = bytearray()
    for n, (addr, page) in enumerate(pages):
        out += struct.pack("<IIIIIIII", UF2_MAGIC0, UF2_MAGIC1, UF2_FLAG_FAMILY_ID, addr, PAGE, n, len(pages), family)
        out += page.ljust(476, b"\0") + struct.pack("<I", UF2_MAGIC_END)
    return bytes(out)


def parse_uf2(data):
    """Returns (family, {flash offset: page}) or raises ValueError."""
    if len(data) % 512:
        raise ValueError("length is not a multiple of 512")
    pages, family, total = {}, None, len(data) // 512
    for n in range(total):
        m0, m1, flags, addr, size, no, count, fam = struct.unpack_from("<IIIIIIII", data, n * 512)
        (end,) = struct.unpack_from("<I", data, n * 512 + 508)
        if (m0, m1, end) != (UF2_MAGIC0, UF2_MAGIC1, UF2_MAGIC_END) or flags != UF2_FLAG_FAMILY_ID or size != PAGE or no != n or count != total:
            raise ValueError(f"block {n} is malformed")
        if addr % PAGE or not FLASH_BASE <= addr < FLASH_BASE + 16 * 1024 * 1024 or addr - FLASH_BASE in pages:
            raise ValueError(f"block {n} has a bad or repeated address {addr:#x}")
        family = fam if family is None else family
        if fam != family:
            raise ValueError("mixed family IDs")
        pages[addr - FLASH_BASE] = data[n * 512 + 32 : n * 512 + 32 + PAGE]
    return family, pages


def runs(pages):
    """Contiguous ranges of pages, in file order: [(offset, bytes)]."""
    out = []
    for off, page in pages.items():
        if out and out[-1][0] + len(out[-1][1]) == off:
            out[-1][1].extend(page)
        else:
            out.append((off, bytearray(page)))
    return out


def build_text(build):
    return f"{build} ({time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime(BUILD_EPOCH + build))})"


def describe_image(info):
    return [
        f"image: {info['len']} bytes ({info['len'] / 1024:.1f} K of {MAX_IMAGE // 1024} K), build {build_text(info['build'])}",
        f"  vector table: initial sp {info['sp']:#010x}, reset {info['reset']:#010x}",
        f"  first block at {info['first_block']:#x}, OFT tag at {info['tag']:#x}, end block at {info['end_block']:#x} (last {END_BLOCK_WORDS * 4} bytes)",
        f"  sha256 {info['sha256']}: ok" + (", TRY-BEFORE-YOU-BUY BIT SET" if info["tbyb"] else ""),
    ]


def inspect(path):
    data = Path(path).read_bytes()
    print(f"{path}: {len(data)} bytes")
    if data[:4] != struct.pack("<I", UF2_MAGIC0):
        print("\n".join(describe_image(verify_image(data))))
        return
    family, pages = parse_uf2(data)
    print(f"UF2, {len(pages)} blocks, family {family:#010x} ({FAMILY_NAMES.get(family, 'unknown')})")
    rs = runs(pages)
    for off, body in rs:
        blank = " (all 0xff: erases the sector)" if all(b == 0xFF for b in body) else ""
        print(f"  flash {off:#08x}..{off + len(body):#08x} = {FLASH_BASE + off:#010x}.., {len(body)} bytes{blank}")
    flash = {off: bytes(body) for off, body in rs}
    if family != FAMILY_ABSOLUTE:
        raise ValueError("not an absolute-family UF2")
    if SLOT_A in flash:
        print("\n".join(describe_partition_table(flash[0])))
        # The last page of the image is padded with 0xff in the UF2; the image says how long it is.
        off, _, link = find_first_block(flash[SLOT_A])
        image = flash[SLOT_A][: off + link + END_BLOCK_WORDS * 4]
        info = verify_image(image)
        if info["tbyb"]:
            raise ValueError("the USB image must not have the try-before-you-buy bit set")
        print(f"slot A at {SLOT_A:#x}:")
        print("\n".join(describe_image(info)))
        if any(b != 0xFF for b in flash.get(SECTOR, b"\0")) or any(b != 0xFF for b in flash.get(SLOT_B, b"\0")):
            raise ValueError("the second partition-table slot and the start of slot B must be erased")
        print(f"erases {SECTOR:#x} (second partition-table slot) and {SLOT_B:#x} (first sector of slot B)")
    else:
        print("flat image at the start of flash (no partition table, no network update)")


def main(argv):
    if len(argv) == 2 and argv[0] == "--inspect":
        try:
            inspect(argv[1])
        except ValueError as e:
            sys.exit(f"{argv[1]}: NOT VALID: {e}")
        return
    args = [a for a in argv if not a.startswith("--")]
    flat = "--flat" in argv
    build = None
    for i, a in enumerate(argv):
        if a == "--build":
            build = int(argv[i + 1], 0)
            args.remove(argv[i + 1])
    if len(args) != 2 or any(a.startswith("--") and a not in ("--flat", "--build") for a in argv):
        sys.exit(__doc__)
    elf, out = args
    raw = flat_binary(elf, need_tag=not flat)
    if flat:
        # The whole flash is one image again: block 0 overwrites the partition table.
        Path(out).write_bytes(uf2_blocks([(0, raw)], FAMILY_ABSOLUTE))
        print(f"{out}: flat image, {len(raw)} bytes at {FLASH_BASE:#x}, absolute family. No network update with this layout.")
        return
    if build is None:
        build = int(time.time()) - BUILD_EPOCH
    if not 0 < build < 1 << 32:
        sys.exit("--build must fit in 32 bits")
    image = seal(raw, build)
    try:
        info = verify_image(image)
    except ValueError as e:
        sys.exit(f"internal error: the sealed image does not verify: {e}")
    table = partition_table()
    binpath = Path(out).with_suffix(".bin")
    binpath.write_bytes(image)
    # Order matters if the copy is interrupted: the partition table first (from then on the
    # boot ROM only starts complete, hashed images), slot B's first sector last (until then
    # an older firmware in B can still boot).
    blank = b"\xff" * PAGE
    uf2 = uf2_blocks([(0, table), (SECTOR, blank), (SLOT_A, image), (SLOT_B, blank)], FAMILY_ABSOLUTE)
    Path(out).write_bytes(uf2)
    print(f"{binpath}: network update image")
    print("\n".join("  " + line for line in describe_image(info)))
    print(f"{out}: USB install, {len(uf2) // 512} blocks, absolute family")
    print(f"  partition table at 0x000000, slot A image at {SLOT_A:#08x}, erases 0x001000 and {SLOT_B:#08x}")


if __name__ == "__main__":
    main(sys.argv[1:])
