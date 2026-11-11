/* Waveshare RP2350-POE-ETH: RP2350A, 16 MB QSPI NOR flash, 520 KB SRAM.
 *
 * The firmware lives in one of two 2 MB flash partitions (A at 0x002000, B at 0x202000, see
 * fwimage/src/layout.rs and tools/uf2.py). The boot ROM maps whichever one it starts to
 * 0x10000000, so the image is linked there and runs unchanged from either.
 * FLASH is one partition less 4 K: tools/uf2.py appends an 88-byte hash block, and the boot
 * ROM refuses an image that reaches the end of its partition (MAX_IMAGE_BYTES). */
MEMORY {
    FLASH : ORIGIN = 0x10000000, LENGTH = 2044K
    RAM   : ORIGIN = 0x20000000, LENGTH = 512K
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
}

SECTIONS {
    /* Boot ROM info: after .vector_table, in the first 4K of flash where the ROM finds it. */
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH
} INSERT AFTER .vector_table;

_stext = ALIGN(ADDR(.start_block) + SIZEOF(.start_block), ALIGNOF(.text));

SECTIONS {
    /* picotool 'Binary Info' entries */
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    /* Boot ROM extra info, after everything, so it can hold a signature. */
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
    } > FLASH
} INSERT AFTER .uninit;

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);
