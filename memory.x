MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    Z80RAM: ORIGIN = 0x20000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000 + 64K, LENGTH = 256K - 64K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

SECTIONS {
    .z80ram (NOLOAD) :
    {
        *(.z80ram .z80ram.*);
    } > Z80RAM
} INSERT BEFORE .data;
