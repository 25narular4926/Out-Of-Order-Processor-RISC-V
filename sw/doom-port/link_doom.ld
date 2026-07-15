/*
 * Linker script for Doom on the OoO RISC-V core.
 *
 * RAM is 48 MB, far larger than the 16 KB the ISA tests used: the FreeDM WAD alone is 21 MB
 * (embedded in .rodata via wad.S), and Doom wants a 6 MB zone heap on top of code, data, and
 * stack. All of it lives below the peripheral MMIO regions (timer at 0x08000004, framebuffer at
 * 0x10000000), so RAM must stay under 0x08000000.
 *
 * Layout:  [ .text | .rodata (incl. 21 MB WAD) | .data | .bss ] _end ... heap grows up (sbrk) ...
 *          ... stack grows down ... __stack_top = 48 MB.
 *
 * _start (in doomgeneric_rvdoom.c, section .init) must be at address 0 -- the core's reset PC.
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

MEMORY
{
  RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 48M
}

__stack_top = ORIGIN(RAM) + LENGTH(RAM) - 16;

SECTIONS
{
  . = ORIGIN(RAM);

  .text : {
    KEEP(*(.init))          /* _start -- must be first, at address 0 */
    KEEP(*(.text.init))
    *(.text .text.*)
  } > RAM

  .rodata : {
    . = ALIGN(4);
    *(.rodata .rodata.* .srodata .srodata.*)   /* includes the embedded WAD */
  } > RAM

  .data : {
    . = ALIGN(4);
    *(.data .data.* .sdata .sdata.*)
  } > RAM

  .bss (NOLOAD) : {
    . = ALIGN(4);
    __bss_start = .;
    *(.bss .bss.* .sbss .sbss.* COMMON)
    . = ALIGN(4);
    __bss_end = .;
  } > RAM

  . = ALIGN(16);
  _end = .;

  /DISCARD/ : { *(.comment) *(.note.*) *(.eh_frame) *(.riscv.attributes) }
}
