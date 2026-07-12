/*
 * Bare-metal linker script for the OoO RISC-V core.
 *
 * The SoC has ONE 16 KB word-addressed memory (OoOParams.memWords = 4096 words x 4 bytes),
 * shared by instruction fetch (port 1) and data accesses (port 2). Code and data therefore
 * live in the same 0x0000-0x3FFF space, and the whole image must fit in 16 KB.
 *
 * Reset PC is 0 (Fetch's pc register resets to 0), so .text.init must be placed first.
 *
 * Word addresses >= vgaBaseWord (0x1000) are NOT ram -- Memory.scala redirects them to the
 * VGA/MMIO port. Word 0x1000 == byte 0x4000, which is exactly the end of RAM, so all 16 KB is
 * usable. We use byte 0x4000 as a "tohost" mailbox: a store there escapes the memory array and
 * is visible to the testbench.
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

MEMORY
{
  RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 16K
}

/* Stack grows down from the top of RAM. Keep it clear of the MMIO boundary at 0x4000. */
__stack_top = ORIGIN(RAM) + LENGTH(RAM) - 16;

/* Testbench mailbox: first word of the MMIO region. */
__tohost = 0x4000;

SECTIONS
{
  . = ORIGIN(RAM);

  .text : {
    *(.text.init)          /* crt0 -- must be at address 0 */
    *(.text .text.*)
  } > RAM

  .rodata : {
    . = ALIGN(4);
    *(.rodata .rodata.* .srodata .srodata.*)
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

  . = ALIGN(4);
  _end = .;

  /DISCARD/ : { *(.comment) *(.note.*) *(.eh_frame) *(.riscv.attributes) }
}
