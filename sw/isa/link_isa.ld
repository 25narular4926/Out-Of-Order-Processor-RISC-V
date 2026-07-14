/*
 * Linker script for the riscv-tests ISA suite on the OoO core.
 *
 * The test itself is _start (there is no crt0): RVTEST_CODE_BEGIN emits .text.init, which must
 * land at address 0 because that is the core's reset PC. Everything must fit in the 16 KB the
 * SoC has (OoOParams.memWords = 4096 words); the ISA tests are tiny, so this is not a
 * constraint here.
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

MEMORY
{
  RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 16K
}

SECTIONS
{
  . = ORIGIN(RAM);

  .text : {
    *(.text.init)          /* the test's _start -- must be at 0 */
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
    *(.bss .bss.* .sbss .sbss.* COMMON)
    . = ALIGN(4);
  } > RAM

  _end = .;

  /DISCARD/ : { *(.comment) *(.note.*) *(.eh_frame) *(.riscv.attributes) }
}
