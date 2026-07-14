# Minimal C runtime startup for the OoO RISC-V core.
#
# Reset PC is 0, so this must be the first thing in the image (.text.init, placed first by
# link.ld). There is no OS, no traps, and no CSRs implemented in the core, so this does the
# bare minimum: set up a stack, zero .bss, call main(), publish the return value, and halt.
#
# HALT / RESULT PROTOCOL: main()'s return value is stored to __tohost (byte 0x4000). That word
# address (0x1000) is above vgaBaseWord, so Memory.scala routes the write out of the RAM array
# and onto the MMIO port, where the testbench can observe it. We then spin forever -- the core
# has no way to stop itself.

    .section .text.init, "ax"
    .globl _start
_start:
    # ---- stack ----
    la      sp, __stack_top

    # ---- zero .bss ----
    la      t0, __bss_start
    la      t1, __bss_end
1:
    bge     t0, t1, 2f
    sw      zero, 0(t0)
    addi    t0, t0, 4
    j       1b
2:

    # ---- run the program ----
    call    main

    # ---- publish main()'s return value to the testbench mailbox ----
    # 0x70000010 is the tohost MMIO word (see OoOParams). It sits well above RAM, so it keeps
    # working no matter how large RAM grows for a Doom-sized image.
    li      t0, 0x70000010
    sw      a0, 0(t0)

    # ---- halt (spin) ----
3:
    j       3b
