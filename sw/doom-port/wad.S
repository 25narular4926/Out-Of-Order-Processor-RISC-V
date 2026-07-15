/*
 * Embed the WAD into the binary via .incbin, replacing the port's C23 `#embed` (which needs
 * GCC 15; our cross-compiler is GCC 13). Exposes the same `doom1_wad` symbol the fake filesystem
 * in stubs.h reads from, plus a length word `doom1_wad_len` to stand in for the `sizeof()` the
 * #embed array allowed.
 */
    .section .rodata
    .align 4
    .global doom1_wad
doom1_wad:
    .incbin "freedm.wad"
doom1_wad_end:

    .align 4
    .global doom1_wad_len
doom1_wad_len:
    .word doom1_wad_end - doom1_wad
