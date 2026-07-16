/*
 * Embed the WAD into the binary via .incbin, replacing the port's C23 `#embed` (which needs
 * GCC 15; our cross-compiler is GCC 13). Exposes the same `doom1_wad` symbol the fake filesystem
 * in stubs.h reads from, plus a length word `doom1_wad_len` to stand in for the `sizeof()` the
 * #embed array allowed.
 *
 * The embedded file is always `game.wad`: build_doom.sh copies the selected WAD (WAD=... env var,
 * e.g. freedm.wad or doom1.wad) to that canonical name before assembling. Keep both WADs on disk;
 * each build embeds exactly one.
 */
    .section .rodata
    .align 4
    .global doom1_wad
doom1_wad:
    .incbin "game.wad"
doom1_wad_end:

    .align 4
    .global doom1_wad_len
doom1_wad_len:
    .word doom1_wad_end - doom1_wad
