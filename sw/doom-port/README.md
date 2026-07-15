# Doom on the OoO RISC-V core

These are the files that port Doom (via **doomgeneric**) to this core. They are kept here,
separate from the Doom source itself, because the Doom clone is GPL and multi-megabyte and is
**not** vendored into this repository (see `.gitignore`: `sw/doom/`).

## What's here

| File | Purpose |
|---|---|
| `build_doom.sh` | Builds the image: compiles doomgeneric at `-march=rv32im -O2 -mstrict-align`, embeds the WAD, links with picolibc, emits `doom.hex` for `$readmemh`. |
| `wad.S` | Embeds the WAD via `.incbin` (the upstream port uses C23 `#embed`, which needs GCC 15; we have 13). |
| `picolibc_glue.c` | Bridges picolibc to the port's newlib-style syscall stubs, and routes `printf` to the sim's debug-char MMIO port so Doom's console is visible. |
| `link_doom.ld` | (Reference only — the build uses picolibc's own `picolibc.ld` with a custom flash/RAM split.) |

## Prerequisites

- `riscv64-unknown-elf-gcc` and `picolibc-riscv64-unknown-elf`
- A WAD. FreeDM (freely redistributable) works:
  `curl -L -o freedm.zip https://github.com/freedoom/freedoom/releases/download/v0.13.0/freedm-0.13.0.zip`

## Build & run

```bash
# 1. fetch the port (doomgeneric + the RISC-V platform layer)
git clone https://github.com/MIT-OpenCompute/doom-hopper.git sw/doom

# 2. drop in a WAD and our port files
cp path/to/freedm.wad             sw/doom/doomgeneric/
cp sw/doom-port/{wad.S,picolibc_glue.c,build_doom.sh} sw/doom/doomgeneric/

# 3. build -> sw/doom/doomgeneric/build/doom.hex
( cd sw/doom/doomgeneric && ./build_doom.sh )

# 4. run it on the core (preloads doom.hex into 64 MB RAM; console streams live)
sbt -Ddoom.cycles=600000000 "testOnly RISCV.DoomSpec"
```

## Status

Doom **boots**: it loads the WAD (identifies "Freedoom: Phase 1"), runs `Z_Init` (allocates the
6 MB zone heap), `I_Init`, `M_Init`, and enters `R_Init` (the renderer's texture/sprite precache).
Reaching the first rendered frame needs on the order of 10^9 instructions; at ~485k simulated
cycles/sec that is a long single run. Observed IPC on Doom is ~0.51 (vs ~0.15 on a tight
microbenchmark) -- the out-of-order engine finds real parallelism in Doom's code.
