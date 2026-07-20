# Native multithreaded Verilator harness

A C++ testbench that drives the SoC (`SocHarness`) directly, so we can do two things ChiselSim
cannot:

1. **Multithreaded simulation** — Verilator's `--threads N` (ChiselSim/svsim does not expose it).
   Expect roughly **2–5×** wall-clock on a multicore machine for a design this size.
2. **Native-speed flashing** — no JVM↔Verilator round-trip, so flashing a Doom-sized image is
   seconds, not the hours ChiselSim's per-word round-trip cost.

It mirrors `ProgramSpec`/`DoomSpec`: flash a hex image, run to the `tohost` mailbox (or a cycle
cap), optionally capture framebuffer PPMs, and print the hardware performance counters.

## Build & run (WSL Ubuntu)

```bash
cd sim/verilator
THREADS=8 ./build.sh                 # emit SV → verilate with 8 threads → compile
# a compiled-C program:
./obj_dir/simx ../../sw/build/arith.hex 2000000
# Doom, capturing 60 frames, walking from frame 30 (like DoomSpec):
./obj_dir/simx ../../sw/doom/doomgeneric/build/doom.hex 500000000 frames 60 1 30
```

Arguments: `simx <image.hex> [maxCycles] [framesDir framesN stride moveFromFrame]`.
Frames dump as `frame_0000.ppm …`; convert with the existing scratchpad `ppm2png.py` / `build_anim.py`.

## How it works

- `EmitSim` (Scala, test scope) emits `SocHarness` to `generated/sim/*.sv`. It bakes in the
  Doom-sized 16 M-word memory so one model runs Doom, the compiled-C programs, and the ISA tests.
- `build.sh` verilates that SV **with `--threads`** plus the sim-speed flags (`-O3 --x-assign fast`)
  and links `sim_main.cpp`.
- `sim_main.cpp` flashes the image through the flash port, runs, scans the framebuffer capture RAM
  for PPMs, and reads the `perf_*` counters.

## Status / caveats

- **Untested first pass.** Port names use Chisel's flattened `io_<field>` convention; if the first
  `build.sh` errors on an unknown member, check `obj_dir/VSocHarness.h` for the exact names and
  adjust `sim_main.cpp`.
- Determinism comes from `-disable-all-randomization` (0-init) at emit time; the harness flashes
  every region it needs, so 0-init is safe.
- This does **not** replace ChiselSim — the ScalaTest suites remain the correctness gate. This is a
  faster *runner* for long workloads (Doom) and iteration.
- Speed is still software-simulation speed (10⁵–10⁶ cycles/sec range). It does **not** reach
  real-time; that needs FPGA/cloud-FPGA.
