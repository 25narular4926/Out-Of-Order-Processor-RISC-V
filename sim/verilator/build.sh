#!/usr/bin/env bash
#
# Build the native, MULTITHREADED Verilator harness for the OoO RISC-V SoC.
#
# Two steps:
#   1. emit SystemVerilog from SocHarness (Core + Memory + observability) via Chisel
#   2. verilate it WITH --threads and compile it against sim_main.cpp
#
# The result (obj_dir/simx) runs the SoC far faster than ChiselSim -- multithreaded, and with no
# JVM<->Verilator round-trip. Run it in WSL Ubuntu (needs verilator + the RISC-V build outputs).
#
#   THREADS=8 ./build.sh                 # override core count (default: nproc)
#   ./obj_dir/simx sw/build/arith.hex 2000000
#   ./obj_dir/simx sw/doom/doomgeneric/build/doom.hex 500000000 frames 60 1 30
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
THREADS="${THREADS:-$(nproc)}"
SVDIR="$ROOT/generated/sim"

echo ">> [1/2] emitting SystemVerilog (SocHarness) via Chisel ..."
( cd "$ROOT" && sbt "Test/runMain RISCV.EmitSim" )

if ! ls "$SVDIR"/*.sv >/dev/null 2>&1; then
    echo "!! no .sv produced in $SVDIR -- did EmitSim run?" >&2
    exit 1
fi

echo ">> [2/2] verilating with --threads $THREADS (this can take a few minutes for the 64 MB model) ..."
verilator \
  --cc --exe --build -j 0 \
  --threads "$THREADS" \
  -O3 --x-assign fast --x-initial fast \
  --top-module SocHarness \
  -Wno-WIDTH -Wno-UNOPTFLAT -Wno-CASEINCOMPLETE -Wno-fatal \
  -CFLAGS "-O2 -std=c++17" \
  --Mdir "$HERE/obj_dir" \
  -o simx \
  "$SVDIR"/*.sv \
  "$HERE/sim_main.cpp"

echo ">> done: $HERE/obj_dir/simx"
echo "   example: $HERE/obj_dir/simx <image.hex> <maxCycles> [framesDir framesN stride moveFrom]"
