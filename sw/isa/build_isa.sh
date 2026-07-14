#!/usr/bin/env bash
#
# Build the riscv-tests RV32UI + RV32UM suites against the OoO core's custom environment
# (riscv_test.h here), producing one flat hex image per test in build/.
#
# Each image is self-contained: the test IS _start at address 0, and it exits by storing a
# result word to the tohost mailbox (pass = 1, fail = (TESTNUM << 1) | 1).
#
#   usage: ./build_isa.sh
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TESTS="$HERE/../riscv-tests/isa"
OUT="$HERE/build"

CC=riscv64-unknown-elf-gcc
OBJCOPY=riscv64-unknown-elf-objcopy

# rv32im: the core implements I and M. No compressed (Fetch assumes fixed 4-byte instructions).
CFLAGS="-march=rv32im -mabi=ilp32 -nostdlib -nostartfiles -fno-builtin -Wno-implicit-function-declaration"
INCLUDES="-I$HERE -I$TESTS/macros/scalar -I$TESTS/../env"

# Tests we deliberately SKIP, with the reason. These are not core bugs -- they exercise
# features this design intentionally does not implement.
#   fence_i : requires FENCE.I + self-modifying code. No fence support, no icache.
#   ma_data : deliberately performs MISALIGNED loads/stores. The LSU has no misaligned support
#             (documented); RISC-V allows a core to trap instead, and we have no trap handler.
SKIP="fence_i ma_data"

mkdir -p "$OUT"
rm -f "$OUT"/*.hex "$OUT"/*.elf 2>/dev/null

built=0; skipped=0; failed=0

for suite in rv32ui rv32um; do
  for src in "$TESTS/$suite"/*.S; do
    name="$(basename "$src" .S)"

    if echo "$SKIP" | grep -qw "$name"; then
      echo "SKIP  $suite-$name"
      skipped=$((skipped + 1))
      continue
    fi

    elf="$OUT/$suite-$name.elf"
    hex="$OUT/$suite-$name.hex"

    if ! $CC $CFLAGS $INCLUDES -T "$HERE/link_isa.ld" -o "$elf" "$src" 2>"$OUT/$name.log"; then
      echo "BUILD-FAIL  $suite-$name   (see $OUT/$name.log)"
      failed=$((failed + 1))
      continue
    fi

    $OBJCOPY -O binary "$elf" "$OUT/$name.bin"
    od -An -tx4 -v -w4 "$OUT/$name.bin" | tr -d ' ' | grep -v '^$' > "$hex"
    rm -f "$OUT/$name.bin" "$OUT/$name.log"
    built=$((built + 1))
  done
done

echo
echo "built=$built  skipped=$skipped  build-failed=$failed"
[ "$failed" -eq 0 ]
