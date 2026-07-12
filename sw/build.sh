#!/usr/bin/env bash
#
# Compile a bare-metal C program for the OoO RISC-V core and emit the flat hex-word image that
# Main's flash port consumes (one 32-bit little-endian word per line, line N == word address N).
#
#   usage:  ./build.sh tests/bytes.c  [outdir]
#
# Toolchain: riscv64-unknown-elf-gcc (Ubuntu package gcc-riscv64-unknown-elf), targeting rv32im.
# NOTE the ISA string: rv32im with NO compressed extension -- Fetch assumes fixed 4-byte
# instructions and increments the PC by 4, so a `c.` instruction would desynchronise it.
set -euo pipefail

SRC="${1:?usage: build.sh <program.c> [outdir]}"
OUT="${2:-build}"
NAME="$(basename "${SRC%.*}")"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CC=riscv64-unknown-elf-gcc
OBJCOPY=riscv64-unknown-elf-objcopy
OBJDUMP=riscv64-unknown-elf-objdump

ARCH_FLAGS="-march=rv32im -mabi=ilp32 -mstrict-align"
# -mstrict-align: the LSU has no misaligned-access support; never let gcc merge into an
#                 unaligned wide access.
CFLAGS="$ARCH_FLAGS -Os -ffreestanding -nostdlib -nostartfiles -fno-builtin -Wall -Wextra"

mkdir -p "$OUT"

echo ">> compiling $SRC for rv32im"
$CC $CFLAGS -T "$HERE/link.ld" -o "$OUT/$NAME.elf" "$HERE/crt0.S" "$SRC"

echo ">> disassembly -> $OUT/$NAME.dis"
$OBJDUMP -d -M no-aliases "$OUT/$NAME.elf" > "$OUT/$NAME.dis"

echo ">> flat binary"
$OBJCOPY -O binary "$OUT/$NAME.elf" "$OUT/$NAME.bin"

# Convert the little-endian binary into one big-endian-printed hex word per line, which is the
# format MainSpec reads (Long.parseLong(line, 16) -> flash_value at flash_address = line index).
echo ">> hex image -> $OUT/$NAME.hex"
od -An -tx4 -v -w4 "$OUT/$NAME.bin" | tr -d ' ' | grep -v '^$' > "$OUT/$NAME.hex"

WORDS=$(wc -l < "$OUT/$NAME.hex")
BYTES=$((WORDS * 4))
echo ">> $NAME: $WORDS words ($BYTES bytes)"
if [ "$WORDS" -gt 4096 ]; then
  echo "!! image is $WORDS words but the core only has 4096 words (16 KB) of memory" >&2
  exit 1
fi
