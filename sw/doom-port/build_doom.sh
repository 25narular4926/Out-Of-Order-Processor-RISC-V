#!/usr/bin/env bash
#
# Build Doom (doomgeneric + the RISC-V port) for the OoO core, producing a flat memory image and
# the $readmemh hex that Memory preloads.
#
# Differences from the upstream doom-hopper Makefile, and why:
#   -march=rv32im (not rv32i): our core HAS hardware multiply/divide. Doom's renderer is wall to
#     wall FixedMul/FixedDiv; letting them be single instructions instead of libgcc software
#     routines is a large win when every instruction is a simulated cycle.
#   -O2 (not -O0): fewer instructions to simulate. Same reason.
#   -mstrict-align: the LSU has no misaligned-access support, so never let gcc emit an unaligned
#     wide access.
#   .incbin instead of C23 #embed: our GCC is 13; #embed needs 15.
#   48 MB stack top instead of 128 MB: our RAM is 48 MB (below the MMIO regions).
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

CC=riscv64-unknown-elf-gcc
OBJCOPY=riscv64-unknown-elf-objcopy
OBJDUMP=riscv64-unknown-elf-objdump

OUT=build
mkdir -p "$OUT"

# picolibc supplies the C library (stdio/stdlib/string), its own crt0 (_start: sets the stack,
# initialises TLS -- picolibc keeps errno in TLS -- copies .data, zeroes .bss, calls main), and
# its own linker script picolibc.ld. We let it own all of that instead of the port's hand-rolled
# _start, which is more robust (TLS especially). --crt0=minimal picks the bare-metal crt0 (no
# semihosting calls that would trap on this core).
PICO="--specs=picolibc.specs --crt0=minimal"
CFLAGS="-march=rv32im -mabi=ilp32 -mstrict-align -O2 $PICO"
CFLAGS="$CFLAGS -fcommon -fno-strict-aliasing -w -DNORMALUNIX -DLINUX -I."

# Memory layout handed to picolibc.ld (all one physical SyncReadMem, split by role):
#   flash 0x00000000 + 40 MB : code, rodata (incl. the 21 MB WAD), and .data init image
#   ram   0x02800000 + 24 MB : .data (runtime), .bss, heap, stack   -> ends at 0x04000000 = 64 MB
# crt0 copies .data from its flash LMA to its ram VMA at boot; both are real RAM here.
LDLAYOUT="-Wl,--defsym=__flash=0x00000000 -Wl,--defsym=__flash_size=0x02800000"
LDLAYOUT="$LDLAYOUT -Wl,--defsym=__ram=0x02800000 -Wl,--defsym=__ram_size=0x01800000"
# no __heap_size -> picolibc gives the heap ALL ram between .bss and the stack.
LDLAYOUT="$LDLAYOUT -Wl,--defsym=__stack_size=0x00100000"

# --- patch the vendored port (idempotent) --------------------------------------------------
echo ">> patching stubs.h (#embed -> extern) and rvdoom stack pointer"
# Replace the #embed WAD definition with extern declarations backed by wad.S.
perl -0777 -pi -e 's/const unsigned char doom1_wad\[\] = \{\s*#embed "doom1\.wad"\s*\};/extern const unsigned char doom1_wad[];\nextern const unsigned int doom1_wad_len;/s' stubs.h
# sizeof() does not work on an extern array of unknown bound -> use the length word from wad.S.
sed -i 's/sizeof(doom1_wad)/doom1_wad_len/g' stubs.h
# The fake filesystem only recognises "doom1.wad"; our WAD is freedm.wad. Accept either name.
sed -i 's/strcmp("doom1.wad", path)/(strcmp("doom1.wad", path) \&\& strcmp("freedm.wad", path))/' stubs.h
# Use picolibc's crt0 for startup, not the port's hand-rolled _start: rename the port symbol so
# it becomes dead code (gc-sectioned away) and does not clash with crt0's _start.
sed -i 's/void _start(void)/void _port_start_unused(void)/' doomgeneric_rvdoom.c
# Replace the stdio WAD backend with the memory-direct one (reads the embedded doom1_wad array
# directly instead of through FILE*/fread, which on picolibc is byte-at-a-time buffered stdio).
if [ -f w_file_mem.c ]; then cp w_file_mem.c w_file_stdc.c; fi

# Make the memory-mapped I/O accesses VOLATILE. The port dereferences the timer (0x8000004) and
# framebuffer (0x10000000) through plain (non-volatile) pointers. At -O0 (which the upstream build
# used) that happens to work; at -O2 (which we use for speed) the compiler hoists the loop-invariant
# timer read out of DG_SleepMs's wait loop -- turning it into an infinite loop -- and may dead-store-
# eliminate the framebuffer writes. Forcing volatile fixes both.
sed -i 's/(uint32_t\*)0x8000004/(volatile uint32_t*)0x8000004/g'   doomgeneric_rvdoom.c
sed -i 's/(uint32_t\*)0x10000000/(volatile uint32_t*)0x10000000/g' doomgeneric_rvdoom.c

# --- source list: the rvdoom object set from the upstream Makefile, minus dummy platform files -
SRCS=$(grep -oE '[a-z0-9_]+\.o' Makefile | sed 's/\.o$/.c/' | sort -u)

echo ">> compiling $(echo "$SRCS" | wc -w) C files (rv32im -O2)"
OBJS=""
fail=0
for c in $SRCS; do
  if [ ! -f "$c" ]; then continue; fi          # skip .o with no matching .c (e.g. platform stubs)
  o="$OUT/${c%.c}.o"
  if ! $CC $CFLAGS -c "$c" -o "$o" 2>"$OUT/${c%.c}.log"; then
    echo "   COMPILE FAILED: $c  (see $OUT/${c%.c}.log)"
    fail=1
  fi
  OBJS="$OBJS $o"
done

# assemble the embedded WAD
$CC $CFLAGS -c wad.S -o "$OUT/wad.o" || fail=1
OBJS="$OBJS $OUT/wad.o"

# picolibc glue (POSIX name forwarding + console device)
if ! $CC $CFLAGS -c picolibc_glue.c -o "$OUT/picolibc_glue.o" 2>"$OUT/picolibc_glue.log"; then
  echo "   COMPILE FAILED: picolibc_glue.c  (see $OUT/picolibc_glue.log)"
  fail=1
fi
OBJS="$OBJS $OUT/picolibc_glue.o"

[ "$fail" -eq 0 ] || { echo ">> compile errors -- stopping"; exit 1; }

echo ">> linking (picolibc crt0 + picolibc.ld)"
$CC $CFLAGS $LDLAYOUT $OBJS -o "$OUT/doom.elf" -Wl,-Map,"$OUT/doom.map" 2>"$OUT/link.log" \
  || { echo ">> LINK FAILED:"; tail -25 "$OUT/link.log"; exit 1; }

# sanity: _start must be at 0 (the core's reset PC)
$OBJDUMP -t "$OUT/doom.elf" | grep -qE '^0+ .* _start' \
  || { echo ">> WARNING: _start is not at address 0"; $OBJDUMP -t "$OUT/doom.elf" | grep -w _start; }

echo ">> objcopy -> flat binary"
$OBJCOPY -O binary "$OUT/doom.elf" "$OUT/doom.bin"

echo ">> hex image for \$readmemh"
od -An -tx4 -v -w4 "$OUT/doom.bin" | tr -d ' ' | grep -v '^$' > "$OUT/doom.hex"

BYTES=$(stat -c%s "$OUT/doom.bin")
WORDS=$(wc -l < "$OUT/doom.hex")
printf ">> image: %.1f MB (%d words)\n" "$(echo "$BYTES/1048576" | bc -l)" "$WORDS"
$OBJDUMP -t "$OUT/doom.elf" | grep -E '_end$|__bss_start|__bss_end' | awk '{print "   " $0}'
echo ">> done: $OUT/doom.hex"
