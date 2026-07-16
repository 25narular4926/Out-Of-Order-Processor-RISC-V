# OoO RISC-V Core — Project Guide (read this first)

A 1-wide **out-of-order RV32IM** CPU written in Chisel (Scala). It is a sibling to MIT
OpenCompute's **in-order** RV32I core "Hopper" (<https://github.com/MIT-OpenCompute/hopper-cpu>,
MIT licensed) and deliberately reuses that project's *memory-map interface* so software written
for it — notably the Doom port — runs here. See `ATTRIBUTION.md`. The core itself is original;
Hopper has no ROB / rename / issue queue / load-store queues.

**Status (as of the last commit `7da3139`): the core is verified against the RISC-V ISA and
RUNS DOOM — it renders the Freedoom Phase 1 title screen (screenshot at `doom_frame.png`).**
Everything below is verified in-session, not assumed. When in doubt, re-verify against the code
before acting — a wrong assumption here can break working progress.

---

## 1. CRITICAL: how to build and test

**Tests MUST run in WSL Ubuntu, NOT native Windows.** ChiselSim shells out to Verilator, which
needs a Unix toolchain; also `firtool` on Windows hits a code-page error creating the
`verification\Assert` layer dirs. Native `sbt compile` works, but `sbt test` does not on Windows.

Run the full suite:
```bash
wsl -d Ubuntu -- bash -lc 'cd /mnt/c/Users/25nar/MITOpenCompute/OoO-RISC-V && sbt test'
```
Run one suite / filter:
```bash
wsl -d Ubuntu -- bash -lc 'cd /mnt/c/Users/25nar/MITOpenCompute/OoO-RISC-V && sbt "testOnly RISCV.ExecuteSpec"'
wsl -d Ubuntu -- bash -lc 'cd /mnt/c/Users/25nar/MITOpenCompute/OoO-RISC-V && sbt "testOnly RISCV.ExecuteSpec -- -z ExecMulDiv"'
```

**Known flaky failure (NOT a real bug):** running from `/mnt/c` (DrvFs) causes
`make: Warning: File ... modification time ... in the future` clock-skew warnings and an
intermittent Verilator build crash `Internal Error: attempted to destroy locked Thread Pool` /
`Verilator aborted`. These are BUILD-time crashes, not logic failures. **Re-run the affected
test(s) and they pass.** A real logic failure shows `was not equal` (ScalaTest) or
`FAILED at sub-test #N` (ISA). Distinguish by grepping the log for those vs `Verilator aborted`.
Building on WSL's native ext4 (not `/mnt/c`) would reduce this flakiness.

**Toolchain installed in WSL Ubuntu:** `riscv64-unknown-elf-gcc` 13.2 (bare-metal),
`picolibc-riscv64-unknown-elf`, Verilator 5.020, Java 11, sbt. `sudo` in WSL needs a password —
Claude cannot run `apt-get install`; ask the user to run installs.

---

## 2. Architecture (verified from source)

1-wide, RV32IM. In-order front end, out-of-order execute, in-order retire.

```
Fetch -> Decode -> Rename -> Dispatch  |  IssueQueue -> {ExecAlu, ExecMulDiv, Lsu} -> CDB
   (in order)                          |     (out of order)              |
                                       ROB (in-order retire) <-----------+
```

- **Recovery model:** commit-time **full flush** (no branch masks). On a mispredict/exception the
  offending instruction waits until it is the ROB head, then the ROB asserts a global `Redirect`
  that clears ALL speculative state. `brMask`/`brTag` fields exist in `MicroOp` but are RESERVED /
  unused (reserved for a future fast-recovery upgrade).
- **Sizes (`OoOParams` defaults):** xlen 32, numArchRegs 32, numPhysRegs 64, robEntries 32,
  iqEntries 16, ldq/stq 8 each, numWbPorts 3 (ALU, MULDIV, MEM), `divBitsPerCycle 4`,
  `memWords 4096` (default; Doom overrides to 16M), `clockHz 125_000_000`, fb 320x240.
- **CDB port convention:** wb(0)=ExecAlu, wb(1)=ExecMulDiv, wb(2)=Lsu. PRF read ports: 0/1=ALU,
  2/3=MULDIV, 4/5=MEM.
- **IssueQueue:** unified, BOOM-style **age matrix**, 1-cycle-late wakeup. `PhysRegFile` has NO
  write-to-read bypass (deliberate — see its header; wakeup is 1-cycle-late so it's redundant and
  a bypass would create a comb loop with the LSU forwarded-load writeback).
- **ExecAlu:** 1-cycle registered. **ExecMulDiv:** MUL combinational (1-cycle); DIV/REM iterative
  restoring divider retiring `divBitsPerCycle` bits/cycle (default 4 => 8-cycle divide).
- Key files: `src/main/scala/RISCV/` — `Params.scala`, `Bundles.scala`, `Memory.scala`,
  `Main.scala`, `core/Core.scala`, `core/frontend/{Fetch,Decode,Rename,MapTable,FreeList,
  BusyTable,Dispatch,Frontend,BranchPredictor}.scala`, `core/execute/{IssueQueue,PhysRegFile,
  ExecAlu,ExecMulDiv}.scala`, `core/memory/Lsu.scala`, `core/commit/ReorderBuffer.scala`.

---

## 3. Bugs fixed (chronological, all verified)

Do NOT reintroduce these. Commit `d3c53e8` fixed the first three; `d2685c8` the predictor.

1. **SB/SH clobbered neighbouring bytes.** The LSU spliced sub-word stores over a ZERO base word.
   Fixed with **per-byte write enables**: `Memory` stores `Vec(4, UInt(8.W))` and takes a 4-bit
   `write_mask_2`. LSU helpers `storeMask`/`storeData`/`loadMask`. SW/SB/SH all exact now.
2. **Store-to-load forwarding is coverage-checked.** A store may forward to a load only if it
   writes EVERY byte lane the load reads; a partial overlap makes the load retry until the store
   commits (deadlock-free — the blocking store is older).
3. **Stores never reported completion to the ROB** → any program with a store deadlocked. Fixed:
   LSU drives its writeback port for stores too (`writesReg=false`, ROB marks `done`). Single wb
   port arbitrated: stage-2 load > forwarding load > store completion.
4. **Missed-wakeup race in `BusyTable`.** A uop renamed in the same cycle its producer wrote back
   waited forever (registered busy bit cleared at T+1, but IQ wakeup skipped a not-yet-valid slot).
   Fixed: source ready = `!busy OR being written back this cycle` (`clearedThisCycle`).
5. **Branch predictor was tied off** (gshare/BTB/RAS never learned). Wired `ROB.brUpdate` (new
   `BrUpdate` bundle; `RobEntry` now records `brTaken`/`brTarget` for EVERY resolved branch) to
   `Fetch`'s predictor update, trained **at commit** (so only architectural branches train; GHR
   needs no snapshot). Result: mispredicts −52% on `bench.c`.
6. **Faster divider:** `divBitsPerCycle=4` (8-cycle divide vs 32). `bench.c` IPC 0.167→0.368.
   RV32M ISA 8/8 still pass. Higher radix = fewer cycles but longer combinational subtract chain
   (Fmax tradeoff on FPGA); tunable via the param.

---

## 4. Memory map (`OoOParams`, byte addresses — matches Hopper)

| Address | Function |
|---|---|
| `0x00000000 .. memWords*4-1` | RAM (shared instruction + data) |
| `0x08000004` | hardware timer, microseconds (Doom's `DG_SleepMs` polls this) |
| `0x08000008 .. 0x08000033` | keyboard bitmap (`io.keys`; low bits = 4 buttons) |
| `0x08000034` | UART TX |
| `0x10000000` | framebuffer base (fbWidth*fbHeight words) |
| `0x70000000` | debug char out (→ Chisel `printf` to sim console) |
| `0x70000008` | debug hex-word out (→ `printf`) |
| `0x70000010` | **tohost mailbox — OUR addition, not Hopper's.** Tests/programs write result here |

`Memory.scala` decodes all of these on port 2, drives `printf` for the debug ports, and has a
`HardwareTimer` (a free-running microsecond counter — mandatory; without it Doom's sleep loop
never terminates). LSU treats any word address `>= memWords` as MMIO (waits until ROB head for
MMIO loads).

---

## 5. Test infrastructure

- **Suites:** `ExecuteSpec` (22: PhysRegFile/ExecAlu/ExecMulDiv, incl. a dense DIVU/REMU sweep),
  `FrontendSpec` (4: Decode/Rename/Frontend), `MemCommitSpec` (8: ROB/LSU incl. SB/SH regression),
  `MainSpec` (1: whole-SoC smoke), `ProgramSpec` (compiled C), `IsaSpec` (48 riscv-tests, generated
  in a loop), `DoomSpec` (1: runs Doom). The RTL/unit suites all pass. **ISA compliance is 48/48
  (rv32ui+rv32um) — the key correctness gate.** `ProgramSpec`/`IsaSpec`/`DoomSpec` **cancel
  cleanly** (not fail) when their hex images are absent (no toolchain / fresh clone / no Doom
  clone). Apply the flaky-Verilator caveat from §1 when reading any failure.
- **`src/test/scala/RISCV/SocHarness.scala`** — shared Core+Memory harness. Exposes: tohost
  mailbox (latched, `tohost_seen`/`tohost_data`), hardware perf counters
  (`perf_cycles/commits/redirects/divbusy/headstall/headstalldiv`), commit trace, framebuffer
  capture (a **combinational-read `Mem`**, scanned out via `fb_read_addr`/`fb_read_data` with NO
  clock step so reads don't advance the CPU), and committed-PC range (`pc_min/max/last`,
  resettable).
- **Fast simulation (essential):** the testbench steps in BATCHES (`dut.clock.step(5000)`), not
  per cycle — each `peek`/`step` is a JVM↔Verilator round-trip. Per-cycle polling ran ~670
  cycles/sec; batched + hardware counters gives ~400-485k cycles/sec. Also uses
  `OptimizeForSimulationSpeed` (adds `-O3 --x-assign fast`). ChiselSim exposes NO Verilator
  `--threads` option (checked the svsim source) — we're already at its optimization ceiling.
- **Memory preload:** `OoOParams.memInitFile` → `loadMemoryFromFileInline` (`$readmemh`) in
  `Memory`. **REQUIRES randomization disabled** — ChiselSim's default `RANDOMIZE_MEM_INIT`
  clobbers the `$readmemh` init. Pass `settings = Settings.defaultRaw[T].copy(randomization =
  Randomization.uninitialized)` (helper `noRandom` in ProgramSpec/DoomSpec). Flashing a Doom-sized
  image through the 1-word/cycle flash port measured ~127 min; preload is instant.
- **riscv-tests:** clone to `sw/riscv-tests/` (gitignored), then `sw/isa/build_isa.sh` builds 48
  rv32ui+rv32um images against our **custom env** `sw/isa/riscv_test.h` (the stock env uses CSRs/
  `ecall`/`mret` which this core does NOT implement — exit is via the tohost mailbox instead).
  Skipped: `fence_i` (no FENCE.I/icache), `ma_data` (deliberate misaligned access; LSU has none).
- **Compiled-C tests:** `sw/build.sh sw/tests/X.c` → `sw/build/X.hex`. `sw/crt0.S`, `sw/link.ld`.
  Programs report via tohost (0x70000010). `sw/tests/`: `bytes.c` (SB/SH), `arith.c` (RV32IM),
  `timer.c` (timer), `bench.c` (prime sieve — divide-heavy benchmark).
- **CI:** `.github/workflows/test.yml` — fast job installs the RISC-V toolchain, rebuilds the C
  hexes, fails if committed `sw/build/*.hex` are stale, runs `sbt test`; a nightly/dispatch job
  runs the full ISA suite.

---

## 6. Performance findings (data-driven — these guide what to optimize)

Measured with the stall-attribution counters. **The core is healthy:**
- `arith.c` (ALU-bound): **IPC 0.99** (near-ideal for 1-wide).
- `bench.c` (prime sieve): was 0.167, **71% of cycles waiting on the divider** → the faster
  divider was the correct win (→ 0.368).
- **Doom init IPC 0.85; Doom render loop IPC 0.79, only 2.2% mispredict, 0% divider.**

Implications for the deferred perf backlog:
- **Branch-mask fast recovery: DATA SAYS LOW VALUE.** Doom's render loop mispredicts only ~2.2%
  and arith has ~0 mispredicts; the big invasive change (touches rename/IQ/LSU, removes the
  "no age comparisons" simplification) would barely help. Do NOT do it without fresh evidence.
- **Multiple loads in flight (MSHRs): FPGA-era only.** Sim memory is 1-cycle, so there is no load
  latency to hide; it only pays off against real DDR latency.

---

## 7. Doom (it renders!)

**Result:** builds, boots, and renders the **Freedoom Phase 1 title screen** (`doom_frame.png` at
repo root, converted from the sim framebuffer; ~95% non-black — a genuine render). Render-loop
IPC 0.79.

### How it's built (the Doom clone is NOT in the repo — it's gitignored & GPL)
Our port files live in **`sw/doom-port/`** (tracked); the clone + WAD are gitignored. See
`sw/doom-port/README.md`. Flow:
```bash
git clone https://github.com/MIT-OpenCompute/doom-hopper.git sw/doom
cp <a WAD> sw/doom/doomgeneric/freedm.wad        # e.g. Freedoom Phase 1 (freedoom1.wad, 27.5MB)
cp sw/doom-port/{wad.S,picolibc_glue.c,build_doom.sh,w_file_mem.c} sw/doom/doomgeneric/
( cd sw/doom/doomgeneric && ./build_doom.sh )     # -> sw/doom/doomgeneric/build/doom.hex
```
- **picolibc** supplies libc (the bare-metal toolchain ships none): `--specs=picolibc.specs
  --crt0=minimal`, its `picolibc.ld` with `__flash`/`__ram` defsyms (flash = low RAM holding code
  + the WAD; ram = high RAM for data/bss/heap/stack). `picolibc_glue.c` forwards POSIX names to
  the port's `_`-prefixed stubs, routes `printf` to the debug console, and provides `_exit`.
- Build flags: **`-march=rv32im -O2 -mstrict-align`** (we have hw mul/div; `-mstrict-align`
  because the LSU has no misaligned support). WAD embedded via `.incbin` (`wad.S`) — the upstream
  `#embed` needs GCC 15; we have 13. `build_doom.sh` patches the vendored port with `sed`/`perl`.
- **`DoomSpec.scala`** preloads `doom.hex` into a 64 MB RAM (`memWords = 16M`), runs it.
  Run: `sbt -Ddoom.cycles=200000000 "testOnly RISCV.DoomSpec"`. Props: `doom.cycles`,
  `doom.frames`/`doom.frameStride` (multi-frame capture to `build/frames/`), `doom.clockHz`,
  `doom.divK`, `doom.random`.

### Doom fixes (software, applied by `build_doom.sh`; the CORE was never the problem)
1. **Memory-direct WAD** (`w_file_mem.c` replaces `w_file_stdc.c`): reads the in-memory
   `doom1_wad` array directly and sets `wad->mapped` for **zero-copy lump access**. The upstream
   stdio path (`fread`/`getc`/`__bufio_*`) took ~600M instructions just to load the 21 MB WAD.
2. **`DG_SleepMs` infinite loop at -O2:** the port read the timer through a NON-volatile pointer;
   `-O2` hoisted the loop-invariant read out, making `while(1)`. Fix: `sed` makes the timer
   (`0x8000004`) and framebuffer (`0x10000000`) accesses `volatile`.
3. **Missing music was fatal:** `S_ChangeMusic` used `W_GetNumForName` (I_Errors if absent);
   patched to `W_CheckNumForName` + guard (non-fatal). Also switched from FreeDM to **Freedoom
   Phase 1** (a complete IWAD; FreeDM is a minimal deathmatch WAD lacking single-player assets).

### Known Doom issues (open)
- **Attract-mode demo does not play** → animation capture yields identical title frames. Freedoom's
  demo lumps are engine-version-incompatible (Doom demos are version-locked). The original
  shareware `doom1.wad` has Doom-1.9-compatible demos and would likely play, but it's copyrighted
  (freely-distributable shareware; the user can supply it). Alternative: scripted input via
  `io.btns` to start a game (slow/fiddly at sim speed).
- **Framebuffer bottom bar (garbage band):** `DG_DrawFrame` downsamples Doom's 640×400 to our
  320×240 by reading source row `2*y`, but 400/2 = only 200 rows, so output rows 200-239 read past
  the screen buffer → garbage. Fix: target **320×200** (Doom's true aspect; loop `y<200`).
- Real-time / interactive play is impossible in simulation (~450k cycles/sec vs millions of
  instructions/frame) — that needs the FPGA path.

---

## 8. FPGA path (discussed, not started)

For real-time playable Doom. The dominant change is a **memory hierarchy**: our memory is
currently INSIDE the RTL (`SyncReadMem`); on FPGA, BRAM (~1-5 MB) can't hold Doom's ~48-64 MB, so
you need **caches + an AXI interface to external DDR3**. Board recommendation: **Nexys Video
(XC7A200T, 512 MB DDR3, HDMI)** or **Arty A7-100T (256 MB DDR3)**; Zynq (Zybo) gives DDR "for
free" via the PS. Use **LiteX/LiteDRAM** to avoid hand-integrating the Xilinx MIG. Sim the new
memory model first (Hopper's C++ harness models DDR as a `mock_ddr3`). Estimated Fmax 50-100 MHz
on Artix-7. Note Hopper ran its Doom in a **C++ Verilator harness** (`simulation/vga-image.cpp`),
not ChiselSim — a native C++ harness would help sim speed ~2-5× (threading + flags) but is
sim-only, doesn't help the core, and isn't worth the rewrite short of that.

---

## 9. Repo / git conventions

- Working tree is CLEAN at `7da3139`; all the above is committed. Commit history is descriptive.
- **Gitignored (do not commit):** `sw/doom/` (GPL clone), `sw/wad/` (WADs), `sw/isa/build/`,
  `sw/riscv-tests/`, `sw/build/*.{elf,bin,dis,map}`, plus the usual `build/`, `target/`, etc.
  **Committed:** `sw/doom-port/`, `sw/build/*.hex`, `sw/isa/*.{sh,ld,h}`, `sw/{crt0.S,link.ld,build.sh}`.
- **`sw/doom` is a nested git repo.** If you ever `git add` it, git tries to make it a submodule —
  it's gitignored to prevent that. Don't `git submodule add` it.
- User setting `includeCoAuthoredBy=false` — **do NOT add a Co-Authored-By trailer to commits.**
  Auto-compact is disabled (`autoCompactEnabled=false`).
- Only commit/push when the user asks.

---

## 10. Auto-memory

`~/.claude/projects/.../memory/` has `running-tests.md` (the WSL-not-Windows + flaky-Verilator
facts) indexed in `MEMORY.md`. This CLAUDE.md supersedes/expands it.
