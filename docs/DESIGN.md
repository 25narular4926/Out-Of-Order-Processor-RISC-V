# OoO RISC-V Core — Complete Design Document

A 1-wide **out-of-order RV32IM** CPU written in Chisel (Scala), verified against the RISC-V ISA
(48/48 rv32ui+rv32um) and demonstrated running **DOOM** (shareware `doom1.wad`) all the way into
live gameplay on E1M1.

This document is the deep reference: architecture and every design decision (§1–§4), how C
programs and DOOM were made to run on it (§5–§7), the bugs found and fixed (§8), the
optimizations that made DOOM reach a frame and animate (§9–§10), verification (§11), and the
known gaps (§12). Code snippets are quoted from the actual source; file/line references are
clickable in the repo.

> **Relationship to Hopper.** This core is a sibling to MIT OpenCompute's *in-order* RV32I core
> "Hopper" (<https://github.com/MIT-OpenCompute/hopper-cpu>, MIT-licensed). It **reuses Hopper's
> memory-map interface** so software written for Hopper — notably the DOOM port — runs here
> unmodified. The core microarchitecture is entirely original: Hopper has no ROB, rename, issue
> queue, or load/store queues. See `ATTRIBUTION.md`.

---

## Table of contents

1. [Design philosophy & the big architectural decisions](#1-design-philosophy--the-big-architectural-decisions)
2. [Parameters, the frozen bundle contract, and the memory map](#2-parameters-the-frozen-bundle-contract-and-the-memory-map)
3. [The pipeline, stage by stage (with code)](#3-the-pipeline-stage-by-stage-with-code)
4. [The recovery model (commit-time full flush)](#4-the-recovery-model-commit-time-full-flush)
5. [Running C programs on the core](#5-running-c-programs-on-the-core)
6. [ISA compliance harness](#6-isa-compliance-harness)
7. [Getting DOOM to run](#7-getting-doom-to-run)
8. [Issues encountered (and how they were fixed)](#8-issues-encountered-and-how-they-were-fixed)
9. [Optimizations](#9-optimizations)
10. [Capturing the animation](#10-capturing-the-animation)
11. [Verification & test infrastructure](#11-verification--test-infrastructure)
12. [Known limitations & gaps](#12-known-limitations--gaps)
13. [Performance data](#13-performance-data)
14. [What's next](#14-whats-next)

---

## 1. Design philosophy & the big architectural decisions

The core is 1-wide (one instruction fetched / decoded / renamed / dispatched / retired per
cycle) with an **out-of-order execution window**: instructions issue, execute, and write back in
dataflow order, but **retire in program order**. The shape is BOOM-like but deliberately
minimal.

```
Fetch → Decode → Rename → Dispatch   |   IssueQueue → {ExecAlu, ExecMulDiv, Lsu} → CDB
   (in order, 1-wide)                |      (out of order)                          |
                                     ROB (in-order retire) <-----------------------+
```

Five decisions shaped everything else:

**(D1) Correctness first, performance second — and every simplification is written down.**
The whole machine was brought up on a *commit-time full-flush* recovery model (§4) instead of
branch-mask fast recovery. This trades some misprediction latency for the removal of an entire
class of complexity (no branch masks, no rename snapshots, **no age comparisons** in the flush
path). The `MicroOp` even carries `brMask`/`brTag` fields — but they are **reserved and unused**,
documented as a future upgrade rather than silently half-implemented.

**(D2) A frozen interface contract.** Every module talks through the bundles in
[Bundles.scala](../src/main/scala/RISCV/Bundles.scala). Each field documents *who fills it* and
*who reads it*. The `MicroOp` is filled **progressively** — Decode fills the decoded operation,
Rename fills the physical tags, Dispatch stamps the queue indices — so the same packet flows
through the IssueQueue, ROB, LDQ, and STQ without repackaging.

**(D3) One parameter object.** Every structural size lives in
[`OoOParams`](../src/main/scala/RISCV/Params.scala); widths are *derived*, never hand-counted, so
the machine can be shrunk to close timing on a small FPGA or grown for a Doom-sized image by
changing one field.

**(D4) Reuse Hopper's memory map verbatim.** Adopting Hopper's MMIO addresses (timer, keyboard,
framebuffer, UART) means Doom's platform layer runs against this core *with no source edits*
(§2, §7).

**(D5) Simulation is a first-class concern.** DOOM is ~10⁸ instructions to the first frame.
Naïve per-cycle testbench polling ran at ~670 cycles/sec (it measures the JVM↔Verilator
round-trip latency, not simulation throughput). Moving all counting into hardware and stepping
the clock in large batches got that to ~400–485k cycles/sec — a ~700× speedup that is the only
reason Doom bring-up was tractable (§9).

**Fixed sizes (`OoOParams` defaults):** xlen 32, 32 arch regs, **64 physical regs**, **32-entry
ROB**, **16-entry issue queue**, **8 LDQ / 8 STQ**, 3 writeback ports (ALU, MULDIV, MEM),
`divBitsPerCycle = 4`, `memWords = 4096` (Doom overrides to 16M), gshare with 512×2-bit counters
and 9 history bits, 16-entry BTB, 4-entry RAS.

---

## 2. Parameters, the frozen bundle contract, and the memory map

### 2.1 `OoOParams`

All sizing and every derived width is centralized. Widths come from `log2Ceil`, and `require`
guards catch illegal configs at elaboration:

```scala
require(numPhysRegs >= numArchRegs + 1, "need at least one free physical register")
require(isPow2(numPhysRegs), "numPhysRegs must be a power of two")
require(robEntries >= 2 && isPow2(robEntries), "robEntries must be a power of two >= 2")
require(divBitsPerCycle >= 1 && xlen % divBitsPerCycle == 0,
  "divBitsPerCycle must be >= 1 and divide xlen")

def pregWidth: Int  = log2Ceil(numPhysRegs)   // physical register tag width
def robIdxWidth: Int = log2Ceil(robEntries)   // ROB index width
def zeroPreg: Int    = 0                        // physical reg permanently bound to x0
```

### 2.2 The memory map (byte addresses — identical to Hopper)

Defined as constants on `OoOParams` and decoded by [Memory.scala](../src/main/scala/RISCV/Memory.scala):

| Byte address | Function |
|---|---|
| `0x00000000 .. memWords*4-1` | RAM (shared instruction + data) |
| `0x08000004` | hardware timer, microseconds since reset (Doom's `DG_SleepMs` polls this) |
| `0x08000008 .. 0x08000033` | keyboard bitmap (11 words) |
| `0x08000034` | UART TX |
| `0x10000000` | framebuffer base (`fbWidth*fbHeight` words) |
| `0x70000000` | debug char out → Chisel `printf` to sim console |
| `0x70000008` | debug hex-word out → `printf` |
| `0x70000010` | **tohost mailbox — our addition, not Hopper's.** Tests/programs report results here |

The LSU works in *word* addresses (it computes `byteAddr >> 2`); anything at or above `memWords`
is treated as MMIO.

### 2.3 The `MicroOp` — the packet that flows through the whole machine

```scala
class MicroOp(p: OoOParams) extends Bundle {
    val pc = UInt(p.xlen.W)
    // decoded operation (Decode)
    val opcode, func3, func7, immediate; val fuType = FuType()   // ALU | MULDIV | MEM
    // architectural regs (Decode) — used for the commit map + debug
    val lrs1, lrs2, ldst
    // control flags (Decode)
    val readsRs1, readsRs2, writesReg, isBranch, isJump, isJalr, isLoad, isStore
    // renamed physical tags (Rename)
    val prs1, prs2, pdst, pdstOld, prs1Ready, prs2Ready
    // queue indices (Dispatch)
    val robIdx, ldqIdx, stqIdx
    // branch prediction (Fetch/Decode; checked at execute)
    val predTaken, predTarget
    // RESERVED for future branch-mask fast recovery — unused in first-pass recovery
    val brMask, brTag
}
```

Other key bundles: `WbPort` (the Common Data Bus — one per functional unit; carries the result,
`robIdx`, `pdst`, and branch resolution), `RobEntry`, `CommitSignal` (retirement broadcast),
`BrUpdate` (predictor training), and `Redirect` (the global flush).

---

## 3. The pipeline, stage by stage (with code)

`Core` ([Core.scala](../src/main/scala/RISCV/core/Core.scala)) owns almost no logic — it
instantiates seven subsystems (`frontend`, `iq`, `prf`, `execAlu`, `execMulDiv`, `lsu`, `rob`),
wires the Common Data Bus, and fans dispatch out. Everything below is the internals of those
subsystems.

### 3.0 The global timing contract (read this before any stage)

Two facts govern how every stage is written; get them wrong and nothing composes.

**(T1) The in-order front end is one combinational chain behind a 2-cycle fetch.** Fetch is a
2-stage pipe because instruction memory is a `SyncReadMem` (data returns one cycle after the
address). But **Decode, Rename, and Dispatch are all pure combinational pass-throughs** — each
sets `out.valid = in.valid` and `in.ready = out.ready` (Decode/Frontend.scala confirm the `<>`
wiring). So once Fetch presents a `FetchPacket` on its `out` port in some cycle *N*, that packet
flows *through Decode → Rename → Dispatch in the same cycle N*, and the resulting allocation
writes (ROB entry, IQ slot, LDQ/STQ entry, rename-map update, busy bit) all land on the **clock
edge ending cycle N**. There is no register between Decode and Dispatch. This is why the missed-
wakeup bug (§3.4) exists: a uop is *renamed and enqueued in the very same cycle*, so a producer
writing back that cycle must be visible through a combinational term, not a registered bit.

**(T2) Wakeup and execution are 1-cycle-late by construction, and everything is tuned to match.**
A result broadcast on the Common Data Bus (CDB) in cycle *T* is:
- *written* into the PhysRegFile on the edge ending *T* (clocked),
- *observed* by IssueQueue wakeup as a set ready bit that is only selectable in *T+1*,
- *observed* by the ROB as `done` in *T+1*,
- *observed* by the BusyTable as a cleared bit in *T+1* (plus the same-cycle bypass term of §3.4).

Because ExecAlu registers its output (1-cycle latency), its result also appears on the CDB one
cycle after it issues. So a dependent chain advances **one instruction per cycle** through the
ALU with no bubbles, and — critically — the PhysRegFile needs **no write-to-read bypass**, because
by the time a consumer is *selected* (T+1) its operand is already in the register file (written at
end of T). §3.7 makes this precise.

**Bus conventions (fixed, memorize):**
- CDB writeback ports: `wb(0) = ExecAlu`, `wb(1) = ExecMulDiv`, `wb(2) = Lsu`.
- PRF read ports: `0/1 = ALU rs1/rs2`, `2/3 = MULDIV rs1/rs2`, `4/5 = MEM rs1/rs2`.
- All speculative structures snoop the *entire* `Vec(numWbPorts, WbPort)`, not a single port.

```scala
val cdb = Wire(Vec(p.numWbPorts, new WbPort(p)))
cdb(0) := execAlu.io.wb;  cdb(1) := execMulDiv.io.wb;  cdb(2) := lsu.io.wb
iq.io.wb := cdb;  prf.io.wb := cdb;  rob.io.wb := cdb;  frontend.io.wb := cdb
```

**Structural allocation is combinational-index / registered-commit.** The ROB and LSU expose the
index they *would* allocate next (`allocIdx = current tail`) combinationally; Dispatch stamps that
index into the uop *before* it raises the alloc request, then the structure commits the write on
the clock edge. This is why `robIdx`/`ldqIdx`/`stqIdx` are already valid inside the uop the moment
it is dispatched — no extra handshake round-trip.

### 3.1 Fetch + branch predictor

[Fetch.scala](../src/main/scala/RISCV/core/frontend/Fetch.scala).

**Interface.** `enable` (Bool), `redirect` (`Redirect`), `brUpdate` (`BrUpdate`, from the ROB),
`imemAddr` (out, 32-bit *byte* address), `imemData` (in, 32-bit word), `out`
(`Decoupled(FetchPacket)`). The `FetchPacket` it produces is `{pc, instruction, predTaken,
predTarget}`.

**State (the registers that make it a pipeline).**
- `pc` — `Reg(UInt(32.W))`, reset 0 (the core's reset PC). Drives `imemAddr` every cycle;
  advances only on `s0Fire`.
- S1 latches: `s1Valid` (RegInit false), `s1Pc`, `s1PredTaken`, `s1PredTarget`. These hold the
  metadata for the address that was issued *last* cycle, so it lines up with `imemData` this cycle.
- Skid latches: `skidValid` (RegInit false), `skidPkt` (a full `FetchPacket`).

**The two stages and their timing.**

```
cycle N   (S0): imemAddr = pc; predictor.predict(pc) combinationally → predTaken/predTarget.
                If s0Fire: register {s1Pc,s1PredTaken,s1PredTarget} := {pc,pred...}, s1Valid:=1,
                and advance pc := predTaken ? predTarget : pc+4.
cycle N+1 (S1): imemData now holds the instruction for the address issued in N.
                freshPkt = {s1Pc, imemData, s1PredTaken, s1PredTarget}. Present on `out`.
```

So the memory's 1-cycle read latency is absorbed by registering the *prediction metadata*
alongside the address, then pairing it with the instruction word when it arrives.

**Backpressure — the 1-deep skid buffer.** `out` may be back-pressured (`out.ready` low) while an
instruction has already been fetched. A pure "freeze S0" scheme would lose that instruction, so
the fetched packet is parked:

```scala
val s1WillDrain = s1Valid && io.out.ready && !skidValid
val hasRoom     = !skidValid && (!s1Valid || s1WillDrain)
val s0Fire      = io.enable && hasRoom && !io.redirect.valid
// output prefers the OLDER instruction in the skid buffer over the fresh S1 one:
io.out.valid := (skidValid || s1Valid) && !io.redirect.valid
io.out.bits  := Mux(skidValid, skidPkt, freshPkt)
```

The rule: S0 issues a new address only when there is somewhere for next cycle's instruction to
go — i.e. the skid is empty *and* S1 is either empty or draining this cycle. When S1 holds an
instruction the consumer didn't take, it moves to the skid (`skidPkt := freshPkt; skidValid := 1`)
and S0 stalls. This never drops or duplicates a fetch; it costs at most one fetch slot of latency
under back-pressure.

**Redirect.** On `redirect.valid`: `pc := redirect.target`, `s1Valid := 0`, `skidValid := 0`.
Everything in flight is younger than the redirecting instruction (full-flush model, §4), so
squashing it is always correct.

**Worked micro-trace (a downstream stall).** Say Decode is ready every cycle until cycle 5:
```
cyc  pc    imemAddr  s1Valid  imemData      out.ready  action
 1   0x40  0x40       0        -             1          s0Fire: s1<={0x40}, pc→0x44
 2   0x44  0x44       1        instr@0x40    1          out=instr@0x40; s0Fire: s1<={0x44}, pc→0x48
 3   0x48  0x48       1        instr@0x44    0 (stall)  out=instr@0x44 not taken → skid<=instr@0x44
 4   0x48  0x48       0        -             0          out=skid(instr@0x44) held; S0 frozen (no room)
 5   0x48  0x48       0        -             1          out=skid drains; s0Fire resumes: pc→0x4C
```

**The predictor** ([BranchPredictor.scala](../src/main/scala/RISCV/core/frontend/BranchPredictor.scala))
— gshare + BTB + RAS, all combinational on the predict side.

*State:* `ghr` (`Reg(UInt(ghistBits.W))` = 9-bit global history), `bht`
(`512 × 2-bit` saturating counters, reset to `2` = *weakly taken*), a direct-mapped BTB of 16
entries (`btbValid`/`btbTag`/`btbTarget`, tag = the PC bits above the index), and a 4-entry `ras`
with a `rasPtr`.

```scala
// gshare index: drop the 2 low (word-aligned) PC bits, take ghistBits of PC, XOR the history
private def bhtIndex(pc: UInt): UInt =
    ((pc >> 2.U)(p.ghistBits - 1, 0) ^ ghr)(bhtIdxBits - 1, 0)

val dirTaken = bht(pIdx)(1)                                    // MSB of the 2-bit counter
val btbHit   = btbValid(bIdx) && (btbTag(bIdx) === btbTagOf(io.predict.pc))
io.predict.taken  := dirTaken && btbHit    // taken ONLY if the counter says taken AND we know a target
io.predict.target := btbTarget(bIdx)
```

*Update (synchronous, one branch per cycle):* the 2-bit counter saturates toward the resolved
direction; `ghr := Cat(ghr(ghistBits-2,0), taken)` shifts the new outcome into the LSB; and on a
*taken* branch the BTB entry is (re)allocated with the resolved target.

**Decision — train at *commit*, not writeback.** The `update` port is driven from the ROB's
`brUpdate` when a branch *retires* (§3.11), not when it resolves on the ALU. Because the ROB
retires in program order, only architecturally-executed branches ever train the tables — wrong-
path branches never pollute them, and *the global history register holds only committed outcomes,
so it needs no snapshot/restore across a flush.* That is precisely what makes the full-flush
recovery model cheap on the predictor side. The cost is training latency (a branch in a tight loop
may be re-fetched before its first instance retires, so the first iterations still mispredict) —
an accepted trade. The RAS is implemented but *not consulted by the predict port*, because
identifying calls/returns needs pre-decode at fetch (`rd`/`rs1 == x1`) that a pure-PC predict port
doesn't have — a documented first-pass simplification.

### 3.2 Decode

Two files: the combinational field extractor
[Decoder.scala](../src/main/scala/RISCV/core/Decoder.scala), wrapped by the stage
[Decode.scala](../src/main/scala/RISCV/core/frontend/Decode.scala) that turns a `FetchPacket` into
a partially-filled `MicroOp`. Both are stateless; `out.valid = in.valid`, `in.ready = out.ready`,
so there is nothing to flush (validity is carried by the Decoupled and Fetch squashes upstream).

**Field extraction.** The `Decoder` splits the 32-bit instruction into
`rs1(19,15)/rs2(24,20)/rd(11,7)/opcode(6,0)/func3(14,12)/func7(31,25)` and assembles the
sign-extended immediate for the instruction's format. The immediate is selected with a **nested
`Mux`** keyed on opcode rather than a format-enum `switch`, because the enum-switch form makes
firtool emit a packed-array select (`_GEN[formatCode]`) that trips Verilator's `WIDTHEXPAND` lint
(fatal by default). Every arm is exactly 32 bits to stay lint-clean:

```scala
val iImm = Fill(20, instr(31)) ## instr(31, 20)
val bImm = Fill(20, instr(31)) ## instr(7) ## instr(30, 25) ## instr(11, 8) ## 0.U(1.W)
// U/J/S similarly, each exactly 32 bits
io.immediate := Mux(isU, uImm, Mux(isJ, jImm, Mux(isB, bImm, Mux(isS, sImm, Mux(isI, iImm, 0.U)))))
```

**Classification → `MicroOp` fields.** The Decode stage derives every Decode-owned field. The
functional unit is chosen by opcode; RV32M is the `OP` opcode with `func7 == 0000001`:

```scala
val isMulDiv = (opcode === OP_OP) && (func7 === "b0000001".U)
val fuType = Wire(FuType())
when(isLoad || isStore) { fuType := FuType.MEM }
    .elsewhen(isMulDiv)  { fuType := FuType.MULDIV }
    .otherwise           { fuType := FuType.ALU }
```

The register-use flags drive rename and the busy lookups, and encode the RISC-V format rules
exactly:

```scala
val readsRs1 = isLoad || isStore || isBranch || isJalr || (opcode===OP_OP) || (opcode===OP_OPIMM)
val readsRs2 = isStore || isBranch || (opcode===OP_OP)               // R/S/B read rs2
val writesRdOpcode = (opcode===OP_OP)||(opcode===OP_OPIMM)||(opcode===OP_LUI)||
                     (opcode===OP_AUIPC)|| isLoad || isJump || isJalr
val writesReg = writesRdOpcode && (rd =/= 0.U)                       // x0 never "writes"
```

The `writesReg && rd != x0` term matters downstream: a uop whose `rd` is x0 (or that is a
store/branch) allocates **no** physical destination, so Rename doesn't pop the free list and the
ROB frees nothing at retire for it. The stage starts from `MicroOp.nop` (all-zero), so every field
a later stage owns (`prs*`, `pdst*`, `robIdx`/`ldqIdx`/`stqIdx`, ready bits, `brMask`/`brTag`) is
explicitly 0 leaving Decode — the "fields not yet filled must be 0, not dangling" rule from the
`MicroOp` contract.

### 3.3 Rename (MapTable + FreeList + BusyTable)

[Rename.scala](../src/main/scala/RISCV/core/frontend/Rename.scala) breaks *false* (WAR/WAW)
dependencies by mapping the 32 architectural registers onto the 64-entry physical register file.
It instantiates the three supporting structures and is combinational (`in`/`out` Decoupled).

**The fire condition.** Rename can proceed this cycle only if downstream is ready *and*, when the
uop writes a register, a free physical register exists:

```scala
val pregAvailable = !freeList.io.empty
val canRename     = !writesReg || pregAvailable
val fire          = io.in.valid && io.out.ready && canRename && !io.redirect.valid
io.in.ready       := io.out.ready && canRename && !io.redirect.valid   // stall Decode if no preg
```

So a `writesReg` uop that finds the free list empty **back-pressures the whole front end** until a
commit frees a preg. All structure updates are gated on `fire`, so a redirect (which forces
`fire = false`) lets no speculative update slip through — the structures' own redirect handling
takes over instead.

**Per-instruction dataflow** (combinational reads from the speculative map, writes on `fire`):

```
prs1 = specMap[lrs1];  prs2 = specMap[lrs2]              // x0 → zeroPreg, always "ready"
if writesReg:
    pdst    = freeList.pop()          // lowest-numbered free preg
    pdstOld = specMap[ldst]           // previous owner of ldst — freed when THIS uop retires
    specMap[ldst] := pdst             // install the new mapping
    busy.set(pdst)                    // the new value is not available yet
else:
    pdst = pdstOld = zeroPreg
```

The renamed uop carries `prs1/prs2` (masked to `zeroPreg` when the instruction doesn't read that
source, to avoid spurious wakeups), `pdst`, `pdstOld`, and the two ready bits read from the
BusyTable.

**MapTable — two maps, with same-cycle commit folding**
([MapTable.scala](../src/main/scala/RISCV/core/frontend/MapTable.scala)). It holds a
`specMap` (updated at rename, read for sources/old-dest) and a `commitMap` (updated at retire).
Both reset to the *identity* map (arch *i* → preg *i*), which is why the free list starts with
pregs `[numArchRegs, numPhysRegs)` free. The subtle part is the redirect + concurrent-commit case:

```scala
when(io.redirect) {                                  // revert speculation to committed state
    for (i <- 0 until p.numArchRegs) specMap(i) := (if (i==0) zeroPreg else commitMap(i))
    when(io.commitValid && io.commitLdst =/= 0.U) {  // ...but a commit landing THIS cycle is older
        specMap(io.commitLdst) := io.commitPdst      // than the flushed work, so fold it in
    }
}.elsewhen(io.allocValid && io.allocLdst =/= 0.U) {  // normal rename
    specMap(io.allocLdst) := io.allocPdst
}
```

It also exposes a `committedMap` output with the same-cycle commit folded in *combinationally*, so
that when a redirect and a commit coincide, the FreeList rebuild (below) sees the just-committed
mapping as architecturally live.

**FreeList — a bitmap with a committed-map rebuild**
([FreeList.scala](../src/main/scala/RISCV/core/frontend/FreeList.scala)). One `free` bit per preg;
allocate = pop the lowest free (`PriorityEncoder`), commit = push `pdstOld` back. Redirect can't
just "undo pops" (there are no snapshots), so it *rebuilds the whole set* from the committed map —
a preg is free iff no non-x0 architectural register maps to it and it isn't `zeroPreg`:

```scala
when(io.redirect) {
    for (pr <- 0 until p.numPhysRegs) {
        val mappedByAny = io.committedMap.zipWithIndex.map { case (m, arch) =>
            (arch != 0).B && (m === pr.U) }          // any arch reg (except x0) maps to pr?
        free(pr) := !((pr.U === zeroPreg.U) || VecInit(mappedByAny).asUInt.orR)
    }
}.otherwise {
    when(doAlloc) { free(allocIdx) := false.B }                       // pop
    when(io.freeReq && io.freeTag =/= zeroPreg.U) { free(io.freeTag) := true.B }  // push at commit
}
```

This bitmap-rebuild is *the* reason the full-flush model is cheap in rename: recovery is a single
combinational recompute, no per-branch snapshot RAM. (The cost of adding branch-mask recovery
later, §4, is precisely that you'd have to snapshot this on every branch.)

### 3.4 The BusyTable and the missed-wakeup subtlety

[BusyTable.scala](../src/main/scala/RISCV/core/frontend/BusyTable.scala) is one bit per physical
register: set at rename, cleared when a CDB port reports that tag. The critical detail is the
`clearedThisCycle` term:

```scala
// A source is ready if it is NOT busy, OR its value is appearing on the bus right now.
io.rs1Ready := (io.rs1 === p.zeroPreg.U) || !busy(io.rs1) || clearedThisCycle(io.rs1)
```

Why this matters (this was a real deadlock — see §8): the busy bit is a **register**, so a
writeback at cycle T only clears it at T+1. But Rename and Dispatch are combinationally chained
(§3.0-T1), so a uop is renamed *and* enqueued into the IssueQueue in the same cycle T — and the
IQ's own wakeup is gated on `when(valids(i))`, but the new slot isn't valid until T+1. Without
`clearedThisCycle`, a uop renamed in the exact cycle its producer writes back latches
`ready = false`, misses the only broadcast of that tag, and waits **forever**.

**Set-over-clear priority.** In the register update, the new-destination *set* takes priority over
a same-cycle *clear* of the same tag, because a freshly-renamed producer is by definition not yet
done — even if some *other* op with the same physical tag (impossible under correct renaming, but
guarded anyway) were clearing:

```scala
when(io.redirect) { for (i ...) busy(i) := false.B }        // full flush: nothing speculative survives
.otherwise {
    for (port <- io.wb) when(port.valid && port.writesReg && port.pdst =/= zeroPreg.U) {
        busy(port.pdst) := false.B }                         // clear on writeback
    when(io.setValid && io.setTag =/= zeroPreg.U) { busy(io.setTag) := true.B }  // set wins
}
busy(p.zeroPreg) := false.B                                  // x0's preg is never busy
```

### 3.5 Dispatch

[Dispatch.scala](../src/main/scala/RISCV/core/frontend/Dispatch.scala) is the last in-order stage.
It stamps the back-end indices into the renamed uop and fires an **atomic** (all-or-nothing)
allocation into the ROB + IssueQueue + (for memory ops) the LDQ/STQ.

**Atomicity.** A uop dispatches only when *every* resource it needs has room; if any is full it
holds the instruction and allocates nothing, so no resource is ever half-claimed:

```scala
val robOk = !io.robFull
val iqOk  = io.iqReady
val ldqOk = !uop.isLoad  || !io.ldqFull      // only loads need an LDQ slot
val stqOk = !uop.isStore || !io.stqFull      // only stores need an STQ slot
val fire  = io.in.valid && (robOk && iqOk && ldqOk && stqOk) && !io.redirect.valid
io.in.ready := (robOk && iqOk && ldqOk && stqOk) && !io.redirect.valid   // back-pressure Rename
```

**Index stamping (combinational hand-off, §3.0).** The ROB and LSU present the index they would
allocate next; Dispatch copies them into the uop and raises `dispatchValid = fire`, and the Core
wires that single `dispatchValid` to *all* the alloc requests at once
(`rob.allocReq`, `iq.enq.valid`, and `lsu.{ldq,stq}AllocReq` gated by `isLoad`/`isStore`):

```scala
val stamped = WireInit(uop)
stamped.robIdx := io.robAllocIdx; stamped.ldqIdx := io.ldqAllocIdx; stamped.stqIdx := io.stqAllocIdx
io.dispatchUop := stamped;  io.dispatchValid := fire
```

Because `dispatchValid` already folds in `robFull`, `iqReady`, the conditional queue-full checks,
*and* rename's free-list back-pressure, the whole "can this instruction enter the out-of-order
window?" question is one signal.

### 3.6 IssueQueue — unified, BOOM-style age matrix

[IssueQueue.scala](../src/main/scala/RISCV/core/execute/IssueQueue.scala) is a single 16-entry
queue serving all three FU classes. Each slot holds a `MicroOp` plus two source-ready bits.

**Ready rule:** `valid && (rs1Rdy || !readsRs1) && (rs2Rdy || !readsRs2)`.

**Age model — an age matrix, not a counter** (chosen to avoid wraparound foot-guns).
`olderThan(i)(j) == true` means slot *i* was enqueued before slot *j*. On enqueue into slot *k*,
every currently-valid slot becomes older than *k*:

```scala
when(io.enq.fire) {
    valids(freeIdx) := true.B;  uops(freeIdx) := io.enq.bits
    rs1Rdy(freeIdx) := io.enq.bits.prs1Ready;  rs2Rdy(freeIdx) := io.enq.bits.prs2Ready
    for (j <- 0 until n) {
        olderThan(freeIdx)(j) := false.B                 // newcomer is older than nobody
        when(valids(j)) { olderThan(j)(freeIdx) := true.B }  // every live slot is older than it
    }
}
```

**Selection** picks, per FU, the *oldest* ready candidate — the one no other ready candidate is
older than:

```scala
val winnerOH = VecInit((0 until n).map { i =>
    cand(i) && !(0 until n).map(j => cand(j) && olderThan(j)(i)).reduce(_ || _)
})
```

**Interface & slot state.** `enq` (`Flipped(Decoupled(MicroOp))`), `wb` (the CDB Vec), three
`Decoupled(MicroOp)` issue ports (`issueAlu`/`issueMulDiv`/`issueMem`), `redirect`. Slot state is
parallel vectors: `valids`, `uops`, `rs1Rdy`, `rs2Rdy`, and the `olderThan` matrix. The ready
bits live *separately* from the stored uop because wakeup mutates them while the uop fields are
otherwise immutable.

**Wakeup, enqueue, dealloc.** Wakeup snoops every CDB port against every valid slot's source tags:

```scala
for (i <- 0 until n) when(valids(i)) { for (port <- io.wb) {
    val hit = port.valid && port.writesReg && port.pdst =/= zeroPreg.U
    when(hit && uops(i).prs1 === port.pdst) { rs1Rdy(i) := true.B }
    when(hit && uops(i).prs2 === port.pdst) { rs2Rdy(i) := true.B }
}}
when(io.issueAlu.fire)    { valids(aluIdx) := false.B }   // free the slot when its port fires
when(io.enq.fire)         { valids(freeIdx) := true.B; ... }  // fill the first free slot (PriorityEncoder)
```

**Wakeup is 1-cycle-late:** a CDB broadcast at T sets the slot's ready bit, observed for selection
at T+1. This deliberately matches ExecAlu's registered 1-cycle output (its result lands on the CDB
one cycle after issue anyway), so a same-cycle speculative wakeup would buy nothing here (it *would*
help a longer-latency producer feed a dependent, which is why the notes list speculative wakeup as
a future win). Because the age relation is gated by validity, flush = clear all `valids` (the
matrix self-clears); the code also resets the matrix for determinism.

### 3.7 PhysRegFile — deliberately no bypass

[PhysRegFile.scala](../src/main/scala/RISCV/core/execute/PhysRegFile.scala) is 64×32-bit with 6
combinational read ports. It has **no write-to-read bypass, on purpose:**

```
Because wakeup is 1-cycle-late, a result broadcast on the CDB at T is not observed for selection
until T+1 — by which time the value has already landed in `regs` (the write is clocked at end of
T). A same-cycle bypass would be redundant AND would create a combinational loop at the Core
level: cdb → prf.wb → prf.read(bypass) → lsu operands → lsu's combinational forwarded-load
writeback → cdb. Reading only the register breaks that loop.
```

### 3.8 ExecAlu — 1-cycle, resolves branches

[ExecAlu.scala](../src/main/scala/RISCV/core/execute/ExecAlu.scala) computes the arithmetic
result *and* branch resolution combinationally, then registers the `WbPort` (1-cycle latency,
`req.ready` always high). ADD/SUB and SRL/SRA are selected by `func7(5)`. Branch/jump resolution
produces the actual next-PC and the mispredict flag:

```scala
val predNextPc = Mux(uop.predTaken, uop.predTarget, pc + 4.U)
val mispred    = isCtrl && (predNextPc =/= actualTarget)
// ...
wbReg.isBranchResolved := io.req.fire && isCtrl
wbReg.mispredicted     := isCtrl && mispred
wbReg.brTaken := brTaken;  wbReg.brTarget := actualTarget   // recorded for EVERY resolved branch
```

### 3.9 ExecMulDiv — combinational MUL, iterative divide

[ExecMulDiv.scala](../src/main/scala/RISCV/core/execute/ExecMulDiv.scala):

- **Multiplies** (MUL/MULH/MULHSU/MULHU) are combinational (full 64-bit product, requested half
  registered) — 1-cycle latency like the ALU.
- **Divides/remainders** use an iterative **restoring divider** with a **radix knob**,
  `divBitsPerCycle` (K). One step shifts the (remainder:quotient) pair, trial-subtracts, and sets
  the quotient bit if it fit; that step is **unrolled K times combinationally**, so each cycle
  retires K bits and a divide takes `xlen/K` cycles:

```scala
val K = p.divBitsPerCycle
def step(rem: UInt, quot: UInt): (UInt, UInt) = {
    val shifted = Cat(rem(xlen - 2, 0), quot(xlen - 1))
    val fits    = shifted >= divisorReg
    (Mux(fits, shifted - divisorReg, shifted), Cat(quot(xlen - 2, 0), fits.asUInt))
}
var r = remainder; var q = quotient
for (_ <- 0 until K) { val (nr, nq) = step(r, q); r = nr; q = nq }
```

`K=4` ⇒ 8-cycle divide (default). Higher K = fewer cycles but a longer combinational subtract
chain (K subtractors in series), so it trades divide latency against Fmax on FPGA.

**The divide FSM and handshake.** Three states `sIdle → sBusy → sDone`. `req.ready` is high only in
`sIdle`, so the IssueQueue (which only frees a slot when its issue port *fires*) naturally stalls
new MULDIV ops while a divide is in flight — the Core needs no fixed-latency assumption:

```scala
io.req.ready := state === sIdle
when(state === sIdle) { when(acceptDiv) { /* latch magnitudes, signs, robIdx/pdst; state:=sBusy */ }}
  .elsewhen(state === sBusy) { /* K restoring steps; count += K; when(count === xlen-K) state:=sDone */ }
  .elsewhen(state === sDone)  { state := sIdle }
when(io.redirect.valid) { state := sIdle }              // abort an in-flight divide on flush
```

At start it captures the operands' magnitudes (signed division works on magnitudes, sign
reapplied at the end), the div-by-zero and `INT_MIN/−1` overflow flags, and the destination
bookkeeping (`divRobIdx`/`divPdst`/`divWrites`) needed to build the `WbPort` when it finishes.

**Writeback timing.** MUL registers its result (`mulValid := io.req.fire && isMulOp`), appearing on
the CDB the cycle after issue. DIV pulses `wb.valid` for exactly one cycle in `sDone`
(`divFinishing = (state === sDone) && !redirect`). A redirect squashes both an in-flight multiply
(`mulValid := false`) and a divide (FSM → `sIdle`, no writeback), so no stale result reaches the
CDB.

**RV32M corner cases** are applied to `divOut` explicitly, with div-by-zero and overflow taking
priority over the normal result: ÷0 ⇒ quotient `-1` / remainder = dividend; `INT_MIN / -1` ⇒
quotient `INT_MIN`, remainder 0.

### 3.10 LSU — LDQ/STQ, per-byte stores, store-to-load forwarding

[Lsu.scala](../src/main/scala/RISCV/core/memory/Lsu.scala) is the most intricate unit. It holds a
Load Queue and Store Queue (8 each), allocated in program order at dispatch. Memory ops compute
their address (and, for stores, their data) when their MEM uop *issues*, and touch architectural
memory under the in-order-commit rule:

- **Stores are speculative until they retire.** A store records addr+data into its STQ slot at
  issue; the **actual memory write happens only at commit**, draining the oldest store.
- **Loads** may read memory speculatively, but only once all *older* stores have known addresses
  (conservative disambiguation). If an older store to the same word covers all the load's bytes,
  the load **forwards** that store's data instead of reading memory.

**Per-byte write enables** make SB/SH exact with no read-modify-write. The memory stores
`Vec(4, UInt(8.W))` and takes a 4-bit mask; the LSU computes it:

```scala
def storeMask(func3: UInt, byteOff: UInt): UInt = {
    val m = WireDefault("b1111".U(4.W))
    switch(func3) {
        is("b000".U) { m := (1.U(4.W) << byteOff)(3, 0) }            // SB  → one lane
        is("b001".U) { m := Mux(byteOff(1), "b1100".U, "b0011".U) }  // SH  → two lanes
        is("b010".U) { m := "b1111".U }                              // SW  → all four
    }
    m
}
```

**Store-to-load forwarding is coverage-checked.** The load forwards from the *youngest older*
store whose word address matches — but only if that store writes *every* byte lane the load
reads:

```scala
val fwdCovers = (reqLoadMask & ~fwdStoreMask) === 0.U
```

A partial overlap (e.g. an SB then an overlapping LW) cannot be satisfied — the load's remaining
bytes live in memory and we can't read memory and forward in the same cycle — so the load is
simply **not accepted** and the IssueQueue retries it. This is **deadlock-free** because the
blocking store is older and therefore retires first, after which the load reads correct memory.

Other rules: **MMIO loads wait until they are the ROB head** (no speculative peripheral reads);
the single memory port gives **commit stores priority** over load reads; one load is in flight at
a time (a 2-stage load pipe; forwarded loads complete in a single cycle without the memory port).

**Entry layouts.** An `StqEntry` is `{valid, addrValid, dataValid, wordAddr(32), byteOff(2),
data(32), func3(3), robIdx, mmio}` — `addrValid`/`dataValid` flip from false to true when the
store *issues* (address + data computed). An `LdqEntry` is minimal (`{valid, robIdx}`) in this
first pass: a load executes directly off `req`, so the LDQ exists mainly for age/flush
bookkeeping and to mirror the STQ for a future load-address-tracking design. Both queues use the
same **extra-wrap-bit** circular-pointer scheme as the ROB (§3.11).

**The age key.** Ordering and forwarding compare instructions by their distance ahead of the ROB
head, which is monotonic within the in-flight window (window ≤ `robEntries`, so no aliasing):

```scala
def ageKey(robIdx: UInt): UInt = robIdx - io.robHeadIdx      // smaller key ⇒ older
```

**The issue-stage decision (all combinational off `req`).** When a MEM uop issues, the LSU
computes its effective word address and, for a load, decides among *forward*, *go to memory*, or
*block-and-retry*:

```scala
val olderUnknownStore = /* any valid older store whose addrValid is still false */
val anyFwd  = /* an older store to the same word, addr+data known */
val fwdCovers = (reqLoadMask & ~fwdStoreMask) === 0.U     // store covers every byte the load reads
val mmioBlocked = reqMmio && (reqUop.robIdx =/= io.robHeadIdx)

val loadCanForward = reqIsLoad && !olderUnknownStore && !mmioBlocked && anyFwd && fwdCovers
val loadNeedsMem   = reqIsLoad && !olderUnknownStore && !mmioBlocked && !anyFwd
val loadMemGo      = loadNeedsMem && !doStoreDrain /*port free*/ && !ld2Valid /*pipe free*/
```

A **partial, uncoverable overlap leaves both `loadCanForward` and `loadNeedsMem` false**, so
`req.ready` is not asserted and the IssueQueue simply retries the load later — deadlock-free
because the blocking store is older and drains first.

**The 2-stage load pipe.** A memory-read load launches `memRead` in its issue cycle and registers
`{ld2Valid, ld2RobIdx, ld2Pdst, ld2Writes, ld2Func3, ld2ByteOff}`; the *next* cycle stage 2 takes
the returned `memReadData`, formats it (`formatLoad`, sign/zero-extend per `func3`), and writes
back. A forwarded load skips stage 2 entirely.

**The single writeback port is strictly arbitrated** — exactly one of three producers may drive it,
in priority order, so it can never be double-driven:

```scala
when(ld2Valid) { /* (1) stage-2 memory-read load result — highest priority, already in flight */ }
.elsewhen(reqFire && loadCanForward) { /* (2) forwarded load, same cycle it issues */ }
.elsewhen(reqFire && reqIsStore)     { /* (3) STORE COMPLETION: writesReg=false, only the ROB acts */ }
```

Case (3) is the fix for the store-deadlock bug (§8): a store produces no register value but *must*
still report to the ROB so its entry becomes `done`. The handshake refuses new store/forward work
whenever stage 2 is presenting a result (`wbPortBusy = ld2Valid`), which is what guarantees the
single port is enough.

**The store lifecycle, end to end:** dispatch (STQ slot allocated, `addrValid=dataValid=false`) →
issue (address+data recorded, both valid; store completion reported to ROB) → **commit** (the ROB
broadcasts the store as the STQ head; `doStoreDrain` drives `memWrite` with the per-byte mask, and
the STQ head pops). The actual RAM write is the *only* architectural effect and it happens exactly
once, at retirement, in program order.

### 3.11 ReorderBuffer — in-order retire, redirect, predictor training

[ReorderBuffer.scala](../src/main/scala/RISCV/core/commit/ReorderBuffer.scala) is a circular FIFO
of `RobEntry`, allocated in program order at dispatch and retired from the head. It uses the
**extra-wrap-bit** technique: head/tail are `(robIdxWidth+1)`-bit counters; the low bits index
the array, the top bit disambiguates full vs empty.

Each CDB port marks its entry `done` and, for branches, records the resolution. Notably it records
the outcome of **every** resolved branch (not just mispredicts) so the predictor can reinforce
branches it already gets right:

```scala
when(entries(port.robIdx).isBranchOrJump && port.isBranchResolved) {
    entries(port.robIdx).brTaken  := port.brTaken
    entries(port.robIdx).brTarget := port.brTarget
}
when(entries(port.robIdx).isBranchOrJump && port.mispredicted) {
    entries(port.robIdx).mispredicted   := true.B
    entries(port.robIdx).redirectTarget := port.brTarget
}
```

**Pointer scheme.** `head`/`tail` are `(robIdxWidth+1)`-bit counters; the low `robIdxWidth` bits
index the entry array and the extra top bit disambiguates the two colliding cases:

```scala
val isEmpty = head === tail
val isFull  = (headIdx === tailIdx) && (head(p.robIdxWidth) =/= tail(p.robIdxWidth))
io.allocIdx := tailIdx      // combinational: the index Dispatch stamps BEFORE raising allocReq
```

Allocation stamps a fresh entry at `tailIdx` (`valid=true, done=false`, plus the uop's
`pc/ldst/pdst/pdstOld/writesReg/isStore/isLoad/isBranchOrJump`) and bumps `tail`.

Retirement happens when the head is `valid && done`. It broadcasts `CommitSignal` (drives the
rename commit map + the LSU store drain) and, for a branch/jump, `BrUpdate` (trains the
predictor). If the retiring head is a mispredict or exception, it asserts the global `Redirect`.

**The flush-vs-retire ordering subtlety.** The normal retire path bumps `head` and clears the
head slot; the flush path resets `head := 0; tail := 0` and clears *every* valid bit. Both can be
generated the same cycle (the flushing instruction *is* retiring), so the code relies on Chisel's
**last-connect-wins**: the flush block is written *after* the retire block, so its assignments
override the `head+1`/clear-slot writes. Result: on a mispredict-at-head, the ROB is reset to
empty in one cycle and everything younger is dropped by construction — the essence of the
full-flush model (§4).

### 3.12 Worked cycle-by-cycle traces

These tie the stages together. Cycles are relative; "→CDB@T" means the result is broadcast on the
Common Data Bus in cycle T (written to the PRF at end of T, selectable/`done` at T+1, per §3.0-T2).

**(a) A dependent ALU pair — `add x1,x2,x3` then `add x4,x1,x5`.** Shows the 1-cycle-late wakeup
giving zero-bubble back-to-back dependent execution.

```
cyc  event
 N   add1 fetched (S1) → Decode/Rename/Dispatch combinationally (same cycle):
       rename: pdst(x1)=p33, busy(p33):=1; alloc ROB[k]; enqueue IQ slot A (both sources ready)
 N+1 add2 fetched → renamed: prs1(x1)=p33, pdst(x4)=p34; IQ slot B enqueued with rs1Rdy=FALSE (p33 busy)
       IQ selects slot A (oldest ready ALU) → issue add1; PRF reads p2,p3
 N+2 ExecAlu computes add1, registers WbPort → CDB@N+2 (pdst=p33). IQ wakeup sets slot B.rs1Rdy:=1
       (observed next cycle). PRF writes p33 at end of N+2.
 N+3 IQ selects slot B (now ready) → issue add2; PRF reads p33 (already written) + p5. No bubble.
 N+4 ExecAlu → CDB@N+4 (pdst=p34).
```

The reason there is **no bubble and no bypass**: add2 isn't *selected* until N+3, and p33 was
written at end of N+2 — so the plain registered PRF read in N+3 already returns the value (§3.7).

**(b) A load feeding a use — `lw x6,0(x7)` then `add x8,x6,x9`.**

```
cyc  event
 N   lw fetched → renamed pdst(x6)=p40, busy(p40):=1; LDQ+ROB alloc; IQ slot (MEM)
 N+1 IQ issues the MEM uop → LSU: addr=x7+0; no older unknown store, no forward ⇒ loadMemGo:
       drive memRead, latch ld2Valid + ld2Pdst=p40
 N+2 stage 2: memReadData formatted (LW ⇒ whole word) → CDB@N+2 (pdst=p40). add's IQ slot wakes.
 N+3 IQ issues add; PRF reads p40 → ExecAlu → CDB@N+3.
```

A **store-to-load forward** instead of a memory read collapses N+1/N+2 into a single cycle: if an
older STQ entry to the same word covers all the load's bytes, `loadCanForward` writes the CDB in
the issue cycle with no `memRead` and no stage 2.

**(c) A mispredicted branch (full-flush recovery).** `bne` predicted not-taken but actually taken;
some younger wrong-path instructions are already in the window.

```
cyc  event
 N   bne issues on ExecAlu; computes actualTarget, mispred=1 → CDB@N+1 with mispredicted=1,brTarget=T
 N+1 ROB records entries[bne].mispredicted:=1, redirectTarget:=T. bne is NOT yet the head, so no
       redirect fires. Wrong-path instructions keep flowing in behind it.
 ...  older entries retire one per cycle until bne reaches the head
 M   bne is head and done ⇒ canRetire & mispredicted ⇒ Redirect.valid, target=T. THIS cycle:
       Fetch pc:=T, squash S1+skid; IQ clears all valids; BusyTable clears all; LSU squashes
       LDQ + uncommitted STQ; Rename specMap:=commitMap, FreeList rebuilt; ROB head:=tail:=0.
       Also brUpdate trains the predictor with {pc=bne.pc, taken=1, target=T}.
 M+1 Fetch presents the instruction at T; the machine refills from a clean state.
```

The extra cost vs. branch-mask recovery is the **N+1 → M drain**: bne must reach the head before
it can redirect. §4 and the discussion of fast recovery quantify why that is an acceptable trade
for this workload (Doom render mispredicts ~2.2%).

---

## 4. The recovery model (commit-time full flush)

This is the single most important architectural decision, so it gets its own section.

**Model.** Branch mispredictions *and* exceptions both recover **in order, at the ROB head, via
one full flush.** When the offending instruction reaches the head and is `done`, the ROB asserts
`Redirect` — meaning "discard ALL speculative state and restart fetch at `target`." Because the
head is the *oldest* in-flight instruction, everything else in the machine is younger and is
simply cleared.

**Consequences (all deliberate):**
- **No branch masks, no rename snapshots, no age comparisons** in the flush path.
- Every speculative structure clears on `redirect.valid`: Fetch squashes S1 + skid and jumps PC;
  the IssueQueue invalidates all slots; the BusyTable clears all bits; the LSU squashes LDQ + the
  uncommitted STQ; ExecMulDiv aborts an in-flight divide; the ROB resets head/tail to 0.
- Rename recovers by reverting the speculative map to the committed map (which the ROB maintains
  as it retires) and rebuilding the free list from it.
- The predictor's global history needs **no** snapshot/restore, because it is only ever trained
  from committed branches (§3.1).

**Cost.** A mispredicted branch cannot redirect until it becomes the ROB head, so the misprediction
penalty is larger than a branch-mask machine's. The performance data (§13) shows this is an
acceptable trade for this workload: Doom's render loop mispredicts only ~2.2–2.5%. The
`brMask`/`brTag` fields exist in `MicroOp` as the hook for a future fast-recovery upgrade, but are
unused today.

---

## 5. Running C programs on the core

### 5.1 Toolchain and ABI

C is compiled with `riscv64-unknown-elf-gcc` (bare-metal) targeting **`-march=rv32im -mabi=ilp32
-mstrict-align`**. Two flags matter:
- **No compressed extension** (`rv32im`, not `rv32imc`): Fetch assumes fixed 4-byte instructions
  and increments PC by 4; a `c.` instruction would desynchronize it.
- **`-mstrict-align`:** the LSU has no misaligned-access support, so gcc must never merge into an
  unaligned wide access.

### 5.2 crt0, linker script, and the tohost protocol

There is no OS, no traps, no CSRs — so [crt0.S](../sw/crt0.S) does the bare minimum: set the
stack, zero `.bss`, call `main()`, publish the return value, spin. The **result protocol** is a
store to the tohost mailbox, which `Memory.scala` routes out of the RAM array to the testbench:

```asm
    call    main
    li      t0, 0x70000010      # tohost MMIO word
    sw      a0, 0(t0)           # publish main()'s return value
3:  j       3b                  # halt (the core cannot stop itself)
```

[link.ld](../sw/link.ld) places `.text.init` first (reset PC = 0) and fits everything in the
16 KB default RAM. The whole flow is one script, [build.sh](../sw/build.sh):

```
riscv64-unknown-elf-gcc -march=rv32im -mabi=ilp32 -mstrict-align -Os ... crt0.S prog.c
objcopy -O binary → od -An -tx4 → prog.hex   # one 32-bit word per line, line N = word N
```

The `.hex` is consumed either by the flash port (streamed one word/cycle) or by `$readmemh`
preload (§9).

### 5.3 The test programs

`sw/tests/` holds `bytes.c` (SB/SH regression), `arith.c` (RV32IM), `timer.c` (the hardware
timer), and `bench.c` (a divide-heavy prime sieve used as a benchmark). They report via tohost
and run under `ProgramSpec`.

---

## 6. ISA compliance harness

The **key correctness gate is 48/48 rv32ui + rv32um** from the official riscv-tests, run by
[IsaSpec.scala](../src/test/scala/RISCV/IsaSpec.scala). The stock riscv-tests environment boots
through machine-mode CSRs and exits via `ecall` — none of which this core implements — so the
tests are built against a **custom env** (`sw/isa/riscv_test.h`) that starts at address 0 and
exits via the tohost mailbox. The **test bodies are untouched**; only the boot/exit scaffolding is
replaced. The result word follows the riscv-tests convention:

```
1                    → pass
(TESTNUM << 1) | 1   → failed sub-test TESTNUM
no write             → the core hung
```

Two tests are intentionally skipped: `fence_i` (needs FENCE.I / self-modifying code / an icache)
and `ma_data` (deliberate misaligned accesses; the LSU has none and the core can't trap). When
the images are absent (no toolchain / fresh clone), the suite **cancels cleanly** rather than
failing.

---

## 7. Getting DOOM to run

DOOM runs by cross-compiling `doomgeneric` + a RISC-V platform port (the same one written for
Hopper) to a flat memory image, preloading it, and letting the core execute. The GPL clone and
WAD are *not* vendored; our port files live in `sw/doom-port/` and the build is
[build_doom.sh](../sw/doom-port/build_doom.sh). This is a software integration story — **the core
was never the problem.**

### 7.1 A C library where there is none: picolibc

The bare-metal toolchain ships no libc, and DOOM needs stdio/stdlib/string. The build uses
**picolibc** (`--specs=picolibc.specs --crt0=minimal`), which supplies libc, its own crt0 (sets
the stack, initializes TLS — picolibc keeps `errno` in TLS — copies `.data`, zeroes `.bss`, calls
`main`), and its own linker script `picolibc.ld` with `__flash`/`__ram` defsyms. We let picolibc
own startup rather than the port's hand-rolled `_start` (more robust, TLS especially), renaming
the port's symbol so it becomes dead code.

The port was written for **newlib** (underscore syscall stubs `_read`/`_write`/`_sbrk`/…), but
picolibc's POSIX layer calls the non-underscore names. [picolibc_glue.c](../sw/doom-port/picolibc_glue.c)
bridges the two and installs a console device that routes `printf` to the debug-char MMIO port —
which is how we watch Doom boot:

```c
ssize_t write(int fd, const void *buf, size_t n) { return _write(fd, buf, (ssize_t)n); }
void   *sbrk(intptr_t inc)                        { return _sbrk(inc); }
#define DBG_CHAR (*(volatile unsigned int *)0x70000000)
static int sim_putc(char c, FILE *f) { DBG_CHAR = (unsigned char)c; return c; }
void _exit(int code) { *(volatile unsigned int *)0x70000010 = code; for(;;){} }  // tohost + halt
```

### 7.2 Embedding the WAD without C23 `#embed`

The upstream port embeds the WAD with C23 `#embed`, which needs GCC 15; we have GCC 13. So the
WAD is embedded via assembler [.incbin](../sw/doom-port/wad.S), exposing the same `doom1_wad`
symbol plus a length word. The build patches the port's `#embed` line to `extern` declarations
with `perl`/`sed`. A **WAD selector** (`WAD=…` env var) copies the chosen WAD to a canonical
`game.wad` before assembling, so Freedoom and shareware `doom1.wad` can both live on disk and be
switched per build.

### 7.3 Memory-direct, zero-copy WAD access

The single biggest software win. The upstream stdio WAD backend reads the 21 MB WAD through
`FILE*`/`fread`, which on picolibc descends into byte-at-a-time buffered stdio — merely *loading*
the WAD cost ~600M instructions. Since the WAD is already resident (it's the `.incbin` array),
[w_file_mem.c](../sw/doom-port/w_file_mem.c) replaces the backend with a memory-direct one that
points `wad->mapped` straight at the array for **zero-copy lump access**:

```c
wad->mapped = (byte *) doom1_wad;   // W_CacheLumpNum now hands back pointers into the WAD
wad->length = doom1_wad_len;        // no read, no memcpy, no per-byte getc
```

### 7.4 Build flags and layout

`-march=rv32im -O2 -mstrict-align`: `rv32im` because we have hardware mul/div (Doom's renderer is
wall-to-wall `FixedMul`/`FixedDiv`; making them single instructions instead of libgcc routines is
a large win when every instruction is a simulated cycle); `-O2` for fewer instructions to
simulate; `-mstrict-align` for the no-misaligned LSU. Memory is split flash (code + rodata + the
WAD + the `.data` init image) / ram (runtime data, bss, heap, stack) via `picolibc.ld` defsyms;
crt0 copies `.data` from its flash LMA to its ram VMA at boot.

`DoomSpec` preloads the resulting `doom.hex` into a **16M-word (64 MB) RAM**
(`OoOParams(memWords = 16*1024*1024)`) and runs it.

---

## 8. Issues encountered (and how they were fixed)

### 8.1 CPU bugs (all fixed and regression-tested)

1. **SB/SH clobbered neighbouring bytes.** The LSU originally spliced sub-word stores over a
   *zeroed* base word, destroying the other three bytes. **Fix:** per-byte write enables — memory
   stores `Vec(4, UInt(8.W))`, the LSU drives a 4-bit mask (`storeMask`/`storeData`/`loadMask`),
   and untouched lanes are preserved by the SRAM. SB/SH/SW are now all exact with no
   read-modify-write.

2. **Stores never reported completion to the ROB → deadlock on the first store.** A store
   produces no register value, so the LSU didn't drive its writeback port — the ROB entry never
   became `done`, the head never retired, and the machine hung. **Fix:** the LSU drives its
   writeback port for stores too (`writesReg=false`, so the PRF/IQ/BusyTable ignore it; only the
   ROB acts on it). The single wb port is arbitrated stage-2-load > forwarding-load > store-completion.

3. **Missed-wakeup race in the BusyTable.** A uop renamed in the same cycle its producer wrote
   back would wait forever (registered busy bit clears at T+1, but the IQ wakeup skipped a
   not-yet-valid slot). This is exactly what hung the first compiled C program (an `sw` whose base
   register was produced by an `addi` writing back that cycle). **Fix:** source ready =
   `!busy OR clearedThisCycle` (§3.4).

4. **Branch predictor was tied off.** The gshare/BTB/RAS `update` port was never wired, so the
   tables stayed frozen at reset — every branch predicted from a cold table, costing ~1 flush per
   3 retired instructions on branch-heavy code. **Fix:** wire `ROB.brUpdate` (new bundle;
   `RobEntry` records `brTaken`/`brTarget` for *every* resolved branch) to Fetch's predictor
   update, trained at commit (§3.1). Mispredicts on `bench.c` dropped 52%.

### 8.2 Build/environment issues

- **`sbt test` fails on native Windows.** ChiselSim shells out to Verilator (needs a Unix
  toolchain), and firtool on Windows hits a code-page error creating the `verification\Assert`
  layer dirs. **Everything runs under WSL Ubuntu instead.** (`sbt compile` works natively.)
- **Flaky Verilator build crash from `/mnt/c`.** Building on the DrvFs mount produces clock-skew
  warnings ("modification time in the future") and an intermittent
  `Internal Error: attempted to destroy locked Thread Pool` — a *build-time* crash, not a logic
  failure. Re-running the affected test passes. A real logic failure shows `was not equal`
  (ScalaTest) or `FAILED at sub-test #N` (ISA); the two are distinguishable in the log.

### 8.3 DOOM-specific issues

- **`DG_SleepMs` infinite loop at `-O2`.** The port read the timer through a non-`volatile`
  pointer; `-O2` hoisted the loop-invariant read out of the wait loop, making `while(1)`. **Fix:**
  a `sed` in the build makes the timer (`0x8000004`) and framebuffer (`0x10000000`) accesses
  `volatile`.
- **WAD load took ~600M instructions** through buffered stdio → the memory-direct backend (§7.3).
- **Missing music was fatal.** `S_ChangeMusic` used `W_GetNumForName` (which `I_Error`s if the
  lump is absent); patched to `W_CheckNumForName` + a guard. Also switched from FreeDM (a minimal
  deathmatch WAD lacking single-player assets) to a complete IWAD.
- **A misdiagnosis worth recording:** a suspected "divider hang" turned out to be Verilator stdout
  buffering, not the divider — proven by showing K=1 and K=4 produced identical behavior. The
  faster divider was kept.

---

## 9. Optimizations

### 9.1 Simulation throughput (~700×) — the enabling optimization

DOOM is ~10⁸ instructions to the first frame; at 670 cycles/sec that looked like ~13 days. The
fix was to stop measuring the JVM↔Verilator round-trip:

- **All counting moved into hardware.** [SocHarness.scala](../src/test/scala/RISCV/SocHarness.scala)
  adds hardware performance counters (`perf_cycles/commits/redirects/divbusy/headstall/…`) and a
  **latched tohost** (so the single-cycle exit pulse can't be missed between polls). The testbench
  then steps the clock in **large batches** (`dut.clock.step(20000)`) and reads totals occasionally
  instead of peeking every cycle. Result: ~400–485k cycles/sec.
- **`OptimizeForSimulationSpeed`** (adds `-O3 --x-assign fast` to Verilator). ChiselSim exposes no
  Verilator `--threads` option, so this is the optimization ceiling.

### 9.2 `$readmemh` preload instead of flashing

The flash port accepts one word per clock, and driving it from the testbench costs a round-trip
per word. A 24 MB Doom image is ~6M words → *hours* before the CPU executes a single instruction.
`OoOParams.memInitFile` → `loadMemoryFromFileInline` (`$readmemh`) loads the whole image at time
zero. **Caveat:** ChiselSim's default `RANDOMIZE_MEM_INIT` clobbers the `$readmemh` init, so the
preloaded specs pass `Randomization.uninitialized` (helper `noRandom`).

### 9.3 The faster divider

`divBitsPerCycle = 4` (8-cycle divide vs the original 32). On `bench.c` (a prime sieve that spent
71% of cycles waiting on the divider), IPC went 0.167 → 0.368. RV32M ISA still 8/8. Radix is a
tunable Fmax↔latency trade (§3.9).

### 9.4 The branch predictor

Wiring commit-time training (§3.1, §8.1-4) cut mispredicts −52% on `bench.c`. Training only at
commit is *itself* the optimization that keeps the full-flush model cheap (no history recovery).

### 9.5 The `-march=rv32im` / `-O2` / zero-copy-WAD stack for Doom

Covered in §7.3–§7.4: hardware mul/div for the renderer, `-O2` to shrink the instruction stream,
and the zero-copy WAD backend that removed ~600M instructions of stdio.

### 9.6 The `clockHz` trick for capturing animation

DOOM paces itself off the hardware timer. At the real 125 MHz, the title screen holds for ~11 s
of game-time (~1.4 *billion* cycles) and each frame waits ~1/35 s (~3.5M cycles) — so a capture
just gets static title frames. **Lowering `clockHz` (to 1 MHz)** makes game-time pass faster per
cycle, so Doom stops waiting between frames (becomes render-bound) and reaches renderable content
in tens of millions of cycles. This changes *nothing* the core computes — only how the timing
loop perceives elapsed time.

---

## 10. Capturing the animation

Reaching gameplay and turning it into a video used three mechanisms.

### 10.1 Framebuffer capture RAM (read without advancing the CPU)

`SocHarness` mirrors every framebuffer write into a **combinational-read `Mem`** (not a
`SyncReadMem`). The testbench scans it out one pixel at a time with poke-address + peek and **no
clock step**, so the readout does not advance the CPU — a frame can be captured without pausing
Doom and without it overwriting the frame mid-scan:

```scala
val fbMem = Mem(p.fbWords, UInt(32.W))          // combinational read on purpose
when(memory.io.write_vga && (memory.io.address_vga < p.fbWords.U)) {
    fbMem.write(memory.io.address_vga, memory.io.write_value_vga)
    fbWrites := fbWrites + 1.U
}
io.fb_read_data := fbMem.read(io.fb_read_addr)  // no step → does not advance the CPU
```

`DoomSpec.dumpFramePPM` scans this into a binary PPM. Multi-frame capture dumps a numbered PPM as
each frame completes, using a fine step so frames aren't overshot.

### 10.2 Scripted keyboard input → live gameplay

DOOM's attract *demo* is version-locked and won't play back on this WAD/port, and auto-advancing
the title takes hundreds of frames. So instead the testbench **drives the menu itself** by poking
the keyboard MMIO bitmap. `SocHarness` widened the keyboard port:

```scala
val key_bits = Input(UInt(32.W))
memory.io.keys := io.btns | io.key_bits
```

`DoomSpec` schedules presses by frame number (Doom advances one game-tic per rendered frame). Bit
8 maps to HID 0x28 = Enter, bit 26 to W = forward:

```scala
def scheduledKeys(frame: Int): BigInt =
    if      (frame >= 3  && frame < 6)  ENTER    // title → main menu
    else if (frame >= 10 && frame < 13) ENTER    // New Game
    else if (frame >= 17 && frame < 20) ENTER    // Episode 1 (Knee-Deep in the Dead)
    else if (frame >= 24 && frame < 27) ENTER    // Skill → drop into E1M1
    else if (frame >= moveFrom)         FORWARD  // in game: walk forward
    else                                BigInt(0)
```

Each press is held a few frames (so `DG_GetKey`, sampled once per frame, sees a clean keydown)
then released so the next press edges. This was **verified frame-by-frame** from the captured
images: title (frame 0) → menu (5) → episode select (12) → skill select (22) → **E1M1
first-person view with the full HUD (frame 33)** → walking.

### 10.3 PPM → PNG → MP4, entirely dependency-free where needed

The 58 captured PPMs were converted to PNG and stitched two ways: a **self-contained HTML player**
(all frames base64-embedded, offline, with a scrubber and phase captions) and an **MP4** (H.264,
640×480 nearest-neighbor upscale, ~7 s). Both live in `doom-gameplay/`. Because the WSL image had
no `ffmpeg` and no sudo, ffmpeg was obtained via the `imageio-ffmpeg` pip package (a static binary
into `~/.local`, no system changes).

---

## 11. Verification & test infrastructure

| Suite | What it covers |
|---|---|
| `ExecuteSpec` | PhysRegFile / ExecAlu / ExecMulDiv, incl. a dense DIVU/REMU sweep |
| `FrontendSpec` | Decode / Rename / Frontend |
| `MemCommitSpec` | ROB / LSU, incl. the SB/SH regression |
| `MainSpec` | whole-SoC smoke test |
| `ProgramSpec` | compiled-C programs (`sw/tests/*.c`) |
| `IsaSpec` | **48 riscv-tests (rv32ui + rv32um) — the key correctness gate** |
| `DoomSpec` | runs DOOM, captures frames |

The RTL/unit suites all pass. `ProgramSpec`/`IsaSpec`/`DoomSpec` **cancel cleanly** (not fail)
when their hex images are absent (no toolchain / fresh clone / no Doom clone).

[SocHarness.scala](../src/test/scala/RISCV/SocHarness.scala) is the shared Core+Memory harness:
tohost mailbox (latched), the hardware perf counters, the commit trace, the framebuffer capture
RAM, and a resettable committed-PC range (`pc_min/max/last`) for localizing a hang loop.

**How to run** (must be WSL, not native Windows):

```bash
wsl -d Ubuntu -- bash -lc 'cd /mnt/c/.../OoO-RISC-V && sbt test'
wsl -d Ubuntu -- bash -lc 'cd /mnt/c/.../OoO-RISC-V && sbt "testOnly RISCV.ExecuteSpec"'
# DOOM (long): sbt -Ddoom.clockHz=1000000 -Ddoom.cycles=500000000 \
#   -Ddoom.frames=60 -Ddoom.frameStride=1 -Ddoom.moveFromFrame=30 "testOnly RISCV.DoomSpec"
```

CI (`.github/workflows/test.yml`) rebuilds the C hexes (fails if committed `sw/build/*.hex` are
stale) and runs `sbt test`; a nightly/dispatch job runs the full ISA suite.

---

## 12. Known limitations & gaps

These are deliberate first-pass simplifications, all documented in-source:

- **No branch-mask fast recovery.** Recovery is the commit-time full flush (§4). The data says
  fast recovery is low value for this workload (§13) — don't build it without fresh evidence.
- **One load in flight** (no MSHRs). Sim memory is 1-cycle, so there is no load latency to hide;
  MSHRs only pay off against real DDR latency.
- **RAS not consulted by the predict port** — needs pre-decode at fetch to identify calls/returns.
- **No CSRs, no `ecall`, no traps, no FENCE.I, no misaligned accesses.** `-mstrict-align` avoids
  the last; the ISA suite skips `fence_i` and `ma_data` accordingly.
- **`CommitSignal.isMMIO` is left false in the ROB** (the ROB tracks no address); the LSU
  determines MMIO from the store's resolved address instead.
- **DOOM open issues:** the attract demo doesn't auto-play (version-locked demo lumps); a
  framebuffer bottom-bar artifact from downsampling 640×400→320×240 (rows 200–239 read past the
  screen buffer — the fix is to target 320×200); and real-time interactive play needs the FPGA
  path (sim is ~450k cycles/sec vs millions of instructions per frame).

---

## 13. Performance data

Measured with the stall-attribution counters. **The core is healthy:**

| Workload | IPC | Notes |
|---|---|---|
| `arith.c` (ALU-bound) | **0.99** | near-ideal for 1-wide |
| `bench.c` (prime sieve) | 0.167 → **0.368** | was 71% divider-bound; the faster divider was the right win |
| DOOM init | **0.85** | — |
| DOOM render loop | **0.79** | only ~2.2–2.5% mispredict, ~0% divider, ~17% head-stall |

Implications: **branch-mask fast recovery is low-value** here (Doom render mispredicts ~2.2%,
arith ~0), and **MSHRs are FPGA-era only** (no sim load latency to hide). The two big wins that
*did* land were the faster divider (divide-bound code) and wiring the predictor (branch-heavy
code).

---

## 14. What's next

The simulation path has been taken about as far as it's worth: the core is proven correct (48/48
ISA) and proven on a real workload (DOOM into gameplay). The natural next chapter is **FPGA**, for
*real-time, playable* DOOM — which simulation fundamentally cannot deliver (~450k cycles/sec).

Suggested staging (hardest part last):
1. **BRAM bring-up** — synthesize as-is, close timing, run a tiny program from on-chip BRAM; get a
   real Fmax and flush out synthesis-vs-sim surprises.
2. **Memory hierarchy** — the dominant new work: I/D cache + an AXI bridge to external DDR3
   (LiteX/LiteDRAM avoids hand-integrating the Xilinx MIG). Model DDR latency in sim first.
3. **DOOM on hardware + HDMI out.**

Board shortlist: Nexys Video (XC7A200T, 512 MB DDR3, HDMI) or Arty A7-100T (256 MB DDR3); a Zynq
board gives DDR "for free" via the PS. Cheap wins to grab along the way: fix the framebuffer
320×200 downsample, and move builds off `/mnt/c` to native ext4 to kill the flaky Verilator
crashes. A larger, orthogonal core project would be going **2-wide superscalar** (the real IPC
lever now that the divider and predictor are fixed), but it doesn't get you closer to playable
DOOM — the FPGA path does.

---

*This document reflects the source at the time of writing; when in doubt, the code in
`src/main/scala/RISCV/` is authoritative. See `CLAUDE.md` for the working-notes version and
`ATTRIBUTION.md` for the Hopper relationship.*
