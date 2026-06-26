# OoO-RISC-V — A minimal out-of-order RV32IM core in Chisel

A from-scratch **out-of-order (OoO)** 32-bit RISC-V core, written in **Chisel**, simulated
and tested with **ChiselSim + ScalaTest**. The microarchitecture follows the **BOOM**
(Berkeley Out-of-Order Machine) family — *explicit register renaming, a unified physical
register file, read-after-issue* — but is deliberately stripped down to fit a small FPGA and
the **RV32I + M** ISA. It is the sibling/successor of the in-order pipelined `RISC-V` project;
the design rationale lives in that repo's `OUT-OF-ORDER-DESIGN.md`.

> **Status:** first-pass implementation. The core elaborates and is exercised by ChiselSim
> tests; it is *not yet* a verified, timing-closed design. See the milestone plan below.

## The one rule

> **Compute out of order; commit in order.** Architectural state — the register file *and*
> memory (including the VGA framebuffer and button MMIO) — only changes in program order, at
> the moment an instruction *retires* from the reorder buffer.

## Microarchitecture (1-wide to start)

```
  IN-ORDER FRONT END                 OUT OF ORDER                 IN-ORDER COMMIT
  Fetch → Decode → Rename → Dispatch  IssueQueue → PRF read →      ReorderBuffer
  (+ branch predict)                  ExecUnits (ALU/MULDIV/AGU)   • free old preg
                                      → writeback (CDB) → wakeup   • commit stores
                                      LSU (LDQ/STQ)                • MMIO at head
                                                                   • redirect/flush
```

Key design choices (vs. a full BOOM): RV32IM only (no FP); gshare + small BTB + RAS (no TAGE);
one unified integer+memory issue queue with an age-matrix scheduler; deterministic, replay-free
load latency (no cache); commit-time stores with MMIO executed at the ROB head; 2–4 in-flight
branch snapshots. All sizes are parameterized in [`Params.scala`](src/main/scala/RISCV/Params.scala).

## Layout

```
build.sbt / project/        Chisel 7.0.0, Scala 2.13.16, ScalaTest 3.2.19
.github/workflows/test.yml  CI: Verilator + `sbt test`
convert.py                  .bin -> .hex (little-endian 32-bit words)
load_program.py             Stream a .hex to a board over UART (for a future FPGA bring-up)

src/main/scala/RISCV/
  Params.scala              OoOParams case class: all structural sizes/widths
  Bundles.scala             Frozen shared interfaces: MicroOp, DecodedInst, CDB, ROB, redirect
  Memory.scala              Word-addressed RAM + VGA region + button MMIO
  Main.scala                SoC top: Core + Memory (+ peripherals)
  core/
    Core.scala              Stitches the whole OoO datapath together
    Decoder.scala           Combinational field + immediate extractor (RV32I)
    frontend/               Fetch, BranchPredictor, Decode, Rename (MapTable/FreeList/BusyTable), Dispatch
    execute/                IssueQueue, PhysRegFile, ExecAlu, ExecMulDiv
    memory/                 Lsu (LDQ/STQ + store-to-load forwarding + MMIO-at-head)
    commit/                 ReorderBuffer (allocate / retire / flush)
  peripheral/               VGAController, UART (optional SoC peripherals)

src/test/scala/RISCV/       ChiselSim + ScalaTest specs (per-module + whole-core)
```

## Build & test

```bash
sbt compile      # elaborate
sbt test         # run the ChiselSim test suite
sbt "runMain RISCV.Main"   # emit SystemVerilog into generated/
```

## Milestone plan

1. Self-checking test harness (golden RV32IM reference, tandem-checked at retire).
2. Rename + PRF + ROB, single FU, in-order issue (validates the plumbing).
3. Issue queue + wakeup → true out-of-order issue.
4. LSU (LDQ/STQ) with conservative ordering + store-to-load forwarding + MMIO-at-head.
5. Branch prediction + branch-mask fast recovery.
6. Optimize: speculative ALU wakeup, DSP multiplier, wider front end.