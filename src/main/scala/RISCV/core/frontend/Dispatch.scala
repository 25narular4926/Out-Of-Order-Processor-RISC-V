package RISCV

import chisel3._
import chisel3.util._

/**
 * Dispatch: the last in-order stage. It stamps the back-end allocation indices (robIdx, ldqIdx,
 * stqIdx) into the renamed MicroOp and fires the atomic allocation into the ROB + IssueQueue +
 * (for memory ops) the LDQ/STQ.
 *
 * "Atomic" means all-or-nothing: a uop dispatches this cycle ONLY when every resource it needs
 * has room. If any needed resource is full, we hold the instruction (deassert in.ready toward
 * Rename) and dispatch nothing, so no resource is half-allocated.
 *
 * Resource requirements:
 *   - ROB:  always (every uop occupies a reorder-buffer entry)        -> needs !robFull
 *   - IQ:   always (every uop issues through the unified issue queue) -> needs iqReady
 *   - LDQ:  only loads                                                -> needs !ldqFull
 *   - STQ:  only stores                                               -> needs !stqFull
 *
 * The allocation indices are supplied combinationally by the back end (the ROB/LSU present the
 * index they WOULD allocate); Dispatch stamps them into the uop and asserts `fire` so the back
 * end commits the allocation. This matches the orchestrator's wiring (see Frontend.scala).
 */
class Dispatch(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        // in-order uop stream from Rename
        val in = Flipped(Decoupled(new MicroOp(p)))

        // ---- back-end resource status / allocation indices ----
        val robFull = Input(Bool())
        val robAllocIdx = Input(UInt(p.robIdxWidth.W))
        val iqReady = Input(Bool())
        val ldqFull = Input(Bool())
        val ldqAllocIdx = Input(UInt(p.ldqIdxWidth.W))
        val stqFull = Input(Bool())
        val stqAllocIdx = Input(UInt(p.stqIdxWidth.W))

        // flush
        val redirect = Input(new Redirect(p))

        // ---- dispatch outputs (consumed by the orchestrator's Core) ----
        val dispatchUop = Output(new MicroOp(p)) // fully stamped
        val dispatchValid = Output(Bool()) // asserted only when ALL needed resources are free
    })

    val uop = io.in.bits

    // Which resources does THIS uop need?
    val needLdq = uop.isLoad
    val needStq = uop.isStore

    // All needed resources have room?
    val robOk = !io.robFull
    val iqOk = io.iqReady
    val ldqOk = !needLdq || !io.ldqFull
    val stqOk = !needStq || !io.stqFull
    val resourcesOk = robOk && iqOk && ldqOk && stqOk

    // Dispatch fires when an instruction is present, all needed resources have room, and we are
    // not flushing.
    val fire = io.in.valid && resourcesOk && !io.redirect.valid

    // Backpressure to Rename: accept the instruction only when it actually dispatches.
    io.in.ready := resourcesOk && !io.redirect.valid

    // ---- stamp allocation indices combinationally ----
    val stamped = WireInit(uop)
    stamped.robIdx := io.robAllocIdx
    stamped.ldqIdx := io.ldqAllocIdx
    stamped.stqIdx := io.stqAllocIdx

    io.dispatchUop := stamped
    io.dispatchValid := fire
}
