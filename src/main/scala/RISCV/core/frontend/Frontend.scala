package RISCV

import chisel3._
import chisel3.util._

/**
 * In-order front end of the OoO core: Fetch -> Decode -> Rename -> Dispatch.
 *
 * Exposes ONLY the Core-facing IO fixed by the orchestrator. Internally it instantiates and
 * wires the four pipeline stages plus the rename structures (the latter live inside Rename).
 *
 * The orchestrator's Core wires the dispatch interface as:
 *     rob.allocReq    := dispatchValid
 *     rob.allocUop    := dispatchUop
 *     iq.enq.valid    := dispatchValid
 *     iq.enq.bits     := dispatchUop ; iqReady := iq.enq.ready
 *     lsu.ldqAllocReq := dispatchValid && dispatchUop.isLoad
 *     lsu.stqAllocReq := dispatchValid && dispatchUop.isStore
 * so `dispatchValid` already factors in robFull, iqReady, ldqFull(if load), stqFull(if store),
 * and free-list availability (the latter via Rename's backpressure into Dispatch).
 */
class Frontend(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val enable = Input(Bool())

        // instruction memory (SyncReadMem: data returns one cycle after the address)
        val imemAddr = Output(UInt(p.xlen.W))
        val imemData = Input(UInt(p.xlen.W))

        // dispatch interface to the back end
        val dispatchUop = Output(new MicroOp(p)) // fully stamped
        val dispatchValid = Output(Bool()) // asserted only when ALL needed resources are free

        // back-end resource status / allocation indices
        val robFull = Input(Bool())
        val robAllocIdx = Input(UInt(p.robIdxWidth.W))
        val iqReady = Input(Bool())
        val ldqFull = Input(Bool())
        val ldqAllocIdx = Input(UInt(p.ldqIdxWidth.W))
        val stqFull = Input(Bool())
        val stqAllocIdx = Input(UInt(p.stqIdxWidth.W))

        // back-end feedback
        val wb = Input(Vec(p.numWbPorts, new WbPort(p)))
        val commit = Input(new CommitSignal(p))
        val redirect = Input(new Redirect(p))
        val brUpdate = Input(new BrUpdate(p)) // predictor training, from the ROB at retirement
    })

    val fetch = Module(new Fetch(p))
    val decode = Module(new Decode(p))
    val rename = Module(new Rename(p))
    val dispatch = Module(new Dispatch(p))

    // ---- Fetch ----
    fetch.io.enable := io.enable
    fetch.io.redirect := io.redirect
    fetch.io.brUpdate := io.brUpdate
    io.imemAddr := fetch.io.imemAddr
    fetch.io.imemData := io.imemData

    // ---- Fetch -> Decode ----
    decode.io.in <> fetch.io.out

    // ---- Decode -> Rename ----
    rename.io.in <> decode.io.out
    rename.io.wb := io.wb
    rename.io.commit := io.commit
    rename.io.redirect := io.redirect

    // ---- Rename -> Dispatch ----
    dispatch.io.in <> rename.io.out
    dispatch.io.robFull := io.robFull
    dispatch.io.robAllocIdx := io.robAllocIdx
    dispatch.io.iqReady := io.iqReady
    dispatch.io.ldqFull := io.ldqFull
    dispatch.io.ldqAllocIdx := io.ldqAllocIdx
    dispatch.io.stqFull := io.stqFull
    dispatch.io.stqAllocIdx := io.stqAllocIdx
    dispatch.io.redirect := io.redirect

    // ---- Dispatch -> Core ----
    io.dispatchUop := dispatch.io.dispatchUop
    io.dispatchValid := dispatch.io.dispatchValid
}