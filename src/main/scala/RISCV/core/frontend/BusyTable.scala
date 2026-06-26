package RISCV

import chisel3._
import chisel3.util._

/**
 * Busy table: one bit per physical register. busy == 1 means "a producer has been renamed for
 * this tag but its value has not yet been written back", i.e. the value is not yet available.
 *
 *   - SET   at rename when a new destination tag is allocated (the producer is now in flight).
 *   - CLEAR when a writeback port reports that tag (wb.valid && wb.writesReg && wb.pdst == tag),
 *           snooping the whole `numWbPorts` Vec.
 *   - READ  with prs1/prs2 to produce prs1Ready/prs2Ready (ready == !busy).
 *   - On redirect: clear ALL busy bits (no speculative producer survives a full flush; the only
 *     surviving values are committed ones, which are by definition already written back).
 *
 * zeroPreg is always ready (never busy).
 */
class BusyTable(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        // ---- read ports (combinational, at rename) ----
        val rs1 = Input(UInt(p.pregWidth.W))
        val rs2 = Input(UInt(p.pregWidth.W))
        val rs1Ready = Output(Bool())
        val rs2Ready = Output(Bool())

        // ---- set at rename ----
        val setValid = Input(Bool())
        val setTag = Input(UInt(p.pregWidth.W))

        // ---- clear from writeback (snoop all CDB ports) ----
        val wb = Input(Vec(p.numWbPorts, new WbPort(p)))

        // ---- flush ----
        val redirect = Input(Bool())
    })

    // busy(i) == true => physical register i has a pending (not-yet-written-back) producer.
    val busy = RegInit(VecInit(Seq.fill(p.numPhysRegs)(false.B)))

    // ---- reads: a source is ready when it is not busy. zeroPreg is always ready. ----
    io.rs1Ready := (io.rs1 === p.zeroPreg.U) || !busy(io.rs1)
    io.rs2Ready := (io.rs2 === p.zeroPreg.U) || !busy(io.rs2)

    when(io.redirect) {
        // Full flush: nothing speculative survives, so no tag is pending. Clear everything.
        for (i <- 0 until p.numPhysRegs) {
            busy(i) := false.B
        }
    }.otherwise {
        // ---- clear bits for every tag written back this cycle ----
        for (port <- io.wb) {
            when(port.valid && port.writesReg && port.pdst =/= p.zeroPreg.U) {
                busy(port.pdst) := false.B
            }
        }

        // ---- set the newly-allocated destination tag (takes priority over a same-cycle clear
        //      of the same tag: a just-renamed producer is definitionally not yet done) ----
        when(io.setValid && io.setTag =/= p.zeroPreg.U) {
            busy(io.setTag) := true.B
        }
    }

    // zeroPreg can never be busy.
    busy(p.zeroPreg) := false.B
}