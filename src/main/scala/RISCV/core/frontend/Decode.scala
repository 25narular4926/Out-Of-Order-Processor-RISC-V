package RISCV

import chisel3._
import chisel3.util._

/**
 * Decode stage. Wraps the combinational `Decoder` (field/immediate extractor) and turns a
 * `FetchPacket` into a partially-filled `MicroOp`.
 *
 * Per the MicroOp contract (Bundles.scala), Decode fills:
 *   pc, opcode, func3, func7, immediate, fuType, lrs1/lrs2/ldst,
 *   readsRs1/readsRs2/writesReg, isBranch/isJump/isJalr/isLoad/isStore, predTaken, predTarget
 * and drives every field a later stage owns (prs*, pdst*, idx*, ready bits, brMask/brTag) to 0.
 *
 * Decode is purely combinational over the incoming packet, so ready/valid pass straight through
 * (out.valid = in.valid, in.ready = out.ready). No state is held, so there is nothing to flush
 * on redirect; the producing/consuming Decoupleds carry validity and Fetch squashes upstream.
 */
class Decode(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val in = Flipped(Decoupled(new FetchPacket(p)))
        val out = Decoupled(new MicroOp(p))
    })

    // ---- RV32I/M opcodes (7-bit) ----
    val OP_LUI = "b0110111".U
    val OP_AUIPC = "b0010111".U
    val OP_OPIMM = "b0010011".U // addi, slti, ... (I-type ALU)
    val OP_OP = "b0110011".U // add, sub, ... and M-extension (R-type)
    val OP_LOAD = "b0000011".U
    val OP_STORE = "b0100011".U
    val OP_BRANCH = "b1100011".U
    val OP_JAL = "b1101111".U
    val OP_JALR = "b1100111".U

    // =====================================================================================
    // Field extraction via the shared combinational decoder.
    // =====================================================================================
    val dec = Module(new Decoder())
    dec.io.instruction := io.in.bits.instruction

    val opcode = dec.io.opcode
    val func3 = dec.io.func3
    val func7 = dec.io.func7
    val rs1 = dec.io.rs1
    val rs2 = dec.io.rs2
    val rd = dec.io.rd

    // =====================================================================================
    // Instruction classification.
    // =====================================================================================
    val isLoad = opcode === OP_LOAD
    val isStore = opcode === OP_STORE
    val isBranch = opcode === OP_BRANCH
    val isJump = opcode === OP_JAL
    val isJalr = opcode === OP_JALR

    // M-extension: RV32M is encoded as the OP opcode with func7 == 0000001
    // (mul/mulh/mulhsu/mulhu/div/divu/rem/remu).
    val isMulDiv = (opcode === OP_OP) && (func7 === "b0000001".U)

    // Functional-unit selection.
    val fuType = Wire(FuType())
    when(isLoad || isStore) {
        fuType := FuType.MEM
    }.elsewhen(isMulDiv) {
        fuType := FuType.MULDIV
    }.otherwise {
        fuType := FuType.ALU
    }

    // =====================================================================================
    // Register-read / register-write classification (drives rename & busy lookups).
    //
    //   readsRs1: every R/I/S/B opcode reads rs1. LUI/AUIPC/JAL do NOT (U/J formats have no
    //             rs1 field that is architecturally read).
    //   readsRs2: R-type, S-type (store data), B-type (compare) read rs2.
    //   writesReg: R/I/U/J/JALR/LOAD write rd; STORE and BRANCH do not. Also false when rd==x0.
    // =====================================================================================
    val readsRs1 = isLoad || isStore || isBranch || isJalr ||
        (opcode === OP_OP) || (opcode === OP_OPIMM)
    val readsRs2 = isStore || isBranch || (opcode === OP_OP)

    val writesRdOpcode = (opcode === OP_OP) || (opcode === OP_OPIMM) ||
        (opcode === OP_LUI) || (opcode === OP_AUIPC) ||
        isLoad || isJump || isJalr
    val writesReg = writesRdOpcode && (rd =/= 0.U)

    // =====================================================================================
    // Assemble the MicroOp. Start from a zeroed template so every back-end-owned field is 0.
    // =====================================================================================
    val uop = WireInit(MicroOp.nop(p))

    uop.pc := io.in.bits.pc
    uop.opcode := opcode
    uop.func3 := func3
    uop.func7 := func7
    uop.immediate := dec.io.immediate
    uop.fuType := fuType

    uop.lrs1 := rs1
    uop.lrs2 := rs2
    uop.ldst := rd

    uop.readsRs1 := readsRs1
    uop.readsRs2 := readsRs2
    uop.writesReg := writesReg
    uop.isBranch := isBranch
    uop.isJump := isJump
    uop.isJalr := isJalr
    uop.isLoad := isLoad
    uop.isStore := isStore

    uop.predTaken := io.in.bits.predTaken
    uop.predTarget := io.in.bits.predTarget

    // prs1/prs2/pdst/pdstOld/prs1Ready/prs2Ready, robIdx/ldqIdx/stqIdx, brMask/brTag remain 0
    // (from MicroOp.nop) -- filled by Rename / Dispatch downstream.

    io.out.bits := uop
    io.out.valid := io.in.valid
    io.in.ready := io.out.ready
}