package RISCV

import chisel3._
import chisel3.util._

/**
 * Pure combinational instruction-field + immediate extractor (RV32I).
 * Splits a 32-bit instruction into rs1/rs2/rd/opcode/func3/func7 and assembles the correctly
 * sign-extended 32-bit immediate for the instruction's format.
 *
 * NOTE on the immediate selection: we select the immediate with a direct nested `Mux` keyed on
 * the opcode (every arm is a 32-bit value) rather than an intermediate format-enum `switch`.
 * The enum-`switch` form makes firtool emit a packed-array select (`_GEN[formatCode]`), which
 * trips Verilator's WIDTHEXPAND lint (fatal by default) on the 3-bit index. The Mux form lowers
 * to plain nested ternaries of equal-width operands and is lint-clean. Decode semantics are
 * identical to the classic R/I/S/B/U/J split.
 */
class Decoder() extends Module {
    val io = IO(new Bundle {
        val instruction = Input(UInt(32.W))
        val rs1 = Output(UInt(5.W))
        val rs2 = Output(UInt(5.W))
        val rd = Output(UInt(5.W))
        val immediate = Output(UInt(32.W))
        val opcode = Output(UInt(7.W))
        val func3 = Output(UInt(3.W))
        val func7 = Output(UInt(7.W))
    })

    val instr = io.instruction

    io.rs1 := instr(19, 15)
    io.rs2 := instr(24, 20)
    io.rd := instr(11, 7)
    io.opcode := instr(6, 0)
    io.func3 := instr(14, 12)
    io.func7 := instr(31, 25)

    // The five RISC-V immediate encodings, each EXACTLY 32 bits (sign-extended). Keeping every
    // arm at 32 bits is what keeps the Mux below lint-clean (no WIDTHTRUNC/WIDTHEXPAND).
    val iImm = Fill(20, instr(31)) ## instr(31, 20) // 20 + 12 = 32
    val sImm = Fill(20, instr(31)) ## instr(31, 25) ## instr(11, 7) // 20 + 7 + 5 = 32
    val bImm = Fill(20, instr(31)) ## instr(7) ## instr(30, 25) ## instr(11, 8) ## 0.U(1.W) // 20+1+6+4+1 = 32
    val uImm = instr(31, 12) ## 0.U(12.W) // 20 + 12 = 32
    val jImm = Fill(12, instr(31)) ## instr(19, 12) ## instr(20) ## instr(30, 21) ## 0.U(1.W) // 12+8+1+10+1 = 32

    val opcode = io.opcode

    // Format classification by opcode (R-type and anything unmatched carry no immediate => 0).
    val isU = opcode === "b0110111".U || opcode === "b0010111".U // lui, auipc
    val isJ = opcode === "b1101111".U // jal
    val isB = opcode === "b1100011".U // branches
    val isS = opcode === "b0100011".U || opcode === "b1110011".U // store, system (S-shaped)
    val isI = opcode === "b0010011".U || // op-imm
        opcode === "b0000011".U || // loads
        opcode === "b1100111".U || // jalr
        opcode === "b0001111".U // fence

    io.immediate := Mux(
      isU,
      uImm,
      Mux(isJ, jImm, Mux(isB, bImm, Mux(isS, sImm, Mux(isI, iImm, 0.U(32.W)))))
    )
}