package RISCV

import chisel3._
import chisel3.util._

object InstructionFormat extends ChiselEnum {
    val R, I, S, B, U, J = Value
}

/**
 * Pure combinational instruction-field + immediate extractor (RV32I).
 * Splits a 32-bit instruction into rs1/rs2/rd/opcode/func3/func7, picks the format from the
 * opcode, and assembles the correctly sign-extended 32-bit immediate for each format.
 * (Ported unchanged from the in-order RISC-V project.)
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

    io.rs1 := io.instruction(19, 15)
    io.rs2 := io.instruction(24, 20)
    io.rd := io.instruction(11, 7)

    val format = Wire(InstructionFormat())
    format := InstructionFormat.R

    io.opcode := io.instruction(6, 0)
    io.func3 := io.instruction(14, 12)
    io.func7 := io.instruction(31, 25)

    switch(io.opcode) {
        is(0b0110111.U) { format := InstructionFormat.U } // lui
        is(0b0010111.U) { format := InstructionFormat.U } // auipc
        is(0b0010011.U) { format := InstructionFormat.I } // addi, slti, ... (OP-IMM)
        is(0b0110011.U) { format := InstructionFormat.R } // add, sub, ... (OP)
        is(0b0001111.U) { format := InstructionFormat.I } // fence
        is(0b1110011.U) { format := InstructionFormat.S } // system (ecall/ebreak...) -> S-ish
        is(0b0000011.U) { format := InstructionFormat.I } // loads
        is(0b0100011.U) { format := InstructionFormat.S } // stores
        is(0b1101111.U) { format := InstructionFormat.J } // jal
        is(0b1100111.U) { format := InstructionFormat.I } // jalr
        is(0b1100011.U) { format := InstructionFormat.B } // branches
    }

    io.immediate := 0.U

    switch(format) {
        is(InstructionFormat.I) {
            io.immediate := Fill(21, io.instruction(31, 31)) ## io.instruction(30, 20)
        }
        is(InstructionFormat.S) {
            io.immediate := Fill(21, io.instruction(31, 31)) ## io.instruction(31, 25) ## io.instruction(11, 7)
        }
        is(InstructionFormat.B) {
            io.immediate := Fill(20, io.instruction(31, 31)) ## io.instruction(7, 7) ## io.instruction(31, 25) ##
                io.instruction(11, 8) ## 0.U(1.W)
        }
        is(InstructionFormat.U) {
            io.immediate := io.instruction(31, 12) ## 0.U(12.W)
        }
        is(InstructionFormat.J) {
            io.immediate := Fill(12, io.instruction(31, 31)) ## io.instruction(19, 12) ## io.instruction(20, 20) ##
                io.instruction(30, 21) ## 0.U(1.W)
        }
    }
}