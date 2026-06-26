package RISCV

import chisel3._
import chisel3.util._

/**
 * Integer ALU + branch/jump resolution unit.
 *
 * LATENCY: 1 cycle. The arithmetic/compare/shift result and the branch resolution are computed
 * combinationally from the incoming ExecReq, then REGISTERED so the WbPort appears on the common
 * data bus the cycle AFTER the request fires. `req.ready` is always high (this unit is fully
 * pipelined / never stalls), so back-to-back requests are accepted every cycle. This 1-cycle
 * registered output is consistent with the IssueQueue's 1-cycle-late wakeup model.
 *
 * OPERATIONS:
 *   LUI                    -> immediate
 *   AUIPC                  -> pc + immediate
 *   OP-IMM (opcode 0010011): ADDI/SLTI/SLTIU/XORI/ORI/ANDI/SLLI/SRLI/SRAI
 *   OP     (opcode 0110011): ADD/SUB/SLL/SLT/SLTU/XOR/SRL/SRA/OR/AND
 *
 * ADD vs SUB and SRL vs SRA are selected by func7 bit 5 (== instruction bit 30). For OP-IMM the
 * SRLI/SRAI choice is the SAME instruction bit 30, which the Decoder also exposes as func7(5)
 * (func7 = instruction(31,25)); we therefore use func7(5) uniformly for both OP and OP-IMM
 * shift-direction selection. SUB is only legal for OP (register-register); OP-IMM never subtracts.
 *
 * BRANCH/JUMP: for isBranch evaluate the condition per func3 and target = pc + immediate; for JAL
 * target = pc + immediate, result = pc + 4; for JALR target = (rs1 + immediate) & ~1, result =
 * pc + 4. `mispredicted` compares the predicted next-PC against the actual next-PC.
 */
class ExecAlu(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val req = Flipped(Decoupled(new ExecReq(p)))
        val wb = Output(new WbPort(p))
    })

    // Fully pipelined: always able to accept.
    io.req.ready := true.B

    val uop = io.req.bits.uop
    val rs1 = io.req.bits.rs1Data
    val rs2 = io.req.bits.rs2Data
    val imm = uop.immediate
    val pc = uop.pc

    // OP-IMM uses the immediate as operand B; OP uses rs2.
    val isOp = uop.opcode === "b0110011".U
    val isOpImm = uop.opcode === "b0010011".U
    val isLui = uop.opcode === "b0110111".U
    val isAuipc = uop.opcode === "b0010111".U

    val opB = Mux(isOp, rs2, imm)

    // SUB is only for register-register OP and only when func7 bit 5 is set.
    val doSub = isOp && uop.func7(5)
    // SRA / SRAI when func7 bit 5 set (instruction bit 30); else SRL / SRLI (logical).
    val arithShift = uop.func7(5)

    val shamt = opB(4, 0) // low 5 bits for RV32 shifts

    val addSub = Mux(doSub, rs1 - opB, rs1 + opB)
    val sll = rs1 << shamt
    val srl = rs1 >> shamt
    val sra = (rs1.asSInt >> shamt).asUInt
    val slt = (rs1.asSInt < opB.asSInt).asUInt // signed
    val sltu = (rs1 < opB).asUInt // unsigned
    val xorR = rs1 ^ opB
    val orR = rs1 | opB
    val andR = rs1 & opB

    // Arithmetic result by func3 (shared decode for OP and OP-IMM).
    val aluByFunc3 = WireDefault(addSub)
    switch(uop.func3) {
        is("b000".U) { aluByFunc3 := addSub } // ADD/SUB/ADDI
        is("b001".U) { aluByFunc3 := sll(p.xlen - 1, 0) } // SLL/SLLI
        is("b010".U) { aluByFunc3 := slt } // SLT/SLTI
        is("b011".U) { aluByFunc3 := sltu } // SLTU/SLTIU
        is("b100".U) { aluByFunc3 := xorR } // XOR/XORI
        is("b101".U) { aluByFunc3 := Mux(arithShift, sra, srl) } // SRL/SRA/SRLI/SRAI
        is("b110".U) { aluByFunc3 := orR } // OR/ORI
        is("b111".U) { aluByFunc3 := andR } // AND/ANDI
    }

    // ---- final non-control result ----
    val aluResult = WireDefault(aluByFunc3)
    when(isLui) { aluResult := imm }
        .elsewhen(isAuipc) { aluResult := pc + imm }
        .elsewhen(uop.isJump || uop.isJalr) { aluResult := pc + 4.U } // link register

    // ---- branch condition (func3) ----
    val eq = rs1 === rs2
    val lt = rs1.asSInt < rs2.asSInt
    val ltu = rs1 < rs2
    val brCond = WireDefault(false.B)
    switch(uop.func3) {
        is("b000".U) { brCond := eq } // BEQ
        is("b001".U) { brCond := !eq } // BNE
        is("b100".U) { brCond := lt } // BLT
        is("b101".U) { brCond := !lt } // BGE
        is("b110".U) { brCond := ltu } // BLTU
        is("b111".U) { brCond := !ltu } // BGEU
    }

    // ---- next-PC resolution ----
    val brTaken = WireDefault(false.B)
    val actualTarget = WireDefault(pc + 4.U)
    when(uop.isBranch) {
        brTaken := brCond
        actualTarget := Mux(brCond, pc + imm, pc + 4.U)
    }.elsewhen(uop.isJump) {
        brTaken := true.B
        actualTarget := pc + imm
    }.elsewhen(uop.isJalr) {
        brTaken := true.B
        actualTarget := (rs1 + imm) & (~1.U(p.xlen.W))
    }

    val isCtrl = uop.isBranchOrJump
    // Predicted next-PC: predTaken/predTarget if predicted-taken, else fall-through pc+4.
    val predNextPc = Mux(uop.predTaken, uop.predTarget, pc + 4.U)
    val mispred = isCtrl && (predNextPc =/= actualTarget)

    // ---- register the WbPort (1-cycle latency) ----
    val wbReg = RegInit(WbPort.default(p))
    wbReg.valid := io.req.fire
    wbReg.robIdx := uop.robIdx
    wbReg.pdst := uop.pdst
    wbReg.writesReg := uop.writesReg
    wbReg.data := aluResult
    wbReg.isBranchResolved := io.req.fire && isCtrl
    wbReg.mispredicted := isCtrl && mispred
    wbReg.brTaken := brTaken
    wbReg.brTarget := actualTarget
    wbReg.exception := false.B

    io.wb := wbReg
}