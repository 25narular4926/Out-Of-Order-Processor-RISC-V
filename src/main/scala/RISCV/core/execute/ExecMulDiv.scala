package RISCV

import chisel3._
import chisel3.util._

/**
 * RV32M multiply / divide unit. Only uops with opcode 0110011 and func7 0000001 reach it.
 *
 *   func3: 000 MUL    001 MULH   010 MULHSU  011 MULHU
 *          100 DIV    101 DIVU   110 REM     111 REMU
 *
 * LATENCY MODEL:
 *   - Multiplies (MUL/MULH/MULHSU/MULHU) are COMBINATIONAL and complete in a single cycle: the
 *     full 64-bit product is computed and the requested half registered, so the WbPort appears
 *     on the CDB the cycle after the request fires (1-cycle latency, like ExecAlu).
 *   - Divides/remainders (DIV/DIVU/REM/REMU) use a MULTI-CYCLE iterative restoring divider:
 *     `xlen` (=32) iterations of 1 bit each. While busy, `req.ready` is deasserted so no new op
 *     is accepted; `wb.valid` pulses for exactly one cycle when the quotient/remainder is ready.
 *
 * The Core must therefore not assume a fixed MULDIV latency -- it relies on req.ready / wb.valid
 * handshaking, which the IssueQueue already honors (it only deallocates a slot when the issue
 * port fires, i.e. when req.ready is high).
 *
 * ARCHITECTURAL CORNER CASES (RV32M):
 *   - divide by zero: DIV/DIVU quotient = all-ones (-1); REM/REMU remainder = dividend.
 *   - signed overflow INT_MIN / -1: DIV quotient = INT_MIN; REM remainder = 0.
 *
 * RECOVERY: on `redirect.valid` an in-flight divide is aborted (FSM returns to idle, no WbPort
 * is produced) -- consistent with the commit-time full-flush model.
 */
class ExecMulDiv(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val req = Flipped(Decoupled(new ExecReq(p)))
        val wb = Output(new WbPort(p))
        val redirect = Input(new Redirect(p))
    })

    val xlen = p.xlen

    // ---------------------------------------------------------------------------------------
    // Decode the incoming request.
    // ---------------------------------------------------------------------------------------
    val uop = io.req.bits.uop
    val a = io.req.bits.rs1Data
    val b = io.req.bits.rs2Data
    val f3 = uop.func3

    val isMulOp = !f3(2) // func3 0xx => multiply family
    val isDivOp = f3(2) // func3 1xx => divide / remainder family

    // ---------------------------------------------------------------------------------------
    // Multiply (combinational). Sign-extend operands as required per func3.
    // ---------------------------------------------------------------------------------------
    val aSext = a.asSInt
    val bSext = b.asSInt
    val aZext = a.zext // unsigned -> wider signed (top bit 0)
    val bZext = b.zext

    // MUL/MULH: signed*signed; MULHSU: signed*unsigned; MULHU: unsigned*unsigned.
    val prodSS = (aSext * bSext) // signed * signed
    val prodSU = (aSext * bZext) // signed(rs1) * unsigned(rs2)
    val prodUU = (aZext * bZext) // unsigned * unsigned

    val mulResult = WireDefault(0.U(xlen.W))
    switch(f3(1, 0)) {
        is("b00".U) { mulResult := prodSS(xlen - 1, 0) } // MUL  -> low word
        is("b01".U) { mulResult := prodSS(2 * xlen - 1, xlen) } // MULH (signed x signed)
        is("b10".U) { mulResult := prodSU(2 * xlen - 1, xlen) } // MULHSU
        is("b11".U) { mulResult := prodUU(2 * xlen - 1, xlen) } // MULHU
    }

    // ---------------------------------------------------------------------------------------
    // Divide / remainder: iterative restoring divider FSM.
    // ---------------------------------------------------------------------------------------
    val isSigned = !f3(0) // func3 100 DIV, 110 REM => signed; 101/111 unsigned
    val wantRem = f3(1) // func3 11x => remainder; 10x => quotient

    val sIdle :: sBusy :: sDone :: Nil = Enum(3)
    val state = RegInit(sIdle)

    // Working registers for the restoring divider.
    val quotient = Reg(UInt(xlen.W))
    val remainder = Reg(UInt(xlen.W))
    val divisorReg = Reg(UInt(xlen.W))
    val count = Reg(UInt(log2Ceil(xlen + 1).W))

    // Sign bookkeeping captured at start.
    val negQuot = Reg(Bool()) // quotient should be negated
    val negRem = Reg(Bool()) // remainder should be negated (== sign of dividend)
    val divByZero = Reg(Bool())
    val overflow = Reg(Bool()) // INT_MIN / -1
    val origDividend = Reg(UInt(xlen.W)) // for div-by-zero REM result
    // Latched control needed to build the WbPort when done.
    val divWantRem = Reg(Bool())
    val divRobIdx = Reg(UInt(p.robIdxWidth.W))
    val divPdst = Reg(UInt(p.pregWidth.W))
    val divWrites = Reg(Bool())

    val intMin = (1.U << (xlen - 1)).asUInt // 0x80000000

    // Accept a new op only when idle.
    val acceptDiv = io.req.fire && isDivOp
    io.req.ready := state === sIdle

    when(state === sIdle) {
        when(acceptDiv) {
            // Magnitudes for signed division; raw values for unsigned.
            val aNeg = isSigned && a(xlen - 1)
            val bNeg = isSigned && b(xlen - 1)
            val aMag = Mux(aNeg, (~a) + 1.U, a)
            val bMag = Mux(bNeg, (~b) + 1.U, b)

            divByZero := (b === 0.U)
            overflow := isSigned && (a === intMin) && (b === "hFFFFFFFF".U)

            negQuot := aNeg ^ bNeg
            negRem := aNeg // remainder takes the sign of the dividend
            origDividend := a

            remainder := 0.U
            quotient := aMag
            divisorReg := bMag
            count := 0.U

            divWantRem := wantRem
            divRobIdx := uop.robIdx
            divPdst := uop.pdst
            divWrites := uop.writesReg

            state := sBusy
        }
    }.elsewhen(state === sBusy) {
        // One restoring-division step per cycle. We shift the (remainder:quotient) pair left by
        // one, trial-subtract the divisor from the remainder, and set the LSB of the quotient if
        // it fit. After `xlen` steps quotient/remainder hold the unsigned results.
        val shifted = Cat(remainder(xlen - 2, 0), quotient(xlen - 1)) // (rem << 1) | quot MSB
        val trial = shifted - divisorReg
        val fits = shifted >= divisorReg
        remainder := Mux(fits, trial, shifted)
        quotient := Cat(quotient(xlen - 2, 0), fits.asUInt) // (quot << 1) | fits
        count := count + 1.U
        when(count === (xlen - 1).U) { state := sDone }
    }.elsewhen(state === sDone) {
        state := sIdle
    }

    // Abort an in-flight (or finishing) divide on redirect.
    when(io.redirect.valid) {
        state := sIdle
    }

    // Final, sign-corrected division results.
    val finalQuot = Mux(negQuot, (~quotient) + 1.U, quotient)
    val finalRem = Mux(negRem, (~remainder) + 1.U, remainder)

    // Apply corner cases (priority: overflow over normal; div-by-zero handled separately).
    val divOut = Wire(UInt(xlen.W))
    when(divByZero) {
        divOut := Mux(divWantRem, origDividend, "hFFFFFFFF".U) // REM=dividend, DIV=-1
    }.elsewhen(overflow) {
        divOut := Mux(divWantRem, 0.U, intMin) // REM=0, DIV=INT_MIN
    }.otherwise {
        divOut := Mux(divWantRem, finalRem, finalQuot)
    }

    // ---------------------------------------------------------------------------------------
    // Writeback. Multiplies register their result one cycle after the request fires; divides
    // pulse valid for one cycle in the sDone state.
    // ---------------------------------------------------------------------------------------
    val mulValid = RegInit(false.B)
    val mulWb = Reg(new WbPort(p))
    mulValid := io.req.fire && isMulOp
    when(io.req.fire && isMulOp) {
        mulWb := WbPort.default(p)
        mulWb.robIdx := uop.robIdx
        mulWb.pdst := uop.pdst
        mulWb.writesReg := uop.writesReg
        mulWb.data := mulResult
    }
    // A multiply in flight is also squashed by redirect (its result must not reach the CDB).
    when(io.redirect.valid) { mulValid := false.B }

    val divFinishing = (state === sDone) && !io.redirect.valid

    val wb = WireDefault(WbPort.default(p))
    when(mulValid) {
        wb := mulWb
        wb.valid := true.B
    }.elsewhen(divFinishing) {
        wb := WbPort.default(p)
        wb.valid := true.B
        wb.robIdx := divRobIdx
        wb.pdst := divPdst
        wb.writesReg := divWrites
        wb.data := divOut
    }
    io.wb := wb
}
