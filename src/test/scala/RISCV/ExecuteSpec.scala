package RISCV

import chisel3._
import chisel3.util._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.should.Matchers

/**
 * ChiselSim tests for the OoO execute stage: PhysRegFile, ExecAlu, ExecMulDiv.
 *
 * All tests are deterministic. Helpers build MicroOp / ExecReq literals field-by-field so the
 * tests do not depend on a Decoder. Encodings used:
 *   OP     opcode = 0110011, OP-IMM opcode = 0010011, LUI = 0110111, AUIPC = 0010111
 *   SUB: func7 = 0100000 (bit5 set). SRA/SRAI: func7 bit5 set. RV32M: func7 = 0000001.
 */
class ExecuteSpec extends AnyFreeSpec with Matchers with ChiselSim {

    val p = OoOParams()

    // --------------------------------------------------------------------------------------
    // PhysRegFile
    // --------------------------------------------------------------------------------------
    "PhysRegFile" - {
        "x0 (zeroPreg) always reads 0 even after a write attempt" in {
            simulate(new PhysRegFile(p)) { dut =>
                // Attempt to write the zero preg on port 0 -- must be ignored.
                dut.io.wb(0).valid.poke(true.B)
                dut.io.wb(0).writesReg.poke(true.B)
                dut.io.wb(0).pdst.poke(0.U)
                dut.io.wb(0).data.poke("hdeadbeef".U)
                dut.io.read(0).addr.poke(0.U)
                dut.io.read(0).data.expect(0.U)
                dut.clock.step()
                dut.io.wb(0).valid.poke(false.B)
                dut.io.read(0).addr.poke(0.U)
                dut.io.read(0).data.expect(0.U)
            }
        }

        "write then read on the NEXT cycle returns the stored value" in {
            simulate(new PhysRegFile(p)) { dut =>
                dut.io.wb(0).valid.poke(true.B)
                dut.io.wb(0).writesReg.poke(true.B)
                dut.io.wb(0).pdst.poke(5.U)
                dut.io.wb(0).data.poke("h12345678".U)
                dut.clock.step()
                dut.io.wb(0).valid.poke(false.B)
                dut.io.read(2).addr.poke(5.U)
                dut.io.read(2).data.expect("h12345678".U)
            }
        }

        // NOTE: the write-to-read bypass was intentionally REMOVED at integration time (it
        // formed a combinational cycle with the LSU's same-cycle forwarded-load writeback, and
        // is redundant given the issue queue's 1-cycle-late wakeup). A same-cycle read therefore
        // returns the OLD (registered) value, not the value being written this cycle.
        "no same-cycle bypass: a read in the write cycle returns the OLD value" in {
            simulate(new PhysRegFile(p)) { dut =>
                // Seed preg 9 with a known value on the prior cycle.
                dut.io.wb(0).valid.poke(true.B)
                dut.io.wb(0).writesReg.poke(true.B)
                dut.io.wb(0).pdst.poke(9.U)
                dut.io.wb(0).data.poke("h11111111".U)
                dut.clock.step()
                dut.io.wb(0).valid.poke(false.B)

                // Now drive a NEW write to preg 9 and read it the same cycle: must see the OLD value.
                dut.io.wb(1).valid.poke(true.B)
                dut.io.wb(1).writesReg.poke(true.B)
                dut.io.wb(1).pdst.poke(9.U)
                dut.io.wb(1).data.poke("hcafef00d".U)
                dut.io.read(4).addr.poke(9.U)
                dut.io.read(4).data.expect("h11111111".U) // registered value, no bypass
                // After the clock edge the new value is visible.
                dut.clock.step()
                dut.io.wb(1).valid.poke(false.B)
                dut.io.read(4).addr.poke(9.U)
                dut.io.read(4).data.expect("hcafef00d".U)
            }
        }
    }

    // --------------------------------------------------------------------------------------
    // MicroOp poke helper. ChiselSim cannot poke an aggregate from a hardware Wire, so we poke
    // every relevant leaf field of the DUT's uop port directly (defaulting the rest to 0).
    // --------------------------------------------------------------------------------------
    private def pokeUop(
        u: MicroOp,
        opcode: Int = 0,
        func3: Int = 0,
        func7: Int = 0,
        imm: Long = 0,
        pc: Long = 0,
        writesReg: Boolean = true,
        pdst: Int = 1,
        prs1: Int = 0,
        prs2: Int = 0,
        readsRs1: Boolean = false,
        readsRs2: Boolean = false,
        prs1Ready: Boolean = false,
        prs2Ready: Boolean = false,
        robIdx: Int = 0,
        fuType: FuType.Type = FuType.ALU,
        isBranch: Boolean = false,
        isJump: Boolean = false,
        isJalr: Boolean = false,
        predTaken: Boolean = false,
        predTarget: Long = 0
    ): Unit = {
        // Zero everything first (covers all the fields we do not explicitly set).
        u.opcode.poke(0.U); u.func3.poke(0.U); u.func7.poke(0.U)
        u.immediate.poke(0.U); u.pc.poke(0.U); u.fuType.poke(FuType.ALU)
        u.lrs1.poke(0.U); u.lrs2.poke(0.U); u.ldst.poke(0.U)
        u.readsRs1.poke(false.B); u.readsRs2.poke(false.B); u.writesReg.poke(false.B)
        u.isBranch.poke(false.B); u.isJump.poke(false.B); u.isJalr.poke(false.B)
        u.isLoad.poke(false.B); u.isStore.poke(false.B)
        u.prs1.poke(0.U); u.prs2.poke(0.U); u.pdst.poke(0.U); u.pdstOld.poke(0.U)
        u.prs1Ready.poke(false.B); u.prs2Ready.poke(false.B)
        u.robIdx.poke(0.U); u.ldqIdx.poke(0.U); u.stqIdx.poke(0.U)
        u.predTaken.poke(false.B); u.predTarget.poke(0.U)
        u.brMask.poke(0.U); u.brTag.poke(0.U)
        // Now the requested overrides.
        u.opcode.poke(opcode.U)
        u.func3.poke(func3.U)
        u.func7.poke(func7.U)
        u.immediate.poke((imm & 0xffffffffL).U(p.xlen.W))
        u.pc.poke((pc & 0xffffffffL).U(p.xlen.W))
        u.writesReg.poke(writesReg.B)
        u.pdst.poke(pdst.U)
        u.prs1.poke(prs1.U)
        u.prs2.poke(prs2.U)
        u.readsRs1.poke(readsRs1.B)
        u.readsRs2.poke(readsRs2.B)
        u.prs1Ready.poke(prs1Ready.B)
        u.prs2Ready.poke(prs2Ready.B)
        u.robIdx.poke(robIdx.U)
        u.fuType.poke(fuType)
        u.isBranch.poke(isBranch.B)
        u.isJump.poke(isJump.B)
        u.isJalr.poke(isJalr.B)
        u.predTaken.poke(predTaken.B)
        u.predTarget.poke((predTarget & 0xffffffffL).U(p.xlen.W))
    }

    // --------------------------------------------------------------------------------------
    // ExecAlu
    // --------------------------------------------------------------------------------------
    "ExecAlu" - {
        // Poke the uop (via `setup`) and operands, step once (1-cycle registered output).
        def runAlu(dut: ExecAlu, rs1: Long, rs2: Long)(setup: MicroOp => Unit): Unit = {
            dut.io.req.valid.poke(true.B)
            setup(dut.io.req.bits.uop)
            dut.io.req.bits.rs1Data.poke((rs1 & 0xffffffffL).U(p.xlen.W))
            dut.io.req.bits.rs2Data.poke((rs2 & 0xffffffffL).U(p.xlen.W))
            dut.clock.step()
            dut.io.req.valid.poke(false.B)
        }

        "ADD (OP, func7=0) adds rs1+rs2" in {
            simulate(new ExecAlu(p)) { dut =>
                runAlu(dut, 10, 32)(pokeUop(_, opcode = 0x33, func3 = 0, func7 = 0))
                dut.io.wb.valid.expect(true.B)
                dut.io.wb.data.expect(42.U)
            }
        }

        "SUB (OP, func7 bit5 set) subtracts rs1-rs2" in {
            simulate(new ExecAlu(p)) { dut =>
                runAlu(dut, 50, 8)(pokeUop(_, opcode = 0x33, func3 = 0, func7 = 0x20))
                dut.io.wb.data.expect(42.U)
            }
        }

        "ADDI (OP-IMM, func7 bit5 set) still ADDS (does not subtract)" in {
            simulate(new ExecAlu(p)) { dut =>
                // func7=0x20 here is just the upper imm bits; OP-IMM never subtracts.
                runAlu(dut, 1, 0)(pokeUop(_, opcode = 0x13, func3 = 0, func7 = 0x20, imm = 0x7ff))
                dut.io.wb.data.expect((1 + 0x7ff).U)
            }
        }

        "SRA (OP, func7 bit5 set) is arithmetic right shift (sign-extends)" in {
            simulate(new ExecAlu(p)) { dut =>
                // 0xFFFFFFF0 >>a 4 = 0xFFFFFFFF
                runAlu(dut, 0xfffffff0L, 4)(pokeUop(_, opcode = 0x33, func3 = 5, func7 = 0x20))
                dut.io.wb.data.expect("hffffffff".U)
            }
        }

        "SRL (OP, func7=0) is logical right shift (zero-fills)" in {
            simulate(new ExecAlu(p)) { dut =>
                runAlu(dut, 0xfffffff0L, 4)(pokeUop(_, opcode = 0x33, func3 = 5, func7 = 0))
                dut.io.wb.data.expect("h0fffffff".U)
            }
        }

        "SRAI (OP-IMM, imm bit10 / func7 bit5 set) is arithmetic" in {
            simulate(new ExecAlu(p)) { dut =>
                // SRAI shamt=4, func7=0x20 sets the arithmetic bit; imm low5 = shamt.
                runAlu(dut, 0x80000000L, 0)(pokeUop(_, opcode = 0x13, func3 = 5, func7 = 0x20, imm = 0x404))
                dut.io.wb.data.expect("hf8000000".U)
            }
        }

        "SLT signed compare: -1 < 1 is true" in {
            simulate(new ExecAlu(p)) { dut =>
                runAlu(dut, 0xffffffffL, 1)(pokeUop(_, opcode = 0x33, func3 = 2, func7 = 0))
                dut.io.wb.data.expect(1.U)
            }
        }

        "BEQ taken, correctly predicted-not-taken => mispredicted" in {
            simulate(new ExecAlu(p)) { dut =>
                // BEQ pc=0x100, imm=0x10 (target 0x110), rs1==rs2 => taken. Predicted not taken.
                runAlu(dut, 7, 7)(
                    pokeUop(_, opcode = 0x63, func3 = 0, imm = 0x10, pc = 0x100, writesReg = false, isBranch = true, predTaken = false)
                )
                dut.io.wb.isBranchResolved.expect(true.B)
                dut.io.wb.brTaken.expect(true.B)
                dut.io.wb.brTarget.expect(0x110.U)
                dut.io.wb.mispredicted.expect(true.B)
            }
        }

        "BEQ not-taken, correctly predicted-not-taken => NOT mispredicted" in {
            simulate(new ExecAlu(p)) { dut =>
                runAlu(dut, 7, 8)(
                    pokeUop(_, opcode = 0x63, func3 = 0, imm = 0x10, pc = 0x100, writesReg = false, isBranch = true, predTaken = false)
                )
                dut.io.wb.brTaken.expect(false.B)
                dut.io.wb.brTarget.expect(0x104.U) // fall through
                dut.io.wb.mispredicted.expect(false.B)
            }
        }

        "JAL writes link = pc+4 and resolves target = pc+imm" in {
            simulate(new ExecAlu(p)) { dut =>
                runAlu(dut, 0, 0)(
                    pokeUop(_, opcode = 0x6f, imm = 0x20, pc = 0x200, isJump = true, predTaken = true, predTarget = 0x220)
                )
                dut.io.wb.data.expect(0x204.U)
                dut.io.wb.brTarget.expect(0x220.U)
                dut.io.wb.mispredicted.expect(false.B) // predicted target matched
            }
        }
    }

    // --------------------------------------------------------------------------------------
    // ExecMulDiv
    // --------------------------------------------------------------------------------------
    "ExecMulDiv" - {
        val RV32M = 0x01 // func7

        "MUL low word (combinational, 1-cycle)" in {
            simulate(new ExecMulDiv(p)) { dut =>
                dut.io.redirect.valid.poke(false.B)
                dut.io.req.valid.poke(true.B)
                dut.io.req.ready.expect(true.B)
                pokeUop(dut.io.req.bits.uop, opcode = 0x33, func3 = 0, func7 = RV32M, fuType = FuType.MULDIV)
                dut.io.req.bits.rs1Data.poke(6.U)
                dut.io.req.bits.rs2Data.poke(7.U)
                dut.clock.step()
                dut.io.req.valid.poke(false.B)
                dut.io.wb.valid.expect(true.B)
                dut.io.wb.data.expect(42.U)
            }
        }

        // Drive a divide request and step until wb.valid, then check the result.
        def runDiv(dut: ExecMulDiv, func3: Int, rs1: Long, rs2: Long): Long = {
            dut.io.redirect.valid.poke(false.B)
            dut.io.req.valid.poke(true.B)
            pokeUop(dut.io.req.bits.uop, opcode = 0x33, func3 = func3, func7 = 0x01, fuType = FuType.MULDIV)
            dut.io.req.bits.rs1Data.poke((rs1 & 0xffffffffL).U(p.xlen.W))
            dut.io.req.bits.rs2Data.poke((rs2 & 0xffffffffL).U(p.xlen.W))
            // First cycle: request is accepted (ready high), FSM enters busy on the edge.
            dut.io.req.ready.expect(true.B)
            dut.clock.step()
            dut.io.req.valid.poke(false.B)
            // Busy for xlen iterations then sDone. Step generously and capture wb.valid.
            var result: Long = -1L
            var seen = false
            for (_ <- 0 until (p.xlen + 4)) {
                if (!seen && dut.io.wb.valid.peek().litToBoolean) {
                    result = dut.io.wb.data.peek().litValue.toLong & 0xffffffffL
                    seen = true
                }
                dut.clock.step()
            }
            assert(seen, "divider never produced a writeback")
            result
        }

        // Dense sweep -- the ISA suite uses curated vectors, which can miss a radix-K divider bug
        // that only shows for certain bit patterns. This mimics itoa (repeated /10, %10), the exact
        // operation Doom's vsnprintf hung on. Every value is checked against Scala's own div/rem.
        "DIVU/REMU dense sweep (itoa pattern: /10, %10, and assorted divisors)" in {
            simulate(new ExecMulDiv(p)) { dut =>
                val divisors = Seq(1L, 2L, 3L, 7L, 10L, 16L, 100L, 255L, 1000L)
                val dividends = (0L to 300L) ++ Seq(0xffffffffL, 0x80000000L, 0x7fffffffL, 12345L, 65535L)
                for (d <- divisors; n <- dividends) {
                    val gotQ = runDiv(dut, func3 = 5, rs1 = n, rs2 = d) & 0xffffffffL // DIVU
                    val gotR = runDiv(dut, func3 = 7, rs1 = n, rs2 = d) & 0xffffffffL // REMU
                    val expQ = (n & 0xffffffffL) / d
                    val expR = (n & 0xffffffffL) % d
                    withClue(f"DIVU $n%d / $d%d: ") { gotQ shouldBe expQ }
                    withClue(f"REMU $n%d %% $d%d: ") { gotR shouldBe expR }
                }
            }
        }

        "DIV signed: -20 / 3 = -6 (truncating toward zero)" in {
            simulate(new ExecMulDiv(p)) { dut =>
                val q = runDiv(dut, func3 = 4, rs1 = -20L & 0xffffffffL, rs2 = 3)
                // -6 in 32-bit two's complement
                q shouldBe (0xffffffffL - 6 + 1) // 0xFFFFFFFA
            }
        }

        "DIVU unsigned: 20 / 3 = 6" in {
            simulate(new ExecMulDiv(p)) { dut =>
                runDiv(dut, func3 = 5, rs1 = 20, rs2 = 3) shouldBe 6L
            }
        }

        "REM signed: -20 % 3 = -2 (sign of dividend)" in {
            simulate(new ExecMulDiv(p)) { dut =>
                val r = runDiv(dut, func3 = 6, rs1 = -20L & 0xffffffffL, rs2 = 3)
                r shouldBe (0x100000000L - 2) // 0xFFFFFFFE
            }
        }

        "DIV by zero: quotient = all-ones (-1)" in {
            simulate(new ExecMulDiv(p)) { dut =>
                runDiv(dut, func3 = 4, rs1 = 1234, rs2 = 0) shouldBe 0xffffffffL
            }
        }

        "REM by zero: remainder = dividend" in {
            simulate(new ExecMulDiv(p)) { dut =>
                runDiv(dut, func3 = 6, rs1 = 1234, rs2 = 0) shouldBe 1234L
            }
        }

        "DIV overflow INT_MIN / -1 = INT_MIN" in {
            simulate(new ExecMulDiv(p)) { dut =>
                runDiv(dut, func3 = 4, rs1 = 0x80000000L, rs2 = -1L & 0xffffffffL) shouldBe 0x80000000L
            }
        }

        "REM overflow INT_MIN / -1 = 0" in {
            simulate(new ExecMulDiv(p)) { dut =>
                runDiv(dut, func3 = 6, rs1 = 0x80000000L, rs2 = -1L & 0xffffffffL) shouldBe 0L
            }
        }
    }
}
