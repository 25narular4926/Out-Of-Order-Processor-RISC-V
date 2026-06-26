package RISCV

import chisel3._
import chisel3.util._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.should.Matchers

/**
 * Front-end unit tests: Decode field extraction, Rename tag allocation + busy behaviour, and a
 * Frontend elaboration/dispatch smoke test. Deterministic (fixed instruction words, no randomness).
 *
 * Instruction words used (verified):
 *   0x00500093  addi x1, x0, 5      (OP-IMM, writes x1, reads x0)
 *   0x002081b3  add  x3, x1, x2     (OP/ALU, writes x3, reads x1,x2)
 *   0x024182b3  mul  x5, x3, x4     (OP/MULDIV, writes x5, reads x3,x4)
 *   0x0000a303  lw   x6, 0(x1)      (LOAD/MEM, writes x6, reads x1)
 *   0x00208023  sb? no -> use store below
 */
class FrontendSpec extends AnyFreeSpec with Matchers with ChiselSim {

    val p = OoOParams()

    // -------------------------------------------------------------------------------------
    // DECODE
    // -------------------------------------------------------------------------------------
    "Decode extracts fields and classifies fuType for a known instruction stream" in {
        simulate(new Decode(p)) { dut =>
            dut.io.out.ready.poke(true.B)

            // ---- addi x1, x0, 5 : ALU, writes x1, reads rs1(x0), imm=5 ----
            dut.io.in.valid.poke(true.B)
            dut.io.in.bits.pc.poke(0x100.U)
            dut.io.in.bits.instruction.poke("h00500093".U)
            dut.io.in.bits.predTaken.poke(false.B)
            dut.io.in.bits.predTarget.poke(0.U)

            dut.io.out.valid.expect(true.B)
            dut.io.out.bits.opcode.expect("b0010011".U)
            dut.io.out.bits.ldst.expect(1.U)
            dut.io.out.bits.lrs1.expect(0.U)
            dut.io.out.bits.immediate.expect(5.U)
            dut.io.out.bits.writesReg.expect(true.B)
            dut.io.out.bits.readsRs1.expect(true.B)
            dut.io.out.bits.readsRs2.expect(false.B)
            dut.io.out.bits.fuType.expect(FuType.ALU)
            dut.io.out.bits.pc.expect(0x100.U)

            // ---- mul x5, x3, x4 : MULDIV ----
            dut.io.in.bits.instruction.poke("h024182b3".U)
            dut.io.out.bits.fuType.expect(FuType.MULDIV)
            dut.io.out.bits.writesReg.expect(true.B)
            dut.io.out.bits.ldst.expect(5.U)
            dut.io.out.bits.lrs1.expect(3.U)
            dut.io.out.bits.lrs2.expect(4.U)
            dut.io.out.bits.readsRs2.expect(true.B)

            // ---- lw x6, 0(x1) : MEM load ----
            dut.io.in.bits.instruction.poke("h0000a303".U)
            dut.io.out.bits.fuType.expect(FuType.MEM)
            dut.io.out.bits.isLoad.expect(true.B)
            dut.io.out.bits.isStore.expect(false.B)
            dut.io.out.bits.writesReg.expect(true.B)
            dut.io.out.bits.ldst.expect(6.U)

            // ---- add x0, x1, x2 : writesReg must be FALSE because rd == x0 ----
            // add x0,x1,x2 = 0x00208033
            dut.io.in.bits.instruction.poke("h00208033".U)
            dut.io.out.bits.writesReg.expect(false.B)
            dut.io.out.bits.fuType.expect(FuType.ALU)
        }
    }

    // -------------------------------------------------------------------------------------
    // RENAME
    // -------------------------------------------------------------------------------------
    "Rename allocates physical tags, records pdstOld, and tracks source readiness" in {
        simulate(new Rename(p)) { dut =>
            // helper: build a writing ALU uop (add ldst, lrs1, lrs2)
            def driveUop(
                ldst: Int,
                lrs1: Int,
                lrs2: Int,
                writes: Boolean,
                readsR1: Boolean,
                readsR2: Boolean
            ): Unit = {
                dut.io.in.bits.pc.poke(0.U)
                dut.io.in.bits.opcode.poke("b0110011".U)
                dut.io.in.bits.func3.poke(0.U)
                dut.io.in.bits.func7.poke(0.U)
                dut.io.in.bits.immediate.poke(0.U)
                dut.io.in.bits.fuType.poke(FuType.ALU)
                dut.io.in.bits.lrs1.poke(lrs1.U)
                dut.io.in.bits.lrs2.poke(lrs2.U)
                dut.io.in.bits.ldst.poke(ldst.U)
                dut.io.in.bits.readsRs1.poke(readsR1.B)
                dut.io.in.bits.readsRs2.poke(readsR2.B)
                dut.io.in.bits.writesReg.poke(writes.B)
                dut.io.in.bits.isBranch.poke(false.B)
                dut.io.in.bits.isJump.poke(false.B)
                dut.io.in.bits.isJalr.poke(false.B)
                dut.io.in.bits.isLoad.poke(false.B)
                dut.io.in.bits.isStore.poke(false.B)
                dut.io.in.bits.predTaken.poke(false.B)
                dut.io.in.bits.predTarget.poke(0.U)
                dut.io.in.bits.prs1.poke(0.U)
                dut.io.in.bits.prs2.poke(0.U)
                dut.io.in.bits.pdst.poke(0.U)
                dut.io.in.bits.pdstOld.poke(0.U)
                dut.io.in.bits.prs1Ready.poke(false.B)
                dut.io.in.bits.prs2Ready.poke(false.B)
                dut.io.in.bits.robIdx.poke(0.U)
                dut.io.in.bits.ldqIdx.poke(0.U)
                dut.io.in.bits.stqIdx.poke(0.U)
                dut.io.in.bits.brMask.poke(0.U)
                dut.io.in.bits.brTag.poke(0.U)
            }

            // quiesce back-end inputs
            def quietBackEnd(): Unit = {
                for (w <- 0 until p.numWbPorts) {
                    dut.io.wb(w).valid.poke(false.B)
                    dut.io.wb(w).writesReg.poke(false.B)
                    dut.io.wb(w).pdst.poke(0.U)
                }
                dut.io.commit.valid.poke(false.B)
                dut.io.commit.writesReg.poke(false.B)
                dut.io.commit.ldst.poke(0.U)
                dut.io.commit.pdst.poke(0.U)
                dut.io.commit.pdstOld.poke(0.U)
                dut.io.redirect.valid.poke(false.B)
                dut.io.redirect.target.poke(0.U)
            }

            quietBackEnd()
            dut.io.out.ready.poke(true.B)

            // ---- cycle 1: add x3, x1, x2 ----
            // reset identity map: prs1 = map(1) = 1, prs2 = map(2) = 2.
            // free list lowest free preg = numArchRegs = 32 -> pdst = 32. pdstOld = map(3) = 3.
            driveUop(ldst = 3, lrs1 = 1, lrs2 = 2, writes = true, readsR1 = true, readsR2 = true)
            dut.io.in.valid.poke(true.B)
            dut.io.out.valid.expect(true.B)
            dut.io.out.bits.prs1.expect(1.U)
            dut.io.out.bits.prs2.expect(2.U)
            dut.io.out.bits.pdst.expect(p.numArchRegs.U) // 32
            dut.io.out.bits.pdstOld.expect(3.U)
            // sources x1,x2 not yet renamed by anyone -> ready
            dut.io.out.bits.prs1Ready.expect(true.B)
            dut.io.out.bits.prs2Ready.expect(true.B)
            dut.clock.step() // commit the rename: map(3):=32, busy(32):=1

            // ---- cycle 2: mul x5, x3, x4 (reads x3 just renamed to preg 32 = busy) ----
            driveUop(ldst = 5, lrs1 = 3, lrs2 = 4, writes = true, readsR1 = true, readsR2 = true)
            dut.io.in.valid.poke(true.B)
            dut.io.out.bits.prs1.expect(p.numArchRegs.U) // map(3) now 32
            dut.io.out.bits.prs2.expect(4.U) // map(4) still identity
            dut.io.out.bits.pdst.expect((p.numArchRegs + 1).U) // next free = 33
            dut.io.out.bits.pdstOld.expect(5.U) // old map(5)
            dut.io.out.bits.prs1Ready.expect(false.B) // preg 32 is busy (in flight)
            dut.io.out.bits.prs2Ready.expect(true.B)
            dut.clock.step()

            // ---- cycle 3: writeback preg 32 -> its busy bit clears ----
            dut.io.in.valid.poke(false.B)
            dut.io.wb(0).valid.poke(true.B)
            dut.io.wb(0).writesReg.poke(true.B)
            dut.io.wb(0).pdst.poke(p.numArchRegs.U) // 32
            dut.clock.step()
            dut.io.wb(0).valid.poke(false.B)

            // ---- cycle 4: read x3 (preg 32) again; now it must be ready ----
            driveUop(ldst = 7, lrs1 = 3, lrs2 = 0, writes = true, readsR1 = true, readsR2 = false)
            dut.io.in.valid.poke(true.B)
            dut.io.out.bits.prs1.expect(p.numArchRegs.U)
            dut.io.out.bits.prs1Ready.expect(true.B) // busy(32) was cleared by writeback
        }
    }

    // -------------------------------------------------------------------------------------
    // RENAME stall when the free list is exhausted
    // -------------------------------------------------------------------------------------
    "Rename deasserts in.ready when a writing uop has no free physical register" in {
        // Shrink to make exhaustion fast: 32 arch + a couple extra physical regs.
        val small = OoOParams(numPhysRegs = 64) // power-of-two requirement; 64-32 = 32 free pregs
        simulate(new Rename(small)) { dut =>
            for (w <- 0 until small.numWbPorts) {
                dut.io.wb(w).valid.poke(false.B)
                dut.io.wb(w).writesReg.poke(false.B)
                dut.io.wb(w).pdst.poke(0.U)
            }
            dut.io.commit.valid.poke(false.B)
            dut.io.commit.writesReg.poke(false.B)
            dut.io.commit.ldst.poke(0.U)
            dut.io.commit.pdst.poke(0.U)
            dut.io.commit.pdstOld.poke(0.U)
            dut.io.redirect.valid.poke(false.B)
            dut.io.redirect.target.poke(0.U)
            dut.io.out.ready.poke(true.B)

            // Drive a writing uop every cycle to a distinct arch dest, exhausting the free list.
            def driveWrite(ldst: Int): Unit = {
                dut.io.in.bits.poke(0.U.asTypeOf(new MicroOp(small)))
                dut.io.in.bits.opcode.poke("b0110011".U)
                dut.io.in.bits.fuType.poke(FuType.ALU)
                dut.io.in.bits.ldst.poke(ldst.U)
                dut.io.in.bits.writesReg.poke(true.B)
            }

            dut.io.in.valid.poke(true.B)
            var sawStall = false
            // 32 free pregs; after they are consumed, in.ready must drop for a writing uop.
            for (cyc <- 0 until 40) {
                val dest = 1 + (cyc % 31) // never x0
                driveWrite(dest)
                val ready = dut.io.in.ready.peek().litToBoolean
                if (!ready) sawStall = true
                dut.clock.step()
            }
            sawStall shouldBe true
        }
    }

    // -------------------------------------------------------------------------------------
    // FRONTEND smoke test: elaborates and produces a dispatchUop from a fed instruction.
    // -------------------------------------------------------------------------------------
    "Frontend elaborates and dispatches a uop when fed an instruction" in {
        simulate(new Frontend(p)) { dut =>
            // back-end always has room
            dut.io.robFull.poke(false.B)
            dut.io.robAllocIdx.poke(7.U)
            dut.io.iqReady.poke(true.B)
            dut.io.ldqFull.poke(false.B)
            dut.io.ldqAllocIdx.poke(0.U)
            dut.io.stqFull.poke(false.B)
            dut.io.stqAllocIdx.poke(0.U)
            for (w <- 0 until p.numWbPorts) {
                dut.io.wb(w).valid.poke(false.B)
                dut.io.wb(w).writesReg.poke(false.B)
                dut.io.wb(w).pdst.poke(0.U)
            }
            dut.io.commit.valid.poke(false.B)
            dut.io.commit.writesReg.poke(false.B)
            dut.io.commit.ldst.poke(0.U)
            dut.io.commit.pdst.poke(0.U)
            dut.io.commit.pdstOld.poke(0.U)
            dut.io.redirect.valid.poke(false.B)
            dut.io.redirect.target.poke(0.U)
            dut.io.enable.poke(true.B)

            // Simple imem model: always return addi x1, x0, 5 one cycle after the address.
            // (We don't model real memory; just feed a constant valid instruction word so the
            //  pipeline can advance. The exact PC->word mapping is irrelevant for the smoke test.)
            dut.io.imemData.poke("h00500093".U)

            // Run enough cycles for fetch(2-stage) -> decode -> rename -> dispatch to fill.
            var sawDispatch = false
            for (_ <- 0 until 16) {
                if (dut.io.dispatchValid.peek().litToBoolean) {
                    sawDispatch = true
                    // a dispatched addi writes x1 and must carry the stamped robIdx
                    dut.io.dispatchUop.writesReg.expect(true.B)
                    dut.io.dispatchUop.ldst.expect(1.U)
                    dut.io.dispatchUop.robIdx.expect(7.U)
                }
                dut.clock.step()
            }
            sawDispatch shouldBe true
        }
    }
}
