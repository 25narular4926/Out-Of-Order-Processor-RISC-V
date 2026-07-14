package RISCV

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.should.Matchers
import scala.io.Source

/**
 * End-to-end tests that run REAL COMPILED C on the out-of-order core.
 *
 * Each program is built by `sw/build.sh` (riscv64-unknown-elf-gcc, -march=rv32im) into a flat
 * hex-word image, flashed into instruction memory, and executed. The core has no I/O, so the
 * program publishes main()'s return value through a "tohost" mailbox:
 *
 *   crt0.S stores the result to byte 0x4000 == word 0x1000 == OoOParams.vgaBaseWord.
 *   Memory.scala routes any word address >= vgaBaseWord OUT of the RAM array and onto its
 *   MMIO/VGA write port -- which this harness exposes. So the store escapes the core and the
 *   testbench sees it, with no RTL changes.
 *
 * This is the first test that exercises the whole machine against compiler-generated code
 * rather than hand-built MicroOp literals.

 
 */
class ProgramSpec extends AnyFreeSpec with Matchers with ChiselSim {

    /**
     * Flash `hexPath` into the core, run it, and return the first value the program writes to
     * the tohost mailbox (or fail after `maxCycles`).
     *
     * If the image is absent, the test is CANCELLED rather than failed: these images are built
     * from C by `sw/build.sh`, which needs a RISC-V cross-compiler. A machine without the
     * toolchain (and a CI job that has not built them) should skip this suite, not report a
     * spurious failure -- the RTL is not what is missing.
     */
    def runProgram(hexPath: String, maxCycles: Int = 20000): BigInt = {
        val f = new java.io.File(hexPath)
        if (!f.exists) {
            cancel(
              s"$hexPath not found -- build it first with `sw/build.sh sw/tests/<prog>.c` " +
                "(needs riscv64-unknown-elf-gcc). Skipping."
            )
        }
        val words = Source.fromFile(f).getLines().map(_.trim).filter(_.nonEmpty).toList
        var result: Option[BigInt] = None

        simulate(new SocHarness()) { dut =>
            // ---- flash ----
            dut.io.execute.poke(false.B)
            dut.io.btns.poke(0.U)
            dut.io.flash.poke(true.B)
            words.zipWithIndex.foreach { case (w, i) =>
                dut.io.flash_address.poke(i.U)
                dut.io.flash_value.poke(java.lang.Long.parseLong(w, 16).U)
                dut.clock.step(1)
            }
            dut.io.flash.poke(false.B)

            // ---- run until the program posts to tohost ----
            dut.io.execute.poke(true.B)
            var cycle = 0
            while (result.isEmpty && cycle < maxCycles) {
                if (dut.io.tohost_valid.peek().litToBoolean) {
                    result = Some(dut.io.tohost_value.peek().litValue)
                }
                dut.clock.step(1)
                cycle += 1
            }
        }

        result.getOrElse(
          fail(s"program $hexPath never wrote to the tohost mailbox within $maxCycles cycles")
        )
    }

    // =====================================================================================
    // Performance measurement.
    //
    // The core retires in program order, so `commit_valid` counts architectural instructions and
    // `redirect_valid` counts branch mispredictions (each one costs a full pipeline flush under
    // the commit-time recovery model). Together with the cycle count these give IPC and the
    // misprediction rate -- the two numbers that say whether the branch predictor is doing
    // anything at all.
    // =====================================================================================
    case class RunStats(cycles: Int, commits: Int, redirects: Int, result: Option[BigInt]) {
        def ipc: Double = if (cycles == 0) 0.0 else commits.toDouble / cycles
        def mispredPerCommit: Double = if (commits == 0) 0.0 else redirects.toDouble / commits
        def summary: String =
            f"cycles=$cycles%-7d commits=$commits%-6d redirects=$redirects%-5d " +
                f"IPC=$ipc%.3f  mispred/instr=$mispredPerCommit%.4f"
    }

    def runWithStats(hexPath: String, maxCycles: Int = 500000): RunStats = {
        val f = new java.io.File(hexPath)
        if (!f.exists) cancel(s"$hexPath not found -- run sw/build.sh first. Skipping.")
        val words = Source.fromFile(f).getLines().map(_.trim).filter(_.nonEmpty).toList

        var cycles = 0; var commits = 0; var redirects = 0
        var result: Option[BigInt] = None

        simulate(new SocHarness()) { dut =>
            dut.io.execute.poke(false.B)
            dut.io.btns.poke(0.U)
            dut.io.flash.poke(true.B)
            words.zipWithIndex.foreach { case (w, i) =>
                dut.io.flash_address.poke(i.U)
                dut.io.flash_value.poke(java.lang.Long.parseLong(w, 16).U)
                dut.clock.step(1)
            }
            dut.io.flash.poke(false.B)

            dut.io.execute.poke(true.B)
            while (result.isEmpty && cycles < maxCycles) {
                if (dut.io.commit_valid.peek().litToBoolean) commits += 1
                if (dut.io.redirect_valid.peek().litToBoolean) redirects += 1
                if (dut.io.tohost_valid.peek().litToBoolean) {
                    result = Some(dut.io.tohost_value.peek().litValue)
                }
                dut.clock.step(1)
                cycles += 1
            }
        }
        RunStats(cycles, commits, redirects, result)
    }

    "PERF: branch predictor effectiveness (sw/tests/bench.c)" in {
        val s = runWithStats("sw/build/bench.hex", maxCycles = 5000000)
        println(s"[PERF] bench.c (primes<500)  ${s.summary}")
        s.result shouldBe Some(BigInt(95)) // pi(500) = 95; must stay correct whatever the perf
    }

    "compiled C: sub-word stores preserve their neighbouring bytes (sw/tests/bytes.c)" in {
        // word = 0xAABBCCDD; word.byte[1] = 0x11; word.byte[3] = 0xEE  =>  0xEEBB11DD
        // Before the byte-write-enable fix the LSU spliced sub-word stores over a ZERO base
        // word, so the surrounding bytes were destroyed and this returned 0xEE001100.
        runProgram("sw/build/bytes.hex") shouldBe BigInt(0xEEBB11DDL)
    }

    "compiled C: RV32IM arithmetic, mul/div/rem and a loop (sw/tests/arith.c)" in {
        runProgram("sw/build/arith.hex") shouldBe BigInt(0x600D)
    }

    "compiled C: hardware timer advances and DG_SleepMs terminates (sw/tests/timer.c)" in {
        // The timer at 0x08000004 is what Doom's DG_SleepMs busy-waits on. If it never advanced,
        // that loop would spin forever and Doom would hang on its first frame. This runs the
        // exact same wait pattern, so a pass here means the loop provably terminates.
        // The test also writes to the debug MMIO ports, so Memory.scala's printf emits its
        // output to the simulation console -- the channel we will watch Doom boot on.
        runProgram("sw/build/timer.hex", maxCycles = 2000000) shouldBe BigInt(0x900D)
    }
}
