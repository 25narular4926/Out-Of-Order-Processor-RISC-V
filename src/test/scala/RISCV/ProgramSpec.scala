package RISCV

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import chisel3.simulator.HasSimulator
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

    // Build the Verilator simulator for SPEED, not for compile time.
    //
    // ChiselSim's default builds the generated C++ with -Os (optimise for SIZE) and enables
    // assertions. For a short unit test that is the right trade -- you spend more time compiling
    // the simulator than running it. For a long-running workload it is exactly backwards: it caps
    // the simulation at a few thousand cycles/sec, which is the difference between Doom booting
    // in an afternoon and Doom booting in a fortnight.
    /**
     * Simulation settings with randomization DISABLED. Required whenever RAM is preloaded via
     * $readmemh (OoOParams.memInitFile): ChiselSim's default RANDOMIZE_MEM_INIT runs in an
     * initial block that overwrites the preloaded contents, so the image must not be randomized.
     */
    def noRandom[A <: chisel3.RawModule]: chisel3.simulator.Settings[A] =
        chisel3.simulator.Settings.defaultRaw[A].copy(
          randomization = chisel3.simulator.Randomization.uninitialized
        )

    implicit val simulator: HasSimulator = HasSimulator.simulators.verilator(
      verilatorSettings = svsim.verilator.Backend.CompilationSettings(
        disabledWarnings = Seq("WIDTH", "STMTDLY"),
        disableFatalExitOnWarnings = true
      ),
      compilationSettings = svsim.CommonCompilationSettings(
        optimizationStyle = svsim.CommonCompilationSettings.OptimizationStyle.OptimizeForSimulationSpeed
      )
    )

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
    case class RunStats(
        cycles:    Long,
        commits:   Long,
        redirects: Long,
        result:    Option[BigInt],
        runSecs:   Double // WALL TIME OF THE STEPPING LOOP ONLY -- excludes Verilator compile
    ) {
        def ipc: Double = if (cycles == 0) 0.0 else commits.toDouble / cycles
        def mispredPerCommit: Double = if (commits == 0) 0.0 else redirects.toDouble / commits
        def cyclesPerSec: Double = if (runSecs <= 0) 0.0 else cycles / runSecs
        def instrPerSec: Double = if (runSecs <= 0) 0.0 else commits / runSecs
        def summary: String =
            f"cycles=$cycles%-7d commits=$commits%-6d redirects=$redirects%-5d " +
                f"IPC=$ipc%.3f  mispred/instr=$mispredPerCommit%.4f"
    }

    /**
     * Run a program, stepping the simulator in LARGE BATCHES.
     *
     * Every peek()/step() call is a round-trip across the JVM<->Verilator boundary and costs on
     * the order of a millisecond. Polling a few signals once per cycle therefore caps the whole
     * simulation at roughly 500 cycles/sec -- which is a measurement of IPC latency, not of the
     * RTL. It made a Doom-sized workload look like a 13-day job.
     *
     * So: the harness counts cycles/commits/redirects in HARDWARE and latches tohost, and we step
     * `batch` cycles at a time and only look at the DUT between batches. Same numbers, orders of
     * magnitude faster.
     */
    def runWithStats(hexPath: String, maxCycles: Long = 500000, batch: Int = 5000): RunStats = {
        val f = new java.io.File(hexPath)
        if (!f.exists) cancel(s"$hexPath not found -- run sw/build.sh first. Skipping.")
        val words = Source.fromFile(f).getLines().map(_.trim).filter(_.nonEmpty).toList

        var cycles = 0L; var commits = 0L; var redirects = 0L
        var runSecs = 0.0
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

            // Time ONLY the stepping loop. `simulate` has already elaborated the design and
            // Verilator-compiled the simulator by this point (tens of seconds); folding that
            // into the rate would report compile time as simulation speed.
            val t0 = System.nanoTime()

            var done = false
            var stepped = 0L
            while (!done && stepped < maxCycles) {
                val n = math.min(batch.toLong, maxCycles - stepped).toInt
                dut.clock.step(n) // <-- ONE round-trip for n cycles
                stepped += n
                if (dut.io.tohost_seen.peek().litToBoolean) done = true
            }

            runSecs = (System.nanoTime() - t0) / 1e9

            cycles = dut.io.perf_cycles.peek().litValue.toLong
            commits = dut.io.perf_commits.peek().litValue.toLong
            redirects = dut.io.perf_redirects.peek().litValue.toLong
            if (dut.io.tohost_seen.peek().litToBoolean) {
                result = Some(dut.io.tohost_data.peek().litValue)
            }
        }
        RunStats(cycles, commits, redirects, result, runSecs)
    }

    "PERF: branch predictor effectiveness (sw/tests/bench.c)" in {
        val s = runWithStats("sw/build/bench.hex", maxCycles = 5000000)

        println(s"[PERF] bench.c (primes<500)  ${s.summary}")

        // SIMULATION THROUGHPUT (stepping only -- Verilator compile time excluded). This is the
        // number that decides whether Doom is tractable: Doom needs on the order of 10^8
        // instructions to reach its first rendered frame, so this rate sets the whole schedule.
        println(f"[PERF] verilator: ${s.cyclesPerSec}%,.0f cycles/sec, ${s.instrPerSec}%,.0f instr/sec " +
            f"(${s.runSecs}%.2f s to run ${s.cycles}%,d cycles)")
        if (s.instrPerSec > 0) {
            println(f"[PERF] => 100M-instruction Doom boot ~= ${100e6 / s.instrPerSec / 3600}%,.1f hours")
        }

        s.result shouldBe Some(BigInt(95)) // pi(500) = 95; must stay correct whatever the perf
    }

    "PRELOAD: $readmemh initialises RAM without using the flash port" in {
        // Doom-scale images cannot be streamed in through the 1-word-per-cycle flash port: it runs
        // at ~800 words/sec (each word is a JVM<->Verilator round-trip), so a 24 MB image would
        // take ~2 HOURS just to load. Everything depends on preloading RAM at elaboration via
        // $readmemh instead -- so prove the mechanism works on a small program first.
        //
        // CRUCIAL: randomization must be disabled. ChiselSim compiles with RANDOMIZE_MEM_INIT by
        // default, and that randomization runs in an initial block that CLOBBERS the $readmemh
        // preload. `Randomization.uninitialized` turns it off so the preload survives.
        val hex = new java.io.File("sw/build/bench.hex")
        if (!hex.exists) cancel("sw/build/bench.hex not found -- run sw/build.sh. Skipping.")

        var result: Option[BigInt] = None
        simulate(
          new SocHarness(OoOParams(memInitFile = hex.getAbsolutePath)),
          settings = noRandom[SocHarness]
        ) { dut =>
            dut.io.btns.poke(0.U)
            dut.io.flash.poke(false.B) // <-- never used
            dut.io.flash_address.poke(0.U)
            dut.io.flash_value.poke(0.U)
            dut.io.execute.poke(true.B) // straight to running: RAM is already populated

            var stepped = 0
            while (result.isEmpty && stepped < 500000) {
                dut.clock.step(5000)
                stepped += 5000
                if (dut.io.tohost_seen.peek().litToBoolean) {
                    result = Some(dut.io.tohost_data.peek().litValue)
                }
            }
        }
        withClue("preloaded RAM did not produce the right answer: ") {
            result shouldBe Some(BigInt(95)) // same pi(500) = 95 as the flashed run
        }
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
