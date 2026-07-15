package RISCV

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import chisel3.simulator.HasSimulator
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.should.Matchers
import java.io.File

/**
 * Run DOOM on the out-of-order core.
 *
 * The image (doomgeneric + the RISC-V port + the FreeDM WAD) is built by
 * `sw/doom/doomgeneric/build_doom.sh` into a ~22 MB flat binary and its $readmemh hex. We preload
 * that hex into a 64 MB RAM (no flashing -- see ProgramSpec) and let the CPU run.
 *
 * Observability: Doom's own console output (banner, WAD load, zone init, ...) is emitted a
 * character at a time to the debug MMIO port (0x70000000), which Memory.scala turns into a Chisel
 * `printf`. So the boot log streams live to the simulation stdout as Doom runs -- that is how we
 * watch it come up.
 *
 * This is heavy (64 MB memory, ~10^8 instructions to first frame) so it is NOT part of the normal
 * suite: it only runs when the image exists AND doom is explicitly requested.
 */
class DoomSpec extends AnyFreeSpec with Matchers with ChiselSim {

    // Build the simulator for speed, exactly like ProgramSpec.
    implicit val simulator: HasSimulator = HasSimulator.simulators.verilator(
      verilatorSettings = svsim.verilator.Backend.CompilationSettings(
        disabledWarnings = Seq("WIDTH", "STMTDLY"),
        disableFatalExitOnWarnings = true
      ),
      compilationSettings = svsim.CommonCompilationSettings(
        optimizationStyle = svsim.CommonCompilationSettings.OptimizationStyle.OptimizeForSimulationSpeed
      )
    )

    // 64 MB RAM: flash 0..40 MB (code + 21 MB WAD + data image), ram 40..64 MB (data/bss/heap/stack).
    // divK / randomize overridable for A/B isolation of the M_LoadDefaults hang.
    val divK = sys.props.getOrElse("doom.divK", "4").toInt
    val randomize = sys.props.getOrElse("doom.random", "false").toBoolean

    // clockHz sets how fast the hardware timer advances PER SIMULATED CYCLE (1 us per clockHz/1e6
    // cycles). Doom paces itself off that timer: at the real 125 MHz it holds the title screen for
    // ~11 s of game-time = ~1.4 BILLION cycles before the attract-mode demo starts, and each frame
    // waits ~1/35 s = ~3.5M cycles -- so a capture just gets static title frames.
    //
    // Lowering clockHz makes game-time pass faster per cycle, so Doom stops waiting between frames
    // (it becomes render-bound) and reaches the demo in tens of millions of cycles instead of
    // billions. This does NOT change what the core computes -- only how the sleep/timing loop
    // perceives elapsed time. 1 MHz (the minimum) is ideal for capturing an animation quickly.
    val clockHz = sys.props.getOrElse("doom.clockHz", "125000000").toInt
    val doomParams = OoOParams(memWords = 16 * 1024 * 1024, divBitsPerCycle = divK, clockHz = clockHz)

    def noRandom[A <: chisel3.RawModule]: chisel3.simulator.Settings[A] =
        chisel3.simulator.Settings.defaultRaw[A].copy(
          randomization = chisel3.simulator.Randomization.uninitialized
        )

    "DOOM boots on the OoO core" in {
        val hex = new File("sw/doom/doomgeneric/build/doom.hex")
        if (!hex.exists) {
            cancel("doom.hex not built -- run sw/doom/doomgeneric/build_doom.sh first. Skipping.")
        }

        // How long to let it run. Boot to first frame is ~10^8 instructions; at ~60k instr/sec
        // that is tens of minutes. Overridable via -Ddoom.cycles=<n> for a quick smoke vs a full
        // boot. Default: a few million cycles -- enough to see the banner and WAD-load messages.
        val maxCycles = sys.props.getOrElse("doom.cycles", "8000000").toLong
        val batch = 200000

        val settings =
            if (randomize) chisel3.simulator.Settings.defaultRaw[SocHarness]
            else noRandom[SocHarness]
        println(s"[DOOM] config: divBitsPerCycle=$divK  randomize=$randomize")
        simulate(new SocHarness(doomParams.copy(memInitFile = hex.getAbsolutePath)),
                 settings = settings) { dut =>
            dut.io.btns.poke(0.U)
            dut.io.flash.poke(false.B)
            dut.io.flash_address.poke(0.U)
            dut.io.flash_value.poke(0.U)
            dut.io.fb_read_addr.poke(0.U)
            dut.io.pc_range_reset.poke(false.B)
            dut.io.execute.poke(true.B)

            println(s"[DOOM] running up to $maxCycles cycles; console output follows:")
            println("----------------------------------------------------------------")

            // Per-WINDOW stall attribution. The hardware counters are cumulative from cycle 0, so
            // to see how a phase behaves (init vs R_Init vs rendering) we snapshot every ~20M
            // cycles and diff. This is what isolates the rendering phase from the init phase.
            val windowCycles = 20000000L
            var winCyc0 = BigInt(0); var winCom0 = BigInt(0)
            var winDiv0 = BigInt(0); var winHs0 = BigInt(0); var winRed0 = BigInt(0)
            var nextWindow = windowCycles

            // Multi-frame capture. When >0, dump a numbered PPM roughly every `frameStride`
            // completed frames, up to `captureFrames` images -- a sequence we stitch into an
            // animation offline. Uses a fine step while capturing so we don't overshoot frames.
            val captureFrames = sys.props.getOrElse("doom.frames", "0").toInt
            val frameStride   = sys.props.getOrElse("doom.frameStride", "2").toInt
            val fbWords       = doomParams.fbWidth * doomParams.fbHeight
            val framesDir     = "sw/doom/doomgeneric/build/frames"
            if (captureFrames > 0) new java.io.File(framesDir).mkdirs()
            val capBatch      = if (captureFrames > 0) 20000 else batch
            var lastFrameCount = BigInt(0)
            var framesDumped   = 0

            var stepped = 0L
            var done = false
            var lastFbWrites = BigInt(0)
            while (!done && stepped < maxCycles) {
                dut.clock.step(capBatch)
                stepped += capBatch
                val fbw = dut.io.fb_writes.peek().litValue
                if (fbw != lastFbWrites) {
                    println(f"\n[DOOM] framebuffer writes so far: $fbw%,d (${fbw / fbWords} full frames)")
                    lastFbWrites = fbw
                }
                // capture a sequence of frames as they complete
                if (captureFrames > 0 && framesDumped < captureFrames) {
                    val frameCount = fbw / fbWords
                    if (frameCount >= lastFrameCount + frameStride) {
                        val path = f"$framesDir/frame_$framesDumped%04d.ppm"
                        dumpFramePPM(dut, path)
                        framesDumped += 1
                        lastFrameCount = frameCount
                        println(s"[DOOM] captured $path (frame $frameCount)")
                    }
                }
                if (stepped >= nextWindow) {
                    val c = dut.io.perf_cycles.peek().litValue
                    val m = dut.io.perf_commits.peek().litValue
                    val d = dut.io.perf_divbusy.peek().litValue
                    val h = dut.io.perf_headstall.peek().litValue
                    val r = dut.io.perf_redirects.peek().litValue
                    val dc = (c - winCyc0).toDouble; val dm = (m - winCom0).toDouble
                    val dd = (d - winDiv0).toDouble; val dh = (h - winHs0).toDouble
                    val dr = (r - winRed0).toDouble
                    val pmin = dut.io.pc_min.peek().litValue
                    val pmax = dut.io.pc_max.peek().litValue
                    val plast = dut.io.pc_last.peek().litValue
                    if (dc > 0) println(f"\n[DOOM][window @ ${c.toLong}%,d cyc] IPC=${dm/dc}%.3f  " +
                        f"div-busy=${100*dd/dc}%.1f%%  head-stall=${100*dh/dc}%.1f%%  mispred/instr=${dr/math.max(1,dm)}%.4f  " +
                        f"PC=[0x${pmin.toString(16)}..0x${pmax.toString(16)}] last=0x${plast.toString(16)}")
                    // reset the PC range for the next window
                    dut.io.pc_range_reset.poke(true.B); dut.clock.step(1); dut.io.pc_range_reset.poke(false.B)
                    winCyc0 = c; winCom0 = m; winDiv0 = d; winHs0 = h; winRed0 = r
                    nextWindow += windowCycles
                }
                if (dut.io.tohost_seen.peek().litToBoolean) done = true
            }
            println("\n----------------------------------------------------------------")
            val commits = dut.io.perf_commits.peek().litValue
            val cycles = dut.io.perf_cycles.peek().litValue
            val fbw = dut.io.fb_writes.peek().litValue
            val redirects = dut.io.perf_redirects.peek().litValue
            val divbusy = dut.io.perf_divbusy.peek().litValue
            val headstall = dut.io.perf_headstall.peek().litValue
            val headstalldiv = dut.io.perf_headstalldiv.peek().litValue
            def pct(n: BigInt) = if (cycles == 0) 0.0 else 100.0 * n.toDouble / cycles.toDouble
            val ipc = if (cycles == 0) 0.0 else commits.toDouble / cycles.toDouble
            println(f"[DOOM] ran $cycles%,d cycles, retired $commits%,d instructions, $fbw%,d fb writes")
            println(f"[DOOM] IPC=$ipc%.3f  mispred/instr=${redirects.toDouble/math.max(1,commits.toDouble)}%.4f")
            println(f"[DOOM] STALLS: divider busy ${pct(divbusy)}%.1f%% | head stalled ${pct(headstall)}%.1f%% | head stalled ON divider ${pct(headstalldiv)}%.1f%%")

            // If any pixels were rendered, scan out the capture RAM and dump a PPM image.
            if (fbw > 0) dumpFramePPM(dut, "sw/doom/doomgeneric/build/frame.ppm")
        }
    }

    /**
     * Scan the framebuffer capture RAM and write a binary PPM (P6). The capture RAM is a
     * combinational-read Mem, so each pixel is just poke-address + peek -- NO clock step. That
     * means the scan does not advance the CPU, so Doom cannot overwrite the frame mid-scan and we
     * need not pause it. The port's DG_DrawFrame stores each pixel as 0x0RGB (RGB444); we expand
     * each 4-bit channel to 8 bits.
     */
    private def dumpFramePPM(dut: SocHarness, path: String): Unit = {
        val w = doomParams.fbWidth
        val h = doomParams.fbHeight
        val bytes = new Array[Byte](w * h * 3)
        var i = 0
        while (i < w * h) {
            dut.io.fb_read_addr.poke(i.U)
            val px = dut.io.fb_read_data.peek().litValue.toInt // combinational read; no step
            val r4 = (px >> 8) & 0xf
            val g4 = (px >> 4) & 0xf
            val b4 = px & 0xf
            bytes(i * 3 + 0) = ((r4 << 4) | r4).toByte
            bytes(i * 3 + 1) = ((g4 << 4) | g4).toByte
            bytes(i * 3 + 2) = ((b4 << 4) | b4).toByte
            i += 1
        }
        val f = new java.io.FileOutputStream(path)
        f.write(s"P6\n$w $h\n255\n".getBytes("US-ASCII"))
        f.write(bytes)
        f.close()
        println(s"[DOOM] wrote frame to $path ($w x $h)")
    }
}
