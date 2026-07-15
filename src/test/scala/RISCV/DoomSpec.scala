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
    val doomParams = OoOParams(memWords = 16 * 1024 * 1024)

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

        simulate(new SocHarness(doomParams.copy(memInitFile = hex.getAbsolutePath)),
                 settings = noRandom[SocHarness]) { dut =>
            dut.io.btns.poke(0.U)
            dut.io.flash.poke(false.B)
            dut.io.flash_address.poke(0.U)
            dut.io.flash_value.poke(0.U)
            dut.io.fb_read_addr.poke(0.U)
            dut.io.execute.poke(true.B)

            println(s"[DOOM] running up to $maxCycles cycles; console output follows:")
            println("----------------------------------------------------------------")
            var stepped = 0L
            var done = false
            var lastFbWrites = BigInt(0)
            while (!done && stepped < maxCycles) {
                dut.clock.step(batch)
                stepped += batch
                val fbw = dut.io.fb_writes.peek().litValue
                if (fbw != lastFbWrites) {
                    println(f"\n[DOOM] framebuffer writes so far: $fbw%,d (${fbw / (doomParams.fbWidth * doomParams.fbHeight)} full frames)")
                    lastFbWrites = fbw
                }
                if (dut.io.tohost_seen.peek().litToBoolean) done = true
            }
            println("\n----------------------------------------------------------------")
            val commits = dut.io.perf_commits.peek().litValue
            val cycles = dut.io.perf_cycles.peek().litValue
            val fbw = dut.io.fb_writes.peek().litValue
            println(f"[DOOM] ran $cycles%,d cycles, retired $commits%,d instructions, $fbw%,d fb writes")

            // If any pixels were rendered, scan out the capture RAM and dump a PPM image.
            if (fbw > 0) dumpFramePPM(dut, "sw/doom/doomgeneric/build/frame.ppm")
        }
    }

    /**
     * Scan the framebuffer capture RAM (one peek per pixel) and write a binary PPM (P6).
     * The port's DG_DrawFrame stores each pixel as 0x0RGB (4-bit channels, RGB444). We expand
     * each 4-bit channel to 8 bits for a viewable image.
     */
    private def dumpFramePPM(dut: SocHarness, path: String): Unit = {
        val w = doomParams.fbWidth
        val h = doomParams.fbHeight
        val bytes = new Array[Byte](w * h * 3)
        var i = 0
        while (i < w * h) {
            dut.io.fb_read_addr.poke(i.U)
            dut.clock.step(1) // SyncReadMem: data valid next cycle
            val px = dut.io.fb_read_data.peek().litValue.toInt
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
