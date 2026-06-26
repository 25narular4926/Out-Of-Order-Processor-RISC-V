package RISCV

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.must.Matchers
import scala.io.Source

/**
 * Whole-core smoke test: flash `test_basic.hex` into instruction memory, release the CPU, and
 * run it for a number of cycles. This exercises the full SoC (Main -> Core -> every OoO
 * subsystem) in simulation end to end.
 *
 * It is currently a SMOKE test (no architectural assertions) because the OoO Core does not yet
 * expose a committed-register / commit-trace debug port. Turning this into a SELF-CHECKING test
 * -- tandem-comparing committed writes against a golden RV32IM reference model -- is milestone 1
 * of the bring-up plan in the README. (See also: add a debug commit-trace port to ReorderBuffer.)
 *
 * NOTE: ChiselSim uses a Verilator backend, which requires a Unix-like environment (it shells
 * out to `which verilator`). It runs in CI (Ubuntu + Verilator); it will NOT run on a native
 * Windows shell.
 */
class MainSpec extends AnyFreeSpec with Matchers with ChiselSim {
    "Main runs test_basic.hex without crashing (smoke test)" in {
        simulate(new Main()) { dut =>
            val path = getClass.getResource("/test_basic.hex").getPath
            val lines = Source.fromFile(path).getLines().toList

            // ---- flash phase: stream each word into instruction memory ----
            dut.io.execute.poke(false.B)
            dut.io.flash.poke(true.B)
            lines.zipWithIndex.foreach { case (line, index) =>
                val value = java.lang.Long.parseLong(line.trim, 16)
                dut.io.flash_address.poke(index.U)
                dut.io.flash_value.poke(value.U)
                dut.clock.step(1)
            }
            dut.io.flash.poke(false.B)

            // ---- run phase ----
            dut.io.btns.poke(0.U)
            dut.io.execute.poke(true.B)
            dut.clock.step(64)
        }
    }
}
