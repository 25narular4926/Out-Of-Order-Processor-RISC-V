package RISCV

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.should.Matchers
import scala.io.Source
import java.io.File

/**
 * RV32IM ISA compliance: the official riscv-tests suites (rv32ui + rv32um) run on the core.
 *
 * The tests are built by `sw/isa/build_isa.sh` against a custom environment (sw/isa/riscv_test.h)
 * because the stock riscv-tests environment boots through machine-mode CSRs and exits via `ecall`
 * -- and this core implements no CSRs, no ecall, and no traps. The TEST BODIES are untouched;
 * only the boot/exit scaffolding is replaced. Each test is _start at address 0 and exits by
 * storing to the tohost mailbox.
 *
 * RESULT PROTOCOL (riscv-tests convention):
 *   1                      -> pass
 *   (TESTNUM << 1) | 1     -> failed sub-test number TESTNUM
 *   no write               -> the core hung
 *
 * Two tests are intentionally not built (see build_isa.sh): `fence_i` (needs FENCE.I and
 * self-modifying code) and `ma_data` (deliberately misaligned accesses -- the LSU has no
 * misaligned support, and RISC-V permits trapping instead, which we cannot do).
 */
class IsaSpec extends AnyFreeSpec with Matchers with ChiselSim {

    private val hexDir = new File("sw/isa/build")

    /** Flash an ISA test, run it, and return the tohost word (None if the core hung). */
    private def runIsaTest(hexPath: File, maxCycles: Int = 200000): Option[BigInt] = {
        val words = Source.fromFile(hexPath).getLines().map(_.trim).filter(_.nonEmpty).toList
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
            var cycle = 0
            while (result.isEmpty && cycle < maxCycles) {
                if (dut.io.tohost_valid.peek().litToBoolean) {
                    result = Some(dut.io.tohost_value.peek().litValue)
                }
                dut.clock.step(1)
                cycle += 1
            }
        }
        result
    }

    /** Turn a tohost word into a human-readable verdict. */
    private def verdict(r: Option[BigInt]): String = r match {
        case None                    => "HUNG (no tohost write -- core never terminated)"
        case Some(v) if v == 1       => "pass"
        case Some(v) if (v & 1) == 1 => s"FAILED at sub-test #${v >> 1}"
        case Some(v)                 => s"FAILED with unexpected tohost value 0x${v.toString(16)}"
    }

    private val hexFiles =
        Option(hexDir.listFiles((_, n) => n.endsWith(".hex")))
            .getOrElse(Array.empty[File])
            .sortBy(_.getName)

    // The ISA images are generated: they need a RISC-V cross-compiler AND a checkout of
    // riscv-tests (7 MB), so neither the images nor the test sources live in this repo. Building
    // them is `sw/isa/build_isa.sh`.
    //
    // When they are absent we CANCEL rather than fail. A missing toolchain is not an RTL defect,
    // and a plain `sbt test` on a fresh clone (or in CI, which has no cross-compiler) must not
    // report a spurious failure. When they ARE present -- the normal local flow, and any CI job
    // that builds them -- every test runs for real.
    if (hexFiles.isEmpty) {
        "riscv-tests ISA suite (not built -- skipping)" in {
            cancel(
              s"No .hex images in ${hexDir.getPath}. Build them with `sw/isa/build_isa.sh` " +
                "(needs riscv64-unknown-elf-gcc and a riscv-tests checkout). Skipping ISA compliance."
            )
        }
    } else {
        hexFiles.foreach { f =>
            val name = f.getName.stripSuffix(".hex") // e.g. rv32ui-add
            s"$name" in {
                val r = runIsaTest(f)
                withClue(s"$name -> ${verdict(r)}: ") { r shouldBe Some(BigInt(1)) }
            }
        }
    }
}
