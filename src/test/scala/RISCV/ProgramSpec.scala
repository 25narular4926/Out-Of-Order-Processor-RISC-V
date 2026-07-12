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

    /** Core + Memory + flash port, with the MMIO write port brought out as a tohost mailbox. */
    class SocHarness(p: OoOParams = OoOParams()) extends Module {
        val io = IO(new Bundle {
            val execute       = Input(Bool())
            val flash         = Input(Bool())
            val flash_address = Input(UInt(32.W))
            val flash_value   = Input(UInt(32.W))
            val btns          = Input(UInt(4.W))

            // tohost mailbox (Memory's MMIO write port)
            val tohost_valid = Output(Bool())
            val tohost_addr  = Output(UInt(32.W))
            val tohost_value = Output(UInt(32.W))

            // commit trace (what the core actually retires, in program order)
            val commit_valid    = Output(Bool())
            val commit_pc       = Output(UInt(32.W))
            val redirect_valid  = Output(Bool())
            val redirect_target = Output(UInt(32.W))
            val dispatch_valid  = Output(Bool())
            val dispatch_pc     = Output(UInt(32.W))
            val rob_empty       = Output(Bool())
            val rob_full        = Output(Bool())
            val iqmem_valid     = Output(Bool())
            val iqmem_ready     = Output(Bool())
            val iqmem_pc        = Output(UInt(32.W))
            val iqmem_isStore   = Output(Bool())
            val iqmem_isLoad    = Output(Bool())
            val lsu_wb_valid    = Output(Bool())
            val lsu_wb_robIdx   = Output(UInt(32.W))
            val rob_head        = Output(UInt(32.W))
        })

        val memory = Module(new Memory(p))
        val core   = Module(new Core(p))

        core.io.execute := io.execute
        memory.io.btns  := io.btns

        // instruction fetch (byte address -> word address)
        memory.io.read_1        := true.B
        memory.io.write_1       := false.B
        memory.io.write_value_1 := 0.U
        memory.io.address_1     := core.io.program_memory_adress / 4.U
        core.io.program_memory_value := memory.io.read_value_1

        // data port
        memory.io.address_2     := core.io.memory_address
        memory.io.read_2        := core.io.memory_read
        memory.io.write_2       := core.io.memory_write
        memory.io.write_value_2 := core.io.memory_write_value
        memory.io.write_mask_2  := core.io.memory_write_mask
        core.io.memory_read_value := memory.io.read_value_2

        // flash while the CPU is held
        when(!io.execute) {
            when(io.flash) {
                memory.io.read_1        := false.B
                memory.io.write_1       := true.B
                memory.io.address_1     := io.flash_address
                memory.io.write_value_1 := io.flash_value
            }
        }

        io.tohost_valid := memory.io.write_vga
        io.tohost_addr  := memory.io.address_vga
        io.tohost_value := memory.io.write_value_vga

        io.commit_valid    := core.io.dbg_commit_valid
        io.commit_pc       := core.io.dbg_commit_pc
        io.redirect_valid  := core.io.dbg_redirect_valid
        io.redirect_target := core.io.dbg_redirect_target
        io.dispatch_valid  := core.io.dbg_dispatch_valid
        io.dispatch_pc     := core.io.dbg_dispatch_pc
        io.rob_empty       := core.io.dbg_rob_empty
        io.rob_full        := core.io.dbg_rob_full
        io.iqmem_valid     := core.io.dbg_iqmem_valid
        io.iqmem_ready     := core.io.dbg_iqmem_ready
        io.iqmem_pc        := core.io.dbg_iqmem_pc
        io.iqmem_isStore   := core.io.dbg_iqmem_isStore
        io.iqmem_isLoad    := core.io.dbg_iqmem_isLoad
        io.lsu_wb_valid    := core.io.dbg_lsu_wb_valid
        io.lsu_wb_robIdx   := core.io.dbg_lsu_wb_robIdx
        io.rob_head        := core.io.dbg_rob_head
    }

    /** Flash an image and print a cycle-by-cycle trace of what the core dispatches/retires. */
    def traceProgram(hexPath: String, cycles: Int): Unit = {
        val words = Source.fromFile(hexPath).getLines().map(_.trim).filter(_.nonEmpty).toList
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

            def hx(v: BigInt) = f"0x$v%04x"
            println("cyc | commit  | head | MEM-issue (iq->lsu)          | lsu.wb  | tohost")
            for (c <- 0 until cycles) {
                val cv   = dut.io.commit_valid.peek().litToBoolean
                val cpc  = dut.io.commit_pc.peek().litValue
                val head = dut.io.rob_head.peek().litValue
                val mv   = dut.io.iqmem_valid.peek().litToBoolean
                val mr   = dut.io.iqmem_ready.peek().litToBoolean
                val mpc  = dut.io.iqmem_pc.peek().litValue
                val mst  = dut.io.iqmem_isStore.peek().litToBoolean
                val mld  = dut.io.iqmem_isLoad.peek().litToBoolean
                val wv   = dut.io.lsu_wb_valid.peek().litToBoolean
                val wrob = dut.io.lsu_wb_robIdx.peek().litValue
                val tv   = dut.io.tohost_valid.peek().litToBoolean
                val tval = dut.io.tohost_value.peek().litValue

                val memStr =
                    if (mv) f"V${if (mr) "/R" else "/-"} pc=${hx(mpc)}%-6s ${if (mst) "ST" else if (mld) "LD" else "??"}"
                    else    "                             "
                println(f"$c%3d | ${if (cv) hx(cpc) else "   -  "}%7s | $head%4d | $memStr%-27s | " +
                        f"${if (wv) f"rob$wrob" else "  -  "}%7s | ${if (tv) hx(tval) else "-"}%s")
                dut.clock.step(1)
            }
        }
    }

    "TRACE bytes.c" in { traceProgram("sw/build/bytes.hex", 80) }

    /**
     * Flash `hexPath` into the core, run it, and return the first value the program writes to
     * the tohost mailbox (or fail after `maxCycles`).
     */
    def runProgram(hexPath: String, maxCycles: Int = 20000): BigInt = {
        val words = Source.fromFile(hexPath).getLines().map(_.trim).filter(_.nonEmpty).toList
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

    "compiled C: sub-word stores preserve their neighbouring bytes (sw/tests/bytes.c)" in {
        // word = 0xAABBCCDD; word.byte[1] = 0x11; word.byte[3] = 0xEE  =>  0xEEBB11DD
        // Before the byte-write-enable fix the LSU spliced sub-word stores over a ZERO base
        // word, so the surrounding bytes were destroyed and this returned 0xEE001100.
        runProgram("sw/build/bytes.hex") shouldBe BigInt(0xEEBB11DDL)
    }

    "compiled C: RV32IM arithmetic, mul/div/rem and a loop (sw/tests/arith.c)" in {
        runProgram("sw/build/arith.hex") shouldBe BigInt(0x600D)
    }
}
