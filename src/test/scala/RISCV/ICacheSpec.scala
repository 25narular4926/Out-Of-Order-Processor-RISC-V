package RISCV

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.should.Matchers
import java.io.{File, PrintWriter}

/**
 * Wire ICache -> ModeledRam and surface the CPU-side handshake + counters for the testbench.
 * ModeledRam is given 2 masters (master 1 tied off) to avoid a 0-width RRArbiter.chosen at n=1.
 */
class ICacheTestHarness(p: OoOParams, nSets: Int, lineWords: Int, readLatency: Int, memInit: String)
    extends Module {
    val io = IO(new Bundle {
        val reqAddr   = Input(UInt(p.xlen.W))
        val reqValid  = Input(Bool())
        val reqReady  = Output(Bool())
        val respValid = Output(Bool())
        val respData  = Output(UInt(p.xlen.W))
        val hits      = Output(UInt(64.W))
        val misses    = Output(UInt(64.W))
    })

    val cache = Module(new ICache(p, nSets, lineWords))
    val ram   = Module(new ModeledRam(p, nMasters = 2, readLatency = readLatency, memInitFile = memInit))

    cache.io.mem <> ram.io.masters(0)
    // tie off the unused second master
    ram.io.masters(1).req.valid := false.B
    ram.io.masters(1).req.bits  := 0.U.asTypeOf(new MemReq(p))

    cache.io.req.valid := io.reqValid
    cache.io.req.bits  := io.reqAddr
    io.reqReady   := cache.io.req.ready
    io.respValid  := cache.io.resp.valid
    io.respData   := cache.io.resp.bits
    io.hits       := cache.io.hits
    io.misses     := cache.io.misses
}

class ICacheSpec extends AnyFreeSpec with Matchers with ChiselSim {

    // $readmemh preload requires randomization disabled (else RANDOMIZE_MEM_INIT clobbers it)
    def noRandom[A <: chisel3.RawModule]: chisel3.simulator.Settings[A] =
        chisel3.simulator.Settings.defaultRaw[A].copy(
          randomization = chisel3.simulator.Randomization.uninitialized
        )

    "I-cache: cold miss refills a line, subsequent words in the line hit, data is correct" in {
        val p         = OoOParams(memWords = 4096)
        val nSets     = 4
        val lineWords = 4
        val lat       = 8

        // known memory pattern: word i = 0xCAFE0000 + i
        val hex = new File("target/icache_test.hex")
        hex.getParentFile.mkdirs()
        val pw = new PrintWriter(hex)
        for (i <- 0 until 4096) pw.println(f"${0xCAFE0000L + i}%08x")
        pw.close()

        simulate(new ICacheTestHarness(p, nSets, lineWords, lat, hex.getAbsolutePath),
                 settings = noRandom) { dut =>

            // issue one address, wait for the (variable-latency) response, return (data, cycles-waited)
            def access(addr: Long): (BigInt, Int) = {
                dut.io.reqAddr.poke(addr.U)
                dut.io.reqValid.poke(true.B)
                var g = 0
                while (!dut.io.reqReady.peek().litToBoolean && g < 500) { dut.clock.step(1); g += 1 }
                dut.clock.step(1)              // consummate the fire
                dut.io.reqValid.poke(false.B)
                var c = 0
                while (!dut.io.respValid.peek().litToBoolean && c < 2000) { dut.clock.step(1); c += 1 }
                val d = dut.io.respValid.peek().litToBoolean match {
                    case true  => dut.io.respData.peek().litValue
                    case false => BigInt(-1)
                }
                dut.clock.step(1)              // consume the response pulse
                (d, c)
            }

            dut.io.reqValid.poke(false.B)
            dut.clock.step(3)

            // (1) cold access to word 0 -> MISS (refills line 0: words 0..3)
            val (d0, c0) = access(0)
            withClue(s"cold word0: data=0x${d0.toString(16)} cycles=$c0: ") {
                d0 shouldBe BigInt(0xCAFE0000L)
            }

            // (2) word 1 is in the just-filled line -> HIT, and fast
            val (d1, c1) = access(1)
            withClue(s"word1 (same line): data=0x${d1.toString(16)} cycles=$c1: ") {
                d1 shouldBe BigInt(0xCAFE0001L)
            }

            // (3) word 3 also in the line -> HIT
            val (d3, _) = access(3)
            d3 shouldBe BigInt(0xCAFE0003L)

            // a miss must take materially longer than a hit
            assert(c0 > c1, s"expected miss ($c0 cyc) slower than hit ($c1 cyc)")

            // counters: exactly one line-miss so far, at least two hits
            dut.io.misses.peek().litValue shouldBe BigInt(1)
            dut.io.hits.peek().litValue should be >= BigInt(2)

            // (4) a different line -> second MISS
            val (dFar, cFar) = access(lineWords.toLong)   // word 4 = line 1, offset 0
            dFar shouldBe BigInt(0xCAFE0000L + lineWords)
            assert(cFar > c1, s"expected new-line miss ($cFar) slower than a hit ($c1)")
            dut.io.misses.peek().litValue shouldBe BigInt(2)
        }
    }
}
