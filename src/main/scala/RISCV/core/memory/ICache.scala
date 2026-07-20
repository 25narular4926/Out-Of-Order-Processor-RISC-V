package RISCV

import chisel3._
import chisel3.util._

/**
 * Direct-mapped, BLOCKING instruction cache (v0) -- the first piece of the FPGA memory hierarchy.
 *
 * WHY IT EXISTS. Against realistic (multi-cycle) DRAM, fetching every instruction from memory
 * would stall the front end for `readLatency` cycles each time and crater IPC. A cache turns the
 * common case (a HIT) back into a ~1-cycle answer -- exactly the timing the rest of the core was
 * designed around -- and only pays the DRAM latency on a MISS (a cold line, or an evicted one).
 *
 * SHAPE (v0, deliberately simple; the interface is what matters, the internals will grow):
 *   - DIRECT-MAPPED: `nSets` lines, `lineWords` words per line. A word address splits as
 *         [ tag | index | offset ]   (offset = which word in the line, index = which line).
 *   - BLOCKING: at most one miss is serviced at a time. A miss refills the whole line by issuing
 *     `lineWords` single-word reads to memory IN SERIES, then answers the original request.
 *     (Bursts and non-blocking/MSHR behaviour are later optimizations; serial refill is correct
 *     and easy to reason about first.)
 *   - HIT LATENCY = 1 cycle (registered response), matching the old SyncReadMem instruction port,
 *     so a fully-cached workload behaves exactly like today.
 *
 * MEMORY SIDE. Talks to `ModeledRam` (in sim) / a DDR AXI bridge (on FPGA) through the shared
 * `MemMasterIO` request/response interface -- the same contract that survives to hardware.
 *
 * FPGA NOTE. The data array is a `Mem` (combinational) in v0 for simplicity; on FPGA you want a
 * `SyncReadMem` so it maps to BRAM (a 1-cycle registered read, which the response path already
 * tolerates). That is a localized v1 change, called out again at the array declaration.
 */
class ICache(
    p: OoOParams,
    nSets: Int = 64,     // number of cache lines (direct-mapped)
    lineWords: Int = 8   // words per line (spatial locality per miss)
) extends Module {
    require(isPow2(nSets) && nSets >= 2, "nSets must be a power of two >= 2")
    require(isPow2(lineWords) && lineWords >= 2, "lineWords must be a power of two >= 2")

    val offBits = log2Ceil(lineWords)
    val idxBits = log2Ceil(nSets)
    val tagBits = p.xlen - offBits - idxBits
    require(tagBits >= 1, "address does not leave room for a tag; shrink nSets/lineWords")

    val io = IO(new Bundle {
        // ---- CPU side: present a WORD address; the instruction returns on `resp` (valid may be
        //      delayed by a miss). One request in flight (req.ready is low while a miss refills). ----
        val req  = Flipped(Decoupled(UInt(p.xlen.W))) // word address
        val resp = Valid(UInt(p.xlen.W))              // instruction word

        // ---- memory side: to ModeledRam (sim) / DDR bridge (fpga) ----
        val mem = new MemMasterIO(p)

        // ---- stats (hardware counters, like the rest of the core) ----
        val hits   = Output(UInt(64.W))
        val misses = Output(UInt(64.W))
    })

    // -------------------------------------------------------------------- address slicing
    private def idxOf(a: UInt): UInt = a(offBits + idxBits - 1, offBits)
    private def tagOf(a: UInt): UInt = a(p.xlen - 1, offBits + idxBits)
    private def offOf(a: UInt): UInt = a(offBits - 1, 0)

    // -------------------------------------------------------------------- storage
    val validBits = RegInit(VecInit(Seq.fill(nSets)(false.B)))
    val tags      = Reg(Vec(nSets, UInt(tagBits.W)))
    // v0: combinational Mem. v1 (FPGA): SyncReadMem(nSets*lineWords, UInt) -> BRAM, 1-cycle read.
    val dataArr   = Mem(nSets * lineWords, UInt(p.xlen.W))

    // -------------------------------------------------------------------- FSM
    val sIdle :: sRefillReq :: sRefillWait :: sRespond :: Nil = Enum(4)
    val state = RegInit(sIdle)

    val reqAddr   = Reg(UInt(p.xlen.W))               // latched miss address
    val refillCnt = Reg(UInt((offBits + 1).W))        // which word of the line we are filling

    val hitCount  = RegInit(0.U(64.W))
    val missCount = RegInit(0.U(64.W))

    // hit detection for the request presented this cycle
    val inIdx = idxOf(io.req.bits)
    val inTag = tagOf(io.req.bits)
    val hit   = validBits(inIdx) && (tags(inIdx) === inTag)

    // -------------------------------------------------------------------- defaults
    io.req.ready     := state === sIdle
    io.mem.req.valid := false.B
    io.mem.req.bits  := 0.U.asTypeOf(new MemReq(p))

    // -------------------------------------------------------------------- response path (1-cycle)
    // `present` marks the cycle whose read we want to hand back; we register the array read so the
    // response arrives one cycle later -- matching the old instruction-memory timing on a hit.
    val present     = WireDefault(false.B)
    val presentAddr = WireDefault(0.U((idxBits + offBits).W))

    // -------------------------------------------------------------------- FSM transitions
    switch(state) {
        is(sIdle) {
            when(io.req.fire) {
                when(hit) {
                    hitCount    := hitCount + 1.U
                    present     := true.B
                    presentAddr := Cat(inIdx, offOf(io.req.bits))
                }.otherwise {
                    missCount := missCount + 1.U
                    reqAddr   := io.req.bits
                    refillCnt := 0.U
                    state     := sRefillReq
                }
            }
        }

        is(sRefillReq) {
            // issue a single-word read for the current word of the missing line
            io.mem.req.valid     := true.B
            io.mem.req.bits.write := false.B
            io.mem.req.bits.addr  := Cat(tagOf(reqAddr), idxOf(reqAddr), refillCnt(offBits - 1, 0))
            when(io.mem.req.fire) { state := sRefillWait }
        }

        is(sRefillWait) {
            when(io.mem.resp.valid) {
                dataArr.write(Cat(idxOf(reqAddr), refillCnt(offBits - 1, 0)), io.mem.resp.bits.rdata)
                when(refillCnt === (lineWords - 1).U) {
                    validBits(idxOf(reqAddr)) := true.B
                    tags(idxOf(reqAddr))      := tagOf(reqAddr)
                    state                     := sRespond
                }.otherwise {
                    refillCnt := refillCnt + 1.U
                    state     := sRefillReq
                }
            }
        }

        is(sRespond) {
            // line is filled; hand back the originally-requested word, then accept new work
            present     := true.B
            presentAddr := Cat(idxOf(reqAddr), offOf(reqAddr))
            state       := sIdle
        }
    }

    // registered response (array read is combinational; register it to land one cycle later)
    val respValid = RegNext(present, false.B)
    val respData  = RegNext(dataArr.read(presentAddr))
    io.resp.valid := respValid
    io.resp.bits  := respData

    io.hits   := hitCount
    io.misses := missCount
}
