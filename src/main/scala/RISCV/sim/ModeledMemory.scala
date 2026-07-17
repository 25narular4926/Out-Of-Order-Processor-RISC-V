package RISCV

import chisel3._
import chisel3.util._
import chisel3.util.experimental.loadMemoryFromFileInline

/**
 * ============================================================================================
 * PROTOTYPE / DESIGN SKETCH -- a latency- and bandwidth-modeled RAM with a multi-master arbiter.
 * ============================================================================================
 *
 * WHY THIS EXISTS. Today the core talks to a `SyncReadMem` that answers in 1 cycle with infinite
 * capacity -- a simulation convenience that hides the single hardest FPGA reality: real DRAM is
 * MANY cycles away and shared between contending masters (instruction fetch, the LSU, and -- once
 * it exists -- the GPU). This module lets you feel that reality inside Verilator BEFORE any board
 * exists, so caches / MSHRs / the GPU get debugged against realistic memory, not the 1-cycle ideal.
 *
 * WHAT IT MODELS.
 *   - READ LATENCY: a read issued in cycle T returns `readLatency` cycles later (>= 1). Set it to
 *     1 and this reproduces exactly today's behaviour; set it to ~10-30 to model post-cache-miss
 *     DRAM latency.
 *   - BANDWIDTH: at most one access is accepted every `issueEvery` cycles. issueEvery=1 is "one
 *     access/cycle" (full throughput); higher values starve the masters to model limited DRAM BW.
 *   - CONTENTION: N masters arbitrate (round-robin) for the single memory. Each response is routed
 *     back to the master that issued it, `readLatency` cycles later.
 *
 * WHAT IT DELIBERATELY DOES NOT MODEL (first pass; add later if you want them):
 *   - Variable / reordered latency (real DRAM has row hits/misses). This is FIXED, in-order latency
 *     -- responses come back in issue order. That is the right first model; it is deterministic and
 *     easy to reason about.
 *   - MMIO. This module is RAM only. The top level should decode MMIO (timer/keys/fb/tohost) BEFORE
 *     routing RAM accesses (addr < memWords) here; MMIO answers immediately and must not pay DRAM
 *     latency. Keeping MMIO out keeps this model focused.
 *
 * INTEGRATION (the real cost is in the consumers, not here):
 *   - master 0 = instruction fetch (read-only), master 1 = LSU data (read+write), master 2 = GPU.
 *   - Fetch/LSU must become LATENCY-TOLERANT: issue a req, then wait for `resp.valid` (variable),
 *     instead of assuming the answer arrives at T+1. Without a cache, EVERY access pays
 *     `readLatency` -- IPC will crater. That is the point: it shows, quantitatively, why you need
 *     an I-cache/D-cache (hits stay ~1 cycle) and MSHRs (overlap misses) before real DRAM.
 * ============================================================================================
 */

/** A memory request. Address is a WORD address (byte addr / 4), matching the LSU/Memory. */
class MemReq(p: OoOParams) extends Bundle {
    val addr  = UInt(p.xlen.W)
    val write = Bool()
    val wdata = UInt(p.xlen.W)
    val wmask = UInt(4.W) // per-byte write enables (bit i writes byte lane i), like the LSU
}

/** A memory response (reads only). */
class MemResp(p: OoOParams) extends Bundle {
    val rdata = UInt(p.xlen.W)
}

/**
 * One master's port, from the MASTER's point of view: it drives `req` (Decoupled) and receives
 * `resp` (Valid, reads only). The memory module wraps this in Flipped(...), so it consumes reqs
 * and produces resps.
 */
class MemMasterIO(p: OoOParams) extends Bundle {
    val req  = Decoupled(new MemReq(p))
    val resp = Flipped(Valid(new MemResp(p)))
}

class ModeledRam(
    p: OoOParams,
    nMasters: Int,
    readLatency: Int = 20,   // total cycles from accept to response (>= 1)
    issueEvery: Int = 1,     // accept a new access only every issueEvery cycles (bandwidth model)
    memInitFile: String = ""
) extends Module {
    require(readLatency >= 1, "readLatency must be >= 1")
    require(issueEvery >= 1, "issueEvery must be >= 1")
    require(nMasters >= 1)

    val io = IO(new Bundle {
        val masters = Vec(nMasters, Flipped(new MemMasterIO(p)))
    })

    // Four byte lanes per word so writes can be masked per byte (matches Memory.scala).
    val mem = SyncReadMem(p.memWords, Vec(4, UInt(8.W)))
    if (memInitFile.nonEmpty) loadMemoryFromFileInline(mem, memInitFile)

    private def toLanes(w: UInt): Vec[UInt] =
        VecInit(Seq.tabulate(4)(i => w(8 * i + 7, 8 * i)))

    // ---- bandwidth throttle: a fresh access may be accepted only when `gap == 0` --------------
    val gap      = RegInit(0.U(log2Ceil(issueEvery + 1).W))
    val slotOpen = gap === 0.U

    // ---- round-robin arbitration among the masters -------------------------------------------
    val arb = Module(new RRArbiter(new MemReq(p), nMasters))
    for (i <- 0 until nMasters) {
        // forward each master's request into the arbiter (explicit to keep directions obvious)
        arb.io.in(i).valid      := io.masters(i).req.valid
        arb.io.in(i).bits       := io.masters(i).req.bits
        io.masters(i).req.ready := arb.io.in(i).ready
    }
    arb.io.out.ready := slotOpen
    val accept = arb.io.out.valid && slotOpen
    val chosen = arb.io.chosen
    val req    = arb.io.out.bits

    when(accept) {
        gap := (issueEvery - 1).U
    }.elsewhen(gap =/= 0.U) {
        gap := gap - 1.U
    }

    // ---- issue the access to the SRAM this cycle ---------------------------------------------
    val doRead  = accept && !req.write
    val doWrite = accept && req.write
    val rdLanes = mem.read(req.addr, doRead) // SyncReadMem: data is valid one cycle later
    when(doWrite) {
        mem.write(req.addr, toLanes(req.wdata), Seq.tabulate(4)(i => req.wmask(i)))
    }

    // ---- stage 1 (T+1): align the token with the SRAM's inherent 1-cycle read ---------------
    // s1Data is `rdLanes` used one cycle after the read was issued (SyncReadMem's own latency).
    val s1Valid  = RegNext(doRead, false.B)
    val s1Master = RegNext(chosen)
    val s1Data   = rdLanes.asUInt

    // ---- delay by (readLatency - 1) MORE cycles to reach the requested total latency ---------
    // readLatency == 1 => shift by 0 => response at T+1, i.e. today's exact behaviour.
    val respValid  = ShiftRegister(s1Valid,  readLatency - 1, false.B, true.B)
    val respMaster = ShiftRegister(s1Master, readLatency - 1)
    val respData   = ShiftRegister(s1Data,   readLatency - 1)

    // ---- route the (in-order) response back to the master that issued it ----------------------
    for (i <- 0 until nMasters) {
        io.masters(i).resp.valid      := respValid && (respMaster === i.U)
        io.masters(i).resp.bits.rdata := respData
    }
}
