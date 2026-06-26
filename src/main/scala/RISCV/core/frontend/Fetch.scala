package RISCV

import chisel3._
import chisel3.util._

/**
 * One packet of fetched-but-not-yet-decoded instruction state. Produced by Fetch, consumed by
 * Decode. (Defined here, in Fetch.scala, per the design split.)
 */
class FetchPacket(p: OoOParams) extends Bundle {
    val pc = UInt(p.xlen.W)
    val instruction = UInt(p.xlen.W)
    val predTaken = Bool()
    val predTarget = UInt(p.xlen.W)
}

/**
 * In-order instruction fetch with an integrated gshare/BTB branch predictor.
 *
 * Memory model: `imemData` is the instruction read from a SyncReadMem, so it appears ONE cycle
 * after `imemAddr` is presented. We therefore run a two-stage pipeline:
 *
 *   S0 (address stage): a PC register drives `imemAddr`. We also predict in S0 (combinational,
 *                       from the predictor) and register {pc, predTaken, predTarget} so they
 *                       line up with the instruction word that arrives next cycle.
 *   S1 (data stage):    `imemData` holds the instruction for the address presented in the
 *                       previous cycle. We pair it with the registered S0 metadata to form the
 *                       FetchPacket presented on `out`.
 *
 * Backpressure (out.ready) + 1-cycle latency are reconciled with a 1-deep SKID BUFFER. When
 * `out` is valid but the consumer is not ready, the freshly-arrived instruction is captured in
 * the skid register and replayed from there; meanwhile S0 stalls (PC frozen, no new address
 * issued) so no fetch is lost. This is the simplest correct scheme; it costs one fetch slot of
 * latency when downstream stalls but never drops or duplicates an instruction.
 *
 * Redirect: on `redirect.valid`, the PC jumps to `redirect.target`, and any instruction in S1
 * or the skid buffer is squashed (it is younger than the redirecting instruction, which by the
 * commit-time full-flush model is always correct to discard).
 */
class Fetch(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val enable = Input(Bool())
        val redirect = Input(new Redirect(p))

        // instruction memory: present a BYTE address, get the word back next cycle
        val imemAddr = Output(UInt(p.xlen.W))
        val imemData = Input(UInt(p.xlen.W))

        val out = Decoupled(new FetchPacket(p))
    })

    val predictor = Module(new BranchPredictor(p))

    // =====================================================================================
    // S0 -- address / predict stage
    // =====================================================================================
    val pc = RegInit(0.U(p.xlen.W))

    // Predict from the current PC (combinational).
    predictor.io.predict.pc := pc
    val predTaken = predictor.io.predict.taken
    val predTarget = predictor.io.predict.target

    // The predictor update port is NOT driven by Fetch in this first pass (branch resolution
    // lives in the back end). The orchestrator wires it from the ROB/ALU. Tie off here so the
    // module elaborates standalone; the orchestrator can override these from outside Fetch by
    // exposing the predictor, but per the fixed Frontend IO the update port stays internal and
    // is left in its reset behaviour for v1. See report.
    predictor.io.update.valid := false.B
    predictor.io.update.pc := 0.U
    predictor.io.update.taken := false.B
    predictor.io.update.target := 0.U
    predictor.io.update.mispredicted := false.B
    predictor.io.update.pushRas := false.B
    predictor.io.update.pushRasAddr := 0.U
    predictor.io.update.popRas := false.B

    // =====================================================================================
    // S1 -- data stage + 1-deep skid buffer
    // =====================================================================================
    // s1Valid: an address was issued last cycle, so imemData is a real instruction this cycle.
    val s1Valid = RegInit(false.B)
    val s1Pc = Reg(UInt(p.xlen.W))
    val s1PredTaken = Reg(Bool())
    val s1PredTarget = Reg(UInt(p.xlen.W))

    // Skid buffer: holds one instruction that arrived while `out` was back-pressured.
    val skidValid = RegInit(false.B)
    val skidPkt = Reg(new FetchPacket(p))

    // The instruction word that arrives this cycle (valid iff s1Valid), packaged with the S0
    // metadata that was registered alongside its address.
    val freshPkt = Wire(new FetchPacket(p))
    freshPkt.pc := s1Pc
    freshPkt.instruction := io.imemData
    freshPkt.predTaken := s1PredTaken
    freshPkt.predTarget := s1PredTarget

    // Output mux: prefer the skid buffer (which holds the OLDER instruction) over fresh S1.
    io.out.valid := (skidValid || s1Valid) && !io.redirect.valid
    io.out.bits := Mux(skidValid, skidPkt, freshPkt)

    // S0 issues a new address (and advances the PC) only when the pipeline has room for the
    // instruction that would arrive next cycle. There is room unless we are already holding a
    // backed-up instruction in the skid buffer, or S1 is occupied by an instruction that is not
    // being consumed this cycle (which would have to go into the skid). Concretely: S0 may fire
    // when the skid is empty AND (S1 is empty OR S1 will drain this cycle).
    val s1WillDrain = s1Valid && io.out.ready && !skidValid
    val hasRoom = !skidValid && (!s1Valid || s1WillDrain)
    val s0Fire = io.enable && hasRoom && !io.redirect.valid

    // The address presented to memory is always the current PC; we only advance it on s0Fire.
    io.imemAddr := pc

    when(io.redirect.valid) {
        // Flush: restart fetch at the redirect target, squash everything in flight. Anything in
        // S1 or the skid buffer is younger than the redirecting instruction, so discarding it is
        // always correct under the commit-time full-flush model.
        pc := io.redirect.target
        s1Valid := false.B
        skidValid := false.B
    }.otherwise {
        // ---- skid buffer: capture S1 when it cannot drain, release when consumed ----
        when(skidValid) {
            when(io.out.ready) { skidValid := false.B } // consumer took the skid entry
        }.elsewhen(s1Valid && !io.out.ready) {
            // S1 held a valid instruction the consumer didn't take this cycle; park it so S0 can
            // (eventually) keep moving without dropping it.
            skidValid := true.B
            skidPkt := freshPkt
        }

        // ---- S1 valid bit ----
        // Next cycle S1 is valid iff we issued a fresh address this cycle (s0Fire). If we did
        // not issue but S1 was occupied and not drained, it has been moved to the skid above, so
        // S1 itself becomes empty. Hence: s1Valid' = s0Fire.
        s1Valid := s0Fire

        // ---- S0 -> S1 metadata handoff and PC advance ----
        when(s0Fire) {
            s1Pc := pc
            s1PredTaken := predTaken
            s1PredTarget := predTarget
            pc := Mux(predTaken, predTarget, pc + 4.U)
        }
    }
}