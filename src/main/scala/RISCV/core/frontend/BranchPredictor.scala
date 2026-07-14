package RISCV

import chisel3._
import chisel3.util._

/**
 * Branch predictor for the 1-wide OoO front end.

 
 *
 * Three cooperating structures, all updated late (at branch resolution, driven by the ROB /
 * ALU via the `update` port which the orchestrator wires in later):
 *
 *   1. gshare DIRECTION predictor:
 *        - `p.bhtEntries` 2-bit saturating counters (MSB == "predict taken").
 *        - Indexed by  hash(PC, GHR)  where GHR is a `p.ghistBits`-bit global history shift
 *          register of recent branch outcomes (1 = taken). The hash is
 *              idx = (PC >> 2)[ghistBits-1:0] XOR GHR
 *          (drop the 2 low PC bits because instructions are word aligned), then truncated to
 *          log2(bhtEntries) bits. Classic gshare.
 *
 *   2. BTB (branch target buffer): `p.btbEntries` direct-mapped entries holding {valid, tag,
 *      target}. Indexed by low PC bits; the rest of the PC is the tag. Supplies the predicted
 *      target for taken branches/jumps. A BTB miss => we cannot know a target, so we predict
 *      not-taken (fall through to PC+4) regardless of the direction counter.
 *
 *   3. RAS (return-address stack): `p.rasEntries` deep. NOTE -- distinguishing calls/returns
 *      requires decode information (rd/rs1 == x1) that is not available at this pure-PC predict
 *      port. The RAS is therefore implemented and exposed but only driven through the update
 *      port's `pushRas`/`popRas` controls; the predict port does not consult it on its own in
 *      this first pass. Documented as a known simplification (see report).
 *
 * Timing: the predict port is purely combinational from the fetch PC so Fetch can compute the
 * next PC in the same cycle it presents the address. All tables update synchronously from the
 * `update` port.
 */
class BranchPredictor(p: OoOParams = OoOParams()) extends Module {

    // ---- derived index widths ----
    val bhtIdxBits = log2Ceil(p.bhtEntries)
    val btbIdxBits = log2Ceil(p.btbEntries)
    val rasIdxBits = log2Ceil(p.rasEntries)

    /** Predict port: given a fetch PC, produce a direction + target prediction. */
    class PredictIO extends Bundle {
        val pc = Input(UInt(p.xlen.W))
        val taken = Output(Bool())
        val target = Output(UInt(p.xlen.W))
    }

    /**
     * Update port: driven at branch resolution by the back end (ALU/ROB). All fields are
     * meaningful only when `valid` is asserted.
     */
    class UpdateIO extends Bundle {
        val valid = Input(Bool()) // a branch/jump resolved this cycle
        val pc = Input(UInt(p.xlen.W)) // PC of the resolved branch
        val taken = Input(Bool()) // actual direction
        val target = Input(UInt(p.xlen.W)) // actual taken target (for the BTB)
        val mispredicted = Input(Bool()) // whether the prediction was wrong (for stats/recovery)
        // RAS controls (driven by the back end once it knows call/return; optional in v1)
        val pushRas = Input(Bool())
        val pushRasAddr = Input(UInt(p.xlen.W))
        val popRas = Input(Bool())
    }

    val io = IO(new Bundle {
        val predict = new PredictIO
        val update = new UpdateIO
    })

    // =====================================================================================
    // Global history register: shift in the resolved direction on every branch update.
    // =====================================================================================
    val ghr = RegInit(0.U(p.ghistBits.W))

    // ---- gshare index helper (shared by predict and update so they stay consistent) ----
    private def bhtIndex(pc: UInt): UInt = {
        // drop the 2 low (always-zero) PC bits, take ghistBits of PC, xor with the history
        val pcBits = (pc >> 2.U)(p.ghistBits - 1, 0)
        val hashed = pcBits ^ ghr
        hashed(bhtIdxBits - 1, 0)
    }

    // =====================================================================================
    // gshare 2-bit saturating counter table. Reset to "weakly taken" (2) so unseen branches
    // default to predicting taken IF the BTB also hits with a target.
    // =====================================================================================
    val bht = RegInit(VecInit(Seq.fill(p.bhtEntries)(2.U(2.W))))

    // =====================================================================================
    // BTB: direct-mapped {valid, tag, target}.
    // =====================================================================================
    val btbTagBits = p.xlen - btbIdxBits - 2 // remaining PC bits above index (word aligned)
    val btbValid = RegInit(VecInit(Seq.fill(p.btbEntries)(false.B)))
    val btbTag = Reg(Vec(p.btbEntries, UInt(btbTagBits.W)))
    val btbTarget = Reg(Vec(p.btbEntries, UInt(p.xlen.W)))

    private def btbIndex(pc: UInt): UInt = (pc >> 2.U)(btbIdxBits - 1, 0)
    private def btbTagOf(pc: UInt): UInt = (pc >> 2.U)(btbIdxBits + btbTagBits - 1, btbIdxBits)

    // =====================================================================================
    // RAS: simple circular stack with a top-of-stack pointer.
    // =====================================================================================
    val ras = Reg(Vec(p.rasEntries, UInt(p.xlen.W)))
    val rasPtr = RegInit(0.U(rasIdxBits.W))

    // -------------------------------------------------------------------------------------
    // PREDICT (combinational)
    // -------------------------------------------------------------------------------------
    val pIdx = bhtIndex(io.predict.pc)
    val dirTaken = bht(pIdx)(1) // MSB of the 2-bit counter

    val bIdx = btbIndex(io.predict.pc)
    val btbHit = btbValid(bIdx) && (btbTag(bIdx) === btbTagOf(io.predict.pc))

    // Predict taken only when the direction counter says taken AND the BTB knows a target.
    io.predict.taken := dirTaken && btbHit
    io.predict.target := btbTarget(bIdx)

    // -------------------------------------------------------------------------------------
    // UPDATE (synchronous) -- driven at branch resolution.
    // -------------------------------------------------------------------------------------
    when(io.update.valid) {
        // ---- gshare counter saturating update ----
        val uIdx = bhtIndex(io.update.pc)
        val ctr = bht(uIdx)
        when(io.update.taken) {
            when(ctr =/= 3.U) { bht(uIdx) := ctr + 1.U }
        }.otherwise {
            when(ctr =/= 0.U) { bht(uIdx) := ctr - 1.U }
        }

        // ---- global history shift (newest bit in LSB) ----
        ghr := Cat(ghr(p.ghistBits - 2, 0), io.update.taken)

        // ---- BTB allocate/refill on taken branches (we only need a target when taken) ----
        when(io.update.taken) {
            val ui = btbIndex(io.update.pc)
            btbValid(ui) := true.B
            btbTag(ui) := btbTagOf(io.update.pc)
            btbTarget(ui) := io.update.target
        }
    }

    // ---- RAS push/pop (independent of branch resolution validity; back-end controlled) ----
    when(io.update.pushRas) {
        val nextPtr = rasPtr + 1.U
        ras(nextPtr) := io.update.pushRasAddr
        rasPtr := nextPtr
    }
    when(io.update.popRas) {
        rasPtr := rasPtr - 1.U
    }
}
