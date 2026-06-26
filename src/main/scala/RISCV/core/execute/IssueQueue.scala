package RISCV

import chisel3._
import chisel3.util._

/**
 * Unified integer + memory issue queue (`iqEntries` slots) for the 1-wide OoO core.
 *
 * Each slot holds a MicroOp plus its two source-ready bits (`prs1Ready`/`prs2Ready`). A uop is
 * enqueued from Dispatch with those bits pre-seeded from the BusyTable. Each cycle the queue
 * wakes up waiting sources from the common data bus, then selects the OLDEST ready uop for each
 * functional-unit class and presents it on the matching Decoupled port. A slot is freed when its
 * issue port fires.
 *
 * READY RULE: a slot is ready when
 *     (prs1Ready || !readsRs1) && (prs2Ready || !readsRs2).
 *
 * AGE MODEL -- AGE MATRIX (BOOM-style), chosen over a free-running counter to avoid any
 * wraparound foot-gun. `olderThan(i)(j) == true` means slot i is older (enqueued earlier) than
 * slot j. Maintenance:
 *   - On enqueue into slot k: every slot that is currently valid is older than k, so set
 *     olderThan(j)(k) := valid(j) for all j, and clear olderThan(k)(j) := false for all j
 *     (k is the youngest, older than nobody). The diagonal is always false.
 *   - Slot validity gates the relation, so a freed-then-reused slot starts fresh.
 * Selection per FU: a ready candidate i is the OLDEST iff no OTHER ready candidate j (same FU)
 * is older than i, i.e. there is no j with olderThan(j)(i). Exactly one candidate satisfies this
 * (the relation is a total order over live slots), so the one-hot select is unambiguous.
 *
 * WAKEUP TIMING: wakeup is 1-cycle-LATE relative to writeback. A CDB broadcast in cycle T sets
 * the slot's ready bit in the slot register, which is observed for selection in cycle T+1. So a
 * dependent uop issues one cycle after its producer's result appears on the CDB. (A speculative
 * same-cycle ALU wakeup is a future optimization; the simple late wakeup is correct and matches
 * the ExecAlu's registered 1-cycle output, which puts the result on the CDB one cycle after
 * issue anyway.)
 *
 * RECOVERY: on `redirect.valid` every slot is invalidated -- the commit-time full-flush model
 * (no branch masks). The age matrix is self-clearing because the relation is gated by validity.
 */
class IssueQueue(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val enq = Flipped(Decoupled(new MicroOp(p)))
        val wb = Input(Vec(p.numWbPorts, new WbPort(p)))
        val issueAlu = Decoupled(new MicroOp(p))
        val issueMulDiv = Decoupled(new MicroOp(p))
        val issueMem = Decoupled(new MicroOp(p))
        val redirect = Input(new Redirect(p))
    })

    private val n = p.iqEntries

    // ---- slot state ----
    val valids = RegInit(VecInit(Seq.fill(n)(false.B)))
    val uops = Reg(Vec(n, new MicroOp(p)))
    // Source-ready bits live alongside the uop but are mutated by wakeup, so track separately
    // from the (otherwise immutable) stored uop fields.
    val rs1Rdy = Reg(Vec(n, Bool()))
    val rs2Rdy = Reg(Vec(n, Bool()))
    // Age matrix: olderThan(i)(j) == slot i is older than slot j.
    val olderThan = RegInit(VecInit(Seq.fill(n)(VecInit(Seq.fill(n)(false.B)))))

    // ---- wakeup from the common data bus (updates the stored ready bits) ----
    // For every valid CDB port that writes a register, any waiting slot whose source physical tag
    // matches the broadcast pdst becomes ready.
    for (i <- 0 until n) {
        when(valids(i)) {
            for (port <- io.wb) {
                val hit = port.valid && port.writesReg && port.pdst =/= p.zeroPreg.U
                when(hit && uops(i).prs1 === port.pdst) { rs1Rdy(i) := true.B }
                when(hit && uops(i).prs2 === port.pdst) { rs2Rdy(i) := true.B }
            }
        }
    }

    // ---- per-slot readiness ----
    val slotReady = VecInit((0 until n).map { i =>
        valids(i) &&
            (rs1Rdy(i) || !uops(i).readsRs1) &&
            (rs2Rdy(i) || !uops(i).readsRs2)
    })

    /**
     * One-hot select of the oldest ready slot whose fuType matches `fu`. A candidate i wins iff
     * it is a candidate and no other candidate j is older than it.
     */
    def selectOldest(fu: FuType.Type): (Bool, UInt, Vec[Bool]) = {
        val cand = VecInit((0 until n).map(i => slotReady(i) && uops(i).fuType === fu))
        val winnerOH = VecInit((0 until n).map { i =>
            cand(i) && !(0 until n).map(j => cand(j) && olderThan(j)(i)).reduce(_ || _)
        })
        val found = cand.asUInt.orR
        val idx = OHToUInt(winnerOH.asUInt)
        (found, idx, winnerOH)
    }

    val (aluFound, aluIdx, _) = selectOldest(FuType.ALU)
    val (mdFound, mdIdx, _) = selectOldest(FuType.MULDIV)
    val (memFound, memIdx, _) = selectOldest(FuType.MEM)

    // ---- drive issue ports (forward the freshest ready bits) ----
    io.issueAlu.valid := aluFound
    io.issueAlu.bits := uops(aluIdx)
    io.issueAlu.bits.prs1Ready := rs1Rdy(aluIdx)
    io.issueAlu.bits.prs2Ready := rs2Rdy(aluIdx)

    io.issueMulDiv.valid := mdFound
    io.issueMulDiv.bits := uops(mdIdx)
    io.issueMulDiv.bits.prs1Ready := rs1Rdy(mdIdx)
    io.issueMulDiv.bits.prs2Ready := rs2Rdy(mdIdx)

    io.issueMem.valid := memFound
    io.issueMem.bits := uops(memIdx)
    io.issueMem.bits.prs1Ready := rs1Rdy(memIdx)
    io.issueMem.bits.prs2Ready := rs2Rdy(memIdx)

    // ---- deallocate fired slots ----
    when(io.issueAlu.fire) { valids(aluIdx) := false.B }
    when(io.issueMulDiv.fire) { valids(mdIdx) := false.B }
    when(io.issueMem.fire) { valids(memIdx) := false.B }

    // ---- enqueue into the first free slot ----
    val freeOH = VecInit((0 until n).map(i => !valids(i)))
    val hasFree = freeOH.asUInt.orR
    val freeIdx = PriorityEncoder(freeOH.asUInt)
    io.enq.ready := hasFree

    when(io.enq.fire) {
        valids(freeIdx) := true.B
        uops(freeIdx) := io.enq.bits
        rs1Rdy(freeIdx) := io.enq.bits.prs1Ready
        rs2Rdy(freeIdx) := io.enq.bits.prs2Ready
        // Age matrix: the newcomer is younger than every currently-valid slot.
        for (j <- 0 until n) {
            olderThan(freeIdx)(j) := false.B // newcomer is older than nobody
            when(valids(j)) { olderThan(j)(freeIdx) := true.B } // every live slot is older than it
        }
        olderThan(freeIdx)(freeIdx) := false.B // diagonal stays false
    }

    // ---- redirect: full flush (commit-time recovery, no branch masks) ----
    when(io.redirect.valid) {
        for (i <- 0 until n) { valids(i) := false.B }
        // The age relation is gated by validity, so clearing valids is sufficient; reset the
        // matrix too for cleanliness/determinism.
        for (i <- 0 until n; j <- 0 until n) { olderThan(i)(j) := false.B }
    }
}
