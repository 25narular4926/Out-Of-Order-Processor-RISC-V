package RISCV

import chisel3._
import chisel3.util._

/**
 * Reorder Buffer (ROB) for the 1-wide BOOM-style out-of-order core.
 *
 * THE ONE RULE: instructions execute out of order but COMMIT IN ORDER. The ROB is the
 * structure that enforces in-order retirement: it is a circular FIFO of `RobEntry`,
 * allocated in program order at Dispatch (tail) and retired in program order from the
 * head. Architectural state (register map, memory) only changes when an entry retires.
 *
 * RECOVERY MODEL (first pass, matches the Bundles.scala header):
 *   Branch mispredicts and exceptions both recover IN ORDER at the head via a single full
 *   flush. When the offending instruction reaches the head and is `done`, the ROB asserts
 *   `Redirect` ("discard ALL speculative state, restart fetch at target") and clears itself
 *   completely. Because the head is the oldest in-flight instruction, every other entry is
 *   younger and is simply dropped -- so there are NO branch masks and NO age comparisons.
 *
 * POINTERS:
 *   - We use the standard "extra wrap bit" technique: head/tail are (robIdxWidth+1)-bit
 *     counters. The low robIdxWidth bits index the entry array; the top bit distinguishes
 *     "empty" (head == tail) from "full" (indices equal, wrap bits differ). This avoids the
 *     classic ambiguity of a circular buffer that can be exactly full or empty.
 *   - `allocIdx`/`robHeadIdx` expose only the low (index) bits, matching robIdxWidth.
 *
 * ALLOCATION INDEX is combinational (= current tail index) so Dispatch can stamp the uop's
 * robIdx BEFORE it asserts allocReq, exactly like the LSU's ldq/stq index handout.
 */
class ReorderBuffer(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        // ---- allocation (from Dispatch, in program order) ----
        val allocReq = Input(Bool())
        val allocUop = Input(new MicroOp(p))
        val allocIdx = Output(UInt(p.robIdxWidth.W)) // current tail index, combinational
        val full     = Output(Bool())

        // ---- writeback snoop (Common Data Bus) ----
        val wb = Input(Vec(p.numWbPorts, new WbPort(p)))

        // ---- in-order retirement broadcast ----
        val commit = Output(new CommitSignal(p))

        // ---- global flush, generated here at the head ----
        val redirect = Output(new Redirect(p))

        // ---- head index (LSU uses it for the MMIO-at-head rule) ----
        val robHeadIdx = Output(UInt(p.robIdxWidth.W))

        val empty = Output(Bool())
    })

    // ------------------------------------------------------------------ storage
    val entries = Reg(Vec(p.robEntries, new RobEntry(p)))

    // head/tail with an extra wrap bit (see header). Width = robIdxWidth + 1.
    val ptrW = p.robIdxWidth + 1
    val head = RegInit(0.U(ptrW.W))
    val tail = RegInit(0.U(ptrW.W))

    // low-order index bits used to address the entry array
    val headIdx = head(p.robIdxWidth - 1, 0)
    val tailIdx = tail(p.robIdxWidth - 1, 0)

    // empty when the full counters match; full when indices match but wrap bits differ.
    val isEmpty = head === tail
    val isFull  = (headIdx === tailIdx) && (head(p.robIdxWidth) =/= tail(p.robIdxWidth))

    io.empty      := isEmpty
    io.full       := isFull
    io.allocIdx   := tailIdx
    io.robHeadIdx := headIdx

    // ------------------------------------------------------------------ writeback snoop
    // For each CDB port, mark the addressed entry done and, for branches/jumps, record the
    // resolution so the redirect can be generated when the entry eventually retires.
    // (We snoop into a "next" view of the array so an alloc + writeback to the same slot in
    // the same cycle is well defined: allocation below writes the freshly-allocated entry.)
    for (port <- io.wb) {
        when(port.valid && entries(port.robIdx).valid) {
            entries(port.robIdx).done := true.B
            // Branch/jump resolution: a mispredict (wrong direction or wrong target) means we
            // must redirect to the architecturally-correct next PC carried on brTarget.
            when(entries(port.robIdx).isBranchOrJump && port.mispredicted) {
                entries(port.robIdx).mispredicted   := true.B
                entries(port.robIdx).redirectTarget := port.brTarget
            }
            // Exceptions recover at commit like a mispredict; redirectTarget is left as the
            // brTarget if the FU provided one (e.g. a trap vector); otherwise it stays 0.
            when(port.exception) {
                entries(port.robIdx).exception     := true.B
                entries(port.robIdx).redirectTarget := port.brTarget
            }
        }
    }

    // ------------------------------------------------------------------ allocation
    // Stamp a fresh entry at the tail from the dispatched uop. Allowed only when not full.
    val doAlloc = io.allocReq && !isFull
    when(doAlloc) {
        val e = Wire(new RobEntry(p))
        e.valid          := true.B
        e.done           := false.B
        e.pc             := io.allocUop.pc
        e.ldst           := io.allocUop.ldst
        e.pdst           := io.allocUop.pdst
        e.pdstOld        := io.allocUop.pdstOld
        e.writesReg      := io.allocUop.writesReg
        e.isStore        := io.allocUop.isStore
        e.isLoad         := io.allocUop.isLoad
        e.isBranchOrJump := io.allocUop.isBranchOrJump
        e.mispredicted   := false.B
        e.exception      := false.B
        e.redirectTarget := 0.U
        entries(tailIdx) := e
        tail := tail + 1.U
    }

    // ------------------------------------------------------------------ retirement (head)
    val headEntry = entries(headIdx)
    val canRetire = !isEmpty && headEntry.valid && headEntry.done

    // Commit broadcast: valid only when the head is ready to retire.
    val commit = Wire(new CommitSignal(p))
    commit.valid     := canRetire
    commit.robIdx    := headIdx
    commit.pc        := headEntry.pc
    commit.ldst      := headEntry.ldst
    commit.pdst      := headEntry.pdst
    commit.pdstOld   := headEntry.pdstOld
    commit.writesReg := headEntry.writesReg
    commit.isStore   := headEntry.isStore
    // isMMIO is not knowable from the ROB alone (no address tracked here). The LSU determines
    // MMIO from the store's resolved address at commit time, so we leave this false. -- GAP, see report.
    commit.isMMIO    := false.B
    io.commit := commit

    // Redirect generation: when the retiring head is a mispredict or an exception, this is the
    // global flush. Everything younger is dropped by clearing the whole ROB.
    val headFlushes = canRetire && (headEntry.mispredicted || headEntry.exception)

    val redirect = Wire(new Redirect(p))
    redirect.valid  := headFlushes
    redirect.target := headEntry.redirectTarget
    io.redirect := redirect

    when(canRetire) {
        // Free the retired slot and advance the head.
        entries(headIdx).valid := false.B
        entries(headIdx).done  := false.B
        head := head + 1.U
    }

    // FULL FLUSH on a head-generated redirect: reset to empty and clear every valid bit.
    // This happens AFTER the normal retire bump above; the explicit reset below wins because
    // last-connect semantics make these assignments override the head+1/clear-slot writes.
    when(headFlushes) {
        head := 0.U
        tail := 0.U
        for (i <- 0 until p.robEntries) {
            entries(i).valid := false.B
            entries(i).done  := false.B
        }
    }
}