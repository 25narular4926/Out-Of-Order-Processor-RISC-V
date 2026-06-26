package RISCV

import chisel3._
import chisel3.util._

/* =====================================================================================
 * FROZEN INTERFACE SPEC
 * -------------------------------------------------------------------------------------
 * Every module in the OoO core talks through the bundles defined here. Treat this file as
 * a contract: do not change a field's name, width, or meaning without updating ALL
 * producers and consumers. Each bundle documents WHO fills each field and WHO reads it.
 *
 * Recovery model (first pass) -- IMPORTANT, read before implementing any flush logic:
 *   Branch mispredictions AND exceptions both recover IN ORDER, at the ROB head, via a
 *   single full flush. When the offending instruction reaches the head and is complete,
 *   the ROB asserts `Redirect`, which means: "discard ALL speculative state and restart
 *   fetch at `target`." Because the head is the oldest instruction, everything else in the
 *   machine is younger and is simply cleared -- so there are NO branch masks, NO rename
 *   snapshots, and NO age comparisons in this version. Rename recovers by reverting to the
 *   committed map table (which the ROB maintains as it retires). The `brMask`/`brTag`
 *   fields in MicroOp are reserved for a future branch-mask fast-recovery upgrade and are
 *   ignored by the first-pass flush path.
 * ===================================================================================== */

/** Which functional unit a uop executes on. Branches/jumps resolve on the ALU unit. */
object FuType extends ChiselEnum {
    val ALU, MULDIV, MEM = Value
}

/**
 * The micro-op: the single packet of state that flows through the whole machine and is
 * stored in the IssueQueue, ROB, LDQ, and STQ. It is filled PROGRESSIVELY:
 *
 *   Decode    fills: pc, opcode, func3, func7, immediate, fuType, lrs1/lrs2/ldst,
 *                    readsRs1/readsRs2/writesReg, isBranch/isJump/isJalr/isLoad/isStore,
 *                    predTaken, predTarget
 *   Rename    fills: prs1, prs2, pdst, pdstOld, prs1Ready, prs2Ready
 *   Dispatch  fills: robIdx, ldqIdx, stqIdx
 *
 * Fields not yet filled by a given stage must be driven to 0 (not left dangling).
 */
class MicroOp(p: OoOParams) extends Bundle {
    // ---- identity / debug ----
    val pc = UInt(p.xlen.W)

    // ---- decoded operation (filled by Decode) ----
    val opcode = UInt(7.W)
    val func3 = UInt(3.W)
    val func7 = UInt(7.W)
    val immediate = UInt(p.xlen.W)
    val fuType = FuType()

    // ---- architectural registers (filled by Decode; used for commit-map update + debug) ----
    val lrs1 = UInt(p.archRegWidth.W)
    val lrs2 = UInt(p.archRegWidth.W)
    val ldst = UInt(p.archRegWidth.W)

    // ---- control flags (filled by Decode) ----
    val readsRs1 = Bool()
    val readsRs2 = Bool()
    val writesReg = Bool() // true => allocates a pdst and writes the register file at writeback
    val isBranch = Bool() // conditional branch (beq/bne/blt/bge/bltu/bgeu)
    val isJump = Bool() // unconditional jal
    val isJalr = Bool() // jalr (indirect jump)
    val isLoad = Bool()
    val isStore = Bool()

    // ---- renamed physical tags (filled by Rename) ----
    val prs1 = UInt(p.pregWidth.W)
    val prs2 = UInt(p.pregWidth.W)
    val pdst = UInt(p.pregWidth.W) // == zeroPreg when !writesReg
    val pdstOld = UInt(p.pregWidth.W) // previous mapping of ldst; freed at commit
    val prs1Ready = Bool() // source-1 value already available (from Busy Table at rename)
    val prs2Ready = Bool() // source-2 value already available

    // ---- index bookkeeping (filled by Dispatch) ----
    val robIdx = UInt(p.robIdxWidth.W)
    val ldqIdx = UInt(p.ldqIdxWidth.W)
    val stqIdx = UInt(p.stqIdxWidth.W)

    // ---- branch prediction (filled by Decode/Fetch; checked at execute) ----
    val predTaken = Bool()
    val predTarget = UInt(p.xlen.W)

    // ---- reserved for future branch-mask fast recovery (unused in first-pass recovery) ----
    val brMask = UInt(p.brMaskWidth.W)
    val brTag = UInt(p.brTagWidth.W)

    def isBranchOrJump: Bool = isBranch || isJump || isJalr
}

object MicroOp {
    def apply(p: OoOParams): MicroOp = new MicroOp(p)

    /** A bubble / cleared micro-op (all fields zero). */
    def nop(p: OoOParams): MicroOp = 0.U.asTypeOf(new MicroOp(p))
}

/**
 * Common Data Bus / writeback port. A functional unit drives one of these when it finishes.
 * There are `numWbPorts` of them (ALU, MULDIV, MEM); all consumers snoop the whole Vec.
 *   - PhysRegFile   reads {valid, writesReg, pdst, data}  -> writes the result.
 *   - IssueQueue    reads {valid, writesReg, pdst}        -> wakeup (clear source-busy).
 *   - BusyTable     reads {valid, writesReg, pdst}        -> clear the busy bit.
 *   - ReorderBuffer reads {valid, robIdx, + branch/excn}  -> mark entry done & record redirect.
 */
class WbPort(p: OoOParams) extends Bundle {
    val valid = Bool()
    val robIdx = UInt(p.robIdxWidth.W)
    val pdst = UInt(p.pregWidth.W)
    val writesReg = Bool()
    val data = UInt(p.xlen.W)

    // branch/jump resolution (meaningful only when the uop isBranchOrJump)
    val isBranchResolved = Bool()
    val mispredicted = Bool() // predicted direction/target != actual
    val brTaken = Bool() // actual taken (for predictor update)
    val brTarget = UInt(p.xlen.W) // actual next PC (the correct redirect target)

    // exceptions (reserved; e.g. illegal instruction). Recovered at commit like a mispredict.
    val exception = Bool()
}

object WbPort {
    def default(p: OoOParams): WbPort = 0.U.asTypeOf(new WbPort(p))
}

/** Operands packaged for a functional unit after the PRF read (Core wires IQ->PRF->FU). */
class ExecReq(p: OoOParams) extends Bundle {
    val uop = new MicroOp(p)
    val rs1Data = UInt(p.xlen.W)
    val rs2Data = UInt(p.xlen.W)
}

/**
 * Global redirect / flush. In the first-pass recovery model this is asserted ONLY by the
 * ReorderBuffer, at the head, for a mispredicted branch or an exception. Its meaning is:
 * "flush ALL speculative state and restart fetch at `target`." Every speculative structure
 * (Fetch, Decode, Rename, Dispatch, IssueQueue, PhysRegFile-pending, LSU, ROB) must clear on
 * `valid`. Rename additionally reverts its map/free-list to the committed state.
 */
class Redirect(p: OoOParams) extends Bundle {
    val valid = Bool()
    val target = UInt(p.xlen.W)
}

object Redirect {
    def default(p: OoOParams): Redirect = {
        val r = Wire(new Redirect(p))
        r.valid := false.B
        r.target := 0.U
        r
    }
}

/** One reorder-buffer entry. Allocated in program order at Dispatch, retired in order at head. */
class RobEntry(p: OoOParams) extends Bundle {
    val valid = Bool() // slot occupied
    val done = Bool() // result written back (ready to retire)
    val pc = UInt(p.xlen.W)
    val ldst = UInt(p.archRegWidth.W) // architectural destination
    val pdst = UInt(p.pregWidth.W) // new physical mapping (becomes committed at retire)
    val pdstOld = UInt(p.pregWidth.W) // previous mapping -> freed at retire
    val writesReg = Bool()
    val isStore = Bool() // commit triggers the actual memory write
    val isLoad = Bool()
    val isBranchOrJump = Bool()
    val mispredicted = Bool() // set at writeback; triggers redirect when this entry retires
    val exception = Bool()
    val redirectTarget = UInt(p.xlen.W) // correct next-PC for a mispredict/exception
}

/**
 * Retirement broadcast from the ROB head (one per cycle in the 1-wide machine).
 *   - Rename uses {valid, ldst, pdst, pdstOld, writesReg} to update the committed map and
 *     push pdstOld back to the free list.
 *   - LSU uses {valid, isStore} to drain the matching STQ head into memory.
 */
class CommitSignal(p: OoOParams) extends Bundle {
    val valid = Bool()
    val robIdx = UInt(p.robIdxWidth.W)
    val pc = UInt(p.xlen.W)
    val ldst = UInt(p.archRegWidth.W)
    val pdst = UInt(p.pregWidth.W)
    val pdstOld = UInt(p.pregWidth.W)
    val writesReg = Bool()
    val isStore = Bool()
    val isMMIO = Bool()
}