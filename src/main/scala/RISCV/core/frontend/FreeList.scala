package RISCV

import chisel3._
import chisel3.util._

/**
 * Physical-register free list.
 *
 * Tracks which physical registers are NOT currently part of any (speculative) mapping and are
 * therefore available to be allocated as a new destination tag at rename.
 *
 * Implemented as a one-bit-per-preg "free" bit vector (a bitmap free list). This is simple and
 * trivially supports the full-flush recovery model: on redirect we rebuild the entire free set
 * from the committed map (everything not architecturally live becomes free again).
 *
 *   - allocate (rename): pop the lowest-numbered free preg, clear its free bit.
 *   - free    (commit) : push `pushTag` (the committed instruction's pdstOld) back, set its free bit.
 *   - redirect         : free := all pregs EXCEPT zeroPreg and those present in the committed map.
 *
 * zeroPreg is never free (it is permanently architectural x0).
 */
class FreeList(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        // ---- allocate ----
        val allocReq = Input(Bool()) // rename wants a new physical destination
        val allocTag = Output(UInt(p.pregWidth.W)) // the preg handed out (valid iff !empty)
        val empty = Output(Bool()) // no free preg available -> rename must stall

        // ---- free (push back at commit) ----
        val freeReq = Input(Bool())
        val freeTag = Input(UInt(p.pregWidth.W))

        // ---- redirect: rebuild from the committed map ----
        val redirect = Input(Bool())
        // the committed mapping of every architectural register (driven by the MapTable's
        // committed copy); used to decide which pregs are architecturally live after a flush.
        val committedMap = Input(Vec(p.numArchRegs, UInt(p.pregWidth.W)))
    })

    // free(i) == 1 means physical register i is available for allocation.
    // Reset state must match MapTable's reset (identity map: arch i -> preg i). Pregs
    // [0, numArchRegs) are therefore live at reset; [numArchRegs, numPhysRegs) are free.
    val initFree = VecInit((0 until p.numPhysRegs).map(i => (i >= p.numArchRegs).B))
    val free = RegInit(initFree)

    // ---- pick the lowest free preg (priority encode) ----
    val hasFree = free.asUInt.orR
    val allocIdx = PriorityEncoder(free.asUInt)

    io.empty := !hasFree
    io.allocTag := allocIdx

    // A real allocation happens only when requested AND a preg is available.
    val doAlloc = io.allocReq && hasFree

    when(io.redirect) {
        // Rebuild: a preg is free iff it is neither zeroPreg nor present in the committed map.
        // Compute architectural liveness as a one-hot-union over the committed map entries.
        val live = Wire(Vec(p.numPhysRegs, Bool()))
        for (pr <- 0 until p.numPhysRegs) {
            // preg pr is live if any architectural register (other than x0) maps to it, or pr is zeroPreg
            val mappedByAny = io.committedMap.zipWithIndex.map { case (m, arch) =>
                (arch != 0).B && (m === pr.U)
            }
            live(pr) := (pr.U === p.zeroPreg.U) || VecInit(mappedByAny).asUInt.orR
        }
        for (pr <- 0 until p.numPhysRegs) {
            free(pr) := !live(pr)
        }
    }.otherwise {
        // Normal operation: allocate and free can both happen the same cycle (different pregs).
        when(doAlloc) {
            free(allocIdx) := false.B
        }
        when(io.freeReq && io.freeTag =/= p.zeroPreg.U) {
            free(io.freeTag) := true.B
        }
    }
}