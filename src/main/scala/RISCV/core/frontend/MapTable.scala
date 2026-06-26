package RISCV

import chisel3._
import chisel3.util._

/**
 * Register rename map table.
 *
 * Holds, for each of the `p.numArchRegs` architectural registers, the physical register
 * (`p.pregWidth` bits) that currently realizes it. Two copies are maintained:
 *
 *   - speculative map: updated at RENAME, read to find a uop's source/old-dest tags. May hold
 *     mappings for not-yet-committed (speculative) instructions.
 *   - committed map: updated at COMMIT (retire). Reflects only architectural state.
 *
 * On `redirect` the speculative map is reverted to the committed map in a single cycle (the
 * commit-time full-flush recovery model: everything speculative is discarded).
 *
 * x0 is permanently mapped to `p.zeroPreg` in both maps and can never be rewritten.
 */
class MapTable(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        // ---- speculative read ports (combinational, used at rename) ----
        val rs1 = Input(UInt(p.archRegWidth.W))
        val rs2 = Input(UInt(p.archRegWidth.W))
        val ldst = Input(UInt(p.archRegWidth.W))
        val prs1 = Output(UInt(p.pregWidth.W))
        val prs2 = Output(UInt(p.pregWidth.W))
        val pdstOld = Output(UInt(p.pregWidth.W)) // current (about-to-be-overwritten) mapping of ldst

        // ---- speculative write (rename allocates a new dest mapping) ----
        val allocValid = Input(Bool()) // a writing uop is renamed this cycle
        val allocLdst = Input(UInt(p.archRegWidth.W))
        val allocPdst = Input(UInt(p.pregWidth.W))

        // ---- committed write (retire updates the committed map) ----
        val commitValid = Input(Bool())
        val commitLdst = Input(UInt(p.archRegWidth.W))
        val commitPdst = Input(UInt(p.pregWidth.W))

        // ---- flush ----
        val redirect = Input(Bool())

        // ---- committed map snapshot (consumed by the FreeList to rebuild on redirect) ----
        val committedMap = Output(Vec(p.numArchRegs, UInt(p.pregWidth.W)))
    })

    // Initialize arch reg i -> physical reg i (identity), which is the natural reset state where
    // each architectural register owns its same-numbered physical register; x0 -> zeroPreg.
    val initMap = VecInit((0 until p.numArchRegs).map(i => i.U(p.pregWidth.W)))

    val specMap = RegInit(initMap)
    val commitMap = RegInit(initMap)

    // ---- speculative reads (x0 always reads as zeroPreg) ----
    io.prs1 := Mux(io.rs1 === 0.U, p.zeroPreg.U, specMap(io.rs1))
    io.prs2 := Mux(io.rs2 === 0.U, p.zeroPreg.U, specMap(io.rs2))
    io.pdstOld := Mux(io.ldst === 0.U, p.zeroPreg.U, specMap(io.ldst))

    // ---- committed map update at retire (never touches x0) ----
    when(io.commitValid && io.commitLdst =/= 0.U) {
        commitMap(io.commitLdst) := io.commitPdst
    }

    // ---- speculative map update ----
    when(io.redirect) {
        // Full flush: revert speculative state to the committed map. If a commit also lands this
        // cycle, fold it in so we do not lose the architectural update (commit is older than the
        // flushed speculative work, so it must survive).
        for (i <- 0 until p.numArchRegs) {
            val committed = if (i == 0) p.zeroPreg.U else commitMap(i.U)
            specMap(i) := committed
        }
        when(io.commitValid && io.commitLdst =/= 0.U) {
            specMap(io.commitLdst) := io.commitPdst
        }
    }.elsewhen(io.allocValid && io.allocLdst =/= 0.U) {
        // Normal rename: install the new mapping for the destination architectural register.
        specMap(io.allocLdst) := io.allocPdst
    }

    // x0 mappings are immutable; re-pin every cycle as a safety belt (cheap, avoids any path
    // accidentally writing arch 0).
    specMap(0) := p.zeroPreg.U
    commitMap(0) := p.zeroPreg.U

    // Expose the committed map for the FreeList's redirect rebuild. Reflect a same-cycle commit
    // combinationally so that, if a redirect and a commit land together, the FreeList rebuild
    // sees the just-committed mapping as architecturally live (and frees the old one) rather
    // than missing it by one cycle.
    val committedView = WireInit(commitMap)
    when(io.commitValid && io.commitLdst =/= 0.U) {
        committedView(io.commitLdst) := io.commitPdst
    }
    committedView(0) := p.zeroPreg.U
    io.committedMap := committedView
}