package RISCV

import chisel3._
import chisel3.util._

/**
 * Register-rename stage. Ties Decode's output to the three rename structures (MapTable,
 * FreeList, BusyTable) and fills the renamed-tag fields of the MicroOp:
 *   prs1, prs2, pdst, pdstOld, prs1Ready, prs2Ready.
 *
 * Per-instruction behaviour (when the uop fires, i.e. in.valid && in.ready):
 *   - prs1 = specMap(lrs1), prs2 = specMap(lrs2)   (x0 reads -> zeroPreg, always ready)
 *   - if writesReg:
 *         pdst    = freelist.pop
 *         pdstOld = specMap(ldst)        (freed at commit)
 *         specMap(ldst) := pdst
 *         busy.set(pdst)
 *     else:
 *         pdst = pdstOld = zeroPreg
 *
 * Stall: if writesReg and the free list is empty there is no destination tag, so we deassert
 * in.ready (cannot rename) until a preg frees up. We also block when the downstream consumer is
 * not ready (out.ready low).
 *
 * Back-end inputs:
 *   - wb       : busy-bit clears (writeback snoop) -- forwarded to the BusyTable.
 *   - commit   : committed-map update + free-list push of pdstOld at retire.
 *   - redirect : full flush -- specMap := commitMap, free list rebuilt from commitMap, busy cleared.
 *
 * Ordering note: structure updates are gated on `fire`, so on a redirect (which forces fire=false)
 * no speculative rename update slips through; the structures' own redirect handling takes over.
 */
class Rename(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val in = Flipped(Decoupled(new MicroOp(p)))
        val out = Decoupled(new MicroOp(p))

        // back-end feedback
        val wb = Input(Vec(p.numWbPorts, new WbPort(p)))
        val commit = Input(new CommitSignal(p))
        val redirect = Input(new Redirect(p))
    })

    val mapTable = Module(new MapTable(p))
    val freeList = Module(new FreeList(p))
    val busyTable = Module(new BusyTable(p))

    val uopIn = io.in.bits
    val writesReg = uopIn.writesReg

    // =====================================================================================
    // Fire condition. We can rename this cycle only if downstream is ready AND, when the uop
    // writes a register, a free physical register is available.
    // =====================================================================================
    val needPreg = writesReg
    val pregAvailable = !freeList.io.empty
    val canRename = !needPreg || pregAvailable

    // Do not rename during a flush; let the structures recover.
    val fire = io.in.valid && io.out.ready && canRename && !io.redirect.valid

    io.in.ready := io.out.ready && canRename && !io.redirect.valid
    io.out.valid := io.in.valid && canRename && !io.redirect.valid

    // =====================================================================================
    // MapTable lookups (combinational).
    // =====================================================================================
    mapTable.io.rs1 := uopIn.lrs1
    mapTable.io.rs2 := uopIn.lrs2
    mapTable.io.ldst := uopIn.ldst

    val prs1 = mapTable.io.prs1
    val prs2 = mapTable.io.prs2
    val pdstOld = Mux(writesReg, mapTable.io.pdstOld, p.zeroPreg.U)
    val pdst = Mux(writesReg, freeList.io.allocTag, p.zeroPreg.U)

    // =====================================================================================
    // BusyTable read for source readiness. Look up the *physical* sources we just mapped.
    // A source the instruction does not actually read is reported ready (it will be ignored
    // downstream, but a clean `true` avoids spurious wakeups).
    // =====================================================================================
    busyTable.io.rs1 := prs1
    busyTable.io.rs2 := prs2
    val prs1Ready = !uopIn.readsRs1 || busyTable.io.rs1Ready
    val prs2Ready = !uopIn.readsRs2 || busyTable.io.rs2Ready

    // =====================================================================================
    // Drive structure updates (only on fire).
    // =====================================================================================
    // MapTable: allocate the new dest mapping; forward commit for the committed map.
    mapTable.io.allocValid := fire && writesReg
    mapTable.io.allocLdst := uopIn.ldst
    mapTable.io.allocPdst := pdst
    mapTable.io.commitValid := io.commit.valid && io.commit.writesReg
    mapTable.io.commitLdst := io.commit.ldst
    mapTable.io.commitPdst := io.commit.pdst
    mapTable.io.redirect := io.redirect.valid

    // FreeList: pop on fire+writesReg; push pdstOld back at commit; rebuild on redirect.
    freeList.io.allocReq := fire && writesReg
    freeList.io.freeReq := io.commit.valid && io.commit.writesReg
    freeList.io.freeTag := io.commit.pdstOld
    freeList.io.redirect := io.redirect.valid
    freeList.io.committedMap := mapTable.io.committedMap

    // BusyTable: set the new dest tag on fire; snoop writebacks; clear all on redirect.
    busyTable.io.setValid := fire && writesReg
    busyTable.io.setTag := pdst
    busyTable.io.wb := io.wb
    busyTable.io.redirect := io.redirect.valid

    // =====================================================================================
    // Assemble the renamed MicroOp (carry through everything Decode filled, add the tags).
    // =====================================================================================
    val outUop = WireInit(uopIn)
    outUop.prs1 := Mux(uopIn.readsRs1, prs1, p.zeroPreg.U)
    outUop.prs2 := Mux(uopIn.readsRs2, prs2, p.zeroPreg.U)
    outUop.pdst := pdst
    outUop.pdstOld := pdstOld
    outUop.prs1Ready := prs1Ready
    outUop.prs2Ready := prs2Ready

    io.out.bits := outUop
}