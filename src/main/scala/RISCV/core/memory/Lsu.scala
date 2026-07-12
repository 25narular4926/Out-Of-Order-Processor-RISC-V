package RISCV

import chisel3._
import chisel3.util._

/**
 * Load/Store Unit (LSU) for the 1-wide BOOM-style out-of-order core.
 *
 * Holds a Load Queue (LDQ) and a Store Queue (STQ). Memory uops are allocated into these
 * queues at Dispatch (in program order), compute their effective address (and, for stores,
 * their data) when their MEM uop ISSUES from the issue queue, and interact with architectural
 * memory under the in-order-commit rule:
 *
 *   - STORES are speculative until they retire. A store computes addr+data at issue but the
 *     actual memory write happens ONLY at commit, draining the oldest store from the STQ head.
 *   - LOADS may read memory speculatively, BUT only after all OLDER stores have known
 *     addresses (conservative memory disambiguation). If an older store with a known address
 *     and known data overlaps the load's word, the load FORWARDS that store's data instead of
 *     reading memory (store-to-load forwarding). Otherwise it reads memory.
 *
 * RECOVERY: on redirect.valid (the ROB head's global flush) every speculative LDQ entry and
 * every not-yet-committed STQ entry is squashed and the queues reset to empty.
 *
 * ===================================================================================
 * FIRST-PASS SIMPLIFICATIONS (documented; correctness-first, performance-later):
 *
 *  (S1) WORD-ADDRESSED memory with PER-BYTE WRITE ENABLES (Memory.scala). We implement:
 *         - SW (func3=010): full word store at commit, mask 1111.
 *         - SB/SH: store at commit with only the touched byte lanes enabled. The SRAM preserves
 *           the untouched bytes, so NO read-modify-write is required and all sizes are EXACT.
 *         - LW  (func3=010): full word load.
 *         - LB/LH/LBU/LHU: byte/halfword extraction from the loaded word with sign/zero ext,
 *           selected by func3 and the low address bits.
 *
 *  (S2) AGE COMPARISON uses robIdx made monotonic against the current ROB head, so a younger
 *       instruction always compares "greater" within the in-flight window (window <= robEntries
 *       guarantees no aliasing). This avoids needing branch masks / snapshots.
 *
 *  (S3) ONE LOAD IN FLIGHT to memory at a time. The load datapath is a 2-stage pipe (issue ->
 *       memory read cycle -> writeback). A new load `req` is only accepted when the load pipe
 *       is free and the load is allowed to proceed. Forwarded loads (hit in STQ) complete in a
 *       single cycle without using the memory port. This is simple and deadlock-free for a
 *       1-wide machine; widening would need multiple MSHRs.
 *
 *  (S4) STORE->LOAD FORWARDING is COVERAGE-CHECKED: a load forwards from the youngest older
 *       store whose word address matches, whose addr+data are known, AND whose byte lanes cover
 *       every lane the load consumes. A partial overlap (e.g. an SB followed by an overlapping
 *       LW) cannot be satisfied, because the load's remaining bytes live in memory and we cannot
 *       read memory and forward in the same cycle. Such a load is simply not accepted and is
 *       retried by the IssueQueue; this is deadlock-free because the blocking store is older and
 *       therefore retires first, after which the load reads the correct memory word.
 *
 *  (S5) SINGLE MEMORY PORT arbitration: commit-store writes ALWAYS win over load reads. If a
 *       commit store and a load both want the port in the same cycle, the load is stalled
 *       (held in the issue handshake / load pipe) and retried. Stores are on the critical
 *       retirement path and must not be delayed.
 * ===================================================================================
 */
class Lsu(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        // ---- allocation from Dispatch (combinational index handout, like the ROB) ----
        val ldqAllocReq = Input(Bool())
        val ldqAllocIdx = Output(UInt(p.ldqIdxWidth.W))
        val ldqFull     = Output(Bool())

        val stqAllocReq = Input(Bool())
        val stqAllocIdx = Output(UInt(p.stqIdxWidth.W))
        val stqFull     = Output(Bool())

        // uop carrying robIdx, isLoad/isStore, ldqIdx/stqIdx already stamped by Dispatch.
        val allocUop = Input(new MicroOp(p))

        // ---- execution of a memory uop issued from the IssueQueue (post-PRF read) ----
        val req = Flipped(Decoupled(new ExecReq(p)))

        // ---- LOAD result writeback onto the CDB ----
        val wb = Output(new WbPort(p))

        // ---- commit-time store drain (driven by ROB.commit) ----
        val commit = Input(new CommitSignal(p))

        // ---- global flush ----
        val redirect = Input(new Redirect(p))

        // ---- ROB head index (for the MMIO-at-head rule) ----
        val robHeadIdx = Input(UInt(p.robIdxWidth.W))

        // ---- single shared data-memory port (orchestrator wires to Memory PORT 2) ----
        val memAddr      = Output(UInt(p.xlen.W)) // WORD address (byte addr / 4)
        val memRead      = Output(Bool())
        val memWrite     = Output(Bool())
        val memWriteData = Output(UInt(p.xlen.W))
        val memWriteMask = Output(UInt(4.W))      // byte enables; bit i writes byte lane i
        val memReadData  = Input(UInt(p.xlen.W))
    })

    // =====================================================================================
    // Queue storage
    // =====================================================================================

    /** One store-queue entry. */
    class StqEntry extends Bundle {
        val valid     = Bool()
        val addrValid = Bool()             // effective address computed (store has issued)
        val dataValid = Bool()             // store data computed (store has issued)
        val wordAddr  = UInt(p.xlen.W)     // effective WORD address (byte addr / 4)
        val byteOff   = UInt(2.W)          // low 2 bits of the byte address (for SB/SH)
        val data      = UInt(p.xlen.W)     // raw rs2 data (pre-splice)
        val func3     = UInt(3.W)          // size selector (000=SB,001=SH,010=SW)
        val robIdx    = UInt(p.robIdxWidth.W)
        val mmio      = Bool()             // address is button/VGA region
    }

    /** One load-queue entry. */
    class LdqEntry extends Bundle {
        val valid     = Bool()
        val robIdx    = UInt(p.robIdxWidth.W)
        // (address is recomputed at issue from rs1Data+imm; we do not persist it here because
        //  in this 1-wide first pass a load executes directly off `req`. The entry exists so
        //  the queue index handout / flush bookkeeping mirrors the STQ and a future design can
        //  track load addresses for store-to-load ordering on the store side.)
    }

    val stq = Reg(Vec(p.stqEntries, new StqEntry))
    val ldq = Reg(Vec(p.ldqEntries, new LdqEntry))

    // circular pointers with an extra wrap bit (same scheme as the ROB) -------------------
    def ptrW(idxW: Int) = idxW + 1
    val stqHead = RegInit(0.U(ptrW(p.stqIdxWidth).W)) // oldest store (next to commit/drain)
    val stqTail = RegInit(0.U(ptrW(p.stqIdxWidth).W)) // next free store slot
    val ldqHead = RegInit(0.U(ptrW(p.ldqIdxWidth).W))
    val ldqTail = RegInit(0.U(ptrW(p.ldqIdxWidth).W))

    val stqHeadIdx = stqHead(p.stqIdxWidth - 1, 0)
    val stqTailIdx = stqTail(p.stqIdxWidth - 1, 0)
    val ldqHeadIdx = ldqHead(p.ldqIdxWidth - 1, 0)
    val ldqTailIdx = ldqTail(p.ldqIdxWidth - 1, 0)

    val stqEmpty = stqHead === stqTail
    val stqFull  = (stqHeadIdx === stqTailIdx) && (stqHead(p.stqIdxWidth) =/= stqTail(p.stqIdxWidth))
    val ldqEmpty = ldqHead === ldqTail
    val ldqFull  = (ldqHeadIdx === ldqTailIdx) && (ldqHead(p.ldqIdxWidth) =/= ldqTail(p.ldqIdxWidth))

    io.stqAllocIdx := stqTailIdx
    io.ldqAllocIdx := ldqTailIdx
    io.stqFull     := stqFull
    io.ldqFull     := ldqFull

    // =====================================================================================
    // Helpers
    // =====================================================================================

    /** True if `addr` (WORD address) is a memory-mapped region that must not be read
     *  speculatively: the button MMIO word, or anything in the VGA framebuffer region. */
    def isMmioWord(wordAddr: UInt): Bool =
        (wordAddr === p.buttonAddrWord.U) || (wordAddr >= p.vgaBaseWord.U)

    /** Age key relative to the ROB head: distance ahead of the oldest in-flight instruction.
     *  Smaller key == older. Valid because the in-flight window <= robEntries, so no aliasing. */
    def ageKey(robIdx: UInt): UInt = robIdx - io.robHeadIdx

    /** Which byte lanes a store of size `func3` at `byteOff` actually writes.
     *  SB -> one lane, SH -> two lanes, SW -> all four. Drives the memory byte-enables, so the
     *  bytes NOT selected are left untouched in the SRAM (no read-modify-write needed). */
    def storeMask(func3: UInt, byteOff: UInt): UInt = {
        val m = WireDefault("b1111".U(4.W))
        switch(func3) {
            is("b000".U) { m := (1.U(4.W) << byteOff)(3, 0) }            // SB
            is("b001".U) { m := Mux(byteOff(1), "b1100".U, "b0011".U) }  // SH
            is("b010".U) { m := "b1111".U }                              // SW
        }
        m
    }

    /** Position the store's raw rs2 data into its byte lanes within the 32-bit word. Lanes that
     *  the mask does not select carry don't-care bits (memory ignores them). */
    def storeData(raw: UInt, func3: UInt, byteOff: UInt): UInt = {
        val shift = (byteOff << 3.U)(4, 0) // 0, 8, 16, 24
        val d = WireDefault(raw)
        switch(func3) {
            is("b000".U) { d := ((raw & "hFF".U) << shift)(p.xlen - 1, 0) }   // SB
            is("b001".U) { d := ((raw & "hFFFF".U) << shift)(p.xlen - 1, 0) } // SH
            is("b010".U) { d := raw }                                          // SW
        }
        d
    }

    /** Which byte lanes a load of size `func3` at `byteOff` actually consumes. Used to decide
     *  whether an older store fully covers the load (and may therefore be forwarded). */
    def loadMask(func3: UInt, byteOff: UInt): UInt = {
        val m = WireDefault("b1111".U(4.W))
        switch(func3) {
            is("b000".U) { m := (1.U(4.W) << byteOff)(3, 0) }            // LB
            is("b100".U) { m := (1.U(4.W) << byteOff)(3, 0) }            // LBU
            is("b001".U) { m := Mux(byteOff(1), "b1100".U, "b0011".U) }  // LH
            is("b101".U) { m := Mux(byteOff(1), "b1100".U, "b0011".U) }  // LHU
            is("b010".U) { m := "b1111".U }                              // LW
        }
        m
    }

    /** Format a loaded word into the destination value per func3 + byte offset (LB/LH/LW + U). */
    def formatLoad(word: UInt, func3: UInt, byteOff: UInt): UInt = {
        val byteSel = (byteOff << 3.U)(4, 0)
        val b       = (word >> byteSel)(7, 0)
        val halfSel = (byteOff(1) << 4.U).asUInt
        val h       = (word >> halfSel)(15, 0)
        val res     = WireDefault(word)
        switch(func3) {
            is("b000".U) { res := Cat(Fill(p.xlen - 8, b(7)), b) }   // LB  (sign-extend byte)
            is("b001".U) { res := Cat(Fill(p.xlen - 16, h(15)), h) } // LH  (sign-extend half)
            is("b010".U) { res := word }                             // LW
            is("b100".U) { res := Cat(0.U((p.xlen - 8).W), b) }      // LBU (zero-extend byte)
            is("b101".U) { res := Cat(0.U((p.xlen - 16).W), h) }     // LHU (zero-extend half)
        }
        res
    }

    // =====================================================================================
    // Allocation (program order, from Dispatch)
    // =====================================================================================
    val doStqAlloc = io.stqAllocReq && !stqFull
    when(doStqAlloc) {
        val e = Wire(new StqEntry)
        e.valid     := true.B
        e.addrValid := false.B
        e.dataValid := false.B
        e.wordAddr  := 0.U
        e.byteOff   := 0.U
        e.data      := 0.U
        e.func3     := 0.U
        e.robIdx    := io.allocUop.robIdx
        e.mmio      := false.B
        stq(stqTailIdx) := e
        stqTail := stqTail + 1.U
    }

    val doLdqAlloc = io.ldqAllocReq && !ldqFull
    when(doLdqAlloc) {
        val e = Wire(new LdqEntry)
        e.valid  := true.B
        e.robIdx := io.allocUop.robIdx
        ldq(ldqTailIdx) := e
        ldqTail := ldqTail + 1.U
    }

    // =====================================================================================
    // Load pipeline state (one in-flight memory load) -- stage 2 (the cycle after the read)
    // =====================================================================================
    val ld2Valid   = RegInit(false.B)
    val ld2RobIdx  = Reg(UInt(p.robIdxWidth.W))
    val ld2Pdst    = Reg(UInt(p.pregWidth.W))
    val ld2Writes  = Reg(Bool())
    val ld2Func3   = Reg(UInt(3.W))
    val ld2ByteOff = Reg(UInt(2.W))

    // =====================================================================================
    // Commit-time STORE drain (highest memory-port priority)
    // =====================================================================================
    // The ROB broadcasts commit; when the committing instruction is the store at the STQ head we
    // drive the memory write. Sub-word stores need NO read-modify-write: the memory has per-byte
    // write enables, so SB/SH simply assert the lanes they touch and the SRAM preserves the rest.
    // The commit path therefore stays single-cycle for every store size, and SB/SH/SW are all
    // exact. The store drain pops the STQ head regardless of size.
    val stHead       = stq(stqHeadIdx)
    val commitIsHead = io.commit.valid && io.commit.isStore && !stqEmpty &&
                       (stHead.robIdx === io.commit.robIdx) && stHead.valid
    // A committing store with an unresolved address would be a bug (it must have issued before
    // it could retire); guard anyway so we never write a garbage address.
    val doStoreDrain = commitIsHead && stHead.addrValid && stHead.dataValid

    val storeWord     = storeData(stHead.data, stHead.func3, stHead.byteOff)
    val storeMaskBits = storeMask(stHead.func3, stHead.byteOff)

    when(doStoreDrain) {
        // pop the store head
        stq(stqHeadIdx).valid := false.B
        stqHead := stqHead + 1.U
    }

    // =====================================================================================
    // LOAD execution off `req`  (issue stage / stage 1)
    // =====================================================================================
    val reqUop      = io.req.bits.uop
    val reqIsLoad   = io.req.valid && reqUop.isLoad
    val reqIsStore  = io.req.valid && reqUop.isStore
    val reqByteAddr = io.req.bits.rs1Data + reqUop.immediate
    val reqWordAddr = (reqByteAddr >> 2).pad(p.xlen)
    val reqByteOff  = reqByteAddr(1, 0)
    val reqMmio     = isMmioWord(reqWordAddr)

    // -- STORE issue: record address + data into its STQ slot (no memory access now) --------
    when(reqIsStore) {
        val si = reqUop.stqIdx
        stq(si).wordAddr  := reqWordAddr
        stq(si).byteOff   := reqByteOff
        stq(si).data      := io.req.bits.rs2Data
        stq(si).func3     := reqUop.func3
        stq(si).mmio      := reqMmio
        stq(si).addrValid := true.B
        stq(si).dataValid := true.B
    }

    // -- LOAD issue: ordering check, forwarding, MMIO-at-head, else memory read -------------

    // (a) conservative ordering: a load may proceed only when EVERY older store has a known
    //     address. Scan the STQ for any valid store that is older (smaller ageKey) than the
    //     load and whose address is not yet resolved.
    val loadAge = ageKey(reqUop.robIdx)
    val olderUnknownStore = VecInit(stq.map { s =>
        s.valid && !s.addrValid && (ageKey(s.robIdx) < loadAge)
    }).asUInt.orR

    // (b) store-to-load forwarding: among older stores with a matching WORD address and known
    //     data, pick the YOUNGEST (largest ageKey that is still < loadAge). We compute, per
    //     entry, whether it is a forwarding candidate and its ageKey, then select the max.
    val fwdHitVec = VecInit(stq.map { s =>
        s.valid && s.addrValid && s.dataValid &&
        (ageKey(s.robIdx) < loadAge) && (s.wordAddr === reqWordAddr)
    })
    val fwdAgeVec = VecInit(stq.zip(fwdHitVec).map { case (s, hit) =>
        Mux(hit, ageKey(s.robIdx), 0.U(p.robIdxWidth.W))
    })
    val anyFwd     = fwdHitVec.asUInt.orR
    // index of the youngest forwarding store (max ageKey among hits)
    val fwdSelAge  = fwdAgeVec.reduce((a, b) => Mux(a >= b, a, b))
    val fwdSelOH   = VecInit(stq.zip(fwdHitVec).map { case (s, hit) =>
        hit && (ageKey(s.robIdx) === fwdSelAge)
    })
    val fwdStore   = Mux1H(fwdSelOH, stq)
    val forwardWord = storeData(fwdStore.data, fwdStore.func3, fwdStore.byteOff)

    // COVERAGE RULE: we may only forward if the selected store writes EVERY byte lane the load
    // consumes. The store's un-written lanes live in memory, and we cannot read memory and
    // forward in the same cycle, so a partial overlap (e.g. an SB followed by an overlapping LW)
    // cannot be satisfied. In that case the load simply does NOT proceed: it is not accepted and
    // the IssueQueue retries it. This is deadlock-free because the blocking store is OLDER, so
    // it retires first; once it drains, its STQ entry clears, the hit disappears, and the load
    // reads the now-correct memory word.
    val reqLoadMask  = loadMask(reqUop.func3, reqByteOff)
    val fwdStoreMask = storeMask(fwdStore.func3, fwdStore.byteOff)
    val fwdCovers    = (reqLoadMask & ~fwdStoreMask) === 0.U

    // (c) MMIO-at-head: an MMIO load must wait until it is the ROB head, then execute once.
    val loadIsHead = reqUop.robIdx === io.robHeadIdx
    val mmioBlocked = reqMmio && !loadIsHead

    // (d) memory-port availability: commit stores win. A non-forwarded load needs the port.
    val portFreeForLoad = !doStoreDrain

    // The load pipe (stage 2) must be free to accept a new memory read.
    val ld2Free = !ld2Valid

    // Decide what the load does this cycle.
    //  - blocked  : cannot proceed (older unknown store, MMIO not at head, port busy, pipe busy)
    //  - forward  : completes THIS cycle via CDB, no memory access, no stage-2 needed
    //  - memread  : drives memRead this cycle; result written back next cycle from stage 2
    // A load forwards only when an older store to the same word covers all of its bytes; it goes
    // to memory only when NO older store hits the word at all. A partial (uncoverable) overlap
    // leaves both false, so the load is not accepted and retries until the store commits.
    val loadCanForward = reqIsLoad && !olderUnknownStore && !mmioBlocked && anyFwd && fwdCovers
    val loadNeedsMem   = reqIsLoad && !olderUnknownStore && !mmioBlocked && !anyFwd
    val loadMemGo      = loadNeedsMem && portFreeForLoad && ld2Free

    // =====================================================================================
    // Decoupled handshake: when do we accept `req`?
    // =====================================================================================
    //  - a store issue is accepted whenever it can report completion on the CDB (see below).
    //  - a forwarding load is accepted (completes same cycle) if the CDB is free.
    //  - a memory load is accepted only when it actually launches the read (loadMemGo).
    //  - anything blocked is NOT accepted (back-pressure; IssueQueue will retry).
    //
    // The LSU has exactly ONE writeback port, and a stage-2 load result MUST take it (it is
    // already in flight and would otherwise be lost). So when stage 2 is presenting a result,
    // no new store or forwarding load may be accepted this cycle.
    // (loadMemGo already implies !ld2Valid, because launching a read requires ld2Free.)
    val wbPortBusy = ld2Valid

    io.req.ready := Mux(reqIsStore, !wbPortBusy,
                     Mux(reqIsLoad, (loadCanForward && !wbPortBusy) || loadMemGo,
                       true.B)) // non-mem uops should never be routed here; accept defensively.
    val reqFire = io.req.valid && io.req.ready

    // launch a memory read for a load
    when(reqFire && loadMemGo) {
        ld2Valid   := true.B
        ld2RobIdx  := reqUop.robIdx
        ld2Pdst    := reqUop.pdst
        ld2Writes  := reqUop.writesReg
        ld2Func3   := reqUop.func3
        ld2ByteOff := reqByteOff
    }

    // =====================================================================================
    // Memory port drive (single port; commit store has priority -- (S5))
    // =====================================================================================
    io.memWrite     := doStoreDrain
    io.memWriteData := storeWord
    // byte enables: only the lanes this store actually writes (0 when not draining).
    io.memWriteMask := Mux(doStoreDrain, storeMaskBits, 0.U(4.W))
    // address: store drain uses the store-head word address; otherwise a launching load read.
    io.memAddr := Mux(doStoreDrain, stHead.wordAddr, reqWordAddr)
    io.memRead := reqFire && loadMemGo

    // =====================================================================================
    // Writeback (CDB) of the LOAD result
    // =====================================================================================
    // Exactly ONE of these may drive the single writeback port in a cycle. The handshake above
    // guarantees that by refusing new work while stage 2 is presenting a result, but we also
    // encode the priority explicitly so the port can never be double-driven.
    val wb = WireDefault(WbPort.default(p))

    when(ld2Valid) {
        // (1) A memory-read load writes back from stage 2, the cycle after the read.
        wb.valid     := true.B
        wb.robIdx    := ld2RobIdx
        wb.pdst      := ld2Pdst
        wb.writesReg := ld2Writes
        wb.data      := formatLoad(io.memReadData, ld2Func3, ld2ByteOff)
        ld2Valid     := false.B // consumed
    }.elsewhen(reqFire && loadCanForward) {
        // (2) A forwarded load completes the SAME cycle it issues -- no memory access.
        wb.valid     := true.B
        wb.robIdx    := reqUop.robIdx
        wb.pdst      := reqUop.pdst
        wb.writesReg := reqUop.writesReg
        wb.data      := formatLoad(forwardWord, reqUop.func3, reqByteOff)
    }.elsewhen(reqFire && reqIsStore) {
        // (3) STORE COMPLETION. A store produces no register value, but it MUST still report to
        // the ROB, otherwise its entry is never marked `done`, the head never retires, and the
        // machine deadlocks the first time a program executes a store. Having computed its
        // address and data into the STQ, the store is architecturally finished at this point --
        // the actual memory write happens later, at commit, when the ROB drains the STQ head.
        // writesReg is false, so the PRF, the IssueQueue wakeup, and the BusyTable all ignore it;
        // only the ROB acts on it.
        wb.valid     := true.B
        wb.robIdx    := reqUop.robIdx
        wb.pdst      := p.zeroPreg.U
        wb.writesReg := false.B
        wb.data      := 0.U
    }

    io.wb := wb

    // Loads are simple here: there is no separate LDQ-head retirement needed because a load
    // produces its register value at execute (above) and its ROB entry retires normally. We
    // still advance the LDQ head opportunistically so the queue does not fill: a load entry is
    // freed when its robIdx becomes older-or-equal to the ROB head (i.e. it has committed).
    // This keeps the LDQ as a pure age/flush bookkeeping structure in the first pass.
    val ldHead = ldq(ldqHeadIdx)
    when(!ldqEmpty && ldHead.valid && (ageKey(ldHead.robIdx) === 0.U) &&
         io.commit.valid && (io.commit.robIdx === ldHead.robIdx)) {
        ldq(ldqHeadIdx).valid := false.B
        ldqHead := ldqHead + 1.U
    }

    // =====================================================================================
    // Flush on redirect: squash speculative LDQ entries and uncommitted STQ entries, reset.
    // =====================================================================================
    // In the first-pass full-flush recovery model the redirect comes from the ROB head, so
    // everything currently in the LSU is younger than (or is) the offending instruction and
    // must be dropped. A store at the head that is being DRAINED this same cycle (doStoreDrain)
    // has already committed and its write has been issued to memory above, so it is safe; the
    // flush below clears the rest. Because the committing store retires in-order before any
    // redirect from a younger entry, a drain and a flush do not target the same store.
    when(io.redirect.valid) {
        stqHead := 0.U
        stqTail := 0.U
        ldqHead := 0.U
        ldqTail := 0.U
        for (i <- 0 until p.stqEntries) { stq(i).valid := false.B }
        for (i <- 0 until p.ldqEntries) { ldq(i).valid := false.B }
        ld2Valid := false.B // drop any in-flight load read result
    }
}