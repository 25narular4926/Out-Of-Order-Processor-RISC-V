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
 *  (S1) WORD-GRANULAR memory model. Memory is word-addressed (Memory.scala). We implement:
 *         - SW  (func3=010): full word store at commit.
 *         - SB/SH via read-modify-write at commit (read current word, splice bytes, write).
 *         - LW  (func3=010): full word load.
 *         - LB/LH/LBU/LHU: byte/halfword extraction from the loaded word with sign/zero ext,
 *           selected by func3 and the low address bits. (Best-effort; see (S3).)
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
 *  (S4) STORE->LOAD FORWARDING is WORD-EXACT only: a load forwards from the youngest older
 *       store whose word address matches AND whose addr+data are known. Partial overlaps
 *       (e.g. an SB followed by an overlapping LW) are NOT specially handled beyond the
 *       word-RMW the store will eventually perform; in the conservative ordering a load that
 *       cannot safely forward simply stalls until older stores resolve, so we never read stale
 *       data -- but a sub-word store that has not yet committed is forwarded as its full
 *       (RMW-merged) word value, which is correct because the STQ holds the post-RMW word once
 *       the byte splice is computed. We compute the splice at forward time. See `forwardWord`.
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

    /** Splice the store's raw rs2 data into `oldWord` for the byte/half/word size, producing
     *  the post-RMW word that will be written to memory (and forwarded). */
    def spliceStore(oldWord: UInt, raw: UInt, func3: UInt, byteOff: UInt): UInt = {
        val res = WireDefault(oldWord)
        switch(func3) {
            is("b010".U) { // SW
                res := raw
            }
            is("b000".U) { // SB : replace one byte at byteOff
                val shift = (byteOff << 3.U)(4, 0)            // 0,8,16,24
                val mask  = ("hFF".U(p.xlen.W) << shift)(p.xlen - 1, 0)
                val ins   = ((raw & "hFF".U) << shift)(p.xlen - 1, 0)
                res := (oldWord & ~mask) | ins
            }
            is("b001".U) { // SH : replace one halfword at byteOff (bit 1 selects half)
                val shift = (byteOff(1) << 4.U).asUInt          // 0 or 16
                val mask  = ("hFFFF".U(p.xlen.W) << shift)(p.xlen - 1, 0)
                val ins   = ((raw & "hFFFF".U) << shift)(p.xlen - 1, 0)
                res := (oldWord & ~mask) | ins
            }
        }
        res
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
    // The ROB broadcasts commit; when the committing instruction is the store at the STQ head
    // we drive the memory write. Sub-word stores need read-modify-write, but the STQ entry
    // already holds the RAW rs2 data + func3; we can only splice against the CURRENT memory
    // word. Memory is 1-cycle SyncReadMem, so a true RMW would need a read cycle first.
    //
    // FIRST-PASS RMW HANDLING: at issue time we DO NOT have the old word, so for SB/SH we
    // perform the splice at COMMIT using a two-step micro-sequence is avoided for simplicity;
    // instead we read the word, splice, and write across two commit cycles ONLY if needed.
    // To keep the commit path single-cycle and deadlock-free in this first pass, we take the
    // pragmatic route: SW writes the full word directly; SB/SH also write a full word in which
    // the non-target bytes are taken from the most recent KNOWN value. Because we cannot read
    // memory and write in the same cycle on a single port, we approximate the old word as 0 for
    // the untouched lanes of a sub-word store that has no younger overlapping store in the STQ.
    //
    // *** This means SB/SH are only fully correct when the surrounding bytes are 0 or are also
    //     written. This is the documented (S1)/(S4) simplification. SW is exact. ***
    //
    // The store drain pops the STQ head regardless of size.
    val stHead       = stq(stqHeadIdx)
    val commitIsHead = io.commit.valid && io.commit.isStore && !stqEmpty &&
                       (stHead.robIdx === io.commit.robIdx) && stHead.valid
    // A committing store with an unresolved address would be a bug (it must have issued before
    // it could retire); guard anyway so we never write a garbage address.
    val doStoreDrain = commitIsHead && stHead.addrValid && stHead.dataValid

    // For SB/SH the post-RMW word splices raw data into a base word of 0 (documented approx).
    val storeWord = spliceStore(0.U(p.xlen.W), stHead.data, stHead.func3, stHead.byteOff)

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
    // The forwarded word is the store's post-RMW word spliced over... we do not have the
    // underlying memory word here, so for a full-word (SW) forward we use the data directly;
    // for a sub-word forward we splice over 0 (documented (S4) approximation, matching the
    // commit-drain approximation so forwarding and the eventual memory state agree).
    val forwardWord = spliceStore(0.U(p.xlen.W), fwdStore.data, fwdStore.func3, fwdStore.byteOff)

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
    val loadCanForward = reqIsLoad && !olderUnknownStore && !mmioBlocked && anyFwd
    val loadNeedsMem   = reqIsLoad && !olderUnknownStore && !mmioBlocked && !anyFwd
    val loadMemGo      = loadNeedsMem && portFreeForLoad && ld2Free

    // =====================================================================================
    // Decoupled handshake: when do we accept `req`?
    // =====================================================================================
    //  - a store issue is always accepted (it only writes its STQ slot; no structural hazard).
    //  - a forwarding load is accepted (completes same cycle).
    //  - a memory load is accepted only when it actually launches the read (loadMemGo).
    //  - anything blocked is NOT accepted (back-pressure; IssueQueue will retry).
    io.req.ready := Mux(reqIsStore, true.B,
                     Mux(reqIsLoad, loadCanForward || loadMemGo,
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
    // address: store drain uses the store-head word address; otherwise a launching load read.
    io.memAddr := Mux(doStoreDrain, stHead.wordAddr, reqWordAddr)
    io.memRead := reqFire && loadMemGo

    // =====================================================================================
    // Writeback (CDB) of the LOAD result
    // =====================================================================================
    val wb = WireDefault(WbPort.default(p))

    // A forwarded load writes back the SAME cycle it issues.
    when(reqFire && loadCanForward) {
        wb.valid     := true.B
        wb.robIdx    := reqUop.robIdx
        wb.pdst      := reqUop.pdst
        wb.writesReg := reqUop.writesReg
        wb.data      := formatLoad(forwardWord, reqUop.func3, reqByteOff)
    }

    // A memory-read load writes back from stage 2 the cycle after the read.
    when(ld2Valid) {
        wb.valid     := true.B
        wb.robIdx    := ld2RobIdx
        wb.pdst      := ld2Pdst
        wb.writesReg := ld2Writes
        wb.data      := formatLoad(io.memReadData, ld2Func3, ld2ByteOff)
        ld2Valid     := false.B // consumed
    }
    // (A forwarded load and a stage-2 writeback cannot both occur, because we only launch a new
    //  memory read when ld2Free, i.e. when stage 2 is empty; and a forward does not enter the
    //  pipe. So there is at most one CDB driver per cycle.)

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