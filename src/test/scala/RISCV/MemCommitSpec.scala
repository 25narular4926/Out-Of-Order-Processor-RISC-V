package RISCV

import chisel3._
import chisel3.util.Decoupled
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.should.Matchers

/**
 * Tests for the memory-ordering + in-order-commit subsystem (Agent C):
 *   - ReorderBuffer: in-order retirement despite out-of-order writeback, and head-generated
 *     redirect on a mispredicted branch.
 *   - Lsu: store-to-load forwarding for a younger load to the same word, and a plain memory
 *     load returning a previously-stored word after commit drains the store.
 *
 * Uses the Chisel 7 ChiselSim PeekPokeAPI (poke / peek / expect / clock.step). All stimulus is
 * deterministic. The LSU test uses a small Chisel test harness that wires the Lsu to a real
 * `Memory` PORT 2 so the full store-commit -> memory -> load read path is exercised.
 */
class MemCommitSpec extends AnyFreeSpec with Matchers with ChiselSim {

    val p = OoOParams()

    // ----------------------------------------------------------------- ROB helpers
    /** Drive every MicroOp field to a benign default, then set the fields the ROB cares about. */
    def pokeAllocUop(
        dut:            ReorderBuffer,
        pc:             Long,
        isBranchOrJump: Boolean = false,
        writesReg:      Boolean = false
    ): Unit = {
        val u = dut.io.allocUop
        u.pc.poke(pc.U)
        u.opcode.poke(0.U); u.func3.poke(0.U); u.func7.poke(0.U)
        u.immediate.poke(0.U); u.fuType.poke(FuType.ALU)
        u.lrs1.poke(0.U); u.lrs2.poke(0.U); u.ldst.poke(0.U)
        u.readsRs1.poke(false.B); u.readsRs2.poke(false.B)
        u.writesReg.poke(writesReg.B)
        // map "isBranchOrJump" onto isBranch so the ROB's isBranchOrJump helper is true
        u.isBranch.poke(isBranchOrJump.B)
        u.isJump.poke(false.B); u.isJalr.poke(false.B)
        u.isLoad.poke(false.B); u.isStore.poke(false.B)
        u.prs1.poke(0.U); u.prs2.poke(0.U); u.pdst.poke(0.U); u.pdstOld.poke(0.U)
        u.prs1Ready.poke(false.B); u.prs2Ready.poke(false.B)
        u.robIdx.poke(0.U); u.ldqIdx.poke(0.U); u.stqIdx.poke(0.U)
        u.predTaken.poke(false.B); u.predTarget.poke(0.U)
        u.brMask.poke(0.U); u.brTag.poke(0.U)
    }

    /** Drive a writeback port to "no-op" defaults. */
    def clearWb(dut: ReorderBuffer, port: Int): Unit = {
        val w = dut.io.wb(port)
        w.valid.poke(false.B); w.robIdx.poke(0.U); w.pdst.poke(0.U)
        w.writesReg.poke(false.B); w.data.poke(0.U)
        w.isBranchResolved.poke(false.B); w.mispredicted.poke(false.B)
        w.brTaken.poke(false.B); w.brTarget.poke(0.U); w.exception.poke(false.B)
    }

    def clearAllWb(dut: ReorderBuffer): Unit =
        for (i <- 0 until p.numWbPorts) clearWb(dut, i)

    "ReorderBuffer retires in program order despite out-of-order writeback" in {
        simulate(new ReorderBuffer(p)) { dut =>
            clearAllWb(dut)
            dut.io.allocReq.poke(false.B)
            dut.clock.step(1)

            // Allocate 3 uops in program order at robIdx 0,1,2.
            val pcs = Seq(0x100L, 0x104L, 0x108L)
            val idxs = scala.collection.mutable.ArrayBuffer[BigInt]()
            for (pc <- pcs) {
                pokeAllocUop(dut, pc, writesReg = true)
                dut.io.allocReq.poke(true.B)
                idxs += dut.io.allocIdx.peek().litValue // combinational tail
                dut.clock.step(1)
            }
            dut.io.allocReq.poke(false.B)
            idxs.toSeq shouldBe Seq(BigInt(0), BigInt(1), BigInt(2))

            // Nothing is done yet -> no commit.
            dut.io.commit.valid.peek().litToBoolean shouldBe false

            // Write back OUT OF ORDER: idx 2 first, then idx 0, then idx 1.
            def wb(idx: Int): Unit = {
                clearAllWb(dut)
                dut.io.wb(0).valid.poke(true.B)
                dut.io.wb(0).robIdx.poke(idx.U)
                dut.io.wb(0).writesReg.poke(true.B)
                dut.clock.step(1)
                clearAllWb(dut)
            }

            // mark idx 2 done -- head (0) still not done, so no commit yet.
            // (`done` is registered: after wb(2)'s step it is set, but the head is idx 0.)
            wb(2)
            dut.io.commit.valid.peek().litToBoolean shouldBe false

            // mark idx 0 done. wb() steps once to register done[0]; now the head (idx 0) is
            // valid && done, so commit is combinationally valid this cycle.
            wb(0)
            dut.io.commit.valid.expect(true.B)
            dut.io.commit.robIdx.expect(0.U)
            dut.io.commit.pc.expect(0x100.U)
            dut.clock.step(1) // retire idx 0; head advances to idx 1

            // head is now idx 1 which is NOT done -> no commit even though idx 2 is done.
            dut.io.commit.valid.peek().litToBoolean shouldBe false

            // mark idx 1 done -> head (idx 1) retires (pc 0x104).
            wb(1)
            dut.io.commit.valid.expect(true.B)
            dut.io.commit.pc.expect(0x104.U)
            dut.clock.step(1) // retire idx 1; head advances to idx 2

            // now head == idx 2, already done from earlier -> commit pc 0x108.
            dut.io.commit.valid.expect(true.B)
            dut.io.commit.pc.expect(0x108.U)
            dut.clock.step(1) // retire idx 2

            // ROB now empty
            dut.io.empty.expect(true.B)
        }
    }

    "ReorderBuffer generates a redirect when a mispredicted branch retires at the head" in {
        simulate(new ReorderBuffer(p)) { dut =>
            clearAllWb(dut)
            dut.io.allocReq.poke(false.B)
            dut.clock.step(1)

            // Allocate a branch at idx 0 and a normal op at idx 1.
            pokeAllocUop(dut, 0x200L, isBranchOrJump = true, writesReg = false)
            dut.io.allocReq.poke(true.B)
            dut.clock.step(1)
            pokeAllocUop(dut, 0x204L, writesReg = true)
            dut.io.allocReq.poke(true.B)
            dut.clock.step(1)
            dut.io.allocReq.poke(false.B)

            // No redirect yet.
            dut.io.redirect.valid.peek().litToBoolean shouldBe false

            // Write back idx 0 as a MISPREDICTED branch with the correct target = 0x300.
            clearAllWb(dut)
            dut.io.wb(0).valid.poke(true.B)
            dut.io.wb(0).robIdx.poke(0.U)
            dut.io.wb(0).isBranchResolved.poke(true.B)
            dut.io.wb(0).mispredicted.poke(true.B)
            dut.io.wb(0).brTaken.poke(true.B)
            dut.io.wb(0).brTarget.poke(0x300.U)
            dut.clock.step(1) // record mispredict + done into entry 0
            clearAllWb(dut)

            // Head (idx 0) is now done & mispredicted -> commit AND redirect this cycle.
            dut.io.commit.valid.expect(true.B)
            dut.io.commit.robIdx.expect(0.U)
            dut.io.redirect.valid.expect(true.B)
            dut.io.redirect.target.expect(0x300.U)
            dut.clock.step(1) // perform retire + full flush

            // After the flush the ROB is empty (the younger idx 1 was dropped).
            dut.io.empty.expect(true.B)
            dut.io.redirect.valid.peek().litToBoolean shouldBe false
        }
    }

    // ----------------------------------------------------------------- LSU harness
    /**
     * Test harness: an Lsu connected to a real Memory PORT 2. Exposes the LSU IO plus a couple
     * of conveniences. Port 1 of the memory is tied off. This lets the store-commit-drain write
     * actually land in SyncReadMem and be read back by a later load.
     */
    class LsuMemHarness(p: OoOParams) extends Module {
        val io = IO(new Bundle {
            // allocation
            val ldqAllocReq = Input(Bool())
            val ldqAllocIdx = Output(UInt(p.ldqIdxWidth.W))
            val stqAllocReq = Input(Bool())
            val stqAllocIdx = Output(UInt(p.stqIdxWidth.W))
            val allocUop    = Input(new MicroOp(p))
            // execute
            val req         = Flipped(Decoupled(new ExecReq(p)))
            val wb          = Output(new WbPort(p))
            // commit + flush + head
            val commit      = Input(new CommitSignal(p))
            val redirect    = Input(new Redirect(p))
            val robHeadIdx  = Input(UInt(p.robIdxWidth.W))
        })

        val lsu = Module(new Lsu(p))
        val mem = Module(new Memory(p))

        // allocation
        lsu.io.ldqAllocReq := io.ldqAllocReq
        lsu.io.stqAllocReq := io.stqAllocReq
        lsu.io.allocUop    := io.allocUop
        io.ldqAllocIdx     := lsu.io.ldqAllocIdx
        io.stqAllocIdx     := lsu.io.stqAllocIdx

        // execute
        lsu.io.req <> io.req
        io.wb := lsu.io.wb

        // commit/flush/head
        lsu.io.commit     := io.commit
        lsu.io.redirect   := io.redirect
        lsu.io.robHeadIdx := io.robHeadIdx

        // memory PORT 2 <-> LSU
        mem.io.address_2    := lsu.io.memAddr
        mem.io.read_2       := lsu.io.memRead
        mem.io.write_2      := lsu.io.memWrite
        mem.io.write_value_2 := lsu.io.memWriteData
        mem.io.write_mask_2 := lsu.io.memWriteMask
        lsu.io.memReadData  := mem.io.read_value_2

        // tie off the unused PORT 1 and buttons
        mem.io.address_1     := 0.U
        mem.io.read_1        := false.B
        mem.io.write_1       := false.B
        mem.io.write_value_1 := 0.U
        mem.io.keys          := 0.U
    }

    // LSU helpers ------------------------------------------------------
    def pokeMemUop(
        u:        MicroOp,
        robIdx:   Int,
        isLoad:   Boolean,
        isStore:  Boolean,
        ldqIdx:   Int = 0,
        stqIdx:   Int = 0,
        func3:    Int = 2,   // word
        imm:      Long = 0,
        pdst:     Int = 0,
        writes:   Boolean = false
    ): Unit = {
        u.pc.poke(0.U)
        u.opcode.poke(0.U); u.func3.poke(func3.U); u.func7.poke(0.U)
        u.immediate.poke(imm.U); u.fuType.poke(FuType.MEM)
        u.lrs1.poke(0.U); u.lrs2.poke(0.U); u.ldst.poke(0.U)
        u.readsRs1.poke(true.B); u.readsRs2.poke(isStore.B)
        u.writesReg.poke(writes.B)
        u.isBranch.poke(false.B); u.isJump.poke(false.B); u.isJalr.poke(false.B)
        u.isLoad.poke(isLoad.B); u.isStore.poke(isStore.B)
        u.prs1.poke(0.U); u.prs2.poke(0.U); u.pdst.poke(pdst.U); u.pdstOld.poke(0.U)
        u.prs1Ready.poke(true.B); u.prs2Ready.poke(true.B)
        u.robIdx.poke(robIdx.U); u.ldqIdx.poke(ldqIdx.U); u.stqIdx.poke(stqIdx.U)
        u.predTaken.poke(false.B); u.predTarget.poke(0.U)
        u.brMask.poke(0.U); u.brTag.poke(0.U)
    }

    def clearLsuCommit(dut: LsuMemHarness): Unit = {
        val c = dut.io.commit
        c.valid.poke(false.B); c.robIdx.poke(0.U); c.pc.poke(0.U); c.ldst.poke(0.U)
        c.pdst.poke(0.U); c.pdstOld.poke(0.U); c.writesReg.poke(false.B)
        c.isStore.poke(false.B); c.isMMIO.poke(false.B)
    }

    def idleReq(dut: LsuMemHarness): Unit = {
        dut.io.req.valid.poke(false.B)
        pokeMemUop(dut.io.req.bits.uop, 0, isLoad = false, isStore = false)
        dut.io.req.bits.rs1Data.poke(0.U)
        dut.io.req.bits.rs2Data.poke(0.U)
    }

    "Lsu forwards a stored word to a younger load at the same address" in {
        simulate(new LsuMemHarness(p)) { dut =>
            // init
            dut.io.ldqAllocReq.poke(false.B)
            dut.io.stqAllocReq.poke(false.B)
            dut.io.redirect.valid.poke(false.B)
            dut.io.redirect.target.poke(0.U)
            dut.io.robHeadIdx.poke(0.U)
            clearLsuCommit(dut)
            idleReq(dut)
            // benign allocUop
            pokeMemUop(dut.io.allocUop, 0, isLoad = false, isStore = false)
            dut.clock.step(1)

            // Dispatch: allocate a STORE at robIdx 0 (stqIdx 0) and a LOAD at robIdx 1 (ldqIdx 0).
            pokeMemUop(dut.io.allocUop, 0, isLoad = false, isStore = true, stqIdx = 0)
            dut.io.stqAllocReq.poke(true.B)
            dut.clock.step(1)
            dut.io.stqAllocReq.poke(false.B)

            pokeMemUop(dut.io.allocUop, 1, isLoad = true, isStore = false, ldqIdx = 0)
            dut.io.ldqAllocReq.poke(true.B)
            dut.clock.step(1)
            dut.io.ldqAllocReq.poke(false.B)

            // Issue the STORE: base=0x40 (byte), imm=0 -> word addr 0x10, data 0xDEADBEEF.
            pokeMemUop(dut.io.req.bits.uop, 0, isLoad = false, isStore = true, stqIdx = 0, func3 = 2)
            dut.io.req.bits.rs1Data.poke(0x40.U)
            dut.io.req.bits.rs2Data.poke(0xDEADBEEFL.U)
            dut.io.req.valid.poke(true.B)
            dut.io.req.ready.expect(true.B) // store issue always accepted
            dut.clock.step(1)
            idleReq(dut)

            // Issue the LOAD to the SAME word (base 0x40). It is younger (robIdx 1 > head 0) and
            // the older store has a known address+data, so it must FORWARD, completing same cycle.
            pokeMemUop(dut.io.req.bits.uop, 1, isLoad = true, isStore = false, ldqIdx = 0,
                       func3 = 2, pdst = 5, writes = true)
            dut.io.req.bits.rs1Data.poke(0x40.U)
            dut.io.req.bits.rs2Data.poke(0.U)
            dut.io.req.valid.poke(true.B)
            // forwarding load is accepted and writes back combinationally this cycle
            dut.io.req.ready.expect(true.B)
            dut.io.wb.valid.expect(true.B)
            dut.io.wb.robIdx.expect(1.U)
            dut.io.wb.pdst.expect(5.U)
            dut.io.wb.data.expect(0xDEADBEEFL.U)
            dut.clock.step(1)
            idleReq(dut)
        }
    }

    "Lsu commits a store to memory and a later load reads it back" in {
        simulate(new LsuMemHarness(p)) { dut =>
            dut.io.ldqAllocReq.poke(false.B)
            dut.io.stqAllocReq.poke(false.B)
            dut.io.redirect.valid.poke(false.B)
            dut.io.redirect.target.poke(0.U)
            dut.io.robHeadIdx.poke(0.U)
            clearLsuCommit(dut)
            idleReq(dut)
            pokeMemUop(dut.io.allocUop, 0, isLoad = false, isStore = false)
            dut.clock.step(1)

            // Allocate a STORE at robIdx 0 / stqIdx 0.
            pokeMemUop(dut.io.allocUop, 0, isLoad = false, isStore = true, stqIdx = 0)
            dut.io.stqAllocReq.poke(true.B)
            dut.clock.step(1)
            dut.io.stqAllocReq.poke(false.B)

            // Issue the store: word addr 0x20 (byte 0x80), data 0x12345678.
            pokeMemUop(dut.io.req.bits.uop, 0, isLoad = false, isStore = true, stqIdx = 0, func3 = 2)
            dut.io.req.bits.rs1Data.poke(0x80.U)
            dut.io.req.bits.rs2Data.poke(0x12345678L.U)
            dut.io.req.valid.poke(true.B)
            dut.clock.step(1)
            idleReq(dut)

            // Commit the store: ROB broadcasts commit for robIdx 0, isStore. This drains the STQ
            // head and drives the memory write this cycle.
            clearLsuCommit(dut)
            dut.io.commit.valid.poke(true.B)
            dut.io.commit.robIdx.poke(0.U)
            dut.io.commit.isStore.poke(true.B)
            dut.io.robHeadIdx.poke(0.U)
            dut.io.req.valid.poke(false.B)
            dut.clock.step(1) // store write committed into memory
            clearLsuCommit(dut)

            // Advance head past the store; allocate + issue a LOAD from the same word (no STQ
            // entry now, so it must READ MEMORY). robHead = 1 so the load (robIdx 1) is at head.
            dut.io.robHeadIdx.poke(1.U)
            pokeMemUop(dut.io.allocUop, 1, isLoad = true, isStore = false, ldqIdx = 0)
            dut.io.ldqAllocReq.poke(true.B)
            dut.clock.step(1)
            dut.io.ldqAllocReq.poke(false.B)

            // Issue the load: base 0x80 -> word 0x20. No older store in STQ -> memory read.
            pokeMemUop(dut.io.req.bits.uop, 1, isLoad = true, isStore = false, ldqIdx = 0,
                       func3 = 2, pdst = 7, writes = true)
            dut.io.req.bits.rs1Data.poke(0x80.U)
            dut.io.req.valid.poke(true.B)
            dut.io.req.ready.expect(true.B) // memory read launches
            dut.clock.step(1) // memory read cycle (SyncReadMem latency)
            idleReq(dut)

            // Next cycle: stage-2 writeback presents the loaded word.
            dut.io.wb.valid.expect(true.B)
            dut.io.wb.robIdx.expect(1.U)
            dut.io.wb.pdst.expect(7.U)
            dut.io.wb.data.expect(0x12345678L.U)
            dut.clock.step(1)
        }
    }

    // ----------------------------------------------------------------- sub-word store tests
    /** Bring the harness up with everything idle. */
    def initLsu(dut: LsuMemHarness): Unit = {
        dut.io.ldqAllocReq.poke(false.B)
        dut.io.stqAllocReq.poke(false.B)
        dut.io.redirect.valid.poke(false.B)
        dut.io.redirect.target.poke(0.U)
        dut.io.robHeadIdx.poke(0.U)
        clearLsuCommit(dut)
        idleReq(dut)
        pokeMemUop(dut.io.allocUop, 0, isLoad = false, isStore = false)
        dut.clock.step(1)
    }

    /** Allocate a store (robIdx/stqIdx) into the STQ. */
    def allocStore(dut: LsuMemHarness, robIdx: Int, stqIdx: Int): Unit = {
        pokeMemUop(dut.io.allocUop, robIdx, isLoad = false, isStore = true, stqIdx = stqIdx)
        dut.io.stqAllocReq.poke(true.B)
        dut.clock.step(1)
        dut.io.stqAllocReq.poke(false.B)
    }

    /** Issue a store: compute its address+data into its STQ slot (no memory access yet). */
    def issueStore(dut: LsuMemHarness, robIdx: Int, stqIdx: Int, func3: Int,
                   byteAddr: Long, data: Long): Unit = {
        pokeMemUop(dut.io.req.bits.uop, robIdx, isLoad = false, isStore = true,
                   stqIdx = stqIdx, func3 = func3)
        dut.io.req.bits.rs1Data.poke(byteAddr.U)
        dut.io.req.bits.rs2Data.poke((data & 0xffffffffL).U(p.xlen.W))
        dut.io.req.valid.poke(true.B)
        dut.clock.step(1)
        idleReq(dut)
    }

    /** Commit the store at the STQ head -> drains it into memory this cycle. */
    def commitStore(dut: LsuMemHarness, robIdx: Int): Unit = {
        clearLsuCommit(dut)
        dut.io.commit.valid.poke(true.B)
        dut.io.commit.robIdx.poke(robIdx.U)
        dut.io.commit.isStore.poke(true.B)
        dut.io.robHeadIdx.poke(robIdx.U)
        dut.io.req.valid.poke(false.B)
        dut.clock.step(1)
        clearLsuCommit(dut)
    }

    /** Issue a load that must go to memory, and return the word it writes back. */
    def loadFromMem(dut: LsuMemHarness, robIdx: Int, func3: Int, byteAddr: Long,
                    pdst: Int = 7): BigInt = {
        dut.io.robHeadIdx.poke(robIdx.U)
        pokeMemUop(dut.io.allocUop, robIdx, isLoad = true, isStore = false, ldqIdx = 0)
        dut.io.ldqAllocReq.poke(true.B)
        dut.clock.step(1)
        dut.io.ldqAllocReq.poke(false.B)

        pokeMemUop(dut.io.req.bits.uop, robIdx, isLoad = true, isStore = false, ldqIdx = 0,
                   func3 = func3, pdst = pdst, writes = true)
        dut.io.req.bits.rs1Data.poke(byteAddr.U)
        dut.io.req.valid.poke(true.B)
        dut.io.req.ready.expect(true.B) // no older store in the STQ -> memory read launches
        dut.clock.step(1)               // SyncReadMem latency
        idleReq(dut)

        dut.io.wb.valid.expect(true.B)
        dut.io.wb.robIdx.expect(robIdx.U)
        val v = dut.io.wb.data.peek().litValue
        dut.clock.step(1)
        v
    }

    // Word 0x30 lives at byte address 0xC0. Byte lanes are little-endian: lane i = byte 0xC0+i.
    "Lsu SB writes one byte and PRESERVES the surrounding three" in {
        simulate(new LsuMemHarness(p)) { dut =>
            initLsu(dut)

            // Two stores: a full word, then a byte into the middle of it.
            allocStore(dut, robIdx = 0, stqIdx = 0)
            allocStore(dut, robIdx = 1, stqIdx = 1)

            // SW 0xAABBCCDD -> word 0x30  (lane0=DD lane1=CC lane2=BB lane3=AA)
            issueStore(dut, robIdx = 0, stqIdx = 0, func3 = 2, byteAddr = 0xC0L, data = 0xAABBCCDDL)
            // SB 0x11 -> byte 0xC1 (lane 1 only)
            issueStore(dut, robIdx = 1, stqIdx = 1, func3 = 0, byteAddr = 0xC1L, data = 0x11L)

            commitStore(dut, robIdx = 0) // drains the SW (mask 1111)
            commitStore(dut, robIdx = 1) // drains the SB (mask 0010) -- must NOT clobber AA/BB/DD

            // The byte store replaced ONLY lane 1: 0xAABBCCDD -> 0xAABB11DD.
            // (The old splice-over-zero implementation produced 0x00001100 here.)
            loadFromMem(dut, robIdx = 2, func3 = 2, byteAddr = 0xC0L) shouldBe BigInt(0xAABB11DDL)
        }
    }

    "Lsu SH writes one halfword and preserves the other" in {
        simulate(new LsuMemHarness(p)) { dut =>
            initLsu(dut)

            allocStore(dut, robIdx = 0, stqIdx = 0)
            allocStore(dut, robIdx = 1, stqIdx = 1)

            // SW 0xAABBCCDD -> word 0x30
            issueStore(dut, robIdx = 0, stqIdx = 0, func3 = 2, byteAddr = 0xC0L, data = 0xAABBCCDDL)
            // SH 0xBEEF -> byte 0xC2 (upper half: lanes 2,3)
            issueStore(dut, robIdx = 1, stqIdx = 1, func3 = 1, byteAddr = 0xC2L, data = 0xBEEFL)

            commitStore(dut, robIdx = 0)
            commitStore(dut, robIdx = 1)

            // Upper half replaced, lower half (CCDD) intact.
            loadFromMem(dut, robIdx = 2, func3 = 2, byteAddr = 0xC0L) shouldBe BigInt(0xBEEFCCDDL)
        }
    }

    "Lsu LBU reads back an individual byte written by SB" in {
        simulate(new LsuMemHarness(p)) { dut =>
            initLsu(dut)

            allocStore(dut, robIdx = 0, stqIdx = 0)
            allocStore(dut, robIdx = 1, stqIdx = 1)

            issueStore(dut, robIdx = 0, stqIdx = 0, func3 = 2, byteAddr = 0xC0L, data = 0xAABBCCDDL)
            issueStore(dut, robIdx = 1, stqIdx = 1, func3 = 0, byteAddr = 0xC2L, data = 0x77L)

            commitStore(dut, robIdx = 0)
            commitStore(dut, robIdx = 1)

            // LBU of byte 0xC2 (lane 2) must be the freshly stored 0x77, zero-extended.
            loadFromMem(dut, robIdx = 2, func3 = 4, byteAddr = 0xC2L) shouldBe BigInt(0x77)
            // and lane 3 is still 0xAA
            loadFromMem(dut, robIdx = 3, func3 = 4, byteAddr = 0xC3L) shouldBe BigInt(0xAA)
        }
    }

    "Lsu refuses to forward a partially-overlapping store to a wider load" in {
        simulate(new LsuMemHarness(p)) { dut =>
            initLsu(dut)

            // Seed the word with a known value (SyncReadMem has no reset, so an unwritten word
            // holds garbage -- we must establish all four lanes to make the check deterministic).
            allocStore(dut, robIdx = 0, stqIdx = 0)
            allocStore(dut, robIdx = 1, stqIdx = 1)
            issueStore(dut, robIdx = 0, stqIdx = 0, func3 = 2, byteAddr = 0xC0L, data = 0xAABBCCDDL)
            // An SB to lane 1 of the same word, which we deliberately leave UNCOMMITTED.
            issueStore(dut, robIdx = 1, stqIdx = 1, func3 = 0, byteAddr = 0xC1L, data = 0x11L)

            commitStore(dut, robIdx = 0) // only the SW drains; the SB is still sitting in the STQ

            // A younger LW of the SAME word. The pending SB covers only lane 1, so it cannot
            // supply the load's other three bytes -> the load must NOT be accepted (it retries).
            dut.io.robHeadIdx.poke(1.U) // SB (rob 1) is now the oldest; the load (rob 2) is younger
            pokeMemUop(dut.io.allocUop, 2, isLoad = true, isStore = false, ldqIdx = 0)
            dut.io.ldqAllocReq.poke(true.B)
            dut.clock.step(1)
            dut.io.ldqAllocReq.poke(false.B)

            pokeMemUop(dut.io.req.bits.uop, 2, isLoad = true, isStore = false, ldqIdx = 0,
                       func3 = 2, pdst = 5, writes = true)
            dut.io.req.bits.rs1Data.poke(0xC0.U)
            dut.io.req.valid.poke(true.B)
            dut.io.req.ready.expect(false.B) // blocked: partial coverage, cannot forward
            dut.io.wb.valid.expect(false.B)
            dut.clock.step(1)
            idleReq(dut)

            // Once the SB commits, the STQ hit disappears and the load reads the merged word.
            commitStore(dut, robIdx = 1)
            loadFromMem(dut, robIdx = 2, func3 = 2, byteAddr = 0xC0L) shouldBe BigInt(0xAABB11DDL)
        }
    }
}