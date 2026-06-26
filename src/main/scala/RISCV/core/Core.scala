package RISCV

import chisel3._
import chisel3.util._

/**
 * The out-of-order core: stitches the front end, the OoO execution engine, the LSU, and the
 * ROB together. This module owns almost no logic of its own -- it is the integration point
 * the orchestrator wires from the subsystem modules built by the three agents.
 *
 * Datapath (1-wide):
 *
 *   Frontend (Fetch->Decode->Rename->Dispatch)
 *        | dispatchUop / dispatchValid (stamped with rob/ldq/stq indices)
 *        v
 *   +----------------+   alloc   +-----+        +-----+
 *   |  IssueQueue    |<----------|     |        | ROB |<-- alloc (in order)
 *   |  (unified)     |           |     |        +-----+
 *   +----------------+           |     |           ^
 *     | issueAlu/MulDiv/Mem      | LSU |           | wb (CDB)
 *     v                          | LDQ |           |
 *   PhysRegFile read --> ExecReq | STQ |-----------+
 *     |                          +-----+
 *     v        ExecAlu / ExecMulDiv / Lsu --> WbPort[] (Common Data Bus)
 *   broadcast CDB back to: PhysRegFile (write), IssueQueue (wakeup),
 *                          ROB (done + branch resolution), Frontend (busy clear)
 *
 *   ROB head retirement --> CommitSignal (to Frontend rename-commit + LSU store-drain)
 *   ROB head mispredict/exception --> Redirect (global flush + fetch redirect)
 *
 * CDB port convention: wb(0)=ExecAlu, wb(1)=ExecMulDiv, wb(2)=Lsu.
 * PRF read-port convention: 0/1=ALU rs1/rs2, 2/3=MULDIV rs1/rs2, 4/5=MEM rs1/rs2.
 */
class Core(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val execute = Input(Bool())

        // instruction fetch (Main divides by 4 -> Memory port 1, word-addressed)
        val program_memory_adress = Output(UInt(p.xlen.W))
        val program_memory_value = Input(UInt(p.xlen.W))

        // data accesses (word address) -> Memory port 2
        val memory_address = Output(UInt(p.xlen.W))
        val memory_read = Output(Bool())
        val memory_read_value = Input(UInt(p.xlen.W))
        val memory_write = Output(Bool())
        val memory_write_value = Output(UInt(p.xlen.W))
    })

    // ---- subsystem instances ------------------------------------------------
    val frontend = Module(new Frontend(p)) // Agent A
    val iq = Module(new IssueQueue(p)) // Agent B
    val prf = Module(new PhysRegFile(p)) // Agent B
    val execAlu = Module(new ExecAlu(p)) // Agent B
    val execMulDiv = Module(new ExecMulDiv(p)) // Agent B
    val lsu = Module(new Lsu(p)) // Agent C
    val rob = Module(new ReorderBuffer(p)) // Agent C

    // ---- common data bus (writeback) ---------------------------------------
    val cdb = Wire(Vec(p.numWbPorts, new WbPort(p)))
    cdb(0) := execAlu.io.wb
    cdb(1) := execMulDiv.io.wb
    cdb(2) := lsu.io.wb

    iq.io.wb := cdb
    prf.io.wb := cdb
    rob.io.wb := cdb
    frontend.io.wb := cdb

    // ---- front end <-> memory (instruction) ---------------------------------
    frontend.io.enable := io.execute
    io.program_memory_adress := frontend.io.imemAddr
    frontend.io.imemData := io.program_memory_value

    // ---- dispatch fan-out (allocate ROB + IQ + LDQ/STQ atomically) -----------
    val duop = frontend.io.dispatchUop
    val dval = frontend.io.dispatchValid

    rob.io.allocReq := dval
    rob.io.allocUop := duop
    frontend.io.robFull := rob.io.full
    frontend.io.robAllocIdx := rob.io.allocIdx

    iq.io.enq.valid := dval
    iq.io.enq.bits := duop
    frontend.io.iqReady := iq.io.enq.ready

    lsu.io.ldqAllocReq := dval && duop.isLoad
    lsu.io.stqAllocReq := dval && duop.isStore
    lsu.io.allocUop := duop
    frontend.io.ldqFull := lsu.io.ldqFull
    frontend.io.ldqAllocIdx := lsu.io.ldqAllocIdx
    frontend.io.stqFull := lsu.io.stqFull
    frontend.io.stqAllocIdx := lsu.io.stqAllocIdx

    // ---- issue -> PRF read -> functional units -------------------------------
    // ALU
    execAlu.io.req.valid := iq.io.issueAlu.valid
    iq.io.issueAlu.ready := execAlu.io.req.ready
    execAlu.io.req.bits.uop := iq.io.issueAlu.bits
    prf.io.read(0).addr := iq.io.issueAlu.bits.prs1
    prf.io.read(1).addr := iq.io.issueAlu.bits.prs2
    execAlu.io.req.bits.rs1Data := prf.io.read(0).data
    execAlu.io.req.bits.rs2Data := prf.io.read(1).data

    // MUL/DIV
    execMulDiv.io.req.valid := iq.io.issueMulDiv.valid
    iq.io.issueMulDiv.ready := execMulDiv.io.req.ready
    execMulDiv.io.req.bits.uop := iq.io.issueMulDiv.bits
    prf.io.read(2).addr := iq.io.issueMulDiv.bits.prs1
    prf.io.read(3).addr := iq.io.issueMulDiv.bits.prs2
    execMulDiv.io.req.bits.rs1Data := prf.io.read(2).data
    execMulDiv.io.req.bits.rs2Data := prf.io.read(3).data
    execMulDiv.io.redirect := rob.io.redirect

    // MEM (LSU)
    lsu.io.req.valid := iq.io.issueMem.valid
    iq.io.issueMem.ready := lsu.io.req.ready
    lsu.io.req.bits.uop := iq.io.issueMem.bits
    prf.io.read(4).addr := iq.io.issueMem.bits.prs1
    prf.io.read(5).addr := iq.io.issueMem.bits.prs2
    lsu.io.req.bits.rs1Data := prf.io.read(4).data
    lsu.io.req.bits.rs2Data := prf.io.read(5).data

    // ---- commit / redirect ---------------------------------------------------
    frontend.io.commit := rob.io.commit
    lsu.io.commit := rob.io.commit
    lsu.io.robHeadIdx := rob.io.robHeadIdx

    frontend.io.redirect := rob.io.redirect
    iq.io.redirect := rob.io.redirect
    lsu.io.redirect := rob.io.redirect

    // ---- LSU <-> data memory -------------------------------------------------
    io.memory_address := lsu.io.memAddr
    io.memory_read := lsu.io.memRead
    io.memory_write := lsu.io.memWrite
    io.memory_write_value := lsu.io.memWriteData
    lsu.io.memReadData := io.memory_read_value
}

object Core extends App {
    _root_.circt.stage.ChiselStage.emitSystemVerilogFile(
      new Core(),
      firtoolOpts = Array(
        "-disable-all-randomization",
        "-strip-debug-info",
        "-default-layer-specialization=enable"
      ),
      args = Array("--target-dir", "generated")
    )
}
