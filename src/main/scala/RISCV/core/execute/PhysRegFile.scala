package RISCV

import chisel3._
import chisel3.util._

/**
 * Unified physical register file for the 1-wide OoO core.
 *
 * `numPhysRegs` entries of `xlen` bits. The physical register at index `zeroPreg` (0) is the
 * architectural-x0 binding: it always reads 0 and is never written.
 *
 * READ PORTS (6, all combinational). The Core wires them by FU class:
 *   read(0), read(1) = ALU    rs1 / rs2
 *   read(2), read(3) = MULDIV rs1 / rs2
 *   read(4), read(5) = MEM    rs1 / rs2
 *
 * WRITE / CDB: every common-data-bus port is snooped. A port writes iff
 *   `valid && writesReg && pdst =/= zeroPreg`.
 *
 * NO WRITE-TO-READ BYPASS (deliberate): reads return the registered value only. This is correct
 * because issue-queue wakeup is 1-cycle-late -- a result broadcast on the CDB at cycle T is not
 * observed for selection until T+1, by which time the value has already landed in `regs` (the
 * write is clocked at the end of T). A same-cycle bypass would therefore be redundant, AND it
 * would create a combinational cycle at the Core level: cdb -> prf.wb -> prf.read(bypass) ->
 * lsu.req operands -> lsu's combinational forwarded-load writeback -> cdb. Reading only the
 * register breaks that loop. If wakeup is ever made speculative (same-cycle), revisit this.
 */
class PhysRegFile(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val read = Vec(
            6,
            new Bundle {
                val addr = Input(UInt(p.pregWidth.W))
                val data = Output(UInt(p.xlen.W))
            }
        )
        val wb = Input(Vec(p.numWbPorts, new WbPort(p)))
    })

    // The register array. Index 0 is reserved for x0 and is never written, but we still keep a
    // slot for it so addressing is uniform; it is initialized to 0 and the read path forces 0.
    val regs = Reg(Vec(p.numPhysRegs, UInt(p.xlen.W)))

    // ---- write port (clocked) ----
    // Lower priority first so the last (highest-indexed) matching port wins, consistent with
    // the combinational bypass below.
    for (port <- io.wb) {
        when(port.valid && port.writesReg && port.pdst =/= p.zeroPreg.U) {
            regs(port.pdst) := port.data
        }
    }

    // ---- read ports (registered value only; no same-cycle bypass -- see header) ----
    for (r <- io.read) {
        r.data := Mux(r.addr === p.zeroPreg.U, 0.U(p.xlen.W), regs(r.addr))
    }
}