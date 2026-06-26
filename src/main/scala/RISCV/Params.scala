package RISCV

import chisel3._
import chisel3.util._

/**
 * Central, parameterized sizing for the out-of-order core.
 *
 * Every structural size lives here so the whole machine can be shrunk to close timing on a
 * small FPGA or grown later. Pass a single `OoOParams` instance down to every module and
 * bundle; modules default to `OoOParams()` for convenience in tests.
 *
 * 1-wide to start (one instruction fetched/decoded/renamed/dispatched/committed per cycle);
 * issue + execute + writeback are out of order. Widths are derived, never hand-counted.
 */
case class OoOParams(
    // --- datapath ---
    xlen: Int = 32, // register / data width (RV32)
    // --- register renaming ---
    numArchRegs: Int = 32, // architectural registers (x0..x31)
    numPhysRegs: Int = 64, // physical registers (>= 32 + max in-flight writers). power of two.
    // --- out-of-order structures ---
    robEntries: Int = 32, // reorder buffer depth = instruction window
    iqEntries: Int = 16, // unified integer+memory issue queue depth
    ldqEntries: Int = 8, // in-flight loads
    stqEntries: Int = 8, // in-flight stores
    maxBranches: Int = 4, // simultaneously-speculated branches = branch-mask width = #snapshots
    numWbPorts: Int = 3, // common-data-bus / writeback ports (ALU, MULDIV, MEM)
    // --- memory ---
    memWords: Int = 4096, // word-addressed data/instruction memory depth
    // --- branch predictor ---
    bhtEntries: Int = 512, // gshare 2-bit counter table
    ghistBits: Int = 9, // global history length (indexes the BHT)
    btbEntries: Int = 16, // branch target buffer
    rasEntries: Int = 4, // return-address stack
    // --- memory-mapped IO (matches the in-order RISC-V SoC memory map) ---
    vgaBaseWord: Int = 0x1000, // word addresses >= this are the VGA framebuffer region
    buttonAddrWord: Int = 0x12c00000 // word read here returns the 4-bit button state
) {
    require(numPhysRegs >= numArchRegs + 1, "need at least one free physical register")
    require(isPow2(numPhysRegs), "numPhysRegs must be a power of two")
    require(robEntries >= 2 && isPow2(robEntries), "robEntries must be a power of two >= 2")
    require(maxBranches >= 1)

    // derived widths --------------------------------------------------------
    def pregWidth: Int = log2Ceil(numPhysRegs) // physical register tag width
    def archRegWidth: Int = log2Ceil(numArchRegs) // architectural register index width (5)
    def robIdxWidth: Int = log2Ceil(robEntries) // reorder buffer index width
    def ldqIdxWidth: Int = log2Ceil(ldqEntries)
    def stqIdxWidth: Int = log2Ceil(stqEntries)
    def iqIdxWidth: Int = log2Ceil(iqEntries)
    def brMaskWidth: Int = maxBranches // one one-hot bit per in-flight branch
    def brTagWidth: Int = log2Ceil(maxBranches)

    // the physical register permanently bound to architectural x0 (reads as zero, never freed)
    def zeroPreg: Int = 0
}