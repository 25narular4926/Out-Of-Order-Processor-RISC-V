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
    memWords: Int = 4096, // word-addressed RAM depth (shared instruction + data)
    // --- branch predictor ---
    bhtEntries: Int = 512, // gshare 2-bit counter table
    ghistBits: Int = 9, // global history length (indexes the BHT)
    btbEntries: Int = 16, // branch target buffer
    rasEntries: Int = 4, // return-address stack
    // --- clock (drives the hardware timer's microsecond tick) ---
    clockHz: Int = 125000000,
    // --- framebuffer geometry (Doom's DG_DrawFrame target) ---
    fbWidth: Int = 320,
    fbHeight: Int = 240,
    // --- memory preload ---
    // Path to a hex image ($readmemh format: one 32-bit word per line, line N == word N). When
    // set, RAM is initialised at elaboration instead of being streamed in through the flash port.
    //
    // This is not a convenience -- it is a hard requirement at Doom scale. The flash port accepts
    // ONE word per clock, and driving it from the testbench costs a JVM<->Verilator round-trip per
    // word (~1 ms). A 24 MB Doom image is ~6M words, so flashing it would take hours before the
    // CPU executed a single instruction. $readmemh loads it in one shot at time zero.
    memInitFile: String = ""
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

    // ======================================================================================
    // MEMORY MAP
    // --------------------------------------------------------------------------------------
    // These are the BYTE addresses used by the Hopper in-order SoC (MemoryWrapper.scala), and
    // by the doomgeneric RISC-V port that targets it (doomgeneric_rvdoom.c / debug.c). Adopting
    // them verbatim means that Doom's platform layer runs against this core with no source
    // edits at all.
    //
    //   RAM              0x00000000 .. memWords*4 - 1
    //   hardware timer   0x08000004   (read: microseconds since reset)
    //   keyboard bitmap  0x08000008 .. 0x08000033
    //   UART TX          0x08000034
    //   framebuffer      0x10000000 .. + fbWidth*fbHeight words
    //   debug char       0x70000000   (write: emit a character to the sim console)
    //   debug num        0x70000008   (write: emit a hex word to the sim console)
    //   tohost           0x70000010   (write: testbench mailbox -- OUR addition, not Hopper's)
    //
    // The core's LSU works in WORD addresses (it computes byteAddr >> 2), so every decode below
    // is exposed as a word address.
    // ======================================================================================
    def timerByte: Int    = 0x08000004
    def keysByte: Int     = 0x08000008
    def keysBytes: Int    = 0x2c // 11 words of key state
    def uartTxByte: Int   = 0x08000034
    def fbBaseByte: Int   = 0x10000000
    def dbgCharByte: Int  = 0x70000000
    def dbgNumByte: Int   = 0x70000008
    def tohostByte: Int   = 0x70000010

    def timerWord: Int    = timerByte / 4
    def keysWord: Int     = keysByte / 4
    def keysWordCount: Int = keysBytes / 4
    def uartTxWord: Int   = uartTxByte / 4
    def fbBaseWord: Int   = fbBaseByte / 4
    def dbgCharWord: Int  = dbgCharByte / 4
    def dbgNumWord: Int   = dbgNumByte / 4
    def tohostWord: Int   = tohostByte / 4

    def fbWords: Int = fbWidth * fbHeight

    require(memWords <= fbBaseWord, "RAM must not overlap the framebuffer MMIO region")
}