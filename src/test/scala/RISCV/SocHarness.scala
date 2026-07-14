package RISCV

import chisel3._

/**
 * Core + Memory + flash port, with the MMIO write port brought out as a "tohost" mailbox and
 * the ROB commit trace exposed.
 *
 * The core has no I/O of its own, so a program reports its result by storing to byte 0x4000
 * (== word 0x1000 == OoOParams.vgaBaseWord). Memory.scala routes any word address at or above
 * vgaBaseWord OUT of the RAM array and onto its MMIO/VGA write port, which this harness surfaces
 * as `tohost_*`. So the store escapes the core and the testbench can see it -- with no changes to
 * the RTL.
 *
 * Shared by ProgramSpec (compiled C) and IsaSpec (the riscv-tests ISA suite).
 */
class SocHarness(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val execute       = Input(Bool())
        val flash         = Input(Bool())
        val flash_address = Input(UInt(32.W))
        val flash_value   = Input(UInt(32.W))
        val btns          = Input(UInt(4.W))

        // tohost mailbox (Memory's MMIO write port)
        val tohost_valid = Output(Bool())
        val tohost_addr  = Output(UInt(32.W))
        val tohost_value = Output(UInt(32.W))

        // commit trace (what the core actually retires, in program order)
        val commit_valid    = Output(Bool())
        val commit_pc       = Output(UInt(32.W))
        val redirect_valid  = Output(Bool())
        val redirect_target = Output(UInt(32.W))
        val dispatch_valid  = Output(Bool())
        val dispatch_pc     = Output(UInt(32.W))
        val rob_empty       = Output(Bool())
        val rob_full        = Output(Bool())
        val rob_head        = Output(UInt(32.W))
    })

    val memory = Module(new Memory(p))
    val core   = Module(new Core(p))

    core.io.execute := io.execute
    memory.io.keys  := io.btns

    // instruction fetch (byte address -> word address)
    memory.io.read_1        := true.B
    memory.io.write_1       := false.B
    memory.io.write_value_1 := 0.U
    memory.io.address_1     := core.io.program_memory_adress / 4.U
    core.io.program_memory_value := memory.io.read_value_1

    // data port
    memory.io.address_2     := core.io.memory_address
    memory.io.read_2        := core.io.memory_read
    memory.io.write_2       := core.io.memory_write
    memory.io.write_value_2 := core.io.memory_write_value
    memory.io.write_mask_2  := core.io.memory_write_mask
    core.io.memory_read_value := memory.io.read_value_2

    // flash while the CPU is held
    when(!io.execute) {
        when(io.flash) {
            memory.io.read_1        := false.B
            memory.io.write_1       := true.B
            memory.io.address_1     := io.flash_address
            memory.io.write_value_1 := io.flash_value
        }
    }

    // tohost is now its own MMIO word (0x70000010), independent of the framebuffer region --
    // so it keeps working no matter how large RAM grows for a Doom-sized image.
    io.tohost_valid := memory.io.tohost_valid
    io.tohost_addr  := p.tohostWord.U
    io.tohost_value := memory.io.tohost_value

    io.commit_valid    := core.io.dbg_commit_valid
    io.commit_pc       := core.io.dbg_commit_pc
    io.redirect_valid  := core.io.dbg_redirect_valid
    io.redirect_target := core.io.dbg_redirect_target
    io.dispatch_valid  := core.io.dbg_dispatch_valid
    io.dispatch_pc     := core.io.dbg_dispatch_pc
    io.rob_empty       := core.io.dbg_rob_empty
    io.rob_full        := core.io.dbg_rob_full
    io.rob_head        := core.io.dbg_rob_head
}
