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
        // Full keyboard bitmap for scripted input. Doom's DG_GetKey reads KEYTRACKER (0x08000008)
        // and maps set bits to HID codes (bit i in the word at offset w => HID w*32+i). Memory
        // returns this same value for every key word, so bit i triggers HID {i, 32+i, ... 224+i};
        // pick bits whose only mapped HID is the intended key (e.g. bit 8 => HID 0x28 = Enter).
        val key_bits      = Input(UInt(32.W))

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

        // ---- HARDWARE PERFORMANCE COUNTERS + LATCHED TOHOST ----
        // These exist for SIMULATION SPEED, not for the RTL's benefit.
        //
        // Every peek()/step() from Scala is a round-trip across the JVM<->Verilator boundary,
        // costing on the order of a millisecond. A testbench that peeks a few signals every
        // single cycle therefore runs at ~500 cycles/sec -- it measures IPC latency, not
        // simulation speed, and it made a Doom-sized workload (10^8 instructions) look like it
        // would take ~13 days.
        //
        // Counting in hardware instead lets the testbench step in huge batches and read the
        // totals once at the end. Latching tohost (rather than watching for its single-cycle
        // pulse) is what makes that safe: the program's exit code cannot be missed between polls.
        val perf_cycles    = Output(UInt(64.W))
        val perf_commits   = Output(UInt(64.W))
        val perf_redirects = Output(UInt(64.W))
        // stall attribution
        val perf_divbusy      = Output(UInt(64.W)) // cycles the divider FSM was busy
        val perf_headstall    = Output(UInt(64.W)) // cycles the ROB head could not retire
        val perf_headstalldiv = Output(UInt(64.W)) // head stalled AND divider busy (divider-bound)
        // committed-PC range (localises a hang loop). Resettable so a single window can be sampled.
        val pc_range_reset = Input(Bool())
        val pc_min         = Output(UInt(32.W))
        val pc_max         = Output(UInt(32.W))
        val pc_last        = Output(UInt(32.W))
        val tohost_seen    = Output(Bool())  // sticky: set on the first tohost write
        val tohost_data    = Output(UInt(32.W)) // sticky: the value written

        // ---- framebuffer capture: read out the last rendered frame at the end of a run ----
        // Doom's DG_DrawFrame writes fbWidth*fbHeight words to the framebuffer MMIO region. We
        // mirror those writes into a capture RAM so the testbench can scan it out ONCE (one peek
        // per pixel) after the run and dump an image -- rather than peeking every cycle.
        val fb_read_addr  = Input(UInt(32.W))
        val fb_read_data  = Output(UInt(32.W))
        val fb_writes     = Output(UInt(64.W)) // how many fb pixels have been written (progress)
    })

    val memory = Module(new Memory(p))
    val core   = Module(new Core(p))

    core.io.execute := io.execute
    memory.io.keys  := io.btns | io.key_bits

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

    // ---- hardware performance counters (only tick while the CPU is actually running) ----
    val cycles    = RegInit(0.U(64.W))
    val commits   = RegInit(0.U(64.W))
    val redirects = RegInit(0.U(64.W))

    val divbusy      = RegInit(0.U(64.W))
    val headstall    = RegInit(0.U(64.W))
    val headstalldiv = RegInit(0.U(64.W))

    when(io.execute) {
        cycles    := cycles + 1.U
        commits   := commits + core.io.dbg_commit_valid   // Bool widens to 0/1
        redirects := redirects + core.io.dbg_redirect_valid
        divbusy      := divbusy + core.io.dbg_muldiv_busy
        headstall    := headstall + core.io.dbg_head_stalled
        headstalldiv := headstalldiv + (core.io.dbg_head_stalled && core.io.dbg_muldiv_busy)
    }

    io.perf_cycles    := cycles
    io.perf_commits   := commits
    io.perf_redirects := redirects
    io.perf_divbusy      := divbusy
    io.perf_headstall    := headstall
    io.perf_headstalldiv := headstalldiv

    // committed-PC range over the current window (to localise a suspected hang loop)
    val pcMin  = RegInit("hffffffff".U(32.W))
    val pcMax  = RegInit(0.U(32.W))
    val pcLast = RegInit(0.U(32.W))
    when(io.pc_range_reset) {
        pcMin := "hffffffff".U; pcMax := 0.U
    }.elsewhen(io.execute && core.io.dbg_commit_valid) {
        when(core.io.dbg_commit_pc < pcMin) { pcMin := core.io.dbg_commit_pc }
        when(core.io.dbg_commit_pc > pcMax) { pcMax := core.io.dbg_commit_pc }
        pcLast := core.io.dbg_commit_pc
    }
    io.pc_min  := pcMin
    io.pc_max  := pcMax
    io.pc_last := pcLast

    // ---- sticky tohost: the write is a single-cycle pulse, so latch it ----
    val tohostSeen = RegInit(false.B)
    val tohostData = RegInit(0.U(32.W))
    when(memory.io.tohost_valid && !tohostSeen) {
        tohostSeen := true.B
        tohostData := memory.io.tohost_value
    }
    io.tohost_seen := tohostSeen
    io.tohost_data := tohostData

    // ---- framebuffer capture ----
    // Mirror every framebuffer write (Memory routes those out on write_vga, addressed relative to
    // the fb base) into a capture RAM, and count them so a run can report rendering progress.
    //
    // The capture RAM is a `Mem` (COMBINATIONAL read), not a SyncReadMem, on purpose: the testbench
    // scans it out one pixel at a time, and a combinational read lets it read a pixel with just
    // poke-address + peek -- no clock step. That means the readout does NOT advance the CPU, so a
    // frame can be captured without pausing Doom and without it overwriting the frame mid-scan.
    val fbMem    = Mem(p.fbWords, UInt(32.W))
    val fbWrites = RegInit(0.U(64.W))
    when(memory.io.write_vga && (memory.io.address_vga < p.fbWords.U)) {
        fbMem.write(memory.io.address_vga, memory.io.write_value_vga)
        fbWrites := fbWrites + 1.U
    }
    io.fb_read_data := fbMem.read(io.fb_read_addr) // combinational
    io.fb_writes    := fbWrites
}
