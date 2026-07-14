package RISCV

import chisel3._
import chisel3.util._
import chisel3.util.experimental.loadMemoryFromFileInline

/**
 * RAM + memory-mapped I/O, decoding the Hopper SoC memory map (see OoOParams).
 *
 *   port 1  -- instruction fetch (and flashing while the CPU is held). RAM only.
 *   port 2  -- data accesses from the load/store unit. RAM *or* MMIO.
 *
 * MMIO (byte addresses; decoded here as word addresses):
 *   0x08000004  timer      read  -> microseconds since reset  (Doom's DG_SleepMs polls this)
 *   0x08000008  keyboard   read  -> key bitmap words          (stubbed to 0 for now)
 *   0x08000034  UART TX    write -> character out
 *   0x10000000  framebuffer write -> exposed on the VGA port
 *   0x70000000  debug char write -> printf a character to the simulation console
 *   0x70000008  debug num  write -> printf a hex word to the simulation console
 *   0x70000010  tohost     write -> testbench mailbox (our addition; how tests report results)
 *
 * The two debug ports are the reason Doom bring-up is tractable: a Chisel `printf` executes in
 * the Verilator simulation, so Doom's own console output (banner, WAD load, zone init) streams
 * out live while it runs. That is worth more during bring-up than a picture.
 *
 * BYTE WRITE ENABLES
 * ------------------
 * The storage element is four byte lanes per word rather than a flat UInt(32), so port 2 can do
 * a MASKED write: `write_mask_2` selects which lanes are updated. This is what makes SB / SH
 * correct without a read-modify-write -- a byte store asserts one lane and the SRAM preserves
 * the other three. Chisel's `Vec.asUInt` puts element 0 in the least-significant bits, which is
 * exactly the little-endian order RISC-V expects: lane i holds byte (addr*4 + i).
 *
 * Port 1 (fetch/flash) only ever moves whole words, so it writes with an all-ones mask.
 */
class Memory(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val address_1 = Input(UInt(32.W))
        val write_1 = Input(Bool())
        val write_value_1 = Input(UInt(32.W))
        val read_1 = Input(Bool())
        val read_value_1 = Output(UInt(32.W))

        val address_2 = Input(UInt(32.W))
        val write_2 = Input(Bool())
        val write_value_2 = Input(UInt(32.W))
        val write_mask_2 = Input(UInt(4.W)) // byte enables: bit i writes byte lane i
        val read_2 = Input(Bool())
        val read_value_2 = Output(UInt(32.W))

        // framebuffer / VGA write port
        val address_vga = Output(UInt(32.W))
        val write_vga = Output(Bool())
        val write_value_vga = Output(UInt(32.W))

        // testbench mailbox
        val tohost_valid = Output(Bool())
        val tohost_value = Output(UInt(32.W))

        // UART TX character stream
        val uart_valid = Output(Bool())
        val uart_char = Output(UInt(8.W))

        // keyboard bitmap (driven by the testbench / a keyboard peripheral; 0 = nothing pressed)
        val keys = Input(UInt(32.W))
    })

    // Four byte lanes per word so writes can be masked per byte.
    val memory = SyncReadMem(p.memWords, Vec(4, UInt(8.W)))

    /** Split a 32-bit word into little-endian byte lanes (lane 0 = bits 7:0). */
    private def toLanes(word: UInt): Vec[UInt] =
        VecInit(Seq.tabulate(4)(i => word(8 * i + 7, 8 * i)))

    private val allLanes = Seq.fill(4)(true.B)

    // ---- port 1: instruction fetch / flash (RAM only, whole words) ----
    val read_lanes_1 = memory.readWrite(
      io.address_1,
      toLanes(io.write_value_1),
      allLanes,
      io.read_1 || io.write_1,
      io.write_1
    )
    io.read_value_1 := read_lanes_1.asUInt

    // ======================================================================================
    // Port 2 address decode (word addresses)
    // ======================================================================================
    val addr = io.address_2

    val isFb       = (addr >= p.fbBaseWord.U) && (addr < (p.fbBaseWord + p.fbWords).U)
    val isTimer    = addr === p.timerWord.U
    val isKeys     = (addr >= p.keysWord.U) && (addr < (p.keysWord + p.keysWordCount).U)
    val isUartTx   = addr === p.uartTxWord.U
    val isDbgChar  = addr === p.dbgCharWord.U
    val isDbgNum   = addr === p.dbgNumWord.U
    val isTohost   = addr === p.tohostWord.U

    val isMmio = isFb || isTimer || isKeys || isUartTx || isDbgChar || isDbgNum || isTohost

    // ---- peripherals ----
    val timer = Module(new HardwareTimer(p))

    // ---- port 2: RAM access, with per-byte write enables. MMIO never touches the array. ----
    val read_lanes_2 = memory.readWrite(
      addr,
      toLanes(io.write_value_2),
      Seq.tabulate(4)(i => io.write_mask_2(i)),
      (io.read_2 || io.write_2) && !isMmio,
      io.write_2 && !isMmio
    )

    // ---- MMIO reads -------------------------------------------------------------------
    // SyncReadMem returns its data one cycle after the address, so an MMIO read must present its
    // value on the SAME schedule: register the select, then mux on the following cycle. (This is
    // the pattern the original button MMIO used.)
    val rdTimer = RegNext(io.read_2 && isTimer, false.B)
    val rdKeys  = RegNext(io.read_2 && isKeys, false.B)
    val timerHeld = RegNext(timer.io.micros)
    val keysHeld  = RegNext(io.keys)

    io.read_value_2 := read_lanes_2.asUInt
    when(rdTimer) {
        io.read_value_2 := timerHeld
    }.elsewhen(rdKeys) {
        io.read_value_2 := keysHeld
    }

    // ---- MMIO writes ------------------------------------------------------------------
    val doWrite = io.write_2

    io.address_vga     := addr - p.fbBaseWord.U
    io.write_vga       := isFb && doWrite
    io.write_value_vga := io.write_value_2

    io.tohost_valid := isTohost && doWrite
    io.tohost_value := io.write_value_2

    io.uart_valid := isUartTx && doWrite
    io.uart_char  := io.write_value_2(7, 0)

    // Simulation console: these printfs are how we watch Doom boot.
    when(isDbgChar && doWrite) {
        printf("%c", io.write_value_2(7, 0))
    }
    when(isDbgNum && doWrite) {
        printf("0x%x", io.write_value_2)
    }
    when(isUartTx && doWrite) {
        printf("%c", io.write_value_2(7, 0))
    }
}
