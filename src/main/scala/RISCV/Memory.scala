package RISCV

import chisel3._
import chisel3.util.experimental.loadMemoryFromFileInline

/**
 * Word-addressed RAM with two ports plus two memory-mapped regions, matching the in-order
 * RISC-V SoC memory map so the same C programs / hex images run unchanged.
 *
 *   port 1  -- instruction fetch (and flashing while the CPU is held)
 *   port 2  -- data accesses from the load/store unit
 *   VGA     -- word addresses >= vgaBaseWord are redirected to the VGA controller
 *   buttons -- a read of buttonAddrWord returns the 4-bit `btns` input instead of RAM
 *
 * BYTE WRITE ENABLES
 * ------------------
 * The storage element is a Vec of four bytes per word rather than a flat UInt(32), so port 2
 * can perform a MASKED write: `write_mask_2` selects which of the four byte lanes are updated.
 * This is what makes SB / SH correct without a read-modify-write: a byte store simply asserts
 * one lane and leaves the other three untouched in the SRAM. (The previous implementation
 * spliced sub-word stores over a zero base word, which silently destroyed the surrounding
 * bytes -- see the LSU header.)
 *
 * Chisel's `Vec.asUInt` places element 0 in the least-significant bits, which is exactly the
 * little-endian byte order RISC-V expects: lane i holds byte (addr*4 + i).
 *
 * Port 1 (fetch / flash) only ever moves whole words, so it writes with an all-ones mask.
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

        val address_vga = Output(UInt(32.W))
        val write_vga = Output(Bool())
        val write_value_vga = Output(UInt(32.W))

        val btns = Input(UInt(4.W))
    })

    // Four byte lanes per word so writes can be masked per byte.
    val memory = SyncReadMem(p.memWords, Vec(4, UInt(8.W)))

    /** Split a 32-bit word into little-endian byte lanes (lane 0 = bits 7:0). */
    private def toLanes(word: UInt): Vec[UInt] =
        VecInit(Seq.tabulate(4)(i => word(8 * i + 7, 8 * i)))

    private val allLanes = Seq.fill(4)(true.B)

    // ---- port 1: instruction fetch / flash (whole words only) ----
    val read_lanes_1 = memory.readWrite(
      io.address_1,
      toLanes(io.write_value_1),
      allLanes,
      io.read_1 || io.write_1,
      io.write_1
    )
    io.read_value_1 := read_lanes_1.asUInt

    // ---- VGA region redirect (word addresses at/above vgaBaseWord) ----
    val is_vga = io.address_2 >= p.vgaBaseWord.U
    io.address_vga := io.address_2 - p.vgaBaseWord.U
    io.write_vga := is_vga && io.write_2
    io.write_value_vga := io.write_value_2

    // ---- port 2: data accesses, with per-byte write enables ----
    val read_lanes_2 = memory.readWrite(
      io.address_2,
      toLanes(io.write_value_2),
      Seq.tabulate(4)(i => io.write_mask_2(i)),
      (io.read_2 || io.write_2) && !is_vga,
      io.write_2
    )
    io.read_value_2 := read_lanes_2.asUInt

    val is_btns = RegInit(false.B)
    is_btns := io.read_2 && io.address_2 === p.buttonAddrWord.U

    when(is_btns) {
        io.read_value_2 := io.btns
    }
}
