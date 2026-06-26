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
 * Ported from the in-order project; sizing is taken from OoOParams.
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
        val read_2 = Input(Bool())
        val read_value_2 = Output(UInt(32.W))

        val address_vga = Output(UInt(32.W))
        val write_vga = Output(Bool())
        val write_value_vga = Output(UInt(32.W))

        val btns = Input(UInt(4.W))
    })

    val memory = SyncReadMem(p.memWords, UInt(32.W))

    io.read_value_1 := memory.readWrite(
      io.address_1,
      io.write_value_1,
      io.read_1 || io.write_1,
      io.write_1
    )

    val is_vga = io.address_2 >= p.vgaBaseWord.U
    io.address_vga := io.address_2 - p.vgaBaseWord.U
    io.write_vga := is_vga && io.write_2
    io.write_value_vga := io.write_value_2

    io.read_value_2 := memory.readWrite(
      io.address_2,
      io.write_value_2,
      (io.read_2 || io.write_2) && !is_vga,
      io.write_2
    )

    val is_btns = RegInit(false.B)
    is_btns := io.read_2 && io.address_2 === p.buttonAddrWord.U

    when(is_btns) {
        io.read_value_2 := io.btns
    }
}