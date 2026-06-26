package RISCV

import chisel3._
import _root_.circt.stage.ChiselStage

/**
 * SoC top level: wires the out-of-order Core to Memory and the VGA controller, and provides
 * the flash path used to load a program into instruction memory before the CPU is released.
 * Structure mirrors the in-order RISC-V project's Main so the same hex images / memory map run.
 */
class Main(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val execute = Input(Bool())

        val flash = Input(Bool())
        val flash_address = Input(UInt(32.W))
        val flash_value = Input(UInt(32.W))

        val vga_clk = Input(Clock())
        val hsync = Output(Bool())
        val vsync = Output(Bool())
        val rgb = Output(UInt(12.W))
        val blanking = Output(Bool())

        val btns = Input(UInt(4.W))
    })

    val memory = Module(new Memory(p))
    memory.io.read_1 := true.B
    memory.io.write_1 := false.B
    memory.io.write_value_1 := 0.U
    memory.io.btns := io.btns

    val vga = Module(new VGAController())
    vga.io.address := memory.io.address_vga
    vga.io.write := memory.io.write_vga
    vga.io.write_value := memory.io.write_value_vga
    vga.io.read_clk := io.vga_clk
    io.hsync := vga.io.hsync
    io.vsync := vga.io.vsync
    io.rgb := vga.io.rgb
    io.blanking := vga.io.blanking

    val core = Module(new Core(p))
    core.io.execute := io.execute

    // instruction fetch: byte address -> word address
    memory.io.address_1 := core.io.program_memory_adress / 4.U
    core.io.program_memory_value := memory.io.read_value_1

    // data accesses (word address already)
    memory.io.address_2 := core.io.memory_address
    memory.io.read_2 := core.io.memory_read
    memory.io.write_2 := core.io.memory_write
    memory.io.write_value_2 := core.io.memory_write_value
    core.io.memory_read_value := memory.io.read_value_2

    // flash mode: while the CPU is held (execute low), stream words into instruction memory
    when(!io.execute) {
        when(io.flash) {
            memory.io.read_1 := false.B
            memory.io.write_1 := true.B
            memory.io.address_1 := io.flash_address
            memory.io.write_value_1 := io.flash_value
        }
    }
}

object Main extends App {
    ChiselStage.emitSystemVerilogFile(
      new Main(),
      firtoolOpts = Array(
        "-disable-all-randomization",
        "-strip-debug-info",
        "-default-layer-specialization=enable"
      ),
      args = Array("--target-dir", "generated")
    )
}