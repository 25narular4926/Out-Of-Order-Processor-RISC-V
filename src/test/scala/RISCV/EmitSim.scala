package RISCV

import circt.stage.ChiselStage

/**
 * Emit the full SoC (`SocHarness`: Core + Memory + observability ports) to SystemVerilog for the
 * NATIVE C++ Verilator harness (sim/verilator/). ChiselSim drives Verilator through svsim and does
 * not expose `--threads`; emitting the RTL and verilating it ourselves lets us build a
 * multithreaded model for a ~2-5x wall-clock speedup on a multicore machine.
 *
 * We bake in the Doom-sized memory (16M words) so ONE model runs everything: Doom, the compiled-C
 * programs, and the ISA tests (small programs just use low addresses). No `$readmemh` preload is
 * baked in -- the C++ harness flashes the image through the flash port at native speed (no
 * JVM<->Verilator round-trip, so flashing a Doom image is seconds, not the ChiselSim hours).
 *
 * Run:  sbt "Test/runMain RISCV.EmitSim"   ->  generated/sim/SocHarness.sv
 */
object EmitSim extends App {
    val p = OoOParams(
      memWords        = 16 * 1024 * 1024, // 64 MB: fits Doom and everything smaller
      divBitsPerCycle = 4
    )

    ChiselStage.emitSystemVerilogFile(
      new SocHarness(p),
      firtoolOpts = Array(
        "-disable-all-randomization", // deterministic 0-init (no X); harness flashes what it needs
        "-strip-debug-info",
        "-default-layer-specialization=enable"
      ),
      args = Array("--target-dir", "generated/sim")
    )
}
