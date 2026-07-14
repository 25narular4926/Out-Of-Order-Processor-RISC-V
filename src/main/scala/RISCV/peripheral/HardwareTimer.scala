package RISCV

import chisel3._
import chisel3.util._

/**
 * Free-running timer, memory-mapped at `p.timerByte` (0x08000004).
 *
 * WHY THIS IS MANDATORY (not a nicety): the doomgeneric RISC-V port implements DG_GetTicksMs and
 * DG_SleepMs by polling this address in a busy-wait loop. If the address read back a constant,
 * the loop condition would never change and Doom would spin there FOREVER. A working timer is
 * the difference between Doom running and Doom hanging on its first frame.
 *
 * INTERFACE COMPATIBILITY
 * -----------------------
 * The `micros` semantics (a 32-bit count of elapsed microseconds, readable at 0x08000004) are
 * chosen to match the memory map of the MIT OpenCompute in-order "Hopper" SoC, because that is
 * the ABI the Doom port was written against. Matching the interface is the whole point -- it is
 * what lets doomgeneric's platform layer run here unmodified. hopper-cpu is MIT licensed.
 * The implementation below is our own.
 *
 * IMPLEMENTATION
 * --------------
 * A microsecond is `clockHz / 1e6` cycles, which is generally NOT an integer (e.g. a 33.33 MHz
 * sim clock). Rather than truncate the divisor -- which makes the timer drift, and drift is
 * exactly the kind of thing that turns into a mysterious Doom timing bug later -- we accumulate
 * error explicitly with a fixed-point phase accumulator:
 *
 *     every cycle:  phase += 1e6
 *     when phase >= clockHz:  phase -= clockHz;  micros += 1
 *
 * This ticks at exactly clockHz/1e6 cycles per microsecond ON AVERAGE, with bounded (sub-tick)
 * error and no long-run drift, for ANY clock frequency -- including ones that are not a whole
 * number of MHz.
 *
 * We also expose the raw `cycles` count. It costs nothing, and in a simulator the exact cycle
 * count is the number you actually want when measuring how long Doom took to render a frame.
 */
class HardwareTimer(p: OoOParams = OoOParams()) extends Module {
    val io = IO(new Bundle {
        val micros = Output(UInt(32.W)) // elapsed microseconds  (the MMIO-visible value)
        val cycles = Output(UInt(64.W)) // elapsed clock cycles  (for simulation measurement)
    })

    require(p.clockHz >= 1000000, "clockHz must be at least 1 MHz to resolve a microsecond")

    private val usPerSec = 1000000

    // Phase accumulator. Must hold clockHz + 1e6 without overflowing.
    private val phaseW = log2Ceil(p.clockHz + usPerSec) + 1

    val phase  = RegInit(0.U(phaseW.W))
    val micros = RegInit(0.U(32.W))
    val cycles = RegInit(0.U(64.W))

    val next = phase + usPerSec.U
    val tick = next >= p.clockHz.U

    phase  := Mux(tick, next - p.clockHz.U, next)
    micros := micros + tick // Bool widens to 0/1
    cycles := cycles + 1.U

    io.micros := micros
    io.cycles := cycles
}
