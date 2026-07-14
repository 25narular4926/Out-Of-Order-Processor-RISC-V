# Attribution

This is an out-of-order RV32IM core. It is a sibling project to the MIT OpenCompute in-order
RV32I core ("Hopper", <https://github.com/MIT-OpenCompute/hopper-cpu>, MIT licensed), and it
deliberately reuses parts of that project's *interfaces* so that software written for it — most
notably the Doom port at <https://github.com/MIT-OpenCompute/doom-hopper> — runs here unmodified.

This file records what is borrowed and what is original, so the boundary is explicit.

## Borrowed from hopper-cpu (MIT licensed)

**The SoC memory map** (`OoOParams`, `Memory.scala`). These byte addresses are taken verbatim
from Hopper's `MemoryWrapper.scala`:

| Address | Function |
|---|---|
| `0x08000004` | hardware timer, microseconds |
| `0x08000008`–`0x08000033` | keyboard bitmap |
| `0x08000034` | UART TX |
| `0x10000000` | framebuffer base |
| `0x70000000` | debug character out |
| `0x70000008` | debug hex-word out |

This is an ABI, not a design: matching it is the entire point, because it is the contract the
Doom platform layer (`doomgeneric_rvdoom.c`, `debug.c`) was written against. Copying it is what
allows that C to run on this core with no source changes.

`0x70000010` (the `tohost` testbench mailbox) is **not** Hopper's; it is ours.

**Timer semantics.** `HardwareTimer` exposes a 32-bit microsecond counter at `0x08000004` to
match Hopper's `HardwareTimer`. The implementation is our own (a fixed-point phase accumulator
that does not drift on non-integer-MHz clocks, plus a raw cycle counter for simulation).

**Peripherals carried over with the original scaffold.** `Memory.scala`, `Decoder.scala`,
`VGAController.scala`, `UARTRx.scala`, `UARTTx.scala` were ported from the in-order project when
this repository was first scaffolded (see the initial commits); their headers say so. `Memory.scala`
has since been substantially rewritten here (per-byte write enables, full MMIO decode).

## Original to this project

The processor itself. Hopper is an **in-order, pipelined** core with instruction/data caches.
This is an **out-of-order** core with register renaming and speculative execution. Hopper contains
no reorder buffer, no rename structures, no issue queue, and no load/store queues — none of the
following has any counterpart there:

- `ReorderBuffer` — in-order retirement, commit-time full-flush recovery
- `Rename`, `MapTable`, `FreeList`, `BusyTable` — register renaming
- `IssueQueue` — unified out-of-order issue with a BOOM-style age matrix
- `PhysRegFile` — unified physical register file
- `Lsu` — load/store queues, store-to-load forwarding, conservative memory disambiguation
- `Dispatch`, `Fetch` (skid buffer), `BranchPredictor` (gshare/BTB/RAS)
- `ExecAlu`, `ExecMulDiv`, `Core`, `Bundles`, `Params`

Also original:

- Per-byte write enables in `Memory` + the LSU byte-lane store path, and **coverage-checked**
  store-to-load forwarding (a store may only forward if it covers every byte lane the load reads).
- The commit-trace debug port on `Core`.
- Three CPU bug fixes found during Doom bring-up: sub-word stores clobbering neighbouring bytes;
  stores never reporting completion to the ROB (deadlocking any program with a store); and a
  missed-wakeup race where a uop renamed in the same cycle its producer wrote back would wait
  forever.
- The custom `riscv-tests` environment (`sw/isa/riscv_test.h`). The stock environment boots
  through machine-mode CSRs and exits via `ecall`; this core implements neither, so the boot/exit
  scaffolding is ours. The test bodies are unmodified riscv-tests.
- The `tohost` mailbox convention, `SocHarness`, `ProgramSpec`, `IsaSpec`, `crt0.S`, `link.ld`,
  and the build scripts under `sw/`.

## Third-party software

- **riscv-tests** (`sw/riscv-tests/`) — RISC-V International, BSD licensed. Test bodies unmodified.
- **doomgeneric / Doom** — id Software, **GPL**. If Doom sources are vendored into this repo, that
  code remains GPL and must be kept clearly separated from the (permissively licensed) RTL.
  Doom's *game assets* (WAD files) are separately licensed and are not redistributable except for
  the shareware `doom1.wad` / the free `freedoom` WADs.
