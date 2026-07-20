# CPU FPGA-Readiness Roadmap

**Goal:** make the RTL **drop-in ready for FPGA** — everything the CPU needs to run (and to play
Doom in real time) is either synthesizable RTL that goes on the chip, or a **sim model of a
physical component** that gets swapped for real IP on hardware without touching the core logic.

**North star:** real-time, playable Doom on the FPGA (CPU + memory + display + input). The GPU is
**tabled** for now — Doom is a software renderer and does not need it; it's an additive capability
for later.

---

## The principle: model every external piece at its interface

Each physical/external component sits behind a **hardware-accurate interface**. In simulation the
interface is filled by a *model*; on FPGA it's filled by *real IP*. A single `target` switch
(`Sim` | `Fpga`) selects which implementation the top-level instantiates — so going to hardware is
a config flip, not a rewrite.

```
target = Sim   → ModeledRam, DisplaySink, InputModel, LoaderModel   (sim/ folder)
target = Fpga  → DdrAxiBridge, HdmiPhy, KeyboardCtrl, SdLoader        (fpga/ folder, later)
             ↑ both implement the SAME interfaces defined with the core ↑
```

### Category discipline (where files live)
| Category | Location | On FPGA |
|---|---|---|
| Synthesizable core RTL (CPU, caches, controllers) | `core/`, main tree | goes on chip |
| Sim-only models (`ModeledRam`, display sink, loader) | `sim/` | replaced by real IP |
| Interfaces / bundles (`MemReq`/`MemResp`, display, input, loader) | shared, with the core | kept — real IP implements them |

---

## What is "external" (modeled now, real later)

| Piece | Sim model | Real IP on FPGA | Interface |
|---|---|---|---|
| External DRAM (64 MB image) | `ModeledRam` → deepen to `fake_ddr3` | MIG / LiteDRAM over AXI | `MemReq`/`MemResp` |
| Display output | `DisplaySink` (scan-out → frame capture) | HDMI/VGA PHY | pixel stream + sync |
| Input (keyboard) | `InputModel` (scripted/poked) | PS/2 or USB controller | key bitmap |
| Program/boot load | `LoaderModel` (preload into DRAM) | SD / UART / JTAG bootloader | load port |
| Console/debug | `printf` (today) | real UART TX | byte stream |
| Clocking / reset | single sim clock | PLL/MMCM + reset sync | clock/reset |

## CPU-logic work needed for FPGA (synthesizable, stays)

- **Latency-tolerant Fetch** — consume a response-valid instead of assuming 1-cycle memory.
- **Latency-tolerant LSU** — wait for `resp.valid`; "one load in flight" → "one *miss* outstanding"
  (blocking first, non-blocking/MSHR later).
- **I-cache and D-cache** (BRAM-modeled) — turn cache *hits* back into ~1 cycle so the machine is
  usable against multi-cycle DRAM.
- **The `target` config switch** and a top-level board SoC that wires core + models (or real IP).

---

## Phases

**Phase 0 — the drop-in contract (structure).**
Freeze the interfaces (memory done; add display/input/loader/UART). Add the `target` switch and a
top-level `Soc`/`Board` skeleton. Establish the `sim/` folder discipline. *Low-risk scaffolding;
does not touch the working core.*

**Phase 1 — realistic memory + caches (the big one).**
Deepen `ModeledRam` toward `fake_ddr3` (variable latency, bursts, refresh). Make Fetch and the LSU
latency-tolerant. Add I/D caches. **Gate:** the ISA suite (48/48) and Doom still pass; measure the
cache's value with the perf counters (the latency-sweep table). *This is the dominant FPGA change.*

**Phase 2 — display + input + loader + UART models.**
Promote the framebuffer to a real arbitrated `FrameBuffer`. Add a display-controller model that
scans it out at a pixel clock to a frame sink (replaces the testbench-only capture). Add input and
runtime-loader models, and a real UART. **Gate:** Doom renders through the modeled display and
responds to modeled input.

**Phase 3 — clock domains + FPGA-readiness pass.**
Model the multiple clock domains (core / memory / pixel) and their crossings. Stub the real-IP
wrappers (`fpga/`) so `target = Fpga` elaborates. Prepare synthesis (constraints/XDC, top-level).
**Gate:** `target = Fpga` builds; `target = Sim` still runs Doom.

---

## The residual that is *irreducibly* FPGA-only

Modeling gets us ~90% de-risked. These three cannot be simulated away and are the real on-board work:
1. **Timing closure / Fmax** — synthesis-only (watch the divider, age matrix, 6-read regfile).
2. **Clock-domain crossings** — deterministic sim hides metastability; CDC is verified on hardware.
3. **Vendor-IP bring-up** — MIG DDR calibration, HDMI PHY init.

---

## First concrete tasks (Phase 0 → Phase 1)

1. `target` config + top-level Sim SoC skeleton (Phase 0).
2. **Latency-tolerant Fetch** against `ModeledRam` — then observe IPC crater with no cache.
3. **Direct-mapped I-cache** — recover the IPC; run the latency-sweep experiment.
4. Repeat for the **LSU + D-cache**.
5. Re-verify: ISA 48/48 + Doom, with the perf counters quantifying each step.
