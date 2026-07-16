# DOOM gameplay capture — OoO RISC-V core

DOOM (shareware `doom1.wad`) running on the out-of-order RV32IM core, captured from
`DoomSpec`. Scripted keyboard input (via the `key_bits` harness port) drives the game from the
title screen into live gameplay on **E1M1 (Hangar)**, then holds *Forward* to walk.

## Contents

- `frames/frame_0000.png … frame_0057.png` — 58 captured framebuffer frames (320×240), scanned
  out of the simulator's framebuffer-capture RAM and written straight to PNG. These are the
  core's own framebuffer writes — not an emulator's output.
- `doom_gameplay.mp4` — the frames stitched into a video (H.264, 640×480 nearest-neighbour
  upscale, 8 fps, ~7 s). Plays anywhere.
- `doom_gameplay.html` — self-contained animation player (all 58 frames embedded as base64;
  runs offline, no assets required). Play/pause, timeline scrub, adjustable FPS, phase markers.

## Scripted-input schedule (frames)

| Frame | Input      | Result                    |
|-------|------------|---------------------------|
| 3     | Enter      | Title → main menu         |
| 10    | Enter      | New Game → episode select |
| 17    | Enter      | Episode 1 (Knee-Deep)     |
| 24    | Enter      | Skill → drop into E1M1    |
| 30+   | Forward (W)| Walk forward in the level |

## How it was produced

```bash
sbt -Ddoom.clockHz=1000000 -Ddoom.cycles=500000000 \
    -Ddoom.frames=60 -Ddoom.frameStride=1 -Ddoom.moveFromFrame=30 \
    "testOnly RISCV.DoomSpec"
```

Measured in-game performance (render loop): **IPC 0.79**, branch mispredict **2.5%**,
divider busy **0.2%**. The core passes **48/48** rv32ui+rv32um ISA tests. See the repo
`CLAUDE.md` §7 for the full Doom build/port details.
