# ADR-021: The camera is the sole color sensor — no dedicated `ColorReader` module

**Status:** Pending implementation (Phase 4)
**Date decided:** 2026-05-20

## Rule

The project uses **one** color-classification device: the camera chosen in [ADR-006](./006-mosaic-via-camera.md). That camera handles **both** use cases:

- The 3×3 mosaic frame pattern read (`capturePattern(out[9])`).
- Single-cell color reads, e.g., identifying a cement block's color before delivery (`readSingleColor() → ColorEnum`).

No dedicated `ColorReader.h/.cpp` module will be built. No separate color sensor will be mounted.

## Why

The original Phase 0 architecture sketched a dedicated `ColorReader` (MXColorV3 / HTColV2 / Vernier) alongside `CameraMosaic`. Two reasons to collapse to one:

1. **The camera can do both.** A camera that can classify 9 cells in a 3×3 grid can also classify a single colored block in its field of view — the per-cell sampling logic generalizes to "sample one region" trivially.
2. **One calibration baseline is easier to maintain.** Lighting calibration at the venue is already the dominant operational pain point (see [ADR-006](./006-mosaic-via-camera.md)). One device means one calibration table, one set of baselines to re-bake on-site, and no risk of the two sensors disagreeing on what "yellow" looks like.

Reducing hardware count also reduces I2C bus traffic, one mounting point, one power draw line, and one open decision in the Roadmap (the "Color sensor" row drops from §11 Open Decisions Log).

## How to apply

- The Phase 4 module list collapses from `{CameraMosaic, ColorReader}` to a single camera module. The class may be named `Camera`, `CameraMosaic` (kept for historical continuity), or `CameraColor` — name is an implementation detail, decided when the module is written.
- Any code or doc referencing `ColorReader` is superseded. `Tasks` that need a color read (cement delivery, mosaic tile color verification) call into the camera module directly.
- Camera mounting geometry must accommodate both use cases: framed view of the 3×3 mosaic AND a usable field of view at the cement-block pick point. If the two positions are incompatible, the manipulator carries the block past the camera between pickup and drop.
- Workshop + venue calibration baselines cover all colors {Y, B, G, W} used in either context, in a single `Config::CAMERA_CALIB_*` table.

## Trigger to revisit

- Camera latency on single-cell reads proves too slow for the 2-minute time budget — a dedicated I2C color sensor returns a reading in <10 ms; a camera capture + classify pass may be 100+ ms. If that latency hurts task completion rate in Phase 5/6 testing, add a dedicated sensor with a new ADR.
- A future game (or manipulator design) requires a color read from a position where the camera can't physically see — e.g., an end-effector sensor to confirm grip on the right tile. Add a dedicated sensor at that location.
- Camera classification accuracy on single-cell reads falls below the per-color thresholds in benchmark B4.1 (≥18/20 correct per color). At that point, evaluate whether the camera baseline calibration is at fault or whether the optical setup genuinely can't serve both use cases.

## See also

- [ADR-006](./006-mosaic-via-camera.md) — mosaic pattern reading via camera (the camera selection itself)
- [Roadmap.md](../../Roadmap.md) — Phase 4 (perception)
