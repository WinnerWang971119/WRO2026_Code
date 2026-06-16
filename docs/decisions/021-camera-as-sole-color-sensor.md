# ADR-021: The camera is the sole color sensor — no dedicated `ColorReader` module

**Status:** Implemented (Refined by ADR-037)
**Date decided:** 2026-05-20

## Rule

The project uses **one** color-classification device: the HUE12 camera module
(`Vision.h/.cpp`). That module handles:

- The 4×3 mosaic frame pattern read (`readMosaic()` → latch; `getColor(cell)` for
  each of the 12 cells). See [ADR-006](./006-mosaic-via-camera.md).
- On-device hue classification via a single `Config::HUE_BANDS` table in
  `ConfigVision.h` — White reads are STA-gated (a chromatic hue whose saturation
  is below `VISION_WHITE_STA_MAX` is classified White).

No dedicated `ColorReader.h/.cpp` module will be built. No separate color sensor
will be mounted.

**Single-region / single-block color read:** A guard-bypassed read for a single
cell (e.g., cement-block color ID from a non-mosaic-anchored stop) is
**DEFERRED** — it is not served by `readMosaic`. The strict 12-active-cell guard
in `readMosaic` would fault on a held-block pose where fewer than 12 cells are
visible. A separate API with its own ADR is required if this use case arises.

The old `capturePattern(out[9])` / `readSingleColor()` names from the Phase 0
sketch are **obsolete**; the current API is `readMosaic()` + `getColor(cell)`.

## Why

The original Phase 0 architecture sketched a dedicated `ColorReader` alongside
`CameraMosaic`. Two reasons to collapse to one:

1. **The camera can do both.** A camera that can classify 12 cells in a 4×3 grid
   can also classify a single colored block — the per-cell sampling logic
   generalizes trivially once frame parsing is in place.
2. **One calibration baseline is easier to maintain.** Lighting calibration at
   the venue is already the dominant operational pain point (see
   [ADR-006](./006-mosaic-via-camera.md)). One device means one
   `Config::HUE_BANDS` table, one set of baselines to re-bake on-site, and no
   risk of two sensors disagreeing on what "yellow" looks like.

Reducing hardware count also reduces I2C/UART bus traffic, one mounting point,
one power draw line, and one open decision in the Roadmap.

## How to apply

- The module is named `Vision` (`Vision.h/.cpp`). `Tasks` that need a color read
  call `Vision.getColor(cell)` after `Vision.readMosaic()` succeeds.
- Camera mounting geometry must serve the 4×3 mosaic view. A single-region
  cement-block read from a different stop position is out of scope until a
  separate ADR covers it.
- Workshop + venue calibration baselines cover all colors {Y, B, G, W} in
  `Config::HUE_BANDS` (`ConfigVision.h`).

## Trigger to revisit

- A single-region cement-block color read is needed — write a new ADR for the
  guard-bypassed API.
- Camera classification accuracy on any color falls below the per-color
  thresholds in benchmark B4.1 (≥18/20 correct per color) and adjusting
  `HUE_BANDS` cannot fix it — evaluate whether the optical setup genuinely
  can't serve both use cases.
- A future game position requires color sensing where the camera can't physically
  see — add a dedicated sensor with a new ADR.

## See also

- [ADR-006](./006-mosaic-via-camera.md) — 4×3 mosaic capture via camera (Refined by ADR-037)
- [ADR-037](./037-hue12-vision-capture-once.md) — capture-once latch, UART parser, hue classification; refines this ADR
- [Roadmap.md](../../Roadmap.md) — Phase 3/4 perception
