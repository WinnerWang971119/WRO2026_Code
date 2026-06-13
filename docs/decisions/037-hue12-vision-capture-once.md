# ADR-037: HUE12 Vision capture-once latch + on-device hue classification

**Status:** Implemented
**Date decided:** 2026-06 (Phase 3)

## Rule

`Vision::readMosaic()` performs a single one-shot read of the HUE12 camera at a
line-anchored full stop and latches 12 cell colors into internal storage. Per-cell
colors are exposed via pure getters (`getColor(cell)` returning `Config::Color`).
Hue-to-color classification runs entirely on the R4 using a single
`Config::HUE_BANDS` table (`ConfigVision.h`). `readMosaic` is the 4th
sanctioned command-time blocking site — it blocks but spins the registered
`cooperativeTick` callback so OLED/heartbeat/sensors keep running.

## Why

A capture-once/latch model keeps all tick-path code I/O-free: `getColor(cell)` is
a pure array read and cannot stall the cooperative loop. On-device classification
keeps the hue-band table version-controlled alongside the firmware and makes the
sim trivially mirrorable (Python reads the same `HUE_BANDS` constants from the
same `ConfigVision.h` rather than maintaining a diverging table).

The bounded-resync UART parser (`VisionFrame.h`) was written from scratch rather
than using the vendor `MiniR4SmartCamReader` because the vendor driver has four
safety issues confirmed by source inspection: it runs off the end of a non-void
function on a misaligned stream start, overruns a 40-byte static buffer when the
camera sends an unvalidated length byte, false-syncs on `0xAA`-as-data inside a
frame body, and busy-waits with no tick hook — making it impossible to spin
cooperativeTick during a capture.

## How to apply

- Call `readMosaic()` only after `Drive.isIdle()` and after the robot has stopped
  at a line-anchored intersection. Calling it mid-move is undefined.
- `readMosaic()` returns `false` on a parse failure. On a failed read the latch
  retains its prior contents — callers that cache `getColor(cell)` before a retry
  may read stale data. Document this at each call site.
- Frame contract (baud rate, byte count, packing) is defined in `VisionFrame.h`;
  camera-specific constants (baud, emission rate) live in `ConfigVision.h`.
- `Vision.setTickCallback(cooperativeTick)` must be called in `setup()` so the
  blocking drain keeps modules ticking.

## Trigger to revisit

- Bench verification (T8) contradicts the reverse-engineered frame layout, baud
  rate, or emission-rate assumptions — the frame contract in `VisionFrame.h` would
  need revision before any task can trust the read.
- A need emerges for a guard-bypassed single-region color read (e.g. cement-block
  ID from a stopped but non-mosaic-anchored position) — that use case is
  deliberately not served by `readMosaic` (the strict 12-active-cell guard would
  fault on a held-block pose); it would need a separate API with its own ADR.

## See also

- [ADR-003](./003-arduino-style-modules.md) — blocking-module rule; `readMosaic` is the 4th permitted command-time blocking site
- [ADR-006](./006-mosaic-via-camera.md) — line-anchored mosaic capture; mosaic is 4×3 (12 cells), not 3×3
- [ADR-021](./021-camera-as-sole-color-sensor.md) — camera as sole color sensor; updated API surface
- [ADR-030](./030-harness-links-real-sources-ci-enforced.md) — host-link test seam
- [ADR-031](./031-config-split-per-domain.md) — `ConfigVision.h` follows the per-domain config split
