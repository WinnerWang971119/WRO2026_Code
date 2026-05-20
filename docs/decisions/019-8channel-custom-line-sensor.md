# ADR-019: Custom 8-channel `WROLineArray` replaces vendor `MatrixLineTracer`

**Status:** Implemented (issue #16 — `ba8a89e`)
**Date decided:** 2026-05 (Phase 1 follow-up)

## Rule

The line sensor is `WROLineArray` — a custom 8-channel reflectance array, backed by 1 or 2 MXColor-protocol I2C boards (front + optional rear for bidirectional follow). It exposes the exact three-method surface the vendor `MatrixLineTracer` did — `getError()`, `isOnline()`, `getJunctionType()` — so it drops into `LineFollower` and `Intersection` with no controller-side logic changes.

**Two-layer calibration (load-bearing — do not delete one layer thinking it's redundant with the other):**
- **Layer 1 — firmware-level hardware calibration / illumination.** Each MXColor board's `setLight(...)` command latches LED brightness and triggers internal photoreceptor normalization. Volatile — must be re-applied at every boot. Brightness sourced from `Config::LINE_LED_PWM`.
- **Layer 2 — software per-channel baselines.** `Config::LINE_CALIB_LOW[B][8]` / `LINE_CALIB_HIGH[B][8]` hold per-channel white/dark references. Non-volatile — hand-edited into `Config.h` from `calibrate()` output, committed to source.

The interface is a stable trio of concrete method signatures — **not** a virtual base class. `LineFollower._array` and `Intersection._array` are `WROLineArray*` directly.

## Why

Hardware ships with two physical 8-channel boards (front + rear) for bidirectional line-following — the vendor's 10-channel tracer is a single-board part and doesn't support the rear-board case. The 8-channel boards use the MXColor wire protocol, packing two channel values into each of four CMYK bytes (tens digit + ones digit); the vendor library doesn't expose this as a line array, only as a color sensor, so the unpacking logic has to live in our class.

Non-virtual interface: virtual dispatch costs RAM (vtable per object + vptr per instance), there is no second backend planned, and the swap from MXLineTracer was a one-time compile-time swap. Pay for what you use.

Two-layer calibration: firmware normalization is global (handles ambient light); software baselines handle per-channel variance (LED placement, photodiode tolerance, mounting height). Deleting either layer silently degrades tracking. The header docstring spells this out so a future maintainer doesn't mistake one for redundant.

## How to apply

- `WROLineArray.begin()` runs once in `setup()` AFTER `MiniR4.begin()` (so `Wire1` is alive) and BEFORE `LineFollower.begin()` (which binds onto it).
- `Config::LINE_BOARD_COUNT` is `1` (front-only) or `2` (front + rear); `setReverseActive(bool)` is auto-driven by `LineFollower::setReverse(bool)` when count == 2.
- `calibrate()` is invoked from sketch top-level only (calibration menu in the .ino), sync-blocking for 5-10+ s. MUST NOT be called from any `update()` path.
- Junction classification returns `{0=NONE, 1=LEFT, 2=RIGHT, 3=T_OR_CROSS}` — `UNKNOWN(4)` is never emitted; predicates are mutually exclusive by construction.

## Trigger to revisit

A swap to a different sensor protocol (analog array, different I2C part). Or if a third board configuration shows up (e.g., side sensors). Adding a base-class interface only becomes worthwhile if a *second* backend is on the table.

## See also

- `WRO2026/WROLineArray.h` — implementation + extended commentary
- [ADR-020](./020-junction-pid-freeze.md) — sibling change that depends on stable junction classification
