# ADR-020: Freeze LineFollower PID output during any intersection

**Status:** Implemented (issue #22 — `fcf5db3`)
**Date decided:** 2026-05 (Phase 2 PR4)

## Rule

When `Intersection.getLastType()` reports any non-NONE junction, `LineFollower.update()` freezes its PID output — it stops issuing wheel-speed corrections and holds the chassis at the prior cruise setpoint until the junction event clears.

## Why

A wide line-array reading at an intersection (where two lines cross) produces an error signal that is meaningless for "lateral offset from the center line" — multiple channels are simultaneously ON, and the weighted-center computation that normally yields a clean error gives a spurious value that yanks the chassis off-line. Without a freeze, the chassis would corkscrew through every intersection.

The freeze is the simplest viable fix. Alternatives — gating individual channels at junctions, switching to a different error formula, or moving the line-follower to encoder-only briefly — all add state that PID gain tuning has to cope with. A clean "ignore PID output during this window" is small, debuggable, and inverts cleanly when the junction window closes.

## How to apply

- `LineFollower.update()` reads `Intersection.getLastType()` at the top of its tick; if non-NONE, skip the PID write but continue to update internal state (so the error history doesn't snap when the freeze releases).
- Cruise setpoint is held by NOT calling `setSpeed` — the prior continuous primitive's setpoint persists in the STM32 until overwritten.
- Junction detection upstream is the `Intersection` module's responsibility; this ADR assumes its event semantics are stable (see [ADR-019](./019-8channel-custom-line-sensor.md)).

## Trigger to revisit

If a future failure mode shows the chassis drifts *during* the freeze (e.g., a long T-intersection where the freeze persists for many cm and natural drift accumulates), introduce a max-freeze-duration timeout and re-engage PID with a soft restart of the error history.

## See also

- [ADR-019](./019-8channel-custom-line-sensor.md) — stable junction classification this depends on
- `WRO2026/LineFollower.cpp` — junction-freeze gate
