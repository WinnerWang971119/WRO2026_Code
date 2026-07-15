# ADR-067: Fault-clear cascades from the recovery entry points down to Drive

**Status:** Implemented
**Date decided:** 2026-07 (Phase 6)

## Rule

Every operator/mission recovery that clears a latched nav fault also clears Drive's OWN latched
fault, via a single cascade point rather than scattered explicit calls. `PathTracker::clearFault()`
and `PathTracker::abort()` each call `Drive.clearFault()`; `Mission::abort()` calls it explicitly
too. Because the BTN_DOWN fault-reset and `Mission::declarePosition()` recover through
`PathTracker::clearFault()`, they inherit the cascade automatically — so `declarePosition` no longer
carries its own explicit `Drive.clearFault()`.

## Why

Drive latches its own fault (`DriveClass::Fault`, e.g. `LINE_NOT_FOUND` from a seek that ran to its
max distance) and freezes its command queue until `Drive.clearFault()` is called — `_dispatchNext`
is a no-op while `Drive.hasFault()` (the fault gate). A Drive fault surfaces up as
`PathTracker::Fault::DRIVE_FAULT` (or `OFF_NODE` on a return seek) through the shared Drive-fault
guard, so the two fault surfaces are almost always latched together. Clearing only the PathTracker
fault left Drive frozen: the very next `goTo()` dispatch went nowhere, the shared guard re-read
`Drive.hasFault()`, and the robot re-tripped `DRIVE_FAULT` — unrecoverable without a reboot.
`Drive.stop()` does not clear `_fault` either. The fix was originally a one-off explicit
`Drive.clearFault()` bolted onto `declarePosition`; making the clear cascade from the tracker's own
`clearFault()`/`abort()` covers every recovery entry point (BTN_DOWN reset, `Mission::abort()`,
`PathTracker::abort()`, `declarePosition`) from one place instead of relying on each caller to
remember the Drive half.

## How to apply

- `PathTracker::clearFault()` and `PathTracker::abort()` call `Drive.clearFault()` as their last
  step. `Drive.clearFault()` is idempotent (a no-op when no fault is latched), so the cascade is
  always safe to fire.
- `Mission::abort()` also calls `Drive.clearFault()` explicitly after its `Drive.stop(true)`, so the
  mission-level halt fully re-arms Drive regardless of `PathTracker::abort()`'s internal ordering.
- `Mission::declarePosition()` clears via `PathTracker.clearFault()` UNCONDITIONALLY (not gated on
  `PathTracker.hasFault()`) so a Drive-side fault is cleared even on the idle-but-Drive-faulted entry
  (a task-issued Drive verb that faulted while PathTracker sat idle). Its former explicit
  `Drive.clearFault()` is removed as redundant.
- Host mocks that shadow `PathTrackerClass` for Mission-layer tests (`mission_mocks/PathTracker.h`)
  mirror the cascade so those tests still prove both faults clear.

## Trigger to revisit

A new recovery path that clears a nav fault WITHOUT routing through `PathTracker::clearFault()` /
`PathTracker::abort()` — it must add its own `Drive.clearFault()` or route through the cascade. Also
revisit if Drive gains a second latch surface (beyond `_fault`) that the queue gate reads.

## See also

- [ADR-027](./027-drive-stop-at-line.md) — `Drive.stopAtLine()` and its `LINE_NOT_FOUND` fault, the
  Drive fault this cascade most often clears
- [ADR-045](./045-imu-yaw-sign-normalization.md) — the single-owner cascade pattern (`YawSource`)
  this mirrors in spirit
- `PathTracker.cpp` `clearFault()` / `abort()`, `Mission.cpp` `abort()` / `declarePosition()`,
  `Drive.cpp` `clearFault()` (the fault gate in `_dispatchNext`)
