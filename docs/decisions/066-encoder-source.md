# ADR-066: EncoderSource — all lower-MCU encoder + drive task-done I/O flows through one owner

**Status:** Implemented
**Date decided:** 2026-07 (Phase 2 hardening)

## Rule

All lower-MCU per-motor encoder-degree reads and the drive task-done read go through the single
`EncoderSource` module (`src/util/EncoderSource.{h,cpp}`), which calls the RESULT-surfacing vendor
APIs (`mmL.GetEncoderDegrees` / `mmL.Get_Drive_isTaskDone`) directly and applies one of two
policies at each site:

- **`readDegrees(id, out)`** — surfacing, retry-once; leaves `out` UNTOUCHED on a hard failure. For
  fresh **baseline / arm / calibration** reads that must be accurate.
- **`readDegreesOr(id, fallback)`** — returns the per-motor last-good (else `fallback`) on a hard
  failure. For **continuous completion** checks where a one-tick-stale value is safe.

`resetCache(id)` invalidates the last-good and is called wherever a `resetCounter()` re-zeros a
motor. `isPrevTaskDone()` returns `true` **only** on a `Drive_RESULT::OK` read whose done-bit reports
completion (drive instance `EncoderSource::DRIVE_INSTANCE_ID == 1`); any non-OK read is "not done". A
genuine `0` returned with an OK result is always cached/served as `0`. No module outside
`EncoderSource` calls `MiniR4.M*.getDegrees` / `MiniR4.DriveDC.isPrevTaskDone` / the underlying
`mmL` reads or interprets their RESULT.

## Why

The vendor `MiniR4DC::getDegrees` and `MiniR4DriveDC::isPrevTaskDone` DISCARD the `MMLower` RESULT,
so a lower-MCU comms timeout surfaces as a silent `0` (encoder) or a fabricated `true` (task-done) —
identical to the fabricated-`0.0`-yaw hazard ADR-045 fixed for the IMU. A timed-out encoder read of
`0` corrupts every `abs(now - baseline)` delta the T-shape profile, stop-at-line seek, and wall-push
watchdogs measure; a fabricated task-done bit hands off a move that never finished. The two failure
modes need opposite treatment: a **continuous** completion check can safely coast on the last-good
value for a tick, but a **baseline** read must never seed a move on stale data — a bad
`_profileStartEnc` / `_stopStartEnc` poisons the move for its whole duration. Centralizing both the
RESULT-surfacing and the last-good cache behind one owner (mirroring `YawSource`) keeps the policy
choice explicit at each call site and gives the cache a single invalidation point tied to
`resetCounter`.

## How to apply

- **Continuous completion reads** (safe to coast one tick): `EncoderSource.readDegreesOr(id,
  fallback)`. Sites: `Drive::_updateProfile`, `Drive::_updateStopAtLine`, `Drive::_updateWallStop`
  (push travel), `Drive::getEncL` / `getEncR` (and therefore `LineFollower`'s distance gate +
  ramp-down, which read through them), `ManipBelt::_getDegrees` (the `beltTo` position delta). Pass
  the move's own baseline (`_profileStartEnc` / `_stopStartEnc` / `_wallStartEnc`) as the fallback so
  a cold-cache miss reads zero travel, not a jump.
- **Baseline / arm / calibration reads** (must be accurate): `EncoderSource.readDegrees(id, out)` and
  refuse to arm on `false`. Sites: `Drive::_execProfileMove` distance arm (returns `0x02` → the
  dispatcher faults `DISPATCH_FAILED`), the wall-stop SQUARE→PUSH seed (`_endWallStop(true)` on
  failure), the stop-at-line dispatch seed (sets `rc` → `DISPATCH_FAILED`), and
  `Drive::calibrateMotorBias` (restores the saved bias and bails).
- **Cache invalidation:** `EncoderSource.resetCache(id)` at every `resetCounter()` — `Drive::resetEnc`
  and `calibrateMotorBias` invalidate BOTH drive motors (the drive-base reset re-zeros both);
  `ManipBelt::_resetCounter` invalidates its bound motor.
- **Task-done:** `EncoderSource.isPrevTaskDone()` replaces `MiniR4.DriveDC.isPrevTaskDone()` in
  `Drive::update`.
- **Header:** `EncoderSource.h` pulls only `<stdint.h>` (like `YawSource.h`) so `Drive.cpp` and the
  header-only `ManipBelt.h` include it cheaply; the MatrixMiniR4 dependency lives in the `.cpp`.

## Trigger to revisit

- A vendor library update where `MiniR4DC::getDegrees` / `isPrevTaskDone` begin surfacing the RESULT
  themselves — the direct `mmL` calls could then be dropped for the wrapped forms.
- Adding a plausibility gate that rejects corrupted-but-complete UART frames (explicitly out of scope
  for #141) — it would extend `EncoderSource`, not add a parallel checker.
- A second encoder consumer that needs a different staleness policy — extend `EncoderSource` rather
  than reading `mmL` directly.

## See also

- [ADR-045](./045-imu-yaw-sign-normalization.md) — the `YawSource` single-owner / RESULT-surfacing /
  leave-out-param-untouched-on-failure pattern this ADR mirrors for encoders.
- [ADR-027](./027-stop-at-line-seek.md) — the stop-at-line seek whose baseline + no-progress
  watchdog reads route through `EncoderSource`.
- [ADR-004](./004-pid-stays-on-stm32.md) — motor position/speed PID stays on the STM32; the R4 only
  reads the encoder degrees this module surfaces.
