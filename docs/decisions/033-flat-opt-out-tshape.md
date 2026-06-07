# ADR-033: `.flat()` is the named opt-out from the default trapezoidal profile

**Status:** Implemented
**Date decided:** 2026-06 (Phase 3)

## Rule

Drive distance/time builders and LineFollower's enable builder expose `.flat()` to opt OUT of the trapezoidal velocity profile. The move (or follow session) then runs at constant cruise PWM with no accel/decel ramps.

- `DriveDistBuilder::flat()` / `DriveTimeBuilder::flat()` — clear the single `tShape` flag, so the builder enqueues a non-profiled `DRIVE_DIST` / `DRIVE_TIME` instead of `PROFILE_*`.
- `LineFollowerEnableBuilder::flat()` — sugar that clears BOTH ramp flags (`_tShapeRampUp` and `_tShapeRampDown`) in one call.

`.flat()` and the ramp setters (`setStartSpeed`/`setEndSpeed`/`setAccel`, and on LineFollower `tShape()`/`setAccel`) are mutually exclusive: those setters imply `tShape()`, so chaining one AFTER `.flat()` re-enables the profile. Last writer wins.

## Why

ADR-017 made SYNC + tShape the default and deliberately left the opt-out as the verbose `.setStartSpeed(cruise).setEndSpeed(cruise)` short-circuit, on the theory that an explicit flat profile was rare. It came up often enough to be worth a first-class name — exactly the condition ADR-017's trigger-to-revisit named ("add the affordance with a clear name").

The name is `.flat()`, not `.tShape(false)`: a boolean overload invites the self-contradictory `.tShape().tShape(false)` chain, whereas `.flat()` reads as a single unambiguous intent. The engine already routed `tShape == false` to the non-profiled path, so no motion-control change was needed — this is purely an API affordance.

## How to apply

- Reach for `.flat()` when you want constant cruise: a deliberately flat short pulse, a calibration move, or a segment where ramp-up/ramp-down distance would distort a short hop.
- It is a no-op effect below `Config::TSHAPE_MIN_MOVE_CM` / `TSHAPE_MIN_MOVE_SEC` — the profiled path already degrades to flat there.
- The legacy `.setStartSpeed(cruise).setEndSpeed(cruise)` spelling still works and is unchanged; prefer `.flat()` in new code.

## Trigger to revisit

If callers routinely want to flatten only ONE side of a Drive move (ramp up but not down, or vice-versa), promote the Drive builders to LineFollower's two-flag model instead of the single `tShape` flag.

## See also

- [ADR-017](./017-sync-plus-tshape-default-profile.md) — the default this opts out of; its trigger anticipated this ADR
- [ADR-014](./014-tshape-trapezoidal-velocity-profile.md) — profile semantics
- [ADR-018](./018-linefollower-distance-gated-stops.md) — LineFollower's two-phase ramp design
- `WRO2026/Drive.h` — `DriveDistBuilder::flat` / `DriveTimeBuilder::flat`; `WRO2026/LineFollower.cpp` — `flat()`
