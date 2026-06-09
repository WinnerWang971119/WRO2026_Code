# ADR-017: `DriveMode::SYNC` + `tShape` are the project-wide default for every bounded move

**Status:** Implemented (opt-out affordance refined by ADR-034)
**Date decided:** 2026-05 (Phase 1.5)

## Rule

Every Drive distance/time builder defaults to `DriveMode::SYNC` with `tShape` on. `LineFollower.setEnabled(true)` also ramps via `tShape` by default.

Opt-outs:
- `.setMode(DriveMode::RAW)` — open-loop PWM (the only path that applies `_applyBias`).
- `.setMode(DriveMode::GYRO)` — yaw-locked, for long open straights with no line reference.
- `.flat()` — the named opt-out (added by ADR-034): clears the profile so the move runs at constant cruise.
- `.setStartSpeed(cruise).setEndSpeed(cruise)` — still works; short-circuits both ramps via the no-ramp epsilon. `.flat()` is the clearer spelling for the common case.

We deliberately did **not** add a `.tShape(false)` setter — it invites the `.tShape().tShape(false)` ambiguity. The opt-out is a distinctly-named `.flat()` instead (ADR-034).

## Why

The dominant consumer of the Drive API is the auto-path planner. Phase 2+ issues dozens of moves in source order; a planner that has to remember `.setMode(SYNC).tShape()` on every line is a planner where one forgotten chain-link silently becomes a lurchy open-loop move that drifts off the line.

The cost of forgetting must be borne by the rare case (calibration routine that wants raw PWM, deliberately flat short pulse), not the common case.

SYNC over GYRO: SYNC closes the loop on left-vs-right encoder match — cancels the manufacturer asymmetry between the two drive motors (the dominant disturbance on a flat line-following segment). GYRO is the right tool for long open straights; on a line-following field the line is the heading reference, so layering yaw-lock on top of the outer P+D would either add latency or fight the line controller.

`tShape` default leans on the existing min-move threshold (`TSHAPE_MIN_MOVE_CM = 5`, `TSHAPE_MIN_MOVE_SEC = 0.4`) for graceful degradation. Below those thresholds, the builder destructor silently downgrades from `PROFILE_*` to `DRIVE_*` and runs flat.

## How to apply

- Don't think about it for normal moves; just call the verb.
- `DriveCommand::mode` initializer is `SYNC` so any code that constructs a command directly inherits the project default rather than falling back to RAW.
- Doxygen on `driveDist` / `driveTime` / `LineFollowerEnableBuilder` references this ADR.

## Trigger to revisit

The `.flat()` affordance landed in ADR-034 (this trigger fired). If a planner needs GYRO more than SYNC for most segments, swap the default — but that requires re-examining the line-follow loop's sensitivity to yaw-lock.

## See also

- [ADR-014](./014-tshape-trapezoidal-velocity-profile.md) — profile semantics
- [ADR-016](./016-sync-default-builders.md) — blocking-by-default complement
- [ADR-034](./034-flat-opt-out-tshape.md) — the named `.flat()` opt-out this ADR's trigger anticipated
