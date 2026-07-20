# ADR-068: In-place turns run a per-tick PD controller in `Drive.cpp`, not `TurnGyro`

**Status:** Implemented (the PD control-law clause — `Kp*err + Kd*rate`, no trapezoid, no
terminal stage — is superseded by ADR-078's breakaway→cruise→creep→settle profile; the
two-level breakaway/creep floor, settle debounce, float-yaw comparison, `ConfigChecks.h`
coupling, and `TurnSignPolicy` wheel dispatch this ADR introduced are reused unchanged by
ADR-078. The time-normalized `Kd*rate` D-term clause was earlier superseded by ADR-077, itself
now folded into ADR-078's supersession. All other clauses remain in force.)
**Date decided:** 2026-07 (Phase 6)

## Rule

`Drive::_updateTurnProfile` — decision math extracted to the header-only, host-tested
`TurnPDPolicy.h` — runs **one PD step per cooperative tick**: `Kp*err + Kd*rate`, where
`rate` is the TIME-NORMALIZED derivative (`(errNew - errOld) / measured_dtSec`, never a raw
per-iteration difference). The command is clamped to the caller's power, slew-limited on
ramp-up only (`Config::TURN_SLEW_PWM_PER_TICK`), and floored where the P term loses
authority. There is no integrator, no trapezoid/two-stage cruise-then-slow shape, no
terminal stage, and no blocking site — the turn is a cooperative FSM step inside
`Drive.update()`, same as every other primitive. Exit requires `TURN_SETTLE_SAMPLES` (2)
consecutive samples inside `TURN_TOLERANCE_DEG` (0.75°); both are coupled to the
bench-measured per-tick angular resolution (`TURN_MIN_RESOLVABLE_DEG`) by `static_assert`s
in `ConfigChecks.h`, turning a future desync into a build failure instead of every turn
silently hunting to `TURN_TIMEOUT_MS`.

**The floor is two-level — load-bearing, beyond the mentor's source law.** Starting an
in-place spin from rest must break STATIC scrub friction, bench-measured on Lofaix HIGHER
than the KINETIC floor that merely sustains an already-turning spin
(`TURN_BREAKAWAY_PWM=46` vs `TURN_CREEP_PWM=38`): a differential-drive spin scrubs every
contact patch sideways, and breaking static friction costs more than sustaining motion.
`_updateTurnProfile` floors its command to the static `TURN_BREAKAWAY_PWM` from rest until
the yaw has rotated `TURN_BREAKAWAY_MOTION_DEG` (1.0°, latched via `_turnBrokeAway`), then
hands off to the kinetic `TURN_CREEP_PWM`. Without this, a sub-45°-error turn — whose PD
command never exceeds a single creep floor — would command only the kinetic value, never
break away, and hunt silently to the non-faulting `TURN_TIMEOUT_MS`.

## Why

The prior two-stage cruise→slow-zone→brake executor (introduced 2026-05-17 to replace the
vendor `TurnGyro`) could never land inside its own tolerance band: the turn path compared
yaw as `int16_t`, and the truncation quantized the error past the point the band could
resolve. Its slow stage also ran near the STATIC breakaway floor rather than the lower
kinetic floor a chassis already rotating actually needs — the single largest contributor to
the overshoot the operator observed on the bench. The mentor's `M_Turn_angle` control law —
continuous PD with a kinetic creep floor and no terminal stage — is competition-verified on
real hardware; porting it (with a time-normalized `Kd` and an added slew limiter for the
heavier Lofaix chassis) replaces a design structurally unable to reach its own target with
one that already has.

## How to apply

- Decision math lives only in `TurnPDPolicy.h` (`turnWrap180`, `turnPdStep`,
  `turnSlewLimit`, `turnCreepFloor`, `turnSettleStep`, `turnSettled`) — pure, header-only,
  zero Arduino/`Config.h` surface, host-unit-tested in `test_turn_pd_policy.cpp`; the
  stateful FSM is covered separately by `test_turn_pd_integration.cpp` against the real
  `Drive.cpp`.
- Apply the operator order exactly: `pdStep → clamp to ±power → slewLimit → creepFloor →
  turnWheelPattern`. `TurnSignPolicy.h` is unchanged by this ADR and still owns wheel
  dispatch and the single sign re-application in the progress frame (ADR-045).
- All turn tunables live in `config/ConfigDrive.h` under the `TURN_*` prefix and are
  cross-checked in `config/ConfigChecks.h`; never hand-tune one without checking the
  coupled `static_assert`s.
- CTPS renders the same shape as a display-only constant-rate model (`ctps/profile.py`) — a
  PD loop's true duration is plant-dependent and not closed-form, so playback claims no
  fidelity here, same posture ADR-061 took for `HOP_SETTLE_MS`.
- Bench values (`TURN_KP`, `TURN_KD`, `TURN_CREEP_PWM`, `TURN_BREAKAWAY_PWM`,
  `TURN_SLEW_PWM_PER_TICK`) are the mentor's TT starting points, re-measured once on Lofaix
  (T1, 2026-07-16); each carries a `TODO bench-tune (T11)` marker in `ConfigDrive.h`.
  **B3.2's measured residual spread is TODO — pending T11's bench re-pass on Lofaix; this
  ADR ships ahead of that number** by explicit operator decision, against the T1 data.

## Trigger to revisit

- The external I2C gyro board lands and offers a faster or cleaner yaw read than the
  current tick-bound resolution — `YawSource` is the sole swap point (ADR-045).
- Sub-degree accuracy is wanted below the measured `TURN_MIN_RESOLVABLE_DEG` floor — that
  needs a faster-than-tick (blocking) loop, its own ADR against ARCHITECTURE.md's iron
  rule 1, not a tightened constant here.

## See also

- [ADR-077](./077-turn-raw-derivative.md) — superseded this ADR's time-normalized D-term clause:
  the turn's derivative reverted to the mentor's raw per-iteration difference (`Kd*(errNew -
  errOld)`, no dt division; `TURN_KP`/`TURN_KD` = 1.5/10). Itself now superseded by ADR-078.
- [ADR-078](./078-turn-profile-controller.md) — supersedes this ADR's PD control-law clause
  outright: no more `Kp*err + Kd*rate` in any form, replaced by a derivative-free
  breakaway→cruise→creep→settle profile. The two-level floor, settle debounce, and FSM
  structure decided here carry over unchanged; the ramp-up slew limiter (`turnSlewLimit`) is
  retired, since CRUISE now launches at a fixed speed with nothing to ramp.
- [ADR-004](./004-pid-stays-on-stm32.md) — its Turn/`TurnGyro`/`TURN_PID_*` clause is
  superseded by this ADR; the Sync and Gyro-straight clauses are unaffected and it remains
  Implemented.
- [ADR-014](./014-tshape-trapezoidal-velocity-profile.md) — scoped turn ramping out as a
  "separate project, requires a custom continuous gyro-locked turn loop"; this ADR is that
  project.
- [ADR-045](./045-imu-yaw-sign-normalization.md) — `YawSource` / `TurnSignPolicy.h` seams
  this ADR reads through unchanged; its own Trigger section anticipated a `TurnGyro`
  revival that instead never happened.
- [ADR-058](./058-ctps-firmware-motion-profile.md) — its config-derived two-stage turn
  render is replaced by this plan's display-only constant-rate model.
- `WRO2026/src/drive/TurnPDPolicy.h`, `WRO2026/src/drive/Drive.cpp::_updateTurnProfile`,
  `WRO2026/config/ConfigDrive.h`, `WRO2026/config/ConfigChecks.h`.
- `docs/plans/2026-07-16-turn-pd-controller.md` — the originating plan.
