# ADR-075: GYRO-straight heading-hold moves to an R4-side per-tick PD; the STM32 `MoveGyro*` write path retires

**Status:** Implemented
**Date decided:** 2026-07 (Phase 6)

## Rule

`Drive`'s GYRO-mode straight-line heading-hold (T-shape straights, wall-depart, and the
GYRO stop-at-line / wall-stop seeks) runs one PD step per cooperative tick —
`Drive::_emitGyroHold`, decision math extracted to the header-only, host-tested
`GyroHoldPolicy.h` — over `Move(L, R)`, reading yaw from `YawSource.readYawDeg` (the
external gyro, ADR-074). The derivative is computed on the measurement (a fold-safe
wrapped yaw-delta between consecutive ticks, not on the target error), so a heading step
at a chained-segment boundary produces no derivative kick. The correction is dynamically
clamped so neither wheel ever reverses (`|corr| ≤ min(GYRO_HOLD_MAX_CORR_PWM, |base|)`),
and the control law is PD plus clamp only — no slew limiter, no creep floor, no
integrator. The STM32 `MoveGyro*` write path — `MoveGyroDegs`/`MoveGyroTime`,
`YawSource::toHwHeading`, `setMoveGyroPID`/`MOVEGYRO_KP/KI/KD`, and the now-orphaned
`Config::IMU_YAW_SIGN` — is retired; no production code issues a heading target to the
STM32's own gyro PID anymore.

## Why

#172 moved yaw *reads* to the external gyro board (ADR-074), but the STM32 `MoveGyro*`
calls still served heading targets against the STM32's own onboard IMU zero — a
split-brain (#169): the first GYRO move after a read swap steered to a wrong absolute
heading immediately, not a slow drift. One coherent heading source has to close the loop
end to end. Porting the mentor's proven `gyro_move` control law (`Kp*err + Kd*rate`,
`Move(base±corr)`) into the same per-tick cooperative-FSM shape ADR-068 already
established for turns — reusing `turnWrap180`/`turnPdStep` from `TurnPDPolicy.h` —
replaces the STM32 write path with an R4-side hold that reads the same external gyro the
rest of `Drive` already trusts.

## How to apply

- Decision math lives only in `GyroHoldPolicy.h` (`gyroHoldCorrection`, `gyroHoldWheels`)
  — pure, header-only, zero Arduino/`Config.h` surface, host-unit-tested; the stateful
  hold (`_gyroHoldPrevYaw`/`_gyroHoldPrevMs`/`_gyroHoldHasPrev`) is covered separately by
  a Drive host-integration test asserting the dispatched `Move(L, R)` differential.
- `_emitGyroHold` is called from inside `_emitContinuous`'s single GYRO choke point,
  covering all three cooperative FSMs (`_updateProfile`, `_updateStopAtLine`,
  `_updateWallStop`) at one call site.
- Flat/short GYRO moves are lowered to a flat T-shape profile at the dispatcher
  (`_dispatchNext`, explicit `accel=Config::TSHAPE_ACCEL_PWM_PER_CM`), not the builder —
  the plan/parity/CTPS/sim contracts still record the PLAN, unchanged.
- On a `readYawDeg` failure mid-hold, emit `Move(base, base)` (symmetric, zero
  correction) and leave `prevYaw`/`prevMs` untouched; every GYRO move is bounded by a
  non-IMU terminator (distance/line/stall), so a stuck gyro degrades to an
  uncorrected-but-completing straight rather than a new fault.
- Wheel-mix polarity reuses the turn's chassis handedness (`turnWheelPattern(+1, corr,
  COUNTER_ROT)`) — do not mint a new sign or copy the mentor's literal (inverted)
  `power±corr` mapping.
- Tunables (`GYRO_HOLD_KP`, `GYRO_HOLD_KD`, `GYRO_HOLD_MAX_CORR_PWM`,
  `GYRO_HOLD_STALE_MS`) live in `config/ConfigDrive.h` under the `GYRO_HOLD_*` prefix,
  cross-checked in `config/ConfigChecks.h`; they ship seeded from the mentor's
  `gyro_move` as UNVERIFIED placeholders pending the bench pass.
- This ADR fulfills the "(Future) outer-loop yaw-track regulators" line ADR-004 reserved
  for application-owned PID.

## Trigger to revisit

- The bench shows a constant (not slowly drifting) lateral offset on a GYRO straight —
  the named steady-state-error mechanism of an unbiased PD with no integrator
  (`disturbance_PWM / KP`). First remedy: apply the existing `_applyBias` feedforward to
  the base (already used by the RAW case); second: add an integral term.

## See also

- [ADR-068](./068-turn-pd-controller.md) — the per-tick PD FSM shape this ADR mirrors
  for the straight-hold case (`TurnPDPolicy.h` → `GyroHoldPolicy.h`, same
  `turnWrap180`/time-normalized-derivative pattern).
- [ADR-004](./004-pid-stays-on-stm32.md) — its Gyro-straight clause (`MoveGyro` closes
  its loop on the STM32) is superseded by this ADR; this ADR also fulfills its
  "(Future) outer-loop yaw-track regulator" note.
- [ADR-045](./045-imu-yaw-sign-normalization.md) — its write half
  (`YawSource::toHwHeading`, `Config::IMU_YAW_SIGN`) is superseded by this ADR; the read
  half (now the external-gyro `GYRO_YAW_SIGN`, ADR-074) is unaffected.
- [ADR-074](./074-external-gyro-yaw-provider.md) — named #169 (this ADR's origin) as its
  own *Trigger to revisit*; that split-brain is resolved by this ADR.
- `WRO2026/src/drive/GyroHoldPolicy.h`, `WRO2026/src/drive/Drive.cpp::_emitGyroHold`,
  `WRO2026/config/ConfigDrive.h`, `WRO2026/config/ConfigChecks.h`.
- `docs/plans/2026-07-19-169-gyro-move-r4-hold.md` — the originating plan.
