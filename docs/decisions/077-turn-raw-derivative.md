# ADR-077: The in-place turn's D term is the mentor's RAW per-iteration difference, not time-normalized

**Status:** Superseded by [ADR-078](./078-turn-profile-controller.md)
**Date decided:** 2026-07 (Phase 6)

## Rule

`Drive::_updateTurnProfile` computes its per-tick PD command with
`TurnPDPolicy.h::turnPdStepRaw(errNew, errOld, Kp, Kd)` = `Kp*errNew + Kd*(errNew - errOld)`
— the derivative is the RAW change in error between two consecutive PD ticks, with **no
division by elapsed time**, exactly as the mentor's competition-verified `M_Turn_angle`
computes it. Gains are the mentor's newest revision: `TURN_KP = 1.5`, `TURN_KD = 10`. This
replaces ADR-068's time-normalized derivative (`Kd*(errNew - errOld)/dtSec`, `TURN_KD = 0.06`)
for the turn only. Everything else ADR-068 established — the two-level breakaway/creep floor,
the ramp-up slew limiter, the N-sample settle debounce, the cooperative-FSM structure, and the
operator order — is UNCHANGED. The straight-line heading hold keeps the time-normalized
`turnPdStep` kernel (ADR-075); this ADR does not touch it.

## Why

Time-normalization is the textbook-correct way to make a derivative gain independent of loop
rate, and ADR-068 chose it deliberately (`TURN_KD = Kd_raw * mentor_loop_period ~= 10 * 0.006`).
But it assumes a clean, continuous error signal sampled at a known dt. The turn has neither:
yaw comes from the external gyro at WHOLE-DEGREE resolution (ADR-074), and the cooperative tick
is a measured ~8 ms that jitters to ~11.5 ms. Dividing a 1-degree quantization step by that
small, varying dt turns a single quantum into a timing-dependent D spike — the same physical
event lands anywhere in a ~5..15 PWM band depending only on when the tick fired — and, because
the real tick is slower than the assumed 6 ms, leaves the loop 25–48% under-damped. The net
effect is a controller whose D response is not repeatable run-to-run, so no fixed gain ever
settles: exactly the "I can't tune this" the operator hit. The mentor's raw per-iteration
difference is deterministic per tick and is the model his proven gains were tuned under; two
independent `M_Turn_angle` revisions (Kp 2 and Kp 1.5) both use it and neither divides by dt.

## How to apply

- The turn calls `turnPdStepRaw` (no `dtSec` argument); `turnPdStep` is retained ONLY as the
  straight-line hold's kernel (`GyroHoldPolicy.h`, ADR-075). Do not route the turn back through
  `turnPdStep`, and do not change `turnPdStep` to raw — that silently re-tunes straight-drive,
  which has its own `GYRO_HOLD_KD` calibrated around the time-normalized model.
- `TURN_KP` / `TURN_KD` in `config/ConfigDrive.h` are the mentor's raw gains; the measured-dt
  member `_turnLastStepMs` is gone. Bench-verify via `inoTest/turn_tune` (T11).
- Operator order is unchanged: `pdStepRaw -> clamp ±power -> slewLimit -> creepFloor ->
  turnWheelPattern`.

## Trigger to revisit

- The yaw source gains sub-degree resolution AND a fixed sample cadence — then a time-normalized
  derivative would no longer amplify quantization, and re-unifying the two kernels could be worth it.
- A future chassis needs loop-rate-independent turn damping across widely varying tick rates.

## See also

- [ADR-068](./068-turn-pd-controller.md) — partially superseded by this ADR (the time-normalized
  D-term clause only); its floor / slew / settle / FSM clauses remain Implemented.
- [ADR-075](./075-gyro-move-r4-hold.md) — the straight-line hold, which keeps the time-normalized
  `turnPdStep` kernel this ADR leaves in place.
- [ADR-074](./074-external-gyro-yaw-provider.md) — the whole-degree external gyro whose
  quantization this ADR's model tolerates.
- `WRO2026/src/drive/TurnPDPolicy.h::turnPdStepRaw`, `WRO2026/src/drive/Drive.cpp::_updateTurnProfile`,
  `WRO2026/config/ConfigDrive.h`, `WRO2026/test/test_turn_pd_policy.cpp`.
- [ADR-078](./078-turn-profile-controller.md) — superseding decision: the turn's PD control law,
  raw derivative included, is replaced by a derivative-free breakaway→cruise→creep→settle
  profile. `turnPdStepRaw` is deleted.
