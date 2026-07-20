# ADR-078: In-place turns run a gated speed profile, not a PD loop

**Status:** Implemented
**Date decided:** 2026-07 (Phase 6)

## Rule

`Drive::_updateTurnProfile` runs a monotonic gated speed profile, decision math extracted to
the header-only, host-tested `TurnProfilePolicy.h`. From rest it floors the command to
`TURN_BREAKAWAY_PWM` until yaw has moved `TURN_BREAKAWAY_MOTION_DEG` — breaking static scrub
friction, the two-level floor kernel reused verbatim from ADR-068 — then hands off to a phase
machine driven by the progress-frame error: **CRUISE** (`|err| > TURN_CREEP_WINDOW_DEG`)
commands the fixed `_turnCruisePwm` (the `.setPower()` override, else `TURN_CRUISE_PWM`);
**CREEP** (`TURN_TOLERANCE_DEG < |err| <= TURN_CREEP_WINDOW_DEG`) hard-switches, no ramp, to the
fixed `TURN_CREEP_PWM`; **SETTLE** (`|err| <= TURN_TOLERANCE_DEG`) latches — the phase only
advances, never regresses — and P-nudges any residual (`TURN_SETTLE_KP`, clamped to
`±TURN_CREEP_PWM`) to zero before releasing (brake, motors off; no active hold). Every phase,
including the breakaway kick, commands `copysign(speed, errProg)` toward the shortest-path
target, so an overshoot in any phase reverses instead of driving on past it. There is no
derivative term anywhere in the turn law; the only CRUISE/CREEP feedback is which error-zone the
turn currently sits in, which makes the controller immune to the tick's rate and jitter.

## Why

The per-tick PD (ADR-068, retuned to a raw derivative by ADR-077) never settled on our actual
cooperative tick. Yaw arrives from the external gyro at whole-degree resolution (ADR-074) and
the tick measures ~8 ms but jitters to ~11.5 ms, so a derivative — time-normalized or raw —
turned a single quantization step into a spike whose size depended only on when the tick
happened to fire. No fixed gain landed inside tolerance without also hunting or overshooting on
a different run. Dropping the derivative removes the timing dependency outright: CRUISE and
CREEP are fixed speeds picked purely by error-zone, and a fixed creep speed can only carry a
fraction of a degree per slow tick, so the turn lands predictably no matter how slow or jittery
the tick runs.

The two failures the original two-stage executor documented in ADR-068 are both still
addressed, not reopened. (a) Sub-degree resolution: ADR-068's fix of comparing yaw as a float
end-to-end instead of truncating to `int16_t` carries over unchanged — the profile's phase
boundaries and settle nudge all read the same float error. (b) The from-rest stall: a small turn
dispatched straight into CREEP would command `TURN_CREEP_PWM`, below the static scrub floor, and
never move. ADR-068's answer was the two-level breakaway/creep floor, kept here verbatim
(`_turnBrokeAway`, `_turnStartYaw`, `TURN_BREAKAWAY_PWM`, `TURN_BREAKAWAY_MOTION_DEG`), paired
with a creep floor shipped at the bench-measured kinetic sustain floor (`TURN_CREEP_PWM=38`, up
from the previously-broken 20) so CREEP itself no longer stalls once the kick hands off.

This ADR replaces the control law, not the surrounding machinery. The breakaway kick, the
`TURN_SETTLE_SAMPLES`-sample settle debounce, and `TurnSignPolicy`'s wheel-pattern and sign math
are all reused from ADR-068 unchanged — only `Kp*err + Kd*rate` (ADR-068, or `Kd*(errNew -
errOld)` under ADR-077) is gone, replaced by the fixed-speed phase selection above.

## How to apply

- Decision math lives in `TurnProfilePolicy.h` (`turnPhaseFor`, `turnProfileSpeed`,
  `turnSettleNudge`) — pure, header-only, zero Arduino/`Config.h` surface — dispatched from
  `Drive::_updateTurnProfile`.
- `TurnPDPolicy.h` keeps only its shared kernel: `turnWrap180`, `turnSettleStep`, `turnSettled`,
  and `turnPdStep`. `turnPdStep` is no longer turn machinery — it is the straight-line
  heading-hold's kernel (`GyroHoldPolicy.h`, ADR-075) and must not be touched for turn reasons.
  `turnPdStepRaw`, `turnSlewLimit`, and `turnCreepFloor` are deleted; nothing else called them.
- `config/ConfigChecks.h` cross-checks the profile's own invariants —
  `TURN_CREEP_WINDOW_DEG > TURN_TOLERANCE_DEG`, `TURN_CREEP_PWM < TURN_CRUISE_PWM`,
  `TURN_BREAKAWAY_PWM >= TURN_CREEP_PWM`, `TURN_SETTLE_KP > 0` — never hand-tune one of these
  constants without checking the coupled `static_assert`.
- Wheel dispatch is unchanged: `TurnSignPolicy.h` still owns the single sign re-application in
  the progress frame and the pivot-vs-counter-rotation wheel pattern (ADR-045, ADR-036,
  ADR-073).

## Trigger to revisit

- The cooperative tick becomes fast and stable enough — an order of magnitude tighter than the
  current ~8–11.5 ms jitter — that a continuous controller could plausibly beat the fixed-creep
  tail on landing time.
- Bench data on the assembled Lofaix chassis shows the hard CRUISE→CREEP switch overshoots or
  jolts the chassis in a way the wide 25° window doesn't absorb; that would call for a decel
  ramp between phases, not a wider window.

## See also

- [ADR-068](./068-turn-pd-controller.md) — its PD control-law clause (`Kp*err + Kd*rate`, no
  trapezoid, no terminal stage) is superseded here; its two-level breakaway/creep floor, settle
  debounce, and FSM/queue structure are reused, not replaced.
- [ADR-077](./077-turn-raw-derivative.md) — fully superseded; the raw-derivative retune it
  introduced is replaced along with the rest of the PD control law.
- [ADR-075](./075-gyro-move-r4-hold.md) — the straight-line heading-hold, untouched; keeps the
  time-normalized `turnPdStep` kernel this ADR leaves in place in `TurnPDPolicy.h`.
- [ADR-036](./036-per-turn-pivot-compensation.md) / [ADR-073](./073-reverse-pivots.md) — pivot
  wheel-pattern math, unaffected; the profile drives the same `turnWheelPattern` dispatch.
- `WRO2026/src/drive/TurnProfilePolicy.h`, `WRO2026/src/drive/Drive.cpp::_updateTurnProfile`,
  `WRO2026/src/drive/TurnPDPolicy.h`, `WRO2026/config/ConfigDrive.h`,
  `WRO2026/config/ConfigChecks.h`.
- `docs/plans/2026-07-21-turn-profile-controller.md` — the originating plan.
