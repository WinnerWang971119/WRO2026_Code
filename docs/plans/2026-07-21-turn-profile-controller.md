# Turn Profile Controller Plan (replace PD with cruise → creep → settle)

**Goal:** Replace the in-place turn's continuous PD controller with a gated trapezoidal profile (CRUISE → CREEP → SETTLE) so turns stop overshooting/oscillating on our slow, jittery cooperative tick.

**Approach:** The turn FSM (`Drive::_updateTurnProfile`) drives a **monotonic three-phase profile** on the progress-frame heading error: fixed-fast CRUISE while far from target, a hard switch to fixed-slow CREEP inside a wide window, then a SETTLE phase that lightly P-nudges any residual to zero and releases (brake, motors off). No derivative term anywhere in the drive-to-target — the only feedback in CRUISE/CREEP is which error-zone we're in, and only the tiny terminal residual sees a (P-only) gain. This is immune to loop-rate: a fixed creep speed can only carry a fraction of a degree per slow tick, so it lands predictably no matter how slow or jittery the tick is. Decision math lives in a new header-only, host-tested policy seam `TurnProfilePolicy.h`, mirroring the existing `TurnPDPolicy.h` / `TurnSignPolicy.h` pattern.

## Scope

- **In scope:**
  - Rewrite `Drive::_updateTurnProfile` as a CRUISE → CREEP → SETTLE monotonic phase machine.
  - New pure policy header `TurnProfilePolicy.h` (phase selection, per-phase speed, settle nudge).
  - Delete the PD turn: `turnPdStepRaw` (TurnPDPolicy.h) and the PD-only Drive state (`_turnErrOld`, `_turnPrevSpeed`, `_turnBrokeAway`, `_turnStartYaw`).
  - Config: add `TURN_CREEP_WINDOW_DEG`, `TURN_HOLD_KP`; retire `TURN_KP`, `TURN_KD`, `TURN_SLEW_PWM_PER_TICK`, `TURN_BREAKAWAY_PWM`, `TURN_BREAKAWAY_MOTION_DEG`, `TURN_CREEP_THRESH_PWM`. Keep `TURN_CRUISE_PWM`, `TURN_CREEP_PWM`, `TURN_TOLERANCE_DEG`, `TURN_SETTLE_SAMPLES`, `TURN_TIMEOUT_MS`, `TURN_MIN_RESOLVABLE_DEG`.
  - `ConfigChecks.h`: add `TURN_CREEP_WINDOW_DEG > TURN_TOLERANCE_DEG` assert; drop asserts on retired constants.
  - Rewrite host tests `test_turn_pd_policy.cpp` (→ `test_turn_profile_policy.cpp`) and `test_turn_pd_integration.cpp` for the profile.
  - Rework `inoTest/turn_autotune` to Twiddle-search the profile knobs (`CREEP_WINDOW`, `CREEP_PWM`, `HOLD_KP`) instead of Kp/Kd.
  - ADR-078 superseding ADR-068's PD/no-trapezoid clause and ADR-077 (raw-D); update `DECISIONS.md`.
- **Out of scope:**
  - Per-mode (pivot vs counter-rotation) tuning — pivots keep working through `turnWheelPattern` unchanged; separate tuning is a later change.
  - The straight-line gyro-hold (`GyroHoldPolicy.h` + `turnPdStep`) — untouched; `turnPdStep` stays as its kernel.
  - CTPS render — `resolve_turn_profile` already renders turns as display-only constant rate from `TURN_CRUISE_PWM` + a settle tail; the profile doesn't change what it reads, so no CTPS change (verify only).
  - Any physical re-bench of `TURN_CREEP_PWM` / `TURN_CRUISE_PWM` values (done on-robot after this lands, via the bench sequence).

## Decisions

- **Delete the PD path, no toggle** (user, batch 1) — the profile is the only turn controller; `turnPdStepRaw` removed, ADR-077 superseded. One path to maintain.
- **Hard switch cruise → creep** (user, batch 2) — no decel ramp; the wide 25° window gives momentum room to bleed. Simplest FSM.
- **`TURN_CREEP_WINDOW_DEG` starts at 25°** (user, batch 2) — wide/safe starting point, bench-tunable.
- **Hard-launch at cruise from rest** (user, batch 1) — first tick commands full `TURN_CRUISE_PWM`; 50 > breakaway 46, no launch ramp, `TURN_SLEW` retired.
- **Rework, not delete, `turn_autotune`** (user, batch 1) — auto-tune the profile knobs.
- **SETTLE is a latched terminal phase** (design) — once error first drops to ≤ tolerance, the FSM never returns to CREEP/CRUISE even on overshoot; the signed nudge (not a forward creep) corrects an overshoot. Prevents driving further past the target. Implemented as `phase = max(phase, turnPhaseFor(...))`.
- **SETTLE nudge is gated to out-of-band only** (design) — in-band ticks command 0 and count toward settle; only an out-of-band residual gets a `clamp(HOLD_KP·err, ±creep)` pulse. Avoids a within-band mini-oscillation.
- **Timeout stays non-faulting** (assumption, unchallenged) — on `TURN_TIMEOUT_MS`, brake + log residual + done, no fault, no queue clear (ADR-068 posture; PathTracker re-anchors at lines).
- **Keep `TURN_TOLERANCE_DEG=3.0`, `TURN_SETTLE_SAMPLES=2`** (assumption, unchallenged).

## Acceptance Criteria

- [ ] AC1 — A turn far from target commands exactly `TURN_CRUISE_PWM` (sign-correct wheels) while `|err| > TURN_CREEP_WINDOW_DEG`. (T1, T3)
- [ ] AC2 — Inside the window (`TURN_TOLERANCE_DEG < |err| ≤ TURN_CREEP_WINDOW_DEG`) it commands exactly `TURN_CREEP_PWM`. (T1, T3)
- [ ] AC3 — Once `|err| ≤ TURN_TOLERANCE_DEG`, the phase latches SETTLE and never re-enters CREEP/CRUISE, even if a later read shows `|err|` back above the window. (T1, T3)
- [ ] AC4 — In SETTLE, an in-band tick commands 0 and increments the settle count; an out-of-band residual commands `clamp(HOLD_KP·err, ±TURN_CREEP_PWM)` toward target and resets the count. (T1, T3)
- [ ] AC5 — An overshoot in SETTLE (err flips sign, `|err| > tolerance`) produces a **reversed** wheel command, not a forward creep. (T1, T3)
- [ ] AC6 — The turn exits (brake per caller, motors off, `_turnDone`/`_moveDone` set) after `TURN_SETTLE_SAMPLES` consecutive in-band reads. (T3)
- [ ] AC7 — On `TURN_TIMEOUT_MS` with no settle, it brakes, logs the residual, marks done, and does NOT latch a fault or clear the queue. (T3)
- [ ] AC8 — A sub-tolerance turn short-circuits to done with no motion (existing `_execTurnToYaw`/`_execTurnDeg` behavior preserved). (T3)
- [ ] AC9 — Pivot turns (`PIVOT_LEFT`/`PIVOT_RIGHT`) run the identical profile with only the wheel pattern differing (one wheel held). (T3)
- [ ] AC10 — Straight-line gyro-hold tests (`test_gyro_hold_policy`, `test_gyro_hold_integration`) stay green — `turnPdStep` and `GyroHoldPolicy.h` are untouched. (T4)
- [ ] AC11 — `turnPdStepRaw` and the retired config constants are gone; full `ctest` and `arduino-cli` compile are green. (T2, T3, T4)
- [ ] AC12 — `turn_autotune` compiles and its Twiddle search ranges over `CREEP_WINDOW`/`CREEP_PWM`/`HOLD_KP`; `turn_probe` and `turn_tune` still compile. (T5, T6)

## Contracts & Interfaces

### New policy header — `WRO2026/src/drive/TurnProfilePolicy.h` (owner: T1; consumers: T3, T4, T5)

Pure, header-only `inline` free functions, zero Arduino/Config surface. Includes `TurnPDPolicy.h` to reuse `turnWrap180`, `turnSettleStep`, `turnSettled` (these stay in `TurnPDPolicy.h`).

```cpp
// Phase ordering is monotonic: CRUISE(0) < CREEP(1) < SETTLE(2). The FSM only ever advances.
enum class TurnPhase : uint8_t { CRUISE = 0, CREEP = 1, SETTLE = 2 };

// Natural phase for a given absolute progress-frame error. The FSM applies
// phase = max(currentPhase, turnPhaseFor(...)) so it latches and never regresses.
//   absErr >  creepWindowDeg               -> CRUISE
//   toleranceDeg < absErr <= creepWindowDeg -> CREEP
//   absErr <= toleranceDeg                  -> SETTLE
TurnPhase turnPhaseFor(float absErrDeg, float creepWindowDeg, float toleranceDeg);

// Fixed magnitude for the driving phases. SETTLE returns 0 (its command is the nudge).
//   CRUISE -> cruisePwm ; CREEP -> creepPwm ; SETTLE -> 0
int16_t turnProfileSpeed(TurnPhase phase, int16_t cruisePwm, int16_t creepPwm);

// SETTLE terminal nudge (signed, progress frame). In-band -> 0 (let it sit + count);
// out-of-band -> clamp(holdKp * errProg, +/- creepPwm), sign preserved so an overshoot reverses.
float turnSettleNudge(float errProg, float toleranceDeg, float holdKp, int16_t creepPwm);
```

`TurnPhase` uint8 ordering (CRUISE<CREEP<SETTLE) is load-bearing — the FSM's `max()` latch depends on it. Do not reorder.

### Drive dispatch (owner: T3)

`_updateTurnProfile` computes `errProg = _turnDeltaSign * turnWrap180(target - yaw)` (unchanged), then:
```
absErr = fabsf(errProg)
_turnPhase = max(_turnPhase, turnPhaseFor(absErr, TURN_CREEP_WINDOW_DEG, TURN_TOLERANCE_DEG))
if _turnPhase == SETTLE:
    _turnInBandCount = turnSettleStep(absErr, TURN_TOLERANCE_DEG, _turnInBandCount)
    if turnSettled(_turnInBandCount, TURN_SETTLE_SAMPLES): brake(_turnProfileBrakeOnDone); done; return
    cmd = turnSettleNudge(errProg, TURN_TOLERANCE_DEG, TURN_HOLD_KP, TURN_CREEP_PWM)   // signed
else:
    cmd = (float) turnProfileSpeed(_turnPhase, TURN_CRUISE_PWM, TURN_CREEP_PWM)         // positive-progress
wheels = turnWheelPattern(_turnDeltaSign, (int16_t) lroundf(cmd), _turnMotorMode)
MiniR4.DriveDC.Move(wheels.left, wheels.right)
```
Timeout check and the IMU-read-stall watchdog (`turnImuReadStalled`) are kept exactly as today. `_turnPhase` (new `TurnPhase` member) is seeded to `CRUISE` at dispatch in `_execTurnDeg`/`_execTurnToYaw`.

### Config surface (owner: T2; consumers: T1 via values, T3, T5)

New in `ConfigDrive.h` (defaults via `#ifndef X_DEFAULT`, comment on the `constexpr` line not the `#define`):
- `TURN_CREEP_WINDOW_DEG` = `25.0f` — cruise→creep handoff error.
- `TURN_HOLD_KP` = `4.0f` — settle nudge P gain (PWM per deg), clamped to `±TURN_CREEP_PWM`.

Kept: `TURN_CRUISE_PWM`(50), `TURN_CREEP_PWM`(20, **bench-critical** — see Notes), `TURN_TOLERANCE_DEG`(3.0), `TURN_SETTLE_SAMPLES`(2), `TURN_TIMEOUT_MS`(3000), `TURN_MIN_RESOLVABLE_DEG`.
Retired: `TURN_KP`, `TURN_KD`, `TURN_SLEW_PWM_PER_TICK`, `TURN_BREAKAWAY_PWM`, `TURN_BREAKAWAY_MOTION_DEG`, `TURN_CREEP_THRESH_PWM`.

`ConfigChecks.h`: add `static_assert(TURN_CREEP_WINDOW_DEG > TURN_TOLERANCE_DEG, ...)`; keep the `TURN_TOLERANCE_DEG`/`TURN_MIN_RESOLVABLE_DEG`/`TURN_SETTLE_SAMPLES` coupling; remove any assert naming a retired constant (e.g. `TURN_BREAKAWAY_MOTION_DEG >= TURN_MIN_RESOLVABLE_DEG`).

### File ownership

| File | Owner | Consumers |
|------|-------|-----------|
| `WRO2026/src/drive/TurnProfilePolicy.h` (new) | T1 | T3, T4, T5 |
| `WRO2026/config/ConfigDrive.h`, `config/ConfigChecks.h` | T2 | T3, T5 |
| `WRO2026/src/drive/Drive.cpp`, `Drive.h`, `TurnPDPolicy.h` (delete `turnPdStepRaw`) | T3 | T4, T5 |
| `WRO2026/test/test_turn_profile_policy.cpp` (rename), `test_turn_pd_integration.cpp`, `test/CMakeLists.txt` | T4 | — |
| `inoTest/turn_autotune/TurnAutotuneHarness.cpp`, `.h` | T5 | — |
| `docs/decisions/078-*.md` (new), `068-*.md`, `077-*.md`, `DECISIONS.md` | T7 | — |

## Error Handling

- **IMU read fails mid-turn** — unchanged: skip the tick (hold last Move); on sustained failure past `TURN_IMU_STALL_TIMEOUT_MS`, hard-stop + `Fault::IMU_READ_FAILED` + clear queue (`turnImuReadStalled`, kept verbatim).
- **Creep stalls short of target** (creep PWM below the kinetic sustain floor) — the turn hunts to `TURN_TIMEOUT_MS`, then brakes + accepts residual (no fault). This is a *tuning* failure, not a code failure: the fix is raising `TURN_CREEP_PWM` (Notes). The plan surfaces it as a gating bench step, not a runtime recovery.
- **Overshoot in SETTLE** — the signed, gated nudge reverses toward target; if it can't converge within the band it times out non-faulting.
- **Non-convergence timeout** — brake, `Serial.print` the residual, mark done, no fault, no queue clear.

## Testing Strategy

**Levels:** Unit (policy header, zero mocks) + Integration (real `Drive.cpp` via `hal_mocks`, stateful FSM).

| ID | Test Case | Type | Expected |
|----|-----------|------|----------|
| TC1 | `turnPhaseFor` boundaries: absErr 30/25/15/3/1 with window 25 tol 3 | Unit | CRUISE, CREEP (==window is CREEP), CREEP, SETTLE (==tol is SETTLE), SETTLE |
| TC2 | `turnProfileSpeed(CRUISE/CREEP/SETTLE, 50, 20)` | Unit | 50, 20, 0 |
| TC3 | `turnSettleNudge` in-band (err 2, tol 3) | Unit | exactly 0 |
| TC4 | `turnSettleNudge` out-of-band positive (err 8, tol 3, kp 4, creep 20) | Unit | +20 (32 clamped) |
| TC5 | `turnSettleNudge` overshoot negative (err -8) | Unit | -20 (sign preserved → reverses) |
| TC6 | `turnSettleNudge` small out-of-band (err 4, kp 4, creep 20) | Unit | +16 (unclamped) |
| TC7 | Monotonic latch: feed absErr 30→20→2→10 through max(phase, turnPhaseFor) | Unit | CRUISE→CREEP→SETTLE→SETTLE (never back to CREEP) |
| TC8 | Integration: dispatch 90° turn, tick far from target | Integration | dispatched `lastMoveR` magnitude == `TURN_CRUISE_PWM` |
| TC9 | Integration: script yaw into the creep window | Integration | magnitude == `TURN_CREEP_PWM` |
| TC10 | Integration: script yaw in-band for `TURN_SETTLE_SAMPLES` ticks | Integration | brakes, `isTurnDone()`, `isIdle()`, no fault |
| TC11 | Integration: script yaw PAST target (overshoot) in SETTLE | Integration | dispatched command reverses sign; not declared done while `|err|>tol` |
| TC12 | Integration: never settles → `TURN_TIMEOUT_MS` | Integration | brakes, done, `!hasFault()`, queue not cleared |
| TC13 | Integration: sub-tolerance turn (2° with tol 3°) | Integration | short-circuits done, no Move |
| TC14 | Integration: pivot mode turn converges | Integration | settles; only the driven wheel is nonzero in CREEP |
| TC15 | Straight-hold regression | Integration | `test_gyro_hold_*` unchanged and green |

**Test data:** the `turn_harness` in `test_turn_pd_integration.cpp` (real `Drive`+`YawSource`, `tickAtYaw`/`enqueueTurnDeg`), reused/adapted. Policy unit tests need no mocks.
**Run command:** `cmake --build build/test && ctest --test-dir build/test --output-on-failure`, then `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` and `uv run inoTest/turn_autotune/build.py --build-only`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | New `TurnProfilePolicy.h` policy header | — | med | `WRO2026/src/drive/TurnProfilePolicy.h` | Create the pure header with `TurnPhase`, `turnPhaseFor`, `turnProfileSpeed`, `turnSettleNudge` exactly per Contracts. `#include "TurnPDPolicy.h"` for `turnWrap180`/`turnSettleStep`/`turnSettled`. Doxygen on each fn. Satisfies AC1–AC5. |
| T2 | Config + ConfigChecks | — | med | `WRO2026/config/ConfigDrive.h`, `WRO2026/config/ConfigChecks.h` | Add `TURN_CREEP_WINDOW_DEG`=25, `TURN_HOLD_KP`=4 (with rationale docblocks); retire the 6 PD constants listed in Contracts; add the `CREEP_WINDOW>TOLERANCE` static_assert and drop asserts on retired constants. Do NOT touch `TURN_CRUISE_PWM`/`TURN_CREEP_PWM`/`TURN_TOLERANCE_DEG`/`TURN_SETTLE_SAMPLES`/`TURN_TIMEOUT_MS`. |
| T3 | Rewrite `_updateTurnProfile` + Drive state; delete `turnPdStepRaw` | T1, T2 | high | `WRO2026/src/drive/Drive.cpp`, `Drive.h`, `TurnPDPolicy.h` | Replace the PD executor body with the phase machine per Contracts. Add `TurnPhase _turnPhase` member, seed to CRUISE in both `_exec*` dispatchers; remove PD-only members (`_turnErrOld`, `_turnPrevSpeed`, `_turnBrokeAway`, `_turnStartYaw`) and their seeding. Keep timeout, settle-count, IMU-stall watchdog, `turnWheelPattern` dispatch. Delete `turnPdStepRaw` + its docblock from `TurnPDPolicy.h`. Satisfies AC1–AC9, AC11. |
| T4 | Rewrite host tests | T1, T3 | med | `WRO2026/test/test_turn_profile_policy.cpp` (rename from `test_turn_pd_policy.cpp`), `test_turn_pd_integration.cpp`, `test/CMakeLists.txt` | Replace the PD policy tests with TC1–TC7 for `TurnProfilePolicy.h`; rewrite the integration cases to TC8–TC14; keep `turnWrap180`/`turnSettleStep`/`turnSettled` tests. Update `CMakeLists.txt` for the renamed file. Do not touch `test_gyro_hold_*`. Satisfies AC10 (regression), AC11, and all TCs. |
| T5 | Rework `turn_autotune` for profile knobs | T1, T3 | med | `inoTest/turn_autotune/TurnAutotuneHarness.cpp`, `.h` | Replace the PD replica loop with the profile FSM (call `TurnProfilePolicy.h`); change Twiddle `SPECS` to search `CREEP_WINDOW`(seed 25), `CREEP_PWM`(seed `TURN_CREEP_PWM`), `HOLD_KP`(seed 4) with sane bounds/steps; update header comments (drop the PD/`turnPdStep` language). Build-only compile check. Satisfies AC12. |
| T6 | Verify `turn_probe`/`turn_tune` still compile | T2, T3 | low | `inoTest/turn_probe/*`, `inoTest/turn_tune/*` | These link real `Drive`/measure the plant; they should compile unchanged. Fix only compile breaks from the config renames (e.g. a retired constant referenced in a print). Confirm via `build.py --build-only`. Satisfies AC12. |
| T7 | ADR-078 + DECISIONS.md | T3 | low | `docs/decisions/078-turn-profile-controller.md` (new), `068-turn-pd-controller.md`, `077-turn-raw-derivative.md`, `DECISIONS.md` | Write ADR-078 (supersedes ADR-068's per-tick-PD/no-trapezoid/no-terminal-stage clause AND ADR-077; explain the two original two-stage failures are fixed — float yaw end-to-end, kinetic creep). Mark 068's PD clause + 077 superseded; update the DECISIONS.md index line + status table (068 partially, 077 fully). Reuse note: ADR-068's settle debounce + `TurnSignPolicy`/`turnWheelPattern` are kept. |
| T8 | CTPS no-change verification | T2 | low | `tools/path_planner/ctps/profile.py` (read-only likely) | Confirm `resolve_turn_profile` only reads `TURN_CRUISE_PWM` (kept) and needs no change; run the ctps pytest subset. Edit only if a retired constant is referenced. |

Parallelism: T1 and T2 run in parallel (no blockers). T3 waits for both. T4/T5 run in parallel after T3. T6/T8 after their config/Drive deps. T7 after T3.

## Notes for Implementer

- **`TURN_CREEP_PWM` is the make-or-break value.** It MUST sit above the chassis's kinetic sustain floor or CREEP stalls short and every turn hunts to timeout — the exact failure we're replacing. Project memory notes the Lofaix kinetic floor measured ~38, but the shipped constant is currently 20. Treat 20 as a placeholder: the on-robot bring-up sequence is **turn_probe first** (its kinetic-sweep phase measures the sustain floor) → set `TURN_CREEP_PWM` a few PWM above it → then turn_tune to confirm landing accuracy. Do NOT trust the turn on hardware until this is done.
- CRUISE is always driven in the positive-progress direction (errProg > tolerance > 0 in CRUISE/CREEP), so `turnProfileSpeed` returns an unsigned magnitude and `turnWheelPattern` applies `_turnDeltaSign`. Only SETTLE produces a signed command (the nudge), which is why overshoot reversal lives entirely in `turnSettleNudge` + the latched SETTLE phase.
- The monotonic `max(phase, turnPhaseFor(...))` latch is essential: without it, an overshoot in SETTLE (|err| back inside the creep window) would re-classify to CREEP and drive a forward creep *further past* the target. Pin this with TC7.
- Whole-degree gyro: a sub-1° read floors to 0°, so `TURN_TOLERANCE_DEG=3` and `TURN_SETTLE_SAMPLES=2` are comfortably resolvable; keep the ConfigChecks coupling.
- Straight-drive is a landmine to avoid, not a thing to change: `turnPdStep` (time-normalized) stays in `TurnPDPolicy.h` because `GyroHoldPolicy.h::gyroHoldCorrection` reuses it. Only `turnPdStepRaw` is deleted. `test_gyro_hold_*` must stay green as the tripwire.
- Rollback: the change is confined to the worktree branch `worktree-turn-raw-d-term` (which also holds the ADR-077 raw-D commit-in-progress). If the profile underperforms on the bench, the raw-D PD is one `git` revert away — but per the user's decision we are not keeping it as a runtime toggle.
- Build/verify locally on the macOS box (full toolchain present): `ctest` + `arduino-cli compile` + `turn_autotune` build-only. None of the `inoTest/` sketches are in CI.
