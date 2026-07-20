# Turn Profile Controller Plan (replace PD with breakaway → cruise → creep → settle)

**Goal:** Replace the in-place turn's continuous PD controller with a gated speed profile (from-rest breakaway kick → CRUISE → CREEP → terminal SETTLE) so turns stop overshooting/oscillating on our slow, jittery cooperative tick.

**Approach:** The turn FSM (`Drive::_updateTurnProfile`) drives a **monotonic phase profile** on the progress-frame heading error. From rest it first kicks at the static breakaway floor until motion is observed (reused from ADR-068 — a differential spin must break static scrub friction to start), then runs fixed-fast CRUISE while far from target, hard-switches to fixed-slow CREEP inside a wide window, and finishes in a SETTLE phase that lightly P-nudges any residual to zero and releases (brake, motors off). **Every phase commands `copysign(speed, errProg)`** — always toward the shortest-path target — so an overshoot reverses in any phase, not just SETTLE. No derivative term anywhere; the only feedback in CRUISE/CREEP is which error-zone we're in. This is immune to loop-rate: a fixed creep speed can only carry a fraction of a degree per slow tick, so it lands predictably no matter how slow or jittery the tick is. Decision math lives in a new header-only, host-tested policy seam `TurnProfilePolicy.h`, mirroring `TurnPDPolicy.h` / `TurnSignPolicy.h`.

## Scope

- **In scope:**
  - Rewrite `Drive::_updateTurnProfile` as breakaway-gate → CRUISE → CREEP → SETTLE, all commands signed by `errProg`.
  - New pure policy header `TurnProfilePolicy.h` (phase selection, per-phase speed, settle nudge).
  - Delete the PD turn: `turnPdStepRaw`, `turnSlewLimit`, `turnCreepFloor` (all now zero-consumer) from `TurnPDPolicy.h` and rewrite its file docblock; delete the PD-only Drive state (`_turnErrOld`, `_turnPrevSpeed`). **Keep** the breakaway state (`_turnBrokeAway`, `_turnStartYaw`) and the power-override state (`_turnCruisePwm`).
  - Config: add `TURN_CREEP_WINDOW_DEG`, `TURN_SETTLE_KP`; retire `TURN_KP`, `TURN_KD`, `TURN_SLEW_PWM_PER_TICK`, `TURN_CREEP_THRESH_PWM`; raise `TURN_CREEP_PWM` default off the known-broken 20. **Keep** `TURN_BREAKAWAY_PWM`, `TURN_BREAKAWAY_MOTION_DEG`, `TURN_CRUISE_PWM`, `TURN_CREEP_PWM`, `TURN_TOLERANCE_DEG`, `TURN_SETTLE_SAMPLES`, `TURN_TIMEOUT_MS`, `TURN_MIN_RESOLVABLE_DEG`.
  - `ConfigChecks.h`: add `TURN_CREEP_WINDOW_DEG > TURN_TOLERANCE_DEG`, `TURN_CREEP_PWM < TURN_CRUISE_PWM`, `TURN_BREAKAWAY_PWM >= TURN_CREEP_PWM`, `TURN_SETTLE_KP > 0`; keep the `TOLERANCE`/`MIN_RESOLVABLE`/`SETTLE_SAMPLES` and `BREAKAWAY_MOTION_DEG >= MIN_RESOLVABLE` couplings; drop only the assert naming the retired `TURN_CREEP_THRESH_PWM`.
  - Rewrite host tests `test_turn_pd_policy.cpp` (→ `test_turn_profile_policy.cpp`) and `test_turn_profile_integration.cpp` (renamed from `test_turn_pd_integration.cpp`) for the profile; port the `turnWrap180` + kept-kernel (`turnPdStep` dt-edge) tests.
  - Rework `inoTest/turn_autotune` to Twiddle-search `CREEP_WINDOW` / `CREEP_PWM` / `SETTLE_KP`; fix `inoTest/turn_tune`'s retired-constant references + operator guidance; verify `turn_probe` compiles.
  - ADR-078 superseding ADR-068's PD/no-trapezoid clause and ADR-077; update `DECISIONS.md`.
  - Fix `ctps/profile.py` docstrings that name the retired `TURN_CREEP_THRESH_PWM` / describe the PD law.
- **Out of scope:**
  - Per-mode (pivot vs counter-rotation) tuning — pivots keep working through `turnWheelPattern` unchanged.
  - The straight-line gyro-hold (`GyroHoldPolicy.h` + `turnPdStep`) — untouched; `turnPdStep` stays as its kernel.
  - CTPS *render behavior* — `resolve_turn_profile` reads only `TURN_CRUISE_PWM` (kept); only stale docstrings change.
  - Physical re-bench of the PWM values — set on-robot after this lands (bench sequence in Notes).

## Decisions

- **Delete the PD path, no toggle** (user, batch 1) — the profile is the only turn controller; `turnPdStepRaw` removed, ADR-077 superseded.
- **Keep the from-rest breakaway kick** (critic-driven, corrects a wrong assumption) — a small turn (`|err| ≤ window`) would otherwise command creep PWM from rest, below the static scrub floor, and never start (ADR-068's documented third failure). The FSM kicks `TURN_BREAKAWAY_PWM` until yaw has moved `TURN_BREAKAWAY_MOTION_DEG`, then hands to the phase profile. `_turnBrokeAway`/`_turnStartYaw` are retained, not deleted.
- **Every phase command is `copysign(speed, errProg)`** (critic-driven) — CRUISE/CREEP/breakaway all drive toward the shortest-path target, so a band-skip overshoot reverses in any phase. The old "errProg > 0 in CRUISE/CREEP" invariant was false and is dropped.
- **Hard switch cruise → creep** (user, batch 2) — no decel ramp; the wide 25° window gives momentum room to bleed.
- **`TURN_CREEP_WINDOW_DEG` starts at 25°** (user, batch 2).
- **Hard-launch at cruise for LARGE turns** (user, batch 1) — for `|err| > window` CRUISE ≈ breakaway, so the kick and cruise coincide; `TURN_SLEW` retired.
- **SETTLE releases (brake, motors off), not an active hold** (user — "settle then release", AskUserQuestion batch after design) — a stationary diff-drive robot doesn't drift; the next move re-anchors heading.
- **SETTLE in-band boundary is strict `<TURN_TOLERANCE_DEG`** (critic-driven) — matches the kept `turnSettleStep`, so a landing exactly at tolerance gets nudged in rather than freezing.
- **CRUISE magnitude is `_turnCruisePwm`, not the raw config** (critic-driven) — preserves the `.setPower()` builder override.
- **Rework, not delete, `turn_autotune`** (user, batch 1).
- **Timeout stays non-faulting** (assumption, unchallenged) — brake + log residual + done, no fault, no queue clear.
- **Keep `TURN_TOLERANCE_DEG=3.0`, `TURN_SETTLE_SAMPLES=2`** (assumption, unchallenged).

## Acceptance Criteria

- [ ] AC1 — Far from target (`|err| > TURN_CREEP_WINDOW_DEG`), once broken away it commands `_turnCruisePwm` toward target. (T1, T3)
- [ ] AC2 — Inside the window (`TURN_TOLERANCE_DEG < |err| ≤ window`), once broken away it commands `TURN_CREEP_PWM` toward target. (T1, T3)
- [ ] AC3 — **From rest, tick 1 commands `TURN_BREAKAWAY_PWM`** regardless of phase, until yaw has moved `TURN_BREAKAWAY_MOTION_DEG`; this covers small (`|err| ≤ window`) turns that would otherwise creep from rest and stall. (T3)
- [ ] AC4 — Once `|err| ≤ TURN_TOLERANCE_DEG`, the phase latches SETTLE and never re-enters CREEP/CRUISE, even if a later read shows `|err|` back above the window. (T1, T3)
- [ ] AC5 — In SETTLE, `|err| < tol` commands 0 and increments the settle count; `|err| >= tol` commands `clamp(TURN_SETTLE_KP·err, ±TURN_CREEP_PWM)` toward target and resets the count. A landing exactly at `tol` nudges (does not freeze). (T1, T3)
- [ ] AC6 — An overshoot in ANY phase (err flips sign) produces a reversed wheel command via `copysign(speed, errProg)`, not forward progress. (T1, T3)
- [ ] AC7 — The turn exits (brake per caller, motors off, done bits set) after `TURN_SETTLE_SAMPLES` consecutive strictly-in-band reads. (T3)
- [ ] AC8 — On `TURN_TIMEOUT_MS` with no settle, it brakes, logs the residual, marks done, and does NOT latch a fault or clear the queue. (T3)
- [ ] AC9 — A sub-tolerance turn short-circuits to done with no motion (existing `_exec*` behavior preserved). (T3)
- [ ] AC10 — Pivot turns run the identical profile with only the wheel pattern differing. (T3)
- [ ] AC11 — `.setPower(P)` override sets the CRUISE magnitude to P, not `TURN_CRUISE_PWM`. (T3)
- [ ] AC12 — Straight-line gyro-hold tests stay green — `turnPdStep`/`GyroHoldPolicy.h` untouched. (T4)
- [ ] AC13 — `turnPdStepRaw`/`turnSlewLimit`/`turnCreepFloor` and the retired config constants are gone; full `ctest` + `arduino-cli` compile green. (T2, T3, T4)
- [ ] AC14 — `turn_autotune` compiles and searches `CREEP_WINDOW`/`CREEP_PWM`/`SETTLE_KP`; `turn_tune` + `turn_probe` compile. (T5, T6)

## Contracts & Interfaces

### New policy header — `WRO2026/src/drive/TurnProfilePolicy.h` (owner: T1; consumers: T3, T4, T5)

Pure, header-only `inline` free functions, zero Arduino/Config surface. `#include "TurnPDPolicy.h"` to reuse `turnWrap180`, `turnSettleStep`, `turnSettled` (kept there).

```cpp
// Monotonic ordering CRUISE(0) < CREEP(1) < SETTLE(2). The FSM only advances (max latch).
enum class TurnPhase : uint8_t { CRUISE = 0, CREEP = 1, SETTLE = 2 };

// Natural phase for an absolute progress-frame error. FSM applies phase = max(phase, turnPhaseFor(...)).
//   absErr >  creepWindowDeg                 -> CRUISE
//   toleranceDeg < absErr <= creepWindowDeg   -> CREEP
//   absErr <= toleranceDeg                    -> SETTLE
TurnPhase turnPhaseFor(float absErrDeg, float creepWindowDeg, float toleranceDeg);

// Unsigned magnitude for the driving phases. SETTLE -> 0 (its command is the nudge).
//   CRUISE -> cruisePwm ; CREEP -> creepPwm ; SETTLE -> 0
int16_t turnProfileSpeed(TurnPhase phase, int16_t cruisePwm, int16_t creepPwm);

// SETTLE terminal nudge (signed, progress frame). STRICT in-band test:
//   |errProg| <  toleranceDeg -> 0            (let it sit + count)
//   |errProg| >= toleranceDeg -> clamp(settleKp * errProg, +/- creepPwm)  (sign preserved -> reverses)
float turnSettleNudge(float errProg, float toleranceDeg, float settleKp, int16_t creepPwm);
```

`TurnPhase` uint8 ordering is load-bearing (the `max()` latch depends on it) — do not reorder.

### Drive dispatch (owner: T3)

```
errProg = _turnDeltaSign * turnWrap180(target - yaw)      // progress frame; >0 = short of target
absErr  = fabsf(errProg)
_turnPhase = max(_turnPhase, turnPhaseFor(absErr, TURN_CREEP_WINDOW_DEG, TURN_TOLERANCE_DEG))

if _turnPhase == SETTLE:
    _turnInBandCount = turnSettleStep(absErr, TURN_TOLERANCE_DEG, _turnInBandCount)   // strict < inside
    if turnSettled(_turnInBandCount, TURN_SETTLE_SAMPLES):
        Move(0,0); brake(_turnProfileBrakeOnDone); done; return

if (millis() - _turnStartMs) >= TURN_TIMEOUT_MS:                                       // unchanged
    Move(0,0); brake(_turnProfileBrakeOnDone); log residual; done (no fault, no queue clear); return

if !_turnBrokeAway and fabsf(turnWrap180(yaw - _turnStartYaw)) >= TURN_BREAKAWAY_MOTION_DEG:
    _turnBrokeAway = true

if !_turnBrokeAway:
    cmd = copysignf((float) TURN_BREAKAWAY_PWM, errProg)                               // from-rest kick
elif _turnPhase == SETTLE:
    cmd = turnSettleNudge(errProg, TURN_TOLERANCE_DEG, TURN_SETTLE_KP, TURN_CREEP_PWM) // signed, gated
else:
    cmd = copysignf((float) turnProfileSpeed(_turnPhase, _turnCruisePwm, TURN_CREEP_PWM), errProg)

wheels = turnWheelPattern(_turnDeltaSign, (int16_t) lroundf(cmd), _turnMotorMode)
MiniR4.DriveDC.Move(wheels.left, wheels.right)
```

IMU-read-stall watchdog (`turnImuReadStalled`) is kept verbatim above this block. New `TurnPhase _turnPhase` member seeded to `CRUISE` at dispatch (`_execTurnDeg`/`_execTurnToYaw`); `_turnBrokeAway=false`, `_turnStartYaw=curYaw`, `_turnInBandCount=0`, `_turnCruisePwm` (power override) all kept as today.

### Config surface (owner: T2; consumers: T3, T5)

New (`#ifndef X_DEFAULT`, comment on the `constexpr` line not the `#define`):
- `TURN_CREEP_WINDOW_DEG` = `25.0f` — cruise→creep handoff.
- `TURN_SETTLE_KP` = `4.0f` — settle nudge P gain (PWM/deg), clamped to `±TURN_CREEP_PWM`.

Changed default: `TURN_CREEP_PWM` `20 → 38` (bench-measured Lofaix kinetic sustain floor per ADR-068; still bench-confirmed on bring-up, but must not ship below the sustain floor).
Kept: `TURN_BREAKAWAY_PWM`(46), `TURN_BREAKAWAY_MOTION_DEG`(1.0), `TURN_CRUISE_PWM`(50), `TURN_TOLERANCE_DEG`(3.0), `TURN_SETTLE_SAMPLES`(2), `TURN_TIMEOUT_MS`(3000), `TURN_MIN_RESOLVABLE_DEG`.
Retired: `TURN_KP`, `TURN_KD`, `TURN_SLEW_PWM_PER_TICK`, `TURN_CREEP_THRESH_PWM`.

`ConfigChecks.h`: add asserts `TURN_CREEP_WINDOW_DEG > TURN_TOLERANCE_DEG`, `TURN_CREEP_PWM < TURN_CRUISE_PWM`, `TURN_BREAKAWAY_PWM >= TURN_CREEP_PWM`, `TURN_SETTLE_KP > 0`. Keep the tolerance/min-resolvable/settle and breakaway-motion couplings. Remove only the assert referencing retired `TURN_CREEP_THRESH_PWM`.

### File ownership

| File | Owner | Consumers |
|------|-------|-----------|
| `WRO2026/src/drive/TurnProfilePolicy.h` (new) | T1 | T3, T4, T5 |
| `WRO2026/config/ConfigDrive.h`, `config/ConfigChecks.h` | T2 | T3, T5 |
| `WRO2026/src/drive/Drive.cpp`, `Drive.h`, `TurnPDPolicy.h` (delete 3 fns + docblock) | T3 | T4, T5 |
| `WRO2026/test/test_turn_profile_policy.cpp` (rename), `test_turn_profile_integration.cpp` (rename), `test/CMakeLists.txt` | T4 | — |
| `inoTest/turn_autotune/*`, `inoTest/turn_tune/*` | T5 | — |
| `docs/decisions/078-*.md` (new), `068-*.md`, `077-*.md`, `DECISIONS.md` | T7 | — |
| `tools/path_planner/ctps/profile.py` (docstrings) | T8 | — |

## Error Handling

- **IMU read fails mid-turn** — unchanged (`turnImuReadStalled`: skip tick; sustained → `Fault::IMU_READ_FAILED` + hard-stop + clear queue).
- **From-rest stall** — prevented by the breakaway kick (AC3). If the chassis still can't break away (mechanical), it times out non-faulting.
- **Creep stalls short** (creep PWM below kinetic sustain) — prevented by shipping `TURN_CREEP_PWM=38` and the `>= ` asserts; residual case times out non-faulting.
- **Band-skip overshoot** — `copysign(speed, errProg)` reverses in any phase; SETTLE latch keeps it from re-cruising.
- **Non-convergence timeout** — brake, print residual, done, no fault, no queue clear.

## Testing Strategy

**Levels:** Unit (policy header, zero mocks) + Integration (real `Drive.cpp` via `hal_mocks`, stateful).

| ID | Test Case | Type | Expected |
|----|-----------|------|----------|
| TC1 | `turnPhaseFor` at absErr 30/25/15/3/1 (window 25, tol 3) | Unit | CRUISE, CREEP (==window→CREEP), CREEP, SETTLE (==tol→SETTLE), SETTLE |
| TC2 | `turnProfileSpeed(CRUISE/CREEP/SETTLE, 50, 38)` | Unit | 50, 38, 0 |
| TC3 | `turnSettleNudge` strict in-band (err 2, tol 3) | Unit | exactly 0 |
| TC4 | `turnSettleNudge` at boundary (err 3.0, tol 3) | Unit | nudges (clamp(4·3,±38)=+12), NOT 0 |
| TC5 | `turnSettleNudge` out-of-band clamp (err 20) | Unit | +38 |
| TC6 | `turnSettleNudge` overshoot (err −8) | Unit | −32 (sign preserved) |
| TC7 | Monotonic latch: absErr 30→20→2→**35** through max(phase, turnPhaseFor) | Unit | CRUISE→CREEP→SETTLE→SETTLE (never regress, even above window) |
| TC8 | Ported `turnPdStep` dt-edge cases + `turnWrap180` | Unit | unchanged (kernel kept for gyro-hold) |
| TC9 | Integration: **small turn (8°) dispatched from rest**, tick 1 | Integration | `lastMove` magnitude == `TURN_BREAKAWAY_PWM` (breaks away, not creep) |
| TC10 | Integration: large turn, far from target, post-breakaway | Integration | magnitude == `_turnCruisePwm` |
| TC11 | Integration: `.setPower(80)` large turn, post-breakaway | Integration | magnitude == 80 (override honored) |
| TC12 | Integration: yaw scripted into the creep window, post-breakaway | Integration | magnitude == `TURN_CREEP_PWM` |
| TC13 | Integration: yaw in-band for `TURN_SETTLE_SAMPLES` ticks | Integration | brakes, `isTurnDone()`, `isIdle()`, no fault |
| TC14 | Integration: **band-skip overshoot in CREEP** (script +4°→−4°, no in-band sample) | Integration | dispatched command REVERSES; error does not grow |
| TC15 | Integration: overshoot after SETTLE latched | Integration | reversed command; not done while `|err|>tol` |
| TC16 | Integration: never settles → `TURN_TIMEOUT_MS` | Integration | brakes, done, `!hasFault()`, queue intact |
| TC17 | Integration: sub-tolerance turn (2°, tol 3°) | Integration | short-circuits done, no Move |
| TC18 | Integration: pivot-mode turn converges | Integration | settles; only the driven wheel nonzero in CREEP |
| TC19 | Straight-hold regression (`test_gyro_hold_*`) | Integration | unchanged, green |

**Test data:** `turn_harness` (real `Drive`+`YawSource`, `tickAtYaw`/`enqueueTurnDeg`) adapted. Policy tests need no mocks.
**Run:** `cmake --build build/test && ctest --test-dir build/test --output-on-failure`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`; `uv run inoTest/turn_autotune/build.py --build-only`; `uv run inoTest/turn_tune/build.py --build-only`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | New `TurnProfilePolicy.h` | — | med | `WRO2026/src/drive/TurnProfilePolicy.h` | Pure header: `TurnPhase`, `turnPhaseFor`, `turnProfileSpeed`, `turnSettleNudge` per Contracts (strict `<` in-band nudge). `#include "TurnPDPolicy.h"`. Doxygen each fn. Satisfies AC1,2,4,5,6. |
| T2 | Config + ConfigChecks | — | med | `WRO2026/config/ConfigDrive.h`, `config/ConfigChecks.h` | Add `TURN_CREEP_WINDOW_DEG`=25, `TURN_SETTLE_KP`=4 (rationale docblocks); change `TURN_CREEP_PWM` default 20→38; retire `TURN_KP`/`TURN_KD`/`TURN_SLEW_PWM_PER_TICK`/`TURN_CREEP_THRESH_PWM`; add the 4 new asserts, drop the `TURN_CREEP_THRESH_PWM` assert, keep the rest. Do NOT touch `TURN_BREAKAWAY_*`/`TURN_CRUISE_PWM`/`TURN_TOLERANCE_DEG`/`TURN_SETTLE_SAMPLES`/`TURN_TIMEOUT_MS`. |
| T3 | Rewrite `_updateTurnProfile`; prune Drive state; delete 3 policy fns | T1, T2 | high | `WRO2026/src/drive/Drive.cpp`, `Drive.h`, `TurnPDPolicy.h` | Replace the PD executor with the breakaway→CRUISE→CREEP→SETTLE machine per Contracts (all `copysign(speed, errProg)`). Add `TurnPhase _turnPhase` (seed CRUISE in both `_exec*`); remove `_turnErrOld`,`_turnPrevSpeed` + their seeding; KEEP `_turnBrokeAway`,`_turnStartYaw`,`_turnCruisePwm`. Delete `turnPdStepRaw`,`turnSlewLimit`,`turnCreepFloor` + rewrite the `TurnPDPolicy.h` file docblock (now: `turnWrap180`,`turnPdStep`,`turnSettleStep`,`turnSettled`). Keep timeout, settle, IMU-stall, `turnWheelPattern`. Satisfies AC1-AC11,AC13. |
| T4 | Rewrite host tests | T1, T3 | med | `WRO2026/test/test_turn_profile_policy.cpp` (rename from `test_turn_pd_policy.cpp`), `test_turn_profile_integration.cpp` (rename from `test_turn_pd_integration.cpp`), `test/CMakeLists.txt` | Policy tests TC1–TC8 (port the kept `turnPdStep`/`turnWrap180` cases; drop `turnPdStepRaw`/`turnSlewLimit`/`turnCreepFloor` tests). Integration TC9–TC18. Update CMakeLists for both renames. Do NOT touch `test_gyro_hold_*` (AC12 tripwire). Satisfies all TCs, AC12/AC13. |
| T5 | Rework `turn_autotune` + fix `turn_tune` | T1, T2, T3 | med | `inoTest/turn_autotune/TurnAutotuneHarness.cpp`,`.h`, `inoTest/turn_tune/TurnTuneHarness.cpp` | autotune: replica → profile FSM (call `TurnProfilePolicy.h`); `SPECS` → `CREEP_WINDOW`(seed 25), `CREEP_PWM`(seed 38), `SETTLE_KP`(seed 4) with sane bounds/steps; drop PD language. turn_tune: remove the `Config::TURN_KP/TURN_KD/TURN_BREAKAWAY_PWM/...` symbol prints (lines ~226–241) and the "lower TURN_KP/raise TURN_KD" operator guidance (~337–339) — replace with the profile knobs. Both build-only compile. Satisfies AC14. |
| T6 | Verify `turn_probe` compiles | T2, T3 | low | `inoTest/turn_probe/*` | Links real `Drive`/measures the plant; should compile unchanged. Fix only a compile break from a retired-constant reference. `build.py --build-only`. Satisfies AC14. |
| T7 | ADR-078 + DECISIONS.md | T3 | low | `docs/decisions/078-turn-profile-controller.md` (new), `068-*.md`, `077-*.md`, `DECISIONS.md` | ADR-078 supersedes ADR-068's per-tick-PD/no-trapezoid/no-terminal-stage clause AND ADR-077; explain the two original two-stage failures are fixed (float yaw end-to-end; kinetic creep + retained breakaway), and that the breakaway kick + settle debounce + `TurnSignPolicy` are REUSED from 068, not discarded. Mark 068 PD clause partial + 077 full superseded in the index + status table. |
| T8 | Fix CTPS stale docstrings | T2 | low | `tools/path_planner/ctps/profile.py` | Update the docstrings that name `TURN_CREEP_THRESH_PWM` / describe the PD law (`resolve_turn_profile` logic itself reads only `TURN_CRUISE_PWM` and needs no behavior change). Run the ctps pytest subset. |

Parallelism: T1 ∥ T2 (no blockers). T3 after both. T4 ∥ T5 after T3. T6 after T2+T3. T7 after T3. T8 after T2.

## Notes for Implementer

- **`TURN_CREEP_PWM` is make-or-break.** Shipped default is now 38 (ADR-068's measured Lofaix kinetic sustain floor) instead of the broken 20. It still must be bench-confirmed: on bring-up run **turn_probe first** (its kinetic-sweep measures the sustain floor), set `TURN_CREEP_PWM` a few PWM above it, then turn_tune for landing accuracy. Below the sustain floor, CREEP stalls and every turn hunts to timeout.
- **The breakaway kick is not optional.** Small turns (`|err| ≤ window`, e.g. wallStop's SQUARE re-square, `PIVOT_MIN_TURN_DEG=5` pre-turns) dispatch straight into CREEP; without the from-rest `TURN_BREAKAWAY_PWM` kick they command 20–38 PWM from a dead stop, below the ~46 static scrub floor, and never move. This is ADR-068's documented third failure — the profile keeps 068's breakaway latch verbatim.
- **Everything is `copysign(speed, errProg)`.** Do not dispatch a bare positive magnitude in CRUISE/CREEP — a long tick (`mmL WaitData` can busy-spin ~100 ms) can skip the tolerance band, and a positive-only command would then drive the chassis *away* from the target until it wraps. Pin with TC14.
- **Boundary consistency.** `turnPhaseFor` enters SETTLE at `<=tol` but `turnSettleStep` counts at strict `<tol`; the nudge's in-band test must therefore also be strict `<tol` so a landing exactly at tolerance is nudged in, not frozen. Pin with TC4.
- **`.setPower()` override:** CRUISE magnitude is `_turnCruisePwm` (seeded from the builder override, else `TURN_CRUISE_PWM`) — not the raw config. Pin with TC11.
- Straight-drive is a landmine to avoid: only `turnPdStepRaw`/`turnSlewLimit`/`turnCreepFloor` are deleted; `turnPdStep` stays for `GyroHoldPolicy`. `test_gyro_hold_*` is the tripwire.
- Rollback: confined to worktree branch `worktree-turn-raw-d-term`; the raw-D PD is one revert away, but per the user's decision it is not kept as a runtime toggle.
- Build/verify locally (macOS full toolchain): `ctest` + `arduino-cli compile` + both bench sketches build-only. No `inoTest/` sketch is in CI.
