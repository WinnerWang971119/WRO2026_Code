# Turn Executor Rewrite — PD + Creep Floor + Slew-Limited Ramp-Up Plan

**Date:** 2026-07-16
**Status:** Draft — pending approval

**Goal:** Replace `Drive`'s two-stage bang-bang turn executor with a per-tick PD controller ported from the mentor's competition-proven `M_Turn_angle`, so in-place turns stop overshooting and land inside a tolerance the hardware can actually reach.

**Approach:** Port the mentor's verified control law — `speed = Kp*err + Kd*d(err)`, clamped to the caller's power, with a fixed creep floor where P loses authority — and run it as **one PD step per cooperative tick** inside the existing `_updateTurnProfile`. Add one thing the mentor's light TT chassis does not need: an output slew-rate limit so heavy Lofaix cannot slip its wheels on the initial kick. No integrator, no trapezoid, no terminal stage, no blocking site.

## Scope

- **In scope:**
  - Rewrite `DriveClass::_updateTurnProfile` (`WRO2026/src/drive/Drive.cpp:1059`) as a per-tick PD step.
  - New header-only `WRO2026/src/drive/TurnPDPolicy.h` holding the pure decision math (host-testable).
  - Float yaw end-to-end in the turn path (kills the `int16_t` truncation bug).
  - Kinetic creep floor replacing the one-arg `deadbandSnap` static-floor snap.
  - Slew-rate-limited ramp-up on the emitted command.
  - Config: add the PD/creep/slew/settle knobs; remove the dead two-stage knobs; add the first turn `static_assert`s.
  - Host tests: new unit tests for the policy; a new stateful integration test asserting the real dispatched PWM; update the pre-turn settle idiom for the N-sample debounce.
  - CTPS: replace the two-stage `TurnProfile` duration model.
  - ADR-066 + `DECISIONS.md` index + `ARCHITECTURE.md` "Where PID lives" row.
  - Bench: measure the Lofaix creep floor and achievable tolerance; re-pass B3.2.
  - Revert the bench-only probe harness in `Tasks.cpp`.

- **Out of scope:**
  - **The new external gyro.** It is an MXColor-protocol I2C board (mentor: `mini_i2c_MXcolor_getColor(I2C3, COLOR=H)`, `*0.996` scale-cal, `mod 360`, wrap). Because of ADR-045 it swaps inside `YawSource` alone and touches nothing here. Separate plan once the hardware lands.
  - **Sub-degree (<0.5°) turns.** Reaching it means running the PD loop faster than the tick, which needs a blocking site and an ADR. The design leaves the door open (the loop rate is the only variable) but does not walk through it. Revisit after the new gyro.
  - **Vendor `TurnGyro`.** Hard-rejected on bench evidence; see ADR-066 Why.
  - **`MoveGyro` / GYRO-mode straights.** Untouched.
  - Pivot geometry / B3.8. Pivot turns get the new executor via `_turnMotorMode`, but the pivot trim model (ADR-036) and its unpassed bench gate are unchanged.
  - `MOTOR_BIAS_*` (ADR-011) — a straight-line feedforward; the turn path does not apply it today and still will not.

## Decisions

- **Port the mentor's PD, do not invent a controller** — `Meat_code/wro20260714_大通道gyro+16路2ms_RT.mbr4::M_Turn_angle` is competition-verified on real hardware. Decoded values: `Kp=1`, `Kd=3`, creep `15` below threshold `20`, tolerance `1°`, debounce `3` consecutive in-band samples, timeout `3000 ms`, active brake on exit. Hardware evidence outranks reasoning.
- **Per-tick FSM, NOT a blocking loop** — the mentor's `repeat_until` blocks only because Blockly has no cooperative scheduler. We had agreed to spin the tick callback inside any blocking loop, which caps it at the ~35 Hz tick anyway — so a per-tick FSM step is *behaviourally identical* while remaining non-blocking. This preserves `.async()` (every PathTracker nav turn), ARCHITECTURE iron rule 1, the host-test yaw-parking idiom, and CTPS. No new blocking site, no ADR for one.
- **No integrator** — the creep floor is the feedforward. When `|Kp*err|` drops below the stiction threshold the mentor's law stops modulating and commands a fixed creep, so there is no steady-state stall for an I term to integrate away. An I term on this stiction-limited plant winds up, lurches, and hunts — the vendor's observed failure mode.
- **Add a slew-rate limit (the one deviation from the mentor)** — at 90° error, `Kp*err` saturates instantly, so the first command steps 0 → `power`. On the heavy Lego chassis that risks a traction break, which rotates the chassis about the gripping wheel and translates the axle off the node — invisible to a heading-only loop. One `constrain()` against the previous command fixes it. Ramp-**down** needs nothing: P shrinks with the error. Mirrors the existing tShape start-seed rationale.
- **Kinetic floor, not static** — `_updateTurnProfile` currently calls the one-arg `deadbandSnap(wheels.left)`, which snaps to `PWM_MIN_MAGNITUDE = 35` (the *breakaway* floor). By the tail the chassis is already rotating, so the *kinetic* floor applies. The mentor creeps at 15. This is the single largest contributor to today's overshoot.
- **Tolerance 1.0°, not 0.2°** — the mentor's proven number. `min_step = ω_creep × tick`; at creep 15 (ω≈34°/s) and a 29 ms tick that is ~1.0°, so 0.2° is unreachable at tick rate by any control law. `TURN_TOLERANCE_DEG` stays a Config knob with a `static_assert` tripwire against the bench-measured floor.
- **Trapezoid abandoned** — a clamped P term *is* a decel ramp. `TURN_SLOW_ZONE_DEG` / `TURN_SLOW_SPEED_PWM` become dead and are removed. `TURN_CRUISE_PWM` survives as the builder's default power (the PD's authority clamp).
- **Non-convergence = accept, log, continue** — brake, print the residual, report the turn done. A 1.2° miss must not end a scored run; PathTracker re-anchors heading at line arrivals (ADR-052) anyway.
- **Approach consult outcome** — Fable VETOED the earlier blocking-fast-terminal design and recommended a cooperative "pulse-trim" stop-and-dose terminal. A `/debate` then scored pulse-trim **5.5/10 (Lean No)**, and the mentor's working code superseded both: it is continuous PD, not dosing. Both are recorded as rejected alternatives below. Codex consult unavailable (its forwarder cannot return a background job result to the main thread; run `/codex:status task-mrmelsyp-oo1my3` to read it).

### Rejected alternatives

| Option | Why rejected |
|---|---|
| Vendor `TurnGyro` | Re-zeroes the STM32 yaw accumulator `GetIMUEuler` reads (proof: `target=0` from yaw −84.3 gave `encL=0/encR=0` yet read back −0.3); `Target_D` is relative despite the docstring; mode 0 picks the anchored wheel from turn direction so it cannot express ADR-036 PIVOT_LEFT/RIGHT; mode 1 translates while turning; P-control crawls at small errors (20–27° took 3.3–3.7 s vs 1.7 s for 86°). |
| Pulse-trim (stop-and-dose) | `/debate` Lean No (5.5/10). `MoveDegs` is non-idempotent under the vendor's re-sending retry loop (`Drive_retry_Period=10`) → a lost ACK doubles the dose → overshoot → the bouncing the operator observed on the bench. Also 2–4 stiction break-aways per turn against a recorded "fewer stops is correctness" finding. |
| Blocking fast terminal (~150 Hz) | Buys precision only by starving every module mid-FSM on the async path all ~15 nav turns use; needs an ADR against iron rule 1; breaks the yaw-parking idiom across 12 host targets. Deferred, not dead — it is the only route to sub-degree, and the new gyro may make it worth revisiting. |
| Adding an I term | The creep floor already removes the steady-state error I exists to remove, without windup or hunting. |

## Acceptance Criteria

- [ ] AC1 — `Drive.cpp` contains no `TurnProfileState::CRUISE`/`SLOW` two-stage logic; `_updateTurnProfile` emits `Kp*err + Kd*d(err)` per tick, clamped, slew-limited, creep-floored.
- [ ] AC2 — The turn path reads and compares yaw in `float` end-to-end; no `(int16_t)` cast of a yaw or a turn delta anywhere in `Drive.cpp`.
- [ ] AC3 — The emitted turn PWM is never snapped to `PWM_MIN_MAGNITUDE`; commands below `TURN_CREEP_THRESH_PWM` emit exactly `±TURN_CREEP_PWM`.
- [ ] AC4 — A command cannot change by more than `TURN_SLEW_PWM_PER_TICK` between consecutive ticks.
- [ ] AC5 — A turn exits only after `TURN_SETTLE_SAMPLES` **consecutive** in-band samples; a single out-of-band sample resets the counter to 0.
- [ ] AC6 — A turn that has not converged by `TURN_TURN_TIMEOUT_MS` brakes, prints the residual error, sets `_turnDone`/`_moveDone`, and does **not** latch a fault or clear the queue.
- [ ] AC7 — `Config::TURN_SLOW_ZONE_DEG` and `Config::TURN_SLOW_SPEED_PWM` are removed from the repo; no C++ or Python references remain.
- [ ] AC8 — `ConfigChecks.h` carries `static_assert`s for: `TURN_TOLERANCE_DEG >= TURN_MIN_RESOLVABLE_DEG`, `TURN_CREEP_PWM < TURN_CREEP_THRESH_PWM`, `TURN_CREEP_PWM > 0`, `TURN_SETTLE_SAMPLES >= 1`, `TURN_SLEW_PWM_PER_TICK > 0`.
- [ ] AC9 — `wallStop`'s SQUARE phase still squares and advances to PUSH (the `power=0` sentinel, the "squared" poll, and the already-squared short-circuit all still work).
- [ ] AC10 — `_heldYaw` is still set to the turn target at dispatch.
- [ ] AC11 — The IMU-read-failure watchdog still escalates to `Fault::IMU_READ_FAILED` after `TURN_IMU_STALL_TIMEOUT_MS` of no good read on an `.async()` turn.
- [ ] AC12 — `Drive.turnDeg(deg)` / `turnToYaw(yaw)` builder signatures are byte-identical to today (no new setters), so `test_ctps_mock_superset.py` and the CTPS conformance TU pass untouched.
- [ ] AC13 — Full `ctest` green; full `uv run --extra dev pytest tools/path_planner/tests/` green; `arduino-cli compile` clean.
- [ ] AC14 — `Tasks.cpp::testOne` is empty and `testTwo` is removed; no vendor `TurnGyro`/`BTN_UP` probe code is committed.
- [ ] AC15 — Bench B3.2 re-passed on Lofaix: `turnDeg(90)` within ±2° on 19/20 trials, with the measured residual spread recorded in the ADR.

## Data Model

```cpp
// Config/ConfigDrive.h — new turn knobs (values are the mentor's TT-verified starting points;
// every one carries a TODO bench-tune marker for Lofaix/EV3).
constexpr float   TURN_KP                 = 1.0f;   // mentor-verified
constexpr float   TURN_KD                 = 3.0f;   // mentor-verified; D dominates 3:1, no I by design
constexpr int16_t TURN_CREEP_THRESH_PWM   = 20;     // below |this| the PD stops modulating
constexpr int16_t TURN_CREEP_PWM          = 15;     // ...and emits exactly this (KINETIC floor). TT=15; EV3 TBD
constexpr int16_t TURN_SLEW_PWM_PER_TICK  = 10;     // ramp-up limiter (NOT in the mentor's code — heavy chassis)
constexpr uint8_t TURN_SETTLE_SAMPLES     = 3;      // consecutive in-band samples required to exit
constexpr uint32_t TURN_TIMEOUT_MS        = 3000;   // then brake + log residual + continue
constexpr float   TURN_TOLERANCE_DEG      = 1.0f;   // RETUNED 0.5 -> 1.0 (0.5 was unreachable anyway)
constexpr float   TURN_MIN_RESOLVABLE_DEG = 1.0f;   // BENCH-MEASURED (T1). Tripwire floor for the assert.

// REMOVED: TURN_SLOW_ZONE_DEG, TURN_SLOW_SPEED_PWM  (the clamped P term is the decel ramp)
// KEPT:    TURN_CRUISE_PWM (builder default power = the PD authority clamp), TURN_MODE, TURN_BRAKE,
//          TURN_IMU_STALL_TIMEOUT_MS

// Drive.h — turn FSM state (replaces the CRUISE/SLOW enum machinery)
float    _turnErrOld;        // previous tick's error, for the discrete derivative
float    _turnPrevSpeed;     // previous emitted command, for the slew limit
uint8_t  _turnInBandCount;   // consecutive in-band samples
uint32_t _turnStartMs;       // timeout epoch
float    _turnTargetYaw;     // WIDENED int16_t -> float
```

## Contracts & Interfaces

### `WRO2026/src/drive/TurnPDPolicy.h` (NEW, header-only)

Owner: **T2**. Consumers: **T4** (Drive.cpp), **T5** (unit tests), **T8** (CTPS golden dump).
Pure functions, no Arduino surface — the host-testable seam (the `Drive.cpp` FSM itself has thin coverage; this is why `TurnSignPolicy.h` exists and this follows it).

```cpp
/** @brief Fold a raw heading delta into (-180, +180]. */
float turnWrap180(float deltaDeg);

/** @brief One PD step. @return unclamped command. */
float turnPdStep(float errNew, float errOld, float kp, float kd);

/** @brief Limit |cmd - prev| to slewPerTick. Applied AFTER the power clamp, BEFORE the creep floor. */
float turnSlewLimit(float cmd, float prevCmd, float slewPerTick);

/** @brief Snap |cmd| < threshPwm to copysign(creepPwm, cmd). cmd == 0 stays 0. */
float turnCreepFloor(float cmd, int16_t threshPwm, int16_t creepPwm);

/** @brief One debounce step. @return new count; 0 on any out-of-band sample. */
uint8_t turnSettleStep(float absErr, float toleranceDeg, uint8_t count);

/** @brief Exit predicate. @return true when count has reached samplesRequired. */
bool turnSettled(uint8_t count, uint8_t samplesRequired);
```

**Operator order is a contract** (T4 and T8 must apply it identically):
`pdStep` → clamp to `±power` → `slewLimit` → `creepFloor` → emit `Move(+cmd, -cmd)` via `turnWheelPattern`.

### Retained seams (unchanged — do not redefine)

- `TurnSignPolicy.h::turnWheelPattern(sign, pwm, mode)` — still owns wheel dispatch for COUNTER_ROT / PIVOT_LEFT / PIVOT_RIGHT. Owner: existing. Consumer: T4.
- `YawSource.readYawDeg(double&) -> bool` / `toHwHeading` — the sole raw-yaw owner (ADR-045). Consumer: T4. **The new gyro swaps here later and nowhere else.**
- `WROMath.h::turnImuReadStalled(nowMs, lastGoodMs, timeoutMs)` — the async IMU watchdog predicate. Consumer: T4. Retained.
- `Drive.h` builder surface — **frozen**. No new setters (AC12).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/src/drive/TurnPDPolicy.h` | T2 | T4, T5, T8 |
| `WRO2026/config/ConfigDrive.h` | T3 | T4, T8 |
| `WRO2026/config/ConfigChecks.h` | T3 | — |
| `WRO2026/src/drive/Drive.cpp` + `Drive.h` | T4 | — |
| `WRO2026/test/test_turn_pd_policy.cpp` | T5 | — |
| `WRO2026/test/test_turn_pd_integration.cpp` | T6 | — |
| `WRO2026/test/test_path_tracker_harness.h` | T7 | — |
| `WRO2026/test/CMakeLists.txt` | T5 (then T6 serialized behind it) | — |
| `tools/path_planner/ctps/profile.py` + `tests/test_ctps_profile.py` | T8 | — |
| `docs/decisions/066-*.md`, `DECISIONS.md`, `ARCHITECTURE.md` | T9 | — |
| `WRO2026/Tasks.cpp`, `Tasks.h` | T10 | — |

### Naming

- ADR number: **066** (065 is the highest on disk; **067 is already claimed** by the unmerged `fix/issues-141-159-hardening` — do not take it).
- Branch: `feat/turn-pd-controller`, cut from **`origin/main`**.
- Config prefix: `TURN_*` in `ConfigDrive.h`. Note `NAV_TURN_TOLERANCE_DEG` (`ConfigNav.h`) is a *different* constant (PathTracker's dispatch gate) and is untouched.

## Error Handling

- **IMU read fails, transient** — skip the tick, hold the previous `Move(L,R)`. Do **not** act on a fabricated 0.0 yaw.
- **IMU read fails, sustained** — `turnImuReadStalled` escalates after `TURN_IMU_STALL_TIMEOUT_MS`: hard stop, `brake(true)`, latch `Fault::IMU_READ_FAILED`, clear the queue. An `.async()` turn has no drain deadline behind it, so this watchdog is the only bound. Retained verbatim.
- **Turn does not converge by `TURN_TIMEOUT_MS`** — `Move(0,0)`, `brake(_turnProfileBrakeOnDone)`, `Serial.print` the residual, set `_turnDone`/`_moveDone`. **No fault, no queue clear.**
- **Turn dispatched already in-band** — the existing tolerance early-out short-circuits at dispatch (sets `_heldYaw`, marks done). `wallStop` SQUARE depends on this; preserve.
- **`Move(L,R)` returns non-OK** — same as today (the vendor wrapper already retries; `Move` is idempotent so a re-send is harmless — unlike `MoveDegs`).

## Testing Strategy

**Levels:** Unit (policy), Integration (real `Drive.cpp` + mock HAL), Bench (hardware).

| ID | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | `turnWrap180` at ±180, ±181, ±359, 0 | Unit | Folds into (−180, +180]; +180 stays +180, −180 → +180 |
| TC2 | `turnPdStep` with err=90, errOld=90 (no motion) | Unit | Returns `Kp*90` exactly (D term contributes 0) |
| TC3 | `turnPdStep` with err shrinking fast | Unit | D term is negative and opposes P (damping) |
| TC4 | `turnSlewLimit(50, 0, 10)` | Unit | Returns 10 — a 0→50 step is clipped to the slew rate |
| TC5 | `turnSlewLimit(12, 10, 10)` | Unit | Returns 12 — within the rate, passes through |
| TC6 | `turnCreepFloor(5, 20, 15)` / `(-5, 20, 15)` / `(19, 20, 15)` / `(25, 20, 15)` / `(0, 20, 15)` | Unit | 15 / −15 / 15 / 25 / **0** (exact zero stays zero) |
| TC7 | `turnSettleStep` in-band 3× then out-of-band | Unit | 1, 2, 3, then **0** — a single out-of-band sample resets |
| TC8 | `turnSettled(2, 3)` / `turnSettled(3, 3)` | Unit | false / true |
| TC9 | Full turn: park `mockYawDeg` at target, tick | Integration | Exits only after `TURN_SETTLE_SAMPLES` ticks, not the first; `MockDriveDC.lastBrake == true` |
| TC10 | Turn from 90° error, first tick | Integration | `abs(lastMoveSyncL)` ≤ `TURN_SLEW_PWM_PER_TICK` — proves the ramp-up limit reaches the actuator (not just the policy) |
| TC11 | Turn with yaw scripted to a 0.5° residual (inside 1.0° band) | Integration | Converges and brakes; never emits `PWM_MIN_MAGNITUDE` |
| TC12 | Turn with yaw scripted to a 5° residual that never closes | Integration | At `TURN_TIMEOUT_MS`: brakes, `isTurnDone()` true, `hasFault()` **false**, queue **not** cleared |
| TC13 | `mmL.mockResult = ERROR` for the whole turn | Integration | After `TURN_IMU_STALL_TIMEOUT_MS`: `Fault::IMU_READ_FAILED`, queue cleared |
| TC14 | `mmL.mockResult = ERROR` for one tick only | Integration | Tick skipped, prior `Move(L,R)` held, no fault, turn still completes |
| TC15 | `wallStop` SQUARE → PUSH with a non-zero wall heading | Integration | Squares, then PUSH seeds; `power=0` sentinel resolves to `TURN_CRUISE_PWM` |
| TC16 | `wallStop` SQUARE already squared | Integration | Short-circuits; PUSH on the very next tick |
| TC17 | Pivot turn (`setMode(1)`) | Integration | Only the free wheel is commanded; anchored wheel PWM == 0 |
| TC18 | `grep -r "TURN_SLOW_ZONE_DEG\|TURN_SLOW_SPEED_PWM"` | Grep | 0 matches repo-wide |
| TC19 | CTPS turn duration vs config | Python | `test_ctps_profile` passes against the new model; no parse of the removed constants |
| B3.2 | `turnDeg(90)` × 20 on Lofaix, IMU logged | Bench | ≤1 trial outside ±2°; residual spread recorded |
| B-T1 | Creep sweep: command PWM 10..30 from rest and while rolling | Bench | Lowest PWM that reliably rotates → `TURN_CREEP_PWM`; ω at that PWM → `TURN_MIN_RESOLVABLE_DEG = ω × tick` |

**Test data:** `hal_mocks/MatrixMiniR4.h` `MockMMLower::mockYawDeg` (script the yaw sequence per tick), `mockResult` (script transport failures), `MockDriveDC::lastMoveSyncL/R` + `lastBrake` (assert the real dispatched PWM). Integration targets link the REAL `Drive.cpp`, link `path_mocks.cpp` **not** `hal_mocks.cpp` (ODR on the `Drive` global), do **not** force-include `hal_mocks/Drive.h` (shared `DRIVE_H` guard), and put `${SKETCH_DIR}` on the include path FIRST.

**Run commands:**
```
cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure
uv run --extra dev pytest tools/path_planner/tests/
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/
```

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Bench-measure the Lofaix creep floor + min resolvable step | — | low | `WRO2026/Tasks.cpp` (throwaway, reverted by T10) | Temporary `testOne` harness: sweep `Move(p,-p)` for p in 10..30 from rest AND while already rotating; record the lowest p that reliably breaks away and the lowest that sustains rotation; time a 90° rotation at that p to get ω; read yaw 100× stationary for the noise spread. Outputs feed T3's `TURN_CREEP_PWM`, `TURN_CREEP_THRESH_PWM`, `TURN_MIN_RESOLVABLE_DEG`. Report numbers; do NOT commit the harness. Satisfies B-T1. |
| T2 | Create `TurnPDPolicy.h` | — | med | `WRO2026/src/drive/TurnPDPolicy.h` | Header-only, `#ifndef TURN_PD_POLICY_H`. Implement exactly the six functions in Contracts & Interfaces with Doxygen on each. No Arduino include, no `Config.h` include — every tunable arrives as a parameter (this is what makes it host-unit-testable). Document the operator order in the file docblock. Satisfies AC1 groundwork. |
| T3 | Config: add PD knobs, remove two-stage knobs, add asserts | T2 | med | `WRO2026/config/ConfigDrive.h`, `WRO2026/config/ConfigChecks.h` | Add the constants in Data Model with Doxygen + `TODO bench-tune (T1)` markers, citing the mentor's file as the source of the TT-verified values. REMOVE `TURN_SLOW_ZONE_DEG` + `TURN_SLOW_SPEED_PWM`. Retune `TURN_TOLERANCE_DEG` 0.5f → 1.0f and fix the false claim at `ConfigDrive.h:497` that `TURN_SLOW_SPEED_PWM` equals `PWM_MIN_MAGNITUDE`. Add the five `static_assert`s to `ConfigChecks.h` (it currently has ZERO turn asserts). Keep each `#define X_DEFAULT` free of trailing `//` comments (breaks the Python config parser). Satisfies AC7, AC8. |
| T4 | Rewrite `_updateTurnProfile` as a per-tick PD step | T2, T3 | high | `WRO2026/src/drive/Drive.cpp`, `WRO2026/src/drive/Drive.h` | Replace the CRUISE/SLOW/BRAKE enum machinery with: read yaw via `YawSource.readYawDeg` into a **float** (no `int16_t` cast — AC2); `err = turnWrap180(_turnTargetYaw - yaw)`; debounce via `turnSettleStep`/`turnSettled` → on settle `Move(0,0)` + `brake(_turnProfileBrakeOnDone)` + `_turnDone=_moveDone=true`; timeout check → same but `Serial.print` the residual and DO NOT fault (AC6); else `turnPdStep` → clamp `±_turnCruisePwm` → `turnSlewLimit(_turnPrevSpeed)` → `turnCreepFloor` → `turnWheelPattern(_turnDeltaSign, pwm, _turnMotorMode)` → `Move(L,R)` **without** `deadbandSnap` (AC3). Widen `_turnTargetYaw` to float; add `_turnErrOld`, `_turnPrevSpeed`, `_turnInBandCount`, `_turnStartMs`; seed all of them in BOTH `_execTurnDeg` and `_execTurnToYaw`. PRESERVE: `_heldYaw = target` at dispatch (AC10), the tolerance early-out short-circuit (AC9/TC16), the `power > 0 ? power : TURN_CRUISE_PWM` sentinel wallStop relies on, the `turnImuReadStalled` escalation (AC11), and the `_turnProfileState != IDLE` signal `_updateWallStop` polls at Drive.cpp:1413 (keep an IDLE/ACTIVE state even though CRUISE/SLOW are gone). Do NOT add builder setters (AC12). Satisfies AC1–AC6, AC9–AC12. |
| T5 | Unit tests for `TurnPDPolicy.h` | T2 | low | `WRO2026/test/test_turn_pd_policy.cpp`, `WRO2026/test/CMakeLists.txt` | New ctest target linking only the header + `MOCK_SOURCES` (no Drive, no MatrixMiniR4) — mirror the `test_turn_sign_policy` target recipe. Implement TC1–TC8. Note TC6's exact-zero case and TC7's reset-on-miss are the two that catch real bugs. |
| T6 | Stateful PD integration test | T4, T5 | med | `WRO2026/test/test_turn_pd_integration.cpp`, `WRO2026/test/CMakeLists.txt` | New integration target linking the REAL `Drive.cpp` per the issue-#106 recipe (see Testing Strategy). Implement TC9–TC17. This is the FIRST stateful test of the turn FSM — today's coverage is pure predicates + end-to-end yaw parking, with nothing in between. TC10 must assert `MockDriveDC.lastMoveSyncL/R` (the dispatched value), not the policy return — a LineFollower-regime mock records the PRE-snap value and false-greens. Serialize the `CMakeLists.txt` edit behind T5. |
| T7 | Update the pre-turn settle idiom for the N-sample debounce | T4 | med | `WRO2026/test/test_path_tracker_harness.h` | `expectPreTurn` parks `mockYawDeg` and ticks until `Drive.isIdle()`. With the debounce the turn now needs ≥`TURN_SETTLE_SAMPLES` ticks in-band before it idles — verify `kMaxHandshakeTicks` still covers it and bump if not. Update the CONTRACT docblock (it describes the retired "custom TurnGyro profile" two-stage machine). Also re-check `test_path_tracker_settle.cpp`, which parks yaw inline every tick. Do NOT model the FSM in the mock — script the yaw sequence. |
| T8 | Replace the CTPS turn duration model | T3 | med | `tools/path_planner/ctps/profile.py`, `tools/path_planner/tests/test_ctps_profile.py` | `TurnProfile` (profile.py:1141-1193) hand-mirrors the dead two-stage FSM and parses `TURN_CRUISE_PWM`/`TURN_SLOW_SPEED_PWM`/`TURN_SLOW_ZONE_DEG` (:238-240). Two of those no longer exist. Replace with a **display-only** constant-rate + settle-allowance model derived from `TURN_CRUISE_PWM` and a nominal creep tail; a PD loop's true duration is plant-dependent and NOT closed-form, so document it as render-layer-only (same posture ADR-061 took for `HOP_SETTLE_MS`). Fix the stale docstring at :1170-1172 claiming `PWM_MIN_MAGNITUDE = 50` (it is 35). Update `test_ctps_profile.py:644-649`, whose two pinned regimes (default constant-rate, perturbed two-stage) no longer exist. Satisfies AC7 (Python half), AC13, TC19. |
| T9 | ADR-066 + index + architecture row | T4 | low | `docs/decisions/066-turn-pd-controller.md`, `DECISIONS.md`, `ARCHITECTURE.md` | Write ADR-066 in the house shape (~30-60 lines: Rule / Why / How to apply / Trigger to revisit / See also). Rule: the turn executor is a per-tick PD with a creep floor and slew-limited ramp-up; no integrator; tolerance is bench-floored. Why: the two-stage bang-bang's band was unreachable (int16 truncation) and its slow stage ran at the static breakaway floor; the mentor's PD is competition-verified. Mark the 2026-05-17 two-stage-profile decision superseded. Trigger to revisit: the new external gyro landing, or wanting sub-degree (which needs a faster-than-tick loop and its own ADR). Record the B3.2 residual spread from T11. Update `DECISIONS.md` (index line + status table) and `ARCHITECTURE.md`'s "Where PID lives" row — the Turn row currently says "STM32 firmware (`TurnGyro`). Gains in `Config::TURN_PID_*`", which is doubly wrong. Use **066**; 067 is taken by an unmerged branch. |
| T10 | Revert the bench probe from `Tasks.cpp` | T1 | low | `WRO2026/Tasks.cpp`, `WRO2026/Tasks.h` | Restore `testOne` to an empty `return true;` body, remove the `TurnProbeStep`/`PROBE_STEPS`/`turnProbe*`/`moveGyroProbeRun` file-locals and the `src/nav/YawSource.h` include added for them. Remove `testTwo` from the registry, from `Tasks.h`, and its stub body. Verify `arduino-cli compile` is clean and the `ctps_dump` target still builds (it compiles the WHOLE `Tasks.cpp`). Satisfies AC14. |
| T11 | Bench: tune gains and re-pass B3.2 on Lofaix | T4, T1 | med | `WRO2026/config/ConfigDrive.h` | Flash and run `turnDeg(90)` ×20 logging the IMU residual. Start from the mentor's `Kp=1`/`Kd=3` but treat them as TT-measured: Lofaix is heavier, so expect to raise `TURN_CREEP_PWM` and possibly `TURN_KD`. Tune `TURN_SLEW_PWM_PER_TICK` down until the initial kick no longer slips (watch the wheels, not the log). Commit the measured values with the battery voltage in the message (repo convention). **Do NOT copy the mentor's motor mapping** — his file is `PINL_REV=false/PINR_REV=true` (TT rover); Lofaix/EV3 needs `(true, false)`. Satisfies AC15. |

## Notes for Implementer

- **The mentor's decoded source of truth** is `Meat_code/wro20260714_大通道gyro+16路2ms_RT.mbr4::M_Turn_angle` (Blockly XML; parse with ElementTree, ns `{http://www.w3.org/1999/xhtml}`; `procedures_prototype` is a **shadow** not a block; inner `math_number` values are vestigial SHADOW defaults overridden by the real operand — read them wrong and you will get the gains backwards). The full decode is in the workspace memory `reference-mentor-turn-pd-mbr4`.
- **The creep floor is not `deadbandSnap`.** `deadbandSnap` snaps small commands UP to the *static* breakaway floor (35). The creep floor snaps them to the *kinetic* creep (15) because the chassis is already rotating by then. Do not reuse `deadbandSnap` in the turn path — that reintroduces the bug this plan exists to fix.
- **`turnCreepFloor(0, ...)` must return exactly 0.** A commanded stop is a stop. Only *non-zero* small commands creep.
- **Slew limiting is applied AFTER the power clamp and BEFORE the creep floor.** Reordering changes behavior: clamp-then-slew bounds the ramp to the authority ceiling; creep-then-slew would let the slew limiter erase the creep floor on the first tick out of rest.
- **Ramp-down needs no code.** P shrinks with the error; the slew limiter will not bind on the way down. Do not add a symmetric decel ramp.
- Drive builders are move-only RAII — emit as one discarded-temporary statement, never `auto b = Drive.turnDeg(90)...`.
- Retuning a drive constant fans out to four surfaces (host goldens `tshape_profile_dump`/`chain_speed_dump`, config-parse pins, behavioral tests, and the silent `sim/runtime.py` hand-mirror). The turn constants are believed to touch only `ctps/profile.py` (T8) — `sim/runtime.py` turns instantly and mirrors no `TURN_*`, and `test_config_parse*` pins none — but run the FULL pytest and the FULL ctest before pushing, not a subset.
- The parity trace records `{kind, body_heading_deg, wheel}` only, so a Drive-internal executor change alters no parity record. Do not "fix" parity if it goes red — that would mean something else broke.
- **Rollback:** the whole change is behind `_updateTurnProfile` + `TurnPDPolicy.h` + the Config block. Reverting T3 and T4 together restores the old behavior; nothing else in the tree depends on the new constants except T8's render model.
- **Adjacent bug, NOT in scope but do not re-break:** `isPrevTaskDone()` (`MiniR4DriveDC.h:693`) reads an uninitialized `bool[2]` and ignores the result code. Turns do not use it (the `update()` drain guard excludes them on purpose — the "W1 fix"). Keep it that way.
