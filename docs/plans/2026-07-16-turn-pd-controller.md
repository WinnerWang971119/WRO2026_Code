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
  - ADR-068 + `DECISIONS.md` index + `ARCHITECTURE.md` "Where PID lives" row.
  - Bench: measure the Lofaix creep floor and achievable tolerance; re-pass B3.2.
  - Revert the bench-only probe harness in `Tasks.cpp`.

- **Out of scope:**
  - **The new external gyro.** It is an MXColor-protocol I2C board (mentor: `mini_i2c_MXcolor_getColor(I2C3, COLOR=H)`, `*0.996` scale-cal, `mod 360`, wrap). Because of ADR-045 it swaps inside `YawSource` alone and touches nothing here. Separate plan once the hardware lands.
  - **Chasing the operator's 0.2° target.** At the measured 8 ms tick the min step is ~0.27°, so 0.2° needs ω at creep to come in under ~25°/s. T1 measures it; if it does, tightening is a one-line Config change with no design impact. Anything below the measured floor needs a faster-than-tick loop (blocking site + ADR) and is deliberately deferred to after the new gyro. **±0.5° is NOT out of scope** — it looks reachable and T11 should tune toward it.
  - **Vendor `TurnGyro`.** Hard-rejected on bench evidence; see ADR-068 Why.
  - **`MoveGyro` / GYRO-mode straights.** Untouched.
  - Pivot geometry / B3.8. Pivot turns get the new executor via `_turnMotorMode`, but the pivot trim model (ADR-036) and its unpassed bench gate are unchanged.
  - `MOTOR_BIAS_*` (ADR-011) — a straight-line feedforward; the turn path does not apply it today and still will not.

## Decisions

- **Port the mentor's PD, do not invent a controller** — `Meat_code/wro20260714_大通道gyro+16路2ms_RT.mbr4::M_Turn_angle` is competition-verified on real hardware. Decoded values: `Kp=1`, `Kd=3`, creep `15` below threshold `20`, tolerance `1°`, debounce `3` consecutive in-band samples, timeout `3000 ms`, active brake on exit. Hardware evidence outranks reasoning.
- **⚠️ MEASURED tick: ~8 ms / ~124 Hz.** `inoTest/tick_bench` on the real robot, 2026-07-16: `TICK us min=7995 avg=8056 max=11484`, verdict GOOD. **Every loop-rate claim in this plan uses this number, NOT the ~35 Hz a prior memory predicted from per-read costs** — that estimate was ~3.5× pessimistic and has been corrected. This one number decides the architecture (see the next bullet). Note `max=11484`: the tick jitters ~8–11.5 ms, which is exactly why `dt` must be measured per step, not assumed constant.
- **⚠️ The D term MUST be time-normalized.** The mentor computes `error = error_new - error_old`, a **per-iteration** difference, so his effective damping is `Kd × ω × loop_period`. At our 8 ms vs his ~6 ms, raw `Kd=3` applies ~1.33× his damping — survivable, but it silently re-scales the moment the tick jitters to 11.5 ms or the new external gyro speeds the loop up. Fix: divide by the measured `dt` so `Kd` carries real units of PWM per (deg/s) and is loop-rate independent: `rate = (errNew - errOld) / dtSec; speed = Kp*err + Kd*rate`. The mentor's equivalent normalized gain is `Kd_norm ≈ 3 × 0.006 s ≈ 0.018` — sanity check: at ω=90°/s that yields `0.018 × 90 ≈ 1.6`, matching his per-iteration `3 × 0.54 ≈ 1.6`. Use the **measured** per-tick `dt` (`millis()` delta), never a hard-coded constant.
- **Per-tick FSM, NOT a blocking loop — settled by measurement, not preference.** The mentor's `repeat_until` blocks only because Blockly has no cooperative scheduler; his loop is fast because it is *dedicated* (one gyro read + one motor write per iteration), not because his hardware is quicker. At our measured 8 ms tick a per-tick step lands in the same regime: min step **~0.27°** vs his **~0.20°**. Blocking would buy 0.27° → 0.20° — **1.4×** — in exchange for a new ADR against ARCHITECTURE iron rule 1, a frozen tick for the turn's 1–2 s duration (so `Mission.update()` could not abort mid-turn), and rewriting the yaw-parking idiom across 12 host targets. **Not worth it.** The cooperative FSM preserves `.async()` (every PathTracker nav turn), the host-test idiom, and CTPS. Revisit only if the external gyro makes a dedicated loop meaningfully faster AND sub-0.27° is proven necessary.
- **No integrator** — the creep floor is the feedforward. When `|Kp*err|` drops below the stiction threshold the mentor's law stops modulating and commands a fixed creep, so there is no steady-state stall for an I term to integrate away. An I term on this stiction-limited plant winds up, lurches, and hunts — the vendor's observed failure mode.
- **Add a slew-rate limit (the one deviation from the mentor)** — at 90° error, `Kp*err` saturates instantly, so the first command steps 0 → `power`. On the heavy Lego chassis that risks a traction break, which rotates the chassis about the gripping wheel and translates the axle off the node — invisible to a heading-only loop. One `constrain()` against the previous command fixes it. Ramp-**down** needs nothing: P shrinks with the error. Mirrors the existing tShape start-seed rationale.
- **Kinetic floor, not static** — `_updateTurnProfile` currently calls the one-arg `deadbandSnap(wheels.left)`, which snaps to `PWM_MIN_MAGNITUDE = 35` (the *breakaway* floor). By the tail the chassis is already rotating, so the *kinetic* floor applies. The mentor creeps at 15. This is the single largest contributor to today's overshoot.
- **Tolerance: 0.5° (operator's call) — keeps today's value, beats the mentor's robot.** `TURN_TOLERANCE_DEG` stays at `origin/main`'s existing `0.5f`; this plan does **not** retune it. Note what changes: 0.5° is unreachable *today* only because of the `int16_t` truncation (AC2), not because of physics — fixing the float path makes the number the config already claims actually attainable. `min_step = ω_creep × tick`; at the measured 8 ms tick and creep 15 (ω≈34°/s) that is **~0.27°**, so a ±0.5° band is ~1.0° wide and the chassis crosses it in ~3.7 samples. **This is tighter than the mentor's own 1.0°** and works because our tick is nearly his loop rate.
- **⚠️ Tolerance and the debounce are COUPLED — this is the plan's sharpest edge.** The exit needs `TURN_SETTLE_SAMPLES` *consecutive* in-band samples, and only `2*TURN_TOLERANCE_DEG / min_step` samples land in-band per crossing. At 0.5° that is ~3.7 against a required 3 — only **1.2× margin**. If T1 measures ω at creep higher than ~45°/s (min step > 0.33°), fewer than 3 samples land, the debounce can never be satisfied, and **every turn hunts to its 3 s timeout — silently, since timeout is non-faulting by design**. Mitigations, in order of preference: (a) drop `TURN_SETTLE_SAMPLES` to 2 (→1.85× margin, still proves settledness); (b) lower `TURN_CREEP_PWM` if T1 shows the chassis sustains rotation below 15; (c) widen the tolerance. The `static_assert` below makes this a build failure rather than a field mystery. The weakest input is ω≈34°/s — **extrapolated linearly from ω=92°/s at PWM 40, and PWM→ω is decidedly non-linear near stiction, so the true value could fall either way.** T1 measures it; T11 tunes against it.
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
- [ ] AC8 — `ConfigChecks.h` carries `static_assert`s for: (a) `TURN_TOLERANCE_DEG >= TURN_MIN_RESOLVABLE_DEG` — the band must be at least one physical step wide; (b) **`(2.0f * TURN_TOLERANCE_DEG) / TURN_MIN_RESOLVABLE_DEG > TURN_SETTLE_SAMPLES`** — enough samples must land in-band per crossing for the debounce to ever be satisfiable, else every turn hunts to its timeout **silently**; (c) `TURN_CREEP_PWM < TURN_CREEP_THRESH_PWM`; (d) `TURN_CREEP_PWM > 0`; (e) `TURN_SETTLE_SAMPLES >= 1`; (f) `TURN_SLEW_PWM_PER_TICK > 0`; (g) `TURN_SLEW_PWM_PER_TICK >= TURN_CREEP_PWM` — a slew below the creep floor is unreachable, so the ramp-up limit would be a lie; (h) `TURN_TIMEOUT_MS < DRIVE_DRAIN_TIMEOUT_MS_PER_CMD` — the turn's own timeout must fire before the blocking drain's 10 s budget. Each carries a message naming the measured input and the fix. **(b) is the one that matters most: it converts the plan's sharpest failure mode into a build error.**
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
constexpr float   TURN_KP                 = 1.0f;   // mentor-verified (dimensionless: PWM per deg)
// TIME-NORMALIZED derivative gain: PWM per (deg/s). NOT the mentor's raw 3 — his is a
// per-ITERATION difference at his ~6 ms loop; ours is a MEASURED ~8 ms and jitters to 11.5 ms,
// so the raw gain re-scales with the tick. Normalizing by measured dt makes it rate-independent.
// 0.018 ~= 3 * 0.006 s reproduces his damping at any loop rate.
constexpr float   TURN_KD                 = 0.018f; // no I by design — the creep floor replaces it
constexpr int16_t TURN_CREEP_THRESH_PWM   = 20;     // below |this| the PD stops modulating
constexpr int16_t TURN_CREEP_PWM          = 15;     // ...and emits exactly this (KINETIC floor). TT=15; EV3 TBD
constexpr int16_t TURN_SLEW_PWM_PER_TICK  = 10;     // ramp-up limiter (NOT in the mentor's code — heavy chassis)
constexpr uint8_t TURN_SETTLE_SAMPLES     = 3;      // consecutive in-band samples required to exit
constexpr uint32_t TURN_TIMEOUT_MS        = 3000;   // then brake + log residual + continue
// UNCHANGED from origin/main. 0.5 is unreachable TODAY only because of the int16_t yaw
// truncation (AC2); the float path makes the value the config already claims attainable.
// Tighter than the mentor's 1.0 -- viable because our measured 8 ms tick ~= his 6 ms loop.
constexpr float   TURN_TOLERANCE_DEG      = 0.5f;
constexpr float   TURN_MIN_RESOLVABLE_DEG = 0.30f;  // PLACEHOLDER -- BENCH-MEASURED by T1 as
                                                    // omega_at_creep * measured_tick. Both asserts
                                                    // below key off it; T3 must not ship a guess.

// REMOVED: TURN_SLOW_ZONE_DEG, TURN_SLOW_SPEED_PWM  (the clamped P term is the decel ramp)
// KEPT:    TURN_CRUISE_PWM (builder default power = the PD authority clamp), TURN_MODE, TURN_BRAKE,
//          TURN_IMU_STALL_TIMEOUT_MS

// Drive.h — turn FSM state (replaces the CRUISE/SLOW enum machinery)
float    _turnErrOld;        // previous tick's error, for the discrete derivative
uint32_t _turnLastStepMs;    // previous step's millis(), for the time-normalized dt
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

/**
 * @brief One PD step with a TIME-NORMALIZED derivative.
 * @param dtSec Measured elapsed time since the previous step. Guard: if dtSec <= 0 the D term
 *              contributes 0 (never divide by zero on a same-millis re-entry).
 * @return unclamped command. kd is PWM per (deg/s) — NOT the mentor's per-iteration gain.
 */
float turnPdStep(float errNew, float errOld, float dtSec, float kp, float kd);

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
`pdStep` → clamp to `±power` → `slewLimit` → `creepFloor` → emit via `turnWheelPattern`.

### ⚠️ Sign frame — pinned here because getting it wrong silently reverses every turn

`TurnSignPolicy.h:107` documents `turnWheelPattern(sign, pwmMag, mode)`'s `pwmMag` as an
**UNSIGNED magnitude**, and it applies the sign itself (`L = -sign*pwmMag`). Feeding it a
*signed* PD command **plus** the dispatch-frozen `_turnDeltaSign` double-applies the sign — a
CW turn (sign=−1, cmd<0) emits L<0/R>0, which is CCW. Feeding `|cmd|` instead makes overshoot
reversal impossible. The mentor has no sign variable at all: he emits `ddc_on(speed, -speed)`
with the signed speed directly.

**The contract: the PD runs in the PROGRESS frame.**
- `errProgress = _turnDeltaSign * turnWrap180(_turnTargetYaw - yaw)` — positive means "still short
  of the target", negative means "overshot".
- The PD, clamp, slew, and creep all operate on `errProgress`, producing `cmdProgress`. Nominally
  `cmdProgress > 0`; it goes negative exactly when the chassis has overshot and must reverse.
- Emit `turnWheelPattern(_turnDeltaSign, cmdProgress, mode)` — the policy re-applies the world sign
  **once**. A negative `cmdProgress` therefore correctly reverses the wheel pattern.
- `_turnDeltaSign` stays frozen at its dispatch value; **do not** recompute it per tick (that is the
  passed-target oscillation the old code had to patch around).

### ⚠️ Slew is RAMP-UP ONLY, and it starts at the breakaway floor

Two traps the naive `constrain(cmd, prev-S, prev+S)` walks into:

1. **Reversal deadlock.** With `prev` holding the *emitted* (post-creep) `+15`, an overshoot wants a
   negative command; slew bounds it to `+15-S`, and `creepFloor` snaps that straight back to `+15`
   (copysign preserves the wrong sign). The command can **never cross zero** — it latches in the
   wrong direction until the 3 s timeout, on **every** overshoot. Fix: the slew limiter binds **only
   when `|cmd|` is growing AND `sign(cmd) == sign(prev)`.` A sign change or a shrinking magnitude
   passes through unlimited (decel and reversal must never be rate-limited).
2. **Ramping from zero is meaningless.** Below the *static breakaway* floor (`PWM_MIN_MAGNITUDE`=35)
   the chassis does not move at all, so a 0→10→20 ramp is dead air, not a gentle start (this is the
   documented `deadbandSnap`/tShape start-seed trap). **Seed `_turnPrevSpeed` at
   `±PWM_MIN_MAGNITUDE` at dispatch**, so the ramp runs breakaway→`power`, which is the interval
   where slew limiting actually buys traction. Stiction means the first *effective* command is 35
   regardless; slew's job is only to stop it jumping 35→100.

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
| `docs/decisions/068-turn-pd-controller.md`, `DECISIONS.md`, `ARCHITECTURE.md` | T9 | — |
| `WRO2026/Tasks.cpp`, `Tasks.h` | T10 | — |

### Naming

- ADR number: **068**. 065 is the highest on `main`, but the unmerged `fix/issues-141-159-hardening` already claims BOTH **066** (`066-encoder-source.md`) and **067** (`067-drive-fault-clear-cascade.md`) — verified across all refs. Do not take either.
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
| TC2 | `turnPdStep` with err=90, errOld=90, dtSec=0.029 (no motion) | Unit | Returns `Kp*90` exactly (D term contributes 0) |
| TC3 | `turnPdStep` with err shrinking fast | Unit | D term is negative and opposes P (damping) |
| TC3b | **`turnPdStep` sampled at a FIXED angular rate ω across two tick periods** — the error delta scales with dt (`errOld = errNew - ω*dtSec`). With `kp=0, kd=0.018, ω=-100 deg/s`: `(errNew=90, errOld=90.6, dtSec=0.006)` vs `(errNew=90, errOld=92.9, dtSec=0.029)` | Unit | **Identical D contribution (`kd*ω = -1.8`), asserted within a tolerance (1e-4), NEVER with `==`** — proves the gain is loop-rate independent. A raw per-iteration port yields `-0.0108` vs `-0.0522` (4.8×) and fails. ⚠️ **CORRECTED 2026-07-16** — the original row said "same err/errOld pair → identical D", which is INVERTED: holding the *numerator* fixed, only the per-iteration form (the bug) returns identical values; the correct normalized form returns values differing by exactly the dt ratio. Implementing the old wording literally would fail correct code and pressure a "fix" back into the bug. The real values are `-1.799995` vs `-1.800001` (~3 ULP), so exact equality also fails. |
| TC3b' | **`turnPdStep` fixed err/errOld numerator at dtSec=0.006 vs dtSec=0.029** | Unit | D contributions **differ by exactly the dt ratio (4.8333×)** — asserts the division positively, the complement to TC3b's invariance argument. |
| TC3c | `turnPdStep` with `dtSec = 0.0` | Unit | D term contributes 0; no divide-by-zero, no NaN/Inf reaching the actuator |
| TC4 | `turnSlewLimit(50, 0, 10)` | Unit | Returns 10 — a 0→50 step is clipped to the slew rate |
| TC5 | `turnSlewLimit(12, 10, 10)` | Unit | Returns 12 — within the rate, passes through |
| TC6 | `turnCreepFloor(5, 20, 15)` / `(-5, 20, 15)` / `(19, 20, 15)` / `(25, 20, 15)` / `(0, 20, 15)` | Unit | 15 / −15 / 15 / 25 / **0** (exact zero stays zero) |
| TC7 | `turnSettleStep` in-band 3× then out-of-band | Unit | 1, 2, 3, then **0** — a single out-of-band sample resets |
| TC8 | `turnSettled(2, 3)` / `turnSettled(3, 3)` | Unit | false / true |
| TC9 | Full turn: park `mockYawDeg` at target, tick | Integration | Exits only after `TURN_SETTLE_SAMPLES` ticks, not the first; `MockDriveDC.lastBrake == true` |
| TC10 | Turn from 90° error, first tick | Integration | `abs(lastMoveSyncL)` ≤ `TURN_SLEW_PWM_PER_TICK` — proves the ramp-up limit reaches the actuator (not just the policy) |
| TC11 | Turn with yaw scripted to a 0.3° residual (inside the ±0.5° band) | Integration | Converges and brakes; never emits `PWM_MIN_MAGNITUDE`. Pick 0.3 deliberately — 0.5 sits exactly ON the boundary where `|err| < TURN_TOLERANCE_DEG` is false, which would test the opposite of the intent. |
| TC11b | Turn with yaw scripted to exactly `TURN_TOLERANCE_DEG` (0.5°) | Integration | Treated as OUT of band (strict `<`): the debounce counter stays 0 and the turn keeps correcting. Pins the boundary so a later `<=`/`<` slip is caught. |
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
| T1 | Bench-measure the Lofaix creep floor, tick period + min resolvable step | — | low | `WRO2026/Tasks.cpp` (throwaway, reverted by T10) | Temporary `testOne` harness measuring four things: (a) sweep `Move(p,-p)` for p in 10..30 from rest AND while already rotating — record the lowest p that reliably breaks away and the lowest that sustains rotation (these differ; the second is `TURN_CREEP_PWM`); (b) time a 90° rotation at that p to get ω; (c) **re-confirm the tick period** via `inoTest/tick_bench` (ALREADY MEASURED 2026-07-16: min=7995 avg=8056 max=11484 us, ~124 Hz — re-run only to confirm it holds on the flashed turn build, since it sets `TURN_MIN_RESOLVABLE_DEG = ω × tick`); (d) read yaw 100× stationary for the IMU noise spread. Outputs feed T3's `TURN_CREEP_PWM`, `TURN_CREEP_THRESH_PWM`, `TURN_MIN_RESOLVABLE_DEG`. Report numbers; do NOT commit the harness. Satisfies B-T1. |
| T2 | Create `TurnPDPolicy.h` | — | med | `WRO2026/src/drive/TurnPDPolicy.h` | Header-only, `#ifndef TURN_PD_POLICY_H`. Implement exactly the six functions in Contracts & Interfaces with Doxygen on each. No Arduino include, no `Config.h` include — every tunable arrives as a parameter (this is what makes it host-unit-testable). Document the operator order in the file docblock. Satisfies AC1 groundwork. |
| T3 | Config: add PD knobs, remove two-stage knobs, add asserts | T1, T2 | med | `WRO2026/config/ConfigDrive.h`, `WRO2026/config/ConfigChecks.h` | Add the constants in Data Model with Doxygen + `TODO bench-tune (T1)` markers, citing the mentor's file as the source of the TT-verified values. REMOVE `TURN_SLOW_ZONE_DEG` + `TURN_SLOW_SPEED_PWM`. Leave `TURN_TOLERANCE_DEG` at its existing `0.5f` (operator decision — do NOT retune it; the float path of AC2 is what makes it reachable). Fix the false claim at `ConfigDrive.h:497` that `TURN_SLOW_SPEED_PWM` equals `PWM_MIN_MAGNITUDE`. Add the five `static_assert`s to `ConfigChecks.h` (it currently has ZERO turn asserts). Keep each `#define X_DEFAULT` free of trailing `//` comments (breaks the Python config parser). Satisfies AC7, AC8. |
| T4 | Rewrite `_updateTurnProfile` as a per-tick PD step | T2, T3 | high | `WRO2026/src/drive/Drive.cpp`, `WRO2026/src/drive/Drive.h` | Replace the CRUISE/SLOW/BRAKE enum machinery with: read yaw via `YawSource.readYawDeg` into a **float** (no `int16_t` cast — AC2); `err = turnWrap180(_turnTargetYaw - yaw)`; debounce via `turnSettleStep`/`turnSettled` → on settle `Move(0,0)` + `brake(_turnProfileBrakeOnDone)` + `_turnDone=_moveDone=true`; timeout check → same but `Serial.print` the residual and DO NOT fault (AC6); else `turnPdStep` → clamp `±_turnCruisePwm` → `turnSlewLimit(_turnPrevSpeed)` → `turnCreepFloor` → `turnWheelPattern(_turnDeltaSign, pwm, _turnMotorMode)` → `Move(L,R)` **without** `deadbandSnap` (AC3). Widen `_turnTargetYaw` to float; add `_turnErrOld`, `_turnPrevSpeed`, `_turnInBandCount`, `_turnStartMs`; seed all of them in BOTH `_execTurnDeg` and `_execTurnToYaw`. PRESERVE: `_heldYaw = target` at dispatch (AC10), the tolerance early-out short-circuit (AC9/TC16), the `power > 0 ? power : TURN_CRUISE_PWM` sentinel wallStop relies on, the `turnImuReadStalled` escalation (AC11), and the `_turnProfileState != IDLE` signal `_updateWallStop` polls at Drive.cpp:1413 (keep an IDLE/ACTIVE state even though CRUISE/SLOW are gone). Do NOT add builder setters (AC12). Satisfies AC1–AC6, AC9–AC12. |
| T5 | Unit tests for `TurnPDPolicy.h` | T2 | low | `WRO2026/test/test_turn_pd_policy.cpp`, `WRO2026/test/CMakeLists.txt` | New ctest target linking only the header + `MOCK_SOURCES` (no Drive, no MatrixMiniR4) — mirror the `test_turn_sign_policy` target recipe. Implement TC1–TC8. Note TC6's exact-zero case and TC7's reset-on-miss are the two that catch real bugs. |
| T6 | Stateful PD integration test | T4, T5 | med | `WRO2026/test/test_turn_pd_integration.cpp`, `WRO2026/test/CMakeLists.txt` | New integration target linking the REAL `Drive.cpp` per the issue-#106 recipe (see Testing Strategy). Implement TC9–TC17. This is the FIRST stateful test of the turn FSM — today's coverage is pure predicates + end-to-end yaw parking, with nothing in between. TC10 must assert `MockDriveDC.lastMoveSyncL/R` (the dispatched value), not the policy return — a LineFollower-regime mock records the PRE-snap value and false-greens. Serialize the `CMakeLists.txt` edit behind T5. |
| T7 | Update the pre-turn settle idiom for the N-sample debounce | T4 | med | `WRO2026/test/test_path_tracker_harness.h` | `expectPreTurn` parks `mockYawDeg` and ticks until `Drive.isIdle()`. With the debounce the turn now needs ≥`TURN_SETTLE_SAMPLES` ticks in-band before it idles — verify `kMaxHandshakeTicks` still covers it and bump if not. Update the CONTRACT docblock (it describes the retired "custom TurnGyro profile" two-stage machine). Also re-check `test_path_tracker_settle.cpp`, which parks yaw inline every tick. Do NOT model the FSM in the mock — script the yaw sequence. |
| T8 | Replace the CTPS turn duration model | T3 | med | `tools/path_planner/ctps/profile.py`, `tools/path_planner/tests/test_ctps_profile.py` | `TurnProfile` (profile.py:1141-1193) hand-mirrors the dead two-stage FSM and parses `TURN_CRUISE_PWM`/`TURN_SLOW_SPEED_PWM`/`TURN_SLOW_ZONE_DEG` (:238-240). Two of those no longer exist. Replace with a **display-only** constant-rate + settle-allowance model derived from `TURN_CRUISE_PWM` and a nominal creep tail; a PD loop's true duration is plant-dependent and NOT closed-form, so document it as render-layer-only (same posture ADR-061 took for `HOP_SETTLE_MS`). Fix the stale docstring at :1170-1172 claiming `PWM_MIN_MAGNITUDE = 50` (it is 35). Update `test_ctps_profile.py:644-649`, whose two pinned regimes (default constant-rate, perturbed two-stage) no longer exist. Satisfies AC7 (Python half), AC13, TC19. |
| T9 | ADR-068 + index + architecture row | T4, T11 | low | `docs/decisions/068-turn-pd-controller.md`, `DECISIONS.md`, `ARCHITECTURE.md` | Write ADR-068 in the house shape (~30-60 lines: Rule / Why / How to apply / Trigger to revisit / See also). Rule: the turn executor is a per-tick PD with a creep floor and slew-limited ramp-up; no integrator; tolerance is bench-floored. Why: the two-stage bang-bang's band was unreachable (int16 truncation) and its slow stage ran at the static breakaway floor; the mentor's PD is competition-verified. Mark the 2026-05-17 two-stage-profile decision superseded. Trigger to revisit: the new external gyro landing, or wanting sub-degree (which needs a faster-than-tick loop and its own ADR). Record the B3.2 residual spread from T11. Update `DECISIONS.md` (index line + status table) and `ARCHITECTURE.md`'s "Where PID lives" row — the Turn row currently says "STM32 firmware (`TurnGyro`). Gains in `Config::TURN_PID_*`", which is doubly wrong. Use **068** — 066 AND 067 are both taken by the unmerged `fix/issues-141-159-hardening` branch. |
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
