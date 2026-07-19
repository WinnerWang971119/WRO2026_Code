# GYRO-straight R4-side heading-hold PD (issue #169) Plan

**Goal:** Kill the GYRO-mode split-brain (#169): after #172 moved yaw *reads* to the external gyro, `Drive`'s GYRO mode still delegates straight-line heading-hold to the STM32's `MoveGyro*` PID (which servos on the STM32's own IMU), so the first GYRO move steers to a wrong absolute heading immediately. Replace that write path with an R4-side per-tick heading-hold PD over `Move(L,R)` driven by the external gyro, so one coherent heading source closes the loop.

**Approach:** Port the mentor's proven `gyro_move` (`speed = Kp·err + Kd·Δ`, `Move(base±speed)`) into the existing cooperative-tick architecture, mirroring the shipped turn-PD controller (ADR-068). Pure decision math goes in a header-only, host-tested `GyroHoldPolicy.h` (reusing `turnWrap180`/`turnPdStep` from `TurnPDPolicy.h`). A new stateful `Drive::_emitGyroHold`, invoked from **inside** `_emitContinuous`'s `GYRO` choke point (so all three cooperative FSMs are covered at one site), runs one PD step per tick reading `YawSource.readYawDeg` (app-frame). The derivative is computed **on the measurement** (`prevYaw`) via a fold-safe wrapped yaw-delta, so a heading step between chained segments produces no derivative kick — this removes the segment-boundary problem structurally, without any begin/end seeding hooks. Flat/short GYRO moves are lowered to a flat profile at the **dispatcher** (not the builder), so the fire-and-forget `MoveGyro*` execs are deleted while the plan/parity/CTPS/sim contracts are untouched. The whole STM32 write path — `MoveGyro*`, `toHwHeading`, `setMoveGyroPID`, `MOVEGYRO_*`, and the now-orphaned `IMU_YAW_SIGN` — retires in a separate cleanup commit.

## Scope

- **In scope:**
  - `GyroHoldPolicy.h` — header-only, host-tested clamped-PD correction + wheel-mix helpers (reuse `turnWrap180`/`turnPdStep`; reuse `TurnSignPolicy` polarity).
  - `Drive::_emitGyroHold` — per-tick R4 heading-hold PD (fold-safe derivative-on-measurement) reading the external gyro; called from inside `_emitContinuous`'s `GYRO` case, covering `_updateProfile` (GYRO_TSHAPE straights + wall-depart), `_updateStopAtLine` (GYRO seek), `_updateWallStop` (GYRO push).
  - Dispatcher-level lowering: `_dispatchNext`'s `DRIVE_DIST`+GYRO / `DRIVE_TIME`+GYRO cases call `_execProfileMove` (flat profile, explicit `accel`) instead of the vendor execs; delete `_execDriveDistGyro`/`_execDriveTimeGyro` and the no-caller `forwardGyro`.
  - New `GYRO_HOLD_KP`/`GYRO_HOLD_KD`/`GYRO_HOLD_MAX_CORR_PWM`/`GYRO_HOLD_STALE_MS` Config constants (`#ifndef` defaults + `ConfigChecks.h` static_asserts, per-chassis), seeded from the mentor's `gyro_move` as UNVERIFIED placeholders.
  - Full retirement of the STM32 write path: `YawSource::toHwHeading`, `setMoveGyroPID`, `MOVEGYRO_KP/KI/KD`, and the orphaned `IMU_YAW_SIGN` (+ its static_assert) — a separate cleanup commit after the behavior lands.
  - Host tests: new `GyroHoldPolicy` pure-math target + a Drive integration target asserting the real `Move(L,R)` differential; rework the ~10 existing suites that assert `lastMoveGyroTarget`; remove the `imu_yaw_sign_fail` compile-fail target; add a `GYRO_HOLD_*` compile-fail target.
  - Docs: new ADR-075; supersede ADR-004's gyro-straight clause + ADR-045's write half; amend ADR-074 (its #169 retirement trigger is resolved); `DECISIONS.md`, `ARCHITECTURE.md` "Where PID lives" table, `Roadmap` deltas.
- **Out of scope:**
  - The external-gyro *read* migration (#172) and its `GYRO_SCALE`/`GYRO_YAW_SIGN` bench calibration (#172 AC-11/12).
  - The in-place turn PD (#168, shipped) and the torn-read plausibility gate (#170).
  - Any Python planner / sim / CTPS **behavior** change — dispatch-only; the plan + parity trace (`GYRO_TSHAPE`, distances) are unchanged. Comment/string touches only if they reference the retired write path.
  - A dedicated bench-tuning sketch — gains are tuned by editing Config + reflashing the main sketch (hardware-gated follow-up).

## Decisions

- **Single coherent heading source** — all GYRO paths use the R4 hold; no `MoveGyro*` survives — Why: the split-brain is the bug; a residual STM32 loop would reintroduce it on the flat/short path. (Both approach consultants endorse.)
- **Fold-safe derivative-on-measurement seeding** — the hold's state is `{prevYaw, prevMs, hasPrev}`; each tick computes `errNew = turnWrap180(headingApp − yaw)` and a fold-safe synthesized `errOld = errNew + turnWrap180(yaw − prevYaw)`, then calls `turnPdStep(errNew, errOld, dtSec, KP, KD)` verbatim — so the D term is `−KD·turnWrap180(yaw − prevYaw)/dt` (pure yaw-rate, one wrap), independent of the target. A heading step between chained segments causes zero derivative kick; and because the yaw delta is wrapped once (not differenced from two separately-wrapped errors), an error near ±180 cannot straddle the fold into a spurious −360 jerk — Why: the queue dispatches the next command on the *same tick* a profile completes (Drive.cpp:151), so there is often **zero** inter-segment gap; a time-gap reseed heuristic misfires there and injects a clamp-saturated jerk at pass-through nodes (a real-field path the axis-aligned test map hides). Rejected alternatives: time-gap heuristic (unsafe); explicit `seedPending`/begin-end hooks (works, but re-introduces a "forgot to seed" seam this removes for free); differencing two wrapped errors (±180 straddle jerk). Both consultants vetoed the heuristic.
- **Emit from the `_emitContinuous` GYRO choke point** — one call site covers all three FSMs; `_emitContinuous` gains PD state (its "stateless" docblock is updated) — Why: branching at the three FSM callers re-opens a forgettable seam; the choke point closes it. (Verified: all three FSMs funnel their GYRO emit through this one case.)
- **Flat/short GYRO lowered at the dispatcher, not the builder** — `_dispatchNext`'s GYRO `DRIVE_DIST/TIME` cases call `_execProfileMove(GYRO, …, startSpeed=cruise, endSpeed=0, accel=Config::TSHAPE_ACCEL_PWM_PER_CM)`; builder + `DriveCommand::Kind` contract untouched — Why: builder-level always-profiling would change what the plan/parity/CTPS/sim contracts record (they log the PLAN); dispatcher lowering is invisible to them. `accel` MUST be passed explicitly — `DriveCommand::accel` defaults to `0.0f` and `_execProfileMove` rejects `accel <= 0` (rc `0x01`), which `_dispatchNext` escalates to `DISPATCH_FAILED`. **Accepted behavior change:** the lowered move gains a decel ramp (cruise→floor→brake, the codebase's preferred smooth stop) that the vendor `MoveGyroDegs` flat lacked; a `<5 cm` move triangle-collapses (peak ~⅔ cruise); it still brakes at the commanded distance (completion is distance-gated). A new failure mode (the profile's baseline-encoder read can fault `rc 0x02` → clean `DISPATCH_FAILED`) is acceptable. (User: flat/short "use the new GYRO too.") Rejected alternative: a new cooperative bounded-flat FSM (Codex) — heavier; reuse the profile engine.
- **No motor-bias feedforward on the GYRO base** — the base rides symmetric; the heading PD is the authority — Why: faithful to the mentor's open-loop `gyro_move` (no bias) and the turn-PD precedent; one fewer interacting surface. **Revisit trigger (named mechanism):** with no bias and no integrator, a constant motor imbalance produces a *steady-state* heading error ≈ `disturbance_PWM / KP` held for the entire straight (≈3° at `KP=1.5` for a 5-PWM imbalance), not merely a slow drift. If the bench shows a constant lateral offset on a GYRO straight, the first remedy is to apply the existing `_applyBias` feedforward to the base (the `_emitContinuous` RAW case already does this for open-loop `Move`); an integral term is the second. Codex argued for bias up front — recorded, deferred to the bench.
- **Full write-path retirement incl. `IMU_YAW_SIGN`** — verified its only functional consumer is `toHwHeading`; removing `MoveGyro*` orphans it — Why: leaving it is exactly the dead-config surface to remove; ADR-074 already names #169 as the retirement trigger. Separate cleanup commit for reviewability. (User: full retire now; both consultants endorse.)
- **New `GYRO_HOLD_KP/KD` seeded from mentor `gyro_move`** — `KP=1.5` (direct), `KD≈0.048` (raw 8 × ~6 ms mentor loop), UNVERIFIED until bench; derivative time-normalized by measured `dt` (per ADR-068 — the tick jitters, and the timing docs are internally inconsistent (~8–11.5 ms vs a ~22.66 ms turn-tick figure), so measured `dt` is mandatory).
- **Skip-and-hold on read failure emits `Move(base, base)`, no new fault** — on a `readYawDeg` failure emit the current base symmetrically (straight, zero correction) and leave `prevYaw/prevMs` untouched so the next good read's `dt` spans the missed samples — Why: re-issuing stale per-wheel values could drive the *previous* segment's direction on the first tick of a new (e.g. reverse) segment; a symmetric `Move(base, base)` on the current base is boundary-safe and loses only one tick of steering correction. Every GYRO move is bounded by a non-IMU terminator (encoder distance / line / stall), so a stuck gyro degrades to an uncorrected-but-completing straight; a new fault path is unneeded surface. (User choice; refined per critic W3.)
- **Correction dynamically clamped so wheels never reverse** — `|corr| ≤ min(GYRO_HOLD_MAX_CORR_PWM, |base|)`; final per-wheel outputs capped to legal PWM; **no** `deadbandSnap` after mixing (`deadbandSnap` applies to the base *before* mixing only — per-wheel snapping destroys the differential) — Why: `base` shrinks through the decel tail below any static knob; a static-only clamp would flip the inner wheel exactly when the chassis is slowest. Deviates from the mentor's unclamped `gyro_move`; the knob defaults generous (`60`). (User choice; both consultants refined the dynamic form.)
- **Wheel-mix polarity reuses the turn's chassis handedness** — `L = base − corr, R = base + corr` (= `turnWheelPattern(+1, corr, COUNTER_ROT)` + base), host-asserted for cross-policy consistency — Why: the inverted-motor-mapping footgun (`gotcha-gyro-inverted-motor-mapping`); the mentor's literal `power+corr/power−corr` is *inverted* vs this repo's documented `Move(30,0) curves forward-right` contract. Do NOT mint a new sign; do NOT copy meat.ino's M2/M3 reverse flags. Bench-verify forward AND reverse.
- **Stack on `feat/163-external-gyro-yaw` in a git worktree** (`WRO2026_Code_wt169`, branch `feat/169-gyro-move-r4-hold`).

## Acceptance Criteria

- [ ] AC1: A GYRO straight move holds heading by an R4-side per-tick PD over `Move(L,R)` reading `YawSource.readYawDeg`; no `MoveGyro*` / `toHwHeading` call remains in `Drive.cpp` or `YawSource`.
- [ ] AC2: `GyroHoldPolicy.h` is header-only with every tunable injected; a host test links the SAME symbols production uses and covers the ±180 fold, `dt≤0`, the dynamic clamp at `|base|`, equal derivative response across different measured `dt`, and the wheel mix (incl. negative base and never-reverse).
- [ ] AC3: A Drive host-integration test asserts the *dispatched* `Move(L,R)` differential (via `hal_mocks` `lastMoveL/lastMoveR`) grows toward re-alignment when the mocked yaw drifts off target, reduces the wrapped error, and never reverses a wheel under the default clamp. Includes: an **immediate GYRO→GYRO chain with a changed heading** (no derivative spike), a **mid-segment tick hitch**, and a **decel-tail case** driving the profile until `base` falls below the static knob with a live heading error, asserting no wheel reversal. (Defuses the parity-vs-dispatch trap.)
- [ ] AC4: Short (`< TSHAPE_MIN_MOVE_CM`) and `.flat()` GYRO moves run the same R4 hold (lowered to a cooperative flat profile at the dispatcher, `accel=TSHAPE_ACCEL_PWM_PER_CM`) and still **brake at the commanded distance**; `_execDriveDistGyro`/`_execDriveTimeGyro`/`forwardGyro` no longer exist.
- [ ] AC5: On a `readYawDeg` failure mid-hold, the tick emits `Move(base, base)` and leaves `prevYaw/prevMs` untouched; the move still terminates on its own distance/line/stall bound; no new fault kind is added.
- [ ] AC6: `IMU_YAW_SIGN`, `MOVEGYRO_KP/KI/KD`, `setMoveGyroPID`, `toHwHeading`, and the `imu_yaw_sign_fail` target/file are removed **from production**; the tree has no dangling **production functional reference** to any of them. Test-only mock scaffolding is **decoupled, not deleted**: the `hal_mocks` mockYawDeg bridge (`MatrixMiniR4.h:1339-1340`) carries its own local sign constant so it no longer depends on `IMU_YAW_SIGN_DEFAULT`, and the retained `MoveGyro*` mocks keep compiling. Doc/comment references inside owned files are cleaned by their owning task.
- [ ] AC7: `GYRO_HOLD_KP` (float), `GYRO_HOLD_KD` (float), `GYRO_HOLD_MAX_CORR_PWM` (int16_t), `GYRO_HOLD_STALE_MS` (uint16_t) land in `ConfigDrive.h` with `#ifndef` defaults + per-chassis, guarded by `ConfigChecks.h` static_asserts: `GYRO_HOLD_KP > 0`, `GYRO_HOLD_KD >= 0`, `0 < GYRO_HOLD_MAX_CORR_PWM <= PWM_MAX`, `GYRO_HOLD_STALE_MS > 0`. A `-DGYRO_HOLD_*_DEFAULT` compile-fail target pins an invalid value.
- [ ] AC8: Full `ctest` green, `arduino-cli compile` green for the main sketch, `pytest` green (planner unaffected — routes/parity unchanged). CI's three jobs pass. Every intermediate wave compiles (no red build between waves).
- [ ] AC9 (hardware, follow-up — gated on chassis/gyro in hand, and on #172's `GYRO_SCALE`/`GYRO_YAW_SIGN` bench calibration having run first via `inoTest/gyro_calib` so heading is trustworthy): with bench-tuned gains, a GYRO straight tracks heading within the lateral tolerance; wheel polarity confirmed non-inverted in **both forward and reverse** (inject a known ± heading error, record the first `L/R` differential, show wrapped error decreases) via the vendor direction check. Placeholder gains + the UNVERIFIED note ship host-green (like #172).

## Data Model / State

**New Config constants (`ConfigDrive.h`, per-`ActiveMotor` + umbrella `#ifndef`-guarded defaults):**
```cpp
static constexpr float   GYRO_HOLD_KP           = 1.5f;   // PWM per deg    — mentor gyro_move (UNVERIFIED)
static constexpr float   GYRO_HOLD_KD           = 0.048f; // PWM per deg/s  — raw 8 × ~6 ms (UNVERIFIED)
static constexpr int16_t GYRO_HOLD_MAX_CORR_PWM = 60;     // static ceiling on |corr| (dynamic clamp also caps at |base|)
static constexpr uint16_t GYRO_HOLD_STALE_MS    = 50;     // prevYaw older than this (dropout/first tick) ⇒ D = 0
```

**New `Drive` private hold state (mutually exclusive across the 3 FSMs — one GYRO segment at a time):**
```cpp
double   _gyroHoldPrevYaw = 0.0;    // last good measured app-frame yaw (fold-safe derivative source)
uint32_t _gyroHoldPrevMs  = 0;      // millis of that measurement — dt + staleness guard
bool     _gyroHoldHasPrev = false;  // false until the first good read (first tick / stale ⇒ D=0)
```
(No `prevL/prevR` — read-fail emits `Move(base, base)`, boundary-safe.)

**Removed:** `_execDriveDistGyro`, `_execDriveTimeGyro`, `forwardGyro` (Drive); `toHwHeading` (YawSource); `IMU_YAW_SIGN` (ConfigNav); `MOVEGYRO_KP/KI/KD` + `setMoveGyroPID` call (ConfigDrive/Drive).

## Contracts & Interfaces

### `GyroHoldPolicy.h` (header-only, owner: T1; consumers: T4 `_emitGyroHold`, T7 tests)
```cpp
#include "TurnPDPolicy.h"    // turnWrap180, turnPdStep — reused verbatim
#include "TurnSignPolicy.h"  // turnWheelPattern — polarity source of truth

// Clamped PD. errOld is the FOLD-SAFE synthesized value (errNew + turnWrap180(yaw − prevYaw)),
// so turnPdStep's derivative reduces to −KD·turnWrap180(yaw − prevYaw)/dt (pure yaw-rate).
// corrLimitPwm = min(GYRO_HOLD_MAX_CORR_PWM, |base|), computed by the caller.
inline float gyroHoldCorrection(float errNew, float errOld, float dtSec,
                                float kp, float kd, int16_t corrLimitPwm);

struct GyroHoldWheels { int16_t left; int16_t right; };
// L = base − corr, R = base + corr (= turnWheelPattern(+1, corr, COUNTER_ROT) + base),
// each capped to ±PWM_MAX. NO per-wheel deadbandSnap.
inline GyroHoldWheels gyroHoldWheels(int16_t base, int16_t corr, int16_t pwmMax);
```
- **Polarity contract:** a positive app-frame error (target CCW of yaw) yields `right > left`; a host test cross-asserts `gyroHoldWheels(b, c, …)` against `turnWheelPattern(+1, c, COUNTER_ROT)`. No new sign constant.

### `Drive::_emitGyroHold(int16_t base, int16_t headingApp, int16_t floorMag)` (owner: T4; called from `_emitContinuous` GYRO case)
- Snap the base first: `base = deadbandSnap(base, floorMag)` (the profile's decel-tail floor rides here; `floorMag` is live, not dead).
- Reads `readYawDeg`; **on fail** emit `Move(base, base)` and return (`prevYaw/prevMs` untouched). **On success:** `errNew = turnWrap180(headingApp − yaw)`; `fresh = _gyroHoldHasPrev && (now − _gyroHoldPrevMs) <= GYRO_HOLD_STALE_MS`; `errOld = fresh ? errNew + turnWrap180(yaw − _gyroHoldPrevYaw) : errNew` (⇒ D=0 on first/stale); `dt = (now − _gyroHoldPrevMs)/1000`; `corr = gyroHoldCorrection(errNew, errOld, dt, KP, KD, min(GYRO_HOLD_MAX_CORR_PWM, |base|))`; `wheels = gyroHoldWheels(base, corr, PWM_MAX)`; `Move(wheels.left, wheels.right)`; store `prevYaw=yaw, prevMs=now, hasPrev=true`.

### Heading-frame contract (owner: T4; consumers: all GYRO callers, PathTracker)
- The GYRO `heading` channel is **app-frame CCW-positive end-to-end**, compared directly against `readYawDeg`. `toHwHeading` is gone; no caller applies an app→STM32 conversion. Existing `.setHeading(...)` sites already pass app-frame values (ADR-045 preconditions) — no caller change. `Localization::toImuYaw` (pure world↔app geometry, no sign) is unchanged; only its doc reference to `toHwHeading`/`IMU_YAW_SIGN` is removed.

### Retirement set (owner: T2b/T3/T4; every PRODUCTION functional reference removed; test scaffolding decoupled — see AC6)
`YawSource::toHwHeading`, `Config::IMU_YAW_SIGN` (+ static_assert), `MOVEGYRO_KP/KI/KD`, `MiniR4.DriveDC.setMoveGyroPID` call, `Drive::forwardGyro`, `Drive::_execDriveDistGyro`, `Drive::_execDriveTimeGyro`.

### File ownership
| File | Owner | Consumers |
|------|-------|-----------|
| `WRO2026/src/drive/GyroHoldPolicy.h` | T1 | T4, T7 |
| `WRO2026/config/ConfigDrive.h` (+`ConfigChecks.h` GYRO_HOLD asserts) | T2a | T4 |
| `WRO2026/config/ConfigDrive.h`/`ConfigNav.h`/`ConfigChecks.h`/`ConfigManipulator.h` (removals) | T2b | — |
| `WRO2026/src/drive/Drive.{h,cpp}` | T4 | — |
| `WRO2026/src/nav/YawSource.{h,cpp}` | T3 | — |
| `WRO2026/src/nav/Localization.{h,cpp}` | T5 | — |
| existing host tests + `test_path_tracker_harness.h` + `hal_mocks/MatrixMiniR4.h` bridge + mock `Drive.h` files | T6 | — |
| new `test_gyro_hold_policy.cpp`, `test_gyro_hold_integration.cpp`, `test_gyro_hold_config_fail.cpp` | T7 | — |
| `WRO2026/test/CMakeLists.txt` (+ delete `test_imu_yaw_sign_fail.cpp`) | T8 | — |
| `docs/decisions/072-*.md`, `045-*.md`, `071-*.md`, `004-*.md`, `DECISIONS.md` | T10 | — |
| `ARCHITECTURE.md`, `Roadmap.md` | T11 | — |

> **T2a and T2b both touch `ConfigDrive.h`/`ConfigChecks.h` but never concurrently** — T2a (Wave 1, add-only) fully precedes T2b (cleanup commit, after T3+T4). No parallel write conflict.

## Error Handling

- **Gyro read fails mid-hold:** emit `Move(base, base)` (current base, zero correction); leave `prevYaw/prevMs` untouched (next good read's `dt` spans the gap). Move terminates on its encoder-distance / line-detect / stall bound. No new fault. Boundary-safe: never re-drives a prior segment's per-wheel values.
- **First tick / stale prevYaw** (`!_gyroHoldHasPrev` or `now − prevMs > GYRO_HOLD_STALE_MS`): `errOld = errNew` ⇒ D term 0 for that tick (P-only command).
- **`dt ≤ 0`** (same-millis re-entry): `turnPdStep` contributes 0 D.
- **±180 fold:** the yaw delta is wrapped once (`turnWrap180(yaw − prevYaw)`), so an error near ±180 cannot straddle into a spurious −360 derivative jerk.
- **Correction saturates:** clamped to `±min(GYRO_HOLD_MAX_CORR_PWM, |base|)`; wheels capped to `±PWM_MAX`; never reverse under default.
- **Chained GYRO→GYRO with a changed heading:** derivative-on-measurement ⇒ no spurious D-kick (regression-tested).

## Testing Strategy

**Levels:** Unit (policy pure-math), Integration (real `Drive.cpp` + `hal_mocks`, asserting dispatched `Move(L,R)`), Compile-fail (Config asserts), Sketch compile (arduino-cli), Planner (pytest, unaffected), Hardware bench (AC9, follow-up).

| ID | Test Case | Type | Expected |
|----|-----------|------|----------|
| TC1 | `gyroHoldCorrection` err>0, dt>0 | Unit | `kp·err + kd·(err−errOld)/dt`, clamped to `±corrLimit` |
| TC2 | `dt≤0` | Unit | D term = 0 |
| TC3 | equal wrapped yaw-delta over two different dt | Unit | same rate-normalized D contribution |
| TC4 | corr beyond `corrLimit` (incl. `corrLimit=|base|`) | Unit | saturates; inner wheel ≥ 0 |
| TC5 | `gyroHoldWheels(base,±corr)` incl. negative base | Unit | `L=base−corr,R=base+corr`, capped ±PWM_MAX, never reverses when `corr≤|base|`; cross-asserts `turnWheelPattern(+1,corr,0)` |
| TC6 | ±180 fold on error AND on `yaw−prevYaw` | Unit | shortest signed value; no −360 straddle |
| TC7 | GYRO profile, mocked yaw drifts +X | Integration | dispatched `Move(L,R)` differential grows to re-align; wrapped error decreases; no `MoveGyro*` |
| TC7b | GYRO profile driven through the decel tail with a live heading error | Integration | as `base` falls below the static knob, no wheel reverses (dynamic `|base|` clamp binds) |
| TC8 | Immediate GYRO→GYRO chain, heading steps | Integration | no derivative spike on the first tick of segment 2 |
| TC9 | `readYawDeg` false one tick | Integration | `Move(base, base)` emitted; `prevYaw` untouched; move completes |
| TC10 | Mid-segment tick hitch (large dt) | Integration | bounded correction, no jerk |
| TC11 | Short (`<5 cm`) / `.flat()` GYRO move | Integration | cooperative flat profile (`accel` set); brakes at commanded distance; no `MoveGyroDegs` |
| TC12 | `-DGYRO_HOLD_{KP,KD,MAX_CORR_PWM,STALE_MS}_DEFAULT=<invalid>` | Compile-fail | `static_assert` fires with the documented message |
| TC13 | Reworked PathTracker GYRO-hop suites | Integration | assert the `Move(L,R)` hold, not `lastMoveGyroTarget` |

**Test data:** `hal_mocks` `mmL.mockYawDeg` bridge (#172, decoupled from `IMU_YAW_SIGN_DEFAULT`) scripts the yaw sequence; `lastMoveL`/`lastMoveR` observe dispatch.
**Run:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`; `uv run --extra dev pytest tools/path_planner/tests/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | `GyroHoldPolicy.h` | — | high | `WRO2026/src/drive/GyroHoldPolicy.h` | New header-only policy: `gyroHoldCorrection` (clamped PD, reuse `turnWrap180`/`turnPdStep`; caller passes fold-safe `errOld`) + `gyroHoldWheels` (mix via `turnWheelPattern(+1,…)`, cap ±PWM_MAX, no per-wheel snap). Doxygen per `TurnPDPolicy.h`. Satisfies AC2. |
| T2a | Config: add `GYRO_HOLD_*` | — | med | `WRO2026/config/ConfigDrive.h`, `ConfigChecks.h` | Add `GYRO_HOLD_KP/KD/MAX_CORR_PWM/STALE_MS` (pinned types + `#ifndef` defaults, per-chassis) + the four static_asserts (AC7). Add-only — no removals (behavior commit). Satisfies AC7. |
| T2b | Config cleanup: retire `MOVEGYRO_*` + `IMU_YAW_SIGN` | T3, T4 | low | `WRO2026/config/ConfigDrive.h`, `ConfigNav.h`, `ConfigChecks.h`, `ConfigManipulator.h` | Remove `MOVEGYRO_KP/KI/KD`, `IMU_YAW_SIGN` + its static_assert; fix `ConfigManipulator.h:98`'s "IMU_YAW_SIGN style" comment. Cleanup commit (after consumers are gone). Satisfies AC6. |
| T4 | Drive core rewrite | T1, T2a | xHigh | `WRO2026/src/drive/Drive.{h,cpp}` | Add `_emitGyroHold` (fold-safe derivative-on-measurement, `Move(base,base)` on read fail) + hold-state members; call it from inside `_emitContinuous`'s GYRO case (update its stateless docblock); lower GYRO `DRIVE_DIST/TIME` at `_dispatchNext` to `_execProfileMove` (startSpeed=cruise, endSpeed=0, **accel=Config::TSHAPE_ACCEL_PWM_PER_CM**); delete `_execDriveDistGyro`/`_execDriveTimeGyro`/`forwardGyro`; drop the `setMoveGyroPID` call. No base bias. `deadbandSnap` base pre-mix. Satisfies AC1/AC3/AC4/AC5. |
| T3 | Retire `toHwHeading` | T4 | low | `WRO2026/src/nav/YawSource.{h,cpp}` | Remove `toHwHeading` + Doxygen once no `Drive.cpp` caller remains. Cleanup commit. Satisfies AC6. |
| T5 | Doc: Localization | T3 | low | `WRO2026/src/nav/Localization.h`, `Localization.cpp` | Update all `IMU_YAW_SIGN` doc/comment references in `Localization.h` (incl. lines 115, 166-167, 458-461 and the `toImuYaw` doc) and the `IMU_YAW_SIGN` comment at `Localization.cpp:48` — `toImuYaw` is pure world↔app geometry, no sign. |
| T6 | Rework existing host tests + decouple mocks | T2a, T3, T4 | med | `test_path_tracker_harness.h`, `test_path_tracker_{projection,reset_arrival,reset_arrival_strict,speed_chain,stop_at_line}.cpp`, `test_return_to_node_dispatch.cpp`, `test_yaw_source.cpp`, `test_turn_pd_integration.cpp`, `drive_hardening/`, `drive_decel_floor/`, `test_mock_hal.cpp`, `hal_mocks/MatrixMiniR4.h`, the 3 mock `Drive.h` files | Replace `lastMoveGyroTarget == appHeadingToHw(...)` with the `Move(L,R)` hold (assert new behavior, don't just delete); drop `appHeadingToHw` + `test_yaw_source` TC21; **decouple the `hal_mocks` mockYawDeg bridge** (`MatrixMiniR4.h:1339-1340`) — give `kBridgeImuSign` its own bridge-local literal so it no longer reads `IMU_YAW_SIGN_DEFAULT`; remove the `forwardGyro` references in the 3 mock `Drive.h` files + `ConfigDrive.h:186`. Satisfies AC3(existing)/AC6. |
| T7 | New tests | T1, T4 | med | `WRO2026/test/test_gyro_hold_policy.cpp`, `test_gyro_hold_integration.cpp`, `test_gyro_hold_config_fail.cpp` | Pure-math policy target (TC1–6) + Drive integration asserting dispatched `Move(L,R)` (TC7–11) incl. chained no-spike, decel-tail no-reversal, skip-and-hold, flat-short-brakes-at-distance; a `GYRO_HOLD_*` compile-fail probe (`test_gyro_hold_config_fail.cpp`, TC12). Satisfies AC2/AC3/AC4/AC7. |
| T8 | CMakeLists + delete old compile-fail target | T6, T7 | low | `WRO2026/test/CMakeLists.txt`, delete `WRO2026/test/test_imu_yaw_sign_fail.cpp` | Add `gyro_hold_policy` + `gyro_hold_integration` + the `gyro_hold_config_fail` compile-fail target (probe = `test_gyro_hold_config_fail.cpp`); remove the `imu_yaw_sign_fail` target + file. Orchestrator owns the CMakeLists edit (append-race). Satisfies AC7/AC8. |
| T10 | ADRs + DECISIONS | T4 | low | `docs/decisions/075-gyro-move-r4-hold.md` (new), `045-*.md`, `071-*.md`, `004-*.md`, `DECISIONS.md` | ADR-075 (R4-side GYRO hold + write-path retirement, fold-safe derivative-on-measurement rationale); supersede ADR-004's gyro-straight clause + ADR-045's write half; amend ADR-074 (#169 trigger resolved). Update index + status table. |
| T11 | ARCHITECTURE + Roadmap | T4 | low | `ARCHITECTURE.md`, `Roadmap.md` | Remove `toHwHeading`/`forwardGyro` from the module surface; flip the "Where PID lives" GYRO-straight row to app-side (`Drive::_emitGyroHold`, `GYRO_HOLD_KP/KD`), mirroring the Turn row. |

**Parallelism:** Wave 1 = T1, T2a (independent). Wave 2 = T4 (after T1, T2a). Wave 3 = T3, T7, T10, T11 (after T4); then T5, T6 (after T3). Wave 4 = T8 (after T6, T7), then T2b (cleanup commit, after T3+T4). No two tasks write the same file concurrently. Behavior commit = T1/T2a/T4/T6/T7/T8; cleanup commit = T2b + T3 + T5 + doc surgery (T10/T11).

## Notes for Implementer

- **Worktree:** all work on `feat/169-gyro-move-r4-hold` in `/Users/diego/Documents/WRO2026/WRO2026_Code_wt169` (off `feat/163-external-gyro-yaw`). In a sibling worktree, sub-agents are sandbox-blocked on Write/Bash — **code-writers RETURN edits; the orchestrator applies them + owns all git**. A worktree has no `.venv` (gitignored) — `uv sync` before any pytest.
- **Toolchain:** the macOS box has cmake/ctest/clang + arduino-cli + the renesas core + MatrixMiniR4 — run `ctest` + `arduino-cli compile` yourself (compile-only, no ninja). Only `arduino-cli` catches link/ODR errors; host ctest uses mocks + separate TUs. Verify **every wave compiles** (the T2a/T2b split exists to keep it so).
- **Commit shape:** land behavior first (T1/T2a/T4/T6/T7/T8), then the retirement as its own cleanup commit (T2b + T3 + T5 + doc surgery). Match repo commit style (`git log -20 --oneline`); no AI attribution.
- **Drive builders** are `[[nodiscard]]` move-only RAII — emit as one discarded temporary ending in the action/`.async()`.
- **Parity-vs-dispatch trap:** the sim parity trace + CTPS record the PLAN (`GYRO_TSHAPE`, distances), NOT dispatched PWM. Parity-green does NOT prove the motion — AC3's `Move(L,R)` assertion is the real host gate, plus the AC9 hardware bench.
- **Inverted-motor-mapping footgun** (`gotcha-gyro-inverted-motor-mapping`): the correction→wheel polarity must match the chassis; reuse `turnWheelPattern(+1,…)` (bench-verified on Lofaix `(true,false)`); the mentor's literal `power+corr/power−corr` is inverted vs `Move(30,0) curves forward-right`. Confirm forward AND reverse with the vendor direction check (AC9).
- **Mentor `gyro_move` reference:** `Kp=1.5`, `Kd=8` raw per-loop, `Move(power+speed, power−speed)`, distance-bound, brake at end, NO clamp/floor/integrator (`/Users/diego/Documents/WRO2026/Meat_code/messageImage_17842751*.jpg`; meat.ino's `acc_gyro_move` still uses STM32 `MoveGyro` — NOT the reference). Our port time-normalizes the derivative, computes it fold-safe on the measurement, and adds the never-reverse clamp — all documented deviations. Gains ship UNVERIFIED, bench-tuned via Config edit + reflash (no dedicated sketch).
- **Control law is PD + clamp only** — no slew limiter, no creep floor, no integrator (the turn needed those because it starts from rest and scrubs; a hold rides a moving base and `corr→0` is the goal). Don't port the turn's extra stages. (Steady-state error from a constant imbalance is the named revisit trigger for adding `_applyBias`/an integral — see Decisions.)
- **SYNC + tShape stays the default** — this change only affects the GYRO branch; RAW/SYNC flat bounded moves keep `MoveDegs`/`MoveSyncDegs` (not split-brain).
- **Keep the `hal_mocks` `MoveGyro*` mock methods** (harmless, now unused) to avoid gratuitous mock churn; only the assertions that read them move, and the mockYawDeg *sign bridge* is decoupled from the retired `IMU_YAW_SIGN_DEFAULT` (T6).
- **Timing doc inconsistency** (Codex): `TurnPDPolicy.h` says ~8–11.5 ms, `ConfigDrive.h:687` records ~22.66 ms for a real turn tick. Do not hardcode a tick period — always use measured `dt` (already core to the design).
```
```

## Review provenance

Approach consult: Fable + Codex (both VETOed the first-draft time-gap seeding / builder-level lowering; reconciled into fold-safe derivative-on-measurement + dispatcher lowering). Critic: Fable (3 critical / 7 warning / 3 nit — all accepted; core control-law design verified sound). Codex critic ran async (inline findings unavailable; noted). Judge (Opus): critical-1 accel-pin CRITIC_WINS, critical-2 T2-split CRITIC_WINS, critical-3 AC6 SYNTHESIS — all applied above.
