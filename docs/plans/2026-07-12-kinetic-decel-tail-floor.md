# Kinetic Decel-Tail Floor Plan

**Goal:** Stop the robot shaking at the end of a T-shape move by letting the ramp-down tail creep at a lower *kinetic* PWM floor (`PWM_MOVING_MIN`, ~30) instead of the *static* breakaway floor (`PWM_MIN_MAGNITUDE`, 35) that the deadband snap currently pins the whole move to.

**Approach:** Add a second, lower deadband floor used ONLY on the decel-to-brake tail. Thread an optional `floorMag` parameter (default = the static floor) through every emit chokepoint that a decel tail flows through — `deadbandSnap`, `Drive::_emitContinuous`, and `Drive::setSpeed` — and set the kinetic floor from `_updateProfile`'s decel region and from `LineFollower`'s stop ramp-down (both its lerp target and its emit). Start/breakaway, cruise, mid-follow PID, `wallStop`, and every other caller keep the static floor via the default. Mirror the decel-tail floor in the CTPS render (`ctps/profile.py`) and the pure-Python sim so neither drifts from firmware.

> **Two intertwined changes.** This plan is the `PWM_MOVING_MIN` *feature*. It sits on top of a **static-floor change (`PWM_MIN_MAGNITUDE` 50→35)** that is already in the user's working tree as WIP. That 50→35 has its own broad test fallout (golden regeneration, turn-render tests, config-parse pins) that is logically a **separate prerequisite work-stream** — see the "Prerequisite: 35 static floor" section. Who lands that fallout (the user's WIP vs this plan) is the one open decision.

## Scope

- **In scope (the `PWM_MOVING_MIN` feature):**
  - New `Config::PWM_MOVING_MIN` in `ConfigGlobal.h` (seed 30) with a `#ifndef PWM_MOVING_MIN_DEFAULT` compile-time override hook.
  - Range `static_assert` in `ConfigChecks.h` (allows bench-retuning; forbids inversion).
  - `deadbandSnap(pwm, floorMag = Config::PWM_MIN_MAGNITUDE)` — additive defaulted floor in `WROMath.h`.
  - `Drive::_emitContinuous(..., floorMag = …)` and `Drive::setSpeed(L, R, floorMag = …)` — additive defaulted floor. `_updateProfile` passes `PWM_MOVING_MIN` ONLY on the decel ramp toward a below-static-floor endSpeed.
  - `LineFollower` distance-gated stop ramp-down: lower its lerp **target** to `±PWM_MOVING_MIN` and emit through `Drive::setSpeed(..., PWM_MOVING_MIN)` during the ramp only; mid-follow PID keeps the static floor.
  - CTPS render parity (`ctps/profile.py`) and pure-Python sim parity (`sim/runtime.py`): decel-side floor tracks `PWM_MOVING_MIN`; accel + cruise keep the static floor.
  - Host C++ + pytest coverage for the feature, incl. the phase-gating **negative** case and the triangle-short-hop edge.

- **Out of scope:**
  - `wallStop` push / back-off floor, turn-profile deadband, and all fixed-power / mid-follow emits — stay on the static floor (via the default arg).
  - The final numeric value of `PWM_MOVING_MIN` — bench-tuned in Phase 3 (30 is a placeholder).
  - A new ADR (user decision) — rationale in commit + Config `@warning`, cross-referencing ADR-014/017/018/058.
  - Committing/reverting the user's working-tree WIP.

## Prerequisite: 35 static floor (separate work-stream — OWNERSHIP IS THE OPEN DECISION)

`PWM_MIN_MAGNITUDE = 35` is already in the user's working tree (HEAD = 50). Landing 50→35 has fallout that is **independent of `PWM_MOVING_MIN`** and is red against the working tree *right now*:

- **Golden regeneration** — `tshape_profile_dump.cpp` consumes live `Config::TSHAPE_START_SPEED` (= `PWM_MIN_MAGNITUDE`), so 50→35 changes the golden inputs (start-at-floor rows: `|85−50|/5=7` → `|85−35|/5=10`) and `tshape_profile_freshness` goes red until `tshape_profile_goldens.json` is regenerated.
- **Turn-render tests** — `test_ctps_profile.py::test_tc6_default_config_turn_renders_constant_rate` (`:669`) and `test_tc6_perturbed_cruise_override_yields_two_distinct_rates` (`:736`) assert `slow_pwm == 50`; they held only because `TURN_CRUISE_PWM = 50` equalled the floor. At floor 35 the invariant (slow snaps to floor) breaks and must be redesigned.
- **Config-parse pins** — `test_config_parse.py:250` and `test_config_parse_hop_settle.py:63` assert `PWM_MIN_MAGNITUDE == 50`.
- **Other WIP in the tree** — `ConfigDrive.h` also carries uncommitted changes (`ActiveMotor = LegoEV3`, `MOVESYNC_KP 0.02→0.05`, `TSHAPE_ACCEL_PWM_PER_CM 5→4`, `TSHAPE_RAMP_MIN_CM 2→5`, `TSHAPE_NO_RAMP_EPSILON_PWM 2→5`, …). Editing `ConfigDrive.h` at all risks staging that WIP.

**Decision RESOLVED — (A): the 50→35 and its fallout are the user's WIP.** This plan assumes 35 as baseline and delivers ONLY the `PWM_MOVING_MIN` feature (T1–T11). The prerequisite fallout below is **out of scope** — the user lands it:
- Golden regeneration (`tshape_profile_goldens.json` + dump) for floor 35.
- Turn-render test redesign (`test_ctps_profile.py:669`, `:736`).
- Config-parse pins `==50`→`==35` (`test_config_parse.py:250`, `test_config_parse_hop_settle.py:63`).
- The `TSHAPE_END_SPEED` doc-comment fix in `ConfigDrive.h` (belongs with the 35 change, and editing that file stages user WIP).

The executor must NOT touch these files as part of the feature. Where a feature task shares a file with the user's WIP (e.g. `test_ctps_profile.py` has both feature cases T10 and the prereq turn tests), edit only the feature cases and coordinate.

## Decisions

- **No new ADR** (user) — rationale in commit + a `@warning` on `PWM_MOVING_MIN`, cross-ref ADR-014/017/018/058.
- **Phase-gated floor** (confirmed) — kinetic floor ONLY on the decel ramp toward a below-static-floor endSpeed (Drive profile decel region) and the LineFollower stop ramp-down. Static floor everywhere else. The **predicate is normative**, not the word "braked": a `.setBrake(false)` coast-stop whose endSpeed magnitude is below the static floor also gets the kinetic tail (intended).
- **Three emit chokepoints, one seam each** — `deadbandSnap`, `_emitContinuous`, and `setSpeed` all gain a defaulted `floorMag`. A decel tail can reach hardware through any of them, so all three must carry the floor or the tail gets re-snapped to 35.
- **Render + sim updated in the same PR** — ADR-058 faithfulness (CTPS) and sim-mirrors-robot fidelity. Golden vectors are pre-*snap* but NOT pre-*Config* — the `PWM_MOVING_MIN` feature does NOT change them (it is a downstream snap), but the 50→35 prerequisite does (see above).
- **Do NOT commit the user's working tree** — scope every `git add` to this feature's files; never `git add -A`.
- **Seed `PWM_MOVING_MIN = 30`, TODO bench-tune** — same posture as `HOP_SETTLE_MS`. Bench-tune AFTER the `MOVESYNC_KP` retune lands and on the LegoEV3 chassis (the emitted mag is a MoveSync setpoint into the STM32 PID, not raw PWM — the open-loop "40/30" story is a model).
- **Off-switch** — `PWM_MOVING_MIN == PWM_MIN_MAGNITUDE` → identity; `#ifndef PWM_MOVING_MIN_DEFAULT` lets a host test pin/disable it. Identity test defines `PWM_MOVING_MIN_DEFAULT=PWM_MIN_MAGNITUDE` (not a hardcoded 35) so it survives a floor retune.

## Acceptance Criteria

- [ ] AC1 — `Config::PWM_MOVING_MIN` exists (30) with the override hook; `ConfigChecks.h` range assert compiles.
- [ ] AC2 — `deadbandSnap(pwm)` byte-identical to before; `deadbandSnap(20, 30)==30`, `deadbandSnap(30,30)==30`, `deadbandSnap(90,30)==90`, `deadbandSnap(0,30)==0`, sign-mirrored.
- [ ] AC3 — a Drive `.tShape()` stop emits `±PWM_MOVING_MIN` on the sub-static-floor decel tail (host-observable via `MoveSync` mock); cruise emits `±cruise`.
- [ ] AC4 — **through real `Drive::setSpeed`**, a LineFollower distance-gated stop's ramp-down tail reaches the DriveDC mock at `±PWM_MOVING_MIN`; a below-static-floor mid-follow correction reaches it at `±PWM_MIN_MAGNITUDE`.
- [ ] AC5 — with `PWM_MOVING_MIN_DEFAULT=PWM_MIN_MAGNITUDE`, the tested profiles' emitted PWM is byte-identical to the pre-feature build.
- [ ] AC6 — the phase-gating negative case: a `.setStartSpeed(0)` rest-start emits `±PWM_MIN_MAGNITUDE` (not `±30`) on its sub-static-floor accel setpoints.
- [ ] AC7 — CTPS render + pure-Python sim: decel-tail floor tracks `PWM_MOVING_MIN`; accel + cruise track `PWM_MIN_MAGNITUDE`.
- [ ] AC8 — `wallStop` push/back-off PWM unchanged (still `PWM_MIN_MAGNITUDE`).
- [ ] AC9 — full CI green: `ctest`, `uv run --extra dev pytest tools/path_planner/tests/`, `arduino-cli compile`. (Assumes the 35-floor prerequisite fallout is resolved per the ownership decision.)

## Contracts & Interfaces

### Config constant (owner: T1)

```cpp
// ConfigGlobal.h, beside PWM_MIN_MAGNITUDE
#ifndef PWM_MOVING_MIN_DEFAULT
#define PWM_MOVING_MIN_DEFAULT 30    // TODO bench-tune (Phase 3): kinetic decel-tail floor
#endif
/** @brief Kinetic (moving) deadband floor for the tShape/line-stop decel tail.
 *  @warning Load-bearing: consumed by Drive::_updateProfile decel, Drive::setSpeed,
 *           LineFollower stop ramp-down, ctps/profile.py, sim/runtime.py. Must stay
 *           <= PWM_MIN_MAGNITUDE. == PWM_MIN_MAGNITUDE disables the feature (identity). */
constexpr int16_t PWM_MOVING_MIN = PWM_MOVING_MIN_DEFAULT;
```

### Range assert (owner: T2)

```cpp
// ConfigChecks.h — use && (NOT a chained 0 < x <= y, which is always-true in C++)
static_assert(Config::PWM_MOVING_MIN > 0 && Config::PWM_MOVING_MIN <= Config::PWM_MIN_MAGNITUDE,
              "PWM_MOVING_MIN must be in (0, PWM_MIN_MAGNITUDE]");
```

### deadbandSnap (owner: T3)

```cpp
inline int16_t deadbandSnap(int16_t pwm, int16_t floorMag = Config::PWM_MIN_MAGNITUDE);
```
`0→0`; `0<|pwm|<floorMag → ±floorMag`; `|pwm|>PWM_MAX → ±PWM_MAX`; else pass-through. Cap stays `PWM_MAX`. Config is already visible (`WROMath.h` includes `Config.h`). Consumers: T4, T5.

### Drive emit seams (owner: T4)

```cpp
void   _emitContinuous(DriveMode mode, int16_t mag, int16_t heading,
                       int16_t floorMag = Config::PWM_MIN_MAGNITUDE);   // Drive.h/.cpp
uint8_t setSpeed(int16_t L, int16_t R, int16_t floorMag = Config::PWM_MIN_MAGNITUDE); // or overload
```
- `_emitContinuous` has **four** call sites: `_updateProfile` (decel passes kinetic), `_updateStopAtLine`, and the wallStop **push** + **back-off** (all keep the default). Thread `floorMag` into every `deadbandSnap` inside `_emitContinuous`. Do NOT thread the kinetic floor into the wall-stop calls.
- `setSpeed` applies `_applyBias` then `deadbandSnap(L, floorMag)` / `deadbandSnap(R, floorMag)`; the default keeps every current caller static.
- **Decel-to-brake predicate** (normative): in the decel region AND heading below the static floor — distance: `traveled >= (_profileTotalEnc - _profileDecelEnc) && abs(_profileEndSpd) < Config::PWM_MIN_MAGNITUDE`; time: the analogue at `Drive.cpp:807-808`. All three operands are stored instance state (`_profileTotalEnc` `:719`, `_profileDecelEnc` `:1012-1014`, `_profileEndSpd` `:882`). The half-of-total cap guarantees `decel < total`, so this exactly matches the kernel's decel branch (`TShapeProfilePolicy.h:229`).

### LineFollower stop floor (owner: T5)

Two coupled changes, both required (a lowered target alone is re-snapped to 35 by `setSpeed`; a floor arg alone still lerps to 35):
1. Stop ramp lerp **target**: `LineFollower.cpp:278-280` `targetFloor = ±Config::PWM_MIN_MAGNITUDE` → `±Config::PWM_MOVING_MIN` (sign-corrected as today), and the ramp-length geometry at `:289` that derives from it.
2. **Emit floor selection** at the shared emit (`LineFollower.cpp:500-501`, also `:358-359`): pass `PWM_MOVING_MIN` to `Drive::setSpeed` ONLY when in the stop ramp-down — normative predicate:
   `floorMag = (_useTShapeRampDown && _distanceTargetActive && remainingCm <= rampDownCm) ? Config::PWM_MOVING_MIN : Config::PWM_MIN_MAGNITUDE`
   (conform the exact member names to `LineFollower`'s current fields). Mid-follow PID keeps the static floor.

### CTPS render + sim floor (owners: T8 CTPS, T9 sim)

CTPS (`profile.py`): add `pwm_moving_min` to `ProfileConfig` + `load_profile_config`; decel side (`velocity_at_s` for `s >= dist_cm - decel_cm`; the decel segment + floor-crawl tail in `_phase_segments`; `_decel_floor_crossing`; `_window_fit_stop`) floors at `pwm_moving_min`; accel + cruise floor at `pwm_min`. `pwm_at_s` (pre-snap) untouched. Fix docstrings `:30-34` and `:147` ("50"→note both floors).
Sim (`sim/runtime.py:317` cold-start seed + the LineFollower ramp-down model): decel tail models `PWM_MOVING_MIN`.

### File ownership

| File | Owner | Notes |
|------|-------|-------|
| `WRO2026/config/ConfigGlobal.h` | T1 | add constant only (do NOT touch the 35 value / user WIP) |
| `WRO2026/config/ConfigChecks.h` | T2 | range assert |
| `WRO2026/src/util/WROMath.h` | T3 | shared seam |
| `WRO2026/src/drive/Drive.cpp` + `Drive.h` | T4 | `_emitContinuous` + `setSpeed` floor |
| `WRO2026/src/drive/LineFollower.cpp` (+ `.h`) | T5 | lerp target + emit floor |
| new `WRO2026/test/drive_decel_floor/` + `WRO2026/test/CMakeLists.txt` | T6 | Drive tail + identity + rest-start + short-hop |
| new/updated LineFollower host target linking **real** `Drive.cpp` | T7 | real `setSpeed` floor path |
| `tools/path_planner/ctps/profile.py` | T8 | CTPS render |
| `tools/path_planner/sim/runtime.py` | T9 | sim render |
| `tools/path_planner/tests/test_ctps_profile.py` (feature cases only) | T10 | decel-floor pins |
| `tools/path_planner/tests/test_runtime_sim.py` (feature cases only) | T11 | sim decel-floor pins |

No two parallel feature tasks share a file. **Caution:** the user's WIP also edits `test_ctps_profile.py` (prereq turn tests `:669`/`:736`) and possibly `ConfigDrive.h` — T10 must edit only its feature cases and coordinate to avoid clobbering that WIP.

## Error Handling

- `PWM_MOVING_MIN > PWM_MIN_MAGNITUDE`: `ConfigChecks.h` static_assert fails the build.
- `PWM_MOVING_MIN` missing when the render parses it: `config_parse` raises `ValueError` (fail-loud) at CTPS build.
- No new FSM state / timeout — the floor is a per-tick emit clamp inside existing paths.

## Testing Strategy

| ID | Test Case | Type | Expected |
|----|-----------|------|----------|
| TC1 | `deadbandSnap(20,30)` vs `deadbandSnap(20)` | Unit host | `±30` vs `±35` — default preserves callers (AC2) |
| TC2 | `deadbandSnap` {28→30, 30→30, 90→90, 0→0, −20→−30} | Unit host | floor+cap+zero+sign (AC2) |
| TC3 | Drive `.tShape()` stop, tick to decel tail, read `MoveSync` | Integration host (real Drive.cpp) | sub-floor tail `±30`; cruise `±cruise` (AC3) |
| TC4 | Same, built `-DPWM_MOVING_MIN_DEFAULT=PWM_MIN_MAGNITUDE` | Integration host | tail `±35` — identity (AC5) |
| TC5 | Drive `.tShape().setStartSpeed(0)` rest-start, read accel setpoints | Integration host | first sub-35 accel emits `±35`, NOT `±30` — phase-gating negative (AC6) |
| TC6 | Triangle short hop (peak below static floor, e.g. start35/end0/cruise60→peak31) | Integration host | document the plateau(35)→decel(31) step; assert intended behavior |
| TC7 | LineFollower distance-gated stop via **real** `Drive::setSpeed` | Integration host | ramp-down tail reaches DriveDC mock at `±30`; mid-follow below-floor correction at `±35` (AC4) |
| TC8 | `wallStop` push/back-off emit | Integration host | still `±PWM_MIN_MAGNITUDE` (AC8) — confirm `test_wall_stop_watchdog` pins the magnitude, else add it |
| TC9 | `resolve_drive_profile` end=0 stop: `pwm_at_s(floor_crossing_cm) ≈ pwm_moving_min`, tail>0 | pytest CTPS | decel crosses kinetic floor (AC7) — updates `test_ctps_profile.py:429` |
| TC10 | rear-window-fit stop: `abs(end_pwm) == pwm_moving_min` | pytest CTPS | window-fit ends at kinetic floor (AC7) — updates `:838` |
| TC11 | accel + cruise velocity floor | pytest CTPS | still `pwm_min` — render phase-gating (AC7) |
| TC12 | sim LineFollower stop tail | pytest sim | models `PWM_MOVING_MIN` (AC7) |
| TC13 | `PWM_MOVING_MIN == 30` parse | pytest | constant parses (AC1) |

**Run:** `cmake --build build/test && ctest --test-dir build/test --output-on-failure` · `uv run --extra dev pytest tools/path_planner/tests/` · `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Add `PWM_MOVING_MIN` | — | low | `config/ConfigGlobal.h` | Constant + override hook + `@warning` (Contracts). Do NOT edit the 35 value or `ConfigDrive.h` (user WIP). AC1. |
| T2 | Range assert | T1 | low | `config/ConfigChecks.h` | `&&` form (Contracts). AC1. |
| T3 | `deadbandSnap` floor arg | — | low | `src/util/WROMath.h` | Defaulted `floorMag`; replace floor compares; update Doxygen. AC2. |
| T4 | Drive emit seams | T1, T3 | med | `src/drive/Drive.cpp`, `src/drive/Drive.h` | `floorMag` on `_emitContinuous` (4 call sites) + `setSpeed`; `_updateProfile` decel predicate passes kinetic; wallStop + stopAtLine keep default. AC3, AC5, AC8. |
| T5 | LineFollower stop floor | T1, T3, T4 | med | `src/drive/LineFollower.cpp` (+`.h`) | Lerp target → `PWM_MOVING_MIN` + ramp geometry; emit-floor predicate (Contracts). Depends on T4 for `setSpeed` signature. AC4. |
| T6 | Host test: Drive tail | T4 | low | new `test/drive_decel_floor/*`, `test/CMakeLists.txt` | TC1–TC6 (tail, identity, rest-start negative, triangle short-hop). AC2/3/5/6. |
| T7 | Host test: LineFollower real setSpeed | T5 | med | new/updated host target linking real `Drive.cpp`+`LineFollower.cpp` | TC7 through real `Drive::setSpeed` (NOT mock Drive). AC4. |
| T8 | CTPS render parity | T1 | med | `ctps/profile.py` | `ProfileConfig`+parse; decel-side floor rule; docstrings `:30-34`,`:147`. AC7. |
| T9 | Sim parity | T1 | med | `sim/runtime.py` | decel-tail floor models `PWM_MOVING_MIN`. AC7. |
| T10 | pytest CTPS pins | T8 | low | `tests/test_ctps_profile.py` | TC9–TC11, TC13 (feature cases only; the user's turn-test WIP also touches this file — coordinate). AC1/7. |
| T11 | pytest sim pins | T9 | low | `tests/test_runtime_sim.py` | TC12 (feature case). AC7. |

Feature fan-out: T1, T3 start immediately; T2/T4/T8/T9 next; T5→T7, T8→T10, T9→T11.

**Out of scope (user WIP — do NOT implement here):** golden regen for floor 35; turn-render test redesign (`test_ctps_profile.py:669,736`); config-parse `==50`→`==35`; the `ConfigDrive.h` `TSHAPE_END_SPEED` doc fix. Full CI green (AC9) depends on the user landing these; the feature tasks are green in isolation against a 35 baseline.

## Notes for Implementer

- **Three chokepoints or the tail dies at 35.** A decel setpoint can reach hardware via `_emitContinuous`→`MoveSync`/`Move` OR via `Drive::setSpeed` (LineFollower path). Both call `deadbandSnap`. Miss either and the tail re-snaps to 35. This is why T4 owns both and T5 depends on T4.
- **Assert the DISPATCHED value, not the plan** — read the DriveDC mock (`lastMoveSyncL/R`, `lastSetSpeedL/R`) on the tail; a short braked hop triangle-collapses the planned peak. And T7 must link **real** `Drive.cpp` — the existing `test_line_follower_overshoot` uses a MOCK Drive whose `setSpeed` records the pre-snap value, which would false-green.
- **Phase-gating is the correctness crux; TC5 is its guard** — `TSHAPE_START_SPEED == PWM_MIN_MAGNITUDE`, so a default-start profile can't distinguish a 35 vs 30 accel floor; the `.setStartSpeed(0)` case is the only test that can catch the kinetic floor leaking onto the accel/breakaway path.
- **Golden vectors:** the `PWM_MOVING_MIN` feature does NOT change them (post-snap). The 50→35 prerequisite DOES (the dump reads the floor as `TSHAPE_START_SPEED`). Don't conflate the two.
- **Dirty tree.** Scope every `git add` to feature files; never `git add -A`; never revert 35; don't stage `ConfigDrive.h` WIP.
- **Rollback:** `PWM_MOVING_MIN_DEFAULT = PWM_MIN_MAGNITUDE` (or `PWM_MOVING_MIN = PWM_MIN_MAGNITUDE`) is a zero-behavior-change disable.
