# Issue #65 — Pivot seek re-anchor: STOP_AT_LINE-incoming pivot trim Plan

**Goal:** A single-wheel pivot reached via a stop-at-line *seek* must leave the wheel axle δ short of the pivot node (like every other approach mechanism), instead of rolling the axle fully onto the node and landing the pivot off-target (LINE_NOT_FOUND on the outgoing follow). Also deliver the audit the issue comment asks for — *every* pivot × stop-mechanism interaction, with a verdict per cell.

**Approach:** The pivot regime matrix (ADR-035, carried by ADR-036) has only two incoming rows — LINE and BLIND/PROJECTION — and **no row for a STOP_AT_LINE *seek* incoming**. A seek re-anchors longitudinally (axle on the node) but, unlike the LINE path, never reduces its post-detect overshoot by δ. Add the missing row: on a seek hop whose approaching pivot is active, reduce the seek's `Drive.stopAtLine` overshoot from `LINE_SENSOR_AXLE_OFFSET_CM` to `offset − δ` (clamped), mirroring the LINE overshoot path. Route both the LINE path and the new seek path through one shared `resolveSensorOvershootTrim` helper (C++ + Python twin) for cross-language parity, extend the validator's δ≤offset budget to the seek-incoming case, and record the new regime row in a new ADR. δ>offset (pivots sharper than ~106° on our track, unrealisable via a forward-only seek) is hard-rejected by the validator; the general fix (a reverse "re-seat") is deferred to issue #75.

### Why the seek fires (ADR-038 precedence ladder) — the user's case is the auto-rule

On `origin/main`, ADR-038 (issue #43) makes the seek fire via a precedence ladder (`StopAtLinePolicy.h::resolveStopAtLine`, fed a `StopAtLineQuery`): (1) call-site `setStopAtLine(ON/OFF)`, (2) per-edge `stop_at_line_override`, (3) legacy `BlindMode::STOP_AT_LINE`, (4) **auto-rule** `BLIND_PROJECTION_AUTO_SEEK && nextHopIsProjection` (PathTracker.cpp:1620, `_nextHopIsProjection()`), (5) never. The issue's `_nextHopIsProjection()` reference is **accurate for main** — the user's scenario (u→v BLIND, v→w PROJECTION) seeks via **rung 4**. The bug is independent of *which* rung fires: every rung reaches the same no-trim seek dispatch (PathTracker.cpp:1665–1669), and the fix keys on `_approachingPivot.active`, so it covers all rungs.

## Scope

- **In scope:**
  - Firmware: a seek hop into an approaching pivot reduces its overshoot by δ (approaching trim only), clamped to the floor.
  - A shared `resolveSensorOvershootTrim` helper in `PivotTrimPolicy.h`; refactor the existing inline LINE overshoot reduction to call it (byte-identical behavior, preserving its direction-resolved offset).
  - Sim (`sim/runtime.py`): land the seek δ-short so parity holds and a sim test catches the bug.
  - Validator: extend the overshoot-budget check (δ ≤ offset) to `"sensor"`-incoming pivots (authored STOP_AT_LINE, per-edge override, AND auto-rule BLIND→PROJECTION); remove the "review fix 3" exemption; fix the muddled regime docstring/comment; error hints the escape.
  - The audit: full pivot × stop-mechanism matrix with a verdict per cell (the issue-comment deliverable), captured in the ADR + this plan.
  - ADR-044 extending the regime matrix with the STOP_AT_LINE-incoming row; DECISIONS.md index update.
  - Host C++ (policy + inline-mirror), validator, and sim-parity tests.
- **Out of scope:**
  - The reverse "re-seat" / signed-overshoot generalization for δ>offset pivots — deferred to **issue #75** (covers both the seek and the LINE follow).
  - REAR/2-board pivots — pivots are FRONT-only in v1 (issue #54).
  - Changing the default `LOST_LINE_ACTION`, the seek arm-window / max-margin contract, the auto-rule, or any pivot geometry (δ formula, wheel resolution).
  - The #64 projection-node rework and #36 localization work (referenced as related only).
  - Physical bench tuning of pivot over-travel magnitude.

## Decisions

- **D1 — Approaching trim only on the overshoot (reject the Opus consult's "subtract both").** The reduction subtracts the *approaching* pivot trim only, never the departing trim. A seek is sensor-terminated: it stops at the line at `v` regardless of where it started, so a departing pivot at `u` is auto-absorbed and the post-detect overshoot depends only on the approaching pivot at `v`. Confirmed by the code trace (Q1), the geometry, and the firmware's own comment (PathTracker.cpp:1625–1627).
- **D2 — One shared helper, parity-enforced.** Add `resolveSensorOvershootTrim(axleOffsetCm, approachingTrimCm, minDriveCm)` to `PivotTrimPolicy.h`, implemented as a thin wrapper over the existing `resolvePivotTrim(axleOffsetCm, 0.0f, approachingTrimCm, minDriveCm)` (departing fixed at 0; verified signature at PivotTrimPolicy.h:58). The LINE path (PathTracker.cpp:826–840) is refactored to call it **passing its existing direction-resolved `overshootAxleOffsetCm`** (FRONT or REAR); the new seek path calls it with the FRONT offset (pivots are FRONT-only); `sim/runtime.py` carries the Python twin. This kills the inline-copy drift the project already eliminated for `resolvePivotTrim` / `resolveStopAtLine` / `resolveDirection`.
- **D3 — Validator budgets the seek-incoming side.** Extend the budget check (`validator.py:687`) from `in_regime == "line"` to `in_regime in ("line", "sensor")`; same budget (`LINE_SENSOR_AXLE_OFFSET_CM`, the FRONT offset, pivots FRONT-only). The `_regime` helper already folds the auto-rule (`in_auto_rule = auto_seek_enabled and next_hop_is_projection`, validator.py:638–639) into the `"sensor"` class, so this covers authored STOP_AT_LINE edges, per-edge overrides, AND auto-rule BLIND→PROJECTION pivots in one change. Update the docstring matrix (validator.py:370–378) and the inline comment (680–686); the δ>offset error names the escape (dead-reckon BLIND approach, center turn, or issue #75). No departing-seek budget is added (D1: departing auto-absorbed; the chained sum-check stays distance-only, validator.py:704–707).
- **D4 — Validator covers authored + auto-rule; runtime clamp backstops only pure call-site ON.** The validator sees the auto-rule (it computes `next_hop_is_projection` offline) and per-edge/legacy STOP_AT_LINE edges, so its δ≤offset guard protects all of those. The one case it cannot see is a pure call-site `setStopAtLine(StopAtLine::ON)` on an edge that is otherwise *not* sensor-classified (no projection continuation, no override). There the firmware fix still lands the axle δ-short (it keys on `_approachingPivot.active`, not the rung); a δ>offset such hop falls to the runtime clamp + "Check validator/config" warning. This matches how the codebase already treats call-site vs edge precedence.
- **D5 — New ADR-044, cross-linking 035 and 036.** Adding the regime row and removing the validator exemption changes an in-force rule, so it gets a new ADR. The regime matrix originates in ADR-035 and is carried by ADR-036, so ADR-044 cross-links both (mark 036 "extended by ADR-044"). 043 is the next free number on `origin/main` (highest is 042; 041 is held by the unmerged blind-decay branch). Re-verify and renumber per the ADR-collision rule if an in-flight branch lands 043 first.
- **D6 — FRONT-forcing is load-bearing; assert it.** A pivot-adjacent hop is forced FRONT in `_onPlanNextHop` (PathTracker.cpp:1271–1273) before `resolveStopAtLine` reads `_resolvedDirection`, so a seek into a pivot is always FRONT (single offset). The validator also rejects authored REAR on a pivot leg. Residual uncovered corner: an operator hand-forcing both `setStopAtLine(ON)` and REAR into a pivot on a 2-board chassis — but the FRONT-force still applies at 1271–1273, so this is defended at runtime. Add a sim assertion so a future REAR-pivot (#54) change can't silently route an untrimmed rear seek.

## The Audit — pivot × stop-mechanism matrix (issue-comment deliverable)

"Find all similar problems." Every incoming mechanism's approach-trim status into a pivot node `v`, plus the adjacent interaction cases. Each cell is code-traced and cited (line numbers on the `origin/main` base, e0451c7).

**Incoming approach trim (does the axle land δ-short of `v`?):**

| Incoming `u→v` | Re-anchors at `v`? | δ-short approach trim? | Verdict |
|---|---|---|---|
| LINE follow | yes (junction line) | yes — overshoot reduced by δ (PathTracker.cpp:826–840) | OK |
| BLIND dead-reckon | no | yes — `driveDist` shortened by δ (`_pivotTrimDistance`) | OK |
| PROJECTION | no (`onDeadReckonedAdvance`) | yes — `driveDist` shortened by δ; encoder-gated, trim bites before the drive | OK (Finding 5) |
| **STOP_AT_LINE seek** (any rung: legacy edge / per-edge override / auto-rule BLIND→PROJECTION) | yes (line) | **NO — axle rolls fully onto `v`** (PathTracker.cpp:1625–1627 comment; dispatch 1665–1669) | **BUG → fixed (D1–D3)** |

**Adjacent interaction cases:**

| Case | Verdict | Why |
|---|---|---|
| Departing-pivot-into-seek (arm-window) | OK | δ_departing auto-absorbed by sensor termination; the line still appears inside the seek's arm window (`STOP_AT_LINE_ARM_WINDOW_CM`, sized generous per ADR-027) and under the max margin (`STOP_AT_LINE_MAX_MARGIN_CM`). Revisit only if the arm window is tightened below the max authored departing δ. (Q1) |
| Chained pivots across a seek (latch carry) | OK | `_pendingPivot → _departingPivot` is latched at plan-time in `_onPlanNextHop`; no STOP_AT_LINE / seek code path clears it. (Q2) |
| Pure call-site `setStopAtLine(ON)` into pivot | Covered + bounded | Reachable; firmware fix lands it δ-short. Validator can't see the call-site-only override → runtime clamp+warning backstop. (Q3, D4) |
| RETURN_TO_NODE (#36) seek onto a pivot node | OK | The return seek precedes pivot resolution and re-acquires the same node; no δ owed. Spec asserts the latch is clean before the return dispatches. (Finding 3) |
| Pivot node that is also a wall-zero reset node (ADR-040) | OK | The WALL_STOP→REANCHOR interlude re-anchors `v` exactly on arrival, erasing any approach error before the pivot fires — the incoming trim is belt-and-braces there. (Finding 4) |
| REAR / 2-board seek into a pivot | Unreachable | FRONT forced on any pivot-adjacent hop (PathTracker.cpp:1271–1273) before the seek dispatches; the validator also rejects authored REAR on a pivot leg. Asserted (D6). (Finding 6) |

## Acceptance Criteria

- [ ] AC1 — A seek hop with `_approachingPivot.active` dispatches `Drive.stopAtLine(...)` with overshoot `= LINE_SENSOR_AXLE_OFFSET_CM − δ` (clamped to `PIVOT_TRIM_MIN_DRIVE_CM`), so the axle settles δ short of the pivot node. Holds for all seek rungs incl. the auto-rule.
- [ ] AC2 — The reduction subtracts the approaching trim only; a departing-pivot-into-seek hop (no approaching pivot) dispatches the seek with the full default overshoot, unchanged.
- [ ] AC3 — The LINE overshoot path and the seek overshoot path both compute the reduction through `resolveSensorOvershootTrim`; the LINE path's observable behavior is byte-identical to before (incl. its REAR offset on REAR LINE-into-pivot hops).
- [ ] AC4 — `sim/runtime.py` lands a seek-into-pivot at `node − δ·h_in`: a sim assertion that landed the axle on the node before the fix lands it δ-short after.
- [ ] AC5 — The validator rejects a `"sensor"`-incoming pivot (authored or auto-rule) with δ > offset (precise `PIVOT_TURNS[i]` error naming the escape) and passes one with δ ≤ offset.
- [ ] AC6 — A pivot-adjacent STOP_AT_LINE hop resolves `Direction::FRONT` before the seek dispatches (asserted in the sim).
- [ ] AC7 — The validator tightening (D3) rejects **zero** currently-committed fields: `build_routes` on the real `field_data` + every committed fixture still passes.
- [ ] AC8 — ADR-044 records the STOP_AT_LINE-incoming regime row + the audit matrix + the δ>offset hard-reject + the #75 deferral; DECISIONS.md index and status table updated; ADR-035 and ADR-036 cross-linked.
- [ ] AC9 — No CI regression: full `uv run --extra dev pytest` green; host ctests green; `arduino-cli` compile clean; routes goldens unchanged (`git diff --exit-code` clean) — true because no emitter/`field_data` change AND AC7 holds.

## Data Model / API

New helper in `WRO2026/PivotTrimPolicy.h` (header-only `inline`, host-linkable):

```cpp
struct SensorOvershootResolution
{
    float overshootCm;   ///< post-detect axle roll: max(offset - approachingTrim, floor).
    bool  clamped;       ///< true if the floor clamp fired (caller traces it).
};

/// Reduce a sensor-anchored hop's post-detect overshoot by the approaching pivot
/// trim so the axle settles delta short of the pivot node. Departing trim is NOT a
/// parameter: a sensor-terminated hop self-corrects on the line (ADR-044 D1).
/// Thin wrapper over resolvePivotTrim(axleOffsetCm, 0.0f, approachingTrimCm, minDriveCm),
/// mapping driveCm -> overshootCm.
inline SensorOvershootResolution resolveSensorOvershootTrim(
    float axleOffsetCm, float approachingTrimCm, float minDriveCm);
```

Python twin: `sim/runtime.py` gains the same function (or an inline mirror) called by both the LINE overshoot reduction and the seek arrival, kept step-for-step identical.

## Error Handling

- **δ > offset, authored/auto-rule sensor-incoming pivot:** validator fail-fast with `FieldDataError("PIVOT_TURNS[i] (...) over-travels δ=… cm, leaving … of the sensor overshoot budget … below the floor … — soften the turn, approach via a dead-reckon BLIND leg, or see issue #75")`. Build stops; no robot motion.
- **δ > offset reaching the firmware anyway (pure call-site `setStopAtLine(ON)`):** `resolveSensorOvershootTrim` clamps the overshoot to `PIVOT_TRIM_MIN_DRIVE_CM` and PathTracker prints the existing overshoot-underflow Serial warning; the axle lands ~on the node (under-compensated) rather than commanding a negative roll. Drive's OVERSHOOT phase treats `overshoot <= 0` as "skip" (Drive.cpp:1300), so it never receives a negative value.
- **Seek finds no line (pre-existing LINE_NOT_FOUND):** unchanged — the seek hard-faults as today (ADR-039, no soft-fallback); the trim does not touch the seek's max-distance / stall watchdog or arm window.

## Testing Strategy

**Levels:** Unit (host C++ policy), inline-MIRROR (host C++ PathTracker decision slice), Sim parity (Python FSM), Unit (Python validator), plus the `arduino-cli` compile gate.

> **Host-link reality:** no host target links `PathTracker.cpp` (`SUT_SOURCES = LineFollower.cpp` only, CMakeLists.txt:53–55; the rationale block at 285–301 states it explicitly). The PathTracker host tests **inline-mirror** the decision slice with `// MIRROR: PathTracker.cpp:<line>` annotations. So the *helper* is host-unit-tested directly, the new dispatch *decision* is an inline-mirror, and the *integration landing* is verified in the sim.

| ID | Test Case | Type | Expected Behavior |
|---|---|---|---|
| TC1 | `resolveSensorOvershootTrim(offset, δ<offset, floor)` | Unit (host C++) | `overshootCm = offset − δ`, `clamped=false` |
| TC2 | `resolveSensorOvershootTrim(offset, δ>offset, floor)` | Unit | `overshootCm = floor`, `clamped=true` |
| TC3 | `resolveSensorOvershootTrim(offset, 0, floor)` | Unit | `overshootCm = offset`, `clamped=false` (no pivot ⇒ unchanged) |
| TC4 | Seek-into-pivot overshoot decision | Inline-MIRROR (host C++) | mirror of the new dispatch line computes overshoot `= offset − δ`; annotated `// MIRROR: PathTracker.cpp:1665` |
| TC5 | LINE-into-pivot overshoot post-refactor | Sim + existing tests | existing LINE-into-pivot overshoot tests pass unchanged (regression); REAR offset preserved |
| TC6 | Pivot-adjacent STOP_AT_LINE hop direction | Sim | resolves `FRONT` before the seek dispatch (AC6) |
| TC7 | Sim: seek-into-pivot axle landing (auto-rule BLIND→PROJECTION) | Sim parity | final axle `= node − δ·h_in` (was `node` pre-fix) — the user's scenario |
| TC8 | Sim: chained pivots across a seek | Sim | latch carries; both pivots land within tolerance |
| TC9 | Sim: pure call-site `setStopAtLine(ON)` into pivot | Sim | same δ-short landing as TC7 |
| TC10 | Validator: `"sensor"`-incoming pivot δ>offset (authored + auto-rule variants) | Unit (Python) | `FieldDataError`, `PIVOT_TURNS[i]` + escape hint |
| TC11 | Validator: `"sensor"`-incoming pivot δ≤offset | Unit | passes (no error) |
| TC12 | Validator: departing-seek shared edge | Unit | a seek `v→w` shared by two pivots is NOT double-budgeted (chained sum-check stays distance-only) |
| TC13 | Firmware ↔ sim parity for the overshoot trim | Parity | C++ helper == Python twin across a value grid (extend the existing pivot-trim parity) |

**Test data:** extend an existing `SimpleNamespace` pivot fixture with sensor-incoming pivots. The δ≤offset (passing) fixture may join `all_fields()`. The δ>offset (rejecting) fixture **must live in a separate factory excluded from `all_fields()`** — `test_pivot_fixtures_validate` sweeps `all_fields()` asserting every fixture passes `validate`, so a reject fixture there breaks that sweep. Per [[gotcha-strict-emitter-field-fixtures]], add a pivot triple + a STOP_AT_LINE/projection edge to an existing fixture without adding a new strict-read attribute; only the full pytest run finds all touch-points.
**Run commands:** `uv run --extra dev pytest tools/path_planner/tests/` ([[gotcha-uv-run-extra-dev-pytest]]); host ctest via the bundled MSVC/cmake; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` ([[arduino_cli_headless_available]]).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | ADR-044 + index | — | low | `docs/decisions/044-stop-at-line-incoming-pivot-trim.md`, `DECISIONS.md` | Write ADR-044: add the STOP_AT_LINE-incoming row to the regime matrix; record D1 (approaching-only), the seek precedence ladder incl. the auto-rule, the δ>offset hard-reject, the validator-covers-authored+auto-rule boundary (D4), and the #75 deferral. Embed the audit matrix (AC8). Mark ADR-036 "extended by ADR-044", cross-link ADR-035 (matrix origin). Add the index line + bump the status table in DECISIONS.md. Verify 043 is free (highest on main is 042); renumber per [[gotcha-adr-number-collision]] if contested. Satisfies AC8. |
| T2 | `resolveSensorOvershootTrim` helper | — | med | `WRO2026/PivotTrimPolicy.h` | Add `SensorOvershootResolution` + `resolveSensorOvershootTrim(axleOffsetCm, approachingTrimCm, minDriveCm)` as a thin wrapper over `resolvePivotTrim(axleOffsetCm, 0.0f, approachingTrimCm, minDriveCm)` (map `driveCm→overshootCm`). Doxygen: departing trim intentionally NOT a parameter (D1). Header-only inline. |
| T3 | Host test: overshoot policy | T2 | low | `WRO2026/test/test_sensor_overshoot_policy.cpp`, `WRO2026/test/CMakeLists.txt` | TC1–TC3 against `resolveSensorOvershootTrim`. Register the target in CMakeLists. |
| T4 | PathTracker: seek overshoot + LINE refactor | T2 | high | `WRO2026/PathTracker.cpp` | (a) In the BLIND seek dispatch (the `Drive.stopAtLine(edge.distance_cm)…` chain at 1665–1669): when `_approachingPivot.active`, chain `.setOvershoot(resolveSensorOvershootTrim(Config::LINE_SENSOR_AXLE_OFFSET_CM, _approachingPivot.trim_cm, Config::PIVOT_TRIM_MIN_DRIVE_CM).overshootCm)`, emitting the existing underflow Serial trace when `clamped`. Keep the move-only RAII chain as one statement. (b) Refactor the inline LINE overshoot reduction (826–840) to call the same helper, **passing its existing direction-resolved `overshootAxleOffsetCm`** (NOT a hard-coded FRONT constant) so REAR LINE-into-pivot stays byte-identical. `_approachingPivot` is live here (the non-seek BLIND path below already reads it via `_pivotTrimDistance`). Do NOT subtract departing trim (D1). Satisfies AC1, AC2, AC3. |
| T5 | Host inline-mirror: seek decision | T4, T3 | low | `WRO2026/test/test_path_tracker_stop_at_line.cpp`, `WRO2026/test/CMakeLists.txt` | Extend the existing inline-mirror PathTracker stop-at-line test with TC4: mirror the new seek-overshoot decision, annotated `// MIRROR: PathTracker.cpp:1665`. This is a decision-slice MIRROR, NOT a link of PathTracker.cpp (no host target links it). The integration landing is TC7 (sim). Blocked by T3 so the CMakeLists edits serialize. |
| T6 | Sim: land seek δ-short | T2 | high | `tools/path_planner/sim/runtime.py` | The sim FRONT seek is a single `forward(distance_cm)` landing the axle on the node (no separate overshoot phase). When the approaching pivot is active, shorten it to `forward(distance_cm − (offset − overshootCm))`, where `overshootCm = resolveSensorOvershootTrim(offset, δ, floor).overshootCm` and `offset − overshootCm` is exactly how far short of the node the firmware's axle lands (`= δ` in the clean case). Add the Python twin of `resolveSensorOvershootTrim` and refactor the sim's LINE overshoot reduction to share it. Freeze the formula against T2's merged signature. Step-for-step identical with T4. Satisfies AC4. |
| T7 | Validator: budget seek-incoming | — | high | `tools/path_planner/validator.py` | Change the budget check (687) to `in_regime in ("line", "sensor")`; update the docstring matrix (370–378) and inline comment (680–686) to drop the "EXEMPT/review fix 3" claim; widen the error message to name the sensor case + escape (BLIND approach / center turn / issue #75). Leave the chained sum-check distance-only (704–707). **This is a TIGHTENING, not additive** — it can newly reject a committed field whose pivot has a `"sensor"` incoming (authored STOP_AT_LINE or auto-rule BLIND→PROJECTION) with δ>offset; AC7 + T11 guard that. Satisfies AC5. |
| T8 | Validator tests | T7, T10 | low | `tools/path_planner/tests/test_validator_pivot_seek.py` | TC10 (δ>offset rejected, authored + auto-rule variants; assert message + key path), TC11 (δ≤offset passes), TC12 (departing-seek shared edge not double-budgeted). |
| T9 | Sim parity tests | T6, T10 | med | `tools/path_planner/tests/test_sim_pivot_seek.py` | TC6 (FRONT-forcing), TC7 (δ-short landing, auto-rule), TC8 (chained), TC9 (call-site ON), TC13 (firmware↔sim parity grid — extend existing pivot-trim parity). |
| T10 | Sensor-incoming pivot fixtures | — | low | `tools/path_planner/tests/fixtures/pivot_field_data.py` | Add a δ≤offset sensor-incoming pivot (may join `all_fields()`) AND a δ>offset sensor-incoming pivot in a **separate factory excluded from `all_fields()`** (so `test_pivot_fixtures_validate` doesn't break). Cover both authored STOP_AT_LINE and auto-rule BLIND→PROJECTION shapes. No new strict-read attribute ([[gotcha-strict-emitter-field-fixtures]]). |
| T11 | Pre-land guard: no committed field newly rejected | T7 | low | (read-only sweep) | Before landing T7, grep/scan the real `field_data` + every committed fixture for a pivot with a `"sensor"`-incoming leg (authored STOP_AT_LINE, per-edge override, or BLIND→PROJECTION under the auto-rule) whose δ>offset; run `build_routes` + full pytest. If any newly fails, surface it (it is a latent real bug, not a test problem) before merge. Satisfies AC7. |

Parallel start: T1, T2, T6, T7, T10 (no blockers). T3 after T2; T4 after T2; T5 after T4+T3; T8 after T7+T10; T9 after T6+T10; T11 after T7.

## Notes for Implementer

- **No `_pivotTrimDistance` on the seek path.** The seek is sensor-terminated; the fix only touches the *overshoot*, never the seek's `expectedCm`/distance or arm window. Do not subtract the departing trim anywhere on the seek (D1) — the firmware comment (1625–1627) + the Q1 trace confirm it; the Opus consult was wrong on this one point.
- **The seek fires via the ADR-038 ladder.** The user's BLIND→PROJECTION case is rung 4 (auto-rule, `_nextHopIsProjection()` at 1620). The fix is rung-agnostic (keys on `_approachingPivot.active`). Don't gate the fix on edge mode or the auto-rule.
- **Refactor risk (T4b).** The LINE overshoot path is working firmware and uses a **direction-resolved** offset (`overshootAxleOffsetCm` = FRONT or REAR). The helper takes the offset as a parameter — pass the existing local, never a hard-coded `Config::LINE_SENSOR_AXLE_OFFSET_CM`, or REAR LINE-into-pivot breaks AC3. Keep T4a (seek) and T4b (LINE refactor) reviewable as two hunks.
- **Host-link reality.** `PivotTrimPolicy.h` is host-linkable (policy headers are). `PathTracker.cpp` is NOT linked by any host target (CMakeLists 53–55, 285–301) — use the inline-MIRROR pattern for T5 and the sim for landing assertions (T7/TC7).
- **Parity is load-bearing.** The firmware (T4) and sim (T6) implement the same formula in two languages; TC13 is the guard. A mismatch silently breaks the sim as a verification oracle.
- **Validator can't see pure call-site `setStopAtLine(ON)`** (D4) — do not try to make it; the runtime clamp + warning is the documented backstop. Say so in the ADR. It DOES see the auto-rule and per-edge/authored edges.
- **Config constants** live in `WRO2026/config/ConfigDrive.h` (`LINE_SENSOR_AXLE_OFFSET_CM`, `LINE_SENSOR_AXLE_OFFSET_REAR_CM`, `PIVOT_TRIM_MIN_DRIVE_CM`); `Config.h` is the umbrella re-include ([[project-config-umbrella-subheaders]]). The validator parses them via `parse_config_float(...)`.
- **CI freshness.** No emitter/`field_data` change ⇒ no `routes.h/.cpp` regen. If a *committed* toy/golden field ever gains a pivot, run `uv run python tools/path_planner/build_routes.py` and commit, or CI's `git diff --exit-code` fails ([[gotcha_ci_routes_freshness]]).
- **ADR number** — re-verify 043 is free against `origin/main` AND open PRs at write time; the in-flight branches use a divergent 038–042 numbering ([[gotcha-adr-number-collision]]).
- **Worktree venv** — this worktree has no `.venv`; run `uv sync --project <worktree>` before pytest in it ([[gotcha-worktree-venv-uv-sync]]).
- **Rollback:** all changes are additive except the validator *tightening* (T7), which can reject a previously-passing field — T11/AC7 prove it rejects none currently committed. Reverting T4/T6/T7 restores prior behavior; the new tests fail closed.
