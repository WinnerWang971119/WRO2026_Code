# Wall-zero rear-contact + margin + return-to-node Plan

**Goal:** Fix the wall-zero reset-node interlude (PR #71) so it touches the wall with the **rear** of the chassis (the front would break the mechanism), re-anchors the believed pose to the actual axle position (not the wall surface), and leaves the robot parked **on the node with turn clearance** instead of pressed against the wall.

**Approach:** Three coupled corrections on top of the existing `WALL_STOP → REANCHOR` interlude, all on PR #71's branch `feat/wall-zero-reset-nodes`. (1) The push goes **reverse** with the dispatched heading set to a redefined `facing` = parked heading (front *away* from the wall); the existing `wallStop` SQUARE phase turns the chassis rear-to-wall before pushing. (2) `reanchorAtWall` subtracts a new `WALL_CONTACT_OFFSET_CM` (rear-contact-face → wheel-axle) so belief lands on the axle, not the wall surface. (3) A new post-`REANCHOR` FSM state drives the robot forward back onto the node (a fresh, clean encoder move immune to the wall-hit slip), so any following pivot has room. A validator clearance floor (`WALL_PIVOT_CLEARANCE_CM`) and the inverted facing-direction check enforce the new contract at authoring time. The whole box/manipulator pre-stage and carrying-state guard stay deferred to issue #77.

## Scope

- **In scope:**
  - Push the rear into the wall: `_dispatchWallStop` chains `.setReverse(true).setBackOff(0.0f)` (keeps the reviewed `DriveMode::RAW`); the SQUARE phase reorients to `facing`.
  - Redefine `facing` semantics to the **parked heading** (robot front pointing away from the wall); flip the toy fixture `Wall1.facing` 90 → 270 and regenerate `routes.h/.cpp` + the toy golden.
  - `reanchorAtWall` sets `belief[axis] = wall_coord − sign(wall_coord − node_coord[axis]) · WALL_CONTACT_OFFSET_CM` instead of `= wall_coord`; heading-square and atomicity unchanged.
  - New PathTracker FSM state after `REANCHOR` (before `PLAN_NEXT_HOP`): `driveDist(standoff − WALL_CONTACT_OFFSET_CM)` forward (away from wall), then `clearOffset()` so belief == node. Keep the `_resetNode == cur` one-shot latch set through it.
  - Two new constants in `WRO2026/config/ConfigNav.h`: `WALL_CONTACT_OFFSET_CM` (placeholder + `TODO: measure at bring-up`) and `WALL_PIVOT_CLEARANCE_CM` (= pivot sweep radius 12.5 + ~1.5 safety = 14.0). A `ConfigChecks.h` `static_assert(WALL_PIVOT_CLEARANCE_CM >= WALL_CONTACT_OFFSET_CM)`.
  - `validator.py`: **invert** the existing `facing`-toward-wall geometry check (now `facing` must point *away* from the wall; the rear, at `facing+180`, faces it) and update its message/comment; **add** a clearance-floor check `|wall_coord − node_coord[axis]| ≥ WALL_PIVOT_CLEARANCE_CM` (value hand-mirrored from ConfigNav.h).
  - Sim parity: mirror `WALL_CONTACT_OFFSET_CM` in `sim/runtime.py`; update `_dispatch_wall_reset` (belief = wall − sign·offset, then a return-to-node move → belief = node) and TC18b.
  - Host tests updated for the new belief value + the new depart state; a Python test for the inverted facing check + clearance floor.
  - Amend **ADR-043** in place (rear/reverse contact, `facing` = parked-heading redefinition, the margin subtraction, the return-to-node clearance step, single-axis-only re-anchor note); cross-reference issue #77.

- **Out of scope (deferred to issue #77):**
  - The manipulator/box pre-stage at the wall and the variable-contact-margin-with-box-extension problem.
  - The "don't wall-zero while carrying a piece" guard (needs a load-state signal not yet built).
  - Switching the wall push from `RAW` to `GYRO`, or enabling `WALL_NORMAL_STRICT_ENABLED` for reset nodes — both are bench-tune decisions referenced in the ADR, not code changes here.
  - PR #72 (blind-decay) files; #72 rebases onto the fixed #71 afterward.

## Decisions

- **Target = PR #71 branch `feat/wall-zero-reset-nodes`** (worktree `.claude/worktrees/pr71`, HEAD d50f2a7, already rebased onto the config-split main). #72 rebases after. — user.
- **Rear contact is a chassis invariant → wall push is always reverse**, not a per-node flag. `facing` redefined as the parked heading (front away from wall). — user + discussion.
- **ADR amended in place** (PR #71 unmerged, so ADR-043 isn't in force yet); no superseding ADR. — user.
- **Back-off = 0 for the wall-zero** (`.setBackOff(0.0f)`); the return-to-node move is the sole clearance mechanism, so belief == physical at the reanchor instant (the axle sits `WALL_CONTACT_OFFSET_CM` behind the rear contact face, which is at the wall). — user.
- **The wall-depart move is `GYRO` holding `toImuYaw(facing)`**, not RAW — a 7 cm open-loop RAW move would drift off-axis with no correction; GYRO matches the existing dead-reckon-return convention. The *push* stays RAW; only the depart is GYRO. — Opus critic.
- **The SQUARE pre-push turn is a fixed `COUNTER_ROT` in-place turn** inside `_updateWallStop` (not governed by `_wallMode`); it reorients the chassis to `facing` regardless of the RAW push mode. — code-confirmed (Drive.cpp `_execTurnToYaw`).
- **Keep the toy node at its 15 cm standoff**; set `WALL_PIVOT_CLEARANCE_CM = 14.0` so 15 cm passes, minimizing routes/golden churn (only `facing` changes). The 15 cm standoff vs the 12.5 cm placeholder pivot radius leaves only ~1.5 cm clearance on the 180° SQUARE turn — a re-validation trigger if the measured pivot radius rises. — user.
- **Both new constants live in `WRO2026/config/ConfigNav.h`** (the nav config home that already holds `WALL_NORMAL_*`); `validator.py` hand-mirrors `WALL_PIVOT_CLEARANCE_CM` (same lockstep pattern `sim/runtime.py` uses for `WALL_STOP_BACK_OFF_CM`). — user ("ConfigNav.h").
- **Keep `DriveMode::RAW` for the push** (the d50f2a7 review-fix deliberately switched from GYRO to RAW); orientation comes from the SQUARE phase, heading correctness from the reanchor square. — code-confirmed.
- **Dedicated new FSM state for the wall departure**, NOT a reuse of the offset-driven `RETURN_TO_NODE` state (which computes heading via `atan2` of an offset vector and re-acquires a line — wrong for this fixed geometric move on a blind-only node). — Opus consult.
- **`static_assert(WALL_PIVOT_CLEARANCE_CM >= WALL_CONTACT_OFFSET_CM)`** so the "return distance ≥ 0" guarantee can't be silently broken when the two TODO-measure constants are measured independently. — Opus consult.

## Acceptance Criteria

- [ ] AC1: At a reset node, the chassis squares to `facing` (rear toward the wall) and pushes **reverse** until the rear stalls; `_dispatchWallStop` chains `.setReverse(true).setBackOff(0.0f)` and keeps `DriveMode::RAW`.
- [ ] AC2: After the stall, `reanchorAtWall(axis, wall_coord, facing)` sets `belief[axis] == wall_coord − sign(wall_coord − node_coord)·WALL_CONTACT_OFFSET_CM` (the axle position, not the wall surface), squares heading to `facing`, zeroes the off-axis + heading-delta, atomically. Reject path (IMU read fail / strict check) leaves pose byte-for-byte unchanged.
- [ ] AC3: After `REANCHOR`, the FSM enters the new depart state, drives `driveDist(standoff − WALL_CONTACT_OFFSET_CM)` forward, and on idle `clearOffset()` so `belief == node coordinate` (offset 0); only then does it reach `PLAN_NEXT_HOP`. The `_resetNode == cur` latch stays set through the depart state (the wall push does not re-fire).
- [ ] AC4: `validator.py` rejects a reset node whose `facing` points **toward** the wall or **orthogonal** to the wall axis (the rear, at `facing+180`, must face the wall) with a clear message — new reject predicate `comp·toward >= 0`. It also rejects any standoff `< WALL_PIVOT_CLEARANCE_CM`; the boundary `== WALL_PIVOT_CLEARANCE_CM` is accepted. A zero-gap (on-wall) node is rejected by the clearance floor (0 < 14), so the old "any facing OK at zero gap" allowance is now obsolete — the clearance floor wins.
- [ ] AC9: When the depart distance `standoff − WALL_CONTACT_OFFSET_CM` is `<= 0` (degenerate; can't occur on a validator-passing field but guarded defensively), the FSM skips the `driveDist` dispatch, still runs `clearOffset()`, and advances to `PLAN_NEXT_HOP` with no hang.
- [ ] AC5: `Wall1.facing` is 270 in `field_data.json`; `routes.h/.cpp` + the toy golden regenerate cleanly and the CI freshness gate (`build_routes` + `git diff --exit-code`) passes.
- [ ] AC6: `PathTrackerSim` mirrors the reverse push, margin-corrected re-anchor, and the return-to-node move; TC18b asserts final believed pose == node coordinate (not wall_coord), with injected inbound drift corrected.
- [ ] AC7: `static_assert(WALL_PIVOT_CLEARANCE_CM >= WALL_CONTACT_OFFSET_CM)` compiles; the placeholder values (offset 8.0, clearance 14.0) satisfy it.
- [ ] AC8: ADR-043 reflects rear/reverse contact, `facing` = parked-heading, the margin subtraction, the return-to-node step, and the single-axis-only re-anchor caveat; cross-links issue #77.

## Data Model

### `WRO2026/config/ConfigNav.h` (new constants)
```cpp
// Rear-contact-face -> wheel-axle distance. reanchorAtWall subtracts this so the
// re-zeroed axis lands on the AXLE (the tracked reference), not the wall surface.
// TODO: measure at bring-up (depends on the empty-box rest geometry; see issue #77).
constexpr float WALL_CONTACT_OFFSET_CM = 8.0f;   // TODO measure

// Minimum node-to-wall standoff a reset node may be authored at: the pivot sweep
// radius (axle -> farthest corner, ~12.5 cm) plus a safety margin. Guarantees both
// the SQUARE pre-push turn and any post-reanchor pivot clear the wall, and (with the
// static_assert below) that the return distance standoff - WALL_CONTACT_OFFSET_CM >= 0.
// Hand-mirrored in tools/path_planner/validator.py — keep in lockstep.
constexpr float WALL_PIVOT_CLEARANCE_CM = 14.0f;   // TODO bench-tune (12.5 pivot radius + safety)
```

### `WRO2026/config/ConfigChecks.h`
```cpp
static_assert(Config::WALL_PIVOT_CLEARANCE_CM >= Config::WALL_CONTACT_OFFSET_CM,
              "WALL_PIVOT_CLEARANCE_CM must be >= WALL_CONTACT_OFFSET_CM, else an "
              "authored reset node can pass the clearance floor yet command a negative "
              "return distance (Drive.driveDist REJECTS a negative distance with no "
              "command -> robot never leaves the wall, then pivots into it).");
```

### `field_data.json`
```jsonc
"RESET_NODES": { "Wall1": { "axis": "y", "wall_coord": 154.0, "facing": 270.0 } }  // facing 90 -> 270
```
`ResetNode` struct (`routes.h`) and the `{axis, wall_coord, facing}` schema are **unchanged**; only the `facing` value changes (re-emits as `int16_t 270`).

### PathTracker FSM
New `State` enum value (name TBD, e.g. `WALL_DEPART`), inserted between `REANCHOR` and `PLAN_NEXT_HOP`. No change to the `Fault` enum.

### `sim/runtime.py`
```python
# MUST mirror Config::WALL_CONTACT_OFFSET_CM in WRO2026/config/ConfigNav.h.
WALL_CONTACT_OFFSET_CM: float = 8.0   # keep in lockstep with the firmware constant
```

## API Contracts

```cpp
// Localization — signature UNCHANGED; behavior change only (margin subtraction).
bool reanchorAtWall(uint8_t axis, float wallCoord, float headingDeg);
//   _offset[axis] = (wallCoord - sign(wallCoord - nodeCoord)*WALL_CONTACT_OFFSET_CM) - nodeCoord

// PathTracker — dispatch change:
Drive.wallStop(distToWall).setReverse(true).setMode(DriveMode::RAW)
     .setHeading(toImuYaw(facing)).setBackOff(0.0f).async();   // was setReverse(false), default back-off
// New state on REANCHOR success: if (standoff - WALL_CONTACT_OFFSET_CM) > eps,
//   driveDist(standoff - WALL_CONTACT_OFFSET_CM).setMode(GYRO).setHeading(toImuYaw(facing)).async()
//   (forward, away from wall) -> on Drive.isIdle(): Localization.clearOffset() -> State::PLAN_NEXT_HOP.
//   Else (<= eps): skip dispatch, clearOffset(), -> PLAN_NEXT_HOP directly.

// validator.py — _validate_reset_nodes:
//   (a) INVERT the geometry test: new reject predicate `comp * toward >= 0.0` (reject when facing
//       points TOWARD the wall OR is orthogonal to the wall axis; facing must point AWAY so the rear
//       at facing+180 presses the wall). The old `<= 0.0` reject is flipped, boundary included.
//   (b) ADD: abs(wall_coord - node_coord) >= WALL_PIVOT_CLEARANCE_CM (mirrored constant). A zero-gap
//       node fails this floor, so the prior zero-gap facing skip is moot (clearance floor wins).
```

## Error Handling

- **`facing` points toward the wall (mis-author):** validator raises `FieldDataError` naming the node, axis, node/wall coords, and the required away-direction.
- **Standoff below clearance floor:** validator raises `FieldDataError` with the measured gap and the floor.
- **`standoff − WALL_CONTACT_OFFSET_CM ≤ 0` at runtime:** prevented at build by the validator floor + the `static_assert`. Defensive: the depart state guards `departCm > eps` *before* dispatch (because `Drive.driveDist` **rejects** a negative distance outright — no command enqueued — and a `0.0` distance enqueues a no-op move); on the `≤ eps` branch it skips the dispatch, runs `clearOffset()`, and advances to `PLAN_NEXT_HOP` on the same tick (no hang). Log a one-line Serial warning.
- **Reverse push never stalls:** unchanged advisory-timeout path (`wallStopTimedOut()`), assume-at-wall, re-anchor, depart, continue.
- **Cross-axis (off-wall-axis) drift:** the re-anchor fixes ONE axis + heading only; off-axis error survives (the wall gives no off-axis information). Documented in ADR-043, not corrected here.
- **Corner-first / skewed stall:** the heading-square trusts physical squareness (`WALL_NORMAL_STRICT_ENABLED` is off by default). Noted in ADR-043 as a bench-tune; no code change.

## Testing Strategy

**Levels:** Unit (host C++ ctest + Python pytest), Integration (sim parity). Hardware bench gates referenced, not automated.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `reanchorAtWall` success, positive-axis (wall > node) | Unit (C++) | `belief.y == wall_coord − WALL_CONTACT_OFFSET_CM`; heading == 270; off-axis + theta ≈ 0; node unchanged (AC2) |
| TC1b | `reanchorAtWall` success, **negative-axis** (wall < node) | Unit (C++) | `belief == wall_coord + WALL_CONTACT_OFFSET_CM` (sign branch); exercises `sign(gap) = −1` (AC2) |
| TC2 | `reanchorAtWall` reject path (IMU fail) | Unit (C++) | returns false; pose byte-for-byte unchanged (AC2) |
| TC3 | reset-node arrival full interlude | Unit (C++) | WALL_STOP → REANCHOR → **WALL_DEPART** → PLAN_NEXT_HOP; final `belief == node coord` after the depart drive + `clearOffset`; latch held (AC1, AC3) |
| TC3b | depart distance ≤ 0 (degenerate, guarded) | Unit (C++) | dispatch skipped, `clearOffset()` runs, advances to PLAN_NEXT_HOP, no hang (AC9) |
| TC4 | strict-reject still faults before depart | Unit (C++) | `reanchorOk == false` → `Fault::WALL_NOT_SQUARE`, no depart, latch re-armed (AC2) |
| TC5 | advisory timeout still continues | Unit (C++) | assume-at-wall → reanchor → depart → continue; advisory surfaced, no fault (AC1) |
| TC6 | dispatch flags | Unit (C++) / read | `.setReverse(true).setBackOff(0.0f)`, push mode RAW; depart mode GYRO (AC1) |
| TC7 | validator: facing toward wall **rewrite** | Unit (Py) | `FieldDataError` (facing must point away); REWRITES the old test that expected away-facing to be rejected (AC4) |
| TC8 | validator: facing away accepted (270) | Unit (Py) | passes (AC4) |
| TC9 | validator: standoff < floor rejected; == floor accepted; **zero-gap rejected** | Unit (Py) | reject below floor + reject on-wall zero-gap; accept at boundary; REWRITE the old "any facing at zero gap OK" test (AC4) |
| TC10 | sim TC18b parity (rebuilt fixture, standoff ≥ 14) | Integration | primitive seq includes `ACT_WALL_DEPART`; final believed pose == node (not wall); injected drift corrected (AC6) |
| TC11 | `static_assert` holds | Compile | builds with offset 8.0 / clearance 14.0 (AC7) |
| TC12 | routes freshness | CI | `build_routes` + `git diff --exit-code` clean after `facing` 90→270 (AC5) |

**Test data:** existing toy fixture (`Wall1`), host C++ mock harness (Drive encoders/IMU, real `Localization`/`routes`), Python `toy_`/`pivot_` fixtures.
**Run commands:** host C++ ctest harness; `uv run --extra dev pytest tools/path_planner/tests/`; `arduino-cli compile` for link/ODR; `uv run python tools/path_planner/build_routes.py` to regenerate.

## Tasks

> All paths are in the `feat/wall-zero-reset-nodes` worktree. Note the config-split layout: sub-headers live under `WRO2026/config/`.

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | ConfigNav constants + static_assert | — | low | `WRO2026/config/ConfigNav.h`, `WRO2026/config/ConfigChecks.h` | Add `WALL_CONTACT_OFFSET_CM = 8.0f` (TODO measure) and `WALL_PIVOT_CLEARANCE_CM = 14.0f` (TODO bench-tune) to ConfigNav.h with Doxygen `@brief`/`@note`/`@warning` matching the existing `WALL_NORMAL_*` style; note the validator.py lockstep mirror. Add the `static_assert(WALL_PIVOT_CLEARANCE_CM >= WALL_CONTACT_OFFSET_CM)` to ConfigChecks.h (included last). Satisfies AC7. |
| T2 | reanchorAtWall margin subtraction | T1 | high | `WRO2026/Localization.cpp`, `WRO2026/Localization.h` | In `reanchorAtWall`, replace `_offset[axis] = wallCoord - nodeCoord` with `gap = wallCoord - nodeCoord; sign = gap >= 0 ? 1.f : -1.f; _offset[axis] = gap - sign*Config::WALL_CONTACT_OFFSET_CM` for both the x and y branches. Keep the single IMU read, atomicity (decide before any write), `_consumeOffset()` baseline re-snapshot, and heading-square unchanged. Update the method docstring + the inline comment that currently says belief == wallCoord. Single-writer rule preserved. Satisfies AC2. |
| T3 | PathTracker reverse dispatch + depart state | T1, T2 | high | `WRO2026/PathTracker.h`, `WRO2026/PathTracker.cpp` | (a) In `_dispatchWallStop`, change `.setReverse(false)` → `.setReverse(true)` and add `.setBackOff(0.0f)`; keep `DriveMode::RAW` and `setHeading(toImuYaw(facing))`; update the inline comment (rear push, no back-off) **and** the stale `PathTracker.h` class/state docstrings (lines ~81-88) that currently say "forward" / "GYRO mode" → rear/reverse + RAW. (b) Add a new `State` enum value `WALL_DEPART`. On `REANCHOR` success (after the existing strict-check branch), compute `standoff = fabsf(wall_coord - nodeCoord[axis])`, `departCm = standoff - Config::WALL_CONTACT_OFFSET_CM`; if `departCm > eps` dispatch `Drive.driveDist(departCm).setMode(DriveMode::GYRO).setHeading(toImuYaw(facing)).async()` (forward, away from wall — GYRO, NOT RAW, so the short move holds heading) and enter `WALL_DEPART`; else (`<= eps`) `clearOffset()` + `_state = PLAN_NEXT_HOP` directly (degenerate guard, AC9 — driveDist would reject a negative arg). (c) `WALL_DEPART` polls `Drive.isIdle()`, then `Localization.clearOffset()` and `_state = State::PLAN_NEXT_HOP`. Keep `_resetNode == cur` latched through `WALL_DEPART` (do NOT clear it). Add the `case State::WALL_DEPART:` to the exhaustive `_stepFsm` switch (no `default:`) + any state-name/`faultMnemonic` helper. Satisfies AC1, AC3, AC9. |
| T4 | validator: invert facing check + clearance floor | — | med | `tools/path_planner/validator.py` | In `_validate_reset_nodes`: (a) flip the geometry test from reject-`comp*toward <= 0.0` to **reject-`comp*toward >= 0.0`** — now `facing` toward the wall OR orthogonal to the wall axis (`comp*toward == 0`) is rejected; `facing` must point AWAY so the rear (`facing+180`) presses the wall. Rewrite the comment + error message (the firmware presses REARWARD; `facing` is the parked heading away from the wall; mode is RAW not GYRO). (b) Add a module constant `WALL_PIVOT_CLEARANCE_CM = 14.0` with a "MUST mirror Config::WALL_PIVOT_CLEARANCE_CM (ConfigNav.h)" comment, and reject `abs(wall_coord - node_coord) < WALL_PIVOT_CLEARANCE_CM` (boundary `==` accepted). The clearance floor now rejects a zero-gap on-wall node (0 < 14), which supersedes the old facing-check zero-gap skip — order the checks so the floor runs and the "any facing at zero gap" allowance is removed. Satisfies AC4. |
| T5 | field_data facing 90→270 | T4 | low | `tools/path_planner/field_data.json` | Change `RESET_NODES.Wall1.facing` 90.0 → 270.0 so it passes the inverted validator. No other field change. Satisfies AC5. |
| T6 | regenerate routes + toy golden | T5 | low | `WRO2026/routes.h`, `WRO2026/routes.cpp`, `tools/path_planner/tests/golden/*` | Run `uv run python tools/path_planner/build_routes.py`; commit regenerated `routes.h/.cpp` (facing emits int16_t 270) and any toy golden that holds the reset row. Verify `git diff --exit-code` clean afterward. Satisfies AC5. |
| T7 | sim mirror: offset + return-to-node + new tag | T1, T2, T3 | med | `tools/path_planner/sim/runtime.py` | Add `WALL_CONTACT_OFFSET_CM = 8.0` (mirror comment) to the constants block + `__all__`, and a new `ACT_WALL_DEPART` primitive tag (+ `__all__` entry + the constants import block at the top). In `_dispatch_wall_reset`: model the reverse push (rear to wall), set believed on-axis to `wall_coord - sign*WALL_CONTACT_OFFSET_CM` in the reanchor, then append the `ACT_WALL_DEPART` forward move (`standoff - offset`) that lands the chassis on the node and clears the offset (final believed pose == node coord). Update the now-stale `LocalizationSim.reanchor_at_wall` docstring (says belief == wall_coord). Keep primitive-tuple parity with the firmware sequence. Satisfies AC6. |
| T8a | host C++ tests | T2, T3 | low | `WRO2026/test/test_reanchor_at_wall.cpp`, `WRO2026/test/test_path_tracker_reset_arrival.cpp` | Update TC1 expected belief from `wall_coord` to `wall_coord − WALL_CONTACT_OFFSET_CM` and add TC1b (negative-axis: `wall < node` → belief `wall + offset`); update `kFacingDeg` 90→270 + its comment. Update the reset-arrival test for the new `WALL_DEPART` state and final `belief == node coord`; add TC3b (depart `≤ eps` → dispatch skipped, clearOffset, PLAN_NEXT_HOP, no hang); keep the strict-reject (`WALL_NOT_SQUARE`) and advisory-timeout cases (now ending in depart + continue) and assert depart mode GYRO. `test_wall_stop_watchdog.cpp` stays unchanged (direction-agnostic). Add the `WALL_DEPART` state to any mock/state harness. Satisfies AC2, AC3, AC9 (TC1–TC6, TC1b, TC3b). |
| T8b | Python tests: validator + sim (rewrites) | T4, T7 | med | `tools/path_planner/tests/test_field_data.py`, `tools/path_planner/tests/test_runtime_sim.py` | **REWRITE** the existing validator tests that encode the OLD toward-wall semantics: the test asserting away-facing is rejected (now must PASS), the negative-axis toward-wall-passes test (now rejected), and the "any facing at zero gap OK" test (now zero-gap rejected by the clearance floor); add a standoff-below-floor reject + `==`-boundary accept (TC7–TC9). **Rebuild** the `_reset_chain_field` fixture in `test_runtime_sim.py` so `wall_coord ≥ node_coord + WALL_PIVOT_CLEARANCE_CM` (the current fixture has zero standoff — incompatible with the new margin/return math) and update both TC18b tests: assert `ACT_WALL_DEPART` in the primitive sequence and final believed pose == node coordinate, drift corrected (TC10). Satisfies AC4, AC6. |
| T9 | amend ADR-043 + index | T2, T3 | low | `docs/decisions/043-wall-zero-reset-nodes.md`, `DECISIONS.md` | Amend ADR-043 in place: rear/reverse contact (SQUARE reorients to `facing`), `facing` = parked-heading-away-from-wall redefinition, the `WALL_CONTACT_OFFSET_CM` margin subtraction (belief on the axle), the return-to-node clearance step + `WALL_PIVOT_CLEARANCE_CM` floor, and the single-axis-only re-anchor caveat (cross-axis drift uncorrected). Note RAW push + strict-flag are bench-tunes. Cross-reference issue #77 for the deferred box pre-stage / carrying guard. Update the DECISIONS.md one-liner if the summary changed. Satisfies AC8. |

## Notes for Implementer

- **Work in the `feat/wall-zero-reset-nodes` worktree** (`.claude/worktrees/pr71`, config-split layout: `WRO2026/config/Config*.h`). The main checkout is the stale pre-rebase #72 — do not edit there.
- **Worktree venv:** the gitignored `.venv` isn't copied into a worktree — run `uv sync --extra dev` against the worktree before pytest (gotcha: `gotcha-worktree-venv-uv-sync`).
- **The SQUARE phase already handles orientation.** `_updateWallStop` turns the chassis in place to `_wallHeading` (= the dispatched `toImuYaw(facing)`) before the push, via a fixed `COUNTER_ROT` in-place turn (`_execTurnToYaw`) that is NOT governed by `_wallMode` — so it reorients regardless of the RAW push mode. A reverse push with `facing=270` reorients rear-to-wall (a ~180° turn at the node, since the inbound hop arrives facing roughly toward the wall). The ≥14 cm clearance floor protects that turn AND any post-reanchor pivot; at the toy node's 15 cm standoff the SQUARE turn clears the wall by only ~1.5 cm — tight, bench-verify, do not lower the floor. **Re-validation trigger:** the floor (14.0) and offset (8.0) are TODO-measure placeholders; if the measured pivot radius rises above ~13.5 cm the toy node's 15 cm standoff fails its own floor — re-confirm both constants on hardware before trusting the toy fixture.
- **`facing` self-consistency (verified):** heading 90 = +y; Wall1's wall is on +y, so parked-away = 270. The reverse push (`_wallReverse ? -mag`) moves the rear toward +y while the chassis holds front=270. The reanchor squares to 270. All consistent.
- **Belief math (verified for Wall1):** wall 154, node 139, sign(+15)=+1 → belief.y = 154 − 8 = 146 at reanchor; depart drives `15 − 8 = 7` cm forward → axle at 139 = node; `clearOffset` makes belief = node. Keep belief == physical at each step (this is why back-off is 0).
- **Do NOT reuse `RETURN_TO_NODE`** (the operator-offset DEQUEUE return): it derives heading from an `atan2` offset vector and re-acquires a line for LINE nodes — wrong for this fixed geometric move on a blind-only node. Use the dedicated `WALL_DEPART` state.
- **Latch hygiene:** `_resetNode == cur` is set in `_dispatchWallStop` and must stay set through `WALL_DEPART`; on the route back to `PLAN_NEXT_HOP`, `_maybeEnterReset(cur)` returns false (latch still set) so the push does not re-fire. It re-arms on DEQUEUE / when the FSM advances off the node.
- **Encoder-slip immunity:** the wall hit corrupts encoders only during the stall; `reanchorAtWall`'s `_consumeOffset()` re-snapshots the integrator baselines, so the `WALL_DEPART` `driveDist` measures from a clean baseline — its precision does not depend on the hit.
- **Constant lockstep:** `WALL_CONTACT_OFFSET_CM` exists in ConfigNav.h (firmware) AND `sim/runtime.py` (hand-mirror). `WALL_PIVOT_CLEARANCE_CM` exists in ConfigNav.h AND `validator.py` (hand-mirror). Both pairs must match; both are TODO-measure placeholders (8.0 / 14.0).
- **Regenerate + commit `routes.h/.cpp` + the toy golden** after the `facing` change or CI fails (gotcha: `gotcha_ci_routes_freshness`). Run pytest via `uv run --extra dev pytest` (gotcha: `gotcha-uv-run-extra-dev-pytest`). Only `arduino-cli` catches link/ODR errors locally (a new `State` enum value can trip ODR/switch-coverage).
- **Adding a new `State` enum value** touches the switch in `_stepFsm`, any `faultMnemonic`/state-name helper, and the sim's FSM mirror — keep them in sync or the parity test breaks.
- **Rollback:** each correction is independent — the dispatch flags, the reanchor margin, and the depart state can each be reverted without breaking the others (a reverted depart leaves the robot pressed but re-anchored; a reverted margin leaves the old wall-surface belief).
