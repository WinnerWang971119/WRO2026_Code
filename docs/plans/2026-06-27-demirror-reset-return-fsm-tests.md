# De-mirror reset_arrival + return_to_node onto the real PathTracker FSM (#113)

**Goal:** Delete the last two inline FSM `// MIRROR:` / `replica_*` slices in the host C++ test suite by driving the REAL `PathTracker` FSM through the shared #106 harness, so ADR-030 can claim zero mirrored FSM logic remains.

**Approach:** Apply the proven #106 real-link recipe to the two laggard targets (`test_path_tracker_reset_arrival`, `test_return_to_node_dispatch`): link the real `PathTracker.cpp` + `Drive.cpp` + `Localization.cpp` + `YawSource.cpp` + `routes_fixture.cpp` + `path_mocks.cpp`, invert the include order to `SKETCH_DIR`-first, and drop the mock-Drive force-include. Add two additive helpers to the shared harness (`expectWallStop`, `pushOperatorOffset`) to retire a `Drive.wallStop` push and to manufacture a genuine operator offset. Rewrite both test bodies to drive real FSM transitions and delete the replicas. Split `reset_arrival` into strict-OFF + strict-ON siblings (the `test_reanchor_at_wall[_strict]` precedent) so `WALL_NOT_SQUARE` is exercised end-to-end. Finally, update ADR-030 + the DECISIONS index.

Both independent approach consultants (Fable, Codex) endorsed this direction; their binding amendments (budget-loop `expectWallStop`, self-asserting two-tick `pushOperatorOffset`, inbound-hop routing guards, probe-before-drop on the dead branches) are folded in below.

## Scope

- **In scope:**
  - Rewrite `WRO2026/test/test_path_tracker_reset_arrival.cpp` to drive the real RESET-arrival interlude (`_maybeEnterReset` → `WALL_STOP` → `REANCHOR` → `WALL_DEPART` → `PLAN_NEXT_HOP`) through the harness; delete `enum RState` / `struct ResetArrivalMirror` and every `// MIRROR:` annotation.
  - Add a new strict-ON sibling `WRO2026/test/test_path_tracker_reset_arrival_strict.cpp` that drives the real FSM into `Fault::WALL_NOT_SQUARE` (real `reanchorAtWall` rejects under strict + off-normal yaw).
  - Rewrite `WRO2026/test/test_return_to_node_dispatch.cpp` to drive the real `RETURN_TO_NODE` per-kind routing; delete `replica_returnTick` + its `ReturnTickResult` / `ReturnTickOutcome` and every `// MIRROR:` annotation. Keep the direct real-symbol vectors (TC4 over real `FieldMap::getNodeType`, TC4c over real `Config::LINE_SENSOR_AXLE_OFFSET_CM`) as routing guards (they are not mirrors).
  - Add two additive helpers to the shared `WRO2026/test/test_path_tracker_harness.h`: `expectWallStop(...)` and `pushOperatorOffset(...)`.
  - Re-wire the three CMake targets (`test_path_tracker_reset_arrival`, `test_path_tracker_reset_arrival_strict`, `test_return_to_node_dispatch`) onto the #106 real-link recipe in `WRO2026/test/CMakeLists.txt`.
  - Update `docs/decisions/030-harness-links-real-sources-ci-enforced.md` (status + "remaining FSM mirrors" bullet → zero mirrored FSM logic remains, #113 landed) and the `DECISIONS.md` status entry if it tracks this.
  - A short **probe** of the `path_mocks` Drive surface to confirm whether a purely additive mock setter can inject a generic (non-`LINE_NOT_FOUND`) Drive fault that surfaces in `RETURN_TO_NODE` — feeding the drop/keep decision for the two synthetic-only branches.
- **Out of scope:**
  - Any change to production FSM behavior (`PathTracker.cpp` / `Drive.cpp` / `Localization.cpp`). This is a test + CMake + ADR change only.
  - Adding an `OFF_MAP_GATE` compile-time override hook (decided against — no production change). The BEST_EFFORT off-map-gate routing of `RETURN_TO_NODE` stays uncovered at the FSM level; the gate *decision* is already covered by the pure `resolveReturnAction` policy test (`test_return_to_node_policy.cpp`).
  - The routes fixture shape (`fixtures/routes_fixture.{py,h,cpp}`) — left frozen; nodes resolved BY NAME only. (Regenerate only if a `routes.h` schema change forces it — not expected here.)
  - The five already-rewritten #106 tests and the Python sim.
  - Behavioral mock modeling of the Drive done-bit/wall FSM (forbidden by the thin-mock rule, ADR-049 / epic #107).

## Decisions

- **FAULT off-map gate only (no prod change):** the `RETURN_TO_NODE` line-seek `LINE_NOT_FOUND` routing is driven through the real FSM under the shipped `OFF_MAP_GATE == FAULT` (→ `Fault::OFF_NODE`). The BEST_EFFORT branch is not driven at the FSM level (no override hook added). Rationale: keep production untouched; the gate decision is already covered by `test_return_to_node_policy.cpp`.
- **Probe-then-drop the two synthetic-only branches:** the non-`LINE_NOT_FOUND` → `DRIVE_FAULT` discriminator and the BLIND dead-reckon drive-failure → `DRIVE_FAULT` branch are tested by the old `replica_returnTick` but cannot be reached through the real Drive surface (the seek has no IMU watchdog in `_updateStopAtLine` and only ever latches `LINE_NOT_FOUND`; a SYNC `driveDist` cannot be faulted via mock observables). Resolution (`--auto`): first **probe** whether a purely additive `path_mocks` Drive setter can inject a generic Drive fault into `RETURN_TO_NODE` *without behavioral FSM modeling*. If yes → cover both branches genuinely through the real FSM. If no (expected) → drop both and document precisely why, citing ADR-030's "intentional divergence / un-mockable dependency" trigger. No tautological `ASSERT_EQ(X, X)` placeholder.
- **Split reset_arrival strict OFF/ON** (the `test_reanchor_at_wall[_strict]` precedent): strict-OFF target drives advisory-timeout + clean-success; a new strict-ON sibling drives real `WALL_NOT_SQUARE`. Genuine end-to-end de-mirror of AC11.
- **New helpers live in the shared harness** (`test_path_tracker_harness.h`), additive only, reusable by future tests — re-run the full ctest because the header is consumed by 7 passing targets.
- **Manufacture the operator offset via the real integrator** (simulate a hand-push: nav inactive + advance encoders + tick `Localization`), not a production test hook. Most faithful to runtime; no prod change.
- **Assumptions accepted** (user-confirmed scope; assumptions batch acknowledged, taken as best-judgment): inbound reset hop = `goTo(Wall1)` from `position1`; RETURN cases isolated via `goTo(<same anchor>)`; keep TC4/TC4c real-symbol vectors; ADR-030 is completed (not a new ADR); done = full host ctest + arduino-cli compile green locally.

## Acceptance Criteria

- [ ] `test_path_tracker_reset_arrival.cpp` contains NO `// MIRROR:` annotation, NO `enum class RState`, NO `struct ResetArrivalMirror`; it drives the real `PathTracker` FSM and `grep -n "MIRROR" ` over the file returns nothing.
- [ ] `test_return_to_node_dispatch.cpp` contains NO `// MIRROR:` annotation and NO `replica_returnTick` / `ReturnTickResult` / `ReturnTickOutcome`; the TC4 / TC4c direct real-symbol guards remain.
- [ ] A new `test_path_tracker_reset_arrival_strict.cpp` drives the real FSM to `PathTracker.getFault().kind == PathTrackerClass::Fault::WALL_NOT_SQUARE` via real `reanchorAtWall` rejecting under strict + off-normal yaw, asserts no depart drive was issued and the reset latch re-arms.
- [ ] The reset-arrival strict-OFF test asserts, against the REAL FSM + REAL `Localization`: arrival at `Wall1` enters `WALL_STOP`; the wallStop advisory timeout is surfaced and is NOT a fault (`!PathTracker.hasFault()`); REANCHOR + WALL_DEPART + clearOffset land the believed pose on the node coordinate (y == 139, not the wall 154 nor the axle); a clean stall-success arrival surfaces NO advisory; the one-shot reset latch holds through `WALL_DEPART`.
- [ ] `test_return_to_node_dispatch.cpp` drives the real FSM for: LINE-seek-return success (`onIntersection` re-anchor onto the LINE anchor, no fault); LINE-seek-return no-line under the shipped FAULT gate (`Fault::OFF_NODE`, no re-anchor); short-LINE return below the axle lead routes through the dead-reckon branch (`clearOffset` success, NOT a seek); BLIND dead-reckon return success (`clearOffset`). Each pins its routing with a `routedFirstHop` / node-kind guard.
- [ ] The two synthetic-only branches are either covered through the real FSM (if the probe finds an additive injection) or deleted with a precise documented reason in the test header.
- [ ] `expectWallStop(...)` and `pushOperatorOffset(...)` exist in `test_path_tracker_harness.h`, are additive, and the five existing #106 tests still pass unchanged.
- [ ] All three targets are wired on the real-link recipe in `CMakeLists.txt`: link real `PathTracker.cpp`/`Drive.cpp`/`Localization.cpp`/`YawSource.cpp` + `routes_fixture.cpp` + `path_mocks.cpp`; include order `${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR}`; `add_fixture_routes_force_include` FIRST then `add_path_force_includes`; NO `add_mock_force_includes`; NO `MOCK_SOURCES`; NO `${MOCKS_DIR}` on the include path.
- [ ] ADR-030 status no longer lists the two slices as a remaining exception; the DECISIONS index reflects it.
- [ ] `cmake --build` of the whole host harness succeeds, `ctest --output-on-failure` is fully green (all targets), and `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` succeeds (the only local catcher of ODR/link regressions in the shared harness).

## Contracts & Interfaces

Single source of truth for the cross-task seams. The two new harness helpers are the seam shared by the three rewritten test bodies; the CMake recipe is the seam shared by the three targets.

### Shared types / symbols (already exist, used as-is)

- Real globals (linked, NOT mocked): `PathTracker`, `Drive`, `Localization`, `YawSource`, `FieldMap`.
- Mocked surfaces: `MiniR4` facade + `mmL` (`hal_mocks/MatrixMiniR4.h`), `LineArray`/`Intersection`/`LineFollower` (`path_mocks/`), `EventTrace` (`hal_mocks/EventTrace.h`).
- Fault enum: `PathTrackerClass::Fault::{NONE, WALL_NOT_SQUARE, OFF_NODE, DRIVE_FAULT}`. Drive fault: `DriveClass::Fault::LINE_NOT_FOUND`.
- Reset record: `Routes::ResetNode { axis, wall_coord, facing }` via `FieldMap::getResetInfo(node)`; `Wall1` = `{axis=y(1), wall_coord=154, facing=270}` at node y=139.

### New harness helper signatures (owner: T1; consumers: T3, T4, T5)

```cpp
/// Retire an in-flight Drive.wallStop push to a clean stall-success or an advisory timeout.
/// Settles the wallStop INTERNAL SQUARE turn first (parks mmL.mockYawDeg at the wall facing
/// each tick, the CONTRACT-2 idiom) so a later clock advance cannot trip the SQUARE turn's
/// IMU stall watchdog; THEN drives the PUSH phase:
///   timedOut==false -> step M2 once (reset the progress baseline), hold it, advance the clock
///                      past the dispatched stall window so _updateWallStop confirms the stall
///                      (back-off is 0 here -> straight to DONE).
///   timedOut==true  -> hold M2 frozen, advance the clock past the dispatched wallStop timeout
///                      so _wallStopTimedOut latches (a NON-fault advisory).
/// Bounded by its own generous tick/clock budget (NOT kMaxHandshakeTicks, which is too small for
/// the multi-second wall timeouts). Reads the DISPATCHED timeout, never a hardcoded 3000.
/// Asserts: Drive.isIdle() && Drive.wallStopTimedOut()==timedOut && !Drive.hasFault().
inline void expectWallStop(int16_t wallFacingAppDeg, bool timedOut);

/// Manufacture a REAL operator offset of magnitude >= targetCm along app heading `alongAppHeadingDeg`
/// by simulating an operator hand-push: ensures the FSM is IDLE (nav inactive -> integrator gate open),
/// parks mmL.mockYawDeg at hwYaw(alongAppHeadingDeg), then runs the integrator's TWO-PHASE reopen edge
/// (first open tick re-baselines and deposits NOTHING; subsequent ticks deposit) — advancing M2/M3 past
/// Config::OFFSET_TRANSLATION_DEADBAND_CM each step and ticking Localization until
/// getOffsetFromNode() magnitude >= targetCm. ASSERTS the achieved magnitude (a zero-accumulation
/// regression fails HERE, loudly, instead of silently degrading the return case to ReturnAction::NONE).
/// Returns the achieved magnitude. Call AFTER seatAt(anchor, alongAppHeadingDeg) and BEFORE goTo(...).
inline float pushOperatorOffset(float targetCm, int16_t alongAppHeadingDeg = 0);
```

### CMake recipe seam (owner: T2; consumers: the three targets)

Identical to the five #106 `add_executable` blocks. Per target: sources = test `.cpp` + `PathTracker.cpp` + `Drive.cpp` + `Localization.cpp` + `YawSource.cpp` + `${FIXTURE_ROUTES_DIR}/routes_fixture.cpp` + `${PATH_MOCKS_DIR}/path_mocks.cpp`; `target_include_directories(... ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})`; `add_fixture_routes_force_include(<t>)` then `add_path_force_includes(<t>)`; the strict target adds `target_compile_definitions(<t> PRIVATE WALL_NORMAL_STRICT_ENABLED_DEFAULT=true)`. Keep each existing `add_test(NAME ...)` name (CI greps them) and add `path_tracker_reset_arrival_strict`.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/test/test_path_tracker_harness.h` | T1 | T3, T4, T5 (include only) |
| `WRO2026/test/test_path_tracker_reset_arrival.cpp` | T3 | — |
| `WRO2026/test/test_path_tracker_reset_arrival_strict.cpp` (new) | T4 | — |
| `WRO2026/test/test_return_to_node_dispatch.cpp` | T5 | — |
| `WRO2026/test/CMakeLists.txt` | T2 | — |
| `docs/decisions/030-...md`, `DECISIONS.md` | T7 | — |

### Naming (exact strings that must match across tasks)

- Node names (resolve BY NAME): `Wall1` (RESET), `position1` (BLIND, inbound origin), `Inter1` (LINE anchor), `Start` (BLIND anchor). CTest target names: `path_tracker_reset_arrival`, `path_tracker_reset_arrival_strict`, `return_to_node_dispatch`.
- Final-line success markers (a test's `main` prints on all-pass): keep each file's existing marker style (`PATH TRACKER RESET ARRIVAL OK` / `RETURN TO NODE DISPATCH OK`); the strict file uses its own (`PATH TRACKER RESET ARRIVAL STRICT OK`).

## Data / FSM facts the implementer must rely on (verify against code, do not re-derive)

- **RESET interlude fires only after a real hop:** `_onPlanNextHop` guards `if (!firstPlan && _maybeEnterReset(cur))`; `_planIsFirstAfterDequeue` is set at DEQUEUE and cleared on the first plan. So `goTo(Wall1)` from `position1` drives one BLIND hop (`onDeadReckonedAdvance(Wall1)`), and the SECOND `_onPlanNextHop` (firstPlan=false) fires the interlude. `position1→Wall1` is a direct BLIND fixture edge (~45.45 cm, heading ~140.36°). Confirm `routedFirstHop(position1, Wall1)==Wall1` and `getEdge(position1,Wall1).type==BLIND` as a routing guard, and that the inbound hop resolves to a dead-reckon advance (NOT a seek — a seek would re-anchor via `onIntersection` and change the believed-pose math).
- **Two turns to settle per reset case:** the inbound hop's pre-edge turn to ~140° (park `mmL.mockYawDeg` at the edge heading each tick, the `driveToSeekInFlight` idiom), THEN the wallStop INTERNAL SQUARE turn to `facing`=270° inside `expectWallStop`.
- **wallStop retire timing:** `_updateWallStop` PUSH has a stall path (`stopAtLineStalled` over `STOP_AT_LINE_STALL_TIMEOUT_MS = 1500`, a SUCCESS here) and an advisory path (`now - _wallStartMs >= _wallTimeoutMs`, default `WALL_STOP_TIMEOUT_MS = 3000`, NON-fault). A static_assert guarantees `WALL_STOP_TIMEOUT_MS > STOP_AT_LINE_STALL_TIMEOUT_MS`, so the two retire paths do not race — but `expectWallStop` MUST read the dispatched timeout (the PathTracker dispatch issues no `.setTimeout`, so it inherits the builder default; verify that default is `WALL_STOP_TIMEOUT_MS`). The wallStop advisory timeout does NOT set `Drive.hasFault()`.
- **Offset integrator gate:** `Localization.update()` integrates only when `!_navActive && _zeroed` AND the gate was already open last tick (`_gateWasOpen`). The FIRST open tick re-baselines and deposits nothing; deposits start the SECOND open tick. The deposit is gated by `Config::OFFSET_TRANSLATION_DEADBAND_CM = 0.2` and decomposed along the live heading — so park `mmL.mockYawDeg` at the seated heading and advance encoders comfortably past the deadband. (Memory: [[gotcha-localization-rezero-reopen-edge]].)
- **RETURN prologue trigger:** at DEQUEUE, `_evaluateReturnGate(off, mag)` fires when `mag > RETURN_TRIGGER_THRESHOLD_CM = 3.0` (cap `RETURN_DISTANCE_CAP_CM = 50.0`). `_returnUsesLineSeek = nodeIsLine && (returnDist >= LINE_SENSOR_AXLE_OFFSET_CM = 12.02)`. So: LINE anchor + push ≥ 12.02 → seek; LINE anchor + 3 < push < 12.02 → dead-reckon (short-LINE #14); BLIND anchor → dead-reckon. `goTo(<same anchor>)` runs the prologue at DEQUEUE, then completes (cur==dest) right after the re-anchor.
- **Strict reject path:** with `WALL_NORMAL_STRICT_ENABLED` ON, park `mmL.mockYawDeg` off-normal (beyond `WALL_NORMAL_TOLERANCE_DEG = 10°` from `facing`) just before the REANCHOR tick so real `reanchorAtWall` returns false → real FSM `_enterFault(WALL_NOT_SQUARE)`. (The boolean provenance is independently proven by `test_reanchor_at_wall_strict`.)

## Error Handling

- **Inbound hop never reaches Wall1 / wrong resolution:** the routing guard (`routedFirstHop` + edge-type assert) fails LOUDLY at the top of the test rather than the case silently testing a different path.
- **Zero-offset accumulation:** `pushOperatorOffset` asserts the achieved magnitude — a gate-edge regression fails inside the helper, not as a misleading green RETURN no-op.
- **wallStop retire stuck:** `expectWallStop` bounds its loop with a generous budget and asserts `Drive.isIdle()` after — a real-FSM regression fails as a budget-exhausted assert, never an infinite hang.
- **Cross-case fault leak:** every case starts with the harness `resetWorld()` (clears both Drive + PathTracker faults), so a prior LINE_NOT_FOUND cannot stall the next case's first `goTo`.
- **ODR / include-order regression:** caught by the `arduino-cli` compile (T8) and a clean `cmake` configure+build; the spec pins the exact include order and force-include order to avoid the `DRIVE_H` mock-vs-real collision.

## Testing Strategy

**Levels:** Host C++ ctest (the deliverable itself), plus the arduino-cli sketch compile as the ODR/link gate. No new Python.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC-RA-route | `routedFirstHop(position1,Wall1)==Wall1` over a BLIND edge into a RESET node | Integration | Routing guard passes; inbound hop is a dead-reckon advance |
| TC-RA1 | Reset arrival, advisory-timeout (`expectWallStop(270,true)`): REANCHOR still runs, advisory surfaced, NOT a fault, WALL_DEPART + clearOffset → believed y==139 | Integration | Real FSM reaches PLAN_NEXT_HOP; `!PathTracker.hasFault()`; believed pose on node coord |
| TC-RA2 | Reset arrival, clean stall-success (`expectWallStop(270,false)`): no advisory surfaced; re-anchor + depart + clearOffset | Integration | Believed pose lands on node coord; no fault |
| TC-RA3 | One-shot latch: after a full success through WALL_DEPART, a re-plan does NOT re-enter the interlude | Integration | Interlude fires exactly once per arrival |
| TC-RA-strict | (strict-ON target) off-normal yaw at REANCHOR → real `reanchorAtWall` false → `Fault::WALL_NOT_SQUARE`, no depart, latch re-armed | Integration | `PathTracker.getFault().kind == WALL_NOT_SQUARE` |
| TC-RN4 | (kept) real `FieldMap::getNodeType` discriminates LINE (`Inter1`) vs BLIND (`Start`) over the fixture | Unit | LINE anchor → seek path; BLIND → dead-reckon (real symbol, not a mirror) |
| TC-RN4c | (kept) short LINE return below `LINE_SENSOR_AXLE_OFFSET_CM` routes through dead-reckon, driven on the REAL FSM | Integration | `clearOffset` success, NOT a seek; long LINE return uses the seek |
| TC-RN-line-ok | LINE anchor, push ≥ axle lead, `goTo(Inter1)` → real seek → crossing → `onIntersection` re-anchor | Integration | Re-anchored on the LINE anchor; no fault |
| TC-RN-line-noline | LINE anchor, push ≥ axle lead, seek no-line under shipped FAULT gate → `Fault::OFF_NODE` | Integration | `OFF_NODE`, no re-anchor, no DRIVE_FAULT |
| TC-RN-blind-ok | BLIND anchor (`Start`), push offset, `goTo(Start)` → dead-reckon return → `clearOffset` success | Integration | Re-anchored via clearOffset; no fault |
| TC-RN-deadbranch | (probe outcome) non-LINE_NOT_FOUND DRIVE_FAULT + BLIND drive-failure: covered through real FSM IF an additive mock injection exists, else deleted with documented reason | Unit/doc | No tautological assert; honest coverage or honest deletion |
| TC-suite | Full `ctest` (all targets incl. the five #106 tests) + `arduino-cli compile` | Integration | All green; shared-harness edit broke nothing |

**Test data:** the frozen `fixtures/routes_fixture.cpp` graph (nodes by name); mock encoders (`MiniR4.M2/M3.mockSetDegrees/mockAdvanceDegrees`), mock clock (`mockAdvanceMillis`), mock IMU (`mmL.mockYawDeg`).
**Run command:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`, then `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`. (Local MSVC via the pre-configured `build/` VS-generator tree per [[gotcha-host-build-vs-generator-not-vcvars-cmd]].)

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Add `expectWallStop` + `pushOperatorOffset` to the shared harness | — | xHigh | `WRO2026/test/test_path_tracker_harness.h` | Implement the two helpers EXACTLY per the Contracts signatures. `expectWallStop`: settle the wallStop internal SQUARE turn first (park `mmL.mockYawDeg` at `wallFacingAppDeg` each tick until PUSH begins), then retire PUSH via stall-success (step M2 once, hold, advance clock past the dispatched stall window) or advisory-timeout (hold M2, advance clock past the DISPATCHED wallStop timeout); own budget loop; assert `isIdle() && wallStopTimedOut()==timedOut && !hasFault()`. `pushOperatorOffset`: two-phase reopen-edge integrator drive (FSM IDLE → nav inactive, park yaw, tick once to re-baseline, then advance M2/M3 past `OFFSET_TRANSLATION_DEADBAND_CM` + tick until magnitude ≥ target), ASSERT the achieved magnitude, return it. Doxygen on both. Additive only — must not alter existing helpers. Satisfies the seam for T3/T4/T5. |
| T3 | Rewrite reset_arrival (strict-OFF) on the real FSM | T1 | high | `WRO2026/test/test_path_tracker_reset_arrival.cpp` | Delete `RState`/`ResetArrivalMirror`/`resetWorldOnResetNode` + all `// MIRROR:`. Use `seatAt("position1", ...)`, `goTo(nodeByName("Wall1")).async()`, park the inbound edge heading each tick to settle the pre-turn, `expectBlindHop` the inbound hop, then a routing guard (`routedFirstHop`/edge-type). Drive the real interlude: TC-RA-route, TC-RA1 (advisory `expectWallStop(270,true)`), TC-RA2 (clean `expectWallStop(270,false)`), TC-RA3 (one-shot latch). Keep the believed-pose post-conditions against real `Localization` (y==139). Satisfies AC: reset advisory/success/latch. |
| T4 | New strict-ON reset_arrival sibling (WALL_NOT_SQUARE) | T1 | high | `WRO2026/test/test_path_tracker_reset_arrival_strict.cpp` (new) | Same inbound-hop + wallStop drive as T3, compiled with `WALL_NORMAL_STRICT_ENABLED` ON (via the target compile-def). Before the REANCHOR tick, park `mmL.mockYawDeg` off-normal (> `WALL_NORMAL_TOLERANCE_DEG` from `facing`=270) so real `reanchorAtWall` rejects → assert `PathTracker.getFault().kind == Fault::WALL_NOT_SQUARE`, no depart drive issued, reset latch re-armed. Header docstring explains the strict split (mirrors `test_reanchor_at_wall_strict`). Satisfies AC: TC-RA-strict. |
| T5 | Rewrite return_to_node on the real FSM + probe the dead branches | T1 | high | `WRO2026/test/test_return_to_node_dispatch.cpp` | FIRST: probe whether `path_mocks` can inject a generic Drive fault into RETURN_TO_NODE via a purely additive setter (no behavioral modeling). Then delete `replica_returnTick`/`ReturnTickResult`/`ReturnTickOutcome` + all `// MIRROR:`. Keep TC4 (real `getNodeType`) + TC4c distance-gate guards. Drive real-FSM cases: seat anchor → `pushOperatorOffset` → `goTo(<same anchor>)` → settle the return pre-turn → retire via `expectStopAtLine`/`expectBlindHop`: TC-RN-line-ok (`onIntersection`), TC-RN-line-noline (FAULT gate → `OFF_NODE`), TC-RN4c short-LINE → dead-reckon, TC-RN-blind-ok (`clearOffset`). Handle the two synthetic branches per the probe (cover or document-and-delete; NO tautological assert). Satisfies AC: return routing + dead-branch handling. |
| T2 | Re-wire the three CMake targets to the real-link recipe | T1, T3, T4, T5 | med | `WRO2026/test/CMakeLists.txt` | Replace the three current target blocks (`test_path_tracker_reset_arrival`, `test_return_to_node_dispatch`) with the #106 recipe and ADD `test_path_tracker_reset_arrival_strict`. Per the Contracts CMake seam: real sources + `routes_fixture.cpp` + `path_mocks.cpp`; `${SKETCH_DIR}`-first includes (drop `COMMON_INCLUDES`/`${MOCKS_DIR}`); `add_fixture_routes_force_include` FIRST then `add_path_force_includes`; DROP `add_mock_force_includes` + `MOCK_SOURCES`. Strict target adds `WALL_NORMAL_STRICT_ENABLED_DEFAULT=true`. Keep/extend `add_test` names. Update the block comments (these are no longer mirror targets). Satisfies AC: recipe wiring. |
| T7 | Update ADR-030 + DECISIONS index | — | low | `docs/decisions/030-harness-links-real-sources-ci-enforced.md`, `DECISIONS.md` | Flip ADR-030 status to drop the "two narrowly-scoped FSM mirror slices remain / #113" caveat; rewrite the "Remaining FSM mirrors (follow-up)" bullet to state both slices are de-mirrored and zero mirrored FSM logic remains (#113 landed). Update the finding-#... trailer + any DECISIONS.md status-summary row that tracks ADR-030. Note any branch documented-as-divergence (the dead branches) under "Trigger to revisit" if T5 dropped them. No new ADR. Satisfies AC: ADR update. |
| T8 | Verify: full ctest + arduino-cli green | T1, T2, T3, T4, T5, T7 | low | (no source edit) | `cmake -S WRO2026/test -B build/test` (or drive the pre-configured VS `build/` tree), `cmake --build`, `ctest --output-on-failure` — ALL targets green (incl. the five #106 tests + the new strict target). Then `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`. Fix any regression in the owning task's file. Satisfies AC: suite green + ODR gate. |

## Notes for Implementer

- **The harness header is shared by 7 passing targets.** T1 is additive ONLY — do not touch `expectPreTurn`/`expectLineHop`/`expectBlindHop`/`expectStopAtLine`/`seatAt`/`resetWorld`. After T1, re-run the FULL ctest, not just the two rewritten targets ([[gotcha-strict-emitter-field-fixtures]]-style "only the full run finds them all").
- **Iterate against the local MSVC build** — the toolchain is available ([[host_cpp_toolchain_available.md]]); drive the pre-configured VS-generator `build/` tree with bundled cmake `--build ... --config Debug` ([[gotcha-host-build-vs-generator-not-vcvars-cmd]]). MSVC catches the inline-incomplete-type + facade-static-call traps g++/CI miss.
- **ODR trap (the #1 re-wiring risk):** these two targets currently link `MOCK_SOURCES` + `add_mock_force_includes` (mock `Drive.h`). The rewrite MUST drop both and omit `${MOCKS_DIR}` entirely — `mocks/Drive.h` and the real `Drive.h` share guard `DRIVE_H`, so any stray `${MOCKS_DIR}`/`COMMON_INCLUDES` re-introduces the mock and the real `Drive.cpp` fails to find its own private members (a confusing error blaming `PathTracker.cpp`). See [[gotcha-host-link-inversion-sketch-dir-first]]: SKETCH_DIR FIRST.
- **Park-yaw-every-tick or the drive loop hangs** ([[gotcha-pathtracker-host-test-preturn-park-yaw]]): the inbound hop's pre-turn AND the wallStop SQUARE turn both need `mmL.mockYawDeg` parked at the (hardware-frame) target each tick to settle; forget it and the loop spins to its budget assert.
- **`pushOperatorOffset` two-tick gate is load-bearing** ([[gotcha-localization-rezero-reopen-edge]]): the first open tick deposits nothing. The helper's magnitude self-assert is the guardrail that turns a silent zero-offset (→ false-green `ReturnAction::NONE`) into a hard failure.
- **Probe before documenting away** (T5): only the seek's non-LINE_NOT_FOUND discriminator is intrinsically host-unreachable; the BLIND-drive-failure branch is a mock-surface question. If covering it needs behavioral done-bit/fault FSM modeling, that violates the thin-mock rule ([[project-host-mock-thin-not-behavioral]]) — then drop it with that exact reason. Keep additive mock setters record+test-set only.
- **Routes fixture is frozen** ([[project-host-test-link-toy-routes]]): resolve every node BY NAME (`Wall1`/`position1`/`Inter1`/`Start`); never an index. Do not regenerate `routes_fixture.*` unless a `routes.h` schema change forces it.
- **Rollback:** the change is test + CMake + ADR only; reverting the four files + the CMakeLists block restores the prior (mirror) state. No production behavior is touched, so a regression cannot reach the robot.
- **Commit/PR:** feature branch (e.g. `feat/demirror-reset-return-fsm-tests`), match the repo's `test(...): ...` / `test(path|mission): ...` commit voice (see `git log -20 --oneline`); read `CONTRIBUTING.md` before opening the PR; reference #113 + epic #107.
