# PathTracker Sim↔Firmware FSM Parity Plan

**Goal:** Add a mechanical, CI-enforced check that proves the Python `PathTrackerSim` makes the same per-hop FSM decisions (and resulting pose) as the real C++ `PathTracker` firmware across the whole dispatch surface — not just the single `resolveStopAtLine` resolver that #63/#115 already cover — so the sim can no longer silently drift from the robot (issue #108).

**Approach:** Drive the **real** C++ `PathTracker` FSM through the existing #112 host harness over the feature-complete `routes_fixture`, emit a normalized semantic per-hop trace (decisions + commanded params) from FSM commit points, and diff it **live** against the Python `PathTrackerSim.step_through()` stream over the **same single-sourced** fixture. The single-hop dispatch decision is extracted from `PathTracker::_dispatchEdgeAfterTurn()` into a new header-only terminal classifier `EdgeDispatchPolicy.h` (mirrored in Python); cross-hop decisions (pass-through lookahead, pivot-latch carry, return-to-node, reset interlude) stay sequence-level in the FSM and are covered by the trace, not the resolver. Pose is integrated by a single shared model (`VirtualChassis`) fed by both command streams, so there is no cross-language float drift.

## Scope

- **In scope:**
  - A new header-only `EdgeDispatchPolicy.h` (terminal one-hop classifier) and the `PathTracker.cpp` refactor to dispatch through it; a Python mirror `resolve_edge_dispatch` and the `runtime.py` `_walk_hops` refactor to dispatch through it.
  - A `PT_PARITY_TRACE`-guarded emit hook in `PathTracker.cpp` (compiled out of firmware) that emits the normalized semantic trace records at FSM commit points.
  - A C++ host **parity-dump** target that drives the real FSM over the fixture (exhaustive all-pairs `goTo` + curated specials) and prints the trace as versioned NDJSON.
  - A Python **parity test** that invokes the dump binary, runs the sim over the same cases, normalizes both into the shared schema, integrates C++ pose via the single `VirtualChassis`, and diffs (decisions exact, pose tolerant), honoring a self-auditing divergence allowlist.
  - Single-sourcing the `routes_fixture` field into an importable module consumed by **both** the C++ regenerator and the Python parity test; a CI freshness gate (`git diff --exit-code`) on the regenerated fixture.
  - A standalone cross-language CSV truth-table parity test for `resolveEdgeDispatch` (defense-in-depth, mirroring the #115 `resolveStopAtLine` pattern).
  - A dedicated CI job that builds the C++ parity target + runs the parity pytest (both toolchains), uploading both traces as artifacts on failure.
  - A new ADR documenting the mechanism + `DECISIONS.md` index update.

- **Out of scope:**
  - **Fixing** the cited two-surface config hazard (`PIPELINE_*` resolving at two surfaces). This task is the **detector**; any drift it surfaces is filed as a follow-up issue, not fixed here. (Per user decision.)
  - Physical / timing fidelity. The sim is constant-velocity and does not model the fault FSM (ADR-033); those dimensions are structurally excluded from the trace schema, not allowlisted.
  - Any change to the production 2026 `field_data.json` / `routes.h` / `routes.cpp`. Parity runs on the fixture only.
  - Re-verifying the existing `resolveStopAtLine` / `resolveDirection` parity (kept as-is; the FSM trace exercises them in integration but does not replace their dedicated tests).
  - Pose comparison via two independent physics models. There is exactly one integrator.

## Decisions

- **Trace topology = live, dedicated CI job** — no committed C++ trace golden. A committed golden can launder a real C++ regression through a routine "golden update", which is the exact silent-drift failure mode #108 exists to catch. The dedicated job always runs (no skip-softening → no vacuous green) and uploads both traces as artifacts on failure for debugging.
- **Fixture = reuse + gate `routes_fixture`** — it is already feature-complete (4 projection nodes, 2 reset/wall nodes, pivots, collinear triples, BLIND+LINE). Single-source it from an importable `field_fixture` module through the real emit pipeline and promote it from frozen to CI-gated. Avoids authoring a duplicate feature-complete fixture. One-time cost: regenerate once and confirm all sibling host tests stay green (they resolve nodes by name).
- **`EdgeDispatchPolicy.h` = terminal one-hop classifier only** — inputs are already-resolved (`edge.type`, dest-node-type, resolved `Direction`, the modes, pass-through flags, pivot trims); output is the full dispatch descriptor. It *composes* the existing pure policies (`resolveStopAtLine`, `resolvePivotTrim`, `resolveSensorOvershootTrim`) but introduces no new cross-hop state. Direction resolution, pass-through lookahead, and the pivot 3-latch carry stay in the FSM. (Both consult models were emphatic; widening it would either duplicate the existing policies or smuggle FSM state into a "pure" function.)
- **Compare decisions + per-hop pose** — decision fields (enums, bools, node names, ordering) compared **exactly**; pose (`x,y,heading`) compared with a float/circular tolerance. Pose is corroborating signal (it independently catches a re-anchor-kind or node-coordinate mapping bug) layered on the primary decision-stream signal.
- **One pose integrator** — pose for *both* sides is produced by the Python `VirtualChassis`, fed by the real emitted commands + re-anchor kind. No cross-language libm drift to absorb.
- **Enumeration = hybrid** — exhaustive all-pairs reachable `goTo` over the fixture (deterministic, name-sorted) as the bulk sweep, plus a small pinned curated list guaranteeing the special features (reset interlude, pivot, both pass-through kinds, REAR) are hit. A coverage guard asserts every record-kind appears ≥1× across the sweep.
- **Self-auditing allowlist** — committed data file; each entry scoped to `(case_selector, event_kind, field_path)` with reason + tracking-issue ref. A mismatch must match exactly one entry; an entry that matches nothing (stale) **fails CI**. Never allowlist a whole case or trace.
- **No freshness gate on a parity golden** — there is no committed parity golden (live topology). The only freshness gate is on the single-sourced `routes_fixture` (regenerate + `git diff`).

## Acceptance Criteria

- [ ] AC1 — `WRO2026/EdgeDispatchPolicy.h` exists as a header-only terminal classifier; `PathTracker.cpp::_dispatchEdgeAfterTurn` dispatches through `resolveEdgeDispatch(...)`. Firmware behavior is unchanged: all existing host ctests pass and `arduino-cli compile` succeeds.
- [ ] AC2 — `tools/path_planner/sim/runtime.py` has `resolve_edge_dispatch` mirroring the C++ resolver field-for-field; `_walk_hops` dispatches through it. All existing pytest passes.
- [ ] AC3 — `routes_fixture.{h,cpp}` is regenerated from an importable `field_fixture` module via the real emit pipeline; **all** existing host tests that link `routes_fixture` still pass; a CI step regenerates the fixture and fails on an uncommitted diff.
- [ ] AC4 — A C++ host parity-dump target drives the real FSM over the fixture (all-pairs + curated) and emits a versioned NDJSON trace of decisions + commanded params, with a `schema_version` field. Building the firmware (`arduino-cli`) is unaffected (the trace hook compiles out).
- [ ] AC5 — A Python parity test invokes the dump binary, runs the sim over the same cases, and diffs: decision fields exact, pose (via the single `VirtualChassis`) within tolerance. The test asserts the swept case count is non-zero and matches between sides (no vacuous pass).
- [ ] AC6 — A deliberately injected divergence (e.g. flip one Python dispatch branch or one C++ commanded value) makes the parity test **FAIL** — demonstrated in the PR description, not committed.
- [ ] AC7 — The divergence allowlist is a committed data file with the known intentional divergences seeded; a stale (unmatched) entry fails the test.
- [ ] AC8 — Coverage guard: the sweep exercises every dispatch branch (line→line+overshoot, line→projection follow, blind dead-reckon, blind stop-at-line seek, line-line-line pass-through, line-proj-line pass-through, REAR/reverse variants, pivot hops, reset interlude, return-to-node) — asserted by record-kind presence.
- [ ] AC9 — `resolveEdgeDispatch` has a standalone cross-language CSV truth-table parity test (a pytest + a C++ host target over one generated CSV), mirroring #115.
- [ ] AC10 — A dedicated CI job builds the C++ parity target and runs the parity pytest; on failure it uploads both traces as artifacts. The existing `path-planner` and `host-tests` jobs are unchanged in responsibility.
- [ ] AC11 — `docs/decisions/050-*.md` documents the mechanism; `DECISIONS.md` index + status table updated.

## Data Model

### `EdgeDispatchPolicy.h` query + decision (C++; Python mirror is field-identical)

```cpp
// Already-resolved inputs — the classifier computes NO cross-hop state.
struct EdgeDispatchQuery
{
    Routes::EdgeType  edgeType;        // LINE | BLIND
    Routes::NodeType  destNodeType;    // LINE | PROJECTION (BLIND/RESET dests not edge-routed)
    Direction         direction;       // FRONT | REAR (already resolved; never AUTO)
    float             rawDistanceCm;   // edge.distance_cm
    Routes::LineMode  lineMode;        // resolved line mode for the edge
    StopAtLineResolution stopAtLine;   // result of resolveStopAtLine(...) for BLIND edges
    bool              projPassThrough; // set by FSM lookahead (LINE→proj→line collapse)
    float             departingTrimCm; // from the pivot 3-latch (FSM-owned)
    float             approachingTrimCm;
    float             axleOffsetCm;    // direction-aware LINE_SENSOR_AXLE_OFFSET[_REAR]_CM
    float             minDriveCm;      // PIVOT_TRIM_MIN_DRIVE_CM
};

enum class DispatchPrimitive : uint8_t
{
    LINE_FOLLOW_INTERSECTION,  // LINE→line: follow to junction, then overshoot
    PROJECTION_FOLLOW,         // LINE→projection: distance-gated follow
    STOP_AT_LINE_SEEK,         // BLIND + seek
    BLIND_DEAD_RECKON          // BLIND, no seek
};

enum class ReanchorKind : uint8_t { ON_INTERSECTION, ON_DEAD_RECKONED_ADVANCE };

struct EdgeDispatchDecision
{
    DispatchPrimitive primitive;
    bool              reverse;          // direction == REAR
    Routes::BlindMode driveMode;        // motion bundle for seek / dead-reckon
    float             commandedDistanceCm;  // post-trim commanded value (NOT displacement)
    float             overshootCm;          // for follow/seek overshoot channel
    ReanchorKind      reanchor;
    bool              brakeOnGate;          // false on projection pass-through
};
```

> **Payload rule (memory-critical):** `commandedDistanceCm` is the **commanded** value (full `edge.distance_cm` for a sensor-terminated seek; trimmed distance for dead-reckon), never the physical displacement. The physical trim lives in the pose integration only.

### Normalized trace record (the cross-language wire schema)

NDJSON, one JSON object per line. The first line per case is a `goal_start`; the last is `goal_done`. Float fields are emitted by C++ as fixed 6-decimal strings (matching the emitter's float format) for stable text.

```
{ "schema_version": 1, "kind": "goal_start", "case_id": "<start>__<goal>", "start": "Inter1", "goal": "Wall1", "route_dir": "AUTO", "arrival_dir": "AUTO" }
{ "kind": "turn",              "body_heading_deg": 90.0, "wheel": "COUNTER_ROT" }
{ "kind": "line_follow",       "reverse": false, "line_mode": "NORMAL" }
{ "kind": "line_overshoot",    "advance_cm": 12.02, "pass_through": false, "reanchor": "ON_INTERSECTION" }
{ "kind": "projection_follow", "reverse": false, "line_mode": "NORMAL", "dist_cm": 40.0, "pass_through": false, "reanchor": "ON_DEAD_RECKONED_ADVANCE" }
{ "kind": "stop_at_line_seek", "reverse": false, "drive_mode": "SYNC_TSHAPE", "dist_cm": 60.0, "overshoot_cm": 12.02, "reanchor": "ON_INTERSECTION" }
{ "kind": "blind_dead_reckon", "reverse": false, "drive_mode": "GYRO_TSHAPE", "dist_cm": 150.0, "reanchor": "ON_DEAD_RECKONED_ADVANCE" }
{ "kind": "return_turn",       "body_heading_deg": 45.0, "wheel": "COUNTER_ROT" }
{ "kind": "return_move",       "mode": "seek", "node": "Inter1", "dist_cm": 8.0, "heading_world_deg": 90.0 }
{ "kind": "wall_stop",         "node": "Wall1", "axis_idx": 1, "wall_coord": 139.0 }
{ "kind": "wall_reanchor",     "node": "Wall1", "axis_idx": 1, "wall_coord": 139.0 }
{ "kind": "wall_depart",       "node": "Wall1", "axis_idx": 1, "node_coord": 110.0 }
{ "kind": "goal_done", "final": "Wall1" }
```

Pose is **not** on the wire — the Python side derives it for both streams via `VirtualChassis`.

### Divergence allowlist entry

```json
{
  "id": "fault-fsm-not-modeled",
  "case_selector": "*",          // glob over case_id; never "*" + "*" + "*" combined to mute everything
  "event_kind": "blind_dead_reckon",
  "field_path": "drive_mode",
  "side": "python",              // which side is expected to differ
  "reason": "sim does not model X; firmware does Y",
  "tracking_issue": "#NNN"
}
```

## Contracts & Interfaces

Single source of truth for every cross-task seam.

### Shared types

- `EdgeDispatchQuery` / `EdgeDispatchDecision` / `DispatchPrimitive` / `ReanchorKind` — defined once in `WRO2026/EdgeDispatchPolicy.h` (owner: T1); the Python mirror in `runtime.py` (owner: T2) must match field names + enum values byte-for-byte.
- The **normalized trace record schema** above is a **3-place contract** (echo of the CTPS `drive.method` contract): the C++ emitter (T6), the Python C++-stream loader (T8), and the Python sim normalizer (T8) must all agree on record `kind` strings and field names. A `kind`/field the emitter produces but the loader doesn't know must be a **loud failure**, never a silent skip.

### Signatures

- `EdgeDispatchDecision resolveEdgeDispatch(const EdgeDispatchQuery& q)` — owner: T1 (`EdgeDispatchPolicy.h`); consumers: `PathTracker.cpp` (T1), the standalone CSV test (T-CSV).
- `def resolve_edge_dispatch(q: EdgeDispatchQuery) -> EdgeDispatchDecision` — owner: T2 (`runtime.py`); consumers: `_walk_hops` (T2), the standalone pytest (T-CSV).
- `field_fixture.build_field_data() -> <field-data-shaped namespace>` — owner: T3 (`WRO2026/test/fixtures/field_fixture.py`); consumers: `regenerate_routes_fixture.py` (T3) and the parity pytest's graph build (T8).
- C++ macro `PT_PARITY_TRACE(record)` — no-op inline when undefined (firmware); emits one NDJSON line when the dump target defines `PT_PARITY_TRACE`. Owner: T1; consumer: T6.
- Python `parity` module (`tools/path_planner/sim/parity.py` or `tests/parity_support.py`) — owner: T5 — exposes: `TraceRecord` schema, `normalize_sim_stream(sim, case) -> list[TraceRecord]`, `load_cpp_ndjson(text) -> dict[case_id, list[TraceRecord]]`, `integrate_pose(records, field) -> list[Pose]`, `load_allowlist(path)`. Consumers: T7 (allowlist data), T8 (the test).

### Enumeration contract

- The case set = **all reachable `(start, goal)` node pairs** over the fixture graph, sorted by `(start_name, goal_name)`, **plus** the curated pins in `CURATED_PARITY_CASES` (a shared constant: e.g. a known reset hop, a pivot hop, a line-proj-line pass-through, a REAR hop). Both the C++ dump (T6) and the Python test (T8) must enumerate identically; `case_id = f"{start}__{goal}"` is the join key.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/EdgeDispatchPolicy.h` (new) | T1 | T2 (mirror reference), T6, T-CSV |
| `WRO2026/PathTracker.cpp`, `PathTracker.h` | T1 | — |
| `tools/path_planner/sim/runtime.py` | T2 | T5, T8 |
| `WRO2026/test/fixtures/field_fixture.py` (new) | T3 | T8 |
| `WRO2026/test/fixtures/regenerate_routes_fixture.py` | T3 | — |
| `WRO2026/test/fixtures/routes_fixture.{h,cpp}` (regen) | T3 | T6 |
| `tools/path_planner/sim/parity.py` (new) | T5 | T7, T8 |
| `WRO2026/test/test_path_tracker_parity_dump.cpp` (new) | T6 | — |
| `WRO2026/test/CMakeLists.txt` | T6 | T-CSV (adds its target too — see note) |
| `tools/path_planner/tests/fixtures/parity_allowlist.json` (new) | T7 | T8 |
| `tools/path_planner/tests/test_pathtracker_parity.py` (new) | T8 | — |
| `WRO2026/EdgeDispatchPolicy` CSV + `gen_*.py` + both CSV tests | T-CSV | — |
| `.github/workflows/ci.yml` | T-CI | — |
| `docs/decisions/050-*.md`, `DECISIONS.md` | T-ADR | — |

> **CMakeLists conflict note:** T6 and T-CSV both add a target to `WRO2026/test/CMakeLists.txt`. To avoid a parallel-edit conflict, **T-CSV is blocked by T6** (T6 lands the CMake edits first; T-CSV appends its target after).

### Naming (must match across tasks)

- Record `kind` strings: `goal_start`, `turn`, `line_follow`, `line_overshoot`, `projection_follow`, `stop_at_line_seek`, `blind_dead_reckon`, `return_turn`, `return_move`, `wall_stop`, `wall_reanchor`, `wall_depart`, `goal_done`.
- Enum string forms reuse the existing `resolveStopAtLine` CSV token spellings (`SYNC_TSHAPE`, `GYRO_TSHAPE`, `RAW_FLAT`, `NORMAL`, `FRONT`, `REAR`, `COUNTER_ROT`, `PIVOT_LEFT`, `PIVOT_RIGHT`) for consistency.
- CI job id: `parity`. Compile definitions: `PT_PARITY_TRACE`, `LINE_BOARD_COUNT=2` (so REAR is reachable), schema constant `PARITY_SCHEMA_VERSION=1`.

## Error Handling

- **Unknown record `kind`/field on load (T8):** raise loudly (no silent skip) — the schema is a 3-place contract; a desync must fail, mirroring the CTPS whitelist lesson.
- **C++ dump binary not found (T8):** the dedicated `parity` CI job builds it, so absence is a hard error there; a *local* run with no binary skips with a clear message (never a silent pass in CI — the job always builds it).
- **Case-count mismatch between sides (T8):** hard fail (a side that enumerated fewer cases is a bug or a desynced fixture).
- **Stale allowlist entry (T8):** an entry matching zero mismatches fails the test.
- **Fixture regen drift (T-CI):** `git diff --exit-code` on `routes_fixture.{h,cpp}` after regen fails the build (the freshness gate).
- **Firmware build:** `PT_PARITY_TRACE` is undefined for the Arduino target → the hook compiles to nothing; `arduino-cli compile` must stay green (regression guard in CI).

## Testing Strategy

**Levels:** Unit (resolver CSV parity), Integration (the FSM trace-diff), CI wiring.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `resolveEdgeDispatch` over the generated truth-table CSV (C++ + Python) | Unit | Both resolvers match the CSV row-for-row; row-count guard rejects a truncated CSV |
| TC2 | All-pairs + curated sweep, decision-stream diff | Integration | Every case's normalized decision records are byte-identical between C++ FSM and Python sim (modulo allowlist) |
| TC3 | Per-hop pose diff via single `VirtualChassis` | Integration | Pose within `1e-3 cm` position / `1e-4°` heading (circular) for every hop |
| TC4 | Injected divergence (flip one Python branch) | Integration | Parity test FAILS at the exact case + record + field |
| TC5 | Injected C++ divergence (perturb one commanded value) | Integration | Parity test FAILS; the diff localizes the case + field |
| TC6 | Coverage guard | Integration | Every record-kind appears ≥1× across the sweep; missing kind fails |
| TC7 | Stale allowlist entry | Unit | An entry matching nothing fails the test |
| TC8 | Fixture freshness | CI | Regenerating `routes_fixture` produces no diff; a hand-edit fails the gate |
| TC9 | Existing host tests over regenerated fixture | Regression | All sibling `routes_fixture`-linked ctests pass post-regen |
| TC10 | Firmware compile with hook compiled out | Regression | `arduino-cli compile` succeeds; binary unchanged in behavior |

**Test data:** the single-sourced `field_fixture` graph; the generated `edge_dispatch_truth_table.csv`; the seeded `parity_allowlist.json`.
**Run commands:** `uv run --extra dev pytest tools/path_planner/tests/` (incl. parity, which builds/locates the dump binary); `ctest --test-dir build/test`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Extract `EdgeDispatchPolicy.h` + refactor FSM + trace hook | — | high | `WRO2026/EdgeDispatchPolicy.h` (new), `WRO2026/PathTracker.cpp`, `WRO2026/PathTracker.h` | Extract the terminal one-hop classifier per the Data Model (composes `resolveStopAtLine`/`resolvePivotTrim`/`resolveSensorOvershootTrim`; takes resolved inputs). Refactor `_dispatchEdgeAfterTurn` to call `resolveEdgeDispatch` and act on the decision. Add `PT_PARITY_TRACE(record)` no-op-when-undefined macro + guarded emit calls at FSM commit points (turn, line_follow, line_overshoot, projection_follow, stop_at_line_seek, blind_dead_reckon, return_*, wall_*). Must NOT change firmware behavior. Satisfies AC1, AC4 (hook). Verify: existing ctests + `arduino-cli compile`. |
| T2 | Python mirror `resolve_edge_dispatch` + dispatch refactor | — | high | `tools/path_planner/sim/runtime.py` | Add `resolve_edge_dispatch` field-identical to T1's resolver; refactor `_walk_hops` to dispatch through it (emitted ACT_* tuples unchanged). Satisfies AC2. Verify: existing pytest. (Parallel with T1; both conform to the Contracts struct.) |
| T3 | Single-source the fixture | — | high | `WRO2026/test/fixtures/field_fixture.py` (new), `.../regenerate_routes_fixture.py`, `.../routes_fixture.{h,cpp}` | Factor the inline field out of `regenerate_routes_fixture.py` into importable `field_fixture.build_field_data()`. Wire the regenerator to import it. Regenerate `routes_fixture.{h,cpp}` via the real emit pipeline. Confirm ALL sibling `routes_fixture`-linked host tests stay green (resolve by name). Satisfies AC3 (artifact), TC9. |
| T5 | Parity support module (schema + normalizer + pose + loaders) | T2 | high | `tools/path_planner/sim/parity.py` (new) | Implement `TraceRecord` schema, `normalize_sim_stream` (maps ACT_* tuples → records incl. front/reverse collapse + reanchor kind), `load_cpp_ndjson` (loud on unknown kind/field), `integrate_pose` (single `VirtualChassis`, snap-to-node on `ON_INTERSECTION`), `load_allowlist`. Implements the 3-place schema contract (Python side). |
| T6 | C++ parity-dump target + CMake | T1, T3 | med | `WRO2026/test/test_path_tracker_parity_dump.cpp` (new), `WRO2026/test/CMakeLists.txt` | New host target linking real `PathTracker`/`Drive`/`Localization`/`YawSource` + `routes_fixture.cpp` + `path_mocks`, using `test_path_tracker_harness.h`. Enumerate all-pairs + `CURATED_PARITY_CASES`, drive each `goTo`, emit NDJSON via `PT_PARITY_TRACE`. Compile defs: `PT_PARITY_TRACE`, `LINE_BOARD_COUNT=2`, `PARITY_SCHEMA_VERSION=1`. Follow the #112 force-include inversion (sketch dir first). Satisfies AC4. |
| T7 | Divergence allowlist data file | T5 | med | `tools/path_planner/tests/fixtures/parity_allowlist.json` (new) | Author the allowlist with the known intentional divergences seeded (fault FSM, constant-velocity timing, host-unreachable `LINE_NOT_FOUND` branch per ADR-030/the host-drive memory), each with reason + tracking issue. Conforms to the entry schema in the Data Model. Satisfies AC7 (data). |
| T8 | Python parity test (live diff) | T2, T5, T6, T7 | high | `tools/path_planner/tests/test_pathtracker_parity.py` (new) | Locate/invoke the dump binary (CTPS `_find_dump_exe`-style discovery), parse NDJSON, build the sim graph from `field_fixture`, run `step_through` per case, normalize both streams, integrate C++ pose, diff decisions (exact) + pose (tolerant), apply the allowlist (fail on stale entry), assert case-count parity + non-zero, run the coverage guard. Satisfies AC5–AC8, TC2–TC7. |
| T-CSV | Standalone `resolveEdgeDispatch` CSV parity | T1, T2, T6 | med | `tools/path_planner/tests/fixtures/edge_dispatch_truth_table.csv` + `gen_*.py` (new), `tools/path_planner/tests/test_runtime_sim.py` (add), `WRO2026/test/test_edge_dispatch_truth_table.cpp` (new), `WRO2026/test/CMakeLists.txt` (append) | Generate a categorical truth-table CSV for `resolveEdgeDispatch` (independent transcription, like `gen_stop_at_line_truth_table.py`); add a pytest + a C++ host target consuming it via `CsvTruthTable.h`. Satisfies AC9, TC1. Blocked by T6 to serialize the CMakeLists edit. |
| T-CI | Dedicated parity CI job + fixture freshness gate | T3, T6, T8 | med | `.github/workflows/ci.yml` | Add a `parity` job that sets up Python (uv) + C++ (cmake/g++), builds the parity-dump (and `test_edge_dispatch_truth_table`) target, runs the parity pytest, and uploads both traces as artifacts on failure. Add a fixture-freshness step (regenerate `routes_fixture` + `git diff --exit-code`). Single owner of `ci.yml` to avoid conflicts. Satisfies AC10, TC8. |
| T-ADR | ADR-050 + DECISIONS.md | T8 | low | `docs/decisions/050-pathtracker-sim-parity.md` (new), `DECISIONS.md` | Document the mechanism (real-FSM trace-diff, single-source gated fixture, semantic schema as a 3-place contract, single-integrator pose, self-auditing allowlist, `EdgeDispatchPolicy` as terminal classifier). Update the index + status table. If ADR-050 was taken by a parallel PR, renumber per the collision convention. Satisfies AC11. |

**Parallelism:** T1, T2, T3 start in parallel (distinct files, all conform to the Contracts). T5 follows T2. T6 follows T1+T3. T7 follows T5. T8 joins T2+T5+T6+T7. T-CSV follows T1+T2+T6. T-CI joins T3+T6+T8. T-ADR last.

## Notes for Implementer

- **Behavior preservation is the load-bearing constraint of T1.** The extraction must be a pure refactor — `_dispatchEdgeAfterTurn` produces the identical Drive/LineFollower calls before and after. The existing real-FSM host tests (`test_path_tracker_*`) are the guardrail; run the full ctest after T1.
- **`commandedDistanceCm` is the command, not the displacement** (memory: `gotcha-sim-primitive-payload-is-command-not-displacement`). Seek = full `edge.distance_cm`; dead-reckon = trimmed. Pose trim lives only in the integrator.
- **The chassis lands axle-on-node** (memory: `gotcha-sim-chassis-axle-on-node`) — do not re-add a per-junction offset in the pose integrator; `ON_INTERSECTION` snaps to the node's mapped coordinate.
- **The trace schema is a 3-place contract** (memory: `gotcha-ctps-drive-method-contract`) — emitter, loader, normalizer change together; unknown kind/field is a loud failure.
- **Front/reverse collapse in the normalizer:** the sim emits distinct ACT tags for FRONT vs REAR (`ACT_FOLLOW_INT` vs `ACT_LINE_REVERSE`, etc.); the normalizer collapses each pair to `{kind, reverse}` so it matches the C++ `{primitive, reverse}` emission.
- **Host-link inversion** (memory: `gotcha-host-link-inversion-sketch-dir-first`): the parity-dump target puts `SKETCH_DIR` first so the real `Drive.h`/`PathTracker.h` win; force-include only the peripheral `path_mocks`; link `path_mocks.cpp` (not `hal_mocks.cpp`). Reuse `add_path_force_includes` + `add_fixture_routes_force_include`.
- **Regenerated fixture must keep node names** the sibling tests resolve by (`Inter1`, `ProjL`, `Wall1`, `Spur`, ...). The single-source field is the same field already inline in the regenerator; factoring it out should be byte-stable. Run the full ctest after T3 and confirm zero behavioral change.
- **Pivot/reset/return/pass-through stay in the FSM** — do not pull them into `EdgeDispatchPolicy`. The trace covers them at the sequence level.
- **The dedicated CI job always builds the binary** — never gate the parity test behind "binary exists else skip" in CI (vacuous green). Local runs may skip with a message.
- **Allowlist discipline:** seed only structurally-excluded dimensions (fault FSM, timing) and the documented host-unreachable branches; never allowlist a whole case. The stale-entry guard is what keeps it honest.
- **Rollback:** every deliverable is additive except the T1 FSM refactor and the T3 fixture regen. If T1 misbehaves, revert the `_dispatchEdgeAfterTurn` change (the resolver header can stay, unused). If T3's regen churns unexpectedly, revert `routes_fixture.{h,cpp}` to the frozen copy and fall back to a separate parity fixture.
- Read `CONTRIBUTING.md` before opening the issue/PR.
