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
- **Fixture = reuse + gate `routes_fixture`, accepting the #91 reversal** — it is already feature-complete (4 projection nodes, 2 reset/wall nodes, pivots, collinear triples, BLIND+LINE). Single-source it from an importable `field_fixture` module through the real emit pipeline and promote it from frozen to CI-gated. **This deliberately reverses issue #91**, which froze `routes_fixture` and decoupled it from `field_data.json` so unrelated edits could not silently break it: `regenerate_routes_fixture.py` reads the real `Config.h` track width (`parse_track_cm_from_config()`) and runs `build_pivot_turns()`, so once gated a `Config.h` track-width edit will stale the committed fixture and fail the gate — which we accept as *desirable* (the fixture then always reflects real track geometry). The change is **not** byte-stable: `emit_routes(..., field_data_source_path=Path(__file__))` SHA-stamps the generator file, and moving the inline field into `field_fixture.py` changes that file's content and the import wiring. The new ADR (T-ADR) **supersedes the #91 freeze/decouple decision** and records the re-coupling as intentional. One-time cost: regenerate once and confirm all sibling host tests stay green (they resolve nodes by name). (User chose this over a separate duplicate fixture.)
- **`EdgeDispatchPolicy.h` = terminal one-hop classifier only** — inputs are already-resolved (`edge.type`, dest-node-type, resolved `Direction`, the modes, pass-through flags, pivot trims); output is the terminal one-hop dispatch **classification** (primitive kind, reverse, drive mode, post-trim commanded distance, overshoot, reanchor kind, `brakeOnGate`) — **not** a full descriptor. The resolver *composes* the existing pure policies (`resolveStopAtLine`, `resolvePivotTrim`, `resolveSensorOvershootTrim`) and owns the trim-clamp arithmetic, but introduces no new cross-hop state. Direction resolution, pass-through lookahead, the pivot 3-latch carry, the GYRO heading target, the velocity-profile start-speed/RAW_FLAT split, and the `PIVOT_TRIM_UNDERFLOW` fault all stay FSM-side. (Both consult models + the judge were emphatic; widening it would either duplicate the existing policies or smuggle FSM state into a "pure" function.)
- **Three comparison buckets (not two)** — every trace-record field maps to exactly one of: (1) **EXACT** — record kind, ordering, enums (`primitive`, `drive_mode`, `line_mode`, `wheel`), bools (`reverse`, `pass_through`, `brake_on_gate`), node names, reanchor kind; (2) **TOLERANT-NUMERIC** — all continuous commanded fields, dimension-split: distance/coord (cm) at `DIST_TOL=1e-4`, angular (deg) circular at `HEAD_TOL=1e-4`; (3) **POSE-TOLERANT** — `x,y` at `1e-3 cm` (looser, since pose is accumulated), `heading` circular at `HEAD_TOL`. The comparator parses C++'s fixed-6-decimal emission to a number before comparing (so `1.500000` vs `1.5` never reaches equality), and **asserts every field is bucketed** (an unassigned field fails). See the Comparison Policy under Data Model.
- **Pose is a cumulative cross-check, not an independent oracle** — pose for *both* sides is produced by the one shared `VirtualChassis`, fed by the real emitted commands + reanchor kind. Pose does **not** and cannot independently catch a reanchor-kind mismatch (already exact-compared) or a node-coordinate bug (a single-sourced shared input) — those are owned by the decision compare and the fixture's own validator. Pose's genuine non-redundant signal is on the **cumulative axis**: same-sign sub-`DIST_TOL` per-hop drift that accumulates across a path, and an in-band `body_heading_deg` divergence that amplifies via lever-arm into out-of-band position error. Both exist only because the continuous compare is tolerant (not exact) and SE(2) composition is non-linear.
- **Parity driver reads observed FSM state, never re-derives** — the generic `goTo(goal)` sweep drives the real FSM one tick at a time and feeds the observable the FSM is *actually* waiting on, dispatched off two new **read-only** `PathTracker` accessors (`state()`, `resolvedBodyHeadingDeg()`; the private `State` enum is promoted to public). It must never re-derive the hop's type/heading in the test — that would make the harness score its own prediction and silently pass while C++ and the sim disagree (the exact drift this check exists to catch; harness invariant "never re-implement an FSM decision").
- **Enumeration = hybrid** — exhaustive all-pairs reachable `goTo` over the fixture (deterministic, name-sorted) as the bulk sweep, plus a small pinned curated list guaranteeing the special features (reset interlude, pivot, both pass-through kinds, REAR) are hit. A coverage guard asserts every record-kind appears ≥1× across the sweep.
- **Self-auditing allowlist** — committed data file; each entry scoped to `(case_selector, event_kind, field_path)` with reason + tracking-issue ref. A mismatch must match exactly one entry; an entry that matches nothing (stale) **fails CI**. Never allowlist a whole case or trace.
- **No freshness gate on a parity golden** — there is no committed parity golden (live topology). The only freshness gate is on the single-sourced `routes_fixture` (regenerate + `git diff`).

## Acceptance Criteria

- [ ] AC1 — `WRO2026/EdgeDispatchPolicy.h` exists as a header-only terminal classifier; `PathTracker.cpp::_dispatchEdgeAfterTurn` dispatches through `resolveEdgeDispatch(...)`. Firmware behavior is unchanged: all existing host ctests pass and `arduino-cli compile` succeeds.
- [ ] AC2 — `tools/path_planner/sim/runtime.py` has `resolve_edge_dispatch` mirroring the C++ resolver field-for-field; `_walk_hops` dispatches through it. All existing pytest passes.
- [ ] AC3 — `routes_fixture.{h,cpp}` is regenerated from an importable `field_fixture` module via the real emit pipeline; **all** existing host tests that link `routes_fixture` still pass; a CI step regenerates the fixture and fails on an uncommitted diff. (This supersedes the #91 freeze and re-couples the fixture to `Config.h` track width — accepted; documented in the ADR.)
- [ ] AC4 — A C++ host parity-dump target drives the real FSM over the fixture (all-pairs + curated) and emits a versioned NDJSON trace of decisions + commanded params, with a `schema_version` field. Building the firmware (`arduino-cli`) is unaffected (the trace hook compiles out).
- [ ] AC5 — A Python parity test invokes the dump binary, runs the sim over the same cases, and diffs per the **three comparison buckets** (EXACT decision fields; TOLERANT-NUMERIC continuous fields split distance/angular; POSE-TOLERANT). It **asserts every record field maps to exactly one bucket** (an unassigned field fails) and asserts the swept case count is non-zero and matches between sides (no vacuous pass). The standing anti-vacuous guards are this case-count check, the AC8 coverage guard, and the AC7 stale-allowlist guard.
- [ ] AC6 — A deliberately injected divergence (flip one Python dispatch branch or one C++ commanded value) makes the parity test **FAIL** at the exact case/record/field. This is a **one-time manual demonstration in the PR description**, not a committed standing test (the standing guards are listed in AC5).
- [ ] AC7 — The divergence allowlist is a committed data file with the known intentional divergences seeded; a stale (unmatched) entry fails the test.
- [ ] AC8 — Coverage guard: the sweep exercises every dispatch branch (line→line+overshoot, line→projection follow, blind dead-reckon, blind stop-at-line seek, line-line-line pass-through, line-proj-line pass-through, REAR/reverse variants, pivot hops, reset interlude, return-to-node) — asserted by record-kind presence. Return-to-node coverage requires the pinned-offset curated cases (see the Enumeration contract); without them the guard fails loudly rather than passing vacuously.
- [ ] AC9 — `resolveEdgeDispatch` has a standalone cross-language CSV truth-table parity test (a pytest + a C++ host target over one generated CSV), mirroring #115.
- [ ] AC10 — A dedicated CI job builds the C++ parity target and runs the parity pytest; on failure it uploads both traces as artifacts. The existing `path-planner` and `host-tests` jobs are unchanged in responsibility.
- [ ] AC11 — `docs/decisions/050-*.md` documents the mechanism and **explicitly supersedes the issue-#91 freeze/decouple decision** (recording the `routes_fixture`↔`Config.h` re-coupling as intentional); `DECISIONS.md` index + status table updated.
- [ ] AC12 — `PathTracker` exposes read-only `state()` + `resolvedBodyHeadingDeg()` (the `State` enum promoted to public), used by the T6 driver and asserted against the sim's per-hop body heading; no setter/mutation added; `arduino-cli compile` unaffected.

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
>
> **Trim-clamp arithmetic (frozen contract — both languages implement this identically):** for dead-reckon/projection hops the resolver computes `commandedDistanceCm = rawDistanceCm − departingTrimCm − approachingTrimCm`, then clamps to `minDriveCm`; the struct carries the **post-clamp** value. `PIVOT_TRIM_UNDERFLOW` is **not** a decision field — consistent with the fault-FSM structural exclusion, the resolver clamps and the FSM raises the fault when `commandedDistanceCm == minDriveCm`. The post-clamp distance is the value the CSV truth-table (T-CSV) compares row-for-row, so T1 and T2 cannot drift on the clamp.

### Comparison Policy (the three buckets)

Every field of every trace record maps to **exactly one** bucket; the comparator asserts this and fails on an unassigned field (so a field added later cannot silently escape comparison — any new `*_cm`/`*_coord` defaults to `DIST_TOL`, any new `*_deg`/heading to `HEAD_TOL`). Numeric fields are parsed to numbers before comparison (C++ emits fixed 6-decimal; Python carries full floats — never compare as strings).

| Bucket | Fields | Comparison |
|--------|--------|-----------|
| EXACT | record `kind`; sequence ordering; enums `primitive`, `drive_mode`, `line_mode`, `wheel`; bools `reverse`, `pass_through`, `brake_on_gate`; node names; `reanchor` | string/enum equality, zero tolerance |
| TOLERANT-NUMERIC (distance) | `advance_cm`, `dist_cm`, `overshoot_cm`, `wall_coord`, `node_coord` | abs `DIST_TOL = 1e-4` cm |
| TOLERANT-NUMERIC (angular) | `body_heading_deg`, `heading_world_deg` | circular (wrap-aware) `HEAD_TOL = 1e-4` deg |
| POSE-TOLERANT | `pose.x`, `pose.y` | abs `1e-3` cm (looser — pose is accumulated) |
| POSE-TOLERANT | `pose.heading` | circular `HEAD_TOL` (same constant as commanded headings) |

> `wheel` (the resolved `TurnMode`) is in the EXACT bucket but **must be emitted from each side's real turn dispatch** — C++ from `_departingPivot.wheel`/`PIPELINE_CONFIG.turn`, Python from the sim's resolved turn mode — **never recomputed in a shared normalizer** (that would produce vacuous agreement). The sim emits only `ACT_TURN_REL(delta)` today, so T5's normalizer surfaces the wheel from the sim's own `PivotTurn`/pipeline resolution, not from a re-derivation.

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

- `EdgeDispatchQuery` / `EdgeDispatchDecision` / `DispatchPrimitive` / `ReanchorKind` — defined once in `WRO2026/EdgeDispatchPolicy.h` (owner: T1); the Python mirror in `runtime.py` (owner: T2) must match field names + enum values byte-for-byte. **This field set, plus the trim-clamp arithmetic in the Data Model, is the frozen contract that lets T1 and T2 run in parallel** — they conform to the spec, not to each other, and the standalone `resolveEdgeDispatch` CSV truth-table parity test (T-CSV / TC1) is the cross-language drift guard. Any field T1 adds beyond this Data Model is a **contract change that updates this spec and re-syncs T2** — never a silent parallel addition. (The judge explicitly kept T1 ∥ T2 rather than serializing; the completeness of this contract is what makes parallel safe.)
- The **normalized trace record schema** above is a **3-place contract** (echo of the CTPS `drive.method` contract): the C++ emitter (T6), the Python C++-stream loader (T8), and the Python sim normalizer (T8) must all agree on record `kind` strings and field names. A `kind`/field the emitter produces but the loader doesn't know must be a **loud failure**, never a silent skip.

### Signatures

- `EdgeDispatchDecision resolveEdgeDispatch(const EdgeDispatchQuery& q)` — owner: T1 (`EdgeDispatchPolicy.h`); consumers: `PathTracker.cpp` (T1), the standalone CSV test (T-CSV).
- `def resolve_edge_dispatch(q: EdgeDispatchQuery) -> EdgeDispatchDecision` — owner: T2 (`runtime.py`); consumers: `_walk_hops` (T2), the standalone pytest (T-CSV).
- `field_fixture.build_field_data() -> <field-data-shaped namespace>` — owner: T3 (`WRO2026/test/fixtures/field_fixture.py`); consumers: `regenerate_routes_fixture.py` (T3) and the parity pytest's graph build (T8).
- C++ macro `PT_PARITY_TRACE(record)` — no-op inline when undefined (firmware); emits one NDJSON line when the dump target defines `PT_PARITY_TRACE`. Owner: T1; consumer: T6.
- Read-only `PathTracker` introspection accessors (owner: T1; consumer: the T6 parity driver): `State state() const` and `float resolvedBodyHeadingDeg() const`. The currently-private `enum class State` (`PathTracker.h`) is promoted to a **public** type (or a public mirror enum) as part of T1 — this is the load-bearing introspection surface the generic driver dispatches on each tick. No setter, no behavioral accessor, no mutation surface — `const` reads only. `resolvedBodyHeadingDeg()` is also the parity *assertion* surface (the T8 body asserts it equals the sim's per-hop body heading): the pre-edge turn target is reachable from no other public surface (`Drive::_turnTargetYaw` is private with no getter; the TurnGyro pre-turn does not stamp `lastMoveGyroDegsTarget`).
- Python `parity` module (`tools/path_planner/sim/parity.py` or `tests/parity_support.py`) — owner: T5 — exposes: `TraceRecord` schema, `normalize_sim_stream(sim, case) -> list[TraceRecord]`, `load_cpp_ndjson(text) -> dict[case_id, list[TraceRecord]]`, `integrate_pose(records, field) -> list[Pose]`, `load_allowlist(path)`. Consumers: T7 (allowlist data), T8 (the test).

### Enumeration contract

- The case set = **all reachable `(start, goal)` node pairs** over the fixture graph, sorted by `(start_name, goal_name)` (each driven from a clean `seatAt(start)` with zero operator offset), **plus** the curated pins in `CURATED_PARITY_CASES`. `case_id = f"{start}__{goal}__{offset_tag}"` is the join key, where `offset_tag` is `"clean"` for all-pairs.
- **Offset dimension (required for return-to-node coverage):** the return-to-node prologue fires only on a non-trivial accumulated operator offset, which a clean `seatAt` clears. So `CURATED_PARITY_CASES` includes a few cases that pre-load a **pinned operator-offset vector** before the `goTo` — applied **identically on both sides** (C++ via the harness `pushOperatorOffset(dx,dy,dθ)`; Python via the sim's `operator_move`), with the same vector encoded in `offset_tag`. Without this, AC8/TC6's `return_turn`/`return_move` coverage is unreachable and the coverage guard would fail (or be vacuous).
- Both the C++ dump (T6) and the Python test (T8) must enumerate the identical case set (all-pairs + the same `CURATED_PARITY_CASES` constant); a case-count mismatch between sides is a hard fail.

### Cross-tree Python import

- The parity pytest lives under `tools/path_planner/tests/` but must import `field_fixture` from `WRO2026/test/fixtures/`. Pin the import contract in T8: insert the repo root (or the fixtures dir) on `sys.path` via a small conftest/path shim, mirroring how `regenerate_routes_fixture.py` already adds `_REPO_ROOT` for the reverse direction. The fixtures dir gets an `__init__.py` if needed so `field_fixture` is importable as a module, not a loose script.

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
- CI job id: `parity`. Compile definitions: `PT_PARITY_TRACE`, `PARITY_SCHEMA_VERSION=1`. **Do NOT** define `-DLINE_BOARD_COUNT=2`: `ConfigLine.h:74` is a bare `constexpr uint8_t LINE_BOARD_COUNT = 2;` with no `_DEFAULT` override hook, so `-D` would rewrite that line to `constexpr uint8_t 2 = 2;` and break the build. It is also redundant — REAR/reverse is already reachable at the default `2`, exactly as the existing bidirectional host targets (which set no board-count def).

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
| T1 | Extract `EdgeDispatchPolicy.h` + refactor FSM + trace hook + read-only accessors | — | high | `WRO2026/EdgeDispatchPolicy.h` (new), `WRO2026/PathTracker.cpp`, `WRO2026/PathTracker.h` | Extract the terminal one-hop classifier per the Data Model (composes `resolveStopAtLine`/`resolvePivotTrim`/`resolveSensorOvershootTrim`; takes resolved inputs; owns the trim-clamp arithmetic, carries the post-clamp distance; `PIVOT_TRIM_UNDERFLOW` stays an FSM-raised fault). Refactor `_dispatchEdgeAfterTurn` to call `resolveEdgeDispatch` and act on the decision; hoist `_evaluateProjectionPassThrough` (side-effect-free) so `projPassThrough` is a resolved input. Add `PT_PARITY_TRACE(record)` no-op-when-undefined macro + guarded emit calls at FSM commit points (turn, line_follow, line_overshoot, projection_follow, stop_at_line_seek, blind_dead_reckon, return_*, wall_*). Promote the private `enum class State` to public and add read-only `state()` + `resolvedBodyHeadingDeg()` accessors (const reads only; no setter/mutation). Must NOT change firmware behavior. Satisfies AC1, AC4 (hook), AC12 (accessors). Verify: existing ctests + `arduino-cli compile`. |
| T2 | Python mirror `resolve_edge_dispatch` + dispatch refactor | — | high | `tools/path_planner/sim/runtime.py` | Add `resolve_edge_dispatch` field-identical to T1's resolver; refactor `_walk_hops` to dispatch through it (emitted ACT_* tuples unchanged). Satisfies AC2. Verify: existing pytest. (Parallel with T1; both conform to the Contracts struct.) |
| T3 | Single-source the fixture | — | high | `WRO2026/test/fixtures/field_fixture.py` (new), `.../regenerate_routes_fixture.py`, `.../routes_fixture.{h,cpp}` | Factor the inline field out of `regenerate_routes_fixture.py` into importable `field_fixture.build_field_data()`. Wire the regenerator to import it. Regenerate `routes_fixture.{h,cpp}` via the real emit pipeline. Confirm ALL sibling `routes_fixture`-linked host tests stay green (resolve by name). Satisfies AC3 (artifact), TC9. |
| T5 | Parity support module (schema + normalizer + pose + loaders) | T2 | high | `tools/path_planner/sim/parity.py` (new) | Implement `TraceRecord` schema, `normalize_sim_stream` (maps ACT_* tuples → records incl. front/reverse collapse + reanchor kind), `load_cpp_ndjson` (loud on unknown kind/field), `integrate_pose` (single `VirtualChassis`, snap-to-node on `ON_INTERSECTION`), `load_allowlist`. Implements the 3-place schema contract (Python side). |
| T6 | C++ parity-dump target + generic driver + CMake | T1, T3 | **high** | `WRO2026/test/test_path_tracker_parity_dump.cpp` (new), `WRO2026/test/CMakeLists.txt` | New host target linking real `PathTracker`/`Drive`/`Localization`/`YawSource` + `routes_fixture.cpp` + `path_mocks`, using `test_path_tracker_harness.h`. **T6.a — generic per-tick driver:** drive each `goTo` one tick at a time, each tick reading `PathTracker.state()` to dispatch to the matching existing helper (`TURNING`→`expectPreTurn` parked at `hwYaw(resolvedBodyHeadingDeg())`, `LINE_FOLLOW`/`LINE_OVERSHOOT`→`expectLineHop`, `BLIND_FORWARD`→`expectBlindHop`, `STOP_AT_LINE`→`expectStopAtLine`, `WALL_STOP`→`expectWallStop`, `PROJECTION_FOLLOW`→distance gate). **Never re-derive hop type/heading in the test** — feed off observed state only. Enumerate all-pairs + `CURATED_PARITY_CASES` (incl. the pinned-offset return-to-node cases), emit NDJSON via `PT_PARITY_TRACE`. Compile defs: `PT_PARITY_TRACE`, `PARITY_SCHEMA_VERSION=1` (NO `LINE_BOARD_COUNT`). Follow the #112 force-include inversion (sketch dir first; link `path_mocks.cpp`). Satisfies AC4. |
| T7 | Divergence allowlist data file | T5 | med | `tools/path_planner/tests/fixtures/parity_allowlist.json` (new) | Author the allowlist with the known intentional divergences seeded (fault FSM, constant-velocity timing, host-unreachable `LINE_NOT_FOUND` branch per ADR-030/the host-drive memory), each with reason + tracking issue. Conforms to the entry schema in the Data Model. Satisfies AC7 (data). |
| T8 | Python parity test (live diff) | T2, T5, T6, T7 | high | `tools/path_planner/tests/test_pathtracker_parity.py` (new) | Pin the cross-tree import of `field_fixture` (sys.path shim). Locate/invoke the dump binary (CTPS `_find_dump_exe`-style discovery), parse NDJSON, build the sim graph from `field_fixture`, run `step_through` per case (applying the pinned operator offset for the offset-tagged curated cases), normalize both streams, integrate C++ pose via the one `VirtualChassis`. Compare per the **three buckets** (EXACT / TOLERANT-NUMERIC dist+ang / POSE-TOLERANT) and **assert every field is bucketed** (unassigned field fails). Apply the allowlist (fail on stale entry), assert case-count parity + non-zero, run the coverage guard. Satisfies AC5–AC9, TC2–TC7. |
| T-CSV | Standalone `resolveEdgeDispatch` CSV parity | T1, T2, T6 | med | `tools/path_planner/tests/fixtures/edge_dispatch_truth_table.csv` + `gen_*.py` (new), `tools/path_planner/tests/test_runtime_sim.py` (add), `WRO2026/test/test_edge_dispatch_truth_table.cpp` (new), `WRO2026/test/CMakeLists.txt` (append) | Generate a categorical truth-table CSV for `resolveEdgeDispatch` (independent transcription, like `gen_stop_at_line_truth_table.py`); add a pytest + a C++ host target consuming it via `CsvTruthTable.h`. The C++ target stays in the **default host-tests ctest suite** (it is a pure-resolver test needing no binary handoff — same home as `test_stop_at_line_truth_table`, label `parity`), NOT the dedicated `parity` CI job. Satisfies AC9, TC1. Blocked by T6 to serialize the CMakeLists edit. |
| T-CI | Dedicated parity CI job + fixture freshness gate | T3, T6, T8 | med | `.github/workflows/ci.yml` | Add a `parity` job (sets up Python via uv + C++ via cmake/g++) that builds **only the trace-diff dump target**, runs the parity pytest (`test_pathtracker_parity.py`), and uploads both traces as artifacts on failure. The `test_edge_dispatch_truth_table` CSV target runs in the existing `host-tests` job (default suite), not here. Add a fixture-freshness step to the `path-planner` job: regenerate `routes_fixture` + `git diff --exit-code` (this is what now re-couples the fixture to `Config.h` — see Decisions / #91 reversal). Single owner of `ci.yml`. Satisfies AC10, TC8. |
| T-ADR | ADR-050 + DECISIONS.md (supersedes the #91 freeze decision) | T8 | low | `docs/decisions/050-pathtracker-sim-parity.md` (new), `DECISIONS.md` | Document the mechanism (real-FSM trace-diff, single-source gated fixture, semantic schema as a 3-place contract, three-bucket comparison, cumulative-pose cross-check, self-auditing allowlist, `EdgeDispatchPolicy` as terminal classifier, read-only parity accessors). **The ADR explicitly supersedes the issue-#91 freeze/decouple decision** and records the `routes_fixture`↔`Config.h` re-coupling as intentional. Update the index + status table. If ADR-050 was taken by a parallel PR, renumber per the collision convention. Satisfies AC11. |

**Parallelism:** T1, T2, T3 start in parallel (distinct files, all conform to the Contracts). T5 follows T2. T6 follows T1+T3. T7 follows T5. T8 joins T2+T5+T6+T7. T-CSV follows T1+T2+T6. T-CI joins T3+T6+T8. T-ADR last.

## Notes for Implementer

- **Behavior preservation is the load-bearing constraint of T1.** The extraction must be a pure refactor — `_dispatchEdgeAfterTurn` produces the identical Drive/LineFollower calls before and after. The existing real-FSM host tests (`test_path_tracker_*`) are the guardrail; run the full ctest after T1.
- **`commandedDistanceCm` is the command, not the displacement** (memory: `gotcha-sim-primitive-payload-is-command-not-displacement`). Seek = full `edge.distance_cm`; dead-reckon = trimmed. Pose trim lives only in the integrator.
- **The chassis lands axle-on-node** (memory: `gotcha-sim-chassis-axle-on-node`) — do not re-add a per-junction offset in the pose integrator; `ON_INTERSECTION` snaps to the node's mapped coordinate.
- **The trace schema is a 3-place contract** (memory: `gotcha-ctps-drive-method-contract`) — emitter, loader, normalizer change together; unknown kind/field is a loud failure.
- **The driver reads observed FSM state; it never re-derives.** The generic `goTo` driver dispatches each tick off `PathTracker.state()` and parks yaw at `hwYaw(resolvedBodyHeadingDeg())`. Computing the expected hop type/heading in the test from NEXT_HOP + EdgeDispatch would make the harness score its own prediction — a silent pass while C++ and the sim disagree. The harness's own invariant is "never re-implement an FSM decision."
- **`wheel` parity must come from each side's real dispatch**, not a shared recompute — C++ from `_departingPivot.wheel`/`PIPELINE_CONFIG.turn`, Python from the sim's resolved turn mode. The sim emits `ACT_TURN_REL(delta)` only, so T5 surfaces the wheel from the sim's own `PivotTurn`/pipeline resolution; never recompute it in the normalizer (vacuous agreement).
- **`commandedDistanceCm` is post-clamp** (`raw − departing − approaching`, clamped to `minDriveCm`); the resolver owns this arithmetic in both languages, and `PIVOT_TRIM_UNDERFLOW` is FSM-raised, not a decision field. This is what lets T1 ∥ T2 stay parallel without drift (the CSV truth-table guards it).
- **The fixture gate re-couples to `Config.h`** (the #91 reversal you accepted) — a `Config.h` track-width edit now restales `routes_fixture` and fails the freshness gate; that's intended. The field-move also changes the SHA stamp, so the first regen is a real (one-time) diff, not byte-identical.
- **Pose is a cumulative cross-check, not an oracle** — it cannot independently catch a reanchor-kind or node-coord bug (decision-compare / fixture-validator own those). Its unique signal is sub-tolerance drift accumulation + heading lever-arm amplification. Don't over-claim it; don't drop it.
- **Hoist `_evaluateProjectionPassThrough` carefully** — it must be side-effect-free when moved ahead of query construction so `projPassThrough` is a clean resolved input; verify no state mutation slipped earlier.
- **Front/reverse collapse in the normalizer:** the sim emits distinct ACT tags for FRONT vs REAR (`ACT_FOLLOW_INT` vs `ACT_LINE_REVERSE`, etc.); the normalizer collapses each pair to `{kind, reverse}` so it matches the C++ `{primitive, reverse}` emission.
- **Host-link inversion** (memory: `gotcha-host-link-inversion-sketch-dir-first`): the parity-dump target puts `SKETCH_DIR` first so the real `Drive.h`/`PathTracker.h` win; force-include only the peripheral `path_mocks`; link `path_mocks.cpp` (not `hal_mocks.cpp`). Reuse `add_path_force_includes` + `add_fixture_routes_force_include`.
- **Regenerated fixture must keep node names** the sibling tests resolve by (`Inter1`, `ProjL`, `Wall1`, `Spur`, ...). The single-source field is the same field already inline in the regenerator; factoring it out should be byte-stable. Run the full ctest after T3 and confirm zero behavioral change.
- **Pivot/reset/return/pass-through stay in the FSM** — do not pull them into `EdgeDispatchPolicy`. The trace covers them at the sequence level.
- **The dedicated CI job always builds the binary** — never gate the parity test behind "binary exists else skip" in CI (vacuous green). Local runs may skip with a message.
- **Allowlist discipline:** seed only structurally-excluded dimensions (fault FSM, timing) and the documented host-unreachable branches; never allowlist a whole case. The stale-entry guard is what keeps it honest.
- **Rollback:** every deliverable is additive except the T1 FSM refactor and the T3 fixture regen. If T1 misbehaves, revert the `_dispatchEdgeAfterTurn` change (the resolver header can stay, unused). If T3's regen churns unexpectedly, revert `routes_fixture.{h,cpp}` to the frozen copy and fall back to a separate parity fixture.
- Read `CONTRIBUTING.md` before opening the issue/PR.
