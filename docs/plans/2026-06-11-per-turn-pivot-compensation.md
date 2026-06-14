# Per-Turn Pivot Compensation Plan

**Goal:** Replace the per-edge single-wheel-pivot model (two unlinked fields on two edges) with a per-turn model — a pivot is declared on a 3-node turn `(u, v, w)`, fires only when the route takes that turn, derives its wheel and over-travel δ from geometry, and is stored as a small sparse wire table — closing the two review blockers at the root and folding in the rest of the "two-sided pivot compensation" review.

**Approach:** Author pivots as enum-free node-name triples in `field_data.json`. The offline pipeline computes, per triple, the over-travel `δ = (T/2)·tan(|θ|/2)` and the pivot wheel from `sign(θ)`, and emits a sparse, sorted `PivotTurn[]` table into `routes.h` (no dense `N³` array). The firmware resolves the turn per hop via a one-hop lookahead (the existing `getNextHop` peek pattern) and a single `_pendingPivot` latch — one lookup decides both the maneuver and the trim, so they cannot disagree. **The runtime trim mechanics — the regime matrix, the LINE-overshoot reduction, the dead-reckon approach/departure subtraction, and the one-hop carry — are carried over from ADR-035 unchanged; only the *source* of δ (per-turn table vs per-edge field) and the *wheel resolution* (one lookup) change.** `pivot_trim_cm` and `pre_turn_mode_override` are removed from `EdgeInfo` entirely. The Python sim mirrors the firmware after a no-op dedup refactor. This supersedes ADR-035 and closes #44; reverse pivots (#54) and editor authoring (#52) are tracked out of scope.

## Scope

- **In scope:**
  - `pivot_turns` authoring in `field_data.json` (node-name triples) + loader threading + `field_data.py` re-export.
  - Offline derivation of δ + wheel per triple; sparse `PivotTurn[]` + `N_PIVOT_TURNS` emission into `routes.h`/`.cpp`.
  - Removal of `pivot_trim_cm` and `pre_turn_mode_override` from `EdgeInfo` (struct, both emitter cell builders, the `offsetof` guard, `FieldMap::noEdge`).
  - Firmware per-turn resolution: `_pendingPivot` latch, one-hop lookahead, FRONT-only guard, raised min-angle floor, EventTrace on pivot fire/skip.
  - Validator: per-turn validation (triple-edge existence, angle bounds, regime matrix, chained-pivot sum-check, FRONT-only, duplicate-triple); the five review regime-boundary fixes; the listed nits.
  - Sim: extract the duplicated per-hop dispatch into one helper (precursor, no-op), then implement per-turn dispatch in that single helper.
  - Regenerate `routes.h`/`.cpp` + toy goldens; new ADR-036 superseding ADR-035; `DECISIONS.md` + `Roadmap.md` doc-drift fixes.
- **Out of scope (tracked):**
  - Reverse (REAR-traversal) pivots — **#54**. v1 is FRONT-only; REAR pivot hops are rejected by the validator and skipped (normal turn + trace) at runtime.
  - Field-editor GUI authoring of pivots and the broader `EDGES`-enum→JSON migration — **comment already posted on #52**.
  - Wiring `Config::PIVOT_SCRUB_FACTOR` (hardware correction factor) — tracked by Roadmap **B3.8**. The constant is referenced forward-lookingly in ADR-035 + Roadmap but is not yet defined; this plan only clarifies that it is a future (not broken) reference, it does not define or wire it.
  - The chassis pivot hardware bench gate (**B3.8**) itself — unchanged; no mission may use a pivot hop until it passes.

## Decisions

- **Per-turn, not per-edge** — a pivot is a property of a turn `(u,v,w)`, fires only when the route takes it, so node `v` may have multiple continuations. Dissolves Blocker 1 (the trim and the maneuver share one source and cannot drift) and deletes the old single-forward-out-edge landing requirement (finding #36).
- **Wheel + δ derived offline from geometry** — `θ = normalize(heading(v→w) − heading(u→v))`; `δ = (T/2)·tan(|θ|/2)`; **`wheel = PIVOT_LEFT if θ>0 else PIVOT_RIGHT`**. Authors never type a wheel. Both baked into the wire record (geometry math stays in Python where pytest reaches it). The sign→wheel mapping is NOT self-evident — it must be derived in ADR-036 from the wheel-byte semantics (`PIVOT_LEFT` = brake left wheel, drive right → nose swings CCW/left = θ>0; PathTracker.cpp:150–152) plus the forward-travel assumption, not asserted.
- **Sparse wire table, not dense** — `PivotTurn PIVOT_TURNS[]` (a handful of records) over `TURNS[N][N][N]` (`N³` ≈ 125 KB at N=50 on a 256 KB-flash board, infeasible). Linear scan; sorted for deterministic goldens; emit `max(1, count)` elements so a pivot-free field compiles.
- **Latch, not previous-node tracking** — resolve the whole turn once at approach dispatch, latch `_pendingPivot{wheel, trim}`, consume it at the next pre-turn (maneuver) and drive (carry). Rides the existing boot/goal/fault resets, so a stop at `v` degrades to a normal turn and pivots never fire across a `goTo()` boundary.
- **Remove the per-edge fields entirely** — no fallback path; it would resurrect the disagreement bug and double the validator + sim surface. (Separately: `EdgeInfo` shrinks 24→16 bytes = 8 bytes/cell × N², a minor incidental flash saving — distinct from the sparse-vs-N³ argument above.)
- **Chained pivots: allow when they fit** — validator rejects a shared edge only when `δ_uv + δ_vw ≥ length(v→w)`; the firmware's two independent subtractions (carry + approach) already land fitting chains correctly with no firmware change.
- **FRONT-only in v1** — reverse pivots deferred to #54.
- **JSON now, narrowly** — `pivot_turns` lands in `field_data.json`; the `EDGES` enum→JSON unwind is left to #52.

## Data Model

### Authoring — `field_data.json`

```jsonc
"pivot_turns": [
  ["A", "B", "C"]    // arrive from A, single-wheel pivot at B, leave to C
]
```
A list of **3-element node-name arrays** (same array-of-arrays shape as `LINE_SEGMENTS` / `BLIND_EDGES`, so the loader's existing tuple discipline applies — see T1). Absent / `[]` is valid (no pivots). Re-exported by the loader/`field_data.py` as `field_data.PIVOT_TURNS` — a list of `(u, v, w)` string **tuples** (the loader converts the JSON arrays to tuples, exactly as it does for the other edge lists; there is no dict form anywhere).

### Offline record (planner) → wire

```python
# planner.build_pivot_turns(graph, track_cm) -> list[PivotTurnRecord]
@dataclass(frozen=True)
class PivotTurnRecord:
    u: str; v: str; w: str        # node names
    wheel: TurnMode               # PIVOT_LEFT | PIVOT_RIGHT (derived from sign(theta))
    trim_cm: float                # delta = (track_cm/2)*tan(|theta|/2)
```

### Wire — `routes.h`

```cpp
struct PivotTurn {
    uint8_t  u;          // node index: arrive-from
    uint8_t  v;          // node index: pivot-at
    uint8_t  w;          // node index: leave-to
    TurnMode wheel;      // PIVOT_LEFT | PIVOT_RIGHT (uint8_t enum)
    float    trim_cm;    // delta over-travel, cm
};                       // 8 bytes (float forces 4-byte align)

constexpr uint8_t N_PIVOT_TURNS = <count>;                 // truth count (may be 0)
extern const PivotTurn PIVOT_TURNS[/* max(1, count) */];   // sorted by (u, v, w)
```

`EdgeInfo` AFTER removal (was 24 bytes, now 16):
```cpp
struct EdgeInfo {
    EdgeType       type;                       // 0
    float          distance_cm;                // 4
    float          heading_deg;                // 8
    LineMode       line_mode_override;         // 12
    BlindMode      blind_mode_override;        // 13
    ProjectionMode projection_mode_override;   // 14
    DirectionMode  direction_override;         // 15
};   // pivot_trim_cm and pre_turn_mode_override REMOVED
```
The `offsetof(EdgeInfo, pivot_trim_cm) == 20` static_assert is removed.

### Firmware pivot state & latch contract

```cpp
// PivotRef = { bool active; TurnMode wheel; float trim_cm; } — a RESOLVED COPY of a
// PivotTurn record, never a table pointer kept across hops.
PivotRef _pendingPivot;       // carry: the approaching pivot latched last hop -> this hop's departing
PivotRef _departingPivot;     // pivot we LEAVE at `cur` this hop (snapshot of _pendingPivot)
PivotRef _approachingPivot;   // pivot we ENTER at `nxt` this hop (fresh lookup)
```
Per-hop contract (resolves the chained case unambiguously — **snapshot BEFORE overwrite**, computed at plan time, both survive the TURNING state into the drive):
```text
// plan time (_onPlanNextHop), BEFORE any overwrite of _pendingPivot:
_departingPivot   = _pendingPivot;                                   // snapshot the carry
_approachingPivot = (nxt == dest) ? NONE
                                  : findPivotTurn(cur, nxt, getNextHop(nxt, dest));
_pendingPivot     = _approachingPivot;                               // carry for the NEXT hop

// pre-turn maneuver wheel = _departingPivot.active ? _departingPivot.wheel : pipeline default
// drive (after TURNING):
//   dead-reckon : effective = raw - _departingPivot.trim   // outgoing comp
//                                 - _approachingPivot.trim  // approach comp
//   LINE-incoming: overshoot advance reduced by _approachingPivot.trim;
//                  _departingPivot dropped (the line re-anchors — regime matrix)
```
The departing trim and the approaching trim come from **two distinct resolved records**, so a chain `u→v→w→x` (pivots at v and w) never loses or misroles a δ on the shared `v→w` edge. A single `_pendingPivot` member is sufficient ONLY because `_departingPivot` is snapshotted before `_pendingPivot` is overwritten.

## API Contracts

### FieldMap (firmware accessor, header-only, bounds-checked per ADR-028)

```cpp
// Returns the matching pivot record for the turn (u,v,w), or nullptr if none.
// Linear scan over PIVOT_TURNS; out-of-range indices return nullptr (ADR-028).
inline const Routes::PivotTurn* FieldMap::findPivotTurn(uint8_t u, uint8_t v, uint8_t w);
```

### Planner

```
build_pivot_turns(graph, track_cm) -> list[PivotTurnRecord]
  - For each (u,v,w) in field_data.PIVOT_TURNS:
      require edges u->v and v->w exist in graph (else handled by validator first)
      theta = normalize(heading(v->w) - heading(u->v)) into (-180, 180]
      delta = compute_pivot_compensation(theta, track_cm)   # reuse, do NOT re-derive
      wheel = PIVOT_LEFT if theta > 0 else PIVOT_RIGHT
  - Returns records; apply_pivot_compensations (edge mutation) + its stale-delta
    idempotency guard are DELETED.
  - track_cm defaults to parse_track_cm_from_config() but stays an explicit param
    (bypass callers e.g. CTPS pass it; no new hard Config.h dependency).
```

### Validator (`_validate_pivot_turns`, replaces `_validate_pivot_regimes` + finding #36)

Fail-fast `FieldDataError`, each precise, on:
- A `pivot_turns` entry whose `u→v` or `v→w` is not an authored edge.
- Duplicate `(u,v,w)` triple.
- Degenerate angle: `|θ| ≥ _PIVOT_MAX_TURN_DEG` (179) or `|θ| < _PIVOT_MIN_TURN_DEG` (raised to `max(15, 3×NAV_TURN_TOLERANCE_DEG)`).
- Unsupported regime combo (incoming × outgoing not in the matrix).
- `δ ≥ edge length` on a dead-reckoned side (incoming or outgoing).
- `δ >` LINE-incoming overshoot budget (= **front** sensor-to-axle offset, FRONT-only; single-sourced from Config.h).
- Chained pivots: for two records sharing edge `(v,w)`, `δ_uv + δ_vw ≥ length(v→w)`.
- FRONT-only: an authored `direction: "rear"` on any edge adjacent to a pivot turn.

## Error Handling

- **Pivot-free field:** `N_PIVOT_TURNS == 0`; emit a 1-element placeholder array; `findPivotTurn` returns `nullptr` for everything. Regeneration is **deterministic** (re-runs produce identical bytes); the toy goldens still **differ from the pre-change goldens** (EdgeInfo shrinks, PIVOT_TURNS added) and are re-baselined in T12.
- **Route doesn't take the declared turn:** `findPivotTurn(cur, nxt, nextnext)` misses → no trim, no pivot maneuver, full distance. Correct by construction.
- **Last hop (`nxt == dest`):** skip only the *forward* lookahead that would latch a NEW pending pivot (no pivot can land on the final node mid-turn). This NEVER skips consuming an already-pending latch — a pivot `(u,v,w)` whose goal is `w` still consumes its wheel + trim on the final `v→w` hop.
- **Direction on a pivot hop is forced FRONT.** A pivot is FRONT-only (wheel/δ derived for forward traversal), so the firmware **overrides the resolved direction to FRONT** on any pivot-adjacent hop (the approach `u→v` or the departure `v→w`), regardless of the auto-bidirectional heuristic. The validator already rejects *authored* REAR there. A silent downgrade to a counter-rotation is unacceptable — the author chose a single-wheel pivot for clearance a center counter-rotation may not have. The earlier "skip the pivot + trace" path is demoted to a defensive backstop for a should-be-impossible state (FRONT genuinely unavailable, which faults via the existing reverse-availability guard), NOT normal semantics (#54).
- **Trim underflow (bad table):** existing `PIVOT_TRIM_MIN_DRIVE_CM` (0.5) clamp + Serial trace retained; should be unreachable given the validator's `δ < length` and chained sum-check.
- **Min-angle vs turn tolerance:** the pre-turn maneuver only dispatches when `|bodyTarget − currentWorld| > NAV_TURN_TOLERANCE_DEG` (PathTracker.cpp:951). The raised min-angle floor (`≥ 3× tolerance`) guarantees a real pivot's turn exceeds the gate **under the assumption that post-arrival IMU heading matches the inbound edge heading within tolerance** — the same assumption the existing yaw-vs-peek divergence guard (PathTracker.cpp:953) already manages. ADR-036 must state this assumption; if IMU drift exceeds it, the divergence guard, not the pivot logic, is the mitigation.

## Testing Strategy

**Levels:** Unit (validator, planner, header_writer), Integration (sim FSM parity, golden byte-compare), Build (ctest host harness + arduino-cli compile).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | δ + wheel derivation: θ=+90 / −90 / +60 / +120 over a triple | Unit | δ matches `(T/2)tan(θ/2)`; wheel = LEFT for θ>0, RIGHT for θ<0 |
| TC2 | `build_pivot_turns` reuses `compute_pivot_compensation` (no inline re-derive) | Unit | Identical to direct call; no second formula in validator/planner |
| TC3 | Validator rejects triple with non-existent `u→v` or `v→w` | Unit | `FieldDataError` naming the missing edge |
| TC4 | Validator rejects duplicate `(u,v,w)` | Unit | `FieldDataError` |
| TC5 | Validator rejects degenerate angle (≥179, and <raised floor) | Unit | `FieldDataError` on both bounds |
| TC6 | Validator rejects `δ ≥ outgoing` and `δ ≥ incoming` dead-reckon length | Unit | `FieldDataError` per side |
| TC7 | Validator rejects `δ >` LINE-incoming budget (front offset, Config-sourced) | Unit | `FieldDataError`; accepts `δ ≤` budget |
| TC8 | Regime fix: no-override BLIND with `PIPELINE_CONFIG.blind=STOP_AT_LINE` classifies sensor | Unit | Outgoing trim no-ops; no false `δ≥length` rejection |
| TC9 | Regime fix: incoming mode read via precedence/bidirectional resolver | Unit | STOP_AT_LINE authored on reverse leg classifies incoming as sensor |
| TC10 | Regime fix: STOP_AT_LINE-incoming NOT subject to the LINE overshoot budget | Unit | Accepts a short STOP_AT_LINE-incoming edge the old gate over-rejected |
| TC11 | Chained pivots: `δ_uv + δ_vw < length(v→w)` accepted; `≥` rejected | Unit | Fitting chain passes; over-trim chain `FieldDataError` |
| TC12 | FRONT-only: authored `direction:"rear"` on a pivot-adjacent edge | Unit | `FieldDataError` |
| TC13 | Header: `PIVOT_TURNS[]` emitted, sorted, with `N_PIVOT_TURNS`; `max(1,count)` for empty | Unit | Correct C++ literal; pivot-free field emits 1-elem placeholder + count 0 |
| TC14 | Header: `EdgeInfo` has no `pivot_trim_cm` / `pre_turn_mode_override`; both cell builders + `noEdge` updated; old offsetof guard gone | Unit | Struct + initializers consistent; compiles |
| TC15 | Toy goldens regenerate byte-identical after the wire change | Integration | `routes_toy.{h,cpp}` match; `build_routes` + `git diff --exit-code` clean |
| TC16 | Sim per-turn dispatch — dead-reckon incoming+outgoing both trimmed; axle lands on `w` (drift ≈ 0) | Integration | `_POSITION_TOL_CM` preserved; matches firmware semantics |
| TC17 | Sim — non-pivot route through a node carrying a pivot turn drives FULL distance | Integration | No trim applied |
| TC18 | Sim — LINE-incoming pivot reduces overshoot by δ, latches outgoing trim | Integration | Overshoot = front offset − δ; outgoing dead-reckon trimmed |
| TC19 | Sim — chained fitting pivots `u→v→w→x` land with ≈0 residual | Integration | Both shared edges net `length − δ_in − δ_out` |
| TC20 | Sim — FRONT-only: a pivot hop the bidirectional **auto** heuristic would resolve REAR is forced FRONT and the pivot fires | Integration | Fixture constructs a pivot hop where auto would pick REAR (obtuse turn + reverse-cheaper geometry); the firmware/sim overrides to FRONT and the trim/maneuver apply normally — no silent downgrade. The skip+trace backstop stays a should-be-impossible path. |
| TC21 | Sim — `go_to` and `step_through` emit identical primitives via the shared helper | Integration | Byte-identical primitive stream |
| TC22 | Host ctest + arduino-cli compile of the firmware | Build | Links + compiles; no ODR/offsetof failures |
| TC23 | Anti-Blocker-1: the firmware applies the SAME `PivotTurn` record's `wheel` as the turn-mode byte that supplied its `trim_cm` | Unit (firmware/host) | The dispatched wheel byte == `record.wheel`; trim == `record.trim_cm`; they cannot come from different sources (the sim cannot catch this — it does not model wheel patterns, so this is a host-side assertion) |
| TC24 | Header: populated multi-record `PIVOT_TURNS` emission via a synthetic 2-pivot graph | Unit | Records emitted sorted by (u,v,w) with correct indices/wheel/δ (the toy golden has zero pivots, so this is the only coverage of a non-empty table) |
| TC25 | `FieldMap::findPivotTurn` bounds-check | Unit (host) | Out-of-range u/v/w returns `nullptr`; a present triple returns the record; an absent triple returns `nullptr` |
| TC26 | Budget/offset constants single-sourced | Unit | Validator + sim read `LINE_SENSOR_AXLE_OFFSET_CM` (and `PIVOT_TRIM_MIN_DRIVE_CM`) from `Config.h` — like `TRACK_MM` today; a parity test asserts the parsed values match the firmware header, so a Config.h edit can't silently desync validator/sim/firmware |

**Test data:** extend the toy fixture with a pivot-bearing variant (a node with two continuations where only one turn is a pivot, plus a fitting chain, plus the auto-REAR fixture for TC20). Reuse `parse_track_cm_from_config` against the real `Config.h` `TRACK_MM` (18 cm).
**Run command:** `uv run pytest tools/path_planner/tests/`; host build via bundled `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` and the ctest harness.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Authoring layer + loader + editor preservation | — | med | `tools/path_planner/field_data.json`, `tools/path_planner/editor/loader.py`, `tools/path_planner/editor/model.py`, `tools/path_planner/field_data.py` | Add `pivot_turns` (array-of-3-arrays) to JSON. Thread the key through ALL THREE loader contract points: `_TOP_LEVEL_KEY_ORDER`, `_convert_arrays_to_tuples` (convert each 3-array to a `(u,v,w)` tuple, like `LINE_SEGMENTS`), and `_build_ordered_payload` (else the determinism dump raises `KeyError`). **Editor preservation: the editor is a SECOND schema — `FieldModel` (dataclass field), its own `_JSON_KEY_ORDER`, `to_json_dict()`, AND the from-JSON constructor in `model.py` all omit unknown keys and would strip `pivot_turns` on open+save. Add `pivot_turns` as a preserved field there (no GUI controls needed).** Ensure `edit_field.py --migrate` emits the key. Re-export `field_data.PIVOT_TURNS`. Remove the old `pivot` / `pre_turn_mode` keys from the `EDGES` example + module docstring. Add loader AND editor round-trip tests. Satisfies authoring half of Decisions. |
| T2 | Planner derivation | T1 | high | `tools/path_planner/planner.py` | Add `build_pivot_turns(graph, track_cm)` returning `PivotTurnRecord`s (θ→δ via existing `compute_pivot_compensation`; wheel from `sign(θ)`). DELETE `apply_pivot_compensations` edge mutation + the stale-δ idempotency guard. Keep `compute_pivot_compensation`, `parse_track_cm_from_config` (track_cm stays an explicit param). Decouple `compute_all_pairs_next_hop` from pivots — it no longer calls the deleted `apply_pivot_compensations`; `build_pivot_turns` is invoked separately by the orchestrator (T16). Stop importing `Pivot`. Satisfies TC1, TC2. |
| T3 | graph_builder geometry helper (add-only) | — | low | `tools/path_planner/graph_builder.py` | ADD a directed-edge-heading helper (`heading(u→v)` for a node pair) that the validator and planner reuse instead of reimplementing `hypot`/`atan2`/`+180`/precedence. Do NOT delete `Pivot` or the directional plumbing yet (consumers still import it until T2/T4/T5 land — see T15). |
| T4 | Header emission | T2, T3 | high | `tools/path_planner/header_writer.py` | Emit `struct PivotTurn` + `const PivotTurn PIVOT_TURNS[max(1,count)]` (sorted by u,v,w) + `N_PIVOT_TURNS`. `emit_routes(...)` gains a `pivot_records` parameter (the list from `build_pivot_turns`, passed by the orchestrator T16). REMOVE `pivot_trim_cm` + `pre_turn_mode_override` from the `EdgeInfo` struct, BOTH `_edge_cell` and `_none_edge_cell` positional initializers, and the `offsetof` static_assert. Satisfies TC13, TC14, TC24. |
| T5 | Validator rewrite + review fixes | T1, T3 | high | `tools/path_planner/validator.py` | Replace `_validate_pivot_regimes` + finding-#36 out-degree check with `_validate_pivot_turns` (triple-edge existence, dup, angle bounds w/ raised floor, regime matrix, `δ<length` both sides, LINE budget, chained sum-check, FRONT-only). Fold ALL FIVE review regime fixes: ① `_regime` reads `PIPELINE_CONFIG.blind`; ② incoming mode via `_resolve_authored_directed_edge` precedence (not bare `entry.get`); ③ LINE budget only for true LINE, STOP_AT_LINE-incoming exempt; ④ the REAR LINE budget boundary is obsoleted by FRONT-only — LINE budget uses the FRONT offset, single-sourced from Config.h; ⑤ planner-side `track_cm`/offset reads stay explicit params (T2). Nits: call `compute_pivot_compensation` (no inline re-derive); use T3's geometry helper (drop the reimplemented `hypot`/`atan2`/precedence); consolidate the duplicate `_normalized_turn_deg` into the one canonical normalizer + fix its `(-180,180]` docstring; early-return so Config.h is parsed only when `pivot_turns` is non-empty (kill the per-build dead read); single-source the LINE budget + min-drive constants from Config.h (parity test TC26); stop importing/validating the EDGES `pre_turn_mode`/`pivot` keys. Satisfies TC3–TC12, TC26. |
| T6 | Regenerate routes | T16 | low | `WRO2026/routes.h`, `WRO2026/routes.cpp` | Run `uv run python tools/path_planner/build_routes.py`; commit regenerated headers. Blocked by T16 (orchestrator must compute + thread pivot records, which itself depends on T2/T3/T4). AC: CI `build_routes` + `git diff --exit-code` clean. |
| T7 | FieldMap accessor + noEdge | T6 | med | `WRO2026/FieldMap.h` | Add `findPivotTurn(u,v,w) -> const PivotTurn*` (bounds-checked linear scan, ADR-028). Update `noEdge()` positional init for the shrunk `EdgeInfo`. NOTE: the firmware build is green only once T6 (struct shrink in routes.h) AND this task BOTH land — `noEdge` must match the regenerated struct. Satisfies TC25. |
| T8 | Firmware per-turn FSM | T7 | high | `WRO2026/PathTracker.cpp`, `WRO2026/PathTracker.h`, `WRO2026/Config.h` | Implement the latch contract from Data Model → *Firmware pivot state* (`_pendingPivot`/`_departingPivot`/`_approachingPivot`; snapshot departing BEFORE overwrite; both survive TURNING). Pre-turn maneuver = `_departingPivot.wheel` if active else pipeline default. Lookahead `findPivotTurn(cur, nxt, getNextHop(nxt,dest))` (skip if `nxt==dest`). **Convert ALL existing pivot reads** to the latched records: `_pivotTrimDistance` (dead-reckon, ~1016/1024), `_resolveSensorEdgePivotCarry` (~1059), AND the LINE_FOLLOW-state read at PathTracker.cpp:672 (LINE-incoming approach trim runs in a DIFFERENT FSM state — confirm `_approachingPivot` is resolved before LINE_OVERSHOOT is armed). Remove `_resolveTurnMode`/`pre_turn_mode_override` reads. **Force FRONT on any pivot-adjacent hop** (override resolved direction; never silently downgrade to a counter-rotation); EventTrace on pivot fire; the REAR skip+trace is a defensive impossible-state backstop only. **Move `PIVOT_TRIM_MIN_DRIVE_CM` from its file-local literal into Config.h** (Config.h single-source rule, Config.h:7). Clear all pivot state at boot/goal/fault. Satisfies firmware half of all ACs + TC23. |
| T9 | Sim dedup precursor | — | med | `tools/path_planner/sim/runtime.py` | Extract the per-hop pivot dispatch shared by `go_to` and `step_through` into ONE private per-hop primitive producer. NOTE the asymmetry: `go_to` returns a list, `step_through` is a generator (`yield (primitive, snapshot)`) — the shared helper should produce per-hop primitives that each caller consumes its own way (append vs yield). No behavior change. TC21 pins equivalence. |
| T10 | Sim per-turn dispatch | T2, T8, T9 | high | `tools/path_planner/sim/runtime.py` | In the single helper from T9, replace per-edge reads with `PIVOT_TURNS` lookup + `_pending_pivot` latch mirroring T8 byte-for-byte; FRONT-only skip; LINE budget single-sourced. Satisfies TC16–TC21. |
| T11 | Python unit tests | T4, T5 | med | `tools/path_planner/tests/test_planner.py`, `tools/path_planner/tests/test_field_data.py`, `tools/path_planner/tests/test_header_writer.py` | Rewrite pivot tests to per-turn authoring; add chained, FRONT-only, dup, regime-fix, populated-emission (TC24) cases; remove out-degree-1 tests. Satisfies TC1–TC14, TC24. |
| T12 | Sim + golden tests | T6, T10 | med | `tools/path_planner/tests/test_runtime_sim.py`, `tools/path_planner/tests/golden/routes_toy.h`, `tools/path_planner/tests/golden/routes_toy.cpp` | Rewrite TC5–TC8-class sim tests to per-turn; add chained-fitting (TC19), auto-REAR-skip (TC20), go_to/step_through parity (TC21); regenerate goldens. Satisfies TC15–TC21. |
| T13 | Docs + ADR | T2 | low | `docs/decisions/036-per-turn-pivot-compensation.md`, `docs/decisions/035-two-sided-pivot-compensation.md`, `DECISIONS.md`, `Roadmap.md` | Write ADR-036 (per-turn model, supersedes 035) — MUST derive the `sign(θ)→wheel` mapping from the wheel-byte semantics + forward-travel assumption (source-cited, not asserted) and state the min-angle/IMU-heading assumption. Mark 035 Superseded by 036. Update DECISIONS.md index + status table (Implemented stays 29: −035 +036; Superseded 1→2: add 035). Clarify the `PIVOT_SCRUB_FACTOR` forward reference + the 12.02-vs-budget drift. Reference #44 (closed by this), #54, #52 (comment already posted). |
| T15 | graph_builder Pivot/plumbing removal | T1, T2, T3, T4, T5 | low | `tools/path_planner/graph_builder.py` | After ALL importers (`field_data`, `planner`, `header_writer`, `validator`) stop using it, delete the `Pivot` enum and the `pivot` / `pre_turn_mode` directional-attr plumbing (`_extract_directional_attrs`, `_DIRECTIONAL_OPTIONAL_KEYS`). Atomic-with-T5 hazard: deleting `Pivot` earlier breaks every importer's `from graph_builder import Pivot`, so this is gated behind all of them. |
| T16 | Orchestrator threading | T2, T4 | low | `tools/path_planner/build_routes.py` | After `build_graph`, compute `pivot_records = build_pivot_turns(graph, track_cm)` and thread them into `emit_routes(..., pivot_records=...)`. The pivot trim is no longer hidden inside `compute_all_pairs_next_hop` (T2 decoupled it), so the orchestrator owns the explicit call. |
| T14 | Integration verify | T6, T8, T10, T11, T12, T13, T15, T16 | med | (no source edits) | Full `uv run pytest tools/path_planner/tests/`; regenerate routes + `git diff --exit-code`; bundled `arduino-cli` compile + host ctest. Confirm TC22 and no golden drift. |

## Notes for Implementer

- **The latch contract (Data Model → *Firmware pivot state*) is the crux and is non-negotiable.** Snapshot `_departingPivot = _pendingPivot` BEFORE the lookahead overwrites `_pendingPivot`. Departing trim (outgoing) and approaching trim (approach) come from TWO distinct resolved records — on a chain `u→v→w→x` the hop `v→w` subtracts both. Both records resolve at plan time and must survive the TURNING state into the drive (members, not locals).
- **Pivot hops are forced FRONT** — override the resolved direction to FRONT on any approach/departure hop of a pivot; never silently downgrade to a counter-rotation (the author chose a pivot for clearance a center turn may not have). The validator forbids *authored* REAR; the runtime force-FRONT handles the auto-bidirectional case.
- **`PIVOT_TRIM_MIN_DRIVE_CM` moves to Config.h** — it is a file-local literal in PathTracker.cpp today, violating the single-source rule (Config.h:7). Validator + sim then read it (plus `LINE_SENSOR_AXLE_OFFSET_CM` and `NAV_TURN_TOLERANCE_DEG`) from the header like `TRACK_MM`. The raised min-angle floor is a **validator-only** Python constant derived from `NAV_TURN_TOLERANCE_DEG`, not a new firmware constant.
- **The LINE-incoming approach trim is in a different FSM state.** It lives in the `update()` LINE_FOLLOW state (PathTracker.cpp:672), not `_onPlanNextHop`/`_pivotTrimDistance`. T8 must ensure `_pendingPivot` is already latched (from the approach-hop dispatch) before LINE_OVERSHOOT is armed there, and convert that `edge.pivot_trim_cm` read to the latched value.
- **EdgeInfo three-site removal gotcha** (project memory): the struct def, BOTH cell builders (`_edge_cell` + `_none_edge_cell`), the `offsetof` static_assert, AND `FieldMap::noEdge`'s positional init must all drop the two fields together, or the positional aggregate init silently misbinds. The shrink only warns under `--warnings all`. The Python sites (T4) and the C++ `noEdge` (T7) land in different tasks — the firmware build is only green once both + the regen (T6) are in.
- **graph_builder `Pivot` deletion is import-ordered.** `validator`, `planner`, `header_writer`, and `field_data` all import `Pivot`/`pivot` plumbing today. T3 only ADDS the geometry helper; the deletion is T15, gated behind every importer. Do not delete `Pivot` in T3.
- **FRONT-only is enforced twice** — validator rejects authored REAR on pivot-adjacent edges (build-time), firmware skips on `_resolvedDirection == REAR` (runtime backstop for auto-bidirectional). Don't rely on only one.
- **Reuse, don't re-derive** — δ from `compute_pivot_compensation`; directed-edge geometry from T3's helper; turn normalization from the one canonical normalizer. The review flagged three duplications; this rewrite collapses them.
- **Goldens are byte-locked in CI.** The toy field has no pivots, so `PIVOT_TURNS` is the 1-element placeholder and `EdgeInfo` cells lose two fields — both change the golden bytes. Regenerate in T6, re-baseline in T12; TC24 covers populated emission since the golden cannot.
- **Sim parity is the oracle** — T9 (dedup) MUST land before T10 so the pivot change is single-site on the sim side; `go_to` and `step_through` must stay identical (TC21). The sim does NOT model wheel patterns, so the wheel↔trim single-source property is asserted host-side (TC23), not in the sim.
- **No dynamic allocation / fixed arrays** (ARCHITECTURE iron rules) — `PIVOT_TURNS` is a `constexpr`-sized const array; `findPivotTurn` is a bounded linear scan.
- **Rollback:** the change is contained to the pivot path and is latent (no pivots authored on the real field yet), so reverting the branch restores byte-identical routes. The B3.8 hardware gate still blocks any mission from using a pivot hop until bench-verified.
