# Issue #44 — Two-Sided Pivot Compensation Plan

**Goal:** Make a pivot turn land the chassis on the node despite the memoryless `NEXT_HOP` table, by correcting the *two-sided* pivot translation (both the incoming and outgoing adjacent drives) with the geometrically correct `δ = (T/2)·tan(θ/2)` magnitude, without ever folding a route-specific distance into a shared global `EDGES` cell. Single-source the chassis track width as part of the fix.

**Approach (precomputed-δ field — chosen after a `/debate`, Lean Yes 7.0):** A single-wheel pivot at node `v` rolls the axle along a chord; decomposed into the incoming/outgoing drive directions it is exactly `δ = (T/2)·tan(θ/2)` of forward over-travel on each side. Compute `δ` **offline** in the planner and store it on the **pivot-tagged edge** `u→v` (not the following edge), emitted as a new `EdgeInfo.pivot_trim_cm` field — so the shared `EDGES` distances and `NEXT_HOP` stay byte-identical and non-pivot routes are unaffected (the #44 aliasing is gone at the root). The firmware (and the sim mirror), while executing a pivot edge, apply the trim at dispatch with **no runtime trig** (just read the field):
- **Dead-reckoned incoming (blind / projection):** trim the approach drive (`L_uv − δ`) so the pivot starts `δ` short of `v`, carry `δ` one hop, and trim the following drive (`L_vw − δ`). Lands the axle on `w` with **zero residual**.
- **Line incoming (re-anchors at `v`):** the robot can't stop short, so reduce the `LINE_OVERSHOOT` advance by `δ` (so the axle settles `δ` short of `v`), then apply the outgoing trim. Equivalent to the dead-reckon case within the overshoot budget.
- **Sensor-terminated edges receive no trim by design** — they self-correct on the physical line, so the trim is correctly inert there; it bites only on distance-terminated drives, which is exactly where subtracting `δ` works.

A validator pass classifies each pivot by its incoming/outgoing edge types and fails fast on any configuration the geometry can't realize (degenerate angle, `δ ≥ edge length`, `δ >` overshoot budget). The planner's hard-coded track is replaced by code-genning `Config::TRACK_MM` so the offline `δ` can never drift from the firmware.

## Scope

- **In scope:**
  - Correct the pivot magnitude to `δ = (T/2)·tan(θ/2)` (the existing `T·sin²(θ/2)` is right only at θ=90°); ship the first-principles derivation in the ADR.
  - Store `δ` on the pivot-tagged edge (replacing `apply_pivot_compensations`' global `EDGES[v][w].distance_cm` mutation, the #44 mechanism); leave shared `EDGES` distances + `NEXT_HOP` byte-identical.
  - New `EdgeInfo.pivot_trim_cm` field: emitted per-cell by `header_writer` (reuse the existing override-field pattern), mirrored into the generated `routes.h` struct.
  - Firmware dispatch-time trims in `PathTracker`: dead-reckon approach trim + one-hop carried state + outgoing following-drive trim (per-primitive: `driveDist`, `FOLLOW_DIST` gate, `stopAtLine` arm-window; no-op `FOLLOW_INT`); `LINE_OVERSHOOT` advance reduced by `δ` for the line-incoming regime.
  - Exact sim mirror in `sim/runtime.py` of the dispatch-time trims, preserving drift ≈ 0 and `_POSITION_TOL_CM`.
  - Validator regime classification + fail-fast (degenerate θ, `δ ≥ edge length`, `δ >` overshoot budget, unsupported combos); keep finding #36 (single forward out-edge) — it is what makes the carried outgoing-trim target `w` unambiguous.
  - Reconcile the authoritative `LINE_SENSOR_AXLE_OFFSET_CM` value (sim `5.0` vs `Config.h` `12.02f`) — now load-bearing because the line-incoming regime reduces that overshoot.
  - Single-source chassis track: code-gen `Config::TRACK_MM` from `WRO2026/Config.h` into the planner; remove the stale `TRACK_CM = 15.0` literal/comment.
  - Regenerate `routes.h` / `routes.cpp` + toy goldens; update existing pivot tests; add δ, emission, validator, and sim-parity tests.
  - ADR documenting the two-sided model + derivation + the δ-on-pivot-edge rule; Roadmap bench-validation gate.
- **Out of scope:**
  - Heading-error modeling — the IMU turn loop (`TurnGyro`) owns final heading under the absolute-heading design; the planner leaves `heading_deg` unchanged. Residual scrub/heading coupling is bounded by next-node re-anchor and measured by the bench gate.
  - Per-chassis track parameterization — one committed `Config.h` value (competition chassis); a different test-rover track is a documented, separate concern.
  - Synthetic-node topology (node-split) and the pure runtime-trig maneuver — both rejected (node-split: `header_writer` emits only `field_data.NODES`, so graph-only nodes `KeyError`, plus multi-predecessor ambiguity; runtime-trig: not golden-testable, on-board float divergence risk).
  - The lateral re-anchor on a *chained* dead-reckon outgoing edge beyond what the two trims already cancel (zero residual is proven for a single pivot; multi-hop dead-reckon drift is the bench gate's concern).

## Decisions

- Precomputed-δ field, stored on the pivot edge, applied at dispatch — chosen over node-split and runtime-trig. Rationale: `/debate` Lean Yes (7.0); exact for the dead-reckon regime with zero residual and zero aliasing, all trig offline/golden-testable, no synthetic-node emitter/ambiguity defects, reuses the `EdgeInfo`-override pattern.
- `δ` keyed on the pivot edge `u→v` and **carried one hop** for the outgoing trim — NOT stored on `v→w`. A non-pivot route through `v→w` reads no trim, so #44 cannot recur.
- Magnitude `(T/2)·tan(θ/2)`, with the source-cited derivation in the ADR (per the repo's load-bearing-geometry bar).
- Trim is correctly inert on sensor-terminated edges (they self-correct on the line) — selectivity, not a gap.
- Line-incoming uses `LINE_OVERSHOOT` reduction (not an outgoing-distance edit) for the incoming side; valid while `δ ≤` the overshoot budget, else validator fail-fast.
- Track width code-gen from `Config::TRACK_MM` (single source of truth); consumed offline in the planner only — the firmware reads the emitted `δ`, not the track, so no runtime trig.
- Heading correction out of scope; bench-validation acceptance gate retained (braked-wheel pivots scrub).

## Acceptance Criteria

- [ ] AC1 — the planner derives chassis track from the single `Config::TRACK_MM` source; no second hard-coded track literal remains; editing `TRACK_MM` changes the emitted `δ` on the toy pivot fixture (verified there, since real `EDGES` is empty).
- [ ] AC2 — the δ helper returns `(T/2)·tan(θ/2)`: equal to the old value at θ=90°, correctly different at θ=60° (~0.289·T) and θ=120° (~0.866·T) (unit test with exact expected values); the ADR carries the derivation.
- [ ] AC3 — `δ` is stored on the pivot-tagged edge and emitted as `EdgeInfo.pivot_trim_cm`; the shared `EDGES[*][*].distance_cm` and `NEXT_HOP` bytes are identical to the pre-change tables, and a NON-pivot route through `v→w` drives the FULL distance (the #44 regression test).
- [ ] AC4 — for a dead-reckon-incoming pivot, the firmware + sim trim both the approach and the following drive; the axle lands on `w` within sim `_POSITION_TOL_CM`, drift ≈ 0, zero residual.
- [ ] AC5 — for a line-incoming pivot, the firmware + sim reduce the `LINE_OVERSHOOT` advance by `δ` (incoming) and apply the outgoing trim; the axle lands within tolerance, CTPS drift ≈ 0 holds, and a non-pivot route through the same line node is unaffected.
- [ ] AC6 — the validator fails fast, with a precise error, on: near-180° pivots; `δ ≥ edge length` on a dead-reckon side; `δ >` the `LINE_SENSOR_AXLE_OFFSET_CM` budget on a line-incoming side; and unsupported incoming×outgoing combos (validator unit tests).
- [ ] AC7 — the OUTGOING trim applies only on distance-terminated following drives (no `δ` subtraction on `FOLLOW_INT`/`STOP_AT_LINE` outgoing edges, which self-correct on the line); the incoming side uses approach-trim (dead-reckon) or `LINE_OVERSHOOT` reduction (line). Verified by a dispatch test.
- [ ] AC8 — `build_routes.py` regenerates `routes.h/.cpp` + toy goldens and `git diff --exit-code` is clean in CI.
- [ ] AC9 — all existing pivot tests updated to the corrected magnitude; full `pytest`, host `ctest`, and `arduino-cli` compile all pass.
- [ ] AC10 — an ADR documents the two-sided model, the derivation, and the δ-on-pivot-edge rule, indexed in `DECISIONS.md`; the Roadmap carries the bench-validation gate.

## Data Model

Geometry (axle-center reference; single-wheel pivot about the **braked wheel's ground contact**, so the axle traces an arc of radius `T/2`; incoming/outgoing edges are straight and meet at `v` with interior turn angle θ):

```
δ = (T/2) · tan(|θ|/2)          # per-edge forward over-travel, identical both sides

# Dead-reckon incoming (exact, zero residual):
#   drive u→v shortened to L_uv − δ   (stop at A = v − δ·ĥ_in)
#   pivot at A → axle to B = v + δ·ĥ_out
#   drive v→w shortened to L_vw − δ   → lands exactly on w
#
# Line incoming (re-anchors on the junction line at v):
#   LINE_OVERSHOOT advance reduced from offset → offset − δ   (axle settles δ short of v)
#   pivot, then outgoing trim L_vw − δ   → lands on w (valid while δ ≤ overshoot offset)
```

`δ` is a single scalar per pivot (same magnitude both sides), stored on the pivot-tagged edge `u→v` as a new attribute → emitted field:

```cpp
struct EdgeInfo {            // generated into routes.h by header_writer
    EdgeType type;
    float    distance_cm;    // UNCHANGED — true geometric distance, shared
    float    heading_deg;    // UNCHANGED
    LineMode line_mode_override;
    BlindMode blind_mode_override;
    ProjectionMode projection_mode_override;
    TurnMode pre_turn_mode_override;
    DirectionMode direction_override;
    float    pivot_trim_cm;  // NEW: δ on the pivot-tagged edge; 0.0 on non-pivot edges
};
```

Regime selection matrix (incoming type × outgoing type), per pivot — drives validator acceptance and dispatch behavior:

```
incoming \ outgoing | LINE / STOP_AT_LINE (sensor) | BLIND-plain / PROJECTION (distance)
--------------------+------------------------------+------------------------------------
LINE (re-anchor)    | overshoot-reduce; outgoing    | overshoot-reduce in; outgoing
                    | trim no-ops (self-corrects @w)| drive trimmed by δ
BLIND/PROJECTION    | approach trim; outgoing no-ops| approach trim + outgoing drive trim
                    | (self-corrects @w)            | (exact, zero residual)
```

`pivot_trim_cm = 0.0` is the default on every non-pivot edge (mirrors the existing `*_override` default-cell pattern; no struct shape change beyond the one field).

## API / Pipeline Contracts

```
build_routes._run() order (unchanged shape):
  validate(field_data)                          # T2: regime + fail-fast (needs track for δ)
  → track_cm = parse Config::TRACK_MM(Config.h) # T1: single source
  → graph = build_graph(field_data)
  → compute_all_pairs_next_hop(graph, track_cm) # T1: apply_pivot_compensations now STORES
  →   apply_pivot_compensations → graph.edges[u][v]['pivot_trim_cm'] = δ   (no distance mutation)
  → emit_routes(...)                            # T3: emit pivot_trim_cm field
  → render_map(...)
```

Firmware/sim dispatch contract (T4 defines, T5 mirrors): when the current hop's edge carries `pivot_trim_cm = δ > 0`: (a) trim the approach (dead-reckon) or reduce `LINE_OVERSHOOT` by δ (line); (b) latch δ as carried state; (c) on the next hop's drive, subtract δ if it is distance-terminated, else no-op. `EDGES.distance_cm` is never altered.

## Error Handling

- **Degenerate pivot (θ→180°):** validator fail-fast — never a silent clamp producing a zero-length drive.
- **`δ ≥ edge length` (dead-reckon side):** validator fail-fast naming the pivot/edge — a trim cannot exceed the drive.
- **`δ >` the line-incoming overshoot budget:** validator fail-fast against the MIN of FRONT (`12.02`) and REAR (`7.68`) `LINE_SENSOR_AXLE_OFFSET_CM` — the overshoot can't absorb the incoming δ, and edge direction (front/rear) is a runtime decision the offline validator can't see.
- **Unsupported incoming×outgoing cell:** validator fail-fast naming the combo.
- **`EdgeInfo` field-order drift (emitter tuple vs C++ struct positional init):** add a generated static_assert or field-count check so a one-off mismatch fails the compile, not silently reads a wrong float.
- **`Config::TRACK_MM` unparseable/missing:** `build_routes` aborts loudly; no fallback to a stale literal.
- **Carried-state edge cases:** pivot edge is the final hop (no following drive — incoming trim only); mission abort mid-pivot must clear the latched δ.
- **Sim divergence from firmware:** parity test fails the build (drift ≈ 0 / `_POSITION_TOL_CM`); never relax the tolerance or re-add per-junction offsets.

## Testing Strategy

**Levels:** Unit (pytest), Golden (routes bytes), Sim-parity (pytest), Host compile (ctest), Firmware compile (arduino-cli), plus a manual bench gate (documented, not automated).

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | δ helper at θ=0/60/90/120 | Unit | δ=(T/2)tan(θ/2); =old at 90°, differs at 60/120 |
| TC2 | θ→180 / δ≥edge / δ>overshoot | Unit (validator) | `FieldDataError`, precise message each (helper itself clamps; validator raises) |
| TC3 | Track code-gen: build with edited `TRACK_MM` | Integration | Emitted δ on toy fixture reflects new value; no second track literal exists |
| TC4 | `δ` stored on pivot edge; non-pivot cell keeps raw distance | Golden | In a toy fixture with both a pivot route and a non-pivot route through `v→w`: `pivot_trim_cm` set on the pivot edge only; the non-pivot `v→w` `distance_cm` equals the raw geometry (not shortened) and `NEXT_HOP` matches the no-trim table |
| TC5 | Non-pivot route over `v→w` | Sim | Drives FULL original distance — no under-drive (the #44 regression) |
| TC6 | Dead-reckon-incoming pivot end-to-end | Sim | Both drives trimmed; lands on node, drift ≈ 0, zero residual |
| TC7 | Line-incoming pivot | Sim-parity | Overshoot reduced by δ + outgoing trim; axle on node within `_POSITION_TOL_CM`; non-pivot route unaffected |
| TC8 | Sensor-terminated edge with a pivot tag | Dispatch unit | No δ subtraction on `FOLLOW_INT`/`STOP_AT_LINE`; self-corrects at next junction |
| TC9 | Existing pivot goldens (7.5/15 at 90/180) | Unit | Updated to the corrected magnitude / new storage; still pass |
| TC10| `EdgeInfo` field-order guard | Compile | static_assert / field-count check fails on a deliberate mismatch |
| TC11| `build_routes` then `git diff --exit-code` | CI | Clean (routes + goldens fresh) |

**Test data:** extend `toy_field_data.py` with one dead-reckon-incoming pivot and one line-incoming pivot; golden rebaseline via the `_toy_field_source.py` anchor. **Run:** `uv run pytest tools/path_planner/tests/`, host `ctest`, `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Track code-gen + δ formula + store-on-pivot-edge | — | high | `tools/path_planner/build_routes.py`, `tools/path_planner/planner.py` | Parse `Config::TRACK_MM` from `WRO2026/Config.h` at build time; remove the stale `TRACK_CM=15.0` literal/comment (assert-equal if a constant is kept). In `compute_pivot_compensation`, compute `δ=(track/2)·tan(|θ|/2)`; change `apply_pivot_compensations` to STORE `δ` on the pivot edge (`edge['pivot_trim_cm']`) instead of mutating the following edge's `distance_cm`; keep the call in `compute_all_pairs_next_hop`. Abort loudly if `TRACK_MM` is missing. Satisfies AC1/AC2; enables AC3. |
| T2 | Validator regime + fail-fast | T1 | med | `tools/path_planner/validator.py` | Classify each pivot by (incoming×outgoing) edge type; compute δ (needs track from T1) and fail fast on θ→180°, `δ≥edge length` (dead-reckon side), `δ>` the line-incoming overshoot budget (use the MIN of FRONT `12.02` and REAR `7.68` `LINE_SENSOR_AXLE_OFFSET_CM`, since edge direction is a runtime decision the offline validator can't see), and unsupported cells, with precise `FieldDataError`s. Keep finding #36. Runs pre-`build_graph`. Satisfies AC6. |
| T3 | Emit `pivot_trim_cm` field | T1 | med | `tools/path_planner/header_writer.py`, `tools/path_planner/compiler.py` | Add `pivot_trim_cm` to the emitted `EdgeInfo` struct + every edge cell (default `0.0`), via BOTH `_edge_cell` AND `_none_edge_cell` (the NONE-cell emitter also lists all fields positionally and must gain the trailing `0.0`) plus `_resolve_overrides`; add a generated `static_assert`/field-count guard against positional-init drift. Ensure the compiler keeps the pivot tag reaching the right hop. Enables AC3/AC10-guard. |
| T4 | Firmware dispatch trims + carried state + overshoot reduction | T8 | high | `WRO2026/PathTracker.cpp`, `WRO2026/PathTracker.h` | Read `edge.pivot_trim_cm`; dead-reckon approach trim (`driveDist`/`stopAtLine` arm-window); latch δ as a one-hop carried member; apply the outgoing following-drive trim per-primitive (`driveDist`, `FOLLOW_DIST` gate) and no-op on `FOLLOW_INT`; reduce the `LINE_OVERSHOOT` advance by δ for line-incoming. Reconcile + use the authoritative FRONT and REAR `LINE_SENSOR_AXLE_OFFSET_CM` (front: sim `5.0` vs Config `12.02f`; rear: sim `5.0` vs Config `7.68f`). Clear δ on abort/final hop. Move-only RAII; no `#include` of `.cpp`. Enables AC4/AC5/AC7. |
| T5 | Sim mirror of dispatch trims | T4 | high | `tools/path_planner/sim/runtime.py` | Reproduce T4's approach/overshoot/outgoing trims and the carried state byte-for-byte; apply only on distance-terminated hops; preserve drift ≈ 0 and `_POSITION_TOL_CM`. Satisfies AC4/AC5 (sim side). |
| T6 | Planner + emitter tests | T1, T3 | low | `tools/path_planner/tests/test_planner.py`, `tools/path_planner/tests/test_header_writer.py` | REWRITE two existing full-flow tests whose assertion subject INVERTS under store-on-pivot-edge: `test_compute_all_pairs_next_hop_compensates_input_graph_in_place` (test_planner.py:397) and `test_pivot_compensation_reaches_emitted_distance` (test_header_writer.py:600) currently assert B→C is shortened to 22.5cm (`22.500000f`); the fix keeps B→C at 30.0 and stores δ on the pivot edge, so each must instead assert B→C `distance_cm` is UNCHANGED and that `pivot_trim_cm` is populated on the pivot-tagged edge (A→B). Update remaining 7.5/15 numeric goldens to corrected magnitude/new storage. Add TC1, TC3 (track edit → δ), TC4 (storage + non-pivot cell unchanged), TC10 guard. Satisfies AC2/AC3/AC9. |
| T7 | Validator tests | T2 | low | `tools/path_planner/tests/test_field_data.py` | Add TC2: fail-fast on θ→180°, δ≥edge, δ>overshoot, unsupported combos. Satisfies AC6. |
| T8 | Regenerate routes + goldens | T1, T3 | low | `WRO2026/routes.h`, `WRO2026/routes.cpp`, `tools/path_planner/tests/golden/routes_toy.h`, `tools/path_planner/tests/golden/routes_toy.cpp` | Run `build_routes.py` + golden rebaseline so `routes.h` carries the new `EdgeInfo` field; commit. Satisfies AC8. |
| T9 | Sim-parity tests | T5 | med | `tools/path_planner/tests/test_sim_runtime.py` (or existing sim test module) | Add TC5 (#44 regression), TC6 (dead-reckon end-to-end), TC7 (line-incoming parity), TC8 (sensor-edge no-op), drift ≈ 0. Satisfies AC4/AC5/AC7. |
| T10 | ADR + Roadmap bench gate | — | low | `docs/decisions/035-two-sided-pivot-compensation.md`, `DECISIONS.md`, `Roadmap.md` | Write the ADR (two-sided model, the `(T/2)tan(θ/2)` derivation, δ-on-pivot-edge / no-global-edge rule, the regime matrix); index in `DECISIONS.md`; add the B3.2-style bench-validation gate to the Roadmap. Verify ADR 035 is unclaimed by a parallel branch (don't replicate the existing ADR-022 number collision). Strip em-dashes / AI-tell words from the ADR commit + PR per the user's global CLAUDE.md. Satisfies AC10. |

Parallelizable from the start: **T1** and **T10** (distinct files, no blockers). T2/T3 follow T1; T8 gathers T1+T3; T4 needs the regenerated struct (T8); T5 mirrors T4; tests trail their impl tasks. Note T4 is `Blocked By: T3, T8` so the firmware compiles against the regenerated `routes.h` field.

## Notes for Implementer

- The whole point: **no route-specific distance on a shared `EDGES` row, and δ lives on the pivot edge, not the following edge.** If you write δ onto `v→w` or into its `distance_cm`, you've recreated #44.
- The trim is **correctly inert** on sensor-terminated edges — they re-anchor on the physical line. Do not "fix" the no-op; verify the dispatch only subtracts δ on distance-terminated primitives (TC8).
- `δ` is the same magnitude on both sides; one `pivot_trim_cm` field + one carried-state member suffices. The carried state is the one bit of "memory" that lets the memoryless table apply the outgoing trim correctly.
- Zero-residual holds only when BOTH sides are corrected (dead-reckon: approach trim; line: overshoot reduction). A line-incoming pivot where `δ >` the overshoot budget cannot stop short → that's the validator fail-fast, not a silent residual.
- The `LINE_SENSOR_AXLE_OFFSET_CM` mismatches are now **load-bearing** (the overshoot reduction subtracts from them): FRONT sim(`5.0`)/`Config.h`(`12.02f`) AND REAR sim(`5.0`)/`Config.h`(`7.68f`). Reconcile which is authoritative for both before T4/T5; do not silently trust either.
- Do not relax sim `_POSITION_TOL_CM` or re-introduce per-junction offsets (a prior over-advance broke CTPS playback). Each hop nets exactly the edge length.
- `routes.h/.cpp` are SHA-stamped; CI runs `build_routes` then `git diff --exit-code` — T8 must land with the emitter edits.
- C++ conventions: strict OOP, move-only RAII builders, never `#include` a `.cpp`; extend host-compile mocks for the new `PathTracker` carried-state surface.
- Rollback: inert while `field_data` `EDGES` is empty (no pivots), so it merges ahead of the first pivot; reverting the emitter + regenerating routes restores the prior tables.
- The bench gate (T10) is a downstream hardware sign-off, not a merge blocker — but a pivot must not be trusted in a mission until it passes (braked-wheel pivots scrub; the closed-form is an idealization).
