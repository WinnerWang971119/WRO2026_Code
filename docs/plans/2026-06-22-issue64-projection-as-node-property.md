# Model projection as a node property, not an edge type (issue #64) — Plan

**Goal:** Replace the `EDGE_PROJECTION` edge type with a `projection` **node** type, so the
"no-junction / dead-reckon / re-anchor-here" semantics live on the node and every per-hop
behavior falls out of `(edge_type, destination_node_type)` — collapsing the goal-relative
`next_hop_is_projection` runtime peek (transcribed across firmware, compiler, and sim) into a
single local lookup, and removing the editor's off-model `ProjectionDelta` store.

**Approach:** Add `PROJECTION` to the node-type taxonomy (mutually-exclusive 4th type, mirroring
the existing on-model `RESET` precedent). Edges become **LINE or BLIND only**: a LINE edge always
line-follows, a BLIND edge never does, and the **destination node type** decides how the move
terminates. Keep the `resolveStopAtLine` / `resolve_stop_at_line` resolver **route-agnostic**
(ADR-038's design) — only the caller swaps `auto_rule_active` from "next hop is a projection
edge" to the local "this BLIND edge's destination is a projection node," so the 936-row CSV
truth-table parity is untouched. Deliver the node-property **core** first (green), then add the
`line–proj–line` pass-through as a final, isolated phase (the riskiest piece: FSM lookahead ×
pivot-trim × re-anchor). Document it in one new ADR that supersedes the ADR-038/039 auto-rule
sub-decisions and amends ADR-023 (pass-through) + ADR-044 (pivot regime).

## Scope

- **In scope:**
  - New `projection` node type in the `field_data` schema, validator, graph builder, emitter, and `routes.*` (`NodeType::PROJECTION`).
  - Edge-type taxonomy reduced to `LINE` / `BLIND`; per-hop primitive derived from `(edge_type, destination_node_type)`.
  - Local auto-seek: a BLIND edge whose destination is a projection node seeks + re-anchors (full advance). Replaces the goal-relative `next_hop_is_projection` peek in firmware, compiler, and sim.
  - **Full clean removal** of `EDGE_PROJECTION`, `EdgeType::PROJECTION`, `next_hop_is_projection`, `ProjectionMode`, `PipelineConfig.projection`, and the `projection_mode_override` `EdgeInfo` field.
  - Validator `_regime` re-derived from `(edge_type, destination_node_type)` (the LINE→projection case flips to the `distance` regime).
  - `line(A) – projection(B) – line(C)` **pass-through**: routed-through B collapses to one continuous A→C line-follow with a dead-reckon Localization advance at B; B stops only when it is the explicit goal.
  - RETURN_TO_NODE handling for projection-node targets (dead-reckon return, no junction-seek).
  - Editor: projection becomes an **on-model** node type via the existing node-type combobox; delete the off-model `ProjectionDelta`, its two commands, the 5 edge scanners, the "+ add projection" button, and the LibCST projection writers.
  - Migration: add a projection node to the **toy** field for golden coverage; migrate all unit fixtures; convert the commented `field_data.py` demo. Update all Python + host-C++ tests.
  - One new ADR (supersedes the ADR-038/039 auto-rule sub-decisions; amends ADR-023 and ADR-044). Update `ARCHITECTURE.md`, `DECISIONS.md`, `Roadmap.md` (Risk #9), and the editor/field_data docs.

- **Out of scope:**
  - Re-authoring the **real** WRO field (`field_data.json`) with projection nodes — that's Phase-3 real-field encoding (P3.5). We touch only the toy field. (Keeps production `routes.cpp` node indices stable, so host C++ tests that link the real routes don't renumber — issue #91.)
  - Reverse (REAR-traversal) projection/pivot handling beyond what already exists (#54).
  - The position-aware come-back recovery (#62) and any change to the `Drive.stopAtLine` primitive itself.
  - A blind-decay routing change — by decision below, a projection node is **not** an anchor, so `planner._ANCHOR_NODE_TYPES` is unchanged. (This disclaims only `_ANCHOR_NODE_TYPES`; the per-hop cost consequence of projection-as-node is *documented* in Decisions, not waved off.)

## Migration: old projection-edge topology → new node model

There are **no live projection edges** in the real or toy field today — the only existing
instances are the commented-out `field_data.py` demo (`A→P1`) and the test fixtures.

- **Topology rewrite rule.** An old projection scenario `blind(u) → junction(v) → [projection edge v→w] → waypoint(w)` rewrites to: drop the standalone projection edge; the waypoint `w` becomes a **projection node**; the hop into it becomes a **BLIND edge → projection node** (`Blind→Proj` = auto-seek + re-anchor). Junction `v` survives as a normal `line` node **only if** it has independent purpose (referenced by another edge/task); otherwise it collapses and the BLIND edge originates directly from `u`.
- **Seek relocates (resolves the moved-re-anchor concern).** The auto-seek's physical location MOVES. Old model: the seek fired on the BLIND hop and re-anchored at junction `v`. New model: the `Blind→Proj` hop seeks the **projection node's own line** at the approach into `w`, not the preceding junction. This is a deliberate behavioral relocation of where re-anchoring happens on the field — the bench gate must exercise the new maneuver.
- **Concrete migration surface (disposition each):** (a) the commented `field_data.py` demo is **deleted**, not rewritten (T3); (b) the toy field gains a fresh `Blind→Proj` projection node plus a `line–proj–line` triple for golden coverage (T11); (c) fixtures are rewritten to the node form (T12). No real-field change.

## Decisions

- **Encoding:** `projection` is a mutually-exclusive 4th node type (`NodeType { LINE, BLIND, RESET, PROJECTION }`), not a `no_marker` bool — mutual exclusivity makes a flag redundant. Mirrors the on-model `RESET` precedent (a node kind that drives a local arrival interlude with no NEXT_HOP peek). — *Fable + Codex both endorsed.*
- **What a projection node is:** on a line but **not** at a junction. You can line-follow across it; you can only dead-reckon (by encoder distance) *to* it (no crossing to sense). A `line` node is on a line AND at a junction; a `blind` node is not on a line at all. The line *passes through* the projection node (longitudinally detectable as a crossing), so a BLIND→projection hop has a real line to stop on — there is just no *junction* marker to identify it, hence dead-reckon for identity/distance.
- **Per-hop primitive table** (source node type never changes the primitive; only edge type + destination type do):

  | Edge | → line node | → projection node | → blind node |
  |------|-------------|-------------------|--------------|
  | **LINE** | line-follow until junction (+ re-anchor) | line-follow + dead-reckon to distance | **illegal** (validator rejects) |
  | **BLIND** | dead-reckon (seek only via explicit override) | **auto-seek stop-at-line + re-anchor** | pure dead-reckon |

- **Blind→line is unchanged:** only BLIND→projection gets the new local auto-seek. Blind→line keeps today's dead-reckon default (seek via call-flag / per-edge override only). Smallest blast radius.
- **Resolver stays route-agnostic:** `resolveStopAtLine` / `resolve_stop_at_line` and the 936-row CSV truth-table are frozen. Only the **caller** changes — `auto_rule_active = (edge is BLIND) ∧ (destination node is PROJECTION)`, read locally. — *the parity-preservation spine; both consultants endorsed.*
- **Seek override surface kept, retargeted:** call-flag ON/OFF, per-edge `stop_at_line_override`, legacy `BlindMode::STOP_AT_LINE`, and the field-wide `BLIND_PROJECTION_AUTO_SEEK` toggle all survive; only the auto-rule **trigger** changes from "next-hop-is-projection-edge" to "destination-is-projection-node."
- **Motion override dropped:** `ProjectionMode` / `projection_mode_override` / `PipelineConfig.projection` deleted. A LINE→projection follow takes its motion profile from the LINE edge's existing `line_mode_override`.
- **Blind→projection re-anchor = full advance:** on the seek-stop, set current node = the projection node and clear the operator offset (`onIntersection`-style), treating the seek-stop as the proj node's mapped position.
- **Pass-through Localization = dead-reckon advance at B:** the continuous A→C follow emits an `onDeadReckonedAdvance` at B mid-follow (no stop), preserving the one-advance-per-node invariant, then re-anchors at C's junction.
- **Projection node is NOT a blind-decay anchor (conservative):** `planner._ANCHOR_NODE_TYPES` stays `{line, reset}`. Rationale: a projection node's longitudinal position is always dead-reckoned (the firmware "full advance" is a pragmatic runtime re-anchor, but the offline routing-cost model treats every projection arrival as still-uncertain). **This is a deliberate decision — do not "fix" it by adding projection to the anchor set.**
- **Projection routing cost follows the incident edge type.** A LINE→projection edge stays factor-1 (identical to the retired projection edge — the LINE carve-out holds by construction); a BLIND→projection edge gets the full blind-decay multiplier (correct — it is a blind move). Combined with projection-not-an-anchor, distance-to-nearest-anchor lengthens for any node reached *through* a projection node. **Production routing is unchanged this phase** — the real field has zero projection nodes; only the toy field gains one. **P3.5 hand-off:** when the real field is re-encoded with projection nodes, this cost change becomes live and that phase must re-validate real-field `NEXT_HOP` and choose LINE-vs-BLIND incidence intentionally per projection approach. Toy-field `NEXT_HOP`/weight changes are intentional and asserted by TC20.
- **RETURN_TO_NODE to a projection node = dead-reckon return** (no junction-seek; there is no junction to detect). `_returnUsesLineSeek` / `is_line_backed` returns false for projection nodes.
- **Pass-through is automatic + in scope now:** no flag; at a non-goal projection middle node with both flanking edges LINE and collinear (ADR-023 heading check), the inbound LINE edge is **distance-terminated at B**, and the FSM fires the pass-through off that **distance gate** — NOT `Intersection.isAtIntersection()`, which never triggers at a projection node — emitting `onDeadReckonedAdvance` at B mid-follow (no stop) and re-anchoring at C's junction. A projection node flanked by any BLIND edge is a normal stop. This pass-through collapse is the *entire* scope of issue-comment-3's "ignored unless fired": a `blind→proj` or `proj→blind` projection node is always materialized (stopped at / dead-reckoned), never ignored.
- **Delivery is staged:** node-property core lands green before the pass-through phase. The pass-through is the only phase that touches the FSM lookahead × pivot-trim × re-anchor interaction.
- **One ADR:** supersedes the ADR-038/039 auto-rule sub-decisions and the implicit projection-as-edge convention; **amends** ADR-023 (narrow its pass-through language to line-proj-line) and ADR-044 (regime from edge+destination).
- **No UI/UX design gate:** the editor change is a node-type combobox value + a node color, following the existing RESET-node UX precedent exactly — no new screen or visual flow to design. (Phase-1.5 gate intentionally skipped.)

## Acceptance Criteria

- [ ] `field_data` accepts `NODE_TYPES[x] = "projection"`; the validator rejects an unknown node type, a LINE edge into a blind node, and a projection node as the start node.
- [ ] `EDGE_PROJECTION`, `EdgeType::PROJECTION`, `next_hop_is_projection`, `ProjectionMode`, `PipelineConfig.projection`, and `projection_mode_override` are gone from the entire tree (Python + C++ + generated `routes.*` + goldens). A repo-wide search returns zero references.
- [ ] The per-hop primitive matches the table above in all three transcriptions (compiler, sim, firmware), verified by tests for each `(edge_type, dest_node_type)` cell.
- [ ] The auto-seek for a BLIND→projection hop fires from a local destination-node read with **no** `NEXT_HOP[next][goal]` lookup; the resolver's output domain (the 936-row stop-at-line CSV) is unchanged and still passes (only the caller's `auto_rule_active` input is computed differently) (TC13 / TC13-CPP).
- [ ] A BLIND→projection seek re-anchors via full advance (current node = projection node, offset cleared); a LINE→projection follow dead-reckons (`onDeadReckonedAdvance`).
- [ ] `validator._regime` returns `distance` for a LINE→projection leg, `line` for LINE→line, `sensor` for BLIND→projection; pivot-trim budgeting for a pivot approaching a projection node draws δ from the follow distance, not the overshoot budget.
- [ ] `line(A)–projection(B)–line(C)` routed to a goal past B emits one continuous A→C line-follow with a dead-reckon advance at B; `goTo(B)` stops at B. Collinearity + both-flanking-LINE gate enforced; a BLIND-flanked projection node is a normal stop.
- [ ] In the editor, setting a node's type to `projection` via the combobox persists through save/reload; the off-model `ProjectionDelta`, the two projection commands, the 5 scanners, the "+ add projection" button, and the LibCST projection writers no longer exist. Rename/delete of a projection node works through the normal on-model cascade.
- [ ] The toy field has a projection node; `routes_toy.h` golden regenerates and the host C++ tests pass against it. Production `routes.cpp` node indices are unchanged.
- [ ] Full `uv run --extra dev pytest tools/path_planner/tests/` and the full host C++ ctest both pass. `build_routes.py` output is committed (CI `git diff --exit-code` clean).
- [ ] One new ADR added + indexed in `DECISIONS.md`; ADR-038/039 marked superseded (auto-rule sub-decision), ADR-023/044 amended/cross-linked.

## Data Model

**Node-type taxonomy** (`field_data` strings ↔ generated C++ enum):

```
NODE_TYPES[name] ∈ {"line", "projection", "blind", "reset"}     # mutually exclusive
```
```cpp
enum class NodeType : uint8_t { LINE = 0, BLIND = 1, RESET = 2, PROJECTION = 3 };  // append PROJECTION
```

**Edge-type taxonomy** (reduced):

```
edge["type"] ∈ {EDGE_LINE, EDGE_BLIND}      # EDGE_PROJECTION deleted
```
```cpp
enum class EdgeType : uint8_t { NONE = 0, LINE = 1, BLIND = 2 };  // PROJECTION = 3 removed
```

**`EdgeInfo` struct** — `projection_mode_override` removed (8 → 7 fields):

```cpp
struct EdgeInfo {
    EdgeType       type;
    float          distance_cm;
    float          heading_deg;
    LineMode       line_mode_override;
    BlindMode      blind_mode_override;
    DirectionMode  direction_override;
    StopAtLineMode stop_at_line_override;
};   // ProjectionMode projection_mode_override; deleted
```

Deleted types: `ProjectionMode` (enum), `PipelineConfig.projection` (field). `PipelineConfig` drops to its 4 surviving motion members.

## Contracts & Interfaces

Single source of truth for every cross-task seam. A task that touches a seam conforms to the
exact name/shape here; it does not redefine it.

### Shared types / values

- **Node-type vocabulary** `{"line", "projection", "blind", "reset"}` — owner **T1** (graph_builder) + **T2** (validator); consumers: field_data (T3), emitter (T6), editor model (T15), fixtures (T11/T12). The C++ `NodeType` enum value `PROJECTION = 3` — owner **T6**; consumers: FieldMap (T8), PathTracker (T10), host tests (T14).
- **Per-hop primitive table** (`(edge_type, dest_node_type) → primitive`, as in Decisions) — the canonical behavior contract; consumers: compiler (T4), sim (T5), firmware (T10). All three must agree cell-for-cell.
- **`auto_rule_active` contract** — the bool fed to the resolver = `(edge.type == BLIND) ∧ (dest node is PROJECTION) ∧ BLIND_PROJECTION_AUTO_SEEK`. Computed by the **caller**, locally, with no NEXT_HOP peek. Owner of the contract doc: **T9** (StopAtLinePolicy.h comment); consumers: T4, T5, T10. The resolver body + the 936-row CSV are **frozen**.
- **`EdgeInfo` 7-field layout** (above) — owner **T6** (emitter `_edge_cell` + `_none_edge_cell`); consumers that hand-write positional inits: `FieldMap::noEdge()` (T8), generated `routes.cpp` (T-REGEN), any host-test `EdgeInfo` init (T14). No `offsetof` static_assert exists today, so a missed positional init only warns under `--warnings all` — coordinate all sites.

### Signatures

- `validator._regime(...) → "line" | "sensor" | "distance" | None` — **TARGET shape** (the current signature is `_regime(edge_type, primitive_mode, stop_override, auto_rule_active)`). The new shape takes the **destination node type** (the LINE→projection=`distance` case needs it) and drops the per-edge `primitive_mode` arm for the retired projection mode. Owner **T2**; consumer: `_validate_pivot_turns`.
- `graph_builder.is_projection_node(graph, n) → bool` (new local helper replacing `next_hop_is_projection`) — owner **T1**; consumers: compiler (T4), sim (T5), validator (T2).
- `FieldMap::getNodeType(id) → NodeType` (exists) + the local auto-rule read in PathTracker — owner **T8/T10**.
- `PathTracker::_peekNextNextHop()` (private, 3 guards: not-final / `NODE_NONE` / self-loop) — **frozen seam** touched by two phases: T10 **removes** its auto-seek caller (`_nextHopIsProjection`, now local) and T19 **adds** a pass-through-lookahead caller. Both conform to the same 3 guards; the guards must stay self-contained after the auto-seek caller is removed. Owner **T10** (split), consumer **T19**.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `tools/path_planner/graph_builder.py` | T1 | T2, T4, T5, T6 |
| `tools/path_planner/validator.py` | T2 | — |
| `tools/path_planner/pipeline_config.py` + `field_data.py` | T3 | T6 |
| `tools/path_planner/compiler.py` | T4 (core), T20 (pass-through) | — |
| `tools/path_planner/sim/runtime.py` | T5 (core), T20 (pass-through) | — |
| `tools/path_planner/header_writer.py` | T6 | T-REGEN |
| `WRO2026/routes.h` + `routes.cpp` + goldens | T-REGEN | T8, T10, T14 |
| `WRO2026/FieldMap.h` | T8 | T10 |
| `WRO2026/StopAtLinePolicy.h` | T9 | T10 |
| `WRO2026/PathTracker.h` + `.cpp` | T10 (core), T19 (pass-through) | — |
| `tools/path_planner/field_data_toy.json` + toy fixtures | T11 | T13, T-REGEN |
| `tools/path_planner/tests/fixtures/*` | T12 | T13 |
| `tools/path_planner/editor/model.py` | T15 | T16, T17 |
| `editor/app.py` + `commands.py` + `projection_edges.py` + `ast_rewrite.py` | T16 | T17, T18 |
| `editor/canvas.py` + `panels.py` + `dialogs.py` | T17 | T18 |
| ADR + `DECISIONS.md` + `ARCHITECTURE.md` + `Roadmap.md` + docs | T22 | — |

### Naming (must match across tasks)

- Node-type string: exactly `"projection"` (lowercase). C++ enum: `NodeType::PROJECTION`.
- The field-wide toggle keeps its name `BLIND_PROJECTION_AUTO_SEEK` (semantics retargeted, name unchanged) — `field_data.json` key `"blind_projection_auto_seek"`, `Routes::BLIND_PROJECTION_AUTO_SEEK`.

## Error Handling

- **Validator rejections (loud, offline):** unknown node type; `LINE` edge with a `blind` destination; a `projection` (or `line`) node as the start node; a `projection` node with conflicting reset spec; a `line` node with **zero** incident LINE edges (warn — a junction must sit on a line edge). A `projection` node is exempt from that warning when it has a `Blind→Proj` in-edge (it's blind-seek-reachable without a LINE edge); warn only if it is otherwise unreachable. Each raises `FieldDataError` with a specific message (warnings for the reachability cases), per ADR-029.
- **BLIND→projection seek miss (runtime):** a no-line fault flows through the existing shared `DRIVE_FAULT` guard — hard fault, identical to every other `stopAtLine` (ADR-039). No special branch.
- **Pass-through guard failure:** if a `line–proj–line` candidate is not collinear or a flank isn't a LINE edge, fall back to the normal per-hop path (stop at B). Never silently merge a non-qualifying triple.
- **Pivot into a projection node:** the LINE→projection `distance` regime budgets δ against the follow distance; the validator hard-rejects δ > follow distance (mirrors the existing ADR-044 budget check).

## Testing Strategy

**Levels:** Unit (validator, graph builder, compiler, emitter, planner), Integration (sim FSM end-to-end + host-C++ resolver/FSM mirrors), Golden (emitter routes goldens), Editor (Tk headless).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `NODE_TYPES[x]="projection"` accepted; unknown type rejected | Unit (validator) | Accept projection; `FieldDataError` on unknown |
| TC2 | LINE edge → blind node | Unit (validator) | Rejected with the line-into-blind message |
| TC3 | projection node as start node | Unit (validator) | Rejected |
| TC4 | reachability warn: line node w/ zero LINE edges; projection node w/ only a BLIND in-edge | Unit (validator) | Warns for the line node; does NOT warn for the blind-seek-reachable projection node |
| TC5 | `_regime`: LINE→line=line, LINE→proj=distance, BLIND→proj=sensor | Unit (validator) | Each leg classified per the table |
| TC6 | LINE→projection compiles to `follow_line_to_distance` | Unit (compiler) | follow-dist primitive; no junction wait |
| TC7 | Proj→line compiles to follow-to-intersection | Unit (compiler) | line-follow until junction |
| TC8 | BLIND→projection auto-seek fires from local dest read | Unit (compiler+sim) | `stop_at_line` emitted; no `NEXT_HOP[next][goal]` access |
| TC9 | Stop-at-line 936-row CSV truth-table (resolver OUTPUT domain) | Unit (sim+C++) | Identical to pre-refactor (only the caller's `auto_rule_active` input changes, not the resolver output); TC13 / TC13-CPP pass |
| TC10 | BLIND→projection seek re-anchors full-advance; LINE→proj dead-reckons | Integration (sim) | node=proj + offset cleared on seek; `on_dead_reckoned_advance` on follow |
| TC11 | `line–proj–line` routed past B | Integration (sim) | continuous A→C, dead-reckon advance at B, re-anchor at C |
| TC12 | `goTo(B)` where B is projection | Integration (sim) | stops at B by distance |
| TC13 | BLIND-flanked projection node | Integration (sim) | normal stop at B (no pass-through) |
| TC14 | RETURN_TO_NODE to a projection node (regression guard) | Unit (sim+C++ policy) | dead-reckon return, no junction-seek — verifies projection is NOT line-backed by default; no projection special-case branch added |
| TC15 | Emitter: NodeType::PROJECTION emitted; EdgeType::PROJECTION / ProjectionMode / projection_mode_override absent | Golden (header_writer) | `routes_toy.h` golden matches; `EdgeInfo` is 7 fields |
| TC16 | Host FSM: firmware LINE→proj distance gate + BLIND→proj seek + full-advance | Integration (host C++) | mirrors compiler/sim; `getNodeType` read local |
| TC17 | Editor: set type=projection persists through save/reload; rename/delete cascade | Editor (headless) | on-model; no `_proj_delta` |
| TC18 | Pivot approaching a projection node (LINE edge) | Unit (validator+sim) | δ from follow distance; over-budget rejected |
| TC19 | Repo-wide: zero references to removed symbols | Unit (grep test or CI check) | none found |
| TC20 | Toy-field `NEXT_HOP` / edge weights through the projection node | Unit (planner) | Matches the documented intended routing: projection is not a blind-decay anchor; LINE→proj is factor-1, BLIND→proj is decay-priced |

**Test data:** existing `pivot_field_data.py` / `toy_field_data.py` SimpleNamespace builders migrated to the node model; the toy field (`field_data_toy.json`) gains a projection node exercising `line–proj–line` and `blind→proj`. Editor tests use the shared conftest Tk root under xvfb.
**Run command:** `uv run --extra dev pytest tools/path_planner/tests/` and the host C++ ctest (`WRO2026/test`, full clean rebuild for the `*_compile_fail` cases).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Graph builder: new node/edge model | — | high | `graph_builder.py` | Read `type="projection"` into the graph. Edge types `LINE`/`BLIND` only; delete `EDGE_PROJECTION` constant + `next_hop_is_projection`; add `is_projection_node(graph,n)`. `_emit_directed_pair` handles LINE/BLIND (drop projection + `ProjectionMode` bidirectional key). Satisfies AC: encoding. |
| T2 | Validator: legality + `_regime` | T1 | high | `validator.py` | Node vocab `{line,projection,blind,reset}`. Reject LINE→blind (generalize `_reject_line_follow_into_blind`), reject projection-as-start. **Reachability warn:** a `line` node needs ≥1 incident LINE edge; a `projection` node is valid with **only a BLIND in-edge** (it's blind-seek-reachable), so the guard must exempt a projection node that has a `Blind→Proj` edge — do NOT warn on a valid `Blind→Proj` node. Re-derive `_regime` from `(edge_type, dest_node_type)` (LINE→proj=`distance`, LINE→line=`line`, BLIND→proj=`sensor`). Remove `EDGE_PROJECTION` from `legal_edge_types`/anti-ambiguity/mode-class/ProjectionMode validation. Keep `blind_projection_auto_seek` bool. Add the conservative-anchor comment near the pivot/regime code. Satisfies AC 1,2,5; TC1–5,18. |
| T3 | Remove `ProjectionMode`; field_data loader | T1, T2 | med | `pipeline_config.py`, `field_data.py` | Delete `ProjectionMode` + `PipelineConfig.projection`. `field_data.py`: drop `EDGE_PROJECTION` import + commented demo; document projection node authoring; keep retargeted `BLIND_PROJECTION_AUTO_SEEK`. Satisfies AC 2. |
| T4 | Compiler: primitive from (edge,dest) | T1, T2 | high | `compiler.py` | Remove `EDGE_PROJECTION`/`next_hop_is_projection`. Emit per the table; `auto_rule_active = is_projection_node(dest)` (local, not `path[i+2]`). LINE→proj uses `line_mode`; drop ProjectionMode. Satisfies AC 3,4; TC6,7,8. |
| T5 | Sim mirror: primitive + re-anchor | T1, T2 | high | `sim/runtime.py` | Mirror T4: drop projection edge + the `_next_hop[nxt][goal]` peek; primitive from dest type; `resolve_stop_at_line` UNCHANGED. BLIND→proj → full-advance re-anchor; LINE→proj → follow-dist + `on_dead_reckoned_advance`. **Return-to-node: confirm `is_line_backed` already returns false for a projection node (it tests `=="line"`) — do NOT add a projection branch; projection dead-reckons by default.** Remove `resolve_projection_mode`. Satisfies AC 3,4,6; TC8,9,10,14. |
| T6 | Emitter: NodeType enum + remove projection plumbing | T1, T3 | high | `header_writer.py` | `NodeType` gains `PROJECTION`; map `"projection"`. Remove `EdgeType::PROJECTION`, `ProjectionMode` enum, `PipelineConfig.projection`, `projection_mode_override` from `EdgeInfo` (`_edge_cell`+`_none_edge_cell`), `_resolve_overrides`, `_PRIMITIVE_MODE_SLOT_*`, `_PROJECTION_MODE_DEFAULT`. **Note: the literal value `3` migrates from `EdgeType::PROJECTION` (deleted) to `NodeType::PROJECTION` (added) in the same emit function — two near-identical "PROJECTION = 3" lines; edit the right one and update the `_EDGE_TYPE_BY_*` / `_NODE_TYPE_BY_*` maps in lockstep.** Keep `BLIND_PROJECTION_AUTO_SEEK`. Satisfies AC 2; TC15. |
| T-REGEN | Regenerate `routes.*` + goldens | T6, T11 | med | `routes.h`, `routes.cpp`, `tests/golden/*` | Run `build_routes.py`; regenerate production `routes.*` (struct shrink, no proj node) + `routes_toy.h` (with the toy proj node). Commit so CI `git diff --exit-code` is clean. Satisfies AC 9. |
| T8 | FieldMap accessors | T6, T-REGEN | med | `FieldMap.h` | `noEdge()` positional `EdgeInfo` init → 7 fields (drop `ProjectionMode::DEFAULT`). Confirm `getNodeType` covers `PROJECTION`; add `isProjectionNode(id)` helper if it cleans up T10. (The *same* 7-field `EdgeInfo` layout change in any host-test positional init is T14's responsibility — T14 is blocked-by T8 for this seam.) Satisfies AC 2. |
| T9 | Resolver contract (route-agnostic, frozen) | — | low | `StopAtLinePolicy.h` | No logic change. Update the `autoRuleActive` doc comment to "BLIND edge whose destination is a projection node." Verify the struct/ladder untouched so the CSV stays frozen. Satisfies AC 4. |
| T10 | Firmware dispatch + local auto-seek | T8, T9 | high | `PathTracker.h`, `PathTracker.cpp` | Remove `_nextHopIsProjection`; auto-rule = `getNodeType(_nextHop)==PROJECTION` (keep `_peekNextNextHop` for pass-through only, guards intact). `_dispatchEdgeAfterTurn`: drop `case PROJECTION`; in `case LINE` branch on dest type (line→intersection; projection→distance gate + dead-reckon advance, reuse `PROJECTION_FOLLOW`); in `case BLIND` projection-dest → seek + full-advance re-anchor. Remove `_resolveProjectionMode`. `reverseUnavailable` keyed on resolved primitive. **Confirm `_returnUsesLineSeek` already returns false for projection (it tests `==NodeType::LINE`) — do NOT add a projection branch.** Satisfies AC 3,4,6,10; TC16. |
| T11 | Toy field: add a projection node | T2 | med | `field_data_toy.json`, `tests/fixtures/toy_field_data.py` | Add a projection node + a `line–proj–line` and a `blind→proj` scenario so the golden + host tests exercise the model. Toy-only (production indices untouched). Satisfies AC 8. |
| T12 | Migrate unit fixtures | T1, T2, T3 | med | `tests/fixtures/pivot_field_data.py` + other SimpleNamespace fixtures | Convert every `EDGE_PROJECTION` / `projection_mode` / `ProjectionMode` fixture to the node model. Satisfies AC 8. |
| T13 | Python tests: update + extend | T4, T5, T6, T11, T12, T-REGEN | med | `tests/test_graph.py`, `test_compiler.py`, `test_planner.py`, `test_field_data.py`, `test_runtime_sim.py`, `test_auto_stop_at_line_projection.py`, `test_validator_pivot_seek.py`, `test_sim_pivot_seek.py`, `test_header_writer.py`, `test_planner.py` | Rewrite projection cases for the node model; add TC1–15,18,20; regenerate header goldens. **Re-run + diff the stop-at-line truth-table CSV generator (`gen_stop_at_line_truth_table.py`) — it must be a no-diff (the resolver is frozen); a diff means a removed symbol leaked into the seek path.** Run full pytest. Satisfies AC 1–9; TC1–15,18,20. |
| T14 | Host C++ tests | T-REGEN, T8, T10 | high | `WRO2026/test/*` | Update `test_path_tracker_stop_at_line` (auto-rule local), `test_path_tracker_bidirectional` (drop `EdgeType::PROJECTION` TC12), any `EdgeInfo` positional init / mock. Add TC16 (firmware LINE→proj + BLIND→proj). Resolve nodes by name. Run full ctest. Satisfies AC 7; TC16. |
| T15 | Editor model: on-model projection type | T2 | med | `editor/model.py` | `NODE_TYPE_VALUES`/`node_types` accept `"projection"`; `to_json_dict`/`to_module_proxy` carry it (node_types already serialized). Satisfies AC 7. |
| T16 | Editor: delete off-model projection machinery | T15 | high | `editor/app.py`, `commands.py`, `projection_edges.py`, `ast_rewrite.py` | Delete `_proj_delta`/`ProjectionDelta`, `AddProjectionEdge`/`DeleteProjectionEdge`, the 5 scanners, the projection key handlers + "blind instead" offer, rename/delete delta patching + save flush, LibCST `append/remove_projection_edge` + `_Projection*` visitors, `read_projection_edges`/`reconcile`/`compute_projection_geometry` (keep `reset_autofill`). Projection set via `SetNodeType`. Satisfies AC 7. |
| T17 | Editor UI surfaces | T15, T16 | med | `editor/canvas.py`, `panels.py`, `dialogs.py` | Remove projection-edge rendering (`set_projection_edges`/`_draw_projection_edges`/artists); add a projection node color (theme). Remove the "+ add projection" ghost button; ensure the node-type combobox lists `projection`. **Unbind or repurpose the freed `P` keybinding (the old project-edge picker) so it doesn't dangle.** Satisfies AC 7. |
| T18 | Editor tests | T15, T16, T17 | med | `tests/test_editor_*.py` | Delete/rewrite `test_editor_projection*`; update model/commands/inspector/panels/ast_rewrite/app_renames tests for the on-model type. Add a test that the freed `P` keybinding no longer maps to a projection action. Add TC17. Run under xvfb shared root. Satisfies AC 7; TC17. |
| T19 | Pass-through: firmware | T10, T14 | xHigh | `PathTracker.h`, `PathTracker.cpp` | `line–proj–line` continuous follow. **CORE WORK: the trigger is the DISTANCE GATE at B, not `Intersection.isAtIntersection()`** — a projection node has no junction marker, so the inbound LINE edge A→B is distance-terminated and runs the `PROJECTION_FOLLOW`/distance-gate path; `isAtIntersection()` never fires at B. At the A→B distance gate, use `_peekNextNextHop` (**lookahead only, for routing**) + the ADR-023 collinear heading check: if B is a non-goal projection node, both flanking edges are LINE, and A→B→C are collinear, then instead of stopping, keep LineFollower enabled, emit `onDeadReckonedAdvance(B)` mid-follow (no stop), and continue line-following to C, where `isAtIntersection()` DOES fire and re-anchors. The state transition is distance-gated, not intersection-gated. Guard: fallback to per-hop (stop at the gate) when any condition fails. Satisfies AC; TC11,12,13. |
| T20 | Pass-through: compiler + sim parity | T4, T5, T19 | high | `compiler.py`, `sim/runtime.py` | Mirror T19 offline, **including the distance-gate trigger (not intersection)**: terminate the A→B LINE edge on distance at the projection node and emit the mid-follow advance there; continuous A→C; same collinear/goal gate. Satisfies AC; TC11,12,13. |
| T21 | Pass-through tests | T19, T20 | med | new `tests/test_pass_through_projection.py` + host test | TC11,12,13 across sim + host C++. New files to avoid collision with T13/T14. Satisfies AC; TC11,12,13. |
| T22 | ADR + docs | T10, T16, T19 | low | new `docs/decisions/0NN-*.md`, `DECISIONS.md`, ADR-023/038/039/044, `ARCHITECTURE.md`, `Roadmap.md`, `field_data`/editor docs | New ADR (supersedes the 038/039 auto-rule sub-decision; amends 023 + 044). **Note in the ADR that no prior ADR established "projection is an edge type" — it was an implicit convention introduced piecemeal — so the new ADR documents/retires a convention, not a numbered decision; also note the issue body's ADR-036/037 citations were wrong (036=pivot, 037=vision).** Mark/cross-link; update module map, glossary, Risk #9. Repo-wide grep AC 2 check. Satisfies AC 10; TC19. |

## Notes for Implementer

- **Parity is the whole game on the seek path.** Do not touch the body of `resolveStopAtLine` / `resolve_stop_at_line` or the 936-row CSV. The only seek-path change is the value the caller computes for `auto_rule_active`. If the CSV golden regenerates, you changed the resolver — stop and revert.
- **`_peekNextNextHop` has two callers today** (auto-seek + ADR-023 pass-through) sharing one guard set. The auto-seek caller **drops** it (now local); the pass-through caller **keeps** it. After the split, re-verify the pass-through peek's NODE_NONE / self-loop / not-final guards are still self-contained.
- **The pass-through fires off the DISTANCE GATE at the projection node B, NOT `Intersection.isAtIntersection()`** — a projection node has no junction marker, so the inbound LINE edge runs `PROJECTION_FOLLOW`/distance-gate and `isAtIntersection()` never fires until the real junction C. `_peekNextNextHop` is reused for the routing lookahead only; it does not change which gate triggers the state transition. This is the load-bearing pass-through hazard — do not try to bolt the line-proj-line collapse onto the intersection-gated ADR-023 path unchanged.
- **The LINE→projection regime flip is the silent-failure trap.** A LINE edge into a projection node is now distance-terminated (no junction to overshoot onto), so a pivot approaching it must trim δ from the **follow distance**, not the overshoot budget. Both `validator._regime` and the firmware `LINE` dispatch branch must handle this; it validates clean if you miss it. (Both Fable and Codex flagged this independently.)
- **`EdgeInfo` field removal is a coordinated multi-site edit** even though there's no `offsetof` static_assert today: emitter `_edge_cell` + `_none_edge_cell`, `FieldMap::noEdge()` positional init, generated `routes.cpp`, and any host-test `EdgeInfo` positional init. A missed positional init only warns under `--warnings all` / arduino-cli, not host ctest — grep for hand-written `EdgeInfo{...}` after the change.
- **`planner.py` needs no functional change.** `_ANCHOR_NODE_TYPES` stays `{line, reset}` by decision; `build_pivot_turns` and the cost model are projection-agnostic. Add a one-line comment recording the conservative-anchor decision so it isn't "fixed."
- **Host C++ tests link the real `routes.cpp` by node name** (`findNodeByName`), and we only add the projection node to the **toy** field, so production indices don't renumber — but still run the full host ctest after T-REGEN (a stale comment about node indices in `test_path_tracker_reset_arrival` is harmless since it resolves by name).
- **Editor `reset` node is the exact precedent** for an on-model node type with a combobox value — follow it, but projection needs **no** sub-panel (type-only). `node_types` already serializes through both `to_json_dict` and `to_module_proxy`, so no dual-serialization trap if you set the type through the model.
- **Fixtures break across both suites.** A schema change to the strict-read SimpleNamespace fixtures hits both Python pytest and the C++ `*_strict` siblings — run the full pytest AND the full ctest, not per-file.
- **Rollback plan:** the staged delivery makes the core independently revertible; if the pass-through (T19–T21) misbehaves on the bench, revert just that phase — the node-property core stands alone and is the issue's primary deliverable.
- **Bench gate:** the pass-through and the projection re-anchor are physical behaviors; no mission should rely on a projection-node hop until verified on the real chassis (analogous to the B3.8 pivot gate). Flag this in the ADR's "Trigger to revisit."
