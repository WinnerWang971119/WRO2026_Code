# PR #98 Triage Fixes — JSON EDGES overrides + operator inspector

**Goal:** Fix the 16 triaged items on PR #98 (issue #79 / ADR-047) — three first-hand GUI bugs and twelve review findings — so the per-edge override overlay stays consistent under every edit, the inline editor is usable, reset nodes are blind-incident only, and the offline sim mirrors the robot.

**Approach:** Land the changes on the existing `feat/issue-79-edges-json-overrides-gui` branch as eight dependency-ordered tasks, grouped by file/concern so disjoint-file work runs in parallel and same-file work serializes. The validator-contract change (U2/U5/R8) lands as one atomic task — rule + Python fixtures + toy golden + C++ host shadow fixtures + ADR-048 — so every later task's CI signal is attributable. GUI guards mirror the validator predicates and come after the rule. Criticals (U1/U3, R1, R2) and the silent sim-mirror divergence (R4) get the most care.

## Scope

- **In scope:**
  - U1/U3: inline override editor flash-close + unable-to-edit (GUI event ordering).
  - U2: reject the contradiction `primitive_mode → BlindMode.STOP_AT_LINE` together with `stop_at_line='off'` (validator + GUI).
  - U5: reset nodes are BLIND-incident only — validator rejection, edge-mode `L` guard, auto-classify.
  - R1, R2: command-overlay consistency on node/edge delete + kind-flip.
  - R3, R4, R5, R6, R7, R8, R9, R10, R11, R12: the review findings (migrate capture, sim/compiler config threading, override-entry preservation, FIELD_MM precision + no-op guard, validator message, rename registry, shared identity helper, refresh dirty-check, direction-combobox DEFAULT).
  - ADR-048 documenting the new validator contract (reset blind-incidence + STOP_AT_LINE/off contradiction).
- **Out of scope:**
  - U4 (the operator's question about LINE-edge `stop_at_line` semantics) — answered in conversation, no code change.
  - Any per-edge numeric blind-speed / new override keys (issue #97, deferred).
  - Re-encoding the real WRO field; production routing changes.
  - The shipped competition mission / firmware behavior (offline tooling only, plus the sim-parity fix).

## Decisions

- **All fixes on PR #98 branch as grouped commits** — no new branch, no separate PRs (user-confirmed).
- **R2 kind-flip clear has a single owner: `DeleteEdge`'s override cascade.** The kind-flip path `_add_plain_edge_with_override` already issues `DeleteEdge(a,b)` as its first MacroCommand step, so once `DeleteEdge.apply` cascades-removes the old-kind override, the flip is clean — no separate clear step in `app.py`, avoiding the double-clear trap. The cascade removes every override entry whose `type` equals the deleted edge's kind and whose `{u,v}` equals the undirected pair `{a,b}` (both directed orientations); revert re-inserts the captured entries at their original indices. (Resolves the consult's open fork: clear the deleted kind's overrides on the pair, both directions.)
- **U1/U3 fix = refresh-first-then-graft.** In `_handle_edge_click`, call `_refresh_view()` BEFORE `_graft_edge_override_panel(a,b)` so the graft locates/prefills against the rebuilt row set. Do NOT weaken `refresh()`'s unconditional `_hide_override_editor()` — it intentionally closes half-entered forms on model replay (load/undo/redo). Clicking an existing-override edge opens the inline editor pre-filled for edit (user-confirmed).
- **R4: compute `PipelineConfig.from_field_data(field_data)` locally and thread it.** `compile_path` already receives `field_data` — build `cfg` once at the top, thread it into `_resolve_line`/`_resolve_blind`/direction resolution (which currently bind the module singleton as default-arg values, so this is a signature-threading change), default the param to the singleton for legacy callers. Same in the sim resolver setup. No `field_data` → singleton fallback keeps every `SimpleNamespace` fixture working (user-confirmed).
- **R5: preserve non-editor-owned keys on edit.** `SetEdgeOverride.apply` carries forward any keys on the replaced entry that the editor does not own (`distance_cm`, `heading_deg`, anything beyond `type/u/v/primitive_mode/direction/stop_at_line`); editor-owned keys absent from the form are removed (set-to-inherit) (user-confirmed).
- **U2 guard is the one contradiction only** — `STOP_AT_LINE` primitive + `stop_at_line='off'`; in validator (single gate, ADR-029) AND the inline editor Save (fast feedback) (user-confirmed).
- **U5 enforcement is defense-in-depth** — validator rejects LINE→reset, edge-mode `L` refuses a reset-incident pair, `auto_edges` classifies reset endpoints as blind. Projection nodes stay LINE-legal (ADR-046) (user-confirmed).
- **R10 shared helper stays in `commands.py`.** `panels.py` imports `_find_override_index` from `commands.py` (acyclic: `commands` never imports `panels`). No new leaf module — minimal churn.
- **R3: capture + template alias both done.** Add `RESET_NODES` + `BLIND_PROJECTION_AUTO_SEEK` to migrate's capture and re-expose them in the regenerated module. The legacy `.py→JSON` path is "dead" per project memory, but `migrate` is shipped code and `header_writer` strict-reads `field_data.RESET_NODES`; the fix is cheap correctness (defer-rule applies only to unbuilt features).
- **New ADR-048** documents the reset-blind-incidence rule and the STOP_AT_LINE/off contradiction rejection (project-wide validator-contract changes crossing editor/validator/auto-edges; under the ADR-029 anti-ambiguity umbrella).

## Acceptance Criteria

- [ ] Clicking an edge in edge-mode opens the inline override editor and it STAYS open (no flash-close). For a new edge it opens a blank add-form pre-filled with the clicked pair + derived type; for an existing-override edge it opens pre-filled with that override for editing. (U1/U3)
- [ ] Editing an existing override and Saving replaces it in place (same `(type,u,v)`), updates the row, and does not create a duplicate. (U1, R5)
- [ ] Editing an override that carried hand-authored `distance_cm`/`heading_deg` preserves those keys after Save. (R5)
- [ ] Authoring `primitive_mode=STOP_AT_LINE` together with `stop_at_line='off'` on a BLIND edge is rejected — by the inline editor Save (status message, no dispatch) and by `validate()` on Save-to-disk (precise `FieldDataError`). (U2)
- [ ] A LINE edge incident to a reset node is rejected by `validate()`; edge-mode `L` on a reset-incident pair refuses with a status message; `auto_edges.propose_edges` classifies a reset-incident candidate as `"blind"`. Projection-incident LINE edges remain legal. (U5)
- [ ] Deleting a node that carries an override removes the override (cascade); undo restores both node and override; a subsequent Save does not raise "EDGES references unknown node". (R1)
- [ ] Deleting an edge that carries an override removes the matching-kind override; `graph_builder` no longer re-emits the deleted edge; undo restores it. Flipping an edge LINE↔BLIND that carried an override leaves no wrong-typed override; Save is not rejected. (R2)
- [ ] After any sequence of node-delete / edge-delete / kind-flip, `model.edges` contains no entry whose `(type,u,v)` lacks a live underlying segment of that exact type. (R1, R2 — invariant test)
- [ ] After setting a non-default `PIPELINE_BLIND_MODE` (or `LINE_MODE`) in the field, the offline sim's emitted primitives match the compiled primitives AND differ from the singleton-default emission. (R4)
- [ ] Migrating a legacy `field_data.py` that defines `RESET_NODES` / `BLIND_PROJECTION_AUTO_SEEK` round-trips both into the JSON and re-exposes them in the regenerated module. (R3)
- [ ] The pipeline DEFAULT validator error names the real field (`PIPELINE_CONFIG.line` / `.blind` / `.turn`), not `.linemode`. (R8)
- [ ] Editing one FIELD_MM dimension and committing leaves the other dimension's sub-mm precision intact; committing an unchanged FIELD_MM fires no command (no undo entry, no dirty flag). (R6, R7)
- [ ] The pipeline-direction combobox no longer offers both `''` and `DEFAULT`; a node nudge (MoveNode) does not rebuild the override-list widgets. (R12, R11)
- [ ] `RenameNode` and `FieldModel.from_snapshot` walk a single node-bearing-container registry; `panels.py` calls `commands._find_override_index` rather than a duplicated match. (R9, R10)
- [ ] Full `pytest` green; full `ctest` green; `build_routes.py` then `git diff --exit-code` clean (committed routes byte-identical); ADR-048 added + indexed in DECISIONS.md.

## Contracts & Interfaces

Cross-task seams — defined once here; each task conforms byte-for-byte.

### Signatures

- `commands._find_override_index(edges: list[dict], edge_type: str, u: str, v: str) -> int` — owner **T2**; consumer **T6** (R10 imports it into `panels.py`). Stays in `commands.py`, signature unchanged.
- `PipelineConfig.from_field_data(field_data) -> PipelineConfig` — exists; owner of new call sites **T4**. `compile_path(path, start_heading_deg, graph, field_data)` computes `cfg = PipelineConfig.from_field_data(field_data)` once and threads `cfg` to the resolve helpers (new param, defaults to `PIPELINE_CONFIG`).

### Overlay-lifecycle semantics (owner T2)

- **`DeleteEdge.apply(model)`** also removes every entry in `model.edges` where `entry["type"] == <deleted edge kind tag>` and `{entry["u"], entry["v"]} == {a, b}` (both directed orientations). Captures them (deep-copied, with original indices) for `revert`, which re-inserts them. This is the sole owner of the kind-flip override clear.
- **`DeleteNode.apply(model)`** also removes every entry in `model.edges` where `entry["u"] == name or entry["v"] == name` (any type). Captures for `revert`.
- **`SetEdgeOverride.apply(model)`** when replacing in place merges editor-owned keys over a deep copy of the prior entry, preserving non-editor-owned keys (geometry). `_build_entry` takes the prior entry (or `None`).
- **Node-bearing-container registry**: one module-level structure in `commands.py` (or `model.py`) naming the containers a rename/snapshot walks — `nodes`, `node_types`, `line_segments`, `blind_edges`, `reset_nodes`, `pivot_turns`, `edges`. `RenameNode.apply/revert` and `FieldModel.from_snapshot` both iterate it.

### Shared rule predicates (must match across modules)

- **Reset-incidence rule**: "a LINE edge incident to a reset node is illegal; an edge to a reset node must be BLIND." Enforced by **T1** (`validator`, sibling to `_reject_line_follow_into_blind`), **T3** (`auto_edges._classify_edge_type`: reset OR blind endpoint → `"blind"`), **T7** (edge-mode `L` guard). Projection endpoints are NOT reset/blind → stay `"line"`.
- **STOP_AT_LINE/off contradiction**: `primitive_mode` resolving to `BlindMode.STOP_AT_LINE` AND `stop_at_line` in `{'off', StopAtLineMode.OFF}` on the same entry. Enforced by **T1** (`validator._validate_edges_field`) and mirrored by **T7** (`panels._handle_save_override`).

### File ownership

| File | Owner task | Notes |
|------|-----------|-------|
| `validator.py` | T1 | U2 rule, U5 LINE→reset, R8 message |
| test fixtures / goldens / host C++ shadow fixtures / ADR | T1 | atomic with the rule |
| `commands.py`, `editor/model.py` | T2 | R1, R2, R5, R9 |
| `editor/auto_edges.py` | T3 | U5 classify |
| `compiler.py`, `sim/runtime.py` | T4 | R4 threading + parity test |
| `editor/migrate.py` | T5 | R3 |
| `editor/panels.py` + override/FIELD_MM parts of `editor/app.py` | T6 then T7 | T6: U1/U3, R10, R11, R12; T7: U2-Save, U5-`L`-guard, R6, R7 |

## Error Handling

- **Inline editor Save rejects** (no segment, STOP_AT_LINE+off): set the status bar message, do NOT dispatch, leave the editor open so the operator can correct.
- **Validator rejection** (LINE→reset, STOP_AT_LINE+off, unknown tag): raise `FieldDataError` with the `EDGES[i]` index + a one-line fix hint, before emit, so a build never reaches `header_writer`.
- **Command revert** after a rejected step (e.g. kind-flip MacroCommand): `MacroCommand.apply` rolls back so the model is never left half-overridden; `_dispatch` surfaces the error.
- **`from_field_data` on a field missing the keys**: falls back to the `PIPELINE_CONFIG` singleton (no error) — fixtures unaffected.

## Testing Strategy

**Levels:** Unit (Python pytest), host C++ (ctest), integration (routes freshness + full suites).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `validate` an EDGES entry `STOP_AT_LINE` + `stop_at_line='off'` | Unit | Raises `FieldDataError` naming the contradiction |
| TC2 | `validate` a LINE edge incident to a reset node | Unit | Raises `FieldDataError` (mirror of blind-node message) |
| TC3 | `validate` a LINE edge incident to a projection node | Unit | Passes (ADR-046, regression guard) |
| TC4 | Pipeline DEFAULT error message text | Unit | Contains `PIPELINE_CONFIG.line`/`.blind`/`.turn`, not `.linemode` |
| TC5 | `DeleteNode` on a node carrying an override, then revert | Unit | Override removed on apply, restored on revert; no orphan |
| TC6 | `DeleteEdge` on an edge carrying an override, then revert | Unit | Matching-kind override removed/restored; `graph_builder` does not re-emit |
| TC7 | Kind-flip LINE→BLIND on an edge with a LINE override | Unit | No LINE override survives; Save not rejected |
| TC8 | No-orphan invariant after random delete/flip sequence | Unit | Every `model.edges` entry has a live same-type segment |
| TC9 | `SetEdgeOverride` edit of an entry with `distance_cm`/`heading_deg` | Unit | Geometry keys preserved after replace |
| TC10 | Sim vs compiler primitives with non-default `PIPELINE_BLIND_MODE` | Unit | Match each other; differ from singleton-default emission |
| TC11 | `migrate` a `.py` defining `RESET_NODES` + `BLIND_PROJECTION_AUTO_SEEK` | Unit | Both reach the JSON and the regenerated module |
| TC12 | `auto_edges.propose_edges` with a reset-incident candidate | Unit | Classified `"blind"` |
| TC13 | Edge-mode `L` on a reset-incident pair | Unit/headless | Refused with a status message; no command dispatched |
| TC14 | Edge-click in edge-mode (new + existing-override edge) | Headless GUI | Editor stays open; existing-override edge opens pre-filled |
| TC15 | Inline Save STOP_AT_LINE+off | Headless GUI | Status message, no dispatch, editor stays open |
| TC16 | Edit one FIELD_MM dim with a sub-mm other dim | Unit/headless | Other dim's precision intact after commit |
| TC17 | Commit unchanged FIELD_MM | Unit/headless | No command dispatched, no dirty flag |
| TC18 | Node nudge (MoveNode) | Headless GUI | Override-list widgets not rebuilt (dirty-check) |
| TC19 | Direction combobox values | Unit | No `DEFAULT` entry; single inherit sentinel |
| TC20 | Full suite + routes freshness | Integration | pytest + ctest green; `git diff --exit-code` clean |

**Test data:** existing `SimpleNamespace` / toy / pivot fixtures; a new dedicated fixture for TC10 (non-default pipeline mode) and TC2/TC11 (a reset node). Editor headless tests force matplotlib `Agg` and a shared Tk root; CI runs them under xvfb.
**Run command:** `uv run --extra dev pytest tools/path_planner/tests/` ; `ctest --test-dir build/test --output-on-failure` ; `uv run python tools/path_planner/build_routes.py && git diff --exit-code`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Validator contract + fixtures + ADR | — | high | `validator.py`, test fixtures, `tests/golden/routes_toy.*`, host C++ shadow fixtures, `tests/test_field_data.py`/validator tests, `docs/decisions/048-*.md`, `DECISIONS.md` | Add U2 STOP_AT_LINE+off rejection and U5 LINE→reset rejection to `_validate_edges_field` (the latter a sibling to `_reject_line_follow_into_blind`, gating on a reset endpoint). Fix R8: give `pipeline_specs` a 5th element = the singleton field name (`line`/`blind`/`turn`/`direction`) and interpolate it at the 3 message sites (~1729/1751/1759) instead of `enum_name.lower()`. AUDIT every reset node across `field_data.json` (none today), the toy/pivot fixtures, the inline `_build_toy_field_data()` in `test_header_writer.py`, the `routes_toy` golden, and the C++ host shadow fixtures for LINE→reset incidence; re-type any to BLIND and regenerate goldens. Add TC1–TC4. Run FULL pytest AND FULL ctest. Write ADR-048 (reset blind-incidence + STOP_AT_LINE/off contradiction) and index it in DECISIONS.md. Satisfies AC U2/U5/R8. |
| T2 | Command overlay lifecycle | — | high | `editor/commands.py`, `editor/model.py` | R1: `DeleteNode.apply` cascade-removes every `model.edges` entry incident to the node (capture for revert); correct the wrong "already covered" docstring. R2: `DeleteEdge.apply` cascade-removes every override of the deleted edge's kind on `{a,b}` both directions (capture+revert) — sole owner of the kind-flip clear. R5: `SetEdgeOverride._build_entry`/`apply` preserve non-editor-owned keys when replacing in place. R9: add a single node-bearing-container registry and rewire `RenameNode.apply/revert` + `FieldModel.from_snapshot` to walk it. Add TC5–TC9 + the no-orphan invariant test. Satisfies AC R1/R2/R5/R9. |
| T3 | Auto-edge reset classification | — | low | `editor/auto_edges.py`, tests | `_classify_edge_type`: return `"blind"` when either endpoint is blind OR reset; else `"line"` (projection stays line). Update the docstring. Add TC12. Satisfies AC U5 (auto-classify). |
| T4 | Sim/compiler pipeline parity | — | high | `compiler.py`, `sim/runtime.py`, tests | R4: compute `PipelineConfig.from_field_data(field_data)` once where `field_data` is in scope; thread it into the line/blind/direction resolve calls in `compile_path` (changing the default-arg-bound `_resolve_line`/`_resolve_blind` signatures to take the resolved config, defaulting to the singleton) and the sim resolver setup; replace every `PIPELINE_CONFIG.<x>` runtime read. Add TC10 with a dedicated non-default-pipeline-mode fixture (exercise `PIPELINE_BLIND_MODE`/`LINE_MODE`, not just direction). Satisfies AC R4. |
| T5 | Migrate capture | — | low | `editor/migrate.py`, tests | R3: add `RESET_NODES` + `BLIND_PROJECTION_AUTO_SEEK` to `_ATTRS_TO_CAPTURE` (update the "fourteen attributes" docstring/count + any attr-set assertion) and re-expose both in `_build_rewritten_source`'s template. Add TC11. Satisfies AC R3. |
| T6 | GUI override-editor interaction | — | med | `editor/panels.py`, `editor/app.py` (override paths) | U1/U3: in `_handle_edge_click` call `_refresh_view()` BEFORE `_graft_edge_override_panel`; make the graft open the inline editor pre-filled (edit) for an existing-override edge (not just highlight), and pre-filled add-form for a new one — both surviving the prior refresh. R10: `panels` imports + calls `commands._find_override_index` (drop the duplicated match in `_find_override_entry` + `_rebuild_edge_override_rows`). R11: dirty-check so `refresh()` skips the override-list rebuild when `model.edges` + segment sets are unchanged (preserve `_edge_override_row_frames` on the skip path). R12: drop `DEFAULT` from `_PIPELINE_DIRECTION_VALUES`. Add TC14, TC18, TC19. Satisfies AC U1/U3/R10/R11/R12. |
| T7 | GUI rule guards + FIELD_MM | T1, T6 | med | `editor/panels.py`, `editor/app.py` | U2 GUI: `panels._handle_save_override` rejects STOP_AT_LINE+off (status message, no dispatch) using the same predicate as T1. U5 GUI: edge-mode `L` handler refuses a reset-incident pair with a status message (consult node types via the model; projection stays allowed). R6: keep FIELD_MM sub-mm precision — set the un-edited dimension's var without `round()` / read the untouched value from the model. R7: guard `_on_panel_field_dimensions_change` to skip dispatch when `(width,height)` equals `model.field_mm` (mirror `_on_panel_global_knob_change`). Add TC13, TC15, TC16, TC17. Satisfies AC U2(GUI)/U5(GUI)/R6/R7. |
| T8 | Integration verification | T1,T2,T3,T4,T5,T6,T7 | low | — (verification; regenerate routes only if a diff appears) | Run full `pytest`, full `ctest`, and `build_routes.py` then `git diff --exit-code`. Confirm committed `routes.h`/`routes.cpp` are byte-identical (no field-affecting change for the committed field). If arduino-cli is available, compile-sanity the sketch. Satisfies AC (final). |

## Notes for Implementer

- **Parallelism:** T1, T2, T3, T4, T5, T6 touch disjoint files and can run concurrently. T7 must wait for T1 (rule decided) and T6 (shares `panels.py` + `app.py`). T8 is the final gate.
- **The U1/U3 fix is an ORDERING change, not a refresh weakening.** Moving the graft after `_refresh_view()` is the fix; the graft's `locate_override_row`/`prefill_add_row` then read the freshly-rebuilt `_edge_override_row_frames`. Do not add a flag to suppress the hide — that reintroduces stale-form-on-replay bugs.
- **R2 / kind-flip:** do NOT also add an override-clear in `app._add_plain_edge_with_override`; the existing `DeleteEdge(a,b)` step now cascades. Verify the MacroCommand revert restores the override (DeleteEdge.revert re-inserts).
- **T1 is the cross-language landmine** (memories: `gotcha-field-contract-change-breaks-cpp-and-py-fixtures`, `gotcha-strict-emitter-field-fixtures`, `gotcha-two-toy-fields-golden-vs-fixture`). The toy golden is built from the inline `_build_toy_field_data()` in `test_header_writer.py`, NOT `fixtures/field_data_toy.json` — rebaseline via that module's `__main__`. Run the FULL pytest and FULL ctest, never per-file.
- **R4 partial-thread hazard:** if you fix `compile_path`'s body but miss the `_resolve_*` default-arg binding, the singleton stays authoritative for primitive_mode while direction gets fixed — a partial mirror that looks fixed. TC10 must exercise a non-default `PIPELINE_BLIND_MODE`/`LINE_MODE`.
- **R8:** `LineMode → line` is not `"LineMode".lower()`; add the field name to the spec tuple explicitly rather than deriving it.
- **No magic node-type strings:** use the existing `_RESET_NODE_TYPE` / node-type constants; don't hardcode `"reset"`.
- **Rollback:** every change is offline-tooling; revert is per-commit. The only firmware-adjacent change is R4 (sim parity) and it cannot alter committed `routes.*` for the current field (TC20 / T8 proves it).
