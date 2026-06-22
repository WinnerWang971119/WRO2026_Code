# Field Editor: 3-Pane Restyle + Save/Build Fix + Reset-Node Creation + Projection Edges + Shortcuts Plan

**Goal:** Take the already-executed #78 editor (de-lag + property inspector + warm theme) the rest of the way to the target mockup — a true 3-pane layout with a left mode toolbar, a sectioned properties inspector, a title bar and a tall status strip — and close the four operator-facing gaps the user hit: Save/Save&Build not reaching the screen, no "reset" choice when creating a node, no way to author projection edges, and no visible shortcut reference. Plus answer "what is wall coord" by auto-filling it.

**Approach:** Editor-only work, continued **on the existing `worktree-issue-78-editor-delag-ux` branch**. Build the mockup in **pure ttk + the existing `theme.py`** (no customtkinter — it stays deferred to #83 to protect the blit path's device-pixel math). Restructure `panels.py` into the 3-pane chrome the mockup shows and give Undo/Redo/Save/Save&Build a fixed, always-visible button block (this is the actual Save/Build "fix" — the buttons exist in code today but are clipped below the fold of the 240 px right panel). Add `"reset"` to the Add-node dialog (creation routes through the existing atomic `SetResetSpec`, which now seeds a geometry-derived default spec). Auto-fill `wall_coord` + `facing` from the node position and `FIELD_MM`. Add a **scoped** projection-edge authoring path: pick `u→v` on the canvas, the editor auto-computes `distance_cm`/`heading_deg` and tracks the add/remove as a **pending EDGES delta flushed via LibCST at Save** (same pattern node renames already use), excluding the `primitive_mode`/`direction` override matrix (that stays #79). Add a modal shortcuts dialog.

## Scope

- **In scope:**
  - **3-pane restyle (`panels.py` + `app.py` + `theme.py`):** a dedicated **left mode toolbar** (vertical: Move / Add / Edge / Obst / Strt, active mode gets a 3 px terracotta left bar + accent-soft tint); a **title bar** ("WRO 2026 · Field Editor", Fraunces ~16, the edited filename `field_data.json`, and a terracotta dirty dot ●); a **right properties inspector** with small-caps section headers + hairline rules and **collapsible** sections (◢ open / ▸ collapsed) for Node, Reset spec, Pivots, Global knobs; **ghost** "+ add pivot" / "+ add projection" buttons; a **button block** (Undo / Redo / Save / Save & Build, Save & Build = terracotta fill) that is always visible; a **tall, 2-line status strip** (⬤ green Valid / brick Invalid+count · MODE · selection · key hint).
  - **Canvas warmth (`canvas.py`):** map PNG drawn at **alpha 0.6 over the cream facecolor** (not grey), **hairline** grid, selected-node ring terracotta. Must not disturb the blit fast-path.
  - **Save / Save & Build reachability + function:** the new button block guarantees the two actions are on-screen; verify the full click → `model.validate()` → `dump_field_json` → `rewrite_edges_uv` → `build_routes._run` path actually runs and regenerates `routes.h`/`routes.cpp`.
  - **Reset-node creation:** add `"reset"` to `AddNodeDialog`'s type combobox; selecting it creates the node, seeds a **default reset spec** (auto-filled `wall_coord`/`facing` from geometry), and auto-opens the Reset-spec inspector section for confirmation. Creation routes through the atomic `SetResetSpec` command (never plain `SetNodeType`), so the type↔spec invariant holds.
  - **Wall-coord + facing auto-fill:** given the node's `(x,y)`, the chosen `axis`, and `FIELD_MM`, suggest `wall_coord` = the field-boundary wall on that axis with valid standoff (≥ `WALL_PIVOT_CLEARANCE_CM` = 14 cm) and `facing` pointing away from it (into the field). Both stay editable; recompute when axis or node position changes; never overwrite a value the user has manually edited away from the suggestion.
  - **Projection edges (scoped):** a projection edit flow (pick source `u` then head `v` on the canvas), auto-computing `distance_cm` (Euclidean) and `heading_deg` (departure heading, CCW from +x, `[0,360)`); tracked as a **pending EDGES delta** (adds/removes) reconciled into `field_data.py`'s `EDGES` list via new LibCST append/remove rewrites at Save; rendered as **directed** teal edges (arrowhead at the head `v`). Undo/redo via the established 4-event delta-tracking pattern.
  - **Shortcuts viewer:** a modal `ShortcutsDialog` opened by `?` / `F1` and a clickable hint in the status strip, listing every keybinding grouped by category (modes, edit, edges, nudge, file).
- **Out of scope:**
  - Per-edge **`primitive_mode` / `direction` override editing** — stays in **#79**. This plan authors only `type=EDGE_PROJECTION` entries with the four required keys.
  - **customtkinter** rounded-elegance restyle — stays in **#83**.
  - Editing **line/blind** edges' rich `EDGES` overrides, or surfacing every `field_data` setting (#80), the web front end (#81).
  - Any change to `validator.py`, `graph_builder.py`, `compiler.py`, `header_writer.py`, or firmware. The downstream pipeline already supports `EDGE_PROJECTION` and reset nodes; we only feed it.
  - Snap-to-grid, multi-select move (dropped earlier in #78).

## Decisions

- **Continue on the `worktree-issue-78-editor-delag-ux` branch** (user-confirmed) — not a new stacked branch. The GUI lives here and is not yet on `main`.
- **Pure ttk + `theme.py`, no customtkinter** — confirmed. The mockup's small-caps headers, hairline rules, collapsible chevrons, ghost buttons, and terracotta-fill button are all achievable with `ttk.Style` + frames; customtkinter's global HiDPI scaling would threaten the blit path (same family as the CTPS HiDPI gotcha) and stays deferred to #83.
- **The Save/Build "bug" is a layout-clipping bug, not a logic bug.** `_save`/`_save_and_build` (`app.py:1988`, `app.py:2076`) and their wiring (`panels.py:1134`) are correct; the 240 px fixed-width `pack_propagate=False` right panel stacks the button frame below the visible fold. The fix is structural (dedicated button block in the 3-pane layout), plus an end-to-end verification of the action path.
- **Projection edges persist via a pending EDGES delta flushed at Save through LibCST** — NOT as a new `FieldModel` JSON field. This honors the loader contract ("the rich `EDGES` table stays in `field_data.py`"), avoids the dual-serialization trap (no new JSON key), and reuses the exact rename-delta pattern (`gotcha-editor-rename-edges-delta`): track adds/removes at the four event points (edit / undo / redo / save-clear), render from `(loaded EDGES projection entries) ∪ pending-adds − pending-removes`, flush at Save, never rebuild the delta from history.
- **Projection edge geometry is editor-derived, not hand-typed.** The user picks `u→v`; the editor computes `distance_cm` and `heading_deg` from the two node coordinates. This keeps the entry valid-by-construction and matches how the demo `EDGES[0]` comment documents the fields.
- **Reset creation reuses `SetResetSpec`'s atomic type+spec invariant.** Adding `"reset"` to the Add-node dialog does not create a divergent (typed-reset, no-spec) node: the create path issues `SetResetSpec` with a seeded default, so Save's `_validate_reset_nodes` never sees a typed-but-spec-less node.
- **Auto-fill suggests, never traps.** `wall_coord`/`facing` defaults are seeded on type→reset and recomputed on axis/position change, but a user-entered value is preserved (don't clobber manual edits). A default that can't satisfy the 14 cm clearance (node too close to both walls on that axis) is still written but leaves the badge Invalid until corrected — the geometry rule stays owned by the Save validator.

## Acceptance Criteria

- [ ] **AC1** — The editor renders as 3 panes: a left mode toolbar, the center canvas, and a right properties inspector, with a title bar above and a status strip below, matching the mockup's structure.
- [ ] **AC2** — The active mode shows a 3 px terracotta left bar + accent-soft tint; clicking a toolbar entry or pressing its key switches mode and updates the visual + the status strip.
- [ ] **AC3** — Undo / Redo / Save / Save & Build are always visible in a dedicated button block (Save & Build terracotta-filled); none is clipped at the default window size.
- [ ] **AC4** — Clicking Save writes `field_data.json` (and flushes any pending renames + projection-edge deltas to `field_data.py`); clicking Save & Build additionally runs `build_routes._run` and regenerates `routes.h`/`routes.cpp`; a validation failure shows the modal and writes nothing.
- [ ] **AC5** — The Add-node dialog offers `line` / `blind` / `reset`; choosing `reset` creates the node with a seeded default spec and auto-opens the Reset-spec section. The node is never saved typed-`reset` without a `RESET_NODES` entry.
- [ ] **AC6** — Selecting axis x or y on a reset node auto-fills `wall_coord` to a field-boundary wall with valid standoff and `facing` pointing away from it; both remain editable and a manually-edited value is not overwritten by a later recompute unless the node moves/axis changes.
- [ ] **AC7** — The operator can add a projection edge by picking `u` then `v`; the editor stores it with auto-computed `distance_cm`/`heading_deg`, renders it as a directed teal edge (arrowhead at `v`), and Save folds it into `field_data.py`'s `EDGES` as a `type=EDGE_PROJECTION` entry that `build_routes` accepts.
- [ ] **AC8** — Adding then undoing a projection edge restores the prior state; redo re-adds it; a Save clears the pending delta; a projection edge deleted in the editor is removed from `EDGES` on Save.
- [ ] **AC9** — Pressing `?` or `F1` (or clicking the status-strip hint) opens a modal listing every keyboard shortcut grouped by category; closing it returns focus to the canvas.
- [ ] **AC10** — The canvas map is the warm cream field (PNG at ~0.6 alpha, hairline grid), not default grey; the selected-node ring is terracotta; the blit drag path still works and the blit-equivalence test still passes.
- [ ] **AC11** — `uv run --extra dev pytest tools/path_planner/tests/` is fully green, including new projection-LibCST, reset-autofill, add-node-reset, shortcuts, and panels-3-pane tests; no existing editor test regresses.

## Data Model

No `FieldModel` JSON field is added (avoids the dual-serialization trap). Projection edges are handled as a session delta + a render cache read from the already-cached `field_data` module:

```python
# Already present (unchanged): FieldModel.line_segments / blind_edges round-trip JSON.
# field_data.py exposes the rich EDGES list (read via the editor's cached module handle).

# New session state on EditorApp (mirrors _pending_renames):
_pending_proj_adds:    list[tuple[str, str, float, float]]   # (u, v, distance_cm, heading_deg)
_pending_proj_removes: list[tuple[str, str]]                 # (u, v) to drop from EDGES at Save

# Render set each frame:
#   projection_render = (EDGES entries where type == EDGE_PROJECTION)
#                       ∪ _pending_proj_adds  −  _pending_proj_removes   (matched on (u, v))
```

Reset spec shape (unchanged, already validated by `_validate_reset_nodes`):

```python
RESET_NODES["<name>"] = {"axis": "x"|"y", "wall_coord": <float cm>, "facing": <float [0,360)>}
# Auto-fill: given node (x,y), axis, and FIELD_MM (mm → cm = /10):
#   axis "x": walls at x=0 and x=field_w_cm; pick the boundary with |wall - x| ≥ 14;
#             facing = 0.0 if wall==0 else 180.0   (front points away from wall, into field)
#   axis "y": walls at y=0 and y=field_h_cm; facing = 90.0 if wall==0 else 270.0
```

Projection `EDGES` entry written at Save (the four required keys only — no overrides):

```python
{"type": EDGE_PROJECTION, "u": "<u>", "v": "<v>", "distance_cm": <float>, "heading_deg": <float>}
```

## Error Handling

- **Invalid inline edit** (bad axis, out-of-range facing, k<0, etc.): reject, revert the widget, status-bar message — unchanged from #78's pattern.
- **Reset auto-fill can't clear 14 cm** (node hugs a wall on the chosen axis): write the best suggestion anyway, leave the badge Invalid; the Save validator is the authority and its precise message guides the fix.
- **Projection pick references an unknown / same node, or duplicates an existing EDGES projection pair:** reject before recording the delta; status-bar message; no render change.
- **LibCST EDGES append/remove failure at Save:** JSON is already written; show the modal (mirrors the existing rename-flush failure path at `app.py:2029`), keep the pending projection delta so the user can retry, return `False` from `_save`.
- **Save & Build with a build failure:** unchanged — captured stdout/stderr in a scrollable modal; no silent success.
- **Shortcuts dialog on a headless/odd backend:** it's plain ttk; build defensively and never hard-fail the editor if it can't open.

## Testing Strategy

**Levels:** Unit (LibCST round-trip, auto-fill geometry, command apply/revert, delta reconciliation, panels smoke, dialog construction). On-screen look + Save&Build-regenerates-routes are manual (GUI).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `append_projection_edge` then re-read EDGES | Unit | New `EDGE_PROJECTION` dict present with the 4 keys; file byte-stable elsewhere; re-running with the same pair is a no-op (mtime preserved). |
| TC2 | `remove_projection_edge` on an existing pair | Unit | The matching EDGES entry is dropped; unrelated entries untouched; removing an absent pair is a no-op. |
| TC3 | Projection delta reconcile (adds ∪ loaded − removes) | Unit | Render set matches expected; an add then a remove of the same pair nets empty; duplicate add rejected. |
| TC4 | Projection add → undo → redo → save-clear | Unit | Delta mutates at each of the 4 event points; Save clears it; undo after save behaves (mirrors rename-delta test). |
| TC5 | Auto-fill geometry, both axes, both walls | Unit | `wall_coord`/`facing` match the table above; a node ≥14 cm from a boundary yields a validator-passing spec; a wall-hugging node yields a flagged (Invalid) spec, not a crash. |
| TC6 | Auto-fill does not clobber a manual edit | Unit | After the user sets `facing` by hand, a non-geometry refresh leaves it; an axis change or node move recomputes. |
| TC7 | Add-node dialog offers reset; create routes via `SetResetSpec` | Unit | Dialog `result` allows `"reset"`; the create path issues `SetResetSpec` (type+spec atomic), never a bare `SetNodeType("reset")`. |
| TC8 | Reset type↔spec divergence still rejected at Save | Unit | A `"reset"` node with no `RESET_NODES` entry → `model.validate()` raises `FieldDataError` (regression guard, TC13 from #78). |
| TC9 | Panels build 3-pane headless; Save/Build buttons exist + wired + mapped | Unit | `SidePanels` constructs under Agg/shared-Tk-root; `_save_button`/`_save_build_button` exist, are mapped, and their commands invoke `on_save`/`on_save_and_build`; every `on_*` callback slot still present. |
| TC10 | `ShortcutsDialog` constructs and lists known bindings | Unit | Builds without raising; its text contains each registered shortcut label. |
| TC11 | Blit-equivalence still holds after canvas restyle | Unit | The #78 blit-equivalence test passes unchanged with the new facecolor/alpha + projection artists present. |
| TC12 | Full JSON round-trip unaffected by projection work | Unit | `from_json_dict(to_json_dict())` stable; projection edges are NOT in the JSON (they live in `field_data.py`). |

**Test data:** reuse the #78 headless patterns — `EditorApp.__new__` + injected collaborators for app-logic tests (force `matplotlib.use("Agg")`, no-op the import-time `use("TkAgg")`); the shared session `tk_root` conftest fixture + xvfb-in-CI for any real-`FieldCanvas`/`SidePanels` test. LibCST tests run against a temp copy of a minimal `field_data.py` containing an `EDGES = [ ... ]` block. **Run command:** `uv run --extra dev pytest tools/path_planner/tests/` (full suite — never just the new file; field/validator-adjacent changes break siblings).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Theme tokens for the new chrome | — | low | `tools/path_planner/editor/theme.py` | Add the named tokens the mockup needs, no magic numbers in callers: title font (Fraunces ~16), small-caps section-header style color (INK_MUTED) + hairline-rule color, ghost-button style, collapsible chevron glyphs (◢ open / ▸ collapsed), dirty-dot color (TERRACOTTA), active-mode accent-soft tint, status-badge valid/invalid colors. Reuse existing palette constants; only add what's missing. Satisfies AC1–AC3, AC10 (palette source). |
| T2 | Projection read + geometry helpers | — | med | `tools/path_planner/editor/projection_edges.py` (new) | Pure, Tk-free module: `read_projection_edges(field_data_module) -> list[tuple[str,str,float,float]]` (parse `EDGES` entries where `type == EDGE_PROJECTION`); `compute_projection_geometry(p_u, p_v) -> (distance_cm, heading_deg)`; `reconcile(loaded, adds, removes)` (the render-set union/diff matched on `(u,v)`); `reset_autofill(node_xy, axis, field_mm) -> (wall_coord, facing)` per the Data Model table. Fully unit-testable. Satisfies AC6, AC7 (geometry), AC8 (reconcile). |
| T3 | LibCST EDGES append/remove | — | high | `tools/path_planner/editor/ast_rewrite.py` | Add `append_projection_edge(path, u, v, distance_cm, heading_deg)` and `remove_projection_edge(path, u, v)` LibCST transforms alongside the existing `rewrite_edges_uv`. Byte-stable output; no-op preserves mtime (no `EDGES` change → no write); append targets the `EDGES = [...]` assignment only (reuse `_is_edges_assignment`); emit `type: EDGE_PROJECTION` as a bare name (not a string). Satisfies AC7, AC8 (persistence). |
| T4 | Projection + reset-autofill commands | T2 | med | `tools/path_planner/editor/commands.py` | `AddProjectionEdge`/`DeleteProjectionEdge` undoable commands operating on the app's pending-projection delta + render cache (apply/revert mutate the delta at the 4 event points per the rename precedent; reject unknown/same/duplicate pairs before mutating). Wire `SetResetSpec` to seed its default spec from `projection_edges.reset_autofill` when none is supplied. Satisfies AC5, AC6, AC8. |
| T5 | Dialogs: reset in Add-node + ShortcutsDialog | — | med | `tools/path_planner/editor/dialogs.py` | Widen `AddNodeDialog`'s `_NODE_TYPE_VALUES` to `("line","blind","reset")`; keep its `result` contract `(name, type)`. New `ShortcutsDialog(tk.Toplevel)` modal listing every keybinding grouped by category (modes / edit / edges / nudge / file), Escape + close-button to dismiss, returns focus to the parent. Satisfies AC5, AC9. |
| T6 | Panels: 3-pane layout + sections + button block + status strip + title | T1 | high | `tools/path_planner/editor/panels.py` | Restructure to the mockup: a dedicated **left mode toolbar** (vertical, active 3 px terracotta bar + accent-soft tint), a **title bar** (Fraunces, `field_data.json`, dirty dot ●), a **right properties inspector** with small-caps headers + hairline rules and **collapsible** Node / Reset spec / Pivots / Global-knobs sections (◢/▸), **ghost** "+ add pivot" / "+ add projection" buttons, a **button block** (Undo/Redo/Save/Save&Build, Save&Build terracotta-filled, always visible — fixes the clipping), a **tall 2-line status strip** (⬤ Valid/Invalid+count · MODE · selection · key hint, hint clickable → shortcuts). New `on_*` slots: `on_add_projection`, `on_show_shortcuts`. Preserve every existing `on_*` contract. Keep ttk only. Satisfies AC1–AC3, AC9. |
| T7 | Canvas: warm map + hairline grid + directed projection edges | T1, T2 | high | `tools/path_planner/editor/canvas.py` | Map PNG at alpha ~0.6 over the cream facecolor (omit `aspect=`, never `"auto"`); hairline grid; selected-node ring terracotta. Render projection edges from the reconciled render set as **directed** teal edges (arrowhead at head `v`), with retained artist handles consistent with the blit path so a dragged endpoint animates correctly and the blit-equivalence test still passes. Satisfies AC7, AC10. |
| T8 | App wiring: layout grid, save/build path, reset flow, projection flow, shortcuts | T3, T4, T5, T6, T7 | high | `tools/path_planner/editor/app.py` | Assemble the 3-pane grid + title/dirty-dot; confirm Save/Save&Build are wired + visible and verify the click→`validate`→`dump_field_json`→`rewrite_edges_uv`→`build_routes._run` path end-to-end. Add-node `reset` flow: dialog `"reset"` → create + `SetResetSpec` seeded default → auto-open Reset-spec section. Reset auto-fill recompute on axis/position change (no clobber of manual edits). Projection flow: a pick-`u`-then-`v` interaction → `AddProjectionEdge` (auto geometry) → pending delta + render; **flush `_pending_proj_adds`/`_pending_proj_removes` via T3 in `_save`** alongside renames; reconcile render set each refresh. Shortcuts: `?`/`F1` + status-hint → `ShortcutsDialog`. Satisfies AC1–AC9. |
| T9 | Tests + manual checklist | T2, T3, T4, T6, T8 | med | `tools/path_planner/tests/test_editor_projection.py` (new), `tools/path_planner/tests/test_editor_reset_autofill.py` (new), `tools/path_planner/tests/test_editor_panels_smoke.py` (extend), `tools/path_planner/tests/test_editor_inspector.py` (extend) | TC1–TC12 across LibCST round-trip, reconcile, auto-fill geometry, command apply/revert, add-node-reset routing, reset divergence regression, panels 3-pane + Save/Build button presence/wiring, shortcuts dialog, blit-equivalence, JSON round-trip. Add a manual visual+Save&Build checklist entry to `docs/editor-restyle-visual-checklist.md`. Run the FULL suite. Satisfies AC11 + verifies AC1–AC10. |

**Parallelism:** T1, T2, T3, T5 have no blockers and start together. Per-file chains keep parallel agents conflict-free: `theme.py` T1; `projection_edges.py` T2; `ast_rewrite.py` T3; `commands.py` T4; `dialogs.py` T5; `panels.py` T6; `canvas.py` T7; `app.py` T8 (last, integrates all); tests T9. No two tasks edit the same file.

## Notes for Implementer

- **The Save/Build fix is structural, not a logic rewrite.** Do not rewrite `_save`/`_save_and_build` — they work. The bug is the button block being clipped out of the 240 px panel; the 3-pane layout's dedicated button block is the fix. Then actually launch the editor and click Save & Build once to confirm `routes.h`/`routes.cpp` regenerate (the manual checklist item).
- **Projection edges follow the rename-delta law (`gotcha-editor-rename-edges-delta`):** track the add/remove delta at exactly four event points (edit / undo / redo / save-clear), never rebuild it from the undo/redo stacks (`History.do` clears redo). Reconcile the render set from `loaded ∪ adds − removes`; flush at Save; after a successful LibCST flush, reload the cached `field_data` module (as `_save` already does for renames) so the next read sees the new entries.
- **Do NOT add a `FieldModel` JSON field for projection edges** — they live in `field_data.py`'s `EDGES`. A new JSON field would hit the dual-serialization trap (`gotcha-fieldmodel-dual-serialization`) and isn't needed.
- **`EDGES` writes go through LibCST only** — do not regex/string-edit `field_data.py`. Emit `type: EDGE_PROJECTION` as a bare name reference (the module imports it), not a quoted string. Keep the no-op-preserves-mtime contract (`gotcha_ci_routes_freshness` lives nearby — a needless rewrite churns the file).
- **matplotlib aspect gotcha (`matplotlib_imshow_aspect_gotcha`):** when setting the map alpha/facecolor, never pass `imshow(aspect="auto")` — it clobbers `set_aspect("equal")` and stretches the field. Omit `aspect=`.
- **Headless test patterns are mandatory (`gotcha-editor-headless-matplotlib-agg`, `gotcha-editor-real-tk-tests-need-shared-root-and-xvfb`):** Agg + no-op `matplotlib.use` for `EditorApp.__new__` logic tests; the shared conftest `tk_root` (never a per-file root) + xvfb-in-CI for any real-`FieldCanvas`/`SidePanels` test.
- **Reset auto-fill suggests, never traps:** seed on type→reset and recompute on axis/position change, but preserve a value the user typed. The 14 cm clearance + facing-away geometry rule stays owned by `_validate_reset_nodes` at Save — the inline tier only does cheap well-formedness.
- **No `validator.py` / pipeline edits** — a contract change there breaks both Python fixtures and the C++ `*_strict` siblings; this plan only feeds the existing validators.
- **Run the FULL pytest suite**, not just new files; a field/fixture-adjacent change can redden siblings.
