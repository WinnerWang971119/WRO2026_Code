# Field Editor: De-lag + UX Pass + Property Inspector + Restyle (#78) Plan

**Goal:** Make the Tk + matplotlib field editor fast and pleasant to use — kill the per-frame drag lag, add the missing keyboard/inspector ergonomics, let the operator tune every JSON-native node/edge property (pivots, reset specs, global routing knobs) in the GUI, and replace the default-grey Tk look with a warm, readable theme.

**Approach:** One spec, **three sequenced waves** that land as independently shippable stacked PRs under #78 (failure domains are disjoint, so a regression in one never blocks the others): **W1 Perf** (cache the background PNG, then a drag-scoped matplotlib blit path that also skips the Tk listbox rebuild during drag), **W2 Inspector** (additive sub-panels + undo-`Command` classes for pivots / reset specs / global knobs + shortcuts + arrow-nudge, with two-tier validation: cheap inline well-formedness guards + the authoritative validator at Save), **W3 Restyle** (a warm ttk/sv-ttk theme + matplotlib facecolors, landed last so it rebases over the new widgets once and never injects HiDPI scaling near the blit path). The work is **editor-only** — `FieldModel` and the validator already carry and enforce every property #78 surfaces.

## Scope

- **In scope:**
  - **Perf:** stop `FieldCanvas.render()` re-reading the map PNG from disk every call (persistent cached background `AxesImage`); a drag-scoped blit API (`begin/update/end_node_drag`) animating only the dragged node + its incident edges + label; ~60 Hz motion throttle; **skip `panels.refresh()` during a live drag** and re-sync the inspector on release.
  - **UX:** new keyboard shortcuts (add-node, mode-switch keys); a bigger/clearer status surface (mode + selection + validation badge); **arrow-key nudge** of the selected node by **0.2 cm** (Shift = 1 cm coarse), clamped to field bounds, suppressed while typing in a text field.
  - **Property inspector:** contextual inspector sub-panels to edit **pivots** (per-turn `u→v→w` triples), **reset-node specs** (`{axis, wall_coord, facing}` + exposing the `"reset"` node type), and **global knobs** (`blind_decay_enabled/k/exponent/d_ref`, `robot_radius_cm`) — all through undoable `Command` classes, round-tripping via the existing Save → `field_data.json`.
  - **Validation:** two-tier — a cheap inline reject (status-bar message, revert widget) using lightweight well-formedness guards (axis ∈ {x,y}, knob ranges, referenced-node existence, pivot-leg checks), plus the authoritative full `model.validate()` unchanged at Save.
  - **Restyle:** a warm "drafting table" palette applied to the panels (via `ttk.Style` / `sv-ttk`) and to the matplotlib figure/axes facecolors so the map area isn't grey; numeric data in a mono font; node/edge palette aligned (reset nodes teal, selection ring terracotta). Driven by the frontend-design skill at implementation time.
- **Out of scope:**
  - **Per-edge mode/direction overrides** (`primitive_mode`/`direction`, ADR-022 enums JSON can't hold) — **deferred to #79.**
  - **customtkinter** rounded-elegance restyle — **deferred to #83** (filed); #78 uses the safe ttk/sv-ttk route only.
  - **Snap-to-grid** and **multi-select + move** — explicitly dropped by the user.
  - Web front end (#81), surfacing *all* `field_data` settings beyond the above (#80), validator/planner/header/firmware changes, the rich `EDGES` table.

## Decisions

- **Three sequenced waves, stacked PRs (not one PR).** Adopted from the approach consult: perf / inspector / restyle have disjoint blast radii; the restyle is the riskiest and must not hold the headline perf fix hostage. Restyle lands last so it rebases over the new inspector widgets exactly once.
- **Cache the background PNG first.** The dominant *static-render* lag is `mpimg.imread()` decoding the PNG from disk on *every* mouse-motion tick; caching it removes that cost (AC2) but does **not** by itself make the drag smooth — during a drag the smoothness comes entirely from T2's blit path bypassing `render()`. Cache-first is sequenced first because it is small and obviously-correct, not because it removes the blit risk (T2/T3 stay the high-risk tasks).
- **Drag path must skip `panels.refresh()`.** `_refresh_view` rebuilds the entire Tk listbox per tick — a second, non-matplotlib lag source. The drag path bypasses it and re-syncs the inspector x/y on release.
- **Editor-only at the data layer; the editor's command/UI layer DOES change.** No validator/model/pipeline edits — `FieldModel` already carries `pivot_turns`, `reset_nodes`, the global knobs; the validator already enforces them and `validator._ALLOWED_NODE_TYPES` already includes `"reset"`. But surfacing the `"reset"` type requires widening the editor's **three** node-type whitelists from `{"line","blind"}` to include `"reset"`: `commands._VALID_NODE_TYPES` (`commands.py:34`, used by `AddNode` and `SetNodeType`), `app._VALID_NODE_TYPES` (`app.py:105`), and `panels._NODE_TYPE_VALUES` (`panels.py:77`), plus their type-change guards. #78 adds GUI + `Command` classes + this whitelist widening.
- **Two-tier validation.** The inline tier is *cheap well-formedness guards only* — reset axis ∈ {x,y}, knob ranges (k ≥ 0, exponent ≥ 1, d_ref > 0, robot_radius > 0), referenced nodes exist, a pivot triple's two legs are real incident edges. Do **not** import `validator._validate_pivot_turns`/`_validate_reset_nodes` for the inline tier — they take a whole `field_data` namespace, parse `Config.h`, and rebuild a networkx graph (far too heavy per keystroke). The authoritative geometry rules live solely in `model.validate()` at Save (unchanged). The inline guards never re-implement the geometry rules — they catch obvious bad input early; the Save validator stays the single source of truth. No edit to `validator.py`.
- **Reset type ↔ spec is an atomic invariant owned by `SetResetSpec`/`ClearResetSpec`.** `SetResetSpec` sets `node_types[name]="reset"` **and** creates/updates the `reset_nodes` entry in one command; `ClearResetSpec` removes the `reset_nodes` entry **and** reverts the type away from `"reset"`. The plain `SetNodeType` path must NOT be used to reach or leave `"reset"` (either route a `"reset"` selection through `SetResetSpec`, or guard `SetNodeType` so it refuses `"reset"`); otherwise a node can be typed `"reset"` with no spec and `validate()` rejects at Save (`_validate_reset_nodes` requires every `"reset"`-typed node to have a `RESET_NODES` entry and vice-versa).
- **Styling: ttk/sv-ttk + matplotlib facecolors, NOT customtkinter.** Fable vetoed customtkinter: it swaps the whole widget tree and sets global Tk HiDPI scaling into the interpreter the blit path's device-pixel math depends on (same family as the CTPS HiDPI ghosting gotcha). User chose the safe route + a follow-up issue (#83).
- **Verifiability via Agg image buffers.** A blit-equivalence test (artist data + `restore_region`/`blit` invocation spy at the same model state) + a panels smoke/contract test convert "GUI, untestable" into headless CI assertions.

## Acceptance Criteria

- [ ] AC1 — Dragging a node over the real field map is smooth, no visible redraw stutter: no per-frame disk read, no per-frame listbox rebuild, blit path active during the drag.
- [ ] AC2 — `FieldCanvas.render()` no longer calls `mpimg.imread()` per call; the decoded PNG is loaded once and reused; discrete-edit renders are visually unchanged.
- [ ] AC3 — After a blit drag-update, the dragged node + incident-edge artists hold the same data a full `render()` would produce at that state, and the blit compositing calls (`restore_region`/`blit`) are exercised (verified by artist-data + invocation spy, not pixel equality).
- [ ] AC4 — Arrow keys nudge the currently-selected node by 0.2 cm (Shift = 1 cm), clamped to field bounds, and do nothing while a text-entry/spinbox/combobox holds focus.
- [ ] AC5 — Add-node and mode-switch keyboard shortcuts work; the existing Ctrl-Z/Y/S, Delete, L, B shortcuts still work and are unbroken by the new bindings.
- [ ] AC6 — The inspector can add/remove pivot triples, set/clear a reset-node spec (with `"reset"` selectable as a node type), and edit every global knob; each is a single undoable action and round-trips through Save into `field_data.json`.
- [ ] AC7 — An obviously-invalid property edit (reset axis ∉ {x,y}, `k < 0`, pivot triple referencing an unknown node / not an incident turn, out-of-range facing) is rejected inline with a status-bar message and the widget reverts; Save still runs the full validator and refuses an invalid model — including a reset node left without a spec (type↔spec divergence, TC13).
- [ ] AC8 — The status surface is visibly bigger/clearer and shows current mode, current selection, and a valid/invalid badge.
- [ ] AC9 — The editor is no longer default grey: the warm palette is applied to the panels and to the matplotlib figure/axes facecolors (no customtkinter dependency added).
- [ ] AC10 — The full `uv run --extra dev pytest tools/path_planner/tests/` suite is green, including new blit-equivalence, inspector/command, and panels-smoke tests; no existing editor test regresses.
- [ ] AC11 — No per-edge `EDGES` override editing is added (deferred to #79); the customtkinter follow-up (#83) is filed.

## Data Model (no change — reference only)

All fields already exist on `FieldModel` (`tools/path_planner/editor/model.py`) and round-trip JSON; the validator already enforces them. #78 adds **no** model fields.

```python
# Already present on FieldModel — the inspector edits these in place via Commands:
pivot_turns: list[tuple[str, str, str]]          # (arrive u, pivot v, leave w)
reset_nodes: dict[str, dict]                      # name -> {"axis": "x"|"y", "wall_coord": float, "facing": float}
node_types: dict[str, str]                        # editor currently allows ("line","blind"); #78 widens the editor whitelists to add "reset" (validator already permits all three)
blind_decay_enabled: bool
blind_decay_k: float                              # >= 0
blind_decay_exponent: float                       # >= 1
blind_decay_d_ref: float                          # > 0  (cm)
robot_radius_cm: float                            # > 0
```

Validator authorities already in `tools/path_planner/validator.py`: `_validate_pivot_turns`, `_validate_reset_nodes`, `_ALLOWED_NODE_TYPES = ("line","blind","reset")`, `_ALLOWED_RESET_AXES = ("x","y")`. These run ONLY at Save (full `model.validate()`); the inline tier uses cheap hand-written well-formedness guards, NOT these heavy whole-field functions (see Decisions §).

## Canvas Blit Contract (W1)

```
FieldCanvas.begin_node_drag(name):
    full render() once; set the dragged node's dot + incident line/blind edges + its label artist.animated = True;
    canvas.draw(); background = canvas.copy_from_bbox(ax.bbox)
FieldCanvas.update_node_drag(name, x, y):     # called from app._on_canvas_drag, throttled ~60 Hz
    move the animated artists' data to (x, y) and the incident-edge endpoints;
    canvas.restore_region(background); for each animated artist: ax.draw_artist(artist); canvas.blit(ax.bbox)
FieldCanvas.end_node_drag():
    set animated = False on all; full render() (re-bakes the static scene)
```

App side (`app._on_canvas_drag` / `_finish_move`): during a live MOVE drag call `update_node_drag` only — do **not** call `_refresh_view`/`panels.refresh`. On release, restore origin, dispatch `MoveNode`, then one full `_refresh_view` re-syncs the inspector.

**T2/T3 are a tight contract:** the `update_node_drag(name, x, y)` signature is shared; a mismatch is a runtime `TypeError`. T2 and T3 should land in the same PR; if split, T4 must exercise the real T2 canvas surface (not a stub) so any drift fails the suite. The blit path also needs **persistent, retained artist handles** for the dragged node's incident line/blind edges and its label — the current `_draw_edges` calls `ax.plot` per edge every render and keeps no handles, so T2 must restructure edge-artist creation to retain those handles before it can animate incident edges.

## Error Handling

- **Invalid inline property edit:** reject, revert the widget to the model value, write a specific message to the status bar (matches the existing coords/heading validation pattern). Never write a half-valid value into the model.
- **Reset spec defaults invalid until edited:** setting type → `"reset"` creates a default spec that may not yet satisfy geometry; the status badge shows invalid and Save is blocked until the operator fills valid values. This is intended (the type change opens the editor; it doesn't fabricate a passing spec).
- **Missing / unreadable map PNG (cache path):** keep the current behavior — warn once, render on the default facecolor; the cache stores "no background" so it isn't re-attempted every frame.
- **Blit unsupported / raises** (exotic backend): fall back to `end_node_drag()`'s full-render path for that tick so a drag still works, just without the fast path.
- **Save validation failure:** unchanged — modal error from `model.validate()`, no JSON written.

## Testing Strategy

**Levels:** Unit (commands, inline validation, throttle, blit-equivalence via Agg buffers, panels smoke). The actual on-screen smoothness and the look are manually verified (GUI).

| ID   | Test Case                                                        | Type | Expected Behavior |
|------|-----------------------------------------------------------------|------|-------------------|
| TC1  | `render()` called twice does not re-`imread` the PNG            | Unit | `mpimg.imread` invoked once (patched/counted); second render reuses cached array |
| TC2  | Blit drag-update updates artist data + composites               | Unit | After `update_node_drag`, the dragged node + incident-edge artists' data equal the post-`render()` positions, AND `restore_region`/`blit` were invoked (spy). No pixel `allclose` (font/AA/version-flaky) |
| TC3  | Motion throttle coalesces sub-16 ms ticks                      | Unit | N rapid `update_node_drag` calls within one frame window collapse to ≤1 blit |
| TC4  | Drag does not rebuild the listbox                              | Unit | `panels.refresh` not called during `_on_canvas_drag`; called once on release; inspector x/y reflect final pos |
| TC5  | AddPivotTurn / DeletePivotTurn apply→revert                    | Unit | Model `pivot_turns` mutates then restores byte-identical |
| TC6  | SetResetSpec / ClearResetSpec + type↔spec sync                 | Unit | Setting type `reset` creates entry; clearing/removing type drops it; both agree |
| TC7  | SetGlobalKnob apply→revert (each knob)                         | Unit | Knob set then reverted; bad value (k<0) rejected before mutation |
| TC8  | Inline reject: bad reset axis / k<0 / bogus pivot triple       | Unit | Callback not dispatched; status message set; widget reverts; model untouched |
| TC9  | Arrow nudge 0.2 cm / Shift 1 cm, clamp, focus-guard            | Unit | Selected node moves by the step; clamped at bounds; suppressed when a text widget has focus |
| TC10 | Mode-switch + add-node shortcuts; existing shortcuts intact    | Unit | Each key fires its handler; L/B/Delete/Ctrl chords unchanged |
| TC11 | Panels smoke after restyle                                     | Unit | `SidePanels` builds headless without raising and still exposes every `on_*` callback attribute |
| TC12 | Full JSON round-trip with pivots+reset+knobs edited            | Unit | `from_json_dict(to_json_dict())` stable; Save writes the edited values |
| TC13 | Reset type↔spec divergence rejected at Save                    | Unit | A node typed `reset` with no `RESET_NODES` entry (or a spec whose node isn't `reset`) → `model.validate()` raises `FieldDataError` |

**Test data:** reuse the headless `EditorApp.__new__` + injected-collaborator pattern from `test_editor_app_renames.py` (force `matplotlib.use("Agg")`, no-op the import-time `use("TkAgg")`). `field_data_small.json` (3 nodes: A/B line, G blind; **no** pivots, **no** reset nodes) is the base only for tests that need no pivot/reset/knob state. The pivot/reset/knob tests (TC5, TC6, TC12, TC13) must construct their state explicitly — either by applying the new `AddPivotTurn`/`SetResetSpec`/knob-edit commands to a base model, or via a dedicated `FieldModel`/JSON fixture carrying at least one pivot triple, one reset-typed node with its spec, and a non-default knob value. Any authored pivot/reset geometry must satisfy the validator so the round-trip (TC12) and Save assertions are meaningful. Commands tested directly against a `FieldModel` (no Tk). Blit tests construct a real `FieldCanvas` under Agg and assert artist data + spy the `restore_region`/`blit` calls.
**Run command:** `uv run --extra dev pytest tools/path_planner/tests/` (full suite — a field-fixture or validator-adjacent change can break siblings; never run only the new file).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Persistent cached background | — | med | `tools/path_planner/editor/canvas.py` | Load the map PNG once (cache `mpimg.imread` keyed by resolved path; store decoded array + a persistent `AxesImage`). `render()` reuses it instead of re-reading disk; keep `ax.clear()` for discrete edits. Preserve `imshow` aspect handling (omit `aspect=` so it inherits `set_aspect("equal")` — do NOT pass `"auto"`). Cache a "no background" sentinel when the PNG is missing. Satisfies AC2; sets up AC1. |
| T2 | Drag-scoped blit API | T1 | high | `tools/path_planner/editor/canvas.py` | Add `begin_node_drag(name)/update_node_drag(name,x,y)/end_node_drag()` per the Blit Contract: animate the dragged node dot + incident line/blind edges + label, `copy_from_bbox` the static background, `restore_region`+`draw_artist`+`blit` on motion; ~60 Hz throttle (millis gate). Fall back to full render if blit raises. **Restructure edge-artist creation so the dragged node's incident line/blind edges (and its label) have persistent, retained `Line2D`/artist handles** — the current `_draw_edges` calls `ax.plot` per edge every render and keeps no handles, so the blit path can't animate incident edges without this; the node dot also needs its own dedicated artist. Keep `update_node_drag(name, x, y)` signature in lockstep with T3 (shared contract). Satisfies AC1, AC3. |
| T3 | App drag path: blit + skip listbox | T2 | high | `tools/path_planner/editor/app.py` | `_on_canvas_drag` (MOVE) calls `canvas.update_node_drag` only; do NOT call `_refresh_view`/`panels.refresh` mid-drag. `_handle_move_click`→`begin_node_drag`; `_finish_move`→`end_node_drag` + restore origin + dispatch `MoveNode` + one `_refresh_view` to re-sync inspector. Satisfies AC1, AC4(prep). |
| T4 | Blit + throttle tests | T2, T3 | med | `tools/path_planner/tests/test_editor_canvas_blit.py` (new) | Agg-backed: TC1 (no re-imread), TC2 (artist-data correctness + `restore_region`/`blit` spy — no pixel `allclose`), TC3 (throttle), TC4 (no listbox rebuild during drag; inspector synced on release). Must exercise the **real** T2 canvas surface (not a stub) so a T2/T3 signature drift fails the suite. Med, not low: this is where a T2/T3 contract drift surfaces. Use the Agg import dance + a real `FieldCanvas`. Satisfies AC1–AC3, AC10. |
| T5 | Property Command classes | — | med | `tools/path_planner/editor/commands.py` | Widen `commands._VALID_NODE_TYPES` (`commands.py:34`) to include `"reset"` and update the `AddNode`/`SetNodeType` guards. New `@dataclass` commands following the `apply`/`revert` Protocol: `AddPivotTurn`/`DeletePivotTurn`; `SetResetSpec`/`ClearResetSpec` which own the **atomic** type+spec change (`SetResetSpec` sets `node_types[name]="reset"` AND creates/updates the `reset_nodes` entry; `ClearResetSpec` removes the entry AND reverts the type) — the plain `SetNodeType` must refuse `"reset"` so the invariant can never be saved divergent; `SetGlobalKnob` (one per knob or a keyed setter for blind_decay_*/robot_radius). Capture inverse data; reject illegal pre-conditions before mutating. Satisfies AC6. |
| T6 | Inspector sub-panels + status surface | — | high | `tools/path_planner/editor/panels.py` | Widen `panels._NODE_TYPE_VALUES` (`panels.py:77`) to include `"reset"` and update the `_handle_type_change` guard. Add contextual Properties sections: node geometry with `"reset"` selectable in the type combobox; a Reset-spec sub-panel (axis combo x/y, wall_coord entry, facing spinbox); a Pivots list (rows `u→v→w` + remove, an add row with node dropdowns); a Global-knobs panel (blind_decay_enabled checkbox, k/exponent/d_ref/robot_radius entries). Enlarge the status surface (multi-line) + a validation badge. New `on_*` callback slots for each. Keep widgets plain tk/ttk here (restyle is W3). Satisfies AC6, AC8. |
| T7 | App wiring + two-tier inline validation | T3, T5, T6 | high | `tools/path_planner/editor/app.py` | Widen `app._VALID_NODE_TYPES` (`app.py:105`) + the `_on_panel_node_type_change` guard to include `"reset"`, and route a `"reset"` selection through `SetResetSpec` (never plain `SetNodeType`). Wire the new panel callbacks to dispatch the T5 commands. Inline tier: cheap hand-written well-formedness guards (axis ∈ {x,y}; k ≥ 0 / exponent ≥ 1 / d_ref > 0 / robot_radius > 0; referenced nodes exist; a pivot triple's two legs are real incident edges) — do NOT import the heavy `_validate_*` whole-field functions per keystroke; on failure, status message + revert + no dispatch. Keep full `model.validate()` at Save unchanged. Satisfies AC6, AC7. |
| T8 | Shortcuts + arrow-key nudge | T7 | med | `tools/path_planner/editor/app.py` | Add add-node + mode-switch shortcuts via `_wire_keyboard_shortcuts` (route through the existing `_shortcut_guard` focus check). Arrow keys nudge the selected node by `_NUDGE_FINE_CM=0.2` (Shift→`_NUDGE_COARSE_CM=1.0`) via a `MoveNode` dispatch, clamped to field bounds, only when a node is selected and focus isn't a text widget. Satisfies AC4, AC5. |
| T9 | Inspector + command + nudge tests | T5, T7, T8 | low | `tools/path_planner/tests/test_editor_inspector.py` (new), `tools/path_planner/tests/test_editor_property_commands.py` (new) | TC5–TC9, TC12, TC13 against bare `EditorApp.__new__` + direct command tests; construct pivot/reset/knob state explicitly (see Test data). Cover reset type↔spec sync, the inline reject, AND the Save-time divergence rejection (TC13). Satisfies AC6, AC7, AC10. |
| T10 | Theme module (palette/fonts/mpl) | — | med | `tools/path_planner/editor/theme.py` (new), `tools/path_planner/editor/assets/fonts/` (new) | Constants for the warm palette (paper/ink/terracotta hexes), a font-loading helper with fallback chain (Fraunces→Cambria, IBM Plex Sans→Segoe UI, IBM Plex Mono→Cascadia Mono), a `ttk.Style`/`sv-ttk` setup function, and a matplotlib facecolor helper (figure/axes/grid/spines/ticks). No magic numbers in callers — everything sourced here. Driven by the frontend-design skill. Satisfies AC9. |
| T11 | Apply theme to panels | T6, T10 | high | `tools/path_planner/editor/panels.py` | Restyle the (now T6) widget tree with the theme: left mode toolbar with active-mode accent bar, themed entries/combos/buttons, the bigger status strip + validation badge, mono for numeric fields. Use `ttk` widgets + `ttk.Style`; no customtkinter. Preserve every `on_*` callback contract. Satisfies AC8, AC9. |
| T12 | Apply theme to canvas | T2, T10 | med | `tools/path_planner/editor/canvas.py` | Set figure/axes facecolors + grid/spine/tick colors from the theme; node palette (line=ink, blind=brick, reset=teal), selection ring → terracotta. Must NOT disturb the blit path (re-run T4 equivalence); keep `imshow` aspect correct (no `aspect="auto"`). Satisfies AC9. |
| T13 | App theme init + mode toolbar wiring | T8, T10 | med | `tools/path_planner/editor/app.py` | Apply the theme at startup (root bg, window chrome, ttk style), wire the left mode toolbar to `mode_var`. No behavior change. Satisfies AC9. |
| T14 | Restyle smoke test + manual checklist | T11, T13 | low | `tools/path_planner/tests/test_editor_panels_smoke.py` (new) | TC11: `SidePanels` builds headless and still exposes every callback attr after restyle. Plus a manual visual checklist (no grey, palette applied, fonts loaded with fallback). Satisfies AC9, AC10. |

**Wave grouping:** W1 Perf = T1–T4 · W2 Inspector = T5–T9 · W3 Restyle = T10–T14. T5, T6, T10 have no blockers and can start in parallel with T1. Per-file chains keep parallel agents conflict-free: `canvas.py` T1→T2→T12; `app.py` T3→T7→T8→T13; `panels.py` T6→T11.

## Notes for Implementer

- **Headless test pattern (mandatory):** editor imports call `matplotlib.use("TkAgg")` at import time. In tests force `matplotlib.use("Agg")` and no-op `matplotlib.use` across the import, then build `EditorApp.__new__(EditorApp)` and inject only the collaborators you touch (see `test_editor_app_renames.py`). Blit tests need a real `FieldCanvas` under Agg.
- **matplotlib aspect gotcha:** when touching `_draw_background`/facecolors, never pass `imshow(aspect="auto")` — it clobbers `set_aspect("equal")` and stretches the field on resize. Omit `aspect=` or pass `"equal"`.
- **No `FieldModel` field additions** — every property is already there and validated. If a future tweak *does* add a model field, it must touch BOTH `to_json_dict`/`from_json_dict` AND `to_module_proxy()` (the dual-serialization trap) — but #78 should not need this.
- **Surfacing `"reset"` touches three editor whitelists** — `commands._VALID_NODE_TYPES` (`commands.py:34`), `app._VALID_NODE_TYPES` (`app.py:105`), `panels._NODE_TYPE_VALUES` (`panels.py:77`) and their type-change guards all currently exclude it; widen all three. The `"reset"` type and its `RESET_NODES` spec must be set/cleared **atomically** via `SetResetSpec`/`ClearResetSpec`, never via the plain `SetNodeType` path, or Save's `_validate_reset_nodes` rejects the divergent state (every `"reset"` node needs a spec and vice-versa).
- **Inline validation = cheap well-formedness guards only** — axis ∈ {x,y}, knob ranges, node existence, pivot-triple legs are real edges. Do NOT import `_validate_pivot_turns`/`_validate_reset_nodes` for the inline tier (they parse `Config.h` + rebuild a networkx graph — far too heavy per keystroke). The authoritative geometry stays in `model.validate()` at Save. Do not edit `validator.py` (a rule/contract change there breaks both Python fixtures and C++ `*_strict` siblings).
- **Blit + DPI:** the blit path uses device-pixel bbox math; keep Tk scaling at its default and verify under the machine's display scaling. This is exactly why customtkinter (global HiDPI scaling) is deferred to #83.
- **Font bundling:** if a bundled `.ttf` can't be private-loaded, fall back silently down the chain — never hard-fail the editor over a font.
- **Run the FULL pytest suite**, not just the new files — editor/validator-adjacent changes can break sibling fixtures.
- **Rollback:** each wave is its own PR; reverting the restyle PR leaves perf + inspector intact, and reverting inspector leaves perf intact.
