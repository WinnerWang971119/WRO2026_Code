# Field Editor — Select Mode Plan

**Goal:** Add a dedicated **Select** mode to the path-planner field editor so the operator can pick a node by clicking directly on it in the canvas (same effect as clicking its row in the right-hand node list), instead of having no way to select-by-canvas-click.

**Approach:** Extend the existing `Mode` enum + mode-routing machinery already in the editor. Add `Mode.SELECT`, give it a rail row above Move, bind the `s` key to it, drop the `s` shortcut from Start (Start stays reachable from the toolbar), and route a Select-mode left-click through the existing `_select_node` path so the listbox + inspector update exactly as a listbox click would. No new selection model is introduced — Select mode reuses the selection plumbing that already exists.

## Scope

- **In scope:**
  - New `Mode.SELECT` enum member (value `"select"`) in `panels.py`.
  - A `Sel` row in the far-left MODES rail, placed **above** Move (Move remains the default mode).
  - `s` (and `S`) keyboard shortcut switches to Select mode; Start loses its keyboard shortcut.
  - Select-mode left-click on a node → selects it (listbox row highlighted, inspector populated, status/selection chips updated) — identical to clicking the node in the right panel.
  - Select-mode left-click on empty canvas → **clears** the current selection (listbox + inspector cleared).
  - Relabel Move's status-line / shortcut-help text from `"Move / select"` to `"Move"` (Move never actually selected into the inspector; a dedicated Select mode now owns that verb).
  - Status-surface mode label for Select; updated Modes shortcut-help group.
  - Headless unit tests for the new mode, shortcut remap, and both click branches.

- **Out of scope:**
  - Changing Move mode's drag behaviour (it still arms a drag on press; it does **not** gain select-into-inspector).
  - Multi-select / rubber-band node selection.
  - Any change to the model, validator, commands, or generated `routes.*`.
  - Reordering or renaming any mode other than Move's label cleanup.
  - No cursor change or hover-highlight affordance in Select mode — selection happens on click, with no canvas pre-hover feedback.

## Decisions

- **Separate mode, not "make Move also select":** The user explicitly asked for a new `Select` mode rather than overloading Move. Move's press still only arms a drag; selecting lives in its own mode. — Honors the request and keeps each mode's mouse semantics single-purpose.
- **Empty-canvas click clears selection (user choice):** In Select mode, clicking off any node deselects (clears listbox highlight + inspector), rather than a no-op. — Direct user decision in Phase 2.
- **Relabel Move → "Move" (user choice):** The legacy `"Move / select"` label was misleading (Move never populated the inspector). With a real Select mode it becomes plain `"Move"`. — Direct user decision in Phase 2.
- **Rail label "Sel" (user choice):** Matches the compact 3–4 char rail style (Move/Add/Edge/Obst/Strt). — Direct user decision in Phase 2.
- **Select placed above Move in the rail:** User said "on top of the move mode"; Move is currently the first rail row, so Select becomes the new first row and Move drops to second. Move stays the default selected mode (`mode_var` still initialises to `Mode.MOVE.value`). — Literal reading of the request; placement is display-only and does not change the default.
- **Reuse `_select_node` for the hit, `_on_panel_node_select(None)` for the clear — no parallel paths:** The hit branch calls the existing `_select_node(name)` (sets `_selected_node`, `_panels._selected_node_var`, selection chip) then `_refresh_view()`, the documented programmatic-selection path already used after node creation. The empty-click clear branch routes through the existing deselect handler `_on_panel_node_select(None)` (after pre-clearing `_panels._selected_node_var`), the same handler a cleared listbox selection uses — rather than hand-rolling a second clear. — Avoids a parallel selection code path in **both** directions.
- **Select hit reproduces the effect, it does NOT re-fire `on_node_select`:** "Just like clicking it on the right panel" means the same observable result (listbox highlight + inspector + chips), achieved via `_select_node`. The canvas handler does **not** call `self._panels.on_node_select(hit)` or simulate a listbox event — that callback is the panel→app inbound edge and re-firing it from the app would invert the flow. — Prevents a divergent implementation reading AC5 too literally.
- **`s` shortcut keeps the existing focus guard:** Binding `s`/`S` to Select changes only the target mode; the shortcut stays gated by the same focus guard that governed every mode key (inert while an Entry/Spinbox/Combobox has focus). No guard change. — `s` was already under this guard as Set-start; behaviour while typing is unchanged.
- **Approach consultation:** Skipped per `--fast` mode. The approach is effectively singular (extend the existing enum/router); the only alternative (overload Move) was explicitly rejected by the user.

## Acceptance Criteria

- [ ] `Mode.SELECT` exists with value `"select"`; `Mode(panels.mode_var.get())` round-trips it.
- [ ] The MODES rail shows a `Sel` row as the **first** row, above `Move`; Move is still selected by default on launch.
- [ ] Pressing `s` (or `S`) while the canvas has focus switches the active mode to Select (radio + accent + status mode-line all reflect it).
- [ ] Start mode has **no** keyboard shortcut (`Mode.START` is not a value in `_MODE_SHORTCUT_KEYS`); Start is still selectable by clicking its rail row.
- [ ] In Select mode, a left-click on a node selects it: that node's listbox row is highlighted, the inspector shows its name/type/coords, `_selected_node` equals the node, and the status "Selection:" chip shows it — matching a right-panel listbox click.
- [ ] In Select mode, a left-click on empty canvas clears the selection: `_selected_node` becomes `None`, the listbox has no highlighted row, and the inspector is in its empty/disabled state.
- [ ] The status-surface mode line reads `Mode: Select` in Select mode and `Mode: Move` in Move mode (no longer `Move / select`).
- [ ] The Shortcuts help dialog's "Modes" group lists `S → Select` and `M → Move` (no `Set start` shortcut row).
- [ ] Existing select-by-listbox, Move drag, Edge, Obstacle, Start, Add behaviours are unchanged; full `pytest` + `ctest` stay green.

## Contracts & Interfaces

Single source of truth for the one cross-file seam.

### Shared constant

- `Mode.SELECT` — enum member defined **once** in `tools/path_planner/editor/panels.py`, value string `"select"`. Owner: T1 (panels). Consumers: T2 (app.py — `_MODE_DISPLAY_LABELS["select"]`, `_MODE_SHORTCUT_KEYS["s"] = Mode.SELECT`, the `_on_canvas_click` router branch), T3 (tests).

### Signatures

- `EditorApp._handle_select_click(self, x: float, y: float) -> None` — owner: T2; consumer: T3 (TC6/TC7 call it by this exact name). The method must keep this name and 2-arg `(x, y)` shape; do not inline it into `_on_canvas_click`. It reproduces the selection effect via `_select_node` (hit) / `_on_panel_node_select(None)` (miss); it does **not** call `on_node_select`.

### Naming (exact strings that must match across files)

- Enum value: `"select"` (used as the `mode_var` string and the key into `_MODE_DISPLAY_LABELS`).
- Rail label: `"Sel"` (in `panels._MODE_LABELS`).
- Status label: `"Select"` (in `app._MODE_DISPLAY_LABELS["select"]`).
- Move status label changes `"Move / select"` → `"Move"` (in `app._MODE_DISPLAY_LABELS["move"]` and the `app._SHORTCUT_GROUPS` "M" row).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `tools/path_planner/editor/panels.py` | T1 | T2, T3 |
| `tools/path_planner/editor/app.py` | T2 | T3 |
| `tools/path_planner/tests/test_editor_select_mode.py` | T3 | — |

## Error Handling

- **Click resolves to no node (empty canvas) in Select mode:** Treated as a deliberate deselect — clear selection, status `"Selection cleared."`. Not an error.
- **`find_node_at` returns a stale name not in `model.nodes`:** `_select_node` writes the name into `_selected_node_var`; `panels.refresh` guards with `previously_selected in model.nodes` and falls back to `_clear_inspector()` if absent — so a stale hit degrades to a cleared inspector rather than a crash. (In practice `find_node_at` only returns live model nodes.)
- **Non-left mouse button:** Right-click still opens the context menu via the existing `_on_canvas_click` guard; middle/other buttons are ignored. Select-mode logic only runs for `_MOUSE_BUTTON_LEFT`.

## Testing Strategy

**Levels:** Unit (headless `EditorApp.__new__` + injected fakes, matplotlib forced to `Agg`), mirroring `test_editor_canvas_blit.py` / `test_editor_projection_node.py`.

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | `Mode.SELECT.value == "select"` and `Mode("select") is Mode.SELECT` | Unit | Enum member exists and round-trips. |
| TC2 | `_MODE_LABELS[0] == (Mode.SELECT, "Sel")` | Unit | Sel is the first rail row, above Move. |
| TC3 | `_MODE_SHORTCUT_KEYS["s"] is Mode.SELECT` and `Mode.START not in _MODE_SHORTCUT_KEYS.values()` | Unit | `s` → Select; Start has no shortcut. |
| TC4 | `_MODE_DISPLAY_LABELS["select"] == "Select"` and `_MODE_DISPLAY_LABELS["move"] == "Move"` | Unit | New label added, Move relabelled. |
| TC5 | `_SHORTCUT_GROUPS` "Modes" group contains `("S", "Select")` and `("M", "Move")`, and contains NEITHER the stale `("S", "Set start")` NOR `("M", "Move / select")` tuple | Unit | Help text updated — positive presence AND negative absence of the old tuples. |
| TC6 | `_handle_select_click(x, y)` with `find_node_at` returning `"N1"` → `_selected_node == "N1"`, `_panels._selected_node_var.get() == "N1"`, and the status text is `"Selected node 'N1'."` | Unit | Canvas click selects the node and emits the same status as a listbox click. |
| TC7 | `_handle_select_click(x, y)` with `find_node_at` returning `None` (and a node currently selected) → `_selected_node is None`, `_panels._selected_node_var.get() == ""` | Unit | Empty click clears the selection. |
| TC8 | Switch handler from `_make_mode_switch_handler(Mode.SELECT)` sets `panels.mode_var` to `"select"` | Unit | Mode-switch handler activates Select. |
| TC9 | With `mode_var == "select"`, `_update_mode_display()` sets the panel mode-display var to `"Mode: Select"`; with `mode_var == "move"` it sets `"Mode: Move"` (not `"Move / select"`) | Unit | Status mode-line reflects the new + relabelled modes (closes AC7/AC8). |
| TC10 | Drive the real `_wire_keyboard_shortcuts` against a binding recorder (mirror `test_editor_projection_node.py`'s `_BindingRecorder`): assert `<KeyPress-s>`/`<KeyPress-S>` are bound and their handler switches `mode_var` to `"select"`, and that NO binding switches to `Mode.START` | Unit | Binding-level proof that `s` → Select and Start has no key (closes AC3/AC4 at the registration layer, not just the dict). |
| TC11 | `_on_canvas_click(x, y, button=3)` in Select mode calls `_show_context_menu`, not `_handle_select_click` | Unit | Right-click still opens the context menu (Error-Handling claim). |

**Test data:** A small in-memory `FieldModel` with ≥2 nodes (reuse the fixture style in `test_editor_canvas_blit.py`); fake `_canvas` exposing `find_node_at`; real or `__new__` `SidePanels` providing `mode_var` / `_selected_node_var`. For TC6/TC7, drive `_handle_select_click` directly; stub `_refresh_view`/`_set_status`/`_update_selection_display` only where the headless instance lacks the widgets, following the existing tests' stubbing pattern.
**Run command:** `uv run --extra dev pytest tools/path_planner/tests/test_editor_select_mode.py` (and the full `uv run --extra dev pytest tools/path_planner/tests/` before finishing).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Add `Mode.SELECT` + rail row | — | low | `tools/path_planner/editor/panels.py` | Add `SELECT = "select"` to the `Mode` enum and a `:cvar SELECT:` docstring line ("Left-click selects the node under the cursor; empty click clears"). **Also correct the now-misleading `:cvar MOVE:` line** — it currently reads "Left-click selects; drag moves the picked node," but Move never selected into the inspector; change it to "Left-click arms a drag; drag moves the picked node." Insert `(Mode.SELECT, "Sel")` as the **first** entry of `_MODE_LABELS` (above `(Mode.MOVE, "Move")`). Optionally refresh the stale `set_mode_display` docstring example from `"Mode: Move / select"` to `"Mode: Select"`. Do NOT change `mode_var`'s default (stays `Mode.MOVE.value`). Satisfies AC1, AC2 (partial). |
| T2 | Wire Select into app routing, shortcut, labels, handler | T1 | low | `tools/path_planner/editor/app.py` | (a) In `_MODE_DISPLAY_LABELS`: change `"move": "Move / select"` → `"move": "Move"` and add `"select": "Select"`. (b) In `_MODE_SHORTCUT_KEYS`: replace `"s": Mode.START` with `"s": Mode.SELECT` (remove the Start entry); update the block comment that says "Plain `s` (Set start)" to "Plain `s` (Select)". (c) In `_SHORTCUT_GROUPS` "Modes": change `("M", "Move / select")` → `("M", "Move")`, change `("S", "Set start")` → `("S", "Select")`. (d) In `_on_canvas_click`, add `elif mode is Mode.SELECT: self._handle_select_click(x, y)`. (e) Add method `_handle_select_click(self, x, y)`: `hit = self._canvas.find_node_at(x, y)`; if `hit is None` → clear selection by pre-clearing the var then reusing the existing deselect handler: `self._panels._selected_node_var.set(""); self._on_panel_node_select(None)` (that handler nulls `_selected_node`, sets status `"Selection cleared."`, updates the selection chip, and refreshes — the pre-clear makes `refresh` drop the listbox highlight + inspector); else → `self._select_node(hit)`; `self._set_status(f"Selected node {hit!r}.")`; `self._refresh_view()`. Do NOT call `self._panels.on_node_select(...)`. (f) Update the `_wire_keyboard_shortcuts` / `_on_canvas_click` docstrings that enumerate A/M/E/O/S so they read Add/Move/Edge/Obstacle/Select (Start = toolbar only). Satisfies AC3, AC4, AC5, AC6, AC7, AC8. |
| T3 | Headless tests for Select mode | T1, T2 | low | `tools/path_planner/tests/test_editor_select_mode.py` | New test module following the `EditorApp.__new__` + forced-`Agg` headless pattern (see `test_editor_canvas_blit.py`, `test_editor_projection_node.py`). Cover TC1–TC11, including the `_BindingRecorder`-style binding test (TC10) that exercises the real `_wire_keyboard_shortcuts`. Force `matplotlib.use("Agg")` before importing app/canvas per the headless gotcha. Satisfies all ACs via verification. |

## Notes for Implementer

- **The clear path must reset BOTH `_selected_node` and `_panels._selected_node_var`.** `_on_panel_node_select(None)` nulls `_selected_node` and refreshes, but it does **not** itself blank `_selected_node_var`. `panels.refresh` restores the listbox highlight from `previously_selected = self._selected_node_var.get() or None`; if the var still holds the old name, refresh re-selects it. So pre-clear the var (`_selected_node_var.set("")`) **before** calling `_on_panel_node_select(None)` — then `previously_selected` falls to `None`, refresh calls `_clear_inspector()` (blanks the var + disables widgets) and the rebuilt listbox gets no `selection_set` → highlight cleared. (Verified against `panels.py` lines 1808–1831 and `_clear_inspector` at 2049.)
- **The hit path is already a solved problem:** `_select_node(name)` is documented as "Mirrors what `_on_panel_node_select` does for a user click, but driven programmatically" and is the exact call used right after a node is created. Reuse it; do not hand-roll listbox/inspector updates.
- **Select mode consumes only the press.** Drag and release stay no-ops for non-MOVE/OBSTACLE modes (`_on_canvas_drag` / `_on_canvas_release` already gate on `Mode.MOVE`/`Mode.OBSTACLE`), so a small click-drag in Select mode still just selects on press. No drag/release changes needed.
- **No existing test pins the changed strings.** `test_editor_inspector.py:350` (`_SAMPLE_SHORTCUT_GROUPS`) and `test_editor_panels_smoke.py:120` (`_SAMPLE_MODE_TEXT = "Mode: Move / select"`) are self-contained local fixtures testing the dialog/status widgets with their own input — they do **not** import the real `_SHORTCUT_GROUPS` / labels, so they won't break. `_SAMPLE_MODE_TEXT` is now a stale example string; updating it is optional cosmetic cleanup, not required.
- **Tk dispatch for `s` vs `Ctrl-S`:** Binding plain `<KeyPress-s>` to Select does not shadow `Ctrl-S` (Save) — Tk fires the most specific binding for a modified keypress. The existing comment block already explains this; just retarget its wording from "Set start" to "Select".
- **Run the full suite, not just the new file** (`uv run --extra dev pytest tools/path_planner/tests/` and the host `ctest`), per the field-contract / fixture gotchas — although this change is GUI-only and touches no field/route data, so cross-suite breakage is not expected.
- **Rollback:** Pure additive + label edits in two files plus one new test file; revert the three files to restore prior behaviour.
