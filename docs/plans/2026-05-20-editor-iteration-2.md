# WRO 2026 Live Field Editor — Iteration 2 Plan

**Goal:** Fix the listbox-select refresh bug and add three editor features (typeable x/y, combined add-node dialog, auto-create-edges with preview) so the user can iterate on the field graph without leaving the editor.

**Approach:** One-line bug fix to plug the missing refresh path. Add a typeable x/y row to the inspector and a `MoveNode` dispatch on commit. Replace the two-step add-node prompt with a single `AddNodeDialog` Toplevel. New `auto_edges` helper computes Delaunay + length-cap edges, filtered by obstacles, classified by node type; a new `AddEdgesBatch` command commits them atomically; the canvas renders a preview overlay until the user clicks Accept or Reject.

## Scope

- **In scope:**
  - Bug fix: `EditorApp._on_panel_node_select` refreshes the view so the inspector shows the freshly selected node's coords/type.
  - Inspector: replace the read-only `(x, y) cm` label with two typeable `Entry` widgets that commit on `<Return>` or `<FocusOut>`, validate floats within field bounds, and dispatch `MoveNode` through history. Live-sync with MOVE-mode drag.
  - Add-node dialog: one combined Toplevel with a Name `Entry` and a Type `Combobox`. Replaces the two-step `simpledialog` flow.
  - Auto-create-edges: new button + max-edge-length Spinbox in the **Nodes** labelframe. On click, compute proposed edges (Delaunay → length cap → obstacle filter → type rule → skip existing pairs). Preview overlay drawn on the canvas in dashed cyan. **Accept** commits via a new `AddEdgesBatch` command (single undo entry); **Reject** clears the preview without mutation. Status bar reports counts.
  - Tests: unit coverage for `AddEdgesBatch` (mirroring existing command tests) and for the `auto_edges` helper (Delaunay, length cap, obstacle filter, type rule, degenerate inputs).
- **Out of scope:**
  - Editing obstacle x/y in the inspector. (Obstacle mode handles creation by drag; deletion is not in this PR either.)
  - Renaming or deleting from the listbox (still right-click only).
  - Changing the static visualizer (`visualizer.py`) or `pick_xy.py`.
  - Bidirectional edge-type editing on existing edges (only the auto-create path applies the type rule; manual L/B in EDGE mode is unchanged).
  - An ADR — these are local editor features, not a project-wide rule.

## Decisions

- **Bug #1 fix location:** `EditorApp._on_panel_node_select` (app.py) — single source of truth for "user picked a node from the list". Adding `_refresh_view()` here fixes the inspector path *and* keeps the canvas's `highlighted_nodes` in sync if we ever route listbox selection through it. Alternative (cache the model on `SidePanels` and call `_apply_inspector` from `_handle_listbox_select`) was rejected as more state to track in two places.
- **x/y commit trigger:** `<Return>` *and* `<FocusOut>`, matching the existing heading Spinbox pattern. Same `_suppress_callbacks` re-entrancy guard.
- **x/y validation:** reject out-of-range, keep prior value, surface the rejection in the status bar. No silent clamp.
- **Drag-time entry sync:** the MOVE-mode drag already calls `_refresh_view` on each motion event; `refresh()` will re-populate the new Entry widgets from the model, so live updates come for free.
- **Add-node dialog:** custom Toplevel in a new `tools/path_planner/editor/dialogs.py` module. One file per dialog class (matches the one-class-per-file convention).
- **Neighbour-limit rule:** Delaunay triangulation + max-edge-length cap. `shapely.ops.triangulate` (already in deps) returns triangle polygons; we extract their edges (deduplicated, undirected).
- **Max-edge-length:** user-tunable `ttk.Spinbox` (default 60 cm, range 10–300 cm, 5 cm step) directly above the auto-create button in the **Nodes** group. Value stored on `SidePanels` as a `tk.StringVar`, read by `EditorApp` when the user clicks Auto-create.
- **Obstacle filter:** reuse `graph_builder.edge_clips_obstacle` against `graph_builder.inflate_obstacles(model.obstacles, model.robot_radius_cm)` — same check the static planner uses. Consistency over duplication.
- **Type rule:** `line + line → line` segment; `any blind → blind` edge. Encoded as a `_classify_edge_type(node_types, a, b)` helper in `auto_edges.py`.
- **Existing-pair handling:** skip pairs that already have an edge in either `line_segments` or `blind_edges` (undirected). Hand-authored work is sacred.
- **Apply UX:** preview overlay + Accept/Reject. Preview state held on `EditorApp` (`_proposed_edges: list[tuple[str, str, str]]`, `None` when no preview is active). The canvas exposes a new `proposed_edges` kwarg parallel to `highlighted_nodes`.
- **Undo granularity:** single `AddEdgesBatch` command — one Ctrl-Z reverts all auto-created edges.
- **Degenerate inputs (< 3 nodes / collinear / no edges survive filtering):** status bar message, no preview, no commit. Button is always enabled.

## Acceptance Criteria

- [ ] **AC1 (Bug #1):** Clicking a node in the listbox immediately updates the inspector's `(x, y)` and `Type` to that node's values, with no mode switch required.
- [ ] **AC2:** Inspector shows two `Entry` widgets for x and y (centimetres, one decimal). The model is the source of truth; widgets re-populate after every model mutation.
- [ ] **AC3:** Typing a valid in-range float in the x or y entry and pressing `<Return>` (or tabbing out) dispatches a `MoveNode` command, marks the model dirty, and refreshes the canvas + inspector.
- [ ] **AC4:** Typing an out-of-range or unparseable value in x/y restores the prior value and shows a status-bar message (e.g. `x must be in [0, 200]`). No `MoveNode` is dispatched.
- [ ] **AC5:** Dragging a node in MOVE mode updates the x/y Entry widgets live during the drag and after release (already-existing refresh path).
- [ ] **AC6:** The "Add node" flow (left-click in ADD_NODE mode) opens a single dialog with a Name `Entry` and a Type `Combobox` (default `line`, values `line`/`blind`). OK dispatches `AddNode`; Cancel does nothing.
- [ ] **AC7:** The Nodes labelframe contains a "Max edge length (cm)" Spinbox (default 60, range 10–300, step 5) and an "Auto-create edges" button.
- [ ] **AC8:** Clicking Auto-create with ≥ 3 non-collinear nodes computes proposed edges (Delaunay → length cap → obstacle filter → type rule → skip existing pairs) and renders them as dashed cyan lines on the canvas. Two new buttons appear next to the auto-create button: **Accept** and **Reject**. Status bar reports `Proposed N edges (line=X, blind=Y, skipped=Z).`.
- [ ] **AC9:** Clicking Accept dispatches a single `AddEdgesBatch` command, clears the preview, marks the model dirty, refreshes the canvas. One Ctrl-Z reverts all the added edges in one step.
- [ ] **AC10:** Clicking Reject clears the preview state without mutating the model. The Accept/Reject buttons disappear (or revert to disabled).
- [ ] **AC11:** Clicking Auto-create with < 3 nodes (or all collinear, or all edges filtered out) shows a status-bar message and does not enter preview mode.
- [ ] **AC12:** `AddEdgesBatch` and the `auto_edges` helper module are covered by unit tests in `tools/path_planner/tests/`. The existing 154-test suite still passes.

## Data Model

No `field_data.json` schema changes. New in-memory types:

```python
# tools/path_planner/editor/auto_edges.py

EdgeType = Literal["line", "blind"]
ProposedEdge = tuple[str, str, EdgeType]  # (node_a, node_b, kind)

def propose_edges(
    model: FieldModel,
    max_edge_length_cm: float,
) -> list[ProposedEdge]:
    """Delaunay -> length cap -> obstacle filter -> type rule -> skip existing.
    Returns deduplicated undirected proposals, sorted by (a, b) for determinism."""

def _classify_edge_type(node_types: dict[str, str], a: str, b: str) -> EdgeType:
    """line + line -> 'line'; any 'blind' -> 'blind'."""
```

```python
# tools/path_planner/editor/commands.py (new dataclass appended)

@dataclass
class AddEdgesBatch:
    """Append multiple edges atomically. One undo entry covers the whole batch."""
    edges: list[tuple[str, str, str]]   # (a, b, "line"|"blind")
    _applied_indices: list[tuple[str, int]] | None = field(default=None, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        # Validates each edge (duplicate detection across both lists), then appends
        # all of them. Captures (list_name, index) for revert. Raises ValueError on
        # the first conflict; the model is untouched (all-or-nothing).
        ...

    def revert(self, model: FieldModel) -> None:
        # Pops each captured (list_name, index) in reverse order.
        ...
```

```python
# Preview state on EditorApp
self._proposed_edges: Optional[list[tuple[str, str, str]]] = None
```

```python
# Canvas render API additions
def render(
    self,
    model: FieldModel,
    *,
    highlighted_nodes: Optional[list[str]] = None,
    proposed_edges: Optional[list[tuple[str, str, str]]] = None,  # new
) -> None: ...
```

```python
# tools/path_planner/editor/dialogs.py (new)

class AddNodeDialog(tk.Toplevel):
    """Modal dialog: Name entry + Type combobox + OK/Cancel.

    Usage:
        dlg = AddNodeDialog(parent, initial_type="line")
        dlg.wait_window()
        if dlg.result is not None:
            name, node_type = dlg.result
    """
    result: tuple[str, str] | None  # None on Cancel
```

## Error Handling

- **Listbox-select with stale model:** `_refresh_view` is idempotent; calling it on a model where the selected node has been deleted (e.g. in a race that shouldn't actually happen given Tk's single-threaded event loop) clears the inspector via `_clear_inspector`. No exception.
- **x/y entry parse failure:** caught in the panel handler; the Entry is reset to the model's current value and the status bar shows `x must be a number` (or `y must be a number`).
- **x/y out of range:** Entry reset, status bar shows `x must be in [0, width_cm]` (or analogous for y). No model mutation.
- **Add-node dialog Cancel:** no command dispatched, no status change. The canvas's click that opened the dialog is consumed without further effect.
- **Add-node duplicate name:** existing `AddNode` already raises `ValueError`; `_dispatch` catches it and surfaces the message in the status bar. The dialog is already closed at that point — the user has to click again. Acceptable for this PR; a "name already taken" inline error in the dialog itself is a follow-up.
- **AddEdgesBatch partial conflict:** all-or-nothing. If any edge in the batch is rejected by the underlying validator (e.g. a stale duplicate slipped through), the entire batch is aborted, `_dispatch` shows the offending edge in the status bar, the preview overlay stays on so the user can see what tried to commit.
- **Auto-create with degenerate inputs:** `propose_edges` returns `[]`; the app shows `No edges to propose (need ≥ 3 non-collinear nodes / all candidates filtered).` in the status bar. No preview overlay, no Accept/Reject buttons.
- **Shapely raises on collinear / repeated points:** caught in `propose_edges`; treated the same as degenerate input.

## Testing Strategy

**Levels:** Unit only (existing pattern — GUI surfaces are smoke-tested manually).

| ID  | Test Case                                                                                | Type | Expected Behavior                                                                                          |
|-----|------------------------------------------------------------------------------------------|------|------------------------------------------------------------------------------------------------------------|
| TC1 | `AddEdgesBatch.apply` on an empty batch                                                  | Unit | No mutation; revert is a no-op.                                                                            |
| TC2 | `AddEdgesBatch.apply` appends 3 mixed line/blind edges                                   | Unit | All 3 appear in the right lists; revert pops them all; model byte-identical to pre-apply.                  |
| TC3 | `AddEdgesBatch.apply` with one duplicate (already in `line_segments`)                    | Unit | Raises `ValueError`; model untouched (no partial mutation).                                                |
| TC4 | `AddEdgesBatch.apply` with two entries referencing the same undirected pair              | Unit | Raises `ValueError`; model untouched.                                                                      |
| TC5 | `propose_edges` on a 4-node square with no obstacles, length cap = 100                   | Unit | Returns the perimeter edges + one diagonal (Delaunay of a square = 2 triangles → 5 edges). Verify exact list + sortedness. |
| TC6 | `propose_edges` with a length cap that excludes the diagonal                             | Unit | Diagonal dropped; only the 4 perimeter edges remain.                                                       |
| TC7 | `propose_edges` with an obstacle clipping one edge                                       | Unit | That edge is filtered out; other edges unaffected.                                                         |
| TC8 | `propose_edges` with one blind node + three line nodes                                   | Unit | Edges touching the blind node have `kind="blind"`; the all-line triangle edge has `kind="line"`.           |
| TC9 | `propose_edges` skips pairs that already have a line edge                                | Unit | Pre-existing edge is not re-proposed regardless of ordering ((a,b) or (b,a)).                              |
| TC10| `propose_edges` skips pairs that already have a blind edge                               | Unit | Same as TC9 for the blind list.                                                                            |
| TC11| `propose_edges` on 2 nodes                                                               | Unit | Returns `[]` (Delaunay needs ≥ 3 points).                                                                  |
| TC12| `propose_edges` on 3 collinear nodes                                                     | Unit | Returns `[]` and does not raise; shapely's degenerate-triangle handling is swallowed.                      |
| TC13| `_classify_edge_type` on (line, line) / (line, blind) / (blind, line) / (blind, blind)   | Unit | Returns `"line"` / `"blind"` / `"blind"` / `"blind"` respectively.                                         |
| TC14| `AddEdgesBatch` round-trip apply → revert → apply (re-do)                                | Unit | Final model state matches the post-first-apply state. Mirrors existing TC4/TC13/TC14 invariants.           |

**Test data:** Build small `FieldModel` instances in-line per test (~5-line literals) — mirrors how the existing `test_editor_commands.py` does it. The auto_edges tests use a couple of fixtures: a 4-node square, a 4-node square with a centred obstacle, and a 3-collinear-node degenerate case.

**Run command:** `uv run pytest tools/path_planner/tests/`

**Manual smoke test plan** (verifying AC1, AC5, AC6, AC8–AC11 — GUI-only paths not covered by unit tests):
1. Launch `python tools/path_planner/edit_field.py`. Click around the listbox; inspector x/y updates immediately.
2. Type a new x value in the inspector, press Enter; the canvas re-renders the node at the new position.
3. Click Add node mode → click on the canvas → fill in the combined dialog → confirm node appears.
4. Click Auto-create edges → preview overlay appears → Accept → edges committed.
5. Ctrl-Z → all auto-created edges disappear in one step.
6. Click Auto-create edges again → Reject → preview clears, no mutation.

## Tasks

| ID  | Task                                                                                  | Blocked By              | Risk | Files                                                                                                  | Description                                                                                                                                                                                                                                                                                                                                              |
|-----|---------------------------------------------------------------------------------------|-------------------------|------|--------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| T1  | Bug #1: refresh inspector on listbox select                                           | —                       | low  | `tools/path_planner/editor/app.py`                                                                     | In `EditorApp._on_panel_node_select`, after the status-bar update, call `self._refresh_view()`. Satisfies AC1. Verify by running the editor and confirming the (x, y) label updates immediately on listbox click.                                                                                                                                       |
| T4  | New `AddEdgesBatch` command + unit tests                                              | —                       | low  | `tools/path_planner/editor/commands.py`, `tools/path_planner/tests/test_editor_commands.py`            | Append the dataclass per Data Model. Implement `apply` as a two-pass validate-then-append; capture `_applied_indices` as `[(list_name, index), …]` so `revert` can pop in reverse order. Add tests TC1–TC4 and TC14. Satisfies AC9 + AC12 (partial).                                                                                                     |
| T5  | New `auto_edges.py` helper + unit tests                                               | —                       | med  | `tools/path_planner/editor/auto_edges.py` (new), `tools/path_planner/tests/test_editor_auto_edges.py` (new) | Implement `propose_edges(model, max_edge_length_cm)`: build `MultiPoint` of node positions, call `shapely.ops.triangulate`, extract undirected pair set from each triangle's exterior, apply length cap, call `graph_builder.edge_clips_obstacle` against pre-inflated polys, apply type rule, filter out pre-existing pairs. Return a deterministic sorted list. Wrap shapely calls in a try/except for the degenerate-input case. Add tests TC5–TC13. Satisfies AC11 + AC12 (partial). |
| T6  | Canvas: render proposed-edges overlay                                                 | —                       | low  | `tools/path_planner/editor/canvas.py`                                                                  | Add `proposed_edges: Optional[list[tuple[str, str, str]]] = None` kwarg to `FieldCanvas.render`. Add a `_draw_proposed_edges` helper drawn with `color="cyan"`, `linestyle=(0, (3, 3))`, `linewidth=1.8`, `zorder` above edges + below highlight ring. New constants in the canvas constants block. Satisfies AC8 (visual part).                          |
| T2  | Inspector: typeable x/y Entry widgets                                                 | T1                      | med  | `tools/path_planner/editor/panels.py`, `tools/path_planner/editor/app.py`                              | Replace the read-only `(x, y) cm` label in `SidePanels._build_node_list_and_inspector` with two `tk.Entry` widgets bound to new `_selected_x_var` / `_selected_y_var` Tk vars. Add `on_node_coords_change: Callable[[str, float, float], None]` callback. Update `_apply_inspector` and `_clear_inspector` to populate / blank the entries. Hook `<Return>` and `<FocusOut>` to `_handle_coords_change`, which parses, validates against `model.field_mm` (via a new `field_mm` cached on the panel from `refresh()`), and either fires the callback or resets the entry + writes a status message. In `EditorApp`, wire the callback to `_on_panel_node_coords_change`, which dispatches `MoveNode`. Satisfies AC2–AC5. |
| T3  | Combined Add-node dialog                                                              | T1                      | low  | `tools/path_planner/editor/dialogs.py` (new), `tools/path_planner/editor/app.py`                       | New `AddNodeDialog(tk.Toplevel)` class: Name `Entry` + Type `Combobox(values=["line","blind"], state="readonly")` + OK/Cancel buttons. Exposes `result: tuple[str, str] | None`. Modal via `transient(parent) + grab_set() + wait_window()`. In `_handle_add_node_click`, replace the two `simpledialog.askstring` calls with `dlg = AddNodeDialog(self._root); dlg.wait_window()`. Satisfies AC6.                                                                              |
| T7  | Auto-create UI: Spinbox + button in Nodes labelframe                                  | T2                      | low  | `tools/path_planner/editor/panels.py`                                                                  | In `SidePanels._build_node_list_and_inspector`, append a Frame holding a "Max edge length (cm)" `ttk.Spinbox` (`from_=10, to=300, increment=5`, bound to new `_max_edge_length_var: StringVar`, default `"60"`) and an "Auto-create edges" `tk.Button`. Expose `on_auto_create_edges: Callable[[], None]` callback. Add `set_auto_create_state(mode)` method that toggles button visibility (`"idle"` shows only Auto-create, `"preview"` shows Accept + Reject). Expose `on_accept_proposed`, `on_reject_proposed` callbacks. Add public `max_edge_length_cm: float` property reading the Spinbox var. Satisfies AC7 (UI surface only). |
| T8  | Wire auto-create flow in EditorApp                                                    | T3, T4, T5, T6, T7      | high | `tools/path_planner/editor/app.py`                                                                     | Wire panel callbacks: `_on_panel_auto_create_edges` calls `auto_edges.propose_edges(self._model, panels.max_edge_length_cm)`, stores result on `self._proposed_edges`, calls `panels.set_auto_create_state("preview")`, refreshes the canvas (passing `proposed_edges=self._proposed_edges`). `_on_panel_accept_proposed` dispatches `AddEdgesBatch([(a,b,kind) for a,b,kind in self._proposed_edges])`, clears `_proposed_edges`, switches panel state back to `"idle"`, refreshes. `_on_panel_reject_proposed` clears `_proposed_edges`, switches state to `"idle"`, refreshes. Update `_refresh_view` to pass `self._proposed_edges` to `canvas.render`. Update `_clear_transient_state` to also clear `_proposed_edges` (mode change kills the preview). Satisfies AC8 (logic), AC9, AC10, AC11. |
| T9  | Verification: full pytest + manual smoke per the Testing Strategy                     | T8                      | low  | —                                                                                                      | Run `uv run pytest tools/path_planner/tests/` and confirm all 154 (now ~164) tests pass. Step through the 6 manual smoke steps. Report any AC that isn't met. Do not edit code in this task — only verify. If anything fails, file the gap and let the user decide whether to land or iterate.                                                            |

### Parallelism

- Round 1 (after T1 lands): T4, T5, T6 in parallel.
- Round 2: T2 unblocks T7 and T3.
- Round 3: T8.
- Round 4: T9.

## Notes for Implementer

- **Imports:** `propose_edges` in `auto_edges.py` should import `inflate_obstacles` and `edge_clips_obstacle` directly from `tools.path_planner.graph_builder`. Don't import shapely classes through graph_builder — import them directly (`from shapely.geometry import MultiPoint, Polygon` and `from shapely.ops import triangulate`).
- **Determinism in `propose_edges`:** after extracting edges from triangles, sort the result by `(min(a,b), max(a,b))` so the output is stable across Python dict orderings and shapely versions. Tests will assert exact lists.
- **Shapely degenerate triangulation:** `triangulate` on collinear points may return an empty list; on fewer than 3 points it may raise. Wrap the whole `triangulate` call in `try/except (ValueError, AttributeError)` and return `[]` on failure. Add a TC12 assertion for this path.
- **Length cap reads from the panel at click time, not at refresh time.** `EditorApp` reads `self._panels.max_edge_length_cm` inside `_on_panel_auto_create_edges`. Don't cache it.
- **Mode change must clear the preview.** Add `self._proposed_edges = None; self._panels.set_auto_create_state("idle")` to `_clear_transient_state` — otherwise switching from MOVE to OBSTACLE mid-preview leaves dangling Accept/Reject buttons.
- **x/y Entry re-entrancy:** the panel's `_suppress_callbacks` flag already exists; reuse it. `_apply_inspector` sets the entry vars and must be wrapped in the suppression block (which `refresh()` already does).
- **Field bounds for x/y validation:** read from `model.field_mm`. Cache `self._field_mm` on `SidePanels` from `refresh()` so `_handle_coords_change` doesn't need a model reference. Field is centimetres so the bound is `field_mm[0] / 10` for x, `field_mm[1] / 10` for y.
- **`AddNodeDialog` keyboard hygiene:** bind `<Return>` on the Name entry to "advance focus to combobox", and `<Return>` on the OK button to commit. Bind `<Escape>` to Cancel. Set initial focus to the Name entry.
- **No new ADR.** These features are local to the editor; they don't establish a project-wide rule.
- **Rollback:** revert is per-task — each task is a clean commit. If T8 is buggy, drop the T8 commit and ship T1–T7 (the bug fix, x/y entries, dialog, batch command, Delaunay helper, and overlay rendering are all useful on their own; the only thing the user loses is the button wiring).
