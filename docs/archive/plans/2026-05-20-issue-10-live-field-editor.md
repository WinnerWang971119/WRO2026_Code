# Issue #10 Phase 2 — Live Field Editor Plan

**Goal:** Build a Tk-based GUI (`tools/path_planner/edit_field.py`) that lets you author the WRO 2026 field graph visually — click to add nodes, drag to move, hotkeys to connect edges, draw obstacles, set start pose — then save back through a JSON sidecar and one-click regenerate `routes.h`. Migrates `field_data.py` from a Python-literal source to a JSON-loader so the GUI is the canonical authoring surface.

**Approach:** Single Tk window: matplotlib map canvas on the left (embedded via `FigureCanvasTkAgg`), side panel on the right (mode toggle, node/edge inspector, start-pose form, Build button). In-memory `FieldModel` mirrors the JSON schema; every mutation goes through a Command on an undo/redo stack. Persistence = `field_data.json` (geometry + start pose) plus an AST-based rewrite of `field_data.py`'s rich `EDGES` table for node-rename cascades. Build button calls `build_routes._run()` in-process and surfaces the result. Validation runs in-memory before any disk write; failures abort the write and dialog the offending entry.

## Scope

- **In scope:**
  - GUI to create / move / rename / delete nodes (line + blind types).
  - GUI to create / delete simple edges (`LINE_SEGMENTS`, `BLIND_EDGES`).
  - GUI to draw / delete axis-aligned obstacle rectangles with a `kind` label.
  - GUI to set `START_NODE` (combobox) and `START_HEADING_DEG` (spinbox 0–359.9°).
  - Undo / redo stack (Ctrl+Z / Ctrl+Y) covering every model mutation.
  - Node rename cascades into `LINE_SEGMENTS`, `BLIND_EDGES`, `NODE_TYPES`, `START_NODE` in JSON, AND into the `u` / `v` keys of `field_data.py`'s `EDGES` table via AST rewrite (LibCST).
  - `field_data.json` schema + loader making `field_data.py` a thin module that re-exposes JSON-loaded values as module attributes.
  - One-shot migration command (`uv run python tools/path_planner/edit_field.py --migrate`) that ports current Python literals into JSON, then strips them from `field_data.py`.
  - Manual **Build** button that validates the model, writes JSON, applies the rich-EDGES AST rewrite (only when renames happened), then calls `build_routes._run()` in-process and shows result.
  - `pytest` coverage: model unit tests, JSON round-trip, AST-rewrite of EDGES, validator integration.

- **Out of scope:**
  - Rich `EDGES` form-based editing (per-edge `primitive_mode` / `pre_turn_mode` / `pivot` overrides — stays hand-edited in `field_data.py`).
  - `PIPELINE_CONFIG` defaults editor (still hand-edited in `pipeline_config.py`).
  - Real WRO 2026 field measurement — that work is Phase 3 (P3.4–P3.6 in `Roadmap.md`) using the new editor.
  - Web / PyQt port. Tk + matplotlib only.
  - Replacing or extending `pick_xy.py` — it stays as the lightweight read-only coordinate reader.
  - Updating CI to install a GUI display server (tests run model + persistence layers; canvas/widgets are not tested in CI).
  - Editing `field_data.py`'s `EDGES` contents (only the `u`/`v` string keys are rewritten on rename — `distance_cm`, `heading_deg`, mode overrides etc. are untouched).

## Decisions

- **Persistence**: JSON sidecar + Python loader — confirmed Batch 2. Geometry + start pose lives in `field_data.json`; rich `EDGES` stays in Python because it carries enum imports (`LineMode`, `Pivot`, etc.).
- **Toolkit**: Tk + embedded matplotlib (`FigureCanvasTkAgg`) — no extra dep, native widgets, matches existing `pick_xy.py` / `visualizer.py` stack.
- **Build trigger**: Manual button only — every save can be deliberate and bundled with the golden-fixture commit dance (matches the existing `field_data.py → routes.h → goldens` workflow already mandated in the user's mem0).
- **Edit history**: Full undo/redo stack via the Command pattern, unlimited depth (in-memory only — discarded on app close).
- **Rename cascade**: GUI owns end-to-end rename. JSON keys are rewritten directly; rich `EDGES` `u`/`v` keys are rewritten via LibCST so comments and formatting in `field_data.py` survive.
- **Validation**: Block on failure. In-memory model is validated (using the existing `validator.validate` adapted to read from the model) before any disk write. JSON and AST rewrite are written together or not at all.
- **Tests**: Existing toy goldens stay on the synthetic `_build_toy_field_data()` helper. Add a new test covering JSON load → validator → graph_builder → planner round-trip with a fresh fixture JSON.
- **Migration**: One-shot subcommand. After it runs, `field_data.py` has no `NODES = {...}` etc. literals — only the loader and `EDGES`. Single source of truth from then on.
- **Entry point**: New file `tools/path_planner/edit_field.py`. `pick_xy.py` stays.

## Acceptance Criteria

- [ ] `uv run python tools/path_planner/edit_field.py --migrate` reads the current `field_data.py` once and writes `tools/path_planner/field_data.json`. Running it twice is a no-op (JSON is byte-identical the second time).
- [ ] After migration, `field_data.py` exposes the same module attributes (`NODES`, `NODE_TYPES`, `LINE_SEGMENTS`, `BLIND_EDGES`, `OBSTACLES`, `FIELD_MM`, `MAP_PNG`, `ROBOT_RADIUS_CM`, `COST_BLIND_MULTIPLIER`, `START_NODE`, `START_HEADING_DEG`, `EDGES`) with values byte-identical to pre-migration via `repr()` comparison.
- [ ] `uv run python tools/path_planner/build_routes.py` produces a byte-identical `routes.h` / `routes.cpp` before vs after the migration (the toy graph itself is unchanged).
- [ ] `uv run python tools/path_planner/edit_field.py` opens a Tk window showing `maps/map.png` as background, existing nodes as labeled dots, edges as lines (LINE = black solid, BLIND = red dashed), obstacles as inflated polygons.
- [ ] Left-click in **Add Node** mode creates a node at the click point; a dialog prompts for name + type. Node appears immediately on canvas and in the right-hand list.
- [ ] Dragging a node moves it; the surrounding edges redraw live. Releasing commits one undo entry.
- [ ] Selecting two nodes and pressing `L` adds a `LINE_SEGMENTS` entry; `B` adds a `BLIND_EDGES` entry. Adding a duplicate is a no-op + status-bar message.
- [ ] Right-click on a node → context menu: **Rename** (text dialog), **Set type** (line/blind), **Delete**. Delete refuses if any `EDGES` (rich Python table) entry references the node, with a dialog showing the offending entry index.
- [ ] Rename a node that is referenced by a rich `EDGES` entry → on Save, `field_data.py` is LibCST-rewritten so the corresponding string literal changes. All comments and surrounding formatting in the file survive.
- [ ] **Obstacle** mode: drag a rectangle, then a dialog prompts for `kind`. Drawn rectangle + inflated polygon redraw immediately.
- [ ] **Start pose** panel: combobox lists every node, spinbox sets heading 0–359.9°. Changes are model mutations (undoable).
- [ ] **Undo (Ctrl+Z) / Redo (Ctrl+Y)** correctly reverse / replay every operation above, including rename cascade.
- [ ] **Save** button writes `field_data.json` (pretty-printed, 2-space indent, deterministic key order) AND applies any pending rich-EDGES AST rewrites. Validation failure aborts both writes with an error dialog naming the offending key (e.g. "NODE_TYPES['G'] = 'blob' — expected 'line' or 'blind'").
- [ ] **Save & Build** button does Save, then calls `build_routes._run()` in-process. Result dialog: "Wrote routes.h, routes.cpp" on success; on failure, prints `build_routes` stderr to the dialog. Exit code mapped to dialog severity.
- [ ] `uv run pytest tools/path_planner/tests/` passes, including new tests covering JSON load, JSON round-trip equality, AST rewrite preserving comments, and rename cascade.
- [ ] CI Arduino compile-check still passes (no firmware change).

## Data Model

### `field_data.json` schema

```json
{
  "FIELD_MM": [3000, 2000],
  "MAP_PNG": "maps/map.png",
  "ROBOT_RADIUS_CM": 8.0,
  "COST_BLIND_MULTIPLIER": 5.0,
  "START_NODE": "A",
  "START_HEADING_DEG": 0.0,
  "NODES": {
    "A": [10.0, 10.0],
    "B": [200.0, 10.0]
  },
  "NODE_TYPES": {
    "A": "line",
    "B": "line"
  },
  "LINE_SEGMENTS": [
    ["A", "B"]
  ],
  "BLIND_EDGES": [
    ["B", "G"]
  ],
  "OBSTACLES": [
    [50.0, 50.0, 80.0, 80.0, "pillar"]
  ]
}
```

Key conventions:
- Tuples → JSON arrays. `NODES['A'] = [10.0, 10.0]` instead of `(10.0, 10.0)`. The loader converts arrays back to tuples on import to keep the existing validator happy.
- Floats always written as `float` literals (never bare ints) so round-trip equality holds.
- Top-level keys ordered: `FIELD_MM`, `MAP_PNG`, `ROBOT_RADIUS_CM`, `COST_BLIND_MULTIPLIER`, `START_NODE`, `START_HEADING_DEG`, `NODES`, `NODE_TYPES`, `LINE_SEGMENTS`, `BLIND_EDGES`, `OBSTACLES`. Within dicts, keys sorted alphabetically.

### In-memory model (Python)

```python
# tools/path_planner/editor/model.py

@dataclass
class FieldModel:
    field_mm: tuple[float, float]
    map_png: str
    robot_radius_cm: float
    cost_blind_multiplier: float
    start_node: str
    start_heading_deg: float
    nodes: dict[str, tuple[float, float]]              # name -> (x_cm, y_cm)
    node_types: dict[str, str]                          # name -> "line" | "blind"
    line_segments: list[tuple[str, str]]                # undirected
    blind_edges: list[tuple[str, str]]                  # undirected
    obstacles: list[tuple[float, float, float, float, str]]

    # Snapshot for undo/redo (deepcopy of all above)
    def snapshot(self) -> "FieldModel": ...
    def restore(self, snap: "FieldModel") -> None: ...
```

### Command pattern for undo/redo

```python
class Command(Protocol):
    def apply(self, model: FieldModel) -> None: ...
    def revert(self, model: FieldModel) -> None: ...

# Concrete commands:
#   AddNode, MoveNode, RenameNode, DeleteNode, SetNodeType,
#   AddLineEdge, AddBlindEdge, DeleteEdge,
#   AddObstacle, DeleteObstacle,
#   SetStart, SetStartHeading
```

`RenameNode.apply()` captures the original name plus a list of `field_data.py` byte ranges (from a prior LibCST scan) that need rewriting on save. Pending AST rewrites are flushed on Save / Save&Build.

### `field_data.py` after migration

```python
"""WRO 2026 field encoding (PR3, JSON-loaded)."""
from tools.path_planner.editor.loader import load_field_json
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION, Pivot
from tools.path_planner.pipeline_config import LineMode, BlindMode, ProjectionMode, TurnMode

_data = load_field_json()
FIELD_MM              = _data["FIELD_MM"]
MAP_PNG               = _data["MAP_PNG"]
ROBOT_RADIUS_CM       = _data["ROBOT_RADIUS_CM"]
COST_BLIND_MULTIPLIER = _data["COST_BLIND_MULTIPLIER"]
START_NODE            = _data["START_NODE"]
START_HEADING_DEG     = _data["START_HEADING_DEG"]
NODES                 = _data["NODES"]
NODE_TYPES            = _data["NODE_TYPES"]
LINE_SEGMENTS         = _data["LINE_SEGMENTS"]
BLIND_EDGES           = _data["BLIND_EDGES"]
OBSTACLES             = _data["OBSTACLES"]

# Rich edge table — hand-edited. GUI rewrites only u/v keys on node rename.
EDGES = [
    # ... existing entries verbatim ...
]
```

## File Layout

```
tools/path_planner/
├── edit_field.py              # NEW — CLI entry: opens GUI (default) or --migrate
├── field_data.py              # MODIFIED — becomes loader; EDGES preserved verbatim
├── field_data.json            # NEW — generated by --migrate; canonical geometry
└── editor/                    # NEW package
    ├── __init__.py
    ├── loader.py              # JSON ↔ Python tuple round-trip; validator hook
    ├── model.py               # FieldModel dataclass + snapshot/restore
    ├── commands.py            # Command pattern + concrete commands
    ├── history.py             # Undo/redo stack
    ├── ast_rewrite.py         # LibCST-based EDGES u/v rewrite
    ├── migrate.py             # One-shot field_data.py → field_data.json
    ├── app.py                 # Tk root window + main loop wiring
    ├── canvas.py              # FigureCanvasTkAgg + render + mouse handlers
    └── panels.py              # Right-side Tk panels (mode, inspector, start, build)

tools/path_planner/tests/
├── test_editor_loader.py      # NEW — JSON round-trip + validator-after-load
├── test_editor_model.py       # NEW — FieldModel snapshot/restore
├── test_editor_commands.py    # NEW — each Command apply/revert round-trip
├── test_editor_ast_rewrite.py # NEW — LibCST EDGES rewrite preserves comments
├── test_editor_migrate.py     # NEW — migrate idempotency + repr equality
└── (existing tests unchanged)

tools/path_planner/requirements.txt
└── ADDED: libcst >= 1.1
```

`libcst` is the only new dep. Already widely available, pure-Python, used by Meta/Instagram.

## Error Handling

- **JSON missing**: loader raises `FieldDataError("field_data.json not found — run `edit_field.py --migrate`")`. `build_routes.py` propagates with exit code 2.
- **JSON malformed**: loader raises `FieldDataError` with `json.JSONDecodeError` message + line/col. No fallback.
- **Validation fails on Save**: dialog with the precise error (re-uses existing `FieldDataError` messages). No file is written; in-memory model is unchanged so the user can keep editing.
- **AST rewrite fails (e.g. EDGES entry references a deleted node)**: dialog naming the offending entry index. Save aborts (JSON not written either). Undo button re-available.
- **`build_routes._run` non-zero exit**: full stderr captured and shown in a scrollable dialog. JSON is already saved at this point, so the user can re-Build after fixing.
- **Migration on already-migrated file**: detected by absence of literal `NODES = {` in `field_data.py`. Migration becomes a no-op and prints "already migrated."
- **Concurrent edits to `field_data.json`**: not handled (single-user tool). The Save dialog warns "this will overwrite `field_data.json`" when the file's mtime moved since open.

## Testing Strategy

**Levels:** Unit (model, commands, AST rewrite, loader), Integration (migrate + load + build pipeline). No GUI / canvas tests — headless CI can't drive Tk reliably.

| ID  | Test Case                                                                                  | Type        | Expected Behavior                                                                                  |
|-----|--------------------------------------------------------------------------------------------|-------------|----------------------------------------------------------------------------------------------------|
| TC1 | `load_field_json()` on golden JSON returns dict whose tuple-types match validator input    | Unit        | Tuples not lists for `NODES` values, `FIELD_MM`, obstacle bounds, edges                            |
| TC2 | Round-trip: `model → dump_json → parse_json → model` equals original                       | Unit        | `repr` equality                                                                                    |
| TC3 | JSON round-trip is byte-identical (idempotency)                                            | Unit        | `dump(load(dump(model)))` byte-equal to `dump(model)`                                              |
| TC4 | `AddNode → revert` restores model exactly                                                  | Unit        | Pre- and post-revert snapshots `repr` equal                                                        |
| TC5 | `RenameNode` cascades into `LINE_SEGMENTS`, `BLIND_EDGES`, `NODE_TYPES`, `START_NODE`      | Unit        | All references switched; counts unchanged                                                          |
| TC6 | `DeleteNode` refuses when a rich-EDGES Python entry references it                          | Unit        | Raises with entry index; model unchanged                                                           |
| TC7 | LibCST `rewrite_edges_uv` renames one node across multiple EDGES entries                   | Unit        | Output file diff is only the `"u"` / `"v"` string literals; all comments + formatting preserved    |
| TC8 | LibCST rewrite on a file with no matching references is a no-op                            | Unit        | Output byte-identical to input                                                                     |
| TC9 | `--migrate` on the shipped repo state produces a JSON that loads back to the same values   | Integration | After migration, `from field_data import *` gives the same `repr` for every public attr           |
| TC10| `--migrate` is idempotent                                                                  | Integration | Running twice — second run is a no-op, JSON byte-identical                                         |
| TC11| `build_routes._run()` after migration produces byte-identical `routes.h` / `routes.cpp`    | Integration | Compared against pre-migration baseline                                                            |
| TC12| Validator rejects invalid in-memory model (e.g. NODE_TYPES coverage gap)                   | Unit        | `FieldDataError` raised with offending key path                                                    |
| TC13| Undo stack: 10 random mutations + 10 undos returns to original                             | Unit        | Final `repr` equals start                                                                          |
| TC14| Redo stack: undo 5, redo 5, model equals state after first 10 mutations                    | Unit        | `repr` equality                                                                                    |
| TC15| Renaming a node also queues a deferred AST rewrite; on Save the rewrite hits disk          | Integration | After Save, `field_data.py` on disk has the new name; `git diff` shows only string literal changes |

**Test data:** A small synthetic JSON fixture (3 nodes, 2 line edges, 1 obstacle) lives in `tools/path_planner/tests/fixtures/field_data_small.json`. Existing toy fixtures in `_build_toy_field_data()` stay untouched (they exercise the validator → graph → planner path on Python objects directly; the JSON loader is exercised by TC1–TC15).

**Run command:** `uv run pytest tools/path_planner/tests/`

## Tasks

| ID  | Task | Blocked By | Risk | Files | Description |
|-----|------|------------|------|-------|-------------|
| T1  | JSON schema + loader | — | med | `tools/path_planner/editor/loader.py`, `tools/path_planner/editor/__init__.py` | Define the JSON schema in a docstring. Implement `load_field_json(path=None) -> dict` that reads `field_data.json` (default location: alongside `field_data.py`), converts JSON arrays to tuples for the fields the validator expects (NODES values, FIELD_MM, LINE_SEGMENTS entries, BLIND_EDGES entries, OBSTACLES entries), and returns a dict keyed by the existing module-attribute names. Raise `FieldDataError` with a friendly hint if the file is missing or malformed. Pure function, no global state. Satisfies AC for migration + load path. |
| T2  | `FieldModel` dataclass + snapshot/restore | — | low | `tools/path_planner/editor/model.py` | Plain dataclass mirroring the JSON schema fields. `snapshot()` deepcopies into a new `FieldModel`; `restore(snap)` overwrites fields in place. Used by the undo stack. No GUI, no I/O. |
| T3  | Migration script `--migrate` | T1, T2 | med | `tools/path_planner/editor/migrate.py`, `tools/path_planner/field_data.py` | One-shot CLI: imports the current `field_data.py`, builds the JSON dict, writes `field_data.json` with deterministic ordering + 2-space indent, then rewrites `field_data.py` so the geometry literals are removed and replaced by a `_data = load_field_json()` block (rich `EDGES` table preserved verbatim, including all its comments). Idempotent: detect via absence of `NODES = {` to skip rewrite. Satisfies AC1, AC2, AC3. |
| T4  | Validator integration with model | T2 | low | `tools/path_planner/editor/model.py`, `tools/path_planner/validator.py` (no change) | `FieldModel.to_module_proxy()` returns an object whose `getattr` lookups match what `validator.validate` reads. Add a `model.validate() -> None` method that wraps it and raises `FieldDataError` on first issue. No changes to `validator.py` itself. |
| T5  | Command pattern + undo/redo stack | T2 | med | `tools/path_planner/editor/commands.py`, `tools/path_planner/editor/history.py` | Implement `Command` protocol + concrete commands listed in the data model. Each Command stores its inverse data on `apply`. `History` exposes `do(cmd)`, `undo()`, `redo()`, `clear()`. Undo stack unbounded in memory. Pure logic, GUI-free. Satisfies AC for TC4, TC13, TC14. |
| T6  | LibCST AST rewrite for rich EDGES u/v keys | — | high | `tools/path_planner/editor/ast_rewrite.py`, `tools/path_planner/requirements.txt` | Add `libcst >= 1.1` to requirements. Implement `rewrite_edges_uv(path: Path, renames: dict[str, str]) -> bool` that opens `field_data.py`, finds the `EDGES = [...]` assignment, walks each dict entry, and rewrites `"u"` / `"v"` string-literal values per the renames map. Comments + formatting preserved. Returns True if any change written. Satisfies TC7, TC8. **HIGH risk** — touches source code with comments. |
| T7  | Loader test fixture + TC1–TC3, TC9–TC11 | T1, T3 | low | `tools/path_planner/tests/test_editor_loader.py`, `tools/path_planner/tests/test_editor_migrate.py`, `tools/path_planner/tests/fixtures/field_data_small.json` | Write tests TC1, TC2, TC3, TC9, TC10, TC11. Use `monkeypatch` to point `load_field_json` at the small fixture for unit tests. The integration tests (TC9–TC11) shell out to `edit_field.py --migrate` in a tmp_path copy of the repo's `field_data.py` + run `build_routes._run`. |
| T8  | Model / Command tests (TC4–TC6, TC12–TC14) | T4, T5 | low | `tools/path_planner/tests/test_editor_model.py`, `tools/path_planner/tests/test_editor_commands.py` | Pure-Python tests for the model + command + undo/redo behavior. |
| T9  | AST rewrite tests TC7, TC8, TC15 | T6 | low | `tools/path_planner/tests/test_editor_ast_rewrite.py` | Build a small synthetic `field_data.py` in `tmp_path` with a stub EDGES table and comments; rewrite via `ast_rewrite.rewrite_edges_uv`; assert diff is exactly the renamed string literals. |
| T10 | Canvas widget: map + nodes + edges + obstacles | T1, T2 | med | `tools/path_planner/editor/canvas.py` | `class FieldCanvas`: holds a `FigureCanvasTkAgg`, exposes `render(model)` that redraws axes, map background, edges, nodes, obstacles, inflated polygons, and labels. Reuse existing `visualizer._draw_*` helpers where possible (refactor them slightly if they only accept a `graph` — split into `_draw_nodes_from_dict`, `_draw_edges_from_lists`). Exposes hooks: `on_click(x, y, button)`, `on_drag(x, y)`, `on_release(x, y)`. |
| T11 | Right-side Tk panels (mode toggle, node inspector, start pose, buttons) | T2 | med | `tools/path_planner/editor/panels.py` | Tk Frame containing: mode RadioButton group (Add Node / Move / Edge / Obstacle / Start), node list (Listbox), inspector (name Entry, type Combobox), start-pose panel (node Combobox, heading Spinbox), bottom buttons (Save, Save & Build, Undo, Redo). Each widget reads from the model and emits Commands. |
| T12 | App orchestrator: glue model + commands + canvas + panels | T5, T10, T11 | high | `tools/path_planner/editor/app.py`, `tools/path_planner/edit_field.py` | `class EditorApp`: holds `FieldModel`, `History`, `FieldCanvas`, side `Panels`. Wire mouse events to Commands via current Mode. Wire menu shortcuts (Ctrl+Z, Ctrl+Y, L, B, Del). Implement Save (validate → write JSON → flush AST rewrites) and Save & Build (Save + `build_routes._run`). `edit_field.py` is the CLI entry: `--migrate` calls `editor.migrate.run`; otherwise opens the app. HIGH risk because it integrates everything. |
| T13 | README + DECISIONS update | T3, T12 | low | `tools/path_planner/README.md`, `DECISIONS.md`, `docs/decisions/022-field-data-json-loader.md` | Document new commands in README (migrate + open editor + Build flow). Write ADR-022 capturing the JSON-loader decision per CLAUDE.md's ADR rules. Update the DECISIONS index. |
| T14 | Manual hardware smoke (post-merge gate) | T12 | low | — | After merge, on dev machine: run `--migrate`, open editor, add a test node, rename, save, build, run pytest. This is not a CI task — it's a "before closing the issue" checklist item in the PR description. |

**Parallelism:**
- T1, T2, T6 can run in parallel.
- T3 needs T1 + T2.
- T4 needs T2.
- T5 needs T2.
- T7 needs T1 + T3.
- T8 needs T4 + T5.
- T9 needs T6.
- T10 needs T1 + T2.
- T11 needs T2.
- T12 needs T5 + T10 + T11.
- T13 needs T3 + T12.
- T14 needs T12.

## Notes for Implementer

- **Tuple vs list discipline.** The validator checks `isinstance(coord, tuple)` for `NODES` values and `isinstance(field_mm, tuple)`. JSON has no tuples — the loader **must** convert arrays to tuples for those specific fields. Tests TC1 lock this down.
- **`OBSTACLES` entries are 5-tuples** with a mixed type tail (`str` `kind`). Serialize as `[xmin, ymin, xmax, ymax, kind]`; load back to tuple of `(float, float, float, float, str)`.
- **Don't touch `field_data.py`'s `EDGES` block in migration**, even to reformat. The block is hand-authored and includes long explanatory comments (look at the `# [2] LINE edge demonstrating pre_turn_mode override` entry). Migration only deletes the geometry literals that exist *above* the EDGES section.
- **LibCST is the right tool for EDGES rewrites** (not regex). The `EDGES` entries include nested dicts with `Pivot.LEFT`-style enum references that regex would misclassify. LibCST keeps a fully addressable tree. Look at the existing `EDGES` table — it's small (3 entries today), so the rewrite cost is trivial.
- **Don't try to test the Tk window in pytest.** Headless CI hates Tk. The unit tests cover model + commands + loader + AST rewrite, and that's enough — the GUI layer is thin glue.
- **Mode mouse semantics:**
  - **Add Node**: left-click empty → name dialog → AddNode command.
  - **Move**: left-click + drag on a node → MoveNode command (single undo entry, fired on release).
  - **Edge**: left-click selects (multi-select up to 2 nodes); `L` / `B` adds the edge.
  - **Obstacle**: drag rectangle (corner-to-corner); release → kind dialog → AddObstacle.
- **Hit-testing**: a click is "on a node" if within a pixel radius matching the canvas dot size (steal `PICKER_DOT_SIZE`-ish from `pick_xy.py`). Otherwise "on empty space."
- **Snap-to-grid is NOT in scope** — out of the time budget. Add later as a panel toggle if Phase 3 measurement work calls for it.
- **Rollback plan:** if T6 (LibCST AST rewrite) turns out to be too messy, fall back to "block rename when rich EDGES references the node." That keeps T1–T5 + T7–T14 viable and downgrades only one acceptance criterion.
- **Don't change `pick_xy.py`** — leave it for backwards compatibility with the existing Phase 3 workflow.
- **Do not** update `routes.h` / golden fixtures as part of this PR if the migration leaves the toy graph byte-identical (which it should). If it doesn't, that's a bug in the migration: investigate.
