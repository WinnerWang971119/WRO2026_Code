# WRO 2026 Sim Map + XY Picker — Plan

**Goal:** Drop in the real WRO 2026 Senior Mosaic Masters game-mat background and ship a build-time matplotlib click-to-read coordinate tool, so Diego can author real-field NODE coordinates by clicking on the printed mat image instead of measuring with a ruler. Resolves issue #10 scope 1.

**Approach:** Three-file change with one new test. (1) Overwrite `tools/path_planner/maps/map.png` with the real WRO mat PNG. (2) Adjust `FIELD_MM` in `field_data.py` to the placeholder `(2500, 1000)` mm (Diego remeasures tomorrow). (3) Add `tools/path_planner/pick_xy.py` — a self-contained matplotlib script that overlays existing NODES on the mat and prints clicked (x, y) in cm. Toy NODES stay untouched this PR so the test suite stays green; replacing them with real measurements is Diego's follow-up.

## Scope

- **In scope:**
  - Replace `tools/path_planner/maps/map.png` with `C:\Users\wangd\Downloads\WRO-2026-GameMat-Senior-Printing-File-1.png` (copied as-is, no cropping).
  - Update `FIELD_MM = (3000, 2000)` → `FIELD_MM = (2500, 1000)` in `field_data.py` with a `# PLACEHOLDER — remeasure` comment.
  - New file `tools/path_planner/pick_xy.py`: matplotlib click-to-read coordinate tool. CLI entry `uv run python tools/path_planner/pick_xy.py`.
  - New test `tools/path_planner/tests/test_pick_xy.py`: covers pure helpers only.
  - README section in `tools/path_planner/README.md` titled "Authoring real-field NODES" pointing at `pick_xy.py` and noting `field_data.py` is the single source feeding both `routes.h` and the sim.
  - Plan doc at `docs/plans/2026-05-17-issue10-scope1-pick-xy.md` (this spec).
  - Cut branch `feat/pick-xy-tool` off `main`; open PR to `main` when ready.

- **Out of scope:**
  - Replacing toy `NODES`/`LINE_SEGMENTS`/`EDGES`/`BLIND_EDGES` — Diego does this himself in a follow-up commit after physical measurement.
  - Regenerating `WRO2026/routes.h` / `WRO2026/routes.cpp` / `output/map.png` — `FIELD_MM` change alone doesn't shift node positions, so artefacts stay consistent. Diego regens after authoring real NODES.
  - Click-to-create / drag-to-reposition node editing (issue #10 scope 2).
  - Edge / blind-edge authoring UI.
  - Two-click pixel-to-cm calibration UI (FIELD_MM = full image extent assumption is good enough).
  - JSON export, write-back to `field_data.py`, auto-rebuild after edits.
  - Updates to `ARCHITECTURE.md` / `DECISIONS.md` / `Roadmap.md` — this is a build-time authoring tool, not architecture.

## Decisions

- **Picker behaviour: accumulate markers; `c` clears all, `u` undoes last** — accumulate is the natural mode for measuring many nodes per session; the two keybinds give clean recovery without growing the surface much.
- **Missing PNG: hard-fail with `sys.exit(2)`** — picker is useless without the background; better to fail loudly than render a blank grid that looks valid.
- **Off-axes click: silently ignore** — `event.xdata is None` outside axes; nothing useful to print; no warning needed.
- **Stdout precision: 3 decimal places** (`(12.345, 45.678)`) — paste-ready for `field_data.py`.
- **Plot label precision: 1 decimal place** (`(12.3, 45.7)`) — keeps figure readable when many markers accumulate.
- **Origin: lower-left, +y up** — matches `field_data.py` convention; use `ax.imshow(img, extent=(0,W,0,H), origin='lower')`.
- **Axes extent: full image = FIELD_MM** — corners of the PNG line up with `(0,0)` and `(FIELD_MM[0]/10, FIELD_MM[1]/10)` cm. Recalibrates automatically when Diego updates `FIELD_MM`.
- **matplotlib backend: system default** — don't call `matplotlib.use(...)`; default is TkAgg on Windows + uv. (Different from `visualizer.py` which uses `Agg` for headless rendering — the picker needs interactivity.)
- **NODES overlay style: alpha=0.4, grey** — dim enough to read through, bright enough to use as reference.
- **NODES overlay source: read existing `field_data.NODES`** — no live update; just snapshot at launch.
- **Toy NODES untouched** — Diego rewrites tomorrow; this PR avoids ~7 test files of cascading churn.
- **Test depth: pure helpers only** — `_field_extent_cm`-style coord conversion + label-formatting function. No matplotlib event simulation.
- **No CLI flags** — keep it dumb. Use `field_data.MAP_PNG` and `FIELD_MM` directly.

## Acceptance Criteria

- [ ] `tools/path_planner/maps/map.png` is the WRO 2026 Senior Mosaic Masters game mat (file size > 100 kB; matches source byte-for-byte).
- [ ] `field_data.FIELD_MM == (2500, 1000)` with a `# PLACEHOLDER — remeasure tomorrow` comment.
- [ ] Running `uv run python tools/path_planner/pick_xy.py` opens a matplotlib window showing the mat as background, existing NODES overlaid in dim grey with name labels, and a title indicating usage.
- [ ] Left-clicking inside the axes prints `(x.xxx, y.yyy)` (3 dp) in cm to stdout AND draws a red dot + `(x.x, y.y)` (1 dp) text label on the figure.
- [ ] Multiple clicks accumulate; pressing `c` clears all picker markers (NODES overlay preserved); pressing `u` removes the most recent marker.
- [ ] Right-click / middle-click / clicks outside the axes do nothing visible and print nothing.
- [ ] If `tools/path_planner/maps/map.png` is missing or unreadable, the script prints a clear stderr message and exits with code 2 (no window opens).
- [ ] Image is drawn with origin at lower-left (matches `field_data.py` SW-corner convention).
- [ ] `pytest tools/path_planner/tests/test_pick_xy.py -v` runs green from repo root and tests the pure helpers (no GUI invocation).
- [ ] Full existing test suite `pytest tools/path_planner/tests/ -v` still passes (no regression from the `FIELD_MM` change).
- [ ] `tools/path_planner/README.md` has an "Authoring real-field NODES" section pointing at `pick_xy.py` with a one-paragraph note on the field_data.py-is-single-source sync property.

## Data Model

Module-level constants in `pick_xy.py`:

```python
PICKER_DOT_COLOR   = "red"
PICKER_DOT_SIZE    = 60       # matplotlib scatter `s`
PICKER_LABEL_FONTSIZE = 8
PICKER_LABEL_OFFSET_PT = (5, 5)  # textcoords='offset points'

NODES_OVERLAY_COLOR = "0.55"  # mid-grey
NODES_OVERLAY_ALPHA = 0.4
NODES_OVERLAY_DOT_SIZE = 40
NODES_OVERLAY_LABEL_FONTSIZE = 8
```

Internal state (not exported): a single mutable list of matplotlib `Artist` handles for accumulated picker markers, used by the `u` and `c` key handlers.

## Module Layout

```
tools/path_planner/
├── pick_xy.py                       (NEW, ~110 lines)
├── field_data.py                    (1-line edit: FIELD_MM + comment)
├── maps/map.png                     (BINARY OVERWRITE)
├── README.md                        (NEW section ~15 lines)
└── tests/
    └── test_pick_xy.py              (NEW, ~40 lines, pure helpers)

docs/plans/
└── 2026-05-17-issue10-scope1-pick-xy.md   (NEW, this spec)
```

## `pick_xy.py` Function Shape

Module-level functions (mirrors `visualizer.py`):

```python
def _field_extent_cm(field_data) -> tuple[float, float]:
    """Return (width_cm, height_cm) derived from FIELD_MM."""

def _resolve_map_png(field_data) -> Path:
    """Return absolute Path to field_data.MAP_PNG (resolved relative to planner root)."""

def _format_stdout(x: float, y: float) -> str:
    """Return '(12.345, 45.678)' style string for stdout."""

def _format_label(x: float, y: float) -> str:
    """Return '(12.3, 45.7)' style string for on-plot label."""

def _draw_nodes_overlay(ax, field_data) -> None:
    """Plot existing NODES in dim grey on the axes."""

def _make_click_handler(ax, picked_artists: list):
    """Return a closure for mpl_connect('button_press_event')."""

def _make_key_handler(picked_artists: list, fig):
    """Return a closure for mpl_connect('key_press_event'). 'c' clears, 'u' undoes."""

def main() -> int:
    """CLI entry point. Returns 0 on clean exit, 2 if map.png is missing."""

if __name__ == "__main__":
    raise SystemExit(main())
```

## Error Handling

- **`maps/map.png` missing**: `main()` prints `"pick_xy: cannot read map at <path>: <reason>"` to stderr and returns 2. Exception types caught: `FileNotFoundError`, `PermissionError`, and matplotlib's `OSError` on corrupt image.
- **`FIELD_MM` malformed (not a 2-tuple of numbers)**: let the `_field_extent_cm` helper raise the natural `TypeError`/`ValueError` — fatal user-config error, not something to swallow.
- **Click event with `event.xdata is None`** (outside axes): handler short-circuits with `return`. No print, no marker.
- **Click event with `event.button != 1`** (right/middle): handler short-circuits with `return`.
- **`u` keypress with empty picker list**: handler short-circuits.

## Testing Strategy

**Levels:** Unit only — matplotlib event simulation is brittle and the GUI logic is thin.

| ID  | Test Case                                                | Type | Expected Behavior |
|-----|----------------------------------------------------------|------|-------------------|
| TC1 | `_field_extent_cm` on `FIELD_MM=(2500,1000)`             | Unit | Returns `(250.0, 100.0)` |
| TC2 | `_field_extent_cm` on `FIELD_MM=(3000,2000)`             | Unit | Returns `(300.0, 200.0)` |
| TC3 | `_format_stdout(12.3456, 45.6789)`                        | Unit | Returns `'(12.346, 45.679)'` |
| TC4 | `_format_stdout(0.0, 0.0)`                                | Unit | Returns `'(0.000, 0.000)'` |
| TC5 | `_format_label(12.345, 45.678)`                           | Unit | Returns `'(12.3, 45.7)'` |
| TC6 | `_resolve_map_png(field_data)` returns absolute path      | Unit | Path is absolute + ends with `maps/map.png` |
| TC7 | `_resolve_map_png` with `MAP_PNG=None`                    | Unit | Returns `None` (mirrors visualizer behaviour) |
| TC8 | Existing test suite `pytest tools/path_planner/tests/`    | Integration | All passes — no regression from `FIELD_MM` change |

**Test data:** No fixtures needed; pure-function tests use literal inputs. TC8 leans on existing tests (no new fixture).

**Run command:** `uv run --extra dev pytest tools/path_planner/tests/ -v`

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Branch + copy WRO mat PNG | — | low | `tools/path_planner/maps/map.png` | `git checkout main && git pull && git checkout -b feat/pick-xy-tool`. Then `Copy-Item "C:\Users\wangd\Downloads\WRO-2026-GameMat-Senior-Printing-File-1.png" "tools\path_planner\maps\map.png" -Force`. Verify `(Get-Item ...).Length` ≥ 100 kB. Satisfies AC 1. |
| T2 | Update FIELD_MM placeholder | T1 | low | `tools/path_planner/field_data.py` | Change `FIELD_MM = (3000, 2000)` to `FIELD_MM = (2500, 1000)  # PLACEHOLDER — remeasure tomorrow (issue #10)`. No other edits to this file. Satisfies AC 2. |
| T3 | Implement `pick_xy.py` | T2 | med | `tools/path_planner/pick_xy.py` | New file. Module docstring describing purpose + `uv run` command. Implement the seven module-level functions in the spec's "Function Shape" section. Use the module-level styling constants from "Data Model". Wire `main()` to: resolve map path → load image via `matplotlib.image.imread` → create figure + axes → `imshow(img, extent=(0,W,0,H), origin='lower')` → set xlim/ylim, aspect=equal, grid → draw NODES overlay → connect button + key handlers → `plt.show()`. On `FileNotFoundError`/`OSError`/`PermissionError` print to stderr and return 2. Imports: only `matplotlib.pyplot`, `matplotlib.image`, `sys`, `pathlib.Path`, `tools.path_planner.field_data`. Satisfies AC 3–8. |
| T4 | Tests for `pick_xy.py` helpers | T3 | low | `tools/path_planner/tests/test_pick_xy.py` | New file. Pattern matches `tests/test_visualizer.py` (module-level test functions, `from tools.path_planner import pick_xy`). Implement TC1–TC7 from the test table. Do NOT call `main()` or invoke matplotlib event loops. Satisfies AC 9. |
| T5 | README "Authoring real-field NODES" section | T3 | low | `tools/path_planner/README.md` | Append a section titled `## Authoring real-field NODES` after "Editing field_data.py — workflow". Two paragraphs: (1) how to launch `pick_xy.py` and the click→stdout flow; (2) the field_data.py-is-single-source sync property — both `routes.h` (robot) and the Python sim read the same file, so editing `NODES` auto-syncs the two pipelines once you rerun `build_routes.py`. Reference issue #10. Satisfies AC 11. |
| T6 | Verify regression-free | T4, T5 | low | — | Run `uv run --extra dev pytest tools/path_planner/tests/ -v` from repo root. Assert: full suite green (including new `test_pick_xy.py`). If any toy-graph test fails: revert `FIELD_MM` change and stop — flag a regression to Diego. Satisfies AC 10. |
| T7 | Manual GUI smoke (operator step) | T3 | low | — | Diego launches `uv run python tools/path_planner/pick_xy.py` once; verifies background loads, NODES overlay shows, click prints + draws marker, `c` clears, `u` undoes, hard-fail message shown when `maps/map.png` is renamed away. Not automated; informational only. |
| T8 | Save plan doc | T1 | low | `docs/plans/2026-05-17-issue10-scope1-pick-xy.md` | Save this approved spec verbatim to the dated plan doc. (Plan skill does this automatically on approval; included for completeness.) |
| T9 | Commit + push + open PR | T6, T8 | low | — | `git add` only the touched files (T1–T5 + T8 outputs). One commit: `feat(planner): pick_xy tool + real WRO mat (#10 scope 1)`. Push `feat/pick-xy-tool`. Open PR to `main`. PR description references issue #10 and links the plan doc. Diego confirms before push. |

**Dependency graph:**

```
T1 ──► T2 ──► T3 ──► T4 ──┐
   │              │       ├──► T6 ──┐
   │              └──► T5 ─┘        │
   └──► T8 ─────────────────────────┴──► T9
                              T7 (manual, parallel)
```

T1 and T8 can run in parallel after the branch is cut. T2 must precede T3 (the picker reads the updated FIELD_MM). T4 and T5 can run in parallel after T3. T6 gates the commit. T7 is manual and informational.

## Notes for Implementer

- **`matplotlib.image.imread` vs `mpimg.imread`**: visualizer.py uses `mpimg.imread` and the existing pattern works — match it.
- **`origin='lower'` is the crucial flag.** Without it, the WRO mat will appear vertically flipped relative to NODES coords.
- **Do not call `matplotlib.use('TkAgg')` or any other backend selection.** Let the system default win; this is the only difference from `visualizer.py` (which hard-pins Agg for headless).
- **Coordinate transform:** matplotlib provides `event.xdata` / `event.ydata` in *axes data coords*, which because of our `extent=(0,W,0,H)` already match cm. No transform math needed.
- **Key handler binding:** `fig.canvas.mpl_connect('key_press_event', ...)`. Matplotlib's default keymap includes `'q'` for quit and a few others — `'c'` and `'u'` may collide with `back`/`undo` defaults in some backends but the conflict is harmless because matplotlib's default just navigates view history; in a fresh figure that's a no-op.
- **`field_data.MAP_PNG` is `"maps/map.png"`** (relative to `tools/path_planner/`); resolve via `_PLANNER_ROOT = Path(__file__).resolve().parent` mirror of visualizer.py.
- **Rollback:** if T6 surfaces a test regression you can't fix in 15 min, `git restore tools/path_planner/field_data.py` to put `FIELD_MM` back to `(3000, 2000)` and ship without the FIELD_MM change. The picker still works with old FIELD_MM; the real-mat PNG still helps even if axes extent is wrong.
- **PR title cap**: 70 chars. `feat(planner): pick_xy tool + real WRO mat (#10 scope 1)` is 56.
