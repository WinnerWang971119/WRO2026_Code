# WRO 2026 Path Planner — Offline Build Pipeline

A small Python pipeline that turns hand-authored field data into the
two C++ files (`routes.h`, `routes.cpp`) the on-robot runtime walks at
every line node. The whole thing runs offline on a developer machine;
the robot never executes any of this code.

```
field_data.py  ──►  validator  ──►  graph_builder (C-space inflation)
                                          │
                                          ▼
                                planner (Dijkstra all-pairs)
                                          │
                                          ▼
                            header_writer ──► WRO2026/src/nav/routes.{h,cpp}
                                          │
                                          └─► visualizer ──► output/map.png
```

The on-robot `PathTracker` (PR 2) reads the next-hop table at every
line intersection to decide which neighbour to step toward — the
expensive shortest-path math has already been done at build time.

## Prerequisites

- **Python 3.11+**.
- One of the two install paths below.

### Recommended: `uv`

```powershell
winget install astral-sh.uv      # Windows
# or
pipx install uv                  # cross-platform fallback
```

Then from the repo root:

```powershell
uv sync --extra dev
```

That reads `pyproject.toml` + `uv.lock` and pulls in `networkx`,
`matplotlib`, `shapely`, and the `dev` extra containing `pytest`.

`uv.lock` is intentionally committed to keep developer installs reproducible
and to keep GitHub Actions on the same dependency graph. The CI workflow uses
`uv sync --extra dev --frozen`, so the lockfile remains the source of truth.

### Fallback: plain venv + pip

Windows (PowerShell):

```powershell
python -m venv .venv
.\.venv\Scripts\pip install -r tools\path_planner\requirements.txt
.\.venv\Scripts\activate
```

Linux / macOS:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r tools/path_planner/requirements.txt
```

## Regenerate routes.h / routes.cpp

One command, from the repo root:

```powershell
uv run python tools/path_planner/build_routes.py
```

(or, with a plain venv, just `python tools/path_planner/build_routes.py`
once the venv is active).

That writes:

- `WRO2026/src/nav/routes.h`
- `WRO2026/src/nav/routes.cpp`
- `tools/path_planner/output/map.png`

### Generated `routes.h` schema (what the firmware consumes)

```cpp
namespace Routes {
    constexpr uint8_t N_NODES = N;
    constexpr uint8_t START_NODE = 0;
    constexpr float   START_HEADING_DEG = 0.0f;

    // projection is a NODE type (issue #64), not an edge type; edges are LINE/BLIND only.
    enum class NodeType : uint8_t { LINE = 0, BLIND = 1, RESET = 2, PROJECTION = 3 };
    enum class EdgeType : uint8_t { NONE = 0, LINE = 1, BLIND = 2 };

    // Mode enums — per-edge overrides for how the firmware executes the move
    enum class LineMode : uint8_t { DEFAULT = 0, ... };
    enum class BlindMode : uint8_t { DEFAULT = 0, ... };
    enum class TurnMode : uint8_t { DEFAULT = 0, ... };

    struct PipelineConfig {
        LineMode line; BlindMode blind; TurnMode turn;
    };
    extern const PipelineConfig PIPELINE_CONFIG;

    struct NodeInfo  { float x_cm, y_cm; NodeType type; const char* name; };
    struct EdgeInfo  { EdgeType type; float distance_cm, heading_deg; /* per-edge mode overrides */ };

    extern const NodeInfo NODES[N_NODES];
    extern const uint8_t  NEXT_HOP[N_NODES][N_NODES];   // 0xFF = unreachable
    extern const EdgeInfo EDGES[N_NODES][N_NODES];
}
```

`PathTracker` reads `NEXT_HOP[currentNode][goal]` at every intersection to pick the next neighbour, and looks up `EDGES[from][to]` for the per-edge primitive and any mode overrides. The primitive falls out of `(edge type, destination node type)`: a LINE edge line-follows (to a junction, or distance-terminated into a projection node); a BLIND edge dead-reckons (or auto-seeks + re-anchors when its destination is a projection node). The defaults in `PIPELINE_CONFIG` apply when an edge's mode field is `DEFAULT`.

## Editing the field graph

### Live GUI (recommended)

```powershell
uv run python tools/path_planner/edit_field.py
```

Opens a Tk window with a matplotlib canvas overlaying `maps/map.png` on the left and
control panels on the right (mode toggle, node list + inspector, start-pose panel,
Save / Save & Build buttons).

| Mode | Action |
| --- | --- |
| Add Node | Left-click empty canvas → enter name + type (`line` / `blind` / `projection` / `reset`) |
| Move | Left-click + drag a node |
| Edge | Left-click selects up to 2 nodes; `L` adds a line edge; `B` adds a blind edge |
| Obstacle | Drag a rectangle; release → enter `kind` label |
| Start | Click a node to set it as `START_NODE` |

Right-click on a node: Rename / Set type / Delete (Delete refuses if `EDGES` references
the node; remove the override row in the EDGE OVERRIDES panel first).

Shortcuts: `Ctrl-Z` undo, `Ctrl-Y` / `Ctrl-Shift-Z` redo, `Ctrl-S` save,
`Delete` removes the selected edge (Edge mode with 2 nodes selected).

**Save** validates the model and writes `field_data.json` (pure-JSON; node renames cascade
into any `EDGES` override entries automatically through the model — no external source
rewriting). **Save & Build** does Save, then regenerates `routes.h` + `routes.cpp` in-process.

The inspector panel (right side) exposes three additional collapsible sections:

- **EDGE OVERRIDES** — add/edit/delete per-edge primitive overrides (`primitive_mode`,
  `direction`, `stop_at_line`). Edge `type` is auto-derived from the selected segment
  (read-only). Clicking an existing edge in Edge mode locates its override row or pre-fills
  the add-row. Override entries use string-tag names verbatim: `NORMAL`/`SLOW`/`FAST` for
  LINE edges, `SYNC_TSHAPE`/`GYRO_TSHAPE`/`RAW_FLAT`/`STOP_AT_LINE` for BLIND edges.
  Geometry (distance/heading) is derived from node coords — not shown or stored.
- **PIPELINE DEFAULTS** — four comboboxes (`Line`, `Blind`, `Turn`, `Direction`) that set
  field-sourced `PIPELINE_CONFIG` defaults. Absent → `pipeline_config.py` fallback (routes
  stay byte-identical). Line/Blind/Turn may not be `DEFAULT`; Direction may.
- **GLOBAL KNOBS** (extended) — `blind_projection_auto_seek` toggle and `FIELD_MM`
  width/height fields, in addition to the existing blind-decay and robot-radius controls.

### Reading coordinates only

```powershell
uv run python tools/path_planner/pick_xy.py
```

Left-click any point on the field and the `(x, y)` in centimetres is printed to
stdout. Read-only; does not modify `field_data.*`.

### Migration (one-shot)

```powershell
uv run python tools/path_planner/edit_field.py --migrate
```

Ports the legacy Python-literal `field_data.py` to `field_data.json` + a thin loader
stub. Any Python-literal `EDGES = [...]` entries are migrated into `field_data.json`'s
`EDGES` key as override-only string-tag dicts; `field_data.py` is rewritten to
`EDGES = _data.get("EDGES", [])`. Idempotent: subsequent runs are no-ops.

### Hand-editing JSON directly

`field_data.json` is plain JSON — you can edit it in any text editor. The schema
mirrors the old Python literal keys. Run `build_routes.py` afterwards to regenerate
`routes.h` / `routes.cpp`.

## CLI flags

| Flag              | Purpose                                                                                  | Example                                                       |
| ----------------- | ---------------------------------------------------------------------------------------- | ------------------------------------------------------------- |
| `--out-dir DIR`   | Write `routes.h` + `routes.cpp` into `DIR` instead of `WRO2026/src/nav/`.                | `... build_routes.py --out-dir build/`                        |
| `--map-out PATH`  | Override the visualization output path.                                                  | `... build_routes.py --map-out /tmp/map.png`                  |
| `--no-viz`        | Skip rendering `map.png` entirely (useful in CI).                                        | `... build_routes.py --no-viz`                                |
| `--verbose`, `-v` | Print per-step progress (`validating ...`, `building graph ...`, etc.) to stderr.        | `... build_routes.py -v`                                      |

## Exit codes

| Code | Meaning                                                                       |
| ---- | ----------------------------------------------------------------------------- |
| `0`  | Success. Both header files (and the map, unless `--no-viz`) were written.     |
| `1`  | Visualization failed, but `routes.h` / `routes.cpp` were still emitted.       |
| `2`  | Validation or data error. `routes.h` was NOT written; fix `field_data.py`.    |

## Running the tests

All tests live under `tools/path_planner/tests/` and run from the
**repo root**, not the package directory — the imports use the
`tools.path_planner.*` form, so `cd`'ing into the package would break
them.

```powershell
uv run --extra dev pytest tools/path_planner/tests/ -v
```

(or `python -m pytest tools/path_planner/tests/ -v` with a plain venv).

## Determinism

Re-running `build_routes.py` against unchanged inputs produces a
**byte-identical** `routes.h` / `routes.cpp` pair (AC2). The `routes.h`
banner embeds the SHA256 of both field-data sources — `field_data.py`
(rich `EDGES` table) **and** `field_data.json` (geometry) — over their
**LF-normalized** bytes (CRLF sequences collapsed to LF first, so the digest
matches across Windows and Linux checkouts regardless of git autocrlf),
joined with a NUL byte. Geometry lives in the JSON, so it must be folded in
or a coordinate edit would leave the stamp unchanged. Reviewers can verify
"the committed header is current with both committed sources" at a glance:

```powershell
python -c "import hashlib, pathlib; h=hashlib.sha256(); \
[h.update((b'\0' if i else b'') + pathlib.Path(p).read_bytes().replace(b'\r\n', b'\n')) \
 for i, p in enumerate(['tools/path_planner/field_data.py', 'tools/path_planner/field_data.json'])]; \
print(h.hexdigest())"
```

If that hex string does not appear at the top of `routes.h`, regen.

`map.png` is **not** part of the determinism contract — matplotlib
output is not byte-stable across versions, and the runtime never reads
it.

## Editing field_data — workflow

Since [ADR-022](../../docs/decisions/022-field-data-json-loader.md) and
[ADR-047](../../docs/decisions/047-json-edges-overrides-and-pipeline-config.md),
`field_data.json` is the single source of truth for geometry **and** per-edge EDGES
overrides. `field_data.py` is a thin loader; use the GUI or hand-edit the JSON directly
(see [Editing the field graph](#editing-the-field-graph) above).

After any edit:

1. Run `uv run python tools/path_planner/build_routes.py`.
2. Open `tools/path_planner/output/map.png` and sanity-check the
   geometry (nodes in the right places, blind edges hop the right
   gaps, no edge clips an obstacle).
3. Commit `field_data.json`, `WRO2026/src/nav/routes.h`, and `WRO2026/src/nav/routes.cpp`
   **together** in the same commit. The `routes.h` banner SHA256 must equal the
   combined **LF-normalized** SHA256 of `field_data.py` + `field_data.json` — see
   the Determinism section above for the one-liner. If reviewers see a mismatch,
   the commit is incomplete.

## Authoring real-field NODES

Use the live GUI (`edit_field.py`) in **Add Node** mode to place nodes directly on the
map image — see [Editing the field graph](#editing-the-field-graph) above. The GUI
writes coordinates to `field_data.json` on Save.

For a read-only coordinate probe (issue #10 scope 1), use `pick_xy.py`:

```powershell
uv run python tools/path_planner/pick_xy.py
```

That opens a matplotlib window with `maps/map.png` as the background and
the current NODES overlaid as dim grey reference dots. Left-click any
point on the field and the `(x, y)` in centimetres is printed to stdout
to 3 decimal places. Keybinds: `c` clears every marker, `u` undoes the
most recent marker. Close the window to exit.

`field_data.json` (loaded by `field_data.py`) is the single source of truth for both
the on-robot routes (`WRO2026/src/nav/routes.h` / `routes.cpp`) and the Python simulator
(`tools/path_planner/sim/`). Once you have authored real NODES, regenerate with
`uv run python tools/path_planner/build_routes.py`; both the robot's compiled
next-hop table and the sim's map snap to the new coordinates automatically — there is
no separate "robot field" and "sim field" to keep in sync.

## Module layout

| File                    | Role                                                                                |
| ----------------------- | ----------------------------------------------------------------------------------- |
| `field_data.json`       | Machine- and human-editable geometry: nodes, edges, obstacles, start pose.          |
| `field_data.py`         | Thin loader (`load_field_json()`); reads `EDGES` from `field_data.json`.            |
| `validator.py`          | Strict shape + reference checks. Raises `FieldDataError` on the first problem.      |
| `graph_builder.py`      | Builds a `networkx.DiGraph`, inflates obstacles by `ROBOT_RADIUS_CM`, drops clipped edges. |
| `planner.py`            | Dijkstra all-pairs next-hop table; `cost = distance * (mult if blind else 1.0)`.    |
| `compiler.py`           | Heading-aware primitive emission for downstream `PathTracker`.                      |
| `pipeline_config.py`    | Default `LineMode` / `BlindMode` / `TurnMode` baked into `routes.cpp`. |
| `visualizer.py`         | Matplotlib renderer for `map.png` (Agg backend, headless-safe).                     |
| `header_writer.py`      | Emits `routes.h` + `routes.cpp`; embeds `field_data.py`'s SHA256 in the banner.     |
| `build_routes.py`       | CLI orchestrator — sequences every layer above, decides exit codes.                 |
| `edit_field.py`         | Entry point for the live Tk field-graph editor (issue #10 scope 2).                 |
| `editor/loader.py`      | JSON load + deterministic dump for `field_data.json`.                               |
| `editor/migrate.py`     | One-shot port from legacy Python-literal `field_data.py` to JSON; migrates any Python `EDGES` literal into `field_data.json`. |
| `pick_xy.py`            | Read-only Tk/matplotlib coordinate probe against `maps/map.png` (issue #10 scope 1). |
| `sim/`                  | Pure-Python mirror of the R4 `PathTracker` FSM for hardware-free verification.      |
| `tests/`                | pytest suite + golden files for the whole pipeline.                                 |

## Pointers to deeper docs

- `docs/archive/PATH_PLANNER_SPEC.md` — original architecture spec (heading
  convention, edge types, C-space inflation rules, deterministic
  output guarantees). Archived 2026-05-20; this README supersedes it for
  current usage.
- `docs/PHASE2_DEFERRED.md` — calibration tiers, scenarios,
  `wall_zero`, and other items intentionally left for a later phase.
- `docs/archive/plans/` — historical PR plans (PR1 path planner, PR2
  PathTracker runtime, PR3 Mission DSL, pipeline-modes, pick_xy).
