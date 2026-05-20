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
                            header_writer ──► WRO2026/routes.{h,cpp}
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

- `WRO2026/routes.h`
- `WRO2026/routes.cpp`
- `tools/path_planner/output/map.png`

### Generated `routes.h` schema (what the firmware consumes)

```cpp
namespace Routes {
    constexpr uint8_t N_NODES = N;
    constexpr uint8_t START_NODE = 0;
    constexpr float   START_HEADING_DEG = 0.0f;

    enum class NodeType : uint8_t { LINE = 0, BLIND = 1 };
    enum class EdgeType : uint8_t { NONE = 0, LINE = 1, BLIND = 2, PROJECTION = 3 };

    // Mode enums — per-edge overrides for how the firmware executes the move
    enum class LineMode       : uint8_t { DEFAULT = 0, ... };
    enum class BlindMode      : uint8_t { DEFAULT = 0, ... };
    enum class ProjectionMode : uint8_t { DEFAULT = 0, ... };
    enum class TurnMode       : uint8_t { DEFAULT = 0, ... };

    struct PipelineConfig {
        LineMode line; BlindMode blind; ProjectionMode projection; TurnMode turn;
    };
    extern const PipelineConfig PIPELINE_CONFIG;

    struct NodeInfo  { float x_cm, y_cm; NodeType type; const char* name; };
    struct EdgeInfo  { EdgeType type; float distance_cm, heading_deg; /* per-edge mode overrides */ };

    extern const NodeInfo NODES[N_NODES];
    extern const uint8_t  NEXT_HOP[N_NODES][N_NODES];   // 0xFF = unreachable
    extern const EdgeInfo EDGES[N_NODES][N_NODES];
}
```

`PathTracker` reads `NEXT_HOP[currentNode][goal]` at every intersection to pick the next neighbour, and looks up `EDGES[from][to]` for the per-edge primitive (line-follow, blind drive, projection-follow) and any mode overrides. The defaults in `PIPELINE_CONFIG` apply when an edge's mode field is `DEFAULT`.

## CLI flags

| Flag              | Purpose                                                                                  | Example                                                       |
| ----------------- | ---------------------------------------------------------------------------------------- | ------------------------------------------------------------- |
| `--out-dir DIR`   | Write `routes.h` + `routes.cpp` into `DIR` instead of `WRO2026/`.                        | `... build_routes.py --out-dir build/`                        |
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

Re-running `build_routes.py` against an unchanged `field_data.py`
produces a **byte-identical** `routes.h` / `routes.cpp` pair (AC2). The
`routes.h` banner embeds the SHA256 of `field_data.py`'s **LF-normalized**
bytes (CRLF sequences collapsed to LF first, so the digest matches across
Windows and Linux checkouts regardless of git autocrlf), so reviewers can
verify "the committed header is current with the committed
`field_data.py`" at a glance:

```powershell
python -c "import hashlib, pathlib; \
print(hashlib.sha256(pathlib.Path('tools/path_planner/field_data.py').read_bytes().replace(b'\r\n', b'\n')).hexdigest())"
```

If that hex string does not appear at the top of `routes.h`, regen.

`map.png` is **not** part of the determinism contract — matplotlib
output is not byte-stable across versions, and the runtime never reads
it.

## Editing field_data.py — workflow

1. Edit `tools/path_planner/field_data.py`.
2. Run `uv run python tools/path_planner/build_routes.py`.
3. Open `tools/path_planner/output/map.png` and sanity-check the
   geometry (nodes in the right places, blind edges hop the right
   gaps, no edge clips an obstacle).
4. Commit `field_data.py`, `WRO2026/routes.h`, and
   `WRO2026/routes.cpp` **together** in the same commit. The `routes.h`
   banner SHA256 must equal the **LF-normalized** SHA256 of `field_data.py`
   (CRLF sequences collapsed to LF first, so the digest is stable across
   Windows and Linux checkouts) — see the Determinism section above for
   the one-liner. If reviewers see a mismatch, the commit is incomplete.

## Authoring real-field NODES

Resolves issue #10 scope 1.

Launch the picker from the repo root:

```powershell
uv run python tools/path_planner/pick_xy.py
```

That opens a matplotlib window with `maps/map.png` as the background and
the current `NODES` overlaid as dim grey reference dots. Left-click any
point on the field and the `(x, y)` in centimetres is printed to stdout
to 3 decimal places, with a red marker plus a 1-decimal-place label
dropped at the click for visual reference. Keybinds: `c` clears every
marker (the NODES overlay stays put), `u` undoes the most recent marker.
Close the window to exit. Paste the printed coordinates straight into
the `NODES` dict in `field_data.py`.

`field_data.py` is the single source of truth for both the on-robot
routes (`WRO2026/routes.h` / `routes.cpp`) and the Python simulator
(`tools/path_planner/sim/`). Once you have authored real NODES,
regenerate with `uv run python tools/path_planner/build_routes.py`; both
the robot's compiled next-hop table and the sim's map snap to the new
coordinates automatically — there is no separate "robot field" and
"sim field" to keep in sync.

## Module layout

| File                | Role                                                                                |
| ------------------- | ----------------------------------------------------------------------------------- |
| `field_data.py`     | Hand-authored field: nodes, line segments, projections, goals, obstacles.           |
| `validator.py`      | Strict shape + reference checks. Raises `FieldDataError` on the first problem.      |
| `graph_builder.py`  | Builds a `networkx.DiGraph`, inflates obstacles by `ROBOT_RADIUS_CM`, drops clipped edges. |
| `planner.py`        | Dijkstra all-pairs next-hop table; `cost = distance * (mult if blind else 1.0)`.    |
| `compiler.py`       | Heading-aware primitive emission for downstream `PathTracker`.                      |
| `pipeline_config.py`| Default `LineMode` / `BlindMode` / `ProjectionMode` / `TurnMode` baked into `routes.cpp`. |
| `visualizer.py`     | Matplotlib renderer for `map.png` (Agg backend, headless-safe).                     |
| `header_writer.py`  | Emits `routes.h` + `routes.cpp`; embeds `field_data.py`'s SHA256 in the banner.     |
| `build_routes.py`   | This script — sequences every layer above, decides exit codes.                      |
| `pick_xy.py`        | Tk/matplotlib GUI for measuring field coordinates off `maps/map.png` (issue #10).   |
| `sim/`              | Pure-Python mirror of the R4 `PathTracker` FSM for hardware-free verification.      |
| `tests/`            | pytest suite + golden files for the whole pipeline.                                 |

## Pointers to deeper docs

- `docs/archive/PATH_PLANNER_SPEC.md` — original architecture spec (heading
  convention, edge types, C-space inflation rules, deterministic
  output guarantees). Archived 2026-05-20; this README supersedes it for
  current usage.
- `docs/PHASE2_DEFERRED.md` — calibration tiers, scenarios,
  `wall_zero`, and other items intentionally left for a later phase.
- `docs/archive/plans/` — historical PR plans (PR1 path planner, PR2
  PathTracker runtime, PR3 Mission DSL, pipeline-modes, pick_xy).
