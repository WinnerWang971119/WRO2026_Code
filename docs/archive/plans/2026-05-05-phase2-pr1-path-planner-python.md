# Phase 2 PR 1 — Path Planner Python Build Pipeline

**Status:** Approved 2026-05-05. Awaiting execute.

**Goal:** Land a laptop-side Python build pipeline that turns a hand-authored field description (nodes, line segments, projection waypoints, goal approaches, obstacles) into a `routes.h` of all-pairs NEXT_HOP tables + edge metadata + goal blind-move sequences, with a matplotlib visualization and a property-based test suite. Zero chassis dependency.

**Approach:** Implement the spec section 10 file layout under `tools/path_planner/`, exactly as: `field_data.py` → `graph_builder.py` (incl. C-space inflation) → `planner.py` (Dijkstra all-pairs) → `compiler.py` (heading propagation, blind-move emission) → `visualizer.py` (map.png) → `header_writer.py` (routes.h emit). Toy mixed-feature graph as the test fixture. Continuous float degrees throughout (no N/E/S/W enum). Generated `routes.h` checked into `WRO2026/routes.h` so the sketch always compiles without running Python first. Tooling: `uv` + `pyproject.toml` + `pytest` + `networkx` + `matplotlib`.

## Phase 2 Delivery Context

This is **PR 1 of 3** for Phase 2. The full split:

| PR | Branch | Scope | Verifiable on |
|----|--------|-------|---------------|
| **PR 1 (this spec)** | `phase2/path-planner-python` | `tools/path_planner/` build pipeline. Toy graph fixture. No chassis dependency. | Laptop only |
| PR 2 | `phase2/path-tracker-runtime` | R4 runtime (`FieldMap`, `Localization`, `PathTracker`, `follow_line_distance` primitive). Wires into `WRO2026.ino`. Same toy graph as PR 1. | Robot + toy printout |
| PR 3 | `phase2/field-2026-and-mission` | Real 2026 field encoded; Serial-console mission entrypoint. Reduced-scale print test runs. | Robot + reduced-scale field |

PR 2 and PR 3 each get their own plan-skill cycle when their turn comes.

## Architecture Note

This phase implements the **offline Python pipeline** architecture per [`docs/PATH_PLANNER_SPEC.md`](../PATH_PLANNER_SPEC.md), which supersedes the runtime-Dijkstra sketch in [`ARCHITECTURE.md`](../../ARCHITECTURE.md) and [`Roadmap.md`](../../Roadmap.md). Those two docs will be reconciled to the new architecture as Phase 2 lands.

## Scope

- **In scope:**
  - `tools/path_planner/` build pipeline (8 Python modules per spec section 10).
  - Continuous-degree heading propagation in compiler + tests.
  - Obstacle inflation (C-space) by `ROBOT_RADIUS_CM`; edges that intersect inflated obstacles are dropped.
  - Toy mixed-feature graph as test fixture: 5 line nodes (A, B, C, D, E) in a plus shape, 1 projection node (`_proj_G`) on segment B-C, 1 goal node (G) reachable via projection + blind moves, 1 obstacle that forces a detour.
  - Visualizer (`map.png`): nodes colored by type, edges colored by type (line/projection/blind), inflated obstacles outlined, path highlighting overlay.
  - Header writer emits `WRO2026/routes.h` (auto-generated, checked in).
  - `pytest` suite covering field-data validation, graph builder, planner cost tradeoffs, compiler heading, header writer (golden-file).
  - `docs/PHASE2_DEFERRED.md` documenting what's not built (calibration tiers, scenarios, wall_zero, end-to-end simulator).
  - `tools/path_planner/README.md` with one-shot setup + run instructions.
  - `pyproject.toml` (uv-managed) + `requirements.txt` (fallback for plain venv users).

- **Out of scope:**
  - **R4 runtime** (`FieldMap`, `Localization`, `PathTracker`, new primitives) — that's PR 2.
  - **Real 2026 field data** in `field_data.py` — that's PR 3. PR 1 ships only the toy graph.
  - **Calibration nodes, blind-cost tiers, constrained Dijkstra, per-scenario NEXT_HOP tables, wall_zero primitive** — deferred to Phase 2.5, captured in `docs/PHASE2_DEFERRED.md`.
  - **End-to-end pure-Python simulator** (spec section 6 step 9) — listed as deferred.
  - **Serial-console mission entrypoint** — that's PR 3.
  - **C++ changes anywhere outside `WRO2026/routes.h` and `WRO2026/routes.cpp`**.

## Decisions

- **Layout:** `tools/path_planner/`. Generated header at `WRO2026/routes.h` + `WRO2026/routes.cpp`.
- **Tooling:** `uv` + `pyproject.toml` + Python 3.11+. `requirements.txt` shipped alongside as a fallback so a plain `python -m venv .venv && pip install -r requirements.txt` workflow also works.
- **Dijkstra:** `networkx.shortest_path` (well-tested, predicate-of-cost edges supported). All-pairs computed by looping `single_source_dijkstra` over every source.
- **Heading representation:** continuous float degrees in `[0, 360)`, with `0° = +x axis` (East), counter-clockwise positive. Each directed edge stores `heading_deg` = `atan2(dy, dx)` of the edge's source-to-target vector. Reverse traversal heading = `(heading_deg + 180) % 360`.
- **Heading tests:** random property-based — generate 200 random graphs, walk random paths, check that compiler-emitted relative turns match `(next_heading − prev_heading)` normalized to `(−180, +180]`. Spec section 7.5's 16 enumerated cases drop out as a special case.
- **Visualizer output path:** `tools/path_planner/output/map.png`. The `output/` dir is gitignored except for `output/.gitkeep`.
- **Generated `routes.h`:** checked in. Header includes a SHA256 of `field_data.py` in a comment block so reviewers can verify the file is up-to-date with the data.
- **Obstacle inflation:** Minkowski-sum approach via `shapely.geometry.Polygon.buffer(ROBOT_RADIUS_CM)`. Edges are line strings; if any inflated obstacle's polygon intersects the line string, the edge is dropped.
- **Toy graph as primary fixture:** all tests run against it. PR 3 will replace `field_data.py` content but not module structure.
- **Branch:** `phase2/path-planner-python` (off `main`).

## Acceptance Criteria

- [ ] **AC1** `uv run python tools/path_planner/build_routes.py` succeeds on the toy graph with exit code 0; emits `WRO2026/routes.h` and `tools/path_planner/output/map.png`.
- [ ] **AC2** Re-running `build_routes.py` produces a byte-identical `routes.h` (idempotent).
- [ ] **AC3** Generated `routes.h` is valid C++ and parses cleanly under `arduino-cli compile WRO2026/` (or, if `arduino-cli` is unavailable, a manual review log of every constexpr declaration).
- [ ] **AC4** `uv run pytest tools/path_planner/tests/` exits 0; all suite categories present (field-data validation, graph builder, planner, compiler, header writer golden-file, visualizer smoke).
- [ ] **AC5** Property-based heading test runs ≥200 random scenarios per session and finds no inconsistency.
- [ ] **AC6** With one obstacle placed mid-graph, planner re-routes around it (Dijkstra returns a longer path that doesn't intersect the inflated polygon).
- [ ] **AC7** Visualizer's `map.png` shows: nodes colored by type (line=black, projection=blue, goal=red), edges colored by type (line=black, projection=blue, blind=red dashed), inflated obstacles as filled gray polygons, an example path highlighted in green.
- [ ] **AC8** `docs/PHASE2_DEFERRED.md` exists and lists every deferred feature with its spec section reference and a one-line "why deferred."
- [ ] **AC9** `tools/path_planner/README.md` documents both the `uv` and plain-venv setup paths plus a single command to regenerate the header.
- [ ] **AC10** `field_data.py` validator rejects: a `PROJECTIONS` entry whose `on_segment` is not in `LINE_SEGMENTS`; a `GOAL_APPROACH` entry whose `entry` is not a known node; a node referenced in `LINE_SEGMENTS` but absent from `NODES`; an obstacle whose corners aren't in `(x_min ≤ x_max, y_min ≤ y_max)` order.
- [ ] **AC11** Branch `phase2/path-planner-python` opened off `main`; commits land per task; nothing pushed to `main`.
- [ ] **AC12** No C++ files modified outside `WRO2026/routes.h` and `WRO2026/routes.cpp`.

## Data Model

### `field_data.py` shape (toy fixture):

```python
# field_data.py — toy mixed-feature graph (PR 3 will replace contents, not structure)

NODES = {
    "A":      (0.0,  0.0),
    "B":      (60.0, 0.0),
    "C":      (60.0, 60.0),
    "D":      (0.0,  60.0),
    "E":      (30.0, 30.0),     # center hub
    "_proj_G": (60.0, 30.0),    # waypoint on segment B-C
    "G":      (90.0, 30.0),     # off-line goal, blind-reachable from _proj_G
}

NODE_TYPES = {
    "A": "line", "B": "line", "C": "line", "D": "line", "E": "line",
    "_proj_G": "projection",
    "G": "goal",
}

# Undirected; graph builder produces two directed edges per segment.
LINE_SEGMENTS = [
    ("A", "B"), ("B", "C"), ("C", "D"), ("D", "A"),
    ("A", "E"), ("B", "E"), ("C", "E"), ("D", "E"),
]

PROJECTIONS = {
    "_proj_G": {"on_segment": ("B", "C"), "from": "B", "distance_cm": 30.0},
}

GOAL_APPROACH = {
    "G": {
        "entry": "_proj_G",
        "approach_heading_deg": 0.0,         # face East on entry
        "blind_moves": [
            ("turn_relative", 0.0),           # already facing right way
            ("forward", 30.0),
        ],
    },
}

OBSTACLES = [
    # (x_min, y_min, x_max, y_max, kind)
    (40.0, 40.0, 50.0, 50.0, "wall"),         # forces detour around quadrant
]

ROBOT_RADIUS_CM = 8.0
COST_BLIND_MULTIPLIER = 5.0

START_NODE = "A"
START_HEADING_DEG = 0.0     # facing East at start
```

### `routes.h` schema (auto-generated):

```cpp
// routes.h — AUTO-GENERATED by tools/path_planner/build_routes.py
// Source SHA256: <hex>

#ifndef ROUTES_H
#define ROUTES_H

#include <stdint.h>

namespace Routes {
    constexpr uint8_t N_NODES = 7;
    constexpr uint8_t N_GOALS = 1;

    enum class NodeType : uint8_t { LINE = 0, PROJECTION = 1, GOAL = 2 };
    enum class EdgeType : uint8_t { NONE = 0, LINE_E = 1, PROJECTION_E = 2, BLIND_E = 3 };
    enum class BlindAction : uint8_t { TURN_RELATIVE = 0, FORWARD = 1 };

    struct NodeInfo { float x_cm; float y_cm; NodeType type; const char* name; };
    struct EdgeInfo { EdgeType type; float distance_cm; float heading_deg; };
    struct BlindMove { BlindAction action; float value; };
    struct GoalApproach {
        uint8_t entry_node;
        float   approach_heading_deg;
        const BlindMove* blind_moves;
        uint8_t blind_move_count;
    };

    extern const NodeInfo NODES[N_NODES];
    extern const uint8_t  NEXT_HOP[N_NODES][N_NODES];      // [from][to] = next-hop node id; 0xFF = unreachable
    extern const EdgeInfo EDGES   [N_NODES][N_NODES];      // sparse; type=NONE for non-edges
    extern const GoalApproach GOAL_APPROACH[N_GOALS];

    constexpr uint8_t START_NODE = 0;
    constexpr float   START_HEADING_DEG = 0.0f;
}
#endif
```

(Definitions for the `extern const` arrays live in a generated `routes.cpp`, also under `WRO2026/`, to keep the .h compile-fast.)

## Error Handling

- **Field data validation failures** (`AC10`): `field_data.py` validator raises `FieldDataError` with the offending key path and a one-line fix hint. `build_routes.py` catches and exits with code 2 + a Serial-friendly summary line.
- **Graph disconnected after C-space inflation:** `graph_builder.py` emits a warning per disconnected node ("Node `X` unreachable after inflation — check ROBOT_RADIUS_CM or obstacle placement"). Build still succeeds with `NEXT_HOP[*][X] = 0xFF` for unreachable destinations.
- **Goal entry not on a path that ends at the entry's heading:** compiler logs which goals had to insert an auto-corrective `turn_to_heading` before the blind sequence. Not an error; just a "watch this" warning.
- **Obstacle bounds invalid:** validator rejects (caught upstream of any geometry math, so no unhelpful Shapely traceback).
- **Visualizer fails to write file:** falls back to `print()`-mode and returns nonzero. `build_routes.py` continues but exits with code 1.

## Testing Strategy

**Levels:** Unit (field_data validator, graph_builder, planner, compiler), Property (compiler heading), Golden-file (header_writer), Smoke (visualizer).

| ID | Test Case | Type | Expected Behavior |
|----|----|----|----|
| TC1 | `PROJECTIONS` references unknown `LINE_SEGMENTS` | Unit | `FieldDataError` with key path |
| TC2 | `GOAL_APPROACH.entry` references unknown node | Unit | `FieldDataError` |
| TC3 | Node referenced in `LINE_SEGMENTS` but missing from `NODES` | Unit | `FieldDataError` |
| TC4 | Segment B-C with projection at distance=30 splits into B-_proj_G-C | Unit | Two edges, original B-C edge gone |
| TC5 | Edge B→C and edge C→B exist; their `heading_deg` differ by exactly 180° | Unit | Modulo 360 |
| TC6 | Obstacle covers segment midpoint; edge dropped after inflation | Unit | Edge absent in output graph |
| TC7 | Dijkstra prefers a longer line route over a shorter blind shortcut when `COST_BLIND_MULTIPLIER ≥ 5` | Unit | Path goes the long way |
| TC8 | NEXT_HOP[a][a] = a (self-loop sentinel) | Unit | True for every node |
| TC9 | 200 random graphs × random A→B paths: relative turns from compiler match manual `(next−prev)` mod 360 | Property | No mismatches |
| TC10 | Goal blind sequence emits `turn_relative` from `approach_heading_deg` to first move | Unit | Angles match data |
| TC11 | `header_writer` emits `routes.h` matching golden fixture for toy graph | Golden | byte-equal to checked-in fixture |
| TC12 | Re-run `build_routes.py` → identical `routes.h` byte-for-byte | Unit | hash unchanged |
| TC13 | Visualizer writes `output/map.png`, size > 1KB | Smoke | file exists & non-trivial |
| TC14 | Disconnected goal after inflation produces warning, not crash; `NEXT_HOP[*][G] = 0xFF` | Unit | Warning logged, build succeeds |

**Test data:** all tests run against the toy graph fixture (committed). Property tests generate random graphs in-memory.

**Run command:** `uv run pytest tools/path_planner/tests/ -v`

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Branch + skeleton + tooling | — | low | `pyproject.toml`, `tools/path_planner/__init__.py`, `tools/path_planner/requirements.txt`, `tools/path_planner/output/.gitkeep`, `.gitignore` | Open `phase2/path-planner-python` off `main`. Add `pyproject.toml` (uv) + `requirements.txt` (fallback) declaring `networkx>=3.2`, `matplotlib>=3.8`, `shapely>=2.0`, `pytest>=8.0`. Create empty `tools/path_planner/` package + `output/` (gitignored except `.gitkeep`). Verify `uv sync` works on Diego's Windows machine. Satisfies AC9 setup half. |
| T2 | `field_data.py` + validator | T1 | med | `tools/path_planner/field_data.py`, `tools/path_planner/validator.py`, `tools/path_planner/tests/test_field_data.py` | Author the toy fixture (NODES, NODE_TYPES, LINE_SEGMENTS, PROJECTIONS, GOAL_APPROACH, OBSTACLES, ROBOT_RADIUS_CM, COST_BLIND_MULTIPLIER, START_NODE, START_HEADING_DEG). Author `validator.validate(field_data) -> None | raises FieldDataError`. Tests TC1–TC3. Satisfies AC10. |
| T3 | `graph_builder.py` + obstacle inflation | T2 | high | `tools/path_planner/graph_builder.py`, `tools/path_planner/tests/test_graph.py` | Build a `networkx.DiGraph`. For each `LINE_SEGMENTS` entry, emit two directed edges; if the segment has a projection on it, split into two sub-edges via the projection node. For each goal, emit `BLIND_E` edges from `entry` to `goal`. Compute heading_deg per directed edge. Apply C-space inflation via `shapely.buffer`; drop any edge whose linestring intersects an inflated polygon. Tests TC4–TC6, TC14. |
| T4 | `visualizer.py` | T3 | low | `tools/path_planner/visualizer.py`, `tools/path_planner/tests/test_visualizer.py` | Render nodes (colored by type), edges (colored by type), inflated obstacles (gray fill), and an optional path highlight (green) to `output/map.png`. Smoke test TC13. Satisfies AC7. |
| T5 | `planner.py` (Dijkstra all-pairs) | T3 | high | `tools/path_planner/planner.py`, `tools/path_planner/tests/test_planner.py` | Compute NEXT_HOP[from][to] for every pair via repeated `nx.single_source_dijkstra`. Cost = distance × (1 if line/projection else COST_BLIND_MULTIPLIER). Unreachable pairs encoded as `0xFF`. Tests TC7, TC8. |
| T6 | `compiler.py` (heading propagation + blind moves) | T5 | high | `tools/path_planner/compiler.py`, `tools/path_planner/tests/test_compiler.py` | Given a path `[n0, n1, ..., goal]`, walk it: starting from `START_HEADING_DEG`, compute the relative turn at each node as `(edge_heading − current_heading)` normalized to `(−180, +180]`, then on goal arrival emit the GOAL_APPROACH blind sequence (auto-insert `turn_to_heading(approach_heading_deg)` if needed). Property test TC9 (200 random graphs). Tests TC10. |
| T7 | `header_writer.py` (emit routes.h) | T5, T6 | med | `tools/path_planner/header_writer.py`, `WRO2026/routes.h`, `WRO2026/routes.cpp`, `tools/path_planner/tests/test_header_writer.py`, `tools/path_planner/tests/golden/routes_toy.h` | Emit `routes.h` (declarations) and `routes.cpp` (definitions) per the schema in this spec. Embed source SHA256 in a comment block. Golden-file test TC11. Idempotency test TC12 (run twice → byte-equal). Satisfies AC2, AC3 (header validity). |
| T8 | `build_routes.py` main script | T4, T7 | med | `tools/path_planner/build_routes.py`, `tools/path_planner/README.md` | Wire everything: load → validate → build graph → plan → compile → emit header → render visualization. Argparse: `--out PATH`, `--no-viz`, `--verbose`. Exit codes 0/1/2 per Error Handling. README documents both `uv` and plain-venv paths + the regen command. Satisfies AC1, AC9. |
| T9 | `docs/PHASE2_DEFERRED.md` | T8 | low | `docs/PHASE2_DEFERRED.md` | Record deferred features with spec section refs: calibration nodes + tiers + constrained Dijkstra (§8.5), per-scenario tables (§7.4), wall_zero primitive (§5.3), end-to-end Python simulator (§6 step 9), serial-console mission entrypoint (PR 3). One paragraph each. Satisfies AC8. |
| T10 | Final verify + commit hygiene | T8, T9 | low | (no new files) | Run `uv run pytest`, regenerate `routes.h`, run `arduino-cli compile WRO2026/` if available else manual audit. Confirm AC12 (no unrelated C++ touched). Squash-merge-ready commit log. |

T1 has no blockers — first task. T2 depends on T1. T3 depends on T2 (validator must exist before graph builder uses it). T4 only needs T3's graph data. T5 only needs T3's graph. T6 needs T5's planner output. T7 needs both T5 and T6. T8 needs T4 (visualization wiring) and T7 (header). Tests inside each task touch only that task's files — no cross-task file conflicts.

## Notes for Implementer

- **Heading sign convention is non-negotiable:** `0° = +x` (East), CCW-positive. The R4's IMU yaw is also CCW-positive on this hardware, so PR 2's `Drive.turnToYaw` will consume `heading_deg` directly without sign flips. Document this at the top of `compiler.py`.
- **`routes.h` is a generated artifact:** never hand-edit. The header has a `// AUTO-GENERATED` warning + source SHA256. PR review should reject any unsolicited diff to it that isn't accompanied by a `field_data.py` change with matching hash.
- **Idempotency matters more than it looks:** if running `build_routes.py` twice produces different bytes (e.g. dict iteration order, timestamp drift), reviewers can't diff PRs cleanly. Sort all dict iterations and omit any wall-clock timestamp from the header (or put it in a separate trailing comment outside byte-equality scope).
- **`uv` not installed yet:** `uv sync` from a fresh `pyproject.toml` is a 30-second install via `pipx install uv` or `winget install astral-sh.uv`. README documents both.
- **`shapely` on Windows:** `pip install shapely` includes a wheel — no GEOS build dependency on Win. Confirmed in T1.
- **Test time budget:** the property-based heading test runs 200 random graphs; on a modern laptop it should complete in <2 s. If it grows to >5 s, switch to `hypothesis` for shrinking + reproducibility.
- **Rollback plan:** PR 1 is purely additive (new files in `tools/`, `docs/`, `WRO2026/routes.h`+`.cpp`). Rollback = revert the merge commit. Sketch continues to compile because Phase 1 doesn't reference `routes.h` yet (PR 2 is what wires it in).
- **PR 2 hand-off contract:** the public surface PR 2 will consume from this PR is the `Routes::` namespace in `routes.h`. PR 2 may need to evolve the schema (e.g. add fields). That's fine — the schema isn't frozen, just the `field_data.py` contract is.
