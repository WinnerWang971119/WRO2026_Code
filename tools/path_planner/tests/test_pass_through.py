"""Pytest suite for ACT_LINE_PASS / pass-through collinear intersection behavior.

Covers TC1–TC12 from the plan at
``docs/plans/2026-05-23-pass-through-collinear-intersections.md``.

All tests exercise the Python sim (``tools.path_planner.sim.runtime``) only.
Hardware verification (HW1–HW3) is deferred to Phase 3.
"""

from __future__ import annotations

import math
import re
from pathlib import Path
from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_TURN_REL,
)
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION
from tools.path_planner.sim import VirtualChassis, PathTrackerSim
from tools.path_planner.sim import runtime as _runtime
from tools.path_planner.sim.runtime import (
    ACT_LINE_OVERSHOOT,
    ACT_LINE_PASS,
    LINE_SENSOR_AXLE_OFFSET_CM,
    NAV_TURN_TOLERANCE_DEG,
    PATH_PASS_THROUGH_ENABLED,
)
from tools.path_planner.tests.fixtures import collinear_4node

# ---------------------------------------------------------------------------
# Tolerance used for chassis position assertions
# ---------------------------------------------------------------------------

_POS_TOL_CM: float = 1e-9

# ---------------------------------------------------------------------------
# Shared graph/field helpers
# ---------------------------------------------------------------------------


def _minimal_field(nodes: dict, start_node: str, start_heading_deg: float) -> SimpleNamespace:
    """Return a ``SimpleNamespace`` field-data stub consumed by ``PathTrackerSim``."""
    return SimpleNamespace(
        NODES=nodes,
        START_NODE=start_node,
        START_HEADING_DEG=start_heading_deg,
    )


def _make_graph(nodes: dict, edges: list) -> nx.DiGraph:
    """Build a ``DiGraph`` from a list of ``(u, v, type, distance_cm, heading_deg)`` tuples.

    Adds a symmetric reverse edge for every forward edge so that
    ``next_hop`` tables can always cover return paths.
    """
    g = nx.DiGraph()
    for name, (x, y) in nodes.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y))
    for u, v, edge_type, dist, hdg in edges:
        g.add_edge(u, v, type=edge_type, distance_cm=float(dist), heading_deg=float(hdg))
        g.add_edge(
            v, u,
            type=edge_type,
            distance_cm=float(dist),
            heading_deg=(float(hdg) + 180.0) % 360.0,
        )
    return g


def _dense_next_hop(nodes: dict, edges_raw: list) -> dict:
    """Build a minimal dense next-hop table for a linear chain of directed edges.

    ``edges_raw`` must form a single directed path from the first entry's
    ``u`` to the last entry's ``v``.  The table is hand-rolled using simple
    successor/predecessor logic so these tests do not depend on the Dijkstra
    planner.  Unreachable entries are ``None``.

    Works for any chain length.
    """
    # Collect ordered node list from the edge sequence.
    ordered: list = []
    for u, v, *_ in edges_raw:
        if not ordered:
            ordered.append(u)
        ordered.append(v)
    # Remove duplicates while preserving order (chain is linear so there
    # are none; the check is defensive).
    seen = set()
    deduped = []
    for n in ordered:
        if n not in seen:
            deduped.append(n)
            seen.add(n)
    ordered = deduped

    fwd = {u: v for u, v, *_ in edges_raw}
    bwd = {v: u for u, v, *_ in edges_raw}

    table: dict = {}
    for src in ordered:
        row: dict = {}
        for tgt in ordered:
            if src == tgt:
                row[tgt] = src
                continue
            src_idx = ordered.index(src)
            tgt_idx = ordered.index(tgt)
            if tgt_idx > src_idx:
                row[tgt] = fwd.get(src)
            else:
                row[tgt] = bwd.get(src)
        table[src] = row
    return table


def _make_sim(nodes: dict, edges_raw: list, start_node: str, start_heading_deg: float):
    """Return a fully wired ``(sim, chassis)`` pair, ready for ``go_to()`` calls."""
    fd = _minimal_field(nodes, start_node, start_heading_deg)
    graph = _make_graph(nodes, edges_raw)
    next_hop = _dense_next_hop(nodes, edges_raw)
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, chassis


# ---------------------------------------------------------------------------
# TC1 — 4-node collinear LINE chain A → D
# ---------------------------------------------------------------------------


def test_tc1_collinear_4node_line_pass_sequence():
    """TC1: A→D on a 4-node collinear chain emits LINE_PASS at B and C, OVERSHOOT at D.

    Expected primitive sequence:
        (ACT_FOLLOW_INT, None)
        (ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM)   ← at B (collinear)
        (ACT_FOLLOW_INT, None)
        (ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM)   ← at C (collinear)
        (ACT_FOLLOW_INT, None)
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM) ← at D (destination)
    """
    fd = collinear_4node.make_field_data()
    graph = collinear_4node.build_graph()
    next_hop = collinear_4node.build_next_hop()
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("D")

    assert prims == [
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM),
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM),
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM),
    ]
    # Localization must end at D.
    assert sim.localization.current_node() == "D"


# ---------------------------------------------------------------------------
# TC2 — 90° turn at B (A→B→C orthogonal, no LINE_PASS)
# ---------------------------------------------------------------------------


def test_tc2_orthogonal_turn_no_line_pass():
    """TC2: A→C via B where B→C is orthogonal (90°) to A→B — no LINE_PASS anywhere.

    Layout:
        A(0,0) --heading 0°--> B(50,0) --heading 90°--> C(50,50)

    Expected sequence from A to C:
        (ACT_FOLLOW_INT, None)
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)  ← B, turn follows
        (ACT_TURN_REL, 90.0)
        (ACT_FOLLOW_INT, None)
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)  ← C (destination)
    """
    nodes = {
        "A": (0.0, 0.0),
        "B": (50.0, 0.0),
        "C": (50.0, 50.0),
    }
    edges_raw = [
        ("A", "B", EDGE_LINE, 50.0, 0.0),
        ("B", "C", EDGE_LINE, 50.0, 90.0),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", 0.0)

    prims = sim.go_to("C")

    act_tags = [tag for tag, _ in prims]
    # Must contain no LINE_PASS.
    assert ACT_LINE_PASS not in act_tags
    # Both intersections emit LINE_OVERSHOOT.
    assert act_tags.count(ACT_LINE_OVERSHOOT) == 2
    # A turn must be emitted between the two FOLLOW_INT hops.
    assert ACT_TURN_REL in act_tags
    assert sim.localization.current_node() == "C"


# ---------------------------------------------------------------------------
# TC3 / TC4 — boundary tolerance (parametrized)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "heading_delta, expected_tag",
    [
        (0.4999, ACT_LINE_PASS),       # TC3: just inside tolerance → LINE_PASS
        (0.5001, ACT_LINE_OVERSHOOT),  # TC4: just outside tolerance → LINE_OVERSHOOT
    ],
    ids=["TC3_inside_tolerance", "TC4_outside_tolerance"],
)
def test_tc3_tc4_heading_tolerance_boundary(heading_delta, expected_tag):
    """TC3/TC4: Heading delta just inside (0.4999°) and just outside (0.5001°) the
    NAV_TURN_TOLERANCE_DEG boundary determines LINE_PASS vs LINE_OVERSHOOT.

    Layout:
        A(0,0) --heading 0°--> B(50,0) --heading delta°--> C

    For the B intersection (not the destination), a collinear next-hop
    (|delta| <= 0.5°) emits LINE_PASS; any larger delta emits LINE_OVERSHOOT.
    """
    c_x = 50.0 + math.cos(math.radians(heading_delta)) * 50.0
    c_y = 0.0 + math.sin(math.radians(heading_delta)) * 50.0
    nodes = {
        "A": (0.0, 0.0),
        "B": (50.0, 0.0),
        "C": (c_x, c_y),
    }
    edges_raw = [
        ("A", "B", EDGE_LINE, 50.0, 0.0),
        ("B", "C", EDGE_LINE, 50.0, heading_delta),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", 0.0)

    prims = sim.go_to("C")

    # Extract the tag immediately after the first FOLLOW_INT (the B-intersection step).
    assert (ACT_FOLLOW_INT, None) in prims
    first_follow_idx = prims.index((ACT_FOLLOW_INT, None))
    overshoot_prim = prims[first_follow_idx + 1]
    assert overshoot_prim == (expected_tag, LINE_SENSOR_AXLE_OFFSET_CM)
    assert sim.localization.current_node() == "C"


# ---------------------------------------------------------------------------
# TC5 — collinear LINE → PROJECTION at B
# ---------------------------------------------------------------------------


def test_tc5_collinear_line_to_projection():
    """TC5: Collinear LINE→PROJECTION at B emits LINE_PASS then FOLLOW_DIST (no turn).

    Layout:
        A(0,0) --LINE heading 0°--> B(50,0) --PROJECTION heading 0°--> C(100,0)

    Expected sequence from A to C:
        (ACT_FOLLOW_INT, None)
        (ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM)  ← B (collinear into PROJECTION)
        (ACT_FOLLOW_DIST, 50.0)
    """
    nodes = {
        "A": (0.0, 0.0),
        "B": (50.0, 0.0),
        "C": (100.0, 0.0),
    }
    edges_raw = [
        ("A", "B", EDGE_LINE, 50.0, 0.0),
        ("B", "C", EDGE_PROJECTION, 50.0, 0.0),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", 0.0)

    prims = sim.go_to("C")

    assert prims == [
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM),
        (ACT_FOLLOW_DIST, 50.0),
    ]
    # No turn between LINE_PASS and FOLLOW_DIST.
    act_tags = [tag for tag, _ in prims]
    assert ACT_TURN_REL not in act_tags
    assert sim.localization.current_node() == "C"


# ---------------------------------------------------------------------------
# TC6 — collinear LINE → BLIND at B
# ---------------------------------------------------------------------------


def test_tc6_collinear_line_to_blind():
    """TC6: Collinear LINE→BLIND at B emits LINE_PASS then FORWARD (no turn).

    Layout:
        A(0,0) --LINE heading 0°--> B(50,0) --BLIND heading 0°--> C(100,0)

    Expected sequence from A to C:
        (ACT_FOLLOW_INT, None)
        (ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM)  ← B (collinear into BLIND)
        (ACT_FORWARD, 50.0)
    """
    nodes = {
        "A": (0.0, 0.0),
        "B": (50.0, 0.0),
        "C": (100.0, 0.0),
    }
    edges_raw = [
        ("A", "B", EDGE_LINE, 50.0, 0.0),
        ("B", "C", EDGE_BLIND, 50.0, 0.0),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", 0.0)

    prims = sim.go_to("C")

    assert prims == [
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM),
        (ACT_FORWARD, 50.0),
    ]
    act_tags = [tag for tag, _ in prims]
    assert ACT_TURN_REL not in act_tags
    assert sim.localization.current_node() == "C"


# ---------------------------------------------------------------------------
# TC7 — goal-terminal: penultimate hop is the destination → LINE_OVERSHOOT
# ---------------------------------------------------------------------------


def test_tc7_goal_terminal_never_line_pass():
    """TC7: When _nextHop == goal (the destination node), LINE_OVERSHOOT fires — never LINE_PASS.

    A→B is the only hop; B is both the penultimate intersection AND the goal.
    Even though a collinear edge B→C exists beyond it, the destination guard
    prevents LINE_PASS from firing at B.

    The collinear B→C edge is added to the graph so that the peek code would
    naively return LINE_PASS if the destination guard were absent.
    """
    nodes = {
        "A": (0.0, 0.0),
        "B": (50.0, 0.0),
        "C": (100.0, 0.0),
    }
    edges_raw = [
        ("A", "B", EDGE_LINE, 50.0, 0.0),
        ("B", "C", EDGE_LINE, 50.0, 0.0),  # collinear — but B is our goal
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", 0.0)

    # Goal is B — the first (and only) intersection is the destination.
    prims = sim.go_to("B")

    assert prims == [
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM),
    ]
    assert sim.localization.current_node() == "B"


# ---------------------------------------------------------------------------
# TC8 — PATH_PASS_THROUGH_ENABLED = False disables all pass-through
# ---------------------------------------------------------------------------


def test_tc8_pass_through_disabled_every_edge_is_overshoot(monkeypatch):
    """TC8: With PATH_PASS_THROUGH_ENABLED monkeypatched to False, TC1's A→D chain
    emits LINE_OVERSHOOT at every intermediate intersection instead of LINE_PASS.
    """
    monkeypatch.setattr(_runtime, "PATH_PASS_THROUGH_ENABLED", False)

    fd = collinear_4node.make_field_data()
    graph = collinear_4node.build_graph()
    next_hop = collinear_4node.build_next_hop()
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("D")

    act_tags = [tag for tag, _ in prims]
    # No LINE_PASS anywhere.
    assert ACT_LINE_PASS not in act_tags
    # Three LINE_OVERSHOOT steps (B, C, D — all intersections).
    assert act_tags.count(ACT_LINE_OVERSHOOT) == 3
    # Three FOLLOW_INT steps.
    assert act_tags.count(ACT_FOLLOW_INT) == 3
    assert sim.localization.current_node() == "D"


# ---------------------------------------------------------------------------
# TC9 — parity: runtime.py constants match Config.h via regex
# ---------------------------------------------------------------------------

# Regex patterns for extracting the two constants from Config.h.
# Handles trailing 'f' suffix on float literals and 'true'/'false' booleans.
_FLOAT_RE = re.compile(
    r"constexpr\s+float\s+NAV_TURN_TOLERANCE_DEG\s*=\s*([0-9]*\.?[0-9]+)f?;",
    re.MULTILINE,
)
_BOOL_RE = re.compile(
    # T10 wrapped the default in an #ifndef/#define block so the value can be
    # overridden via `-DPATH_PASS_THROUGH_ENABLED_DEFAULT=...` at compile time.
    # The `#define PATH_PASS_THROUGH_ENABLED_DEFAULT <bool>` is the source of
    # truth for the default; the `constexpr bool PATH_PASS_THROUGH_ENABLED = ...;`
    # line just propagates that to the public symbol.
    r"#define\s+PATH_PASS_THROUGH_ENABLED_DEFAULT\s+(true|false)",
    re.MULTILINE,
)

# Path to Config.h relative to the repo root — resolved absolutely so the
# test is runnable from any working directory.
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent
_CONFIG_H = _REPO_ROOT / "WRO2026" / "Config.h"


def test_tc9_nav_turn_tolerance_parity():
    """TC9a: NAV_TURN_TOLERANCE_DEG in runtime.py matches Config.h via regex grep."""
    assert _CONFIG_H.exists(), (
        f"Config.h not found at {_CONFIG_H}. "
        "Ensure the test is run from within the WRO2026_Code repo."
    )
    text = _CONFIG_H.read_text(encoding="utf-8")
    match = _FLOAT_RE.search(text)
    assert match is not None, (
        "Could not find 'constexpr float NAV_TURN_TOLERANCE_DEG = ...;' in Config.h"
    )
    config_value = float(match.group(1))
    assert _runtime.NAV_TURN_TOLERANCE_DEG == pytest.approx(config_value, abs=1e-9), (
        f"runtime.NAV_TURN_TOLERANCE_DEG ({_runtime.NAV_TURN_TOLERANCE_DEG}) "
        f"!= Config.h ({config_value})"
    )


def test_tc9_path_pass_through_enabled_parity():
    """TC9b: PATH_PASS_THROUGH_ENABLED in runtime.py matches Config.h via regex grep."""
    assert _CONFIG_H.exists(), (
        f"Config.h not found at {_CONFIG_H}."
    )
    text = _CONFIG_H.read_text(encoding="utf-8")
    match = _BOOL_RE.search(text)
    assert match is not None, (
        "Could not find '#define PATH_PASS_THROUGH_ENABLED_DEFAULT <bool>' in Config.h "
        "(the #ifndef-wrapped default introduced by T10's host-compile harness)"
    )
    config_raw = match.group(1)          # "true" or "false"
    config_value: bool = config_raw == "true"
    assert _runtime.PATH_PASS_THROUGH_ENABLED == config_value, (
        f"runtime.PATH_PASS_THROUGH_ENABLED ({_runtime.PATH_PASS_THROUGH_ENABLED}) "
        f"!= Config.h ({config_value})"
    )


# ---------------------------------------------------------------------------
# TC14 — parity: BIDIRECTIONAL_ENABLED / BIDIRECTIONAL_TURN_THRESHOLD_DEG
# ---------------------------------------------------------------------------


def test_tc14a_bidirectional_enabled_config_parity():
    """TC14a: Config.h defines BIDIRECTIONAL_ENABLED_DEFAULT macro guard
    and the BIDIRECTIONAL_ENABLED constexpr. Sim's hand-mirrored value
    matches the C++ default."""
    assert _CONFIG_H.exists(), (
        f"Config.h not found at {_CONFIG_H}. "
        "Ensure the test is run from within the WRO2026_Code repo."
    )
    config_h = _CONFIG_H.read_text(encoding="utf-8")
    # Anchored regex defends against false positives from unrelated bools.
    assert re.search(
        r"#define\s+BIDIRECTIONAL_ENABLED_DEFAULT\s+true",
        config_h,
    ), "Config.h missing #define BIDIRECTIONAL_ENABLED_DEFAULT true"
    assert re.search(
        r"constexpr\s+bool\s+BIDIRECTIONAL_ENABLED\s*=",
        config_h,
    ), "Config.h missing constexpr bool BIDIRECTIONAL_ENABLED = ..."
    # Sim parity: runtime.BIDIRECTIONAL_ENABLED must equal C++ default (True).
    assert _runtime.BIDIRECTIONAL_ENABLED is True, (
        "Sim BIDIRECTIONAL_ENABLED diverged from C++ default (true)"
    )


def test_tc14b_bidirectional_turn_threshold_config_parity():
    """TC14b: Config.h defines BIDIRECTIONAL_TURN_THRESHOLD_DEG as
    constexpr float = 90.0f. Sim hand-mirrored value matches."""
    assert _CONFIG_H.exists(), (
        f"Config.h not found at {_CONFIG_H}. "
        "Ensure the test is run from within the WRO2026_Code repo."
    )
    config_h = _CONFIG_H.read_text(encoding="utf-8")
    assert re.search(
        r"constexpr\s+float\s+BIDIRECTIONAL_TURN_THRESHOLD_DEG\s*=\s*90\.0f",
        config_h,
    ), (
        "Config.h missing constexpr float BIDIRECTIONAL_TURN_THRESHOLD_DEG "
        "= 90.0f (anchored pattern)"
    )
    assert _runtime.BIDIRECTIONAL_TURN_THRESHOLD_DEG == 90.0, (
        "Sim BIDIRECTIONAL_TURN_THRESHOLD_DEG diverged from C++ default (90.0f)"
    )


# ---------------------------------------------------------------------------
# TC10 — heading wrap: 359.5° → 0.0° normalizes to -0.5° → LINE_PASS
# ---------------------------------------------------------------------------


def test_tc10_heading_wrap_line_pass():
    """TC10: Edge A→B at 359.5° then B→C at 0.0° — normalized delta = 0.5° → LINE_PASS.

    The wrap normalization (via normalize_turn) folds the 359.5→0.0 transition
    to a -0.5° delta, which is within NAV_TURN_TOLERANCE_DEG (0.5°), so
    LINE_PASS must fire at B.
    """
    hdg_ab = 359.5
    hdg_bc = 0.0

    # Place nodes along the A→B direction so geometry matches the heading.
    ax, ay = 0.0, 0.0
    bx = ax + math.cos(math.radians(hdg_ab)) * 50.0
    by = ay + math.sin(math.radians(hdg_ab)) * 50.0
    cx = bx + math.cos(math.radians(hdg_bc)) * 50.0
    cy = by + math.sin(math.radians(hdg_bc)) * 50.0

    nodes = {"A": (ax, ay), "B": (bx, by), "C": (cx, cy)}
    edges_raw = [
        ("A", "B", EDGE_LINE, 50.0, hdg_ab),
        ("B", "C", EDGE_LINE, 50.0, hdg_bc),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", hdg_ab)

    prims = sim.go_to("C")

    # B is not the destination and the heading delta wraps to 0.5° → LINE_PASS.
    first_follow_idx = prims.index((ACT_FOLLOW_INT, None))
    b_overshoot = prims[first_follow_idx + 1]
    assert b_overshoot == (ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM), (
        f"Expected LINE_PASS at B (wrapped delta), got {b_overshoot}"
    )
    assert sim.localization.current_node() == "C"


# ---------------------------------------------------------------------------
# TC11 — position invariant: LINE_PASS chassis advance == LINE_OVERSHOOT
# ---------------------------------------------------------------------------


def test_tc11_position_invariant_line_pass_equals_overshoot():
    """TC11: Chassis x/y after LINE_PASS at node B == chassis x/y after LINE_OVERSHOOT
    for the same node geometry.

    Both code paths call ``chassis.forward(LINE_SENSOR_AXLE_OFFSET_CM)`` after
    the intersection, so the absolute advance must be identical.  We verify this
    by using ``step_through`` on two separate sims and capturing the chassis
    snapshot immediately after the B-intersection primitive:
      - Collinear A→B→C→D (LINE_PASS at B, inspected via step_through).
      - 90°-turn A→B (LINE_OVERSHOOT at B; B is the goal).
    Both chassis must be at (50 + LINE_SENSOR_AXLE_OFFSET_CM, 0) after the
    B-intersection step.
    """
    # Common geometry: A at origin, B at 50cm along +x.
    a_pos = (0.0, 0.0)
    b_pos = (50.0, 0.0)

    # --- Collinear path: use step_through on A→B→C→D and capture snapshot
    #     after the (ACT_LINE_PASS, ...) step at B. ---
    fd_col = collinear_4node.make_field_data()
    graph_col = collinear_4node.build_graph()
    next_hop_col = collinear_4node.build_next_hop()
    chassis_col = VirtualChassis.from_field_data(fd_col)
    sim_col = PathTrackerSim(fd_col, graph_col, next_hop_col, chassis_col)
    sim_col.localization.zero_heading_from_current()

    # Walk step-by-step to "D" and grab the snapshot right after the first
    # LINE_PASS step (which is the B-intersection overshoot advance).
    pass_snapshot = None
    for (tag, payload), snap in sim_col.step_through("D"):
        if tag == ACT_LINE_PASS and pass_snapshot is None:
            pass_snapshot = snap
            break
    assert pass_snapshot is not None, "Expected ACT_LINE_PASS step not found in step_through"
    pass_x, pass_y = pass_snapshot.x_cm, pass_snapshot.y_cm

    # --- Non-collinear path: A→B where B is the destination.
    #     B lies along heading 0° and is not collinear (no further edge), so
    #     LINE_OVERSHOOT is emitted. ---
    nodes_turn = {"A": a_pos, "B": b_pos}
    edges_turn = [
        ("A", "B", EDGE_LINE, 50.0, 0.0),
    ]
    sim_turn, chassis_turn = _make_sim(nodes_turn, edges_turn, "A", 0.0)
    prims_turn = sim_turn.go_to("B")
    assert (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM) in prims_turn
    overshoot_x, overshoot_y = chassis_turn.x_cm, chassis_turn.y_cm

    # After the B-intersection both chassis started at A=(0,0), followed
    # A→B (50cm along +x), then advanced LINE_SENSOR_AXLE_OFFSET_CM further
    # along heading 0°.  Expected: (50 + LINE_SENSOR_AXLE_OFFSET_CM, 0).
    expected_x = b_pos[0] + LINE_SENSOR_AXLE_OFFSET_CM
    expected_y = b_pos[1]

    assert pass_x == pytest.approx(expected_x, abs=_POS_TOL_CM)
    assert pass_y == pytest.approx(expected_y, abs=_POS_TOL_CM)
    assert overshoot_x == pytest.approx(expected_x, abs=_POS_TOL_CM)
    assert overshoot_y == pytest.approx(expected_y, abs=_POS_TOL_CM)


# ---------------------------------------------------------------------------
# TC12 — PROJECTION-inbound: pass-through NOT applied, legacy sequence emitted
# ---------------------------------------------------------------------------


def test_tc12_projection_inbound_no_line_pass():
    """TC12: A path whose intersection at B is reached via a PROJECTION edge (not LINE)
    emits the legacy sequence — no LINE_PASS — because pass-through is only checked
    in the EDGE_LINE branch of go_to.

    Layout:
        A(0,0) --PROJECTION heading 0°--> B(50,0) --LINE heading 0°--> C(100,0)

    Expected sequence from A to C:
        (ACT_FOLLOW_DIST, 50.0)   ← A→B via PROJECTION (no FOLLOW_INT, no overshoot)
        (ACT_FOLLOW_INT, None)    ← B→C via LINE
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM) ← C is destination
    """
    nodes = {
        "A": (0.0, 0.0),
        "B": (50.0, 0.0),
        "C": (100.0, 0.0),
    }
    edges_raw = [
        ("A", "B", EDGE_PROJECTION, 50.0, 0.0),
        ("B", "C", EDGE_LINE, 50.0, 0.0),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", 0.0)

    prims = sim.go_to("C")

    act_tags = [tag for tag, _ in prims]
    # No LINE_PASS anywhere — inbound to B was PROJECTION, not LINE.
    assert ACT_LINE_PASS not in act_tags
    # The PROJECTION leg emits FOLLOW_DIST, not FOLLOW_INT + overshoot.
    assert prims[0] == (ACT_FOLLOW_DIST, 50.0)
    # The LINE leg B→C emits FOLLOW_INT + LINE_OVERSHOOT (C is the goal).
    assert ACT_FOLLOW_INT in act_tags
    assert ACT_LINE_OVERSHOOT in act_tags
    assert sim.localization.current_node() == "C"
