"""TC12 / TC13 — turn-gate parity between the sim, the compiler, and Config (#147).

#147 aligns the sim's pre-edge turn gate (``PathTrackerSim._emit_turn_to``) and the
compiler's turn-emission gates (``compile_path`` / ``final_heading_after``) to
:data:`tools.path_planner.sim.runtime.NAV_TURN_TOLERANCE_DEG` (0.5°) -- the same
tolerance the firmware dispatches a turn at -- so a real field's tiny edge-heading
residuals never surface as spurious ``ACT_TURN_REL`` micro-turns. It also applies ONE
logical-heading rule across all three surfaces: even when the sub-tolerance turn is
SKIPPED, the logical heading still advances onto the edge, so a chain of individually
sub-tolerance hops can never accumulate into a compiler-only turn on a later hop.

TC12 (parametric parity) pins every ``config_parse``-readable mirrored ``runtime.py``
constant to its firmware-header value. ``sim/runtime.py`` hand-mirrors the Config
constants (it cannot import the C++ headers), so this is the guard that the silent
mirror surface cannot drift from the value the robot drives with.

TC13 (near-collinear turn suppression) drives BOTH the sim and the compiler over a
single near-collinear hop AND a multi-hop chain whose per-hop delta is sub-tolerance
but whose cumulative swing exceeds the tolerance, asserting neither surface emits a
spurious ``ACT_TURN_REL`` and both advance the logical heading identically.
"""

from __future__ import annotations

import math
import re
import warnings
from pathlib import Path
from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner import config_parse
from tools.path_planner.compiler import (
    ACT_TURN_REL,
    compile_path,
    final_heading_after,
    normalize_turn,
)
from tools.path_planner.graph_builder import EDGE_LINE, build_graph
from tools.path_planner.planner import BlindDecayParams, compute_all_pairs_next_hop
from tools.path_planner.sim import runtime as _runtime
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.parity import walk_next_hop
from tools.path_planner.sim.runtime import NAV_TURN_TOLERANCE_DEG, PathTrackerSim
from tools.path_planner.tests.fixtures import collinear_4node

# ---------------------------------------------------------------------------
# Firmware header paths (absolute, cwd-independent -- mirror test_pass_through.py).
# ---------------------------------------------------------------------------
_CONFIG_DIR = Path(__file__).resolve().parents[3] / "WRO2026" / "config"
_DRIVE_H = _CONFIG_DIR / "ConfigDrive.h"
_NAV_H = _CONFIG_DIR / "ConfigNav.h"
_LINE_H = _CONFIG_DIR / "ConfigLine.h"


# ===========================================================================
# TC12 — parametric parity: every config_parse-readable mirrored runtime.py
#        constant equals its firmware-header value.
# ===========================================================================
#
# Only the constants ``config_parse`` can structurally read are listed. Two mirrored
# floats -- ``LINE_HEADING_RESET_TOLERANCE_DEG`` and ``PATH_BLIND_CHAIN_MAX_CM`` --
# use the ``#ifndef X_DEFAULT / #define X_DEFAULT <literal> / constexpr float X =
# X_DEFAULT`` override idiom, whose ``#define``-backed float RHS
# :func:`config_parse.parse_config_float` deliberately does not resolve (it only
# reads a numeric-literal RHS). Each keeps its own dedicated ``#define``-anchored
# regex parity test instead: ``PATH_BLIND_CHAIN_MAX_CM`` by
# ``test_pass_through.py::test_adr059_path_blind_chain_max_cm_parity`` and
# ``LINE_HEADING_RESET_TOLERANCE_DEG`` by
# :func:`test_line_heading_reset_tolerance_deg_parity` below -- so the parametric
# table here covers only the ``config_parse``-readable subset by design.

# (runtime.py name, header the declaration lives in)
_MIRRORED_FLOAT_CONSTANTS = [
    ("LINE_SENSOR_AXLE_OFFSET_CM", _DRIVE_H),
    ("LINE_SENSOR_AXLE_OFFSET_REAR_CM", _DRIVE_H),
    ("PIVOT_TRIM_MIN_DRIVE_CM", _DRIVE_H),
    ("NAV_TURN_TOLERANCE_DEG", _NAV_H),
    ("TSHAPE_MIN_MOVE_CM", _DRIVE_H),
    ("BIDIRECTIONAL_TURN_THRESHOLD_DEG", _NAV_H),
    ("OFFSET_TRANSLATION_DEADBAND_CM", _NAV_H),
    ("OFFSET_ROTATION_DEADBAND_DEG", _NAV_H),
    ("RETURN_TRIGGER_THRESHOLD_CM", _NAV_H),
    ("RETURN_DISTANCE_CAP_CM", _NAV_H),
    ("WALL_STOP_BACK_OFF_CM", _DRIVE_H),
    ("WALL_CONTACT_OFFSET_CM", _NAV_H),
]

# (runtime.py name, header the declaration lives in, symbolic-RHS search headers).
# ``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE`` resolves through the default search set
# (PWM_MIN_MAGNITUDE lives in ConfigGlobal.h); ``MAX_CHAIN_HOPS = MAX_CHAIN_HOPS_DEFAULT``
# resolves against a ConfigNav.h ``#define``, so it needs ConfigNav.h in its search set.
_MIRRORED_INT_CONSTANTS = [
    ("SPEED_NORMAL", _DRIVE_H, config_parse.DEFAULT_INT_SEARCH_HEADERS),
    ("SPEED_SLOW", _DRIVE_H, config_parse.DEFAULT_INT_SEARCH_HEADERS),
    ("SPEED_FAST", _DRIVE_H, config_parse.DEFAULT_INT_SEARCH_HEADERS),
    ("TSHAPE_START_SPEED", _DRIVE_H, config_parse.DEFAULT_INT_SEARCH_HEADERS),
    ("LINE_BOARD_COUNT", _LINE_H, config_parse.DEFAULT_INT_SEARCH_HEADERS),
    ("MAX_CHAIN_HOPS", _NAV_H, (_NAV_H,)),
]


@pytest.mark.parametrize(
    "name, header",
    _MIRRORED_FLOAT_CONSTANTS,
    ids=[c[0] for c in _MIRRORED_FLOAT_CONSTANTS],
)
def test_tc12_mirrored_float_constant_matches_header(name, header):
    """Each mirrored ``constexpr float`` in runtime.py equals its header value."""
    assert header.exists(), (
        f"{header} not found; run the suite from within the WRO2026_Code repo."
    )
    header_value = config_parse.parse_config_float(name, header)
    runtime_value = getattr(_runtime, name)
    assert runtime_value == pytest.approx(header_value, abs=1e-9), (
        f"runtime.{name} ({runtime_value}) drifted from {header.name} "
        f"({header_value}). sim/runtime.py hand-mirrors the firmware Config "
        f"constants -- update the mirror when the header changes (#147)."
    )


@pytest.mark.parametrize(
    "name, header, search_headers",
    _MIRRORED_INT_CONSTANTS,
    ids=[c[0] for c in _MIRRORED_INT_CONSTANTS],
)
def test_tc12_mirrored_int_constant_matches_header(name, header, search_headers):
    """Each mirrored ``constexpr int/uint`` in runtime.py equals its header value."""
    assert header.exists(), (
        f"{header} not found; run the suite from within the WRO2026_Code repo."
    )
    header_value = config_parse.parse_config_int(
        name, header, search_headers=search_headers
    )
    runtime_value = getattr(_runtime, name)
    assert runtime_value == header_value, (
        f"runtime.{name} ({runtime_value}) drifted from {header.name} "
        f"({header_value}). sim/runtime.py hand-mirrors the firmware Config "
        f"constants -- update the mirror when the header changes (#147)."
    )


# ===========================================================================
# Dedicated regex parity for LINE_HEADING_RESET_TOLERANCE_DEG (#147 AC).
# ===========================================================================
#
# ``LINE_HEADING_RESET_TOLERANCE_DEG`` uses the ``#ifndef ..._DEFAULT`` override
# idiom (a ``#define``-backed float RHS that :func:`config_parse.parse_config_float`
# deliberately cannot resolve), so it is excluded from the TC12 parametric table
# above. Its ``runtime.py`` mirror is still exported in ``__all__`` and consumed by
# the sim, so #147's "pin every mirrored runtime.py constant" AC needs a dedicated
# guard here -- mirroring ``test_pass_through.py`` ADR-059's ``#define``-anchored
# regex for ``PATH_BLIND_CHAIN_MAX_CM``.

_LINE_HEADING_RESET_TOLERANCE_DEG_RE = re.compile(
    r"#define\s+LINE_HEADING_RESET_TOLERANCE_DEG_DEFAULT\s+([0-9]*\.?[0-9]+)f?",
    re.MULTILINE,
)


def test_line_heading_reset_tolerance_deg_parity():
    """The mirrored ``LINE_HEADING_RESET_TOLERANCE_DEG`` equals its ConfigNav.h value.

    Parsed via regex (not the TC12 ``config_parse`` table) because the constant uses
    the ``#ifndef ..._DEFAULT`` idiom :func:`config_parse.parse_config_float` cannot
    resolve. Same ``#define``-anchored approach as
    ``test_pass_through.py::test_adr059_path_blind_chain_max_cm_parity``.
    """
    assert _NAV_H.exists(), (
        f"{_NAV_H} not found; run the suite from within the WRO2026_Code repo."
    )
    text = _NAV_H.read_text(encoding="utf-8")
    match = _LINE_HEADING_RESET_TOLERANCE_DEG_RE.search(text)
    assert match is not None, (
        "Could not find '#define LINE_HEADING_RESET_TOLERANCE_DEG_DEFAULT <float>' "
        f"in {_NAV_H.name}."
    )
    header_value = float(match.group(1))
    runtime_value = _runtime.LINE_HEADING_RESET_TOLERANCE_DEG
    assert runtime_value == pytest.approx(header_value, abs=1e-9), (
        f"runtime.LINE_HEADING_RESET_TOLERANCE_DEG ({runtime_value}) drifted from "
        f"{_NAV_H.name} ({header_value}). sim/runtime.py hand-mirrors the firmware "
        f"Config constants -- update the mirror when the header changes (#147)."
    )


# ===========================================================================
# TC13 — near-collinear turn suppression across the sim + the compiler.
# ===========================================================================


def _build_line_chain(headings):
    """Hand-roll a straight N-edge LINE chain A, B, C, ... at the given headings.

    Node positions follow the per-edge headings so the geometry is self-consistent,
    though the sim / compiler read the heading off the edge attribute (not the node
    positions). Every node is a ``line`` node, so no projection / blind branch is
    exercised -- the hops are pure line-follows and the only thing under test is the
    per-hop turn decision.

    :param headings: per-edge world headings in degrees (edge ``i`` connects node
        ``i`` to node ``i + 1``).
    :return: ``(graph, next_hop, field_data, path)``.
    """
    names = [chr(ord("A") + i) for i in range(len(headings) + 1)]
    pos = {names[0]: (0.0, 0.0)}
    x = y = 0.0
    for i, h in enumerate(headings):
        rad = math.radians(h)
        x += 50.0 * math.cos(rad)
        y += 50.0 * math.sin(rad)
        pos[names[i + 1]] = (x, y)

    graph = nx.DiGraph()
    for name, (px, py) in pos.items():
        graph.add_node(name, x_cm=px, y_cm=py, type="line")
    for i, h in enumerate(headings):
        u, v = names[i], names[i + 1]
        graph.add_edge(u, v, type=EDGE_LINE, distance_cm=50.0, heading_deg=float(h))
        graph.add_edge(
            v, u, type=EDGE_LINE, distance_cm=50.0, heading_deg=(float(h) + 180.0) % 360.0
        )

    path = list(names)
    next_hop = {}
    for i, src in enumerate(path):
        row = {}
        for j, tgt in enumerate(path):
            if src == tgt:
                row[tgt] = src
            elif j > i:
                row[tgt] = path[i + 1]
            else:
                row[tgt] = path[i - 1]
        next_hop[src] = row

    field_data = SimpleNamespace(
        NODES=pos,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        BLIND_PROJECTION_AUTO_SEEK=True,
        PIVOT_TURNS=[],
    )
    return graph, next_hop, field_data, path


def _drive_sim(graph, next_hop, field_data, goal):
    """Zero the sim's heading and drive it to @p goal, returning ``(sim, primitives)``."""
    chassis = VirtualChassis.from_field_data(field_data)
    sim = PathTrackerSim(field_data, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        emitted = sim.go_to(goal)
    return sim, emitted


def _turns(primitives):
    """Return the ``ACT_TURN_REL`` primitives in @p primitives."""
    return [p for p in primitives if p[0] == ACT_TURN_REL]


def test_tc13_single_near_collinear_hop_emits_no_turn():
    """A hop whose heading is sub-tolerance off the start emits no ``ACT_TURN_REL``.

    ``delta`` sits in ``(1e-6, NAV_TURN_TOLERANCE_DEG]`` -- above the old 1e-6 gate
    (which would have emitted a micro-turn) but at/below the firmware tolerance, so
    neither surface emits a turn. Both still advance the logical heading onto the edge.
    """
    delta_deg = 0.4
    assert 1e-6 < delta_deg <= NAV_TURN_TOLERANCE_DEG
    graph, next_hop, field_data, path = _build_line_chain([delta_deg])

    compiled = compile_path(path, field_data.START_HEADING_DEG, graph, field_data)
    assert _turns(compiled) == [], "compiler emitted a spurious sub-tolerance turn"
    assert final_heading_after(
        path, field_data.START_HEADING_DEG, graph
    ) == pytest.approx(delta_deg, abs=1e-9)

    sim, emitted = _drive_sim(graph, next_hop, field_data, "B")
    assert _turns(emitted) == [], "sim emitted a spurious sub-tolerance turn"
    assert sim.chassis.heading_deg == pytest.approx(delta_deg, abs=1e-9)


def test_tc13_multi_hop_cumulative_no_compiler_only_turn():
    """Sub-tolerance-per-hop chain whose cumulative swing exceeds the tolerance.

    The exact #147 trap: each per-hop delta is <= tolerance, but the running total
    crosses it. The old "advance only when a turn is emitted" rule would leave the
    logical heading stuck at the start, so once the cumulative reached the tolerance a
    spurious compiler-only ``ACT_TURN_REL`` would appear. With the one logical-heading
    rule the heading advances onto every edge, so no turn is ever emitted.
    """
    field_start = 0.0
    headings = [0.0, 0.3, 0.6, 0.9, 1.2]
    prior = [field_start] + headings[:-1]
    # Setup invariants: every per-hop step is sub-tolerance, cumulative is not.
    for before, after in zip(prior, headings):
        assert abs(normalize_turn(after - before)) <= NAV_TURN_TOLERANCE_DEG
    assert abs(normalize_turn(headings[-1] - field_start)) > NAV_TURN_TOLERANCE_DEG

    graph, next_hop, field_data, path = _build_line_chain(headings)

    compiled = compile_path(path, field_data.START_HEADING_DEG, graph, field_data)
    assert _turns(compiled) == [], (
        "compiler accumulated sub-tolerance hops into a spurious compiler-only turn "
        "(the #147 bug: the logical heading was not advanced on the skipped hops)."
    )
    final_compiler = final_heading_after(path, field_data.START_HEADING_DEG, graph)
    assert final_compiler == pytest.approx(headings[-1], abs=1e-9)

    sim, emitted = _drive_sim(graph, next_hop, field_data, path[-1])
    assert _turns(emitted) == [], "sim accumulated sub-tolerance hops into a turn"
    # Both surfaces advanced the logical heading identically onto the final edge.
    assert sim.chassis.heading_deg == pytest.approx(final_compiler, abs=1e-9)
    assert sim.chassis.heading_deg == pytest.approx(headings[-1], abs=1e-9)


def test_tc13_skewed_collinear_fixture_emits_no_turn():
    """Reuse the near-collinear skewed CTPS fixture through the real graph builder.

    ``collinear_4node.make_ctps_field_data_skewed`` nudges node ``C`` off the axis, so
    :func:`build_graph` derives sub-tolerance alignment turns at ``B`` / ``C`` / ``D``
    exactly the way the real ``field_data.json`` geometry does. Neither surface may
    emit a turn, and their advanced headings must agree.
    """
    field_data = collinear_4node.make_ctps_field_data_skewed()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        graph = build_graph(field_data)
        next_hop = compute_all_pairs_next_hop(
            graph, BlindDecayParams.from_field_data(field_data)
        )

    walked = walk_next_hop(next_hop, field_data.START_NODE, "D")
    compiled = compile_path(walked, field_data.START_HEADING_DEG, graph, field_data)
    assert _turns(compiled) == [], "compiler emitted a turn on the skewed collinear chain"

    sim, emitted = _drive_sim(graph, next_hop, field_data, "D")
    assert _turns(emitted) == [], "sim emitted a turn on the skewed collinear chain"
    assert sim.chassis.heading_deg == pytest.approx(
        final_heading_after(walked, field_data.START_HEADING_DEG, graph), abs=1e-9
    )


@pytest.mark.parametrize(
    "turn_deg, expect_emit",
    [
        (90.0, True),                     # a real turn still emits
        (0.6, True),                      # just above tolerance -> emits
        (NAV_TURN_TOLERANCE_DEG, False),  # exactly 0.5 -> skipped (gate is strictly >)
        (0.4, False),                     # sub-tolerance -> skipped
    ],
    ids=["90deg", "just-above", "exactly-tol", "sub-tol"],
)
def test_tc13_turn_emission_boundary(turn_deg, expect_emit):
    """The sim and the compiler agree on the emission boundary AND always advance heading.

    Guards against over-suppression: a genuine turn (> NAV_TURN_TOLERANCE_DEG) still
    emits on both surfaces; the boundary value itself is skipped by both. Regardless of
    emission, the logical heading advances onto the edge (the one logical-heading rule).
    """
    graph, next_hop, field_data, path = _build_line_chain([turn_deg])

    compiled = compile_path(path, field_data.START_HEADING_DEG, graph, field_data)
    assert bool(_turns(compiled)) is expect_emit
    assert final_heading_after(
        path, field_data.START_HEADING_DEG, graph
    ) == pytest.approx(turn_deg, abs=1e-9)

    sim, emitted = _drive_sim(graph, next_hop, field_data, "B")
    assert bool(_turns(emitted)) is expect_emit
    assert sim.chassis.heading_deg == pytest.approx(turn_deg, abs=1e-9)
