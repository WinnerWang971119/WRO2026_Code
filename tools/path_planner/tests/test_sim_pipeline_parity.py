"""TC10 / TC10b — sim <-> compiler parity under a field-sourced PIPELINE_CONFIG.

R4 (PR #98 triage). ``PipelineConfig.from_field_data(field_data)`` reached the
firmware only through ``header_writer``; the offline resolvers
``resolve_line_mode`` / ``resolve_blind_mode`` read the module-global
``PIPELINE_CONFIG`` singleton directly, so once an operator set a field-sourced
``PIPELINE_LINE_MODE`` / ``PIPELINE_BLIND_MODE`` the emitted ``routes.cpp``
reflected it but the offline sim / compiler did NOT — a silent sim↔robot
divergence. T4 threads a resolved ``cfg`` from ``compile_path`` /
``PathTrackerSim._walk_hops`` into every resolver call site.

These tests are deliberately built on an OWN, isolated synthetic fixture (the
toy golden is owned by another task and must not be touched). The fixture
exercises:

* a REAR ``EDGE_LINE`` hop with NO ``primitive_mode`` override, so the emitted
  ``ACT_LINE_REVERSE`` carries the resolved ``cfg.line`` — observable ONLY via
  ``resolve_line_mode``; a missed call site there leaves the singleton default;
* a REAR ``EDGE_BLIND`` hop with NO ``primitive_mode`` override, so the emitted
  ``ACT_BLIND_REVERSE`` carries the resolved ``cfg.blind`` — observable ONLY via
  ``resolve_blind_mode``.

REAR hops are required because the resolved mode only appears in the emitted
primitive payload on REAR primitives (FRONT hops fold the mode into
``resolve_stop_at_line`` without surfacing it). The hops are forced REAR with a
per-edge ``direction_override = FORCE_REVERSE`` (step-1 binding in
``resolve_direction``).
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import List

import networkx as nx

from tools.path_planner.compiler import (
    ACT_TURN_REL,
    Primitive,
    compile_path,
)
from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    EDGE_LINE,
)
from tools.path_planner.pipeline_config import (
    PIPELINE_CONFIG,
    BlindMode,
    DirectionMode,
    LineMode,
    PipelineConfig,
)
from tools.path_planner.sim import (
    PathTrackerSim,
    VirtualChassis,
    walk_next_hop,
)
from tools.path_planner.sim.runtime import (
    ACT_BLIND_REVERSE,
    ACT_LINE_OVERSHOOT,
    ACT_LINE_REVERSE,
    LINE_SENSOR_AXLE_OFFSET_REAR_CM,
)

# A non-default field-sourced pipeline pair. Both differ from the singleton
# (NORMAL / SYNC_TSHAPE), and neither is the STOP_AT_LINE blind tag — so the
# REAR blind hop deterministically dead-reckons (ACT_BLIND_REVERSE) regardless
# of the chassis board count, keeping the emitted blind_mode observable.
_FIELD_LINE_MODE = LineMode.FAST
_FIELD_BLIND_MODE = BlindMode.GYRO_TSHAPE

# Sanity: the chosen field modes MUST differ from the singleton defaults, or
# the "differs from singleton-default emission" assertion is vacuous.
assert _FIELD_LINE_MODE != PIPELINE_CONFIG.line
assert _FIELD_BLIND_MODE != PIPELINE_CONFIG.blind
assert _FIELD_BLIND_MODE != BlindMode.STOP_AT_LINE


def _build_fixture(*, line_mode_name=None, blind_mode_name=None):
    """Return ``(field_data, graph, next_hop)`` for the A->B->C REAR fixture.

    ``A --LINE--> B --BLIND--> C`` with a 90deg turn at B, both edges forced
    REAR. The turn at B is deliberate: a collinear A->B->C triple would let the
    sim collapse the intermediate junction into an ``ACT_LINE_PASS`` (which
    ``compile_path`` never emits), breaking byte-for-byte parity. The turn keeps
    each LINE hop on the plain ``ACT_LINE_OVERSHOOT`` path so the two streams
    stay comparable.

    When ``line_mode_name`` / ``blind_mode_name`` are given (enum member names)
    they are attached as ``PIPELINE_LINE_MODE`` / ``PIPELINE_BLIND_MODE`` on the
    field; omitting them leaves the field with no pipeline keys (so
    ``from_field_data`` falls back to the singleton).
    """
    nodes = {"A": (0.0, 0.0), "B": (50.0, 0.0), "C": (50.0, 40.0)}
    field_kwargs = dict(
        NODES=nodes,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        PIVOT_TURNS=[],
    )
    if line_mode_name is not None:
        field_kwargs["PIPELINE_LINE_MODE"] = line_mode_name
    if blind_mode_name is not None:
        field_kwargs["PIPELINE_BLIND_MODE"] = blind_mode_name
    field_data = SimpleNamespace(**field_kwargs)

    graph = nx.DiGraph()
    for name, (x, y) in nodes.items():
        graph.add_node(name, x_cm=float(x), y_cm=float(y))

    # A->B LINE (forward heading 0), B->C BLIND (forward heading 0). Both
    # carry direction_override=FORCE_REVERSE and NO primitive_mode, so the
    # resolved cfg.line / cfg.blind is the only source of the emitted mode.
    graph.add_edge(
        "A", "B",
        type=EDGE_LINE,
        distance_cm=50.0,
        heading_deg=0.0,
        direction_override=int(DirectionMode.FORCE_REVERSE),
    )
    graph.add_edge(
        "B", "A",
        type=EDGE_LINE,
        distance_cm=50.0,
        heading_deg=180.0,
    )
    graph.add_edge(
        "B", "C",
        type=EDGE_BLIND,
        distance_cm=40.0,
        heading_deg=90.0,
        direction_override=int(DirectionMode.FORCE_REVERSE),
    )
    graph.add_edge(
        "C", "B",
        type=EDGE_BLIND,
        distance_cm=40.0,
        heading_deg=270.0,
    )

    next_hop = {
        "A": {"A": "A", "B": "B", "C": "B"},
        "B": {"A": "A", "B": "B", "C": "C"},
        "C": {"A": "B", "B": "B", "C": "C"},
    }
    return field_data, graph, next_hop


def _run_sim(field_data, graph, next_hop, goal: str) -> List[Primitive]:
    """Drive a fresh ``PathTrackerSim`` to ``goal`` and return its primitives."""
    chassis = VirtualChassis.from_field_data(field_data)
    sim = PathTrackerSim(field_data, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim.go_to(goal)


def _interleave_line_overshoot(primitives: List[Primitive]) -> List[Primitive]:
    """Insert the sim-only ``ACT_LINE_OVERSHOOT`` after each REAR LINE primitive.

    ``compile_path`` never emits the LINE_OVERSHOOT step (the C++ runtime owns
    it as an FSM state); the sim emits it after every EDGE_LINE traversal. This
    fixture has a single REAR LINE hop, so the rolled offset is always the REAR
    board's own axle offset.
    """
    out: List[Primitive] = []
    for prim in primitives:
        out.append(prim)
        if prim[0] == ACT_LINE_REVERSE:
            out.append((ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_REAR_CM))
    return out


def _line_reverse_mode(primitives: List[Primitive]) -> LineMode:
    """Return the line_mode element of the lone ACT_LINE_REVERSE primitive."""
    matches = [p for p in primitives if p[0] == ACT_LINE_REVERSE]
    assert len(matches) == 1, f"expected exactly one ACT_LINE_REVERSE, got {matches!r}"
    # (ACT_LINE_REVERSE, target_node, line_mode)
    return matches[0][2]


def _blind_reverse_mode(primitives: List[Primitive]) -> BlindMode:
    """Return the blind_mode element of the lone ACT_BLIND_REVERSE primitive."""
    matches = [p for p in primitives if p[0] == ACT_BLIND_REVERSE]
    assert len(matches) == 1, f"expected exactly one ACT_BLIND_REVERSE, got {matches!r}"
    # (ACT_BLIND_REVERSE, target_node, distance_cm, blind_mode)
    return matches[0][3]


# ---------------------------------------------------------------------------
# TC10 — sim vs compiler primitives with non-default PIPELINE_LINE_MODE /
#        PIPELINE_BLIND_MODE; match each other AND differ from the singleton
#        default emission (a missed call site diverges).
# ---------------------------------------------------------------------------


def test_tc10_sim_matches_compiler_under_field_pipeline_modes():
    """The sim mirrors the compiler when the field carries non-default modes."""
    field_data, graph, next_hop = _build_fixture(
        line_mode_name=_FIELD_LINE_MODE.name,
        blind_mode_name=_FIELD_BLIND_MODE.name,
    )

    walked = walk_next_hop(next_hop, "A", "C")
    compiler_prims = compile_path(
        walked, field_data.START_HEADING_DEG, graph, field_data
    )
    sim_prims = _run_sim(field_data, graph, next_hop, "C")

    # Byte-identical parity: the sim stream equals the compiler stream with the
    # sim-only LINE_OVERSHOOT step re-inserted after the REAR LINE hop. If any
    # resolver call site were left on the singleton in EITHER module, the
    # emitted mode elements would diverge and this equality would fail.
    assert sim_prims == _interleave_line_overshoot(compiler_prims)

    # The fixture forces both hops REAR with a 90deg turn at B, so the sim path
    # is:
    #   (ACT_TURN_REL, 180.0)                          face REAR for A->B
    #   (ACT_LINE_REVERSE, 'B', cfg.line)              REAR line hop
    #   (ACT_LINE_OVERSHOOT, REAR_OFFSET)              sim-only roll
    #   (ACT_TURN_REL, 90.0)                           re-aim REAR for B->C
    #   (ACT_BLIND_REVERSE, 'C', 40.0, cfg.blind)      REAR blind dead-reckon
    assert sim_prims[0] == (ACT_TURN_REL, 180.0)
    assert sim_prims[-1][0] == ACT_BLIND_REVERSE


def test_tc10_emitted_modes_follow_the_field_not_the_singleton():
    """The emitted modes equal the field-sourced cfg, and differ from the singleton.

    This is the missed-call-site detector: ``resolve_line_mode`` /
    ``resolve_blind_mode`` are the ONLY path the field value reaches these REAR
    payloads, so a site left on ``PIPELINE_CONFIG`` would surface the singleton
    default here.
    """
    field_data, graph, next_hop = _build_fixture(
        line_mode_name=_FIELD_LINE_MODE.name,
        blind_mode_name=_FIELD_BLIND_MODE.name,
    )
    walked = walk_next_hop(next_hop, "A", "C")
    compiler_prims = compile_path(
        walked, field_data.START_HEADING_DEG, graph, field_data
    )
    sim_prims = _run_sim(field_data, graph, next_hop, "C")

    # Compiler side: emitted modes follow the field-sourced config.
    assert _line_reverse_mode(compiler_prims) == _FIELD_LINE_MODE
    assert _blind_reverse_mode(compiler_prims) == _FIELD_BLIND_MODE
    # Sim side: same.
    assert _line_reverse_mode(sim_prims) == _FIELD_LINE_MODE
    assert _blind_reverse_mode(sim_prims) == _FIELD_BLIND_MODE

    # And they differ from the singleton-default emission — built by compiling
    # the SAME path against a field that carries no pipeline keys (so cfg ==
    # PIPELINE_CONFIG). A missed call site would collapse the two emissions.
    default_field, default_graph, default_next_hop = _build_fixture()
    default_walked = walk_next_hop(default_next_hop, "A", "C")
    default_compiler_prims = compile_path(
        default_walked, default_field.START_HEADING_DEG, default_graph, default_field
    )
    default_sim_prims = _run_sim(default_field, default_graph, default_next_hop, "C")

    assert _line_reverse_mode(default_compiler_prims) == PIPELINE_CONFIG.line
    assert _blind_reverse_mode(default_compiler_prims) == PIPELINE_CONFIG.blind
    assert _line_reverse_mode(default_sim_prims) == PIPELINE_CONFIG.line
    assert _blind_reverse_mode(default_sim_prims) == PIPELINE_CONFIG.blind

    # The field-sourced emission must NOT equal the singleton-default emission.
    assert _line_reverse_mode(compiler_prims) != _line_reverse_mode(
        default_compiler_prims
    )
    assert _blind_reverse_mode(compiler_prims) != _blind_reverse_mode(
        default_compiler_prims
    )
    assert _line_reverse_mode(sim_prims) != _line_reverse_mode(default_sim_prims)
    assert _blind_reverse_mode(sim_prims) != _blind_reverse_mode(default_sim_prims)


def test_tc10_default_field_sim_matches_default_compiler():
    """Regression: with no pipeline keys the sim still mirrors the compiler."""
    field_data, graph, next_hop = _build_fixture()
    walked = walk_next_hop(next_hop, "A", "C")
    compiler_prims = compile_path(
        walked, field_data.START_HEADING_DEG, graph, field_data
    )
    sim_prims = _run_sim(field_data, graph, next_hop, "C")
    assert sim_prims == _interleave_line_overshoot(compiler_prims)


# ---------------------------------------------------------------------------
# TC10b — from_field_data(field with no pipeline keys) == PIPELINE_CONFIG
#         (byte-identity precondition: a field that omits the keys must not
#          perturb committed routes.cpp).
# ---------------------------------------------------------------------------


def test_tc10b_from_field_data_without_keys_equals_singleton():
    """A field with no PIPELINE_* keys resolves byte-identical to the singleton."""
    field_no_keys = SimpleNamespace(
        NODES={"A": (0.0, 0.0)},
        START_NODE="A",
        START_HEADING_DEG=0.0,
    )
    cfg = PipelineConfig.from_field_data(field_no_keys)
    assert cfg == PIPELINE_CONFIG
    # Field-by-field, to make a future field addition fail loudly here too.
    assert cfg.line == PIPELINE_CONFIG.line
    assert cfg.blind == PIPELINE_CONFIG.blind
    assert cfg.turn == PIPELINE_CONFIG.turn
    assert cfg.direction == PIPELINE_CONFIG.direction


def test_tc10b_from_field_data_with_keys_overrides_singleton():
    """A field that SETS the keys resolves to those values, not the singleton."""
    field_with_keys = SimpleNamespace(
        NODES={"A": (0.0, 0.0)},
        START_NODE="A",
        START_HEADING_DEG=0.0,
        PIPELINE_LINE_MODE=_FIELD_LINE_MODE.name,
        PIPELINE_BLIND_MODE=_FIELD_BLIND_MODE.name,
    )
    cfg = PipelineConfig.from_field_data(field_with_keys)
    assert cfg.line == _FIELD_LINE_MODE
    assert cfg.blind == _FIELD_BLIND_MODE
    # Unset keys still inherit the singleton.
    assert cfg.turn == PIPELINE_CONFIG.turn
    assert cfg.direction == PIPELINE_CONFIG.direction
    assert cfg != PIPELINE_CONFIG
