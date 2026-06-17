"""End-to-end blind->projection auto-seek parity tests (T16).

The auto-rule under test: a BLIND hop that FEEDS a PROJECTION hop should
re-anchor on a line first (``Drive.stopAtLine``) instead of dead-reckoning,
because the dead-reckoned projection that follows wants a fresh localization
fix to gate against. The decision is keyed on
``field_data.BLIND_PROJECTION_AUTO_SEEK and next_hop_is_projection`` and is
read defensively in both producers via
``getattr(field_data, "BLIND_PROJECTION_AUTO_SEEK", True)``.

Two producers must agree byte-for-byte on the emitted primitive stream:

* the runtime sim -- :meth:`tools.path_planner.sim.PathTrackerSim.go_to`
  (BLIND branch of ``_walk_hops``), and
* the offline compiler -- :func:`tools.path_planner.compiler.compile_path`
  (FRONT + REAR BLIND branches).

If they disagree, the sim-mirrors-robot guarantee is broken on exactly the
blind->projection class of edges this test exists to guard. Each case asserts
both (a) the specific opcode on the blind hop and (b) sim stream == compiler
stream.

Synthetic graph (smallest shape with the essential property)::

    S --BLIND 30cm  heading 0--> A --PROJECTION 20cm heading 0--> B

* ``S->A`` is BLIND; its NEXT hop ``A->B`` is PROJECTION, so when going S->B
  the auto-rule fires on ``S->A`` (auto on) -> ``ACT_STOP_AT_LINE``.
* Going S->A instead, the blind hop is the LAST hop (no projection follows),
  so the auto-rule never fires -> ``ACT_FORWARD`` (dead-reckon).
* With the toggle off, S->B suppresses the auto-rule -> ``ACT_FORWARD`` (the
  exact pre-auto dead-reckon behavior).

Every edge is heading 0 from a start heading of 0, so direction auto-resolves
FRONT on every hop (turn delta 0 < ``BIDIRECTIONAL_TURN_THRESHOLD_DEG`` = 90)
and no ``ACT_TURN_REL`` is emitted. There are no LINE edges, so the sim emits
no ``ACT_LINE_OVERSHOOT`` step -- the sim and compiler streams are directly
comparable without re-interleaving any sim-only overshoot steps.
"""

from __future__ import annotations

from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FORWARD,
    ACT_STOP_AT_LINE,
    compile_path,
)
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_PROJECTION
from tools.path_planner.sim import PathTrackerSim, VirtualChassis, walk_next_hop

# Edge geometry for the synthetic S -> A -> B graph. Distances are distinct so
# the per-hop payloads in the asserted streams are unambiguous.
_BLIND_DISTANCE_CM: float = 30.0
_PROJECTION_DISTANCE_CM: float = 20.0
_START_HEADING_DEG: float = 0.0

# Node coordinates placed along +x so each edge's heading is 0 deg, matching
# the declared ``heading_deg`` on every edge below. (The geometry is not
# asserted here -- only the emitted primitive streams are -- but keeping the
# coordinates consistent with the headings avoids a misleading fixture.)
_NODES: dict = {
    "S": (0.0, 0.0),
    "A": (_BLIND_DISTANCE_CM, 0.0),
    "B": (_BLIND_DISTANCE_CM + _PROJECTION_DISTANCE_CM, 0.0),
}

# Hand-rolled next-hop table for the 3-node chain S -> A -> B. Avoids the
# planner (and any pivot path) entirely, exactly like the 2-node ``_next_hop_*``
# helpers in ``test_runtime_sim.py`` / ``test_compiler.py``.
_NEXT_HOP: dict = {
    "S": {"S": "S", "A": "A", "B": "A"},
    "A": {"S": "S", "A": "A", "B": "B"},
    "B": {"S": "A", "A": "A", "B": "B"},
}


def _make_field_data(*, auto_seek: bool) -> SimpleNamespace:
    """Return a field_data namespace carrying the auto-seek toggle.

    Mirrors the minimal field surface the sim consumes (``NODES`` +
    ``START_NODE`` + ``START_HEADING_DEG`` + an empty ``PIVOT_TURNS`` so the
    ADR-036 pivot resolver constructs cleanly) plus the
    ``BLIND_PROJECTION_AUTO_SEEK`` attribute the auto-rule reads. The same
    object is handed to :class:`PathTrackerSim` (sim path) and as the
    ``field_data`` argument to :func:`compile_path` (compiler path), so both
    producers observe the identical toggle value.

    :param auto_seek: value baked onto ``BLIND_PROJECTION_AUTO_SEEK``.
    :return: a :class:`types.SimpleNamespace` field-data mirror.
    """
    return SimpleNamespace(
        NODES=dict(_NODES),
        START_NODE="S",
        START_HEADING_DEG=_START_HEADING_DEG,
        PIVOT_TURNS=[],
        BLIND_PROJECTION_AUTO_SEEK=auto_seek,
    )


def _build_graph() -> nx.DiGraph:
    """Build the S->A (BLIND) ->B (PROJECTION) DiGraph with reverse edges.

    The forward edges carry no ``primitive_mode`` override, so the BLIND edge
    resolves to ``PIPELINE_CONFIG.blind`` (``SYNC_TSHAPE``, NOT
    ``STOP_AT_LINE``): the only thing that can make ``S->A`` seek a line is the
    auto-rule, which is exactly what these tests toggle. Reverse edges are
    added so any all-pairs sweep is well-formed and so the next-hop table can
    walk in either direction; they are never exercised by the S->B / S->A
    forward walks asserted here.

    :return: a 3-node :class:`networkx.DiGraph` ready for ``compile_path`` and
        :class:`PathTrackerSim`.
    """
    g = nx.DiGraph()
    for name, (x, y) in _NODES.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y), type="line")

    # S -> A: BLIND (plain; no primitive_mode -> inherits PIPELINE_CONFIG.blind).
    g.add_edge(
        "S", "A",
        type=EDGE_BLIND,
        distance_cm=_BLIND_DISTANCE_CM,
        heading_deg=0.0,
    )
    g.add_edge(
        "A", "S",
        type=EDGE_BLIND,
        distance_cm=_BLIND_DISTANCE_CM,
        heading_deg=180.0,
    )

    # A -> B: PROJECTION.
    g.add_edge(
        "A", "B",
        type=EDGE_PROJECTION,
        distance_cm=_PROJECTION_DISTANCE_CM,
        heading_deg=0.0,
    )
    g.add_edge(
        "B", "A",
        type=EDGE_PROJECTION,
        distance_cm=_PROJECTION_DISTANCE_CM,
        heading_deg=180.0,
    )
    return g


def _sim_stream(field_data: SimpleNamespace, graph: nx.DiGraph, goal: str) -> list:
    """Run ``PathTrackerSim.go_to(goal)`` from ``S`` and return its stream."""
    chassis = VirtualChassis.from_field_data(field_data)
    sim = PathTrackerSim(field_data, graph, _NEXT_HOP, chassis)
    sim.localization.zero_heading_from_current()
    return sim.go_to(goal)


def _compiler_stream(field_data: SimpleNamespace, graph: nx.DiGraph, goal: str) -> list:
    """Walk NEXT_HOP from ``S`` to ``goal`` and compile it to a primitive stream."""
    walked = walk_next_hop(_NEXT_HOP, "S", goal)
    return compile_path(walked, field_data.START_HEADING_DEG, graph, field_data)


# ---------------------------------------------------------------------------
# TC-E2E -- auto on: blind hop feeding a projection seeks a line.
# ---------------------------------------------------------------------------


def test_tc_e2e_auto_on_blind_hop_emits_stop_at_line() -> None:
    """S->B with BLIND_PROJECTION_AUTO_SEEK=True: blind hop S->A emits ACT_STOP_AT_LINE.

    The auto-rule fires because the hop after S->A (A->B) is a PROJECTION edge.
    Both the sim and the compiler must emit ``ACT_STOP_AT_LINE`` for the blind
    hop and ``ACT_FOLLOW_DIST`` for the projection hop, and the two full
    streams must be byte-identical (there are no LINE edges, so no sim-only
    ``ACT_LINE_OVERSHOOT`` step is interleaved -- the streams compare directly).
    """
    field_data = _make_field_data(auto_seek=True)
    graph = _build_graph()

    sim_stream = _sim_stream(field_data, graph, "B")
    compiler_stream = _compiler_stream(field_data, graph, "B")

    expected = [
        (ACT_STOP_AT_LINE, _BLIND_DISTANCE_CM),
        (ACT_FOLLOW_DIST, _PROJECTION_DISTANCE_CM),
    ]

    # (a) the blind hop seeks a line in BOTH producers.
    assert sim_stream[0] == (ACT_STOP_AT_LINE, _BLIND_DISTANCE_CM)
    assert compiler_stream[0] == (ACT_STOP_AT_LINE, _BLIND_DISTANCE_CM)
    # The blind hop is NOT a dead-reckoned forward when the auto-rule fires.
    assert sim_stream[0][0] != ACT_FORWARD
    assert compiler_stream[0][0] != ACT_FORWARD

    # (b) full streams are byte-identical to each other (the point of the test)
    # and match the expected blind-seek + projection-follow sequence.
    assert sim_stream == compiler_stream
    assert sim_stream == expected


# ---------------------------------------------------------------------------
# TC-E2E-final -- blind hop is the goal: no projection follows, no seek.
# ---------------------------------------------------------------------------


def test_tc_e2e_final_blind_goal_hop_dead_reckons() -> None:
    """S->A with auto on: the blind hop is the LAST hop, so it dead-reckons.

    Going to ``A`` makes ``S->A`` the final hop with no projection edge after
    it, so ``next_hop_is_projection`` is False and the auto-rule never fires --
    even though the toggle is on. Both producers must emit ``ACT_FORWARD``
    (dead-reckon), NOT ``ACT_STOP_AT_LINE``, and the streams must be
    byte-identical. This guards the boundary: the auto-rule keys on the NEXT
    hop, not on the blind edge alone.
    """
    field_data = _make_field_data(auto_seek=True)
    graph = _build_graph()

    sim_stream = _sim_stream(field_data, graph, "A")
    compiler_stream = _compiler_stream(field_data, graph, "A")

    expected = [(ACT_FORWARD, _BLIND_DISTANCE_CM)]

    # (a) the blind hop dead-reckons (no line seek) in BOTH producers.
    assert sim_stream[0] == (ACT_FORWARD, _BLIND_DISTANCE_CM)
    assert compiler_stream[0] == (ACT_FORWARD, _BLIND_DISTANCE_CM)
    assert sim_stream[0][0] != ACT_STOP_AT_LINE
    assert compiler_stream[0][0] != ACT_STOP_AT_LINE

    # (b) full streams are byte-identical and match the dead-reckon sequence.
    assert sim_stream == compiler_stream
    assert sim_stream == expected


# ---------------------------------------------------------------------------
# TC-TOGGLE-OFF -- auto off: the rule is suppressed, restoring prior behavior.
# ---------------------------------------------------------------------------


def test_tc_toggle_off_suppresses_auto_seek() -> None:
    """S->B with BLIND_PROJECTION_AUTO_SEEK=False: blind hop S->A dead-reckons.

    The blind hop S->A still feeds a PROJECTION hop A->B, but the toggle is
    off, so ``auto_rule_active`` is False and the seek is suppressed. Both
    producers must emit ``ACT_FORWARD`` for the blind hop (identical to the
    pre-auto dead-reckon behavior), NOT ``ACT_STOP_AT_LINE``, and the full
    streams must be byte-identical. This proves the toggle restores exact prior
    behavior -- the projection hop A->B is unchanged (``ACT_FOLLOW_DIST``).
    """
    field_data = _make_field_data(auto_seek=False)
    graph = _build_graph()

    sim_stream = _sim_stream(field_data, graph, "B")
    compiler_stream = _compiler_stream(field_data, graph, "B")

    expected = [
        (ACT_FORWARD, _BLIND_DISTANCE_CM),
        (ACT_FOLLOW_DIST, _PROJECTION_DISTANCE_CM),
    ]

    # (a) the blind hop dead-reckons in BOTH producers (auto-rule suppressed).
    assert sim_stream[0] == (ACT_FORWARD, _BLIND_DISTANCE_CM)
    assert compiler_stream[0] == (ACT_FORWARD, _BLIND_DISTANCE_CM)
    assert sim_stream[0][0] != ACT_STOP_AT_LINE
    assert compiler_stream[0][0] != ACT_STOP_AT_LINE

    # (b) full streams are byte-identical and match the suppressed sequence.
    assert sim_stream == compiler_stream
    assert sim_stream == expected
