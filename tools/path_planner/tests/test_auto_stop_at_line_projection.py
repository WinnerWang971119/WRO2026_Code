"""End-to-end blind->projection-node auto-seek parity tests (issue #64).

The auto-rule under test: a BLIND hop whose OWN destination is a PROJECTION
node should re-anchor on that node's line first (``Drive.stopAtLine``) instead
of dead-reckoning, because the projection node is on a line but at no junction
-- the seek gives a fresh localization fix. Issue #64 moved projection from an
edge type to a NODE type, so the decision is now a purely LOCAL read on the
hop's destination node (``is_projection_node(graph, dest)``), gated by
``field_data.BLIND_PROJECTION_AUTO_SEEK`` -- there is NO goal-relative
``next_hop_is_projection`` / ``NEXT_HOP[next][goal]`` peek. The toggle is read
defensively in both producers via
``getattr(field_data, "BLIND_PROJECTION_AUTO_SEEK", True)``.

Two producers must agree byte-for-byte on the emitted primitive stream:

* the runtime sim -- :meth:`tools.path_planner.sim.PathTrackerSim.go_to`
  (BLIND branch of ``_walk_hops``), and
* the offline compiler -- :func:`tools.path_planner.compiler.compile_path`
  (FRONT + REAR BLIND branches).

If they disagree, the sim-mirrors-robot guarantee is broken on exactly the
blind->projection class of hops this test exists to guard. Each case asserts
both (a) the specific opcode on the blind hop and (b) sim stream == compiler
stream.

Synthetic graph (smallest shape with the essential property)::

    S --BLIND 30cm heading 0--> P  (P is a PROJECTION node)

* ``S->P`` is a BLIND hop whose destination ``P`` is a projection node, so when
  going S->P the LOCAL auto-rule fires on ``S->P`` (auto on) ->
  ``ACT_STOP_AT_LINE``. The re-anchor is a full advance onto P.
* With the toggle off, S->P suppresses the auto-rule -> ``ACT_FORWARD`` (the
  exact pre-auto dead-reckon behavior).

A second node ``Q`` (a plain blind node reached by a BLIND hop ``P->Q``) lets a
route ``S->Q`` prove the auto-rule keys on the LOCAL destination, not the goal:
the ``P->Q`` hop into a blind node never seeks even though it follows a
projection-node arrival.

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
    ACT_FORWARD,
    ACT_STOP_AT_LINE,
    compile_path,
)
from tools.path_planner.graph_builder import EDGE_BLIND
from tools.path_planner.sim import PathTrackerSim, VirtualChassis, walk_next_hop

# Edge geometry for the synthetic S -> P -> Q graph. Distances are distinct so
# the per-hop payloads in the asserted streams are unambiguous.
_SP_DISTANCE_CM: float = 30.0
_PQ_DISTANCE_CM: float = 20.0
_START_HEADING_DEG: float = 0.0

# Node coordinates placed along +x so each edge's heading is 0 deg, matching
# the declared ``heading_deg`` on every edge below. (The geometry is not
# asserted here -- only the emitted primitive streams are -- but keeping the
# coordinates consistent with the headings avoids a misleading fixture.)
_NODES: dict = {
    "S": (0.0, 0.0),
    "P": (_SP_DISTANCE_CM, 0.0),
    "Q": (_SP_DISTANCE_CM + _PQ_DISTANCE_CM, 0.0),
}

# Hand-rolled next-hop table for the 3-node chain S -> P -> Q. Avoids the
# planner (and any pivot path) entirely, exactly like the 2-node ``_next_hop_*``
# helpers in ``test_runtime_sim.py`` / ``test_compiler.py``.
_NEXT_HOP: dict = {
    "S": {"S": "S", "P": "P", "Q": "P"},
    "P": {"S": "S", "P": "P", "Q": "Q"},
    "Q": {"S": "P", "P": "P", "Q": "Q"},
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
    """Build the S->P (BLIND) ->Q (BLIND) DiGraph with reverse edges.

    Issue #64 node model: ``P`` is typed ``"projection"`` (the on-line,
    not-at-a-junction waypoint) and every edge is a plain BLIND hop. The local
    auto-rule fires on a BLIND hop whose DESTINATION is the projection node ``P``
    -- the only thing that can make ``S->P`` seek a line -- which is exactly what
    these tests toggle. The ``P->Q`` hop targets the blind node ``Q``, so it
    never seeks: it proves the rule reads the LOCAL destination, not the goal.
    Reverse edges are added so any all-pairs sweep is well-formed and so the
    next-hop table can walk in either direction.

    :return: a 3-node :class:`networkx.DiGraph` ready for ``compile_path`` and
        :class:`PathTrackerSim`.
    """
    g = nx.DiGraph()
    types = {"S": "blind", "P": "projection", "Q": "blind"}
    for name, (x, y) in _NODES.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y), type=types[name])

    # S -> P: BLIND into the projection node P (the local auto-seek target).
    g.add_edge("S", "P", type=EDGE_BLIND, distance_cm=_SP_DISTANCE_CM, heading_deg=0.0)
    g.add_edge("P", "S", type=EDGE_BLIND, distance_cm=_SP_DISTANCE_CM, heading_deg=180.0)

    # P -> Q: BLIND into the blind node Q (never auto-seeks -- local dest read).
    g.add_edge("P", "Q", type=EDGE_BLIND, distance_cm=_PQ_DISTANCE_CM, heading_deg=0.0)
    g.add_edge("Q", "P", type=EDGE_BLIND, distance_cm=_PQ_DISTANCE_CM, heading_deg=180.0)
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
# TC-E2E -- auto on: a blind hop INTO a projection node seeks a line.
# ---------------------------------------------------------------------------


def test_tc_e2e_auto_on_blind_into_projection_emits_stop_at_line() -> None:
    """S->P with BLIND_PROJECTION_AUTO_SEEK=True: blind hop S->P emits ACT_STOP_AT_LINE.

    The auto-rule fires because the hop's OWN destination ``P`` is a projection
    node (a purely LOCAL read -- no goal-relative peek). Both the sim and the
    compiler must emit ``ACT_STOP_AT_LINE`` for the blind hop, and the two full
    streams must be byte-identical (there are no LINE edges, so no sim-only
    ``ACT_LINE_OVERSHOOT`` step is interleaved -- the streams compare directly).
    """
    field_data = _make_field_data(auto_seek=True)
    graph = _build_graph()

    sim_stream = _sim_stream(field_data, graph, "P")
    compiler_stream = _compiler_stream(field_data, graph, "P")

    expected = [(ACT_STOP_AT_LINE, _SP_DISTANCE_CM)]

    # (a) the blind hop seeks a line in BOTH producers.
    assert sim_stream[0] == (ACT_STOP_AT_LINE, _SP_DISTANCE_CM)
    assert compiler_stream[0] == (ACT_STOP_AT_LINE, _SP_DISTANCE_CM)
    # The blind hop is NOT a dead-reckoned forward when the auto-rule fires.
    assert sim_stream[0][0] != ACT_FORWARD
    assert compiler_stream[0][0] != ACT_FORWARD

    # (b) full streams are byte-identical to each other (the point of the test)
    # and match the expected blind-seek sequence.
    assert sim_stream == compiler_stream
    assert sim_stream == expected
    # The seek re-anchors localization ON the projection node P (full advance).
    chassis = VirtualChassis.from_field_data(field_data)
    sim = PathTrackerSim(field_data, graph, _NEXT_HOP, chassis)
    sim.localization.zero_heading_from_current()
    sim.go_to("P")
    assert sim.localization.current_node() == "P"


# ---------------------------------------------------------------------------
# TC-E2E-local -- the rule reads the LOCAL destination, not the goal.
# ---------------------------------------------------------------------------


def test_tc_e2e_local_dest_read_not_goal_relative() -> None:
    """S->Q with auto on: only the S->P hop (into the projection node) seeks.

    Routing S->Q walks S->P->Q. The first hop ``S->P`` seeks (its destination
    ``P`` is a projection node); the second hop ``P->Q`` targets the blind node
    ``Q``, so it dead-reckons even though it FOLLOWS a projection-node arrival.
    This pins the issue-#64 contract that the auto-rule is a LOCAL destination
    read, not a goal-relative ``next_hop_is_projection`` peek. Both producers
    must agree byte-for-byte.
    """
    field_data = _make_field_data(auto_seek=True)
    graph = _build_graph()

    sim_stream = _sim_stream(field_data, graph, "Q")
    compiler_stream = _compiler_stream(field_data, graph, "Q")

    expected = [
        (ACT_STOP_AT_LINE, _SP_DISTANCE_CM),   # S->P: into a projection node, seeks.
        (ACT_FORWARD, _PQ_DISTANCE_CM),        # P->Q: into a blind node, dead-reckons.
    ]

    assert sim_stream == compiler_stream
    assert sim_stream == expected


# ---------------------------------------------------------------------------
# TC-TOGGLE-OFF -- auto off: the rule is suppressed, restoring prior behavior.
# ---------------------------------------------------------------------------


def test_tc_toggle_off_suppresses_auto_seek() -> None:
    """S->P with BLIND_PROJECTION_AUTO_SEEK=False: blind hop S->P dead-reckons.

    The blind hop ``S->P`` still targets the projection node ``P``, but the
    toggle is off, so ``auto_rule_active`` is False and the seek is suppressed.
    Both producers must emit ``ACT_FORWARD`` for the blind hop (identical to the
    pre-auto dead-reckon behavior), NOT ``ACT_STOP_AT_LINE``, and the full
    streams must be byte-identical. This proves the toggle restores exact prior
    behavior.
    """
    field_data = _make_field_data(auto_seek=False)
    graph = _build_graph()

    sim_stream = _sim_stream(field_data, graph, "P")
    compiler_stream = _compiler_stream(field_data, graph, "P")

    expected = [(ACT_FORWARD, _SP_DISTANCE_CM)]

    # (a) the blind hop dead-reckons in BOTH producers (auto-rule suppressed).
    assert sim_stream[0] == (ACT_FORWARD, _SP_DISTANCE_CM)
    assert compiler_stream[0] == (ACT_FORWARD, _SP_DISTANCE_CM)
    assert sim_stream[0][0] != ACT_STOP_AT_LINE
    assert compiler_stream[0][0] != ACT_STOP_AT_LINE

    # (b) full streams are byte-identical and match the suppressed sequence.
    assert sim_stream == compiler_stream
    assert sim_stream == expected
