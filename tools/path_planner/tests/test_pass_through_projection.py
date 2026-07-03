"""Dedicated `line(A)-projection(B)-line(C)` pass-through suite (T21, issue #64).

Covers the three pass-through test cases from the plan
``docs/plans/2026-06-22-issue64-projection-as-node-property.md`` (Testing
Strategy table) at the SIM layer. The host-C++ mirror lives in
``WRO2026/test/test_path_tracker_pass_through_projection.cpp``.

* **TC11** -- ``line(A)-projection(B)-line(C)`` routed PAST the projection
  node ``B``: one continuous A->C line-follow with a MID-FOLLOW dead-reckon
  advance at ``B`` (no stop) and a re-anchor at ``C``'s junction. The sim must
  emit :data:`ACT_FOLLOW_DIST_PASS` at ``B`` (NOT the plain ``ACT_FOLLOW_DIST``
  stop, and NOT the junction-gated ``ACT_LINE_PASS`` -- ``B`` is marker-less),
  and the chassis axle must land ON ``C`` (axle-on-node, tight tolerance).
* **TC12** -- ``goTo(B)`` where ``B`` IS the projection goal: the not-final
  guard makes ``B`` a DISTANCE stop. The sim emits the plain
  ``ACT_FOLLOW_DIST`` (no ``_PASS`` tag) and the chassis stops AT ``B``.
* **TC13** -- a BLIND-flanked projection node is a NORMAL stop (no
  pass-through): with the outbound ``B->C`` flank a BLIND edge, the gate's
  both-flanks-LINE condition fails and the sim falls back to the plain
  ``ACT_FOLLOW_DIST`` stop at ``B``.

CRITICAL fixture note (see the plan's T21 row + the TOY-field warning): the toy
field deliberately keeps a DIRECT ``A-B`` LINE edge alongside the
``A-P1-B`` triple (equal cost), so the toy ``NEXT_HOP`` may BYPASS ``P1`` and a
plain ``go_to`` would never reach the projection node. These tests therefore
build their OWN small fields whose routed shortest path genuinely goes
``A->B->C`` THROUGH the projection node ``B`` -- the direct A->C edge is OMITTED
so the only route is through ``B``. Each test then asserts the pass-through (or
its absence) ACTUALLY FIRED via the emitted ``_PASS`` tag, never a route that
silently skipped the projection node.

All tests exercise the Python sim (``tools.path_planner.sim.runtime``) only;
hardware verification is deferred to the bench gate (ADR amendment).
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_TURN_REL,
)
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE
from tools.path_planner.sim import VirtualChassis, PathTrackerSim
from tools.path_planner.sim.runtime import (
    ACT_FOLLOW_DIST_PASS,
    ACT_LINE_OVERSHOOT,
    ACT_LINE_PASS,
    ACT_STOP_AT_LINE,
    LINE_SENSOR_AXLE_OFFSET_CM,
)

# Tolerance for the axle-on-node position assertions. The sim's chassis motion is
# exact float arithmetic (no encoder noise model), so the continuous A->C follow
# nets EXACTLY the summed edge lengths -- a 1e-9 cm tolerance is the same one
# ``test_pass_through.py`` uses for the line-line-line collapse.
_POS_TOL_CM: float = 1e-9


# ---------------------------------------------------------------------------
# Field / graph / next-hop builders (custom fields so the route genuinely goes
# THROUGH the projection node -- the toy field's direct A-B edge is omitted).
# ---------------------------------------------------------------------------


def _minimal_field(nodes: dict, start_node: str, start_heading_deg: float) -> SimpleNamespace:
    """Return the ``SimpleNamespace`` field-data stub ``PathTrackerSim`` consumes."""
    return SimpleNamespace(
        NODES=nodes,
        START_NODE=start_node,
        START_HEADING_DEG=start_heading_deg,
    )


def _make_graph(nodes: dict, edges: list, node_types: dict) -> nx.DiGraph:
    """Build a ``DiGraph`` from ``(u, v, type, distance_cm, heading_deg)`` tuples.

    A symmetric reverse edge is added for every forward edge (heading + 180 deg)
    so the hand-rolled ``next_hop`` table can always cover the return direction.
    ``node_types`` sets each node's ``type`` attribute so the sim's LOCAL
    ``is_projection_node`` read resolves correctly; an absent entry defaults to
    ``"line"``.
    """
    g = nx.DiGraph()
    for name, (x, y) in nodes.items():
        g.add_node(
            name,
            x_cm=float(x),
            y_cm=float(y),
            type=node_types.get(name, "line"),
        )
    for u, v, edge_type, dist, hdg in edges:
        g.add_edge(u, v, type=edge_type, distance_cm=float(dist), heading_deg=float(hdg))
        g.add_edge(
            v, u,
            type=edge_type,
            distance_cm=float(dist),
            heading_deg=(float(hdg) + 180.0) % 360.0,
        )
    return g


def _dense_next_hop(edges_raw: list) -> dict:
    """Build a dense next-hop table for a single linear directed chain.

    ``edges_raw`` must form ONE directed path from the first entry's ``u`` to the
    last entry's ``v`` (no branches). Because there is no direct shortcut edge,
    every interior node is forced onto the chain -- so a ``go_to`` toward a node
    beyond the projection node MUST route THROUGH it (the whole point of these
    custom fixtures). Unreachable entries are ``None``.
    """
    ordered: list = []
    for u, v, *_ in edges_raw:
        if not ordered:
            ordered.append(u)
        ordered.append(v)

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
            row[tgt] = fwd.get(src) if tgt_idx > src_idx else bwd.get(src)
        table[src] = row
    return table


def _make_sim(
    nodes: dict,
    edges_raw: list,
    start_node: str,
    start_heading_deg: float,
    node_types: dict,
):
    """Return a fully wired ``(sim, chassis)`` pair ready for ``go_to()`` calls."""
    fd = _minimal_field(nodes, start_node, start_heading_deg)
    graph = _make_graph(nodes, edges_raw, node_types)
    next_hop = _dense_next_hop(edges_raw)
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, chassis


# ===========================================================================
# TC11 -- line(A)-projection(B)-line(C) routed PAST B: the distance-gated
#         pass-through actually FIRES (continuous A->C, dead-reckon at B,
#         re-anchor at C, axle ON C).
# ===========================================================================


def test_tc11_line_projection_line_routed_past_b_passes_through():
    """TC11: a collinear ``line(A)-projection(B)-line(C)`` triple routed to a goal
    PAST ``B`` collapses to one continuous A->C follow.

    The route genuinely goes A->B->C THROUGH the projection node ``B``: there is
    NO direct A->C edge, so ``NEXT_HOP[A][C] = B`` and the planner cannot bypass
    ``B``. The A->B LINE-into-projection hop must therefore emit
    :data:`ACT_FOLLOW_DIST_PASS` (the distance-gated pass-through, NOT a stop and
    NOT the junction-gated ``ACT_LINE_PASS``), the B->C LINE hop re-anchors at the
    junction ``C`` (``FOLLOW_INT`` + ``LINE_OVERSHOOT``), and the chassis axle
    lands ON ``C``.

    Layout (both flanks LINE + collinear on y = 0 -> pass-through qualifies):
        A(0,0) --LINE 0deg--> B(50,0) [projection] --LINE 0deg--> C(120,0)
    """
    nodes = {
        "A": (0.0, 0.0),
        "B": (50.0, 0.0),
        "C": (120.0, 0.0),
    }
    edges_raw = [
        ("A", "B", EDGE_LINE, 50.0, 0.0),
        ("B", "C", EDGE_LINE, 70.0, 0.0),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", 0.0, node_types={"B": "projection"})

    # Sanity: the route really goes A->B->C through the projection node (no direct
    # A->C shortcut). A test that "passes" on a route that never reaches B is
    # worthless, so pin the walk first.
    assert sim.walked_path("C") == ["A", "B", "C"]

    prims = sim.go_to("C")
    # Index by tag only (`p[0]`), not a 2-tuple unpack: this exact PROJECTION
    # pass-through also emits a variable-length ADR-052
    # `(ACT_HEADING_RESET, node, pass_kind, heading_deg)` record (the knob
    # defaults ON), so a blind `for tag, _ in prims` 2-tuple unpack would raise.
    act_tags = [p[0] for p in prims]

    # (1) The A->B leg PASSED THROUGH B: the distance-gated pass-through tag fired,
    #     NOT the plain FOLLOW_DIST stop. This is the load-bearing assertion -- it
    #     proves the pass-through actually triggered, not a bypassed route.
    assert prims[0] == (ACT_FOLLOW_DIST_PASS, 50.0)
    assert ACT_FOLLOW_DIST not in act_tags, (
        "A plain ACT_FOLLOW_DIST means the follow STOPPED at B -- the pass-through "
        "did not fire."
    )
    # (2) B is marker-less, so the junction-gated ACT_LINE_PASS must NOT appear for
    #     the A->B leg (it would imply B was treated as a real intersection).
    assert ACT_LINE_PASS not in act_tags

    # (3) The B->C LINE leg re-anchors at the real junction C: FOLLOW_INT + the
    #     LINE_OVERSHOOT axle roll.
    assert ACT_FOLLOW_INT in act_tags
    assert (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM) in prims

    # (4) A->B->C is collinear (both edges heading 0deg), so the pass-through fires
    #     WITHOUT a turn between the two legs.
    assert ACT_TURN_REL not in act_tags

    # (5) Localization re-anchored at C, and the continuous A->C follow nets exactly
    #     edge(A->B) + edge(B->C) -- the chassis axle lands ON C (axle-on-node).
    assert sim.localization.current_node() == "C"
    cx, cy = nodes["C"]
    assert math.hypot(sim.chassis.x_cm - cx, sim.chassis.y_cm - cy) < _POS_TOL_CM


def test_tc11_dead_reckon_advance_stamped_mid_follow_at_b():
    """TC11 (Localization): the dead-reckon advance at ``B`` is stamped MID-FOLLOW
    -- the moment the ``ACT_FOLLOW_DIST_PASS`` step is emitted Localization is
    already on ``B`` (no stop), and the very next emitted step is the onward B->C
    follow, NOT a stop-and-replan at ``B``.

    Walk the stream step-by-step via ``step_through`` and assert the node is ``B``
    immediately after the pass-through step, then advances to ``C`` -- the
    one-advance-per-node invariant holds across the marker-less node with no
    intervening stop primitive.
    """
    nodes = {
        "A": (0.0, 0.0),
        "B": (40.0, 0.0),
        "C": (100.0, 0.0),
    }
    edges_raw = [
        ("A", "B", EDGE_LINE, 40.0, 0.0),
        ("B", "C", EDGE_LINE, 60.0, 0.0),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", 0.0, node_types={"B": "projection"})

    saw_pass_at_b = False
    node_after_pass = None
    # Unpack the primitive by tag only (`prim[0]`), not a 2-tuple destructure:
    # this exact PROJECTION pass-through also emits a variable-length ADR-052
    # `(ACT_HEADING_RESET, node, pass_kind, heading_deg)` record (the knob
    # defaults ON), so a blind `(tag, _payload)` unpack would raise.
    for prim, _snap in sim.step_through("C"):
        tag = prim[0]
        if tag == ACT_FOLLOW_DIST_PASS:
            saw_pass_at_b = True
            # The pass-through stamps on_dead_reckoned_advance(B) BEFORE yielding,
            # so Localization is already on B at this step (mid-follow, no stop).
            node_after_pass = sim.localization.current_node()

    assert saw_pass_at_b, "the line-proj-line pass-through step never fired"
    assert node_after_pass == "B", (
        "the dead-reckon advance to B must be stamped at the pass-through step"
    )
    # And the follow continued to the junction C (the re-anchor target).
    assert sim.localization.current_node() == "C"


# ===========================================================================
# TC12 -- goTo(B) where B IS the projection goal: a DISTANCE stop (no
#         pass-through), plain ACT_FOLLOW_DIST, chassis stops at B.
# ===========================================================================


def test_tc12_goto_projection_goal_stops_at_b_no_pass_through():
    """TC12: ``goTo(B)`` where ``B`` is the projection node (the goal) STOPS at
    ``B`` by distance -- the pass-through's not-final guard rejects a goal-is-B
    route.

    Even though a collinear LINE edge ``B->C`` exists beyond ``B`` (so the gate
    would otherwise see a valid C), the destination guard
    (``_nextHop == _destinationNode``) makes ``B`` a plain DISTANCE stop. The sim
    must emit the plain :data:`ACT_FOLLOW_DIST` (NO ``_PASS`` tag) and the chassis
    must stop AT ``B``.

    Layout:
        A(0,0) --LINE 0deg--> B(50,0) [projection, GOAL] --LINE 0deg--> C(120,0)
    """
    nodes = {
        "A": (0.0, 0.0),
        "B": (50.0, 0.0),
        "C": (120.0, 0.0),
    }
    edges_raw = [
        ("A", "B", EDGE_LINE, 50.0, 0.0),
        ("B", "C", EDGE_LINE, 70.0, 0.0),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", 0.0, node_types={"B": "projection"})

    prims = sim.go_to("B")
    act_tags = [tag for tag, _ in prims]

    # The single A->B hop is a plain DISTANCE stop at the projection goal.
    assert prims == [(ACT_FOLLOW_DIST, 50.0)]
    # The pass-through tag must NOT appear -- B is the goal, so there is no collapse.
    assert ACT_FOLLOW_DIST_PASS not in act_tags
    assert ACT_LINE_PASS not in act_tags

    # Chassis stopped ON B (the distance gate lands the axle on the node).
    assert sim.localization.current_node() == "B"
    bx, by = nodes["B"]
    assert math.hypot(sim.chassis.x_cm - bx, sim.chassis.y_cm - by) < _POS_TOL_CM


# ===========================================================================
# TC13 -- a BLIND-flanked projection node is a NORMAL stop (no pass-through):
#         the outbound flank is a BLIND edge, so the both-flanks-LINE gate
#         fails and the A->B leg STOPS at B (plain ACT_FOLLOW_DIST).
# ===========================================================================


def test_tc13_blind_flanked_projection_node_normal_stop():
    """TC13: a projection node ``B`` whose outbound ``B->C`` flank is a BLIND edge
    is a NORMAL stop -- no pass-through.

    The route genuinely goes A->B->C THROUGH ``B`` (no direct A->C edge), so ``B``
    IS reached, but the pass-through gate's condition "both flanks LINE" fails on
    the BLIND ``B->C`` edge. The A->B LINE-into-projection leg therefore falls back
    to the plain DISTANCE stop (:data:`ACT_FOLLOW_DIST`, NOT ``_PASS``), the
    chassis stops AT ``B``, and the onward B->C BLIND hop dead-reckons forward
    (:data:`ACT_FORWARD`).

    Layout (inbound LINE into projection, OUTBOUND flank BLIND -> normal stop):
        A(0,0) --LINE 0deg--> B(50,0) [projection] --BLIND 0deg--> C(120,0)
    """
    nodes = {
        "A": (0.0, 0.0),
        "B": (50.0, 0.0),
        "C": (120.0, 0.0),
    }
    edges_raw = [
        ("A", "B", EDGE_LINE, 50.0, 0.0),
        ("B", "C", EDGE_BLIND, 70.0, 0.0),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "A", 0.0, node_types={"B": "projection"})

    # Pin the route: it really goes A->B->C through B (the BLIND flank does not
    # divert it -- there is no direct A->C edge).
    assert sim.walked_path("C") == ["A", "B", "C"]

    prims = sim.go_to("C")
    act_tags = [tag for tag, _ in prims]

    # The A->B leg STOPPED at B: the plain DISTANCE follow, NOT the pass-through.
    assert prims[0] == (ACT_FOLLOW_DIST, 50.0)
    assert ACT_FOLLOW_DIST_PASS not in act_tags, (
        "a BLIND outbound flank must NOT pass through -- it is a normal stop at B"
    )
    assert ACT_LINE_PASS not in act_tags
    # The onward B->C BLIND hop dead-reckons forward (no auto-seek: C is a line
    # node, not a projection node, so the BLIND_PROJECTION_AUTO_SEEK rule is off).
    assert (ACT_FORWARD, 70.0) in prims
    assert ACT_STOP_AT_LINE not in act_tags

    # Localization stopped at B mid-route, then advanced to C by dead-reckoning.
    assert sim.localization.current_node() == "C"


def test_tc13_blind_inbound_into_projection_is_not_a_line_pass_through():
    """TC13 (variant): an INBOUND BLIND edge into the projection node is never a
    line-proj-line pass-through -- the LINE-into-projection dispatch (the only path
    that evaluates the projection pass-through gate) is not reached for a BLIND
    inbound.

    A ``blind(S) -> projection(B) -> line(C)`` route auto-seeks at ``B`` (re-anchor
    on the projection node's own line) and is a STOP, never a continuous follow:
    the ``_PASS`` tag must be absent. This guards the "BLIND-flanked projection
    node is a normal stop" rule from the inbound side too.

    Layout:
        S(0,0) --BLIND 0deg--> B(50,0) [projection] --LINE 0deg--> C(120,0)
    """
    nodes = {
        "S": (0.0, 0.0),
        "B": (50.0, 0.0),
        "C": (120.0, 0.0),
    }
    edges_raw = [
        ("S", "B", EDGE_BLIND, 50.0, 0.0),
        ("B", "C", EDGE_LINE, 70.0, 0.0),
    ]
    sim, _ = _make_sim(nodes, edges_raw, "S", 0.0, node_types={"S": "blind", "B": "projection"})

    prims = sim.go_to("C")
    act_tags = [tag for tag, _ in prims]

    # The S->B BLIND-into-projection hop auto-seeks (BLIND_PROJECTION_AUTO_SEEK) and
    # re-anchors on B's line -- a STOP_AT_LINE, never a continuous follow.
    assert (ACT_STOP_AT_LINE, 50.0) in prims
    # No projection pass-through (the gate is only evaluated on the LINE->projection
    # dispatch path, which a BLIND inbound never takes).
    assert ACT_FOLLOW_DIST_PASS not in act_tags
    assert ACT_LINE_PASS not in act_tags

    # The seek re-anchors at B (full-advance), then the B->C LINE hop re-anchors at
    # the junction C.
    assert sim.localization.current_node() == "C"
