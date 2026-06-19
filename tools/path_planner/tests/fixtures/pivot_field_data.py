"""Per-turn single-wheel pivot fixtures for the runtime-sim tests (ADR-036).

Exposes purpose-built synthetic fields, each carrying a ``PIVOT_TURNS`` list of
``(u, v, w)`` node-name triples, for the per-turn pivot sim parity tests
(``test_runtime_sim.py`` TC16-TC21).  Unlike the hand-rolled bidirectional
fixtures, every field here exposes the FULL field-data attribute surface
(``NODES`` / ``NODE_TYPES`` / ``LINE_SEGMENTS`` / ``BLIND_EDGES`` / ``OBSTACLES``
/ the scalars / ``PIVOT_TURNS``) so it can be run through the real
:func:`tools.path_planner.graph_builder.build_graph` +
:func:`tools.path_planner.validator.validate` pipeline -- the validator now
enforces the pivot regime / angle / budget rules (T5), so a fixture that passes
``validate`` is a legal field by construction (``test_pivot_fixtures_validate``
in the sim test module asserts exactly that).

The sim resolves each field's pivots from ``field_data.PIVOT_TURNS`` against the
graph it is constructed on (``build_pivot_turns(self._graph,
pivot_turns=self._field_data.PIVOT_TURNS)``), so the trim ``delta`` and the
pivot wheel come from the SAME geometry the validator checked.

Geometry note (single source of delta)
---------------------------------------

Every expected over-travel ``delta`` is computed by the test via
:func:`tools.path_planner.planner.compute_pivot_compensation` -- the same closed
form (``(track/2) * tan(|theta|/2)``) the planner bakes into each
:class:`PivotTurnRecord`.  The fixtures therefore never hard-code a second delta
formula; they only place nodes at coordinates whose geometric headings produce
the intended turn angle ``theta`` at each pivot node.  With the firmware track
``Config::TRACK_MM = 180 mm`` (= 18 cm):

* a 90 deg pivot gives ``delta = 9.0`` cm (dead-reckon fixtures);
* a 70 deg pivot gives ``delta ~= 6.30`` cm (LINE-incoming fixture, within the
  12.02 cm FRONT overshoot budget);
* a 120 deg pivot gives ``delta ~= 15.59`` cm (auto-REAR fixture -- the obtuse
  turn is what makes the bidirectional auto heuristic resolve the departure hop
  REAR, so the firmware/sim force-FRONT override is observable).

Fields
------

``make_dead_reckon_pivot_field()``  -- TC16 / TC17
    Tree rooted at ``U``.  ``U -> V -> W`` are BLIND (dead-reckon) edges with a
    declared pivot ``(U, V, W)`` (90 deg right turn at ``V``).  ``V`` has a
    SECOND continuation ``V -> Y`` (collinear with ``U -> V``, NOT a pivot), so a
    route ``U -> Y`` passes through the pivot node ``V`` without taking the
    declared turn -- the TC17 "fires only when the route takes that turn" case.

``make_line_incoming_pivot_field()`` -- TC18
    ``L0 -> L1 -> L2`` where ``L0 -> L1`` is a LINE edge (re-anchors on the
    junction line) and ``L1 -> L2`` is BLIND.  The declared pivot ``(L0, L1,
    L2)`` is a 70 deg turn, so a LINE-incoming pivot reduces the post-detect
    overshoot advance by ``delta`` and trims the outgoing dead-reckon hop.

``make_chained_pivot_field()``      -- TC19
    Fitting chain ``U -> V -> W -> X`` (all BLIND) with pivots at BOTH ``V`` and
    ``W`` (each a 90 deg turn).  The shared edge ``V -> W`` nets ``length -
    delta_V - delta_W``; the validator's chained sum-check passes because
    ``9 + 9 = 18 < 60``.

``make_auto_rear_pivot_field()``    -- TC20
    ``P -> Q -> R`` (all BLIND) with an obtuse 120 deg pivot ``(P, Q, R)``.  On
    the departure hop ``Q -> R`` the bidirectional auto heuristic would resolve
    REAR (the body would turn 120 deg > the 90 deg threshold to face the edge
    forward), so the FRONT-only force-override is load-bearing and observable:
    the hop must emit FRONT primitives + the trim, never ``ACT_BLIND_REVERSE``.

``make_sensor_autorule_pivot_field()`` -- TC10 / TC11 / TC12 (validator), TC6-TC9 (sim)

    ::

        A(0,0) --BLIND,heading 0--> B(60,0) --PROJECTION,heading 90--> C(60,100)

    The sim "user's scenario": incoming ``A -> B`` is a plain BLIND hop, the
    outgoing ``B -> C`` is a PROJECTION authored via ``EDGES``.  With
    ``BLIND_PROJECTION_AUTO_SEEK=True`` the incoming BLIND FEEDS a projection,
    so the auto-rule (``next_hop_is_projection(graph, B, C)``) makes it seek a
    line and classify ``"sensor"``.  The declared pivot ``(A, B, C)`` is a
    90 deg turn (``delta = 9.0`` cm), within the 12.02 cm FRONT overshoot
    budget, so the sensor-incoming correction reduces the overshoot advance and
    trims the outgoing PROJECTION hop.  The 100 cm PROJECTION edge clears the
    distance-side trim floor.  PASSES ``validate`` -- joins ``all_fields()``.

``make_sensor_authored_pivot_field()`` -- TC10 / TC11 / TC12 (validator), TC6-TC9 (sim)

    ::

        A(0,0) --BLIND,stop_at_line "on",heading 0--> B(60,0)
                                              --BLIND,heading 70--> C(94.2,93.97)

    Incoming ``A -> B`` is a BLIND hop authored via ``EDGES`` carrying
    ``stop_at_line: "on"``, so the per-edge override forces the seek and the
    incoming leg classifies ``"sensor"`` regardless of the auto-rule.  The
    outgoing ``B -> C`` is a plain (non-seeking) BLIND edge, so it is
    ``"distance"`` and carries the departure trim.  The declared pivot
    ``(A, B, C)`` is a 70 deg turn (``delta ~= 6.30`` cm), within the 12.02 cm
    FRONT budget; the 100 cm outgoing edge clears the distance-side floor.
    PASSES ``validate`` -- joins ``all_fields()``.

``over_budget_sensor_fields()`` -- TC10 / TC11 / TC12 (validator REJECT cases)

    ::

        A(0,0) --BLIND[auto-rule | stop_at_line "on"],heading 0--> B(60,0)
                                          --[PROJECTION | BLIND],heading 120--> C(10,86.6)

    Two over-budget variants (an auto-rule BLIND->PROJECTION one and an authored
    ``stop_at_line: "on"`` one), each an obtuse 120 deg pivot
    (``delta ~= 15.59`` cm > the 12.02 cm FRONT budget).  Both edges are long
    (60 cm incoming, 100 cm outgoing) so ONLY the sensor/LINE overshoot-budget
    gate trips -- never the distance-side trim floor (100 - 15.59 = 84.4 cm
    remains).  These are designed to be REJECTED by ``validate`` and are
    therefore EXCLUDED from ``all_fields()``.
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple

import networkx as nx

from tools.path_planner.graph_builder import build_graph
from tools.path_planner.planner import compute_all_pairs_next_hop


# Shared field scalars. The values are arbitrary but legal -- the pivot tests
# never read them; they exist only so each fixture passes ``validate``.
_FIELD_MM: Tuple[float, float] = (3000.0, 2000.0)
_MAP_PNG: str = "maps/map.png"
_ROBOT_RADIUS_CM: float = 8.0
_COST_BLIND_MULTIPLIER: float = 5.0


def _make_field(
    *,
    nodes: Dict[str, Tuple[float, float]],
    line_segments: List[Tuple[str, str]],
    blind_edges: List[Tuple[str, str]],
    pivot_turns: List[Tuple[str, str, str]],
    start_node: str,
    start_heading_deg: float,
    edges: Optional[List[dict]] = None,
) -> SimpleNamespace:
    """Return a full field-data ``SimpleNamespace`` for one pivot scenario.

    Exposes the complete attribute surface ``validate`` /
    ``VirtualChassis.from_field_data`` / ``PathTrackerSim`` consume, including
    ``PIVOT_TURNS``.  Every node is classified ``"line"`` (the type only gates
    visualization + the LINE/BLIND node-category split; the per-edge type lives
    on the segment lists, not the node type), and there are no obstacles, so
    ``build_graph`` keeps every authored edge.

    :param nodes: node-name -> ``(x_cm, y_cm)`` coordinates.  Edge headings are
        derived geometrically from these by ``build_graph``.
    :param line_segments: undirected LINE edges as ``(a, b)`` pairs.
    :param blind_edges: undirected BLIND edges as ``(a, b)`` pairs.
    :param pivot_turns: declared single-wheel pivots as ``(u, v, w)`` triples.
    :param start_node: ``START_NODE`` for the chassis spawn.
    :param start_heading_deg: ``START_HEADING_DEG`` in ``[0, 360)``.
    :param edges: optional rich-edge list -- each entry a dict
        ``{"type": ..., "u": ..., "v": ...}`` plus any of ``primitive_mode`` /
        ``direction`` / ``stop_at_line``.  When supplied it is set verbatim as
        the ``EDGES`` attribute (``build_graph`` reads it OPTIONALLY via
        ``getattr``); when ``None`` the attribute is OMITTED entirely so legacy
        fixtures keep their exact attribute surface.
    :return: a field-data namespace ready for ``build_graph`` / ``validate`` /
        ``PathTrackerSim``.
    """
    field = SimpleNamespace(
        FIELD_MM=_FIELD_MM,
        MAP_PNG=_MAP_PNG,
        ROBOT_RADIUS_CM=_ROBOT_RADIUS_CM,
        COST_BLIND_MULTIPLIER=_COST_BLIND_MULTIPLIER,
        BLIND_PROJECTION_AUTO_SEEK=True,
        START_NODE=start_node,
        START_HEADING_DEG=start_heading_deg,
        NODES={name: tuple(coord) for name, coord in nodes.items()},
        NODE_TYPES={name: "line" for name in nodes},
        LINE_SEGMENTS=[tuple(seg) for seg in line_segments],
        BLIND_EDGES=[tuple(seg) for seg in blind_edges],
        OBSTACLES=[],
        PIVOT_TURNS=[tuple(turn) for turn in pivot_turns],
    )
    # Only attach EDGES when authored -- build_graph reads it via getattr, so an
    # absent attribute (the legacy fixtures) is the "no rich edges" signal. A
    # dict-EDGES fixture must NOT also list the same pair in BLIND_EDGES /
    # LINE_SEGMENTS, or the edge would be emitted twice.
    if edges is not None:
        field.EDGES = [dict(edge) for edge in edges]
    return field


def build_graph_and_next_hop(field) -> Tuple[nx.DiGraph, dict]:
    """Return ``(graph, next_hop)`` for a pivot fixture field.

    Builds the directed graph (C-space inflation is a no-op here -- the fields
    carry no obstacles) and the all-pairs next-hop table the sim walks.

    :param field: a field-data namespace from one of the ``make_*`` factories.
    :return: the ``(DiGraph, NextHopTable)`` pair.
    """
    graph = build_graph(field)
    next_hop = compute_all_pairs_next_hop(graph, field.COST_BLIND_MULTIPLIER)
    return graph, next_hop


# ---------------------------------------------------------------------------
# Fixture 1 -- dead-reckon pivot + non-pivot continuation (TC16 / TC17)
# ---------------------------------------------------------------------------
#
#            Y (0,120)
#            |  BLIND (collinear with U->V; NOT a declared pivot)
#            |
#   W(60,60)-+ V(0,60)
#       BLIND |  ^
#             |  | BLIND
#             |  U(0,0)
#
#   Pivot (U, V, W): arrive U->V (heading 90 deg), pivot at V, leave V->W
#   (heading 0 deg) -> theta = -90 deg (right turn), delta = 9.0 cm.
#   Route U->W takes the pivot; route U->Y (V->Y collinear, theta 0) does not.

_DR_NODES: Dict[str, Tuple[float, float]] = {
    "U": (0.0, 0.0),
    "V": (0.0, 60.0),
    "W": (60.0, 60.0),
    "Y": (0.0, 120.0),
}


def make_dead_reckon_pivot_field() -> SimpleNamespace:
    """Return the dead-reckon pivot tree (TC16 land-on-W, TC17 non-pivot route).

    ``U -> V -> W`` are BLIND edges with the declared pivot ``(U, V, W)`` (90 deg
    right turn at ``V``).  ``V -> Y`` is a second BLIND continuation collinear
    with ``U -> V`` (turn 0 deg), so it is NOT a pivot: routing ``U -> Y`` passes
    through the pivot node ``V`` without taking the ``(U, V, W)`` turn.
    """
    return _make_field(
        nodes=_DR_NODES,
        line_segments=[],
        blind_edges=[("U", "V"), ("V", "W"), ("V", "Y")],
        pivot_turns=[("U", "V", "W")],
        start_node="U",
        start_heading_deg=90.0,    # facing along U->V (straight up)
    )


# ---------------------------------------------------------------------------
# Fixture 2 -- LINE-incoming pivot (TC18)
# ---------------------------------------------------------------------------
#
#   L0(0,0) --LINE,heading 0--> L1(60,0) --BLIND,heading 70--> L2
#
#   Pivot (L0, L1, L2): LINE-incoming (re-anchors on the junction line) +
#   BLIND-outgoing.  A 70 deg turn gives delta ~= 6.30 cm, within the 12.02 cm
#   FRONT overshoot budget, so the overshoot advance shrinks to (offset - delta)
#   and the outgoing dead-reckon hop is trimmed by delta.

_LI_L0: Tuple[float, float] = (0.0, 0.0)
_LI_L1: Tuple[float, float] = (60.0, 0.0)
_LI_OUT_CM: float = 40.0
_LI_TURN_DEG: float = 70.0
_LI_L2: Tuple[float, float] = (
    _LI_L1[0] + _LI_OUT_CM * math.cos(math.radians(_LI_TURN_DEG)),
    _LI_L1[1] + _LI_OUT_CM * math.sin(math.radians(_LI_TURN_DEG)),
)

_LI_NODES: Dict[str, Tuple[float, float]] = {
    "L0": _LI_L0,
    "L1": _LI_L1,
    "L2": _LI_L2,
}


def make_line_incoming_pivot_field() -> SimpleNamespace:
    """Return the LINE-incoming pivot chain (TC18).

    ``L0 -> L1`` is a LINE edge (heading 0 deg); ``L1 -> L2`` is BLIND (heading
    70 deg).  The declared pivot ``(L0, L1, L2)`` is a 70 deg turn, so its
    over-travel ``delta`` reduces the post-detect LINE overshoot advance and
    trims the outgoing dead-reckon hop.
    """
    return _make_field(
        nodes=_LI_NODES,
        line_segments=[("L0", "L1")],
        blind_edges=[("L1", "L2")],
        pivot_turns=[("L0", "L1", "L2")],
        start_node="L0",
        start_heading_deg=0.0,    # facing along L0->L1
    )


# ---------------------------------------------------------------------------
# Fixture 3 -- chained fitting pivots (TC19)
# ---------------------------------------------------------------------------
#
#   U(0,0) --B,90--> V(0,60) --B,0--> W(60,60) --B,-90--> X(60,0)
#
#   Pivots (U,V,W) and (V,W,X), each a 90 deg right turn (delta = 9.0 cm).
#   The shared dead-reckoned edge V->W (length 60) carries the departing trim of
#   the V-pivot AND the approaching trim of the W-pivot: 9 + 9 = 18 < 60, so the
#   validator's chained sum-check passes and V->W nets 60 - 9 - 9 = 42 cm.

_CHAIN_NODES: Dict[str, Tuple[float, float]] = {
    "U": (0.0, 0.0),
    "V": (0.0, 60.0),
    "W": (60.0, 60.0),
    "X": (60.0, 0.0),
}


def make_chained_pivot_field() -> SimpleNamespace:
    """Return the fitting chain ``U -> V -> W -> X`` with pivots at V and W (TC19).

    All four edges are BLIND.  The two declared pivots ``(U, V, W)`` and
    ``(V, W, X)`` are each a 90 deg right turn; the shared edge ``V -> W`` is
    trimmed on BOTH sides (departing V-pivot + approaching W-pivot), netting
    ``length - delta_V - delta_W``.
    """
    return _make_field(
        nodes=_CHAIN_NODES,
        line_segments=[],
        blind_edges=[("U", "V"), ("V", "W"), ("W", "X")],
        pivot_turns=[("U", "V", "W"), ("V", "W", "X")],
        start_node="U",
        start_heading_deg=90.0,    # facing along U->V
    )


# ---------------------------------------------------------------------------
# Fixture 4 -- obtuse auto-REAR pivot forced FRONT (TC20)
# ---------------------------------------------------------------------------
#
#   P(0,0) --BLIND,heading 0--> Q(40,0) --BLIND,heading 120--> R
#
#   Pivot (P, Q, R): an obtuse 120 deg turn (delta ~= 15.59 cm).  On the
#   departure hop Q->R the chassis heading after the FRONT approach is 0 deg and
#   the edge heading is 120 deg, so |delta| = 120 deg > the 90 deg bidirectional
#   threshold -> the auto heuristic would pick REAR.  The pivot force-FRONT
#   override flips it back to FRONT, so the hop emits ACT_FORWARD (trimmed),
#   never ACT_BLIND_REVERSE.

_AR_P: Tuple[float, float] = (0.0, 0.0)
_AR_Q: Tuple[float, float] = (40.0, 0.0)
_AR_OUT_CM: float = 40.0
_AR_TURN_DEG: float = 120.0
_AR_R: Tuple[float, float] = (
    _AR_Q[0] + _AR_OUT_CM * math.cos(math.radians(_AR_TURN_DEG)),
    _AR_Q[1] + _AR_OUT_CM * math.sin(math.radians(_AR_TURN_DEG)),
)

_AR_NODES: Dict[str, Tuple[float, float]] = {
    "P": _AR_P,
    "Q": _AR_Q,
    "R": _AR_R,
}


def make_auto_rear_pivot_field() -> SimpleNamespace:
    """Return the obtuse-pivot field whose departure hop auto-resolves REAR (TC20).

    ``P -> Q`` and ``Q -> R`` are BLIND.  The declared pivot ``(P, Q, R)`` is a
    120 deg turn, so the auto bidirectional heuristic would resolve the departure
    hop ``Q -> R`` REAR.  The FRONT-only pivot override must flip it FRONT and
    still fire the pivot trim -- the load-bearing FRONT-only sim case.
    """
    return _make_field(
        nodes=_AR_NODES,
        line_segments=[],
        blind_edges=[("P", "Q"), ("Q", "R")],
        pivot_turns=[("P", "Q", "R")],
        start_node="P",
        start_heading_deg=0.0,    # facing along P->Q
    )


# ---------------------------------------------------------------------------
# Sensor-incoming pivot geometry (shared by the in-budget + over-budget fields)
# ---------------------------------------------------------------------------
#
#   A(0,0) --BLIND,heading 0--> B(_SI_IN_CM,0) --[PROJECTION|BLIND],theta--> C
#
# A and B sit on the +x axis so the incoming heading is 0 deg; C is placed at
# the requested turn angle so theta == the authored angle.  Node C is derived
# from the angle (single-sourced) -- never a hard-coded second coordinate -- so
# the delta the validator computes always matches the geometry below.
_SI_A: Tuple[float, float] = (0.0, 0.0)
_SI_IN_CM: float = 60.0
_SI_B: Tuple[float, float] = (_SI_IN_CM, 0.0)
# Outgoing edge long enough that the distance-side trim floor never bites; only
# the sensor/LINE overshoot budget can gate these fixtures.
_SI_OUT_CM: float = 100.0


def _sensor_incoming_node_c(turn_deg: float) -> Tuple[float, float]:
    """Return node ``C`` placed ``_SI_OUT_CM`` from ``B`` at ``turn_deg``.

    Single source of the outgoing geometry: ``B`` is on the +x axis (incoming
    heading 0 deg), so the pivot angle ``theta`` equals ``turn_deg`` exactly and
    the validator's ``delta = (track/2)*tan(|theta|/2)`` follows from this one
    placement -- no second delta formula is hard-coded.
    """
    return (
        _SI_B[0] + _SI_OUT_CM * math.cos(math.radians(turn_deg)),
        _SI_B[1] + _SI_OUT_CM * math.sin(math.radians(turn_deg)),
    )


def _sensor_autorule_field(turn_deg: float) -> SimpleNamespace:
    """Return a BLIND->PROJECTION auto-rule sensor-incoming pivot field.

    ``A -> B`` is a plain BLIND hop (no override); ``B -> C`` is a PROJECTION
    authored via ``EDGES``.  With ``BLIND_PROJECTION_AUTO_SEEK=True`` the
    incoming BLIND FEEDS a projection, so the auto-rule classifies it
    ``"sensor"``; the outgoing PROJECTION is ``"distance"``.  ``turn_deg`` sets
    the pivot angle -> ``delta``: <=106 deg fits the FRONT budget (accepted),
    120 deg overflows it (rejected).
    """
    c = _sensor_incoming_node_c(turn_deg)
    return _make_field(
        nodes={"A": _SI_A, "B": _SI_B, "C": c},
        line_segments=[],
        blind_edges=[("A", "B")],
        pivot_turns=[("A", "B", "C")],
        start_node="A",
        start_heading_deg=0.0,    # facing along A->B
        edges=[
            {
                "type": "PROJECTION",
                "u": "B",
                "v": "C",
                "distance_cm": _SI_OUT_CM,
                "heading_deg": turn_deg,    # B->C bears at the turn angle (B on +x)
            }
        ],
    )


def _sensor_authored_field(turn_deg: float) -> SimpleNamespace:
    """Return an authored-STOP_AT_LINE sensor-incoming pivot field.

    ``A -> B`` is a BLIND hop authored via ``EDGES`` carrying
    ``stop_at_line: "on"``, so the per-edge override forces the seek and the
    incoming leg classifies ``"sensor"`` independent of the auto-rule.
    ``B -> C`` is a plain (non-seeking) BLIND edge -> ``"distance"``, carrying
    the departure trim.  ``turn_deg`` sets the pivot angle -> ``delta``.
    """
    c = _sensor_incoming_node_c(turn_deg)
    return _make_field(
        nodes={"A": _SI_A, "B": _SI_B, "C": c},
        line_segments=[],
        blind_edges=[("B", "C")],
        pivot_turns=[("A", "B", "C")],
        start_node="A",
        start_heading_deg=0.0,    # facing along A->B
        edges=[
            {
                "type": "BLIND",
                "u": "A",
                "v": "B",
                "distance_cm": _SI_IN_CM,
                "heading_deg": 0.0,    # A->B runs along +x
                "stop_at_line": "on",
            }
        ],
    )


# Turn magnitudes for the sensor-incoming fixtures (single-sourced; the tests
# recompute delta from these via compute_pivot_compensation).
_SI_AUTORULE_TURN_DEG: float = 90.0     # delta = 9.0 cm  (<= 12.02 FRONT budget)
_SI_AUTHORED_TURN_DEG: float = 70.0     # delta ~= 6.30 cm (<= 12.02 FRONT budget)
_SI_OVER_BUDGET_TURN_DEG: float = 120.0  # delta ~= 15.59 cm (> 12.02 -> REJECT)


def make_sensor_autorule_pivot_field() -> SimpleNamespace:
    """Return the delta<=budget BLIND->PROJECTION auto-rule pivot (sim user's scenario).

    Incoming ``A -> B`` BLIND, outgoing ``B -> C`` PROJECTION (authored via
    ``EDGES``), declared pivot ``(A, B, C)`` a 90 deg turn (``delta = 9.0`` cm,
    within the 12.02 cm FRONT overshoot budget).  The auto-rule resolves the
    incoming BLIND to ``"sensor"`` (it feeds a projection), so the
    sensor-incoming correction trims the overshoot advance and the 100 cm
    outgoing PROJECTION hop.  PASSES ``validate``; joins ``all_fields()``.
    """
    return _sensor_autorule_field(_SI_AUTORULE_TURN_DEG)


def make_sensor_authored_pivot_field() -> SimpleNamespace:
    """Return the delta<=budget authored-STOP_AT_LINE sensor-incoming pivot.

    Incoming ``A -> B`` BLIND with an ``EDGES`` ``stop_at_line: "on"`` override
    (forces ``"sensor"``), outgoing ``B -> C`` plain BLIND (``"distance"``),
    declared pivot ``(A, B, C)`` a 70 deg turn (``delta ~= 6.30`` cm, within the
    12.02 cm FRONT budget).  PASSES ``validate``; joins ``all_fields()``.
    """
    return _sensor_authored_field(_SI_AUTHORED_TURN_DEG)


def over_budget_sensor_fields() -> List[SimpleNamespace]:
    """Return the delta>budget sensor-incoming pivot fields (validate REJECT cases).

    Both variants are an obtuse 120 deg pivot (``delta ~= 15.59`` cm), which
    exceeds the 12.02 cm FRONT overshoot budget, so ``validate`` must REJECT
    them.  Both edges are long (60 cm incoming, 100 cm outgoing) so ONLY the
    sensor/LINE overshoot-budget gate trips -- the distance-side trim floor
    leaves ``100 - 15.59 = 84.4`` cm and never fires.

    EXCLUDED from ``all_fields()`` on purpose: the legality sweep asserts every
    ``all_fields()`` entry PASSES ``validate``, and these are built to fail.

    :return: ``[auto_rule_variant, authored_stop_at_line_variant]``.
    """
    return [
        _sensor_autorule_field(_SI_OVER_BUDGET_TURN_DEG),
        _sensor_authored_field(_SI_OVER_BUDGET_TURN_DEG),
    ]


def all_fields() -> List[SimpleNamespace]:
    """Return every pivot fixture field (for the legality / validate sweep).

    Only the PASSING fixtures belong here -- the over-budget rejection fixtures
    in :func:`over_budget_sensor_fields` are deliberately excluded so the
    ``validate`` sweep stays green.
    """
    return [
        make_dead_reckon_pivot_field(),
        make_line_incoming_pivot_field(),
        make_chained_pivot_field(),
        make_auto_rear_pivot_field(),
        make_sensor_autorule_pivot_field(),
        make_sensor_authored_pivot_field(),
    ]
