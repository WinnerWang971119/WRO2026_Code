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
    forward), so the force-SAME override is load-bearing and observable: it
    holds the departure to the SAME direction as this FRONT approach, so the
    hop must emit FRONT primitives + the trim, never ``ACT_BLIND_REVERSE``.

``make_line_departing_pivot_field()`` -- CTPS issue #55 TC12
    ``Z -> P`` is BLIND (dead-reckon); ``P -> Q`` is LINE, into the junction
    line node ``Q``.  The declared pivot ``(Z, P, Q)`` (90 deg right turn,
    ``delta = 9.0`` cm) departs over a LINE edge -- the mirror image of
    ``make_line_incoming_pivot_field``, whose pivot instead ARRIVES over a
    LINE edge.  The sim takes NO departing trim on a LINE-outgoing hop (it
    re-anchors on the destination line), so the along-edge residual is exactly
    ``delta`` -- the CTPS ``track_builder`` sensor-departure re-anchor
    (ADR-044 D1) this fixture exists to exercise.  PASSES ``validate``.
    Deliberately EXCLUDED from ``all_fields()``: that list feeds the sim's
    per-turn legality sweep, which pins ``len(fields) == 6``; this fixture is
    validated directly by its own CTPS test instead.

``make_chained_sensor_departure_pivot_field()`` -- CTPS issue #55 TC13
    ``Z -> P`` is BLIND; ``P -> Q`` is LINE; ``Q -> X`` is BLIND, with pivots
    at BOTH ``P`` and ``Q`` (each a 90 deg right turn, ``delta = 9.0`` cm).
    The shared LINE leg ``P -> Q`` is pivot 1's SENSOR-terminated departure
    AND pivot 2's SENSOR-terminated approach at once: the sim takes no
    departing trim on it but shortens the landing by pivot 2's approach trim,
    so the robot physically lands ``delta`` SHORT of ``Q`` before pivot 2's
    arc.  This is the chained topology where the CTPS ``track_builder``
    sensor-departure landing must subtract the DEPARTING trim along the edge
    ray rather than snap absolutely onto ``Q`` (an absolute snap erases the
    approach trim and translates the whole tail ``delta`` off ``X``).  All
    three legs are pinned ``direction:"front"`` (T9, issue #54): pivot 2's
    sensor-terminated approach is the shared ``P -> Q`` LINE leg, and an
    unpinned reverse-reachable sensor-terminated approach is now budgeted
    against the smaller REAR offset, which this 90 deg turn's delta would
    exceed -- pinning all three is behavior-preserving (this was a pre-#54
    forward-only chain) and keeps the fixture legal.  PASSES ``validate``.
    Deliberately EXCLUDED from ``all_fields()``: that list feeds the sim's
    per-turn legality sweep, which pins ``len(fields) == 6``; this fixture is
    validated directly by its own CTPS test instead.

``make_sensor_autorule_pivot_field()`` -- TC10 / TC11 / TC12 (validator), TC6-TC9 (sim)

    ::

        A(0,0) --BLIND,heading 0--> B(60,0) [projection node] --BLIND,heading 90--> C(60,100)

    The sim "user's scenario", issue #64 node model: ``A``, ``B``, ``C`` are
    joined by plain BLIND edges and ``B`` is a ``"projection"`` node.  The
    incoming ``A -> B`` BLIND hop's OWN destination is the projection node ``B``,
    so -- with ``BLIND_PROJECTION_AUTO_SEEK=True`` -- the LOCAL
    ``is_projection_node(B)`` auto-rule makes it seek a line and classify
    ``"sensor"`` (the seek re-anchors at ``B``).  The outgoing ``B -> C`` is a
    non-seeking BLIND edge into the blind node ``C``, so it is ``"distance"``.
    The declared pivot ``(A, B, C)`` is a 90 deg turn (``delta = 9.0`` cm),
    within the 12.02 cm FRONT overshoot budget, so the sensor-incoming correction
    reduces the overshoot advance and trims the outgoing ``B -> C`` hop.  The
    100 cm outgoing edge clears the distance-side trim floor.  Both legs are
    now pinned ``direction:"front"`` via ``EDGES`` (T9, issue #54): an unpinned
    reverse-reachable sensor-terminated approach is now budgeted against the
    smaller REAR offset, which this 90 deg turn's delta would exceed, so the
    FRONT pin is required to keep the fixture legal -- behavior-preserving,
    since this was a pre-#54 forward-only pivot.  The pin is orthogonal to the
    projection-node auto-seek classification, which still fires locally off
    ``B``'s node type.  PASSES ``validate`` -- joins ``all_fields()``.

``make_sensor_authored_pivot_field()`` -- TC10 / TC11 / TC12 (validator), TC6-TC9 (sim)

    ::

        A(0,0) --BLIND,stop_at_line "on",heading 0--> B(60,0)
                                              --BLIND,heading 70--> C(94.2,93.97)

    Incoming ``A -> B`` is a BLIND hop authored via ``EDGES`` carrying
    ``stop_at_line: "on"``, so the per-edge override forces the seek and the
    incoming leg classifies ``"sensor"`` regardless of the projection auto-rule
    (every node here is ``"blind"`` -- no projection node is needed because the
    override sits higher in the precedence ladder).  The outgoing ``B -> C`` is a
    plain (non-seeking) BLIND edge, so it is ``"distance"`` and carries the
    departure trim.  The declared pivot ``(A, B, C)`` is a 70 deg turn
    (``delta ~= 6.30`` cm), within the 12.02 cm FRONT budget; the 100 cm outgoing
    edge clears the distance-side floor.  PASSES ``validate`` -- joins
    ``all_fields()``.

``over_budget_sensor_fields()`` -- TC10 / TC11 / TC12 (validator REJECT cases)

    ::

        A(0,0) --BLIND[B projection node | stop_at_line "on"],heading 0--> B(60,0)
                                          --BLIND,heading 120--> C(10,86.6)

    Two over-budget variants (an auto-rule Blind->Projection-node one and an
    authored ``stop_at_line: "on"`` one), each an obtuse 120 deg pivot
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
from tools.path_planner.planner import (
    BlindDecayParams,
    compute_all_pairs_next_hop,
    compute_pivot_compensation,
    parse_track_cm_from_config,
)
from tools.path_planner.validator import (
    parse_line_sensor_rear_axle_offset_cm,
    parse_pivot_trim_min_drive_cm_from_config,
)


# Shared field scalars. The values are arbitrary but legal -- the pivot tests
# never read them; they exist only so each fixture passes ``validate``.
_FIELD_MM: Tuple[float, float] = (3000.0, 2000.0)
_MAP_PNG: str = "maps/map.png"
_ROBOT_RADIUS_CM: float = 8.0
_BLIND_DECAY_ENABLED: bool = True
_BLIND_DECAY_K: float = 0.05
_BLIND_DECAY_EXPONENT: float = 2.0
_BLIND_DECAY_D_REF: float = 50.0


def _make_field(
    *,
    nodes: Dict[str, Tuple[float, float]],
    node_types: Dict[str, str],
    line_segments: List[Tuple[str, str]],
    blind_edges: List[Tuple[str, str]],
    pivot_turns: List[Tuple[str, str, str]],
    start_node: str,
    start_heading_deg: float,
    edges: Optional[List[dict]] = None,
    reset_nodes: Optional[Dict[str, dict]] = None,
) -> SimpleNamespace:
    """Return a full field-data ``SimpleNamespace`` for one pivot scenario.

    Exposes the complete attribute surface ``validate`` /
    ``VirtualChassis.from_field_data`` / ``PathTrackerSim`` consume, including
    ``PIVOT_TURNS``.  There are no obstacles, so ``build_graph`` keeps every
    authored edge.

    Node typing is now explicit per fixture (issue #64): the T2 validator
    rejects a ``"line"`` / ``"projection"`` ``START_NODE`` and rejects a LINE
    edge incident to a ``"blind"`` node, so a one-size ``"line"`` typing is no
    longer legal.  Each caller therefore supplies a ``node_types`` map that
    matches its topology:

    * a START node is ``"blind"`` (dead-reckon spawn, no prior line-follow);
    * a node sitting at a junction on a LINE edge is ``"line"``;
    * an on-line non-junction waypoint reached by a ``Blind->Proj`` hop is
      ``"projection"`` (the auto-seek re-anchor target -- issue #64);
    * an off-line dead-reckon node is ``"blind"``.

    :param nodes: node-name -> ``(x_cm, y_cm)`` coordinates.  Edge headings are
        derived geometrically from these by ``build_graph``.
    :param node_types: node-name -> one of ``"line"`` / ``"projection"`` /
        ``"blind"`` / ``"reset"``.  Must cover every key in ``nodes`` (the
        validator and the sim's bench-fidelity guard both require full
        coverage).
    :param line_segments: undirected LINE edges as ``(a, b)`` pairs.
    :param blind_edges: undirected BLIND edges as ``(a, b)`` pairs.
    :param pivot_turns: declared single-wheel pivots as ``(u, v, w)`` triples.
    :param start_node: ``START_NODE`` for the chassis spawn (must be a
        ``"blind"`` node -- the validator rejects an on-line start).
    :param start_heading_deg: ``START_HEADING_DEG`` in ``[0, 360)``.
    :param edges: optional rich-edge list -- each entry a dict
        ``{"type": ..., "u": ..., "v": ...}`` plus any of ``speed`` / ``mode`` /
        ``direction`` / ``stop_at_line``.  When supplied it is set verbatim as
        the ``EDGES`` attribute (``build_graph`` reads it OPTIONALLY via
        ``getattr``); when ``None`` the attribute is OMITTED entirely so legacy
        fixtures keep their exact attribute surface.
    :param reset_nodes: optional ``RESET_NODES`` sparse table (node name ->
        ``{"axis": ..., "wall_coord": ..., "facing": ...}``), for the
        wall-zero reset-node pivot fixtures (issue #54, T9). ``None`` (the
        default) yields the empty table every pre-#54 pivot fixture carries --
        RESET_NODES is already part of the strict ``field_data`` attribute
        surface (carried since the header_writer strict-read fanout), so
        making it non-empty here is a parameter, not a new attribute.
    :return: a field-data namespace ready for ``build_graph`` / ``validate`` /
        ``PathTrackerSim``.
    """
    field = SimpleNamespace(
        FIELD_MM=_FIELD_MM,
        MAP_PNG=_MAP_PNG,
        ROBOT_RADIUS_CM=_ROBOT_RADIUS_CM,
        BLIND_DECAY_ENABLED=_BLIND_DECAY_ENABLED,
        BLIND_DECAY_K=_BLIND_DECAY_K,
        BLIND_DECAY_EXPONENT=_BLIND_DECAY_EXPONENT,
        BLIND_DECAY_D_REF=_BLIND_DECAY_D_REF,
        BLIND_PROJECTION_AUTO_SEEK=True,
        START_NODE=start_node,
        START_HEADING_DEG=start_heading_deg,
        NODES={name: tuple(coord) for name, coord in nodes.items()},
        NODE_TYPES={name: node_types[name] for name in nodes},
        LINE_SEGMENTS=[tuple(seg) for seg in line_segments],
        BLIND_EDGES=[tuple(seg) for seg in blind_edges],
        OBSTACLES=[],
        PIVOT_TURNS=[tuple(turn) for turn in pivot_turns],
        # Sparse wall-zero reset table; empty on every pre-#54 pivot fixture,
        # carried so the strict header_writer read (`field_data.RESET_NODES`)
        # never AttributeErrors.
        RESET_NODES=dict(reset_nodes) if reset_nodes else {},
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
    next_hop = compute_all_pairs_next_hop(
        graph, BlindDecayParams.from_field_data(field)
    )
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
        # All edges are BLIND (dead-reckon), so every node is a blind node --
        # there is no line under the sensor anywhere in this tree, and the
        # START node U must be blind (the validator rejects an on-line start).
        node_types={name: "blind" for name in _DR_NODES},
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
#   S0(-60,0) --BLIND,heading 0--> L0(0,0) --LINE,heading 0--> L1(60,0)
#                                                  --BLIND,heading 70--> L2
#
#   Pivot (L0, L1, L2): FRONT-pinned LINE-incoming (re-anchors on the junction
#   line) + FRONT-pinned BLIND-outgoing. A 70 deg turn gives delta ~= 6.30 cm,
#   within the 12.02 cm FRONT overshoot budget, so the overshoot advance shrinks
#   to (offset - delta) and the outgoing dead-reckon hop is trimmed by delta.
#
#   Issue #64 note: the T2 validator rejects an on-line START node and rejects a
#   LINE edge incident to a blind node, so the old START=L0 (a node sitting on
#   the L0->L1 LINE edge) is no longer legal.  A dead-reckon origin S0 now feeds
#   L0 over a BLIND edge (mirroring the toy field's blind start `S`), and L0 is
#   typed "line" (it is LINE-incident via L0->L1, and the blind S0->L0 hop is a
#   legal blind approach to a line node).  The declared pivot is unchanged --
#   the route to L2 is now S0 -> L0 -> L1 -> L2 and still takes the (L0, L1, L2)
#   turn.

_LI_S0: Tuple[float, float] = (-60.0, 0.0)
_LI_L0: Tuple[float, float] = (0.0, 0.0)
_LI_L1: Tuple[float, float] = (60.0, 0.0)
_LI_OUT_CM: float = 40.0
_LI_TURN_DEG: float = 70.0
_LI_L2: Tuple[float, float] = (
    _LI_L1[0] + _LI_OUT_CM * math.cos(math.radians(_LI_TURN_DEG)),
    _LI_L1[1] + _LI_OUT_CM * math.sin(math.radians(_LI_TURN_DEG)),
)

_LI_NODES: Dict[str, Tuple[float, float]] = {
    "S0": _LI_S0,
    "L0": _LI_L0,
    "L1": _LI_L1,
    "L2": _LI_L2,
}

# S0 is the dead-reckon spawn (blind); L0/L1 are junction line nodes (L0->L1 is
# a LINE edge); L2 is the off-line dead-reckon pivot exit (BLIND L1->L2).
_LI_NODE_TYPES: Dict[str, str] = {
    "S0": "blind",
    "L0": "line",
    "L1": "line",
    "L2": "blind",
}


def make_line_incoming_pivot_field() -> SimpleNamespace:
    """Return the LINE-incoming pivot chain (TC18).

    ``S0 -> L0`` is a BLIND dead-reckon approach from the spawn (the validator
    requires a blind START node); ``L0 -> L1`` is a LINE edge (heading 0 deg);
    ``L1 -> L2`` is BLIND (heading 70 deg). Both pivot legs are FRONT-pinned so
    the sensor approach uses the 12.02 cm FRONT budget after the rear board's
    bench offset was tuned to 6.0 cm. The declared pivot ``(L0, L1, L2)`` is a
    70 deg turn, so its over-travel ``delta`` reduces the post-detect LINE
    overshoot advance and trims the outgoing dead-reckon hop.
    """
    return _make_field(
        nodes=_LI_NODES,
        node_types=_LI_NODE_TYPES,
        line_segments=[],
        blind_edges=[("S0", "L0")],
        pivot_turns=[("L0", "L1", "L2")],
        start_node="S0",          # blind spawn (validator rejects an on-line start)
        start_heading_deg=0.0,    # facing along S0->L0->L1 (the +x line)
        edges=[
            {"type": "LINE", "u": "L0", "v": "L1", "direction": "front"},
            {"type": "BLIND", "u": "L1", "v": "L2", "direction": "front"},
        ],
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
        # All four edges are BLIND (dead-reckon), so every node is a blind node,
        # and the START node U must be blind (the validator rejects an on-line
        # start).
        node_types={name: "blind" for name in _CHAIN_NODES},
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
    hop ``Q -> R`` REAR.  The force-SAME pivot override must hold the departure
    to the SAME (FRONT) direction as the approach and still fire the pivot trim
    -- the load-bearing force-SAME sim case.
    """
    return _make_field(
        nodes=_AR_NODES,
        # Both edges are BLIND (dead-reckon), so every node is a blind node, and
        # the START node P must be blind (the validator rejects an on-line
        # start).
        node_types={name: "blind" for name in _AR_NODES},
        line_segments=[],
        blind_edges=[("P", "Q"), ("Q", "R")],
        pivot_turns=[("P", "Q", "R")],
        start_node="P",
        start_heading_deg=0.0,    # facing along P->Q
    )


# ---------------------------------------------------------------------------
# Fixture 5 -- LINE-departing pivot (CTPS issue #55, TC12)
# ---------------------------------------------------------------------------
#
#   Z(0,0) --BLIND,heading 90--> P(0,60) --LINE,heading 0--> Q(80,60)
#
#   Pivot (Z, P, Q): BLIND-incoming (distance-terminated) + LINE-outgoing
#   (sensor-terminated) -- the mirror image of ``make_line_incoming_pivot_field``.
#   A 90 deg right turn gives delta = 9.0 cm. The outgoing LINE edge takes NO
#   departing trim in the sim (a LINE hop always drives its full commanded
#   distance and re-anchors via LINE_OVERSHOOT at Q), so the chassis dead-
#   reckons `delta` PAST Q along the departure edge before the destination's
#   line re-anchor corrects it -- exactly the CTPS `track_builder`
#   sensor-departure case (ADR-044 D1), unlike every other fixture above
#   (whose declared pivot's SENSOR leg, if any, is always the INCOMING one).

_LD_Z: Tuple[float, float] = (0.0, 0.0)
_LD_P: Tuple[float, float] = (0.0, 60.0)
_LD_OUT_CM: float = 80.0
_LD_Q: Tuple[float, float] = (_LD_P[0] + _LD_OUT_CM, _LD_P[1])

_LD_NODES: Dict[str, Tuple[float, float]] = {
    "Z": _LD_Z,
    "P": _LD_P,
    "Q": _LD_Q,
}

# Z is the blind dead-reckon spawn; P and Q are both junction line nodes (the
# endpoints of the outgoing P->Q LINE edge).
_LD_NODE_TYPES: Dict[str, str] = {
    "Z": "blind",
    "P": "line",
    "Q": "line",
}


def make_line_departing_pivot_field() -> SimpleNamespace:
    """Return the LINE-departing pivot chain (CTPS issue #55, TC12).

    ``Z -> P`` is a BLIND dead-reckon approach from the spawn (the validator
    requires a blind START node); ``P -> Q`` is a LINE edge (heading 0 deg)
    into the junction line node ``Q``. The declared pivot ``(Z, P, Q)`` is a
    90 deg right turn (``delta = 9.0`` cm). Unlike every other fixture in this
    module, the SENSOR-terminated leg here is the DEPARTURE (``P -> Q``), not
    the approach: the sim drives the full ``P -> Q`` distance with no trim and
    re-anchors on ``Q``'s line, so the along-edge residual the CTPS
    ``track_builder`` must re-anchor onto the destination node is exactly
    ``delta``, purely along the departure edge ray (no lateral component).
    """
    return _make_field(
        nodes=_LD_NODES,
        node_types=_LD_NODE_TYPES,
        line_segments=[("P", "Q")],
        blind_edges=[("Z", "P")],
        pivot_turns=[("Z", "P", "Q")],
        start_node="Z",            # blind spawn (validator rejects an on-line start)
        start_heading_deg=90.0,    # facing along Z->P (straight up)
    )


# ---------------------------------------------------------------------------
# Fixture 6 -- chained pivots over a LINE (sensor) middle leg (CTPS #55, TC13)
# ---------------------------------------------------------------------------
#
#   Z(0,0) --BLIND,heading 90--> P(0,60) --LINE,heading 0--> Q(80,60)
#                                            --BLIND,heading 270--> X(80,-20)
#
#   Pivots (Z, P, Q) and (P, Q, X), each a 90 deg right turn (delta = 9.0 cm).
#   The SHARED middle leg P->Q is a LINE edge, so it is pivot 1's
#   sensor-terminated DEPARTURE and pivot 2's sensor-terminated APPROACH at
#   once: the sim takes no departing trim on it but DOES shorten the landing
#   by pivot 2's approach trim, so the chassis physically lands delta short of
#   Q -- the true arc-start pose for pivot 2.  The validator's chained
#   sum-check passes (9 + 9 = 18 < 80).  An ABSOLUTE snap-to-Q of the
#   sensor-departure landing erases that approach trim and translates the
#   whole tail delta off the final node X; the relative (trim-subtraction)
#   landing this fixture exists to pin lands the tail exactly on X.

_CS_Z: Tuple[float, float] = (0.0, 0.0)
_CS_P: Tuple[float, float] = (0.0, 60.0)
_CS_MID_CM: float = 80.0
_CS_Q: Tuple[float, float] = (_CS_P[0] + _CS_MID_CM, _CS_P[1])
_CS_OUT_CM: float = 80.0
_CS_X: Tuple[float, float] = (_CS_Q[0], _CS_Q[1] - _CS_OUT_CM)

_CS_NODES: Dict[str, Tuple[float, float]] = {
    "Z": _CS_Z,
    "P": _CS_P,
    "Q": _CS_Q,
    "X": _CS_X,
}

# Z is the blind dead-reckon spawn; P and Q are junction line nodes (the
# endpoints of the middle P->Q LINE edge); X is the off-line blind exit.
_CS_NODE_TYPES: Dict[str, str] = {
    "Z": "blind",
    "P": "line",
    "Q": "line",
    "X": "blind",
}


def make_chained_sensor_departure_pivot_field() -> SimpleNamespace:
    """Return the chained pivots joined by a LINE middle leg (CTPS #55, TC13).

    ``Z -> P`` is a BLIND dead-reckon approach from the spawn (the validator
    requires a blind START node); ``P -> Q`` is a LINE edge (heading 0 deg)
    between the junction line nodes ``P`` and ``Q``; ``Q -> X`` is BLIND
    (heading 270 deg).  The declared pivots ``(Z, P, Q)`` and ``(P, Q, X)``
    are each a 90 deg right turn (``delta = 9.0`` cm), so the shared LINE leg
    is simultaneously pivot 1's SENSOR-terminated departure and pivot 2's
    SENSOR-terminated approach: the sim shortens its landing by pivot 2's
    approach trim, leaving the chassis ``delta`` SHORT of ``Q`` -- the true
    arc-start pose for pivot 2.  A sensor-departure landing that snaps
    absolutely onto ``Q`` erases that trim and translates the whole tail
    ``delta`` off ``X``; the CTPS ``track_builder`` must instead subtract the
    DEPARTING trim along the departure-edge ray.  All three legs (``Z -> P``,
    ``P -> Q``, ``Q -> X``) are pinned ``direction:"front"`` via ``EDGES``
    (T9, issue #54): pivot 2's sensor-terminated approach is the shared
    ``P -> Q`` LINE leg, and an unpinned reverse-reachable sensor-terminated
    approach is now budgeted against the smaller REAR offset (7.68 cm), which
    this turn's delta would exceed -- pinning all three (this was a pre-#54
    forward-only chain) is behavior-preserving and keeps the fixture legal.
    PASSES ``validate``.  Deliberately EXCLUDED from ``all_fields()`` (that
    sweep pins ``len(fields) == 6``); validated directly by its own CTPS test
    instead.
    """
    return _make_field(
        nodes=_CS_NODES,
        node_types=_CS_NODE_TYPES,
        line_segments=[],
        blind_edges=[],
        pivot_turns=[("Z", "P", "Q"), ("P", "Q", "X")],
        start_node="Z",            # blind spawn (validator rejects an on-line start)
        start_heading_deg=90.0,    # facing along Z->P (straight up)
        edges=[
            {"type": "BLIND", "u": "Z", "v": "P", "direction": "front"},
            {"type": "LINE", "u": "P", "v": "Q", "direction": "front"},
            {"type": "BLIND", "u": "Q", "v": "X", "direction": "front"},
        ],
    )


# ---------------------------------------------------------------------------
# Sensor-incoming pivot geometry (shared by the in-budget + over-budget fields)
# ---------------------------------------------------------------------------
#
#   A(0,0) --BLIND,heading 0--> B(_SI_IN_CM,0) --BLIND,theta--> C
#
# A and B sit on the +x axis so the incoming heading is 0 deg; C is placed at
# the requested turn angle so theta == the authored angle.  Node C is derived
# from the angle (single-sourced) -- never a hard-coded second coordinate -- so
# the delta the validator computes always matches the geometry below.
#
# Issue #64 migration: the old model put a PROJECTION *edge* on B->C.  Projection
# is now a NODE property -- so in the auto-rule field, B (not the edge) is the
# projection node and the incoming A->B BLIND hop is a `Blind->Proj` hop whose
# OWN destination is a projection node, which is what now classifies it
# "sensor" via the LOCAL is_projection_node(B) auto-rule (no goal-relative
# next_hop_is_projection peek).  Both edges are plain BLIND in both fields; only
# the node typing differs between the two.
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


def _sensor_autorule_field(
    turn_deg: float, *, direction: Optional[str] = None
) -> SimpleNamespace:
    """Return a Blind->Projection auto-rule sensor-incoming pivot field (issue #64).

    ``A -> B`` and ``B -> C`` are both plain BLIND hops, and ``B`` is a
    ``"projection"`` node.  The incoming ``A -> B`` BLIND hop's OWN destination
    is the projection node ``B``, so -- with ``BLIND_PROJECTION_AUTO_SEEK=True``
    -- the LOCAL ``is_projection_node(B)`` auto-rule classifies it ``"sensor"``
    (the seek re-anchors at ``B``).  The outgoing ``B -> C`` is a non-seeking
    BLIND edge into the blind node ``C``, so it is ``"distance"`` and carries the
    departure trim.  ``turn_deg`` sets the pivot angle -> ``delta``: <=106 deg
    fits the FRONT budget (accepted), 120 deg overflows it (rejected).

    Migration note: the old fixture authored a PROJECTION *edge* ``B -> C`` and
    relied on the retired goal-relative ``next_hop_is_projection(B, C)`` peek to
    classify the incoming hop.  The node model moves the projection onto ``B``
    itself, so the very same incoming hop classifies ``"sensor"`` from a purely
    local destination-node read -- preserving the seek-on-the-incoming-hop intent
    the TC7 sim test exercises.

    :param direction: when supplied (``"front"``/``"rear"``), both legs are
        authored via ``EDGES`` pinned to that direction instead of the plain
        ``BLIND_EDGES`` pair (T9, issue #54) -- the pin is orthogonal to the
        projection-node auto-seek classification on ``B``, which fires either
        way.  ``None`` (the default) keeps the legacy unpinned pair; the
        over-budget (120 deg) caller still uses the unpinned form.
    """
    c = _sensor_incoming_node_c(turn_deg)
    if direction is None:
        blind_edges: List[Tuple[str, str]] = [("A", "B"), ("B", "C")]
        edges: Optional[List[dict]] = None
    else:
        blind_edges = []
        edges = [
            {"type": "BLIND", "u": "A", "v": "B", "direction": direction},
            {"type": "BLIND", "u": "B", "v": "C", "direction": direction},
        ]
    return _make_field(
        nodes={"A": _SI_A, "B": _SI_B, "C": c},
        # B is the projection node (the incoming A->B Blind->Proj hop seeks +
        # re-anchors on B's line); A is the blind dead-reckon spawn; C is the
        # off-line blind pivot exit (the non-seeking B->C departure).
        node_types={"A": "blind", "B": "projection", "C": "blind"},
        line_segments=[],
        blind_edges=blind_edges,
        pivot_turns=[("A", "B", "C")],
        start_node="A",
        start_heading_deg=0.0,    # facing along A->B
        edges=edges,
    )


def _sensor_authored_field(turn_deg: float) -> SimpleNamespace:
    """Return an authored-STOP_AT_LINE sensor-incoming pivot field.

    ``A -> B`` is a BLIND hop authored via ``EDGES`` carrying
    ``stop_at_line: "on"``, so the per-edge override forces the seek and the
    incoming leg classifies ``"sensor"`` independent of the projection auto-rule
    (no projection node needed -- the override sits higher in the precedence
    ladder).  ``B -> C`` is a plain (non-seeking) BLIND edge -> ``"distance"``,
    carrying the departure trim.  ``turn_deg`` sets the pivot angle -> ``delta``.
    """
    c = _sensor_incoming_node_c(turn_deg)
    return _make_field(
        nodes={"A": _SI_A, "B": _SI_B, "C": c},
        # All BLIND, no projection node here -- the explicit stop_at_line:"on"
        # override (not the auto-rule) forces the incoming "sensor" seek. Every
        # node is a blind dead-reckon node; the START A must be blind.
        node_types={"A": "blind", "B": "blind", "C": "blind"},
        line_segments=[],
        blind_edges=[],
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
                "direction": "front",
            },
            {"type": "BLIND", "u": "B", "v": "C", "direction": "front"},
        ],
    )


# Turn magnitudes for the sensor-incoming fixtures (single-sourced; the tests
# recompute delta from these via compute_pivot_compensation).
_SI_AUTORULE_TURN_DEG: float = 90.0     # delta = 9.0 cm  (<= 12.02 FRONT budget)
_SI_AUTHORED_TURN_DEG: float = 70.0     # delta ~= 6.30 cm (<= 12.02 FRONT budget)
_SI_OVER_BUDGET_TURN_DEG: float = 120.0  # delta ~= 15.59 cm (> 12.02 -> REJECT)


def make_sensor_autorule_pivot_field() -> SimpleNamespace:
    """Return the delta<=budget Blind->Projection-node auto-rule pivot (sim user's scenario).

    Incoming ``A -> B`` BLIND into the projection node ``B``, outgoing
    ``B -> C`` plain BLIND into the blind node ``C``, declared pivot
    ``(A, B, C)`` a 90 deg turn (``delta = 9.0`` cm).  The LOCAL
    ``is_projection_node(B)`` auto-rule resolves the incoming BLIND to
    ``"sensor"`` (its destination is a projection node), so the sensor-incoming
    correction trims the overshoot advance and the 100 cm outgoing ``B -> C``
    hop.  Both legs are pinned ``direction:"front"`` via ``EDGES`` (T9, issue
    #54): this pivot was a pre-#54 forward pivot (force-FRONT was the only
    regime), and the T7 validator now budgets an unpinned reverse-reachable
    sensor-terminated approach against the smaller REAR offset (6.0 cm),
    which this turn's delta exceeds -- the FRONT pin is behavior-preserving
    and makes the always-implicit forward direction explicit.  The pin is
    orthogonal to the projection-node auto-seek classification (still driven
    locally off ``B``'s node type), so the sensor-incoming correction still
    fires.  PASSES ``validate``; joins ``all_fields()``.
    """
    return _sensor_autorule_field(_SI_AUTORULE_TURN_DEG, direction="front")


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

    :return: ``[auto_rule_variant, authored_stop_at_line_variant]`` -- the first
        is the Blind->Projection-node auto-rule case (B is a projection node),
        the second the authored ``stop_at_line: "on"`` case (all blind nodes).
    """
    return [
        _sensor_autorule_field(_SI_OVER_BUDGET_TURN_DEG),
        _sensor_authored_field(_SI_OVER_BUDGET_TURN_DEG),
    ]


# =============================================================================
# Reverse-pivot fixtures (issue #54, T9) -- see
# docs/plans/2026-07-19-issue54-reverse-pivots.md, "Testing Strategy: Test
# data". Six triples exercise the REAR-traversal single-wheel pivot the
# T1-T8 tasks add on top of the FRONT-only fixtures above:
#
#   * an authored-REAR pivot (both legs pinned `direction:"rear"`), delta
#     derived under the REAR overshoot budget minus the trim floor (AC5);
#   * an AUTO-resolved REAR-approach pivot with a genuinely >90 deg approach
#     delta, doubling as the coherent-run (force-SAME) proof (AC1, AC2);
#   * a binding-contradiction pivot (approach FRONT-pinned, departure
#     REAR-pinned) (AC3, AC7c);
#   * a reset-node (wall-zero) pivot, plus its FRONT-pinned accepted
#     counterpart (AC7b);
#   * a chained reverse-pivot pair sharing an edge, both legs REAR.
#
# None of these join `all_fields()`: that sweep pins `len(fields) == 6` and
# asserts every member PASSES `validate()` (mirroring how
# `over_budget_sensor_fields()` -- built to be REJECTED -- is excluded too).
# This set is deliberately MIXED accept/reject (unlike `all_fields()`'s
# all-pass sweep): `make_binding_contradiction_pivot_field` and
# `make_reset_node_pivot_field` are built to be REJECTED by the validator's
# reverse-pivot rules (AC3/AC7c, AC7b); `make_authored_rear_pivot_field`,
# `make_auto_rear_approach_pivot_field`, `make_reset_node_pivot_field_front_pinned`,
# and `make_chained_reverse_pivot_field` are built to be ACCEPTED. T7/T8/T10
# import the specific factory each test needs -- see `reverse_pivot_fields()`
# below for the full accept/reject matrix.
#
# Every delta below is DERIVED from the parsed firmware constants, never a
# bare literal: `_turn_deg_for_delta` inverts the ONE delta formula
# (`compute_pivot_compensation`) the planner/validator/sim all share, so a
# fixture can target a delta that is provably inside/outside a budget instead
# of hoping a hand-picked turn angle happens to land there (Notes for
# Implementer: "Symbolic budgets everywhere").
# =============================================================================

_TRACK_CM: float = parse_track_cm_from_config()
_REAR_OVERSHOOT_BUDGET_CM: float = parse_line_sensor_rear_axle_offset_cm()
_PIVOT_TRIM_FLOOR_CM: float = parse_pivot_trim_min_drive_cm_from_config()


def _turn_deg_for_delta(delta_cm: float) -> float:
    """Return the turn magnitude (degrees) whose pivot delta is ``delta_cm``.

    Inverts :func:`compute_pivot_compensation`'s closed form
    ``delta = (track/2) * tan(|theta|/2)`` -- ``theta = 2*atan(delta/(track/2))``
    -- the single-sourced inverse of the ONE delta formula the
    planner/validator/sim share, so a fixture can target a delta DERIVED from
    a parsed Config budget instead of hand-picking a turn angle and hoping the
    resulting delta happens to land where a budget test needs it.

    :param delta_cm: the desired non-negative pivot over-travel, in cm.
    :return: the unsigned turn magnitude (degrees) that yields ``delta_cm``
        with the parsed chassis track width (:data:`_TRACK_CM`).
    """
    return 2.0 * math.degrees(math.atan(delta_cm / (_TRACK_CM / 2.0)))


# ---------------------------------------------------------------------------
# Fixture 7 -- authored-REAR pivot, both legs pinned `direction:"rear"` (AC5)
# ---------------------------------------------------------------------------
#
#   S0(-40,0) --BLIND,heading 0--> L0(0,0) --LINE,"rear",heading 0--> L1(60,0)
#                          --BLIND,"rear",heading _ARV_TURN_DEG--> L2
#
#   Pivot (L0, L1, L2): the SENSOR-terminated incoming leg L0->L1 is a LINE
#   edge, explicitly pinned `direction:"rear"`; the outgoing BLIND leg L1->L2
#   is pinned `direction:"rear"` too (both legs, per the spec). The turn angle
#   is DERIVED to target a delta at HALF the (REAR budget - trim floor)
#   headroom -- comfortably inside `Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM`
#   (6.0 cm) minus `Config::PIVOT_TRIM_MIN_DRIVE_CM` (0.5 cm) -- so AC5's
#   "accept a REAR pivot with delta inside the REAR budget" case has an
#   unambiguous PASS.

_ARV_DELTA_TARGET_CM: float = (_REAR_OVERSHOOT_BUDGET_CM - _PIVOT_TRIM_FLOOR_CM) / 2.0
_ARV_TURN_DEG: float = _turn_deg_for_delta(_ARV_DELTA_TARGET_CM)

_ARV_S0: Tuple[float, float] = (-40.0, 0.0)
_ARV_L0: Tuple[float, float] = (0.0, 0.0)
_ARV_L1_CM: float = 60.0
_ARV_L1: Tuple[float, float] = (_ARV_L1_CM, 0.0)
_ARV_L2_CM: float = 40.0
_ARV_L2: Tuple[float, float] = (
    _ARV_L1[0] + _ARV_L2_CM * math.cos(math.radians(_ARV_TURN_DEG)),
    _ARV_L1[1] + _ARV_L2_CM * math.sin(math.radians(_ARV_TURN_DEG)),
)

_ARV_NODES: Dict[str, Tuple[float, float]] = {
    "S0": _ARV_S0, "L0": _ARV_L0, "L1": _ARV_L1, "L2": _ARV_L2,
}
# S0 is the dead-reckon spawn; L0/L1 are junction line nodes (L0->L1 is the
# REAR-pinned LINE edge); L2 is the off-line dead-reckon pivot exit.
_ARV_NODE_TYPES: Dict[str, str] = {
    "S0": "blind", "L0": "line", "L1": "line", "L2": "blind",
}


def make_authored_rear_pivot_field() -> SimpleNamespace:
    """Return the authored-REAR pivot: both legs pinned `direction:"rear"` (AC5).

    ``S0 -> L0`` is a BLIND dead-reckon approach from the spawn (the validator
    requires a blind START node); ``L0 -> L1`` is a LINE edge (the
    SENSOR-terminated incoming leg) pinned `direction:"rear"`; ``L1 -> L2`` is
    BLIND (the dead-reckoned outgoing leg), also pinned `direction:"rear"`.
    The declared pivot ``(L0, L1, L2)``'s delta (:data:`_ARV_DELTA_TARGET_CM`)
    is derived to sit at half the (REAR budget - trim floor) headroom,
    comfortably inside `Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM`.
    """
    return _make_field(
        nodes=_ARV_NODES,
        node_types=_ARV_NODE_TYPES,
        line_segments=[],
        blind_edges=[("S0", "L0")],
        pivot_turns=[("L0", "L1", "L2")],
        start_node="S0",           # blind spawn (validator rejects an on-line start)
        start_heading_deg=0.0,    # facing along S0->L0->L1 (the +x line)
        edges=[
            {"type": "LINE", "u": "L0", "v": "L1", "direction": "rear"},
            {"type": "BLIND", "u": "L1", "v": "L2", "direction": "rear"},
        ],
    )


# ---------------------------------------------------------------------------
# Fixture 8 -- AUTO-resolved REAR approach + coherent-run proof (AC1, AC2)
# ---------------------------------------------------------------------------
#
#   U(0,0) --BLIND,heading _RAA_ENTRY_HEADING_DEG--> V --BLIND,heading _RAA_DEPART_HEADING_DEG--> W
#
#   `START_HEADING_DEG = 0.0`, so the FIRST hop U->V resolves under
#   `BIDIRECTIONAL_AUTO` from `wrapDelta(_RAA_ENTRY_HEADING_DEG - 0.0)` --
#   `_RAA_ENTRY_HEADING_DEG` (100 deg) is STRICTLY greater than the 90 deg
#   threshold and deliberately NOT axis-aligned (an exact 90/180/270 deg entry
#   ties FRONT -- the false-green trap the plan's Notes call out), so U->V
#   genuinely resolves REAR under AUTO -- no authored direction override
#   anywhere in this field. The declared pivot (U, V, W) turns `_RAA_TURN_DEG`
#   (120 deg, itself > 90 deg -- a "sharp" AUTO pivot). Post-REAR-approach,
#   the departure V->W's OWN naive (force-SAME-free) AUTO delta would be
#   `wrap(theta - 180) = wrap(120 - 180) = -60` deg -- inside the 90 deg
#   threshold, so an UNFORCED departure would independently resolve FRONT,
#   contradicting the REAR approach. Force-SAME (T3/T4) must override that
#   independent resolution and hold the departure REAR too, proving the pivot
#   runs COHERENTLY (both legs the SAME direction) instead of degrading into a
#   mismatched turn -- the REAR-approach mirror of the pre-existing
#   `make_auto_rear_pivot_field` above (whose FRONT approach is the OTHER
#   half of TC3's "both approaches" coverage). Every edge here is
#   BLIND/non-seeking ("distance" regime both sides), so this fixture is about
#   direction RESOLUTION, not the sensor overshoot budget (that is
#   `make_authored_rear_pivot_field`'s job).

_RAA_ENTRY_HEADING_DEG: float = 100.0    # > 90 deg, NOT axis-aligned (avoids the tie-goes-FRONT trap)
_RAA_TURN_DEG: float = 120.0             # > 90 deg; naive post-REAR departure delta = wrap(120-180) = -60 (<= 90)
_RAA_IN_CM: float = 40.0
_RAA_OUT_CM: float = 40.0

_RAA_U: Tuple[float, float] = (0.0, 0.0)
_RAA_V: Tuple[float, float] = (
    _RAA_IN_CM * math.cos(math.radians(_RAA_ENTRY_HEADING_DEG)),
    _RAA_IN_CM * math.sin(math.radians(_RAA_ENTRY_HEADING_DEG)),
)
_RAA_DEPART_HEADING_DEG: float = _RAA_ENTRY_HEADING_DEG + _RAA_TURN_DEG
_RAA_W: Tuple[float, float] = (
    _RAA_V[0] + _RAA_OUT_CM * math.cos(math.radians(_RAA_DEPART_HEADING_DEG)),
    _RAA_V[1] + _RAA_OUT_CM * math.sin(math.radians(_RAA_DEPART_HEADING_DEG)),
)

_RAA_NODES: Dict[str, Tuple[float, float]] = {"U": _RAA_U, "V": _RAA_V, "W": _RAA_W}


def make_auto_rear_approach_pivot_field() -> SimpleNamespace:
    """Return the AUTO-resolved REAR-approach pivot proving coherent force-SAME (AC1, AC2).

    ``U -> V`` and ``V -> W`` are both BLIND with no authored direction
    override anywhere. `START_HEADING_DEG=0.0` makes the FIRST hop's AUTO
    delta exactly `_RAA_ENTRY_HEADING_DEG` (100 deg, strictly > the 90 deg
    threshold and deliberately not axis-aligned), so `U -> V` genuinely
    resolves REAR under `BIDIRECTIONAL_AUTO` -- exercising AC1's wheel-flip on
    an AUTO-resolved (not authored) REAR approach. The declared pivot
    ``(U, V, W)`` turns `_RAA_TURN_DEG` (120 deg): the departure `V -> W`'s
    own UNFORCED AUTO delta (`wrap(120-180) = -60`) would independently
    resolve FRONT, so force-SAME holding it REAR instead is the coherent-run
    proof (AC2) -- the REAR-approach half of TC3's "both approaches" matrix
    (the pre-existing `make_auto_rear_pivot_field` covers the FRONT-approach
    half).
    """
    return _make_field(
        nodes=_RAA_NODES,
        node_types={name: "blind" for name in _RAA_NODES},
        line_segments=[],
        blind_edges=[("U", "V"), ("V", "W")],
        pivot_turns=[("U", "V", "W")],
        start_node="U",
        start_heading_deg=0.0,
    )


# ---------------------------------------------------------------------------
# Fixture 9 -- binding-contradiction pivot: FRONT approach, REAR departure
# (AC3, AC7c)
# ---------------------------------------------------------------------------
#
#   U(0,0) --BLIND,"front",heading 0--> V(40,0) --BLIND,"rear",heading 90--> W
#
#   Both legs carry a BINDING per-edge override (step 1 of `resolveDirection`'s
#   precedence ladder -- no AUTO heuristic involved), so the contradiction is
#   STATICALLY VISIBLE: the approach U->V is pinned `direction:"front"`
#   (resolves FRONT unconditionally) while the departure V->W is pinned
#   `direction:"rear"` (resolves REAR unconditionally) -- directly opposing
#   force-SAME's rule that a pivot's departure must match its approach
#   direction. The validator must reject this offline (AC7c); the firmware
#   backstop faults `PIVOT_DIRECTION_MISMATCH` (AC3) if it is ever reached at
#   runtime via a hand-edited table.

_BC_U: Tuple[float, float] = (0.0, 0.0)
_BC_V: Tuple[float, float] = (40.0, 0.0)
_BC_OUT_CM: float = 40.0
_BC_TURN_DEG: float = 90.0
_BC_W: Tuple[float, float] = (
    _BC_V[0] + _BC_OUT_CM * math.cos(math.radians(_BC_TURN_DEG)),
    _BC_V[1] + _BC_OUT_CM * math.sin(math.radians(_BC_TURN_DEG)),
)

_BC_NODES: Dict[str, Tuple[float, float]] = {"U": _BC_U, "V": _BC_V, "W": _BC_W}


def make_binding_contradiction_pivot_field() -> SimpleNamespace:
    """Return the binding-contradiction pivot: FRONT approach, REAR departure (AC3, AC7c).

    Both pivot-adjacent legs carry a BINDING per-edge `direction` override (no
    AUTO heuristic involved): `U -> V` is pinned `direction:"front"`, `V -> W`
    is pinned `direction:"rear"` -- a contradiction visible purely from the
    authored data, which the validator must reject offline and the firmware
    backstop must fault `PIVOT_DIRECTION_MISMATCH` on if ever reached.
    """
    return _make_field(
        nodes=_BC_NODES,
        node_types={name: "blind" for name in _BC_NODES},
        line_segments=[],
        blind_edges=[],
        pivot_turns=[("U", "V", "W")],
        start_node="U",
        start_heading_deg=0.0,
        edges=[
            {"type": "BLIND", "u": "U", "v": "V", "direction": "front"},
            {"type": "BLIND", "u": "V", "v": "W", "direction": "rear"},
        ],
    )


# ---------------------------------------------------------------------------
# Fixture 9b -- builder-route contradiction pivot: per-edge FRONT approach,
# UNPINNED departure that a runtime `.setRouteDirection(REAR)` forces REAR (W1)
# ---------------------------------------------------------------------------
#
#   U(0,0) --BLIND,"front",heading 0--> V(40,0) --BLIND,(unpinned),heading 90--> W
#
#   The approach U->V is pinned `direction:"front"` (step 1 of resolveDirection,
#   binding), so the pivot COMMITS to FRONT. The departure V->W carries NO
#   authored direction override, so it is invisible to the offline validator's
#   static contradiction check -- unlike `make_binding_contradiction_pivot_field`
#   (both legs pinned). Only when a caller drives it with a RUNTIME
#   `.setRouteDirection(REAR)` (builder route, step 4) does the departure resolve
#   REAR and contradict the FRONT approach. This is the builder-rung twin of the
#   per-edge fixture: force-SAME's runtime backstop (`PIVOT_DIRECTION_MISMATCH`)
#   is the ONLY guard, since the validator never sees the builder override
#   (ADR-073 "Faults / rejects", canonizing builder route/arrival as binding).

_RC_U: Tuple[float, float] = (0.0, 0.0)
_RC_V: Tuple[float, float] = (40.0, 0.0)
_RC_OUT_CM: float = 40.0
_RC_TURN_DEG: float = 90.0
_RC_W: Tuple[float, float] = (
    _RC_V[0] + _RC_OUT_CM * math.cos(math.radians(_RC_TURN_DEG)),
    _RC_V[1] + _RC_OUT_CM * math.sin(math.radians(_RC_TURN_DEG)),
)

_RC_NODES: Dict[str, Tuple[float, float]] = {"U": _RC_U, "V": _RC_V, "W": _RC_W}


def make_route_contradiction_pivot_field() -> SimpleNamespace:
    """Return the builder-route contradiction pivot: FRONT approach, unpinned departure (W1).

    `U -> V` is pinned `direction:"front"` (a binding per-edge override), so the
    pivot `(U, V, W)` commits FRONT. `V -> W` carries NO authored override, so it
    stays statically AUTO -- the offline validator sees no contradiction. A
    runtime `go_to("W", route_direction=Direction.REAR)` then resolves the
    departure REAR (builder route, step 4), contradicting the FRONT approach, and
    the force-SAME runtime backstop must fault `PIVOT_DIRECTION_MISMATCH` -- the
    builder-rung counterpart to `make_binding_contradiction_pivot_field`'s
    statically-visible per-edge contradiction.
    """
    return _make_field(
        nodes=_RC_NODES,
        node_types={name: "blind" for name in _RC_NODES},
        line_segments=[],
        blind_edges=[],
        pivot_turns=[("U", "V", "W")],
        start_node="U",
        start_heading_deg=0.0,
        edges=[
            {"type": "BLIND", "u": "U", "v": "V", "direction": "front"},
            {"type": "BLIND", "u": "V", "v": "W"},
        ],
    )


# ---------------------------------------------------------------------------
# Fixture 10 -- reset-node (wall-zero) pivot, reject + FRONT-pinned accept
# (AC7b)
# ---------------------------------------------------------------------------
#
#   U(0,0) --BLIND,heading 90--> V(0,60) [RESET, y-wall @ 80] --BLIND,heading 0--> W(40,60)
#
#   V is typed "reset" with a `RESET_NODES["V"]` wall-zero record (a y-axis
#   wall 20 cm north of V -- clear of the 14 cm `WALL_PIVOT_CLEARANCE_CM`
#   floor; `facing=270` points south, away from the wall, so the firmware's
#   rearward wall-press is geometrically valid, mirroring `field_fixture.py`'s
#   `Wall1`). The declared pivot (U, V, W) is a 90 deg right turn
#   (delta = 9.0 cm). Neither leg carries a direction override in the base
#   (reject) variant, so on a 2-board chassis this pivot is
#   reverse-reachable: the wall-zero interlude's forward-off-the-wall
#   clearance move is sized for the FRONT sweep radius
#   (PathTracker.cpp:1409-1429), untested for a REAR departure's tail-arc
#   sweep near the wall -- T7 rejects a reverse-reachable reset-node pivot
#   unless FRONT-pinned. `make_reset_node_pivot_field_front_pinned` is the
#   SAME geometry with both legs pinned `direction:"front"` -- the accepted
#   counterpart.

_RN_U: Tuple[float, float] = (0.0, 0.0)
_RN_V: Tuple[float, float] = (0.0, 60.0)
_RN_OUT_CM: float = 40.0
_RN_W: Tuple[float, float] = (_RN_OUT_CM, 60.0)    # heading 0 from V (turn = wrap(0-90) = -90)

_RN_NODES: Dict[str, Tuple[float, float]] = {"U": _RN_U, "V": _RN_V, "W": _RN_W}
_RN_NODE_TYPES: Dict[str, str] = {"U": "blind", "V": "reset", "W": "blind"}

# The wall sits 20 cm north of V (clear of the 14 cm WALL_PIVOT_CLEARANCE_CM
# floor); facing=270 points south, away from the wall (mirrors
# field_fixture.py's Wall1: axis "y", facing 270, wall to the north).
_RN_WALL_STANDOFF_CM: float = 20.0
_RN_RESET_NODES: Dict[str, dict] = {
    "V": {
        "axis": "y",
        "wall_coord": _RN_V[1] + _RN_WALL_STANDOFF_CM,
        "facing": 270.0,
    }
}


def _reset_node_pivot_field(*, front_pinned: bool) -> SimpleNamespace:
    """Shared builder for the reset-node pivot's reject/accept pair (AC7b).

    :param front_pinned: when ``True``, both pivot-adjacent legs carry a
        binding `direction:"front"` override (the accepted counterpart);
        when ``False``, neither leg carries an override (reverse-reachable,
        the expected-reject case).
    """
    return _make_field(
        nodes=_RN_NODES,
        node_types=_RN_NODE_TYPES,
        line_segments=[],
        blind_edges=[] if front_pinned else [("U", "V"), ("V", "W")],
        pivot_turns=[("U", "V", "W")],
        start_node="U",
        start_heading_deg=90.0,    # facing along U->V (straight up)
        edges=(
            [
                {"type": "BLIND", "u": "U", "v": "V", "direction": "front"},
                {"type": "BLIND", "u": "V", "v": "W", "direction": "front"},
            ]
            if front_pinned
            else None
        ),
        reset_nodes=_RN_RESET_NODES,
    )


def make_reset_node_pivot_field() -> SimpleNamespace:
    """Return the reverse-reachable reset-node pivot (expected REJECT, AC7b).

    ``V`` (the pivot node) is typed ``"reset"`` with a wall-zero record; the
    pivot ``(U, V, W)`` (a 90 deg turn, delta = 9.0 cm) carries no direction
    override on either leg, so it is reverse-reachable on a 2-board chassis.
    The validator (T7) rejects a reverse-reachable reset-node pivot unless
    FRONT-pinned (the wall-zero clearance move is sized for the FRONT sweep
    radius, untested for a REAR tail-arc sweep near the wall) -- see
    :func:`make_reset_node_pivot_field_front_pinned` for the accepted,
    FRONT-pinned counterpart.
    """
    return _reset_node_pivot_field(front_pinned=False)


def make_reset_node_pivot_field_front_pinned() -> SimpleNamespace:
    """Return the SAME reset-node pivot with both legs pinned `direction:"front"` (accepted).

    Identical geometry to :func:`make_reset_node_pivot_field`, except both
    pivot-adjacent legs are pinned `direction:"front"` -- statically
    FRONT-pinned, so the validator (T7) accepts it despite ``V`` being typed
    ``"reset"``.
    """
    return _reset_node_pivot_field(front_pinned=True)


# ---------------------------------------------------------------------------
# Fixture 11 -- chained reverse-pivot pair (two consecutive pivots sharing an
# edge, both legs REAR)
# ---------------------------------------------------------------------------
#
#   U(0,0) --BLIND,"rear",90--> V(0,60) --BLIND,"rear",0--> W(60,60) --BLIND,"rear",-90--> X(60,0)
#
#   The REAR-mirror of `make_chained_pivot_field` above: pivots (U, V, W) and
#   (V, W, X), each a 90 deg right turn (delta = 9.0 cm), sharing the middle
#   edge V->W. All three edges are pinned `direction:"rear"`, so the shared
#   V->W edge is simultaneously pivot 1's REAR departure (force-SAME'd to
#   match its REAR approach U->V) and pivot 2's REAR approach (in turn
#   force-SAME'ing its own departure W->X to REAR) -- a two-pivot chain that
#   must run REAR end to end (wheel-flipped at both V and W). The chained
#   sum-check (`delta_uv + delta_vw < length(V->W)`) passes exactly as it does
#   in the FRONT-only original: 9 + 9 = 18 < 60.

_CR_NODES: Dict[str, Tuple[float, float]] = {
    "U": (0.0, 0.0), "V": (0.0, 60.0), "W": (60.0, 60.0), "X": (60.0, 0.0),
}


def make_chained_reverse_pivot_field() -> SimpleNamespace:
    """Return the chained REAR pivot pair `U -> V -> W -> X` (all three legs REAR).

    The REAR-mirror of :func:`make_chained_pivot_field`: identical geometry
    (two 90 deg right turns sharing the middle edge V->W), but all three
    edges are pinned `direction:"rear"`, so the pivots at V and W both run
    REAR (wheel-flipped) and the shared V->W edge is simultaneously pivot 1's
    forced-REAR departure and pivot 2's REAR approach.
    """
    return _make_field(
        nodes=_CR_NODES,
        node_types={name: "blind" for name in _CR_NODES},
        line_segments=[],
        blind_edges=[],
        pivot_turns=[("U", "V", "W"), ("V", "W", "X")],
        start_node="U",
        start_heading_deg=90.0,
        edges=[
            {"type": "BLIND", "u": "U", "v": "V", "direction": "rear"},
            {"type": "BLIND", "u": "V", "v": "W", "direction": "rear"},
            {"type": "BLIND", "u": "W", "v": "X", "direction": "rear"},
        ],
    )


def reverse_pivot_fields() -> List[SimpleNamespace]:
    """Return every reverse-pivot fixture (issue #54, T9) for smoke-import sweeps.

    Unlike :func:`all_fields`, this list is NOT asserted to uniformly pass
    `validate()` -- it is a deliberately MIXED accept/reject set:
    `make_binding_contradiction_pivot_field` and `make_reset_node_pivot_field`
    are built to be REJECTED (AC3/AC7c, AC7b); the other four (the
    authored-REAR, AUTO-REAR-approach, FRONT-pinned reset-node, and
    chained-REAR fixtures) are built to be ACCEPTED. Use the specific
    `make_*` factory for a targeted accept/reject test instead of iterating
    this list for a uniform pass/fail sweep.

    :return: all six reverse-pivot factories -- the five triple concepts plus
        the reset-node FRONT-pinned accepted counterpart.
    """
    return [
        make_authored_rear_pivot_field(),
        make_auto_rear_approach_pivot_field(),
        make_binding_contradiction_pivot_field(),
        make_reset_node_pivot_field(),
        make_reset_node_pivot_field_front_pinned(),
        make_chained_reverse_pivot_field(),
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
