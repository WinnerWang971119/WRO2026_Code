"""Tests for the PR3 field_data schema + validator.

PR3 schema additions: BLIND_EDGES, FIELD_MM, MAP_PNG.
PR3 schema removals: PROJECTIONS, GOAL_APPROACH.
"""

from __future__ import annotations

import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.path_planner.editor.loader import load_field_json
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION, Pivot
from tools.path_planner.pipeline_config import BlindMode, LineMode
from tools.path_planner.tests.fixtures import toy_field_data as real_field_data
from tools.path_planner.validator import (
    FieldDataError,
    _validate_pivot_regimes,
    validate,
)

_STOP_AT_LINE_FIXTURE = (
    Path(__file__).resolve().parent / "fixtures" / "field_data_stop_at_line.json"
)

_FIELD_ATTRS = (
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "BLIND_EDGES",
    "OBSTACLES",
    "ROBOT_RADIUS_CM",
    "COST_BLIND_MULTIPLIER",
    "START_NODE",
    "START_HEADING_DEG",
    "FIELD_MM",
    "MAP_PNG",
)


def _clone_field_data() -> SimpleNamespace:
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


def test_valid_fixture_passes() -> None:
    assert validate(real_field_data) is None


def test_line_segments_unknown_node() -> None:
    fd = _clone_field_data()
    fd.LINE_SEGMENTS = [("A", "ZZZ")]
    with pytest.raises(FieldDataError, match=r"LINE_SEGMENTS.*unknown node"):
        validate(fd)


def test_blind_edges_unknown_node() -> None:
    fd = _clone_field_data()
    fd.BLIND_EDGES = [("A", "ZZZ")]
    with pytest.raises(FieldDataError, match=r"BLIND_EDGES.*unknown node"):
        validate(fd)


def test_blind_edges_self_loop_rejected() -> None:
    fd = _clone_field_data()
    fd.BLIND_EDGES = [("A", "A")]
    with pytest.raises(FieldDataError, match=r"self-loop"):
        validate(fd)


def test_blind_edges_duplicate_of_line_segment_rejected() -> None:
    fd = _clone_field_data()
    # ("A","B") is already a LINE_SEGMENTS entry in the placeholder field.
    fd.BLIND_EDGES = [("A", "B")] + list(fd.BLIND_EDGES)
    with pytest.raises(FieldDataError, match=r"duplicates a LINE_SEGMENTS"):
        validate(fd)


def test_obstacle_invalid_bounds() -> None:
    fd = _clone_field_data()
    fd.OBSTACLES = [(50.0, 0.0, 40.0, 10.0, "wall")]
    with pytest.raises(FieldDataError, match=r"invalid bounds"):
        validate(fd)


# -- TC4: non-finite numbers (NaN / Inf) rejected at every numeric gate -----
#
# `_is_number` is the single root gate for NODES coords, OBSTACLES bounds,
# ROBOT_RADIUS_CM, COST_BLIND_MULTIPLIER, FIELD_MM and EDGES scalars. A NaN
# is a float and not a bool, so it used to pass; worse, the obstacle bounds
# check `x_min >= x_max` is always False when an operand is NaN, so a NaN
# rectangle slipped through and reached the Euclidean planner math as a
# corrupt edge weight. Requiring finiteness makes NaN/Inf fail with a clean
# FieldDataError instead.


def test_nan_node_coord_rejected() -> None:
    fd = _clone_field_data()
    fd.NODES = dict(fd.NODES)
    fd.NODES["A"] = (float("nan"), 0.0)
    with pytest.raises(FieldDataError, match=r"NODES\['A'\]"):
        validate(fd)


def test_inf_node_coord_rejected() -> None:
    fd = _clone_field_data()
    fd.NODES = dict(fd.NODES)
    fd.NODES["A"] = (0.0, float("inf"))
    with pytest.raises(FieldDataError, match=r"NODES\['A'\]"):
        validate(fd)


def test_nan_obstacle_bound_rejected() -> None:
    fd = _clone_field_data()
    fd.OBSTACLES = [(float("nan"), 0.0, 40.0, 10.0, "wall")]
    with pytest.raises(FieldDataError, match=r"OBSTACLES\[0\]"):
        validate(fd)


def test_inf_obstacle_bound_rejected() -> None:
    fd = _clone_field_data()
    fd.OBSTACLES = [(0.0, 0.0, float("inf"), 10.0, "wall")]
    with pytest.raises(FieldDataError, match=r"OBSTACLES\[0\]"):
        validate(fd)


# -- Finding #49: node names must survive emission into a C++ string literal --
#
# header_writer._node_row builds NodeInfo.name as `'"' + name + '"'` with no
# escaping. A double-quote, backslash, or non-printable/control char produces a
# malformed literal that fails the Arduino build on the auto-generated
# routes.cpp. The validator rejects such names so the error surfaces at the
# field-data layer instead.


def test_node_name_with_double_quote_rejected() -> None:
    fd = _clone_field_data()
    fd.NODES = dict(fd.NODES)
    fd.NODE_TYPES = dict(fd.NODE_TYPES)
    fd.NODES['A"B'] = (0.0, 0.0)
    fd.NODE_TYPES['A"B'] = "line"
    with pytest.raises(FieldDataError, match=r"disallowed character"):
        validate(fd)


def test_node_name_with_backslash_rejected() -> None:
    fd = _clone_field_data()
    fd.NODES = dict(fd.NODES)
    fd.NODE_TYPES = dict(fd.NODE_TYPES)
    fd.NODES["A\\B"] = (0.0, 0.0)
    fd.NODE_TYPES["A\\B"] = "line"
    with pytest.raises(FieldDataError, match=r"disallowed character"):
        validate(fd)


def test_node_name_with_non_printable_char_rejected() -> None:
    fd = _clone_field_data()
    fd.NODES = dict(fd.NODES)
    fd.NODE_TYPES = dict(fd.NODE_TYPES)
    fd.NODES["A\nB"] = (0.0, 0.0)
    fd.NODE_TYPES["A\nB"] = "line"
    with pytest.raises(FieldDataError, match=r"disallowed character"):
        validate(fd)


def test_ordinary_node_name_accepted() -> None:
    """A plain letters/digits/underscores name still validates."""
    fd = _clone_field_data()
    fd.NODES = dict(fd.NODES)
    fd.NODE_TYPES = dict(fd.NODE_TYPES)
    fd.NODES["Node_1"] = (0.0, 0.0)
    fd.NODE_TYPES["Node_1"] = "line"
    assert validate(fd) is None


def test_node_type_unknown_value() -> None:
    fd = _clone_field_data()
    fd.NODE_TYPES["A"] = "weird"
    with pytest.raises(FieldDataError, match=r"NODE_TYPES\['A'\]"):
        validate(fd)


def test_field_mm_must_be_pair_of_positives() -> None:
    fd = _clone_field_data()
    fd.FIELD_MM = (3000,)
    with pytest.raises(FieldDataError, match=r"FIELD_MM"):
        validate(fd)
    fd.FIELD_MM = (0, 100)
    with pytest.raises(FieldDataError, match=r"FIELD_MM"):
        validate(fd)


def test_map_png_must_be_non_empty_string() -> None:
    fd = _clone_field_data()
    fd.MAP_PNG = ""
    with pytest.raises(FieldDataError, match=r"MAP_PNG"):
        validate(fd)
    fd.MAP_PNG = 42
    with pytest.raises(FieldDataError, match=r"MAP_PNG"):
        validate(fd)


def test_missing_required_attr_rejected() -> None:
    fd = _clone_field_data()
    delattr(fd, "BLIND_EDGES")
    with pytest.raises(FieldDataError, match=r"missing required attribute: BLIND_EDGES"):
        validate(fd)


def test_validator_rejects_invalid_direction_value() -> None:
    """TC16: EDGES `direction` accepts only 'front' or 'rear' (or absent)."""
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": "LINE",
            "u": "A",
            "v": "B",
            "distance_cm": 100.0,
            "heading_deg": 0.0,
            "direction": "backwards",
        }
    ]
    with pytest.raises(FieldDataError, match="direction"):
        validate(fd)


# -- TC4: STOP_AT_LINE blind-mode override ----------------------------------
#
# A per-edge BLIND override of STOP_AT_LINE must validate; an override of the
# wrong enum type (or an unknown string) must still be rejected. The schema
# spells the per-edge primitive override as ``primitive_mode`` carrying a
# :class:`BlindMode` member, so a STOP_AT_LINE override is
# ``primitive_mode=BlindMode.STOP_AT_LINE`` on a BLIND edge.


def test_validator_accepts_stop_at_line_blind_override() -> None:
    """TC4: an EDGES BLIND edge with primitive_mode=BlindMode.STOP_AT_LINE validates."""
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_BLIND,
            "u": "B",
            "v": "G",
            "distance_cm": 86.0,
            "heading_deg": 54.46,
            "primitive_mode": BlindMode.STOP_AT_LINE,
        }
    ]
    assert validate(fd) is None


def test_validator_rejects_unknown_blind_mode() -> None:
    """TC4 (reject): an unknown/wrong-type blind mode override is still rejected.

    DEFAULT is the "absent" sentinel and a string is not a BlindMode, so both
    must fail with a precise ``primitive_mode`` diagnostic. This proves the
    accept above is not a blanket pass on any value.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_BLIND,
            "u": "B",
            "v": "G",
            "distance_cm": 86.0,
            "heading_deg": 54.46,
            "primitive_mode": "STOP_AT_LINE",  # a raw string, not a BlindMode
        }
    ]
    with pytest.raises(FieldDataError, match="primitive_mode"):
        validate(fd)


def test_validator_rejects_line_mode_on_stop_at_line_blind_edge() -> None:
    """TC4 (cross-type): a LineMode override on a BLIND edge is rejected.

    The mode enum must match the edge type. A ``LineMode`` on a BLIND edge --
    even a concrete one -- is a category error and must fail rather than be
    silently coerced toward STOP_AT_LINE.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_BLIND,
            "u": "B",
            "v": "G",
            "distance_cm": 86.0,
            "heading_deg": 54.46,
            "primitive_mode": LineMode.FAST,
        }
    ]
    with pytest.raises(FieldDataError, match="BlindMode"):
        validate(fd)


# -- TC5: EDGES vs LINE_SEGMENTS / BLIND_EDGES anti-ambiguity cross-check ----
#
# Findings #38/#47: an EDGES entry whose unordered pair {u, v} already exists
# in LINE_SEGMENTS or BLIND_EDGES with a conflicting type silently overwrites
# the earlier edge's type and geometry in graph_builder (LINE then BLIND then
# EDGES; networkx add_edge replaces attributes). The validator must reject the
# conflict, while still accepting a same-type override.


def test_validator_rejects_edge_blind_over_line_segment_pair() -> None:
    """TC5: EDGE_BLIND on a pair already in LINE_SEGMENTS is rejected.

    ('A','B') is a LINE_SEGMENTS pair in the toy field. Re-declaring it as a
    BLIND edge via EDGES would silently flip the line edge's type/geometry in
    graph_builder -- the exact corruption the LINE-vs-BLIND guard prevents,
    bypassed through EDGES.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_BLIND,
            "u": "A",
            "v": "B",
            "distance_cm": 100.0,
            "heading_deg": 0.0,
        }
    ]
    with pytest.raises(FieldDataError, match=r"LINE_SEGMENTS"):
        validate(fd)


def test_validator_rejects_edge_line_over_blind_edge_pair() -> None:
    """TC5 (mirror): EDGE_LINE on a pair already in BLIND_EDGES is rejected.

    ('B','G') is the toy field's single BLIND_EDGES pair. Re-declaring it as a
    LINE edge via EDGES is the symmetric conflict and must fail too.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "B",
            "v": "G",
            "distance_cm": 86.0,
            "heading_deg": 54.46,
        }
    ]
    with pytest.raises(FieldDataError, match=r"BLIND_EDGES"):
        validate(fd)


def test_validator_rejects_edge_projection_over_line_segment_pair() -> None:
    """TC5: EDGE_PROJECTION on a LINE_SEGMENTS pair is a conflict.

    EDGE_PROJECTION maps to neither LINE_SEGMENTS nor BLIND_EDGES, so a
    PROJECTION entry on an existing line pair still reclassifies the edge and
    must be rejected. ('B','C') is a LINE_SEGMENTS pair in the toy field.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_PROJECTION,
            "u": "B",
            "v": "C",
            "distance_cm": 100.0,
            "heading_deg": 0.0,
        }
    ]
    with pytest.raises(FieldDataError, match=r"LINE_SEGMENTS"):
        validate(fd)


def test_validator_accepts_same_type_line_override() -> None:
    """TC5 (positive): EDGE_LINE on a LINE_SEGMENTS pair is a valid override.

    The intended use of a rich EDGES entry is as an override layer attaching
    distance/heading/pivot to an existing edge. A matching-type entry on an
    existing pair must NOT be rejected by the anti-ambiguity rule.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "B",
            "distance_cm": 100.0,
            "heading_deg": 0.0,
            "primitive_mode": LineMode.FAST,
        }
    ]
    assert validate(fd) is None


def test_validator_accepts_same_type_blind_override() -> None:
    """TC5 (positive, mirror): EDGE_BLIND on a BLIND_EDGES pair is accepted.

    ('B','G') is the toy field's BLIND_EDGES pair; a matching-type BLIND
    override is the legitimate path the STOP_AT_LINE edge relies on.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_BLIND,
            "u": "B",
            "v": "G",
            "distance_cm": 86.0,
            "heading_deg": 54.46,
            "primitive_mode": BlindMode.STOP_AT_LINE,
        }
    ]
    assert validate(fd) is None


def test_stop_at_line_fixture_loads_and_validates() -> None:
    """TC4: the committed STOP_AT_LINE fixture field loads and passes validation.

    The fixture authors the stop-at-line BLIND edge in the on-disk JSON
    schema. Loading it through the editor loader and validating the result
    must succeed, so the fixture stays a usable, schema-clean exemplar of a
    field carrying a STOP_AT_LINE edge.
    """
    data = load_field_json(_STOP_AT_LINE_FIXTURE)
    fd = SimpleNamespace(**data)
    assert validate(fd) is None
    # The fixture carries exactly the one stop-at-line BLIND edge.
    assert any(
        edge.get("blind_mode") == "STOP_AT_LINE" for edge in data["EDGES"]
    )


# -- Finding #36: pivot out-degree check moved into the validator ------------
#
# A single-wheel pivot entering 'v' is compensated by shortening 'v's ONE
# forward continuation. The planner cannot pick among several before Dijkstra
# runs, so it requires 'v' to have exactly one forward out-edge (excluding the
# back-edge to the pivot's source 'u'). This used to fail late as a raw
# ValueError from planner.apply_pivot_compensations (an uncaught traceback at
# build time); finding #36 (decision B) moved it into the validator so a bad
# pivot fails as a clean FieldDataError. The count is over the AUTHORED
# adjacency, before obstacle / C-space inflation clipping.


def test_validator_rejects_pivot_with_multiple_forward_out_edges() -> None:
    """A pivot edge whose destination has >1 forward out-edge is rejected.

    In the toy field, node 'E' is line-connected to A, B, C, and D. A pivot
    edge A->E lands at E, whose authored out-neighbors are {A, B, C, D};
    excluding the back-edge to the source 'A' leaves {B, C, D} -- three
    forward continuations, so the pivot landing is ambiguous and must fail.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "E",
            "distance_cm": 100.0,
            "heading_deg": 0.0,
            "pivot": Pivot.LEFT,
        }
    ]
    with pytest.raises(FieldDataError, match=r"exactly one\s+forward out-edge"):
        validate(fd)


def test_validator_rejects_pivot_with_zero_forward_out_edges() -> None:
    """A pivot edge whose destination has no forward out-edge is rejected.

    Node 'G' is a leaf: its only authored neighbor is 'B' (the single
    BLIND_EDGES pair). A pivot edge B->G lands at G, whose out-neighbors are
    {B}; excluding the back-edge to the source 'B' leaves an empty set --
    zero forward continuations, which is equally ambiguous and must fail.
    The entry is EDGE_BLIND so it is a legitimate same-type override of the
    existing ('B','G') blind pair (it is the pivot count, not the type
    cross-check, that rejects it).
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_BLIND,
            "u": "B",
            "v": "G",
            "distance_cm": 86.0,
            "heading_deg": 54.46,
            "pivot": Pivot.RIGHT,
        }
    ]
    with pytest.raises(FieldDataError, match=r"exactly one\s+forward out-edge"):
        validate(fd)


def test_validator_accepts_pivot_with_single_forward_out_edge() -> None:
    """A pivot landing with exactly one forward out-edge validates.

    Self-contained A->B->C chain: the pivot edge A->B lands at B, whose
    authored out-neighbors are {A, C}; excluding the back-edge to the source
    'A' leaves {C} -- exactly one forward continuation, so the pivot landing
    is unambiguous and validation passes. This proves the rejection cases
    above are not a blanket failure on any pivot tag.
    """
    fd = SimpleNamespace(
        NODES={"A": (0.0, 0.0), "B": (100.0, 0.0), "C": (100.0, 100.0)},
        NODE_TYPES={"A": "line", "B": "blind", "C": "blind"},
        LINE_SEGMENTS=[],
        BLIND_EDGES=[],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        COST_BLIND_MULTIPLIER=5.0,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        EDGES=[
            {
                "type": EDGE_BLIND,
                "u": "A",
                "v": "B",
                "distance_cm": 100.0,
                "heading_deg": 0.0,
                "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND,
                "u": "B",
                "v": "C",
                "distance_cm": 100.0,
                "heading_deg": 90.0,
            },
        ],
    )
    assert validate(fd) is None


def test_malformed_later_edge_does_not_mask_with_pivot_error() -> None:
    """A malformed LATER entry must fail with its own shape error, not a pivot one.

    The pivot out-degree check (finding #36) scans the full authored EDGES
    list, including entries that the per-entry shape loop has not reached
    yet. Before the fix, a later entry with ``u`` set but ``v`` missing
    contributed a ``None`` out-neighbor to an EARLIER pivot landing,
    inflating its forward out-degree to 2 and raising a misleading
    "exactly one forward out-edge" error instead of the precise
    "missing required key 'v'" error for the malformed entry.

    Fixture: A->B is a valid LEFT pivot landing at B (forward out = {C},
    exactly one), B->C is the single continuation, and a third entry sets
    ``u='B'`` but omits ``v``. The validator must blame EDGES[2], not the
    pivot at EDGES[0].
    """
    fd = SimpleNamespace(
        NODES={"A": (0.0, 0.0), "B": (100.0, 0.0), "C": (100.0, 100.0)},
        NODE_TYPES={"A": "line", "B": "blind", "C": "blind"},
        LINE_SEGMENTS=[],
        BLIND_EDGES=[],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        COST_BLIND_MULTIPLIER=5.0,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        EDGES=[
            {
                "type": EDGE_BLIND, "u": "A", "v": "B",
                "distance_cm": 100.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND, "u": "B", "v": "C",
                "distance_cm": 100.0, "heading_deg": 90.0,
            },
            {  # malformed: missing 'v'; must be blamed, not the pivot above
                "type": EDGE_BLIND, "u": "B",
                "distance_cm": 50.0, "heading_deg": 45.0,
            },
        ],
    )
    with pytest.raises(FieldDataError, match=r"EDGES\[2\] missing required key 'v'"):
        validate(fd)


# -- TC2 (issue #44): two-sided pivot over-travel geometry fail-fast ----------
#
# `validator._validate_pivot_regimes` classifies every LEFT/RIGHT pivot edge by
# its INCOMING regime (the pivot edge `u->v`) x OUTGOING regime (the unique
# forward continuation `v->w`, guaranteed unique by finding #36), computes the
# single-wheel over-travel delta = (track/2)*tan(|theta|/2) with the live track
# (Config::TRACK_MM = 180 mm -> 18.0 cm), and raises a precise FieldDataError
# on geometry the chassis cannot realise. A regime is SENSOR-terminated (LINE,
# or BLIND with primitive_mode == BlindMode.STOP_AT_LINE) -- it re-anchors on
# the physical line and self-corrects -- or DISTANCE-terminated (plain BLIND /
# PROJECTION) -- dead-reckoned, where the over-travel trim actually bites.
#
# Reference geometry at track = 18.0 cm:
#   theta = 90 deg  -> delta = 9.000 cm  (> 7.68 budget; > any edge < 9 cm)
#   theta = 10 deg  -> delta = 0.787 cm  (well under budget and short edges)
#   the LINE overshoot budget delta = 7.68 cm is reached at theta ~= 80.95 deg.
#
# All fixtures follow the self-contained A->B->C chain of
# `test_validator_accepts_pivot_with_single_forward_out_edge`: a pivot edge
# A->B lands at B whose only forward continuation is C, so finding #36's
# single-forward-out-edge precondition is satisfied and the SPECIFIC
# pivot-regime error under test is the one that raises.


def _pivot_field(edges) -> SimpleNamespace:
    """Build a minimal A/B/C field carrying the given EDGES list.

    Mirrors the fixture shape of
    :func:`test_validator_accepts_pivot_with_single_forward_out_edge`: three
    nodes, no LINE_SEGMENTS / BLIND_EDGES (so all adjacency and geometry come
    from the explicit EDGES entries), and a clean A->B->C pivot chain. Node
    coordinates are inert for the pivot-regime pass -- it reads the explicit
    ``heading_deg`` / ``distance_cm`` / ``type`` off each EDGES entry -- so the
    turn angle and over-travel are fully determined by the entries, not the
    geometry.
    """
    return SimpleNamespace(
        NODES={"A": (0.0, 0.0), "B": (100.0, 0.0), "C": (100.0, 100.0)},
        NODE_TYPES={"A": "line", "B": "blind", "C": "blind"},
        LINE_SEGMENTS=[],
        BLIND_EDGES=[],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        COST_BLIND_MULTIPLIER=5.0,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        EDGES=edges,
    )


def test_pivot_regime_rejects_degenerate_180_turn() -> None:
    """TC2: a pivot whose in/out headings differ by ~180 deg is rejected.

    Incoming A->B heads 0 deg; outgoing B->C heads 180 deg, so the turn
    normalizes to theta = -180 deg -- a near-full one-wheel reversal where
    (track/2)*tan(theta/2) diverges. Both edges are long (100 cm), so only the
    degenerate-angle gate can trip.
    """
    fd = _pivot_field(
        [
            {
                "type": EDGE_BLIND, "u": "A", "v": "B",
                "distance_cm": 100.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND, "u": "B", "v": "C",
                "distance_cm": 100.0, "heading_deg": 180.0,
            },
        ]
    )
    with pytest.raises(FieldDataError, match=r"degenerate turn\s+angle"):
        validate(fd)


def test_pivot_regime_rejects_degenerate_zero_turn() -> None:
    """TC2: a pivot with a ~0 deg turn is rejected (no real turn -> no pivot).

    Incoming A->B heads 0 deg; outgoing B->C heads 0.5 deg, so theta = 0.5 deg
    is below the 1.0 deg minimum -- the "pivot" does not turn, which is an
    authoring mistake, not a no-op. Long edges isolate the angle gate.
    """
    fd = _pivot_field(
        [
            {
                "type": EDGE_BLIND, "u": "A", "v": "B",
                "distance_cm": 100.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND, "u": "B", "v": "C",
                "distance_cm": 100.0, "heading_deg": 0.5,
            },
        ]
    )
    with pytest.raises(FieldDataError, match=r"degenerate turn\s+angle"):
        validate(fd)


def test_pivot_regime_rejects_delta_exceeds_outgoing_dead_reckon() -> None:
    """TC2: delta >= the OUTGOING dead-reckoned edge length is rejected.

    A 90 deg pivot (delta = 9.0 cm at track 18) whose outgoing continuation is
    a plain BLIND edge only 2.0 cm long. The out trim cannot meet or exceed the
    drive it shortens, so the validator names the OUTGOING side. The incoming
    edge is long (100 cm) so the incoming check passes first.
    """
    fd = _pivot_field(
        [
            {
                "type": EDGE_BLIND, "u": "A", "v": "B",
                "distance_cm": 100.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND, "u": "B", "v": "C",
                "distance_cm": 2.0, "heading_deg": 90.0,
            },
        ]
    )
    with pytest.raises(
        FieldDataError, match=r">= the OUTGOING dead-reckoned edge length"
    ):
        validate(fd)


def test_pivot_regime_accepts_long_outgoing_dead_reckon() -> None:
    """TC2 (pass counterpart): a long-enough outgoing dead-reckon edge validates.

    Same 90 deg / delta = 9.0 cm pivot as the rejection above, but the outgoing
    BLIND continuation is now 100 cm -- comfortably longer than the 9.0 cm trim
    -- and the incoming BLIND edge is 100 cm too, so neither dead-reckon gate
    nor (BLIND incoming, no overshoot budget) trips. Proves the rejection is
    about the short length, not the 90 deg turn itself.
    """
    fd = _pivot_field(
        [
            {
                "type": EDGE_BLIND, "u": "A", "v": "B",
                "distance_cm": 100.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND, "u": "B", "v": "C",
                "distance_cm": 100.0, "heading_deg": 90.0,
            },
        ]
    )
    assert validate(fd) is None


def test_pivot_regime_rejects_delta_exceeds_incoming_dead_reckon() -> None:
    """TC2: delta >= the INCOMING dead-reckoned edge length is rejected.

    A 90 deg pivot (delta = 9.0 cm) whose INCOMING dead-reckoned edge A->B is
    only 2.0 cm long. The approach trim cannot meet or exceed the drive it
    shortens, so the validator names the INCOMING side. The outgoing edge is
    long (100 cm), so the incoming gate is the one that trips.
    """
    fd = _pivot_field(
        [
            {
                "type": EDGE_BLIND, "u": "A", "v": "B",
                "distance_cm": 2.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND, "u": "B", "v": "C",
                "distance_cm": 100.0, "heading_deg": 90.0,
            },
        ]
    )
    with pytest.raises(
        FieldDataError, match=r">= the INCOMING dead-reckoned edge length"
    ):
        validate(fd)


def test_pivot_regime_rejects_delta_exceeds_line_overshoot_budget() -> None:
    """TC2: a LINE-incoming pivot whose delta exceeds the 7.68 cm budget is rejected.

    A LINE incoming edge realises its pivot correction as a reduction of the
    post-detect LINE_OVERSHOOT advance, valid only while delta stays within the
    7.68 cm budget (the min of the front 12.02 / rear 7.68 sensor-to-axle
    offsets). A 90 deg turn gives delta = 9.0 cm > 7.68 cm, so it is rejected.
    Both edges are long, so only the overshoot-budget gate trips.
    """
    fd = _pivot_field(
        [
            {
                "type": EDGE_LINE, "u": "A", "v": "B",
                "distance_cm": 100.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND, "u": "B", "v": "C",
                "distance_cm": 100.0, "heading_deg": 90.0,
            },
        ]
    )
    with pytest.raises(
        FieldDataError, match=r"exceeds the LINE overshoot budget"
    ):
        validate(fd)


def test_pivot_regime_accepts_line_incoming_within_overshoot_budget() -> None:
    """TC2 (pass counterpart): a shallow LINE-incoming pivot stays within budget.

    Same LINE incoming edge, but a shallow 10 deg turn gives
    delta = 0.787 cm <= 7.68 cm, so the overshoot-budget gate does not trip and
    the outgoing BLIND edge (100 cm) easily clears its dead-reckon gate. Proves
    the budget rejection is about the over-large delta, not the LINE incoming
    edge per se.
    """
    fd = _pivot_field(
        [
            {
                "type": EDGE_LINE, "u": "A", "v": "B",
                "distance_cm": 100.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND, "u": "B", "v": "C",
                "distance_cm": 100.0, "heading_deg": 10.0,
            },
        ]
    )
    assert validate(fd) is None


def test_pivot_regime_exempts_outgoing_stop_at_line() -> None:
    """TC2 (AC7 regression guard): a STOP_AT_LINE outgoing continuation is EXEMPT.

    The most important regression pair. A 90 deg pivot (delta = 9.0 cm) whose
    outgoing BLIND continuation is only 2.0 cm long -- shorter than the trim --
    would be rejected as a too-short dead-reckon (see
    ``test_pivot_regime_rejects_delta_exceeds_outgoing_dead_reckon``). But here
    that continuation carries primitive_mode = BlindMode.STOP_AT_LINE, making it
    SENSOR-terminated: it self-corrects on the physical line and takes NO
    outgoing trim, so the delta >= out_distance gate MUST NOT fire.

    This is the AC7 guard the prior review required: the outgoing side's
    primitive_mode is in scope and a STOP_AT_LINE continuation is exempt. The
    contrast with the plain-BLIND rejection above is the load-bearing assertion.
    """
    fd = _pivot_field(
        [
            {
                "type": EDGE_BLIND, "u": "A", "v": "B",
                "distance_cm": 100.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND, "u": "B", "v": "C",
                "distance_cm": 2.0, "heading_deg": 90.0,
                "primitive_mode": BlindMode.STOP_AT_LINE,
            },
        ]
    )
    assert validate(fd) is None


def test_pivot_regime_exempts_outgoing_stop_at_line_on_reverse_leg() -> None:
    """TC2 (AC7, reverse leg): STOP_AT_LINE authored on w->v still exempts v->w.

    primitive_mode is a bidirectional edge attribute in graph_builder, so a
    STOP_AT_LINE override authored on the REVERSE leg C->B must still classify
    the forward continuation B->C as SENSOR-terminated. The reverse leg
    C->B is authored heading 270 deg, so the forward B->C resolves to
    270 + 180 = 90 deg -- the same 90 deg pivot whose delta = 9.0 cm exceeds the
    2.0 cm length, yet the STOP_AT_LINE exemption applies and validation passes.
    """
    fd = _pivot_field(
        [
            {
                "type": EDGE_BLIND, "u": "A", "v": "B",
                "distance_cm": 100.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
            },
            {
                "type": EDGE_BLIND, "u": "C", "v": "B",
                "distance_cm": 2.0, "heading_deg": 270.0,
                "primitive_mode": BlindMode.STOP_AT_LINE,
            },
        ]
    )
    assert validate(fd) is None


def test_pivot_regime_unsupported_combo_is_defensive_depth() -> None:
    """TC2: the unsupported-regime-combo branch is unreachable via validate().

    The branch raises when either side's edge type classifies as neither
    sensor- nor distance-terminated. But every type that survives the per-entry
    shape check in ``_validate_edges_field`` is one of EDGE_LINE / EDGE_BLIND /
    EDGE_PROJECTION, and all three classify (LINE -> sensor, PROJECTION ->
    distance, BLIND -> sensor/distance). A bogus type is rejected by the shape
    check BEFORE the pivot-regime pass ever runs, so the public ``validate()``
    can never reach this branch -- it is defensive depth.

    Rather than fabricate a fixture that cannot trigger the path through the
    public API, this test asserts the branch directly by calling the internal
    ``_validate_pivot_regimes`` pass on a fixture whose outgoing edge carries an
    unclassifiable type -- the only honest route to the defensive code. The
    companion assertion confirms the public ``validate()`` rejects the same
    fixture EARLIER, at the shape check, naming ``EDGES[1].type`` (not the
    regime combo), which is why the branch is unreachable end to end.
    """
    edges = [
        {
            "type": EDGE_BLIND, "u": "A", "v": "B",
            "distance_cm": 100.0, "heading_deg": 0.0, "pivot": Pivot.LEFT,
        },
        {  # unclassifiable outgoing type -- bypasses the shape check only
            # because we call the internal pass directly below.
            "type": "WEIRD", "u": "B", "v": "C",
            "distance_cm": 100.0, "heading_deg": 90.0,
        },
    ]
    fd = _pivot_field(edges)

    # Direct route: the internal pass reaches the defensive branch.
    with pytest.raises(FieldDataError, match=r"unsupported\s+regime combo"):
        _validate_pivot_regimes(fd)

    # End-to-end route: the public validator never reaches it -- the shape
    # check rejects the bogus type first, proving the branch is defensive depth.
    with pytest.raises(FieldDataError, match=r"EDGES\[1\]\.type = 'WEIRD'"):
        validate(fd)
