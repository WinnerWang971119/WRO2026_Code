"""Tests for the PR3 field_data schema + validator.

PR3 schema additions: BLIND_EDGES, FIELD_MM, MAP_PNG.
PR3 schema removals: PROJECTIONS, GOAL_APPROACH.
"""

from __future__ import annotations

import copy
import warnings
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.path_planner.editor.loader import load_field_json
from tools.path_planner.graph_builder import build_graph, EDGE_BLIND, EDGE_LINE
from tools.path_planner.pipeline_config import BlindMode, LineMode, StopAtLineMode
from tools.path_planner.tests.fixtures import toy_field_data as real_field_data
from tools.path_planner.validator import (
    BLIND_DECAY_EXPONENT_MAX,
    FieldDataError,
    parse_line_sensor_axle_offset_cm,
    parse_pivot_min_turn_deg_from_config,
    parse_pivot_trim_min_drive_cm_from_config,
    validate,
    WALL_PIVOT_CLEARANCE_CM,
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
    "BLIND_DECAY_ENABLED",
    "BLIND_DECAY_K",
    "BLIND_DECAY_EXPONENT",
    "BLIND_DECAY_D_REF",
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
# ROBOT_RADIUS_CM, the BLIND_DECAY_* knobs, FIELD_MM and EDGES scalars. A NaN
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


# -- issue #64: projection NODE type (TC1..TC5) -------------------------------
#
# Projection is a 4th, mutually-exclusive NODE type. The validator accepts
# ``"projection"`` in NODE_TYPES, rejects a projection (or line) START node,
# allows a LINE edge into a projection node (the line passes through it), and
# warns on a projection node only when it is otherwise unreachable (a
# Blind->Proj in-edge exempts it).


def _projection_field(node_types_overrides=None, line_segments=None,
                      blind_edges=None, start_node="S") -> SimpleNamespace:
    """Build a tiny S/A/B/P field for projection-node validator tests.

    ``S`` is a blind dead-reckon origin; ``A`` and ``B`` are junction line nodes
    on a LINE edge; ``P`` is a projection node sitting on the A-B line. By default
    ``P`` is reached both by a LINE edge (``A-P``) and a blind in-edge (``S-P``),
    so the base field validates cleanly. Tests pass overrides to author the
    specific case.
    """
    node_types = {
        "S": "blind", "A": "line", "B": "line", "P": "projection",
    }
    if node_types_overrides is not None:
        node_types = dict(node_types_overrides)
    return SimpleNamespace(
        NODES={
            "S": (50.0, -40.0),
            "A": (0.0, 0.0),
            "B": (100.0, 0.0),
            "P": (50.0, 0.0),
        },
        NODE_TYPES=node_types,
        LINE_SEGMENTS=[("A", "B"), ("A", "P"), ("P", "B")]
        if line_segments is None else line_segments,
        BLIND_EDGES=[("S", "A"), ("S", "P")]
        if blind_edges is None else blind_edges,
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE=start_node,
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
    )


def test_tc1_projection_node_type_accepted() -> None:
    """TC1: ``NODE_TYPES[x] = "projection"`` validates; an unknown type is rejected.

    The base field types ``P`` projection and validates. A bogus type still fails
    the NODE_TYPES value gate, proving the accept is not a blanket pass.
    """
    assert validate(_projection_field()) is None
    fd_bad = _projection_field()
    fd_bad.NODE_TYPES["P"] = "weird"
    with pytest.raises(FieldDataError, match=r"NODE_TYPES\['P'\]"):
        validate(fd_bad)


def test_tc2_line_edge_into_projection_node_accepted() -> None:
    """TC2 (positive): a LINE edge into a ``projection`` node is legal.

    Only a LINE edge into a ``blind`` node is rejected; the line passes THROUGH a
    projection node, so ``A-P`` / ``P-B`` LINE edges into ``P`` validate. (The
    LINE-into-blind rejection itself is pinned in the ADR-041 section below.)
    """
    assert validate(_projection_field()) is None


def test_tc3_projection_start_node_rejected() -> None:
    """TC3: a ``projection`` START node is rejected (sensor-anchored arrival at t=0).

    The chassis is operator-placed with no prior line-follow, so a projection
    START (reached by a distance-gated / blind seek) presumes an arrival that has
    not happened at mission start. The validator rejects it and points the author
    at a blind origin.
    """
    fd = _projection_field(start_node="P")
    with pytest.raises(
        FieldDataError, match=r"START_NODE 'P' is typed 'projection'"
    ):
        validate(fd)


def test_tc3_line_start_node_rejected() -> None:
    """TC3 (mirror): a ``line`` START node is rejected for the same reason."""
    fd = _projection_field(start_node="A")
    with pytest.raises(
        FieldDataError, match=r"START_NODE 'A' is typed 'line'"
    ):
        validate(fd)


def test_tc4_line_node_zero_line_edges_warns() -> None:
    """TC4: a ``line`` node with no incident LINE edge warns (reachability nudge).

    A junction must sit on a line edge. Here ``B`` is typed ``line`` but its only
    incidence is the BLIND edge ``S-B`` (the LINE segments are dropped to a single
    ``A-P`` edge that does not touch ``B``), so the reachability warn fires for
    ``B``. The blind spawn ``S`` stays the START node, so the line-as-start guard
    never pre-empts the warn.
    """
    fd = _projection_field(
        line_segments=[("A", "P")],          # B has no LINE edge now
        blind_edges=[("S", "A"), ("S", "B")],
    )
    with pytest.warns(UserWarning, match=r"line node 'B' has no incident LINE edge"):
        validate(fd)


def test_tc4_projection_node_blind_in_edge_does_not_warn() -> None:
    """TC4: a projection node reachable ONLY by a Blind->Proj in-edge does NOT warn.

    ``P`` has no LINE edge but keeps its ``S-P`` blind in-edge, so it is
    blind-seek-reachable and exempt from the reachability warn. An isolated
    projection node (no LINE, no BLIND) WOULD warn -- the control below. The
    ``A-B`` LINE segment keeps both junction nodes LINE-incident, so no
    reachability warning of any kind should fire.
    """
    fd = _projection_field(
        line_segments=[("A", "B")],   # drop A-P / P-B so P has no LINE edge
        blind_edges=[("S", "A"), ("S", "P")],
    )
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        assert validate(fd) is None
    reachability = [
        w for w in caught
        if issubclass(w.category, UserWarning) and "no incident" in str(w.message)
        or (issubclass(w.category, UserWarning) and "is unreachable" in str(w.message))
    ]
    assert not reachability, (
        "no reachability warning should fire for a blind-seek-reachable "
        f"projection node; got: {[str(w.message) for w in reachability]}"
    )


def test_tc4_isolated_projection_node_warns() -> None:
    """TC4 (control): an otherwise-unreachable projection node warns.

    ``P`` with neither a LINE edge nor a BLIND in-edge is unreachable and trips
    the projection reachability warn -- proving the exemption above is the
    blind-seek-reachability, not a blanket projection exemption.
    """
    fd = _projection_field(
        line_segments=[("A", "B")],
        blind_edges=[("S", "A")],     # P now has no incident edge at all
    )
    with pytest.warns(UserWarning, match=r"projection node 'P' is unreachable"):
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


def test_validator_accepts_blind_mode_name_string() -> None:
    """Issue #79 (TC: string-tag): a valid BlindMode NAME string validates.

    Under ADR-047 ``primitive_mode`` may be the bare enum member NAME the JSON
    encoding uses (``"STOP_AT_LINE"``), not only a ``BlindMode`` instance. A
    valid BlindMode name on a BLIND edge must now be ACCEPTED -- this is the
    old-contract reversal: pre-#79 the string form was rejected, but the JSON
    editor authors exactly this form, so the validator must resolve it.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_BLIND,
            "u": "B",
            "v": "G",
            "distance_cm": 86.0,
            "heading_deg": 54.46,
            "primitive_mode": "STOP_AT_LINE",  # a bare BlindMode member NAME
        }
    ]
    assert validate(fd) is None


def test_validator_rejects_unknown_blind_mode() -> None:
    """Issue #79 (TC5): a genuinely-unknown blind-mode tag is still rejected.

    The string-tag accept above is not a blanket pass on any string: a name
    that is not a ``BlindMode`` member (``"NOT_A_MODE"``) must fail with a
    precise ``primitive_mode`` diagnostic. This pins the unknown-name reject
    that survives the #79 widening of the accepted vocabulary.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_BLIND,
            "u": "B",
            "v": "G",
            "distance_cm": 86.0,
            "heading_deg": 54.46,
            "primitive_mode": "NOT_A_MODE",  # not a BlindMode member name
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


def test_validator_rejects_retired_projection_edge_type() -> None:
    """TC5 / issue #64: the retired ``"PROJECTION"`` EDGES type is rejected.

    Issue #64 reduced the edge-type vocabulary to LINE / BLIND -- projection is a
    NODE property now, not an edge type. A stale EDGES entry still spelling
    ``type="PROJECTION"`` must fail the edge-type guard with a precise message
    that points the author at the node-typing migration. ('B','C') is a
    LINE_SEGMENTS pair in the toy field, but the type guard fires first (before
    the anti-ambiguity cross-check).
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": "PROJECTION",
            "u": "B",
            "v": "C",
            "distance_cm": 100.0,
            "heading_deg": 0.0,
        }
    ]
    with pytest.raises(FieldDataError, match=r"expected one of"):
        validate(fd)


def test_validator_accepts_same_type_line_override() -> None:
    """TC5 (positive): EDGE_LINE on a LINE_SEGMENTS pair is a valid override.

    The intended use of a rich EDGES entry is as an override layer attaching
    distance/heading/primitive_mode to an existing edge. A matching-type entry
    on an existing pair must NOT be rejected by the anti-ambiguity rule.
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


# -- Issue #79 / ADR-047: JSON string-tag primitive_mode + optional geometry --
#
# Per-edge ``primitive_mode`` gains a JSON-representable string-tag encoding
# (the enum member NAME verbatim) accepted alongside the legacy enum instance,
# cross-checked against the edge type, and rejecting the DEFAULT sentinel in
# BOTH forms. Geometry (distance_cm / heading_deg) becomes optional on an
# override-only overlay entry and is derived from the node coords downstream.
# The toy field already carries the A-B LINE pair and the B-G BLIND pair, so a
# same-type override on either is a legitimate overlay.


def test_tc1_edges_line_mode_name_string_validates_and_resolves() -> None:
    """TC1 (validate + build): a LINE edge with primitive_mode='FAST' validates.

    The string TAG (a LineMode member name) on a same-type LINE override of the
    A-B pair must validate, and ``build_graph`` must forward it verbatim onto the
    resolved directed edge so the emitter can resolve it to ``LineMode::FAST``
    (the emit half lives in test_header_writer). Geometry is supplied here; the
    override-only (geometry-omitted) form is TC2.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "B",
            "distance_cm": 190.0,
            "heading_deg": 0.0,
            "primitive_mode": "FAST",
        }
    ]
    assert validate(fd) is None
    g = build_graph(fd)
    # The string tag is forwarded verbatim onto both directions (bidirectional).
    assert g.edges["A", "B"]["primitive_mode"] == "FAST"
    assert g.edges["B", "A"]["primitive_mode"] == "FAST"


def test_tc2_edges_entry_geometry_omitted_validates_and_derives() -> None:
    """TC2: an EDGES override with distance_cm / heading_deg OMITTED validates.

    An override-only overlay entry carries only ``{type, u, v, primitive_mode}``.
    The validator must NOT require geometry, and ``build_graph`` must derive
    distance / heading from the node coords (A=(10,10), B=(200,10): 190 cm due
    East), byte-identically to the planner graph.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "B",
            "primitive_mode": "FAST",
        }
    ]
    assert validate(fd) is None
    g = build_graph(fd)
    # Geometry derived from coords (not pinned): A->B is 190 cm due East.
    assert g.edges["A", "B"]["distance_cm"] == pytest.approx(190.0, abs=1e-6)
    assert g.edges["A", "B"]["heading_deg"] == pytest.approx(0.0, abs=1e-6)
    assert g.edges["B", "A"]["distance_cm"] == pytest.approx(190.0, abs=1e-6)
    assert g.edges["B", "A"]["heading_deg"] == pytest.approx(180.0, abs=1e-6)
    # The override still rides on the derived edge.
    assert g.edges["A", "B"]["primitive_mode"] == "FAST"


def test_tc2_stop_at_line_fixture_carries_geometry_verbatim() -> None:
    """TC2 (backward-compat): a geometry-bearing EDGES entry still validates.

    The committed ``field_data_stop_at_line.json`` fixture authors a BLIND
    override that DOES carry ``distance_cm`` / ``heading_deg`` (intentional
    backward-compat). It must validate, and ``build_graph`` must honor the pinned
    geometry rather than re-deriving it -- pinning that a hand-authored
    geometry-carrying entry is preserved alongside the geometry-optional path.
    """
    data = load_field_json(_STOP_AT_LINE_FIXTURE)
    fd = SimpleNamespace(**data)
    assert validate(fd) is None
    g = build_graph(fd)
    # The fixture pins 86.0232... cm; build_graph honors it (not the coord-
    # derived B=(200,10)->G=(250,80) length).
    assert g.edges["B", "G"]["distance_cm"] == pytest.approx(86.0232526704263, abs=1e-9)


def test_tc3_edges_primitive_mode_enum_instance_validates() -> None:
    """TC3: a primitive_mode authored as an enum INSTANCE still validates (legacy).

    The legacy Python authoring form (``LineMode.FAST`` -- an enum instance, not
    a string) must keep validating and build into the graph identically. Pairs
    with the string-tag path (TC1) so both encodings stay live.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "B",
            "primitive_mode": LineMode.FAST,
        }
    ]
    assert validate(fd) is None
    g = build_graph(fd)
    assert g.edges["A", "B"]["primitive_mode"] is LineMode.FAST


def test_tc4_line_mode_name_string_on_blind_edge_rejected() -> None:
    """TC4: a LineMode NAME string on a BLIND edge is a cross-type error.

    ``"FAST"`` is a LineMode member, illegal on a BLIND edge. The validator must
    reject it (the BlindMode cross-check), naming the expected ``BlindMode``.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_BLIND,
            "u": "B",
            "v": "G",
            "primitive_mode": "FAST",  # a LineMode name on a BLIND edge
        }
    ]
    with pytest.raises(FieldDataError, match="BlindMode"):
        validate(fd)


def test_tc4_blind_mode_name_string_on_line_edge_rejected() -> None:
    """TC4 (inverse): a BlindMode NAME string on a LINE edge is rejected.

    ``"STOP_AT_LINE"`` is a BlindMode member, illegal on a LINE edge. The
    validator must reject it, naming the expected ``LineMode``.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "B",
            "primitive_mode": "STOP_AT_LINE",  # a BlindMode name on a LINE edge
        }
    ]
    with pytest.raises(FieldDataError, match="LineMode"):
        validate(fd)


def test_tc5_unknown_primitive_mode_tag_rejected() -> None:
    """TC5: an unknown primitive_mode tag is rejected with a precise diagnostic."""
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "B",
            "primitive_mode": "BOGUS",  # not a LineMode member name
        }
    ]
    with pytest.raises(FieldDataError, match="primitive_mode"):
        validate(fd)


def test_tc6_primitive_mode_default_string_rejected() -> None:
    """TC6: the literal string 'DEFAULT' is rejected (DEFAULT == absence).

    ``LineMode["DEFAULT"]`` RESOLVES (DEFAULT is a real member), so the
    unknown-name guard would miss it -- the validator must reject the literal
    ``"DEFAULT"`` string explicitly. Authoring DEFAULT is a confusion: absence
    already encodes "inherit PIPELINE_CONFIG".
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "B",
            "primitive_mode": "DEFAULT",
        }
    ]
    with pytest.raises(FieldDataError, match="primitive_mode"):
        validate(fd)


def test_tc6_primitive_mode_default_enum_instance_rejected() -> None:
    """TC6 (enum form): the LineMode.DEFAULT INSTANCE is rejected too.

    Both forms of DEFAULT -- the string and the enum instance -- must fail, so
    a legacy Python entry that explicitly sets ``primitive_mode=LineMode.DEFAULT``
    is caught the same way as the JSON ``"DEFAULT"`` string.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "B",
            "primitive_mode": LineMode.DEFAULT,
        }
    ]
    with pytest.raises(FieldDataError, match="DEFAULT"):
        validate(fd)


def test_tc6_blind_primitive_mode_default_string_rejected() -> None:
    """TC6 (blind mirror): 'DEFAULT' on a BLIND edge is rejected too.

    The DEFAULT-sentinel rejection is symmetric across edge types:
    ``BlindMode["DEFAULT"]`` also resolves, so a BLIND override of ``"DEFAULT"``
    must be rejected before the enum lookup.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_BLIND,
            "u": "B",
            "v": "G",
            "primitive_mode": "DEFAULT",
        }
    ]
    with pytest.raises(FieldDataError, match="primitive_mode"):
        validate(fd)


def test_tc20_override_only_pivot_leg_seen_by_pivot_validator() -> None:
    """TC20: an override-only (geometry-less) pivot leg is NOT dropped.

    Guards the ``_build_authored_graph`` derive-not-skip fix. The pivot
    ``(A, B, C)`` is authored as override-only EDGES entries (no distance_cm /
    heading_deg). The earlier helper ``continue``d past geometry-less entries,
    which silently dropped both legs from the pivot-validation graph -- the pivot
    would then read as "incoming edge not authored". With the fix, geometry is
    derived from coords so ``_validate_pivot_turns`` SEES both legs and the
    geometry resolves: A->B (90 cm East), B->C (90 cm North) is a 90 deg pivot
    whose delta (9.0 cm at track 18) clears both long dead-reckon legs.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B"},  # override-only, no geometry
            {"type": EDGE_BLIND, "u": "B", "v": "C"},  # override-only, no geometry
        ],
        pivot_turns=[("A", "B", "C")],
    )
    # Coords: A=(0,0), B=(100,0), C=(100,100) -> A->B East, B->C North (90 deg).
    # The legs are derived (100 cm each) and the pivot validates -- proving the
    # geometry-less legs were NOT skipped by _build_authored_graph.
    assert validate(fd) is None


def test_tc20_override_only_pivot_leg_resolves_stop_at_line() -> None:
    """TC20 (override resolves): a geometry-less pivot leg honors stop_at_line.

    An override-only incoming leg ``A->B`` (no geometry) carrying
    ``stop_at_line: "on"`` must be derived AND have its seek override honored:
    the leg classifies SENSOR-terminated (takes no approach trim), so a 90 deg
    pivot onto a derived-100cm leg validates, AND with the trigger removed the
    short-distance gate would not even apply here -- so to prove the override is
    read on a derived edge, the outgoing leg is pinned SHORT via a geometry-
    bearing entry that would reject a 'distance' incoming. Simpler: the incoming
    is derived-100cm and seeking; we assert it validates with the override and
    REJECTS the FRONT-only rear guard when direction:'rear' is added -- pinning
    that direction/stop_at_line resolve on the geometry-derived leg.
    """
    # Incoming leg is override-only (geometry derived) + stop_at_line on -> the
    # pivot validator must read the override off the DERIVED edge. A 'rear'
    # direction on that same derived leg trips the FRONT-only guard, proving the
    # directional overrides are resolved on the derived edge, not dropped.
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "stop_at_line": "on", "direction": "rear"},
            {"type": EDGE_BLIND, "u": "B", "v": "C"},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    with pytest.raises(
        FieldDataError, match=r"direction:'rear' on its incoming edge"
    ):
        validate(fd)


# -- Issue #79 / ADR-047: field-sourced PIPELINE_CONFIG validation -------------
#
# The four optional PIPELINE_* keys carry mission-wide motion-mode defaults as
# enum member NAMES. _validate_pipeline_config mirrors
# PipelineConfig.__post_init__: line / blind / turn may NOT be DEFAULT (circular
# when set as the pipeline default itself); DirectionMode.DEFAULT IS allowed.
# Reads are non-strict, so a field that omits every key validates unchanged.


def test_tc9_pipeline_line_mode_default_rejected() -> None:
    """TC9: pipeline_line_mode = 'DEFAULT' is rejected (line may not be DEFAULT).

    DEFAULT means "inherit PIPELINE_CONFIG.line", which is circular when set as
    the pipeline default itself. ``LineMode["DEFAULT"]`` resolves, so the literal
    must be rejected before the enum lookup.
    """
    fd = _clone_field_data()
    fd.PIPELINE_LINE_MODE = "DEFAULT"
    with pytest.raises(FieldDataError, match="PIPELINE_LINE_MODE"):
        validate(fd)


def test_tc9_pipeline_blind_mode_default_rejected() -> None:
    """TC9 (blind): pipeline_blind_mode = 'DEFAULT' is rejected too."""
    fd = _clone_field_data()
    fd.PIPELINE_BLIND_MODE = "DEFAULT"
    with pytest.raises(FieldDataError, match="PIPELINE_BLIND_MODE"):
        validate(fd)


def test_tc9_pipeline_turn_mode_default_rejected() -> None:
    """TC9 (turn): pipeline_turn_mode = 'DEFAULT' is rejected too."""
    fd = _clone_field_data()
    fd.PIPELINE_TURN_MODE = "DEFAULT"
    with pytest.raises(FieldDataError, match="PIPELINE_TURN_MODE"):
        validate(fd)


def test_tc9_pipeline_direction_mode_default_accepted() -> None:
    """TC9 (positive): pipeline_direction_mode = 'DEFAULT' IS accepted.

    DirectionMode.DEFAULT means "defer to the builder auto algorithm" -- a
    legitimate value, the one exception to the no-DEFAULT rule. This pins that
    the reject above is the line/blind/turn circular-DEFAULT rule, not a blanket
    DEFAULT ban.
    """
    fd = _clone_field_data()
    fd.PIPELINE_DIRECTION_MODE = "DEFAULT"
    assert validate(fd) is None


def test_pipeline_modes_concrete_name_strings_accepted() -> None:
    """All four PIPELINE_* keys accept a concrete member-name string.

    Pins the positive path the field-sourced emission relies on: a field that
    authors concrete tags for every category validates clean.
    """
    fd = _clone_field_data()
    fd.PIPELINE_LINE_MODE = "FAST"
    fd.PIPELINE_BLIND_MODE = "GYRO_TSHAPE"
    fd.PIPELINE_TURN_MODE = "PIVOT_LEFT"
    fd.PIPELINE_DIRECTION_MODE = "FORCE_FORWARD"
    assert validate(fd) is None


def test_pipeline_mode_unknown_name_rejected() -> None:
    """An unknown PIPELINE_* member name is rejected with a precise diagnostic."""
    fd = _clone_field_data()
    fd.PIPELINE_BLIND_MODE = "NOT_A_MODE"
    with pytest.raises(FieldDataError, match="PIPELINE_BLIND_MODE"):
        validate(fd)


def test_pipeline_mode_enum_instance_accepted() -> None:
    """A PIPELINE_* key authored as an enum INSTANCE validates (legacy/programmatic).

    Mirrors the primitive_mode enum-instance acceptance: a Python fixture that
    sets ``PIPELINE_LINE_MODE = LineMode.FAST`` is accepted, while the same
    field's enum-instance DEFAULT is rejected by the circular-DEFAULT rule.
    """
    fd = _clone_field_data()
    fd.PIPELINE_LINE_MODE = LineMode.FAST
    assert validate(fd) is None
    fd.PIPELINE_LINE_MODE = LineMode.DEFAULT
    with pytest.raises(FieldDataError, match="PIPELINE_LINE_MODE"):
        validate(fd)


def test_stop_at_line_fixture_loads_and_validates() -> None:
    """TC4: the committed STOP_AT_LINE fixture field loads, validates, and seeks.

    The fixture authors the stop-at-line BLIND edge in the on-disk JSON
    schema via ``stop_at_line: "on"`` (a per-edge seek trigger, distinct from
    the ``primitive_mode`` motion bundle). Under issue #79 / ADR-047 a JSON
    ``primitive_mode`` string TAG (``"STOP_AT_LINE"``) is now also accepted
    (see ``test_validator_accepts_blind_mode_name_string``); this fixture keeps
    using ``stop_at_line: "on"`` because the seek trigger is the behavior it
    exercises. Loading it through the editor loader and validating must
    succeed, and building the
    graph must resolve the override onto the edge -- otherwise the "exemplar"
    silently dead-reckons through a crossing meant to re-seek the line
    (issue #93). Asserting the resolved ``StopAtLineMode.ON`` on the built
    edge, not a raw dict key, is what makes this test prove anything.
    """
    data = load_field_json(_STOP_AT_LINE_FIXTURE)
    fd = SimpleNamespace(**data)
    assert validate(fd) is None
    # The override survives the build: the one BLIND edge actually seeks.
    g = build_graph(fd)
    assert g.get_edge_data("B", "G")["stop_at_line_override"] is StopAtLineMode.ON


# -- Obsolete per-edge pivot keys rejected (issue #44 per-turn model, T5) -----
#
# The per-turn model removed the per-edge ``pivot`` / ``pre_turn_mode`` EDGES
# keys (single-wheel pivots are now authored as (u, v, w) triples in the
# top-level PIVOT_TURNS list). A field that still carries one of these on an
# EDGES entry pre-dates the migration: the EDGES shape check rejects it up
# front with a precise FieldDataError so the author is pointed at PIVOT_TURNS
# (and the stale ``pivot`` string can never reach graph_builder and crash on
# ``int("left")``).


def test_validator_rejects_obsolete_pivot_edge_key() -> None:
    """An EDGES entry carrying the removed ``pivot`` key is rejected."""
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "B",
            "distance_cm": 100.0,
            "heading_deg": 0.0,
            "pivot": 1,
        }
    ]
    with pytest.raises(FieldDataError, match=r"removed per-edge key 'pivot'"):
        validate(fd)


def test_validator_rejects_obsolete_pre_turn_mode_edge_key() -> None:
    """An EDGES entry carrying the removed ``pre_turn_mode`` key is rejected."""
    fd = _clone_field_data()
    fd.EDGES = [
        {
            "type": EDGE_LINE,
            "u": "A",
            "v": "B",
            "distance_cm": 100.0,
            "heading_deg": 0.0,
            "pre_turn_mode": "PIVOT_LEFT",
        }
    ]
    with pytest.raises(FieldDataError, match=r"removed per-edge key 'pre_turn_mode'"):
        validate(fd)


# -- TC3..TC12: per-turn pivot geometry fail-fast (_validate_pivot_turns) ------
#
# A single-wheel pivot is authored per TURN -- a 3-node triple ``(u, v, w)`` in
# ``field_data.PIVOT_TURNS`` (arrive from ``u``, pivot at ``v``, leave to
# ``w``). ``validate`` derives the turn angle theta at ``v`` from the two
# directed-edge headings, computes the single-wheel over-travel
# delta = (track/2)*tan(|theta|/2) with the live track (Config::TRACK_MM = 180
# mm -> 18.0 cm), and raises a precise FieldDataError on geometry the chassis
# cannot realise. A regime is SENSOR-terminated (LINE into a junction node, or
# BLIND that SEEKS -- STOP_AT_LINE / override-ON / the Blind->projection-node
# auto-rule) -- it re-anchors on the physical line and self-corrects -- or
# DISTANCE-terminated (plain BLIND, or LINE into a projection node) --
# dead-reckoned, where the over-travel trim actually bites (issue #64).
#
# Reference geometry at track = 18.0 cm:
#   theta = 90 deg  -> delta = 9.000 cm
#   theta = 120 deg -> delta = 15.588 cm
#   theta = 10 deg  -> delta = 0.787 cm
#   the LINE overshoot budget (FRONT axle offset) is 12.02 cm, reached at
#   theta ~= 106.4 deg.
#
# Every fixture is built by ``_pivot_field`` below: three nodes A/B/C, all
# adjacency authored via explicit EDGES entries, and a single ``PIVOT_TURNS``
# triple. ``_validate_pivot_turns`` reads ``heading_deg`` / ``distance_cm`` /
# ``type`` off the entries, so the turn angle and over-travel are fully
# determined by the authored edges, not by the (inert) node coordinates.


def _pivot_field(edges, pivot_turns, node_types=None) -> SimpleNamespace:
    """Build a minimal A/B/C/D field carrying the given EDGES + PIVOT_TURNS.

    Three real nodes plus a spare ``D`` (so a fixture can author a fourth leg
    for a chained-pivot test). No LINE_SEGMENTS / BLIND_EDGES, so all adjacency
    and geometry come from the explicit EDGES entries; the ``PIVOT_TURNS``
    triples are the authored per-turn pivots the validator checks.

    ``node_types`` defaults to ``B/C/D`` all blind (the BLIND-incoming pivot
    cases). A LINE-incoming pivot must override it to type the pivot node ``B``
    as ``"line"`` -- a LINE (line-follow) edge cannot terminate on a blind node,
    since the validator rejects that (ADR-041). A ``"projection"`` destination is
    legal for a LINE hop (issue #64), used by the Blind->projection auto-rule
    cases below.

    ``START_NODE`` is the spare blind node ``D``: issue #64's validator rejects a
    ``line`` / ``projection`` START node (the operator-placed spawn is
    dead-reckoned), and the pivot node ``A`` is often typed ``line`` (a
    LINE-incoming pivot). ``D`` is blind in every ``node_types`` map here and is
    not part of the single-pivot triples, so it is a legal dead-reckon spawn that
    leaves the pivot geometry untouched.
    """
    if node_types is None:
        node_types = {"A": "line", "B": "blind", "C": "blind", "D": "blind"}
    return SimpleNamespace(
        NODES={
            "A": (0.0, 0.0),
            "B": (100.0, 0.0),
            "C": (100.0, 100.0),
            "D": (200.0, 100.0),
        },
        NODE_TYPES=node_types,
        LINE_SEGMENTS=[],
        BLIND_EDGES=[],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE="D",          # spare blind spawn (validator rejects an on-line start)
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        EDGES=edges,
        PIVOT_TURNS=pivot_turns,
    )


def test_pivot_turn_rejects_missing_incoming_edge() -> None:
    """TC3: a pivot whose incoming leg ``u->v`` is not authored is rejected.

    Only the outgoing leg ``B->C`` is authored, so the incoming ``A->B`` is
    missing. The error names the absent ``('A','B')`` edge.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    with pytest.raises(
        FieldDataError, match=r"incoming edge\s+\('A','B'\) that is not authored"
    ):
        validate(fd)


def test_pivot_turn_rejects_missing_outgoing_edge() -> None:
    """TC3 (mirror): a pivot whose outgoing leg ``v->w`` is not authored is rejected.

    Only the incoming leg ``A->B`` is authored, so the outgoing ``B->C`` is
    missing. The error names the absent ``('B','C')`` edge.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    with pytest.raises(
        FieldDataError, match=r"outgoing edge\s+\('B','C'\) that is not authored"
    ):
        validate(fd)


def test_pivot_turn_rejects_duplicate_triple() -> None:
    """TC4: the same ``(u, v, w)`` triple authored twice is rejected.

    A turn is authored once; a duplicate triple is an authoring mistake and
    must fail with a precise duplicate diagnostic.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C"), ("A", "B", "C")],
    )
    with pytest.raises(FieldDataError, match=r"duplicate pivot turn"):
        validate(fd)


def test_pivot_turn_rejects_degenerate_180_angle() -> None:
    """TC5: a pivot whose in/out headings differ by ~180 deg is rejected.

    Incoming ``A->B`` heads 0 deg; outgoing ``B->C`` heads 180 deg, so theta
    normalizes to -180 deg -- at/above the 179 deg ceiling where
    (track/2)*tan(theta/2) diverges. Both legs are long (100 cm) so only the
    degenerate-angle gate can trip.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 180.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    with pytest.raises(FieldDataError, match=r"degenerate turn\s+angle"):
        validate(fd)


def test_pivot_turn_rejects_below_floor_angle() -> None:
    """TC5 (mirror): a sub-floor turn (~3 deg) is rejected.

    Incoming heads 0 deg, outgoing heads 3 deg -- below the configurable
    Config::PIVOT_MIN_TURN_DEG = 5 deg floor. A turn that small would slip under
    the firmware's turn-dispatch gate and silently no-op the pivot, so it is an
    authoring mistake that must fail. (3 deg, not 5 deg: 5 deg now sits exactly
    AT the floor and is accepted -- see the pass counterpart below.)
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 3.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    with pytest.raises(FieldDataError, match=r"degenerate turn\s+angle"):
        validate(fd)


def test_pivot_turn_accepts_angle_at_floor() -> None:
    """TC5 (pass counterpart): a turn exactly AT the floor (5 deg) validates.

    The floor check rejects ``|theta| < PIVOT_MIN_TURN_DEG``, so a turn at the
    5 deg floor is accepted. Both BLIND legs are long (100 cm) so the tiny
    delta (~0.39 cm at track 18) clears the dead-reckon gate -- this pins the
    inclusive floor boundary the configurable knob introduced.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 5.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    assert validate(fd) is None


def test_pivot_turn_rejects_delta_exceeds_outgoing_dead_reckon() -> None:
    """TC6: delta >= the OUTGOING dead-reckoned edge length is rejected.

    A 90 deg pivot (delta = 9.0 cm at track 18) whose outgoing continuation is
    a plain BLIND edge only 2.0 cm long. The out trim cannot meet or exceed the
    drive it shortens, so the validator names the OUTGOING side. The incoming
    edge is long (100 cm) so the incoming check passes first.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 2.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    with pytest.raises(
        FieldDataError, match=r"the OUTGOING dead-reckoned edge length"
    ):
        validate(fd)


def test_pivot_turn_rejects_delta_exceeds_incoming_dead_reckon() -> None:
    """TC6 (mirror): delta >= the INCOMING dead-reckoned edge length is rejected.

    A 90 deg pivot (delta = 9.0 cm) whose INCOMING dead-reckoned edge ``A->B``
    is only 2.0 cm long. The approach trim cannot meet or exceed the drive it
    shortens, so the validator names the INCOMING side. The outgoing edge is
    long (100 cm), so the incoming gate is the one that trips.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 2.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    with pytest.raises(
        FieldDataError, match=r"the INCOMING dead-reckoned edge length"
    ):
        validate(fd)


def test_pivot_turn_accepts_long_dead_reckon_edges() -> None:
    """TC6 (pass counterpart): long-enough dead-reckon edges validate.

    Same 90 deg / delta = 9.0 cm pivot, but both BLIND legs are 100 cm --
    comfortably longer than the 9.0 cm trim -- so neither dead-reckon gate
    trips. Proves the rejections above are about the short length, not the
    90 deg turn itself.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    assert validate(fd) is None


def test_pivot_turn_rejects_delta_within_trim_floor_of_dead_reckon() -> None:
    """TC6b: a delta leaving < PIVOT_TRIM_MIN_DRIVE_CM of the edge is rejected.

    The firmware/sim clamp a trimmed dead-reckon drive to the trim-underflow
    floor (Config::PIVOT_TRIM_MIN_DRIVE_CM = 0.5 cm). A 90 deg pivot (delta =
    9.0 cm at track 18) onto a 9.3 cm BLIND edge leaves 0.3 cm < 0.5 cm: the old
    raw ``delta >= length`` gate accepted it, but the runtime would clamp to the
    floor and overshoot the node. The validator now rejects on the same floor.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 9.3, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    with pytest.raises(
        FieldDataError, match=r"below the firmware's trim-underflow floor"
    ):
        validate(fd)


def test_pivot_turn_rejects_delta_exceeds_line_overshoot_budget() -> None:
    """TC7: a LINE-incoming pivot whose delta exceeds the budget is rejected.

    A LINE incoming edge realises its pivot correction as a reduction of the
    post-detect LINE_OVERSHOOT advance, valid only while delta stays within the
    FRONT sensor-to-axle offset budget (Config::LINE_SENSOR_AXLE_OFFSET_CM =
    12.02 cm). A 120 deg turn gives delta = 15.588 cm > 12.02 cm, so it is
    rejected. Both edges are long, so only the overshoot-budget gate trips.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_LINE, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 120.0},
        ],
        pivot_turns=[("A", "B", "C")],
        node_types={"A": "line", "B": "line", "C": "blind", "D": "blind"},
    )
    with pytest.raises(
        FieldDataError, match=r"the sensor/LINE overshoot budget"
    ):
        validate(fd)


def test_pivot_turn_accepts_line_incoming_within_overshoot_budget() -> None:
    """TC7 (pass counterpart): a LINE-incoming pivot within budget validates.

    Same LINE incoming edge, but a 90 deg turn gives delta = 9.0 cm <= 12.02 cm
    budget, so the overshoot-budget gate does not trip and the outgoing BLIND
    edge (100 cm) easily clears its dead-reckon gate. Proves the budget
    rejection is about the over-large delta, not the LINE incoming edge per se.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_LINE, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
        node_types={"A": "line", "B": "line", "C": "blind", "D": "blind"},
    )
    assert validate(fd) is None


def test_pivot_turn_rejects_line_delta_within_trim_floor_of_budget() -> None:
    """TC7b: a LINE-incoming delta within the trim floor of the budget is rejected.

    The review's exact case: track 18 cm, FRONT overshoot budget 12.02 cm. A
    ~105 deg turn gives delta ~= 11.73 cm, so the reduced overshoot advance is
    ~0.29 cm < the 0.5 cm trim floor. The old ``delta > budget`` gate accepted
    it (11.73 <= 12.02), but the firmware clamps the advance and overshoots the
    node; the validator now rejects on the same floor.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_LINE, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 105.0},
        ],
        pivot_turns=[("A", "B", "C")],
        node_types={"A": "line", "B": "line", "C": "blind", "D": "blind"},
    )
    with pytest.raises(
        FieldDataError, match=r"below the firmware's trim-underflow floor"
    ):
        validate(fd)


def test_pivot_turn_no_override_blind_classifies_sensor_under_stop_at_line_default(
    monkeypatch,
) -> None:
    """TC8: a no-override BLIND outgoing leg inherits PIPELINE_CONFIG.blind.

    Review fix 1: a BLIND edge with no per-edge ``primitive_mode`` classifies on
    the EFFECTIVE mode, which inherits ``PIPELINE_CONFIG.blind``. When that
    default is STOP_AT_LINE, the no-override BLIND outgoing leg is SENSOR-
    terminated, so the outgoing trim no-ops and a 90 deg pivot onto a SHORT
    (2.0 cm) outgoing edge must NOT be falsely rejected as a too-short
    dead-reckon. With the live default (SYNC_TSHAPE -> distance) the same
    fixture WOULD reject, so this test pins the regime to STOP_AT_LINE.
    """
    import tools.path_planner.pipeline_config as pc
    from tools.path_planner.pipeline_config import PipelineConfig

    stop_at_line_config = PipelineConfig(blind=BlindMode.STOP_AT_LINE)
    monkeypatch.setattr(pc, "PIPELINE_CONFIG", stop_at_line_config)

    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 2.0, "heading_deg": 90.0},  # no primitive_mode override
        ],
        pivot_turns=[("A", "B", "C")],
    )
    # Outgoing leg classifies sensor (inherits STOP_AT_LINE), so the short
    # outgoing edge takes no trim and the dead-reckon gate must not fire.
    assert validate(fd) is None


def test_pivot_turn_incoming_blind_seeks_under_stop_at_line_pipeline_default(
    monkeypatch,
) -> None:
    """TC9: a no-override BLIND incoming leg classifies sensor under the STOP_AT_LINE pipeline default.

    Issue #64 retired the per-edge ``primitive_mode``-drives-the-seek arm of
    ``_regime``: a BLIND leg's seek now resolves against the pipeline-default
    ``BlindMode`` (plus its per-edge ``stop_at_line`` override and the local
    auto-rule), never its bidirectional ``primitive_mode`` motion bundle. So the
    surviving "incoming seeks" path on the INCOMING side is the STOP_AT_LINE
    pipeline default: with ``PIPELINE_CONFIG.blind == STOP_AT_LINE`` a no-override
    BLIND incoming leg ``A->B`` is SENSOR-terminated (takes no approach trim), so
    a 90 deg pivot whose incoming edge is only 2.0 cm must NOT be rejected as a
    too-short dead-reckon. With the live default (SYNC_TSHAPE) the same fixture
    WOULD reject, pinning the regime to STOP_AT_LINE.
    """
    import tools.path_planner.pipeline_config as pc
    from tools.path_planner.pipeline_config import PipelineConfig

    monkeypatch.setattr(
        pc, "PIPELINE_CONFIG", PipelineConfig(blind=BlindMode.STOP_AT_LINE)
    )

    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 2.0, "heading_deg": 0.0},  # no primitive_mode override
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    # Incoming leg classifies sensor (inherits STOP_AT_LINE), so the short
    # incoming edge takes no trim and the dead-reckon gate must not fire.
    assert validate(fd) is None


def test_pivot_turn_rejects_stop_at_line_incoming_over_overshoot_budget() -> None:
    """ADR-044: a STOP_AT_LINE (sensor)-incoming pivot is now budgeted too.

    A BLIND incoming edge that SEEKS classifies as 'sensor' and re-anchors via the
    seek, reducing the post-detect overshoot by delta exactly like a LINE incoming
    -- so it shares the same FRONT overshoot budget (12.02 cm). A 120 deg pivot
    (delta = 15.588 cm, well over the budget) is therefore REJECTED, not exempt.
    Issue #64 retired the per-edge ``primitive_mode``-drives-the-seek arm, so the
    seek trigger here is the per-edge ``stop_at_line: "on"`` override on the
    INCOMING leg (a surviving seek trigger ``_regime`` reads). Mirrors
    ``test_pivot_turn_rejects_delta_exceeds_line_overshoot_budget`` (LINE
    incoming, same 120 deg, rejected).
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0,
             "stop_at_line": "on"},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 120.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    with pytest.raises(
        FieldDataError, match=r"the sensor/LINE overshoot budget"
    ):
        validate(fd)


def test_pivot_turn_accepts_chained_pivots_within_shared_edge() -> None:
    """TC11: chained pivots with delta_uv + delta_vw < length(v->w) validate.

    Two pivots ``A->B->C`` and ``B->C->D`` share the dead-reckoned edge
    ``B->C``: the first pivot's OUTGOING (departing) trim and the second's
    INCOMING (approaching) trim both subtract from it. Each is a 90 deg turn
    (delta = 9.0 cm at track 18), so the two trims sum to 18.0 cm; the shared
    edge is 30.0 cm, comfortably above 18.0, so the chain fits and validates.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 30.0, "heading_deg": 90.0},
            {"type": EDGE_BLIND, "u": "C", "v": "D",
             "distance_cm": 100.0, "heading_deg": 0.0},
        ],
        pivot_turns=[("A", "B", "C"), ("B", "C", "D")],
    )
    assert validate(fd) is None


def test_pivot_turn_rejects_chained_pivots_over_trimming_shared_edge() -> None:
    """TC11 (reject): chained pivots with delta_uv + delta_vw >= length is rejected.

    Same chain as the accept above, but the shared dead-reckoned edge ``B->C``
    is only 15.0 cm. The two 90 deg trims sum to 18.0 cm >= 15.0 cm, so the
    firmware's two independent subtractions would command a non-positive drive
    on ``B->C`` and the chained-pivot sum-check rejects it.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 15.0, "heading_deg": 90.0},
            {"type": EDGE_BLIND, "u": "C", "v": "D",
             "distance_cm": 100.0, "heading_deg": 0.0},
        ],
        pivot_turns=[("A", "B", "C"), ("B", "C", "D")],
    )
    with pytest.raises(FieldDataError, match=r"chain on\s+the shared dead-reckoned edge"):
        validate(fd)


def test_pivot_turn_rejects_authored_rear_on_pivot_adjacent_edge() -> None:
    """TC12: an authored direction:'rear' on a pivot-adjacent edge is rejected.

    Single-wheel pivots are FRONT-only in v1 (reverse pivots are tracked by
    #54). A ``direction:'rear'`` override on the exact incoming ``A->B`` leg of
    a pivot is rejected with a precise FRONT-only diagnostic naming the
    offending edge.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0, "direction": "rear"},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    with pytest.raises(
        FieldDataError, match=r"direction:'rear' on its incoming edge"
    ):
        validate(fd)


# -- TC13..TC15: the NEW seek triggers exempt a pivot leg from the trim --------
#
# A blind pivot leg is SENSOR-terminated (no over-travel trim) whenever it
# SEEKS a line at runtime. Besides the legacy ``BlindMode.STOP_AT_LINE`` tag
# (TC8..TC10 above), two triggers were added: a per-edge ``stop_at_line``
# override of ON, and the BLIND->PROJECTION auto-rule. The validator must read
# both off the same shared ``resolve_stop_at_line`` resolver the robot runs, or
# it falsely rejects a legal field (gates a sensor leg's over-travel as if
# dead-reckoned). Each accept below uses a leg too SHORT to survive the
# dead-reckon gate, so it rejects without the trigger and validates with it.


def test_pivot_turn_incoming_stop_at_line_override_on_is_sensor_exempt() -> None:
    """TC13: a per-edge stop_at_line:'on' on the incoming leg makes it sensor.

    A 90 deg pivot (delta = 9.0 cm) whose INCOMING BLIND edge ``A->B`` is only
    2.0 cm long. Without an override it classifies 'distance' and the incoming
    dead-reckon gate rejects it (see
    ``test_pivot_turn_rejects_delta_exceeds_incoming_dead_reckon``). The per-edge
    ``stop_at_line:'on'`` override makes the edge SEEK at runtime, so it is
    SENSOR-terminated -- takes no approach trim -- and the short incoming leg
    must NOT be falsely rejected.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 2.0, "heading_deg": 0.0, "stop_at_line": "on"},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    assert validate(fd) is None


def test_pivot_turn_outgoing_stop_at_line_override_on_is_sensor_exempt() -> None:
    """TC14: a per-edge stop_at_line:'on' on the outgoing leg makes it sensor.

    The outgoing leg's auto-rule is goal-relative and not resolved per-triple,
    but an EXPLICIT ``stop_at_line:'on'`` IS honored: a 90 deg pivot onto a SHORT
    (2.0 cm) outgoing BLIND edge that would otherwise reject as a too-short
    dead-reckon (see
    ``test_pivot_turn_rejects_delta_exceeds_outgoing_dead_reckon``) validates,
    because the seeking edge is SENSOR-terminated and takes no outgoing trim.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 2.0, "heading_deg": 90.0, "stop_at_line": "on"},
        ],
        pivot_turns=[("A", "B", "C")],
    )
    assert validate(fd) is None


def test_pivot_turn_incoming_blind_into_projection_node_auto_seeks_sensor() -> None:
    """TC15: an incoming BLIND whose DESTINATION is a projection node auto-seeks -> sensor.

    Issue #64: projection is a NODE property and the auto-rule is LOCAL. With
    ``BLIND_PROJECTION_AUTO_SEEK`` on (the default), a blind hop whose OWN
    destination is a projection node seeks that node's line first to re-anchor
    before the dead-reckoned slide. For the pivot ``(A, B, C)`` the incoming leg
    ``A->B`` lands on the projection node ``B``, so ``A->B`` is SENSOR-terminated
    and takes no approach trim -- a 90 deg pivot whose short (2.0 cm) incoming
    edge would otherwise reject as a too-short dead-reckon must validate. (The
    outgoing leg ``B->C`` is a plain BLIND into a line node, dead-reckoned.)
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 2.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
        # B is the projection node (the incoming A->B Blind->Proj hop seeks +
        # re-anchors on B's line). A is a blind dead-reckon origin; C is a blind
        # pivot exit reached by the plain B->C dead-reckon.
        node_types={"A": "blind", "B": "projection", "C": "blind", "D": "blind"},
    )
    assert validate(fd) is None


def test_pivot_turn_incoming_blind_into_projection_node_dead_reckons_when_auto_off() -> None:
    """TC15 (control): auto-seek OFF removes the trigger, so the leg dead-reckons.

    Disabling ``BLIND_PROJECTION_AUTO_SEEK`` removes the only seek trigger on the
    incoming ``A->B`` leg (no override, not STOP_AT_LINE, even though ``B`` is a
    projection node), so it classifies 'distance' again and the short 2.0 cm
    incoming edge is rejected -- proving the accept above is the auto-rule firing,
    not an unrelated exemption.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_BLIND, "u": "A", "v": "B",
             "distance_cm": 2.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
        node_types={"A": "blind", "B": "projection", "C": "blind", "D": "blind"},
    )
    fd.BLIND_PROJECTION_AUTO_SEEK = False
    with pytest.raises(
        FieldDataError, match=r"the INCOMING dead-reckoned edge length"
    ):
        validate(fd)


# -- TC5 / TC18: the LINE->projection regime is DISTANCE-terminated ------------
#
# Issue #64 silent-failure trap: a LINE edge into a projection node has NO
# junction to overshoot onto, so the pivot regime flips from 'line' (overshoot-
# budget) to 'distance' (follow-distance budget). A pivot APPROACHING a
# projection node over a LINE edge therefore draws its delta from the follow
# distance -- the validator must reject delta > follow distance there, and must
# NOT (falsely) treat it as overshoot-budgeted.


def test_pivot_turn_line_into_projection_node_is_distance_terminated() -> None:
    """TC5/TC18: a LINE->projection incoming pivot is DISTANCE-terminated and trims OK.

    The incoming leg ``A->B`` is a LINE edge into the projection node ``B``, so
    its regime is ``"distance"`` (not ``"line"``): the over-travel ``delta``
    bites the FOLLOW distance, not the overshoot budget. A 90 deg pivot
    (delta = 9.0 cm) onto a LONG (100 cm) LINE->projection edge clears the
    follow-distance floor and validates. (``A`` is the line origin; ``C`` is a
    plain blind pivot exit.)
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_LINE, "u": "A", "v": "B",
             "distance_cm": 100.0, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
        # B is the projection node the LINE A->B distance-terminates on; A is the
        # line origin (LINE-incident via A->B). C is the blind pivot exit.
        node_types={"A": "line", "B": "projection", "C": "blind", "D": "blind"},
    )
    assert validate(fd) is None


def test_pivot_turn_line_into_projection_node_rejects_delta_over_follow_distance() -> None:
    """TC18 (reject): a LINE->projection pivot whose delta exceeds the FOLLOW distance.

    Because the LINE->projection leg is DISTANCE-terminated, a 90 deg pivot
    (delta = 9.0 cm) onto a SHORT (9.3 cm) LINE->projection incoming edge leaves
    0.3 cm < the 0.5 cm trim floor and is rejected by the SAME dead-reckon floor
    gate the plain-BLIND distance regime uses -- NOT the overshoot budget. The
    error names the INCOMING dead-reckoned edge (the follow distance), proving
    the regime flip is in force: a 'line' regime would have budgeted this against
    the 12.02 cm overshoot offset and wrongly accepted it.
    """
    fd = _pivot_field(
        edges=[
            {"type": EDGE_LINE, "u": "A", "v": "B",
             "distance_cm": 9.3, "heading_deg": 0.0},
            {"type": EDGE_BLIND, "u": "B", "v": "C",
             "distance_cm": 100.0, "heading_deg": 90.0},
        ],
        pivot_turns=[("A", "B", "C")],
        node_types={"A": "line", "B": "projection", "C": "blind", "D": "blind"},
    )
    with pytest.raises(
        FieldDataError, match=r"the INCOMING dead-reckoned edge length"
    ):
        validate(fd)


# -- TC13: wall-zero reset nodes (_validate_reset_nodes) ----------------------
#
# A reset node re-localizes the chassis against a known wall. It is a normal
# node (entry in NODES + NODE_TYPES with the value "reset") PLUS a separate
# top-level sparse table RESET_NODES, keyed by node name, mapping each
# "reset"-typed node to {"axis", "wall_coord", "facing"}. RESET_NODES is
# OPTIONAL (absent => empty dict). The validator cross-checks the "reset"
# typing both directions and fail-fasts on a malformed spec. Every fixture
# here is an inline SimpleNamespace -- field_data.json is never touched.


def _reset_field(
    node_types_overrides=None, reset_nodes=None, start_node="S"
) -> SimpleNamespace:
    """Build a minimal S/A/B field carrying the given NODE_TYPES + RESET_NODES.

    Real nodes A/B joined by a LINE segment, plus a blind dead-reckon spawn ``S``
    on a BLIND edge into ``B``. ``A`` is the candidate reset node: by default it
    is typed ``"reset"`` and carries a valid RESET_NODES entry, so the base field
    validates. The START_NODE defaults to the blind spawn ``S`` -- issue #64's
    validator rejects an on-line (``line`` / ``projection``) START as well as a
    reset-typed START, so the spawn must be a blind dead-reckon origin. Pass
    ``start_node="A"`` to exercise the reset-START guard. Tests pass
    ``node_types_overrides`` / ``reset_nodes`` to author the specific malformed
    (or absent) case; ``S`` is always re-typed ``"blind"`` AFTER any override so
    callers need not carry it.

    The default spec re-zeroes against a +y wall at ``wall_coord=220`` (A is at
    y=200, so the standoff is 20 cm -- clear of the ``WALL_PIVOT_CLEARANCE_CM``
    floor) with ``facing=270`` (sin(270)=-1, pointing -y AWAY from the +y wall so
    the rear presses it). Both the clearance floor and the away-facing geometry
    gate pass, so callers that expect ``validate(...) is None`` keep passing.
    """
    node_types = {"A": "reset", "B": "line"}
    if node_types_overrides is not None:
        node_types = dict(node_types_overrides)
    # The blind spawn S is a fixture detail (the legal dead-reckon origin), not
    # part of the reset-typing cases the overrides author -- inject it AFTER any
    # override so callers stay focused on A/B.
    node_types["S"] = "blind"
    if reset_nodes is None:
        reset_nodes = {"A": {"axis": "y", "wall_coord": 220.0, "facing": 270.0}}
    return SimpleNamespace(
        NODES={"A": (12.0, 200.0), "B": (12.0, 50.0), "S": (60.0, 50.0)},
        NODE_TYPES=node_types,
        LINE_SEGMENTS=[("A", "B")],
        BLIND_EDGES=[("S", "B")],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE=start_node,
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        RESET_NODES=reset_nodes,
    )


def test_reset_node_valid_field_passes() -> None:
    """TC13: a well-formed reset node + RESET_NODES entry validates."""
    assert validate(_reset_field()) is None


def test_reset_node_absent_table_passes() -> None:
    """A reset-free field (no "reset" type, empty RESET_NODES) validates.

    The validator early-returns when both the RESET_NODES table and the set of
    "reset"-typed nodes are empty, so a plain LINE/BLIND field is unaffected.
    """
    fd = _reset_field(
        node_types_overrides={"A": "line", "B": "line"},
        reset_nodes={},
    )
    assert validate(fd) is None


def test_reset_node_missing_attr_treated_as_empty() -> None:
    """A field with no RESET_NODES attribute at all is reset-free and valid.

    ``getattr(field_data, "RESET_NODES", {})`` defaults the optional table, so
    a field that never authored one (and types no node "reset") passes.
    """
    fd = _reset_field(node_types_overrides={"A": "line", "B": "line"})
    delattr(fd, "RESET_NODES")
    assert validate(fd) is None


def test_reset_node_axis_not_x_or_y_rejected() -> None:
    """TC13: axis outside {x, y} is rejected with a precise key path."""
    fd = _reset_field(
        reset_nodes={"A": {"axis": "z", "wall_coord": 205.0, "facing": 90.0}}
    )
    with pytest.raises(FieldDataError, match=r"RESET_NODES\['A'\]\.axis"):
        validate(fd)


def test_reset_node_facing_360_rejected() -> None:
    """TC13: facing == 360 is out of the [0, 360) range and rejected."""
    fd = _reset_field(
        reset_nodes={"A": {"axis": "y", "wall_coord": 205.0, "facing": 360.0}}
    )
    with pytest.raises(FieldDataError, match=r"RESET_NODES\['A'\]\.facing"):
        validate(fd)


def test_reset_node_facing_negative_rejected() -> None:
    """TC13: a negative facing is out of [0, 360) and rejected."""
    fd = _reset_field(
        reset_nodes={"A": {"axis": "y", "wall_coord": 205.0, "facing": -1.0}}
    )
    with pytest.raises(FieldDataError, match=r"RESET_NODES\['A'\]\.facing"):
        validate(fd)


def test_reset_node_start_node_typed_reset_rejected() -> None:
    """#3: a START node typed 'reset' is rejected (the interlude never runs at start).

    The default fixture types A 'reset'; pointing START_NODE at A trips the guard
    before any spec-level check. A is at y=200 with a valid wall (so only the
    START guard, not a spec error, is the cause).
    """
    fd = _reset_field(start_node="A")
    with pytest.raises(FieldDataError, match=r"START_NODE 'A' is typed 'reset'"):
        validate(fd)


def test_reset_node_facing_toward_wall_rejected() -> None:
    """TC13: facing must point AWAY from the wall, not toward it.

    A is at y=200 and the wall is at y=220 (toward +y, standoff 20 cm clears the
    pivot floor), but facing 90 deg points +y -- the FRONT would press into the
    wall instead of the rear. With the floor satisfied, the away-facing geometry
    gate is the one that rejects (the firmware presses REARWARD, along facing+180).
    """
    fd = _reset_field(
        reset_nodes={"A": {"axis": "y", "wall_coord": 220.0, "facing": 90.0}}
    )
    with pytest.raises(FieldDataError, match=r"RESET_NODES\['A'\]\.facing"):
        validate(fd)


def test_reset_node_facing_away_from_wall_negative_axis_passes() -> None:
    """A wall on the -y side with facing 90 deg (+y) points AWAY from it: valid."""
    # A is at y=200; wall at y=50 (toward -y, standoff 150 cm); facing 90 deg
    # points +y so the FRONT faces away and the rear presses the -y wall:
    # sin(90)=+1, +1 * (50 - 200) = -150 < 0 -> away/valid.
    fd = _reset_field(
        reset_nodes={"A": {"axis": "y", "wall_coord": 50.0, "facing": 90.0}}
    )
    assert validate(fd) is None


def test_reset_node_on_wall_rejected_by_clearance_floor() -> None:
    """A node authored exactly on the wall (zero gap) is rejected by the floor.

    With wall_coord == the node coordinate the standoff is 0 cm, below the
    WALL_PIVOT_CLEARANCE_CM pivot floor -- the chassis pivots in place around the
    wall-zero and would clip the wall. The floor gate fires before the facing
    geometry check, so the error names the pivot clearance, not ``.facing``.
    """
    fd = _reset_field(
        reset_nodes={"A": {"axis": "y", "wall_coord": 200.0, "facing": 270.0}}
    )
    with pytest.raises(FieldDataError, match=r"pivot clearance floor"):
        validate(fd)


def test_reset_node_standoff_at_clearance_floor_passes() -> None:
    """The standoff boundary is INCLUSIVE: exactly WALL_PIVOT_CLEARANCE_CM passes.

    A is at y=200; wall at y=214 gives a 14.0 cm standoff == the floor. The floor
    test is a strict ``<``, so the boundary is accepted. facing 270 (sin=-1) points
    -y away from the +y wall, so the geometry gate passes too.
    """
    fd = _reset_field(
        reset_nodes={
            "A": {
                "axis": "y",
                "wall_coord": 200.0 + WALL_PIVOT_CLEARANCE_CM,
                "facing": 270.0,
            }
        }
    )
    assert validate(fd) is None


def test_reset_node_standoff_below_clearance_floor_rejected() -> None:
    """A standoff just under WALL_PIVOT_CLEARANCE_CM is rejected by the floor."""
    fd = _reset_field(
        reset_nodes={
            "A": {
                "axis": "y",
                "wall_coord": 200.0 + WALL_PIVOT_CLEARANCE_CM - 0.01,
                "facing": 270.0,
            }
        }
    )
    with pytest.raises(FieldDataError, match=r"pivot clearance floor"):
        validate(fd)


def test_reset_node_orthogonal_facing_rejected() -> None:
    """A facing orthogonal to the reset axis (zero on-axis component) is rejected.

    A is at y=200; wall at y=220 (standoff 20 cm clears the floor). facing 0 deg
    is parallel to the wall (sin(0)=0), so its on-axis component is 0 -- neither
    toward nor away. ``comp * toward = 0 >= 0`` trips the facing gate: the rear
    never squarely faces the wall, so the press has no defined direction.
    """
    fd = _reset_field(
        reset_nodes={"A": {"axis": "y", "wall_coord": 220.0, "facing": 0.0}}
    )
    with pytest.raises(FieldDataError, match=r"RESET_NODES\['A'\]\.facing"):
        validate(fd)


def test_reset_node_wall_coord_non_finite_rejected() -> None:
    """TC13: a non-finite wall_coord (NaN / Inf) is rejected.

    Mirrors the finiteness gate used everywhere else in the validator: a NaN
    is a float but slips through downstream numeric math, so it must fail with
    a clean FieldDataError at the field-data layer.
    """
    fd = _reset_field(
        reset_nodes={
            "A": {"axis": "y", "wall_coord": float("inf"), "facing": 90.0}
        }
    )
    with pytest.raises(FieldDataError, match=r"RESET_NODES\['A'\]\.wall_coord"):
        validate(fd)


def test_reset_node_key_not_in_nodes_rejected() -> None:
    """TC13: a RESET_NODES key that is not a NODES entry is rejected.

    No node is typed "reset" here, so the consistency check passes and the
    unknown-key gate is the one that trips on the bogus ``ZZZ`` key.
    """
    fd = _reset_field(
        node_types_overrides={"A": "line", "B": "line"},
        reset_nodes={
            "ZZZ": {"axis": "y", "wall_coord": 205.0, "facing": 90.0}
        },
    )
    with pytest.raises(
        FieldDataError, match=r"RESET_NODES references unknown node: 'ZZZ'"
    ):
        validate(fd)


def test_reset_node_typed_reset_without_entry_rejected() -> None:
    """TC13: a node typed "reset" with NO RESET_NODES entry is rejected.

    Consistency, one direction: typing a node "reset" without authoring its
    {axis, wall_coord, facing} spec leaves the re-localization undefined.
    """
    fd = _reset_field(reset_nodes={})  # A is still typed "reset"
    with pytest.raises(
        FieldDataError, match=r"node 'A' is typed 'reset' .* has no\s+RESET_NODES entry"
    ):
        validate(fd)


def test_reset_node_entry_without_reset_type_rejected() -> None:
    """TC13: a RESET_NODES entry whose node is NOT typed "reset" is rejected.

    Consistency, the other direction: authoring a RESET_NODES spec for a node
    typed "line" (or anything but "reset") is contradictory and must fail.
    """
    fd = _reset_field(
        node_types_overrides={"A": "line", "B": "line"},
        reset_nodes={"A": {"axis": "y", "wall_coord": 205.0, "facing": 90.0}},
    )
    with pytest.raises(
        FieldDataError, match=r"RESET_NODES\['A'\] is set but NODE_TYPES\['A'\]"
    ):
        validate(fd)


def test_reset_node_missing_spec_key_rejected() -> None:
    """TC13: a RESET_NODES spec missing a required key is rejected."""
    fd = _reset_field(reset_nodes={"A": {"axis": "y", "facing": 90.0}})
    with pytest.raises(
        FieldDataError, match=r"RESET_NODES\['A'\] missing required key 'wall_coord'"
    ):
        validate(fd)


def test_reset_node_type_accepted_in_node_types() -> None:
    """TC13: "reset" is now an allowed NODE_TYPES value (alongside line/blind)."""
    fd = _reset_field()
    # Sanity: the "reset" typing on A did not trip the NODE_TYPES value gate.
    assert validate(fd) is None
    # A bogus type still fails, proving the gate is not a blanket pass.
    fd_bad = _reset_field(node_types_overrides={"A": "weird", "B": "line"})
    with pytest.raises(FieldDataError, match=r"NODE_TYPES\['A'\]"):
        validate(fd_bad)


# -- T21: blind-distance decay knobs (_validate_blind_decay) ------------------
#
# The four BLIND_DECAY_* top-level scalars parameterise the planner's blind-edge
# cost penalty (T22): weight = length * (1 + k * (d / d_ref) ** exponent). They
# replace the old COST_BLIND_MULTIPLIER knob and are authored in field_data.json
# as plain JSON scalars (ADR-022). The validator enforces each value's domain:
# enabled is a bool, k >= 0, exponent >= 1, d_ref > 0. The base clone
# (_clone_field_data) carries the valid authored defaults, so each test mutates
# one knob to the offending value.


def test_decay_default_knobs_pass() -> None:
    """The toy fixture's authored decay knobs validate as-is."""
    fd = _clone_field_data()
    assert validate(fd) is None


def test_decay_enabled_non_bool_rejected() -> None:
    """BLIND_DECAY_ENABLED must be a bool, not a truthy int."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_ENABLED = 1
    with pytest.raises(FieldDataError, match=r"BLIND_DECAY_ENABLED"):
        validate(fd)


def test_decay_enabled_false_accepted() -> None:
    """BLIND_DECAY_ENABLED=False is a valid bool and validates."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_ENABLED = False
    assert validate(fd) is None


def test_decay_k_negative_rejected() -> None:
    """BLIND_DECAY_K must be >= 0; a negative weight is rejected."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_K = -0.1
    with pytest.raises(FieldDataError, match=r"BLIND_DECAY_K"):
        validate(fd)


def test_decay_k_zero_accepted() -> None:
    """BLIND_DECAY_K == 0 disables the penalty and is a valid value."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_K = 0.0
    assert validate(fd) is None


def test_decay_k_non_finite_rejected() -> None:
    """A non-finite BLIND_DECAY_K (NaN / Inf) is rejected at the numeric gate."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_K = float("inf")
    with pytest.raises(FieldDataError, match=r"BLIND_DECAY_K"):
        validate(fd)


def test_decay_exponent_below_one_rejected() -> None:
    """BLIND_DECAY_EXPONENT must be >= 1; a sub-linear exponent is rejected."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_EXPONENT = 0.5
    with pytest.raises(FieldDataError, match=r"BLIND_DECAY_EXPONENT"):
        validate(fd)


def test_decay_exponent_at_one_accepted() -> None:
    """BLIND_DECAY_EXPONENT == 1 sits exactly at the inclusive floor."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_EXPONENT = 1.0
    assert validate(fd) is None


def test_decay_d_ref_zero_rejected() -> None:
    """BLIND_DECAY_D_REF must be > 0; zero would divide by zero downstream."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_D_REF = 0.0
    with pytest.raises(FieldDataError, match=r"BLIND_DECAY_D_REF"):
        validate(fd)


def test_decay_d_ref_negative_rejected() -> None:
    """A negative BLIND_DECAY_D_REF is rejected."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_D_REF = -50.0
    with pytest.raises(FieldDataError, match=r"BLIND_DECAY_D_REF"):
        validate(fd)


def test_decay_missing_required_attr_rejected() -> None:
    """A field missing a BLIND_DECAY_* attribute fails the structural check."""
    fd = _clone_field_data()
    delattr(fd, "BLIND_DECAY_K")
    with pytest.raises(
        FieldDataError, match=r"missing required attribute: BLIND_DECAY_K"
    ):
        validate(fd)


def test_decay_exponent_above_max_rejected() -> None:
    """BLIND_DECAY_EXPONENT above the cap is rejected before it can overflow.

    Without a ceiling a fat-fingered exponent (e.g. 400) makes
    ``(d / d_ref) ** exponent`` overflow to an uncaught OverflowError deep in the
    planner; the validator caps it at ``BLIND_DECAY_EXPONENT_MAX`` so the value
    fails here with a precise message instead.
    """
    fd = _clone_field_data()
    fd.BLIND_DECAY_EXPONENT = 400.0
    with pytest.raises(FieldDataError, match=r"BLIND_DECAY_EXPONENT"):
        validate(fd)


def test_decay_exponent_at_max_accepted() -> None:
    """BLIND_DECAY_EXPONENT == BLIND_DECAY_EXPONENT_MAX sits at the inclusive ceiling."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_EXPONENT = BLIND_DECAY_EXPONENT_MAX
    assert validate(fd) is None


def test_decay_exponent_non_finite_rejected() -> None:
    """A non-finite BLIND_DECAY_EXPONENT (inf / NaN) is rejected at the numeric gate."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_EXPONENT = float("inf")
    with pytest.raises(FieldDataError, match=r"BLIND_DECAY_EXPONENT"):
        validate(fd)


def test_decay_d_ref_non_finite_rejected() -> None:
    """A non-finite BLIND_DECAY_D_REF (NaN) is rejected at the numeric gate."""
    fd = _clone_field_data()
    fd.BLIND_DECAY_D_REF = float("nan")
    with pytest.raises(FieldDataError, match=r"BLIND_DECAY_D_REF"):
        validate(fd)


# -- Fix A (ADR-041): a line-follow edge cannot be incident to a blind node ----
#
# A LINE edge is a line-follow segment; it needs a line under the sensor, so it
# cannot terminate on a node typed "blind". Beyond being physically impossible,
# such a seam would let the planner's blind-decay inf->factor-1 fallback silently
# under-price the pocket behind it. A blind node must be reached by a BLIND edge
# instead. (Issue #64 retired the projection EDGE type; a LINE edge into a
# projection node stays legal -- the line passes through it.)


def test_line_segment_into_blind_node_rejected() -> None:
    """A LINE_SEGMENTS edge terminating on a blind node is rejected.

    'G' is the toy field's blind node; a line-follow segment to it is invalid.
    """
    fd = _clone_field_data()
    fd.LINE_SEGMENTS = list(fd.LINE_SEGMENTS) + [("A", "G")]
    with pytest.raises(
        FieldDataError, match=r"LINE_SEGMENTS.*incident to blind node 'G'"
    ):
        validate(fd)


def test_line_edges_override_into_blind_node_rejected() -> None:
    """A LINE EDGES override into a blind node is rejected too.

    Issue #64 retired the projection EDGE type; the line-follow-into-blind guard
    now fires on a LINE EDGES override (the same path the old PROJECTION override
    exercised). 'G' is the toy field's blind node.
    """
    fd = _clone_field_data()
    fd.EDGES = [
        {"type": EDGE_LINE, "u": "A", "v": "G",
         "distance_cm": 50.0, "heading_deg": 0.0},
    ]
    with pytest.raises(
        FieldDataError, match=r"EDGES\[0\].*incident to blind node 'G'"
    ):
        validate(fd)


def test_blind_edge_into_blind_node_still_accepted() -> None:
    """A BLIND edge into a blind node stays valid -- only line-follow seams are barred."""
    fd = _clone_field_data()
    # 'G' is blind and ('B','G') is already the toy's blind edge; a second blind
    # hop A->G into G must NOT trip the line-follow rule.
    fd.BLIND_EDGES = list(fd.BLIND_EDGES) + [("A", "G")]
    assert validate(fd) is None


# -- TC26: validator parse-helper / firmware Config.h parity ------------------
#
# The validator single-sources its pivot constants from WRO2026/Config.h
# (mirroring planner.parse_track_cm_from_config). These parity checks assert
# the parsed values equal the firmware literals so an offline check can never
# desync from the value the robot drives with.


def test_parse_line_sensor_axle_offset_matches_config() -> None:
    """TC26: parse_line_sensor_axle_offset_cm() equals Config::LINE_SENSOR_AXLE_OFFSET_CM."""
    assert parse_line_sensor_axle_offset_cm() == pytest.approx(12.02)


def test_parse_pivot_min_turn_deg_matches_config() -> None:
    """TC26: parse_pivot_min_turn_deg_from_config() equals Config::PIVOT_MIN_TURN_DEG.

    The minimum authored pivot angle is a tunable Config.h knob the validator
    single-sources (it replaced the old ``3 x NAV_TURN_TOLERANCE_DEG`` derived
    floor). Pin the parsed value to the firmware literal so a retune of the floor
    that desyncs the offline check fails the suite.
    """
    assert parse_pivot_min_turn_deg_from_config() == pytest.approx(5.0)


def test_parse_pivot_trim_min_drive_matches_config() -> None:
    """TC26: parse_pivot_trim_min_drive_cm_from_config() equals Config::PIVOT_TRIM_MIN_DRIVE_CM.

    T8 added ``constexpr float PIVOT_TRIM_MIN_DRIVE_CM = 0.5f`` to the firmware
    header, so the parser no longer raises and the old skip is gone. Pin the
    parsed value to the firmware literal (and the sim's mirror of it in
    ``sim/runtime.py``) so a Config.h edit that desyncs the validator/sim/
    firmware floor fails the suite instead of slipping through a ``> 0.0`` check.
    """
    assert parse_pivot_trim_min_drive_cm_from_config() == pytest.approx(0.5)
