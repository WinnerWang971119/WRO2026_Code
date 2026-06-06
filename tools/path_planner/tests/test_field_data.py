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
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE
from tools.path_planner.pipeline_config import BlindMode, LineMode
from tools.path_planner.tests.fixtures import toy_field_data as real_field_data
from tools.path_planner.validator import FieldDataError, validate

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
