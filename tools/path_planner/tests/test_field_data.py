"""Tests for tools.path_planner.field_data and tools.path_planner.validator.

Covers TC1 (PROJECTIONS unknown segment), TC2 (GOAL_APPROACH unknown entry),
and TC3 (LINE_SEGMENTS unknown node) from the PR 1 plan, plus extra
guardrails on obstacles and node-type values.
"""

from __future__ import annotations

import copy
from types import SimpleNamespace

import pytest

from tools.path_planner import field_data as real_field_data
from tools.path_planner.validator import FieldDataError, validate

# Names of the public top-level attributes the validator inspects.
_FIELD_ATTRS = (
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "PROJECTIONS",
    "GOAL_APPROACH",
    "OBSTACLES",
    "ROBOT_RADIUS_CM",
    "COST_BLIND_MULTIPLIER",
    "START_NODE",
    "START_HEADING_DEG",
)


def _clone_field_data() -> SimpleNamespace:
    """Return a deep-copied SimpleNamespace mirror of the real field_data.

    Mutating the returned namespace (or any nested container) does **not**
    affect the live ``tools.path_planner.field_data`` module.
    """
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


def test_valid_fixture_passes() -> None:
    """The unmodified real field_data module must validate cleanly."""
    # Returns None on success; the assertion is simply 'no exception'.
    assert validate(real_field_data) is None


def test_projections_unknown_segment() -> None:
    """TC1: PROJECTIONS.on_segment must reference a known LINE_SEGMENTS entry."""
    fd = _clone_field_data()
    fd.PROJECTIONS["_proj_G"]["on_segment"] = ("Z", "Q")
    with pytest.raises(FieldDataError, match=r"PROJECTIONS.*is not a known LINE_SEGMENTS entry"):
        validate(fd)


def test_goal_approach_unknown_entry() -> None:
    """TC2: GOAL_APPROACH.entry must be a known node name."""
    fd = _clone_field_data()
    fd.GOAL_APPROACH["G"]["entry"] = "ZZZ"
    with pytest.raises(FieldDataError, match=r"GOAL_APPROACH.*unknown node"):
        validate(fd)


def test_line_segments_unknown_node() -> None:
    """TC3: every endpoint in LINE_SEGMENTS must exist in NODES."""
    fd = _clone_field_data()
    fd.LINE_SEGMENTS = [("A", "ZZZ")]
    with pytest.raises(FieldDataError, match=r"LINE_SEGMENTS.*unknown node"):
        validate(fd)


def test_obstacle_invalid_bounds() -> None:
    """OBSTACLES with x_min > x_max (or y_min > y_max) must be rejected."""
    fd = _clone_field_data()
    fd.OBSTACLES = [(50.0, 0.0, 40.0, 10.0, "wall")]
    with pytest.raises(FieldDataError, match=r"invalid bounds"):
        validate(fd)


def test_node_type_unknown_value() -> None:
    """NODE_TYPES values must be one of {'line','projection','goal'}."""
    fd = _clone_field_data()
    fd.NODE_TYPES["A"] = "weird"
    with pytest.raises(FieldDataError, match=r"NODE_TYPES\['A'\]"):
        validate(fd)


def test_node_in_segments_missing_from_nodes() -> None:
    """Removing a node referenced by LINE_SEGMENTS must trigger the segment check."""
    fd = _clone_field_data()
    # 'E' is referenced by ("A","E"), ("B","E"), ("C","E"), ("D","E").
    del fd.NODES["E"]
    # Also drop the dangling NODE_TYPES entry so the node-type-coverage step
    # (which fires before the segment-coverage step) does not pre-empt the
    # error we are actually testing for.
    del fd.NODE_TYPES["E"]
    with pytest.raises(FieldDataError, match=r"LINE_SEGMENTS.*unknown node"):
        validate(fd)
