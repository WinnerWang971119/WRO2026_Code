"""Tests for FieldModel snapshot/restore, JSON round-trip, and validator integration.

Covers TC2 (JSON round-trip equality) and TC12 (validator rejects malformed model).
No GUI, no file I/O -- everything is built in memory.
"""

from __future__ import annotations

import pytest

from tools.path_planner.editor.model import FieldModel
from tools.path_planner.validator import FieldDataError


# ---------------------------------------------------------------------------
# Fixture factory
# ---------------------------------------------------------------------------


def make_model() -> FieldModel:
    """Return a small but fully-valid FieldModel built in memory."""
    return FieldModel(
        field_mm=(3000, 2000),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        cost_blind_multiplier=5.0,
        start_node="A",
        start_heading_deg=0.0,
        nodes={"A": (10.0, 10.0), "B": (200.0, 10.0)},
        node_types={"A": "line", "B": "line"},
        line_segments=[("A", "B")],
        blind_edges=[],
        obstacles=[(50.0, 50.0, 80.0, 80.0, "pillar")],
    )


# ---------------------------------------------------------------------------
# snapshot / restore tests
# ---------------------------------------------------------------------------


def test_snapshot_is_deep_copy() -> None:
    """Mutating any nested container in a snapshot must not affect the original."""
    model = make_model()
    snap = model.snapshot()

    # Mutate every mutable container in the snapshot.
    snap.nodes["C"] = (999.0, 999.0)
    snap.node_types["C"] = "blind"
    snap.line_segments.append(("B", "C"))
    snap.blind_edges.append(("A", "C"))
    snap.obstacles.append((0.0, 0.0, 1.0, 1.0, "wall"))

    # Original must be untouched.
    assert "C" not in model.nodes
    assert "C" not in model.node_types
    assert len(model.line_segments) == 1
    assert len(model.blind_edges) == 0
    assert len(model.obstacles) == 1


def test_restore_overwrites_in_place() -> None:
    """restore() must update every field value while keeping the same object identity."""
    model = make_model()
    snap = model.snapshot()
    original_id = id(model)

    # Mutate the live model.
    model.nodes["C"] = (500.0, 500.0)
    model.node_types["C"] = "blind"
    model.start_node = "B"
    model.start_heading_deg = 90.0

    model.restore(snap)

    # Same object, values back to the snapshot state.
    assert id(model) == original_id
    assert model.nodes == snap.nodes
    assert model.node_types == snap.node_types
    assert model.start_node == snap.start_node
    assert model.start_heading_deg == snap.start_heading_deg
    assert "C" not in model.nodes
    assert "C" not in model.node_types


# ---------------------------------------------------------------------------
# JSON round-trip tests (TC2)
# ---------------------------------------------------------------------------


def test_round_trip_to_from_json_dict_equality() -> None:
    """TC2: FieldModel.from_json_dict(model.to_json_dict()) == original model."""
    model = make_model()
    restored = FieldModel.from_json_dict(model.to_json_dict())
    assert restored == model


def test_to_json_dict_top_level_key_order() -> None:
    """to_json_dict() must emit exactly the 11 keys in the canonical order."""
    model = make_model()
    expected_keys = (
        "FIELD_MM",
        "MAP_PNG",
        "ROBOT_RADIUS_CM",
        "COST_BLIND_MULTIPLIER",
        "START_NODE",
        "START_HEADING_DEG",
        "NODES",
        "NODE_TYPES",
        "LINE_SEGMENTS",
        "BLIND_EDGES",
        "OBSTACLES",
    )
    assert tuple(model.to_json_dict().keys()) == expected_keys


def test_to_json_dict_nodes_sorted_alphabetically() -> None:
    """NODES dict in to_json_dict() output must have alphabetically sorted keys."""
    model = make_model()
    # Insert nodes in reverse-alphabetical order to stress the sort.
    model.nodes["Z"] = (100.0, 100.0)
    model.node_types["Z"] = "line"
    model.nodes["M"] = (50.0, 50.0)
    model.node_types["M"] = "blind"

    json_nodes = list(model.to_json_dict()["NODES"].keys())
    assert json_nodes == sorted(json_nodes)


# ---------------------------------------------------------------------------
# Validator integration tests (TC12)
# ---------------------------------------------------------------------------


def test_validate_passes_on_well_formed_model() -> None:
    """validate() must return None (no exception) for a well-formed model."""
    model = make_model()
    result = model.validate()
    assert result is None


def test_validate_rejects_node_types_coverage_gap() -> None:
    """TC12: removing a NODE_TYPES entry for an existing node must raise FieldDataError
    mentioning the missing node name."""
    model = make_model()
    # "B" is in nodes but no longer in node_types.
    del model.node_types["B"]

    with pytest.raises(FieldDataError) as exc_info:
        model.validate()

    assert "B" in str(exc_info.value)


def test_validate_rejects_unknown_node_in_line_segments() -> None:
    """A line_segment referencing a node not in NODES must raise FieldDataError
    mentioning the unknown node name."""
    model = make_model()
    model.line_segments.append(("A", "ZZZ"))

    with pytest.raises(FieldDataError) as exc_info:
        model.validate()

    assert "ZZZ" in str(exc_info.value)
