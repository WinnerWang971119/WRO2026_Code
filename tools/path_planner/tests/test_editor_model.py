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
    """to_json_dict() must emit the 12 keys in the canonical order when a pivot is set.

    ``pivot_turns`` is optional and emitted only when non-empty (see
    ``test_to_json_dict_omits_empty_pivot_turns``), so this canonical-slot check
    authors one pivot to make the key present in its slot between BLIND_EDGES and
    OBSTACLES.
    """
    model = make_model()
    model.pivot_turns = [("A", "B", "C")]
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
        "pivot_turns",
        "OBSTACLES",
    )
    assert tuple(model.to_json_dict().keys()) == expected_keys


def test_to_json_dict_omits_empty_pivot_turns() -> None:
    """A pivot-free model omits the optional ``pivot_turns`` key entirely.

    Matches the loader's ``dump_field_json`` "absent stays absent" round-trip so
    opening + saving a pivot-free field through the model produces no spurious
    ``pivot_turns: []`` diff against a loader-path dump.
    """
    model = make_model()
    assert model.pivot_turns == []
    json_dict = model.to_json_dict()
    assert "pivot_turns" not in json_dict
    # The remaining 11 keys keep their canonical order; OBSTACLES stays last.
    assert tuple(json_dict.keys())[-1] == "OBSTACLES"
    assert len(json_dict) == 11


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
# pivot_turns preservation: the editor has no GUI control for pivots, so the
# model must carry the field verbatim across open (from_json_dict) -> save
# (to_json_dict) without stripping it.
# ---------------------------------------------------------------------------


def test_pivot_turns_defaults_to_empty_list() -> None:
    """A model constructed without pivot_turns has an empty list, not an error.

    The dataclass field is defaulted so existing call sites that build a
    FieldModel without pivots stay valid.
    """
    model = make_model()
    assert model.pivot_turns == []


def test_pivot_turns_survives_to_from_json_round_trip() -> None:
    """from_json_dict(to_json_dict()) preserves an authored pivot_turns list.

    Open -> save must not drop the field. The triple is carried as a
    (u, v, w) tuple in memory and a 3-element list in JSON.
    """
    model = make_model()
    model.pivot_turns = [("A", "B", "C")]

    restored = FieldModel.from_json_dict(model.to_json_dict())

    assert restored.pivot_turns == [("A", "B", "C")]
    assert restored == model


def test_pivot_turns_emitted_as_lists_in_json_dict() -> None:
    """to_json_dict() renders pivot_turns as JSON-native lists of strings."""
    model = make_model()
    model.pivot_turns = [("A", "B", "C")]

    json_dict = model.to_json_dict()

    assert json_dict["pivot_turns"] == [["A", "B", "C"]]
    # No tuples leak into the JSON-native output.
    assert all(isinstance(turn, list) for turn in json_dict["pivot_turns"])


def test_from_json_dict_tolerates_absent_pivot_turns() -> None:
    """A JSON dict predating pivot_turns loads with an empty list.

    Older field_data.json files omit the optional key; from_json_dict must
    default it rather than raise KeyError. A pivot-free model's to_json_dict
    already omits the key, so this dict has no ``pivot_turns`` to begin with.
    """
    data = make_model().to_json_dict()
    assert "pivot_turns" not in data

    model = FieldModel.from_json_dict(data)

    assert model.pivot_turns == []


def test_validate_enforces_pivot_geometry_via_proxy() -> None:
    """model.validate() reaches the pivot rules because the proxy exposes PIVOT_TURNS.

    Before ``to_module_proxy`` forwarded ``PIVOT_TURNS``,
    :func:`validator._validate_pivot_turns` hit its
    ``getattr(..., 'PIVOT_TURNS', None)`` early-return and the editor's Save
    silently accepted any pivot. This authors a pivot whose outgoing leg
    ``B->C`` is not an edge; a proxy that carries the field makes ``validate()``
    reject it instead of deferring the failure to build time.
    """
    model = FieldModel(
        field_mm=(3000, 2000),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        cost_blind_multiplier=5.0,
        start_node="A",
        start_heading_deg=0.0,
        nodes={
            "A": (10.0, 10.0),
            "B": (200.0, 10.0),
            "C": (200.0, 200.0),
            "D": (10.0, 200.0),
        },
        node_types={"A": "line", "B": "line", "C": "line", "D": "line"},
        line_segments=[("A", "B"), ("B", "D"), ("C", "D")],
        blind_edges=[],
        obstacles=[],
    )
    model.pivot_turns = [("A", "B", "C")]  # B->C is not an authored edge
    with pytest.raises(
        FieldDataError, match=r"outgoing edge\s+\('B','C'\) that is not authored"
    ):
        model.validate()


def test_pivot_turns_open_save_open_preservation(tmp_path) -> None:
    """An on-disk field with pivots survives a loader/model open -> save -> open.

    Exercises the full editor preservation path the GUI uses: load the JSON
    (loader), build the model (from_json_dict), save it (to_json_dict ->
    dump_field_json), then reload. The pivot must come back unchanged.
    """
    import json
    from pathlib import Path

    from tools.path_planner.editor.loader import dump_field_json, load_field_json

    # Author a small field carrying a pivot directly in JSON.
    source = make_model().to_json_dict()
    source["pivot_turns"] = [["A", "B", "C"]]
    src_path = Path(tmp_path) / "field_with_pivots.json"
    src_path.write_text(json.dumps(source, indent=2) + "\n", encoding="utf-8", newline="\n")

    # Open -> build model -> save (the editor's Save flow).
    opened = FieldModel.from_json_dict(load_field_json(src_path))
    assert opened.pivot_turns == [("A", "B", "C")]

    saved_path = Path(tmp_path) / "field_with_pivots_saved.json"
    dump_field_json(opened.to_json_dict(), saved_path)

    # Reopen and confirm the pivot survived the full round-trip.
    reopened = FieldModel.from_json_dict(load_field_json(saved_path))
    assert reopened.pivot_turns == [("A", "B", "C")]
    assert reopened == opened


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
