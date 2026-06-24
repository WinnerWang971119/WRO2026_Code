"""Tests for FieldModel snapshot/restore, JSON round-trip, and validator integration.

Covers TC2 (JSON round-trip equality) and TC12 (validator rejects malformed model).
No GUI, no file I/O -- everything is built in memory.
"""

from __future__ import annotations

import pytest

from tools.path_planner.editor.commands import AddNode
from tools.path_planner.editor.history import History
from tools.path_planner.editor.model import FieldModel
from tools.path_planner.validator import FieldDataError


# ---------------------------------------------------------------------------
# Fixture factory
# ---------------------------------------------------------------------------


def make_model() -> FieldModel:
    """Return a small but fully-valid FieldModel built in memory.

    ``S`` is a dead-reckon spawn (a ``"blind"`` node) used as the ``START_NODE``:
    issue #64's validator rejects a ``"line"`` / ``"projection"`` start (an
    on-line spawn presumes a sensor-anchored arrival the chassis has not made at
    t=0), so the start must be a blind origin. ``S`` carries no incident edge --
    the blind-edge list stays empty so the snapshot deep-copy guard still sees
    ``blind_edges == []`` -- and an isolated blind node validates clean.
    """
    return FieldModel(
        field_mm=(3000, 2000),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="S",
        start_heading_deg=0.0,
        nodes={"S": (10.0, 100.0), "A": (10.0, 10.0), "B": (200.0, 10.0)},
        node_types={"S": "blind", "A": "line", "B": "line"},
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
    """to_json_dict() must emit the 15 keys in the canonical order when a pivot is set.

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
        "BLIND_DECAY_ENABLED",
        "BLIND_DECAY_K",
        "BLIND_DECAY_EXPONENT",
        "BLIND_DECAY_D_REF",
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
    # The remaining 14 keys keep their canonical order; OBSTACLES stays last.
    assert tuple(json_dict.keys())[-1] == "OBSTACLES"
    assert len(json_dict) == 14


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
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        # ``S`` is a blind dead-reckon spawn so the issue #64 line-/projection-
        # START rejection (which runs before the pivot rules) does not pre-empt
        # the pivot error this test asserts on.
        start_node="S",
        start_heading_deg=0.0,
        nodes={
            "S": (10.0, 100.0),
            "A": (10.0, 10.0),
            "B": (200.0, 10.0),
            "C": (200.0, 200.0),
            "D": (10.0, 200.0),
        },
        node_types={
            "S": "blind",
            "A": "line",
            "B": "line",
            "C": "line",
            "D": "line",
        },
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
# reset_nodes preservation (PR2 T12): like pivot_turns, the editor has no GUI
# control for wall-zero reset nodes, so the model must carry the sparse
# RESET_NODES table verbatim across open (from_json_dict) -> save
# (to_json_dict), expose it on the validator proxy, and deep-copy it through
# snapshot/restore so undo/redo preserves it.
# ---------------------------------------------------------------------------


def _reset_spec() -> dict:
    """Return one valid reset-node spec ``{axis, wall_coord, facing}``."""
    return {"axis": "y", "wall_coord": 205.0, "facing": 90.0}


def test_reset_nodes_defaults_to_empty_dict() -> None:
    """A model constructed without reset_nodes has an empty dict, not an error.

    The dataclass field is defaulted so existing call sites that build a
    FieldModel without reset nodes stay valid.
    """
    model = make_model()
    assert model.reset_nodes == {}


def test_reset_nodes_survives_to_from_json_round_trip() -> None:
    """from_json_dict(to_json_dict()) preserves an authored reset_nodes table.

    Open -> save must not drop the field. The spec dict is carried verbatim.
    """
    model = make_model()
    model.node_types["A"] = "reset"
    model.reset_nodes = {"A": _reset_spec()}

    restored = FieldModel.from_json_dict(model.to_json_dict())

    assert restored.reset_nodes == {"A": _reset_spec()}
    assert restored == model


def test_reset_nodes_emitted_as_dict_in_json_dict() -> None:
    """to_json_dict() renders RESET_NODES as a JSON-native dict of specs."""
    model = make_model()
    model.node_types["A"] = "reset"
    model.reset_nodes = {"A": _reset_spec()}

    json_dict = model.to_json_dict()

    assert json_dict["RESET_NODES"] == {"A": _reset_spec()}


def test_to_json_dict_omits_empty_reset_nodes() -> None:
    """A reset-free model omits the optional ``RESET_NODES`` key entirely.

    Matches the loader's ``dump_field_json`` "absent stays absent" round-trip so
    opening + saving a reset-free field through the model produces no spurious
    ``RESET_NODES: {}`` diff.
    """
    model = make_model()
    assert model.reset_nodes == {}
    json_dict = model.to_json_dict()
    assert "RESET_NODES" not in json_dict
    # OBSTACLES still closes the canonical order even with both optional keys
    # (pivot_turns + RESET_NODES) absent.
    assert tuple(json_dict.keys())[-1] == "OBSTACLES"


def test_to_json_dict_reset_nodes_in_canonical_slot() -> None:
    """RESET_NODES is emitted between pivot_turns and OBSTACLES when both are set."""
    model = make_model()
    model.node_types["A"] = "reset"
    model.pivot_turns = [("A", "B", "A")]  # shape only; not validated here
    model.reset_nodes = {"A": _reset_spec()}
    expected_keys = (
        "FIELD_MM",
        "MAP_PNG",
        "ROBOT_RADIUS_CM",
        "BLIND_DECAY_ENABLED",
        "BLIND_DECAY_K",
        "BLIND_DECAY_EXPONENT",
        "BLIND_DECAY_D_REF",
        "START_NODE",
        "START_HEADING_DEG",
        "NODES",
        "NODE_TYPES",
        "LINE_SEGMENTS",
        "BLIND_EDGES",
        "pivot_turns",
        "RESET_NODES",
        "OBSTACLES",
    )
    assert tuple(model.to_json_dict().keys()) == expected_keys


def test_reset_nodes_sorted_alphabetically_in_json_dict() -> None:
    """RESET_NODES keys are emitted alphabetically for a deterministic dump."""
    model = make_model()
    model.node_types["A"] = "reset"
    model.node_types["B"] = "reset"
    # Insert in reverse-alphabetical order to stress the sort.
    model.reset_nodes = {
        "B": {"axis": "x", "wall_coord": 10.0, "facing": 0.0},
        "A": _reset_spec(),
    }
    json_keys = list(model.to_json_dict()["RESET_NODES"].keys())
    assert json_keys == sorted(json_keys) == ["A", "B"]


def test_from_json_dict_tolerates_absent_reset_nodes() -> None:
    """A JSON dict predating RESET_NODES loads with an empty dict.

    Older field_data.json files omit the optional key; from_json_dict must
    default it rather than raise KeyError. A reset-free model's to_json_dict
    already omits the key, so this dict has no ``RESET_NODES`` to begin with.
    """
    data = make_model().to_json_dict()
    assert "RESET_NODES" not in data

    model = FieldModel.from_json_dict(data)

    assert model.reset_nodes == {}


def test_to_module_proxy_exposes_reset_nodes() -> None:
    """to_module_proxy() carries RESET_NODES so validate() actually sees it.

    The dual-serialization trap: a field added to to_json_dict() but not to the
    proxy is persisted-but-unvalidated (the validator early-returns on the
    missing attribute). The proxy must expose RESET_NODES with the exact dict
    the model holds.
    """
    model = make_model()
    model.node_types["A"] = "reset"
    model.reset_nodes = {"A": _reset_spec()}

    proxy = model.to_module_proxy()

    assert hasattr(proxy, "RESET_NODES")
    assert proxy.RESET_NODES == {"A": _reset_spec()}
    # Same object the model holds (the validator reads it by reference).
    assert proxy.RESET_NODES is model.reset_nodes


def test_validate_reaches_reset_rules_via_proxy() -> None:
    """model.validate() rejects a bad reset node because the proxy exposes it.

    A reset node typed "reset" with NO RESET_NODES entry is malformed. If the
    proxy did not carry RESET_NODES, _validate_reset_nodes would still catch the
    "reset"-typed node via NODE_TYPES, but a bad SPEC (here, an unknown axis)
    only surfaces when the table itself reaches the validator. This proves the
    proxy plumbs the spec through, not just the node type.
    """
    model = make_model()
    # Use B as the reset node. ``S`` is the (blind) START node, so the
    # reset-typed node B is NOT the start -- the validator rejects a reset-typed
    # START node before any spec check, so keeping the start off B lets the bad
    # SPEC (here, an unknown axis) reach the validator.
    model.node_types["B"] = "reset"
    # A reset node is BLIND-incident only (ADR-048): drop the ``A-B`` LINE segment
    # (which would now be rejected as a LINE edge into a reset node, masking the
    # axis-spec error this test asserts) and reach B over a BLIND edge from the
    # blind spawn instead.
    model.line_segments = []
    model.blind_edges = [("S", "B")]
    model.reset_nodes = {"B": {"axis": "z", "wall_coord": 10.0, "facing": 0.0}}
    with pytest.raises(FieldDataError, match=r"RESET_NODES\['B'\]\.axis"):
        model.validate()


def test_snapshot_restore_preserves_reset_nodes() -> None:
    """snapshot()/restore() deep-copies reset_nodes so undo unwinding preserves it.

    restore() is the path the undo stack uses to unwind to a prior state. A
    shallow copy would alias the spec dicts, so a later live mutation would leak
    into the snapshot (or vice versa). Assert deep independence both ways.
    """
    model = make_model()
    model.node_types["A"] = "reset"
    model.reset_nodes = {"A": _reset_spec()}
    snap = model.snapshot()

    # Mutating the snapshot's nested spec must not touch the live model.
    snap.reset_nodes["A"]["facing"] = 270.0
    assert model.reset_nodes["A"]["facing"] == 90.0

    # Mutate the live model, then restore: the table must come back exactly.
    model.reset_nodes["A"]["wall_coord"] = 999.0
    model.reset_nodes["B"] = {"axis": "x", "wall_coord": 1.0, "facing": 0.0}
    model.restore(snap)
    assert model.reset_nodes == {
        "A": {"axis": "y", "wall_coord": 205.0, "facing": 270.0}
    }
    # The restored table is independent of the snapshot (deep copy on restore).
    model.reset_nodes["A"]["facing"] = 0.0
    assert snap.reset_nodes["A"]["facing"] == 270.0


def test_history_undo_redo_preserves_reset_nodes() -> None:
    """A History do -> undo -> redo cycle leaves reset_nodes intact.

    The editor has no command that touches reset_nodes, so an unrelated edit
    (AddNode) driven through the real History must not disturb the sparse table
    across an undo/redo round-trip. Guards the snapshot/restore deep-copy from a
    regression where a History cycle drops or aliases the preserved-only field.
    """
    model = make_model()
    model.node_types["A"] = "reset"
    model.reset_nodes = {"A": _reset_spec()}
    before = {k: dict(v) for k, v in model.reset_nodes.items()}

    history = History()
    history.do(AddNode(name="Z", xy=(5.0, 5.0), node_type="line"), model)
    assert model.reset_nodes == before

    assert history.undo(model) is True
    assert model.reset_nodes == before
    assert "Z" not in model.nodes

    assert history.redo(model) is True
    assert model.reset_nodes == before
    assert "Z" in model.nodes


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


# ---------------------------------------------------------------------------
# blind-distance decay knobs (T21): the four BLIND_DECAY_* scalars replace the
# old cost_blind_multiplier. They are required dataclass fields, so they must
# survive snapshot/restore (undo/redo) and the JSON round-trip, reach the
# validator via the proxy, and live in their canonical JSON slot.
# ---------------------------------------------------------------------------


def test_decay_knobs_survive_to_from_json_round_trip() -> None:
    """from_json_dict(to_json_dict()) preserves the four decay knobs."""
    model = make_model()
    restored = FieldModel.from_json_dict(model.to_json_dict())

    assert restored.blind_decay_enabled is True
    assert restored.blind_decay_k == 0.05
    assert restored.blind_decay_exponent == 2.0
    assert restored.blind_decay_d_ref == 50.0
    assert restored == model


def test_decay_knobs_emitted_in_json_dict() -> None:
    """to_json_dict() renders the decay knobs as JSON-native scalars in slot."""
    json_dict = make_model().to_json_dict()

    assert json_dict["BLIND_DECAY_ENABLED"] is True
    assert json_dict["BLIND_DECAY_K"] == 0.05
    assert json_dict["BLIND_DECAY_EXPONENT"] == 2.0
    assert json_dict["BLIND_DECAY_D_REF"] == 50.0
    # Canonical slot: the four knobs sit between ROBOT_RADIUS_CM and START_NODE.
    keys = list(json_dict.keys())
    assert (
        keys.index("ROBOT_RADIUS_CM")
        < keys.index("BLIND_DECAY_ENABLED")
        < keys.index("BLIND_DECAY_K")
        < keys.index("BLIND_DECAY_EXPONENT")
        < keys.index("BLIND_DECAY_D_REF")
        < keys.index("START_NODE")
    )


def test_to_module_proxy_exposes_decay_knobs() -> None:
    """to_module_proxy() carries the decay knobs so validate() actually sees them.

    The dual-serialization trap: a field added to to_json_dict() but not to the
    proxy is persisted-but-unvalidated (the validator early-returns / never reads
    the missing attribute). The proxy must expose all four knobs.
    """
    proxy = make_model().to_module_proxy()

    assert proxy.BLIND_DECAY_ENABLED is True
    assert proxy.BLIND_DECAY_K == 0.05
    assert proxy.BLIND_DECAY_EXPONENT == 2.0
    assert proxy.BLIND_DECAY_D_REF == 50.0


def test_validate_reaches_decay_rules_via_proxy() -> None:
    """model.validate() rejects a bad decay knob because the proxy exposes it.

    A negative k is malformed. If the proxy did not carry the decay knobs, the
    validator would never see the value (persisted-but-unvalidated). This proves
    the proxy plumbs the knobs through to _validate_blind_decay.
    """
    model = make_model()
    model.blind_decay_k = -1.0
    with pytest.raises(FieldDataError, match=r"BLIND_DECAY_K"):
        model.validate()


def test_snapshot_restore_preserves_decay_knobs() -> None:
    """snapshot()/restore() round-trips the decay knobs so undo unwinding keeps them."""
    model = make_model()
    snap = model.snapshot()

    # Mutate the live model, then restore: the knobs must come back exactly.
    model.blind_decay_enabled = False
    model.blind_decay_k = 99.0
    model.blind_decay_exponent = 5.0
    model.blind_decay_d_ref = 1.0
    model.restore(snap)

    assert model.blind_decay_enabled is True
    assert model.blind_decay_k == 0.05
    assert model.blind_decay_exponent == 2.0
    assert model.blind_decay_d_ref == 50.0


def test_history_undo_redo_preserves_decay_knobs() -> None:
    """A History do -> undo -> redo cycle leaves the decay knobs intact.

    The editor has no command that touches the decay knobs, so an unrelated edit
    (AddNode) driven through the real History must not disturb them across an
    undo/redo round-trip. Guards the snapshot/restore path from a regression
    where a History cycle drops or corrupts a required scalar field.
    """
    model = make_model()
    before = (
        model.blind_decay_enabled,
        model.blind_decay_k,
        model.blind_decay_exponent,
        model.blind_decay_d_ref,
    )

    def _current() -> tuple:
        return (
            model.blind_decay_enabled,
            model.blind_decay_k,
            model.blind_decay_exponent,
            model.blind_decay_d_ref,
        )

    history = History()
    history.do(AddNode(name="Z", xy=(5.0, 5.0), node_type="line"), model)
    assert _current() == before

    assert history.undo(model) is True
    assert _current() == before
    assert "Z" not in model.nodes

    assert history.redo(model) is True
    assert _current() == before
    assert "Z" in model.nodes


# ---------------------------------------------------------------------------
# blind_projection_auto_seek (field-wide BLIND->PROJECTION auto-seek flag)
# ---------------------------------------------------------------------------


def test_blind_projection_auto_seek_defaults_true() -> None:
    """A model built without the flag defaults it to True (the loader default)."""
    model = make_model()
    assert model.blind_projection_auto_seek is True


def test_blind_projection_auto_seek_omitted_when_default_true() -> None:
    """to_json_dict omits the key when it equals the default True (absent stays
    absent), so a field that never set it produces no spurious diff."""
    model = make_model()
    json_dict = model.to_json_dict()
    assert "blind_projection_auto_seek" not in json_dict
    assert tuple(json_dict.keys())[-1] == "OBSTACLES"


def test_blind_projection_auto_seek_emitted_and_round_trips_when_false() -> None:
    """Setting the flag False emits it, and from_json_dict reads it back.

    This is the dual-serialization fix: before the field was tracked, saving
    from the editor dropped the key and field_data.py reverted it to True.
    """
    model = make_model()
    model.blind_projection_auto_seek = False

    json_dict = model.to_json_dict()
    assert json_dict["blind_projection_auto_seek"] is False
    # Canonical slot: between RESET_NODES (absent here) and OBSTACLES.
    keys = tuple(json_dict.keys())
    assert keys.index("blind_projection_auto_seek") == keys.index("OBSTACLES") - 1

    restored = FieldModel.from_json_dict(json_dict)
    assert restored.blind_projection_auto_seek is False
    assert restored == model


def test_from_json_dict_tolerates_absent_blind_projection_auto_seek() -> None:
    """A JSON dict predating the flag loads with the True default, not KeyError."""
    model = make_model()
    data = model.to_json_dict()
    assert "blind_projection_auto_seek" not in data  # absent (default True)
    restored = FieldModel.from_json_dict(data)
    assert restored.blind_projection_auto_seek is True


def test_to_module_proxy_exposes_blind_projection_auto_seek() -> None:
    """The validator proxy carries BLIND_PROJECTION_AUTO_SEEK so validate() can
    type-check the edited value (otherwise it is persisted-but-unvalidated)."""
    model = make_model()
    model.blind_projection_auto_seek = False
    proxy = model.to_module_proxy()
    assert proxy.BLIND_PROJECTION_AUTO_SEEK is False


def test_snapshot_restore_preserves_blind_projection_auto_seek() -> None:
    """snapshot()/restore() round-trips the flag so undo unwinding keeps it."""
    model = make_model()
    model.blind_projection_auto_seek = False
    snap = model.snapshot()

    model.blind_projection_auto_seek = True
    model.restore(snap)

    assert model.blind_projection_auto_seek is False
