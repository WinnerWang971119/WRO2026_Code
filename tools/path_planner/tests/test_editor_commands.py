"""Tests for the Command pattern and History undo/redo stack.

Covers TC4 (snapshot/restore on AddNode), TC5 (RenameNode cascade),
TC6 (DeleteNode refusal), TC13 (10-mutation undo round-trip),
TC14 (undo + redo state equality).

No GUI, no file I/O -- everything is built in memory.
"""

from __future__ import annotations

import random

import pytest

from tools.path_planner.editor.commands import (
    AddBlindEdge,
    AddEdgesBatch,
    AddLineEdge,
    AddNode,
    AddObstacle,
    AddPivotTurn,
    ClearEdgeOverride,
    DeleteEdge,
    DeleteNode,
    DeleteObstacle,
    MacroCommand,
    MoveNode,
    RenameNode,
    SetEdgeOverride,
    SetFieldDimensions,
    SetFieldWideFlag,
    SetPipelineMode,
    SetStart,
    SetStartHeading,
)
from tools.path_planner.editor.history import History
from tools.path_planner.editor.model import FieldModel


# ---------------------------------------------------------------------------
# Fixture factory (duplicated from test_editor_model so each test file is
# self-contained; the plan allows this rather than mandating a conftest)
# ---------------------------------------------------------------------------


def make_model() -> FieldModel:
    """Return a small but fully-valid FieldModel built in memory."""
    return FieldModel(
        field_mm=(3000, 2000),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="A",
        start_heading_deg=0.0,
        nodes={"A": (10.0, 10.0), "B": (200.0, 10.0)},
        node_types={"A": "line", "B": "line"},
        line_segments=[("A", "B")],
        blind_edges=[],
        obstacles=[(50.0, 50.0, 80.0, 80.0, "pillar")],
    )


# ---------------------------------------------------------------------------
# AddNode (TC4)
# ---------------------------------------------------------------------------


def test_add_node_apply_then_revert_restores_exactly() -> None:
    """TC4: applying AddNode and then reverting must return the model to the
    exact pre-apply state (compared via to_json_dict repr)."""
    model = make_model()
    pre_apply_repr = repr(model.to_json_dict())

    cmd = AddNode("C", (300.0, 10.0), "blind")
    cmd.apply(model)
    assert "C" in model.nodes
    assert "C" in model.node_types

    cmd.revert(model)
    assert repr(model.to_json_dict()) == pre_apply_repr


# ---------------------------------------------------------------------------
# MoveNode
# ---------------------------------------------------------------------------


def test_move_node_revert_uses_captured_xy() -> None:
    """MoveNode.revert must restore the node to its original coordinates."""
    model = make_model()
    cmd = MoveNode("A", (99.0, 99.0))
    cmd.apply(model)
    assert model.nodes["A"] == (99.0, 99.0)

    cmd.revert(model)
    assert model.nodes["A"] == (10.0, 10.0)


# ---------------------------------------------------------------------------
# RenameNode (TC5)
# ---------------------------------------------------------------------------


def test_rename_node_cascades_to_all_containers() -> None:
    """TC5: RenameNode must cascade through nodes, node_types, line_segments,
    and start_node; revert must restore all of them."""
    model = make_model()
    cmd = RenameNode("A", "AA")
    cmd.apply(model)

    # Forward: every container updated.
    assert "A" not in model.nodes
    assert "AA" in model.nodes
    assert "AA" in model.node_types
    assert "A" not in model.node_types
    assert model.line_segments == [("AA", "B")]
    assert model.start_node == "AA"

    # Revert: everything back to original.
    cmd.revert(model)
    assert "A" in model.nodes
    assert "AA" not in model.nodes
    assert "A" in model.node_types
    assert "AA" not in model.node_types
    assert model.line_segments == [("A", "B")]
    assert model.start_node == "A"


# ---------------------------------------------------------------------------
# DeleteNode (TC6 scope -- simple edges only)
# ---------------------------------------------------------------------------


def test_delete_node_refuses_when_line_segment_references_it() -> None:
    """TC6: DeleteNode.apply must raise ValueError when the node is still
    referenced by a line_segments entry; the message must name 'line_segments'
    and the entry index (0)."""
    model = make_model()
    # "A" is endpoint of line_segments[0] = ("A", "B").
    with pytest.raises(ValueError) as exc_info:
        DeleteNode("A").apply(model)

    msg = str(exc_info.value)
    assert "line_segments" in msg
    assert "0" in msg
    # Model must be untouched.
    assert "A" in model.nodes


def test_delete_node_succeeds_when_no_edges_reference_it() -> None:
    """Deleting a node with no edge references succeeds; revert restores it."""
    model = make_model()
    # Add an isolated node so we have something to delete cleanly.
    model.nodes["X"] = (999.0, 999.0)
    model.node_types["X"] = "blind"

    cmd = DeleteNode("X")
    cmd.apply(model)
    assert "X" not in model.nodes
    assert "X" not in model.node_types

    cmd.revert(model)
    assert "X" in model.nodes
    assert "X" in model.node_types


def test_rename_node_cascades_reset_spec_and_pivot_turns() -> None:
    """RenameNode must re-key a reset spec and rename pivot-turn legs; revert
    must restore both."""
    model = make_model()
    model.nodes["C"] = (300.0, 10.0)
    model.node_types["A"] = "reset"
    model.node_types["C"] = "line"
    model.reset_nodes = {"A": {"axis": "x", "wall_coord": 0.0, "facing": 0.0}}
    model.pivot_turns = [("B", "A", "C")]

    cmd = RenameNode("A", "AA")
    cmd.apply(model)

    # Reset spec re-keyed onto the new name (same spec body); pivot leg renamed.
    assert "A" not in model.reset_nodes
    assert model.reset_nodes["AA"] == {
        "axis": "x",
        "wall_coord": 0.0,
        "facing": 0.0,
    }
    assert model.pivot_turns == [("B", "AA", "C")]

    cmd.revert(model)
    assert "AA" not in model.reset_nodes
    assert model.reset_nodes["A"] == {
        "axis": "x",
        "wall_coord": 0.0,
        "facing": 0.0,
    }
    assert model.pivot_turns == [("B", "A", "C")]


def test_delete_node_cascades_reset_spec_and_revert_restores() -> None:
    """Deleting a reset node removes its own reset spec; revert restores it."""
    model = make_model()
    model.nodes["R"] = (400.0, 400.0)
    model.node_types["R"] = "reset"
    spec = {"axis": "y", "wall_coord": 0.0, "facing": 90.0}
    model.reset_nodes = {"R": dict(spec)}

    cmd = DeleteNode("R")
    cmd.apply(model)
    assert "R" not in model.nodes
    assert "R" not in model.reset_nodes

    cmd.revert(model)
    assert "R" in model.nodes
    assert model.reset_nodes["R"] == spec


def test_delete_node_refuses_when_pivot_references_it() -> None:
    """DeleteNode.apply must raise (model untouched) when a pivot turn names the
    node, and the message must name 'pivot_turns'."""
    model = make_model()
    model.nodes["C"] = (300.0, 10.0)
    model.node_types["C"] = "line"
    model.pivot_turns = [("A", "B", "C")]

    with pytest.raises(ValueError) as exc_info:
        DeleteNode("C").apply(model)

    assert "pivot_turns" in str(exc_info.value)
    assert "C" in model.nodes  # untouched


def test_add_pivot_turn_rejects_degenerate_triple() -> None:
    """AddPivotTurn must reject u==v or v==w (mirrors the validator)."""
    model = make_model()
    model.nodes["C"] = (300.0, 10.0)
    model.node_types["C"] = "line"

    with pytest.raises(ValueError, match="degenerate"):
        AddPivotTurn("A", "A", "B").apply(model)  # u == v
    with pytest.raises(ValueError, match="degenerate"):
        AddPivotTurn("A", "B", "B").apply(model)  # v == w
    assert model.pivot_turns == []  # nothing committed


# ---------------------------------------------------------------------------
# MacroCommand (atomic override)
# ---------------------------------------------------------------------------


def test_macro_command_applies_and_reverts_as_one_unit() -> None:
    """A MacroCommand bundling delete-then-add applies both, and a single revert
    restores both (one undo for the whole override)."""
    model = make_model()  # line_segments == [("A", "B")]
    macro = MacroCommand([DeleteEdge("A", "B"), AddBlindEdge("A", "B")])

    macro.apply(model)
    assert model.line_segments == []
    assert model.blind_edges == [("A", "B")]

    macro.revert(model)
    assert model.line_segments == [("A", "B")]
    assert model.blind_edges == []


def test_macro_command_rolls_back_prefix_on_subcommand_failure() -> None:
    """If a later sub-command raises, the already-applied prefix is reverted so
    the model is left untouched and the exception propagates."""
    model = make_model()
    pre = repr(model.to_json_dict())
    # First delete succeeds; the second delete of the now-gone pair raises.
    macro = MacroCommand([DeleteEdge("A", "B"), DeleteEdge("A", "B")])

    with pytest.raises(ValueError):
        macro.apply(model)

    # The first delete was rolled back -> model byte-identical to pre-apply.
    assert repr(model.to_json_dict()) == pre


# ---------------------------------------------------------------------------
# AddLineEdge duplicate / collision tests
# ---------------------------------------------------------------------------


def test_add_line_edge_duplicate_raises() -> None:
    """AddLineEdge for a pair already in line_segments must raise ValueError."""
    model = make_model()
    with pytest.raises(ValueError):
        AddLineEdge("A", "B").apply(model)


def test_add_line_edge_reverse_direction_is_duplicate() -> None:
    """Edges are undirected: AddLineEdge('B', 'A') when ('A','B') exists raises."""
    model = make_model()
    with pytest.raises(ValueError):
        AddLineEdge("B", "A").apply(model)


def test_add_line_edge_blind_collision_raises() -> None:
    """AddBlindEdge for a pair already in line_segments must raise ValueError."""
    model = make_model()
    # ("A","B") is in line_segments; adding it as a blind edge is forbidden.
    with pytest.raises(ValueError):
        AddBlindEdge("A", "B").apply(model)


# ---------------------------------------------------------------------------
# AddObstacle revert
# ---------------------------------------------------------------------------


def test_add_obstacle_revert_pops_specific_entry() -> None:
    """Reverting an AddObstacle must remove only the newly added obstacle,
    not the one that already existed before apply."""
    model = make_model()
    # model already has one obstacle: (50.0, 50.0, 80.0, 80.0, "pillar").
    original_obstacles = list(model.obstacles)

    cmd = AddObstacle(100.0, 100.0, 150.0, 150.0, "wall")
    cmd.apply(model)
    assert len(model.obstacles) == 2

    cmd.revert(model)
    assert model.obstacles == original_obstacles


# ---------------------------------------------------------------------------
# DeleteObstacle apply / revert / out-of-range guard
# ---------------------------------------------------------------------------


def test_delete_obstacle_revert_reinserts_at_original_index() -> None:
    """DeleteObstacle.revert must put the obstacle back where it was."""
    model = make_model()
    # Two obstacles so the index actually matters for revert.
    model.obstacles.append((100.0, 100.0, 150.0, 150.0, "wall"))
    original_obstacles = list(model.obstacles)

    cmd = DeleteObstacle(0)
    cmd.apply(model)
    assert model.obstacles == [(100.0, 100.0, 150.0, 150.0, "wall")]

    cmd.revert(model)
    assert model.obstacles == original_obstacles


def test_delete_obstacle_raises_on_out_of_range_index() -> None:
    """DeleteObstacle must raise IndexError -- not silently no-op -- on a
    stale index. The editor's context menu relies on this so a confirm
    dialog opened against an already-removed obstacle surfaces cleanly."""
    model = make_model()
    with pytest.raises(IndexError):
        DeleteObstacle(99).apply(model)
    with pytest.raises(IndexError):
        DeleteObstacle(-1).apply(model)


# ---------------------------------------------------------------------------
# DeleteEdge revert (index restoration)
# ---------------------------------------------------------------------------


def test_delete_edge_revert_reinserts_at_original_index() -> None:
    """Reverting a DeleteEdge must put the edge back at its original index."""
    model = make_model()
    # Add a second line edge so the list has two entries.
    model.nodes["C"] = (300.0, 10.0)
    model.node_types["C"] = "line"
    model.line_segments.append(("B", "C"))
    # line_segments is now [("A","B"), ("B","C")]; delete index 0.
    original_segments = list(model.line_segments)

    cmd = DeleteEdge("A", "B")
    cmd.apply(model)
    assert model.line_segments == [("B", "C")]

    cmd.revert(model)
    assert model.line_segments == original_segments


# ---------------------------------------------------------------------------
# SetStart / SetStartHeading validation
# ---------------------------------------------------------------------------


def test_set_start_raises_on_unknown_node() -> None:
    """SetStart must raise ValueError when the target node is not in model.nodes."""
    model = make_model()
    with pytest.raises(ValueError):
        SetStart("ZZZ").apply(model)


def test_set_start_heading_raises_out_of_range() -> None:
    """SetStartHeading must reject 360.0 and -1.0; accept 0.0 and 359.9."""
    model = make_model()

    with pytest.raises(ValueError):
        SetStartHeading(360.0).apply(model)

    with pytest.raises(ValueError):
        SetStartHeading(-1.0).apply(model)

    # Valid boundary values must not raise.
    cmd_zero = SetStartHeading(0.0)
    cmd_zero.apply(model)
    cmd_zero.revert(model)

    cmd_max = SetStartHeading(359.9)
    cmd_max.apply(model)
    cmd_max.revert(model)


# ---------------------------------------------------------------------------
# History: failure / redo-clearing tests
# ---------------------------------------------------------------------------


def test_history_do_failure_does_not_push() -> None:
    """A failed do() (AddLineEdge duplicate) must not push onto the undo stack."""
    model = make_model()
    history = History()

    with pytest.raises(ValueError):
        history.do(AddLineEdge("A", "B"), model)

    assert history.can_undo is False


def test_history_do_success_clears_redo() -> None:
    """After undo + a new successful do, can_redo must be False."""
    model = make_model()
    history = History()

    # First successful do.
    model.nodes["C"] = (300.0, 10.0)
    model.node_types["C"] = "line"
    history.do(AddLineEdge("B", "C"), model)
    history.undo(model)
    assert history.can_redo is True

    # A fresh command on a different pair should clear the redo stack.
    model.nodes["D"] = (400.0, 10.0)
    model.node_types["D"] = "line"
    history.do(AddNode("E", (500.0, 10.0), "line"), model)
    assert history.can_redo is False


# ---------------------------------------------------------------------------
# History: 10-mutation undo/redo round-trip (TC13 + TC14)
# ---------------------------------------------------------------------------


def test_history_undo_redo_round_trip_10_mutations() -> None:
    """TC13 + TC14: apply 10 mutations, undo all 10, then redo all 10.

    After 10 undos the model must equal the original make_model() state.
    After each redo step, the model must equal the captured intermediate
    state from the forward pass.

    Bonus assertion: undo 5, redo 5 -- model equals the state after
    the first 5 mutations.
    """
    model = make_model()
    history = History()

    # Snapshot the original state before any mutations.
    original_json = model.to_json_dict()

    # Build 10 distinct mutations and capture the model state after each.
    # We need extra nodes in the model for some commands to work; those setup
    # mutations go through history so they are counted.
    states_after: list[dict] = []

    # Mutation 1: AddNode "C"
    history.do(AddNode("C", (300.0, 10.0), "line"), model)
    states_after.append(model.to_json_dict())

    # Mutation 2: AddNode "D"
    history.do(AddNode("D", (400.0, 10.0), "blind"), model)
    states_after.append(model.to_json_dict())

    # Mutation 3: AddNode "E"
    history.do(AddNode("E", (500.0, 10.0), "line"), model)
    states_after.append(model.to_json_dict())

    # Mutation 4: MoveNode "C"
    history.do(MoveNode("C", (305.0, 15.0)), model)
    states_after.append(model.to_json_dict())

    # Mutation 5: MoveNode "D"
    history.do(MoveNode("D", (405.0, 15.0)), model)
    states_after.append(model.to_json_dict())

    # Mutation 6: AddLineEdge "B" -- "C"
    history.do(AddLineEdge("B", "C"), model)
    states_after.append(model.to_json_dict())

    # Mutation 7: AddLineEdge "C" -- "D"
    history.do(AddLineEdge("C", "D"), model)
    states_after.append(model.to_json_dict())

    # Mutation 8: AddObstacle
    history.do(AddObstacle(200.0, 200.0, 250.0, 250.0, "box"), model)
    states_after.append(model.to_json_dict())

    # Mutation 9: AddObstacle (different coords)
    history.do(AddObstacle(300.0, 300.0, 350.0, 350.0, "cone"), model)
    states_after.append(model.to_json_dict())

    # Mutation 10: SetStartHeading 45.0
    history.do(SetStartHeading(45.0), model)
    states_after.append(model.to_json_dict())

    assert len(states_after) == 10

    # TC13: undo all 10 -- must return to the original state.
    for _ in range(10):
        result = history.undo(model)
        assert result is True, "undo() returned False before the stack was empty"

    assert model.to_json_dict() == original_json
    assert history.can_undo is False

    # TC14: redo all 10 -- after each redo, model must equal the captured state.
    for step in range(10):
        result = history.redo(model)
        assert result is True, f"redo() returned False at step {step}"
        assert model.to_json_dict() == states_after[step], (
            f"model state after redo step {step} does not match captured state"
        )

    assert history.can_redo is False

    # Bonus: undo 5, redo 5 -- model must equal the state after the first 5 mutations.
    for _ in range(5):
        history.undo(model)

    assert model.to_json_dict() == states_after[4]

    for _ in range(5):
        history.redo(model)

    assert model.to_json_dict() == states_after[9]


# ---------------------------------------------------------------------------
# AddEdgesBatch (TC1, TC2, TC3, TC4, TC14)
# ---------------------------------------------------------------------------


def make_four_node_model() -> FieldModel:
    """Return a model with four nodes (a, b, c, d) and no edges."""
    return FieldModel(
        field_mm=(3000, 2000),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="a",
        start_heading_deg=0.0,
        nodes={
            "a": (10.0, 10.0),
            "b": (100.0, 10.0),
            "c": (200.0, 10.0),
            "d": (300.0, 10.0),
        },
        node_types={"a": "line", "b": "line", "c": "line", "d": "line"},
        line_segments=[],
        blind_edges=[],
        obstacles=[],
    )


def test_add_edges_batch_empty_is_noop() -> None:
    """TC1: empty batch — apply and revert must leave the model untouched."""
    model = make_four_node_model()
    snap_before = model.snapshot()

    cmd = AddEdgesBatch(edges=[])
    cmd.apply(model)

    # Model is unchanged after apply.
    assert model.to_json_dict() == snap_before.to_json_dict()

    cmd.revert(model)

    # Model is still unchanged after revert.
    assert model.to_json_dict() == snap_before.to_json_dict()


def test_add_edges_batch_three_mixed_edges_apply_and_revert() -> None:
    """TC2: 3 mixed edges apply to the correct lists; revert restores the model."""
    model = make_four_node_model()
    snap_before = model.snapshot()

    cmd = AddEdgesBatch(edges=[
        ("a", "b", "line"),
        ("b", "c", "blind"),
        ("c", "d", "line"),
    ])
    cmd.apply(model)

    # line_segments must contain the two line edges.
    assert ("a", "b") in model.line_segments
    assert ("c", "d") in model.line_segments
    # blind_edges must contain the one blind edge.
    assert ("b", "c") in model.blind_edges
    # Counts are exact.
    assert len(model.line_segments) == 2
    assert len(model.blind_edges) == 1

    cmd.revert(model)

    # After revert, model must be byte-identical to the pre-apply snapshot.
    assert model.to_json_dict() == snap_before.to_json_dict()
    assert model.line_segments == []
    assert model.blind_edges == []


def test_add_edges_batch_duplicate_existing_edge_raises_and_model_untouched() -> None:
    """TC3: batch containing an edge that already exists raises ValueError;
    model must be untouched."""
    model = make_four_node_model()
    # Seed an existing line edge.
    model.line_segments.append(("a", "b"))
    snap_before = model.snapshot()

    cmd = AddEdgesBatch(edges=[("a", "b", "line")])

    with pytest.raises(ValueError) as exc_info:
        cmd.apply(model)

    # Error message must name the list and index.
    msg = str(exc_info.value)
    assert "line_segments" in msg
    assert "0" in msg

    # Model must be untouched.
    assert model.to_json_dict() == snap_before.to_json_dict()


def test_add_edges_batch_intra_batch_duplicate_raises_and_model_untouched() -> None:
    """TC4: two entries with the same undirected pair within the batch raise
    ValueError; model must be untouched."""
    model = make_four_node_model()
    snap_before = model.snapshot()

    # ("a","b","line") and ("b","a","blind") form the same undirected pair.
    cmd = AddEdgesBatch(edges=[
        ("a", "b", "line"),
        ("b", "a", "blind"),
    ])

    with pytest.raises(ValueError) as exc_info:
        cmd.apply(model)

    assert "duplicate" in str(exc_info.value).lower()

    # Model must be untouched — no partial writes.
    assert model.to_json_dict() == snap_before.to_json_dict()


def test_add_edges_batch_round_trip_apply_revert_apply() -> None:
    """TC14 (AddEdgesBatch): apply → revert → apply; final state matches
    the state captured after the FIRST apply.  This verifies that
    _applied_indices is rebuilt correctly on the re-apply."""
    model = make_four_node_model()

    cmd = AddEdgesBatch(edges=[
        ("a", "b", "line"),
        ("b", "c", "blind"),
        ("c", "d", "line"),
    ])

    # First apply.
    cmd.apply(model)
    state_after_first_apply = model.to_json_dict()

    # Revert.
    cmd.revert(model)
    assert model.line_segments == []
    assert model.blind_edges == []

    # Re-apply — _applied_indices must be rebuilt from scratch.
    cmd.apply(model)
    assert model.to_json_dict() == state_after_first_apply


# ---------------------------------------------------------------------------
# Issue #79 — per-edge override + field-tunable commands
#
# Fixture factory carrying both a LINE and a BLIND segment so an override can
# attach to each, and the (type, u, v) identity guard (TC14) can prove a LINE
# and a BLIND override on the SAME directed pair are independent entries.
# ---------------------------------------------------------------------------


def make_override_model() -> FieldModel:
    """Return a model with one LINE and one BLIND segment, no overrides yet.

    ``A``--``B`` is a line segment and ``B``--``G`` a blind edge.  The two
    share no direction, but the identity-guard test deliberately authors a
    LINE and a BLIND override on the SAME ``A -> B`` directed pair to prove the
    ``(type, u, v)`` key keeps them distinct.
    """
    return FieldModel(
        field_mm=(3000, 2000),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="A",
        start_heading_deg=0.0,
        nodes={
            "A": (10.0, 10.0),
            "B": (200.0, 10.0),
            "G": (250.0, 80.0),
        },
        node_types={"A": "line", "B": "line", "G": "blind"},
        line_segments=[("A", "B")],
        blind_edges=[("B", "G")],
        obstacles=[],
    )


# ---------------------------------------------------------------------------
# TC13 — SetEdgeOverride apply / revert
# ---------------------------------------------------------------------------


def test_set_edge_override_apply_adds_entry_and_revert_restores_exactly() -> None:
    """TC13: applying SetEdgeOverride adds the override-only entry; reverting
    restores the model byte-identically to its pre-apply state."""
    model = make_override_model()
    pre_apply_repr = repr(model.to_json_dict())
    assert model.edges == []

    cmd = SetEdgeOverride("LINE", "A", "B", primitive_mode="FAST")
    cmd.apply(model)

    # The override entry is present and override-only (identity triple + the
    # one non-None override key; no geometry keys are ever written).
    assert model.edges == [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"}
    ]

    cmd.revert(model)
    assert model.edges == []
    assert repr(model.to_json_dict()) == pre_apply_repr


def test_set_edge_override_omits_inherit_keys() -> None:
    """TC13: a None override field is OMITTED from the entry (absence == inherit),
    so only the explicitly-set keys appear alongside the identity triple."""
    model = make_override_model()

    # Only stop_at_line is set; primitive_mode + direction stay None (omitted).
    SetEdgeOverride("BLIND", "B", "G", stop_at_line="on").apply(model)

    assert model.edges == [
        {"type": "BLIND", "u": "B", "v": "G", "stop_at_line": "on"}
    ]


def test_set_edge_override_replace_in_place_preserves_position_and_reverts() -> None:
    """TC13: re-setting an existing (type, u, v) REPLACES the entry in place
    (same list position); revert restores the exact prior entry, not a removal."""
    model = make_override_model()

    # Seed two overrides so the replaced one is NOT at the tail (position matters).
    SetEdgeOverride("LINE", "A", "B", primitive_mode="FAST").apply(model)
    SetEdgeOverride("BLIND", "B", "G", direction="front").apply(model)
    pre_replace_repr = repr(model.to_json_dict())

    replace = SetEdgeOverride("LINE", "A", "B", primitive_mode="SLOW")
    replace.apply(model)

    # The LINE entry stays at index 0 (replaced in place), now carrying SLOW.
    assert model.edges[0] == {
        "type": "LINE",
        "u": "A",
        "v": "B",
        "primitive_mode": "SLOW",
    }
    assert model.edges[1] == {
        "type": "BLIND",
        "u": "B",
        "v": "G",
        "direction": "front",
    }

    # Revert restores the displaced (FAST) entry at its original index.
    replace.revert(model)
    assert repr(model.to_json_dict()) == pre_replace_repr


def test_set_edge_override_undo_redo_undo_is_stable() -> None:
    """TC13: SetEdgeOverride survives an undo -> redo -> undo cycle through the
    real History stack and lands back at the pre-apply state each time."""
    model = make_override_model()
    history = History()
    original_json = model.to_json_dict()

    history.do(SetEdgeOverride("LINE", "A", "B", primitive_mode="FAST"), model)
    state_after_apply = model.to_json_dict()
    assert state_after_apply != original_json

    # undo -> back to empty.
    assert history.undo(model) is True
    assert model.to_json_dict() == original_json

    # redo -> back to the override.
    assert history.redo(model) is True
    assert model.to_json_dict() == state_after_apply

    # undo again -> stable, identical to the first undo.
    assert history.undo(model) is True
    assert model.to_json_dict() == original_json


# ---------------------------------------------------------------------------
# TC14 — ClearEdgeOverride / SetPipelineMode / SetFieldWideFlag /
#        SetFieldDimensions apply / revert round-trip invariant.
# ---------------------------------------------------------------------------


def test_clear_edge_override_apply_revert_round_trip() -> None:
    """TC14: ClearEdgeOverride removes the entry on apply and re-inserts it at
    its original index on revert (round-trip invariant)."""
    model = make_override_model()
    SetEdgeOverride("LINE", "A", "B", primitive_mode="FAST").apply(model)
    SetEdgeOverride("BLIND", "B", "G", stop_at_line="on").apply(model)
    pre_clear_repr = repr(model.to_json_dict())

    # Clear the FIRST entry so revert's index restoration is meaningful.
    cmd = ClearEdgeOverride("LINE", "A", "B")
    cmd.apply(model)

    assert model.edges == [
        {"type": "BLIND", "u": "B", "v": "G", "stop_at_line": "on"}
    ]

    cmd.revert(model)
    assert repr(model.to_json_dict()) == pre_clear_repr
    # The re-inserted entry is back at index 0 (its original position).
    assert model.edges[0] == {
        "type": "LINE",
        "u": "A",
        "v": "B",
        "primitive_mode": "FAST",
    }


def test_clear_edge_override_missing_identity_raises() -> None:
    """TC14: clearing a non-existent (type, u, v) raises ValueError and leaves
    the model untouched (History treats it as a no-op)."""
    model = make_override_model()
    pre = repr(model.to_json_dict())

    with pytest.raises(ValueError):
        ClearEdgeOverride("LINE", "A", "B").apply(model)

    assert repr(model.to_json_dict()) == pre


def test_edge_override_type_u_v_identity_is_independent() -> None:
    """TC14 (identity guard): a LINE and a BLIND override sharing the SAME
    ``u -> v`` directed pair are independent entries.

    Setting both must produce two distinct entries (keyed on the full
    ``(type, u, v)`` triple), and clearing ONE must leave the other untouched --
    a clear keyed on ``(u, v)`` alone would clobber the wrong-typed override.
    """
    model = make_override_model()

    # Both overrides target the SAME directed pair A -> B but different types.
    SetEdgeOverride("LINE", "A", "B", primitive_mode="FAST").apply(model)
    SetEdgeOverride("BLIND", "A", "B", stop_at_line="on").apply(model)

    # Two distinct entries — the directed pair is NOT unique across types.
    assert len(model.edges) == 2
    line_entry = {
        "type": "LINE",
        "u": "A",
        "v": "B",
        "primitive_mode": "FAST",
    }
    blind_entry = {"type": "BLIND", "u": "A", "v": "B", "stop_at_line": "on"}
    assert line_entry in model.edges
    assert blind_entry in model.edges

    # Clear ONLY the LINE override; the BLIND one on the same pair survives.
    ClearEdgeOverride("LINE", "A", "B").apply(model)
    assert model.edges == [blind_entry]

    # Symmetrically, clearing the BLIND one now empties the overlay.
    ClearEdgeOverride("BLIND", "A", "B").apply(model)
    assert model.edges == []


@pytest.mark.parametrize(
    ("which", "attr", "value"),
    [
        ("line", "pipeline_line_mode", "FAST"),
        ("blind", "pipeline_blind_mode", "GYRO_TSHAPE"),
        ("turn", "pipeline_turn_mode", "COUNTER_ROT"),
        ("direction", "pipeline_direction_mode", "DEFAULT"),
    ],
)
def test_set_pipeline_mode_apply_revert_round_trip(
    which: str, attr: str, value: str
) -> None:
    """TC14: SetPipelineMode for each of the four categories sets the matching
    ``pipeline_<which>_mode`` attribute on apply and restores the prior value
    (None by default) on revert."""
    model = make_override_model()
    assert getattr(model, attr) is None  # absent-by-default

    cmd = SetPipelineMode(which, value)
    cmd.apply(model)
    assert getattr(model, attr) == value

    cmd.revert(model)
    assert getattr(model, attr) is None


def test_set_pipeline_mode_clear_to_none_round_trip() -> None:
    """TC14: SetPipelineMode with value=None clears a previously-set mode back to
    inherit; revert restores the prior member name exactly."""
    model = make_override_model()
    SetPipelineMode("line", "FAST").apply(model)
    assert model.pipeline_line_mode == "FAST"

    clear = SetPipelineMode("line", None)
    clear.apply(model)
    assert model.pipeline_line_mode is None

    clear.revert(model)
    assert model.pipeline_line_mode == "FAST"


def test_set_pipeline_mode_unknown_which_raises() -> None:
    """TC14: an unrecognized ``which`` selector raises ValueError and leaves the
    model untouched."""
    model = make_override_model()
    pre = repr(model.to_json_dict())

    with pytest.raises(ValueError):
        SetPipelineMode("bogus", "FAST").apply(model)

    assert repr(model.to_json_dict()) == pre


def test_set_field_wide_flag_apply_revert_round_trip() -> None:
    """TC14: SetFieldWideFlag toggles ``blind_projection_auto_seek`` on apply and
    restores the prior boolean on revert."""
    model = make_override_model()
    original = model.blind_projection_auto_seek  # defaults True

    cmd = SetFieldWideFlag("blind_projection_auto_seek", not original)
    cmd.apply(model)
    assert model.blind_projection_auto_seek == (not original)

    cmd.revert(model)
    assert model.blind_projection_auto_seek == original


def test_set_field_wide_flag_unknown_flag_raises() -> None:
    """TC14: an unrecognized field-wide flag raises ValueError and leaves the
    model untouched."""
    model = make_override_model()
    pre = repr(model.to_json_dict())

    with pytest.raises(ValueError):
        SetFieldWideFlag("not_a_flag", True).apply(model)

    assert repr(model.to_json_dict()) == pre


def test_set_field_dimensions_apply_revert_round_trip() -> None:
    """TC14: SetFieldDimensions writes the new (width, height) pair on apply and
    restores the captured prior pair on revert."""
    model = make_override_model()
    original = model.field_mm  # (3000, 2000)

    cmd = SetFieldDimensions(2362.0, 1143.0)
    cmd.apply(model)
    assert model.field_mm == (2362.0, 1143.0)

    cmd.revert(model)
    assert model.field_mm == original


def test_set_field_dimensions_non_positive_raises() -> None:
    """TC14: a non-positive dimension raises ValueError and leaves the model
    untouched (History treats it as a no-op)."""
    model = make_override_model()
    pre = repr(model.to_json_dict())

    with pytest.raises(ValueError):
        SetFieldDimensions(0.0, 1143.0).apply(model)
    with pytest.raises(ValueError):
        SetFieldDimensions(2362.0, -1.0).apply(model)

    assert repr(model.to_json_dict()) == pre


def test_issue79_commands_undo_redo_round_trip_through_history() -> None:
    """TC14 (cont.): all five new commands round-trip cleanly through the real
    History stack — apply each via do(), undo all, redo all, and the model
    returns to (and re-reaches) the captured states at each step."""
    model = make_override_model()
    history = History()
    original_json = model.to_json_dict()

    states_after: list[dict] = []

    history.do(SetEdgeOverride("LINE", "A", "B", primitive_mode="FAST"), model)
    states_after.append(model.to_json_dict())

    history.do(SetEdgeOverride("BLIND", "B", "G", stop_at_line="on"), model)
    states_after.append(model.to_json_dict())

    history.do(ClearEdgeOverride("LINE", "A", "B"), model)
    states_after.append(model.to_json_dict())

    history.do(SetPipelineMode("blind", "GYRO_TSHAPE"), model)
    states_after.append(model.to_json_dict())

    history.do(SetFieldWideFlag("blind_projection_auto_seek", False), model)
    states_after.append(model.to_json_dict())

    history.do(SetFieldDimensions(2362.0, 1143.0), model)
    states_after.append(model.to_json_dict())

    # Undo all six — back to the original state.
    for _ in range(len(states_after)):
        assert history.undo(model) is True
    assert model.to_json_dict() == original_json
    assert history.can_undo is False

    # Redo all six — each step re-reaches the captured intermediate state.
    for step in range(len(states_after)):
        assert history.redo(model) is True
        assert model.to_json_dict() == states_after[step], (
            f"redo step {step} diverged from the captured state"
        )
    assert history.can_redo is False


# ---------------------------------------------------------------------------
# TC21 — node rename cascades into model.edges (replaces the deleted
# test_editor_app_renames.py assertions; pure model/command level here).
# ---------------------------------------------------------------------------


def test_rename_node_cascades_into_edges_forward_and_reverse() -> None:
    """TC21: a RenameNode on a node carrying EDGES overrides rewrites the entry's
    ``u`` / ``v`` for BOTH the forward (u == renamed) and reverse (v == renamed)
    directed overrides, preserving each entry's ``(type, u, v)`` identity and all
    other override keys; undo restores the original names.

    This is the model-native replacement for the retired ``ast_rewrite`` EDGES
    rewriter.
    """
    model = make_override_model()
    # Forward override A -> B (A is renamed) and a reverse override B -> A
    # (A appears as v).  A second BLIND override on B -> G (untouched by the
    # rename) proves only matching endpoints move.
    model.edges = [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"},
        {"type": "LINE", "u": "B", "v": "A", "direction": "rear"},
        {"type": "BLIND", "u": "B", "v": "G", "stop_at_line": "on"},
    ]
    pre_rename_repr = repr(model.to_json_dict())

    cmd = RenameNode("A", "AA")
    cmd.apply(model)

    # Forward entry: u rewritten A -> AA; type + primitive_mode preserved.
    assert model.edges[0] == {
        "type": "LINE",
        "u": "AA",
        "v": "B",
        "primitive_mode": "FAST",
    }
    # Reverse entry: v rewritten A -> AA; type + direction preserved.
    assert model.edges[1] == {
        "type": "LINE",
        "u": "B",
        "v": "AA",
        "direction": "rear",
    }
    # Untouched entry: neither endpoint was the renamed node.
    assert model.edges[2] == {
        "type": "BLIND",
        "u": "B",
        "v": "G",
        "stop_at_line": "on",
    }

    # Undo restores every name exactly.
    cmd.revert(model)
    assert repr(model.to_json_dict()) == pre_rename_repr
    assert model.edges[0]["u"] == "A"
    assert model.edges[1]["v"] == "A"


def test_rename_node_edges_cascade_does_not_alias_original_entries() -> None:
    """TC21: the rename rebuilds each EDGES entry as a fresh dict, so a later
    revert operates on the renamed copy — never mutating an aliased original.

    Guards against an in-place ``entry["u"] = ...`` mutation that would corrupt
    the captured pre-rename state and break undo.
    """
    model = make_override_model()
    original_entry = {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"}
    model.edges = [original_entry]

    RenameNode("A", "AA").apply(model)

    # The model's live entry is renamed...
    assert model.edges[0]["u"] == "AA"
    # ...but the dict object the test still holds is a DIFFERENT object that was
    # not mutated in place (the command rebuilt the list with fresh dicts).
    assert original_entry["u"] == "A"
    assert model.edges[0] is not original_entry


def test_rename_node_through_history_redo_keeps_edges_cascade() -> None:
    """TC21: rename -> undo -> redo through History keeps the EDGES cascade in
    lockstep (redo re-applies the u/v rewrite, undo reverts it)."""
    model = make_override_model()
    model.edges = [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"}
    ]
    history = History()
    original_json = model.to_json_dict()

    history.do(RenameNode("A", "AA"), model)
    renamed_json = model.to_json_dict()
    assert model.edges[0]["u"] == "AA"

    assert history.undo(model) is True
    assert model.to_json_dict() == original_json
    assert model.edges[0]["u"] == "A"

    assert history.redo(model) is True
    assert model.to_json_dict() == renamed_json
    assert model.edges[0]["u"] == "AA"


# ---------------------------------------------------------------------------
# T2 (issue #98) — command-overlay lifecycle: node/edge delete + kind-flip
# cascade the EDGES overlay so a "deleted" override never orphans (R1, R2), and
# an in-place override edit preserves hand-authored geometry (R5).
#
# TC5  DeleteNode + override apply/revert (removed then restored, no orphan)
# TC6  DeleteEdge + override apply/revert; graph_builder does not re-emit
# TC7  kind-flip LINE->BLIND drops the LINE override; Save not rejected
# TC8  no-orphan invariant after a fixed-seed delete/flip sequence
# TC9  SetEdgeOverride edit preserves distance_cm / heading_deg
# ---------------------------------------------------------------------------


#: The lowercase edge-list kind -> SCREAMING overlay ``type`` tag mapping, kept
#: local to the test so the invariant helper does not import a private command
#: constant (it must mirror ``graph_builder.EDGE_LINE`` / ``EDGE_BLIND``).
_SEGMENT_KIND_TO_TYPE_TAG = {"line": "LINE", "blind": "BLIND"}


def make_valid_override_model() -> FieldModel:
    """Return a FULLY validate-clean model carrying a LINE and a BLIND segment.

    Distinct from :func:`make_override_model` (which is convenient but not
    Save-valid: its start node is a ``"line"`` node).  The two tests that run a
    real ``validate()`` -- directly (TC7) or transitively through
    ``build_graph`` (TC6, which validates its input) -- need a field that passes
    every schema rule, so the override-cascade behavior is what's under test,
    not an unrelated rejection.

    Topology: ``S`` is the blind (dead-reckoned) spawn, ``S``--``A`` a blind
    edge, ``A``--``B`` the LINE segment an override attaches to, and ``B``--``G``
    a second blind edge.  Every node is reachable.
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
        nodes={
            "S": (10.0, 10.0),
            "A": (100.0, 10.0),
            "B": (200.0, 10.0),
            "G": (250.0, 120.0),
        },
        node_types={"S": "blind", "A": "line", "B": "line", "G": "blind"},
        line_segments=[("A", "B")],
        blind_edges=[("S", "A"), ("B", "G")],
        obstacles=[],
    )


def _undirected(pair: tuple[str, str]) -> frozenset[str]:
    """Return the order-independent key for an undirected node pair."""
    return frozenset(pair)


def _assert_no_orphan_overrides(model: FieldModel) -> None:
    """Assert every ``model.edges`` entry has a live SAME-TYPE segment.

    The no-orphan invariant (R1/R2): an overlay entry's identity is
    ``(type, u, v)``, and it may only exist while a segment of that exact type
    still connects ``{u, v}``.  A LINE override needs a live ``line_segments``
    pair; a BLIND override needs a live ``blind_edges`` pair.  An entry that
    survives without its underlying segment is an orphan -- ``graph_builder``
    would re-emit the "deleted" edge, and a node-delete orphan fails the Save
    validator ("EDGES references unknown node").
    """
    live_line = {_undirected(seg) for seg in model.line_segments}
    live_blind = {_undirected(seg) for seg in model.blind_edges}
    for entry in model.edges:
        pair = frozenset({entry["u"], entry["v"]})
        if entry["type"] == _SEGMENT_KIND_TO_TYPE_TAG["line"]:
            assert pair in live_line, (
                f"orphaned LINE override {entry!r}: no live line_segments pair"
            )
        elif entry["type"] == _SEGMENT_KIND_TO_TYPE_TAG["blind"]:
            assert pair in live_blind, (
                f"orphaned BLIND override {entry!r}: no live blind_edges pair"
            )
        else:  # pragma: no cover - defensive: unknown tag is itself a bug
            raise AssertionError(f"unexpected override type tag: {entry!r}")


# ---------------------------------------------------------------------------
# TC5 — DeleteNode cascades its incident overrides (apply removes, revert
# restores), so deleting a node carrying an override leaves no orphan.
# ---------------------------------------------------------------------------


def test_delete_node_cascades_incident_overrides_and_reverts() -> None:
    """TC5: DeleteNode.apply removes every overlay entry incident to the node
    (either endpoint, any type) and revert re-inserts each at its original
    index; a subsequent state has no orphaned override referencing the node."""
    model = make_override_model()
    # Make G isolated-deletable: only the B--G blind edge references it, and
    # G is not the start node. Author two overrides incident to G (forward and
    # reverse) plus one NOT incident, to prove only the incident ones cascade
    # and the untouched one keeps its position.
    model.edges = [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"},
        {"type": "BLIND", "u": "B", "v": "G", "stop_at_line": "on"},
        {"type": "BLIND", "u": "G", "v": "B", "direction": "rear"},
    ]
    # Remove the only segment referencing G so DeleteNode is not blocked by the
    # edge guard (the override cascade is what we are exercising, not the guard).
    model.blind_edges = []
    pre_apply_repr = repr(model.to_json_dict())

    cmd = DeleteNode("G")
    cmd.apply(model)

    # Both G-incident overrides are gone; the A--B override survives untouched.
    assert model.edges == [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"}
    ]
    assert "G" not in model.nodes
    # No surviving entry references the deleted node (no orphan).
    for entry in model.edges:
        assert entry["u"] != "G" and entry["v"] != "G"

    cmd.revert(model)
    # Every captured override is back at its exact original index, node restored.
    assert "G" in model.nodes
    assert repr(model.to_json_dict()) == pre_apply_repr


def test_delete_node_override_cascade_survives_history_undo_redo() -> None:
    """TC5 (cont.): the cascade round-trips through the real History stack --
    undo restores node + overrides, redo re-removes them."""
    model = make_override_model()
    model.edges = [
        {"type": "BLIND", "u": "B", "v": "G", "stop_at_line": "on"},
        {"type": "BLIND", "u": "G", "v": "B", "direction": "rear"},
    ]
    model.blind_edges = []
    history = History()
    original_json = model.to_json_dict()

    history.do(DeleteNode("G"), model)
    deleted_json = model.to_json_dict()
    assert model.edges == []

    assert history.undo(model) is True
    assert model.to_json_dict() == original_json
    assert len(model.edges) == 2

    assert history.redo(model) is True
    assert model.to_json_dict() == deleted_json
    assert model.edges == []


# ---------------------------------------------------------------------------
# TC6 — DeleteEdge cascades the matching-kind override, so graph_builder no
# longer re-emits the deleted edge; revert restores both segment and override.
# ---------------------------------------------------------------------------


def test_delete_edge_cascades_override_and_graph_builder_does_not_reemit() -> None:
    """TC6: DeleteEdge removes the matching-kind override on the pair (both
    orientations); afterwards graph_builder does NOT re-emit the deleted edge,
    and revert restores the segment + override exactly."""
    from tools.path_planner.graph_builder import build_graph

    model = make_valid_override_model()
    # A--B is a LINE segment; author a LINE override on it (forward + reverse).
    model.edges = [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"},
        {"type": "LINE", "u": "B", "v": "A", "direction": "rear"},
    ]
    pre_apply_repr = repr(model.to_json_dict())

    # Sanity: before the delete, the graph contains the A<->B directed pair.
    graph_before = build_graph(model.to_module_proxy())
    assert graph_before.has_edge("A", "B")
    assert graph_before.has_edge("B", "A")

    cmd = DeleteEdge("A", "B")
    cmd.apply(model)

    # The LINE segment is gone AND both LINE overrides on the pair are gone.
    assert ("A", "B") not in model.line_segments
    assert model.edges == []

    # graph_builder must NOT re-emit A<->B: without the cascade the orphaned
    # override would put the "deleted" edge straight back into the graph.
    graph_after = build_graph(model.to_module_proxy())
    assert not graph_after.has_edge("A", "B")
    assert not graph_after.has_edge("B", "A")

    cmd.revert(model)
    assert ("A", "B") in model.line_segments
    assert repr(model.to_json_dict()) == pre_apply_repr


def test_delete_edge_cascade_matches_only_the_deleted_kind() -> None:
    """TC6 (identity guard): DeleteEdge removes only the DELETED kind's override
    on the pair; a same-pair override of the OTHER kind (with a live segment of
    that other kind) is left untouched."""
    model = make_override_model()
    # A--B is a LINE segment. Add a BLIND segment on the SAME pair is illegal
    # (mutually exclusive), so instead use the B--G blind segment and put a
    # BLIND override on A--B that has NO live segment is the orphan we DON'T
    # want to touch here -- so test the realistic case: a LINE override on the
    # deleted LINE pair AND a BLIND override on a DIFFERENT live blind pair.
    model.edges = [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"},
        {"type": "BLIND", "u": "B", "v": "G", "stop_at_line": "on"},
    ]

    DeleteEdge("A", "B").apply(model)

    # Only the LINE override on A--B was cascaded; the BLIND override on the
    # still-live B--G blind segment survives.
    assert model.edges == [
        {"type": "BLIND", "u": "B", "v": "G", "stop_at_line": "on"}
    ]


# ---------------------------------------------------------------------------
# TC7 — kind-flip LINE->BLIND (the MacroCommand the editor dispatches) drops the
# stale LINE override via DeleteEdge's cascade, so the post-flip model has no
# wrong-typed override and Save is not rejected.
# ---------------------------------------------------------------------------


def test_kind_flip_line_to_blind_drops_line_override_and_save_ok() -> None:
    """TC7: flipping A--B from LINE to BLIND through the editor's
    MacroCommand([DeleteEdge, AddBlindEdge]) removes the stale LINE override
    (DeleteEdge is the sole owner of that clear), leaving no LINE-typed override
    on a now-BLIND segment; validate() accepts the result."""
    model = make_valid_override_model()
    # Re-type A / B to blind so the post-flip topology (A--B as a blind edge
    # between two nodes) is fully warning-clean -- this isolates the assertion to
    # "no wrong-typed override survives", not an unrelated line-node warning.
    model.node_types["A"] = "blind"
    model.node_types["B"] = "blind"
    # A--B carries a LINE override; flip it to a BLIND edge.
    model.edges = [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"}
    ]
    pre_flip_repr = repr(model.to_json_dict())

    flip = MacroCommand([DeleteEdge("A", "B"), AddBlindEdge("A", "B")])
    flip.apply(model)

    # The segment is now blind, and NO stale LINE override survives on the pair.
    assert ("A", "B") in model.blind_edges
    assert ("A", "B") not in model.line_segments
    assert model.edges == []
    _assert_no_orphan_overrides(model)

    # Save must not be rejected: no LINE override on a BLIND segment, no
    # orphaned reference. (The model is otherwise a valid field.)
    model.validate()

    # The MacroCommand revert restores both the segment and the override.
    flip.revert(model)
    assert ("A", "B") in model.line_segments
    assert repr(model.to_json_dict()) == pre_flip_repr


def test_kind_flip_macro_revert_through_history_restores_override() -> None:
    """TC7 (cont.): the kind-flip macro round-trips through History -- undo
    restores the LINE segment + its override, redo re-flips and re-drops it."""
    model = make_override_model()
    model.edges = [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"}
    ]
    history = History()
    original_json = model.to_json_dict()

    history.do(
        MacroCommand([DeleteEdge("A", "B"), AddBlindEdge("A", "B")]), model
    )
    flipped_json = model.to_json_dict()
    assert model.edges == []
    assert ("A", "B") in model.blind_edges

    assert history.undo(model) is True
    assert model.to_json_dict() == original_json
    assert model.edges == [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"}
    ]

    assert history.redo(model) is True
    assert model.to_json_dict() == flipped_json
    assert model.edges == []


# ---------------------------------------------------------------------------
# TC8 — no-orphan invariant after a fixed-seed delete / edge-delete / kind-flip
# sequence: every surviving model.edges entry has a live same-type segment.
# ---------------------------------------------------------------------------


def _make_invariant_seed_model() -> FieldModel:
    """Return a model with a few nodes, line + blind segments, and overrides on
    several of them, for the fixed-seed mutation fuzz (TC8)."""
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
        nodes={
            "S": (10.0, 10.0),
            "A": (100.0, 10.0),
            "B": (200.0, 10.0),
            "C": (300.0, 10.0),
            "D": (400.0, 10.0),
        },
        node_types={
            "S": "line",
            "A": "line",
            "B": "line",
            "C": "blind",
            "D": "blind",
        },
        line_segments=[("S", "A"), ("A", "B")],
        blind_edges=[("B", "C"), ("C", "D")],
        obstacles=[],
        edges=[
            {"type": "LINE", "u": "S", "v": "A", "primitive_mode": "FAST"},
            {"type": "LINE", "u": "A", "v": "B", "direction": "front"},
            {"type": "BLIND", "u": "B", "v": "C", "stop_at_line": "on"},
            {"type": "BLIND", "u": "C", "v": "D", "primitive_mode": "GYRO"},
            # A reverse-direction override on a live pair, to exercise both
            # orientations of the DeleteEdge cascade.
            {"type": "BLIND", "u": "D", "v": "C", "direction": "rear"},
        ],
    )


def test_no_orphan_override_invariant_after_fixed_seed_sequence() -> None:
    """TC8: apply a deterministic (fixed-seed) sequence of edge-deletes and
    kind-flips; after EVERY step the no-orphan invariant holds -- no surviving
    overlay entry lacks a live segment of its exact type."""
    model = _make_invariant_seed_model()
    rng = random.Random(20260624)

    # The undirected pairs we will churn, with each pair's current kind tracked
    # so a flip targets the right Add command.
    pairs = [("S", "A"), ("A", "B"), ("B", "C"), ("C", "D")]

    # Precondition: the seed itself is orphan-free.
    _assert_no_orphan_overrides(model)

    for _ in range(40):
        a, b = rng.choice(pairs)
        # Determine the pair's current kind (it may have been flipped already).
        in_line = (a, b) in model.line_segments or (b, a) in model.line_segments
        in_blind = (a, b) in model.blind_edges or (b, a) in model.blind_edges
        if not (in_line or in_blind):
            # The pair was deleted outright on a prior step; re-add it as a
            # line edge so the churn continues (no override attached).
            AddLineEdge(a, b).apply(model)
            _assert_no_orphan_overrides(model)
            continue

        action = rng.choice(("delete", "flip"))
        if action == "delete":
            DeleteEdge(a, b).apply(model)
        else:  # flip to the opposite kind
            add = AddBlindEdge(a, b) if in_line else AddLineEdge(a, b)
            MacroCommand([DeleteEdge(a, b), add]).apply(model)

        # INVARIANT: after every mutating step, no override is orphaned.
        _assert_no_orphan_overrides(model)


# ---------------------------------------------------------------------------
# TC9 — SetEdgeOverride edit of an entry carrying hand-authored geometry
# (distance_cm / heading_deg) preserves those keys; an editor-owned key absent
# from the edit is removed (inherit).
# ---------------------------------------------------------------------------


def test_set_edge_override_replace_preserves_hand_authored_geometry() -> None:
    """TC9: editing an existing override that carries hand-authored
    distance_cm / heading_deg keeps those (non-editor-owned) keys after the
    replace; the editor-owned keys are updated."""
    model = make_override_model()
    # Seed an entry with geometry plus an editor-owned primitive_mode, as a
    # hand-authored field_data.json EDGES entry would carry.
    model.edges = [
        {
            "type": "LINE",
            "u": "A",
            "v": "B",
            "primitive_mode": "FAST",
            "distance_cm": 42.5,
            "heading_deg": 17.0,
        }
    ]

    # Edit only the editor-owned primitive_mode (the inline editor's add-form
    # carries no geometry controls, so distance_cm / heading_deg are absent).
    SetEdgeOverride("LINE", "A", "B", primitive_mode="SLOW").apply(model)

    entry = model.edges[0]
    # Editor-owned key updated...
    assert entry["primitive_mode"] == "SLOW"
    # ...and the hand-authored geometry survives the in-place replace.
    assert entry["distance_cm"] == 42.5
    assert entry["heading_deg"] == 17.0
    # Identity intact, single entry (replaced in place, no duplicate).
    assert len(model.edges) == 1
    assert (entry["type"], entry["u"], entry["v"]) == ("LINE", "A", "B")


def test_set_edge_override_replace_removes_omitted_editor_key_keeps_geometry() -> None:
    """TC9 (cont.): an editor-owned key present on the prior entry but ABSENT
    from the edit is removed (set-to-inherit), while non-editor-owned geometry
    keys still survive."""
    model = make_override_model()
    model.edges = [
        {
            "type": "BLIND",
            "u": "B",
            "v": "G",
            "primitive_mode": "GYRO",
            "stop_at_line": "on",
            "distance_cm": 60.0,
        }
    ]

    # Re-author with ONLY direction set: primitive_mode and stop_at_line are
    # absent from the form -> they must be removed (inherit), geometry survives.
    SetEdgeOverride("BLIND", "B", "G", direction="rear").apply(model)

    entry = model.edges[0]
    assert entry["direction"] == "rear"
    assert "primitive_mode" not in entry  # omitted -> removed (inherit)
    assert "stop_at_line" not in entry  # omitted -> removed (inherit)
    assert entry["distance_cm"] == 60.0  # non-editor-owned key preserved
    assert len(model.edges) == 1


def test_set_edge_override_replace_preserving_geometry_reverts_exactly() -> None:
    """TC9 (cont.): reverting a geometry-preserving replace restores the EXACT
    prior entry (geometry and all), not a stripped override-only entry."""
    model = make_override_model()
    original_entry = {
        "type": "LINE",
        "u": "A",
        "v": "B",
        "primitive_mode": "FAST",
        "distance_cm": 42.5,
        "heading_deg": 17.0,
    }
    model.edges = [dict(original_entry)]
    pre_replace_repr = repr(model.to_json_dict())

    cmd = SetEdgeOverride("LINE", "A", "B", primitive_mode="SLOW")
    cmd.apply(model)
    assert model.edges[0]["primitive_mode"] == "SLOW"

    cmd.revert(model)
    assert model.edges[0] == original_entry
    assert repr(model.to_json_dict()) == pre_replace_repr
