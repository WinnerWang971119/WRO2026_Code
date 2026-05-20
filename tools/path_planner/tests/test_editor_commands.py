"""Tests for the Command pattern and History undo/redo stack.

Covers TC4 (snapshot/restore on AddNode), TC5 (RenameNode cascade),
TC6 (DeleteNode refusal), TC13 (10-mutation undo round-trip),
TC14 (undo + redo state equality).

No GUI, no file I/O -- everything is built in memory.
"""

from __future__ import annotations

import pytest

from tools.path_planner.editor.commands import (
    AddBlindEdge,
    AddLineEdge,
    AddNode,
    AddObstacle,
    DeleteEdge,
    DeleteNode,
    MoveNode,
    RenameNode,
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
