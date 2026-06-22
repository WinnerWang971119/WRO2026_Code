"""Tests for the property-inspector commands (T9 of the editor de-lag plan).

Covers the new ``AddPivotTurn`` / ``DeletePivotTurn`` / ``SetResetSpec`` /
``ClearResetSpec`` / ``SetGlobalKnob`` commands at the command + model level --
no Tk, no canvas. Mirrors the direct-command test style of
``test_editor_commands.py``.

Test cases (from the plan):

* **TC5** -- ``AddPivotTurn`` / ``DeletePivotTurn`` apply -> revert byte-identity,
  plus their refuse-before-mutation pre-conditions.
* **TC6** -- ``SetResetSpec`` / ``ClearResetSpec`` keep ``node_types`` and
  ``reset_nodes`` in lockstep through apply and revert.
* **TC7** -- ``SetGlobalKnob`` apply -> revert per knob, plus per-knob bound
  rejection leaves the model untouched.
* **TC12** -- a full JSON round-trip with pivots + a VALID reset node + non-default
  knobs reproduces an equal model, validates, and surfaces the edits in
  ``to_json_dict`` (optionally through the loader's tmp_path dump/load).
* **TC13** -- reset type<->spec divergence is rejected at Save by
  ``model.validate()`` (the gate the inline tier does not cover).
* **Bounds-parity guard** -- the app's inline ``_FLOAT_KNOB_BOUNDS`` agree with the
  per-knob bounds ``SetGlobalKnob`` enforces, so the two tiers cannot drift.

The reset geometry below is authored to PASS ``validator._validate_reset_nodes``:
node ``G`` sits at ``(250, 80)`` cm; the wall is at ``x = 280`` cm, so the
node-to-wall standoff is ``30 cm >= WALL_PIVOT_CLEARANCE_CM (14)`` and the parked
facing ``180 deg`` (``cos 180 = -1``) points toward DECREASING x -- away from the
wall, so the rear (at ``facing + 180``) presses it. The divergent TC13 cases
deliberately break the type<->spec agreement the inline tier never re-checks.

No GUI, no file I/O (except the optional TC12 loader round-trip through tmp_path).
"""

from __future__ import annotations

import pytest

from tools.path_planner.editor import loader
from tools.path_planner.editor.commands import (
    AddPivotTurn,
    ClearResetSpec,
    DeletePivotTurn,
    SetGlobalKnob,
    SetResetSpec,
)
from tools.path_planner.editor.model import FieldModel
from tools.path_planner.validator import FieldDataError

# Pull in the app's inline bound table for the bounds-parity guard so the two
# validation tiers are pinned against each other (the T7 review request).
from tools.path_planner.editor.app import _FLOAT_KNOB_BOUNDS


# ---------------------------------------------------------------------------
# Fixtures -- a small, fully-valid FieldModel, plus a VALID reset geometry that
# satisfies validator._validate_reset_nodes (see module docstring).
# ---------------------------------------------------------------------------

#: The node the reset-spec tests convert. It is a plain ``"blind"`` node in the
#: base model, reachable by a BLIND edge -- a legal reset candidate.
_RESET_NODE: str = "G"

#: Reset geometry for ``G`` (at x=250) that PASSES the validator: wall at x=280
#: gives a 30 cm standoff (>= the 14 cm pivot clearance floor), and facing
#: 180 deg points away from the wall (cos 180 = -1, opposite the +x toward-wall
#: direction) so the rear presses it.
_RESET_AXIS: str = "x"
_RESET_WALL_COORD: float = 280.0
_RESET_FACING: float = 180.0


def make_model() -> FieldModel:
    """Return a small, fully-valid :class:`FieldModel` built in memory.

    ``A`` -- ``B`` is a line segment, ``B`` -- ``G`` a blind edge, ``G`` a blind
    node sitting clear of the x=280 wall so it can be converted to a valid reset
    node. The field is 300 x 200 cm so every coordinate is inside the bounds.
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
        nodes={"A": (10.0, 10.0), "B": (200.0, 10.0), "G": (250.0, 80.0)},
        node_types={"A": "line", "B": "line", "G": "blind"},
        line_segments=[("A", "B")],
        blind_edges=[("B", "G")],
        obstacles=[(50.0, 50.0, 80.0, 80.0, "pillar")],
    )


# ---------------------------------------------------------------------------
# TC5 -- AddPivotTurn / DeletePivotTurn apply -> revert + refusals
# ---------------------------------------------------------------------------


def test_add_pivot_turn_apply_then_revert_is_byte_identical() -> None:
    """TC5: AddPivotTurn appends, revert removes -> pivot_turns unchanged."""
    model = make_model()
    snapshot = list(model.pivot_turns)

    cmd = AddPivotTurn("A", "B", "G")
    cmd.apply(model)
    assert model.pivot_turns == [("A", "B", "G")]

    cmd.revert(model)
    assert model.pivot_turns == snapshot


def test_delete_pivot_turn_apply_then_revert_is_byte_identical() -> None:
    """TC5: DeletePivotTurn removes a present triple; revert restores it exactly."""
    model = make_model()
    model.pivot_turns.append(("A", "B", "G"))
    snapshot = list(model.pivot_turns)

    cmd = DeletePivotTurn(("A", "B", "G"))
    cmd.apply(model)
    assert ("A", "B", "G") not in model.pivot_turns

    cmd.revert(model)
    assert model.pivot_turns == snapshot


def test_add_pivot_turn_refuses_duplicate_before_mutation() -> None:
    """TC5: AddPivotTurn refuses an already-present triple, model untouched."""
    model = make_model()
    model.pivot_turns.append(("A", "B", "G"))
    snapshot = list(model.pivot_turns)

    with pytest.raises(ValueError):
        AddPivotTurn("A", "B", "G").apply(model)

    assert model.pivot_turns == snapshot


def test_add_pivot_turn_refuses_unknown_node_before_mutation() -> None:
    """TC5: AddPivotTurn refuses a triple naming an unknown node, model untouched."""
    model = make_model()
    snapshot = list(model.pivot_turns)

    with pytest.raises(ValueError):
        AddPivotTurn("A", "B", "ZZZ").apply(model)

    assert model.pivot_turns == snapshot


def test_delete_pivot_turn_refuses_absent_triple_before_mutation() -> None:
    """TC5: DeletePivotTurn refuses a triple that is not present, model untouched."""
    model = make_model()
    model.pivot_turns.append(("A", "B", "G"))
    snapshot = list(model.pivot_turns)

    with pytest.raises(ValueError):
        DeletePivotTurn(("A", "G", "B")).apply(model)

    assert model.pivot_turns == snapshot


# ---------------------------------------------------------------------------
# TC6 -- SetResetSpec / ClearResetSpec: type<->spec sync + byte-identity
# ---------------------------------------------------------------------------


def test_set_reset_spec_syncs_type_and_entry_then_revert_restores_both() -> None:
    """TC6: SetResetSpec on a plain node types it 'reset' AND creates the entry;
    revert restores BOTH (type back to original, entry removed if absent)."""
    model = make_model()
    assert model.node_types[_RESET_NODE] == "blind"
    assert _RESET_NODE not in model.reset_nodes

    cmd = SetResetSpec(
        _RESET_NODE, _RESET_AXIS, _RESET_WALL_COORD, _RESET_FACING
    )
    cmd.apply(model)

    # Forward: type and spec agree -- both present and consistent.
    assert model.node_types[_RESET_NODE] == "reset"
    assert model.reset_nodes[_RESET_NODE] == {
        "axis": _RESET_AXIS,
        "wall_coord": _RESET_WALL_COORD,
        "facing": _RESET_FACING,
    }

    cmd.revert(model)

    # Revert: type back to original 'blind', entry removed (it was absent).
    assert model.node_types[_RESET_NODE] == "blind"
    assert _RESET_NODE not in model.reset_nodes


def test_set_reset_spec_apply_then_revert_is_byte_identical() -> None:
    """TC6: SetResetSpec apply -> revert leaves the whole model byte-identical."""
    model = make_model()
    pre = repr(model.to_json_dict())

    cmd = SetResetSpec(
        _RESET_NODE, _RESET_AXIS, _RESET_WALL_COORD, _RESET_FACING
    )
    cmd.apply(model)
    cmd.revert(model)

    assert repr(model.to_json_dict()) == pre


def test_clear_reset_spec_removes_entry_and_retypes_then_revert_restores() -> None:
    """TC6: ClearResetSpec removes the entry AND retypes to a plain type;
    revert restores both halves so type and spec never diverge."""
    model = make_model()
    # First make G a valid reset node so there is something to clear.
    SetResetSpec(
        _RESET_NODE, _RESET_AXIS, _RESET_WALL_COORD, _RESET_FACING
    ).apply(model)
    assert model.node_types[_RESET_NODE] == "reset"
    assert _RESET_NODE in model.reset_nodes

    cmd = ClearResetSpec(_RESET_NODE, new_type="blind")
    cmd.apply(model)

    # Forward: entry gone, node retyped to the requested plain type.
    assert _RESET_NODE not in model.reset_nodes
    assert model.node_types[_RESET_NODE] == "blind"

    cmd.revert(model)

    # Revert: reset typing and the spec entry both restored, still in lockstep.
    assert model.node_types[_RESET_NODE] == "reset"
    assert model.reset_nodes[_RESET_NODE] == {
        "axis": _RESET_AXIS,
        "wall_coord": _RESET_WALL_COORD,
        "facing": _RESET_FACING,
    }


def test_clear_reset_spec_apply_then_revert_is_byte_identical() -> None:
    """TC6: ClearResetSpec apply -> revert leaves the model byte-identical."""
    model = make_model()
    SetResetSpec(
        _RESET_NODE, _RESET_AXIS, _RESET_WALL_COORD, _RESET_FACING
    ).apply(model)
    pre = repr(model.to_json_dict())

    cmd = ClearResetSpec(_RESET_NODE, new_type="blind")
    cmd.apply(model)
    cmd.revert(model)

    assert repr(model.to_json_dict()) == pre


def test_reset_type_and_spec_never_diverge_through_set_then_clear() -> None:
    """TC6: after every apply/revert step, a reset typing implies a spec entry and
    vice-versa -- the type and the entry agree at all times."""
    model = make_model()
    set_cmd = SetResetSpec(
        _RESET_NODE, _RESET_AXIS, _RESET_WALL_COORD, _RESET_FACING
    )

    def _assert_consistent() -> None:
        is_reset_typed = model.node_types.get(_RESET_NODE) == "reset"
        has_spec = _RESET_NODE in model.reset_nodes
        assert is_reset_typed == has_spec

    _assert_consistent()
    set_cmd.apply(model)
    _assert_consistent()

    clear_cmd = ClearResetSpec(_RESET_NODE, new_type="blind")
    clear_cmd.apply(model)
    _assert_consistent()
    clear_cmd.revert(model)
    _assert_consistent()
    set_cmd.revert(model)
    _assert_consistent()


# ---------------------------------------------------------------------------
# TC7 -- SetGlobalKnob apply -> revert per knob, plus per-knob bound rejection
# ---------------------------------------------------------------------------

#: Each knob paired with a valid new value distinct from make_model()'s default.
_KNOB_VALID_VALUES: tuple[tuple[str, object], ...] = (
    ("blind_decay_enabled", False),
    ("blind_decay_k", 0.25),
    ("blind_decay_exponent", 3.0),
    ("blind_decay_d_ref", 75.0),
    ("robot_radius_cm", 9.5),
)

#: Each knob paired with a value just outside its accepted range.
_KNOB_BAD_VALUES: tuple[tuple[str, object], ...] = (
    ("blind_decay_k", -0.0001),
    ("blind_decay_exponent", 0.5),
    ("blind_decay_d_ref", 0.0),
    ("robot_radius_cm", 0.0),
)


@pytest.mark.parametrize("knob, new_value", _KNOB_VALID_VALUES)
def test_set_global_knob_apply_then_revert_restores(knob, new_value) -> None:
    """TC7: each knob set then reverted returns to its original value, and the
    coerced new value is observed after apply."""
    model = make_model()
    original = getattr(model, knob)
    assert new_value != original, "fixture value must differ from the default"

    cmd = SetGlobalKnob(knob, new_value)
    cmd.apply(model)
    assert getattr(model, knob) == new_value

    cmd.revert(model)
    assert getattr(model, knob) == original


@pytest.mark.parametrize("knob, bad_value", _KNOB_BAD_VALUES)
def test_set_global_knob_bad_value_raises_before_mutation(knob, bad_value) -> None:
    """TC7: an out-of-range float raises BEFORE mutation -- the model attribute is
    unchanged and no dangling inverse is recorded (revert would assert)."""
    model = make_model()
    original = getattr(model, knob)

    cmd = SetGlobalKnob(knob, bad_value)
    with pytest.raises(ValueError):
        cmd.apply(model)

    # Attribute untouched ...
    assert getattr(model, knob) == original
    # ... and no inverse captured: a revert after a rejected apply must assert,
    # proving the command did not half-commit.
    with pytest.raises(AssertionError):
        cmd.revert(model)


def test_set_global_knob_unknown_name_raises() -> None:
    """TC7: an unrecognized knob name raises and never touches the model."""
    model = make_model()
    pre = repr(model.to_json_dict())

    with pytest.raises(ValueError):
        SetGlobalKnob("not_a_knob", 1.0).apply(model)

    assert repr(model.to_json_dict()) == pre


# ---------------------------------------------------------------------------
# TC12 -- full JSON round-trip with pivots + a VALID reset + non-default knobs
# ---------------------------------------------------------------------------


def _edited_model() -> FieldModel:
    """Return a model carrying a pivot, a VALID reset node, and non-default knobs,
    all applied through the new commands so the result is a real edit history."""
    model = make_model()
    # A pivot whose two legs (A-B line, B-G blind) are real incident edges.
    AddPivotTurn("A", "B", "G").apply(model)
    # A valid reset node (geometry passes the validator, see module docstring).
    SetResetSpec(
        _RESET_NODE, _RESET_AXIS, _RESET_WALL_COORD, _RESET_FACING
    ).apply(model)
    # Non-default global knobs.
    SetGlobalKnob("blind_decay_k", 0.25).apply(model)
    SetGlobalKnob("robot_radius_cm", 9.5).apply(model)
    return model


def test_round_trip_reproduces_equal_model_and_validates() -> None:
    """TC12: from_json_dict(to_json_dict(model)) reproduces an equal model, the
    edited model validates, and the edits appear in to_json_dict output."""
    model = _edited_model()

    # The edited geometry must be a legal field (the round-trip + Save are only
    # meaningful if validate passes).
    model.validate()

    payload = model.to_json_dict()
    # The edits surface in the serialized payload.
    assert payload["pivot_turns"] == [["A", "B", "G"]]
    assert payload["RESET_NODES"] == {
        _RESET_NODE: {
            "axis": _RESET_AXIS,
            "wall_coord": _RESET_WALL_COORD,
            "facing": _RESET_FACING,
        }
    }
    assert payload["BLIND_DECAY_K"] == 0.25
    assert payload["ROBOT_RADIUS_CM"] == 9.5
    assert payload["NODE_TYPES"][_RESET_NODE] == "reset"

    # Stable round-trip: rebuilding from the payload yields an equal model.
    rebuilt = FieldModel.from_json_dict(payload)
    assert rebuilt == model
    # And the rebuilt model still validates and re-serializes identically.
    rebuilt.validate()
    assert rebuilt.to_json_dict() == payload


def test_round_trip_through_loader_tmp_path_preserves_edits(tmp_path) -> None:
    """TC12: a Save (dump_field_json) + reload (load_field_json) through tmp_path
    preserves the pivots, the reset node, and the non-default knobs."""
    model = _edited_model()
    model.validate()

    json_path = tmp_path / "field_data.json"
    loader.dump_field_json(model.to_json_dict(), json_path)
    reloaded = FieldModel.from_json_dict(loader.load_field_json(json_path))

    assert reloaded == model
    reloaded.validate()
    assert reloaded.pivot_turns == [("A", "B", "G")]
    assert reloaded.reset_nodes == {
        _RESET_NODE: {
            "axis": _RESET_AXIS,
            "wall_coord": _RESET_WALL_COORD,
            "facing": _RESET_FACING,
        }
    }
    assert reloaded.blind_decay_k == 0.25
    assert reloaded.robot_radius_cm == 9.5


# ---------------------------------------------------------------------------
# TC13 -- reset type<->spec divergence rejected at Save (validate)
# ---------------------------------------------------------------------------


def test_validate_rejects_reset_typed_node_without_spec() -> None:
    """TC13: a node typed 'reset' with NO reset_nodes entry is rejected by
    model.validate() -- the Save gate the inline tier does not cover."""
    model = make_model()
    # Hand-craft the divergence: type says reset, no matching spec entry.
    model.node_types[_RESET_NODE] = "reset"
    assert _RESET_NODE not in model.reset_nodes

    with pytest.raises(FieldDataError):
        model.validate()


def test_validate_rejects_spec_entry_on_non_reset_typed_node() -> None:
    """TC13: a reset_nodes entry whose node is NOT typed 'reset' is rejected by
    model.validate() (the other divergence direction)."""
    model = make_model()
    # Hand-craft the reverse divergence: a spec entry, but the node stays blind.
    model.reset_nodes[_RESET_NODE] = {
        "axis": _RESET_AXIS,
        "wall_coord": _RESET_WALL_COORD,
        "facing": _RESET_FACING,
    }
    assert model.node_types[_RESET_NODE] != "reset"

    with pytest.raises(FieldDataError):
        model.validate()


# ---------------------------------------------------------------------------
# Bounds-parity guard -- inline _FLOAT_KNOB_BOUNDS agree with SetGlobalKnob
# ---------------------------------------------------------------------------


def test_inline_bounds_and_command_reject_the_same_boundary_values() -> None:
    """Pin the two validation tiers together: for every knob in the app's inline
    ``_FLOAT_KNOB_BOUNDS``, the just-out-of-range value the inline guard rejects
    is ALSO rejected by ``SetGlobalKnob`` -- so the cheap tier-1 copy and the
    authoritative command can never silently drift (T7 review request)."""
    # A tiny epsilon to step just past the bound. The bound is the smallest
    # accepted value for an inclusive knob (so bound - eps is the first reject),
    # or the bound itself for an exclusive knob (the bound is the first reject).
    epsilon = 1e-4

    for knob, (minimum, inclusive) in _FLOAT_KNOB_BOUNDS.items():
        if inclusive:
            # value just BELOW the inclusive floor must be rejected by both.
            reject_value = minimum - epsilon
            inline_accepts = reject_value >= minimum
        else:
            # the exclusive floor itself must be rejected by both.
            reject_value = minimum
            inline_accepts = reject_value > minimum

        assert not inline_accepts, (
            f"inline guard for {knob!r} should reject {reject_value}"
        )

        model = make_model()
        original = getattr(model, knob)
        with pytest.raises(ValueError):
            SetGlobalKnob(knob, reject_value).apply(model)
        # The command rejected it before mutating, matching the inline verdict.
        assert getattr(model, knob) == original
