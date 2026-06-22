"""On-model ``projection`` node-type tests for the live field editor (issue #64).

Issue #64 retired the off-model projection EDGE machinery (the ``ProjectionDelta``
store, the ``AddProjectionEdge`` / ``DeleteProjectionEdge`` commands, the LibCST
``append/remove_projection_edge`` writers, the "+ add projection" panel button,
and the freed ``P`` keybinding). Projection is now a mutually-exclusive 4th node
TYPE -- a line crossing with no junction marker, dead-reckoned to -- set through
the SAME ``SetNodeType`` command and node-type combobox the ``line`` / ``blind``
types use, with no projection sub-panel (it mirrors the ``reset`` precedent minus
the reset spec). These tests replace the deleted ``test_editor_projection*.py``
suites with the node-model coverage:

* **TC17** -- setting a node's type to ``"projection"`` via the model /
  ``SetNodeType`` command persists through a save -> reload round-trip, and a
  projection node renames + deletes through the normal on-model cascade (no
  ``_proj_delta`` / off-model bookkeeping). Follows the ``reset``-node test
  precedent (``test_editor_property_commands.py`` / ``test_editor_model.py``).
* **Freed ``P`` keybinding** -- the editor no longer binds ``P`` / ``p`` to a
  projection action (the old project-edge picker). The binding is asserted absent
  from ``EditorApp._wire_keyboard_shortcuts``.

The TC17 tests are pure logic (model + command + loader); no Tk is required. The
keybinding test builds a bare ``EditorApp.__new__`` instance and drives
``_wire_keyboard_shortcuts`` against a fake root that records every ``bind_all``
sequence, so no real Tk window is built. The matplotlib import dance mirrors the
sibling editor tests: pin Agg and swallow ``editor.canvas``'s import-time
``use("TkAgg")``.
"""

from __future__ import annotations

import matplotlib

# Pin the headless-safe Agg backend before importing the editor, then no-op the
# one ``matplotlib.use("TkAgg")`` switch that ``editor.canvas`` performs at
# import time (it would raise on headless CI and clobber the Agg backend the
# other tests rely on). Restore the real ``use`` immediately afterwards.
matplotlib.use("Agg")
_real_mpl_use = matplotlib.use
matplotlib.use = lambda *args, **kwargs: None
try:
    from tools.path_planner.editor.app import EditorApp  # noqa: E402
finally:
    matplotlib.use = _real_mpl_use

from tools.path_planner.editor import loader  # noqa: E402
from tools.path_planner.editor.commands import (  # noqa: E402
    DeleteNode,
    RenameNode,
    SetNodeType,
)
from tools.path_planner.editor.model import FieldModel  # noqa: E402

#: The node converted to a projection type in the TC17 cases. ``P`` is on the
#: ``A`` -- ``P`` -- ``B`` line, a non-junction crossing reached by a blind seek,
#: which is exactly what a projection node models.
_PROJ_NODE: str = "P"


def make_model() -> FieldModel:
    """Return a small, fully-valid :class:`FieldModel` carrying a projection node.

    Topology: ``S`` (blind dead-reckon spawn) -- BLIND --> ``A`` -- LINE -->
    ``P`` -- LINE --> ``B``, with ``P`` typed ``"projection"`` (an on-line
    non-junction waypoint reached by the blind ``S`` -> ... seek). ``A`` and ``B``
    are junction ``line`` nodes; ``S`` is the START node because the issue #64
    validator rejects a ``"line"`` / ``"projection"`` start (an on-line spawn
    presumes a sensor-anchored arrival the chassis has not made at t=0).

    The field validates clean -- a LINE edge into a projection node is legal (the
    line passes through it; the hop is distance-terminated, not junction-anchored)
    -- so a save -> reload round-trip is a meaningful persistence check.
    """
    return FieldModel(
        field_mm=(3000.0, 2000.0),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="S",
        start_heading_deg=0.0,
        nodes={
            "S": (10.0, 100.0),
            "A": (10.0, 10.0),
            "P": (100.0, 10.0),
            "B": (200.0, 10.0),
        },
        node_types={
            "S": "blind",
            "A": "line",
            "P": "projection",
            "B": "line",
        },
        line_segments=[("A", "P"), ("P", "B")],
        blind_edges=[("S", "A")],
        obstacles=[],
    )


# ---------------------------------------------------------------------------
# TC17 -- set type=projection via SetNodeType persists through save/reload
# ---------------------------------------------------------------------------


def test_set_node_type_projection_applies_and_reverts() -> None:
    """TC17: ``SetNodeType(name, "projection")`` types the node projection and
    revert restores its prior type -- projection rides the SAME command the
    ``line`` / ``blind`` plain types use (no projection-specific command)."""
    model = make_model()
    # B starts as a junction line node; retype it projection through SetNodeType.
    assert model.node_types["B"] == "line"

    cmd = SetNodeType("B", "projection")
    cmd.apply(model)
    assert model.node_types["B"] == "projection"

    cmd.revert(model)
    assert model.node_types["B"] == "line"


def test_projection_type_round_trips_to_from_json_dict() -> None:
    """TC17: a projection node survives ``from_json_dict(to_json_dict(model))``.

    ``"projection"`` rides ``node_types`` through both serialization surfaces
    (no extra plumbing -- the ``reset`` precedent minus the reset spec), so the
    rebuilt model is equal and re-types the node ``"projection"``.
    """
    model = make_model()
    assert model.node_types[_PROJ_NODE] == "projection"

    payload = model.to_json_dict()
    assert payload["NODE_TYPES"][_PROJ_NODE] == "projection"

    rebuilt = FieldModel.from_json_dict(payload)
    assert rebuilt == model
    assert rebuilt.node_types[_PROJ_NODE] == "projection"


def test_projection_node_persists_through_save_reload(tmp_path) -> None:
    """TC17: a Save (dump_field_json) + reload (load_field_json) of a model whose
    node was typed ``"projection"`` via ``SetNodeType`` preserves the type and a
    model that validates -- the editor's full open -> edit -> save -> open path."""
    model = make_model()
    # Type B projection through the command, exactly as the inspector combobox
    # would dispatch it, then Save.
    SetNodeType("B", "projection").apply(model)
    model.validate()  # the edited field is legal (LINE -> projection is valid)

    json_path = tmp_path / "field_data.json"
    loader.dump_field_json(model.to_json_dict(), json_path)
    reloaded = FieldModel.from_json_dict(loader.load_field_json(json_path))

    assert reloaded == model
    reloaded.validate()
    assert reloaded.node_types["B"] == "projection"
    assert reloaded.node_types[_PROJ_NODE] == "projection"


# ---------------------------------------------------------------------------
# TC17 -- rename / delete of a projection node go through the normal on-model
# cascade (no _proj_delta off-model bookkeeping).
# ---------------------------------------------------------------------------


def test_rename_projection_node_cascades_through_node_types() -> None:
    """TC17: renaming a projection node carries its ``"projection"`` typing to the
    new name through the normal ``RenameNode`` cascade -- no off-model delta."""
    model = make_model()

    cmd = RenameNode(_PROJ_NODE, "P2")
    cmd.apply(model)

    # The node and its projection typing moved to the new name; the line edges
    # that flanked it followed (the on-model cascade), and the field still
    # validates (projection is still LINE-flanked on both sides).
    assert "P2" in model.nodes and _PROJ_NODE not in model.nodes
    assert model.node_types["P2"] == "projection"
    assert _PROJ_NODE not in model.node_types
    assert ("A", "P2") in model.line_segments
    assert ("P2", "B") in model.line_segments
    model.validate()

    cmd.revert(model)
    assert _PROJ_NODE in model.node_types
    assert model.node_types[_PROJ_NODE] == "projection"
    assert "P2" not in model.nodes


def test_delete_projection_node_cascades_through_node_types() -> None:
    """TC17: deleting a projection node (after detaching its edges) removes it from
    ``node_types`` through the normal ``DeleteNode`` cascade, and revert restores
    its projection typing -- no off-model ``_proj_delta`` is involved."""
    model = make_model()
    # DeleteNode refuses while edges still reference the node, so detach the two
    # LINE edges flanking the projection node first (the on-model precondition).
    model.line_segments = [
        edge for edge in model.line_segments if _PROJ_NODE not in edge
    ]

    cmd = DeleteNode(_PROJ_NODE)
    cmd.apply(model)
    assert _PROJ_NODE not in model.nodes
    assert _PROJ_NODE not in model.node_types

    cmd.revert(model)
    assert _PROJ_NODE in model.nodes
    assert model.node_types[_PROJ_NODE] == "projection"


# ---------------------------------------------------------------------------
# Freed ``P`` keybinding -- the editor no longer binds P / p to a projection
# action (the old project-edge picker, removed in issue #64 / T17).
# ---------------------------------------------------------------------------


class _BindingRecorder:
    """A fake Tk root that records every ``bind_all`` sequence string.

    ``EditorApp._wire_keyboard_shortcuts`` only calls ``self._root.bind_all``
    (plus the real ``self._shortcut_guard`` / handler-factory methods, which run
    on a bare instance), so recording the bound sequences is enough to assert the
    full keybinding set without building a real Tk window.
    """

    def __init__(self) -> None:
        self.sequences: list[str] = []

    def bind_all(self, sequence: str, _handler) -> None:
        self.sequences.append(sequence)


def test_p_key_is_not_bound_to_any_editor_action() -> None:
    """The freed ``P`` / ``p`` keybinding is gone: ``_wire_keyboard_shortcuts``
    binds no ``<KeyPress-p>`` / ``<KeyPress-P>`` sequence.

    Issue #64 (T17) removed the off-model projection-edge picker that ``P`` used
    to open, so the key must no longer map to any handler. Driving the real
    ``_wire_keyboard_shortcuts`` against a recording fake root proves the binding
    is absent from the actual registration, not merely from the docstring.
    """
    app = EditorApp.__new__(EditorApp)
    recorder = _BindingRecorder()
    app._root = recorder

    app._wire_keyboard_shortcuts()

    # No projection-key binding survives. Check both case forms and the bare key
    # so a re-introduced ``<KeyPress-p>`` / ``<KeyPress-P>`` (or a stray ``<p>``)
    # fails this guard loudly.
    forbidden = {"<KeyPress-p>", "<KeyPress-P>", "<p>", "<P>"}
    assert forbidden.isdisjoint(recorder.sequences), (
        "the freed P keybinding must not map to any editor action, but found "
        f"{sorted(forbidden.intersection(recorder.sequences))}"
    )
    # Sanity: the registration actually ran (the known mode/edit keys are present),
    # so the disjoint check above is meaningful and not asserting on an empty set.
    assert "<KeyPress-l>" in recorder.sequences
    assert "<Control-s>" in recorder.sequences
