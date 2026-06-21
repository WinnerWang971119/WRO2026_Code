"""Tests for the projection-edge stack of the editor restyle wave (T9).

Pure-logic, Tk-free coverage of the three projection-edge layers the restyle
added:

* **TC1 / TC2** -- the LibCST source mutators
  ``ast_rewrite.append_projection_edge`` / ``remove_projection_edge``: a single
  append adds one ``EDGE_PROJECTION`` dict to ``field_data.py``'s ``EDGES``
  list with the four required keys and the ``type`` value emitted as the *bare*
  ``EDGE_PROJECTION`` name (never a quoted string), a second append of the same
  ``(u, v)`` is a no-op that preserves the file bytes + mtime, and the **W1**
  regression guard proves a list-valued key nested inside an entry (e.g.
  ``"waypoints": [...]``) is never grafted into or matched against.
* **TC3** -- the deterministic fold ``projection_edges.reconcile``: adds
  override loaded on ``(u, v)``, removes suppress, duplicates collapse, and the
  emit order is loaded-first (source order) then adds (user order).
* **TC4** -- the undoable ``AddProjectionEdge`` / ``DeleteProjectionEdge``
  commands over an app-owned ``ProjectionDelta``: apply -> revert restores the
  delta byte-identically and redo re-applies, across all four mutation branches
  (``"appended"`` / ``"uncanceled_remove"`` / ``"dropped_add"`` /
  ``"added_remove"``).

These modules are deliberately Tk-free and matplotlib-free, so this file needs
neither the shared ``tk_root`` fixture nor the Agg pin -- it imports the editor
logic packages directly.
"""

from __future__ import annotations

import ast
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.path_planner.editor import projection_edges
from tools.path_planner.editor.ast_rewrite import (
    append_projection_edge,
    remove_projection_edge,
)
from tools.path_planner.editor.commands import (
    AddProjectionEdge,
    DeleteProjectionEdge,
    ProjectionDelta,
)
from tools.path_planner.graph_builder import EDGE_PROJECTION

# ---------------------------------------------------------------------------
# Shared source fixtures (plain string constants -- mirrors the style of
# ``test_editor_ast_rewrite.py``). Each declares ``EDGE_PROJECTION`` as a bare
# imported name so an appended ``"type": EDGE_PROJECTION`` resolves, and an
# ``EDGES`` list the mutators append into / remove from.
# ---------------------------------------------------------------------------

# Minimal source with an EMPTY ``EDGES`` list -- the first-append target.
_SOURCE_EMPTY_EDGES = '''\
"""Toy field for projection-edge append tests."""

from tools.path_planner.graph_builder import EDGE_PROJECTION

# Comments here must survive every append / remove.
EDGES = []
'''

# Source whose single ``EDGES`` entry carries a LIST-valued key
# (``"waypoints"``). The W1 guard asserts an append never mutates that nested
# list, and a remove of a ``(u, v)`` that exists only inside it is a no-op.
_SOURCE_LIST_VALUED_KEY = '''\
"""Toy field with a list-valued key inside an EDGES entry (W1 guard)."""

from tools.path_planner.graph_builder import EDGE_PROJECTION

EDGES = [
    {
        "type": "EDGE_LINE",
        "u": "A",
        "v": "B",
        "distance_cm": 50.0,
        "heading_deg": 0.0,
        "waypoints": [1, 2, 3],
    },
]
'''

# Required keys on the appended projection dict (matches
# ``projection_edges._REQUIRED_EDGE_KEYS`` plus the bare ``type`` name).
_REQUIRED_PROJECTION_KEYS: tuple[str, ...] = (
    "type",
    "u",
    "v",
    "distance_cm",
    "heading_deg",
)


def _write(tmp_path: Path, name: str, content: str) -> Path:
    """Write *content* to ``tmp_path/name`` as UTF-8 with LF newlines.

    Forces ``newline="\\n"`` to match what the mutators write back, so a
    byte-identity assertion on a no-op never trips over a CRLF / LF mismatch on
    Windows.

    :param tmp_path: The pytest temp directory.
    :param name: File name to create.
    :param content: File body.
    :return: The path written.
    """
    path = tmp_path / name
    path.write_text(content, encoding="utf-8", newline="\n")
    return path


def _parse_edges_list(source: str) -> ast.List:
    """Return the ``ast.List`` node that is the top-level ``EDGES`` value.

    Parses *source* with the stdlib :mod:`ast` and finds the single-target
    ``EDGES = [...]`` assignment, returning its list node so a test can inspect
    the structure (entry count, key kinds) without LibCST.

    :param source: Python source text containing an ``EDGES`` assignment.
    :return: The ``ast.List`` for the ``EDGES`` value.
    :raises AssertionError: If no ``EDGES`` list assignment is found.
    """
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if len(node.targets) != 1:
            continue
        target = node.targets[0]
        if isinstance(target, ast.Name) and target.id == "EDGES":
            assert isinstance(node.value, ast.List), "EDGES must be a list"
            return node.value
    raise AssertionError("no top-level EDGES = [...] assignment found")


def _projection_entries(edges_list: ast.List) -> list[ast.Dict]:
    """Return the ``EDGE_PROJECTION`` dict entries from an ``EDGES`` list node.

    An entry is treated as a projection edge when its ``"type"`` value is the
    bare ``ast.Name`` ``EDGE_PROJECTION`` (NOT a ``"EDGE_PROJECTION"`` string).

    :param edges_list: The ``ast.List`` returned by :func:`_parse_edges_list`.
    :return: The dict nodes whose ``type`` is the bare ``EDGE_PROJECTION`` name.
    """
    found: list[ast.Dict] = []
    for element in edges_list.elts:
        if not isinstance(element, ast.Dict):
            continue
        for key, value in zip(element.keys, element.values):
            if (
                isinstance(key, ast.Constant)
                and key.value == "type"
                and isinstance(value, ast.Name)
                and value.id == "EDGE_PROJECTION"
            ):
                found.append(element)
                break
    return found


# ===========================================================================
# TC1 -- append_projection_edge
# ===========================================================================


def test_tc1_append_adds_one_projection_entry_with_bare_name(tmp_path) -> None:
    """A single append adds exactly one ``EDGE_PROJECTION`` entry with the four
    keys, and ``type`` is the bare ``EDGE_PROJECTION`` name (never quoted)."""
    path = _write(tmp_path, "field_data.py", _SOURCE_EMPTY_EDGES)

    result = append_projection_edge(path, "A", "B", 45.0, 90.0)

    assert result is True, "first append must report a write"

    new_text = path.read_text(encoding="utf-8")

    # The appended entry parses, has all four required keys, and its ``type`` is
    # the bare name ``EDGE_PROJECTION`` -- confirmed structurally via ``ast``.
    edges_list = _parse_edges_list(new_text)
    assert len(edges_list.elts) == 1, "exactly one EDGES entry after append"
    projections = _projection_entries(edges_list)
    assert len(projections) == 1, "the one entry must be an EDGE_PROJECTION"

    entry = projections[0]
    key_names = [
        key.value for key in entry.keys if isinstance(key, ast.Constant)
    ]
    for required in _REQUIRED_PROJECTION_KEYS:
        assert required in key_names, f"appended entry missing key {required!r}"

    # The ``type`` value is a bare ``Name`` node (EDGE_PROJECTION), not a string
    # constant -- ``field_data.py`` imports the name, so a quoted value would
    # not resolve at build time.
    for key, value in zip(entry.keys, entry.values):
        if isinstance(key, ast.Constant) and key.value == "type":
            assert isinstance(value, ast.Name), "type must be a bare Name node"
            assert value.id == "EDGE_PROJECTION"

    # Belt-and-braces on the raw text: the bare token appears, the quoted form
    # does not. (``EDGE_PROJECTION == "PROJECTION"`` -- a stray ``"PROJECTION"``
    # quoted value would be wrong too.)
    assert "EDGE_PROJECTION" in new_text
    assert '"EDGE_PROJECTION"' not in new_text
    assert f'"{EDGE_PROJECTION}"' not in new_text  # no quoted "PROJECTION"

    # The values round-tripped: distance and heading are the floats we passed.
    spec = _projection_spec(new_text, "A", "B")
    assert spec["distance_cm"] == pytest.approx(45.0)
    assert spec["heading_deg"] == pytest.approx(90.0)

    # The pre-existing comment survived the append.
    assert "# Comments here must survive every append / remove." in new_text


def test_tc1_append_same_uv_is_noop_preserving_bytes_and_mtime(
    tmp_path,
) -> None:
    """A second append of the same ``(u, v)`` returns ``False`` and leaves the
    file byte-identical with an unchanged mtime."""
    path = _write(tmp_path, "field_data.py", _SOURCE_EMPTY_EDGES)

    assert append_projection_edge(path, "A", "B", 45.0, 90.0) is True
    after_first_bytes = path.read_bytes()
    mtime_after_first = path.stat().st_mtime_ns

    # Re-append the SAME pair (even with a different geometry payload): the
    # ``(u, v)`` dedup makes it a no-op.
    result = append_projection_edge(path, "A", "B", 999.0, 12.0)

    assert result is False, "duplicate (u, v) append must be a no-op"
    assert path.read_bytes() == after_first_bytes, "bytes must be unchanged"
    assert path.stat().st_mtime_ns == mtime_after_first, "mtime must be unchanged"


# ===========================================================================
# TC2 -- remove_projection_edge + W1 regression guard
# ===========================================================================


def test_tc2_remove_drops_matching_entry(tmp_path) -> None:
    """``remove_projection_edge`` drops the matching ``(u, v)`` entry."""
    path = _write(tmp_path, "field_data.py", _SOURCE_EMPTY_EDGES)
    assert append_projection_edge(path, "A", "B", 45.0, 90.0) is True

    result = remove_projection_edge(path, "A", "B")

    assert result is True, "remove of a present (u, v) must report a write"

    edges_list = _parse_edges_list(path.read_text(encoding="utf-8"))
    assert len(edges_list.elts) == 0, "the EDGES list must be empty after remove"


def test_tc2_remove_absent_uv_is_noop(tmp_path) -> None:
    """Removing an absent ``(u, v)`` returns ``False`` and changes nothing."""
    path = _write(tmp_path, "field_data.py", _SOURCE_EMPTY_EDGES)
    assert append_projection_edge(path, "A", "B", 45.0, 90.0) is True
    bytes_before = path.read_bytes()
    mtime_before = path.stat().st_mtime_ns

    result = remove_projection_edge(path, "X", "Y")

    assert result is False, "remove of an absent (u, v) must be a no-op"
    assert path.read_bytes() == bytes_before
    assert path.stat().st_mtime_ns == mtime_before


def test_tc2_w1_append_does_not_mutate_nested_list_valued_key(
    tmp_path,
) -> None:
    """W1 guard: appending a projection edge into a file whose existing entry
    has a list-valued key (``"waypoints": [...]``) adds exactly one top-level
    dict and leaves the nested list untouched."""
    path = _write(tmp_path, "field_data.py", _SOURCE_LIST_VALUED_KEY)

    result = append_projection_edge(path, "C", "D", 30.0, 180.0)

    assert result is True, "append into a non-empty EDGES must write"

    new_text = path.read_text(encoding="utf-8")
    edges_list = _parse_edges_list(new_text)

    # Exactly ONE top-level dict was added: the list grew from 1 to 2 entries,
    # never grafting the new element into the nested ``waypoints`` list.
    assert len(edges_list.elts) == 2, "one top-level dict appended"

    # The original entry's ``waypoints`` list is byte-for-byte intact.
    assert '"waypoints": [1, 2, 3]' in new_text, "nested list must be untouched"

    # The appended entry is the projection edge we asked for.
    projections = _projection_entries(edges_list)
    assert len(projections) == 1, "the appended entry is the projection edge"
    spec = _projection_spec(new_text, "C", "D")
    assert spec["distance_cm"] == pytest.approx(30.0)
    assert spec["heading_deg"] == pytest.approx(180.0)


def test_tc2_w1_remove_of_uv_only_in_nested_list_is_noop(tmp_path) -> None:
    """W1 guard: a ``(u, v)`` that exists only inside a nested list-valued key
    is NOT matched -- the remove is a no-op."""
    path = _write(tmp_path, "field_data.py", _SOURCE_LIST_VALUED_KEY)
    bytes_before = path.read_bytes()
    mtime_before = path.stat().st_mtime_ns

    # ``(1, 2)`` only appears as elements of the nested ``waypoints`` list, never
    # as a top-level entry's ``(u, v)``. The remover keys on the top-level dict's
    # ``"u"`` / ``"v"`` values, so it must not match.
    result = remove_projection_edge(path, "1", "2")

    assert result is False, "a (u, v) only in a nested list must not match"
    assert path.read_bytes() == bytes_before
    assert path.stat().st_mtime_ns == mtime_before


# ===========================================================================
# TC3 -- projection_edges.reconcile
# ===========================================================================


def test_tc3_reconcile_add_overrides_loaded_on_same_pair() -> None:
    """An add re-stating an existing ``(u, v)`` overrides the loaded geometry."""
    loaded = [("A", "B", 10.0, 0.0)]
    adds = [("A", "B", 99.0, 180.0)]

    render = projection_edges.reconcile(loaded, adds, removes=[])

    assert render == [("A", "B", 99.0, 180.0)], "add overrides loaded geometry"


def test_tc3_reconcile_remove_suppresses_loaded_and_pending_add() -> None:
    """A ``(u, v)`` in ``removes`` is excluded whether it is loaded or a
    pending add."""
    loaded = [("A", "B", 10.0, 0.0), ("C", "D", 20.0, 90.0)]
    adds = [("E", "F", 30.0, 270.0)]
    removes = [("A", "B"), ("E", "F")]

    render = projection_edges.reconcile(loaded, adds, removes)

    # Only the non-removed loaded edge survives; the removed add is suppressed.
    assert render == [("C", "D", 20.0, 90.0)]


def test_tc3_reconcile_dedups_and_is_deterministic_order() -> None:
    """The render order is loaded-first (source order) then adds (user order),
    with duplicate ``(u, v)`` pairs collapsed to a single entry."""
    loaded = [("A", "B", 10.0, 0.0), ("C", "D", 20.0, 90.0)]
    # First add overrides ``(C, D)``; second add is a brand-new pair; third add
    # duplicates the second pair and must be dropped (earlier add wins).
    adds = [
        ("C", "D", 99.0, 45.0),
        ("E", "F", 30.0, 270.0),
        ("E", "F", 77.0, 5.0),
    ]

    render = projection_edges.reconcile(loaded, adds, removes=[])

    # ``(A, B)`` survives from loaded (source order). ``(C, D)`` is dropped from
    # the loaded slot because an add re-states it, then re-emitted in the adds
    # block with the overriding geometry. ``(E, F)`` appears once (first add).
    assert render == [
        ("A", "B", 10.0, 0.0),
        ("C", "D", 99.0, 45.0),
        ("E", "F", 30.0, 270.0),
    ]


# ===========================================================================
# TC4 -- AddProjectionEdge / DeleteProjectionEdge apply / revert / redo
# ===========================================================================


def _model_with_nodes(*names: str) -> SimpleNamespace:
    """Return a minimal stub model exposing only ``.nodes`` for the commands.

    The projection commands consult the model purely for read-only
    node-existence validation, so a ``SimpleNamespace`` with a ``nodes`` dict is
    a sufficient stand-in for a real :class:`FieldModel`.

    :param names: Node names to populate ``nodes`` with (positions are unused).
    :return: A ``SimpleNamespace`` with a ``nodes`` mapping.
    """
    return SimpleNamespace(nodes={name: (0.0, 0.0) for name in names})


def _delta_state(
    delta: ProjectionDelta,
) -> tuple[list, list, list]:
    """Snapshot the delta's three lists as independent deep copies.

    Used to assert byte-identity of ``loaded`` / ``adds`` / ``removes`` across
    an apply -> revert cycle.

    :param delta: The delta to snapshot.
    :return: ``(loaded, adds, removes)`` deep copies.
    """
    return (
        copy.deepcopy(delta.loaded),
        copy.deepcopy(delta.adds),
        copy.deepcopy(delta.removes),
    )


def test_tc4_add_appended_branch_apply_revert_redo() -> None:
    """``AddProjectionEdge`` on a fresh pair: ``"appended"`` branch -- apply
    pushes onto ``adds``, revert restores the delta byte-identically, redo
    re-appends."""
    model = _model_with_nodes("A", "B")
    delta = ProjectionDelta(loaded=[], adds=[], removes=[])
    before = _delta_state(delta)

    cmd = AddProjectionEdge(delta, "A", "B", 45.0, 90.0)
    cmd.apply(model)

    assert cmd._mode == "appended"
    assert delta.adds == [("A", "B", 45.0, 90.0)]
    assert delta.has("A", "B")

    cmd.revert(model)
    assert _delta_state(delta) == before, "revert must restore byte-identically"

    # Redo (apply again) re-applies cleanly.
    cmd.apply(model)
    assert delta.adds == [("A", "B", 45.0, 90.0)]


def test_tc4_add_uncanceled_remove_branch_apply_revert_redo() -> None:
    """``AddProjectionEdge`` re-adding a loaded edge the user had pending-deleted:
    ``"uncanceled_remove"`` branch -- apply drops the removal, revert re-cancels,
    redo re-drops."""
    model = _model_with_nodes("A", "B")
    # ``(A, B)`` is a LOADED edge currently suppressed by a pending removal.
    delta = ProjectionDelta(
        loaded=[("A", "B", 10.0, 0.0)], adds=[], removes=[("A", "B")]
    )
    before = _delta_state(delta)
    assert not delta.has("A", "B"), "removed loaded edge does not render"

    cmd = AddProjectionEdge(delta, "A", "B", 45.0, 90.0)
    cmd.apply(model)

    assert cmd._mode == "uncanceled_remove"
    assert delta.removes == [], "the removal was dropped, not a duplicate add"
    assert delta.adds == [], "no duplicate add appended"
    assert delta.has("A", "B"), "the loaded edge renders again"

    cmd.revert(model)
    assert _delta_state(delta) == before, "revert restores the pending removal"

    cmd.apply(model)
    assert delta.removes == [], "redo re-drops the removal"


def test_tc4_delete_dropped_add_branch_apply_revert_redo() -> None:
    """``DeleteProjectionEdge`` on a pending add: ``"dropped_add"`` branch --
    apply removes the add (capturing its index), revert re-inserts it at the same
    index, redo drops it again."""
    model = _model_with_nodes("A", "B", "C", "D")
    # Two pending adds; deleting the FIRST must re-insert at index 0 on revert.
    delta = ProjectionDelta(
        loaded=[],
        adds=[("A", "B", 10.0, 0.0), ("C", "D", 20.0, 90.0)],
        removes=[],
    )
    before = _delta_state(delta)

    cmd = DeleteProjectionEdge(delta, "A", "B")
    cmd.apply(model)

    assert cmd._mode == "dropped_add"
    assert cmd._dropped_index == 0
    assert delta.adds == [("C", "D", 20.0, 90.0)]
    assert not delta.has("A", "B")

    cmd.revert(model)
    assert _delta_state(delta) == before, "the dropped add returns to index 0"

    cmd.apply(model)
    assert delta.adds == [("C", "D", 20.0, 90.0)], "redo drops the add again"


def test_tc4_delete_added_remove_branch_apply_revert_redo() -> None:
    """``DeleteProjectionEdge`` on a loaded edge: ``"added_remove"`` branch --
    apply appends to ``removes``, revert pops that suppression, redo re-adds it."""
    model = _model_with_nodes("A", "B")
    delta = ProjectionDelta(
        loaded=[("A", "B", 10.0, 0.0)], adds=[], removes=[]
    )
    before = _delta_state(delta)
    assert delta.has("A", "B")

    cmd = DeleteProjectionEdge(delta, "A", "B")
    cmd.apply(model)

    assert cmd._mode == "added_remove"
    assert delta.removes == [("A", "B")]
    assert not delta.has("A", "B"), "the loaded edge is now suppressed"

    cmd.revert(model)
    assert _delta_state(delta) == before, "the suppression is removed on revert"

    cmd.apply(model)
    assert delta.removes == [("A", "B")], "redo re-suppresses the loaded edge"


# ---------------------------------------------------------------------------
# Local helper used by the TC1 / TC2 value assertions.
# ---------------------------------------------------------------------------


def _projection_spec(source: str, u: str, v: str) -> dict:
    """Return the ``{distance_cm, heading_deg}`` of the ``(u, v)`` entry.

    Parses *source* with :mod:`ast`, finds the top-level ``EDGES`` entry whose
    ``"u"`` / ``"v"`` string constants equal *u* / *v*, and returns its numeric
    payload so a test can assert the geometry round-tripped.

    :param source: Python source containing an ``EDGES`` assignment.
    :param u: Source node name to match.
    :param v: Destination node name to match.
    :return: ``{"distance_cm": float, "heading_deg": float}`` for the entry.
    :raises AssertionError: If no entry with that ``(u, v)`` is found.
    """
    edges_list = _parse_edges_list(source)
    for element in edges_list.elts:
        if not isinstance(element, ast.Dict):
            continue
        pairs = {
            key.value: value
            for key, value in zip(element.keys, element.values)
            if isinstance(key, ast.Constant)
        }
        u_node = pairs.get("u")
        v_node = pairs.get("v")
        if (
            isinstance(u_node, ast.Constant)
            and u_node.value == u
            and isinstance(v_node, ast.Constant)
            and v_node.value == v
        ):
            return {
                "distance_cm": ast.literal_eval(pairs["distance_cm"]),
                "heading_deg": ast.literal_eval(pairs["heading_deg"]),
            }
    raise AssertionError(f"no EDGES entry with u={u!r}, v={v!r}")
