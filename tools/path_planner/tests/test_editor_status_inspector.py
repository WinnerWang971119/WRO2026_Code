"""Tests for the per-move validation status bar and the inspector's in-place
rename / Delete controls (Issue #78 follow-up).

Two features land together here:

* **Per-move validation** -- the app re-runs the full validator after every
  model mutation (``EditorApp._revalidate``, hooked into ``_refresh_view``) and
  drives both the badge and the message slot via ``SidePanels.set_validation``.
  The message slot shows the validator's first-error detail in the INVALID tone
  while invalid and snaps back to the normal tone (clearing a stale complaint)
  once the model is valid again.
* **Inspector rename / delete** -- the property panel's Name field is editable
  (commit fires ``on_node_rename(old, new)``) and a Delete button removes the
  selected node (``on_node_delete(name)``), reusing the same app logic as the
  right-click menu.

The panel-widget tests build the REAL ``SidePanels`` on the shared session Tk
root (``tk_root`` from ``conftest.py``); the app-logic tests build a bare
``EditorApp`` with injected collaborators so no Tk window is needed.  The
matplotlib import dance mirrors the sibling editor tests: pin Agg and swallow
``editor.canvas``'s import-time ``use("TkAgg")``.
"""

from __future__ import annotations

from types import SimpleNamespace

import matplotlib

matplotlib.use("Agg")
_real_mpl_use = matplotlib.use
matplotlib.use = lambda *args, **kwargs: None
try:
    from tools.path_planner.editor.app import EditorApp  # noqa: E402
    from tools.path_planner.editor import canvas as _canvas_mod  # noqa: F401,E402
finally:
    matplotlib.use = _real_mpl_use

import pytest  # noqa: E402

from tools.path_planner.editor.commands import RenameNode  # noqa: E402
from tools.path_planner.editor.history import History  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402
from tools.path_planner.editor.panels import (  # noqa: E402
    _BADGE_INVALID_TEXT,
    _BADGE_VALID_TEXT,
    _DEFAULT_STATUS_TEXT,
    SidePanels,
)


def _make_model(**override) -> FieldModel:
    """A small, fully-valid in-memory model (``A`` -- ``B`` line, ``B`` -- ``G``
    blind, ``S`` a blind dead-reckon spawn wired to ``G``); ``override`` lets a
    test break exactly one field to force INVALID.

    ``S`` is the ``START_NODE`` because the issue #64 validator rejects a
    ``"line"`` / ``"projection"`` start (an on-line spawn presumes a
    sensor-anchored arrival the chassis has not made at t=0), so the start must
    be a dead-reckon ``"blind"`` origin for the model to validate clean.
    """
    base = dict(
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
            "S": (250.0, 150.0),
            "A": (0.0, 50.0),
            "B": (200.0, 10.0),
            "G": (250.0, 80.0),
        },
        node_types={"S": "blind", "A": "line", "B": "line", "G": "blind"},
        line_segments=[("A", "B")],
        blind_edges=[("B", "G"), ("S", "G")],
        obstacles=[],
    )
    base.update(override)
    return FieldModel(**base)


@pytest.fixture
def panels(tk_root):
    """A fresh REAL :class:`SidePanels` on the shared session Tk root."""
    return SidePanels(tk_root)


# ---------------------------------------------------------------------------
# SidePanels.set_validation -- badge + message slot together
# ---------------------------------------------------------------------------


def test_set_validation_invalid_paints_detail(panels):
    """An invalid verdict shows the detail in the message slot + sets the flag."""
    panels.set_validation(False, "pivot angle 4 < min 15")

    assert panels._badge_var.get() == _BADGE_INVALID_TEXT
    assert panels._status_var.get() == "pivot angle 4 < min 15"
    assert panels._status_showing_error is True


def test_set_validation_valid_clears_stale_error(panels):
    """Going valid after an error clears the stale complaint to the default."""
    panels.set_validation(False, "boom")
    panels.set_validation(True)

    assert panels._badge_var.get() == _BADGE_VALID_TEXT
    assert panels._status_var.get() == _DEFAULT_STATUS_TEXT
    assert panels._status_showing_error is False


def test_transient_status_survives_valid_revalidate(panels):
    """A transient note set by the same edit is NOT wiped by a VALID verdict."""
    panels.set_status("Node moved to (1, 2)")
    assert panels._status_showing_error is False

    panels.set_validation(True)
    assert panels._status_var.get() == "Node moved to (1, 2)"
    assert panels._status_showing_error is False


# ---------------------------------------------------------------------------
# EditorApp._revalidate -- drives set_validation from a real validate() call
# ---------------------------------------------------------------------------


class _ValidationSpy:
    """Records ``set_validation`` calls so a bare app can be asserted Tk-free."""

    def __init__(self) -> None:
        self.calls: list[tuple[bool, str]] = []

    def set_validation(self, ok: bool, detail: str = "") -> None:
        self.calls.append((ok, detail))


def _bare_app(model: FieldModel) -> tuple[EditorApp, _ValidationSpy]:
    app = EditorApp.__new__(EditorApp)
    app._model = model
    spy = _ValidationSpy()
    app._panels = spy
    return app, spy


def test_revalidate_valid_model_reports_ok():
    app, spy = _bare_app(_make_model())
    app._revalidate()
    assert spy.calls[-1] == (True, "")


def test_revalidate_invalid_model_reports_detail():
    # start_node pointing at a missing node is a clean FieldDataError.
    app, spy = _bare_app(_make_model(start_node="ZZ"))
    app._revalidate()
    ok, detail = spy.calls[-1]
    assert ok is False
    assert "ZZ" in detail


# ---------------------------------------------------------------------------
# Inspector name-entry rename + Delete button (panel layer)
# ---------------------------------------------------------------------------


def test_inspector_name_commit_fires_rename(panels):
    fired: list[tuple[str, str]] = []
    panels.on_node_rename = lambda old, new: fired.append((old, new))
    panels._selected_node_var.set("A")
    panels._name_edit_var.set("B")

    panels._handle_name_commit()

    assert fired == [("A", "B")]


def test_inspector_name_commit_empty_or_unchanged_is_noop(panels):
    fired: list[tuple[str, str]] = []
    panels.on_node_rename = lambda old, new: fired.append((old, new))
    panels._selected_node_var.set("A")

    # Whitespace-only: no rename, entry snaps back to the canonical name.
    panels._name_edit_var.set("   ")
    panels._handle_name_commit()
    assert fired == []
    assert panels._name_edit_var.get() == "A"

    # Identical name: no rename.
    panels._name_edit_var.set("A")
    panels._handle_name_commit()
    assert fired == []


def test_inspector_delete_button_fires_delete(panels):
    fired: list[str] = []
    panels.on_node_delete = lambda node: fired.append(node)
    panels._selected_node_var.set("A")

    panels._handle_node_delete()
    assert fired == ["A"]

    # No selection -> no fire.
    panels._selected_node_var.set("")
    panels._handle_node_delete()
    assert fired == ["A"]


def test_inspector_rename_delete_suppressed_during_refresh(panels):
    """Both handlers no-op while ``_suppress_callbacks`` is set (refresh replay)."""
    fired: list[object] = []
    panels.on_node_rename = lambda *a: fired.append(a)
    panels.on_node_delete = lambda *a: fired.append(a)
    panels._selected_node_var.set("A")
    panels._name_edit_var.set("B")

    panels._suppress_callbacks = True
    panels._handle_name_commit()
    panels._handle_node_delete()

    assert fired == []


# ---------------------------------------------------------------------------
# EditorApp._perform_rename -- shared rename core (app layer, Tk-free)
# ---------------------------------------------------------------------------


def _rename_app(model: FieldModel) -> EditorApp:
    panels = SimpleNamespace(
        _selected_node_var=SimpleNamespace(set=lambda *a, **k: None),
        set_validation=lambda *a, **k: None,
        set_status=lambda *a, **k: None,
        set_selection_display=lambda *a, **k: None,
    )
    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = History()
    app._panels = panels
    app._selected_node = "A"
    app._mark_dirty = lambda: None
    app._refresh_view = lambda: None
    app._set_status = lambda *a, **k: None
    app._update_selection_display = lambda *a, **k: None
    return app


def test_perform_rename_dispatches_and_follows_selection():
    app = _rename_app(_make_model())

    assert app._perform_rename("A", "AA") is True
    # The rename cascades through the model (no separate pending-rename delta --
    # that subsystem is retired in issue #79; EDGES cascade is covered by TC21).
    assert "AA" in app._model.nodes and "A" not in app._model.nodes
    # Selection follows the renamed node.
    assert app._selected_node == "AA"


def test_perform_rename_blank_or_unchanged_returns_false():
    app = _rename_app(_make_model())

    assert app._perform_rename("A", "A") is False
    assert app._perform_rename("A", "   ") is False
    # Model untouched by either no-op.
    assert "A" in app._model.nodes
