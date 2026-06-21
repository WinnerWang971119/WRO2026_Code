"""Tests for ``EditorApp`` pending-rename delta bookkeeping.

The editor flushes node renames into ``field_data.py``'s ``EDGES`` table on
Save so the Python source tracks the JSON sidecar. The bookkeeping that decides
*which* rename to flush is maintained as a delta in ``_pending_renames``: a
fresh rename records forward, an undo records the inverse, a redo re-records
forward. These tests pin that delta against the bug it was rewritten to fix --
undoing an already-saved rename (optionally followed by an unrelated edit that
clears the redo stack) must revert ``EDGES`` on the next Save, not leave
``field_data.py`` and ``field_data.json`` diverged.

``EditorApp`` normally builds a Tk window in ``__init__``; Tk under pytest is
flaky (repeated/`matplotlib`-adjacent ``Tk()`` calls raise ``TclError``) and
absent on headless CI, so it would only skip. Instead we build a bare instance
and inject the non-Tk collaborators the rename methods actually touch
(``_history``, ``_model``, ``_pending_renames``), stubbing the view/status
helpers. EDGES rewrites are captured rather than written to the real
``field_data.py``.
"""

from __future__ import annotations

import shutil
import types
from pathlib import Path

import matplotlib
import pytest

# Importing EditorApp pulls in editor.canvas, which calls
# ``matplotlib.use("TkAgg")`` at module-import time. On headless CI (no display)
# that raises and fails collection; locally it would also leave a Tk backend
# active and break test_visualizer's PNG dumps. These tests build a bare
# EditorApp and never construct the canvas, so pin the headless-safe Agg backend
# and no-op that one switch for the duration of the import.
matplotlib.use("Agg")
_real_mpl_use = matplotlib.use
matplotlib.use = lambda *args, **kwargs: None
try:
    from tools.path_planner.editor.app import EditorApp  # noqa: E402
finally:
    matplotlib.use = _real_mpl_use

from tools.path_planner.editor import ast_rewrite as ast_rewrite_mod  # noqa: E402
from tools.path_planner.editor import loader  # noqa: E402
from tools.path_planner.editor.commands import (  # noqa: E402
    RenameNode,
    SetNodeType,
)
from tools.path_planner.editor.history import History  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402

_FIXTURE = Path(__file__).parent / "fixtures" / "field_data_small.json"


@pytest.fixture
def app(tmp_path, monkeypatch):
    """A bare ``EditorApp`` wired with just the rename collaborators.

    No Tk: the view/status/dirty helpers are stubbed and only ``_history``,
    ``_model`` and ``_pending_renames`` are real. EDGES rewrites land in
    ``instance.captured_rewrites`` instead of touching ``field_data.py``.
    """
    json_path = tmp_path / "field_data.json"
    shutil.copyfile(_FIXTURE, json_path)
    model = FieldModel.from_json_dict(loader.load_field_json(json_path))

    captured: list[dict] = []
    monkeypatch.setattr(
        ast_rewrite_mod,
        "rewrite_edges_uv",
        lambda path, mapping: captured.append(dict(mapping)),
    )

    instance = EditorApp.__new__(EditorApp)
    instance._history = History()
    instance._model = model
    instance._pending_renames = {}
    instance._json_path = json_path
    instance._field_data_module = None  # skip the post-rewrite importlib.reload
    # Stub the Tk-touching helpers the rename methods call.
    instance._refresh_view = lambda: None
    instance._set_status = lambda *a, **k: None
    instance._mark_dirty = lambda: None
    instance._clear_dirty = lambda: None
    # _save drives the validation badge on the SidePanels surface (T7 / AC8);
    # this bare instance has no Tk panel, so stub just the one setter it calls.
    instance._panels = types.SimpleNamespace(
        set_validation_badge=lambda *a, **k: None
    )
    instance.captured_rewrites = captured
    return instance


def _rename(app: EditorApp, old: str, new: str) -> None:
    """Drive a rename exactly as ``_context_rename`` does, minus the dialog."""
    assert app._dispatch(RenameNode(old, new))
    app._record_pending_rename(old, new)


def test_undo_saved_rename_then_save_reverts_edges(app):
    """Documented repro: rename, Save, undo, Save -> EDGES revert to old name."""
    _rename(app, "A", "AA")
    assert app._pending_renames == {"A": "AA"}

    assert app._save() is True
    assert app.captured_rewrites == [{"A": "AA"}]
    assert app._pending_renames == {}

    app._undo()
    assert "A" in app._model.nodes and "AA" not in app._model.nodes
    # The saved rename was undone: the next Save must rewrite EDGES AA -> A.
    assert app._pending_renames == {"AA": "A"}

    assert app._save() is True
    assert app.captured_rewrites == [{"A": "AA"}, {"AA": "A"}]
    assert app._pending_renames == {}


def test_undo_saved_rename_then_unrelated_edit_then_save(app):
    """Redo-cleared variant a history-stack-walk fix would miss.

    After undoing a saved rename, any new edit clears the redo stack and
    evicts the rename command from history entirely. The delta must survive.
    """
    _rename(app, "A", "AA")
    assert app._save() is True

    app._undo()
    assert app._pending_renames == {"AA": "A"}

    # An unrelated edit clears the redo stack (History.do discards the redo
    # branch), so the RenameNode is now on neither stack. Retype the blind node
    # G to "line" -- a benign edit that keeps the field valid. (Typing the
    # line-incident node A to "blind" would now fail the line-into-blind save
    # check, ADR-041, aborting the save this test asserts succeeds.)
    assert app._dispatch(SetNodeType("G", "line"))
    assert not app._history.can_redo
    assert app._pending_renames == {"AA": "A"}  # delta is not stack-derived

    assert app._save() is True
    assert app.captured_rewrites[-1] == {"AA": "A"}
    assert app._pending_renames == {}


def test_rename_then_undo_before_save_flushes_nothing(app):
    """A rename undone before any Save collapses to no pending rewrite."""
    _rename(app, "A", "AA")
    app._undo()
    assert app._pending_renames == {}

    assert app._save() is True
    assert app.captured_rewrites == []  # EDGES untouched


def test_undo_then_redo_restores_pending(app):
    """Undo then redo of a rename leaves the delta forward again (symmetry)."""
    _rename(app, "A", "AA")
    app._undo()
    assert app._pending_renames == {}

    app._redo()
    assert "AA" in app._model.nodes
    assert app._pending_renames == {"A": "AA"}


def test_rename_chain_collapses_on_save(app):
    """A -> AA -> AAA flushes a single A -> AAA rewrite."""
    _rename(app, "A", "AA")
    _rename(app, "AA", "AAA")
    assert app._pending_renames == {"A": "AAA"}

    assert app._save() is True
    assert app.captured_rewrites == [{"A": "AAA"}]
