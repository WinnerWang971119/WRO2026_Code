"""T7 GUI rule-guard + FIELD_MM tests (PR #98 triage: U2, U5, R6, R7).

Headless GUI tests for the four T7 acceptance criteria, mirroring the validator
rules T1 landed so the GUI and the Save-to-disk validator never disagree:

* **TC13** (U5) -- edge-mode ``L`` on a reset-incident pair is REFUSED with a
  status message and no dispatch; a control ``L`` on a PROJECTION-incident pair
  is ALLOWED (LINE->projection stays legal, ADR-046).
* **TC15** (U2, superseded by ADR-064) -- ``BlindMode.STOP_AT_LINE`` is folded
  onto the ``stop_at_line`` axis and is no longer an authorable per-edge
  ``mode`` tag at all, so the old contradiction (``primitive_mode=STOP_AT_LINE``
  + ``stop_at_line='off'``) is now inexpressible through the GUI: the Mode
  picker never offers ``STOP_AT_LINE``, and the save-mirror check that used to
  reject the pairing was dropped along with it.
* **TC16** (R6) -- editing one FIELD_MM dimension while the model holds a sub-mm
  OTHER dimension leaves the untouched dimension's stored model value unchanged
  after commit (no truncation through the rounded display var).
* **TC17** (R7) -- committing an UNCHANGED FIELD_MM fires no command (no undo
  entry, no dirty flag).

Headless pattern (mirrors ``test_editor_override_interaction.py``): pin the
``Agg`` matplotlib backend and no-op the one ``matplotlib.use("TkAgg")``
``editor.canvas`` performs at import time, build a REAL :class:`SidePanels` on the
SESSION-SCOPED shared ``tk_root`` from ``conftest.py``, and drive a bare
``EditorApp.__new__`` instance whose only Tk collaborator is that real panel plus
a tiny stub canvas -- no real editor window is constructed.
"""

from __future__ import annotations

import matplotlib

# Pin Agg before importing the editor, then swallow the one
# ``matplotlib.use("TkAgg")`` ``editor.canvas`` runs at import (it would raise on
# headless CI and clobber the Agg backend the shared root relies on).
matplotlib.use("Agg")
_real_mpl_use = matplotlib.use
matplotlib.use = lambda *args, **kwargs: None
try:
    from tools.path_planner.editor.app import EditorApp  # noqa: E402
    from tools.path_planner.editor import canvas as _canvas_mod  # noqa: F401,E402
finally:
    matplotlib.use = _real_mpl_use

import pytest  # noqa: E402

from tools.path_planner.editor.history import History  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402
from tools.path_planner.editor.panels import (  # noqa: E402
    Mode,
    SidePanels,
)


# ---------------------------------------------------------------------------
# Headless harness
# ---------------------------------------------------------------------------


class _StubCanvas:
    """Minimal canvas stand-in for the app's edge-click + refresh path.

    ``find_node_at`` pops the next queued node (the click target the test set
    up); ``render`` is a no-op (no real figure under the headless Agg backend).
    """

    def __init__(self) -> None:
        self._queue: list[str | None] = []

    def queue_hit(self, name: str | None) -> None:
        """Queue *name* as the node the next ``find_node_at`` returns."""
        self._queue.append(name)

    def find_node_at(self, _x: float, _y: float) -> str | None:
        """Return the next queued click target (``None`` when exhausted)."""
        return self._queue.pop(0) if self._queue else None

    def render(self, *_args, **_kwargs) -> None:
        """No-op redraw under the headless backend."""


def _make_model(
    *,
    field_mm: tuple[float, float] = (3000.0, 2000.0),
    edges: list[dict] | None = None,
) -> FieldModel:
    """Return a small :class:`FieldModel` with line / blind / reset / projection.

    ``A`` -- ``B`` is a LINE segment and ``B`` -- ``G`` a BLIND edge, so the
    edge-mode flow can derive both a LINE and a BLIND type.  ``R`` is a reset
    node and ``P`` a projection node, both isolated, so the U5 LINE-edge guard
    can be exercised on a reset-incident pair (refused) and a projection-incident
    pair (allowed).  The model need not be geometry-valid: the L-key guard fires
    BEFORE any dispatch / validation, and the app's per-move ``_revalidate`` is
    stubbed out in these harnesses.

    :param field_mm: Outer field dimensions ``(width_mm, height_mm)``.  Carry a
        sub-mm value to exercise the FIELD_MM precision guard.
    :param edges: Optional initial override overlay entries.
    :return: A model ready for a headless panel refresh.
    """
    return FieldModel(
        field_mm=field_mm,
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="A",
        start_heading_deg=0.0,
        nodes={
            "A": (0.0, 50.0),
            "B": (200.0, 10.0),
            "G": (250.0, 80.0),
            "R": (40.0, 40.0),
            "P": (60.0, 60.0),
        },
        node_types={
            "A": "line",
            "B": "line",
            "G": "blind",
            "R": "reset",
            "P": "projection",
        },
        line_segments=[("A", "B")],
        blind_edges=[("B", "G")],
        obstacles=[],
        reset_nodes={"R": {"axis": "x", "wall_coord": 0.0, "facing": 0.0}},
        edges=edges if edges is not None else [],
    )


def _bare_app(model: FieldModel, panels: SidePanels, canvas: _StubCanvas):
    """Return a bare ``EditorApp`` wired to *model* / *panels* / *canvas*.

    Built via ``__new__`` (its ``__init__`` would construct a full Tk window),
    then the load-bearing collaborators are injected: the model, a real history,
    the real panel, and the stub canvas.  ``_refresh_view`` / ``_update_title``
    are stubbed (they would need the real ``_root`` window + a full canvas
    render), and ``_mark_dirty`` is a REAL bookkeeper so the dirty flag tracks
    dispatch.  The dirty flag starts clean and the edge-selection empty.

    :param model: The in-memory field model the app mutates.
    :param panels: The real side panel (for the genuine widget state).
    :param canvas: The stub canvas feeding the edge-click hits.
    :return: The bare ``EditorApp`` instance.
    """
    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = History()
    app._panels = panels
    app._canvas = canvas
    app._edge_selection = []
    app._proposed_edges = None
    app._selected_node = None
    app._dirty = False
    app._update_title = lambda: None
    app._refresh_view = lambda: None

    def _mark_dirty() -> None:
        app._dirty = True

    app._mark_dirty = _mark_dirty
    return app


# ---------------------------------------------------------------------------
# TC13 -- edge-mode L on a reset-incident pair is refused (U5)
# ---------------------------------------------------------------------------


@pytest.fixture
def edge_app(tk_root):
    """A bare app + real panel + stub canvas in EDGE mode.

    Yields ``(app, panels, model, canvas)``; the panel is torn down after the
    test while the shared session root survives.
    """
    model = _make_model()
    panels = SidePanels(tk_root)
    canvas = _StubCanvas()
    app = _bare_app(model, panels, canvas)
    panels.mode_var.set(Mode.EDGE.value)
    panels.refresh(model)
    try:
        yield app, panels, model, canvas
    finally:
        try:
            panels.destroy()
        except Exception:  # noqa: BLE001 -- teardown best-effort
            pass


def _select_edge_pair(app, a: str, b: str) -> None:
    """Latch the two-node EDGE-mode selection to the directed pair ``a -> b``."""
    app._edge_selection = [a, b]


def test_line_key_on_reset_incident_pair_is_refused(edge_app):
    """TC13 (U5 refuse): pressing ``L`` on a pair incident to a reset node is
    refused -- a status message names the reset node, NO line edge is created,
    and no command is dispatched.

    A reset node is reached by dead reckoning (BLIND-incident only); the GUI
    guard mirrors the validator's LINE->reset rejection so the operator must use
    ``B`` instead.
    """
    app, _panels, model, _canvas = edge_app
    before = list(model.line_segments)

    # Select the A (line) -> R (reset) pair, then press L.
    _select_edge_pair(app, "A", "R")
    app._handle_edge_line_key()

    # No LINE edge was added, no command was dispatched (history empty), and the
    # status names the refused reset node.
    assert model.line_segments == before, "no LINE edge may touch a reset node"
    assert not app._history.can_undo, "the refused L must dispatch nothing"
    assert app._dirty is False, "a refused L must not dirty the buffer"
    status = app._panels._status_var.get()
    assert "reset" in status.lower(), "status must name the reset refusal"
    assert "'R'" in status or "R" in status, "status must name the reset node"


def test_line_key_on_projection_incident_pair_is_allowed(edge_app):
    """TC13 (control): pressing ``L`` on a pair incident to a PROJECTION node is
    ALLOWED -- the LINE edge is created (LINE->projection stays legal, ADR-046).

    The control proves the reset guard is specific to reset nodes and does not
    over-reach onto projection endpoints.
    """
    app, _panels, model, _canvas = edge_app
    assert ("A", "P") not in model.line_segments

    # Select the A (line) -> P (projection) pair, then press L.
    _select_edge_pair(app, "A", "P")
    app._handle_edge_line_key()

    # The LINE edge WAS created and a command WAS dispatched.
    created = ("A", "P") in model.line_segments or ("P", "A") in model.line_segments
    assert created, "a LINE edge into a projection node is legal and must be added"
    assert app._history.can_undo, "the allowed L must dispatch a command"


# ---------------------------------------------------------------------------
# TC15 (superseded by ADR-064) -- STOP_AT_LINE folded onto the stop_at_line
# axis; the old U2 save-mirror contradiction is now inexpressible through the
# GUI.
# ---------------------------------------------------------------------------


@pytest.fixture
def save_app(tk_root):
    """A bare app + real panel with a recording override-save spy.

    The override-save slot records its calls instead of dispatching, so a test
    can prove the inline editor did or did not fire ``on_set_edge_override``.

    Yields ``(app, panels, model, save_calls)``.
    """
    model = _make_model()
    panels = SidePanels(tk_root)
    canvas = _StubCanvas()
    app = _bare_app(model, panels, canvas)

    save_calls: list[tuple] = []
    panels.on_set_edge_override = lambda *args: save_calls.append(args)

    panels.refresh(model)
    try:
        yield app, panels, model, save_calls
    finally:
        try:
            panels.destroy()
        except Exception:  # noqa: BLE001 -- teardown best-effort
            pass


def _editor_is_open(panels: SidePanels) -> bool:
    """Return ``True`` when the inline override editor sub-frame is mapped."""
    return bool(panels._edge_override_editor.winfo_manager())


def test_blind_mode_picker_excludes_stop_at_line(save_app):
    """TC15 (superseded by ADR-064): the BLIND Mode combobox never offers
    ``STOP_AT_LINE``.

    ADR-064 folds ``BlindMode.STOP_AT_LINE`` onto the ``stop_at_line`` axis: a
    seek is authored via ``stop_at_line: on``, never via the Mode picker, so
    the old contradiction (``STOP_AT_LINE`` + ``stop_at_line='off'``) is now
    inexpressible through the GUI -- the picker cannot even offer the tag the
    old save-mirror check used to reject.
    """
    _app, panels, _model, _save_calls = save_app

    panels.prefill_add_row("BLIND", "B", "G")
    values = tuple(panels._edge_override_mode_combo.cget("values"))

    assert "STOP_AT_LINE" not in values


def test_inline_save_stop_at_line_via_axis_dispatches(save_app):
    """TC15 (control): authoring ``stop_at_line='on'`` alone (no Mode override)
    dispatches normally and closes the editor.

    Proves the seek is still authorable -- just on its own ``stop_at_line``
    axis, never as a ``mode`` tag.
    """
    _app, panels, _model, save_calls = save_app

    panels.prefill_add_row("BLIND", "B", "G")
    panels._edge_override_stop_at_line_var.set("on")

    panels._override_save_button.invoke()

    assert len(save_calls) == 1, "stop_at_line='on' alone must dispatch"
    edge_type, u, v, speed, mode, _direction, stop = save_calls[0]
    assert (edge_type, u, v) == ("BLIND", "B", "G")
    assert speed is None
    assert mode is None
    assert stop == "on"
    assert not _editor_is_open(panels), "a successful Save closes the editor"


# ---------------------------------------------------------------------------
# TC16 -- editing one FIELD_MM dim preserves the other's sub-mm value (R6)
# ---------------------------------------------------------------------------


@pytest.fixture
def field_mm_app(tk_root):
    """A bare app + real panel wired to the REAL FIELD_MM commit handler.

    The model holds a sub-mm HEIGHT (2000.5 mm) so the precision guard is
    exercised.  ``on_field_dimensions_change`` is wired to the app's real handler
    so a panel commit runs the genuine no-op guard + ``SetFieldDimensions``
    dispatch path.

    Yields ``(app, panels, model)``.
    """
    model = _make_model(field_mm=(3000.0, 2000.5))
    panels = SidePanels(tk_root)
    canvas = _StubCanvas()
    app = _bare_app(model, panels, canvas)
    panels.on_field_dimensions_change = app._on_panel_field_dimensions_change
    panels.refresh(model)
    try:
        yield app, panels, model
    finally:
        try:
            panels.destroy()
        except Exception:  # noqa: BLE001 -- teardown best-effort
            pass


def test_editing_width_preserves_submm_height(field_mm_app):
    """TC16 (R6): editing only the WIDTH commits the new width and leaves the
    untouched HEIGHT at its exact stored sub-mm value (2000.5 mm), not the
    rounded integer the entry displays.

    The height entry shows ``"2000"`` (``%d`` of round(2000.5)); without the
    carry-forward the commit would dispatch ``(new_width, 2000.0)`` and truncate
    the stored 2000.5.  The panel must instead take the un-edited height from the
    cached ``model.field_mm``.
    """
    app, panels, model = field_mm_app

    # Edit ONLY the width entry to a genuinely new value; leave height as shown.
    panels._field_width_mm_var.set("3500")
    panels._handle_field_dimensions_commit()

    # Width changed; height keeps its exact sub-mm value (not truncated to 2000).
    assert model.field_mm == (3500.0, 2000.5), (
        "editing width must not truncate the untouched sub-mm height"
    )
    assert app._history.can_undo, "a real width edit must dispatch a command"


def test_editing_height_preserves_width(field_mm_app):
    """TC16 (R6, mirror): editing only the HEIGHT leaves the untouched WIDTH at
    its stored value.  Width here is an integer (3000.0), so the guard simply
    proves the symmetric carry-forward."""
    app, panels, model = field_mm_app

    panels._field_height_mm_var.set("2500")
    panels._handle_field_dimensions_commit()

    assert model.field_mm == (3000.0, 2500.0), (
        "editing height must not alter the untouched width"
    )
    assert app._history.can_undo


# ---------------------------------------------------------------------------
# TC17 -- committing an unchanged FIELD_MM fires no command (R7)
# ---------------------------------------------------------------------------


def test_unchanged_field_mm_commit_dispatches_nothing(field_mm_app):
    """TC17 (R7): a bare ``<FocusOut>`` on an UNCHANGED FIELD_MM (no edit) fires
    no command -- no undo entry, no dirty flag.

    Both entries still display the model's rounded values, so the panel carries
    the exact ``model.field_mm`` forward and the app's no-op guard skips the
    dispatch entirely.
    """
    app, panels, model = field_mm_app
    before_field_mm = model.field_mm

    # Commit without touching either entry (the model-seeded display values).
    panels._handle_field_dimensions_commit()

    assert model.field_mm == before_field_mm, "an unchanged commit must not mutate"
    assert not app._history.can_undo, "an unchanged commit must dispatch nothing"
    assert app._dirty is False, "an unchanged commit must not dirty the buffer"
