"""App-level tests for the EDGE-mode projection delete / override handlers (F2).

Covers the real ``EditorApp`` keyboard handlers that mutate projection edges in
EDGE mode -- ``_handle_delete_key`` and the ``L`` / ``B`` / ``P`` override
handlers (``_handle_edge_line_key`` / ``_handle_edge_blind_key`` /
``_handle_edge_projection_key``) -- exercised through a bare ``EditorApp.__new__``
instance with the non-Tk collaborators injected.  This is the same headless
pattern as ``test_editor_app_renames.py`` / ``test_editor_inspector.py`` /
``test_editor_reset_autofill.py`` (TC6): a REAL ``History`` and ``FieldModel``
so a dispatch is genuinely observable, a REAL ``ProjectionDelta`` so the
projection commands run end-to-end, a ``_PanelsSpy`` for the panel surface, and
``_refresh_view`` / ``_mark_dirty`` / ``_sync_projection_render`` stubbed away
from their Tk / canvas dependencies.  No real Tk window is constructed.

The load-bearing contract under test (the F2 review's central concern) is the
**stored-orientation** projection delete: a projection edge is DIRECTED and may
be stored ``(u, v)`` while the operator clicks the endpoints in the reverse
order ``[v, u]``.  ``_find_projection_on_pair`` returns the STORED directed pair,
so the dispatched :class:`DeleteProjectionEdge` must target the stored ``(u, v)``
-- never the click order -- or the delete silently misses and a stale edge
survives.

``EditorApp`` builds a Tk window in ``__init__``; Tk is flaky / absent under
pytest, so we never call it.  Importing ``EditorApp`` pulls in ``editor.canvas``,
which calls ``matplotlib.use("TkAgg")`` at import time -- on headless CI that
raises.  Pin the Agg backend and no-op that one switch for the duration of the
import (mirrors the sibling app-level test modules).
"""

from __future__ import annotations

import matplotlib
import pytest

# Pin the headless-safe Agg backend before importing the editor, then no-op the
# one ``matplotlib.use("TkAgg")`` switch that ``editor.canvas`` performs at
# import time (it would raise on headless CI and clobber the Agg backend the
# other tests rely on).  Restore the real ``use`` immediately afterwards.
matplotlib.use("Agg")
_real_mpl_use = matplotlib.use
matplotlib.use = lambda *args, **kwargs: None
try:
    from tools.path_planner.editor.app import EditorApp  # noqa: E402
finally:
    matplotlib.use = _real_mpl_use

from tools.path_planner.editor import projection_edges  # noqa: E402
from tools.path_planner.editor.commands import (  # noqa: E402
    AddProjectionEdge,
    DeleteProjectionEdge,
    ProjectionDelta,
)
from tools.path_planner.editor.history import History  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402
from tools.path_planner.editor.panels import Mode  # noqa: E402

# A 300 cm x 200 cm field, millimetres (the ``FieldModel.field_mm`` convention).
_FIELD_MM: tuple[float, float] = (3000.0, 2000.0)

# Node positions used by every model below.  Distinct, non-coincident points so
# ``compute_projection_geometry`` always yields a defined heading.
_NODE_A_XY: tuple[float, float] = (0.0, 50.0)
_NODE_B_XY: tuple[float, float] = (200.0, 10.0)
_NODE_C_XY: tuple[float, float] = (250.0, 80.0)


# ---------------------------------------------------------------------------
# Headless harness
# ---------------------------------------------------------------------------


class _PanelsSpy:
    """Minimal stand-in for ``SidePanels`` recording the handler's side effects.

    The EDGE-mode handlers only touch ``mode_var.get()`` (read through
    ``_current_mode``), ``set_status`` (via ``_set_status``), and ``refresh``
    (via the stubbed ``_refresh_view``).  ``mode_var`` is a tiny duck-typed
    holder so no Tk is needed; each ``set_status`` / ``refresh`` call is recorded
    so a test can assert the handler's effects without a real Tk panel.
    """

    class _ModeVar:
        """A no-Tk stand-in for the panel's ``mode_var`` StringVar."""

        def __init__(self, value: str) -> None:
            self._value = value

        def get(self) -> str:
            return self._value

        def set(self, value: str) -> None:
            self._value = value

    def __init__(self, mode: Mode) -> None:
        self.mode_var = _PanelsSpy._ModeVar(mode.value)
        self.status_messages: list[str] = []
        self.refresh_count: int = 0

    def set_status(self, text: str) -> None:
        self.status_messages.append(text)

    def refresh(self, model) -> None:
        self.refresh_count += 1


def _make_model() -> FieldModel:
    """Return a small, fully-valid three-node :class:`FieldModel`.

    Nodes ``A`` / ``B`` / ``C`` all exist so the projection / line / blind
    commands' node-existence validation passes for any pair.  No plain edges are
    seeded by default -- each test adds exactly the edge it needs.
    """
    return FieldModel(
        field_mm=_FIELD_MM,
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="A",
        start_heading_deg=0.0,
        nodes={"A": _NODE_A_XY, "B": _NODE_B_XY, "C": _NODE_C_XY},
        node_types={"A": "line", "B": "line", "C": "line"},
        line_segments=[],
        blind_edges=[],
        obstacles=[],
    )


def _make_app(
    *,
    mode: Mode = Mode.EDGE,
    selection: list[str] | None = None,
) -> tuple[EditorApp, _PanelsSpy, FieldModel, ProjectionDelta]:
    """Return a bare ``EditorApp`` wired for headless EDGE-handler testing.

    Real ``_history`` / ``_model`` / ``_proj_delta`` so every dispatch is
    genuinely observable; a ``_PanelsSpy`` for ``_panels`` (carrying a no-Tk
    ``mode_var`` so ``_current_mode`` resolves).  ``_refresh_view`` is stubbed
    to drive only the panel refresh (the real one also touches the canvas +
    undo/redo enables the spy does not provide), ``_mark_dirty`` is a no-op, and
    ``_sync_projection_render`` is a no-op (the real one needs a live canvas).

    :param mode: The active editor mode the spy reports.
    :param selection: The EDGE-mode two-node pick order; copied onto
        ``_edge_selection``.  ``None`` -> empty selection.
    :return: ``(app, panels_spy, model, proj_delta)``.
    """
    model = _make_model()
    panels = _PanelsSpy(mode)
    proj_delta = ProjectionDelta()

    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = History()
    app._panels = panels
    app._proj_delta = proj_delta
    app._edge_selection = list(selection) if selection is not None else []
    app._mark_dirty = lambda: None
    app._sync_projection_render = lambda: None
    app._refresh_view = lambda: panels.refresh(model)
    return app, panels, model, proj_delta


def _seed_projection(
    proj_delta: ProjectionDelta, model: FieldModel, u: str, v: str
) -> None:
    """Push a projection edge ``u -> v`` onto *proj_delta* via the real command.

    Routes through :class:`AddProjectionEdge` against a throwaway
    :class:`History` so the geometry / validation path matches production, then
    leaves the seeded edge in the delta's ``adds`` for the handler-under-test to
    act on.  A separate history is used so the seed is not counted toward the
    app's own ``can_undo`` assertions.

    :param proj_delta: The app's projection delta to seed.
    :param model: The model whose node positions drive the geometry.
    :param u: Source node name (stored orientation).
    :param v: Destination node name.
    """
    distance_cm, heading_deg = projection_edges.compute_projection_geometry(
        model.nodes[u], model.nodes[v]
    )
    seed_history = History()
    seed_history.do(
        AddProjectionEdge(proj_delta, u, v, distance_cm, heading_deg), model
    )


def _renders_pair(proj_delta: ProjectionDelta, u: str, v: str) -> bool:
    """Return ``True`` iff the directed pair ``(u, v)`` is in the render set."""
    return any(
        edge[0] == u and edge[1] == v for edge in proj_delta.render()
    )


def _plain_edge_exists(model: FieldModel, a: str, b: str) -> bool:
    """Return ``True`` iff an undirected line OR blind edge connects ``{a, b}``."""
    pair = {a, b}
    return any(set(e) == pair for e in model.line_segments) or any(
        set(e) == pair for e in model.blind_edges
    )


# ---------------------------------------------------------------------------
# Stored-vs-click-order projection delete (the load-bearing contract)
# ---------------------------------------------------------------------------


def test_delete_targets_stored_orientation_not_click_order() -> None:
    """A projection edge stored ``(A, B)`` is deleted even when the operator
    picked the endpoints in the REVERSE order ``[B, A]``.

    ``_find_projection_on_pair`` returns the STORED directed pair, so the
    dispatched :class:`DeleteProjectionEdge` targets ``(A, B)`` regardless of
    pick order.  After the delete the ``(A, B)`` edge no longer renders -- the
    delete hit the stored orientation, not the click order.
    """
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["B", "A"]
    )
    # Stored orientation is (A, B); the operator's pick order is the reverse.
    _seed_projection(proj_delta, model, "A", "B")
    assert _renders_pair(proj_delta, "A", "B"), "seed must render (A, B)"

    app._handle_delete_key()

    # The stored (A, B) projection edge is gone from the render set.
    assert not _renders_pair(proj_delta, "A", "B"), (
        "delete must drop the STORED (A, B) edge despite the [B, A] pick order"
    )
    # A real DeleteProjectionEdge went through history (the seed used a separate
    # history, so this undo entry is the delete alone).
    assert app._history.can_undo is True
    assert app._history.undo_stack, "a command must be recorded"
    assert isinstance(app._history.undo_stack[-1], DeleteProjectionEdge)
    deleted = app._history.undo_stack[-1]
    assert (deleted.u, deleted.v) == ("A", "B"), (
        "the delete must target the stored (A, B), not the click order (B, A)"
    )
    # The selection is cleared after a successful delete.
    assert app._edge_selection == []


# ---------------------------------------------------------------------------
# L / B override: projection -> line / blind (delete-then-add ordering)
# ---------------------------------------------------------------------------


def test_line_key_overrides_projection_with_line_edge() -> None:
    """``L`` on a pair carrying a projection edge deletes the projection FIRST,
    then adds an undirected line edge -- two history entries, both effects."""
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["A", "B"]
    )
    _seed_projection(proj_delta, model, "A", "B")
    assert _renders_pair(proj_delta, "A", "B")

    app._handle_edge_line_key()

    # Projection gone, line edge present on {A, B}.
    assert not _renders_pair(proj_delta, "A", "B"), (
        "the projection edge must be overridden (deleted)"
    )
    assert ("A", "B") in model.line_segments, "a line edge must replace it"
    assert not model.blind_edges, "no blind edge should appear"
    # Delete-then-add ordering: two genuine history entries, projection delete
    # first, line add second.
    assert len(app._history.undo_stack) == 2
    assert isinstance(app._history.undo_stack[0], DeleteProjectionEdge)
    # The second entry is the line add (no plain-edge command type is imported
    # here, so assert its effect rather than its class).
    assert app._history.undo_stack[1].__class__.__name__ == "AddLineEdge"
    assert app._edge_selection == []


def test_blind_key_overrides_projection_with_blind_edge() -> None:
    """``B`` on a pair carrying a projection edge deletes the projection FIRST,
    then adds an undirected blind edge -- two history entries, both effects."""
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["A", "B"]
    )
    _seed_projection(proj_delta, model, "A", "B")
    assert _renders_pair(proj_delta, "A", "B")

    app._handle_edge_blind_key()

    assert not _renders_pair(proj_delta, "A", "B"), (
        "the projection edge must be overridden (deleted)"
    )
    assert ("A", "B") in model.blind_edges, "a blind edge must replace it"
    assert not model.line_segments, "no line edge should appear"
    assert len(app._history.undo_stack) == 2
    assert isinstance(app._history.undo_stack[0], DeleteProjectionEdge)
    assert app._history.undo_stack[1].__class__.__name__ == "AddBlindEdge"
    assert app._edge_selection == []


def test_line_key_override_finds_projection_in_reverse_orientation() -> None:
    """The L override also resolves a projection stored in the REVERSE
    orientation to the pick order -- it deletes the stored ``(A, B)`` even when
    the operator picked ``[B, A]``, then adds the line edge on ``{A, B}``."""
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["B", "A"]
    )
    _seed_projection(proj_delta, model, "A", "B")

    app._handle_edge_line_key()

    assert not _renders_pair(proj_delta, "A", "B"), (
        "the stored (A, B) projection must be overridden despite [B, A] picks"
    )
    # The line edge is undirected; it lands on {A, B} as the picked (B, A).
    assert _plain_edge_exists(model, "A", "B")
    deleted = app._history.undo_stack[0]
    assert isinstance(deleted, DeleteProjectionEdge)
    assert (deleted.u, deleted.v) == ("A", "B")


# ---------------------------------------------------------------------------
# P override: line -> projection
# ---------------------------------------------------------------------------


def test_projection_key_overrides_line_edge() -> None:
    """``P`` on a pair carrying a line edge deletes the line edge FIRST, then
    adds a DIRECTED projection edge ``(A, B)`` (first pick is the source)."""
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["A", "B"]
    )
    model.line_segments.append(("A", "B"))

    app._handle_edge_projection_key()

    # The plain line edge is gone from the model...
    assert not _plain_edge_exists(model, "A", "B"), (
        "the line edge must be overridden (deleted)"
    )
    # ...and a directed projection (A, B) now renders (first pick = source).
    assert _renders_pair(proj_delta, "A", "B"), (
        "a directed projection (A, B) must replace the line edge"
    )
    # Delete-then-add ordering: DeleteEdge first, AddProjectionEdge second.
    assert len(app._history.undo_stack) == 2
    assert app._history.undo_stack[0].__class__.__name__ == "DeleteEdge"
    assert isinstance(app._history.undo_stack[1], AddProjectionEdge)
    assert app._edge_selection == []


def test_projection_key_direction_follows_pick_order() -> None:
    """The projection direction is the pick order: picking ``[B, A]`` over a
    line edge produces the directed projection ``(B, A)``, not ``(A, B)``."""
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["B", "A"]
    )
    model.line_segments.append(("A", "B"))

    app._handle_edge_projection_key()

    assert not _plain_edge_exists(model, "A", "B")
    assert _renders_pair(proj_delta, "B", "A"), (
        "the projection direction must follow the [B, A] pick order"
    )
    assert not _renders_pair(proj_delta, "A", "B")


# ---------------------------------------------------------------------------
# Bail on a rejected first delete: the add must NOT run
# ---------------------------------------------------------------------------


def test_line_override_bails_when_projection_delete_is_rejected() -> None:
    """If the projection delete is rejected, the line add does NOT run -- no
    orphaned conflict, the model and history are left coherent.

    The override branch is forced by stubbing :meth:`_find_projection_on_pair`
    to report a stored ``(A, B)`` projection on the pair, while the app's REAL
    (empty) ``_proj_delta`` does NOT render it.  The L handler therefore tries
    :class:`DeleteProjectionEdge` first; the command's pre-condition (the edge
    must currently render) fails, ``_dispatch`` swallows the ``ValueError`` and
    returns ``False``, and ``_add_plain_edge_with_override`` bails before the
    add.  The result: no line edge appears, NO history entry is recorded (the
    rejected delete is a History no-op), and a status message names the
    rejection -- exactly the "never leave a half-applied override" contract.
    """
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["A", "B"]
    )
    # Force the override branch: claim a stored (A, B) projection on the pair.
    # The app's real delta is empty, so the dispatched DeleteProjectionEdge will
    # reject (the edge does not render), and the add must NOT proceed.
    app._find_projection_on_pair = lambda a, b: ("A", "B")
    assert not _renders_pair(proj_delta, "A", "B"), (
        "the real delta is empty, so the delete will be rejected"
    )

    app._handle_edge_line_key()

    # Bailed before the add: no line edge, no history entry, a reject status.
    assert ("A", "B") not in model.line_segments, (
        "the add must not run after a rejected projection delete"
    )
    assert not model.blind_edges
    assert app._history.can_undo is False, (
        "the rejected delete is a History no-op -- nothing recorded"
    )
    assert panels.status_messages, "the rejection must surface a status message"


def test_plain_add_path_unchanged_with_no_other_type_edge() -> None:
    """With NO projection edge on the pair, ``L`` runs the plain add path
    unchanged: a single :class:`AddLineEdge`, one history entry, no override."""
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["A", "B"]
    )
    # The delta is empty -- no projection to override.
    assert not _renders_pair(proj_delta, "A", "B")

    app._handle_edge_line_key()

    assert ("A", "B") in model.line_segments
    assert len(app._history.undo_stack) == 1, "exactly one (the line add) entry"
    assert app._history.undo_stack[0].__class__.__name__ == "AddLineEdge"
    assert app._edge_selection == []


# ---------------------------------------------------------------------------
# Delete-key priority + the non-EDGE / non-two-node early returns
# ---------------------------------------------------------------------------


def test_delete_prefers_projection_over_plain_edge() -> None:
    """When a pair carries BOTH a projection edge and a plain edge,
    ``_handle_delete_key`` deletes the PROJECTION first (documented priority).

    (The L/B/P overrides normally keep the two mutually exclusive, but the
    priority is asserted directly here by seeding both and checking which one
    the single Delete removes.)
    """
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["A", "B"]
    )
    _seed_projection(proj_delta, model, "A", "B")
    model.line_segments.append(("A", "B"))

    app._handle_delete_key()

    # Projection removed; the plain line edge survives (priority = projection).
    assert not _renders_pair(proj_delta, "A", "B"), (
        "projection must be deleted first"
    )
    assert ("A", "B") in model.line_segments, (
        "the plain edge must survive a single Delete (projection took priority)"
    )
    assert isinstance(app._history.undo_stack[-1], DeleteProjectionEdge)


def test_delete_key_is_noop_outside_edge_mode() -> None:
    """In a non-EDGE mode the Delete key is an early-return no-op -- it never
    touches the projection delta, the model, or the history (node deletes are
    constrained to the right-click context menu)."""
    app, panels, model, proj_delta = _make_app(
        mode=Mode.MOVE, selection=["A", "B"]
    )
    _seed_projection(proj_delta, model, "A", "B")

    app._handle_delete_key()

    # Nothing happened: the seeded edge still renders, no app-history command,
    # no status message, selection untouched.
    assert _renders_pair(proj_delta, "A", "B"), "delete must not fire in MOVE"
    assert app._history.can_undo is False
    assert panels.status_messages == []
    assert app._edge_selection == ["A", "B"]


def test_delete_key_requires_two_selected_nodes() -> None:
    """In EDGE mode with fewer than two nodes picked, Delete sets a prompt and
    does NOT dispatch -- the early-return guard before any edge lookup."""
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["A"]
    )
    _seed_projection(proj_delta, model, "A", "B")

    app._handle_delete_key()

    assert _renders_pair(proj_delta, "A", "B"), "no edge may be deleted"
    assert app._history.can_undo is False
    assert panels.status_messages, "a 'select two nodes' prompt must be set"


def test_delete_key_reports_no_edge_when_pair_is_bare() -> None:
    """In EDGE mode with two nodes picked but no edge of any type between them,
    Delete sets a 'no edge' status and dispatches nothing."""
    app, panels, model, proj_delta = _make_app(
        mode=Mode.EDGE, selection=["A", "C"]
    )
    # A projection exists on (A, B) but NOT on the picked (A, C) pair.
    _seed_projection(proj_delta, model, "A", "B")

    app._handle_delete_key()

    assert app._history.can_undo is False
    assert _renders_pair(proj_delta, "A", "B"), "the unrelated edge survives"
    assert panels.status_messages, "a 'no edge' status must be set"
