"""Tests for the live field editor's drag fast-path (Issue #78, T4).

The de-lag wave (T1-T3) replaced the per-mouse-motion full ``render`` with a
blit fast-path: the background PNG is decoded once and cached (T1), the canvas
animates only the dragged node's dot / label / incident edges against a baked
static scene (T2), and the app routes a MOVE drag through that fast-path so the
listbox is rebuilt once on release rather than on every frame (T3). These tests
are the automated guard for that wave -- they exercise the REAL
``FieldCanvas`` and ``EditorApp`` surfaces (not a stub) so any signature drift
in T2 / T3 fails the suite (AC1, AC2, AC3, AC10).

Headless pattern (mirrors ``test_editor_app_renames.py``): pin the ``Agg``
backend and no-op ``matplotlib.use`` for the duration of the editor import so
``editor.canvas``'s import-time ``use("TkAgg")`` is swallowed. The canvas tests
build a real ``FieldCanvas`` under Agg; the app test builds a bare
``EditorApp.__new__(EditorApp)`` with injected collaborators.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import matplotlib

# Pin the headless-safe Agg backend before importing the editor, then no-op the
# one ``matplotlib.use("TkAgg")`` switch that ``editor.canvas`` performs at
# import time (it would raise on headless CI and clobber the Agg backend the
# other tests rely on). Restore the real ``use`` immediately afterwards.
matplotlib.use("Agg")
_real_mpl_use = matplotlib.use
matplotlib.use = lambda *args, **kwargs: None
try:
    from tools.path_planner.editor import canvas as canvas_mod  # noqa: E402
    from tools.path_planner.editor.app import EditorApp  # noqa: E402
finally:
    matplotlib.use = _real_mpl_use

import pytest  # noqa: E402

from tools.path_planner.editor import loader  # noqa: E402
from tools.path_planner.editor.canvas import FieldCanvas  # noqa: E402
from tools.path_planner.editor.history import History  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402
from tools.path_planner.editor.panels import Mode  # noqa: E402

_FIXTURE = Path(__file__).parent / "fixtures" / "field_data_small.json"

# A real, on-disk map PNG (sibling-of-field_data ``maps/map.png``) so TC1 can
# assert ``imread`` runs exactly once across two renders. Pointing at a real
# file decodes a real array the cache then reuses; the fixture's own relative
# ``MAP_PNG`` resolves to the same file, but pinning it here keeps the cache
# assertion independent of the fixture's path string.
_REAL_MAP_PNG = "maps/map.png"

# The label sits at (x + offset, y + offset) -- see ``canvas._NODE_LABEL_OFFSET_CM``
# and ``FieldCanvas._apply_drag_geometry``. Bound the test against the module
# constant so a future offset change updates both in lockstep.
_LABEL_OFFSET_CM = canvas_mod._NODE_LABEL_OFFSET_CM

# Drag-target node and the two coordinates we slide it between. "A" is a line
# node incident to exactly one edge in the fixture (the A-B line segment), so
# the edge-endpoint assertions have a single, unambiguous edge to check.
_DRAG_NODE = "A"
_DRAG_X2 = 42.0
_DRAG_Y2 = 73.0

# Mouse button number per the matplotlib MouseEvent contract (1 = left). Named
# so the TC4 release call reads as a deliberate left-button release.
_MOUSE_BUTTON_LEFT = 1


def _load_model() -> FieldModel:
    """Load the small fixture into a fresh :class:`FieldModel`."""
    return FieldModel.from_json_dict(loader.load_field_json(_FIXTURE))


@pytest.fixture
def field_canvas(tk_root):
    """A fresh :class:`FieldCanvas` (Agg backend) built on the shared Tk root.

    Function-scoped so every test gets a clean canvas / artist state, while the
    underlying Tk root is the single session-scoped ``tk_root`` from
    ``conftest.py`` (one root per session -- see that fixture's docstring for why
    it is shared across the GUI test modules rather than redefined here).
    """
    return FieldCanvas(tk_root)


class _CompositeSpy:
    """Counting wrapper for the canvas' ``restore_region`` / ``blit`` methods.

    The blit fast-path composites a frame via
    ``FieldCanvas._canvas.restore_region(...)`` followed by
    ``FieldCanvas._canvas.blit(...)`` (both methods on the embedded
    ``FigureCanvasTkAgg``). Replacing them with this spy lets the tests count
    composites without asserting pixel equality (which is font / AA / matplotlib
    -version flaky). Each spy delegates to the real method so the underlying
    backend state stays consistent.
    """

    def __init__(self, real_method):
        self._real_method = real_method
        self.calls = 0

    def __call__(self, *args, **kwargs):
        self.calls += 1
        return self._real_method(*args, **kwargs)


def _install_composite_spies(canvas: FieldCanvas):
    """Replace ``restore_region`` and ``blit`` with counting spies.

    :return: ``(restore_spy, blit_spy)`` -- inspect ``.calls`` on each.
    """
    restore_spy = _CompositeSpy(canvas._canvas.restore_region)
    blit_spy = _CompositeSpy(canvas._canvas.blit)
    canvas._canvas.restore_region = restore_spy
    canvas._canvas.blit = blit_spy
    return restore_spy, blit_spy


# ---------------------------------------------------------------------------
# TC1 -- the decoded background PNG is cached: a second render does NOT re-imread.
# ---------------------------------------------------------------------------


def test_render_twice_decodes_background_png_once(field_canvas, monkeypatch):
    """Two renders of the same model decode ``map.png`` exactly once (T1 cache).

    ``FieldCanvas._cached_background`` memoizes the decoded array keyed by the
    resolved absolute path, so ``mpimg.imread`` must fire on the first render
    and never again on the second. Without the cache, ``render`` (which runs on
    every mouse-motion tick during a drag) would re-decode the PNG dozens of
    times per second.
    """
    model = _load_model()
    model.map_png = _REAL_MAP_PNG

    real_imread = canvas_mod.mpimg.imread
    imread_calls = {"count": 0}

    def _counting_imread(*args, **kwargs):
        imread_calls["count"] += 1
        return real_imread(*args, **kwargs)

    monkeypatch.setattr(canvas_mod.mpimg, "imread", _counting_imread)

    field_canvas.render(model)
    field_canvas.render(model)

    assert imread_calls["count"] == 1


# ---------------------------------------------------------------------------
# TC2 -- a blit drag-update moves the artist data AND fires the composite calls.
# ---------------------------------------------------------------------------


def test_update_node_drag_moves_artists_and_composites(field_canvas):
    """A drag update repositions the node's animated artists and composites.

    Asserts the DATA (not pixels): the dragged dot's coordinates, both incident
    edge endpoints (moved end follows, far end fixed), and the label offset --
    plus that the ``restore_region`` / ``blit`` compositing pair actually fired
    for the frame.
    """
    model = _load_model()
    field_canvas.render(model)

    # The fixture's single A-incident edge is the A-B line segment; capture B's
    # fixed coordinate so we can prove the far endpoint does NOT move.
    far_x, far_y = model.nodes["B"]

    field_canvas.begin_node_drag(_DRAG_NODE)
    restore_spy, blit_spy = _install_composite_spies(field_canvas)

    field_canvas.update_node_drag(_DRAG_NODE, _DRAG_X2, _DRAG_Y2)

    # ---- Dot data follows the cursor ----
    dot = field_canvas._node_dot_artists[_DRAG_NODE]
    dot_x, dot_y = list(dot.get_xdata()), list(dot.get_ydata())
    assert dot_x == [_DRAG_X2]
    assert dot_y == [_DRAG_Y2]

    # ---- Label data follows the cursor, shifted by the node-label offset ----
    label = field_canvas._node_label_artists[_DRAG_NODE]
    label_x, label_y = label.get_position()
    assert label_x == pytest.approx(_DRAG_X2 + _LABEL_OFFSET_CM)
    assert label_y == pytest.approx(_DRAG_Y2 + _LABEL_OFFSET_CM)

    # ---- Every A-incident edge moves its A endpoint, leaves the far end fixed -
    incident = [
        (u, v, artist)
        for (u, v, _kind, artist) in field_canvas._edge_artists
        if u == _DRAG_NODE or v == _DRAG_NODE
    ]
    assert incident, "fixture must have at least one edge incident to A"
    for u, v, artist in incident:
        xdata, ydata = list(artist.get_xdata()), list(artist.get_ydata())
        if u == _DRAG_NODE:
            assert (xdata[0], ydata[0]) == (_DRAG_X2, _DRAG_Y2)
            assert (xdata[1], ydata[1]) == (far_x, far_y)
        else:  # v == _DRAG_NODE
            assert (xdata[1], ydata[1]) == (_DRAG_X2, _DRAG_Y2)
            assert (xdata[0], ydata[0]) == (far_x, far_y)

    # ---- The compositing pair fired (do NOT assert pixels) ----
    assert restore_spy.calls >= 1
    assert blit_spy.calls >= 1


# ---------------------------------------------------------------------------
# TC3 -- the ~60 Hz throttle coalesces rapid updates into one composite, while
# the artist data still tracks the LAST update.
# ---------------------------------------------------------------------------


def test_rapid_updates_throttle_composite_but_track_last_position(field_canvas):
    """Many updates inside one interval -> one composite; data is the last call.

    ``update_node_drag`` updates the artist data on every tick but only
    composites when at least ``_BLIT_MIN_INTERVAL_S`` has elapsed since the last
    composite (``canvas.time.monotonic()`` is the clock). With the clock frozen
    inside one interval, a burst of updates after the forced opening frame must
    fire the composite at most once, yet the dot data must reflect the final
    call.
    """
    model = _load_model()
    field_canvas.render(model)

    # Controllable clock: ``begin_node_drag`` resets ``_last_blit_time`` to the
    # force-first sentinel (0.0), so the FIRST update always composites. Freeze
    # ``monotonic`` at a fixed time strictly inside one interval of that first
    # composite so every subsequent update is throttled away.
    frozen_now = 1000.0
    monotonic_calls = {"value": frozen_now}

    # Patch the module-level ``time.monotonic`` the canvas reads.
    real_monotonic = canvas_mod.time.monotonic
    canvas_mod.time.monotonic = lambda: monotonic_calls["value"]
    try:
        field_canvas.begin_node_drag(_DRAG_NODE)
        restore_spy, blit_spy = _install_composite_spies(field_canvas)

        # First update at frozen_now: forced composite (force-first sentinel).
        field_canvas.update_node_drag(_DRAG_NODE, 5.0, 5.0)

        # A burst of updates, all within the SAME interval (clock unchanged and
        # < _BLIT_MIN_INTERVAL_S past the first composite). Each must update the
        # data but skip the composite.
        burst_positions = [
            (11.0, 12.0),
            (13.0, 14.0),
            (_DRAG_X2, _DRAG_Y2),  # last call wins for the data assertion
        ]
        for bx, by in burst_positions:
            # Advance the clock by less than one interval so the throttle holds.
            monotonic_calls["value"] = frozen_now + (
                canvas_mod._BLIT_MIN_INTERVAL_S / 2.0
            )
            field_canvas.update_node_drag(_DRAG_NODE, bx, by)

        # Exactly one composite for the forced first frame; the burst coalesced.
        assert blit_spy.calls == 1
        assert restore_spy.calls == 1

        # Data still reflects the LAST update, not the composited first frame.
        dot = field_canvas._node_dot_artists[_DRAG_NODE]
        assert list(dot.get_xdata()) == [_DRAG_X2]
        assert list(dot.get_ydata()) == [_DRAG_Y2]
    finally:
        canvas_mod.time.monotonic = real_monotonic


# ---------------------------------------------------------------------------
# TC4 -- a MOVE drag does NOT rebuild the listbox; release rebuilds it once.
# ---------------------------------------------------------------------------


class _PanelsSpy:
    """Minimal stand-in for ``SidePanels`` that counts ``refresh`` calls.

    Provides exactly the surface ``_refresh_view`` and the MOVE-drag path touch:
    ``refresh`` (the listbox / inspector rebuild we are counting),
    ``set_undo_enabled`` / ``set_redo_enabled`` (button enable state),
    ``set_status`` (status bar text), ``set_dirty`` (the title-bar dirty dot,
    flipped by ``_mark_dirty`` on the MoveNode dispatch), and a ``mode_var``
    whose ``.get()`` returns the MOVE mode value (read by
    ``EditorApp._current_mode``).
    """

    def __init__(self) -> None:
        self.refresh_count = 0
        self.last_model = None
        self.undo_enabled = None
        self.redo_enabled = None
        self.last_status = None
        self.dirty = None
        self.mode_var = SimpleNamespace(get=lambda: Mode.MOVE.value)

    def refresh(self, model) -> None:
        self.refresh_count += 1
        self.last_model = model

    def set_undo_enabled(self, enabled: bool) -> None:
        self.undo_enabled = enabled

    def set_redo_enabled(self, enabled: bool) -> None:
        self.redo_enabled = enabled

    def set_status(self, text: str) -> None:
        self.last_status = text

    def set_dirty(self, dirty: bool) -> None:
        self.dirty = dirty

    def set_validation(self, ok: bool, detail: str = "") -> None:
        # Per-move validation runs at the tail of the real ``_refresh_view``;
        # the blit test only cares about refresh/dirty bookkeeping, so record
        # the latest verdict without asserting on it.
        self.last_validation = (ok, detail)


def test_move_drag_does_not_rebuild_listbox_until_release(field_canvas):
    """A live MOVE drag rebuilds the listbox zero times; release rebuilds once.

    Drives the REAL ``EditorApp._on_canvas_drag`` and ``_finish_move`` against a
    spy ``SidePanels`` and a real ``FieldCanvas``. During the drag the app must
    route every motion through ``FieldCanvas.update_node_drag`` (no
    ``_refresh_view``, hence ``panels.refresh`` is never called); on release the
    single ``MoveNode`` dispatch runs exactly one ``_refresh_view`` so the
    inspector / listbox re-sync to the final position. Exercising the real
    handlers is what surfaces any T2 / T3 contract drift.
    """
    model = _load_model()

    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = History()
    app._canvas = field_canvas
    panels = _PanelsSpy()
    app._panels = panels
    app._dirty = False
    app._edge_selection = []
    app._proposed_edges = None
    # ``_refresh_view`` -> ``_update_title`` -> ``self._root.title(...)``; a
    # bare namespace with a no-op ``title`` keeps ``_refresh_view`` fully real
    # without spawning a Tk window.
    app._root = SimpleNamespace(title=lambda *_a, **_k: None)

    # Seed the canvas so ``begin_node_drag`` / ``find_node_at`` have a model.
    field_canvas.render(model)

    origin_xy = model.nodes[_DRAG_NODE]

    # ---- Latch the drag exactly as _handle_move_click does ----
    app._drag_target = _DRAG_NODE
    app._drag_origin_xy = origin_xy
    field_canvas.begin_node_drag(_DRAG_NODE)

    refresh_before_drag = panels.refresh_count

    # ---- Several live drag ticks: model untouched, listbox NOT rebuilt ----
    drag_path = [(20.0, 20.0), (30.0, 35.0), (_DRAG_X2, _DRAG_Y2)]
    for dx, dy in drag_path:
        app._on_canvas_drag(dx, dy)
        # The blit fast-path never writes the model during a drag.
        assert model.nodes[_DRAG_NODE] == origin_xy

    assert panels.refresh_count == refresh_before_drag, (
        "listbox must not be rebuilt mid-drag"
    )

    # ---- Release: one MoveNode dispatch -> exactly one listbox rebuild ----
    app._on_canvas_release(_DRAG_X2, _DRAG_Y2, _MOUSE_BUTTON_LEFT)

    assert panels.refresh_count == refresh_before_drag + 1
    # The model (and therefore the inspector / listbox source) now holds the
    # final dragged position.
    assert model.nodes[_DRAG_NODE] == (_DRAG_X2, _DRAG_Y2)
    # The release cleared the drag latch so the next interaction starts clean.
    assert app._drag_target is None
    assert app._drag_origin_xy is None
    # The single MoveNode is on the undo stack (one undoable entry per drag).
    assert app._history.can_undo


# ---------------------------------------------------------------------------
# TC5 -- button-release robustness (#9 None-button, #15 mismatched button).
# ---------------------------------------------------------------------------


_MOUSE_BUTTON_RIGHT = 3


def _in_axes_event(field_canvas, *, button, xdata, ydata):
    """Build a minimal in-axes matplotlib-style event for the release handler."""
    return SimpleNamespace(
        button=button,
        inaxes=field_canvas._ax,
        xdata=xdata,
        ydata=ydata,
        x=0.0,
        y=0.0,
    )


def test_release_with_none_button_still_fires_on_release(field_canvas):
    """An out-of-window release that reports button=None must still commit.

    Some matplotlib backends pass ``None`` for the button on a release outside
    the window. With a drag latched, the handler must clear the latch AND fire
    ``on_release`` using the LATCHED button (#9) -- the old code early-returned
    on a None button, leaving a MOVE drag stuck with no MoveNode and no undo.
    """
    field_canvas.render(_load_model())
    # A left-button drag is in progress (press latched the button).
    field_canvas._active_button = _MOUSE_BUTTON_LEFT
    received: list = []
    # 4-arg form: (x, y, button, key) -- ``_in_axes_event`` builds a plain
    # ``SimpleNamespace`` with no ``key`` attribute, so the handler's
    # ``getattr(event, "key", None)`` fallback supplies ``None`` here.
    field_canvas.on_release = lambda x, y, b, k: received.append((x, y, b, k))

    field_canvas._on_button_release(
        _in_axes_event(field_canvas, button=None, xdata=10.0, ydata=20.0)
    )

    assert field_canvas._active_button is None  # latch cleared
    assert received == [
        (10.0, 20.0, _MOUSE_BUTTON_LEFT, None)
    ]  # fired w/ latched btn, no modifier key


def test_release_with_mismatched_button_clears_latch(field_canvas):
    """A different button's release must still clear the single-button latch.

    Press right (latch=3); a backend delivers a LEFT release with no matching
    right release. The latch must be cleared (#15) so bare-hover motion no
    longer fires ``on_drag`` as if a drag were still in progress.
    """
    field_canvas.render(_load_model())
    field_canvas._active_button = _MOUSE_BUTTON_RIGHT
    field_canvas.on_release = lambda *_a: None

    field_canvas._on_button_release(
        _in_axes_event(
            field_canvas, button=_MOUSE_BUTTON_LEFT, xdata=5.0, ydata=6.0
        )
    )

    assert field_canvas._active_button is None

    # A subsequent bare hover must NOT fire on_drag (the stuck-latch symptom).
    drags: list = []
    field_canvas.on_drag = lambda x, y: drags.append((x, y))
    field_canvas._on_motion_notify(
        _in_axes_event(
            field_canvas, button=None, xdata=7.0, ydata=8.0
        )
    )
    assert drags == []


# ---------------------------------------------------------------------------
# TC6 -- the OBSTACLE rubber-band drag is de-lagged via the blit fast-path (#11).
# ---------------------------------------------------------------------------


def test_obstacle_drag_blits_preview_without_full_render(field_canvas):
    """Sizing an obstacle composites only the preview rect (no full render).

    ``begin_obstacle_drag`` bakes the static scene + arms an animated preview
    rectangle; ``update_obstacle_preview`` restores that background and blits
    just the rectangle (the ~60 Hz fast-path the MOVE drag uses), instead of the
    old full ``_refresh_view`` per motion event. ``end_obstacle_drag`` tears the
    preview down.
    """
    field_canvas.render(_load_model())

    field_canvas.begin_obstacle_drag()
    rect = field_canvas._obstacle_preview_artist
    assert rect is not None
    assert rect.get_animated() is True

    # Spy AFTER arming (begin uses draw/copy_from_bbox, not restore/blit).
    restore_spy, blit_spy = _install_composite_spies(field_canvas)
    field_canvas.update_obstacle_preview(10.0, 20.0, 40.0, 60.0)

    # The blit compositing pair fired (NOT a full ax.clear + render).
    assert restore_spy.calls >= 1
    assert blit_spy.calls >= 1
    # The rectangle spans the dragged extent (lower-left + width/height).
    assert rect.get_x() == pytest.approx(10.0)
    assert rect.get_y() == pytest.approx(20.0)
    assert rect.get_width() == pytest.approx(30.0)
    assert rect.get_height() == pytest.approx(40.0)

    field_canvas.end_obstacle_drag()
    assert field_canvas._obstacle_preview_artist is None
    assert field_canvas._obstacle_blit_background is None
