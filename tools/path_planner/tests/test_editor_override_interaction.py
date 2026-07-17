"""T6 GUI override-editor interaction tests (PR #98 triage: U1/U3, R11, R12).

These are the headless GUI tests for the first-hand operator bugs the override
inline editor had: clicking an edge in EDGE mode flash-closed the editor (U1) and
an already-overridden edge could only be highlighted, never re-opened for edit
(U3).  The fix is event-ordering (``_refresh_view`` BEFORE the graft) plus a
graft-behaviour change (existing override -> OPEN the editor pre-filled for edit),
backed by a ``refresh`` dirty-check (R11) and a direction-combobox vocabulary
trim (R12).

Test cases (from the plan's Testing Strategy):

* **TC14** -- edge-click in EDGE mode opens the inline editor and it STAYS open:
  a new edge opens the blank add-form pre-filled with the clicked pair + derived
  type; an existing-override edge opens pre-filled for edit.
* **TC21** -- open an existing override, change ``speed``, Save -> the
  single ``model.edges`` entry is updated in place (same index, same
  ``(type, u, v)``), no duplicate.
* **TC18** -- a node nudge (``MoveNode``) does NOT rebuild the override-list
  widgets (the dirty-check preserves the row frames).
* **TC19** -- the direction combobox values contain no ``DEFAULT`` and exactly
  one inherit sentinel.

Headless pattern (mirrors ``test_editor_panels_smoke.py`` /
``test_editor_inspector.py``): pin the ``Agg`` matplotlib backend and no-op the
one ``matplotlib.use("TkAgg")`` ``editor.canvas`` performs at import time, build
a REAL :class:`SidePanels` on the SESSION-SCOPED shared ``tk_root`` from
``conftest.py`` (repeatedly creating ``tk.Tk()`` in one process is unstable), and
drive a bare ``EditorApp.__new__`` instance whose only Tk collaborator is that
real panel plus a tiny stub canvas -- no real editor window is constructed.
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
    from tools.path_planner.editor.app import (  # noqa: E402
        EditorApp,
        _NUDGE_FINE_CM,
    )
    from tools.path_planner.editor import canvas as _canvas_mod  # noqa: F401,E402
finally:
    matplotlib.use = _real_mpl_use

import pytest  # noqa: E402

from tools.path_planner.editor.history import History  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402
from tools.path_planner.editor.panels import (  # noqa: E402
    _EDGE_TYPE_LINE,
    _INHERIT_OVERRIDE_LABEL,
    _LINE_MODE_DIRECTIONAL_HINT,
    _LINE_MODE_TAGS,
    _MODE_VALUES_BY_TYPE,
    _PIPELINE_DIRECTION_VALUES,
    _PIPELINE_UNSET_LABEL,
    SidePanels,
)
from tools.path_planner.pipeline_config import (  # noqa: E402
    DirectionMode,
    LineFollowMode,
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
        self.render_count: int = 0

    def queue_hit(self, name: str | None) -> None:
        """Queue *name* as the node the next ``find_node_at`` returns."""
        self._queue.append(name)

    def find_node_at(self, _x: float, _y: float) -> str | None:
        """Return the next queued click target (``None`` when exhausted)."""
        return self._queue.pop(0) if self._queue else None

    def render(self, *_args, **_kwargs) -> None:
        """No-op redraw; just count the calls."""
        self.render_count += 1


def _make_model(edges: list[dict] | None = None) -> FieldModel:
    """Return a small, fully-valid :class:`FieldModel`.

    ``A`` -- ``B`` is a LINE segment and ``B`` -- ``G`` a BLIND edge, so the
    edge-mode graft can derive both a LINE and a BLIND type.  ``edges`` seeds the
    per-edge override overlay (empty by default).

    :param edges: Optional initial override overlay entries.
    :return: A valid model ready for a headless panel refresh.
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
        nodes={"A": (0.0, 50.0), "B": (200.0, 10.0), "G": (250.0, 80.0)},
        node_types={"A": "line", "B": "line", "G": "blind"},
        line_segments=[("A", "B")],
        blind_edges=[("B", "G")],
        obstacles=[],
        edges=edges if edges is not None else [],
    )


@pytest.fixture
def app_with_panels(tk_root):
    """A bare ``EditorApp`` wired to a REAL :class:`SidePanels` + stub canvas.

    The app is built via ``__new__`` (its ``__init__`` would construct a full Tk
    window), then the load-bearing collaborators for the edge-click -> refresh ->
    graft path are injected: a real model + history, a real ``SidePanels`` on the
    shared session root (so the inline editor's open/closed state and the row
    widgets are the genuine article), and a small stub canvas.  ``_update_title``
    is stubbed (it would need the real ``_root`` window), and the override-save
    slot is wired to the real handler so a Save dispatches a real command.

    Yields ``(app, panels, model, canvas)``.  The panel is torn down after the
    test; the shared root survives for the session.
    """
    model = _make_model()
    panels = SidePanels(tk_root)
    canvas = _StubCanvas()

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
    app._mark_dirty = lambda: None
    # Wire the override-save slot to the real app handler so the inline editor's
    # Save runs a real SetEdgeOverride through history + refresh.
    panels.on_set_edge_override = app._on_panel_set_edge_override

    # Seed the panel from the model once (the app does this at start-up).
    panels.refresh(model)

    try:
        yield app, panels, model, canvas
    finally:
        # Drop the panel's bands so the next test's panel starts clean; the
        # shared root persists for the session.
        try:
            panels.destroy()
        except Exception:  # noqa: BLE001 -- teardown best-effort
            pass


def _editor_is_open(panels: SidePanels) -> bool:
    """Return ``True`` when the inline override editor sub-frame is mapped.

    A withdrawn test root never reports ``winfo_ismapped``, so the geometry
    manager string ("pack" once shown, "" once hidden) is the robust
    "is it in the layout" probe used across the editor GUI tests.
    """
    return bool(panels._edge_override_editor.winfo_manager())


def _click_edge(app, canvas, a: str, b: str) -> None:
    """Drive two EDGE-mode clicks selecting the directed pair ``a -> b``.

    Queues *a* then *b* as the canvas hits and fires ``_handle_edge_click``
    twice; the second click fills the two-slot selection and triggers the
    refresh-then-graft path under test.
    """
    canvas.queue_hit(a)
    app._handle_edge_click(0.0, 0.0)
    canvas.queue_hit(b)
    app._handle_edge_click(0.0, 0.0)


# ---------------------------------------------------------------------------
# TC14 -- edge-click opens the inline editor and it STAYS open
# ---------------------------------------------------------------------------


def test_edge_click_new_edge_opens_blank_addform_and_stays_open(
    app_with_panels,
):
    """TC14 (new edge): clicking a LINE edge with NO override opens the blank
    add-form pre-filled with the clicked pair + derived type, and it STAYS open.

    This is the U1 regression: the graft now runs AFTER ``_refresh_view``, so the
    refresh's unconditional ``_hide_override_editor`` cannot flash the just-opened
    editor shut.  The add-form opens at the inherit sentinels (no override key).
    """
    app, panels, _model, canvas = app_with_panels

    _click_edge(app, canvas, "A", "B")

    # The inline editor is OPEN after the full edge-click (refresh + graft) ran.
    assert _editor_is_open(panels), "editor must stay open after edge click"
    # Pre-filled from the clicked LINE segment, add-form at the inherit sentinel.
    assert panels._edge_override_u_var.get() == "A"
    assert panels._edge_override_v_var.get() == "B"
    assert panels._edge_override_type_var.get() == "LINE"
    assert (
        panels._edge_override_speed_var.get() == _INHERIT_OVERRIDE_LABEL
    )
    assert (
        panels._edge_override_mode_var.get() == _INHERIT_OVERRIDE_LABEL
    )


def test_edge_click_existing_override_opens_prefilled_for_edit(
    app_with_panels,
):
    """TC14 (existing-override edge): clicking an edge that ALREADY has an
    override opens the inline editor PRE-FILLED from that override for edit, and
    it STAYS open.

    This is the U3 regression: the existing-override branch used to only
    highlight the row (editor closed); it now opens the editor pre-filled so the
    operator can correct the override in one click.
    """
    app, panels, model, canvas = app_with_panels
    # Author an override on the A->B LINE edge, then re-seed the panel.
    model.edges = [{"type": "LINE", "u": "A", "v": "B", "speed": "FAST"}]
    panels.refresh(model)

    _click_edge(app, canvas, "A", "B")

    assert _editor_is_open(panels), "editor must open for an existing override"
    assert panels._edge_override_u_var.get() == "A"
    assert panels._edge_override_v_var.get() == "B"
    assert panels._edge_override_type_var.get() == "LINE"
    # Pre-filled with the authored tag (not the inherit sentinel).
    assert panels._edge_override_speed_var.get() == "FAST"


def test_edge_click_blind_edge_derives_blind_type(app_with_panels):
    """TC14 (derived type): clicking a BLIND segment opens the editor with the
    ``"BLIND"`` derived type, not ``"LINE"``."""
    app, panels, _model, canvas = app_with_panels

    _click_edge(app, canvas, "B", "G")

    assert _editor_is_open(panels)
    assert panels._edge_override_u_var.get() == "B"
    assert panels._edge_override_v_var.get() == "G"
    assert panels._edge_override_type_var.get() == "BLIND"


# ---------------------------------------------------------------------------
# TC21 -- open an existing override, change speed, Save -> in-place
# ---------------------------------------------------------------------------


def test_edit_existing_override_save_updates_in_place_no_duplicate(
    app_with_panels,
):
    """TC21: opening an existing override, changing ``speed``, and
    Saving updates the single ``model.edges`` entry IN PLACE -- same index, same
    ``(type, u, v)`` identity, no duplicate row.

    Drives the full operator flow: edge-click opens the editor pre-filled for
    edit (U3), the operator changes the speed var, and the Save button's
    command dispatches a real ``SetEdgeOverride`` keyed on the unchanged
    ``(LINE, A, B)`` identity.
    """
    app, panels, model, canvas = app_with_panels
    model.edges = [{"type": "LINE", "u": "A", "v": "B", "speed": "FAST"}]
    panels.refresh(model)
    assert len(model.edges) == 1

    _click_edge(app, canvas, "A", "B")
    assert _editor_is_open(panels)
    assert panels._edge_override_speed_var.get() == "FAST"

    # The operator changes the speed, then clicks Save.
    panels._edge_override_speed_var.set("SLOW")
    panels._override_save_button.invoke()

    # Exactly one entry survives -- updated in place, not duplicated.
    assert len(model.edges) == 1, "edit must not create a second entry"
    entry = model.edges[0]
    assert entry["type"] == "LINE"
    assert entry["u"] == "A"
    assert entry["v"] == "B"
    assert entry["speed"] == "SLOW", "speed updated in place"
    # No second (LINE, A, B) entry exists anywhere in the overlay.
    matches = [
        e
        for e in model.edges
        if (e.get("type"), e.get("u"), e.get("v")) == ("LINE", "A", "B")
    ]
    assert len(matches) == 1, "the (type,u,v) identity must be unique"


# ---------------------------------------------------------------------------
# TC18 -- a node nudge does NOT rebuild the override-list widgets (R11)
# ---------------------------------------------------------------------------


def test_node_nudge_does_not_rebuild_override_rows(app_with_panels):
    """TC18: a ``MoveNode`` (node nudge) leaves ``model.edges`` + the segment
    sets unchanged, so ``refresh`` SKIPS the override-row rebuild -- the existing
    row ``tk.Frame`` widgets survive identically (the dirty-check, R11).

    A node nudge mutates only ``model.nodes``; rebuilding the whole override list
    on every coordinate tick is wasted work AND would wipe the row frames the
    edge-mode graft reads after the refresh.  The frame-object identity is the
    robust "was it rebuilt" probe: a rebuild destroys + re-creates every row.
    """
    app, panels, model, _canvas = app_with_panels
    # Seed an override so there is a real row whose widget identity we can track.
    model.edges = [{"type": "LINE", "u": "A", "v": "B", "speed": "FAST"}]
    panels.refresh(model)
    before = dict(panels._edge_override_row_frames)
    assert ("LINE", "A", "B") in before, "the seeded override row must exist"

    # Nudge the SELECTED node up by the fine step (a pure MoveNode dispatch).
    app._selected_node = "A"
    app._nudge_selected(0.0, _NUDGE_FINE_CM)

    # The override-row frames were PRESERVED (same widget objects) -- the
    # dirty-check skipped the teardown/rebuild because edges + segments are
    # unchanged.
    after = panels._edge_override_row_frames
    assert set(after) == set(before), "the row identity set must be unchanged"
    for identity, frame in before.items():
        assert after[identity] is frame, (
            f"row {identity} must be the SAME widget (no rebuild)"
        )


def test_override_rows_rebuild_when_edges_change(app_with_panels):
    """R11 (negative guard): when ``model.edges`` DOES change, the dirty-check
    must NOT suppress the rebuild -- the row set tracks the new overlay.

    Counterpart to TC18: proves the dirty-check skips a rebuild only when the
    inputs are genuinely unchanged, never masking a real edit.
    """
    app, panels, model, _canvas = app_with_panels
    panels.refresh(model)
    assert panels._edge_override_row_frames == {}

    # Add an override via the real command path, then refresh: the row appears.
    app._on_panel_set_edge_override("LINE", "A", "B", "FAST", None, None, None)

    assert ("LINE", "A", "B") in panels._edge_override_row_frames, (
        "a real edges change must rebuild the rows"
    )


# ---------------------------------------------------------------------------
# TC19 -- the direction combobox offers no DEFAULT + one inherit sentinel (R12)
# ---------------------------------------------------------------------------


def test_pipeline_direction_values_drop_default(tk_root):
    """TC19: ``_PIPELINE_DIRECTION_VALUES`` (the direction combobox vocabulary)
    contains NO ``DEFAULT`` member and EXACTLY ONE inherit sentinel.

    R12: direction joined line/blind/turn -- the empty unset sentinel is the
    single spelling of "inherit the pipeline default", so the redundant
    ``DEFAULT`` member is dropped.  Asserted on the module constant AND on the
    live combobox values so a future enum change cannot silently desync them.
    """
    # The unset sentinel leads the list, exactly once.
    assert _PIPELINE_DIRECTION_VALUES[0] == _PIPELINE_UNSET_LABEL
    assert _PIPELINE_DIRECTION_VALUES.count(_PIPELINE_UNSET_LABEL) == 1

    offered = set(_PIPELINE_DIRECTION_VALUES[1:])
    assert DirectionMode.DEFAULT.name not in offered, (
        "direction must not offer DEFAULT (the empty sentinel means inherit)"
    )
    # The offered tags are exactly the non-DEFAULT direction members.
    expected = {
        member.name
        for member in DirectionMode
        if member is not DirectionMode.DEFAULT
    }
    assert offered == expected

    # The live combobox the panel builds carries the same trimmed vocabulary.
    panels = SidePanels(tk_root)
    try:
        values = tuple(panels._pipeline_mode_combos["direction"].cget("values"))
        assert values[0] == _PIPELINE_UNSET_LABEL
        assert values.count(_PIPELINE_UNSET_LABEL) == 1
        assert DirectionMode.DEFAULT.name not in set(values[1:])
        assert set(values[1:]) == expected
    finally:
        try:
            panels.destroy()
        except Exception:  # noqa: BLE001 -- teardown best-effort
            pass


# ---------------------------------------------------------------------------
# TC14 (issue #156 / ADR-069 / AC11) -- the LINE Mode bar offers LEFT/RIGHT +
# the directional help note; a LEFT LINE edge round-trips clean through the
# model's to_json_dict / validate / to_module_proxy path.
# ---------------------------------------------------------------------------


def test_tc14_line_mode_bar_offers_left_right() -> None:
    """The LINE Mode combobox vocabulary offers MIDDLE / LEFT / RIGHT.

    Single-sourced from ``LineFollowMode`` (minus DEFAULT == absence): the
    lateral offset-follow bias tags are now first-class editor choices, with the
    inherit sentinel leading the type-switched combobox values.
    """
    assert _LINE_MODE_TAGS == (
        LineFollowMode.MIDDLE.name,
        LineFollowMode.LEFT.name,
        LineFollowMode.RIGHT.name,
    )
    line_values = _MODE_VALUES_BY_TYPE[_EDGE_TYPE_LINE]
    # The inherit sentinel leads, then the three lateral-bias tags.
    assert line_values[0] == _INHERIT_OVERRIDE_LABEL
    assert "LEFT" in line_values
    assert "RIGHT" in line_values
    assert "MIDDLE" in line_values
    # DEFAULT (== absence / the inherit sentinel) is never offered as a tag.
    assert LineFollowMode.DEFAULT.name not in set(line_values[1:])


def test_tc14_line_mode_directional_hint_present() -> None:
    """The Mode bar carries a note that LINE mode is directional (not mirrored)."""
    assert "directional" in _LINE_MODE_DIRECTIONAL_HINT
    # Names the forward-only semantics so the operator knows to author a
    # companion reverse edge.
    assert "u->v" in _LINE_MODE_DIRECTIONAL_HINT


def test_tc14_left_line_edge_round_trips_clean() -> None:
    """A LEFT LINE edge round-trips through to_json_dict / validate / to_module_proxy.

    The editor persists per-edge overrides via ``to_json_dict``; a LEFT LINE
    override must survive that serialization AND pass ``validate()`` (which wraps
    the model through ``to_module_proxy``), so authoring a biased edge in the
    editor and saving it produces a field the validator accepts.
    """
    model = FieldModel(
        field_mm=(3000, 2000),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="S",
        start_heading_deg=0.0,
        nodes={"S": (10.0, -40.0), "A": (10.0, 10.0), "B": (200.0, 10.0)},
        node_types={"S": "blind", "A": "line", "B": "line"},
        line_segments=[("A", "B")],
        blind_edges=[("S", "A")],
        obstacles=[],
        edges=[{"type": "LINE", "u": "A", "v": "B", "mode": "LEFT"}],
    )
    # validate() wraps the model via to_module_proxy internally -> clean.
    model.validate()

    # The LEFT override survives to_json_dict, and the reloaded model still
    # validates (a full editor save/reopen round-trip on a biased edge).
    dumped = model.to_json_dict()
    assert dumped["EDGES"] == [{"type": "LINE", "u": "A", "v": "B", "mode": "LEFT"}]
    restored = FieldModel.from_json_dict(dumped)
    restored.validate()
    assert restored.edges[0]["mode"] == "LEFT"
