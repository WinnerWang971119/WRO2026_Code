"""Hermetic tests for the CTPS track builder (T10: TC1-TC4).

These tests exercise :func:`tools.path_planner.ctps.track_builder.build`
end-to-end against the stable toy field, with **no C++ compiler** anywhere
in the path — geometry comes entirely from the parity-locked Python sim.

Covered cases (CTPS plan, T10):

* **TC1** — a two-``goto`` command stream replays into keyframe poses that
  equal an INDEPENDENT ``step_through`` reference run, with a monotonic clock
  and ``duration_s == keyframes[-1].t``.
* **TC2** — a ``task_start`` / ``manip`` / ``goto`` / ``manip`` / ``task_end``
  stream produces ordered Track events with the expected labels.
* **TC3** — a raw ``drive driveDist 15`` before a ``goto`` routes through
  ``operator_move``; the following goto's return prologue drives the offset
  back, so a ``return_to_node`` marker is emitted and NO residual ``drift_warn``
  survives (issue #130, T11).
* **TC4** — the same shape with a 2 cm drive sits under the return trigger, so
  neither a ``return_to_node`` marker nor a ``drift_warn`` is emitted (T11).

Fixtures are hand-authored Data Model A command streams under
``tests/fixtures/ctps_*.json``; every ``goto`` target is reachable from the
toy field's ``START_NODE`` ("A").

Drive-recording additions (CTPS plan, T9 — NC9-NC14, NC16):

* **NC9** — a GYRO+heading ``driveDist`` renders exactly three keyframes
  (seed -> rotate-in-place -> advance along the held heading).
* **NC10** — a heading WITHOUT ``mode == "gyro"`` is ignored: the move renders
  along the current facing, byte-identical to a plain ``driveDist`` (firmware
  parity).
* **NC11** — ``lineFollowDist`` renders a forward advance + a drive event.
* **NC12** — ``reverse`` on ``lineFollowDist`` / ``stopAtLine`` / ``wallStop``
  moves backward along the heading; a plain ``driveDist`` moves forward.
* **NC13** — GYRO composes with ``reverse`` (rotate then back up), and GYRO is
  honoured on all three forward-geometry methods.
* **NC14** — a cm-less ``wallStop`` / ``lineFollowDist`` narrates only, with NO
  geometry keyframe and no ``float(None)`` crash.
* **NC16** — a ``lineFollowDist`` offset before a ``goto`` routes through the
  operator-offset / return-to-node path (a ``return_to_node`` marker, no
  residual ``drift_warn``), exactly like ``driveDist`` (issue #130, T11).

The toy chassis seeds facing ``START_HEADING_DEG == 90`` (node ``S``), so
"forward along the current facing" is ``+y`` in these tests, not ``+x``.

Pivot-arc additions (CTPS issue #55, T5 — TC1-TC7, TC11, TC12):

* **TC1** — a goto over a 90 deg dead-reckon pivot lands on the node with NO
  ``drift_warn`` (today it drifts 12.7 cm and warns).
* **TC2** — the pivot's arc sub-keyframes trace the rigid rotation about the
  planted wheel, evenly timed, replacing the single in-place turn keyframe.
* **TC3** — the emitted ``pivots`` segment has the right shape (wheel side,
  centre, angle, time bracket).
* **TC4** — chained pivots (two turns in one goto) each animate and the whole
  move lands on the final node with no ``drift_warn``.
* **TC5** — a 70 deg LINE-incoming pivot lands on its node.
* **TC6** — a pivot-free goto keeps the pre-#55 keyframe shape and omits the
  ``pivots`` key entirely (AC6).
* **TC7** — the chassis-nudge never perturbs the sim's primitive stream (the
  parity tripwire), verified by driving ``step_through`` directly.
* **TC11** — ``ctps/config.py`` exposes ``ARC_STEP_DEG > 0``.
* **TC12** — a LINE-departing pivot (sensor-terminated departure) lands on the
  destination node via the along-edge re-anchor (ADR-044 D1), using the new
  ``make_line_departing_pivot_field`` fixture.
* **TC13** — chained pivots joined by a LINE (sensor) middle leg land on the
  final node: the sensor-departure landing subtracts the DEPARTING trim along
  the departure-edge ray instead of snapping onto the node, so the destination
  pivot's approach trim survives (an unconditional node-snap translated the
  whole tail 9 cm off, silently — under the drift threshold).

These reuse the existing per-turn pivot fixtures in
``tests/fixtures/pivot_field_data.py`` (ADR-036) via a synthetic single-``goto``
``Trace`` built directly (no JSON, no C++ dump) — see ``_goto_trace`` below.

Structured manip-key additions (CTPS object-cargo plan, T9 — P-TC3, P-TC4):

* **P-TC3** — a ``manip`` Track event carries structured ``method``/``arg``
  keys, additive to the unchanged ``label``.
* **P-TC4** — a manip-free trace's Track carries none of those new keys
  anywhere (golden byte-identical).

Boundary-invariant + return/declare rebaseline (CTPS motion-profile plan, T13):

* **TC7** — a GENERAL boundary-pose invariant: for every fixture in
  :data:`_TC7_FIXTURES`, every ``step_through`` snapshot pose (an INDEPENDENT
  reference replay) appears byte-exact among the built ``Track``'s keyframes,
  in order, and the keyframe clock is non-decreasing (AC10). Scoped to
  pivot-free fixtures — a pivot's chassis-nudge deliberately re-poses the
  SHARED sim before its next primitive dispatches (the whole point of the
  nudge is killing drift), so an independently-constructed reference sim
  legitimately diverges from the rendered keyframes there; the pivot suite
  above (TC1-TC13) already pins those via the node-landing + arc-geometry
  assertions instead.
* **TC8-scaffold** — ``ctps_all_hops_drift.json`` (new fixture): a mission
  over the toy field graph exercising LINE, BLIND, PROJECTION, STOP_AT_LINE,
  and reverse hops (the toy field's ``PIVOT_TURNS`` is empty, so no pivot hop
  is reachable here — see the fixture docstring), asserting drift ~= 0 (float
  noise) at every ``goto`` arrival. The full regression-locked sweep is T15's
  ``test_ctps_drift.py``; this is the scaffold + the reusable fixture.
* **TC9** — a raw drive integrates the offset; the next ``goto`` emits +
  renders ``return_to_node``; no residual ``drift_warn`` survives (AC7).
* **TC10** — a raw drive at trace end leaves a residual ``drift_warn``; an
  over-cap raw drive under the FAULT gate raises ``OffMapFaultError``, which
  ``build`` turns into a fault ``goto`` event that ends the track (AC7).
* **TC11** — ``declare_position`` applies the sim belief reset (node change +
  offset clear, heading preserved) and emits a marker; an unknown-node declare
  no-ops and renders BYTE-IDENTICAL (sha256 of the sorted-key JSON) to the
  same trace with the declare op dropped entirely (AC8).

CTPS RETURN-move speed (issue #157):

* **TC17** — under a field-sourced ``PIPELINE_SPEED: FAST``, both
  ``ACT_RETURN_FORWARD`` (dead-reckon return) and ``ACT_RETURN_STOP_AT_LINE``
  (line-seek return) render their cruise plateau at the FAST PWM
  (``cfg.speed_fast``, matching firmware's ``_dispatchReturnMoveAfterTurn``
  dispatching both at ``PIPELINE_CONFIG.speed``), while ``ACT_WALL_DEPART``
  stays at the un-tiered ``SPEED_NORMAL`` default (matching firmware's
  ``.setPower``-free wall-departure move) — proving the three tags are no
  longer conflated.
"""

from __future__ import annotations

import hashlib
import json
import warnings
from math import ceil, hypot
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.path_planner.compiler import ACT_TURN_REL
from tools.path_planner.ctps.config import ARC_STEP_DEG, SCALE_CM_S_PER_PWM
from tools.path_planner.ctps.profile import default_config
from tools.path_planner.ctps.track_builder import build
from tools.path_planner.ctps.trace_loader import Command, Trace, load
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.pipeline_config import SpeedTier
from tools.path_planner.planner import (
    BlindDecayParams,
    compute_all_pairs_next_hop,
    parse_track_cm_from_config,
)
from tools.path_planner.sim import runtime as _runtime
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import (
    OffMapFaultError,
    OffMapGate,
    PathTrackerSim,
    RETURN_DISTANCE_CAP_CM,
)
from tools.path_planner.tests.fixtures import pivot_field_data, toy_field_data

_FIXTURES = Path(__file__).resolve().parent / "fixtures"

#: Position tolerance for keyframe-vs-reference pose equality (cm / deg).
_POSE_TOL = 1e-6


# ---------------------------------------------------------------------------
# Densification helpers (issue #130, T10)
#
# Since T10 the track builder DENSIFIES every geometry step off the firmware
# motion profile: the single flat keyframe the pre-#130 builder appended per
# primitive is replaced by a run of profile-timed sub-keyframes whose LAST
# sub-keyframe byte-equals the destination (snapshot) pose. So a test can no
# longer assert an exact keyframe COUNT or index a "the single advance"
# keyframe. The invariant that survives is: every destination pose appears
# byte-exact among the keyframes IN ORDER (the boundary-pose invariant), and
# the clock is monotone non-decreasing. These helpers express that contract.
# ---------------------------------------------------------------------------


def _pose_matches(kf, pose, tol=_POSE_TOL) -> bool:
    """Whether a keyframe's ``(x, y, h)`` matches ``pose`` within ``tol``."""
    px, py, ph = pose
    return (
        abs(kf["x"] - px) <= tol
        and abs(kf["y"] - py) <= tol
        and abs(kf["h"] - ph) <= tol
    )


def _boundary_indices(keyframes, ref_poses, tol=_POSE_TOL):
    """Return the keyframe indices at which ``ref_poses`` appear, in order.

    Walks ``keyframes`` once and, for each pose in ``ref_poses`` (the ordered
    list of primitive-boundary / snapshot poses), finds the NEXT keyframe that
    matches it within ``tol``. Asserts every ref pose is found (so the
    boundary-pose invariant holds: each densified step ends exactly on its
    snapshot pose) and that the matches are strictly increasing (in order).

    :return: the list of matched keyframe indices, one per ref pose.
    """
    indices = []
    cursor = 0
    for pose in ref_poses:
        found = None
        for i in range(cursor, len(keyframes)):
            if _pose_matches(keyframes[i], pose, tol):
                found = i
                break
        assert found is not None, f"boundary pose {pose} not found in keyframes"
        indices.append(found)
        cursor = found + 1
    return indices


def _assert_monotone(times, strict=False):
    """Assert a timestamp sequence is (strictly) non-decreasing."""
    if strict:
        assert all(b > a for a, b in zip(times, times[1:]))
    else:
        assert all(b >= a for a, b in zip(times, times[1:]))


def _build_quiet(trace, field_data):
    """Run :func:`build` with the toy field's orphan-node warning suppressed.

    ``build`` calls ``build_graph(field_data)`` directly; the toy field's P1
    node is orphaned (it is only reached via the demo EDGES projection edge,
    which the toy fixture omits), so ``build_graph`` emits a benign
    ``UserWarning``. That warning is expected fixture noise and is silenced
    here so it does not interfere with the assertions.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        return build(trace, field_data)


def _reference_snapshots(field_data, goto_targets):
    """Replay ``goto_targets`` on a fresh, INDEPENDENT sim; return its snapshots.

    Mirrors exactly what :func:`build` does for a pure ``goto`` stream — one
    long-lived chassis + sim, heading zeroed once, then ``step_through`` per
    goal — but is constructed entirely independently of the builder so it is a
    genuine cross-check rather than a tautology.

    :return: a flat list of ``(x_cm, y_cm, heading_deg)`` tuples, one per
        ``step_through`` yield across all goals, in dispatch order.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        graph = build_graph(field_data)
        next_hop = compute_all_pairs_next_hop(
            graph, BlindDecayParams.from_field_data(field_data)
        )
    chassis = VirtualChassis.from_field_data(field_data)
    sim = PathTrackerSim(field_data, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    snapshots = []
    for target in goto_targets:
        for _primitive, snap in sim.step_through(target):
            snapshots.append((snap.x_cm, snap.y_cm, snap.heading_deg))
    return snapshots


# ---------------------------------------------------------------------------
# TC1 — two-goto stream: keyframes match an independent reference run
# ---------------------------------------------------------------------------


def test_tc1_two_goto_keyframes_match_reference_run():
    """TC1: every step_through boundary pose appears in order; clock monotonic.

    Post-#130 the builder densifies each hop, so keyframes no longer map 1:1
    onto ``step_through`` yields. The invariant that survives is the
    boundary-pose one: every snapshot of an independently constructed
    ``step_through`` reference replay appears BYTE-EXACT among the keyframes, in
    order (the last sub-keyframe of each densified step). The densified track
    therefore has MORE keyframes than the reference has snapshots. Also asserts
    the running clock is monotonic non-decreasing and
    ``duration_s == keyframes[-1]["t"]``.
    """
    trace = load(_FIXTURES / "ctps_two_goto.json")
    track = _build_quiet(trace, toy_field_data)

    # The stream is exactly two reachable gotos.
    goto_targets = [cmd.target for cmd in trace.commands if cmd.op == "goto"]
    assert goto_targets == ["B", "C"]

    reference = _reference_snapshots(toy_field_data, goto_targets)

    # Densification: strictly more keyframes than reference snapshots, and every
    # snapshot pose appears in order as a densified step's final sub-keyframe.
    assert len(track.keyframes) > len(reference) + 1
    _boundary_indices(track.keyframes, reference)

    # Seed keyframe sits at the start pose at t = 0.
    sx, sy = toy_field_data.NODES[toy_field_data.START_NODE]
    assert track.keyframes[0]["t"] == 0.0
    assert track.keyframes[0]["x"] == pytest.approx(sx, abs=_POSE_TOL)
    assert track.keyframes[0]["y"] == pytest.approx(sy, abs=_POSE_TOL)

    # Monotonic non-decreasing clock.
    _assert_monotone([kf["t"] for kf in track.keyframes])

    # duration_s is the final keyframe timestamp.
    assert track.duration_s == pytest.approx(track.keyframes[-1]["t"], abs=_POSE_TOL)
    assert track.duration_s > 0.0


# ---------------------------------------------------------------------------
# TC2 — task/manip stream produces ordered events with the right labels
# ---------------------------------------------------------------------------


def test_tc2_task_manip_events_in_order():
    """TC2: ordered Track events for task_start -> manip -> goto -> manip -> task_end.

    Key assertion: the event ``(type, label)`` sequence contains, in order,
    the ``task_start`` banner, ``setGripper(180)``, the ``goto`` drift check
    slot, ``setLift(120)``, and the ``task_end`` banner — proving manip args
    and task names survive into the narration track.

    P-TC3 (CTPS object-cargo plan, T9): each ``manip`` event ALSO carries
    structured ``method``/``arg`` keys additive to the existing ``label`` —
    proving the player can dispatch on ``method``/``arg`` directly without
    parsing the label string, while ``label`` itself is unchanged.
    """
    trace = load(_FIXTURES / "ctps_task_events.json")
    track = _build_quiet(trace, toy_field_data)

    typed = [(ev["type"], ev["label"]) for ev in track.events]

    # The manip labels carry their uint8 argument.
    assert ("manip", "setGripper(180)") in typed
    assert ("manip", "setLift(120)") in typed

    # Task banners carry the task name (+ result on end).
    assert ("task_start", "PickAndPlace") in typed
    assert ("task_end", "PickAndPlace ok") in typed

    # Relative ordering: task_start -> setGripper -> setLift -> task_end.
    order = [
        typed.index(("task_start", "PickAndPlace")),
        typed.index(("manip", "setGripper(180)")),
        typed.index(("manip", "setLift(120)")),
        typed.index(("task_end", "PickAndPlace ok")),
    ]
    assert order == sorted(order)

    # Event timestamps are non-decreasing (shared running clock).
    ev_times = [ev["t"] for ev in track.events]
    assert all(b >= a for a, b in zip(ev_times, ev_times[1:]))

    # P-TC3: the manip events carry structured method/arg, additive to the
    # unchanged label.
    manip_events = [ev for ev in track.events if ev["type"] == "manip"]
    assert len(manip_events) == 2
    grip, lift = manip_events

    assert grip["method"] == "setGripper"
    assert grip["arg"] == 180
    assert grip["label"] == "setGripper(180)"

    assert lift["method"] == "setLift"
    assert lift["arg"] == 120
    assert lift["label"] == "setLift(120)"


# ---------------------------------------------------------------------------
# P-TC4 — a manip-free trace's Track stays byte-identical (no new keys)
# ---------------------------------------------------------------------------


def test_p_tc4_manip_free_trace_track_byte_identical():
    """P-TC4: a manip-free trace's Track carries NO new keys anywhere.

    ``ctps_two_goto.json`` has no ``manip`` command, so the new ``method``/
    ``arg`` keys (additive to the ``manip`` event only) must never appear:
    every event dict keeps EXACTLY its pre-T9 key set, and the whole
    ``to_dict()`` output keeps EXACTLY its pre-T9 top-level key set (no
    ``pivots`` — the toy field is pivot-free). Combined with TC1's independent
    keyframe cross-check, this pins the additive change to the ``manip``
    branch alone.
    """
    trace = load(_FIXTURES / "ctps_two_goto.json")
    track = _build_quiet(trace, toy_field_data)

    # No manip commands in this trace at all.
    assert all(cmd.op != "manip" for cmd in trace.commands)

    # Every event is a `goto` (this fixture has no other event-emitting op);
    # each keeps exactly its pre-T9 key set -- no `method`/`arg` leakage.
    for ev in track.events:
        assert ev["type"] == "goto"
        assert set(ev.keys()) == {"t", "type", "t_end", "label"}

    # The top-level Track dict keeps exactly its pre-T9 key set (no "pivots"
    # -- the toy field is pivot-free).
    assert set(track.to_dict().keys()) == {
        "schema_version",
        "field",
        "duration_s",
        "keyframes",
        "events",
    }

    # Every keyframe keeps exactly its pre-T9 key set too.
    for kf in track.keyframes:
        assert set(kf.keys()) == {"t", "x", "y", "h"}


# ---------------------------------------------------------------------------
# TC3 — a raw drive before a goto renders a return-to-node maneuver (issue #130)
# ---------------------------------------------------------------------------


def test_tc3_drive_before_goto_renders_return_no_residual_drift():
    """TC3 (T11): a 15 cm raw drive before goto B routes through ``operator_move``.

    Since issue #130 (T11) a raw off-node move integrates the operator offset on
    the shared sim; the following goto's ``step_through`` return prologue then
    physically drives that offset back onto the node. So the trace renders a
    ``return_to_node`` marker and — because the goto re-anchors — leaves NO
    residual ``drift_warn`` (the pre-#130 open/close drift span is gone). This
    replaces the old drift-span assertion; T13 owns the comprehensive
    track-level rebaseline.
    """
    trace = load(_FIXTURES / "ctps_drive_drift.json")
    track = _build_quiet(trace, toy_field_data)

    return_events = [ev for ev in track.events if ev["type"] == "return_to_node"]
    assert len(return_events) == 1
    # A goto follows the raw move, so the return re-anchors it: no residual.
    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []


# ---------------------------------------------------------------------------
# TC4 — a sub-return-trigger raw drive before a goto surfaces the drift
# ---------------------------------------------------------------------------


def test_tc4_small_drive_before_goto_surfaces_subthreshold_drift():
    """TC4: a 2 cm raw drive before goto B sits under the return trigger.

    The 2 cm offset is below ``RETURN_TRIGGER_THRESHOLD_CM`` (3 cm), so the
    return gate resolves to NONE — no physical return maneuver is dispatched (the
    firmware does NOT drive back for a sub-3 cm offset either), and so no
    ``return_to_node`` marker. But the following goto's arrival re-anchor clears
    the offset before the trace-end residual check, so the excursion is a real,
    uncorrected error that would otherwise vanish. It is surfaced as a POINT
    ``drift_warn`` at the goto start reporting the offset off the parked node.
    """
    trace = load(_FIXTURES / "ctps_drive_no_drift.json")
    track = _build_quiet(trace, toy_field_data)

    # Sub-3 cm: no physical return maneuver, hence no return marker.
    assert [ev for ev in track.events if ev["type"] == "return_to_node"] == []
    # ... but the sub-threshold excursion is still flagged so it is not hidden.
    drift_events = [ev for ev in track.events if ev["type"] == "drift_warn"]
    assert len(drift_events) == 1
    warn = drift_events[0]
    assert "2.0 cm off node" in warn["label"]
    # A point event stamped at the goto start (before the arrival re-anchor), not
    # a span: no t_end key.
    assert "t_end" not in warn
    assert warn["t"] > 0.0


def test_drive_without_following_goto_keeps_trace_end_residual():
    """A raw drive that ends the mission keeps a trace-end residual ``drift_warn``.

    No ``goto`` follows the drive, so nothing re-anchors the accumulated offset
    (issue #130, T11): the residual surfaces as a single POINT ``drift_warn`` at
    the track end (``t == duration_s``), its label reporting the residual
    magnitude off the parked node. It is no longer a span (there is no re-anchor
    to close it), so it carries no ``t_end``.
    """
    trace = Trace(
        schema_version=1,
        start_node="A",
        start_heading_deg=0.0,
        build_error=False,
        commands=[Command(seq=0, op="drive", method="driveDist", cm=20.0)],
    )
    track = _build_quiet(trace, toy_field_data)

    drift_events = [ev for ev in track.events if ev["type"] == "drift_warn"]
    assert len(drift_events) == 1
    warn = drift_events[0]
    assert warn["t"] == pytest.approx(track.duration_s, abs=_POSE_TOL)
    # A point event, not a span: no t_end key.
    assert "t_end" not in warn


# ---------------------------------------------------------------------------
# TC5 — stopAtLine advances the chassis by its nominal expectedCm
# ---------------------------------------------------------------------------


def test_tc5_stop_at_line_advances_chassis_by_expected_cm():
    """TC5: a ``stopAtLine`` drive moves the chassis forward by its ``cm``.

    Regression guard for the PR-49 finding that ``stopAtLine`` traces rendered
    as no movement. The recorder serialises the nominal ``expectedCm`` to the
    crossing as ``cm``; playback must replay that as a forward advance (like
    ``driveDist``), emitting one geometry keyframe past the seeded start pose.
    """
    from math import hypot

    trace = load(_FIXTURES / "ctps_stop_at_line.json")
    track = _build_quiet(trace, toy_field_data)

    # Densified: the single advance is now a run of profile-timed sub-keyframes
    # (seed + N), so assert the FINAL keyframe lands 12 cm from the seed rather
    # than a fixed count. Every interior sub-keyframe lies on the seed->final
    # segment, so the straight-line advance still totals the nominal cm.
    assert len(track.keyframes) >= 2
    seed, moved = track.keyframes[0], track.keyframes[-1]

    advance = hypot(moved["x"] - seed["x"], moved["y"] - seed["y"])
    assert advance == pytest.approx(12.0, abs=_POSE_TOL)

    # The drive still surfaces as a timeline event.
    assert any(ev["type"] == "drive" for ev in track.events)


# ===========================================================================
# T9 drive-recording tests (NC9-NC14, NC16)
#
# These exercise the T7 renderer additions: the gyro two-phase render,
# non-gyro heading parity, ``lineFollowDist`` geometry, reverse direction, the
# cm-less narrate-only guard, and a ``lineFollowDist``-induced drift warning.
# They consume T8's committed fixtures (``ctps_gyro_heading.json`` /
# ``ctps_line_follow.json`` / ``ctps_reverse.json`` / ``ctps_wallstop_no_cm.json``)
# and build inline traces via ``load({...})`` where no fixture applies.
# ===========================================================================


def _inline_trace(*commands):
    """Build an in-memory Data Model A :class:`Trace` from raw command dicts.

    Mirrors the shape of the committed ``ctps_*.json`` fixtures (schema
    version 1, toy start pose) but keeps the command list inline so a case a
    fixture does not cover can be authored directly in the test body.

    :param commands: one or more raw command dicts (``{"seq", "op", ...}``).
    :return: the loaded :class:`Trace`.
    """
    return load(
        {
            "schema_version": 1,
            "start_node": "A",
            "start_heading_deg": 0.0,
            "build_error": False,
            "commands": list(commands),
        }
    )


def _heading_projection(prev, cur, heading_deg):
    """Return the signed advance of ``cur`` past ``prev`` projected onto a heading.

    Projects the ``(dx, dy)`` displacement between two keyframes onto the unit
    vector of ``heading_deg`` — POSITIVE means the chassis advanced forward
    along that heading, NEGATIVE means it moved backward. Direction-agnostic,
    so it reads the same whether the toy seed faces ``+x`` or ``+y``.

    :param prev: earlier keyframe dict (``{"x", "y", ...}``).
    :param cur: later keyframe dict.
    :param heading_deg: heading to project onto (deg, ``0 = +x``, CCW positive).
    :return: signed distance in centimetres.
    """
    from math import cos, radians, sin

    rad = radians(heading_deg)
    return (cur["x"] - prev["x"]) * cos(rad) + (cur["y"] - prev["y"]) * sin(rad)


def _planar_distance(prev, cur):
    """Return the straight-line distance (cm) between two keyframe poses."""
    from math import hypot

    return hypot(cur["x"] - prev["x"], cur["y"] - prev["y"])


# ---------------------------------------------------------------------------
# NC9 — GYRO + heading driveDist renders exactly three keyframes
# ---------------------------------------------------------------------------


def test_nc9_gyro_heading_drive_dist_renders_two_phase_rotate_then_advance():
    """NC9 (AC3): a single GYRO+heading ``driveDist`` renders rotate then advance.

    ``driveDist(35).setMode(GYRO).setHeading(90)`` renders two-phase: seed ->
    rotate-in-place to the held heading (a marker keyframe at the UNCHANGED
    position) -> a DENSIFIED advance along that heading. Post-#130 the advance
    is a run of profile-timed sub-keyframes, so the count is no longer exactly
    three — but the two-phase structure survives: the second keyframe is the
    rotation marker at the seed position facing the held heading, and the FINAL
    keyframe is the 35 cm advance.

    The toy chassis seeds facing ``START_HEADING_DEG == 90`` (node ``S``), so
    the turn-to-90 is a no-op rotation here; it still emits its marker keyframe.
    The advance runs along heading 90 (``0deg = +x``, ``90deg = +y``),
    displacing ``+y`` by 35 and ending facing 90.
    """
    track = _build_quiet(
        _inline_trace(
            {
                "seq": 0,
                "op": "drive",
                "method": "driveDist",
                "cm": 35.0,
                "mode": "gyro",
                "heading": 90.0,
            }
        ),
        toy_field_data,
    )

    seed = track.keyframes[0]
    rotation = track.keyframes[1]
    translation = track.keyframes[-1]

    # Seed sits at the start pose at t = 0.
    assert seed["t"] == 0.0

    # Rotation-phase marker: UNCHANGED position, heading turned to the held 90.
    assert rotation["x"] == pytest.approx(seed["x"], abs=_POSE_TOL)
    assert rotation["y"] == pytest.approx(seed["y"], abs=_POSE_TOL)
    assert rotation["h"] == pytest.approx(90.0, abs=_POSE_TOL)

    # Translation phase (final keyframe): displaced along +y by 35, facing 90.
    assert translation["h"] == pytest.approx(90.0, abs=_POSE_TOL)
    assert translation["x"] == pytest.approx(rotation["x"], abs=_POSE_TOL)
    assert translation["y"] - rotation["y"] == pytest.approx(35.0, abs=_POSE_TOL)
    assert _planar_distance(rotation, translation) == pytest.approx(
        35.0, abs=_POSE_TOL
    )

    # Timestamps monotonic non-decreasing.
    _assert_monotone([kf["t"] for kf in track.keyframes])


def test_nc9_gyro_fixture_adds_one_rotation_keyframe_per_gyro_move():
    """NC9 (AC3, broader): the gyro fixture renders one extra keyframe per move.

    ``ctps_gyro_heading.json`` chains two GYRO+heading moves (a ``driveDist``
    and a ``stopAtLine``) around one plain ``driveDist``. The two-phase render
    inserts one extra rotation keyframe per gyro move, so it produces exactly
    two MORE keyframes than the same three commands stripped of mode/heading —
    proof that GYRO renders two-phase without pinning brittle poses.
    """
    gyro = _build_quiet(
        load(_FIXTURES / "ctps_gyro_heading.json"), toy_field_data
    )
    non_gyro = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "drive", "method": "driveDist", "cm": 35.0},
            {"seq": 1, "op": "drive", "method": "driveDist", "cm": 20.0},
            {"seq": 2, "op": "drive", "method": "stopAtLine", "cm": 12.0},
        ),
        toy_field_data,
    )

    # Two gyro moves -> two extra rotation keyframes over the non-gyro variant.
    assert len(gyro.keyframes) == len(non_gyro.keyframes) + 2

    # Monotonic clock, and all three moves surface as drive events.
    times = [kf["t"] for kf in gyro.keyframes]
    assert all(b >= a for a, b in zip(times, times[1:]))
    assert sum(1 for ev in gyro.events if ev["type"] == "drive") == 3


# ---------------------------------------------------------------------------
# NC10 — a heading WITHOUT gyro is ignored (firmware parity)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "extra, heading",
    [
        ({"mode": "sync"}, 90.0),   # explicit non-gyro mode
        ({}, 90.0),                 # mode absent entirely
        ({"mode": "sync"}, 0.0),    # heading differs from facing -> still ignored
    ],
    ids=["mode-sync-h90", "mode-absent-h90", "mode-sync-h0-differs"],
)
def test_nc10_non_gyro_heading_is_ignored(extra, heading):
    """NC10 (AC4): a heading without ``mode == "gyro"`` renders along the facing.

    ``driveDist(35).setHeading(H)`` in SYNC (or with no explicit mode) must NOT
    turn in place — firmware consults ``setHeading`` only under GYRO. The move
    advances along the chassis's CURRENT facing and renders exactly TWO
    keyframes (seed + single advance), byte-identical to a plain
    ``driveDist(35)`` with no heading at all, and NOT the three-keyframe gyro
    two-phase (NC9).

    The decisive parity check is the byte-identical comparison to the plain
    move: if the heading were honoured the geometry would diverge (most
    visibly in the ``heading=0`` case, whose held heading differs from the toy
    seed facing 90).
    """
    track = _build_quiet(
        _inline_trace(
            {
                "seq": 0,
                "op": "drive",
                "method": "driveDist",
                "cm": 35.0,
                "heading": heading,
                **extra,
            }
        ),
        toy_field_data,
    )

    # No rotation phase: every keyframe stays on the seed facing (a gyro
    # two-phase would insert a rotation marker turning to the held heading).
    seed, moved = track.keyframes[0], track.keyframes[-1]
    assert all(
        kf["h"] == pytest.approx(seed["h"], abs=_POSE_TOL) for kf in track.keyframes
    )

    # Advance runs along the CURRENT facing (seed heading) by 35 cm, regardless
    # of the recorded (ignored) heading value.
    assert _heading_projection(seed, moved, seed["h"]) == pytest.approx(
        35.0, abs=_POSE_TOL
    )

    # Decisive firmware-parity check: identical densified geometry to a plain
    # move with no heading at all -> the recorded heading changed nothing (same
    # distance + same resolved speed -> byte-identical densification).
    plain = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "drive", "method": "driveDist", "cm": 35.0}
        ),
        toy_field_data,
    )
    assert len(track.keyframes) == len(plain.keyframes)
    for got, ref in zip(track.keyframes, plain.keyframes):
        assert got["x"] == pytest.approx(ref["x"], abs=_POSE_TOL)
        assert got["y"] == pytest.approx(ref["y"], abs=_POSE_TOL)
        assert got["h"] == pytest.approx(ref["h"], abs=_POSE_TOL)


# ---------------------------------------------------------------------------
# NC11 — lineFollowDist renders a forward advance + a drive event
# ---------------------------------------------------------------------------


def test_nc11_line_follow_dist_renders_forward_advance():
    """NC11 (AC5): ``lineFollowDist`` advances forward and emits a drive event.

    An inline ``lineFollowDist(90)`` advances the shared chassis forward along
    its current facing by 90 cm — one geometry keyframe past the seed — and the
    follow surfaces as a ``drive`` timeline event. ``lineFollowDist`` is
    excluded from the gyro two-phase set (the line follower regulates its own
    heading), so a plain forward push is the whole render.
    """
    track = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "drive", "method": "lineFollowDist", "cm": 90.0}
        ),
        toy_field_data,
    )

    # Seed + a densified forward advance (final keyframe is the 90 cm landing).
    assert len(track.keyframes) >= 2
    seed, moved = track.keyframes[0], track.keyframes[-1]

    assert moved["h"] == pytest.approx(seed["h"], abs=_POSE_TOL)  # no turn
    assert _heading_projection(seed, moved, seed["h"]) == pytest.approx(
        90.0, abs=_POSE_TOL
    )
    assert _planar_distance(seed, moved) == pytest.approx(90.0, abs=_POSE_TOL)

    # The follow surfaces as a drive-type event.
    assert any(ev["type"] == "drive" for ev in track.events)


def test_nc11_line_follow_fixture_forward_leg_advances():
    """NC11 (AC5): the committed ``ctps_line_follow.json`` forward leg advances.

    Consumes T8's ``ctps_line_follow.json`` (a forward follow then a reverse
    follow): its first ``lineFollowDist(60)`` advances the chassis forward
    along the current facing, and both follows surface as drive events.
    """
    track = _build_quiet(
        load(_FIXTURES / "ctps_line_follow.json"), toy_field_data
    )

    # Densified: the forward leg is a run of sub-keyframes; its landing is the
    # peak forward projection (the reverse follow then walks it back), which
    # equals the recorded 60 cm.
    seed = track.keyframes[0]
    peak = max(
        _heading_projection(seed, kf, seed["h"]) for kf in track.keyframes[1:]
    )
    assert peak == pytest.approx(60.0, abs=_POSE_TOL)

    drive_events = [ev for ev in track.events if ev["type"] == "drive"]
    assert len(drive_events) == 2


# ---------------------------------------------------------------------------
# NC12 — reverse moves backward; the plain move moves forward (contrast)
# ---------------------------------------------------------------------------


def test_nc12_reverse_moves_backward_forward_moves_forward():
    """NC12 (AC6): reversed follows/stops back up; the plain ``driveDist`` advances.

    ``ctps_reverse.json`` chains a reversed ``lineFollowDist`` / ``stopAtLine``
    / ``wallStop`` followed by a plain forward ``driveDist``. Each reversed move
    displaces the chassis BACKWARD along its (unchanged) heading — a NEGATIVE
    projection onto the facing — while the trailing ``driveDist`` displaces it
    FORWARD, for contrast. Magnitudes equal the recorded ``cm``.
    """
    from math import cos, radians, sin

    track = _build_quiet(load(_FIXTURES / "ctps_reverse.json"), toy_field_data)

    kfs = track.keyframes
    seed = kfs[0]
    heading = seed["h"]

    # Heading never changes — reverse translates, it never rotates.
    assert all(kf["h"] == pytest.approx(heading, abs=_POSE_TOL) for kf in kfs)

    # Each command's LANDING pose (densified: the last sub-keyframe of that
    # move) sits at the cumulative signed projection from the seed. Signed
    # per-move deltas: -40, -15, -10 (reversed), then +20 (forward). Build the
    # ordered boundary poses off the seed and confirm each appears byte-exact.
    ux, uy = cos(radians(heading)), sin(radians(heading))
    cumulative = 0.0
    boundary_poses = []
    for delta in (-40.0, -15.0, -10.0, 20.0):
        cumulative += delta
        boundary_poses.append(
            (seed["x"] + cumulative * ux, seed["y"] + cumulative * uy, heading)
        )
    landings = _boundary_indices(kfs, boundary_poses)

    # Per-move signed projection deltas between consecutive landings.
    prev_idx = 0
    for landing_idx, delta in zip(landings, (-40.0, -15.0, -10.0, 20.0)):
        projection = _heading_projection(kfs[prev_idx], kfs[landing_idx], heading)
        assert projection == pytest.approx(delta, abs=_POSE_TOL)
        prev_idx = landing_idx

    # The three reversed drive events carry the "reverse" token; the last does not.
    drive_labels = [ev["label"] for ev in track.events if ev["type"] == "drive"]
    assert len(drive_labels) == 4
    assert all("reverse" in label for label in drive_labels[:3])
    assert "reverse" not in drive_labels[3]


# ---------------------------------------------------------------------------
# W1 — a negative `.setPower` driveDist backs up (signed power = reverse)
# ---------------------------------------------------------------------------


def test_w1_negative_power_drive_dist_moves_backward_positive_moves_forward():
    """A raw ``driveDist(20).setPower(-70)`` backs the chassis up along its facing.

    ``DriveDistBuilder::setPower`` takes a signed ``[-100, 100]`` with NO
    ``setReverse`` — for a raw dead-reckon ``driveDist`` a negative power is the
    reverse mechanism (the firmware drives a negative cruise backward). So the
    render must move the chassis BACKWARD (a NEGATIVE projection onto the
    unchanged heading) with the SAME magnitude as the recorded ``cm``. The old
    ``abs()`` resolver rendered this as a forward move and integrated the
    operator offset the wrong way. A positive power still advances FORWARD (no
    regression). Neither move rotates — the facing is unchanged throughout.
    """
    track = _build_quiet(
        _inline_trace(
            {
                "seq": 0,
                "op": "drive",
                "method": "driveDist",
                "cm": 20.0,
                "power": -70,
            },
            {
                "seq": 1,
                "op": "drive",
                "method": "driveDist",
                "cm": 15.0,
                "power": 70,
            },
        ),
        toy_field_data,
    )

    kfs = track.keyframes
    seed = kfs[0]
    heading = seed["h"]

    # Pure translation — a signed-power driveDist never rotates.
    assert all(kf["h"] == pytest.approx(heading, abs=_POSE_TOL) for kf in kfs)

    # Boundary poses off the seed: the -70 move backs up 20 cm (signed -20),
    # then the +70 move advances 15 cm (signed +15), both along the facing.
    from math import cos, radians, sin

    ux, uy = cos(radians(heading)), sin(radians(heading))
    cumulative = 0.0
    boundary_poses = []
    for delta in (-20.0, 15.0):
        cumulative += delta
        boundary_poses.append(
            (seed["x"] + cumulative * ux, seed["y"] + cumulative * uy, heading)
        )
    landings = _boundary_indices(kfs, boundary_poses)

    # Per-move signed projection: the -70 move is NEGATIVE (backward), the +70
    # move is POSITIVE (forward). Magnitudes equal the recorded cm.
    prev_idx = 0
    for landing_idx, delta in zip(landings, (-20.0, 15.0)):
        projection = _heading_projection(kfs[prev_idx], kfs[landing_idx], heading)
        assert projection == pytest.approx(delta, abs=_POSE_TOL)
        prev_idx = landing_idx

    # The first move (negative power) moved backward — assert the sign directly
    # so a re-introduced abs() (rendering it forward) fails loudly here.
    first_landing_proj = _heading_projection(seed, kfs[landings[0]], heading)
    assert first_landing_proj < 0.0

    _assert_monotone([kf["t"] for kf in kfs])


# ---------------------------------------------------------------------------
# NC13 — GYRO composes with reverse; honoured on all forward-geometry methods
# ---------------------------------------------------------------------------


def test_nc13_gyro_reverse_stop_at_line_rotates_then_backs_up():
    """NC13 (AC3+AC6): a GYRO + reverse ``stopAtLine`` rotates then backs up along H.

    ``stopAtLine(15).setMode(GYRO).setHeading(45).setReverse(true)`` from the
    toy seed (facing 90) renders two-phase: rotate in place to 45 (a keyframe at
    the UNCHANGED position), THEN back up along 45 (``chassis.reverse``) — a
    NEGATIVE projection onto heading 45. Exactly three keyframes. Heading 45 is
    chosen distinct from the seed facing so the rotation phase is genuinely
    visible.
    """
    track = _build_quiet(
        _inline_trace(
            {
                "seq": 0,
                "op": "drive",
                "method": "stopAtLine",
                "cm": 15.0,
                "mode": "gyro",
                "heading": 45.0,
                "reverse": True,
            }
        ),
        toy_field_data,
    )

    seed = track.keyframes[0]
    translation = track.keyframes[-1]
    # Rotation-end: the densified rotate-in-place keeps the seed position while
    # sweeping heading to 45; the last keyframe still AT the seed position is
    # the rotation endpoint (the advance then leaves that position).
    rotation = seed
    for kf in track.keyframes:
        if _planar_distance(seed, kf) <= _POSE_TOL:
            rotation = kf
        else:
            break

    # Rotation phase: UNCHANGED position, heading turned to the held 45.
    assert rotation["x"] == pytest.approx(seed["x"], abs=_POSE_TOL)
    assert rotation["y"] == pytest.approx(seed["y"], abs=_POSE_TOL)
    assert rotation["h"] == pytest.approx(45.0, abs=_POSE_TOL)

    # Translation phase: BACKWARD along 45 (negative projection), still facing 45.
    assert translation["h"] == pytest.approx(45.0, abs=_POSE_TOL)
    assert _heading_projection(rotation, translation, 45.0) == pytest.approx(
        -15.0, abs=_POSE_TOL
    )
    assert _planar_distance(rotation, translation) == pytest.approx(
        15.0, abs=_POSE_TOL
    )


@pytest.mark.parametrize("method", ["driveDist", "stopAtLine", "wallStop"])
def test_nc13_gyro_heading_honoured_on_all_forward_geometry_methods(method):
    """NC13 (AC3): GYRO+heading is honoured on driveDist, stopAtLine AND wallStop.

    Each of the three forward-geometry methods, given ``mode == "gyro"`` and an
    explicit heading, renders the two-phase rotate-then-advance from a fresh
    chassis: exactly three keyframes (seed -> rotation to 45 at unchanged
    position -> forward advance along 45). This pins that the gyro two-phase is
    not driveDist-only.
    """
    track = _build_quiet(
        _inline_trace(
            {
                "seq": 0,
                "op": "drive",
                "method": method,
                "cm": 15.0,
                "mode": "gyro",
                "heading": 45.0,
            }
        ),
        toy_field_data,
    )

    seed = track.keyframes[0]
    translation = track.keyframes[-1]
    # Rotation-end: last densified keyframe still at the seed position.
    rotation = seed
    for kf in track.keyframes:
        if _planar_distance(seed, kf) <= _POSE_TOL:
            rotation = kf
        else:
            break

    # Rotation to the held heading at the unchanged position.
    assert rotation["x"] == pytest.approx(seed["x"], abs=_POSE_TOL)
    assert rotation["y"] == pytest.approx(seed["y"], abs=_POSE_TOL)
    assert rotation["h"] == pytest.approx(45.0, abs=_POSE_TOL)

    # Forward advance along 45 (positive projection), still facing 45.
    assert translation["h"] == pytest.approx(45.0, abs=_POSE_TOL)
    assert _heading_projection(rotation, translation, 45.0) == pytest.approx(
        15.0, abs=_POSE_TOL
    )


# ---------------------------------------------------------------------------
# NC14 — cm-less forward-geometry commands narrate only, never crash
# ---------------------------------------------------------------------------


def test_nc14_cm_less_forward_geometry_narrate_only_no_crash():
    """NC14 (AC7 negative): cm-less wallStop/lineFollowDist narrate, never crash.

    ``ctps_wallstop_no_cm.json`` holds a ``wallStop`` and a ``lineFollowDist``
    with NO ``cm`` (a legacy/partial trace). ``build`` must not raise (the
    ``cm is None`` guard defends ``float(None)``): both commands narrate as
    drive events but add NO geometry keyframe, so the keyframe list stays at
    just the seed and the clock never advances.
    """
    # Must not raise — the cm-is-None guard defends float(None).
    track = _build_quiet(
        load(_FIXTURES / "ctps_wallstop_no_cm.json"), toy_field_data
    )

    # Narrate-only: only the seed keyframe, no geometry added.
    assert len(track.keyframes) == 1
    assert track.keyframes[0]["t"] == 0.0

    # Both cm-less commands still surface as drive events.
    drive_events = [ev for ev in track.events if ev["type"] == "drive"]
    assert len(drive_events) == 2

    # Clock stays monotonic (and at zero — nothing advanced it).
    times = [kf["t"] for kf in track.keyframes]
    assert all(b >= a for a, b in zip(times, times[1:]))
    assert track.duration_s == pytest.approx(0.0, abs=_POSE_TOL)


# ---------------------------------------------------------------------------
# NC16 — a lineFollowDist offset before a goto renders a return maneuver
# ---------------------------------------------------------------------------


def test_nc16_line_follow_dist_before_goto_renders_return():
    """NC16 (T11): a ``lineFollowDist`` offset then goto routes through the return.

    Mirrors TC3's structure but with ``lineFollowDist`` as the raw move: a 15 cm
    distance-gated follow before ``goto B`` integrates the operator offset on the
    shared sim (issue #130, T11), and the following goto's return prologue drives
    it back — so a ``return_to_node`` marker is emitted and, because the goto
    re-anchors, NO residual ``drift_warn`` survives. Pins that a raw
    ``lineFollowDist`` participates in the operator-offset / return-to-node path
    exactly like ``driveDist``.
    """
    track = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "drive", "method": "lineFollowDist", "cm": 15.0},
            {"seq": 1, "op": "goto", "target": "B"},
        ),
        toy_field_data,
    )

    return_events = [ev for ev in track.events if ev["type"] == "return_to_node"]
    assert len(return_events) == 1
    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []


# ---------------------------------------------------------------------------
# T11 smoke — return_to_node / declare_position localization-lane markers
# (issue #130). These prove this task's own change; T13 owns the comprehensive
# TC9-TC11 track-level coverage + golden re-bless.
# ---------------------------------------------------------------------------


def test_t11_return_to_node_marker_shape_and_ordering():
    """The ``return_to_node`` marker sits inside its goto's span and names the node.

    A 15 cm raw drive then ``goto B`` renders the return prologue; the marker is
    a POINT event stamped at the goto's start (so ``events`` stays sorted), its
    ``type`` is exactly ``"return_to_node"`` (the string T12's ``buildDriftLane``
    branches on), and its label names the node the return re-anchored (the toy
    field's true start node ``S`` — ``build`` seeds from ``field_data.START_NODE``,
    not the trace's informational ``start_node``).
    """
    track = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "drive", "method": "driveDist", "cm": 15.0},
            {"seq": 1, "op": "goto", "target": "B"},
        ),
        toy_field_data,
    )

    return_events = [ev for ev in track.events if ev["type"] == "return_to_node"]
    assert len(return_events) == 1
    marker = return_events[0]
    assert "S" in marker["label"]

    # The marker is stamped at (== ) the goto span's start and inside its bracket.
    goto = next(ev for ev in track.events if ev["type"] == "goto")
    assert marker["t"] == pytest.approx(goto["t"], abs=_POSE_TOL)
    assert goto["t"] <= marker["t"] <= goto["t_end"]

    # Event timestamps remain non-decreasing overall.
    ev_times = [ev["t"] for ev in track.events]
    assert all(b >= a for a, b in zip(ev_times, ev_times[1:]))


def test_t11_declare_position_marker_and_node_change():
    """A ``declare_position`` op emits a declare marker and changes the sim node.

    The op is a pure belief reset: it emits exactly one localization-lane event
    of ``type == "declare_position"`` (the string T12 branches on) naming the
    declared node, advances no keyframe / clock, and — because it re-anchored the
    offset onto the (physically-unchanged) chassis — leaves no trace-end residual
    ``drift_warn``. A downstream ``goto`` then plans FROM the declared node,
    proving the belief actually moved.
    """
    track = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "declare_position", "node": "C"},
            {"seq": 1, "op": "goto", "target": "D"},
        ),
        toy_field_data,
    )

    declare_events = [
        ev for ev in track.events if ev["type"] == "declare_position"
    ]
    assert len(declare_events) == 1
    assert "C" in declare_events[0]["label"]

    # The following goto plans FROM the declared node C (not the start A).
    goto = next(ev for ev in track.events if ev["type"] == "goto")
    assert goto["label"].startswith("C->")

    # No spurious residual: the declare cleared the offset.
    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []


def test_t11_declare_position_unknown_node_emits_no_marker():
    """A ``declare_position`` to a node not in NODES emits NO marker (T7 no-op).

    Per T7's contract the sim warns + no-ops on an unknown node (node + offset
    unchanged), so the belief did not change and track_builder emits no
    localization-lane marker. A following ``goto`` still plans from the ORIGINAL
    start node ``S`` (the toy field's ``START_NODE``), proving the declare was
    skipped.
    """
    track = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "declare_position", "node": "NOPE"},
            {"seq": 1, "op": "goto", "target": "B"},
        ),
        toy_field_data,
    )

    assert [
        ev for ev in track.events if ev["type"] == "declare_position"
    ] == []
    goto = next(ev for ev in track.events if ev["type"] == "goto")
    assert goto["label"].startswith("S->")


def test_t11_clean_two_goto_trace_has_no_localization_markers():
    """A clean (raw-move-free / declare-free) trace carries none of T11's markers.

    Byte-invariance guard: without a raw move or a declare op, the trace emits no
    ``return_to_node`` / ``declare_position`` markers and no residual
    ``drift_warn`` — its events are exactly the pre-#130 (T10) set. Locks that
    T11 is purely additive on the affected ops.
    """
    trace = load(_FIXTURES / "ctps_two_goto.json")
    track = _build_quiet(trace, toy_field_data)

    for kind in ("return_to_node", "declare_position", "drift_warn"):
        assert [ev for ev in track.events if ev["type"] == kind] == []


# ---------------------------------------------------------------------------
# goto forced-direction override reaches the sim (setArrivalDirection /
# setRouteDirection carried end-to-end)
# ---------------------------------------------------------------------------


def test_goto_direction_tags_forwarded_to_step_through(monkeypatch):
    """A goto's ``arrival``/``route`` tags reach ``sim.step_through`` as kwargs.

    The final Python stage of the CTPS direction-override pipeline: ``build``
    must thread the loaded ``front``/``rear`` tags into ``step_through`` as
    ``arrival_direction`` / ``route_direction``. Without this the sim falls back
    to AUTO and silently ignores the operator's ``setArrivalDirection`` — the
    original bug. Spying ``step_through`` isolates the forwarding from the sim's
    own resolution (covered by the direction-parity suite).
    """
    from tools.path_planner.sim.runtime import Direction

    captured: list = []

    def _spy(
        self,
        goal_name,
        *,
        route_direction=Direction.AUTO,
        arrival_direction=Direction.AUTO,
    ):
        captured.append((goal_name, route_direction, arrival_direction))
        return iter(())  # no primitives — build still emits the goto span event

    monkeypatch.setattr(PathTrackerSim, "step_through", _spy)

    _build_quiet(
        _inline_trace(
            {
                "seq": 0,
                "op": "goto",
                "target": "B",
                "arrival": "front",
                "route": "rear",
            }
        ),
        toy_field_data,
    )

    # arrival "front" -> Direction.FRONT (final hop); route "rear" -> Direction.REAR.
    assert captured == [("B", Direction.REAR, Direction.FRONT)]


def test_goto_without_direction_tags_defaults_to_auto(monkeypatch):
    """A direction-free goto forwards ``Direction.AUTO`` for both kwargs.

    Backward-compat guard: an existing trace with no ``arrival``/``route`` keys
    must reach ``step_through`` exactly as before the schema change (AUTO), so
    direction-free traces render identically.
    """
    from tools.path_planner.sim.runtime import Direction

    captured: list = []

    def _spy(
        self,
        goal_name,
        *,
        route_direction=Direction.AUTO,
        arrival_direction=Direction.AUTO,
    ):
        captured.append((goal_name, route_direction, arrival_direction))
        return iter(())

    monkeypatch.setattr(PathTrackerSim, "step_through", _spy)

    _build_quiet(
        _inline_trace({"seq": 0, "op": "goto", "target": "B"}),
        toy_field_data,
    )

    assert captured == [("B", Direction.AUTO, Direction.AUTO)]


# ===========================================================================
# Pivot-arc tests (CTPS issue #55, T5 — TC1-TC7, TC11, TC12, TC13)
#
# These pin the single-wheel pivot-arc rendering + chassis-nudge (T2): dense
# arc keyframes tracing the rigid rotation about the planted wheel, the
# emitted ``pivots`` Track segment, the on-node landing for BOTH a
# distance-terminated pivot departure (TC1/TC4/TC5) and a sensor-terminated
# one (TC12, plus the chained-sensor TC13), the pivot-free no-``pivots``-key
# guard (AC6, TC6), and the primitive-stream-unchanged parity tripwire
# (AC7, TC7).
#
# Test data: the existing per-turn pivot fixtures in
# ``tests/fixtures/pivot_field_data.py`` (ADR-036), replayed through a
# synthetic single-``goto`` Trace built directly via ``_goto_trace`` below (no
# JSON, no C++ dump). TC12 uses a NEW fixture, ``make_line_departing_pivot_field``
# (added to that module by this task): a pivot whose DEPARTURE is a LINE edge
# into a line node — the mirror image of the existing LINE-incoming fixture.
# TC13 uses ``make_chained_sensor_departure_pivot_field``: two pivots whose
# SHARED middle leg is that same LINE edge, so the sensor departure lands on a
# node that itself carries an approaching-pivot trim.
# ===========================================================================


def _goto_trace(field_data, target: str) -> Trace:
    """Build a synthetic single-``goto`` Trace targeting ``target``.

    The pivot fixtures are hand-built ``SimpleNamespace`` field-data (ADR-036),
    not real trace JSON, so this constructs the Data Model A ``Trace`` /
    ``Command`` dataclasses directly rather than routing through
    :func:`tools.path_planner.ctps.trace_loader.load`. ``start_node`` /
    ``start_heading_deg`` are informational only — :func:`build` always seeds
    the chassis from ``field_data``, never from the ``Trace``.

    :param field_data: a pivot fixture field (or any field-data namespace).
    :param target: the single goto's destination node name.
    :return: a one-command :class:`Trace`.
    """
    return Trace(
        schema_version=1,
        start_node=field_data.START_NODE,
        start_heading_deg=field_data.START_HEADING_DEG,
        build_error=False,
        commands=[Command(seq=0, op="goto", target=target)],
    )


def _rotate_vector(dx: float, dy: float, ang_rad: float) -> tuple[float, float]:
    """Rotate the 2D vector ``(dx, dy)`` by ``ang_rad`` (radians, CCW positive)."""
    from math import cos, sin

    return (
        dx * cos(ang_rad) - dy * sin(ang_rad),
        dx * sin(ang_rad) + dy * cos(ang_rad),
    )


def _expected_arc_centers(
    x0: float,
    y0: float,
    h0_deg: float,
    theta_deg: float,
    wheel_sign: int,
    radius_cm: float,
    n: int,
):
    """Independently recompute the pivot-arc centres from the Contracts formula.

    Deliberately REIMPLEMENTS the geometry here rather than importing
    ``track_builder._build_pivot_arc`` — this pins the DOCUMENTED contract
    (the CTPS-55 plan's "Arc geometry" section:
    ``center_k = W + Rot(tau*theta) * (c0 - W)``) rather than tautologically
    re-checking the same private helper it exists to verify.

    :param x0: arc-start chassis-centre x (cm).
    :param y0: arc-start chassis-centre y (cm).
    :param h0_deg: incoming heading (deg).
    :param theta_deg: signed swing angle (deg, CCW positive).
    :param wheel_sign: ``+1`` left-wheel pivot, ``-1`` right-wheel pivot.
    :param radius_cm: pivot radius (``track_cm / 2``).
    :param n: number of arc sub-keyframes.
    :return: ``((wx, wy), [(cx_1, cy_1), ..., (cx_n, cy_n)])`` — the planted
        wheel centre and the N expected chassis-centre samples, ``k = 1..n``.
    """
    from math import cos, radians, sin

    wx = x0 + radius_cm * cos(radians(h0_deg + wheel_sign * 90.0))
    wy = y0 + radius_cm * sin(radians(h0_deg + wheel_sign * 90.0))
    rx, ry = x0 - wx, y0 - wy

    centers = []
    for k in range(1, n + 1):
        tau = k / n
        drx, dry = _rotate_vector(rx, ry, radians(tau * theta_deg))
        centers.append((wx + drx, wy + dry))
    return (wx, wy), centers


def _dead_reckon_pivot_track():
    """Build the Track for a single ``goto("W")`` over the 90 deg dead-reckon pivot.

    Shared by TC1-TC3, which all assert on the SAME goto (on-node landing, the
    arc sub-keyframes, and the emitted ``pivots`` segment, respectively).

    :return: ``(field, track)``.
    """
    field = pivot_field_data.make_dead_reckon_pivot_field()
    track = _build_quiet(_goto_trace(field, "W"), field)
    return field, track


# ---------------------------------------------------------------------------
# TC1 — 90 deg dead-reckon pivot: lands on W, no drift_warn
# ---------------------------------------------------------------------------


def test_tc1_dead_reckon_pivot_lands_on_node_no_drift_warn():
    """TC1 (AC2): the 90 deg dead-reckon pivot goto lands on W with NO drift_warn.

    Regression guard for the pre-#55 in-place-turn drift (12.7 cm + a spurious
    ``drift_warn``): the chassis-nudge (T2) lands the chassis natively on the
    node for a DISTANCE-terminated pivot departure.
    """
    field, track = _dead_reckon_pivot_track()

    wx, wy = field.NODES["W"]
    last = track.keyframes[-1]
    drift = hypot(last["x"] - wx, last["y"] - wy)
    assert drift < _POSE_TOL

    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []
    assert len(track.pivots) == 1


# ---------------------------------------------------------------------------
# TC2 — the arc sub-keyframes trace the rotation about the planted wheel
# ---------------------------------------------------------------------------


def test_tc2_arc_subkeyframes_trace_rotation_evenly_timed():
    """TC2 (AC1, AC4): the pivot's arc sub-keyframes trace the rigid rotation.

    The pivot arc's GEOMETRY is unchanged from pre-#130 (``N = ceil(90 /
    ARC_STEP_DEG)`` sub-keyframes tracing the rigid rotation about the planted
    wheel); only the TIMING now comes from the firmware turn profile. The
    approach and departure legs are ALSO densified now, so the arc is no longer
    at a fixed keyframe index — it is located via the emitted ``pivots``
    segment's ``[t0, t1]`` bracket. Each arc sample matches the documented
    Contracts formula; heading rotates through the full 90 deg; the arc spans
    exactly ``resolve_turn_profile(theta, is_pivot=True).duration_s()`` seconds,
    split into a fast cruise group then a slower slow-zone group (the two-stage
    turn, AC3); the running clock stays strictly monotonic.
    """
    from tools.path_planner.ctps.profile import resolve_turn_profile

    field, track = _dead_reckon_pivot_track()

    n = ceil(90.0 / ARC_STEP_DEG)
    segment = track.pivots[0]

    # Locate the arc via the pivots-segment time bracket: the arc keyframes are
    # the N with t0 < t <= t1; the approach-end (arc start) is the keyframe
    # immediately before the first arc keyframe.
    arc_start_idx = next(
        i for i, kf in enumerate(track.keyframes) if kf["t"] > segment["t0"]
    )
    approach_end = track.keyframes[arc_start_idx - 1]
    arc = track.keyframes[arc_start_idx : arc_start_idx + n]
    assert len(arc) == n
    assert approach_end["t"] == pytest.approx(segment["t0"], abs=1e-9)

    radius_cm = parse_track_cm_from_config() / 2.0
    _wheel_center, expected_centers = _expected_arc_centers(
        approach_end["x"],
        approach_end["y"],
        approach_end["h"],
        theta_deg=-90.0,
        wheel_sign=-1,   # a right turn (theta < 0) pivots about the RIGHT wheel
        radius_cm=radius_cm,
        n=n,
    )
    for kf, (ex, ey) in zip(arc, expected_centers):
        assert kf["x"] == pytest.approx(ex, abs=1e-6)
        assert kf["y"] == pytest.approx(ey, abs=1e-6)

    # Heading rotates through the full 90 deg (a right turn: 90 -> 0).
    assert approach_end["h"] == pytest.approx(90.0, abs=_POSE_TOL)
    assert arc[-1]["h"] == pytest.approx(0.0, abs=_POSE_TOL)

    # Two-stage timing across the N uniform-ANGLE sub-keyframes: the cruise-band
    # degrees sweep at the fast (50) rate and the final TURN_SLOW_ZONE_DEG degrees
    # at the snapped floor (35), so the per-step durations split into a short
    # cruise group then a longer slow group, in the inverse PWM ratio 50 / 35.
    turn = resolve_turn_profile(-90.0, is_pivot=True)
    n_slow = round(turn.slow_deg / ARC_STEP_DEG)
    n_cruise = n - n_slow
    assert 0 < n_cruise < n
    times = [approach_end["t"]] + [kf["t"] for kf in arc]
    steps = [b - a for a, b in zip(times, times[1:])]
    cruise_steps = steps[:n_cruise]
    slow_steps = steps[n_cruise:]
    for step in cruise_steps:
        assert step == pytest.approx(cruise_steps[0], abs=1e-9)
    for step in slow_steps:
        assert step == pytest.approx(slow_steps[0], abs=1e-9)
    assert slow_steps[0] / cruise_steps[0] == pytest.approx(50.0 / 35.0, rel=1e-6)

    # Total arc duration: the firmware pivot-turn profile duration.
    expected_dur = resolve_turn_profile(-90.0, is_pivot=True).duration_s()
    assert arc[-1]["t"] - approach_end["t"] == pytest.approx(expected_dur, abs=1e-9)
    assert segment["t1"] == pytest.approx(arc[-1]["t"], abs=1e-9)

    # The running clock stays monotonic non-decreasing across the whole list
    # (the terminal brake keyframe of a densified leg may equal its
    # predecessor's t, so this is non-strict).
    _assert_monotone([kf["t"] for kf in track.keyframes])


# ---------------------------------------------------------------------------
# TC3 — the emitted pivots segment has the right shape
# ---------------------------------------------------------------------------


def test_tc3_pivots_segment_shape_wheel_right_for_negative_90deg():
    """TC3 (AC4): one ``pivots`` segment; ``wheel == "right"`` for the -90 deg turn.

    ``theta_deg`` matches the ``ACT_TURN_REL`` delta (a RIGHT turn is
    negative); ``(cx, cy)`` is the planted wheel (the SAME centre TC2 already
    pinned); ``t0 < t1`` brackets the arc span.
    """
    field, track = _dead_reckon_pivot_track()

    assert len(track.pivots) == 1
    segment = track.pivots[0]

    assert segment["wheel"] == "right"
    assert segment["theta_deg"] == pytest.approx(-90.0, abs=1e-9)
    assert segment["t0"] < segment["t1"]

    # The approach-end (arc start) is the keyframe at the segment's t0; the
    # approach leg densifies now, so it is no longer keyframes[1].
    approach_end = next(
        kf for kf in track.keyframes if kf["t"] == pytest.approx(segment["t0"], abs=1e-9)
    )
    radius_cm = parse_track_cm_from_config() / 2.0
    (wx, wy), _centers = _expected_arc_centers(
        approach_end["x"],
        approach_end["y"],
        approach_end["h"],
        theta_deg=-90.0,
        wheel_sign=-1,
        radius_cm=radius_cm,
        n=1,
    )
    assert segment["cx"] == pytest.approx(wx, abs=1e-6)
    assert segment["cy"] == pytest.approx(wy, abs=1e-6)
    assert segment["t0"] == pytest.approx(approach_end["t"], abs=1e-9)


# ---------------------------------------------------------------------------
# TC4 — chained pivots (two turns in one goto) each animate; land on X
# ---------------------------------------------------------------------------


def test_tc4_chained_pivots_land_on_node_no_drift_warn():
    """TC4 (AC3): chained pivots U->V->W->X animate both arcs and land on X.

    Regression guard for the pre-#55 18 cm drift + ``drift_warn`` on the
    chained route (two 90 deg right turns, at V and at W).
    """
    field = pivot_field_data.make_chained_pivot_field()
    track = _build_quiet(_goto_trace(field, "X"), field)

    xx, xy = field.NODES["X"]
    last = track.keyframes[-1]
    drift = hypot(last["x"] - xx, last["y"] - xy)
    assert drift < _POSE_TOL

    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []

    assert len(track.pivots) == 2
    t0s = [segment["t0"] for segment in track.pivots]
    assert t0s == sorted(t0s)   # emitted in chronological order
    assert all(segment["wheel"] == "right" for segment in track.pivots)
    assert all(segment["t0"] < segment["t1"] for segment in track.pivots)


# ---------------------------------------------------------------------------
# TC5 — a 70 deg LINE-incoming pivot lands on its node
# ---------------------------------------------------------------------------


def test_tc5_line_incoming_70deg_pivot_lands_on_node():
    """TC5 (AC2): the 70 deg LINE-incoming pivot lands on L2; one left-wheel segment.

    ``make_line_incoming_pivot_field`` arrives at the pivot node L1 over a
    LINE edge (re-anchoring) and departs over a distance-terminated BLIND
    edge, so the chassis-nudge lands it on L2 exactly like TC1's dead-reckon
    case — the arc-start pose is simply whatever the LINE approach's keyframe
    holds.
    """
    field = pivot_field_data.make_line_incoming_pivot_field()
    track = _build_quiet(_goto_trace(field, "L2"), field)

    lx, ly = field.NODES["L2"]
    last = track.keyframes[-1]
    drift = hypot(last["x"] - lx, last["y"] - ly)
    assert drift < _POSE_TOL

    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []

    assert len(track.pivots) == 1
    segment = track.pivots[0]
    assert segment["wheel"] == "left"
    assert segment["theta_deg"] == pytest.approx(70.0, abs=1e-6)
    # An arc is present -- N >= 1 always (the Contracts N = max(1, ceil(...))).
    assert segment["t0"] < segment["t1"]


# ---------------------------------------------------------------------------
# TC6 — a pivot-free goto keeps the pre-#55 shape and omits "pivots" (AC6)
# ---------------------------------------------------------------------------


def test_tc6_pivot_free_goto_has_no_pivots_key():
    """TC6 (AC6): a pivot-free goto keeps the pre-#55 keyframe shape, no "pivots".

    Reuses the existing two-goto toy-field topology (TC1 above): keyframes
    still equal an independent ``step_through`` reference run, and neither
    ``Track.pivots`` nor ``to_dict()["pivots"]`` appear — the toy field's
    ``PIVOT_TURNS`` is empty (absent, even), so every pivot-detection lookup
    misses by construction.
    """
    trace = load(_FIXTURES / "ctps_two_goto.json")
    track = _build_quiet(trace, toy_field_data)

    goto_targets = [cmd.target for cmd in trace.commands if cmd.op == "goto"]
    reference = _reference_snapshots(toy_field_data, goto_targets)

    # Densified: every step_through boundary pose appears in order (the toy field
    # is pivot-free, so this is a plain LINE/BLIND densification with no arc).
    _boundary_indices(track.keyframes, reference)

    assert track.pivots == []
    assert "pivots" not in track.to_dict()


# ---------------------------------------------------------------------------
# TC7 — the chassis-nudge does not perturb the sim's primitive stream (AC7)
# ---------------------------------------------------------------------------


def test_tc7_chassis_nudge_does_not_perturb_primitive_stream():
    """TC7 (AC7 tripwire): mutating chassis.x/y mid-goto never perturbs dispatch.

    ``Track.build()`` discards primitives, so this drives
    :meth:`PathTrackerSim.step_through` DIRECTLY on the dead-reckon pivot
    fixture — the only way to inspect the primitive stream. Applies the SAME
    kind of mutation ``track_builder``'s chassis-nudge performs (a raw
    ``chassis.x_cm`` / ``chassis.y_cm`` write, never routed through
    ``LocalizationSim``) right after the pivot's ``ACT_TURN_REL`` primitive,
    well off the true pose so any dispatch dependence on position would
    visibly diverge the stream. Asserts the resulting primitive sequence is
    IDENTICAL to an unperturbed reference run — the Contracts claim that
    ``_walk_hops`` dispatches on ``heading_deg`` only (runtime.py:2693), never
    chassis x/y.
    """
    field = pivot_field_data.make_dead_reckon_pivot_field()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        graph, next_hop = pivot_field_data.build_graph_and_next_hop(field)

    def _run(mutate: bool):
        chassis = VirtualChassis.from_field_data(field)
        sim = PathTrackerSim(field, graph, next_hop, chassis)
        sim.localization.zero_heading_from_current()
        primitives = []
        for primitive, _snap in sim.step_through("W"):
            primitives.append(primitive)
            if mutate and primitive[0] == ACT_TURN_REL:
                # Mirror the nudge: a raw x_cm/y_cm write, well off the true
                # pose, so any dispatch dependence on position would visibly
                # diverge the primitive stream.
                chassis.x_cm += 37.0
                chassis.y_cm -= 23.0
        return primitives

    reference = _run(mutate=False)
    perturbed = _run(mutate=True)

    assert len(reference) == 3   # sanity: the fixture's known 3-primitive shape
    assert perturbed == reference


# ---------------------------------------------------------------------------
# TC11 — ARC_STEP_DEG is present and positive
# ---------------------------------------------------------------------------


def test_tc11_arc_step_deg_present_and_positive():
    """TC11: ``ctps/config.py`` exposes ``ARC_STEP_DEG > 0``."""
    assert ARC_STEP_DEG > 0


# ---------------------------------------------------------------------------
# TC12 — a LINE-departing pivot (sensor-terminated departure) lands on node
# ---------------------------------------------------------------------------


def test_tc12_line_departing_pivot_lands_on_destination_node_no_drift_warn():
    """TC12 (AC2 sensor-departure): a LINE-departing pivot lands on the node.

    ``make_line_departing_pivot_field`` declares a pivot whose DEPARTURE leg
    is a LINE edge into a line node — sensor-terminated. The sim takes NO
    departing trim on that hop (a LINE-outgoing edge always drives its full
    commanded distance and re-anchors via the LINE_OVERSHOOT step), so it
    would otherwise dead-reckon ``delta`` PAST the node along the departure
    edge; ``track_builder`` pulls the landing back by the departing pivot's
    ``trim_cm`` along that edge ray, which lands exactly ON the node here (the
    real robot's along-edge line re-anchor, ADR-044 D1).
    """
    from tools.path_planner.validator import validate

    field = pivot_field_data.make_line_departing_pivot_field()
    validate(field)   # must pass -- this task owns the fixture's legality

    track = _build_quiet(_goto_trace(field, "Q"), field)

    qx, qy = field.NODES["Q"]
    last = track.keyframes[-1]
    drift = hypot(last["x"] - qx, last["y"] - qy)
    assert drift < _POSE_TOL

    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []

    assert len(track.pivots) == 1
    assert track.pivots[0]["wheel"] == "right"


# ---------------------------------------------------------------------------
# TC13 — chained pivots over a LINE (sensor) middle leg land on the final node
# ---------------------------------------------------------------------------


def test_tc13_chained_sensor_departure_pivots_land_on_final_node():
    """TC13 (AC3 sensor middle leg): chained pivots over a LINE leg land on X.

    ``make_chained_sensor_departure_pivot_field`` chains two 90 deg pivots
    whose SHARED middle leg ``P -> Q`` is a LINE edge: pivot 1's departure AND
    pivot 2's approach are both sensor-terminated. The sim already shortens
    that leg's landing by pivot 2's approach trim, so the robot physically
    lands ``delta_Q`` SHORT of Q — the true arc-start pose for pivot 2. The
    sensor-departure correction must therefore subtract the DEPARTING trim
    along the departure-edge ray (landing ``delta_Q`` short of Q here) rather
    than snap absolutely onto Q's coordinates: the unconditional snap started
    pivot 2's arc ``delta_Q`` ahead of the true pivot point and translated the
    whole tail 9 cm off X, silently (under the 10 cm drift threshold).
    """
    from tools.path_planner.validator import validate

    field = pivot_field_data.make_chained_sensor_departure_pivot_field()
    validate(field)   # must pass -- this task owns the fixture's legality

    track = _build_quiet(_goto_trace(field, "X"), field)

    xx, xy = field.NODES["X"]
    last = track.keyframes[-1]
    drift = hypot(last["x"] - xx, last["y"] - xy)
    assert drift < _POSE_TOL

    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []

    assert len(track.pivots) == 2
    t0s_chained_sensor = [segment["t0"] for segment in track.pivots]
    assert t0s_chained_sensor == sorted(t0s_chained_sensor)   # emitted in chronological order
    assert all(segment["wheel"] == "right" for segment in track.pivots)


# ===========================================================================
# T13 rebaseline: TC7 (boundary-pose invariant), TC8-scaffold (all-hops drift
# fixture), TC9-TC11 (return-to-node / declare_position track-level coverage)
# ===========================================================================


def _track_hash(track) -> str:
    """Return a stable sha256 hex digest of ``track``'s JSON-serialisable dict.

    Sorts keys so field/dict ordering never perturbs the digest; floats
    serialise via ``repr`` (Python's ``json`` module default), which is
    deterministic for identical float values — sufficient for an
    identical-build-vs-identical-build comparison (never cross-platform text
    diffing). Used by TC11 to prove a ``declarePosition``-free trace and its
    unknown-node-declare counterpart render BYTE-IDENTICAL tracks.

    :param track: a built :class:`~tools.path_planner.ctps.track_builder.Track`.
    :return: the hex sha256 digest of ``json.dumps(track.to_dict(), sort_keys=True)``.
    """
    payload = json.dumps(track.to_dict(), sort_keys=True).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


# ---------------------------------------------------------------------------
# TC7 — GENERAL boundary-pose invariant (AC10)
# ---------------------------------------------------------------------------
#
# Scoped to PIVOT-FREE fixtures: a pivot's chassis-nudge (`_build_pivot_arc`'s
# caller) deliberately re-poses the SHARED sim's chassis before its NEXT
# primitive dispatches -- that is the whole point of the nudge (killing the
# ~12.7 cm in-place-turn drift, AC2). So an INDEPENDENTLY constructed
# reference sim (which never receives that nudge) legitimately diverges from
# the built Track's keyframes on a pivot fixture -- not a bug, the documented
# TC7 tripwire in `_build_pivot_arc`'s own docstring covers that case
# differently (the primitive-STREAM-unchanged assertion, `test_tc7_...`
# above, in the pivot-arc test group). The pivot suite (TC1-TC13 above)
# already pins pivot correctness via node-landing + arc-geometry assertions
# instead of a step_through cross-check.
_TC7_FIXTURES: list[tuple[str, object, list[str]]] = [
    ("ctps_two_goto.json", toy_field_data, ["B", "C"]),
    (
        "ctps_all_hops_drift.json",
        toy_field_data,
        ["P1", "B", "S", "A", "P1", "C", "G", "B", "S"],
    ),
]


@pytest.mark.parametrize(
    "fixture_name, field_data, goto_targets",
    _TC7_FIXTURES,
    ids=[name for name, _fd, _targets in _TC7_FIXTURES],
)
def test_tc7_boundary_pose_invariant_general(fixture_name, field_data, goto_targets):
    """TC7 (AC10): every primitive's boundary pose byte-equals a keyframe, in order.

    A GENERAL invariant test over multiple fixtures (not a one-off): for each
    fixture, an INDEPENDENT ``step_through`` reference replay (fresh chassis +
    sim, never touched by ``build``) yields the ordered sequence of primitive
    boundary poses; every one of those poses must appear BYTE-EXACT among the
    built ``Track``'s keyframes, in the same order (:func:`_boundary_indices`
    asserts both — a miss raises, and the match cursor only moves forward).
    Keyframe timestamps are asserted NON-DECREASING (AC10's other half) — a
    terminal instantaneous-brake keyframe may equal its predecessor's ``t``,
    so this is the non-strict check, matching ``player.js:110``'s zero-length
    bracket tolerance.
    """
    trace = load(_FIXTURES / fixture_name)
    track = _build_quiet(trace, field_data)

    actual_targets = [cmd.target for cmd in trace.commands if cmd.op == "goto"]
    assert actual_targets == goto_targets

    reference = _reference_snapshots(field_data, goto_targets)

    # Every reference boundary pose appears byte-exact, in order (raises on a
    # miss or an out-of-order match).
    _boundary_indices(track.keyframes, reference)

    # Timestamps are non-decreasing across the whole densified keyframe list.
    _assert_monotone([kf["t"] for kf in track.keyframes])


# ---------------------------------------------------------------------------
# TC8-scaffold — the all-hop-types fixture: drift ~= 0 at every goto arrival
# ---------------------------------------------------------------------------


def test_tc8_scaffold_all_hop_types_drift_sweep():
    """TC8-scaffold (AC5): ``ctps_all_hops_drift.json`` lands drift ~= 0 everywhere.

    Drives the new all-hop-types fixture (a 9-goto tour of the toy field graph
    starting from its true ``START_NODE`` "S") and asserts, at EVERY ``goto``
    arrival, the chassis lands on its target node to float-noise precision —
    the sim chassis is the wheel axle and a clean goto (any hop type) lands
    drift ~= 0 regardless of the final hop's kind (LINE / BLIND / PROJECTION /
    STOP_AT_LINE); only a raw ``drive`` move injects real drift (T9/T10 above
    already cover that path). Also confirms the fixture emits no
    ``drift_warn`` anywhere (a clean all-goto mission never triggers the
    residual indicator) and asserts the ACTUAL hop-tag set the route
    exercises covers LINE / BLIND / PROJECTION / STOP_AT_LINE / reverse — the
    toy field's ``PIVOT_TURNS`` is empty (confirmed by
    ``toy_field_data.PIVOT_TURNS == []``), so no pivot hop is reachable here;
    that gap is noted, not hidden. The FULL regression-locked drift sweep
    (every hop type incl. pivot, via the pivot fixtures) is T15's
    ``test_ctps_drift.py`` — this is the scaffold + the reusable fixture
    T15 consumes.
    """
    assert toy_field_data.PIVOT_TURNS == []   # documents the known gap (no pivot hop)

    trace = load(_FIXTURES / "ctps_all_hops_drift.json")
    goto_targets = [cmd.target for cmd in trace.commands if cmd.op == "goto"]
    assert len(goto_targets) == 9

    # Independent reference replay: collect (primitive tag, arrival snapshot)
    # so the hop-type coverage claim and the drift claim are both checked
    # against a genuinely separate sim run, not the builder's own state.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        graph = build_graph(toy_field_data)
        next_hop = compute_all_pairs_next_hop(
            graph, BlindDecayParams.from_field_data(toy_field_data)
        )
    chassis = VirtualChassis.from_field_data(toy_field_data)
    sim = PathTrackerSim(toy_field_data, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    tags_seen: set[str] = set()
    for target in goto_targets:
        snap = None
        for primitive, snap in sim.step_through(target):
            tags_seen.add(primitive[0])
        nx_, ny_ = toy_field_data.NODES[target]
        drift = hypot(snap.x_cm - nx_, snap.y_cm - ny_)
        assert drift < 1e-9, f"goto {target} landed off-node by {drift} cm"

    # LINE (follow-to-intersection + its overshoot roll, AND the reverse
    # follow), BLIND (forward dead-reckon AND its reverse), PROJECTION
    # (follow-to-distance AND its reverse), and STOP_AT_LINE (the seek) are
    # all present. No pivot tag can appear (PIVOT_TURNS is empty).
    assert {
        "follow_line_to_intersection",  # LINE forward
        "line_overshoot",               # LINE roll-onto-node
        "line_reverse",                 # LINE reverse
        "forward",                      # BLIND forward
        "blind_reverse",                # BLIND reverse
        "follow_line_to_distance",      # PROJECTION forward
        "projection_reverse",           # PROJECTION reverse
        "stop_at_line",                 # STOP_AT_LINE seek
    } <= tags_seen

    # Now drive it through the real builder and confirm no residual indicator.
    track = _build_quiet(trace, toy_field_data)
    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []
    assert track.duration_s > 0.0


# ---------------------------------------------------------------------------
# TC9 — raw drive integrates offset; next goto renders return, no residual
# ---------------------------------------------------------------------------


def test_tc9_raw_drive_then_goto_renders_return_no_residual(monkeypatch):
    """TC9 (AC7): a raw drive's offset integrates; the next goto returns + re-anchors.

    A fresh scenario (distinct from TC3/NC16 above, which this test's name
    formalises as the plan's TC9): a 20 cm raw ``driveDist`` off the toy
    field's start node ``S`` is followed by ``goto A``. The raw move must
    route through ``operator_move`` (spied here to prove the WIRING, not just
    the downstream effect) so the offset integrates on the shared sim; the
    following goto then emits exactly one ``return_to_node`` marker and NO
    residual ``drift_warn`` anywhere in the trace (the goto's return prologue
    re-anchors the offset before the planned walk).
    """
    calls: list[tuple[float, float]] = []
    original = PathTrackerSim.operator_move

    def _spy(self, distance_cm, *, turn_deg=0.0):
        calls.append((distance_cm, turn_deg))
        return original(self, distance_cm, turn_deg=turn_deg)

    monkeypatch.setattr(PathTrackerSim, "operator_move", _spy)

    track = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "drive", "method": "driveDist", "cm": 20.0},
            {"seq": 1, "op": "goto", "target": "A"},
        ),
        toy_field_data,
    )

    # The raw move actually reached operator_move (a real translation call).
    assert any(dist == pytest.approx(20.0) for dist, _turn in calls)

    return_events = [ev for ev in track.events if ev["type"] == "return_to_node"]
    assert len(return_events) == 1
    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []


# ---------------------------------------------------------------------------
# TC10 — raw drive at trace end residual; over-cap raw drive faults the track
# ---------------------------------------------------------------------------


def test_tc10_raw_drive_at_trace_end_leaves_residual_drift_warn():
    """TC10 (AC7): a raw drive with NO following goto leaves a residual indicator.

    Formalises the plan's TC10 trace-end case (already exercised by
    ``test_drive_without_following_goto_keeps_trace_end_residual`` above) as
    its own named test: nothing re-anchors a raw move that ends the mission,
    so exactly one ``drift_warn`` point event survives, stamped at the track
    end, and no ``return_to_node`` marker is ever emitted (there is no
    following goto to render one).
    """
    track = _build_quiet(
        _inline_trace({"seq": 0, "op": "drive", "method": "driveDist", "cm": 20.0}),
        toy_field_data,
    )

    drift_events = [ev for ev in track.events if ev["type"] == "drift_warn"]
    assert len(drift_events) == 1
    assert drift_events[0]["t"] == pytest.approx(track.duration_s, abs=_POSE_TOL)
    assert [ev for ev in track.events if ev["type"] == "return_to_node"] == []


def test_tc10_over_cap_raw_drive_under_fault_gate_ends_track():
    """TC10 (AC7): an over-cap raw drive under the FAULT gate faults + ends the track.

    A raw ``driveDist`` well past ``RETURN_DISTANCE_CAP_CM`` (50 cm) leaves an
    accumulated offset the return gate refuses to plan for under the default
    ``OffMapGate.FAULT`` (mirrors ``Fault::OFF_NODE`` — no chassis motion is
    attempted for the return). ``sim.step_through`` raises
    :class:`~tools.path_planner.sim.runtime.OffMapFaultError`, which ``build``
    catches and turns into an ``"OFF-MAP"``-labelled ``goto`` fault event that
    ENDS the track (no further commands are processed) — the branch T11 left
    for T13 to cover. The default gate is FAULT (no monkeypatch needed), and
    the trace-end residual check after the loop ALSO fires (the offset was
    never cleared), so a fault-ended track surfaces both a fault ``goto``
    event and a residual ``drift_warn``.
    """
    assert _runtime.OFF_MAP_GATE == OffMapGate.FAULT   # default gate; no patch needed

    over_cap_cm = RETURN_DISTANCE_CAP_CM + 10.0
    track = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "drive", "method": "driveDist", "cm": over_cap_cm},
            {"seq": 1, "op": "goto", "target": "A"},
        ),
        toy_field_data,
    )

    fault_events = [
        ev
        for ev in track.events
        if ev["type"] == "goto" and ev["label"].startswith("OFF-MAP")
    ]
    assert len(fault_events) == 1
    assert "S->A" in fault_events[0]["label"]
    assert f"{over_cap_cm:.1f}" in fault_events[0]["label"]
    # An UNREACHABLE/OFF-MAP fault event carries no `t_end` (the goto never
    # completed) -- it ended the track, so no return_to_node marker either.
    assert "t_end" not in fault_events[0]
    assert [ev for ev in track.events if ev["type"] == "return_to_node"] == []

    # The trace-end residual fires too -- nothing re-anchored the offset.
    drift_events = [ev for ev in track.events if ev["type"] == "drift_warn"]
    assert len(drift_events) == 1
    assert f"{over_cap_cm:.1f}" in drift_events[0]["label"]


def test_tc10_over_cap_raw_drive_under_best_effort_gate_proceeds_capped(monkeypatch):
    """TC10 contrast (AC7): the SAME over-cap drive under BEST_EFFORT proceeds capped.

    Patches ``OFF_MAP_GATE`` to ``BEST_EFFORT`` (the sim reads the module
    global at call time, so patching ``tools.path_planner.sim.runtime``'s
    attribute — not a name re-imported into ``track_builder`` — is the
    correct seam, mirroring ``test_runtime_sim.py``'s own pattern). No
    ``OffMapFaultError`` is raised: the goto still renders a (capped) return
    maneuver and completes, so a ``return_to_node`` marker appears and NO
    fault ``goto`` event does.
    """
    monkeypatch.setattr(_runtime, "OFF_MAP_GATE", OffMapGate.BEST_EFFORT)

    over_cap_cm = RETURN_DISTANCE_CAP_CM + 10.0
    track = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "drive", "method": "driveDist", "cm": over_cap_cm},
            {"seq": 1, "op": "goto", "target": "A"},
        ),
        toy_field_data,
    )

    assert [
        ev
        for ev in track.events
        if ev["type"] == "goto" and ev["label"].startswith("OFF-MAP")
    ] == []
    return_events = [ev for ev in track.events if ev["type"] == "return_to_node"]
    assert len(return_events) == 1
    goto_events = [
        ev
        for ev in track.events
        if ev["type"] == "goto" and not ev["label"].startswith("OFF-MAP")
    ]
    assert len(goto_events) == 1
    assert "t_end" in goto_events[0]   # a completed (capped) goto carries t_end


# ---------------------------------------------------------------------------
# TC11 — declare_position applies + marks; a declare-free trace stays
# byte-identical to an unknown-node (no-op) declare
# ---------------------------------------------------------------------------


def test_tc11_declare_position_applies_node_change_and_clears_offset():
    """TC11 (AC8): ``declare_position`` changes the sim's belief, not the chassis.

    A raw drive grows the offset off node ``A``; the operator then declares
    the (unrelated) node ``C`` — a pure belief reset per T7's contract (node
    change + offset clear, heading calibration untouched, NO chassis motion).
    The marker is a zero-dwell event (the clock does not advance for it), the
    trace-end residual is gone (the declare cleared the offset the raw drive
    grew), and a following goto plans FROM ``C`` — proving the belief, not
    just a label, actually changed.
    """
    track = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "drive", "method": "driveDist", "cm": 15.0},
            {"seq": 1, "op": "declare_position", "node": "C"},
            {"seq": 2, "op": "goto", "target": "D"},
        ),
        toy_field_data,
    )

    typed = [(ev["type"], ev["t"]) for ev in track.events]
    drive_t = next(t for kind, t in typed if kind == "drive")
    declare_events = [ev for ev in track.events if ev["type"] == "declare_position"]
    assert len(declare_events) == 1
    assert "C" in declare_events[0]["label"]

    # The declare comes after the drive's narration event and shares its
    # timestamp with the FOLLOWING goto's span start -- the declare itself
    # does not advance the clock (no dwell, no geometry): it is emitted at
    # whatever running clock the raw drive's densified advance left behind.
    goto_events = [ev for ev in track.events if ev["type"] == "goto"]
    assert declare_events[0]["t"] >= drive_t
    assert declare_events[0]["t"] == pytest.approx(goto_events[0]["t"], abs=_POSE_TOL)
    assert goto_events[0]["label"].startswith("C->")

    # No residual anywhere -- the declare cleared the offset the raw drive grew.
    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []


def test_tc11_unknown_node_declare_is_byte_identical_to_declare_free_trace():
    """TC11 negative path (AC8): an unknown-node declare renders BYTE-IDENTICAL.

    Per T7's contract, ``LocalizationSim.declare_position`` on an unknown node
    warns and no-ops (node + offset left completely unchanged), and
    ``track_builder`` mirrors that by pre-validating against ``NODES`` and
    emitting NO marker for it. So a trace carrying an unknown-node
    ``declare_position`` op must produce a Track whose sha256 (over the
    sorted-key JSON serialisation — :func:`_track_hash`) EXACTLY matches the
    same trace with that op removed entirely: the op contributes nothing —
    not a marker, not a keyframe, not a clock tick, not a warning suppressed
    into a silent state change.
    """
    declare_free = _inline_trace({"seq": 0, "op": "goto", "target": "B"})
    with_unknown_declare = _inline_trace(
        {"seq": 0, "op": "declare_position", "node": "NOPE"},
        {"seq": 1, "op": "goto", "target": "B"},
    )

    track_free = _build_quiet(declare_free, toy_field_data)
    track_unknown = _build_quiet(with_unknown_declare, toy_field_data)

    assert [
        ev for ev in track_unknown.events if ev["type"] == "declare_position"
    ] == []
    assert _track_hash(track_free) == _track_hash(track_unknown)


# ===========================================================================
# TC17 — CTPS RETURN-move speed (issue #157)
#
# Covers: under a field-sourced ``PIPELINE_SPEED: FAST``, both return-to-node
# prologue tags (``ACT_RETURN_FORWARD`` dead-reckon, ``ACT_RETURN_STOP_AT_LINE``
# line-seek) render their cruise plateau at the FAST PWM, matching firmware's
# ``_dispatchReturnMoveAfterTurn`` (both dispatched at ``PIPELINE_CONFIG.speed``);
# ``ACT_WALL_DEPART`` stays at the un-tiered ``SPEED_NORMAL`` default, matching
# firmware's ``.setPower``-free wall-departure move.
#
# Two hand-rolled minimal fields (not the toy field, which has no reset node and
# whose START_NODE is always BLIND) drive :func:`build` directly:
#
# * ``_return_speed_field_data`` — a 2-node ``S``(blind, start) -> ``A``(line)
#   chain. A 45 cm raw ``driveDist`` off ``S`` before ``goto A`` triggers the
#   dead-reckon return (``S`` is BLIND); a second 45 cm raw ``driveDist`` off
#   ``A`` before a second ``goto A`` triggers the line-seek return (``A`` is
#   LINE-typed and 45 cm clears ``LINE_SENSOR_AXLE_OFFSET_CM``). 45 cm is chosen
#   so the return's own T-shape profile is a genuine full trapezoid (not
#   triangular) at cruise=90 -- a short return never reaches the nominal cruise
#   PWM, which would make the velocity assertion fail for a geometric reason
#   unrelated to the fix under test.
# * ``_wall_depart_field_data`` — a 2-node ``S``(blind, start) -> ``R``(reset)
#   chain with a single ``goto R``: the arrival auto-fires the WALL_STOP ->
#   REANCHOR -> WALL_DEPART interlude (no raw drive needed). The 60 cm wall
#   standoff nets a WALL_DEPART distance of 52 cm (standoff -
#   ``WALL_CONTACT_OFFSET_CM``), comfortably long enough for a full trapezoid at
#   cruise=70 too.
#
# Both fields set ``PIPELINE_SPEED = SpeedTier.FAST`` directly (a Python
# fixture may author the enum instance, not just the string name -- see
# ``validator._validate_pipeline_config``), so the ONLY per-edge / mission-wide
# speed knob active is the mission-wide FAST tier: no ``EDGES`` overrides exist
# on either field, so any segment resolving a tier at all resolves FAST.
# ===========================================================================

#: Velocity match tolerance (cm/s) for the cruise-plateau speed assertions
#: below, mirroring ``test_ctps_run_grouper.py``'s ``_VEL_TOL``.
_TC17_VEL_TOL: float = 1e-6


def _return_speed_field_data() -> SimpleNamespace:
    """A 2-node BLIND(start) -> LINE field that exercises both return tags.

    :return: a validator-clean :class:`SimpleNamespace` field-data object.
    """
    return SimpleNamespace(
        FIELD_MM=(3000.0, 2000.0),
        MAP_PNG="maps/map.png",
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE="S",
        START_HEADING_DEG=0.0,
        NODES={"S": (0.0, 0.0), "A": (60.0, 0.0)},
        NODE_TYPES={"S": "blind", "A": "line"},
        LINE_SEGMENTS=[],
        BLIND_EDGES=[("S", "A")],
        OBSTACLES=[],
        PIVOT_TURNS=[],
        RESET_NODES={},
        EDGES=[],
        BLIND_PROJECTION_AUTO_SEEK=True,
        PIPELINE_SPEED=SpeedTier.FAST,
    )


def _wall_depart_field_data() -> SimpleNamespace:
    """A 2-node BLIND(start) -> RESET field that exercises ``ACT_WALL_DEPART``.

    :return: a validator-clean :class:`SimpleNamespace` field-data object.
    """
    return SimpleNamespace(
        FIELD_MM=(3000.0, 2000.0),
        MAP_PNG="maps/map.png",
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE="S",
        START_HEADING_DEG=0.0,
        NODES={"S": (0.0, 0.0), "R": (180.0, 0.0)},
        NODE_TYPES={"S": "blind", "R": "reset"},
        LINE_SEGMENTS=[],
        BLIND_EDGES=[("S", "R")],
        OBSTACLES=[],
        PIVOT_TURNS=[],
        RESET_NODES={"R": {"axis": "x", "wall_coord": 120.0, "facing": 0.0}},
        EDGES=[],
        BLIND_PROJECTION_AUTO_SEEK=True,
        PIPELINE_SPEED=SpeedTier.FAST,
    )


def _max_inbound_velocity(keyframes, start_idx: int, end_idx: int) -> float:
    """Return the fastest inbound speed (cm/s) among ``keyframes[start_idx:end_idx]``.

    A ramped T-shape segment's per-step average speed rises through the accel
    ramp, holds flat across the cruise plateau, then falls through the decel
    ramp -- so the MAXIMUM inbound speed in the slice equals the cruise PWM's
    speed exactly (``SCALE_CM_S_PER_PWM * cruise_pwm``), regardless of exactly
    where the interior ramp samples land. This avoids hand-picking the plateau
    keyframe pair by index or position.

    :param keyframes: the full ``track.keyframes`` list.
    :param start_idx: index of the segment's first (entry) keyframe.
    :param end_idx: index of the segment's last (landing) keyframe.
    :return: the fastest inbound speed observed between consecutive keyframes
        in ``[start_idx, end_idx]``; ``0.0`` if the slice is empty/degenerate.
    """
    speeds = []
    for a, b in zip(keyframes[start_idx:end_idx], keyframes[start_idx + 1 : end_idx + 1]):
        dt = b["t"] - a["t"]
        dist = hypot(b["x"] - a["x"], b["y"] - a["y"])
        speeds.append(dist / dt if dt > 1e-12 else 0.0)
    return max(speeds) if speeds else 0.0


def test_tc17_return_moves_render_at_pipeline_speed_wall_depart_stays_normal():
    """TC17 (issue #157): both return tags render at FAST; wall-depart stays NORMAL.

    Two independent builds, each isolating one primitive family so no boundary
    pose repeats ambiguously within a single build:

    1. ``_return_speed_field_data`` drives ``drive driveDist 45`` -> ``goto A``
       -> ``drive driveDist 45`` -> ``goto A``. The first ``goto`` triggers
       ``ACT_RETURN_FORWARD`` (dead-reckon, ``S`` is BLIND); the second
       triggers ``ACT_RETURN_STOP_AT_LINE`` (line-seek, ``A`` is LINE-typed and
       re-anchors back onto itself, so the trailing ``goto A`` needs no further
       hop). Both must render their cruise plateau at ``cfg.speed_fast`` (90),
       matching ``PIPELINE_SPEED: FAST`` -- pre-fix, both rendered at
       ``cfg.speed_normal`` (70) instead, conflating the return moves with the
       un-tiered wall-depart default.
    2. ``_wall_depart_field_data`` drives a single ``goto R`` (``R`` typed
       ``reset``): the arrival auto-fires WALL_STOP -> REANCHOR -> WALL_DEPART.
       ``ACT_WALL_DEPART`` must STILL render at ``cfg.speed_normal`` (70) despite
       the SAME ``PIPELINE_SPEED: FAST`` field -- it mirrors the firmware's
       ``.setPower``-free wall-departure move, which never consults
       ``PIPELINE_CONFIG.speed`` at all.
    """
    cfg = default_config()
    fast_speed = SCALE_CM_S_PER_PWM * cfg.speed_fast
    normal_speed = SCALE_CM_S_PER_PWM * cfg.speed_normal
    # Sanity-pin the current firmware Config values the plan cites (90 / 70) so
    # a future retune of ConfigDrive.h is caught here rather than silently
    # changing what "renders at 90" means.
    assert cfg.speed_fast == 90
    assert cfg.speed_normal == 70

    # -- Return tags: ACT_RETURN_FORWARD (dead-reckon) + ACT_RETURN_STOP_AT_LINE (line-seek) --
    return_trace = Trace(
        schema_version=1,
        start_node="S",
        start_heading_deg=0.0,
        build_error=False,
        commands=[
            Command(seq=0, op="drive", method="driveDist", cm=45.0),
            Command(seq=1, op="goto", target="A"),
            Command(seq=2, op="drive", method="driveDist", cm=45.0),
            Command(seq=3, op="goto", target="A"),
        ],
    )
    return_track = _build_quiet(return_trace, _return_speed_field_data())

    # Boundary poses, in chronological order: the return-forward span (facing
    # 180 after the inverse-offset pre-turn, S's exact node coordinate at the
    # far end), then the return-stop-at-line span (facing 0 after ITS
    # inverse-offset pre-turn, A's exact node coordinate at the far end).
    # ``_boundary_indices`` matches poses IN ORDER via an advancing cursor, so
    # this list alone disambiguates every coordinate from any other occurrence
    # of the same (x, y) at a different heading or trace position.
    return_boundaries = _boundary_indices(
        return_track.keyframes,
        [
            (45.0, 0.0, 180.0),  # return_forward: entry (post pre-turn)
            (0.0, 0.0, 180.0),  # return_forward: landing (S's node coordinate)
            (15.0, 0.0, 0.0),  # return_stop_at_line: entry (post pre-turn)
            (60.0, 0.0, 0.0),  # return_stop_at_line: landing (A's node coordinate)
        ],
    )
    return_forward_v = _max_inbound_velocity(
        return_track.keyframes, return_boundaries[0], return_boundaries[1]
    )
    return_stop_at_line_v = _max_inbound_velocity(
        return_track.keyframes, return_boundaries[2], return_boundaries[3]
    )
    assert return_forward_v == pytest.approx(fast_speed, abs=_TC17_VEL_TOL)
    assert return_stop_at_line_v == pytest.approx(fast_speed, abs=_TC17_VEL_TOL)

    # -- ACT_WALL_DEPART: unaffected by PIPELINE_SPEED, stays NORMAL --
    wall_trace = Trace(
        schema_version=1,
        start_node="S",
        start_heading_deg=0.0,
        build_error=False,
        commands=[Command(seq=0, op="goto", target="R")],
    )
    wall_track = _build_quiet(wall_trace, _wall_depart_field_data())

    # (128, 0, 0) is the unique REANCHOR-landed axle coordinate (wall_coord 120
    # + WALL_CONTACT_OFFSET_CM 8); (180, 0, 0) is R's node coordinate, which
    # ALSO appears earlier (the inbound arrival, before WALL_STOP runs) -- the
    # ordered cursor in ``_boundary_indices`` skips that earlier occurrence and
    # lands on WALL_DEPART's own landing, the later one.
    wall_boundaries = _boundary_indices(
        wall_track.keyframes,
        [
            (128.0, 0.0, 0.0),  # wall_depart: entry (post REANCHOR)
            (180.0, 0.0, 0.0),  # wall_depart: landing (R's node coordinate)
        ],
    )
    wall_depart_v = _max_inbound_velocity(
        wall_track.keyframes, wall_boundaries[0], wall_boundaries[1]
    )
    assert wall_depart_v == pytest.approx(normal_speed, abs=_TC17_VEL_TOL)
    assert wall_depart_v != pytest.approx(fast_speed, abs=_TC17_VEL_TOL)
