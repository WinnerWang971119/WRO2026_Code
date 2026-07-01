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
* **TC3** — a raw ``drive driveDist 15`` before a ``goto`` injects a chassis
  offset past :data:`DRIFT_THRESHOLD_CM`, so a ``drift_warn`` event is emitted.
* **TC4** — the same shape with a 2 cm drive stays under the threshold, so NO
  ``drift_warn`` is emitted.

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
* **NC16** — a ``lineFollowDist`` offset before a ``goto`` emits a
  ``drift_warn``; TC1-TC5 are kept unchanged.

The toy chassis seeds facing ``START_HEADING_DEG == 90`` (node ``S``), so
"forward along the current facing" is ``+y`` in these tests, not ``+x``.
"""

from __future__ import annotations

import warnings
from pathlib import Path

import pytest

from tools.path_planner.ctps.config import DRIFT_THRESHOLD_CM
from tools.path_planner.ctps.track_builder import build
from tools.path_planner.ctps.trace_loader import load
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.planner import BlindDecayParams, compute_all_pairs_next_hop
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import PathTrackerSim
from tools.path_planner.tests.fixtures import toy_field_data

_FIXTURES = Path(__file__).resolve().parent / "fixtures"

#: Position tolerance for keyframe-vs-reference pose equality (cm / deg).
_POSE_TOL = 1e-6


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
    """TC1: keyframe poses equal an independent step_through run; clock monotonic.

    Key assertion: ``track.keyframes[1:]`` (every keyframe past the seeded
    start pose) equals, pose-for-pose, the snapshots of an independently
    constructed ``step_through`` reference replay of the same two goals.
    Also asserts the running clock is monotonic non-decreasing and
    ``duration_s == keyframes[-1]["t"]``.
    """
    trace = load(_FIXTURES / "ctps_two_goto.json")
    track = _build_quiet(trace, toy_field_data)

    # The stream is exactly two reachable gotos.
    goto_targets = [cmd.target for cmd in trace.commands if cmd.op == "goto"]
    assert goto_targets == ["B", "C"]

    reference = _reference_snapshots(toy_field_data, goto_targets)

    # Keyframe[0] is the seeded start pose at t = 0; every subsequent keyframe
    # is one step_through yield, so keyframes[1:] maps 1:1 onto the reference.
    geometry_keyframes = track.keyframes[1:]
    assert len(geometry_keyframes) == len(reference)
    for kf, (rx, ry, rh) in zip(geometry_keyframes, reference):
        assert kf["x"] == pytest.approx(rx, abs=_POSE_TOL)
        assert kf["y"] == pytest.approx(ry, abs=_POSE_TOL)
        assert kf["h"] == pytest.approx(rh, abs=_POSE_TOL)

    # Seed keyframe sits at the start pose at t = 0.
    sx, sy = toy_field_data.NODES[toy_field_data.START_NODE]
    assert track.keyframes[0]["t"] == 0.0
    assert track.keyframes[0]["x"] == pytest.approx(sx, abs=_POSE_TOL)
    assert track.keyframes[0]["y"] == pytest.approx(sy, abs=_POSE_TOL)

    # Monotonic non-decreasing clock.
    times = [kf["t"] for kf in track.keyframes]
    assert all(b >= a for a, b in zip(times, times[1:]))

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


# ---------------------------------------------------------------------------
# TC3 — raw drive past the threshold raises a drift warning
# ---------------------------------------------------------------------------


def test_tc3_drive_before_goto_emits_drift_warn():
    """TC3: a 15 cm raw drive before goto B injects > DRIFT_THRESHOLD_CM offset.

    Key assertion: a ``drift_warn`` event is present, its label names the node
    the chassis drifted off ("B"), and the reported offset exceeds
    :data:`DRIFT_THRESHOLD_CM`. A clean goto now lands the axle ON the node
    (no overshoot baseline — drift ≈ 0 without a raw move), so the 15 cm raw
    move alone is the drift and clears the 10 cm threshold.
    """
    trace = load(_FIXTURES / "ctps_drive_drift.json")
    track = _build_quiet(trace, toy_field_data)

    drift_events = [ev for ev in track.events if ev["type"] == "drift_warn"]
    assert len(drift_events) == 1

    warn = drift_events[0]
    assert "B" in warn["label"]

    # Pull the centimetre figure out of the label ("chassis 15.0 cm off node B").
    reported_cm = float(warn["label"].split("cm")[0].split()[-1])
    assert reported_cm > DRIFT_THRESHOLD_CM


# ---------------------------------------------------------------------------
# TC4 — a small raw drive stays under the threshold (no warning)
# ---------------------------------------------------------------------------


def test_tc4_small_drive_before_goto_no_drift_warn():
    """TC4: a 2 cm raw drive before goto B stays under DRIFT_THRESHOLD_CM.

    Key assertion: NO ``drift_warn`` event is emitted. A clean goto lands the
    axle ON the node (no overshoot baseline), so the 2 cm raw move alone is the
    drift — below the 10 cm threshold — and the drift check stays silent.
    """
    trace = load(_FIXTURES / "ctps_drive_no_drift.json")
    track = _build_quiet(trace, toy_field_data)

    drift_events = [ev for ev in track.events if ev["type"] == "drift_warn"]
    assert drift_events == []


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

    # Seed pose + exactly one advanced geometry keyframe.
    assert len(track.keyframes) == 2
    seed, moved = track.keyframes[0], track.keyframes[1]

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


def test_nc9_gyro_heading_drive_dist_renders_exactly_three_keyframes():
    """NC9 (AC3): a single GYRO+heading ``driveDist`` yields exactly 3 keyframes.

    ``driveDist(35).setMode(GYRO).setHeading(90)`` renders two-phase: seed ->
    rotate-in-place to the held heading (a keyframe at the UNCHANGED position)
    -> advance along that heading (the translation keyframe). A single-command
    trace therefore produces EXACTLY three keyframes — the rotation keyframe is
    what distinguishes it from the two-keyframe plain move (NC10).

    The toy chassis seeds facing ``START_HEADING_DEG == 90`` (node ``S``), so
    the turn-to-90 is a no-op rotation here; it still emits its own keyframe, so
    the count is three. The advance runs along heading 90 (``0deg = +x``,
    ``90deg = +y``), displacing ``+y`` by 35 and ending facing 90.
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

    # EXACTLY three keyframes: seed, rotation, translation.
    assert len(track.keyframes) == 3
    seed, rotation, translation = track.keyframes

    # Seed sits at the start pose at t = 0.
    assert seed["t"] == 0.0

    # Rotation phase: UNCHANGED position, heading turned to the held 90.
    assert rotation["x"] == pytest.approx(seed["x"], abs=_POSE_TOL)
    assert rotation["y"] == pytest.approx(seed["y"], abs=_POSE_TOL)
    assert rotation["h"] == pytest.approx(90.0, abs=_POSE_TOL)

    # Translation phase: displaced along +y (heading 90), still facing 90.
    assert translation["h"] == pytest.approx(90.0, abs=_POSE_TOL)
    assert translation["x"] == pytest.approx(rotation["x"], abs=_POSE_TOL)
    assert translation["y"] - rotation["y"] == pytest.approx(35.0, abs=_POSE_TOL)
    assert _planar_distance(rotation, translation) == pytest.approx(
        35.0, abs=_POSE_TOL
    )

    # Timestamps monotonic non-decreasing.
    times = [kf["t"] for kf in track.keyframes]
    assert all(b >= a for a, b in zip(times, times[1:]))


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

    # Two keyframes only: seed + a single advance (NOT the gyro two-phase).
    assert len(track.keyframes) == 2
    seed, moved = track.keyframes

    # No rotation phase: heading is unchanged from the seed facing.
    assert moved["h"] == pytest.approx(seed["h"], abs=_POSE_TOL)

    # Advance runs along the CURRENT facing (seed heading) by 35 cm, regardless
    # of the recorded (ignored) heading value.
    assert _heading_projection(seed, moved, seed["h"]) == pytest.approx(
        35.0, abs=_POSE_TOL
    )

    # Decisive firmware-parity check: identical geometry to a plain move with no
    # heading at all -> the recorded heading changed nothing.
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

    # Seed + exactly one forward geometry keyframe.
    assert len(track.keyframes) == 2
    seed, moved = track.keyframes

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

    seed, forward_leg = track.keyframes[0], track.keyframes[1]
    assert _heading_projection(seed, forward_leg, seed["h"]) == pytest.approx(
        60.0, abs=_POSE_TOL
    )

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
    track = _build_quiet(load(_FIXTURES / "ctps_reverse.json"), toy_field_data)

    # Seed + one keyframe per command (no gyro two-phase in this fixture).
    assert len(track.keyframes) == 5
    kfs = track.keyframes
    heading = kfs[0]["h"]

    # Heading never changes — reverse translates, it never rotates.
    assert all(kf["h"] == pytest.approx(heading, abs=_POSE_TOL) for kf in kfs)

    # Reversed lineFollowDist(40) / stopAtLine(15) / wallStop(10): backward.
    for index, cm in enumerate((40.0, 15.0, 10.0), start=1):
        projection = _heading_projection(kfs[index - 1], kfs[index], heading)
        assert projection == pytest.approx(-cm, abs=_POSE_TOL)

    # Trailing plain driveDist(20): forward (positive projection), for contrast.
    assert _heading_projection(kfs[3], kfs[4], heading) == pytest.approx(
        20.0, abs=_POSE_TOL
    )

    # The three reversed drive events carry the "reverse" token; the last does not.
    drive_labels = [ev["label"] for ev in track.events if ev["type"] == "drive"]
    assert len(drive_labels) == 4
    assert all("reverse" in label for label in drive_labels[:3])
    assert "reverse" not in drive_labels[3]


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

    assert len(track.keyframes) == 3
    seed, rotation, translation = track.keyframes

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

    assert len(track.keyframes) == 3
    seed, rotation, translation = track.keyframes

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
# NC16 — a lineFollowDist offset before a goto emits a drift warning
# ---------------------------------------------------------------------------


def test_nc16_line_follow_dist_before_goto_emits_drift_warn():
    """NC16 (AC4/AC11 regression): a ``lineFollowDist`` offset then goto drifts.

    Mirrors TC3's structure but with ``lineFollowDist`` as the offending raw
    move: a 15 cm distance-gated follow before ``goto B`` injects an offset the
    goto does not correct — ``lineFollowDist`` advances the shared chassis
    without advancing the logical node, exactly the desync the drift check
    surfaces. A ``drift_warn`` naming node B is emitted with a reported offset
    past :data:`DRIFT_THRESHOLD_CM`. This pins that authored line-follows now
    contribute drift events where they were previously invisible.

    (TC1-TC5 above are kept unchanged and continue to run as-is.)
    """
    track = _build_quiet(
        _inline_trace(
            {"seq": 0, "op": "drive", "method": "lineFollowDist", "cm": 15.0},
            {"seq": 1, "op": "goto", "target": "B"},
        ),
        toy_field_data,
    )

    drift_events = [ev for ev in track.events if ev["type"] == "drift_warn"]
    assert len(drift_events) == 1

    warn = drift_events[0]
    assert "B" in warn["label"]

    # Pull the centimetre figure out of the label ("chassis 15.0 cm off node B").
    reported_cm = float(warn["label"].split("cm")[0].split()[-1])
    assert reported_cm > DRIFT_THRESHOLD_CM
