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
"""

from __future__ import annotations

import warnings
from math import ceil, hypot
from pathlib import Path

import pytest

from tools.path_planner.compiler import ACT_TURN_REL
from tools.path_planner.ctps.config import ARC_STEP_DEG, DRIFT_THRESHOLD_CM, SIM_TURN_DEG_S
from tools.path_planner.ctps.track_builder import build
from tools.path_planner.ctps.trace_loader import Command, Trace, load
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.planner import (
    BlindDecayParams,
    compute_all_pairs_next_hop,
    parse_track_cm_from_config,
)
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import PathTrackerSim
from tools.path_planner.tests.fixtures import pivot_field_data, toy_field_data

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
# TC4 — even a small raw drive opens a drift span (threshold-free)
# ---------------------------------------------------------------------------


def test_tc4_small_drive_before_goto_opens_span_closed_by_goto():
    """TC4: even a *small* raw drive before goto B opens a DRIFT span.

    The ``DRIFT_THRESHOLD_CM`` gate is gone: any non-``goto`` translation is
    off-plan, so a 2 cm ``driveDist`` emits exactly one ``drift_warn`` — a SPAN
    opened at the drive and CLOSED by the following goto re-anchor (a finite
    ``t_end`` strictly after the open time). The reported offset is still small
    (under the old threshold) but it is now surfaced, not hidden. The fixture
    keeps its historical ``_no_drift`` name.
    """
    trace = load(_FIXTURES / "ctps_drive_no_drift.json")
    track = _build_quiet(trace, toy_field_data)

    drift_events = [ev for ev in track.events if ev["type"] == "drift_warn"]
    assert len(drift_events) == 1

    warn = drift_events[0]
    assert "B" in warn["label"]
    reported_cm = float(warn["label"].split("cm")[0].split()[-1])
    assert reported_cm < DRIFT_THRESHOLD_CM     # small — but flagged now

    # The span is CLOSED by the goto re-anchor: finite t_end, after the open t.
    assert warn["t_end"] is not None
    assert warn["t_end"] > warn["t"]


def test_drive_without_following_goto_drifts_to_end():
    """A raw drive that ends the mission drifts to the end of the track.

    No ``goto`` follows the drive, so nothing re-anchors: the ``drift_warn``
    span stays open and its ``t_end`` is the track's ``duration_s`` (runs to
    end), with the offset measured off the logical node the chassis left.
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
    assert warn["t_end"] == pytest.approx(track.duration_s, abs=_POSE_TOL)
    assert warn["t_end"] > warn["t"]


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

    ``N = ceil(90 / ARC_STEP_DEG)`` sub-keyframes REPLACE the single in-place
    turn keyframe (``N - 1`` intermediate keyframes plus the arc-end); each
    sample is within tolerance of the documented Contracts formula (rotation
    about the planted wheel); heading rotates through the full 90 deg; the
    whole arc spans exactly ``90 / SIM_TURN_DEG_S`` seconds, split evenly
    across the N steps; the running clock stays strictly monotonic across the
    replacement span.
    """
    field, track = _dead_reckon_pivot_track()

    n = ceil(90.0 / ARC_STEP_DEG)
    # seed + the trimmed U->V approach + N arc keyframes + the trimmed V->W
    # departure — the arc REPLACES the single in-place turn keyframe.
    assert len(track.keyframes) == n + 3

    approach_end = track.keyframes[1]
    arc = track.keyframes[2 : 2 + n]
    assert len(arc) == n

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

    # Even timing across the N replacement sub-keyframes.
    times = [approach_end["t"]] + [kf["t"] for kf in arc]
    steps = [b - a for a, b in zip(times, times[1:])]
    for step in steps:
        assert step == pytest.approx(steps[0], abs=1e-9)

    # Total arc duration: exactly the in-place turn's old visual duration.
    assert arc[-1]["t"] - approach_end["t"] == pytest.approx(
        90.0 / SIM_TURN_DEG_S, abs=1e-9
    )

    # The running clock stays strictly monotonic across the whole keyframe
    # list (the arc replacement plus the trailing departure keyframe).
    all_times = [kf["t"] for kf in track.keyframes]
    assert all(b > a for a, b in zip(all_times, all_times[1:]))


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

    approach_end = track.keyframes[1]
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

    geometry_keyframes = track.keyframes[1:]
    assert len(geometry_keyframes) == len(reference)
    for kf, (rx, ry, rh) in zip(geometry_keyframes, reference):
        assert kf["x"] == pytest.approx(rx, abs=_POSE_TOL)
        assert kf["y"] == pytest.approx(ry, abs=_POSE_TOL)
        assert kf["h"] == pytest.approx(rh, abs=_POSE_TOL)

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
    t0s = [segment["t0"] for segment in track.pivots]
    assert t0s == sorted(t0s)   # emitted in chronological order
    assert all(segment["wheel"] == "right" for segment in track.pivots)
