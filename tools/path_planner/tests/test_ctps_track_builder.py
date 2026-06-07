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
"""

from __future__ import annotations

import warnings
from pathlib import Path

import pytest

from tools.path_planner.ctps.config import DRIFT_THRESHOLD_CM
from tools.path_planner.ctps.track_builder import build
from tools.path_planner.ctps.trace_loader import load
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.planner import compute_all_pairs_next_hop
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
            graph, field_data.COST_BLIND_MULTIPLIER
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
    :data:`DRIFT_THRESHOLD_CM`. The 15 cm raw move stacks on the ~5 cm line
    overshoot baseline, clearing the 10 cm threshold.
    """
    trace = load(_FIXTURES / "ctps_drive_drift.json")
    track = _build_quiet(trace, toy_field_data)

    drift_events = [ev for ev in track.events if ev["type"] == "drift_warn"]
    assert len(drift_events) == 1

    warn = drift_events[0]
    assert "B" in warn["label"]

    # Pull the centimetre figure out of the label ("chassis 20.0 cm off node B").
    reported_cm = float(warn["label"].split("cm")[0].split()[-1])
    assert reported_cm > DRIFT_THRESHOLD_CM


# ---------------------------------------------------------------------------
# TC4 — a small raw drive stays under the threshold (no warning)
# ---------------------------------------------------------------------------


def test_tc4_small_drive_before_goto_no_drift_warn():
    """TC4: a 2 cm raw drive before goto B stays under DRIFT_THRESHOLD_CM.

    Key assertion: NO ``drift_warn`` event is emitted. The 2 cm move plus the
    ~5 cm line overshoot baseline (~7 cm total) sits below the 10 cm threshold,
    so the drift check stays silent.
    """
    trace = load(_FIXTURES / "ctps_drive_no_drift.json")
    track = _build_quiet(trace, toy_field_data)

    drift_events = [ev for ev in track.events if ev["type"] == "drift_warn"]
    assert drift_events == []
