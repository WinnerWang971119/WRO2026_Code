"""AC5 regression lock: no drift at any ``goto`` arrival, across ALL SIX hop types.

Issue #130, T15 — the "No drift, 100%" investigation + regression. The plan's
AC5 requires that a representative clean mission lands the chassis on each
``goto`` node (drift ≈ 0, float-noise only) across **LINE / BLIND / PROJECTION /
STOP_AT_LINE / pivot / reverse** hops, and that any residual is investigated and
closed rather than masked by a loose threshold.

Investigation outcome (see the T15 summary)
-------------------------------------------
The sweep was run against every hop type. **No residual > float noise was found**
and **no production fix was needed** — ``track_builder.py`` / ``profile.py`` are
untouched by T15. The observed max drift at any ``goto`` arrival is:

* LINE / BLIND / PROJECTION / STOP_AT_LINE / reverse (the 9-goto toy tour):
  ``5.7e-14`` cm.
* pivot (the ADR-036 pivot fixtures, each a single ``goto`` that takes a real
  single-wheel pivot arc): ``≤ 1.4e-14`` cm.

Both are ~5 orders of magnitude under the :data:`_DRIFT_BOUND_CM` = ``1e-9`` cm
regression bound, which itself has huge headroom over the noise while still
catching any real drift (a raw off-node drive injects centimetres — see
``test_ctps_track_builder.py`` TC9/TC10). The bound is deliberately tight: do
NOT loosen it to absorb a residual — a nonzero drift on a CLEAN mission would be
a real bug (the sim chassis is the wheel axle and must land ON the node), not
something to hide behind a bigger tolerance.

Why measuring the right surface matters (the pivot subtlety)
------------------------------------------------------------
There are two distinct "drift" quantities and this test pins the correct one for
each hop family:

* The sim's localization **offset** (``get_offset_from_node().magnitude_cm()``) —
  the exact quantity the firmware return gate and the CTPS trace-end
  ``drift_warn`` indicator read. It is ``0`` at every ``goto`` arrival for EVERY
  hop type, pivots included (a ``goto`` re-anchors on its node; no raw move
  integrated an offset).
* The **rendered chassis pose** — ``keyframes[-1]`` (equivalently the keyframe at
  the ``goto`` span's ``t_end``) vs ``NODES[target]``. For LINE / BLIND /
  PROJECTION / STOP_AT_LINE / reverse the bare sim ``step_through`` pose already
  lands on-node, so an independent reference replay measures ~0 directly. For a
  PIVOT hop the bare ``step_through`` pose is the *un-nudged* in-place-turn pose
  (off-node by the turn radius); ``build`` applies the ADR-044 along-edge
  re-anchor **chassis nudge** (``track_builder._build_pivot_arc`` + the
  sensor-departure pull-back) so the RENDERED chassis lands on the node. The
  pivot drift is therefore measured on the built ``Track`` — the surface that
  carries the nudge — not on a bare replay that would report the pre-nudge pose.

So the pivot sweep confirms exactly what the plan's gotcha predicts: the pivot
nudge writes ``chassis.x/y`` directly and lands ON the node, so pivot
drift-at-arrival is ≈ 0 — verified here, not assumed.
"""

from __future__ import annotations

import warnings
from math import hypot
from pathlib import Path

import pytest

from tools.path_planner.ctps.track_builder import build
from tools.path_planner.ctps.trace_loader import Command, Trace, load
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.planner import (
    BlindDecayParams,
    compute_all_pairs_next_hop,
)
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import PathTrackerSim
from tools.path_planner.tests.fixtures import pivot_field_data, toy_field_data

_FIXTURES = Path(__file__).resolve().parent / "fixtures"

#: The regression drift bound (cm). The T13 scaffold observed ~5.7e-14 cm of
#: float noise at every goto arrival, so 1e-9 has ~5 orders of magnitude of
#: headroom while still catching any REAL drift (a raw off-node drive injects
#: centimetres). Do NOT loosen this to absorb a residual — a nonzero drift on a
#: CLEAN goto is a real bug, not a tolerance to widen.
_DRIFT_BOUND_CM: float = 1e-9

#: The hop-tag families the toy all-hops tour must exercise (the five non-pivot
#: hop types AC5 names). PIVOT is covered separately by the ADR-036 fixtures —
#: the toy field's ``PIVOT_TURNS`` is empty, so no pivot tag is reachable here.
_REQUIRED_NON_PIVOT_TAGS: frozenset[str] = frozenset(
    {
        "follow_line_to_intersection",  # LINE forward (follow-to-intersection)
        "line_overshoot",               # LINE roll-onto-node re-anchor
        "line_reverse",                 # LINE reverse
        "forward",                      # BLIND forward (dead-reckon)
        "blind_reverse",                # BLIND reverse
        "follow_line_to_distance",      # PROJECTION forward (follow-to-distance)
        "projection_reverse",           # PROJECTION reverse
        "stop_at_line",                 # STOP_AT_LINE seek
    }
)


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _build_quiet(trace: Trace, field_data):
    """Run :func:`build` with benign fixture ``UserWarning``s suppressed.

    ``build`` calls ``build_graph(field_data)`` directly; the toy field's ``P1``
    node is reachable only via a demo projection edge the fixture omits, so
    ``build_graph`` emits a benign orphan-node ``UserWarning`` that is expected
    fixture noise, silenced here so it cannot interfere with the assertions.

    :param trace: the command-stream trace to replay.
    :param field_data: the field-data namespace to build against.
    :return: the built :class:`~tools.path_planner.ctps.track_builder.Track`.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        return build(trace, field_data)


def _fresh_sim(field_data):
    """Return a fresh, INDEPENDENT ``(chassis, sim)`` pair for a reference replay.

    Mirrors the exact construction :func:`build` performs (graph + all-pairs
    next-hop table, one long-lived chassis + sim, heading zeroed once) but is
    built entirely independently of the builder, so a drift reading off it is a
    genuine cross-check rather than a tautology on the builder's own state.

    :param field_data: the field-data namespace to build the sim against.
    :return: the ``(VirtualChassis, PathTrackerSim)`` pair, heading pre-zeroed.
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
    return chassis, sim


def _goto_trace(field_data, target: str) -> Trace:
    """Build a synthetic single-``goto`` Trace targeting ``target``.

    The pivot fixtures are hand-built ``SimpleNamespace`` field-data (ADR-036),
    not real trace JSON, so this constructs the Data Model A ``Trace`` /
    ``Command`` dataclasses directly rather than routing through
    :func:`tools.path_planner.ctps.trace_loader.load`. ``start_node`` /
    ``start_heading_deg`` are informational only — :func:`build` always seeds the
    chassis from ``field_data``, never from the ``Trace``.

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


def _keyframe_at(keyframes, t: float):
    """Return the last keyframe whose timestamp is at/just before ``t``.

    The rendered chassis pose "at a goto arrival" is the keyframe at the goto
    span event's ``t_end`` — the last sub-keyframe of the goto's final densified
    hop (which is pinned byte-exact to the sim snapshot / nudged pose). Walk to
    the last keyframe with ``kf["t"] <= t`` (a tiny epsilon guards float noise on
    the equality); timestamps are non-decreasing so this is well-defined.

    :param keyframes: the built track's keyframe list (non-decreasing ``t``).
    :param t: the arrival timestamp (a goto span's ``t_end``).
    :return: the keyframe dict at that arrival.
    """
    chosen = keyframes[0]
    for kf in keyframes:
        if kf["t"] <= t + 1e-12:
            chosen = kf
        else:
            break
    return chosen


def _goto_arrival_drift(track, field_data, goto_targets) -> float:
    """Max rendered-chassis-vs-node drift (cm) across every goto arrival in ``track``.

    For each successful ``goto`` span event (carrying a ``t_end``), reads the
    rendered chassis pose at that arrival (:func:`_keyframe_at`) and measures its
    distance to the declared target node ``NODES[target]``. This is the surface
    that carries the pivot re-anchor nudge, so it is correct for EVERY hop type
    (LINE / BLIND / PROJECTION / STOP_AT_LINE / reverse land on-node natively;
    pivot lands on-node via the nudge ``build`` applied).

    :param track: the built :class:`Track`.
    :param field_data: the field-data namespace (for ``NODES``).
    :param goto_targets: the ordered list of goto target node names.
    :return: the maximum arrival drift (cm) over all gotos (0.0 if none).
    """
    goto_events = [
        ev for ev in track.events if ev["type"] == "goto" and "t_end" in ev
    ]
    # A successful goto emits exactly one span event with t_end, in order; a
    # faulted goto (UNREACHABLE / OFF-MAP) has NO t_end and ends the track, so
    # the span-event list is exactly the successful-goto prefix of goto_targets.
    assert len(goto_events) == len(goto_targets), (
        "every goto must succeed (a clean mission has no faulted goto): "
        f"{len(goto_events)} span events vs {len(goto_targets)} targets"
    )
    max_drift = 0.0
    for ev, target in zip(goto_events, goto_targets):
        kf = _keyframe_at(track.keyframes, ev["t_end"])
        node_x, node_y = field_data.NODES[target]
        max_drift = max(max_drift, hypot(kf["x"] - node_x, kf["y"] - node_y))
    return max_drift


# ---------------------------------------------------------------------------
# LINE / BLIND / PROJECTION / STOP_AT_LINE / reverse — the toy all-hops tour
# ---------------------------------------------------------------------------


def test_non_pivot_hops_drift_sweep_zero_at_every_goto_arrival():
    """AC5: LINE/BLIND/PROJECTION/STOP_AT_LINE/reverse land drift ≈ 0 every arrival.

    Drives the T13 all-hop-types fixture (``ctps_all_hops_drift.json`` — a
    9-goto tour of the toy field from its true ``START_NODE`` "S") and asserts,
    at EVERY ``goto`` arrival, drift < :data:`_DRIFT_BOUND_CM` on BOTH surfaces:

    1. the sim localization **offset** magnitude
       (``get_offset_from_node().magnitude_cm()`` — the return-gate /
       ``drift_warn`` metric) read off an INDEPENDENT reference replay after each
       goto's ``step_through`` completes; and
    2. the rendered chassis pose at the goto span's ``t_end`` vs the declared
       node, off the real built ``Track``.

    Also asserts the ACTUAL hop-tag set the route exercises covers all five
    non-pivot hop types AC5 names (:data:`_REQUIRED_NON_PIVOT_TAGS`) — so this is
    a genuine all-hop-types sweep, not a route that happened to skip a family —
    and that the toy field carries no reachable pivot (``PIVOT_TURNS == []``), the
    documented reason PIVOT is covered separately below.
    """
    assert toy_field_data.PIVOT_TURNS == []  # no pivot reachable here — see pivot test

    trace = load(_FIXTURES / "ctps_all_hops_drift.json")
    goto_targets = [cmd.target for cmd in trace.commands if cmd.op == "goto"]
    assert len(goto_targets) == 9

    # (1) Independent reference replay: sim offset + bare chassis-vs-node at each
    #     goto arrival, plus the hop-tag coverage set.
    _chassis, sim = _fresh_sim(toy_field_data)
    tags_seen: set[str] = set()
    for target in goto_targets:
        snap = None
        for primitive, snap in sim.step_through(target):
            tags_seen.add(primitive[0])
        offset_mag = sim.localization.get_offset_from_node().magnitude_cm()
        assert offset_mag < _DRIFT_BOUND_CM, (
            f"goto {target}: sim offset {offset_mag} cm exceeds bound "
            f"{_DRIFT_BOUND_CM}"
        )
        node_x, node_y = toy_field_data.NODES[target]
        chassis_drift = hypot(snap.x_cm - node_x, snap.y_cm - node_y)
        assert chassis_drift < _DRIFT_BOUND_CM, (
            f"goto {target}: chassis landed {chassis_drift} cm off-node"
        )

    # All five non-pivot hop families are genuinely exercised (LINE forward +
    # overshoot + reverse, BLIND forward + reverse, PROJECTION forward + reverse,
    # STOP_AT_LINE seek). No pivot tag can appear — PIVOT_TURNS is empty.
    assert _REQUIRED_NON_PIVOT_TAGS <= tags_seen, (
        f"missing hop families: {_REQUIRED_NON_PIVOT_TAGS - tags_seen}"
    )

    # (2) Real built Track: rendered chassis lands on each node, and NO residual
    #     drift_warn indicator fires anywhere in the clean mission.
    track = _build_quiet(trace, toy_field_data)
    arrival_drift = _goto_arrival_drift(track, toy_field_data, goto_targets)
    assert arrival_drift < _DRIFT_BOUND_CM, (
        f"rendered chassis max arrival drift {arrival_drift} cm exceeds bound"
    )
    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []
    assert track.duration_s > 0.0


# ---------------------------------------------------------------------------
# PIVOT — the gap T13 left; covered via the ADR-036 pivot fixtures
# ---------------------------------------------------------------------------
#
# Each fixture is a purpose-built field with a REACHABLE declared pivot
# (``PIVOT_TURNS`` non-empty), replayed through a single ``goto`` that takes the
# turn. ``build`` applies the ADR-044 along-edge re-anchor chassis nudge on the
# pivot arc, so the RENDERED chassis lands ON the destination node — pivot
# drift-at-arrival is ≈ 0. The pivot list spans dead-reckon, LINE-incoming,
# chained, obtuse auto-REAR-forced-FRONT, sensor-incoming (projection + authored
# stop_at_line), LINE-departing, and chained-sensor-departure topologies — the
# full reachable-pivot surface the sim + track_builder support.

#: ``(fixture_id, field_factory, goto_target, expected_pivot_count)`` for the
#: pivot drift sweep. ``expected_pivot_count`` pins that a REAL pivot arc fired
#: (an empty ``pivots`` overlay would mean the turn degraded to a plain in-place
#: turn and the sweep would not actually exercise the pivot path).
_PIVOT_CASES = [
    ("dead_reckon", pivot_field_data.make_dead_reckon_pivot_field, "W", 1),
    ("line_incoming", pivot_field_data.make_line_incoming_pivot_field, "L2", 1),
    ("chained", pivot_field_data.make_chained_pivot_field, "X", 2),
    ("auto_rear_forced_front", pivot_field_data.make_auto_rear_pivot_field, "R", 1),
    ("sensor_autorule", pivot_field_data.make_sensor_autorule_pivot_field, "C", 1),
    ("sensor_authored", pivot_field_data.make_sensor_authored_pivot_field, "C", 1),
    ("line_departing", pivot_field_data.make_line_departing_pivot_field, "Q", 1),
    (
        "chained_sensor_departure",
        pivot_field_data.make_chained_sensor_departure_pivot_field,
        "X",
        2,
    ),
]


@pytest.mark.parametrize(
    "fixture_id, field_factory, target, expected_pivot_count",
    _PIVOT_CASES,
    ids=[case[0] for case in _PIVOT_CASES],
)
def test_pivot_hop_drift_sweep_zero_at_goto_arrival(
    fixture_id, field_factory, target, expected_pivot_count
):
    """AC5 (the pivot gap): a pivot ``goto`` lands the RENDERED chassis drift ≈ 0.

    Covers the PIVOT hop AC5 names but the toy all-hops fixture cannot reach
    (``PIVOT_TURNS == []``). Each ADR-036 pivot fixture routes a single ``goto``
    that takes a real single-wheel pivot arc; the assertions pin, at the goto
    arrival:

    * the RENDERED chassis pose (built ``Track`` at the goto span's ``t_end``)
      lands within :data:`_DRIFT_BOUND_CM` of the destination node — the pivot
      re-anchor nudge (``build`` writes ``chassis.x/y`` directly on the arc) puts
      the axle ON the node, exactly as the plan's gotcha predicts;
    * ``expected_pivot_count`` pivot-arc overlay segments were emitted, so a REAL
      pivot fired (the sweep genuinely exercises the pivot path, not a degraded
      plain turn);
    * the sim localization offset is ≈ 0 (a clean goto integrated no operator
      offset — the return-gate / ``drift_warn`` metric); and
    * NO residual ``drift_warn`` indicator fired.
    """
    field = field_factory()
    assert field.PIVOT_TURNS, f"{fixture_id}: fixture must declare a reachable pivot"

    trace = _goto_trace(field, target)
    track = _build_quiet(trace, field)

    # A real pivot arc fired (overlay segment emitted per taken pivot).
    assert len(track.pivots) == expected_pivot_count, (
        f"{fixture_id}: expected {expected_pivot_count} pivot arc(s), "
        f"got {len(track.pivots)}"
    )

    # The RENDERED chassis lands on the destination node (the nudge surface).
    arrival_drift = _goto_arrival_drift(track, field, [target])
    assert arrival_drift < _DRIFT_BOUND_CM, (
        f"{fixture_id}: rendered chassis {arrival_drift} cm off node {target}"
    )
    # Final keyframe is the same arrival pose — pin it too (belt and braces).
    last = track.keyframes[-1]
    node_x, node_y = field.NODES[target]
    final_drift = hypot(last["x"] - node_x, last["y"] - node_y)
    assert final_drift < _DRIFT_BOUND_CM, (
        f"{fixture_id}: final keyframe {final_drift} cm off node {target}"
    )

    # The sim offset is ≈ 0 at the arrival (independent reference replay through
    # the same step_through — the return-gate / drift_warn metric). A pivot's
    # geometric off-node residual lives in the un-nudged chassis pose, NOT the
    # localization offset (a clean goto re-anchors and integrates no offset).
    _chassis, sim = _fresh_sim(field)
    for _primitive, _snap in sim.step_through(target):
        pass
    offset_mag = sim.localization.get_offset_from_node().magnitude_cm()
    assert offset_mag < _DRIFT_BOUND_CM, (
        f"{fixture_id}: sim offset {offset_mag} cm exceeds bound"
    )

    # No residual drift_warn indicator on the clean pivot mission.
    assert [ev for ev in track.events if ev["type"] == "drift_warn"] == []


# ---------------------------------------------------------------------------
# The DRIFT_THRESHOLD_CM display gate: a clean mission never trips it
# ---------------------------------------------------------------------------


def test_clean_mission_never_triggers_drift_warn():
    """AC5 / plan Notes: no clean mission's built Track carries any ``drift_warn``.

    The residual ``drift_warn`` indicator (the ``DRIFT_THRESHOLD_CM`` display
    gate) must fire ONLY for a raw off-node move nothing re-anchors (a raw drive
    at trace end) or an over-cap off-map fault — NEVER on a clean all-goto
    mission, whatever its final-hop type. This asserts the invariant across every
    clean-mission source the drift sweep uses: the toy all-hops tour (LINE /
    BLIND / PROJECTION / STOP_AT_LINE / reverse) AND every pivot fixture. A
    ``drift_warn`` here would mean a clean goto left the chassis off its node —
    the very bug this file locks out.
    """
    # Toy all-hops tour.
    all_hops = load(_FIXTURES / "ctps_all_hops_drift.json")
    toy_track = _build_quiet(all_hops, toy_field_data)
    assert [ev for ev in toy_track.events if ev["type"] == "drift_warn"] == [], (
        "clean toy all-hops mission fired a drift_warn"
    )

    # Every pivot fixture (single-goto clean mission each).
    for fixture_id, field_factory, target, _n in _PIVOT_CASES:
        field = field_factory()
        track = _build_quiet(_goto_trace(field, target), field)
        assert [ev for ev in track.events if ev["type"] == "drift_warn"] == [], (
            f"clean pivot mission {fixture_id} fired a drift_warn"
        )
