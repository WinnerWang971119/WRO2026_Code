"""Track builder: replay a command-stream :class:`Trace` into a :class:`Track`.

This module turns a loaded command-stream :class:`~tools.path_planner.ctps.trace_loader.Trace`
(the ordered list of ``goto`` / ``drive`` / ``manip`` / ``wait`` / ... commands a
host build of the real ``Mission.cpp`` produced) into a :class:`Track`: a sparse
``(timestamp -> pose)`` keyframe list plus a ``(timestamp -> event)`` narration
list, ready for the self-contained HTML player (T8) to embed and animate.

Geometry comes from the **existing parity-locked** sim, never a re-implementation:

* ``goto`` targets are expanded through
  :meth:`tools.path_planner.sim.runtime.PathTrackerSim.step_through`, which walks
  the precomputed ``NEXT_HOP`` table and yields ``(primitive, snapshot)`` for
  every dispatched primitive. Keyframe poses are read from the **snapshot**, the
  authoritative pose — never recomputed from a primitive's offset argument. The
  LINE_OVERSHOOT step lands the wheel axle ON the intersection node: the follow
  phase stops the axle one sensor-offset short, then the overshoot rolls it onto
  the node (net advance == edge length), matching the firmware re-anchor. A clean
  ``goto`` therefore ends with the chassis on the node (drift ≈ 0).
* Raw ``drive`` moves apply directly to the **shared** :class:`VirtualChassis`
  without advancing the logical ``current_node`` — that desync is exactly what
  the drift check surfaces.

Key invariants (see the CTPS plan, T7):

* Exactly **one** :class:`VirtualChassis` and **one** :class:`PathTrackerSim`
  live for the whole track. They are never reconstructed per ``goto`` — a fresh
  sim would reset accumulated drift, defeating the drift check.
* ``localization.zero_heading_from_current()`` is called **once**, before the
  first command.
* Playback timing is **visual only** (constant velocity from
  :mod:`tools.path_planner.ctps.config`); it is NOT a prediction of real robot
  run time. The running clock is monotonic non-decreasing.

Coordinate / heading convention matches
:mod:`tools.path_planner.sim.chassis`: centimetres, degrees in ``[0, 360)``,
``0 deg = +x`` (East), CCW positive.

The track schema (Data Model B) is intentionally source-agnostic: a future
logged trace (LRS, issue #23) can produce the same :class:`Track` without going
through this builder.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Any, Dict, List

from tools.path_planner.graph_builder import build_graph
from tools.path_planner.planner import compute_all_pairs_next_hop
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import (
    PathTrackerSim,
    UnreachableGoalError,
    wrap_pm180,
)

from tools.path_planner.ctps.config import (
    DRIFT_THRESHOLD_CM,
    MANIP_NOMINAL_S,
    SIM_DRIVE_CM_S,
    SIM_TURN_DEG_S,
)
from tools.path_planner.ctps.trace_loader import Command, Trace

__all__ = ["Track", "build"]


# ---------------------------------------------------------------------------
# Output data model (Data Model B — Track)
# ---------------------------------------------------------------------------


@dataclass
class Track:
    """A fully-expanded, playback-ready trajectory + narration track.

    This is the contract between the Python track builder and the HTML player
    (Data Model B). It is JSON-serialisable via :meth:`to_dict` and is kept
    source-agnostic so a future logged trace can produce the same shape
    without the command-stream builder.

    :param schema_version: Track schema version. Always ``1`` for v1.
    :param field: Static field render data — ``nodes`` (name -> ``[x, y]``),
        ``obstacles`` (``[[x0, y0, x1, y1, kind], ...]``), and ``bounds``
        (``[x_min, y_min, x_max, y_max]`` in centimetres).
    :param duration_s: Total playback duration in seconds — the timestamp of
        the final keyframe (equivalently, the running clock at end of trace).
    :param keyframes: Sparse pose samples, one per primitive boundary / dwell
        endpoint. Each entry is ``{"t": float, "x": float, "y": float,
        "h": float}``. The first entry is always the start pose at ``t = 0``.
        Timestamps are monotonic non-decreasing. The player lerps pose between
        adjacent keyframes (x/y linearly; heading on the shortest arc).
    :param events: Narration / warning markers, each
        ``{"t": float, "type": str, "label": str}``, in emission order. Event
        timestamps are non-decreasing (they share the same running clock as the
        keyframes).
    """

    schema_version: int
    field: Dict[str, Any]
    duration_s: float
    keyframes: List[Dict[str, Any]]
    events: List[Dict[str, Any]]

    def to_dict(self) -> Dict[str, Any]:
        """Return the JSON-serialisable Data Model B dict for this track.

        Nested keyframe/event dicts are shared references (they are never
        mutated after construction), so the player can embed the result
        directly.

        :return: ``{"schema_version", "field", "duration_s", "keyframes",
            "events"}`` — exactly the shape the HTML player (T8) consumes.
        """
        return {
            "schema_version": self.schema_version,
            "field": self.field,
            "duration_s": self.duration_s,
            "keyframes": self.keyframes,
            "events": self.events,
        }


# ---------------------------------------------------------------------------
# Internal helpers (pure)
# ---------------------------------------------------------------------------


def _build_field_block(field_data) -> Dict[str, Any]:
    """Assemble the static ``field`` render block from ``field_data``.

    * ``nodes``: ``{name: [x_cm, y_cm]}`` — JSON lists, not tuples.
    * ``obstacles``: ``[[x_min, y_min, x_max, y_max, kind], ...]`` — adapted
      from ``field_data.OBSTACLES`` (already a list of 5-tuples
      ``(x_min, y_min, x_max, y_max, kind)``), normalised to JSON lists.
    * ``bounds``: ``[0.0, 0.0, width_cm, height_cm]`` where
      ``width/height`` come from ``field_data.FIELD_MM`` (a ``(w_mm, h_mm)``
      pair) divided by 10 to convert millimetres to centimetres.

    :param field_data: validated field-data namespace exposing ``NODES``,
        ``OBSTACLES``, ``FIELD_MM``.
    :return: the ``field`` dict for Data Model B.
    """
    nodes: Dict[str, List[float]] = {
        name: [float(x), float(y)] for name, (x, y) in field_data.NODES.items()
    }

    obstacles: List[List[Any]] = []
    for obs in field_data.OBSTACLES:
        x_min, y_min, x_max, y_max, kind = obs
        obstacles.append(
            [float(x_min), float(y_min), float(x_max), float(y_max), str(kind)]
        )

    width_mm, height_mm = field_data.FIELD_MM
    width_cm = float(width_mm) / 10.0
    height_cm = float(height_mm) / 10.0
    bounds: List[float] = [0.0, 0.0, width_cm, height_cm]

    return {"nodes": nodes, "obstacles": obstacles, "bounds": bounds}


def _pose_delta_duration(
    x_prev: float,
    y_prev: float,
    h_prev: float,
    x_new: float,
    y_new: float,
    h_new: float,
) -> float:
    """Return the visual duration of a geometry step, from previous to new pose.

    Robust pose-delta method (snapshot-only): a primitive is usually pure-turn
    OR pure-move, so one term dominates, but summing the translation and
    rotation durations handles a combined move cleanly too.

    * Translation time: ``hypot(dx, dy) / SIM_DRIVE_CM_S``.
    * Rotation time: ``|wrap_pm180(h_new - h_prev)| / SIM_TURN_DEG_S`` — the
      heading delta wrapped into ``(-180, +180]`` so a turn never costs more
      than the half-circle it actually traverses.

    Both ``SIM_DRIVE_CM_S`` and ``SIM_TURN_DEG_S`` are strictly positive
    constants (see :mod:`tools.path_planner.ctps.config`), so neither divisor
    can be zero. The result is always non-negative, keeping the running clock
    monotonic.

    :param x_prev: previous pose x (cm).
    :param y_prev: previous pose y (cm).
    :param h_prev: previous pose heading (deg).
    :param x_new: new pose x (cm).
    :param y_new: new pose y (cm).
    :param h_new: new pose heading (deg).
    :return: non-negative duration in seconds.
    """
    dist = hypot(x_new - x_prev, y_new - y_prev)
    dh = abs(wrap_pm180(h_new - h_prev))
    return dist / SIM_DRIVE_CM_S + dh / SIM_TURN_DEG_S


def _drive_label(cmd: Command) -> str:
    """Return a human-readable ``method(...)`` label for a ``drive`` command.

    The argument shown matches the method's geometry parameter so the side
    panel reads naturally:

    * ``driveDist(<cm>)`` / ``stopAtLine(<cm>)`` / ``turnDeg(<deg>)`` /
      ``turnToYaw(<deg>)``.
    * any other method (``stop``, unknown) renders as ``<method>()``.

    :param cmd: the ``drive`` command.
    :return: the side-panel label string.
    """
    method = cmd.method
    if method == "driveDist" or method == "stopAtLine":
        return f"{method}({cmd.cm})"
    if method == "turnDeg":
        return f"turnDeg({cmd.deg})"
    if method == "turnToYaw":
        return f"turnToYaw({cmd.deg})"
    return f"{method}()"


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def build(trace: Trace, field_data) -> Track:
    """Replay ``trace`` through the parity-locked sim and return a :class:`Track`.

    Builds the planner graph and all-pairs next-hop table once, constructs a
    single long-lived :class:`VirtualChassis` + :class:`PathTrackerSim`, zeroes
    the heading once, then iterates ``trace.commands`` in order on that shared
    sim/chassis:

    * ``goto`` — expanded through :meth:`PathTrackerSim.step_through`; keyframe
      poses read from each yielded snapshot. After the final hop, a drift check
      compares the chassis pose to ``NODES[current_node]`` and emits a
      ``drift_warn`` event past :data:`DRIFT_THRESHOLD_CM`. An
      :class:`UnreachableGoalError` emits an informational ``goto`` event and
      ends the track.
    * ``drive`` — applied DIRECTLY to the shared chassis without advancing the
      logical node (``driveDist`` -> forward, ``turnDeg`` -> turn_relative,
      ``turnToYaw`` -> turn_to_heading); other methods narrate only.
    * ``manip`` / ``wait`` — emit an event then dwell the clock (same-pose
      keyframe so playback visibly pauses).
    * ``print`` / ``beep`` / ``led`` / ``task_start`` / ``task_end`` —
      zero-dwell events.

    This is a **pure function** of ``(trace, field_data)`` — no file I/O, no
    compiler invocation.

    :param trace: a validated command-stream :class:`Trace` (from
        :func:`tools.path_planner.ctps.trace_loader.load`).
    :param field_data: validated field-data namespace exposing ``NODES``,
        ``OBSTACLES``, ``FIELD_MM``, ``START_NODE``, ``START_HEADING_DEG``,
        ``COST_BLIND_MULTIPLIER``.
    :return: a fully-expanded :class:`Track` whose ``duration_s`` is the final
        running clock value.
    """
    # 1. Planner inputs: build the graph and the all-pairs next-hop table once.
    graph = build_graph(field_data)
    next_hop = compute_all_pairs_next_hop(graph, field_data.COST_BLIND_MULTIPLIER)

    # 2. Exactly ONE chassis + ONE sim for the whole track (drift accumulates
    #    correctly only on a single long-lived pair). Zero the heading once,
    #    before the first command.
    chassis = VirtualChassis.from_field_data(field_data)
    sim = PathTrackerSim(field_data, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    # 3. Seed the keyframe list with the start pose at t = 0, plus an empty
    #    event list and a running clock owned by this function.
    keyframes: List[Dict[str, Any]] = [
        {
            "t": 0.0,
            "x": chassis.x_cm,
            "y": chassis.y_cm,
            "h": chassis.heading_deg,
        }
    ]
    events: List[Dict[str, Any]] = []
    t = 0.0

    # -- clock-advancing closures (single owner of ``t``) -------------------

    def append_geometry_keyframe(x_new: float, y_new: float, h_new: float) -> None:
        """Advance the clock by the pose-delta duration; append a keyframe.

        Uses the most recent keyframe as the previous pose so pure turns,
        pure translations, and combined moves are all timed consistently.
        """
        nonlocal t
        prev = keyframes[-1]
        t += _pose_delta_duration(
            prev["x"], prev["y"], prev["h"], x_new, y_new, h_new
        )
        keyframes.append({"t": t, "x": x_new, "y": y_new, "h": h_new})

    def append_dwell_keyframe(dwell_s: float) -> None:
        """Advance the clock by ``dwell_s``; append a same-pose keyframe.

        Used by ``manip`` / ``wait`` so playback visibly pauses on the command
        (pose unchanged; only time advances). ``dwell_s`` is clamped at 0 so a
        negative duration cannot rewind the monotonic clock.
        """
        nonlocal t
        prev = keyframes[-1]
        t += max(0.0, dwell_s)
        keyframes.append({"t": t, "x": prev["x"], "y": prev["y"], "h": prev["h"]})

    # 4. Iterate the commands in order on the single shared sim/chassis.
    for cmd in trace.commands:
        if cmd.op == "goto":
            from_node = sim.localization.current_node()
            target = cmd.target
            try:
                for _primitive, snap in sim.step_through(target):
                    append_geometry_keyframe(snap.x_cm, snap.y_cm, snap.heading_deg)
            except UnreachableGoalError:
                # The fault point is the node the sim is parked at after any
                # intermediate hops it managed before the dead end.
                fault_from = sim.localization.current_node()
                events.append(
                    {
                        "t": t,
                        "type": "goto",
                        "label": f"UNREACHABLE {fault_from}->{target}",
                    }
                )
                # Stop processing further commands and end the track.
                break

            # A successful goto intentionally emits NO discrete command event:
            # the per-hop keyframes appended above ARE its narration — the player
            # animates the chassis driving the route. The event list is reserved
            # for what the motion alone cannot convey: the UNREACHABLE fault above
            # and the drift warning below. (The player's `goto` event renderer
            # serves that fault banner, not a per-hop caption for healthy moves.)
            # Drift check after the final hop: a clean goto lands ON the node
            # (drift ~ 0); a preceding raw drive injects an offset that goto
            # does not correct.
            node = sim.localization.current_node()
            nx_, ny_ = field_data.NODES[node]
            drift = hypot(chassis.x_cm - nx_, chassis.y_cm - ny_)
            if drift > DRIFT_THRESHOLD_CM:
                events.append(
                    {
                        "t": t,
                        "type": "drift_warn",
                        "label": f"chassis {drift:.1f} cm off node {node}",
                    }
                )
            # ``from_node`` is captured for symmetry with the UNREACHABLE label
            # path; a clean goto needs no banner.
            _ = from_node

        elif cmd.op == "drive":
            # Raw move applied DIRECTLY to the shared chassis; never advance the
            # logical node. Narration is timestamped at the command's start
            # (before any geometry advance).
            event_t = t
            method = cmd.method
            if method == "driveDist" or method == "stopAtLine":
                # Both carry a forward distance in ``cm``. For ``stopAtLine``
                # that is the nominal ``expectedCm`` to the line crossing — the
                # robot drives ~that far before the sensor trips, so it is the
                # best playback estimate of the advance. (Drift from the true
                # stop point is unmodeled; see the CTPS plan for a future
                # estimate-tagged keyframe.)
                chassis.forward(float(cmd.cm))
                append_geometry_keyframe(
                    chassis.x_cm, chassis.y_cm, chassis.heading_deg
                )
            elif method == "turnDeg":
                chassis.turn_relative(float(cmd.deg))
                append_geometry_keyframe(
                    chassis.x_cm, chassis.y_cm, chassis.heading_deg
                )
            elif method == "turnToYaw":
                chassis.turn_to_heading(float(cmd.deg))
                append_geometry_keyframe(
                    chassis.x_cm, chassis.y_cm, chassis.heading_deg
                )
            # else: stop / unknown -> no geometry change.
            events.append(
                {"t": event_t, "type": "drive", "label": _drive_label(cmd)}
            )

        elif cmd.op == "manip":
            events.append(
                {"t": t, "type": "manip", "label": f"{cmd.method}({cmd.arg})"}
            )
            append_dwell_keyframe(MANIP_NOMINAL_S)

        elif cmd.op == "wait":
            events.append({"t": t, "type": "wait", "label": f"wait {cmd.ms} ms"})
            append_dwell_keyframe((cmd.ms or 0) / 1000.0)

        elif cmd.op == "print":
            events.append({"t": t, "type": "print", "label": cmd.text})

        elif cmd.op == "beep":
            events.append(
                {
                    "t": t,
                    "type": "beep",
                    "label": f"beep {cmd.freq_hz} Hz {cmd.ms} ms",
                }
            )

        elif cmd.op == "led":
            events.append(
                {"t": t, "type": "led", "label": f"led ({cmd.r},{cmd.g},{cmd.b})"}
            )

        elif cmd.op == "task_start":
            events.append({"t": t, "type": "task_start", "label": cmd.name})

        elif cmd.op == "task_end":
            events.append(
                {"t": t, "type": "task_end", "label": f"{cmd.name} {cmd.result}"}
            )

        # Any other op cannot reach here — the loader excludes unknown ops.

    # 5. Build-error banner (informational). Emit at the end regardless of
    #    whether the track was truncated by an unreachable goal.
    if trace.build_error:
        events.append(
            {
                "t": t,
                "type": "print",
                "label": "mission build error — trace truncated",
            }
        )

    return Track(
        schema_version=1,
        field=_build_field_block(field_data),
        duration_s=t,
        keyframes=keyframes,
        events=events,
    )
