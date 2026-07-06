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

from dataclasses import dataclass, field as dc_field
from math import ceil, cos, hypot, radians, sin
from typing import Any, Dict, List, Optional, Tuple

from tools.path_planner.compiler import ACT_STOP_AT_LINE, ACT_TURN_REL
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.pipeline_config import TurnMode
from tools.path_planner.planner import (
    BlindDecayParams,
    compute_all_pairs_next_hop,
    parse_track_cm_from_config,
)
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import (
    ACT_LINE_OVERSHOOT,
    ACT_LINE_PASS,
    ACT_REANCHOR,
    ACT_STOP_AT_LINE_REVERSE,
    ACT_WALL_DEPART,
    ACT_WALL_STOP,
    PathTrackerSim,
    UnreachableGoalError,
    wrap_pm180,
)

from tools.path_planner.ctps.config import (
    ARC_STEP_DEG,
    DRIFT_THRESHOLD_CM,
    MANIP_NOMINAL_S,
    SIM_DRIVE_CM_S,
    SIM_TURN_DEG_S,
)
from tools.path_planner.ctps.trace_loader import Command, Trace

__all__ = ["Track", "build"]


# ---------------------------------------------------------------------------
# Drive-method families (shared by ``build`` and ``_drive_label``)
# ---------------------------------------------------------------------------

#: ``drive`` methods that carry a forward distance in ``cm`` and advance the
#: shared chassis along its heading. ``lineFollowDist`` joins the original
#: ``driveDist`` / ``stopAtLine`` / ``wallStop`` trio (T7): all four render a
#: forward advance (``chassis.forward``) — or a backward one
#: (``chassis.reverse``) when ``reverse`` is set.
_FORWARD_GEOMETRY_METHODS: frozenset[str] = frozenset(
    {"driveDist", "stopAtLine", "wallStop", "lineFollowDist"}
)

#: The subset of :data:`_FORWARD_GEOMETRY_METHODS` that honours a GYRO held
#: heading via the two-phase (rotate-in-place then advance) render.
#: ``lineFollowDist`` is excluded — the line follower regulates its own
#: heading, so a recorded heading never turns it in place.
_GYRO_HEADING_METHODS: frozenset[str] = frozenset(
    {"driveDist", "stopAtLine", "wallStop"}
)


def _honors_gyro_heading(cmd) -> bool:
    """Whether the render turns in place to ``cmd.heading`` before advancing.

    The two-phase (rotate-in-place, then advance) render fires only for a
    forward-geometry method in :data:`_GYRO_HEADING_METHODS`, under an explicit
    ``mode == "gyro"``, carrying a recorded ``heading``. The renderer and the
    side-panel label both gate on this single predicate so the ``heading=``
    token never advertises a heading the drawn trajectory does not follow — a
    SYNC move ignores ``setHeading`` on the real robot.

    :param cmd: the ``drive`` command.
    :return: ``True`` iff the heading is honoured (and rendered) two-phase.
    """
    return (
        cmd.method in _GYRO_HEADING_METHODS
        and cmd.mode == "gyro"
        and cmd.heading is not None
    )


# ---------------------------------------------------------------------------
# Pivot-arc detection tag sets (issue #55)
# ---------------------------------------------------------------------------

#: Departure-hop LANDING primitive tags that re-anchor via a SENSOR
#: (``on_intersection``) rather than dead-reckoning onto the node. The sim
#: takes no departing trim on a sensor hop, so a pivot departure over one of
#: these dead-reckons the departing ``trim_cm`` PAST its true landing along
#: the departure-edge ray; the real robot re-anchors on the destination line
#: (ADR-044 D1), so the departure landing is pulled BACK by exactly that trim
#: along the ray — onto the node in the plain case, and the destination's own
#: approach trim SHORT of it when the destination is itself a chained pivot
#: the route turns at. Any OTHER landing tag is distance-terminated and lands
#: on-node natively via the sim's delta-trimmed leg, so it needs no
#: correction. (Pivots are FRONT-only in v1 (#54), so the REAR reverse tags
#: never actually fire on a pivot departure; they are listed for a complete
#: on_intersection-landing set.)
_SENSOR_LANDING_TAGS: frozenset[str] = frozenset(
    {
        ACT_LINE_OVERSHOOT,
        ACT_LINE_PASS,
        ACT_STOP_AT_LINE,
        ACT_STOP_AT_LINE_REVERSE,
    }
)

#: Wall-zero reset interlude primitive tags. If any of these fired at the pivot
#: node between the approach and the departing turn, the reset has already
#: snapped the pose (destroying the ``c0`` arc-start premise), so the arc is
#: skipped and the plain in-place turn is rendered instead.
_WALL_RESET_TAGS: frozenset[str] = frozenset(
    {ACT_WALL_STOP, ACT_REANCHOR, ACT_WALL_DEPART}
)


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
        ``obstacles`` (``[[x0, y0, x1, y1, kind], ...]``), ``bounds``
        (``[x_min, y_min, x_max, y_max]`` in centimetres), and
        ``robot_radius_cm`` (the robot's C-space radius, for the world-sized
        robot marker).
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
        keyframes). A successful ``goto`` additionally carries ``"t_end":
        float`` (>= ``"t"``), spanning the whole multi-hop move; the
        UNREACHABLE fault ``goto`` event has no ``"t_end"``.
    :param pivots: Optional single-wheel pivot-arc overlay segments (issue #55),
        one per pivot the trace's routes actually take, sorted by ``t0``. Each
        entry is ``{"t0": float, "t1": float, "cx": float, "cy": float,
        "theta_deg": float, "wheel": "left"|"right"}`` — the arc-span
        timestamps, the planted-wheel world centre (rotation centre), the signed
        swing angle (CCW positive, the ``ACT_TURN_REL`` delta), and the pivot
        wheel side. Empty (and omitted from :meth:`to_dict`) for every
        pivot-free trace, so ``schema_version`` stays ``1`` (additive optional
        field). The dense arc keyframes between ``t0`` and ``t1`` already trace
        the swing through the player's lerp; this segment just tells the
        ``wheel-arc`` render mode WHEN a pivot is active and WHERE its wheel is.
    """

    schema_version: int
    field: Dict[str, Any]
    duration_s: float
    keyframes: List[Dict[str, Any]]
    events: List[Dict[str, Any]]
    pivots: List[Dict[str, Any]] = dc_field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Return the JSON-serialisable Data Model B dict for this track.

        Nested keyframe/event dicts are shared references (they are never
        mutated after construction), so the player can embed the result
        directly.

        The ``"pivots"`` key is included **only when non-empty**: a pivot-free
        trace emits exactly the pre-#55 dict (same keys, same order), so its
        Track JSON is byte-identical to before this change (AC6).

        :return: ``{"schema_version", "field", "duration_s", "keyframes",
            "events"}`` — plus ``"pivots"`` when the trace took at least one
            pivot — exactly the shape the HTML player consumes.
        """
        result: Dict[str, Any] = {
            "schema_version": self.schema_version,
            "field": self.field,
            "duration_s": self.duration_s,
            "keyframes": self.keyframes,
            "events": self.events,
        }
        if self.pivots:
            result["pivots"] = self.pivots
        return result


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
    * ``robot_radius_cm``: ``field_data.ROBOT_RADIUS_CM`` (the planner's
      C-space inflation radius), so the player can draw the robot marker at
      its true world size (a square of side ``2 * radius``) instead of a
      fixed pixel box. ``0.0`` when the field omits it.

    :param field_data: validated field-data namespace exposing ``NODES``,
        ``OBSTACLES``, ``FIELD_MM``, ``ROBOT_RADIUS_CM``.
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

    robot_radius_cm = float(getattr(field_data, "ROBOT_RADIUS_CM", 0.0) or 0.0)

    return {
        "nodes": nodes,
        "obstacles": obstacles,
        "bounds": bounds,
        "robot_radius_cm": robot_radius_cm,
    }


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


def _build_pivot_arc(
    x0: float,
    y0: float,
    h0_deg: float,
    theta_deg: float,
    t0: float,
    radius_cm: float,
    wheel_sign: int,
) -> Tuple[List[Dict[str, Any]], Tuple[float, float], Tuple[float, float], float]:
    """Sample a rigid single-wheel pivot rotation into dense arc keyframes.

    The chassis centre ``(x0, y0)`` rotates rigidly about the planted (braked)
    pivot wheel through ``theta_deg`` (signed, CCW positive). The planted wheel
    — the rotation centre ``W`` — sits ``radius_cm`` off the chassis centre,
    ``90 deg`` to the ``wheel_sign`` side of the incoming heading ``h0_deg``
    (``+1`` = left wheel / ``PIVOT_LEFT``, ``-1`` = right wheel /
    ``PIVOT_RIGHT``)::

        W        = (x0, y0) + radius * (cos(h0 + sign*90), sin(h0 + sign*90))
        N        = max(1, ceil(|theta| / ARC_STEP_DEG))
        center_k = W + Rot(tau*theta) * ((x0, y0) - W)      tau = k / N, k = 1..N
        heading_k= normalize(h0 + tau*theta)

    where ``Rot(a)*(dx, dy) = (dx*cos a - dy*sin a, dx*sin a + dy*cos a)``.

    Each sub-keyframe advances the clock by the SAME pure-rotation slice
    ``(|theta| / N) / SIM_TURN_DEG_S`` (a turning box, not a driving one), so the
    whole arc occupies ``|theta| / SIM_TURN_DEG_S`` — the exact visual duration
    the single in-place turn keyframe had today. Timestamps are pinned to
    ``t0 + k*dt`` (never accumulated) so the running clock stays strictly
    monotonic and the total is exact.

    The first sample is ``k = 1`` (NOT ``k = 0``): the arc-start pose is the
    already-present previous keyframe at ``(x0, y0, h0)``, so these ``N`` samples
    REPLACE the single in-place turn keyframe (``N - 1`` intermediate keyframes
    plus the arc-end pose).

    Pure and side-effect free — the caller decides where to append the samples,
    where to place the wheel pin, and whether/where to nudge the chassis.

    :param x0: arc-start chassis-centre x (cm) — the previous keyframe.
    :param y0: arc-start chassis-centre y (cm).
    :param h0_deg: incoming heading (deg) — the previous keyframe's heading.
    :param theta_deg: signed swing angle (deg, CCW positive) — the
        ``ACT_TURN_REL`` delta.
    :param t0: timestamp of the arc-start keyframe (the running clock).
    :param radius_cm: pivot radius ``track_cm / 2`` (cm).
    :param wheel_sign: ``+1`` for a left-wheel pivot, ``-1`` for a right-wheel.
    :return: ``(arc_keyframes, (wx, wy), (cx_end, cy_end), t_end)`` — the ``N``
        replacement keyframes, the planted-wheel world centre, the arc-end
        chassis centre, and the running clock after the arc.
    """
    wheel_ang_rad = radians(h0_deg + wheel_sign * 90.0)
    wx = x0 + radius_cm * cos(wheel_ang_rad)
    wy = y0 + radius_cm * sin(wheel_ang_rad)

    abs_theta = abs(theta_deg)
    n = max(1, ceil(abs_theta / ARC_STEP_DEG))
    dt = (abs_theta / n) / SIM_TURN_DEG_S

    # Offset of the chassis centre from the planted wheel, rotated by tau*theta.
    rx = x0 - wx
    ry = y0 - wy

    arc: List[Dict[str, Any]] = []
    cx_end, cy_end = x0, y0
    for k in range(1, n + 1):
        tau = k / n
        ang_rad = radians(tau * theta_deg)
        cos_a = cos(ang_rad)
        sin_a = sin(ang_rad)
        cx = wx + (rx * cos_a - ry * sin_a)
        cy = wy + (rx * sin_a + ry * cos_a)
        heading = (h0_deg + tau * theta_deg) % 360.0
        arc.append({"t": t0 + k * dt, "x": cx, "y": cy, "h": heading})
        cx_end, cy_end = cx, cy

    return arc, (wx, wy), (cx_end, cy_end), t0 + n * dt


def _drive_label(cmd: Command) -> str:
    """Return a human-readable ``method(...)`` label for a ``drive`` command.

    The argument shown matches the method's geometry parameter so the side
    panel reads naturally:

    * ``driveDist(<cm>)`` / ``stopAtLine(<cm>)`` / ``wallStop(<cm>)`` /
      ``lineFollowDist(<cm>)`` / ``turnDeg(<deg>)`` / ``turnToYaw(<deg>)``.
    * a cm-less forward-geometry move (legacy/partial trace) drops the argument
      (``wallStop()``); any other method (``stop``, unknown) renders as
      ``<method>()``.

    For the forward-geometry methods, the captured ``mode`` / ``reverse`` are
    **appended** only when set, and ``heading`` only when the render actually
    honours it (the two-phase gyro gate — see :func:`_honors_gyro_heading`), so
    the label never advertises a heading the drawn path does not follow (e.g.
    ``driveDist(35) mode=gyro heading=0 reverse``). A plain move — with none of
    those set — keeps its exact base label (``driveDist(35)``).

    :param cmd: the ``drive`` command.
    :return: the side-panel label string.
    """
    method = cmd.method
    if method in _FORWARD_GEOMETRY_METHODS:
        # Omit the argument on the cm-less path (legacy/partial trace) so it reads
        # `wallStop()` rather than the literal `wallStop(None)`.
        label = f"{method}({cmd.cm})" if cmd.cm is not None else f"{method}()"
        # Append captured drive state ONLY when it matches the drawn path, so the
        # narration never contradicts the trajectory. `mode` / `reverse` are
        # shown when set; `heading` only when the render actually honours it (the
        # two-phase gyro gate) — a heading recorded without it (e.g. a SYNC move)
        # is ignored by both the render and the robot, so it must not appear.
        if cmd.mode is not None:
            label += f" mode={cmd.mode}"
        if _honors_gyro_heading(cmd):
            label += f" heading={cmd.heading}"
        if cmd.reverse:
            label += " reverse"
        return label
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
      logical node. The forward-geometry methods (``driveDist`` /
      ``stopAtLine`` / ``wallStop`` / ``lineFollowDist``) advance along the
      current heading — ``chassis.reverse`` when ``reverse`` is set, else
      ``chassis.forward`` — and narrate-only (no geometry) when ``cm`` is
      absent (legacy/partial trace). A ``mode == "gyro"`` move with an explicit
      ``heading`` (``driveDist`` / ``stopAtLine`` / ``wallStop``, NOT
      ``lineFollowDist``) renders two-phase: ``turn_to_heading(H)`` +
      a same-position keyframe, then the advance keyframe — "align to H, then
      advance along H". ``turnDeg`` -> turn_relative, ``turnToYaw`` ->
      turn_to_heading; other methods narrate only.
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
        and the ``BLIND_DECAY_*`` decay knobs.
    :return: a fully-expanded :class:`Track` whose ``duration_s`` is the final
        running clock value.
    """
    # 1. Planner inputs: build the graph and the all-pairs next-hop table once.
    graph = build_graph(field_data)
    next_hop = compute_all_pairs_next_hop(
        graph, BlindDecayParams.from_field_data(field_data)
    )

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

    # Optional single-wheel pivot-arc overlay segments (issue #55). Appended in
    # chronological (t0-increasing) order as each goto's route takes a declared
    # pivot; stays empty — and the ``pivots`` key is omitted from the Track — for
    # every pivot-free trace, so a ``PIVOT_TURNS``-empty field is byte-identical
    # to before this change (AC6).
    pivots: List[Dict[str, Any]] = []
    # Chassis track width (cm), parsed LAZILY from ConfigDrive.h on the FIRST
    # detected pivot only. A pivot-free build never reads the firmware header
    # (issue #44 fail-loud parse), preserving the byte-identical guarantee.
    track_cm: Optional[float] = None

    # -- clock-advancing closures (primary writers of ``t``) ----------------
    # The pivot-arc branch below is the one OTHER writer of ``t``: it assigns
    # the ``_build_pivot_arc`` end time directly (same function scope, same
    # monotonic clock — the arc replaces the in-place turn keyframe these
    # closures would otherwise have timed).

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

    def append_advance_keyframe(distance_cm: float, reverse: bool) -> None:
        """Translate the chassis along its heading, then append a keyframe.

        Advances the shared chassis by ``distance_cm`` — backward
        (``chassis.reverse``) when ``reverse`` is truthy, forward
        (``chassis.forward``) otherwise — then appends the resulting pose. Kept
        as one helper so the reverse ternary is written once and reused by both
        the plain and the GYRO two-phase drive paths.
        """
        if reverse:
            chassis.reverse(distance_cm)
        else:
            chassis.forward(distance_cm)
        append_geometry_keyframe(chassis.x_cm, chassis.y_cm, chassis.heading_deg)

    # 4. Iterate the commands in order on the single shared sim/chassis.
    for cmd in trace.commands:
        if cmd.op == "goto":
            from_node = sim.localization.current_node()
            target = cmd.target
            t_start = t

            # --- per-goto pivot-arc detection state ----------------------
            # Reset per goto, mirroring the firmware's per-goal pivot-latch
            # reset (`_reset_pivot_latch`): a pivot fires only when the
            # approach `u->v` AND the departure `v->w` fall inside the SAME
            # goto, so a goto that merely STARTS on a pivot node (no tracked
            # predecessor) degrades to a plain turn, exactly as on the robot.
            cur_node = from_node
            prev_node: Optional[str] = None        # predecessor of `cur_node`
            wall_reset_at_node = False             # an ACT_WALL_* fired at cur_node
            # Pivot departure awaiting a landing: ``(w, trim_cm, ux, uy)`` —
            # the departure hop's destination, the DEPARTING pivot's trim, and
            # the departure-edge heading unit ray (u-hat-1).
            pending_pivot_land: Optional[Tuple[str, float, float, float]] = None
            try:
                for primitive, snap in sim.step_through(target):
                    tag = primitive[0]
                    node_now = sim.localization.current_node()
                    if node_now != cur_node:
                        # A hop just re-anchored onto a new node: the OLD current
                        # node is this one's predecessor, and any wall reset seen
                        # at the old node no longer applies.
                        prev_node = cur_node
                        cur_node = node_now
                        wall_reset_at_node = False
                    if tag in _WALL_RESET_TAGS:
                        wall_reset_at_node = True

                    arced = False
                    if tag == ACT_TURN_REL and not wall_reset_at_node:
                        # One-hop lookahead, mirroring the firmware key
                        # `findPivotTurn(cur, nxt, getNextHop(nxt, dest))`
                        # (ADR-036). `w` is a SINGLE `None`-safe dict index —
                        # NEVER `walk_next_hop`, which raises
                        # `UnreachableGoalError` eagerly and would drop the
                        # reachable-prefix keyframes + change the UNREACHABLE
                        # label. A miss (no predecessor at the goto's first turn,
                        # a non-pivot turn, `w is None` at the terminal node, or
                        # an empty `PIVOT_TURNS`) renders the turn unchanged.
                        w = next_hop[cur_node].get(target)
                        record = sim.find_pivot_turn(prev_node, cur_node, w)
                        if record is not None:
                            arced = True
                            # Parse the track width ONLY on the pivot path.
                            if track_cm is None:
                                track_cm = parse_track_cm_from_config()
                            theta_deg = float(primitive[1])
                            start_kf = keyframes[-1]
                            wheel_sign = (
                                1 if record.wheel == TurnMode.PIVOT_LEFT else -1
                            )
                            arc, (wx, wy), (cx_end, cy_end), t = _build_pivot_arc(
                                start_kf["x"],
                                start_kf["y"],
                                start_kf["h"],
                                theta_deg,
                                start_kf["t"],
                                track_cm / 2.0,
                                wheel_sign,
                            )
                            # The N arc sub-keyframes REPLACE the single in-place
                            # turn keyframe (which is never appended here).
                            keyframes.extend(arc)
                            # One overlay segment per taken pivot, appended in
                            # chronological (t0-increasing) order — already sorted.
                            pivots.append(
                                {
                                    "t0": start_kf["t"],
                                    "t1": t,
                                    "cx": wx,
                                    "cy": wy,
                                    "theta_deg": theta_deg,
                                    "wheel": (
                                        "left"
                                        if record.wheel == TurnMode.PIVOT_LEFT
                                        else "right"
                                    ),
                                }
                            )
                            # Chassis nudge: land the shared chassis on the arc
                            # end so the sim's existing delta-trimmed departure
                            # leg (dispatched on the NEXT yield) drives from the
                            # true pivot exit and lands on-node — killing the
                            # ~12.7 cm in-place-turn drift (AC2). Parity-safe for
                            # two reasons: (a) `_walk_hops` reads chassis state
                            # only via `heading_deg` for dispatch
                            # (runtime.py:2693), so an x/y write cannot change the
                            # primitive stream; and (b) a raw `chassis.x_cm/y_cm`
                            # write never reaches `LocalizationSim` — its offset
                            # grows ONLY through `accumulate_operator_move`, which
                            # this does not call. (This is NOT a `nav_active`
                            # gate: the write simply never feeds the integrator.)
                            # TC7 is the primitive-stream-unchanged tripwire.
                            chassis.x_cm = cx_end
                            chassis.y_cm = cy_end
                            # Arm the sensor-departure landing correction: the
                            # arc-end heading IS the departure-edge heading
                            # (the pre-edge turn faces the edge exactly), so
                            # its unit vector is the ray any sensor-landing
                            # residual lies along.
                            depart_rad = radians(arc[-1]["h"])
                            pending_pivot_land = (
                                w,
                                record.trim_cm,
                                cos(depart_rad),
                                sin(depart_rad),
                            )

                    if not arced:
                        append_geometry_keyframe(snap.x_cm, snap.y_cm, snap.heading_deg)
                        if (
                            pending_pivot_land is not None
                            and node_now == pending_pivot_land[0]
                        ):
                            # The pivot's departure hop just landed on `w`.
                            if tag in _SENSOR_LANDING_TAGS:
                                # Sensor-terminated departure (LINE-out / seek):
                                # the sim takes no departing trim, so this leg
                                # dead-reckons from `c_end` — the departing
                                # `trim_cm` PAST its true landing along the
                                # departure-edge ray (purely along-edge, no
                                # lateral). The real robot re-anchors on `w`'s
                                # line (ADR-044 D1), so pull the departure
                                # keyframe — and the shared chassis — BACK by
                                # exactly that trim along the ray. Plain case:
                                # the sim netted the full edge, so this lands
                                # exactly ON `w`. Chained case (`w` is ITSELF a
                                # pivot the route turns at): the sim already
                                # shortened the landing by `w`'s approach trim,
                                # so this lands that much SHORT of `w` — the
                                # true physical pose the next arc must start
                                # from. Never snap to `w`'s coordinates: an
                                # absolute snap erases the approach trim and
                                # translates the whole tail by it. The arc
                                # still ends at `c_end`; only the landing moves.
                                # Accepted cosmetic: on a chained-sensor leg the
                                # mid-hop follow keyframe keeps the un-pulled
                                # dead-reckon, so the box plays a small backward
                                # hitch onto the corrected landing. Playback
                                # timing is visual-only, so this is left as-is.
                                _w, trim_cm, ux, uy = pending_pivot_land
                                land_x = keyframes[-1]["x"] - trim_cm * ux
                                land_y = keyframes[-1]["y"] - trim_cm * uy
                                keyframes[-1]["x"] = land_x
                                keyframes[-1]["y"] = land_y
                                chassis.x_cm = land_x
                                chassis.y_cm = land_y
                            pending_pivot_land = None
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

            # A successful goto emits a `goto` SPAN event covering the whole
            # multi-hop move (`t_start` -> `t`, the running clock before/after
            # `step_through`): the per-hop keyframes appended above remain its
            # motion narration, and this span lets the GOTO timeline lane (and
            # the Log / Current-command panels) caption the healthy move too.
            # `t_start` is `t` before any hop, so it is >= every prior event's
            # `t`, keeping `events` sorted; `t` here is >= `t_start`.
            events.append(
                {
                    "t": t_start,
                    "type": "goto",
                    "t_end": t,
                    "label": f"{from_node}->{target}",
                }
            )
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

        elif cmd.op == "drive":
            # Raw move applied DIRECTLY to the shared chassis; never advance the
            # logical node. Narration is timestamped at the command's start
            # (before any geometry advance).
            event_t = t
            method = cmd.method
            if method in _FORWARD_GEOMETRY_METHODS:
                # driveDist / stopAtLine / wallStop / lineFollowDist all carry a
                # forward distance in ``cm``. For stopAtLine / wallStop /
                # lineFollowDist that value is only nominal — the real robot
                # stops on a sensor / physical wall / line-follow distance gate
                # rather than exact odometry — but replaying the requested
                # advance is still the best lightweight host estimate.
                if cmd.cm is None:
                    # Legacy / partial trace (e.g. a cm-less wallStop or
                    # lineFollowDist): no distance to replay. Narrate only —
                    # never ``float(None)``. Kept even though the recorder now
                    # emits ``cm`` for every forward-geometry method, to defend
                    # partial/legacy traces.
                    pass
                elif _honors_gyro_heading(cmd):
                    # GYRO + explicit heading → two-phase render: first rotate
                    # in place to the held heading (a keyframe at the unchanged
                    # position), THEN advance / back up along it (the
                    # translation keyframe). Heading is honoured ONLY under
                    # ``mode == "gyro"`` — firmware consults setHeading only in
                    # GYRO, so a heading recorded without GYRO must render along
                    # the current facing (the plain branch below).
                    chassis.turn_to_heading(float(cmd.heading))
                    append_geometry_keyframe(
                        chassis.x_cm, chassis.y_cm, chassis.heading_deg
                    )
                    append_advance_keyframe(float(cmd.cm), bool(cmd.reverse))
                else:
                    # Plain advance along the CURRENT facing (non-gyro, or no
                    # heading, or lineFollowDist). Byte-identical to the pre-T7
                    # forward render for the non-reverse case.
                    append_advance_keyframe(float(cmd.cm), bool(cmd.reverse))
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
        pivots=pivots,
    )
