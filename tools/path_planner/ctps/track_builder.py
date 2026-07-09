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
* Raw off-node ``drive`` moves route through
  :meth:`~tools.path_planner.sim.runtime.PathTrackerSim.operator_move` (issue
  #130, T11), so the operator offset integrates on the SHARED sim exactly as it
  does on the robot — the raw move mutates the shared :class:`VirtualChassis`
  AND grows the localization offset (subject to the sim's nav-active gate +
  deadbands). The move renders its own physical excursion, then the NEXT
  ``goto``'s :meth:`~tools.path_planner.sim.runtime.PathTrackerSim.step_through`
  yields the firmware-faithful **return-to-node prologue** (a pre-edge
  ``ACT_TURN_REL`` then ``ACT_RETURN_FORWARD`` dead-reckon or
  ``ACT_RETURN_STOP_AT_LINE`` line-seek) BEFORE the planned walk. CTPS renders
  that prologue (profile-timed like any other primitive) and emits a brief
  ``return_to_node`` localization-lane marker; the goto re-anchors, so a raw
  move FOLLOWED by a goto leaves NO residual drift indicator. A residual
  ``drift_warn`` survives only for a raw move at trace end (nothing re-anchors
  it) or an over-cap off-map excursion (``OffMapFaultError`` under the FAULT
  gate ends the track; a ``BEST_EFFORT`` capped return renders with the marker).
* ``declare_position`` ops apply a belief reset via
  :meth:`~tools.path_planner.sim.runtime.LocalizationSim.declare_position`
  (change node, clear offset, preserve heading calibration) and emit a
  ``declare_position`` localization-lane marker — semantically distinct from
  return-to-node (operator-declared belief reset vs. physical correction). An
  unknown node warns + no-ops in the sim, so CTPS emits NO marker for it.

Key invariants (see the CTPS plan, T7 / T11):

* Exactly **one** :class:`VirtualChassis` and **one** :class:`PathTrackerSim`
  live for the whole track. They are never reconstructed per ``goto`` — a fresh
  sim would reset accumulated drift, defeating the drift check.
* ``localization.zero_heading_from_current()`` is called **once**, before the
  first command.
* Playback timing is **profile-derived** (each geometry step is densified off
  the firmware T-shape / turn profile in
  :mod:`tools.path_planner.ctps.profile`, so the render accelerates, cruises,
  decelerates, and stops as the robot would); it is NOT a prediction of
  absolute real-robot run time (issue #130). The running clock is monotonic
  non-decreasing.

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

from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_STOP_AT_LINE,
    ACT_TURN_REL,
)
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.pipeline_config import TurnMode
from tools.path_planner.planner import (
    BlindDecayParams,
    compute_all_pairs_next_hop,
    parse_track_cm_from_config,
)
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import (
    ACT_BLIND_REVERSE,
    ACT_FOLLOW_DIST_PASS,
    ACT_HEADING_RESET,
    ACT_LINE_OVERSHOOT,
    ACT_LINE_PASS,
    ACT_LINE_REVERSE,
    ACT_PROJECTION_REVERSE,
    ACT_PROJECTION_REVERSE_PASS,
    ACT_REANCHOR,
    ACT_RETURN_FORWARD,
    ACT_RETURN_STOP_AT_LINE,
    ACT_STOP_AT_LINE_REVERSE,
    ACT_WALL_DEPART,
    ACT_WALL_STOP,
    Direction,
    OffMapFaultError,
    PathTrackerSim,
    UnreachableGoalError,
    wrap_pm180,
)

from tools.path_planner.ctps.config import (
    ARC_STEP_DEG,
    MANIP_NOMINAL_S,
    ROBOT_FRAME_CM,
)
from tools.path_planner.ctps.profile import (
    default_config,
    resolve_drive_profile,
    resolve_effective_speed,
    resolve_turn_profile,
    sample_drive,
    sample_turn,
)
from tools.path_planner.ctps.trace_loader import Command, Trace

__all__ = ["Track", "build", "build_field_block"]


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


#: goto direction tags -> the sim's :class:`Direction`. A ``goto`` carries an
#: ``arrival`` / ``route`` tag ("front"/"rear") only when the operator called
#: ``setArrivalDirection`` / ``setRouteDirection``; an absent tag (``None``) or
#: any unrecognised value maps to ``AUTO`` (defer to the sim's bidirectional
#: resolution), so a direction-free goto is unchanged.
_DIRECTION_BY_TAG = {"front": Direction.FRONT, "rear": Direction.REAR}


def _direction_from_tag(tag) -> "Direction":
    """Map a goto ``arrival`` / ``route`` tag to a :class:`Direction`; AUTO otherwise."""
    return _DIRECTION_BY_TAG.get(tag, Direction.AUTO)


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
        UNREACHABLE fault ``goto`` event has no ``"t_end"``. A ``manip`` event
        additionally carries structured ``"method": str`` / ``"arg": int``
        keys (additive to the unchanged ``"label"`` — CTPS object-cargo plan,
        T9), so the player can dispatch on them directly instead of parsing
        the label string. No other event type gains new keys, so a
        manip-free trace's events stay byte-identical.
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


def build_field_block(field_data) -> Dict[str, Any]:
    """Assemble the static ``field`` render block from ``field_data``.

    * ``nodes``: ``{name: [x_cm, y_cm]}`` — JSON lists, not tuples.
    * ``obstacles``: ``[[x_min, y_min, x_max, y_max, kind], ...]`` — adapted
      from ``field_data.OBSTACLES`` (already a list of 5-tuples
      ``(x_min, y_min, x_max, y_max, kind)``), normalised to JSON lists.
    * ``bounds``: ``[0.0, 0.0, width_cm, height_cm]`` where
      ``width/height`` come from ``field_data.FIELD_MM`` (a ``(w_mm, h_mm)``
      pair) divided by 10 to convert millimetres to centimetres.
    * ``robot_radius_cm``: ``field_data.ROBOT_RADIUS_CM`` (the planner's
      C-space inflation radius). ``0.0`` when the field omits it. Retained for
      a possible C-space overlay; the drawn robot marker uses
      ``robot_frame_cm`` (below), NOT this radius.
    * ``robot_frame_cm``: :data:`~tools.path_planner.ctps.config.ROBOT_FRAME_CM`
      — the robot's TRUE physical chassis footprint (square side, cm). The
      player and layout editor draw the robot marker at this size, decoupled
      from the C-space radius above.

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
        "robot_frame_cm": float(ROBOT_FRAME_CM),
    }


#: A translation below this magnitude (cm) is treated as "no advance", so the
#: geometry step is a pure in-place turn and is timed by the turn profile
#: (:func:`tools.path_planner.ctps.profile.sample_turn`) rather than the drive
#: profile. A genuine in-place turn leaves ``x``/``y`` byte-unchanged, so this
#: only needs to swallow float noise, not a real (however short) advance — the
#: sub-``TSHAPE_MIN_MOVE_CM`` flat-cruise regime already handles a real tiny
#: drive inside :func:`resolve_drive_profile`.
_TRANSLATION_EPS_CM: float = 1e-9

#: Chassis-vs-node distance (cm) below which the trace-end residual is treated as
#: float noise and NO ``drift_warn`` indicator is emitted (issue #130, T11). A
#: clean mission lands the chassis on the node to sub-micron precision, so this
#: only needs to swallow accumulated float error, not any real off-node residual
#: (a raw move that ends the trace leaves the chassis well clear of this floor).
_RESIDUAL_DRIFT_EPS_CM: float = 1e-6

#: Goto-hop primitive tags that render as a LINE-follow advance — cruise
#: defaults to ``ConfigLine::CRUISE_SPEED`` (the follow-phase speed). Covers the
#: forward follows, both pass-through variants, the REAR line/projection
#: follows, and the line-backed return seek. (``ACT_LINE_OVERSHOOT`` /
#: ``ACT_LINE_PASS`` are the roll-onto-node step handled with the distance-gated
#: stop family below.)
_LINE_FOLLOW_TAGS: frozenset[str] = frozenset(
    {
        ACT_FOLLOW_INT,
        ACT_FOLLOW_DIST,
        ACT_FOLLOW_DIST_PASS,
        ACT_LINE_REVERSE,
        ACT_PROJECTION_REVERSE,
        ACT_PROJECTION_REVERSE_PASS,
        ACT_RETURN_STOP_AT_LINE,
    }
)

#: Goto-hop primitive tags whose advance ramps cruise -> ``PWM_MIN_MAGNITUDE``
#: then brakes to rest (the distance-gated stop / LINE_OVERSHOOT roll family).
#: They cruise at the Drive preset (``SPEED_NORMAL``) with ``end == 0`` so the
#: visible stop shape (ramp-down, floor-crawl, brake) renders (AC2).
_STOP_GATED_TAGS: frozenset[str] = frozenset(
    {
        ACT_STOP_AT_LINE,
        ACT_STOP_AT_LINE_REVERSE,
        ACT_LINE_OVERSHOOT,
        ACT_LINE_PASS,
    }
)

#: Goto-hop primitive tags rendered as a blind / dead-reckon advance — cruise
#: defaults to the Drive preset T-shape (``SPEED_NORMAL``).
_BLIND_TAGS: frozenset[str] = frozenset(
    {
        ACT_FORWARD,
        ACT_BLIND_REVERSE,
        ACT_RETURN_FORWARD,
        ACT_WALL_DEPART,
    }
)

#: Goto-hop primitive tags rendered as a pure in-place turn (no advance), timed
#: by the turn profile. ``ACT_HEADING_RESET`` is a heading-only re-bind with no
#: pose change (the sim snapshot heading equals the incoming), so it lands here
#: for completeness; a zero-magnitude turn simply emits no interior sub-frames.
_TURN_TAGS: frozenset[str] = frozenset({ACT_TURN_REL, ACT_HEADING_RESET})

#: Return-to-node prologue DRIVE tags (issue #130, T11). These fire ONLY in the
#: ``step_through`` return prologue (never in the planned walk), so the first one
#: seen inside a goto marks a rendered physical return-to-node maneuver — the cue
#: to emit the ``return_to_node`` localization-lane marker. The prologue's leading
#: ``ACT_TURN_REL`` pre-edge turn renders as a plain turn (``find_pivot_turn`` with
#: ``prev_node=None`` misses, so no spurious pivot arc); the marker on the drive
#: step alone is the "brief return marker" the plan asks for.
_RETURN_DRIVE_TAGS: frozenset[str] = frozenset(
    {ACT_RETURN_FORWARD, ACT_RETURN_STOP_AT_LINE}
)


def _primitive_drive_speed(tag: str) -> Tuple[int, int, int, bool]:
    """Resolve a goto-hop primitive's effective ``(cruise, start, end, profiled)``.

    Goto-expanded hops carry no recorded speed, so each primitive KIND sources
    its cruise from the parsed firmware Config (plan Decisions, the per-kind
    cruise table):

    * LINE follow (:data:`_LINE_FOLLOW_TAGS`) -> ``ConfigLine::CRUISE_SPEED``.
    * distance-gated stop / LINE_OVERSHOOT roll (:data:`_STOP_GATED_TAGS`) ->
      the Drive preset ``SPEED_NORMAL``, ramping cruise -> ``PWM_MIN_MAGNITUDE``
      then braking (``end == 0``) so the stop is visible (AC2).
    * ``wallStop`` (:data:`ACT_WALL_STOP`) -> a CONSTANT ``WALL_STOP_PUSH_PWM``
      crawl (flat, no ramp), matching the firmware's constant-push seek.
    * blind / dead-reckon (:data:`_BLIND_TAGS`) -> the Drive preset
      ``SPEED_NORMAL`` T-shape.

    ``start`` is the sign-by-direction ``TSHAPE_START_SPEED`` seed (the sign of
    ``cruise`` carries the drive direction into the ramp); ``end`` is 0 (brake to
    rest at the hop boundary — every goto hop terminates stationary in the sim's
    step model). ``profiled`` is ``True`` for the ramped families and ``False``
    for the constant ``wallStop`` crawl. A tag that reaches here unclassified
    (never expected — the sim's advance tags are exhaustively covered) defaults
    to the blind preset so timing stays finite rather than raising.

    The reverse direction is NOT applied here: the geometry (and thus the drive
    direction) already lives in the snapshot pose the caller passes as
    ``end_pose``; the profile only supplies the timing curve, whose shape is
    direction-agnostic (it depends on the |PWM| magnitudes). ``cruise`` is kept
    POSITIVE so :func:`resolve_drive_profile`'s sign-by-cruise ``start`` seed is
    forward — the visible ramp shape is identical either way.

    :param tag: the primitive action tag (``primitive[0]``).
    :return: ``(cruise, start, end, profiled)`` — signed PWM for the first
        three (all non-negative here), bool for profiled.
    """
    cfg = default_config()
    start = int(cfg.start_speed)

    if tag == ACT_WALL_STOP:
        # Constant-push wall seek: a flat crawl at WALL_STOP_PUSH_PWM, no ramp.
        return int(cfg.wall_push_pwm), int(cfg.wall_push_pwm), int(cfg.wall_push_pwm), False
    if tag in _LINE_FOLLOW_TAGS:
        return int(cfg.line_cruise), start, 0, True
    # The stop-gated family and the blind/dead-reckon family BOTH cruise at the
    # Drive preset SPEED_NORMAL and brake to rest (``end == 0``); they differ
    # only in intent (a sensor/line gate vs. pure odometry), which the snapshot
    # pose already captures, so the ramped-to-rest T-shape timing is identical.
    # A tag outside every known set (never expected) also lands here — a finite
    # blind-preset T-shape rather than raising.
    if tag in _STOP_GATED_TAGS or tag in _BLIND_TAGS:
        return int(cfg.speed_normal), start, 0, True
    return int(cfg.speed_normal), start, 0, True


def _densify_drive(
    prev_kf: Dict[str, Any],
    end_pose: Tuple[float, float, float],
    cruise: int,
    start: int,
    end: int,
    profiled: bool,
) -> List[Dict[str, Any]]:
    """Densify one drive/line geometry step into profile-timed sub-keyframes.

    Builds a :class:`DriveProfile` for the straight-line advance from
    ``prev_kf`` to ``end_pose`` (distance = the chord between the two poses),
    then samples it into dense keyframes via
    :func:`tools.path_planner.ctps.profile.sample_drive`, stitching the clock so
    the FIRST sub-keyframe sits at ``prev_kf["t"]`` and equals ``prev_kf``'s
    pose byte-for-byte. That duplicate first sub-keyframe is DROPPED (the caller
    already holds it as the previous keyframe), so the returned list is the
    REPLACEMENT for the single flat keyframe the pre-#130 builder appended:
    ``N - 1`` interior samples plus the final sub-keyframe, whose pose is forced
    byte-equal to ``end_pose`` (the sim snapshot) — the boundary-pose invariant
    (TC7).

    Timestamps are non-decreasing and start strictly after ``prev_kf["t"]``
    (``sample_drive`` clocks each interior sample by the closed-form profile
    duration). A zero-length advance (``end_pose`` equals ``prev_kf``'s pose)
    yields an empty list (``sample_drive`` returns only the start keyframe, which
    is the dropped duplicate) — no keyframe is appended and the clock does not
    advance.

    :param prev_kf: the already-appended previous keyframe
        (``{"t", "x", "y", "h"}``); its pose is the profile's start pose and its
        ``t`` is the arc-start clock.
    :param end_pose: ``(x, y, heading_deg)`` at the end of the advance — the sim
        snapshot pose, pinned byte-exact as the last sub-keyframe.
    :param cruise: signed cruise PWM.
    :param start: signed start PWM.
    :param end: signed end PWM.
    :param profiled: ``True`` for a ``.tShape()`` move, ``False`` for a flat one.
    :return: the replacement sub-keyframes (may be empty for a zero-length step).
    """
    x0, y0, h0 = prev_kf["x"], prev_kf["y"], prev_kf["h"]
    x1, y1, h1 = end_pose
    dist = hypot(x1 - x0, y1 - y0)

    profile = resolve_drive_profile(
        dist,
        cruise_pwm=cruise,
        start_pwm=start,
        end_pwm=end,
        profiled=profiled,
    )
    sub = sample_drive(profile, (x0, y0, h0), (x1, y1, h1), prev_kf["t"])
    # sub[0] duplicates prev_kf (byte-equal pose at the same t); drop it.
    return sub[1:]


def _densify_turn(
    prev_kf: Dict[str, Any],
    end_pose: Tuple[float, float, float],
) -> List[Dict[str, Any]]:
    """Densify one in-place turn geometry step into profile-timed sub-keyframes.

    Times the pure rotation from ``prev_kf`` to ``end_pose`` with the turn
    profile (:func:`resolve_turn_profile` + :func:`sample_turn`), which — with
    the current Config (both turn stages snap to ``|PWM| = 50``) — renders a
    CONSTANT-RATE arc (evenly-timed sub-keyframes) at the derived angular rate.
    The swing angle is the shortest-arc heading delta ``prev -> end``.

    As with :func:`_densify_drive`, the FIRST sub-keyframe (which
    :func:`sample_turn` pins to ``prev_kf``'s pose at ``prev_kf["t"]``) is
    DROPPED — the caller already holds it — so the returned list is the
    replacement for the single flat turn keyframe: interior samples plus the
    final sub-keyframe, whose pose is forced byte-equal to ``end_pose``. A
    zero-magnitude turn yields an empty list (no visible rotation to animate).

    :param prev_kf: the already-appended previous keyframe; its ``t`` is the
        turn-start clock and its pose is the turn-start pose.
    :param end_pose: ``(x, y, heading_deg)`` at the end of the turn — pinned
        byte-exact as the last sub-keyframe.
    :return: the replacement sub-keyframes (empty for a zero-magnitude turn).
    """
    x0, y0, h0 = prev_kf["x"], prev_kf["y"], prev_kf["h"]
    x1, y1, h1 = end_pose
    theta_deg = wrap_pm180(h1 - h0)

    profile = resolve_turn_profile(theta_deg, is_pivot=False)
    sub = sample_turn(profile, (x0, y0, h0), (x1, y1, h1), prev_kf["t"])
    return sub[1:]


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

    The arc GEOMETRY (``N`` sub-keyframe centres + headings) is unchanged from
    the pre-#130 builder; only the TIMING now comes from the firmware turn
    profile. The ``N`` samples are re-timed by
    :func:`tools.path_planner.ctps.profile.sample_turn` (the ``arc_keyframes``
    re-time path): the whole arc occupies ``resolve_turn_profile(theta).duration_s()``
    at the snapped constant rate, its sub-keyframes pinned to ``t0 + k/N *
    duration`` (uniform grid, never accumulated) so the running clock stays
    exactly monotonic. With the current Config both turn stages snap to
    ``|PWM| = 50`` -> a constant-rate arc (AC3).

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
        replacement keyframes (profile-timed), the planted-wheel world centre,
        the arc-end chassis centre, and the running clock after the arc.
    """
    wheel_ang_rad = radians(h0_deg + wheel_sign * 90.0)
    wx = x0 + radius_cm * cos(wheel_ang_rad)
    wy = y0 + radius_cm * sin(wheel_ang_rad)

    abs_theta = abs(theta_deg)
    n = max(1, ceil(abs_theta / ARC_STEP_DEG))

    # Offset of the chassis centre from the planted wheel, rotated by tau*theta.
    rx = x0 - wx
    ry = y0 - wy

    # Build the arc GEOMETRY (poses only); timing is placeholder here and set by
    # the turn profile below.
    geom: List[Dict[str, Any]] = []
    cx_end, cy_end = x0, y0
    for k in range(1, n + 1):
        tau = k / n
        ang_rad = radians(tau * theta_deg)
        cos_a = cos(ang_rad)
        sin_a = sin(ang_rad)
        cx = wx + (rx * cos_a - ry * sin_a)
        cy = wy + (rx * sin_a + ry * cos_a)
        heading = (h0_deg + tau * theta_deg) % 360.0
        geom.append({"t": t0, "x": cx, "y": cy, "h": heading})
        cx_end, cy_end = cx, cy

    # Re-time the arc geometry with the firmware turn profile: a pivot rotation.
    # ``end_pose`` is the arc-end pose; the ``arc_keyframes`` path preserves each
    # pose byte-for-byte and only assigns the profile-derived timestamps.
    profile = resolve_turn_profile(theta_deg, is_pivot=True)
    arc = sample_turn(
        profile,
        (x0, y0, h0_deg),
        (cx_end, cy_end, geom[-1]["h"]),
        t0,
        arc_keyframes=geom,
    )
    t_end = arc[-1]["t"]

    return arc, (wx, wy), (cx_end, cy_end), t_end


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
      poses read from each yielded snapshot. After the final hop it re-anchors
      onto its node, so it CLOSES any open ``drift_warn`` span, stamping the
      residual offset off ``NODES[current_node]`` as the span's ``t_end`` +
      label. An :class:`UnreachableGoalError` emits an informational ``goto``
      event and ends the track (an open span then runs to the end).
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
    # Every geometry step is DENSIFIED off the firmware motion profile
    # (``ctps/profile.py``): the single flat keyframe the pre-#130 builder
    # appended is replaced by a run of profile-timed sub-keyframes whose LAST
    # sub-keyframe byte-equals the destination pose (the boundary-pose invariant,
    # TC7). Each closure stitches the clock by pinning the new sub-keyframes'
    # timestamps to the profile duration measured from ``keyframes[-1]["t"]``, so
    # the running clock ``t`` becomes the last appended sub-keyframe's time.
    #
    # The pivot-arc branch below is the one OTHER writer of ``t``: it assigns
    # the ``_build_pivot_arc`` (profile-timed) end time directly (same function
    # scope, same monotonic clock — the arc replaces the in-place turn keyframe
    # these closures would otherwise have timed).

    def _extend(sub_keyframes: List[Dict[str, Any]]) -> None:
        """Append profile-timed sub-keyframes and advance ``t`` to the last one.

        ``sub_keyframes`` is a densification result (the first, duplicate
        keyframe already dropped by :func:`_densify_drive` / :func:`_densify_turn`),
        so it is the REPLACEMENT run for one geometry step. An empty run (a
        zero-length advance / zero-magnitude turn) leaves the clock and keyframes
        untouched.
        """
        nonlocal t
        if not sub_keyframes:
            return
        keyframes.extend(sub_keyframes)
        t = keyframes[-1]["t"]

    def append_drive_keyframes(
        x_new: float,
        y_new: float,
        h_new: float,
        speed: Tuple[int, int, int, bool],
    ) -> None:
        """Densify a drive/line advance to ``(x_new, y_new, h_new)`` and append it.

        ``speed`` is the resolved ``(cruise, start, end, profiled)`` for this
        step; the straight-line advance from the last keyframe is profiled and
        sampled dense-near-ramps. The final sub-keyframe is pinned to the
        destination pose byte-exact.
        """
        cruise, start, end, profiled = speed
        _extend(
            _densify_drive(
                keyframes[-1], (x_new, y_new, h_new), cruise, start, end, profiled
            )
        )

    def append_turn_keyframes(x_new: float, y_new: float, h_new: float) -> None:
        """Densify an in-place turn to ``(x_new, y_new, h_new)`` and append it.

        Times the pure rotation with the turn profile (constant-rate at the
        snapped floor for the default Config). The final sub-keyframe is pinned
        to the destination pose byte-exact. A zero-magnitude turn appends
        nothing.
        """
        _extend(_densify_turn(keyframes[-1], (x_new, y_new, h_new)))

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

    def operator_advance_keyframe(
        distance_cm: float, speed: Tuple[int, int, int, bool]
    ) -> None:
        """Route a raw translation through ``sim.operator_move``, then densify it.

        Issue #130 (T11): a raw off-node drive is a hand-drive off the anchored
        node, so it goes through
        :meth:`~tools.path_planner.sim.runtime.PathTrackerSim.operator_move` —
        which BOTH moves the shared chassis AND integrates the operator offset
        (subject to the sim's nav-active gate + deadbands) — never a bare
        ``chassis.forward``/``chassis.reverse``. Passing a NEGATIVE distance
        drives the chassis backward along its heading (the offset then
        decomposes along that same heading with a negative magnitude), matching
        the firmware integrating a reverse encoder delta.

        The advance DIRECTION is the resolved cruise sign, not the recorded
        ``reverse`` flag: ``.setPower`` takes a signed ``[-100, 100]`` with no
        ``setReverse``, so a negative power on a non-reverse ``driveDist`` is a
        backward move. ``resolve_effective_speed`` folds BOTH the power sign and
        the ``reverse`` flag into a single signed ``cruise`` (exactly one
        direction channel), so a NEGATIVE resolved cruise drives backward and a
        positive one forward. (goto-hop advances never reach here — they use
        per-kind positive cruise + sim-snapshot geometry — so this only re-signs
        the RAW recorded drive.)

        The rendered geometry is read back from the (now-moved) shared chassis
        and densified off ``speed`` (the recorded move's resolved
        ``(cruise, start, end, profiled)``), so the raw excursion still animates;
        the growing offset is what the NEXT goto's return prologue corrects.
        """
        cruise = speed[0]
        signed_cm = distance_cm if cruise >= 0 else -distance_cm
        sim.operator_move(signed_cm, turn_deg=0.0)
        append_drive_keyframes(
            chassis.x_cm, chassis.y_cm, chassis.heading_deg, speed
        )

    def operator_turn_keyframe(turn_deg: float) -> None:
        """Route a raw in-place turn through ``sim.operator_move``, then densify it.

        Issue #130 (T11): a raw ``turnDeg`` / ``turnToYaw`` / the gyro pre-advance
        alignment feeds the offset's rotation channel through
        :meth:`~tools.path_planner.sim.runtime.PathTrackerSim.operator_move`
        (``distance_cm = 0``, ``turn_deg`` the signed in-place rotation), so a
        manual turn accumulates yaw offset exactly as the robot does. The chassis
        heading is updated by ``operator_move``; the pure rotation is then timed by
        the turn profile from the resulting pose. A zero-magnitude turn appends
        nothing (``operator_move`` no-ops the sub-epsilon rotation and the turn
        densifier emits no interior frames).
        """
        sim.operator_move(0.0, turn_deg=turn_deg)
        append_turn_keyframes(chassis.x_cm, chassis.y_cm, chassis.heading_deg)

    # 4. Iterate the commands in order on the single shared sim/chassis.
    #
    # Raw off-node drives (issue #130, T11) route through ``sim.operator_move``,
    # which grows the localization offset; the NEXT goto's ``step_through``
    # return prologue physically drives that offset back onto the node (rendered
    # + marked ``return_to_node``) so no drift indicator survives a raw
    # move-then-goto. The ONLY residual ``drift_warn`` is the trace-end check
    # after the loop (a raw move with nothing to re-anchor it) or the FAULT-gate
    # over-cap fault handled inline below — so there is no longer a per-move
    # open-span variable to carry.
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
            # One ``return_to_node`` marker per goto whose ``step_through``
            # yielded a physical return prologue (issue #130, T11). Latched the
            # first time a return DRIVE tag appears so the pre-edge turn +
            # (seek/dead-reckon) drive share a single brief marker.
            return_marked = False
            try:
                for primitive, snap in sim.step_through(
                    target,
                    route_direction=_direction_from_tag(cmd.route),
                    arrival_direction=_direction_from_tag(cmd.arrival),
                ):
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

                    # Emit the brief return-to-node marker the first time this
                    # goto's return prologue reaches its DRIVE step (issue #130,
                    # T11). The prologue is yielded BEFORE the planned walk, so
                    # ``t_start`` (the clock before any hop) brackets it; the
                    # marker is stamped there so ``events`` stays sorted while the
                    # rendered return maneuver plays out over the sub-keyframes
                    # appended below. Only the DRIVE tags are return-exclusive
                    # (the pre-edge ``ACT_TURN_REL`` also appears in the walk), so
                    # the marker latches on the drive, not the turn.
                    if not return_marked and tag in _RETURN_DRIVE_TAGS:
                        return_marked = True
                        events.append(
                            {
                                "t": t_start,
                                "type": "return_to_node",
                                "label": f"return to {cur_node}",
                            }
                        )

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
                        # Densify this hop's geometry step off the firmware
                        # motion profile. Classify by the ACTUAL translation, not
                        # the tag: a pure in-place turn (``ACT_TURN_REL`` /
                        # heading-only re-anchor squares) leaves x/y unchanged and
                        # is timed by the turn profile; any advance (LINE / BLIND /
                        # STOP_AT_LINE / wall depart, and the rare combined move)
                        # is timed by the drive profile with the per-primitive-kind
                        # cruise. The LAST sub-keyframe is pinned to the snapshot
                        # pose byte-exact (boundary-pose invariant).
                        prev_kf = keyframes[-1]
                        step_dist = hypot(
                            snap.x_cm - prev_kf["x"], snap.y_cm - prev_kf["y"]
                        )
                        if step_dist <= _TRANSLATION_EPS_CM:
                            append_turn_keyframes(
                                snap.x_cm, snap.y_cm, snap.heading_deg
                            )
                        else:
                            append_drive_keyframes(
                                snap.x_cm,
                                snap.y_cm,
                                snap.heading_deg,
                                _primitive_drive_speed(tag),
                            )
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
            except OffMapFaultError as fault:
                # Over-cap off-map excursion under the FAULT gate (issue #130,
                # T11): the return gate refused to plan (mirror of
                # ``Fault::OFF_NODE``) BEFORE any return primitive was yielded, so
                # nothing rendered this goto. Surface it as a fault event that
                # ends the track, exactly like the UNREACHABLE path — the sim
                # only raises this under BEST_EFFORT-off (FAULT gate); a
                # BEST_EFFORT gate would instead have yielded a capped return
                # that renders + marks ``return_to_node`` above. The chassis was
                # not moved, so its residual off the node is reported for context.
                fault_from = sim.localization.current_node()
                events.append(
                    {
                        "t": t,
                        "type": "goto",
                        "label": (
                            f"OFF-MAP {fault_from}->{target} "
                            f"({fault.offset_mag_cm:.1f} cm off node)"
                        ),
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
            # No drift-span bookkeeping here (issue #130, T11): a preceding raw
            # move already integrated its offset through ``sim.operator_move``,
            # and this goto's ``step_through`` return prologue physically drove
            # that offset back onto the node (rendered + marked
            # ``return_to_node`` above), so nothing is left to close. A raw move
            # with NO following goto is the only residual, handled once after the
            # command loop.

        elif cmd.op == "drive":
            # Raw off-node move (issue #130, T11): route through
            # ``sim.operator_move`` so the operator offset integrates on the
            # SHARED sim exactly as the robot integrates raw encoder / IMU deltas
            # while ``!navActive``. The logical node is never advanced; the next
            # goto's return prologue drives the accumulated offset back. Narration
            # is timestamped at the command's start (before any geometry advance).
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
                    # never ``float(None)``. No chassis motion, so nothing
                    # integrates into the offset. Kept even though the recorder
                    # now emits ``cm`` for every forward-geometry method, to
                    # defend partial/legacy traces.
                    pass
                else:
                    # Raw (recorded) move: resolve the effective (cruise, start,
                    # end, profiled) from the recorded builder knobs + Config
                    # defaults (sign-by-cruise start seed, no-brake -> endSpeed =
                    # cruise), then densify the advance off that profile.
                    speed = resolve_effective_speed(cmd, method)
                    if _honors_gyro_heading(cmd):
                        # GYRO + explicit heading → two-phase render: first
                        # rotate in place to the held heading (a raw turn through
                        # ``operator_move``'s rotation channel), THEN advance /
                        # back up along it (its translation channel). Heading is
                        # honoured ONLY under ``mode == "gyro"`` — firmware
                        # consults setHeading only in GYRO, so a heading recorded
                        # without GYRO renders along the current facing (the plain
                        # branch below). ``operator_turn_keyframe`` takes the
                        # signed in-place delta so both the render and the offset
                        # rotation match the chassis.
                        operator_turn_keyframe(
                            wrap_pm180(float(cmd.heading) - chassis.heading_deg)
                        )
                        operator_advance_keyframe(float(cmd.cm), speed)
                    else:
                        # Plain advance along the CURRENT facing (non-gyro, or no
                        # heading, or lineFollowDist).
                        operator_advance_keyframe(float(cmd.cm), speed)
            elif method == "turnDeg":
                # Raw relative turn: the signed delta IS the ``turnDeg`` argument.
                operator_turn_keyframe(float(cmd.deg))
            elif method == "turnToYaw":
                # Raw absolute turn: route the shortest-arc delta to the target
                # yaw through ``operator_move`` (``operator_move`` turns relative),
                # so the render and the integrated yaw offset both land on the
                # requested heading.
                operator_turn_keyframe(
                    wrap_pm180(float(cmd.deg) - chassis.heading_deg)
                )
            # else: stop / unknown -> no geometry change, nothing to integrate.
            events.append(
                {"t": event_t, "type": "drive", "label": _drive_label(cmd)}
            )
            # No drift span is opened here (issue #130, T11): the offset now lives
            # in the shared sim, and the NEXT goto's return prologue corrects it
            # (rendered + marked ``return_to_node``). A raw move with NO following
            # goto surfaces once as the trace-end residual after the loop.

        elif cmd.op == "declare_position":
            # Operator-declared belief reset (issue #130, T11): set the current
            # node + clear the offset via the sim, and emit a ``declare_position``
            # localization-lane marker. This is DISTINCT from return-to-node — no
            # physical motion, no chassis mutation, no keyframe: only the belief
            # changes (the clock does not advance). ``declare_position`` warns +
            # no-ops on an unknown node (mirroring the loader's strict-field
            # pattern), so pre-validate against ``NODES`` the same way and emit NO
            # marker for an unknown node (the belief did not change). The loader
            # already dropped a structurally-malformed declare, so ``cmd.node``
            # is a well-formed name here; the NODES guard defends the unknown-name
            # case the loader still forwards.
            node = cmd.node
            if node is not None and node in field_data.NODES:
                sim.localization.declare_position(node)
                events.append(
                    {
                        "t": t,
                        "type": "declare_position",
                        "label": f"declare {node}",
                    }
                )
            # Unknown / absent node → no belief change, no marker (the sim would
            # only warn + no-op; skipping the call avoids a spurious UserWarning
            # in the render path).

        elif cmd.op == "manip":
            # `method`/`arg` are structured (additive) alongside the existing
            # `label`, so the player can dispatch on them directly instead of
            # parsing the label string; `label` is kept for display/back-compat
            # (T9, CTPS object-cargo plan). Purely additive: a manip-free trace
            # never touches this branch, so its Track stays byte-identical.
            events.append(
                {
                    "t": t,
                    "type": "manip",
                    "method": cmd.method,
                    "arg": cmd.arg,
                    "label": f"{cmd.method}({cmd.arg})",
                }
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

    # Trace-end residual drift indicator (issue #130, T11). A raw off-node move
    # with NO following goto never re-anchors — nothing drives its offset back —
    # so the sim ends the track believing the chassis sits off its logical node.
    # The residual is the sim's accumulated operator offset (the SAME quantity
    # the return gate reads, ``getOffsetFromNode().magnitude()``), so a raw move
    # ≥ the deadband surfaces here while:
    #
    #   * a clean mission (all gotos) leaves offset 0 — each goto re-anchors, and
    #     a preceding raw move's offset was already corrected by the return
    #     prologue — so NO indicator is emitted (a ``drift_warn``-free /
    #     raw-move-free trace renders byte-identically to T10);
    #   * a ``declare_position`` belief reset also leaves offset 0 (it cleared the
    #     offset without moving the chassis), so declaring position never fabricates
    #     a residual off the physically-unchanged chassis pose;
    #   * a sub-deadband raw move leaves offset 0 (firmware-faithful — the encoder
    #     baseline stays unadvanced below the deadband), so it, too, warns nothing.
    #
    # The indicator is a POINT event at the track end (there is no span to open
    # anymore); its label reports the residual magnitude off the parked node.
    node = sim.localization.current_node()
    residual_cm = sim.localization.get_offset_from_node().magnitude_cm()
    if residual_cm > _RESIDUAL_DRIFT_EPS_CM:
        events.append(
            {
                "t": t,
                "type": "drift_warn",
                "label": f"chassis {residual_cm:.1f} cm off node {node}",
            }
        )

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
        field=build_field_block(field_data),
        duration_s=t,
        keyframes=keyframes,
        events=events,
        pivots=pivots,
    )
