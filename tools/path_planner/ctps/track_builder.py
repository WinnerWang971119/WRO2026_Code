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

import warnings
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
from tools.path_planner.pipeline_config import PipelineConfig, SpeedTier, TurnMode
from tools.path_planner.planner import (
    BlindDecayParams,
    compute_all_pairs_next_hop,
    parse_track_cm_from_config,
)
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import (
    ACT_BLIND_PASS,
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
    NAV_TURN_TOLERANCE_DEG,
    Direction,
    OffMapFaultError,
    PathTrackerSim,
    UnreachableGoalError,
    resolve_speed,
    wrap_pm180,
)

from tools.path_planner.ctps.config import (
    ARC_STEP_DEG,
    KEYFRAME_COUNT_CEILING,
    LINE_FOLLOW_OFFSET_DISPLAY_CM,
    MANIP_NOMINAL_S,
    RAMP_SAMPLE_STEP_CM,
    ROBOT_FRAME_CM,
    SCALE_CM_S_PER_PWM,
)
from tools.path_planner.ctps.profile import (
    ApproachSchedule,
    approach_speed_at,
    default_config,
    resolve_approach_schedule,
    resolve_drive_profile,
    resolve_effective_speed,
    resolve_turn_profile,
    sample_drive,
    sample_drive_run,
    sample_turn,
)
from tools.path_planner.ctps.trace_loader import Command, MosaicTrace, Trace

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
#: correction. Since #54 / ADR-073 a REAR pivot can depart over a sensor-
#: terminated leg, so the REAR reverse tags (``ACT_STOP_AT_LINE_REVERSE`` and
#: the following ``ACT_LINE_OVERSHOOT``) now genuinely fire on a pivot
#: departure; the arming site flips the residual ray into the travel frame for
#: REAR (see ``pending_pivot_land``).
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
        T9). A ``read_mosaic`` event additionally carries an ``"success":
        Optional[bool]`` key (the read outcome echoed from the trace's
        ``read_mosaic`` op) — a non-geometry event, like ``drive``'s ``stop``
        method, that dwells nothing and registers no new timeline lane (CTPS
        randomized-vision plan, T9). Both are additive to the unchanged
        ``"label"`` string, so the player can dispatch on the structured keys
        directly instead of parsing the label. No other event type gains new
        keys, so a manip-free / read_mosaic-free trace's events stay
        byte-identical.
    :param pivots: Optional single-wheel pivot-arc overlay segments (issue #55),
        one per pivot the trace's routes actually take, sorted by ``t0``. Each
        entry is ``{"t0": float, "t1": float, "cx": float, "cy": float,
        "theta_deg": float, "wheel": "left"|"right", "direction":
        "front"|"rear"}`` — the arc-span timestamps, the planted-wheel world
        centre (rotation centre), the signed swing angle (CCW positive, the
        ``ACT_TURN_REL`` delta), the PLANTED (post-flip) pivot wheel side, and
        the sim-derived pivot direction. ``wheel`` and ``direction`` both come
        from the sim's taken-pivot surface (#54 / ADR-073): on a REAR pivot the
        wheel is the L<->R flip of the stored forward wheel, and ``cx``/``cy``
        already sit on the flipped (correct world / travel) side, so the player
        needs no direction logic to pin the wheel. Empty (and omitted from
        :meth:`to_dict`) for every pivot-free trace, so ``schema_version`` stays
        ``1`` (additive optional fields, all INSIDE the non-empty-gated list).
        The dense arc keyframes between ``t0`` and ``t1`` already trace the
        swing through the player's lerp; this segment just tells the
        ``wheel-arc`` render mode WHEN a pivot is active and WHERE its wheel is.
    :param lateral_offsets: Optional visualization-only lateral hug-side overlay
        segments (issue #156), one per biased ``lineFollowDist`` leg the trace
        renders, in chronological (``t0``-increasing) order. Each entry is
        ``{"t0": float, "t1": float, "offset_cm": float}`` — the span the follow
        is biased and the signed on-screen lateral nudge (cm; ``+`` = robot-left,
        the LEFT bias), mapped from the recorded ``follow_offset`` via
        :data:`~tools.path_planner.ctps.config.LINE_FOLLOW_OFFSET_DISPLAY_CM`. The
        player nudges the chassis marker perpendicular to its heading by
        ``offset_cm`` while the playhead is inside a span — a pure DRAW-time
        overlay (ADR-054 pivot-arc precedent): the keyframes stay on the
        centerline and the parity-locked :class:`LocalizationSim` pose is never
        touched. Empty (and omitted from :meth:`to_dict`) for every centered /
        MIDDLE trace, so ``schema_version`` stays ``1`` (additive optional field).
    :param mosaic: The trace's top-level ``mosaic`` echo
        (:class:`~tools.path_planner.ctps.trace_loader.MosaicTrace`), carried
        through UNCHANGED from :attr:`Trace.mosaic` — a non-geometry,
        document-level attribute (it does not move the chassis). ``None`` for
        a trace with no ``mosaic`` block (no ``--mosaic`` on the dump, or an
        old trace), which is the normal/unwarned case (CTPS randomized-vision
        plan, T9). The player embed (T10) and ``vision.js`` (T11) read this to
        resolve + draw the rolled cells; :meth:`to_dict` omits the key
        entirely when ``None``, so a mosaic-free trace's Track JSON stays
        byte-identical to before this change (AC12).
    """

    schema_version: int
    field: Dict[str, Any]
    duration_s: float
    keyframes: List[Dict[str, Any]]
    events: List[Dict[str, Any]]
    pivots: List[Dict[str, Any]] = dc_field(default_factory=list)
    lateral_offsets: List[Dict[str, Any]] = dc_field(default_factory=list)
    mosaic: Optional[MosaicTrace] = None

    def to_dict(self) -> Dict[str, Any]:
        """Return the JSON-serialisable Data Model B dict for this track.

        Nested keyframe/event dicts are shared references (they are never
        mutated after construction), so the player can embed the result
        directly.

        The ``"pivots"`` key is included **only when non-empty**: a pivot-free
        trace emits exactly the pre-#55 dict (same keys, same order), so its
        Track JSON is byte-identical to before this change (AC6). The
        ``"lateral_offsets"`` key (issue #156) is likewise included **only when
        non-empty**: a centered / MIDDLE trace emits no such key and stays
        byte-identical. Likewise ``"mosaic"`` is included **only when set**: a
        mosaic-free trace emits exactly the pre-mosaic dict, so its Track JSON
        stays byte-identical (AC12). When present, ``"mosaic"`` is the plain
        field-value dict
        (``{"cells", "seed", "geom_hash", "injected"}``, ``seed`` omitted when
        unset) — the same shape :class:`~tools.path_planner.ctps.trace_loader.MosaicTrace`
        carries, JSON-serialisable via ``dataclasses.asdict``-equivalent
        field access.

        :return: ``{"schema_version", "field", "duration_s", "keyframes",
            "events"}`` — plus ``"pivots"`` when the trace took at least one
            pivot, plus ``"mosaic"`` when the trace carried a mosaic roll —
            exactly the shape the HTML player consumes.
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
        if self.lateral_offsets:
            result["lateral_offsets"] = self.lateral_offsets
        if self.mosaic is not None:
            mosaic_dict: Dict[str, Any] = {
                "cells": list(self.mosaic.cells),
                "geom_hash": self.mosaic.geom_hash,
                "injected": self.mosaic.injected,
            }
            if self.mosaic.seed is not None:
                mosaic_dict["seed"] = self.mosaic.seed
            result["mosaic"] = mosaic_dict
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
#: forward follows, both pass-through variants, and the REAR line/projection
#: follows. (``ACT_LINE_OVERSHOOT`` / ``ACT_LINE_PASS`` are the roll-onto-node
#: step handled with the distance-gated stop family below. ``ACT_RETURN_STOP_AT_LINE``
#: — the line-backed return seek — is deliberately EXCLUDED: issue #157 renders it
#: at ``PIPELINE_CONFIG.speed`` via :data:`_RETURN_DRIVE_TAGS` instead, matching
#: firmware's ``_dispatchReturnMoveAfterTurn`` rather than the un-tiered line
#: cruise.)
_LINE_FOLLOW_TAGS: frozenset[str] = frozenset(
    {
        ACT_FOLLOW_INT,
        ACT_FOLLOW_DIST,
        ACT_FOLLOW_DIST_PASS,
        ACT_LINE_REVERSE,
        ACT_PROJECTION_REVERSE,
        ACT_PROJECTION_REVERSE_PASS,
    }
)

#: Goto-hop primitive tags whose advance ramps cruise -> ``PWM_MIN_MAGNITUDE``
#: then brakes to rest (the distance-gated stop / LINE_OVERSHOOT roll family).
#: They cruise at the Drive preset (``SPEED_NORMAL``) with ``end == 0`` so the
#: visible stop shape (ramp-down, floor-crawl, brake) renders (AC2).
#:
#: NOTE: the collinear PASS tags (:data:`_PASS_TAGS`) are deliberately NOT in
#: this set. A pass-through node is a NON-braking boundary — the follower holds
#: cruise across it — so a pass tag must NOT force ``end == 0`` (that was the
#: shipped stop-at-every-pass-node lie the run-grouper repairs). Only a genuine
#: distance-gated stop that lands the chassis at rest belongs here.
_STOP_GATED_TAGS: frozenset[str] = frozenset(
    {
        ACT_STOP_AT_LINE,
        ACT_STOP_AT_LINE_REVERSE,
        ACT_LINE_OVERSHOOT,
    }
)

#: Mid-run geometry contributors — a follow leg that lands the sensor short of
#: its node and is ALWAYS immediately followed by a pass or an overshoot (never
#: braking on its own). It contributes distance to the open run but never ENDS
#: it; the pass / overshoot that follows decides the boundary.
#:
#: The FORWARD (``ACT_FOLLOW_INT``) and REAR (``ACT_LINE_REVERSE``) line-follow
#: legs are structurally identical — the sim emits each as a follow leg that
#: lands one axle-offset short of the node, then a SEPARATE ``ACT_LINE_OVERSHOOT``
#: (or ``ACT_LINE_PASS``) that rolls onto / passes the node (``runtime.py`` LINE
#: dispatch). Both must therefore be mid-run contributors, or a REAR line hop's
#: collinear run splits at the reverse leg's short-landing point into two ramps
#: (the stop-at-every-node lie the grouper repairs for FORWARD hops) — and, once
#: ADR-061's hop-settle lands, fires a spurious mid-edge settle dwell there.
_MID_RUN_CONTRIB_TAGS: frozenset[str] = frozenset({ACT_FOLLOW_INT, ACT_LINE_REVERSE})

#: Collinear pass-through primitive tags — a NON-braking node boundary. The
#: firmware holds cruise across these (ADR-023 line pass-through; the T2
#: ``setTShapeRampDown(!projPassThrough)`` fix for projection pass-through; the
#: ADR-059 blind-chain ``blind_pass`` mid-hop), so CTPS must render them WITHOUT a
#: decel-to-rest: a pass tag's per-primitive cruise carries ``end == cruise`` (no
#: ramp-down), and — when it sits inside a multi-node run — the run-grouper merges
#: the whole run into ONE trapezoid so there is no intra-run dip at all.
#:
#: ``ACT_BLIND_PASS`` (T9) is a 2-tuple ZERO-MOTION marker
#: (``(ACT_BLIND_PASS, node)``): the chained ``ACT_FORWARD`` / ``ACT_BLIND_REVERSE``
#: hop already advanced onto the node and re-anchored, so the pass tag itself
#: carries no translation. It is listed here so the run-grouper's ``braked``
#: classification treats it as a non-braking boundary that CONTINUES the run,
#: exactly as the line / projection pass tags do; the actual transparency comes
#: from its zero-translation snapshot hitting the zero-motion branch in ``build``.
_PASS_TAGS: frozenset[str] = frozenset(
    {
        ACT_LINE_PASS,
        ACT_FOLLOW_DIST_PASS,
        ACT_PROJECTION_REVERSE_PASS,
        ACT_BLIND_PASS,
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

#: Goto-hop primitive tags whose firmware dispatch resolves a PER-EDGE
#: :class:`SpeedTier` (ADR-064, issue #97) — the forward / reverse blind
#: dead-reckon nav hops and the forward / rear line seeks. These are the ONLY
#: tags for which a LONE (non-chain) hop's cruise is tier-resolved from the edge
#: (:func:`_primitive_drive_speed`'s ``speed_tier``). Deliberately EXCLUDES three
#: families, each rendered a different way (issue #157):
#:   * ``ACT_LINE_OVERSHOOT`` — a LINE-family roll-past; the CTPS models the LINE
#:     follow itself at ``ConfigLine::CRUISE_SPEED`` (un-tiered, a pre-existing sim
#:     limitation), so tier-adjusting only the overshoot would desync it from its
#:     own follow leg. (Both stay at the ``SPEED_NORMAL``-valued line cruise.)
#:   * ``ACT_RETURN_FORWARD`` / ``ACT_RETURN_STOP_AT_LINE`` — the return-to-node
#:     prologue's dead-reckon / line-seek moves (:data:`_RETURN_DRIVE_TAGS`). The
#:     firmware's ``_dispatchReturnMoveAfterTurn`` drives BOTH at
#:     ``PIPELINE_CONFIG.speed`` — the MISSION-WIDE tier, since a synthetic return
#:     hop has no ``EdgeInfo`` to per-edge-resolve. :func:`_primitive_drive_speed`
#:     intercepts these directly via ``pipeline_cfg.speed``, bypassing this
#:     per-edge set entirely.
#:   * ``ACT_WALL_DEPART`` — the wall-departure recovery move. Unlike the return
#:     moves above, the firmware drives this at the Drive ``NORMAL`` default with
#:     NO ``.setPower`` call at all — neither per-edge NOR
#:     ``PIPELINE_CONFIG.speed``. CTPS mirrors that by leaving it out of every
#:     speed set: it falls through :func:`_primitive_drive_speed` to the
#:     ``SPEED_NORMAL`` fallback, unaffected by ``pipeline_cfg``.
#: A chained hop never uses this path (it consumes the recorded per-hop suffix via
#: ``_resolve_blind_run_profile``), so only cold-start lone hops / seeks resolve here.
_PER_EDGE_SPEED_TAGS: frozenset[str] = frozenset(
    {
        ACT_FORWARD,
        ACT_BLIND_REVERSE,
        ACT_STOP_AT_LINE,
        ACT_STOP_AT_LINE_REVERSE,
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

#: Goto-hop stop tags whose braked arrival is REAR-led: the rear sensor board
#: leads the travel direction, so the firmware sheds cruise -> floor inside the
#: REAR overshoot window (``LINE_SENSOR_AXLE_OFFSET_REAR_CM`` — the
#: ``LineFollower.cpp:287-301`` ramp-down window-fit; ``PathTracker`` picks the
#: window with ``reverse ? REAR : FRONT``). A run ENDING on one of these renders
#: the window-fitted shed (AC8): ``flush_run`` passes the rear window into the
#: run profile so a shed that would overrun the window is compressed to it.
#:
#: The sole member is the REAR ``stopAtLine`` seek — the one rear-led arrival
#: whose braked render segment spans the WHOLE hop (seek + overshoot roll in one
#: primitive), so its natural shed can overrun the rear window. The other
#: rear/reverse tags are excluded on purpose:
#:
#: * ``ACT_LINE_REVERSE`` — the reverse follow LEG only; it lands one rear
#:   axle-offset SHORT of the node (the junction detect), and the roll the
#:   window-fit models renders as the SEPARATE overshoot segment that follows.
#: * that following overshoot segment (``ACT_LINE_OVERSHOOT`` after a reverse
#:   follow) — its render distance IS the rear window (the sim's
#:   ``overshoot_advance_cm``), so the T-shape kernel already fits the ramps
#:   inside it; a window spanning the whole segment can never overrun.
#: * ``ACT_PROJECTION_REVERSE`` — a distance-gated follow whose gate is the
#:   COMMANDED edge distance (``stopAfterCm``), not the axle-offset roll; the
#:   kernel's half-of-total ramp cap models that clamp.
#: * every blind dead-reckon stop (``ACT_BLIND_REVERSE`` + the blind-chain
#:   braked final hop) — a Drive ``PROFILE_DIST`` move fits its ramps to the
#:   known distance upfront; no LineFollower gate exists.
#:
#: FRONT-led stops (``ACT_STOP_AT_LINE`` / ``ACT_LINE_OVERSHOOT`` / the return
#: seek) keep ``rear_window_cm=None``: the fit is defined ONLY over the rear
#: window, and the larger FRONT window (12.02 cm) does not bind the natural
#: FAST shed (8 cm) anyway.
_REAR_LED_STOP_TAGS: frozenset[str] = frozenset({ACT_STOP_AT_LINE_REVERSE})


def _blind_cruise_pwm(cfg, speed_tier: Optional[SpeedTier]) -> int:
    """Map a resolved per-edge :class:`SpeedTier` to its blind cruise PWM magnitude.

    Mirrors the firmware ``presetToPwm(speedTierToPreset(_resolveSpeed(edge)))``
    chain (ADR-064) over the parsed :class:`ProfileConfig` presets so a goto-hop
    BLIND / seek advance renders at its OWN resolved tier instead of the hardcoded
    ``SPEED_NORMAL`` default: ``SLOW`` -> ``SPEED_SLOW`` (60), ``FAST`` ->
    ``SPEED_FAST`` (90). ``NORMAL`` / ``DEFAULT`` / ``None`` (a tier that was not
    resolved — an edge outside the graph, or a non-blind primitive) map to
    ``SPEED_NORMAL`` (70), so an all-``NORMAL`` field is byte-identical to the
    pre-ADR-064 render.

    :param cfg: the parsed :class:`ProfileConfig` carrying the preset PWMs.
    :param speed_tier: the hop edge's resolved :class:`SpeedTier`, or ``None``
        when no per-edge tier applies.
    :return: the unsigned cruise PWM magnitude (the caller keeps it positive; the
        travel direction lives in the snapshot pose — see the note below).
    """
    if speed_tier == SpeedTier.SLOW:
        return int(cfg.speed_slow)
    if speed_tier == SpeedTier.FAST:
        return int(cfg.speed_fast)
    return int(cfg.speed_normal)


def _primitive_drive_speed(
    tag: str, pipeline_cfg: PipelineConfig, speed_tier: Optional[SpeedTier] = None
) -> Tuple[int, int, int, bool]:
    """Resolve a goto-hop primitive's effective ``(cruise, start, end, profiled)``.

    Goto-expanded hops carry no recorded speed, so each primitive KIND sources
    its cruise from the parsed firmware Config (plan Decisions, the per-kind
    cruise table):

    * the return-to-node prologue (:data:`_RETURN_DRIVE_TAGS` —
      ``ACT_RETURN_FORWARD`` / ``ACT_RETURN_STOP_AT_LINE``) -> ``pipeline_cfg``'s
      MISSION-WIDE :class:`SpeedTier` PWM (issue #157 — matches firmware
      ``_dispatchReturnMoveAfterTurn``'s ``PIPELINE_CONFIG.speed`` dispatch; a
      synthetic return hop has no ``EdgeInfo`` to per-edge-resolve), ramping
      cruise -> ``PWM_MIN_MAGNITUDE`` then braking (``end == 0``).
      ``ACT_WALL_DEPART`` is deliberately NOT in this family — the firmware
      drives it at the Drive ``NORMAL`` default with no ``.setPower`` call at
      all, so it falls through to the generic ``SPEED_NORMAL`` branch below,
      unaffected by ``pipeline_cfg``.
    * collinear PASS (:data:`_PASS_TAGS`) -> the follow cruise
      (``ConfigLine::CRUISE_SPEED``) held THROUGH the node: ``end == cruise``,
      NO brake. A pass-through is a non-braking boundary (the firmware holds
      cruise across it), so it must not decel to rest — the shipped ``end == 0``
      here was the stop-at-every-pass-node lie. Inside a multi-node run the
      grouper further merges the whole run into ONE trapezoid; a lone pass tag
      (a degenerate single-hop run) still renders no dip via this ``end ==
      cruise``.
    * LINE follow (:data:`_LINE_FOLLOW_TAGS`) -> ``ConfigLine::CRUISE_SPEED``.
    * distance-gated stop / LINE_OVERSHOOT roll (:data:`_STOP_GATED_TAGS`) ->
      this hop's resolved per-edge :class:`SpeedTier` PWM (``speed_tier``;
      ADR-064 — the seek's ``.setPower(cruiseMag)`` on the robot), ramping
      cruise -> ``PWM_MIN_MAGNITUDE`` then braking (``end == 0``) so the stop is
      visible (AC2). ``NORMAL`` / absent stays the ``SPEED_NORMAL`` default (70).
    * ``wallStop`` (:data:`ACT_WALL_STOP`) -> a CONSTANT ``WALL_STOP_PUSH_PWM``
      crawl (flat, no ramp), matching the firmware's constant-push seek.
    * blind / dead-reckon (:data:`_BLIND_TAGS`) -> this hop's resolved per-edge
      :class:`SpeedTier` PWM (``speed_tier``; ADR-064 — the firmware blind cruise
      ``presetToPwm(speedTierToPreset(_resolveSpeed(edge)))``) T-shape.
      ``NORMAL`` / absent stays the ``SPEED_NORMAL`` default (70), byte-identical
      to the pre-ADR-064 render. (``ACT_RETURN_FORWARD`` is nominally a member of
      :data:`_BLIND_TAGS` too, but the return-tag branch above intercepts it
      first, so it never reaches this generic per-edge-tier path.)

    ``start`` is the sign-by-direction ``TSHAPE_START_SPEED`` seed (the sign of
    ``cruise`` carries the drive direction into the ramp); ``end`` is 0 (brake to
    rest at the hop boundary) for every BRAKED hop, and ``cruise`` for a PASS tag
    (held through the node). ``profiled`` is ``True`` for the ramped families and
    ``False`` for the constant ``wallStop`` crawl. A tag that reaches here
    unclassified (never expected — the sim's advance tags are exhaustively
    covered) defaults to the blind preset so timing stays finite rather than
    raising.

    The reverse direction is NOT applied here: the geometry (and thus the drive
    direction) already lives in the snapshot pose the caller passes as
    ``end_pose``; the profile only supplies the timing curve, whose shape is
    direction-agnostic (it depends on the |PWM| magnitudes). ``cruise`` is kept
    POSITIVE so :func:`resolve_drive_profile`'s sign-by-cruise ``start`` seed is
    forward — the visible ramp shape is identical either way.

    :param tag: the primitive action tag (``primitive[0]``).
    :param pipeline_cfg: the field-sourced :class:`PipelineConfig` resolved ONCE
        in :func:`build` (``PipelineConfig.from_field_data``). Consumed ONLY by
        the return-to-node branch (:data:`_RETURN_DRIVE_TAGS`); every other
        branch — including ``ACT_WALL_DEPART`` — is unaffected by it.
    :param speed_tier: this hop edge's resolved :class:`SpeedTier` (ADR-064), or
        ``None`` when no per-edge tier applies. Consumed ONLY by the BLIND /
        stop-gated seek branches (the two families that carried the hardcoded
        ``SPEED_NORMAL`` blind cruise); the line-follow / pass / wallStop cruises
        are unaffected. ``None`` / ``NORMAL`` / ``DEFAULT`` keeps the 70 default.
    :return: ``(cruise, start, end, profiled)`` — signed PWM for the first
        three (all non-negative here), bool for profiled.
    """
    cfg = default_config()
    start = int(cfg.start_speed)

    if tag in _RETURN_DRIVE_TAGS:
        # Return-to-node prologue (issue #157): the firmware's
        # _dispatchReturnMoveAfterTurn drives BOTH the dead-reckon
        # (ACT_RETURN_FORWARD) and line-seek (ACT_RETURN_STOP_AT_LINE) return
        # moves at PIPELINE_CONFIG.speed — the mission-wide tier, not a
        # per-edge resolution (a synthetic return hop has no EdgeInfo).
        # ACT_WALL_DEPART is intentionally excluded from _RETURN_DRIVE_TAGS:
        # the firmware drives it at the Drive NORMAL default with no
        # .setPower call, so it falls through to the generic SPEED_NORMAL
        # path below instead.
        cruise = _blind_cruise_pwm(cfg, pipeline_cfg.speed)
        return cruise, start, 0, True
    if tag == ACT_WALL_STOP:
        # Constant-push wall seek: a flat crawl at WALL_STOP_PUSH_PWM, no ramp.
        return int(cfg.wall_push_pwm), int(cfg.wall_push_pwm), int(cfg.wall_push_pwm), False
    if tag in _PASS_TAGS:
        # Non-braking collinear pass-through: hold the follow cruise THROUGH the
        # node (end == cruise), never decel to rest. All existing pass tags are
        # line-family follows, so the cruise is ConfigLine::CRUISE_SPEED.
        cruise = int(cfg.line_cruise)
        return cruise, start, cruise, True
    if tag in _LINE_FOLLOW_TAGS:
        return int(cfg.line_cruise), start, 0, True
    # The stop-gated seek family, the blind/dead-reckon family, and any tag
    # outside every known set (never expected) all share ONE ramped-to-rest
    # T-shape: cruise at this hop's resolved per-edge SpeedTier (ADR-064;
    # NORMAL/absent/None == SPEED_NORMAL, byte-identical to the old hardcode),
    # brake to rest (``end == 0``). Intent (a sensor/line gate vs. pure odometry)
    # lives in the snapshot pose, not the timing curve, so one arm covers all.
    return _blind_cruise_pwm(cfg, speed_tier), start, 0, True


def _blind_chain_suffix(
    primitive: Tuple[Any, ...],
) -> Optional[Tuple[int, int, bool]]:
    """Extract a chained blind hop's recorded ``(start_pwm, end_pwm, chain)`` suffix.

    A collinear BLIND chain that the sim (or firmware recorder) drove as ONE
    continuous run rides the resolved per-hop velocity endpoints on the blind
    advance primitive's TRAILING elements (ADR-059) — the recorded speeds CTPS
    CONSUMES to render the whole-run trapezoid (AC11). CTPS never re-derives a
    chain decision; it reads exactly what was recorded.

    The suffix layout is tag-specific (mirroring ``sim/runtime.py``'s emit):

    * ``ACT_FORWARD`` — ``(ACT_FORWARD, dist, start_pwm, end_pwm, chain)`` (len 5);
      the plain cold-start form ``(ACT_FORWARD, dist)`` (len 2) carries NO suffix.
    * ``ACT_BLIND_REVERSE`` —
      ``(ACT_BLIND_REVERSE, target, dist, blind_mode, start_pwm, end_pwm, chain)``
      (len 7); the plain form ``(ACT_BLIND_REVERSE, target, dist, blind_mode)``
      (len 4) carries NO suffix.

    A NON-chain hop (a lone pass-through-seeded blind hop the sim marks
    ``chain == False``, or a plain cold-start hop with no suffix) returns
    ``None`` — it takes the ordinary per-primitive drive path, NOT the blind
    whole-run consume path (AC12: a non-chain single hop is unchanged).

    :param primitive: the yielded sim primitive tuple.
    :return: ``(start_pwm, end_pwm, chain)`` when the primitive is a CHAINED
        blind hop (``chain is True``); ``None`` otherwise.
    """
    tag = primitive[0]
    if tag == ACT_FORWARD and len(primitive) >= 5:
        start_pwm, end_pwm, chain = primitive[2], primitive[3], primitive[4]
    elif tag == ACT_BLIND_REVERSE and len(primitive) >= 7:
        start_pwm, end_pwm, chain = primitive[4], primitive[5], primitive[6]
    else:
        return None
    # Only a genuine chained hop (chain True) takes the whole-run consume path. A
    # pass-through-seeded LONE blind hop rides the same suffix but marks
    # chain == False (it is a single hop, not a run); it is rendered by the
    # ordinary per-primitive path, so it is not a blind-run segment.
    if not bool(chain):
        return None
    return int(start_pwm), int(end_pwm), bool(chain)


def _densify_drive(
    prev_kf: Dict[str, Any],
    end_pose: Tuple[float, float, float],
    cruise: int,
    start: int,
    end: int,
    profiled: bool,
    rear_window_cm: Optional[float] = None,
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
    :param rear_window_cm: the REAR overshoot window for a rear-led braked
        distance-gated stop (the AC8 LineFollower window-fit); ``None`` (the
        default) for every front-led / non-stop move.
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
        rear_window_cm=rear_window_cm,
    )
    sub = sample_drive(profile, (x0, y0, h0), (x1, y1, h1), prev_kf["t"])
    # sub[0] duplicates prev_kf (byte-equal pose at the same t); drop it.
    return sub[1:]


# ---------------------------------------------------------------------------
# Anticipatory line-approach ramp rendering (T10, ADR-058 display layer)
# ---------------------------------------------------------------------------
#
# A terminating LINE hop (a non-pass-through ``ACT_LINE_OVERSHOOT`` roll onto
# the node) firmware-side ramps cruise -> the kinetic floor (``PWM_MOVING_MIN``)
# over a zone that bottoms out ``LINE_APPROACH_MARGIN_CM`` BEFORE the predicted
# sensor-crossing distance, holds the floor through detection, then the
# overshoot roll re-arms its OWN (unchanged, existing) floor -> 0 brake — see
# the 2026-07-17 anticipatory-line-approach-ramp plan (T2/T4/T6/T8). CTPS
# renders this PURELY from the run's own geometry + the parsed firmware Config
# (never a per-hop velocity read off the sim, matching every other primitive
# this module renders): the follow leg immediately preceding a terminal
# ``ACT_LINE_OVERSHOOT`` IS the predicted sensor-crossing distance (the sim
# already lands its END pose one axle-offset short of the node, by
# construction — no separate offset subtraction is needed here).
#
# The render below reuses the golden-pinned :func:`resolve_approach_schedule` /
# :func:`approach_speed_at` kernel (T2/T4) for the schedule and its per-distance
# magnitude, but does NOT hand the whole terminating edge to
# :func:`resolve_drive_profile` in one call: that kernel's half-of-total-distance
# ramp cap (:func:`resolve_tshape_ramps`) is scoped to treat its ``dist_cm`` as
# the WHOLE move, which would misfit a decel ramp that is deliberately confined
# to a SUB-window of the follow leg (bottoming out short of the crossing, then
# holding). Three phases are stitched instead, each independently well-behaved:
# (A) any leading collinear-pass-through prefix plus the follow leg's own
# ramp-up/cruise, rendered via the EXISTING whole-run mechanism clipped to
# ``schedule.ramp_start_cm`` (an unbraked ``end_pwm == cruise`` sub-profile, so
# the cap only ever bounds the legitimate ramp-up); (B) the dense RAMP +
# FLOOR_HOLD deceleration, sampled directly off ``approach_speed_at``; (C) the
# overshoot roll's own floor -> 0 brake, seeded AT the floor magnitude (AC3's
# monotonic detect-handoff), via the same kernel the plain path already uses.


def _resolve_line_approach_schedule(
    predicted_cm: float, cfg
) -> Optional["ApproachSchedule"]:
    """Resolve the anticipatory approach-ramp schedule for a terminating LINE hop.

    Thin wrapper around :func:`~tools.path_planner.ctps.profile.resolve_approach_schedule`
    (the T2/T4 cross-language-pinned kernel): feeds the parsed firmware Config
    (the SAME ``TSHAPE_*`` / ``LINE_APPROACH_*`` / ``PWM_MOVING_MIN`` values the
    real robot runs with) plus the LineFollower cruise
    (``ConfigLine::CRUISE_SPEED``) as the cruise/floor magnitudes.

    :param predicted_cm: the follow leg's own chord distance (cm) — the CTPS
        stand-in for the firmware's ``predictedCm = _pivotTrimDistance(edge) -
        axleOffsetCm``. The sim's follow-leg advance is ALREADY that
        axle-offset-adjusted quantity by construction (it stops one axle
        offset short of the node so the separate overshoot roll can land on
        it), so no separate offset subtraction is needed here.
    :param cfg: the parsed :class:`~tools.path_planner.ctps.profile.ProfileConfig`.
    :return: the active :class:`ApproachSchedule`, or ``None`` when the
        resolved schedule is degenerate (``active == False`` — a short hop
        below the engage threshold, AC7's documented flat fallback) so the
        caller renders the ordinary flat/braked trapezoid unchanged.
    """
    schedule = resolve_approach_schedule(
        predicted_cm,
        cfg.approach_margin_cm,
        cfg.approach_max_margin_cm,
        int(cfg.line_cruise),
        int(cfg.pwm_moving_min),
        cfg.accel,
        cfg.ramp_min,
        cfg.ramp_max,
        cfg.no_ramp_eps,
        cfg.min_move_cm,
    )
    return schedule if schedule.active else None


def _lerp_pose(
    p0: Tuple[float, float, float], p1: Tuple[float, float, float], frac: float
) -> Tuple[float, float, float]:
    """Linear-interpolate a pose ``frac`` of the way from ``p0`` to ``p1``.

    x/y lerp linearly; heading takes the shortest arc (:func:`wrap_pm180`),
    matching every other pose interpolation in this module and
    :func:`~tools.path_planner.ctps.profile.sample_drive`. ``frac`` is not
    clamped — callers pass values already known to lie in ``[0, 1]``.

    :param p0: ``(x, y, heading_deg)`` at ``frac == 0``.
    :param p1: ``(x, y, heading_deg)`` at ``frac == 1``.
    :param frac: interpolation fraction.
    :return: the interpolated ``(x, y, heading_deg)``.
    """
    x0, y0, h0 = p0
    x1, y1, h1 = p1
    dh = wrap_pm180(h1 - h0)
    return (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac, (h0 + dh * frac) % 360.0)


def _sample_approach_ramp_keyframes(
    follow_seg: "_RunSegment",
    schedule: "ApproachSchedule",
    predicted_cm: float,
    t0: float,
    cfg,
) -> List[Dict[str, Any]]:
    """Dense-sample the RAMP + FLOOR_HOLD phases of an active approach schedule.

    Renders the anticipatory deceleration AC12/TC15 ask CTPS to show: the
    follow leg's interior from ``schedule.ramp_start_cm`` (where the caller's
    leading cruise/ramp-up prefix already lands — the CRUISE -> RAMP
    handoff) through ``predicted_cm`` (the sensor-crossing point;
    ``schedule.floor_start_cm``'s FLOOR_HOLD tail is included), sampled every
    ``RAMP_SAMPLE_STEP_CM`` (bounded by ``KEYFRAME_COUNT_CEILING``) via
    :func:`~tools.path_planner.ctps.profile.approach_speed_at` directly — the
    T2/T4-pinned kernel, not a re-derivation.

    Poses are lerped along the follow leg's own straight chord by fraction of
    ``predicted_cm`` (matching this module's general drive-densification
    convention). Timestamps are a trapezoidal-average Euler integration of
    ``ds/v`` — display-only, deliberately bypassing the ``DriveProfile``
    kernel's exact closed-form timing here: that kernel's half-of-total-distance
    ramp cap is scoped to treat its argument as a WHOLE move and would misfit
    this sub-window (see the section banner above).

    :param follow_seg: the run segment for the LINE-follow leg immediately
        PRECEDING the terminal ``ACT_LINE_OVERSHOOT`` roll; its ``dist_cm``
        MUST equal ``predicted_cm``.
    :param schedule: the ACTIVE schedule from
        :func:`_resolve_line_approach_schedule`.
    :param predicted_cm: the follow leg's own length (== ``follow_seg.dist_cm``).
    :param t0: the running clock at ``schedule.ramp_start_cm`` (after the
        caller already rendered the leading cruise/ramp-up prefix).
    :param cfg: the parsed :class:`~tools.path_planner.ctps.profile.ProfileConfig`
        (kinetic floor + cruise).
    :return: dense keyframes strictly AFTER ``t0``, ending at ``predicted_cm``
        (``follow_seg.end_pose``, byte-exact); empty if the window is
        zero-length (never expected on an ``active`` schedule, guarded
        defensively).
    """
    lo = schedule.ramp_start_cm
    hi = predicted_cm
    length = hi - lo
    if length <= _TRANSLATION_EPS_CM:
        return []

    n_steps = max(1, ceil(length / RAMP_SAMPLE_STEP_CM))
    n_steps = min(n_steps, KEYFRAME_COUNT_CEILING - 1)
    step = length / n_steps
    sample_distances = [lo + step * i for i in range(1, n_steps)]
    sample_distances.append(hi)

    kinetic_floor = float(cfg.pwm_moving_min)

    x0, y0, h0 = follow_seg.start_pose
    x1, y1, h1 = follow_seg.end_pose
    dh = wrap_pm180(h1 - h0)

    def _velocity_at(d: float) -> float:
        speed = approach_speed_at(schedule, d)
        return SCALE_CM_S_PER_PWM * max(float(speed.pwm_mag), kinetic_floor)

    d_prev = lo
    v_prev = _velocity_at(lo)
    t_running = t0

    keyframes: List[Dict[str, Any]] = []
    for d in sample_distances:
        v = _velocity_at(d)
        t_running += (d - d_prev) / ((v + v_prev) / 2.0)
        frac = d / predicted_cm
        keyframes.append(
            {
                "t": t_running,
                "x": x0 + (x1 - x0) * frac,
                "y": y0 + (y1 - y0) * frac,
                "h": (h0 + dh * frac) % 360.0,
            }
        )
        d_prev, v_prev = d, v

    # Pin the last sample byte-exact to the follow leg's own snapshot pose (the
    # TC7 boundary-pose invariant every other densifier in this module upholds).
    keyframes[-1] = {"t": keyframes[-1]["t"], "x": x1, "y": y1, "h": h1}
    return keyframes


# ---------------------------------------------------------------------------
# Collinear run-grouper (CTPS truth-fix, T3)
# ---------------------------------------------------------------------------
#
# A collinear pass-through RUN is a chain of geometry primitives bounded by
# braked stops: the follower ramps up once at the run start, holds cruise
# THROUGH every intermediate (pass-through) node, and brakes to rest once at the
# run end. Before this fix the CTPS builder densified each primitive on its own
# with ``end == 0``, so it rendered a full stop (ramp-down + floor-crawl +
# brake) at EVERY node — a per-node dip that the firmware does not actually
# perform (ADR-023 line pass-through, plus the T2 projection ramp-down fix).
#
# The grouper (below, inside ``build``) buffers a run's geometry segments and
# flushes the whole run as ONE velocity profile over the summed chord distance,
# sliced back onto each segment's node boundary — so the render is one trapezoid
# with no intra-run stop keyframes, while every segment's LAST sub-keyframe
# still pins its sim-snapshot pose byte-exact (TC7). Zero-motion records
# (heading resets, in-place turns) are transparent: they neither contribute
# distance nor break a run. The structure is intentionally reusable — a later
# task extends it to BLIND runs by feeding the recorded per-hop speeds instead
# of the resolved run cruise.


@dataclass
class _RunSegment:
    """One geometry primitive's contribution to a buffered collinear run.

    :param start_pose: ``(x, y, heading_deg)`` at the segment start (the
        previous keyframe's pose when the segment was buffered).
    :param end_pose: ``(x, y, heading_deg)`` at the segment end — the sim
        snapshot pose, pinned byte-exact as the segment's last sub-keyframe.
    :param dist_cm: the segment's own chord length (cm); the run profile is
        built over the sum of these.
    :param braked: ``True`` iff this segment is a braked distance-gated stop
        (the run's terminal segment). A pass / mid-run follow leg is ``False``.
    :param chain_start_pwm: for a BLIND-chain hop (ADR-059), the RECORDED signed
        start PWM the sim/firmware resolved for this hop; ``None`` for a
        line/projection run (whose cruise is the shared follow cruise, resolved
        from ``run_speed`` instead). A segment carrying this is a "blind-run"
        segment — ``flush_run`` builds the whole-run trapezoid from the recorded
        per-hop endpoints, never re-deriving the chain decision (AC11).
    :param chain_end_pwm: the RECORDED signed end PWM for a blind-chain hop
        (0 on the run's final braked hop; the mid-hop cruise otherwise);
        ``None`` for a line/projection run.
    :param rear_led: ``True`` iff this segment is a REAR-led braked
        distance-gated stop (:data:`_REAR_LED_STOP_TAGS`). Meaningful only on a
        run's braked terminal segment: ``flush_run`` then fits the run's
        cruise -> floor shed into the REAR overshoot window (the AC8
        LineFollower window-fit). Always ``False`` on a blind-chain segment —
        a chained final hop is a Drive dead-reckon stop, not a LineFollower
        distance-gated one.
    :param approach_predicted_cm: set ONLY on a terminating (non-pass-through)
        ``ACT_LINE_OVERSHOOT`` roll segment (T10, ADR-058 display layer), to
        the chord distance of the follow leg immediately PRECEDING it — the
        CTPS-computed "predicted sensor-crossing distance"
        (:func:`_resolve_line_approach_schedule`'s input). ``None`` for every
        other segment kind (pass tags, blind, stop-at-line, projection follow)
        — those keep today's plain whole-run trapezoid unchanged.
    """

    start_pose: Tuple[float, float, float]
    end_pose: Tuple[float, float, float]
    dist_cm: float
    braked: bool
    chain_start_pwm: Optional[int] = None
    chain_end_pwm: Optional[int] = None
    rear_led: bool = False
    approach_predicted_cm: Optional[float] = None


def _build_run_profile(
    total_cm: float,
    cruise: int,
    start: int,
    braked: bool,
    profiled: bool,
    rear_window_cm: Optional[float] = None,
):
    """Resolve the ONE :class:`DriveProfile` for a whole buffered run.

    The run ramps up once (from ``start``), holds ``cruise`` through every
    intermediate pass-through node, and — when the run ends on a braked stop —
    ramps down to rest once at the very end. A run that ends WITHOUT a braked
    stop (a degenerate all-pass run, e.g. a malformed trace whose final pass has
    no following braked stop, or the reusable blind-pass path a later task adds)
    holds ``cruise`` to the end (``end == cruise``) so it renders no phantom
    terminal dip.

    :param total_cm: the run's summed chord distance (cm).
    :param cruise: signed run cruise PWM.
    :param start: signed run start-seed PWM.
    :param braked: ``True`` iff the run terminates in a braked stop (end == 0).
    :param profiled: ``True`` for the ramped families, ``False`` for a flat run.
    :param rear_window_cm: the REAR overshoot window when the run's braked
        terminal stop is REAR-led (:data:`_REAR_LED_STOP_TAGS`) — the AC8
        LineFollower window-fit compresses the cruise -> floor shed into it;
        ``None`` (the default) for a front-led or unbraked run.
    :return: the resolved :class:`~tools.path_planner.ctps.profile.DriveProfile`.
    """
    end = 0 if braked else cruise
    return resolve_drive_profile(
        total_cm,
        cruise_pwm=cruise,
        start_pwm=start,
        end_pwm=end,
        profiled=profiled,
        rear_window_cm=rear_window_cm,
    )


def _resolve_blind_run_profile(
    segs: List["_RunSegment"],
    run_speed: Optional[Tuple[int, int, int, bool]] = None,
):
    """Build the whole-run :class:`DriveProfile` for a BLIND-chain run from RECORDED speeds.

    ADR-059 / AC11: a collinear BLIND chain that the sim (or firmware recorder)
    drove as ONE continuous run rides the resolved per-hop ``(start_pwm,
    end_pwm)`` on each blind advance primitive. CTPS **consumes** those recorded
    endpoints to render the whole-run trapezoid — it NEVER re-derives a chain
    decision (does not call ``chain_speed.resolve_chain_speeds``). The chain
    planning already happened upstream; this only maps the recorded endpoints
    onto a single :class:`DriveProfile` over the run's summed chord distance.

    Two well-formed shapes reach here:

    * **pure blind run** — EVERY segment carries recorded speeds.
    * **LINE/projection → BLIND spanning run** (the plan's Decision "a collinear
      run MAY span LINE/projection → BLIND") — LEADING speed-less line /
      projection pass segments feed straight into the recorded blind chain.
      Those leading segments legitimately carry NO chain speeds (line-follow
      speed comes from the LineFollower preset, not per-hop ``ChainSpeedPolicy``),
      so they are NOT malformed: they contribute their distance at the RUN
      CRUISE — they ARE the line-follow-at-cruise portion. The firmware passes
      through at cruise and hands the captured cruise into the first blind hop
      (ADR-023, ``PathTracker.cpp:2674-2690``), so the physical motion is ONE
      continuous trapezoid with velocity continuous across the handoff.

    Profile endpoints:

    * **start** — pure blind run: the FIRST hop's recorded ``start_pwm`` (the
      chain ``entryPwm`` the planner resolved: the captured cruise after a
      pass-through, else the cold-start seed — the ADR-023 cruise carry, AC5).
      Spanning run: the run launched from rest on its leading line-follow leg,
      so the start is the ordinary run start seed (``run_speed``'s start — the
      same seed the line-only branch uses), signed by the run direction.
    * **cruise (the run peak)** — the maximum recorded |PWM| across every chain
      hop endpoint. The two-pass planner clamps every node speed to ``[pwmMin,
      |cruise|]``, so the largest recorded magnitude IS the peak the run reached
      — read from the record, not recomputed. After a pass-through the chain's
      cruise equals the captured line cruise (the same physical cruise), so the
      leading line segments ride this same peak.
    * **end** — the FINAL hop's recorded ``end_pwm`` (``0`` on a braked chain, so
      the single trapezoid ramps down to the ordinary cruise→floor→brake stop).

    The run direction (sign) comes from the recorded endpoints: a reverse chain
    records negative PWMs, so the peak is signed by the first chain hop's start
    sign. The single trapezoid's interior slice at each node boundary (via
    :func:`sample_drive_run`) reproduces the pass-through-then-shed shape without
    a per-node dip.

    Returns ``None`` when the run is genuinely MALFORMED — a speed-less segment
    AT OR AFTER the first recorded chain hop (a BLIND hop missing ITS OWN
    recorded ``(start_pwm, end_pwm)``: the recorded chain is incomplete). The
    caller then falls back to per-primitive rendering + a ``UserWarning``
    (AC11 / TC11). Only LEADING speed-less segments are legitimate.

    :param segs: the buffered run segments (at least one carries recorded chain
        speeds — the caller's blind-run classification).
    :param run_speed: the run's first-segment ``(cruise, start, end, profiled)``
        seed; used ONLY to seed a spanning run's start. ``None`` falls back to
        the parsed Config start seed.
    :return: the resolved run :class:`DriveProfile`, or ``None`` if malformed.
    """
    # The chain proper begins at the first segment carrying recorded speeds;
    # everything before it is the run's leading line/projection pass portion
    # (a legitimate spanning run, never malformed).
    first_chain = next(
        (i for i, seg in enumerate(segs) if seg.chain_start_pwm is not None),
        None,
    )
    if first_chain is None:
        # Unreachable via ``flush_run`` (the caller classified the run blind off
        # a speed-carrying segment); defensively malformed.
        return None
    chain_segs = segs[first_chain:]

    # Malformed: from the first recorded chain hop onward, EVERY segment must
    # carry recorded speeds. A gap (a BLIND hop that lost its own suffix) means
    # the recorded chain is incomplete — reject rather than fabricate a cruise
    # for the missing hop.
    if any(
        seg.chain_start_pwm is None or seg.chain_end_pwm is None
        for seg in chain_segs
    ):
        return None

    first = chain_segs[0]
    last = chain_segs[-1]
    end = int(last.chain_end_pwm)

    # The run peak is the largest recorded |PWM| across all chain hop endpoints
    # (the planner clamps to |cruise|, so the observed max IS the peak). Sign it
    # by the run direction (the first chain hop's start sign; a cold-start floor
    # seed and the cruise share a sign, so this is unambiguous).
    peak_mag = 0
    for seg in chain_segs:
        peak_mag = max(
            peak_mag, abs(int(seg.chain_start_pwm)), abs(int(seg.chain_end_pwm))
        )
    direction = -1 if int(first.chain_start_pwm) < 0 else 1
    cruise = direction * peak_mag

    if first_chain == 0:
        # Pure blind run: the chain's own recorded entry (the resolved entryPwm).
        start = int(first.chain_start_pwm)
    else:
        # Spanning run: the run launched from rest on its leading line-follow
        # leg, so it seeds the ordinary run start (the same seed the line-only
        # branch uses), signed by the run direction — one ramp-up, then cruise
        # held across the line → blind handoff (velocity continuous, no dip).
        seed = (
            int(run_speed[1])
            if run_speed is not None
            else int(default_config().start_speed)
        )
        start = direction * abs(seed)

    total_cm = sum(seg.dist_cm for seg in segs)
    return resolve_drive_profile(
        total_cm,
        cruise_pwm=cruise,
        start_pwm=start,
        end_pwm=end,
        # A blind(-terminated) run is always a profiled (.tShape) run — the
        # whole-run trapezoid is exactly the ramped shape the recorded
        # endpoints describe.
        profiled=True,
    )


def _fallback_segment_speed(seg: "_RunSegment") -> Tuple[int, int, int, bool]:
    """Resolve one rejected-run segment's ``(cruise, start, end, profiled)`` for fallback.

    When the run-grouper rejects a malformed run and renders it per-primitive
    (:func:`build`'s ``_fallback_per_primitive``), each segment needs its own
    profile. A BLIND-chain segment (it carries recorded endpoints) renders with
    its RECORDED ``(start_pwm, end_pwm)`` — the honest per-hop shape even though
    the run as a whole was malformed — with the cruise held at the larger
    magnitude of the two endpoints so the segment's own ramp is coherent. A
    segment with no recorded speeds (a line/projection segment in a
    pass-without-stop run) renders as a held-cruise pass (``end == cruise``,
    no phantom decel), matching the per-primitive pass-tag render.

    :param seg: the rejected run segment.
    :return: ``(cruise, start, end, profiled)`` for :func:`_densify_drive`.
    """
    if seg.chain_start_pwm is not None and seg.chain_end_pwm is not None:
        start = int(seg.chain_start_pwm)
        end = int(seg.chain_end_pwm)
        direction = -1 if start < 0 else 1
        cruise = direction * max(abs(start), abs(end))
        return cruise, start, end, True
    # Line/projection segment with no recorded speed: a held-cruise pass (the
    # per-primitive pass-tag shape — never a decel-to-rest).
    cfg = default_config()
    cruise = int(cfg.line_cruise)
    start = int(cfg.start_speed)
    return cruise, start, cruise, True


def _recorded_drive_rear_window(
    cmd: Command, speed: Tuple[int, int, int, bool]
) -> Optional[float]:
    """REAR overshoot window for a recorded rear-led ``stopAtLine``, else ``None``.

    A recorded ``stopAtLine`` with ``reverse == True`` is a REAR-led
    distance-gated stop: the rear board detects the crossing line, then the
    firmware rolls the rear axle offset (``LINE_SENSOR_AXLE_OFFSET_REAR_CM`` —
    ``PathTracker``'s ``reverse ? REAR : FRONT`` pick) shedding cruise -> floor
    inside that window (the ``LineFollower.cpp:287-301`` ramp-down window-fit).
    The render fits the shed the same way (AC8): at the FAST preset the natural
    shed (8 cm) overruns the 7.68 cm rear window, so an unfitted render would
    show a floor-crawl tail the firmware never drives.

    Every other recorded shape keeps ``None``:

    * ``stopAtLine`` FORWARD — front-led; the fit is defined only over the rear
      window (and the larger FRONT window does not bind the FAST shed anyway).
    * ``driveDist`` — a dead-reckon Drive move; the T-shape kernel already fits
      its ramps to the known distance, no LineFollower gate exists.
    * ``lineFollowDist`` — its distance gate is the COMMANDED cm (the
      ``stopAfterCm`` clamp the kernel's half-of-total cap models), not the
      axle-offset roll — in either direction.
    * ``wallStop`` — a flat constant-push crawl, no ramp to fit.
    * a non-braked resolve (``.setBrake(false)`` / explicit ``end_speed``) or a
      ``.flat()`` move — not a braked profiled stop, so no shed to fit.

    :param cmd: the recorded drive command (carries the strict-bool ``reverse``).
    :param speed: the move's resolved ``(cruise, start, end, profiled)``.
    :return: the rear window (cm) for a rear-led braked ``stopAtLine``;
        ``None`` otherwise.
    """
    if cmd.method != "stopAtLine" or not cmd.reverse:
        return None
    _cruise, _start, end, profiled = speed
    if end != 0 or not profiled:
        return None
    return float(default_config().rear_axle_offset_cm)


def _densify_turn(
    prev_kf: Dict[str, Any],
    end_pose: Tuple[float, float, float],
    signed_turn_deg: Optional[float] = None,
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
    :param signed_turn_deg: the signed commanded rotation when the caller knows it
        (a raw ``turnDeg`` argument), threaded to :func:`sample_turn` only to
        disambiguate the exact ±180 in-place boundary. ``None`` for pose-derived
        turns (the goto walk), which keep the shortest-arc direction.
    :return: the replacement sub-keyframes (empty for a zero-magnitude turn).
    """
    x0, y0, h0 = prev_kf["x"], prev_kf["y"], prev_kf["h"]
    x1, y1, h1 = end_pose
    theta_deg = wrap_pm180(h1 - h0)

    profile = resolve_turn_profile(theta_deg, is_pivot=False)
    sub = sample_turn(
        profile,
        (x0, y0, h0),
        (x1, y1, h1),
        prev_kf["t"],
        signed_theta_deg=signed_turn_deg,
    )
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
    the pre-#130 builder; only the TIMING now comes from the CTPS turn profile
    (ADR-068 T8: display-only, not a physics sim). The ``N`` samples are
    re-timed by :func:`tools.path_planner.ctps.profile.sample_turn` (the
    ``arc_keyframes`` re-time path): the whole arc occupies
    ``resolve_turn_profile(theta).duration_s()`` seconds at a SINGLE constant
    rate, with the final sub-keyframe alone also carrying the fixed nominal
    settle-allowance tail — so the running clock stays monotonic
    non-decreasing.

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
    # Cap the sub-keyframe count at KEYFRAME_COUNT_CEILING - 1 so a very large
    # pivot (or a future lower ARC_STEP_DEG) cannot exceed the ceiling that bounds
    # every primitive — matching sample_turn's in-place path, so the two turn
    # paths agree.
    n = max(1, ceil(abs_theta / ARC_STEP_DEG))
    n = min(n, KEYFRAME_COUNT_CEILING - 1)

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
        # Line-follow lateral hug bias (issue #156) — shown only when the follow
        # recorded a non-zero setLineFollowOffset (lineFollowDist only), so a
        # centered / MIDDLE follow keeps its exact base label.
        if cmd.follow_offset:
            label += f" offset={cmd.follow_offset}"
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
    # Field-sourced pipeline config, resolved ONCE (mirrors the sim's
    # ``_walk_hops``: ``PipelineConfig.from_field_data(self._field_data)``). The
    # per-edge SpeedTier resolution below MUST use this, not the module singleton
    # ``PIPELINE_CONFIG`` — a field that sets a mission-wide ``PIPELINE_SPEED``
    # inherits through it, so an inheriting lone blind hop renders at the same tier
    # the robot / sim run (the two-resolution-surfaces trap). ``from_field_data``
    # falls back to the singleton for a fixture lacking the key, so this is safe
    # for every field.
    pipeline_cfg = PipelineConfig.from_field_data(field_data)

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

    # Post-stop settle dwell (Config::HOP_SETTLE_MS, ADR-061), owned here in
    # SECONDS: ``ProfileConfig.hop_settle_ms`` stays ms (T5's parse surface),
    # the render call sites own the ms -> s conversion. ``0`` (the config
    # default disabled, or a firmware ``-DHOP_SETTLE_MS_DEFAULT=0`` pin) fully
    # disables every dwell insertion below, so the render stays byte-identical
    # to pre-feature. See the Settle-fire matrix in
    # docs/plans/2026-07-12-hop-settle-wait.md — the paired firmware host
    # ctest (T4) and this render (T7's pytest) both assert against that same
    # matrix; a future edit to either eligibility mapping must update both.
    hop_settle_s: float = default_config().hop_settle_ms / 1000.0
    # Deferred one-primitive dwell-2 flag: set by a rendered COLD pre-edge turn
    # on a normal hop (either the plain in-place turn or the pivot-arc path),
    # consumed on the NEXT real drive advance (turn -> move), matching the
    # firmware's SETTLE(EDGE_AFTER_TURN). A zero-motion heading-only re-anchor
    # between the turn and the drive must NOT clear it — only the advance
    # branch (the ``else`` of the zero-motion check) does.
    pending_turn_dwell: bool = False

    # Optional single-wheel pivot-arc overlay segments (issue #55). Appended in
    # chronological (t0-increasing) order as each goto's route takes a declared
    # pivot; stays empty — and the ``pivots`` key is omitted from the Track — for
    # every pivot-free trace, so a ``PIVOT_TURNS``-empty field is byte-identical
    # to before this change (AC6).
    pivots: List[Dict[str, Any]] = []
    # Optional visualization-only lateral hug-side overlay segments (issue #156).
    # Appended in chronological (t0-increasing) order as each biased
    # ``lineFollowDist`` leg renders; stays empty — and the ``lateral_offsets``
    # key is omitted from the Track — for every centered / MIDDLE trace, so a
    # bias-free trace's Track JSON is byte-identical. The nudge is a DRAW-time
    # overlay (ADR-054 pivot-arc precedent): the keyframes below stay on the
    # centerline and the sim / LocalizationSim pose is never touched.
    lateral_offsets: List[Dict[str, Any]] = []
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
        rear_window_cm: Optional[float] = None,
    ) -> None:
        """Densify a drive/line advance to ``(x_new, y_new, h_new)`` and append it.

        ``speed`` is the resolved ``(cruise, start, end, profiled)`` for this
        step; the straight-line advance from the last keyframe is profiled and
        sampled dense-near-ramps. The final sub-keyframe is pinned to the
        destination pose byte-exact. ``rear_window_cm`` (a rear-led braked
        distance-gated stop only) threads the AC8 LineFollower window-fit to
        the profile.
        """
        cruise, start, end, profiled = speed
        _extend(
            _densify_drive(
                keyframes[-1],
                (x_new, y_new, h_new),
                cruise,
                start,
                end,
                profiled,
                rear_window_cm=rear_window_cm,
            )
        )

    # -- collinear run buffer (T3) ------------------------------------------
    # Geometry drive primitives inside a goto are buffered here instead of being
    # densified one-at-a-time, so a whole collinear pass-through run renders as
    # ONE trapezoid (see the ``_RunSegment`` / ``_build_run_profile`` note). The
    # buffer holds the run's ordered segments; ``_run_speed`` is the run's
    # resolved ``(cruise, start, end, profiled)`` seed, taken from the FIRST
    # segment's primitive (every mid-run primitive in a line/projection run
    # shares the follow cruise, so the run cruise is well-defined). ``flush_run``
    # emits the accumulated run and clears the buffer; it is called at every run
    # boundary — a braked stop, a turn / pivot / dwell, or the goto's end.
    run_segments: List[_RunSegment] = []
    run_speed: Optional[Tuple[int, int, int, bool]] = None

    def _fallback_per_primitive(segs: List[_RunSegment], reason: str) -> None:
        """Render a rejected run per-primitive (each its own profile) + warn.

        The malformed-run escape hatch (AC11 / TC11): when the grouper cannot
        build a coherent whole-run profile — a blind-chain run MISSING its
        recorded per-hop speeds, or a pass tag with NO following braked stop —
        it does NOT guess a run cruise. It falls back to the pre-grouper
        behavior: densify EACH buffered segment on its own via
        :func:`_densify_drive`, so every segment still renders (nothing is
        dropped) and each pins its snapshot pose. A :class:`UserWarning` surfaces
        the malformed structure so a trace bug is visible rather than silently
        smoothed over. Zero-motion records never reach here (they were
        transparent), so this only fires on a genuinely broken run.

        :param segs: the buffered run segments (already popped from the buffer).
        :param reason: a human-readable description of the malformation, for the
            warning message.
        """
        warnings.warn(
            f"ctps.track_builder: malformed collinear run rejected "
            f"({reason}) — falling back to per-primitive rendering.",
            UserWarning,
            stacklevel=2,
        )
        for seg in segs:
            cruise, start, end, profiled = _fallback_segment_speed(seg)
            _extend(
                _densify_drive(
                    keyframes[-1], seg.end_pose, cruise, start, end, profiled
                )
            )

    def flush_run(strict: bool = False) -> None:
        """Emit the buffered collinear run as ONE profile, then clear the buffer.

        Builds a single :class:`DriveProfile` over the run's summed chord
        distance (ramp up once, hold cruise through the intermediate nodes, ramp
        down once iff the run ends braked), slices it back onto each segment's
        node boundary via
        :func:`tools.path_planner.ctps.profile.sample_drive_run`, and appends the
        per-segment sub-keyframes — each segment's last sub-keyframe still
        byte-pins its snapshot pose (TC7). A single-segment run reduces to the
        same shape the old per-primitive path produced (one ramp/stop), so a
        non-chain hop is unchanged. An empty buffer is a no-op.

        Two run shapes are grouped:

        * **line / projection** — every segment shares the follow cruise, so the
          run cruise is the ``run_speed`` seed (resolved from the first
          segment's primitive kind).
        * **blind chain** (ADR-059) — the segments carry the RECORDED per-hop
          ``(start_pwm, end_pwm)`` the sim/firmware resolved; the whole-run
          trapezoid is built from those recorded endpoints (start = first hop's
          recorded start, cruise = the peak the run actually reached, end = the
          final hop's recorded end), NEVER by re-deriving a chain decision
          (AC11). A run MAY span LINE/projection → BLIND: leading speed-less
          line/projection pass segments are the run's line-follow-at-cruise
          portion (the ADR-023 pass-through cruise carry), so they contribute
          distance at the run cruise and the run start falls back to the
          ordinary ``run_speed`` seed — one continuous trapezoid across the
          handoff. A BLIND (chain) hop missing ITS OWN recorded speeds is
          malformed and falls back to per-primitive rendering.

        :param strict: gate for the pass-without-braked-stop malformed check.
            Passed ``True`` ONLY at the GOTO-END flush, where a healthy goto
            always ends on a braked goal arrival — so a GROUPED (multi-segment)
            unbraked run there is a genuinely malformed trace (a pass tag with NO
            following braked stop) and falls back to per-primitive rendering + a
            ``UserWarning`` (AC11 / TC11). Every OTHER flush is an INTERIOR run
            boundary (a turn / pivot / dwell / pending-pivot landing) or a
            fault-handler flush of a legitimate reachable prefix — where an
            unbraked pass run is EXPECTED (the run ends because the route turns,
            not because it braked) — so those flush non-strict (the default) and
            never warn. The blind-missing-speeds malformation is ALWAYS rejected
            (a gap in the recorded chain is broken regardless of the boundary).
        """
        nonlocal t, run_segments, run_speed
        if not run_segments:
            return
        segs = run_segments
        # ``speed`` feeds the line/projection branch below AND seeds a
        # LINE/projection → BLIND spanning run's start (the run launches from
        # rest on its leading line-follow leg); a PURE blind run ignores it by
        # design (its profile is CONSUMED from the recorded per-hop endpoints,
        # AC11).
        speed = run_speed
        # Reset the buffer BEFORE emitting so a re-entrant tick callback (the
        # sim never re-enters ``build``, but keep the invariant local) sees a
        # clean buffer.
        run_segments = []
        run_speed = None

        braked = segs[-1].braked
        # A run whose segments carry recorded chain endpoints is a BLIND run: its
        # whole-run trapezoid is CONSUMED from the recorded speeds, never
        # re-derived. Any segment carrying a recorded start/end marks the run.
        is_blind_run = any(seg.chain_start_pwm is not None for seg in segs)

        if is_blind_run:
            resolved = _resolve_blind_run_profile(segs, speed)
            if resolved is None:
                # Missing recorded speeds on a BLIND (chain) hop → malformed. Do
                # not guess a cruise; fall back to per-primitive rendering +
                # warn. Always rejected — a gap in the recorded chain is broken
                # even on the fault-flush path. (LEADING speed-less line /
                # projection pass segments are a legitimate spanning run and
                # never trip this.)
                _fallback_per_primitive(
                    segs, "blind run missing recorded per-hop speeds"
                )
                return
            profile = resolved
        else:
            if strict and not braked and len(segs) > 1:
                # A grouped (multi-segment) line/projection run that never braked
                # is malformed: a pass tag opened a run with no following braked
                # stop. A DEGENERATE single-segment lone-pass run is NOT malformed
                # (it renders as its own held-cruise segment — the T3 contract),
                # and a NON-strict fault-flush of a reachable prefix is a
                # legitimate dead-end stop, so only reject a genuinely grouped run
                # at a normal run boundary.
                _fallback_per_primitive(
                    segs, "pass tag with no following braked stop"
                )
                return
            # T10 (ADR-058 display layer): a terminating (non-pass-through)
            # LINE hop's run carries a predicted sensor-crossing distance on
            # its terminal segment (set only for ACT_LINE_OVERSHOOT). Render
            # the anticipatory approach-ramp deceleration instead of the plain
            # whole-run trapezoid when the resolved schedule is active; a
            # degenerate schedule (short hop / disabled) falls through to the
            # ordinary render below, unmodified (AC7 flat fallback).
            approach_predicted_cm = (
                segs[-1].approach_predicted_cm if braked else None
            )
            if approach_predicted_cm is not None and len(segs) >= 2:
                if _emit_line_approach_run(segs, speed, approach_predicted_cm):
                    return
            cruise, start, _end_seed, profiled = speed
            total_cm = sum(seg.dist_cm for seg in segs)
            # AC8 LineFollower window-fit: a run ending on a REAR-led braked
            # distance-gated stop (_REAR_LED_STOP_TAGS) only has the REAR
            # overshoot window (LINE_SENSOR_AXLE_OFFSET_REAR_CM) to shed
            # cruise -> floor, so the run profile fits the shed into it. A
            # front-led or unbraked run keeps None (the natural shed).
            rear_window = (
                default_config().rear_axle_offset_cm
                if braked and segs[-1].rear_led
                else None
            )
            profile = _build_run_profile(
                total_cm, cruise, start, braked, profiled, rear_window_cm=rear_window
            )

        boundaries = [
            (seg.start_pose, seg.end_pose, seg.dist_cm) for seg in segs
        ]
        sliced = sample_drive_run(profile, boundaries, keyframes[-1]["t"])
        for sub in sliced:
            _extend(sub)

    def _emit_line_approach_run(
        segs: List["_RunSegment"],
        speed: Tuple[int, int, int, bool],
        predicted_cm: float,
    ) -> bool:
        """Render a terminating LINE hop's run with the anticipatory approach ramp.

        Replaces the ordinary whole-run trapezoid (T10, ADR-058 display layer)
        for a run whose LAST segment is a non-pass-through
        ``LINE_FOLLOW_INTERSECTION`` roll (``segs[-1].approach_predicted_cm``
        set) with three profile-timed phases, chained onto the running clock:

        (A) any leading collinear-pass-through prefix plus the follow leg's own
        ramp-up/cruise, rendered via the EXISTING whole-run mechanism clipped
        to ``schedule.ramp_start_cm`` within the follow leg (an unbraked
        ``end_pwm == cruise`` sub-profile carries no decel ramp of its own, so
        the half-of-total-distance cap only ever bounds the legitimate
        ramp-up);
        (B) the dense RAMP + FLOOR_HOLD deceleration, sampled directly off
        ``approach_speed_at`` (:func:`_sample_approach_ramp_keyframes`);
        (C) the overshoot roll's own floor -> 0 brake, seeded AT the floor
        magnitude (AC3's monotonic detect-handoff), via the SAME T-shape
        kernel the plain path already uses.

        Falls through (returns ``False``, appending nothing) when the resolved
        schedule is degenerate (AC7 — a short hop below the engage threshold):
        the caller then renders the ordinary flat/braked trapezoid unchanged.

        :param segs: the buffered run's segments; ``segs[-1]`` is the terminal
            ``ACT_LINE_OVERSHOOT`` roll and ``segs[-2]`` its paired follow leg
            (guaranteed by the sim's own follow-then-roll emission pairing —
            the only way ``segs[-1].approach_predicted_cm`` is set at all).
        :param speed: the run's ``(cruise, start, end_seed, profiled)`` seed.
        :param predicted_cm: ``segs[-1].approach_predicted_cm`` — the follow
            leg's own chord distance.
        :return: ``True`` iff the approach-ramp render fired (keyframes
            already appended); ``False`` to fall through to the ordinary
            whole-run render.
        """
        overshoot_seg = segs[-1]
        follow_seg = segs[-2]
        cfg = default_config()
        schedule = _resolve_line_approach_schedule(predicted_cm, cfg)
        if schedule is None:
            return False

        cruise, start, _end_seed, profiled = speed
        prefix_segs = segs[:-2]
        prefix_total_cm = sum(s.dist_cm for s in prefix_segs)

        # -- Phase A: prefix (if any) + ramp-up + cruise, clipped to
        #    schedule.ramp_start_cm within the follow leg (the CRUISE -> RAMP
        #    handoff). end_pwm == cruise, so this sub-profile carries no decel
        #    ramp of its own.
        ramp_start_local = schedule.ramp_start_cm
        clip_frac = ramp_start_local / predicted_cm
        clip_pose = _lerp_pose(follow_seg.start_pose, follow_seg.end_pose, clip_frac)
        profile_a = resolve_drive_profile(
            prefix_total_cm + ramp_start_local,
            cruise_pwm=cruise,
            start_pwm=start,
            end_pwm=cruise,
            profiled=profiled,
        )
        boundaries_a = [(s.start_pose, s.end_pose, s.dist_cm) for s in prefix_segs]
        boundaries_a.append((follow_seg.start_pose, clip_pose, ramp_start_local))
        for sub in sample_drive_run(profile_a, boundaries_a, keyframes[-1]["t"]):
            _extend(sub)

        # -- Phase B: dense RAMP + FLOOR_HOLD deceleration off approach_speed_at.
        _extend(
            _sample_approach_ramp_keyframes(
                follow_seg, schedule, predicted_cm, keyframes[-1]["t"], cfg
            )
        )

        # -- Phase C: the overshoot roll's own floor -> 0 crawl-then-brake (AC3
        #    handoff: seeded AT the floor magnitude, never re-accelerating to
        #    cruise). The commanded magnitude decreases from the floor toward
        #    0, but DriveProfile's decel-region display convention ALREADY
        #    floors velocity at the KINETIC floor (PWM_MOVING_MIN) — and this
        #    leg starts AT that floor by construction — so the whole leg
        #    renders as a constant-velocity floor-crawl, ending with an
        #    (implicit, zero-duration) terminal brake at the snapshot pose.
        #    Deliberately NOT routed through resolve_drive_profile: that
        #    kernel's CRUISE-phase display floor is the STATIC breakaway floor
        #    (PWM_MIN_MAGNITUDE), which sits ABOVE the kinetic floor by design
        #    — feeding it cruise_pwm == floor_mag would inflate the displayed
        #    speed above the true floor speed for the leg's leading portion
        #    (caught by the TC2 collinear-run-grouper monotonicity regression).
        floor_mag = int(cfg.pwm_moving_min)
        crawl_speed = SCALE_CM_S_PER_PWM * floor_mag
        dt = overshoot_seg.dist_cm / crawl_speed if crawl_speed > 0.0 else 0.0
        x1, y1, h1 = overshoot_seg.end_pose
        _extend([{"t": keyframes[-1]["t"] + dt, "x": x1, "y": y1, "h": h1}])
        return True

    def append_run_segment(
        x_new: float,
        y_new: float,
        h_new: float,
        speed: Tuple[int, int, int, bool],
        braked: bool,
        chain_speeds: Optional[Tuple[int, int]] = None,
        rear_led: bool = False,
        approach_predicted_cm: Optional[float] = None,
    ) -> None:
        """Buffer one geometry drive primitive into the open collinear run.

        The segment spans from the CURRENT run tail pose (the last buffered
        segment's end, or the last emitted keyframe when the buffer is empty) to
        ``(x_new, y_new, h_new)`` — the primitive's sim-snapshot pose. ``speed``
        seeds the run's ``(cruise, start, end, profiled)`` on the FIRST segment;
        subsequent segments in a line/projection run share that cruise, so the
        seed is not overwritten. A ``braked`` segment is the run's terminal
        stop, so the run is flushed immediately after buffering it.

        ``chain_speeds`` — when set, the RECORDED ``(start_pwm, end_pwm)`` for a
        BLIND-chain hop (ADR-059). It rides ONTO the segment so ``flush_run``
        builds the whole-run trapezoid from the recorded endpoints instead of the
        ``run_speed`` seed; a line/projection segment leaves it ``None``.

        ``rear_led`` — ``True`` for a REAR-led braked distance-gated stop
        (:data:`_REAR_LED_STOP_TAGS`); it rides onto the segment so
        ``flush_run`` applies the AC8 LineFollower window-fit when this segment
        terminates the run.

        ``approach_predicted_cm`` — set ONLY by the caller for a terminating
        (non-pass-through) ``ACT_LINE_OVERSHOOT`` roll (T10, ADR-058 display
        layer), to the immediately-preceding follow leg's own chord distance.
        It rides onto the segment so ``flush_run`` renders the anticipatory
        approach-ramp deceleration instead of the plain whole-run trapezoid.

        A zero-length segment (the primitive did not translate) is dropped — it
        contributes no distance and does not open/extend a run — but if it is
        flagged ``braked`` (a distance-gated stop that happened to net zero
        advance) the open run is still flushed so its terminal stop is not lost.
        """
        nonlocal run_speed
        # The run tail is the previous segment's end, or the last emitted
        # keyframe when the buffer is empty.
        prev_pose = run_tail_pose()

        dist = hypot(x_new - prev_pose[0], y_new - prev_pose[1])
        if dist <= _TRANSLATION_EPS_CM:
            # No advance: nothing to buffer. A braked zero-advance still closes an
            # open run (its terminal stop already rendered on the prior segment).
            if braked:
                flush_run()
            return

        if not run_segments:
            run_speed = speed
        chain_start = chain_speeds[0] if chain_speeds is not None else None
        chain_end = chain_speeds[1] if chain_speeds is not None else None
        run_segments.append(
            _RunSegment(
                start_pose=prev_pose,
                end_pose=(x_new, y_new, h_new),
                dist_cm=dist,
                braked=braked,
                chain_start_pwm=chain_start,
                chain_end_pwm=chain_end,
                rear_led=rear_led,
                approach_predicted_cm=approach_predicted_cm,
            )
        )
        if braked:
            flush_run()

    def run_tail_pose() -> Tuple[float, float, float]:
        """The current run-tail pose: the buffered run's end, else last keyframe.

        While a collinear run is being buffered, the emitted ``keyframes[-1]`` is
        STALE (it is the pre-run keyframe — nothing has been appended for the
        buffered segments yet). Any per-primitive classification that needs the
        chassis's CURRENT logical pose (the translation / heading delta a new
        primitive makes) must read it from the run tail, not ``keyframes[-1]``,
        so a zero-motion record mid-run is not misread as a giant advance.
        """
        if run_segments:
            return run_segments[-1].end_pose
        last = keyframes[-1]
        return (last["x"], last["y"], last["h"])

    def append_turn_keyframes(
        x_new: float,
        y_new: float,
        h_new: float,
        signed_turn_deg: Optional[float] = None,
    ) -> None:
        """Densify an in-place turn to ``(x_new, y_new, h_new)`` and append it.

        Times the pure rotation with the turn profile (constant-rate at the
        snapped floor for the default Config). The final sub-keyframe is pinned
        to the destination pose byte-exact. A zero-magnitude turn appends
        nothing. ``signed_turn_deg`` (a raw ``turnDeg`` argument, when known) is
        forwarded only to disambiguate the exact ±180 sweep direction.
        """
        _extend(
            _densify_turn(
                keyframes[-1], (x_new, y_new, h_new), signed_turn_deg=signed_turn_deg
            )
        )

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
        distance_cm: float,
        speed: Tuple[int, int, int, bool],
        rear_window_cm: Optional[float] = None,
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
        ``rear_window_cm`` (a recorded rear-led braked ``stopAtLine`` only —
        :func:`_recorded_drive_rear_window`) threads the AC8 LineFollower
        window-fit to the rendered profile.
        """
        cruise = speed[0]
        # Direction comes from exactly ONE channel — the resolved cruise sign
        # (which already folds setPower's sign and the reverse flag). Take the
        # MAGNITUDE from abs(distance_cm) so a negative cm literal cannot
        # double-negate a reverse move: driveDist(-10).setReverse(true) resolves
        # cruise < 0 and would otherwise render -(-10) = +10 cm FORWARD. driveDist
        # rejects a negative cm at the firmware boundary, so this only hardens a
        # malformed / hand-authored trace.
        signed_cm = abs(distance_cm) if cruise >= 0 else -abs(distance_cm)
        sim.operator_move(signed_cm, turn_deg=0.0)
        append_drive_keyframes(
            chassis.x_cm,
            chassis.y_cm,
            chassis.heading_deg,
            speed,
            rear_window_cm=rear_window_cm,
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
        # Forward the SIGNED commanded rotation so an exact turnDeg(±180) animates
        # the direction the robot turned (start/end headings alone are ambiguous
        # at ±180). Non-boundary magnitudes ignore it and keep the shortest arc.
        append_turn_keyframes(
            chassis.x_cm,
            chassis.y_cm,
            chassis.heading_deg,
            signed_turn_deg=turn_deg,
        )

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

            # Operator offset accumulated onto ``from_node`` by any preceding raw
            # hand-drive, captured BEFORE ``step_through`` re-anchors (clears) it.
            # Used to surface a sub-threshold off-node excursion this goto silently
            # corrects — see the post-loop ``drift_warn`` below.
            pre_goto_offset_cm = sim.localization.get_offset_from_node().magnitude_cm()

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
                    # dwell-1 (Settle-fire matrix), deferred per-iteration flag:
                    # a settle-eligible braked arrival OWES a post-stop dwell,
                    # but the ``append_dwell_keyframe`` must run AFTER the
                    # ``pending_pivot_land`` landing-correction block below — that
                    # block rewrites ``keyframes[-1]`` (the ARRIVAL keyframe) to
                    # the trim-corrected pose, so an inline dwell here would take
                    # the correction on the dwell keyframe and leave the arrival
                    # keyframe at the over-shot dead-reckoned pose (C1). The flag
                    # is set at the braked ``append_run_segment`` sites and
                    # consumed once, at the very end of this iteration, so it can
                    # never leak a dwell into a later primitive. This mirrors the
                    # deferred dwell-2 (``pending_turn_dwell``) already above.
                    settle_dwell_pending = False
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
                            # A pivot is a run boundary: close any open collinear
                            # run FIRST so the arc starts from the run's last
                            # emitted keyframe (its snapshot pose), not a
                            # still-buffered mid-run pose.
                            flush_run()
                            # Parse the track width ONLY on the pivot path.
                            if track_cm is None:
                                track_cm = parse_track_cm_from_config()
                            theta_deg = float(primitive[1])
                            start_kf = keyframes[-1]
                            # REAR pivots (#54 / ADR-073): consume the sim's
                            # DERIVED taken-pivot surface -- the direction
                            # force-SAME committed the pivot to and the PLANTED
                            # (post-flip) wheel. The direction is NOT recorded in
                            # the command stream (recorder / trace_loader are
                            # untouched, ADR-054); it is derived by the sim, so we
                            # read it here rather than re-deriving. The departure
                            # hop is where the pivot is TAKEN, so ``_taken_pivots``
                            # is already populated by the time this pre-edge
                            # ``ACT_TURN_REL`` is yielded. A miss (defensive only:
                            # the pivot was not taken by the walk) falls back to
                            # the direction-agnostic record's FORWARD wheel + a
                            # FRONT direction -- byte-identical to the pre-#54
                            # FRONT-only path.
                            taken = sim.taken_pivot(prev_node, cur_node, w)
                            if taken is not None:
                                planted_wheel = taken.wheel
                                pivot_dir = taken.direction
                            else:
                                planted_wheel = record.wheel
                                pivot_dir = Direction.FRONT
                            is_rear = pivot_dir == Direction.REAR
                            # The planted-wheel side comes from the PLANTED
                            # (post-flip) wheel; ``start_kf["h"]`` already carries
                            # the +180 body-heading for a REAR approach (the sim
                            # records the reverse-facing body pose), so the L<->R
                            # flip + the +180 keep the wheel on the SAME world /
                            # travel side as the forward pivot (Data / geometry
                            # model, ADR-073).
                            wheel_sign = (
                                1 if planted_wheel == TurnMode.PIVOT_LEFT else -1
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
                            # ``wheel`` is the PLANTED (post-flip) wheel and
                            # ``direction`` is the sim-derived pivot direction;
                            # both live INSIDE the non-empty-gated ``pivots`` list,
                            # so ``schema_version`` stays 1 and a pivot-free
                            # trace's Track JSON is byte-identical (ADR-073 / AC9).
                            pivots.append(
                                {
                                    "t0": start_kf["t"],
                                    "t1": t,
                                    "cx": wx,
                                    "cy": wy,
                                    "theta_deg": theta_deg,
                                    "wheel": (
                                        "left"
                                        if planted_wheel == TurnMode.PIVOT_LEFT
                                        else "right"
                                    ),
                                    "direction": "rear" if is_rear else "front",
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
                            # residual over-travel lies along the TRAVEL-frame
                            # departure ray. For a FRONT pivot the arc-end BODY
                            # heading IS the travel direction (the pre-edge turn
                            # faces the edge exactly). For a REAR pivot the body
                            # faces OPPOSITE travel (rear sensor leads), so the
                            # arc-end body heading points backward along the edge
                            # -- flip it 180 deg to recover the travel ray, or the
                            # correction would pull the landing the WRONG way
                            # (ADR-073 / spec item 4).
                            depart_rad = radians(
                                arc[-1]["h"] + (180.0 if is_rear else 0.0)
                            )
                            pending_pivot_land = (
                                w,
                                record.trim_cm,
                                cos(depart_rad),
                                sin(depart_rad),
                            )
                            # dwell-2 (Settle-fire matrix): a pivot IS a cold
                            # turn on a normal hop (the return prologue's own
                            # turn never reaches this branch — ``find_pivot_turn``
                            # misses with ``prev_node=None`` at a goto's start,
                            # so it renders as the plain in-place turn above,
                            # not an arc). No ``_RETURN_DRIVE_TAGS`` check
                            # needed here; the suppression is at the advance
                            # site regardless.
                            pending_turn_dwell = True

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
                        #
                        # A drive advance is BUFFERED into the open collinear run
                        # (T3): a pass tag / follow leg CONTINUES the run, a braked
                        # distance-gated stop ENDS it. The whole run is then
                        # emitted as ONE trapezoid, so there is no intra-run dip.
                        # A zero-motion record is transparent to the run: a
                        # heading-only re-anchor (``ACT_HEADING_RESET``) has a
                        # zero-magnitude turn that emits nothing and does NOT close
                        # the run; a REAL in-place turn (a genuine heading change)
                        # is a run boundary, so it flushes the run before its
                        # rotation renders.
                        # Classify against the RUN-TAIL pose (the buffered run's
                        # end, or the last emitted keyframe when idle) — NOT the
                        # stale ``keyframes[-1]`` — so a zero-motion record that
                        # arrives mid-run (its snapshot equals the run tail) is
                        # correctly seen as a no-op, not a giant advance back to
                        # the pre-run keyframe.
                        tail_x, tail_y, tail_h = run_tail_pose()
                        step_dist = hypot(snap.x_cm - tail_x, snap.y_cm - tail_y)
                        turn_mag = abs(wrap_pm180(snap.heading_deg - tail_h))
                        if step_dist <= _TRANSLATION_EPS_CM:
                            # Zero-motion primitive. A real (above-tolerance)
                            # heading change breaks the run (it is not collinear
                            # across the turn); a heading-reset OR a sub-tolerance
                            # alignment turn is TRANSPARENT. Gate on the SAME
                            # NAV_TURN_TOLERANCE_DEG the sim uses to decide a
                            # pass-through (``_should_pass_through`` in
                            # runtime.py): the sim emits a tiny ``turn_relative``
                            # (a few hundredths of a degree) to align onto each
                            # next edge's exact heading between two NEAR-collinear
                            # pass-through edges — a real field's nodes are never
                            # perfectly axis-aligned. That turn is BELOW tolerance,
                            # so the sim already passed through (a ``line_pass``,
                            # NOT a braked stop); the grouper must agree and keep
                            # the run open, or the whole-run trapezoid splits into
                            # two ramps with a spurious dip at the pass-through node
                            # (the exact stop-at-every-node lie this grouper
                            # repairs). A hard 1e-9 threshold here re-introduced it
                            # on the real field, which the perfectly-collinear toy
                            # fixtures could not surface.
                            if turn_mag > NAV_TURN_TOLERANCE_DEG:
                                # A real in-place turn: flush the open run FIRST so
                                # ``append_turn_keyframes`` renders the rotation
                                # from the run's last emitted keyframe, then emit
                                # the turn.
                                flush_run()
                                append_turn_keyframes(
                                    snap.x_cm, snap.y_cm, snap.heading_deg
                                )
                                # dwell-2 (Settle-fire matrix): a cold pre-edge
                                # turn on a normal hop settles before the move
                                # that follows. Deferred: the return-prologue's
                                # own turn also renders here (a non-pivot plain
                                # turn), so the suppression is applied at the
                                # NEXT advance site (``_RETURN_DRIVE_TAGS``),
                                # not here.
                                pending_turn_dwell = True
                            # else: a heading-reset / no-op OR a sub-tolerance
                            # pass-through alignment turn is fully transparent —
                            # append NOTHING and leave the run open. Rendering a
                            # turn here would interpolate position from the STALE
                            # (pre-run) ``keyframes[-1]`` and forge a spurious
                            # keyframe at the run-tail pose; the next follow leg's
                            # snapshot pins the corrected heading byte-exact anyway.
                        else:
                            # dwell-2 (Settle-fire matrix): consume the deferred
                            # cold-turn flag on this, the NEXT real drive advance
                            # after a rendered turn (the "turn -> move" seam).
                            # Suppressed when this advance IS the return-to-node
                            # prologue's own drive (``_RETURN_DRIVE_TAGS`` —
                            # matrix row "Return-prologue pre-edge turn" ->
                            # settles neither the turn nor the move). Cleared
                            # unconditionally on this advance regardless of
                            # whether the dwell actually fires, matching the
                            # firmware's one-shot ``SettleResume::EDGE_AFTER_TURN``
                            # (a zero-motion heading reset above does NOT reach
                            # this branch, so it cannot consume/clear the flag).
                            if pending_turn_dwell:
                                if hop_settle_s > 0 and tag not in _RETURN_DRIVE_TAGS:
                                    append_dwell_keyframe(hop_settle_s)
                                pending_turn_dwell = False
                            # SQUARE-then-advance: a primitive that BOTH rotates
                            # and translates is two physical maneuvers, and the
                            # firmware always runs them in that order — the wall
                            # interlude's `Drive.wallStop(...).setHeading(facing)`
                            # turns in place to the wall normal (its internal
                            # SQUARE phase) and only then presses the rear
                            # straight into the wall (PathTracker.cpp,
                            # `_beginWallStop`). The sim collapses both into ONE
                            # `ACT_WALL_STOP` record (`_wall_reset_interlude`
                            # phase 1 mutates heading AND the on-axis coordinate
                            # before its single yield), so without this split the
                            # whole rotation is handed to `_densify_drive`, which
                            # lerps heading by traveled-distance fraction — the
                            # player then renders the robot pirouetting WHILE it
                            # crosses the standoff, a maneuver the robot never
                            # performs. Emit the rotation as its own in-place turn
                            # at the run-tail position first (turn-profile timed),
                            # so the advance below runs at a FIXED heading.
                            #
                            # Gated on the SAME NAV_TURN_TOLERANCE_DEG as the
                            # zero-motion arm above: a sub-tolerance pass-through
                            # alignment stays transparent (the advance's own
                            # snapshot still pins the corrected heading byte-exact),
                            # so every collinear line / blind / projection run
                            # renders byte-unchanged. `pending_turn_dwell` is
                            # deliberately NOT re-armed — the square is INTERNAL
                            # to one firmware move, which never settles between
                            # squaring and pushing.
                            if turn_mag > NAV_TURN_TOLERANCE_DEG:
                                # Close the open run first so the rotation renders
                                # from the run's last EMITTED keyframe; after the
                                # flush that keyframe is byte-exact the run tail
                                # (TC7), so the turn is genuinely in place.
                                flush_run()
                                append_turn_keyframes(
                                    tail_x, tail_y, snap.heading_deg
                                )
                            # A CHAINED blind hop (ADR-059) rides its resolved
                            # per-hop (start_pwm, end_pwm, chain) on the primitive
                            # tuple — CTPS CONSUMES those recorded speeds for the
                            # whole-run trapezoid, never re-deriving a chain
                            # decision (AC11). A chained mid-hop passes the node
                            # WITHOUT braking (end != 0); the run's FINAL chained
                            # hop is the ordinary braked stop (end == 0), which
                            # ends the run. A non-chain hop leaves ``chain_speeds``
                            # None and takes the ordinary per-primitive-kind path
                            # (AC12: a single hop is byte-unchanged).
                            chain_suffix = _blind_chain_suffix(primitive)
                            if chain_suffix is not None:
                                start_pwm, end_pwm, _chain = chain_suffix
                                braked = end_pwm == 0
                                append_run_segment(
                                    snap.x_cm,
                                    snap.y_cm,
                                    snap.heading_deg,
                                    _primitive_drive_speed(tag, pipeline_cfg),
                                    braked,
                                    chain_speeds=(start_pwm, end_pwm),
                                )
                                # dwell-1 (Settle-fire matrix): a blind-chain
                                # hop only settles on its FINAL (braked) hop —
                                # an intermediate chained hop nets end_pwm != 0
                                # (not braked) and never reaches here armed.
                                # Suppressed at a wall-reset / return-prologue
                                # tag, or an arrival at a RESET node (the wall
                                # interlude owns that boundary — mirrors the
                                # firmware ``getNodeType(arrived) != RESET``
                                # gate in ``_enterSettle``). DEFERRED: the dwell
                                # keyframe is appended at the end of this
                                # iteration, AFTER the ``pending_pivot_land``
                                # landing-correction runs, so the correction
                                # patches the ARRIVAL keyframe and the dwell then
                                # holds the corrected pose (C1).
                                if (
                                    hop_settle_s > 0
                                    and braked
                                    and tag not in _WALL_RESET_TAGS
                                    and tag not in _RETURN_DRIVE_TAGS
                                    and not sim.localization.is_reset_node(node_now)
                                ):
                                    settle_dwell_pending = True
                            else:
                                # A braked stop ends the run; a pass tag or a
                                # mid-run follow leg (``ACT_FOLLOW_INT``) continues
                                # it. A REAR-led stop tag additionally marks the
                                # segment so ``flush_run`` fits the run's shed
                                # into the rear overshoot window (AC8).
                                braked = (
                                    tag not in _PASS_TAGS
                                    and tag not in _MID_RUN_CONTRIB_TAGS
                                )
                                # ADR-064 per-edge blind speed: a LONE (non-chain)
                                # blind hop or a seek carries NO recorded per-hop
                                # speed — the sim leaves the cold-start tuple
                                # pristine (AC12); only chained / pass-through hops
                                # ride a (start_pwm, end_pwm) suffix (handled in the
                                # chain branch above via ``_resolve_blind_run_profile``).
                                # So resolve THIS hop's edge SpeedTier from the field
                                # graph the CTPS already built — the SAME pure per-edge
                                # read the firmware dispatch runs (``_resolveSpeed(edge)``),
                                # against the FIELD-SOURCED ``pipeline_cfg`` (so a
                                # mission-wide ``PIPELINE_SPEED`` inherits identically to
                                # the sim / robot) — NOT a chain re-derivation — so a
                                # ``speed: FAST`` lone blind hop / seek renders at
                                # SPEED_FAST (90) instead of the hardcoded SPEED_NORMAL.
                                # Scoped to the genuine per-edge nav-hop tags
                                # (``_PER_EDGE_SPEED_TAGS``) — the line-overshoot / return
                                # / wall families are excluded so they keep their existing
                                # cruise. ``has_edge`` guards a hop whose (prev, cur) is
                                # not a graph edge. NORMAL / absent / non-edge → 70
                                # (byte-identical to the pre-ADR-064 render, so all-NORMAL
                                # goldens are unchanged).
                                hop_speed_tier = None
                                if (
                                    tag in _PER_EDGE_SPEED_TAGS
                                    and prev_node is not None
                                    and graph.has_edge(prev_node, cur_node)
                                ):
                                    hop_speed_tier = resolve_speed(
                                        graph.edges[prev_node, cur_node], pipeline_cfg
                                    )
                                # T10 (ADR-058 display layer): a terminating
                                # (non-pass-through) LINE hop's roll-onto-node
                                # is ALWAYS the primitive immediately following
                                # its own paired follow leg (ACT_FOLLOW_INT /
                                # ACT_LINE_REVERSE), which the run buffer holds
                                # as its own last segment right now (BEFORE this
                                # overshoot segment is appended below) — so its
                                # own chord distance IS the predicted
                                # sensor-crossing distance the schedule needs
                                # (already axle-offset-adjusted by the sim's own
                                # follow-leg geometry; see the section banner
                                # above _resolve_line_approach_schedule).
                                # ``ACT_LINE_PASS`` (a collinear pass-through)
                                # is deliberately excluded (AC5): it never
                                # reaches here with ``braked``.
                                approach_predicted_cm = (
                                    run_segments[-1].dist_cm
                                    if tag == ACT_LINE_OVERSHOOT and run_segments
                                    else None
                                )
                                append_run_segment(
                                    snap.x_cm,
                                    snap.y_cm,
                                    snap.heading_deg,
                                    _primitive_drive_speed(tag, pipeline_cfg, hop_speed_tier),
                                    braked,
                                    rear_led=tag in _REAR_LED_STOP_TAGS,
                                    approach_predicted_cm=approach_predicted_cm,
                                )
                                # dwell-1 (Settle-fire matrix): post-stop, before
                                # the next maneuver — fires on every settle-
                                # eligible braked arrival (LINE_OVERSHOOT /
                                # PROJECTION_FOLLOW stop / STOP_AT_LINE / a
                                # non-chain BLIND_FORWARD), which also covers the
                                # goal's final braked hop (its internal
                                # ``flush_run`` IS this call site — the goal-end
                                # ``flush_run(strict=True)`` below operates on an
                                # already-empty buffer for a healthy goto, so it
                                # must NOT also dwell, or the goal would double-
                                # dwell). Suppressed at a wall-reset / return-
                                # prologue tag, or an arrival at a RESET node.
                                # DEFERRED: the dwell keyframe is appended at the
                                # end of this iteration, AFTER the
                                # ``pending_pivot_land`` landing-correction runs,
                                # so on a pivot-departure landing the correction
                                # patches the ARRIVAL keyframe FIRST and the dwell
                                # then holds the corrected (settled) pose (C1). A
                                # plain braked arrival (no pivot landing) still
                                # gets its dwell immediately after the arrival
                                # keyframe — unchanged.
                                if (
                                    hop_settle_s > 0
                                    and braked
                                    and tag not in _WALL_RESET_TAGS
                                    and tag not in _RETURN_DRIVE_TAGS
                                    and not sim.localization.is_reset_node(node_now)
                                ):
                                    settle_dwell_pending = True
                        if (
                            pending_pivot_land is not None
                            and node_now == pending_pivot_land[0]
                        ):
                            # A pivot-departure landing correction rewrites this
                            # primitive's landing keyframe, so the run must be
                            # closed first (a pass-through landing would otherwise
                            # still be buffered). Flushing here with the run's
                            # accumulated cruise is safe: the departure edge is a
                            # single hop, so the run holds exactly this one
                            # segment.
                            flush_run()
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
                        # dwell-1 (Settle-fire matrix), deferred append: now that
                        # the ``pending_pivot_land`` landing-correction has run
                        # (patching the ARRIVAL keyframe ``keyframes[-1]`` to the
                        # trim-corrected pose on a pivot-departure landing), hold
                        # the settle at that corrected pose. For a plain braked
                        # arrival (no pivot landing) this fires immediately after
                        # the arrival keyframe, exactly as an inline append would
                        # (behavior unchanged). The flag is consumed here on the
                        # SAME iteration as the braked arrival, so it never leaks
                        # into a later primitive.
                        if settle_dwell_pending:
                            append_dwell_keyframe(hop_settle_s)
                            settle_dwell_pending = False
            except UnreachableGoalError:
                # Emit any buffered collinear run of the REACHABLE prefix before
                # the fault so its keyframes are on the timeline (a partial run
                # is a genuine braked stop at the dead end, NOT a malformed run).
                # Non-strict (the default) so the unbraked reachable prefix does
                # not trip the goto-end pass-without-braked-stop fallback warning.
                flush_run()
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
                # Non-strict (the default): a partial reachable prefix is
                # legitimate, not a malformed run.
                flush_run()
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

            # The goto walk completed: flush the final open collinear run (its
            # last hop is the goal's braked arrival, so any healthy goto leaves
            # no run dangling across the goto boundary). This is also the
            # boundary-safety net — no run ever spans two gotos. STRICT here: a
            # healthy goto always ends braked, so a GROUPED unbraked run surviving
            # to the goto boundary is a malformed trace (a pass tag with no
            # following braked stop) → per-primitive fallback + UserWarning
            # (AC11 / TC11). Interior boundary flushes above stay non-strict —
            # an unbraked pass run ending at a turn/pivot is legitimate.
            #
            # NO dwell-1 insertion here: for a healthy goto this flush operates
            # on an already-empty ``run_segments`` buffer (the goal's last hop
            # was itself a braked ``append_run_segment`` call above, whose own
            # internal ``flush_run()`` already fired dwell-1). Adding a second
            # dwell here would double the goal-arrival settle (the firmware
            # emits exactly ONE SETTLE per braked arrival). The malformed-trace
            # fallback path warns and returns without reaching a dwell decision
            # here either way.
            flush_run(strict=True)

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
            # Sub-threshold drift indicator: a preceding raw off-node hand-drive
            # whose accumulated offset was BELOW RETURN_TRIGGER_THRESHOLD_CM (3 cm)
            # gets no physical return prologue (``_evaluate_return_gate`` returns
            # NONE) and thus no ``return_to_node`` marker, yet this goto's arrival
            # re-anchor still cleared the offset before the trace-end residual check
            # runs. The firmware does NOT physically return for a sub-3 cm offset
            # either, so the excursion is a real, uncorrected error — surface it as a
            # POINT ``drift_warn`` at the goto start (mirrors the trace-end residual
            # label) instead of letting it vanish. The >= 3 cm case already renders +
            # marks ``return_to_node`` (``return_marked``), so skip it there to avoid
            # a double mark; the ``operator_move`` integrate deadband keeps a
            # sub-0.2 cm nudge at offset 0, so it never trips this.
            if not return_marked and pre_goto_offset_cm > _RESIDUAL_DRIFT_EPS_CM:
                events.append(
                    {
                        "t": t_start,
                        "type": "drift_warn",
                        "label": (
                            f"chassis {pre_goto_offset_cm:.1f} cm off node {from_node}"
                        ),
                    }
                )

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
                    # cruise), then densify the advance off that profile. A
                    # rear-led braked stopAtLine additionally carries the REAR
                    # overshoot window so the render fits its cruise -> floor
                    # shed into it (AC8); every other recorded move keeps None.
                    speed = resolve_effective_speed(cmd, method)
                    rear_window = _recorded_drive_rear_window(cmd, speed)
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
                        operator_advance_keyframe(
                            float(cmd.cm), speed, rear_window_cm=rear_window
                        )
                    else:
                        # Plain advance along the CURRENT facing (non-gyro, or no
                        # heading, or lineFollowDist).
                        operator_advance_keyframe(
                            float(cmd.cm), speed, rear_window_cm=rear_window
                        )
            elif method == "turnDeg":
                # Raw relative turn: the signed delta IS the ``turnDeg`` argument.
                if cmd.deg is None:
                    # Legacy / partial trace (a deg-less turnDeg, or a deg dropped
                    # by the loader's numeric guard): no rotation to replay.
                    # Narrate only — never ``float(None)``, which raises TypeError
                    # and would abort the ENTIRE build (no player HTML). Mirrors
                    # the cm-less forward-geometry guard above.
                    pass
                else:
                    operator_turn_keyframe(float(cmd.deg))
            elif method == "turnToYaw":
                # Raw absolute turn: route the shortest-arc delta to the target
                # yaw through ``operator_move`` (``operator_move`` turns relative),
                # so the render and the integrated yaw offset both land on the
                # requested heading.
                if cmd.deg is None:
                    # deg-less / dropped absolute turn: narrate only, no rotation
                    # (same defense as the cm-less and turnDeg guards).
                    pass
                else:
                    operator_turn_keyframe(
                        wrap_pm180(float(cmd.deg) - chassis.heading_deg)
                    )
            # else: stop / unknown -> no geometry change, nothing to integrate.

            # Visualization-only lateral hug-side nudge (issue #156, ADR-054
            # chassis-nudge precedent): a biased ``lineFollowDist`` carries the
            # signed LineFollower offset (``follow_offset``, centroid-index units)
            # recorded by the ctps_mocks setLineFollowOffset seam. Register the
            # [event_t, t] follow span with an on-screen lateral cm mapped through
            # the CTPS-layer display constant so the renderer draws the chassis
            # marker biased to that side for the whole leg. This is a DRAW-time
            # overlay ONLY — the keyframes above already integrated on the shared
            # sim (centerline), and nothing here writes an x/y back to the sim /
            # LocalizationSim (the pivot-arc invariant). Guarded on a real advance
            # (``t > event_t``) and a set, non-zero offset, so a centered / MIDDLE
            # follow — or any non-lineFollowDist move — appends nothing and stays
            # byte-identical.
            if (
                method == "lineFollowDist"
                and cmd.follow_offset is not None
                and cmd.follow_offset != 0.0
                and t > event_t
            ):
                lateral_offsets.append(
                    {
                        "t0": event_t,
                        "t1": t,
                        "offset_cm": (
                            LINE_FOLLOW_OFFSET_DISPLAY_CM * float(cmd.follow_offset)
                        ),
                    }
                )

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

        elif cmd.op == "read_mosaic":
            # Non-geometry event (CTPS randomized-vision plan, T9): a
            # `readMosaic()` call never moves the chassis, so — like `stop` /
            # `stopAtLine` before it — it dwells nothing and appends only an
            # event, no keyframe. Carries `success` (the read outcome; `None`
            # if the loader could not validate it) alongside the unchanged
            # `label` string. Deliberately NOT registered as a new timeline
            # lane: the dormant case (no vision-consuming mission yet) gets an
            # event only, so it renders wherever the generic event/log views
            # already read `track.events` — never touching `PLAYER_META`'s
            # frozen `lanes` list or `timeline.js`'s lane registry (T10/T11
            # own any future lane wiring, not this builder).
            events.append(
                {
                    "t": t,
                    "type": "read_mosaic",
                    "label": f"read_mosaic success={cmd.success}",
                    "success": cmd.success,
                }
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
        lateral_offsets=lateral_offsets,
        mosaic=trace.mosaic,
    )
