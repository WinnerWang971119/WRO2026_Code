"""PathTracker FSM mirror in pure Python (verification harness).

Mirrors the C++ PathTracker FSM closely enough that its dispatched-primitive
sequence matches the runtime PathTracker for the same NEXT_HOP-walked path.
Not tick-by-tick faithful -- each Drive/LineFollower dispatch + completion is
collapsed into a single atomic transition. That is enough for end-to-end
planning + heading-propagation verification on any graph without hardware.

Edge dispatch (mirrors :func:`tools.path_planner.compiler.compile_path` plus
the runtime-only LINE_OVERSHOOT FSM transition). Projection is no longer an
edge type -- it is the *destination node* type (issue #64): the per-hop
primitive falls out of ``(edge_type, destination_node_type)`` via
:func:`tools.path_planner.graph_builder.is_projection_node`, a purely LOCAL
lookup on the destination node (no goal-relative ``NEXT_HOP`` peek):

* :data:`EDGE_LINE` **into a junction (line) node** -- emit
  ``(ACT_FOLLOW_INT, None)`` to walk to the intersection marker, **then**
  emit a runtime-only ``(ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)``
  step that advances the chassis by the line-sensor-to-axle offset so the
  wheel axle (not the sensor) ends up on the node. Localization then
  advances via :meth:`LocalizationSim.on_intersection`.
* :data:`EDGE_LINE` **into a projection node** -- emit
  ``(ACT_FOLLOW_DIST, distance_cm)`` (line-follow stopped by encoder
  distance, no intersection marker). Localization advances via
  :meth:`LocalizationSim.on_dead_reckoned_advance` (NOT ``on_intersection``
  -- the chassis is dead-reckoning into the node via the encoder gate, not
  snapping to a physical marker). No LINE_OVERSHOOT step follows -- the
  encoder gate already lands the chassis on the node. (Replaces the retired
  ``EDGE_PROJECTION`` dispatch; the motion profile now comes from the LINE
  edge's ``line_mode_override``, ``ProjectionMode`` having been deleted.)

  **Pass-through (T20, issue #64).** When this projection node B is a
  NON-goal node whose outbound B→C hop is a :data:`EDGE_LINE` edge into a
  LINE (junction) node AND A→B→C are collinear / same-direction (the SAME
  ADR-023 heading check, via :func:`_should_projection_pass_through`), the
  ``line(A)–projection(B)–line(C)`` triple collapses to ONE continuous A→C
  follow: the A→B distance gate stamps the dead-reckon advance MID-FOLLOW
  (no stop) and the follower stays live across B. The emitted tag becomes
  ``(ACT_FOLLOW_DIST_PASS, distance_cm)`` (the projection twin of
  ``ACT_LINE_PASS``) -- the chassis advance + re-anchor are IDENTICAL, only
  the tag differs. The TRIGGER is the A→B DISTANCE gate, not an intersection
  (B has no junction marker). Mirrors the firmware PROJECTION_FOLLOW
  ``_projPassThrough`` branch (T19). Any guard miss ⇒ the normal stop at B
  (plain ``ACT_FOLLOW_DIST``). Like ``ACT_LINE_PASS``, the pass-through is an
  FSM transition the firmware owns and is intentionally NOT compiled, so
  :mod:`tools.path_planner.compiler` stays pass-through-unaware on the
  line-proj-line collapse just as it is on the line-line-line one.
* :data:`EDGE_BLIND` -- both FRONT and REAR traversal are mode-sensitive
  (mirrors the blind branch of
  :func:`tools.path_planner.compiler.compile_path`). The per-hop decision
  runs through :func:`resolve_stop_at_line` (call flag ``AUTO`` — the
  offline streams have no call-site verb, matching the firmware's per-edge
  AUTO default). The resolver walks a precedence ladder (call-site verb →
  per-edge ``stop_at_line_override`` → legacy ``STOP_AT_LINE`` tag →
  auto-rule → never) and returns ``(seek, motion_mode)``. The
  auto-rule fires when ``field_data.BLIND_PROJECTION_AUTO_SEEK`` is set AND
  this BLIND edge's *destination node* is a projection node, so a
  blind→projection hop re-anchors on the projection node's own line first.
  The DECISION to seek now also fires on the auto-rule:

    * ``seek`` + FRONT -> ``(ACT_STOP_AT_LINE, distance_cm)``; re-anchors via
      :meth:`LocalizationSim.on_intersection` (the front board stops on the
      physical crossing line, so the chassis ends on the node). A
      BLIND→projection seek is a FULL-ADVANCE re-anchor: the seek-stop is
      treated as the projection node's mapped position.
    * ``seek`` + REAR -> ``(ACT_STOP_AT_LINE_REVERSE, target, distance_cm,
      blind_mode)``; re-anchors via :meth:`LocalizationSim.on_intersection`
      (the rear board detects the crossing). Requires a 2-board chassis —
      :func:`resolve_stop_at_line` returns ``seek=False`` for REAR on
      1-board, degrading to the dead-reckon branch below.
    * ``not seek`` + FRONT -> ``(ACT_FORWARD, distance_cm)``; advances via
      :meth:`LocalizationSim.on_dead_reckoned_advance`.
    * ``not seek`` + REAR -> ``(ACT_BLIND_REVERSE, target, distance_cm,
      blind_mode)``; advances via
      :meth:`LocalizationSim.on_dead_reckoned_advance`.

  The non-``STOP_AT_LINE`` (dead-reckon) FRONT/REAR streams stay
  byte-identical to the pre-resolver output
  (``test_property_random_walks`` / TC12). The two ``seek`` branches are the
  new behavior (mode-sensitive REAR via the rear board; AC1).

Public surface:

* :class:`LocalizationSim` -- mirror of ``LocalizationClass``.
* :class:`PathTrackerSim` -- the FSM. ``go_to(goal_name)`` walks NEXT_HOP.
* :func:`walk_next_hop` -- given a NextHopTable, returns the explicit
  node-name list from source to target.
* :func:`resolve_speed` / :func:`resolve_blind_mode` -- pure helpers that
  mirror the C++ ``_resolveSpeed`` / ``_resolveBlindMode(EdgeInfo)`` lookups:
  read the edge's optional override attribute (``speed`` / ``mode``) and fall
  back to :data:`PIPELINE_CONFIG` when the override is absent or set to its
  ``DEFAULT`` sentinel. The pre-turn pivot wheel is no
  longer an edge field -- it is resolved per turn from the
  :data:`PivotTurnRecord` table (ADR-036) latched in :meth:`_walk_hops`.
  (``resolve_projection_mode`` is gone -- ``ProjectionMode`` was deleted with
  the projection edge type; a LINE→projection follow uses ``resolve_speed``.)
* :class:`UnreachableGoalError` -- raised on ``next_hop[from][goal] is None``.
* :data:`ACT_LINE_OVERSHOOT` -- sim-only primitive tag for the
  LINE_OVERSHOOT FSM transition. NOT emitted by
  :func:`tools.path_planner.compiler.compile_path` (the C++ runtime owns
  this transition as an FSM state, not as a compiled primitive), so it
  is intentionally kept out of :mod:`tools.path_planner.compiler`.
* :data:`ACT_LINE_PASS` -- sim-only primitive tag emitted INSTEAD OF
  ``ACT_LINE_OVERSHOOT`` when the next-hop edge after an EDGE_LINE
  intersection is collinear with the inbound edge (heading delta
  <= :data:`NAV_TURN_TOLERANCE_DEG`) and the intersected node is not the
  final destination. Chassis advance is identical to LINE_OVERSHOOT;
  only the primitive name differs (for downstream verification +
  telemetry). See ADR-023 "Pass-through collinear intersections".

Operator offset + return-to-node prologue (issue #36, PR1b):

The generator action-stream above is stateless w.r.t. operator excursions --
it walks NEXT_HOP from the anchored node and never models a raw, non-``goTo``
operator move. PR1b adds a STATEFUL extension mirroring the two firmware
behaviors (``WRO2026/Localization.cpp`` + ``WRO2026/PathTracker.cpp``):

1. **Offset accumulation.** :class:`LocalizationSim` grows an
   :class:`Offset2D` ``(x, y, theta_deg)`` from raw operator moves while NOT
   navigating, mirroring ``LocalizationClass::_integrateOffset``. The gate
   (``set_nav_active`` pushed down from PathTracker, plus the ``zeroed``
   latch) and the two independent deadbands
   (:data:`OFFSET_TRANSLATION_DEADBAND_CM` /
   :data:`OFFSET_ROTATION_DEADBAND_DEG`) match the firmware. A node arrival
   (:meth:`LocalizationSim.on_intersection` /
   :meth:`~LocalizationSim.on_dead_reckoned_advance`) re-anchors by clearing
   the offset (kills the arrival-tick phantom).

2. **Return-to-node prologue.** :meth:`PathTrackerSim.go_to` first evaluates
   the return gate (:func:`resolve_return_action`, the Python mirror of
   ``ReturnToNodePolicy.h::resolveReturnAction``): an offset at or below the
   trigger is a true no-op (the stream stays byte-identical to today's),
   trigger < mag <= cap drives the inverse return onto the anchored node
   first, and mag > cap honors the off-map gate (FAULT raises
   :class:`OffMapFaultError`; BEST_EFFORT proceeds with a capped return).
   The return is dispatched on the inverse offset vector (heading
   ``atan2(-y, -x)`` in the world frame, distance ``sqrt(x^2+y^2)`` clamped
   to the cap), mirroring ``PathTracker::_dispatchReturnMove``:

   * **Line-backed anchor** (:data:`Routes::NodeType::LINE`): emit
     ``(ACT_RETURN_STOP_AT_LINE, node, dist, heading_world)`` -- the
     ``Drive.stopAtLine`` re-acquire. The sim models the SAME
     overshoot/axle-roll the LINE hops use: a follow phase advances
     ``dist - LINE_SENSOR_AXLE_OFFSET_CM`` then a roll advances the
     remaining offset so the wheel AXLE (not the sensor) lands ON the node.
     Re-anchors via :meth:`LocalizationSim.on_intersection` (the same node).
   * **Blind-only anchor** (:data:`Routes::NodeType::BLIND`): emit an
     ``(ACT_TURN_REL, delta)`` to face the return heading followed by
     ``(ACT_RETURN_FORWARD, node, dist)`` -- the dead-reckoned
     ``turnToYaw`` + ``driveDist``. Re-anchors via
     :meth:`LocalizationSim.clear_offset` (no node change -- we returned TO
     the node, not advanced).

   The return leaves the chassis axle-on-node with the offset cleared BEFORE
   the first planned hop departs, so a clean ``go_to`` after a return nets
   the original path's geometry with drift~0 (TC18a / AC17 pillar 1).

* :data:`ACT_RETURN_STOP_AT_LINE` -- sim-only primitive tag for the
  line-backed return seek. 4-tuple
  ``(ACT_RETURN_STOP_AT_LINE, node, dist_cm, heading_world_deg)``.
* :data:`ACT_RETURN_FORWARD` -- sim-only primitive tag for the blind-only
  dead-reckon return drive. 3-tuple
  ``(ACT_RETURN_FORWARD, node, dist_cm)``, preceded by the heading-align
  ``(ACT_TURN_REL, delta)``.

Wall-zero reset nodes (PR2, AC17 pillar 2):

On arrival at a node typed ``"reset"`` (carrying a sparse ``RESET_NODES``
entry ``{axis, wall_coord, facing}``), the firmware runs a two-phase interlude
BEFORE planning the next hop or declaring the goal reached
(``PathTracker::_maybeEnterReset`` -> WALL_STOP -> REANCHOR). The sim mirrors
it inline in :meth:`PathTrackerSim._walk_hops`, the moment a hop re-anchors
localization onto a reset node (:meth:`PathTrackerSim._dispatch_wall_reset`):

1. **wallStop.** Mirror of ``Drive.wallStop(...).setMode(GYRO).setHeading(facing)``.
   The deterministic sim has no real stall, so it models the successful press:
   square to ``facing`` (the wall normal), advance to ``wall_coord`` on ``axis``,
   then back off :data:`WALL_STOP_BACK_OFF_CM` in the reverse-of-push direction.
2. **reanchor.** Mirror of ``Localization.reanchorAtWall(axis, wall_coord,
   facing)``. Re-anchors the BELIEVED on-axis coordinate to the wheel AXLE --
   ``wall_coord - sign(gap)*WALL_CONTACT_OFFSET_CM`` where ``gap = wall_coord -
   node_coord`` -- on the NODE side of the wall surface (the chassis presses its
   REAR face flat against the wall, but the tracked reference is the axle, which
   sits :data:`WALL_CONTACT_OFFSET_CM` short of that contact face). Heading
   becomes ``facing``; any pre-existing drift on ``axis`` is corrected.
3. **wallDepart.** Mirror of the firmware's WALL_DEPART move
   (``Drive.driveDist(standoff - WALL_CONTACT_OFFSET_CM).setMode(GYRO)`` then
   ``clearOffset()``, where ``standoff = abs(wall_coord - node_coord)``). The
   chassis is squared to ``facing`` (front AWAY from the wall), so it drives
   FORWARD off the wall back onto the node. The net believed on-axis coordinate
   after the depart is exactly ``node_coord`` (the drift on ``axis`` is corrected
   TO the node, not to the wall), so the sim's chassis -- which IS the believed
   pose driving downstream NEXT_HOP geometry + parity -- is snapped to
   ``node_coord``.

The interlude fires whether the reset node is an intermediate hop (planning
continues) or the final destination. A clean run through a reset node leaves
the chassis pose EXACTLY on the node (drift on ``axis`` = 0 by construction).

* :data:`ACT_WALL_STOP` -- sim-only primitive tag for the wall push. 4-tuple
  ``(ACT_WALL_STOP, node, axis_idx, wall_coord)`` (``axis_idx`` 0 = x, 1 = y).
* :data:`ACT_REANCHOR` -- sim-only primitive tag for the wall re-anchor.
  4-tuple ``(ACT_REANCHOR, node, axis_idx, wall_coord)``.
* :data:`ACT_WALL_DEPART` -- sim-only primitive tag for the return-to-node
  depart. 4-tuple ``(ACT_WALL_DEPART, node, axis_idx, node_coord)`` -- the 4th
  element is the node's own on-axis coordinate the chassis returns to.
"""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Iterator, List, Optional, Tuple

import networkx as nx

from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_STOP_AT_LINE,
    ACT_TURN_REL,
    Primitive,
    normalize_turn,
)
from tools.path_planner.ctps.chain_speed import (
    ChainHopSpeed,
    resolve_chain_speeds,
)
from tools.path_planner import pipeline_config as _pipeline_config
from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    EDGE_LINE,
    is_projection_node,
)
from tools.path_planner.pipeline_config import (
    PIPELINE_CONFIG,
    BlindMode,
    DirectionMode,
    PipelineConfig,
    SpeedTier,
    StopAtLineMode,
)
from tools.path_planner.planner import (
    NextHopTable,
    PivotTurnRecord,
    build_pivot_turns,
)
from tools.path_planner.sim.chassis import VirtualChassis, _normalize

TURN_EPSILON_DEG: float = 1e-6

# MUST mirror ``Config::LINE_SENSOR_AXLE_OFFSET_CM`` in WRO2026/Config.h.
# Python cannot import C++ headers, so this constant is hand-mirrored. If
# the C++ side changes, update this value too -- the sim's LINE_OVERSHOOT
# advance must match the runtime's ``LineFollower.setDistanceTarget`` budget
# or the cross-check goal-position test will diverge.
#
# ADR-035 reconciliation: this was historically a placeholder ``5.0`` while
# the comment already claimed it mirrors ``Config::LINE_SENSOR_AXLE_OFFSET_CM``.
# Config.h carries the authoritative physical measurement FRONT = 12.02 cm,
# and the firmware (ADR-035, two-sided pivot compensation) reduces THIS offset
# by a line-incoming pivot trim ``delta`` (validated ``delta <= 12.02``). With
# the stale 5.0 a line-incoming pivot up to 7.68 cm would drive the reduced
# overshoot NEGATIVE in the sim and diverge from the firmware. The offset is
# therefore aligned to Config.h's real value. This preserves drift == 0 on
# every NON-pivot hop because each hop still nets exactly the edge length
# (``(distance_cm - offset)`` followed by ``offset`` is ``distance_cm``,
# independent of the offset magnitude).
LINE_SENSOR_AXLE_OFFSET_CM: float = 12.02

# MUST mirror ``Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM`` in WRO2026/Config.h.
# Used for REAR (reverse) overshoot so a rear-board hop dead-reckons with the
# rear board's own axle offset instead of the front board's. ADR-035: aligned
# to Config.h's authoritative REAR measurement (7.68 cm); the validator caps a
# line-incoming pivot ``delta`` at ``MIN(12.02, 7.68) = 7.68`` so the reduced
# REAR overshoot stays non-negative. Keep the two C++ and Python values in
# lockstep.
LINE_SENSOR_AXLE_OFFSET_REAR_CM: float = 7.68

# Floor (cm) for a pivot-trimmed dead-reckon advance / reduced LINE overshoot.
# Mirrors ``Config::PIVOT_TRIM_MIN_DRIVE_CM`` in WRO2026/Config.h (ADR-036; T8
# promoted it from a PathTracker.cpp file-local literal into Config.h per the
# single-source rule). The value stays 0.5.
# The validator guarantees ``delta < edge length`` on every dead-reckon side and
# ``delta <= overshoot budget`` on every LINE-incoming side, so a correctly built
# routes table never drives the effective distance to (or below) zero. This is a
# DEFENSIVE floor only: a future bad table / validator-config mismatch that
# produced a non-positive effective distance would, on the robot, make
# ``Drive.driveDist`` reject the negative cm and stall the FSM. Clamping to a
# tiny positive advance keeps the sim's geometry forward and in parity with the
# firmware's clamp instead of producing a negative displacement.
PIVOT_TRIM_MIN_DRIVE_CM: float = 0.5

# MUST mirror ``Config::NAV_TURN_TOLERANCE_DEG`` in WRO2026/Config.h.
# Pytest TC9 verifies parity via regex grep on the header (no C parser).
NAV_TURN_TOLERANCE_DEG: float = 0.5

# MUST mirror ``Config::PATH_PASS_THROUGH_ENABLED`` in WRO2026/Config.h.
# Pytest TC9 verifies parity via the same regex pattern (see
# ``test_tc9_path_pass_through_enabled_parity``). TC8 covers the
# behavioral disable, not the constant-value parity check.
PATH_PASS_THROUGH_ENABLED: bool = True

# MUST mirror ``Config::SPEED_NORMAL`` in WRO2026/config/ConfigDrive.h. The
# blind-run cruise PWM (``CRUISE_SPEED`` is symbolically the same value). The sim
# does not model speed presets or ramps for MOTION, but the ADR-059 blind-chain
# plan emits the RESOLVED velocity endpoints into the parity trace, so the run
# cruise + the captured pass-through cruise must match the firmware's
# ``Config::SPEED_NORMAL`` byte-for-byte. Hand-mirrored (Python cannot import the
# C++ header); no dedicated regex-parity test (it rides the golden-pinned
# ``chain_speed.py`` config parse + the parity sweep).
SPEED_NORMAL: int = 70

# MUST mirror ``Config::SPEED_SLOW`` / ``Config::SPEED_FAST`` in
# WRO2026/config/ConfigDrive.h -- the SLOW / FAST cruise PWM presets a per-edge
# ``SpeedTier`` resolves to (ADR-064; blind speed reuses the line presets). The
# resolved blind cruise, the RAW_FLAT flat plan, the blind-chain cruise, and the
# clamped pass-through entry seed all derive from these via
# :func:`_speed_tier_to_pwm`. Hand-mirrored alongside SPEED_NORMAL (Python cannot
# import the C++ header); keep SLOW=60 / NORMAL=70 / FAST=90 byte-identical to
# ConfigDrive.h -- the sim MUST mirror the robot (the parity sweep catches drift).
SPEED_SLOW: int = 60
SPEED_FAST: int = 90

# MUST mirror ``Config::TSHAPE_START_SPEED`` in WRO2026/config/ConfigDrive.h
# (``= PWM_MIN_MAGNITUDE`` = 35). The cold-start ramp seed a non-pass-through
# blind hop's velocity plan begins at (ADR-023). ADR-059 emits it as the
# ``start_pwm`` of every non-chained, non-pass-through ``blind_dead_reckon``
# record, so it must match the firmware's ``Config::TSHAPE_START_SPEED``.
# Hand-mirrored; keep in lockstep.
TSHAPE_START_SPEED: int = 35

# MUST mirror ``Config::TSHAPE_MIN_MOVE_CM`` in WRO2026/config/ConfigDrive.h
# (5.0). The Drive-profile floor below which a hop silently degrades to a flat
# move (its start/end speeds ignored). ADR-059: hop 0 and every candidate
# mid-hop below this floor is chain-ineligible (a chained hop must be able to
# carry its start/end speeds). Hand-mirrored; keep in lockstep with the firmware.
TSHAPE_MIN_MOVE_CM: float = 5.0

# ---------------------------------------------------------------------------
# Blind-chain continuous motion (ADR-059) — mirrored from ConfigNav.h. Each is a
# regex-grep parity test in ``tests/test_pass_through.py`` (same idiom as
# ``PATH_PASS_THROUGH_ENABLED``). All three gate the blind-chain walk-ahead in
# :meth:`PathTrackerSim._try_start_blind_chain`.
# ---------------------------------------------------------------------------

# MUST mirror ``Config::PATH_BLIND_CHAIN_ENABLED`` in WRO2026/config/ConfigNav.h.
# Master kill-switch: when True (default), consecutive collinear BLIND hops are
# folded into one chained run (setStartSpeed/setEndSpeed/setBrake(false) on every
# hop but the last) instead of braking to rest at each node. When False, every
# blind hop is its own braked stop-and-go move (today's / pre-ADR-059 behavior).
# Pytest verifies parity via regex grep on ConfigNav.h.
PATH_BLIND_CHAIN_ENABLED: bool = True

# MUST mirror ``Config::PATH_BLIND_CHAIN_MAX_CM`` in WRO2026/config/ConfigNav.h.
# LOOSE runaway guard on the total run distance (cm) a blind chain may span
# before it is forced to end (that hop brakes; a fresh chain re-forms after). The
# owner decision (2026-07) decoupled this from ``STOP_AT_LINE_ARM_WINDOW_CM`` and
# set it to 1000.0 (10 m) so it never binds in practice — ``MAX_CHAIN_HOPS`` is
# the practical bound. Mirror THIS value (1000.0). Pytest verifies parity via
# regex grep on ConfigNav.h.
PATH_BLIND_CHAIN_MAX_CM: float = 1000.0

# MUST mirror ``Config::MAX_CHAIN_HOPS`` in WRO2026/config/ConfigNav.h. Cap on
# the number of hops one blind chain run may contain (the firmware's fixed-size
# run-buffer dimension). The sim has no fixed-buffer constraint but honors the
# same cap so its output matches the firmware (ADR-050 parity). Pytest verifies
# parity via regex grep on ConfigNav.h.
MAX_CHAIN_HOPS: int = 8

# MUST mirror ``Config::LINE_HEADING_RESET_ON_PROJECTION`` in
# WRO2026/config/ConfigNav.h (ADR-052). Enables the line-arrival heading
# reset on a collinear PROJECTION pass-through (the line-follow PID never
# freezes across a projection node, so the heading is clean). Firmware
# default is ON.
LINE_HEADING_RESET_ON_PROJECTION: bool = True

# MUST mirror ``Config::LINE_HEADING_RESET_ON_JUNCTION`` in
# WRO2026/config/ConfigNav.h (ADR-052). Enables the line-arrival heading
# reset on a collinear physical-junction (LINE node) pass-through (the
# junction PID-freeze can inject a small wiggle into the bind). Firmware
# default is OFF for v1.
LINE_HEADING_RESET_ON_JUNCTION: bool = False

# MUST mirror ``Config::LINE_HEADING_RESET_STRICT_ENABLED`` in
# WRO2026/config/ConfigNav.h (ADR-052). The sim has no IMU/drift model and
# never strict-rejects a bind -- this constant is mirrored only for
# completeness/parity of the config surface, it is not read anywhere in
# this module.
LINE_HEADING_RESET_STRICT_ENABLED: bool = True

# MUST mirror ``Config::LINE_HEADING_RESET_TOLERANCE_DEG`` in
# WRO2026/config/ConfigNav.h (ADR-052). Gross-wrong-state strict-gate
# tolerance (deg). The sim never evaluates it (see
# ``LINE_HEADING_RESET_STRICT_ENABLED`` above) -- mirrored for config
# surface parity only.
LINE_HEADING_RESET_TOLERANCE_DEG: float = 10.0

# Sim-only primitive tag for the LINE_OVERSHOOT FSM transition. The C++
# runtime models this as an FSM state (not as a compiled primitive), so
# it is intentionally NOT added to
# :mod:`tools.path_planner.compiler`'s primitive vocabulary -- only the
# sim emits it, downstream of every ``ACT_FOLLOW_INT``.
ACT_LINE_OVERSHOOT: str = "line_overshoot"

# Sim-only primitive tag for the pass-through (collinear) LINE_OVERSHOOT
# transition. Emitted INSTEAD OF ACT_LINE_OVERSHOOT when the next-hop edge
# from the intersected node is collinear (heading delta <= NAV_TURN_TOLERANCE_DEG)
# with the inbound edge AND the intersected node is not the final destination.
# Chassis position update is IDENTICAL to ACT_LINE_OVERSHOOT (5cm advance);
# the primitive name differs only for downstream verification + telemetry.
# See ADR-023 "Pass-through collinear intersections".
ACT_LINE_PASS: str = "line_pass"

# Sim-only primitive tag for the DISTANCE-gated `line(A)–projection(B)–line(C)`
# pass-through (T20, issue #64). Emitted INSTEAD OF ACT_FOLLOW_DIST on the A→B
# LINE-into-projection hop when B is a non-goal projection node whose outbound
# B→C hop is a LINE edge into a LINE (junction) node AND A→B→C are collinear /
# same-direction (the SAME ADR-023 heading check, plus the LINE-flank + LINE-C
# guards the firmware `_evaluateProjectionPassThrough` applies). The chassis
# advance and the `on_dead_reckoned_advance(B)` re-anchor are IDENTICAL to
# ACT_FOLLOW_DIST -- only the primitive name differs, signalling that the
# follower stays live across the marker-less node B for one continuous A→C
# motion (no stop). This is the projection twin of ACT_LINE_PASS: the firmware
# (T19) owns it as the `_projPassThrough` distance-gate FSM transition in the
# PROJECTION_FOLLOW state, NOT as a compiled primitive, so -- exactly like
# ACT_LINE_PASS / ACT_LINE_OVERSHOOT -- it is intentionally kept OUT of
# :mod:`tools.path_planner.compiler` (the compiler is pass-through-unaware on
# BOTH the line-line-line and the line-proj-line collapse; it emits the plain
# follow primitive either way). The TRIGGER is the A→B DISTANCE gate, not an
# intersection -- a projection node has no junction marker, so
# `Intersection.isAtIntersection()` never fires at B.
ACT_FOLLOW_DIST_PASS: str = "follow_line_to_distance_pass"

# Bidirectional line-following primitive tags. Each mirrors its forward
# counterpart (``ACT_FOLLOW_INT`` / ``ACT_FORWARD`` / ``ACT_FOLLOW_DIST``)
# but drives the chassis BACKWARD along its current heading instead of
# rotating in place. Emitted by :class:`PathTrackerSim.go_to` (T11+) when
# :func:`resolve_direction` selects REVERSE for the upcoming edge based on
# the turn delta vs :data:`BIDIRECTIONAL_TURN_THRESHOLD_DEG`. The C++
# runtime owns the matching ``EDGE_*_REVERSE`` dispatch path; the sim's
# emission must stay byte-identical to the compiler's output (TC18 parity).
#
# ``ACT_PROJECTION_REVERSE`` is the REAR mirror of the LINE→projection
# follow-to-distance hop (a LINE edge whose destination is a projection
# node, driven backward). Projection is no longer an edge type (issue #64),
# so the primitive's 4th element is the LINE edge's resolved ``SpeedTier`` (via
# :func:`resolve_speed`) -- ``ProjectionMode`` was deleted with the
# projection edge type.
ACT_LINE_REVERSE: str       = "line_reverse"          # (ACT_LINE_REVERSE, target_node, line_mode)
ACT_BLIND_REVERSE: str      = "blind_reverse"         # (ACT_BLIND_REVERSE, target_node, distance_cm, blind_mode)
ACT_PROJECTION_REVERSE: str = "projection_reverse"    # (ACT_PROJECTION_REVERSE, target_node, distance_cm, line_mode)

# REAR twin of :data:`ACT_FOLLOW_DIST_PASS` (T20, issue #64): the
# `line(A)–projection(B)–line(C)` pass-through emitted INSTEAD OF
# ``ACT_PROJECTION_REVERSE`` when the A→B reverse follow into a projection node
# qualifies for the continuous A→C collapse (the gate's direction check
# requires BOTH A→B and B→C to resolve REAR, so a REAR pass-through only fires
# on a fully-reverse collinear triple). The chassis reverse advance + the
# ``on_dead_reckoned_advance(B)`` re-anchor are IDENTICAL to
# ``ACT_PROJECTION_REVERSE`` -- only the tag differs, signalling the follower
# stays live across B. The firmware (T19) owns it as the same
# ``_projPassThrough`` distance-gate transition in PROJECTION_FOLLOW (uniform
# FRONT/REAR), so -- like the FRONT twin -- it is kept OUT of the compiler.
# 4-tuple shape: ``(ACT_PROJECTION_REVERSE_PASS, target_node, distance_cm,
# line_mode)``.
ACT_PROJECTION_REVERSE_PASS: str = "projection_reverse_pass"

# Reverse-direction stop-at-line primitive. Mirrors ``ACT_BLIND_REVERSE``'s
# 4-tuple shape ``(action, target_node, distance_cm, blind_mode)`` but
# re-anchors localization via :meth:`LocalizationSim.on_intersection` (the rear
# board detects the physical crossing line) instead of dead-reckoning. Emitted
# only for a 2-board REAR ``STOP_AT_LINE`` blind hop, where
# :func:`resolve_stop_at_line` returns ``seek=True`` with a rear-facing board.
# On a 1-board chassis a forced-REAR STOP_AT_LINE hop degrades to
# ``ACT_BLIND_REVERSE`` (no rear board to seek with). The C++ runtime owns the
# matching ``Drive.stopAtLine(...).setReverse(True)`` dispatch; this emission
# must stay byte-identical to the compiler's output (AC1 firmware<->offline
# agreement).
ACT_STOP_AT_LINE_REVERSE: str = "stop_at_line_reverse"  # (ACT_STOP_AT_LINE_REVERSE, target_node, distance_cm, blind_mode)

# Sim-only primitive tag for an ADR-059 blind-chain NON-braking mid-hop node
# boundary. Emitted at each intermediate node of a collinear BLIND chain that is
# driven as ONE continuous run — the twin of the firmware's `_ptEmitBlindPass`
# `blind_pass` parity record, and the blind counterpart of ``ACT_LINE_PASS`` /
# ``ACT_FOLLOW_DIST_PASS``. It carries NO chassis motion of its own (the chained
# ``ACT_FORWARD`` / ``ACT_BLIND_REVERSE`` hop already advanced onto the node and
# re-anchored via ``on_dead_reckoned_advance``); it marks the boundary the run
# passed WITHOUT braking so downstream consumers render one whole-run trapezoid
# instead of a per-node dip. 2-tuple shape: ``(ACT_BLIND_PASS, node)`` (``node``
# is the intermediate node just passed, mirroring the firmware record's
# ``node`` field — deliberately NOT ``kind``, the record-type discriminator).
#
# THREE lock-step places (memory `gotcha-ctps-drive-method-contract`): (1) the
# sim emit HERE + in ``_walk_hops``; (2) the CTPS render in
# ``ctps/track_builder.py`` (a later task imports THIS constant); (3) the strict
# parity schema in ``sim/parity.py`` (``TRACE_RECORD_FIELDS["blind_pass"]``). It
# is intentionally NOT in ``ctps/trace_loader.py`` — that module whitelists
# ``drive.method`` strings, not ``ACT_*`` tags. The final (braked) arrival of a
# chain emits NO ``ACT_BLIND_PASS`` (the run retires at the final hop's dispatch,
# exactly like the firmware's `_chain.active` gate on the pass emit).
ACT_BLIND_PASS: str = "blind_pass"


# MUST mirror ``Config::BIDIRECTIONAL_AUTO_ENABLED`` in WRO2026/Config.h.
# Python cannot import C++ headers, so this constant is hand-mirrored.
# Pytest TC14a/b verify parity via regex grep on the header (no C parser);
# see ``test_tc14_bidirectional_*_parity`` for the defending tests.
BIDIRECTIONAL_AUTO_ENABLED: bool = True

# MUST mirror ``Config::BIDIRECTIONAL_TURN_THRESHOLD_DEG`` in WRO2026/Config.h.
# When the pre-edge turn delta exceeds this threshold (absolute degrees),
# the runtime drives the rear sensor toward the next intersection (emitting
# ``ACT_*_REVERSE``) instead of rotating in place + driving forward. TC14a/b
# defend the value parity.
BIDIRECTIONAL_TURN_THRESHOLD_DEG: float = 90.0

# MUST mirror ``Config::LINE_BOARD_COUNT`` in WRO2026/Config.h.
# Python cannot import C++ headers, so this constant is hand-mirrored.
# Default to 2 here (the bidirectional-capable chassis configuration);
# the resolve_direction "auto" step short-circuits to FRONT when
# LINE_BOARD_COUNT < 2 since the rear board is required to follow a line
# backward. Tests that need to exercise the 1-board fault path can patch
# this module-level constant the same way TC9 patches PATH_PASS_THROUGH_ENABLED.
LINE_BOARD_COUNT: int = 2


# ---------------------------------------------------------------------------
# Operator offset + return-to-node constants (issue #36, PR1b)
# ---------------------------------------------------------------------------
#
# Every constant below is hand-mirrored from WRO2026/ConfigLocalization.h.
# Python cannot import C++ headers, so the values must be kept in lockstep with
# the firmware (the parity tests drive both the C++ resolveReturnAction and the
# Python resolve_return_action over the same boundaries).

# MUST mirror ``Config::OFFSET_TRANSLATION_DEADBAND_CM`` in
# WRO2026/ConfigLocalization.h. Per-tick net translation (cm) below which a raw
# operator move is treated as encoder noise and dropped from the offset's
# translation channel (the channel's baseline is also left unadvanced so
# sub-deadband motion can still accumulate across ticks).
OFFSET_TRANSLATION_DEADBAND_CM: float = 0.2

# MUST mirror ``Config::OFFSET_ROTATION_DEADBAND_DEG`` in
# WRO2026/ConfigLocalization.h. Per-tick absolute IMU-yaw delta (deg) below
# which a raw operator turn is treated as IMU noise and dropped from the
# offset's rotation channel. Gated INDEPENDENTLY of the translation deadband.
OFFSET_ROTATION_DEADBAND_DEG: float = 0.5

# MUST mirror ``Config::RETURN_TRIGGER_THRESHOLD_CM`` in
# WRO2026/ConfigLocalization.h. Offset translation magnitude (cm) at or below
# which a ``go_to`` is treated as on-node (no return move; the
# :class:`ReturnAction.NONE` path). Above it the return-to-node prologue fires.
RETURN_TRIGGER_THRESHOLD_CM: float = 3.0

# MUST mirror ``Config::RETURN_DISTANCE_CAP_CM`` in
# WRO2026/ConfigLocalization.h. Offset translation magnitude (cm) above which
# the excursion is too large to trust for a dead-reckoned return and the
# off-map gate (:data:`OFF_MAP_GATE`) fires. At or below it a return is
# dispatched (the return distance is itself clamped to this cap).
RETURN_DISTANCE_CAP_CM: float = 50.0


class OffMapGate(IntEnum):
    """Policy applied when the offset magnitude exceeds the return cap.

    Mirror of the C++ ``enum class Config::OffMapGate : uint8_t`` in
    ``WRO2026/ConfigLocalization.h``.

    * ``FAULT`` -- refuse to plan; raise the off-map fault (the firmware
      latches ``Fault::OFF_NODE``). The safer default.
    * ``BEST_EFFORT`` -- log a warning and attempt the (capped) return
      anyway.
    """

    FAULT       = 0
    BEST_EFFORT = 1


# MUST mirror ``Config::OFF_MAP_GATE`` in WRO2026/ConfigLocalization.h. The
# firmware default is ``OffMapGate::FAULT`` (refuse-and-fault when the offset
# exceeds the cap). Tests that need to exercise the BEST_EFFORT branch patch
# this module-level constant the same way TC9 patches PATH_PASS_THROUGH_ENABLED.
OFF_MAP_GATE: "OffMapGate" = OffMapGate.FAULT


class ReturnAction(IntEnum):
    """Outcome of the return-to-node gate decision for one ``go_to`` start.

    Mirror of the C++ ``enum class ReturnAction : uint8_t`` in
    ``WRO2026/ReturnToNodePolicy.h``. See :func:`resolve_return_action` for
    the four-case rule.

    * ``NONE`` -- offset within trigger; behave exactly as today (no return).
    * ``RETURN`` -- offset over trigger but within cap; drive the inverse.
    * ``OFF_MAP_FAULT`` -- offset over cap and gate ``FAULT``; refuse to plan.
    * ``OFF_MAP_PROCEED`` -- offset over cap and gate ``BEST_EFFORT``; return
      anyway (capped).
    """

    NONE            = 0
    RETURN          = 1
    OFF_MAP_FAULT   = 2
    OFF_MAP_PROCEED = 3


# Sim-only primitive tag for the line-backed return seek. Mirrors the firmware's
# ``Drive.stopAtLine(returnDistCm).setMode(GYRO).setHeading(...)`` dispatch in
# ``PathTracker::_dispatchReturnMove`` for a ``NodeType::LINE`` anchor. The C++
# runtime owns this as the RETURN_TO_NODE FSM state (not a compiled primitive),
# so -- like ACT_LINE_OVERSHOOT -- it is intentionally kept out of
# :mod:`tools.path_planner.compiler`. 4-tuple shape:
# ``(ACT_RETURN_STOP_AT_LINE, node, dist_cm, heading_world_deg)``.
ACT_RETURN_STOP_AT_LINE: str = "return_stop_at_line"

# Sim-only primitive tag for the blind-only dead-reckon return drive. Mirrors
# the firmware's ``Drive.turnToYaw(...) ; Drive.driveDist(returnDistCm)``
# dispatch for a ``NodeType::BLIND`` anchor. Preceded by an ``(ACT_TURN_REL,
# delta)`` that aligns the chassis to the return heading (the ``turnToYaw``).
# 3-tuple shape: ``(ACT_RETURN_FORWARD, node, dist_cm)``.
ACT_RETURN_FORWARD: str = "return_forward"


# ---------------------------------------------------------------------------
# Wall-zero reset nodes (PR2, AC17 pillar 2)
# ---------------------------------------------------------------------------
#
# On arrival at a node typed ``"reset"`` (carrying a sparse ``RESET_NODES``
# entry ``{axis, wall_coord, facing}``), the firmware runs a two-phase
# interlude BEFORE planning the next hop or declaring the goal reached
# (``PathTracker::_onPlanNextHop`` -> ``_maybeEnterReset`` -> WALL_STOP ->
# REANCHOR). The sim mirrors that interlude inline in :meth:`PathTrackerSim._walk_hops`
# the moment a hop re-anchors localization onto a reset node.

# MUST mirror ``Config::WALL_STOP_BACK_OFF_CM`` in WRO2026/Config.h. Distance
# (cm) the chassis rolls back off the wall after a confirmed stall, in the
# reverse-of-push direction, so it is no longer pressing the wall when control
# returns to the caller. Hand-mirrored (Python cannot import the C++ header);
# keep in lockstep with the firmware constant.
WALL_STOP_BACK_OFF_CM: float = 2.0

# MUST mirror ``Config::WALL_CONTACT_OFFSET_CM`` in WRO2026/config/ConfigNav.h.
# Distance (cm) the wheel AXLE sits short of the wall surface when the chassis
# presses its REAR contact face flat against the wall -- the axle is the tracked
# reference, on the NODE side of the wall. The reset re-anchor sets the believed
# on-axis coordinate to ``wall_coord - sign(gap)*WALL_CONTACT_OFFSET_CM`` (the
# axle), then the depart drives this margin forward back onto the node.
# Hand-mirrored (Python cannot import the C++ header); keep in lockstep with the
# firmware constant.
WALL_CONTACT_OFFSET_CM: float = 8.0

# Sim-only primitive tag for the wall-stop push (phase 1 of the reset
# interlude). Mirrors the firmware's ``Drive.wallStop(distToWall).setMode(GYRO)
# .setHeading(facing).async()`` dispatch in ``PathTracker::_dispatchWallStop``.
# The C++ runtime owns this as the WALL_STOP FSM state (not a compiled
# primitive), so -- like ACT_LINE_OVERSHOOT / ACT_RETURN_* -- it is
# intentionally kept out of :mod:`tools.path_planner.compiler`. 4-tuple shape:
# ``(ACT_WALL_STOP, node, axis, wall_coord)`` -- ``axis`` is ``0`` (x) or ``1``
# (y), matching the cross-task ResetNode encoding the firmware decodes.
ACT_WALL_STOP: str = "wall_stop"

# Sim-only primitive tag for the wall re-anchor (phase 2 of the reset
# interlude). Mirrors the firmware's ``Localization.reanchorAtWall(axis,
# wall_coord, facing)`` call in the REANCHOR FSM state: it squares heading to
# ``facing`` and sets the BELIEVED on-axis coordinate to the wheel AXLE --
# ``wall_coord - sign(gap)*WALL_CONTACT_OFFSET_CM`` where ``gap = wall_coord -
# node_coord`` -- on the NODE side of the wall surface (any pre-existing drift on
# ``axis`` is corrected). The 4th element stays ``wall_coord`` (the press target
# the firmware reports); the axle pull-back is applied to the believed pose.
# 4-tuple shape: ``(ACT_REANCHOR, node, axis, wall_coord)``.
ACT_REANCHOR: str = "reanchor"

# Sim-only primitive tag for the line-arrival heading reset (ADR-052).
# Mirrors the firmware's ``Localization.reanchorHeading(...)`` call in
# ``PathTracker::_maybeHeadingReset``: on a collinear pass-through arrival at
# node B (a PROJECTION or LINE node, gated by the matching
# ``LINE_HEADING_RESET_ON_*`` knob), the IMU-yaw offset is re-bound to the
# incoming edge's already-known body heading -- heading-only, no motion, no
# pose change. Emitted ONLY on a successful bind (mirroring the firmware's
# accept-only parity record; the sim never strict-rejects since it has no
# drift model). 4-tuple shape: ``(ACT_HEADING_RESET, node, pass_kind,
# heading_deg)`` -- ``pass_kind`` is ``"proj"`` (PROJECTION pass-through) or
# ``"junction"`` (LINE pass-through), ``heading_deg`` is the body-corrected
# incoming heading the chassis arrives on at ``node`` (the REAR ``+180`` is
# already applied when the pass-through hop is reversed).
ACT_HEADING_RESET: str = "heading_reset"

# Sim-only primitive tag for the return-to-node depart (phase 3 of the reset
# interlude). Mirrors the firmware's WALL_DEPART move in
# ``PathTracker::_stepFsm`` -- ``Drive.driveDist(standoff -
# WALL_CONTACT_OFFSET_CM).setMode(GYRO)`` (``standoff = abs(wall_coord -
# node_coord)``) then ``Localization.clearOffset()``: the chassis (squared to
# ``facing``, front AWAY from the wall) drives FORWARD off the wall back onto the
# node, so the net believed on-axis coordinate becomes exactly ``node_coord``.
# Like ACT_WALL_STOP / ACT_REANCHOR the C++ runtime owns this as an FSM state,
# not a compiled primitive, so it is intentionally kept out of
# :mod:`tools.path_planner.compiler`. 4-tuple shape:
# ``(ACT_WALL_DEPART, node, axis, node_coord)`` -- the 4th element is the node's
# own on-axis coordinate the chassis returns to (NOT ``wall_coord``).
ACT_WALL_DEPART: str = "wall_depart"

# The load-bearing ResetNode axis encoding (``"x" -> 0``, ``"y" -> 1``) -- the
# cross-task contract the emitter (``header_writer._RESET_AXIS_TO_UINT8``) and
# the firmware (``Localization::coordOnAxis``) also hold. Named once here so the
# sim does not open-code ``0 if axis == "x" else 1`` inline (a drift hazard if
# the encoding ever changes). Do NOT reorder.
_RESET_AXIS_TO_IDX: dict = {"x": 0, "y": 1}


def _wall_axle_coord(wall_coord: float, node_coord: float) -> float:
    """Pull the believed on-axis coordinate back from the wall to the wheel axle.

    Mirror of the firmware's ``reanchorAtWall`` axle re-anchor: the chassis
    presses its REAR contact face against the wall surface (``wall_coord``), but
    the tracked reference is the wheel AXLE, which sits
    :data:`WALL_CONTACT_OFFSET_CM` short of that surface on the NODE side.
    ``sign(gap)`` (``gap = wall_coord - node_coord``) selects the node-side
    direction so the pull-back is toward the node regardless of which side of the
    node the wall is on. Shared by :meth:`PathTrackerSim._dispatch_wall_reset`
    (phase 2) and :func:`tools.path_planner.sim.parity._apply_wall` so the two
    surfaces compute one identical axle coordinate.

    :param wall_coord: the wall-surface on-axis coordinate the chassis pressed to.
    :param node_coord: the reset node's own on-axis coordinate.
    :return: the axle on-axis coordinate (:data:`WALL_CONTACT_OFFSET_CM` short of
        the wall, on the node side).
    """
    sign = 1.0 if (wall_coord - node_coord) >= 0.0 else -1.0
    return wall_coord - sign * WALL_CONTACT_OFFSET_CM


class Direction(IntEnum):
    """Travel direction the chassis takes through an edge.

    Mirror of the C++ ``enum class Direction : uint8_t`` in
    ``WRO2026/NavTypes.h``. ``AUTO`` and :class:`DirectionMode.DEFAULT`
    both encode "no override" but are distinct types — never compare
    across them. The resolution algorithm tests each independently.
    """

    AUTO  = 0    # defer to per-edge override / auto algorithm
    FRONT = 1    # force forward (chassis front leads)
    REAR  = 2    # force reverse (chassis rear leads)


class StopAtLine(IntEnum):
    """Per-hop call-site stop-at-line override.

    Mirror of the C++ ``enum class StopAtLine : uint8_t`` in
    ``WRO2026/NavTypes.h``. ``AUTO`` defers to the per-edge
    :class:`BlindMode` (seek only when the edge is ``STOP_AT_LINE``); ``ON``
    forces the seek on any BLIND edge; ``OFF`` force-disables it (the hop
    dead-reckons). The offline compiler/sim have no call-site verb today, so
    they always pass ``AUTO`` — matching the firmware's per-edge AUTO default.
    """

    AUTO = 0    # defer to the per-edge BlindMode override
    ON   = 1    # force stop-at-line on this hop's BLIND edge (overrides edge)
    OFF  = 2    # force-disable; traverse the BLIND edge as a dead-reckoned hop


def wrap_pm180(delta_deg: float) -> float:
    """Wrap a heading delta into ``(-180, +180]`` degrees.

    Thin alias of :func:`tools.path_planner.compiler.normalize_turn`,
    re-exposed under the name the resolution algorithm uses in the plan
    so the Python code reads identically to the C++ helper.
    """
    return normalize_turn(delta_deg)


__all__ = [
    "ACT_BLIND_PASS",
    "ACT_BLIND_REVERSE",
    "ACT_FOLLOW_DIST_PASS",
    "ACT_HEADING_RESET",
    "ACT_LINE_OVERSHOOT",
    "ACT_LINE_PASS",
    "ACT_LINE_REVERSE",
    "ACT_PROJECTION_REVERSE",
    "ACT_PROJECTION_REVERSE_PASS",
    "ACT_REANCHOR",
    "ACT_RETURN_FORWARD",
    "ACT_RETURN_STOP_AT_LINE",
    "ACT_STOP_AT_LINE_REVERSE",
    "ACT_WALL_DEPART",
    "ACT_WALL_STOP",
    "BIDIRECTIONAL_AUTO_ENABLED",
    "BIDIRECTIONAL_TURN_THRESHOLD_DEG",
    "LINE_BOARD_COUNT",
    "LINE_HEADING_RESET_ON_JUNCTION",
    "LINE_HEADING_RESET_ON_PROJECTION",
    "LINE_HEADING_RESET_STRICT_ENABLED",
    "LINE_HEADING_RESET_TOLERANCE_DEG",
    "LINE_SENSOR_AXLE_OFFSET_CM",
    "LINE_SENSOR_AXLE_OFFSET_REAR_CM",
    "MAX_CHAIN_HOPS",
    "NAV_TURN_TOLERANCE_DEG",
    "OFFSET_ROTATION_DEADBAND_DEG",
    "OFFSET_TRANSLATION_DEADBAND_CM",
    "OFF_MAP_GATE",
    "PATH_BLIND_CHAIN_ENABLED",
    "PATH_BLIND_CHAIN_MAX_CM",
    "PATH_PASS_THROUGH_ENABLED",
    "PIVOT_TRIM_MIN_DRIVE_CM",
    "RETURN_DISTANCE_CAP_CM",
    "RETURN_TRIGGER_THRESHOLD_CM",
    "SPEED_FAST",
    "SPEED_NORMAL",
    "SPEED_SLOW",
    "TSHAPE_MIN_MOVE_CM",
    "TSHAPE_START_SPEED",
    "TURN_EPSILON_DEG",
    "WALL_CONTACT_OFFSET_CM",
    "WALL_STOP_BACK_OFF_CM",
    "Direction",
    "DispatchPrimitive",
    "EdgeDispatchDecision",
    "EdgeDispatchQuery",
    "LocalizationSim",
    "Offset2D",
    "OffMapFaultError",
    "OffMapGate",
    "PathTrackerSim",
    "ReanchorKind",
    "ReturnAction",
    "StopAtLine",
    "UnreachableGoalError",
    "resolve_blind_mode",
    "resolve_direction",
    "resolve_edge_dispatch",
    "resolve_return_action",
    "resolve_sensor_overshoot_trim",
    "resolve_speed",
    "resolve_stop_at_line",
    "walk_next_hop",
    "wrap_pm180",
]


class UnreachableGoalError(RuntimeError):
    """Raised when ``next_hop[from][goal] is None`` mid-traverse."""

    def __init__(
        self,
        from_node: str,
        goal_node: str,
        *,
        current: Optional[str] = None,
    ) -> None:
        ctx = f" (current={current!r})" if current is not None else ""
        super().__init__(
            f"unreachable: from={from_node!r} to={goal_node!r}{ctx}"
        )
        self.from_node = from_node
        self.goal_node = goal_node
        self.current = current


class OffMapFaultError(RuntimeError):
    """Raised by :meth:`PathTrackerSim.go_to` on an over-cap offset under FAULT.

    Sim mirror of the firmware's ``_enterFault(Fault::OFF_NODE, ...)`` path in
    ``PathTracker::_evaluateReturnGate``: when the accumulated operator-offset
    magnitude exceeds :data:`RETURN_DISTANCE_CAP_CM` AND
    :data:`OFF_MAP_GATE` is :attr:`OffMapGate.FAULT`, the firmware refuses to
    plan and latches ``Fault::OFF_NODE``. The sim has no latched-fault FSM
    state to mirror, so it surfaces the same refusal as a raised exception the
    caller can assert on (mirroring how :class:`UnreachableGoalError` surfaces
    the firmware's ``Fault::UNREACHABLE``). The chassis is NOT moved and the
    offset is left untouched -- the goal is rejected before any return drive.
    """

    def __init__(self, node: str, goal_node: str, offset_mag_cm: float) -> None:
        super().__init__(
            f"off-map: offset {offset_mag_cm:.3f} cm exceeds cap "
            f"{RETURN_DISTANCE_CAP_CM} cm at node={node!r} "
            f"(goal={goal_node!r}); OFF_MAP_GATE=FAULT, refusing to plan"
        )
        self.node = node
        self.goal_node = goal_node
        self.offset_mag_cm = offset_mag_cm


@dataclass
class Offset2D:
    """Accumulated pose offset from the last anchored node (operator excursion).

    Function-identical Python mirror of the C++ ``struct Offset2D`` in
    ``WRO2026/Localization.h``: ``x`` / ``y`` are the world-frame displacement
    (cm) from the anchored node, ``theta_deg`` a signed heading delta (deg).
    All three fields are ``0`` while the chassis sits on its anchored node -- a
    node arrival (re-anchor) zeroes the offset.

    :ivar x: world-frame ``+x`` displacement from the anchored node (cm).
    :ivar y: world-frame ``+y`` displacement from the anchored node (cm).
    :ivar theta_deg: signed heading delta from the anchored node (deg). A
        delta-from-anchor, NOT an authoritative heading (mirrors the C++
        field's ``thetaDeg`` semantics).
    """

    x: float = 0.0
    y: float = 0.0
    theta_deg: float = 0.0

    def magnitude_cm(self) -> float:
        """Return the translation magnitude ``sqrt(x^2 + y^2)`` (cm).

        Mirrors ``sqrtf(off.x * off.x + off.y * off.y)`` in
        ``PathTracker::_evaluateReturnGate`` -- the position metric that
        governs the trigger / cap (the heading delta ``theta_deg`` does NOT
        gate the return; see the Config docstrings).
        """
        return math.hypot(self.x, self.y)


def resolve_return_action(
    offset_mag_cm: float,
    trigger_cm: float,
    cap_cm: float,
    gate: "OffMapGate",
) -> "ReturnAction":
    """Resolve whether a ``go_to`` returns to the anchored node first, and how.

    Function-identical Python mirror of the C++
    ``ReturnToNodePolicy.h::resolveReturnAction`` (T8). Both implementations
    must stay byte-for-byte equivalent so the offline sim and the robot agree
    on the gate at every ``goTo`` / ``go_to`` start;
    ``test_return_action_parity_*`` drives the same boundaries through this
    helper and a frozen table of the C++ outputs (matching how
    :func:`resolve_stop_at_line` / :func:`resolve_direction` mirror their C++
    policy headers).

    The four-case rule (branch order matters -- the cap test runs BEFORE
    deciding RETURN so an over-cap offset never silently falls into a plain
    return):

    1. ``mag <= trigger`` -> :attr:`ReturnAction.NONE` (on-node; plan as
       today). Checked first so the offset-around-zero case short-circuits.
    2. ``mag > cap`` -> off-map gate:
       :attr:`ReturnAction.OFF_MAP_FAULT` if ``gate == OffMapGate.FAULT`` else
       :attr:`ReturnAction.OFF_MAP_PROCEED`.
    3. ``trigger < mag <= cap`` -> :attr:`ReturnAction.RETURN` (drive the
       inverse return).

    The Config static_asserts in ``ConfigLocalization.h`` guarantee
    ``0 < trigger <= cap``, so the cases are exhaustive and non-overlapping for
    any finite non-negative @p offset_mag_cm.

    :param offset_mag_cm: accumulated operator-offset translation magnitude
        (``sqrt(off.x^2 + off.y^2)``), in cm. Expected finite, non-negative.
    :param trigger_cm: :data:`RETURN_TRIGGER_THRESHOLD_CM` -- at or below this
        the chassis is treated as on-node (NONE).
    :param cap_cm: :data:`RETURN_DISTANCE_CAP_CM` -- above this the off-map
        gate fires; at or below it a return is dispatched.
    :param gate: :data:`OFF_MAP_GATE` -- selects FAULT vs BEST_EFFORT when the
        offset exceeds @p cap_cm.
    :return: the :class:`ReturnAction` per the four-case rule.
    """
    # Case 1: within trigger -- on-node, plan as today. Checked first so the
    # offset-around-zero case short-circuits with no further work.
    if offset_mag_cm <= trigger_cm:
        return ReturnAction.NONE

    # Case 3/4: over the trust cap -- the off-map gate decides. Checked BEFORE
    # returning RETURN so an over-cap offset can never fall into a plain return.
    if offset_mag_cm > cap_cm:
        return (
            ReturnAction.OFF_MAP_FAULT
            if gate == OffMapGate.FAULT
            else ReturnAction.OFF_MAP_PROCEED
        )

    # Case 2: trigger < mag <= cap -- drive the inverse return.
    return ReturnAction.RETURN


def resolve_pivot_trim(
    raw_distance_cm: float,
    departing_trim_cm: float,
    approaching_trim_cm: float,
    min_drive_cm: float,
) -> Tuple[float, float, bool]:
    """Resolve the effective distance after the two pivot trims (shared core).

    Function-identical Python twin of the C++
    ``PivotTrimPolicy.h::resolvePivotTrim``. The single subtract-and-clamp core
    that BOTH the DISTANCE path (:meth:`PathTrackerSim._pivot_trim_distance`) and
    the SENSOR path (:func:`resolve_sensor_overshoot_trim`, departing fixed at
    ``0.0``) route through -- mirroring the C++ ``resolvePivotTrim`` /
    ``resolveSensorOvershootTrim`` split so the two paths physically cannot
    drift::

        effective = raw_distance_cm - departing_trim_cm - approaching_trim_cm
        drive     = max(effective, min_drive_cm)
        clamped   = effective < min_drive_cm   # strict < : floor-equal is NOT clamped

    :param raw_distance_cm: the directed edge length (DISTANCE path) or the
        sensor-to-axle offset (SENSOR path).
    :param departing_trim_cm: outgoing trim of the pivot being LEFT, or ``0.0``.
    :param approaching_trim_cm: approach trim of the pivot being ENTERED, or
        ``0.0``.
    :param min_drive_cm: :data:`PIVOT_TRIM_MIN_DRIVE_CM` underflow floor.
    :return: ``(drive_cm, effective_cm, clamped)`` -- ``effective_cm`` is the
        pre-clamp value (kept for the underflow trace), ``drive_cm`` is
        ``max(effective, floor)``, ``clamped`` is true iff the floor fired.
    """
    effective_cm = raw_distance_cm - departing_trim_cm - approaching_trim_cm
    drive_cm = max(effective_cm, min_drive_cm)
    clamped = effective_cm < min_drive_cm
    return drive_cm, effective_cm, clamped


def resolve_sensor_overshoot_trim(
    axle_offset_cm: float,
    approaching_trim_cm: float,
    min_drive_cm: float,
) -> Tuple[float, bool]:
    """Reduce a SENSOR-anchored hop's post-detect overshoot by the pivot trim.

    Function-identical Python twin of the C++
    ``PivotTrimPolicy.h::resolveSensorOvershootTrim`` (ADR-044 D1). Both
    implementations MUST stay step-for-step identical so the offline sim and
    the robot agree on the post-detect axle roll for every sensor-terminated
    hop that drives INTO a pivot node; TC13 (T9) drives a value grid through
    this helper and the C++ helper and asserts they match.

    A sensor-anchored hop (a LINE overshoot or a STOP_AT_LINE seek) stops the
    chassis when the board detects the physical crossing line, then rolls the
    axle forward by the returned ``overshoot_cm`` so the wheel axle -- not the
    sensor -- settles relative to the node. When the hop drives into a pivot
    node, the roll is shortened by the approaching trim so the axle settles
    ``delta`` short of the node (the pivot starts at the geometric pivot
    point). The subtract-and-clamp is delegated to the shared core
    :func:`resolve_pivot_trim` (departing fixed at ``0.0``) -- the SAME core the
    DISTANCE-path :meth:`PathTrackerSim._pivot_trim_distance` routes through, so
    the SENSOR path and the DISTANCE path can never drift:

        effective   = axle_offset_cm - approaching_trim_cm
        overshoot_cm = max(effective, min_drive_cm)
        clamped      = effective < min_drive_cm

    With no approaching pivot the trim is ``0.0`` so ``overshoot_cm`` stays the
    full ``axle_offset_cm`` and a non-pivot hop is unchanged.

    :param axle_offset_cm: nominal post-detect roll -- the sensor-to-axle
        offset (e.g. :data:`LINE_SENSOR_AXLE_OFFSET_CM`).
    :param approaching_trim_cm: approach trim of the pivot being ENTERED, or
        ``0.0`` if this hop does not drive into a pivot.
    :param min_drive_cm: :data:`PIVOT_TRIM_MIN_DRIVE_CM` underflow floor.
    :return: ``(overshoot_cm, clamped)`` -- ``overshoot_cm = max(axle_offset_cm
        - approaching_trim_cm, min_drive_cm)``, ``clamped`` true iff the floor
        fired.
    :note: The departing trim is intentionally NOT a parameter: a sensor-
        terminated hop self-corrects on the line, so the over-travel of the
        pivot just left does not accumulate into this roll (ADR-044 D1). The
        C++ twin passes the departing slot as a hard-coded ``0.0f`` for the
        same reason.
    """
    overshoot_cm, _effective_cm, clamped = resolve_pivot_trim(
        axle_offset_cm, 0.0, approaching_trim_cm, min_drive_cm
    )
    return overshoot_cm, clamped


def walk_next_hop(
    next_hop: NextHopTable,
    source: str,
    target: str,
) -> List[str]:
    """Return the node-name list traversed by repeatedly indexing ``next_hop``."""
    if source == target:
        return [source]
    path: List[str] = [source]
    current = source
    max_steps = len(next_hop) + 1
    for _ in range(max_steps):
        nxt = next_hop[current][target]
        if nxt is None:
            raise UnreachableGoalError(source, target, current=current)
        path.append(nxt)
        current = nxt
        if current == target:
            break
    if current != target:
        raise ValueError(
            f"walk_next_hop ran out of steps: source={source!r} target={target!r}"
        )
    return path


# ---------------------------------------------------------------------------
# Per-edge mode resolution helpers (pure-Python mirror of the C++ helpers)
# ---------------------------------------------------------------------------
#
# The runtime PathTracker resolves per-edge overrides at dispatch time:
# ``mode = edge.<category>_mode_override`` if that field is set to anything
# other than ``DEFAULT``, else ``PIPELINE_CONFIG.<category>``. These pure
# helpers mirror that lookup for the sim so T12 can verify the resolution
# matches the C++ structure on the same fixture graph.
#
# They do not change the sim's actual behaviour today (the sim does not
# model speed presets or wheel patterns); they exist as a clean
# verification surface for tests.


def _resolve_override(
    edge,
    attr: str,
    default_value,
    enum_cls,
):
    """Look up an edge override attribute, falling back to a pipeline default.

    Accepts either a dict-like ``edge`` (with ``.get``) or a plain mapping;
    treats a missing attribute as "inherit pipeline default", and treats
    an attribute equal to ``enum_cls.DEFAULT`` the same way (so explicit
    ``DEFAULT`` and missing both resolve identically, mirroring the C++
    ``DEFAULT == 0 == "inherit"`` sentinel).
    """
    override = edge.get(attr) if hasattr(edge, "get") else None
    if override is None:
        return default_value
    if isinstance(override, enum_cls):
        if override == enum_cls.DEFAULT:
            return default_value
        return override
    # Bare string names (e.g. 'NORMAL') -- use key lookup so IntEnum does not
    # reject them.  Mirrors header_writer._override_enum_value's str branch so
    # the two consumers agree on the accepted type set (finding #41).
    if isinstance(override, str):
        try:
            coerced = enum_cls[override]
        except KeyError:
            return default_value
        if coerced == enum_cls.DEFAULT:
            return default_value
        return coerced
    # Tolerate raw int values (the graph attribute could be persisted as an
    # int in some fixtures); coerce via the enum so type stays consistent.
    coerced = enum_cls(override)
    if coerced == enum_cls.DEFAULT:
        return default_value
    return coerced


def resolve_speed(edge, cfg: PipelineConfig = PIPELINE_CONFIG) -> SpeedTier:
    """Return the effective :class:`SpeedTier` for a traversal (any edge type).

    The per-edge ``speed`` override (a verbatim :class:`SpeedTier` name the graph
    builder forwards onto both directions) applies to BOTH LINE and BLIND hops
    (ADR-064 split the old unified ``primitive_mode`` key into ``speed`` +
    ``mode``); absent / ``DEFAULT`` inherits ``cfg.speed``.

    :param edge: directed graph edge attribute mapping (e.g.
        ``graph.edges[u, v]``). May or may not carry ``speed``.
    :param cfg: the resolved :class:`PipelineConfig` whose ``speed`` field
        supplies the inherit-on-DEFAULT fallback. Defaults to the module-level
        :data:`PIPELINE_CONFIG` singleton so legacy callers (and the many
        ``SimpleNamespace`` fixtures) keep working unchanged. ``compile_path``
        and ``PathTrackerSim._walk_hops`` pass a
        :meth:`PipelineConfig.from_field_data` result so a field-sourced
        ``PIPELINE_SPEED`` reaches the offline sim, not just the emitted
        ``routes.cpp`` (R4: the sim must mirror the robot).
    :return: the override if set (non-DEFAULT), else ``cfg.speed``.
    """
    return _resolve_override(edge, "speed", cfg.speed, SpeedTier)


def _speed_tier_to_pwm(tier: SpeedTier) -> int:
    """Map a resolved :class:`SpeedTier` to its cruise PWM magnitude.

    Function-identical mirror of the firmware
    ``presetToPwm(speedTierToPreset(tier))`` chain (``WROMath.h`` +
    ``ConfigDrive.h``): ``SLOW`` -> :data:`SPEED_SLOW` (60), ``NORMAL`` ->
    :data:`SPEED_NORMAL` (70), ``FAST`` -> :data:`SPEED_FAST` (90). ``DEFAULT``
    defensively maps to :data:`SPEED_NORMAL` -- the resolver delivers a concrete
    tier before this point (``DEFAULT`` is inherited upstream via
    :func:`resolve_speed`), matching the C++ default arm. Hand-mirrored; the
    parity sweep pins the values (the sim MUST mirror the robot).

    :param tier: a resolved :class:`SpeedTier` (never expected to be
        :attr:`SpeedTier.DEFAULT` in practice).
    :return: the unsigned cruise PWM magnitude; the caller applies the
        travel-direction sign.
    """
    if tier == SpeedTier.SLOW:
        return SPEED_SLOW
    if tier == SpeedTier.FAST:
        return SPEED_FAST
    return SPEED_NORMAL


def resolve_blind_mode(edge, cfg: PipelineConfig = PIPELINE_CONFIG) -> BlindMode:
    """Return the effective :class:`BlindMode` for an EDGE_BLIND traversal.

    :param edge: directed graph edge attribute mapping. May or may not carry
        ``mode`` (the BLIND drive-mode override the graph builder forwards onto
        both directions; ADR-064 split the old unified ``primitive_mode`` key
        into ``speed`` + ``mode``).
    :param cfg: the resolved :class:`PipelineConfig` whose ``blind`` field
        supplies the inherit-on-DEFAULT fallback. Defaults to the module-level
        :data:`PIPELINE_CONFIG` singleton (see :func:`resolve_speed` for
        why ``compile_path`` / ``_walk_hops`` thread a field-sourced config).
        ``BlindMode.STOP_AT_LINE`` stays resolvable from ``cfg.blind`` for the
        firmware seek-ladder dead path, but is no longer per-edge authorable.
    :return: the override if set (non-DEFAULT), else ``cfg.blind``.
    """
    return _resolve_override(edge, "mode", cfg.blind, BlindMode)


def resolve_stop_at_line(
    call_flag: "StopAtLine",
    edge_override: StopAtLineMode,
    edge_mode: BlindMode,
    pipeline_blind: BlindMode,
    direction: "Direction",
    line_board_count: int,
    auto_rule_active: bool,
) -> tuple[bool, BlindMode]:
    """Resolve whether a BLIND hop seeks a line and which motion bundle it runs.

    Function-identical Python mirror of the C++
    ``StopAtLinePolicy.h::resolveStopAtLine`` helper. Answers one question per
    BLIND hop: does this edge run a ``Drive.stopAtLine`` seek, and if not,
    which motion bundle does the dead-reckoned fallback use? The seek decision
    follows a strict precedence ladder.

    Precedence (highest wins; first match decides ``requested``):

    1. Call-site :attr:`StopAtLine.ON` / :attr:`StopAtLine.OFF` -- a per-hop
       verb beats everything below it.
    2. Per-edge :attr:`StopAtLineMode.ON` / :attr:`StopAtLineMode.OFF` -- the
       Routes ``stop_at_line_override`` baked by the graph builder (T5).
    3. Legacy force-on: the resolved edge :class:`BlindMode` is itself
       :attr:`BlindMode.STOP_AT_LINE` (the historical "STOP_AT_LINE tag means
       seek" behavior, kept for back-compat).
    4. Auto-rule: ``auto_rule_active`` (the caller's
       ``BLIND_PROJECTION_AUTO_SEEK and is_projection_node(destination)``) --
       a BLIND edge whose destination is a projection node seeks that node's
       own line first to re-anchor (full advance).
    5. Otherwise ``requested = False`` -- a call-site/edge ``AUTO``/``DEFAULT``
       with no rule active never seeks. This reproduces the pre-auto behavior
       across the FULL domain (including the degenerate
       ``edge_mode != STOP_AT_LINE, pipeline_blind == STOP_AT_LINE`` case,
       which still does not seek): the resolved ``edge_mode`` already encodes
       the pipeline default, so there is intentionally NO pipeline-default seek
       rung here.

    ``board_ok``: a line board must face the travel direction -- the front
    board always faces forward, but a rear seek needs a second board
    (``line_board_count >= 2``). ``seek = requested and board_ok``.

    ``motion_mode`` (unchanged from the pre-precedence logic): when seeking, the
    motion bundle is the edge's mode, except a bare :attr:`BlindMode.STOP_AT_LINE`
    edge has no velocity profile of its own and seeks under
    :attr:`BlindMode.SYNC_TSHAPE`. When not seeking, the hop dead-reckons: a
    :attr:`BlindMode.STOP_AT_LINE` edge falls back to the pipeline blind mode;
    any residual ``STOP_AT_LINE`` defensively degrades to
    :attr:`BlindMode.SYNC_TSHAPE`, so the returned ``motion_mode`` is never
    ``STOP_AT_LINE`` and never ``DEFAULT``.

    The offline compiler/sim have no call-site stop-at-line verb today, so they
    always pass ``call_flag=StopAtLine.AUTO`` -- matching the firmware's per-edge
    AUTO default. The full ``ON``/``OFF`` branches are implemented anyway so the
    signature mirrors the C++ surface and TC13 can drive every truth-table row
    through both languages.

    Keep step-for-step identical with
    ``WRO2026/StopAtLinePolicy.h::resolveStopAtLine``;
    ``tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv`` is the
    normative source; TC13 enforces parity.

    :param call_flag: PathTracker's ``_activeStopAtLine`` (ON/OFF/AUTO).
    :param edge_override: the per-edge ``stop_at_line_override``
        (``ON``/``OFF``/``DEFAULT``) baked into Routes by the graph builder.
    :param edge_mode: ``resolve_blind_mode(edge)`` -- already DEFAULT->pipeline
        resolved, so never :attr:`BlindMode.DEFAULT`.
    :param pipeline_blind: ``PIPELINE_CONFIG.blind`` -- the mission-wide blind
        default, used as the dead-reckon fallback for a ``STOP_AT_LINE`` edge
        that does not seek.
    :param direction: the post-:func:`resolve_direction` travel direction
        (``FRONT``/``REAR``; never ``AUTO`` here).
    :param line_board_count: number of MXColor reflectance boards on the
        chassis (mirrors ``Config::LINE_BOARD_COUNT``); gates a rear seek.
    :param auto_rule_active: the caller's
        ``BLIND_PROJECTION_AUTO_SEEK and is_projection_node(destination)`` --
        True when the field-wide auto-seek toggle is on AND this BLIND edge's
        destination node is a projection node.
    :return: a ``(seek, motion_mode)`` tuple. ``seek`` is True
        iff the ``Drive.stopAtLine`` seek should be dispatched; ``motion_mode``
        is the seek's drive mode (when seeking) or the dead-reckon fallback's
        drive mode (when not) -- always a concrete bundle, never ``STOP_AT_LINE``
        or ``DEFAULT``.
    """
    # Step 1: precedence ladder for who asked. Call-site ON/OFF beats the
    # per-edge override, which beats the legacy STOP_AT_LINE tag, which beats
    # the auto-rule. AUTO + DEFAULT + no rule active never seeks.
    if call_flag == StopAtLine.ON:
        requested = True
    elif call_flag == StopAtLine.OFF:
        requested = False
    elif edge_override == StopAtLineMode.ON:
        requested = True
    elif edge_override == StopAtLineMode.OFF:
        requested = False
    elif edge_mode == BlindMode.STOP_AT_LINE:    # legacy force-on
        requested = True
    elif auto_rule_active:
        requested = True
    else:    # AUTO call / DEFAULT edge, no rule active: never seek
        requested = False

    # Step 2: a line board must face the travel direction. Front always; rear
    # needs a second board.
    board_ok = (direction == Direction.FRONT) or (line_board_count >= 2)

    # Step 3: the seek runs only when requested and physically possible.
    seek = requested and board_ok

    if seek:
        # Step 4: keep the edge's typed mode; a bare STOP_AT_LINE edge seeks
        # under SYNC_TSHAPE (it has no velocity profile of its own).
        motion_mode = (
            BlindMode.SYNC_TSHAPE
            if edge_mode == BlindMode.STOP_AT_LINE
            else edge_mode
        )
    else:
        # Step 5: dead-reckon fallback. A STOP_AT_LINE edge falls back to the
        # pipeline blind mode; any residual STOP_AT_LINE defensively degrades
        # to SYNC_TSHAPE so the returned mode is never STOP_AT_LINE.
        base = pipeline_blind if edge_mode == BlindMode.STOP_AT_LINE else edge_mode
        motion_mode = BlindMode.SYNC_TSHAPE if base == BlindMode.STOP_AT_LINE else base

    return seek, motion_mode


def resolve_direction(
    edge_dir_override: DirectionMode,
    pipeline_default: DirectionMode,
    builder_arrival: "Direction",
    builder_route: "Direction",
    current_heading: float,
    edge_heading: float,
    is_last_hop: bool,
    bidirectional_enabled: bool,
    line_board_count: int,
) -> "Direction":
    """Resolve which physical direction the chassis takes through one hop.

    Function-identical Python mirror of the C++ ``resolveDirection`` helper in
    ``WRO2026/DirectionPolicy.h`` (T5). The algorithm is fully specified in the
    plan "API Contracts — Resolution algorithm" section; both implementations
    must stay byte-for-byte equivalent so the offline sim and the robot agree on
    every hop's direction. Cross-language parity is enforced by the shared-vector
    golden in ``tools/path_planner/tests/test_direction_parity.py``, which feeds
    one input set through this helper and asserts each result against a frozen
    table of the C++ outputs. (TC18 only checks the compiler<->sim
    *primitive-stream* parity, not this resolver algorithm.)

    Precedence (highest first):

    1. Per-edge override (``EdgeInfo::direction_override``) is binding.
    2. Pipeline default (``PipelineConfig::direction``) is binding when
       not ``DEFAULT``.
    3. Builder arrival override wins on the final hop.
    4. Builder route override wins on every hop.
    5. Auto: REAR only if both the kill-switch is on AND the chassis has
       a rear board AND the body would need to turn more than
       :data:`BIDIRECTIONAL_TURN_THRESHOLD_DEG` to face the edge forward.

    :param edge_dir_override: :class:`DirectionMode` from the graph edge
        attribute (or ``DirectionMode.DEFAULT`` when absent).
    :param pipeline_default: :class:`DirectionMode` from
        :data:`PIPELINE_CONFIG.direction`.
    :param builder_arrival: :class:`Direction` set via
        ``setArrivalDirection``; ``Direction.AUTO`` means no override.
        Only considered on the last hop of the route.
    :param builder_route: :class:`Direction` set via
        ``setRouteDirection``; ``Direction.AUTO`` means no override.
        Applies to every hop.
    :param current_heading: chassis heading in degrees (any wrap; the
        algorithm wraps the delta into ``(-180, +180]`` internally).
    :param edge_heading: forward heading of the candidate edge in degrees.
    :param is_last_hop: True iff the candidate edge terminates at the
        final destination of the route.
    :param bidirectional_enabled: project-wide kill switch for the auto
        REAR step (mirrors ``Config::BIDIRECTIONAL_AUTO_ENABLED``).
    :param line_board_count: number of MXColor reflectance boards on
        the chassis (mirrors ``Config::LINE_BOARD_COUNT``). Auto step
        short-circuits to ``Direction.FRONT`` when ``< 2``.
    :return: resolved :class:`Direction` — always one of ``FRONT`` or
        ``REAR`` (never ``AUTO``; the algorithm always resolves).
    """
    # Step 1: per-edge override is binding.
    if edge_dir_override == DirectionMode.FORCE_FORWARD:
        return Direction.FRONT
    if edge_dir_override == DirectionMode.FORCE_REVERSE:
        return Direction.REAR

    # Step 2: pipeline default is binding when not DEFAULT.
    if pipeline_default == DirectionMode.FORCE_FORWARD:
        return Direction.FRONT
    if pipeline_default == DirectionMode.FORCE_REVERSE:
        return Direction.REAR

    # Step 3: arrival override wins on the final hop.
    if is_last_hop and builder_arrival != Direction.AUTO:
        return builder_arrival

    # Step 4: route override wins on all hops.
    if builder_route != Direction.AUTO:
        return builder_route

    # Step 5: auto -- only if 2-board chassis AND kill-switch on.
    if not bidirectional_enabled:
        return Direction.FRONT
    if line_board_count < 2:
        return Direction.FRONT

    delta = wrap_pm180(edge_heading - current_heading)
    if abs(delta) > BIDIRECTIONAL_TURN_THRESHOLD_DEG:
        return Direction.REAR
    return Direction.FRONT


# ---------------------------------------------------------------------------
# Terminal one-hop edge-dispatch classifier (mirror of EdgeDispatchPolicy.h)
# ---------------------------------------------------------------------------


class DispatchPrimitive(IntEnum):
    """The terminal one-hop motion primitive a resolved edge dispatches to.

    Mirror of the C++ ``enum class DispatchPrimitive : uint8_t`` in
    ``WRO2026/EdgeDispatchPolicy.h``; the integer values are byte-identical so a
    cross-language CSV truth-table can compare the two surfaces row-for-row.

    * ``LINE_FOLLOW_INTERSECTION`` -- LINE edge into a junction (line) node:
      follow to the crossing marker, then roll the LINE_OVERSHOOT.
    * ``PROJECTION_FOLLOW`` -- LINE edge into a projection node: distance-gated
      line follow (no marker beneath the strip).
    * ``STOP_AT_LINE_SEEK`` -- BLIND edge resolved to a ``Drive.stopAtLine`` seek.
    * ``BLIND_DEAD_RECKON`` -- BLIND edge with no seek: dead-reckoned drive.
    """

    LINE_FOLLOW_INTERSECTION = 0
    PROJECTION_FOLLOW        = 1
    STOP_AT_LINE_SEEK        = 2
    BLIND_DEAD_RECKON        = 3


class ReanchorKind(IntEnum):
    """How localization re-anchors after the hop's motion completes.

    Mirror of the C++ ``enum class ReanchorKind : uint8_t`` in
    ``WRO2026/EdgeDispatchPolicy.h``.

    * ``ON_INTERSECTION`` -- the chassis stops on a physical crossing marker, so
      localization snaps to the destination node
      (:meth:`LocalizationSim.on_intersection`).
    * ``ON_DEAD_RECKONED_ADVANCE`` -- the chassis dead-reckons into the node via
      the encoder gate (:meth:`LocalizationSim.on_dead_reckoned_advance`).
    """

    ON_INTERSECTION          = 0
    ON_DEAD_RECKONED_ADVANCE = 1


@dataclass
class EdgeDispatchQuery:
    """Already-resolved inputs to the terminal one-hop dispatch classifier.

    Function-identical Python mirror of the C++ ``struct EdgeDispatchQuery`` in
    ``WRO2026/EdgeDispatchPolicy.h``. Every field is resolved by the FSM
    (:meth:`PathTrackerSim._walk_hops`) BEFORE the call -- the classifier owns NO
    cross-hop state (no pivot latch, no direction resolution, no pass-through
    lookahead); it only composes the existing pure policies
    (:func:`resolve_pivot_trim`, :func:`resolve_sensor_overshoot_trim`) and the
    trim-clamp arithmetic.

    :ivar edge_type: :data:`EDGE_LINE` or :data:`EDGE_BLIND`.
    :ivar dest_is_projection: True iff the destination node is a projection node
        (the 2-state flag mirroring the C++ ``NodeType`` LINE|PROJECTION split;
        only those two node types are edge-routed).
    :ivar direction: the resolved travel direction (``FRONT``/``REAR``; never
        ``AUTO``).
    :ivar raw_distance_cm: the directed edge length (``edge.distance_cm``).
    :ivar line_mode: the resolved :class:`SpeedTier` for the edge (carried to
        mirror the C++ struct; the decision logic does not read it). The field
        keeps its ``line_mode`` name so the cross-language dispatch truth-table
        fixture stays byte-identical; ADR-064 renames only the C++ struct field.
    :ivar stop_at_line: the ``(seek, motion_mode)`` from
        :func:`resolve_stop_at_line` -- read only for BLIND edges.
    :ivar proj_pass_through: FSM pass-through lookahead flag (LINE->proj->line
        collapse); only meaningful on a LINE-into-projection hop.
    :ivar departing_trim_cm: outgoing trim of the pivot being LEFT, or ``0.0``.
    :ivar approaching_trim_cm: approach trim of the pivot being ENTERED, or
        ``0.0``.
    :ivar axle_offset_cm: direction-aware sensor-to-axle offset
        (``REAR`` -> :data:`LINE_SENSOR_AXLE_OFFSET_REAR_CM`, ``FRONT`` ->
        :data:`LINE_SENSOR_AXLE_OFFSET_CM`).
    :ivar min_drive_cm: :data:`PIVOT_TRIM_MIN_DRIVE_CM` underflow floor.
    """

    edge_type: str
    dest_is_projection: bool
    direction: "Direction"
    raw_distance_cm: float
    line_mode: SpeedTier
    stop_at_line: Tuple[bool, BlindMode]
    proj_pass_through: bool
    departing_trim_cm: float
    approaching_trim_cm: float
    axle_offset_cm: float
    min_drive_cm: float


@dataclass
class EdgeDispatchDecision:
    """Terminal one-hop dispatch classification (the classifier output).

    Function-identical Python mirror of the C++ ``struct EdgeDispatchDecision``
    in ``WRO2026/EdgeDispatchPolicy.h``. Exactly seven fields -- the classifier
    returns a CLASSIFICATION, not a full descriptor.

    :ivar primitive: the :class:`DispatchPrimitive` this hop dispatches to.
    :ivar reverse: ``direction == Direction.REAR``.
    :ivar drive_mode: motion bundle for the blind primitives
        (``stop_at_line``'s ``motion_mode``); :attr:`BlindMode.DEFAULT` (unused)
        for the line primitives.
    :ivar commanded_distance_cm: the POST-clamp COMMANDED value, NOT the physical
        displacement -- the full ``raw_distance_cm`` for a sensor-terminated seek,
        the trimmed distance for a dead-reckon/projection hop. The physical trim
        lives in the pose integration only (memory:
        ``gotcha-sim-primitive-payload-is-command-not-displacement``).
    :ivar overshoot_cm: the post-detect axle roll for the seek overshoot channel
        (``0.0`` for the non-seek primitives).
    :ivar reanchor: the :class:`ReanchorKind` localization applies after motion.
    :ivar brake_on_gate: ``False`` only on a projection pass-through; ``True``
        otherwise.
    """

    primitive: DispatchPrimitive
    reverse: bool
    drive_mode: BlindMode
    commanded_distance_cm: float
    overshoot_cm: float
    reanchor: ReanchorKind
    brake_on_gate: bool


def resolve_edge_dispatch(q: EdgeDispatchQuery) -> EdgeDispatchDecision:
    """Classify one already-resolved edge into its terminal dispatch decision.

    Function-identical Python twin of the C++
    ``EdgeDispatchPolicy.h::resolveEdgeDispatch``. Both implementations consume
    the SAME frozen field set (the :class:`EdgeDispatchQuery` / the C++
    ``EdgeDispatchQuery``) and emit the SAME seven-field decision, so the offline
    sim and the robot agree on every hop's primitive, direction, commanded
    distance, overshoot, re-anchor kind and gate-brake flag. The two surfaces
    conform to the spec contract -- not to each other -- and a standalone
    cross-language CSV truth-table is the drift guard (mirroring how
    :func:`resolve_stop_at_line` / :func:`resolve_pivot_trim` mirror their C++
    policy headers).

    Cross-language parity: keep step-for-step identical with
    ``WRO2026/EdgeDispatchPolicy.h::resolveEdgeDispatch``.

    The classifier owns NO cross-hop state: direction resolution, the pivot
    3-latch carry, the pass-through lookahead, the GYRO heading target and the
    velocity-profile split all stay FSM-side and arrive pre-resolved on @p q. It
    COMPOSES the existing pure policies and owns only the trim-clamp arithmetic:
    a dead-reckon / projection hop's ``commanded_distance_cm`` is
    ``raw_distance_cm - departing_trim_cm - approaching_trim_cm`` clamped to
    ``min_drive_cm`` (the post-clamp value is carried). The
    ``PIVOT_TRIM_UNDERFLOW`` fault is intentionally NOT a decision field -- the
    sim does not model the fault FSM (ADR-033), so the resolver just clamps and
    the firmware raises the fault.

    Payload rule (memory-critical,
    ``gotcha-sim-primitive-payload-is-command-not-displacement``):
    ``commanded_distance_cm`` is the COMMANDED value (full ``raw_distance_cm``
    for a sensor-terminated seek -- the crossing terminates it), never the
    trimmed physical displacement.

    :param q: the already-resolved :class:`EdgeDispatchQuery` for this hop.
    :return: the :class:`EdgeDispatchDecision` per the frozen contract.
    """
    reverse = q.direction == Direction.REAR

    if q.edge_type == EDGE_LINE:
        if q.dest_is_projection:
            # LINE -> projection node: distance-gated line follow. The commanded
            # distance is the two-trim subtract-and-clamp (no junction marker, so
            # the encoder gate lands the chassis). brakeOnGate is suppressed only
            # when the FSM collapses a line-proj-line pass-through.
            primitive = DispatchPrimitive.PROJECTION_FOLLOW
            commanded_distance_cm = resolve_pivot_trim(
                q.raw_distance_cm,
                q.departing_trim_cm,
                q.approaching_trim_cm,
                q.min_drive_cm,
            )[0]
            overshoot_cm = 0.0
            reanchor = ReanchorKind.ON_DEAD_RECKONED_ADVANCE
            brake_on_gate = not q.proj_pass_through
            drive_mode = BlindMode.DEFAULT
        else:
            # LINE -> junction (line) node: sensor-terminated follow to the
            # crossing marker. The LINE_OVERSHOOT advance is NOT owned here -- it
            # stays an FSM concern (resolve_sensor_overshoot_trim in _walk_hops),
            # so this classifier reports a zero overshoot and a zero commanded
            # distance (the crossing, not a distance, terminates the follow).
            primitive = DispatchPrimitive.LINE_FOLLOW_INTERSECTION
            commanded_distance_cm = 0.0
            overshoot_cm = 0.0
            reanchor = ReanchorKind.ON_INTERSECTION
            brake_on_gate = True
            drive_mode = BlindMode.DEFAULT
    else:    # EDGE_BLIND
        seek, motion_mode = q.stop_at_line
        if seek:
            # BLIND seek: the crossing terminates the drive, so the commanded
            # distance stays the FULL raw command (NOT the trimmed displacement);
            # the approaching-pivot trim folds into the separate overshoot
            # channel via the shared sensor-overshoot twin.
            primitive = DispatchPrimitive.STOP_AT_LINE_SEEK
            commanded_distance_cm = q.raw_distance_cm
            overshoot_cm = resolve_sensor_overshoot_trim(
                q.axle_offset_cm,
                q.approaching_trim_cm,
                q.min_drive_cm,
            )[0]
            reanchor = ReanchorKind.ON_INTERSECTION
            brake_on_gate = True
            drive_mode = motion_mode
        else:
            # BLIND dead-reckon: distance-terminated, so the commanded distance is
            # the two-trim subtract-and-clamp (the same core the projection hop
            # uses).
            primitive = DispatchPrimitive.BLIND_DEAD_RECKON
            commanded_distance_cm = resolve_pivot_trim(
                q.raw_distance_cm,
                q.departing_trim_cm,
                q.approaching_trim_cm,
                q.min_drive_cm,
            )[0]
            overshoot_cm = 0.0
            reanchor = ReanchorKind.ON_DEAD_RECKONED_ADVANCE
            brake_on_gate = True
            drive_mode = motion_mode

    return EdgeDispatchDecision(
        primitive=primitive,
        reverse=reverse,
        drive_mode=drive_mode,
        commanded_distance_cm=commanded_distance_cm,
        overshoot_cm=overshoot_cm,
        reanchor=reanchor,
        brake_on_gate=brake_on_gate,
    )


def _read_edge_override(edge, attr: str, enum_cls):
    """Coerce a graph edge override attribute to a RAW member of ``enum_cls``.

    Shared coercion for the per-edge override readers. Unlike
    :func:`_resolve_override`, this does NOT collapse ``DEFAULT`` into a
    pipeline value -- it returns the literal enum member (a real member or
    ``enum_cls.DEFAULT``). The caller decides what ``DEFAULT`` means.

    Tolerates absence (returns ``enum_cls.DEFAULT``), already-typed
    ``enum_cls`` instances (returned as-is), and raw int values (coerced via
    the enum constructor).

    Cross-enum int-coercion hazard: the int branch maps an int back to an
    ``enum_cls`` member via ``enum_cls(raw)``. This is latent-safe ONLY
    because the field_data builder bakes exactly the real enum members of
    this category (never an arbitrary or cross-enum int) -- an out-of-range
    int would mis-map (or raise ``ValueError``). Keep the builder the single
    source of these attribute values.
    """
    raw = edge.get(attr) if hasattr(edge, "get") else None
    if raw is None:
        return enum_cls.DEFAULT
    if isinstance(raw, enum_cls):
        return raw
    return enum_cls(raw)


def _read_edge_dir_override(edge) -> DirectionMode:
    """Coerce the graph edge's ``direction_override`` attribute to
    :class:`DirectionMode`.

    Tolerates absence (returns ``DirectionMode.DEFAULT``), raw int values
    (coerced via the enum constructor), and already-typed
    :class:`DirectionMode` instances (returned as-is). Thin wrapper over
    :func:`_read_edge_override` (which preserves ``DEFAULT``, unlike
    :func:`_resolve_override`).
    """
    return _read_edge_override(edge, "direction_override", DirectionMode)


def _read_edge_stop_at_line_override(edge) -> StopAtLineMode:
    """Coerce the graph edge's ``stop_at_line_override`` attribute to
    :class:`StopAtLineMode`.

    Tolerates absence (returns ``StopAtLineMode.DEFAULT``), raw int values
    (coerced via the enum constructor), and already-typed
    :class:`StopAtLineMode` instances (returned as-is). Thin wrapper over
    :func:`_read_edge_override` (which preserves ``DEFAULT``). The graph
    builder (T5) bakes this attribute as a :class:`StopAtLineMode` member;
    absence means no per-edge override (defer to the resolver's
    lower-precedence rungs).
    """
    return _read_edge_override(edge, "stop_at_line_override", StopAtLineMode)


def _body_heading_for(direction: "Direction", edge_heading: float) -> float:
    """Return the body heading the chassis assumes for one hop.

    For ``FRONT`` the body faces along the edge; for ``REAR`` the body
    faces opposite the edge so the rear sensor leads. Mirrors the
    ``(edge.heading_deg + 180.0) % 360.0`` computation in
    ``PathTracker::_onPlanNextHop`` (T6).
    """
    if direction == Direction.REAR:
        return _normalize(edge_heading + 180.0)
    return _normalize(edge_heading)


def _pass_through_heading_eligible(
    outbound_edge,
    outbound_is_last_hop: bool,
    *,
    inbound_body_heading: float,
    inbound_direction: "Direction",
    pipeline_default: DirectionMode = None,
    builder_route: "Direction" = None,
    builder_arrival: "Direction" = None,
    bidirectional_enabled: bool = None,
    line_board_count: int = None,
) -> bool:
    """Pure ADR-023/ADR-059 collinearity + same-direction eligibility predicate.

    Function-identical Python mirror of the firmware
    ``PathTracker::passThroughHeadingEligible`` (ADR-059). Every input is
    EXPLICIT — no FSM/sim state is read — so the blind-chain walk-ahead
    (:meth:`PathTrackerSim._try_start_blind_chain`) can call it repeatedly for
    multi-hop lookahead with SYNTHESIZED inbound body headings, and the two
    pass-through gates (:func:`_should_pass_through` for the line-line-line
    collapse, :func:`_should_projection_pass_through` for the line-proj-line
    one) share this one predicate so the heading math cannot drift between the
    three call sites (mirrors the firmware's one-predicate / three-gates
    structure).

    Resolves the outbound direction with @p inbound_body_heading as the current
    heading, rotates the outbound edge's forward heading +180° when the resolved
    direction is REAR (the body heading the rear-led chassis assumes), wraps the
    delta into ``(-180, +180]``, and is eligible iff
    ``|delta| <= NAV_TURN_TOLERANCE_DEG`` AND the resolved outbound direction
    equals @p inbound_direction (a FRONT↔REAR flip is never a continuation — it
    would force a mid-flight board swap / blind-chain sign flip).

    :param outbound_edge: the graph edge attribute mapping for the candidate
        outbound hop (must carry ``heading_deg`` + optional ``direction_override``).
    :param outbound_is_last_hop: True iff the outbound hop terminates at the
        route's final destination (gates the builder arrival override).
    :param inbound_body_heading: the body heading the chassis holds as it reaches
        the joint (for FRONT the inbound edge heading; for REAR that +180°).
    :param inbound_direction: the resolved travel direction of the inbound hop.
    :param pipeline_default: optional pipeline-level direction default; ``None``
        resolves to :data:`PIPELINE_CONFIG.direction`.
    :param builder_route: optional builder route override; ``None`` -> AUTO.
    :param builder_arrival: optional builder arrival override; ``None`` -> AUTO.
    :param bidirectional_enabled: optional kill-switch override; ``None`` ->
        :data:`BIDIRECTIONAL_AUTO_ENABLED`.
    :param line_board_count: optional board count override; ``None`` ->
        :data:`LINE_BOARD_COUNT`.
    :return: True iff the outbound hop is collinear + same-direction with the
        inbound one (a pass-through / chain continuation is eligible).
    """
    outbound_heading = float(outbound_edge["heading_deg"])

    resolved_pipeline = (
        pipeline_default
        if pipeline_default is not None
        else _pipeline_config.PIPELINE_CONFIG.direction
    )
    resolved_route = builder_route if builder_route is not None else Direction.AUTO
    resolved_arrival = (
        builder_arrival if builder_arrival is not None else Direction.AUTO
    )
    resolved_enabled = (
        bidirectional_enabled if bidirectional_enabled is not None else BIDIRECTIONAL_AUTO_ENABLED
    )
    resolved_board_count = (
        line_board_count if line_board_count is not None else LINE_BOARD_COUNT
    )

    # Resolve the outbound hop's direction using the inbound body heading as
    # `current_heading` (the critical body-heading peek: a mixed REAR->FORWARD
    # transition across anti-parallel forward edges has a raw-edge delta of 180°
    # but a body-heading delta of ~0).
    next_dir = resolve_direction(
        edge_dir_override=_read_edge_dir_override(outbound_edge),
        pipeline_default=resolved_pipeline,
        builder_arrival=resolved_arrival,
        builder_route=resolved_route,
        current_heading=inbound_body_heading,
        edge_heading=outbound_heading,
        is_last_hop=outbound_is_last_hop,
        bidirectional_enabled=resolved_enabled,
        line_board_count=resolved_board_count,
    )
    next_body_heading = _body_heading_for(next_dir, outbound_heading)

    heading_delta = wrap_pm180(next_body_heading - inbound_body_heading)
    if abs(heading_delta) > NAV_TURN_TOLERANCE_DEG:
        return False
    # Pass-through requires matching direction across the joint. A REAR->FRONT
    # or FRONT->REAR transition would force a mid-flight LineFollower.setReverse
    # flip (or a blind-chain sign flip), never a clean continuation. Mirror of
    # the firmware guard in `passThroughHeadingEligible`.
    return next_dir == inbound_direction


def _should_pass_through(
    graph,
    next_hop,
    intersected_node,
    goal_name,
    *,
    inbound_body_heading: float,
    inbound_direction: "Direction",
    pipeline_default: DirectionMode = None,
    builder_route: "Direction" = None,
    builder_arrival: "Direction" = None,
    bidirectional_enabled: bool = None,
    line_board_count: int = None,
) -> bool:
    """Return True iff a LINE-edge intersection at ``intersected_node`` should
    emit ``ACT_LINE_PASS`` instead of ``ACT_LINE_OVERSHOOT``.

    Conditions (all must hold):

    * :data:`PATH_PASS_THROUGH_ENABLED` is True.
    * ``intersected_node`` is not the goal (final destination never passes through).
    * A next-next hop exists from ``intersected_node`` toward ``goal_name``
      (non-``None``, non-self).
    * The next-next edge's *body* heading (resolved via
      :func:`resolve_direction` against ``inbound_body_heading``),
      wrap-normalized against the inbound body heading, has absolute value
      <= :data:`NAV_TURN_TOLERANCE_DEG`.

    The peek MUST use resolved body-headings, not raw forward edge
    headings. Without this, mixed REAR <-> FORWARD transitions across
    anti-parallel forward edges (TC9c) would silently skip pass-through
    even though the body never actually turned.

    Note: ``inbound_body_heading`` and all subsequent parameters are
    keyword-only. The previous signature placed ``inbound_body_heading``
    positionally between ``goal_name`` and the kwargs, which made it easy
    for external callers using the pre-bidirectional positional shape
    (``graph, next_hop, current_edge, intersected_node, goal_name``) to
    silently mis-bind. The ``*`` marker promotes any such mistake to a
    loud ``TypeError`` at the call site.

    :param inbound_body_heading: the body heading of the chassis as it
        crosses ``intersected_node`` — for FRONT this is the inbound
        edge's heading; for REAR it is that heading + 180° (mod 360).
    :param pipeline_default: optional pipeline-level direction default.
        ``None`` resolves to :data:`PIPELINE_CONFIG.direction`. Tests
        and the FSM pass it explicitly when they have a value in scope.
    :param builder_route: optional builder route override; ``None``
        resolves to ``Direction.AUTO``.
    :param builder_arrival: optional builder arrival override; ``None``
        resolves to ``Direction.AUTO``. The next-next hop is the last
        hop iff ``next_next == goal_name``.
    :param bidirectional_enabled: optional module-level kill switch
        override; ``None`` resolves to :data:`BIDIRECTIONAL_AUTO_ENABLED`.
    :param line_board_count: optional board count override; ``None``
        resolves to :data:`LINE_BOARD_COUNT`.
    """
    if not PATH_PASS_THROUGH_ENABLED:
        return False
    if intersected_node == goal_name:
        return False
    next_next = next_hop[intersected_node][goal_name]
    if next_next is None or next_next == intersected_node:
        return False
    next_edge = graph.edges[intersected_node, next_next]

    # Delegate the heading/direction MATH to the shared pure predicate (mirror
    # of the firmware's `_passThroughHeadingOk` -> `passThroughHeadingEligible`
    # adapter), so the intersection-gated line-line-line collapse, the
    # distance-gated line-proj-line collapse, and the ADR-059 blind-chain walk
    # all resolve collinearity through ONE function.
    return _pass_through_heading_eligible(
        next_edge,
        outbound_is_last_hop=(next_next == goal_name),
        inbound_body_heading=inbound_body_heading,
        inbound_direction=inbound_direction,
        pipeline_default=pipeline_default,
        builder_route=builder_route,
        builder_arrival=builder_arrival,
        bidirectional_enabled=bidirectional_enabled,
        line_board_count=line_board_count,
    )


def _should_projection_pass_through(
    graph,
    next_hop,
    projection_node,
    goal_name,
    *,
    inbound_body_heading: float,
    inbound_direction: "Direction",
    pipeline_default: DirectionMode = None,
    builder_route: "Direction" = None,
    builder_arrival: "Direction" = None,
    bidirectional_enabled: bool = None,
    line_board_count: int = None,
) -> bool:
    """Return True iff a `line(A)–projection(B)–line(C)` triple collapses to one
    continuous A→C follow at the marker-less projection node ``B``.

    Function-identical Python mirror of the firmware
    ``PathTracker::_evaluateProjectionPassThrough`` (T19, issue #64). Called on
    the A→B LINE-into-projection hop (``B == projection_node``,
    ``inbound_body_heading`` / ``inbound_direction`` are the A→B hop's resolved
    body heading + travel direction). It decides WHETHER the distance-gated
    follow into ``B`` keeps the follower live and continues to ``C`` (no stop),
    emitting :data:`ACT_FOLLOW_DIST_PASS` instead of :data:`ACT_FOLLOW_DIST`. It
    does NOT change which gate fires the state transition -- that is ALWAYS the
    A→B DISTANCE gate (a projection node has no junction marker, so an
    intersection never fires at ``B``). This is ROUTING LOOKAHEAD ONLY.

    Every condition must hold; any miss ⇒ a normal stop at ``B`` (the per-hop
    fallback, byte-identical to the pre-T20 ``ACT_FOLLOW_DIST`` stream):

    1. The shared ADR-023 collinearity / direction / lookahead gate
       (:func:`_should_pass_through`) passes: :data:`PATH_PASS_THROUGH_ENABLED`
       is set, ``B`` is NOT the goal (so a ``goTo(B)`` stops at B -- the
       firmware's not-final ``_nextHop == _destinationNode`` guard), a real
       next-next hop ``C`` exists (non-``None``, non-self), A→B→C are collinear
       (body-heading delta <= :data:`NAV_TURN_TOLERANCE_DEG`), and the resolved
       direction matches across ``B`` (no mid-flight FRONT↔REAR board swap).
    2. The outbound edge ``B→C`` is a :data:`EDGE_LINE` edge (both flanks LINE).
       The inbound A→B is LINE by construction (this helper is only reached on
       the LINE→projection dispatch path). A BLIND flank ⇒ a normal stop at B.
    3. ``C`` is a LINE (junction) node, so the B→C hop re-anchors on a real
       intersection marker (the firmware's ``getNodeType(c) == NodeType::LINE``).
       A projection ``C`` (B→C would itself be a distance-gated follow with no
       re-anchor) or any other type is rejected -- out of scope for the
       line-proj-line collapse.

    The shared :func:`_should_pass_through` owns the (1) checks, so the ADR-023
    heading logic physically cannot drift between the intersection-gated
    line-line-line pass-through and this distance-gated line-proj-line one. The
    (2)/(3) flank + C-type guards are the projection-specific additions the
    firmware applies on top.

    :param projection_node: the middle node ``B`` (a projection node by the
        caller's :func:`is_projection_node` check on this LINE hop's
        destination).
    :param inbound_body_heading: the body heading of the chassis as it
        dead-reckons into ``B`` (FRONT: the A→B edge heading; REAR: +180°).
    :param inbound_direction: the resolved travel direction of the A→B hop.
    All remaining parameters carry the same meaning as in
    :func:`_should_pass_through` (its keyword-only resolution context).
    """
    # (2)/(3) FIRST: read the projection-specific flank + C-type guards, then
    # delegate the shared ADR-023 lookahead/collinearity to _should_pass_through.
    # The order is immaterial to the result (all conditions must hold) but
    # surfacing the LINE-flank / LINE-C guards here keeps the projection-specific
    # logic in one place and leaves the shared heading check untouched.
    if not PATH_PASS_THROUGH_ENABLED:
        return False
    if projection_node == goal_name:
        return False
    c = next_hop[projection_node][goal_name]
    if c is None or c == projection_node:
        return False

    # (2) The outbound edge B→C must be a LINE edge (both flanks LINE).
    outbound = graph.edges[projection_node, c]
    if outbound["type"] != EDGE_LINE:
        return False

    # (3) C must be a LINE (junction) node so the B→C hop re-anchors on a real
    # intersection marker. Read the node type LOCALLY off the graph, mirroring
    # the firmware's `getNodeType(c) == NodeType::LINE`.
    if graph.nodes[c].get("type") != "line":
        return False

    # (1) The shared ADR-023 collinearity / direction / lookahead gate -- the
    # SAME helper the intersection-gated line-line-line pass-through uses, so the
    # heading math never drifts between the two collapses.
    return _should_pass_through(
        graph,
        next_hop,
        projection_node,
        goal_name,
        inbound_body_heading=inbound_body_heading,
        inbound_direction=inbound_direction,
        pipeline_default=pipeline_default,
        builder_route=builder_route,
        builder_arrival=builder_arrival,
        bidirectional_enabled=bidirectional_enabled,
        line_board_count=line_board_count,
    )


class LocalizationSim:
    """Mirror of ``LocalizationClass`` (current node + operator offset).

    Holds the discrete graph position (``current_node``) plus the PR1b
    operator-offset state: a 2D :class:`Offset2D` accumulated from raw,
    non-``go_to`` operator moves while the nav-active gate is open, and the
    ``nav_active`` flag PathTracker pushes down. The integrator
    (:meth:`accumulate_operator_move`) mirrors
    ``LocalizationClass::_integrateOffset`` -- the two channels (translation /
    rotation) gate INDEPENDENTLY on :data:`OFFSET_TRANSLATION_DEADBAND_CM` /
    :data:`OFFSET_ROTATION_DEADBAND_DEG`, and the gate only opens when
    ``!nav_active and is_zeroed()`` (mirroring the firmware's
    ``(!_navActive) && _zeroed``).

    The sim does not model encoder pulses or IMU drift, so unlike the firmware
    there is no per-tick baseline re-snapshot to "kill the arrival-tick
    phantom" -- :meth:`clear_offset` just zeroes the accumulated offset. The
    node-arrival hooks call it, matching the firmware's ``clearOffset()`` from
    ``onIntersection`` / ``onDeadReckonedAdvance``.
    """

    def __init__(self, field_data) -> None:
        self._current_node = field_data.START_NODE
        self._zeroed = False
        self._field_data = field_data
        # PR1b operator-offset state (mirror of LocalizationClass members).
        self._offset = Offset2D()
        self._nav_active = False
        # Sub-deadband carry residuals (mirror of the firmware's UNADVANCED encoder /
        # yaw baselines). `_integrateOffset` leaves the baseline un-advanced when a tick's
        # delta is below the deadband, so slow sub-deadband motion accumulates across ticks
        # until it crosses the floor, then flushes in full. The sim integrates explicit
        # operator moves rather than per-tick encoder deltas, so it carries the un-applied
        # amount here instead of an un-advanced baseline (#5). Both reset on re-anchor.
        self._pending_translation_cm = 0.0
        self._pending_rotation_deg = 0.0
        # Node-type lookup so the return-to-node dispatch can pick the LINE vs
        # BLIND re-anchor path (mirrors FieldMap::getNodeType). Absent on the
        # minimal synthetic fixtures -- default to an empty map; the consumer
        # (`node_type`) then reports the safe LINE default (the firmware's
        # noNode() sentinel type is also LINE).
        self._node_types: Dict[str, str] = dict(
            getattr(field_data, "NODE_TYPES", {}) or {}
        )
        # Bench-fidelity guard (#9): when a fixture DOES author NODE_TYPES, every graph node
        # must have an entry. The firmware's getNodeType returns the authored type for every
        # valid node (only out-of-range ids fall back to LINE), so a node present in the graph
        # but missing from NODE_TYPES would silently default to "line" here while the firmware
        # used its real (possibly BLIND) type -- a false-green parity for that node. A fixture
        # with NO NODE_TYPES at all is exempt (the minimal synthetic graphs that never exercise
        # the return dispatch). The real validated field data always has full coverage.
        if self._node_types:
            missing = [n for n in field_data.NODES if n not in self._node_types]
            if missing:
                raise ValueError(
                    "NODE_TYPES is missing entries for graph node(s) "
                    f"{sorted(missing)}; every node must have a type or the "
                    "return-to-node dispatch silently defaults them to LINE "
                    "(bench-fidelity gap vs FieldMap::getNodeType)."
                )
        # Sparse wall-zero reset table keyed by node name (mirror of the
        # firmware's ``Routes::RESET_NODES`` / ``FieldMap::getResetInfo``). Each
        # value is a ``{axis, wall_coord, facing}`` spec. Absent on a reset-free
        # field -- default to an empty map; the consumer (`reset_info`) then
        # reports ``None`` and the interlude no-ops (PR2 AC5 "no behavior change
        # when not applicable").
        self._reset_nodes: Dict[str, Dict] = dict(
            getattr(field_data, "RESET_NODES", {}) or {}
        )

    def zero_heading_from_current(self) -> bool:
        self._zeroed = True
        return True

    def is_zeroed(self) -> bool:
        return self._zeroed

    def current_node(self) -> str:
        return self._current_node

    def node_type(self, node: str) -> str:
        """Return the lowercase node-type string (``"line"`` / ``"blind"``).

        Mirror of ``FieldMap::getNodeType`` for the return-to-node dispatch.
        An unknown node (or a fixture with no ``NODE_TYPES``) reports
        ``"line"`` -- the same safe default the firmware's ``noNode()``
        sentinel carries (``NodeType::LINE``), so a missing entry never
        accidentally routes through the blind dead-reckon return.

        :param node: node name to look up.
        :return: ``"line"`` or ``"blind"`` (lowercased).
        """
        return str(self._node_types.get(node, "line")).lower()

    def is_line_backed(self, node: str) -> bool:
        """Return ``True`` iff @p node is a LINE-backed anchor.

        Mirrors ``FieldMap::getNodeType(cur) == Routes::NodeType::LINE`` in
        ``PathTracker::_dispatchReturnMove``: a LINE node re-acquires its line
        via ``Drive.stopAtLine``; anything else dead-reckons back.
        """
        return self.node_type(node) == "line"

    def is_reset_node(self, node: str) -> bool:
        """Return ``True`` iff @p node is a wall-zero reset node.

        Mirrors ``FieldMap::getNodeType(cur) == Routes::NodeType::RESET`` in
        ``PathTracker::_maybeEnterReset``. A node is a reset node iff it is typed
        ``"reset"`` AND carries a sparse ``RESET_NODES`` entry -- both must agree
        (the validator enforces this both directions). A node typed ``"reset"``
        with no spec is treated as a non-reset node here (the interlude no-ops),
        mirroring the firmware's defensive ``getResetInfo == nullptr`` skip.

        :param node: node name to look up.
        :return: ``True`` iff the interlude should fire on arrival at @p node.
        """
        return self.node_type(node) == "reset" and node in self._reset_nodes

    def reset_info(self, node: str) -> Optional[Dict]:
        """Return the ``{axis, wall_coord, facing}`` reset spec for @p node, or None.

        Mirror of ``FieldMap::getResetInfo``: a node with no sparse
        ``RESET_NODES`` entry reports ``None`` (the interlude no-ops). The
        returned dict is the live spec; callers read it but never mutate it.

        :param node: node name to look up.
        :return: the reset spec dict if @p node carries one, else ``None``.
        """
        return self._reset_nodes.get(node)

    def reanchor_at_wall(self, axis: str, wall_coord: float, facing: float) -> None:
        """Re-anchor the believed on-axis coordinate to the wheel axle; square heading.

        Sim mirror of ``LocalizationClass::reanchorAtWall``. The firmware tracks
        position as ``node + offset`` and sets ``_offset[axis] = (wall_coord -
        sign(gap)*WALL_CONTACT_OFFSET_CM) - nodeCoord[axis]`` (``gap = wall_coord
        - nodeCoord[axis]``), so the net BELIEF on ``axis`` becomes the wheel AXLE
        -- ``wall_coord - sign(gap)*WALL_CONTACT_OFFSET_CM``, on the NODE side of
        the wall surface -- then re-binds heading to @p facing. (The subsequent
        WALL_DEPART move drives this margin forward, returning the belief to the
        node; see :meth:`PathTrackerSim._dispatch_wall_reset`.) The sim tracks the
        chassis pose directly (no node + offset split), so the LocalizationSim
        analogue clears the operator-offset frame (the re-anchor zeroes it, just
        like ``on_intersection``) -- the chassis-pose snaps (axle on REANCHOR,
        then node on WALL_DEPART) are applied by
        :meth:`PathTrackerSim._dispatch_wall_reset`, which owns the chassis. This
        method keeps the LocalizationSim offset state consistent (cleared) with a
        node arrival.

        :param axis: ``"x"`` or ``"y"`` -- the wall is parallel to this field
            axis; the orthogonal coordinate is re-zeroed.
        :param wall_coord: the re-zeroed coordinate on @p axis (cm).
        :param facing: the squared heading (deg, world frame, ``[0, 360)``).
        """
        # The wall re-anchor lands on (and re-zeroes against) the current node,
        # exactly like a sensed intersection: clear the operator-offset frame.
        # The believed on-axis coordinate + heading live on the chassis pose,
        # snapped by the PathTrackerSim dispatch that calls this.
        self.clear_offset()

    def set_nav_active(self, active: bool) -> None:
        """Push PathTracker's hop activity DOWN to the integrator gate.

        Mirror of ``LocalizationClass::setNavActive``. The integrator's
        translation and rotation channels both gate on ``!nav_active``, so
        motion that PathTracker itself commands never grows the operator
        offset. PathTracker sets this ``True`` while executing a hop and
        ``False`` on idle / done / fault.

        :param active: ``True`` while a hop is executing; ``False`` otherwise.
        """
        self._nav_active = bool(active)

    def get_offset_from_node(self) -> Offset2D:
        """Read the accumulated pose offset from the last anchored node.

        Pure accessor (mirror of ``LocalizationClass::getOffsetFromNode``):
        returns a COPY so a caller cannot mutate the live integrator state.

        :return: a copy of the accumulated :class:`Offset2D` (all-zero while
            anchored on a node).
        """
        return Offset2D(self._offset.x, self._offset.y, self._offset.theta_deg)

    def clear_offset(self) -> None:
        """Zero the accumulated offset (re-anchor on the current node).

        Mirror of ``LocalizationClass::clearOffset`` / ``_consumeOffset``. The
        firmware also re-snapshots the encoder/yaw baselines to kill the
        arrival-tick phantom; the sim integrates explicit operator moves rather
        than per-tick encoder deltas, so zeroing the offset AND the sub-deadband
        carry residuals is the equivalent re-anchor (the firmware's baseline
        re-snapshot discards any un-flushed sub-deadband motion too).
        """
        self._offset = Offset2D()
        self._pending_translation_cm = 0.0
        self._pending_rotation_deg = 0.0

    def accumulate_operator_move(
        self,
        distance_cm: float,
        heading_deg: float,
        theta_delta_deg: float = 0.0,
    ) -> None:
        """Integrate one raw, non-``go_to`` operator move into the offset.

        Sim mirror of ``LocalizationClass::_integrateOffset`` collapsed to a
        single atomic move (the sim does not tick): a raw operator
        ``Drive.driveDist`` of @p distance_cm along world heading
        @p heading_deg, optionally with a raw ``Drive.turnDeg`` of
        @p theta_delta_deg. The two channels gate INDEPENDENTLY:

        * **Translation channel** -- add @p distance_cm to the pending carry; once
          ``abs(pending) >= OFFSET_TRANSLATION_DEADBAND_CM`` flush the WHOLE pending
          amount, decomposed along @p heading_deg into ``x`` / ``y``, and reset the
          carry. A sub-deadband move is NOT dropped -- it accumulates across calls
          until the carry crosses the floor, mirroring the firmware leaving the
          encoder baseline UNADVANCED below the deadband (#5). The flush decomposes
          along the heading of the call that crosses the floor, exactly as the
          firmware decomposes the accumulated encoder delta along the current heading.
        * **Rotation channel** -- add @p theta_delta_deg to the pending rotation
          carry; flush the whole carry into ``theta_deg`` once
          ``abs(pending) >= OFFSET_ROTATION_DEADBAND_DEG``. A pure manual turn nets
          ~zero translation but a real yaw change, so the channels are independent.

        The whole integration is gated on ``!nav_active and is_zeroed()``
        (mirror of the firmware's ``(!_navActive) && _zeroed``): a move
        commanded while PathTracker is navigating, or before the world frame is
        bound, accumulates nothing.

        :param distance_cm: signed net translation of the move (cm). Positive
            is forward along @p heading_deg.
        :param heading_deg: world-frame heading the move ran along (deg, CCW,
            ``0 = +x``). Used to decompose @p distance_cm into ``x`` / ``y``.
        :param theta_delta_deg: signed IMU-yaw change of the move (deg). A
            pure translation passes ``0`` (the default).
        """
        # Gate: only accumulate when PathTracker is NOT driving AND the world
        # frame is bound. Mirrors `update()`'s `(!_navActive) && _zeroed`.
        if self._nav_active or not self._zeroed:
            return

        # Translation channel (independent deadband, with sub-deadband carry). Accumulate
        # into the pending residual; flush the WHOLE residual once it crosses the floor,
        # then reset it -- mirroring the firmware advancing the encoder baseline to "now"
        # only when the accumulated delta crosses OFFSET_TRANSLATION_DEADBAND_CM.
        self._pending_translation_cm += distance_cm
        if abs(self._pending_translation_cm) >= OFFSET_TRANSLATION_DEADBAND_CM:
            rad = math.radians(heading_deg)
            self._offset.x += self._pending_translation_cm * math.cos(rad)
            self._offset.y += self._pending_translation_cm * math.sin(rad)
            self._pending_translation_cm = 0.0

        # Rotation channel (independent deadband, with sub-deadband carry — same rationale).
        self._pending_rotation_deg += theta_delta_deg
        if abs(self._pending_rotation_deg) >= OFFSET_ROTATION_DEADBAND_DEG:
            self._offset.theta_deg += self._pending_rotation_deg
            self._pending_rotation_deg = 0.0

    def on_intersection(self, advance_to: str) -> None:
        self._current_node = advance_to
        # Arriving on a node re-anchors the operator-offset frame (mirror of
        # LocalizationClass::onIntersection -> clearOffset()).
        self.clear_offset()

    def on_dead_reckoned_advance(self, advance_to: str) -> None:
        self._current_node = advance_to
        # Same re-anchor as on_intersection (mirror of
        # LocalizationClass::onDeadReckonedAdvance -> clearOffset()).
        self.clear_offset()

    def declare_position(self, node_name: str) -> None:
        """Operator-declared belief reset: set the current node, clear the offset.

        Sim mirror of ``LocalizationClass::declarePosition`` (``Localization.cpp``):
        the same unconditional low-level write ``onIntersection`` /
        ``onDeadReckonedAdvance`` perform (set node + re-anchor the offset), but
        operator-initiated rather than triggered by a sensed crossing. The
        heading calibration (``_zeroed`` -- the sim's analogue of the firmware's
        untouched ``_yawOffsetDeg``) is deliberately left alone: the operator is
        only declaring graph position and zeroing the operator offset, not
        re-zeroing heading.

        This is a pure belief change -- unlike :meth:`on_intersection` /
        :meth:`on_dead_reckoned_advance`, no chassis primitive precedes it and no
        physical motion occurs, so callers must NOT pair this with any
        :class:`~tools.path_planner.sim.chassis.VirtualChassis` mutation.

        Contract for an unknown @p node_name (mirrors the loader's strict-field
        pattern, ``ctps/trace_loader.py``): this method WARNS
        (:class:`UserWarning`) and no-ops rather than raising -- the current
        node and offset are left completely unchanged. This differs from
        :meth:`PathTrackerSim.go_to` / :meth:`~PathTrackerSim.step_through`,
        which raise ``KeyError`` on an unknown goal: those reject a caller
        (API) mistake, whereas ``declare_position`` is driven off a recorded
        CTPS trace op (untrusted external data) where the established
        convention is warn-and-skip so one malformed op does not abort an
        otherwise-valid trace.

        :param node_name: node name to declare as the current position. Must be
            a key of ``field_data.NODES``; an unknown name warns and no-ops.
        """
        if node_name not in self._field_data.NODES:
            warnings.warn(
                f"LocalizationSim.declare_position: unknown node {node_name!r} "
                "-- not in field_data.NODES; declaration skipped, node and "
                "offset left unchanged.",
                UserWarning,
                stacklevel=2,
            )
            return
        self._current_node = node_name
        # Re-anchor the operator-offset frame exactly like a sensed arrival,
        # but the heading calibration flag (`_zeroed`) is untouched -- mirrors
        # the firmware leaving `_yawOffsetDeg` alone in `declarePosition`.
        self.clear_offset()


@dataclass
class _ChainRun:
    """One walked-ahead ADR-059 blind-chain run (sim mirror of ``ChainRun``).

    Function-identical to the firmware ``PathTrackerClass::ChainRun`` fixed-size
    run buffer, minus the RAM-bound array dimension (the sim uses growable lists
    honoring the same :data:`MAX_CHAIN_HOPS` cap). Built by
    :meth:`PathTrackerSim._try_start_blind_chain` when a >=2-hop collinear BLIND
    run forms, consumed hop-by-hop in :meth:`PathTrackerSim._walk_hops`.

    :ivar active: True while a run is in flight (gates every read).
    :ivar reverse: the run's uniform travel direction (a FRONT<->REAR flip ends
        a run, so it is constant along the whole run).
    :ivar nodes: the run's node names, in order (``hop_count + 1`` entries) —
        ``nodes[k] -> nodes[k+1]`` is hop ``k``.
    :ivar speeds: the resolved per-hop :class:`ChainHopSpeed`
        (``hop_count`` entries) from :func:`resolve_chain_speeds`; ``speeds[k]``
        carries hop ``k``'s signed ``(start_pwm, end_pwm, brake)``.
    :ivar hop_count: the number of hops (``>= 2``).
    :ivar hop_index: the index of the hop most recently dispatched (starts at
        ``-1`` before hop 0; the walk consumes ``speeds[hop_index]``).
    """

    active: bool = False
    reverse: bool = False
    nodes: List[str] = None      # type: ignore[assignment]
    speeds: List[ChainHopSpeed] = None    # type: ignore[assignment]
    hop_count: int = 0
    hop_index: int = -1


class PathTrackerSim:
    """Mirror of ``PathTrackerClass`` (LINE + BLIND edges).

    Per-hop dispatch from ``(edge_type, destination_node_type)`` -- projection
    is the *destination node* type (issue #64), not an edge type:

    * ``EDGE_LINE`` into a junction (line) node -> ``(ACT_FOLLOW_INT, None)``
      followed by a sim-only ``(ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)``
      step mirroring the runtime LINE_OVERSHOOT FSM transition.
    * ``EDGE_LINE`` into a projection node -> ``(ACT_FOLLOW_DIST, distance_cm)``;
      Localization advances via :meth:`LocalizationSim.on_dead_reckoned_advance`
      (the distance-terminated follow that retired the ``EDGE_PROJECTION``
      dispatch; the destination is read locally via
      :func:`tools.path_planner.graph_builder.is_projection_node`).
    * ``EDGE_BLIND`` -> resolved per :func:`resolve_stop_at_line`:
      ``seek`` FRONT -> ``(ACT_STOP_AT_LINE, distance_cm)`` +
      :meth:`LocalizationSim.on_intersection`; ``seek`` REAR ->
      ``(ACT_STOP_AT_LINE_REVERSE, target, distance_cm, blind_mode)`` +
      :meth:`LocalizationSim.on_intersection`; ``not seek`` FRONT ->
      ``(ACT_FORWARD, distance_cm)`` +
      :meth:`LocalizationSim.on_dead_reckoned_advance`; ``not seek`` REAR ->
      ``(ACT_BLIND_REVERSE, target, distance_cm, blind_mode)`` +
      :meth:`LocalizationSim.on_dead_reckoned_advance`.

    Operator offset + return-to-node (issue #36, PR1b):

    * :meth:`operator_move` injects a raw, non-``go_to`` operator move --
      moving the chassis AND accumulating the offset on the attached
      :class:`LocalizationSim` (subject to the nav-active gate + deadbands).
      It is the sim counterpart to the firmware integrating raw encoder / IMU
      deltas while ``!navActive``.
    * :meth:`go_to` runs a return-to-node prologue before walking NEXT_HOP:
      :meth:`_evaluate_return_gate` resolves the gate
      (:func:`resolve_return_action`) and, when an offset is above the trigger,
      :meth:`_dispatch_return_move` drives the inverse offset back onto the
      anchored node (modeling the ``stopAtLine`` overshoot/axle-roll for a
      LINE node, a dead-reckon turn+drive for a BLIND node) and re-anchors,
      PREPENDING the return primitive(s) to the planned stream.
    """

    def __init__(
        self,
        field_data,
        graph: nx.DiGraph,
        next_hop: NextHopTable,
        chassis: VirtualChassis,
    ) -> None:
        self._field_data = field_data
        self._graph = graph
        self._next_hop = next_hop
        self._chassis = chassis
        self._localization = LocalizationSim(field_data)

        # ADR-036: resolve the per-turn pivot table ONCE, against THIS sim's own
        # field_data. ``build_pivot_turns`` defaults to the module-global
        # field_data.PIVOT_TURNS, but a sim is constructed with an arbitrary
        # (often toy/synthetic) fixture -- letting it resolve the REAL field's
        # pivots against a toy graph would be a parity bug. Pass the sim's own
        # triples explicitly. ``track_cm`` stays None so it resolves
        # ``Config::TRACK_MM`` via ``parse_track_cm_from_config()`` -- the same
        # single source the emitter used, so the trims are byte-identical to the
        # firmware's ``PIVOT_TURNS`` records. ``getattr`` with an empty default
        # keeps the sim constructible from a pre-pivot fixture (one authored
        # before ``PIVOT_TURNS`` existed) -- such a field simply has no pivots.
        records = build_pivot_turns(
            self._graph, pivot_turns=getattr(self._field_data, "PIVOT_TURNS", [])
        )
        self._pivot_records: Dict[Tuple[str, str, str], PivotTurnRecord] = {
            (r.u, r.v, r.w): r for r in records
        }

        # ADR-036: the per-turn pivot latch contract (mirror of
        # ``PathTrackerClass``'s ``_pendingPivot`` / ``_departingPivot`` /
        # ``_approachingPivot`` PivotRefs). ``None`` is the "no pivot" sentinel.
        # The single ``_pending_pivot`` carry suffices ONLY because the departing
        # pivot is snapshotted BEFORE ``_pending_pivot`` is overwritten in
        # ``_walk_hops`` (snapshot-before-overwrite). All three reset to None at
        # construction and at the start of every ``go_to`` / ``step_through`` so a
        # final-hop pivot cannot leak its trim across a goal boundary.
        self._pending_pivot: Optional[PivotTurnRecord] = None
        self._departing_pivot: Optional[PivotTurnRecord] = None
        self._approaching_pivot: Optional[PivotTurnRecord] = None

        # ADR-059 blind-chain run buffer (mirror of the firmware ``_chain``).
        # Inactive until :meth:`_try_start_blind_chain` folds a >=2-hop collinear
        # BLIND run; reset at each ``go_to`` / ``step_through`` start and when a
        # run retires. ``active == False`` gates every read.
        self._chain: _ChainRun = _ChainRun()

    @property
    def localization(self) -> LocalizationSim:
        return self._localization

    @property
    def chassis(self) -> VirtualChassis:
        return self._chassis

    # -- operator offset + return-to-node (issue #36, PR1b) ------------------

    def set_nav_active(self, active: bool) -> None:
        """Push the hop-activity flag DOWN to the localization integrator gate.

        Thin passthrough to :meth:`LocalizationSim.set_nav_active`, mirroring
        how the firmware's ``PathTracker::update`` pushes
        ``Localization.setNavActive(...)`` down each tick. ``go_to`` runs
        atomically in the sim (it does not tick), so it manages this flag
        itself around the planned walk; this method exists so a test can drive
        the gate explicitly (e.g. to prove a move issued while navigating
        accumulates nothing).

        :param active: ``True`` while a hop executes; ``False`` otherwise.
        """
        self._localization.set_nav_active(active)

    def operator_move(
        self,
        distance_cm: float,
        *,
        turn_deg: float = 0.0,
    ) -> None:
        """Apply a raw, non-``go_to`` operator move (hand-driving the chassis).

        Models what happens between ``go_to`` calls when the operator
        hand-drives the chassis off its anchored node: the
        :class:`VirtualChassis` actually moves, AND
        :meth:`LocalizationSim.accumulate_operator_move` integrates that motion
        into the operator offset (subject to the nav-active gate + deadbands).
        This is the sim counterpart to the firmware integrating raw encoder /
        IMU deltas in ``Localization.update()`` while ``!navActive``.

        Order mirrors a real hand-drive: an optional in-place turn first
        (``Drive.turnDeg``), then a straight translation along the new heading
        (``Drive.driveDist``). The turn updates the chassis heading and feeds
        the offset's rotation channel; the translation moves the chassis along
        the (post-turn) heading and feeds the translation channel. The world
        heading the translation runs along is read AFTER the turn so the offset
        decomposition matches the chassis displacement exactly.

        :param distance_cm: signed straight-line distance to drive (cm),
            positive forward along the post-turn heading.
        :param turn_deg: signed in-place rotation to apply first (deg, CCW
            positive). Defaults to ``0`` (a pure translation).
        """
        if abs(turn_deg) > TURN_EPSILON_DEG:
            self._chassis.turn_relative(turn_deg)
        # Decompose the translation along the chassis heading AFTER the turn so
        # the integrated offset matches the chassis's actual displacement.
        heading_deg = self._chassis.heading_deg
        if abs(distance_cm) > 0.0:
            self._chassis.forward(distance_cm)
        self._localization.accumulate_operator_move(
            distance_cm, heading_deg, turn_deg
        )

    def _emit_turn_to(self, target_heading_deg: float) -> Optional[Primitive]:
        """Rotate the chassis to face @p target_heading_deg -- the SHARED pre-edge turn.

        Sim mirror of the firmware's ``PathTracker::_dispatchTurnIfNeeded``: the SAME
        turn decision every planned hop AND the return-to-node prologue go through, so a
        return can never skip the pre-edge turn (issue #36, #6).

        The turn-emission gate is :data:`NAV_TURN_TOLERANCE_DEG` (0.5°), NOT a raw float
        epsilon (#147): a heading delta at or below the tolerance emits NO
        ``ACT_TURN_REL`` (the firmware would not dispatch a turn that small either), so
        the sim's emission stream matches :func:`tools.path_planner.compiler.compile_path`,
        which gates on the same tolerance -- otherwise a real field's ~0.01° edge
        residuals would surface as spurious sim-only micro-turns.

        One logical-heading rule (#147): even when the turn is SKIPPED, the chassis
        (logical) heading is still advanced exactly onto @p target_heading_deg via
        :meth:`VirtualChassis.turn_to_heading`. Leaving it on the prior heading would let
        a chain of individually sub-tolerance hops accumulate a >tolerance drift and
        surface a spurious compiler-only turn on a later hop. ``turn_to_heading`` lands the
        heading on the same normalized target the emitted ``turn_relative(delta)`` branch
        would (``heading = _normalize(target)``), so the chassis pose -- and therefore
        every downstream drive and localization update -- is byte-identical whether the
        micro-turn was emitted or skipped; only the presence of the tiny ``ACT_TURN_REL``
        primitive differs. ``compile_path`` and :func:`compiler.final_heading_after` apply
        the identical unconditional advance, so the three surfaces track one logical
        heading.

        :param target_heading_deg: world-frame heading the chassis must face (deg).
        :return: the ``(ACT_TURN_REL, delta)`` primitive if a turn fired, else ``None``.
        """
        delta = wrap_pm180(target_heading_deg - self._chassis.heading_deg)
        if abs(delta) <= NAV_TURN_TOLERANCE_DEG:
            # Sub-tolerance: emit no ACT_TURN_REL, but STILL advance the logical
            # heading onto the target so a chain of sub-tolerance hops can never
            # accumulate into a compiler-only turn (one logical-heading rule, #147).
            self._chassis.turn_to_heading(target_heading_deg)
            return None
        self._chassis.turn_relative(delta)
        return (ACT_TURN_REL, delta)

    def _dispatch_return_move(
        self, off: Offset2D, mag: float
    ) -> Iterator[Primitive]:
        """Drive the inverse of the operator offset back onto the anchored node.

        Sim mirror of ``PathTracker::_dispatchReturnMove`` + ``_dispatchReturnMoveAfterTurn``.
        The chassis sits at ``node + offset``; the return drives the inverse offset vector so
        the wheel AXLE lands back ON the node, then re-anchors localization (which clears the
        offset). Emitted BEFORE the first planned hop departs, so a clean ``go_to`` after a
        return nets the original path's geometry with drift~0.

        A **generator**: each primitive is ``yield``ed AFTER its own chassis mutation, exactly
        like :meth:`_walk_hops`, so a :meth:`step_through` consumer snapshots the pose at THAT
        primitive (the pre-edge turn snapshots the post-turn pose, not the post-move pose) --
        the snapshot-invariant fix of #7.

        Return-travel world heading is ``atan2(-off.y, -off.x)`` (the heading of the inverse
        offset vector) normalized into ``[0, 360)`` -- the world frame is CCW-positive with
        ``0 = +x``, exactly atan2's convention. The drive distance is ``mag`` clamped to
        :data:`RETURN_DISTANCE_CAP_CM` (so a BEST_EFFORT over-cap return cannot command an
        unbounded drive). The caller guarantees ``mag > RETURN_TRIGGER_THRESHOLD_CM`` (>= a few
        cm), so the inverse vector is well-defined and atan2 never sees ``(0, 0)``.

        Both paths first dispatch the SHARED pre-edge turn (:meth:`_emit_turn_to`) -- the
        mirror of the firmware routing the return through ``_dispatchTurnIfNeeded`` -- so the
        ``ACT_TURN_REL`` is emitted conditionally for line re-acquires too, not only dead-reckon
        returns (#4). Then by whether the return re-acquires via the forward line seek (mirror of
        the firmware's ``_returnUsesLineSeek``: a LINE node AND ``dist >=
        LINE_SENSOR_AXLE_OFFSET_CM`` -- a shorter LINE return leaves the front sensor already past
        the node and would fault ``LINE_NOT_FOUND``, so it dead-reckons back instead, #14):

        * **Line seek** (:meth:`LocalizationSim.is_line_backed` and ``dist`` >= the axle lead):
          the SAME ``stopAtLine`` overshoot/axle-roll the LINE hops use -- a follow phase advances
          ``dist - LINE_SENSOR_AXLE_OFFSET_CM`` then a roll advances the remaining
          ``LINE_SENSOR_AXLE_OFFSET_CM`` so the AXLE (not the sensor) lands on the node.
          Yields ``(ACT_RETURN_STOP_AT_LINE, node, dist, heading_world)``; re-anchors via
          :meth:`LocalizationSim.on_intersection` (the same node).
        * **Dead-reckon** (blind anchor, OR a short LINE return): a dead-reckon forward
          ``driveDist``. Yields ``(ACT_RETURN_FORWARD, node, dist)``; re-anchors via
          :meth:`LocalizationSim.clear_offset` (no node change).

        :param off: the accumulated :class:`Offset2D` to invert.
        :param mag: ``off.magnitude_cm()`` (passed in to match the firmware
            signature; recomputed here would be identical).
        :return: the emitted return primitive(s), in emission order.
        """
        node = self._localization.current_node()

        # Inverse-offset world heading: direction from the hand-pushed pose
        # back to the anchor (mirror of atan2f(-off.y, -off.x) in degrees).
        return_heading_deg = _normalize(
            math.degrees(math.atan2(-off.y, -off.x))
        )

        # Clamp the drive distance to the trust cap (mirror of the firmware
        # clamp): an over-cap BEST_EFFORT return cannot command an unbounded
        # drive; an in-trust return (mag <= cap) is unaffected.
        return_dist_cm = min(mag, RETURN_DISTANCE_CAP_CM)

        # Decide whether this return re-acquires via the forward line seek (mirror of
        # PathTracker::_dispatchReturnMove's `_returnUsesLineSeek`). A LINE anchor is re-acquired
        # by driving FORWARD until the FRONT sensor re-crosses the node line; that sensor leads the
        # axle by LINE_SENSOR_AXLE_OFFSET_CM, so a return shorter than that lead leaves the sensor
        # already PAST the node and would fault LINE_NOT_FOUND on hardware. Such short LINE returns
        # dead-reckon back exactly like a blind anchor (#14) -- never via the (faking) line seek.
        uses_line_seek = (
            self._localization.is_line_backed(node)
            and return_dist_cm >= LINE_SENSOR_AXLE_OFFSET_CM
        )

        # Shared pre-edge turn (mirror of _dispatchTurnIfNeeded): face the return heading
        # first, conditionally -- skipped when already aligned (e.g. a straight-back push
        # where the return heading equals the facing). Yielded as its own step so the turn
        # snapshot is the post-turn pose (#4 / #7 / #8).
        turn = self._emit_turn_to(return_heading_deg)
        if turn is not None:
            yield turn

        if uses_line_seek:
            # Line-seek return: re-acquire the line. The stopAtLine seek lands the wheel AXLE on
            # the node (ADR-027), so the sim models the SAME overshoot/axle-roll the LINE hops
            # model (follow phase short of the node, then roll the sensor-to-axle offset onto it)
            # -- snapping the chassis straight to the node would make the return parity fake. Net
            # advance == return_dist_cm along the (now-faced) heading, which is exactly the inverse
            # offset, so the axle lands back on the node. The follow phase is non-negative here
            # because uses_line_seek guarantees return_dist_cm >= LINE_SENSOR_AXLE_OFFSET_CM.
            follow_cm = return_dist_cm - LINE_SENSOR_AXLE_OFFSET_CM
            self._chassis.follow_line_to_intersection(follow_cm)
            self._chassis.forward(LINE_SENSOR_AXLE_OFFSET_CM)
            # Re-anchor the SAME node (we returned TO it, not advanced) via the
            # verified physical re-localisation, which also clears the offset.
            self._localization.on_intersection(node)
            yield (ACT_RETURN_STOP_AT_LINE, node, return_dist_cm, return_heading_deg)
            return

        # Dead-reckon return: a blind-only anchor OR a short LINE return whose front sensor already
        # sits past the node line (#14). Dead-reckon the inverse drive forward (the shared pre-turn
        # above already faced the return heading -- the firmware's turnToYaw + driveDist split into
        # a TURNING step then the drive). The dead-reckoned drive IS the re-anchor -- clear_offset
        # zeroes the offset without changing the node id (we returned to the same node).
        self._chassis.forward(return_dist_cm)
        self._localization.clear_offset()
        yield (ACT_RETURN_FORWARD, node, return_dist_cm)

    def _dispatch_wall_reset(self, node: str) -> Iterator[Primitive]:
        """Run the wall-zero reset interlude on arrival at a reset node.

        Sim mirror of the firmware's reset interlude
        (``PathTracker::_stepFsm`` -> WALL_STOP -> REANCHOR -> WALL_DEPART ->
        PLAN_NEXT_HOP). Called the moment a hop re-anchors localization onto a
        node that :meth:`LocalizationSim.is_reset_node` reports as a reset node --
        BEFORE any further planning -- so it fires whether the reset node is an
        intermediate hop or the final destination, matching ``_maybeEnterReset``
        running at the top of ``_onPlanNextHop``.

        Like :meth:`_dispatch_return_move`, this is a generator that mutates the
        chassis then ``yield``\\s each primitive, so a consumer snapshotting the
        chassis at each yield (``step_through``) sees the pose AT that primitive.
        The three phases end at three distinct poses (wall surface -> axle ->
        node), so the per-phase mutate-then-yield ordering is load-bearing.

        Three phases:

        1. **wallStop** (mirror of ``Drive.wallStop(distToWall).setMode(GYRO)
           .setHeading(facing)``). The deterministic sim has no real stall, so it
           models the SUCCESSFUL press: turn to face ``facing`` (the wall normal)
           and advance to the wall coordinate on ``axis``. The firmware then backs
           off :data:`WALL_STOP_BACK_OFF_CM`, but phase 2 overwrites the believed
           on-axis coordinate, so the sim does NOT model the back-off (it would be
           discarded immediately and never reaches any observable output). Emits
           ``(ACT_WALL_STOP, node, axis_idx, wall_coord)``.
        2. **reanchor** (mirror of ``Localization.reanchorAtWall(axis, wall_coord,
           facing)``). Re-anchors the BELIEVED on-axis coordinate to the wheel
           AXLE -- ``wall_coord - sign*WALL_CONTACT_OFFSET_CM`` where
           ``sign = 1 if gap >= 0 else -1`` and ``gap = wall_coord - node_coord``
           -- on the NODE side of the wall surface (the axle is the tracked
           reference, :data:`WALL_CONTACT_OFFSET_CM` short of the rear contact
           face). Heading stays ``facing``; any pre-existing drift on ``axis`` is
           corrected. The sim's chassis IS the believed pose, so it is snapped to
           the axle. Emits ``(ACT_REANCHOR, node, axis_idx, wall_coord)`` (the
           4th element stays ``wall_coord``, the press target the firmware
           reports).
        3. **wallDepart** (mirror of the WALL_DEPART move
           ``Drive.driveDist(standoff - WALL_CONTACT_OFFSET_CM).setMode(GYRO)``
           then ``Localization.clearOffset()``, ``standoff = abs(wall_coord -
           node_coord)``). The chassis is squared to ``facing`` (front AWAY from
           the wall), so it drives FORWARD off the wall back onto the node: the
           net believed on-axis coordinate becomes exactly ``node_coord`` (drift
           on ``axis`` corrected TO the node, not the wall). Emits
           ``(ACT_WALL_DEPART, node, axis_idx, node_coord)``.

        :param node: the just-arrived reset node (localization is already
            anchored here). The caller guarantees
            :meth:`LocalizationSim.is_reset_node` is ``True``.
        :return: yields ``(ACT_WALL_STOP, ...)``, ``(ACT_REANCHOR, ...)``,
            ``(ACT_WALL_DEPART, ...)`` in emission order. Yields nothing if the
            node lost its reset spec between the ``is_reset_node`` check and now
            (defensive; mirrors the firmware's ``getResetInfo == nullptr`` skip).
        """
        info = self._localization.reset_info(node)
        if info is None:    # pragma: no cover -- defensive only
            return

        axis = info["axis"]                       # "x" or "y"
        wall_coord = float(info["wall_coord"])
        facing = _normalize(float(info["facing"]))
        axis_idx = _RESET_AXIS_TO_IDX[axis]       # cross-task ResetNode encoding (x=0/y=1)

        # The reset node's own stored on-axis coordinate -- the same source
        # `VirtualChassis.from_field_data` reads (`field_data.NODES[node]` ->
        # (x, y)), mirroring the firmware's `Localization::coordOnAxis(node, axis)`.
        node_coord = float(self._field_data.NODES[node][axis_idx])

        # -- Phase 1: square + press to the wall surface ---------------------
        # The firmware squares to `facing`, presses to the wall, then backs off
        # `WALL_STOP_BACK_OFF_CM`; phase 2 overwrites the on-axis belief, so the
        # back-off is dead work here. Snap the believed pose to the wall surface:
        # square to `facing` (the wall normal) and set the on-axis coordinate to
        # `wall_coord`; the off-axis component is left untouched.
        self._chassis.turn_to_heading(facing)
        if axis_idx == 0:
            self._chassis.x_cm = wall_coord
        else:
            self._chassis.y_cm = wall_coord
        yield (ACT_WALL_STOP, node, axis_idx, wall_coord)

        # -- Phase 2: re-anchor to the wheel axle (node side of the wall) -----
        # `reanchorAtWall` sets the believed on-axis coordinate to the AXLE,
        # `WALL_CONTACT_OFFSET_CM` short of the wall surface on the node side
        # (the rear face presses the wall, but the axle is the tracked
        # reference). The sim's chassis IS that believed pose, so snap it to the
        # axle. `reanchor_at_wall` keeps LocalizationSim's offset frame consistent
        # with a node arrival (cleared).
        axle_coord = _wall_axle_coord(wall_coord, node_coord)
        if axis_idx == 0:
            self._chassis.x_cm = axle_coord
        else:
            self._chassis.y_cm = axle_coord
        self._localization.reanchor_at_wall(axis, wall_coord, facing)
        yield (ACT_REANCHOR, node, axis_idx, wall_coord)

        # -- Phase 3: depart forward off the wall, back onto the node --------
        # The firmware drives FORWARD `standoff - WALL_CONTACT_OFFSET_CM` (the
        # axle was that much short of the node) then `clearOffset()`, so the net
        # belief returns to the node. The sim's chassis IS the believed pose, so
        # snap the on-axis coordinate straight to `node_coord` (heading already
        # squared to `facing`). The off-axis component is left untouched.
        if axis_idx == 0:
            self._chassis.x_cm = node_coord
        else:
            self._chassis.y_cm = node_coord
        yield (ACT_WALL_DEPART, node, axis_idx, node_coord)

    def _evaluate_return_gate(self, goal_name: str) -> Iterator[Primitive]:
        """Resolve the return gate and emit the return primitive(s), if any.

        Sim mirror of ``PathTracker::_evaluateReturnGate``. Reads the current
        accumulated offset, runs the pure gate decision
        (:func:`resolve_return_action`) on its translation magnitude, then:

        * :attr:`ReturnAction.NONE` -- offset within trigger; no return move,
          no chassis motion. Yields nothing so the planned-path stream stays
          byte-identical to today's (AC5).
        * :attr:`ReturnAction.RETURN` / :attr:`ReturnAction.OFF_MAP_PROCEED` --
          dispatch the inverse return (the over-cap BEST_EFFORT case clamps the
          distance to the cap inside :meth:`_dispatch_return_move`).
        * :attr:`ReturnAction.OFF_MAP_FAULT` -- offset over cap, FAULT gate;
          raise :class:`OffMapFaultError` and move nothing (mirror of the
          firmware's ``_enterFault(Fault::OFF_NODE, ...)`` refusal).

        A **generator** (like :meth:`_dispatch_return_move` it delegates to): ``go_to``
        forces it eagerly with ``list(...)`` so the chassis mutation + re-anchor (and the
        FAULT-gate raise) happen before the planned walk, exactly as before; ``step_through``
        iterates it lazily so each return primitive pairs with its own post-mutation snapshot.

        :param goal_name: the requested goal (carried only for the fault
            diagnostic, matching the firmware's ``_destinationNode``).
        :return: the return primitive(s) prepended to the planned stream (empty
            for the NONE no-op path).
        :raises OffMapFaultError: on an over-cap offset under the FAULT gate.
        """
        off = self._localization.get_offset_from_node()
        mag = off.magnitude_cm()
        action = resolve_return_action(
            mag,
            RETURN_TRIGGER_THRESHOLD_CM,
            RETURN_DISTANCE_CAP_CM,
            OFF_MAP_GATE,
        )

        if action == ReturnAction.NONE:
            return
        if action == ReturnAction.OFF_MAP_FAULT:
            raise OffMapFaultError(
                self._localization.current_node(), goal_name, mag
            )
        # RETURN or OFF_MAP_PROCEED: drive the inverse return (BEST_EFFORT
        # over-cap is clamped to the cap inside _dispatch_return_move).
        yield from self._dispatch_return_move(off, mag)

    # -- mode resolution (instance-method passthroughs) ----------------------

    def resolve_speed(
        self, edge, cfg: PipelineConfig = PIPELINE_CONFIG
    ) -> SpeedTier:
        """See module-level :func:`resolve_speed`.

        ``cfg`` defaults to the :data:`PIPELINE_CONFIG` singleton so existing
        callers keep working; ``_walk_hops`` threads the field-sourced config
        so the sim mirrors the robot's per-field pipeline default (R4).
        """
        return resolve_speed(edge, cfg)

    def resolve_blind_mode(
        self, edge, cfg: PipelineConfig = PIPELINE_CONFIG
    ) -> BlindMode:
        """See module-level :func:`resolve_blind_mode`.

        ``cfg`` defaults to the :data:`PIPELINE_CONFIG` singleton; ``_walk_hops``
        threads the field-sourced config (R4).
        """
        return resolve_blind_mode(edge, cfg)

    def find_pivot_turn(
        self, u: str, v: str, w: str
    ) -> Optional[PivotTurnRecord]:
        """Return the resolved pivot record for the turn ``(u, v, w)``, or None.

        Mirror of the firmware's ``FieldMap::findPivotTurn(u, v, w)`` (ADR-036):
        a route that does not actually take the declared turn -- or a turn with
        no authored pivot -- misses and gets no trim/maneuver. A ``None`` for any
        of ``u`` / ``v`` / ``w`` (e.g. ``next_next`` past the goal) simply misses,
        which is correct: no pivot can be keyed on a missing node.

        :param u: arrive-from node name.
        :param v: pivot-at node name.
        :param w: leave-to node name.
        :return: the :class:`PivotTurnRecord` for ``(u, v, w)`` if one is
            declared, else ``None``.
        """
        return self._pivot_records.get((u, v, w))

    # -- ADR-036 pivot-trim (mirror of PathTracker.cpp) -----------------------

    def _pivot_trim_distance(self, edge) -> float:
        """Return the LOCAL effective distance for a DISTANCE-terminated hop.

        Byte-for-byte mirror of ``PathTrackerClass::_pivotTrimDistance`` (ADR-036,
        issue #44, per-turn model). The edge's ``distance_cm`` (the shared
        ``EDGES[*][*]`` cell) is NEVER mutated; the trim is folded into a local
        effective distance only. No carry / latch lives here anymore -- the
        one-hop carry is owned by the ``_pending_pivot`` latch contract resolved
        in :meth:`_walk_hops`. This function just SUBTRACTS the two already-
        resolved trims and clamps:

        1. **Departing trim:** if this hop LEAVES a pivot node
           (``_departing_pivot`` set), subtract its ``trim_cm`` -- the outgoing
           over-travel of the pivot we just turned through.
        2. **Approaching trim:** if this dead-reckon edge drives INTO a pivot node
           (``_approaching_pivot`` set), subtract its ``trim_cm`` so the chassis
           stops ``delta`` short and the pivot starts at the geometric pivot
           point.

        On a chain ``u->v->w->x`` the hop ``v->w`` nets
        ``length - delta_departing - delta_approaching`` (this is exactly what
        the validator's chained sum-check guards).

        A defensive floor (:data:`PIVOT_TRIM_MIN_DRIVE_CM`) keeps the result
        positive if a future bad table ever over-trims, mirroring the firmware
        clamp; the validator otherwise guarantees the trims fit.

        The subtract-and-clamp itself is delegated to the shared core
        :func:`resolve_pivot_trim` -- the SAME core the SENSOR path
        (:func:`resolve_sensor_overshoot_trim`) routes through, mirroring the C++
        ``resolvePivotTrim`` both firmware paths share so the two cannot drift.

        :param edge: directed graph edge attribute mapping for the hop.
        :return: the effective (possibly trimmed) advance distance in cm.
        """
        departing_trim_cm = (
            self._departing_pivot.trim_cm if self._departing_pivot is not None else 0.0
        )
        approaching_trim_cm = (
            self._approaching_pivot.trim_cm
            if self._approaching_pivot is not None
            else 0.0
        )
        drive_cm, _effective_cm, _clamped = resolve_pivot_trim(
            float(edge["distance_cm"]),
            departing_trim_cm,
            approaching_trim_cm,
            PIVOT_TRIM_MIN_DRIVE_CM,
        )
        return drive_cm

    # -- shared per-hop dispatch (single source for go_to + step_through) ------

    def _walk_hops(
        self,
        goal_name: str,
        *,
        route_direction: "Direction",
        arrival_direction: "Direction",
    ) -> Iterator[Primitive]:
        """Yield the primitive stream from the current node to ``goal_name``.

        Single source of the per-hop dispatch shared by :meth:`go_to` and
        :meth:`step_through`. Walks NEXT_HOP one edge at a time, resolves each
        hop's direction + per-edge modes, applies the pivot-trim latch/carry,
        mutates the chassis + localization, and ``yield``\\s every primitive in
        emission order. Each primitive is yielded **after** its own chassis
        mutation (and any localization re-anchor), so a consumer that snapshots
        the chassis at each yield sees the same post-mutation state the old
        inline :meth:`step_through` captured.

        The two public entry points differ only in how they consume this
        stream — :meth:`go_to` collects the primitives into a flat list, while
        :meth:`step_through` pairs each with a :meth:`_snapshot_chassis` copy.
        Neither the resolution logic nor the pivot latch/carry threading lives
        in the callers, so a change to the per-hop dispatch lands in one place.

        Goal validation and the per-goal pivot-carry reset are the caller's
        responsibility (each entry point performs them before iterating this
        generator); this method assumes a clean carry and a known goal.

        :param goal_name: destination node name (already validated by the
            caller to be in ``field_data.NODES``).
        :param route_direction: builder override applied to every hop.
        :param arrival_direction: builder override applied only to the final
            hop.
        :raises UnreachableGoalError: if any intermediate hop is unreachable.
        """
        # R4: resolve the field-sourced pipeline defaults ONCE per walk and
        # thread the same ``cfg`` through every direction / mode resolution so
        # the sim mirrors the robot. A field-set ``PIPELINE_LINE_MODE`` /
        # ``BLIND_MODE`` / ``DIRECTION_MODE`` reaches the emitted primitives
        # here exactly as it reaches ``routes.cpp`` via ``header_writer``;
        # ``from_field_data`` falls back to the ``PIPELINE_CONFIG`` singleton
        # for a field that omits the keys, so the many ``SimpleNamespace``
        # fixtures are byte-identical to the prior behaviour.
        cfg = PipelineConfig.from_field_data(self._field_data)

        # ADR-023 pass-through carry (mirror of the firmware `_passThrough` flag):
        # set True when a hop emits a collinear pass tag (LINE/projection) and the
        # follower stays live across the node, consumed by the NEXT hop's BLIND
        # dispatch to seed the ADR-059 chain `entry_pwm` with the CAPTURED cruise
        # (AC5) instead of the cold-start +/-TSHAPE_START_SPEED. Reset every hop.
        pass_through_active = False

        current = self._localization.current_node()
        while current != goal_name:
            nxt = self._next_hop[current][goal_name]
            if nxt is None:
                raise UnreachableGoalError(current, goal_name, current=current)
            edge = self._graph.edges[current, nxt]
            edge_heading = float(edge["heading_deg"])
            is_last_hop = (nxt == goal_name)

            # -----------------------------------------------------------------
            # Per-turn pivot latch contract (ADR-036, mirror of
            # ``PathTracker._onPlanNextHop``). A pivot is a property of a 3-node
            # turn ``(u, v, w)``, not of an edge. Snapshot the carry into
            # ``_departing_pivot`` BEFORE the one-hop lookahead overwrites
            # ``_pending_pivot`` -- the single carry member is sufficient ONLY
            # because the departing pivot is read out first.
            #
            # Last hop: no pivot can latch onto the FINAL node mid-turn, so skip
            # the forward lookahead. This NEVER skips consuming an already-pending
            # latch -- a pivot ``(u, v, w)`` whose goal is ``w`` still consumes its
            # trim on the final ``v->w`` hop via ``_departing_pivot``.
            # -----------------------------------------------------------------
            self._departing_pivot = self._pending_pivot    # snapshot the carry FIRST
            if is_last_hop:
                self._approaching_pivot = None
            else:
                next_next = self._next_hop[nxt][goal_name]    # may be None
                self._approaching_pivot = self.find_pivot_turn(current, nxt, next_next)
            self._pending_pivot = self._approaching_pivot     # carry for the NEXT hop

            direction = resolve_direction(
                edge_dir_override=_read_edge_dir_override(edge),
                pipeline_default=cfg.direction,
                builder_arrival=arrival_direction,
                builder_route=route_direction,
                current_heading=self._chassis.heading_deg,
                edge_heading=edge_heading,
                is_last_hop=is_last_hop,
                bidirectional_enabled=BIDIRECTIONAL_AUTO_ENABLED,
                line_board_count=LINE_BOARD_COUNT,
            )

            # ADR-036: pivots are FRONT-only in v1 (#54). The wheel + delta are
            # derived for forward traversal, so force FRONT on any pivot-adjacent
            # hop -- the approach ``u->v`` (``_approaching_pivot``) or the
            # departure ``v->w`` (``_departing_pivot``). Never silently downgrade
            # to a counter-rotation: the author chose a single-wheel pivot for a
            # clearance a center turn may not have. Mirrors the force-FRONT
            # override in ``PathTracker._onPlanNextHop``.
            #
            # This force-FRONT is the single invariant the seek/LINE overshoot
            # trims rely on: a pivot-adjacent hop is always FRONT, so the FRONT
            # seek branch below (and the FRONT delta-budget the validator + the
            # firmware use) is the only one that ever sees an approaching pivot.
            # A REAR seek therefore CANNOT carry an approaching pivot today --
            # that is why the REAR-seek branch needs no overshoot trim. #54 (REAR
            # pivots) must add a REAR delta-budget on the seek + LINE overshoot
            # paths BEFORE relaxing this (ADR-044 trigger).
            if self._departing_pivot is not None or self._approaching_pivot is not None:
                direction = Direction.FRONT

            body_heading = _body_heading_for(direction, edge_heading)

            # Shared pre-edge turn (the SAME helper the return prologue uses) — emits a
            # conditional ACT_TURN_REL and rotates the chassis onto body_heading (#6 / #8).
            turn = self._emit_turn_to(body_heading)
            if turn is not None:
                yield turn

            edge_type = edge["type"]
            distance_cm = float(edge["distance_cm"])
            # Projection is now a DESTINATION-NODE type (issue #64), read
            # LOCALLY off ``nxt`` -- no goal-relative ``NEXT_HOP`` peek. The
            # per-hop primitive falls out of ``(edge_type, dest_node_type)``:
            # a LINE edge into a projection node is line-followed but
            # DISTANCE-terminated (the retired ``EDGE_PROJECTION`` behavior),
            # while a LINE edge into a junction (line) node re-anchors on the
            # crossing marker. Mirrors ``compile_path`` (T4) cell-for-cell.
            dest_is_projection = is_projection_node(self._graph, nxt)

            if edge_type not in (EDGE_LINE, EDGE_BLIND):    # pragma: no cover -- defensive only
                raise ValueError(
                    f"unknown edge type {edge_type!r} on edge ({current!r}, {nxt!r})"
                )

            # -----------------------------------------------------------------
            # Resolve every dispatch input HERE, then classify the hop through the
            # terminal one-hop classifier ``resolve_edge_dispatch`` (the Python
            # twin of ``EdgeDispatchPolicy.h::resolveEdgeDispatch``). The
            # classifier owns NO cross-hop state -- direction, the pivot 3-latch
            # carry, the pass-through lookahead and the stop-at-line seek are all
            # resolved here and arrive on the query pre-resolved; the classifier
            # only composes the existing pure trim policies and reports the
            # primitive + reverse + commanded distance + overshoot + re-anchor
            # kind. The ACT_* tuples and chassis/localization mutations below stay
            # byte-identical to the pre-classifier dispatch.
            # -----------------------------------------------------------------
            # The two pivot trims latched at the top of this loop:
            # ``_departing_pivot`` is the pivot just LEFT, ``_approaching_pivot``
            # the pivot being ENTERED. ``distance_cm`` (the shared table cell) is
            # never mutated -- the trims fold into a LOCAL commanded distance.
            departing_trim_cm = (
                self._departing_pivot.trim_cm
                if self._departing_pivot is not None
                else 0.0
            )
            approaching_trim_cm = (
                self._approaching_pivot.trim_cm
                if self._approaching_pivot is not None
                else 0.0
            )
            # Sensor-to-axle offset of the board that LEADS this hop; a REAR hop
            # uses the rear board's own offset (mirrors PathTracker). Feeds the
            # classifier's seek overshoot channel.
            axle_offset_cm = (
                LINE_SENSOR_AXLE_OFFSET_REAR_CM
                if direction == Direction.REAR
                else LINE_SENSOR_AXLE_OFFSET_CM
            )

            # `line(A)-projection(B)-line(C)` pass-through lookahead (issue #64),
            # resolved here so it is a clean classifier input (the C++ FSM hoists
            # the side-effect-free ``_evaluateProjectionPassThrough`` the same
            # way). Only meaningful on a LINE-into-projection hop; this same local
            # also selects the pass-variant emission tag in the PROJECTION_FOLLOW
            # branch below (where the detailed T20 note lives). Evaluating it here
            # is byte-identical to the old in-branch call -- the predicate is pure.
            proj_pass_through = (
                _should_projection_pass_through(
                    self._graph,
                    self._next_hop,
                    nxt,
                    goal_name,
                    inbound_body_heading=body_heading,
                    inbound_direction=direction,
                    pipeline_default=cfg.direction,
                    builder_route=route_direction,
                    builder_arrival=arrival_direction,
                    bidirectional_enabled=BIDIRECTIONAL_AUTO_ENABLED,
                    line_board_count=LINE_BOARD_COUNT,
                )
                if (edge_type == EDGE_LINE and dest_is_projection)
                else False
            )

            # BLIND stop-at-line seek decision (resolved only for BLIND edges; a
            # LINE edge passes a neutral default the classifier never reads). The
            # offline streams carry no call-site verb, so resolve with
            # ``StopAtLine.AUTO`` (the firmware's per-edge AUTO default) and let
            # the precedence ladder decide. REAR seek needs a 2-board chassis --
            # ``resolve_stop_at_line`` returns ``seek=False`` for REAR on 1-board,
            # degrading to the dead-reckon reverse. Auto-rule (issue #64): a BLIND
            # edge whose DESTINATION is a projection node seeks that node's own
            # line first to re-anchor -- a purely LOCAL read off ``nxt``, mirroring
            # the firmware ``getNodeType(_nextHop) == PROJECTION``.
            if edge_type == EDGE_BLIND:
                auto_rule_active = (
                    getattr(self._field_data, "BLIND_PROJECTION_AUTO_SEEK", True)
                    and dest_is_projection
                )
                stop_at_line = resolve_stop_at_line(
                    StopAtLine.AUTO,
                    _read_edge_stop_at_line_override(edge),
                    resolve_blind_mode(edge, cfg),
                    cfg.blind,
                    direction,
                    LINE_BOARD_COUNT,
                    auto_rule_active,
                )
            else:
                stop_at_line = (False, BlindMode.DEFAULT)

            decision = resolve_edge_dispatch(
                EdgeDispatchQuery(
                    edge_type=edge_type,
                    dest_is_projection=dest_is_projection,
                    direction=direction,
                    raw_distance_cm=distance_cm,
                    line_mode=(
                        resolve_speed(edge, cfg)
                        if edge_type == EDGE_LINE
                        else cfg.speed
                    ),
                    stop_at_line=stop_at_line,
                    proj_pass_through=proj_pass_through,
                    departing_trim_cm=departing_trim_cm,
                    approaching_trim_cm=approaching_trim_cm,
                    axle_offset_cm=axle_offset_cm,
                    min_drive_cm=PIVOT_TRIM_MIN_DRIVE_CM,
                )
            )

            # Line-arrival heading reset (ADR-052) eligibility, latched by the
            # pass-through branches below. `None` unless THIS hop's arrival at
            # `nxt` is a collinear pass-through: `"proj"` for a PROJECTION-node
            # pass-through (`proj_pass_through`, PROJECTION_FOLLOW branch),
            # `"junction"` for a LINE-node pass-through (`pass_through`,
            # LINE_FOLLOW_INTERSECTION branch). Mirrors the firmware's
            # `_passThrough` flag + `FieldMap::getNodeType(cur)` switch in
            # `PathTracker::_maybeHeadingReset` -- BLIND/STOP_AT_LINE/dead-reckon
            # arrivals never set it, exactly as `_passThrough` is latched only on
            # a genuine LINE-edge pass-through arrival.
            heading_reset_kind: Optional[str] = None

            if decision.primitive == DispatchPrimitive.PROJECTION_FOLLOW:
                # LINE -> projection node: line-follow stopped by the encoder
                # distance gate (no junction marker beneath the strip), so the
                # chassis dead-reckons into the node via
                # ``on_dead_reckoned_advance`` -- NOT ``on_intersection``. This
                # is the distance-terminated follow that replaced the
                # ``EDGE_PROJECTION`` dispatch; the motion profile comes from the
                # LINE edge's ``line_mode`` (``ProjectionMode`` deleted).
                #
                # ADR-036: a DISTANCE-terminated hop takes BOTH the departing
                # trim (the pivot we just left, ``_departing_pivot``) and the
                # approach trim (the pivot at ``nxt``, ``_approaching_pivot``) --
                # both folded into a LOCAL effective distance by
                # ``_pivot_trim_distance``. ``distance_cm`` (the shared table
                # cell) is never mutated. No LINE_OVERSHOOT step follows -- the
                # encoder gate already lands the chassis on the node.
                effective_dist_cm = decision.commanded_distance_cm

                # T20 (issue #64): `line(A)–projection(B)–line(C)` PASS-THROUGH.
                # The TRIGGER is THIS A→B DISTANCE gate, not an intersection -- a
                # projection node has no junction marker, so the firmware's
                # `Intersection.isAtIntersection()` never fires at B (mirror of the
                # PROJECTION_FOLLOW `_projPassThrough` branch in PathTracker.cpp,
                # T19). When B is a non-goal projection node whose outbound B→C is
                # a LINE edge into a LINE junction node AND A→B→C are collinear /
                # same-direction, the follower stays live across B for one
                # continuous A→C motion: stamp the dead-reckon advance at B
                # MID-FOLLOW (no stop, same `on_dead_reckoned_advance(B)`) and emit
                # the pass-variant tag (`ACT_FOLLOW_DIST_PASS` / its REAR twin)
                # instead of the plain follow primitive. The chassis advance and
                # the re-anchor are IDENTICAL to the non-pass branch -- only the
                # tag differs (the projection twin of ADR-023's
                # ACT_LINE_PASS-vs-ACT_LINE_OVERSHOOT). Any guard miss ⇒ the
                # normal per-hop stop at B below (the T4/T5 fallback). The B→C hop
                # then re-anchors at C's junction on the next loop iteration. The
                # gate's own `inbound_direction == next_dir` check means a REAR
                # pass-through only fires on a fully-reverse collinear triple.
                if decision.reverse:
                    self._chassis.reverse(effective_dist_cm)
                    self._localization.on_dead_reckoned_advance(nxt)
                    yield (
                        ACT_PROJECTION_REVERSE_PASS
                        if proj_pass_through
                        else ACT_PROJECTION_REVERSE,
                        nxt,
                        effective_dist_cm,
                        resolve_speed(edge, cfg),
                    )
                else:
                    self._chassis.follow_line_to_distance(effective_dist_cm)
                    self._localization.on_dead_reckoned_advance(nxt)
                    yield (
                        ACT_FOLLOW_DIST_PASS if proj_pass_through else ACT_FOLLOW_DIST,
                        effective_dist_cm,
                    )
                if proj_pass_through:
                    heading_reset_kind = "proj"
            elif decision.primitive == DispatchPrimitive.LINE_FOLLOW_INTERSECTION:
                # LINE -> junction (line) node: line-follow terminated by the
                # physical intersection marker, then a LINE_OVERSHOOT roll.
                # The classifier intentionally does NOT own the LINE_OVERSHOOT
                # advance (it returns overshoot_cm == 0 here) -- that FSM
                # transition stays a ``_walk_hops`` concern, computed below via the
                # shared sensor-overshoot twin EXACTLY as before.
                #
                # ADR-036: a LINE outgoing edge is SENSOR-terminated -- it
                # re-anchors on the junction line, so it takes NO dead-reckon
                # distance trim. The departing trim is irrelevant here (the line
                # corrects it); the approach trim for a LINE-incoming pivot is the
                # overshoot-advance reduction below; the outgoing carry for the
                # next hop is already owned by the ``_pending_pivot`` latch
                # resolved at the top of this loop. Nothing to do here.

                # Dead-reckon with the axle offset of the board that LED this hop;
                # a REAR hop uses the rear board's own offset (mirrors PathTracker).
                overshoot_offset_cm = (
                    LINE_SENSOR_AXLE_OFFSET_REAR_CM
                    if direction == Direction.REAR
                    else LINE_SENSOR_AXLE_OFFSET_CM
                )
                # The line follower stops the chassis when the SENSOR reaches the
                # intersection marker, but the sensor leads the wheel axle by
                # `overshoot_offset_cm`, so the axle is still that far short of the
                # node at detect (PathTracker.cpp LINE_FOLLOW: "the chassis center
                # is still that distance short of the junction node"). Advance only
                # `distance_cm - overshoot_offset_cm` in the follow phase; the
                # LINE_OVERSHOOT step below rolls the remaining offset so the wheel
                # axle -- not the sensor -- lands ON the node. Net advance over the
                # hop is exactly `distance_cm`, matching the firmware re-anchor.
                #
                # GUARD: a LINE edge shorter than `overshoot_offset_cm` would push
                # `follow_cm` negative. The whole edge is then rolled as overshoot
                # (follow_cm clamped to 0) so the net advance stays `distance_cm`
                # and the sim never emits a negative follow displacement. No
                # field/toy LINE edge is currently this short (all >= 30 cm vs the
                # 12.02 cm FRONT offset); this is defensive only.
                follow_cm = max(0.0, distance_cm - overshoot_offset_cm)

                # ADR-036 LINE-incoming pivot trim (per-turn compensation). If
                # THIS hop drives into a pivot node (``_approaching_pivot`` set, a
                # turn ``(current, nxt, next_next)`` resolved at the top of this
                # loop), a LINE edge cannot stop short on its approach (it
                # re-anchors on the junction line), so instead shorten the
                # OVERSHOOT advance by delta -- the axle settles delta short of the
                # node. Net hop becomes `distance_cm - delta` (the LINE row of the
                # ADR-036 regime matrix). Mirrors PathTracker.cpp's LINE_FOLLOW
                # overshoot arming: `overshootTargetCm = offset - delta`, clamped
                # to the PIVOT_TRIM_MIN_DRIVE_CM floor; `follow_cm` is unchanged by
                # delta. The OUTGOING carry for the next hop is NOT latched here:
                # it is already owned by ``_pending_pivot`` -> the next hop's
                # ``_departing_pivot``.
                inbound_pivot_trim = (
                    self._approaching_pivot.trim_cm
                    if self._approaching_pivot is not None
                    else 0.0
                )
                # Subtract-and-clamp the approaching trim via the shared twin
                # (mirror of PivotTrimPolicy.h::resolveSensorOvershootTrim,
                # ADR-044 D1). Pass the SAME ``overshoot_offset_cm`` already
                # resolved above so REAR keeps the rear board's own offset.
                # Byte-identical to the old inline branch: trim == 0 ->
                # overshoot_offset_cm; trim > 0 within budget -> offset - trim;
                # over budget -> the PIVOT_TRIM_MIN_DRIVE_CM floor.
                overshoot_advance_cm, _clamped = resolve_sensor_overshoot_trim(
                    overshoot_offset_cm,
                    inbound_pivot_trim,
                    PIVOT_TRIM_MIN_DRIVE_CM,
                )

                if decision.reverse:
                    self._chassis.reverse(follow_cm)
                    yield (ACT_LINE_REVERSE, nxt, resolve_speed(edge, cfg))
                else:
                    self._chassis.follow_line_to_intersection(follow_cm)
                    yield (ACT_FOLLOW_INT, None)
                # LINE_OVERSHOOT FSM transition -- emit ACT_LINE_PASS when
                # the resolved body-heading of the next-next hop is collinear
                # with the inbound body-heading (delta <= NAV_TURN_TOLERANCE_DEG),
                # else ACT_LINE_OVERSHOOT. The peek MUST use resolved
                # body-headings, not raw forward edge headings, so mixed
                # REAR<->FORWARD transitions on anti-parallel forward edges still
                # pass through cleanly (TC9c). The primitive name differs only for
                # downstream verification + telemetry. See ADR-023.
                #
                # The payload is the (possibly pivot-reduced) overshoot advance,
                # matching the distance the firmware commands into the gate.
                pass_through = _should_pass_through(
                    self._graph,
                    self._next_hop,
                    nxt,
                    goal_name,
                    inbound_body_heading=body_heading,
                    inbound_direction=direction,
                    pipeline_default=cfg.direction,
                    builder_route=route_direction,
                    builder_arrival=arrival_direction,
                    bidirectional_enabled=BIDIRECTIONAL_AUTO_ENABLED,
                    line_board_count=LINE_BOARD_COUNT,
                )
                # REAR overshoot: rear board (at -AXLE_OFFSET in body-forward) leads
                # in the travel direction. When it detects the intersection at v,
                # the chassis center is AXLE_OFFSET cm behind v in the travel
                # direction. Advancing AXLE_OFFSET in the travel direction = reverse
                # in body coordinates (body heading points opposite to travel).
                if decision.reverse:
                    self._chassis.reverse(overshoot_advance_cm)
                else:
                    self._chassis.forward(overshoot_advance_cm)
                self._localization.on_intersection(nxt)
                yield (
                    ACT_LINE_PASS if pass_through else ACT_LINE_OVERSHOOT,
                    overshoot_advance_cm,
                )
                if pass_through:
                    heading_reset_kind = "junction"
            elif decision.primitive == DispatchPrimitive.STOP_AT_LINE_SEEK:
                # BLIND stop-at-line seek (the seek + motion bundle were resolved
                # into the query above; the classifier returned STOP_AT_LINE_SEEK,
                # so this hop seeks regardless of FRONT/REAR). The board stops on
                # the physical crossing line, so both directions re-anchor via
                # ``on_intersection``.
                if not decision.reverse:
                    # Front board stops on the physical crossing line; the
                    # chassis re-anchors via ``on_intersection`` (the "better
                    # localization") instead of dead-reckoning. The seek drives
                    # up to ``distance_cm`` and the crossing -- not the distance
                    # -- terminates it; the outgoing carry for the next hop is
                    # already owned by the ``_pending_pivot`` latch (a pivot
                    # reached via a seek still over-travels on its OUTGOING side;
                    # the next hop's ``_departing_pivot`` applies that trim).
                    # ADR-044 D1 / issue #64 seek-trim parity: a seek INTO an
                    # approaching pivot must land the axle ``delta`` short of the node
                    # (like every other approach mechanism), so the firmware reduces
                    # the post-detect axle roll from the full ``offset`` to
                    # ``offset - delta`` (the shared sensor-overshoot twin, clamped to
                    # the floor). The firmware seek-arming has NO projection
                    # special-case -- a BLIND->projection auto-seek and a BLIND->line
                    # override seek arm the SAME
                    # ``Drive.stopAtLine(edge.distance_cm).setOvershoot(offset - delta)``
                    # (``PathTracker.cpp`` STOP_AT_LINE dispatch) -- so this branch
                    # applies the trim uniformly regardless of ``dest_is_projection``.
                    # A ``(u, projection, w)`` pivot whose ``u->P`` is a BLIND auto-seek
                    # is validator-legal (``_validate_pivot_turns`` resolves that leg
                    # through the ``sensor`` regime and budgets the overshoot), so a
                    # projection-special full advance here would stop the sim short of
                    # mirroring the firmware's delta-short pose (the sim is the
                    # firmware's pose oracle). A seek is sensor-terminated, so the
                    # DEPARTING trim is auto-absorbed on the line and never subtracted
                    # here. With NO approaching pivot the trim is 0 ->
                    # ``offset - overshoot_cm == 0`` -> the advance is the full
                    # ``distance_cm`` (AC2): a plain BLIND->projection re-anchor still
                    # full-advances onto the node, and only a pivot-approaching seek
                    # lands delta-short.
                    #
                    # The sim chassis center is the axle, so shrinking the FORWARD
                    # displacement by ``offset - overshoot_cm`` lands the center
                    # ``delta`` short -- exactly the firmware's reduced post-detect
                    # roll. Localization still re-anchors at the node (the seek stops
                    # ON the line; ``on_intersection`` below); only the physical
                    # displacement shrinks. This is the FRONT seek branch, so the
                    # offset is the FRONT sensor-to-axle offset (force-FRONT guarantees
                    # every pivot seek lands here). The ``max(0.0, ...)`` clamp mirrors
                    # the LINE follow's degenerate-geometry guard; the firmware instead
                    # hard-faults on ``delta > offset`` (PIVOT_TRIM_UNDERFLOW), which
                    # the validator's delta-budget precludes for any legal sensor-regime
                    # pivot. The approaching-pivot trim was already folded into
                    # ``decision.overshoot_cm`` by the classifier (the shared
                    # sensor-overshoot twin, clamped to the floor).
                    offset = LINE_SENSOR_AXLE_OFFSET_CM
                    seek_advance_cm = max(
                        0.0, distance_cm - (offset - decision.overshoot_cm)
                    )
                    self._chassis.forward(seek_advance_cm)
                    self._localization.on_intersection(nxt)
                    # The emitted primitive carries the COMMAND distance
                    # (``distance_cm``), NOT the trimmed physical advance. The
                    # firmware dispatches ``Drive.stopAtLine(edge.distance_cm)``
                    # (the seek's max search bound -- the crossing, not the
                    # distance, terminates it) and folds the approaching-pivot
                    # trim into the separate ``.setOvershoot(...)`` channel, so
                    # the commanded distance stays full. ``compile_path`` emits
                    # the same ``(ACT_STOP_AT_LINE, distance_cm)``; surfacing
                    # ``seek_advance_cm`` here would diverge the sim's primitive
                    # stream from the compiler/CTPS consumers (T7 byte-identical).
                    # ``seek_advance_cm`` is the chassis-model displacement only
                    # (applied above) -- it lands the axle delta-short without
                    # changing what the robot is commanded to do.
                    yield (ACT_STOP_AT_LINE, distance_cm)
                else:
                    # Pivots are FRONT-only in v1 (ADR-036 / #54): the force-FRONT
                    # above sets ``direction = FRONT`` whenever a pivot is active,
                    # so a REAR seek can NEVER carry an approaching pivot and this
                    # branch needs no overshoot trim. That invariant is documented
                    # (and enforced) at the force-FRONT site -- not re-asserted
                    # here, where it would be unreachable dead code. #54 (REAR
                    # pivots) must add a REAR overshoot trim on this branch when it
                    # relaxes the force-FRONT (see the ADR-044 trigger).
                    #
                    # 2-board REAR stop-at-line: the rear board detects the
                    # crossing, so the chassis re-anchors via
                    # ``on_intersection`` (mirrors the firmware reverse seek +
                    # ``Localization.onIntersection``; AC1). The chassis
                    # advances backward by ``distance_cm`` to the node.
                    # ADR-036: sensor-terminated -- no distance trim; the outgoing
                    # carry is owned by ``_pending_pivot`` -> next hop's
                    # ``_departing_pivot``.
                    self._chassis.reverse(distance_cm)
                    self._localization.on_intersection(nxt)
                    yield (ACT_STOP_AT_LINE_REVERSE, nxt, distance_cm, resolve_blind_mode(edge, cfg))
            else:    # DispatchPrimitive.BLIND_DEAD_RECKON
                # Dead-reckoned blind move (byte-identical to the pre-resolver
                # output when no pivot trim applies): a non-STOP_AT_LINE edge, or a
                # forced-REAR STOP_AT_LINE hop on a 1-board chassis with no rear
                # board to seek with (AC2). ADR-036: DISTANCE-terminated -- the
                # departing + approach trims are folded into the classifier's
                # ``commanded_distance_cm`` (the ``_pivot_trim_distance``
                # equivalent); the table cell is untouched and the value matches
                # what the firmware commands into Drive.driveDist.
                effective_dist_cm = decision.commanded_distance_cm

                # -----------------------------------------------------------------
                # ADR-059 blind-chain resolution (mirror of the firmware dispatch
                # site). Exactly one of three paths applies, matching
                # `_dispatchEdgeAfterTurn`'s (a)/(b)/(c):
                #   (a) a run is IN FLIGHT and this hop is its next hop — take the
                #       resolved (start_pwm, end_pwm, brake) from the run buffer;
                #   (b) no run + chaining enabled — try to START one via the
                #       walk-ahead; on success this hop is the run's hop 0;
                #   (c) chaining disabled / no eligible run — the ordinary per-hop
                #       dispatch (AC12: byte-identical to the compiler / pre-ADR-059,
                #       so NO speed suffix rides the tuple).
                # The sim has no encoder-noise boundary carry (AC9: the measured
                # carry is 0 wherever encoders are exact — the sim and the
                # parked-encoder parity harness), so the dispatched distance is the
                # planned `effective_dist_cm` on every hop.
                # -----------------------------------------------------------------
                # This hop's resolved cruise magnitude (per-edge SpeedTier ->
                # PWM; ADR-064). Every blind PWM below derives from it: the
                # pass-through entry-seed clamp, the RAW_FLAT flat plan, and the
                # blind-chain cruise. NORMAL stays byte-identical to the old
                # hardcoded SPEED_NORMAL.
                cruise_mag = _speed_tier_to_pwm(resolve_speed(edge, cfg))

                # `entry_pwm`: captured cruise after a pass-through (ADR-023 carry,
                # AC5), else the cold-start +/-TSHAPE_START_SPEED seed. KNOWN
                # SIM LIMITATION (pre-existing, not introduced by ADR-064): the sim
                # does not model per-tier LineFollower presets, so it models the
                # captured cruise as SPEED_NORMAL. The firmware carries the TRUE
                # inbound tier (`presetToPwm(LineFollower.getPreset())`), so for a
                # non-NORMAL LINE-into-BLIND pass-through the firmware entry speed can
                # exceed this model (a FAST line into a FAST blind enters at 90 on the
                # robot, 70 here). The all-NORMAL parity field keeps the two surfaces
                # in agreement; closing the gap (seed from the previous hop's resolved
                # tier) is a deferred sim-fidelity follow-up. Regardless, the seed is
                # CLAMPED to this hop's resolved cruise so a fast entry into a slower
                # blind hop never exceeds the hop's own cruise (ADR-064 / AC12: a FAST
                # line into a SLOW blind enters at <= 60). The cold-start seed
                # (TSHAPE_START_SPEED = 35) is already below every tier, so only the
                # pass-through branch needs the clamp. `resolve_chain_speeds` re-clamps
                # into [pwm_min, |cruise|] on top.
                if pass_through_active:
                    entry_mag = min(SPEED_NORMAL, cruise_mag)
                else:
                    entry_mag = TSHAPE_START_SPEED
                entry_pwm = -entry_mag if decision.reverse else entry_mag

                chain_hop = False        # this hop is part of a chained run
                chain_final = False      # ... and it is the run's braked final hop
                hop_start_pwm = entry_pwm
                hop_end_pwm = 0

                if self._chain.active:
                    k = self._chain.hop_index + 1
                    match = (
                        k < self._chain.hop_count
                        and current == self._chain.nodes[k]
                        and nxt == self._chain.nodes[k + 1]
                        and decision.reverse == self._chain.reverse
                        and decision.drive_mode != BlindMode.RAW_FLAT
                    )
                    if match:
                        self._chain.hop_index = k
                        chain_hop = True
                        hop_start_pwm = self._chain.speeds[k].start_pwm
                        chain_final = self._chain.speeds[k].brake
                        hop_end_pwm = 0 if chain_final else self._chain.speeds[k].end_pwm
                    else:
                        # Divergence from the walked plan — unreachable by
                        # construction on a static NEXT_HOP table (a mid-run turn
                        # already ends the run); reset defensively and dispatch this
                        # hop as an ordinary per-hop move.
                        self._chain = _ChainRun()
                elif PATH_BLIND_CHAIN_ENABLED:
                    self._try_start_blind_chain(
                        current,
                        nxt,
                        goal_name,
                        reverse=decision.reverse,
                        hop0_mode=decision.drive_mode,
                        hop0_dist_cm=effective_dist_cm,
                        hop0_body_heading=body_heading,
                        entry_pwm=entry_pwm,
                        cfg=cfg,
                        route_direction=route_direction,
                        arrival_direction=arrival_direction,
                    )
                    if self._chain.active:
                        # Hop 0 of a fresh run: speeds[0].start_pwm is entry_pwm
                        # clamped into [pwm_min, |cruise|] by the planner. hop_count
                        # >= 2, so hop 0 is always a non-braking mid-hop.
                        self._chain.hop_index = 0
                        chain_hop = True
                        chain_final = self._chain.speeds[0].brake
                        hop_start_pwm = self._chain.speeds[0].start_pwm
                        hop_end_pwm = 0 if chain_final else self._chain.speeds[0].end_pwm

                # A speed suffix rides the tuple whenever the sim resolved a
                # NON-default plan (a chained hop, or a pass-through-seeded lone
                # hop) — it carries (start_pwm, end_pwm, chain). A plain cold-start
                # non-chain hop leaves the tuple PRISTINE (AC12: byte-identical to
                # the compiler's `(ACT_FORWARD, dist)` / `(ACT_BLIND_REVERSE, ...)`),
                # and downstream consumers compute the cold-start endpoints.
                # RAW_FLAT never chains: it carries a suffix only when pass-through-
                # seeded, and its end speed is the cruise (its flat open-loop end),
                # so the endpoints are (rawStart, cruise) — matching the firmware.
                emit_suffix = chain_hop or pass_through_active
                if decision.drive_mode == BlindMode.RAW_FLAT:
                    # RAW_FLAT plan endpoints: start = pass-through captured cruise
                    # else cruise; end = cruise (the flat contract). The cruise is
                    # this edge's resolved SpeedTier (ADR-064), not a hardcoded
                    # SPEED_NORMAL, so a `speed: FAST` RAW_FLAT hop runs flat at 90.
                    raw_cruise = -cruise_mag if decision.reverse else cruise_mag
                    hop_start_pwm = entry_pwm if pass_through_active else raw_cruise
                    hop_end_pwm = raw_cruise

                if not decision.reverse:
                    self._chassis.forward(effective_dist_cm)
                    self._localization.on_dead_reckoned_advance(nxt)
                    if emit_suffix:
                        yield (
                            ACT_FORWARD, effective_dist_cm,
                            hop_start_pwm, hop_end_pwm, chain_hop,
                        )
                    else:
                        yield (ACT_FORWARD, effective_dist_cm)
                else:
                    self._chassis.reverse(effective_dist_cm)
                    self._localization.on_dead_reckoned_advance(nxt)
                    if emit_suffix:
                        yield (
                            ACT_BLIND_REVERSE, nxt, effective_dist_cm,
                            resolve_blind_mode(edge, cfg),
                            hop_start_pwm, hop_end_pwm, chain_hop,
                        )
                    else:
                        yield (
                            ACT_BLIND_REVERSE, nxt, effective_dist_cm,
                            resolve_blind_mode(edge, cfg),
                        )

                # ADR-059 `blind_pass` boundary marker: a chained NON-final hop
                # passes the node WITHOUT braking, so emit the pass tag AFTER the
                # hop's dead-reckon record and BEFORE moving on (the firmware emits
                # `_ptEmitBlindPass(_nextHop)` in BLIND_FORWARD, between the two
                # adjacent hops' `blind_dead_reckon` records). The run's FINAL
                # (braked) hop emits NO pass tag, and the run retires here so a
                # fresh chain may re-form on the far side.
                if chain_hop and not chain_final:
                    yield (ACT_BLIND_PASS, nxt)
                if chain_hop and chain_final:
                    self._chain = _ChainRun()

            # Wall-zero reset interlude (PR2, AC17 pillar 2). The hop above just
            # re-anchored localization onto `nxt`. If `nxt` is a reset node, run
            # the WALL_STOP -> REANCHOR interlude NOW -- before the loop advances
            # to plan the next hop -- mirroring the firmware's `_maybeEnterReset`
            # at the top of `_onPlanNextHop`. This fires whether `nxt` is an
            # intermediate hop (the loop continues afterward) OR the final
            # destination (the loop then exits because `current == goal_name`).
            if self._localization.is_reset_node(nxt):
                for primitive in self._dispatch_wall_reset(nxt):
                    yield primitive

            # Line-arrival heading reset (ADR-052). Mirror of the firmware's
            # `_maybeHeadingReset`, fired at the same logical instant as
            # `PathTracker::_onPlanNextHop` -- AFTER the wall-reset interlude,
            # once per hop's arrival at `nxt`. Fires only on the accept path:
            # the sim has no IMU/drift model, so it never strict-rejects (the
            # firmware's strict-gate reject is host-C++-only, not mirrored
            # here) -- whenever the pass-through + knob condition holds, a
            # driftless bench firmware would accept, so the sim always emits.
            # `heading_reset_kind` is `None` unless THIS hop's arrival was a
            # collinear pass-through (latched above); a reset/blind/stop-and-go
            # arrival leaves it `None`, matching the firmware's BLIND/RESET
            # no-op branch and non-pass-through no-op. `body_heading` is this
            # hop's already-REAR-corrected body heading -- the sim's
            # equivalent of `_resolvedBodyHeading` / `incomingBodyHeadingDeg`,
            # the heading the chassis arrives at `nxt` on. No pose mutation:
            # the sim's heading is already ground truth, there is nothing to
            # correct.
            if heading_reset_kind == "proj":
                if LINE_HEADING_RESET_ON_PROJECTION:
                    yield (ACT_HEADING_RESET, nxt, "proj", body_heading)
            elif heading_reset_kind == "junction":
                if LINE_HEADING_RESET_ON_JUNCTION:
                    yield (ACT_HEADING_RESET, nxt, "junction", body_heading)

            # ADR-023 pass-through carry for the NEXT hop's BLIND `entry_pwm`
            # (mirror of the firmware `_passThrough` flag): a LINE / projection
            # collinear pass-through keeps the follower live at cruise across the
            # node, so a blind hop departing this node seeds the captured cruise,
            # not the cold-start floor. Set iff THIS hop emitted a pass tag; reset
            # otherwise so a stop-and-go arrival correctly seeds the cold start.
            pass_through_active = heading_reset_kind is not None

            current = nxt

    def _reset_pivot_latch(self) -> None:
        """Clear the three pivot-latch members (and the blind-chain run) to inactive.

        ADR-036: each goal starts with a clean pivot latch so a pivot whose
        FINAL hop was the previous goal's last hop (leaving ``_pending_pivot``
        active with no following hop to consume it) never fires across a
        ``go_to`` boundary -- a stop at the pivot node degrades to a normal
        turn. Mirror of the firmware's ``_resetPivotLatch()``; factored out so
        the two public entry points (:meth:`go_to`, :meth:`step_through`) cannot
        drift if a fourth latch member is ever added (parity contract).

        ADR-059: also resets the blind-chain run buffer here (the firmware
        retires a chain at its final hop's dispatch, but a fresh goal must never
        inherit a stale run). Bundled into this one goal-start reset so the two
        entry points cannot diverge on which state is cleaned.
        """
        self._pending_pivot = None
        self._departing_pivot = None
        self._approaching_pivot = None
        self._chain = _ChainRun()

    def _try_start_blind_chain(
        self,
        cur: str,
        nxt: str,
        goal_name: str,
        *,
        reverse: bool,
        hop0_mode: BlindMode,
        hop0_dist_cm: float,
        hop0_body_heading: float,
        entry_pwm: int,
        cfg: PipelineConfig,
        route_direction: "Direction",
        arrival_direction: "Direction",
    ) -> None:
        """Walk NEXT_HOP forward from a BLIND hop-0, folding a collinear run.

        Function-identical Python mirror of the firmware
        ``PathTrackerClass::_chainTryStart`` (ADR-059). On entry the hop being
        dispatched (hop 0: ``cur -> nxt``) is already classified
        BLIND_DEAD_RECKON by :meth:`_walk_hops`. This method decides whether hop
        0 STARTS a chained run and, if so, how far it extends. It leaves
        :attr:`_chain` INACTIVE unless a genuine ``>= 2``-hop run forms (AC12: a
        non-chain hop takes the ordinary per-hop path untouched, byte-identical
        to the compiler).

        Every ``break`` ENDS the run BEFORE the hop under evaluation (Error
        Handling): that hop dispatches later as a normal braked hop. The same
        ineligibility reasons the firmware uses end the chain: a pivot-adjacent
        joint, a seek (``stopAtLine``) hop, a RAW_FLAT motion bundle, a
        ``< TSHAPE_MIN_MOVE_CM`` hop, a non-collinear bend / direction flip, a
        goal / reset node, and the two run caps (:data:`MAX_CHAIN_HOPS` /
        :data:`PATH_BLIND_CHAIN_MAX_CM`). The pairwise walk updates the inbound
        BODY heading per admitted hop (the outbound edge heading, +180° wrapped
        iff REAR), exactly as ``_onPlanNextHop`` computes ``bodyTarget``.

        :param cur: hop 0's arrive-from node.
        :param nxt: hop 0's destination (``cur -> nxt`` is hop 0).
        :param goal_name: the active ``go_to`` destination.
        :param reverse: hop 0's resolved travel direction (uniform along the run).
        :param hop0_mode: hop 0's resolved :class:`BlindMode` motion bundle.
        :param hop0_dist_cm: hop 0's COMMANDED (pivot-trimmed) distance — the run
            total includes it (matches the firmware's ``totalCm = hop0DistCm``).
        :param hop0_body_heading: hop 0's resolved body heading (seeds the walk).
        :param entry_pwm: the signed PWM the chassis is physically at when the
            run begins (the ADR-023 captured cruise after a pass-through, else the
            +/-TSHAPE_START_SPEED cold-start seed) — the forward-pass seed (AC5).
        :param cfg: the resolved pipeline config for this walk.
        :param route_direction: the builder route override (every hop).
        :param arrival_direction: the builder arrival override (final hop only).
        """
        # ---- Hop-0 eligibility (mirror of _chainTryStart's three early returns).
        # RAW_FLAT has no ramps to chain; a sub-floor hop silently degrades the
        # Drive profile; an active approaching pivot means the chassis must stop
        # delta short of `nxt` and pivot there. All three make hop 0 ordinary.
        if hop0_mode == BlindMode.RAW_FLAT:
            return
        if hop0_dist_cm < TSHAPE_MIN_MOVE_CM:
            return
        if self._approaching_pivot is not None:
            return

        # Hop 0's resolved speed tier defines the whole run's cruise (ADR-064): a
        # chained run shares one velocity profile, so the walk-ahead below breaks
        # at any hop whose resolved tier differs (SPEED-only restriction; mode
        # mixing SYNC<->GYRO still chains). Mirrors the firmware `_chainTryStart`.
        hop0_tier = resolve_speed(self._graph.edges[cur, nxt], cfg)
        cruise_mag = _speed_tier_to_pwm(hop0_tier)

        nodes: List[str] = [cur, nxt]
        hop_dist_cm: List[float] = [hop0_dist_cm]
        count = 1                     # hops admitted so far (hop 0 is in)
        total_cm = hop0_dist_cm       # summed COMMANDED run distance
        inbound_body = hop0_body_heading    # hop 0's body heading seeds the walk

        # ---- Walk NEXT_HOP forward, admitting one hop per iteration. -----------
        while True:
            a = nodes[count]          # node the run currently ends on

            # Goal: a run may END on the goal but never continues past it. RESET:
            # the wall-zero interlude needs a stopped chassis, so brake there too.
            if a == goal_name:
                break
            if self._localization.is_reset_node(a):
                break

            b = self._next_hop[a][goal_name]
            if b is None or b == a:
                break    # no route / self-loop (same guards as _peek_next_next_hop)

            edge = self._graph.edges[a, b]
            if edge["type"] != EDGE_BLIND:
                break    # a LINE hop is the line-follower's domain — run ends here

            # Pivot guards (defensive: a validator-passed pivot is never
            # collinear, so the heading predicate below would reject these
            # joints anyway). A pivot AT the joint (prev, a, b) or a pivot ONTO b
            # (a, b, c) both brake.
            if self.find_pivot_turn(nodes[count - 1], a, b) is not None:
                break
            if b != goal_name:
                c = self._next_hop[b][goal_name]
                if c is not None and c != b and self.find_pivot_turn(a, b, c) is not None:
                    break

            # Collinear + same-direction — the PURE predicate (a bend or a
            # FRONT<->REAR flip ends the run; the direction stays constant).
            if not _pass_through_heading_eligible(
                edge,
                outbound_is_last_hop=(b == goal_name),
                inbound_body_heading=inbound_body,
                inbound_direction=(Direction.REAR if reverse else Direction.FRONT),
                pipeline_default=cfg.direction,
                builder_route=route_direction,
                builder_arrival=arrival_direction,
                bidirectional_enabled=BIDIRECTIONAL_AUTO_ENABLED,
                line_board_count=LINE_BOARD_COUNT,
            ):
                break

            # Seek / motion-bundle resolution — the SAME resolver inputs the
            # dispatch site builds. A seek hop is chain-ineligible (the seek is a
            # fresh braked hop); a RAW_FLAT bundle has no ramps to chain.
            auto_rule_active = (
                getattr(self._field_data, "BLIND_PROJECTION_AUTO_SEEK", True)
                and is_projection_node(self._graph, b)
            )
            seek, motion_mode = resolve_stop_at_line(
                StopAtLine.AUTO,
                _read_edge_stop_at_line_override(edge),
                resolve_blind_mode(edge, cfg),
                cfg.blind,
                Direction.REAR if reverse else Direction.FRONT,
                LINE_BOARD_COUNT,
                auto_rule_active,
            )
            if seek:
                break
            if motion_mode == BlindMode.RAW_FLAT:
                break

            # SPEED-only chain restriction (ADR-064): a hop whose resolved speed
            # tier differs from hop 0's ends the run — one chained run carries a
            # single cruise, so it cannot straddle a tier boundary. Mode mixing
            # (SYNC<->GYRO) still chains (only SPEED gates), matching the firmware
            # `_chainTryStart` walk-ahead restriction.
            if resolve_speed(edge, cfg) != hop0_tier:
                break

            # A hop below the profile floor cannot carry chained start/end speeds.
            if float(edge["distance_cm"]) < TSHAPE_MIN_MOVE_CM:
                break

            # ---- Run caps (mirror of the firmware's LOGGED truncations). -------
            # The sim has no fixed-size buffer, but honors the SAME caps so its
            # output matches the firmware (ADR-050 parity). The truncation Serial
            # log is firmware-only; the sim just ends the run at the cap.
            if count >= MAX_CHAIN_HOPS:
                break
            if total_cm + float(edge["distance_cm"]) > PATH_BLIND_CHAIN_MAX_CM:
                break

            # ---- Admit the hop. Mid-run hops carry the raw edge length (no pivot
            #      can trim them — the pivot guards above ended the run at any
            #      pivot-adjacent joint, so the walk-time distance equals the
            #      dispatch-time value exactly). -------------------------------
            hop_dist_cm.append(float(edge["distance_cm"]))
            nodes.append(b)
            count += 1
            total_cm += float(edge["distance_cm"])

            # The next joint's inbound body heading is THIS hop's body heading
            # (REAR rotates 180°, matching `_onPlanNextHop`'s bodyTarget math).
            inbound_body = _body_heading_for(
                Direction.REAR if reverse else Direction.FRONT,
                float(edge["heading_deg"]),
            )

        if count < 2:
            return    # not a chain — hop 0 dispatches on the ordinary per-hop path

        # ---- Resolve the per-hop speed plan (pure two-pass planner). -----------
        # `entry_pwm` seeds the forward pass (AC5); the final hop's plan is
        # `{start_pwm, 0, brake=True}` by construction. cruise = +/-cruise_mag,
        # hop 0's resolved SpeedTier PWM (ADR-064; the whole tier-homogeneous run
        # shares it). Sign carries the run direction, matching the firmware
        # `cruisePwm`.
        cruise_pwm = -cruise_mag if reverse else cruise_mag
        speeds = resolve_chain_speeds(hop_dist_cm, count, cruise_pwm, entry_pwm)

        self._chain = _ChainRun(
            active=True,
            reverse=reverse,
            nodes=nodes,
            speeds=speeds,
            hop_count=count,
            hop_index=-1,
        )

    def go_to(
        self,
        goal_name: str,
        *,
        route_direction: "Direction" = Direction.AUTO,
        arrival_direction: "Direction" = Direction.AUTO,
    ) -> List[Primitive]:
        """Walk from the current node to ``goal_name``.

        Emits per edge: an optional ``ACT_TURN_REL`` to align heading,
        then the per-hop motion primitive derived from
        ``(edge_type, destination_node_type)``. For an EDGE_LINE hop into a
        junction (line) node, an ``ACT_LINE_OVERSHOOT`` step is appended after
        ``ACT_FOLLOW_INT``, mirroring the runtime LINE_OVERSHOOT FSM transition
        that advances the chassis by :data:`LINE_SENSOR_AXLE_OFFSET_CM` so the
        wheel axle (not the line sensor) lands on the node. An EDGE_LINE hop
        into a projection node (issue #64) instead emits ``ACT_FOLLOW_DIST``
        and advances Localization via
        :meth:`LocalizationSim.on_dead_reckoned_advance` (no overshoot step --
        the encoder distance gate already lands the chassis on the node). When
        that projection node is a non-goal middle node of a collinear
        ``line(A)–projection(B)–line(C)`` triple routed PAST it (T20), the hop
        instead emits ``ACT_FOLLOW_DIST_PASS`` -- the distance-gated pass-through
        that keeps the follower live for one continuous A→C motion (see
        :func:`_should_projection_pass_through`).

        Per-edge direction resolution mirrors the runtime
        ``PathTracker::_resolveDirection``: :func:`resolve_direction` is
        called for every hop with ``route_direction`` /
        ``arrival_direction`` as the builder inputs. REAR hops:

        * Turn the chassis to ``(edge.heading_deg + 180°) mod 360`` (so
          the rear sensor faces along the edge) before dispatch.
        * Emit ``ACT_LINE_REVERSE`` / ``ACT_PROJECTION_REVERSE`` /
          ``ACT_BLIND_REVERSE`` instead of the forward primitive.
        * Advance the chassis backward via ``VirtualChassis.reverse``
          for BLIND (mirrors how FRONT uses ``forward``).

        Return-to-node prologue (issue #36, PR1b): before walking NEXT_HOP,
        the accumulated operator offset is evaluated through the same gate the
        firmware uses (:func:`resolve_return_action`). An offset above the
        trigger first drives the chassis back onto the anchored node (the
        return primitive(s) are PREPENDED to the returned stream), an offset
        over the cap honors the off-map gate (FAULT raises
        :class:`OffMapFaultError`; BEST_EFFORT proceeds capped), and an offset
        at or below the trigger is a true no-op -- the returned stream is then
        byte-identical to today's (AC5). The nav-active gate is held ``True``
        across the planned walk so the walk's own motion never grows the
        offset, mirroring the firmware's ``setNavActive`` push.

        :param goal_name: destination node name (must be in
            ``field_data.NODES``).
        :param route_direction: builder override applied to every hop
            (``Direction.AUTO`` = no override; runs the auto algorithm).
        :param arrival_direction: builder override applied only to the
            final hop (``Direction.AUTO`` = no override).
        :raises KeyError: if ``goal_name`` is not a known node.
        :raises RuntimeError: if ``zero_heading_from_current()`` was not called.
        :raises OffMapFaultError: if the offset exceeds the cap under the FAULT
            off-map gate (mirror of ``Fault::OFF_NODE``).
        :raises UnreachableGoalError: if any intermediate hop is unreachable.
        """
        if goal_name not in self._field_data.NODES:
            raise KeyError(f"goal {goal_name!r} not in field_data.NODES")
        if not self._localization.is_zeroed():
            raise RuntimeError(
                "PathTrackerSim.go_to() called before "
                "LocalizationSim.zero_heading_from_current()"
            )

        # ADR-036: each goal starts with a clean pivot latch (mirror of the
        # firmware's DEQUEUE / goal-complete resets). Done here (not in
        # ``_walk_hops``) so the reset semantics stay owned by the public entry
        # point. See :meth:`_reset_pivot_latch` for the rationale.
        self._reset_pivot_latch()

        # Return-to-node prologue (PR1b). Evaluate the operator offset BEFORE
        # planning: an over-cap FAULT raises (no chassis motion); otherwise the
        # return primitive(s) (possibly empty) are prepended to the planned
        # stream. `_evaluate_return_gate` is a generator (so step_through can
        # snapshot each return primitive); force it EAGERLY here with list(...)
        # so the chassis mutation + re-anchor (and any FAULT-gate raise) happen
        # before the walk, exactly as the non-generator version did.
        return_prims = list(self._evaluate_return_gate(goal_name))

        # The planned walk's own motion must NOT grow the operator offset
        # (mirror of the firmware's setNavActive(true) while a hop executes).
        # Hold the gate closed across the walk, then reopen it so a later raw
        # operator_move between goals accumulates again.
        self._localization.set_nav_active(True)
        try:
            planned = list(
                self._walk_hops(
                    goal_name,
                    route_direction=route_direction,
                    arrival_direction=arrival_direction,
                )
            )
        finally:
            self._localization.set_nav_active(False)

        return return_prims + planned

    def walked_path(self, goal_name: str) -> List[str]:
        return walk_next_hop(self._next_hop, self._localization.current_node(), goal_name)

    def step_through(
        self,
        goal_name: str,
        *,
        route_direction: "Direction" = Direction.AUTO,
        arrival_direction: "Direction" = Direction.AUTO,
    ):
        """Generator variant of :meth:`go_to` yielding ``(primitive, snapshot)``.

        Mirrors :meth:`go_to`'s emission shape exactly, including direction
        resolution, the return-to-node prologue (PR1b), and ACT_*_REVERSE
        primitives for REAR hops. The return primitive(s), if any, are yielded
        FIRST (each paired with a post-mutation snapshot), then the planned
        walk.

        :param goal_name: destination node name.
        :param route_direction: builder override applied to every hop.
        :param arrival_direction: builder override applied only to the
            final hop.
        :raises OffMapFaultError: if the offset exceeds the cap under the FAULT
            off-map gate (mirror of ``Fault::OFF_NODE``).
        """
        if goal_name not in self._field_data.NODES:
            raise KeyError(f"goal {goal_name!r} not in field_data.NODES")
        if not self._localization.is_zeroed():
            raise RuntimeError(
                "PathTrackerSim.step_through() called before "
                "LocalizationSim.zero_heading_from_current()"
            )

        # ADR-036: clean pivot latch per goal (see go_to for the rationale).
        self._reset_pivot_latch()

        # Return-to-node prologue (PR1b). Yield the return primitive(s) first. The gate is a
        # generator that yields each primitive AFTER its own chassis mutation (the pre-edge
        # turn, then the seek/drive), so the snapshot taken here is the pose AT that primitive
        # -- the pre-edge turn pairs with the post-turn pose, the move with the post-move pose,
        # honouring the same "snapshot == state at this primitive" invariant the planned walk
        # holds (#7). An over-cap FAULT raises before any yield, matching go_to.
        for primitive in self._evaluate_return_gate(goal_name):
            yield primitive, self._snapshot_chassis()

        # Drive the shared per-hop generator and pair each primitive with a
        # chassis snapshot. ``_walk_hops`` yields every primitive AFTER its own
        # chassis mutation (and any localization re-anchor), so the snapshot
        # taken here captures the identical post-mutation state the previous
        # inline implementation recorded. Hold the nav-active gate closed across
        # the walk so the walk's motion never grows the offset (mirror go_to).
        self._localization.set_nav_active(True)
        try:
            for primitive in self._walk_hops(
                goal_name,
                route_direction=route_direction,
                arrival_direction=arrival_direction,
            ):
                yield primitive, self._snapshot_chassis()
        finally:
            self._localization.set_nav_active(False)

    def _snapshot_chassis(self) -> VirtualChassis:
        return VirtualChassis(
            x_cm=self._chassis.x_cm,
            y_cm=self._chassis.y_cm,
            heading_deg=self._chassis.heading_deg,
        )
