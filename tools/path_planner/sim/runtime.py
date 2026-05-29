"""PathTracker FSM mirror in pure Python (verification harness).

Mirrors the C++ PathTracker FSM closely enough that its dispatched-primitive
sequence matches the runtime PathTracker for the same NEXT_HOP-walked path.
Not tick-by-tick faithful -- each Drive/LineFollower dispatch + completion is
collapsed into a single atomic transition. That is enough for end-to-end
planning + heading-propagation verification on any graph without hardware.

Edge dispatch (mirrors :func:`tools.path_planner.compiler.compile_path` plus
the runtime-only LINE_OVERSHOOT FSM transition):

* :data:`EDGE_LINE` -- emit ``(ACT_FOLLOW_INT, None)`` to walk to the
  intersection marker, **then** emit a runtime-only
  ``(ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)`` step that advances
  the chassis by the line-sensor-to-axle offset so the wheel axle (not
  the sensor) ends up on the node. Localization then advances via
  :meth:`LocalizationSim.on_intersection`.
* :data:`EDGE_PROJECTION` -- emit ``(ACT_FOLLOW_DIST, distance_cm)``
  (line-follow stopped by encoder distance, no intersection marker).
  Localization advances via :meth:`LocalizationSim.on_dead_reckoned_advance`
  (NOT ``on_intersection`` — the chassis is dead-reckoning into the node
  via the encoder gate, not snapping to a physical marker). No
  LINE_OVERSHOOT step follows -- the encoder gate already lands the
  chassis on the node.
* :data:`EDGE_BLIND` -- emit ``(ACT_FORWARD, distance_cm)``; Localization
  advances via :meth:`LocalizationSim.on_dead_reckoned_advance`.

Public surface:

* :class:`LocalizationSim` -- mirror of ``LocalizationClass``.
* :class:`PathTrackerSim` -- the FSM. ``go_to(goal_name)`` walks NEXT_HOP.
* :func:`walk_next_hop` -- given a NextHopTable, returns the explicit
  node-name list from source to target.
* :func:`resolve_line_mode` / :func:`resolve_blind_mode` /
  :func:`resolve_projection_mode` / :func:`resolve_pre_turn_mode` -- pure
  helpers that mirror the C++ ``_resolve*Mode(EdgeInfo)`` lookups: read
  the edge's optional override attribute and fall back to
  :data:`PIPELINE_CONFIG` when the override is absent or set to its
  ``DEFAULT`` sentinel.
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
"""

from __future__ import annotations

from enum import IntEnum
from typing import List, Optional

import networkx as nx

from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_TURN_REL,
    Primitive,
    normalize_turn,
)
from tools.path_planner import pipeline_config as _pipeline_config
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION
from tools.path_planner.pipeline_config import (
    PIPELINE_CONFIG,
    BlindMode,
    DirectionMode,
    LineMode,
    ProjectionMode,
    TurnMode,
)
from tools.path_planner.planner import NextHopTable
from tools.path_planner.sim.chassis import VirtualChassis, _normalize

TURN_EPSILON_DEG: float = 1e-6

# MUST mirror ``Config::LINE_SENSOR_AXLE_OFFSET_CM`` in WRO2026/Config.h.
# Python cannot import C++ headers, so this constant is hand-mirrored. If
# the C++ side changes, update this value too -- the sim's LINE_OVERSHOOT
# advance must match the runtime's ``LineFollower.setDistanceTarget`` budget
# or the cross-check goal-position test will diverge.
LINE_SENSOR_AXLE_OFFSET_CM: float = 5.0

# MUST mirror ``Config::NAV_TURN_TOLERANCE_DEG`` in WRO2026/Config.h.
# Pytest TC9 verifies parity via regex grep on the header (no C parser).
NAV_TURN_TOLERANCE_DEG: float = 0.5

# MUST mirror ``Config::PATH_PASS_THROUGH_ENABLED`` in WRO2026/Config.h.
# Pytest TC9 verifies parity via the same regex pattern (see
# ``test_tc9_path_pass_through_enabled_parity``). TC8 covers the
# behavioral disable, not the constant-value parity check.
PATH_PASS_THROUGH_ENABLED: bool = True

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

# Bidirectional line-following primitive tags. Each mirrors its forward
# counterpart (``ACT_FOLLOW_INT`` / ``ACT_FORWARD`` / ``ACT_FOLLOW_DIST``)
# but drives the chassis BACKWARD along its current heading instead of
# rotating in place. Emitted by :class:`PathTrackerSim.go_to` (T11+) when
# :func:`resolve_direction` selects REVERSE for the upcoming edge based on
# the turn delta vs :data:`BIDIRECTIONAL_TURN_THRESHOLD_DEG`. The C++
# runtime owns the matching ``EDGE_*_REVERSE`` dispatch path; the sim's
# emission must stay byte-identical to the compiler's output (TC18 parity).
ACT_LINE_REVERSE: str       = "line_reverse"          # (ACT_LINE_REVERSE, target_node, line_mode)
ACT_BLIND_REVERSE: str      = "blind_reverse"         # (ACT_BLIND_REVERSE, target_node, distance_cm, blind_mode)
ACT_PROJECTION_REVERSE: str = "projection_reverse"    # (ACT_PROJECTION_REVERSE, target_node, distance_cm, projection_mode)


# MUST mirror ``Config::BIDIRECTIONAL_ENABLED`` in WRO2026/Config.h.
# Python cannot import C++ headers, so this constant is hand-mirrored.
# Pytest TC14a/b verify parity via regex grep on the header (no C parser);
# see ``test_tc14_bidirectional_*_parity`` for the defending tests.
BIDIRECTIONAL_ENABLED: bool = True

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


def wrap_pm180(delta_deg: float) -> float:
    """Wrap a heading delta into ``(-180, +180]`` degrees.

    Thin alias of :func:`tools.path_planner.compiler.normalize_turn`,
    re-exposed under the name the resolution algorithm uses in the plan
    so the Python code reads identically to the C++ helper.
    """
    return normalize_turn(delta_deg)


__all__ = [
    "ACT_BLIND_REVERSE",
    "ACT_LINE_OVERSHOOT",
    "ACT_LINE_PASS",
    "ACT_LINE_REVERSE",
    "ACT_PROJECTION_REVERSE",
    "BIDIRECTIONAL_ENABLED",
    "BIDIRECTIONAL_TURN_THRESHOLD_DEG",
    "LINE_BOARD_COUNT",
    "LINE_SENSOR_AXLE_OFFSET_CM",
    "NAV_TURN_TOLERANCE_DEG",
    "PATH_PASS_THROUGH_ENABLED",
    "TURN_EPSILON_DEG",
    "Direction",
    "LocalizationSim",
    "PathTrackerSim",
    "UnreachableGoalError",
    "resolve_blind_mode",
    "resolve_direction",
    "resolve_line_mode",
    "resolve_pre_turn_mode",
    "resolve_projection_mode",
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
    # Tolerate raw int values (the graph attribute could be persisted as an
    # int in some fixtures); coerce via the enum so type stays consistent.
    coerced = enum_cls(override)
    if coerced == enum_cls.DEFAULT:
        return default_value
    return coerced


def resolve_line_mode(edge) -> LineMode:
    """Return the effective :class:`LineMode` for an EDGE_LINE traversal.

    :param edge: directed graph edge attribute mapping (e.g.
        ``graph.edges[u, v]``). May or may not carry ``primitive_mode``.
    :return: the override if set (non-DEFAULT), else
        ``PIPELINE_CONFIG.line``.
    """
    return _resolve_override(edge, "primitive_mode", PIPELINE_CONFIG.line, LineMode)


def resolve_blind_mode(edge) -> BlindMode:
    """Return the effective :class:`BlindMode` for an EDGE_BLIND traversal."""
    return _resolve_override(edge, "primitive_mode", PIPELINE_CONFIG.blind, BlindMode)


def resolve_projection_mode(edge) -> ProjectionMode:
    """Return the effective :class:`ProjectionMode` for an EDGE_PROJECTION traversal."""
    return _resolve_override(
        edge, "primitive_mode", PIPELINE_CONFIG.projection, ProjectionMode
    )


def resolve_pre_turn_mode(edge) -> TurnMode:
    """Return the effective :class:`TurnMode` for the turn preceding an edge."""
    return _resolve_override(edge, "pre_turn_mode", PIPELINE_CONFIG.turn, TurnMode)


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

    Function-identical Python mirror of the C++ ``PathTracker::_resolveDirection``
    helper (T5). The algorithm is fully specified in the plan
    "API Contracts — Resolution algorithm" section; both implementations
    must stay byte-for-byte equivalent so the offline sim and the robot
    agree on every hop's direction (TC18 parity).

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
        REAR step (mirrors ``Config::BIDIRECTIONAL_ENABLED``).
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


def _read_edge_dir_override(edge) -> DirectionMode:
    """Coerce the graph edge's ``direction_override`` attribute to
    :class:`DirectionMode`.

    Tolerates absence (returns ``DirectionMode.DEFAULT``), raw int values
    (coerced via the enum constructor), and already-typed
    :class:`DirectionMode` instances (returned as-is). Mirrors the
    tolerance pattern of :func:`_resolve_override`.
    """
    raw = edge.get("direction_override") if hasattr(edge, "get") else None
    if raw is None:
        return DirectionMode.DEFAULT
    if isinstance(raw, DirectionMode):
        return raw
    return DirectionMode(raw)


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


def _should_pass_through(
    graph,
    next_hop,
    intersected_node,
    goal_name,
    *,
    inbound_body_heading: float,
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
        override; ``None`` resolves to :data:`BIDIRECTIONAL_ENABLED`.
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
    next_edge_heading = float(next_edge["heading_deg"])

    # Resolve the next-next hop's direction using the inbound body heading
    # as `current_heading`. This is the critical fix vs the old raw-edge
    # peek: on a mixed REAR -> FORWARD transition with anti-parallel
    # forward edges, the raw-edge delta would be 180° (and incorrectly
    # block pass-through) but the resolved body-headings are collinear.
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
        bidirectional_enabled if bidirectional_enabled is not None else BIDIRECTIONAL_ENABLED
    )
    resolved_board_count = (
        line_board_count if line_board_count is not None else LINE_BOARD_COUNT
    )

    next_dir = resolve_direction(
        edge_dir_override=_read_edge_dir_override(next_edge),
        pipeline_default=resolved_pipeline,
        builder_arrival=resolved_arrival,
        builder_route=resolved_route,
        current_heading=inbound_body_heading,
        edge_heading=next_edge_heading,
        is_last_hop=(next_next == goal_name),
        bidirectional_enabled=resolved_enabled,
        line_board_count=resolved_board_count,
    )
    next_body_heading = _body_heading_for(next_dir, next_edge_heading)

    heading_delta = wrap_pm180(next_body_heading - inbound_body_heading)
    return abs(heading_delta) <= NAV_TURN_TOLERANCE_DEG


class LocalizationSim:
    """Mirror of ``LocalizationClass``."""

    def __init__(self, field_data) -> None:
        self._current_node = field_data.START_NODE
        self._zeroed = False
        self._field_data = field_data

    def zero_heading_from_current(self) -> bool:
        self._zeroed = True
        return True

    def is_zeroed(self) -> bool:
        return self._zeroed

    def current_node(self) -> str:
        return self._current_node

    def on_intersection(self, advance_to: str) -> None:
        self._current_node = advance_to

    def on_dead_reckoned_advance(self, advance_to: str) -> None:
        self._current_node = advance_to


class PathTrackerSim:
    """Mirror of ``PathTrackerClass`` (LINE + BLIND + PROJECTION).

    Per-edge dispatch:

    * ``EDGE_LINE`` -> ``(ACT_FOLLOW_INT, None)`` followed by a
      sim-only ``(ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)`` step
      mirroring the runtime LINE_OVERSHOOT FSM transition.
    * ``EDGE_PROJECTION`` -> ``(ACT_FOLLOW_DIST, distance_cm)``;
      Localization advances via :meth:`LocalizationSim.on_dead_reckoned_advance`.
    * ``EDGE_BLIND`` -> ``(ACT_FORWARD, distance_cm)``.
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

    @property
    def localization(self) -> LocalizationSim:
        return self._localization

    @property
    def chassis(self) -> VirtualChassis:
        return self._chassis

    # -- mode resolution (instance-method passthroughs) ----------------------

    def resolve_line_mode(self, edge) -> LineMode:
        """See module-level :func:`resolve_line_mode`."""
        return resolve_line_mode(edge)

    def resolve_blind_mode(self, edge) -> BlindMode:
        """See module-level :func:`resolve_blind_mode`."""
        return resolve_blind_mode(edge)

    def resolve_projection_mode(self, edge) -> ProjectionMode:
        """See module-level :func:`resolve_projection_mode`."""
        return resolve_projection_mode(edge)

    def resolve_pre_turn_mode(self, edge) -> TurnMode:
        """See module-level :func:`resolve_pre_turn_mode`."""
        return resolve_pre_turn_mode(edge)

    def go_to(
        self,
        goal_name: str,
        *,
        route_direction: "Direction" = Direction.AUTO,
        arrival_direction: "Direction" = Direction.AUTO,
    ) -> List[Primitive]:
        """Walk from the current node to ``goal_name``.

        Emits per edge: an optional ``ACT_TURN_REL`` to align heading,
        then the per-type motion primitive. For EDGE_LINE traversals, an
        ``ACT_LINE_OVERSHOOT`` step is appended after ``ACT_FOLLOW_INT``,
        mirroring the runtime LINE_OVERSHOOT FSM transition that
        advances the chassis by :data:`LINE_SENSOR_AXLE_OFFSET_CM` so
        the wheel axle (not the line sensor) lands on the node.
        EDGE_PROJECTION emits ``ACT_FOLLOW_DIST`` and advances Localization
        via :meth:`LocalizationSim.on_dead_reckoned_advance` (no overshoot
        step -- the encoder gate already lands the chassis on the node).

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

        :param goal_name: destination node name (must be in
            ``field_data.NODES``).
        :param route_direction: builder override applied to every hop
            (``Direction.AUTO`` = no override; runs the auto algorithm).
        :param arrival_direction: builder override applied only to the
            final hop (``Direction.AUTO`` = no override).
        :raises KeyError: if ``goal_name`` is not a known node.
        :raises RuntimeError: if ``zero_heading_from_current()`` was not called.
        :raises UnreachableGoalError: if any intermediate hop is unreachable.
        """
        if goal_name not in self._field_data.NODES:
            raise KeyError(f"goal {goal_name!r} not in field_data.NODES")
        if not self._localization.is_zeroed():
            raise RuntimeError(
                "PathTrackerSim.go_to() called before "
                "LocalizationSim.zero_heading_from_current()"
            )

        out: List[Primitive] = []
        current = self._localization.current_node()

        while current != goal_name:
            nxt = self._next_hop[current][goal_name]
            if nxt is None:
                raise UnreachableGoalError(current, goal_name, current=current)
            edge = self._graph.edges[current, nxt]
            edge_heading = float(edge["heading_deg"])
            is_last_hop = (nxt == goal_name)

            direction = resolve_direction(
                edge_dir_override=_read_edge_dir_override(edge),
                pipeline_default=_pipeline_config.PIPELINE_CONFIG.direction,
                builder_arrival=arrival_direction,
                builder_route=route_direction,
                current_heading=self._chassis.heading_deg,
                edge_heading=edge_heading,
                is_last_hop=is_last_hop,
                bidirectional_enabled=BIDIRECTIONAL_ENABLED,
                line_board_count=LINE_BOARD_COUNT,
            )
            body_heading = _body_heading_for(direction, edge_heading)

            delta = wrap_pm180(body_heading - self._chassis.heading_deg)
            if abs(delta) > TURN_EPSILON_DEG:
                out.append((ACT_TURN_REL, delta))
                self._chassis.turn_relative(delta)
                self._chassis.heading_deg = _normalize(body_heading)

            edge_type = edge["type"]
            distance_cm = float(edge["distance_cm"])
            if edge_type == EDGE_LINE:
                if direction == Direction.REAR:
                    out.append((ACT_LINE_REVERSE, nxt, resolve_line_mode(edge)))
                    self._chassis.reverse(distance_cm)
                else:
                    out.append((ACT_FOLLOW_INT, None))
                    self._chassis.follow_line_to_intersection(distance_cm)
                # LINE_OVERSHOOT FSM transition -- emit ACT_LINE_PASS when
                # the resolved body-heading of the next-next hop is
                # collinear with the inbound body-heading (delta <=
                # NAV_TURN_TOLERANCE_DEG), else ACT_LINE_OVERSHOOT. The
                # peek MUST use resolved body-headings, not raw forward
                # edge headings, so mixed REAR<->FORWARD transitions on
                # anti-parallel forward edges still pass through cleanly
                # (TC9c). Chassis advance is identical (5cm) regardless
                # of direction; the primitive name differs only for
                # downstream verification + telemetry. See ADR-023.
                if _should_pass_through(
                    self._graph,
                    self._next_hop,
                    nxt,
                    goal_name,
                    inbound_body_heading=body_heading,
                    pipeline_default=_pipeline_config.PIPELINE_CONFIG.direction,
                    builder_route=route_direction,
                    builder_arrival=arrival_direction,
                    bidirectional_enabled=BIDIRECTIONAL_ENABLED,
                    line_board_count=LINE_BOARD_COUNT,
                ):
                    out.append((ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM))
                else:
                    out.append((ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM))
                # REAR overshoot: rear board (at -AXLE_OFFSET in body-forward) leads
                # in the travel direction. When it detects the intersection at v,
                # the chassis center is AXLE_OFFSET cm behind v in the travel
                # direction. Advancing AXLE_OFFSET in the travel direction = reverse
                # in body coordinates (body heading points opposite to travel).
                if direction == Direction.REAR:
                    self._chassis.reverse(LINE_SENSOR_AXLE_OFFSET_CM)
                else:
                    self._chassis.forward(LINE_SENSOR_AXLE_OFFSET_CM)
                self._localization.on_intersection(nxt)
            elif edge_type == EDGE_PROJECTION:
                if direction == Direction.REAR:
                    out.append(
                        (ACT_PROJECTION_REVERSE, nxt, distance_cm, resolve_projection_mode(edge))
                    )
                    self._chassis.reverse(distance_cm)
                else:
                    out.append((ACT_FOLLOW_DIST, distance_cm))
                    self._chassis.follow_line_to_distance(distance_cm)
                self._localization.on_dead_reckoned_advance(nxt)
            elif edge_type == EDGE_BLIND:
                if direction == Direction.REAR:
                    out.append(
                        (ACT_BLIND_REVERSE, nxt, distance_cm, resolve_blind_mode(edge))
                    )
                    self._chassis.reverse(distance_cm)
                else:
                    out.append((ACT_FORWARD, distance_cm))
                    self._chassis.forward(distance_cm)
                self._localization.on_dead_reckoned_advance(nxt)
            else:    # pragma: no cover -- defensive only
                raise ValueError(
                    f"unknown edge type {edge_type!r} on edge ({current!r}, {nxt!r})"
                )

            current = nxt

        return out

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
        resolution and ACT_*_REVERSE primitives for REAR hops.

        :param goal_name: destination node name.
        :param route_direction: builder override applied to every hop.
        :param arrival_direction: builder override applied only to the
            final hop.
        """
        if goal_name not in self._field_data.NODES:
            raise KeyError(f"goal {goal_name!r} not in field_data.NODES")
        if not self._localization.is_zeroed():
            raise RuntimeError(
                "PathTrackerSim.step_through() called before "
                "LocalizationSim.zero_heading_from_current()"
            )

        current = self._localization.current_node()
        while current != goal_name:
            nxt = self._next_hop[current][goal_name]
            if nxt is None:
                raise UnreachableGoalError(current, goal_name, current=current)
            edge = self._graph.edges[current, nxt]
            edge_heading = float(edge["heading_deg"])
            is_last_hop = (nxt == goal_name)

            direction = resolve_direction(
                edge_dir_override=_read_edge_dir_override(edge),
                pipeline_default=_pipeline_config.PIPELINE_CONFIG.direction,
                builder_arrival=arrival_direction,
                builder_route=route_direction,
                current_heading=self._chassis.heading_deg,
                edge_heading=edge_heading,
                is_last_hop=is_last_hop,
                bidirectional_enabled=BIDIRECTIONAL_ENABLED,
                line_board_count=LINE_BOARD_COUNT,
            )
            body_heading = _body_heading_for(direction, edge_heading)

            delta = wrap_pm180(body_heading - self._chassis.heading_deg)
            if abs(delta) > TURN_EPSILON_DEG:
                self._chassis.turn_relative(delta)
                self._chassis.heading_deg = _normalize(body_heading)
                yield (ACT_TURN_REL, delta), self._snapshot_chassis()

            edge_type = edge["type"]
            distance_cm = float(edge["distance_cm"])
            if edge_type == EDGE_LINE:
                if direction == Direction.REAR:
                    self._chassis.reverse(distance_cm)
                    yield (
                        (ACT_LINE_REVERSE, nxt, resolve_line_mode(edge)),
                        self._snapshot_chassis(),
                    )
                else:
                    self._chassis.follow_line_to_intersection(distance_cm)
                    yield (ACT_FOLLOW_INT, None), self._snapshot_chassis()
                # LINE_OVERSHOOT FSM transition -- emit ACT_LINE_PASS on
                # collinear next-hop body heading, else ACT_LINE_OVERSHOOT.
                # See go_to for rationale and ADR-023.
                pass_through = _should_pass_through(
                    self._graph,
                    self._next_hop,
                    nxt,
                    goal_name,
                    inbound_body_heading=body_heading,
                    pipeline_default=_pipeline_config.PIPELINE_CONFIG.direction,
                    builder_route=route_direction,
                    builder_arrival=arrival_direction,
                    bidirectional_enabled=BIDIRECTIONAL_ENABLED,
                    line_board_count=LINE_BOARD_COUNT,
                )
                # REAR overshoot: rear board (at -AXLE_OFFSET in body-forward) leads
                # in the travel direction. When it detects the intersection at v,
                # the chassis center is AXLE_OFFSET cm behind v in the travel
                # direction. Advancing AXLE_OFFSET in the travel direction = reverse
                # in body coordinates (body heading points opposite to travel).
                if direction == Direction.REAR:
                    self._chassis.reverse(LINE_SENSOR_AXLE_OFFSET_CM)
                else:
                    self._chassis.forward(LINE_SENSOR_AXLE_OFFSET_CM)
                self._localization.on_intersection(nxt)
                yield (
                    (
                        ACT_LINE_PASS if pass_through else ACT_LINE_OVERSHOOT,
                        LINE_SENSOR_AXLE_OFFSET_CM,
                    ),
                    self._snapshot_chassis(),
                )
            elif edge_type == EDGE_PROJECTION:
                if direction == Direction.REAR:
                    self._chassis.reverse(distance_cm)
                    self._localization.on_dead_reckoned_advance(nxt)
                    yield (
                        (ACT_PROJECTION_REVERSE, nxt, distance_cm, resolve_projection_mode(edge)),
                        self._snapshot_chassis(),
                    )
                else:
                    self._chassis.follow_line_to_distance(distance_cm)
                    self._localization.on_dead_reckoned_advance(nxt)
                    yield (ACT_FOLLOW_DIST, distance_cm), self._snapshot_chassis()
            elif edge_type == EDGE_BLIND:
                if direction == Direction.REAR:
                    self._chassis.reverse(distance_cm)
                    self._localization.on_dead_reckoned_advance(nxt)
                    yield (
                        (ACT_BLIND_REVERSE, nxt, distance_cm, resolve_blind_mode(edge)),
                        self._snapshot_chassis(),
                    )
                else:
                    self._chassis.forward(distance_cm)
                    self._localization.on_dead_reckoned_advance(nxt)
                    yield (ACT_FORWARD, distance_cm), self._snapshot_chassis()
            else:    # pragma: no cover
                raise ValueError(
                    f"unknown edge type {edge_type!r} on edge ({current!r}, {nxt!r})"
                )
            current = nxt

    def _snapshot_chassis(self) -> VirtualChassis:
        return VirtualChassis(
            x_cm=self._chassis.x_cm,
            y_cm=self._chassis.y_cm,
            heading_deg=self._chassis.heading_deg,
        )
