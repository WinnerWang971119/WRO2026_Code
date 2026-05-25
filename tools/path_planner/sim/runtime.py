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
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION
from tools.path_planner.pipeline_config import (
    PIPELINE_CONFIG,
    BlindMode,
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


__all__ = [
    "ACT_LINE_OVERSHOOT",
    "ACT_LINE_PASS",
    "LINE_SENSOR_AXLE_OFFSET_CM",
    "NAV_TURN_TOLERANCE_DEG",
    "PATH_PASS_THROUGH_ENABLED",
    "TURN_EPSILON_DEG",
    "LocalizationSim",
    "PathTrackerSim",
    "UnreachableGoalError",
    "resolve_blind_mode",
    "resolve_line_mode",
    "resolve_pre_turn_mode",
    "resolve_projection_mode",
    "walk_next_hop",
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


def _should_pass_through(graph, next_hop, current_edge, intersected_node, goal_name) -> bool:
    """Return True iff a LINE-edge intersection at ``intersected_node`` should
    emit ``ACT_LINE_PASS`` instead of ``ACT_LINE_OVERSHOOT``.

    Conditions (all must hold):

    * :data:`PATH_PASS_THROUGH_ENABLED` is True.
    * ``intersected_node`` is not the goal (final destination never passes through).
    * A next-next hop exists from ``intersected_node`` toward ``goal_name``
      (non-``None``, non-self).
    * The next-next edge's ``heading_deg``, wrap-normalized via
      :func:`tools.path_planner.compiler.normalize_turn` against
      ``current_edge``'s ``heading_deg``, has absolute value
      <= :data:`NAV_TURN_TOLERANCE_DEG`.

    The wrap normalization is REQUIRED -- without it the 359.5deg / 0.0deg
    boundary diverges from the C++ side (which also wrap-normalizes per
    ADR-023).
    """
    if not PATH_PASS_THROUGH_ENABLED:
        return False
    if intersected_node == goal_name:
        return False
    next_next = next_hop[intersected_node][goal_name]
    if next_next is None or next_next == intersected_node:
        return False
    next_edge = graph.edges[intersected_node, next_next]
    heading_delta = normalize_turn(
        float(next_edge["heading_deg"]) - float(current_edge["heading_deg"])
    )
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

    def go_to(self, goal_name: str) -> List[Primitive]:
        """Walk from the current node to ``goal_name``.

        Emits per edge: an optional ``ACT_TURN_REL`` to align heading,
        then the per-type motion primitive. For EDGE_LINE traversals, an
        ``ACT_LINE_OVERSHOOT`` step is appended after ``ACT_FOLLOW_INT``,
        mirroring the runtime LINE_OVERSHOOT FSM transition that
        advances the chassis by :data:`LINE_SENSOR_AXLE_OFFSET_CM` so
        the wheel axle (not the line sensor) lands on the node. EDGE_PROJECTION
        emits ``ACT_FOLLOW_DIST`` and advances Localization via
        :meth:`LocalizationSim.on_dead_reckoned_advance` (no overshoot
        step -- the encoder gate already lands the chassis on the node).

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
            target_heading = float(edge["heading_deg"])

            delta = normalize_turn(target_heading - self._chassis.heading_deg)
            if abs(delta) > TURN_EPSILON_DEG:
                out.append((ACT_TURN_REL, delta))
                self._chassis.turn_relative(delta)
                self._chassis.heading_deg = _normalize(target_heading)

            edge_type = edge["type"]
            distance_cm = float(edge["distance_cm"])
            if edge_type == EDGE_LINE:
                out.append((ACT_FOLLOW_INT, None))
                self._chassis.follow_line_to_intersection(distance_cm)
                # LINE_OVERSHOOT FSM transition -- emit ACT_LINE_PASS when the
                # next-hop edge after ``nxt`` is collinear with the inbound
                # edge, else ACT_LINE_OVERSHOOT. Chassis advance is identical
                # (5cm); the primitive name differs only for downstream
                # verification + telemetry. See ADR-023.
                if _should_pass_through(
                    self._graph, self._next_hop, edge, nxt, goal_name
                ):
                    out.append((ACT_LINE_PASS, LINE_SENSOR_AXLE_OFFSET_CM))
                else:
                    out.append((ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM))
                self._chassis.forward(LINE_SENSOR_AXLE_OFFSET_CM)
                self._localization.on_intersection(nxt)
            elif edge_type == EDGE_PROJECTION:
                out.append((ACT_FOLLOW_DIST, distance_cm))
                self._chassis.follow_line_to_distance(distance_cm)
                self._localization.on_dead_reckoned_advance(nxt)
            elif edge_type == EDGE_BLIND:
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

    def step_through(self, goal_name: str):
        """Generator variant of :meth:`go_to` yielding ``(primitive, snapshot)``.

        Mirrors :meth:`go_to`'s emission shape exactly: an ``ACT_LINE_OVERSHOOT``
        step is yielded after every ``ACT_FOLLOW_INT``, and EDGE_PROJECTION
        yields ``ACT_FOLLOW_DIST`` then advances Localization via
        :meth:`LocalizationSim.on_dead_reckoned_advance`.
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
            target_heading = float(edge["heading_deg"])

            delta = normalize_turn(target_heading - self._chassis.heading_deg)
            if abs(delta) > TURN_EPSILON_DEG:
                self._chassis.turn_relative(delta)
                self._chassis.heading_deg = _normalize(target_heading)
                yield (ACT_TURN_REL, delta), self._snapshot_chassis()

            edge_type = edge["type"]
            distance_cm = float(edge["distance_cm"])
            if edge_type == EDGE_LINE:
                self._chassis.follow_line_to_intersection(distance_cm)
                yield (ACT_FOLLOW_INT, None), self._snapshot_chassis()
                # LINE_OVERSHOOT FSM transition -- emit ACT_LINE_PASS on
                # collinear next-hop, else ACT_LINE_OVERSHOOT. Chassis advance
                # is identical. See go_to for rationale and ADR-023.
                pass_through = _should_pass_through(
                    self._graph, self._next_hop, edge, nxt, goal_name
                )
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
                self._chassis.follow_line_to_distance(distance_cm)
                self._localization.on_dead_reckoned_advance(nxt)
                yield (ACT_FOLLOW_DIST, distance_cm), self._snapshot_chassis()
            elif edge_type == EDGE_BLIND:
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
