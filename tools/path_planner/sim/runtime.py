"""PathTracker FSM mirror in pure Python (verification harness, PR3).

Mirrors the C++ PathTracker FSM closely enough that its dispatched-primitive
sequence matches :func:`tools.path_planner.compiler.compile_path` for the
same NEXT_HOP-walked path. Not tick-by-tick faithful -- each Drive/LineFollower
dispatch + completion is collapsed into a single atomic transition. That is
enough for end-to-end planning + heading-propagation verification on any
graph without hardware.

PR3 simplification: drops the PROJECTION / GOAL_APPROACH branches. Every
node is reached via a LINE or BLIND edge.

Public surface:

* :class:`LocalizationSim` -- mirror of ``LocalizationClass``.
* :class:`PathTrackerSim` -- the FSM. ``go_to(goal_name)`` walks NEXT_HOP.
* :func:`walk_next_hop` -- given a NextHopTable, returns the explicit
  node-name list from source to target.
* :class:`UnreachableGoalError` -- raised on ``next_hop[from][goal] is None``.
"""

from __future__ import annotations

from typing import List, Optional

import networkx as nx

from tools.path_planner.compiler import (
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_TURN_REL,
    Primitive,
    normalize_turn,
)
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE
from tools.path_planner.planner import NextHopTable
from tools.path_planner.sim.chassis import VirtualChassis, _normalize

TURN_EPSILON_DEG: float = 1e-6


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
    """Mirror of ``PathTrackerClass`` for PR3 (LINE + BLIND only)."""

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

    def go_to(self, goal_name: str) -> List[Primitive]:
        """Walk from the current node to ``goal_name``.

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
                self._localization.on_intersection(nxt)
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
        """Generator variant of :meth:`go_to` yielding ``(primitive, snapshot)``."""
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
                self._localization.on_intersection(nxt)
                yield (ACT_FOLLOW_INT, None), self._snapshot_chassis()
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
