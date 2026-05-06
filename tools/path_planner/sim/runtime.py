"""PathTracker FSM mirror in pure Python (verification harness for PR 2).

The runtime simulator mirrors the C++ PathTracker FSM closely enough that
its dispatched-primitive sequence matches what ``compile_path`` produces
for the same NEXT_HOP-walked path on a graph + field_data pair. It is
NOT a faithful tick-by-tick replica — it collapses each Drive/LineFollower
dispatch + completion into a single atomic transition. That is fine for
the verification we care about: end-to-end correctness of the planning +
heading propagation + goal-blind sequence on the toy graph (and any
future graph), without flashing hardware.

Public surface:

* :class:`LocalizationSim` — mirror of ``LocalizationClass``: holds the
  current node and the world-frame yaw offset captured at simulated
  start-button press.
* :class:`PathTrackerSim` — the FSM. ``go_to(goal_name)`` walks
  ``NEXT_HOP`` and returns the dispatched primitive list while mutating
  the chassis.
* :func:`walk_next_hop` — given a NextHopTable, returns the explicit
  node-name list from a source to a target. Useful for tests that want
  to compare the simulator's primitive sequence to ``compile_path``.
* :class:`UnreachableGoalError` — raised by ``go_to`` when
  ``next_hop[from][goal] is None`` (mirrors PathTracker's
  ``Fault::UNREACHABLE``).
"""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Tuple

import networkx as nx

from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_TURN_REL,
    ACT_TURN_TO,
    Primitive,
    normalize_turn,
)
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION
from tools.path_planner.planner import NextHopTable
from tools.path_planner.sim.chassis import VirtualChassis, _normalize

# Same threshold compile_path uses to decide whether a sub-degree turn is
# worth emitting. Re-exported here so test assertions can import a single
# constant rather than picking a magic number.
TURN_EPSILON_DEG: float = 1e-6


class UnreachableGoalError(RuntimeError):
    """Raised when ``next_hop[from][goal] is None`` mid-traverse.

    Carries the source and goal node names plus the running ``current``
    node at the moment of failure so test assertions can pinpoint where
    the planner table's hole was hit.
    """

    def __init__(self, from_node: str, goal_node: str, *, current: Optional[str] = None) -> None:
        ctx = f" (current={current!r})" if current is not None else ""
        super().__init__(
            f"unreachable: from={from_node!r} to={goal_node!r}{ctx}"
        )
        self.from_node = from_node
        self.goal_node = goal_node
        self.current   = current


def walk_next_hop(
    next_hop: NextHopTable,
    source: str,
    target: str,
) -> List[str]:
    """Return the node-name list traversed by repeatedly indexing ``next_hop``.

    Mirrors what PathTracker on the R4 will physically walk. The first
    element is ``source``; the last is ``target``. A self-target walk
    (``source == target``) returns ``[source]``.

    :raises UnreachableGoalError: if ``next_hop[s][t] is None`` for some
        intermediate ``s`` on the path. The exception carries the source
        and target plus the running ``current`` for diagnostic context.
    :raises ValueError: if a runaway-loop guard trips (should not happen
        on a valid NEXT_HOP table — kept as a defensive backstop in
        case a future cost model produces a cycle).
    """
    if source == target:
        return [source]
    path: List[str] = [source]
    current = source
    # Bound to len(next_hop) + 1 to catch any pathological cycle on a
    # malformed NEXT_HOP table (which the tests build deliberately for
    # the unreachable case). networkx's Dijkstra cannot produce cycles
    # on positive-weight graphs, so this is a defensive guard only.
    max_steps = len(next_hop) + 1
    for _ in range(max_steps):
        nxt = next_hop[current][target]
        if nxt is None:
            raise UnreachableGoalError(source, target, current=current)
        # NEXT_HOP self-loop sentinel: if we are already AT the target,
        # the table reports `next_hop[target][target] == target`. We
        # break BEFORE consuming that step so the walk ends cleanly.
        if current == target:
            break
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
    """Mirror of ``LocalizationClass`` (single source of truth for node + yaw offset).

    The simulator's "IMU" is just the chassis's world-frame heading. The
    yaw offset model is unnecessary at this level (no separate IMU
    frame) — but the class still exposes ``zero_heading_from_current()``
    so tests can exercise the "goTo before zero faults" path the C++
    runtime takes.
    """

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
    """Mirror of ``PathTrackerClass`` for the toy graph.

    Each ``go_to(goal_name)`` call walks ``NEXT_HOP`` from the current
    node to ``goal_name`` exactly the way the runtime FSM will, emits
    the dispatched primitive sequence, and mutates the attached chassis.
    On any unreachable hop, ``UnreachableGoalError`` is raised — the
    Python equivalent of ``Fault::UNREACHABLE``.

    The class deliberately mirrors :func:`compile_path`'s primitive
    emission rather than re-implementing it independently — the runtime
    consumes ``Routes::EDGES`` headings (which compile_path produced) and
    the runtime's per-edge logic mirrors compile_path's loop. The
    end-to-end test then only has to assert that the primitive list
    matches what compile_path returns for the walked path.
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

    # -- public API -----------------------------------------------------------

    def go_to(self, goal_name: str) -> List[Primitive]:
        """Walk from the current node to ``goal_name``.

        Mirrors PathTrackerClass's behaviour:

        * ``BAD_NODE``-style guard: if ``goal_name`` is not in the graph,
          raise ``KeyError`` (caller's input is the bug, not the planner).
        * ``NOT_ZEROED`` guard: if ``LocalizationSim.is_zeroed()`` is
          False, raise ``RuntimeError`` (matches the C++ Fault path).
        * ``UNREACHABLE`` guard: if the planner reports no path from any
          intermediate node, raise :class:`UnreachableGoalError`.
        * Otherwise: emit the primitive sequence, advance the chassis,
          and update the localization on each edge.

        :param goal_name: target node name.
        :return: list of ``(action, value)`` tuples.
        :raises KeyError: if ``goal_name`` is not a known node.
        :raises RuntimeError: if ``zero_heading_from_current()`` was not
            called first.
        :raises UnreachableGoalError: if any intermediate ``next_hop``
            returns ``None``.
        """
        if goal_name not in self._field_data.NODES:
            raise KeyError(
                f"goal {goal_name!r} not in field_data.NODES"
            )
        if not self._localization.is_zeroed():
            raise RuntimeError(
                "PathTrackerSim.go_to() called before "
                "LocalizationSim.zero_heading_from_current()"
            )

        out: List[Primitive] = []
        current = self._localization.current_node()

        # Walk one edge at a time. Mirrors compile_path's main loop:
        # turn (relative delta in (-180, +180]) -> motion primitive ->
        # advance current_heading.
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
                # Snap to target_heading exactly to match compile_path's
                # behaviour (which sets current_heading = edge_heading
                # rather than current + delta to avoid float drift).
                self._chassis.heading_deg = _normalize(target_heading)

            edge_type = edge["type"]
            distance_cm = float(edge["distance_cm"])
            if edge_type == EDGE_LINE:
                out.append((ACT_FOLLOW_INT, None))
                self._chassis.follow_line_to_intersection(distance_cm)
                self._localization.on_intersection(nxt)
            elif edge_type == EDGE_PROJECTION:
                out.append((ACT_FOLLOW_DIST, distance_cm))
                self._chassis.follow_line_distance(distance_cm)
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

        # Goal arrival blind sequence (only for goal-typed nodes that have
        # a GOAL_APPROACH entry). Mirrors compile_path's third stage.
        if (
            self._field_data.NODE_TYPES.get(goal_name) == "goal"
            and goal_name in self._field_data.GOAL_APPROACH
        ):
            spec = self._field_data.GOAL_APPROACH[goal_name]
            approach = float(spec["approach_heading_deg"])
            if abs(normalize_turn(approach - self._chassis.heading_deg)) > TURN_EPSILON_DEG:
                out.append((ACT_TURN_TO, approach))
                self._chassis.turn_to_heading(approach)
            for action, value in spec["blind_moves"]:
                if action == ACT_TURN_REL:
                    deg = float(value)
                    if abs(deg) > TURN_EPSILON_DEG:
                        out.append((ACT_TURN_REL, deg))
                        self._chassis.turn_relative(deg)
                elif action == ACT_FORWARD:
                    cm = float(value)
                    if cm > 0.0:
                        out.append((ACT_FORWARD, cm))
                        self._chassis.forward(cm)
                else:    # pragma: no cover -- defensive only
                    raise ValueError(
                        f"unsupported blind-move action {action!r} for goal {goal_name!r}"
                    )

        return out

    # -- introspection helpers used by tests ---------------------------------

    def walked_path(self, goal_name: str) -> List[str]:
        """Return the node-name list ``go_to(goal_name)`` would walk.

        Convenience wrapper over :func:`walk_next_hop` that uses the
        simulator's currently-held ``current_node`` as the source.
        """
        return walk_next_hop(self._next_hop, self._localization.current_node(), goal_name)

    def step_through(self, goal_name: str):
        """Generator variant of :meth:`go_to` that yields per-primitive.

        Yields a stream of ``(primitive, chassis_snapshot)`` tuples, one
        per dispatched primitive. The ``chassis_snapshot`` is a copy of
        the chassis taken AFTER the primitive has been applied — useful
        for the CLI demo's per-step pose printout and for any future
        visual replay tool.

        Mirrors :meth:`go_to` exactly in terms of which primitives are
        emitted, the order, and the chassis end state. The same fault
        and validation rules apply.

        :param goal_name: target node name.
        :yields: ``(primitive, VirtualChassis)`` per primitive.
        :raises: same exceptions as :meth:`go_to`.
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
                self._localization.on_intersection(nxt)
                yield (ACT_FOLLOW_INT, None), self._snapshot_chassis()
            elif edge_type == EDGE_PROJECTION:
                self._chassis.follow_line_distance(distance_cm)
                self._localization.on_dead_reckoned_advance(nxt)
                yield (ACT_FOLLOW_DIST, distance_cm), self._snapshot_chassis()
            elif edge_type == EDGE_BLIND:
                self._chassis.forward(distance_cm)
                self._localization.on_dead_reckoned_advance(nxt)
                yield (ACT_FORWARD, distance_cm), self._snapshot_chassis()
            else:    # pragma: no cover -- defensive only
                raise ValueError(
                    f"unknown edge type {edge_type!r} on edge ({current!r}, {nxt!r})"
                )
            current = nxt

        if (
            self._field_data.NODE_TYPES.get(goal_name) == "goal"
            and goal_name in self._field_data.GOAL_APPROACH
        ):
            spec = self._field_data.GOAL_APPROACH[goal_name]
            approach = float(spec["approach_heading_deg"])
            if abs(normalize_turn(approach - self._chassis.heading_deg)) > TURN_EPSILON_DEG:
                self._chassis.turn_to_heading(approach)
                yield (ACT_TURN_TO, approach), self._snapshot_chassis()
            for action, value in spec["blind_moves"]:
                if action == ACT_TURN_REL:
                    deg = float(value)
                    if abs(deg) > TURN_EPSILON_DEG:
                        self._chassis.turn_relative(deg)
                        yield (ACT_TURN_REL, deg), self._snapshot_chassis()
                elif action == ACT_FORWARD:
                    cm = float(value)
                    if cm > 0.0:
                        self._chassis.forward(cm)
                        yield (ACT_FORWARD, cm), self._snapshot_chassis()
                else:    # pragma: no cover -- defensive only
                    raise ValueError(
                        f"unsupported blind-move action {action!r} for goal {goal_name!r}"
                    )

    def _snapshot_chassis(self) -> VirtualChassis:
        """Return a copy of the current chassis pose (for step_through yields)."""
        return VirtualChassis(
            x_cm=self._chassis.x_cm,
            y_cm=self._chassis.y_cm,
            heading_deg=self._chassis.heading_deg,
        )
