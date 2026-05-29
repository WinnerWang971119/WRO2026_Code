# ADR-024: Bidirectional line following — runtime auto-direction + 3-layer override

**Status:** Implemented
**Date decided:** 2026-05 (Phase 4, issue #21)

## Rule

When `LINE_BOARD_COUNT == 2` and `Config::BIDIRECTIONAL_AUTO_ENABLED` is true,
PathTracker chooses front-board (forward) or rear-board (reverse) traversal per
hop by comparing the absolute heading delta to a strict
`BIDIRECTIONAL_TURN_THRESHOLD_DEG` (default `90.0f`; ties go forward).
Manual overrides resolve in strict precedence: per-directed-edge
`direction_override` in `routes.h` > `setArrivalDirection` (final hop only) >
`setRouteDirection` (all hops) > auto. Pass-through (ADR-023) composes via the
resolved body-heading delta, not the raw forward edge headings.

The kill-switch (`BIDIRECTIONAL_AUTO_ENABLED`, renamed in PR #34 review from
the misleading "ENABLED") gates ONLY the auto branch. Operator overrides
(per-edge, pipeline default, builder route/arrival) bypass it. To fully
disable bidirectional, clear the overrides in `field_data.py` AND set
`BIDIRECTIONAL_AUTO_ENABLED` to false.

When `LINE_BOARD_COUNT == 1` and REAR is forced (via override, builder, or edge
tag), PathTracker raises `Fault::REVERSE_UNAVAILABLE`. Auto never picks REAR on
a 1-board chassis (the resolver short-circuits at Step 5).

## Why

Two-way line following lets the robot avoid 180° in-place turns on collinear
retrace paths, saving roughly turn-time × number-of-retraces per run. Issue #21
requested the feature gated on the existing 2-board hardware (ADR-019), with
explicit operator override at three layers: mission DSL, PathTracker builder, and
offline edge tagging.

The auto-decision lives at runtime in `PathTracker::_onPlanNextHop`. A debate
skill run (verdict 3.0/10 against the planner-side alternative) established that
adding a turn-cost penalty to the offline Dijkstra requires line-graph state
expansion the current `NEXT_HOP[u][v]` schema cannot represent — incoming heading
would have to become part of the table key, breaking field- and host-side
compatibility. The runtime branch uses live IMU heading and is localized to one
function.

The 90° threshold is a geometric constant, not a tuning parameter: it is the
exact break-even angle at which an in-place turn costs the same as the equivalent
reverse traversal. Ties go forward to preserve "REAR is the non-canonical mode"
intuition.

## How to apply

- **C++ API:** `PathTracker.goTo(name).setRouteDirection(Direction::REAR)` and
  `.setArrivalDirection(Direction::REAR)`. Mission DSL mirrors these:
  `Mission.goTo("X").setRouteDirection(...).setArrivalDirection(...)`.
- **Offline override:** add `"direction": "front"` or `"rear"` to an EDGES entry
  in `field_data.py`. Applies only to the forward `(u,v)` directed edge (Codex
  P1 invariant; the reverse `(v,u)` edge retains `DEFAULT`).
- **Resolution algorithm:** `_resolveDirection` in the anonymous namespace of
  `PathTracker.cpp` is the single source of truth; the Python mirror is
  `tools/path_planner/sim/runtime.py::resolve_direction`. TC18 enforces
  cross-language parity via a pytest that greps the generated header for matching
  constant definitions.

## Trigger to revisit

- A third sensor board (e.g. left/right side board) is added — the `Direction`
  enum and `_resolveDirection` would need new enumerators.
- Turn-penalty Dijkstra is implemented with line-graph state expansion — would
  supersede the runtime auto-decision with a globally-optimal planner-side choice.
- The chassis kinematics change enough that 90° is no longer the correct
  break-even angle (e.g. a much wider chassis with a slower in-place turn).

## See also

- [ADR-019 — Custom 8-channel `WROLineArray` (front + optional rear board)](./019-8channel-custom-line-sensor.md)
- [ADR-023 — Pass-through collinear intersections (composes via body-heading delta, not raw forward edge headings)](./023-pass-through-collinear-intersections.md)
- [ADR-012 — Fluent builder for verbs with 3+ knobs](./012-fluent-builder-3plus-knobs.md)
- `docs/plans/2026-05-25-bidirectional-line-following.md` — the originating spec.
