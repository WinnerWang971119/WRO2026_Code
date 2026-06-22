# ADR-023: Pass-through collinear intersections + runtime overshoot-mode probe

**Status:** Implemented
**Date decided:** 2026-05-23 (issue #19)

> **Amended by [ADR-046](./046-projection-as-node-property.md) (issue #64).** Projection is now a
> node type, not an edge type, so this ADR's collinear pass-through is narrowed to the
> `line–proj–line` case. A projection node has **no junction marker**, so its pass-through fires off
> the **distance gate** at the node, not the `Intersection.isAtIntersection()` branch this ADR
> describes; the mid-traverse `onDeadReckonedAdvance` and the re-anchor at the next real junction are
> unchanged. The "PROJECTION next" / "next edge is BLIND or PROJECTION" wording below refers to the
> retired edge type — read "destination is a projection node" / "BLIND edge."

## Rule

PathTracker peeks the next-hop edge at intersection-detect (`LINE_FOLLOW` state in
`update()`). When the next-hop edge's heading (wrap-normalized) differs from the inbound
edge's heading by no more than `Config::NAV_TURN_TOLERANCE_DEG`, AND the intersected
node is not the final destination, AND `Config::PATH_PASS_THROUGH_ENABLED` is true, the
chassis traverses the node at cruise PWM with no ramp-down → brake → ramp-up cycle.
Localization still advances exactly once at the axle-on-node moment.

LineFollower's distance-gated stop (defined in ADR-018) gains two builder knobs that
implement the pass-through and the underlying overshoot semantics:
- `setOvershootMode(true)` replaces the ADR-018 lost-line dispatch with a runtime
  line-presence probe: when the line goes offline during the gated overshoot window, the
  controller dead-reckons symmetric cruise PWM instead of dispatching
  `Config::LOST_LINE_ACTION`. **This applies project-wide to every overshoot, not just
  pass-through hops** — it eliminates the latent "chassis stops mid-overshoot because
  the line ends at the node" failure for stop-and-go traversals too.
- `setBrakeOnGate(false)` makes the distance-gate trip latch `_distanceReached` without
  self-disabling the controller, so PathTracker can keep cruise live through the 5 cm
  sensor-to-axle window for pass-through hops.

## Why

The original LINE_OVERSHOOT FSM step (ADR-018) ramps down to floor PWM, active-brakes,
then ramps back up from 0 at every intersection — adding stop-and-go behavior even when
the geometry doesn't require it. For paths that cross multiple collinear intersections,
this is wasteful: the chassis comes to a full stop at every node when it could be
traversing each one at cruise.

The pass-through rule eliminates the stop-and-go when the next-hop edge is collinear
(heading delta within `NAV_TURN_TOLERANCE_DEG`). The runtime overshoot-mode probe
additionally fixes a pre-existing edge case: when the inbound LINE edge ends at a node
where the line physically terminates (next edge is BLIND or PROJECTION with no painted
line continuation), today's ADR-018 dispatch would fire `LOST_LINE_ACTION` mid-overshoot
and brake the chassis short of the axle-on-node mark. The runtime probe lets the
chassis dead-reckon the remaining 5 cm using only encoder distance, regardless of
whether the line still exists.

Heading tolerance reuses `NAV_TURN_TOLERANCE_DEG` (0.5°) — the same threshold that
already gates the pre-edge yaw turn dispatch in `_onPlanNextHop`. Reusing one constant
keeps the two "is this collinear enough?" decisions in lock-step.

## How to apply

- **PathTracker** does the peek in `LINE_FOLLOW`'s `Intersection.isAtIntersection()`
  branch. The peek uses `FieldMap::getNextHop(_nextHop, _destinationNode)` plus
  `FieldMap::getEdge(...)` to read the inbound and outbound edge headings; the delta is
  wrap-normalized exactly like the sim's `normalize_turn`.
- **PathTracker** always arms `LineFollower.setOvershootMode(true)` for the 5 cm
  overshoot — pass-through OR stop-and-go. The runtime probe is project-wide.
- **PathTracker** arms `LineFollower.setBrakeOnGate(!_passThrough)` so pass-through
  keeps the controller live, while stop-and-go preserves today's self-disable behavior.
- **PathTracker** dispatches the next edge in `_dispatchEdgeAfterTurn`:
  - LINE next: chain `.setOvershootMode(false)` to flip back to P+D; already-enabled
    controller hits the apply-builder fast-path so no ramp-up reseed fires.
  - PROJECTION next: same, plus `.setDistanceTarget(edge.distance_cm)` re-arming.
  - BLIND next: when `_passThrough`, capture cruise PWM BEFORE disabling LineFollower
    (with `.setBrake(false)`, no stop), then chain `.setStartSpeed(cruise)` onto
    `Drive.driveDist(...)` so the chassis hands cruise off into the dead-reckoned move.
    RAW_FLAT specifically: non-pass-through still pins start=cruisePwm (preserves the
    FLAT contract); pass-through uses captured cruise (may introduce a small ramp by
    design when presets differ).
- **Simulator** mirrors the rule: `tools/path_planner/sim/runtime.py` emits a new
  `ACT_LINE_PASS` primitive (instead of `ACT_LINE_OVERSHOOT`) when the peek decides
  pass-through. Chassis position update is identical. The sim hand-mirrors
  `Config::NAV_TURN_TOLERANCE_DEG` and `Config::PATH_PASS_THROUGH_ENABLED`; a pytest
  verifies parity via regex grep on the header.

## Trigger to revisit

- The hardware verification (HW1 / HW2 / HW3 — Phase 3) reveals that 0.5° is too tight
  or too loose for the real field geometry. Either widen via a new constant
  `PASS_THROUGH_TURN_TOLERANCE_DEG`, OR investigate whether routes / planner should
  declare collinear-passthrough explicitly on edges.
- The runtime overshoot-mode probe causes a regression in the no-line zone (e.g. the
  chassis fails to dead-reckon properly because of a sensor miscalibration). Rollback
  is the `.setOvershootMode(false)` chain in PathTracker's LINE_FOLLOW arming.
- A future ADR introduces an explicit "stop here" annotation on LINE edges (e.g. for
  manipulator pickups), at which point the destination-only guard
  (`_nextHop != _destinationNode`) needs to extend to those annotated edges.

## See also

- [ADR-018: LineFollower distance-gated stops use a symmetric trapezoidal profile](./018-linefollower-distance-gated-stops.md) — refined by this ADR. ADR-018 remains
  the default contract for distance-gated stops; ADR-023's overshoot-mode is the
  runtime-probe override.
- [ADR-020: LineFollower PID freeze during junction dwell](./020-junction-pid-freeze.md) — junction-freeze + overshoot-mode coexist: when line IS online during overshoot,
  both branches can fire; both produce symmetric cruise.
- [ADR-046: Projection is a node property, not an edge type](./046-projection-as-node-property.md) — amends this ADR; the `line–proj–line` pass-through fires off the distance gate (a projection node has no junction marker).
- Issue #19 (the originating bug report).
- `docs/plans/2026-05-23-pass-through-collinear-intersections.md` (this implementation
  plan).
