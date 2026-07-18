# ADR-071: Anticipatory line-approach ramp — reactive-stop hops decelerate before detection, not after

**Status:** Implemented
**Date decided:** 2026-07 (Phase 3)

> **Number-collision note:** verified against `main` (2026-07): ADR-067/068/069 have since merged
> (fault-clear cascade, in-place turn PD, lateral offset-follow); `feat/163-external-gyro-yaw`
> holds 070 (unmerged, PR #172); 071 is free and unique to this branch, and 072 is the next open
> number — no live collision. Re-verify against `main` at merge and renumber this file (never a
> merged ADR's) only if a new branch has since taken 071 — see the ADR-number-collision gotcha.

## Rule

Both reactive-stop approaches — the LineFollower junction hop (`PathTracker::LINE_FOLLOW_INTERSECTION`)
and the `Drive.stopAtLine` seek (SEEK/ARMED) — ramp their commanded PWM down over the *predicted*
sensor-crossing distance, each minus the direction-aware sensor-to-axle offset. The LineFollower
hop takes the pivot-trimmed edge length (`PathTracker::_pivotTrimDistance(edge)`, **not**
`commandedDistanceCm`, which is `0.0f` for that sensor-terminated primitive); the seek takes its
axle-frame `_stopExpectedCm`. The ramp bottoms out near the kinetic floor `PWM_MOVING_MIN` before
the physical crossing. Physical line detection (`Intersection::isAtIntersection()` /  the seek's
`T_OR_CROSS`) stays the sole authoritative stop and `Localization` re-anchor; encoder distance only
bounds a new overrun fault, never the nominal-arrival trigger. Default-on via
`Config::LINE_APPROACH_RAMP_ENABLED` (byte-identical to today when `false`). Collinear pass-through
nodes are excluded via a dispatch-time latch (ADR-023/059) — a hop that doesn't terminate at the
target node keeps full cruise.

## Why

The old behavior was blind cruise followed by a hard brake on detection — the chassis decelerated
roughly a robot-length too late, producing overshoot or an abrupt stop. The anticipatory ramp
composes the already golden-vector-pinned `resolveTShapeRampCm` kernel (`TShapeProfilePolicy.h`,
ADR-058) inside one new header-only `ApproachRampPolicy.h`, so both consumers and the `profile.py`
CTPS mirror share a single arithmetic source instead of three copies. No change to ramp slope,
bounds, or the arrival floor — it reuses `TSHAPE_ACCEL_PWM_PER_CM`, `TSHAPE_RAMP_MIN/MAX_CM`, and
`PWM_MOVING_MIN` unchanged, and it closes a real gap: the LineFollower hop previously had no
overrun bound at all.

## How to apply

- `ApproachRampPolicy.h` exposes `resolveApproachSchedule` (composes `resolveTShapeRampCm`) +
  `approachSpeedAt`, pre-snap and magnitude-domain; `ApproachSchedule.active == false` means the
  caller degrades to today's flat path (short hop or toggle off) — the policy never fabricates a
  partial ramp.
- LineFollower gains `setApproachTarget(cm)` / `approachPhase()`; Drive's seek feeds
  `approachSpeedAt(...)` into SEEK/ARMED using `_stopExpectedCm − axleOffsetCm` as the sensor-frame
  prediction. The seek's `driveContinuous` snaps to the kinetic floor `PWM_MOVING_MIN` (not the
  static breakaway floor), so the seek's arrival speed matches the LineFollower hop's — one arrival
  speed across both reactive-stop paths. Byte-identical when the ramp is off: `_stopCruise` and the
  OVERSHOOT ramp's own math both stay at or above the static floor, so the lower floor is a no-op.
- **Monotonic detect handoff:** the `LINE_OVERSHOOT` re-arm and the seek's `OVERSHOOT` leg both
  seed from the *last commanded* (floor) magnitude, not full preset / `_stopCruise` — PWM never
  jolts back up on the tick detection fires.
- Two new knobs in `ConfigDrive.h`: `LINE_APPROACH_MARGIN_CM` (bottom out this far before the
  prediction) and `LINE_APPROACH_MAX_MARGIN_CM` (bounded overrun before `LINE_NOT_FOUND` /
  `DRIVE_FAULT` on the LineFollower hop, mirroring the seek's existing max-margin fault).
- No `PathTrackerParityTrace.h` change and no `PathTrackerSim` change — the ramp is a sub-FSM speed
  detail that leaves the FSM dispatch trace and the distance-driven cumulative pose unchanged, so
  ADR-050 parity stays green untouched. CTPS renders the deceleration in `track_builder.py` via
  `profile.py`'s `approach_speed_at` (a display-layer computation, ADR-058), not from a trace record.

## Trigger to revisit

- **Short-hop engage threshold.** With `TSHAPE_ACCEL_PWM_PER_CM=4`, cruise ~80→floor 30 needs a
  ~12.5 cm ramp; adding the margin (4) and the front offset (12.02) puts the forward engage
  threshold at ~28–29 cm axle-frame. Hops shorter than that run flat today (documented known
  limit). If the real mat carries many short junction hops left unimproved, add a reduced-cruise
  or triangle-profile fallback.

## See also

- [ADR-018](./018-linefollower-distance-gated-stops.md) / [ADR-023](./023-pass-through-collinear-intersections.md) — the existing distance-gated ramp-down and pass-through exclusion this feature runs alongside, byte-identical.
- [ADR-027](./027-drive-stop-at-line.md) — `Drive.stopAtLine`'s seek/arm-window/fault machinery this ramp feeds without altering.
- [ADR-046](./046-projection-as-node-property.md) — the pass-through / projection-node model the dispatch-time latch mirrors.
- [ADR-058](./058-ctps-firmware-motion-profile.md) — the `TShapeProfilePolicy.h` kernel and golden-vector convention this policy composes.
- [ADR-059](./059-collinear-run-continuous-motion.md) — collinear-chain pass-through this feature must not slow.
- [ADR-061](./061-hop-settle-wait.md) — the `==0`/`false` byte-identical-toggle precedent this feature's `LINE_APPROACH_RAMP_ENABLED` follows.
