# ADR-038: `Localization` tracks a 2D operator-offset delta from the last anchored node

**Status:** Implemented
**Date decided:** 2026-06 (Phase 3)

## Rule

`Localization` now maintains a second piece of stored state alongside the current node: a
2D pose offset `Offset2D {x, y, thetaDeg}` that accumulates how far the chassis has been
hand-driven away from its last anchored node. The offset is exposed as **read-only telemetry**
(`getOffsetFromNode()`) — nothing in PR1a acts on it. It is written exclusively through
`Localization`'s own methods (`clearOffset()`, called from the arrival hooks and future
recovery verbs), so ADR-005's single-writer rule remains in force; only the write *set*
grows, not the number of modules that mutate position state.

This amends ADR-005's "heading is computed live from IMU yaw + offset, never stored" clause.
`thetaDeg` is a **bounded delta-from-anchor**, not the authoritative heading. The authoritative
heading is still computed live from the IMU via `getHeadingDeg()`; `thetaDeg` is a recovery
hint that is zeroed on every node arrival. (The pre-existing `_lastGoodHeadingDeg` cache
already partially relaxed "never stored" before this PR — see ADR-005.)

## Why

The competition can end with the robot parked off its final node after an operator hand-drive
correction. Without a stored displacement the return-to-node recovery verb (a later PR) has
no information — it can only see the current node id (which is stale if the robot was moved)
and the live heading. A bounded dead-reckoning offset closes this information gap at low cost:
it integrates only during the gap between PathTracker hops, is re-zeroed on every arrival, and
therefore never accumulates error across a complete navigation sequence.

Per ADR-001's honesty rule: dead-reckoning error grows with distance and time. The offset is
only ever consumed immediately after a correction move ends — it is never held across a full
mission — so the accumulated error stays within the bounds of a single inter-node gap.

## How to apply

- `Localization.update()` runs once per cooperative tick AFTER `PathTracker.update()` so the
  `_navActive` flag reflects PathTracker's final state for that tick. The integration gate is
  open only when `!_navActive && isZeroed()`.
- PathTracker calls `Localization.setNavActive(bool)` (true while driving a hop; false on
  idle / done / FAULT). A faulted robot counts as not-driving, so hand-correction after a
  fault is tracked.
- `clearOffset()` is called by the node-arrival hooks (`onIntersection`,
  `onDeadReckonedAdvance`) and may be called by future recovery verbs to re-anchor explicitly.
- `getOffsetFromNode()` is a pure, no-I/O accessor. Telemetry and recovery callers may call
  it freely without worrying about bus latency.
- Independent deadbands (`Config::OFFSET_TRANSLATION_DEADBAND_CM` /
  `Config::OFFSET_ROTATION_DEADBAND_DEG` in `ConfigLocalization.h`) prevent vibration jitter
  from growing the offset while the robot is stationary.

## Trigger to revisit

- The return-to-node consumer PR lands and reveals that the bounded dead-reckoning accuracy
  is insufficient for a reliable return move — at that point consider adding a visual anchor
  (camera ground-truth fix on a known node marker) before the return.
- A rules change allows time-of-flight wall anchoring; the integrator could be dropped in
  favour of a sensor-fusion corrector (still routed through Localization — ADR-005 applies).

## See also

- [ADR-001](./001-no-wall-follow-no-dead-reckon.md) — dead-reckoning honesty; the operator
  offset is bounded dead-reckoning, not continuous odometry
- [ADR-005](./005-localization-single-source-of-truth.md) — single-writer rule; this ADR
  amends its "never stored" heading clause
- [ADR-031](./031-config-split-per-domain.md) — `ConfigLocalization.h` follows the
  per-domain config split
