# ADR-001: Lines and intersections are the primary sensing substrate — no wall-following, no dead-reckoning

**Status:** Implemented
**Date decided:** 2026-05 (Phase 0)

## Rule

The robot does not use time-of-flight wall sensors and does not pose-track via odometry alone. Navigation is built on the field's existing black lines and intersections. Every intersection crossing refreshes ground truth.

## Why

The field has too many obstacles and zone boundaries that confuse distance sensors. Pure dead-reckoning accumulates error too quickly across a 2-minute run — encoder slip, wheel drift, and a single missed turn compound silently until the robot is somewhere it doesn't know about. Lines are reliable, fixed, and unambiguous; intersections are discrete events the robot can use as ground-truth checkpoints.

## How to apply

- All localization re-anchors on intersection events (see [ADR-005](./005-localization-single-source-of-truth.md)).
- Edge primitives in the path planner are line-follow segments, never odometry "drive 300 cm forward".
- Blind edges (no line — short connecting segments) are permitted only between calibrated nodes and are tracked with a blind-distance budget (see deferred work in [PHASE2_DEFERRED.md](../PHASE2_DEFERRED.md)).

## Trigger to revisit

A future field that does not have continuous lines between every pair of waypoints, or a rules change that prohibits line-following.

## See also

- [ADR-002](./002-three-layer-mental-model.md) — perception → map → planning layering
- [ADR-005](./005-localization-single-source-of-truth.md)
