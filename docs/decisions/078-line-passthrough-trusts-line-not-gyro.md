# ADR-078: A LINE pass-through continuation trusts the line, not the drifted gyro — no pre-edge turn

**Status:** Implemented
**Date decided:** 2026-07 (bench, branch `0721BeltTest`)

## Rule

On a collinear pass-through continuation into a **LINE** edge, `PathTracker::_dispatchTurnIfNeeded`
suppresses the pre-edge yaw turn **unconditionally** — regardless of the live IMU-heading delta —
and keeps the LineFollower live so its P+D washes out any residual heading over the hop. Gated by
`Config::PATH_PASS_THROUGH_TRUST_LINE_ENABLED` (default **ON**). The guard fires only when
`_passThrough` is latched **and** the outbound `_activeEdgeType == LINE`; a BLIND continuation and
the return prologue are unaffected. This reverses, for the LINE pass-through case only, the
ADR-023/ADR-052 "yaw-vs-peek divergence" guard that treated the live IMU delta as the authoritative
turn trigger.

## Why

Pass-through is classified at plan time on EDGE-table headings (`_evaluateLineFollowPassThrough`,
pure geometry) — that decision is correct. But the pre-edge turn dispatch then re-checked the
**live IMU heading** against the outbound edge heading and fired a real turn when the delta exceeded
`NAV_TURN_TOLERANCE_DEG` (0.5°). The LineFollower corrects lateral position only and never feeds the
gyro, so IMU-yaw drifts freely over the inbound leg; by the junction the accumulated drift routinely
exceeds 0.5°, so a geometry-correct pass-through was broken into a stop-and-turn on the real robot.
Each such stop is actively harmful (`project-fewer-stops-better-friction-drift`): it trades continuous
line-follow correction for a static-friction break-away that slips the wheels and drifts the next move.

On a LINE continuation the physical line **is** the heading authority — the follower is live and will
steer the chassis back onto it — so consulting the drifted gyro is both unnecessary (the line
self-corrects) and counterproductive (it adds a stop). Suppressing the turn lands the FSM in the
exact same continue-live state as a driftless pass-through, which is already the tested happy path.
The ADR-052 junction heading-reset (`LINE_HEADING_RESET_ON_JUNCTION`, default OFF) is the other lever
for this drift and stays available; this rule fixes the behaviour without depending on that reset
firing or on its 10° strict-reject gate.

## How to apply

- The guard is the first statement of `PathTracker::_dispatchTurnIfNeeded`; it returns `false` (the
  "no turn" result), so the caller dispatches the edge and the LINE_FOLLOW_INTERSECTION fast-path
  keeps the follower live (no ramp reseed). `LINE_LOST` (#146) remains the safety net if the follower
  genuinely leaves the line.
- The **sim** (`tools/path_planner/sim/runtime.py`) needs no logic change: it has no IMU-drift model,
  so on a collinear hop its `_emit_turn_to` delta is already sub-tolerance and it never turns. The
  toggle is mirrored there as a value for config-contract completeness only. **Parity** and **CTPS**
  react to the presence/absence of a turn primitive rather than re-deciding it, so neither changes.
- The divergent (drift) case is exercised only in a host C++ test that injects `mmL.mockYawDeg` drift
  on a `_passThrough` LINE hop — the sim cannot reproduce it. `test_path_tracker_passthrough_drift_{on,off}`
  proves both the fix (ON: no turn, follower live) and the bug it fixes (OFF: drift breaks the pass-through).

## Trigger to revisit

- Bench data shows the follower cannot recover a large residual heading over a short LINE hop (i.e.
  the line-follow lookahead is too short to absorb the suppressed correction) — then re-introduce a
  bounded live-yaw turn above some larger threshold, or lean on `LINE_HEADING_RESET_ON_JUNCTION` instead.
- The classification itself proves wrong (a hop marked pass-through that is not truly collinear) — that
  is an `_evaluateLineFollowPassThrough` bug, not this rule; fix it there.

## See also

- [ADR-023 — Pass-through collinear intersections + runtime overshoot-mode probe](./023-pass-through-collinear-intersections.md) — this rule reverses its "reuse `NAV_TURN_TOLERANCE_DEG` for the pre-edge yaw turn" coupling for the LINE pass-through case.
- [ADR-052 — Wall-free heading-only re-anchor at collinear line pass-throughs](./052-line-arrival-heading-reset.md) — the other lever for the same IMU-yaw drift; its `_ON_JUNCTION` knob stays default OFF.
- [ADR-059 — Collinear runs chain velocity across hops](./059-collinear-run-continuous-motion.md) — the continuous-cruise family this protects.
- Modules: `WRO2026/src/mission/PathTracker.cpp` (`_dispatchTurnIfNeeded`), `WRO2026/config/ConfigNav.h` (the toggle), `tools/path_planner/sim/runtime.py` (mirrored value), `WRO2026/test/test_path_tracker_passthrough_drift.cpp`.
