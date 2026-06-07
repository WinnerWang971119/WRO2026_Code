# ADR-018: `LineFollower.setDistanceTarget(cm)` uses a symmetric trapezoidal profile with active brake at the gate

**Status:** Implemented (Refined by ADR-023)
**Date decided:** 2026-05 (Phase 2 PR2)

## Rule

`LineFollower.setEnabled(true).setDistanceTarget(cm)` applies:
- Cruise **ramp-up** from `Config::TSHAPE_START_SPEED` (the motor deadband floor, == `PWM_MIN_MAGNITUDE`, sign-corrected for reverse) up to cruise PWM, on the off→on transition only (skipped on re-enable).
- Cruise **plateau**.
- Cruise **ramp-down** from cruise PWM to `Config::PWM_MIN_MAGNITUDE` over the approach distance.
- Final **active brake** at the gate, fired at floor PWM (not from cruise).

The ramp-up START floor is symmetric with the ramp-down END floor on purpose: starting the ramp from 0 makes the first tick command sub-deadband PWM (snapped flat by `deadbandSnap`), and on a centered line the very first tick is `setSpeed(0, 0)` — the chassis cannot break static friction and the distance-gated ramp cannot advance. Same rationale as `Config::TSHAPE_START_SPEED` for Drive's sibling profile (ADR-014).

Ramp distances are fitted to available distance. With no ramp-up (re-arm case like `LINE_OVERSHOOT`) the ramp-down consumes the whole travel; with both ramps active each is clamped to half. Manual opt-outs via `setTShapeRampUp(bool)` / `setTShapeRampDown(bool)`.

## Why

The active brake at floor PWM (rather than braking from cruise) eliminates the lower MCU's brake-from-cruise stopping distance variance. This matters for `PROJECTION_FOLLOW` node-precision: stopping reliably at the intended node, not 1-3 cm past it.

The re-arm guard fixes a Copilot-flagged bug in PR #12: PathTracker's `LINE_OVERSHOOT` state re-armed an already-enabled LineFollower with `setEnabled(true).setDistanceTarget(LINE_SENSOR_AXLE_OFFSET_CM)`, which under the old behavior would drop cruise to 0 and ramp back up. The new guard only seeds the ramp-up on a true off→on transition; re-enabling a running controller preserves whatever ramp/plateau state was in flight.

## How to apply

- Default: just chain `.setDistanceTarget(cm)` — symmetric trapezoid is automatic.
- Re-arm case (LineFollower already on, just want a new distance gate): chain `.setTShapeRampUp(false)` defensively at the call site. The guard already suppresses the reseed, but the explicit chain documents intent.
- Per-side ramp opt-out via `setTShapeRampUp(false)` / `setTShapeRampDown(false)`.

## Trigger to revisit

If venue mat surface changes the brake stopping distance significantly (e.g., much higher friction), revisit `PWM_MIN_MAGNITUDE` rather than restructuring the profile.

## See also

- [ADR-014](./014-tshape-trapezoidal-velocity-profile.md) — Drive-side tShape (sibling profile)
- [ADR-023](./023-pass-through-collinear-intersections.md) — refines this ADR by adding a runtime line-presence probe option (`setOvershootMode(true)`) for the 5 cm sensor-to-axle overshoot window.
- `WRO2026/LineFollower.cpp` — `_applyEnableBuilder`
