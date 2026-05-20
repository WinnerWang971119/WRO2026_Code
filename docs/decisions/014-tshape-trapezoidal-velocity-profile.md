# ADR-014: T-shape trapezoidal velocity profile — acceleration-based, opt-in per call

**Status:** Implemented
**Date decided:** 2026-05 (Phase 1.5)

## Rule

`.tShape()` is a fluent modifier on Drive distance/time builders and on LineFollower's enable builder. When enabled, the move ramps from rest (or `setStartSpeed`) up to cruise, holds cruise, then ramps back down to `setEndSpeed` (default 0).

Profile is **acceleration-based**, not proportion-based: `Config::TSHAPE_ACCEL_PWM_PER_CM` defines PWM gained per cm of encoder travel; per-side ramp distance falls out as `(cruise - endpoint) / accel`.

Smart default: if the move is configured `.setBrake(false)`, the default endSpeed becomes `cruise` instead of 0 — assumes the caller is chaining into a follow-up move and wants speed preserved at the join.

## Why

Constant-PWM moves lurch — full power instantly at start, full stop instantly at end. Acceptable for short moves; jarring for long ones, dangerous for fragile cargo (mosaic tiles, cement blocks), and accumulates yaw error during the abrupt acceleration window. The trapezoidal profile fixes all three.

Acceleration-based (not proportion-based) because it's more physically meaningful and makes chained moves natural — pass `endSpeed = cruise` and the formula returns 0 (no ramp-down).

## How to apply

- Per-side ramp distances are clamped to `[TSHAPE_RAMP_MIN_CM, TSHAPE_RAMP_MAX_CM]`. Also capped at half the move so the two ramps don't overlap.
- If acceleration + deceleration distance > total move length, profile collapses to a triangle: peak speed = `(start + end + cruise) / 3` clamped ≤ cruise.
- No-ramp epsilon (`TSHAPE_NO_RAMP_EPSILON_PWM = 2`): skips the ramp on the side where startSpeed or endSpeed is already at cruise.
- Profile runs as a cooperative state machine driven from `Drive.update()`; reads encoder (or `millis()`) per tick, looks up setpoint, issues continuous primitive per chosen `DriveMode`.
- `driveTime`-with-tShape: cm-based accel reinterpreted as `rampMs ≈ rampCm / |cruise| * 1000`. Approximation, but consistent API.
- **Turn ramping is out of scope** — `TurnGyro` is internally bounded by the lower MCU. Intercepting tick-by-tick to vary power would require a custom continuous gyro-locked turn loop. Separate project.

## Trigger to revisit

If long-distance accuracy degrades because the ramp-up window is too wide, expose `TSHAPE_ACCEL_PWM_PER_CM` per-builder. If turn smoothness becomes a real cargo-safety issue, build the custom turn ramp.

## See also

- [ADR-017](./017-sync-plus-tshape-default-profile.md) — why tShape is also the default
- `WRO2026/Drive.cpp` — `_updateProfile`
