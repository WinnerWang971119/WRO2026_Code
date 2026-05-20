# ADR-013: `LineFollower.setEnabled(false)` does not implicitly brake — the planner owns end-of-segment behavior

**Status:** Implemented
**Date decided:** 2026-05 (Phase 1.5)

## Rule

`LineFollower.setEnabled(false)` clears `_enabled`, zeroes `_lastErr`, clears `_lostLine` — that part is invariant. Whether the chassis actively brakes is the **planner's** decision, expressed through `.setBrake(bool)` on the EnableBuilder.

Default is `setBrake(true)`. Explicit `setEnabled(false).setBrake(false)` lets the caller take over from there — if no follow-up command is issued, the chassis continues at whatever speeds were last latched on `Drive`.

The `setBrake` flag is silently ignored on `setEnabled(true)`.

## Why

The old "disable always brakes" behavior made the most common path-planner move impossible: "drop the line and coast a little further before turning, so there is room to swing into the next leg." The planner had no way to ask for "stop following but keep rolling."

The general principle: any module that "turns off" should not implicitly issue actuator commands on behalf of the caller. The next layer up is the right place to decide what comes next.

## How to apply

- Coast-into-corner pattern: `LineFollower.setEnabled(false).setBrake(false); Drive.driveDist(...).tShape().setStartSpeed(cruise);`
- Stop-and-hold pattern: `LineFollower.setEnabled(false);` (default-brake=true is fine).
- Doxygen on `setBrake` documents the silent-ignore behavior when chained after `setEnabled(true)`.

## Trigger to revisit

If a generalization of this principle appears in a different module (e.g., `Manipulator.setEnabled(false)` ambiguity), the same "no implicit actuator command" rule applies — refactor in that direction.

## See also

- `WRO2026/LineFollower.h` — `LineFollowerEnableBuilder`
