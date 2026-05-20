# ADR-004: Motor-speed / position PID stays on the STM32; only outer-loop PID (line, future yaw-track) is ours

**Status:** Implemented
**Date decided:** 2026-05 (Phase 0)

## Rule

The application never sees PWM duty cycles or raw encoder counts in the inner control loop. When the application says "set the left motor to 200 RPM," the STM32 closes the loop on encoder feedback by itself.

The only PID loops the application owns:
- **Line offset → wheel-speed difference** — in `LineFollower::update()`.
- (Future) outer-loop yaw-track regulators — if added.

## Why

The Matrix R4's STM32 co-processor handles low-level motor control internally. Putting motor PID in the application would either duplicate or fight the STM32 loop. The right division is: STM32 owns "make the wheel hit this speed"; the app owns "compute what the wheel speed should be."

## How to apply

- `Drive.setSpeed(L, R)` writes to the STM32; the STM32 does the closed-loop PWM.
- Sync mode (`MoveSync`), Gyro mode (`MoveGyro`), and Turn (`TurnGyro`) all close their loops on the STM32. Application code only tunes their gains via `Config::SYNC_PID_*` / `GYRO_PID_*` / `TURN_PID_*`.
- LineFollower computes its own PID error and outputs a wheel-speed *delta* added on top of cruise. This is the only application-level PID in the steady-state code path.

## Trigger to revisit

A defect in STM32 firmware that makes one of the inner loops unfit for use. Or a board swap that removes the co-processor.

## See also

- [ARCHITECTURE.md](../../ARCHITECTURE.md) — "Where PID lives" table
- [ADR-011](./011-motor-bias-strategy-b.md) — why the outer PWM path still gets a feedforward bias
