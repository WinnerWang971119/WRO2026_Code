# ADR-015: Drive owns a FIFO command queue; builder destructors enqueue, not dispatch

**Status:** Implemented
**Date decided:** 2026-05 (Phase 1.5)

## Rule

`DriveClass` owns a fixed-capacity FIFO queue (`Config::DRIVE_QUEUE_CAPACITY = 8`). Builder destructors capture parameters into a `DriveCommand` POD and **push** onto the queue. `Drive.update()` is the only thing that pops and dispatches.

`Drive.update()` runs three stages per tick:
1. Drain any in-flight bounded MMLower task.
2. Advance the trapezoidal-profile state machine.
3. If chassis is fully idle and queue is non-empty, pop and dispatch.

Stage order matters: stage 2 must flip `_moveDone` before stage 3 looks at it, so back-to-back queued moves dispatch with no inter-segment gap.

Queue overflow drops the new command and prints a Serial warning — surfaces caller bugs rather than silently degrading.

## Why

Without the queue, RAII-execute on the builder fired at every semicolon, **before** any `loop()` tick. Successive builder destructors cancelled in-flight work from the previous, so only the *last* command actually ran. The auto-path planner, which issues dozens of moves in source order, would have been unusable.

The queue makes source-order intent (`driveDist(30); driveDist(20); turnDeg(90);`) work as written.

## How to apply

- `Drive.stop()` clears the queue (true abort semantics — pending work dropped).
- `setBlocking(true)` bypasses async queue semantics by spinning `_drainBlocking()` until the queue is empty and chassis is idle.
- LineFollower is auto-disabled when a new command pops off the queue — prevents LineFollower's `setSpeed` writes from racing the queued bounded move.
- Profile DONE branch is queue-aware: when `endSpeed == 0` we inline `stop()` equivalents *without* `_clearQueue()`; when `endSpeed != 0` we issue no command so the prior tick's setpoint holds until the next queued command overwrites it.
- Each `_execXxx` async path only sets `_moveDone = false` when the lower MCU returned `0x00` — without this guard, a failed dispatch would hang `_drainBlocking()` on a task that never started.

## Trigger to revisit

Queue capacity hit (8 commands) in real mission code — bump capacity or split the maneuver. Stage-ordering bug that re-surfaces (e.g., LineFollower auto-disable races a profile completion) — re-examine the per-tick stage order.

## See also

- [ADR-014](./014-tshape-trapezoidal-velocity-profile.md) — profile state machine that the queue dispatches into
- [ADR-016](./016-sync-default-builders.md) — sync-default blocking spins use the queue's drain
