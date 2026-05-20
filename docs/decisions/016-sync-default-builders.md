# ADR-016: Drive builders block by default; `.async()` opts out

**Status:** Implemented
**Date decided:** 2026-05 (Phase 1.5)

## Rule

Drive builder destructors block by default — the next line of mission code can assume the previous move completed. Hand-written mission scripts read like Python:

```cpp
Drive.driveDist(30);
Drive.turnDeg(-90);
Drive.driveDist(30);
```

The auto-path planner opts out per call with `.async()`:

```cpp
if (Drive.isIdle()) {
    Drive.driveDist(30).setMode(GYRO).tShape().async();   // enqueues, returns immediately
    advanceMissionState();
}
```

The blocking spin (`_drainBlocking()`) re-enters every module's `update()` each iteration via a registered tick callback (`Drive.setTickCallback(cooperativeTick)`) — so OLED, heartbeat, sensors keep running during the block.

## Why

Async-default required callers to remember `Drive.waitUntilIdle()` at every sync point — between consecutive moves, before manipulator actions that depended on chassis position, before any decision that read sensor state. This is exactly the kind of thing humans forget under deadline pressure: code looks complete, but a missing `waitUntilIdle()` silently runs the next command before the previous one finished, and you don't notice until the chassis does something weird on the field.

Sync-default flips the cost: forgetting `.async()` is harmless (it just runs synchronously), forgetting `waitUntilIdle()` is silent data corruption. The asymmetry favors sync.

## How to apply

- Hand-written mission code: don't think about it; just call.
- Auto-planner: chain `.async()` on every queued call.
- The registered tick callback **MUST NOT** issue sync-blocking Drive commands — that would re-enter `_drainBlocking()` and recurse. Today no module's `update()` issues queued Drive commands; LineFollower uses continuous primitives (`setSpeed`, `stop`) which bypass the queue. Documented as `@warning` on `setTickCallback`.
- `isIdle()` is queue-aware (true only when chassis AND queue are both empty). `isMoveDone()` / `isTurnDone()` are queue-blind — they carry a `@warning` to reach for `isIdle()` when queue-awareness is wanted.
- If no tick callback is registered, `_drainBlocking()` falls back to ticking only `Drive.update()` and emits a one-time Serial warning.

## Trigger to revisit

A future module's `update()` genuinely needs to issue a Drive move — must use `.async()`. If many do, consider promoting it to a hard contract enforced at compile time (e.g., a "tick context" flag).

## See also

- [ADR-015](./015-drive-command-queue.md)
- `WRO2026/Drive.cpp` — `_drainBlocking`, `setTickCallback`
