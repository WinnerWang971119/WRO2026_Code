# ADR-012: Fluent builder for any verb with 3+ knobs; direct call for 0–2

**Status:** Implemented
**Date decided:** 2026-05 (Phase 1.5)

## Rule

Any verb that takes 3 or more configurable parameters is exposed as a `[[nodiscard]]`-marked fluent builder. Chained `.setX(...)` setters return a reference; the action executes when the temporary destructs at end-of-statement (RAII auto-execute).

Verbs that take 0–2 parameters stay direct (non-fluent).

Examples:
```cpp
Drive.driveDist(30).setMode(DriveMode::GYRO).tShape().setBrake(false);   // 4 knobs → builder
Drive.setSpeed(50, 50);                                                  // 2 knobs → direct
```

## Why

Wide positional argument lists (`moveDist(power, cm, brake, mode, blocking, heading)`) become unreadable at the call site within weeks — every parameter has to be looked up, and every reordering breaks every caller. Without the rule, the Drive layer would grow half a dozen mutually-exclusive `driveDistSync` / `driveDistGyro` / `driveDistGyroBlocking` variants duplicating doxygen and hiding the differentiator behind a method-name suffix.

3-knob threshold is empirical: at 2 parameters direct calls are still readable; at 3+ the cost of looking up positional meaning exceeds the cost of typing `.setX()`.

## How to apply

- Builder destructors capture parameters into a POD and dispatch to a private `_execXxx()` — they do not block (see [ADR-015](./015-drive-command-queue.md), [ADR-016](./016-sync-default-builders.md)).
- Builders are copy-deleted; move is implemented with an `_executed` flag transfer for idempotent destruction.
- `[[nodiscard]]` makes "I built it and forgot to execute" a compile warning.
- Rule applies project-wide: Drive, LineFollower's `setEnabled`, PathTracker's `goTo`, Mission verbs. Future modules inherit the convention.

## Trigger to revisit

A use case where a builder's RAII destruction is wrong (e.g., the call site genuinely wants to construct without executing). Until then, `[[nodiscard]]` + RAII is the default.

## See also

- [ADR-015](./015-drive-command-queue.md) — queue dispatcher that the builders feed
- [ADR-016](./016-sync-default-builders.md) — destructor blocking policy
