# ADR-009: Three named failure modes get explicit, bounded recovery branches

**Status:** Partially implemented (lost-line + missed-intersection in Phase 1/2; failed-grip pending manipulator)
**Date decided:** 2026-05 (Phase 0)

## Rule

Three failure modes are first-class and handled explicitly. Each recovery is **simple, bounded, and cheap** — never heroic.

1. **Lost line** — LineFollower stops seeing readings consistent with a line. Pause, sweep slowly left/right, resume if reacquired.
2. **Missed intersection** — too much time has passed since the last expected node. Reverse a short distance and re-run the intersection detector.
3. **Failed grip** — Manipulator reports the target wasn't picked up. Task retries once, then yields to the next task.

One task failing should never cascade into the rest of the run failing.

## Why

Without explicit recovery, any one failure ends the run. With explicit recovery, individual tasks degrade gracefully and the scheduler keeps the higher-value tasks running.

The bound matters as much as the recovery itself. An unbounded sweep or unbounded retry burns the 2-minute budget; bounded ones cost a known amount of time and the scheduler can decide whether to abort.

## How to apply

- Each recovery branch lives in the module that owns the failure (lost-line in LineFollower, missed-intersection in PathTracker, failed-grip in Manipulator-side Tasks).
- Each recovery has a hard timeout in `Config`. On timeout, escalate to the scheduler.
- Recoveries do not retry indefinitely — at most one retry, then yield.

## Trigger to revisit

A new failure mode that turns up on the practice field — e.g., the chassis lifts off the floor on an obstacle and the encoders read garbage. Add a fourth named recovery rather than overloading an existing one.

## See also

- [Roadmap.md](../../Roadmap.md) — Phase 4 task P4.6/P4.7/P4.8
