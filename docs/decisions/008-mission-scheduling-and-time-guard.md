# ADR-008: Greedy score-per-second task scheduling; 2-minute time guard with forced return-to-start

**Status:** Partially implemented (Mission DSL shipped; greedy scheduler + time guard deferred to Phase 4)
**Date decided:** 2026-05 (Phase 0)

## Rule

- The `Mission` scheduler treats task selection as an explicit optimization: each task carries an estimated duration and an expected score; a greedy "score per second" ordering decides what to attempt and in what order.
- Every task carries a max-time guard. If a task overruns, the scheduler aborts it and moves on.
- With ~20 s left in the 2-minute run, the robot abandons whatever it's doing and executes return-to-start (to grab brick-trowel-in-start-zone points and avoid finishing in a random location).

## Why

Max score is 233 but no team realistically clears every task in 2 minutes. Without explicit scoring optimization, the run defaults to whatever-was-coded-first; with it, the schedule is tunable per venue and per practice result.

The 20 s return-to-start guard is the difference between "ran out of time mid-task with 0 incremental points" and "got the trowel return + ended safely."

## How to apply

- PR3 ships the operator-authored Mission DSL (`goTo` / `wait` / `print` / `beep` / `led`) — hand-typed task order, no automatic scheduling yet.
- Phase 4 will add the greedy ordering on top of the DSL: estimated-duration / expected-score metadata on each task, sort, dispatch, abort-on-overrun, forced return-to-start at `t = 100 s`.

## Trigger to revisit

If practice runs show the greedy heuristic consistently picks the wrong order (e.g., cement delivery is undervalued because its time estimate is too conservative), revisit the per-task estimates before changing the heuristic.

## See also

- [Roadmap.md](../../Roadmap.md) — Phase 4
- [ADR-009](./009-failure-recovery-first-class.md) — what happens when an individual task fails
