# ADR-026: `Tasks.runTask` is non-reentrant — nested dispatch is rejected

**Status:** Implemented
**Date decided:** 2026-05 (Phase 5)

## Rule

A task body must not dispatch another task. `TasksClass::runTask` reads `_inTask`
on entry and, if a task is already in flight, refuses: it prints a
`[Tasks] runTask: refusing reentrant call to '<name>'` warning and returns
`false` without resolving the name or arming the `InTaskGuard`. Tasks are leaf
operations — logic shared between two tasks lives in a private helper method on
`TasksClass`, not in one registered task calling another.

## Why

`runTask` brackets each dispatch with an `InTaskGuard` whose destructor
unconditionally clears `_inTask`. That guard does not save and restore the prior
value, so a nested `runTask` would clear `_inTask` the moment the *inner* task
returned — while the *outer* body is still running. Two things break as a result:

1. `MissionClass::update()` short-circuits on `if (Tasks.inTask()) return;` so a
   fake-blocking task body that re-enters `cooperativeTick()` (while draining a
   `Drive` / `PathTracker` sync wait) does not make Mission recurse into the same
   `TASK` step. A clobbered `_inTask` defeats that guard: any sync-blocking work
   in the outer body after the nested call returns lets `Mission.update()` run the
   step again — unbounded re-execution.
2. There is no depth bound or cycle detection. A task calling itself, or `A→B→A`,
   recurses until the small Cortex-M4 stack is exhausted.

Rejecting nested dispatch outright is the cheapest fix that removes all of these
at once, and it strengthens rather than loosens the single-task-in-flight
invariant that the Mission FSM already depends on. Allowing safe nesting (save/
restore the flag, optionally with a depth cap) was considered and rejected:
nothing in the current architecture needs composable registered tasks, and the
natural way to share steps between tasks is a plain private helper, which keeps
the registry a Mission-facing concept.

## How to apply

- Inside a task body, never call `Tasks.runTask(...)`. Call the underlying
  singletons directly (`PathTracker`, `Drive`, `LineFollower`, `MiniR4.Buzzer`,
  `MiniR4.LED`) — the same rule ADR-025 states for Mission verbs.
- To reuse a sequence across tasks, factor it into a private method on
  `TasksClass` and call that from each task body. A private helper is an ordinary
  function call: it does not touch `_inTask` and is not registry-visible.
- The guard complements ADR-025's queue-layer check: `Mission.task()` already
  ignores a `task()` enqueue made from inside a task body (it would otherwise
  append a step that runs *after* the current step's `_advanceStep`). ADR-026
  closes the other vector — a body bypassing Mission and dispatching directly.

## Trigger to revisit

- A concrete task genuinely needs to invoke another *registered* task (e.g. a
  Phase 6 composite). At that point switch `InTaskGuard` to save/restore the prior
  `_inTask` value and add a `Config::MAX_TASK_DEPTH` counter with a cap, then
  rewrite this ADR's rule rather than editing it in place.

## See also

- [ADR-025 — Tasks layer composes subsystem calls as blocking method bodies](./025-tasks-as-blocking-methods.md) — establishes the queue-only-verbs rule and the `Mission.update()` re-entrancy guard this ADR backs up at the dispatch layer.
- [ADR-016 — Drive builders block by default; `.async()` opts out](./016-sync-default-builders.md) — the fake-blocking drain pattern task bodies use.
- `WRO2026/Tasks.cpp::runTask` — the guard; `WRO2026/test/test_tasks_dispatch.cpp` TC15 — the regression test.
