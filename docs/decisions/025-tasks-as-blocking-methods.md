# ADR-025: Tasks layer composes subsystem calls as blocking method bodies; no JSON authoring

**Status:** Implemented
**Date decided:** 2026-05

## Rule

Tasks are `bool` instance methods on `TasksClass`, registered by name in a
compile-time string→member-fn-ptr table in `Tasks.cpp`. `Mission`'s `.task(name,
args...)` verb enqueues a `TASK` step that dispatches into `Tasks.runTask` at
update-time. Task method bodies fake-block by composing existing sync-default
builders (`PathTracker.goTo`, `Drive.driveDist`, etc.) — the same drain pattern
ADR-016 codified. Operator authoring stays in C++ in `WRO2026.ino::buildMission()`.
JSON as an authored format is explicitly rejected.

## Why

A debate skill run returned a strong No on JSON authoring (verdict 3.75/10
against). The R4 has no SD card, so the one benefit JSON provides — recompile-free
edits at runtime — is eliminated: task bodies would have to be baked into PROGMEM
anyway. The `022-field-data-json-loader.md` precedent already keeps enum-rich
tables in Python rather than JSON because JSON carries no enum type; task bodies
are even more enum-rich (subsystem calls reference `Direction`, `DriveMode`,
`JunctionType`, etc.). Replacing 22 lines of fluent C++ with ~600–1000 lines of
JSON loader glue is a net loss in readability with no corresponding gain.

Blocking method bodies win over per-task FSMs for the same reason sync-default
builders won in ADR-016: readability. A task body reads like a sequential script,
making field-day debugging straightforward. The trade-off is that a blocking task
cannot be preempted mid-body; this is acceptable because individual subsystem drains
already operate this way under ADR-016, and any task that needs preemption can be
converted to a per-task FSM later without changing the registry shape. The Phase 6
greedy scheduler (ADR-008) can target specific slow tasks at that point.

## How to apply

- Every task is a `bool TasksClass::<name>(const TaskArgs&)` method declared in
  `Tasks.h` and registered in the `_registry[]` table in `Tasks.cpp`. Hand-edit the
  table to add a new task; do not use string-keyed maps or runtime registration.
- Task bodies call existing sync-default builders. Return `true` for OK (Mission
  advances) or `false` for yield (Mission logs a `TASK YIELD` event and advances).
  Retry-once policy from ADR-009 lives inside task bodies, not in the Mission FSM.
- Operators author missions in `WRO2026.ino::buildMission()` as a chain of
  `Mission.<verb>(...)` calls including `.task(name, args...)`. Task names are
  validated against `Tasks._registry` at queue time inside `task<>()`: an
  unknown name does NOT enqueue, sets a sticky `_buildError` latch, and
  `Mission.start()` refuses to arm — surfacing typos before any chassis motion.
- **Mission verbs are queue-only — do not call them from task bodies.** All
  `Mission.<verb>(...)` calls (including `goTo` / `wait` / `print` / `beep` /
  `led` / `task`) APPEND to the queue; calling them inside a task body queues
  the step for execution AFTER the current TASK step's `_advanceStep`, not
  synchronously. Task bodies that need to drive the chassis, beep, or check
  state MUST use the underlying singletons directly: `PathTracker`, `Drive`,
  `LineFollower`, `MiniR4.Buzzer`, `MiniR4.LED`. Only `task` guards against
  nested calls with a Serial warning; the others queue silently.
- **Sync-drain-inside-tick exception to ADR-016:** ADR-016 forbids sync-blocking
  calls from registered tick callbacks. Task bodies execute inside
  `Mission.update()` via `Tasks.runTask`, which IS called from `cooperativeTick`.
  This is a deliberate exception: the Mission FSM owns the cooperative tick at the
  operator-script layer, mirroring what `buildMission()` would do as an unrolled
  blocking sequence in `setup()`. Re-entrancy is bounded by the `Tasks.inTask()`
  guard checked at the top of `MissionClass::update()`.
- **`Tasks._aborted` lifecycle.** `Mission.abort()` calls `Tasks.abort()` FIRST
  (before disabling PathTracker/LineFollower/Drive) so any subsystem polling
  `Tasks.isAborted()` during its own shutdown sees the latched flag. The latch
  is cleared by `Mission.start()` via `Tasks.clearAbort()` so it does not bleed
  across mission runs. Task bodies that want cooperative cancellation should
  poll `Tasks.isAborted()` at appropriate checkpoints inside their script.
- Sim parity (issue #33 CTPS): the name-string registry is enumerable; a Python
  mirror of each task body follows naturally because both languages support
  sequential script-style flow. Sim work is a follow-up to this ADR.
  **Superseded by ADR-031:** this "Python mirror of each task body" direction was
  not taken. CTPS records the real C++ at the hardware-facade boundary
  (a `CTPS_RECORD` hook plus recording mocks) and ingests a derived trace, so
  there is no Python re-implementation of task bodies. See
  [ADR-031](./031-ctps-derived-mission-trace.md).

## Trigger to revisit

- Phase 6 greedy scheduler (ADR-008) requires mid-body preemption of a running task.
  Blocking bodies cannot be preempted; convert the specific task to a per-task FSM
  (the registry shape supports this without a framework rewrite).
- Registry grows past 16 tasks, making the hand-mirrored memcpy arg layout
  convention awkward; consider promoting to per-task strongly-typed param structs.
- Sim parity (issue #33 CTPS) ships and the Python mirror diverges enough that
  codegen from a shared source becomes warranted. (Moot under ADR-031: CTPS
  records the real C++ rather than mirroring task bodies in Python, so there is
  no second source to diverge.)

## See also

- [ADR-007 — Manipulator exposes control primitives only](./007-manipulator-control-primitives-only.md)
- [ADR-008 — Greedy score-per-second scheduling + 2-minute time guard](./008-mission-scheduling-and-time-guard.md) (partially implemented)
- [ADR-009 — Three named failure modes get explicit bounded recovery](./009-failure-recovery-first-class.md)
- [ADR-016 — Drive builders block by default; `.async()` opts out](./016-sync-default-builders.md)
- [ADR-031 — CTPS ingests a derived mission trace recorded from the real C++](./031-ctps-derived-mission-trace.md) — supersedes the "Python mirror of each task body" sim-parity sub-decision above.
- [`022-field-data-json-loader.md`](./022-field-data-json-loader.md) — JSON works for geometry data but is explicitly rejected for enum-rich task bodies; contrast with this ADR.
- [`docs/plans/2026-05-26-tasks-framework-mission-task-verb.md`](../plans/2026-05-26-tasks-framework-mission-task-verb.md) — the originating implementation plan.
