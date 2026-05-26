# Tasks Framework + Mission `.task()` Verb Plan

**Goal:** Add a Tasks layer (composite multi-step robot routines) between the existing Mission DSL and the subsystem facades (PathTracker, Manipulator, camera-TBD), with a new `Mission.task(name, args...)` verb that dispatches into it. This plan ships the **framework only** — registry, dispatch, return-code semantics, FSM wiring — with a single stub task (`demoEcho`) so tests can exercise dispatch end-to-end. Real task bodies (tool delivery, mosaic place, cement delivery, brick trowel return) land in follow-up plans once Manipulator hardware and the camera module are real (Roadmap P5.5+).

**Approach:** A new `TasksClass` singleton, mirroring `MissionClass` in shape. Each task is a **`bool` instance method** on `TasksClass` whose body composes existing sync-default builders (`PathTracker.goTo(...)`, `Drive.driveDist(...)`, future `Manipulator.setGripper(...)`). The methods "fake-block" using the same drain pattern Drive sync-default already uses (ADR-016) — they appear sequential to the reader, but internally each sub-call re-enters `cooperativeTick()` so every other module keeps ticking. A fixed-size compile-time **name-string → member-function-pointer registry** in `Tasks.cpp` lets `Mission.task("name", ...)` dispatch by operator-friendly string. Arguments use a **16-byte POD `TaskArgs` union** packed at the call site via a variadic-template Mission verb (`memcpy` from typed args to the union). The retry-once-then-yield policy from ADR-009 lives **inside task bodies**, not in the Mission FSM — bodies return `true` for "advance" and `false` for "yield." JSON authoring was explicitly debated and rejected (see "Decisions" below); the operator continues authoring missions as hand-typed C++ fluent calls in `WRO2026.ino::buildMission()`.

## Scope

- **In scope:**
  - New module `WRO2026/Tasks.h` + `Tasks.cpp` (`TasksClass` singleton).
  - `Mission.task<Args...>(const char* name, Args... args)` variadic-template verb.
  - New `StepType::TASK` in `MissionClass::Step`, plus 16-byte `TaskArgs` storage slot and `const char* task_name`.
  - `TaskArgs` POD union (16 bytes) + `TaskFn` typedef + `TaskRegistryEntry` row type.
  - Compile-time registry table in `Tasks.cpp` (single source of truth for valid task names).
  - One stub task body `TasksClass::demoEcho(const TaskArgs&) -> bool` that logs `[Tasks] demoEcho a=<u8> b=<u8>` and returns `true`. Exists only so dispatch + arg-pack tests have a target.
  - Mission FSM extension: TASK step type — ENTER calls `Tasks.runTask(name, args)` (blocking); on return, `true` advances the queue, `false` advances the queue with a `TASK YIELD` event, unknown name triggers Mission's existing `_enterFaultBeeping` path.
  - `Mission._inTask` re-entrancy guard so `cooperativeTick → Mission.update()` re-entry inside a task body is a no-op.
  - `Tasks.abort()` hook wired into `Mission.abort()`.
  - `Tasks.update()` ticked from `cooperativeTick()` between `Manipulator.update()` and `PathTracker.update()`. Update body is housekeeping-only in this plan (push `TASK START <name>` / `TASK OK <name>` / `TASK YIELD <name>` EventTrace events at boundaries; no FSM state).
  - Host-compile C++ test target `WRO2026/test/test_tasks_dispatch.cpp` + CMake entry, mirroring the T17 (`test_path_tracker_bidirectional.cpp`) pattern.
  - ADR-025 capturing the framework-as-blocking-methods + no-JSON decision.
  - Doc updates: `DECISIONS.md` index, `ARCHITECTURE.md` module map + dependency graph, `Roadmap.md` P5.4 status note.
- **Out of scope:**
  - **JSON authoring of Missions or Tasks.** Decided against via the `/debate` skill (verdict: Strong No, 3.75/10). Rationale captured in ADR-025.
  - **Sim parity / issue #33 CTPS.** Tasks framework will be designed so sim parity is easy (name-string registry is enumerable, blocking-method bodies mirror trivially in Python), but no sim code lands in this plan. Separate plan after this one closes.
  - **Real task bodies** (tool delivery, mosaic pattern read, mosaic place, cement delivery, brick trowel return). Roadmap P5.5 onward.
  - **Greedy score-per-second scheduler** (ADR-008). The current Mission queue stays hand-ordered.
  - **2-minute time guard / forced return-to-start** (ADR-008, Roadmap P6.6). Tasks framework adds no per-task timeout enforcement; if a task overruns, the operator notices on the field this season.
  - **Failed-grip recovery** as a framework feature. Retry-once policy from ADR-009 lives inside task bodies that need it (i.e., the future `mosaicPlace` / `toolDelivery` will internally retry once before returning `false`).
  - **Mission scheduler preemption** of a running task. The blocking-method choice trades preemption for "easy code"; if Phase 6 needs preemption, specific tasks can be converted to per-task FSMs later without changing the framework.
  - **Nested task calls.** A task body must not call `Mission.task(...)`. Enforced by the `_inTask` guard returning early; documented in `Tasks.h`.

## Decisions

- **No JSON authoring.** Debate ran on the proposition "Define Missions + Tasks as JSON files parsed by both R4 firmware and Python CTPS." Verdict Strong No (3.75/10). Reasons: (a) user's explicit objection — "if the code is using jason, it will be harder for us to code by manual which we don't want"; (b) R4 has no SD card, so PROGMEM-baking JSON re-introduces a recompile step that eliminates the only headline benefit; (c) the JSON-loader ADR ([`022-field-data-json-loader.md`](../decisions/022-field-data-json-loader.md)) precedent kept enum-rich tables out of JSON, and Task bodies invoke enum-rich subsystem methods (`Direction`, `DriveMode`, `ColorEnum`, `JunctionType`); (d) the `routes.h` codegen-from-Python pattern gives shared-truth without paying the JSON cost; (e) ~600-1000 lines of new glue replacing 22 lines of fluent C++ that already work. Captured in ADR-025.
- **Sync-drain-inside-tick exception to ADR-016.** ADR-016 normally forbids sync-blocking Drive/PathTracker/LineFollower calls from a registered tick callback (to prevent recursive `_drainBlocking`). Task bodies execute inside `Mission.update()` via `Tasks.runTask`, which IS called from `cooperativeTick`. This is a deliberate exception: the Mission FSM owns the cooperative tick at the operator-script layer, mirroring what `buildMission()` would do as an unrolled blocking sequence in `setup()`. The recursion is bounded by the `Tasks.inTask()` re-entrancy guard at the top of `MissionClass::update()`. ADR-025 records this exception and the bounding invariants explicitly so future contributors don't re-open the rule.
- **Tasks-as-blocking-methods (fake-block, Drive-style).** Each task is a `bool` member function on `TasksClass`. Body reads as a script: `if (!PathTracker.goTo("B")) return false; Manipulator.setGripper(180); ...`. Internally every sub-call drains via `cooperativeTick`, same as `Drive` sync-default per ADR-016. Trade preemption + per-task timeout for code that reads in 5 lines instead of 30. Conversion path to per-task FSM (when needed) is local — registry stores a function pointer either way.
- **`bool` return, retry-inside-task.** Task methods return `true` (advance to next Mission step) or `false` (yield; skip and advance to next Mission step). ADR-009's "one retry then yield" policy is implemented **inside the task body** with a local retry counter, not as Mission FSM machinery. Why: the user explicitly asked for the simpler model ("return true or false, we can do logic inside task and outside mission"). Keeps Mission FSM minimal.
- **Unknown-name → Mission fault.** `Mission.task("typoName")` resolves at enqueue time (`Tasks.findTask(name)` returns null) — but per parity with `Mission.goTo("unknownNode")` (Mission.cpp:273-281), the fault is emitted at **execute** time so the queue can still be authored speculatively. Mission's existing `_enterFaultBeeping(now)` path is reused; no new fault type.
- **Variadic-template + 16-byte memcpy pack for args, with per-task struct as the recommended layout convention.** Call site is compile-time-typed (variadic template enforces caller's argument types); the framework `memcpy`'s the typed pack into a fixed 16-byte POD union slot on the `Step`. Compile-time `static_assert((sizeof...(Args) == 0) || ((sizeof(Args) + ...) <= 16), "TaskArgs overflow")` guards overflow. **Layout convention:** each task body that takes args defines a `struct <Name>Args { ... };` and unpacks via `const auto* a = reinterpret_cast<const FooArgs*>(args.bytes);`. The corresponding `Mission.task("foo", FooArgs{...})` call site passes one struct, packing it as a single contiguous block (avoids the alignment-trap where `Mission.task("foo", uint8_t(7), uint32_t(0xCAFEBABE))` packs back-to-back without padding and produces a `u32[0]` that doesn't equal `0xCAFEBABE`). Mixing-raw-types call sites are permitted only when the layout is byte-identical to a single `uint8_t[N]` (e.g., two `uint8_t` args is fine; `uint8_t + uint32_t` is NOT — use a struct).
- **Tick order.** `Tasks.update()` runs between `Manipulator.update()` and `PathTracker.update()` in `cooperativeTick()`. Tasks ticks after the subsystems it owns are fresh; Mission still sees fresh task state in the same iteration.
- **No persistent active-task state in `TasksClass`.** The `runTask()` call is the entire lifecycle — there is no `start/poll/finish` surface. `update()` only emits boundary events (`TASK START`, `TASK OK`, `TASK YIELD`). Simplifies the FSM picture: Mission's `_stepState` is the only state machine for an in-flight task step.
- **No `Tasks` ↔ `Mission` cyclic header dep, and a `TasksTypes.h` thin header for shared POD types.** The shared POD types (`TaskArgs`, `TaskFn`, `TaskRegistryEntry`) live in a new `WRO2026/TasksTypes.h` so `Mission.h` can include just the POD layer without dragging the full `TasksClass` declaration into every Mission TU. `Tasks.h` includes `TasksTypes.h` and adds the class. `Mission.h` includes `TasksTypes.h` only. `Mission.cpp` includes `Tasks.h` and calls `Tasks.runTask(...)`. The opposite direction (Tasks → Mission) is forbidden — task bodies must NOT call `Mission.task(...)`; this is **actively enforced** by a `Tasks.inTask()` check at the top of `MissionClass::task<Args...>` that prints a Serial warning and skips enqueue (the `_inTask` guard only short-circuits Mission.update, not the enqueue verb). `Tasks.abort()` is invoked from `Mission.abort()`, so the runtime call edge stays one-way.
- **Defer Manipulator integration.** `Tasks.h` does not include `Manipulator.h`. Real task bodies will include it themselves when they land (P5.5+). The stub `demoEcho` task uses no subsystem.

## Acceptance Criteria

- [ ] AC1 — `WRO2026/WRO2026.ino` compiles via `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` with the Tasks framework included and `buildMission()` calling `Mission.task("demoEcho", uint8_t(7), uint8_t(42));`.
- [ ] AC2 — Stub task `TasksClass::demoEcho(const TaskArgs&)` logs `[Tasks] demoEcho a=7 b=42` to `Serial` and returns `true`; Mission advances to the next queued step (next `goTo` / `print` / etc. executes).
- [ ] AC3 — `Mission.task<uint8_t, uint8_t>("demoEcho", 7, 42)` packs the two `uint8_t` values into `Step::task_args` byte slots 0 and 1 (verified via host-compile test).
- [ ] AC4 — `Mission.task("doesNotExist")` triggers `MissionClass::_enterFaultBeeping(...)` exactly once, identical to `Mission.goTo("unknownNode")` behaviour. `Serial` prints `[Mission] unknown task 'doesNotExist'`; `EventTrace` pushes `NAV FAULT TASK`. Subsequent queue steps are dropped.
- [ ] AC5 — A task returning `false` emits a `TASK YIELD <name>` event via `EventTrace.pushEvent`, prints `[Mission] task '<name>' yielded` to Serial, and Mission advances to the next queued step (queue is NOT aborted; no fault path).
- [ ] AC6 — Subsystem fault during a task (`PathTracker.hasFault()` becomes true mid-body): Mission's `_emitFault()` + `_enterFaultBeeping(now)` path fires on the **first** Mission.update() call after the task method returns. Latency is bounded by the duration of the in-flight subsystem builder (existing ADR-016 cost).
- [ ] AC7 — `Mission.abort()` invokes `Tasks.abort()` which sets `_aborted = true`. After `Mission.abort()` returns, `Tasks.isAborted()` reads `true`. **This is a flag-wiring AC only.** Subsystem honour of the flag (Drive / PathTracker / LineFollower sync drains polling `Tasks.isAborted()` and bailing fast) is explicitly a follow-up to this plan; in this plan an in-flight task is NOT preempted by the abort signal — it runs until the current subsystem builder finishes, at which point existing `PathTracker.abort()` / `LineFollower.setEnabled(false)` / `Drive.stop(true)` from `Mission.abort()` have already braked the chassis, so subsequent `PathTracker.goTo()` calls inside the task return `false` immediately (PathTracker is IDLE post-abort) and the task body unwinds.
- [ ] AC8 — `cooperativeTick()` re-entry inside a blocking task body is a no-op for Mission: a re-entrant `MissionClass::update()` call returns immediately because `Tasks.inTask() == true`. Asserted via a host-compile test that mirrors the guard inline (per T17 precedent — does NOT link production `Mission.cpp`).
- [ ] AC9 — Compile-time `static_assert` rejects a `Mission.task<...>(...)` whose arg-pack total size exceeds `Config::MAX_TASK_ARG_BYTES` (16). Positive case: `Mission.task<uint32_t, uint32_t, uint32_t, uint32_t>(...)` (exactly 16 bytes) compiles. Negative case: `Mission.task<uint8_t[17]>(...)` (17 bytes) FAILS to compile. The negative case is enforced by a dedicated CMake target listed in `WRO2026/test/CMakeLists.txt` that uses `add_test(NAME tasks_overflow_compile_fail COMMAND ${CMAKE_COMMAND} --build . --target test_tasks_dispatch_overflow)` with `set_tests_properties(tasks_overflow_compile_fail PROPERTIES WILL_FAIL TRUE)`. The `test_tasks_dispatch_overflow` target is `EXCLUDE_FROM_ALL` and `EXCLUDE_FROM_DEFAULT_BUILD` so it doesn't break normal builds; ctest runs it explicitly and expects non-zero exit (compile failure).
- [ ] AC10 — Host-compile target `test_tasks_dispatch` builds and the resulting binary's `ctest` invocation returns 0, covering TCs 1, 2, 3, 5, 6, 7, 8, 9 (positive), 10, 11, 12 (see Testing Strategy).
- [ ] AC11 — `ARCHITECTURE.md`, `DECISIONS.md`, `Roadmap.md`, and a new `docs/decisions/025-tasks-as-blocking-methods.md` reflect the framework. The ADR cross-references ADR-007 (Manipulator primitives only), ADR-008 (scheduler — partially implemented), ADR-009 (failure recovery), ADR-016 (sync-default blocking — and the deliberate exception this plan grants), and the JSON-loader ADR ([`022-field-data-json-loader.md`](../decisions/022-field-data-json-loader.md), distinguished from the dual-motor `022-dual-motor-profile.md`). All references to "ADR-022" in this spec disambiguate via filename — the project has a pre-existing collision on 022 that ADR-025 must NOT compound; the ADR file uses the filename, not the bare number.

## Data Model

```cpp
// WRO2026/TasksTypes.h   (new — POD layer; included by both Mission.h and Tasks.h)

#ifndef TASKS_TYPES_H
#define TASKS_TYPES_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include "Config.h"   // Config::MAX_TASKS, Config::MAX_TASK_ARG_BYTES

class TasksClass; // forward — full declaration in Tasks.h

/**
 * @brief Fixed-size POD for task argument storage. Operator-facing call sites
 *        use Mission.task<Args...>(name, args...) which packs typed args via
 *        memcpy into TaskArgs::bytes. Recommended layout: define a
 *        per-task struct <Name>Args and pass a single struct value, so byte
 *        layout matches the body's `reinterpret_cast<const NameArgs*>` unpack.
 *
 * @note 16 bytes (Config::MAX_TASK_ARG_BYTES) covers all foreseeable task
 *       signatures for Phase 5/6 (e.g. struct MosaicPlaceArgs { uint8_t cell;
 *       uint8_t color; uint16_t timeout_ms; } fits easily). Compile-time
 *       enforced by static_assert in Mission.h's task<Args...> template.
 */
union TaskArgs
{
    uint8_t  bytes[Config::MAX_TASK_ARG_BYTES];
    uint8_t  u8 [Config::MAX_TASK_ARG_BYTES / sizeof(uint8_t)];
    uint16_t u16[Config::MAX_TASK_ARG_BYTES / sizeof(uint16_t)];
    uint32_t u32[Config::MAX_TASK_ARG_BYTES / sizeof(uint32_t)];
    int32_t  i32[Config::MAX_TASK_ARG_BYTES / sizeof(int32_t)];
    float    f32[Config::MAX_TASK_ARG_BYTES / sizeof(float)];
};
static_assert(sizeof(TaskArgs) == Config::MAX_TASK_ARG_BYTES, "TaskArgs layout");

/**
 * @brief Pointer to a task body. Bodies are non-static member functions on
 *        TasksClass so they can use the singleton's private helpers
 *        (currently none; future failure-flag access).
 */
using TaskFn = bool (TasksClass::*)(const TaskArgs&);

/**
 * @brief Compile-time registry row. The Tasks.cpp registry array is the
 *        single source of truth for valid task names.
 */
struct TaskRegistryEntry
{
    const char* name;
    TaskFn      fn;
};

#endif // TASKS_TYPES_H
```

```cpp
// WRO2026/Tasks.h   (new — adds the singleton class on top of TasksTypes.h)

#ifndef TASKS_H
#define TASKS_H

#include "TasksTypes.h"

// ... TasksClass declaration follows (see API Contracts) ...

#endif // TASKS_H
```

```cpp
// WRO2026/Config.h additions

namespace Config
{
    // Tasks framework
    static constexpr size_t MAX_TASKS           = 16;   ///< compile-time bound on registry size
    static constexpr size_t MAX_TASK_ARG_BYTES  = 16;   ///< TaskArgs payload size
}
```

```cpp
// WRO2026/Mission.h additions to Step + StepType

enum class StepType : uint8_t
{
    GOTO,
    WAIT,
    PRINT,
    BEEP,
    LED,
    TASK,    ///< new
};

struct Step
{
    StepType    type;
    uint32_t    duration_ms;
    uint16_t    freq_hz;
    uint8_t     goal;
    uint8_t     r;
    uint8_t     g;
    uint8_t     b;
    const char* text;
    Direction   route_direction;
    Direction   arrival_direction;
    // New fields (no other field reuse — keeps existing GOTO/etc. paths intact):
    const char* task_name;    ///< TASK: registry key (string-literal lifetime, same rule as `text`)
    TaskArgs    task_args;    ///< TASK: packed args
};
```

## API Contracts

### `TasksClass` (new — `WRO2026/Tasks.h` + `Tasks.cpp`)

```cpp
class TasksClass
{
public:
    bool begin();
    void update();   ///< Housekeeping only; no FSM state in this plan.

    /**
     * @brief Resolve `name` in the registry and invoke the matching method.
     *        Returns the task body's bool verbatim.
     *
     * @return `true`  — task completed OK; Mission should advance.
     *         `false` — task yielded; Mission should advance with a YIELD log.
     *
     * @note Caller is responsible for resolving unknown names via
     *       findTask(name) before invoking runTask. Calling runTask with an
     *       unregistered name is a precondition violation: the call will
     *       print a Serial error and return false, but the Mission FSM
     *       distinguishes unknown-vs-yield via findTask, NOT via runTask's
     *       return value (see Mission.update() TASK case).
     */
    bool runTask(const char* name, const TaskArgs& args);

    /**
     * @brief Return registry pointer for `name`, or nullptr if not registered.
     *        Called by Mission.update() to fault on unknown-name before
     *        invoking runTask.
     */
    const TaskRegistryEntry* findTask(const char* name) const;

    /**
     * @brief Cooperative abort signal. Set by Mission.abort(). Subsystem
     *        builders that want to honour task-level abort can poll
     *        isAborted() between sub-steps.
     *
     * @note In THIS plan we wire the flag and expose isAborted(); we do NOT
     *       modify Drive / PathTracker / LineFollower to poll it. Real
     *       integration of the abort flag into sync drains is a follow-up.
     */
    void abort();
    bool isAborted() const;

    /**
     * @brief True while runTask() is in-flight (between method entry and
     *        return). Read by MissionClass::update() to short-circuit
     *        re-entrant ticks from inside a blocking task body.
     */
    bool inTask() const;

private:
    // Task bodies. One static-storage row per task in `_registry`.
    bool demoEcho(const TaskArgs& args);

    // Registry — defined in Tasks.cpp; bounded by Config::MAX_TASKS.
    static const TaskRegistryEntry _registry[];
    static const size_t            _registry_count;

    bool _inTask   = false;
    bool _aborted  = false;
};

extern TasksClass Tasks;
```

```cpp
// Tasks.cpp registry definition

const TaskRegistryEntry TasksClass::_registry[] =
{
    { "demoEcho", &TasksClass::demoEcho },
    // Future entries added here as task bodies land.
};

const size_t TasksClass::_registry_count =
    sizeof(_registry) / sizeof(_registry[0]);

static_assert(sizeof(TasksClass::_registry) / sizeof(TaskRegistryEntry)
              <= Config::MAX_TASKS, "Registry exceeds MAX_TASKS");
```

### `MissionClass` (extension — `WRO2026/Mission.h`)

```cpp
public:
    /**
     * @brief Enqueue a TASK step. Resolves the name lazily at execute time
     *        (mirrors goTo's deferred-resolution behaviour). Args are packed
     *        into Step::task_args via memcpy at the call site, typed at
     *        compile time by the variadic template.
     *
     * @tparam Args   The caller's argument pack. Total size must be
     *                <= Config::MAX_TASK_ARG_BYTES (static_assert).
     * @param  name   Task name; must outlive the enqueued Step (string
     *                literal is the intended idiom).
     * @param  args   Pack of typed arguments. Order and types must match
     *                the task body's documented layout (compile-time check
     *                on total size; layout match is a hand-mirrored contract
     *                per task — documented in each task's @note).
     */
    template <typename... Args>
    MissionClass& task(const char* name, Args... args);
```

Header-only definition (template):

```cpp
template <typename... Args>
MissionClass& MissionClass::task(const char* name, Args... args)
{
    constexpr size_t TOTAL =
        (sizeof...(Args) == 0) ? 0 : (sizeof(Args) + ... + 0);
    static_assert(TOTAL <= Config::MAX_TASK_ARG_BYTES,
                  "Mission.task args exceed MAX_TASK_ARG_BYTES");

    // Reject nested-task authoring at the enqueue point. Mission.task()
    // called from inside a running task body would silently grow the
    // queue (the _inTask guard only short-circuits Mission.update, not
    // this template). Print a Serial warning and skip the enqueue so
    // the queue stays stable.
    if (Tasks.inTask())
    {
        Serial.println(F("[Mission] task() called from inside a task body; ignored"));
        return *this;
    }

    Step s{};
    s.type      = StepType::TASK;
    s.task_name = name;
    _packArgs(s.task_args.bytes, args...);
    _push(s);
    return *this;
}

// Helper (header-only): byte-pack a typed argument list into a buffer.
template <typename T, typename... Rest>
inline void _packArgs(uint8_t* dst, const T& head, const Rest&... rest)
{
    memcpy(dst, &head, sizeof(T));
    _packArgs(dst + sizeof(T), rest...);
}
inline void _packArgs(uint8_t* /*dst*/) {}
```

### Mission FSM extension — `Mission.cpp::update()`

Add a `case StepType::TASK:` to the switch in `update()`:

```cpp
case StepType::TASK:
{
    if (_stepState == StepState::ENTER)
    {
        const TaskRegistryEntry* entry =
            Tasks.findTask(s.task_name == nullptr ? "" : s.task_name);
        if (entry == nullptr)
        {
            Serial.print(F("[Mission] unknown task '"));
            Serial.print(s.task_name == nullptr ? "?" : s.task_name);
            Serial.println(F("'"));
            EventTrace.pushEvent("NAV FAULT TASK");
            _enterFaultBeeping(now);
            return;
        }

        // Boundary event before dispatch.
        char buf[EventTraceClass::MAX_LINE_LEN];
        snprintf(buf, sizeof(buf), "TASK START %s", s.task_name);
        EventTrace.pushEvent(buf);

        const bool ok = Tasks.runTask(s.task_name, s.task_args);

        // Subsystem fault during the blocking task body would not have
        // surfaced — the top-of-RUNNING PathTracker.hasFault() check at
        // Mission.cpp:248 short-circuits while _inTask is true. Check
        // here, post-return, so a fault during the task triggers the
        // beep-flash path BEFORE we advance past the step — same
        // semantics as the existing GOTO path which never advances on
        // a faulted leg.
        if (PathTracker.hasFault())
        {
            _emitFault();
            _enterFaultBeeping(now);
            return;
        }

        snprintf(buf, sizeof(buf), ok ? "TASK OK %s" : "TASK YIELD %s",
                 s.task_name);
        EventTrace.pushEvent(buf);
        if (!ok)
        {
            Serial.print(F("[Mission] task '"));
            Serial.print(s.task_name);
            Serial.println(F("' yielded"));
        }
        _advanceStep();
    }
    // No ACTIVE branch and we do NOT set _stepState = ACTIVE — runTask
    // is fake-blocking; the in-flight period is fully covered by
    // Tasks.inTask() short-circuiting Mission.update() at the top.
    // _stepState stays in ENTER; _advanceStep() resets it on the way out.
    break;
}
```

Re-entrancy guard, top of `MissionClass::update()`:

```cpp
void MissionClass::update()
{
    if (Tasks.inTask()) { return; }   // re-entered from inside a blocking task
    // ... existing switch on _state ...
}
```

### `Mission.abort()` extension

```cpp
void MissionClass::abort()
{
    Tasks.abort();
    PathTracker.abort();
    LineFollower.setEnabled(false);
    Drive.stop(true);
    _count = _idx = 0;
    _state = State::IDLE;
    _stepState = StepState::ENTER;
    Serial.println(F("[Mission] abort"));
}
```

## Error Handling

- **Unknown task name** — Mission FSM prints `[Mission] unknown task '<name>'`, pushes `NAV FAULT TASK` to EventTrace, calls `_enterFaultBeeping(now)`. Resets the queue. Operator presses BTN_DOWN to reset (existing fault path). Symmetric with the unknown-node-name path at `Mission.cpp:273-281`.
- **Task body returns `false` (yield)** — Mission pushes `TASK YIELD <name>` to EventTrace, prints `[Mission] task '<name>' yielded`, advances to the next queued step. No fault, no beep.
- **Subsystem fault during a blocking task body** — `PathTracker.hasFault()` is checked at the start of each `MissionClass::update()` RUNNING-state pass; this check runs after the task method returns (it cannot fire while `_inTask` is true because the guard short-circuits). Fault latency is bounded by the duration of the in-flight subsystem builder, same cost as today's `Mission.goTo`. EventTrace pushes `NAV FAULT <mn>` and Mission enters `FAULT_BEEPING`.
- **Mission.abort() while task is running** — `Tasks.abort()` sets `_aborted = true`. Subsystem sync builders that opt into the convention will return `false` to the task body, which propagates `false` upward. In THIS plan, `_aborted` is exposed but not yet consumed by subsystems; existing `PathTracker.abort()` / `LineFollower.setEnabled(false)` / `Drive.stop(true)` still fire from `Mission.abort()`, so the chassis halts even if the task body keeps issuing further calls (each subsequent `PathTracker.goTo()` returns `false` immediately because PathTracker is in `IDLE` post-abort).
- **TaskArgs overflow at call site** — Compile-time `static_assert`. Build fails with a readable message.
- **Mission queue full when enqueueing a task** — Existing `_push()` returns `false` and prints `[Mission] queue full; step dropped`. Operator sees the warning at build-mission time. No change.

## Testing Strategy

**Levels:** Unit + Integration (host-compile C++, mirroring T17's `test_path_tracker_bidirectional.cpp` precedent).

| ID  | Test Case                                                                                                | Type        | Expected Behavior                                                                                                            |
|-----|----------------------------------------------------------------------------------------------------------|-------------|------------------------------------------------------------------------------------------------------------------------------|
| TC1 | `Mission.task<uint8_t, uint8_t>("demoEcho", 7, 42)` packs args                                            | Unit        | `Step::task_args.u8[0] == 7`, `Step::task_args.u8[1] == 42`. Bytes [2..15] are zero — **note the dependence on aggregate-init `Step s{}`** which zero-fills `task_args.bytes` before pack. Comment in the test code marks this. AC3.                                  |
| TC2 | `Mission.task("demoEcho")` with zero args                                                                | Unit        | `Step::task_args.bytes` is all zero; static_assert passes (TOTAL=0). AC3.                                                    |
| TC3 | `Mission.task<uint32_t, uint32_t, uint32_t, uint32_t>("X", 1,2,3,4)` packs exactly 16 bytes               | Unit        | Compiles (boundary case). u32 slots [0..3] = 1,2,3,4. AC9 positive.                                                          |
| TC4 | `Mission.task` with a 17-byte arg pack (`uint8_t[17]` payload)                                            | Compile-fail | Dedicated `test_tasks_dispatch_overflow` target invoked by `ctest -R tasks_overflow_compile_fail` with `WILL_FAIL TRUE`. Target is `EXCLUDE_FROM_ALL`; ctest builds it on demand and expects non-zero exit (the `static_assert` fires). AC9 negative. |
| TC5 | `Tasks.findTask("demoEcho")` returns non-null; `findTask("nope")` returns null                            | Unit        | Pointer equality with `&_registry[0]` for the positive case; nullptr for the negative. AC4.                                  |
| TC6 | `Tasks.runTask("demoEcho", args)` returns `true` and the body's Serial output contains `a=7 b=42`         | Integration | Capture mock `Serial` stream; assert substring. AC2.                                                                          |
| TC7 | Mission FSM with a one-step queue `[task("demoEcho", 7, 42)]` — drain the queue                           | Integration | After update() returns, `_state == IDLE`, `_count == 0`, mock EventTrace contains `TASK START demoEcho` and `TASK OK demoEcho`. Mock Serial buffer ALSO contains `[Tasks] demoEcho a=7 b=42` (proves the body actually ran end-to-end via the FSM path, not just via direct runTask in TC6). AC2. |
| TC8 | Mission FSM with `[task("doesNotExist")]`                                                                | Integration | EventTrace contains `NAV FAULT TASK`; `_state == FAULT_BEEPING`. **Negative-path check:** `demoEcho`'s Serial signature `[Tasks] demoEcho` is NOT in the captured stream (proves the dispatcher didn't mis-resolve). AC4. |
| TC9 | Mission FSM with `[task("alwaysYield")]` (stub body returning `false`)                                   | Integration | EventTrace contains `TASK YIELD alwaysYield`; `_count == 0` after drain; `_state == IDLE`. AC5.                              |
| TC10 | Re-entrancy guard — inline mirror of `Mission.update()` containing `if (Tasks.inTask()) return;` plus an instrumented test fake | Integration | Test task body sets `Tasks._inTask = true` (test fake exposes a setter), then calls the mirrored `update()`. Mirror returns at the guard line; an instrumented counter never increments. AC8. Mirror is annotated `// MIRROR: WRO2026/Mission.cpp:<line>` like the T17 precedent so guard drift in production fails the test. |
| TC11 | Unit test on a tiny `MissionMirror::abort()` (inline replica of the abort sequence) — assert `Tasks.isAborted() == true` after calling it | Unit        | AC7. Mirror's first line is `Tasks.abort();`. Test verifies the flag transition; subsystem honour is OUT OF SCOPE this plan.                                              |
| TC12 | Queue-full path — fill `MissionMirror::_queue` to `Config::MAX_MISSION_STEPS`, then call `Mission.task("demoEcho")` | Unit        | Mock Serial buffer contains `queue full; step dropped`. `_count` is unchanged. AC1 (defensive — ensures the new template path doesn't bypass `_push`). |

**Test infrastructure:**
- New file: `WRO2026/test/test_tasks_dispatch.cpp` — host-compile harness, following the precedent of `WRO2026/test/test_path_tracker_passthrough.cpp` and `WRO2026/test/test_path_tracker_bidirectional.cpp` (T17): production types are referenced from the real headers where it makes sense (`TaskArgs`, `TaskRegistryEntry` via `TasksTypes.h` — small, dependency-free header). Mission's FSM logic is **mirrored inline** with `// MIRROR: WRO2026/Mission.cpp:<line>` annotations rather than linked from production, so the test doesn't drag in PathTracker/Drive/LineFollower/MatrixMiniR4. Mock `Serial`, `EventTraceClass::pushEvent` (ring buffer), `PathTracker.hasFault()` (constant false unless a TC sets the test fake).
- `WRO2026/test/CMakeLists.txt` adds:
  - `test_tasks_dispatch` — normal target, run by `ctest`.
  - `test_tasks_dispatch_overflow` — overflow compile-fail target, declared `EXCLUDE_FROM_ALL EXCLUDE_FROM_DEFAULT_BUILD`. A custom `add_test(NAME tasks_overflow_compile_fail COMMAND ${CMAKE_COMMAND} --build . --target test_tasks_dispatch_overflow)` invokes the build; `set_tests_properties(tasks_overflow_compile_fail PROPERTIES WILL_FAIL TRUE)` expects non-zero exit (the `static_assert` makes the build fail).
- Pytest CI workflow unchanged: `uv run pytest tools/path_planner/tests/` still green (no Python changes here).

**Run command:**
- Python suite: `uv run pytest tools/path_planner/tests/`
- C++ host-compile: `cd WRO2026/test && cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure`
- Arduino verify: `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Define `TaskArgs`, `TaskFn`, `TaskRegistryEntry` in new `TasksTypes.h` POD-layer header | — | low | `WRO2026/TasksTypes.h` (new) | Create file with include guard `TASKS_TYPES_H`. Forward-declare `class TasksClass;`. Declare `union TaskArgs` (16-byte payload, multiple-view union per Data Model). Declare `using TaskFn = bool (TasksClass::*)(const TaskArgs&);`. Declare `struct TaskRegistryEntry { const char* name; TaskFn fn; };`. Doxygen on every type. No implementation. Dependency-free except for `<stdint.h>`, `<stddef.h>`, and `Config.h`. Satisfies the foundation for AC1, AC3, AC9 and supports the Mission.h ↔ Tasks.h decoupling (see Decisions). |
| T2 | Add `MAX_TASKS = 16` and `MAX_TASK_ARG_BYTES = 16` to `Config.h` | — | low | `WRO2026/Config.h` | Two `static constexpr size_t` constants inside `namespace Config`. Place in a new `// Tasks framework` section near the existing `MAX_MISSION_STEPS = 64` declaration (currently at `Config.h`, search for `MAX_MISSION_STEPS`). Satisfies AC9 (compile-time bound source). |
| T3 | Implement `TasksClass` — registry array, `runTask`, `findTask`, `abort`, `isAborted`, `inTask`, `update` (housekeeping), stub `demoEcho` body | T1, T2 | med | `WRO2026/Tasks.h` (new), `WRO2026/Tasks.cpp` (new) | `Tasks.h` includes `TasksTypes.h` only and declares `class TasksClass` + singleton `extern Tasks`. `Tasks.cpp`: file-static `const TaskRegistryEntry _registry[]` with `{ "demoEcho", &TasksClass::demoEcho }` and `_registry_count`. `findTask` is `strcmp`-based linear scan (registry is ≤16 entries). `runTask` sets `_inTask = true`, dispatches via member-fn-ptr, resets `_inTask = false` on return (use RAII guard or try/finally pattern via a flag-cleanup-on-scope-exit struct, since exceptions are off — a simple local helper is enough). `update()` is a no-op for now (event emission lives in Mission's TASK case). `demoEcho` calls `Serial.print/println` with the two unpacked uint8 values and returns true. Add `static_assert(sizeof(_registry)/sizeof(TaskRegistryEntry) <= Config::MAX_TASKS, ...)`. Satisfies AC2, AC7, AC8 (mechanics). |
| T4 | Extend `Mission.h` Step struct + `StepType` enum — add `TASK` enumerator and `task_name` + `task_args` fields | T1 | low | `WRO2026/Mission.h` | Add `TASK` to private `StepType` enum (after `LED`). Add `const char* task_name = nullptr;` and `TaskArgs task_args{};` to the private `struct Step`. `#include "TasksTypes.h"` (NOT `Tasks.h`) for `TaskArgs` — preserves the layered dependency. Note: `Step` and `StepType` stay private to `MissionClass`; the variadic `task<Args...>` template is a member function so private access works at instantiation. Existing `Step s{}` aggregate-init in `goTo/wait/print/beep/led` zero-initializes the new fields for free (the template uses default-member-init plus aggregate `{}`); add a comment on each modified call site. Satisfies AC1 prerequisite. |
| T5 | Add header-only `MissionClass::task<Args...>` variadic template + `_packArgs` helper in `Mission.h` | T4 | med | `WRO2026/Mission.h` | Template definition lives in the header (not the .cpp). `static_assert((sizeof...(Args) == 0) || ((sizeof(Args) + ... + 0) <= Config::MAX_TASK_ARG_BYTES), "Mission.task args exceed MAX_TASK_ARG_BYTES")`. Recursive `_packArgs(uint8_t* dst, const T& head, const Rest&... rest)` + terminator overload. `#include <string.h>` for memcpy. Pack into `s.task_args.bytes`, set `s.task_name = name`, push. Satisfies AC1, AC3, AC9. |
| T6 | Extend `MissionClass::update()` with the `TASK` case + add `Tasks.inTask()` re-entrancy guard at the top of `update()` + post-runTask fault check | T3, T5 | high | `WRO2026/Mission.cpp` | At top of `update()`: `if (Tasks.inTask()) { return; }`. In the `RUNNING` state switch: new `case StepType::TASK:` per the API Contracts section — `findTask` → unknown → fault path; else push `TASK START <name>` EventTrace, call `Tasks.runTask`, **then check `PathTracker.hasFault()` and fault before advancing** (mirrors how GOTO never advances on a faulted leg — see updated API Contracts code), else push `TASK OK` / `TASK YIELD` accordingly and call `_advanceStep()`. Do NOT set `_stepState = ACTIVE` (would be dead code per Notes). Satisfies AC2, AC4, AC5, AC6, AC8. |
| T7 | Wire `Tasks.abort()` into `MissionClass::abort()` | T3, T6 | low | `WRO2026/Mission.cpp` | First line of `MissionClass::abort()`: `Tasks.abort();`. Existing `PathTracker.abort()` / `LineFollower.setEnabled(false)` / `Drive.stop(true)` keep firing. **Serialized after T6** — do not dispatch in the same parallel batch as T6; same-file edit on `Mission.cpp`. Satisfies AC7 (flag is set; subsystem-side honour is documented as a follow-up). |
| T8 | Wire Tasks into `WRO2026.ino` — `Tasks.begin()`, `Tasks.update()`, demo `Mission.task("demoEcho", uint8_t(7), uint8_t(42))` in `buildMission()` | T6, T7 | low | `WRO2026/WRO2026.ino` | Include `Tasks.h`. In `setup()`: `const bool tasksOk = Tasks.begin();` after `const bool missOk = ...`; add `tasksOk` to `_setupFailed` and to the per-module fail-print + EventTrace lines. In `cooperativeTick()`: insert `Tasks.update();` between `Manipulator.update();` and `PathTracker.update();`. In `buildMission()`: add `.task("demoEcho", uint8_t(7), uint8_t(42))` somewhere in the chain so the on-robot smoke test exercises dispatch. Satisfies AC1, AC2. |
| T9 | Host-compile tests — `test_tasks_dispatch.cpp` + CMake targets, covering TC1–TC12 | T6 | med | `WRO2026/test/test_tasks_dispatch.cpp` (new), `WRO2026/test/CMakeLists.txt` | Follow `test_path_tracker_bidirectional.cpp` precedent (the T17 commit added a 659-line host-compile harness that replicates relevant production types inline with `// MIRROR: <path>:<line>` annotations). Same mirror approach for Mission's FSM logic. Two CMake targets: `test_tasks_dispatch` (normal `ctest` run) and `test_tasks_dispatch_overflow` (`EXCLUDE_FROM_ALL EXCLUDE_FROM_DEFAULT_BUILD`, declared as a `ctest` test with `WILL_FAIL TRUE` that builds the failing TU on demand). Mock `Serial` (string buffer), mock `EventTraceClass::pushEvent` (string-vector ring), `PathTracker.hasFault()` returning `false` by default with a test fake setter. Reference production headers ONLY for `TasksTypes.h` (POD layer — cheap to include). Satisfies AC10 and all unit/integration ACs above. |
| T10 | ADR-025 + `DECISIONS.md` index entry + `ARCHITECTURE.md` module map / dependency graph update | T6 | low | `docs/decisions/025-tasks-as-blocking-methods.md` (new), `DECISIONS.md`, `ARCHITECTURE.md` | ADR follows the ~30-60-line template (Rule / Why / How to apply / Trigger to revisit / See also). Status: Implemented. Cross-link ADR-007 (Manipulator primitives only), ADR-008 (scheduler partial), ADR-009 (failure recovery), ADR-016 (sync-default — the ADR-025 "How to apply" section must explicitly call out the sync-drain-inside-tick exception this plan introduces and the bounding `Tasks.inTask()` guard), and the JSON-loader ADR (`docs/decisions/022-field-data-json-loader.md`) — explicitly contrasted: JSON works for graph data but is rejected for enum-rich task bodies. **Disambiguate 022 by filename in every cross-reference** — the project already has both `022-dual-motor-profile.md` and `022-field-data-json-loader.md`; ADR-025 must not compound the ambiguity. `DECISIONS.md`: add row to Mission-layer section + bump status table. `ARCHITECTURE.md`: add `Tasks` row to the Module responsibilities table; insert `Tasks` into the dependency graph ASCII diagram between `Manipulator` and `PathTracker`, and update the cooperativeTick order line (which currently omits `LineArray.update();` from its prose — fix that drift too while editing the file). Satisfies AC11. |
| T11 | `Roadmap.md` — rewrite the P5.4 cell to reflect the blocking-method framework shape | T6 | low | `Roadmap.md` | Roadmap line 189 currently says `P5.4 | Tasks.h/.cpp framework (start/update/isDone/abort per scoring action)` — that surface description is **superseded** by this plan. Rewrite the cell to: `Tasks framework (TasksClass singleton; blocking-method bodies registered by name; Mission.task() verb) — see ADR-025`. Mark P5.4 status as shipped via this plan. No phase-table column changes. Satisfies AC11. |

**Parallelism:** T1, T2 can start in parallel. T4 needs T1. T3 needs T1+T2. T5 needs T4. T6 needs T3+T5 and is the high-risk merge point. T7 runs strictly **after** T6 completes (both edit `Mission.cpp` — never dispatch T6 and T7 concurrently). Once T6 is done, T7 and T9 may run in parallel with each other (T7 edits `Mission.cpp`, T9 edits `WRO2026/test/` only — disjoint files). T8, T10, T11 follow T6/T7.

## Notes for Implementer

- **Test scaffolding precedent.** Read `WRO2026/test/test_path_tracker_bidirectional.cpp` first — that file is the T17 commit's host-compile harness. The pattern: replicate the production types inline with `// MIRROR: <header_path>:<line>` annotations so renames in the real headers fail the build. Same approach here for the small `TaskArgs` / `TaskRegistryEntry` / `Step` types.
- **No `std::tuple`, no `std::initializer_list`, no `std::array` in the framework.** Arduino's libstdc++ is partial; the registry is a C-array of POD rows. The variadic template uses a fold expression for the size check (`(sizeof(Args) + ...)`) — fold expressions are C++17 and the Arduino UNO R4 toolchain supports them (already used across the codebase).
- **The `_packArgs` helper must `memcpy` not `*reinterpret_cast<T*>(dst) = head`** — alignment guarantees on R4 are weak, and a `uint8_t*` cast to `uint32_t*` can crash on misaligned arg packs. `memcpy` is the only safe path.
- **`_inTask` is set/cleared in `TasksClass::runTask`**, NOT in the task body itself. Don't ask task authors to manage this — frame ownership is in the framework.
- **The Mission FSM TASK case must NOT introduce a separate ACTIVE state.** Because `runTask` is fake-blocking, ENTER is the only state we'll ever observe externally — the in-flight period is entirely covered by `Tasks.inTask() == true` short-circuiting Mission.update(). Adding ACTIVE here would be dead code that confuses readers.
- **Don't `#include "Mission.h"` from `Tasks.h` or `Tasks.cpp`.** The call edge is `Mission → Tasks` only. If you find yourself wanting a `Mission.` call inside a task body, that's a smell — push the work back up to the operator-authored `buildMission()` instead.
- **The `Step` struct gains 20 bytes** (`const char* task_name` = 4 bytes + `TaskArgs task_args` = 16 bytes on the R4's 32-bit ARM ABI; alignment is already 4-byte so no extra padding). Confirmed by computing `sizeof(Step)` before/after in T6 and asserting via `static_assert` in a comment near the Step declaration. `MissionClass::_queue[Config::MAX_MISSION_STEPS]` (currently `MAX_MISSION_STEPS = 64`, declared explicitly in `WRO2026/Config.h`) grows by exactly `64 × 20 = 1280` bytes in `.bss`. Fits comfortably in 32 KB SRAM alongside PathTracker's queue, Drive's queue, EventTrace's 4×20 ring, LineArray calibration tables, etc. If `MAX_MISSION_STEPS` is reconfigured higher, recompute before merging.
- **Rollback plan.** If the framework misbehaves on hardware, revert these commits and the existing PR3 Mission DSL is unaffected — the new `task()` template is additive and the TASK enumerator is the last value in `StepType`, so existing GOTO/WAIT/PRINT/BEEP/LED step handling is bit-for-bit identical. The `_inTask` guard is the only existing-path edit; reverting it removes the guard but leaves PR3 behaviour.
- **What's deliberately left to a follow-up:**
  1. Subsystem honour of `Tasks.isAborted()` — modify `Drive._drainBlocking`, `PathTracker.update()`, `LineFollower.update()` to poll the flag and bail out fast. Wire up after this plan lands.
  2. First real task body — `toolDelivery` (Roadmap P5.5), waiting on real Manipulator.
  3. Sim parity — Python-side `TasksSim` mirroring the registry; ties into issue #33 CTPS scoping.
  4. Per-task estimated-duration / expected-score metadata for the ADR-008 greedy scheduler — add columns to `TaskRegistryEntry` when the scheduler lands.
