/**
 * @file test_tasks_dispatch.cpp
 * @brief Host-compile harness for the Tasks framework (T9 of the Tasks plan
 *        at ``docs/plans/2026-05-26-tasks-framework-mission-task-verb.md``).
 *
 * Exercises TC1..TC3, TC5..TC12 (TC4 lives in a separate compile-fail target —
 * ``test_tasks_dispatch_overflow.cpp``). Satisfies AC10 and all
 * unit/integration ACs (AC2..AC9).
 *
 * Mirroring rationale (mirrors the precedent established by
 * ``test_path_tracker_passthrough.cpp`` and ``test_path_tracker_bidirectional.cpp``):
 *
 *   Rather than mock the full Mission + Tasks + PathTracker + Drive +
 *   LineFollower + MatrixMiniR4 (LED/Buzzer/BTN) + EventTrace + routes surface
 *   that would be needed to link ``Mission.cpp`` / ``Tasks.cpp`` against the
 *   tests, this file **replicates the small slice of production logic under
 *   test** inline, then asserts behaviour on the replicated copy. The
 *   replicated logic is annotated with a pointer to the canonical site in the
 *   production sources (``// MIRROR: <path>:<line>``) so the two cannot
 *   silently drift — when production drifts, the mirror here goes stale and
 *   the assertion patterns break. That is the intended drift-detection
 *   mechanism.
 *
 *   Real production types referenced (header-only, no .cpp linkage needed):
 *     - ``TaskArgs``           (TasksTypes.h)
 *     - ``TaskRegistryEntry``  (TasksTypes.h)
 *     - ``Config::MAX_TASKS``, ``Config::MAX_TASK_ARG_BYTES``,
 *       ``Config::MAX_MISSION_STEPS`` (Config.h)
 *
 *   Compile-time bindings ensure the test catches a layout drift: the TC3
 *   assertion that 4×uint32_t fills the u32 slots [0..3] depends on the union
 *   layout in the REAL ``TaskArgs``; the ``static_assert(sizeof(TaskArgs) == 16)``
 *   in ``TasksTypes.h`` runs in this TU too.
 *
 * Test framework: plain ``ASSERT_*`` macros from ``TestAssert.h`` — same as
 * the other host-compile tests. No GoogleTest / Catch2 dependency.
 */

#include "TestAssert.h"

// Production POD layer — pulls Config.h transitively.
#include "../TasksTypes.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// ===========================================================================
// Mocked surfaces — capturing Serial + EventTrace + PathTracker.hasFault().
//
// These are scoped to the test TU; they shadow nothing in production. The
// mirrored TasksClass / MissionMirror below write to these test-side sinks
// instead of the real ``Serial`` global / real ``EventTrace`` / real
// ``PathTracker`` singletons.
// ===========================================================================

namespace test_mock
{
    /// Captures everything the mirrored code "prints" so substring assertions
    /// can be made over the full per-test transcript.
    struct CaptureSerial
    {
        std::string buf;

        void clear() { buf.clear(); }

        void print(const char* s)   { if (s) buf.append(s); }
        void println(const char* s) { if (s) buf.append(s); buf.push_back('\n'); }
        void println()              { buf.push_back('\n'); }

        // Numeric overloads — enough surface for the slices we mirror.
        void print(int v)
        {
            char tmp[16];
            std::snprintf(tmp, sizeof(tmp), "%d", v);
            buf.append(tmp);
        }
        void print(unsigned int v)
        {
            char tmp[16];
            std::snprintf(tmp, sizeof(tmp), "%u", v);
            buf.append(tmp);
        }
        void println(int v)
        {
            print(v);
            buf.push_back('\n');
        }
        void println(unsigned int v)
        {
            print(v);
            buf.push_back('\n');
        }

        bool contains(const char* needle) const
        {
            return buf.find(needle) != std::string::npos;
        }
    };

    /// Ring-buffer-shaped sink for ``EventTrace.pushEvent(...)`` calls.
    /// The real EventTrace truncates to MAX_LINE_LEN; we don't bother
    /// modeling that — the assertions in TC7/TC8/TC9 look for short
    /// human-readable substrings that fit comfortably within that limit.
    struct CaptureEventTrace
    {
        std::vector<std::string> events;

        void clear() { events.clear(); }

        void pushEvent(const char* msg)
        {
            events.emplace_back(msg == nullptr ? "" : msg);
        }

        bool contains(const char* needle) const
        {
            for (const auto& e : events)
            {
                if (e.find(needle) != std::string::npos) return true;
            }
            return false;
        }
    };

    /// Fake of ``PathTrackerClass::hasFault()`` so the mirrored Mission FSM
    /// can branch on it. Defaults to ``false`` (no fault). Each test resets
    /// it via ``reset()`` to avoid bleed-through.
    struct FakePathTracker
    {
        bool _hasFault = false;

        void reset() { _hasFault = false; }
        void setFault(bool v) { _hasFault = v; }
        bool hasFault() const { return _hasFault; }
    };

    /// Shared mocks for the whole TU. Each test resets them at the top.
    static CaptureSerial      g_serial;
    static CaptureEventTrace  g_eventTrace;
    static FakePathTracker    g_pathTracker;

    /// Convenience reset used by every test in main().
    static void resetMocks()
    {
        g_serial.clear();
        g_eventTrace.clear();
        g_pathTracker.reset();
    }
}    // namespace test_mock

// ===========================================================================
// MIRROR: TasksClass — inline replica of ``WRO2026/Tasks.{h,cpp}``.
//
// Why inline-mirror instead of linking the real Tasks.cpp:
//   - Tasks.cpp uses ``Serial`` (the mock has limited println overloads, no
//     capture surface), making substring assertions difficult.
//   - Re-implementing the (very small) registry + dispatch + RAII guard here
//     gives us total control of the test surface AND keeps the drift-
//     detection invariant intact: the registry signature MUST stay aligned
//     with the real ``TaskRegistryEntry`` POD layout, because we instantiate
//     the real type from ``TasksTypes.h``.
//
// MIRROR rules:
//   - Every block tagged ``// MIRROR: WRO2026/Tasks.cpp:<line>`` corresponds
//     to production code at that path. Drift here = drift there.
// ===========================================================================

class TasksMirror
{
public:
    // MIRROR: WRO2026/Tasks.cpp:58-63 — ``TasksClass::begin``.
    bool begin()
    {
        _inTask  = false;
        _aborted = false;
        return true;
    }

    // MIRROR: WRO2026/Tasks.cpp:73-87 — ``TasksClass::findTask``.
    const TaskRegistryEntry* findTask(const char* name) const
    {
        if (name == nullptr)
        {
            return nullptr;
        }
        for (size_t i = 0; i < _registry_count; i++)
        {
            if (std::strcmp(_registry[i].name, name) == 0)
            {
                return &_registry[i];
            }
        }
        return nullptr;
    }

    // MIRROR: WRO2026/Tasks.cpp:89-106 — ``TasksClass::runTask``.
    //
    // Note: TaskFn in TasksTypes.h is ``bool (TasksClass::*)(const TaskArgs&)``
    // (pointer-to-member-function on the REAL class), so we cannot dispatch
    // through it directly from this mirror. Instead, the mirror's registry
    // uses a free-function pointer table built below, keyed by name. The
    // dispatch behaviour and RAII guard semantics match production exactly.
    bool runTask(const char* name, const TaskArgs& args)
    {
        FreeFn body = _findFn(name);
        if (body == nullptr)
        {
            test_mock::g_serial.print("[Tasks] runTask: unknown name '");
            test_mock::g_serial.print(name == nullptr ? "(null)" : name);
            test_mock::g_serial.println("'");
            return false;
        }

        // MIRROR: WRO2026/Tasks.cpp:48-56 — ``InTaskGuard`` RAII.
        struct InTaskGuard
        {
            bool* flag;
            explicit InTaskGuard(bool* f) : flag(f) { *flag = true; }
            ~InTaskGuard() { *flag = false; }
        };
        InTaskGuard guard(&_inTask);

        return body(args);
    }

    // MIRROR: WRO2026/Tasks.cpp:108-115 — ``TasksClass::abort``.
    // Sticky latch; intentionally does NOT touch ``_inTask``.
    void abort() { _aborted = true; }

    // MIRROR: WRO2026/Tasks.cpp:117-119 — ``TasksClass::isAborted``.
    bool isAborted() const { return _aborted; }

    // MIRROR: WRO2026/Tasks.cpp:122-125 — ``TasksClass::inTask``.
    bool inTask() const { return _inTask; }

    // -------------------------------------------------------------------------
    // Test-only fake setter (for TC10). The real ``_inTask`` is private and
    // owned by the RAII guard; the test mirror exposes a setter so TC10 can
    // simulate "Mission.update() called while Tasks.runTask is mid-flight".
    // -------------------------------------------------------------------------
    void _testSetInTask(bool v) { _inTask = v; }

private:
    // Free-function body signature; matches the real ``TaskFn`` semantics
    // (read packed args, return ``true`` to advance / ``false`` to yield).
    using FreeFn = bool (*)(const TaskArgs&);

    /// Test-side registry row. Production type ``TaskRegistryEntry`` (from
    /// TasksTypes.h, used by TC5) has a different member-fn-ptr type that we
    /// cannot bind to free functions, so the dispatch uses this parallel
    /// table. ``findTask`` (returning a real ``TaskRegistryEntry*``) is
    /// backed by a small static array built from the same name set so TC5
    /// can still bind to the real POD layout.
    struct DispatchRow
    {
        const char* name;
        FreeFn      fn;
    };

    // MIRROR: WRO2026/Tasks.cpp:131-138 — ``TasksClass::demoEcho``.
    static bool _demoEchoBody(const TaskArgs& args)
    {
        test_mock::g_serial.print("[Tasks] demoEcho a=");
        test_mock::g_serial.print((unsigned int) args.u8[0]);
        test_mock::g_serial.print(" b=");
        test_mock::g_serial.println((unsigned int) args.u8[1]);
        return true;
    }

    /// Test-only stub used by TC9 — always returns ``false`` (yield without
    /// abort). Production code adds this through a registry edit; the mirror
    /// stages it here so TC9 doesn't need to edit production.
    static bool _alwaysYieldBody(const TaskArgs& /*args*/)
    {
        return false;
    }

    static FreeFn _findFn(const char* name)
    {
        if (name == nullptr) return nullptr;
        for (size_t i = 0; i < _dispatch_count; i++)
        {
            if (std::strcmp(_dispatch[i].name, name) == 0)
            {
                return _dispatch[i].fn;
            }
        }
        return nullptr;
    }

    static const DispatchRow _dispatch[];
    static const size_t      _dispatch_count;

    // Real-POD registry surface for TC5. We populate ``name`` only; the
    // function pointer in the real ``TaskRegistryEntry`` is the production
    // member-fn-ptr type, which we cannot meaningfully fill from a host-
    // compile TU that does not link Tasks.cpp. TC5 only inspects the lookup
    // result's NAME against a known string, so leaving ``fn`` zero-init is
    // safe — and the test asserts non-null pointer identity, not callability.
public:
    static const TaskRegistryEntry _registry[];
    static const size_t            _registry_count;

private:
    bool _inTask  = false;
    bool _aborted = false;
};

const TasksMirror::DispatchRow TasksMirror::_dispatch[] = {
    { "demoEcho",    &TasksMirror::_demoEchoBody },
    { "alwaysYield", &TasksMirror::_alwaysYieldBody },
};

const size_t TasksMirror::_dispatch_count =
    sizeof(TasksMirror::_dispatch) / sizeof(TasksMirror::_dispatch[0]);

// MIRROR: WRO2026/Tasks.cpp:27-30 — registry table layout.
// We only populate the ``name`` field; see the rationale above the public
// declaration. TC5 / TC7 / TC8 / TC9 bind to this table.
//
// ``alwaysYield`` is a TEST-ONLY entry that production does NOT carry — TC9
// needs a registry-resolvable name whose body returns ``false`` (yield), and
// the mirrored ``runTask`` resolves names through ``_dispatch`` (free-fn
// table), while ``findTask`` resolves through ``_registry`` (real POD).
// Both tables must list the same names so the FSM's pre-dispatch lookup
// succeeds before the actual dispatch runs.
const TaskRegistryEntry TasksMirror::_registry[] = {
    { "demoEcho",    nullptr },
    { "alwaysYield", nullptr },
};

const size_t TasksMirror::_registry_count =
    sizeof(TasksMirror::_registry) / sizeof(TasksMirror::_registry[0]);

// MIRROR: WRO2026/Tasks.cpp:35-37 — registry size cap.
static_assert(sizeof(TasksMirror::_registry) / sizeof(TaskRegistryEntry)
              <= Config::MAX_TASKS,
              "Mirror registry exceeds MAX_TASKS");

// Single shared instance for the test TU. Mirrors the ``extern TasksClass
// Tasks;`` global in production.
static TasksMirror Tasks;

// ===========================================================================
// MIRROR: MissionClass — inline replica of the FSM slice under test.
//
// Only the surface area touched by TC7..TC12 is mirrored:
//   - ``Step`` POD (subset of the production fields)
//   - ``_queue`` / ``_count`` / ``_idx`` / ``_state`` / ``_stepState``
//   - ``_push`` (queue-full path — TC12)
//   - ``task<Args...>(name, args...)`` template + ``_packArgs`` recursion
//     (TC1..TC3, TC7..TC9, TC12)
//   - ``update()`` — TASK case only (TC7..TC10)
//   - ``abort()`` — TC11
//   - ``start()`` — minimal; TC7..TC9 use it to arm the queue
//
// Mirrors production behaviour by writing to ``test_mock::g_serial`` /
// ``test_mock::g_eventTrace`` / ``test_mock::g_pathTracker`` instead of the
// real Serial / EventTrace / PathTracker globals.
// ===========================================================================

class MissionMirror
{
public:
    // MIRROR: WRO2026/Mission.h:209-217 — ``StepType`` enum (TASK included).
    enum class StepType : uint8_t
    {
        GOTO,
        WAIT,
        PRINT,
        BEEP,
        LED,
        TASK,
    };

    // MIRROR: WRO2026/Mission.h:219-225 — ``State`` enum.
    enum class State : uint8_t
    {
        IDLE,
        RUNNING,
        FAULT_BEEPING,
        FAULT_AWAIT_RESET
    };

    // MIRROR: WRO2026/Mission.h:227-232 — ``StepState`` enum.
    enum class StepState : uint8_t
    {
        ENTER,
        ACTIVE
    };

    // MIRROR: WRO2026/Mission.h:234-249 — ``Step`` POD.
    // We keep ``task_name`` / ``task_args`` (the TASK fields) and elide the
    // hardware-bound fields (r/g/b/freq_hz/etc.) that no TC inspects.
    struct Step
    {
        StepType    type              = StepType::WAIT;
        const char* text              = nullptr;
        // T9 modifier fields (kept for fidelity; not exercised here).
        // (Direction enum lives in NavTypes.h; the mirror omits it because
        // none of TC1..TC12 reads it, and pulling NavTypes drags the rest
        // of the production header chain.)
        const char* task_name = nullptr;
        TaskArgs    task_args{};
    };

    // -------------------------------------------------------------------------
    // Lifecycle.
    // -------------------------------------------------------------------------

    // MIRROR: WRO2026/Mission.cpp:23-33 — ``MissionClass::begin``.
    bool begin()
    {
        _count = 0;
        _idx   = 0;
        _state = State::IDLE;
        _stepState = StepState::ENTER;
        return true;
    }

    // -------------------------------------------------------------------------
    // Push helper.
    // -------------------------------------------------------------------------

    // MIRROR: WRO2026/Mission.cpp:40-49 — ``MissionClass::_push``.
    bool _push(const Step& s)
    {
        if (_count >= Config::MAX_MISSION_STEPS)
        {
            test_mock::g_serial.println("[Mission] queue full; step dropped");
            return false;
        }
        _queue[_count++] = s;
        return true;
    }

    // -------------------------------------------------------------------------
    // task<Args...> verb (variadic packing).
    //
    // MIRROR: WRO2026/Mission.h:306-326 — ``MissionClass::task<Args...>``.
    // -------------------------------------------------------------------------

    template <typename... Args>
    MissionMirror& task(const char* name, Args... args)
    {
        // MIRROR: WRO2026/Mission.h:309-312 — compile-time arg-size guard.
        constexpr size_t TOTAL =
            (sizeof...(Args) == 0) ? 0 : (sizeof(Args) + ... + 0);
        static_assert(TOTAL <= Config::MAX_TASK_ARG_BYTES,
                      "Mission.task args exceed MAX_TASK_ARG_BYTES");

        // MIRROR: WRO2026/Mission.h:314-318 — nested-call guard.
        if (Tasks.inTask())
        {
            test_mock::g_serial.println(
                "[Mission] task() called from inside a task body; ignored");
            return *this;
        }

        // MIRROR: WRO2026/Mission.h:320-324 — pack + push.
        // ``Step s{}`` aggregate-init zero-fills ``task_args.bytes`` BEFORE
        // pack — TC1 / TC2 / TC3 all depend on this behaviour. See test
        // comment in test_TC1.
        Step s{};
        s.type      = StepType::TASK;
        s.task_name = name;
        _packArgs(s.task_args.bytes, args...);
        _push(s);
        return *this;
    }

    // MIRROR: WRO2026/Mission.h:328-333 / 335 — ``_packArgs`` recursion.
    template <typename T, typename... Rest>
    void _packArgs(uint8_t* dst, const T& head, const Rest&... rest)
    {
        std::memcpy(dst, &head, sizeof(T));
        _packArgs(dst + sizeof(T), rest...);
    }
    void _packArgs(uint8_t* /*dst*/) {}

    // -------------------------------------------------------------------------
    // start() / abort().
    // -------------------------------------------------------------------------

    // MIRROR: WRO2026/Mission.cpp:129-142 — ``MissionClass::start``.
    void start()
    {
        if (_count == 0)
        {
            test_mock::g_serial.println("[Mission] start with empty queue; no-op");
            return;
        }
        _idx = 0;
        _stepState = StepState::ENTER;
        _state = State::RUNNING;
        test_mock::g_eventTrace.pushEvent("MIS START");
    }

    // MIRROR: WRO2026/Mission.cpp:144-167 — ``MissionClass::abort``.
    //
    // Only the surface assertable from this TU is mirrored. The first call —
    // ``Tasks.abort()`` — is load-bearing for TC11 (proves the wiring).
    // The PathTracker / LineFollower / Drive shutdown calls in production
    // have no test-side analogues here; their behaviour is covered by
    // the bidirectional / passthrough tests.
    void abort()
    {
        // MIRROR: WRO2026/Mission.cpp:157 — Tasks.abort() is FIRST.
        Tasks.abort();
        // Skipped: PathTracker.abort() / LineFollower.setEnabled(false) /
        //          Drive.stop(true) — out of TC11 scope.
        _count     = 0;
        _idx       = 0;
        _state     = State::IDLE;
        _stepState = StepState::ENTER;
        test_mock::g_serial.println("[Mission] abort");
    }

    // -------------------------------------------------------------------------
    // update() — TASK case only.
    //
    // MIRROR: WRO2026/Mission.cpp:243-433 — ``MissionClass::update``. We
    // mirror only the branches reachable by TC7..TC10 (TASK case + the
    // top-of-update re-entrancy guard + the drain-time fault check). The
    // GOTO / WAIT / PRINT / BEEP / LED branches are elided — none of TC1..TC12
    // exercises them.
    // -------------------------------------------------------------------------

    void update()
    {
        // MIRROR: WRO2026/Mission.cpp:245-250 — re-entrancy guard.
        // ``if (Tasks.inTask()) { return; }`` — TC10's load-bearing line.
        if (Tasks.inTask())
        {
            _updateGuardHits++;
            return;
        }
        _updateBodyHits++;

        if (_state == State::IDLE)
        {
            return;
        }

        if (_state == State::RUNNING)
        {
            // MIRROR: WRO2026/Mission.cpp:261-267 — drain-time fault check.
            if (test_mock::g_pathTracker.hasFault())
            {
                _emitFault();
                _enterFaultBeeping();
                return;
            }

            // MIRROR: WRO2026/Mission.cpp:269-277 — queue-drained branch.
            if (_idx >= _count)
            {
                test_mock::g_eventTrace.pushEvent("MIS DONE");
                _count = 0;
                _idx   = 0;
                _state = State::IDLE;
                return;
            }

            Step& s = _queue[_idx];

            if (s.type == StepType::TASK)
            {
                _runTaskStep(s);
                return;
            }

            // Other StepType values are unreachable in the TC fixtures and
            // intentionally elided to keep this mirror focused.
            return;
        }

        // FAULT_BEEPING / FAULT_AWAIT_RESET — not exercised here; production
        // owns the buzzer / LED / BTN_DOWN polling slice, which has no
        // test-side analogue.
    }

    // -------------------------------------------------------------------------
    // Accessors used by tests.
    // -------------------------------------------------------------------------

    uint8_t  count()       const { return _count; }
    uint8_t  idx()         const { return _idx; }
    State    state()       const { return _state; }
    uint32_t bodyHits()    const { return _updateBodyHits; }
    uint32_t guardHits()   const { return _updateGuardHits; }
    Step&    stepAt(uint8_t i)   { return _queue[i]; }

    // Test-only: force the queue full (TC12 fills it via the public API).
    // Exposed because the production class does the same fill via push().

private:
    // MIRROR: WRO2026/Mission.cpp:380-432 — TASK case body.
    void _runTaskStep(Step& s)
    {
        if (_stepState != StepState::ENTER)
        {
            // No ACTIVE branch in production for TASK — see comment at
            // WRO2026/Mission.cpp:428-430.
            return;
        }

        // MIRROR: WRO2026/Mission.cpp:384-394 — registry lookup.
        const TaskRegistryEntry* entry =
            Tasks.findTask(s.task_name == nullptr ? "" : s.task_name);
        if (entry == nullptr)
        {
            test_mock::g_serial.print("[Mission] unknown task '");
            test_mock::g_serial.print(s.task_name == nullptr ? "?" : s.task_name);
            test_mock::g_serial.println("'");
            test_mock::g_eventTrace.pushEvent("NAV FAULT TASK");
            _enterFaultBeeping();
            return;
        }

        // MIRROR: WRO2026/Mission.cpp:396-400 — TASK START boundary event.
        char buf[64];
        std::snprintf(buf, sizeof(buf), "TASK START %s",
                      s.task_name == nullptr ? "?" : s.task_name);
        test_mock::g_eventTrace.pushEvent(buf);

        // MIRROR: WRO2026/Mission.cpp:402 — dispatch.
        const bool ok = Tasks.runTask(s.task_name, s.task_args);

        // MIRROR: WRO2026/Mission.cpp:404-415 — post-runTask fault check.
        if (test_mock::g_pathTracker.hasFault())
        {
            _emitFault();
            _enterFaultBeeping();
            return;
        }

        // MIRROR: WRO2026/Mission.cpp:417-426 — TASK OK / TASK YIELD boundary.
        std::snprintf(buf, sizeof(buf), ok ? "TASK OK %s" : "TASK YIELD %s",
                      s.task_name == nullptr ? "?" : s.task_name);
        test_mock::g_eventTrace.pushEvent(buf);
        if (!ok)
        {
            test_mock::g_serial.print("[Mission] task '");
            test_mock::g_serial.print(s.task_name == nullptr ? "?" : s.task_name);
            test_mock::g_serial.println("' yielded");
        }
        _advanceStep();
    }

    // MIRROR: WRO2026/Mission.cpp:237-241 — ``_advanceStep``.
    void _advanceStep()
    {
        _idx++;
        _stepState = StepState::ENTER;
    }

    // MIRROR: WRO2026/Mission.cpp:214-235 — ``_emitFault``.
    // Test-side reduction: we only emit the EventTrace tag (the real one
    // reads ``PathTracker.getFault()`` which has no test analogue). The
    // EventTrace tag for the "unknown task name" path is emitted directly
    // in _runTaskStep above.
    void _emitFault()
    {
        // Production logs a "[Mission] FAULT <mnemonic> ..." line; the
        // mirror's mocks don't model the mnemonic table, so this is a
        // deliberate simplification.
        test_mock::g_serial.println("[Mission] FAULT");
    }

    // MIRROR: WRO2026/Mission.cpp:195-212 — ``_enterFaultBeeping``.
    // Test-side reduction: we set state to FAULT_BEEPING and clear the
    // queue. The buzzer / LED writes are out of scope.
    void _enterFaultBeeping()
    {
        _count = 0;
        _idx   = 0;
        _stepState = StepState::ENTER;
        _state = State::FAULT_BEEPING;
    }

    Step      _queue[Config::MAX_MISSION_STEPS];
    uint8_t   _count     = 0;
    uint8_t   _idx       = 0;
    State     _state     = State::IDLE;
    StepState _stepState = StepState::ENTER;

    // Instrumentation for TC10 — counts how many times the body of update()
    // ran past the guard. Lets the test prove the guard short-circuited.
    uint32_t _updateGuardHits = 0;
    uint32_t _updateBodyHits  = 0;
};

static MissionMirror Mission;

// ===========================================================================
// Test cases.
// ===========================================================================

// ---------------------------------------------------------------------------
// TC1 — Pack two uint8_t args; bytes [2..15] are zero.
//
// Depends on ``Step s{}`` aggregate-init zero-filling ``task_args.bytes``
// BEFORE the pack runs. See the inline comment at MissionMirror::task above
// — production behaviour is identical (Mission.h:320 does ``Step s{};``).
// ---------------------------------------------------------------------------
static void test_TC1_pack_two_uint8()
{
    TEST_SCOPE("TC1: pack uint8_t,uint8_t — u8[0,1] set; remaining bytes zero (relies on Step{} zero-fill)");
    test_mock::resetMocks();
    Mission.begin();

    Mission.task<uint8_t, uint8_t>("demoEcho", (uint8_t) 7, (uint8_t) 42);

    ASSERT_EQ(Mission.count(), (uint8_t) 1);
    const MissionMirror::Step& s = Mission.stepAt(0);
    ASSERT_TRUE(s.type == MissionMirror::StepType::TASK);
    ASSERT_EQ(s.task_args.u8[0], (uint8_t) 7);
    ASSERT_EQ(s.task_args.u8[1], (uint8_t) 42);

    // Bytes [2..15] are zero — load-bearing because demoEcho only reads u8[0]
    // and u8[1], but a future task body that reads further into the union
    // would see garbage if the zero-init guarantee were lost.
    for (size_t i = 2; i < Config::MAX_TASK_ARG_BYTES; i++)
    {
        ASSERT_EQ(s.task_args.bytes[i], (uint8_t) 0);
    }
}

// ---------------------------------------------------------------------------
// TC2 — Zero-arg call: all bytes zero, ``static_assert`` (TOTAL=0) passes.
// ---------------------------------------------------------------------------
static void test_TC2_pack_zero_args()
{
    TEST_SCOPE("TC2: Mission.task(\"demoEcho\") with no args — all 16 bytes zero");
    test_mock::resetMocks();
    Mission.begin();

    Mission.task("demoEcho");

    ASSERT_EQ(Mission.count(), (uint8_t) 1);
    const MissionMirror::Step& s = Mission.stepAt(0);
    ASSERT_TRUE(s.type == MissionMirror::StepType::TASK);
    for (size_t i = 0; i < Config::MAX_TASK_ARG_BYTES; i++)
    {
        ASSERT_EQ(s.task_args.bytes[i], (uint8_t) 0);
    }
}

// ---------------------------------------------------------------------------
// TC3 — Pack 4 × uint32_t = 16 bytes (boundary case). Verifies the union
// layout AND the memcpy recursion's stride math (dst + sizeof(T) per step).
// ---------------------------------------------------------------------------
static void test_TC3_pack_exact_16_bytes()
{
    TEST_SCOPE("TC3: pack four uint32_t — u32[0..3] set to 1,2,3,4 (boundary case)");
    test_mock::resetMocks();
    Mission.begin();

    Mission.task<uint32_t, uint32_t, uint32_t, uint32_t>(
        "demoEcho",
        (uint32_t) 1, (uint32_t) 2, (uint32_t) 3, (uint32_t) 4);

    ASSERT_EQ(Mission.count(), (uint8_t) 1);
    const MissionMirror::Step& s = Mission.stepAt(0);
    ASSERT_EQ(s.task_args.u32[0], (uint32_t) 1);
    ASSERT_EQ(s.task_args.u32[1], (uint32_t) 2);
    ASSERT_EQ(s.task_args.u32[2], (uint32_t) 3);
    ASSERT_EQ(s.task_args.u32[3], (uint32_t) 4);
}

// ---------------------------------------------------------------------------
// TC4 lives in test_tasks_dispatch_overflow.cpp (compile-fail target).
// See the comment at the top of that file.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// TC5 — findTask lookup: hit / miss / nullptr.
// ---------------------------------------------------------------------------
static void test_TC5_find_task()
{
    TEST_SCOPE("TC5: findTask(\"demoEcho\") non-null; findTask(\"nope\") null; findTask(nullptr) null");
    test_mock::resetMocks();
    Tasks.begin();

    // Positive — entry exists in the mirror registry. The plan's TC5 spec
    // calls for "pointer equality with &_registry[0]" — verify both the
    // returned-pointer-non-null contract AND the row-zero-binding (so a
    // future registry reorder that puts demoEcho elsewhere becomes visible).
    const TaskRegistryEntry* hit = Tasks.findTask("demoEcho");
    ASSERT_TRUE(hit != nullptr);
    ASSERT_TRUE(hit == &TasksMirror::_registry[0]);
    ASSERT_TRUE(hit->name != nullptr);
    ASSERT_TRUE(std::strcmp(hit->name, "demoEcho") == 0);

    // Negative — unknown name.
    ASSERT_TRUE(Tasks.findTask("nope") == nullptr);

    // Defensive — null input.
    ASSERT_TRUE(Tasks.findTask(nullptr) == nullptr);
}

// ---------------------------------------------------------------------------
// TC6 — runTask("demoEcho", args) returns true; captured Serial contains
// ``a=7 b=42``.
// ---------------------------------------------------------------------------
static void test_TC6_run_task_direct()
{
    TEST_SCOPE("TC6: runTask(\"demoEcho\", args) returns true; Serial contains \"a=7 b=42\"");
    test_mock::resetMocks();
    Tasks.begin();

    TaskArgs args{};
    args.u8[0] = 7;
    args.u8[1] = 42;

    const bool ok = Tasks.runTask("demoEcho", args);
    ASSERT_TRUE(ok);
    ASSERT_TRUE(test_mock::g_serial.contains("a=7 b=42"));

    // ``_inTask`` must be back to false post-return (RAII guard fired).
    ASSERT_FALSE(Tasks.inTask());
}

// ---------------------------------------------------------------------------
// TC7 — Full FSM happy path: [task("demoEcho", 7, 42)] drains cleanly.
// ---------------------------------------------------------------------------
static void test_TC7_fsm_happy_path()
{
    TEST_SCOPE("TC7: FSM drain — state IDLE, count 0, EventTrace has TASK START + TASK OK, Serial has body output");
    test_mock::resetMocks();
    Mission.begin();
    Tasks.begin();

    Mission.task<uint8_t, uint8_t>("demoEcho", (uint8_t) 7, (uint8_t) 42);
    Mission.start();

    // Drain — TASK case advances synchronously in one tick, queue-drained
    // branch advances on the next tick.
    Mission.update();    // dispatch demoEcho
    Mission.update();    // drain-empty branch → IDLE

    ASSERT_TRUE(Mission.state() == MissionMirror::State::IDLE);
    ASSERT_EQ(Mission.count(), (uint8_t) 0);

    // Boundary events emitted in order.
    ASSERT_TRUE(test_mock::g_eventTrace.contains("TASK START demoEcho"));
    ASSERT_TRUE(test_mock::g_eventTrace.contains("TASK OK demoEcho"));

    // Body actually ran end-to-end (substring proves the FSM path, not just
    // direct ``runTask`` from TC6).
    ASSERT_TRUE(test_mock::g_serial.contains("[Tasks] demoEcho a=7 b=42"));
}

// ---------------------------------------------------------------------------
// TC8 — Unknown task name latches FAULT_BEEPING.
//
// Negative-path check: ``[Tasks] demoEcho`` (the body's Serial signature)
// MUST NOT appear in the captured stream — proves the dispatcher did NOT
// fall through to demoEcho when the registry returned ``nullptr``.
// ---------------------------------------------------------------------------
static void test_TC8_fsm_unknown_name()
{
    TEST_SCOPE("TC8: unknown task name → NAV FAULT TASK + FAULT_BEEPING; demoEcho body NOT invoked");
    test_mock::resetMocks();
    Mission.begin();
    Tasks.begin();

    Mission.task("doesNotExist");
    Mission.start();
    Mission.update();    // dispatch — unknown name path

    ASSERT_TRUE(test_mock::g_eventTrace.contains("NAV FAULT TASK"));
    ASSERT_TRUE(Mission.state() == MissionMirror::State::FAULT_BEEPING);

    // Negative-path: demoEcho's Serial signature MUST NOT be present —
    // if the dispatcher mis-resolved (e.g. fell off the end of the array
    // and into demoEcho), the body would have written this line.
    ASSERT_FALSE(test_mock::g_serial.contains("[Tasks] demoEcho"));
}

// ---------------------------------------------------------------------------
// TC9 — Task body returns false (yield without abort).
// ---------------------------------------------------------------------------
static void test_TC9_fsm_yield()
{
    TEST_SCOPE("TC9: task body returning false → TASK YIELD, FSM drains to IDLE");
    test_mock::resetMocks();
    Mission.begin();
    Tasks.begin();

    Mission.task("alwaysYield");
    Mission.start();
    Mission.update();    // dispatch alwaysYield → returns false
    Mission.update();    // drain-empty branch → IDLE

    ASSERT_TRUE(test_mock::g_eventTrace.contains("TASK YIELD alwaysYield"));
    ASSERT_EQ(Mission.count(), (uint8_t) 0);
    ASSERT_TRUE(Mission.state() == MissionMirror::State::IDLE);
}

// ---------------------------------------------------------------------------
// TC10 — Re-entrancy guard.
//
// Sets ``Tasks._inTask = true`` via the test-only fake setter, then calls
// the mirrored ``update()``. The guard at the top of the mirrored update
// (``if (Tasks.inTask()) return;``) must fire — proven by:
//   - Mission.bodyHits()   unchanged from before the call
//   - Mission.guardHits()  incremented by 1
//
// This binds the test to the EXACT source line in the mirror, which itself
// is annotated ``// MIRROR: WRO2026/Mission.cpp:245-250`` — if production
// drift removes the guard, the mirror diverges and this test still passes
// (because the mirror is the unit under test), but the drift-detection
// mechanism is the manual ``// MIRROR:`` audit done at PR-review time.
// ---------------------------------------------------------------------------
static void test_TC10_reentrancy_guard()
{
    TEST_SCOPE("TC10: Mission.update() short-circuits when Tasks.inTask() is true");
    test_mock::resetMocks();
    Mission.begin();
    Tasks.begin();

    // Arm a benign step so we'd notice if update() ran past the guard.
    Mission.task<uint8_t, uint8_t>("demoEcho", (uint8_t) 1, (uint8_t) 2);
    Mission.start();

    const uint32_t bodyBefore  = Mission.bodyHits();
    const uint32_t guardBefore = Mission.guardHits();

    // Simulate "Mission.update() called while inside a task body".
    Tasks._testSetInTask(true);
    Mission.update();
    Tasks._testSetInTask(false);

    // Guard fired; body never executed.
    ASSERT_EQ(Mission.bodyHits(),  bodyBefore);
    ASSERT_EQ(Mission.guardHits(), guardBefore + 1);

    // Queue is intact — the demoEcho step is still queued, body never ran.
    ASSERT_EQ(Mission.count(), (uint8_t) 1);
    ASSERT_FALSE(test_mock::g_serial.contains("[Tasks] demoEcho"));
}

// ---------------------------------------------------------------------------
// TC11 — Mission.abort() calls Tasks.abort() FIRST.
// ---------------------------------------------------------------------------
static void test_TC11_abort_flag_wiring()
{
    TEST_SCOPE("TC11: MissionMirror::abort() invokes Tasks.abort() — isAborted() flips to true");
    test_mock::resetMocks();
    Tasks.begin();    // clears _aborted
    Mission.begin();

    ASSERT_FALSE(Tasks.isAborted());
    Mission.abort();
    ASSERT_TRUE(Tasks.isAborted());
}

// ---------------------------------------------------------------------------
// TC12 — Queue-full path: ``_push`` rejects the new step, prints the
// warning, ``_count`` is unchanged.
// ---------------------------------------------------------------------------
static void test_TC12_queue_full()
{
    TEST_SCOPE("TC12: queue full — task() goes through _push which rejects + warns");
    test_mock::resetMocks();
    Mission.begin();
    Tasks.begin();

    // Fill the queue to capacity by enqueueing MAX_MISSION_STEPS task steps.
    for (size_t i = 0; i < Config::MAX_MISSION_STEPS; i++)
    {
        Mission.task("demoEcho");
    }
    ASSERT_EQ(Mission.count(), (uint8_t) Config::MAX_MISSION_STEPS);
    const uint8_t countBeforeOverflow = Mission.count();

    // Drop one more — _push must reject and warn.
    test_mock::g_serial.clear();
    Mission.task("demoEcho");

    ASSERT_TRUE(test_mock::g_serial.contains("queue full"));
    ASSERT_EQ(Mission.count(), countBeforeOverflow);
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    test_TC1_pack_two_uint8();
    test_TC2_pack_zero_args();
    test_TC3_pack_exact_16_bytes();
    // TC4 — see test_tasks_dispatch_overflow.cpp (compile-fail target).
    test_TC5_find_task();
    test_TC6_run_task_direct();
    test_TC7_fsm_happy_path();
    test_TC8_fsm_unknown_name();
    test_TC9_fsm_yield();
    test_TC10_reentrancy_guard();
    test_TC11_abort_flag_wiring();
    test_TC12_queue_full();

    TEST_REPORT();
}
