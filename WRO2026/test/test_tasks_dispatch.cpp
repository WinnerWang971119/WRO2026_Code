/**
 * @file test_tasks_dispatch.cpp
 * @brief Host-compile harness for the REAL Tasks framework registry + dispatch
 *        (#46 Phase 1; T5 of the CTPS plan at
 *        ``docs/plans/2026-06-07-ctps-clear-trajectory-prediction-simulator.md``).
 *
 * This target links the production ``Tasks.cpp`` (see ``CMakeLists.txt``) and
 * asserts its behaviour through the capturing ``MockSerial`` (``Serial``) in
 * ``mocks/Arduino.h``. The earlier inline ``TasksMirror`` / ``MissionMirror`` /
 * in-file ``CaptureSerial`` (the deferral documented as PR #35 finding #78) are
 * gone — the deliverable is REAL dispatch + REAL Serial capture.
 *
 * Scope — **Tasks-owned behaviour only** (TC10 of the CTPS plan; #46 Phase 1):
 *   - ``findTask``: hit (``demoEcho`` / ``debugYield``), miss, ``nullptr`` input.
 *   - ``runTask``: dispatch + ``MockSerial`` capture (``demoEcho`` body output),
 *     the unknown-name path, and the ``debugYield`` yield path.
 *   - the ``inTask()`` RAII invariant (false before AND after every ``runTask``).
 *
 * Out of scope for this target: Mission's queue FSM and its abort ordering.
 * Those are covered by ``test_mission_dispatch`` (#46 Phase 2), which links the
 * real ``Mission.cpp`` against the ``hal_mocks/`` HAL + ``mission_mocks/`` layer.
 *
 * Two production branches in ``TasksClass::runTask`` are intentionally NOT
 * asserted here because they are UNREACHABLE against the real, fixed registry
 * (see the comment block above ``main`` for detail):
 *   - the **reentrancy-rejection** path (``_inTask`` already true) — no
 *     registered task re-calls ``runTask``; ``_inTask`` is owned exclusively by
 *     the private ``InTaskGuard``, with no public setter. Triggering it needs a
 *     reentrant task body / Mission.cpp (#46 Phase 2).
 *   - the **null-fn defensive** path (``entry->fn == nullptr``) — every real
 *     ``_registry`` row has a non-null member-fn pointer.
 *
 * Test framework: plain ``ASSERT_*`` macros from ``TestAssert.h`` — same as the
 * other host-compile tests. No GoogleTest / Catch2 dependency. A ``0`` exit
 * (from ``TEST_REPORT()``) means pass.
 */

#include "TestAssert.h"

// Production headers — the REAL Tasks singleton + the POD layer it dispatches
// over. ``Tasks.h`` pulls in the mock ``Arduino.h`` (capturing ``MockSerial``)
// and ``TasksTypes.h`` (``TaskArgs`` / ``TaskRegistryEntry``) transitively.
#include "../Tasks.h"
#include "../TasksTypes.h"

#include <cstring>

static uint32_t g_idleWaitTickCalls = 0;

static void idleWaitTick()
{
    g_idleWaitTickCalls++;
    mockAdvanceMillis(1);
}

// ===========================================================================
// Test cases — all against the real global ``Tasks`` singleton, asserting via
// the real global ``Serial`` (MockSerial) capture buffer. Each case resets the
// runner state (``Tasks.begin()``) and clears the capture (``Serial.clearCapture()``).
// ===========================================================================

// ---------------------------------------------------------------------------
// findTask lookup: hits resolve with a matching ``.name``; an unknown name and
// a ``nullptr`` name both return ``nullptr``.
// ---------------------------------------------------------------------------
static void test_find_task()
{
    TEST_SCOPE("findTask: demoEcho/debugYield hit with matching name; unknown + nullptr miss");
    Tasks.begin();
    Serial.clearCapture();

    // Positive — both registered tasks resolve, and the returned row's name
    // matches the queried name. (Asserting the name, not a fixed registry
    // index, keeps the test robust to a future registry reorder.)
    // The ASSERTs here are non-aborting (TestAssert.h), so every deref of a
    // findTask result is short-circuit-guarded on non-null: a miss (e.g. a task
    // renamed/removed out from under this test) then fails CLEANLY rather than
    // dereferencing nullptr and segfaulting the whole binary.
    const TaskRegistryEntry* echo = Tasks.findTask("demoEcho");
    ASSERT_TRUE(echo != nullptr);
    ASSERT_TRUE(echo != nullptr && echo->name != nullptr);
    ASSERT_TRUE(echo != nullptr && echo->name != nullptr && std::strcmp(echo->name, "demoEcho") == 0);

    const TaskRegistryEntry* yield = Tasks.findTask("debugYield");
    ASSERT_TRUE(yield != nullptr);
    ASSERT_TRUE(yield != nullptr && yield->name != nullptr);
    ASSERT_TRUE(yield != nullptr && yield->name != nullptr
                && std::strcmp(yield->name, "debugYield") == 0);

    // Negative — unknown name.
    ASSERT_TRUE(Tasks.findTask("doesNotExist") == nullptr);

    // Defensive — null input must not feed nullptr into strcmp.
    ASSERT_TRUE(Tasks.findTask(nullptr) == nullptr);
}

// ---------------------------------------------------------------------------
// runTask("demoEcho", {7, 42}) → returns true; the body's Serial signature
// (``[Tasks] demoEcho a=7 b=42``) lands in the capture buffer; ``inTask()`` is
// false both before and after (RAII guard cleared on the normal-return path).
// ---------------------------------------------------------------------------
static void test_run_task_demo_echo()
{
    TEST_SCOPE("runTask(demoEcho, {7,42}) → true; capture has \"[Tasks] demoEcho a=7 b=42\"; inTask false");
    Tasks.begin();
    Serial.clearCapture();

    ASSERT_FALSE(Tasks.inTask());

    TaskArgs args{};
    args.u8[0] = 7;
    args.u8[1] = 42;

    const bool ok = Tasks.runTask("demoEcho", args);
    ASSERT_TRUE(ok);
    ASSERT_TRUE(Serial.captured().find("[Tasks] demoEcho a=7 b=42") != std::string::npos);

    // RAII ``InTaskGuard`` clears ``_inTask`` on the way out.
    ASSERT_FALSE(Tasks.inTask());
}

// ---------------------------------------------------------------------------
// runTask(<unknown>) → returns false; the unknown-name diagnostic is captured;
// ``inTask()`` never flips (the guard is armed AFTER the name resolves).
// ---------------------------------------------------------------------------
static void test_run_task_unknown_name()
{
    TEST_SCOPE("runTask(doesNotExist) → false; capture has \"unknown name\"; inTask false");
    Tasks.begin();
    Serial.clearCapture();

    TaskArgs args{};

    const bool ok = Tasks.runTask("doesNotExist", args);
    ASSERT_FALSE(ok);
    ASSERT_TRUE(Serial.captured().find("unknown name") != std::string::npos);

    // No body ran, so demoEcho's signature must be absent.
    ASSERT_TRUE(Serial.captured().find("[Tasks] demoEcho") == std::string::npos);
    ASSERT_FALSE(Tasks.inTask());
}

// ---------------------------------------------------------------------------
// runTask("debugYield") → returns false (the yield path: body returns false
// without aborting); the body's signature (``[Tasks] debugYield``) is captured;
// ``inTask()`` is false after (the guard clears even though the body yielded).
// ---------------------------------------------------------------------------
static void test_run_task_debug_yield()
{
    TEST_SCOPE("runTask(debugYield) → false (yield); capture has \"[Tasks] debugYield\"; inTask false");
    Tasks.begin();
    Serial.clearCapture();

    TaskArgs args{};

    const bool ok = Tasks.runTask("debugYield", args);
    ASSERT_FALSE(ok);
    ASSERT_TRUE(Serial.captured().find("[Tasks] debugYield") != std::string::npos);

    // Yield is NOT an abort — the latch stays clear and the guard still fired.
    ASSERT_FALSE(Tasks.isAborted());
    ASSERT_FALSE(Tasks.inTask());
}

// ---------------------------------------------------------------------------
// runTask("idleWait") → returns true after the requested timeout while the
// cooperative tick callback keeps the mock clock advancing.
// ---------------------------------------------------------------------------
static void test_run_task_idle_wait()
{
    TEST_SCOPE("runTask(idleWait, 5ms) → true; cooperative tick advances mock clock; inTask false");
    Tasks.begin();
    Serial.clearCapture();
    mockResetMillis();
    Drive.mockReset();
    g_idleWaitTickCalls = 0;
    Tasks.setTickCallback(idleWaitTick);

    TaskArgs args{};
    args.u32[0] = 5;

    const bool ok = Tasks.runTask("idleWait", args);
    ASSERT_TRUE(ok);
    ASSERT_TRUE(millis() == 5);
    ASSERT_TRUE(g_idleWaitTickCalls == 5);
    ASSERT_FALSE(Tasks.inTask());
}

// ---------------------------------------------------------------------------
// runTask("idleWait") refuses to start if motion is already active.
// ---------------------------------------------------------------------------
static void test_run_task_idle_wait_rejects_motion()
{
    TEST_SCOPE("runTask(idleWait) while Drive busy → false; warns; no time advance");
    Tasks.begin();
    Serial.clearCapture();
    mockResetMillis();
    Drive.mockReset();
    Drive.mockSetIdle(false);
    g_idleWaitTickCalls = 0;
    Tasks.setTickCallback(idleWaitTick);

    TaskArgs args{};
    args.u32[0] = 5;

    const bool ok = Tasks.runTask("idleWait", args);
    ASSERT_FALSE(ok);
    ASSERT_TRUE(Serial.captured().find("refusing idle wait while motion is active") != std::string::npos);
    ASSERT_TRUE(millis() == 0);
    ASSERT_TRUE(g_idleWaitTickCalls == 0);
    ASSERT_FALSE(Tasks.inTask());
}

// ===========================================================================
// Entry.
//
// UNREACHABLE-against-the-real-registry branches — deliberately NOT asserted:
//
//   1. Reentrancy rejection (``runTask`` while ``_inTask`` is already true).
//      ``_inTask`` is owned exclusively by the private ``InTaskGuard`` inside
//      ``runTask``; there is no public setter, and none of the two registered
//      tasks (``demoEcho`` / ``debugYield``) calls ``runTask`` from its body.
//      So nothing in the FIXED production registry can make ``_inTask`` true on
//      entry to ``runTask``. Exercising this path needs a reentrant task body
//      or Mission.cpp re-entering Tasks mid-dispatch — that is #46 Phase 2.
//      Faking it (e.g. adding a test-only ``_inTask`` setter or a synthetic
//      reentrant registry row) would assert against a fabricated surface, not
//      the real one, so it is intentionally omitted.
//
//   2. Null-fn defensive guard (``entry->fn == nullptr``). Every row in the
//      real ``_registry`` has a non-null member-fn pointer, so ``findTask``
//      never returns a row whose ``fn`` is null. The branch only fires on a
//      hand-edit typo like ``{ "name", nullptr }`` — not reproducible without
//      mutating the production registry, which this test must not do.
//
// Both guards remain valuable production defences; they are simply not
// triggerable through the real registry, and this target asserts only REAL
// dispatch behaviour.
// ===========================================================================

int main()
{
    test_find_task();
    test_run_task_demo_echo();
    test_run_task_unknown_name();
    test_run_task_debug_yield();
    test_run_task_idle_wait();
    test_run_task_idle_wait_rejects_motion();

    TEST_REPORT();
}
