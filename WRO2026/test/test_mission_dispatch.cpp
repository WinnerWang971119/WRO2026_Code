/**
 * @file test_mission_dispatch.cpp
 * @brief Comprehensive host-test of the REAL `Mission.cpp` queue FSM and its
 *        `TASK`-case dispatch into the REAL `Tasks.cpp` (#46 Phase 2, T3).
 *
 * This target links the production `Mission.cpp` + `Tasks.cpp` (see
 * `CMakeLists.txt`, wired by T4) against the shared `hal_mocks/` HAL (the
 * `MiniR4` facade + Drive + EventTrace) and the `mission_mocks/` upper-facade
 * surfaces (a test-controllable `PathTracker`, plus record/no-op `LineFollower`
 * and `Localization`). The 14-node `fixtures/routes_fixture.cpp` is linked and
 * resolved BY NAME (`FieldMap::findNodeByName`) — never by index, which
 * renumbers across field edits.
 *
 * The deliverable is REAL FSM behaviour asserted through:
 *   - `EventTrace.mockEventAt(i)` / `mockCount()` — ordered FIFO event capture,
 *   - `Serial.captured()` — the capturing `MockSerial`,
 *   - `MiniR4.Buzzer` / `MiniR4.LED` / `MiniR4.BTN_DOWN` records,
 *   - `Drive.mockRecord()` / `Drive.getFault()`,
 *   - the `mission_mocks/` `PathTracker` / `LineFollower` / `Localization` records.
 * Every assertion runs against the production `Mission` / `Tasks` singletons —
 * there is NO `MissionMirror` / `TasksMirror` copy (the Phase-1 inline mirror is
 * gone; this is the whole point of the real-link).
 *
 * Test framework: the repo `TestAssert.h` `ASSERT_*` macros — `main` returns
 * `TEST_REPORT()` (exit 0 = pass). No GoogleTest / Catch2.
 *
 * ---------------------------------------------------------------------------
 * Time advance: ONLY via `mockAdvanceMillis` (no `delay()`).
 * ---------------------------------------------------------------------------
 * `delay()` is inline-defined in `mocks/Arduino.h` and its body odr-uses
 * `Serial1` / `mockTickCount`, which `hal_mocks/hal_mocks.cpp` does NOT define —
 * a single `delay()` anywhere in this TU would fail the link. The FAULT_BEEPING
 * sub-FSM is `millis()`-indexed, so the clock is advanced exclusively with
 * `mockAdvanceMillis` and `millis()` reads the mock clock.
 *
 * ---------------------------------------------------------------------------
 * Genuinely-unreachable Mission.cpp branches — documented, NOT faked:
 * ---------------------------------------------------------------------------
 * Three production branches are intentional coverage gaps here because they can
 * only fire from a task body that the FIXED 8-entry registry (`demoEcho`,
 * `debugYield`, `servoTest`, `home`, `testChassis`, `testOne`, `test1`,
 * `test0621`) does not provide — and faking them via a synthetic registry row
 * would assert against a fabricated surface, not the real one (the same
 * justification as `test_tasks_dispatch.cpp`'s reentrancy / null-fn gaps):
 *
 *   1. The POST-`runTask` PathTracker-fault check (Mission.cpp:547). On host,
 *      `runTask` is fake-blocking and no registry body latches a PathTracker
 *      fault DURING dispatch, so the top-of-RUNNING drain check (Mission.cpp:372)
 *      always fires first — TC11/TC12 cover that reachable branch. The
 *      post-`runTask` check only fires when a fault latches mid-`runTask` while
 *      `Tasks.inTask()` short-circuits `update()`.
 *   2. The task-body `Mission.abort()` bail (`_state != RUNNING` after
 *      `runTask`, Mission.cpp:559). No registry body calls `Mission.abort()`.
 *   3. The `update()`-during-`inTask()` short-circuit (Mission.cpp:360). No
 *      registry body re-enters `Mission.update()` (none of `demoEcho` /
 *      `debugYield` calls a cooperative tick that loops back to Mission).
 *
 * All three require a reentrant / abort-issuing task body. They remain valuable
 * production defences; they are simply not triggerable through the real registry.
 */

#include "Mission.h"        // real — the SUT (../Mission.h via the SKETCH_DIR include path)
#include "Tasks.h"          // real — the TASK-case dispatch target
#include "FieldMap.h"       // real — findNodeByName / NODE_NONE (Mission.h does not pull it)
#include "TestAssert.h"

#include <Arduino.h>        // mock — Serial, millis, mockAdvanceMillis, mockResetMillis
#include <MatrixMiniR4.h>   // mock — MiniR4, mockResetMiniR4
#include "Drive.h"          // mock — Drive
#include "EventTrace.h"     // mock — EventTrace
#include "PathTracker.h"    // mock — PathTracker, PathTrackerClass::Fault
#include "LineFollower.h"   // mock — LineFollower
#include "Localization.h"   // mock — Localization

#include <cstring>          // strcmp
#include <string>           // std::string::npos

// ===========================================================================
// Shared helpers.
// ===========================================================================

/**
 * @brief Per-case reset of the mock clock + every mock surface + the SUT singletons.
 *
 * Order matters: the `begin()` calls below emit Serial chatter, so `Serial`'s
 * capture buffer is cleared LAST (after every begin) so each case starts with a
 * clean capture. `EventTrace.begin()` calls `mockReset()` internally, clearing
 * the capture ring.
 */
static void resetAll()
{
    mockResetMillis();        // g_mockMillis = 0
    mockResetMiniR4();        // resets MiniR4.LED / Buzzer / BTN_DOWN / ... records
    Drive.mockReset();
    EventTrace.begin();       // begin() -> mockReset(): clears the capture ring
    PathTracker.mockReset();
    LineFollower.mockReset();
    Localization.mockReset();
    Tasks.begin();
    Mission.begin();
    Serial.clearCapture();    // clear AFTER the begins so their output does not pollute the case
}

/** @brief `true` iff the captured Serial output contains @p needle. */
static bool serialHas(const char* needle)
{
    return Serial.captured().find(needle) != std::string::npos;
}

/** @brief `true` iff the i-th captured EventTrace event equals @p expected (FIFO, 0 = oldest). */
static bool eventAtEquals(uint8_t i, const char* expected)
{
    return std::strcmp(EventTrace.mockEventAt(i), expected) == 0;
}

/**
 * @brief Index of the first captured event equal to @p needle, or `0xFF` if absent.
 *
 * Lets a case assert relative ordering ("`T> demoEcho` before `T+ demoEcho`")
 * without pinning the absolute slot, which shifts as earlier steps push events.
 */
static uint8_t eventIndexOf(const char* needle)
{
    for (uint8_t i = 0; i < EventTrace.mockCount(); ++i)
    {
        if (std::strcmp(EventTrace.mockEventAt(i), needle) == 0)
        {
            return i;
        }
    }
    return 0xFF;
}

/** @brief `true` iff some captured event equals @p needle. */
static bool eventPresent(const char* needle)
{
    return eventIndexOf(needle) != 0xFF;
}

// Packed-LED constants the assertions compare against (`MockLED::lastRgb`, 0x00RRGGBB).
static constexpr uint32_t RGB_RED = 0xFF0000u;    ///< red(255,0,0)
static constexpr uint32_t RGB_OFF = 0x000000u;    ///< off(0,0,0)

/**
 * @brief Drive a faulted Mission through the FAULT_BEEPING schedule into
 *        FAULT_AWAIT_RESET, leaving BTN_DOWN UNPRESSED.
 *
 * Pre: Mission is in FAULT_BEEPING (one entry beep already emitted by
 * `_enterFaultBeeping`, `toneCallCount == 1`). Advances the clock by one
 * `cycle_ms` (= `MISSION_FAULT_BEEP_MS * 2`) per `update()` so the beep index
 * climbs 1..MISSION_FAULT_BEEPS. After this returns Mission is in
 * FAULT_AWAIT_RESET (it captured `_btnPrev = getState()` == false on entry).
 */
static void runFaultBeepingToAwaitReset()
{
    const uint32_t cycle_ms = Config::MISSION_FAULT_BEEP_MS * 2u;
    // idx climbs 0->1->2->3->4->5; the 5th crossing transitions to AWAIT_RESET.
    for (uint8_t cycle = 1; cycle <= Config::MISSION_FAULT_BEEPS; ++cycle)
    {
        mockAdvanceMillis(cycle_ms);
        Mission.update();
    }
}

// ===========================================================================
// TC1 — begin() resets a stale queue.
// ===========================================================================

static void tc1_begin_resets_stale_queue()
{
    TEST_SCOPE("TC1 begin() resets a stale queue → idle, update() runs nothing");
    resetAll();

    // Enqueue a step, then begin() again — the stale queue must be dropped.
    Mission.print("stale");
    Mission.begin();

    ASSERT_TRUE(Mission.isIdle());

    // An update() on an idle, empty Mission runs nothing: no events, no Serial.
    Serial.clearCapture();
    Mission.update();
    ASSERT_TRUE(Mission.isIdle());
    ASSERT_EQ(EventTrace.mockCount(), (uint8_t) 0);
    ASSERT_TRUE(Serial.captured().empty());
}

// ===========================================================================
// TC2 — start() with an empty queue.
// ===========================================================================

static void tc2_start_empty_queue()
{
    TEST_SCOPE("TC2 start() empty queue → no-op Serial, stays IDLE, no MIS START");
    resetAll();

    Mission.start();

    ASSERT_TRUE(serialHas("start with empty queue; no-op"));
    ASSERT_TRUE(Mission.isIdle());
    ASSERT_FALSE(eventPresent("MIS START"));
    ASSERT_EQ(EventTrace.mockCount(), (uint8_t) 0);
}

// ===========================================================================
// TC3 — start() normal.
// ===========================================================================

static void tc3_start_normal()
{
    TEST_SCOPE("TC3 start() normal → Serial \"start; steps=N\", MIS START event, not idle");
    resetAll();

    Mission.print("a").print("b");
    Mission.start();

    ASSERT_TRUE(serialHas("start; steps=2"));
    ASSERT_GE(EventTrace.mockCount(), (uint8_t) 1);
    ASSERT_TRUE(eventAtEquals(0, "MIS START"));
    ASSERT_FALSE(Mission.isIdle());
}

// ===========================================================================
// TC4 — queue drains in order → done.
// ===========================================================================

static void tc4_queue_drains_in_order()
{
    TEST_SCOPE("TC4 print→print drains in order → MIS DONE event, back to idle");
    resetAll();

    Mission.print("first").print("second");
    Mission.start();

    // PRINT steps advance immediately; each update() runs exactly one step.
    Mission.update();    // "first"
    Mission.update();    // "second"
    Mission.update();    // drain → done

    // Serial holds the two texts in order.
    const std::string& cap = Serial.captured();
    const size_t posFirst  = cap.find("first");
    const size_t posSecond = cap.find("second");
    ASSERT_TRUE(posFirst != std::string::npos);
    ASSERT_TRUE(posSecond != std::string::npos);
    ASSERT_TRUE(posFirst < posSecond);
    ASSERT_TRUE(serialHas("done"));

    // Events: MIS START, "first", "second", MIS DONE — in order.
    ASSERT_TRUE(eventAtEquals(0, "MIS START"));
    ASSERT_TRUE(eventAtEquals(1, "first"));
    ASSERT_TRUE(eventAtEquals(2, "second"));
    ASSERT_TRUE(eventAtEquals(3, "MIS DONE"));

    ASSERT_TRUE(Mission.isIdle());
}

// ===========================================================================
// TC5 — GOTO happy path (holds on not-idle, arrives on idle).
// ===========================================================================

static void tc5_goto_happy_path()
{
    TEST_SCOPE("TC5 GOTO records goTo(goal); holds while busy; ARR + advance when idle");
    resetAll();

    const uint8_t inter1 = FieldMap::findNodeByName("Inter1");
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);

    // Hold PathTracker busy so the GOTO step stays ACTIVE after ENTER.
    PathTracker.mockSetIdle(false);

    Mission.goTo("Inter1");
    Mission.start();

    // ENTER: dispatches goTo(goal) and goes ACTIVE.
    Mission.update();
    ASSERT_EQ(PathTracker.goToCallCount, (uint16_t) 1);
    ASSERT_EQ(PathTracker.lastGoToGoal, inter1);
    ASSERT_TRUE(serialHas("goTo Inter1"));

    // Still busy → step holds, no ARR, no advance.
    Mission.update();
    ASSERT_FALSE(eventPresent("ARR Inter1"));
    ASSERT_FALSE(Mission.isIdle());

    // Let it arrive.
    PathTracker.mockSetIdle(true);
    Mission.update();    // ACTIVE → arrived → advance
    ASSERT_TRUE(serialHas("arrived Inter1"));
    ASSERT_TRUE(eventPresent("ARR Inter1"));

    // Next update drains the now-empty queue → done → idle.
    Mission.update();
    ASSERT_TRUE(Mission.isIdle());
}

// ===========================================================================
// TC6 — GOTO unknown node.
// ===========================================================================

static void tc6_goto_unknown_node()
{
    TEST_SCOPE("TC6 GOTO unknown node → unknown-node Serial, NAV FAULT BAD, FAULT_BEEPING");
    resetAll();

    // A bogus name resolves to NODE_NONE, which fails the GOTO ENTER bounds check.
    Mission.goTo("Nowhere");
    Mission.start();

    Mission.update();    // GOTO ENTER → unknown node → fault

    ASSERT_TRUE(serialHas("unknown node"));
    ASSERT_TRUE(eventPresent("NAV FAULT BAD"));
    ASSERT_TRUE(Mission.hasFault());

    // _enterFaultBeeping braked the chassis once and emitted the entry beep + red LED.
    ASSERT_EQ(Drive.mockRecord().stopBrakeTrueCount, (uint32_t) 1);
    ASSERT_EQ(MiniR4.Buzzer.toneCallCount, (uint32_t) 1);
    ASSERT_GE(MiniR4.LED.setColorCallCount, (uint32_t) 1);
    ASSERT_EQ(MiniR4.LED.lastRgb, RGB_RED);

    // No real goTo was dispatched (the node was bad before dispatch).
    ASSERT_EQ(PathTracker.goToCallCount, (uint16_t) 0);
}

// ===========================================================================
// TC7 — WAIT / PRINT / BEEP / LED.
// ===========================================================================

static void tc7_wait_print_beep_led()
{
    TEST_SCOPE("TC7 WAIT/PRINT/BEEP/LED: wait gates on millis, beep/led record, led timeout clears");
    resetAll();

    // --- WAIT: advances only after the duration elapses. ---
    Mission.wait(500);
    Mission.start();

    Mission.update();    // ENTER: arm the wait (start = now = 0)
    ASSERT_FALSE(Mission.isIdle());

    mockAdvanceMillis(499);
    Mission.update();    // not yet elapsed → holds
    ASSERT_FALSE(Mission.isIdle());

    mockAdvanceMillis(1);    // now = 500 >= 500
    Mission.update();    // advances past WAIT
    Mission.update();    // drain → done → idle
    ASSERT_TRUE(Mission.isIdle());

    // --- PRINT: emits Serial + the text as an event, advances immediately. ---
    resetAll();
    Mission.print("hello");
    Mission.start();
    Mission.update();    // PRINT runs in one tick
    ASSERT_TRUE(serialHas("hello"));
    ASSERT_TRUE(eventPresent("hello"));

    // --- BEEP: records Buzzer.Tone(freq, dur) on ENTER, advances after the duration. ---
    resetAll();
    Mission.beep(1500, 200);
    Mission.start();
    Mission.update();    // ENTER → beep
    ASSERT_EQ(MiniR4.Buzzer.toneCallCount, (uint32_t) 1);
    ASSERT_EQ(MiniR4.Buzzer.lastFrequency, (uint16_t) 1500);
    ASSERT_EQ(MiniR4.Buzzer.lastDuration, (uint32_t) 200);
    ASSERT_FALSE(Mission.isIdle());    // still waiting out the tone duration
    mockAdvanceMillis(200);
    Mission.update();    // advance past BEEP
    Mission.update();    // drain → idle
    ASSERT_TRUE(Mission.isIdle());

    // --- LED with duration 0: sets the color, advances immediately. ---
    resetAll();
    Mission.led(0, 255, 0, 0);
    Mission.start();
    Mission.update();    // ENTER → set LEDs, dur==0 → advance now
    ASSERT_GE(MiniR4.LED.setColorCallCount, (uint32_t) 1);
    ASSERT_EQ(MiniR4.LED.lastRgb, (uint32_t) 0x00FF00u);    // green(0,255,0)
    Mission.update();    // drain → idle
    ASSERT_TRUE(Mission.isIdle());

    // --- LED with non-zero duration: clears to (0,0,0) after the timeout. ---
    resetAll();
    Mission.led(255, 0, 0, 300);
    Mission.start();
    Mission.update();    // ENTER → set red, arm timeout
    ASSERT_EQ(MiniR4.LED.lastRgb, RGB_RED);
    ASSERT_FALSE(Mission.isIdle());
    mockAdvanceMillis(300);
    Mission.update();    // timeout → clear LEDs to off + advance
    ASSERT_EQ(MiniR4.LED.lastRgb, RGB_OFF);
    Mission.update();    // drain → idle
    ASSERT_TRUE(Mission.isIdle());
}

// ===========================================================================
// TC8 — TASK dispatch into the real Tasks.cpp.
// ===========================================================================

static void tc8_task_dispatch_real_body()
{
    TEST_SCOPE("TC8 TASK demoEcho → T> before dispatch, real body Serial, T+ after, advances");
    resetAll();

    // TYPED scalars: a braced {7,42} is a non-deduced context for task<Args...>.
    Mission.task("demoEcho", (uint8_t) 7, (uint8_t) 42);
    Mission.start();

    Mission.update();    // TASK ENTER → T> → runTask(demoEcho) → T+ → advance

    // T> precedes T+ in the capture ring.
    const uint8_t idxStart = eventIndexOf("T> demoEcho");
    const uint8_t idxEnd   = eventIndexOf("T+ demoEcho");
    ASSERT_TRUE(idxStart != 0xFF);
    ASSERT_TRUE(idxEnd != 0xFF);
    ASSERT_TRUE(idxStart < idxEnd);

    // The REAL demoEcho body printed its signature with the typed args.
    ASSERT_TRUE(serialHas("[Tasks] demoEcho a=7 b=42"));

    // Step advanced → next tick drains to done → idle.
    Mission.update();
    ASSERT_TRUE(Mission.isIdle());
}

// ===========================================================================
// TC9 — TASK yield (real debugYield returns false).
// ===========================================================================

static void tc9_task_yield()
{
    TEST_SCOPE("TC9 TASK debugYield → real body false → T- event + Serial yielded, not aborted");
    resetAll();

    Mission.task("debugYield");
    Mission.start();

    Mission.update();    // TASK ENTER → runTask(debugYield) returns false → T-

    ASSERT_TRUE(serialHas("[Tasks] debugYield"));
    ASSERT_TRUE(eventPresent("T- debugYield"));
    ASSERT_TRUE(serialHas("yielded"));
    ASSERT_FALSE(Tasks.isAborted());    // yield is NOT an abort

    // Yield still advances the step.
    Mission.update();
    ASSERT_TRUE(Mission.isIdle());
}

// ===========================================================================
// TC10 — TASK unknown at queue time.
// ===========================================================================

static void tc10_task_unknown_at_queue_time()
{
    TEST_SCOPE("TC10 task(\"nope\") build error → start() refuses → MIS BUILD ERR + FAULT_BEEPING, no motion");
    resetAll();

    // Unknown name at queue time sets the build-error latch + warns on Serial.
    Mission.task("nope");
    ASSERT_TRUE(serialHas("unknown at queue time"));

    Mission.start();    // refuses to arm: build error path

    ASSERT_TRUE(eventPresent("MIS BUILD ERR"));
    ASSERT_TRUE(Mission.hasFault());

    // No chassis motion beyond the FAULT_BEEPING brake: only stop(true) recorded.
    ASSERT_EQ(Drive.mockRecord().driveDistCallCount, (uint32_t) 0);
    ASSERT_EQ(Drive.mockRecord().turnDegCallCount, (uint32_t) 0);
    ASSERT_EQ(Drive.mockRecord().stopBrakeTrueCount, (uint32_t) 1);
    ASSERT_EQ(PathTracker.goToCallCount, (uint16_t) 0);

    // MIS START must NOT have fired (start refused).
    ASSERT_FALSE(eventPresent("MIS START"));
}

// ===========================================================================
// TC11 — PathTracker fault during RUNNING (drain-time, OFN content).
// ===========================================================================

static void tc11_pathtracker_fault_drain_time()
{
    TEST_SCOPE("TC11 OFN fault latched before tick → NAV FAULT OFN + Serial from/to, FAULT_BEEPING, no advance");
    resetAll();

    const uint8_t inter1 = FieldMap::findNodeByName("Inter1");
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);

    Mission.goTo("Inter1");
    Mission.start();

    // Latch an OFF_NODE fault with explicit from/dest BEFORE the first tick.
    PathTracker.mockSetFault(PathTrackerClass::Fault::OFF_NODE, 5, 9);

    Mission.update();    // top-of-RUNNING drain check fires → fault

    ASSERT_TRUE(eventPresent("NAV FAULT OFN"));
    ASSERT_TRUE(serialHas("FAULT OFN from=5 to=9"));
    ASSERT_TRUE(Mission.hasFault());

    // The GOTO step never dispatched (the fault check precedes the step switch).
    ASSERT_EQ(PathTracker.goToCallCount, (uint16_t) 0);
    ASSERT_FALSE(eventPresent("ARR Inter1"));
}

// ===========================================================================
// TC12 — fault appears mid-ACTIVE GOTO.
// ===========================================================================

static void tc12_fault_mid_active_goto()
{
    TEST_SCOPE("TC12 fault latched after GOTO goes ACTIVE → FAULT_BEEPING instead of ARR, never arrives");
    resetAll();

    // Hold busy so the GOTO step reaches ACTIVE without arriving.
    PathTracker.mockSetIdle(false);

    Mission.goTo("Inter1");
    Mission.start();

    Mission.update();    // GOTO ENTER → dispatch → ACTIVE
    ASSERT_EQ(PathTracker.goToCallCount, (uint16_t) 1);
    ASSERT_FALSE(Mission.hasFault());

    // Now latch a fault mid-ACTIVE.
    PathTracker.mockSetFault(PathTrackerClass::Fault::OFF_NODE, 2, 4);

    Mission.update();    // top-of-RUNNING fault check → FAULT_BEEPING (not ARR)

    ASSERT_TRUE(Mission.hasFault());
    ASSERT_TRUE(eventPresent("NAV FAULT OFN"));
    ASSERT_FALSE(eventPresent("ARR Inter1"));    // never reported arrival
}

// ===========================================================================
// TC13 — FAULT_BEEPING schedule.
// ===========================================================================

static void tc13_fault_beeping_schedule()
{
    TEST_SCOPE("TC13 FAULT_BEEPING: exactly MISSION_FAULT_BEEPS tones, then FAULT_AWAIT_RESET + PRESS BTN_DOWN");
    resetAll();

    // Enter the fault FSM via an unknown-node GOTO (deterministic, no PathTracker fault).
    Mission.goTo("Nowhere");
    Mission.start();
    Mission.update();    // → _enterFaultBeeping: first beep (toneCallCount == 1)

    ASSERT_TRUE(Mission.hasFault());
    ASSERT_EQ(MiniR4.Buzzer.toneCallCount, (uint32_t) 1);
    ASSERT_FALSE(eventPresent("PRESS BTN_DOWN"));

    // Advance one cycle (= MISSION_FAULT_BEEP_MS*2) per tick; idx climbs 1..N.
    runFaultBeepingToAwaitReset();

    // Exactly MISSION_FAULT_BEEPS tones total (1 on entry + (N-1) at cycle boundaries).
    ASSERT_EQ(MiniR4.Buzzer.toneCallCount, (uint32_t) Config::MISSION_FAULT_BEEPS);

    // After the last cycle: FAULT_AWAIT_RESET, PRESS BTN_DOWN event, solid red.
    ASSERT_TRUE(eventPresent("PRESS BTN_DOWN"));
    ASSERT_EQ(MiniR4.LED.lastRgb, RGB_RED);
    ASSERT_TRUE(Mission.hasFault());    // still faulted, awaiting the operator ack
}

// ===========================================================================
// TC14 — BTN_DOWN reset.
// ===========================================================================

static void tc14_btn_down_reset()
{
    TEST_SCOPE("TC14 FAULT_AWAIT_RESET + BTN_DOWN rising edge → clearFault, IDLE, fault cleared + MIS RESET");
    resetAll();

    // Drive a PathTracker fault all the way to FAULT_AWAIT_RESET so a fault is
    // still latched when the reset edge fires (clearFault only runs if faulted).
    Mission.goTo("Inter1");
    Mission.start();
    PathTracker.mockSetFault(PathTrackerClass::Fault::OFF_NODE, 1, 2);
    Mission.update();    // drain-time fault → FAULT_BEEPING (entry beep)

    runFaultBeepingToAwaitReset();    // → FAULT_AWAIT_RESET, BTN unpressed (edge baseline = false)
    ASSERT_TRUE(eventPresent("PRESS BTN_DOWN"));
    ASSERT_EQ(PathTracker.clearFaultCallCount, (uint16_t) 0);

    // Operator presses BTN_DOWN → rising edge on the next tick.
    MiniR4.BTN_DOWN.mockSetPressed(true);
    Mission.update();

    ASSERT_EQ(PathTracker.clearFaultCallCount, (uint16_t) 1);
    ASSERT_TRUE(Mission.isIdle());
    ASSERT_FALSE(Mission.hasFault());
    ASSERT_TRUE(serialHas("fault cleared"));
    ASSERT_TRUE(eventPresent("MIS RESET"));
    ASSERT_EQ(MiniR4.LED.lastRgb, RGB_OFF);    // LEDs cleared on reset
}

// ===========================================================================
// TC15 — abort() cancels subsystems.
// ===========================================================================

static void tc15_abort_cancels_subsystems()
{
    TEST_SCOPE("TC15 abort() from RUNNING → Tasks aborted, PathTracker.abort, LineFollower off, Drive stop, IDLE");
    resetAll();

    // Hold a GOTO ACTIVE so abort() cancels a genuinely in-flight mission.
    PathTracker.mockSetIdle(false);
    Mission.goTo("Inter1").print("after");
    Mission.start();
    Mission.update();    // GOTO ENTER → ACTIVE
    ASSERT_FALSE(Mission.isIdle());

    Mission.abort();

    ASSERT_TRUE(Tasks.isAborted());
    ASSERT_EQ(PathTracker.abortCallCount, (uint16_t) 1);
    ASSERT_FALSE(LineFollower.lastEnabled);
    ASSERT_GE(LineFollower.setEnabledCount, 1);
    ASSERT_GE(Drive.mockRecord().stopBrakeTrueCount, (uint32_t) 1);
    ASSERT_TRUE(Mission.isIdle());

    // NOT asserted here: the relative ORDER of the cancel calls (Mission.cpp:211-214
    // runs Tasks.abort() before the chassis-shutdown verbs so a task body polling
    // Tasks.isAborted() sees the latch before motion stops). The mocks record per-verb
    // counts with no shared sequence, and Tasks here is the REAL class with no ordering
    // surface, so call order is unobservable without coupling a mock to the Tasks
    // global. Ordering is also not load-bearing yet — Mission.cpp's abort() notes the
    // flag is wired but subsystems do not poll it (an in-flight body unwinds via
    // PathTracker.abort()/Drive.stop()). The start()→Tasks.clearAbort() pairing is
    // covered separately by TC20.

    // Queue was cleared: a post-abort update() runs nothing and the "after" print never fires.
    Serial.clearCapture();
    Mission.update();
    ASSERT_TRUE(Mission.isIdle());
    ASSERT_FALSE(serialHas("after"));
}

// ===========================================================================
// TC16 — declarePosition success.
// ===========================================================================

static void tc16_declare_position_success()
{
    TEST_SCOPE("TC16 declarePosition(\"Inter1\") idle → Localization records id, Drive.clearFault, DECL event, true");
    resetAll();

    const uint8_t inter1 = FieldMap::findNodeByName("Inter1");
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);

    // PathTracker idle (default), no fault → the guard passes.
    const bool ok = Mission.declarePosition("Inter1");

    ASSERT_TRUE(ok);
    ASSERT_EQ(Localization.lastDeclaredNode, inter1);
    ASSERT_EQ(Localization.declareCount, 1);
    ASSERT_TRUE(eventPresent("DECL Inter1"));
    ASSERT_TRUE(serialHas("declarePosition Inter1"));
    // Drive.clearFault() is unconditionally reached on the success path: with no
    // fault latched it is a no-op, so hasFault() stays false (the reachability of
    // the call is what TC18 confirms via a latched-fault clear).
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================
// TC17 — declarePosition guards.
// ===========================================================================

static void tc17_declare_position_guards()
{
    TEST_SCOPE("TC17 declarePosition guards: busy (no fault) → false busy; unknown name → false unknown");
    resetAll();

    // --- Busy guard: not idle AND no fault → refuse. ---
    PathTracker.mockSetIdle(false);    // busy, fault still NONE
    const bool busyResult = Mission.declarePosition("Inter1");
    ASSERT_FALSE(busyResult);
    ASSERT_TRUE(serialHas("PathTracker busy"));
    ASSERT_EQ(Localization.declareCount, 0);    // never re-anchored

    // --- Unknown-node guard: idle but the name does not resolve → refuse. ---
    resetAll();
    const bool unknownResult = Mission.declarePosition("Nowhere");
    ASSERT_FALSE(unknownResult);
    ASSERT_TRUE(serialHas("unknown node"));
    ASSERT_EQ(Localization.declareCount, 0);    // never re-anchored
}

// ===========================================================================
// TC18 — declarePosition clears latched faults.
// ===========================================================================

static void tc18_declare_position_clears_faults()
{
    TEST_SCOPE("TC18 declarePosition with PathTracker + Drive faults → clears both, returns true");
    resetAll();

    const uint8_t inter1 = FieldMap::findNodeByName("Inter1");
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);

    // Latch BOTH a PathTracker nav fault and a Drive-layer fault.
    PathTracker.mockSetFault(PathTrackerClass::Fault::OFF_NODE, 1, 2);
    Drive.mockSetFault(DriveClass::Fault::DRAIN_TIMEOUT);
    ASSERT_TRUE(PathTracker.hasFault());
    ASSERT_TRUE(Drive.hasFault());

    // Mirror the real PathTracker invariant: isIdle() is `_qCount==0 && _state==IDLE
    // && fault==NONE` (PathTracker.cpp), so it returns FALSE whenever a fault is
    // latched. The mock's idle/fault knobs are independent, so set busy explicitly —
    // otherwise isIdle() keeps its default true and the declarePosition guard
    // `!isIdle() && !hasFault()` short-circuits on the (false) `!isIdle()` term,
    // letting this case pass WITHOUT ever depending on hasFault(). With idle=false the
    // guard's pass genuinely hinges on the fault term, so narrowing the guard to
    // strictly-idle would correctly FAIL TC18 (this is the busy+faulted quadrant).
    PathTracker.mockSetIdle(false);

    const bool ok = Mission.declarePosition("Inter1");

    ASSERT_TRUE(ok);
    ASSERT_EQ(Localization.lastDeclaredNode, inter1);
    ASSERT_EQ(PathTracker.clearFaultCallCount, (uint16_t) 1);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_FALSE(Drive.hasFault());    // Drive.clearFault() reached on the success path
    ASSERT_TRUE(eventPresent("DECL Inter1"));
}

// ===========================================================================
// TC19 — _push queue-overflow rejection (re-establishes the deleted mirror case).
// ===========================================================================

static void tc19_queue_overflow_rejected()
{
    TEST_SCOPE("TC19 _push overflow: fill MAX steps, the next push is dropped with a warning, count unchanged");
    resetAll();

    // Fill the queue to exactly capacity. None of the in-capacity pushes warn.
    for (uint8_t i = 0; i < Config::MAX_MISSION_STEPS; ++i)
    {
        Mission.print("fill");
    }
    ASSERT_FALSE(serialHas("queue full"));    // every in-capacity push was accepted

    // The NEXT push overflows: _push rejects it (Mission.cpp:49-53) and warns.
    Serial.clearCapture();
    Mission.print("OVERFLOW_SENTINEL");
    ASSERT_TRUE(serialHas("queue full; step dropped"));

    // Prove the count is unchanged (still MAX, not MAX+1): arm + drain the whole
    // queue and confirm the dropped step never executed — a wrongly-stored 65th
    // PRINT step would emit its "OVERFLOW_SENTINEL" text on its tick.
    Serial.clearCapture();
    Mission.start();
    for (uint16_t i = 0; i < (uint16_t) Config::MAX_MISSION_STEPS + 2u && !Mission.isIdle(); ++i)
    {
        Mission.update();
    }
    ASSERT_TRUE(Mission.isIdle());                  // all in-capacity steps drained
    ASSERT_FALSE(serialHas("OVERFLOW_SENTINEL"));   // the dropped step was never stored
}

// ===========================================================================
// TC20 — start() clears a stale abort latch (Mission.cpp:189 → Tasks.clearAbort()).
// ===========================================================================

static void tc20_start_clears_abort_latch()
{
    TEST_SCOPE("TC20 start() clears a stale abort latch → Tasks.isAborted() false on the new run");
    resetAll();

    // Run then abort a mission so the framework abort latch is set.
    PathTracker.mockSetIdle(false);
    Mission.goTo("Inter1");
    Mission.start();
    Mission.update();    // GOTO ENTER → ACTIVE
    Mission.abort();     // Mission.abort() → Tasks.abort() sets the latch
    ASSERT_TRUE(Tasks.isAborted());

    // Arm a fresh mission. start() must clear the stale latch (Mission.cpp:189) so a
    // task body polling Tasks.isAborted() in THIS run sees a clean state — an abort
    // from the prior run must not bleed across.
    Mission.print("fresh");
    Mission.start();
    ASSERT_FALSE(Tasks.isAborted());
    ASSERT_FALSE(Mission.isIdle());    // armed → RUNNING
}

// ===========================================================================
// TC21 — setRouteDirection / setArrivalDirection write through to PathTracker.goTo
//        at GOTO execute. (Restores the direction-modifier coverage the old
//        test_path_tracker_bidirectional mirror deleted and wrongly claimed had
//        moved here — #112 triage. Asserted against the REAL Mission this time.)
// ===========================================================================

static void tc21_direction_modifiers_write_through_to_goto()
{
    TEST_SCOPE("TC21 goTo().setRouteDirection(REAR).setArrivalDirection(FRONT) → both forwarded at execute");
    resetAll();

    const uint8_t inter1 = FieldMap::findNodeByName("Inter1");
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);

    // Hold PathTracker busy so the GOTO step dispatches on ENTER and stays ACTIVE — ENTER is
    // where Mission forwards the per-step direction overrides to PathTracker.goTo(...).
    PathTracker.mockSetIdle(false);

    Mission.goTo("Inter1").setRouteDirection(Direction::REAR).setArrivalDirection(Direction::FRONT);
    Mission.start();

    Mission.update();    // GOTO ENTER → goTo(goal).setRouteDirection(REAR).setArrivalDirection(FRONT)

    ASSERT_EQ(PathTracker.goToCallCount, (uint16_t) 1);
    ASSERT_EQ(PathTracker.lastGoToGoal, inter1);
    ASSERT_TRUE(PathTracker.lastRouteDirection == Direction::REAR);
    ASSERT_TRUE(PathTracker.lastArrivalDirection == Direction::FRONT);
}

// ===========================================================================
// TC22 — direction modifiers are no-ops off a GOTO step: chained on a non-GOTO
//        step they warn and must NOT corrupt an earlier GOTO; on an empty queue
//        they warn "did not enqueue" (the _lastPushOk guard, Mission.cpp:124).
// ===========================================================================

static void tc22_direction_modifier_noop_off_goto()
{
    TEST_SCOPE("TC22 modifier off a GOTO → warns + leaves the earlier goTo at AUTO; empty queue warns");
    resetAll();

    const uint8_t inter1 = FieldMap::findNodeByName("Inter1");
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);

    PathTracker.mockSetIdle(false);    // hold the GOTO ACTIVE after ENTER

    // GOTO then PRINT, then chain the modifier on the PRINT: the last step is not a GOTO, so the
    // modifier must no-op (warn) WITHOUT rewriting the earlier GOTO's direction (Mission.cpp:129).
    Mission.goTo("Inter1");
    Mission.print("x").setRouteDirection(Direction::REAR);
    ASSERT_TRUE(serialHas("no preceding goTo step (ignored)"));

    Mission.start();
    Mission.update();    // GOTO ENTER → dispatch the earlier goTo, which stayed at AUTO (uncorrupted)
    ASSERT_EQ(PathTracker.lastGoToGoal, inter1);
    ASSERT_TRUE(PathTracker.lastRouteDirection == Direction::AUTO);

    // Empty-queue modifier: nothing pushed, so the `!_lastPushOk` guard fires first with the
    // "did not enqueue" message (begin() seeds _lastPushOk = false).
    resetAll();
    Mission.setArrivalDirection(Direction::REAR);
    ASSERT_TRUE(serialHas("did not enqueue"));
}

// ===========================================================================
// TC23 — a direction modifier targets the MOST-RECENT goTo only: a second goTo's
//        setRouteDirection must not rewrite the first goTo's direction.
// ===========================================================================

static void tc23_direction_modifier_targets_last_goto_only()
{
    TEST_SCOPE("TC23 two goTos; setRouteDirection after the 2nd → only the 2nd is REAR, the 1st stays AUTO");
    resetAll();

    const uint8_t inter1 = FieldMap::findNodeByName("Inter1");
    const uint8_t inter2 = FieldMap::findNodeByName("Inter2");
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter2 != FieldMap::NODE_NONE);

    PathTracker.mockSetIdle(false);    // hold goto #1 ACTIVE after its ENTER

    Mission.goTo("Inter1");                                       // goto #1 — no modifier (AUTO)
    Mission.goTo("Inter2").setRouteDirection(Direction::REAR);    // goto #2 — REAR
    Mission.start();

    Mission.update();    // goto #1 ENTER → dispatch
    ASSERT_EQ(PathTracker.lastGoToGoal, inter1);
    ASSERT_TRUE(PathTracker.lastRouteDirection == Direction::AUTO);    // #1 untouched by #2's modifier

    // Let goto #1 arrive, then goto #2 dispatches with its own REAR override.
    PathTracker.mockSetIdle(true);
    Mission.update();    // goto #1 ACTIVE → idle → arrived → advance
    Mission.update();    // goto #2 ENTER → dispatch
    ASSERT_EQ(PathTracker.lastGoToGoal, inter2);
    ASSERT_TRUE(PathTracker.lastRouteDirection == Direction::REAR);    // only #2 got REAR
}

// ===========================================================================
// Entry point.
// ===========================================================================

int main()
{
    tc1_begin_resets_stale_queue();
    tc2_start_empty_queue();
    tc3_start_normal();
    tc4_queue_drains_in_order();
    tc5_goto_happy_path();
    tc6_goto_unknown_node();
    tc7_wait_print_beep_led();
    tc8_task_dispatch_real_body();
    tc9_task_yield();
    tc10_task_unknown_at_queue_time();
    tc11_pathtracker_fault_drain_time();
    tc12_fault_mid_active_goto();
    tc13_fault_beeping_schedule();
    tc14_btn_down_reset();
    tc15_abort_cancels_subsystems();
    tc16_declare_position_success();
    tc17_declare_position_guards();
    tc18_declare_position_clears_faults();
    tc19_queue_overflow_rejected();
    tc20_start_clears_abort_latch();
    tc21_direction_modifiers_write_through_to_goto();
    tc22_direction_modifier_noop_off_goto();
    tc23_direction_modifier_targets_last_goto_only();

    TEST_REPORT();
}
