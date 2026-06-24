/**
 * @file test_mock_hal.cpp
 * @brief HAL mock proof: signature-conformance block + live control-surface assertions.
 * @author WRO 2026
 *
 * @details
 * This file serves two purposes:
 *
 * **Part A — Signature-conformance block (`hal_signature_conformance()`).**
 * A dead-code block (`if (false) { ... }`) that is COMPILED but NEVER EXECUTED.
 * Dead-code elimination is post-typecheck, so every call inside is type-checked
 * against the mock surface — a missing or wrong-signature method is a compile
 * error. The calls were authored from GREPPING the three real production files:
 *
 *   - `WRO2026/Drive.cpp`     — patterns `MiniR4\.`
 *   - `WRO2026/Mission.cpp`   — patterns `MiniR4\.`, `EventTrace\.`
 *   - `WRO2026/PathTracker.cpp` — patterns `Drive\.`, `EventTrace\.`
 *
 * The grep-derived call list is captured below so issue #106 can reuse it as the
 * host-link checklist.
 *
 * **Part B — Live control-surface assertions (TC2–TC8).**
 * `TestAssert.h`-based cases that exercise the `mock*` setter/getter API of each
 * facade, proving that set→observe→reset round-trips work correctly.
 *
 * ---------------------------------------------------------------------------
 * GREP-DERIVED CALL LIST (reproduced verbatim for #106)
 * ---------------------------------------------------------------------------
 *
 * From Drive.cpp (pattern MiniR4\.):
 *   MiniR4.M2.setReverse(bool)
 *   MiniR4.M3.setReverse(bool)
 *   MiniR4.M2.setPPR_RPM(uint16_t, uint16_t)
 *   MiniR4.M3.setPPR_RPM(uint16_t, uint16_t)
 *   MiniR4.DriveDC.begin(uint8_t, uint8_t, bool, bool)
 *   MiniR4.DriveDC.setMoveSyncPID(float, float, float)
 *   MiniR4.DriveDC.setMoveGyroPID(float, float, float)
 *   MiniR4.DriveDC.setTurnGyroPID(float, float, float)
 *   MiniR4.DriveDC.isPrevTaskDone()
 *   MiniR4.DriveDC.Move(int16_t, int16_t)
 *   MiniR4.DriveDC.MoveSync(int16_t, int16_t)
 *   MiniR4.DriveDC.MoveGyro(int16_t, int16_t)
 *   MiniR4.DriveDC.brake(bool)
 *   MiniR4.DriveDC.MoveDegs(int16_t, int16_t, uint16_t, bool, bool)
 *   MiniR4.DriveDC.MoveSyncDegs(int16_t, int16_t, uint16_t, bool, bool)
 *   MiniR4.DriveDC.MoveGyroDegs(int16_t, int16_t, uint16_t, bool, bool)
 *   MiniR4.DriveDC.MoveTime(int16_t, int16_t, float, bool, bool)
 *   MiniR4.DriveDC.MoveSyncTime(int16_t, int16_t, float, bool, bool)
 *   MiniR4.DriveDC.MoveGyroTime(int16_t, int16_t, float, bool, bool)
 *   MiniR4.DriveDC.resetCounter()
 *   MiniR4.M2.getDegrees()
 *   MiniR4.M3.getDegrees()
 *
 * From Mission.cpp (patterns MiniR4\. and EventTrace\.):
 *   MiniR4.LED.setColor(uint8_t, uint8_t, uint8_t, uint8_t)
 *   MiniR4.Buzzer.Tone(uint16_t, uint32_t)
 *   MiniR4.BTN_DOWN.getState()
 *   EventTrace.pushEvent(const char*)
 *   EventTrace.pushEvent(const char*)   [multiple call sites; single method]
 *
 * From PathTracker.cpp (patterns Drive\. and EventTrace\.):
 *   Drive.stop(bool)
 *   Drive.isIdle()
 *   Drive.hasFault()
 *   Drive.getFault().kind                  [field access — proves FaultInfo::kind present]
 *   Drive.getFault().cmdKind               [field access — proves FaultInfo::cmdKind present]
 *   Drive.clearFault()
 *   Drive.wallStopTimedOut()
 *   Drive.driveDist(float)                 [+ fluent chain .setMode().async() etc.]
 *   Drive.stopAtLine(float)
 *   Drive.turnToYaw(int16_t)
 *   Drive.wallStop(float)
 *   EventTrace.pushEvent(const char*)
 *
 * ---------------------------------------------------------------------------
 * TC mapping
 * ---------------------------------------------------------------------------
 *   TC2  MockDriveDC Move/MoveDegs capture
 *   TC3  MockDriveDC isPrevTaskDone test-set
 *   TC4  MockDC resetCounter zeroes getDegrees (single-call contract)
 *   TC5  Drive fault surface test-set
 *   TC6  Drive builder capture timing
 *   TC7  EventTrace pushEvent ordered capture + truncation + clear
 *   TC8  mockReset restores defaults on all three facades
 */

#include "TestAssert.h"

// The hal_mocks/ headers are force-included by CMake (`add_hal_force_includes`), but
// including them explicitly here makes the file self-documenting and allows the compiler
// to resolve them without the force-include when inspecting the TU in isolation.
#include "MatrixMiniR4.h"
#include "Drive.h"
#include "EventTrace.h"

// ============================================================================
// Part A — Signature-conformance block.
//
// NEVER called — the `if (false)` guard ensures zero runtime execution.
// Compiled unconditionally so every expression is type-checked against the mock
// surface. A removed or misspelled mock method is a compile error here, which is
// exactly the point: this block is the link checklist for issue #106.
// ============================================================================

/**
 * @brief Verify the HAL mock surface matches every call site in the real production files.
 *
 * This function is NEVER invoked. `if (false)` makes its body dead code, but the
 * compiler still performs overload resolution and type-checking on every expression
 * inside (dead-code elimination is a post-typecheck optimisation). A mismatch between
 * the mock signature and a real call expression is therefore a COMPILE ERROR, not a
 * runtime failure.
 *
 * @note Dummy literal arguments are used throughout. The values are irrelevant — only
 *       the argument types matter for signature conformance.
 */
static void hal_signature_conformance()
{
    if (false)
    {
        // ==================================================================
        // --- from Drive.cpp (grep MiniR4\.) ---
        // ==================================================================

        // Motor-channel configuration (Drive::begin calls these before DriveDC.begin).
        MiniR4.M2.setReverse(false);
        MiniR4.M3.setReverse(false);
        MiniR4.M2.setPPR_RPM(0, 0);
        MiniR4.M3.setPPR_RPM(0, 0);

        // DriveDC initialisation (Drive::begin).
        MiniR4.DriveDC.begin(2, 3, false, false);
        MiniR4.DriveDC.setMoveSyncPID(0.0f, 0.0f, 0.0f);
        MiniR4.DriveDC.setMoveGyroPID(0.0f, 0.0f, 0.0f);
        MiniR4.DriveDC.setTurnGyroPID(0.0f, 0.0f, 0.0f);

        // Drain-loop gate (Drive::_drainBlocking polls this each iteration).
        (void) MiniR4.DriveDC.isPrevTaskDone();

        // Continuous open-loop (Drive::setSpeed / Drive::_applyRaw).
        MiniR4.DriveDC.Move(0, 0);

        // Continuous encoder-sync (Drive::setSpeedSync).
        MiniR4.DriveDC.MoveSync(0, 0);

        // Continuous yaw-lock (Drive::forwardGyro).
        MiniR4.DriveDC.MoveGyro(0, 0);

        // Active brake (Drive::stop, Drive::_brakeDC, various FSM abort paths).
        MiniR4.DriveDC.brake(false);

        // Bounded distance move (Drive::_dispatchMoveDegs — RAW/SYNC paths).
        MiniR4.DriveDC.MoveDegs(0, 0, 0, true, false);

        // Bounded sync-distance move (Drive::_dispatchMoveDegs — SYNC path).
        MiniR4.DriveDC.MoveSyncDegs(0, 0, 0, true, false);

        // Bounded gyro-distance move (Drive::_dispatchMoveDegs — GYRO path).
        MiniR4.DriveDC.MoveGyroDegs(0, 0, 0, true, false);

        // Bounded time move (Drive::_dispatchMoveTime — RAW/SYNC paths).
        MiniR4.DriveDC.MoveTime(0, 0, 0.0f, true, false);

        // Bounded sync-time move.
        MiniR4.DriveDC.MoveSyncTime(0, 0, 0.0f, true, false);

        // Bounded gyro-time move.
        MiniR4.DriveDC.MoveGyroTime(0, 0, 0.0f, true, false);

        // Encoder reset (Drive::resetEnc + Drive::_dispatchEdge, etc.).
        MiniR4.DriveDC.resetCounter();

        // Per-wheel encoder reads (Drive::getEncL/R, profile FSM tick, wallStop tick, etc.).
        (void) MiniR4.M2.getDegrees();
        (void) MiniR4.M3.getDegrees();

        // ==================================================================
        // --- from Mission.cpp (grep MiniR4\. and EventTrace\.) ---
        // ==================================================================

        // Status-LED dispatch (Mission::_setLed).
        MiniR4.LED.setColor(static_cast<uint8_t>(1),
                            static_cast<uint8_t>(0),
                            static_cast<uint8_t>(0),
                            static_cast<uint8_t>(0));

        // Beep dispatch (Mission::_beep).
        MiniR4.Buzzer.Tone(static_cast<uint16_t>(1000), static_cast<uint32_t>(150));

        // Operator-reset button poll (Mission::_loop fault-wait and reset path).
        (void) MiniR4.BTN_DOWN.getState();

        // Event-trace push — multiple call sites in Mission (build-error, start, goTo
        // label, step label, done, fault, task, reset).
        EventTrace.pushEvent("x");

        // ==================================================================
        // --- from PathTracker.cpp (grep Drive\. and EventTrace\.) ---
        // ==================================================================

        // Chassis stop (PathTracker abort, _exitCurrentEdge, fault-handler).
        Drive.stop(true);

        // Idle poll (PathTracker dispatch gate: waits until Drive.isIdle() before
        // issuing the next command).
        (void) Drive.isIdle();

        // Fault predicate (PathTracker checks this after every seek/move completion).
        (void) Drive.hasFault();

        // Fault detail — kind field used to discriminate LINE_NOT_FOUND vs other.
        // cmdKind field exists in the struct (proves FaultInfo mirrors production shape
        // for issue #106's real PathTracker.cpp link).
        {
            DriveClass::FaultInfo fi = Drive.getFault();
            (void) fi.kind;
            (void) fi.rc;       // proves FaultInfo::rc present (full 3-field shape for #106)
            (void) fi.cmdKind;
        }

        // Fault clear (PathTracker BEST_EFFORT recovery path).
        Drive.clearFault();

        // Wall-stop timeout advisory (PathTracker WALL_STOP state, issue T14).
        (void) Drive.wallStopTimedOut();

        // Fluent verb — distance move (PathTracker _dispatchEdgeAfterTurn + wall-zero depart).
        Drive.driveDist(10.0f).setMode(DriveMode::GYRO).async();

        // Fluent verb — drive-until-line seek (PathTracker STOP_AT_LINE path).
        Drive.stopAtLine(10.0f).async();

        // Fluent verb — absolute-yaw turn (PathTracker _dispatchTurn).
        Drive.turnToYaw(static_cast<int16_t>(0)).async();

        // Fluent verb — wall push (PathTracker _dispatchWallStop).
        Drive.wallStop(10.0f).async();

        // EventTrace push from PathTracker (wall-stop outcome messages).
        EventTrace.pushEvent("y");
    }
}

// ============================================================================
// Part B — Live control-surface assertions.
// ============================================================================

// ----------------------------------------------------------------------------
// TC2: MockDriveDC Move/MoveDegs capture.
//
// After Move(40,-40), lastMoveL==40 and lastMoveR==-40.
// After MoveDegs(30,-30,200,true,false), the per-field captures reflect each arg.
// ----------------------------------------------------------------------------

static void test_TC2_driveDC_move_capture()
{
    TEST_SCOPE("TC2: MockDriveDC Move/MoveDegs capture");

    MiniR4.DriveDC.mockReset();

    MiniR4.DriveDC.Move(40, -40);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) 40);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, (int16_t) -40);
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, (uint32_t) 1);

    MiniR4.DriveDC.MoveDegs(30, -30, 200, true, false);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveDegsL, (int16_t) 30);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveDegsR, (int16_t) -30);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveDegsDeg, (uint16_t) 200);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveDegsBrake);
    ASSERT_FALSE(MiniR4.DriveDC.lastMoveDegsAsync);
    ASSERT_EQ(MiniR4.DriveDC.moveDegsCallCount, (uint32_t) 1);
}

// ----------------------------------------------------------------------------
// TC3: MockDriveDC isPrevTaskDone test-set.
//
// Default is true. mockSetPrevTaskDone(false) flips it. mockReset() restores true.
// ----------------------------------------------------------------------------

static void test_TC3_driveDC_isPrevTaskDone_testset()
{
    TEST_SCOPE("TC3: MockDriveDC isPrevTaskDone test-set");

    MiniR4.DriveDC.mockReset();

    // Default: task considered done.
    ASSERT_TRUE(MiniR4.DriveDC.isPrevTaskDone());

    // Flip to not-done.
    MiniR4.DriveDC.mockSetPrevTaskDone(false);
    ASSERT_FALSE(MiniR4.DriveDC.isPrevTaskDone());

    // Reset restores default.
    MiniR4.DriveDC.mockReset();
    ASSERT_TRUE(MiniR4.DriveDC.isPrevTaskDone());
}

// ----------------------------------------------------------------------------
// TC4: MockDC resetCounter zeroes getDegrees (single-call contract).
//
// Seed MiniR4.M2 with 500 degrees, call resetCounter(), then getDegrees() must
// return 0. This mirrors the vendor contract: a real encoder reset zeroes the
// degree counter, and the mock honours that observable contract.
// ----------------------------------------------------------------------------

static void test_TC4_mockDC_resetCounter_zeroes_degrees()
{
    TEST_SCOPE("TC4: MockDC resetCounter zeroes getDegrees (single-call contract)");

    MiniR4.M2.mockReset();

    MiniR4.M2.mockSetDegrees(500);
    ASSERT_EQ(MiniR4.M2.getDegrees(), (int32_t) 500);

    MiniR4.M2.resetCounter();
    ASSERT_EQ(MiniR4.M2.getDegrees(), (int32_t) 0);
}

// ----------------------------------------------------------------------------
// TC5: Drive fault surface test-set.
//
// mockSetFault(LINE_NOT_FOUND):
//   - hasFault()            == true
//   - getFault().kind       == LINE_NOT_FOUND
//   - isLineFound()         == false   (fault != isLineFound; separate fields)
// clearFault() clears hasFault and getFault().kind back to NONE.
// ----------------------------------------------------------------------------

static void test_TC5_drive_fault_testset()
{
    TEST_SCOPE("TC5: Drive fault surface test-set");

    Drive.mockReset();

    // No fault by default.
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_EQ(static_cast<int>(Drive.getFault().kind),
              static_cast<int>(DriveClass::Fault::NONE));

    // Inject a fault.
    Drive.mockSetFault(DriveClass::Fault::LINE_NOT_FOUND);
    ASSERT_TRUE(Drive.hasFault());
    ASSERT_EQ(static_cast<int>(Drive.getFault().kind),
              static_cast<int>(DriveClass::Fault::LINE_NOT_FOUND));

    // isLineFound is INDEPENDENT of the fault — it is a separate test-settable flag
    // (the real production code uses getFault().kind, not isLineFound(), for this
    // discrimination). Default after mockReset() is false.
    ASSERT_FALSE(Drive.isLineFound());

    // clearFault() restores the no-fault state.
    Drive.clearFault();
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_EQ(static_cast<int>(Drive.getFault().kind),
              static_cast<int>(DriveClass::Fault::NONE));
    ASSERT_FALSE(Drive.isLineFound());    // unchanged by clearFault
}

// ----------------------------------------------------------------------------
// TC6: Drive builder capture timing.
//
// The verb + key argument are captured AT CONSTRUCTION (when Drive.stopAtLine(30)
// is evaluated), BEFORE any chained setter. The discarded-temporary pattern
// `Drive.stopAtLine(30).setReverse(true).async()` is the real call-site form.
// After this statement:
//   - mockRecord().lastVerb         == STOP_AT_LINE
//   - mockRecord().lastStopAtLineCm == 30.0f
//   - mockRecord().stopAtLineCallCount == 1
// ----------------------------------------------------------------------------

static void test_TC6_builder_capture_timing()
{
    TEST_SCOPE("TC6: Drive builder capture timing");

    Drive.mockReset();

    // Discarded-temporary builder chain — mirrors real PathTracker call sites.
    // The capture (verb + 30.0f) happens at construction of StopAtLineBuilder;
    // .setReverse(true).async() are no-ops that return *this.
    Drive.stopAtLine(30.0f).setReverse(true).async();

    ASSERT_EQ(static_cast<int>(Drive.mockRecord().lastVerb),
              static_cast<int>(DriveVerb::STOP_AT_LINE));
    ASSERT_EQ(Drive.mockRecord().stopAtLineCallCount, (uint32_t) 1);

    // Verify the key arg (30.0f) was captured — compare as integer cm to avoid
    // floating-point representation issues with exact equality.
    ASSERT_EQ((int) Drive.mockRecord().lastStopAtLineCm, 30);
}

// ----------------------------------------------------------------------------
// TC7: EventTrace pushEvent ordered capture + truncation + clear.
//
// pushEvent("A") then pushEvent("B"):
//   - mockCount()     == 2
//   - mockEventAt(0)  == "A"   (oldest = FIFO index 0)
//   - mockEventAt(1)  == "B"
// clear() empties the ring.
// A 25-character string (> MAX_LINE_LEN-1 = 19) is truncated to 19 usable chars.
// ----------------------------------------------------------------------------

static void test_TC7_eventtrace_pushEvent_ordered_capture()
{
    TEST_SCOPE("TC7: EventTrace pushEvent ordered capture + truncation + clear");

    EventTrace.mockReset();

    // Ordered FIFO capture.
    EventTrace.pushEvent("A");
    EventTrace.pushEvent("B");

    ASSERT_EQ(EventTrace.mockCount(), (uint8_t) 2);

    // mockEventAt(0) = oldest (first pushed).
    ASSERT_TRUE(EventTrace.mockEventAt(0)[0] == 'A' && EventTrace.mockEventAt(0)[1] == '\0');
    // mockEventAt(1) = second pushed.
    ASSERT_TRUE(EventTrace.mockEventAt(1)[0] == 'B' && EventTrace.mockEventAt(1)[1] == '\0');

    // clear() empties.
    EventTrace.clear();
    ASSERT_EQ(EventTrace.mockCount(), (uint8_t) 0);

    // A 25-char string is truncated to MAX_LINE_LEN-1 == 19 usable chars + NUL.
    // "ABCDEFGHIJKLMNOPQRSTUVWXY" has 25 chars; after truncation the stored string
    // must be exactly 19 chars long (mockEventAt returns a NUL-terminated buffer of
    // at most MAX_LINE_LEN-1 usable chars).
    const char* long_msg = "ABCDEFGHIJKLMNOPQRSTUVWXY";    // 25 chars
    EventTrace.pushEvent(long_msg);
    ASSERT_EQ(EventTrace.mockCount(), (uint8_t) 1);
    {
        const char* stored = EventTrace.mockEventAt(0);
        // Length must be exactly 19 (MAX_LINE_LEN - 1).
        uint8_t stored_len = 0;
        while (stored[stored_len] != '\0')
        {
            stored_len++;
        }
        ASSERT_EQ(stored_len, (uint8_t) (EventTraceClass::MAX_LINE_LEN - 1));
        // The truncated content must match the first 19 chars of long_msg.
        ASSERT_TRUE(stored_len == 19 && stored[18] == 'S' && stored[0] == 'A');
    }
}

// ----------------------------------------------------------------------------
// TC8: mockReset restores defaults on every facade and Drive.
//
// Exercise each facade's recording fields, then call mockReset() on each and
// verify the test-visible state returns to defaults.
// ----------------------------------------------------------------------------

static void test_TC8_mockReset_clears_all_state()
{
    TEST_SCOPE("TC8: mockReset clears all state on MiniR4, Drive, EventTrace");

    // --- MiniR4 / MockDriveDC ---

    // Dirty the DriveDC state.
    MiniR4.DriveDC.Move(99, -99);
    MiniR4.DriveDC.mockSetPrevTaskDone(false);
    MiniR4.DriveDC.mockSetDegrees(1234);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) 99);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, (int16_t) -99);
    ASSERT_FALSE(MiniR4.DriveDC.isPrevTaskDone());
    ASSERT_EQ(MiniR4.DriveDC.getDegrees(), (int32_t) 1234);

    // Dirty M2 encoder.
    MiniR4.M2.mockSetDegrees(777);
    ASSERT_EQ(MiniR4.M2.getDegrees(), (int32_t) 777);

    // Dirty LED.
    MiniR4.LED.setColor(static_cast<uint8_t>(1),
                        static_cast<uint8_t>(255),
                        static_cast<uint8_t>(0),
                        static_cast<uint8_t>(0));
    ASSERT_EQ(MiniR4.LED.setColorCallCount, (uint32_t) 1);

    // mockReset on the top-level facade clears all children.
    MiniR4.mockReset();

    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, (int16_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, (uint32_t) 0);
    ASSERT_TRUE(MiniR4.DriveDC.isPrevTaskDone());       // default restored
    ASSERT_EQ(MiniR4.DriveDC.getDegrees(), (int32_t) 0);
    ASSERT_EQ(MiniR4.M2.getDegrees(), (int32_t) 0);
    ASSERT_EQ(MiniR4.LED.setColorCallCount, (uint32_t) 0);

    // --- Drive ---

    Drive.mockSetFault(DriveClass::Fault::DRAIN_TIMEOUT);
    Drive.mockSetLineFound(true);
    Drive.mockSetWallStopTimedOut(true);
    Drive.driveDist(55.0f).async();
    ASSERT_TRUE(Drive.hasFault());
    ASSERT_TRUE(Drive.isLineFound());
    ASSERT_TRUE(Drive.wallStopTimedOut());
    ASSERT_EQ(Drive.mockRecord().driveDistCallCount, (uint32_t) 1);

    Drive.mockReset();
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_FALSE(Drive.isLineFound());
    ASSERT_FALSE(Drive.wallStopTimedOut());
    ASSERT_EQ(Drive.mockRecord().driveDistCallCount, (uint32_t) 0);
    ASSERT_EQ(static_cast<int>(Drive.mockRecord().lastVerb),
              static_cast<int>(DriveVerb::NONE));

    // --- EventTrace ---

    EventTrace.mockReset();   // establish a clean ring regardless of prior test cases
    EventTrace.pushEvent("X");
    EventTrace.pushEvent("Y");
    ASSERT_EQ(EventTrace.mockCount(), (uint8_t) 2);

    EventTrace.mockReset();
    ASSERT_EQ(EventTrace.mockCount(), (uint8_t) 0);
}

// ============================================================================
// Entry point.
// ============================================================================

int main()
{
    // Reference the never-run conformance function so it is type-checked and not
    // reported unused, without ever calling it. Taking its address counts as a use
    // on MSVC (no C4505) and g++ (no -Wunused-function); the cast discards the value.
    (void) &hal_signature_conformance;

    test_TC2_driveDC_move_capture();
    test_TC3_driveDC_isPrevTaskDone_testset();
    test_TC4_mockDC_resetCounter_zeroes_degrees();
    test_TC5_drive_fault_testset();
    test_TC6_builder_capture_timing();
    test_TC7_eventtrace_pushEvent_ordered_capture();
    test_TC8_mockReset_clears_all_state();

    TEST_REPORT();
}
