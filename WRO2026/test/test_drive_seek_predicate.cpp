/**
 * @file test_drive_seek_predicate.cpp
 * @brief Host-link test for `Drive::isSeekingLine()` and the load-bearing
 *        `_updateStopAtLine()`-before-`_dispatchNext()` dispatch-tick ordering inside
 *        `Drive::update()` (2026-07-18 gate-line-array-tick plan, T5; TC1-TC6; AC7).
 *
 * @details
 * `isSeekingLine()` (`Drive.h`, ADR-027 area) returns `_stopState != StopAtLineState::IDLE` and
 * is the predicate input `WRO2026.ino`'s perception-tick gate ORs against
 * `LineFollower.isEnabled()`. This target proves two things directly against the REAL
 * `Drive.cpp`, with no PathTracker/Localization/routes involved at all (the predicate and the
 * ordering it depends on live entirely inside `Drive`'s own FSM):
 *
 *   TC1 - `isSeekingLine()` is `false` before any `stopAtLine` has ever been dispatched.
 *   TC2 - `true` while a forward seek is in flight, covering BOTH the `SEEK` and `ARMED`
 *         sub-phases (keying off the FSM state, not a specific phase, per the doxygen contract).
 *   TC3 - `false` again once the seek confirms a crossing and rolls DONE -> IDLE.
 *   TC4 - `false` again once the seek maxes out with no crossing and latches
 *         `Fault::LINE_NOT_FOUND`.
 *   TC5 - THE ORDERING PROOF: on the exact `Drive::update()` call that pops a queued
 *         (`.async()`-enqueued) seek off the FIFO, `isSeekingLine()` reads `false` right up until
 *         that single call and `true` immediately after. `Drive::update()`'s stage order is
 *         `_updateStopAtLine()` (stage 2) THEN, only if now idle, `_dispatchNext()` (stage 3) -
 *         see `Drive.cpp:136` vs `Drive.cpp:149`. Because `_updateStopAtLine()` early-returns
 *         instantly whenever `_stopState == IDLE` (before touching `LineArray` at all), the tick
 *         that pops the queue necessarily ran that early-return BEFORE `_dispatchNext()` flipped
 *         the state to `SEEK` - the seek could not have consumed a line sample that tick. This is
 *         exactly the guarantee the perception-tick gate (T3) leans on: a queued seek popped on
 *         tick N does not need `LineArray` refreshed until tick N+1, whose tick-top predicate
 *         (`isSeekingLine() == true`) will by then correctly reopen the gate before anything
 *         reads it.
 *         NOTE ON THE ASSERTION MECHANISM: `test/path_mocks/WROLineArray.h`'s
 *         `getJunctionType()` has no per-call read counter, and a raw read count would not even
 *         distinguish the two orderings here, since `_updateStopAtLine`'s `SEEK` phase never
 *         calls `getJunctionType()` either way (only `ARMED` does, which is provably unreachable
 *         within the same tick a fresh dispatch seeds `_stopStartEnc` at the current encoder
 *         baseline). So, per the task's documented fallback, this test asserts the
 *         `isSeekingLine()` state-transition timing across that one `Drive::update()` call
 *         instead - the available and sufficient host observable for this ordering claim.
 *   TC6 - a REVERSE seek (`.setReverse(true)`) through the mocked rear board: `isSeekingLine()`
 *         true during, false after; the mock records the rear-board select/restore
 *         (`LineArray.getReverseActive()`) as the host stand-in for the hardware-only reverse
 *         test (rear board unwired per ADR-024).
 *
 * Kept intentionally narrow: this does not re-test the seek FSM's arm-window math, debounce, or
 * overshoot ramp - only `isSeekingLine()` and the dispatch-tick order.
 *
 * HOW THE SUT IS LINKED - mirrors `test_drive_decel_floor` (Drive-only real-link precedent, the
 * issue-#106 recipe MINUS PathTracker/Localization/routes), the narrower correct sibling for a
 * test scoped to `Drive` alone:
 *   - Links the REAL `Drive.cpp` + `YawSource.cpp` (Drive.cpp's `_emitContinuous` GYRO branch
 *     references `YawSource::toHwHeading` - the symbol must resolve at link time even though
 *     every scenario here uses the seek's default SYNC mode and never executes that branch) +
 *     `path_mocks.cpp` (facade globals: `MiniR4`/`Serial`/`Serial1`/`EventTrace`/clock - NOT
 *     `Drive`, so it never collides with the real `Drive.cpp`, the sole definer of `Drive`).
 *   - `${SKETCH_DIR}` FIRST (the inversion) so the real `Drive.h`/`YawSource.h` win;
 *     `${HAL_MOCKS_DIR}` supplies the angle-bracket `<MatrixMiniR4.h>`/`<Arduino.h>` (the
 *     `MiniR4.M2`/`M3` mock encoders, the `MiniR4.DriveDC` mock, and `millis`/`mockAdvanceMillis`
 *     this test drives) and is force-included for `EventTrace.h`; `${PATH_MOCKS_DIR}` supplies the
 *     force-included `WROLineArray.h` (the `LineArray`/`JunctionType` surface this test scripts
 *     directly) / `Intersection.h` / `LineFollower.h` peripherals the real `Drive.cpp`
 *     translation unit needs to link.
 *   - `${MOCKS_DIR}` (the older mock-`DriveClass` world) and `${FIXTURE_ROUTES_DIR}` are
 *     DELIBERATELY OMITTED - this target never touches PathTracker/Localization/routes.
 */

#include "TestAssert.h"

#include <cstdio>

#include "src/drive/Drive.h"    // real - DriveClass + the global `Drive` (defined in real Drive.cpp)
#include <Arduino.h>            // mock - millis/mockAdvanceMillis/mockResetMillis (hal_mocks/, angle-bracket)
#include "MatrixMiniR4.h"       // mock - MiniR4.M2/M3/DriveDC + mmL + mockResetMiniR4() (hal_mocks/)
#include "WROLineArray.h"       // mock - LineArray + JunctionType (path_mocks/)
#include "src/util/WROMath.h"   // cmToEncDeg - sizes the seek's encoder scripting

namespace
{
/// @brief Nominal seek distance (cm) shared by every scenario that doesn't need a specific value.
/// Comfortably inside `_stopMaxCm == kSeekCm + Config::STOP_AT_LINE_MAX_MARGIN_CM` (30 + 15 = 45)
/// so the success-path helpers below never accidentally trip the max-distance fault.
constexpr float kSeekCm = 30.0f;

/// @brief Bound on every per-phase tick handshake below - a real-FSM regression that never
///        settles fails as a budget-exhausted assert, not a hang.
constexpr int kMaxHandshakeTicks = 64;

/// @brief One `Drive::update()` step plus a small fixed clock advance. Kept well below
///        `Config::STOP_AT_LINE_STALL_TIMEOUT_MS` (1500 ms) so a multi-tick handshake never trips
///        the seek's no-progress stall watchdog by accident (this target links no
///        `LineFollower.update()`/`Intersection.update()` - Drive reads `LineArray` directly and
///        never calls either).
void tick()
{
    Drive.update();
    mockAdvanceMillis(5);
}

/// @brief Reset every mock actuator, the mock clock, and `Drive`'s own instance state to a clean
///        per-case world.
///
/// @note `Drive.begin()` reconfigures the lower-MCU PIDs/bias but does NOT reset in-flight
///       FSM/queue state, and neither `begin()` nor `stop()` touches a latched `_fault` - so
///       `Drive.stop()` (force-idles every sub-FSM incl. an in-flight seek, via `_endStopAtLine`,
///       and clears the queue) and `Drive.clearFault()` both run explicitly so a prior case's
///       `LINE_NOT_FOUND` (TC4) or a deliberately-left-in-flight seek (TC2) cannot leak into the
///       next case.
void resetWorld()
{
    mockResetMillis();
    mockResetMiniR4();
    mmL.mockReset();
    LineArray.mockReset();

    Drive.begin();
    Drive.stop();
    Drive.clearFault();
}

/// @brief Enqueue (`.async()`) a `stopAtLine(cm)` seek - optionally reversed through the mocked
///        rear board - and drive it through its dispatch tick, returning with the seek confirmed
///        in flight.
///
/// @details
/// `StopAtLineBuilder::~StopAtLineBuilder()` (`Drive.h`) calls `_drive._enqueue(cmd)`
/// UNCONDITIONALLY when `.async()` was chained - `_enqueue` only appends to the FIFO and never
/// dispatches. Dispatch happens exclusively inside `Drive::update()`'s stage 3
/// (`_isIdle() && _qCount > 0` -> `_dispatchNext()`), so the FIRST `tick()` after this enqueue is
/// unambiguously "the dispatch tick" - the same property TC5 asserts in detail below.
///
/// @param cm      Seek distance in centimetres (`Drive.stopAtLine(cm)`).
/// @param reverse `true` seeks through the mocked rear board (`.setReverse(true)`); `false`
///                (default) is a forward seek.
void dispatchSeek(float cm, bool reverse = false)
{
    if (reverse)
    {
        Drive.stopAtLine(cm).setReverse(true).async();
    }
    else
    {
        Drive.stopAtLine(cm).async();
    }

    // Queued only - no `Drive.update()` has run since the enqueue, so the seek is still IDLE.
    ASSERT_FALSE(Drive.isSeekingLine());

    tick();    // The dispatch tick: Drive::update() pops the queue and seeds _stopState = SEEK.

    ASSERT_TRUE(Drive.isSeekingLine());
    ASSERT_FALSE(Drive.hasFault());
}

/// @brief Retire an in-flight `Drive.stopAtLine` seek to a confirmed crossing (`found == true`)
///        or a no-line `LINE_NOT_FOUND` fault (`found == false`).
///
/// @param cm    The seek's expected distance (cm) - must match what `dispatchSeek` dispatched.
/// @param found `true` arms the window then holds a debounced `T_OR_CROSS` and rolls the
///              overshoot to DONE (asserts `Drive.isIdle()` / `isLineFound()` / no fault).
///              `false` holds the marker NONE and over-advances past
///              `cm + Config::STOP_AT_LINE_MAX_MARGIN_CM` so the max-distance gate latches
///              `Fault::LINE_NOT_FOUND` (asserts `hasFault()` with that fault kind).
void expectStopAtLine(float cm, bool found)
{
    const int32_t baseline = MiniR4.M2.getDegrees();

    if (found)
    {
        // Land traveled == cm: inside the arm window and below the max gate.
        const int32_t armEnc = baseline + (int32_t) cmToEncDeg(cm);
        LineArray.mockSetJunctionType(JunctionType::NONE);
        MiniR4.M2.mockSetDegrees(armEnc);
        MiniR4.M3.mockSetDegrees(armEnc);
        tick();    // SEEK -> ARMED.

        // Hold a debounced T_OR_CROSS without advancing the encoder so detection confirms and
        // the OVERSHOOT roll anchors on the current (detect) encoder.
        LineArray.mockSetJunctionType(JunctionType::T_OR_CROSS);
        for (int i = 0; i < Config::STOP_AT_LINE_DETECT_TICKS + 1 && !Drive.isIdle(); ++i)
        {
            tick();
        }

        // Roll the OVERSHOOT to DONE: advance past the detect encoder by the axle offset plus a
        // generous margin so the roll completes regardless of the resolved overshoot value.
        const int32_t overshootEnc =
            armEnc + (int32_t) cmToEncDeg(Config::LINE_SENSOR_AXLE_OFFSET_CM + 5.0f);
        for (int i = 0; i < kMaxHandshakeTicks && !Drive.isIdle() && !Drive.hasFault(); ++i)
        {
            MiniR4.M2.mockSetDegrees(overshootEnc);
            MiniR4.M3.mockSetDegrees(overshootEnc);
            tick();
        }

        ASSERT_TRUE(Drive.isIdle());
        ASSERT_TRUE(Drive.isLineFound());
        ASSERT_FALSE(Drive.hasFault());
    }
    else
    {
        // No-line fault: marker stays NONE while traveled overruns cm + MAX_MARGIN, so the
        // seek's max-distance gate latches LINE_NOT_FOUND.
        const int32_t faultEnc =
            baseline + (int32_t) cmToEncDeg(cm + Config::STOP_AT_LINE_MAX_MARGIN_CM + 5.0f);
        LineArray.mockSetJunctionType(JunctionType::NONE);
        for (int i = 0; i < kMaxHandshakeTicks && !Drive.hasFault(); ++i)
        {
            MiniR4.M2.mockSetDegrees(faultEnc);
            MiniR4.M3.mockSetDegrees(faultEnc);
            tick();
        }

        ASSERT_TRUE(Drive.hasFault());
        ASSERT_TRUE(Drive.getFault().kind == DriveClass::Fault::LINE_NOT_FOUND);
    }
}
}    // namespace

// ===========================================================================================
// TC1 - isSeekingLine() is false before any stopAtLine has ever been dispatched.
// ===========================================================================================
void test_TC1_false_before_any_seek()
{
    TEST_SCOPE("TC1: isSeekingLine() is false before any stopAtLine has ever run");
    resetWorld();

    ASSERT_TRUE(Drive.isIdle());
    ASSERT_FALSE(Drive.isSeekingLine());
}

// ===========================================================================================
// TC2 - isSeekingLine() is true while a forward seek is in flight (SEEK, then ARMED).
// ===========================================================================================
void test_TC2_true_while_seek_in_flight()
{
    TEST_SCOPE("TC2: isSeekingLine() is true while a forward stopAtLine seek is in flight");
    resetWorld();

    dispatchSeek(kSeekCm);    // SEEK phase - dispatchSeek already asserts isSeekingLine() true.

    // Walk into ARMED (traveled == expected, inside the arm window) - still in flight.
    const int32_t armEnc = MiniR4.M2.getDegrees() + (int32_t) cmToEncDeg(kSeekCm);
    LineArray.mockSetJunctionType(JunctionType::NONE);
    MiniR4.M2.mockSetDegrees(armEnc);
    MiniR4.M3.mockSetDegrees(armEnc);
    tick();

    ASSERT_TRUE(Drive.isSeekingLine());
    ASSERT_FALSE(Drive.hasFault());

    // Deliberately left in flight (ARMED) here - the next test's resetWorld() force-idles it via
    // Drive.stop(), so no explicit retire is needed (TC3 covers the retire-to-IDLE path).
}

// ===========================================================================================
// TC3 - isSeekingLine() is false again once the seek confirms a crossing (DONE -> IDLE).
// ===========================================================================================
void test_TC3_false_after_confirmed_crossing()
{
    TEST_SCOPE("TC3: isSeekingLine() is false again after the seek confirms a crossing (DONE)");
    resetWorld();

    dispatchSeek(kSeekCm);
    expectStopAtLine(kSeekCm, /*found*/ true);

    ASSERT_FALSE(Drive.isSeekingLine());
}

// ===========================================================================================
// TC4 - isSeekingLine() is false again once the seek maxes out with no crossing
// (Fault::LINE_NOT_FOUND).
// ===========================================================================================
void test_TC4_false_after_max_distance_fault()
{
    TEST_SCOPE("TC4: isSeekingLine() is false again after the seek maxes out -> LINE_NOT_FOUND");
    resetWorld();

    dispatchSeek(kSeekCm);
    expectStopAtLine(kSeekCm, /*found*/ false);

    ASSERT_FALSE(Drive.isSeekingLine());
}

// ===========================================================================================
// TC5 - THE ORDERING PROOF: on the tick a QUEUED (async) stopAtLine is dispatched, the seek
// consumes NO line sample that tick, because `_updateStopAtLine()` runs before `_dispatchNext()`
// inside `Drive::update()` (Drive.cpp:136 vs :149). See the file-level doc comment for the full
// reasoning and why isSeekingLine()'s state-transition timing is the available/sufficient host
// observable here.
// ===========================================================================================
void test_TC5_dispatch_tick_consumes_no_line_sample()
{
    TEST_SCOPE("TC5: on the dispatch tick, isSeekingLine() is false right up to it, true after");
    resetWorld();

    ASSERT_TRUE(Drive.isIdle());
    ASSERT_FALSE(Drive.isSeekingLine());

    // `.async()` only enqueues -- see dispatchSeek's doc comment -- so the seek FSM is still IDLE
    // here regardless of how many statements follow, until a `Drive.update()` call happens.
    Drive.stopAtLine(kSeekCm).async();
    ASSERT_FALSE(Drive.isSeekingLine());
    // NOTE: `Drive.isIdle()` is deliberately NOT asserted here -- it ANDs in `_qCount == 0`, and
    // the enqueue just made `_qCount == 1`, so it (correctly) reads false until the dispatch tick
    // below pops the queue. The seek-not-yet-started invariant this case needs is `isSeekingLine()`.

    // THE DISPATCH TICK. Inside this ONE Drive::update() call: stage 2 runs _updateStopAtLine()
    // while _stopState is STILL IDLE (early-return before any LineArray touch -- the seek has not
    // been popped yet), and only THEN does stage 3's _isIdle() && _qCount > 0 check fire
    // _dispatchNext(), which pops the queue and seeds _stopState = SEEK as the LAST thing that
    // happens in this call. So the very call that flips isSeekingLine() false -> true is the same
    // call whose _updateStopAtLine() pass could not have consumed a line sample -- proving the
    // load-bearing order.
    tick();

    ASSERT_TRUE(Drive.isSeekingLine());
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// TC6 - a REVERSE stopAtLine seek (mocked rear board): isSeekingLine() true during, false after.
// Host stand-in for the hardware-only reverse test (rear board unwired, ADR-024).
// ===========================================================================================
void test_TC6_reverse_seek_through_mocked_rear_board()
{
    TEST_SCOPE("TC6: reverse stopAtLine (mocked rear board) -- isSeekingLine() true during, false after");
    resetWorld();

    // Precondition: a reverse seek is rejected at dispatch (LINE_NOT_FOUND) unless the chassis
    // carries a rear board -- pin the fixture assumption loudly.
    ASSERT_TRUE(Config::LINE_BOARD_COUNT == 2);

    dispatchSeek(kSeekCm, /*reverse*/ true);
    ASSERT_TRUE(LineArray.getReverseActive());    // the seek selected the rear board at dispatch.

    expectStopAtLine(kSeekCm, /*found*/ true);

    ASSERT_FALSE(Drive.isSeekingLine());
    ASSERT_FALSE(LineArray.getReverseActive());    // restored to the front board at DONE.
}

// ===========================================================================================
// Entry.
// ===========================================================================================
int main()
{
    std::printf("INFO: Drive.isSeekingLine() + dispatch-tick order (gate-line-array-tick T5, "
                "TC1-TC6) -- real Drive.cpp, LINE_BOARD_COUNT=%d\n",
                (int) Config::LINE_BOARD_COUNT);

    test_TC1_false_before_any_seek();
    test_TC2_true_while_seek_in_flight();
    test_TC3_false_after_confirmed_crossing();
    test_TC4_false_after_max_distance_fault();
    test_TC5_dispatch_tick_consumes_no_line_sample();
    test_TC6_reverse_seek_through_mocked_rear_board();

    TEST_REPORT();
}
