/**
 * @file test_manipulator.cpp
 * @brief Host-compile tests for the Manipulator facade + ManipServo + ManipBelt (T9).
 *
 * Links the real production `Manipulator.cpp` (and the header-only `ManipServo.h` /
 * `ManipBelt.h` it owns) against the host mocks under `WRO2026/test/mocks/`:
 *   - `<Arduino.h>`       -> mock clock (`g_mockMillis`); `delay(ms)` advances it.
 *   - `<MatrixMiniR4.h>`  -> mock `MiniR4` facade with `RC1..RC4` (MockRC) +
 *                            `M1..M4` (MockDC), each recording call counts / last
 *                            args, with test-settable encoder degrees + rotate-end
 *                            flag (`mockSetDegrees`, `mockRotateEnded`).
 * The global `Manipulator` is defined in `Manipulator.cpp` (linked by CMake, T11).
 *
 * Channel wiring (from ConfigManipulator.h):
 *   - grabber -> RC1   (GRABBER_RC_ID = 1, home 90 / open 30 / close 150)
 *   - sorter  -> RC2   (SORTER_RC_ID  = 2, bins {30, 90, 150})
 *   - dumper  -> RC3   (DUMPER_RC_ID  = 3)
 *   - belt    -> M1    (BELT_M_ID     = 1)
 *
 * Test cases: TC1..TC12 + TC15. TC16 is OMITTED — see the comment block above
 * `test_TC15_begin_homes_all_and_zeroes_belt_once()` for the host-untriggerability
 * rationale (the mock returns true unconditionally and `delay()` advances the clock,
 * so begin()'s wall-clock-bounded home always completes; the timeout-false branch is
 * verified by inspection, and the bounded watchdog is covered by TC9).
 *
 * Each test resets the mock world (MiniR4 channels + clock) and re-`begin()`s the
 * Manipulator at scope entry. The clock is NOT re-zeroed after `begin()` — it sits at
 * the post-boot baseline (~1580 ms, advanced by begin()'s settle + belt home), at which
 * every actuator already reports done. Per-test timing math is RELATIVE: each test that
 * inspects timing re-`setAngle()`s the servo (re-stamping its start to the current clock)
 * or issues a belt move, then advances via `mockAdvanceMillis()` deltas — so the nonzero
 * baseline is harmless. See `resetWorld()` for the full rationale.
 */

#include "TestAssert.h"

#include "Arduino.h"
#include "MatrixMiniR4.h"

// Production code under test (pulls Config.h / ConfigManipulator.h / ManipServo.h /
// ManipBelt.h transitively):
#include "Manipulator.h"

// ---------------------------------------------------------------------------
// Shared helpers.
// ---------------------------------------------------------------------------

/// Reset every mock + the clock, then re-init the Manipulator for a fresh scenario.
///
/// `Manipulator` is a global already constructed at static-init, so we cannot
/// reconstruct it. Instead we wipe the mock recording state, zero the clock, and
/// call `Manipulator.begin()` so the servos seed to home and the belt stall-homes —
/// the actuators all start in their booted, done state and the mock counters are
/// clean.
///
/// Post-`begin()` baseline: the clock sits at ~1580 ms — begin()'s internal settle
/// plus the TIMED belt home (which advances `g_mockMillis` via its `delay()`-driven
/// poll loop) have run it forward. We do NOT re-zero the clock afterward: each servo's
/// `_startMs` was stamped at ~0 during begin() and its `_estMs` is the slew+settle
/// budget (~80 ms for a home no-op), so with the clock now well past that budget every
/// servo's `isDone()` (`millis() - _startMs >= _estMs`) is true. The belt finished its
/// home and is IDLE, so `belt.isDone()` is true. Hence the facade is homed and
/// `Manipulator.isDone()` is true at the end of this function — the correct post-boot
/// baseline. (Re-zeroing the clock here would make `millis() - _startMs == 0 < _estMs`
/// and falsely report the servos NOT done; that was the TC5 bug this fixes.)
///
/// Every per-test scenario that inspects timing first re-`setAngle()`s the servo (which
/// re-stamps `_startMs` to the current clock) or issues a belt move (stamping its start),
/// then uses `mockAdvanceMillis()` RELATIVE deltas — so a nonzero baseline clock is fine.
///
/// We re-wipe the mock after `begin()` so the begin()-time writes/counters do not leak
/// into per-test call-count assertions; the belt encoder is re-zeroed by `begin()`'s
/// home `resetCounter()` either way. We also clear any registered tick callback so a
/// test that registered one (TC10) cannot leak its callback into a later test (S1).
static void resetWorld()
{
    mockResetMiniR4();
    mockResetMillis();

    Manipulator.begin();    // home servos + stall-home belt (consumes mock millis)

    // Wipe begin()-time recording so per-test call-count deltas start from zero. Do NOT
    // re-zero the clock: clearing the mock does not touch the Manipulator's internal
    // timer/state fields (servos done because their settle budget elapsed against the
    // advanced clock, belt IDLE), so the facade stays in its homed, isDone()==true state.
    mockResetMiniR4();

    // Clear any sticky tick callback so the TC10 callback cannot leak across tests.
    Manipulator.setTickCallback(nullptr);
}

/// Clear the registered tick callback so tests that do NOT exercise the callback path
/// are independent of one that did (the callback pointer is a sticky facade member).
///
/// `resetWorld()` already clears the callback at the start of every test (S1), so these
/// explicit calls are redundant but harmless — they are kept for local readability and
/// create no ordering dependency (calling `setTickCallback(nullptr)` twice is idempotent).
static void clearTickCallback()
{
    Manipulator.setTickCallback(nullptr);
}

// ===========================================================================
// TC1 — setAngle clamps to [0,180] and writes the clamped angle (no SW reverse).
// ===========================================================================

static void test_TC1_grabber_setangle_clamps_to_180()
{
    TEST_SCOPE("TC1: grabber.setAngle(200) clamps to 180; RC1 records 180");

    resetWorld();
    clearTickCallback();

    Manipulator.grabber.setAngle(200);

    // Commanded shadow is the clamped value (no software reverse transform).
    ASSERT_EQ(Manipulator.grabber.commandedAngle(), (uint16_t) 180);
    // The mock RC channel for the grabber (RC1) recorded the clamped angle verbatim.
    ASSERT_EQ(MiniR4.RC1.lastAngle, (uint16_t) 180);
    ASSERT_EQ(MiniR4.RC1.setAngleCallCount, (uint32_t) 1);
}

// ===========================================================================
// TC2 / TC3 — timer-based completion crosses the slew+settle budget.
//
// Estimate from cfg: |target - prevShadow| * msPerDeg + settleMs.
// grabber home shadow = 90 (post-begin). setAngle(180):
//   est = |180 - 90| * 2.0 + 80 = 180 + 80 = 260 ms.
// ===========================================================================

static void test_TC2_grabber_not_done_below_estimate()
{
    TEST_SCOPE("TC2: grabber not done just below slew+settle estimate");

    resetWorld();
    clearTickCallback();

    Manipulator.grabber.setAngle(180);    // shadow 90 -> 180

    const uint32_t est = (uint32_t) (90.0f * 2.0f) + 80;    // 260 ms
    // Advance to just BELOW the estimate.
    mockAdvanceMillis(est - 1);
    ASSERT_FALSE(Manipulator.grabber.isDone());
}

static void test_TC3_grabber_done_at_estimate()
{
    TEST_SCOPE("TC3: grabber done once estimate elapses");

    resetWorld();
    clearTickCallback();

    Manipulator.grabber.setAngle(180);    // shadow 90 -> 180

    const uint32_t est = (uint32_t) (90.0f * 2.0f) + 80;    // 260 ms
    // Advance to exactly the estimate (>= path).
    mockAdvanceMillis(est);
    ASSERT_TRUE(Manipulator.grabber.isDone());
}

// ===========================================================================
// TC4 — sortTo(valid bin).async() commands the sorter to SORTER_BIN_ANGLE[bin].
// ===========================================================================

static void test_TC4_sortto_valid_bin_commands_sorter()
{
    TEST_SCOPE("TC4: sortTo(1).async() drives sorter (RC2) to SORTER_BIN_ANGLE[1]");

    resetWorld();
    clearTickCallback();

    Manipulator.sortTo(1).async();

    // Sorter is on RC2; bin 1 angle is SORTER_BIN_ANGLE[1] (== 90).
    ASSERT_EQ(MiniR4.RC2.lastAngle, (uint16_t) Config::SORTER_BIN_ANGLE[1]);
    ASSERT_EQ(MiniR4.RC2.setAngleCallCount, (uint32_t) 1);
}

// ===========================================================================
// TC5 — sortTo(out-of-range) performs NO sorter write and completes immediately.
//
// bin == SORTER_BIN_COUNT is the first OOB index. The guard returns an empty-mask
// action (drain is a no-op) and issues no servo write — so a read of
// SORTER_BIN_ANGLE[bin] never happens (an OOB read would trap under ASan/UBSan, T11).
// ===========================================================================

static void test_TC5_sortto_out_of_range_no_write_and_done()
{
    TEST_SCOPE("TC5: sortTo(SORTER_BIN_COUNT).async() — no sorter write, immediately done");

    resetWorld();
    clearTickCallback();

    const uint32_t writesBefore = MiniR4.RC2.setAngleCallCount;

    Manipulator.sortTo(Config::SORTER_BIN_COUNT).async();

    // No sorter write happened (call count unchanged) — the OOB SORTER_BIN_ANGLE read
    // was avoided by the bounds guard.
    ASSERT_EQ(MiniR4.RC2.setAngleCallCount, writesBefore);
    // Empty-mask action drains as an immediate no-op; the facade is still all-done.
    ASSERT_TRUE(Manipulator.isDone());
}

// ===========================================================================
// TC6 — belt POSITION move to MID (target 400) from getDegrees()==100.
//
// rotateFor is RELATIVE: delta = 400 - 100 = 300 (>0) -> +|BELT_MOVE_SPEED| speed,
// degree magnitude 300. Completion is read from ChkRotateEnd; once the mock's
// rotate-end flag is set true, belt.update() returns the belt to IDLE (done).
// ===========================================================================

static void test_TC6_belt_position_move_to_mid()
{
    TEST_SCOPE("TC6: belt.toStation(MID) issues +speed rotateFor(300); ChkRotateEnd ends it");

    resetWorld();
    clearTickCallback();

    // Belt is on M1. Seed the encoder to 100 before the move.
    MiniR4.M1.mockSetDegrees(100);

    Manipulator.belt.toStation(Config::BeltStation::MID);    // POSITION, abs target 400

    // rotateFor called once with POSITIVE speed (toward FAR_END) and |delta| == 300.
    ASSERT_EQ(MiniR4.M1.rotateForCallCount, (uint32_t) 1);
    ASSERT_GT(MiniR4.M1.lastRotateSpeed, (int16_t) 0);
    ASSERT_EQ(MiniR4.M1.lastRotateDegree, (uint16_t) 300);

    // Move is in flight (POS_MOVE) until the rotate-done bit is set.
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Signal the lower MCU "rotate done"; a single update() retires the move.
    MiniR4.M1.mockRotateEnded = true;
    Manipulator.belt.update();
    ASSERT_TRUE(Manipulator.belt.isDone());
}

// ===========================================================================
// TC7 — belt TIMED move to FAR_END: setSpeed issued, done only after the timer.
//
// FAR_END is TIMED with ms == 1500 (BELT_STATIONS[FAR_END].ms). The belt runs at a
// fixed setSpeed and completes purely on the timer (no ChkRotateEnd).
// ===========================================================================

static void test_TC7_belt_timed_move_to_far_end()
{
    TEST_SCOPE("TC7: belt.toStation(FAR_END) is TIMED — setSpeed, done after the run ms");

    resetWorld();
    clearTickCallback();

    Manipulator.belt.toStation(Config::BeltStation::FAR_END);    // TIMED, 1500 ms

    // setSpeed was issued to drive the belt (M1).
    ASSERT_GE(MiniR4.M1.setSpeedCallCount, (uint32_t) 1);
    // Not done immediately (the run timer has not elapsed).
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Advance past the station's run duration; update() then brakes + idles.
    const uint32_t runMs = Config::BELT_STATIONS[(uint8_t) Config::BeltStation::FAR_END].ms;
    mockAdvanceMillis(runMs);
    Manipulator.belt.update();
    ASSERT_TRUE(Manipulator.belt.isDone());
}

// ===========================================================================
// TC8 — belt home(): TIMED run toward HOME, then brake + resetCounter (zero encoder).
//
// home() is a TIMED run for BELT_HOME_MS. On completion update() brakes the motor and
// calls resetCounter(), which zeroes the mock encoder backing.
// ===========================================================================

static void test_TC8_belt_home_brakes_then_resets_counter()
{
    TEST_SCOPE("TC8: belt.home() -> after BELT_HOME_MS, brake + resetCounter, getDegrees==0");

    resetWorld();
    clearTickCallback();

    // Park the encoder at a nonzero position so the reset is observable.
    MiniR4.M1.mockSetDegrees(250);

    Manipulator.belt.home();    // TIMED toward HOME for BELT_HOME_MS
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Advance past the home run duration; the completing update() brakes + zeroes.
    mockAdvanceMillis(Config::BELT_HOME_MS);
    Manipulator.belt.update();

    ASSERT_TRUE(Manipulator.belt.isDone());
    // resetCounter ran exactly once and zeroed the encoder backing.
    ASSERT_EQ(MiniR4.M1.resetCounterCallCount, (uint32_t) 1);
    ASSERT_EQ(MiniR4.M1.getDegrees(), (int32_t) 0);
    // A brake was engaged on completion (the mock records no global ordering, so we
    // assert the brake fired at least once; the source brakes BEFORE resetCounter()).
    ASSERT_GE(MiniR4.M1.setBrakeCallCount, (uint32_t) 1);
}

// ===========================================================================
// TC9 (AC10) — POSITION-move watchdog forces completion at BELT_MOVE_TIMEOUT_MS.
//
// Start a POSITION move with the rotate-end flag held FALSE forever. update() must
// keep the move in flight until millis()-start >= BELT_MOVE_TIMEOUT_MS, then the
// watchdog fires and forces IDLE (done).
// ===========================================================================

static void test_TC9_belt_position_watchdog_fires()
{
    TEST_SCOPE("TC9: POSITION move watchdog fires at BELT_MOVE_TIMEOUT_MS (AC10)");

    resetWorld();
    clearTickCallback();

    // Encoder far from the target so a real (nonzero) POSITION move is issued.
    MiniR4.M1.mockSetDegrees(0);
    MiniR4.M1.mockRotateEnded = false;    // never reports done — force the watchdog path

    Manipulator.belt.beltTo(800);    // |800 - 0| = 800 -> POS_MOVE in flight
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Just below the watchdog ceiling: still in flight.
    mockAdvanceMillis(Config::BELT_MOVE_TIMEOUT_MS - 1);
    Manipulator.belt.update();
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Cross the ceiling: the watchdog fires and forces completion.
    mockAdvanceMillis(1);    // now == BELT_MOVE_TIMEOUT_MS since the move start
    Manipulator.belt.update();
    ASSERT_TRUE(Manipulator.belt.isDone());
}

// ===========================================================================
// TC10 (AC7) — blocking verb drains until the actuator settles.
//
// With a tick callback registered, _drain spins: callback() + delay(2) each iteration.
// delay(2) advances g_mockMillis, so the grabber's slew+settle timer eventually
// elapses and the blocking grab() statement returns only once grabber.isDone().
// ===========================================================================

static void test_TC10_blocking_grab_drains_until_done()
{
    TEST_SCOPE("TC10: blocking grab() drains until grabber.isDone() (AC7)");

    resetWorld();

    // Register an empty tick callback so _drain uses the callback path (its delay(2)
    // advances the mock clock and the grabber timer elapses).
    Manipulator.setTickCallback([]() {});

    // Blocking verb (no .async()) as a full statement — the temporary ManipAction's
    // destructor drains at end-of-statement.
    Manipulator.grab();

    // After the blocking drain returns, the grabber must be done.
    ASSERT_TRUE(Manipulator.grabber.isDone());

    clearTickCallback();
}

// ===========================================================================
// TC11 (AC7) — async verb returns immediately, mid-travel.
//
// grab() closes the grabber (close pose 150) from the home shadow 90:
//   est = |150 - 90| * 2.0 + 80 = 120 + 80 = 200 ms (> 0), so with no time advanced
// the grabber is still mid-travel right after the async statement returns.
// ===========================================================================

static void test_TC11_async_grab_returns_mid_travel()
{
    TEST_SCOPE("TC11: grab().async() returns immediately, grabber still mid-travel (AC7)");

    resetWorld();
    clearTickCallback();

    Manipulator.grab().async();    // fires close (150), does NOT drain

    // No time advanced -> the close move's timer has not elapsed -> not done.
    ASSERT_FALSE(Manipulator.grabber.isDone());
    // The setpoint WAS issued (async gates the wait, not the firing): RC1 got close.
    ASSERT_EQ(MiniR4.RC1.lastAngle, (uint16_t) 150);
}

// ===========================================================================
// TC12 — reset() homes all three servos and runs the belt home path.
// ===========================================================================

static void test_TC12_reset_homes_all_servos_and_belt()
{
    TEST_SCOPE("TC12: reset().async() homes all three servos + invokes belt.home()");

    resetWorld();
    clearTickCallback();

    // Move every servo OFF home first so the home writes are observable.
    Manipulator.grabber.setAngle(10);
    Manipulator.sorter.setAngle(10);
    Manipulator.dumper.setAngle(10);

    const uint32_t beltResetsBefore = MiniR4.M1.resetCounterCallCount;

    Manipulator.reset().async();    // homes 3 servos + belt.home(); async so no drain

    // Each servo commanded back to its configured home angle (raw, no SW reverse).
    ASSERT_EQ(MiniR4.RC1.lastAngle, (uint16_t) Config::GRABBER.homeDeg);
    ASSERT_EQ(MiniR4.RC2.lastAngle, (uint16_t) Config::SORTER.homeDeg);
    ASSERT_EQ(MiniR4.RC3.lastAngle, (uint16_t) Config::DUMPER.homeDeg);

    // belt.home() ran: a TIMED home toward HOME was kicked off (setSpeed issued, belt
    // now in flight). Drive it to completion to confirm the home path (brake + reset).
    ASSERT_FALSE(Manipulator.belt.isDone());
    mockAdvanceMillis(Config::BELT_HOME_MS);
    Manipulator.belt.update();
    ASSERT_TRUE(Manipulator.belt.isDone());
    // The completing home re-zeroed the encoder via resetCounter().
    ASSERT_EQ(MiniR4.M1.resetCounterCallCount, beltResetsBefore + 1);
}

// ===========================================================================
// TC15 (AC8) — begin() homes everything and zeroes the belt encoder exactly once.
//
// This test deliberately does NOT use resetWorld() (which calls begin() then wipes the
// mock + clock); it drives begin() directly off a freshly-wiped mock/clock so the
// begin()-time recording is preserved for assertion.
//
// --- TC16 OMITTED (host-untriggerable) -------------------------------------
// TC16 would force begin()'s belt home to exceed BELT_HOME_TIMEOUT_MS (the
// timeout-returns-false branch in ManipBelt::begin). It is NOT host-triggerable:
//   1. The T8 MockDC exposes NO fault-injection hook — every vendor call
//      (setSpeed/rotateFor/setBrake/resetCounter/setPPR_RPM/setReverse/ChkRotateEnd)
//      returns true unconditionally, and there is no hook to hold the home incomplete.
//   2. The belt home is a TIMED run: it completes at millis()-start >= BELT_HOME_MS
//      (1500 ms), and begin()'s poll loop advances the clock with delay(2) each spin,
//      bounded by `millis()-start < BELT_HOME_TIMEOUT_MS` (2500 ms). Since
//      BELT_HOME_MS (1500) < BELT_HOME_TIMEOUT_MS (2500), the home ALWAYS completes
//      before the ceiling on host — by construction.
// The timeout-returns-false branch is therefore verified by inspection
// (ManipBelt::begin lines: `if (!isDone()) { _setBrake(true); ... return false; }`),
// and the bounded-watchdog behavior it guards against is covered by TC9. Faking a
// failure here (e.g. a speculative mock hook that does not exist) would not exercise
// real production behavior, so TC16 is intentionally not implemented.
// ===========================================================================

static void test_TC15_begin_homes_all_and_zeroes_belt_once()
{
    TEST_SCOPE("TC15: begin() returns true, all done, belt resetCounter once (AC8)");

    mockResetMiniR4();
    mockResetMillis();

    const bool ok = Manipulator.begin();

    ASSERT_TRUE(ok);
    // All four actuators report done once begin() returns.
    ASSERT_TRUE(Manipulator.grabber.isDone());
    ASSERT_TRUE(Manipulator.sorter.isDone());
    ASSERT_TRUE(Manipulator.dumper.isDone());
    ASSERT_TRUE(Manipulator.belt.isDone());
    ASSERT_TRUE(Manipulator.isDone());
    // The belt home zeroed the encoder exactly once during begin().
    ASSERT_EQ(MiniR4.M1.resetCounterCallCount, (uint32_t) 1);

    // Leave the world reset for any later test ordering hygiene.
    mockResetMiniR4();
    mockResetMillis();
}

// ===========================================================================
// Test entry point.
// ===========================================================================

int main()
{
    test_TC1_grabber_setangle_clamps_to_180();
    test_TC2_grabber_not_done_below_estimate();
    test_TC3_grabber_done_at_estimate();
    test_TC4_sortto_valid_bin_commands_sorter();
    test_TC5_sortto_out_of_range_no_write_and_done();
    test_TC6_belt_position_move_to_mid();
    test_TC7_belt_timed_move_to_far_end();
    test_TC8_belt_home_brakes_then_resets_counter();
    test_TC9_belt_position_watchdog_fires();
    test_TC10_blocking_grab_drains_until_done();
    test_TC11_async_grab_returns_mid_travel();
    test_TC12_reset_homes_all_servos_and_belt();
    test_TC15_begin_homes_all_and_zeroes_belt_once();
    TEST_REPORT();
}
