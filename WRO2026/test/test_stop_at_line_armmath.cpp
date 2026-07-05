/**
 * @file test_stop_at_line_armmath.cpp
 * @brief Host-compile tests for the pure `Drive.stopAtLine()` seek decisions
 *        extracted into `WROMath.h` — TC11 (arm-window predicate + detection
 *        debounce counter).
 *
 * These are the ONLY parts of the stateful `Drive._updateStopAtLine()` machine
 * that are unit-testable on the host: the arm-window predicate `stopAtLineArmed`
 * and the detection-debounce step `debounceStep` were factored out of the seek
 * (see the WROMath.h docblock and the 2026-05-30 stop-at-line plan, TC11) so the
 * arm-window DECISION (AC2) and the N-of-N debounce DECISION can be verified in
 * CI. The stateful seek itself (PWM overshoot ramp + encoder roll + active brake)
 * is bench-verified on hardware (B3.7), not here.
 *
 * Pure functions only — no Drive / LineFollower / Localization linkage. `Config.h`
 * and `WROMath.h` are pulled in for the real default constants
 * (`STOP_AT_LINE_ARM_WINDOW_CM`, `STOP_AT_LINE_DETECT_TICKS`) so the assertions
 * bind to the shipped values, not magic numbers.
 */

#include "TestAssert.h"

#include "Arduino.h"    // Mock — supplies PI / millis / Serial that WROMath.h's include of <Arduino.h> resolves.
#include "Config.h"
#include "src/util/WROMath.h"

#include <stdint.h>
#include <cstdio>

// ===========================================================================
// TC11a — stopAtLineArmed(): the arm-window predicate.
//
// Source of truth: `WROMath.h::stopAtLineArmed`, called by
// `Drive::_updateStopAtLine()` for the SEEK → ARMED transition. Detection is
// armed once `traveledCm >= max(0, expectedCm - armWithin)`; a line crossed
// before the window does NOT arm (AC2).
// ===========================================================================

static void test_TC11_arm_window_disarmed_before_window()
{
    TEST_SCOPE("TC11: detection is DISARMED before the chassis reaches the arm window");

    // expected = 40 cm, armWithin = 10 cm → threshold = 30 cm.
    constexpr float expectedCm = 40.0f;
    constexpr float armWithin  = 10.0f;    // arm threshold = 30 cm

    // Travelled 20 cm: well short of the 30 cm threshold — must stay disarmed
    // so a line crossed early en route cannot trigger a premature stop.
    ASSERT_FALSE(stopAtLineArmed(20.0f, expectedCm, armWithin));

    // Travelled 29.9 cm: still just below threshold — disarmed.
    ASSERT_FALSE(stopAtLineArmed(29.9f, expectedCm, armWithin));
}

static void test_TC11_arm_window_armed_inside_window()
{
    TEST_SCOPE("TC11: detection ARMS once travel reaches the window threshold");

    constexpr float expectedCm = 40.0f;
    constexpr float armWithin  = 10.0f;    // arm threshold = 30 cm

    // Exactly at the 30 cm threshold: the predicate uses `>=`, so this arms.
    ASSERT_TRUE(stopAtLineArmed(30.0f, expectedCm, armWithin));

    // Past the threshold (at the expected endpoint and beyond): armed.
    ASSERT_TRUE(stopAtLineArmed(35.0f, expectedCm, armWithin));
    ASSERT_TRUE(stopAtLineArmed(40.0f, expectedCm, armWithin));
    ASSERT_TRUE(stopAtLineArmed(55.0f, expectedCm, armWithin));
}

static void test_TC11_arm_window_small_hop_arms_immediately()
{
    TEST_SCOPE("TC11: armWithin >= expectedCm arms from the first tick (clamped threshold)");

    // Small-hop / "already near the line" case: armWithin >= expectedCm means
    // `expectedCm - armWithin` is <= 0. The predicate clamps the threshold to 0,
    // so detection arms from travel 0 instead of computing a negative threshold.
    ASSERT_TRUE(stopAtLineArmed(0.0f, 5.0f, 10.0f));    // armWithin > expected
    ASSERT_TRUE(stopAtLineArmed(0.0f, 10.0f, 10.0f));   // armWithin == expected
    ASSERT_TRUE(stopAtLineArmed(2.5f, 5.0f, 10.0f));    // mid-hop, still armed

    // Boundary: armWithin exactly equal to expected clamps to 0, so even the
    // very first tick (travel 0) is armed.
    ASSERT_TRUE(stopAtLineArmed(0.0f, 0.0f, 0.0f));
}

static void test_TC11_arm_window_uses_config_default()
{
    TEST_SCOPE("TC11: stopAtLineArmed honors Config::STOP_AT_LINE_ARM_WINDOW_CM as the default window");

    // Bind the test to the shipped default window so a Config change that
    // narrows or widens the window is caught here. With the default window the
    // arm threshold for a 40 cm hop is `40 - STOP_AT_LINE_ARM_WINDOW_CM`.
    constexpr float expectedCm = 40.0f;
    const float     threshold  = expectedCm - Config::STOP_AT_LINE_ARM_WINDOW_CM;

    // Just below the default threshold → disarmed; at the threshold → armed.
    ASSERT_FALSE(stopAtLineArmed(threshold - 0.5f, expectedCm,
                                 Config::STOP_AT_LINE_ARM_WINDOW_CM));
    ASSERT_TRUE(stopAtLineArmed(threshold, expectedCm,
                                Config::STOP_AT_LINE_ARM_WINDOW_CM));
}

// ===========================================================================
// TC11b — debounceStep(): the N-of-N consecutive-sample detection debounce.
//
// Source of truth: `WROMath.h::debounceStep`, called each ARMED tick by
// `Drive::_updateStopAtLine()` with `eventNow = getJunctionType() == T_OR_CROSS`
// and `requiredTicks = Config::STOP_AT_LINE_DETECT_TICKS`. Detection is confirmed
// only when the return value reaches `requiredTicks`.
// ===========================================================================

static void test_TC11_debounce_requires_consecutive_hits()
{
    TEST_SCOPE("TC11: debounce confirms only after N consecutive event hits");

    const uint8_t required = Config::STOP_AT_LINE_DETECT_TICKS;    // shipped default (2)

    // Start with no streak. One hit advances toward — but does not yet reach —
    // the required count (since the shipped default is > 1).
    uint8_t streak = 0;
    streak = debounceStep(streak, /*eventNow*/ true, required);
    ASSERT_EQ(streak, (uint8_t) 1);
    ASSERT_TRUE(streak < required);    // one hit is not enough to confirm

    // A second consecutive hit reaches the required count → detection confirmed.
    streak = debounceStep(streak, /*eventNow*/ true, required);
    ASSERT_EQ(streak, required);
}

static void test_TC11_debounce_resets_on_miss()
{
    TEST_SCOPE("TC11: a single missed sample resets the debounce streak to zero");

    const uint8_t required = Config::STOP_AT_LINE_DETECT_TICKS;

    // Build a partial streak, then miss once — the streak must collapse to 0 so
    // a transient noise blip cannot accumulate toward a false confirm.
    uint8_t streak = 0;
    streak = debounceStep(streak, /*eventNow*/ true, required);
    ASSERT_EQ(streak, (uint8_t) 1);

    streak = debounceStep(streak, /*eventNow*/ false, required);
    ASSERT_EQ(streak, (uint8_t) 0);

    // After the reset, confirmation must again require a fresh run of N hits.
    for (uint8_t i = 0; i < required; ++i)
    {
        streak = debounceStep(streak, /*eventNow*/ true, required);
    }
    ASSERT_EQ(streak, required);
}

static void test_TC11_debounce_saturates_at_required()
{
    TEST_SCOPE("TC11: debounce saturates at requiredTicks while the event stays present");

    const uint8_t required = Config::STOP_AT_LINE_DETECT_TICKS;

    // Hammer the counter with many consecutive hits — it must clamp at
    // `required` and never overflow past it.
    uint8_t streak = 0;
    for (uint8_t i = 0; i < (uint8_t) (required + 5); ++i)
    {
        streak = debounceStep(streak, /*eventNow*/ true, required);
        ASSERT_TRUE(streak <= required);
    }
    ASSERT_EQ(streak, required);
}

static void test_TC11_debounce_requirement_of_one_confirms_immediately()
{
    TEST_SCOPE("TC11: with requiredTicks == 1 a single hit confirms immediately");

    // Degenerate-but-valid config: a 1-tick requirement confirms on the first
    // hit and still resets on a miss. Guards the boundary of the saturation
    // clamp (`streak >= requiredTicks` short-circuits at the first hit).
    uint8_t streak = 0;
    streak = debounceStep(streak, /*eventNow*/ true, /*requiredTicks*/ 1);
    ASSERT_EQ(streak, (uint8_t) 1);

    // Re-hitting stays clamped at 1 (no overflow).
    streak = debounceStep(streak, /*eventNow*/ true, /*requiredTicks*/ 1);
    ASSERT_EQ(streak, (uint8_t) 1);

    // A miss still resets to 0.
    streak = debounceStep(streak, /*eventNow*/ false, /*requiredTicks*/ 1);
    ASSERT_EQ(streak, (uint8_t) 0);
}

// ===========================================================================
// TC16 — stopAtLineSeekMaxedOut(): boundary behaviour of the max-distance cap.
//
// Source of truth: `WROMath.h::stopAtLineSeekMaxedOut`, called each tick by
// `Drive::_updateStopAtLine()` in both the SEEK and ARMED phases to fault the
// seek with `Fault::LINE_NOT_FOUND` once `traveled >= max`. The boundary is
// inclusive (`>=`), so a traveled value exactly equal to `maxCm` counts as
// maxed-out (AC6).
// ===========================================================================

static void test_TC16_seek_maxed_out_boundary()
{
    TEST_SCOPE("TC16: stopAtLineSeekMaxedOut boundary — below / at / above max");

    // Below the cap: seek is still in progress — must NOT fault.
    ASSERT_FALSE(stopAtLineSeekMaxedOut(19.9f, 20.0f));

    // Exactly at the cap: the `>=` boundary is inclusive — counts as maxed-out.
    ASSERT_TRUE(stopAtLineSeekMaxedOut(20.0f, 20.0f));

    // Above the cap: clearly maxed-out.
    ASSERT_TRUE(stopAtLineSeekMaxedOut(25.0f, 20.0f));
}

// ===========================================================================
// TC17 — watchdog pair: stopAtLineProgressed() + stopAtLineStalled().
//
// Source of truth: `WROMath.h::stopAtLineProgressed` and `stopAtLineStalled`,
// called together each tick by `Drive::_updateStopAtLine()`. When `Progressed`
// returns true, Drive resets both `_stopLastProgressCm` and `_stopLastProgressMs`
// so the stall clock does not accumulate. `Stalled` faults the seek once the
// elapsed time since the last progress reset exceeds `stallTimeoutMs`.
//
// Three sub-cases (a) slow-but-moving never stalls, (b) never-advancing trips,
// and (c) rollover safety for the unsigned timestamp subtraction.
// ===========================================================================

static void test_TC17_slow_but_moving_never_stalls()
{
    TEST_SCOPE("TC17a: SLOW-BUT-MOVING — progress resets the watchdog, stall never fires");

    // Seek parameters matching the shipped Config defaults.
    constexpr float    eps     = Config::STOP_AT_LINE_PROGRESS_EPS_CM;
    constexpr uint32_t timeout = Config::STOP_AT_LINE_STALL_TIMEOUT_MS;

    // Simulate 100 cm seek advancing 0.5 cm/tick with now += 20 ms/tick.
    // 0.5 cm > eps (0.2 cm), so every tick records progress and resets lastProgressMs.
    float    traveled       = 0.0f;
    float    lastProgressCm = 0.0f;
    uint32_t lastProgressMs = 0u;
    uint32_t now            = 0u;

    while (traveled < 100.0f)
    {
        traveled += 0.5f;
        now += 20u;

        // Update the watchdog baseline whenever progress is detected — mirrors Drive's logic.
        if (stopAtLineProgressed(traveled, lastProgressCm, eps))
        {
            lastProgressCm = traveled;
            lastProgressMs = now;
        }

        // With fresh progress each tick the elapsed time stays near 20 ms — far below timeout.
        ASSERT_FALSE(stopAtLineStalled(now, lastProgressMs, timeout));
    }
}

static void test_TC17_never_advancing_trips_stall()
{
    TEST_SCOPE("TC17b: NEVER-ADVANCING — stall fires after timeout elapses");

    constexpr uint32_t timeout = Config::STOP_AT_LINE_STALL_TIMEOUT_MS;

    // Travel is frozen at 5.0 cm — no progress is ever recorded.
    constexpr float    traveled       = 5.0f;
    constexpr float    lastProgressCm = 5.0f;    // baseline matches current (no advance)
    constexpr uint32_t lastProgressMs = 0u;

    // Step now forward in 50 ms increments; stall must not fire before the timeout.
    uint32_t now = 0u;
    while (now < timeout)
    {
        ASSERT_FALSE(stopAtLineStalled(now, lastProgressMs, timeout));
        now += 50u;
    }

    // now >= timeout: stall must fire.
    ASSERT_TRUE(stopAtLineStalled(now, lastProgressMs, timeout));

    // Travel is still not progressing — confirm the progress predicate stays false.
    ASSERT_FALSE(stopAtLineProgressed(traveled, lastProgressCm, Config::STOP_AT_LINE_PROGRESS_EPS_CM));
}

static void test_TC17_rollover_safe_stall()
{
    TEST_SCOPE("TC17c: rollover — unsigned subtraction wraps correctly around millis() overflow");

    // Rollover: lastProgressMs near UINT32_MAX, now has wrapped past 0.
    //
    // Case 1: nowMs=50, lastProgressMs=0xFFFFFFF0, stallTimeoutMs=100.
    //   elapsed = (uint32_t)(50 - 0xFFFFFFF0) = (uint32_t)(50 + 16) = 66.
    //   66 < 100 → NOT stalled yet.
    ASSERT_FALSE(stopAtLineStalled(50u, 0xFFFFFFF0u, 100u));

    // Case 2: nowMs=200, lastProgressMs=0xFFFFFFF0, stallTimeoutMs=100.
    //   elapsed = (uint32_t)(200 - 0xFFFFFFF0) = (uint32_t)(200 + 16) = 216.
    //   216 >= 100 → stalled.
    ASSERT_TRUE(stopAtLineStalled(200u, 0xFFFFFFF0u, 100u));
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: stopAtLine arm-window + debounce pure-math (TC11)\n");

    // TC11a — arm-window predicate.
    test_TC11_arm_window_disarmed_before_window();
    test_TC11_arm_window_armed_inside_window();
    test_TC11_arm_window_small_hop_arms_immediately();
    test_TC11_arm_window_uses_config_default();

    // TC11b — detection debounce counter.
    test_TC11_debounce_requires_consecutive_hits();
    test_TC11_debounce_resets_on_miss();
    test_TC11_debounce_saturates_at_required();
    test_TC11_debounce_requirement_of_one_confirms_immediately();

    // TC16 — max-distance cap boundary.
    test_TC16_seek_maxed_out_boundary();

    // TC17 — stall watchdog: progress detection + timeout + rollover.
    test_TC17_slow_but_moving_never_stalls();
    test_TC17_never_advancing_trips_stall();
    test_TC17_rollover_safe_stall();

    TEST_REPORT();
}
