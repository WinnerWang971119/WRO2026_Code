/**
 * @file test_turn_imu_watchdog.cpp
 * @brief Host-compile tests for the persistent-IMU-fault watchdog DECISION of the custom
 *        TurnGyro profile — `WROMath.h::turnImuReadStalled`.
 *
 * `Drive.cpp` is NOT host-compiled (heavy MatrixMiniR4 deps), so the only machine-verifiable
 * part of the read-failure escalation in `Drive::_updateTurnProfile()` is the timeout predicate.
 * It was factored into `WROMath.h` (the `stopAtLineStalled` / `test_wall_stop_watchdog`
 * precedent) so production and this test link the SAME symbol and the trip / no-trip / rollover
 * cases are CI-verified.
 *
 * The stateful escalation itself (hard-stop + `Fault::IMU_READ_FAILED` + `_clearQueue`) is
 * bench-verified on hardware — but the DECISION of WHEN to escalate is exactly this predicate,
 * bound here to the shipped `Config::TURN_IMU_STALL_TIMEOUT_MS` so a Config change is caught.
 *
 * Pure function only — no Drive / Localization / MatrixMiniR4 linkage.
 */

#include "TestAssert.h"

#include "Arduino.h"    // Mock — supplies millis / Serial that WROMath.h's <Arduino.h> resolves.
#include "Config.h"
#include "WROMath.h"

#include <stdint.h>
#include <cstdio>

// ===========================================================================
// A good read every tick keeps the watchdog reset — it must NEVER trip.
//
// Mirrors Drive::_updateTurnProfile()'s good-read path, which sets
// `_turnLastGoodReadMs = millis()` on every successful read.
// ===========================================================================

static void test_good_read_each_tick_never_trips()
{
    TEST_SCOPE("turn watchdog: a good read every tick never trips, even over a long turn");

    constexpr uint32_t timeout = Config::TURN_IMU_STALL_TIMEOUT_MS;

    // Simulate a 5 s turn at ~28 ms/tick (the ~35 Hz loop), with a good read each tick.
    uint32_t now            = 0u;
    uint32_t lastGoodReadMs = 0u;
    for (uint32_t i = 0; i < 180u; ++i)
    {
        now += 28u;
        // Good read → reset, exactly as the production code does.
        lastGoodReadMs = now;
        ASSERT_FALSE(turnImuReadStalled(now, lastGoodReadMs, timeout));
    }
}

// ===========================================================================
// A SUSTAINED read failure trips once the timeout elapses (and not before).
// ===========================================================================

static void test_sustained_failure_trips_after_timeout()
{
    TEST_SCOPE("turn watchdog: sustained read failure trips exactly at the timeout boundary");

    constexpr uint32_t timeout = Config::TURN_IMU_STALL_TIMEOUT_MS;

    // Watchdog seeded at dispatch (t=0); no good read ever lands again.
    constexpr uint32_t lastGoodReadMs = 0u;

    // Before the timeout: must NOT trip (hold the previous Move(L,R)).
    uint32_t now = 0u;
    while (now < timeout)
    {
        ASSERT_FALSE(turnImuReadStalled(now, lastGoodReadMs, timeout));
        now += 28u;
    }

    // At / past the timeout: trip. (`now` is now the first tick at or beyond `timeout`.)
    ASSERT_TRUE(turnImuReadStalled(now, lastGoodReadMs, timeout));

    // Exactly at the boundary is inclusive (`>=`).
    ASSERT_TRUE(turnImuReadStalled(timeout, lastGoodReadMs, timeout));
    ASSERT_FALSE(turnImuReadStalled(timeout - 1u, lastGoodReadMs, timeout));
}

// ===========================================================================
// A burst of failures shorter than the timeout, then a good read, must NOT
// trip — the recovery resets the clock (transient-tolerant, the whole point).
// ===========================================================================

static void test_transient_burst_then_recovery_never_trips()
{
    TEST_SCOPE("turn watchdog: failures under the timeout followed by a good read never trip");

    constexpr uint32_t timeout = Config::TURN_IMU_STALL_TIMEOUT_MS;

    uint32_t lastGoodReadMs = 0u;

    // Fail for (timeout - one tick) worth of time — still under the limit.
    uint32_t now = 0u;
    while (now + 28u < timeout)
    {
        now += 28u;
        ASSERT_FALSE(turnImuReadStalled(now, lastGoodReadMs, timeout));
    }

    // A good read lands just in time → reset the window.
    now += 28u;
    lastGoodReadMs = now;
    ASSERT_FALSE(turnImuReadStalled(now, lastGoodReadMs, timeout));

    // Fresh failures from here must again survive the full timeout before tripping —
    // i.e. the earlier burst did not accumulate.
    ASSERT_FALSE(turnImuReadStalled(now + (timeout - 1u), lastGoodReadMs, timeout));
    ASSERT_TRUE(turnImuReadStalled(now + timeout, lastGoodReadMs, timeout));
}

// ===========================================================================
// Rollover safety — the unsigned subtraction wraps correctly around the
// millis() overflow, identical to stopAtLineStalled (TC17c).
// ===========================================================================

static void test_rollover_safe()
{
    TEST_SCOPE("turn watchdog: rollover — unsigned subtraction wraps correctly around millis()");

    // lastGoodReadMs near UINT32_MAX, now has wrapped past 0.
    //   elapsed = (uint32_t)(50 - 0xFFFFFFF0) = 66  → < 100, not stalled.
    ASSERT_FALSE(turnImuReadStalled(50u, 0xFFFFFFF0u, 100u));
    //   elapsed = (uint32_t)(200 - 0xFFFFFFF0) = 216 → >= 100, stalled.
    ASSERT_TRUE(turnImuReadStalled(200u, 0xFFFFFFF0u, 100u));
}

// ===========================================================================
// The shipped Config default is sane: leaves room for transient retries.
// ===========================================================================

static void test_config_default_is_sane()
{
    TEST_SCOPE("turn watchdog: Config::TURN_IMU_STALL_TIMEOUT_MS leaves room for transient misses");

    // The static_assert in ConfigDrive.h already pins this >= 100 ms at compile time; assert it
    // here too so a future Config edit that drops below a few loop ticks fails a test, not just a
    // build the reviewer might bypass.
    ASSERT_TRUE(Config::TURN_IMU_STALL_TIMEOUT_MS >= 100u);
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: turn-profile persistent-IMU-fault watchdog (turnImuReadStalled)\n");

    test_good_read_each_tick_never_trips();
    test_sustained_failure_trips_after_timeout();
    test_transient_burst_then_recovery_never_trips();
    test_rollover_safe();
    test_config_default_is_sane();

    TEST_REPORT();
}
