/**
 * @file speed_test.ino
 * @brief Does per-motor closed-loop setSpeed regulate under the swapped encoders, or saturate?
 * @author WRO 2026
 *
 * @details
 * Settles whether per-motor `setSpeed()` is actually broken by the encoder crossing (as inferred)
 * or works despite it (as the working group MoveSync hints). Commands setSpeed at several levels on
 * each motor and measures the driven wheel's real rate. Logic in `SpeedTestHarness.cpp`. Minimal
 * init: `MiniR4.begin()` plus the same per-motor reverse/PPR config `Drive::begin` applies.
 *
 * @note BUILD/FLASH/MONITOR: `uv run inoTest/speed_test/build.py`. Log to a file. LIFT the robot.
 * @note NOT in CI.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "EventTrace.h"
#include "SpeedTestHarness.h"

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    if (!EventTrace.begin()) { Serial.println(F("[speed_test] EventTrace begin() FAILED")); }

    runSpeedTest(nullptr);
}

void loop()
{
    // runSpeedTest() returns when done; keep idle.
}
