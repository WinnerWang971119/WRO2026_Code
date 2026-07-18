/**
 * @file motor_id.ino
 * @brief Drive M2 then M3, see which wheel + which encoder — closes the power->wheel->encoder loop.
 * @author WRO 2026
 *
 * @details
 * Companion to `inoTest/enc_id` (which proved the encoder->wheel mapping by hand). This drives each
 * motor channel with the raw per-motor `setPower` (no DriveDC group / reverse flags to confound it)
 * and reports which encoder responds. Logic in `MotorIdHarness.cpp`. Minimal init — only
 * `MiniR4.begin()` is needed to bring up the motors + encoders.
 *
 * @note BUILD/FLASH/MONITOR: `uv run inoTest/motor_id/build.py`. Log to a file. LIFT the robot.
 * @note NOT in CI.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "EventTrace.h"
#include "MotorIdHarness.h"

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    if (!EventTrace.begin()) { Serial.println(F("[motor_id] EventTrace begin() FAILED")); }

    runMotorId(nullptr);
}

void loop()
{
    // runMotorId() returns after both drives; keep idle.
}
