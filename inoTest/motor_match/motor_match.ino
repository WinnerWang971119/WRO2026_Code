/**
 * @file motor_match.ino
 * @brief Bench-test spare motors one at a time and pick the best-matched LEFT/RIGHT pair.
 * @author WRO 2026
 *
 * @details
 * Plug ONE motor into the M2 port, tap BTN_UP to record its speed at three PWM levels (stored to
 * EEPROM), reset and repeat for every spare, then tap BTN_DOWN to print a ranked pair table and the
 * recommended pair (closest speed at equal power, faster wins ties). Logic in
 * `MotorMatchHarness.cpp`. Minimal init: `MiniR4.begin()` + `setBattCell`.
 *
 * @note BUILD/FLASH/MONITOR: `uv run inoTest/motor_match/build.py`. LIFT / free the shafts.
 * @note NOT in CI.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "MotorMatchHarness.h"

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    runMotorMatch(nullptr);    // interactive; never returns
}

void loop()
{
    // runMotorMatch() owns the loop.
}
