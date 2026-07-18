/**
 * @file MotorIdHarness.cpp
 * @brief Drive-each-motor identify — closes the power->wheel->encoder loop.
 * @author WRO 2026
 *
 * @details
 * `enc_id` proved by hand which encoder belongs to which wheel (LEFT wheel = M3 encoder, RIGHT
 * wheel = M2 encoder). This does the other half: it drives motor channel M2, then M3, one at a
 * time with the per-motor `setPower` (NO DriveDC group, no reverse flags), and reports which
 * encoder channel responds to each. Combined with the hand-spin, that pins the whole chain:
 *   power channel -> which wheel physically turns (operator watches) -> which encoder counts.
 *
 * Expected, if the swap is real and Config's MOTOR_LEFT_ID=2 is right: driving M2 spins the LEFT
 * wheel and moves the M3 encoder; driving M3 spins the RIGHT wheel and moves the M2 encoder.
 *
 * @note Uses raw `getCounter()` (PPR-independent). Lift the robot so the wheels spin free.
 * @note NOT in CI. Build/flash/monitor via `uv run inoTest/motor_id/build.py`. Log to a file.
 */

#include "MotorIdHarness.h"

#include <MatrixMiniR4.h>

#include <stdint.h>
#include <stdlib.h>    // labs

#include "EventTrace.h"

namespace
{
    constexpr int16_t  DRIVE_SPEED = 45;
    constexpr uint32_t WINDOW_MS   = 1600;
    constexpr uint32_t SAMPLE_MS   = 250;
    constexpr uint32_t SETTLE_MS   = 900;

    void coastBoth()
    {
        MiniR4.M2.setPower(0);
        MiniR4.M3.setPower(0);
        MiniR4.M2.setBrake(false);
        MiniR4.M3.setBrake(false);
    }

    void waitForBtnUp(const __FlashStringHelper* prompt)
    {
        Serial.println();
        Serial.println(prompt);
        Serial.println(F("[MI] press BTN_UP to start..."));
        while (MiniR4.BTN_UP.getState()) { yield(); }
        while (!MiniR4.BTN_UP.getState()) { yield(); }
        Serial.println(F("[MI] ...go."));
    }

    /// @brief Drive one motor channel, print the live encoder deltas, and return which channel
    /// responded (2, 3, or 0 if ambiguous).
    uint8_t driveMotorAndReport(uint8_t motorId)
    {
        Serial.print(F("\n[MI] === driving M"));
        Serial.print(motorId);
        Serial.print(F(".setPower("));
        Serial.print(DRIVE_SPEED);
        Serial.println(F(") -- WATCH which WHEEL turns ==="));

        MiniR4.M2.resetCounter();
        MiniR4.M3.resetCounter();

        if (motorId == 2) { MiniR4.M2.setPower(DRIVE_SPEED); }
        else              { MiniR4.M3.setPower(DRIVE_SPEED); }

        const uint32_t start = millis();
        while ((uint32_t) (millis() - start) < WINDOW_MS)
        {
            delay(SAMPLE_MS);
            Serial.print(F("[MI]   M2 enc="));
            Serial.print(MiniR4.M2.getCounter());
            Serial.print(F("   M3 enc="));
            Serial.println(MiniR4.M3.getCounter());
        }

        const int32_t m2 = MiniR4.M2.getCounter();
        const int32_t m3 = MiniR4.M3.getCounter();
        coastBoth();

        const long a2 = labs((long) m2);
        const long a3 = labs((long) m3);
        uint8_t    responder = 0;
        Serial.print(F("[MI] -> M"));
        Serial.print(motorId);
        Serial.print(F(" power moved the "));
        if (a3 > a2 * 3 && a3 > 10)
        {
            Serial.println(F("M3 encoder (M2 stayed ~still)."));
            responder = 3;
        }
        else if (a2 > a3 * 3 && a2 > 10)
        {
            Serial.println(F("M2 encoder (M3 stayed ~still)."));
            responder = 2;
        }
        else
        {
            Serial.println(F("??? encoder (both moved, or neither -- check the wheel)."));
        }

        const uint32_t s = millis();
        while ((uint32_t) (millis() - s) < SETTLE_MS) { yield(); }
        return responder;
    }
}    // namespace

void runMotorId(void (*tickCb)())
{
    (void) tickCb;
    EventTrace.setHeartbeatEnabled(false);
    coastBoth();

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 motor identify (inoTest/motor_id)               #"));
    Serial.println(F("#  Drive M2, then M3; see which wheel + which encoder       #"));
    Serial.println(F("############################################################"));
    Serial.println(F("[MI] Drives one motor channel at a time (raw setPower). Note which WHEEL"));
    Serial.println(F("[MI] turns for each, and read which encoder it reports the response on."));

    waitForBtnUp(F("[MI] LIFT the robot so the wheels spin free."));

    const uint8_t m2responder = driveMotorAndReport(2);
    const uint8_t m3responder = driveMotorAndReport(3);

    coastBoth();

    Serial.println(F("\n[MI] ================= RESULT ================="));
    Serial.print(F("[MI] M2 power -> encoder M"));
    Serial.println(m2responder);
    Serial.print(F("[MI] M3 power -> encoder M"));
    Serial.println(m3responder);
    Serial.println(F("[MI] You confirmed by hand: LEFT wheel = M3 encoder, RIGHT wheel = M2 encoder."));
    Serial.println(F("[MI] So the motor that moved the M3 encoder drives the LEFT wheel; the one"));
    Serial.println(F("[MI] that moved M2 drives the RIGHT wheel. Cross-check that against the wheel"));
    Serial.println(F("[MI] you actually saw spin for each -- all three should agree."));
    if (m2responder == 3 && m3responder == 2)
    {
        Serial.println(F("[MI] => M2=LEFT wheel, M3=RIGHT wheel (matches Config MOTOR_LEFT_ID=2);"));
        Serial.println(F("[MI]    power correct, encoders crossed. Swap CONFIRMED both ways."));
    }
    else if (m2responder == 2 && m3responder == 3)
    {
        Serial.println(F("[MI] => encoders are NOT crossed (each motor moves its own encoder)."));
        Serial.println(F("[MI]    That contradicts enc_id -- tell me, we need to re-look."));
    }
    Serial.println(F("[MI] press RESET to run again."));
    Serial.println(F("[MI] ==========================================="));
}
