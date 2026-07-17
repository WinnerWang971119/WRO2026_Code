/**
 * @file EncIdHarness.cpp
 * @brief Hand-spin encoder identify — the definitive wheel -> encoder mapping, zero motor drive.
 * @author WRO 2026
 *
 * @details
 * `enc_check` drove the motors and asked the operator to correlate "which wheel spun" with the
 * encoder columns -- easy to second-guess. This test removes every variable: it drives NOTHING
 * (motors coast), and just prints both encoder counters live with a "MOVING" flag on whichever one
 * changes. The operator spins ONE wheel by hand and reads, straight off the wheel, which channel
 * (M2 or M3) that wheel's encoder is on. No motor power, no `DriveDC` grouping, no `MOTOR_*_REVERSE`
 * flag, no direction inference -- the pure physical wheel->encoder wiring, self-verified.
 *
 * Uses `getCounter()` (RAW encoder counts, independent of the PPR setting) so the reading cannot be
 * distorted by an unset/default PPR.
 *
 * @note NOT in CI. Build/flash/monitor via `uv run inoTest/enc_id/build.py`. Log to a file.
 */

#include "EncIdHarness.h"

#include <MatrixMiniR4.h>

#include <stdint.h>
#include <stdlib.h>    // labs

#include "EventTrace.h"

namespace
{
    constexpr int32_t  MOVING_THRESH = 5;      // counts/sample above which a channel is "moving"
    constexpr uint32_t SAMPLE_MS     = 150;
}

void runEncId(void (*tickCb)())
{
    (void) tickCb;
    EventTrace.setHeartbeatEnabled(false);

    // Coast both motors so the wheels spin FREELY by hand. Nothing is ever driven.
    MiniR4.M2.setBrake(false);
    MiniR4.M3.setBrake(false);
    MiniR4.M2.setPower(0);
    MiniR4.M3.setPower(0);

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 encoder identify (inoTest/enc_id)               #"));
    Serial.println(F("#  MOTORS OFF. You spin the wheels BY HAND.                 #"));
    Serial.println(F("############################################################"));
    Serial.println(F("[EI] Nothing is driven. Lift or tip the robot so a wheel spins free."));
    Serial.println(F("[EI]   1) Spin ONLY the LEFT wheel  -> the line flags M2 or M3 as MOVING."));
    Serial.println(F("[EI]      That channel is the LEFT wheel's encoder."));
    Serial.println(F("[EI]   2) Spin ONLY the RIGHT wheel -> same."));
    Serial.println(F("[EI] This is the raw wheel->encoder wiring: no motor power, no config.\n"));

    int32_t lastM2 = MiniR4.M2.getCounter();
    int32_t lastM3 = MiniR4.M3.getCounter();

    for (;;)
    {
        delay(SAMPLE_MS);
        const int32_t m2 = MiniR4.M2.getCounter();
        const int32_t m3 = MiniR4.M3.getCounter();
        const int32_t d2 = m2 - lastM2;
        const int32_t d3 = m3 - lastM3;
        lastM2            = m2;
        lastM3            = m3;

        const bool m2moving = labs((long) d2) >= MOVING_THRESH;
        const bool m3moving = labs((long) d3) >= MOVING_THRESH;

        Serial.print(F("[EI] M2 cnt="));
        Serial.print(m2);
        Serial.print(F(" (d="));
        Serial.print(d2);
        Serial.print(F(")   M3 cnt="));
        Serial.print(m3);
        Serial.print(F(" (d="));
        Serial.print(d3);
        Serial.print(F(")   "));

        if (m2moving && !m3moving)
        {
            Serial.println(F("<==== M2 MOVING  (the wheel you're spinning has its encoder on M2)"));
        }
        else if (m3moving && !m2moving)
        {
            Serial.println(F("<==== M3 MOVING  (the wheel you're spinning has its encoder on M3)"));
        }
        else if (m2moving && m3moving)
        {
            Serial.println(F("<==== BOTH moving -- spin only ONE wheel at a time"));
        }
        else
        {
            Serial.println(F("(still)"));
        }
    }
}
