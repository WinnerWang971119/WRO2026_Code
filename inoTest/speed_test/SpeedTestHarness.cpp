/**
 * @file SpeedTestHarness.cpp
 * @brief Per-motor closed-loop `setSpeed` test — does it regulate under the swapped encoders?
 * @author WRO 2026
 *
 * @details
 * Open question: the encoder cables are crossed (motor 2 drives the LEFT wheel but its encoder,
 * channel 2, reads the RIGHT wheel). A per-motor `setSpeed()` PID reads the driven motor's OWN
 * encoder channel, so it would be regulating the left wheel off the right wheel's feedback -- which
 * should run away. But the vendor's GROUP `MoveSync` straight-drive works fine, so the "per-motor
 * is broken" claim was never actually tested. This tests it directly.
 *
 * METHOD. For each motor, command `setSpeed()` at low / mid / high and measure the DRIVEN wheel's
 * real rate via its CORRECT (crossed) encoder — M2 drives the left wheel, read on M3; M3 drives the
 * right wheel, read on M2. If the rate SCALES with the command, the loop regulates on correct
 * feedback and per-motor closed-loop is usable. If it SATURATES (same near-max rate at every
 * command level), the loop is reading the wrong wheel and per-motor closed-loop is out.
 *
 * The per-motor speed PID uses the STM32 default gains (this sketch does not set `setFixSpeedPID`),
 * and PPR/reverse are configured exactly as `Drive::begin` does so the behaviour matches production.
 *
 * @note Uses raw `getCounter()` for the measurement (PPR-independent). LIFT the robot -- if the
 *       loop saturates, the wheel spins at full speed.
 * @note NOT in CI. Build/flash/monitor via `uv run inoTest/speed_test/build.py`. Log to a file.
 */

#include "SpeedTestHarness.h"

#include <MatrixMiniR4.h>

#include <math.h>      // fabsf
#include <stdint.h>

#include "Config.h"
#include "EventTrace.h"

namespace
{
    constexpr int16_t  SPEED_LEVELS[] = {20, 40, 70};
    constexpr uint8_t  NUM_LEVELS     = sizeof(SPEED_LEVELS) / sizeof(SPEED_LEVELS[0]);
    constexpr uint32_t WINDOW_MS      = 1400;
    constexpr uint32_t SETTLE_MS      = 900;

    void coastBoth()
    {
        MiniR4.M2.setBrake(false);
        MiniR4.M3.setBrake(false);
        MiniR4.M2.setPower(0);
        MiniR4.M3.setPower(0);
    }

    /// @brief Encoder channel that actually reads the wheel driven by @p motorId (crossed): motor 2
    /// drives the left wheel -> M3 encoder; motor 3 drives the right wheel -> M2 encoder.
    int32_t readDrivenWheelCounter(uint8_t motorId)
    {
        return (motorId == 2) ? MiniR4.M3.getCounter() : MiniR4.M2.getCounter();
    }

    void resetDrivenWheelCounter(uint8_t motorId)
    {
        if (motorId == 2) { MiniR4.M3.resetCounter(); }
        else              { MiniR4.M2.resetCounter(); }
    }

    /// @brief Command `setSpeed(@p speed)` on @p motorId and return the driven wheel's rate (deg/s).
    float measureSetSpeed(uint8_t motorId, int16_t speed)
    {
        coastBoth();
        {
            const uint32_t s = millis();
            while ((uint32_t) (millis() - s) < SETTLE_MS) { yield(); }
        }

        resetDrivenWheelCounter(motorId);
        const uint32_t t0 = millis();
        if (motorId == 2) { MiniR4.M2.setSpeed(speed); }
        else              { MiniR4.M3.setSpeed(speed); }

        while ((uint32_t) (millis() - t0) < WINDOW_MS) { yield(); }

        const int32_t  delta = readDrivenWheelCounter(motorId);
        const uint32_t t1    = millis();
        coastBoth();

        const float secs = (float) (t1 - t0) / 1000.0f;
        return (secs > 0.0f) ? fabsf((float) delta) / secs : 0.0f;
    }

    void testMotor(uint8_t motorId, const __FlashStringHelper* wheel, uint8_t encCh)
    {
        Serial.print(F("\n[ST] === M"));
        Serial.print(motorId);
        Serial.print(F(".setSpeed  (drives the "));
        Serial.print(wheel);
        Serial.print(F(" wheel; measured on encoder M"));
        Serial.print(encCh);
        Serial.println(F(") ==="));

        float rateLow = 0.0f, rateHigh = 0.0f;
        for (uint8_t i = 0; i < NUM_LEVELS; ++i)
        {
            const float rate = measureSetSpeed(motorId, SPEED_LEVELS[i]);
            if (i == 0) { rateLow = rate; }
            if (i == NUM_LEVELS - 1) { rateHigh = rate; }
            Serial.print(F("[ST]   setSpeed("));
            Serial.print(SPEED_LEVELS[i]);
            Serial.print(F(") -> wheel rate "));
            Serial.print(rate, 1);
            Serial.println(F(" deg/s"));
        }

        Serial.print(F("[ST] -> "));
        if (rateLow > 5.0f && rateHigh > rateLow * 1.8f)
        {
            Serial.println(F("rate SCALES with the command -> speed loop REGULATES (works!)."));
        }
        else if (rateHigh > 5.0f && rateHigh < rateLow * 1.4f)
        {
            Serial.println(F("rate SATURATES (same at every level) -> loop reads the WRONG wheel "
                             "-> per-motor closed-loop BROKEN."));
        }
        else
        {
            Serial.println(F("inconclusive -- read the three rates above by eye."));
        }
    }
}    // namespace

void runSpeedTest(void (*tickCb)())
{
    (void) tickCb;
    EventTrace.setHeartbeatEnabled(false);

    MiniR4.M2.setReverse(Config::MOTOR_LEFT_REVERSE);
    MiniR4.M3.setReverse(Config::MOTOR_RIGHT_REVERSE);
    MiniR4.M2.setPPR_RPM(Config::MOTOR_PPR, Config::MOTOR_RPM);
    MiniR4.M3.setPPR_RPM(Config::MOTOR_PPR, Config::MOTOR_RPM);
    coastBoth();

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 per-motor setSpeed test (inoTest/speed_test)    #"));
    Serial.println(F("#  Does closed-loop speed regulate under the enc swap?      #"));
    Serial.println(F("############################################################"));
    Serial.println(F("[ST] Commands setSpeed at 20/40/70 on each motor and measures the driven"));
    Serial.println(F("[ST] wheel's real rate. Scales with the command = works; saturates = broken."));

    Serial.println(F("\n[ST] LIFT the robot (a broken loop spins the wheel at full speed)."));
    Serial.println(F("[ST] press BTN_UP to start..."));
    while (MiniR4.BTN_UP.getState()) { yield(); }
    while (!MiniR4.BTN_UP.getState()) { yield(); }
    Serial.println(F("[ST] ...go."));

    testMotor(2, F("LEFT"), 3);
    testMotor(3, F("RIGHT"), 2);

    coastBoth();
    Serial.println(F("\n[ST] ================= DONE ================="));
    Serial.println(F("[ST] If BOTH regulate -> closed-loop per-wheel is usable despite the swap,"));
    Serial.println(F("[ST]   and a coprocessor-balanced turn is back on the table."));
    Serial.println(F("[ST] If BOTH saturate -> per-motor closed-loop is out; only group MoveSync is."));
    Serial.println(F("[ST] press RESET to run again."));
    Serial.println(F("[ST] ==========================================="));
}
