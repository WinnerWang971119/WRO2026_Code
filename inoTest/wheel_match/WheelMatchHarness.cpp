/**
 * @file WheelMatchHarness.cpp
 * @brief Lifted wheel-match test — is the turn imbalance WEIGHT or mechanical DRAG?
 * @author WRO 2026
 *
 * @details
 * The in-place turn rotates off-center because the light right wheel spins much faster than the
 * heavy left at the same PWM. That could be two very different things:
 *   - WEIGHT: the heavy left side loads its wheel with more ground friction, so at a fixed PWM it
 *     turns slower ON THE GROUND. The motors themselves are fine. -> compensate in software.
 *   - DRAG: the left drivetrain has extra mechanical resistance (a tight bearing, a rub, a
 *     misaligned wheel) that slows it even with NO load. -> fix at the mechanism; compensating in
 *     software would just paper over a mechanical fault.
 *
 * This test separates them by removing the ground load: with the robot LIFTED (wheels spinning
 * free), it drives both wheels at matched PWM levels and compares their encoder rates.
 *   - rates match across levels           -> motors/drivetrains are matched; the turn imbalance is
 *                                             the WEIGHT. Software compensation is the right lever.
 *   - left persistently slower even lifted -> mechanical DRAG on the left; fix it there first.
 *
 * Reads the correct (swapped) encoder channel per wheel: LEFT -> M3, RIGHT -> M2 (bench-verified
 * via `inoTest/enc_check`; the encoder cables are crossed relative to the motor-power cables, and
 * that cabling is fixed by the motor harness). Drives both wheels with the raw group `Move`
 * (no sync), so each wheel gets exactly its commanded PWM. Compares MAGNITUDES (MOTOR_LEFT_REVERSE
 * makes the left encoder count down).
 *
 * @note NOT in CI. Build/flash/monitor via `uv run inoTest/wheel_match/build.py`. Log to a file.
 *       LIFT THE ROBOT before pressing BTN_UP -- it drives both wheels and would otherwise shoot
 *       across the bench.
 */

#include "WheelMatchHarness.h"

#include <MatrixMiniR4.h>

#include <math.h>      // fabsf
#include <stdint.h>

#include "Config.h"
#include "EventTrace.h"

namespace
{
    constexpr int16_t  PWM_LEVELS[]     = {30, 40, 50, 60, 70};
    constexpr uint8_t  NUM_LEVELS       = sizeof(PWM_LEVELS) / sizeof(PWM_LEVELS[0]);
    constexpr uint8_t  TRIALS_PER_LEVEL = 2;
    constexpr uint32_t DRIVE_WINDOW_MS  = 500;
    constexpr uint32_t SETTLE_MS        = 400;
    constexpr float    MATCH_RATIO_OK   = 0.85f;    // left/right within this = matched motors

    enum Side
    {
        LEFT = 0,
        RIGHT
    };

    /// @brief Read a wheel's encoder degrees from its ACTUAL (swapped) channel: left -> M3,
    /// right -> M2. Magnitude only (left counts down under MOTOR_LEFT_REVERSE).
    int32_t readWheelDeg(Side s)
    {
        return (s == LEFT) ? MiniR4.M3.getDegrees() : MiniR4.M2.getDegrees();
    }

    void resetBothEncoders()
    {
        MiniR4.M3.resetCounter();    // left wheel's encoder (swapped channel)
        MiniR4.M2.resetCounter();    // right wheel's encoder (swapped channel)
    }

    void stopAndBrake()
    {
        MiniR4.DriveDC.Move(0, 0);
        MiniR4.DriveDC.brake(true);
    }

    bool g_aborted = false;

    void matchPump(void (*tickCb)())
    {
        if (tickCb != nullptr) { tickCb(); }
        yield();
        if (MiniR4.BTN_DOWN.getState()) { g_aborted = true; }
    }

    void matchPumpFor(void (*tickCb)(), uint32_t ms)
    {
        const uint32_t start = millis();
        while ((uint32_t) (millis() - start) < ms)
        {
            matchPump(tickCb);
        }
    }

    void waitForBtnUp(void (*tickCb)())
    {
        Serial.println(F("\n[WM] *** LIFT THE ROBOT so BOTH wheels spin FREE (off the ground). ***"));
        Serial.println(F("[WM] press BTN_UP to start (BTN_DOWN aborts)..."));
        while (MiniR4.BTN_UP.getState()) { matchPump(tickCb); }
        g_aborted = false;
        while (!MiniR4.BTN_UP.getState())
        {
            if (tickCb != nullptr) { tickCb(); }
            yield();
        }
        Serial.println(F("[WM] ...go."));
    }

    /// @brief Drive both wheels at @p pwm (raw, no sync) for a window; return each wheel's rate.
    void measureLevel(int16_t pwm, void (*tickCb)(), float& leftRate, float& rightRate)
    {
        stopAndBrake();
        matchPumpFor(tickCb, SETTLE_MS);

        resetBothEncoders();
        const uint32_t t0 = millis();
        MiniR4.DriveDC.Move(pwm, pwm);
        matchPumpFor(tickCb, DRIVE_WINDOW_MS);

        const int32_t  lDeg = readWheelDeg(LEFT);
        const int32_t  rDeg = readWheelDeg(RIGHT);
        const uint32_t t1   = millis();
        stopAndBrake();

        const float secs = (float) (t1 - t0) / 1000.0f;
        leftRate         = (secs > 0.0f) ? fabsf((float) lDeg) / secs : 0.0f;
        rightRate        = (secs > 0.0f) ? fabsf((float) rDeg) / secs : 0.0f;
    }
}    // namespace

void runWheelMatch(void (*tickCb)())
{
    EventTrace.setHeartbeatEnabled(false);
    g_aborted = false;

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 lifted wheel-match test (inoTest/wheel_match)    #"));
    Serial.println(F("#  Weight vs mechanical drag: drives both wheels UNLOADED   #"));
    Serial.println(F("############################################################"));
    Serial.println(F("[WM] With the robot LIFTED, each wheel sees no ground load, so a rate"));
    Serial.println(F("[WM] difference here is the motor/drivetrain itself, not the weight."));

    waitForBtnUp(tickCb);

    Serial.println(F("[WM] pwm,trial,leftRate,rightRate,ratio(L/R)"));
    float ratioSum   = 0.0f;
    uint8_t ratioN   = 0;
    for (uint8_t i = 0; i < NUM_LEVELS && !g_aborted; ++i)
    {
        const int16_t pwm = PWM_LEVELS[i];
        for (uint8_t trial = 1; trial <= TRIALS_PER_LEVEL && !g_aborted; ++trial)
        {
            float lRate = 0.0f, rRate = 0.0f;
            measureLevel(pwm, tickCb, lRate, rRate);
            const float ratio = (rRate > 0.0f) ? lRate / rRate : 0.0f;
            if (rRate > 0.0f)
            {
                ratioSum += ratio;
                ratioN += 1;
            }
            Serial.print(F("[WM] "));
            Serial.print(pwm);
            Serial.print(',');
            Serial.print(trial);
            Serial.print(',');
            Serial.print(lRate, 1);
            Serial.print(',');
            Serial.print(rRate, 1);
            Serial.print(',');
            Serial.println(ratio, 2);
        }
    }

    stopAndBrake();

    Serial.println(F("\n[WM] ================= RESULT ================="));
    if (g_aborted)
    {
        Serial.println(F("[WM] ABORTED by BTN_DOWN."));
    }
    if (ratioN == 0)
    {
        Serial.println(F("[WM] no valid samples (right wheel never turned?) -- check the wiring."));
    }
    else
    {
        const float avg = ratioSum / (float) ratioN;
        Serial.print(F("[WM] average left/right rate ratio, UNLOADED = "));
        Serial.println(avg, 2);
        if (avg >= MATCH_RATIO_OK)
        {
            Serial.println(F("[WM] -> motors MATCHED unloaded. The turn imbalance is the WEIGHT"));
            Serial.println(F("[WM]    loading the heavy wheel; software compensation is the lever."));
        }
        else
        {
            Serial.println(F("[WM] -> left is slower EVEN UNLOADED. Mechanical DRAG on the left"));
            Serial.println(F("[WM]    drivetrain (bearing/rub/alignment) -- fix that first; it is"));
            Serial.println(F("[WM]    a fault, not something to compensate in software."));
        }
    }
    Serial.println(F("[WM] press RESET to run again."));
    Serial.println(F("[WM] ==========================================="));
}
