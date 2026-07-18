/**
 * @file EncCheckHarness.cpp
 * @brief Encoder-read ground-truth diagnostic — finds which encoder read tracks a spinning wheel.
 * @author WRO 2026
 *
 * @details
 * WHY. `inoTest/wheel_floor` drove each wheel and read `MiniR4.M2/M3.getDegrees()` to detect
 * motion, but on the bench the wheel visibly turned while the read said "no". The MatrixMiniR4
 * library exposes TWO encoder paths: per-motor (`M2/M3.getCounter()`/`getDegrees()` ->
 * `mmL.GetEncoderCounter/Degrees(num)`) and a GROUP-level one (`DriveDC.getCounter()` ->
 * `mmL.Get_Drive_EncoderCounter`). Because the turn drives the wheels as a grouped pair
 * (`DriveDC.begin` -> `Set_Drive2Motor_PARAM`), the STM32 may only maintain the GROUP encoder and
 * leave the per-motor telemetry at 0 — or the per-motor `getDegrees` may need a PPR that is never
 * set (`getCounter` is raw and PPR-independent). Rather than guess, this dumps every candidate read
 * so the real one is visible.
 *
 * WHAT. Four labelled tests, each spinning one wheel for ~1 s while printing, every 200 ms, all
 * five reads: `M2.getCounter()`, `M2.getDegrees()`, `M3.getCounter()`, `M3.getDegrees()`,
 * `DriveDC.getCounter()`:
 *   (1) GROUP drive, LEFT   : `DriveDC.Move(PWM, 0)`   — the turn's actuation, left wheel
 *   (2) GROUP drive, RIGHT  : `DriveDC.Move(0, PWM)`   — the turn's actuation, right wheel
 *   (3) PER-MOTOR drive LEFT: `M2.setPower(PWM)`       — ungrouped drive, in case grouping is why
 *   (4) PER-MOTOR drive RIGHT:`M3.setPower(PWM)`
 *
 * It does NOT pump `cooperativeTick()` during the timed drives, so the encoder reads are the only
 * lower-MCU traffic — if a read only works here (and not in `wheel_floor`, which pumps the full
 * tick), the culprit is bus contention, not grouping.
 *
 * HOW TO READ IT. For each test, watch which wheel physically turns and which columns move:
 *   - per-motor columns track under GROUP drive        -> `wheel_floor` just had a threshold/scale
 *                                                          bug; keep `getCounter`.
 *   - per-motor columns are 0 under GROUP drive but the GROUP column moves
 *                                                       -> grouped motors only report the group
 *                                                          encoder; read that (and drive one wheel
 *                                                          at a time so the group value is that
 *                                                          wheel).
 *   - per-motor columns are 0 under GROUP drive but move under PER-MOTOR drive (tests 3/4)
 *                                                       -> grouping suppresses per-motor telemetry;
 *                                                          drive per-motor via `setPower` in the
 *                                                          probe (floor is the same physical PWM).
 *   - getCounter moves but getDegrees stays ~0          -> PPR unset; use counter, not degrees.
 *
 * @note NOT in CI. Build/flash/monitor via `uv run inoTest/enc_check/build.py`. Log to a file.
 */

#include "EncCheckHarness.h"

#include <MatrixMiniR4.h>

#include <stdint.h>

#include "Config.h"
#include "EventTrace.h"

namespace
{
    constexpr int16_t  DRIVE_PWM       = 50;    // safely above the floor so the wheel definitely turns
    constexpr uint8_t  SAMPLES         = 5;      // per test
    constexpr uint32_t SAMPLE_GAP_MS   = 200;    // ~1 s of drive per test
    constexpr uint32_t BETWEEN_TEST_MS = 700;    // stop + settle between tests

    void stopAndBrake()
    {
        MiniR4.DriveDC.Move(0, 0);
        MiniR4.DriveDC.brake(true);
    }

    void resetAllEncoders()
    {
        MiniR4.M2.resetCounter();
        MiniR4.M3.resetCounter();
        MiniR4.DriveDC.resetCounter();
    }

    /// @brief Print one sample row of all five encoder reads.
    void printReads(uint8_t i)
    {
        Serial.print(F("[EC]   s"));
        Serial.print(i);
        Serial.print(F("  M2.cnt="));
        Serial.print(MiniR4.M2.getCounter());
        Serial.print(F(" M2.deg="));
        Serial.print(MiniR4.M2.getDegrees());
        Serial.print(F("  M3.cnt="));
        Serial.print(MiniR4.M3.getCounter());
        Serial.print(F(" M3.deg="));
        Serial.print(MiniR4.M3.getDegrees());
        Serial.print(F("  GROUP.cnt="));
        Serial.println(MiniR4.DriveDC.getCounter());
    }

    /// @brief One test: reset encoders, start @p driveMode, sample all reads, then stop.
    /// @param driveMode 0=Move(PWM,0), 1=Move(0,PWM), 2=M2.setPower(PWM), 3=M3.setPower(PWM).
    void runOneTest(const __FlashStringHelper* label, uint8_t driveMode)
    {
        Serial.print(F("\n[EC] == "));
        Serial.print(label);
        Serial.println(F(" ==  (watch which wheel turns; note which columns move)"));

        stopAndBrake();
        delay(BETWEEN_TEST_MS);
        resetAllEncoders();

        switch (driveMode)
        {
        case 0: MiniR4.DriveDC.Move(DRIVE_PWM, 0); break;
        case 1: MiniR4.DriveDC.Move(0, DRIVE_PWM); break;
        case 2: MiniR4.M2.setPower(DRIVE_PWM); break;
        case 3: MiniR4.M3.setPower(DRIVE_PWM); break;
        default: break;
        }

        for (uint8_t i = 0; i < SAMPLES; ++i)
        {
            delay(SAMPLE_GAP_MS);
            printReads(i);
        }

        stopAndBrake();
        // For the per-motor tests, also cut that motor explicitly in case brake() only acts on the
        // grouped pair.
        MiniR4.M2.setPower(0);
        MiniR4.M3.setPower(0);
    }

    void waitForBtnUp(void (*tickCb)())
    {
        Serial.println(F("\n[EC] place the chassis in a CLEAR CIRCLE. press BTN_UP to start..."));
        while (MiniR4.BTN_UP.getState())
        {
            if (tickCb) { tickCb(); }
            yield();
        }
        while (!MiniR4.BTN_UP.getState())
        {
            if (tickCb) { tickCb(); }
            yield();
        }
        Serial.println(F("[EC] ...go."));
    }
}    // namespace

void runEncCheck(void (*tickCb)())
{
    EventTrace.setHeartbeatEnabled(false);

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 encoder ground-truth check (inoTest/enc_check)   #"));
    Serial.println(F("############################################################"));
    Serial.println(F("[EC] Finds which encoder read tracks a spinning wheel. For each test, note"));
    Serial.println(F("[EC] which WHEEL physically turns and which COLUMN(s) change. Columns:"));
    Serial.println(F("[EC]   M2.cnt/deg = left motor per-motor encoder (raw counts / degrees)"));
    Serial.println(F("[EC]   M3.cnt/deg = right motor per-motor encoder"));
    Serial.println(F("[EC]   GROUP.cnt  = DriveDC group encoder (Get_Drive_EncoderCounter)"));
    Serial.println(F("[EC] No cooperativeTick runs during the drives, so the encoder reads are the"));
    Serial.println(F("[EC] only bus traffic (isolates grouping vs bus-contention)."));

    waitForBtnUp(tickCb);

    runOneTest(F("(1) GROUP drive LEFT  : DriveDC.Move(50,0)"), 0);
    runOneTest(F("(2) GROUP drive RIGHT : DriveDC.Move(0,50)"), 1);
    runOneTest(F("(3) PER-MOTOR LEFT    : M2.setPower(50)"), 2);
    runOneTest(F("(4) PER-MOTOR RIGHT   : M3.setPower(50)"), 3);

    stopAndBrake();
    Serial.println(F("\n[EC] done. Tell me which columns moved for each test -> that picks the fix."));
    Serial.println(F("[EC] press RESET to run again."));
}
