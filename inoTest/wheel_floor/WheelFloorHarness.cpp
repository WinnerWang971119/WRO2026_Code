/**
 * @file WheelFloorHarness.cpp
 * @brief Per-wheel breakaway/creep floor probe — measures each wheel's own PWM floor with its
 *        own encoder, for the asymmetric-chassis turn fix (Option A).
 * @author WRO 2026
 *
 * @details
 * WHY THIS EXISTS. The chassis's left side is much heavier than the right. In an in-place
 * counter-rotation both wheels get the SAME PWM magnitude, but at the creep/breakaway floor only
 * the light RIGHT wheel breaks away — the heavy LEFT wheel stays below its own friction threshold
 * and barely moves, so the robot pivots around the near-stationary left wheel instead of rotating
 * about its center. A single symmetric `minPWM` is physically wrong when the two wheels carry
 * different loads. The fix (Option A) is a PER-WHEEL floor: the heavy wheel gets a higher
 * breakaway + creep floor. This probe measures those four numbers.
 *
 * WHY THE ENCODER, NOT THE IMU. A yaw reading cannot tell a clean center-rotation from a pivot
 * around the stuck wheel — both change yaw by the same amount, which is exactly why the turn
 * "worked" by yaw while actually pivoting. Each motor has its OWN encoder
 * (`MiniR4.M2/M3.getDegrees()`), so this probe drives one wheel at a time and reads that wheel's
 * rotation directly: the encoder, not a human eye, decides whether the wheel turned. That makes
 * the whole probe automatic and removes the which-wheel blindness.
 *
 * FIDELITY. The wheels are driven through the SAME actuation path the turn uses —
 * `MiniR4.DriveDC.Move(left, right)` (the combined lower-MCU Drive2Motor command configured by
 * `Drive.begin()` with `Config::MOTOR_LEFT_ID/RIGHT_ID` and the reverse flags), NOT per-motor
 * `setPower`. So a floor measured here is the floor the real turn will see. Because
 * `MOTOR_LEFT_REVERSE == true`, the left encoder counts DOWN for a forward-chassis command, so
 * every comparison here is on the encoder-delta MAGNITUDE. The encoder CABLES are also swapped
 * relative to the motor-power cables (bench-verified via `inoTest/enc_check`): the left wheel's
 * encoder reports on channel M3 and the right wheel's on M2, so `readWheelDeg` reads the crossed
 * channel for each wheel — see its comment.
 *
 * WHAT IT DOES, per wheel (LEFT = M2 driven via `Move(pwm, 0)`, RIGHT = M3 via `Move(0, pwm)`):
 *   (1) Breakaway sweep, from rest: raise PWM until the wheel's encoder shows real rotation over a
 *       window. Lowest PWM that moves (unanimously across trials) = that wheel's breakaway floor.
 *   (2) Creep sweep, while rotating: kick the wheel, drop to a test PWM, measure the sustained
 *       encoder rate. Lowest PWM that still sustains = that wheel's kinetic creep floor.
 * Then a COUNTER-ROTATION confirmation: drive both wheels at their measured breakaway floors and
 * read both encoders, reporting whether they turn at ~equal rate (the "did we fix the pivot"
 * check), with a yaw-rate cross-read.
 *
 * The encoder-delta of a stationary wheel is ~0 (a real encoder does not drift like the IMU yaw),
 * so the moved/not-moved threshold is robust and needs no noise-floor derivation. Raw deltas are
 * printed every row so the actual encoder scale is visible on the first flash.
 *
 * Trials alternate drive direction to keep the chassis (which arcs around the idle wheel while one
 * wheel drives) roughly in place; `BTN_DOWN` aborts if it wanders. Zero production weight —
 * `Drive.cpp` is untouched; this only READS encoders and drives raw `Move`. The four floors feed a
 * later per-wheel-floor change to `_updateTurnProfile`.
 *
 * @note NOT in CI — the `inoTest/` diagnostic sketches never are. `arduino-cli` is the only local
 *       check that this links. Build/flash/monitor via `uv run inoTest/wheel_floor/build.py`.
 *       Log STRAIGHT to a file, not `| tee`, or the R4 USB CDC backpressures and freezes mid-run.
 */

#include "WheelFloorHarness.h"

#include <MatrixMiniR4.h>

#include <math.h>      // fabsf
#include <stdint.h>
#include <stdlib.h>    // labs

#include "Config.h"
#include "EventTrace.h"
#include "YawSource.h"

namespace
{
    // ---- Sweep protocol (edit + reflash to change it) -----------------------------------------

    constexpr int16_t  SWEEP_MIN_PWM  = 8;     // start below any plausible floor
    constexpr int16_t  SWEEP_MAX_PWM  = 60;    // ceiling; a wheel that needs more than this is stuck
    constexpr int16_t  SWEEP_STEP_PWM = 2;
    constexpr uint8_t  TRIALS_PER_PWM = 2;     // alternate direction each trial to unwind the arc

    // (1) breakaway, from rest
    constexpr uint32_t BREAKAWAY_WINDOW_MS = 350;    // drive window to judge "did it turn"
    constexpr int32_t  MOVED_THRESH_DEG    = 8;      // encoder |delta| above this = real rotation
    constexpr uint32_t STEP_SETTLE_MS      = 350;    // stop + settle so each rest trial starts still

    // (2) creep, while rotating
    constexpr int16_t  KICK_PWM        = 60;     // stiction-breaking kick before each creep trial
    constexpr uint32_t KICK_MS         = 200;
    constexpr uint32_t DROP_SETTLE_MS  = 300;    // let the kick transient die before measuring
    constexpr uint32_t CREEP_WINDOW_MS = 500;    // encoder-rate measurement window
    constexpr float    SUSTAIN_RATE_FLOOR_DEG_S = 30.0f;    // below this = not sustaining (a coast)

    // confirmation
    constexpr uint32_t CONFIRM_WINDOW_MS = 900;
    constexpr float    BALANCE_RATIO_OK  = 0.80f;    // min/max wheel-rate >= this = balanced

    // ---- Wheels -------------------------------------------------------------------------------

    enum Side
    {
        LEFT = 0,
        RIGHT
    };

    const __FlashStringHelper* sideName(Side s)
    {
        return (s == LEFT) ? F("LEFT(M2)") : F("RIGHT(M3)");
    }

    /// @brief Drive ONE wheel at a signed PWM via the turn's actuation path; the other coasts.
    void driveWheel(Side s, int16_t signedPwm)
    {
        if (s == LEFT)
        {
            MiniR4.DriveDC.Move(signedPwm, 0);
        }
        else
        {
            MiniR4.DriveDC.Move(0, signedPwm);
        }
    }

    // HARDWARE QUIRK (bench-verified 2026-07 via inoTest/enc_check): the encoder cables on this
    // chassis are SWAPPED relative to the motor-power cables. Motor power is correct (M2 drives the
    // LEFT wheel, M3 the RIGHT), but the LEFT wheel's ENCODER reports on channel M3 and the RIGHT
    // wheel's on channel M2. So to read the wheel we DRIVE, we read the OTHER channel. enc_check
    // proved it four ways: Move(50,0) spins the left wheel and ONLY M3 counts (M2 stays 0);
    // Move(0,50) spins the right wheel and ONLY M2 counts. The read itself is clean; the fault was
    // reading the wrong channel. (This same swap crosses the STM32's per-motor encoder feedback —
    // see the memory note; it can affect the encoder-based SYNC/GYRO drive modes, but NOT the turn,
    // which is IMU-based.)

    /// @brief Read one wheel's accumulated encoder degrees, from its ACTUAL (swapped) channel.
    /// Raw value may be negative (MOTOR_LEFT_REVERSE) — always compare magnitudes.
    int32_t readWheelDeg(Side s)
    {
        return (s == LEFT) ? MiniR4.M3.getDegrees() : MiniR4.M2.getDegrees();
    }

    /// @brief Zero one wheel's encoder accumulator (its swapped channel) so the next read is a
    /// clean delta.
    void resetWheel(Side s)
    {
        if (s == LEFT)
        {
            MiniR4.M3.resetCounter();    // left wheel's encoder is on channel M3 (swapped)
        }
        else
        {
            MiniR4.M2.resetCounter();    // right wheel's encoder is on channel M2 (swapped)
        }
    }

    void stopAndBrake()
    {
        MiniR4.DriveDC.Move(0, 0);
        MiniR4.DriveDC.brake(true);
    }

    /// @brief @p mag carrying the sign of @p ref (so a trial's kick and test drive share one
    /// direction; measuring a spin-up through zero would score the wrong transient).
    int16_t signWithMag(int16_t ref, int16_t mag)
    {
        return (ref >= 0) ? mag : (int16_t) -mag;
    }

    // ---- Cooperative pump / operator helpers --------------------------------------------------

    bool g_aborted = false;

    void wheelPump(void (*tickCb)())
    {
        if (tickCb != nullptr)
        {
            tickCb();
        }
        yield();
        if (MiniR4.BTN_DOWN.getState())
        {
            g_aborted = true;
        }
    }

    void wheelPumpFor(void (*tickCb)(), uint32_t ms)
    {
        const uint32_t start = millis();
        while ((uint32_t) (millis() - start) < ms)
        {
            wheelPump(tickCb);
        }
    }

    void wheelWaitForBtnUp(void (*tickCb)(), const __FlashStringHelper* prompt)
    {
        Serial.println();
        Serial.println(prompt);
        Serial.println(F("[WF] press BTN_UP to start (BTN_DOWN aborts at any time)..."));
        while (MiniR4.BTN_UP.getState())
        {
            wheelPump(tickCb);
        }
        // ignore an abort latched by a bounce during the wait
        g_aborted = false;
        while (!MiniR4.BTN_UP.getState())
        {
            if (tickCb != nullptr)
            {
                tickCb();
            }
            yield();
        }
        Serial.println(F("[WF] ...go."));
    }

    // ---- Single-trial measurements ------------------------------------------------------------

    /// @brief From rest, drive one wheel at @p signedPwm for a window; report the encoder delta and
    /// whether it exceeded the moved threshold.
    bool wheelMovedFromRest(Side s, int16_t signedPwm, void (*tickCb)(), int32_t& outDelta)
    {
        stopAndBrake();
        wheelPumpFor(tickCb, STEP_SETTLE_MS);

        resetWheel(s);
        driveWheel(s, signedPwm);
        wheelPumpFor(tickCb, BREAKAWAY_WINDOW_MS);
        outDelta = readWheelDeg(s);
        stopAndBrake();

        return labs((long) outDelta) >= MOVED_THRESH_DEG;
    }

    /// @brief Kick one wheel, drop to @p signedPwm, and return the sustained encoder rate (deg/s).
    float wheelSustainRate(Side s, int16_t signedPwm, void (*tickCb)())
    {
        driveWheel(s, signWithMag(signedPwm, KICK_PWM));
        wheelPumpFor(tickCb, KICK_MS);
        driveWheel(s, signedPwm);
        wheelPumpFor(tickCb, DROP_SETTLE_MS);

        resetWheel(s);
        const uint32_t t0 = millis();
        wheelPumpFor(tickCb, CREEP_WINDOW_MS);
        const int32_t  delta = readWheelDeg(s);
        const uint32_t t1    = millis();

        stopAndBrake();
        wheelPumpFor(tickCb, STEP_SETTLE_MS);

        const float secs = (float) (t1 - t0) / 1000.0f;
        return (secs > 0.0f) ? fabsf((float) delta) / secs : 0.0f;
    }

    // ---- Per-wheel sweeps ---------------------------------------------------------------------

    /// @brief Lowest PWM at which the wheel breaks away from rest on ALL trials; -1 if none did.
    int16_t measureBreakaway(Side s, void (*tickCb)())
    {
        Serial.print(F("\n[WF] --- ("));
        Serial.print(sideName(s));
        Serial.println(F(") breakaway sweep, from rest ---"));
        Serial.println(F("[WF] wheel,pwm,trial,dir,encDelta,moved"));

        for (int16_t pwm = SWEEP_MIN_PWM; pwm <= SWEEP_MAX_PWM && !g_aborted; pwm += SWEEP_STEP_PWM)
        {
            uint8_t passes = 0;
            for (uint8_t trial = 1; trial <= TRIALS_PER_PWM && !g_aborted; ++trial)
            {
                const int16_t dir = (trial % 2 == 1) ? (int16_t) 1 : (int16_t) -1;
                int32_t       delta = 0;
                const bool    moved = wheelMovedFromRest(s, (int16_t) (dir * pwm), tickCb, delta);
                if (moved)
                {
                    passes += 1;
                }
                Serial.print(F("[WF] "));
                Serial.print(sideName(s));
                Serial.print(',');
                Serial.print(pwm);
                Serial.print(',');
                Serial.print(trial);
                Serial.print(',');
                Serial.print(dir > 0 ? F("+1") : F("-1"));
                Serial.print(',');
                Serial.print(delta);
                Serial.print(',');
                Serial.println(moved ? F("YES") : F("no"));
            }
            if (passes == TRIALS_PER_PWM)
            {
                return pwm;
            }
        }
        return -1;
    }

    /// @brief Lowest PWM that still SUSTAINS rotation after a kick; -1 if even the top of the sweep
    /// did not sustain. Sweeps downward and stops at the first PWM that fails.
    int16_t measureCreep(Side s, void (*tickCb)(), float& rateAtFloor)
    {
        Serial.print(F("\n[WF] --- ("));
        Serial.print(sideName(s));
        Serial.println(F(") creep sweep, while rotating ---"));
        Serial.println(F("[WF] wheel,pwm,trial,dir,rateDegS,sustains"));

        int16_t lastSustain = -1;
        rateAtFloor         = 0.0f;
        for (int16_t pwm = SWEEP_MAX_PWM; pwm >= SWEEP_MIN_PWM && !g_aborted; pwm -= SWEEP_STEP_PWM)
        {
            uint8_t passes  = 0;
            float   rateSum = 0.0f;
            for (uint8_t trial = 1; trial <= TRIALS_PER_PWM && !g_aborted; ++trial)
            {
                const int16_t dir  = (trial % 2 == 1) ? (int16_t) 1 : (int16_t) -1;
                const float   rate = wheelSustainRate(s, (int16_t) (dir * pwm), tickCb);
                rateSum += rate;
                const bool sustains = rate >= SUSTAIN_RATE_FLOOR_DEG_S;
                if (sustains)
                {
                    passes += 1;
                }
                Serial.print(F("[WF] "));
                Serial.print(sideName(s));
                Serial.print(',');
                Serial.print(pwm);
                Serial.print(',');
                Serial.print(trial);
                Serial.print(',');
                Serial.print(dir > 0 ? F("+1") : F("-1"));
                Serial.print(',');
                Serial.print(rate, 1);
                Serial.print(',');
                Serial.println(sustains ? F("YES") : F("no"));
            }
            if (passes == TRIALS_PER_PWM)
            {
                lastSustain = pwm;
                rateAtFloor = rateSum / (float) TRIALS_PER_PWM;
            }
            else
            {
                break;    // a friction floor is monotonic: nothing lower will sustain either
            }
        }
        return lastSustain;
    }

    // ---- Counter-rotation confirmation --------------------------------------------------------

    /// @brief Drive a real counter-rotation at the per-wheel breakaway floors and report whether
    /// both wheels turn at ~equal rate (about center) rather than one wheel pivoting.
    void confirmBalance(int16_t lBreak, int16_t rBreak, void (*tickCb)())
    {
        Serial.println(F("\n[WF] --- counter-rotation confirmation (both wheels at their breakaway "
                         "floors) ---"));
        if (lBreak <= 0 || rBreak <= 0)
        {
            Serial.println(F("[WF] a breakaway floor was NOT FOUND -- skipping confirmation."));
            return;
        }

        stopAndBrake();
        wheelPumpFor(tickCb, STEP_SETTLE_MS);

        double yaw0 = 0.0;
        const bool haveYaw0 = YawSource.readYawDeg(yaw0);
        resetWheel(LEFT);
        resetWheel(RIGHT);
        const uint32_t t0 = millis();

        // Counter-rotation: opposite signs on the two wheels, each at its own breakaway floor.
        MiniR4.DriveDC.Move(lBreak, (int16_t) -rBreak);
        wheelPumpFor(tickCb, CONFIRM_WINDOW_MS);

        const int32_t dL = readWheelDeg(LEFT);
        const int32_t dR = readWheelDeg(RIGHT);
        const uint32_t t1 = millis();
        double yaw1 = 0.0;
        const bool haveYaw1 = YawSource.readYawDeg(yaw1);
        stopAndBrake();

        const float secs  = (float) (t1 - t0) / 1000.0f;
        const float rateL = (secs > 0.0f) ? fabsf((float) dL) / secs : 0.0f;
        const float rateR = (secs > 0.0f) ? fabsf((float) dR) / secs : 0.0f;

        Serial.print(F("[WF] left rate="));
        Serial.print(rateL, 1);
        Serial.print(F(" deg/s,  right rate="));
        Serial.print(rateR, 1);
        Serial.println(F(" deg/s (wheel encoders)"));

        const float hi = (rateL > rateR) ? rateL : rateR;
        const float lo = (rateL < rateR) ? rateL : rateR;
        if (hi <= 0.0f)
        {
            Serial.println(F("[WF] neither wheel turned at the breakaway floors -- both floors read "
                             "too low, or a stall. Re-check the sweep rows above."));
        }
        else
        {
            const float ratio = lo / hi;
            Serial.print(F("[WF] balance ratio (slow/fast) = "));
            Serial.print(ratio, 2);
            Serial.println(ratio >= BALANCE_RATIO_OK
                               ? F("  -> BALANCED (rotates about center)")
                               : F("  -> LOPSIDED (the slower wheel needs a higher floor)"));
        }

        if (haveYaw0 && haveYaw1)
        {
            float yawRate = 0.0f;
            if (secs > 0.0f)
            {
                // shortest signed delta, /time; magnitude only (direction irrelevant here)
                float d = (float) (yaw1 - yaw0);
                while (d > 180.0f) { d -= 360.0f; }
                while (d <= -180.0f) { d += 360.0f; }
                yawRate = fabsf(d) / secs;
            }
            Serial.print(F("[WF] chassis yaw rate (IMU cross-check) = "));
            Serial.print(yawRate, 1);
            Serial.println(F(" deg/s"));
        }
    }
}    // namespace

void runWheelFloor(void (*tickCb)())
{
    EventTrace.setHeartbeatEnabled(false);
    g_aborted = false;

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 wheel-floor probe (inoTest/wheel_floor)         #"));
    Serial.println(F("#  Per-wheel breakaway/creep floors via each wheel encoder #"));
    Serial.println(F("############################################################"));
    Serial.println(F("[WF] drives ONE wheel at a time and reads THAT wheel's encoder, so it sees"));
    Serial.println(F("[WF] which wheel actually moved -- the IMU cannot. Raw encoder deltas are"));
    Serial.println(F("[WF] printed each row so the encoder scale is visible."));

    wheelWaitForBtnUp(
        tickCb, F("[WF] place the chassis in a CLEAR CIRCLE (it arcs while one wheel drives)."));

    int16_t lBreak = -1, rBreak = -1, lCreep = -1, rCreep = -1;
    float   lCreepRate = 0.0f, rCreepRate = 0.0f;

    lBreak = measureBreakaway(LEFT, tickCb);
    lCreep = measureCreep(LEFT, tickCb, lCreepRate);
    rBreak = measureBreakaway(RIGHT, tickCb);
    rCreep = measureCreep(RIGHT, tickCb, rCreepRate);

    if (!g_aborted)
    {
        confirmBalance(lBreak, rBreak, tickCb);
    }

    stopAndBrake();

    Serial.println(F("\n[WF] ================= RESULT ================="));
    if (g_aborted)
    {
        Serial.println(F("[WF] ABORTED by BTN_DOWN -- values below are whatever finished:"));
    }
    Serial.print(F("[WF] LEFT (heavy?) : breakaway="));
    Serial.print(lBreak);
    Serial.print(F("  creep="));
    Serial.print(lCreep);
    Serial.print(F("  (creep rate "));
    Serial.print(lCreepRate, 1);
    Serial.println(F(" deg/s)"));
    Serial.print(F("[WF] RIGHT        : breakaway="));
    Serial.print(rBreak);
    Serial.print(F("  creep="));
    Serial.print(rCreep);
    Serial.print(F("  (creep rate "));
    Serial.print(rCreepRate, 1);
    Serial.println(F(" deg/s)"));
    Serial.println(F("[WF] a value of -1 = NOT FOUND (wheel never met the criterion in-sweep)."));
    Serial.println(F("[WF] feed these into the per-wheel floors on the turn path (Option A)."));
    Serial.println(F("[WF] press RESET to run again."));
    Serial.println(F("[WF] ==========================================="));
}
