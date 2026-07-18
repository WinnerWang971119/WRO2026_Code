/**
 * @file WheelFloorHarness.cpp
 * @brief Counter-rotation floor probe — measures the in-place-turn breakaway/creep PWM floors in
 *        the REAL turn geometry, requiring BOTH wheels to turn (encoder-verified).
 * @author WRO 2026
 *
 * @details
 * WHY THIS EXISTS. The chassis's left side is much heavier than the right. In an in-place
 * counter-rotation both wheels get the same PWM magnitude, but at the shared floor only the light
 * RIGHT wheel breaks away; the heavy LEFT wheel stays stuck and the robot pivots around it instead
 * of rotating about its center.
 *
 * WHY NOT A PER-WHEEL SOLO SWEEP (the first cut, and why it was wrong). Driving one wheel while the
 * other FREEWHEELS lets the robot pivot around that free wheel like a caster -- cheap. So a solo
 * sweep read the floors far too LOW: bench run gave left=42 / right=28, but driving that pair in a
 * real counter-rotation barely moved (left 2.2 deg/s, right 0, yaw 0). A real counter-rotation
 * drives BOTH wheels, so both must SCRUB sideways at once (no free caster), which costs much more
 * PWM. The floor must therefore be measured in counter-rotation, not solo.
 *
 * WHAT IT MEASURES NOW. Symmetric counter-rotation `MiniR4.DriveDC.Move(+p, -p)`, sweeping p, with
 * BOTH wheel encoders read (plus a yaw cross-check):
 *   (1) Breakaway, from rest: the lowest p where BOTH wheels turn (>= MOVED_RATE). This is set by
 *       the HEAVY wheel and is the floor a turn needs to break away cleanly. Requiring BOTH wheels
 *       is the fix for the old `turn_probe` blind spot -- it detected yaw change, which a one-wheel
 *       pivot also produces, so it read the floor too low.
 *   (2) Creep, while rotating: kick both, drop to p, sweep down to the lowest p where BOTH wheels
 *       still SUSTAIN rotation.
 * Because a rigid counter-rotation about center forces both wheels to the SAME rate, the per-row
 * left/right rates also reveal balance: equal rates + healthy yaw = clean center rotation; a big
 * split = one wheel slipping / a translating pivot.
 *
 * ENCODER CHANNELS ARE SWAPPED (bench-verified via `inoTest/enc_check`): motor power is correct
 * (M2 drives the LEFT wheel, M3 the RIGHT) but the LEFT wheel's ENCODER reports on channel M3 and
 * the RIGHT wheel's on M2. `readWheelDeg`/`resetWheel` read the crossed channel. `MOTOR_LEFT_REVERSE`
 * also makes the left encoder count DOWN, so every comparison is on the MAGNITUDE.
 *
 * OUTCOME. If both wheels turn at ~equal rate at the measured floor, the fix is simply to raise the
 * turn's SYMMETRIC floor (`TURN_BREAKAWAY_PWM` / `TURN_CREEP_PWM`) to these values -- no per-wheel
 * split needed (the heavy wheel sets the floor; the light wheel is kinematically held to the same
 * rate). If the rates stay lopsided even above the floor, a per-wheel PWM trim is the follow-up.
 * Zero production weight: `Drive.cpp` is untouched; this only drives raw `Move` and reads encoders.
 *
 * @note NOT in CI. Build/flash/monitor via `uv run inoTest/wheel_floor/build.py`. Log to a file,
 *       not `| tee`. BTN_UP starts, BTN_DOWN aborts.
 */

#include "WheelFloorHarness.h"

#include <MatrixMiniR4.h>

#include <math.h>      // fabsf
#include <stdint.h>

#include "Config.h"
#include "EventTrace.h"
#include "YawSource.h"

namespace
{
    // ---- Sweep protocol (edit + reflash to change it) -----------------------------------------

    // Counter-rotation floors run HIGHER than solo (both wheels scrub), and the heavy wheel did not
    // break away even at cruise (50) on the bench, so the sweep must reach well past it.
    constexpr int16_t  SWEEP_MIN_PWM  = 20;
    constexpr int16_t  SWEEP_MAX_PWM  = 85;
    constexpr int16_t  SWEEP_STEP_PWM = 2;
    constexpr uint8_t  TRIALS_PER_PWM = 2;     // alternate spin direction each trial to stay put

    // A wheel counts as turning above this encoder rate (deg/s). Well above stationary encoder
    // jitter (~0), well below a real creep. Same threshold for breakaway and creep so the two are
    // comparable and breakaway comes out >= creep, as the physics requires.
    constexpr float    MOVED_RATE_DEG_S = 12.0f;

    constexpr uint32_t BREAKAWAY_WINDOW_MS = 400;    // from-rest drive window
    constexpr uint32_t STEP_SETTLE_MS      = 350;    // stop + settle so each rest trial starts still

    constexpr int16_t  KICK_PWM        = 85;     // stiction-breaking counter-rotation kick
    constexpr uint32_t KICK_MS         = 250;
    constexpr uint32_t DROP_SETTLE_MS  = 300;    // let the kick transient die before measuring
    constexpr uint32_t CREEP_WINDOW_MS = 500;    // rate-measurement window

    constexpr float    BALANCE_RATIO_OK = 0.80f;    // min/max wheel rate >= this = balanced

    // ---- Wheels + encoder mapping (channels SWAPPED -- see file header) ------------------------

    enum Side
    {
        LEFT = 0,
        RIGHT
    };

    /// @brief Read one wheel's accumulated encoder degrees from its ACTUAL (swapped) channel: the
    /// LEFT wheel's encoder is on M3, the RIGHT wheel's on M2 (bench-verified via enc_check). Raw
    /// value may be negative (MOTOR_LEFT_REVERSE) -- always compare magnitudes.
    int32_t readWheelDeg(Side s)
    {
        return (s == LEFT) ? MiniR4.M3.getDegrees() : MiniR4.M2.getDegrees();
    }

    /// @brief Zero one wheel's encoder accumulator (its swapped channel) for a clean delta.
    void resetWheel(Side s)
    {
        if (s == LEFT)
        {
            MiniR4.M3.resetCounter();
        }
        else
        {
            MiniR4.M2.resetCounter();
        }
    }

    void stopAndBrake()
    {
        MiniR4.DriveDC.Move(0, 0);
        MiniR4.DriveDC.brake(true);
    }

    /// @brief Drive a symmetric counter-rotation at magnitude @p p; @p dir flips the spin sense
    /// (+1/-1) so trials alternate and the chassis stays roughly put.
    void counterRotate(int16_t p, int8_t dir)
    {
        MiniR4.DriveDC.Move((int16_t) (dir * p), (int16_t) (-dir * p));
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
        g_aborted = false;    // ignore an abort latched by a bounce during the wait
        while (!MiniR4.BTN_UP.getState())
        {
            if (tickCb != nullptr) { tickCb(); }
            yield();
        }
        Serial.println(F("[WF] ...go."));
    }

    // ---- One counter-rotation sample ----------------------------------------------------------

    /// @brief Rates measured over one counter-rotation window. Magnitudes, deg/s.
    struct CrSample
    {
        float leftRate  = 0.0f;
        float rightRate = 0.0f;
        float yawRate   = 0.0f;
    };

    float shortestYawRate(double yaw0, double yaw1, float secs)
    {
        if (secs <= 0.0f)
        {
            return 0.0f;
        }
        float d = (float) (yaw1 - yaw0);
        while (d > 180.0f) { d -= 360.0f; }
        while (d <= -180.0f) { d += 360.0f; }
        return fabsf(d) / secs;
    }

    /// @brief From rest, counter-rotate at @p p (@p dir sense) for a window; return both wheel
    /// rates and the yaw rate.
    CrSample sampleFromRest(int16_t p, int8_t dir, void (*tickCb)())
    {
        CrSample out;
        stopAndBrake();
        wheelPumpFor(tickCb, STEP_SETTLE_MS);

        resetWheel(LEFT);
        resetWheel(RIGHT);
        double         yaw0 = 0.0;
        const bool     y0ok = YawSource.readYawDeg(yaw0);
        const uint32_t t0   = millis();

        counterRotate(p, dir);
        wheelPumpFor(tickCb, BREAKAWAY_WINDOW_MS);

        const int32_t  lDeg = readWheelDeg(LEFT);
        const int32_t  rDeg = readWheelDeg(RIGHT);
        const uint32_t t1   = millis();
        double         yaw1 = 0.0;
        const bool     y1ok = YawSource.readYawDeg(yaw1);
        stopAndBrake();

        const float secs = (float) (t1 - t0) / 1000.0f;
        out.leftRate     = (secs > 0.0f) ? fabsf((float) lDeg) / secs : 0.0f;
        out.rightRate    = (secs > 0.0f) ? fabsf((float) rDeg) / secs : 0.0f;
        out.yawRate      = (y0ok && y1ok) ? shortestYawRate(yaw0, yaw1, secs) : 0.0f;
        return out;
    }

    /// @brief Kick both wheels, drop to @p p, and return the sustained counter-rotation rates.
    CrSample sampleSustain(int16_t p, int8_t dir, void (*tickCb)())
    {
        counterRotate(KICK_PWM, dir);
        wheelPumpFor(tickCb, KICK_MS);
        counterRotate(p, dir);
        wheelPumpFor(tickCb, DROP_SETTLE_MS);

        CrSample out;
        resetWheel(LEFT);
        resetWheel(RIGHT);
        double         yaw0 = 0.0;
        const bool     y0ok = YawSource.readYawDeg(yaw0);
        const uint32_t t0   = millis();

        wheelPumpFor(tickCb, CREEP_WINDOW_MS);

        const int32_t  lDeg = readWheelDeg(LEFT);
        const int32_t  rDeg = readWheelDeg(RIGHT);
        const uint32_t t1   = millis();
        double         yaw1 = 0.0;
        const bool     y1ok = YawSource.readYawDeg(yaw1);
        stopAndBrake();
        wheelPumpFor(tickCb, STEP_SETTLE_MS);

        const float secs = (float) (t1 - t0) / 1000.0f;
        out.leftRate     = (secs > 0.0f) ? fabsf((float) lDeg) / secs : 0.0f;
        out.rightRate    = (secs > 0.0f) ? fabsf((float) rDeg) / secs : 0.0f;
        out.yawRate      = (y0ok && y1ok) ? shortestYawRate(yaw0, yaw1, secs) : 0.0f;
        return out;
    }

    void printSampleRow(int16_t p, uint8_t trial, int8_t dir, const CrSample& s, bool ok)
    {
        Serial.print(F("[WF] "));
        Serial.print(p);
        Serial.print(',');
        Serial.print(trial);
        Serial.print(',');
        Serial.print(dir > 0 ? F("+") : F("-"));
        Serial.print(',');
        Serial.print(s.leftRate, 1);
        Serial.print(',');
        Serial.print(s.rightRate, 1);
        Serial.print(',');
        Serial.print(s.yawRate, 1);
        Serial.print(',');
        Serial.println(ok ? F("BOTH") : F("no"));
    }

    // ---- Sweeps -------------------------------------------------------------------------------

    /// @brief Lowest symmetric PWM where BOTH wheels break away from rest; -1 if none did. Fills
    /// @p atFloor with the rates at that PWM.
    int16_t measureBreakaway(void (*tickCb)(), CrSample& atFloor)
    {
        Serial.println(F("\n[WF] --- counter-rotation BREAKAWAY sweep (both wheels, from rest) ---"));
        Serial.println(F("[WF] pwm,trial,dir,leftRate,rightRate,yawRate,verdict"));

        for (int16_t p = SWEEP_MIN_PWM; p <= SWEEP_MAX_PWM && !g_aborted; p += SWEEP_STEP_PWM)
        {
            uint8_t  passes = 0;
            CrSample last;
            for (uint8_t trial = 1; trial <= TRIALS_PER_PWM && !g_aborted; ++trial)
            {
                const int8_t   dir = (trial % 2 == 1) ? (int8_t) 1 : (int8_t) -1;
                const CrSample s   = sampleFromRest(p, dir, tickCb);
                const bool     ok =
                    s.leftRate >= MOVED_RATE_DEG_S && s.rightRate >= MOVED_RATE_DEG_S;
                if (ok)
                {
                    passes += 1;
                }
                last = s;
                printSampleRow(p, trial, dir, s, ok);
            }
            if (passes == TRIALS_PER_PWM)
            {
                atFloor = last;
                return p;
            }
        }
        return -1;
    }

    /// @brief Lowest symmetric PWM where BOTH wheels still SUSTAIN rotation after a kick; -1 if the
    /// top of the sweep did not. Sweeps downward, stops at the first PWM that fails.
    int16_t measureCreep(void (*tickCb)(), CrSample& atFloor)
    {
        Serial.println(F("\n[WF] --- counter-rotation CREEP sweep (both wheels, while rotating) ---"));
        Serial.println(F("[WF] pwm,trial,dir,leftRate,rightRate,yawRate,verdict"));

        int16_t last = -1;
        for (int16_t p = SWEEP_MAX_PWM; p >= SWEEP_MIN_PWM && !g_aborted; p -= SWEEP_STEP_PWM)
        {
            uint8_t  passes = 0;
            CrSample lastS;
            for (uint8_t trial = 1; trial <= TRIALS_PER_PWM && !g_aborted; ++trial)
            {
                const int8_t   dir = (trial % 2 == 1) ? (int8_t) 1 : (int8_t) -1;
                const CrSample s   = sampleSustain(p, dir, tickCb);
                const bool     ok =
                    s.leftRate >= MOVED_RATE_DEG_S && s.rightRate >= MOVED_RATE_DEG_S;
                if (ok)
                {
                    passes += 1;
                }
                lastS = s;
                printSampleRow(p, trial, dir, s, ok);
            }
            if (passes == TRIALS_PER_PWM)
            {
                last    = p;
                atFloor = lastS;
            }
            else
            {
                break;
            }
        }
        return last;
    }

    void reportBalance(const __FlashStringHelper* label, int16_t pwm, const CrSample& s)
    {
        Serial.print(F("[WF]   "));
        Serial.print(label);
        Serial.print(F(" @ pwm="));
        Serial.print(pwm);
        Serial.print(F(": left="));
        Serial.print(s.leftRate, 1);
        Serial.print(F(" right="));
        Serial.print(s.rightRate, 1);
        Serial.print(F(" yaw="));
        Serial.print(s.yawRate, 1);
        Serial.print(F(" deg/s  -> "));

        const float hi = (s.leftRate > s.rightRate) ? s.leftRate : s.rightRate;
        const float lo = (s.leftRate < s.rightRate) ? s.leftRate : s.rightRate;
        if (hi <= 0.0f)
        {
            Serial.println(F("neither wheel turned"));
        }
        else if ((lo / hi) >= BALANCE_RATIO_OK)
        {
            Serial.println(F("BALANCED (clean center rotation; symmetric floor is enough)"));
        }
        else
        {
            Serial.println(F("LOPSIDED (a per-wheel PWM trim is needed on top of the floor)"));
        }
    }
}    // namespace

void runWheelFloor(void (*tickCb)())
{
    EventTrace.setHeartbeatEnabled(false);
    g_aborted = false;

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 counter-rotation floor probe (wheel_floor)      #"));
    Serial.println(F("#  Both-wheel breakaway/creep floors in real turn geometry #"));
    Serial.println(F("############################################################"));
    Serial.println(F("[WF] Sweeps symmetric counter-rotation Move(+p,-p) and requires BOTH wheels'"));
    Serial.println(F("[WF] encoders to turn (M3=left, M2=right; channels are swapped). The lowest"));
    Serial.println(F("[WF] pwm where BOTH move is the floor a real turn needs -- set by the heavy"));
    Serial.println(F("[WF] wheel. Left/right rates each row show whether it rotates about center."));

    wheelWaitForBtnUp(tickCb, F("[WF] place the chassis in a CLEAR CIRCLE (it spins in place)."));

    CrSample breakFloor, creepFloor;
    const int16_t crBreak = measureBreakaway(tickCb, breakFloor);
    const int16_t crCreep = measureCreep(tickCb, creepFloor);

    stopAndBrake();

    Serial.println(F("\n[WF] ================= RESULT ================="));
    if (g_aborted)
    {
        Serial.println(F("[WF] ABORTED by BTN_DOWN -- values below are whatever finished."));
    }
    Serial.print(F("[WF] counter-rotation BREAKAWAY (both wheels move from rest) = "));
    Serial.println(crBreak);
    Serial.print(F("[WF] counter-rotation CREEP     (both wheels sustain)        = "));
    Serial.println(crCreep);
    Serial.println(F("[WF] a value of -1 = NOT FOUND (never met the both-wheels criterion in-sweep)."));

    if (crBreak > 0)
    {
        reportBalance(F("breakaway"), crBreak, breakFloor);
    }
    if (crCreep > 0)
    {
        reportBalance(F("creep    "), crCreep, creepFloor);
    }

    Serial.println(F("[WF] If BALANCED: raise TURN_BREAKAWAY_PWM / TURN_CREEP_PWM to these values."));
    Serial.println(F("[WF] If LOPSIDED: tell me the split and we add a per-wheel PWM trim."));
    Serial.println(F("[WF] press RESET to run again."));
    Serial.println(F("[WF] ==========================================="));
}
