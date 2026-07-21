/**
 * @file TurnTuneHarness.cpp
 * @brief Implementation of the closed-loop turn-accuracy bench harness (T11 gain tuning).
 * @author WRO 2026
 *
 * @details
 * The companion to `inoTest/turn_probe`. The probe measures the OPEN-LOOP plant constants the
 * PD turn executor needs before it can run at all — the kinetic/breakaway creep floors, the
 * turn-representative tick, and the IMU noise floor (docs/plans/2026-07-16-turn-pd-controller.md,
 * tasks T1/T10). This harness is the next step in that plan (T11): with the executor built, it
 * runs the REAL `Drive.turnDeg()` PD path over and over and reports how accurately it lands, so
 * `Config::TURN_KP` / `TURN_KD` / `TURN_CREEP_PWM` / `TURN_BREAKAWAY_PWM` / `TURN_SLEW_PWM_PER_TICK`
 * can be bench-tuned on the assembled Lofaix chassis and B3.2 re-passed.
 *
 * Kept in its own `.cpp` (not inlined into `turn_tune.ino`) for the same ctags/auto-prototype
 * reason `TurnProbeHarness.cpp` documents at length — see that file's header.
 *
 * WHAT IT MEASURES, per trial:
 *   - restDelta : the SIGNED rotation the IMU actually turned, read after a settle pause so
 *                 any post-brake coast is included. This is the number nav accuracy pays.
 *   - overshoot : |restDelta| - |cmd|. Positive = the chassis went PAST the target; negative =
 *                 it stopped SHORT. Direction-independent, so a symmetric over/under-shoot shows
 *                 as a non-zero MEAN rather than cancelling across the alternating directions.
 *   - coast     : rotation between the instant `Drive.turnDeg()` returned (brake applied) and
 *                 the settled read — how far the chassis drifts after the controller declares
 *                 the turn done. A large coast is a brake/creep tuning signal the controller's
 *                 own in-band exit cannot see.
 *   - durMs     : wall-clock the blocking turn took. A clean turn settles fast; a hunting/
 *                 oscillating one is slow; a stalled one runs to `Config::TURN_TIMEOUT_MS`.
 *
 * WHY THE TURNS ARE SYNC-BLOCKING (not `.async()` + poll): a sync turn spins on the sketch's
 * real `cooperativeTick()` (the same module set, in the same order, at the same
 * turn-representative tick a real nav turn pays), so the PD dynamics measured here match what
 * the mission will see. Polling from the harness would add a second yaw read per iteration and
 * slow the loop, changing the very dynamics being tuned. The cost is that peak mid-turn
 * excursion is not captured directly — it surfaces indirectly as `coast`, `durMs`, and the
 * resting `overshoot`, which is what the operator tunes against anyway.
 *
 * TETHER: every trial is an in-place counter-rotation and the operator reads results over the
 * USB cable, so a same-way run winds the cable onto the chassis and its drag loads the plant.
 * Trials alternate direction (`TURN_TUNE_TRIALS` is even, `static_assert`-pinned), so the net
 * wind is bounded by a single trial's rotation and cancels to ~zero over the run. The running
 * net is printed periodically so a runaway (e.g. repeated one-way timeouts) is visible on the
 * spot rather than only as a taut cable.
 *
 * @note NOT in CI — the `inoTest/` diagnostic sketches never are. `arduino-cli` is the only
 *       local check that this links. Build/flash/monitor via `uv run inoTest/turn_tune/build.py`.
 */

#include "TurnTuneHarness.h"

#include <MatrixMiniR4.h>

#include <math.h>      // fabsf, sqrtf
#include <stdint.h>

#include "Config.h"
#include "Drive.h"
#include "EventTrace.h"
#include "YawSource.h"

namespace
{
    // ---- Run protocol (edit + reflash to change the sweep; gains live in ConfigDrive.h) -------

    /// @brief Commanded turn magnitude per trial, degrees. The plan's B3.2 turn is 90; a smaller
    /// value (e.g. 45) stresses the breakaway floor near the end of a turn, a larger one stresses
    /// overshoot. One magnitude per flash keeps the tether math (below) trivial.
    constexpr int16_t TURN_TUNE_MAGNITUDE_DEG = 90;

    /// @brief Number of turns per run. MUST be even so the alternating +/- direction leaves zero
    /// net tether wind — see the `static_assert` below.
    constexpr uint16_t TURN_TUNE_TRIALS = 20;

    /// @brief Pause after each turn returns, before the settled read, so any post-brake coast is
    /// captured in `restDelta`. Sized well above the chassis's mechanical settle.
    constexpr uint32_t TURN_TUNE_COAST_SETTLE_MS = 600;

    /// @brief Idle gap between trials — long enough for the operator to see the alternation and
    /// for the chassis to sit fully still before the next `turnDeg`.
    constexpr uint32_t TURN_TUNE_GAP_MS = 500;

    /// @brief Print the running tether net every N trials.
    constexpr uint16_t TURN_TUNE_TETHER_PRINT_EVERY = 5;

    /// @brief A turn whose wall-clock lands within this margin of `Config::TURN_TIMEOUT_MS` is
    /// flagged as a probable timeout (it never settled) rather than a clean landing.
    constexpr uint32_t TURN_TUNE_TIMEOUT_GUARD_MS = 75;

    static_assert(TURN_TUNE_TRIALS % 2 == 0,
                  "TURN_TUNE_TRIALS must be EVEN: trials alternate +/- direction to unwind the "
                  "USB tether, and an odd count leaves a net one-way wind that drags the plant.");
    static_assert(TURN_TUNE_TRIALS >= 2,
                  "TURN_TUNE_TRIALS must be >= 2 to produce any per-direction split.");
    static_assert(TURN_TUNE_MAGNITUDE_DEG > 0 && TURN_TUNE_MAGNITUDE_DEG <= 180,
                  "TURN_TUNE_MAGNITUDE_DEG must be in (0, 180]; a single turn never wraps.");

    // ---- Cooperative pump + heading helpers (file-local twins of the probe's) -----------------

    /// @brief Drive the caller's tick callback and yield. Falls back to `Drive.update()` alone if
    /// none is registered (defensive only; `turn_tune.ino` always passes its real
    /// `cooperativeTick`).
    void turnTunePump(void (*tickCb)())
    {
        if (tickCb != nullptr)
        {
            tickCb();
        }
        else
        {
            Drive.update();
        }
        yield();
    }

    /// @brief Pump cooperative ticks for @p ms milliseconds, keeping every module alive.
    void turnTunePumpFor(void (*tickCb)(), uint32_t ms)
    {
        const uint32_t start = millis();
        while ((uint32_t) (millis() - start) < ms)
        {
            turnTunePump(tickCb);
        }
    }

    /// @brief Fold a raw heading delta into (-180, +180]. File-local twin of
    /// `TurnPDPolicy.h::turnWrap180`, kept local so this harness is one cleanly-revertible block.
    float turnTuneWrap180(float deltaDeg)
    {
        while (deltaDeg > 180.0f)
        {
            deltaDeg -= 360.0f;
        }
        while (deltaDeg <= -180.0f)
        {
            deltaDeg += 360.0f;
        }
        return deltaDeg;
    }

    /// @brief Blocking wait for a fresh `BTN_UP` rising edge, pumping cooperative ticks. Waits out
    /// any already-held press first so a leftover press cannot satisfy this without a real
    /// press-after-prompt.
    void turnTuneWaitForBtnUp(void (*tickCb)(), const __FlashStringHelper* prompt)
    {
        Serial.println();
        Serial.println(prompt);
        Serial.println(F("[TT] press BTN_UP to start..."));

        while (MiniR4.BTN_UP.getState())
        {
            turnTunePump(tickCb);
        }
        while (!MiniR4.BTN_UP.getState())
        {
            turnTunePump(tickCb);
        }
        Serial.println(F("[TT] ...go."));
    }

    /// @brief Read yaw via `YawSource`, printing a warning and leaving @p out untouched on a hard
    /// transport failure (`YawSource::readYawDeg` already retries once inside).
    bool turnTuneReadYaw(double& out, const __FlashStringHelper* site)
    {
        if (YawSource.readYawDeg(out))
        {
            return true;
        }
        Serial.print(F("[TT] yaw read FAILED at "));
        Serial.println(site);
        return false;
    }

    // ---- Running statistics -------------------------------------------------------------------

    /// @brief Accumulates a stream of samples for mean / mean-abs / stdev / extremes. Cheap,
    /// fixed-size — no per-trial storage.
    struct Stat
    {
        uint16_t n      = 0;
        float    sum    = 0.0f;
        float    sumsq  = 0.0f;
        float    sumAbs = 0.0f;
        float    minV   = 0.0f;
        float    maxV   = 0.0f;
        float    maxAbs = 0.0f;

        void add(float v)
        {
            if (n == 0)
            {
                minV = v;
                maxV = v;
            }
            else
            {
                if (v < minV) { minV = v; }
                if (v > maxV) { maxV = v; }
            }
            n += 1;
            sum += v;
            sumsq += v * v;
            sumAbs += fabsf(v);
            if (fabsf(v) > maxAbs) { maxAbs = fabsf(v); }
        }

        float mean() const { return n ? sum / (float) n : 0.0f; }
        float meanAbs() const { return n ? sumAbs / (float) n : 0.0f; }

        float stdev() const
        {
            if (n < 2)
            {
                return 0.0f;
            }
            const float m   = mean();
            const float var = (sumsq - (float) n * m * m) / (float) (n - 1);
            return var > 0.0f ? sqrtf(var) : 0.0f;
        }
    };

    /// @brief Echo the gains in force so each run's log self-documents which build produced it.
    void turnTunePrintGains()
    {
        Serial.println(F("[TT] gains in force (WRO2026/config/ConfigDrive.h):"));
        Serial.print(F("[TT]   TURN_KP="));
        Serial.print(Config::TURN_KP, 4);
        Serial.print(F("  TURN_KD="));
        Serial.print(Config::TURN_KD, 4);
        Serial.print(F("  TURN_CRUISE_PWM="));
        Serial.println(Config::TURN_CRUISE_PWM);
        Serial.print(F("[TT]   TURN_BREAKAWAY_PWM="));
        Serial.print(Config::TURN_BREAKAWAY_PWM);
        Serial.print(F("  TURN_BREAKAWAY_MOTION_DEG="));
        Serial.print(Config::TURN_BREAKAWAY_MOTION_DEG, 3);
        Serial.print(F("  TURN_CREEP_THRESH_PWM="));
        Serial.print(Config::TURN_CREEP_THRESH_PWM);
        Serial.print(F("  TURN_CREEP_PWM="));
        Serial.println(Config::TURN_CREEP_PWM);
        Serial.print(F("[TT]   TURN_SLEW_PWM_PER_TICK="));
        Serial.print(Config::TURN_SLEW_PWM_PER_TICK);
        Serial.print(F("  TURN_SETTLE_SAMPLES="));
        Serial.print(Config::TURN_SETTLE_SAMPLES);
        Serial.print(F("  TURN_TOLERANCE_DEG="));
        Serial.print(Config::TURN_TOLERANCE_DEG, 3);
        Serial.print(F("  TURN_TIMEOUT_MS="));
        Serial.println(Config::TURN_TIMEOUT_MS);
    }

    /// @brief Print the per-run summary from the accumulated statistics.
    /// @param ov         Signed overshoot (|restDelta| - |cmd|) across all trials.
    /// @param ovPlus     Overshoot on +direction trials only (motor-bias split).
    /// @param ovMinus    Overshoot on -direction trials only.
    /// @param dur        Turn wall-clock, ms.
    /// @param coast      Post-brake coast, deg.
    /// @param completed  Trials that produced a full measurement.
    /// @param inTol      Trials whose true miss stayed within `Config::TURN_TOLERANCE_DEG`.
    /// @param timedOut   Trials that ran to (near) `Config::TURN_TIMEOUT_MS`.
    /// @param signFlips  Trials whose measured rotation opposed the command (a sign fault).
    /// @param readFails  Trials skipped on a yaw-read failure.
    void turnTunePrintSummary(const Stat& ov, const Stat& ovPlus, const Stat& ovMinus,
                              const Stat& dur, const Stat& coast, uint16_t completed,
                              uint16_t inTol, uint16_t timedOut, uint16_t signFlips,
                              uint16_t readFails)
    {
        Serial.println(F("\n[TT] ================= SUMMARY ================="));
        Serial.print(F("[TT] commanded turn = "));
        Serial.print(TURN_TUNE_MAGNITUDE_DEG);
        Serial.print(F(" deg x "));
        Serial.print(TURN_TUNE_TRIALS);
        Serial.print(F(" trials (alternating direction);  completed="));
        Serial.print(completed);
        Serial.print(F("  readFails="));
        Serial.println(readFails);

        if (signFlips > 0)
        {
            Serial.print(F("[TT] *** WARNING: "));
            Serial.print(signFlips);
            Serial.println(F(" trial(s) rotated OPPOSITE the command. The turn command sign and "));
            Serial.println(F("[TT]     the IMU yaw sign disagree -- fix that FIRST (Config::IMU_YAW_SIGN "));
            Serial.println(F("[TT]     / TurnSignPolicy); every accuracy number below is meaningless "));
            Serial.println(F("[TT]     until it reads zero."));
        }

        if (completed == 0)
        {
            Serial.println(F("[TT] no completed trials -- nothing to summarise."));
            return;
        }

        Serial.println(F("[TT] accuracy (resting IMU residual, post-coast):"));
        Serial.print(F("[TT]   overshoot mean="));
        Serial.print(ov.mean(), 2);
        Serial.print(F(" deg (+ past target / - short),  mean|miss|="));
        Serial.print(ov.meanAbs(), 2);
        Serial.print(F(" deg,  stdev="));
        Serial.print(ov.stdev(), 2);
        Serial.print(F(" deg,  worst|miss|="));
        Serial.print(ov.maxAbs, 2);
        Serial.println(F(" deg"));

        Serial.print(F("[TT]   in tolerance (<= "));
        Serial.print(Config::TURN_TOLERANCE_DEG, 2);
        Serial.print(F(" deg): "));
        Serial.print(inTol);
        Serial.print('/');
        Serial.print(completed);
        Serial.print(F(" trials;  timeouts (~"));
        Serial.print(Config::TURN_TIMEOUT_MS);
        Serial.print(F(" ms): "));
        Serial.println(timedOut);

        Serial.print(F("[TT]   direction split (motor bias if these differ): +dir mean="));
        Serial.print(ovPlus.mean(), 2);
        Serial.print(F(" deg ("));
        Serial.print(ovPlus.n);
        Serial.print(F("),  -dir mean="));
        Serial.print(ovMinus.mean(), 2);
        Serial.print(F(" deg ("));
        Serial.print(ovMinus.n);
        Serial.println(F(")"));

        Serial.println(F("[TT] settle behaviour:"));
        Serial.print(F("[TT]   turn time mean="));
        Serial.print(dur.mean(), 0);
        Serial.print(F(" ms,  max="));
        Serial.print(dur.maxV, 0);
        Serial.println(F(" ms  (a slow max = hunting/oscillation; near-timeout = stall)"));
        Serial.print(F("[TT]   post-brake coast mean|.|="));
        Serial.print(coast.meanAbs(), 2);
        Serial.print(F(" deg,  worst="));
        Serial.print(coast.maxAbs, 2);
        Serial.println(F(" deg  (large coast = raise brake authority / lower creep)"));

        Serial.println(F("[TT] tuning hints:"));
        Serial.println(F("[TT]   overshoot mean > 0 and repeatable  -> lower TURN_KP or raise TURN_KD"));
        Serial.println(F("[TT]   overshoot mean < 0 (stops short)    -> raise TURN_CREEP_PWM / TURN_KP"));
        Serial.println(F("[TT]   turn-time max >> mean (hunting)      -> raise TURN_KD"));
        Serial.println(F("[TT]   +dir vs -dir split                  -> motor bias, not the gains"));
        Serial.println(F("[TT] ==========================================="));
    }
}    // namespace

void runTurnTune(void (*tickCb)())
{
    // Quiet the 500 ms EventTrace heartbeat for the whole run so the per-trial CSV is readable
    // (the exact clutter that made the T1 bench unreadable -- see the turn-PD branch). One-shot
    // sketch, so no restore.
    EventTrace.setHeartbeatEnabled(false);

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 turn tune (inoTest/turn_tune)                    #"));
    Serial.println(F("#  Closed-loop turnDeg() accuracy sweep for PD gain tuning  #"));
    Serial.println(F("############################################################"));
    turnTunePrintGains();

    turnTuneWaitForBtnUp(
        tickCb,
        F("[TT] place the chassis in a CLEAR CIRCLE (it spins in place both ways)."));

    Stat     ov, ovPlus, ovMinus, dur, coast;
    uint16_t completed = 0;
    uint16_t inTol     = 0;
    uint16_t timedOut  = 0;
    uint16_t signFlips = 0;
    uint16_t readFails = 0;
    float    tetherNet = 0.0f;    // signed net rotation so far, deg

    Serial.println(F("[TT] trial,dir,cmd,exitDelta,restDelta,overshoot,coast,durMs,flag"));

    for (uint16_t trial = 1; trial <= TURN_TUNE_TRIALS; ++trial)
    {
        const int16_t dir = (trial % 2 == 1) ? (int16_t) 1 : (int16_t) -1;
        const int16_t cmd = (int16_t) (dir * TURN_TUNE_MAGNITUDE_DEG);

        double yawBefore = 0.0;
        if (!turnTuneReadYaw(yawBefore, F("before")))
        {
            readFails += 1;
            turnTunePumpFor(tickCb, TURN_TUNE_GAP_MS);
            continue;
        }

        // Turns are non-faulting by design (ADR-068), but clear any latched fault defensively so
        // a stale one from a prior subsystem cannot freeze the queue mid-run.
        if (Drive.hasFault())
        {
            Drive.clearFault();
        }

        // The real PD turn: sync-blocking temporary (RAII -- emit as ONE statement, never a local;
        // a stored builder corrupts the queue). Blocks until the executor settles or times out,
        // spinning cooperativeTick() throughout via the registered tick callback.
        const uint32_t t0 = millis();
        Drive.turnDeg(cmd);
        const uint32_t t1 = millis();

        double     yawExit = 0.0;
        const bool okExit  = turnTuneReadYaw(yawExit, F("exit"));

        turnTunePumpFor(tickCb, TURN_TUNE_COAST_SETTLE_MS);

        double     yawRest = 0.0;
        const bool okRest  = turnTuneReadYaw(yawRest, F("rest"));

        if (!okExit || !okRest)
        {
            readFails += 1;
            turnTunePumpFor(tickCb, TURN_TUNE_GAP_MS);
            continue;
        }

        const float exitDelta = turnTuneWrap180((float) (yawExit - yawBefore));
        const float restDelta  = turnTuneWrap180((float) (yawRest - yawBefore));
        const float coastDeg   = turnTuneWrap180((float) (yawRest - yawExit));
        const float overshoot  = fabsf(restDelta) - fabsf((float) cmd);
        const float absErr     = fabsf((float) cmd - restDelta);
        const uint32_t durMs   = t1 - t0;

        const bool signFlip = (restDelta * (float) cmd) < 0.0f;
        const bool timeout  = durMs + TURN_TUNE_TIMEOUT_GUARD_MS >= Config::TURN_TIMEOUT_MS;
        const bool tol      = absErr <= Config::TURN_TOLERANCE_DEG;

        completed += 1;
        ov.add(overshoot);
        (dir > 0 ? ovPlus : ovMinus).add(overshoot);
        dur.add((float) durMs);
        coast.add(coastDeg);
        if (tol) { inTol += 1; }
        if (timeout) { timedOut += 1; }
        if (signFlip) { signFlips += 1; }
        tetherNet += restDelta;

        const __FlashStringHelper* flag = timeout    ? F("TIMEOUT")
                                          : signFlip  ? F("SIGN!")
                                          : tol       ? F("ok")
                                                      : F("OUT");

        Serial.print(F("[TT] "));
        Serial.print(trial);
        Serial.print(',');
        Serial.print(dir);
        Serial.print(',');
        Serial.print(cmd);
        Serial.print(',');
        Serial.print(exitDelta, 2);
        Serial.print(',');
        Serial.print(restDelta, 2);
        Serial.print(',');
        Serial.print(overshoot, 2);
        Serial.print(',');
        Serial.print(coastDeg, 2);
        Serial.print(',');
        Serial.print(durMs);
        Serial.print(',');
        Serial.println(flag);

        if (trial % TURN_TUNE_TETHER_PRINT_EVERY == 0)
        {
            Serial.print(F("[TT]   tether net = "));
            Serial.print(tetherNet, 1);
            Serial.print(F(" deg ("));
            Serial.print(tetherNet / 360.0f, 2);
            Serial.println(F(" rev) -- alternation keeps it near zero; watch for a runaway."));
        }

        turnTunePumpFor(tickCb, TURN_TUNE_GAP_MS);
    }

    turnTunePrintSummary(ov, ovPlus, ovMinus, dur, coast, completed, inTol, timedOut, signFlips,
                         readFails);
    Serial.println(F("[TT] done. press RESET to run again (e.g. after editing a TURN_* gain)."));
}
