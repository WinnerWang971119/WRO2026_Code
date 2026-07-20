/**
 * @file TurnAutotuneHarness.cpp
 * @brief On-robot Twiddle auto-tuner for the turn profile knobs
 *        (docs/plans/2026-07-21-turn-profile-controller.md, T5).
 * @author WRO 2026
 *
 * @details
 * The third and last of the turn bench sketches. `turn_probe` measures the open-loop plant
 * constants; `turn_tune` reports how accurately the REAL `Drive.turnDeg()` lands at a fixed knob
 * set; this sketch SEARCHES the knob set automatically, on the robot, with no laptop in the loop.
 *
 * ## Zero production weight — how
 *
 * The profile decision math already lives in header-only, parameter-driven policy functions
 * (`TurnProfilePolicy.h`: `turnPhaseFor`, `turnProfileSpeed`, `turnSettleNudge`; `TurnPDPolicy.h`:
 * `turnWrap180`, `turnSettleStep`, `turnSettled`; `TurnSignPolicy.h`: `turnWheelPattern`) precisely
 * so they can be host-unit-tested with every tunable injected. That same property lets THIS sketch
 * call the exact same functions with the knobs held in RUNTIME variables. `runOneTurn` below is a
 * faithful, line-for-line mirror of `Drive::_updateTurnProfile` (and its `_execTurnDeg` dispatch
 * seeding), with `Config::TURN_*` swapped for a `TurnGains` struct and `MiniR4.DriveDC.Move` driven
 * directly — the same raw vendor call the real executor issues. `Drive.cpp` is not touched: the
 * executor stays compile-time `constexpr` (ADR-078), and the search's winning vector is printed
 * for the operator to paste into `ConfigDrive.h` as the new default.
 *
 * ## Fidelity — why the tuned knobs transfer
 *
 * A replica is only useful if its optimum is production's optimum. Two things buy that:
 *   1. The LOAD-BEARING math is shared, not re-derived — `runOneTurn` calls the same policy
 *      symbols `_updateTurnProfile` calls, in the same operator order (phase-latch -> settle-exit
 *      -> timeout -> breakaway hand-off -> per-phase copysign command -> wheelPattern), with the
 *      same monotonic phase latch and the same absolute-error settle debounce.
 *   2. The tick PERIOD is still matched, even though the profile itself carries no derivative term
 *      and is immune to loop-rate by design (ADR-078): a slow or jittery tick still changes how
 *      many ticks a fixed-speed CREEP/SETTLE command gets to act before the next phase check,
 *      which shapes settle time and how often a candidate runs to `TURN_TIMEOUT_MS` — both scored
 *      terms. Each replica iteration pumps the full `cooperativeTick()` (Drive idle) THEN does the
 *      turn work, which sums to the same per-tick cost a real nav turn pays (in production the
 *      turn work runs INSIDE `Drive.update()` in place of that idle). The tick is the
 *      bench-measured ~22.66 ms either way.
 * The residual risk is closed by the operator's final step: flash `inoTest/turn_tune` with the
 * winning knobs and confirm the REAL `Drive.turnDeg()` residual matches what this search reported.
 *
 * ## Safety
 *
 * Every searched knob is bounded (`SPECS` below); the search never leaves the safe box, so a
 * candidate can only ever turn the chassis in place slowly-to-briskly, never drive it off the mat.
 * `BTN_DOWN` aborts between turns and between candidates (polled each tick, not interrupt-driven).
 * Iterations are capped, and every exit path stops the motors and brakes.
 *
 * @note NOT in CI — the `inoTest/` diagnostic sketches never are. `arduino-cli` is the only local
 *       check that this links. Build/flash/monitor via `uv run inoTest/turn_autotune/build.py`.
 *       Log STRAIGHT to a file, not `| tee`, or the R4 USB CDC backpressures and freezes mid-run.
 */

#include "TurnAutotuneHarness.h"

#include <MatrixMiniR4.h>

#include <math.h>      // fabsf, lroundf, copysignf, fmodf
#include <stdint.h>

#include "Config.h"
#include "EventTrace.h"
#include "TurnProfilePolicy.h"    // Pulls TurnPDPolicy.h transitively (turnWrap180, turnSettleStep,
                                  // turnSettled) -- same include the production Drive.cpp relies on.
#include "TurnSignPolicy.h"
#include "YawSource.h"

namespace
{
    // ---- Search protocol (edit + reflash to change it) ----------------------------------------

    /// @brief Commanded turn magnitude per scoring turn, degrees. Matches the plan's B3.2 turn.
    constexpr int16_t AUTOTUNE_MAGNITUDE_DEG = 90;

    /// @brief Turns per candidate score. MUST be even so the alternating +/- direction leaves zero
    /// net tether wind across a candidate (and so the whole run stays bounded). More turns average
    /// out IMU/plant noise so Twiddle chases signal not noise, at a linear time cost.
    constexpr uint16_t AUTOTUNE_TURNS_PER_CANDIDATE = 4;

    /// @brief Cap on Twiddle passes over the full parameter set. A pass with no improvement stops
    /// early; this only bounds a pathological non-convergence.
    constexpr uint16_t AUTOTUNE_MAX_ITERATIONS = 12;

    /// @brief Pause after a turn returns, before the settled read, so post-brake coast is captured.
    constexpr uint32_t AUTOTUNE_COAST_SETTLE_MS = 500;

    /// @brief Idle gap between turns; chassis fully still before the next dispatch.
    constexpr uint32_t AUTOTUNE_GAP_MS = 300;

    // ---- Cost weights (what "best" means) -----------------------------------------------------

    /// @brief Cost per degree of mean |resting residual| — the steady-state miss nav actually pays.
    /// Dominant term.
    constexpr float COST_W_RESIDUAL = 1.0f;
    /// @brief Cost per degree of mean peak transient overshoot (how far past target it swings).
    constexpr float COST_W_OVERSHOOT = 0.30f;
    /// @brief Cost per second of mean settle time — rewards a fast, non-hunting turn.
    constexpr float COST_W_TIME_PER_S = 0.50f;
    /// @brief Flat penalty added per turn that ran to timeout (never settled). Deliberately heavy:
    /// a gain set that stalls is disqualified, not merely down-ranked.
    constexpr float COST_TIMEOUT_PENALTY = 50.0f;
    /// @brief Cost returned when a whole candidate produced no valid turn (all yaw reads failed) —
    /// worse than any real gain set, so the search abandons it without crashing.
    constexpr float COST_UNSCORABLE = 1.0e6f;

    // ---- The searched parameters --------------------------------------------------------------

    /// @brief One searched knob: its label, Twiddle seed + initial step, safe bounds, and whether
    /// it is quantised to an integer PWM before use.
    struct ParamSpec
    {
        const char* name;
        float       init;
        float       step;
        float       lo;
        float       hi;
        bool        isInt;
    };

    enum ParamId
    {
        P_CREEP_WINDOW = 0,
        P_CREEP_PWM,
        P_SETTLE_KP,
        PARAM_COUNT
    };

    // Seeds are the knobs shipped in ConfigDrive.h (ADR-078). Bounds keep the search safe:
    //   - CREEP_WINDOW must sit strictly above `TURN_TOLERANCE_DEG` for a CREEP zone to exist at
    //     all (ConfigChecks enforces this on the production default; the search bound below is a
    //     looser sanity box, not a re-derivation of that assert).
    //   - CREEP_PWM's search floor sits above the kinetic sustain floor `turn_probe` measures — a
    //     candidate that drifts below the true floor simply hunts to timeout and disqualifies
    //     itself via the timeout penalty rather than being excluded here.
    //   - breakaway (`TURN_BREAKAWAY_PWM`) and cruise (`TURN_CRUISE_PWM`) are NOT searched here:
    //     `turn_probe` MEASURED the breakaway floor and cruise is the builder's own authority
    //     knob — both are grounded truth, not guesses. Add either to this table only if the
    //     closed-loop optimum is suspected to differ from that open-loop grounding.
    const ParamSpec SPECS[PARAM_COUNT] = {
        {"CREEP_WINDOW", (float) Config::TURN_CREEP_WINDOW_DEG, 2.0f, 10.0f, 40.0f, false},
        {"CREEP_PWM", (float) Config::TURN_CREEP_PWM, 3.0f, 30.0f, 60.0f, true},
        {"SETTLE_KP", (float) Config::TURN_SETTLE_KP, 0.5f, 1.0f, 10.0f, false},
    };

    static_assert(AUTOTUNE_TURNS_PER_CANDIDATE % 2 == 0 && AUTOTUNE_TURNS_PER_CANDIDATE >= 2,
                  "AUTOTUNE_TURNS_PER_CANDIDATE must be even (tether unwind) and >= 2.");
    static_assert(AUTOTUNE_MAGNITUDE_DEG > 0 && AUTOTUNE_MAGNITUDE_DEG <= 180,
                  "AUTOTUNE_MAGNITUDE_DEG must be in (0, 180]; a single turn never wraps.");

    /// @brief The full knob set for one turn: the searched params plus the fixed Config values the
    /// executor also reads. Mirrors every `Config::TURN_*` `_updateTurnProfile` touches.
    struct TurnGains
    {
        float    creepWindowDeg;
        int16_t  creepPwm;
        float    settleKp;
        int16_t  breakawayPwm;
        float    breakawayMotionDeg;
        int16_t  cruisePwm;
        float    toleranceDeg;
        uint8_t  settleSamples;
        uint32_t timeoutMs;
        uint8_t  motorMode;
    };

    /// @brief Build a `TurnGains` from a Twiddle parameter vector, filling the un-searched fields
    /// from `Config`.
    TurnGains gainsFromParams(const float p[PARAM_COUNT])
    {
        TurnGains g;
        g.creepWindowDeg     = p[P_CREEP_WINDOW];
        g.creepPwm           = (int16_t) lroundf(p[P_CREEP_PWM]);
        g.settleKp           = p[P_SETTLE_KP];
        g.breakawayPwm       = Config::TURN_BREAKAWAY_PWM;
        g.breakawayMotionDeg = (float) Config::TURN_BREAKAWAY_MOTION_DEG;
        g.cruisePwm          = Config::TURN_CRUISE_PWM;
        g.toleranceDeg       = (float) Config::TURN_TOLERANCE_DEG;
        g.settleSamples      = Config::TURN_SETTLE_SAMPLES;
        g.timeoutMs          = Config::TURN_TIMEOUT_MS;
        g.motorMode          = Config::TURN_MODE;
        return g;
    }

    // ---- Cooperative pump / heading / operator helpers ----------------------------------------

    /// @brief File-scope abort latch, set once BTN_DOWN is seen. Every loop checks it and unwinds.
    bool g_aborted = false;

    /// @brief Drive the caller's tick callback and yield (defensive fallback to `Drive` idle if
    /// none, though `turn_autotune.ino` always passes its real `cooperativeTick`).
    void autotunePump(void (*tickCb)())
    {
        if (tickCb != nullptr)
        {
            tickCb();
        }
        yield();
    }

    /// @brief Pump cooperative ticks for @p ms, checking the abort button throughout.
    void autotunePumpFor(void (*tickCb)(), uint32_t ms)
    {
        const uint32_t start = millis();
        while ((uint32_t) (millis() - start) < ms)
        {
            autotunePump(tickCb);
            if (MiniR4.BTN_DOWN.getState())
            {
                g_aborted = true;
            }
        }
    }

    /// @brief Blocking wait for a fresh BTN_UP rising edge, pumping cooperative ticks. Waits out an
    /// already-held press first.
    void autotuneWaitForBtnUp(void (*tickCb)(), const __FlashStringHelper* prompt)
    {
        Serial.println();
        Serial.println(prompt);
        Serial.println(F("[TA] press BTN_UP to start (BTN_DOWN aborts at any time)..."));
        while (MiniR4.BTN_UP.getState())
        {
            autotunePump(tickCb);
        }
        while (!MiniR4.BTN_UP.getState())
        {
            autotunePump(tickCb);
        }
        Serial.println(F("[TA] ...go."));
    }

    /// @brief Stop the motors and brake. Called on settle, timeout, and every abort/exit path.
    void autotuneStop()
    {
        MiniR4.DriveDC.Move(0, 0);
        MiniR4.DriveDC.brake(true);
    }

    // ---- Per-turn metrics + the faithful executor replica -------------------------------------

    /// @brief One turn's outcome. `valid == false` means a yaw read failed at dispatch and every
    /// other field is meaningless.
    struct TurnMetrics
    {
        bool     valid            = false;
        float    restDeltaDeg     = 0.0f;    // signed measured rotation, post coast-settle read
        float    peakOvershootDeg = 0.0f;    // max transient PAST target during the turn (>= 0)
        uint32_t durMs            = 0;       // dispatch -> settle/timeout
        bool     timedOut         = false;
    };

    /// @brief Run ONE in-place turn of @p deltaDeg through a runtime-knob replica of
    /// `Drive::_updateTurnProfile`, then read the settled rotation. Drives `MiniR4.DriveDC.Move`
    /// directly; `Drive` is not involved.
    ///
    /// Line-for-line faithful to the production executor — see the field-by-field seeding in
    /// `Drive.cpp::_execTurnDeg` and the per-tick loop in `_updateTurnProfile`. The ONLY departures
    /// are: the searched knobs come from @p g not `Config`, and the tick is pumped explicitly at
    /// the top of each iteration (in production the loop body IS `Drive.update()`, called once per
    /// tick by the cooperative scheduler).
    TurnMetrics runOneTurn(const TurnGains& g, int16_t deltaDeg, void (*tickCb)())
    {
        TurnMetrics m;

        // --- dispatch seeding (mirror _execTurnDeg) ---
        double curYaw = 0.0;
        if (!YawSource.readYawDeg(curYaw))
        {
            Serial.println(F("[TA] yaw read FAILED at dispatch -- turn skipped"));
            return m;    // invalid
        }
        const float yawBefore = (float) curYaw;
        const float target    = fmodf(yawBefore + (float) deltaDeg, 360.0f);
        const float deltaNorm = turnWrap180(target - yawBefore);

        // No-op short-circuit (cannot fire for a fresh 90 deg command; mirrored for fidelity).
        if (fabsf(deltaNorm) <= g.toleranceDeg)
        {
            m.valid        = true;
            m.restDeltaDeg = 0.0f;
            return m;
        }

        const int8_t sign        = (deltaNorm > 0.0f) ? (int8_t) +1 : (int8_t) -1;
        TurnPhase    phase       = TurnPhase::CRUISE;    // monotonic phase latch starts wide.
        uint8_t      inBandCount = 0;
        bool         brokeAway   = false;
        const float  startYaw    = yawBefore;
        const uint32_t startMs   = millis();
        float        minErrProg  = fabsf(deltaNorm);            // most-negative errProg -> overshoot

        // --- per-tick loop (mirror _updateTurnProfile) ---
        for (;;)
        {
            autotunePump(tickCb);    // full cooperativeTick cost -> production-representative tick
            if (MiniR4.BTN_DOWN.getState())
            {
                g_aborted = true;
            }
            if (g_aborted)
            {
                autotuneStop();
                m.durMs = millis() - startMs;
                m.valid = false;    // aborted mid-turn: do not score it
                return m;
            }

            double y = 0.0;
            if (!YawSource.readYawDeg(y))
            {
                // Transient read miss: hold the last Move and try again next tick. A SUSTAINED
                // failure falls through to the timeout below (bounded), same net effect as the
                // production watchdog without duplicating its state here.
                continue;
            }
            const float yaw     = (float) y;
            const float errProg = (float) sign * turnWrap180(target - yaw);
            const float absErr  = fabsf(errProg);
            if (errProg < minErrProg)
            {
                minErrProg = errProg;
            }

            // Monotonic phase latch (ADR-078): the FSM only ever advances CRUISE -> CREEP ->
            // SETTLE; a later read whose absErr swings back above the window cannot regress it.
            const TurnPhase natural = turnPhaseFor(absErr, g.creepWindowDeg, g.toleranceDeg);
            if (natural > phase)
            {
                phase = natural;
            }

            // Settle exit — only inside the latched SETTLE phase, same debounce production uses.
            if (phase == TurnPhase::SETTLE)
            {
                inBandCount = turnSettleStep(absErr, g.toleranceDeg, inBandCount);
                if (turnSettled(inBandCount, g.settleSamples))
                {
                    autotuneStop();
                    m.durMs    = millis() - startMs;
                    m.timedOut = false;
                    break;
                }
            }

            if ((uint32_t) (millis() - startMs) >= g.timeoutMs)
            {
                autotuneStop();
                m.durMs    = millis() - startMs;
                m.timedOut = true;
                break;
            }

            if (!brokeAway &&
                fabsf(turnWrap180(yaw - startYaw)) >= g.breakawayMotionDeg)
            {
                brokeAway = true;
            }

            // Per-phase command — ALWAYS copysignf(speed, errProg), matching _updateTurnProfile:
            //   - not yet broken away -> the static breakaway kick;
            //   - SETTLE              -> the terminal P-nudge (already signed + gated + clamped);
            //   - CRUISE / CREEP      -> the fixed profile magnitude, signed here by copysignf.
            float cmd;
            if (!brokeAway)
            {
                cmd = copysignf((float) g.breakawayPwm, errProg);
            }
            else if (phase == TurnPhase::SETTLE)
            {
                cmd = turnSettleNudge(errProg, g.toleranceDeg, g.settleKp, g.creepPwm);
            }
            else
            {
                cmd = copysignf((float) turnProfileSpeed(phase, g.cruisePwm, g.creepPwm), errProg);
            }

            const TurnWheels wheels = turnWheelPattern(sign, (int16_t) lroundf(cmd), g.motorMode);
            MiniR4.DriveDC.Move(wheels.left, wheels.right);
        }

        // --- settled read (post-brake coast included) ---
        autotunePumpFor(tickCb, AUTOTUNE_COAST_SETTLE_MS);
        double yawRest = 0.0;
        if (!YawSource.readYawDeg(yawRest))
        {
            Serial.println(F("[TA] yaw read FAILED at rest -- turn skipped"));
            return m;    // invalid (valid still false)
        }

        m.restDeltaDeg     = turnWrap180((float) yawRest - yawBefore);
        m.peakOvershootDeg = (minErrProg < 0.0f) ? -minErrProg : 0.0f;
        m.valid            = true;
        return m;
    }

    // ---- Candidate scoring --------------------------------------------------------------------

    /// @brief Score a gain vector: run `AUTOTUNE_TURNS_PER_CANDIDATE` alternating turns and reduce
    /// them to a single cost. Lower is better. Returns `COST_UNSCORABLE` if no turn scored.
    float evaluateCandidate(const float p[PARAM_COUNT], void (*tickCb)())
    {
        const TurnGains g = gainsFromParams(p);

        uint16_t valid    = 0;
        float    sumAbs   = 0.0f;    // |cmd - restDelta|
        float    sumOver  = 0.0f;    // peak transient overshoot
        float    sumSec   = 0.0f;    // settle time
        uint16_t timeouts = 0;

        for (uint16_t t = 1; t <= AUTOTUNE_TURNS_PER_CANDIDATE; ++t)
        {
            if (g_aborted)
            {
                break;
            }
            const int16_t dir = (t % 2 == 1) ? (int16_t) 1 : (int16_t) -1;
            const int16_t cmd = (int16_t) (dir * AUTOTUNE_MAGNITUDE_DEG);

            const TurnMetrics m = runOneTurn(g, cmd, tickCb);
            if (m.valid)
            {
                valid += 1;
                sumAbs += fabsf((float) cmd - m.restDeltaDeg);
                sumOver += m.peakOvershootDeg;
                sumSec += (float) m.durMs / 1000.0f;
                if (m.timedOut)
                {
                    timeouts += 1;
                }
            }
            autotunePumpFor(tickCb, AUTOTUNE_GAP_MS);
        }

        if (valid == 0)
        {
            return COST_UNSCORABLE;
        }

        const float invN = 1.0f / (float) valid;
        return COST_W_RESIDUAL * (sumAbs * invN) + COST_W_OVERSHOOT * (sumOver * invN) +
               COST_W_TIME_PER_S * (sumSec * invN) + COST_TIMEOUT_PENALTY * (float) timeouts;
    }

    // ---- Twiddle search -----------------------------------------------------------------------

    float clampParam(float v, const ParamSpec& s)
    {
        if (v < s.lo) { return s.lo; }
        if (v > s.hi) { return s.hi; }
        return v;
    }

    void printParams(const float p[PARAM_COUNT], float cost)
    {
        Serial.print(F("[TA]   "));
        for (uint8_t i = 0; i < PARAM_COUNT; ++i)
        {
            Serial.print(SPECS[i].name);
            Serial.print('=');
            Serial.print(p[i], SPECS[i].isInt ? 0 : 4);
            Serial.print(' ');
        }
        Serial.print(F(" -> cost="));
        Serial.println(cost, 3);
    }
}    // namespace

void runTurnAutotune(void (*tickCb)())
{
    // Quiet the 500 ms EventTrace heartbeat so the search log is readable (one-shot; no restore).
    EventTrace.setHeartbeatEnabled(false);
    g_aborted = false;

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 turn autotune (inoTest/turn_autotune)           #"));
    Serial.println(F("#  On-robot Twiddle search for the turn profile knobs (T5) #"));
    Serial.println(F("############################################################"));
    Serial.print(F("[TA] searching "));
    Serial.print((int) PARAM_COUNT);
    Serial.print(F(" params x "));
    Serial.print(AUTOTUNE_TURNS_PER_CANDIDATE);
    Serial.print(F(" turns/candidate, magnitude "));
    Serial.print(AUTOTUNE_MAGNITUDE_DEG);
    Serial.println(F(" deg. breakaway/cruise held at bench-measured/builder values."));

    autotuneWaitForBtnUp(
        tickCb, F("[TA] place the chassis in a CLEAR CIRCLE (it spins in place both ways)."));

    float p[PARAM_COUNT];
    float d[PARAM_COUNT];
    for (uint8_t i = 0; i < PARAM_COUNT; ++i)
    {
        p[i] = clampParam(SPECS[i].init, SPECS[i]);
        d[i] = SPECS[i].step;
    }

    Serial.println(F("[TA] scoring the seed gain set..."));
    float bestCost = evaluateCandidate(p, tickCb);
    Serial.print(F("[TA] seed"));
    printParams(p, bestCost);

    for (uint16_t iter = 1; iter <= AUTOTUNE_MAX_ITERATIONS && !g_aborted; ++iter)
    {
        Serial.print(F("\n[TA] --- iteration "));
        Serial.print(iter);
        Serial.print('/');
        Serial.print(AUTOTUNE_MAX_ITERATIONS);
        Serial.println(F(" ---"));

        bool improved = false;
        for (uint8_t i = 0; i < PARAM_COUNT && !g_aborted; ++i)
        {
            const float saved = p[i];

            // try + d[i]
            p[i]        = clampParam(saved + d[i], SPECS[i]);
            float cost  = evaluateCandidate(p, tickCb);
            if (!g_aborted && cost < bestCost)
            {
                bestCost = cost;
                d[i] *= 1.1f;
                improved = true;
                Serial.print(SPECS[i].name);
                Serial.print(F(" + accepted"));
                printParams(p, cost);
                continue;
            }

            // try - d[i]
            p[i]  = clampParam(saved - d[i], SPECS[i]);
            cost  = evaluateCandidate(p, tickCb);
            if (!g_aborted && cost < bestCost)
            {
                bestCost = cost;
                d[i] *= 1.1f;
                improved = true;
                Serial.print(SPECS[i].name);
                Serial.print(F(" - accepted"));
                printParams(p, cost);
                continue;
            }

            // neither helped: restore and shrink the step
            p[i] = saved;
            d[i] *= 0.9f;
            Serial.print(F("[TA]   "));
            Serial.print(SPECS[i].name);
            Serial.print(F(" held (step -> "));
            Serial.print(d[i], SPECS[i].isInt ? 1 : 4);
            Serial.println(F(")"));
        }

        Serial.print(F("[TA] iteration best cost="));
        Serial.print(bestCost, 3);
        printParams(p, bestCost);

        if (!improved)
        {
            Serial.println(F("[TA] converged: a full pass improved nothing."));
            break;
        }
    }

    autotuneStop();

    Serial.println(F("\n[TA] ================= RESULT ================="));
    if (g_aborted)
    {
        Serial.println(F("[TA] ABORTED by BTN_DOWN -- best knobs found SO FAR:"));
    }
    else
    {
        Serial.println(F("[TA] search finished -- best knobs:"));
    }
    Serial.print(F("[TA]   final cost="));
    Serial.println(bestCost, 3);
    Serial.println(F("[TA] paste these into WRO2026/config/ConfigDrive.h (the searched knobs):"));
    Serial.print(F("[TA]   TURN_CREEP_WINDOW_DEG = "));
    Serial.print(p[P_CREEP_WINDOW], 4);
    Serial.println(F("f"));
    Serial.print(F("[TA]   TURN_CREEP_PWM        = "));
    Serial.println((int16_t) lroundf(p[P_CREEP_PWM]));
    Serial.print(F("[TA]   TURN_SETTLE_KP        = "));
    Serial.print(p[P_SETTLE_KP], 4);
    Serial.println(F("f"));
    Serial.println(F("[TA] then VALIDATE on the real Drive.turnDeg() path: flash inoTest/turn_tune"));
    Serial.println(F("[TA] with these values and confirm its residual matches this search."));
    Serial.println(F("[TA] press RESET to run again."));
    Serial.println(F("[TA] ==========================================="));
}
