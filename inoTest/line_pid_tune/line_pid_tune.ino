/**
 * @file line_pid_tune.ino
 * @brief On-robot auto-tuner for the LineFollower P+D gains — FRONT and REAR sets in one session.
 * @author WRO 2026
 *
 * @details
 * A hands-off, OLED-guided gain search you run entirely from the robot's two buttons — no laptop
 * needed while tuning. It runs TWO independent twiddle (coordinate-descent) optimisers at once:
 * forward runs (front board) tune `Config::LINE_KP` / `LINE_KD`, reverse runs (rear board) tune
 * `Config::LINE_KP_REAR` / `LINE_KD_REAR` (ADR-066). The two boards sit at different distances
 * from the axle (12.02 cm vs 7.68 cm), so the same gains steer differently — each direction needs
 * its own tune, and this finds both in one ping-pong session.
 *
 * WHY replicate the controller here instead of driving the real LineFollower:
 *   `LineFollower` reads its gains as compile-time `constexpr` — there is no runtime setter — so
 *   it cannot be re-tuned live. This sketch reproduces `LineFollower.cpp`'s online P+D law EXACTLY
 *   (same error source, ±`LINE_MAX_CORR` clamp, `L=cruise+corr`/`R=cruise-corr` mix, cruise negated
 *   in reverse, rear board selected via `setReverseActive`, `Drive.setSpeed` deadband) so the gains
 *   it finds transfer 1:1 into `Config`.
 *   @warning If `LineFollower.cpp`'s control law changes, mirror it in `stepControl()` below.
 *
 * ------------------------------------------------------------------------------------------
 * OPERATOR FLOW (parallel ping-pong)
 * ------------------------------------------------------------------------------------------
 * The OLED tells you, before every run, how to place the robot AND which way it will drive. Each
 * optimiser's evaluation is the three placements below; the schedule interleaves forward+reverse
 * so the robot shuttles on ONE 50 cm stretch and both sets advance every round (6 runs/round with
 * a rear board, 3 forward-only if one board).
 *   1. VERY OFF   — start ~45-70 deg across the line (big recovery).
 *   2. A LITTLE OFF — a small angle / small lateral offset.
 *   3. ALMOST ON  — nearly centred; tests steady-state wobble / overshoot.
 * Sequence per run:
 *   - OLED shows "<FWD|REV> <placement>" + the gains about to be tested for THAT direction.
 *   - Place the robot that way on a straight 50 cm+ line (a forward run ends downrange; the paired
 *     reverse run drives it back — set the next placement at whichever end it now sits).
 *   - Press BTN_UP -> drives 50 cm, scoring as it goes, then brakes.
 *   - Each direction converges on its own; when BOTH are done the OLED shows "TUNED" + both sets
 *     and Serial prints two paste blocks (front + rear).
 * Buttons:
 *   - BTN_UP   — start the next guided run (or restart after TUNED).
 *   - BTN_DOWN — finish now and show the best-so-far for both directions.
 *
 * @note Ki is held at 0 (the P+D law has no integral term). Only Kp and Kd are searched.
 * @note SAFETY: every run self-terminates on distance, a time watchdog, OR a sustained lost-line
 *       (so a bad gain set cannot drive the robot off the table). An aborted run scores a large
 *       penalty so the optimiser avoids that region.
 *
 * Build / flash / monitor:
 * @code
 *   uv run inoTest/line_pid_tune/build.py
 * @endcode
 *
 * @note NOT in CI — the `inoTest/` diagnostic sketches never are. `arduino-cli` is the only local
 *       check that this links.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "Drive.h"
#include "WROLineArray.h"

namespace
{
    // -------------------------------------------------------------------------------------
    // Fixed run conditions (flat, no T-shape, cruise 70).
    // -------------------------------------------------------------------------------------
    constexpr float   RUN_DISTANCE_CM = 50.0f;                ///< Distance per scored run, cm.
    constexpr int16_t CRUISE_PWM      = Config::SPEED_NORMAL; ///< Flat forward cruise PWM (70).

    // -------------------------------------------------------------------------------------
    // Cost weights. cost = MSE_W*meanSqErr + OSC_W*oscCount + LOST_W*lostTicks (+ ABORT if cut short).
    // All three components are printed per run so a score is always explainable.
    // -------------------------------------------------------------------------------------
    constexpr float MSE_WEIGHT    = 50.0f;    ///< Penalise mean-squared tracking error (Kp).
    constexpr float OSC_WEIGHT    = 3.0f;     ///< Penalise oscillation / overshoot (Kd).
    constexpr float LOST_WEIGHT   = 10.0f;    ///< Penalise ticks spent off the line.
    constexpr float ABORT_PENALTY = 5000.0f;  ///< Dominant penalty for a run that never finished.
    constexpr float OSC_HYSTERESIS = 0.4f;    ///< err band before a sign-flip counts as oscillation.

    // -------------------------------------------------------------------------------------
    // Twiddle optimiser knobs (shared by both direction searches).
    // -------------------------------------------------------------------------------------
    constexpr float KP_STEP_INIT = 1.5f;
    constexpr float KD_STEP_INIT = 0.5f;
    constexpr float STEP_GROW    = 1.1f;
    constexpr float STEP_SHRINK  = 0.9f;
    constexpr float CONVERGE_TOL = 0.25f;

    // -------------------------------------------------------------------------------------
    // Safety watchdogs.
    // -------------------------------------------------------------------------------------
    constexpr uint32_t RUN_TIMEOUT_MS = 6000;
    constexpr uint32_t LOST_ABORT_MS  = 1200;

    /// @brief Encoder-degrees -> cm using the configured drive-wheel diameter.
    constexpr float CM_PER_ENC_DEG = 3.14159265f * Config::WHEEL_DIAM_MM / 10.0f / 360.0f;

    // Direction index into the per-direction optimiser arrays.
    constexpr uint8_t FRONT = 0;    ///< Forward follow (front board).
    constexpr uint8_t REAR  = 1;    ///< Reverse follow (rear board).

    // =====================================================================================
    // Guided placement regimes.
    // =====================================================================================
    enum class Regime : uint8_t { VERY_OFF = 0, LITTLE_OFF = 1, ALMOST_ON = 2, COUNT = 3 };

    const __FlashStringHelper* regimeInstruction(Regime r)
    {
        switch (r)
        {
            case Regime::VERY_OFF:   return F("VERY OFF ~60deg");
            case Regime::LITTLE_OFF: return F("A LITTLE OFF");
            case Regime::ALMOST_ON:  return F("ALMOST ON LINE");
            default:                 return F("?");
        }
    }

    // =====================================================================================
    // Twiddle optimiser phase (per direction).
    // =====================================================================================
    enum class Phase : uint8_t { BASELINE, PLUS, MINUS, DONE };

    // ---------------- Per-direction optimiser state. [dir][coord]; coord 0 = Kp, 1 = Kd -------
    float    g[2][2]     = {{Config::LINE_KP, Config::LINE_KD},
                            {Config::LINE_KP_REAR, Config::LINE_KD_REAR}};
    float    stepSz[2][2] = {{KP_STEP_INIT, KD_STEP_INIT}, {KP_STEP_INIT, KD_STEP_INIT}};
    float    bestG[2][2]  = {{Config::LINE_KP, Config::LINE_KD},
                             {Config::LINE_KP_REAR, Config::LINE_KD_REAR}};
    float    bestCost[2]  = {1.0e30f, 1.0e30f};
    uint8_t  coordSel[2]  = {0, 0};
    Phase    phase[2]     = {Phase::BASELINE, Phase::BASELINE};
    float    evalCostSum[2] = {0.0f, 0.0f};
    uint32_t evalNumber[2]  = {0, 0};

    // ---------------- Schedule / run state ----------
    bool    canReverse   = false;
    uint8_t stepsPerEval = 3;
    uint8_t stepIdx      = 0;
    uint8_t curDir       = FRONT;

    bool     running     = false;
    uint32_t runStartMs  = 0;
    int32_t  startEncL   = 0;
    int32_t  startEncR   = 0;
    float    lastErr     = 0.0f;
    bool     seedErr     = true;
    double   sumSqErr    = 0.0;
    uint32_t onlineTicks = 0;
    uint32_t lostTicks   = 0;
    uint32_t oscCount    = 0;
    int8_t   lastSign    = 0;
    bool     lineWasLost = false;
    uint32_t lostSinceMs = 0;

    bool prevUp   = false;
    bool prevDown = false;

    /// @brief Placement regime for the current schedule step (2 steps per regime when ping-ponging).
    Regime currentRegime()
    {
        return canReverse ? static_cast<Regime>(stepIdx / 2) : static_cast<Regime>(stepIdx);
    }

    /// @brief Travel direction for the current step — reverse on odd steps when the rear board is up.
    bool currentReverse()
    {
        return canReverse && ((stepIdx & 1u) != 0u);
    }

    /// @brief Optimiser index for the current step.
    uint8_t currentDir() { return currentReverse() ? REAR : FRONT; }

    // =====================================================================================
    // OLED — a purpose-built 4-line readout (128x32, size 1 -> ~21 chars x 4 lines).
    // =====================================================================================
    String gainsLine(const char* tag, float kp, float kd)
    {
        return String(tag) + "Kp" + String(kp, 2) + " Kd" + String(kd, 2);
    }

    void showWaiting()
    {
        const uint8_t d = currentDir();
        MiniR4.OLED.clearDisplay();
        MiniR4.OLED.setTextSize(1);
        MiniR4.OLED.setTextColor(WHITE);
        MiniR4.OLED.setCursor(0, 0);
        MiniR4.OLED.print(currentReverse() ? F("REV ") : F("FWD "));
        MiniR4.OLED.println(regimeInstruction(currentRegime()));
        MiniR4.OLED.setCursor(0, 8);
        MiniR4.OLED.println(gainsLine("test ", g[d][0], g[d][1]));
        MiniR4.OLED.setCursor(0, 16);
        MiniR4.OLED.println(String("run ") + String((int) stepIdx + 1) + "/"
                            + String((int) stepsPerEval) + "  press UP");
        MiniR4.OLED.setCursor(0, 24);
        MiniR4.OLED.print(phase[d] == Phase::DONE ? F("done ") : F("best "));
        MiniR4.OLED.println(gainsLine("", bestG[d][0], bestG[d][1]));
        MiniR4.OLED.display();
    }

    void showRunning()
    {
        const uint8_t d = currentDir();
        MiniR4.OLED.clearDisplay();
        MiniR4.OLED.setTextSize(1);
        MiniR4.OLED.setTextColor(WHITE);
        MiniR4.OLED.setCursor(0, 0);
        MiniR4.OLED.println(currentReverse() ? F("RUNNING <- REV") : F("RUNNING -> FWD"));
        MiniR4.OLED.setCursor(0, 8);
        MiniR4.OLED.println(gainsLine("test ", g[d][0], g[d][1]));
        MiniR4.OLED.setCursor(0, 16);
        MiniR4.OLED.println(String("run ") + String((int) stepIdx + 1) + "/"
                            + String((int) stepsPerEval));
        MiniR4.OLED.setCursor(0, 24);
        MiniR4.OLED.println(gainsLine("best ", bestG[d][0], bestG[d][1]));
        MiniR4.OLED.display();
    }

    void showDone()
    {
        MiniR4.OLED.clearDisplay();
        MiniR4.OLED.setTextSize(1);
        MiniR4.OLED.setTextColor(WHITE);
        MiniR4.OLED.setCursor(0, 0);
        MiniR4.OLED.println(F("TUNED  (UP=redo)"));
        MiniR4.OLED.setCursor(0, 8);
        MiniR4.OLED.println(gainsLine("F ", bestG[FRONT][0], bestG[FRONT][1]));
        MiniR4.OLED.setCursor(0, 16);
        if (canReverse)
        {
            MiniR4.OLED.println(gainsLine("R ", bestG[REAR][0], bestG[REAR][1]));
        }
        else
        {
            MiniR4.OLED.println(F("R (1 board: n/a)"));
        }
        MiniR4.OLED.setCursor(0, 24);
        MiniR4.OLED.println(F("see serial to paste"));
        MiniR4.OLED.display();
    }

    // =====================================================================================
    // Control law — EXACT mirror of LineFollower.cpp online P+D block, with per-direction gains.
    // =====================================================================================
    void stepControl()
    {
        // Cruise negated in reverse (LineFollower.cpp: `if (_reverse) cruise = -cruise`). The rear
        // board is already selected via setReverseActive() at run start.
        const int16_t cruise = currentReverse() ? (int16_t) -CRUISE_PWM : CRUISE_PWM;
        const bool    online = LineArray.isOnline();

        if (!online)
        {
            lastErr = 0.0f;
            seedErr = true;
            Drive.setSpeed(cruise, cruise);
            ++lostTicks;
            if (!lineWasLost)
            {
                lineWasLost = true;
                lostSinceMs = millis();
            }
            return;
        }
        lineWasLost = false;

        const float err = LineArray.getError();
        if (seedErr)
        {
            lastErr = err;
            seedErr = false;
        }
        const float dErr = err - lastErr;
        lastErr          = err;

        // Per-direction gains (ADR-066): front set forward, rear set reverse.
        float corr = (g[curDir][0] * err) + (g[curDir][1] * dErr);
        if (corr > Config::LINE_MAX_CORR)
        {
            corr = Config::LINE_MAX_CORR;
        }
        else if (corr < -Config::LINE_MAX_CORR)
        {
            corr = -Config::LINE_MAX_CORR;
        }

        const int16_t L = (int16_t) (cruise + corr);
        const int16_t R = (int16_t) (cruise - corr);
        Drive.setSpeed(L, R);

        sumSqErr += (double) err * (double) err;
        ++onlineTicks;

        if (err > OSC_HYSTERESIS || err < -OSC_HYSTERESIS)
        {
            const int8_t s = (err > 0.0f) ? 1 : -1;
            if (lastSign != 0 && s != lastSign)
            {
                ++oscCount;
            }
            lastSign = s;
        }
    }

    // =====================================================================================
    // Run lifecycle.
    // =====================================================================================
    void startRun()
    {
        curDir = currentDir();
        // Select the leading board for this direction BEFORE the first control tick.
        LineArray.setReverseActive(currentReverse());

        startEncL   = Drive.getEncL();
        startEncR   = Drive.getEncR();
        runStartMs  = millis();
        lastErr     = 0.0f;
        seedErr     = true;
        sumSqErr    = 0.0;
        onlineTicks = 0;
        lostTicks   = 0;
        oscCount    = 0;
        lastSign    = 0;
        lineWasLost = false;
        lostSinceMs = 0;
        running     = true;

        Serial.print(F("[tune] RUN start  dir="));
        Serial.print(currentReverse() ? F("REV") : F("FWD"));
        Serial.print(F("  regime="));
        Serial.print(regimeInstruction(currentRegime()));
        Serial.print(F("  Kp="));
        Serial.print(g[curDir][0], 3);
        Serial.print(F("  Kd="));
        Serial.println(g[curDir][1], 3);
        showRunning();
    }

    float travelledCm()
    {
        const int32_t dL = Drive.getEncL() - startEncL;
        const int32_t dR = Drive.getEncR() - startEncR;
        return ((float) (abs(dL) + abs(dR)) * 0.5f) * CM_PER_ENC_DEG;
    }

    // Forward declarations — twiddle advance + finalisation.
    void advanceOptimizer(uint8_t dir, float cost);
    bool bothConverged();
    void printFinalSummary();

    void finishRun(bool aborted, const __FlashStringHelper* reason)
    {
        Drive.stop(true);
        running = false;

        const float meanSq = (onlineTicks > 0) ? (float) (sumSqErr / onlineTicks) : 10.0f;
        float cost = (MSE_WEIGHT * meanSq) + (OSC_WEIGHT * (float) oscCount)
                   + (LOST_WEIGHT * (float) lostTicks);
        if (aborted)
        {
            cost += ABORT_PENALTY;
        }

        Serial.print(F("[tune] RUN done   cost="));
        Serial.print(cost, 1);
        Serial.print(F("  ( meanSqErr="));
        Serial.print(meanSq, 3);
        Serial.print(F(" osc="));
        Serial.print(oscCount);
        Serial.print(F(" lostTicks="));
        Serial.print(lostTicks);
        Serial.print(F(" ticks="));
        Serial.print(onlineTicks);
        if (aborted)
        {
            Serial.print(F(" ABORTED:"));
            Serial.print(reason);
        }
        Serial.println(F(" )"));

        evalCostSum[curDir] += cost;
        ++stepIdx;

        if (stepIdx < stepsPerEval)
        {
            showWaiting();
            return;
        }

        // End of a full round: every active direction has completed its 3-placement evaluation.
        stepIdx = 0;
        const uint8_t nd = canReverse ? 2 : 1;
        for (uint8_t d = 0; d < nd; ++d)
        {
            advanceOptimizer(d, evalCostSum[d]);
            evalCostSum[d] = 0.0f;
            ++evalNumber[d];
        }

        if (bothConverged())
        {
            printFinalSummary();
            showDone();
        }
        else
        {
            showWaiting();
        }
    }

    // =====================================================================================
    // Twiddle advance (per direction).
    // =====================================================================================
    void announceCandidate(uint8_t dir)
    {
        Serial.print(F("[tune] next "));
        Serial.print(dir == REAR ? F("REAR") : F("FRONT"));
        Serial.print(F("  Kp="));
        Serial.print(g[dir][0], 3);
        Serial.print(F("  Kd="));
        Serial.print(g[dir][1], 3);
        Serial.print(F("  (stepKp="));
        Serial.print(stepSz[dir][0], 3);
        Serial.print(F(" stepKd="));
        Serial.print(stepSz[dir][1], 3);
        Serial.println(F(")"));
    }

    void markConverged(uint8_t dir)
    {
        phase[dir] = Phase::DONE;
        g[dir][0]  = bestG[dir][0];
        g[dir][1]  = bestG[dir][1];
        Serial.print(F("[tune] "));
        Serial.print(dir == REAR ? F("REAR") : F("FRONT"));
        Serial.print(F(" CONVERGED  Kp="));
        Serial.print(bestG[dir][0], 3);
        Serial.print(F("  Kd="));
        Serial.print(bestG[dir][1], 3);
        Serial.print(F("  cost="));
        Serial.println(bestCost[dir], 1);
    }

    void advanceCoord(uint8_t dir)
    {
        coordSel[dir] = (coordSel[dir] + 1) % 2;
        if (coordSel[dir] == 0)
        {
            if ((stepSz[dir][0] + stepSz[dir][1]) < CONVERGE_TOL)
            {
                markConverged(dir);
                return;
            }
        }
        g[dir][coordSel[dir]] += stepSz[dir][coordSel[dir]];    // begin a PLUS probe
        phase[dir] = Phase::PLUS;
        announceCandidate(dir);
    }

    void advanceOptimizer(uint8_t dir, float cost)
    {
        const uint8_t c = coordSel[dir];
        switch (phase[dir])
        {
            case Phase::BASELINE:
                bestCost[dir] = cost;
                bestG[dir][0] = g[dir][0];
                bestG[dir][1] = g[dir][1];
                Serial.print(F("[tune] "));
                Serial.print(dir == REAR ? F("REAR") : F("FRONT"));
                Serial.print(F(" baseline cost="));
                Serial.println(cost, 2);
                coordSel[dir] = 0;
                g[dir][0]    += stepSz[dir][0];    // first PLUS probe (Kp up)
                phase[dir]    = Phase::PLUS;
                announceCandidate(dir);
                break;

            case Phase::PLUS:
                if (cost < bestCost[dir])
                {
                    bestCost[dir] = cost;
                    bestG[dir][0] = g[dir][0];
                    bestG[dir][1] = g[dir][1];
                    stepSz[dir][c] *= STEP_GROW;
                    advanceCoord(dir);
                }
                else
                {
                    g[dir][c] -= 2.0f * stepSz[dir][c];    // try the other direction
                    phase[dir] = Phase::MINUS;
                    announceCandidate(dir);
                }
                break;

            case Phase::MINUS:
                if (cost < bestCost[dir])
                {
                    bestCost[dir] = cost;
                    bestG[dir][0] = g[dir][0];
                    bestG[dir][1] = g[dir][1];
                    stepSz[dir][c] *= STEP_GROW;
                }
                else
                {
                    g[dir][c]     += stepSz[dir][c];    // revert to pre-probe value
                    stepSz[dir][c] *= STEP_SHRINK;
                }
                advanceCoord(dir);
                break;

            case Phase::DONE:
            default:
                break;    // already converged — this direction's runs just reposition the chassis.
        }
    }

    bool bothConverged()
    {
        return phase[FRONT] == Phase::DONE && phase[REAR] == Phase::DONE;
    }

    void printFinalSummary()
    {
        Serial.println();
        Serial.println(F("========== TUNING COMPLETE =========="));
        Serial.println(F("Paste into WRO2026/config/ConfigLine.h:"));
        Serial.print(F("    constexpr float LINE_KP = "));
        Serial.print(bestG[FRONT][0], 3);
        Serial.println(F("f;"));
        Serial.print(F("    constexpr float LINE_KD = "));
        Serial.print(bestG[FRONT][1], 3);
        Serial.println(F("f;"));
        if (canReverse)
        {
            Serial.print(F("    constexpr float LINE_KP_REAR = "));
            Serial.print(bestG[REAR][0], 3);
            Serial.println(F("f;"));
            Serial.print(F("    constexpr float LINE_KD_REAR = "));
            Serial.print(bestG[REAR][1], 3);
            Serial.println(F("f;"));
        }
        else
        {
            Serial.println(F("    // (rear set not tuned — LINE_BOARD_COUNT < 2)"));
        }
        Serial.print(F("front cost="));
        Serial.print(bestCost[FRONT], 1);
        if (canReverse)
        {
            Serial.print(F("  rear cost="));
            Serial.print(bestCost[REAR], 1);
        }
        Serial.println();
        Serial.println(F("====================================="));
    }

    void finishNow()
    {
        // Operator-forced finish: latch whatever each optimiser has as its best-so-far.
        for (uint8_t d = 0; d < 2; ++d)
        {
            if (phase[d] != Phase::DONE)
            {
                markConverged(d);
            }
        }
        printFinalSummary();
        showDone();
    }

    void resetSession()
    {
        g[FRONT][0] = Config::LINE_KP;      g[FRONT][1] = Config::LINE_KD;
        g[REAR][0]  = Config::LINE_KP_REAR; g[REAR][1]  = Config::LINE_KD_REAR;
        for (uint8_t d = 0; d < 2; ++d)
        {
            bestG[d][0]  = g[d][0]; bestG[d][1] = g[d][1];
            stepSz[d][0] = KP_STEP_INIT; stepSz[d][1] = KD_STEP_INIT;
            bestCost[d]  = 1.0e30f;
            coordSel[d]  = 0;
            evalCostSum[d] = 0.0f;
            evalNumber[d]  = 0;
            phase[d]     = Phase::BASELINE;
        }
        // With one board the rear optimiser never runs — mark it done so bothConverged() gates on
        // the front alone.
        if (!canReverse)
        {
            phase[REAR] = Phase::DONE;
        }
        stepIdx = 0;
    }
}    // namespace

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    if (!LineArray.begin())
    {
        Serial.println(F("[tune] LineArray.begin() FAILED — check board wiring / I2C port."));
    }
    Drive.begin();

    canReverse   = (Config::LINE_BOARD_COUNT >= 2);
    stepsPerEval = canReverse ? 6 : 3;
    LineArray.setReverseActive(false);
    resetSession();

    Serial.println();
    Serial.println(F("=== LineFollower P+D auto-tuner (parallel ping-pong) ==="));
    Serial.print(F("cruise="));
    Serial.print(CRUISE_PWM);
    Serial.print(F("  distance="));
    Serial.print(RUN_DISTANCE_CM, 0);
    Serial.println(F("cm"));
    Serial.print(F("start FRONT Kp="));
    Serial.print(g[FRONT][0], 3);
    Serial.print(F(" Kd="));
    Serial.print(g[FRONT][1], 3);
    if (canReverse)
    {
        Serial.print(F("   REAR Kp="));
        Serial.print(g[REAR][0], 3);
        Serial.print(F(" Kd="));
        Serial.print(g[REAR][1], 3);
        Serial.println();
        Serial.println(F("rear board present — tuning BOTH sets (6 runs/round)."));
    }
    else
    {
        Serial.println();
        Serial.println(F("LINE_BOARD_COUNT < 2 — FRONT set only (3 runs/round)."));
    }
    Serial.println(F("BTN_UP = run next guided placement.  BTN_DOWN = finish + show best."));
    Serial.println(F("Place the robot as the OLED says, then press BTN_UP."));

    showWaiting();
}

void loop()
{
    LineArray.update();

    const bool upNow    = MiniR4.BTN_UP.getState();
    const bool downNow  = MiniR4.BTN_DOWN.getState();
    const bool upEdge   = upNow && !prevUp;
    const bool downEdge = downNow && !prevDown;
    prevUp   = upNow;
    prevDown = downNow;

    if (running)
    {
        stepControl();

        if (travelledCm() >= RUN_DISTANCE_CM)
        {
            finishRun(false, F(""));
        }
        else if ((millis() - runStartMs) > RUN_TIMEOUT_MS)
        {
            finishRun(true, F("timeout"));
        }
        else if (lineWasLost && (millis() - lostSinceMs) > LOST_ABORT_MS)
        {
            finishRun(true, F("lost-line"));
        }
        return;
    }

    // --- idle: waiting for the operator ---
    if (bothConverged())
    {
        if (upEdge || downEdge)
        {
            resetSession();
            Serial.println(F("[tune] restart — baseline from Config defaults."));
            showWaiting();
        }
        return;
    }

    if (downEdge)
    {
        finishNow();
        return;
    }
    if (upEdge)
    {
        startRun();
    }
}
