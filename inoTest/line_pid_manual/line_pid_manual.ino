/**
 * @file line_pid_manual.ino
 * @brief Live manual tuner for the LineFollower P+D gains — set gains over Serial, run, repeat.
 * @author WRO 2026
 *
 * @details
 * The hand-driven companion to `inoTest/line_pid_tune` (the auto-tuner). Instead of a twiddle
 * search, YOU set the gains: type them in the serial monitor, press BTN_UP (or type `go`) to run
 * a flat line-follow, read the score, tweak, run again. Gains live in RAM, so there is NO reflash
 * between trials — you hill-climb by hand.
 *
 * It reproduces `LineFollower.cpp`'s online P+D law EXACTLY (same error source, ±`LINE_MAX_CORR`
 * clamp, `L=cruise+corr`/`R=cruise-corr` mix, cruise negated in reverse, rear board selected via
 * `setReverseActive`, `Drive.setSpeed` deadband), so a gain that feels good here transfers 1:1 to
 * `Config`. It holds a FRONT set and a REAR set (ADR-066); `kp`/`kd` edit whichever direction is
 * currently selected.
 *
 * ------------------------------------------------------------------------------------------
 * SERIAL COMMANDS (type in the monitor, press Enter)
 * ------------------------------------------------------------------------------------------
 *   kp <v>     set Kp of the ACTIVE direction
 *   kd <v>     set Kd of the ACTIVE direction
 *   c <v>      set cruise PWM (default SPEED_NORMAL = 70)
 *   dist <v>   set run distance in cm (default 50)
 *   f          select FORWARD (front board)
 *   r          select REVERSE (rear board)
 *   go         run one line-follow with the current gains (same as BTN_UP)
 *   stop       abort a run in progress / stop the motors
 *   ?          print current settings + this help
 *
 * BUTTONS: BTN_UP = run.  BTN_DOWN = toggle direction (forward <-> reverse).
 *
 * After every run it prints the score so you can compare trials:
 *   cost=142.3 ( meanSqErr=1.204 osc=9 lostTicks=3 peakErr=2.80 ticks=118 )
 * Lower cost = better. See `inoTest/line_pid_tune/README.md` for what the terms mean.
 *
 * @note Reverse runs read the REAR board — calibrate it first (main firmware LINE calibration) or
 *       the reverse score is noise.
 * @note SAFETY: every run self-terminates on distance, a time watchdog, OR a sustained lost-line,
 *       so a bad gain cannot drive the robot off the table.
 *
 * Build / flash / monitor:
 * @code
 *   uv run inoTest/line_pid_manual/build.py
 * @endcode
 *
 * @note NOT in CI — the `inoTest/` diagnostic sketches never are.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "Drive.h"
#include "WROLineArray.h"

namespace
{
    constexpr uint8_t FRONT = 0;
    constexpr uint8_t REAR  = 1;

    constexpr float MSE_WEIGHT     = 50.0f;
    constexpr float OSC_WEIGHT     = 3.0f;
    constexpr float LOST_WEIGHT    = 10.0f;
    constexpr float ABORT_PENALTY  = 5000.0f;
    constexpr float OSC_HYSTERESIS = 0.4f;

    constexpr uint32_t RUN_TIMEOUT_MS = 6000;
    constexpr uint32_t LOST_ABORT_MS  = 1200;
    constexpr float    CM_PER_ENC_DEG = 3.14159265f * Config::WHEEL_DIAM_MM / 10.0f / 360.0f;

    // ---------------- Live, operator-set state (survives across runs; no reflash) ----------
    float    kp[2]      = {Config::LINE_KP, Config::LINE_KP_REAR};
    float    kd[2]      = {Config::LINE_KD, Config::LINE_KD_REAR};
    int16_t  cruisePwm  = Config::SPEED_NORMAL;
    float    distanceCm = 50.0f;
    bool     reverseSel = false;    ///< Active direction (false = forward/front, true = reverse/rear).

    // ---------------- Per-run state ----------
    bool     running     = false;
    uint32_t runStartMs  = 0;
    int32_t  startEncL   = 0;
    int32_t  startEncR   = 0;
    float    lastErr     = 0.0f;
    bool     seedErr     = true;
    double   sumSqErr    = 0.0;
    float    peakAbsErr  = 0.0f;
    uint32_t onlineTicks = 0;
    uint32_t lostTicks   = 0;
    uint32_t oscCount    = 0;
    int8_t   lastSign    = 0;
    bool     lineWasLost = false;
    uint32_t lostSinceMs = 0;
    float    lastCost    = -1.0f;

    // ---------------- Serial line buffer + button edges ----------
    String inbuf;
    bool   prevUp   = false;
    bool   prevDown = false;

    uint8_t dir() { return reverseSel ? REAR : FRONT; }

    // =====================================================================================
    // OLED — active-direction gains + cruise + last score.
    // =====================================================================================
    void render()
    {
        const uint8_t d = dir();
        MiniR4.OLED.clearDisplay();
        MiniR4.OLED.setTextSize(1);
        MiniR4.OLED.setTextColor(WHITE);
        MiniR4.OLED.setCursor(0, 0);
        MiniR4.OLED.println(running ? (reverseSel ? F("RUN <- REV") : F("RUN -> FWD"))
                                    : (reverseSel ? F("REV (rear)") : F("FWD (front)")));
        MiniR4.OLED.setCursor(0, 8);
        MiniR4.OLED.println(String("Kp ") + String(kp[d], 2) + "  Kd " + String(kd[d], 2));
        MiniR4.OLED.setCursor(0, 16);
        MiniR4.OLED.println(String("cruise ") + String((int) cruisePwm) + "  d "
                            + String((int) distanceCm));
        MiniR4.OLED.setCursor(0, 24);
        if (lastCost < 0.0f)
        {
            MiniR4.OLED.println(F("press UP to run"));
        }
        else
        {
            MiniR4.OLED.println(String("last cost ") + String(lastCost, 1));
        }
        MiniR4.OLED.display();
    }

    // =====================================================================================
    // Control law — EXACT mirror of LineFollower.cpp online P+D block, with the live gains.
    // =====================================================================================
    void stepControl()
    {
        const uint8_t d      = dir();
        const int16_t cruise = reverseSel ? (int16_t) -cruisePwm : cruisePwm;
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

        float corr = (kp[d] * err) + (kd[d] * dErr);
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
        const float a = (err < 0.0f) ? -err : err;
        if (a > peakAbsErr)
        {
            peakAbsErr = a;
        }
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
        LineArray.setReverseActive(reverseSel);
        startEncL   = Drive.getEncL();
        startEncR   = Drive.getEncR();
        runStartMs  = millis();
        lastErr     = 0.0f;
        seedErr     = true;
        sumSqErr    = 0.0;
        peakAbsErr  = 0.0f;
        onlineTicks = 0;
        lostTicks   = 0;
        oscCount    = 0;
        lastSign    = 0;
        lineWasLost = false;
        lostSinceMs = 0;
        running     = true;

        Serial.print(F("[man] RUN  dir="));
        Serial.print(reverseSel ? F("REV") : F("FWD"));
        Serial.print(F("  Kp="));
        Serial.print(kp[dir()], 3);
        Serial.print(F("  Kd="));
        Serial.print(kd[dir()], 3);
        Serial.print(F("  cruise="));
        Serial.println(cruisePwm);
        render();
    }

    float travelledCm()
    {
        const int32_t dL = Drive.getEncL() - startEncL;
        const int32_t dR = Drive.getEncR() - startEncR;
        return ((float) (abs(dL) + abs(dR)) * 0.5f) * CM_PER_ENC_DEG;
    }

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
        lastCost = cost;

        Serial.print(F("[man] done cost="));
        Serial.print(cost, 1);
        Serial.print(F("  ( meanSqErr="));
        Serial.print(meanSq, 3);
        Serial.print(F(" osc="));
        Serial.print(oscCount);
        Serial.print(F(" lostTicks="));
        Serial.print(lostTicks);
        Serial.print(F(" peakErr="));
        Serial.print(peakAbsErr, 2);
        Serial.print(F(" ticks="));
        Serial.print(onlineTicks);
        if (aborted)
        {
            Serial.print(F(" ABORTED:"));
            Serial.print(reason);
        }
        Serial.println(F(" )"));
        render();
    }

    // =====================================================================================
    // Serial command parser.
    // =====================================================================================
    void printSettings()
    {
        Serial.print(F("[man] dir="));
        Serial.print(reverseSel ? F("REV") : F("FWD"));
        Serial.print(F("  FRONT Kp="));
        Serial.print(kp[FRONT], 3);
        Serial.print(F(" Kd="));
        Serial.print(kd[FRONT], 3);
        Serial.print(F("  REAR Kp="));
        Serial.print(kp[REAR], 3);
        Serial.print(F(" Kd="));
        Serial.print(kd[REAR], 3);
        Serial.print(F("  cruise="));
        Serial.print(cruisePwm);
        Serial.print(F("  dist="));
        Serial.println(distanceCm, 0);
    }

    void printHelp()
    {
        Serial.println(F("commands: kp <v> | kd <v> | c <v> | dist <v> | f | r | go | stop | ?"));
        Serial.println(F("buttons:  BTN_UP = run    BTN_DOWN = toggle direction"));
        printSettings();
    }

    void handleCommand(const String& raw)
    {
        String line = raw;
        line.trim();
        if (line.length() == 0)
        {
            return;
        }
        const int sp   = line.indexOf(' ');
        String    cmd  = (sp < 0) ? line : line.substring(0, sp);
        cmd.toLowerCase();
        const bool  hasVal = (sp >= 0);
        const float val    = hasVal ? line.substring(sp + 1).toFloat() : 0.0f;

        if (cmd == "kp" && hasVal)
        {
            kp[dir()] = val;
            printSettings();
        }
        else if (cmd == "kd" && hasVal)
        {
            kd[dir()] = val;
            printSettings();
        }
        else if (cmd == "c" && hasVal)
        {
            cruisePwm = (int16_t) val;
            printSettings();
        }
        else if (cmd == "dist" && hasVal)
        {
            distanceCm = (val > 1.0f) ? val : 1.0f;
            printSettings();
        }
        else if (cmd == "f")
        {
            reverseSel = false;
            printSettings();
            render();
        }
        else if (cmd == "r")
        {
            reverseSel = true;
            printSettings();
            render();
        }
        else if (cmd == "go" || cmd == "g" || cmd == "run")
        {
            if (!running)
            {
                startRun();
            }
        }
        else if (cmd == "stop" || cmd == "s")
        {
            if (running)
            {
                finishRun(true, F("operator-stop"));
            }
            else
            {
                Drive.stop(true);
            }
        }
        else
        {
            printHelp();
        }
    }

    void pollSerial()
    {
        while (Serial.available() > 0)
        {
            const char ch = (char) Serial.read();
            if (ch == '\n' || ch == '\r')
            {
                if (inbuf.length() > 0)
                {
                    handleCommand(inbuf);
                    inbuf = "";
                }
            }
            else if (inbuf.length() < 40)
            {
                inbuf += ch;
            }
        }
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
        Serial.println(F("[man] LineArray.begin() FAILED — check board wiring / I2C port."));
    }
    Drive.begin();
    LineArray.setReverseActive(false);

    Serial.println();
    Serial.println(F("=== LineFollower P+D manual tuner ==="));
    printHelp();
    Serial.println(F("Place the robot on the line, set gains, press BTN_UP (or type 'go')."));

    render();
}

void loop()
{
    LineArray.update();
    pollSerial();

    const bool upNow    = MiniR4.BTN_UP.getState();
    const bool downNow  = MiniR4.BTN_DOWN.getState();
    const bool upEdge   = upNow && !prevUp;
    const bool downEdge = downNow && !prevDown;
    prevUp   = upNow;
    prevDown = downNow;

    if (running)
    {
        stepControl();

        if (travelledCm() >= distanceCm)
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
        else if (upEdge || downEdge)
        {
            finishRun(true, F("button-stop"));    // either button aborts a run in progress
        }
        return;
    }

    // --- idle ---
    if (downEdge)
    {
        reverseSel = !reverseSel;
        printSettings();
        render();
    }
    if (upEdge)
    {
        startRun();
    }
}
