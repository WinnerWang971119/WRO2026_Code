/**
 * @file MotorMatchHarness.cpp
 * @brief Characterize spare motors one at a time and pick the best-matched pair for the chassis.
 * @author WRO 2026
 *
 * @details
 * PURPOSE. The chassis needs its LEFT and RIGHT motors to spin at the same rate for the same power
 * (a mismatch makes the robot curve and corrupts symmetric-PWM counter-rotation turns). Given a box
 * of spare motors, this finds the two that match each other best -- and, among equally matched
 * candidates, the faster pair.
 *
 * FLOW. Plug ONE motor into the M2 port (the Matrix motor cable carries power + encoder together, so
 * a bench motor's encoder rides the same port). Tap BTN_UP: the sketch spins it forward and reverse
 * at three PWM levels and stores the average shaft rate (raw encoder counts/s) into an EEPROM slot.
 * Reset, plug in the next motor, tap BTN_UP again -- records persist across resets. After all motors
 * are in, tap BTN_DOWN to print a ranked pair table and a recommendation.
 *
 * ENCODER CHANNEL. The measurement reads WHICHEVER of M2/M3 actually moved and reports it, so it is
 * immune to the chassis encoder-cable crossing (a bench motor on a single cable reads on M2; a motor
 * left in the crossed chassis wiring would read on M3). See the enc-swap memory.
 *
 * VOLTAGE. A pack sags as you work through the box, and a DC motor's no-load speed tracks voltage,
 * so a later motor would look slower for no real reason. Each record stamps the pack voltage; the
 * pair math normalizes every rate to the mean pack voltage (first-order Vbatt correction) before
 * comparing. This is a NO-LOAD speed match -- it does not capture torque/under-load behaviour.
 *
 * @note LIFT / free the shaft -- motors spin unloaded. Build/flash/monitor: `uv run
 *       inoTest/motor_match/build.py`. NOT in CI.
 */

#include "MotorMatchHarness.h"

#include <EEPROM.h>
#include <MatrixMiniR4.h>

#include <math.h>       // fabsf, fmaxf
#include <stdint.h>

#include "Config.h"

namespace
{
    constexpr uint8_t  NUM_SLOTS         = 8;
    constexpr uint8_t  NUM_LEVELS        = 3;
    constexpr int16_t  PWM_LEVELS[NUM_LEVELS] = {40, 60, 80};
    constexpr uint32_t WINDOW_MS         = 1200;    ///< spin/measure window per level+direction
    constexpr uint32_t SETTLE_MS         = 350;     ///< coast-to-rest before each spin
    constexpr uint32_t LONG_PRESS_MS     = 900;     ///< tap vs hold threshold
    constexpr float    MIN_VALID_RATE    = 5.0f;    ///< below this on every level -> "no motion"
    constexpr float    MATCH_TOL_PCT     = 6.0f;    ///< a pair within this speed spread is "matched"
    constexpr uint32_t STORE_MAGIC       = 0x4D4D3031UL;    ///< "MM01" -- detects an uninitialized EEPROM
    constexpr int      STORE_ADDR        = 0;

    /// @brief One motor's persisted stats. Raw rate + the pack voltage it was measured at.
    struct MotorRecord
    {
        uint8_t used;          ///< 0 = empty slot, 1 = recorded
        uint8_t encChannel;    ///< which encoder read the shaft (2 or 3), for diagnostics
        float   vbatt;         ///< pack voltage during this motor's test
        float   rate[NUM_LEVELS];    ///< |shaft rate| in raw counts/s, averaged over fwd+rev
    };

    /// @brief The whole persisted table, magic-stamped so a blank EEPROM is detected and cleared.
    struct Store
    {
        uint32_t    magic;
        MotorRecord motors[NUM_SLOTS];
    };

    Store g_store;

    enum MenuEvent
    {
        EV_UP_SHORT,
        EV_UP_LONG,
        EV_DOWN_SHORT,
        EV_DOWN_LONG
    };

    // ---- low-level motor + timing helpers ---------------------------------------------------

    void coastBoth()
    {
        MiniR4.M2.setPower(0);
        MiniR4.M3.setPower(0);
        MiniR4.M2.setBrake(false);
        MiniR4.M3.setBrake(false);
    }

    void settleMs(uint32_t ms)
    {
        const uint32_t s = millis();
        while ((uint32_t) (millis() - s) < ms) { yield(); }
    }

    float readVbatt()
    {
        float v = MiniR4.PWR.getBattVoltage();
        if (v <= 0.0f)
        {
            delay(50);
            v = MiniR4.PWR.getBattVoltage();    // retry-once idiom (a 0 read is an I2C miss)
        }
        return v;
    }

    /// @brief One measured spin: the shaft rate magnitude (counts/s) and which channel saw it.
    struct Spin
    {
        float   mag;
        uint8_t ch;
    };

    /// @brief Drive M2 at @p pwm for one window and return the driven shaft's rate + channel. Reads
    /// BOTH encoders and keeps the one that moved, so it is immune to the encoder-cable crossing.
    Spin spinOnce(int16_t pwm)
    {
        coastBoth();
        settleMs(SETTLE_MS);

        MiniR4.M2.resetCounter();
        MiniR4.M3.resetCounter();

        const uint32_t t0 = millis();
        MiniR4.M2.setPower(pwm);
        while ((uint32_t) (millis() - t0) < WINDOW_MS) { yield(); }

        const int32_t  d2 = MiniR4.M2.getCounter();
        const int32_t  d3 = MiniR4.M3.getCounter();
        const uint32_t t1 = millis();
        coastBoth();

        const float secs = (float) (t1 - t0) / 1000.0f;
        const float a2   = fabsf((float) d2);
        const float a3   = fabsf((float) d3);

        Spin s;
        s.ch  = (a2 >= a3) ? 2 : 3;
        s.mag = (secs > 0.0f) ? (fmaxf(a2, a3) / secs) : 0.0f;
        return s;
    }

    // ---- record one motor -------------------------------------------------------------------

    /// @brief Spin the motor on the M2 port fwd+rev at every level, fill @p rec, return whether it
    /// actually moved. Prints a per-level line incl. the fwd/rev asymmetry (a healthy motor is even).
    bool recordMotor(MotorRecord& rec)
    {
        uint8_t votes2 = 0, votes3 = 0;

        for (uint8_t k = 0; k < NUM_LEVELS; ++k)
        {
            const int16_t pwm = PWM_LEVELS[k];
            const Spin    f   = spinOnce(pwm);
            const Spin    r   = spinOnce((int16_t) -pwm);
            const float   avg = (f.mag + r.mag) * 0.5f;

            rec.rate[k] = avg;
            (f.ch == 2 ? votes2 : votes3)++;
            (r.ch == 2 ? votes2 : votes3)++;

            const float asym = (avg > 1.0f) ? (fabsf(f.mag - r.mag) / avg * 100.0f) : 0.0f;
            Serial.print(F("[MM]   pwm "));
            Serial.print(pwm);
            Serial.print(F(": fwd "));
            Serial.print(f.mag, 0);
            Serial.print(F(" | rev "));
            Serial.print(r.mag, 0);
            Serial.print(F(" | avg "));
            Serial.print(avg, 0);
            Serial.print(F(" c/s | dir-asym "));
            Serial.print(asym, 0);
            Serial.println(F("%"));
        }

        rec.encChannel = (votes2 >= votes3) ? 2 : 3;
        rec.vbatt      = readVbatt();
        rec.used       = 1;

        for (uint8_t k = 0; k < NUM_LEVELS; ++k)
        {
            if (rec.rate[k] > MIN_VALID_RATE) { return true; }
        }
        return false;
    }

    // ---- EEPROM store -----------------------------------------------------------------------

    void saveStore() { EEPROM.put(STORE_ADDR, g_store); }

    void eraseStore()
    {
        g_store.magic = STORE_MAGIC;
        for (uint8_t i = 0; i < NUM_SLOTS; ++i) { g_store.motors[i] = MotorRecord{}; }
        saveStore();
    }

    void loadStore()
    {
        EEPROM.get(STORE_ADDR, g_store);
        if (g_store.magic != STORE_MAGIC) { eraseStore(); }
    }

    uint8_t nextFreeSlot()
    {
        for (uint8_t i = 0; i < NUM_SLOTS; ++i)
        {
            if (!g_store.motors[i].used) { return i; }
        }
        return NUM_SLOTS;
    }

    int8_t lastUsedSlot()
    {
        for (int8_t i = NUM_SLOTS - 1; i >= 0; --i)
        {
            if (g_store.motors[i].used) { return i; }
        }
        return -1;
    }

    uint8_t usedCount()
    {
        uint8_t n = 0;
        for (uint8_t i = 0; i < NUM_SLOTS; ++i)
        {
            if (g_store.motors[i].used) { ++n; }
        }
        return n;
    }

    // ---- reporting --------------------------------------------------------------------------

    void printTable()
    {
        Serial.println(F("\n[MM] slot | Vbatt |enc| rate@40  rate@60  rate@80  (raw c/s)"));
        for (uint8_t i = 0; i < NUM_SLOTS; ++i)
        {
            const MotorRecord& m = g_store.motors[i];
            Serial.print(F("[MM]  #"));
            Serial.print(i);
            Serial.print(F("   | "));
            if (!m.used)
            {
                Serial.println(F("  --  | - |   (empty)"));
                continue;
            }
            Serial.print(m.vbatt, 2);
            Serial.print(F(" |M"));
            Serial.print(m.encChannel);
            Serial.print(F("| "));
            for (uint8_t k = 0; k < NUM_LEVELS; ++k)
            {
                Serial.print(m.rate[k], 0);
                Serial.print(F("     "));
            }
            Serial.println();
        }
    }

    /// @brief Rank every recorded pair by speed match (Vbatt-normalized), then recommend the fastest
    /// pair that matches within tolerance -- exactly "same power, prefer higher speed".
    void computeBestPair()
    {
        uint8_t idx[NUM_SLOTS];
        uint8_t n    = 0;
        float   vsum = 0.0f;
        for (uint8_t i = 0; i < NUM_SLOTS; ++i)
        {
            if (g_store.motors[i].used)
            {
                idx[n++] = i;
                vsum += g_store.motors[i].vbatt;
            }
        }
        if (n < 2)
        {
            Serial.println(F("\n[MM] need at least 2 recorded motors to compare."));
            return;
        }
        const float vmean = vsum / (float) n;

        struct Pair
        {
            uint8_t i, j;
            float   mismatch;    ///< mean relative speed spread across levels, in %
            float   speed;       ///< mean normalized speed of the pair, counts/s
        };
        Pair    pairs[NUM_SLOTS * (NUM_SLOTS - 1) / 2];
        uint8_t np = 0;

        for (uint8_t a = 0; a < n; ++a)
        {
            for (uint8_t b = a + 1; b < n; ++b)
            {
                const MotorRecord& A = g_store.motors[idx[a]];
                const MotorRecord& B = g_store.motors[idx[b]];
                const float        va = (A.vbatt > 0.0f) ? A.vbatt : vmean;
                const float        vb = (B.vbatt > 0.0f) ? B.vbatt : vmean;

                float   mm = 0.0f, sp = 0.0f;
                uint8_t cnt = 0;
                for (uint8_t k = 0; k < NUM_LEVELS; ++k)
                {
                    const float na = A.rate[k] * vmean / va;    // normalize to mean pack voltage
                    const float nb = B.rate[k] * vmean / vb;
                    const float av = (na + nb) * 0.5f;
                    if (av > 1.0f)
                    {
                        mm += fabsf(na - nb) / av;
                        sp += av;
                        ++cnt;
                    }
                }
                if (cnt == 0) { continue; }

                Pair p;
                p.i        = idx[a];
                p.j        = idx[b];
                p.mismatch = mm / (float) cnt * 100.0f;
                p.speed    = sp / (float) cnt;
                pairs[np++] = p;
            }
        }

        // insertion sort by mismatch ascending (best-matched first)
        for (uint8_t x = 1; x < np; ++x)
        {
            const Pair key = pairs[x];
            int8_t     y   = (int8_t) x - 1;
            while (y >= 0 && pairs[y].mismatch > key.mismatch)
            {
                pairs[y + 1] = pairs[y];
                --y;
            }
            pairs[y + 1] = key;
        }

        Serial.println(F("\n[MM] ===== PAIR RANKING (best speed match first) ====="));
        Serial.println(F("[MM]   pair     match-spread   avg-speed (norm c/s)"));
        for (uint8_t x = 0; x < np; ++x)
        {
            Serial.print(F("[MM]   #"));
            Serial.print(pairs[x].i);
            Serial.print(F(" + #"));
            Serial.print(pairs[x].j);
            Serial.print(F("     "));
            Serial.print(pairs[x].mismatch, 1);
            Serial.print(F("%          "));
            Serial.println(pairs[x].speed, 0);
        }

        // recommend: fastest pair that matches within tolerance; else the closest match
        int8_t best = -1;
        for (uint8_t x = 0; x < np; ++x)
        {
            if (pairs[x].mismatch <= MATCH_TOL_PCT)
            {
                if (best < 0 || pairs[x].speed > pairs[best].speed) { best = (int8_t) x; }
            }
        }

        Serial.println();
        if (best >= 0)
        {
            Serial.print(F("[MM] RECOMMENDED PAIR (matched within "));
            Serial.print(MATCH_TOL_PCT, 0);
            Serial.print(F("%, fastest): #"));
            Serial.print(pairs[best].i);
            Serial.print(F(" + #"));
            Serial.println(pairs[best].j);
        }
        else
        {
            Serial.print(F("[MM] no pair matched within "));
            Serial.print(MATCH_TOL_PCT, 0);
            Serial.print(F("%. Closest match: #"));
            Serial.print(pairs[0].i);
            Serial.print(F(" + #"));
            Serial.println(pairs[0].j);
        }
        Serial.print(F("[MM] (speeds normalized to mean pack voltage "));
        Serial.print(vmean, 2);
        Serial.println(F(" V)"));
    }

    // ---- button menu ------------------------------------------------------------------------

    bool upPressed() { return MiniR4.BTN_UP.getState(); }
    bool downPressed() { return MiniR4.BTN_DOWN.getState(); }

    /// @brief Block until one button is pressed and released; classify by which + hold duration.
    MenuEvent waitMenuEvent()
    {
        while (upPressed() || downPressed()) { yield(); }    // drain any held button first

        uint8_t which = 0;
        while (which == 0)
        {
            if (upPressed()) { which = 1; }
            else if (downPressed()) { which = 2; }
            yield();
        }

        const uint32_t t0 = millis();
        while (which == 1 ? upPressed() : downPressed()) { yield(); }
        const bool isLong = (uint32_t) (millis() - t0) >= LONG_PRESS_MS;

        if (which == 1) { return isLong ? EV_UP_LONG : EV_UP_SHORT; }
        return isLong ? EV_DOWN_LONG : EV_DOWN_SHORT;
    }

    void printMenu()
    {
        Serial.println(F("\n[MM] ---------------- CONTROLS ----------------"));
        Serial.println(F("[MM]  BTN_UP   tap  : record the motor on the M2 port -> next free slot"));
        Serial.println(F("[MM]  BTN_UP   hold : clear the last recorded slot (redo it)"));
        Serial.println(F("[MM]  BTN_DOWN tap  : compute + print the best matched pair"));
        Serial.println(F("[MM]  BTN_DOWN hold : ERASE all stored records"));
        Serial.println(F("[MM] ------------------------------------------"));
        Serial.print(F("[MM]  recorded: "));
        Serial.print(usedCount());
        Serial.print(F("/"));
        Serial.print(NUM_SLOTS);
        Serial.print(F("   next free slot: "));
        const uint8_t ns = nextFreeSlot();
        if (ns >= NUM_SLOTS) { Serial.println(F("(full)")); }
        else
        {
            Serial.print(F("#"));
            Serial.println(ns);
        }
    }
}    // namespace

void runMotorMatch(void (*tickCb)())
{
    (void) tickCb;
    coastBoth();
    loadStore();

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 motor-match tester (inoTest/motor_match)        #"));
    Serial.println(F("#  Characterize each spare motor, pick the best pair.      #"));
    Serial.println(F("############################################################"));
    Serial.println(F("[MM] Plug ONE motor into the M2 port (one cable carries power+encoder),"));
    Serial.println(F("[MM] shaft free to spin. Tap BTN_UP to record it. Repeat for every motor"));
    Serial.println(F("[MM] (records persist across resets), then tap BTN_DOWN to compare."));

    printTable();
    printMenu();

    for (;;)
    {
        switch (waitMenuEvent())
        {
            case EV_UP_SHORT:
            {
                const uint8_t slot = nextFreeSlot();
                if (slot >= NUM_SLOTS)
                {
                    Serial.println(F("\n[MM] all slots full. BTN_DOWN to compute, or hold BTN_DOWN "
                                     "to erase."));
                    printMenu();
                    break;
                }
                Serial.print(F("\n[MM] === recording slot #"));
                Serial.print(slot);
                Serial.println(F(" === (spinning now, keep the shaft clear)"));

                MotorRecord rec = MotorRecord{};
                if (!recordMotor(rec))
                {
                    Serial.println(F("[MM] !! no encoder motion -- NOT saved. Check the cable is "
                                     "fully seated in the M2 port."));
                }
                else
                {
                    g_store.motors[slot] = rec;
                    saveStore();
                    Serial.print(F("[MM] saved slot #"));
                    Serial.print(slot);
                    Serial.print(F("  (enc M"));
                    Serial.print(rec.encChannel);
                    Serial.print(F(", "));
                    Serial.print(rec.vbatt, 2);
                    Serial.println(F(" V)"));
                }
                printTable();
                printMenu();
                break;
            }
            case EV_UP_LONG:
            {
                const int8_t last = lastUsedSlot();
                if (last < 0) { Serial.println(F("\n[MM] nothing to clear.")); }
                else
                {
                    g_store.motors[last] = MotorRecord{};
                    saveStore();
                    Serial.print(F("\n[MM] cleared slot #"));
                    Serial.print(last);
                    Serial.println(F(" -- re-plug that motor and tap BTN_UP to redo it."));
                }
                printTable();
                printMenu();
                break;
            }
            case EV_DOWN_SHORT:
                computeBestPair();
                printMenu();
                break;

            case EV_DOWN_LONG:
                eraseStore();
                Serial.println(F("\n[MM] all records ERASED. Starting fresh."));
                printTable();
                printMenu();
                break;
        }
    }
}
