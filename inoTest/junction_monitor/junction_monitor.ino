/**
 * @file junction_monitor.ino
 * @brief Passive junction-classifier monitor — logs what WROLineArray sees, every 100 ms.
 * @author WRO 2026
 *
 * @details
 * Bring-up diagnostic for LineFollower junction tuning. The motors are NEVER driven: push
 * the chassis over a junction by hand and watch the classification stream. That isolates
 * `WROLineArrayClass::_classifyJunction()` from `Drive` completely, so anything seen here is
 * the sensor + the predicates, never the controller.
 *
 * Samples the sensor every tick (so the cache stays fresh) but prints on a fixed 100 ms
 * cadence — 10 readable lines per second. At hand-push speed every junction state lasts far
 * longer than 100 ms, so nothing of interest is aliased away.
 *
 * Columns:
 *   ms       — `millis()` timestamp
 *   type     — raw `getJunctionType()` value + name (0 NONE / 1 LEFT / 2 RIGHT / 3 T_OR_CROSS)
 *   onCount  — channels reading ON at `Config::LINE_CHANNEL_ON_RATIO` (drives the predicates)
 *   online   — `isOnline()`
 *   err      — `getError()`, the weighted centroid in [-3.5, +3.5]
 *   ch0..ch7 — raw packed 0..9 per-channel reflectance (leftmost .. rightmost)
 *
 * Build / flash / monitor:
 * @code
 *   uv run inoTest/junction_monitor/build.py
 * @endcode
 *
 * @note Watches the FRONT board by default. Flip `USE_REAR_BOARD` below to watch the rear
 *       bar instead (only meaningful when `Config::LINE_BOARD_COUNT == 2`).
 * @note NOT in CI — the `inoTest/` diagnostic sketches never are. `arduino-cli` is the only
 *       local check that this links.
 * @note `onCount` and `ch0..ch7` come from a SECOND `readActiveBoard()` I2C read, taken a few
 *       hundred microseconds after the cached sample that produced `type` / `err` / `online`.
 *       At hand-push speeds they are effectively the same sample; do not read a disagreement
 *       right at a predicate boundary as a defect.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "WROLineArray.h"

namespace
{
    /**
     * @brief Which bar to watch: `false` = FRONT board (0), `true` = REAR board (1).
     *
     * Drives one `setReverseActive()` call in `setup()`. Ignored by `WROLineArray` when
     * `Config::LINE_BOARD_COUNT < 2` — the front board is always active in a 1-board config.
     */
    constexpr bool USE_REAR_BOARD = false;

    /// @brief Serial print cadence. Every tick would flood; 100 ms is 10 readable lines/sec.
    constexpr uint32_t PRINT_PERIOD_MS = 100;

    /// @brief `millis()` of the last printed line.
    uint32_t lastPrintMs = 0;

    /**
     * @brief Human-readable name for a raw `WROLineArrayClass::getJunctionType()` value.
     *
     * @param type Raw 0..3 as returned by the getter.
     * @return Flash-resident name. `UNKNOWN` is unreachable with the WROLineArray backend
     *         (its predicates only ever return 0..3) but is handled so a future classifier
     *         that emits it does not print as a bare number.
     */
    const __FlashStringHelper* junctionName(uint8_t type)
    {
        switch (type)
        {
            case 0:  return F("NONE");
            case 1:  return F("LEFT");
            case 2:  return F("RIGHT");
            case 3:  return F("T_OR_CROSS");
            default: return F("UNKNOWN");
        }
    }

    /**
     * @brief Count channels reading ON, mirroring the classifier's normalisation + ratio rule.
     *
     * Deliberately duplicates the private `_onnessNorm()` + `Config::LINE_CHANNEL_ON_RATIO`
     * math: `onCount` is exactly what the T_OR_CROSS / LEFT / RIGHT predicates branch on and
     * there is no public accessor for it, so showing it is this sketch's whole purpose.
     *
     * @param boardIdx Active board index — indexes the `Config::LINE_CALIB_*` rows.
     * @param channels Raw packed 0..9 readings from `readActiveBoard()`.
     * @return Number of channels above the ON ratio, in [0, 8].
     * @warning Keep in sync with `WROLineArrayClass::_onnessNorm()` and `_classifyJunction()`.
     *          A drift here silently misreports the number being tuned against.
     */
    uint8_t onCountOf(uint8_t boardIdx, const uint8_t channels[8])
    {
        uint8_t count = 0;
        for (uint8_t c = 0; c < 8; ++c)
        {
            const uint8_t low  = Config::LINE_CALIB_LOW[boardIdx][c];
            const uint8_t high = Config::LINE_CALIB_HIGH[boardIdx][c];

            // Dead/inverted calibration collapses to OFF — same fail-soft rule the
            // classifier applies, so a botched paste reads the same in both places.
            if (low >= high)
            {
                continue;
            }

            float onness = (float) ((int) high - (int) channels[c])
                         / (float) ((int) high - (int) low);
            if (onness < 0.0f)
            {
                onness = 0.0f;
            }
            else if (onness > 1.0f)
            {
                onness = 1.0f;
            }

            if (onness > Config::LINE_CHANNEL_ON_RATIO)
            {
                ++count;
            }
        }
        return count;
    }

    /**
     * @brief Print @p name padded to a fixed width so the type column stays aligned.
     *
     * @param name  Flash-resident classification name.
     * @param width Column width in characters; names at or over it get no padding.
     */
    void printPadded(const __FlashStringHelper* name, uint8_t width)
    {
        const size_t written = Serial.print(name);
        for (size_t i = written; i < width; ++i)
        {
            Serial.print(' ');
        }
    }
}    // namespace

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }    // let USB CDC settle, but never hang headless

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // Only LineArray comes up. Drive/LineFollower are deliberately absent: nothing in this
    // sketch may command a motor, so the classification log cannot be blamed on the chassis.
    if (!LineArray.begin())
    {
        Serial.println(F("[jmon] LineArray.begin() FAILED — check board wiring / I2C port."));
    }

    // Route every getter + readActiveBoard() at the bar we want to watch. This is the same
    // call LineFollower issues from setReverse(), so the board selection matches production.
    LineArray.setReverseActive(USE_REAR_BOARD);

    Serial.println();
    Serial.println(F("=== junction monitor — push the chassis over a junction by hand ==="));

    Serial.print(F("watching: "));
    Serial.print(LineArray.getReverseActive() ? F("REAR board (1)") : F("FRONT board (0)"));
    Serial.print(F("   boards="));
    Serial.println(Config::LINE_BOARD_COUNT);

    if (USE_REAR_BOARD && Config::LINE_BOARD_COUNT < 2)
    {
        Serial.println(F("[jmon] WARN: USE_REAR_BOARD set but LINE_BOARD_COUNT < 2 — "
                         "the request was ignored and the FRONT board is active."));
    }

    // Echo the predicate knobs so a pasted log is self-describing months from now.
    Serial.print(F("ON_RATIO="));
    Serial.print(Config::LINE_CHANNEL_ON_RATIO);
    Serial.print(F("  T_MIN_ON="));
    Serial.print(Config::LINE_JUNCTION_T_MIN_ON);
    Serial.print(F("  LEFT_HI="));
    Serial.print(Config::LINE_JUNCTION_LEFT_HI);
    Serial.print(F("  RIGHT_LO="));
    Serial.println(Config::LINE_JUNCTION_RIGHT_LO);
    Serial.println();
    Serial.println(F("ms\ttype          \tonCount\tonline\terr\tch0..ch7"));
}

void loop()
{
    // Config::LINE_ARRAY_TICK_CACHE_ENABLED is true, so the getters below are pure cache
    // readers and this is the ONLY refresh site. Refresh EVERY tick (not just on a print
    // tick) so each printed sample is fresh rather than up to 100 ms stale.
    LineArray.update();

    const uint32_t now = millis();
    if ((now - lastPrintMs) < PRINT_PERIOD_MS)
    {
        return;
    }
    lastPrintMs = now;

    const uint8_t boardIdx = LineArray.getReverseActive() ? 1 : 0;

    uint8_t    channels[8];
    const bool readOk = LineArray.readActiveBoard(channels);

    Serial.print(now);
    Serial.print('\t');
    printPadded(junctionName(LineArray.getJunctionType()), 10);
    Serial.print(F(" ("));
    Serial.print(LineArray.getJunctionType());
    Serial.print(F(")\t"));

    if (readOk)
    {
        Serial.print(onCountOf(boardIdx, channels));
    }
    else
    {
        Serial.print(F("?"));    // I2C miss on the diagnostic re-read only
    }
    Serial.print('\t');

    Serial.print(LineArray.isOnline() ? F("yes") : F("NO"));
    Serial.print('\t');
    Serial.print(LineArray.getError(), 2);
    Serial.print('\t');

    if (readOk)
    {
        for (uint8_t c = 0; c < 8; ++c)
        {
            Serial.print(channels[c]);
            if (c < 7)
            {
                Serial.print(' ');
            }
        }
    }
    else
    {
        Serial.print(F("(read failed)"));
    }
    Serial.println();
}
