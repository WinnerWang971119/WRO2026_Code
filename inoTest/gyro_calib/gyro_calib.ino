/**
 * @file gyro_calib.ino
 * @brief Standalone bench sketch to measure the external MXColor gyro's scale, handedness
 *        (sign), and zero-rate drift for `Config::GYRO_SCALE` / `Config::GYRO_YAW_SIGN`
 *        (issue #163, task T8).
 * @author WRO 2026
 *
 * @details
 * Three operator-gated measurement phases, each advanced ONLY by an explicit `BTN_UP` press
 * (never an auto-countdown), with a ~60 s per-phase timeout (repo convention):
 *
 *   1. SIGN    -- baseline raw is captured the instant the phase is entered; the operator
 *                 rotates the robot COUNTER-CLOCKWISE by hand, then presses `BTN_UP` to end
 *                 the window. If the raw count INCREASED, the board is already CCW-positive
 *                 (`GYRO_YAW_SIGN = +1.0f`); if it DECREASED, `GYRO_YAW_SIGN = -1.0f`.
 *   2. SCALE   -- the operator hand-turns the robot exactly `SCALE_TURNS_N` full turns against
 *                 a fixed reference mark, then presses `BTN_UP` to end the window. The sketch
 *                 accumulates the UNWRAPPED raw travel via a fast per-sample delta sum (see
 *                 `shortestRawDelta`), then `GYRO_SCALE = (N * 360) / |accumulated|`.
 *   3. DRIFT   -- the operator holds the robot still and presses `BTN_UP` to START a fixed
 *                 logging window (`DRIFT_WINDOW_MS`); drift is reported in deg/min. Report-only
 *                 -- nothing compensates for it.
 *
 * After the sequence, the sketch Serial-prints paste-ready `#define ..._DEFAULT` lines for
 * `config/ConfigNav.h` and enters an optional LIVE heading display (decoded heading refreshed
 * on Serial + OLED every `LIVE_UPDATE_PERIOD_MS`; `BTN_DOWN` zeros the gyro via `getColor(K)`).
 * Serial command `r` re-runs the SIGN/SCALE/DRIFT sequence without reflashing -- useful for the
 * AC-11 3-run repeatability check.
 *
 * This sketch is NOT production code and carries ZERO runtime weight in the mission firmware --
 * it shares `GyroDecodePolicy.h`'s pure decode math with `YawSource` (never reimplements it), and
 * scale/sign are threaded through as ordinary local variables, exactly like the production call
 * site does with `Config::GYRO_SCALE` / `Config::GYRO_YAW_SIGN`.
 *
 * ASSUMPTIONS CARRIED (see the plan's Decisions section):
 *   - Raw counts are approximately degrees (assumption #2) -- this is what makes the SCALE
 *     phase's fast-polling requirement ("never miss more than <180 deg of true rotation between
 *     samples") meaningful in raw-count units.
 *   - `MatrixColor::begin()` may legitimately return `false` on this board (assumption #5 -- it
 *     ID-checks `Device_ID == 0x43` and writes a CONFIG reset the mentor's own code never calls).
 *     This sketch treats that `false` as INCONCLUSIVE, not fatal, and falls through to a raw
 *     `getColor` liveness probe -- the only authoritative check.
 *
 * BUILD MECHANIC -- `WRO2026/` and `WRO2026/src/nav/` as legacy (1.0) Arduino libraries
 * -------------------------------------------------------------------------------------------------
 * Neither `WRO2026/` nor `WRO2026/src/nav/` carries a `library.properties`, so each resolves as a
 * flat legacy-1.0 library root when passed via its own `--library` flag: `--library WRO2026` adds
 * only `WRO2026/`'s root files (reaching `Config.h`) non-recursively, and `--library
 * WRO2026/src/nav` adds only that folder (reaching `GyroDecodePolicy.h`) non-recursively. Neither
 * flag recurses into the other's subfolders, so both are required and every `#include` below
 * stays a BARE filename (no path-qualified include) -- same convention as
 * `inoTest/tick_bench/tick_bench.ino`.
 *
 * @note BUILD: run from the repo root --
 *       arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi inoTest/gyro_calib \
 *         --library WRO2026 --library WRO2026/src/nav
 *
 * @note Serial diagnostics run at 115200 baud (matches `WRO2026.ino`); `setup()` waits up to 3 s
 *       for a USB CDC listener but does not hang a headless boot.
 *
 * @note NO `build.py` on this base -- `inoTest/` sketches are bare `.ino` files (see
 *       `inoTest/tick_bench/tick_bench.ino`, `inoTest/gyro-move.ino`), not a Python build wrapper.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "GyroDecodePolicy.h"

namespace
{
    // =========================================================================================
    // Bench-tunable constants. This is NOT production code -- edit freely before flashing.
    // =========================================================================================

    /// @brief Whole number of full turns (N x 360 deg) the operator performs in the SCALE
    ///        phase. Larger N makes a fixed scale error more measurable (10 turns -> 3600 raw
    ///        counts, so a 0.3% scale error is an ~11-count difference). Edit before flashing
    ///        to match how many turns you plan to physically perform.
    constexpr uint8_t SCALE_TURNS_N = 10;

    /// @brief Per-phase operator-response timeout (ms) -- repo convention (~60 s).
    constexpr uint32_t PHASE_TIMEOUT_MS = 60000;

    /// @brief Fixed drift-logging window (ms) for the DRIFT phase (spec range: 30-60 s).
    constexpr uint32_t DRIFT_WINDOW_MS = 30000;

    /// @brief Serial log sample period (ms) while the DRIFT phase's window runs.
    constexpr uint32_t DRIFT_SAMPLE_PERIOD_MS = 1000;

    /// @brief Poll delay (ms) inside the BTN_UP wait / SCALE accumulation loops. Fast enough
    ///        that a hand rotation can never cover >=180 deg between two consecutive samples
    ///        (the vendor read itself costs ~4-6 ms, so the loop already runs at ~10 ms).
    constexpr uint32_t POLL_PERIOD_MS = 5;

    /// @brief Serial + OLED refresh period (ms) for the post-calibration LIVE heading display.
    constexpr uint32_t LIVE_UPDATE_PERIOD_MS = 250;

    /// @brief Half and full period of the raw register's native 16-bit rollover, used ONLY by
    ///        `shortestRawDelta` (raw-count space, NOT the 360 deg space `gyroWrap180` wraps).
    constexpr int32_t RAW_ROLLOVER_HALF   = 32768;
    constexpr int32_t RAW_ROLLOVER_PERIOD = 65536;

    // =========================================================================================
    // Measured-so-far state. Falls back to the Config placeholder until a phase succeeds, so
    // `printResults()` and the LIVE display always have SOMETHING usable to decode with.
    // =========================================================================================

    float _measuredSign   = Config::GYRO_YAW_SIGN;
    float _measuredScale  = Config::GYRO_SCALE;
    bool  _haveSign       = false;
    bool  _haveScale      = false;
    float _driftDegPerMin = 0.0f;
    bool  _haveDrift      = false;
}    // namespace

/// @brief The gyro's `MatrixColor` transport instance. Wired onto `Wire1` + the configured mux
///        channel in `setup()`, exactly like `WROLineArray`'s per-board instances.
MatrixColor gyro;

// =============================================================================================
// Low-level I/O helpers
// =============================================================================================

/**
 * @brief Read the gyro's two heading bytes G (high) then R (low) BACK-TO-BACK.
 * @warning Nothing may touch the I2C bus between the two calls -- each `getColor()` re-runs the
 *          board's internal mux-select, so any intervening bus traffic corrupts the pair.
 */
void readGyroBytes(uint8_t& hiG, uint8_t& loR)
{
    hiG = gyro.getColor(G);
    loR = gyro.getColor(R);
}

/**
 * @brief Read + decode the current UNWRAPPED raw biased count, rejecting an all-zero (dead bus)
 *        pair.
 * @param[out] biasedOut Set to `gyroRawBiased(hiG, loR)` on success; untouched on failure.
 * @return `false` if the pair read all-zero (see `gyroBytesDead`); `true` otherwise.
 */
bool readRawBiasedChecked(int32_t& biasedOut)
{
    uint8_t hiG = 0, loR = 0;
    readGyroBytes(hiG, loR);
    if (gyroBytesDead(hiG, loR))
    {
        return false;
    }
    biasedOut = gyroRawBiased(hiG, loR);
    return true;
}

/**
 * @brief Shortest signed delta between two raw biased counts, unwrapped across a possible
 *        16-bit register rollover (period 65536 -- the RAW register's OWN native modulus, NOT
 *        the 360 deg modulus `GyroDecodePolicy.h`'s `gyroWrap180` applies to the SCALED,
 *        DECODED heading).
 *
 * Sketch-local on purpose: the SCALE phase accumulates in raw-count space, before scale is even
 * known, so this cannot be expressed via the shared policy header (whose functions all operate
 * on already-scaled degrees). Mirrors `gyroWrap180`'s own wrap logic at a different modulus, and
 * carries the same assumption: consecutive samples must be closer together than half a rollover
 * period, which the SCALE phase's fast polling loop guarantees for any physically plausible
 * hand-rotation speed.
 *
 * @param current Latest raw biased count.
 * @param last    Previous raw biased count.
 * @return The shortest signed delta `current - last`, corrected for rollover.
 */
int32_t shortestRawDelta(int32_t current, int32_t last)
{
    int32_t delta = current - last;
    while (delta > RAW_ROLLOVER_HALF)
    {
        delta -= RAW_ROLLOVER_PERIOD;
    }
    while (delta <= -RAW_ROLLOVER_HALF)
    {
        delta += RAW_ROLLOVER_PERIOD;
    }
    return delta;
}

/**
 * @brief Block until `BTN_UP` transitions low->high (a fresh press, ignoring any already-held
 *        state), or until `timeoutMs` elapses.
 * @return `true` on a fresh `BTN_UP` press; `false` on timeout.
 */
bool waitForBtnUpPress(uint32_t timeoutMs)
{
    bool           prev = MiniR4.BTN_UP.getState();
    const uint32_t t0   = millis();
    while (millis() - t0 < timeoutMs)
    {
        const bool now = MiniR4.BTN_UP.getState();
        if (now && !prev)
        {
            while (MiniR4.BTN_UP.getState()) { delay(POLL_PERIOD_MS); }    // debounce: wait for release
            return true;
        }
        prev = now;
        delay(POLL_PERIOD_MS);
    }
    return false;
}

// =============================================================================================
// OLED helpers
// =============================================================================================

/// @brief Clear the OLED and draw `title` on row 0. Caller adds rows 1-3 and calls `display()`.
void oledClearHeader(const char* title)
{
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setTextSize(1);
    MiniR4.OLED.setTextColor(WHITE);
    MiniR4.OLED.setCursor(0, 0);
    MiniR4.OLED.println(title);
}

// =============================================================================================
// Serial banner / help / commands
// =============================================================================================

/// @brief Print the `?` help block + the currently measured/placeholder state.
void printHelp()
{
    Serial.println(F("\n-- gyro_calib commands --"));
    Serial.println(F("  ?  help + current measured state"));
    Serial.println(F("  r  re-run the SIGN -> SCALE -> DRIFT calibration sequence"));
    Serial.println(F("  (BTN_DOWN, live mode) zero the gyro via getColor(K)"));
    Serial.print(F("[state] sign="));
    Serial.print(_haveSign ? F("MEASURED ") : F("placeholder "));
    Serial.print(_measuredSign, 1);
    Serial.print(F("  scale="));
    Serial.print(_haveScale ? F("MEASURED ") : F("placeholder "));
    Serial.print(_measuredScale, 4);
    Serial.print(F("  drift="));
    if (_haveDrift)
    {
        Serial.print(_driftDegPerMin, 3);
        Serial.println(F(" deg/min"));
    }
    else
    {
        Serial.println(F("not measured"));
    }
}

/// @brief One-time boot banner: title, the configured I2C port, and the Config placeholders.
void printBanner()
{
    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 gyro_calib -- external gyro calibration (#163)  #"));
    Serial.println(F("############################################################"));
    Serial.print(F("Config::GYRO_I2C_PORT = I2C"));
    Serial.println(Config::GYRO_I2C_PORT);
    Serial.print(F("Config placeholders: GYRO_SCALE="));
    Serial.print(Config::GYRO_SCALE, 4);
    Serial.print(F("  GYRO_YAW_SIGN="));
    Serial.println(Config::GYRO_YAW_SIGN, 1);
    printHelp();
}

// =============================================================================================
// Calibration phases
// =============================================================================================

/**
 * @brief Phase 1/3 -- sign / handedness.
 *
 * Baseline raw is captured the instant this phase is entered; the operator rotates the robot
 * CCW by hand and presses `BTN_UP` to end the window. Sets `_measuredSign` + `_haveSign` on
 * success; leaves the Config placeholder untouched on a dead bus, a zero delta, or a timeout.
 */
void runSignPhase()
{
    Serial.println(F("\n=== PHASE 1/3: SIGN (handedness) ==="));
    Serial.println(F("Rotate the robot COUNTER-CLOCKWISE (viewed from above) by a clear amount,"));
    Serial.println(F("then press BTN_UP."));

    oledClearHeader("P1 SIGN");
    MiniR4.OLED.setCursor(0, 8);
    MiniR4.OLED.println(F("Rotate robot CCW"));
    MiniR4.OLED.setCursor(0, 16);
    MiniR4.OLED.println(F("then press BTN_UP"));
    MiniR4.OLED.display();

    int32_t before = 0;
    if (!readRawBiasedChecked(before))
    {
        Serial.println(F("[gyro_calib] SIGN: dead bus (G=R=0) at phase entry -- skipping."));
        return;
    }

    if (!waitForBtnUpPress(PHASE_TIMEOUT_MS))
    {
        Serial.println(F("[gyro_calib] SIGN: timed out waiting for BTN_UP -- skipping."));
        return;
    }

    int32_t after = 0;
    if (!readRawBiasedChecked(after))
    {
        Serial.println(F("[gyro_calib] SIGN: dead bus (G=R=0) at phase end -- skipping."));
        return;
    }

    const int32_t delta = after - before;
    Serial.print(F("[gyro_calib] SIGN: raw before="));
    Serial.print(before);
    Serial.print(F("  after="));
    Serial.print(after);
    Serial.print(F("  delta="));
    Serial.println(delta);

    if (delta == 0)
    {
        Serial.println(F("[gyro_calib] SIGN: zero delta -- rotation too small/fast to measure; "
                          "not updating sign."));
        return;
    }

    _measuredSign = (delta > 0) ? 1.0f : -1.0f;
    _haveSign     = true;
    Serial.print(F("[gyro_calib] SIGN measured: GYRO_YAW_SIGN = "));
    Serial.println(_measuredSign > 0.0f ? F("+1.0f") : F("-1.0f"));
}

/**
 * @brief Phase 2/3 -- scale.
 *
 * Accumulates the UNWRAPPED raw travel across the operator's `SCALE_TURNS_N` x 360 deg
 * hand-rotation via a running per-sample delta sum (`shortestRawDelta`), then
 * `GYRO_SCALE = (N * 360) / |accumulated|`. Sets `_measuredScale` + `_haveScale` on success.
 */
void runScalePhase()
{
    Serial.println(F("\n=== PHASE 2/3: SCALE ==="));
    Serial.print(F("Rotate the robot EXACTLY "));
    Serial.print(SCALE_TURNS_N);
    Serial.println(F(" full turns against a fixed reference mark, then press BTN_UP."));

    oledClearHeader("P2 SCALE");
    MiniR4.OLED.setCursor(0, 8);
    MiniR4.OLED.print(F("Rotate "));
    MiniR4.OLED.print(SCALE_TURNS_N);
    MiniR4.OLED.println(F(" full turns"));
    MiniR4.OLED.setCursor(0, 16);
    MiniR4.OLED.println(F("then press BTN_UP"));
    MiniR4.OLED.display();

    int32_t lastBiased = 0;
    if (!readRawBiasedChecked(lastBiased))
    {
        Serial.println(F("[gyro_calib] SCALE: dead bus (G=R=0) at phase entry -- skipping."));
        return;
    }

    int64_t        totalRawAccum = 0;
    bool           btnSeen       = false;
    bool           prevBtnUp     = MiniR4.BTN_UP.getState();
    const uint32_t t0            = millis();

    while (millis() - t0 < PHASE_TIMEOUT_MS)
    {
        const bool nowBtnUp = MiniR4.BTN_UP.getState();
        if (nowBtnUp && !prevBtnUp)
        {
            btnSeen = true;
            while (MiniR4.BTN_UP.getState()) { delay(POLL_PERIOD_MS); }    // debounce
            break;
        }
        prevBtnUp = nowBtnUp;

        // A single torn/dead sample is skipped, not fatal -- aborting the whole phase on one bad
        // I2C read would throw away several seconds of the operator's hand-rotation.
        int32_t curBiased = 0;
        if (readRawBiasedChecked(curBiased))
        {
            totalRawAccum += shortestRawDelta(curBiased, lastBiased);
            lastBiased = curBiased;
        }

        delay(POLL_PERIOD_MS);
    }

    if (!btnSeen)
    {
        Serial.println(F("[gyro_calib] SCALE: timed out waiting for BTN_UP -- skipping."));
        return;
    }

    if (totalRawAccum == 0)
    {
        Serial.println(F("[gyro_calib] SCALE: zero accumulated raw travel -- not updating scale."));
        return;
    }

    _measuredScale = ((float) SCALE_TURNS_N * 360.0f) / fabsf((float) totalRawAccum);
    _haveScale     = true;

    Serial.print(F("[gyro_calib] SCALE: totalRawAccum="));
    Serial.print((long) totalRawAccum);
    Serial.print(F("  N="));
    Serial.print(SCALE_TURNS_N);
    Serial.print(F("  -> GYRO_SCALE = "));
    Serial.println(_measuredScale, 4);
}

/**
 * @brief Phase 3/3 -- zero-rate drift.
 *
 * The operator holds the robot still and presses `BTN_UP` to START a fixed logging window
 * (`DRIFT_WINDOW_MS`). Drift is reported as the net wrapped-heading change over that window, in
 * deg/min, decoded with whatever sign/scale the SIGN/SCALE phases measured (or the Config
 * placeholder if a phase was skipped). Report-only -- nothing compensates for it anywhere.
 */
void runDriftPhase()
{
    Serial.println(F("\n=== PHASE 3/3: ZERO-RATE DRIFT ==="));
    Serial.println(F("Hold the robot PERFECTLY STILL, then press BTN_UP to start logging."));

    oledClearHeader("P3 DRIFT");
    MiniR4.OLED.setCursor(0, 8);
    MiniR4.OLED.println(F("Hold robot STILL"));
    MiniR4.OLED.setCursor(0, 16);
    MiniR4.OLED.println(F("then press BTN_UP"));
    MiniR4.OLED.display();

    if (!waitForBtnUpPress(PHASE_TIMEOUT_MS))
    {
        Serial.println(F("[gyro_calib] DRIFT: timed out waiting for BTN_UP -- skipping."));
        return;
    }

    uint8_t hiG = 0, loR = 0;
    readGyroBytes(hiG, loR);
    if (gyroBytesDead(hiG, loR))
    {
        Serial.println(F("[gyro_calib] DRIFT: dead bus (G=R=0) at start -- skipping."));
        return;
    }
    const float startDeg = decodeGyroYawDeg(hiG, loR, _measuredScale, _measuredSign);

    Serial.print(F("[gyro_calib] DRIFT: logging for "));
    Serial.print(DRIFT_WINDOW_MS / 1000);
    Serial.println(F(" s -- keep the robot still."));
    Serial.println(F("  t(ms)   heading(deg)"));

    const uint32_t t0           = millis();
    uint32_t       nextSampleAt = t0;
    while (millis() - t0 < DRIFT_WINDOW_MS)
    {
        if (millis() >= nextSampleAt)
        {
            readGyroBytes(hiG, loR);
            if (!gyroBytesDead(hiG, loR))
            {
                const float nowDeg = decodeGyroYawDeg(hiG, loR, _measuredScale, _measuredSign);
                Serial.print(F("  "));
                Serial.print(millis() - t0);
                Serial.print(F("\t"));
                Serial.println(nowDeg, 2);
            }
            nextSampleAt += DRIFT_SAMPLE_PERIOD_MS;
        }
        delay(POLL_PERIOD_MS);
    }

    readGyroBytes(hiG, loR);
    if (gyroBytesDead(hiG, loR))
    {
        Serial.println(F("[gyro_calib] DRIFT: dead bus (G=R=0) at end -- cannot compute drift."));
        return;
    }
    const float endDeg     = decodeGyroYawDeg(hiG, loR, _measuredScale, _measuredSign);
    const float driftDeg   = gyroShortestDelta(endDeg, startDeg);
    const float elapsedMin = (float) DRIFT_WINDOW_MS / 60000.0f;
    _driftDegPerMin         = driftDeg / elapsedMin;
    _haveDrift              = true;

    Serial.print(F("[gyro_calib] DRIFT: start="));
    Serial.print(startDeg, 2);
    Serial.print(F(" end="));
    Serial.print(endDeg, 2);
    Serial.print(F(" -> drift = "));
    Serial.print(_driftDegPerMin, 3);
    Serial.println(F(" deg/min (report only, no compensation applied)"));
}

/**
 * @brief Serial-print paste-ready `#define ..._DEFAULT` lines for `config/ConfigNav.h`, plus the
 *        drift figure as a report-only comment.
 * @note No trailing `//` comment on either `#define` line -- the Python `config_parse` regex
 *       breaks on a trailing comment if the operator pastes a line verbatim, so any UNVERIFIED
 *       annotation goes on its OWN line instead.
 */
void printResults()
{
    Serial.println(F("\n// --- gyro_calib results (paste into config/ConfigNav.h) ---"));

    if (!_haveSign)
    {
        Serial.println(
            F("// UNVERIFIED -- SIGN phase skipped; below is the unchanged Config placeholder"));
    }
    Serial.print(F("#define GYRO_YAW_SIGN_DEFAULT  ("));
    Serial.print(_measuredSign > 0.0f ? F("+1.0f") : F("-1.0f"));
    Serial.println(F(")"));

    if (!_haveScale)
    {
        Serial.println(
            F("// UNVERIFIED -- SCALE phase skipped; below is the unchanged Config placeholder"));
    }
    Serial.print(F("#define GYRO_SCALE_DEFAULT     "));
    Serial.print(_measuredScale, 4);
    Serial.println(F("f"));

    Serial.println(F("// zero-rate drift (report only -- nothing consumes this at compile time):"));
    if (_haveDrift)
    {
        Serial.print(F("//   "));
        Serial.print(_driftDegPerMin, 3);
        Serial.println(F(" deg/min"));
    }
    else
    {
        Serial.println(F("//   NOT MEASURED -- DRIFT phase was skipped"));
    }
}

/// @brief Run SIGN -> SCALE -> DRIFT in order, then print the paste-ready results. Callable again
///        over Serial ('r') without reflashing -- e.g. for the AC-11 3-run repeatability check.
void runCalibrationSequence()
{
    _haveSign  = false;
    _haveScale = false;
    _haveDrift = false;

    runSignPhase();
    runScalePhase();
    runDriftPhase();
    printResults();

    Serial.println(F("\n[gyro_calib] Sequence complete -- entering LIVE heading display."));
    Serial.println(F("[gyro_calib] '?' help | 'r' re-run | BTN_DOWN zeros the gyro (live mode)."));
}

// =============================================================================================
// LIVE heading display (optional 4th mode -- mirrors the mentor's Gyro_display)
// =============================================================================================

/**
 * @brief Continuously decode + display the current heading using whatever scale/sign the
 *        calibration sequence measured (or the Config placeholder). `BTN_DOWN` zeros the gyro
 *        via `getColor(K)` (a read-with-side-effect; the returned byte is discarded, mirroring
 *        `YawSource::zeroHardware`).
 */
void liveDisplayTick()
{
    static uint32_t lastUpdateMs = 0;
    static bool     prevBtnDown  = false;

    const bool nowBtnDown = MiniR4.BTN_DOWN.getState();
    if (nowBtnDown && !prevBtnDown)
    {
        gyro.getColor(K);    // zero -- side-effecting read, byte discarded
        Serial.println(F("[gyro_calib] LIVE: gyro zeroed (BTN_DOWN)."));
    }
    prevBtnDown = nowBtnDown;

    const uint32_t now = millis();
    if (now - lastUpdateMs < LIVE_UPDATE_PERIOD_MS)
    {
        return;
    }
    lastUpdateMs = now;

    uint8_t hiG = 0, loR = 0;
    readGyroBytes(hiG, loR);
    if (gyroBytesDead(hiG, loR))
    {
        Serial.println(F("[gyro_calib] LIVE: dead bus (G=R=0)."));
        oledClearHeader("LIVE heading");
        MiniR4.OLED.setCursor(0, 12);
        MiniR4.OLED.println(F("DEAD BUS"));
        MiniR4.OLED.display();
        return;
    }

    const float headingDeg = decodeGyroYawDeg(hiG, loR, _measuredScale, _measuredSign);

    Serial.print(F("[gyro_calib] LIVE heading = "));
    Serial.print(headingDeg, 2);
    Serial.println(F(" deg"));

    oledClearHeader("LIVE heading");
    MiniR4.OLED.setCursor(0, 14);
    MiniR4.OLED.setTextSize(2);
    MiniR4.OLED.print(headingDeg, 1);
    MiniR4.OLED.display();
}

/// @brief Poll Serial for the single-char command set (tick_bench / gyro-move style).
void pollSerial()
{
    if (!Serial.available())
    {
        return;
    }
    const int c = Serial.read();
    switch (c)
    {
    case '?':
        printHelp();
        break;
    case 'r':
        runCalibrationSequence();
        break;
    case '\n':
    case '\r':
    case ' ':
        break;    // ignore whitespace
    default:
        Serial.print(F("[?] unknown cmd '"));
        Serial.print((char) c);
        Serial.println(F("' -- press ? for help"));
        break;
    }
}

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }    // give USB CDC a moment, but don't hang headless

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // Wire the gyro's MatrixColor instance onto Wire1 at the configured mux channel BEFORE
    // touching it at all -- the driver caches both fields and uses them on every transaction
    // (WROLineArray precedent). Port LABEL != mux channel: channel is PORT - 1.
    gyro._pWire = &Wire1;
    gyro._ch    = static_cast<uint8_t>(Config::GYRO_I2C_PORT - 1);

    // Tolerate MatrixColor::begin() returning false (assumption #5): it ID-checks
    // Device_ID == 0x43 and writes a CONFIG reset this board may not implement, and the
    // mentor's own code never calls it. A false return here is INCONCLUSIVE, not fatal -- fall
    // through to the raw getColor probe below, the only authoritative liveness check.
    const bool vendorBeginOk = gyro.begin();
    if (!vendorBeginOk)
    {
        Serial.println(F("[gyro_calib] WARN: MatrixColor::begin() returned false -- this board may "
                          "not implement the vendor ID/reset handshake. Proceeding to a raw read "
                          "probe anyway (GyroDecodePolicy.h assumption #5)."));
    }

    uint8_t hiG = 0, loR = 0;
    readGyroBytes(hiG, loR);
    const bool devicePresent = !gyroBytesDead(hiG, loR);

    oledClearHeader("gyro_calib");
    MiniR4.OLED.setCursor(0, 8);
    if (devicePresent)
    {
        Serial.println(F("[gyro_calib] Gyro responding on the configured port -- BOOT OK."));
        MiniR4.OLED.println(F("BOOT OK"));
    }
    else
    {
        Serial.println(F("[gyro_calib] FATAL: gyro read all-zero bytes (G=R=0) on the configured "
                          "port -- dead bus / not wired / wrong port. Check Config::GYRO_I2C_PORT."));
        MiniR4.OLED.println(F("BOOT FAIL"));
        MiniR4.OLED.setCursor(0, 16);
        MiniR4.OLED.println(F("no gyro on port"));
    }
    MiniR4.OLED.display();

    printBanner();

    if (devicePresent)
    {
        runCalibrationSequence();
    }
    else
    {
        Serial.println(F("[gyro_calib] Skipping calibration -- fix the wiring, then send 'r' over "
                          "Serial to retry once the gyro responds."));
    }
}

void loop()
{
    pollSerial();
    liveDisplayTick();
}
