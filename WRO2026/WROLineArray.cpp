/**
 * @file WROLineArray.cpp
 * @brief Implementation of the 8-channel custom line sensor wrapper.
 * @author WRO 2026
 *
 * @details
 * `WROLineArrayClass` owns 1 or 2 `MatrixColor`-protocol I²C boards and exposes the
 * `MatrixLineTracer`-compatible trio (`getError()`, `isOnline()`, `getJunctionType()`)
 * that `LineFollower` and `Intersection` consume. Implementation responsibilities split
 * across the two-layer calibration contract documented in the header:
 *
 *  - **Layer 1 — hardware / illumination calibration.** Performed inside `begin()` via
 *    one `MatrixColor::setLight(state=true, mode=false, pwm=Config::LINE_LED_PWM)` call
 *    per configured board. This latches the on-board LED brightness and triggers the
 *    firmware's internal photoreceptor normalization. The setting is volatile (resets on
 *    power cycle) which is precisely WHY it must live in `begin()` rather than be
 *    flashed once: every boot re-applies it. Do NOT delete this call thinking the
 *    `Config::LINE_CALIB_*` arrays already cover it — they normalise per-channel
 *    spread on top of the firmware baseline, not in place of it.
 *
 *  - **Layer 2 — software per-channel baselines.** `calibrate()` (operator-driven,
 *    sync-blocking ~5 s) sweeps the bar over white + line, captures per-channel
 *    min/max into `Config::LINE_CALIB_LOW` / `LINE_CALIB_HIGH` constants the operator
 *    pastes into `Config.h`, and reflashes. All getter math (`_computeError`,
 *    `_isOnlineFromCache`, `_classifyJunction`) reads these constants to normalise
 *    each channel onto a `[0, 1]` intensity scale before applying the symmetric
 *    weight set or the on-line ratio gate.
 *
 * @note **INFERRED CONTRACT (load-bearing — validate on bring-up).** The CMYK-byte
 *       packing used by `_readBoardInternal()` is inferred from `meat.ino` lines
 *       670–684 (the reference sketch's `light_8()` helper). The MXColor firmware
 *       documentation does not state the packing order. TC9 of the test plan must
 *       validate by physically covering each channel index in turn and confirming
 *       the expected slot in `_lastChannels[]` drops. If TC9 fails, the fix is
 *       local to `_readBoardInternal()`'s unpacking table — no caller-side changes.
 *
 * @note `delay()` is used only inside `calibrate()` (operator-driven, top-level
 *       only, mirrors `Drive.calibrateMotorBias()`). Every getter / `update()` path
 *       remains strictly non-blocking.
 */

#include "WROLineArray.h"

#include "Config.h"

/**
 * @brief Single global instance referenced by `LineFollower`, `Intersection`, and the
 *        sketch's calibration entry points.
 */
WROLineArrayClass LineArray;

// =========================================================================================
// Lifecycle
// =========================================================================================

bool WROLineArrayClass::begin()
{
    // -------------------------------------------------------------------------------------
    // Per-board hardware bring-up. Wire the I²C transport (`Wire1` + mux channel from
    // Config) BEFORE invoking `MatrixColor::begin()` — the driver caches the pointer
    // and the mux channel internally and uses them on every transaction.
    // -------------------------------------------------------------------------------------
    for (uint8_t b = 0; b < Config::LINE_BOARD_COUNT; ++b)
    {
        _boards[b]._pWire = &Wire1;
        _boards[b]._ch    = (b == 0) ? Config::LINE_FRONT_I2C_PORT
                                     : Config::LINE_REAR_I2C_PORT;

        const bool boardOk = _boards[b].begin();
        if (!boardOk)
        {
            // Log to Serial but do NOT halt or return false. A 2-board configuration
            // missing its rear board (operator forgot to plug it in) must still allow
            // the front board to drive a forward-only mission — same fail-soft
            // discipline as the rest of the boot path. The front-board failure case
            // bubbles up via the empty `_lastChannels` cache rather than via this
            // return value (operator catches the "no line ever online" symptom).
            Serial.print(F("[WROLineArray.begin] WARN: board "));
            Serial.print(b);
            Serial.print(F(" (I2C port "));
            Serial.print(_boards[b]._ch);
            Serial.println(F(") begin() returned false."));
        }

        // ---------------------------------------------------------------------------------
        // Layer-1 hardware calibration: latch on-board LED brightness + trigger the
        // firmware's internal photoreceptor normalization. `setLight()` returns void;
        // there is no per-call error code to check. See file-level @note.
        //
        // setLight signature: (state, mode, pwm) — state=true turns the LED on,
        // mode=false selects constant-brightness PWM (true would select a blink mode).
        // ---------------------------------------------------------------------------------
        _boards[b].setLight(true, false, Config::LINE_LED_PWM);
    }

    // -------------------------------------------------------------------------------------
    // Layer-2 sanity check: walk every configured board / channel and warn once on any
    // calibration row where LOW >= HIGH. The most common cause is a botched paste from
    // `calibrate()` output (operator dropped a row, swapped LOW/HIGH, or pasted before
    // the sweep produced real data). Without this guard the dead channel silently
    // contributes zero weight from `_computeError` and zero votes from
    // `_isOnlineFromCache` / `_classifyJunction` — a quiet degradation in tracking
    // quality that's hard to attribute weeks later.
    // -------------------------------------------------------------------------------------
    for (uint8_t b = 0; b < Config::LINE_BOARD_COUNT; ++b)
    {
        for (uint8_t c = 0; c < 8; ++c)
        {
            if (Config::LINE_CALIB_LOW[b][c] >= Config::LINE_CALIB_HIGH[b][c])
            {
                Serial.print(F("[WROLineArray.begin] WARN: dead/inverted calibration on "
                               "board "));
                Serial.print(b);
                Serial.print(F(" channel "));
                Serial.print(c);
                Serial.print(F(" (LOW="));
                Serial.print(Config::LINE_CALIB_LOW[b][c]);
                Serial.print(F(", HIGH="));
                Serial.print(Config::LINE_CALIB_HIGH[b][c]);
                Serial.println(F("). Re-run LineArray.calibrate()."));
            }
        }
    }

    // -------------------------------------------------------------------------------------
    // Reset cached state. Front board active by default; cache zeroed; on-line latch
    // cleared so the first `isOnline()` call after boot reports a clean "no line yet"
    // until the first I²C read populates the cache.
    // -------------------------------------------------------------------------------------
    _activeBoard = 0;
    for (uint8_t c = 0; c < 8; ++c)
    {
        _lastChannels[c] = 0;
    }
    _lastOnline = false;

    return true;
}

// =========================================================================================
// Cooperative tick — optional cache refresh
// =========================================================================================

void WROLineArrayClass::update()
{
    // The three getters refresh internally on every call, so this is purely a "take one
    // I²C transaction per loop tick" optimisation for callers that want predictable
    // bus bandwidth. The transaction is the only blocking thing here (~1-2 ms on the
    // I²C bus); no `delay()` is permitted in this path.
    uint8_t fresh[8];
    if (!_readBoardInternal(_activeBoard, fresh))
    {
        // I²C failure: documented contract — leave `_lastChannels` untouched, clear the
        // on-line latch. The next getter call will report the lost-line state without
        // re-issuing the failed transaction.
        _lastOnline = false;
        return;
    }

    for (uint8_t c = 0; c < 8; ++c)
    {
        _lastChannels[c] = fresh[c];
    }
    _lastOnline = _isOnlineFromCache(_lastChannels);
}

// =========================================================================================
// MatrixLineTracer-compatible getters (refresh-on-call)
// =========================================================================================

float WROLineArrayClass::getError()
{
    // Same refresh path as `update()` — fold the cache update in so callers that skip
    // `update()` still see a fresh sample.
    uint8_t fresh[8];
    if (!_readBoardInternal(_activeBoard, fresh))
    {
        _lastOnline = false;
        return 0.0f;
    }

    for (uint8_t c = 0; c < 8; ++c)
    {
        _lastChannels[c] = fresh[c];
    }
    _lastOnline = _isOnlineFromCache(_lastChannels);

    if (!_lastOnline)
    {
        // Mirror MatrixLineTracer's behaviour: no on-line channel ⇒ zero error so the
        // outer P+D controller commands no correction and the lost-line dispatch
        // (`Config::LOST_LINE_ACTION`) takes over via `isOnline()`.
        return 0.0f;
    }
    return _computeError(_lastChannels);
}

bool WROLineArrayClass::isOnline()
{
    uint8_t fresh[8];
    if (!_readBoardInternal(_activeBoard, fresh))
    {
        _lastOnline = false;
        return false;
    }

    for (uint8_t c = 0; c < 8; ++c)
    {
        _lastChannels[c] = fresh[c];
    }
    _lastOnline = _isOnlineFromCache(_lastChannels);
    return _lastOnline;
}

uint8_t WROLineArrayClass::getJunctionType()
{
    uint8_t fresh[8];
    if (!_readBoardInternal(_activeBoard, fresh))
    {
        _lastOnline = false;
        // Integer literal `0` ⇒ JunctionType::NONE in Intersection.h. We deliberately
        // do not include Intersection.h here — the wrapper is the LOWER layer and
        // pulling in the upper-layer enum would create an unnecessary header cycle.
        return 0;
    }

    for (uint8_t c = 0; c < 8; ++c)
    {
        _lastChannels[c] = fresh[c];
    }
    _lastOnline = _isOnlineFromCache(_lastChannels);

    if (!_lastOnline)
    {
        return 0;    // NONE
    }
    return _classifyJunction(_lastChannels);
}

// =========================================================================================
// Active-board routing (follows `LineFollower::setReverse()`)
// =========================================================================================

void WROLineArrayClass::setReverseActive(bool reverse)
{
    // No-op in 1-board configurations — the front board is always active. Returning
    // early keeps `_activeBoard == 0` invariant for any caller that asks via
    // `getReverseActive()`.
    if (Config::LINE_BOARD_COUNT < 2)
    {
        return;
    }
    _activeBoard = reverse ? (uint8_t) 1 : (uint8_t) 0;
}

bool WROLineArrayClass::getReverseActive() const
{
    return _activeBoard == 1;
}

// =========================================================================================
// Diagnostic raw-channel reads
// =========================================================================================

bool WROLineArrayClass::readActiveBoard(uint8_t out[8])
{
    return _readBoardInternal(_activeBoard, out);
}

bool WROLineArrayClass::readBoard(uint8_t boardIdx, uint8_t out[8])
{
    // Two guards collapse here:
    //  1. Hard cap at the storage bound — `_boards[2]` cannot accept index >= 2.
    //  2. Config bound — even with storage available, a 1-board config has no
    //     initialised `_boards[1]`, so reads would return undefined CMYK bytes.
    if (boardIdx >= 2 || boardIdx >= Config::LINE_BOARD_COUNT)
    {
        return false;
    }
    return _readBoardInternal(boardIdx, out);
}

// =========================================================================================
// Operator-driven Layer-2 calibration sweep (~5 s, sync-blocking)
// =========================================================================================

void WROLineArrayClass::calibrate()
{
    // Allowed `delay()` site — top-level operator action, off the cooperative tick.
    // Same discipline as `Drive::calibrateMotorBias()`: print a banner, run a bounded
    // sync loop, print results, return. NO callers in any `update()` path may invoke
    // this method (would freeze the cooperative loop and break LineFollower /
    // Intersection / Drive queue dispatch).

    // -------------------------------------------------------------------------------------
    // Per-board, per-channel running min/max. Seed `min` to the maximum possible packed
    // value (9) and `max` to the minimum (0) so the very first sample shrinks `min`
    // and grows `max` regardless of what it reads.
    // -------------------------------------------------------------------------------------
    uint8_t mn[2][8];
    uint8_t mx[2][8];
    for (uint8_t b = 0; b < 2; ++b)
    {
        for (uint8_t c = 0; c < 8; ++c)
        {
            mn[b][c] = 9;
            mx[b][c] = 0;
        }
    }

    Serial.println(F("=== WROLineArray.calibrate(): sweep bar over white and over line "
                     "for 5 s... ==="));

    // -------------------------------------------------------------------------------------
    // Sampling loop. ~50 Hz at 20 ms cadence × 5 s ≈ 250 samples per board — plenty of
    // signal for the operator to sweep both extremes without rushing.
    // -------------------------------------------------------------------------------------
    const uint32_t start = millis();
    while ((millis() - start) < 5000U)
    {
        for (uint8_t b = 0; b < Config::LINE_BOARD_COUNT; ++b)
        {
            uint8_t sample[8];
            if (!_readBoardInternal(b, sample))
            {
                // Skip this board on a transient I²C miss; the next iteration will
                // try again. We do NOT bail the whole sweep — a brief mux contention
                // glitch should not invalidate 5 s of operator work.
                continue;
            }
            for (uint8_t c = 0; c < 8; ++c)
            {
                if (sample[c] < mn[b][c])
                {
                    mn[b][c] = sample[c];
                }
                if (sample[c] > mx[b][c])
                {
                    mx[b][c] = sample[c];
                }
            }
        }
        delay(20);
    }

    // -------------------------------------------------------------------------------------
    // Fill rows for boards beyond `LINE_BOARD_COUNT` with safe defaults so the printed
    // block ALWAYS parses as valid C++ (operator can paste it verbatim regardless of
    // current board count). LOW=0 / HIGH=9 matches the Config.h pre-calibration
    // defaults — and the `_isOnlineFromCache` math against unused rows is moot since
    // `setReverseActive()` cannot reach them in a 1-board config anyway.
    // -------------------------------------------------------------------------------------
    for (uint8_t b = Config::LINE_BOARD_COUNT; b < 2; ++b)
    {
        for (uint8_t c = 0; c < 8; ++c)
        {
            mn[b][c] = 0;
            mx[b][c] = 9;
        }
    }

    // -------------------------------------------------------------------------------------
    // Paste-ready output. Match the Config.h declaration form EXACTLY so the operator
    // can drop it in without reformatting:
    //
    //     constexpr uint8_t LINE_CALIB_LOW[2][8]  = { {..}, {..}, };
    //     constexpr uint8_t LINE_CALIB_HIGH[2][8] = { {..}, {..}, };
    // -------------------------------------------------------------------------------------
    Serial.println(F("// Paste this into Config.h:"));

    Serial.println(F("constexpr uint8_t LINE_CALIB_LOW[2][8] = {"));
    for (uint8_t b = 0; b < 2; ++b)
    {
        Serial.print(F("    {"));
        for (uint8_t c = 0; c < 8; ++c)
        {
            Serial.print(mn[b][c]);
            if (c < 7)
            {
                Serial.print(F(","));
            }
        }
        Serial.println(F("},"));
    }
    Serial.println(F("};"));

    Serial.println(F("constexpr uint8_t LINE_CALIB_HIGH[2][8] = {"));
    for (uint8_t b = 0; b < 2; ++b)
    {
        Serial.print(F("    {"));
        for (uint8_t c = 0; c < 8; ++c)
        {
            Serial.print(mx[b][c]);
            if (c < 7)
            {
                Serial.print(F(","));
            }
        }
        Serial.println(F("},"));
    }
    Serial.println(F("};"));

    Serial.println(F("=== calibrate done. Copy the block above into Config.h. ==="));
}

// =========================================================================================
// Private helpers
// =========================================================================================

/**
 * @note **INFERRED CONTRACT — validate on bring-up.** The CMYK-byte → channel-index
 *       table below is inferred from `meat.ino` lines 670–684 (the reference sketch's
 *       `light_8()` author packs the bytes this way when writing to the array). The
 *       MXColor firmware documentation does not state the packing order, so TC9 of
 *       the test plan physically covers each channel in turn and confirms the matching
 *       slot in `_lastChannels[]` drops. If TC9 fails, the fix is local to this method.
 */
bool WROLineArrayClass::_readBoardInternal(uint8_t boardIdx, uint8_t out[8])
{
    // Bounds check first — the public `readBoard()` already guards, but
    // `readActiveBoard()`, `update()`, `getError()`, `isOnline()`,
    // `getJunctionType()`, and `calibrate()` all delegate here too, so the guard
    // must live at the chokepoint.
    if (boardIdx >= 2)
    {
        return false;
    }

    // -------------------------------------------------------------------------------------
    // Read the 4 CMYK bytes. `MatrixColor::getColor(ColorType)` has no error return —
    // an I²C bus failure surfaces as a 0 byte, which is indistinguishable from a fully
    // saturated "off-line" reading. We accept the read at face value and return true;
    // higher layers detect the all-zero pathology via `_isOnlineFromCache` returning
    // false (the calibrated LOW baseline is typically non-zero so 0 lands below the
    // on-line ratio).
    // -------------------------------------------------------------------------------------
    const uint8_t bc = _boards[boardIdx].getColor(C);
    const uint8_t bm = _boards[boardIdx].getColor(M);
    const uint8_t by = _boards[boardIdx].getColor(Y);
    const uint8_t bk = _boards[boardIdx].getColor(K);

    // -------------------------------------------------------------------------------------
    // Unpack per the inferred table (see method @note above + header doxygen):
    //   C byte: tens digit → ch0 (leftmost),  ones digit → ch1
    //   M byte: tens digit → ch2,             ones digit → ch3
    //   Y byte: tens digit → ch4,             ones digit → ch5
    //   K byte: tens digit → ch6,             ones digit → ch7 (rightmost)
    // -------------------------------------------------------------------------------------
    out[0] = (uint8_t) (bc / 10);
    out[1] = (uint8_t) (bc % 10);
    out[2] = (uint8_t) (bm / 10);
    out[3] = (uint8_t) (bm % 10);
    out[4] = (uint8_t) (by / 10);
    out[5] = (uint8_t) (by % 10);
    out[6] = (uint8_t) (bk / 10);
    out[7] = (uint8_t) (bk % 10);

    return true;
}

float WROLineArrayClass::_computeError(const uint8_t channels[8]) const
{
    // Symmetric weights centred on the gap between ch3 and ch4. Negative weights pull
    // the chassis LEFT (line is left-of-centre); positive weights pull RIGHT. Magnitude
    // grows linearly toward the outer channels so an outer-edge detection swings the
    // correction harder than an inner-edge one — same scheme as MatrixLineTracer, just
    // narrowed for 8 channels (vendor was 10).
    static constexpr float _sensorWeights[8] = {
        -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f
    };

    float weightedSum = 0.0f;
    float totalWeight = 0.0f;

    for (uint8_t c = 0; c < 8; ++c)
    {
        const uint8_t low  = Config::LINE_CALIB_LOW[_activeBoard][c];
        const uint8_t high = Config::LINE_CALIB_HIGH[_activeBoard][c];

        // Dead-channel handling: a botched calibration paste (LOW >= HIGH) is
        // treated as permanently off-line, contributing neither weight nor norm.
        // This matches `_isOnlineFromCache` so a single bad channel cannot silently
        // bias the weighted centre.
        if (low >= high)
        {
            continue;
        }

        // Normalise into [0, 1]. The raw byte can momentarily fall below LOW or
        // exceed HIGH if the operator's calibration sweep missed an extreme — clamp
        // so the weight contribution stays bounded and the controller does not see
        // out-of-range error magnitudes.
        float norm = (float) (channels[c] - low) / (float) (high - low);
        if (norm < 0.0f)
        {
            norm = 0.0f;
        }
        else if (norm > 1.0f)
        {
            norm = 1.0f;
        }

        // Only channels above the ON ratio contribute. A channel reading just above
        // the LOW baseline (low norm) is almost certainly background noise and
        // including it would pull the weighted centre toward whichever outer
        // channel happens to be slightly noisier. Compare to `> ratio` (strict) to
        // match `_isOnlineFromCache` and avoid a ratio==0.5 + noisy floor edge case.
        if (norm > Config::LINE_CHANNEL_ON_RATIO)
        {
            weightedSum += norm * _sensorWeights[c];
            totalWeight += norm;
        }
    }

    return (totalWeight > 0.0f) ? (weightedSum / totalWeight) : 0.0f;
}

bool WROLineArrayClass::_isOnlineFromCache(const uint8_t channels[8]) const
{
    for (uint8_t c = 0; c < 8; ++c)
    {
        const uint8_t low  = Config::LINE_CALIB_LOW[_activeBoard][c];
        const uint8_t high = Config::LINE_CALIB_HIGH[_activeBoard][c];

        // Dead channel (see `_computeError` note) — skip.
        if (low >= high)
        {
            continue;
        }

        float norm = (float) (channels[c] - low) / (float) (high - low);
        if (norm < 0.0f)
        {
            norm = 0.0f;
        }
        else if (norm > 1.0f)
        {
            norm = 1.0f;
        }

        // Early exit on the first qualifying channel — `isOnline()` semantics is
        // "ANY channel exceeds the ratio", not a vote count.
        if (norm > Config::LINE_CHANNEL_ON_RATIO)
        {
            return true;
        }
    }
    return false;
}

uint8_t WROLineArrayClass::_classifyJunction(const uint8_t channels[8]) const
{
    // -------------------------------------------------------------------------------------
    // Stage 1 — materialise per-channel ON booleans using the same normalisation +
    // ratio rule as `_isOnlineFromCache` / `_computeError`. Doing this once up front
    // lets the predicate evaluators below stay simple boolean reductions and ensures
    // all three predicates see EXACTLY the same on/off interpretation of the sample
    // (no risk of a numeric drift between predicates).
    //
    // Dead channels (LOW >= HIGH) collapse to `false` — same fail-soft contract as
    // the on-line getter. A botched paste therefore never silently lights up a
    // junction predicate just because the calibration constants happen to compare
    // favourably against an arbitrary raw value.
    // -------------------------------------------------------------------------------------
    bool on[8];
    for (uint8_t c = 0; c < 8; ++c)
    {
        const uint8_t low  = Config::LINE_CALIB_LOW[_activeBoard][c];
        const uint8_t high = Config::LINE_CALIB_HIGH[_activeBoard][c];

        if (low >= high)
        {
            on[c] = false;
            continue;
        }

        float norm = (float) (channels[c] - low) / (float) (high - low);
        if (norm < 0.0f)
        {
            norm = 0.0f;
        }
        else if (norm > 1.0f)
        {
            norm = 1.0f;
        }

        on[c] = (norm > Config::LINE_CHANNEL_ON_RATIO);
    }

    // -------------------------------------------------------------------------------------
    // Stage 2 — priority-ordered predicates. Return as soon as one fires so the result
    // is deterministic (no later predicate can override an earlier match). The order
    // matches the header doxygen: T_OR_CROSS → LEFT → RIGHT → NONE.
    // -------------------------------------------------------------------------------------

    // ---- Predicate 1: T_OR_CROSS (3) — fat marker lighting up most of the bar. ----
    uint8_t onCount = 0;
    for (uint8_t c = 0; c < 8; ++c)
    {
        if (on[c])
        {
            ++onCount;
        }
    }
    if (onCount >= Config::LINE_JUNCTION_T_MIN_ON)
    {
        return 3;
    }

    // ---- Predicate 2: LEFT (1) — channels 0..LEFT_HI all ON, rest all OFF. ----
    {
        bool ok = true;
        // Left half must be fully lit.
        for (uint8_t c = 0; c <= Config::LINE_JUNCTION_LEFT_HI && c < 8; ++c)
        {
            if (!on[c])
            {
                ok = false;
                break;
            }
        }
        if (ok)
        {
            // Right half must be fully dark.
            for (uint8_t c = (uint8_t) (Config::LINE_JUNCTION_LEFT_HI + 1); c < 8; ++c)
            {
                if (on[c])
                {
                    ok = false;
                    break;
                }
            }
        }
        if (ok)
        {
            return 1;
        }
    }

    // ---- Predicate 3: RIGHT (2) — channels RIGHT_LO..7 all ON, rest all OFF. ----
    {
        bool ok = true;
        // Right half must be fully lit.
        for (uint8_t c = Config::LINE_JUNCTION_RIGHT_LO; c < 8; ++c)
        {
            if (!on[c])
            {
                ok = false;
                break;
            }
        }
        if (ok && Config::LINE_JUNCTION_RIGHT_LO > 0)
        {
            // Left half must be fully dark. Guarded against `RIGHT_LO == 0` (which
            // would underflow the loop bound below and is a degenerate "every channel
            // is in the right half" config — left half is then trivially empty).
            for (uint8_t c = 0; c < Config::LINE_JUNCTION_RIGHT_LO; ++c)
            {
                if (on[c])
                {
                    ok = false;
                    break;
                }
            }
        }
        if (ok)
        {
            return 2;
        }
    }

    // ---- Predicate 4: NONE (0) — fallthrough. ----
    return 0;
}
