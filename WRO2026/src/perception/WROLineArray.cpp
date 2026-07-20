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
 *    sync-blocking ~5 s in 1-board configs, 10+ s in 2-board configs because the
 *    front and rear bars must be swept sequentially with an operator-paced BTN_UP
 *    gate between phases) sweeps the bar over white +
 *    line, captures per-channel min/max into `Config::LINE_CALIB_LOW` /
 *    `LINE_CALIB_HIGH` constants the operator pastes into `Config.h`, and
 *    reflashes. All getter math (`_computeError`, `_isOnlineFromCache`,
 *    `_classifyJunction`) reads these constants to normalise each channel onto a
 *    `[0, 1]` intensity scale before applying the symmetric weight set or the
 *    on-line ratio gate.
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

#include "../../Config.h"

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
    // Front-board health accumulator. The FRONT board is mandatory (default driving
    // direction); a failure here propagates out of `begin()` as `false` so the sketch's
    // `_setupFailed` boot health gate catches it and the operator sees a hard boot
    // failure instead of `BOOT OK` with a dead sensor. The REAR board is optional by
    // design (only used when LineFollower.setReverse(true) AND LINE_BOARD_COUNT == 2),
    // so its failure stays warn-only and does NOT affect this accumulator.
    // -------------------------------------------------------------------------------------
    bool frontOk = true;

    // -------------------------------------------------------------------------------------
    // Per-board hardware bring-up. Wire the I²C transport (`Wire1` + mux channel from
    // Config) BEFORE invoking `MatrixColor::begin()` — the driver caches the pointer
    // and the mux channel internally and uses them on every transaction.
    // -------------------------------------------------------------------------------------
    for (uint8_t b = 0; b < Config::LINE_BOARD_COUNT; ++b)
    {
        _boards[b]._pWire = &Wire1;

        // ---------------------------------------------------------------------------------
        // Convert user-facing 1..4 port labels (matching the physical I2C1..I2C4
        // markings on the MiniR4 board AND meat.ino's MiniR4.I2C{N}.MXColor
        // pattern) to the 0-based PCA9548A mux channel ID that MatrixColor::_ch
        // expects. The vendor convention (MatrixMiniR4.h:96-100):
        //   I2C1 → mux ch 0, I2C2 → ch 1, I2C3 → ch 2, I2C4 → ch 3.
        // MatrixColor::i2cMUXSelect() writes (1 << _ch), so passing the raw 1..4
        // would select the WRONG physical port (e.g., port label 4 → ch 4, which
        // doesn't exist on a MiniR4). The old MatrixLineTracer hid this bug by
        // no-op'ing its mux-select. See PR #20 codex P2 + DECISIONS.md.
        // ---------------------------------------------------------------------------------
        const uint8_t portLabel = (b == 0) ? Config::LINE_FRONT_I2C_PORT
                                           : Config::LINE_REAR_I2C_PORT;
        _boards[b]._ch          = static_cast<uint8_t>(portLabel - 1);

        const bool boardOk = _boards[b].begin();
        if (!boardOk)
        {
            // Log to Serial unconditionally — the warning is useful diagnostic context
            // regardless of which board failed. Front-board failure additionally flips
            // `frontOk` so the return value below signals the sketch's `_setupFailed`
            // boot health gate; rear-board failure in a 2-board configuration is
            // warn-only by design (operator can still run a forward-only mission on
            // the front alone — same fail-soft discipline as the rest of the boot
            // path). We deliberately do NOT `continue` here: the `setLight()` call
            // below remains best-effort even on a failed `begin()` since the LED may
            // still be controllable on a partially-initialised board.
            //
            // Log the user-facing port label (1..4) — not the 0-based mux channel —
            // so the operator can match the warning to the physical I2C1..I2C4
            // silkscreen on the board without doing the conversion in their head.
            Serial.print(F("[WROLineArray.begin] WARN: board "));
            Serial.print(b);
            Serial.print(F(" (I2C port "));
            Serial.print(portLabel);
            Serial.println(F(") begin() returned false."));

            if (b == 0)
            {
                frontOk = false;
            }
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

    // Front-board failure surfaces here so the sketch's `_setupFailed` boot health
    // gate catches it; rear-board failure was already logged warn-only above.
    return frontOk;
}

// =========================================================================================
// Cooperative tick — toggle-driven cache refresh
// =========================================================================================

void WROLineArrayClass::update()
{
    if (!Config::LINE_ARRAY_TICK_CACHE_ENABLED)
    {
        // Legacy path: update() is a no-op. The three getters refresh
        // independently on every call (current PR #20 behaviour). Leaving
        // update() as a no-op here keeps cooperativeTick() unchanged when
        // the toggle is flipped at compile time — no extra I²C traffic.
        return;
    }
    _refreshCache();
}

// =========================================================================================
// MatrixLineTracer-compatible getters (cache-read when toggle ON, refresh-on-call when OFF)
// =========================================================================================

float WROLineArrayClass::getError()
{
    // When the tick-cache toggle is OFF (legacy / default), refresh inline so
    // callers that skip `update()` still see a fresh sample — preserves the
    // current PR #20 behaviour byte-for-byte. When the toggle is ON, the cache
    // was already populated by `update()` at the top of the cooperative tick;
    // we read it directly without any further I²C traffic. The compiler
    // dead-code-eliminates whichever branch the constexpr toggle disables.
    if (!Config::LINE_ARRAY_TICK_CACHE_ENABLED)
    {
        _refreshCache();
    }
    // Mirror MatrixLineTracer's behaviour: no on-line channel ⇒ zero error so the
    // outer P+D controller commands no correction and the lost-line dispatch
    // (`Config::LOST_LINE_ACTION`) takes over via `isOnline()`.
    return _lastOnline ? _computeError(_lastChannels) : 0.0f;
}

bool WROLineArrayClass::isOnline()
{
    if (!Config::LINE_ARRAY_TICK_CACHE_ENABLED)
    {
        _refreshCache();
    }
    return _lastOnline;
}

uint8_t WROLineArrayClass::getJunctionType()
{
    if (!Config::LINE_ARRAY_TICK_CACHE_ENABLED)
    {
        _refreshCache();
    }
    // Integer literal `0` ⇒ JunctionType::NONE in Intersection.h. We deliberately
    // do not include Intersection.h here — the wrapper is the LOWER layer and
    // pulling in the upper-layer enum would create an unnecessary header cycle.
    return _lastOnline ? _classifyJunction(_lastChannels) : (uint8_t) 0;
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

    const uint8_t newActiveBoard = reverse ? (uint8_t) 1 : (uint8_t) 0;
    if (newActiveBoard == _activeBoard)
    {
        // Direction unchanged (e.g. a redundant `setReverse(true)` while already
        // reversed) — skip the refresh below so a no-op flip never issues an
        // extra I2C transaction outside the cooperative tick's `update()`
        // chokepoint.
        return;
    }
    _activeBoard = newActiveBoard;

    // -----------------------------------------------------------------------------------
    // Stale-cache-on-flip fix (#149). When `Config::LINE_ARRAY_TICK_CACHE_ENABLED` is
    // `true`, `getError()` / `isOnline()` / `getJunctionType()` are pure cache readers for
    // the remainder of the tick — they do NOT re-sample the active board (see `update()`
    // doxygen). Without this refresh, a same-tick read taken right after this flip would
    // return `_lastChannels` / `_lastOnline` as latched from the board that was active
    // BEFORE the flip, silently reporting the OLD board's line state under the NEW
    // board's identity. Refreshing immediately here closes that window so any getter
    // called later in the same tick observes the NEW board's sample.
    //
    // When the toggle is `false` (legacy path), every getter already refreshes inline on
    // each call, so this refresh would be redundant I2C traffic — skip it.
    // -----------------------------------------------------------------------------------
    if (Config::LINE_ARRAY_TICK_CACHE_ENABLED)
    {
        _refreshCache();
    }
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

bool WROLineArrayClass::probeBoard(uint8_t boardIdx)
{
    // Same hard bounds as readBoard(): storage cap first, then configured-board cap.
    if (boardIdx >= 2 || boardIdx >= Config::LINE_BOARD_COUNT)
    {
        return false;
    }

    TwoWire* const pWire = _boards[boardIdx]._pWire;
    const uint8_t  ch    = _boards[boardIdx]._ch;

    // -------------------------------------------------------------------------------------
    // Mirrors MatrixColor::i2cMUXSelect() (MiniR4ColorSensorExt.cpp ~line 189), which is
    // PRIVATE to MatrixColor and therefore not callable from here: select the board's
    // PCA9548A mux channel, then hold the vendor's own 300 us settle before touching the
    // device address. This writes to the MUX, not to the MXColor board being probed —
    // no device register on the board itself is touched by this step.
    // -------------------------------------------------------------------------------------
    pWire->beginTransmission(ADDR_PCA954X);
    pWire->write((1 << ch));
    pWire->endTransmission(1);
    delayMicroseconds(300);

    // -------------------------------------------------------------------------------------
    // Value-blind liveness probe: a bare address ACK, nothing more. No register write,
    // no setLight(...), no MatrixColor::begin() (which would reset Device_CONFIG and
    // destroy the Layer-1 firmware illumination calibration begin() latched — see the
    // two-layer calibration contract in the file-level doxygen). `_lastChannels`,
    // `_lastOnline`, and `_activeBoard` are all left untouched by this method.
    // -------------------------------------------------------------------------------------
    pWire->beginTransmission(MatrixColor_ADDR);
    return pWire->endTransmission() == 0;
}

// =========================================================================================
// Operator-driven Layer-2 calibration sweep (~5 s × N boards, sequential, sync-blocking)
// =========================================================================================

void WROLineArrayClass::_sweepOneBoardForCalibration(uint8_t   boardIdx,
                                                     uint8_t   mnRow[8],
                                                     uint8_t   mxRow[8],
                                                     uint16_t& validCount)
{
    // -------------------------------------------------------------------------------------
    // Single-board, 5-second sampling phase used by `calibrate()` to sweep boards
    // sequentially. Front and rear bars sit at opposite ends of the chassis, so the
    // operator physically cannot present both to the reference at the same time — the
    // previous monolithic "read every configured board every 20 ms" loop recorded
    // ambient garbage on whichever bar was off-target. Splitting the inner per-board
    // branch into this helper lets `calibrate()` dispatch phases one-board-at-a-time
    // with an inter-phase repositioning cue.
    //
    // Sampling cadence matches the original sweep (~50 Hz over 5 s ≈ 250 samples per
    // call), so the per-board statistical quality is identical to the pre-refactor
    // behaviour — only the wall-clock arrangement of phases has changed.
    // -------------------------------------------------------------------------------------
    const uint32_t start = millis();
    while ((millis() - start) < 5000U)
    {
        uint8_t sample[8];
        if (!_readBoardInternal(boardIdx, sample))
        {
            // Transient I²C miss; the next iteration will retry. validCount
            // is NOT bumped — if every iteration fails, calibrate()'s
            // post-sweep handler substitutes safe defaults for this row.
            delay(20);
            continue;
        }
        ++validCount;
        for (uint8_t c = 0; c < 8; ++c)
        {
            if (sample[c] < mnRow[c])
            {
                mnRow[c] = sample[c];
            }
            if (sample[c] > mxRow[c])
            {
                mxRow[c] = sample[c];
            }
        }
        delay(20);
    }
}

void WROLineArrayClass::calibrate()
{
    // Allowed `delay()` site — top-level operator action, off the cooperative tick.
    // Same discipline as `Drive::calibrateMotorBias()`: print a banner, run a bounded
    // sync loop, print results, return. NO callers in any `update()` path may invoke
    // this method (would freeze the cooperative loop and break LineFollower /
    // Intersection / Drive queue dispatch).

    // -------------------------------------------------------------------------------------
    // Layer 1 — pre-flight connectivity check (Diego's PR #20 review, fix #7).
    //
    // One read per configured board BEFORE the 5 s sweep, so the operator does not waste
    // 5 s sweeping over a dead bar (unplugged, wrong port, mux mis-wired). This catches
    // the 99 % failure mode at the very start of `calibrate()` rather than after the
    // sweep, when the only signal would have been seed `mn={9..}` / `mx={0..}` arrays
    // that paste as the dead-channel pattern (LOW >= HIGH on every channel).
    //
    // Abort criterion: ANY configured board failing to respond cancels the whole sweep —
    // we do NOT print a partial calibration block, because the operator may not notice
    // a missing row and could paste a half-calibrated config that silently drives the
    // robot reverse-only (or forward-only) on a bad sensor.
    //
    // The CMYK byte > 99 guard inside `_readBoardInternal` (previous commit) feeds this
    // pre-flight naturally — a corrupt-byte response is also a `false` return.
    // -------------------------------------------------------------------------------------
    for (uint8_t b = 0; b < Config::LINE_BOARD_COUNT; ++b)
    {
        uint8_t preflight[8];
        if (!_readBoardInternal(b, preflight))
        {
            // Print the user-facing 1..4 port label (matches silkscreen + Config) — not
            // the 0-based mux channel — so the operator can match the warning to the
            // physical I2C1..I2C4 marking without doing the conversion in their head.
            // See DECISIONS.md discussion of the 1..4 → 0..3 mux conversion that
            // happens in `begin()`.
            const uint8_t portLabel = (b == 0) ? Config::LINE_FRONT_I2C_PORT
                                               : Config::LINE_REAR_I2C_PORT;
            Serial.print(F("[WROLineArray.calibrate] ABORT: board "));
            Serial.print(b);
            Serial.print(F(" (port I2C"));
            Serial.print(portLabel);
            Serial.println(F(") not responding. Check wiring/port label. "
                             "No values printed; re-run after fixing."));
            return;
        }
    }

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

    // -------------------------------------------------------------------------------------
    // Layer 2 — per-board valid-sample counter (Diego's PR #20 review, fix #7).
    //
    // Counts successful reads from each board across its 5 s phase. After the sweep,
    // any board with `validSamples[b] == 0` (passed pre-flight but ALL 250 in-sweep
    // reads failed — intermittent connection, brownout under LED load, etc.) gets a
    // per-board warning + safe-default row in the printed paste block instead of the
    // seed `mn`/`mx` values. Safe defaults (LOW=0, HIGH=9) match the Config.h
    // pre-calibration baseline — non-inverted, so they will NOT trigger the dead-
    // channel warning at next boot — but they also give lost-line behaviour on that
    // board, so the operator will notice and re-run `calibrate()`.
    //
    // Only indices `[0..LINE_BOARD_COUNT-1]` are touched; the size-2 array gives us a
    // fixed stack layout regardless of board count.
    // -------------------------------------------------------------------------------------
    uint16_t validSamples[2] = {0, 0};

    // -------------------------------------------------------------------------------------
    // Phase 1: sweep the FRONT board (boards[0]) only. Operator presents the FRONT bar
    // to the white + line reference during this 5 s window. Front and rear bars sit at
    // opposite ends of the chassis and cannot be physically swept at once — sequencing
    // the phases is what makes the rear-board calibration meaningful (the previous
    // monolithic loop recorded ambient garbage on the inactive bar).
    // -------------------------------------------------------------------------------------
    if (Config::LINE_BOARD_COUNT >= 2)
    {
        Serial.println(F("=== PHASE 1/2: sweep FRONT bar over white and line for 5 s... ==="));
    }
    else
    {
        Serial.println(F("=== PHASE 1/1: sweep FRONT bar over white and line for 5 s... ==="));
    }
    _sweepOneBoardForCalibration(0, mn[0], mx[0], validSamples[0]);

    // -------------------------------------------------------------------------------------
    // Phase 2: only when a REAR board is configured. Audible cue + operator-gated wait
    // gives the operator UNBOUNDED time to physically reposition the chassis or the
    // reference strip so the REAR bar is now over the target. Then a second 5 s sweep
    // on boards[1]. Previously this was a 3 s auto-countdown — replaced (Diego's PR #20
    // review) with an explicit BTN_UP press gate because the timer pressured rushed /
    // bad calibration. 60 s hard timeout caps the unbounded wait so the routine can't
    // hang indefinitely if the operator wanders off.
    // -------------------------------------------------------------------------------------
    if (Config::LINE_BOARD_COUNT >= 2)
    {
        Serial.println(F("=== PHASE 1 done. Reposition for REAR bar..."));

        // Two short beeps to alert the operator. Frequencies chosen distinct from
        // the Mission fault-beep pattern (1 kHz × 500 ms) so they're not confused
        // with a fault.
        MiniR4.Buzzer.Tone(1200, 200);
        delay(300);
        MiniR4.Buzzer.Tone(1200, 200);

        // Wait for the operator to confirm by pressing BTN_UP. 60 s timeout
        // so the routine can't hang indefinitely if the operator wanders off.
        // Rising-edge detection: ignore the BTN_UP state when entering this
        // loop in case it's already pressed; require a confirmed release-then-
        // press before accepting.
        Serial.println(F("  Press BTN_UP when REAR bar is positioned, "
                         "or wait 60 s to abort..."));
        const uint32_t waitStart = millis();
        bool btnSeenReleased     = false;
        bool phase2Confirmed     = false;
        while ((millis() - waitStart) < 60000U)
        {
            const bool btnUpNow = MiniR4.BTN_UP.getState();
            if (!btnUpNow)
            {
                btnSeenReleased = true;
            }
            else if (btnSeenReleased)
            {
                // Rising edge after at least one observed release.
                phase2Confirmed = true;
                break;
            }
            delay(20);   // Polling cadence; matches the calibrate() sweep tick.
        }

        if (!phase2Confirmed)
        {
            Serial.println(F("[WROLineArray.calibrate] PHASE 2 ABORTED: "
                             "BTN_UP not pressed within 60 s. REAR row will "
                             "use safe defaults; re-run calibrate() when ready."));
            // Fall through — the post-sweep Layer-2 safety net will substitute
            // LOW=0/HIGH=9 for board 1 because validSamples[1] is still zero.
            // No explicit overwrite needed here.
        }
        else
        {
            Serial.println(F("=== PHASE 2/2: sweep REAR bar over white and "
                             "line for 5 s... ==="));
            _sweepOneBoardForCalibration(1, mn[1], mx[1], validSamples[1]);
        }
    }

    // -------------------------------------------------------------------------------------
    // Post-sweep: substitute safe defaults for any CONFIGURED board that produced zero
    // valid samples during the sweep (intermittent connection / brownout after the
    // pre-flight check passed). LOW=0 / HIGH=9 matches the Config.h pre-calibration
    // defaults; non-inverted so the boot-time dead-channel warning in `begin()` will
    // NOT fire — but the operator will still see lost-line behaviour on that board
    // and know calibration needs another run.
    //
    // Print one warning per affected board BEFORE the paste block so the operator
    // notices the issue at the top of the output, not buried inside a "// Paste this"
    // banner.
    // -------------------------------------------------------------------------------------
    for (uint8_t b = 0; b < Config::LINE_BOARD_COUNT; ++b)
    {
        if (validSamples[b] == 0)
        {
            Serial.print(F("[WROLineArray.calibrate] WARN: board "));
            Serial.print(b);
            Serial.println(F(" produced NO valid samples during the sweep "
                             "(intermittent connection? brownout?). "
                             "Skipping its row — safe defaults emitted so the "
                             "paste still parses, but you MUST re-run calibrate()."));
            for (uint8_t c = 0; c < 8; ++c)
            {
                mn[b][c] = 0;
                mx[b][c] = 9;
            }
        }
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

void WROLineArrayClass::_refreshCache()
{
    // Single chokepoint shared by `update()` (caching path) and the three getters
    // (legacy refresh-on-call path). Consolidating the read + cache + on-line
    // latch here guarantees the two modes see exactly the same I²C-failure
    // semantics and the same `_lastChannels` / `_lastOnline` state transitions.
    // Safe to pass _lastChannels directly as the out buffer: _readBoardInternal
    // returns false BEFORE any write to out[] on both of its failure paths today
    // (the boardIdx >= 2 bounds check, AND the CMYK byte > 99 transport-corruption
    // guard added in the previous commit). The last-known-good invariant in
    // _lastChannels therefore survives any failure. If _readBoardInternal ever
    // grows a partial-write failure path, restore the temporary-buffer + memcpy
    // pattern to preserve this invariant.
    if (_readBoardInternal(_activeBoard, _lastChannels))
    {
        _lastOnline = _isOnlineFromCache(_lastChannels);
    }
    else
    {
        _lastOnline = false;
        // _lastChannels deliberately left as-is — last-known-good values
        // remain available; the offline flag is the canonical signal.
    }
}

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
    // Reject obviously-corrupt CMYK bytes (P1 from PR #20 codex review).
    // Each packed byte must be in [0..99] (two decimal digits 0..9). On I²C
    // bus failure / dropout MatrixColor::getColor() can return 0xFF (=255)
    // since the underlying i2cReadData has no transport-status check. Letting
    // 0xFF through would unpack to (25, 5) and downstream _computeError would
    // see norm values >> 1 (above the HIGH calibration of 9), falsely
    // reporting a very strong on-line and silently defeating lost-line safety.
    //
    // Return false WITHOUT touching out[] so the caller's last-known-good
    // cache is preserved (matches the existing failure-path contract).
    // -------------------------------------------------------------------------------------
    if (bc > 99 || bm > 99 || by > 99 || bk > 99)
    {
        return false;
    }

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

float WROLineArrayClass::_onnessNorm(uint8_t raw, uint8_t low, uint8_t high)
{
    // The followed line is DARK on a light field, so a channel is "more on-line" the
    // closer its reading falls to the calibrated dark endpoint (`low`, the per-channel
    // minimum) and away from the white endpoint (`high`, the per-channel maximum):
    // 1.0 at the darkest calibrated value, 0.0 at the brightest, clamped to [0, 1].
    // Callers pre-reject dead channels (`low >= high`) so the denominator is positive.
    // The `(int)` promotion keeps the subtraction signed, so a raw byte that momentarily
    // reads brighter than `high` (or darker than `low`) clamps to the range instead of
    // underflowing the unsigned arithmetic.
    float onness = (float) ((int) high - (int) raw) / (float) ((int) high - (int) low);
    if (onness < 0.0f)
    {
        onness = 0.0f;
    }
    else if (onness > 1.0f)
    {
        onness = 1.0f;
    }
    return onness;
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

        // Normalised on-line-ness in [0, 1]: 1.0 = channel over the dark line, 0.0 =
        // over the white field. `_onnessNorm` owns the dark-line polarity + clamping.
        const float norm = _onnessNorm(channels[c], low, high);

        // Only channels above the ON ratio contribute. A channel with only weak
        // on-line-ness (norm just above 0 — reading near white, off the line) is
        // almost certainly background and including it would pull the weighted centre
        // toward whichever outer channel happens to be slightly noisier. Compare to
        // `> ratio` (strict) to match `_isOnlineFromCache` and avoid a ratio==0.5 +
        // noisy floor edge case.
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

        const float norm = _onnessNorm(channels[c], low, high);

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

        const float norm = _onnessNorm(channels[c], low, high);

        on[c] = (norm > Config::LINE_CHANNEL_ON_RATIO);
    }

    // -------------------------------------------------------------------------------------
    // Stage 2 — priority-ordered predicates. Return as soon as one fires so the result
    // is deterministic (no later predicate can override an earlier match). The order
    // matches the header doxygen: T_OR_CROSS → LEFT → RIGHT → NONE.
    //
    // LEFT and RIGHT each test only their OWN half (the opposite side is DON'T-CARE), so
    // the ONLY sample that could satisfy both is a fully lit bar — which trips T_OR_CROSS
    // first and returns before either is reached (holds while RIGHT_LO <= LEFT_HI + 1, the
    // no-gap default). Determinism therefore survives dropping the opposite-side-off checks.
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

    // ---- Predicate 2: LEFT (1) — channels 0..LEFT_HI all ON; right side DON'T-CARE. ----
    //
    // Only the left edge is required lit. The opposite (right) half is deliberately NOT
    // required dark: a one-sided branch keeps its own edge channels lit for the whole
    // crossing while the centre channels flicker on and off as the chassis sweeps over it,
    // so the earlier exact-partition rule ("right half all OFF") dropped the verdict to NONE
    // for most of the crossing and the debounce never armed at speed. The LEFTMOST channel
    // (ch0) being lit is the discriminator a centred straight line can never produce, so
    // dropping the opposite-side check stays false-fire-safe on a plain line.
    {
        bool ok = true;
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
            return 1;
        }
    }

    // ---- Predicate 3: RIGHT (2) — channels RIGHT_LO..7 all ON; left side DON'T-CARE. ----
    //
    // Mirror of LEFT above: only the right edge is required lit, the left half is ignored.
    // The RIGHTMOST channel (ch7) being lit is the discriminator a centred line never trips.
    {
        bool ok = true;
        for (uint8_t c = Config::LINE_JUNCTION_RIGHT_LO; c < 8; ++c)
        {
            if (!on[c])
            {
                ok = false;
                break;
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
