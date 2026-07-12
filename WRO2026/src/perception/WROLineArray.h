/**
 * @file WROLineArray.h
 * @brief 8-channel reflectance line-array sensor wrapper backed by 1 or 2 MXColor boards.
 * @author WRO 2026
 *
 * @details
 * `WROLineArray` is the WRO 2026 competition replacement for the vendor `MatrixLineTracer`
 * 10-channel line sensor. It owns 1 or 2 `MatrixColor`-protocol I2C boards (front, optional
 * rear), unpacks each board's 4 CMYK bytes into 8 reflectance channels (each 0..9, higher =
 * brighter/whiter, lower = darker; the followed line is DARK, so a LOW reading means the
 * channel is over the line), applies per-channel software calibration on top of the
 * firmware-level hardware calibration, and exposes the exact three-method surface
 * `MatrixLineTracer` did — `getError()`, `isOnline()`, `getJunctionType()` — so it drops in
 * to `LineFollower` and `Intersection` with no controller-side logic changes.
 *
 * Why a custom class:
 *  - Hardware ships with two physical 8-channel boards (front + rear) for bidirectional
 *    line-following — the vendor 10-channel tracer is a single-board part.
 *  - The 8-channel boards use the MXColor wire protocol, packing two channel values into
 *    each of the four CMYK bytes (tens digit + ones digit). The vendor library does not
 *    expose this as a line array — only as a colour sensor — so the unpacking logic has
 *    to live here. See `_readBoardInternal()` and the cross-reference to TC9.
 *  - The "interface" is intentionally a stable trio of concrete method signatures, NOT a
 *    virtual base class — see DECISIONS.md "8-channel custom line sensor replaces
 *    MXLineTracer" for the rationale (RAM cost, no other backend planned, compile-time-only
 *    swap). `LineFollower._array` and `Intersection._array` are `WROLineArray*` directly.
 *
 * Two-layer calibration contract (LOAD-BEARING — DO NOT delete one layer thinking it is
 * redundant with the other):
 *  - **Layer 1 — firmware-level hardware calibration / illumination.** Each `MatrixColor`
 *    board exposes a `setLight(...)` command that latches the on-board LED brightness and
 *    triggers the firmware's internal photoreceptor normalization. `begin()` invokes this
 *    on every configured board exactly once, sourcing brightness from
 *    `Config::LINE_LED_PWM`. This layer is **volatile** — it resets on every power cycle,
 *    so it MUST be re-applied at every boot.
 *  - **Layer 2 — software per-channel baselines.** `Config::LINE_CALIB_LOW[B][8]` and
 *    `Config::LINE_CALIB_HIGH[B][8]` hold per-channel white/dark reference values for each
 *    board. These compensate for residual per-channel variance that the firmware's
 *    global normalization cannot see (a single channel may read slightly brighter or
 *    darker than its neighbours due to LED placement, photodiode tolerance, mounting
 *    height). They are hand-edited into `Config.h` from `calibrate()` output and survive
 *    re-flash. This layer is **non-volatile** — committed to source.
 *
 * Both layers run on every reading; they are NOT alternatives. A future maintainer who
 * deletes the `setLight()` call in `begin()` thinking the software baselines already
 * cover it will silently degrade tracking when ambient light changes. A future maintainer
 * who removes the `Config::LINE_CALIB_*` arrays thinking the firmware normalization is
 * sufficient will see channel-to-channel asymmetry in the weighted error.
 *
 * Board count and active-board routing:
 *  - `Config::LINE_BOARD_COUNT == 1` — front-only configuration. `_boards[1]` is inert,
 *    `setReverseActive()` is a no-op, all reads go to `_boards[0]`.
 *  - `Config::LINE_BOARD_COUNT == 2` — front + rear configuration. The active board
 *    auto-follows `LineFollower::setReverse(bool)`: when reverse is true the rear board
 *    drives the controller, otherwise the front board does. `LineFollower._applyEnableBuilder`
 *    propagates the flag by calling `setReverseActive(_reverse)` after every write to
 *    `_reverse`. There is no separate "set active board" API — direction is the single
 *    source of truth.
 *
 * Lifecycle (see WRO2026.ino):
 *  - `WROLineArray::begin()` once in `setup()` AFTER `MiniR4.begin()` (so `Wire1` is alive)
 *    and BEFORE `LineFollower.begin()` (which binds onto `LineArray` and assumes it is
 *    already initialised). The hardware-layer `setLight()` command runs here.
 *  - `update()` is OPTIONAL — `getError()`, `isOnline()`, and `getJunctionType()` all
 *    refresh internally on every call. `update()` exists only so callers that prefer to
 *    take a single I2C transaction per loop tick can do so explicitly, then read the
 *    cached values via the three getters.
 *  - `calibrate()` is invoked from sketch top-level only (e.g. on a `BTN_DOWN` long-press
 *    during pre-mission idle). It is sync-blocking for ~5 s (1-board config) or 10+ s
 *    (2-board config — 5 s front sweep + operator-paced reposition gated on BTN_UP +
 *    5 s rear sweep; unbounded by operator pace, capped at 5 + 60 + 5 = 70 s if the
 *    BTN_UP timeout fires) and MUST NOT be called from any module's `update()` path.
 *
 * Junction classification:
 *  - Returns one of {0=NONE, 1=LEFT, 2=RIGHT, 3=T_OR_CROSS} per the priority order
 *    documented in `getJunctionType()`. `UNKNOWN` (4) is NEVER emitted — our predicates
 *    are mutually exclusive by construction (see Decisions in the design plan).
 *  - Predicate thresholds: `Config::LINE_JUNCTION_T_MIN_ON` (T_OR_CROSS),
 *    `Config::LINE_JUNCTION_LEFT_HI` (LEFT), `Config::LINE_JUNCTION_RIGHT_LO` (RIGHT).
 *
 * @note See DECISIONS.md "8-channel custom line sensor replaces MXLineTracer" for the
 *       full design rationale, the inferred byte-packing format, and the calibration
 *       workflow modelled on `Drive.calibrateMotorBias()`.
 */

#ifndef WROLINEARRAY_H
#define WROLINEARRAY_H

#include <Arduino.h>
#include <MatrixMiniR4.h>    // Brings in `MatrixColor` transitively via MiniR4I2C.h
                             // (`#include "Sensors/MiniR4ColorSensorExt.h"`). The Arduino
                             // 1.5 library spec only adds `<lib>/src/` to the include
                             // path — not nested subdirectories like
                             // `src/Modules/Sensors/` — so a direct
                             // `#include <MiniR4ColorSensorExt.h>` is not portable across
                             // build environments. The vendor facade `<MatrixMiniR4.h>`
                             // already declares the type, which is all we need for the
                             // `MatrixColor _boards[2]` member below. (Mirrors the F9
                             // comment in `LineFollower.h`.)

#include "../../Config.h"

/**
 * @brief Singleton wrapper around 1 or 2 MXColor-protocol I2C boards exposing an
 *        8-channel reflectance line array.
 *
 * @details
 * Owns `MatrixColor _boards[2]` (front + optional rear). All three getter methods —
 * `getError()`, `isOnline()`, `getJunctionType()` — refresh internally on every call by
 * sampling the **active** board (selected by `_activeBoard`, which is flipped by
 * `setReverseActive()` in 2-board configs and ignored in 1-board configs). Channel values
 * are normalized against `Config::LINE_CALIB_LOW[B][8]` and `Config::LINE_CALIB_HIGH[B][8]`
 * before the weighted error and the on-line predicate are evaluated.
 *
 * Channel index convention: channel 0 is the LEFTMOST channel on the forward-facing chassis,
 * channel 7 is the rightmost. The same convention applies to the rear board — the rear
 * board's leftmost channel (ch0) is still on the chassis's *left* when reversing, so the
 * symmetric weights `{-3.5, -2.5, ..., +2.5, +3.5}` apply identically to both boards and the
 * controller does NOT need to negate `getError()` when the rear board is active. (See
 * `LineFollower.h` setReverse note for the kinematics derivation.)
 *
 * @note See the file-level doxygen for the **two-layer calibration contract** (firmware
 *       `setLight()` in `begin()` + software `Config::LINE_CALIB_LOW/HIGH`). Both layers
 *       are load-bearing and NOT redundant.
 * @note See the file-level doxygen for `Config::LINE_BOARD_COUNT` semantics (1=front-only,
 *       2=front+rear with active board following `LineFollower::setReverse()`).
 */
class WROLineArrayClass
{
public:
    /**
     * @brief Bring up the sensor boards and run the firmware-level hardware calibration.
     *
     * Initialises `_boards[0]` (front) and, when `Config::LINE_BOARD_COUNT == 2`,
     * `_boards[1]` (rear) on their respective I2C ports from `Config::LINE_FRONT_I2C_PORT`
     * / `Config::LINE_REAR_I2C_PORT` (both on `Wire1`). For every configured board,
     * invokes the firmware-level illumination / hardware-calibration command
     * (`MatrixColor::setLight(...)`) exactly once with brightness `Config::LINE_LED_PWM`.
     * Resets `_activeBoard = 0` (front) and clears the channel cache.
     *
     * @return `true` on success; `false` if the FRONT board's `MatrixColor::begin()`
     *         returned false (the sketch's `_setupFailed` boot health gate catches this
     *         and halts the boot with a hard failure indication). Rear-board failures
     *         (only relevant when `Config::LINE_BOARD_COUNT == 2`) are logged to Serial
     *         but do NOT affect the return value — the operator can still run a
     *         forward-only mission on the front alone (see "Error Handling" in the
     *         design plan).
     *
     * @note Must be called from `setup()` AFTER `MiniR4.begin()` (so `Wire1` is alive)
     *       and BEFORE `LineFollower.begin()` (which binds onto the `LineArray`
     *       singleton and assumes it is already initialised).
     * @note `begin()` converts the user-facing 1..4 port labels in
     *       `Config::LINE_FRONT_I2C_PORT` / `LINE_REAR_I2C_PORT` (which match the
     *       physical I2C1..I2C4 silkscreen on the MiniR4 board and meat.ino's
     *       `MiniR4.I2C{N}.MXColor` pattern) into the 0-based PCA9548A mux channel
     *       IDs that `MatrixColor::_ch` expects (I2C1 → ch 0 ... I2C4 → ch 3 —
     *       see `MatrixMiniR4.h:96-100`). `MatrixColor::i2cMUXSelect()` writes
     *       `(1 << _ch)`, so passing the raw 1..4 would select the WRONG physical
     *       port. Config keeps the 1..4 form so the operator's mental model matches
     *       the board labels; the conversion is the wrapper's responsibility.
     * @note This is **Layer 1** of the two-layer calibration contract — see file-level
     *       doxygen. The hardware calibration is volatile and resets on power cycle, so
     *       it MUST be re-applied at every boot. Do NOT remove the `setLight(...)` call
     *       thinking the software `Config::LINE_CALIB_*` arrays already cover it.
     * @note If `setLight(...)` fails for a board, the failure is logged to Serial but
     *       `begin()` still returns `true` — the on-board LED falls back to its power-on
     *       default state and readings remain usable, just less reliable until a
     *       power-cycle clears it.
     */
    bool begin();

    /**
     * @brief Cooperative-tick hook whose behaviour depends on the compile-time
     *        `Config::LINE_ARRAY_TICK_CACHE_ENABLED` toggle.
     *
     * @details
     * Dual-mode dispatch (resolved at compile time — the off-path branch is
     * dead-code-eliminated):
     *
     *  - When `Config::LINE_ARRAY_TICK_CACHE_ENABLED == true`:
     *    `update()` is the SINGLE per-tick refresh site. It samples the active
     *    board once (one 4-byte CMYK I2C transaction) and stores the per-channel
     *    values + on-line latch into the cache. `getError()`, `isOnline()`, and
     *    `getJunctionType()` then become pure cache readers — they do NOT
     *    re-issue any I2C traffic for the remainder of the tick. **`update()`
     *    MUST be called BEFORE any of those three getters in a given tick**; the
     *    sketch's `cooperativeTick()` does this automatically by placing the
     *    `LineArray.update()` call at the top of the dispatch list. Per-tick
     *    I2C cost: 1 read (~11 ms).
     *
     *  - When `Config::LINE_ARRAY_TICK_CACHE_ENABLED == false` (default — current
     *    PR #20 behaviour): `update()` is a NO-OP. Each of the three getters
     *    refreshes the cache independently on every call. Per-tick I2C cost: 3
     *    reads (~33 ms) when all three getters are touched in the same tick.
     *
     * The toggle is a `constexpr bool` so the compiler can dead-code-eliminate
     * the off-path branch entirely; flipping the toggle requires a recompile and
     * reflash. The no-op-when-disabled property means `cooperativeTick()` does
     * not need to be edited when the toggle changes — `LineArray.update()` stays
     * at the top of the dispatch list either way and harms nothing in the legacy
     * configuration.
     *
     * @note Non-blocking aside from the underlying ~1-2 ms synchronous I2C read on the
     *       active board (4 CMYK bytes via `MatrixColor::getColor(...)`) when the
     *       caching toggle is enabled. Safe to call at any cadence.
     * @note On I2C read failure (caching path only) the channel cache is left
     *       untouched (last-known-good values remain available) and `_lastOnline`
     *       is set to `false` — subsequent cached getter calls in the tick will
     *       report the lost-line state without re-issuing the failed transaction.
     */
    void update();

    /**
     * @brief Weighted-center error for the active board, mirroring
     *        `MatrixLineTracer::getError()`.
     *
     * @return Signed weighted-center error in the range `[-3.5, +3.5]`. Negative values
     *         mean the line is to the LEFT of the chassis (channels 0..3 weighted more
     *         heavily than 4..7); positive values mean the line is to the RIGHT. Returns
     *         exactly `0.0f` when no channel is on-line (so the P+D controller in
     *         `LineFollower::update()` commands zero correction and the lost-line
     *         dispatch — `Config::LOST_LINE_ACTION` — takes over).
     * @note Range is `[-3.5, +3.5]` — narrower than the vendor tracer's `[-4.5, +4.5]`
     *       because we have 8 channels rather than 10. `Config::LINE_KP` may need a
     *       one-shot retune at first hardware test; this is normal Phase 1 tuning.
     * @note When `Config::LINE_ARRAY_TICK_CACHE_ENABLED == true` this is a pure
     *       cache read — the value is whatever `update()` latched at the top of
     *       the current cooperative tick. When the toggle is `false` (legacy /
     *       default) the channel cache is refreshed inline on every call, so
     *       calling `update()` first is optional.
     * @note On I2C read failure returns `0.0f` (matches the no-line-online case so the
     *       lost-line dispatch fires identically).
     */
    float getError();

    /**
     * @brief Whether ANY channel on the active board is on-line, mirroring
     *        `MatrixLineTracer::isOnline()`.
     *
     * @return `true` if at least one channel's on-line-ness
     *         `(HIGH - value) / (HIGH - LOW)` exceeds `Config::LINE_CHANNEL_ON_RATIO`
     *         (default `0.5f`, i.e. the channel reads at least halfway from calibrated
     *         white toward calibrated dark); `false` otherwise.
     * @note When `Config::LINE_ARRAY_TICK_CACHE_ENABLED == true` this is a pure
     *       cache read — the value reflects whatever `update()` latched at the
     *       top of the current cooperative tick. When the toggle is `false`
     *       (legacy / default) the channel cache is refreshed inline on every
     *       call, so calling `update()` first is optional.
     * @note On I2C read failure returns `false` so `LineFollower::update()` enters its
     *       lost-line branch and dispatches `Config::LOST_LINE_ACTION`. No automatic
     *       fall-back to the inactive board (would deliver opposite-signed error).
     * @note Dead channels (where `Config::LINE_CALIB_LOW[B][i] >= LINE_CALIB_HIGH[B][i]`
     *       from a botched calibration paste) are treated as permanently off-line and do
     *       not contribute. This keeps a bad paste from silently fighting the controller.
     */
    bool isOnline();

    /**
     * @brief Junction classification for the active board, mirroring
     *        `MatrixLineTracer::getJunctionType()`.
     *
     * Predicates are evaluated in strict priority order so AT MOST ONE verdict can fire
     * per tick (`UNKNOWN` is structurally unreachable from this backend):
     *
     *  1. `T_OR_CROSS` (3) — if at least `Config::LINE_JUNCTION_T_MIN_ON` of 8 channels
     *     are on-line.
     *  2. `LEFT` (1) — if channels `0 .. Config::LINE_JUNCTION_LEFT_HI` are ALL on-line
     *     AND channels `LINE_JUNCTION_LEFT_HI+1 .. 7` are ALL off-line.
     *  3. `RIGHT` (2) — if channels `Config::LINE_JUNCTION_RIGHT_LO .. 7` are ALL on-line
     *     AND channels `0 .. LINE_JUNCTION_RIGHT_LO-1` are ALL off-line.
     *  4. `NONE` (0) — otherwise.
     *
     * @return One of `{0, 1, 2, 3}` mirroring the `JunctionType` enum in
     *         `Intersection.h`. `4` (UNKNOWN) is NEVER returned by this backend — the
     *         predicates above are mutually exclusive by construction.
     * @note When `Config::LINE_ARRAY_TICK_CACHE_ENABLED == true` this is a pure
     *       cache read — the value reflects whatever `update()` latched at the
     *       top of the current cooperative tick, so the sample is **provably
     *       shared** with `getError()` / `isOnline()` in the same tick. When the
     *       toggle is `false` (legacy / default) the channel cache is refreshed
     *       inline on every call; the cache is still shared across getters
     *       within a tick because each refresh writes the same `_lastChannels`
     *       buffer, but a different mid-tick sample is taken per call.
     * @note The asymmetric overlap between LEFT (`ch0..LEFT_HI`) and RIGHT
     *       (`RIGHT_LO..ch7`) predicates on channels `[RIGHT_LO..LEFT_HI]` is by
     *       design — the strict opposite-side-off requirement eliminates false fires
     *       mid-segment when a few contiguous centre channels straddle a fat line. See
     *       Decisions in the design plan.
     * @note `Intersection.update()` interprets `3` as `JunctionType::T_OR_CROSS`. The
     *       existing UNKNOWN-handling branch in `Intersection.cpp` becomes structurally
     *       unreachable with this backend but is left intact for safety.
     */
    uint8_t getJunctionType();

    /**
     * @brief Raw `getJunctionType()` return value indicating a full-bar T-junction or
     *        4-way crossing — the highest-priority junction verdict.
     *
     * Consumers should compare `LineArray.getJunctionType() == WROLineArrayClass::JUNCTION_T_OR_CROSS`
     * rather than hardcoding the literal `3`. The value maps to `JunctionType::T_OR_CROSS`
     * in `Intersection.h`; `UNKNOWN` (4) is never emitted by this backend.
     *
     * @note The full return-value contract is `{0=NONE, 1=LEFT, 2=RIGHT, 3=T_OR_CROSS}` —
     *       see `getJunctionType()` for the complete priority-ordered predicate documentation.
     */
    static constexpr uint8_t JUNCTION_T_OR_CROSS = 3;

    /**
     * @brief Flip the active board to follow the chassis's reverse-direction state.
     *
     * @param reverse `true` to select the REAR board (`_boards[1]`); `false` to select
     *                the FRONT board (`_boards[0]`).
     *
     * @note In 2-board configurations (`Config::LINE_BOARD_COUNT == 2`) this swaps which
     *       physical board `getError()` / `isOnline()` / `getJunctionType()` sample from.
     *       In 1-board configurations this is a NO-OP — the front board is always active.
     * @note `LineFollower::_applyEnableBuilder` wires this method automatically, calling
     *       `setReverseActive(_reverse)` after every write to its internal `_reverse`
     *       flag (both the fluent `setEnabled(...).setReverse(...)` path and the direct
     *       non-fluent `LineFollower::setReverse(bool)` setter). Direct callers usually
     *       do NOT need to invoke this method — modify `LineFollower::setReverse(bool)`
     *       instead so the controller and the sensor stay in sync.
     */
    void setReverseActive(bool reverse);

    /**
     * @brief Read the currently selected active-board direction.
     *
     * @return `true` if the rear board is active; `false` if the front board is active.
     * @note Always returns `false` in 1-board configurations regardless of any prior
     *       `setReverseActive(true)` call (the setter is a no-op there).
     */
    bool getReverseActive() const;

    /**
     * @brief Read the raw 8 per-channel reflectance values from the currently active
     *        board.
     *
     * @param out  Output buffer of exactly 8 bytes. Channel index 0 is the leftmost
     *             channel; channel index 7 is the rightmost. Values are in the packed
     *             range `0..9` (higher = brighter/white, lower = darker; the followed
     *             line is DARK, so a LOW value means over the line), unmodified by
     *             any software calibration.
     * @return `true` if the I2C transaction succeeded and `out` is populated; `false`
     *         on transaction failure (`out` is left untouched — caller must zero
     *         beforehand if a defined value is required).
     *
     * @note Reserved for diagnostics and bring-up sketches (TC9 channel-position
     *       validation in particular). Production code paths should use the higher-level
     *       `getError()` / `isOnline()` / `getJunctionType()` getters.
     * @note In 1-board configurations the active board is always the front board.
     */
    bool readActiveBoard(uint8_t out[8]);

    /**
     * @brief Read the raw 8 per-channel reflectance values from a SPECIFIC board, by index.
     *
     * @param boardIdx  `0` for the front board, `1` for the rear board.
     * @param out       Output buffer of exactly 8 bytes (same convention as
     *                  `readActiveBoard()`).
     * @return `true` if the I2C transaction succeeded; `false` on transaction failure
     *         OR when `boardIdx == 1` in a 1-board configuration
     *         (`Config::LINE_BOARD_COUNT == 1`).
     *
     * @note Reserved for diagnostics — lets a bring-up sketch sample both boards
     *       independently without touching the `_activeBoard` selection. The `calibrate()`
     *       routine uses this internally to print per-board calibration blocks.
     */
    bool readBoard(uint8_t boardIdx, uint8_t out[8]);

    /**
     * @brief Software per-channel calibration routine — captures min/max over a per-board
     *        5 s sweep (sequential for 2-board configs) and prints a Config-paste-ready
     *        block to Serial.
     *
     * Caller-managed workflow (mirrors `Drive::calibrateMotorBias()`):
     *  1. Operator holds `BTN_DOWN` for >=2 s during the pre-mission idle screen.
     *  2. The sketch's idle handler calls `LineArray.calibrate()` — control transfers
     *     here.
     *  3. Operator manually sweeps the FRONT sensor bar over the test surface for the
     *     full 5 s of Phase 1, alternating between bare white substrate (captures the
     *     per-channel LOW baseline) and directly over a black line (captures the
     *     per-channel HIGH baseline). For 2-board configurations the routine then
     *     beeps twice and waits indefinitely for the operator to press BTN_UP
     *     (60 s timeout) while they reposition; the operator then sweeps the REAR
     *     bar for the 5 s of Phase 2. See the two-phase sweep `@note` below for
     *     the rationale.
     *  4. On return, the method has printed two `constexpr uint8_t LINE_CALIB_LOW[...] = { ... };`
     *     and `constexpr uint8_t LINE_CALIB_HIGH[...] = { ... };` blocks to Serial at
     *     115200 baud, in C++ paste-ready format.
     *  5. Operator copies the printed blocks into `Config.h` by hand, re-flashes, and the
     *     new baselines take effect.
     *
     * No EEPROM, no auto-baseline on boot — deterministic and reviewable from source.
     *
     * @warning Sync-blocking for ~5 s (1-board config) or 10+ s (2-board config — 5 s
     *          front sweep + operator-paced reposition gated on BTN_UP + 5 s rear sweep;
     *          unbounded by operator pace, capped at 5 + 60 + 5 = 70 s if the BTN_UP
     *          timeout fires and Phase 2 is skipped). MUST be called from sketch
     *          top-level only (e.g. an idle-screen handler before the mission starts).
     *          Calling from any module's `update()` path would freeze the cooperative
     *          loop and break `LineFollower`, `Intersection`, and the `Drive` queue
     *          dispatcher.
     *
     * @note This is **Layer 2** of the two-layer calibration contract — see file-level
     *       doxygen. The software baselines printed here compensate for residual
     *       per-channel variance that the firmware-level `setLight(...)` normalization
     *       (Layer 1, invoked in `begin()`) cannot see. Do NOT remove this method
     *       thinking the firmware calibration already covers it.
     * @note On a 2-board configuration this captures both boards; on a 1-board
     *       configuration only the front board's block is printed.
     * @note Two-phase sweep workflow (for `LINE_BOARD_COUNT == 2`). Front and rear bars
     *       cannot be physically swept simultaneously (they sit at opposite ends of the
     *       chassis), so this routine sweeps them SEQUENTIALLY inside a single operator
     *       trigger:
     *        1. Phase 1: 5 s sweep of the FRONT bar only. Operator presents the front
     *           bar to the white + line reference.
     *        2. Audible cue (two 1.2 kHz beeps), then WAIT for operator to press
     *           BTN_UP confirming the REAR bar is positioned. 60 s timeout: if
     *           BTN_UP is not pressed in time, Phase 2 is skipped entirely and the
     *           REAR row falls back to safe defaults (LOW=0, HIGH=9) via the
     *           existing Layer-2 safety net.
     *        3. Phase 2: 5 s sweep of the REAR bar only. Operator presents the rear bar.
     *       `LINE_BOARD_COUNT == 1` configs skip Phase 2 entirely. Total wall-clock
     *       time: ~5 s (1-board) or ~5 s + operator-paced reposition + 5 s = 10+ s
     *       (2-board — unbounded by operator pace, capped at 5 + 60 + 5 = 70 s if the
     *       timeout fires). The per-phase sampling loop lives in the private helper
     *       `_sweepOneBoardForCalibration()`.
     * @note **Safety net for unresponsive / flaky boards (PR #20 fix #7):**
     *       (1) A pre-flight connectivity check runs ONE read per configured board
     *       before the 5 s sweep — if any configured board does not respond, the
     *       whole sweep is aborted with a message naming the failing port (no values
     *       printed, so the operator cannot accidentally paste a half-calibrated
     *       block). (2) During the sweep, per-board valid-sample counters track
     *       intermittent failures; any configured board that produced zero valid
     *       samples gets a per-board warning AND has its row replaced with safe
     *       defaults (LOW=0, HIGH=9 — matches the Config.h pre-calibration baseline,
     *       so the printed block always parses as valid C++ and the next-boot dead-
     *       channel warning in `begin()` does NOT fire). The operator will still see
     *       lost-line behaviour on the affected board and know calibration needs
     *       another run. The boot-time dead-channel warning, the CMYK byte > 99
     *       guard in `_readBoardInternal`, and the 1..4 → 0..3 port conversion in
     *       `begin()` all compose cleanly with this safety net.
     */
    void calibrate();

private:
    MatrixColor _boards[2];                  ///< Front, rear. `_boards[1]` is inert when
                                             ///< `Config::LINE_BOARD_COUNT == 1`.
    uint8_t     _activeBoard      = 0;       ///< `0` = front, `1` = rear. Flipped by
                                             ///< `setReverseActive()` in 2-board configs.
    uint8_t     _lastChannels[8]  = {0};     ///< Cached per-channel readings from the
                                             ///< most recent active-board sample. Shared
                                             ///< by all three getters so a single tick
                                             ///< issues exactly one I2C transaction.
    bool        _lastOnline       = false;   ///< Cached on-line state for the most recent
                                             ///< sample (cleared to false on I2C failure).

    /**
     * @brief Refresh `_lastChannels` and `_lastOnline` from the active board.
     *
     * Reads the 4 CMYK bytes via `_readBoardInternal`; on I²C failure sets
     * `_lastOnline = false` and leaves `_lastChannels` untouched.
     *
     * Called by:
     *  - `update()` (when `Config::LINE_ARRAY_TICK_CACHE_ENABLED == true`)
     *  - The three public getters (when the toggle is `false` — legacy path)
     */
    void _refreshCache();

    /**
     * @brief Read one board's 4 CMYK bytes via `MatrixColor::getColor(...)` and unpack
     *        them into 8 per-channel values.
     *
     * Each CMYK byte packs two channel values as decimal digits per the table below
     * (`channel_a = byte / 10`, `channel_b = byte % 10`):
     *
     *  | CMYK byte | tens digit (channel) | ones digit (channel) |
     *  |-----------|----------------------|----------------------|
     *  | C         | 0 (leftmost)         | 1                    |
     *  | M         | 2                    | 3                    |
     *  | Y         | 4                    | 5                    |
     *  | K         | 6                    | 7 (rightmost)        |
     *
     * @param boardIdx  `0` for front, `1` for rear.
     * @param out       Output buffer of exactly 8 bytes, channels indexed left-to-right.
     * @return `false` on either of the two failure modes enforced INSIDE this method:
     *         (a) `boardIdx >= 2` (hard physical-board-count bound), OR
     *         (b) any of the 4 CMYK bytes reads as `> 99` (invalid packed-decimal-digit
     *         form, surfaces I²C transport corruption — `MatrixColor::getColor()` returns
     *         raw `Wire.read()` bytes with no transport-status check, so a bus failure /
     *         dropout can surface as `0xFF` which unpacks to nonsense out-of-range values).
     *         Callers that accept `boardIdx` from external input should additionally enforce
     *         `boardIdx < Config::LINE_BOARD_COUNT` before calling here — see the public
     *         `readBoard()` method, which performs that check.
     *         On ALL failure paths `out[]` is NOT written, so the caller's cache is
     *         preserved as last-known-good. `true` if the I2C transaction succeeded AND
     *         all 4 bytes fell in the valid `[0..99]` range.
     *
     * @note **INFERRED CONTRACT — must be validated on hardware before merge.** The
     *       CMYK-byte → channel-index mapping above is inferred from meat.ino's
     *       `light_8()` (lines 670-684) — the MXColor firmware documentation does not
     *       state the packing order. TC9 of the test plan validates the mapping by
     *       physically covering each channel index in turn and confirming the expected
     *       slot in `_lastChannels[]` drops. If TC9 fails, the fix is local to this
     *       method's unpacking table — no caller-side changes are required.
     */
    bool _readBoardInternal(uint8_t boardIdx, uint8_t out[8]);

    /**
     * @brief Normalise a raw channel reading into a [0,1] on-line-ness score.
     *
     * The followed line is DARK on a light field, so on-line-ness grows as the reading
     * falls from the calibrated white endpoint (`high`, per-channel max) toward the dark
     * endpoint (`low`, per-channel min): returns `(high - raw) / (high - low)`, 1.0 at
     * `low`, 0.0 at `high`, clamped to `[0,1]`. Callers MUST pre-reject dead channels
     * (`low >= high`). Shared by `_computeError`, `_isOnlineFromCache`, and
     * `_classifyJunction` so the three cannot drift out of polarity sync.
     *
     * @param raw  Raw packed channel value (`0..9`).
     * @param low  Calibrated DARK / on-line endpoint (`Config::LINE_CALIB_LOW`).
     * @param high Calibrated WHITE / off-line endpoint (`Config::LINE_CALIB_HIGH`).
     * @return On-line-ness in `[0,1]`; higher = darker = more on the line.
     */
    static float _onnessNorm(uint8_t raw, uint8_t low, uint8_t high);

    /**
     * @brief Compute the weighted-center error for an 8-channel sample.
     *
     * Each channel `i` contributes weight `_sensorWeights[i] = {-3.5, -2.5, -1.5, -0.5,
     * +0.5, +1.5, +2.5, +3.5}` scaled by its normalized on-line-ness
     * `(HIGH[B][i] - channels[i]) / (HIGH[B][i] - LOW[B][i])` (1.0 = over the dark line,
     * 0.0 = over white). Dead channels (where `LOW >= HIGH`) contribute zero.
     *
     * @param channels  Raw per-channel reading buffer (8 bytes, `0..9`).
     * @return Signed weighted-center error in `[-3.5, +3.5]`, or `0.0f` when the
     *         normalized total is zero.
     */
    float _computeError(const uint8_t channels[8]) const;

    /**
     * @brief Test whether ANY channel in the given sample is on-line.
     *
     * @param channels  Raw per-channel reading buffer (8 bytes, `0..9`).
     * @return `true` if at least one channel's normalized reading exceeds
     *         `Config::LINE_CHANNEL_ON_RATIO`. Dead channels are skipped.
     */
    bool _isOnlineFromCache(const uint8_t channels[8]) const;

    /**
     * @brief Run the priority-ordered junction predicates on the given sample.
     *
     * Implements the priority order documented on `getJunctionType()`: T_OR_CROSS → LEFT
     * → RIGHT → NONE. Returns the raw `uint8_t` value (`0..3`) that `getJunctionType()`
     * surfaces to callers.
     *
     * @param channels  Raw per-channel reading buffer (8 bytes, `0..9`).
     * @return `0` = NONE, `1` = LEFT, `2` = RIGHT, `3` = T_OR_CROSS. Never returns `4`.
     */
    uint8_t _classifyJunction(const uint8_t channels[8]) const;

    /**
     * @brief Run one ~5 s sampling phase for a single board.
     *
     * Used by `calibrate()` to sequence boards one at a time (front and rear cannot
     * be physically swept simultaneously since they sit at opposite ends of the
     * chassis). Reads only the specified board every 20 ms for 5 s, updating its
     * `mnRow` / `mxRow` min/max accumulators and bumping `validCount` on each
     * successful `_readBoardInternal` call.
     *
     * @param boardIdx Which `_boards[]` slot to sample (must be `< LINE_BOARD_COUNT`).
     * @param mnRow     Per-channel minimum accumulator for this board (size 8).
     *                  Caller seeds with 9 (the highest valid value); this helper
     *                  only updates if a sampled channel is lower.
     * @param mxRow     Per-channel maximum accumulator for this board (size 8).
     *                  Caller seeds with 0; only updated upward.
     * @param validCount Output reference. Incremented once per successful sample.
     *                  Caller should pre-zero.
     */
    void _sweepOneBoardForCalibration(uint8_t   boardIdx,
                                      uint8_t   mnRow[8],
                                      uint8_t   mxRow[8],
                                      uint16_t& validCount);
};

/**
 * @brief Single global instance — defined in WROLineArray.cpp.
 */
extern WROLineArrayClass LineArray;

#endif    // WROLINEARRAY_H
