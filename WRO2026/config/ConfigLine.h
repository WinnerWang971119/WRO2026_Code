#ifndef CONFIG_LINE_H
#define CONFIG_LINE_H

#include <stdint.h>

#include "ConfigGlobal.h"
#include "ConfigDrive.h"

/**
 * @brief Line-following + line-sensor configuration — reopens `namespace Config` per ADR-031.
 *
 * Holds the line-follower outer-loop PID gains, the cruise / correction limits, the
 * 8-channel WROLineArray sensor table (board count, I²C ports, two-layer calibration,
 * junction predicates), the lost-line recovery behaviour, and the Intersection debounce.
 *
 * Depends on `ConfigDrive.h` for `SPEED_NORMAL` (`CRUISE_SPEED` forwards it) and on
 * `ConfigGlobal.h` for the PWM bounds referenced by the CRAWL_PWM assert (which lives in
 * `ConfigChecks.h`, included last).
 */
namespace Config
{
    /** @brief Line-follower outer-loop Kp (FRONT board / forward follow). @note TODO: tune on real strip. */
    constexpr float LINE_KP = 6.0f;
    /** @brief Line-follower outer-loop Ki. @note Unused by the P+D law (kept for future I-term). */
    constexpr float LINE_KI = 0.0f;
    /** @brief Line-follower outer-loop Kd (FRONT board / forward follow). @note TODO: tune on real strip. */
    constexpr float LINE_KD = 2.6f;

    // ---------------------------------------------------------------------------------------
    // Rear-board (reverse follow) gains — ADR-070.
    //
    // The front line sensor sits `LINE_SENSOR_AXLE_OFFSET_CM` (12.02 cm) ahead of the axle;
    // the rear sits `LINE_SENSOR_AXLE_OFFSET_REAR_CM` (7.68 cm). That distance is the
    // controller's lookahead, so the SAME Kp/Kd steer differently on each board: the shorter
    // rear lookahead goes unstable (over-corrects, swings off the line) at gains the longer
    // front lookahead rides smoothly. `LineFollower` therefore selects these gains whenever the
    // rear board leads (reverse follow, `_reverse == true`); the FRONT gains above are used for
    // forward follow. Tune them independently with the `inoTest/line_pid_tune` ping-pong tuner
    // (its reverse runs optimise THIS set).
    //
    // Default to the front gains so behaviour is unchanged until the rear set is tuned — replace
    // with the tuner's printed REAR values once measured.
    // -------------------------------------------6-------------------------------------------

    /** @brief Line-follower outer-loop Kp (REAR board / reverse follow). @note TODO: tune via ping-pong tuner. */
    constexpr float LINE_KP_REAR = 6.8f;
    /** @brief Line-follower outer-loop Kd (REAR board / reverse follow). @note TODO: tune via ping-pong tuner. */
    constexpr float LINE_KD_REAR = 12.5f;

    // ---------------------------------------------------------------------------------------
    // Cruise / line following
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Forward cruise power for line following, in the [-100, 100] DriveDC range.
     *
     * Kept as a named constant for back-compat with code that still references it directly;
     * new callers should prefer `LineFollower.setPreset(SpeedPreset::NORMAL)` which resolves
     * to `SPEED_NORMAL` (currently equal to this value).
     */
    constexpr int16_t CRUISE_SPEED = SPEED_NORMAL;

    /** @brief Maximum left/right correction added to / subtracted from cruise speed. */
    constexpr int16_t LINE_MAX_CORR = 20;

    /**
     * @brief Signed lateral-offset setpoint bias for LINE follow, in sensor-centroid index units.
     *
     * Shifts the P-term setpoint so the line settles off-center under the array, riding the
     * chassis LEFT (+) or RIGHT (-) of the line. Mapped from the per-edge LineFollowMode by
     * PathTracker via WROMath::lineFollowModeToSetpoint. NOT centimeters — error units don't map
     * to a fixed distance without sensor pitch/height/line-width; measured on the mat.
     * @note TODO: bench-tune. LEFT = +, RIGHT = -.
     */
    constexpr float LINE_FOLLOW_OFFSET_ERROR = 1.5f;

    // ---------------------------------------------------------------------------------------
    // 8-channel custom line sensor (WROLineArray)
    //
    // WROLineArray drives 1-2 MXColor-protocol I²C boards. Each board packs 8 reflectance
    // channels into 4 CMYK bytes, with tens digit = channel N and ones digit = channel N+1
    // (0..9 packed-byte range per channel after firmware-level scaling).
    //
    // Two-layer calibration:
    //   Layer 1 — firmware-level hardware calibration: WROLineArray::begin() issues the
    //             firmware setLight() command and calls the board's calibration routine.
    //             This layer is VOLATILE — it resets on every power cycle and must be
    //             re-run on every boot via begin().
    //   Layer 2 — software per-channel baselines: LINE_CALIB_LOW / LINE_CALIB_HIGH below
    //             are hand-edited into this file after running WROLineArray::calibrate()
    //             and pasting the Serial output (same workflow as Drive.calibrateMotorBias()
    //             → copy scalars → paste into Config.h → reflash). This layer SURVIVES
    //             re-flash and power cycles because it lives in firmware. Both layers are
    //             required; they are NOT redundant — Layer 1 brings the hardware to a
    //             consistent operating point, Layer 2 normalises inter-channel spread.
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Number of MXColor-protocol boards installed.
     *
     * 1 = front board only; 2 = front + rear boards present (the bidirectional
     * line-following configuration introduced by ADR-024 — required for the
     * auto-direction step in `PathTracker::_resolveDirection`).
     * When set to 1, `LINE_REAR_I2C_PORT` is ignored at compile time.
     *
     * @note **Compile-time override:** guarded by `#ifndef LINE_BOARD_COUNT_DEFAULT`
     *       so a host test can pin it target-wide via `-DLINE_BOARD_COUNT_DEFAULT=1`
     *       without editing this file — same idiom as `LINE_JUNCTION_T_MIN_ON_DEFAULT`
     *       below (issue #54, T10: a 1-board real-FSM host target). Range-guarded to
     *       `{1, 2}` in `ConfigChecks.h`.
     */
#ifndef LINE_BOARD_COUNT_DEFAULT
#define LINE_BOARD_COUNT_DEFAULT 2
#endif
    constexpr uint8_t LINE_BOARD_COUNT = LINE_BOARD_COUNT_DEFAULT;

    /**
     * @brief I²C port index (1..4) on Wire1 carrying the FRONT line-sensor board.
     *
     * Maps to the hardware I2C1..I2C4 external ports on the MATRIX Mini R4 (matches
     * the physical silkscreen labels AND meat.ino's `MiniR4.I2C{N}.MXColor` pattern).
     * Port 0 is the on-board `Wire` — never use 0 for an external sensor board.
     *
     * @note Valid range is `[1..4]`. The 1..4 → 0..3 conversion to the underlying
     *       PCA9548A mux channel ID (`MatrixColor::_ch`, which `i2cMUXSelect()` shifts
     *       as `(1 << _ch)`) happens inside `WROLineArrayClass::begin()` — do NOT
     *       pre-convert here. Vendor source cross-reference:
     *       `MatrixMiniR4.h:96-100` (`I2C1` → mux ch 0 ... `I2C4` → ch 3).
     */
    constexpr uint8_t LINE_FRONT_I2C_PORT = 1;

    /**
     * @brief I²C port index (1..4) on Wire1 carrying the REAR line-sensor board.
     *
     * Maps to the hardware I2C1..I2C4 external ports on the MATRIX Mini R4 (matches
     * the physical silkscreen labels AND meat.ino's `MiniR4.I2C{N}.MXColor` pattern).
     * Ignored when `LINE_BOARD_COUNT == 1`. Must be distinct from `LINE_FRONT_I2C_PORT`.
     *
     * @note Valid range is `[1..4]`. The 1..4 → 0..3 conversion to the underlying
     *       PCA9548A mux channel ID (`MatrixColor::_ch`, which `i2cMUXSelect()` shifts
     *       as `(1 << _ch)`) happens inside `WROLineArrayClass::begin()` — do NOT
     *       pre-convert here. Vendor source cross-reference:
     *       `MatrixMiniR4.h:96-100` (`I2C1` → mux ch 0 ... `I2C4` → ch 3).
     */
    constexpr uint8_t LINE_REAR_I2C_PORT = 4;

    /**
     * @brief Per-board, per-channel calibrated DARK (on-line) reflectance value — the
     *        per-channel MINIMUM seen during the sweep (a channel reads its darkest when
     *        it is over the black line).
     *
     * Dimension: [LINE_BOARD_COUNT][8]. Channel values are in the 0..9 packed-byte
     * range produced by the MXColor firmware after Layer-1 hardware calibration. The
     * followed line is DARK on a light field, so LOW is the low-reflectance endpoint:
     * on-line-ness is `(HIGH - raw) / (HIGH - LOW)`, which is 1.0 at LOW and 0.0 at HIGH.
     *
     * Workflow: run WROLineArray::calibrate(), sweep each board over white AND the line
     * during the timed window, then paste the whole printed block (both arrays) here and
     * reflash. Values survive power cycles because they live in firmware (Layer-2
     * calibration). The default of 0 places the dark endpoint at full dark; combined with
     * the HIGH default of 9, an uncalibrated channel only reads on-line below mid
     * brightness, so a bright surface fails safe to off-line.
     */
    constexpr uint8_t LINE_CALIB_LOW[2][8] = {
        {0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0},
    };

    /**
     * @brief Per-board, per-channel calibrated WHITE (off-line) reflectance value — the
     *        per-channel MAXIMUM seen during the sweep (a channel reads its brightest
     *        over the white field, off the line).
     *
     * Dimension: [LINE_BOARD_COUNT][8]. Channel values are in the 0..9 packed-byte
     * range produced by the MXColor firmware after Layer-1 hardware calibration. HIGH is
     * the bright endpoint of the on-line-ness map `(HIGH - raw) / (HIGH - LOW)`.
     *
     * Workflow: same sweep as LINE_CALIB_LOW — paste the whole printed block. Defaults of
     * 9 (max brightness) keep every channel off-line until a distinctly dark reading
     * appears, and guarantee HIGH > LOW so no channel is flagged dead before calibration.
     */
    constexpr uint8_t LINE_CALIB_HIGH[2][8] = {
        {9, 9, 9, 9, 9, 9, 9, 9},
        {9, 9, 9, 9, 9, 9, 9, 9},
    };

    /**
     * @brief Channel ON-LINE threshold expressed as a fraction of (HIGH - LOW).
     *
     * A channel is declared "on-line" when its normalised reading ≥ this ratio.
     * 0.5f = midpoint between the calibrated white and dark values. Increase toward
     * 1.0f to require a stronger dark reading (reduces false positives on smudged
     * surfaces); decrease toward 0.0f to trigger earlier (increases sensitivity).
     *
     * Applied per-channel: onLine = (HIGH[b][ch] - raw) / (HIGH[b][ch] - LOW[b][ch])
     *                               >= LINE_CHANNEL_ON_RATIO.
     */
    constexpr float LINE_CHANNEL_ON_RATIO = 0.6f;

    /**
     * @brief Minimum number of channels simultaneously ON-LINE to declare a
     *        T_OR_CROSS junction (highest-priority junction class, checked first).
     *
     * At 7 of 8 channels lit, the robot is clearly centred on a wide cross or T
     * marker. Lower values risk false triggers on debris or sensor variation;
     * higher values may miss junctions under partial occlusion.
     *
     * @note **Compile-time override:** guarded by `#ifndef LINE_JUNCTION_T_MIN_ON_DEFAULT`
     *       so a host test can pin it target-wide via `-DLINE_JUNCTION_T_MIN_ON_DEFAULT=...`
     *       without editing this file — same idiom as `HOP_SETTLE_MS_DEFAULT`
     *       (`ConfigNav.h`). Range-guarded in `ConfigChecks.h` (issue #152).
     * @note TODO: tune on real mat after per-channel calibration is complete.
     */
#ifndef LINE_JUNCTION_T_MIN_ON_DEFAULT
#define LINE_JUNCTION_T_MIN_ON_DEFAULT 7
#endif
    constexpr uint8_t LINE_JUNCTION_T_MIN_ON = LINE_JUNCTION_T_MIN_ON_DEFAULT;

    /**
     * @brief Upper inclusive channel index for the LEFT junction predicate.
     *
     * LEFT junction fires when all channels ch[0..LINE_JUNCTION_LEFT_HI] are ON-LINE.
     * The opposite (right-side) channels are DON'T-CARE — a one-sided branch is detected by
     * its own edge staying lit through the crossing, not by the far side going dark (the old
     * opposite-side-off rule flickered to NONE mid-junction at speed). ch0 (the LEFTMOST
     * channel) being in the required set is what keeps a centred straight line from firing
     * it. Default 3 = the left half of the 8-channel bar (ch0..3), adjacent to RIGHT_LO=4.
     *
     * @note **Compile-time override:** guarded by `#ifndef LINE_JUNCTION_LEFT_HI_DEFAULT`
     *       so a host test can pin it target-wide via `-DLINE_JUNCTION_LEFT_HI_DEFAULT=...`
     *       without editing this file — same idiom as `HOP_SETTLE_MS_DEFAULT`
     *       (`ConfigNav.h`). Range-guarded in `ConfigChecks.h` (issue #152).
     * @note TODO: tune on real mat.
     */
#ifndef LINE_JUNCTION_LEFT_HI_DEFAULT
#define LINE_JUNCTION_LEFT_HI_DEFAULT 4
#endif
    constexpr uint8_t LINE_JUNCTION_LEFT_HI = LINE_JUNCTION_LEFT_HI_DEFAULT;

    /**
     * @brief Lower inclusive channel index for the RIGHT junction predicate.
     *
     * RIGHT junction fires when all channels ch[LINE_JUNCTION_RIGHT_LO..7] are ON-LINE.
     * The opposite (left-side) channels are DON'T-CARE — mirror of LEFT_HI above. ch7 (the
     * RIGHTMOST channel) being in the required set is what keeps a centred straight line from
     * firing it. Default 4 = the right half of the 8-channel bar (ch4..7), adjacent to
     * LEFT_HI=3. Keep RIGHT_LO <= LEFT_HI + 1 (no gap) so a fully lit bar always resolves to
     * T_OR_CROSS before it could satisfy both LEFT and RIGHT.
     *
     * @note **Compile-time override:** guarded by `#ifndef LINE_JUNCTION_RIGHT_LO_DEFAULT`
     *       so a host test can pin it target-wide via `-DLINE_JUNCTION_RIGHT_LO_DEFAULT=...`
     *       without editing this file — same idiom as `HOP_SETTLE_MS_DEFAULT`
     *       (`ConfigNav.h`). Range-guarded in `ConfigChecks.h` (issue #152).
     * @note TODO: tune on real mat.
     */
#ifndef LINE_JUNCTION_RIGHT_LO_DEFAULT
#define LINE_JUNCTION_RIGHT_LO_DEFAULT 3
#endif
    constexpr uint8_t LINE_JUNCTION_RIGHT_LO = LINE_JUNCTION_RIGHT_LO_DEFAULT;

    /**
     * @brief On-board LED brightness level applied by WROLineArray::begin() via the
     *        firmware-level setLight() command (0..255).
     *
     * Higher values improve contrast on pale surfaces; too high can wash out the dark
     * line or saturate the detector. 200 is a reasonable starting point for indoor
     * fluorescent lighting on the WRO competition mat.
     *
     * @note This is a Layer-1 (volatile) setting — it is re-applied on every begin()
     *       call and does not persist across power cycles on its own.
     */
    constexpr uint8_t LINE_LED_PWM = 200;

    /**
     * @brief Toggle tick-scoped caching in WROLineArrayClass.
     *
     * When `false` (legacy / pre-cache behaviour): each call to
     * `getError()`, `isOnline()`, and `getJunctionType()` independently issues
     * a full 4-byte CMYK I²C transaction. `WROLineArrayClass::update()` is a
     * no-op. Total per cooperative tick: 3 sensor reads (~33 ms at vendor I²C
     * timing), giving ~28-30 Hz loop rate.
     *
     * When `true` (default): `WROLineArrayClass::update()` issues the single
     * sensor read for the tick (must be called at the top of
     * `cooperativeTick()` for this to work). The three getters become pure
     * cache readers. Total per tick: 1 sensor read (~11 ms), giving ~70-80 Hz
     * loop rate.
     *
     * Default is `true`: hardware bring-up confirmed the cached path behaves
     * identically (no controller wobble from one-tick-stale data on a
     * chassis travelling at typical cruise). Flip to `false` to fall back to
     * the legacy uncached per-getter reads if a regression is suspected.
     *
     * See PR #20 review thread + DECISIONS.md if added later.
     */
    constexpr bool LINE_ARRAY_TICK_CACHE_ENABLED = true;

    /**
     * @brief Toggle the line-sensor consumer gate inside `cooperativeTick()`.
     *
     * When `true` (default): `cooperativeTick()` skips `LineArray.update()`,
     * `LineFollower.update()`, and `Intersection.update()` on ticks where
     * nothing needs a fresh sample — i.e. no line-follow is enabled and no
     * `stopAtLine` seek is in flight — reclaiming the slow Wire1 I²C read.
     * `stopAtLine` and junction detection keep the gate open, so those paths
     * still see a fresh sample every tick.
     *
     * When `false`: the gate predicate constant-folds to `true` and the
     * three updates run unconditionally, matching pre-gate behavior.
     *
     * This is a compile-time `constexpr`, so the off-branch is dead-code
     * eliminated by the compiler, mirroring `LINE_ARRAY_TICK_CACHE_ENABLED`
     * above. Flipping this to `false` is the one-line rollback.
     *
     * See issue #164.
     */
    constexpr bool LINE_ARRAY_TICK_GATE_ENABLED = true;

    // ---------------------------------------------------------------------------------------
    // Lost-line behavior
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Action taken by the LineFollower when the strip is lost beneath every channel.
     *
     * - `STOP`  — actively brake to zero and surface the lost-line event.
     * - `COAST` — release power and let the chassis roll to a stop.
     * - `CRAWL` — keep cruising at a reduced speed in the last-known direction to try to reacquire.
     */
    enum class LostLineAction : uint8_t
    {
        STOP,
        COAST,
        CRAWL
    };

    /** @brief Default lost-line action used by the LineFollower. */
    constexpr LostLineAction LOST_LINE_ACTION = LostLineAction::STOP;

    /**
     * @brief Deliberate lost-line creep PWM used by the CRAWL recovery branch, in [-100, 100].
     *
     * Intentionally decoupled from the active SpeedPreset so that changing cruise speed never
     * silently alters the CRAWL behavior. At 60 the creep sits just below the NORMAL cruise of 70
     * and comfortably above the `PWM_MIN_MAGNITUDE = 45` deadband floor — a deliberately chosen
     * absolute PWM, not a fraction of cruise. Deriving it as e.g. `SPEED_NORMAL / 2` at compile
     * time would re-introduce exactly the coupling this constant exists to avoid, and could land
     * below `PWM_MIN_MAGNITUDE` for a slower active motor profile — silently snapped UP to the
     * floor by `deadbandSnap`. Stating the creep PWM explicitly sidesteps both problems.
     *
     * @note Current values (`ConfigDrive.h` / `ConfigGlobal.h`): SPEED_NORMAL = 70,
     *       SPEED_SLOW = 60, PWM_MIN_MAGNITUDE = 45 — 60 sits just below NORMAL cruise and
     *       comfortably above the floor for both presets today. Re-derive this note if any of
     *       those three constants are retuned.
     * @note Must satisfy PWM_MIN_MAGNITUDE ≤ CRAWL_PWM ≤ PWM_MAX so the value is never
     *       silently floored or capped by deadbandSnap. The static_assert below enforces this.
     */
    constexpr int16_t CRAWL_PWM = 46;

    /**
     * @brief Hard ceiling for the blocking `LineFollower.waitUntilDistanceReached()` helper.
     *
     * The new sync helper is intentionally a thin orchestration wrapper around repeated
     * `LineFollower.update()` ticks. Without a deadline, a bad sensor read, wrong target, or
     * a caller accidentally handing motor ownership back to a bounded `Drive` move would hang
     * the sketch forever. 10 s matches `DRIVE_DRAIN_TIMEOUT_MS_PER_CMD`, which is already the
     * project-wide "something is stuck" ceiling for a single bounded chassis segment. On
     * expiry the helper disables the follower with active brake and returns `false`.
     */
    constexpr uint32_t LINE_FOLLOW_WAIT_TIMEOUT_MS = 10000;

    // =============================================================================
    // Intersection debounce (code review F7).
    //
    // The MX line tracer can flicker between NONE, UNKNOWN, and a valid junction
    // type at the leading/trailing edges of a junction marker, under lighting
    // changes, or during fast motion. Without debounce a single physical
    // intersection can fire multiple software edges. With these knobs we require
    // a stable streak of valid samples to ARM and a stable streak of NONE samples
    // to RELEASE — a single noisy sample no longer crosses the threshold.
    //
    // At ~50-100 Hz cooperative tick, 2 ticks ≈ 20-40 ms ≈ ~0.5-1 cm at typical
    // line-follow speeds (15-30 cm/s). Junction markers are ~5-10 cm wide so 2
    // ticks is well within the dwell time but rejects single-sample flicker.
    // =============================================================================

    /**
     * @brief Consecutive valid-junction samples required to arm a rising edge.
     */
    constexpr uint8_t INTERSECTION_DEBOUNCE_TICKS = 1;

    /**
     * @brief Consecutive NONE samples required to release before re-arming.
     */
    constexpr uint8_t INTERSECTION_RELEASE_TICKS = 2;
}    // namespace Config

#endif    // CONFIG_LINE_H
