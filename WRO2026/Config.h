#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Single source of truth for every tunable constant; modules MUST source values from
 *        here, never hardcode.
 *
 * Every downstream subsystem (Drive, Manipulator, LineFollower, Intersection, EventTrace, etc.)
 * pulls its tuning, hardware-mapping and behavioural constants from this namespace. Changing a
 * value here is intended to be the only edit required to retune the robot — never duplicate any
 * of these constants in a `.cpp`/`.ino` or as a function-local literal.
 *
 * @note PID gains and motor specs target TT-motors (the chassis we are building). Values are
 *       sourced from MatrixMiniR4's `MiniR4DriveDC.h` TT defaults (`setMoveSyncPID` /
 *       `setMoveGyroPID` / `setTurnGyroPID` docstrings). If we ever switch motor families,
 *       re-tune in this section, not via a compile-time flag.
 */
namespace Config
{
    // ---------------------------------------------------------------------------------------
    // Battery
    // ---------------------------------------------------------------------------------------

    /** @brief Battery cell count passed to `MiniR4.PWR.setBattCell()`. MUST match the pack. */
    constexpr uint8_t BATT_CELLS = 2;

    // ---------------------------------------------------------------------------------------
    // Motor IDs and direction
    // ---------------------------------------------------------------------------------------

    /** @brief Motor controller ID for the left drive wheel (M2 per WRO chassis convention). */
    constexpr uint8_t MOTOR_LEFT_ID = 2;     // M2

    /** @brief Motor controller ID for the right drive wheel (M3 per WRO chassis convention). */
    constexpr uint8_t MOTOR_RIGHT_ID = 3;    // M3

    /** @brief Reverse the left motor's positive-power direction at the driver level. */
    constexpr bool MOTOR_LEFT_REVERSE = true;

    /** @brief Reverse the right motor's positive-power direction at the driver level. */
    constexpr bool MOTOR_RIGHT_REVERSE = false;

    // ---------------------------------------------------------------------------------------
    // Motor profile + PID gains (TT-motor)
    // ---------------------------------------------------------------------------------------
    //
    // PID gains taken from MatrixMiniR4 `MiniR4DriveDC.h` TT defaults:
    //   - setMoveSyncPID  TT: Kp=0.02,  Ki=0.00, Kd=0.04
    //   - setMoveGyroPID  TT: Kp=6.01,  Ki=0.00, Kd=2.15
    //   - setTurnGyroPID  TT: Kp=22.75, Ki=0.08, Kd=0.25
    //

    /** @brief Encoder pulses per output-shaft revolution for the TT motor. */
    constexpr uint16_t MOTOR_PPR = 545;

    /** @brief Nominal no-load RPM for the TT motor at full power. */
    constexpr uint16_t MOTOR_RPM = 200;

    /** @brief MoveSync (left/right encoder match) Kp. */
    constexpr float MOVESYNC_KP = 0.02f;
    /** @brief MoveSync Ki. */
    constexpr float MOVESYNC_KI = 0.0f;
    /** @brief MoveSync Kd. */
    constexpr float MOVESYNC_KD = 0.04f;

    /** @brief MoveGyro (straight-line yaw lock) Kp. */
    constexpr float MOVEGYRO_KP = 6.01f;
    /** @brief MoveGyro Ki. */
    constexpr float MOVEGYRO_KI = 0.0f;
    /** @brief MoveGyro Kd. */
    constexpr float MOVEGYRO_KD = 2.15f;

    /** @brief TurnGyro (in-place rotation) Kp. */
    constexpr float TURNGYRO_KP = 22.75f;
    /** @brief TurnGyro Ki. */
    constexpr float TURNGYRO_KI = 0.08f;
    /** @brief TurnGyro Kd. */
    constexpr float TURNGYRO_KD = 0.25f;

    /** @brief Line-follower outer-loop Kp. @note TODO: tune on real strip. */
    constexpr float LINE_KP = 8.0f;
    /** @brief Line-follower outer-loop Ki. @note TODO: tune on real strip. */
    constexpr float LINE_KI = 0.0f;
    /** @brief Line-follower outer-loop Kd. @note TODO: tune on real strip. */
    constexpr float LINE_KD = 4.0f;

    // ---------------------------------------------------------------------------------------
    // Geometry (TODO: measure on real chassis)
    // ---------------------------------------------------------------------------------------

    /** @brief Drive-wheel diameter in millimetres. @note TODO: measure on real chassis. */
    constexpr float WHEEL_DIAM_MM = 60.0f;

    /** @brief Wheel-to-wheel track width in millimetres. @note TODO: measure on real chassis. */
    constexpr float TRACK_MM = 150.0f;

    // ---------------------------------------------------------------------------------------
    // Speed presets (consumed by the fluent Drive / LineFollower APIs)
    // ---------------------------------------------------------------------------------------
    //
    // The fluent chassis API takes `SpeedPreset preset = SpeedPreset::NORMAL` as its default
    // power source. Each preset maps to a PWM in the [-100, 100] DriveDC range below. Callers
    // that need a non-preset value can still override via `.setPower(int16_t)` on the builder.
    //

    /**
     * @brief Coarse speed selector for chassis movement primitives.
     *
     * Used by every fluent verb that takes a power: Drive.driveDist / driveTime /
     * forwardGyro / turnDeg / turnToYaw, plus LineFollower.setPreset for cruise. Default is
     * `NORMAL`; pass `SLOW` for intersection approaches and tight corners, `FAST` for long
     * straights.
     */
    enum class SpeedPreset : uint8_t
    {
        SLOW,
        NORMAL,
        FAST
    };

    /** @brief PWM target for `SpeedPreset::SLOW` (intersections, tight corners). */
    constexpr int16_t SPEED_SLOW = 30;

    /** @brief PWM target for `SpeedPreset::NORMAL` (default cruise / general moves). */
    constexpr int16_t SPEED_NORMAL = 50;

    /** @brief PWM target for `SpeedPreset::FAST` (long straights, low-precision moves). */
    constexpr int16_t SPEED_FAST = 80;

    // =============================================================================
    // PWM bounds (saturation cap + motor deadband floor).
    //
    // Two physical properties of the TT motors:
    //   * Saturation: PWM above ~95% gives no headroom for closed-loop corrections
    //     — anything beyond gets clipped by the lower MCU.
    //   * Deadband: PWM below ~20% can't break static friction; the motor stays
    //     still even though a non-zero command was issued. Any P+D correction in
    //     the (0, 20) range would be silently lost without compensation.
    //
    // `deadbandSnap()` (defined in WROMath.h) takes a raw PWM and returns a value
    // in {0} U [PWM_MIN_MAGNITUDE, PWM_MAX] (with mirrored negative range): zero
    // stays zero, non-zero values below the floor snap UP to the floor magnitude,
    // values above the cap clamp DOWN to the cap. Apply this at every actuator
    // boundary where a computed PWM hits the lower MCU.
    // =============================================================================

    /** @brief Saturation cap — any |PWM| above this gets clipped to this magnitude. */
    constexpr int16_t PWM_MAX = 95;

    /**
     * @brief Motor deadband floor — non-zero |PWM| below this can't break stiction.
     *
     * `deadbandSnap()` snaps non-zero commands UP to this floor (preserving sign)
     * so P+D corrections always result in motion. Zero commands pass through —
     * the caller intended to stop, deadband is irrelevant.
     */
    constexpr int16_t PWM_MIN_MAGNITUDE = 20;

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
    constexpr int16_t LINE_MAX_CORR = 40;

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
     * 1 = front board only (typical WRO setup); 2 = front + rear boards present.
     * When set to 1, LINE_REAR_I2C_PORT is ignored at compile time.
     */
    constexpr uint8_t LINE_BOARD_COUNT = 1;

    static_assert(LINE_BOARD_COUNT == 1 || LINE_BOARD_COUNT == 2,
                  "Config::LINE_BOARD_COUNT must be 1 or 2 — WROLineArrayClass has fixed [2][8] "
                  "storage and front/rear semantics. Update both this assertion AND the array "
                  "dimensions if a third board is ever added.");

    /**
     * @brief I²C port index (1..4) on Wire1 carrying the FRONT line-sensor board.
     *
     * Maps to the hardware I2C1..I2C4 external ports on the MATRIX Mini R4.
     * Port 0 is the on-board Wire — never use 0 for an external sensor board.
     */
    constexpr uint8_t LINE_FRONT_I2C_PORT = 1;

    /**
     * @brief I²C port index (1..4) on Wire1 carrying the REAR line-sensor board.
     *
     * Ignored when LINE_BOARD_COUNT == 1. Must be distinct from LINE_FRONT_I2C_PORT.
     */
    constexpr uint8_t LINE_REAR_I2C_PORT = 4;

    /**
     * @brief Per-board, per-channel calibrated WHITE (off-line) reflectance value.
     *
     * Dimension: [LINE_BOARD_COUNT][8]. Channel values are in the 0..9 packed-byte
     * range produced by the MXColor firmware after Layer-1 hardware calibration.
     *
     * Workflow: run WROLineArray::calibrate() with the sensor over a WHITE surface,
     * read the Serial output, and paste the 8 values per board into the inner arrays
     * here, then reflash. Values survive power cycles because they live in firmware
     * (Layer-2 calibration). Zeroed defaults are safe before first calibration —
     * they cause every channel to be treated as "fully bright = off-line".
     */
    constexpr uint8_t LINE_CALIB_LOW[2][8] = {
        {0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0},
    };

    /**
     * @brief Per-board, per-channel calibrated DARK (on-line) reflectance value.
     *
     * Dimension: [LINE_BOARD_COUNT][8]. Channel values are in the 0..9 packed-byte
     * range produced by the MXColor firmware after Layer-1 hardware calibration.
     *
     * Workflow: run WROLineArray::calibrate() with the sensor over the DARK line,
     * read the Serial output, and paste the 8 values per board into the inner arrays
     * here, then reflash. Defaults of 9 (maximum brightness delta) are safe before
     * first calibration — they prevent false on-line detections when LOW == HIGH.
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
     * Applied per-channel: onLine = (raw - LOW[b][ch]) / (HIGH[b][ch] - LOW[b][ch])
     *                               >= LINE_CHANNEL_ON_RATIO.
     */
    constexpr float LINE_CHANNEL_ON_RATIO = 0.5f;

    /**
     * @brief Minimum number of channels simultaneously ON-LINE to declare a
     *        T_OR_CROSS junction (highest-priority junction class, checked first).
     *
     * At 7 of 8 channels lit, the robot is clearly centred on a wide cross or T
     * marker. Lower values risk false triggers on debris or sensor variation;
     * higher values may miss junctions under partial occlusion.
     *
     * @note TODO: tune on real mat after per-channel calibration is complete.
     */
    constexpr uint8_t LINE_JUNCTION_T_MIN_ON = 7;

    /**
     * @brief Upper inclusive channel index for the LEFT junction predicate.
     *
     * LEFT junction fires when all channels ch[0..LINE_JUNCTION_LEFT_HI] are ON-LINE
     * AND all channels ch[LINE_JUNCTION_LEFT_HI+1..7] are OFF-LINE. Adjust to match
     * the physical width of the left-branch marker relative to the sensor array.
     *
     * @note TODO: tune on real mat.
     */
    constexpr uint8_t LINE_JUNCTION_LEFT_HI = 4;

    /**
     * @brief Lower inclusive channel index for the RIGHT junction predicate.
     *
     * RIGHT junction fires when all channels ch[LINE_JUNCTION_RIGHT_LO..7] are ON-LINE
     * AND all channels ch[0..LINE_JUNCTION_RIGHT_LO-1] are OFF-LINE. Adjust to match
     * the physical width of the right-branch marker relative to the sensor array.
     *
     * @note TODO: tune on real mat.
     */
    constexpr uint8_t LINE_JUNCTION_RIGHT_LO = 3;

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
     * When `false` (default — legacy / current PR behaviour): each call to
     * `getError()`, `isOnline()`, and `getJunctionType()` independently issues
     * a full 4-byte CMYK I²C transaction. `WROLineArrayClass::update()` is a
     * no-op. Total per cooperative tick: 3 sensor reads (~33 ms at vendor I²C
     * timing), giving ~28-30 Hz loop rate.
     *
     * When `true`: `WROLineArrayClass::update()` issues the single sensor
     * read for the tick (must be called at the top of `cooperativeTick()` for
     * this to work). The three getters become pure cache readers. Total per
     * tick: 1 sensor read (~11 ms), giving ~70-80 Hz loop rate.
     *
     * Default is `false` to preserve the currently-tested code path on first
     * flash. Flip to `true` after hardware bring-up confirms the cached path
     * behaves identically (no controller wobble from one-tick-stale data on
     * a chassis travelling at typical cruise).
     *
     * See PR #20 review thread + DECISIONS.md if added later.
     */
    constexpr bool LINE_ARRAY_TICK_CACHE_ENABLED = false;

    // ---------------------------------------------------------------------------------------
    // Turn defaults
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Default wheel-pattern selector for the custom TurnGyro state machine.
     *
     * Encoding (NEW — set by the 2026-05-17 PR4 plan; intentionally different from
     * the legacy `MiniR4.DriveDC.TurnGyro` byte values):
     *   - `0` = COUNTER_ROT — both wheels spin opposite directions, chassis pivots
     *                         around its center. Tightest in-place rotation.
     *   - `1` = PIVOT_LEFT  — left wheel braked, right wheel drives; chassis pivots
     *                         around the LEFT wheel.
     *   - `2` = PIVOT_RIGHT — right wheel braked, left wheel drives; chassis pivots
     *                         around the RIGHT wheel.
     *
     * @note `TurnDegBuilder::setMode` / `TurnToYawBuilder::setMode` accept this byte;
     *       PathTracker maps `Routes::TurnMode::*` to it via `_turnModeToByte`. See
     *       `Drive.h::TurnDegBuilder::setMode` for the full encoding contract.
     */
    constexpr uint8_t TURN_MODE = 0;

    /** @brief Apply active brake at end of turn (vs. coast). */
    constexpr bool TURN_BRAKE = true;

    // ---------------------------------------------------------------------------------------
    // Line-sensor → axle geometric offset (intersection overshoot compensation)
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Distance from the line-sensor centerline to the wheel-axle line, in cm.
     *
     * Positive = sensor is in front of (ahead of) the axle.
     *
     * Used by the PathTracker LINE_OVERSHOOT state: the intersection-detect event fires
     * when the SENSOR is over the junction marker, but we want the chassis CENTER (the
     * axle midpoint) to land on the node. After detecting the intersection, PathTracker
     * re-arms LineFollower with `setDistanceTarget(LINE_SENSOR_AXLE_OFFSET_CM)` to roll
     * forward by exactly this amount before stopping.
     *
     * @note TODO: measure on real chassis.
     */
    constexpr float LINE_SENSOR_AXLE_OFFSET_CM = 5.0f;

    // ---------------------------------------------------------------------------------------
    // Custom TurnGyro state machine (replaces MiniR4.DriveDC.TurnGyro)
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Cruise PWM for `Drive.turnDeg` / `Drive.turnToYaw`, in [-100, 100].
     *
     * Two roles:
     *   1. Default value of `TurnDegBuilder::_power` / `TurnToYawBuilder::_power`;
     *      callers can override per-call via `.setPower(...)` on the builder.
     *   2. Fast-phase PWM magnitude for the custom TurnGyro state machine. Applied
     *      until the heading error falls within ±TURN_SLOW_ZONE_DEG of the target,
     *      after which the turn ramps down to `TURN_SLOW_SPEED_PWM` (not overridable).
     *
     * @note TODO: tune on real chassis.
     */
    constexpr int16_t TURN_CRUISE_PWM = 50;

    /**
     * @brief Slow-zone half-width in degrees.
     *
     * When |heading error| ≤ this value, the turn ramps down from TURN_CRUISE_PWM to
     * TURN_SLOW_SPEED_PWM. Sized to give enough time to bleed rotational momentum
     * before the tolerance gate fires.
     *
     * @note TODO: tune on real chassis.
     */
    constexpr float TURN_SLOW_ZONE_DEG = 30.0f;

    /**
     * @brief PWM applied in the slow zone, in [-100, 100].
     *
     * Held constant while |heading error| > TURN_TOLERANCE_DEG. Set equal to
     * Config::PWM_MIN_MAGNITUDE so the motor reliably breaks static friction even
     * at minimum command.
     *
     * @note TODO: tune on real chassis.
     */
    constexpr int16_t TURN_SLOW_SPEED_PWM = 20;

    /**
     * @brief Final heading tolerance in degrees.
     *
     * When |heading error| ≤ this value, the TurnGyro state machine brakes and
     * declares the turn complete.
     *
     * @note TODO: tune on real chassis.
     */
    constexpr float TURN_TOLERANCE_DEG = 1.0f;

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

    // ---------------------------------------------------------------------------------------
    // OLED event trace
    // ---------------------------------------------------------------------------------------

    /** @brief Number of recent events kept in the EventTrace ring buffer for the OLED. */
    constexpr uint8_t EVENT_TRACE_CAPACITY = 4;

    // =========================================================================================
    // Motor-bias calibration (Strategy B from Phase 1.5 chassis-API research).
    //
    // TT-motors of the same model can vary 10-20% in RPM at the same PWM. Our
    // raw-PWM Drive paths (setSpeed, driveDist, driveTime — all routed through
    // MatrixMiniR4 Move/MoveDegs/MoveTime, which are open-loop) are the ones
    // affected; Sync/Gyro variants close the loop on the STM32 and don't need it.
    //
    // Workflow: at first hardware bring-up, call Drive.calibrateMotorBias()
    // once with the wheels free of the ground. The routine prints two scalars
    // to Serial; copy them into MOTOR_BIAS_L / MOTOR_BIAS_R below. From then on,
    // Drive.begin() applies them automatically via setMotorBias().
    //
    // 1.0f = no correction (default; safe before calibration).
    //
    // See DECISIONS.md "Motor-bias calibration (Strategy B)" for full rationale,
    // including the on-shelf Strategy C (R4-side encoder-rate PID) we'd add if
    // B alone is not enough on the actual track.
    // =========================================================================================

    /** @brief Static manufacturer-bias scalar applied to the LEFT motor in raw-PWM paths. */
    static constexpr float MOTOR_BIAS_L = 1.0f;

    /** @brief Static manufacturer-bias scalar applied to the RIGHT motor in raw-PWM paths. */
    static constexpr float MOTOR_BIAS_R = 1.0f;

    // Calibration-routine defaults. The routine spins both motors at this power
    // for this duration, then computes scalars from the resulting encoder deltas.
    // 50% power for 2 s gives ~1-2k encoder counts on TT motors — plenty of signal,
    // not enough to spin the chassis off a bench in wheels-up mode.

    /** @brief PWM power used by `Drive.calibrateMotorBias()` during the wheels-free spin. */
    static constexpr int16_t MOTOR_BIAS_CALIB_POWER = 50;

    /** @brief Spin duration in milliseconds used by `Drive.calibrateMotorBias()`. */
    static constexpr uint32_t MOTOR_BIAS_CALIB_DURATION_MS = 2000;

    // =============================================================================
    // Trapezoidal velocity profile (`.tShape()` modifier on Drive / LineFollower).
    //
    // Acceleration-based: speed ramps at TSHAPE_ACCEL_PWM_PER_CM (PWM units gained
    // per cm of encoder travel) until the cruise target is reached, then plateaus,
    // then ramps back down at the same rate. Per-side ramp distance per move is
    // implicit: (cruise - endpointSpeed) / accel, clamped to [MIN_CM, MAX_CM] and
    // to half of the total move length. If accel- and decel-distances together
    // exceed total, the cruise plateau collapses (triangle fallback) and the peak
    // speed is reduced so the integral still matches the requested distance.
    //
    // Endpoint-default convention: when a move is configured `.setBrake(false)`,
    // the default `endSpeed` resolves to `cruise` (not 0) — the assumption is that
    // a no-brake move is chaining into a follow-up. Caller can override with an
    // explicit `.setEndSpeed(0)` if they really want a coast-to-stop.
    //
    // See DECISIONS.md "Trapezoidal velocity profile" for full rationale.
    // =============================================================================

    /** @brief PWM units gained per cm of travel during the ramp phase. */
    constexpr float TSHAPE_ACCEL_PWM_PER_CM = 5.0f;

    /** @brief Floor on the per-side ramp distance — never shorter than this (cm). */
    constexpr float TSHAPE_RAMP_MIN_CM = 2.0f;

    /** @brief Ceiling on the per-side ramp distance — never longer than this (cm). */
    constexpr float TSHAPE_RAMP_MAX_CM = 20.0f;

    /**
     * @brief Default starting speed for a profiled move (raw PWM).
     *
     * Set to `PWM_MIN_MAGNITUDE` (the motor deadband floor) rather than 0 so the
     * very first ramp tick emits a PWM the motor can actually act on. Starting at
     * 0 would waste the first ~20% of the ramp distance with the chassis sitting
     * still while the speed setpoint climbs through the dead zone.
     *
     * Override per-call with `.setStartSpeed(0)` if a true rest-start is desired
     * (e.g. for very long ramps where the lost first centimetres don't matter).
     */
    constexpr int16_t TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE;

    /**
     * @brief Default ending speed for a profiled move (raw PWM). 0 = end at rest.
     *
     * Stays at 0 (no symmetric deadband floor) by design: the motor naturally
     * stalls in the last bit when the ramp passes through the dead zone, then
     * `stop()` engages the brake immediately — the chassis comes to rest cleanly
     * without needing a ramp-down floor.
     */
    constexpr int16_t TSHAPE_END_SPEED = 0;

    /**
     * @brief Hysteresis: if `|cruise - endpointSpeed| < TSHAPE_NO_RAMP_EPSILON_PWM`,
     *        skip the ramp on that side entirely.
     *
     * Prevents the MIN_CM floor from forcing a tiny pointless ramp on chained moves
     * where startSpeed or endSpeed is already at (or very near) cruise.
     */
    constexpr int16_t TSHAPE_NO_RAMP_EPSILON_PWM = 2;

    /** @brief Tick-rate target for the profile state machine — only used as a comment marker. */
    // Profile is driven from Drive.update() at whatever rate the main loop ticks (~50-200 Hz typical).

    /**
     * @brief Minimum move length below which `.tShape()` silently no-ops.
     *
     * Below this threshold, the half-of-total clamp on per-side ramp distances would force
     * the effective acceleration steeper than `TSHAPE_ACCEL_PWM_PER_CM`, breaking the
     * constant-acceleration contract. Cleaner to fall back to a non-profiled constant-cruise
     * move below the threshold than to deliver an inconsistent ramp.
     *
     * @note When tShape no-ops, the move still runs — just without the ramp. Caller does not
     *       need to special-case short moves.
     */
    constexpr float TSHAPE_MIN_MOVE_CM = 5.0f;

    /** @brief Same threshold as TSHAPE_MIN_MOVE_CM but for time-bounded moves (seconds). */
    constexpr float TSHAPE_MIN_MOVE_SEC = 0.4f;

    // =============================================================================
    // Drive command queue (sequential dispatch).
    //
    // Drive's fluent builder destructors enqueue commands instead of dispatching
    // immediately. `Drive.update()` pops the next command when the current bounded
    // move + profile state machine are both idle. This makes source-order sequencing
    // of multiple moves work without caller-side polling — the future auto-path
    // planner can issue dozens of moves and they execute in order:
    //
    //   Drive.driveDist(30).tShape();
    //   Drive.driveDist(20).tShape();
    //   Drive.turnDeg(90);
    //
    // Blocking variants (`.setBlocking(true)`) bypass the queue: they drain existing
    // entries first, then dispatch synchronously.
    //
    // `Drive.stop()` clears the queue (abort semantics).
    //
    // See DECISIONS.md "Drive command queue" for full rationale.
    // =============================================================================

    /**
     * @brief Maximum number of pending commands the Drive queue can hold.
     *
     * 8 is comfortably more than a typical multi-segment maneuver and keeps the
     * struct array under 1 KB even with the largest command POD. Overflow drops
     * the new command and prints a Serial warning — that's a caller bug.
     */
    constexpr uint8_t DRIVE_QUEUE_CAPACITY = 8;

    /**
     * @brief Per-command worst-case ceiling used by `_drainBlocking` to bound its
     *        synchronous spin (code review F3).
     *
     * A typical 30 cm @ 50 PWM move completes in ~1 s; 10 s is a 10× safety factor
     * that still fits the 2-minute WRO run clock even if a full queue (8 cmds plus
     * the in-flight one) all hit the ceiling. The drain budget is computed as
     * `(qCount + 1) * DRIVE_DRAIN_TIMEOUT_MS_PER_CMD`. On expiry the drain stops the
     * chassis, clears the queue, and surfaces a Serial warning.
     */
    constexpr uint32_t DRIVE_DRAIN_TIMEOUT_MS_PER_CMD = 10000;

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
    constexpr uint8_t INTERSECTION_DEBOUNCE_TICKS = 2;

    /**
     * @brief Consecutive NONE samples required to release before re-arming.
     */
    constexpr uint8_t INTERSECTION_RELEASE_TICKS = 2;

    // =========================================================================================
    // PathTracker queue + drain timeout + heading tolerance (Phase 2 PR2).
    //
    // PathTracker.goTo() returns a fluent builder that auto-enqueues a goal on the FIFO. The
    // destructor sync-blocks by default (drains the queue via the registered cooperative tick
    // callback) so source-order chains like
    //
    //   PathTracker.goTo("C");
    //   PathTracker.goTo("G");
    //
    // execute one-after-the-other with no caller polling. Chain `.async()` on the builder to
    // opt out (auto-planner case where the caller ticks loop() and polls PathTracker.isIdle()).
    //
    // See docs/plans/2026-05-06-phase2-pr2-path-tracker-runtime.md and
    // user_fluent_api_preference.md for the rationale.
    // =========================================================================================

    /**
     * @brief Maximum number of pending goTo() entries the PathTracker queue can hold.
     *
     * Bumped 8→32 to accommodate missions chaining 10+ destinations; trivial RAM cost
     * (32 bytes since GoalEntry/NavEntry is a single uint8_t). Overflow drops the new
     * entry and prints a Serial warning — that is a caller bug.
     */
    constexpr uint8_t PATH_QUEUE_CAPACITY = 32;

    /**
     * @brief Per-goal worst-case ceiling used by `PathTracker._drainBlocking` to bound
     *        the synchronous spin (mirrors `DRIVE_DRAIN_TIMEOUT_MS_PER_CMD`).
     *
     * 60 s per goal accommodates a worst-case multi-edge traverse on the toy graph at slow
     * cruise. Drain budget is `(qCount + 1) * PATH_DRAIN_TIMEOUT_MS_PER_GOAL`. On expiry,
     * the chassis is stopped, the queue cleared, and a fault is latched.
     */
    constexpr uint32_t PATH_DRAIN_TIMEOUT_MS_PER_GOAL = 60000;

    /**
     * @brief Tolerance (degrees) below which `PathTracker` skips an absolute turn dispatch.
     *
     * Avoids a sub-degree twitch at every edge join when consecutive edges happen to be
     * colinear (e.g. mid-segment projection-node hand-off). 0.5 deg is well below the
     * `Drive.turnToYaw` resolvable range.
     */
    constexpr float NAV_TURN_TOLERANCE_DEG = 0.5f;

    // =========================================================================================
    // Mission DSL (PR3).
    //
    // Mission is a thin, fully-manual scheduler the operator hand-writes in
    // WRO2026.ino. Each `Mission.<verb>(...)` call enqueues one step; the
    // queue advances entirely from `Mission.update()` in the cooperative
    // tick. No automatic planning — that is intentional for PR3.
    //
    // On PathTracker fault, Mission halts the chassis, beeps five long
    // tones, flashes the RGB LEDs red, and waits for BTN_DOWN to clear.
    // =========================================================================================

    /** @brief Maximum number of pending Mission steps (no dynamic alloc). */
    constexpr uint8_t MAX_MISSION_STEPS = 64;

    /** @brief Tone frequency used for the halt-on-fault beep sequence. */
    constexpr uint16_t MISSION_FAULT_BEEP_HZ = 1000;

    /** @brief Duration of each on/off pulse in the halt-on-fault beep sequence. */
    constexpr uint16_t MISSION_FAULT_BEEP_MS = 500;

    /** @brief Number of beeps emitted before Mission awaits BTN_DOWN to clear. */
    constexpr uint8_t MISSION_FAULT_BEEPS = 5;

    /** @brief Interval at which EventTrace emits its `HB ...` status heartbeat. */
    constexpr uint32_t MISSION_HEARTBEAT_MS = 500;
}    // namespace Config

#endif    // CONFIG_H
