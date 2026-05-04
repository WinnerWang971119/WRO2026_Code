#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>

/*
For who ever is writing the code.
Please write your things and make modification here.
Thanks!
*/


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

    /** @brief Threshold passed to `MXLineTracer.setThreshold()` (0..100). */
    constexpr uint8_t LINE_THRESHOLD = 60;

    // ---------------------------------------------------------------------------------------
    // MXLineTracer wiring
    // ---------------------------------------------------------------------------------------

    /** @brief External I2C port index (I2C1 = first external port on Wire1). */
    constexpr uint8_t LINE_I2C_PORT = 1;

    // ---------------------------------------------------------------------------------------
    // Turn defaults
    // ---------------------------------------------------------------------------------------

    /** @brief Default rotation power for `DriveDC.TurnGyro` calls, in [-100, 100]. */
    constexpr int16_t TURN_POWER = 50;

    /** @brief Turn motor mode: 0 = single-motor pivot, 1 = two-motor counter-rotation. */
    constexpr uint8_t TURN_MODE = 0;

    /** @brief Apply active brake at end of turn (vs. coast). */
    constexpr bool TURN_BRAKE = true;

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
}    // namespace Config

#endif    // CONFIG_H
