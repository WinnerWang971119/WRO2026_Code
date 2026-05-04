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
 * @note Define `MOTOR_PROFILE_TT` (either as a build flag or `#define MOTOR_PROFILE_TT` above the
 *       `#include "Config.h"` line) to switch the encoder PPR / nominal RPM and the closed-loop
 *       PID gains from the LEGO-motor defaults to the TT-motor profile used by the
 *       `MiniR4_DriveDC_Test` example.
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
    // Motor profile + PID gains (LEGO motors are the default; #define MOTOR_PROFILE_TT to swap)
    // ---------------------------------------------------------------------------------------

#ifndef MOTOR_PROFILE_TT
    // ---- LEGO-motor profile (default) ------------------------------------------------------

    /** @brief Encoder pulses per output-shaft revolution for the LEGO motor. */
    constexpr uint16_t MOTOR_PPR = 360;

    /** @brief Nominal no-load RPM for the LEGO motor at full power. */
    constexpr uint16_t MOTOR_RPM = 300;

    /** @brief MoveSync (left/right encoder match) Kp. */
    constexpr float MOVESYNC_KP = 0.002f;
    /** @brief MoveSync Ki. */
    constexpr float MOVESYNC_KI = 0.0f;
    /** @brief MoveSync Kd. */
    constexpr float MOVESYNC_KD = 0.004f;

    /** @brief MoveGyro (straight-line yaw lock) Kp. */
    constexpr float MOVEGYRO_KP = 0.52f;
    /** @brief MoveGyro Ki. */
    constexpr float MOVEGYRO_KI = 0.0f;
    /** @brief MoveGyro Kd. */
    constexpr float MOVEGYRO_KD = 0.15f;

    /** @brief TurnGyro (in-place rotation) Kp. */
    constexpr float TURNGYRO_KP = 22.75f;
    /** @brief TurnGyro Ki. */
    constexpr float TURNGYRO_KI = 0.02f;
    /** @brief TurnGyro Kd. */
    constexpr float TURNGYRO_KD = 1.54f;
#else
    // ---- TT-motor profile (enabled via -DMOTOR_PROFILE_TT) --------------------------------

    /** @brief Encoder pulses per output-shaft revolution for the TT motor. */
    constexpr uint16_t MOTOR_PPR = 545;

    /** @brief Nominal no-load RPM for the TT motor at full power. */
    constexpr uint16_t MOTOR_RPM = 200;

    /** @brief MoveSync (left/right encoder match) Kp — TT profile. */
    constexpr float MOVESYNC_KP = 0.02f;
    /** @brief MoveSync Ki — TT profile. */
    constexpr float MOVESYNC_KI = 0.0f;
    /** @brief MoveSync Kd — TT profile. */
    constexpr float MOVESYNC_KD = 0.04f;

    /** @brief MoveGyro (straight-line yaw lock) Kp — TT profile. */
    constexpr float MOVEGYRO_KP = 6.01f;
    /** @brief MoveGyro Ki — TT profile. */
    constexpr float MOVEGYRO_KI = 0.0f;
    /** @brief MoveGyro Kd — TT profile. */
    constexpr float MOVEGYRO_KD = 2.15f;

    /** @brief TurnGyro (in-place rotation) Kp — TT profile. */
    constexpr float TURNGYRO_KP = 22.75f;
    /** @brief TurnGyro Ki — TT profile. */
    constexpr float TURNGYRO_KI = 0.08f;
    /** @brief TurnGyro Kd — TT profile. */
    constexpr float TURNGYRO_KD = 0.25f;
#endif

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
    // Cruise / line following
    // ---------------------------------------------------------------------------------------

    /** @brief Forward cruise power for line following, in the [-100, 100] DriveDC range. */
    constexpr int16_t CRUISE_SPEED = 50;

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
}    // namespace Config

#endif    // CONFIG_H
