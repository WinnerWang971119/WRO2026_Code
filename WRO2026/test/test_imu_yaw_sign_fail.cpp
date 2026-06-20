/**
 * @file test_imu_yaw_sign_fail.cpp
 * @brief TC7 / AC4 — Compile-fail target proving the IMU_YAW_SIGN tripwire in
 *        ``ConfigChecks.h`` fires for a non-(+/-1) sign.
 *
 * ADR-045 negative branch. T7's CMake declares ONE ``EXCLUDE_FROM_ALL`` +
 * ``EXCLUDE_FROM_DEFAULT_BUILD`` target (``test_imu_yaw_sign_zero``) that compiles
 * this TU with the override:
 *
 *   TC7 — ``-DIMU_YAW_SIGN_DEFAULT=0.0f``
 *     Forces ``Config::IMU_YAW_SIGN == 0.0f`` (the ``#ifndef``-guarded hook in
 *     ``config/ConfigNav.h``) — a value that would silently zero all heading math.
 *     Fires:
 *       "Config::IMU_YAW_SIGN must be exactly 1.0f or -1.0f -- it is the pure
 *        CW-to-CCW sign applied by YawSource::readYawDeg and YawSource::toHwHeading,
 *        not a scale factor. See config/ConfigNav.h."
 *     ``PASS_REGULAR_EXPRESSION`` substring (no metacharacters):
 *       ``IMU_YAW_SIGN must be exactly``
 *
 * With NO ``-D`` override ``IMU_YAW_SIGN`` takes its shipped default (``-1.0f``), the
 * assert passes, and this TU compiles cleanly — that is expected behaviour, not a
 * test failure.
 *
 * @note T7 requirement (mirrors ``test_manipulator_config_fail.cpp``): the CMake
 *       target must add both the sketch directory (for the quoted ``"Config.h"``
 *       include chain to resolve) AND the ``mocks/`` directory (so
 *       ``#include <Arduino.h>`` inside ``Config.h`` resolves without the real SDK).
 */

#include "Config.h"

int main()
{
    return 0;
}
