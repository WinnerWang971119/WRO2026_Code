/**
 * @file test_gyro_config_fail.cpp
 * @brief TC22 / AC9 — Compile-fail target proving the GYRO_YAW_SIGN tripwire in
 *        ``ConfigChecks.h`` fires for a non-(+/-1) sign.
 *
 * ADR-070 negative branch. T6's CMake declares ONE ``EXCLUDE_FROM_ALL`` +
 * ``EXCLUDE_FROM_DEFAULT_BUILD`` target (``test_gyro_config_fail``) that compiles
 * this TU with the override:
 *
 *   TC22 — ``-DGYRO_YAW_SIGN_DEFAULT=0.5f``
 *     Forces ``Config::GYRO_YAW_SIGN == 0.5f`` (the ``#ifndef``-guarded hook in
 *     ``config/ConfigNav.h``) -- a value that is neither ``+1.0f`` nor ``-1.0f``, so it
 *     would silently scale (rather than just mirror) the decoded gyro heading. Fires:
 *       "Config::GYRO_YAW_SIGN must be exactly 1.0f or -1.0f -- it is the pure
 *        raw-gyro-to-app CCW sign applied by GyroDecodePolicy.h::decodeGyroYawDeg,
 *        not a scale factor. See config/ConfigNav.h."
 *     ``PASS_REGULAR_EXPRESSION`` substring (no metacharacters):
 *       ``GYRO_YAW_SIGN must be exactly``
 *
 * With NO ``-D`` override ``GYRO_YAW_SIGN`` takes its shipped default (``-1.0f``), the
 * assert passes, and this TU compiles cleanly -- that is expected behaviour, not a
 * test failure.
 *
 * @note Mirrors ``test_imu_yaw_sign_fail.cpp`` EXACTLY (same tripwire shape, one sign
 *       config constant over): the CMake target must add both the sketch directory
 *       (for the quoted ``"Config.h"`` include chain to resolve) AND the ``mocks/``
 *       directory (so ``#include <Arduino.h>`` inside ``Config.h`` resolves without
 *       the real SDK).
 */

#include "Config.h"

int main()
{
    return 0;
}
