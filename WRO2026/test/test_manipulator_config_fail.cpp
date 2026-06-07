/**
 * @file test_manipulator_config_fail.cpp
 * @brief TC13/TC14 — Compile-fail targets proving the manipulator
 *        ``static_assert``s in ``ConfigChecks.h`` fire for invalid configs.
 *
 * AC2 negative branch. T11's CMake declares TWO ``EXCLUDE_FROM_ALL`` +
 * ``EXCLUDE_FROM_DEFAULT_BUILD`` targets, each compiling this same TU with a
 * different ``-D`` override:
 *
 *   TC13 — ``-DBELT_M_ID_DEFAULT=2``
 *     Forces ``BELT_M_ID == MOTOR_LEFT_ID`` (both are 2).  Fires:
 *       "BELT_M_ID collides with a drive motor (MOTOR_LEFT_ID/MOTOR_RIGHT_ID)."
 *     ``PASS_REGULAR_EXPRESSION`` substring (no metacharacters):
 *       ``collides with a drive motor``
 *
 *   TC14 — ``-DSORTER_RC_ID_DEFAULT=1``
 *     Forces ``SORTER_RC_ID == GRABBER_RC_ID`` (both are 1).  Fires:
 *       "Manipulator servo RC IDs must be pairwise distinct."
 *     ``PASS_REGULAR_EXPRESSION`` substring (no metacharacters):
 *       ``pairwise distinct``
 *
 * With NO ``-D`` override the two IDs take their defaults from
 * ``ConfigManipulator.h`` (legal values), so the asserts pass and this TU
 * compiles cleanly — that is expected behaviour, not a test failure.
 *
 * @note T11 requirement: the CMake target must add both the sketch directory
 *       (for the quoted ``"Config.h"`` include chain to resolve) AND the
 *       ``mocks/`` directory (so ``#include <Arduino.h>`` inside
 *       ``MatrixMiniR4.h`` / ``Config.h`` resolves without the real SDK).
 */

#include "Config.h"

int main()
{
    return 0;
}
