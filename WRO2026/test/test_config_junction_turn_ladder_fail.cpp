/**
 * @file test_config_junction_turn_ladder_fail.cpp
 * @brief TC10a-TC10c — Compile-fail targets proving the junction-knob range asserts in
 *        `ConfigChecks.h` fire for out-of-range configs (issue #152 config hardening).
 *
 * @note The three turn-ladder probes (TC10d-TC10f) were removed when ADR-068's per-tick PD
 *       turn controller replaced the two-stage turn model — `TURN_SLOW_SPEED_PWM` /
 *       `TURN_SLOW_ZONE_DEG` and their turn-ladder asserts no longer exist. The file name is
 *       kept to avoid churning the CMake targets and the host-coverage baseline.
 *
 * Three `EXCLUDE_FROM_ALL` + `EXCLUDE_FROM_DEFAULT_BUILD` CMake targets each compile THIS SAME
 * minimal TU with a different `-D..._DEFAULT` override, forcing exactly one of the three
 * junction `ConfigChecks.h` static_asserts to fire:
 *
 *   TC10a — `-DLINE_JUNCTION_LEFT_HI_DEFAULT=8`
 *     Forces `LINE_JUNCTION_LEFT_HI == 8`, past the 8-channel array's last valid index (7).
 *     Fires: "Config::LINE_JUNCTION_LEFT_HI must be <= 7 -- ..."
 *     PASS_REGULAR_EXPRESSION substring (no metacharacters):
 *       `LINE_JUNCTION_LEFT_HI must be <= 7`
 *
 *   TC10b — `-DLINE_JUNCTION_RIGHT_LO_DEFAULT=0`
 *     Forces `LINE_JUNCTION_RIGHT_LO == 0`, below the `[1, 7]` floor.
 *     Fires: "Config::LINE_JUNCTION_RIGHT_LO must be between 1 and 7 inclusive -- ..."
 *     PASS_REGULAR_EXPRESSION substring:
 *       `LINE_JUNCTION_RIGHT_LO must be between 1 and 7 inclusive`
 *
 *   TC10c — `-DLINE_JUNCTION_T_MIN_ON_DEFAULT=9`
 *     Forces `LINE_JUNCTION_T_MIN_ON == 9`, past the 8-channel array's channel-lit ceiling.
 *     Fires: "Config::LINE_JUNCTION_T_MIN_ON must be between 1 and 8 inclusive -- ..."
 *     PASS_REGULAR_EXPRESSION substring:
 *       `LINE_JUNCTION_T_MIN_ON must be between 1 and 8 inclusive`
 *
 * With NO `-D` override every knob takes its shipped default (all legal), so all three asserts
 * pass and this TU compiles cleanly -- that is expected behaviour, not a test failure. Mirrors
 * the `test_manipulator_config_fail.cpp` / `test_config_hop_settle_ceiling_fail.cpp` minimal-TU
 * shape (never `WILL_FAIL` -- memory `gotcha_ctest_compile_fail_assertion`); each override is a
 * `target_compile_definitions` entry, never an in-file `#define` (memory
 * `gotcha-cross-tu-config-override-compile-def`).
 *
 * @note Requirement (mirrors `test_manipulator_config_fail.cpp`): the CMake target must add
 *       both the sketch directory (for the quoted `"Config.h"` include chain to resolve) AND
 *       the `mocks/` directory (so `#include <Arduino.h>` inside `Config.h` resolves without
 *       the real SDK).
 */

#include "Config.h"

int main()
{
    return 0;
}
