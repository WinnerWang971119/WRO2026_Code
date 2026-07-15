/**
 * @file test_config_junction_turn_ladder_fail.cpp
 * @brief TC10a-TC10f — Compile-fail targets proving the junction-knob range asserts and the
 *        turn-ladder invariant asserts in `ConfigChecks.h` fire for out-of-range configs
 *        (issue #152 config hardening).
 *
 * Six `EXCLUDE_FROM_ALL` + `EXCLUDE_FROM_DEFAULT_BUILD` CMake targets each compile THIS SAME
 * minimal TU with a different `-D..._DEFAULT` override, forcing exactly one of the six new
 * `ConfigChecks.h` static_asserts to fire:
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
 *   TC10d — `-DTURN_CRUISE_PWM_DEFAULT=100`
 *     Forces `TURN_CRUISE_PWM (100) > PWM_MAX (95)`, breaking the turn-ladder ordering chain
 *     (`TURN_SLOW_SPEED_PWM <= TURN_CRUISE_PWM <= PWM_MAX`) while `TURN_CRUISE_PWM` still clears
 *     `PWM_MIN_MAGNITUDE`, so ONLY the ordering assert fires.
 *     Fires: "Config::TURN_SLOW_SPEED_PWM/TURN_CRUISE_PWM/PWM_MAX turn-ladder ordering violated -- ..."
 *     PASS_REGULAR_EXPRESSION substring:
 *       `TURN_SLOW_SPEED_PWM/TURN_CRUISE_PWM/PWM_MAX turn-ladder ordering violated`
 *
 *   TC10e — `-DTURN_CRUISE_PWM_DEFAULT=10 -DTURN_SLOW_SPEED_PWM_DEFAULT=5`
 *     Forces `TURN_CRUISE_PWM (10) < PWM_MIN_MAGNITUDE (35)`. `TURN_SLOW_SPEED_PWM` is paired
 *     down to 5 in the SAME override so the ordering chain (`0 < 5 <= 10 <= 95`) still holds --
 *     ONLY the cruise-floor assert fires, isolated from TC10d's ordering assert.
 *     Fires: "Config::TURN_CRUISE_PWM must be >= PWM_MIN_MAGNITUDE -- ..."
 *     PASS_REGULAR_EXPRESSION substring:
 *       `TURN_CRUISE_PWM must be >= PWM_MIN_MAGNITUDE`
 *
 *   TC10f — `-DTURN_TOLERANCE_DEG_DEFAULT=35.0f`
 *     Forces `TURN_TOLERANCE_DEG (35.0) >= TURN_SLOW_ZONE_DEG (30.0)`, so the slow zone would
 *     never be strictly wider than the final tolerance gate.
 *     Fires: "Config::TURN_TOLERANCE_DEG must be < TURN_SLOW_ZONE_DEG -- ..."
 *     PASS_REGULAR_EXPRESSION substring:
 *       `TURN_TOLERANCE_DEG must be < TURN_SLOW_ZONE_DEG`
 *
 * With NO `-D` override every knob takes its shipped default (all legal), so all six asserts
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
