/**
 * @file test_gyro_hold_config_fail.cpp
 * @brief TC12a-TC12d / AC7 — Compile-fail probes proving the four `GYRO_HOLD_*` tripwires in
 *        `ConfigChecks.h` fire for an out-of-range gain/knob (ADR-072, issue #169).
 *
 * T8's CMake declares FOUR separate `EXCLUDE_FROM_ALL` + `EXCLUDE_FROM_DEFAULT_BUILD` targets,
 * all compiling THIS SAME minimal TU with one `-D..._DEFAULT` override each — mirroring
 * `test_config_junction_turn_ladder_fail.cpp`'s TC10a-TC10c multi-target-single-source pattern
 * (see `CMakeLists.txt`'s `test_config_turn_ladder_fail_tc10a/b/c` block) rather than
 * `test_gyro_config_fail.cpp` / `test_imu_yaw_sign_fail.cpp`'s one-target-one-source pattern
 * (those each pin exactly one sign constant, this file pins one of four independent knobs):
 *
 *   TC12a — `-DGYRO_HOLD_KP_DEFAULT=0.0f`
 *     Forces `Config::GYRO_HOLD_KP == 0.0f` — a zero proportional gain would apply no heading
 *     correction at all (`Drive::_emitGyroHold` has no other authority source to hold a GYRO
 *     straight to heading). Fires:
 *       "Config::GYRO_HOLD_KP must be > 0 ..."
 *     `PASS_REGULAR_EXPRESSION` substring (no metacharacters):
 *       `Config::GYRO_HOLD_KP must be > 0`
 *
 *   TC12b — `-DGYRO_HOLD_KD_DEFAULT=-0.01f`
 *     Forces `Config::GYRO_HOLD_KD == -0.01f` — a negative derivative gain would invert the
 *     damping term into positive feedback on yaw rate. Fires:
 *       "Config::GYRO_HOLD_KD must be >= 0 ..."
 *     `PASS_REGULAR_EXPRESSION` substring:
 *       `Config::GYRO_HOLD_KD must be >= 0`
 *
 *   TC12c — `-DGYRO_HOLD_MAX_CORR_PWM_DEFAULT=0`
 *     Forces `Config::GYRO_HOLD_MAX_CORR_PWM == 0` — zero disables all heading correction (a
 *     zero-width clamp). Fires:
 *       "Config::GYRO_HOLD_MAX_CORR_PWM must be in (0, PWM_MAX] ..."
 *     `PASS_REGULAR_EXPRESSION` substring (deliberately stops BEFORE the message's `(0,` —
 *     `(`/`)`/`]` are regex metacharacters, per `gotcha_ctest_compile_fail_assertion`):
 *       `Config::GYRO_HOLD_MAX_CORR_PWM must be in`
 *
 *   TC12d — `-DGYRO_HOLD_STALE_MS_DEFAULT=0`
 *     Forces `Config::GYRO_HOLD_STALE_MS == 0` — zero would force the derivative term to 0 on
 *     EVERY tick (even back-to-back ones), silently degrading the hold to P-only. Fires:
 *       "Config::GYRO_HOLD_STALE_MS must be > 0 ..."
 *     `PASS_REGULAR_EXPRESSION` substring:
 *       `Config::GYRO_HOLD_STALE_MS must be > 0`
 *
 * With NO `-D` override, all four `GYRO_HOLD_*` constants take their shipped (UNVERIFIED
 * bench-tuning placeholder) defaults, every assert passes, and this TU compiles cleanly — that
 * is expected behaviour, not a test failure.
 *
 * @note Mirrors `test_gyro_config_fail.cpp` / `test_imu_yaw_sign_fail.cpp` in TU shape (a bare
 *       `#include "Config.h"` + a no-op `main()`) and `test_config_junction_turn_ladder_fail.cpp`
 *       in multi-target reuse: each of the four CMake targets adds both the sketch directory
 *       (for the quoted `"Config.h"` include chain to resolve) AND the `mocks/` directory (so
 *       `#include <Arduino.h>` inside `Config.h` resolves without the real SDK) — NO
 *       `add_mock_force_includes`, so a mock typo elsewhere can never mask the intended
 *       diagnostic. Builds are EXPECTED TO FAIL; `PASS_REGULAR_EXPRESSION` (never `WILL_FAIL`,
 *       per `gotcha_ctest_compile_fail_assertion`) decides pass/fail purely on whether the
 *       regex is found in the build output, ignoring the exit code — so each of the four ctest
 *       entries passes ONLY when its OWN assert fires, never a coincidentally-earlier one.
 */

#include "Config.h"

int main()
{
    return 0;
}
