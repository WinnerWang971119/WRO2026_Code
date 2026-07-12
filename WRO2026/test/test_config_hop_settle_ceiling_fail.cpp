/**
 * @file test_config_hop_settle_ceiling_fail.cpp
 * @brief TC12 — compile-fail probe proving the `HOP_SETTLE_MS` ceiling static_assert in
 *        `ConfigChecks.h` fires for an over-ceiling value (ADR-061, hop-settle-wait plan).
 *
 * @details
 * The hop-settle-wait plan's `ConfigChecks.h` static_assert guards `Config::HOP_SETTLE_MS <= 2000`
 * so a settle dwell longer than ~2 s per braked hop cannot silently eat into the
 * `PATH_DRAIN_TIMEOUT_MS_PER_GOAL` drain budget. This TU just `#include`s `Config.h` (which pulls
 * `ConfigChecks.h` last); the CMake target `test_config_hop_settle_ceiling_fail` compiles it with
 * `-DHOP_SETTLE_MS_DEFAULT=3000`, an over-ceiling value, so the static_assert fires and the build
 * FAILS. The ctest (`config_hop_settle_ceiling_compile_fail`) uses `PASS_REGULAR_EXPRESSION` on the
 * distinctive, metacharacter-free substring `Config::HOP_SETTLE_MS must be <= 2000 ms` so it PASSES
 * only when the intended assert fires (never `WILL_FAIL`, which would pass on any build break — see
 * memory `gotcha_ctest_compile_fail_assertion`).
 *
 * With NO `-D` override `HOP_SETTLE_MS` takes its default (100) from `ConfigNav.h` (legal), so the
 * assert passes and this TU compiles cleanly — expected behaviour, not a test failure. Mirrors the
 * `test_manipulator_config_fail.cpp` minimal-TU shape.
 */

#include "Config.h"

int main()
{
    return 0;
}
