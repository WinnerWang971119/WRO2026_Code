# =========================================================================================
# WRO2026 / R4 firmware — host-compile test harness for LineFollower + PathTracker.
#
# Compiles the production `WRO2026/LineFollower.{h,cpp}` (and `WROMath.h`) against the
# mock Arduino / Drive / WROLineArray / Intersection surface under `WRO2026/test/mocks/`,
# producing standalone test executables that ctest can run on any host with a C++17
# compiler (g++, clang, MSVC).
#
# AC6, AC7b, AC9 are exercised by these tests:
#   - AC6: LineFollower overshoot-mode dead-reckon  (test_line_follower_overshoot)
#   - AC9: junction-freeze interaction               (test_line_follower_overshoot)
#   - AC7b: PATH_PASS_THROUGH_ENABLED behavioral toggle
#           (test_path_tracker_passthrough — built TWICE with the flag flipped).
#
# Usage:
#   cmake -S WRO2026/test -B build/test
#   cmake --build build/test
#   ctest --test-dir build/test --output-on-failure
#
# CI integration: NOT done in this PR. The follow-up will add a `host-compile-tests`
# job to `.github/workflows/ci.yml` once this harness is validated locally. The job
# should install a C++ compiler (g++ on ubuntu-latest is built-in), then run the
# three commands above.
# =========================================================================================

cmake_minimum_required(VERSION 3.16)
project(wro2026_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

# Absolute paths to source roots so the harness works regardless of build dir.
set(SKETCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(TEST_DIR   ${CMAKE_CURRENT_SOURCE_DIR})
set(MOCKS_DIR  ${TEST_DIR}/mocks)

# -----------------------------------------------------------------------------------------
# Production sources under test — compiled UNCHANGED.
#   - LineFollower.cpp pulls in Drive.h / WROLineArray.h / Intersection.h via the include
#     path, all of which resolve to mocks because `mocks/` comes first.
# -----------------------------------------------------------------------------------------
set(SUT_SOURCES
    ${SKETCH_DIR}/LineFollower.cpp
)

# -----------------------------------------------------------------------------------------
# Mock TU — defines `Drive`, `LineArray`, `Intersection`, `Serial`, `g_mockMillis`.
# -----------------------------------------------------------------------------------------
set(MOCK_SOURCES
    ${MOCKS_DIR}/mocks.cpp
)

# -----------------------------------------------------------------------------------------
# Common include search path. **`mocks/` MUST come BEFORE the sketch dir** so the
# `Arduino.h` / `MatrixMiniR4.h` / `Drive.h` / `WROLineArray.h` / `Intersection.h`
# headers under `mocks/` shadow the production hardware-bound versions.
# -----------------------------------------------------------------------------------------
set(COMMON_INCLUDES
    ${MOCKS_DIR}
    ${SKETCH_DIR}
    ${TEST_DIR}
)

# -----------------------------------------------------------------------------------------
# Compiler-warning settings — strict but not punitive. `LineFollower.cpp` is
# production code we don't want to retouch just to silence a warning the
# host compiler raises but Arduino's avr-g++ doesn't.
# -----------------------------------------------------------------------------------------
if(MSVC)
    add_compile_options(/W3 /permissive- /wd4244 /wd4267 /wd4305 /wd4127 /wd4456 /wd4458 /wd4189 /wd4834)
    # 4244/4267/4305 — narrowing int<->int16/float conversions (intentional in PWM clamp paths).
    # 4127 — conditional expression is constant (intentional: tShape gates).
    # 4456 — declaration hides previous local (the production code shadows `traveledCm` in two scopes).
    # 4458 — declaration hides class member (parameter / member name collisions in builder ctors).
    # 4189 — local variable initialized but not used (defensive guards like `clearedStaleEdge`).
    # 4834 — discarding return value with [[nodiscard]] (the LineFollower fluent builder is
    #        designed to be used as a temporary — `setEnabled(false);` is intentional usage).
else()
    # -Wno-unused-result / -Wno-attributes parallel MSVC's /wd4834: the
    # LineFollower fluent builder is `[[nodiscard]]` and `setEnabled(false);` as
    # a bare expression-statement is intentional usage. Without these flags
    # gcc/clang emit -Wunused-result on the test mains' `resetWorld()` helper
    # (and -Werror in a future CI tightening would break the host-compile job).
    add_compile_options(-Wall -Wextra
                        -Wno-narrowing
                        -Wno-sign-compare
                        -Wno-unused-but-set-variable
                        -Wno-unused-result
                        -Wno-attributes)
endif()

# -----------------------------------------------------------------------------------------
# Force-include the mock headers BEFORE any source-file parsing. This is the
# load-bearing trick that lets us shadow the production `Drive.h`,
# `WROLineArray.h`, and `Intersection.h` headers (which live next to
# `LineFollower.cpp`, so the `""` source-relative include rule would otherwise
# always pick the real ones over our mocks under `mocks/`).
#
# Each mock's include guard matches the production header's guard exactly
# (e.g. `DRIVE_H`, `WROLINEARRAY_H`, `INTERSECTION_H`), so once force-included
# the real headers no-op when LineFollower.h transitively re-requests them.
#
# `Arduino.h` and `MatrixMiniR4.h` are angle-bracket includes inside
# LineFollower.h, so the include-path search alone picks `mocks/`. No
# force-include needed for those two.
# -----------------------------------------------------------------------------------------
if(MSVC)
    add_compile_options(
        "/FI${MOCKS_DIR}/Drive.h"
        "/FI${MOCKS_DIR}/WROLineArray.h"
        "/FI${MOCKS_DIR}/Intersection.h"
    )
else()
    # Use the `SHELL:` prefix so CMake keeps each `-include <path>` pair atomic. Without it,
    # CMake de-duplicates the repeated `-include` flag, collapsing the three to one and
    # leaving the paths as bare arguments -- g++ then treats them as extra source files and
    # fails with "cannot specify '-o' with '-c' ... with multiple files".
    add_compile_options(
        "SHELL:-include ${MOCKS_DIR}/Drive.h"
        "SHELL:-include ${MOCKS_DIR}/WROLineArray.h"
        "SHELL:-include ${MOCKS_DIR}/Intersection.h"
    )
endif()

# =========================================================================================
# Test executable #1 — LineFollower overshoot / brake-on-gate / junction-freeze.
# Exercises T10a, T10b, T10d (AC6 + AC9).
# =========================================================================================
add_executable(test_line_follower_overshoot
    ${TEST_DIR}/test_line_follower_overshoot.cpp
    ${SUT_SOURCES}
    ${MOCK_SOURCES}
)
target_include_directories(test_line_follower_overshoot PRIVATE ${COMMON_INCLUDES})
add_test(NAME line_follower_overshoot COMMAND test_line_follower_overshoot)

# =========================================================================================
# Test executable #2a — PATH_PASS_THROUGH_ENABLED == true build.
# =========================================================================================
add_executable(test_path_tracker_passthrough_on
    ${TEST_DIR}/test_path_tracker_passthrough.cpp
    ${SUT_SOURCES}
    ${MOCK_SOURCES}
)
target_compile_definitions(test_path_tracker_passthrough_on
    PRIVATE PATH_PASS_THROUGH_ENABLED_DEFAULT=true)
target_include_directories(test_path_tracker_passthrough_on PRIVATE ${COMMON_INCLUDES})
add_test(NAME path_tracker_passthrough_on COMMAND test_path_tracker_passthrough_on)

# =========================================================================================
# Test executable #2b — PATH_PASS_THROUGH_ENABLED == false build (legacy stop-and-go).
# =========================================================================================
add_executable(test_path_tracker_passthrough_off
    ${TEST_DIR}/test_path_tracker_passthrough.cpp
    ${SUT_SOURCES}
    ${MOCK_SOURCES}
)
target_compile_definitions(test_path_tracker_passthrough_off
    PRIVATE PATH_PASS_THROUGH_ENABLED_DEFAULT=false)
target_include_directories(test_path_tracker_passthrough_off PRIVATE ${COMMON_INCLUDES})
add_test(NAME path_tracker_passthrough_off COMMAND test_path_tracker_passthrough_off)

# =========================================================================================
# Test executable #3a — BIDIRECTIONAL_AUTO_ENABLED == true build (kill-switch ON, default).
#
# Exercises TC12 (REVERSE_UNAVAILABLE fault on 1-board + REAR LINE/PROJECTION),
# TC13 (Mission modifier no-op on non-GOTO step), and TC14c (compile-time kill-switch
# override propagates into runtime behavior). See the test file's docblock for the
# rationale on why the production .cpp sources for PathTracker / Mission are NOT
# linked in here — the test mirrors the relevant slices inline, following the same
# precedent as `test_path_tracker_passthrough.cpp`.
# =========================================================================================
add_executable(test_path_tracker_bidirectional_on
    ${TEST_DIR}/test_path_tracker_bidirectional.cpp
    ${SUT_SOURCES}
    ${MOCK_SOURCES}
)
target_compile_definitions(test_path_tracker_bidirectional_on
    PRIVATE BIDIRECTIONAL_AUTO_ENABLED_DEFAULT=true)
target_include_directories(test_path_tracker_bidirectional_on PRIVATE ${COMMON_INCLUDES})
add_test(NAME path_tracker_bidirectional_on COMMAND test_path_tracker_bidirectional_on)

# =========================================================================================
# Test executable #3b — BIDIRECTIONAL_AUTO_ENABLED == false build (kill-switch OFF).
#
# Asserts that the same 175-deg fixture that picks REAR on the ON build picks FRONT
# on the OFF build — i.e. the compile-time toggle genuinely controls runtime behavior,
# not just compilation.
# =========================================================================================
add_executable(test_path_tracker_bidirectional_off
    ${TEST_DIR}/test_path_tracker_bidirectional.cpp
    ${SUT_SOURCES}
    ${MOCK_SOURCES}
)
target_compile_definitions(test_path_tracker_bidirectional_off
    PRIVATE BIDIRECTIONAL_AUTO_ENABLED_DEFAULT=false)
target_include_directories(test_path_tracker_bidirectional_off PRIVATE ${COMMON_INCLUDES})
add_test(NAME path_tracker_bidirectional_off COMMAND test_path_tracker_bidirectional_off)
