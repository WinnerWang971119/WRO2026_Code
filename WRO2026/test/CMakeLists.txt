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
#
# **Applied PER TARGET, not directory-wide**, so the compile-fail target
# (`test_tasks_dispatch_overflow`, PASS_REGULAR_EXPRESSION on the static_assert
# diagnostic) does NOT inherit them. Directory-wide force-includes would weaken
# AC9-negative: any unrelated typo in a mock header would make the overflow build
# fail without printing the `MAX_TASK_ARG_BYTES` message, so the target would no
# longer isolate the intended static_assert. See PR #35 review
# finding S1 / C8.
# -----------------------------------------------------------------------------------------
function(add_mock_force_includes target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            "/FI${MOCKS_DIR}/Drive.h"
            "/FI${MOCKS_DIR}/WROLineArray.h"
            "/FI${MOCKS_DIR}/Intersection.h"
        )
    else()
        # Use the `SHELL:` prefix so CMake keeps each `-include <path>` pair atomic. Without it,
        # CMake de-duplicates the repeated `-include` flag, collapsing the three to one and
        # leaving the paths as bare arguments -- g++ then treats them as extra source files and
        # fails with "cannot specify '-o' with '-c' ... with multiple files".
        target_compile_options(${target} PRIVATE
            "SHELL:-include ${MOCKS_DIR}/Drive.h"
            "SHELL:-include ${MOCKS_DIR}/WROLineArray.h"
            "SHELL:-include ${MOCKS_DIR}/Intersection.h"
        )
    endif()
endfunction()

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
add_mock_force_includes(test_line_follower_overshoot)
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
add_mock_force_includes(test_path_tracker_passthrough_on)
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
add_mock_force_includes(test_path_tracker_passthrough_off)
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
add_mock_force_includes(test_path_tracker_bidirectional_on)
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
add_mock_force_includes(test_path_tracker_bidirectional_off)
add_test(NAME path_tracker_bidirectional_off COMMAND test_path_tracker_bidirectional_off)

# =========================================================================================
# Test executable #3c — PathTracker STOP_AT_LINE mirror-slice (TC7..TC10, T16).
#
# Exercises the STOP_AT_LINE FSM slice introduced by the 2026-05-30 stop-at-line
# plan: re-anchor-on-success (TC7), DRIVE_FAULT-on-no-line (TC8), and the
# call-flag OFF/ON precedence over the per-edge BlindMode (TC9/TC10).
#
# Rationale for NOT linking the real `PathTracker.cpp` and NOT mocking
# `Drive.stopAtLine` (mirrors the precedent set by
# `test_path_tracker_passthrough.cpp` and `test_path_tracker_bidirectional.cpp`):
# the test inline-mirrors the small STOP_AT_LINE decision slice rather than drag
# in the full Drive-builder / Localization / FieldMap / routes.cpp / MatrixMiniR4
# / EventTrace surface. Mirror sites are annotated `// MIRROR: PathTracker.cpp:<line>`
# for drift detection at PR-review time. The mock `Drive.h` is deliberately NOT
# extended with a fault surface — the production `Drive.hasFault()` / `isIdle()`
# observations are modelled as plain `bool` inputs to the replicated tick.
#
# Links `SUT_SOURCES` for parity with the sibling mirror targets (the test does
# not touch LineFollower, but the force-included Drive/LineArray/Intersection
# mocks resolve against the same TU set).
# =========================================================================================
add_executable(test_path_tracker_stop_at_line
    ${TEST_DIR}/test_path_tracker_stop_at_line.cpp
    ${SUT_SOURCES}
    ${MOCK_SOURCES}
)
target_include_directories(test_path_tracker_stop_at_line PRIVATE ${COMMON_INCLUDES})
add_mock_force_includes(test_path_tracker_stop_at_line)
add_test(NAME path_tracker_stop_at_line COMMAND test_path_tracker_stop_at_line)

# =========================================================================================
# Test executable #3d — stopAtLine arm-window + debounce pure-math (TC11, T16).
#
# Tests the pure `WROMath.h` free functions `stopAtLineArmed` and `debounceStep`
# directly — the only host-testable part of the stateful `Drive._updateStopAtLine`
# seek (the arm-window DECISION of AC2 and the N-of-N detection debounce). The
# stateful seek (PWM overshoot ramp + encoder roll + active brake) is bench-verified
# on hardware (B3.7), not in CI.
#
# Minimal TU set: the pure functions touch no `Drive` / `LineArray` / `Intersection`
# surface, so this target does NOT force-include those mocks. It links `MOCK_SOURCES`
# only for the `Serial` / `g_mockMillis` definitions the mock `Arduino.h` declares
# `extern` (pulled in transitively by `WROMath.h`'s `<Arduino.h>` include).
# =========================================================================================
add_executable(test_stop_at_line_armmath
    ${TEST_DIR}/test_stop_at_line_armmath.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_stop_at_line_armmath PRIVATE ${COMMON_INCLUDES})
add_test(NAME stop_at_line_armmath COMMAND test_stop_at_line_armmath)

# =========================================================================================
# Test executable #4 — Tasks framework dispatch (T9 of the Tasks plan).
#
# Exercises TC1..TC3, TC5..TC12 of the Tasks-framework plan
# (`docs/plans/2026-05-26-tasks-framework-mission-task-verb.md`). Satisfies AC10
# and all unit/integration ACs (AC2..AC9 positive). The compile-fail target
# below covers TC4 / AC9 negative.
#
# Rationale for NOT linking the real `Tasks.cpp` / `Mission.cpp` (mirrors the
# precedent established by `test_path_tracker_passthrough.cpp` and
# `test_path_tracker_bidirectional.cpp`): the test inline-mirrors the slice of
# Mission's TASK-case FSM + TasksClass's registry + dispatch + RAII guard so
# the assertions don't need to mock the full PathTracker / Drive / LineFollower
# / MatrixMiniR4 / EventTrace surface. Mirror sites are annotated
# `// MIRROR: <path>:<line>` for drift detection at PR-review time. Real
# production POD types (`TaskArgs`, `TaskRegistryEntry`) are pulled in from
# `WRO2026/TasksTypes.h` so the union-layout / size invariants are bound to
# the real definitions at compile time.
# =========================================================================================
add_executable(test_tasks_dispatch
    ${TEST_DIR}/test_tasks_dispatch.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_tasks_dispatch PRIVATE ${COMMON_INCLUDES})
add_mock_force_includes(test_tasks_dispatch)
add_test(NAME test_tasks_dispatch COMMAND test_tasks_dispatch)

# =========================================================================================
# Test executable #5 — TC4 / AC9 negative (compile-fail target).
#
# Proves the `static_assert` in `MissionClass::task<Args...>` fires when the
# variadic argument pack exceeds `Config::MAX_TASK_ARG_BYTES`. The build is
# expected to FAIL; ctest uses
# `PASS_REGULAR_EXPRESSION "Mission.task args exceed MAX_TASK_ARG_BYTES"`
# to require that the build output contains the intended static_assert
# diagnostic. The regex anchors on the FULL assert message, not the bare
# `MAX_TASK_ARG_BYTES` token — TasksTypes.h carries its own layout
# static_assert that also mentions that token, so a TasksTypes.h / Config.h
# break could surface the bare token and false-PASS; the full-message anchor
# matches only the intended assert. When PASS_REGULAR_EXPRESSION is set, ctest
# decides pass/fail purely on whether the regex matches the captured output and
# IGNORES the process exit code — so the test PASSES only when the build prints
# the full `Mission.task args exceed MAX_TASK_ARG_BYTES` message, and FAILS if
# the build broke for any unrelated reason (toolchain break, stale generator,
# an unrelated TasksTypes.h / Config.h edit). This distinguishes "the intended
# static_assert fired" from "the toolchain broke" — which a bare WILL_FAIL,
# passing on ANY non-zero exit, could not.
#
# `EXCLUDE_FROM_ALL` + `EXCLUDE_FROM_DEFAULT_BUILD` keep the target out of the
# default `cmake --build .` so a regular build does not break. The ctest
# command below invokes the per-target build on demand.
# =========================================================================================
add_executable(test_tasks_dispatch_overflow
    ${TEST_DIR}/test_tasks_dispatch_overflow.cpp
)
set_target_properties(test_tasks_dispatch_overflow PROPERTIES
    EXCLUDE_FROM_ALL TRUE
    EXCLUDE_FROM_DEFAULT_BUILD TRUE
)
target_include_directories(test_tasks_dispatch_overflow PRIVATE ${COMMON_INCLUDES})

# DELIBERATELY no `add_mock_force_includes(...)` call here. A typo in any
# force-included mock header would make this build fail for the wrong reason —
# without printing the `MAX_TASK_ARG_BYTES` diagnostic — and PASS_REGULAR_EXPRESSION
# would then correctly FAIL the test, but more importantly the target would no
# longer isolate AC9-negative to the intended static_assert. The overflow TU only
# references TasksTypes.h + the synthetic `Seventeen` struct + the inline
# `check_overflow` template, so it does not need Drive/WROLineArray/Intersection
# mocks at all. The only path that can produce the `static_assert("Mission.task
# args exceed MAX_TASK_ARG_BYTES")` diagnostic is the assert itself — which is
# precisely what AC9-negative wants.

add_test(NAME tasks_overflow_compile_fail
    COMMAND ${CMAKE_COMMAND} --build "${CMAKE_BINARY_DIR}"
            --target test_tasks_dispatch_overflow
)
set_tests_properties(tasks_overflow_compile_fail PROPERTIES
    PASS_REGULAR_EXPRESSION "Mission.task args exceed MAX_TASK_ARG_BYTES")
