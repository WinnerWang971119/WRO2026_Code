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
# CI integration: this harness IS CI-enforced. The `host-tests` job
# (name: "Host tests") in `.github/workflows/ci.yml` runs on every push and pull
# request, using the built-in g++ on ubuntu-latest to run the three commands above:
#   cmake -S WRO2026/test -B build/test
#   cmake --build build/test
#   ctest --test-dir build/test --output-on-failure
# =========================================================================================

cmake_minimum_required(VERSION 3.16)
project(wro2026_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

# Absolute paths to source roots so the harness works regardless of build dir.
set(SKETCH_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(TEST_DIR       ${CMAKE_CURRENT_SOURCE_DIR})
set(MOCKS_DIR      ${TEST_DIR}/mocks)
# Task-body facade mocks (PathTracker / LineFollower / Manipulator) live in their
# OWN dir, NOT `mocks/`. `mocks/` is on every target's include path, so a mock
# there would shadow the REAL header for the sibling tests that compile the real
# `LineFollower.cpp` / `Manipulator.cpp`. Keeping these out of `mocks/` and
# force-including them only into `test_tasks_dispatch` avoids that collision.
set(TASK_MOCKS_DIR ${TEST_DIR}/task_mocks)

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

# -----------------------------------------------------------------------------------------
# Force-include the FULL task-body facade mock surface — the same trick as
# `add_mock_force_includes`, extended with the `PathTracker` / `LineFollower` /
# `Manipulator` mocks that a `Tasks.cpp` task body drives.
#
# Used ONLY by `test_tasks_dispatch` (NOT the LineFollower / PathTracker mirror
# tests, which compile the REAL `LineFollower.cpp` and must keep the real
# `LineFollower.h` declarations — force-including the mock there would clash with
# the production class definition the controller TU compiles against).
#
# Each mock's include guard matches the production header exactly (`PATHTRACKER_H`,
# `LINEFOLLOWER_H`, `MANIPULATOR_H`, ...), so a task body's quote-include of the
# real header (source-relative to `Tasks.cpp` in `${SKETCH_DIR}`) resolves to the
# already-force-included mock instead. The `PathTracker` / `LineFollower` /
# `Manipulator` globals are `inline`-defined in those mock headers, so no extra
# TU is linked — see each mock's global comment.
# -----------------------------------------------------------------------------------------
function(add_task_mock_force_includes target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            "/FI${MOCKS_DIR}/Drive.h"
            "/FI${MOCKS_DIR}/WROLineArray.h"
            "/FI${MOCKS_DIR}/Intersection.h"
            "/FI${TASK_MOCKS_DIR}/PathTracker.h"
            "/FI${TASK_MOCKS_DIR}/LineFollower.h"
            "/FI${TASK_MOCKS_DIR}/Manipulator.h"
        )
    else()
        # `SHELL:` keeps each `-include <path>` pair atomic — see the note in
        # `add_mock_force_includes` for why CMake de-dup would otherwise break the build.
        target_compile_options(${target} PRIVATE
            "SHELL:-include ${MOCKS_DIR}/Drive.h"
            "SHELL:-include ${MOCKS_DIR}/WROLineArray.h"
            "SHELL:-include ${MOCKS_DIR}/Intersection.h"
            "SHELL:-include ${TASK_MOCKS_DIR}/PathTracker.h"
            "SHELL:-include ${TASK_MOCKS_DIR}/LineFollower.h"
            "SHELL:-include ${TASK_MOCKS_DIR}/Manipulator.h"
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
#
# `routes.cpp` IS linked (both ON and OFF builds) so the `extern` graph-data symbols
# declared by `routes.h` (NODES / NEXT_HOP / EDGES / PIPELINE_CONFIG, pulled in
# transitively via DirectionPolicy.h / PathTracker.h / FieldMap.h) have a definition.
# It is pure const POD data with no hardware deps, so it compiles cleanly against the
# mock surface and is the sole definer of those symbols. The PathTracker / Mission
# *logic* is still mirrored inline — only the data definitions are linked in, so a
# future fixture that odr-uses `FieldMap::getNextHop/getEdge/getNode` or reads
# `PIPELINE_CONFIG` will link instead of failing on undefined references.
# =========================================================================================
add_executable(test_path_tracker_bidirectional_on
    ${TEST_DIR}/test_path_tracker_bidirectional.cpp
    ${SUT_SOURCES}
    ${MOCK_SOURCES}
    ${SKETCH_DIR}/routes.cpp
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
    ${SKETCH_DIR}/routes.cpp
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
# Test executable #4 — Tasks framework dispatch, real-linked (#46 Phase 1).
#
# Links the REAL `Tasks.cpp` and asserts its registry/dispatch behaviour through
# the capturing `MockSerial` in `mocks/Arduino.h`. Task bodies drive the facade
# singletons directly (e.g. `testOne` calls `PathTracker.goTo(...)`), so this
# target force-includes the FULL task-body facade mock surface via
# `add_task_mock_force_includes` — the `PathTracker` / `Drive` / `LineFollower` /
# `Manipulator` mocks (with `inline` globals) so Tasks.cpp links without dragging
# in the real `PathTracker.cpp` / `Drive.cpp` / Localization / routes chain.
#
# Tasks-owned behaviour only: registry lookup, `runTask` dispatch + capture, the
# unknown-name path, the `debugYield` yield path, and the `inTask()` RAII
# invariant. Mission-owned behaviour (queue FSM, abort ordering) stays out of
# scope — that needs `Mission.cpp`, which drags in EventTrace / the `MiniR4`
# global and is #46 Phase 2.
# =========================================================================================
add_executable(test_tasks_dispatch
    ${TEST_DIR}/test_tasks_dispatch.cpp
    ${SKETCH_DIR}/Tasks.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_tasks_dispatch PRIVATE ${COMMON_INCLUDES})
add_task_mock_force_includes(test_tasks_dispatch)
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

# =========================================================================================
# Test executable #6 — Manipulator facade + ManipServo + ManipBelt (T9, AC1).
#
# Links the real production `Manipulator.cpp` against the host mocks. `ManipServo.h`
# and `ManipBelt.h` are header-only (owned by Manipulator.cpp via `Manipulator.h`),
# so there is no `.cpp` for them. Exercises TC1..TC12 + TC15 (TC16 host-untriggerable —
# see the test file docblock).
#
# Manipulator's TUs pull `<MatrixMiniR4.h>` / `<Arduino.h>` via ANGLE brackets, so the
# include-path search alone picks `mocks/` (it leads `COMMON_INCLUDES`). The
# Drive / WROLineArray / Intersection mocks are NOT involved, so this target
# DELIBERATELY does NOT call `add_mock_force_includes(...)` — Manipulator never
# includes those production headers, so there is nothing to shadow.
#
# `-fsanitize=address,undefined` (compile + link) is added ONLY for GNU/Clang —
# CI runs on g++, so it gets ASan/UBSan to back TC5's no-OOB-read assertion
# (the `SORTER_BIN_ANGLE[bin]` out-of-range read is trapped). Local MSVC skips it.
# =========================================================================================
add_executable(test_manipulator
    ${TEST_DIR}/test_manipulator.cpp
    ${SKETCH_DIR}/Manipulator.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_manipulator PRIVATE ${COMMON_INCLUDES})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_manipulator PRIVATE -fsanitize=address,undefined)
    target_link_options(test_manipulator PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME manipulator COMMAND test_manipulator)

# =========================================================================================
# Test executable #7a — TC13 / AC2 negative (compile-fail target).
#
# Proves the `static_assert` in `ConfigChecks.h` fires when the belt motor ID collides
# with a drive motor. Built with `-DBELT_M_ID_DEFAULT=2` so `BELT_M_ID == MOTOR_LEFT_ID`
# (both 2). The build is expected to FAIL; ctest uses PASS_REGULAR_EXPRESSION on the
# metacharacter-free substring `collides with a drive motor` (the full assert message
# carries regex metacharacters `( ) /`). When PASS_REGULAR_EXPRESSION is set, ctest
# decides pass/fail purely on whether the regex matches the captured build output and
# IGNORES the process exit code — so the test PASSES only when the intended assert
# fires, and FAILS if the build broke for any unrelated reason.
#
# Mirrors `test_tasks_dispatch_overflow` EXACTLY: `EXCLUDE_FROM_ALL` +
# `EXCLUDE_FROM_DEFAULT_BUILD` keep it out of the default build, and NO
# `add_mock_force_includes(...)` so a typo in a mock header cannot mask the intended
# diagnostic. Include dirs = sketch + mocks (via `COMMON_INCLUDES`) so the quoted
# `"Config.h"` chain resolves and its `<Arduino.h>` picks the mock.
# =========================================================================================
add_executable(test_manipulator_belt_collision
    ${TEST_DIR}/test_manipulator_config_fail.cpp
)
set_target_properties(test_manipulator_belt_collision PROPERTIES
    EXCLUDE_FROM_ALL TRUE
    EXCLUDE_FROM_DEFAULT_BUILD TRUE
)
target_include_directories(test_manipulator_belt_collision PRIVATE ${COMMON_INCLUDES})
target_compile_definitions(test_manipulator_belt_collision PRIVATE BELT_M_ID_DEFAULT=2)

add_test(NAME manipulator_belt_collision_compile_fail
    COMMAND ${CMAKE_COMMAND} --build "${CMAKE_BINARY_DIR}"
            --target test_manipulator_belt_collision --config $<CONFIG>
)
set_tests_properties(manipulator_belt_collision_compile_fail PROPERTIES
    PASS_REGULAR_EXPRESSION "collides with a drive motor")

# =========================================================================================
# Test executable #7b — TC14 / AC2 negative (compile-fail target).
#
# Proves the `static_assert` in `ConfigChecks.h` fires when two manipulator servo RC IDs
# collide. Built with `-DSORTER_RC_ID_DEFAULT=1` so `SORTER_RC_ID == GRABBER_RC_ID`
# (both 1). PASS_REGULAR_EXPRESSION uses the metacharacter-free substring
# `pairwise distinct` (the full assert message carries no metachars there, but the
# substring is kept minimal for parity with the sibling target).
#
# Same structure as `test_manipulator_belt_collision` above (and the overflow precedent):
# EXCLUDE_FROM_ALL + EXCLUDE_FROM_DEFAULT_BUILD, no force-includes, sketch + mocks
# include dirs.
# =========================================================================================
add_executable(test_manipulator_servo_dup
    ${TEST_DIR}/test_manipulator_config_fail.cpp
)
set_target_properties(test_manipulator_servo_dup PROPERTIES
    EXCLUDE_FROM_ALL TRUE
    EXCLUDE_FROM_DEFAULT_BUILD TRUE
)
target_include_directories(test_manipulator_servo_dup PRIVATE ${COMMON_INCLUDES})
target_compile_definitions(test_manipulator_servo_dup PRIVATE SORTER_RC_ID_DEFAULT=1)

add_test(NAME manipulator_servo_dup_compile_fail
    COMMAND ${CMAKE_COMMAND} --build "${CMAKE_BINARY_DIR}"
            --target test_manipulator_servo_dup --config $<CONFIG>
)
set_tests_properties(manipulator_servo_dup_compile_fail PROPERTIES
    PASS_REGULAR_EXPRESSION "pairwise distinct")

# =========================================================================================
# Test executable #9 — Vision (HUE12) module: VisionFrame.h + Vision.cpp (T7, TC1..TC21).
#
# Links the real production `Vision.cpp` (the ONLY sketch .cpp under test —
# `VisionFrame.h` / `ConfigVision.h` / `Vision.h` are header-only / included) against the
# host mocks. `Vision.cpp` pulls `<Arduino.h>` via ANGLE brackets, so the include-path
# search alone picks `mocks/` (it leads `COMMON_INCLUDES`): the mock supplies the mock
# clock, the capturing USB `Serial`, AND the streaming `Serial1` UART mock + per-`delay()`
# tick counter (`mockTickCount`) that the integration cases drive. The `Serial1` /
# `mockTickCount` globals are defined in `mocks.cpp` (already in `${MOCK_SOURCES}`), so no
# extra source is needed.
#
# `Vision.cpp` does NOT include Drive.h / WROLineArray.h / Intersection.h (it talks to
# `Serial1` directly and never `#include <MatrixMiniR4.h>`), so this target DELIBERATELY does
# NOT call `add_mock_force_includes(...)` — there is no production header to shadow.
#
# `-fsanitize=address,undefined` (compile + link) is added ONLY for GNU/Clang — copied
# verbatim from `test_manipulator` — so the no-overrun claims (TC10 corrupt length byte,
# TC17 out-of-range getter) are machine-checked. Local MSVC skips it (the guard is false);
# the suite still BUILDS and all cases PASS under MSVC.
# =========================================================================================
add_executable(test_vision
    ${TEST_DIR}/test_vision.cpp
    ${SKETCH_DIR}/Vision.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_vision PRIVATE ${COMMON_INCLUDES})

# TC19 cell->slot direction proof. `Vision.cpp` reads `VISION_CELL_TO_SLOT` at RUNTIME inside
# readMosaic(), so the non-identity override MUST be a TARGET-WIDE compile definition (seen by
# BOTH test_vision.cpp and Vision.cpp) — a per-file `#define` would leave Vision.cpp on the
# identity map and produce a spurious mismatch. The map is the self-inverse permutation
# cell c -> slot (11 - c): every slot 0..11 referenced once, so ConfigChecks.h's
# uniqueness/range static_asserts still hold. Mirrors the `PATH_PASS_THROUGH_ENABLED_DEFAULT`
# / `BELT_M_ID_DEFAULT` compile-definition override idiom used by the sibling targets.
# The braces/commas are wrapped in CMake's `[=[ ... ]=]` bracket-argument so CMake passes the
# whole brace-init list as one definition value (quoted into the vcxproj for MSVC, escaped for
# GNU/Clang) without splitting on the commas.
target_compile_definitions(test_vision PRIVATE
    [=[VISION_CELL_TO_SLOT_DEFAULT={11,10,9,8,7,6,5,4,3,2,1,0}]=])

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_vision PRIVATE -fsanitize=address,undefined)
    target_link_options(test_vision PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME test_vision COMMAND test_vision)

# =========================================================================================
# CTPS dump #8 — `ctps_dump` (Mechanism #4, task T4).
#
# Links the REAL `Mission.cpp` + `Tasks.cpp` + `MissionScript.cpp` + `routes.cpp`
# against the recording mocks under `ctps_mocks/` (NOT the sibling `mocks/`
# LineFollower harness). Running the executable executes a mission to completion
# and serializes the captured, ordered command stream to the Data-Model-A JSON.
# `--self-test` asserts the op stream + the tick-budget hang-guard (TC8 / TC9 /
# AC4 / AC6); the JSON modes (`--fixture` / default) feed the CTPS player loader.
#
# This is an INDEPENDENT include + force-include surface from the LineFollower
# harness above: `ctps_mocks/` (not `mocks/`) must come FIRST so the recording
# `Arduino.h` / `MatrixMiniR4.h` / `Drive.h` / `PathTracker.h` / `Manipulator.h`
# / `EventTrace.h` / `LineFollower.h` shadow the production hardware-bound
# versions. `${MOCKS_DIR}` is DELIBERATELY NOT on this target's include path —
# pulling it in would let `mocks/MatrixMiniR4.h` (which has no `MiniR4`) shadow
# the recording one and break the Mission.cpp / EventTrace compile.
# =========================================================================================
set(CTPS_MOCKS_DIR ${TEST_DIR}/ctps_mocks)

# -----------------------------------------------------------------------------------------
# Force-include the recording mock headers BEFORE any source-file parsing — the same
# load-bearing trick as `add_mock_force_includes`, but pointed at `ctps_mocks/` and using
# ABSOLUTE `${CTPS_MOCKS_DIR}/` paths.
#
# Why absolute, not bare names: the REAL `Drive.h` / `PathTracker.h` / `EventTrace.h` /
# `LineFollower.h` sit next to `Mission.cpp` in `${SKETCH_DIR}`, so a bare-name
# force-include (`/FIDrive.h`) would resolve source-relative to the REAL header. Each
# mock's include guard matches the production guard exactly (`DRIVE_H`, `PATHTRACKER_H`,
# `EVENTTRACE_H`, `LINEFOLLOWER_H`), so once force-included the real header no-ops when
# re-requested transitively.
#
# `MatrixMiniR4.h` / `Arduino.h` are angle-bracket includes inside the sketch headers, so
# the `${CTPS_MOCKS_DIR}`-first include order alone picks the recording versions — no
# force-include needed for those two.
# -----------------------------------------------------------------------------------------
function(add_ctps_mock_force_includes target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            "/FI${CTPS_MOCKS_DIR}/Drive.h"
            "/FI${CTPS_MOCKS_DIR}/PathTracker.h"
            "/FI${CTPS_MOCKS_DIR}/EventTrace.h"
            "/FI${CTPS_MOCKS_DIR}/LineFollower.h"
            "/FI${CTPS_MOCKS_DIR}/Localization.h"
            "/FI${CTPS_MOCKS_DIR}/Manipulator.h"
        )
    else()
        # `SHELL:` keeps each `-include <path>` pair atomic — see the note in
        # `add_mock_force_includes` for why CMake de-dup would otherwise break the build.
        target_compile_options(${target} PRIVATE
            "SHELL:-include ${CTPS_MOCKS_DIR}/Drive.h"
            "SHELL:-include ${CTPS_MOCKS_DIR}/PathTracker.h"
            "SHELL:-include ${CTPS_MOCKS_DIR}/EventTrace.h"
            "SHELL:-include ${CTPS_MOCKS_DIR}/LineFollower.h"
            "SHELL:-include ${CTPS_MOCKS_DIR}/Localization.h"
            "SHELL:-include ${CTPS_MOCKS_DIR}/Manipulator.h"
        )
    endif()
endfunction()

add_executable(ctps_dump
    ${TEST_DIR}/ctps_dump.cpp
    ${SKETCH_DIR}/Mission.cpp
    ${SKETCH_DIR}/Tasks.cpp
    ${SKETCH_DIR}/MissionScript.cpp
    ${SKETCH_DIR}/routes.cpp
    ${CTPS_MOCKS_DIR}/ctps_record.cpp
)
# `${CTPS_MOCKS_DIR}` FIRST; `${MOCKS_DIR}` intentionally omitted (see the block comment).
target_include_directories(ctps_dump PRIVATE ${CTPS_MOCKS_DIR} ${SKETCH_DIR} ${TEST_DIR})
target_compile_definitions(ctps_dump PRIVATE CTPS_RECORD)
add_ctps_mock_force_includes(ctps_dump)
add_test(NAME ctps_dump_selftest COMMAND ctps_dump --self-test)
set_tests_properties(ctps_dump_selftest PROPERTIES
    PASS_REGULAR_EXPRESSION "CTPS DUMP SELFTEST OK")

# =========================================================================================
# Test executable #10 — Localization operator-offset integrator (PR1a, TC1/TC2/TC2b/TC3).
#
# FIRST host target to compile the real production `Localization.cpp`. It drives the
# per-tick offset integrator (`Localization::update()`) via the mock `Drive` encoders
# and the mock lower-MCU IMU (`mmL.GetIMUEuler`, added additively to mocks/MatrixMiniR4.h).
#
# `Localization.cpp` includes `"Drive.h"` (quote, source-relative to the sketch dir), so
# `add_mock_force_includes` force-includes the mock `Drive.h` (guard `DRIVE_H`) to shadow
# the real one — same mechanism as the PathTracker mirror targets. `<MatrixMiniR4.h>` /
# `<Arduino.h>` are angle-bracket includes resolved by the `mocks/`-first include path,
# so the mock IMU surface (`mmL`, `MMLower::RESULT`) is picked without a force-include.
#
# `routes.cpp` IS linked: `Localization.cpp` odr-uses `Routes::START_NODE`,
# `START_HEADING_DEG`, `NODES`, and `N_NODES` (pulled in via routes.h / FieldMap.h), so
# the const POD graph data needs a definer — routes.cpp is the sole definer and compiles
# cleanly against the mock surface (no hardware deps). `MOCK_SOURCES` supplies the
# `Drive` / `MiniR4` / `Serial` / `g_mockMillis` globals (the `mmL` global is an inline
# variable in the mock header, so no extra source is linked for it).
# =========================================================================================
add_executable(test_localization_offset
    ${TEST_DIR}/test_localization_offset.cpp
    ${SKETCH_DIR}/Localization.cpp
    ${SKETCH_DIR}/routes.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_localization_offset PRIVATE ${COMMON_INCLUDES})
add_mock_force_includes(test_localization_offset)   # shadows the real Drive.h with the mock
add_test(NAME localization_offset COMMAND test_localization_offset)

# =========================================================================================
# Test executable #9 — PivotTrimPolicy pure-math (ADR-036, #11 promotion).
#
# `resolvePivotTrim` is a header-only inline policy (`PivotTrimPolicy.h`) promoted
# out of PathTracker.cpp so production and this host test link the SAME subtract-
# and-clamp symbol, mirroring `DirectionPolicy.h` / `StopAtLinePolicy.h` (PR #34).
# The host harness does NOT compile `PathTracker.cpp`, so before this promotion the
# trim formula had no host coverage at all. The policy is pure float math with no
# Arduino surface, so this target needs no mocks / SUT sources — just the sketch
# include dir for the header.
# =========================================================================================
add_executable(test_pivot_trim_policy
    ${TEST_DIR}/test_pivot_trim_policy.cpp
)
target_include_directories(test_pivot_trim_policy PRIVATE ${SKETCH_DIR})
add_test(NAME pivot_trim_policy COMMAND test_pivot_trim_policy)

# =========================================================================================
# Test executable #9b — SensorOvershootPolicy pure-math (issue #65, ADR-043 D1).
#
# `resolveSensorOvershootTrim` is a header-only inline policy (`PivotTrimPolicy.h`) —
# the SENSOR-anchored twin of `resolvePivotTrim`. It feeds the post-detect axle roll
# through the shared subtract-and-clamp with departing trim hard-coded to 0 (ADR-043 D1:
# sensor-terminated hops self-correct on the line, so outgoing over-travel does not
# accumulate). TC1-TC4 from the issue #65 plan cover the normal, over-trim, no-trim, and
# strict-boundary cases; together with `test_pivot_trim_policy` they ensure the DISTANCE
# and SENSOR paths cannot drift from each other on host.
#
# Pure float math with no Arduino surface: no mocks, no SUT sources — just the sketch
# include dir for the header. Mirrors the `test_pivot_trim_policy` block exactly.
# =========================================================================================
add_executable(test_sensor_overshoot_policy
    ${TEST_DIR}/test_sensor_overshoot_policy.cpp
)
target_include_directories(test_sensor_overshoot_policy PRIVATE ${SKETCH_DIR})
add_test(NAME sensor_overshoot_policy COMMAND test_sensor_overshoot_policy)

# =========================================================================================
# Test executable #11 — ReturnToNodePolicy pure-math (issue #36, PR1b, T8/T10).
#
# `resolveReturnAction` is a header-only inline policy (`ReturnToNodePolicy.h`) — the
# return-to-node GATE decision (trigger / cap / off-map), promoted out of PathTracker.cpp
# so production (`PathTracker::_evaluateReturnGate`) and this host test link the SAME symbol,
# mirroring `PivotTrimPolicy.h` / `DirectionPolicy.h` / `StopAtLinePolicy.h`. It is the TC4 /
# TC5 / TC6 / TC7 coverage layer (RETURN trigger, below-trigger NONE, over-cap FAULT, over-cap
# BEST_EFFORT) plus the exact trigger-closed / cap-open boundaries.
#
# Pure enum/float logic with no Drive / Localization runtime surface and no SUT `.cpp`.
# Unlike `test_pivot_trim_policy` (whose `PivotTrimPolicy.h` is self-contained),
# `ReturnToNodePolicy.h` includes `Config.h` for the `Config::OffMapGate` enum, and
# `Config.h` pulls in `<Arduino.h>` — so this target needs the `mocks/`-first
# `COMMON_INCLUDES` to resolve that angle-bracket header to `mocks/Arduino.h`. It still
# links NO `MOCK_SOURCES`: the policy + Config are header-only and the test never odr-uses
# a mock global (no `Serial` / `millis()` call), so there is nothing to link. No
# force-include needed (the policy never includes the production `Drive.h`).
# =========================================================================================
add_executable(test_return_to_node_policy
    ${TEST_DIR}/test_return_to_node_policy.cpp
)
target_include_directories(test_return_to_node_policy PRIVATE ${COMMON_INCLUDES})
add_test(NAME return_to_node_policy COMMAND test_return_to_node_policy)

# =========================================================================================
# Test executable #12 — return-to-node NODE-KIND dispatch slice (issue #36, PR1b, T8/T10).
#
# Two layers (see the test file docblock):
#   - TC4 / TC4c: the `_returnUsesLineSeek` decision (LINE-vs-BLIND node kind AND the
#     return-distance vs axle-lead gate, #14) is tested DIRECTLY against the REAL
#     `FieldMap::getNodeType` over the REAL `routes.cpp` graph data and the REAL
#     `Config::LINE_SENSOR_AXLE_OFFSET_CM` (the toy fixture carries both kinds) — not a tautology.
#   - TC4b: the per-kind RETURN_TO_NODE state-body routing is inline-MIRRORED (annotated
#     `// MIRROR: PathTracker.cpp:<line>`), following the precedent of
#     `test_path_tracker_stop_at_line` — the full `PathTracker.cpp` is NOT linked by ANY host
#     target. The full FSM transition + the Drive builder chain + axle-on-node geometry are
#     delegated to the Python sim parity test (T11) and the firmware compile.
#
# Links `routes.cpp` (the sole definer of the `Routes::NODES` / `N_NODES` data that
# `FieldMap::getNodeType` odr-uses) + `MOCK_SOURCES` (for the `Serial` / `MiniR4` globals the
# mock `<Arduino.h>` / `<MatrixMiniR4.h>` — pulled in transitively by `PathTracker.h` /
# `FieldMap.h` — declare `extern`). The `mocks/`-first `COMMON_INCLUDES` resolves those two
# angle-bracket headers. NO Drive force-include: none of the headers this TU includes
# (FieldMap.h / routes.h / PathTracker.h) include the production `Drive.h`, so there is
# nothing to shadow (the `PathTracker.cpp` that would is never linked).
# =========================================================================================
add_executable(test_return_to_node_dispatch
    ${TEST_DIR}/test_return_to_node_dispatch.cpp
    ${SKETCH_DIR}/routes.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_return_to_node_dispatch PRIVATE ${COMMON_INCLUDES})
add_test(NAME return_to_node_dispatch COMMAND test_return_to_node_dispatch)
