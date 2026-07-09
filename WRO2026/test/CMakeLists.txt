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
# Shared host-test HAL mock dir (issue #105). Hoisted here from its historical
# definition near `test_mock_hal` because the issue-#106 PathTracker+Drive
# real-link targets below ALSO consume it: their `add_path_force_includes` helper
# force-includes `${HAL_MOCKS_DIR}/EventTrace.h`, and their include list leads with
# `${HAL_MOCKS_DIR}` so the angle-bracket `<MatrixMiniR4.h>` / `<Arduino.h>` resolve
# to the HAL mock. A `set()` must execute before any line that expands the variable,
# and those targets sit ABOVE the old definition site — so the dir var lives here now.
set(HAL_MOCKS_DIR  ${TEST_DIR}/hal_mocks)
# Issue #106 PathTracker + Drive host-link thin-peripheral mocks. Holds the
# record-and-test-set `WROLineArray.h` / `Intersection.h` / `LineFollower.h` the real
# `Drive.cpp` + `PathTracker.cpp` command, plus `path_mocks.cpp` (the facade-globals TU
# these targets link — it defines `MiniR4` / `Serial` / `Serial1` / `g_mockMillis` /
# `mockTickCount` / `EventTrace` but DELIBERATELY NOT `Drive`, so it never collides with
# the real `Drive.cpp` that is the sole definer of the `Drive` global).
set(PATH_MOCKS_DIR ${TEST_DIR}/path_mocks)
# Task-body facade mocks (PathTracker / LineFollower / Manipulator) live in their
# OWN dir, NOT `mocks/`. `mocks/` is on every target's include path, so a mock
# there would shadow the REAL header for the sibling tests that compile the real
# `LineFollower.cpp` / `Manipulator.cpp`. Keeping these out of `mocks/` and
# force-including them only into `test_tasks_dispatch` avoids that collision.
set(TASK_MOCKS_DIR ${TEST_DIR}/task_mocks)

# Toy-field golden routes (issue #64 / TC16). The PRODUCTION `routes.cpp` has NO
# projection node — issue #64 keeps the real field unchanged this phase and only
# the TOY field gains one — so the projection-dispatch host test
# (`test_path_tracker_projection`) links the TOY golden instead. The toy
# `routes_toy.h` shares the production `ROUTES_H` include guard and the identical
# `Routes::` ABI, so force-including it ahead of the sketch `routes.h` shadows the
# production data for that ONE target without disturbing every other target (which
# keeps linking the real `routes.cpp`).
set(TOY_ROUTES_DIR ${SKETCH_DIR}/../tools/path_planner/tests/golden)

# Issue #91 host-test fixture routes. The FSM-logic tests below
# (`return_to_node_dispatch`, `reanchor_at_wall`[`_strict`],
# `path_tracker_reset_arrival`, and the `ctps_dump` self-test) assert
# PathTracker / Localization LOGIC that is field-INDEPENDENT, but resolve LINE /
# BLIND / RESET("Wall1") nodes BY NAME. The production `routes.cpp` carries none of
# those once the real field collapses to a single spawn node (#64), so these
# targets link this dedicated, STABLE 7-node fixture instead of
# `${SKETCH_DIR}/src/nav/routes.cpp`. `routes_fixture.h` shares the production `ROUTES_H`
# guard, so force-including it ahead of the sketch `routes.h` shadows the
# production data for those targets only. The fixture is frozen and is NOT part of
# the CI routes-freshness gate; regenerate it only on a `routes.h` schema change
# with `WRO2026/test/fixtures/regenerate_routes_fixture.py`.
set(FIXTURE_ROUTES_DIR ${TEST_DIR}/fixtures)

# -----------------------------------------------------------------------------------------
# Production sources under test — compiled UNCHANGED.
#   - LineFollower.cpp pulls in Drive.h / WROLineArray.h / Intersection.h via the include
#     path, all of which resolve to mocks because `mocks/` comes first.
# -----------------------------------------------------------------------------------------
set(SUT_SOURCES
    ${SKETCH_DIR}/src/drive/LineFollower.cpp
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

# -----------------------------------------------------------------------------------------
# Force-include the TOY golden `routes_toy.h` BEFORE any source-file parsing — the same
# load-bearing shadow trick as `add_mock_force_includes`, pointed at the toy routes header.
#
# Why: the projection-dispatch test needs a projection NODE, which lives ONLY in the toy
# field (the production `routes.cpp` has none this phase). `routes_toy.h` carries the
# production `ROUTES_H` include guard, so once force-included it shadows the sketch
# `routes.h` everywhere it is transitively requested (FieldMap.h / PathTracker.h /
# StopAtLinePolicy.h) — AND in `routes_toy.cpp` itself, whose `#include "routes.h"` then
# no-ops so the 8-node data is sized against the toy `N_NODES`, not the sketch's 7.
#
# Applied PER TARGET (only `test_path_tracker_projection`), so no sibling target that
# links the real `routes.cpp` inherits the toy shadow. An ABSOLUTE path is used (not a
# bare name) because the real `routes.h` sits next to the test's other sketch includes, so
# a bare-name force-include could resolve to the wrong header.
# -----------------------------------------------------------------------------------------
function(add_toy_routes_force_include target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            "/FI${TOY_ROUTES_DIR}/routes_toy.h"
        )
    else()
        # `SHELL:` keeps the `-include <path>` pair atomic — see `add_mock_force_includes`.
        target_compile_options(${target} PRIVATE
            "SHELL:-include ${TOY_ROUTES_DIR}/routes_toy.h"
        )
    endif()
endfunction()

# -----------------------------------------------------------------------------------------
# Force-include the issue #91 fixture `routes_fixture.h` BEFORE any source-file parsing —
# the same shadow trick as `add_toy_routes_force_include`, pointed at the host-test fixture
# routes. `routes_fixture.h` carries the production `ROUTES_H` guard, so once force-included
# it shadows the sketch `routes.h` everywhere it is transitively requested (FieldMap.h /
# PathTracker.h / Localization.h) — AND in `routes_fixture.cpp` itself, whose
# `#include "routes.h"` then no-ops so the 7-node fixture data is sized against the fixture
# `N_NODES`. Applied PER TARGET (only the field-independent FSM-logic tests), so no sibling
# target that links the real `routes.cpp` inherits the fixture shadow. An ABSOLUTE path is
# used (not a bare name) because the real `routes.h` sits next to the test's sketch includes.
# -----------------------------------------------------------------------------------------
function(add_fixture_routes_force_include target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            "/FI${FIXTURE_ROUTES_DIR}/routes_fixture.h"
        )
    else()
        # `SHELL:` keeps the `-include <path>` pair atomic — see `add_mock_force_includes`.
        target_compile_options(${target} PRIVATE
            "SHELL:-include ${FIXTURE_ROUTES_DIR}/routes_fixture.h"
        )
    endif()
endfunction()

# -----------------------------------------------------------------------------------------
# Force-include the ADR-052 / T6 forced-reset routes table
# (`fixtures/routes_reset_passthrough_forced.h`) BEFORE any source-file parsing — the same
# shadow trick as `add_fixture_routes_force_include`, pointed at a DELIBERATELY invalid,
# hand-written 3-node table (see that header's docblock for why: TC11 / AC9 needs a LINE edge
# incident to a RESET node, a topology ADR-048 guarantees the real
# `field_data.json` -> validator -> graph_builder -> build_routes.py` pipeline can never emit).
# NOT single-sourced from `field_fixture.py`, NOT governed by ADR-050. Used by exactly ONE
# target: `test_path_tracker_heading_reset_forced_reset`.
# -----------------------------------------------------------------------------------------
function(add_forced_reset_routes_force_include target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            "/FI${FIXTURE_ROUTES_DIR}/routes_reset_passthrough_forced.h"
        )
    else()
        # `SHELL:` keeps the `-include <path>` pair atomic — see `add_mock_force_includes`.
        target_compile_options(${target} PRIVATE
            "SHELL:-include ${FIXTURE_ROUTES_DIR}/routes_reset_passthrough_forced.h"
        )
    endif()
endfunction()

# -----------------------------------------------------------------------------------------
# Force-include the issue-#106 PathTracker+Drive real-link peripheral mocks BEFORE any
# source-file parsing — the same load-bearing shadow trick as `add_hal_force_includes` /
# `add_mission_mock_force_includes`, pointed at `hal_mocks/EventTrace.h` + the three
# `path_mocks/` peripherals, using ABSOLUTE paths.
#
# THE INVERSION (vs `test_mission_dispatch` / #111 — read this before copying that recipe):
# these targets host-link the REAL `Drive.cpp` + `PathTracker.cpp`, so the PRODUCTION
# `Drive.h` (guard `DRIVE_H`) and `PathTracker.h` (guard `PATHTRACKER_H`) MUST be the
# definitions that compile. We therefore do **NOT** force-include `hal_mocks/Drive.h` (that
# would swap in the mock `DriveClass` and the real `Drive.cpp` would fail to find its own
# private members + multiply-type `Drive`), and there is **NO** PathTracker mock at all.
# Likewise NO `Localization.h` / `YawSource.h` force-include — the real headers win, paired
# with the real `Localization.cpp` / `YawSource.cpp` these targets link. The trust boundary
# sits one layer DOWN: the `MatrixMiniR4` facade + the line-sensor leaf are mocked, and real
# Drive/PathTracker/Localization/YawSource win above it.
#
# What IS force-included (in this order):
#   - `${HAL_MOCKS_DIR}/EventTrace.h` (guard `EVENTTRACE_H`) — the no-op EventTrace mock.
#     Real `PathTracker.cpp` only calls `EventTrace.pushEvent(...)`, which the no-op mock
#     satisfies; no real `EventTrace.cpp` is linked (so the `MiniR4OLED.h WHITE`-macro
#     concern never arises — satisfied-by-avoidance, same as #111).
#   - `${PATH_MOCKS_DIR}/WROLineArray.h` (guard `WROLINEARRAY_H`) — real `Drive.cpp` calls
#     `getJunctionType()` / `setReverseActive(bool)` on it; the `Intersection` / `LineFollower`
#     mocks reference its `JunctionType` enum.
#   - `${PATH_MOCKS_DIR}/Intersection.h`  (guard `INTERSECTION_H`).
#   - `${PATH_MOCKS_DIR}/LineFollower.h`  (guard `LINEFOLLOWER_H`).
#
# NO `MatrixMiniR4.h` force-include: `<MatrixMiniR4.h>` / `<Arduino.h>` are ANGLE-bracket
# includes with NO counterpart in `${SKETCH_DIR}` (the sketch never ships them), so the
# `${HAL_MOCKS_DIR}`-on-the-include-path order alone picks the HAL mock — exactly like the
# sibling force-include helpers. Each mock's guard matches the production header's guard, so
# once force-included the real header no-ops when re-requested transitively (FieldMap.h /
# PathTracker.h / Drive.h …) — the SKETCH_DIR-FIRST include order does NOT un-shadow them.
#
# Composed AFTER `add_fixture_routes_force_include(target)` (which must win FIRST so its
# `ROUTES_H` is defined before any header pulls production `routes.h`), exactly like #111.
# -----------------------------------------------------------------------------------------
function(add_path_force_includes target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            "/FI${HAL_MOCKS_DIR}/EventTrace.h"
            "/FI${PATH_MOCKS_DIR}/WROLineArray.h"
            "/FI${PATH_MOCKS_DIR}/Intersection.h"
            "/FI${PATH_MOCKS_DIR}/LineFollower.h"
        )
    else()
        # `SHELL:` keeps each `-include <path>` pair atomic — see the note in
        # `add_mock_force_includes` for why CMake de-dup would otherwise collapse the four to
        # one and break the build (`cannot specify -o with -c ... multiple files`).
        target_compile_options(${target} PRIVATE
            "SHELL:-include ${HAL_MOCKS_DIR}/EventTrace.h"
            "SHELL:-include ${PATH_MOCKS_DIR}/WROLineArray.h"
            "SHELL:-include ${PATH_MOCKS_DIR}/Intersection.h"
            "SHELL:-include ${PATH_MOCKS_DIR}/LineFollower.h"
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
# Test executables #2/#3 — the FIVE PathTracker mirror tests, re-wired to host-link the
# REAL `PathTracker.cpp` + `Drive.cpp` (issue #106, closing the last gap in ADR-030).
#
# THE SHARED RECIPE (all seven `add_executable` blocks below — the passthrough + bidirectional
# groups are doubled for their ON/OFF compile flag — use it identically). It INVERTS the #111
# `test_mission_dispatch` wiring on two load-bearing points:
#
#   1. LINK the REAL sources, not mocks: `PathTracker.cpp` + `Drive.cpp` + `Localization.cpp` +
#      `YawSource.cpp` + the feature-complete `fixtures/routes_fixture.cpp` (all five resolve
#      nodes BY NAME) + `path_mocks/path_mocks.cpp` — the facade-globals TU. We link
#      `path_mocks.cpp` and **NOT** `hal_mocks/hal_mocks.cpp`: `hal_mocks.cpp` defines a MOCK
#      `DriveClass Drive`, which alongside the real `Drive.cpp` (the sole real definer of
#      `Drive`) would multiply-define `Drive` (and `MiniR4` / `EventTrace` / `Serial` /
#      `g_mockMillis`, which `path_mocks.cpp` already provides). `path_mocks.cpp` defines the
#      facade globals WITHOUT a mock `Drive`. NEVER link `hal_mocks.cpp` or `mocks/mocks.cpp`
#      into these five targets.
#
#   2. Include order leads with `${SKETCH_DIR}`, NOT `${HAL_MOCKS_DIR}`. The harness
#      (`test_path_tracker_harness.h`) and the test `.cpp`s quote-include `"Drive.h"` /
#      `"PathTracker.h"` / `"Localization.h"` / `"YawSource.h"` / `"FieldMap.h"` / `"Config.h"`
#      with no source-relative match in `${TEST_DIR}`, so they resolve via the `-I` order.
#      `${HAL_MOCKS_DIR}` CONTAINS a `Drive.h` (the MOCK `DriveClass`) — if it led, `"Drive.h"`
#      would resolve to the mock (missing `isTurnDone()` / `isLineFound()` / `getFault()` the
#      harness calls, AND an ODR clash with real `Drive.cpp`). `${SKETCH_DIR}`-FIRST makes the
#      real sketch headers win — THE INVERSION holds. The mocks still win exactly where intended:
#        - `<Arduino.h>` / `<MatrixMiniR4.h>` are ANGLE includes with no `${SKETCH_DIR}`
#          counterpart, so they fall through to `${HAL_MOCKS_DIR}` → the HAL mock.
#        - `WROLineArray.h` / `Intersection.h` / `LineFollower.h` / `EventTrace.h` DO have
#          `${SKETCH_DIR}` counterparts, but `add_path_force_includes` force-includes the
#          `path_mocks/` / `hal_mocks/` versions by ABSOLUTE path FIRST, so their guards are
#          already defined → the real sketch headers no-op (SKETCH_DIR-first does not un-shadow).
#        - `routes.h` is shadowed by the force-included `routes_fixture.h` (guard `ROUTES_H`).
#      `${MOCKS_DIR}` is OMITTED entirely: `mocks/Drive.h` and `hal_mocks/Drive.h` share the
#      `DRIVE_H` guard (ODR mutual-exclusion), so no target may see both dirs.
#
# Force-include ORDER (critical): `add_fixture_routes_force_include` FIRST (its `ROUTES_H`
# must win before any header pulls production `routes.h`, else "too many initializers"), THEN
# `add_path_force_includes` (EventTrace + the three `path_mocks/` peripherals). NO Drive.h, NO
# PathTracker.h, NO MatrixMiniR4.h, NO Localization.h/YawSource.h force-include — see the
# `add_path_force_includes` block comment above for why each is omitted.
#
# The targets inherit the directory-wide warning suppressions (`add_compile_options` near the
# top) just like the LineFollower harness, and preserve their ctest names + ON/OFF flag defs
# (CI greps the names; the flags prove the compile-time toggle drives runtime behaviour).
# =========================================================================================

# --- Test executable #2a — PATH_PASS_THROUGH_ENABLED == true build. ---
add_executable(test_path_tracker_passthrough_on
    ${TEST_DIR}/test_path_tracker_passthrough.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_compile_definitions(test_path_tracker_passthrough_on
    PRIVATE PATH_PASS_THROUGH_ENABLED_DEFAULT=true)
# SKETCH_DIR FIRST (the inversion); hal_mocks/ + path_mocks/ supply the angle-bracket vendor
# headers + the force-included peripheral mocks; mocks/ OMITTED (ODR mutual-exclusion).
target_include_directories(test_path_tracker_passthrough_on PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_passthrough_on)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_passthrough_on)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_passthrough_on COMMAND test_path_tracker_passthrough_on)

# --- Test executable #2b — PATH_PASS_THROUGH_ENABLED == false build (legacy stop-and-go). ---
add_executable(test_path_tracker_passthrough_off
    ${TEST_DIR}/test_path_tracker_passthrough.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_compile_definitions(test_path_tracker_passthrough_off
    PRIVATE PATH_PASS_THROUGH_ENABLED_DEFAULT=false)
target_include_directories(test_path_tracker_passthrough_off PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_passthrough_off)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_passthrough_off)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_passthrough_off COMMAND test_path_tracker_passthrough_off)

# --- Test executable #3a — BIDIRECTIONAL_AUTO_ENABLED == true build (kill-switch ON, default). ---
# The compile-flag toggle flips the resolved direction the REAL FSM takes on the same fixture
# heading; the rear-override LINE hop on the default 2-board config drives through the real
# `Drive.cpp` + `PathTracker.cpp` (no board-count target split — `LINE_BOARD_COUNT` has no
# per-target override hook, so the 1-board REVERSE_UNAVAILABLE fault is asserted at the frozen
# `reverseUnavailable` resolver level inside the test, not via a 1-board real-FSM build).
add_executable(test_path_tracker_bidirectional_on
    ${TEST_DIR}/test_path_tracker_bidirectional.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_compile_definitions(test_path_tracker_bidirectional_on
    PRIVATE BIDIRECTIONAL_AUTO_ENABLED_DEFAULT=true)
target_include_directories(test_path_tracker_bidirectional_on PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_bidirectional_on)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_bidirectional_on)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_bidirectional_on COMMAND test_path_tracker_bidirectional_on)

# --- Test executable #3b — BIDIRECTIONAL_AUTO_ENABLED == false build (kill-switch OFF). ---
# Asserts that the same fixture heading the ON build resolves one way, the OFF build resolves
# the other — i.e. the compile-time toggle genuinely controls the REAL FSM's runtime direction
# resolution, not just compilation.
add_executable(test_path_tracker_bidirectional_off
    ${TEST_DIR}/test_path_tracker_bidirectional.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_compile_definitions(test_path_tracker_bidirectional_off
    PRIVATE BIDIRECTIONAL_AUTO_ENABLED_DEFAULT=false)
target_include_directories(test_path_tracker_bidirectional_off PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_bidirectional_off)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_bidirectional_off)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_bidirectional_off COMMAND test_path_tracker_bidirectional_off)

# --- Test executable #3c — PathTracker STOP_AT_LINE seek/re-anchor/fault, real-linked. ---
# Drives the REAL STOP_AT_LINE state body: `Start→Inter1` seek success re-anchor (via real
# `Localization.onIntersection`), DRIVE_FAULT on no-line (the real `Drive` seek latches its
# no-line fault, the real PathTracker reads `Drive.hasFault()` → `Fault::DRIVE_FAULT`), and the
# seek-overshoot trim on the sensor-regime pivot — all over the real frozen resolvers
# (`resolveStopAtLine` / `resolveSensorOvershootTrim`). The KEPT direct-call resolver truth-table
# TCs are NOT mirrors and stay.
add_executable(test_path_tracker_stop_at_line
    ${TEST_DIR}/test_path_tracker_stop_at_line.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_include_directories(test_path_tracker_stop_at_line PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_stop_at_line)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_stop_at_line)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_stop_at_line COMMAND test_path_tracker_stop_at_line)

# --- Test executable #3e — PathTracker projection-NODE dispatch, real-linked (issue #64 / #106). ---
# Drives the two projection cells of the REAL `_dispatchEdgeAfterTurn` through the real FSM:
#   - LINE  → projection : distance-gated PROJECTION_FOLLOW, dead-reckon advance (`Inter1→ProjL`).
#   - BLIND → projection : LOCAL auto-seek (`BLIND_PROJECTION_AUTO_SEEK && isProjectionNode`)
#                          → real `Drive.stopAtLine` seek, full-advance re-anchor (`Corner→ProjB`).
# The auto-rule is graph DATA baked into the fixture, asserted via the real frozen resolver.
#
# SWITCHED off the TOY golden `routes_toy.cpp` onto the feature-complete
# `fixtures/routes_fixture.cpp` (the consolidation: all five tests resolve nodes BY NAME over
# the one fixture, which already carries the `ProjL` / `ProjB` projection nodes). Same shared
# recipe as the siblings — real `PathTracker.cpp` + `Drive.cpp` + `Localization.cpp` +
# `YawSource.cpp`, `path_mocks.cpp` globals, fixture routes force-included FIRST.
add_executable(test_path_tracker_projection
    ${TEST_DIR}/test_path_tracker_projection.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_include_directories(test_path_tracker_projection PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_projection)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_projection)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_projection COMMAND test_path_tracker_projection)

# --- Test executable #3f — PathTracker `line(A)-projection(B)-line(C)` pass-through, real-linked. ---
# Drives the REAL pass-through gate (`PathTracker::_evaluateProjectionPassThrough`) over
# `Inter1→ProjL→Inter2`: TC-PP1 the distance-gated gate FIRES (continuous follow, dead-reckon
# advance at `ProjL`, re-anchor at `Inter2`); TC-PP2 `goTo(ProjL)` where `ProjL` is the goal →
# not-final guard → stop; TC-PP3 BLIND-flanked / non-collinear → gate does NOT fire → stop.
# Each pins routing with `getNextHop` + `assertNoBypass(Inter1,Inter2)` (the bypass edge is NONE).
#
# SWITCHED off the TOY golden `routes_toy.cpp` onto the feature-complete
# `fixtures/routes_fixture.cpp` (same consolidation as `test_path_tracker_projection`). Same
# shared recipe — real `PathTracker.cpp` + `Drive.cpp` + `Localization.cpp` + `YawSource.cpp`,
# `path_mocks.cpp` globals, fixture routes force-included FIRST.
add_executable(test_path_tracker_pass_through_projection
    ${TEST_DIR}/test_path_tracker_pass_through_projection.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_include_directories(test_path_tracker_pass_through_projection PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_pass_through_projection)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_pass_through_projection)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_pass_through_projection COMMAND test_path_tracker_pass_through_projection)

# =========================================================================================
# Test executable #3g — sim<->firmware parity DUMP target (issue #108, T6 / AC4).
#
# Drives the REAL `PathTracker` + `Drive` + `Localization` + `YawSource` FSM over the
# feature-complete `fixtures/routes_fixture.cpp` (all-pairs + curated specials/offset cases)
# and prints the normalized per-hop semantic trace as VERSIONED NDJSON on stdout. Same shared
# recipe as the sibling path_tracker targets — real sources + `path_mocks.cpp` globals, fixture
# routes force-included FIRST, then the `path_mocks` peripherals; SKETCH_DIR leads the include
# dirs (THE INVERSION, so the real `Drive.h` / `PathTracker.h` win).
#
# It compiles with `PT_PARITY_TRACE` (turns on PathTracker.cpp's in-FSM trace hook +
# PathTrackerParityTrace.h's `pt_parity::emit`) and `PARITY_SCHEMA_VERSION=2`. It does NOT
# define `LINE_BOARD_COUNT`: `ConfigLine.h` has a bare `constexpr uint8_t LINE_BOARD_COUNT = 2;`
# with no `_DEFAULT` override hook, so `-D` would rewrite that line to `constexpr uint8_t 2 = 2;`
# and break the build — and it is redundant (REAR/reverse is already reachable at the default 2,
# exactly like the bidirectional targets, which set no board-count def).
#
# Deliberately NO `add_test(...)`: this is a DUMP binary the dedicated `parity` CI job and the
# parity pytest (`test_pathtracker_parity.py`) build + invoke — it is NOT a ctest. It still gets
# a compile check from `cmake --build` (the `host-tests` job builds the whole tree).
# =========================================================================================
add_executable(test_path_tracker_parity_dump
    ${TEST_DIR}/test_path_tracker_parity_dump.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_include_directories(test_path_tracker_parity_dump PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_parity_dump)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_parity_dump)            # EventTrace + path_mocks peripherals
target_compile_definitions(test_path_tracker_parity_dump
    PRIVATE PT_PARITY_TRACE PARITY_SCHEMA_VERSION=2)

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
# Test executable — TickProfiler min/avg/max accumulation math (#123, T4).
# TickProfiler.h is <stdint.h>-only, so no production .cpp / MOCK_SOURCES needed; a local
# duck-typed CaptureStream (no Print base) exercises report().
# =========================================================================================
add_executable(test_tick_profiler ${TEST_DIR}/test_tick_profiler.cpp)
target_include_directories(test_tick_profiler PRIVATE ${COMMON_INCLUDES})
add_test(NAME tick_profiler COMMAND test_tick_profiler)

# =========================================================================================
# Test executable — stop-at-line full-domain CSV parity (#63).
#
# Drives ALL 936 rows of tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv
# through the header-only production resolveStopAtLine (StopAtLinePolicy.h) and asserts
# (seek, motionMode) per row — the C++ mirror of Python TC13. The resolver is a pure
# inline free function that reads nothing from the route tables, so this target links NO
# real PathTracker/Drive/routes.cpp — only MOCK_SOURCES for the Serial/g_mockMillis externs
# the mock Arduino.h declares (StopAtLinePolicy.h pulls Config.h -> <Arduino.h>, and
# routes.h for enum TYPES only — no route-table symbol is ODR-used).
#
# The CSV path is baked as a compile definition (absolute, forward-slash CMake path, so the
# string literal needs no backslash escaping and ifstream accepts it on both OSes), robust
# regardless of ctest cwd/build-dir. Stays in the DEFAULT suite (CI host-tests + "Run All"
# run it, so a resolver regression turns CI red) but carries a `parity` label so
# `ctest -LE parity` skips it locally and `ctest -L parity` / `-R` runs only it.
# =========================================================================================
add_executable(test_stop_at_line_truth_table
    ${TEST_DIR}/test_stop_at_line_truth_table.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_stop_at_line_truth_table PRIVATE ${COMMON_INCLUDES})

set(STOP_AT_LINE_CSV ${SKETCH_DIR}/../tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv)
target_compile_definitions(test_stop_at_line_truth_table
    PRIVATE STOP_AT_LINE_TRUTH_TABLE_CSV="${STOP_AT_LINE_CSV}")

add_test(NAME stop_at_line_truth_table COMMAND test_stop_at_line_truth_table)
set_tests_properties(stop_at_line_truth_table PROPERTIES LABELS "parity")

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
# scope HERE — it links the REAL `Mission.cpp` in the dedicated
# `test_mission_dispatch` target below (#46 Phase 2, which landed), against the
# `hal_mocks/` EventTrace + `MiniR4` that this `mocks/`-world target lacks.
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
    ${SKETCH_DIR}/src/manip/Manipulator.cpp
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
    ${SKETCH_DIR}/src/perception/Vision.cpp
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
# Links the REAL `Mission.cpp` + `Tasks.cpp` + `MissionScript.cpp` + the production
# `routes.cpp` against the recording mocks under `ctps_mocks/` (NOT the sibling `mocks/`
# LineFollower harness). The production field now carries a full multi-node graph, so
# the default dump renders the REAL `buildMission` against the real field, and the
# `--self-test` fixture mission resolves its node names (`Start` + `i5`) against that
# same field. (The issue #91 `routes_fixture.cpp` is still linked by the FSM host tests
# that assert RESET / projection nodes the production field lacks — just not here.)
# Running the executable executes a mission to completion
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
    ${SKETCH_DIR}/src/mission/Mission.cpp
    ${SKETCH_DIR}/Tasks.cpp
    ${SKETCH_DIR}/MissionScript.cpp
    ${SKETCH_DIR}/src/nav/routes.cpp
    ${CTPS_MOCKS_DIR}/ctps_record.cpp
)
# `${CTPS_MOCKS_DIR}` FIRST; `${MOCKS_DIR}` intentionally omitted (see the block comment).
target_include_directories(ctps_dump PRIVATE ${CTPS_MOCKS_DIR} ${SKETCH_DIR} ${TEST_DIR})
target_compile_definitions(ctps_dump PRIVATE CTPS_RECORD)
# The default dump runs the REAL `buildMission`, whose gotos reference PRODUCTION node
# names (`i5`, `shovel`, ...), so the production `routes.cpp` MUST be the linked graph —
# resolving names against the issue #91 fixture routes would record every goto as the
# empty node. The production field is now multi-node, so the `--self-test` fixture mission
# resolves its own names (`Start` + `i5`) against it too. No routes force-include: the
# recording mocks (`PathTracker.h` / `Localization.h`) pull in the production `routes.h`
# directly.
add_ctps_mock_force_includes(ctps_dump)
# Pin the dump binary to the build-tree root so the Python orchestrator
# (tools/path_planner/ctps) locates it under any generator. Single-config
# generators (Make/Ninja) already place it there; the _DEBUG/_RELEASE variants
# cover multi-config generators (Xcode/MSBuild) that would otherwise nest it in
# a per-config subdir.
set_target_properties(ctps_dump PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY         ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR})
add_test(NAME ctps_dump_selftest COMMAND ctps_dump --self-test)
set_tests_properties(ctps_dump_selftest PROPERTIES
    PASS_REGULAR_EXPRESSION "CTPS DUMP SELFTEST OK")

# =========================================================================================
# CTPS mock-conformance TU #8b — `test_ctps_mock_conformance` (issue #89, task T5).
#
# The COMPILE-TIME half of the anti-drift guard. This always-built TU chains every
# CURRENT real fluent-builder setter — with its real argument type — on a discarded
# temporary of each recording mock builder (`ctps_mocks/Drive.h` + `LineFollower.h`).
# A removed / renamed / signature-incompatible mock setter breaks THIS target's build,
# which ctest surfaces (`ctps_mock_conformance`). The Python name-superset check (T12)
# is the complementary "real-grew" half. Known TU limit: it verifies call-site
# ASSIGNABILITY, not exact param types (a convertible type can pass) — see the file
# docstring.
#
# Wiring MIRRORS `ctps_dump` (the independent CTPS include + force-include surface):
# `${CTPS_MOCKS_DIR}` FIRST on the include path so the recording `Drive.h` /
# `LineFollower.h` (and the transitively force-included `PathTracker.h` etc.) shadow the
# production headers; `${MOCKS_DIR}` is DELIBERATELY OMITTED (pulling it in would let the
# `mocks/` MatrixMiniR4 shadow the recording one). The mock builder destructors call
# `ctps::recordDriveMove`, so the target links `ctps_record.cpp` (mock `Drive` /
# `LineFollower` globals + the recorder) and the issue #91 `routes_fixture.cpp` (the
# force-included mock `PathTracker.h` transitively pulls `routes.h`). The fixture routes
# are force-included FIRST so their `ROUTES_H` wins before the mocks request the sketch
# `routes.h`. No `EXCLUDE_FROM_ALL` — the guard must build on every `cmake --build`.
# =========================================================================================
add_executable(test_ctps_mock_conformance
    ${CTPS_MOCKS_DIR}/test_ctps_mock_conformance.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${CTPS_MOCKS_DIR}/ctps_record.cpp
)
target_include_directories(test_ctps_mock_conformance PRIVATE ${CTPS_MOCKS_DIR} ${SKETCH_DIR} ${TEST_DIR})
target_compile_definitions(test_ctps_mock_conformance PRIVATE CTPS_RECORD)
add_fixture_routes_force_include(test_ctps_mock_conformance)   # FIRST — its ROUTES_H must win
add_ctps_mock_force_includes(test_ctps_mock_conformance)
add_test(NAME ctps_mock_conformance COMMAND test_ctps_mock_conformance)

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
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${SKETCH_DIR}/src/nav/routes.cpp
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
# Test executable #9b — SensorOvershootPolicy pure-math (issue #65, ADR-044 D1).
#
# `resolveSensorOvershootTrim` is a header-only inline policy (`PivotTrimPolicy.h`) —
# the SENSOR-anchored twin of `resolvePivotTrim`. It feeds the post-detect axle roll
# through the shared subtract-and-clamp with departing trim hard-coded to 0 (ADR-044 D1:
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
# Test executable #12 — RETURN_TO_NODE per-kind dispatch, real-linked (issue #36 / #113).
#
# DE-MIRRORED onto the real FSM (#113): this target was a PURE-DATA target (it linked NO
# real SUT — only `routes_fixture.cpp` + `MOCK_SOURCES` — and inline-MIRRORED the per-kind
# routing as `replica_returnTick`). It now host-links the REAL `PathTracker.cpp` +
# `Drive.cpp` + `Localization.cpp` + `YawSource.cpp` on the shared issue-#106 recipe and
# drives the genuine RETURN_TO_NODE state body: LINE-seek-return success/no-line, the
# short-LINE dead-reckon gate, and the BLIND dead-reckon return — all over the real frozen
# resolver and the real `Localization` offset integrator.
#
# The TC4 (real `FieldMap::getNodeType` LINE-vs-BLIND) and TC4c (real
# `Config::LINE_SENSOR_AXLE_OFFSET_CM` distance gate) direct real-symbol vectors are NOT
# mirrors — they stay as routing guards.
#
# Same shared recipe as the siblings — see the issue-#106 block comment above
# `test_path_tracker_passthrough_on` for the full rationale: `${SKETCH_DIR}`-FIRST include
# order (the inversion), `${MOCKS_DIR}` / `COMMON_INCLUDES` OMITTED entirely (`mocks/Drive.h`
# shares guard `DRIVE_H` with the real `Drive.h`), link `path_mocks.cpp` (NOT `hal_mocks.cpp`
# / `mocks/mocks.cpp`, which double-define `Drive` / `MiniR4`), fixture routes force-included
# FIRST then `add_path_force_includes`. NO `add_mock_force_includes` (that mock-Drive shadow
# is exactly what the inversion drops).
# =========================================================================================
add_executable(test_return_to_node_dispatch
    ${TEST_DIR}/test_return_to_node_dispatch.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_include_directories(test_return_to_node_dispatch PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_return_to_node_dispatch)   # FIRST (ROUTES_H)
add_path_force_includes(test_return_to_node_dispatch)            # EventTrace + path_mocks peripherals
add_test(NAME return_to_node_dispatch COMMAND test_return_to_node_dispatch)

# =========================================================================================
# Test executable #13 — wall-zero re-anchor (Localization::reanchorAtWall), strict OFF (PR2, T18).
#
# TC11 (AC9) + TC12-OFF (AC11): links the REAL production `Localization.cpp` (same surface as
# `test_localization_offset`) and drives `reanchorAtWall(axis, wallCoord, headingDeg)` through the
# mock IMU (`mmL`) + mock `Drive` encoders, asserting the atomic success post-conditions, the
# no-mutation reject path (failed IMU read), and that with the strict flag OFF the re-anchor always
# proceeds. Compiles against the SHIPPED Config default (strict OFF).
#
# Same link recipe as `test_localization_offset`: mock `Drive.h` force-included (guard `DRIVE_H`)
# to shadow the real one; `<MatrixMiniR4.h>` / `<Arduino.h>` resolved by the `mocks/`-first path;
# `routes.cpp` linked for the `Routes::START_NODE / NODES / N_NODES` symbols Localization odr-uses
# (FieldMap::getNode reads NODES for the believed-coordinate post-condition).
# =========================================================================================
add_executable(test_reanchor_at_wall
    ${TEST_DIR}/test_reanchor_at_wall.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_reanchor_at_wall PRIVATE ${COMMON_INCLUDES})
# Issue #91: links the stable fixture routes for the `Wall1` RESET node Localization
# re-anchors against (resolved by name); the single-node production field carries none.
# Force-include FIRST so the fixture `ROUTES_H` wins before any header pulls in routes.h.
add_fixture_routes_force_include(test_reanchor_at_wall)
add_mock_force_includes(test_reanchor_at_wall)   # shadows the real Drive.h with the mock
add_test(NAME reanchor_at_wall COMMAND test_reanchor_at_wall)

# =========================================================================================
# Test executable #14 — wall-normal STRICT boundary, strict ON (PR2, T18). TC12-ON (AC11).
#
# `Config::WALL_NORMAL_STRICT_ENABLED` (in `config/ConfigNav.h`) is guarded by an
# `#ifndef WALL_NORMAL_STRICT_ENABLED_DEFAULT` override hook — the SHIPPED default is `false`, so a
# merge can never silently flip it. This target flips it ON target-wide with
# `target_compile_definitions(... WALL_NORMAL_STRICT_ENABLED_DEFAULT=true)`, so BOTH the
# separately-compiled `Localization.cpp` TU and the test main see the flag ON (the `-D` reaches every
# TU in the target, sidestepping the cross-TU `constexpr` override trap). Mirrors the
# `MISSION_ALLOW_REARM_DEFAULT` / `PATH_PASS_THROUGH_ENABLED_DEFAULT` idiom — no Config shadow header
# needed. Otherwise identical to `test_reanchor_at_wall` (strict OFF); the mock Drive.h is still
# force-included (guard `DRIVE_H`).
# =========================================================================================
add_executable(test_reanchor_at_wall_strict
    ${TEST_DIR}/test_reanchor_at_wall_strict.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_reanchor_at_wall_strict PRIVATE ${COMMON_INCLUDES})
# Force the strict wall-normal flag ON for THIS target only. The production default in
# `config/ConfigNav.h` is guarded by `#ifndef WALL_NORMAL_STRICT_ENABLED_DEFAULT`, so this `-D`
# flips it target-wide (both `Localization.cpp` and the test main see strict = true) without a
# per-target Config shadow header or the cross-TU `constexpr` override trap. Mirrors the
# `MISSION_ALLOW_REARM_DEFAULT` / `PATH_PASS_THROUGH_ENABLED_DEFAULT` idiom.
target_compile_definitions(test_reanchor_at_wall_strict PRIVATE WALL_NORMAL_STRICT_ENABLED_DEFAULT=true)
# Issue #91 (fixture FIRST — see test_reanchor_at_wall).
add_fixture_routes_force_include(test_reanchor_at_wall_strict)
add_mock_force_includes(test_reanchor_at_wall_strict)   # shadows the real Drive.h with the mock
add_test(NAME reanchor_at_wall_strict COMMAND test_reanchor_at_wall_strict)

# =========================================================================================
# Test executable #15 — Drive.wallStop watchdog state machine (PR2, T14). TC9 + TC10 (AC8).
#
# `Drive.cpp` is NOT host-compiled (heavy MatrixMiniR4 deps), so this target INLINE-MIRRORS the
# WALL_STOP PUSH -> stall? -> BACK_OFF / outer-timeout -> advisory slice (annotated `// MIRROR:
# Drive.cpp:<line>`), the same precedent as `test_path_tracker_stop_at_line`. The mirror calls the
# SAME `stopAtLineProgressed` / `stopAtLineStalled` primitives from `WROMath.h` (host-compilable)
# with the SAME `Config` constants the production `_updateWallStop` uses, so the watchdog DECISION
# is exercised, not re-derived.
#
# Minimal TU set: the mirror touches no Drive/LineArray/Intersection surface, so this target does
# NOT force-include those mocks (mirrors `test_stop_at_line_armmath`). It links `MOCK_SOURCES` only
# for the `Serial` / `g_mockMillis` globals the mock `<Arduino.h>` declares `extern` (pulled in via
# `WROMath.h`'s `<Arduino.h>`).
# =========================================================================================
add_executable(test_wall_stop_watchdog
    ${TEST_DIR}/test_wall_stop_watchdog.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_wall_stop_watchdog PRIVATE ${COMMON_INCLUDES})
add_test(NAME wall_stop_watchdog COMMAND test_wall_stop_watchdog)

# =========================================================================================
# Test executable #15b — turn-profile persistent-IMU-fault watchdog (PR #82 review fix).
#
# `Drive.cpp` is NOT host-compiled, so the read-failure escalation in `_updateTurnProfile()`
# (hard-stop + `Fault::IMU_READ_FAILED` once no good IMU yaw read has landed for
# `Config::TURN_IMU_STALL_TIMEOUT_MS`) is verified at the DECISION layer: the `turnImuReadStalled`
# predicate in `WROMath.h`, bound to the shipped Config timeout. Same minimal recipe as
# `test_wall_stop_watchdog` / `test_stop_at_line_armmath` — the predicate touches no
# Drive/LineArray/Intersection surface, so no force-includes; `MOCK_SOURCES` only supplies the
# `Serial` / `g_mockMillis` globals the mock `<Arduino.h>` (pulled in via `WROMath.h`) declares.
# =========================================================================================
add_executable(test_turn_imu_watchdog
    ${TEST_DIR}/test_turn_imu_watchdog.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_turn_imu_watchdog PRIVATE ${COMMON_INCLUDES})
add_test(NAME turn_imu_watchdog COMMAND test_turn_imu_watchdog)

# =========================================================================================
# Test executable #16 — PathTracker reset-node arrival interlude, real-linked, strict OFF (#113).
# TC-RA1..TC-RA3 (AC10 + AC11).
#
# DE-MIRRORED onto the real FSM (#113): this target previously INLINE-MIRRORED the
# reset-arrival decision slice (`enum RState` / `struct ResetArrivalMirror`, annotated
# `// MIRROR: PathTracker.cpp:<line>`) and linked only `Localization.cpp` + mock `Drive`. It
# now host-links the REAL `PathTracker.cpp` + `Drive.cpp` + `Localization.cpp` +
# `YawSource.cpp` on the shared issue-#106 recipe and drives the genuine reset interlude
# (`_maybeEnterReset` -> WALL_STOP -> REANCHOR -> WALL_DEPART -> PLAN_NEXT_HOP): the
# advisory-timeout path, the clean stall-success path, and the one-shot latch — the REANCHOR
# step calls the real `reanchorAtWall(...)` over the fixture `Wall1` reset record.
#
# This is the strict-OFF half (shipped Config default); the strict-ON sibling
# `test_path_tracker_reset_arrival_strict` below drives the real `WALL_NOT_SQUARE` reject —
# the same OFF/ON split as `test_reanchor_at_wall` / `test_reanchor_at_wall_strict`.
#
# Same shared recipe as the siblings — see the issue-#106 block comment above
# `test_path_tracker_passthrough_on` for the full rationale: `${SKETCH_DIR}`-FIRST include
# order (the inversion), `${MOCKS_DIR}` / `COMMON_INCLUDES` OMITTED entirely (`mocks/Drive.h`
# shares guard `DRIVE_H` with the real `Drive.h`), link `path_mocks.cpp` (NOT `hal_mocks.cpp`
# / `mocks/mocks.cpp`), fixture routes force-included FIRST then `add_path_force_includes`.
# NO `add_mock_force_includes`.
# =========================================================================================
add_executable(test_path_tracker_reset_arrival
    ${TEST_DIR}/test_path_tracker_reset_arrival.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_include_directories(test_path_tracker_reset_arrival PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_reset_arrival)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_reset_arrival)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_reset_arrival COMMAND test_path_tracker_reset_arrival)

# =========================================================================================
# Test executable #16b — PathTracker reset-node arrival, real-linked, strict ON (#113).
# TC-RA-strict (AC11).
#
# The strict-ON half of the reset-arrival split (the `test_reanchor_at_wall_strict`
# precedent): same inbound-hop + wallStop drive as `test_path_tracker_reset_arrival`, but
# compiled with `WALL_NORMAL_STRICT_ENABLED` flipped ON target-wide. With strict ON and the
# mock IMU parked off-normal at the REANCHOR tick, the real `reanchorAtWall(...)` rejects, so
# the real FSM enters `Fault::WALL_NOT_SQUARE` and issues NO depart drive.
#
# `Config::WALL_NORMAL_STRICT_ENABLED` (in `config/ConfigNav.h`) is guarded by an
# `#ifndef WALL_NORMAL_STRICT_ENABLED_DEFAULT` override hook (shipped default `false`). The
# `target_compile_definitions(... WALL_NORMAL_STRICT_ENABLED_DEFAULT=true)` below flips it ON
# for EVERY TU in this target (the separately-compiled `Localization.cpp` that runs the
# strict check AND the test main that asserts the precondition) — the same target-wide `-D`
# idiom as `test_reanchor_at_wall_strict`, sidestepping the cross-TU `constexpr` override trap.
#
# Otherwise identical to the strict-OFF target: same shared issue-#106 recipe
# (`${SKETCH_DIR}`-FIRST, `path_mocks.cpp`, fixture routes force-included FIRST, NO
# `add_mock_force_includes`).
# =========================================================================================
add_executable(test_path_tracker_reset_arrival_strict
    ${TEST_DIR}/test_path_tracker_reset_arrival_strict.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
# Force the strict wall-normal flag ON for THIS target only (both `Localization.cpp` and the
# test main see strict = true) — same target-wide `-D` as `test_reanchor_at_wall_strict`.
target_compile_definitions(test_path_tracker_reset_arrival_strict PRIVATE WALL_NORMAL_STRICT_ENABLED_DEFAULT=true)
target_include_directories(test_path_tracker_reset_arrival_strict PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_reset_arrival_strict)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_reset_arrival_strict)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_reset_arrival_strict COMMAND test_path_tracker_reset_arrival_strict)

# =========================================================================================
# Test executable #17 — YawSource CW->CCW adapter, real-linked (ADR-045, TC3/TC4).
#
# Links the REAL production `YawSource.cpp` against the mock lower-MCU IMU surface
# (`mmL.GetIMUEuler` + `mmL.mockYawDeg` / `mmL.mockResult` / `mmL.getImuEulerCallCount`
# in `mocks/MatrixMiniR4.h`), exercising the sole `Config::IMU_YAW_SIGN` conversion
# boundary: `readYawDeg` / `readYawDegOr` (STM32 raw -> app frame, with the retry-once
# transient-failure path) and the static `toHwHeading` (app heading -> STM32 gyro-PID
# target). With the shipped default `IMU_YAW_SIGN == -1.0f`, every corrected value is the
# raw value negated, so the test pins both the sign and the untouched-out-param-on-failure
# contract.
#
# `YawSource.cpp` pulls `<MatrixMiniR4.h>` + quoted `"Config.h"` via the `mocks/`-first
# `COMMON_INCLUDES`, so the mock IMU + the host `Arduino.h` resolve without the real SDK.
# It does NOT include the production `Drive.h` / `WROLineArray.h` / `Intersection.h`, so
# this target DELIBERATELY does NOT call `add_mock_force_includes(...)` — there is no
# production header to shadow. `MOCK_SOURCES` supplies the `Serial` / `g_mockMillis`
# globals the mock `<Arduino.h>` declares `extern` (pulled in via YawSource.cpp's
# `delay()` on the retry path); the `mmL` global is an `inline` variable in the mock
# header, so no extra source defines it.
# =========================================================================================
add_executable(test_yaw_source
    ${TEST_DIR}/test_yaw_source.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_yaw_source PRIVATE ${COMMON_INCLUDES})
add_test(NAME yaw_source COMMAND test_yaw_source)

# =========================================================================================
# Test executable #18 — TurnSignPolicy pure-math (ADR-045, TC5/TC6).
#
# `normalizeTurnDelta` / `turnSign` / `turnWheelPattern` are header-only inline policies
# (`TurnSignPolicy.h`) promoted out of Drive.cpp so production AND this host test link the
# SAME symbols — the `PivotTrimPolicy.h` / `DirectionPolicy.h` precedent. Drive.cpp is NOT
# host-compiled (heavy MatrixMiniR4 deps), so before this promotion the turn-fold, the
# sign capture, and the wheel-pattern switch had no automated coverage at all.
#
# Pure integer math with zero Arduino / Config.h surface, so this target needs no mocks /
# SUT sources — just the sketch include dir for the self-contained header (mirrors
# `test_pivot_trim_policy`, which is `${SKETCH_DIR}`-only for the same reason).
# =========================================================================================
add_executable(test_turn_sign_policy
    ${TEST_DIR}/test_turn_sign_policy.cpp
)
target_include_directories(test_turn_sign_policy PRIVATE ${SKETCH_DIR})
add_test(NAME turn_sign_policy COMMAND test_turn_sign_policy)

# =========================================================================================
# Test executable #19 — IMU_YAW_SIGN tripwire negative (ADR-045, TC7 / AC4, compile-fail).
#
# Proves the `static_assert` in `ConfigChecks.h` fires when `Config::IMU_YAW_SIGN` is not
# exactly +/-1. Built with `-DIMU_YAW_SIGN_DEFAULT=0.0f` (the `#ifndef`-guarded override
# hook in `config/ConfigNav.h`), so `IMU_YAW_SIGN == 0.0f` — a value that would silently
# zero all heading math. The build is expected to FAIL; ctest uses PASS_REGULAR_EXPRESSION
# on the metacharacter-free substring `IMU_YAW_SIGN must be exactly`. When
# PASS_REGULAR_EXPRESSION is set, ctest decides pass/fail purely on whether the regex
# matches the captured build output and IGNORES the process exit code — so the test PASSES
# only when the intended assert fires, and FAILS if the build broke for any unrelated
# reason.
#
# Mirrors `test_manipulator_belt_collision` EXACTLY: `EXCLUDE_FROM_ALL` +
# `EXCLUDE_FROM_DEFAULT_BUILD` keep it out of the default build, and NO
# `add_mock_force_includes(...)` so a typo in a mock header cannot mask the intended
# diagnostic. Include dirs = sketch + mocks (via `COMMON_INCLUDES`) so the quoted
# `"Config.h"` chain resolves and its `<Arduino.h>` picks the mock.
# =========================================================================================
add_executable(test_imu_yaw_sign_zero
    ${TEST_DIR}/test_imu_yaw_sign_fail.cpp
)
set_target_properties(test_imu_yaw_sign_zero PROPERTIES
    EXCLUDE_FROM_ALL TRUE
    EXCLUDE_FROM_DEFAULT_BUILD TRUE
)
target_include_directories(test_imu_yaw_sign_zero PRIVATE ${COMMON_INCLUDES})
target_compile_definitions(test_imu_yaw_sign_zero PRIVATE IMU_YAW_SIGN_DEFAULT=0.0f)

add_test(NAME imu_yaw_sign_zero_compile_fail
    COMMAND ${CMAKE_COMMAND} --build "${CMAKE_BINARY_DIR}"
            --target test_imu_yaw_sign_zero --config $<CONFIG>
)
set_tests_properties(imu_yaw_sign_zero_compile_fail PROPERTIES
    PASS_REGULAR_EXPRESSION "IMU_YAW_SIGN must be exactly")

# =========================================================================================
# Test executable #20 — shared host-test mock HAL signature conformance (issue #105).
#
# `test_mock_hal` is the self-test for the unified `hal_mocks/` mock HAL — the one shared
# `MiniR4` facade + full Drive-builder mock + `EventTrace` mock that the later epic-#107
# issues (#106 PathTracker/Drive, #46 Phase 2 Mission) will host-link the REAL `.cpp`
# against. This target proves the surface WITHOUT linking any heavy production `.cpp`:
#
#   - a never-run signature-conformance block in `test_mock_hal.cpp` writes every
#     `Drive.` / `MiniR4.` / `EventTrace.` call expression grepped from the real
#     `Drive.cpp` / `Mission.cpp` / `PathTracker.cpp`, so a missing or wrong-signature
#     mock method is a COMPILE error (DCE runs post-typecheck);
#   - live `TestAssert.h` cases exercise the control surface (fault set->observe, builder
#     argument capture, `resetCounter` zeroes degrees, `pushEvent` ordered capture, reset).
#
# Only `hal_mocks/hal_mocks.cpp` is linked (the single definer of the `hal_mocks/`
# globals `MiniR4` / `Drive` / `EventTrace` + the `<Arduino.h>` globals the target
# odr-uses); NO production `.cpp` and NEVER `mocks/mocks.cpp`.
#
# Like `ctps_dump`, this is an INDEPENDENT include + force-include surface from the
# LineFollower harness: `hal_mocks/` must come FIRST so the angle-bracket
# `<MatrixMiniR4.h>` / `<Arduino.h>` resolve to the HAL mocks, and `${MOCKS_DIR}` is
# DELIBERATELY OMITTED from this target's include path. That omission is the
# ODR mutual-exclusion invariant: `mocks/Drive.h` and `hal_mocks/Drive.h` are two
# different `class DriveClass` definitions sharing the `DRIVE_H` guard (likewise the two
# facades), so no single target may have BOTH dirs reachable — pulling `mocks/` in would
# double-define the facade / Drive surface. Mirror `ctps_dump`'s isolation exactly.
# =========================================================================================
# `HAL_MOCKS_DIR` is now set in the top dir-var block (hoisted there for the issue-#106
# PathTracker+Drive real-link targets that also consume it) — no re-`set()` needed here.

# -----------------------------------------------------------------------------------------
# Force-include the QUOTE-included HAL mock headers BEFORE any source-file parsing — the same
# load-bearing shadow trick as `add_ctps_mock_force_includes`, pointed at `hal_mocks/` and
# using ABSOLUTE `${HAL_MOCKS_DIR}/` paths.
#
# ONLY the quote-included production headers need force-including: `hal_mocks/Drive.h`
# (guard `DRIVE_H`) and `hal_mocks/EventTrace.h` (guard `EVENTTRACE_H`). The real
# `Drive.h` / `EventTrace.h` sit next to `Drive.cpp` / `Mission.cpp` in `${SKETCH_DIR}`, so
# a quote-include would otherwise resolve source-relative to the REAL header; force-including
# the mock first makes the production header no-op (its guard is already defined) when it is
# re-requested transitively.
#
# `MatrixMiniR4.h` / `Arduino.h` are ANGLE-bracket includes, so the `${HAL_MOCKS_DIR}`-first
# include order alone picks the mock versions — no force-include needed for those two (exactly
# like `add_ctps_mock_force_includes` force-includes Drive/PathTracker/EventTrace but NOT
# MatrixMiniR4/Arduino).
# -----------------------------------------------------------------------------------------
function(add_hal_force_includes target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            "/FI${HAL_MOCKS_DIR}/Drive.h"
            "/FI${HAL_MOCKS_DIR}/EventTrace.h"
        )
    else()
        # `SHELL:` keeps each `-include <path>` pair atomic — see the note in
        # `add_mock_force_includes` for why CMake de-dup would otherwise break the build
        # (`cannot specify -o with -c ... multiple files`).
        target_compile_options(${target} PRIVATE
            "SHELL:-include ${HAL_MOCKS_DIR}/Drive.h"
            "SHELL:-include ${HAL_MOCKS_DIR}/EventTrace.h"
        )
    endif()
endfunction()

add_executable(test_mock_hal
    ${TEST_DIR}/test_mock_hal.cpp
    ${HAL_MOCKS_DIR}/hal_mocks.cpp
)
# hal_mocks/ FIRST so <MatrixMiniR4.h>/<Arduino.h> resolve to the HAL mock; mocks/ is
# DELIBERATELY OMITTED (ODR mutual-exclusion invariant — mirror ctps_dump's isolation).
target_include_directories(test_mock_hal PRIVATE ${HAL_MOCKS_DIR} ${SKETCH_DIR} ${TEST_DIR})
add_hal_force_includes(test_mock_hal)
add_test(NAME mock_hal COMMAND test_mock_hal)

# =========================================================================================
# Test executable #21 — Mission queue FSM + TASK dispatch, real-linked (#46 Phase 2).
#
# Links the REAL `Mission.cpp` + REAL `Tasks.cpp` and asserts the production Mission queue
# FSM — queue drain order, `start()` build-error / empty / normal, GOTO happy / unknown-node
# / holds-on-not-idle, WAIT / PRINT / BEEP / LED, the `TASK`-case dispatch into the real
# `Tasks.cpp` (with `T>`/`T+`/`T-` events + real task Serial bodies), the drain-time
# PathTracker-fault check, the FAULT_BEEPING -> FAULT_AWAIT_RESET -> BTN_DOWN reset path,
# `abort()`, and `declarePosition` success / busy / unknown / clears-faults — against the
# production singletons (no inline `MissionMirror`). This closes the gap Phase 1 left when it
# deleted the inline mirror.
#
# Upper-facade surfaces the HAL deliberately doesn't cover come from the new `mission_mocks/`:
# a test-CONTROLLABLE `PathTracker` (independent `mockSetIdle` / `mockSetFault` knobs, the REAL
# 10-value `Fault` enum + exact mnemonics) plus record/no-op `LineFollower` / `Manipulator` /
# `Localization`. The chassis HAL (the `MiniR4` facade + Drive-builder mock + EventTrace mock)
# is the shared `hal_mocks/`. The 14-node `fixtures/routes_fixture.cpp` resolves graph nodes
# BY NAME (`Inter1`.. / `Start` / `final` / `Wall1` / ...) — never by index.
#
# This is the epic-#107 sibling that stays Drive/PathTracker-MOCKED: host-linking the REAL
# `PathTracker.cpp` / `Drive.cpp` (and deleting their `// MIRROR:` slices) is the adjacent
# issue #106, not this target.
#
# Like `ctps_dump` / `test_mock_hal`, this is an INDEPENDENT include + force-include surface:
# `hal_mocks/` must come FIRST so the angle-bracket `<MatrixMiniR4.h>` / `<Arduino.h>` resolve
# to the HAL mock, and `${MOCKS_DIR}` is DELIBERATELY OMITTED from this target's include path.
# That omission is the ODR mutual-exclusion invariant: `mocks/Drive.h` and `hal_mocks/Drive.h`
# are two different `class DriveClass` definitions sharing the `DRIVE_H` guard, so a target
# that reached BOTH would silently drop the second definition (the include guard no-ops it)
# with NO linker error. Mirror `ctps_dump`'s isolation exactly.
# =========================================================================================
set(MISSION_MOCKS_DIR ${TEST_DIR}/mission_mocks)

# -----------------------------------------------------------------------------------------
# Force-include ONLY the 4 mission-mock headers BEFORE any source-file parsing — the same
# load-bearing shadow trick as `add_hal_force_includes`, pointed at `mission_mocks/` and using
# ABSOLUTE `${MISSION_MOCKS_DIR}/` paths.
#
# This helper does NOT re-list `Drive.h` / `EventTrace.h` — those are `add_hal_force_includes`'s
# job, kept single-sourced so the HAL set stays one definition (if `hal_mocks/` grows a third
# force-included header, the target inherits it through that helper, not this one). The real
# `PathTracker.h` / `LineFollower.h` / `Manipulator.h` / `Localization.h` sit next to
# `Mission.cpp` / `Tasks.cpp` in `${SKETCH_DIR}`, so a quote-include would otherwise resolve
# source-relative to the REAL header; force-including the mock first makes the production header
# no-op (its guard `PATHTRACKER_H` / `LINEFOLLOWER_H` / `MANIPULATOR_H` / `LOCALIZATION_H` is
# already defined) when it is re-requested transitively.
#
# `MatrixMiniR4.h` / `Arduino.h` are ANGLE-bracket includes, so the `${HAL_MOCKS_DIR}`-first
# include order alone picks the mock versions — no force-include needed for those two.
# -----------------------------------------------------------------------------------------
function(add_mission_mock_force_includes target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            "/FI${MISSION_MOCKS_DIR}/PathTracker.h"
            "/FI${MISSION_MOCKS_DIR}/LineFollower.h"
            "/FI${MISSION_MOCKS_DIR}/Manipulator.h"
            "/FI${MISSION_MOCKS_DIR}/Localization.h"
        )
    else()
        # `SHELL:` keeps each `-include <path>` pair atomic — see the note in
        # `add_mock_force_includes` for why CMake de-dup would otherwise break the build
        # (`cannot specify -o with -c ... multiple files`).
        target_compile_options(${target} PRIVATE
            "SHELL:-include ${MISSION_MOCKS_DIR}/PathTracker.h"
            "SHELL:-include ${MISSION_MOCKS_DIR}/LineFollower.h"
            "SHELL:-include ${MISSION_MOCKS_DIR}/Manipulator.h"
            "SHELL:-include ${MISSION_MOCKS_DIR}/Localization.h"
        )
    endif()
endfunction()

add_executable(test_mission_dispatch
    ${TEST_DIR}/test_mission_dispatch.cpp
    ${SKETCH_DIR}/src/mission/Mission.cpp
    ${SKETCH_DIR}/Tasks.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${HAL_MOCKS_DIR}/hal_mocks.cpp)
# CUSTOM include list — NOT COMMON_INCLUDES (which contains mocks/, breaking the ODR
# mutual-exclusion: mocks/Drive.h and hal_mocks/Drive.h share the DRIVE_H guard, so the
# second definition is silently dropped with NO linker error). hal_mocks/ FIRST so the
# angle-bracket <MatrixMiniR4.h>/<Arduino.h> resolve to the HAL mock; mocks/ OMITTED;
# mission_mocks/ on the path so the test's quote-includes of the mission mocks resolve.
target_include_directories(test_mission_dispatch PRIVATE
    ${HAL_MOCKS_DIR} ${MISSION_MOCKS_DIR} ${SKETCH_DIR} ${TEST_DIR})
# Force-include composition ORDER (critical):
add_fixture_routes_force_include(test_mission_dispatch)   # FIRST — routes_fixture.h (guard ROUTES_H) MUST win before Mission.cpp/FieldMap.h/Localization pull routes.h
add_hal_force_includes(test_mission_dispatch)             # REUSE as-is — hal_mocks/Drive.h (DRIVE_H) + EventTrace.h (EVENTTRACE_H)
add_mission_mock_force_includes(test_mission_dispatch)    # the 4 mission mocks only
add_test(NAME mission_dispatch COMMAND test_mission_dispatch)

# =========================================================================================
# Test executable — edge-dispatch full-domain CSV parity (#108, T-CSV / TC1 / AC9).
#
# Drives ALL 864 rows of tools/path_planner/tests/fixtures/edge_dispatch_truth_table.csv
# through the header-only production resolveEdgeDispatch (EdgeDispatchPolicy.h) and asserts
# the seven decision fields {primitive, reverse, driveMode, commandedDistanceCm, overshootCm,
# reanchor, brakeOnGate} per row — the C++ mirror of Python TC1
# (test_runtime_sim.py::test_tc1_resolve_edge_dispatch_matches_fixture). The generator
# gen_edge_dispatch_truth_table.py is an INDEPENDENT transcription (never calls either
# resolver), so a drift in the table OR in either resolver turns CI red.
#
# Same shape as test_stop_at_line_truth_table: resolveEdgeDispatch is a pure inline free
# function that COMPOSES resolvePivotTrim / resolveSensorOvershootTrim (PivotTrimPolicy.h)
# and reads only enum TYPES from routes.h — no route-table symbol is ODR-used — so this
# target links NO real PathTracker/Drive/routes.cpp, only MOCK_SOURCES for the
# Serial/g_mockMillis externs the mock Arduino.h declares (EdgeDispatchPolicy.h pulls
# StopAtLinePolicy.h -> Config.h -> <Arduino.h>).
#
# The CSV path is baked as a compile definition (absolute, forward-slash CMake path, so the
# string literal needs no backslash escaping and ifstream accepts it on both OSes), robust
# regardless of ctest cwd/build-dir. Stays in the DEFAULT host-tests suite (NOT the dedicated
# `parity` CI job, which is only for the trace-diff dump) — it is a pure-resolver test with
# no binary handoff, exactly like test_stop_at_line_truth_table — but carries a `parity`
# label so `ctest -LE parity` skips it locally and `ctest -L parity` runs only it.
# =========================================================================================
add_executable(test_edge_dispatch_truth_table
    ${TEST_DIR}/test_edge_dispatch_truth_table.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_edge_dispatch_truth_table PRIVATE ${COMMON_INCLUDES})

set(EDGE_DISPATCH_CSV ${SKETCH_DIR}/../tools/path_planner/tests/fixtures/edge_dispatch_truth_table.csv)
target_compile_definitions(test_edge_dispatch_truth_table
    PRIVATE EDGE_DISPATCH_TRUTH_TABLE_CSV="${EDGE_DISPATCH_CSV}")

add_test(NAME edge_dispatch_truth_table COMMAND test_edge_dispatch_truth_table)
set_tests_properties(edge_dispatch_truth_table PROPERTIES LABELS "parity")

# =========================================================================================
# Test executable #22 — heading-only line-arrival re-anchor (Localization::reanchorHeading),
# strict ON (ADR-052, T3). TC1/TC2/TC3/TC4/TC5/TC6 (AC1-AC5).
#
# Links the REAL production `Localization.cpp` — same link recipe as `test_reanchor_at_wall`
# (mock `Drive.h` force-included to shadow the real one; `<MatrixMiniR4.h>` / `<Arduino.h>`
# resolved by the `mocks/`-first include path; the issue #91 fixture `routes_fixture.cpp`
# linked for the `Routes::START_NODE / START_HEADING_DEG` symbols `Localization.cpp`
# odr-uses, even though `reanchorHeading` itself never reads a node).
#
# STRICT ON/OFF split (see the file docblock in `test_reanchor_heading.cpp`): unlike
# `WALL_NORMAL_STRICT_ENABLED` (shipped OFF), `LINE_HEADING_RESET_STRICT_ENABLED` ships ON —
# so THIS target explicitly forces `LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT=true` /
# `_TOLERANCE_DEG_DEFAULT=10.0f` target-wide (reaches both `Localization.cpp` and the test
# main) rather than relying on the shipped default alone, so the test's own precondition
# check is a real trip-wire against a future accidental default flip — the
# `test_reanchor_at_wall_strict` `-D` idiom, `gotcha-cross-tu-config-override-compile-def`.
# Runs TC1/TC2/TC5/TC6 (guard + accept + failed-read + offset-untouched, flag-independent)
# PLUS TC3 (strict reject) and TC4 (inclusive boundary), which only apply with strict ON.
# =========================================================================================
add_executable(test_reanchor_heading
    ${TEST_DIR}/test_reanchor_heading.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_reanchor_heading PRIVATE ${COMMON_INCLUDES})
# Force strict ON + the 10-deg tolerance target-wide (see the block comment above).
target_compile_definitions(test_reanchor_heading PRIVATE
    LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT=true
    LINE_HEADING_RESET_TOLERANCE_DEG_DEFAULT=10.0f)
# Issue #91: fixture routes for the START_NODE / START_HEADING_DEG symbols Localization.cpp
# odr-uses. Force-include FIRST so the fixture `ROUTES_H` wins before any header pulls routes.h.
add_fixture_routes_force_include(test_reanchor_heading)
add_mock_force_includes(test_reanchor_heading)   # shadows the real Drive.h with the mock
add_test(NAME reanchor_heading COMMAND test_reanchor_heading)

# =========================================================================================
# Test executable #22b — heading-only line-arrival re-anchor, strict OFF (ADR-052, T3).
# Non-vacuity proof for TC3/TC4's ON-boundary: a delta (15 deg) that the strict-ON target
# above REJECTS must be ACCEPTED here, proving the `-D` actually reached the
# separately-compiled `Localization.cpp` TU (not just this test main).
#
# Same `test_reanchor_heading.cpp` SOURCE FILE, compiled into a SECOND executable with
# `LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT=false` forced target-wide. `main()` branches at
# runtime on the compiled-in `Config::LINE_HEADING_RESET_STRICT_ENABLED` value, so this build
# runs TC1/TC2/TC5/TC6 (flag-independent) plus the non-vacuity case instead of TC3/TC4.
# Otherwise identical recipe to `test_reanchor_heading` (fixture routes + mock Drive.h).
# =========================================================================================
add_executable(test_reanchor_heading_strict_off
    ${TEST_DIR}/test_reanchor_heading.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${MOCK_SOURCES}
)
target_include_directories(test_reanchor_heading_strict_off PRIVATE ${COMMON_INCLUDES})
target_compile_definitions(test_reanchor_heading_strict_off PRIVATE
    LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT=false)
add_fixture_routes_force_include(test_reanchor_heading_strict_off)
add_mock_force_includes(test_reanchor_heading_strict_off)
add_test(NAME reanchor_heading_strict_off COMMAND test_reanchor_heading_strict_off)

# =========================================================================================
# Test executables #23a-d — PathTracker FSM gating for the line-arrival heading reset
# (ADR-052, T6). Covers TC8-TC14 (AC7-AC11) — see the file docblock in
# `test_path_tracker_heading_reset.cpp` for the full four-target breakdown.
#
# All four link the REAL `PathTracker.cpp` + `Drive.cpp` + `Localization.cpp` +
# `YawSource.cpp` + `path_mocks.cpp` — the SAME shared recipe as the other PathTracker+Drive
# real-link targets above (SKETCH_DIR-first include order, `add_path_force_includes` for the
# EventTrace/WROLineArray/Intersection/LineFollower peripheral mocks). #23a/b/d link the issue
# #91 `routes_fixture.cpp`; #23c links the DELIBERATELY invalid, hand-written
# `routes_reset_passthrough_forced.cpp` instead (see that file's docblock).
# =========================================================================================

# --- Test executable #23a — default knobs (proj ON, junction OFF, strict ON, tol 10 deg). ---
# Runs TC8, TC9, TC10 (OFF half), TC12, TC13.
add_executable(test_path_tracker_heading_reset
    ${TEST_DIR}/test_path_tracker_heading_reset.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_include_directories(test_path_tracker_heading_reset PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_heading_reset)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_heading_reset)            # EventTrace + path_mocks peripherals
add_test(NAME path_tracker_heading_reset COMMAND test_path_tracker_heading_reset)

# --- Test executable #23b — junction knob ON. Runs TC10's ON half only. ---
add_executable(test_path_tracker_heading_reset_junction_on
    ${TEST_DIR}/test_path_tracker_heading_reset.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_compile_definitions(test_path_tracker_heading_reset_junction_on
    PRIVATE LINE_HEADING_RESET_ON_JUNCTION_DEFAULT=true)
target_include_directories(test_path_tracker_heading_reset_junction_on PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_heading_reset_junction_on)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_heading_reset_junction_on)
add_test(NAME path_tracker_heading_reset_junction_on
    COMMAND test_path_tracker_heading_reset_junction_on)

# --- Test executable #23c — forced-state RESET/BLIND no-op (TC11, AC9). ---
# Links the DELIBERATELY invalid `routes_reset_passthrough_forced.cpp` INSTEAD of
# `routes_fixture.cpp` — see that file's docblock for why AC9's forced-state case needs its
# own topology (a LINE edge into a RESET node, unreachable on any validator-passed field).
add_executable(test_path_tracker_heading_reset_forced_reset
    ${TEST_DIR}/test_path_tracker_heading_reset.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_reset_passthrough_forced.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_compile_definitions(test_path_tracker_heading_reset_forced_reset
    PRIVATE HEADING_RESET_FORCED_RESET_FIXTURE)
target_include_directories(test_path_tracker_heading_reset_forced_reset PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_forced_reset_routes_force_include(test_path_tracker_heading_reset_forced_reset)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_heading_reset_forced_reset)
add_test(NAME path_tracker_heading_reset_forced_reset
    COMMAND test_path_tracker_heading_reset_forced_reset)

# --- Test executable #23d — PT_PARITY_TRACE build (TC14, AC11: parity record only on a bind). ---
# The capture path is baked as an absolute compile definition (CMake forward-slash path, so the
# string literal needs no backslash escaping and ifstream/fopen accept it on both OSes) into
# THIS target's own binary dir, mirroring `STOP_AT_LINE_TRUTH_TABLE_CSV`'s idiom. A normal
# ctest (unlike `test_path_tracker_parity_dump`, which is a dump-only binary with no
# `add_test`) — it asserts pass/fail itself via TestAssert, it does not print NDJSON for an
# external harness to diff.
add_executable(test_path_tracker_heading_reset_parity
    ${TEST_DIR}/test_path_tracker_heading_reset.cpp
    ${SKETCH_DIR}/src/mission/PathTracker.cpp
    ${SKETCH_DIR}/src/drive/Drive.cpp
    ${SKETCH_DIR}/src/nav/Localization.cpp
    ${SKETCH_DIR}/src/nav/YawSource.cpp
    ${FIXTURE_ROUTES_DIR}/routes_fixture.cpp
    ${PATH_MOCKS_DIR}/path_mocks.cpp
)
target_compile_definitions(test_path_tracker_heading_reset_parity
    PRIVATE PT_PARITY_TRACE
    HEADING_RESET_PARITY_CAPTURE_PATH="${CMAKE_CURRENT_BINARY_DIR}/heading_reset_parity_capture.ndjson")
target_include_directories(test_path_tracker_heading_reset_parity PRIVATE
    ${SKETCH_DIR} ${HAL_MOCKS_DIR} ${PATH_MOCKS_DIR} ${TEST_DIR} ${FIXTURE_ROUTES_DIR})
add_fixture_routes_force_include(test_path_tracker_heading_reset_parity)   # FIRST (ROUTES_H)
add_path_force_includes(test_path_tracker_heading_reset_parity)
add_test(NAME path_tracker_heading_reset_parity COMMAND test_path_tracker_heading_reset_parity)

# =========================================================================================
# Test executable #24 — `tshape_profile_dump` + freshness gate (issue #130, T3).
#
# Host golden-vector dump for the header-only, Arduino-free `TShapeProfilePolicy.h` (T2).
# Includes ONLY the policy header plus the REAL `Config::TSHAPE_*` / `PWM_MIN_MAGNITUDE`
# constants via `config/ConfigGlobal.h` + `config/ConfigDrive.h` directly — both are pure
# `<stdint.h>`-only `constexpr` headers with ZERO Arduino/MatrixMiniR4 surface, so they are
# host-includable as-is with no mock indirection (unlike the `Config.h` umbrella, which pulls
# in `<Arduino.h>`). No SUT `.cpp`, no `MOCK_SOURCES`, no force-includes needed — the dump TU
# is entirely self-contained.
#
# This is DELIBERATELY not a self-asserting ctest target on its own — the executable just
# prints JSON to stdout. The actual ctest is `tshape_profile_freshness` below, which drives
# `tshape_profile_freshness.cmake`: it re-runs this binary fresh into a build-tree temp file
# and BYTE-COMPARES it against the committed
# `tools/path_planner/tests/fixtures/tshape_profile_goldens.json` (the `routes.h`
# regenerate-then-diff freshness-gate pattern, ctest-native so it also runs locally, not just
# in a dedicated CI job). A firmware retune that changes `Config::TSHAPE_*` — or any change to
# the policy's arithmetic — changes this binary's stdout and fails the ctest until the fixture
# is regenerated and recommitted. Consumed by the Python parity test (T6 / TC1).
# =========================================================================================
set(TSHAPE_DUMP_DIR ${TEST_DIR}/tshape_profile_dump)

add_executable(tshape_profile_dump
    ${TSHAPE_DUMP_DIR}/tshape_profile_dump.cpp
)
target_include_directories(tshape_profile_dump PRIVATE ${SKETCH_DIR})

set(TSHAPE_GOLDEN_FIXTURE
    ${SKETCH_DIR}/../tools/path_planner/tests/fixtures/tshape_profile_goldens.json)

add_test(NAME tshape_profile_freshness
    COMMAND ${CMAKE_COMMAND}
            -DDUMP_EXE=$<TARGET_FILE:tshape_profile_dump>
            -DGOLDEN_FILE=${TSHAPE_GOLDEN_FIXTURE}
            -DOUT_FILE=${CMAKE_CURRENT_BINARY_DIR}/tshape_profile_goldens.fresh.json
            -P ${TSHAPE_DUMP_DIR}/tshape_profile_freshness.cmake
)

# =========================================================================================
# Test executable #25 — `chain_speed_dump` + freshness gate (ADR-059, T4).
#
# Host golden-vector dump for the header-only, Arduino-free `ChainSpeedPolicy.h` (T4's two-pass
# collinear-run velocity planner). Includes ONLY the policy header plus the REAL
# `Config::PWM_MIN_MAGNITUDE` / `PWM_MAX` / `TSHAPE_ACCEL_PWM_PER_CM` / `TSHAPE_MIN_MOVE_CM` /
# `TSHAPE_START_SPEED` / `SPEED_NORMAL` constants (all pure `<stdint.h>`-only `constexpr`), so it
# needs NOTHING on the include path except `${SKETCH_DIR}` — the TU is entirely self-contained.
#
# This is DELIBERATELY not a self-asserting ctest target on its own — the executable just prints
# JSON to stdout. The actual ctest is `chain_speed_freshness` below, which drives
# `chain_speed_freshness.cmake`: it re-runs this binary fresh into a build-tree temp file and
# BYTE-COMPARES it against the committed
# `tools/path_planner/tests/fixtures/chain_speed_goldens.json` (the `routes.h` /
# `tshape_profile` regenerate-then-diff freshness-gate pattern, ctest-native so it also runs
# locally, not just in a dedicated CI job). A firmware retune that changes
# `Config::TSHAPE_ACCEL_PWM_PER_CM` / `PWM_MIN_MAGNITUDE` — or any change to the policy's
# arithmetic — changes this binary's stdout and fails the ctest until the fixture is regenerated
# and recommitted. Consumed by the Python parity mirror `chain_speed.py` (T5 / AC6 / TC1).
# =========================================================================================
set(CHAIN_SPEED_DUMP_DIR ${TEST_DIR}/chain_speed_dump)

add_executable(chain_speed_dump
    ${CHAIN_SPEED_DUMP_DIR}/chain_speed_dump.cpp
)
target_include_directories(chain_speed_dump PRIVATE ${SKETCH_DIR})

set(CHAIN_SPEED_GOLDEN_FIXTURE
    ${SKETCH_DIR}/../tools/path_planner/tests/fixtures/chain_speed_goldens.json)

add_test(NAME chain_speed_freshness
    COMMAND ${CMAKE_COMMAND}
            -DDUMP_EXE=$<TARGET_FILE:chain_speed_dump>
            -DGOLDEN_FILE=${CHAIN_SPEED_GOLDEN_FIXTURE}
            -DOUT_FILE=${CMAKE_CURRENT_BINARY_DIR}/chain_speed_goldens.fresh.json
            -P ${CHAIN_SPEED_DUMP_DIR}/chain_speed_freshness.cmake
)
