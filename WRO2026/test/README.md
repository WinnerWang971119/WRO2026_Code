# WRO2026 Host-Compile Test Harness

Cross-platform host-compile harness for C++ unit tests that exercise the
production `WRO2026/LineFollower.{h,cpp}` state machine without an Arduino
in the loop. Built for the **T10 acceptance criteria** of the
[pass-through collinear intersections plan](../../docs/plans/2026-05-23-pass-through-collinear-intersections.md):

| ctest name                          | Executable                              | Acceptance criteria | What it covers                                                                                                               |
|-------------------------------------|-----------------------------------------|---------------------|------------------------------------------------------------------------------------------------------------------------------|
| `line_follower_overshoot`           | `test_line_follower_overshoot`          | AC6, AC9            | T10a (`setOvershootMode` dead-reckon), T10b (`setBrakeOnGate(false)` no self-disable), T10d (junction-freeze interaction)    |
| `path_tracker_passthrough_on`       | `test_path_tracker_passthrough_on`      | AC7b                | `PATH_PASS_THROUGH_ENABLED_DEFAULT=true` build — collinear fixture passes through                                            |
| `path_tracker_passthrough_off`      | `test_path_tracker_passthrough_off`     | AC7b                | `PATH_PASS_THROUGH_ENABLED_DEFAULT=false` build — same fixture, legacy stop-and-go                                           |
| `path_tracker_bidirectional_on`     | `test_path_tracker_bidirectional_on`    | TC12, TC13, TC14c   | `BIDIRECTIONAL_AUTO_ENABLED_DEFAULT=true` — REVERSE_UNAVAILABLE fault, Mission modifier no-op, kill-switch propagation       |
| `path_tracker_bidirectional_off`    | `test_path_tracker_bidirectional_off`   | TC14c               | `BIDIRECTIONAL_AUTO_ENABLED_DEFAULT=false` — same fixture picks FRONT, confirming toggle genuinely controls runtime behavior  |
| `path_tracker_stop_at_line`         | `test_path_tracker_stop_at_line`        | TC7–TC10, T16       | STOP_AT_LINE FSM slice: re-anchor-on-success (TC7), DRIVE_FAULT-on-no-line (TC8), call-flag / BlindMode precedence (TC9/10)  |
| `stop_at_line_armmath`              | `test_stop_at_line_armmath`             | TC11, T16           | Pure-math arm-window (`stopAtLineArmed`) and N-of-N debounce (`debounceStep`) from `WROMath.h`                               |
| `test_tasks_dispatch`               | `test_tasks_dispatch`                   | AC2–AC10            | Tasks framework dispatch: registry, RAII guard, TC1–TC3, TC5–TC12; real `TaskArgs`/`TaskRegistryEntry` types from production  |
| `tasks_overflow_compile_fail`       | `test_tasks_dispatch_overflow`          | AC9 (negative)      | Build-that-must-fail: asserts `MAX_TASK_ARG_BYTES` `static_assert` fires when arg pack is oversized (ctest PASS_REGULAR_EXPRESSION on the `MAX_TASK_ARG_BYTES` static_assert diagnostic — passes only if the build emits that message, not on any non-zero exit) |

## Quick start

```
cmake -S WRO2026/test -B build/test
cmake --build build/test
ctest --test-dir build/test --output-on-failure
```

Expected output:

```
    Start 1: line_follower_overshoot
1/9 Test #1: line_follower_overshoot ..............   Passed    0.01 sec
    Start 2: path_tracker_passthrough_on
2/9 Test #2: path_tracker_passthrough_on ..........   Passed    0.01 sec
    Start 3: path_tracker_passthrough_off
3/9 Test #3: path_tracker_passthrough_off .........   Passed    0.01 sec
    Start 4: path_tracker_bidirectional_on
4/9 Test #4: path_tracker_bidirectional_on ........   Passed    0.01 sec
    Start 5: path_tracker_bidirectional_off
5/9 Test #5: path_tracker_bidirectional_off .......   Passed    0.01 sec
    Start 6: path_tracker_stop_at_line
6/9 Test #6: path_tracker_stop_at_line ............   Passed    0.01 sec
    Start 7: stop_at_line_armmath
7/9 Test #7: stop_at_line_armmath .................   Passed    0.01 sec
    Start 8: test_tasks_dispatch
8/9 Test #8: test_tasks_dispatch ..................   Passed    0.01 sec
    Start 9: tasks_overflow_compile_fail
9/9 Test #9: tasks_overflow_compile_fail ..........   Passed    0.12 sec
100% tests passed, 0 tests failed out of 9
```

### Platform notes

- **Linux / Ubuntu CI:** any modern `g++` (7+) or `clang++` is sufficient. CMake 3.16+.
- **Windows:** MSVC 19.20+ via "Developer Command Prompt for VS 2022" (run
  `vcvars64.bat` first, then the cmake/ctest commands above). The harness has
  been validated against MSVC 19.44 + CMake 3.27 + Ninja 1.11 on Windows 11.
- **macOS:** AppleClang 12+ via Xcode CLT.

No platform-specific source paths or compiler flags — CMake selects the
generator automatically.

## What's mocked, what's real

**Real** production code under test (compiled unchanged, full assertion behavior):
- `WRO2026/LineFollower.h` + `WRO2026/LineFollower.cpp`
- `WRO2026/WROMath.h` (free helpers — pulled in via the mock `Drive.h`)
- `WRO2026/Config.h` (the actual Config namespace, including the
  `PATH_PASS_THROUGH_ENABLED_DEFAULT` `#ifndef`-guarded constant)

**Mocked** (under `mocks/`, shadowing the production headers via matching include guards
and `/FI` (MSVC) / `-include` (gcc/clang) force-include):

| Mock                 | Source under shadow         | Surface stubbed                                            |
|----------------------|-----------------------------|------------------------------------------------------------|
| `mocks/Arduino.h`    | toolchain `<Arduino.h>`     | `millis`, `delay`, `yield`, `Serial`, `F()`, `PI`, `using std::abs;` |
| `mocks/MatrixMiniR4.h` | vendor `<MatrixMiniR4.h>` | (empty — production deps reach into it through other headers we mock) |
| `mocks/Drive.h`      | `WRO2026/Drive.h`           | `setSpeed`, `stop`, `getEncL`, `getEncR`, `isIdle` — every call records via `mockRecord()`; encoders settable via `mockAdvanceEncoders` |
| `mocks/WROLineArray.h` | `WRO2026/WROLineArray.h`  | `getError`, `isOnline`, `getJunctionType`, `setReverseActive` — values settable via `mockSet*` |
| `mocks/Intersection.h` | `WRO2026/Intersection.h`  | `update`, `isAtIntersection` (read-and-clear), `getEdgeCount`, `getLastType` |

The include-shadow trick is the load-bearing mechanism: each mock header
re-uses the SAME include guard as its production counterpart (e.g.
`#ifndef DRIVE_H`), so once the mock is force-included at compile start, any
subsequent `#include "Drive.h"` from `LineFollower.h` short-circuits.

## Adding a new test

1. Create `WRO2026/test/test_<name>.cpp` with a `main()` that follows the
   pattern in `test_line_follower_overshoot.cpp` (use `TestAssert.h` macros,
   call `resetWorld()` at the top of each scenario).
2. Add an `add_executable(...)` + `add_test(...)` pair to `CMakeLists.txt`,
   listing the production sources you're exercising as additional SUT inputs.
3. Re-run `cmake -S WRO2026/test -B build/test && cmake --build build/test`.

The mock surface is intentionally minimal — extend `mocks/Drive.h` /
`mocks/WROLineArray.h` etc. only when a new test requires a method not yet
stubbed. Each method addition is one declaration + a `mockRecord` counter.

## CI integration

This harness is wired into `.github/workflows/ci.yml` as the `host-tests`
job (display name: "Host tests"). It runs on every push to `main` and on
every pull request, using the built-in `g++` on `ubuntu-latest`. The three
steps it executes are:

```
cmake -S WRO2026/test -B build/test
cmake --build build/test
ctest --test-dir build/test --output-on-failure
```

The job gates merging: a failing test or a compile error in any registered
ctest case blocks the PR. All build variants (`PATH_PASS_THROUGH_ENABLED`,
`BIDIRECTIONAL_AUTO_ENABLED`, and the compile-fail overflow target) are
exercised as separate ctest cases in a single job — no matrix needed.

## Limitations

- **No PathTracker FSM coverage.** T10c is exercised behaviorally by directly
  executing the LineFollower arming chain PathTracker would issue (the
  canonical collinear-fixture peek logic is inlined in
  `test_path_tracker_passthrough.cpp`). A full PathTracker FSM test would
  require mocking `FieldMap`, `Localization`, and `Routes::` — out of scope
  for this PR (would balloon the harness ~5×).
- **No timing / debounce coverage.** The mock `millis()` is a test-controlled
  counter; real-world debounce and sensor noise are HW3 territory (Phase 3).
- **No sync-helper coverage** (`waitUntilDistanceReached` /
  `waitUntilIntersection`). The tests tick `LineFollower.update()` directly.
  Adding sync-helper tests would require driving the mock `Drive.isIdle()` /
  `Intersection.isAtIntersection()` through scripted sequences — feasible
  with the existing mocks but not implemented yet.
