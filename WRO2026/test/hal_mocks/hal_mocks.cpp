/**
 * @file hal_mocks.cpp
 * @brief Global singleton storage for the `hal_mocks/` host-test HAL (epic #107 / issue #105).
 *
 * Single translation unit that defines every global object declared `extern` in the
 * `hal_mocks/` headers so that `test_mock_hal` (and any future target that links this
 * file) satisfies all external-linkage ODR requirements at link time.
 *
 * Singletons defined here:
 *   - `DriveClass        Drive`       — declared `extern` in `hal_mocks/Drive.h`.
 *   - `EventTraceClass   EventTrace`  — declared `extern` in `hal_mocks/EventTrace.h`.
 *   - `MatrixMiniR4      MiniR4`      — declared `extern` in `hal_mocks/MatrixMiniR4.h`.
 *   - `uint32_t          g_mockMillis` — declared `extern` in `mocks/Arduino.h`
 *                                        (reached via `hal_mocks/Arduino.h`).
 *   - `MockSerial        Serial`       — declared `extern` in `mocks/Arduino.h`.
 *
 * @note `mmL` is declared `inline MockMMLower mmL;` inside `hal_mocks/MatrixMiniR4.h`
 *       (a C++17 inline variable). It has a single definition across every TU that
 *       includes that header without any entry in this file — do NOT define it here.
 *
 * @note `MockHardwareSerial Serial1` and `uint32_t mockTickCount` are conditionally
 *       added below ONLY if a link error reports them undefined. The Vision mock
 *       infrastructure that odr-uses `Serial1` / `mockTickCount` lives in `mocks/`
 *       and is not linked by the initial `test_mock_hal` target; omitting them now
 *       keeps this file minimal and avoids a multiply-defined symbol if a future
 *       target merges both mock sets.
 *
 * Style: mirrors `mocks/mocks.cpp` exactly (section comment blocks, ordering:
 * Arduino globals first, then module singletons alphabetically).
 */

#include "Arduino.h"
#include "Drive.h"
#include "EventTrace.h"
#include "MatrixMiniR4.h"

// ---------------------------------------------------------------------------
// Arduino core globals — declared `extern` in `mocks/Arduino.h` (reached via
// `hal_mocks/Arduino.h`). Defined here so `test_mock_hal` needs no dependency
// on `mocks/mocks.cpp` (which would pull the old facade singletons).
// ---------------------------------------------------------------------------

uint32_t  g_mockMillis = 0;
MockSerial Serial;

// ---------------------------------------------------------------------------
// Module singletons — match the `extern` declarations in each hal_mocks/ header.
// ---------------------------------------------------------------------------

DriveClass      Drive;
EventTraceClass EventTrace;
MatrixMiniR4    MiniR4;
