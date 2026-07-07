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
 * @note `MockHardwareSerial Serial1` and `uint32_t mockTickCount` are defined here
 *       because the shared mock `delay()` helper odr-uses them. Targets that link the
 *       real `Tasks.cpp` may now call that helper even when they otherwise do not touch
 *       the Vision surface, so these Arduino-core globals belong with the rest of the
 *       shared mock runtime storage.
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
MockHardwareSerial Serial1;
uint32_t mockTickCount = 0;

// ---------------------------------------------------------------------------
// Module singletons — match the `extern` declarations in each hal_mocks/ header.
// ---------------------------------------------------------------------------

DriveClass      Drive;
EventTraceClass EventTrace;
MatrixMiniR4    MiniR4;
