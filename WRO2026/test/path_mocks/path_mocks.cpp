/**
 * @file path_mocks.cpp
 * @brief Facade-globals storage for the PathTracker + Drive host-link targets (issue #106).
 *
 * Single translation unit that defines the global objects declared `extern` by the
 * `path_mocks/`-first include set so the five new PathTracker+Drive host-link targets
 * satisfy their external-linkage ODR requirements at link time. These targets compile
 * the REAL `Drive.cpp`, so `Drive` itself is NOT defined here — `Drive.cpp` is its sole
 * definer (defining it here too would multiply-define the symbol against real Drive.cpp).
 *
 * Singletons defined here:
 *   - `MatrixMiniR4      MiniR4`       — declared `extern` in `hal_mocks/MatrixMiniR4.h`.
 *   - `MockSerial        Serial`       — declared `extern` in `mocks/Arduino.h`
 *                                        (reached via `hal_mocks/Arduino.h`).
 *   - `uint32_t          g_mockMillis` — declared `extern` in `mocks/Arduino.h`.
 *   - `MockHardwareSerial Serial1`     — declared `extern` in `mocks/Arduino.h`.
 *   - `uint32_t          mockTickCount` — declared `extern` in `mocks/Arduino.h`.
 *   - `EventTraceClass   EventTrace`   — declared `extern` in `hal_mocks/EventTrace.h`.
 *
 * This is `hal_mocks/hal_mocks.cpp` MINUS `Drive` (real `Drive.cpp` owns it) PLUS
 * `Serial1` + `mockTickCount`. `Serial1` / `mockTickCount` are added HERE — and are
 * deliberately ABSENT from `hal_mocks.cpp` — because the real `Drive.cpp` compiles a
 * `delay()` whose mock body odr-uses the Vision/Arduino mock infrastructure (`Serial1`
 * via `_mockSerial1OnDelayTick()`, and `mockTickCount`). `hal_mocks.cpp`'s consumers
 * (`test_mock_hal` / `test_mission_dispatch`) do not link real `Drive.cpp`, so they omit
 * both; these targets do, so they require both here.
 *
 * @note `mmL` is declared `inline MockMMLower mmL;` inside `hal_mocks/MatrixMiniR4.h`
 *       (a C++17 inline variable). It has a single definition across every TU that
 *       includes that header without any entry in this file — do NOT define it here.
 *
 * @note The peripheral globals `LineArray` / `Intersection` / `LineFollower` are declared
 *       `inline` in the `path_mocks/` mock headers, so this file defines NONE of them.
 *
 * Style: mirrors `hal_mocks/hal_mocks.cpp` (section comment blocks, ordering: Arduino
 * globals first, then module singletons alphabetically).
 */

#include "Arduino.h"
#include "EventTrace.h"
#include "MatrixMiniR4.h"

// ---------------------------------------------------------------------------
// Arduino core globals — declared `extern` in `mocks/Arduino.h` (reached via
// `hal_mocks/Arduino.h`). `Serial1` + `mockTickCount` are added HERE because the
// real `Drive.cpp`'s compiled `delay()` odr-uses them (see file header).
// ---------------------------------------------------------------------------

uint32_t           g_mockMillis = 0;
MockSerial         Serial;
MockHardwareSerial Serial1;
uint32_t           mockTickCount = 0;

// ---------------------------------------------------------------------------
// Module singletons — match the `extern` declarations in each hal_mocks/ header.
// `Drive` is intentionally absent: the real `Drive.cpp` is its sole definer.
// ---------------------------------------------------------------------------

EventTraceClass EventTrace;
MatrixMiniR4    MiniR4;
