/**
 * @file mocks.cpp
 * @brief Global singleton storage + the mock `millis()` clock for the host-compile harness.
 *
 * Single TU defines `Drive`, `LineArray`, `Intersection`, `MiniR4`, `Serial`, and the
 * `g_mockMillis` clock. Each test main links against this file.
 */

#include "Arduino.h"
#include "Drive.h"
#include "WROLineArray.h"
#include "Intersection.h"
#include "MatrixMiniR4.h"

// ---------------------------------------------------------------------------
// Time / Serial globals.
// ---------------------------------------------------------------------------

uint32_t g_mockMillis = 0;
MockSerial Serial;

// Streaming `Serial1` (UART RX) mock consumed by `Vision.cpp`, plus the per-`delay()`
// tick counter (TC18). Both are declared `extern` in the mock `Arduino.h`; this TU is
// their single definition, alongside `Serial` / `g_mockMillis`.
MockHardwareSerial Serial1;
uint32_t           mockTickCount = 0;

// ---------------------------------------------------------------------------
// Module singletons — match the `extern` declarations in each mock header.
// ---------------------------------------------------------------------------

DriveClass        Drive;
WROLineArrayClass LineArray;
IntersectionClass Intersection;
MatrixMiniR4      MiniR4;
