/**
 * @file Arduino.h (HAL MOCK — hal_mocks/)
 * @brief Self-contained forward to the shared facade-free Arduino mock.
 * @author WRO 2026
 *
 * @details
 * `hal_mocks/` must be self-contained: the `test_mock_hal` CMake target adds only
 * `hal_mocks/` to its include path (mirroring the `ctps_mocks/` precedent, ADR-033).
 * `mocks/` must NOT appear on that path — doing so would pull the old facade headers
 * (e.g. `mocks/MatrixMiniR4.h`, `mocks/Drive.h`) into the same TU as the new HAL
 * mocks, violating the ODR mutual-exclusion invariant (#105).
 *
 * The shared `mocks/Arduino.h` is facade-free (it declares only the Arduino core
 * runtime shim: `MockSerial`, `MockHardwareSerial`, `millis`, `delay`, `F()`, etc.)
 * so it is safe to reuse here without any risk of pulling in the old facade. The
 * forwarded header's own include guard (`WRO_TEST_MOCK_ARDUINO_H`) prevents double
 * inclusion; no second guard is needed on this forwarding file.
 */

// hal_mocks/ is self-contained — all consumers include this file, never mocks/ directly.
#include "../mocks/Arduino.h"
