/**
 * @file Arduino.h (HOST MOCK)
 * @brief Host-compile shim for the Arduino core runtime.
 *
 * Provides just enough of the Arduino API surface for `LineFollower.cpp` to compile
 * on a host x86 toolchain (MSVC / gcc / clang) without any Arduino hardware
 * dependency. This file replaces `<Arduino.h>` because `WRO2026/test/mocks/` is
 * the FIRST entry on the include path.
 *
 * Only the symbols actually referenced from `LineFollower.cpp`, `Config.h`,
 * `WROMath.h`, and the mock Drive/LineArray/Intersection are stubbed:
 *
 *   - basic fixed-width integer types (via `<stdint.h>`)
 *   - `PI`, `abs()` macros
 *   - `delay`, `millis`, `yield` (no-op / monotonic counter)
 *   - `F()` flash-string macro (host: returns its argument unchanged)
 *   - a `Serial` singleton with `print`/`println(const char*)` no-ops
 *
 * The mock `millis()` is incremented by a test-controllable global so the
 * `waitUntilDistanceReached` / `waitUntilIntersection` time-bounded paths can
 * be exercised deterministically.
 */

#ifndef WRO_TEST_MOCK_ARDUINO_H
#define WRO_TEST_MOCK_ARDUINO_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <cstdlib>    // std::abs — pulled in BEFORE the `abs` macro below would have masked it.

// ---------------------------------------------------------------------------
// Math constants used by `WROMath.h` (real Arduino.h defines PI).
// ---------------------------------------------------------------------------

#ifndef PI
#define PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
// `abs(x)` — Arduino's avr-libc defines `abs` as a function-like MACRO. On
// host MSVC / clang the same macro mangles the `std::abs` overloads in
// `<cstdlib>` (the standard header is included transitively by the test
// mains' `<cstdio>` etc., and re-tokenising the parameter list with the
// macro produces a syntax error). Instead, we pull `std::abs` into the
// global namespace via a `using` declaration. `LineFollower.cpp` calls
// `abs(int32_t)` and `abs(int16_t)` — both resolve to the `<cstdlib>`
// integer overloads. Float `abs(float)` resolves via `<cmath>`'s overload
// in the global namespace (clang/MSVC headers do this for us).
// ---------------------------------------------------------------------------

using std::abs;

// ---------------------------------------------------------------------------
// Time / loop-yield primitives.
// ---------------------------------------------------------------------------

// Test-visible mock clock. Tests advance it manually via `mockAdvanceMillis()`.
extern uint32_t g_mockMillis;

inline uint32_t millis()
{
    return g_mockMillis;
}

inline void delay(uint32_t /*ms*/)
{
    // No-op on host. Tests don't depend on wall-clock delay.
}

inline void yield()
{
    // No-op on host.
}

// Test helpers — used directly by test mains.
inline void mockResetMillis()
{
    g_mockMillis = 0;
}

inline void mockAdvanceMillis(uint32_t deltaMs)
{
    g_mockMillis += deltaMs;
}

// ---------------------------------------------------------------------------
// `F()` flash-string macro — on host we just pass through the C-string literal.
// ---------------------------------------------------------------------------

#ifndef F
#define F(string_literal) (string_literal)
#endif

// ---------------------------------------------------------------------------
// `Serial` singleton — host stub.
//
// `LineFollower.cpp` calls `Serial.println(F("…"))` exactly twice (in the
// fallback warning paths of `waitUntilDistanceReached` / `waitUntilIntersection`).
// We provide a class with overloads accepting `const char*` and that's it.
// ---------------------------------------------------------------------------

class MockSerial
{
public:
    void begin(unsigned long /*baud*/) {}
    void print(const char* /*s*/) {}
    void println() {}
    void println(const char* /*s*/) {}
    void println(int /*n*/) {}
    void println(unsigned int /*n*/) {}
    void println(long /*n*/) {}
    void println(unsigned long /*n*/) {}
    void println(float /*v*/, int /*decimals*/ = 2) {}
};

extern MockSerial Serial;

#endif    // WRO_TEST_MOCK_ARDUINO_H
