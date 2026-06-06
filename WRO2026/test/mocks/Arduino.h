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
 *   - `delay` (advances the mock clock), `millis`, `yield`
 *   - `F()` flash-string macro (host: returns its argument unchanged)
 *   - a `Serial` singleton with `print`/`println` no-op overloads (`const char*`
 *     plus the common numeric types)
 *
 * The mock `millis()` is incremented by a test-controllable global so the
 * `waitUntilDistanceReached` / `waitUntilIntersection` time-bounded paths can
 * be exercised deterministically. `delay(ms)` additionally ADVANCES that clock
 * by `ms`, so production blocking loops that poll `millis()` against a wall-clock
 * timeout while sleeping with plain `delay()` (e.g. the manipulator belt-homing
 * loop) terminate on host instead of spinning forever.
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

inline void delay(uint32_t ms)
{
    // Advance the mock clock so production blocking loops that sleep with a plain
    // `delay()` and bound themselves on `millis()` (e.g. the manipulator belt-home
    // timed run) make forward progress and terminate on host. The advance is
    // monotonic, matching real Arduino behavior where wall-clock time elapses
    // across a `delay()`. Tests that need an exact clock can still pin it with
    // `mockResetMillis()` / `mockAdvanceMillis()`.
    g_mockMillis += ms;
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
// `LineFollower.cpp` calls `Serial.println(F("…"))` (in the fallback warning paths
// of `waitUntilDistanceReached` / `waitUntilIntersection`). The production
// `Manipulator.cpp` warning paths additionally `print()` an index / value
// alongside the message, so the stub carries matching numeric `print(...)`
// overloads as well as the `println(...)` set.
//
// `uint16_t` arguments promote to `int` and resolve to the `int` overload, so a
// distinct `uint16_t` overload is intentionally omitted — adding one would be
// ambiguous against `int` on every host where `int` is wider than 16 bits.
// ---------------------------------------------------------------------------

class MockSerial
{
public:
    void begin(unsigned long /*baud*/) {}

    void print(const char* /*s*/) {}
    void print(int /*n*/) {}
    void print(unsigned int /*n*/) {}
    void print(long /*n*/) {}
    void print(unsigned long /*n*/) {}
    void print(float /*v*/, int /*decimals*/ = 2) {}

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
