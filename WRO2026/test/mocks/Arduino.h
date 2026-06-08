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
 *   - a `Serial` singleton whose `print`/`println` overloads capture all
 *     written text into an inspectable buffer (`Serial.captured()`)
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
#include <string>     // std::string capture buffer + std::to_string for integer overloads.

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
// Originally a pure no-op shim for the two `Serial.println(F("…"))` calls in
// `LineFollower.cpp`. It now also CAPTURES everything written so host tests can
// assert on production Serial output (e.g. `Tasks.cpp` emits
// `"[Tasks] demoEcho a=7 b=42"`; `Mission.cpp` emits `"[Mission] start; steps=3"`).
// `Manipulator.cpp` only emits `println(const char*)` warning strings, so its
// output is captured by the `const char*` overload — no numeric print needed.
//
// Capture rules:
//   - Every `print(...)` overload appends its text to `_captured`.
//   - Every `println(...)` overload appends its text followed by a single '\n'.
//   - Integer overloads stringify via `std::to_string(...)`.
//   - `begin(unsigned long)` stays a no-op (baud rate is not observable output).
//
// Overload set mirrors the Arduino `Print` API and is kept SYMMETRIC between
// `print` and `println` so a call like `Serial.print(someUint8)` binds to the
// exact `print(uint8_t)` overload (no integral promotion, no ambiguity). The
// integer types here cover every value actually printed by the production
// callers: `const char*` (string literals via `F(...)`) and `uint8_t`
// (`TaskArgs::u8[]`, `Mission::_count`, `PathTracker::FaultInfo::{fromNode,
// destNode}`); the wider `int / unsigned int / long / unsigned long` siblings
// and the `float` overload are retained for symmetry with the pre-existing
// `println` set. A `uint16_t` argument promotes to `int` and binds the `int`
// overload, so no distinct `uint16_t` overload is needed. No `char` overload is
// added because no caller prints a bare `char` — adding one would make
// `print(uint8_t)` ambiguous since `uint8_t == unsigned char`.
// ---------------------------------------------------------------------------

class MockSerial
{
public:
    void begin(unsigned long /*baud*/) {}

    // --- print: append text, no trailing newline. ---
    void print(const char* s) { if (s != nullptr) _captured.append(s); }
    void print(int n) { _captured.append(std::to_string(n)); }
    void print(unsigned int n) { _captured.append(std::to_string(n)); }
    void print(long n) { _captured.append(std::to_string(n)); }
    void print(unsigned long n) { _captured.append(std::to_string(n)); }
    void print(uint8_t n) { _captured.append(std::to_string(static_cast<unsigned>(n))); }
    void print(float v, int /*decimals*/ = 2) { _captured.append(std::to_string(v)); }

    // --- println: append text, then a single '\n'. ---
    void println() { _captured.push_back('\n'); }
    void println(const char* s)
    {
        if (s != nullptr)
        {
            _captured.append(s);
        }
        _captured.push_back('\n');
    }
    void println(int n) { _captured.append(std::to_string(n)); _captured.push_back('\n'); }
    void println(unsigned int n) { _captured.append(std::to_string(n)); _captured.push_back('\n'); }
    void println(long n) { _captured.append(std::to_string(n)); _captured.push_back('\n'); }
    void println(unsigned long n) { _captured.append(std::to_string(n)); _captured.push_back('\n'); }
    void println(uint8_t n)
    {
        _captured.append(std::to_string(static_cast<unsigned>(n)));
        _captured.push_back('\n');
    }
    void println(float v, int /*decimals*/ = 2)
    {
        _captured.append(std::to_string(v));
        _captured.push_back('\n');
    }

    // --- Capture inspection (test-only). ---
    /**
     * @brief Accumulated text written via every print/println call.
     * @return Const reference to the capture buffer.
     */
    const std::string& captured() const { return _captured; }

    /**
     * @brief Reset the capture buffer (call between test cases).
     */
    void clearCapture() { _captured.clear(); }

private:
    std::string _captured;
};

extern MockSerial Serial;

#endif    // WRO_TEST_MOCK_ARDUINO_H
