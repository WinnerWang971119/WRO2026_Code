/**
 * @file Arduino.h (CTPS HOST MOCK)
 * @brief Self-contained host-compile shim for the Arduino core runtime.
 * @author WRO 2026
 *
 * @details
 * Provides just enough of the Arduino API surface for the real
 * `Mission.cpp` + `Tasks.cpp` + `MissionScript.cpp` (and the CTPS recording
 * mocks) to compile on a host x86 toolchain (MSVC / gcc / clang) with NO
 * Arduino hardware dependency. This file shadows `<Arduino.h>` because the
 * `ctps_dump` build adds `ctps_mocks/` as its ONLY include directory.
 *
 * Deliberately self-contained: it pulls in nothing from the sibling
 * `WRO2026/test/mocks/` directory, so the dump's include path stays
 * `ctps_mocks/`-only (see the CTPS plan's "include dir is ctps_mocks/ only"
 * decision — otherwise `mocks/MatrixMiniR4.h` would shadow the recording one).
 *
 * Symbols stubbed (the exact surface the three real `.cpp` files reference):
 *   - fixed-width integer types + `size_t` (via `<stdint.h>` / `<stddef.h>`)
 *   - `snprintf` (via `<stdio.h>`), `memcpy` / `strcmp` (via `<string.h>`)
 *   - `PI`, `abs()` (the `<cstdlib>` overloads, hoisted into the global ns)
 *   - `delay` / `yield` no-ops
 *   - a test-controllable mock clock: `millis()` + `mockAdvanceMillis(...)`
 *   - `F()` flash-string macro (host: yields `const __FlashStringHelper*`, matching Arduino)
 *   - a `Serial` singleton — NO-OP `print` / `println` overloads (the dump
 *     emits no Serial text; the recording sink is the only output)
 *
 * The mock clock is the dump's instant-completion lever: `ctps_dump` advances
 * `g_ctpsMillis` each tick so Mission's WAIT / BEEP / LED `now - start >=
 * duration` gates trip on the next tick instead of hanging the dump.
 */

#ifndef CTPS_MOCK_ARDUINO_H
#define CTPS_MOCK_ARDUINO_H

#include <stddef.h>    // size_t
#include <stdint.h>
#include <stdio.h>     // snprintf (Mission.cpp's snprintf(buf, ...))
#include <stdlib.h>
#include <string.h>    // memcpy (Mission._packArgs), strcmp (Tasks.findTask / FieldMap)
#include <math.h>

#include <cstdlib>     // std::abs — pulled in BEFORE any `abs` macro could mask it.

// ---------------------------------------------------------------------------
// Math constants. Real Arduino.h defines PI; some sketch headers expect it.
// ---------------------------------------------------------------------------

#ifndef PI
#define PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
// `abs(x)` — Arduino's avr-libc ships `abs` as a function-like MACRO, which
// would mangle the `<cstdlib>` overloads on a host toolchain. Hoist the real
// `std::abs` overloads into the global namespace instead, so any `abs(...)`
// the sketch uses resolves to the standard overloads.
// ---------------------------------------------------------------------------

using std::abs;

// ---------------------------------------------------------------------------
// `__FlashStringHelper` — incomplete forward declaration.
// ---------------------------------------------------------------------------

class __FlashStringHelper;

// ---------------------------------------------------------------------------
// Time / loop-yield primitives.
// ---------------------------------------------------------------------------

/**
 * @brief Mock millisecond clock backing `millis()`. Defined in ctps_record.cpp.
 *
 * The dump advances this via `mockAdvanceMillis(...)` each tick so Mission's
 * duration gates complete deterministically (instant-completion).
 */
extern uint32_t g_ctpsMillis;

/** @brief Host `millis()` — returns the mock clock. */
inline uint32_t millis()
{
    return g_ctpsMillis;
}

/** @brief Advance the mock clock by @p deltaMs (called once per dump tick). */
inline void mockAdvanceMillis(uint32_t deltaMs)
{
    g_ctpsMillis += deltaMs;
}

/** @brief Reset the mock clock to zero. */
inline void mockResetMillis()
{
    g_ctpsMillis = 0;
}

/** @brief Host `delay()` — no-op (the dump never waits on wall-clock time). */
inline void delay(uint32_t /*ms*/)
{
}

/** @brief Host `yield()` — no-op. */
inline void yield()
{
}

// ---------------------------------------------------------------------------
// `F()` flash-string macro — host cast mirroring Arduino's flash-string type.
// ---------------------------------------------------------------------------

#ifndef F
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper*>(string_literal))
#endif

// ---------------------------------------------------------------------------
// `Serial` singleton — host NO-OP stub.
//
// The dump produces output only via the recording sink (ctps_record), so every
// Serial call is a no-op. The overload set covers exactly what the real
// Mission.cpp / Tasks.cpp / MissionScript.cpp emit:
//   - `print`/`println(const char*)`  — string literals via F(...)
//   - `print`/`println(uint8_t)`      — TaskArgs::u8[], Mission::_count,
//                                       FaultInfo::{fromNode,destNode}
// The wider integer overloads + the no-arg `println()` + `begin(unsigned long)`
// are retained for API symmetry so any future production print binds cleanly.
// No bare-`char` overload is added: `uint8_t == unsigned char`, so a `char`
// overload would make `print(uint8_t)` ambiguous.
// ---------------------------------------------------------------------------

/**
 * @brief No-op host Serial stub. Defined as the global `Serial` in ctps_record.cpp.
 */
class MockSerial
{
public:
    void begin(unsigned long /*baud*/) {}

    void print(const char* /*s*/) {}
    void print(const __FlashStringHelper* /*s*/) {}
    void print(int /*n*/) {}
    void print(unsigned int /*n*/) {}
    void print(long /*n*/) {}
    void print(unsigned long /*n*/) {}
    void print(uint8_t /*n*/) {}
    void print(float /*v*/, int /*decimals*/ = 2) {}

    void println() {}
    void println(const char* /*s*/) {}
    void println(const __FlashStringHelper* /*s*/) {}
    void println(int /*n*/) {}
    void println(unsigned int /*n*/) {}
    void println(long /*n*/) {}
    void println(unsigned long /*n*/) {}
    void println(uint8_t /*n*/) {}
    void println(float /*v*/, int /*decimals*/ = 2) {}
};

/** @brief Single global Serial instance — defined in ctps_record.cpp. */
extern MockSerial Serial;

#endif    // CTPS_MOCK_ARDUINO_H
