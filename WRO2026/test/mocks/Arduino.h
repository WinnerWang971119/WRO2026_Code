/**
 * @file Arduino.h (HOST MOCK)
 * @brief Host-compile shim for the Arduino core runtime.
 *
 * Provides just enough of the Arduino API surface for the host-linked production
 * sources (`LineFollower.cpp`, and — since #106 — the real `Drive.cpp` /
 * `PathTracker.cpp`) to compile on a host x86 toolchain (MSVC / gcc / clang) without
 * any Arduino hardware dependency. This file replaces `<Arduino.h>` because
 * `WRO2026/test/mocks/` is the FIRST entry on the include path.
 *
 * Only the symbols actually referenced from `LineFollower.cpp`, `Config.h`,
 * `WROMath.h`, the real `Drive.cpp` / `PathTracker.cpp`, and the mock
 * Drive/LineArray/Intersection are stubbed:
 *
 *   - basic fixed-width integer types (via `<stdint.h>`)
 *   - `PI`, `abs()` macros
 *   - `delay` (advances the mock clock), `millis`, `yield`
 *   - `F()` flash-string macro (host: casts to `const __FlashStringHelper*`, as on real Arduino)
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

#include <stddef.h>   // size_t — `mockSerial1Feed(const uint8_t*, size_t)` signature.
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <cstdlib>    // std::abs — pulled in BEFORE the `abs` macro below would have masked it.
#include <deque>      // mock Serial1 RX byte queues (released + drip-held pending).
#include <string>     // std::string capture buffer + std::to_string for integer overloads.

// ---------------------------------------------------------------------------
// Math constants used by `WROMath.h` (real Arduino.h defines PI).
// ---------------------------------------------------------------------------

#ifndef PI
#define PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
// `__FlashStringHelper` — incomplete forward declaration (#106).
//
// Real Arduino declares this as an opaque class used only through pointers: a
// flash-string literal wrapped by `F(...)` is a `const __FlashStringHelper*`,
// and `Print::print(const __FlashStringHelper*)` reads it from PROGMEM. The
// first host-linked production `.cpp`s that use this surface are `Drive.cpp`
// and `PathTracker.cpp` (#106): two PathTracker diagnostic helpers take a
// `const __FlashStringHelper* site/adrTag` parameter, and the `F()` macro below
// now yields that pointer type so those parameters match their call args. Since
// the type is only ever used as a pointer, an incomplete (forward-declared)
// type is sufficient — no members are needed.
// ---------------------------------------------------------------------------

class __FlashStringHelper;

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

// Forward declaration — defined after the mock `Serial1` class below. `delay()` is
// inline-defined here but its body odr-uses `_mockSerial1OnDelayTick()`; an inline
// function may be defined later in the same header because the call site lives in a
// separate TU (each test main / `Vision.cpp`) that sees BOTH full definitions before
// it odr-uses `delay()`. This is the only ordering subtlety the Serial1 mock adds.
inline void _mockSerial1OnDelayTick();

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

    // Drive the mock `Serial1` "tick": advance the tick counter (TC18) and, when
    // drip-feed is enabled, release the next batch of held bytes into the readable
    // queue. `Vision::readMosaic`'s wait loop calls `delay(small)` every iteration
    // (mirroring `Drive::_drainBlocking`, WRO2026/Drive.cpp), so coupling the drip
    // pump to `delay()` makes the wait loop and timeout tests progress and terminate
    // deterministically — no infinite spin. See the Serial1 mock block below.
    _mockSerial1OnDelayTick();
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
// `F()` flash-string macro (#106).
//
// Real Arduino defines `F(s)` as `reinterpret_cast<const __FlashStringHelper*>(s)`
// so a wrapped literal carries the flash-string TYPE, which selects the PROGMEM
// `Print::print` overload. We now mirror that exactly (instead of the former
// passthrough `(string_literal)`) because `PathTracker.cpp` passes `F("...")`
// into helper params typed `const __FlashStringHelper*` (e.g. `_printReverse-
// Unavailable`, `_printOvershootTrimUnderflow`); with the old passthrough the
// arg was `const char*` and the param type would not match. The cast back to
// `const char*` happens in `MockSerial`'s `__FlashStringHelper*` overloads
// below, so captured text is byte-for-byte identical to the old `const char*`
// path. No test `.cpp` consumes an `F(...)` result as a `const char*` (verified),
// so this type change is regression-safe. The macro is defined only here — no
// real `<Arduino.h>` competes for the name.
// ---------------------------------------------------------------------------

#ifndef F
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper*>(string_literal))
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
//
// #106 additions (real Drive.cpp / PathTracker.cpp link):
//   - `print/println(const __FlashStringHelper*)`: a flash-string wrapped by the
//     `F(...)` macro now has this pointer type. The overload reinterpret_casts it
//     back to `const char*` and appends IDENTICALLY to the `const char*` overload
//     (same null-guard, same bytes), so every existing `Serial.print(F("..."))`
//     captures exactly the text it did before — the key no-regression property.
//   - radix constants `DEC/HEX/OCT/BIN` + `print/println(uint8_t, int base)`:
//     `Drive.cpp:1788` does `Serial.println(rc, HEX)` where `rc` is `uint8_t`,
//     formatting the lower-MCU result code in hex. ONLY the exact `uint8_t` base
//     overload is added — a generic `(int, int)` or `(float, int)` base overload
//     would make `(rc, HEX)` ambiguous (uint8_t->int and uint8_t->float are both
//     conversions, and `print(float, int=2)` already exists). The 1-arg integer
//     and float overloads above are left untouched.
// ---------------------------------------------------------------------------

// Radix bases for the `(value, base)` print overloads. Real Arduino exposes these
// as plain ints (`#define`s in `Print.h`); an unnamed enum gives the same int
// values without polluting the namespace with macros. `Serial.println(rc, HEX)`
// (Drive.cpp:1788) is the sole current user.
enum
{
    DEC = 10,
    HEX = 16,
    OCT = 8,
    BIN = 2
};

// Format `value` as text in `base` (2/8/10/16, like Arduino's `Print`). Lower-case
// hex digits, no `0x` prefix — the caller emits its own prefix (Drive.cpp prints
// "rc=0x" then `println(rc, HEX)`). `base <= 1` falls back to decimal. Free helper
// (not a member) so both `MockSerial` base overloads share one implementation.
inline std::string mockFormatRadix(unsigned long value, int base)
{
    if (base <= 1)
    {
        return std::to_string(value);
    }
    if (value == 0)
    {
        return std::string("0");
    }
    static const char digits[] = "0123456789abcdef";
    std::string       out;
    while (value != 0)
    {
        out.push_back(digits[value % static_cast<unsigned long>(base)]);
        value /= static_cast<unsigned long>(base);
    }
    return std::string(out.rbegin(), out.rend());
}

class MockSerial
{
public:
    void begin(unsigned long /*baud*/) {}

    // --- print: append text, no trailing newline. ---
    void print(const char* s) { if (s != nullptr) _captured.append(s); }
    // Flash-string overload (#106): `F("...")` is a `const __FlashStringHelper*`;
    // cast back to `const char*` and append IDENTICALLY to the `const char*` path
    // (same null-guard, same bytes) so captured text is byte-for-byte unchanged.
    void print(const __FlashStringHelper* s)
    {
        const char* p = reinterpret_cast<const char*>(s);
        if (p != nullptr)
        {
            _captured.append(p);
        }
    }
    void print(int n) { _captured.append(std::to_string(n)); }
    void print(unsigned int n) { _captured.append(std::to_string(n)); }
    void print(long n) { _captured.append(std::to_string(n)); }
    void print(unsigned long n) { _captured.append(std::to_string(n)); }
    void print(uint8_t n) { _captured.append(std::to_string(static_cast<unsigned>(n))); }
    // Radix overload (#106) — exact `uint8_t` match for `Serial.print(rc, HEX)`-style
    // calls. Only `uint8_t` is provided so `(rc, HEX)` is unambiguous (see header note).
    void print(uint8_t n, int base) { _captured.append(mockFormatRadix(n, base)); }
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
    // Flash-string overload (#106): mirrors `println(const char*)` after casting the
    // `F("...")` pointer back to `const char*` — same bytes, then a single '\n'.
    void println(const __FlashStringHelper* s)
    {
        const char* p = reinterpret_cast<const char*>(s);
        if (p != nullptr)
        {
            _captured.append(p);
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
    // Radix overload (#106) — the call that motivated this change: `Drive.cpp:1788`
    // `Serial.println(rc, HEX)`, `rc` typed `uint8_t`. Format in `base`, then '\n'.
    void println(uint8_t n, int base)
    {
        _captured.append(mockFormatRadix(n, base));
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

// ---------------------------------------------------------------------------
// `Serial1` singleton — streaming UART (RX) host mock for `Vision.cpp`.
//
// `Vision::readMosaic()` reads one mosaic frame from the HUE12 board over
// `Serial1` (the R4's hardware UART on D0/D1). On host there is no UART, so this
// mock lets a test FEED a fake byte stream that `readMosaic`'s bounded-resync
// parser consumes. It is `HardwareSerial`-like: `begin`/`available`/`read` plus
// no-op TX (`write`/`print`/`println`), modelled in the same style as the
// `Serial` (USB) mock above.
//
// Two RX queues back the mock:
//   - `_rx`      — bytes that are READABLE NOW (`available()` counts these, `read()`
//                  pops these).
//   - `_pending` — bytes fed while DRIP-FEED mode is ON; held back, then released
//                  into `_rx` in batches of `_dripPerTick` on each mock "tick"
//                  (i.e. each `delay()` call — see `_mockSerial1OnDelayTick()`).
//
// Drip-feed exists so timeout / tick-callback tests (TC16, TC18) actually exercise
// `readMosaic`'s wait loop: with drip-feed OFF (the DEFAULT) a fed frame is fully
// `available()` immediately and the read completes in one pass; with drip-feed ON
// the bytes trickle out per `delay()` tick, so a frame can be made to NEVER fully
// arrive within `VISION_READ_TIMEOUT_MS`, driving a deterministic, bounded timeout
// (no infinite spin, because `millis()` advances on every `delay()`).
//
// The tick counter (`mockTickCount`, below) increments on every mock tick so TC18
// can assert the wait loop actually iterated (> 0) during a drip-fed wait.
// ---------------------------------------------------------------------------

class MockHardwareSerial
{
public:
    /**
     * @brief Record the configured baud rate (the only observable effect on host).
     * @param baud The baud passed by `Vision::begin()` (T7 asserts `VISION_UART_BAUD`).
     */
    void begin(unsigned long baud) { _lastBaud = baud; }

    /**
     * @brief Number of bytes currently readable from the RX queue.
     * @return Count of released (immediately readable) bytes. Drip-held `_pending`
     *         bytes are NOT counted until a tick releases them.
     */
    int available() { return static_cast<int>(_rx.size()); }

    /**
     * @brief Pop and return the next readable RX byte.
     * @return The next byte (0..255), or -1 if the readable queue is empty — matching
     *         Arduino `Stream::read()` semantics.
     */
    int read()
    {
        if (_rx.empty())
        {
            return -1;
        }
        const uint8_t b = _rx.front();
        _rx.pop_front();
        return static_cast<int>(b);
    }

    // --- TX is unused by Vision; present so any code that writes still compiles. ---
    size_t write(uint8_t /*b*/) { return 1; }
    size_t write(const uint8_t* /*buf*/, size_t n) { return n; }
    void   print(const char* /*s*/) {}
    // Flash-string no-op (#106): defensive symmetry so `Serial1.print(F("..."))` still
    // compiles after the `F()` macro yields `const __FlashStringHelper*`. TX is unused.
    void   print(const __FlashStringHelper* /*s*/) {}
    void   print(int /*n*/) {}
    void   print(unsigned int /*n*/) {}
    void   print(long /*n*/) {}
    void   print(unsigned long /*n*/) {}
    void   print(uint8_t /*n*/) {}
    void   println() {}
    void   println(const char* /*s*/) {}
    void   println(const __FlashStringHelper* /*s*/) {}    // (#106) — see print() above.
    void   println(int /*n*/) {}
    void   println(unsigned int /*n*/) {}
    void   println(long /*n*/) {}
    void   println(unsigned long /*n*/) {}
    void   println(uint8_t /*n*/) {}

    // --- Test-only mock controls (free helpers below delegate here). ---

    /**
     * @brief Append bytes to the RX stream.
     * @param bytes Source buffer; ignored if null.
     * @param n     Number of bytes to append.
     * @note With drip-feed OFF (default) bytes land directly in the readable queue
     *       and are `available()` immediately. With drip-feed ON they land in the
     *       held `_pending` queue and are released per tick by `_dripTick()`.
     */
    void feed(const uint8_t* bytes, size_t n)
    {
        if (bytes == nullptr)
        {
            return;
        }
        std::deque<uint8_t>& dst = _dripEnabled ? _pending : _rx;
        for (size_t i = 0; i < n; ++i)
        {
            dst.push_back(bytes[i]);
        }
    }

    /**
     * @brief Empty BOTH RX queues (readable + drip-held). Used by flush tests and
     *        `resetWorld()`. Does not change the drip-feed mode or per-tick count.
     */
    void clearRx()
    {
        _rx.clear();
        _pending.clear();
    }

    /**
     * @brief Enable/disable drip-feed and set how many held bytes release per tick.
     * @param bytesPerTick Bytes moved from `_pending` to `_rx` on each mock tick.
     *                     `0` disables drip-feed (bytes fed thereafter are readable
     *                     immediately); any value `>= 1` enables it.
     * @note Switching drip-feed OFF does NOT auto-flush already-held `_pending`
     *       bytes — call `drainPending()` if a test wants them released at once.
     *       Switching it ON does not retroactively hold already-readable `_rx` bytes.
     */
    void setDripFeed(size_t bytesPerTick)
    {
        _dripEnabled = (bytesPerTick > 0);
        _dripPerTick = bytesPerTick;
    }

    /**
     * @brief Release the next drip batch (no-op unless drip-feed is enabled).
     *        Called once per mock tick from `_mockSerial1OnDelayTick()`.
     */
    void dripTick()
    {
        if (!_dripEnabled)
        {
            return;
        }
        for (size_t i = 0; i < _dripPerTick && !_pending.empty(); ++i)
        {
            _rx.push_back(_pending.front());
            _pending.pop_front();
        }
    }

    /**
     * @brief Move ALL remaining held bytes into the readable queue at once.
     *        Lets a test stop drip-feeding mid-frame and complete the read.
     */
    void drainPending()
    {
        while (!_pending.empty())
        {
            _rx.push_back(_pending.front());
            _pending.pop_front();
        }
    }

    /**
     * @brief Last baud passed to `begin()`; `0` if `begin()` was never called.
     * @return The recorded baud rate.
     */
    unsigned long lastBaud() const { return _lastBaud; }

private:
    std::deque<uint8_t> _rx;                 // readable now
    std::deque<uint8_t> _pending;            // drip-held, released per tick
    bool                _dripEnabled = false;
    size_t              _dripPerTick = 0;
    unsigned long       _lastBaud    = 0;
};

extern MockHardwareSerial Serial1;

// The firmware selects the HUE12 camera UART through `Config::VISION_SERIAL`, which resolves
// to `Serial2` on the UNO R4 WiFi (the D0/D1 header socket — see ConfigVision.h). On host there
// is exactly one mock UART, so alias `Serial2` onto it: an object-like macro rewriting `Serial2`
// -> `Serial1` at every use site, so `Vision.cpp` reads the very stream the tests feed through
// `mockSerial1Feed`. Nothing else on host references `Serial2`, so the rewrite is inert elsewhere.
#define Serial2 Serial1

// ---------------------------------------------------------------------------
// Mock "tick" counter — increments once per `delay()` call (each mock tick).
//
// `Vision::readMosaic`'s wait loop calls the registered tick callback and then
// `delay(small)` every iteration. TC18 registers a callback and drip-feeds a read,
// then asserts the loop actually iterated during the wait. The callback itself is
// production code (registered on `Vision`), so the mock cannot count it directly;
// instead it counts the `delay()` ticks that bracket every callback invocation —
// `mockTickCount > 0` after a drip-fed wait proves the loop spun. Reset between
// scenarios with `mockResetTickCount()`.
// ---------------------------------------------------------------------------

extern uint32_t mockTickCount;

inline void mockResetTickCount()
{
    mockTickCount = 0;
}

// ---------------------------------------------------------------------------
// Free helpers over the global `Serial1` — the test-facing mock API.
// (Names match the Vision plan's Test-data note: `mockSerial1Feed` /
// `mockSerial1Clear` + a drip-feed setter + a tick counter.)
// ---------------------------------------------------------------------------

/**
 * @brief Append `n` bytes to `Serial1`'s RX stream (readable now, or drip-held if
 *        drip-feed is enabled).
 */
inline void mockSerial1Feed(const uint8_t* bytes, size_t n) { Serial1.feed(bytes, n); }

/**
 * @brief Empty `Serial1`'s RX queues. Used by flush tests and `resetWorld()`.
 */
inline void mockSerial1Clear() { Serial1.clearRx(); }

/**
 * @brief Enable drip-feed and set the per-tick release count (0 disables it).
 * @param bytesPerTick Bytes released from the held queue on each `delay()` tick.
 */
inline void mockSerial1SetDripFeed(size_t bytesPerTick) { Serial1.setDripFeed(bytesPerTick); }

/**
 * @brief Release all currently held drip bytes into the readable queue at once.
 */
inline void mockSerial1DrainPending() { Serial1.drainPending(); }

/**
 * @brief One mock tick: bump the tick counter and release the next drip batch.
 *        Invoked from `delay()` (declared above `delay()`, defined here once both
 *        `Serial1` and `mockTickCount` are complete types).
 */
inline void _mockSerial1OnDelayTick()
{
    ++mockTickCount;
    Serial1.dripTick();
}

#endif    // WRO_TEST_MOCK_ARDUINO_H
