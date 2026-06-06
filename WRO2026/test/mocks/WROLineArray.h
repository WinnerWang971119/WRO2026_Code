/**
 * @file WROLineArray.h (HOST MOCK)
 * @brief Minimal `WROLineArrayClass` mock — only the surface called by `LineFollower.cpp`.
 *
 * `LineFollower.cpp` and `Intersection.cpp` call:
 *   - `LineArray.getError()`         — weighted-center P+D error
 *   - `LineArray.isOnline()`         — any-channel-on-line predicate
 *   - `LineArray.getJunctionType()`  — junction classification (uint8_t mirroring JunctionType)
 *   - `LineArray.setReverseActive(bool)` — flip active board (front/rear)
 *
 * Each getter returns a test-settable scalar. Multi-value sequences can be
 * driven by calling `mockSet*` between ticks (the simplest possible "scripted
 * sensor" pattern — no queue, just last-write-wins).
 *
 * The `JunctionType` enum mirrors the production definition (mocks/Intersection.h
 * declares it identically so all three compilation units agree).
 */

// NOTE: This mock uses the SAME include guard (`WROLINEARRAY_H`) as the production
// `WRO2026/WROLineArray.h`. The test TU includes this mock FIRST so the guard is
// set; when `LineFollower.h` later resolves `#include "WROLineArray.h"` to the
// production file (source-relative include lookup wins over `-I` paths), the
// real header is short-circuited by the already-defined guard. See
// `WRO2026/test/CMakeLists.txt` and the explanatory block in each test main.
#ifndef WROLINEARRAY_H
#define WROLINEARRAY_H

#include <stdint.h>

/**
 * @brief Mirror of the production `JunctionType` enum
 *        (`WRO2026/Intersection.h`). Underlying values match the library 0..4
 *        convention exactly.
 *
 * Declared HERE (not in mocks/Intersection.h) because `WROLineArray.cpp`'s
 * production code casts `JunctionType::NONE` / `UNKNOWN` against the raw
 * `uint8_t` return from `getJunctionType()`. Both Intersection.h and
 * LineFollower.cpp include this header, so the enum must live where every
 * caller sees it — same module shape as production.
 */
enum class JunctionType : uint8_t
{
    NONE       = 0,
    LEFT       = 1,
    RIGHT      = 2,
    T_OR_CROSS = 3,
    UNKNOWN    = 4
};

class WROLineArrayClass
{
public:
    WROLineArrayClass() = default;

    void mockReset()
    {
        _error        = 0.0f;
        _isOnline     = true;
        _junctionType = (uint8_t) JunctionType::NONE;
        _reverse      = false;
        _setReverseCallCount = 0;
        _updateCallCount     = 0;
    }

    // ------------------------------------------------------------------ surface used by LineFollower / Intersection

    float   getError()         { return _error; }
    bool    isOnline()         { return _isOnline; }
    uint8_t getJunctionType()  { return _junctionType; }

    // Per-tick refresh hook. On hardware this re-samples the board; the mock's getters
    // already return whatever the test set, so it just records the call so a test can
    // assert the blocking-wait refresh happened (production calls it in its wait loops).
    void update()              { _updateCallCount++; }

    void setReverseActive(bool reverse)
    {
        _reverse = reverse;
        _setReverseCallCount++;
    }

    bool getReverseActive() const { return _reverse; }

    // ------------------------------------------------------------------ test setters

    void mockSetError(float e)              { _error = e; }
    void mockSetOnline(bool on)             { _isOnline = on; }
    void mockSetJunctionType(JunctionType j){ _junctionType = (uint8_t) j; }
    void mockSetJunctionTypeRaw(uint8_t v)  { _junctionType = v; }

    uint32_t mockSetReverseCallCount() const { return _setReverseCallCount; }
    uint32_t mockUpdateCallCount() const { return _updateCallCount; }

private:
    float    _error        = 0.0f;
    bool     _isOnline     = true;
    uint8_t  _junctionType = (uint8_t) JunctionType::NONE;
    bool     _reverse      = false;
    uint32_t _setReverseCallCount = 0;
    uint32_t _updateCallCount     = 0;
};

extern WROLineArrayClass LineArray;

#endif    // WROLINEARRAY_H
