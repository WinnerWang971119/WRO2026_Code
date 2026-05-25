/**
 * @file Intersection.h (HOST MOCK)
 * @brief Minimal `IntersectionClass` mock — only the surface called by
 *        `LineFollower.cpp` and the test scenarios.
 *
 * `LineFollower.cpp` calls only `Intersection.isAtIntersection()` and
 * `Intersection.update()` from inside its sync-blocking helpers
 * (`waitUntilIntersection`). The overshoot-mode + brake-on-gate tests do NOT
 * exercise the sync helpers — they tick `LineFollower.update()` directly — so
 * these methods can be trivial stubs.
 *
 * The mock also exposes a `mockSetAtIntersection(...)` setter for the
 * sync-helper case if a future test wants to cover it.
 *
 * `JunctionType` is declared in `mocks/WROLineArray.h` (see comment there for
 * why); we forward-declare nothing and just include that header.
 */

// NOTE: This mock uses the SAME include guard (`INTERSECTION_H`) as the production
// `WRO2026/Intersection.h`. See `mocks/WROLineArray.h` for the full rationale.
#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <stdint.h>
#include "WROLineArray.h"    // For `JunctionType`.

class IntersectionClass
{
public:
    void mockReset()
    {
        _atIntersection  = false;
        _lastType        = JunctionType::NONE;
        _edgeCount       = 0;
        _isAtCallCount   = 0;
        _updateCallCount = 0;
    }

    // ------------------------------------------------------------------ surface used by LineFollower

    void update() { _updateCallCount++; }

    /// Read-and-clear semantics mirror production. After this returns `true`,
    /// the latched flag clears.
    bool isAtIntersection()
    {
        _isAtCallCount++;
        const bool latched = _atIntersection;
        _atIntersection = false;
        return latched;
    }

    uint32_t     getEdgeCount() const { return _edgeCount; }
    JunctionType getLastType()  const { return _lastType;  }

    // ------------------------------------------------------------------ test setters

    void mockSetAtIntersection(bool v) { _atIntersection = v; }
    void mockSetLastType(JunctionType t) { _lastType = t; }
    void mockBumpEdgeCount() { _edgeCount++; }

    uint32_t mockIsAtCallCount() const { return _isAtCallCount; }
    uint32_t mockUpdateCallCount() const { return _updateCallCount; }

private:
    bool         _atIntersection  = false;
    JunctionType _lastType        = JunctionType::NONE;
    uint32_t     _edgeCount       = 0;
    uint32_t     _isAtCallCount   = 0;
    uint32_t     _updateCallCount = 0;
};

extern IntersectionClass Intersection;

#endif    // INTERSECTION_H
