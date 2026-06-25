/**
 * @file Intersection.h (PATH HOST MOCK)
 * @brief Thin record-and-test-set `IntersectionClass` mock for the PathTracker + Drive
 *        host-link targets (issue #106).
 * @author WRO 2026
 *
 * @details
 * Shadows the production `WRO2026/Intersection.h` (same `INTERSECTION_H` guard,
 * force-included by `add_path_force_includes`) so the five PathTracker host tests link this
 * stub instead of the real `Intersection.cpp` (which binds the shared `WROLineArrayClass`
 * and the `MiniR4` global). It mocks the rising-edge detector that the REAL
 * `PathTracker.cpp` only *consumes*: the FSM polls `Intersection.isAtIntersection()` to
 * re-anchor on a junction crossing and drains a stale edge with `(void) isAtIntersection()`.
 *
 * Verified production call sites this surface satisfies:
 *   - `PathTracker.cpp:712`   `if (Intersection.isAtIntersection())`
 *   - `PathTracker.cpp:1278`  `(void) Intersection.isAtIntersection()`  (stale-edge drain)
 *   - `PathTracker.cpp:1880`  `(void) Intersection.isAtIntersection()`
 *   - `PathTracker.cpp:2028`  `(void) Intersection.isAtIntersection()`
 *
 * `getLastType()` / `getEdgeCount()` / `begin()` / `update()` are not reached by the linked
 * production `.cpp`s today, but are provided so the test harness can drive and assert
 * junction type/count without touching production. `JunctionType` is declared in
 * `path_mocks/WROLineArray.h` (see the comment there for why); this header just includes it.
 *
 * ---------------------------------------------------------------------------
 * Fidelity — where this mock can lie vs real hardware
 * ---------------------------------------------------------------------------
 * THIN record-and-test-set per ADR-049. It models nothing the host cannot observe:
 *   - No shared `WROLineArrayClass` sampling, no debounce streaks, no rising-edge math.
 *     `update()` is a no-op (the test scripts the latched flag directly), `begin()` is a
 *     no-op.
 *   - `isAtIntersection()` honors the production READ-AND-CLEAR contract as a single-call
 *     CONTRACT (returns the latched flag, then clears it) — this is observation, not
 *     multi-tick modeling: the mock never sets the flag on its own across ticks. A test
 *     raises it via `mockSetAtIntersection(true)` for the tick it wants the edge seen.
 *   - `getLastType()` / `getEdgeCount()` echo whatever the test set; no edge is latched or
 *     counted unless the test does it via the `mock*` setters.
 */

// NOTE: This mock uses the SAME include guard (`INTERSECTION_H`) as the production
// `WRO2026/Intersection.h`. See `path_mocks/WROLineArray.h` for the full guard-shadow
// rationale.
#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <stdint.h>

#include "WROLineArray.h"    // For `JunctionType` (declared in the WROLineArray path mock).

/**
 * @brief Thin record-and-test-set mock of the edge-triggered intersection detector.
 *
 * @details
 * Exposes the public surface the real `PathTracker.cpp` consumes plus the passive
 * observer accessors (`getLastType()` / `getEdgeCount()`) so the harness can assert the
 * captured junction without production wiring. See the file-level Fidelity block for what
 * this mock deliberately does not model.
 */
class IntersectionClass
{
public:
    IntersectionClass() = default;

    /**
     * @brief Reset every recording/test-set field to its construction default.
     *
     * @note Call between test cases so a prior case's latched edge, captured type, or edge
     *       count does not leak into the next.
     */
    void mockReset()
    {
        _atIntersection  = false;
        _lastType        = JunctionType::NONE;
        _edgeCount       = 0;
        _isAtCallCount   = 0;
        _updateCallCount = 0;
    }

    // ------------------------------------------------------------------ lifecycle (no-op)

    /**
     * @brief Bind / reset hook — no-op on host (no shared line array to bind onto).
     * @return Always `true` (the real non-null-array success path).
     */
    bool begin() { return true; }

    /**
     * @brief Cooperative tick — records the call, runs no edge detector.
     * @note The test scripts the latched flag via `mockSetAtIntersection(...)`; this mock
     *       does NOT derive an edge from any sensor sample (ADR-049). The counter lets a
     *       test assert the harness ticked it.
     */
    void update() { _updateCallCount++; }

    // ------------------------------------------------------------------ surface real PathTracker.cpp reaches

    /**
     * @brief Read-and-clear consumer for the edge-triggered junction event.
     *
     * @return `true` if an edge was latched (via `mockSetAtIntersection(true)`) since the
     *         last call; `false` otherwise. ALWAYS clears the latch before returning.
     * @note Honors the production read-and-clear contract: the real `PathTracker.cpp` reads
     *       this once per crossing to advance/re-anchor, and drains a stale edge with
     *       `(void) isAtIntersection()`. Intentionally non-`const` because it mutates the
     *       latch.
     */
    bool isAtIntersection()
    {
        _isAtCallCount++;
        const bool latched = _atIntersection;
        _atIntersection = false;
        return latched;
    }

    /**
     * @brief Non-consuming rising-edge counter (test-set).
     * @return The count last set/bumped via the `mock*` setters; `0` until set.
     * @note Survives `isAtIntersection()` consumption — mirrors the production passive
     *       observer accessor.
     */
    uint32_t getEdgeCount() const { return _edgeCount; }

    /**
     * @brief Type classification captured at the most recent edge (test-set).
     * @return The `JunctionType` last set via `mockSetLastType(...)`; `NONE` until set.
     * @note Survives `isAtIntersection()` consumption — callers inspect the type after
     *       acknowledging the event, exactly as production does.
     */
    JunctionType getLastType() const { return _lastType; }

    // ------------------------------------------------------------------ test setters

    /** @brief Latch (or clear) the edge the next `isAtIntersection()` will report. */
    void mockSetAtIntersection(bool v) { _atIntersection = v; }

    /** @brief Set the captured junction type the next `getLastType()` returns. */
    void mockSetLastType(JunctionType t) { _lastType = t; }

    /** @brief Set the edge count the next `getEdgeCount()` returns. */
    void mockSetEdgeCount(uint32_t n) { _edgeCount = n; }

    /** @brief Increment the edge count (convenience for sequential-edge scripts). */
    void mockBumpEdgeCount() { _edgeCount++; }

    /** @brief Number of `isAtIntersection()` calls since the last `mockReset()`. */
    uint32_t mockIsAtCallCount() const { return _isAtCallCount; }

    /** @brief Number of `update()` calls since the last `mockReset()`. */
    uint32_t mockUpdateCallCount() const { return _updateCallCount; }

private:
    bool         _atIntersection  = false;                 ///< Test-set latched edge.
    JunctionType _lastType        = JunctionType::NONE;    ///< Test-set captured type.
    uint32_t     _edgeCount       = 0;                     ///< Test-set edge count.
    uint32_t     _isAtCallCount   = 0;                     ///< `isAtIntersection` call count.
    uint32_t     _updateCallCount = 0;                     ///< `update` call count.
};

/**
 * @brief Single global instance — `inline` so exactly ONE definition exists across the
 *        target's TUs without anything in `path_mocks.cpp`.
 *
 * @note The `mocks/` / `task_mocks/` / `mmL` idiom: declaring the global `inline` in-header
 *       (C++17 inline variable) means the facade-globals TU defines no peripheral global,
 *       decoupling it from this file.
 */
inline IntersectionClass Intersection;

#endif    // INTERSECTION_H
