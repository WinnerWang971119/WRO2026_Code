/**
 * @file Intersection.cpp
 * @brief Implementation of the edge-triggered intersection detector.
 * @author WRO 2026
 *
 * @details
 * See `Intersection.h` for the design rationale. This translation unit owns the single
 * `Intersection` global and is the only place that needs to depend on `LineFollower.h` —
 * the header keeps that include out so downstream consumers don't transitively pull in
 * `MatrixMiniR4.h`, `Config.h`, etc.
 */

#include "Intersection.h"
#include "../../Config.h"          // INTERSECTION_DEBOUNCE_TICKS / INTERSECTION_RELEASE_TICKS
#include "LineFollower.h"    // For LineFollower.getArray() — kept out of the header.

/**
 * @brief Single global instance.
 */
IntersectionClass Intersection;

bool IntersectionClass::begin()
{
    _array = LineFollower.getArray();
    if (_array == nullptr)
    {
        // Defensive — should not happen if LineFollower.begin() ran first.
        return false;
    }

    _wasAtJunction = false;
    _edgeFired     = false;
    _lastType      = JunctionType::NONE;
    _edgeCount     = 0;
    _validStreak   = 0;
    _noneStreak    = 0;
    return true;
}

void IntersectionClass::update()
{
    if (_array == nullptr)
    {
        return;
    }

    // @note With the WROLineArrayClass backend (post-Issue #16), our
    // classifier returns NONE/LEFT/RIGHT/T_OR_CROSS only — UNKNOWN is
    // never emitted. This validation and the UNKNOWN-as-UNKNOWN clamp
    // remain in place defensively in case (a) a future sensor backend
    // re-introduces ambiguous samples, or (b) bus corruption returns a
    // byte outside the [NONE..UNKNOWN] enum range. Removing the guard
    // would silently broadcast nonsense to consumers — keep it.
    //
    // Validate the raw byte is a known enum value before treating it as a JunctionType
    // (code review F7). Anything outside `[NONE..UNKNOWN]` is treated as UNKNOWN — the same
    // category as a sensor flicker. Without this guard a corrupt I2C byte would be
    // `static_cast` directly into `_lastType`, broadcasting nonsense to consumers.
    //
    // @note This `getJunctionType()` read reuses the sample LineFollower.update() just took
    // ONLY when `Config::LINE_ARRAY_TICK_CACHE_ENABLED` is true (the shipped default,
    // Config.h:450) — then it is a pure cache reader and issues no extra I2C transaction. With
    // the cache disabled this call re-reads the Wire1 bus, so Intersection observes a sample
    // distinct from LineFollower's (see the shared-single-sample design rule in Intersection.h).
    const uint8_t rawByte         = _array->getJunctionType();
    const bool    rawInRange      = rawByte <= static_cast<uint8_t>(JunctionType::UNKNOWN);
    const uint8_t raw             = rawInRange ? rawByte
                                                : static_cast<uint8_t>(JunctionType::UNKNOWN);
    const bool    atJunctionNow   = (raw != static_cast<uint8_t>(JunctionType::NONE)
                                   && raw != static_cast<uint8_t>(JunctionType::UNKNOWN));

    // Debounce streaks (code review F7). A sample only counts toward the "armed" or
    // "released" verdict once it has been observed for `*_TICKS` consecutive ticks.
    // Cap the increment at the threshold so the counters can't overflow on a long dwell.
    if (atJunctionNow)
    {
        if (_validStreak < Config::INTERSECTION_DEBOUNCE_TICKS)
        {
            _validStreak++;
        }
        _noneStreak = 0;
    }
    else
    {
        if (_noneStreak < Config::INTERSECTION_RELEASE_TICKS)
        {
            _noneStreak++;
        }
        _validStreak = 0;
    }

    const bool stableValid = _validStreak >= Config::INTERSECTION_DEBOUNCE_TICKS;
    const bool stableNone  = _noneStreak  >= Config::INTERSECTION_RELEASE_TICKS;

    // Rising edge: armed only when the valid streak crosses the debounce threshold AND
    // the previous debounced state was "released". A single noisy NONE/UNKNOWN sample no
    // longer flips `_wasAtJunction` — the release streak must be satisfied first.
    if (stableValid && !_wasAtJunction)
    {
        _lastType      = static_cast<JunctionType>(raw);
        _edgeFired     = true;
        _edgeCount++;
        _wasAtJunction = true;
    }
    else if (stableNone && _wasAtJunction)
    {
        _wasAtJunction = false;
    }
    // Otherwise: hold the previous debounced state. This is the key F7 fix — without it,
    // a single intermediate NONE/UNKNOWN tick mid-junction would reset `_wasAtJunction`
    // and the next valid sample would fire a duplicate edge for the same physical
    // junction.
}

bool IntersectionClass::isAtIntersection()
{
    const bool fired = _edgeFired;
    _edgeFired       = false;
    return fired;
}

uint32_t IntersectionClass::getEdgeCount() const
{
    return _edgeCount;
}

JunctionType IntersectionClass::getLastType() const
{
    return _lastType;
}
