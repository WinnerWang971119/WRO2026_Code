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
#include "LineFollower.h"    // For LineFollower.getTracer() — kept out of the header.

/**
 * @brief Single global instance.
 */
IntersectionClass Intersection;

bool IntersectionClass::begin()
{
    _tracer = LineFollower.getTracer();
    if (_tracer == nullptr)
    {
        // Defensive — should not happen if LineFollower.begin() ran first.
        return false;
    }

    _wasAtJunction = false;
    _edgeFired     = false;
    _lastType      = JunctionType::NONE;
    return true;
}

void IntersectionClass::update()
{
    if (_tracer == nullptr)
    {
        return;
    }

    const uint8_t raw           = _tracer->getJunctionType();
    const bool    atJunctionNow = (raw != static_cast<uint8_t>(JunctionType::NONE)
                                   && raw != static_cast<uint8_t>(JunctionType::UNKNOWN));

    // Rising edge: latch the type and raise the consumer flag.
    if (atJunctionNow && !_wasAtJunction)
    {
        _lastType  = static_cast<JunctionType>(raw);
        _edgeFired = true;
    }

    _wasAtJunction = atJunctionNow;
}

bool IntersectionClass::isAtIntersection()
{
    const bool fired = _edgeFired;
    _edgeFired       = false;
    return fired;
}

JunctionType IntersectionClass::getLastType() const
{
    return _lastType;
}
