/**
 * @file Localization.cpp
 * @brief Implementation of the `Localization` singleton (see Localization.h).
 * @author WRO 2026
 */

#include "Localization.h"

#include <MatrixMiniR4.h>
#include <math.h>

#include "routes.h"

LocalizationClass Localization;    // single global instance

bool LocalizationClass::begin()
{
    _currentNode  = Routes::START_NODE;
    _yawOffsetDeg = 0.0f;
    _zeroed       = false;
    return true;
}

bool LocalizationClass::zeroHeadingFromCurrent()
{
    // Read the IMU yaw NOW. MiniR4.Motion.getEuler returns a double in degrees; the
    // value can lie anywhere in (-360, +360) depending on the boot orientation and any
    // accumulated rotation since `MiniR4.begin()` ran.
    const float imuYaw = (float) MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);

    // World-frame definition: world = normalize(imu - offset). At the zero moment we
    // want world == Routes::START_HEADING_DEG, so offset = imu - START_HEADING_DEG.
    _yawOffsetDeg = imuYaw - (float) Routes::START_HEADING_DEG;
    _zeroed       = true;
    return true;
}

bool LocalizationClass::isZeroed() const
{
    return _zeroed;
}

uint8_t LocalizationClass::getCurrentNode() const
{
    return _currentNode;
}

float LocalizationClass::getHeadingDeg() const
{
    if (!_zeroed)
    {
        // Defensive: callers should not consult heading before zeroing, but if they
        // do, return the start heading rather than a garbage offset-relative value.
        return Routes::START_HEADING_DEG;
    }
    const float imuYaw = (float) MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
    return _normalize(imuYaw - _yawOffsetDeg);
}

int16_t LocalizationClass::toImuYaw(float worldHeadingDeg) const
{
    const float w        = _normalize(worldHeadingDeg);
    const float imuFloat = w + _yawOffsetDeg;
    // Drive's `_normalizeYaw(int32_t)` further reduces to (-360, +360). Round to nearest
    // int16 here. roundf is closest-to-zero for half-values; that's fine for our use,
    // which never hits half-degree targets in practice (the toy fixture uses 45-deg
    // multiples; the real field is integer headings).
    const long rounded = lroundf(imuFloat);
    return (int16_t) rounded;
}

void LocalizationClass::onIntersection(uint8_t advanceTo)
{
    _currentNode = advanceTo;
}

void LocalizationClass::onDeadReckonedAdvance(uint8_t advanceTo)
{
    _currentNode = advanceTo;
}

float LocalizationClass::_normalize(float deg)
{
    // Handle arbitrary inputs: shift into [0, 360) by repeatedly adding/subtracting 360.
    // fmodf can return negative results for negative inputs, so the explicit guard
    // afterward ensures the final value is in the half-open interval.
    float r = fmodf(deg, 360.0f);
    if (r < 0.0f)
    {
        r += 360.0f;
    }
    // r is now in [0, 360]. Edge case: r == 360.0 exactly snaps back to 0.
    if (r >= 360.0f)
    {
        r -= 360.0f;
    }
    return r;
}
