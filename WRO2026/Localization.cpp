/**
 * @file Localization.cpp
 * @brief Implementation of the `Localization` singleton (see Localization.h).
 * @author WRO 2026
 */

#include "Localization.h"

#include <MatrixMiniR4.h>
#include <math.h>

#include "routes.h"

namespace
{
/// Backoff before the single retry of a transient lower-MCU IMU read. The
/// MMLower-caller convention is to retry once after a short delay on a non-OK
/// result; `GetIMUEuler` already carries its own 100 ms wait timeout, so this
/// is just bus settle time before re-issuing the command.
constexpr uint8_t IMU_READ_RETRY_DELAY_MS = 2;
}    // namespace

LocalizationClass Localization;    // single global instance

bool LocalizationClass::begin()
{
    _currentNode  = Routes::START_NODE;
    _yawOffsetDeg = 0.0f;
    _zeroed       = false;
    // Re-seed the heading cache so a re-begin / re-zero between bench runs does not serve
    // the prior run's last heading. The stamp resets to 0 ("no live read yet") so a
    // staleness check treats the seed value as not-yet-refreshed.
    _lastGoodHeadingDeg = (float) Routes::START_HEADING_DEG;
    _lastGoodHeadingMs  = 0;
    return true;
}

bool LocalizationClass::_readImuYaw(float& yawOut)
{
    // `MiniR4Motion::getEuler` reads via `mmL.GetIMUEuler` but DISCARDS the RESULT, so a
    // comms timeout there surfaces as a silent 0.0 yaw. Call the lower-MCU API directly so
    // the RESULT is visible. On a non-OK read the library leaves the out-params untouched
    // (MMLower.cpp GetIMUEuler returns before writing yaw on a wait/read timeout), so we
    // must NOT copy `yaw` into `yawOut` unless the read returned OK.
    double          roll = 0.0, pitch = 0.0, yaw = 0.0;
    MMLower::RESULT result = mmL.GetIMUEuler(roll, pitch, yaw);
    if (result != MMLower::RESULT::OK)
    {
        // Transient lower-MCU timeout: retry once after a short backoff (the
        // MMLower-caller convention) before giving up, so a single missed frame
        // does not fail zeroHeadingFromCurrent() or strand getHeadingDeg() on a
        // stale heading when the retry would have recovered.
        delay(IMU_READ_RETRY_DELAY_MS);
        result = mmL.GetIMUEuler(roll, pitch, yaw);
    }
    if (result != MMLower::RESULT::OK)
    {
        return false;
    }
    yawOut = (float) yaw;
    return true;
}

bool LocalizationClass::zeroHeadingFromCurrent()
{
    // Read the IMU yaw NOW through the result-surfacing path. The value can lie anywhere in
    // (-360, +360) depending on the boot orientation and any accumulated rotation since
    // `MiniR4.begin()` ran.
    float imuYaw = 0.0f;
    if (!_readImuYaw(imuYaw))
    {
        // Transient lower-MCU timeout: do NOT bind a corrupt frame. Leave `_yawOffsetDeg`
        // and `_zeroed` untouched and report failure so the .ino BTN_UP retry path engages.
        return false;
    }

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
    float imuYaw = 0.0f;
    if (!_readImuYaw(imuYaw))
    {
        // Transient lower-MCU timeout: a 0.0-derived heading would be wrong by up to a
        // full revolution and can flip a turn decision. Serve the last good heading
        // instead. The cache is not refreshed, so it stays at its last valid value.
        return _lastGoodHeadingDeg;
    }
    const float heading = _normalize(imuYaw - _yawOffsetDeg);
    _lastGoodHeadingDeg = heading;
    _lastGoodHeadingMs  = millis();
    return heading;
}

float LocalizationClass::getLastHeadingDeg() const
{
    // No I/O: return the value cached by the most recent successful getHeadingDeg() read.
    return _lastGoodHeadingDeg;
}

uint32_t LocalizationClass::getLastHeadingUpdateMs() const
{
    // No I/O: timestamp of the last OK getHeadingDeg() read (0 if none since begin()).
    return _lastGoodHeadingMs;
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
