/**
 * @file Localization.cpp
 * @brief Implementation of the `Localization` singleton (see Localization.h).
 * @author WRO 2026
 */

#include "Localization.h"

#include <MatrixMiniR4.h>
#include <math.h>

#include "Drive.h"
#include "WROMath.h"
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
    // Arriving on a node re-anchors the operator-offset frame. The baseline re-snapshot inside
    // clearOffset() is what wins the arrival-tick race: on the single tick where Drive finishes
    // the approach and PathTracker re-anchors, the very next update() integrates a zero delta, so
    // getOffsetFromNode() reads ~0 instead of a phantom equal to the final approach motion.
    clearOffset();
}

void LocalizationClass::onDeadReckonedAdvance(uint8_t advanceTo)
{
    _currentNode = advanceTo;
    // Same re-anchor as onIntersection — a dead-reckoned advance lands on a node too, so the
    // offset frame resets to zero with the baselines re-snapshotted (kills the arrival phantom).
    clearOffset();
}

void LocalizationClass::update()
{
    // Two conditions open the integrate gate: PathTracker is not driving a hop (the operator may
    // be hand-driving) AND the world frame is bound (heading is meaningless before zeroing).
    const bool gateOpen = (!_navActive) && _zeroed;
    if (gateOpen)
    {
        _integrateOffset();
        return;
    }

    // Gate closed: do NOT accumulate, but keep every baseline current so that the moment the gate
    // reopens the first per-tick delta is ~0 (no jump from motion that happened while PathTracker
    // was driving, and no jump across the pre-zero window).
    _lastEncL   = Drive.getEncL();
    _lastEncR   = Drive.getEncR();
    _lastYawDeg = getHeadingDeg();
}

Offset2D LocalizationClass::getOffsetFromNode() const
{
    // Read-only telemetry: no I/O, no mutation — just hand back the accumulated offset.
    return _offset;
}

void LocalizationClass::clearOffset()
{
    // Thin wrapper over the re-anchor helper (naming per the plan: integrate vs consume).
    _consumeOffset();
}

void LocalizationClass::setNavActive(bool active)
{
    _navActive = active;
}

void LocalizationClass::_integrateOffset()
{
    // Snapshot the live heading once: both channels reference the same world-frame reading so the
    // translation decomposition and the rotation delta are consistent for this tick. getHeadingDeg()
    // is const and refreshes its own mutable cache, so calling it from this non-const path is fine.
    const float heading = getHeadingDeg();

    // ---- Translation channel (gated independently of rotation) -------------------------------
    const int32_t encL = Drive.getEncL();
    const int32_t encR = Drive.getEncR();
    // Average the two wheel deltas (encoder degrees) so a pure on-the-spot turn — where the wheels
    // move equal-and-opposite — nets ~zero translation. encDegToCm carries the sign through.
    const int32_t dEnc = ((encL - _lastEncL) + (encR - _lastEncR)) / 2;
    const float   dCm  = encDegToCm(dEnc);
    if (fabsf(dCm) >= Config::OFFSET_TRANSLATION_DEADBAND_CM)
    {
        // Above the deadband: decompose along the current world heading and advance the baselines.
        // Use the Arduino `PI` macro (defined by Arduino.h and the host mock) rather than M_PI,
        // which the host test mock does not provide — matching the existing WROMath.h convention.
        const float headingRad = heading * ((float) PI / 180.0f);
        _offset.x += dCm * cosf(headingRad);
        _offset.y += dCm * sinf(headingRad);
        _lastEncL = encL;
        _lastEncR = encR;
    }
    // Below the deadband: leave _offset AND the encoder baselines UNADVANCED, so slow sub-deadband
    // motion still accumulates across ticks until it crosses the floor. A stationary chassis never
    // crosses it (dCm stays ~0 forever), satisfying the no-phantom-drift requirement.

    // ---- Rotation channel (gated independently of translation) -------------------------------
    const float dYaw = _shortestAngleDelta(heading, _lastYawDeg);
    if (fabsf(dYaw) >= Config::OFFSET_ROTATION_DEADBAND_DEG)
    {
        // A pure manual turnDeg nets ~zero translation but a real yaw change, so the rotation
        // channel must update thetaDeg even when the translation channel skipped this tick.
        _offset.thetaDeg += dYaw;
        _lastYawDeg = heading;
    }
    // Below the deadband: leave thetaDeg and the yaw baseline UNADVANCED (same sub-deadband
    // carry-across rationale as the translation channel).
}

void LocalizationClass::_consumeOffset()
{
    // Re-anchor: zero the offset and re-snapshot all three baselines to the CURRENT readings, so
    // the next update() integrates a zero delta. This is the arrival-tick phantom kill — the
    // baselines jump forward to "now" before the next integrate, discarding the approach motion.
    _offset     = Offset2D{};
    _lastEncL   = Drive.getEncL();
    _lastEncR   = Drive.getEncR();
    _lastYawDeg = getHeadingDeg();
}

float LocalizationClass::_shortestAngleDelta(float toDeg, float fromDeg)
{
    // Signed difference wrapped into [-180, 180]. fmodf can return a value in (-360, 360); the
    // two guards fold it into the symmetric half-revolution interval. +180 maps to +180, and the
    // -180 boundary maps to +180 as well (both name the same antipode) — fine for accumulation.
    float delta = fmodf(toDeg - fromDeg, 360.0f);
    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }
    return delta;
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
