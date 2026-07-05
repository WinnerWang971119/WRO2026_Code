/**
 * @file YawSource.cpp
 * @brief Implementation of the `YawSource` singleton (see YawSource.h).
 * @author WRO 2026
 */

#include "YawSource.h"

#include <MatrixMiniR4.h>

#include "../../Config.h"

namespace
{
/// Backoff before the single retry of a transient lower-MCU IMU read. Moved here from
/// Localization.cpp — YawSource is now the single owner of the lower-MCU yaw read. The
/// MMLower-caller convention is retry-once after a short delay on a non-OK result;
/// `GetIMUEuler` already carries its own 100 ms wait timeout, so this is just bus settle
/// time before re-issuing the command.
constexpr uint8_t IMU_READ_RETRY_DELAY_MS = 2;
}    // namespace

YawSourceClass YawSource;    // single global instance

bool YawSourceClass::readYawDeg(double& yawOut) const
{
    // `MiniR4Motion::getEuler` reads via `mmL.GetIMUEuler` but DISCARDS the RESULT, so a
    // comms timeout there surfaces as a silent 0.0 yaw. Call the lower-MCU API directly so
    // the RESULT is visible. On a non-OK read the library leaves the out-params untouched
    // (MMLower.cpp GetIMUEuler returns before writing yaw on a wait/read timeout), so we
    // must NOT write `yawOut` unless the read returned OK.
    double          roll = 0.0, pitch = 0.0, yaw = 0.0;
    MMLower::RESULT result = mmL.GetIMUEuler(roll, pitch, yaw);
    if (result != MMLower::RESULT::OK)
    {
        // Transient lower-MCU timeout: retry once after a short backoff (the MMLower-caller
        // convention) before giving up, so a single missed frame does not strand a caller
        // (zeroHeadingFromCurrent / getHeadingDeg) when the retry would have recovered.
        delay(IMU_READ_RETRY_DELAY_MS);
        result = mmL.GetIMUEuler(roll, pitch, yaw);
    }
    if (result != MMLower::RESULT::OK)
    {
        return false;
    }
    // Apply the per-chassis sign in floating point BEFORE any int cast (callers cast once).
    // Raw STM32 yaw is CW-positive; multiplying by IMU_YAW_SIGN (-1 on this chassis) yields
    // the app CCW-positive frame.
    yawOut = (double) Config::IMU_YAW_SIGN * yaw;
    return true;
}

double YawSourceClass::readYawDegOr(double fallback) const
{
    double y = 0.0;
    return readYawDeg(y) ? y : fallback;
}

int16_t YawSourceClass::toHwHeading(int16_t appHeadingDeg)
{
    // Pure sign flip: app CCW-positive heading -> STM32 raw CW-positive gyro-PID target.
    // app<->raw differ by sign alone (the Localization offset is already folded into the
    // app-frame value reaching here), so no offset term. Float multiply before the cast.
    return (int16_t) (Config::IMU_YAW_SIGN * (float) appHeadingDeg);
}
