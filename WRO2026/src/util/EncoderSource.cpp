/**
 * @file EncoderSource.cpp
 * @brief Implementation of the `EncoderSource` singleton (see EncoderSource.h).
 * @author WRO 2026
 */

#include "EncoderSource.h"

#include <MatrixMiniR4.h>

#include "../../Config.h"

namespace
{
/// Backoff before the single retry of a transient lower-MCU encoder read. Mirrors
/// `YawSource`'s `IMU_READ_RETRY_DELAY_MS`: the MMLower-caller convention is retry-once
/// after a short bus-settle delay on a non-OK result.
constexpr uint8_t ENC_READ_RETRY_DELAY_MS = 2;
}    // namespace

EncoderSourceClass EncoderSource;    // single global instance

bool EncoderSourceClass::readDegrees(uint8_t motorId, int32_t& out)
{
    if (!_validId(motorId))
    {
        return false;    // out untouched
    }

    // `MiniR4DC::getDegrees` reads `mmL.GetEncoderDegrees` but DISCARDS the RESULT, so a
    // comms timeout there surfaces as a silent 0. Call the lower-MCU API directly so the
    // RESULT is visible. On a non-OK read the library leaves the out-param untouched, so we
    // must NOT write `out` (or refresh the cache) unless the read returned OK.
    int32_t         degs   = 0;
    MMLower::RESULT result = mmL.GetEncoderDegrees(motorId, degs);
    if (result != MMLower::RESULT::OK)
    {
        // Transient lower-MCU timeout: retry once after a short backoff (the MMLower-caller
        // convention) before giving up, so a single missed frame does not strand a baseline
        // read when the retry would have recovered.
        delay(ENC_READ_RETRY_DELAY_MS);
        result = mmL.GetEncoderDegrees(motorId, degs);
    }
    if (result != MMLower::RESULT::OK)
    {
        return false;    // hard failure — out untouched, cache NOT refreshed
    }

    // Success: publish the value and refresh the last-good cache. A genuine 0 read with an
    // OK result is cached as 0 (a success-0 is never treated as a failure).
    out                      = degs;
    _cache[motorId].lastGood = degs;
    _cache[motorId].valid    = true;
    return true;
}

int32_t EncoderSourceClass::readDegreesOr(uint8_t motorId, int32_t fallback)
{
    int32_t degs = 0;
    if (readDegrees(motorId, degs))
    {
        return degs;    // fresh read (cache already refreshed inside readDegrees)
    }
    // Hard failure: serve the centralized last-good if one exists, else the caller's
    // fallback. After a resetCache the slot is invalid, so the pre-reset value can never be
    // served here.
    if (_validId(motorId) && _cache[motorId].valid)
    {
        return _cache[motorId].lastGood;
    }
    return fallback;
}

void EncoderSourceClass::resetCache(uint8_t motorId)
{
    if (!_validId(motorId))
    {
        return;
    }
    _cache[motorId].lastGood = 0;
    _cache[motorId].valid    = false;
}

bool EncoderSourceClass::isPrevTaskDone()
{
    // `MiniR4DriveDC::isPrevTaskDone` reads `mmL.Get_Drive_isTaskDone` but DISCARDS the
    // RESULT, so a comms timeout there fabricates a done-bit. Call the lower-MCU API directly
    // and gate on the RESULT: any non-OK read is treated as "not done" (the safe direction —
    // a continuous completion check simply retries next tick). `isEnd[0] == true` is the
    // vendor's "still running" convention.
    bool                  stsu[2] = {false, false};
    MMLower::Drive_RESULT result  = mmL.Get_Drive_isTaskDone(DRIVE_INSTANCE_ID, stsu);
    if (result != MMLower::Drive_RESULT::OK)
    {
        return false;
    }
    return stsu[0] ? false : true;
}
