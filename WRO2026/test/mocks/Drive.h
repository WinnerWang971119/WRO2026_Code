/**
 * @file Drive.h (HOST MOCK)
 * @brief Minimal `DriveClass` mock — only the surface called by `LineFollower.cpp`.
 *
 * `LineFollower.cpp` calls exactly these `Drive` methods:
 *   - `Drive.setSpeed(L, R)`   (continuous PWM emit)
 *   - `Drive.stop(brake)`      (brake/coast)
 *   - `Drive.getEncL()`        (left encoder, post-direction)
 *   - `Drive.getEncR()`        (right encoder, post-direction)
 *   - `Drive.isIdle()`         (used only by the sync-blocking helpers)
 *
 * The mock records each call's arguments and exposes them via public counters
 * + last-call accessors so the tests can assert on dispatch behavior.
 *
 * Encoder values are settable so a test can simulate forward chassis motion by
 * incrementing the encoders manually between `LineFollower.update()` ticks.
 */

// NOTE: This mock uses the SAME include guard (`DRIVE_H`) as the production
// `WRO2026/Drive.h`. See `mocks/WROLineArray.h` for the full rationale.
#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

// Production `LineFollower.cpp` relies on `Drive.h` transitively pulling in
// `WROMath.h` (which holds the free functions `deadbandSnap`, `presetToPwm`,
// `encDegToCm`). We mirror that include order here so the controller TU finds
// those symbols when the mock Drive.h shadows the real one.
#include "WROMath.h"

/**
 * @brief Captured details of the most recent `Drive` method call.
 */
struct DriveCallRecord
{
    // setSpeed counters
    uint32_t setSpeedCallCount = 0;
    int16_t  lastSetSpeedL     = 0;
    int16_t  lastSetSpeedR     = 0;

    // stop counters
    uint32_t stopCallCount       = 0;
    bool     lastStopBrake       = false;
    uint32_t stopBrakeTrueCount  = 0;
    uint32_t stopBrakeFalseCount = 0;
};

class DriveClass
{
public:
    // ------------------------------------------------------------------ ctor / reset
    DriveClass() = default;

    /// Reset all recorded state + encoder counters. Tests call this at the top
    /// of each scenario.
    void mockReset()
    {
        _record    = DriveCallRecord{};
        _encL      = 0;
        _encR      = 0;
        _isIdle    = true;
        _hasFault  = false;
    }

    // ------------------------------------------------------------------ surface used by LineFollower

    uint8_t setSpeed(int16_t L, int16_t R)
    {
        _record.setSpeedCallCount++;
        _record.lastSetSpeedL = L;
        _record.lastSetSpeedR = R;
        return 0;
    }

    uint8_t stop(bool brake = true)
    {
        _record.stopCallCount++;
        _record.lastStopBrake = brake;
        if (brake) { _record.stopBrakeTrueCount++; }
        else       { _record.stopBrakeFalseCount++; }
        return 0;
    }

    int32_t getEncL() const { return _encL; }
    int32_t getEncR() const { return _encR; }

    bool isIdle() const { return _isIdle; }

    // ------------------------------------------------------------------ test setters

    void mockSetEncL(int32_t v) { _encL = v; }
    void mockSetEncR(int32_t v) { _encR = v; }
    void mockAdvanceEncoders(int32_t deltaL, int32_t deltaR) { _encL += deltaL; _encR += deltaR; }
    void mockSetIdle(bool idle) { _isIdle = idle; }

    const DriveCallRecord& mockRecord() const { return _record; }

private:
    DriveCallRecord _record;
    int32_t         _encL     = 0;
    int32_t         _encR     = 0;
    bool            _isIdle   = true;
    bool            _hasFault = false;
};

/// Single global instance — defined in `mocks/Drive.cpp`.
extern DriveClass Drive;

#endif    // DRIVE_H
