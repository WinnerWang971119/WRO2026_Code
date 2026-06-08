/**
 * @file LineFollower.h (CTPS HOST MOCK)
 * @brief No-op mock of the LineFollower facade for the CTPS dump.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `LineFollower.h` (quote-included; force-included by the T4
 * cmake wiring). The include guard is the REAL one (`LINEFOLLOWER_H`) so the
 * real header no-ops if reached afterwards.
 *
 * LineFollower is not a recording seam (line following produces no command-stream
 * entries the CTPS player consumes), so every method is a no-op. Only the surface
 * Mission.cpp references is provided: the `setEnabled(bool)` fluent builder and
 * `begin()`.
 *
 * The enable builder mirrors the real move-only `[[nodiscard]]` shape so the
 * discarded-temporary call site `LineFollower.setEnabled(false);` compiles the
 * same way it does against the real header. The chain setters Mission.cpp does
 * not use are included so any future chained call site still compiles.
 */

#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <stdint.h>

#include "Config.h"    // Config::SpeedPreset for the chain setters

class LineFollowerClass;

/**
 * @brief No-op recording-free mock of `LineFollowerEnableBuilder`.
 *
 * Move-only `[[nodiscard]]` like the real builder; every setter is a no-op
 * returning `*this`. No state is applied on destruction.
 */
class [[nodiscard]] LineFollowerEnableBuilder
{
public:
    LineFollowerEnableBuilder(LineFollowerClass& /*owner*/, bool /*targetEnabled*/) {}

    LineFollowerEnableBuilder(const LineFollowerEnableBuilder&)            = delete;
    LineFollowerEnableBuilder& operator=(const LineFollowerEnableBuilder&) = delete;
    LineFollowerEnableBuilder(LineFollowerEnableBuilder&&)                 = delete;
    LineFollowerEnableBuilder& operator=(LineFollowerEnableBuilder&&)      = delete;

    LineFollowerEnableBuilder& setBrake(bool) { return *this; }
    LineFollowerEnableBuilder& setPreset(Config::SpeedPreset) { return *this; }
    LineFollowerEnableBuilder& tShape() { return *this; }
    LineFollowerEnableBuilder& setTShapeRampUp(bool) { return *this; }
    LineFollowerEnableBuilder& setTShapeRampDown(bool) { return *this; }
    LineFollowerEnableBuilder& setAccel(float) { return *this; }
    LineFollowerEnableBuilder& setReverse(bool) { return *this; }
    LineFollowerEnableBuilder& setDistanceTarget(float) { return *this; }
    LineFollowerEnableBuilder& setOvershootMode(bool) { return *this; }
    LineFollowerEnableBuilder& setBrakeOnGate(bool) { return *this; }
};

/**
 * @brief No-op mock of the LineFollower facade.
 */
class LineFollowerClass
{
public:
    /** @brief No-op bring-up; always reports success. */
    bool begin() { return true; }

    /** @brief No-op cooperative tick. */
    void update() {}

    /** @brief Return a no-op enable builder (matches the real `[[nodiscard]]` verb). */
    [[nodiscard]] LineFollowerEnableBuilder setEnabled(bool on)
    {
        return LineFollowerEnableBuilder(*this, on);
    }

    void                setPreset(Config::SpeedPreset) {}
    Config::SpeedPreset getPreset() const { return Config::SpeedPreset::NORMAL; }
    void                setReverse(bool) {}
    bool                getReverse() const { return false; }
    bool                isEnabled() const { return false; }
    bool                isLineLost() const { return false; }
    bool                isPidFrozen() const { return false; }
    float               getRawError() const { return 0.0f; }
    bool                isDistanceReached() { return true; }
    void                setTickCallback(void (*)()) {}
    bool                followDistance(float) { return true; }
    bool                waitUntilDistanceReached() { return true; }
    bool                followUntilIntersection() { return true; }
    bool                waitUntilIntersection() { return true; }
};

/** @brief Single global instance — defined in ctps_record.cpp. */
extern LineFollowerClass LineFollower;

#endif    // LINEFOLLOWER_H
