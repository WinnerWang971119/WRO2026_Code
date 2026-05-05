/**
 * @file LineFollower.cpp
 * @brief Implementation of the outer P+D line-following controller and its fluent builder.
 * @author WRO 2026
 */

#include "LineFollower.h"

#include "Drive.h"

/**
 * @brief Single global instance referenced by every other module.
 */
LineFollowerClass LineFollower;

// =========================================================================================
// LineFollowerEnableBuilder — RAII auto-execute fluent builder.
// =========================================================================================

LineFollowerEnableBuilder::LineFollowerEnableBuilder(LineFollowerClass& owner, bool targetEnabled)
    : _owner(owner)
    , _targetEnabled(targetEnabled)
{
    // Intentional: stage only. State is applied in the destructor (RAII auto-execute) so the
    // entire fluent chain is committed atomically at end-of-statement.
}

LineFollowerEnableBuilder::~LineFollowerEnableBuilder()
{
    if (_executed)
    {
        // Defensive guard against double-execute. Copy and move are deleted, so reaching this
        // path would imply manual destructor invocation — still a no-op for safety.
        return;
    }

    _owner._applyEnableBuilder(_targetEnabled, _brake, _hasPresetOverride, _presetOverride);
    _executed = true;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setBrake(bool brake)
{
    // Per the doxygen contract: this is silently ignored when transitioning to enabled, but
    // we still record it so unit-test inspection (if ever wired up) is consistent.
    _brake = brake;
    return *this;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setPreset(Config::SpeedPreset preset)
{
    _presetOverride    = preset;
    _hasPresetOverride = true;
    return *this;
}

// =========================================================================================
// LineFollowerClass — lifecycle.
// =========================================================================================

bool LineFollowerClass::begin()
{
    // Wire the tracer onto Wire1 + the configured mux channel BEFORE calling its begin().
    // The driver caches the pointer/channel internally and uses them on every transaction.
    _tracer._pWire = &Wire1;
    _tracer._ch    = Config::LINE_I2C_PORT;

    _tracer.begin();
    _tracer.setThreshold(Config::LINE_THRESHOLD);

    _enabled  = false;
    _lostLine = false;
    _lastErr  = 0.0f;
    _preset   = Config::SpeedPreset::NORMAL;

    return true;
}

// =========================================================================================
// Cooperative tick.
// =========================================================================================

void LineFollowerClass::update()
{
    if (!_enabled)
    {
        return;
    }

    // Resolve cruise from the active preset. Reading once at the top of the tick keeps the
    // L/R mix symmetric and avoids any chance of preset mid-tick flip-flopping.
    const int16_t cruise = _presetToPwm(_preset);

    if (!_tracer.isOnline())
    {
        // Clear the D-term history so the first tick after reacquisition does not see a stale
        // derivative computed against an old (possibly several-second-stale) error sample.
        _lastErr = 0.0f;

        // Line lost beneath every channel. Dispatch on Config::LOST_LINE_ACTION.
        _lostLine = true;

        switch (Config::LOST_LINE_ACTION)
        {
            case Config::LostLineAction::STOP:
                Drive.stop(true);
                break;

            case Config::LostLineAction::COAST:
                Drive.stop(false);
                break;

            case Config::LostLineAction::CRAWL:
                // Straight creep forward (no steering) so the strip may be reacquired.
                Drive.setSpeed((int16_t) (cruise / 2), (int16_t) (cruise / 2));
                break;
        }
        return;
    }

    // -------------------------------------------------------------------------------------
    // Line online: run the P+D controller.
    // -------------------------------------------------------------------------------------
    _lostLine = false;

    const float err  = _tracer.getError();
    const float dErr = err - _lastErr;
    _lastErr         = err;

    float corr = (Config::LINE_KP * err) + (Config::LINE_KD * dErr);

    // Clamp the correction to the configured maximum.
    if (corr > Config::LINE_MAX_CORR)
    {
        corr = Config::LINE_MAX_CORR;
    }
    else if (corr < -Config::LINE_MAX_CORR)
    {
        corr = -Config::LINE_MAX_CORR;
    }

    // Differential mix around the cruise setpoint (sourced from the active preset).
    int16_t L = (int16_t) (cruise + corr);
    int16_t R = (int16_t) (cruise - corr);

    // Re-clamp each channel to the DriveDC PWM range.
    L = (int16_t) constrain(L, -100, 100);
    R = (int16_t) constrain(R, -100, 100);

    Drive.setSpeed(L, R);
}

// =========================================================================================
// Fluent state control.
// =========================================================================================

LineFollowerEnableBuilder LineFollowerClass::setEnabled(bool on)
{
    // Construct in place — guaranteed copy-elision (NRVO) means the temporary lives at the
    // call site. The builder's destructor then applies the staged transition exactly once.
    return LineFollowerEnableBuilder(*this, on);
}

void LineFollowerClass::setPreset(Config::SpeedPreset preset)
{
    _preset = preset;
}

void LineFollowerClass::_applyEnableBuilder(bool                targetEnabled,
                                            bool                brake,
                                            bool                hasPresetOverride,
                                            Config::SpeedPreset preset)
{
    // Apply preset first so that, if anyone ever extends this to react synchronously to the
    // enable transition, they see the freshly chosen cruise.
    if (hasPresetOverride)
    {
        _preset = preset;
    }

    if (targetEnabled)
    {
        // Enable path: just flip the flag. The next update() tick will issue setSpeed.
        // No Drive.stop(...) call here regardless of `brake` — by contract `setBrake(...)` is
        // only meaningful on disable.
        _enabled = true;
    }
    else
    {
        // Disable path. Iron rule: always flip the flag and always clear `_lastErr` and
        // `_lostLine`. The brake choice ONLY affects whether `Drive.stop(...)` fires.
        _enabled  = false;
        _lastErr  = 0.0f;
        _lostLine = false;

        if (brake)
        {
            Drive.stop(true);
        }
        // brake == false: caller wants to coast. Leave Drive on whatever speeds it last
        // latched; the path planner is responsible for issuing the next move.
    }
}

// =========================================================================================
// Accessors.
// =========================================================================================

bool LineFollowerClass::isEnabled() const
{
    return _enabled;
}

bool LineFollowerClass::isLineLost() const
{
    return _lostLine;
}

float LineFollowerClass::getRawError() const
{
    return _lastErr;
}

MatrixLineTracer* LineFollowerClass::getTracer()
{
    return &_tracer;
}

Config::SpeedPreset LineFollowerClass::getPreset() const
{
    return _preset;
}

// =========================================================================================
// Private helpers.
// =========================================================================================

int16_t LineFollowerClass::_presetToPwm(Config::SpeedPreset preset) const
{
    switch (preset)
    {
        case Config::SpeedPreset::SLOW:
            return Config::SPEED_SLOW;
        case Config::SpeedPreset::NORMAL:
            return Config::SPEED_NORMAL;
        case Config::SpeedPreset::FAST:
            return Config::SPEED_FAST;
    }
    return Config::SPEED_NORMAL;    // unreachable; calms the compiler on the no-default switch.
}
