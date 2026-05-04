/**
 * @file LineFollower.cpp
 * @brief Implementation of the outer P+D line-following controller.
 * @author WRO 2026
 */

#include "LineFollower.h"

#include "Drive.h"

/**
 * @brief Single global instance referenced by every other module.
 */
LineFollowerClass LineFollower;

// -----------------------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------------------

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

    return true;
}

// -----------------------------------------------------------------------------------------
// Cooperative tick
// -----------------------------------------------------------------------------------------

void LineFollowerClass::update()
{
    if (!_enabled)
    {
        return;
    }

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
                Drive.setSpeed(Config::CRUISE_SPEED / 2, Config::CRUISE_SPEED / 2);
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

    // P+D only — Phase 1 deliberately omits the integral term (Config::LINE_KI is unused).
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

    // Differential mix around the cruise setpoint.
    int16_t L = (int16_t) (Config::CRUISE_SPEED + corr);
    int16_t R = (int16_t) (Config::CRUISE_SPEED - corr);

    // Re-clamp each channel to the DriveDC PWM range.
    L = (int16_t) constrain(L, -100, 100);
    R = (int16_t) constrain(R, -100, 100);

    Drive.setSpeed(L, R);
}

// -----------------------------------------------------------------------------------------
// State control
// -----------------------------------------------------------------------------------------

void LineFollowerClass::setEnabled(bool on)
{
    _enabled = on;
    if (!on)
    {
        // Brake immediately and clear the D-term history so the next enable does not see a
        // spurious derivative spike from the stale `_lastErr`.
        Drive.stop(true);
        _lastErr = 0.0f;
    }
}

// -----------------------------------------------------------------------------------------
// Accessors
// -----------------------------------------------------------------------------------------

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
