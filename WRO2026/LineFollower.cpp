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

    _owner._applyEnableBuilder(_targetEnabled,
                               _brake,
                               _hasPresetOverride,
                               _presetOverride,
                               _tShape,
                               _accel,
                               _hasReverseOverride,
                               _reverseOverride,
                               _hasDistanceTarget,
                               _distanceTargetCm);
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

LineFollowerEnableBuilder& LineFollowerEnableBuilder::tShape()
{
    _tShape = true;
    return *this;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setAccel(float pwmPerCm)
{
    _accel         = pwmPerCm;
    _accelExplicit = true;
    _tShape        = true;
    return *this;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setReverse(bool reverse)
{
    _reverseOverride    = reverse;
    _hasReverseOverride = true;
    return *this;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setDistanceTarget(float cm)
{
    // Clamp non-positive values to zero — gate trips on the first tick (equivalent to
    // "disable now"). Negative inputs are almost certainly a caller bug, but failing
    // loud here would propagate into a fault that the call site cannot fix; clamping
    // keeps the contract simple.
    _distanceTargetCm  = (cm > 0.0f) ? cm : 0.0f;
    _hasDistanceTarget = true;
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

    _enabled   = false;
    _lostLine  = false;
    _lastErr   = 0.0f;
    _preset    = Config::SpeedPreset::NORMAL;
    _reverse   = false;
    _useTShape = false;

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
    // L/R mix symmetric and avoids any chance of preset mid-tick flip-flopping. When
    // `_reverse` is set, negate cruise so the chassis drives backward — the P+D mix below
    // works unchanged because differential-drive geometry is direction-symmetric: a CW body
    // rotation moves the front-mounted tracer the same way regardless of whether the body
    // is travelling forward or backward. See DECISIONS.md "Reverse line-following".
    int16_t cruise = presetToPwm(_preset);
    if (_reverse)
    {
        cruise = (int16_t) -cruise;
    }

    if (_useTShape)
    {
        // Distance-based ramp from `_tShapeStartPwm` to `_tShapeTargetPwm`. The ramp distance
        // is implicit: `|cruise - start| / accel`, clamped to `[MIN_CM, MAX_CM]`. The
        // shared `encDegToCm()` helper (WROMath.h) is the single source of truth for the
        // inverse of `cmToEncDeg` — this used to duplicate the math inline.
        const int32_t now      = Drive.getEncL();
        const int32_t delta    = now - _tShapeStartEnc;
        // Use the absolute value so a momentary counter-rotation cannot drive `traveledCm`
        // negative and pull `cruise` below the starting PWM.
        const float traveledCm = encDegToCm(abs(delta));

        float rampCm = (float) abs(cruise - _tShapeStartPwm) / _tShapeAccel;
        if (rampCm < Config::TSHAPE_RAMP_MIN_CM)
        {
            rampCm = Config::TSHAPE_RAMP_MIN_CM;
        }
        if (rampCm > Config::TSHAPE_RAMP_MAX_CM)
        {
            rampCm = Config::TSHAPE_RAMP_MAX_CM;
        }

        if (traveledCm >= rampCm)
        {
            // Ramp consumed — switch to full preset cruise for the rest of this enable
            // session. Subsequent ticks short-circuit this branch entirely.
            _useTShape = false;
        }
        else
        {
            const float r = (rampCm > 0.0f) ? (traveledCm / rampCm) : 1.0f;
            cruise = (int16_t) (_tShapeStartPwm
                                + (int16_t) ((_tShapeTargetPwm - _tShapeStartPwm) * r));
        }
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
            {
                // Straight creep forward (no steering) so the strip may be reacquired.
                // `cruise / 2` is already in-range, but snap for consistency with the main
                // mix path so every PWM hitting the lower MCU goes through the same gate.
                const int16_t crawl = deadbandSnap((int16_t) (cruise / 2));
                Drive.setSpeed(crawl, crawl);
                break;
            }
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

    // Snap each channel into {0} U [PWM_MIN_MAGNITUDE, PWM_MAX] (mirrored for negatives).
    // This compensates for both the saturation cap and the motor deadband floor: a small P+D
    // correction in the (0, 20) range would otherwise be silently lost in the dead zone.
    // Drive.setSpeed re-snaps internally — the snap here ensures the value passed in already
    // reflects the post-mix correction so future readers see a consistent post-snap value.
    L = deadbandSnap(L);
    R = deadbandSnap(R);

    Drive.setSpeed(L, R);

    // -------------------------------------------------------------------------------------
    // Distance gate (PathTracker PROJECTION_FOLLOW). Evaluated AFTER the PID issues the
    // correction so the wheels are commanded one last time on the trip-tick before the
    // controller self-disables and brakes. Sampling both wheels and averaging the absolute
    // deltas guards against a momentary single-wheel slip pulling the gate too early.
    // -------------------------------------------------------------------------------------
    if (_distanceTargetActive)
    {
        const int32_t dL = Drive.getEncL() - _distanceStartEncL;
        const int32_t dR = Drive.getEncR() - _distanceStartEncR;
        const int32_t avgDeg = (abs(dL) + abs(dR)) / 2;
        const float traveledCm = encDegToCm(avgDeg);
        if (traveledCm >= _distanceTargetCm)
        {
            // Trip the latch and self-disable with active brake. Delegating to the
            // builder's apply path keeps the disable semantics identical to a caller-issued
            // setEnabled(false) — including clearing _lastErr / _lostLine / _useTShape.
            _distanceReached = true;
            _applyEnableBuilder(/*targetEnabled=*/false,
                                /*brake=*/true,
                                /*hasPresetOverride=*/false,
                                /*preset=*/_preset,
                                /*tShape=*/false,
                                /*accel=*/Config::TSHAPE_ACCEL_PWM_PER_CM,
                                /*hasReverseOverride=*/false,
                                /*reverse=*/_reverse,
                                /*hasDistanceTarget=*/false,
                                /*distanceTargetCm=*/0.0f);
        }
    }
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
                                            Config::SpeedPreset preset,
                                            bool                tShape,
                                            float               accel,
                                            bool                hasReverseOverride,
                                            bool                reverse,
                                            bool                hasDistanceTarget,
                                            float               distanceTargetCm)
{
    // Apply preset and reverse first so that, if anyone ever extends this to react
    // synchronously to the enable transition, they see the freshly chosen cruise + direction.
    if (hasPresetOverride)
    {
        _preset = preset;
    }
    if (hasReverseOverride)
    {
        _reverse = reverse;
    }

    if (targetEnabled)
    {
        // Enable path: just flip the flag. The next update() tick will issue setSpeed.
        // No Drive.stop(...) call here regardless of `brake` — by contract `setBrake(...)` is
        // only meaningful on disable.
        _enabled = true;

        if (tShape)
        {
            // Seed the cruise-ramp state. The ramp progresses with the left encoder; once
            // the ramp distance is consumed, `update()` clears `_useTShape` so subsequent
            // ticks fall straight through to the full preset cruise.
            //
            // Sign-by-direction on the target PWM: when `_reverse` is set, the steady-state
            // `cruise` in `update()` is negated, so the ramp target must be negated too —
            // otherwise the ramp interpolation overrides the correctly-signed cruise with a
            // positive value during the ramp window and the chassis briefly drives forward
            // before reversing (see code review F2).
            _useTShape       = true;
            _tShapeAccel     = accel;
            _tShapeStartEnc  = Drive.getEncL();
            _tShapeStartPwm  = 0;
            _tShapeTargetPwm = _reverse ? (int16_t) -presetToPwm(_preset)
                                        : presetToPwm(_preset);
        }
        else
        {
            _useTShape = false;
        }

        if (hasDistanceTarget)
        {
            // Arm the encoder-distance gate. Capture both wheels' counters as the
            // baseline so a single-wheel encoder hiccup cannot bias the average.
            // Clear the latch — a re-arm after a previous gate trip starts fresh.
            _distanceTargetActive = true;
            _distanceTargetCm     = distanceTargetCm;
            _distanceStartEncL    = Drive.getEncL();
            _distanceStartEncR    = Drive.getEncR();
            _distanceReached      = false;
        }
        else
        {
            // Re-enabling without a distance target disarms any prior gate so a stale
            // baseline cannot fire mid-segment after the new enable.
            _distanceTargetActive = false;
        }
    }
    else
    {
        // Disable path. Iron rule: always flip the flag and always clear `_lastErr` and
        // `_lostLine`. The brake choice ONLY affects whether `Drive.stop(...)` fires.
        _enabled              = false;
        _lastErr              = 0.0f;
        _lostLine             = false;
        _useTShape            = false;    // Disabling cancels any active cruise ramp.
        _distanceTargetActive = false;    // Disabling also disarms the distance gate.
        _distanceReached      = false;    // Clear the latch so a re-enable without a new
                                          // setDistanceTarget cannot report a stale trip.

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

bool LineFollowerClass::isDistanceReached()
{
    const bool latched  = _distanceReached;
    _distanceReached    = false;
    return latched;
}

Config::SpeedPreset LineFollowerClass::getPreset() const
{
    return _preset;
}

void LineFollowerClass::setReverse(bool reverse)
{
    _reverse = reverse;
}

bool LineFollowerClass::getReverse() const
{
    return _reverse;
}

// `_presetToPwm` removed — see `presetToPwm` in WROMath.h. Internal callers above invoke
// the free function directly.
