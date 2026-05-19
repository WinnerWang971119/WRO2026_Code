/**
 * @file LineFollower.cpp
 * @brief Implementation of the outer P+D line-following controller and its fluent builder.
 * @author WRO 2026
 */

#include "LineFollower.h"

#include "Drive.h"
#include "Intersection.h"
#include "WROLineArray.h"    // Direct include for IWYU clarity — `LineFollower.h` already
                             // transitively brings this in, but binding to `&LineArray` and
                             // dereferencing `WROLineArrayClass*` makes the dependency
                             // explicit at the use site.

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
                               _tShapeRampUp,
                               _tShapeRampDown,
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
    // Idempotent — `_tShapeRampUp` defaults to `true`. Kept as an explicit-readability alias
    // for callers that want the chain to spell out "yes, ramp on this enable transition."
    // The symmetric ramp-down is governed by `setTShapeRampDown(bool)` and is also on by
    // default; this setter does NOT touch it (see header doxygen).
    _tShapeRampUp = true;
    return *this;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setTShapeRampUp(bool on)
{
    _tShapeRampUp = on;
    return *this;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setTShapeRampDown(bool on)
{
    _tShapeRampDown = on;
    return *this;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setAccel(float pwmPerCm)
{
    _accel         = pwmPerCm;
    _accelExplicit = true;
    // setAccel implies a ramp is desired — turning on ramp-up matches the previous
    // contract. Ramp-down stays at its own default (also on); a caller who wants only
    // ramp-down can chain `.setTShapeRampUp(false)` afterwards.
    _tShapeRampUp  = true;
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
    // Bind to the WROLineArrayClass singleton. The sketch's setup() is
    // responsible for calling LineArray.begin() BEFORE LineFollower.begin()
    // so the Layer-1 hardware calibration (setLight()) and per-board mux
    // channels are already up. See DECISIONS.md "8-channel custom line
    // sensor" and the WROLineArrayClass class header.
    _array = &LineArray;

    _enabled           = false;
    _lostLine          = false;
    _lastErr           = 0.0f;
    _preset            = Config::SpeedPreset::NORMAL;
    _reverse           = false;
    _useTShape         = false;
    _useTShapeRampDown = false;

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
        // If a distance gate is armed AND a ramp-down is also active, clamp the ramp-up to
        // at most half the distance target so the two zones cannot overlap. Without this
        // clamp the ramp-down branch's lerp would fight the ramp-up branch's lerp on very
        // short distance moves. The ramp-down branch applies a matching half-clamp.
        if (_distanceTargetActive && _useTShapeRampDown)
        {
            const float halfCm = _distanceTargetCm * 0.5f;
            if (rampCm > halfCm)
            {
                rampCm = halfCm;
            }
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

    // -------------------------------------------------------------------------------------
    // Trapezoidal ramp-DOWN before the distance gate. Lerps cruise from the captured
    // `_tShapeRampDownStart` PWM down to `Config::PWM_MIN_MAGNITUDE` (sign-corrected for
    // reverse) over the approach distance, so when the gate trips the active brake fires
    // at floor PWM rather than from full cruise. Runs INDEPENDENTLY of the ramp-up branch
    // above — both can apply in sequence over a long PROJECTION_FOLLOW edge.
    //
    // See DECISIONS.md "LineFollower distance-gated stops use a symmetric trapezoidal
    // profile" for the full rationale.
    // -------------------------------------------------------------------------------------
    if (_useTShapeRampDown && _distanceTargetActive)
    {
        // Average wheel deltas (same baseline as the distance gate below).
        const int32_t dL          = Drive.getEncL() - _distanceStartEncL;
        const int32_t dR          = Drive.getEncR() - _distanceStartEncR;
        const float   traveledCm  = encDegToCm((abs(dL) + abs(dR)) / 2);
        const float   remainingCm = _distanceTargetCm - traveledCm;

        // Sign-by-direction on the floor target: when `_tShapeRampDownStart` is negative
        // (reverse), the floor is `-PWM_MIN_MAGNITUDE` so the lerp stays on the negative
        // side of zero and the deadband floor is honored in reverse too.
        const int16_t targetFloor = (_tShapeRampDownStart < 0)
                                        ? (int16_t) -Config::PWM_MIN_MAGNITUDE
                                        : (int16_t) Config::PWM_MIN_MAGNITUDE;

        // Default geometry: how far the ramp WOULD want, given full cruise range and the
        // configured accel. Clamped to global ramp bounds, then fitted to the available
        // distance. With ramp-up active, at most half the distance is rampdown (the
        // ramp-up branch above applies a matching half-clamp). Without ramp-up (the
        // re-arm path used by LINE_OVERSHOOT), the ramp-down may consume the full travel.
        const float defaultRampCm =
            (_tShapeAccel > 0.0f)
                ? ((float) abs(_tShapeRampDownStart - targetFloor) / _tShapeAccel)
                : 0.0f;
        const float distanceClamp = _useTShape ? (_distanceTargetCm * 0.5f)
                                                : _distanceTargetCm;
        float rampDownCm = (defaultRampCm < distanceClamp) ? defaultRampCm : distanceClamp;
        if (rampDownCm < Config::TSHAPE_RAMP_MIN_CM)
        {
            rampDownCm = Config::TSHAPE_RAMP_MIN_CM;
        }
        if (rampDownCm > Config::TSHAPE_RAMP_MAX_CM)
        {
            rampDownCm = Config::TSHAPE_RAMP_MAX_CM;
        }

        if (remainingCm <= rampDownCm)
        {
            // In the ramp-down zone. Lerp cruise from `_tShapeRampDownStart` toward
            // `targetFloor` as `remaining` shrinks to 0. When `remaining` <= 0 (we have
            // already reached or overshot the gate), pin to `targetFloor` until the
            // distance-gate branch below fires the active brake.
            const float r        = (rampDownCm > 0.0f) ? (remainingCm / rampDownCm) : 0.0f;
            const float clampedR = (r < 0.0f) ? 0.0f : ((r > 1.0f) ? 1.0f : r);
            cruise               = (int16_t) (targetFloor
                                + (int16_t) ((_tShapeRampDownStart - targetFloor) * clampedR));
        }
        // else: still in the cruise plateau zone — leave `cruise` unchanged.
    }

    if (!_array->isOnline())
    {
        // Clear the D-term history so the first tick after reacquisition does not see a stale
        // derivative computed against an old (possibly several-second-stale) error sample.
        _lastErr = 0.0f;

        // Line lost beneath every channel. Dispatch on Config::LOST_LINE_ACTION.
        _lostLine = true;
        // Clear the junction freeze flag — if a lost-line condition fires while frozen,
        // we treat lost-line as the authoritative path and want the next online tick to
        // start fresh (no stale freeze flag carrying over).
        _pidFrozen = false;

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
    // Line online: junction-freeze gate, then P+D controller.
    // -------------------------------------------------------------------------------------
    _lostLine = false;

    // Sample the junction classifier ONCE per tick. With Config::LINE_ARRAY_TICK_CACHE_ENABLED
    // == true this is a free cache read; with caching off it is one ~9 ms I2C transaction.
    // Intersection.update() runs after us and will issue its own call against the same sensor
    // — eliminating that double-read is tracked separately (out of scope for issue #22).
    //
    // Freeze fires on any classifier value other than NONE / UNKNOWN — broadened from the
    // original issue #22 scope (T_OR_CROSS only) because the centroid-skew defect affects
    // LEFT and RIGHT junctions just as much (the issue body itself noted this). See
    // DECISIONS.md "LineFollower PID freeze during junction dwell".
    const uint8_t junctionRaw = _array->getJunctionType();
    const bool    atJunction  = (junctionRaw != (uint8_t) JunctionType::NONE
                              && junctionRaw != (uint8_t) JunctionType::UNKNOWN);

    if (atJunction)
    {
        _pidFrozen = true;

        // Command symmetric cruise. `cruise` already reflects ramp-up / ramp-down progress
        // (modified by the branches above) and is sign-flipped for reverse. Drive.setSpeed
        // applies per-motor bias calibration automatically (DECISIONS.md "Strategy B"), so
        // no additional bias handling is needed here. deadbandSnap mirrors the snap applied
        // in the P+D path so a cruise value inside the deadband range (rare but possible
        // during ramp-up) is not silently dropped by Drive.setSpeed's internal re-snap.
        const int16_t snapped = deadbandSnap(cruise);
        Drive.setSpeed(snapped, snapped);

        // Fall through to the distance-gate block below — do NOT early return. The encoder
        // distance counter must keep ticking during the freeze (acceptance criterion).
    }
    else
    {
        if (_pidFrozen)
        {
            // First non-frozen tick after a freeze: clear the D-term history. Without this
            // the dErr computed against a pre-freeze `_lastErr` (potentially several ticks
            // stale) would produce a derivative kick on the first post-junction PID tick.
            // Mirrors the lost-line branch's existing `_lastErr = 0.0f;` reset.
            _lastErr   = 0.0f;
            _pidFrozen = false;
        }

        const float err  = _array->getError();
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
        // This compensates for both the saturation cap and the motor deadband floor: a small
        // P+D correction in the (0, 20) range would otherwise be silently lost in the dead
        // zone. Drive.setSpeed re-snaps internally — the snap here ensures the value passed
        // in already reflects the post-mix correction so future readers see a consistent
        // post-snap value.
        L = deadbandSnap(L);
        R = deadbandSnap(R);

        Drive.setSpeed(L, R);
    }

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
            // Self-disable with active brake. Delegating to the builder's apply path keeps
            // the disable semantics identical to a caller-issued setEnabled(false) —
            // including clearing _lastErr / _lostLine / _useTShape.
            //
            // The latch is set AFTER the apply call because the disable path itself clears
            // `_distanceReached` (so a user-initiated disable cannot leak a stale trip into
            // a later re-enable). Setting it before would be immediately clobbered, leaving
            // `isDistanceReached()` permanently false and hanging PathTracker's
            // PROJECTION_FOLLOW state until its drain-timeout fault fires.
            _applyEnableBuilder(/*targetEnabled=*/false,
                                /*brake=*/true,
                                /*hasPresetOverride=*/false,
                                /*preset=*/_preset,
                                /*tShapeRampUp=*/false,
                                /*tShapeRampDown=*/false,
                                /*accel=*/Config::TSHAPE_ACCEL_PWM_PER_CM,
                                /*hasReverseOverride=*/false,
                                /*reverse=*/_reverse,
                                /*hasDistanceTarget=*/false,
                                /*distanceTargetCm=*/0.0f);
            _distanceReached = true;
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
                                            bool                tShapeRampUp,
                                            bool                tShapeRampDown,
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
        // Propagate direction to the sensor so 2-board configurations swap to the rear
        // board atomically with the controller flipping cruise sign. The nullptr guard
        // makes this a silent no-op if the builder somehow fires before begin() runs —
        // same defensive pattern used elsewhere in this file.
        if (_array != nullptr)
        {
            _array->setReverseActive(_reverse);
        }
    }

    if (targetEnabled)
    {
        // Snapshot the previous enable state BEFORE we flip the flag. The re-arm guard
        // below distinguishes a true off→on transition (seed ramp-up from 0) from a
        // re-enable on an already-running controller (preserve in-flight ramp, do NOT
        // reset cruise to 0 — that is the Copilot-flagged PR12 bug). See DECISIONS.md
        // "LineFollower distance-gated stops use a symmetric trapezoidal profile".
        const bool wasAlreadyEnabled = _enabled;
        _enabled = true;

        // Ramp-up arming: only seed on a true off→on transition. A `setEnabled(true)` issued
        // while the controller is already enabled (e.g. PathTracker re-arming the distance
        // gate mid-segment from LINE_FOLLOW into LINE_OVERSHOOT) must NOT reset cruise — it
        // must preserve whatever ramp/plateau state was already in flight.
        //
        // Sign-by-direction on the target PWM: when `_reverse` is set, the steady-state
        // `cruise` in `update()` is negated, so the ramp target must be negated too —
        // otherwise the ramp interpolation overrides the correctly-signed cruise with a
        // positive value during the ramp window and the chassis briefly drives forward
        // before reversing (see code review F2).
        const bool shouldRampUp = tShapeRampUp && !wasAlreadyEnabled;
        if (shouldRampUp)
        {
            _useTShape       = true;
            _tShapeAccel     = accel;
            _tShapeStartEnc  = Drive.getEncL();
            _tShapeStartPwm  = 0;
            _tShapeTargetPwm = _reverse ? (int16_t) -presetToPwm(_preset)
                                        : presetToPwm(_preset);
        }
        else if (!tShapeRampUp)
        {
            // Caller explicitly opted out of ramp-up — kill any in-flight ramp.
            _useTShape = false;
        }
        // else (wasAlreadyEnabled && tShapeRampUp): leave `_useTShape` alone so any
        // in-flight ramp keeps progressing.

        // Capture the accel for the ramp-down even when ramp-up did not fire, so a
        // re-arm-only path (LINE_OVERSHOOT) still has a usable accel.
        if (!shouldRampUp)
        {
            _tShapeAccel = accel;
        }

        // Ramp-down arming: only meaningful with a distance target. Capture the cruise PWM
        // the ramp-down will descend FROM at gate-arm time. The math in `update()` lerps
        // cruise from this value down to `Config::PWM_MIN_MAGNITUDE` (sign-corrected) as
        // the chassis approaches the gate, so the active brake fires at floor PWM rather
        // than from full cruise — eliminates the lower-MCU brake-from-cruise stopping
        // distance variance.
        _useTShapeRampDown = tShapeRampDown && hasDistanceTarget;
        if (_useTShapeRampDown)
        {
            _tShapeRampDownStart = _reverse ? (int16_t) -presetToPwm(_preset)
                                            : presetToPwm(_preset);
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
            // baseline cannot fire mid-segment after the new enable. Ramp-down also makes
            // no sense without a gate to ramp toward.
            _distanceTargetActive = false;
            _useTShapeRampDown    = false;
        }
    }
    else
    {
        // Disable path. Iron rule: always flip the flag and always clear `_lastErr`,
        // `_lostLine`, and `_pidFrozen`. The brake choice ONLY affects whether
        // `Drive.stop(...)` fires. `_pidFrozen` is cleared here so a freeze that was
        // active at disable time cannot leak into the next enable cycle — AC #8 / AC #10.
        _enabled              = false;
        _lastErr              = 0.0f;
        _lostLine             = false;
        _pidFrozen            = false;    // Clear junction-freeze flag — no stale state on re-enable.
        _useTShape            = false;    // Disabling cancels any active cruise ramp-up.
        _useTShapeRampDown    = false;    // Disabling also cancels any active ramp-down.
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

bool LineFollowerClass::isPidFrozen() const
{
    return _pidFrozen;
}

float LineFollowerClass::getRawError() const
{
    return _lastErr;
}

WROLineArrayClass* LineFollowerClass::getArray()
{
    return _array;
}

bool LineFollowerClass::isDistanceReached()
{
    const bool latched  = _distanceReached;
    _distanceReached    = false;
    return latched;
}

void LineFollowerClass::setTickCallback(void (*cb)())
{
    _tickCallback                = cb;
    _tickCallbackWarningEmitted  = false;    // reset so re-register clears the warned state
}

bool LineFollowerClass::followDistance(float cm)
{
    if (!Drive.isIdle())
    {
        return false;
    }

    setEnabled(true).setDistanceTarget(cm);
    return waitUntilDistanceReached();
}

bool LineFollowerClass::waitUntilDistanceReached()
{
    if (!_enabled || !_distanceTargetActive || !Drive.isIdle())
    {
        return false;
    }

    const uint32_t startMs = millis();

    while (_enabled)
    {
        if (!Drive.isIdle()
            || (millis() - startMs) >= Config::LINE_FOLLOW_WAIT_TIMEOUT_MS)
        {
            setEnabled(false);
            return false;
        }

        // Sync-blocking helpers must keep the rest of the cooperative tick alive — in
        // particular `LineArray.update()` when tick-cache mode is enabled. Mirror
        // `Drive::_drainBlocking`: dispatch the registered `cooperativeTick`-style
        // callback, falling back to local `update()` with a one-shot warning if no
        // callback has been registered yet.
        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            if (!_tickCallbackWarningEmitted)
            {
                Serial.println(F("[LineFollower] waitUntilDistanceReached: no tick "
                                 "callback set — only LineFollower.update() will tick "
                                 "during the wait. Call LineFollower.setTickCallback(...) "
                                 "in setup()."));
                _tickCallbackWarningEmitted = true;
            }
            update();
        }
        yield();
    }

    return isDistanceReached();
}

bool LineFollowerClass::followUntilIntersection()
{
    if (!Drive.isIdle())
    {
        return false;
    }

    setEnabled(true);
    return waitUntilIntersection();
}

bool LineFollowerClass::waitUntilIntersection()
{
    if (!_enabled || !Drive.isIdle())
    {
        return false;
    }

    // Consume one stale latched edge, if any, so this wait targets the next fresh intersection
    // after entry rather than a previous segment's leftover edge.
    const bool clearedStaleEdge = Intersection.isAtIntersection();
    (void) clearedStaleEdge;

    const uint32_t startMs = millis();
    while (_enabled)
    {
        if (!Drive.isIdle()
            || (millis() - startMs) >= Config::LINE_FOLLOW_WAIT_TIMEOUT_MS)
        {
            setEnabled(false);
            return false;
        }

        // Match the documented loop order from Intersection.h and WRO2026.ino: LineArray
        // refresh first, then LineFollower, then Intersection reads the same shared
        // tracer state on this iteration. `cooperativeTick()` itself preserves that
        // order — see WRO2026.ino. The fallback path (no callback registered) keeps
        // the legacy explicit `update(); Intersection.update();` pair so a sketch that
        // forgets the registration still produces correct (if less complete) behaviour
        // with a one-shot Serial warning.
        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            if (!_tickCallbackWarningEmitted)
            {
                Serial.println(F("[LineFollower] waitUntilIntersection: no tick "
                                 "callback set — only LineFollower.update() + "
                                 "Intersection.update() will tick during the wait. "
                                 "Call LineFollower.setTickCallback(...) in setup()."));
                _tickCallbackWarningEmitted = true;
            }
            update();
            Intersection.update();
        }
        if (Intersection.isAtIntersection())
        {
            setEnabled(false);
            return true;
        }

        yield();
    }

    return false;
}

Config::SpeedPreset LineFollowerClass::getPreset() const
{
    return _preset;
}

void LineFollowerClass::setReverse(bool reverse)
{
    _reverse = reverse;
    // Mirror the apply-builder path so the non-fluent setter keeps the sensor's active
    // board in sync with the controller's direction. nullptr-guarded for the same
    // pre-begin() safety reason.
    if (_array != nullptr)
    {
        _array->setReverseActive(_reverse);
    }
}

bool LineFollowerClass::getReverse() const
{
    return _reverse;
}

// `_presetToPwm` removed — see `presetToPwm` in WROMath.h. Internal callers above invoke
// the free function directly.
