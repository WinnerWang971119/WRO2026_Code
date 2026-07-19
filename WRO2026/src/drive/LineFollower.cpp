/**
 * @file LineFollower.cpp
 * @brief Implementation of the outer P+D line-following controller and its fluent builder.
 * @author WRO 2026
 */

#include "LineFollower.h"

#include "Drive.h"
#include "Intersection.h"
#include "../perception/WROLineArray.h"    // Direct include for IWYU clarity — `LineFollower.h` already
                             // transitively brings this in, but binding to `&LineArray` and
                             // dereferencing `WROLineArrayClass*` makes the dependency
                             // explicit at the use site.
#include "../util/WROMath.h"    // `lineFollowModeToSetpoint(...)` — maps the per-edge follow mode to
                             // a signed offset. Pulls the full `Routes::LineFollowMode` definition
                             // (the header only opaque-forward-declares it) into this TU.

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
                               _distanceTargetCm,
                               _hasOvershootModeOverride,
                               _overshootModeOverride,
                               _hasBrakeOnGateOverride,
                               _brakeOnGateOverride,
                               _hasLineFollowOffsetOverride,
                               _lineFollowOffsetOverride,
                               _hasApproachTarget,
                               _approachPredictedCm);
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

LineFollowerEnableBuilder& LineFollowerEnableBuilder::flat()
{
    // Sugar over the two existing opt-outs — LineFollower has two independent ramp
    // phases (ramp-up on enable, ramp-down before a distance gate), so a flat session
    // means clearing both. See header doxygen and ADR-034.
    _tShapeRampUp   = false;
    _tShapeRampDown = false;
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

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setOvershootMode(bool on)
{
    // Mirrors the setPreset / setReverse override-pair pattern: stage the flag plus a
    // "has-override" marker so the owner's existing value is preserved across re-enables
    // when the setter is not chained. See ADR-023.
    _overshootModeOverride    = on;
    _hasOvershootModeOverride = true;
    return *this;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setBrakeOnGate(bool on)
{
    // Mirrors the setPreset / setReverse override-pair pattern: stage the flag plus a
    // "has-override" marker so the owner's existing value is preserved across re-enables
    // when the setter is not chained. See ADR-023.
    _brakeOnGateOverride    = on;
    _hasBrakeOnGateOverride = true;
    return *this;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setLineFollowOffset(float offset)
{
    // Mirrors the setOvershootMode / setBrakeOnGate override-pair pattern: stage the value
    // plus a "has-override" marker so the owner's existing offset is preserved across a re-arm
    // when the setter is not chained. Unlike setPreset / setReverse, the owner RESETS this to
    // 0.0f on disable (see _applyEnableBuilder's disable branch), so a later followDistance() /
    // followUntilIntersection() never inherits a stale bias. See issue #156.
    _lineFollowOffsetOverride    = offset;
    _hasLineFollowOffsetOverride = true;
    return *this;
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setLineFollowMode(Routes::LineFollowMode mode)
{
    // Ergonomic sugar over setLineFollowOffset: map the per-edge mode to the same signed offset the
    // float knob takes, then delegate — so the staging, reset-on-disable lifecycle, and P+D maths
    // are byte-identical to the float path. FORWARD-frame only; the reverse-hop sign flip stays in
    // PathTracker's float call (ADR-069). DEFAULT resolves to MIDDLE (offset 0) in the mapper.
    return setLineFollowOffset(lineFollowModeToSetpoint(mode));
}

LineFollowerEnableBuilder& LineFollowerEnableBuilder::setApproachTarget(float predictedCm)
{
    // Stage the predicted sensor-crossing distance plus a "has-target" marker. The owner
    // resolves and arms the ApproachSchedule in _applyEnableBuilder (only on an enable
    // transition, feature-gated). Unlike the override-pair setters, ABSENCE of this setter on
    // an enable apply DISARMS any prior schedule — that disarm-on-omission is the LINE_OVERSHOOT
    // re-arm contract, handled in _applyEnableBuilder, not here.
    //
    // Clamp negatives to 0 for symmetry with setDistanceTarget: a negative prediction is almost
    // certainly a caller bug, and 0 degrades the schedule to inactive (flat path, AC7) instead
    // of propagating garbage into the resolver.
    _approachPredictedCm = (predictedCm > 0.0f) ? predictedCm : 0.0f;
    _hasApproachTarget   = true;
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
    _seedLastErr       = false;
    _preset            = Config::SpeedPreset::NORMAL;
    _reverse           = false;
    _useTShape         = false;
    _useTShapeRampDown = false;
    _approachActive    = false;
    _approachPhase     = ApproachPhase::CRUISE;
    _approachLastMag   = 0;

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
        // Measure ramp-up progress from the both-wheel average, identical to the baseline
        // the ramp-down branch (below) and the distance gate (bottom of update()) use. A
        // single-wheel measure drifts from the symmetric gate measure on a curvy approach
        // where one wheel slips or steers harder, producing a small cruise discontinuity at
        // the ramp seam. See trace finding #20.
        const int32_t now      = (Drive.getEncL() + Drive.getEncR()) / 2;
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

    // Emit floor for this tick's Drive.setSpeed calls. Defaults to the static deadband
    // floor and is lowered to the kinetic floor ONLY inside the stop ramp-down zone, or in
    // the ramp-down plateau below when the seeded `_tShapeRampDownStart` is itself already
    // sub-static (the approach-ramp case) — mid-follow PID, lost-line CRAWL, and a legacy
    // (PROJECTION_FOLLOW) plateau seeded at full preset cruise must all keep the static
    // floor (a stationary chassis, or one whose inside wheel is near-stalled through a turn,
    // needs the higher static-friction breakaway PWM), while a chassis already crawling at
    // or below the kinetic floor can stay there without stalling. See the PWM_MOVING_MIN
    // @warning in ConfigGlobal.h for the rationale.
    int16_t stopFloorMag = Config::PWM_MIN_MAGNITUDE;

    // -------------------------------------------------------------------------------------
    // Trapezoidal ramp-DOWN before the distance gate. Lerps cruise from the captured
    // `_tShapeRampDownStart` PWM down to `Config::PWM_MOVING_MIN` (the kinetic floor,
    // sign-corrected for reverse) over the approach distance, so when the gate trips the
    // active brake fires at the kinetic floor PWM rather than from full cruise. Runs
    // INDEPENDENTLY of the ramp-up branch above — both can apply in sequence over a long
    // PROJECTION_FOLLOW edge.
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
        // (reverse), the floor is `-PWM_MOVING_MIN` so the lerp stays on the negative
        // side of zero and the kinetic floor is honored in reverse too. The tail lerps to
        // the kinetic floor (not the static deadband floor) so the crawl right before the
        // gate trips does not stall against static friction.
        const int16_t targetFloor = (_tShapeRampDownStart < 0)
                                        ? (int16_t) -Config::PWM_MOVING_MIN
                                        : (int16_t) Config::PWM_MOVING_MIN;

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

            // In the stop ramp-down zone: this tick's emit(s) must use the kinetic floor
            // so `deadbandSnap` does not re-snap the lerped-down `cruise` back up to the
            // static floor. See the PWM_MOVING_MIN @warning in ConfigGlobal.h.
            stopFloorMag = Config::PWM_MOVING_MIN;
        }
        else if (!_useTShape)
        {
            // Still in the cruise plateau zone (far from the gate) AND the ramp-UP branch is
            // not concurrently owning `cruise` this tick: HOLD `cruise` at the captured
            // ramp-down start rather than leaving it at the top-of-tick preset default. Gated
            // on `!_useTShape` so a session that also has ramp-up in flight (e.g.
            // `setDistanceTarget(...)` with both ramps on, the historical
            // ramp-up-then-ramp-down combo) is untouched — the ramp-up branch above already
            // owns `cruise` for that portion, exactly as before this fix.
            //
            // For a LEGACY ramp-down (no approach ramp armed this session)
            // `_tShapeRampDownStart == presetToPwm(_preset)` (sign-corrected) already — the T6
            // AC3 seed contract — so holding it here is numerically byte-identical to "leave
            // cruise unchanged" and changes nothing observable pre-feature / with the feature
            // disabled (AC8/AC13). It is ALSO byte-identical immediately after ramp-up
            // completes (`_useTShape` flips false): the ramp-up's own target
            // (`_tShapeTargetPwm == presetToPwm(_preset)`, sign-corrected) already equals
            // `_tShapeRampDownStart` in that no-approach-ramp case.
            //
            // When an approach ramp DID seed a lower magnitude (T11 TC3 finding), the prior
            // "leave unchanged" fell through to `cruise = presetToPwm(_preset)` computed at
            // the top of this tick — full preset cruise — jolting the chassis from the
            // approach floor back UP to cruise for the plateau portion of the overshoot
            // window before ramping back down over the last `rampDownCm`. That violated the
            // AC3 / F2 monotonic detect-handoff guarantee this seed exists to provide.
            // Holding `_tShapeRampDownStart` here keeps the commanded magnitude flat at the
            // seeded (already-reduced) value through the plateau, so the detect→overshoot
            // handoff is genuinely non-increasing.
            //
            // `stopFloorMag` is lowered to the kinetic floor here ONLY when the seed itself
            // is already sub-static (`abs(_tShapeRampDownStart) < PWM_MIN_MAGNITUDE` — the
            // approach-ramp case, where the seed is `PWM_MOVING_MIN`), so `deadbandSnap`
            // cannot re-snap that sub-static-floor seed back up to the static floor
            // mid-plateau. A legacy/PROJECTION_FOLLOW plateau seeds `_tShapeRampDownStart`
            // at full preset cruise (>= SPEED_SLOW, well above the static floor) and must
            // keep `stopFloorMag` at the static floor — lowering it there would let the
            // inside wheel of a turn crawl at the kinetic floor instead of the static
            // breakaway floor, reintroducing the inside-wheel stall the static floor exists
            // to prevent (F4/AC13 PROJECTION_FOLLOW byte-identical guarantee).
            cruise = _tShapeRampDownStart;
            if (abs(_tShapeRampDownStart) < Config::PWM_MIN_MAGNITUDE)
            {
                stopFloorMag = Config::PWM_MOVING_MIN;
            }
        }
        // else (`_useTShape` still true): the ramp-up branch above owns `cruise` this tick —
        // leave it untouched (matches the pre-fix behavior for this combo).
    }

    // -------------------------------------------------------------------------------------
    // Anticipatory approach ramp (2026-07-17 line-approach-ramp plan, T6). Runs ALONGSIDE the
    // `_useTShapeRampDown` branch above but takes PRECEDENCE once past its CRUISE phase: it then
    // OVERWRITES `cruise` and lowers `stopFloorMag` to the kinetic floor, so a freeze or P+D mix
    // later in this tick sees the ramped-DOWN cruise, not full preset cruise (ADR-020 ordering).
    // During the CRUISE phase it only RECORDS phase/magnitude and leaves `cruise` untouched, so
    // the `_useTShape` soft-start ramp-up above survives an off→on entry from rest. Armed by
    // PathTracker via `setApproachTarget(predictedCm)` on a terminating
    // LINE_FOLLOW_INTERSECTION hop; the two ramps are never both meaningfully active on one hop
    // (the LINE hop carries no distance gate, and the distance-gated LINE_OVERSHOOT re-arm
    // disarms this schedule in `_applyEnableBuilder`).
    //
    // The policy ramps cruise from full preset down to `Config::PWM_MOVING_MIN` over the
    // predicted sensor-crossing distance so the leading board is already slow when it physically
    // detects the marker. Detection stays the authoritative stop (PathTracker terminates the hop
    // on `Intersection.isAtIntersection()` and re-anchors `Localization`); this only shapes the
    // commanded magnitude and records the phase for `approachPhase()` so PathTracker can raise a
    // bounded-overrun fault if the line is never seen.
    // -------------------------------------------------------------------------------------
    if (_approachActive)
    {
        // Same both-wheel absolute-average baseline the ramp-down branch and distance gate use,
        // so the approach measure never diverges from them on a curvy follow. See finding #20.
        const int32_t dL         = Drive.getEncL() - _approachStartEncL;
        const int32_t dR         = Drive.getEncR() - _approachStartEncR;
        const float   traveledCm = encDegToCm((abs(dL) + abs(dR)) / 2);

        const ApproachSpeed step = approachSpeedAt(_approachSchedule, traveledCm);
        // Record phase + (unsigned) commanded magnitude UNCONDITIONALLY — every phase, every
        // tick. The phase feeds PathTracker's bounded-overrun fault poll (`approachPhase()`);
        // the magnitude lets the LINE_OVERSHOOT re-arm seed its ramp-down from the floor the
        // chassis actually arrived at, not full preset (AC3).
        _approachPhase   = step.phase;
        _approachLastMag = step.pwmMag;

        // Shape `cruise` ONLY once the schedule is past its CRUISE phase (RAMP / FLOOR_HOLD /
        // OVERRUN). During CRUISE, `step.pwmMag == cruiseMag` (full preset), and overwriting
        // `cruise` here would CLOBBER the `_useTShape` soft-start ramp-up above — commanding
        // full preset from a dead stop on an off→on entry (the launch lurch/slip that
        // TSHAPE_START_SPEED exists to prevent; launch wheel-slip would also corrupt the
        // encoder-derived `traveledCm` this very schedule keys on). On a normal terminating hop
        // `rampStartCm` sits well past the ~11 cm ramp-up, so the two branches never fight —
        // the approach takes over in RAMP.
        if (step.phase != ApproachPhase::CRUISE)
        {
            // Replace the preset cruise with the ramped magnitude, sign-corrected for reverse
            // (mirrors the top-of-tick `cruise = -cruise` reverse handling).
            cruise = _reverse ? (int16_t) -step.pwmMag : step.pwmMag;

            // Past CRUISE the chassis is already moving and can crawl at the kinetic floor,
            // which is BELOW the static deadband floor. Lower `stopFloorMag` so the downstream
            // `deadbandSnap` in the freeze / P+D blocks does not re-snap the ramped-down
            // magnitude back up to `Config::PWM_MIN_MAGNITUDE`. Mirrors the
            // `_useTShapeRampDown` decel tail exactly — do NOT pre-snap the magnitude here; the
            // policy returns pre-snap values and the same downstream snap must apply. See the
            // PWM_MOVING_MIN @warning in ConfigGlobal.h.
            stopFloorMag = Config::PWM_MOVING_MIN;
        }
    }

    // Tracks whether the line-online middle block (junction-freeze + P+D mix) should run
    // this tick. Set false by the overshoot-mode dead-reckon branch below so control falls
    // through to the distance-gate block at the bottom of update() without sampling the
    // (offline) sensor for junction state or weighted error.
    bool runOnlinePath = true;

    if (!_array->isOnline())
    {
        // Clear the D-term history so the first tick after reacquisition does not see a stale
        // derivative computed against an old (possibly several-second-stale) error sample.
        // Keeping the explicit zero also honors the `getRawError()` contract (a caller polling
        // immediately after the line drops sees 0.0f, not a stale sample). The `_seedLastErr`
        // flag then suppresses the derivative for exactly one tick on reacquisition: the P+D
        // block seeds `_lastErr = err` before computing `dErr`, so the first `dErr` is ~0
        // instead of the full `err` that differencing against 0 would produce. See finding #19.
        _lastErr     = 0.0f;
        _seedLastErr = true;

        // Set `_lostLine = true` BEFORE the overshoot-mode branch so EventTrace's
        // LINE LOST event still fires correctly via `isLineLost()`, even when
        // overshoot-mode suppresses LOST_LINE_ACTION dispatch. See T12 / ADR-023.
        _lostLine = true;
        // Clear the junction freeze flag — if a lost-line condition fires while frozen,
        // we treat lost-line as the authoritative path and want the next online tick to
        // start fresh (no stale freeze flag carrying over).
        _pidFrozen = false;

        if (_overshootMode)
        {
            // Overshoot-mode dead-reckon: ignore line-offline and command symmetric
            // cruise PWM (deadband-snapped) so the chassis advances through the
            // sensor-to-axle window even when the line physically ends at the node.
            // See ADR-023 "Pass-through collinear intersections" and ADR-018.
            //
            // The encoder distance gate evaluation MUST still run (the block below
            // this one), so the gate trip fires at LINE_SENSOR_AXLE_OFFSET_CM even
            // though the line is gone. Mirrors the junction-freeze pattern: do NOT
            // early-return — fall through to the distance-gate block at the bottom
            // of update(). The online-path middle block (junction-freeze + P+D) is
            // skipped via `runOnlinePath = false` because the sensor is offline and
            // sampling getJunctionType()/getError() against it would be invalid.
            const int16_t snapped = deadbandSnap(cruise, stopFloorMag);
            Drive.setSpeed(snapped, snapped, stopFloorMag);
            runOnlinePath = false;
            // Fall through to the distance-gate block below — do NOT early return.
            // The encoder gate must keep ticking even in dead-reckon mode.
        }
        else
        {
            // Existing LOST_LINE_ACTION dispatch (today's behavior).
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
                    // Use Config::CRAWL_PWM directly — a fixed constant that is guaranteed
                    // to be in [PWM_MIN_MAGNITUDE, PWM_MAX] regardless of the active cruise
                    // preset. The previous cruise/2 derivation was broken for SPEED_SLOW
                    // (15 < PWM_MIN_MAGNITUDE=20 → silently floored to 20 = 67% of SLOW
                    // cruise, not the intended ~50%). Sign-preserve cruise direction so
                    // reverse-mode CRAWL still creeps backward. Pass through deadbandSnap
                    // for the same consistency gate as every other PWM on this path.
                    const int16_t crawlSigned =
                        (cruise < 0) ? (int16_t) -Config::CRAWL_PWM : (int16_t) Config::CRAWL_PWM;
                    const int16_t crawl = deadbandSnap(crawlSigned);
                    Drive.setSpeed(crawl, crawl);
                    break;
                }
            }
            // Fall through to the distance-gate block below — do NOT early return. The
            // LOST_LINE_ACTION dispatch above already commanded the wheels this tick; the
            // encoder gate is purely encoder-based and stays valid while the line is lost,
            // so a real gap / faded patch / brief drift can dead-reckon the remaining
            // distance and complete the segment instead of hanging the full
            // LINE_FOLLOW_WAIT_TIMEOUT_MS. Mirrors the overshoot path's intent (it also
            // falls through here). The online-path middle block (junction-freeze + P+D) is
            // skipped via `runOnlinePath = false` because the sensor is offline and sampling
            // getJunctionType()/getError() against it would be invalid. See trace finding #17.
            runOnlinePath = false;
        }
    }

    // -------------------------------------------------------------------------------------
    // Line online: junction-freeze gate, then P+D controller. Skipped when
    // `runOnlinePath == false` (overshoot-mode dead-reckon fallthrough above) so the
    // distance-gate block at the bottom still runs but no junction/P+D math is done
    // against an offline sensor.
    // -------------------------------------------------------------------------------------
    if (runOnlinePath)
    {
        _lostLine = false;

        // Sample the junction classifier ONCE per tick. With
        // Config::LINE_ARRAY_TICK_CACHE_ENABLED == true this is a free cache read; with
        // caching off it is one ~9 ms I2C transaction. Intersection.update() runs after us
        // and will issue its own call against the same sensor — eliminating that double-read
        // is tracked separately (out of scope for issue #22).
        //
        // Freeze fires on any classifier value other than NONE / UNKNOWN — broadened from
        // the original issue #22 scope (T_OR_CROSS only) because the centroid-skew defect
        // affects LEFT and RIGHT junctions just as much (the issue body itself noted this).
        // See DECISIONS.md "LineFollower PID freeze during junction dwell".
        const uint8_t junctionRaw = _array->getJunctionType();
        const bool    atJunction  = (junctionRaw != (uint8_t) JunctionType::NONE
                                  && junctionRaw != (uint8_t) JunctionType::UNKNOWN);

        if (atJunction)
        {
            _pidFrozen = true;

            // Command symmetric cruise. `cruise` already reflects ramp-up / ramp-down
            // progress (modified by the branches above) and is sign-flipped for reverse.
            // Drive.setSpeed applies per-motor bias calibration automatically (DECISIONS.md
            // "Strategy B"), so no additional bias handling is needed here. deadbandSnap
            // mirrors the snap applied in the P+D path so a cruise value inside the deadband
            // range (rare but possible during ramp-up) is not silently dropped by
            // Drive.setSpeed's internal re-snap.
            const int16_t snapped = deadbandSnap(cruise, stopFloorMag);
            Drive.setSpeed(snapped, snapped, stopFloorMag);

            // Fall through to the distance-gate block below — do NOT early return. The
            // encoder distance counter must keep ticking during the freeze (acceptance
            // criterion).
        }
        else
        {
            if (_pidFrozen)
            {
                // First non-frozen tick after a freeze: clear the D-term history. Without
                // this the dErr computed against a pre-freeze `_lastErr` (potentially
                // several ticks stale) would produce a derivative kick on the first
                // post-junction PID tick. Mirrors the lost-line branch's existing
                // `_lastErr = 0.0f;` reset. The `_seedLastErr` flag below then suppresses
                // the derivative for exactly this one tick so even the reset-to-0 cannot
                // inject a `LINE_KD*err` kick on freeze exit. See trace finding #19.
                _lastErr     = 0.0f;
                _seedLastErr = true;
                _pidFrozen   = false;
            }

            const float err = _array->getError();
            // First online non-junction tick after a lost-line / freeze recovery: seed the
            // derivative baseline with the fresh error so this tick's `dErr` is ~0, rather
            // than differencing against the reset 0 and producing a one-tick derivative kick.
            // Steady-state behavior is unchanged — the flag is set only on recovery. See #19.
            if (_seedLastErr)
            {
                _lastErr     = err;
                _seedLastErr = false;
            }
            const float dErr = err - _lastErr;
            _lastErr         = err;

            // Select the gain set for the leading board (ADR-070): the rear board's shorter
            // sensor-to-axle lookahead needs its own Kp/Kd, so reverse follow uses the REAR
            // gains and forward follow the FRONT gains.
            const float kp = _reverse ? Config::LINE_KP_REAR : Config::LINE_KP;
            const float kd = _reverse ? Config::LINE_KD_REAR : Config::LINE_KD;

            // Bias the proportional term by the staged signed lateral offset (issue #156 /
            // ADR-069). ONLY the P setpoint shifts: `_lastErr` / `dErr` above stay the RAW
            // centroid, so `getRawError()` telemetry is intact and a mid-follow offset change
            // injects no derivative kick — just a bounded P-step of ≈ kp*_lineFollowOffset.
            // `_lineFollowOffset` is 0.0f on a centred follow, making this byte-identical to the
            // pre-#156 steering. The biased P term is reached ONLY on this online, non-junction
            // path, so a zero-weight offline read (runOnlinePath == false above) never sees it
            // and no extra dropout guard is needed. The board-selected gains (ADR-070) apply to
            // both the P and D terms.
            float corr = (kp * (err - _lineFollowOffset)) + (kd * dErr);

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

            // Snap each channel into {0} U [PWM_MIN_MAGNITUDE, PWM_MAX] (mirrored for
            // negatives). This compensates for both the saturation cap and the motor
            // deadband floor: a small P+D correction in the (0, 20) range would otherwise
            // be silently lost in the dead zone. Drive.setSpeed re-snaps internally — the
            // snap here ensures the value passed in already reflects the post-mix
            // correction so future readers see a consistent post-snap value.
            L = deadbandSnap(L, stopFloorMag);
            R = deadbandSnap(R, stopFloorMag);

            Drive.setSpeed(L, R, stopFloorMag);
        }
    }    // end if (runOnlinePath)

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
            if (_brakeOnGate)
            {
                // Today's behavior (default): self-disable with active brake. Delegating to
                // the builder's apply path keeps the disable semantics identical to a
                // caller-issued setEnabled(false) — including clearing _lastErr /
                // _lostLine / _useTShape.
                //
                // The latch is set AFTER the apply call because the disable path itself
                // clears `_distanceReached` (so a user-initiated disable cannot leak a stale
                // trip into a later re-enable). Setting it before would be immediately
                // clobbered, leaving `isDistanceReached()` permanently false and hanging
                // PathTracker's PROJECTION_FOLLOW state until its drain-timeout fault fires.
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
                                    /*distanceTargetCm=*/0.0f,
                                    /*hasOvershootModeOverride=*/false,
                                    /*overshootMode=*/false,
                                    /*hasBrakeOnGateOverride=*/false,
                                    /*brakeOnGate=*/true,
                                    /*hasLineFollowOffsetOverride=*/false,
                                    /*lineFollowOffset=*/0.0f,
                                    /*hasApproachTarget=*/false,
                                    /*approachPredictedCm=*/0.0f);
            }
            else
            {
                // Pass-through (ADR-023): gate trip latches `_distanceReached` and clears
                // the gate / ramp state, but the controller stays enabled and keeps issuing
                // Drive.setSpeed at cruise. PathTracker is the single consumer of
                // `_distanceReached`; it will re-arm the controller for the next edge via
                // `setEnabled(true)` on an already-enabled controller (no ramp-up reseed).
                //
                // Do NOT call `_applyEnableBuilder`; do NOT call `Drive.stop`. `_enabled`
                // stays true; cruise PWM continues on the next tick.
                _distanceTargetActive = false;
                _useTShape            = false;
                _useTShapeRampDown    = false;
            }
            // Latch the one-shot trip flag AFTER the brake-path apply call (which itself
            // clears `_distanceReached` via the disable branch). On the pass-through path
            // there is no apply call, so the order is harmless. PathTracker reads the latch
            // via `isDistanceReached()`.
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
                                            float               distanceTargetCm,
                                            bool                hasOvershootModeOverride,
                                            bool                overshootMode,
                                            bool                hasBrakeOnGateOverride,
                                            bool                brakeOnGate,
                                            bool                hasLineFollowOffsetOverride,
                                            float               lineFollowOffset,
                                            bool                hasApproachTarget,
                                            float               approachPredictedCm)
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

    // Apply overshoot-mode override if set. Default false on the owner preserves
    // today's LOST_LINE_ACTION dispatch when no caller has opted in. See ADR-023.
    if (hasOvershootModeOverride)
    {
        _overshootMode = overshootMode;
    }

    // Apply brake-on-gate override if set. Default true on the owner preserves
    // today's self-disable-with-brake behavior on gate trip. See ADR-023.
    if (hasBrakeOnGateOverride)
    {
        _brakeOnGate = brakeOnGate;
    }

    // Apply the lateral-offset override if set. Default 0.0f on the owner preserves centred
    // following when no caller has staged a bias; absent → the owner's current offset is
    // preserved so a re-arm on an already-enabled follower keeps its bias. Unlike the
    // preset / reverse overrides, the disable path below RESETS this to 0.0f. See issue #156.
    if (hasLineFollowOffsetOverride)
    {
        _lineFollowOffset            = lineFollowOffset;
        _hasLineFollowOffsetOverride = true;
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
            // Seed the ramp-up baseline from the both-wheel average so the ramp-up progress
            // measure in update() matches the ramp-down + distance-gate baselines. See
            // trace finding #20.
            _tShapeStartEnc  = (Drive.getEncL() + Drive.getEncR()) / 2;
            // Start the ramp from the motor deadband floor (Config::TSHAPE_START_SPEED
            // == PWM_MIN_MAGNITUDE), sign-corrected for reverse — NOT 0. Starting at 0
            // made the first tick command setSpeed(0, 0) on a centered line (cruise 0,
            // P+D corr ~0), so the chassis never broke static friction and the
            // distance-gated ramp could not advance; the sub-deadband head of the ramp
            // was also wasted as a snapped-flat plateau at PWM_MIN_MAGNITUDE. Mirrors
            // the ramp-DOWN floor (`targetFloor` in update()) and Drive's sibling
            // trapezoid (_startSpeed defaults to TSHAPE_START_SPEED). See the
            // Config::TSHAPE_START_SPEED doc and ADR-018.
            _tShapeStartPwm  = _reverse ? (int16_t) -Config::TSHAPE_START_SPEED
                                        : (int16_t) Config::TSHAPE_START_SPEED;
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
            // AC3 detect-handoff seed (F2): when the just-finished session ran an approach ramp
            // (`_approachActive` still true here — the disarm below happens AFTER this read), the
            // chassis is already crawling at the approach floor. Seed the ramp-down FROM the last
            // commanded approach magnitude, not the full preset, so commanded PWM is
            // NON-INCREASING across the detect tick (no jolt back to cruise on detection). When no
            // approach ramp ran, `_approachActive` is false and `seedMag` is `presetToPwm(_preset)`
            // — byte-identical to the pre-feature seed.
            const int16_t seedMag = _approachActive ? _approachLastMag : presetToPwm(_preset);
            _tShapeRampDownStart  = _reverse ? (int16_t) -seedMag : (int16_t) seedMag;
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

        // Anticipatory approach-ramp arm/disarm (T6). This runs LAST in the enable branch so the
        // ramp-down seed above still sees the PREVIOUS session's approach state (AC3). A
        // setEnabled(true) apply WITH setApproachTarget resolves and arms the schedule
        // (feature-gated); WITHOUT it, any prior schedule is disarmed — so the LINE_OVERSHOOT
        // re-arm (which does not chain setApproachTarget) clears it, and no stale FLOOR_HOLD /
        // OVERRUN phase leaks into the overshoot leg or the next hop.
        if (hasApproachTarget && Config::LINE_APPROACH_RAMP_ENABLED)
        {
            // cruiseMag = presetToPwm(_preset) — already a non-negative magnitude, so no abs()
            // (the Arduino abs() macro double-evaluates its argument). The schedule works in
            // the unsigned magnitude domain; the sign is applied per-tick in update() from
            // `_reverse`.
            const int16_t          cruiseMag = presetToPwm(_preset);
            const ApproachSchedule sched     = resolveApproachSchedule(
                approachPredictedCm,
                Config::LINE_APPROACH_MARGIN_CM,
                Config::LINE_APPROACH_MAX_MARGIN_CM,
                cruiseMag,
                Config::PWM_MOVING_MIN,
                Config::TSHAPE_ACCEL_PWM_PER_CM,
                Config::TSHAPE_RAMP_MIN_CM,
                Config::TSHAPE_RAMP_MAX_CM,
                Config::TSHAPE_NO_RAMP_EPSILON_PWM,
                Config::TSHAPE_MIN_MOVE_CM);

            if (sched.active)
            {
                // Arm: capture both wheels' encoder counters as the traveled-distance baseline
                // (same convention as the distance gate) and seed the phase/last-magnitude state.
                _approachActive    = true;
                _approachSchedule  = sched;
                _approachStartEncL = Drive.getEncL();
                _approachStartEncR = Drive.getEncR();
                _approachPhase     = ApproachPhase::CRUISE;
                _approachLastMag   = sched.cruiseMag;
            }
            else
            {
                // Schedule degraded to flat (short hop / degenerate) — leave the approach ramp
                // disarmed and run today's flat cruise-until-detected path (AC7).
                _approachActive = false;
                _approachPhase  = ApproachPhase::CRUISE;
            }
        }
        else
        {
            // No approach target on this enable apply (or the feature is globally disabled):
            // disarm any prior schedule and clear the phase so approachPhase() reads the CRUISE
            // sentinel, never a stale OVERRUN/FLOOR_HOLD.
            _approachActive = false;
            _approachPhase  = ApproachPhase::CRUISE;
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
        _seedLastErr          = false;    // Clear derivative-seed flag — no stale state on re-enable.
        _lostLine             = false;
        _pidFrozen            = false;    // Clear junction-freeze flag — no stale state on re-enable.
        _useTShape            = false;    // Disabling cancels any active cruise ramp-up.
        _useTShapeRampDown    = false;    // Disabling also cancels any active ramp-down.
        _distanceTargetActive = false;    // Disabling also disarms the distance gate.
        _distanceReached      = false;    // Clear the latch so a re-enable without a new
                                          // setDistanceTarget cannot report a stale trip.
        _overshootMode        = false;    // Restore owner default so a re-enable without an
                                          // explicit setOvershootMode(...) starts from the
                                          // LOST_LINE_ACTION dispatch path. See ADR-023.
        _brakeOnGate          = true;     // Restore owner default so a re-enable without an
                                          // explicit setBrakeOnGate(...) starts with the
                                          // self-disable-with-brake behavior. See ADR-023.
        _lineFollowOffset            = 0.0f;   // Restore centred following so a later
                                               // followDistance()/followUntilIntersection()
                                               // never inherits a stale lateral bias — mirrors
                                               // the overshoot/brake-on-gate reset-on-disable
                                               // (NOT the persistent preset/reverse). See #156.
        _hasLineFollowOffsetOverride = false; // Clear the owner override marker on disable.
        _approachActive       = false;    // Disabling cancels any armed approach ramp.
        _approachPhase        = ApproachPhase::CRUISE;  // Clear so approachPhase() reads the
                                                        // sentinel — no stale OVERRUN leaks.

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

ApproachPhase LineFollowerClass::approachPhase() const
{
    return _approachPhase;
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
            // Refresh the line array before update() so the P+D loop, lost-line detection,
            // and the distance gate read fresh channels. With
            // Config::LINE_ARRAY_TICK_CACHE_ENABLED == true (the shipped config) the cache
            // is only refreshed by LineArray.update(); without this call the fallback spins
            // on the sample frozen at entry. See trace finding #18.
            LineArray.update();
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
            // Refresh the line array first (the documented loop order: LineArray →
            // LineFollower → Intersection). With Config::LINE_ARRAY_TICK_CACHE_ENABLED ==
            // true (the shipped config) getJunctionType() is a pure cache reader, so without
            // this refresh the junction sample stays frozen on entry, no rising edge ever
            // fires, and the wait can only time out. See trace finding #18.
            LineArray.update();
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
