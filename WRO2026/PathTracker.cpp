/**
 * @file PathTracker.cpp
 * @brief Implementation of the PathTracker FSM (PR3 + pipeline-modes update).
 * @author WRO 2026
 */

#include "PathTracker.h"

#include <Arduino.h>
#include <math.h>

#include "DirectionPolicy.h"
#include "Drive.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Localization.h"
#include "PivotTrimPolicy.h"
#include "StopAtLinePolicy.h"

PathTrackerClass PathTracker;

// =========================================================================================
// Mode resolution helpers (file-local).
//
// Each `Routes::EdgeInfo` carries per-category override fields
// (`line_mode_override`, `blind_mode_override`, `projection_mode_override`,
// `direction_override`). The value `DEFAULT` means "inherit from
// `Routes::PIPELINE_CONFIG.<category>`"; anything else wins over the
// pipeline-wide default. These helpers resolve DEFAULT → concrete at
// edge-dispatch time so the FSM never has to chase the override semantics
// inline. The pre-edge turn wheel is resolved per turn from the
// `Routes::PivotTurn` table (ADR-036), not from an `EdgeInfo` field.
// =========================================================================================

namespace
{
    /// Resolve the LINE-edge cruise-mode for the given edge.
    Routes::LineMode _resolveLineMode(const Routes::EdgeInfo& edge)
    {
        return (edge.line_mode_override == Routes::LineMode::DEFAULT)
            ? Routes::PIPELINE_CONFIG.line
            : edge.line_mode_override;
    }

    /// Resolve the BLIND-edge motion-profile mode for the given edge.
    Routes::BlindMode _resolveBlindMode(const Routes::EdgeInfo& edge)
    {
        return (edge.blind_mode_override == Routes::BlindMode::DEFAULT)
            ? Routes::PIPELINE_CONFIG.blind
            : edge.blind_mode_override;
    }

    /// Resolve the PROJECTION-edge cruise-mode for the given edge.
    Routes::ProjectionMode _resolveProjectionMode(const Routes::EdgeInfo& edge)
    {
        return (edge.projection_mode_override == Routes::ProjectionMode::DEFAULT)
            ? Routes::PIPELINE_CONFIG.projection
            : edge.projection_mode_override;
    }

    // NOTE: the production direction-resolution helper has been promoted to
    // `DirectionPolicy.h::resolveDirection` (header-only inline) so both
    // production and the host-compile test target link the same definition.
    // See ADR-024 and the PR #34 review.

    /// Emit the standard REVERSE_UNAVAILABLE diagnostic. Shared by both guard sites
    /// (the load-bearing pre-turn guard in `_onPlanNextHop` and the defensive post-turn
    /// guard in `_dispatchEdgeAfterTurn`) so the two messages cannot drift; @p site names
    /// which guard fired. The caller still issues `_enterFault(...)`.
    void _printReverseUnavailable(uint8_t from, uint8_t to, Routes::EdgeType edgeType,
                                  const __FlashStringHelper* site)
    {
        Serial.print(F("[PathTracker] REVERSE_UNAVAILABLE: edge "));
        Serial.print(edgeType == Routes::EdgeType::LINE ? F("LINE") : F("PROJECTION"));
        Serial.print(F(" at u="));
        Serial.print(from);
        Serial.print(F(" v="));
        Serial.print(to);
        Serial.print(F(" requires rear board ("));
        Serial.print(site);
        Serial.println(F(")"));
    }

    /**
     * @brief Map a `Routes::LineMode` to a `Config::SpeedPreset` for
     *        `LineFollower.setPreset(...)`.
     *
     * @note `LineMode::DEFAULT` should not reach this helper (callers resolve it
     *       via `_resolveLineMode()` first). It is mapped to `NORMAL` defensively.
     */
    Config::SpeedPreset _lineModeToPreset(Routes::LineMode m)
    {
        switch (m)
        {
            case Routes::LineMode::SLOW:    return Config::SpeedPreset::SLOW;
            case Routes::LineMode::FAST:    return Config::SpeedPreset::FAST;
            case Routes::LineMode::NORMAL:  return Config::SpeedPreset::NORMAL;
            case Routes::LineMode::DEFAULT:
            default:                        return Config::SpeedPreset::NORMAL;
        }
    }

    /**
     * @brief Map a `Routes::ProjectionMode` to a `Config::SpeedPreset` for
     *        `LineFollower.setPreset(...)`. Mirrors `_lineModeToPreset`.
     *
     * @note `ProjectionMode::DEFAULT` should not reach this helper (callers resolve
     *       it via `_resolveProjectionMode()` first). Mapped to `NORMAL` defensively.
     */
    Config::SpeedPreset _projectionModeToPreset(Routes::ProjectionMode m)
    {
        switch (m)
        {
            case Routes::ProjectionMode::SLOW:    return Config::SpeedPreset::SLOW;
            case Routes::ProjectionMode::FAST:    return Config::SpeedPreset::FAST;
            case Routes::ProjectionMode::NORMAL:  return Config::SpeedPreset::NORMAL;
            case Routes::ProjectionMode::DEFAULT:
            default:                              return Config::SpeedPreset::NORMAL;
        }
    }

    /**
     * @brief Map a `Routes::TurnMode` to the wheel-pattern byte consumed by
     *        `TurnDegBuilder::setMode(uint8_t)` /
     *        `TurnToYawBuilder::setMode(uint8_t)`.
     *
     * Byte encoding matches the custom TurnGyro state machine introduced by the
     * 2026-05-17 PR4 plan (see `Drive.h::TurnDegBuilder::setMode` and
     * `Config::TURN_MODE` for the authoritative contract):
     *
     *   0 = COUNTER_ROT — both wheels spin opposite directions; tightest in-place rotation.
     *   1 = PIVOT_LEFT  — left wheel braked, right drives; chassis pivots around left wheel.
     *   2 = PIVOT_RIGHT — right wheel braked, left drives; chassis pivots around right wheel.
     *
     * @note Intentionally distinct from the legacy `MiniR4.DriveDC.TurnGyro` byte values —
     *       the lower-MCU TurnGyro is no longer called.
     * @note `TurnMode::DEFAULT` should not reach this helper (callers resolve the pre-turn
     *       wheel — pivot wheel or pipeline default — to a concrete value first). It is
     *       mapped to counter-rotation defensively.
     */
    uint8_t _turnModeToByte(Routes::TurnMode m)
    {
        switch (m)
        {
            case Routes::TurnMode::COUNTER_ROT: return 0;
            case Routes::TurnMode::PIVOT_LEFT:  return 1;
            case Routes::TurnMode::PIVOT_RIGHT: return 2;
            case Routes::TurnMode::DEFAULT:     return 0;    // defensive — see doc above
            default:                            return 0;
        }
    }

    /**
     * @brief Map a resolved `Routes::BlindMode` motion bundle to the `DriveMode`
     *        the `Drive.stopAtLine` seek runs under.
     *
     * @param m The `StopAtLineResolution::motionMode` from `resolveStopAtLine(...)`.
     *          Always one of the three concrete velocity bundles — `SYNC_TSHAPE`,
     *          `GYRO_TSHAPE`, or `RAW_FLAT` — so `STOP_AT_LINE` and `DEFAULT` never
     *          reach this helper.
     * @return The matching `DriveMode` (`SYNC`/`GYRO`/`RAW`).
     * @note The `default` returns `DriveMode::SYNC` purely defensively; the resolver
     *       guarantees `motionMode` is never `STOP_AT_LINE`/`DEFAULT`.
     */
    DriveMode blindModeToDriveMode(Routes::BlindMode m)
    {
        switch (m)
        {
            case Routes::BlindMode::SYNC_TSHAPE: return DriveMode::SYNC;
            case Routes::BlindMode::GYRO_TSHAPE: return DriveMode::GYRO;
            case Routes::BlindMode::RAW_FLAT:    return DriveMode::RAW;
            default:                             return DriveMode::SYNC;    // defensive — see doc above
        }
    }
}    // namespace

// =========================================================================================
// PathTrackerGoToBuilder
// =========================================================================================

PathTrackerGoToBuilder::PathTrackerGoToBuilder(PathTrackerClass& owner, uint8_t goalId)
    : _owner(owner)
    , _goalId(goalId)
{
}

PathTrackerGoToBuilder::~PathTrackerGoToBuilder()
{
    if (_executed)
    {
        return;
    }
    _owner._applyGoToBuilder(
        _goalId, _blocking, _routeDirection, _arrivalDirection, _stopAtLine);
    _executed = true;
}

PathTrackerGoToBuilder& PathTrackerGoToBuilder::async()
{
    _blocking = false;
    return *this;
}

PathTrackerGoToBuilder& PathTrackerGoToBuilder::setRouteDirection(Direction d)
{
    _routeDirection = d;
    return *this;
}

PathTrackerGoToBuilder& PathTrackerGoToBuilder::setArrivalDirection(Direction d)
{
    _arrivalDirection = d;
    return *this;
}

PathTrackerGoToBuilder& PathTrackerGoToBuilder::setStopAtLine(StopAtLine s)
{
    _stopAtLine = s;
    return *this;
}

// =========================================================================================
// PathTrackerClass — lifecycle
// =========================================================================================

bool PathTrackerClass::begin()
{
    _qHead  = 0;
    _qTail  = 0;
    _qCount = 0;
    _state  = State::IDLE;
    _destinationNode         = FieldMap::NODE_NONE;
    _nextHop                 = FieldMap::NODE_NONE;
    _activeEdgeType          = Routes::EdgeType::NONE;
    _passThrough             = false;
    _resolvedDirection       = Direction::AUTO;
    _resolvedBodyHeading     = 0.0f;
    // ADR-036: no pivot latched at boot.
    _resetPivotLatch();
    _activeRouteDirection    = Direction::AUTO;
    _activeArrivalDirection  = Direction::AUTO;
    _activeStopAtLine        = StopAtLine::AUTO;
    _fault = FaultInfo{};
    return true;
}

void PathTrackerClass::setTickCallback(void (*cb)())
{
    _tickCallback                = cb;
    _tickCallbackWarningEmitted  = false;
}

bool PathTrackerClass::isIdle() const
{
    return (_qCount == 0) && (_state == State::IDLE) && (_fault.kind == Fault::NONE);
}

bool PathTrackerClass::hasFault() const
{
    return _fault.kind != Fault::NONE;
}

PathTrackerClass::FaultInfo PathTrackerClass::getFault() const
{
    return _fault;
}

void PathTrackerClass::clearFault()
{
    _fault = FaultInfo{};
    if (_state == State::FAULT)
    {
        _state = State::IDLE;
    }
}

void PathTrackerClass::abort()
{
    // Stop the chassis first. Disable LineFollower BEFORE Drive.stop(true)
    // because the follower writes to Drive every tick — if it is still
    // enabled after Drive.stop, the next cooperativeTick() iteration's
    // LineFollower.update() re-issues Drive.setSpeed(...) and undoes the
    // halt. Mirror of Mission::abort()'s ordering (Mission.cpp:166).
    LineFollower.setEnabled(false);
    Drive.stop(true);

    _clearQueue();
    _state                  = State::IDLE;
    _destinationNode        = FieldMap::NODE_NONE;
    _nextHop                = FieldMap::NODE_NONE;
    _activeEdgeType         = Routes::EdgeType::NONE;
    _passThrough            = false;
    _resolvedDirection      = Direction::AUTO;
    _resolvedBodyHeading    = 0.0f;
    // ADR-036: drop any latched pivot so a mid-pivot abort cannot leak the
    // outgoing trim into the next goal's first dead-reckon drive.
    _resetPivotLatch();
    _activeRouteDirection   = Direction::AUTO;
    _activeArrivalDirection = Direction::AUTO;
    _activeStopAtLine       = StopAtLine::AUTO;
    _fault                  = FaultInfo{};
}

const char* PathTrackerClass::faultMnemonic(Fault kind)
{
    switch (kind)
    {
        case Fault::NONE:                return "OK";
        case Fault::NOT_ZEROED:          return "NZ";
        case Fault::UNREACHABLE:         return "UNR";
        case Fault::DRIVE_FAULT:         return "DRV";
        case Fault::BAD_NODE:            return "BAD";
        case Fault::DRAIN_TIMEOUT:       return "DRN";
        case Fault::REVERSE_UNAVAILABLE: return "REV";
    }
    return "??";
}

// =========================================================================================
// Queue
// =========================================================================================

bool PathTrackerClass::_enqueue(uint8_t goalId, Direction routeDir, Direction arrivalDir)
{
    if (_qCount >= Config::PATH_QUEUE_CAPACITY)
    {
        Serial.println(F("[PathTracker] queue overflow; goTo dropped"));
        return false;
    }
    _queue[_qTail].nodeId            = goalId;
    _queue[_qTail].route_direction   = routeDir;
    _queue[_qTail].arrival_direction = arrivalDir;
    // Default the call-site stop-at-line override to AUTO so a recycled FIFO
    // slot never carries a stale value. The real value (when the operator
    // chained `.setStopAtLine(...)`) is written into this same slot by
    // `_applyGoToBuilder` immediately after this returns — `_enqueue` keeps its
    // 3-arg signature, mirroring the queue plumbing for the Direction overrides.
    _queue[_qTail].stop_at_line      = StopAtLine::AUTO;
    _qTail  = (uint8_t) ((_qTail + 1) % Config::PATH_QUEUE_CAPACITY);
    _qCount++;
    return true;
}

bool PathTrackerClass::_pop(NavEntry& out)
{
    if (_qCount == 0)
    {
        return false;
    }
    out    = _queue[_qHead];
    _qHead = (uint8_t) ((_qHead + 1) % Config::PATH_QUEUE_CAPACITY);
    _qCount--;
    return true;
}

void PathTrackerClass::_clearQueue()
{
    _qHead  = 0;
    _qTail  = 0;
    _qCount = 0;
}

// =========================================================================================
// Builder dispatch
// =========================================================================================

void PathTrackerClass::_applyGoToBuilder(
    uint8_t    goalId,
    bool       blocking,
    Direction  routeDir,
    Direction  arrivalDir,
    StopAtLine stopAtLine)
{
    // No-op when a fault is already latched. This MUST run before the
    // goal-id and isZeroed() checks below: those branches call
    // `_enterFault(...)`, which would overwrite the existing
    // `_fault.kind`/`fromNode`/`destNode` and destroy the original
    // diagnostic. Guarding first preserves the latched fault so the
    // operator still sees what tripped it (clear with clearFault()).
    if (_fault.kind != Fault::NONE)
    {
        Serial.print(F("[PathTracker] goTo while faulted (kind="));
        Serial.print(faultMnemonic(_fault.kind));
        Serial.println(F("); clear with clearFault()"));
        return;
    }
    if (goalId == FieldMap::NODE_NONE || goalId >= Routes::N_NODES)
    {
        _enterFault(Fault::BAD_NODE, Localization.getCurrentNode(), goalId);
        return;
    }
    if (!Localization.isZeroed())
    {
        _enterFault(Fault::NOT_ZEROED, Localization.getCurrentNode(), goalId);
        return;
    }
    // Slot that `_enqueue` is about to fill (it advances `_qTail` past it on
    // success). Captured BEFORE the call so the call-site stop-at-line override
    // can be written into the same NavEntry without widening `_enqueue`.
    const uint8_t slot = _qTail;
    if (!_enqueue(goalId, routeDir, arrivalDir))
    {
        return;
    }
    _queue[slot].stop_at_line = stopAtLine;
    if (blocking)
    {
        _drainBlocking();
    }
}

// =========================================================================================
// Drain spin
// =========================================================================================

void PathTrackerClass::_drainBlocking()
{
    const uint32_t budget = (uint32_t) (_qCount + 1) * Config::PATH_DRAIN_TIMEOUT_MS_PER_GOAL;
    const uint32_t deadline = millis() + budget;

    if (_tickCallback == nullptr && !_tickCallbackWarningEmitted)
    {
        Serial.println(
            F("[PathTracker] no tick callback registered; other modules will freeze "
              "during drain."));
        _tickCallbackWarningEmitted = true;
    }

    while (true)
    {
        if (_fault.kind != Fault::NONE)
        {
            return;
        }
        if (_qCount == 0 && _state == State::IDLE)
        {
            return;
        }
        if ((int32_t) (millis() - deadline) > 0)
        {
            Drive.stop(true);
            _enterFault(Fault::DRAIN_TIMEOUT, Localization.getCurrentNode(), _destinationNode);
            return;
        }
        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            update();
        }
        delay(2);
    }
}

// =========================================================================================
// FSM tick
// =========================================================================================

void PathTrackerClass::update()
{
    // ADR-038: a missed stop-at-line seek latches `DriveClass::Fault::LINE_NOT_FOUND`
    // and `Drive.hasFault()` goes true; the shared Drive-fault guard below catches it
    // and hard-faults into `Fault::DRIVE_FAULT` (no soft fallback — a missed seek is
    // always a hard fault).
    if (_state != State::IDLE && _state != State::FAULT && Drive.hasFault())
    {
        Drive.stop(true);
        _enterFault(Fault::DRIVE_FAULT, Localization.getCurrentNode(), _destinationNode);
        return;
    }

    switch (_state)
    {
        case State::IDLE:
        {
            if (_qCount > 0)
            {
                _state = State::DEQUEUE;
            }
            return;
        }

        case State::DEQUEUE:
        {
            NavEntry e{};
            if (!_pop(e))
            {
                _state = State::IDLE;
                return;
            }
            _destinationNode        = e.nodeId;
            _activeRouteDirection   = e.route_direction;
            _activeArrivalDirection = e.arrival_direction;
            _activeStopAtLine       = e.stop_at_line;
            // ADR-036: each goal starts with a clean pivot latch. A pivot whose
            // FINAL hop was the previous goal's last hop left `_pendingPivot`
            // active with no following hop to consume it — clear all three here
            // so a pivot never fires across a `goTo(...)` boundary and a stop at
            // the pivot node degrades to a normal turn.
            _resetPivotLatch();
            _state                  = State::PLAN_NEXT_HOP;
            return;
        }

        case State::PLAN_NEXT_HOP:
        {
            _onPlanNextHop();
            return;
        }

        case State::TURNING:
        {
            if (Drive.isIdle())
            {
                _dispatchEdgeAfterTurn();
            }
            return;
        }

        case State::LINE_FOLLOW:
        {
            // On intersection-detect: do NOT advance Localization yet. The line
            // sensor sits `Config::LINE_SENSOR_AXLE_OFFSET_CM` ahead of the wheel
            // axle, so the chassis center is still that distance short of the
            // junction node. Re-arm the LineFollower with a distance gate equal
            // to that offset and wait for `isDistanceReached()` in LINE_OVERSHOOT
            // before calling `Localization.onIntersection(...)`.
            //
            // The controller is already enabled; re-issuing `setEnabled(true)`
            // is the cleanest way to arm the distance target via the builder
            // chain (it does NOT reset preset / direction / `_lastErr`).
            if (Intersection.isAtIntersection())
            {
                // -----------------------------------------------------------------
                // Pass-through decision: peek the next hop AFTER _nextHop. If the
                // peeked edge is collinear with the inbound edge (heading delta
                // wrap-normalized to within NAV_TURN_TOLERANCE_DEG) AND _nextHop is
                // not the final destination, take the pass-through branch — keep
                // LineFollower running across the overshoot without braking.
                // See ADR-023 and issue #19.
                //
                // Wrap-normalize MUST match the sim's `normalize_turn` so the
                // 359.5°/0.0° boundary classifies the same on both sides. Uses the
                // shared `NavTypes.h::wrapDeltaDeg` helper (was an inlined loop here,
                // in `_onPlanNextHop`, and in `DirectionPolicy.h`).
                // -----------------------------------------------------------------
                _passThrough = false;
                // `Config::PATH_PASS_THROUGH_ENABLED` leads so the peek's NEXT_HOP
                // lookup is skipped entirely when pass-through is compiled out. The
                // shared `_peekNextNextHop()` applies the not-final / NODE_NONE /
                // self-loop guards and returns `NODE_NONE` when there is no valid
                // next-next hop.
                const uint8_t nextNext =
                    Config::PATH_PASS_THROUGH_ENABLED ? _peekNextNextHop() : FieldMap::NODE_NONE;
                if (nextNext != FieldMap::NODE_NONE)
                {
                    const Routes::EdgeInfo& outbound = FieldMap::getEdge(_nextHop, nextNext);
                    if (outbound.type != Routes::EdgeType::NONE)
                    {
                        // Body-heading peek: the pass-through
                        // delta MUST compare BODY-headings, not raw forward edge
                        // headings. The chassis arrives at the intersection
                        // facing `_resolvedBodyHeading` (set by `_onPlanNextHop`
                        // for the inbound hop), which is either the inbound
                        // edge heading (FRONT) or `+180°` (REAR). The outbound
                        // body-heading is computed by re-resolving direction for
                        // the next-next hop with `inboundBody` as currentHeading.
                        // See TC9 (passthrough) + TC9c (mixed FRONT/REAR pass).
                        const float inboundBody = _resolvedBodyHeading;

                        // Read the builder overrides cached on DEQUEUE for the
                        // in-flight goal. The next-next hop is still part of the
                        // active goTo() call, so the same route_direction /
                        // arrival_direction values apply (arrival only takes effect
                        // when this is the final hop into the destination, which
                        // _resolveDirection handles via the isLastHop flag).
                        const Direction nextDir = resolveDirection(
                            outbound.direction_override,
                            Routes::PIPELINE_CONFIG.direction,
                            _activeArrivalDirection,
                            _activeRouteDirection,
                            inboundBody,
                            outbound.heading_deg,
                            (nextNext == _destinationNode),
                            Config::BIDIRECTIONAL_AUTO_ENABLED,
                            Config::LINE_BOARD_COUNT);

                        float outboundBody = outbound.heading_deg;
                        if (nextDir == Direction::REAR)
                        {
                            outboundBody = fmodf(outboundBody + 180.0f, 360.0f);
                            if (outboundBody < 0.0f) outboundBody += 360.0f;
                        }

                        const float delta = wrapDeltaDeg(outboundBody - inboundBody);

                        // Pass-through requires BOTH (1) collinear body
                        // headings AND (2) matching direction across the
                        // intersection. A REAR→FRONT or FRONT→REAR transition
                        // would force the dispatcher to flip
                        // LineFollower.setReverse(...) mid-flight — instantly
                        // swapping the active line board and inverting cruise
                        // sign in one tick, jolting the chassis without a
                        // controlled stop. Falling through to LINE_OVERSHOOT
                        // (with brake, no pass-through) gives the controller
                        // a clean re-init window before the new edge starts.
                        // PR #34 review.
                        if (fabsf(delta) <= Config::NAV_TURN_TOLERANCE_DEG &&
                            nextDir == _resolvedDirection)
                        {
                            _passThrough = true;
                        }
                    }
                }

                // Arm the overshoot. Always set overshootMode=true (project-wide rule
                // per ADR-023: the 5cm window dead-reckons through line-offline
                // conditions). brakeOnGate and tShapeRampDown depend on _passThrough.
                //
                // `.setReverse(...)` is chained explicitly (PR #34 review). The
                // previous code relied on LineFollower's sticky _reverse state —
                // `setEnabled(true)` without `setReverse` left whatever was last
                // configured. That works today, but a future change to
                // LineFollower::_applyEnableBuilder that reset _reverse on
                // re-enable would silently break the overshoot window on REAR
                // LINE hops (overshoot dead-reckons forward instead of reverse).
                // Making the direction explicit at every re-arm point removes
                // the implicit coupling.
                const bool overshootReverse = (_resolvedDirection == Direction::REAR);
                // Dead-reckon with the axle offset of the board that LED this hop. A REAR
                // hop tracked with the front board's offset lands off-node and feeds that
                // error into the next dead-reckoned hop, so a reverse overshoot uses the
                // rear board's own offset.
                const float overshootAxleOffsetCm =
                    overshootReverse ? Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM
                                     : Config::LINE_SENSOR_AXLE_OFFSET_CM;

                // ----------------------------------------------------------------
                // ADR-036 — LINE-incoming pivot trim (per-turn compensation).
                //
                // This LINE hop drives INTO `_nextHop`. If `_nextHop` is a pivot
                // node, the approaching pivot was resolved at THIS hop's plan-time
                // dispatch (`_onPlanNextHop`'s one-hop lookahead) into
                // `_approachingPivot`, which survives into this LINE_FOLLOW state.
                //
                // A LINE edge re-anchors on the junction line and cannot stop
                // short, so instead of an approach-distance trim we shorten the
                // overshoot advance by δ — the axle settles δ short of the node.
                // The OUTGOING hop's dead-reckon trim is NOT latched here: the
                // one-hop carry is already owned by `_pendingPivot` → next hop's
                // `_departingPivot`. This is the LINE row of the regime matrix.
                //
                // The validator guarantees δ ≤ the overshoot budget (the FRONT
                // sensor-to-axle offset; pivots are FRONT-only in v1), so the
                // reduced target stays positive; a non-positive result means a
                // config/validator mismatch, so clamp defensively to a tiny
                // positive advance rather than command a zero/negative distance
                // into the gate.
                const float inboundPivotTrim =
                    _approachingPivot.active ? _approachingPivot.trim_cm : 0.0f;
                float overshootTargetCm = overshootAxleOffsetCm;
                if (inboundPivotTrim > 0.0f)
                {
                    overshootTargetCm = overshootAxleOffsetCm - inboundPivotTrim;
                    if (overshootTargetCm < Config::PIVOT_TRIM_MIN_DRIVE_CM)
                    {
                        Serial.print(F("[PathTracker] ADR-036: LINE overshoot trim "
                                       "underflow (offset="));
                        Serial.print(overshootAxleOffsetCm);
                        Serial.print(F(" delta="));
                        Serial.print(inboundPivotTrim);
                        Serial.println(F("); clamping. Check validator/config."));
                        overshootTargetCm = Config::PIVOT_TRIM_MIN_DRIVE_CM;
                    }
                }

                LineFollower.setEnabled(true)
                            .setTShapeRampUp(false)
                            .setTShapeRampDown(!_passThrough)
                            .setOvershootMode(true)
                            .setReverse(overshootReverse)
                            .setBrakeOnGate(!_passThrough)
                            .setDistanceTarget(overshootTargetCm);
                _state = State::LINE_OVERSHOOT;
            }
            return;
        }

        case State::LINE_OVERSHOOT:
        {
            // Wait until the chassis has rolled forward by LINE_SENSOR_AXLE_OFFSET_CM
            // past the intersection-detect point. `isDistanceReached()` is read-and-clear;
            // the LineFollower's internal gate handling differs based on `_passThrough`:
            //
            //   - Stop-and-go (_passThrough == false): LineFollower's update() armed
            //     `_brakeOnGate = true` (default), so when the gate trips it self-disables
            //     via _applyEnableBuilder with active brake. By the time we observe
            //     isDistanceReached() here, the chassis is already braked and the
            //     controller is disabled. Same as pre-issue-19 behavior.
            //
            //   - Pass-through (_passThrough == true): LineFollower's update() armed
            //     `_brakeOnGate = false`, so the gate trip latches `_distanceReached`
            //     without disabling the controller. The chassis continues at cruise PWM
            //     under LineFollower's symmetric output. _dispatchEdgeAfterTurn (T6/T7)
            //     consumes _passThrough to decide whether to keep LineFollower live (LINE
            //     next), re-arm for a new distance target (PROJECTION next), or release
            //     with no brake before handing off to Drive.driveDist (BLIND next).
            //
            // Either way, advance Localization to the intersected node. _passThrough is
            // consumed by the upcoming PLAN_NEXT_HOP → _dispatchEdgeAfterTurn dispatch.
            //
            // See ADR-023 and issue #19.
            if (LineFollower.isDistanceReached())
            {
                Localization.onIntersection(_nextHop);
                _state = State::PLAN_NEXT_HOP;
            }
            return;
        }

        case State::PROJECTION_FOLLOW:
        {
            // PROJECTION edges terminate at a node with no physical intersection
            // marker; the runtime stops when the encoder distance gate
            // (armed with `edge.distance_cm` in `_dispatchEdgeAfterTurn`) trips.
            // No physical re-localisation event — use `onDeadReckonedAdvance`.
            if (LineFollower.isDistanceReached())
            {
                LineFollower.setEnabled(false);
                Localization.onDeadReckonedAdvance(_nextHop);
                _state = State::PLAN_NEXT_HOP;
            }
            return;
        }

        case State::BLIND_FORWARD:
        {
            if (Drive.isIdle())
            {
                Localization.onDeadReckonedAdvance(_nextHop);
                _state = State::PLAN_NEXT_HOP;
            }
            return;
        }

        case State::STOP_AT_LINE:
        {
            // Drive.stopAtLine(...) seek dispatched in `_dispatchEdgeAfterTurn`.
            //
            // No-line / max-distance fault: when the seek runs the full max
            // distance without a crossing, Drive latches `Fault::LINE_NOT_FOUND`
            // and `Drive.hasFault()` goes true. That is caught by the shared
            // Drive-fault guard at the TOP of update() (same as BLIND_FORWARD),
            // which stops the chassis and enters `Fault::DRIVE_FAULT` — so this
            // state body only has to handle the success path. No dead-reckon
            // fallback (ADR-027 / AC7).
            //
            // Success: the seek detected the crossing line, rolled the axle onto
            // it, and active-braked — a true physical re-localisation event. Unlike
            // BLIND_FORWARD (which dead-reckons via `onDeadReckonedAdvance`), advance
            // the node with `onIntersection(...)` so this hop RE-ANCHORS the discrete
            // position on the real line (the "better localization" of issue #27).
            if (Drive.isIdle())
            {
                Localization.onIntersection(_nextHop);
                _state = State::PLAN_NEXT_HOP;
            }
            return;
        }

        case State::FAULT:
        {
            return;
        }
    }
}

// =========================================================================================
// FSM helpers
// =========================================================================================

void PathTrackerClass::_enterFault(Fault kind, uint8_t from, uint8_t to)
{
    _fault.kind     = kind;
    _fault.fromNode = from;
    _fault.destNode = to;

    // Disable LineFollower BEFORE Drive.stop(true). If the follower is left
    // enabled (e.g., fault fired during LINE_FOLLOW or LINE_OVERSHOOT with
    // brakeOnGate=false), its next update() tick re-issues Drive.setSpeed()
    // and undoes the halt — making the fault non-fail-safe in exactly the
    // scenario it is meant to protect. Codex P1 (PR #34 review).
    LineFollower.setEnabled(false);
    Drive.stop(true);

    _clearQueue();
    _state                  = State::FAULT;
    _destinationNode        = FieldMap::NODE_NONE;
    _nextHop                = FieldMap::NODE_NONE;
    _activeEdgeType         = Routes::EdgeType::NONE;
    _activeRouteDirection   = Direction::AUTO;
    _activeArrivalDirection = Direction::AUTO;
    // ADR-036: a fault aborts the in-flight goal — drop any latched pivot so it
    // cannot bleed into a post-clearFault() goal.
    _resetPivotLatch();

    Serial.print(F("[PathTracker] FAULT "));
    Serial.print(faultMnemonic(kind));
    Serial.print(F(" from="));
    Serial.print(from);
    Serial.print(F(" to="));
    Serial.println(to);
}

void PathTrackerClass::_onPlanNextHop()
{
    const uint8_t cur = Localization.getCurrentNode();

    // Already at the destination — done.
    if (cur == _destinationNode)
    {
        _destinationNode = FieldMap::NODE_NONE;
        // ADR-036: the goal is complete. If its final hop departed a pivot node
        // `_pendingPivot` is still active with no following hop to consume it —
        // drop all three so a later `goTo(...)` (or the next queued goal, on the
        // DEQUEUE path) does not inherit a stale latch.
        _resetPivotLatch();
        _state           = (_qCount > 0) ? State::DEQUEUE : State::IDLE;
        return;
    }

    const uint8_t nxt = FieldMap::getNextHop(cur, _destinationNode);
    if (nxt == FieldMap::NODE_NONE)
    {
        _enterFault(Fault::UNREACHABLE, cur, _destinationNode);
        return;
    }

    const Routes::EdgeInfo& edge = FieldMap::getEdge(cur, nxt);
    if (edge.type == Routes::EdgeType::NONE)
    {
        _enterFault(Fault::UNREACHABLE, cur, nxt);
        return;
    }

    _nextHop        = nxt;
    _activeEdgeType = edge.type;

    // -----------------------------------------------------------------------
    // Per-turn pivot latch contract (ADR-036, supersedes ADR-035).
    //
    // A pivot is a property of a 3-node turn `(u, v, w)`, not of an edge. Snapshot
    // the carry into `_departingPivot` BEFORE the one-hop lookahead overwrites
    // `_pendingPivot` — the single carry member is sufficient ONLY because the
    // departing pivot is read out first. Both records survive the TURNING state
    // into the drive dispatch (and `_approachingPivot` into the LINE_FOLLOW
    // overshoot-arm, a different FSM state).
    //
    // Last hop: no pivot can latch onto the FINAL node mid-turn, so skip the
    // forward lookahead. This never skips CONSUMING an already-pending latch — a
    // pivot `(u,v,w)` whose goal is `w` still consumes its wheel + trim on the
    // final `v→w` hop via `_departingPivot`.
    // -----------------------------------------------------------------------
    const bool isLastHop = (nxt == _destinationNode);

    _departingPivot = _pendingPivot;    // snapshot the carry FIRST
    if (isLastHop)
    {
        _approachingPivot = PivotRef{};    // no pivot can latch onto the final node
    }
    else
    {
        const uint8_t nextNext = FieldMap::getNextHop(nxt, _destinationNode);
        const Routes::PivotTurn* p = FieldMap::findPivotTurn(cur, nxt, nextNext);
        _approachingPivot =
            p ? PivotRef{true, p->wheel, p->trim_cm} : PivotRef{};
    }
    _pendingPivot = _approachingPivot;    // carry for the NEXT hop

    const float currentWorld = Localization.getHeadingDeg();

    // -----------------------------------------------------------------------
    // Bidirectional direction resolution.
    //
    // Resolve per-hop precedence: per-edge override → pipeline default →
    // builder arrival override (last-hop only) → builder route override →
    // auto (turn-saving comparison against BIDIRECTIONAL_TURN_THRESHOLD_DEG,
    // gated by 2-board chassis + kill-switch). Then compute the body-target
    // heading the chassis must face: equal to edge.heading_deg for FRONT,
    // or (edge.heading_deg + 180°) mod 360 for REAR (so the rear board ends
    // up pointing along the edge direction).
    //
    // The skip-turn check below MUST compare against bodyTarget — not the raw
    // edge.heading_deg — otherwise a chassis already aligned with the raw
    // forward heading would short-circuit a REAR hop into a forward-going
    // LineFollower dispatch.
    //
    // `isLastHop` was hoisted above for the pivot latch contract.
    // -----------------------------------------------------------------------

    // Builder overrides cached on DEQUEUE for the in-flight goal (T8).
    // Both default to Direction::AUTO for bare `goTo(...)` calls — the
    // `_resolveDirection` precedence layers below per-edge / pipeline
    // overrides only kick in when the operator chained
    // `.setRouteDirection(...)` / `.setArrivalDirection(...)`.
    const Direction builderArrival = _activeArrivalDirection;
    const Direction builderRoute   = _activeRouteDirection;

    // `edge.direction_override` is a per-DIRECTED-edge value: this hop reads
    // `EDGES[cur][nxt]`. A field_data `direction:` override is baked onto the
    // forward (u->v) edge only, so a hop the planner routes the other way
    // (v->u) reads DEFAULT here and falls through to the AUTO heuristic below.
    // That is intentional — front/rear depends on the entry direction, so an
    // override authored for u->v must NOT bind v->u (mirroring it would reverse
    // the chassis when forward is correct). Author a swapped EDGES entry to
    // force the reverse sense. See graph_builder `_emit_directed_pair` warning.
    Direction resolved = resolveDirection(
        edge.direction_override,
        Routes::PIPELINE_CONFIG.direction,
        builderArrival,
        builderRoute,
        currentWorld,
        edge.heading_deg,
        isLastHop,
        Config::BIDIRECTIONAL_AUTO_ENABLED,
        Config::LINE_BOARD_COUNT);

    // ADR-036: pivots are FRONT-only in v1 (#54). The wheel + δ are derived for
    // forward traversal, so force FRONT on any pivot-adjacent hop — the approach
    // `u→v` (`_approachingPivot`) or the departure `v→w` (`_departingPivot`) —
    // and never silently downgrade to a counter-rotation: the author chose a
    // single-wheel pivot for a clearance a center turn may not have. Because
    // FRONT is always available, the `reverseUnavailable(...)` guard below is
    // inert for pivot hops (that is the intended "FRONT genuinely unavailable
    // faults via the existing guard" backstop — no separate pivot-skip path).
    if (_departingPivot.active || _approachingPivot.active)
    {
        resolved = Direction::FRONT;
    }

    // ------------------------------------------------------------------------
    // Early reverse-availability guard.
    //
    // Reject REAR LINE/PROJECTION on a 1-board chassis BEFORE the pre-edge turn
    // is dispatched, otherwise the chassis physically rotates 180° to face the
    // (unservable) REAR body-heading and then halts in fault — leaving the
    // robot pointing backward with REVERSE_UNAVAILABLE latched and the operator
    // forced to manually re-orient.
    //
    // The auto branch of `_resolveDirection` short-circuits to FRONT when
    // `LINE_BOARD_COUNT < 2`, so this guard only fires for an operator-forced
    // REAR (per-edge override, pipeline default, or builder route/arrival).
    // BLIND REAR is permitted on 1-board chassis (no line sensor required),
    // so the check is scoped to LINE / PROJECTION.
    //
    // The same guard is left in `_dispatchEdgeAfterTurn` as defensive
    // redundancy — if a future FSM change introduces a path into the
    // dispatcher that bypasses this early site, the late guard still fires.
    // ------------------------------------------------------------------------
    if (reverseUnavailable(resolved, edge.type, Config::LINE_BOARD_COUNT))
    {
        _printReverseUnavailable(cur, nxt, edge.type, F("pre-turn guard"));
        _enterFault(Fault::REVERSE_UNAVAILABLE, cur, nxt);
        return;
    }

    float bodyTarget = edge.heading_deg;
    if (resolved == Direction::REAR)
    {
        bodyTarget = fmodf(bodyTarget + 180.0f, 360.0f);
        if (bodyTarget < 0.0f) bodyTarget += 360.0f;
    }

    _resolvedDirection   = resolved;
    _resolvedBodyHeading = bodyTarget;

    const float delta = wrapDeltaDeg(bodyTarget - currentWorld);

    if (fabsf(delta) > Config::NAV_TURN_TOLERANCE_DEG)
    {
        // Yaw-vs-peek divergence guard (ADR-023 follow-up): the LINE_OVERSHOOT
        // pass-through decision (T4) uses EDGE headings from FieldMap, while
        // this turn dispatch decision uses live IMU heading (Localization).
        // If the IMU has drifted slightly, the peek may have classified the
        // hop as pass-through (controller left enabled) while this branch now
        // decides a turn is required. Without a guard, LineFollower.update()
        // would keep issuing Drive.setSpeed each tick while Drive.turnToYaw
        // owns the wheels — two writers fighting for motors. Disable the
        // follower cleanly (no brake) so Drive's turn command runs alone.
        //
        // The same guard fires for REAR hops where the peek (now also
        // body-heading aware) and this branch agree on FRONT semantics
        // but disagree on whether a turn is needed: the bodyTarget vs.
        // currentWorld delta is the authoritative trigger.
        if (LineFollower.isEnabled())
        {
            LineFollower.setEnabled(false).setBrake(false);
        }

        // Clear any pass-through eligibility the LINE_FOLLOW peek latched. That
        // peek used FieldMap edge headings; this branch fired because the live
        // IMU heading disagrees, so a real pre-edge turn IS required. Leaving
        // `_passThrough` set would make the post-turn `_dispatchEdgeAfterTurn`
        // run its blind pass-through branch (cruise start-speed) from a dead
        // stop with no soft-start ramp, jolting the chassis. The chassis is
        // stopped after the turn, so the blind move must soft-start from
        // TSHAPE_START_SPEED. (PR#35 findings #32 / #90.)
        _passThrough = false;

        const int16_t imuTarget = Localization.toImuYaw(bodyTarget);

        // ADR-036: the pre-turn maneuver wheel comes from the DEPARTING pivot
        // (the turn we are pivoting through at `cur` this hop), else the pipeline
        // default. A valid pivot's turn (validator floor >= Config::PIVOT_MIN_TURN_DEG)
        // always exceeds NAV_TURN_TOLERANCE_DEG (0.5° by default), so this branch fires and
        // the pivot wheel applies — no firmware min-angle change is needed. The wheel and the δ
        // come from the SAME resolved record, so they cannot disagree.
        const Routes::TurnMode resolvedTurnMode =
            _departingPivot.active ? _departingPivot.wheel : Routes::PIPELINE_CONFIG.turn;
        const uint8_t turnModeByte = _turnModeToByte(resolvedTurnMode);
        if (_departingPivot.active)
        {
            Serial.print(F("[PathTracker] ADR-036: pivot turn at v="));
            Serial.print(cur);
            Serial.print(F(" wheel="));
            Serial.print(turnModeByte);
            Serial.print(F(" trim="));
            Serial.println(_departingPivot.trim_cm);
        }
        Drive.turnToYaw(imuTarget).setMode(turnModeByte).async();
        _state = State::TURNING;
    }
    else
    {
        _dispatchEdgeAfterTurn();
    }
}

void PathTrackerClass::_resetPivotLatch()
{
    // ADR-036: all three latch members move together. Clearing only some would
    // leak a pivot across a goal boundary — the bleed these resets prevent.
    _pendingPivot     = PivotRef{};
    _departingPivot   = PivotRef{};
    _approachingPivot = PivotRef{};
}

float PathTrackerClass::_pivotTrimDistance(const Routes::EdgeInfo& edge)
{
    // ADR-036 (issue #44) — per-turn pivot compensation, distance-terminated side.
    // LOCAL effective distance only; EDGES[*][*].distance_cm is never mutated. The
    // one-hop carry / latch is owned by the `_pendingPivot` contract resolved in
    // `_onPlanNextHop()`; this wrapper only gathers the two already-resolved trims
    // and delegates the subtract-and-clamp to the shared `resolvePivotTrim` policy
    // (PivotTrimPolicy.h) so the host test links the SAME symbol.
    //
    // DEPARTING trim: this hop LEAVES a pivot node (`cur`) — its outgoing
    // over-travel δ, so the axle lands exactly on the node. APPROACHING trim: this
    // dead-reckon edge drives INTO a pivot node (`nxt`) — its δ, so the chassis
    // stops δ short and the pivot starts at the geometric pivot point. On a chain
    // u→v→w→x the hop v→w nets `length − δ_v(departing) − δ_w(approaching)`.
    const float departingTrim   = _departingPivot.active ? _departingPivot.trim_cm : 0.0f;
    const float approachingTrim =
        _approachingPivot.active ? _approachingPivot.trim_cm : 0.0f;

    const PivotTrimResolution trim = resolvePivotTrim(
        edge.distance_cm, departingTrim, approachingTrim, Config::PIVOT_TRIM_MIN_DRIVE_CM);

    // Defensive clamp trace. The validator guarantees δ leaves at least the floor
    // on every dead-reckon side (and the chained sum-check on a shared edge), so a
    // correctly-built table never clamps here. A clamp means a future bad table /
    // validator mismatch — trace it (the policy already degraded to the floor so
    // Drive.driveDist never sees a non-positive cm, which it rejects and no-ops).
    if (trim.clamped)
    {
        Serial.print(F("[PathTracker] ADR-036: dead-reckon trim underflow (raw="));
        Serial.print(edge.distance_cm);
        Serial.print(F(" eff="));
        Serial.print(trim.effectiveCm);
        Serial.println(F("); clamping. Check validator/config."));
    }

    return trim.driveCm;
}

uint8_t PathTrackerClass::_peekNextNextHop() const
{
    // Shared one-hop lookahead guard. Two FSM sites peek the hop AFTER `_nextHop`
    // toward `_destinationNode`: the LINE_FOLLOW pass-through decision and the
    // BLIND auto-rule projection classification (`_nextHopIsProjection`). Both need
    // the SAME three guards, kept in one place here so they cannot drift:
    //   1. not-final  — `_nextHop` is the destination, so there is no next-next hop
    //                   (the final/goal hop must never crash or trigger either rule).
    //   2. NODE_NONE  — `getNextHop` returns no route from `_nextHop` to the goal.
    //   3. self-loop  — the lookahead returns `_nextHop` itself.
    // Any failed guard yields `FieldMap::NODE_NONE`; otherwise the valid next-next
    // node id is returned for the caller to classify.
    if (_nextHop == _destinationNode)
    {
        return FieldMap::NODE_NONE;
    }
    const uint8_t nextNext = FieldMap::getNextHop(_nextHop, _destinationNode);
    if (nextNext == FieldMap::NODE_NONE || nextNext == _nextHop)
    {
        return FieldMap::NODE_NONE;
    }
    return nextNext;
}

bool PathTrackerClass::_nextHopIsProjection() const
{
    // ADR-038 auto-rule predicate: is the hop AFTER `_nextHop` a PROJECTION edge?
    //
    // Short-circuit to `false` (no NEXT_HOP work) when the feature is compiled out
    // via `Routes::BLIND_PROJECTION_AUTO_SEEK == false`, so the auto-seek peek costs
    // nothing on a build that never auto-seeks. When enabled, reuse the shared
    // `_peekNextNextHop()` guard and classify the resulting edge — a `NODE_NONE`
    // peek (final/goal hop, unreachable, or self-loop) classifies as not-projection.
    if (!Routes::BLIND_PROJECTION_AUTO_SEEK)
    {
        return false;
    }
    const uint8_t nextNext = _peekNextNextHop();
    if (nextNext == FieldMap::NODE_NONE)
    {
        return false;
    }
    return FieldMap::getEdge(_nextHop, nextNext).type == Routes::EdgeType::PROJECTION;
}

void PathTrackerClass::_dispatchEdgeAfterTurn()
{
    const uint8_t cur = Localization.getCurrentNode();
    const Routes::EdgeInfo& edge = FieldMap::getEdge(cur, _nextHop);

    // -----------------------------------------------------------------------
    // Reverse-availability guard (T7 / issue #19) — defensive redundancy.
    //
    // Same condition as the early guard in `_onPlanNextHop` (see the block
    // beginning "Early reverse-availability guard"). The early guard is the
    // load-bearing site — it fires before the 180° pre-edge turn dispatches —
    // and this dispatcher-side check exists only to defend against future
    // FSM changes that might route into `_dispatchEdgeAfterTurn` without
    // re-running `_onPlanNextHop`.
    //
    // BLIND REAR is permitted on a 1-board chassis: dead-reckoned reverse
    // does not require a rear line sensor — only negative PWM.
    // -----------------------------------------------------------------------
    if (reverseUnavailable(_resolvedDirection, edge.type, Config::LINE_BOARD_COUNT))
    {
        _printReverseUnavailable(cur, _nextHop, edge.type, F("post-turn guard"));
        _enterFault(Fault::REVERSE_UNAVAILABLE, cur, _nextHop);
        return;
    }

    const bool reverse = (_resolvedDirection == Direction::REAR);

    switch (edge.type)
    {
        case Routes::EdgeType::LINE:
        {
            // ADR-036: a LINE outgoing edge applies no dead-reckon trim — it is
            // sensor-terminated and re-anchors on the junction line. The departing
            // trim is irrelevant on a LINE hop (the line corrects it); the approach
            // trim for a LINE-incoming pivot is the overshoot-advance reduction in
            // the LINE_FOLLOW state; the outgoing carry for the next hop is already
            // owned by the `_pendingPivot` latch. Nothing to do here.

            // Drain any stale Intersection edge so the first LINE_FOLLOW tick
            // does not skip the segment.
            (void) Intersection.isAtIntersection();

            // Resolve the per-edge cruise-mode override (DEFAULT → pipeline
            // default) and chain it onto the enable transition.
            //
            // Chain `.setOvershootMode(false)` to flip the controller back to
            // normal P+D after the overshoot window. T4 always sets
            // overshootMode=true during the LINE_OVERSHOOT arming; here we
            // clear it so the new LINE segment runs the full PID controller.
            //
            // If _passThrough is true (pass-through into a collinear LINE next
            // edge), the controller is already enabled — `setEnabled(true)`
            // hits the already-enabled fast-path in `_applyEnableBuilder`, so
            // no ramp-up reseed fires. Cruise PWM is preserved across the
            // intersection (preset may snap if the new edge has a different
            // LineMode override; see ADR-023 trade-off).
            //
            // `.setReverse(reverse)` selects forward-vs-rear board + error sign
            // (T7 / issue #19). REAR is only reachable here on a 2-board chassis
            // — the guard above rejects REAR LINE on a 1-board chassis.
            const Routes::LineMode    resolvedLineMode = _resolveLineMode(edge);
            const Config::SpeedPreset preset           = _lineModeToPreset(resolvedLineMode);
            LineFollower.setEnabled(true)
                        .setPreset(preset)
                        .setReverse(reverse)
                        .setOvershootMode(false);
            _state = State::LINE_FOLLOW;
            _passThrough = false;    // consumed
            break;
        }

        case Routes::EdgeType::PROJECTION:
        {
            // PROJECTION edges run the line follower like LINE edges, but the
            // terminating node has no physical junction marker — so we arm the
            // LineFollower's distance gate to `edge.distance_cm` and advance the
            // graph node when `isDistanceReached()` fires (see PROJECTION_FOLLOW
            // state in update()).
            (void) Intersection.isAtIntersection();    // defensive — drain any latched edge

            // For pass-through LINE→PROJECTION, the controller is already enabled
            // from the LINE_OVERSHOOT pass-through (T5). `setEnabled(true)` hits
            // the already-enabled fast-path → no ramp-up reseed; the new
            // distance-target gate baseline is captured at this re-arm tick.
            // `setOvershootMode(false)` flips the controller back to normal P+D.
            //
            // For non-pass-through, the controller is already disabled (T5 path);
            // `setEnabled(true)` performs a fresh enable with ramp-up.
            //
            // `.setReverse(reverse)` selects forward-vs-rear board + error sign
            // (T7 / issue #19). REAR is only reachable here on a 2-board chassis
            // — the guard above rejects REAR PROJECTION on a 1-board chassis.
            // ADR-036: PROJECTION is distance-terminated (the gate uses the
            // edge length, no physical marker), so it takes both the departing
            // trim (`_departingPivot`, the pivot we just left) and the approach
            // trim (`_approachingPivot`, the pivot at `_nextHop`) — both folded
            // into a LOCAL effective distance by `_pivotTrimDistance`.
            // `distance_cm` itself is untouched.
            const float effectiveDistCm = _pivotTrimDistance(edge);

            const Routes::ProjectionMode resolvedProjMode = _resolveProjectionMode(edge);
            const Config::SpeedPreset    preset           = _projectionModeToPreset(resolvedProjMode);
            LineFollower.setEnabled(true)
                        .setPreset(preset)
                        .setReverse(reverse)
                        .setOvershootMode(false)
                        .setDistanceTarget(effectiveDistCm);
            _state = State::PROJECTION_FOLLOW;
            _passThrough = false;    // consumed
            break;
        }

        case Routes::EdgeType::BLIND:
        {
            // -----------------------------------------------------------------
            // Effective stop-at-line resolution (ADR-027 / issue #27).
            //
            // One canonical decision — `resolveStopAtLine(...)` in
            // `StopAtLinePolicy.h` (mirrored by `sim/runtime.py`) — folds the
            // call-site `_activeStopAtLine` flag, the per-edge BlindMode, the
            // pipeline blind default, the resolved travel direction, and the
            // board count into `{seek, motionMode}`:
            //   - seek == true  → run a `Drive.stopAtLine` seek in
            //                      `motionMode`'s DriveMode (FRONT always; REAR
            //                      only on a 2-board chassis).
            //   - seek == false → dead-reckon the hop in `motionMode` (OFF, or a
            //                      forced-REAR degrade on a 1-board chassis).
            //
            // `_resolveBlindMode(edge)` already maps DEFAULT → pipeline blind, so
            // the resolver's `edgeMode` input is never DEFAULT (no double-resolve).
            // `motionMode` is guaranteed concrete — never STOP_AT_LINE/DEFAULT.
            //
            // ADR-038 auto-rule: a blind hop that feeds a PROJECTION edge seeks a
            // line first to re-anchor before the marker-less projection.
            // `_nextHopIsProjection()` peeks the hop AFTER `_nextHop` toward the goal
            // and classifies its edge type (folding in the `BLIND_PROJECTION_AUTO_SEEK`
            // compile-time toggle so it short-circuits to `false` — no NEXT_HOP work —
            // when the feature is built out). It shares the `_peekNextNextHop()` guard
            // with the LINE_FOLLOW pass-through peek (same NODE_NONE / self-loop guard)
            // so the final/goal blind hop (no next-next) never crashes and never
            // auto-seeks. The result feeds the resolver's `autoRuleActive` input.
            // -----------------------------------------------------------------
            const StopAtLineQuery q{
                _activeStopAtLine,
                edge.stop_at_line_override,
                _resolveBlindMode(edge),
                Routes::PIPELINE_CONFIG.blind,
                _resolvedDirection,
                Config::LINE_BOARD_COUNT,
                _nextHopIsProjection()};
            const StopAtLineResolution sal = resolveStopAtLine(q);

            if (sal.seek)
            {
                // ADR-036: a STOP_AT_LINE seek is sensor-terminated — it self-
                // corrects on the physical line, so it takes NO incoming/outgoing
                // distance trim. The seek drives the FULL `edge.distance_cm` (max
                // seek distance — the crossing, not the distance, terminates it).
                // The outgoing carry for the next hop is already owned by the
                // `_pendingPivot` latch, so there is nothing to do here — a pivot
                // reached via a seek still over-travels on its OUTGOING side, and
                // the next hop's `_departingPivot` (snapshot of `_pendingPivot`)
                // applies that trim.

                // A pass-through LINE→BLIND handoff leaves the LineFollower
                // enabled (T5's brake-on-gate=false path). Release it WITHOUT
                // braking before handing the wheels to the Drive seek so the
                // two writers never fight over the motors — same ordering the
                // dead-reckoned blind branch below uses.
                if (_passThrough)
                {
                    LineFollower.setEnabled(false).setBrake(false);
                }

                // Drain any stale Intersection edge so the seek's own physical
                // crossing cannot leak a phantom nav edge into the next LINE hop
                // (AC7 — the forward seek drives over its target T_OR_CROSS while
                // Intersection.update() independently counts it).
                (void) Intersection.isAtIntersection();

                // Honor the edge's resolved motion bundle. SYNC by default; GYRO
                // additionally pins the held heading (body heading already accounts
                // for REAR via `_resolvedBodyHeading`). Reverse is selected per the
                // resolved hop direction. `.async()` keeps the FSM non-blocking —
                // the STOP_AT_LINE state below polls `Drive.isIdle()` /
                // `Drive.hasFault()` for the outcome.
                const DriveMode mode = blindModeToDriveMode(sal.motionMode);

                // ADR-038: a missed seek is always a hard fault — the seek uses the
                // Drive default max search distance (`expected +
                // Config::STOP_AT_LINE_MAX_MARGIN_CM`); no `.setMaxDistance(...)`
                // zero-margin override and no soft-fallback recovery. The Drive
                // builder is move-only (deleted copy ctor + early enqueue), so the
                // whole chain MUST be one discarded-temporary RAII statement.
                Drive.stopAtLine(edge.distance_cm)
                    .setReverse(reverse)
                    .setMode(mode)
                    .setHeading(Localization.toImuYaw(_resolvedBodyHeading))
                    .async();
                _state       = State::STOP_AT_LINE;
                _passThrough = false;    // consumed
                break;
            }

            // Pass-through LINE→BLIND handoff: when _passThrough is set, the
            // LineFollower is still enabled (T5's brake-on-gate=false path), so
            // we need to:
            //   1. Capture the current cruise PWM BEFORE disabling LineFollower
            //      (so we know what speed to hand off to Drive.driveDist).
            //   2. Disable LineFollower WITHOUT braking — it releases the motors
            //      cleanly; whatever speeds Drive last latched stay until the
            //      next move's setStartSpeed kicks in.
            //   3. Pass the captured cruise as .setStartSpeed(...) to Drive
            //      builders that otherwise default to Config::TSHAPE_START_SPEED
            //      (PWM_MIN_MAGNITUDE), which would dip the chassis to the
            //      deadband floor first.
            //
            // For non-pass-through, LineFollower was already disabled by T5's
            // path and startSpeed defaults to the existing TSHAPE_START_SPEED.
            //
            // See ADR-023.
            int16_t startSpeed = Config::TSHAPE_START_SPEED;
            if (_passThrough)
            {
                startSpeed = presetToPwm(LineFollower.getPreset());
                LineFollower.setEnabled(false).setBrake(false);
            }

            // BLIND REAR (T7 / issue #19): sign-by-power is the convention —
            // `driveDist` rejects negative cm at Drive.h:1254 (positive distance,
            // signed PWM). Negate the cruise + start/end speeds when REAR so the
            // trapezoid ramps from a negative-direction PWM straight away
            // (Drive.h:1288-1296 documents the matching sign-by-cruise rule).
            const int16_t cruisePwm = reverse ? (int16_t) -Config::SPEED_NORMAL
                                              : (int16_t)  Config::SPEED_NORMAL;
            const int16_t signedStartSpeed = reverse ? (int16_t) -startSpeed : startSpeed;

            // ADR-036: a dead-reckon BLIND hop (plain `driveDist`, sal.seek ==
            // false) is distance-terminated, so it takes both the departing trim
            // (`_departingPivot`) and the approach trim (`_approachingPivot`) —
            // both folded into a LOCAL effective distance. The seek path above
            // returned early and never reaches this; `edge.distance_cm` stays
            // untouched in the table.
            const float effectiveDistCm = _pivotTrimDistance(edge);

            // Per-edge motion-profile override (DEFAULT → pipeline default).
            // Three concrete BlindMode values map onto the Drive builder chain:
            //   - SYNC_TSHAPE : Drive.driveDist(cm).setMode(SYNC).tShape().async()
            //                   (today's behaviour — encoder-rate sync PID + trapezoidal
            //                   velocity profile)
            //   - GYRO_TSHAPE : Drive.driveDist(cm).setMode(GYRO).tShape().async()
            //                   (straight-line yaw lock + trapezoidal profile; heading
            //                   sourced from edge.heading_deg, converted into the IMU
            //                   frame via Localization.toImuYaw — body heading already
            //                   accounts for REAR via `_resolvedBodyHeading`, so passing
            //                   `edge.heading_deg` keeps the yaw target correct only when
            //                   FRONT; REAR uses `_resolvedBodyHeading`).
            //   - RAW_FLAT    : Drive.driveDist(cm).setMode(RAW)
            //                       .setStartSpeed(cruise).setEndSpeed(cruise).async()
            //                   (open-loop PWM, no ramp — start/end speeds pinned to the
            //                   cruise PWM short-circuits the trapezoidal profile)
            //
            // DEFAULT and STOP_AT_LINE cannot reach this switch: `sal.motionMode`
            // from `resolveStopAtLine(...)` is guaranteed to be one of the three
            // concrete velocity bundles (SYNC_TSHAPE / GYRO_TSHAPE / RAW_FLAT). The
            // resolver already folded the OFF-on-a-STOP_AT_LINE-edge case into the
            // pipeline blind default, and the forced-REAR 1-board degrade keeps the
            // edge's own mode — so the substitution that used to live here is gone.
            // The defensive SYNC_TSHAPE default below remains as belt-and-braces.
            switch (sal.motionMode)
            {
                case Routes::BlindMode::GYRO_TSHAPE:
                {
                    // GYRO holds the BODY heading — use `_resolvedBodyHeading` (already
                    // rotated 180° for REAR by `_onPlanNextHop`), not raw edge.heading_deg.
                    const int16_t imuTarget = Localization.toImuYaw(_resolvedBodyHeading);
                    Drive.driveDist(effectiveDistCm)    // ADR-036 pivot-trimmed (LOCAL); table distance untouched
                         .setMode(DriveMode::GYRO)
                         .setHeading(imuTarget)
                         .setPower(cruisePwm)
                         .setStartSpeed(signedStartSpeed)    // pass-through: cruise; otherwise: ±PWM_MIN_MAGNITUDE
                         .tShape()
                         .async();
                    break;
                }

                case Routes::BlindMode::RAW_FLAT:
                {
                    // RAW_FLAT pins start=end=cruise for a flat open-loop profile (the "FLAT"
                    // in the name). Pass-through is the one case where the inbound
                    // LineFollower cruise can legitimately differ from cruisePwm (non-NORMAL
                    // preset on the inbound LINE edge); use the captured startSpeed there.
                    // Non-pass-through MUST keep .setStartSpeed(cruisePwm) — anything else
                    // introduces a ramp and breaks the FLAT contract for callers that picked
                    // RAW_FLAT precisely to avoid a ramp. See ADR-023 and issue #19.
                    const int16_t rawStart = _passThrough ? signedStartSpeed : cruisePwm;
                    Drive.driveDist(effectiveDistCm)    // ADR-036 pivot-trimmed (LOCAL); table distance untouched
                         .setMode(DriveMode::RAW)
                         .setPower(cruisePwm)
                         .setStartSpeed(rawStart)
                         .setEndSpeed(cruisePwm)
                         .async();
                    break;
                }

                case Routes::BlindMode::SYNC_TSHAPE:
                case Routes::BlindMode::DEFAULT:    // defensive — see comment above
                default:
                {
                    Drive.driveDist(effectiveDistCm)    // ADR-036 pivot-trimmed (LOCAL); table distance untouched
                         .setMode(DriveMode::SYNC)
                         .setPower(cruisePwm)
                         .setStartSpeed(signedStartSpeed)
                         .tShape()
                         .async();
                    break;
                }
            }
            _state = State::BLIND_FORWARD;
            _passThrough = false;    // consumed
            break;
        }

        case Routes::EdgeType::NONE:
        default:
        {
            _enterFault(Fault::UNREACHABLE, cur, _nextHop);
            break;
        }
    }
}

// =========================================================================================
// goTo() public surface
// =========================================================================================

PathTrackerGoToBuilder PathTrackerClass::goTo(uint8_t goalId)
{
    return PathTrackerGoToBuilder(*this, goalId);
}

PathTrackerGoToBuilder PathTrackerClass::goTo(const char* goalName)
{
    const uint8_t id = FieldMap::findNodeByName(goalName);
    return PathTrackerGoToBuilder(*this, id);
}
