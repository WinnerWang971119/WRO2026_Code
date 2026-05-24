/**
 * @file PathTracker.cpp
 * @brief Implementation of the PathTracker FSM (PR3 + pipeline-modes update).
 * @author WRO 2026
 */

#include "PathTracker.h"

#include <Arduino.h>
#include <math.h>

#include "Drive.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Localization.h"

PathTrackerClass PathTracker;

// =========================================================================================
// Mode resolution helpers (file-local).
//
// Each `Routes::EdgeInfo` carries per-category override fields
// (`line_mode_override`, `blind_mode_override`, `projection_mode_override`,
// `pre_turn_mode_override`). The value `DEFAULT` means "inherit from
// `Routes::PIPELINE_CONFIG.<category>`"; anything else wins over the
// pipeline-wide default. These helpers resolve DEFAULT → concrete at
// edge-dispatch time so the FSM never has to chase the override semantics
// inline.
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

    /// Resolve the pre-edge turn motor-mode for the given edge.
    Routes::TurnMode _resolveTurnMode(const Routes::EdgeInfo& edge)
    {
        return (edge.pre_turn_mode_override == Routes::TurnMode::DEFAULT)
            ? Routes::PIPELINE_CONFIG.turn
            : edge.pre_turn_mode_override;
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
     * @note `TurnMode::DEFAULT` should not reach this helper (callers resolve it via
     *       `_resolveTurnMode()` first). It is mapped to counter-rotation defensively.
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
    _owner._applyGoToBuilder(_goalId, _blocking);
    _executed = true;
}

PathTrackerGoToBuilder& PathTrackerGoToBuilder::async()
{
    _blocking = false;
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
    _destinationNode = FieldMap::NODE_NONE;
    _nextHop         = FieldMap::NODE_NONE;
    _activeEdgeType  = Routes::EdgeType::NONE;
    _passThrough     = false;
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
    // Stop the chassis first; Drive.stop(true) also clears Drive's queue,
    // so a half-issued turn or driveDist will not resume.
    Drive.stop(true);

    _clearQueue();
    _state           = State::IDLE;
    _destinationNode = FieldMap::NODE_NONE;
    _nextHop         = FieldMap::NODE_NONE;
    _activeEdgeType  = Routes::EdgeType::NONE;
    _passThrough     = false;
    _fault           = FaultInfo{};
}

const char* PathTrackerClass::faultMnemonic(Fault kind)
{
    switch (kind)
    {
        case Fault::NONE:          return "OK";
        case Fault::NOT_ZEROED:    return "NZ";
        case Fault::UNREACHABLE:   return "UNR";
        case Fault::DRIVE_FAULT:   return "DRV";
        case Fault::BAD_NODE:      return "BAD";
        case Fault::DRAIN_TIMEOUT: return "DRN";
    }
    return "??";
}

// =========================================================================================
// Queue
// =========================================================================================

bool PathTrackerClass::_enqueue(uint8_t goalId)
{
    if (_qCount >= Config::PATH_QUEUE_CAPACITY)
    {
        Serial.println(F("[PathTracker] queue overflow; goTo dropped"));
        return false;
    }
    _queue[_qTail].nodeId = goalId;
    _qTail = (uint8_t) ((_qTail + 1) % Config::PATH_QUEUE_CAPACITY);
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

void PathTrackerClass::_applyGoToBuilder(uint8_t goalId, bool blocking)
{
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
    if (_fault.kind != Fault::NONE)
    {
        Serial.print(F("[PathTracker] goTo while faulted (kind="));
        Serial.print(faultMnemonic(_fault.kind));
        Serial.println(F("); clear with clearFault()"));
        return;
    }
    if (!_enqueue(goalId))
    {
        return;
    }
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
            _destinationNode = e.nodeId;
            _state           = State::PLAN_NEXT_HOP;
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
                // 359.5°/0.0° boundary classifies the same on both sides.
                // TODO: extract the wrap-normalize loop (duplicated in
                //       `_onPlanNextHop`) into a `WROMath` free helper.
                // -----------------------------------------------------------------
                _passThrough = false;
                if (Config::PATH_PASS_THROUGH_ENABLED && _nextHop != _destinationNode)
                {
                    const uint8_t nextNext = FieldMap::getNextHop(_nextHop, _destinationNode);
                    if (nextNext != FieldMap::NODE_NONE && nextNext != _nextHop)
                    {
                        const Routes::EdgeInfo& nextEdge = FieldMap::getEdge(_nextHop, nextNext);
                        if (nextEdge.type != Routes::EdgeType::NONE)
                        {
                            const uint8_t cur = Localization.getCurrentNode();
                            const Routes::EdgeInfo& inboundEdge = FieldMap::getEdge(cur, _nextHop);
                            float delta = nextEdge.heading_deg - inboundEdge.heading_deg;
                            while (delta >   180.0f) { delta -= 360.0f; }
                            while (delta <= -180.0f) { delta += 360.0f; }
                            if (fabsf(delta) <= Config::NAV_TURN_TOLERANCE_DEG)
                            {
                                _passThrough = true;
                            }
                        }
                    }
                }

                // Arm the overshoot. Always set overshootMode=true (project-wide rule
                // per ADR-023: the 5cm window dead-reckons through line-offline
                // conditions). brakeOnGate and tShapeRampDown depend on _passThrough.
                LineFollower.setEnabled(true)
                            .setTShapeRampUp(false)
                            .setTShapeRampDown(!_passThrough)
                            .setOvershootMode(true)
                            .setBrakeOnGate(!_passThrough)
                            .setDistanceTarget(Config::LINE_SENSOR_AXLE_OFFSET_CM);
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

    Drive.stop(true);

    _clearQueue();
    _state           = State::FAULT;
    _destinationNode = FieldMap::NODE_NONE;
    _nextHop         = FieldMap::NODE_NONE;
    _activeEdgeType  = Routes::EdgeType::NONE;

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

    const float currentWorld = Localization.getHeadingDeg();
    const float targetWorld  = edge.heading_deg;
    float delta = targetWorld - currentWorld;
    while (delta > 180.0f)   { delta -= 360.0f; }
    while (delta <= -180.0f) { delta += 360.0f; }

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
        if (LineFollower.isEnabled())
        {
            LineFollower.setEnabled(false).setBrake(false);
        }

        const int16_t imuTarget = Localization.toImuYaw(targetWorld);

        // Resolve the per-edge turn motor-mode override (DEFAULT → pipeline
        // default) and map to the lower-MCU mode byte the Turn builders accept.
        const Routes::TurnMode resolvedTurnMode = _resolveTurnMode(edge);
        const uint8_t          turnModeByte     = _turnModeToByte(resolvedTurnMode);
        Drive.turnToYaw(imuTarget).setMode(turnModeByte).async();
        _state = State::TURNING;
    }
    else
    {
        _dispatchEdgeAfterTurn();
    }
}

void PathTrackerClass::_dispatchEdgeAfterTurn()
{
    const uint8_t cur = Localization.getCurrentNode();
    const Routes::EdgeInfo& edge = FieldMap::getEdge(cur, _nextHop);

    switch (edge.type)
    {
        case Routes::EdgeType::LINE:
        {
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
            const Routes::LineMode    resolvedLineMode = _resolveLineMode(edge);
            const Config::SpeedPreset preset           = _lineModeToPreset(resolvedLineMode);
            LineFollower.setEnabled(true)
                        .setPreset(preset)
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
            const Routes::ProjectionMode resolvedProjMode = _resolveProjectionMode(edge);
            const Config::SpeedPreset    preset           = _projectionModeToPreset(resolvedProjMode);
            LineFollower.setEnabled(true)
                        .setPreset(preset)
                        .setOvershootMode(false)
                        .setDistanceTarget(edge.distance_cm);
            _state = State::PROJECTION_FOLLOW;
            _passThrough = false;    // consumed
            break;
        }

        case Routes::EdgeType::BLIND:
        {
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

            // Per-edge motion-profile override (DEFAULT → pipeline default).
            // Three concrete BlindMode values map onto the Drive builder chain:
            //   - SYNC_TSHAPE : Drive.driveDist(cm).setMode(SYNC).tShape().async()
            //                   (today's behaviour — encoder-rate sync PID + trapezoidal
            //                   velocity profile)
            //   - GYRO_TSHAPE : Drive.driveDist(cm).setMode(GYRO).tShape().async()
            //                   (straight-line yaw lock + trapezoidal profile; heading
            //                   sourced from edge.heading_deg, converted into the IMU
            //                   frame via Localization.toImuYaw)
            //   - RAW_FLAT    : Drive.driveDist(cm).setMode(RAW)
            //                       .setStartSpeed(cruise).setEndSpeed(cruise).async()
            //                   (open-loop PWM, no ramp — start/end speeds pinned to the
            //                   cruise PWM short-circuits the trapezoidal profile)
            //
            // DEFAULT cannot reach here (resolved above), but we route it through
            // the SYNC_TSHAPE branch defensively to match the historical behaviour.
            const Routes::BlindMode resolvedBlindMode = _resolveBlindMode(edge);
            const int16_t           cruisePwm         = Config::SPEED_NORMAL;

            switch (resolvedBlindMode)
            {
                case Routes::BlindMode::GYRO_TSHAPE:
                {
                    const int16_t imuTarget = Localization.toImuYaw(edge.heading_deg);
                    Drive.driveDist(edge.distance_cm)
                         .setMode(DriveMode::GYRO)
                         .setHeading(imuTarget)
                         .setStartSpeed(startSpeed)    // pass-through: cruise; otherwise: PWM_MIN_MAGNITUDE
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
                    const int16_t rawStart = _passThrough ? startSpeed : cruisePwm;
                    Drive.driveDist(edge.distance_cm)
                         .setMode(DriveMode::RAW)
                         .setStartSpeed(rawStart)
                         .setEndSpeed(cruisePwm)
                         .async();
                    break;
                }

                case Routes::BlindMode::SYNC_TSHAPE:
                case Routes::BlindMode::DEFAULT:    // defensive — see comment above
                default:
                {
                    Drive.driveDist(edge.distance_cm)
                         .setMode(DriveMode::SYNC)
                         .setStartSpeed(startSpeed)
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
