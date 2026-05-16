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
     * @brief Map a `Routes::TurnMode` to the lower-MCU turn motor-mode byte
     *        consumed by `TurnDegBuilder::setMode(uint8_t)` /
     *        `TurnToYawBuilder::setMode(uint8_t)`.
     *
     * Byte encoding (matches the lower-MCU's `TurnGyro` mode argument):
     *   0 = counter-rotation (both wheels driven, opposite signs)
     *   1 = pivot left  (left wheel braked, right drives)
     *   2 = pivot right (right wheel braked, left drives)
     *
     * `TurnMode::DEFAULT` should not reach this helper (callers resolve it via
     * `_resolveTurnMode()` first). It is mapped to counter-rotation defensively.
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
                // Already-enabled re-arm: skip ramp-up reseed; ramp-down to floor handles the smooth stop.
                LineFollower.setEnabled(true)
                            .setTShapeRampUp(false)
                            .setDistanceTarget(Config::LINE_SENSOR_AXLE_OFFSET_CM);
                _state = State::LINE_OVERSHOOT;
            }
            return;
        }

        case State::LINE_OVERSHOOT:
        {
            // Wait until the chassis has rolled forward by LINE_SENSOR_AXLE_OFFSET_CM
            // past the intersection-detect point. `isDistanceReached()` is
            // read-and-clear; the LineFollower self-disables (active brake) when
            // the gate fires, so we just need to drop the enable flag explicitly
            // and advance the graph node.
            if (LineFollower.isDistanceReached())
            {
                LineFollower.setEnabled(false);
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
            const Routes::LineMode    resolvedLineMode = _resolveLineMode(edge);
            const Config::SpeedPreset preset           = _lineModeToPreset(resolvedLineMode);
            LineFollower.setEnabled(true).setPreset(preset);
            _state = State::LINE_FOLLOW;
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

            const Routes::ProjectionMode resolvedProjMode = _resolveProjectionMode(edge);
            const Config::SpeedPreset    preset           = _projectionModeToPreset(resolvedProjMode);
            LineFollower.setEnabled(true)
                        .setPreset(preset)
                        .setDistanceTarget(edge.distance_cm);
            _state = State::PROJECTION_FOLLOW;
            break;
        }

        case Routes::EdgeType::BLIND:
        {
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
                         .tShape()
                         .async();
                    break;
                }

                case Routes::BlindMode::RAW_FLAT:
                {
                    Drive.driveDist(edge.distance_cm)
                         .setMode(DriveMode::RAW)
                         .setStartSpeed(cruisePwm)
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
                         .tShape()
                         .async();
                    break;
                }
            }
            _state = State::BLIND_FORWARD;
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
