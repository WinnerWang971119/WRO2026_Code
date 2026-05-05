/**
 * @file PathTracker.cpp
 * @brief Implementation of the PathTracker FSM, queue, and fluent goTo() builder.
 * @author WRO 2026
 */

#include "PathTracker.h"

#include <Arduino.h>
#include <math.h>

#include "Drive.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Localization.h"

PathTrackerClass PathTracker;    // single global instance

// =========================================================================================
// PathTrackerGoToBuilder — RAII auto-execute fluent builder.
// =========================================================================================

PathTrackerGoToBuilder::PathTrackerGoToBuilder(PathTrackerClass& owner, uint8_t goalId)
    : _owner(owner)
    , _goalId(goalId)
{
    // Stage only — destructor applies.
}

PathTrackerGoToBuilder::~PathTrackerGoToBuilder()
{
    if (_executed)
    {
        // Defensive guard against double-execute. Copy/move are deleted, so reaching here
        // would imply manual destructor invocation — still a no-op for safety.
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
// PathTrackerClass — lifecycle.
// =========================================================================================

bool PathTrackerClass::begin()
{
    _qHead  = 0;
    _qTail  = 0;
    _qCount = 0;
    _state  = State::IDLE;
    _currentGoal       = FieldMap::NODE_NONE;
    _nextHop           = FieldMap::NODE_NONE;
    _activeEdgeType    = Routes::EdgeType::NONE;
    _activeGoalApproach = nullptr;
    _blindIndex        = 0;
    _fault = FaultInfo{};
    return true;
}

void PathTrackerClass::setTickCallback(void (*cb)())
{
    _tickCallback                  = cb;
    _tickCallbackWarningEmitted    = false;
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

// =========================================================================================
// Queue plumbing.
// =========================================================================================

bool PathTrackerClass::_enqueue(uint8_t goalId)
{
    if (_qCount >= Config::PATH_QUEUE_CAPACITY)
    {
        Serial.println(F("[PathTracker] queue overflow; goTo dropped"));
        return false;
    }
    _queue[_qTail].goalId = goalId;
    _qTail = (uint8_t) ((_qTail + 1) % Config::PATH_QUEUE_CAPACITY);
    _qCount++;
    return true;
}

bool PathTrackerClass::_pop(GoalEntry& out)
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
// Builder dispatch — invoked by PathTrackerGoToBuilder's destructor.
// =========================================================================================

void PathTrackerClass::_applyGoToBuilder(uint8_t goalId, bool blocking)
{
    // Pre-flight validation. We surface bad calls at the call site rather than at FSM
    // dispatch time so the operator sees the fault on the line of code that caused it.

    // 1. Bad node id (from a name-lookup miss or out-of-range integer overload).
    if (goalId == FieldMap::NODE_NONE || goalId >= Routes::N_NODES)
    {
        _enterFault(Fault::BAD_NODE, Localization.getCurrentNode(), goalId);
        return;
    }

    // 2. goTo before zero — yawOffset is uninitialised, so any turn would point at a
    // garbage IMU target. Surface loud per Diego's start-button-press convention.
    if (!Localization.isZeroed())
    {
        _enterFault(Fault::NOT_ZEROED, Localization.getCurrentNode(), goalId);
        return;
    }

    // 3. Already faulted — refuse new entries until the caller clears.
    if (_fault.kind != Fault::NONE)
    {
        Serial.print(F("[PathTracker] goTo while faulted (kind="));
        Serial.print((uint8_t) _fault.kind);
        Serial.println(F("); cleared after clearFault()"));
        return;
    }

    if (!_enqueue(goalId))
    {
        // _enqueue printed the warning. Drop silently — overflow is a caller bug.
        return;
    }

    if (blocking)
    {
        _drainBlocking();
    }
}

// =========================================================================================
// Drain spin (sync-blocking goTo() destructor path).
// =========================================================================================

void PathTrackerClass::_drainBlocking()
{
    // Bound the spin: per-goal worst-case x (queued + in-flight). On expiry we treat the
    // hang as a DRAIN_TIMEOUT fault rather than spinning forever, mirroring Drive's F3 fix.
    const uint32_t budget = (uint32_t) (_qCount + 1) * Config::PATH_DRAIN_TIMEOUT_MS_PER_GOAL;
    const uint32_t deadline = millis() + budget;

    if (_tickCallback == nullptr && !_tickCallbackWarningEmitted)
    {
        Serial.println(
            F("[PathTracker] no tick callback registered; only PathTracker.update() will "
              "tick during goTo() drain (other modules will freeze)."));
        _tickCallbackWarningEmitted = true;
    }

    // Spin until queue empty, FSM idle, or fault. Each iteration ticks every cooperative
    // module via the registered callback (which itself calls PathTracker.update()), then
    // delays 2 ms — same shape as Drive's _drainBlocking.
    while (true)
    {
        if (_fault.kind != Fault::NONE)
        {
            return;    // _enterFault already emptied the queue and stopped the chassis.
        }
        if (_qCount == 0 && _state == State::IDLE)
        {
            return;
        }
        if ((int32_t) (millis() - deadline) > 0)
        {
            // Bounded budget exceeded. Stop the chassis, clear the queue, latch the fault.
            Drive.stop(true);
            _enterFault(Fault::DRAIN_TIMEOUT, Localization.getCurrentNode(), _currentGoal);
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
// FSM tick.
// =========================================================================================

void PathTrackerClass::update()
{
    // Drive-fault propagation: if the lower-level chassis layer faulted, surface it here
    // so the goTo() drain can return and the operator sees a single coherent error path.
    if (_state != State::IDLE && _state != State::FAULT && Drive.hasFault())
    {
        Drive.stop(true);
        _enterFault(Fault::DRIVE_FAULT, Localization.getCurrentNode(), _currentGoal);
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
            GoalEntry e{};
            if (!_pop(e))
            {
                _state = State::IDLE;
                return;
            }
            _currentGoal       = e.goalId;
            _activeGoalApproach = FieldMap::getGoalApproach(_currentGoal);
            _blindIndex        = 0;
            _state             = State::PLAN_NEXT_HOP;
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
            // Read-and-clear consumer of the intersection edge per Intersection.h F6.
            if (Intersection.isAtIntersection())
            {
                LineFollower.setEnabled(false);
                Localization.onIntersection(_nextHop);
                _state = State::PLAN_NEXT_HOP;
            }
            return;
        }

        case State::PROJECTION_FOLLOW:
        {
            // LineFollower auto-disabled itself with active brake on gate trip.
            if (LineFollower.isDistanceReached())
            {
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

        case State::GOAL_BLIND_DISPATCH:
        {
            _onGoalBlindDispatch();
            return;
        }

        case State::GOAL_BLIND_AWAIT:
        {
            if (Drive.isIdle())
            {
                _state = State::GOAL_BLIND_DISPATCH;
            }
            return;
        }

        case State::FAULT:
        {
            // Terminal sink — caller must clearFault() to re-arm. Updates here are no-ops.
            return;
        }
    }
}

// =========================================================================================
// FSM helpers.
// =========================================================================================

void PathTrackerClass::_enterFault(Fault kind, uint8_t from, uint8_t to)
{
    _fault.kind     = kind;
    _fault.fromNode = from;
    _fault.goalNode = to;

    // Defensive: ensure the chassis is stopped on any fault path so a partially-issued
    // motion does not continue. Drive.stop also clears Drive's queue.
    Drive.stop(true);

    // Empty our own queue and reset the FSM so subsequent ticks are no-ops until cleared.
    _clearQueue();
    _state             = State::FAULT;
    _currentGoal       = FieldMap::NODE_NONE;
    _nextHop           = FieldMap::NODE_NONE;
    _activeEdgeType    = Routes::EdgeType::NONE;
    _activeGoalApproach = nullptr;
    _blindIndex        = 0;

    Serial.print(F("[PathTracker] FAULT kind="));
    Serial.print((uint8_t) kind);
    Serial.print(F(" from="));
    Serial.print(from);
    Serial.print(F(" to="));
    Serial.println(to);
}

void PathTrackerClass::_onPlanNextHop()
{
    const uint8_t cur = Localization.getCurrentNode();

    // Already at the goal node? Skip directly to the goal-blind sequence (if it's a goal)
    // or back to IDLE / DEQUEUE. No spurious turn dispatch.
    if (cur == _currentGoal)
    {
        if (_activeGoalApproach != nullptr)
        {
            _state       = State::GOAL_BLIND_DISPATCH;
            _blindIndex  = 0;
        }
        else
        {
            // Goal is not a GOAL-type node (e.g. line node used as a waypoint). Just done.
            _currentGoal = FieldMap::NODE_NONE;
            _state       = State::IDLE;
        }
        return;
    }

    const uint8_t nxt = FieldMap::getNextHop(cur, _currentGoal);
    if (nxt == FieldMap::NODE_NONE)
    {
        _enterFault(Fault::UNREACHABLE, cur, _currentGoal);
        return;
    }

    const Routes::EdgeInfo& edge = FieldMap::getEdge(cur, nxt);
    if (edge.type == Routes::EdgeType::NONE)
    {
        // NEXT_HOP claimed a hop, but the EDGES table has no direct edge for it. Either
        // the planner is buggy or the schema is corrupted — surface as UNREACHABLE so the
        // caller sees a recovery hook.
        _enterFault(Fault::UNREACHABLE, cur, nxt);
        return;
    }

    _nextHop        = nxt;
    _activeEdgeType = edge.type;

    // Decide whether a turn is needed. Compare edge heading (world frame) to the chassis's
    // current world heading; only dispatch turnToYaw if the absolute delta exceeds the
    // configured tolerance. Avoids a sub-degree twitch at colinear edge joins.
    const float currentWorld = Localization.getHeadingDeg();
    const float targetWorld  = edge.heading_deg;
    float delta = targetWorld - currentWorld;
    // Normalise to (-180, +180].
    while (delta > 180.0f)  { delta -= 360.0f; }
    while (delta <= -180.0f) { delta += 360.0f; }

    if (fabsf(delta) > Config::NAV_TURN_TOLERANCE_DEG)
    {
        const int16_t imuTarget = Localization.toImuYaw(targetWorld);
        Drive.turnToYaw(imuTarget).async();
        _state = State::TURNING;
    }
    else
    {
        // No turn needed — dispatch the motion primitive directly.
        _dispatchEdgeAfterTurn();
    }
}

void PathTrackerClass::_dispatchEdgeAfterTurn()
{
    const uint8_t cur = Localization.getCurrentNode();
    const Routes::EdgeInfo& edge = FieldMap::getEdge(cur, _nextHop);

    switch (edge.type)
    {
        case Routes::EdgeType::LINE_E:
        {
            LineFollower.setEnabled(true);
            _state = State::LINE_FOLLOW;
            break;
        }
        case Routes::EdgeType::PROJECTION_E:
        {
            LineFollower.setEnabled(true).setDistanceTarget(edge.distance_cm);
            _state = State::PROJECTION_FOLLOW;
            break;
        }
        case Routes::EdgeType::BLIND_E:
        {
            Drive.driveDist(edge.distance_cm).async();
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

void PathTrackerClass::_onGoalBlindDispatch()
{
    if (_activeGoalApproach == nullptr || _blindIndex >= _activeGoalApproach->blind_move_count)
    {
        // Goal sequence complete. Pop next queued goal (if any).
        _activeGoalApproach = nullptr;
        _currentGoal        = FieldMap::NODE_NONE;
        _blindIndex         = 0;
        _state              = (_qCount > 0) ? State::DEQUEUE : State::IDLE;
        return;
    }

    const Routes::BlindMove& m = _activeGoalApproach->blind_moves[_blindIndex];
    _blindIndex++;

    switch (m.action)
    {
        case Routes::BlindAction::TURN_RELATIVE:
        {
            if (fabsf(m.value) > Config::NAV_TURN_TOLERANCE_DEG)
            {
                // Drive.turnDeg expects an int16_t relative delta. The IMU is the source
                // of truth — Drive samples the current yaw and computes the absolute target
                // internally. Round here rather than truncate so a 0.5 deg target maps to 1.
                Drive.turnDeg((int16_t) lroundf(m.value)).async();
                _state = State::GOAL_BLIND_AWAIT;
            }
            else
            {
                // Skipped sub-degree turn — re-enter dispatch on the next tick to advance.
                // Stay in GOAL_BLIND_DISPATCH; the next update() call will pick up the next
                // step. We can also fall through inline, but a single transition per tick
                // keeps the FSM contract uniform.
                _state = State::GOAL_BLIND_DISPATCH;
            }
            break;
        }
        case Routes::BlindAction::FORWARD:
        {
            if (m.value > 0.0f)
            {
                Drive.driveDist(m.value).async();
                _state = State::GOAL_BLIND_AWAIT;
            }
            else
            {
                _state = State::GOAL_BLIND_DISPATCH;
            }
            break;
        }
        case Routes::BlindAction::TURN_TO_HEADING:
        {
            // Absolute heading target (world frame). Convert to IMU frame for Drive.
            const int16_t imuTarget = Localization.toImuYaw(m.value);
            Drive.turnToYaw(imuTarget).async();
            _state = State::GOAL_BLIND_AWAIT;
            break;
        }
        default:
        {
            _enterFault(Fault::UNREACHABLE, Localization.getCurrentNode(), _currentGoal);
            break;
        }
    }
}

// =========================================================================================
// goTo() public surface.
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
