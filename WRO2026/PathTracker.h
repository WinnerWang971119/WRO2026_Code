/**
 * @file PathTracker.h
 * @brief Non-blocking PathTracker FSM + fluent goTo() builder (PR3).
 * @author WRO 2026
 *
 * @details
 * Consumes the precomputed `Routes::NEXT_HOP` table and dispatches one
 * edge at a time onto `Drive` and `LineFollower` to navigate the chassis
 * from its current node to a goal node. PR3 simplification: only LINE
 * and BLIND edge types; the PR2 PROJECTION/GOAL_APPROACH states are
 * gone.
 *
 * Owns:
 *  - A FIFO queue of pending goal entries (`Config::PATH_QUEUE_CAPACITY`).
 *  - A per-edge state machine driven from `update()` (cooperative tick).
 *  - A fluent `goTo(node)` builder. Default is sync-blocking; chain
 *    `.async()` to enqueue + return immediately.
 *  - A latched fault state (`Fault::NOT_ZEROED`, `UNREACHABLE`,
 *    `DRIVE_FAULT`, `BAD_NODE`, `DRAIN_TIMEOUT`) surfaced via
 *    `hasFault()` / `getFault()`. `clearFault()` re-arms the queue but
 *    does not restart motion. `faultMnemonic()` returns a short tag
 *    (`UNR`, `DRV`, ...) for the EventTrace / Mission output.
 *
 * Iron rules (match `Drive`'s discipline):
 *  1. `update()` is non-blocking; every Drive call is `.async()`.
 *  2. Localization is the only writer of `_currentNode`.
 */

#ifndef PATHTRACKER_H
#define PATHTRACKER_H

#include <Arduino.h>
#include <stdint.h>

#include "Config.h"
#include "FieldMap.h"
#include "routes.h"

class PathTrackerClass;

/**
 * @brief RAII fluent builder returned by `PathTrackerClass::goTo(...)`.
 *
 * Sync-blocking by default: destructor enqueues + drains. Chain
 * `.async()` to opt out (the destructor still enqueues but returns).
 * Mission DSL uses `.async()` and polls `isIdle()`.
 */
class [[nodiscard]] PathTrackerGoToBuilder
{
public:
    PathTrackerGoToBuilder(PathTrackerClass& owner, uint8_t goalId);

    PathTrackerGoToBuilder(const PathTrackerGoToBuilder&)            = delete;
    PathTrackerGoToBuilder& operator=(const PathTrackerGoToBuilder&) = delete;
    PathTrackerGoToBuilder(PathTrackerGoToBuilder&&)                 = delete;
    PathTrackerGoToBuilder& operator=(PathTrackerGoToBuilder&&)      = delete;

    ~PathTrackerGoToBuilder();

    /**
     * @brief Opt out of the default sync-blocking semantics.
     */
    PathTrackerGoToBuilder& async();

private:
    PathTrackerClass& _owner;
    uint8_t           _goalId;
    bool              _blocking = true;
    bool              _executed = false;
};

class PathTrackerClass
{
    friend class PathTrackerGoToBuilder;

public:
    enum class Fault : uint8_t
    {
        NONE         = 0,
        NOT_ZEROED   = 1,
        UNREACHABLE  = 2,
        DRIVE_FAULT  = 3,
        BAD_NODE     = 4,
        DRAIN_TIMEOUT = 5
    };

    struct FaultInfo
    {
        Fault   kind     = Fault::NONE;
        uint8_t fromNode = FieldMap::NODE_NONE;
        uint8_t goalNode = FieldMap::NODE_NONE;
    };

    bool begin();
    void update();
    void setTickCallback(void (*cb)());

    bool isIdle() const;

    [[nodiscard]] PathTrackerGoToBuilder goTo(uint8_t goalId);
    [[nodiscard]] PathTrackerGoToBuilder goTo(const char* goalName);

    bool       hasFault() const;
    FaultInfo  getFault() const;
    void       clearFault();

    /**
     * @brief Short mnemonic ("UNR", "DRV", ...) for a Fault enum value.
     */
    static const char* faultMnemonic(Fault kind);

private:
    enum class State : uint8_t
    {
        IDLE,
        DEQUEUE,
        PLAN_NEXT_HOP,
        TURNING,
        LINE_FOLLOW,
        BLIND_FORWARD,
        FAULT
    };

    struct GoalEntry
    {
        uint8_t goalId;
    };

    bool _enqueue(uint8_t goalId);
    bool _pop(GoalEntry& out);
    void _clearQueue();

    void _enterFault(Fault kind, uint8_t from, uint8_t to);
    void _onPlanNextHop();
    void _dispatchEdgeAfterTurn();

    void _drainBlocking();

    void _applyGoToBuilder(uint8_t goalId, bool blocking);

    GoalEntry _queue[Config::PATH_QUEUE_CAPACITY];
    uint8_t   _qHead  = 0;
    uint8_t   _qTail  = 0;
    uint8_t   _qCount = 0;

    State            _state          = State::IDLE;
    uint8_t          _currentGoal    = FieldMap::NODE_NONE;
    uint8_t          _nextHop        = FieldMap::NODE_NONE;
    Routes::EdgeType _activeEdgeType = Routes::EdgeType::NONE;

    FaultInfo _fault;
    void (*_tickCallback)()        = nullptr;
    bool  _tickCallbackWarningEmitted = false;
};

extern PathTrackerClass PathTracker;

#endif    // PATHTRACKER_H
