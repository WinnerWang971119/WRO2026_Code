/**
 * @file PathTracker.h
 * @brief Non-blocking PathTracker FSM + fluent goTo() builder (PR3).
 * @author WRO 2026
 *
 * @details
 * Consumes the precomputed `Routes::NEXT_HOP` table and dispatches one
 * edge at a time onto `Drive` and `LineFollower` to navigate the chassis
 * from its current node (single source of truth: `Localization`) toward
 * a queued destination node.
 *
 * Owns:
 *  - A FIFO queue of pending navigation entries (`Config::PATH_QUEUE_CAPACITY`).
 *  - A per-edge state machine driven from `update()` (cooperative tick).
 *  - A fluent `goTo(node)` builder. Default is sync-blocking; chain
 *    `.async()` to enqueue + return immediately.
 *  - A latched fault state (`Fault::NOT_ZEROED`, `UNREACHABLE`,
 *    `DRIVE_FAULT`, `BAD_NODE`, `DRAIN_TIMEOUT`) surfaced via
 *    `hasFault()` / `getFault()`. `clearFault()` re-arms the queue but
 *    does not restart motion. `faultMnemonic()` returns a short tag
 *    (`UNR`, `DRV`, ...) for the EventTrace / Mission output.
 *
 * FSM states (`State`):
 *  - `IDLE`              — nothing pending; waits for a goal to be enqueued.
 *  - `DEQUEUE`           — pops the next nav entry off the FIFO.
 *  - `PLAN_NEXT_HOP`     — resolves the next edge via `FieldMap::getNextHop`,
 *                          dispatches the pre-edge yaw turn (if non-trivial).
 *  - `TURNING`           — waits for `Drive.isIdle()` after a turn dispatch.
 *  - `LINE_FOLLOW`       — waits for `Intersection.isAtIntersection()` while
 *                          the LineFollower drives the chassis along a line
 *                          segment.
 *  - `LINE_OVERSHOOT`    — after intersection-detect, re-arms LineFollower
 *                          with a distance gate equal to
 *                          `Config::LINE_SENSOR_AXLE_OFFSET_CM` so the
 *                          chassis center (axle) lands on the node, not
 *                          the sensor. Waits for
 *                          `LineFollower.isDistanceReached()`.
 *  - `PROJECTION_FOLLOW` — line-follows an edge of type `EdgeType::PROJECTION`
 *                          using the edge's `distance_cm` as the gate,
 *                          since the next node has no physical intersection
 *                          marker. Waits for
 *                          `LineFollower.isDistanceReached()`.
 *  - `BLIND_FORWARD`     — waits for `Drive.isIdle()` after a dead-reckoned
 *                          forward move.
 *  - `FAULT`             — latched fault; cleared via `clearFault()` or
 *                          `abort()`.
 *
 * Per-edge motion-mode resolution:
 *  - Each `Routes::EdgeInfo` carries per-category override fields
 *    (`line_mode_override`, `blind_mode_override`,
 *    `projection_mode_override`, `pre_turn_mode_override`). `DEFAULT` means
 *    "inherit `Routes::PIPELINE_CONFIG.<category>`." The dispatcher resolves
 *    DEFAULT → concrete at edge-dispatch time and chains the appropriate
 *    builder setters.
 *
 * Iron rules (match `Drive`'s discipline):
 *  1. `update()` is non-blocking; every Drive call is `.async()`.
 *  2. Localization is the only writer of `_currentNode` — PathTracker
 *     advances it via `onIntersection(...)` / `onDeadReckonedAdvance(...)`
 *     ONLY after the corresponding edge has truly completed (for LINE
 *     edges, this means AFTER the overshoot has rolled the axle onto the
 *     node, not at the intersection-detect tick).
 */

#ifndef PATHTRACKER_H
#define PATHTRACKER_H

#include <Arduino.h>
#include <stdint.h>

#include "Config.h"
#include "FieldMap.h"
#include "NavTypes.h"
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

    /**
     * @brief Snapshot of the latched fault, returned by `getFault()`.
     *
     * @note `destNode` is the active destination node that was in flight
     *       when the fault tripped (renamed from the historical
     *       `goalNode` field as part of the PR3 internals cleanup).
     */
    struct FaultInfo
    {
        Fault   kind     = Fault::NONE;
        uint8_t fromNode = FieldMap::NODE_NONE;
        uint8_t destNode = FieldMap::NODE_NONE;
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
     * @brief Hard-cancel any in-flight goal: stop chassis, empty queue,
     *        reset FSM to IDLE, clear any latched fault.
     *
     * Symmetric with `Drive.stop(true)`. Use this as a real safety stop
     * (e.g. `Mission.abort()`) — `clearFault()` alone is a no-op when
     * the FSM is mid-edge (`TURNING` / `LINE_FOLLOW` / `LINE_OVERSHOOT` /
     * `PROJECTION_FOLLOW` / `BLIND_FORWARD`), which would otherwise let
     * the next `update()` tick resume motion.
     */
    void       abort();

    /**
     * @brief Short mnemonic ("UNR", "DRV", ...) for a Fault enum value.
     */
    static const char* faultMnemonic(Fault kind);

private:
    /**
     * @brief Per-edge FSM state. See class doxygen for the full lifecycle.
     */
    enum class State : uint8_t
    {
        IDLE,
        DEQUEUE,
        PLAN_NEXT_HOP,
        TURNING,
        LINE_FOLLOW,
        LINE_OVERSHOOT,
        PROJECTION_FOLLOW,
        BLIND_FORWARD,
        FAULT
    };

    /**
     * @brief One pending entry on PathTracker's FIFO.
     *
     * A `goTo(...)` call enqueues exactly one of these; the destination
     * node id is the only payload (the planner re-resolves the next-hop
     * at every PLAN_NEXT_HOP tick, so no edge state needs to be cached).
     * Renamed from `GoalEntry` (PR3 internals cleanup) — the public
     * `goTo()` verb is unchanged.
     */
    struct NavEntry
    {
        uint8_t nodeId;
    };

    bool _enqueue(uint8_t goalId);
    bool _pop(NavEntry& out);
    void _clearQueue();

    void _enterFault(Fault kind, uint8_t from, uint8_t to);
    void _onPlanNextHop();
    void _dispatchEdgeAfterTurn();

    void _drainBlocking();

    void _applyGoToBuilder(uint8_t goalId, bool blocking);

    NavEntry _queue[Config::PATH_QUEUE_CAPACITY];
    uint8_t  _qHead  = 0;
    uint8_t  _qTail  = 0;
    uint8_t  _qCount = 0;

    State            _state           = State::IDLE;
    uint8_t          _destinationNode = FieldMap::NODE_NONE;
    uint8_t          _nextHop         = FieldMap::NODE_NONE;
    Routes::EdgeType _activeEdgeType  = Routes::EdgeType::NONE;

    /// @brief Pass-through eligibility for the in-flight LINE_OVERSHOOT.
    ///
    /// Set in `update()`'s `LINE_FOLLOW` case at intersection-detect when the
    /// next-hop edge after `_nextHop` (toward `_destinationNode`) is collinear
    /// with the inbound edge — heading delta wrap-normalized to within
    /// `Config::NAV_TURN_TOLERANCE_DEG`. Consumed by `LINE_OVERSHOOT` (T5) and
    /// `_dispatchEdgeAfterTurn` (T6/T7) to skip the brake-and-restart cycle.
    /// Reset to false on every fresh intersection-detect tick.
    /// See ADR-023 and issue #19.
    bool _passThrough = false;

    /// @brief Resolved direction (FRONT/REAR) for the current hop.
    ///
    /// Set per-hop in `_onPlanNextHop()` by `_resolveDirection(...)` and consumed
    /// by `_dispatchEdgeAfterTurn()` (T7) to pick forward vs. reverse dispatch
    /// onto LineFollower/Drive. `AUTO` is the unresolved sentinel (only valid
    /// before the first plan tick).
    Direction _resolvedDirection = Direction::AUTO;

    /// @brief Resolved body-target heading (world frame, degrees in `[0, 360)`)
    ///        for the current hop.
    ///
    /// Equal to `edge.heading_deg` when `_resolvedDirection == FRONT`, or
    /// `(edge.heading_deg + 180°) mod 360` when `_resolvedDirection == REAR`.
    /// This is the heading the chassis BODY must face — used both for the
    /// `Drive.turnToYaw(...)` target AND for the `NAV_TURN_TOLERANCE_DEG`
    /// skip-turn comparison in `_onPlanNextHop()`. The pass-through peek
    /// (T6) also uses it as the inbound-body heading when classifying the
    /// next-next hop's geometry.
    float _resolvedBodyHeading = 0.0f;

    FaultInfo _fault;
    void (*_tickCallback)()        = nullptr;
    bool  _tickCallbackWarningEmitted = false;
};

extern PathTrackerClass PathTracker;

#endif    // PATHTRACKER_H
