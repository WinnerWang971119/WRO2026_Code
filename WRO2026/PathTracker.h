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
 *    `DRIVE_FAULT`, `BAD_NODE`, `DRAIN_TIMEOUT`, `REVERSE_UNAVAILABLE`)
 *    surfaced via `hasFault()` / `getFault()`. `clearFault()` re-arms
 *    the queue but does not restart motion. `faultMnemonic()` returns
 *    a short tag (`UNR`, `DRV`, ...) for the EventTrace / Mission output.
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
 *  - `STOP_AT_LINE`      — waits for `Drive.isIdle()` after a
 *                          `Drive.stopAtLine(...)` dispatch on a BLIND hop
 *                          resolved to stop-at-line. On success re-anchors
 *                          `Localization` via `onIntersection(...)` (a true
 *                          re-anchor, not a dead-reckon advance); on
 *                          `Drive.hasFault()` enters `Fault::DRIVE_FAULT`.
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

    /**
     * @brief Force the direction of travel for EVERY hop on the path to the
     *        goal. Defaults to Direction::AUTO (let _resolveDirection pick).
     * @note  Loses to per-edge `direction_override` (set in field_data.py).
     *        See Notes for Implementer in the spec.
     */
    PathTrackerGoToBuilder& setRouteDirection(Direction d);

    /**
     * @brief Force the direction of travel for the FINAL hop into the goal.
     *        Intermediate hops still use auto (or whatever `setRouteDirection`
     *        set). Defaults to Direction::AUTO.
     * @note  Wins over `setRouteDirection` for the final hop only.
     */
    PathTrackerGoToBuilder& setArrivalDirection(Direction d);

    /**
     * @brief Force stop-at-line behaviour for the BLIND hops on the path.
     *        Defaults to StopAtLine::AUTO (defer to each edge's BlindMode
     *        override).
     * @param s  ON forces stop-at-line on a resolved BLIND hop even when the
     *           edge is `BlindMode::DEFAULT`; OFF force-disables it even when
     *           the edge says `STOP_AT_LINE`; AUTO defers to the edge.
     * @note  The call-site ON/OFF wins over the per-edge BlindMode override;
     *        AUTO defers to it (mirrors `setArrivalDirection` precedence).
     *        Only affects hops whose resolved edge is `EdgeType::BLIND`.
     */
    PathTrackerGoToBuilder& setStopAtLine(StopAtLine s);

private:
    PathTrackerClass& _owner;
    uint8_t           _goalId;
    bool              _blocking         = true;
    bool              _executed         = false;
    Direction         _routeDirection   = Direction::AUTO;
    Direction         _arrivalDirection = Direction::AUTO;
    StopAtLine        _stopAtLine       = StopAtLine::AUTO;
};

class PathTrackerClass
{
    friend class PathTrackerGoToBuilder;

public:
    enum class Fault : uint8_t
    {
        NONE                = 0,
        NOT_ZEROED          = 1,
        UNREACHABLE         = 2,
        DRIVE_FAULT         = 3,
        BAD_NODE            = 4,
        DRAIN_TIMEOUT       = 5,
        REVERSE_UNAVAILABLE = 6
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
        STOP_AT_LINE,
        FAULT
    };

    /**
     * @brief One pending entry on PathTracker's FIFO.
     *
     * A `goTo(...)` call enqueues exactly one of these. The destination
     * node id is the planner payload (re-resolved at every PLAN_NEXT_HOP
     * tick, so no edge state needs to be cached). The two `Direction`
     * fields carry the per-call bidirectional builder overrides:
     *
     *  - `route_direction`   — applies to EVERY hop on the path (T8).
     *  - `arrival_direction` — applies only to the FINAL hop into the
     *                          destination; wins over `route_direction`
     *                          for that hop. See `_resolveDirection` for
     *                          precedence semantics.
     *  - `stop_at_line`      — call-site stop-at-line override for the path's
     *                          BLIND hops; ON/OFF wins over the per-edge
     *                          BlindMode, AUTO defers to it.
     *
     * All three default to their `AUTO` enumerator (= 0), so zero-initialized
     * FIFO slots yield AUTO naturally — no constructor needed.
     *
     * Renamed from `GoalEntry` (PR3 internals cleanup) — the public
     * `goTo()` verb is unchanged.
     */
    struct NavEntry
    {
        uint8_t    nodeId;
        Direction  route_direction;      ///< Forces every hop's direction (default AUTO).
        Direction  arrival_direction;    ///< Forces final-hop direction (default AUTO).
        StopAtLine stop_at_line;         ///< Forces BLIND-hop stop-at-line (default AUTO).
    };

    bool _enqueue(uint8_t goalId, Direction routeDir, Direction arrivalDir);
    bool _pop(NavEntry& out);
    void _clearQueue();

    void _enterFault(Fault kind, uint8_t from, uint8_t to);
    void _onPlanNextHop();
    void _dispatchEdgeAfterTurn();

    /**
     * @brief Compute the effective dead-reckon distance for a DISTANCE-terminated
     *        pivot edge, applying ADR-035 two-sided pivot trims.
     *
     * Called only on distance-terminated dispatches (BLIND dead-reckon
     * `driveDist`, PROJECTION distance gate). Folds three ADR-035 effects into a
     * LOCAL effective distance — `EDGES[*][*].distance_cm` is never mutated:
     *   1. OUTGOING trim: if a δ is carried from the previous pivot hop, subtract
     *      it (the outgoing over-travel of that pivot), then clear the carry.
     *   2. APPROACH trim: if this edge itself drives INTO a pivot node
     *      (`edge.pivot_trim_cm > 0`), subtract δ so the chassis stops δ short.
     *   3. LATCH: if this edge is a pivot edge, latch its δ into
     *      `_carriedPivotTrimCm` for the NEXT hop's outgoing trim.
     * The result is clamped to `PIVOT_TRIM_MIN_DRIVE_CM` so a bad table can never
     * command a non-positive drive (which `Drive.driveDist` would reject).
     *
     * @param edge The directed edge being dispatched (`getEdge(cur, _nextHop)`).
     * @return The trimmed, clamped distance to feed the distance-terminated builder.
     */
    float _pivotTrimDistance(const Routes::EdgeInfo& edge);

    /**
     * @brief Resolve the ADR-035 carried δ on a SENSOR-terminated pivot hop.
     *
     * Sensor-terminated dispatches (LINE follow, STOP_AT_LINE seek) self-correct
     * on the physical line, so they take NO outgoing trim: any δ carried from the
     * previous pivot hop is simply DROPPED (it lives exactly one hop). When
     * @p latchOwn is true, this edge's own δ is still latched into
     * `_carriedPivotTrimCm` for the next hop — a pivot reached via a STOP_AT_LINE
     * seek still over-travels on the OUTGOING side. (LINE edges latch their own δ
     * later, at overshoot-arm time, alongside the overshoot reduction, so they
     * pass `latchOwn = false` here.)
     *
     * @param edge     The directed edge being dispatched.
     * @param latchOwn Whether to latch this edge's own δ for the outgoing hop.
     */
    void _resolveSensorEdgePivotCarry(const Routes::EdgeInfo& edge, bool latchOwn);

    void _drainBlocking();

    void _applyGoToBuilder(
        uint8_t    goalId,
        bool       blocking,
        Direction  routeDir,
        Direction  arrivalDir,
        StopAtLine stopAtLine);

    NavEntry _queue[Config::PATH_QUEUE_CAPACITY];
    uint8_t  _qHead  = 0;
    uint8_t  _qTail  = 0;
    uint8_t  _qCount = 0;

    State            _state           = State::IDLE;
    uint8_t          _destinationNode = FieldMap::NODE_NONE;
    uint8_t          _nextHop         = FieldMap::NODE_NONE;
    Routes::EdgeType _activeEdgeType  = Routes::EdgeType::NONE;

    /// @brief Bidirectional builder overrides cached for the IN-FLIGHT goal.
    ///
    /// Populated at `DEQUEUE` from the popped `NavEntry` and consumed by
    /// `_onPlanNextHop()` (every hop) and by the pass-through peek in
    /// `update()`'s `LINE_FOLLOW` case (next-next hop classification).
    /// Both default to `Direction::AUTO` — a bare `goTo(...)` with no
    /// modifier keeps them AUTO, so `_resolveDirection` falls through to
    /// the auto algorithm. See ADR-023 and issue #19.
    Direction _activeRouteDirection   = Direction::AUTO;
    Direction _activeArrivalDirection = Direction::AUTO;

    /// @brief Call-site stop-at-line override cached for the IN-FLIGHT goal.
    ///
    /// Populated at `DEQUEUE` from the popped `NavEntry` (parallel to the
    /// `Direction` overrides above) and consumed by the BLIND-hop dispatch in
    /// `update()`: ON/OFF wins over the edge's `BlindMode` override, AUTO
    /// defers to it. Defaults to `StopAtLine::AUTO` — a bare `goTo(...)` with
    /// no modifier keeps it AUTO, so the per-edge BlindMode governs. See ADR-027.
    StopAtLine _activeStopAtLine = StopAtLine::AUTO;

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

    /// @brief δ (cm) carried ONE hop for the outgoing pivot trim; 0 = none.
    ///        See ADR-035 / issue #44 (two-sided pivot compensation).
    ///
    /// A single-wheel pivot at node `v` over-travels by `δ` on BOTH adjacent
    /// drives. `δ` is stored on the pivot-tagged edge `u→v` as
    /// `EdgeInfo::pivot_trim_cm`. The INCOMING side is trimmed while the pivot
    /// edge is dispatched (approach trim on a dead-reckon edge, or an
    /// overshoot-advance reduction on a LINE edge). The OUTGOING side cannot
    /// see `u→v` once the FSM has advanced to `v`, so `δ` is latched here at
    /// the pivot-edge dispatch and consumed on the NEXT hop `v→w`:
    ///   - `v→w` distance-terminated (dead-reckon `driveDist` / PROJECTION
    ///     gate)  → subtract `δ` from the effective drive (outgoing trim).
    ///   - `v→w` sensor-terminated (LINE / STOP_AT_LINE) → no trim; the carry
    ///     is simply DROPPED (the line re-anchors v→w, so δ is inert there).
    /// Either way the carry lives exactly one hop. Reset to 0 on abort, on a
    /// fault, on DEQUEUE of a fresh goal, and on the IDLE/done transition so a
    /// pivot edge that is the FINAL hop cannot leak δ into the next goal.
    float _carriedPivotTrimCm = 0.0f;

    FaultInfo _fault;
    void (*_tickCallback)()        = nullptr;
    bool  _tickCallbackWarningEmitted = false;
};

extern PathTrackerClass PathTracker;

#endif    // PATHTRACKER_H
