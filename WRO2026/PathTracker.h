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
 *    `DRIVE_FAULT`, `BAD_NODE`, `DRAIN_TIMEOUT`, `REVERSE_UNAVAILABLE`,
 *    `OFF_NODE`) surfaced via `hasFault()` / `getFault()`. `clearFault()` re-arms
 *    the queue but does not restart motion. `faultMnemonic()` returns
 *    a short tag (`UNR`, `DRV`, ...) for the EventTrace / Mission output.
 *
 * FSM states (`State`):
 *  - `IDLE`              — nothing pending; waits for a goal to be enqueued.
 *  - `DEQUEUE`           — pops the next nav entry off the FIFO, then evaluates
 *                          the accumulated operator offset (issue #36, PR1b):
 *                          on a non-trivial offset it drives a return-to-node
 *                          prologue (`RETURN_TO_NODE`) BEFORE planning; on an
 *                          over-cap offset it honors the off-map gate
 *                          (`Fault::OFF_NODE` under `Config::OffMapGate::FAULT`,
 *                          log-and-proceed under `BEST_EFFORT`); otherwise it
 *                          transitions straight to `PLAN_NEXT_HOP` (no behavior
 *                          change for the offset≈0 case).
 *  - `RETURN_TO_NODE`    — drives the approximate inverse of the tracked operator
 *                          offset back onto the anchored node before planning the
 *                          requested goal (issue #36, PR1b). LINE-backed anchors
 *                          re-acquire the line with `Drive.stopAtLine(...)` (the
 *                          same axle-on-node seek the `STOP_AT_LINE` hop uses,
 *                          ADR-027) and re-anchor via `Localization.onIntersection`
 *                          on the SAME node; blind-only anchors dead-reckon the
 *                          inverse drive back and re-anchor via
 *                          `Localization.clearOffset()`. On a `LINE_NOT_FOUND`
 *                          (line never re-acquired) this state routes to the
 *                          off-map gate — `Fault::OFF_NODE` under FAULT, or a
 *                          best-effort accept-dead-reckon under BEST_EFFORT — so
 *                          it is EXEMPT from the generic Drive-fault guard (which
 *                          would otherwise mislabel it `DRIVE_FAULT`). On success
 *                          it transitions to `PLAN_NEXT_HOP`.
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
 *    `projection_mode_override`, `direction_override`). `DEFAULT` means
 *    "inherit `Routes::PIPELINE_CONFIG.<category>`." The dispatcher resolves
 *    DEFAULT → concrete at edge-dispatch time and chains the appropriate
 *    builder setters. The pre-edge turn wheel is no longer an `EdgeInfo`
 *    field: it is resolved per turn from the `Routes::PivotTurn` table
 *    (ADR-036) when the hop departs a pivot node, else the pipeline default.
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

/// @brief Operator-offset POD defined in `Localization.h`. Forward-declared here
///        because the return-to-node helpers (`_evaluateReturnGate` /
///        `_dispatchReturnMove`) take it by const-reference — the full definition
///        is only needed in `PathTracker.cpp`, which includes `Localization.h`.
///        Keeps the header from pulling the lower Localization layer in.
struct Offset2D;

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
        REVERSE_UNAVAILABLE = 6,
        OFF_NODE            = 7  ///< Operator-offset cap exceeded; off-map gate (Config::OFF_MAP_GATE == FAULT) refused to plan return-to-node.
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
        RETURN_TO_NODE,
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

    /**
     * @brief Run one tick of the per-edge FSM (the guard + state switch).
     *
     * The body of `update()` minus the trailing nav-active push. Split out so the
     * public `update()` stays a thin wrapper: it runs the full FSM step (which may
     * resolve a state transition via any of the per-state `return` paths) and THEN
     * pushes the final `_state` down to `Localization.setNavActive(...)`. Keeping the
     * push in `update()` rather than inside this helper guarantees it fires exactly
     * once per tick, after every FSM path has settled.
     */
    void _stepFsm();

    void _enterFault(Fault kind, uint8_t from, uint8_t to);

    /**
     * @brief Evaluate the accumulated operator offset at `goTo()` start and decide
     *        whether to drive a return-to-node prologue (issue #36, PR1b, AC4–AC6).
     *
     * Called from the `DEQUEUE` state right after `_pop(...)` sets `_destinationNode`,
     * BEFORE the transition to `PLAN_NEXT_HOP`. Reads `Localization.getOffsetFromNode()`,
     * computes the translation magnitude `sqrt(off.x^2 + off.y^2)`, and runs the pure
     * `resolveReturnAction(...)` gate (ReturnToNodePolicy.h) over
     * `Config::RETURN_TRIGGER_THRESHOLD_CM`, `RETURN_DISTANCE_CAP_CM`, and
     * `OFF_MAP_GATE`:
     *
     *   - `NONE`            — offset ≈0 (≤ trigger): transition straight to
     *                         `PLAN_NEXT_HOP`. A true no-op — no Drive dispatch, no
     *                         behavior change versus the pre-PR1b code path (AC5).
     *   - `OFF_MAP_FAULT`   — offset over cap, `OFF_MAP_GATE == FAULT`: latch
     *                         `Fault::OFF_NODE` and refuse to plan (AC6).
     *   - `OFF_MAP_PROCEED` — offset over cap, `OFF_MAP_GATE == BEST_EFFORT`: log a
     *                         Serial warning and dispatch the (capped) return anyway
     *                         (AC6).
     *   - `RETURN`          — trigger < offset ≤ cap: dispatch the return via
     *                         `_dispatchReturnMove(...)` (AC4).
     *
     * @param off The accumulated operator offset (world-frame cm + heading delta).
     * @param mag The pre-computed translation magnitude `sqrt(off.x^2 + off.y^2)`.
     */
    void _evaluateReturnGate(const Offset2D& off, float mag);

    /**
     * @brief Dispatch the inverse-of-offset return move onto the anchored node and
     *        enter the `RETURN_TO_NODE` state (issue #36, PR1b, AC4).
     *
     * Drives the chassis from its current (hand-pushed) pose back toward the
     * anchored node. The return-travel world heading is the direction from the
     * current pose back to the anchor — i.e. the heading of the inverse offset
     * vector `(-off.x, -off.y)`, computed as `atan2(-off.y, -off.x)` and normalized
     * to `[0, 360)`. The drive distance is the offset translation magnitude @p
     * mag, clamped to `Config::RETURN_DISTANCE_CAP_CM` so a best-effort over-cap
     * return cannot run unbounded.
     *
     * Node-kind dispatch (`FieldMap::getNodeType(Localization.getCurrentNode())`):
     *  - LINE  : re-acquire the line with a forward `Drive.stopAtLine(...)` seek
     *            facing the return heading. The seek lands the wheel AXLE on the
     *            node (ADR-027) and the `RETURN_TO_NODE` state re-anchors the SAME
     *            node via `Localization.onIntersection(...)` on success. The max
     *            seek distance is the capped magnitude (the crossing, not the
     *            distance, terminates a healthy seek).
     *  - BLIND : there is no line to snap onto — turn to face the return heading,
     *            then dead-reckon the inverse drive back. The `RETURN_TO_NODE`
     *            state treats `Drive.isIdle()` as arrival and re-anchors via
     *            `Localization.clearOffset()` (the node id does not change).
     *
     * Sets `_returnIsLineBacked` so the `RETURN_TO_NODE` state body knows which
     * arrival/re-anchor semantics to apply. Every Drive call is emitted as a single
     * discarded-temporary `.async()` statement (RAII builder rule — never split a
     * builder chain across statements / a local variable).
     *
     * @param off The accumulated operator offset (world-frame cm + heading delta).
     * @param mag The offset translation magnitude `sqrt(off.x^2 + off.y^2)`.
     */
    void _dispatchReturnMove(const Offset2D& off, float mag);

    void _onPlanNextHop();
    void _dispatchEdgeAfterTurn();

    /**
     * @brief One-hop lookahead: the hop AFTER `_nextHop` toward `_destinationNode`.
     *
     * Shared guard for the two FSM sites that peek the next-next hop — the
     * `LINE_FOLLOW` pass-through decision in `update()` and the BLIND auto-rule
     * projection classification (`_nextHopIsProjection()`). Centralises the three
     * guards both need so they cannot drift: the not-final guard (`_nextHop` is the
     * destination), the `NODE_NONE` guard (no route from `_nextHop` to the goal),
     * and the self-loop guard (the lookahead returns `_nextHop` itself).
     *
     * @return The next-next node id, or `FieldMap::NODE_NONE` when any guard fails
     *         (there is no valid next-next hop to classify).
     */
    uint8_t _peekNextNextHop() const;

    /**
     * @brief ADR-038 auto-rule predicate: is the hop after `_nextHop` a PROJECTION edge?
     *
     * Feeds the `autoRuleActive` input of `resolveStopAtLine(...)` on a BLIND hop —
     * a blind hop that feeds a projection edge seeks a line first to re-anchor
     * before the marker-less projection. Reuses the shared `_peekNextNextHop()`
     * guard and classifies the resulting edge type.
     *
     * @return `true` when the next-next edge is `Routes::EdgeType::PROJECTION`.
     * @note Short-circuits to `false` — with NO `NEXT_HOP` lookup — when the
     *       feature is compiled out via `Routes::BLIND_PROJECTION_AUTO_SEEK ==
     *       false`, so the auto-seek peek costs nothing on a build that never
     *       auto-seeks.
     */
    bool _nextHopIsProjection() const;

    /**
     * @brief Compute the effective dead-reckon distance for a DISTANCE-terminated
     *        hop, applying the per-turn pivot trims (ADR-036).
     *
     * Called only on distance-terminated dispatches (BLIND dead-reckon
     * `driveDist`, PROJECTION distance gate). Subtracts the two RESOLVED pivot
     * trims latched at plan time into a LOCAL effective distance —
     * `EDGES[*][*].distance_cm` is never mutated:
     *   1. DEPARTING trim: if this hop LEAVES a pivot node (`_departingPivot`
     *      active), subtract its δ (the outgoing over-travel of the pivot we
     *      just turned through).
     *   2. APPROACHING trim: if this dead-reckon edge drives INTO a pivot node
     *      (`_approachingPivot` active), subtract its δ so the chassis stops δ
     *      short and the pivot starts at the geometric pivot point.
     * On a chain `u→v→w→x` the hop `v→w` nets `length − δ_v(departing) −
     * δ_w(approaching)`. The result is clamped to `Config::PIVOT_TRIM_MIN_DRIVE_CM`
     * so a bad table can never command a non-positive drive (which
     * `Drive.driveDist` would reject). No carry/latch lives here anymore — the
     * one-hop carry is owned by the `_pendingPivot` latch contract resolved in
     * `_onPlanNextHop()`. The subtract-and-clamp itself is the shared
     * `resolvePivotTrim` policy (PivotTrimPolicy.h); this method only gathers the
     * latched trims and traces an underflow.
     *
     * @param edge The directed edge being dispatched (`getEdge(cur, _nextHop)`).
     * @return The trimmed, clamped distance to feed the distance-terminated builder.
     */
    float _pivotTrimDistance(const Routes::EdgeInfo& edge);

    /**
     * @brief Clear all three pivot-latch members to the inactive sentinel (ADR-036).
     *
     * Resets `_pendingPivot`, `_departingPivot`, and `_approachingPivot` to a
     * default-constructed `PivotRef{}` (`active == false`). Called at every point
     * the latch must not survive: boot (`begin`), abort, fault (`_enterFault`),
     * DEQUEUE of a fresh goal, and the goal-complete IDLE/DEQUEUE transition.
     * Centralising the reset keeps the three members in lock-step — a site that
     * cleared only some of them would leak a pivot across a `goTo(...)` boundary,
     * the exact cross-goal bleed these resets guard against.
     */
    void _resetPivotLatch();

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

    /// @brief Node kind of the anchor for the IN-FLIGHT return-to-node prologue (PR1b).
    ///
    /// Set by `_dispatchReturnMove(...)` when entering `RETURN_TO_NODE`: `true` for a
    /// LINE-backed anchor (a `Drive.stopAtLine` seek re-anchors via `onIntersection`),
    /// `false` for a blind-only anchor (a dead-reckoned `Drive.driveDist` re-anchors via
    /// `clearOffset`). Read only by the `RETURN_TO_NODE` state body to pick the success
    /// path. Meaningless outside `RETURN_TO_NODE`; defaults to `false`.
    bool _returnIsLineBacked = false;

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

    /// @brief A RESOLVED COPY of a `Routes::PivotTurn` record (ADR-036).
    ///
    /// Never a pointer into the `PIVOT_TURNS` table kept across hops — the
    /// firmware resolves the turn once, copies the wheel + δ out, and consumes
    /// the copy at the maneuver and the drive. `active == false` is the
    /// "no pivot" sentinel (every field at its default).
    struct PivotRef
    {
        bool             active  = false;
        Routes::TurnMode wheel   = Routes::TurnMode::DEFAULT;
        float            trim_cm = 0.0f;
    };

    /// @brief The per-turn pivot latch contract (ADR-036, supersedes ADR-035).
    ///
    /// A pivot is a property of a 3-node turn `(u, v, w)`, not of an edge. At
    /// plan time `_onPlanNextHop()` resolves the approaching pivot via a one-hop
    /// lookahead (`findPivotTurn(cur, nxt, getNextHop(nxt, dest))`) and latches
    /// it. The single `_pendingPivot` carry suffices ONLY because the departing
    /// pivot is snapshotted BEFORE `_pendingPivot` is overwritten — see the
    /// "snapshot BEFORE overwrite" sequence in `_onPlanNextHop()`.
    ///
    ///  - `_pendingPivot`     — carry: the approaching pivot latched LAST hop,
    ///                          which becomes THIS hop's departing pivot. The
    ///                          critical one to clear across goals so a stop at
    ///                          `v` degrades to a normal turn and a pivot never
    ///                          fires across a `goTo()` boundary.
    ///  - `_departingPivot`   — the pivot we LEAVE at `cur` this hop (snapshot of
    ///                          `_pendingPivot`). Supplies the pre-turn maneuver
    ///                          wheel and the departing (outgoing) dead-reckon trim.
    ///  - `_approachingPivot` — the pivot we ENTER at `nxt` this hop (fresh
    ///                          lookahead). Supplies the approach trim on a
    ///                          dead-reckon edge or the LINE overshoot reduction.
    ///
    /// All three are members (not locals) because they are set at plan time and
    /// must survive the `TURNING` state into the drive dispatch (and, for the
    /// approaching pivot, into the `LINE_FOLLOW` overshoot-arm in a different FSM
    /// state). Reset to inactive on abort, on a fault, on DEQUEUE of a fresh
    /// goal, and on the IDLE/done transition.
    PivotRef _pendingPivot;
    PivotRef _departingPivot;
    PivotRef _approachingPivot;

    FaultInfo _fault;
    void (*_tickCallback)()        = nullptr;
    bool  _tickCallbackWarningEmitted = false;
};

extern PathTrackerClass PathTracker;

#endif    // PATHTRACKER_H
