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
 *    `OFF_NODE`, `WALL_NOT_SQUARE`, `PIVOT_TRIM_UNDERFLOW`, `LINE_LOST`)
 *    surfaced via `hasFault()` / `getFault()`.
 *    `clearFault()` re-arms the queue but does not restart motion.
 *    `faultMnemonic()` returns a short tag (`UNR`, `DRV`, ...) for the
 *    EventTrace / Mission output.
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
 *                          dispatches the pre-edge yaw turn (if non-trivial) via the shared
 *                          `_dispatchTurnIfNeeded(...)`.
 *  - `TURNING`           — waits for `Drive.isIdle()` after a pre-edge turn dispatch, then
 *                          dispatches the move: a normal hop's edge
 *                          (`_dispatchEdgeAfterTurn`) or, when a return prologue is in flight
 *                          (`_returnActive`), the return seek/drive
 *                          (`_dispatchReturnMoveAfterTurn`). The return reuses this SAME
 *                          pre-turn path, so a return can never skip the turn.
 *  - `LINE_FOLLOW`       — waits for `Intersection.isAtIntersection()` while
 *                          the LineFollower drives the chassis along a line
 *                          segment.
 *  - `LINE_OVERSHOOT`    — after intersection-detect, re-arms LineFollower
 *                          with a distance gate equal to
 *                          `Config::LINE_SENSOR_AXLE_OFFSET_CM` so the
 *                          chassis center (axle) lands on the node, not
 *                          the sensor. Waits for
 *                          `LineFollower.isDistanceReached()`.
 *  - `PROJECTION_FOLLOW` — line-follows a LINE edge whose DESTINATION is a
 *                          `NodeType::PROJECTION` node, using the (pivot-trimmed)
 *                          edge `distance_cm` as the gate, since a projection
 *                          node sits on the line but at no physical intersection
 *                          marker. Waits for `LineFollower.isDistanceReached()`,
 *                          then dead-reckons the advance (`onDeadReckonedAdvance`)
 *                          — there is no junction to re-anchor against.
 *                          `line(A)–projection(B)–line(C)` PASS-THROUGH (T19,
 *                          issue #64): when B is a NON-goal projection node, the
 *                          outbound edge B→C is also a LINE edge into a LINE
 *                          (junction) node, and A→B→C are collinear (ADR-023
 *                          heading check), this state does NOT stop at B. The
 *                          DISTANCE GATE — not `Intersection.isAtIntersection()`,
 *                          which never fires at a marker-less projection node — is
 *                          the pass-through trigger: it stamps `onDeadReckonedAdvance(B)`
 *                          MID-FOLLOW (LineFollower stays enabled, no brake, no
 *                          re-kick) and continues line-following toward C as the
 *                          B→C hop, where `isAtIntersection()` DOES fire and
 *                          re-anchors normally. The eligibility is latched at arm
 *                          time into `_projPassThrough`, which also armed the gate
 *                          with `brakeOnGate == false` so the gate trip never
 *                          self-disables the follower. Any failed gate condition
 *                          falls back to the normal stop-at-B per-hop path.
 *  - `BLIND_FORWARD`     — waits for `Drive.isIdle()` after a dead-reckoned
 *                          forward move. ADR-059 blind-chain: when a collinear
 *                          run of BLIND dead-reckon hops is in flight (the
 *                          `_chain` run buffer is active), each NON-final hop
 *                          was dispatched with chained `setStartSpeed` /
 *                          `setEndSpeed` + `setBrake(false)` so the chassis
 *                          never brakes to rest at the intermediate node; this
 *                          state still stamps `onDeadReckonedAdvance` exactly
 *                          once per node (blind nodes have no positional
 *                          re-anchor). The run's FINAL hop is an ordinary
 *                          braked stop and retires the run at dispatch.
 *  - `STOP_AT_LINE`      — waits for `Drive.isIdle()` after a
 *                          `Drive.stopAtLine(...)` dispatch on a BLIND hop
 *                          resolved to stop-at-line. On success re-anchors
 *                          `Localization` via `onIntersection(...)` (a true
 *                          re-anchor, not a dead-reckon advance); on
 *                          `Drive.hasFault()` enters `Fault::DRIVE_FAULT`.
 *  - `WALL_STOP`         — wall-zero reset interlude, phase 1 (PR2, AC9). Entered
 *                          from `PLAN_NEXT_HOP` when the just-arrived node is a
 *                          `Routes::NodeType::RESET` node (the reset hook, the
 *                          arrival analogue of the `RETURN_TO_NODE` prologue).
 *                          Dispatches one latched `Drive.wallStop(...)` REVERSE
 *                          push (rear into the wall — only the rear may take wall
 *                          load): the primitive's SQUARE phase first turns to the
 *                          reset record's `facing` (the PARKED heading, front AWAY
 *                          from the wall), then presses the rear straight in RAW mode
 *                          with back-off 0. Waits for `Drive.isIdle()` before
 *                          transitioning to `REANCHOR`.
 *  - `REANCHOR`          — wall-zero reset interlude, phase 2 (PR2, AC9–AC11).
 *                          Surfaces a `Drive.wallStopTimedOut()` advisory (AC10 —
 *                          assume-at-wall and continue, NOT a halt), then calls
 *                          `Localization.reanchorAtWall(axis, wall_coord, facing)`.
 *                          On `false` (strict wall-normal check rejected the
 *                          squaring, only possible under
 *                          `Config::WALL_NORMAL_STRICT_ENABLED`) it raises
 *                          `Fault::WALL_NOT_SQUARE` (AC11). On `true` it transitions
 *                          to `WALL_DEPART` to pull off the wall before any planning.
 *  - `WALL_DEPART`       — wall-zero reset interlude, phase 3. The chassis is pressed
 *                          against the wall (belief is the axle, a contact-offset
 *                          short of it), so before `PLAN_NEXT_HOP` can dispatch a
 *                          pivot it drives FORWARD off the wall back onto the node
 *                          (`standoff - WALL_CONTACT_OFFSET_CM`, GYRO holding the
 *                          squared heading), then `clearOffset()` (belief == node)
 *                          and continues to `PLAN_NEXT_HOP`. The reset latch stays
 *                          set, so the interlude is not re-entered.
 *  - `FAULT`             — latched fault; cleared via `clearFault()` or
 *                          `abort()`.
 *
 * Per-edge motion-mode resolution:
 *  - Each `Routes::EdgeInfo` carries per-category override fields
 *    (`speed_override`, `blind_mode_override`, `direction_override`,
 *    `stop_at_line_override`). `DEFAULT` means "inherit
 *    `Routes::PIPELINE_CONFIG.<category>`." The dispatcher resolves DEFAULT →
 *    concrete at edge-dispatch time and chains the appropriate builder setters.
 *    Edges are LINE or BLIND only (issue #64); both edge types take their cruise
 *    SPEED from the same `speed_override` tier (a LINE edge into a projection
 *    DESTINATION node included). The pre-edge turn wheel is no longer an `EdgeInfo`
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

#include "../../Config.h"
#include "../drive/ChainSpeedPolicy.h"
#include "../nav/FieldMap.h"
#include "../nav/NavTypes.h"
#include "../nav/routes.h"

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
class PathTrackerGoToBuilder
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
    /**
     * @brief Per-edge FSM state. See class doxygen for the full lifecycle.
     *
     * @note Public (issue #108, AC12) so the parity driver can dispatch each tick
     *       off the observable state the FSM is actually waiting on (read via
     *       `state()`). It is a read-only introspection surface — there is no
     *       setter and no behavioral accessor; the FSM remains the sole writer.
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
        WALL_STOP,
        REANCHOR,
        WALL_DEPART,
        SETTLE,
        FAULT
    };

    enum class Fault : uint8_t
    {
        NONE                     = 0,
        NOT_ZEROED               = 1,
        UNREACHABLE              = 2,
        DRIVE_FAULT              = 3,
        BAD_NODE                 = 4,
        DRAIN_TIMEOUT            = 5,
        REVERSE_UNAVAILABLE      = 6,
        OFF_NODE                 = 7, ///< Operator-offset cap exceeded; off-map gate (Config::OFF_MAP_GATE == FAULT) refused to plan return-to-node.
        WALL_NOT_SQUARE          = 8, ///< Wall-zero reset rejected: the strict wall-normal check (Config::WALL_NORMAL_STRICT_ENABLED) found the chassis too far off the wall normal to trust the re-square (AC11).
        PIVOT_TRIM_UNDERFLOW     = 9, ///< A STOP_AT_LINE seek into a pivot needs δ>offset (sharper than ~106°); a forward-only seek cannot land δ-short, so the dispatch faults instead of clamping off-target (ADR-044 D4; the validator catches every case but a pure call-site setStopAtLine(ON)).
        LINE_LOST                = 10, ///< The LineFollower reported a continuous lost line for at least `Config::LINE_LOST_FAULT_MS` while `LINE_FOLLOW`-ing outside a deliberate dead-reckon window (issue #146). Bounded fault ESCALATION, not the deferred ADR-009 sweep/reacquire/resume recovery.
        PIVOT_DIRECTION_MISMATCH = 11  ///< A pivot's departure hop (v→w) carried a BINDING direction override that contradicts the pivot's approach direction (u→v); force-SAME cannot reconcile the two, so the dispatch faults instead of running a mis-trimmed maneuver (issue #54, ADR-071).
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

    /**
     * @brief Read the current per-edge FSM state (issue #108, AC12).
     *
     * @return The live `State` the FSM is in this tick.
     * @note Read-only introspection for the parity driver — no mutation. The
     *       driver dispatches each tick off the observable the FSM is actually
     *       waiting on; it never re-derives the hop type from the graph.
     */
    State state() const { return _state; }

    /**
     * @brief Read the resolved body-target heading (world frame, degrees) for the
     *        current hop / return prologue (issue #108, AC12).
     *
     * @return `_resolvedBodyHeading` — the heading the chassis BODY must face for
     *         the in-flight hop (the `Drive.turnToYaw` target). For a REAR hop this
     *         is already rotated 180 deg from the edge heading.
     * @note Read-only introspection: this is the only public surface exposing the
     *       pre-edge turn target (`Drive`'s turn target is private), so the parity
     *       driver can park yaw at it and assert it against the sim's per-hop body
     *       heading. No mutation.
     */
    float resolvedBodyHeadingDeg() const { return _resolvedBodyHeading; }

    /**
     * @brief PURE ADR-023 collinearity + same-direction lookahead predicate (ADR-059).
     *
     * The single source of truth for "may the chassis continue THROUGH the next node
     * without a pre-edge turn?" — shared by all three lookahead gates: the
     * intersection-gated `line–line–line` pass-through (`LINE_FOLLOW`), the
     * distance-gated `line–proj–line` pass-through (`_evaluateProjectionPassThrough`),
     * and the ADR-059 blind-chain walk-ahead (`_chainTryStart`). Unlike the historical
     * `_passThroughHeadingOk` (now a thin live-state adapter over this), it reads NO
     * FSM member state, so the blind-chain walk can call it repeatedly for multi-hop
     * lookahead with synthesized inbound headings. The Python sim mirrors THIS function
     * (one predicate, two languages — ADR-050).
     *
     * The test: re-resolve the outbound hop's direction with @p inboundBodyHeadingDeg
     * as the current heading (`resolveDirection`, DirectionPolicy.h), rotate the
     * outbound edge heading 180° for a REAR result, wrap-normalize the delta against
     * the inbound BODY heading, and require BOTH (1) `|delta| <=
     * Config::NAV_TURN_TOLERANCE_DEG` (collinear) AND (2) the resolved outbound
     * direction equals @p inboundDirection (a FRONT<->REAR flip is never a
     * continuation).
     *
     * @param inboundBodyHeadingDeg  World-frame BODY heading (deg) the chassis crosses
     *                               the mid node on (REAR already rotated 180°).
     * @param inboundDirection       Resolved direction of the inbound hop (FRONT/REAR;
     *                               never AUTO).
     * @param outboundEdge           The outbound (mid -> next) edge under evaluation.
     * @param outboundIsLastHop      Whether the outbound hop lands on the goal node
     *                               (binds the builder ARRIVAL override in
     *                               `resolveDirection`).
     * @param activeArrivalDirection The goal's cached `setArrivalDirection` override.
     * @param activeRouteDirection   The goal's cached `setRouteDirection` override.
     * @return `true` iff the outbound hop is collinear with the inbound one within
     *         tolerance AND same-direction — i.e. eligible to continue without a stop.
     *
     * @note Pure: reads only its arguments plus compile-time constants
     *       (`Config::NAV_TURN_TOLERANCE_DEG`, `Config::BIDIRECTIONAL_AUTO_ENABLED`,
     *       `Config::LINE_BOARD_COUNT`, `Routes::PIPELINE_CONFIG.direction`). Public
     *       static so the host tests and the parity harness can drive it directly.
     */
    static bool passThroughHeadingEligible(float                   inboundBodyHeadingDeg,
                                           Direction               inboundDirection,
                                           const Routes::EdgeInfo& outboundEdge,
                                           bool                    outboundIsLastHop,
                                           Direction               activeArrivalDirection,
                                           Direction               activeRouteDirection);

    PathTrackerGoToBuilder goTo(uint8_t goalId);
    PathTrackerGoToBuilder goTo(const char* goalName);

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
     * Latches the in-flight return state (`_returnUsesLineSeek`, `_returnDistCm`,
     * `_resolvedBodyHeading = returnHeading`, `_resolvedDirection = FRONT`, `_returnActive`),
     * then dispatches the pre-edge turn through the SAME `_dispatchTurnIfNeeded(...)` every hop
     * uses. When a turn fires the FSM enters `TURNING` and `_dispatchReturnMoveAfterTurn()`
     * runs once it completes; when the chassis is already aligned the move dispatches
     * immediately. The actual move lives in `_dispatchReturnMoveAfterTurn()` and is chosen by
     * `_returnUsesLineSeek` (a LINE anchor whose return distance is at least the front sensor's
     * axle lead `Config::LINE_SENSOR_AXLE_OFFSET_CM`):
     *  - line seek : a forward `Drive.stopAtLine(_returnDistCm)` GYRO seek that lands the wheel
     *                AXLE on the node (ADR-027); `RETURN_TO_NODE` re-anchors the SAME node via
     *                `Localization.onIntersection(...)` on success.
     *  - dead-reckon : a forward `Drive.driveDist(_returnDistCm)` (a blind anchor, OR a short
     *                LINE return whose front sensor already sits past the node line, #14);
     *                `RETURN_TO_NODE` re-anchors via `Localization.clearOffset()` (node unchanged).
     *
     * Routing the return through the shared pre-turn is the structural fix for the
     * LINE-return-without-turn bug: the pre-edge turn is no longer inlined per node kind, so
     * it cannot be omitted on one branch. Every Drive call is emitted as a single
     * discarded-temporary `.async()` statement (RAII builder rule — never split a
     * builder chain across statements / a local variable).
     *
     * @param off The accumulated operator offset (world-frame cm + heading delta).
     * @param mag The offset translation magnitude `sqrt(off.x^2 + off.y^2)`.
     */
    void _dispatchReturnMove(const Offset2D& off, float mag);

    /**
     * @brief Dispatch the return seek/drive AFTER the shared pre-turn has aligned the chassis
     *        (PR1b, issue #36, #6 refactor).
     *
     * Called either directly by `_dispatchReturnMove(...)` when no pre-turn was needed, or
     * from the `TURNING` state once the return's pre-edge `Drive.turnToYaw` completes (the
     * `_returnActive` branch). A line-seek return (`_returnUsesLineSeek`) re-acquires the line
     * with a forward `Drive.stopAtLine(_returnDistCm)` GYRO seek; a dead-reckon return drives a
     * forward `Drive.driveDist(_returnDistCm)`. Both enter `RETURN_TO_NODE`, which polls the
     * outcome. Reads `_returnUsesLineSeek`, `_returnDistCm`, and `_resolvedBodyHeading` (all
     * latched by `_dispatchReturnMove`). Every Drive call is one discarded-temporary `.async()`.
     */
    void _dispatchReturnMoveAfterTurn();

    /**
     * @brief Wall-zero reset hook: if the just-arrived node carries a reset record,
     *        enter the wall-stop interlude before planning the next hop (PR2, AC9).
     *
     * Called at the TOP of `_onPlanNextHop()` (the single arrival chokepoint — every
     * hop's success path advances `Localization` and routes to `PLAN_NEXT_HOP`). When
     * `FieldMap::getNodeType(cur) == Routes::NodeType::RESET` AND the per-arrival latch
     * `_resetNode` does not already name `cur`, this dispatches the wall-push via
     * `_dispatchWallStop(...)` and transitions to `WALL_STOP`. It is the arrival-side
     * analogue of the `RETURN_TO_NODE` prologue that `DEQUEUE` inserts before the first
     * hop: the reset interlude inserts before the next hop / before the goal-reached
     * decision, then hands control back to the unchanged hop loop.
     *
     * @param cur The just-arrived node id (`Localization.getCurrentNode()`).
     * @return `true` if the reset interlude was entered (caller must `return`
     *         immediately — `_state` is now `WALL_STOP`); `false` to continue normal
     *         planning (non-reset node, or the latch shows this reset already ran). The
     *         `false` path is a true no-op for non-reset nodes (AC5-style "no change
     *         when not applicable").
     */
    bool _maybeEnterReset(uint8_t cur);

    /**
     * @brief Dispatch the latched `Drive.wallStop(...)` push and enter `WALL_STOP` (PR2, AC9).
     *
     * Reads the reset record via `FieldMap::getResetInfo(node)` for `(axis, wall_coord,
     * facing)`. Computes the advisory push distance as the absolute gap between the
     * node's coordinate on `axis` and `wall_coord`
     * (`fabs(wall_coord - node_coord[axis])`) — advisory only, since `wallStop` is
     * stall/timeout-terminated (T14), not distance-terminated. `wallStop` first SQUARES
     * (turns in place to `facing`, the PARKED heading — front away from the wall, so the
     * rear faces it) then presses the REAR straight into the wall in `DriveMode::RAW`
     * (open-loop + motor bias) with back-off 0, latching `_resetNode = node` so the push
     * fires exactly once. Only the rear may take wall load — a front press would break the
     * mechanism.
     *
     * The square-up corrects the arrival-heading error up front, so a misaligned arrival
     * presses square instead of arcing into the wall (#1). RAW — not GYRO — for the push:
     * once squared, a heading-hold PID would only fight the wall contact as one wheel
     * stalls first; an open-loop straight press with motor bias stalls cleanly. A residual
     * gross misalignment is still caught by the strict wall-normal check in `REANCHOR`
     * (under `Config::WALL_NORMAL_STRICT_ENABLED`).
     *
     * RAII builder rule: the whole `Drive.wallStop(...)....async()` chain is ONE
     * discarded-temporary statement — never split across statements / a local variable.
     *
     * @param node The reset node id (already confirmed `NodeType::RESET` by the caller).
     */
    void _dispatchWallStop(uint8_t node);

    /**
     * @brief Line-arrival heading reset hook (ADR-052): re-bind heading for free on a
     *        collinear pass-through arrival — no wall, no maneuver.
     *
     * Called from `_onPlanNextHop()` immediately after the wall `_maybeEnterReset` block and
     * before the destination check — the one chokepoint every LINE arrival funnels through.
     * Firing here (before the next hop's direction is resolved via `getHeadingDeg()`) is
     * intentional: the next hop is planned against the freshly-corrected heading. Fires ONLY
     * when the just-completed arrival was a collinear pass-through (`_passThrough`), gated by
     * two independent global knobs:
     *   - a `NodeType::PROJECTION` pass-through binds under
     *     `Config::LINE_HEADING_RESET_ON_PROJECTION` (default ON — a clean sample, the
     *     line-follow PID never freezes at a marker-less projection node),
     *   - a `NodeType::LINE` (physical junction) pass-through binds under
     *     `Config::LINE_HEADING_RESET_ON_JUNCTION` (default OFF — the junction PID-freeze
     *     window can inject a small heading wiggle).
     * `NodeType::BLIND` / `NodeType::RESET` are a defensive SILENT no-op: per ADR-048 no LINE
     * edge is incident to a RESET node and `_passThrough` is latched only on a LINE-edge
     * arrival, so a pass-through into BLIND/RESET is unreachable on any validator-passed field.
     * The node-type switch enumerates all four `Routes::NodeType`s with NO `default`, so a
     * future enum member trips `-Wswitch`.
     *
     * On a knob-ON eligible node it calls `Localization.reanchorHeading(incomingBodyHeadingDeg,
     * &delta)`: on a successful bind it emits a Serial fire log, one fixed
     * `EventTrace.pushEvent(...)` (wall-path style), and — under `PT_PARITY_TRACE` — a
     * `heading_reset` parity record; on a soft-skip (failed IMU read or strict-gate rejection)
     * it emits a loud Serial skip log and the run continues (no fault). NEVER changes `_state`
     * and NEVER touches `Localization::_offset`. Knob-OFF and the BLIND/RESET branch are
     * entirely SILENT — no IMU read, no log, no parity record.
     *
     * @param cur                    The just-arrived node id (`Localization.getCurrentNode()`).
     * @param firstPlan              The captured `_planIsFirstAfterDequeue`; a first plan is
     *                               skipped (mirrors the wall hook's `!firstPlan` gate —
     *                               `_passThrough` cannot be true on a first plan anyway).
     * @param incomingBodyHeadingDeg The world-frame body heading the chassis JUST arrived on,
     *                               captured from `_resolvedBodyHeading` at the TOP of
     *                               `_onPlanNextHop()` before the next hop overwrites it
     *                               (`:1887`). Passed explicitly so the bind target never
     *                               depends on member-write ordering. Already body-corrected
     *                               (a REAR arrival is edge heading + 180).
     */
    void _maybeHeadingReset(uint8_t cur, bool firstPlan, float incomingBodyHeadingDeg);

    void _onPlanNextHop();

    /**
     * @brief Dispatch a pre-edge yaw turn toward @p bodyHeadingWorld, shared by normal hops
     *        and the return-to-node prologue (issue #36 #6 refactor).
     *
     * Computes the wrap-normalized delta between the live heading and @p bodyHeadingWorld; if
     * it is within `Config::NAV_TURN_TOLERANCE_DEG` NO turn is dispatched (the caller proceeds
     * straight to the move). Otherwise it disables a live LineFollower (the wheels go to
     * `Drive.turnToYaw` alone — two writers would fight) and clears any pass-through
     * eligibility the LINE_FOLLOW peek latched (a real turn means the chassis stops and the
     * next move must soft-start), then dispatches `Drive.turnToYaw(...).setMode(turnModeByte)`
     * `.async()`. Centralising the turn here makes the pre-edge turn structurally
     * unskippable — the return prologue and every hop go through the SAME decision.
     *
     * @param bodyHeadingWorld The world-frame body-target heading (deg) the chassis must face.
     * @param turnModeByte     The wheel-pattern byte for `TurnToYawBuilder::setMode`
     *                         (pivot wheel for a pivot hop, else the pipeline default).
     * @return `true` if a turn was dispatched (caller enters `TURNING`); `false` if the
     *         chassis is already aligned (caller dispatches the move immediately).
     */
    bool _dispatchTurnIfNeeded(float bodyHeadingWorld, uint8_t turnModeByte);

    void _dispatchEdgeAfterTurn();

    /// @brief Where the FSM resumes when the SETTLE dwell elapses.
    ///
    /// Defined here — BEFORE `_enterSettle(SettleResume)` and the `_settleResume`
    /// member that consume it — because a member function's parameter type must be a
    /// COMPLETE type at its point of declaration. MSVC rejects a forward use (C2061);
    /// g++ is lenient, so the CI (g++) host-tests job would NOT catch a misordering.
    enum class SettleResume : uint8_t
    {
        PLAN,            // -> State::PLAN_NEXT_HOP (post-stop, pre-maneuver / goal arrival)
        EDGE_AFTER_TURN  // -> _dispatchEdgeAfterTurn() (post-cold-turn, pre-move)
    };

    /**
     * @brief Enter the non-blocking post-stop settle dwell (ADR-061).
     *
     * Stamps `_settleDeadlineMs = millis() + Config::HOP_SETTLE_MS`, latches @p resume, and
     * transitions the FSM into `State::SETTLE`. The dwell is a `millis()`-gated hold — it
     * issues NO Drive command (Drive stays idle/braked), so it is self-bounded by
     * `_settleDeadlineMs` and cannot hang the cooperative tick (iron rule 1).
     *
     * Called only where the Settle-fire matrix says "yes": each of the four braked arrival
     * states (via `_settleOrPlanAfterArrival`) with `SettleResume::PLAN`, and the cold
     * pre-edge turn on a normal hop with `SettleResume::EDGE_AFTER_TURN`. The caller is
     * responsible for advancing `Localization` FIRST (order is load-bearing — settling before
     * the node stamp would delay `getCurrentNode()` and every observer by the whole dwell).
     *
     * @param resume Where the `SETTLE` state routes when the deadline elapses:
     *               `PLAN` -> `State::PLAN_NEXT_HOP`, `EDGE_AFTER_TURN` ->
     *               `_dispatchEdgeAfterTurn()`.
     */
    void _enterSettle(SettleResume resume);

    /**
     * @brief Shared dwell-1 decision for a braked arrival (ADR-061).
     *
     * Applied by the four braked arrival states (`LINE_OVERSHOOT` non-pass, `STOP_AT_LINE`,
     * `PROJECTION_FOLLOW` stop, non-chain `BLIND_FORWARD`) AFTER they have advanced
     * `Localization` for @p arrivedNode. Enters `State::SETTLE` (resume `PLAN`) when
     * `Config::HOP_SETTLE_MS > 0` AND @p arrivedNode is not a `Routes::NodeType::RESET` node
     * (the wall interlude owns the RESET boundary); otherwise transitions straight to
     * `State::PLAN_NEXT_HOP` exactly as the pre-feature code did. Centralising the decision
     * keeps the four arrival sites from drifting.
     *
     * @param arrivedNode The just-stamped arrival node (`_nextHop` ==
     *                    `Localization.getCurrentNode()` after the stamp).
     */
    void _settleOrPlanAfterArrival(uint8_t arrivedNode);

    /**
     * @brief One-hop lookahead: the hop AFTER `_nextHop` toward `_destinationNode`.
     *
     * Shared guard for the FSM site that peeks the next-next hop: the `LINE_FOLLOW`
     * pass-through decision in `update()`. Centralises the three guards it needs so
     * they cannot drift: the not-final guard (`_nextHop` is the destination), the
     * `NODE_NONE` guard (no route from `_nextHop` to the goal), and the self-loop
     * guard (the lookahead returns `_nextHop` itself).
     *
     * @return The next-next node id, or `FieldMap::NODE_NONE` when any guard fails
     *         (there is no valid next-next hop to classify).
     *
     * @note Issue #64 retired the OLD second caller (the BLIND auto-rule projection
     *       classification): the auto-seek decision is now a LOCAL read of the
     *       CURRENT hop's destination node (`isProjectionNode(_nextHop)`), with no
     *       next-next peek. T19 re-added a routing-lookahead caller —
     *       `_evaluateProjectionPassThrough()` — so this helper again has two
     *       callers sharing ONE guard set: the intersection-gated `LINE_FOLLOW`
     *       pass-through and the distance-gated `line–proj–line` pass-through. Both
     *       conform to the same three guards, kept intact and self-contained here.
     */
    uint8_t _peekNextNextHop() const;

    /**
     * @brief Shared ADR-023 pass-through heading test for a mid→next (B→C) edge.
     *
     * The collinearity + same-direction half of BOTH pass-through gates: the
     * intersection-gated `line(A)–line(B)–line(C)` collapse in `LINE_FOLLOW` and the
     * distance-gated `line(A)–projection(B)–line(C)` collapse in
     * `_evaluateProjectionPassThrough()`. Both require the SAME two conditions across
     * the mid node, so the math lives here once (mirroring the Python sim's single
     * `_should_pass_through()`) rather than verbatim at both sites where a future edit
     * to the heading rule could update one and silently miss the other.
     *
     * Compares BODY headings: the chassis arrives facing `_resolvedBodyHeading`, so the
     * B→C direction is re-resolved with that as the current heading, the outbound edge
     * heading is rotated 180° for a REAR result, and the wrap-normalized delta is tested
     * against `Config::NAV_TURN_TOLERANCE_DEG`. The resolved B→C direction must also
     * equal `_resolvedDirection` — a FRONT↔REAR flip would force a mid-flight
     * LineFollower board swap, never a pass-through.
     *
     * @param outbound The mid→next (B→C) edge under evaluation.
     * @param next     The next-next node id (C), for the isLastHop arrival-direction flag
     *                 (`next == _destinationNode`).
     * @return `true` iff B→C is collinear with A→B within tolerance AND same-direction.
     *
     * @note ADR-059: this is now a thin LIVE-STATE ADAPTER over the PURE public
     *       `passThroughHeadingEligible(...)` — the heading/direction math lives only
     *       there, so the two pass-through gates and the blind-chain walk-ahead share
     *       one predicate and cannot drift.
     */
    bool _passThroughHeadingOk(const Routes::EdgeInfo& outbound, uint8_t next) const;

    /**
     * @brief `line(A)–projection(B)–line(C)` pass-through eligibility for the
     *        in-flight LINE→projection follow (T19, issue #64).
     *
     * Called at LINE→projection arm time in `_dispatchEdgeAfterTurn` (B == `_nextHop`,
     * `_resolvedBodyHeading` / `_resolvedDirection` already set for the A→B hop). It
     * is the ROUTING-LOOKAHEAD gate that decides whether the distance-gated follow
     * into projection node B should collapse the triple into one continuous A→C
     * follow (dead-reckon advance stamped at B, re-anchor at C) instead of stopping
     * at B. ALL of the following must hold (any failure ⇒ normal stop-at-B per-hop):
     *
     *   1. `Config::PATH_PASS_THROUGH_ENABLED` (leads the check so the NEXT_HOP peek
     *      is skipped entirely when pass-through is compiled out).
     *   2. B is a NON-goal projection node: `_peekNextNextHop()` returns a valid C
     *      (its not-final guard rejects `goTo(B)`, where B is the destination — a
     *      `goTo(B)` correctly stops at B).
     *   3. The outbound edge B→C is a LINE edge (`getEdgeType(B, C) == LINE`). The
     *      inbound A→B is LINE by construction (we are in the LINE→projection path).
     *   4. C is a LINE (junction) node: `isAtIntersection()` DOES fire at C, so the
     *      B→C hop re-anchors normally. A projection or blind C is rejected.
     *   5. A→B→C are collinear AND same-direction: the B→C body heading is within
     *      `Config::NAV_TURN_TOLERANCE_DEG` of the A→B body heading (the SAME ADR-023
     *      heading check the intersection-gated pass-through uses) and the resolved
     *      B→C direction equals the A→B direction (a FRONT↔REAR flip would force a
     *      mid-flight board swap — never a pass-through).
     *
     * @return `true` iff the triple qualifies for the continuous pass-through.
     *
     * @note Routing lookahead ONLY: this does NOT change which gate fires the state
     *       transition — that is always the A→B DISTANCE gate (a projection node has
     *       no junction marker, so `isAtIntersection()` never fires at B). It only
     *       decides whether the distance-gate trip stops at B or stamps the advance
     *       and continues. See the `PROJECTION_FOLLOW` doxygen and ADR-023.
     */
    bool _evaluateProjectionPassThrough() const;

    /**
     * @brief `line(A)–line(B)–line(C)` intersection-gated pass-through eligibility for the
     *        LINE→line (junction) hop being dispatched (F3 — evaluate-once hoist).
     *
     * The dispatch-time twin of the peek the `LINE_FOLLOW` state historically ran at
     * intersection-detect. Called from `_dispatchEdgeAfterTurn`'s
     * `LINE_FOLLOW_INTERSECTION` arm (B == `_nextHop`, `_resolvedBodyHeading` /
     * `_resolvedDirection` already resolved for the A→B hop). Peeks the hop AFTER
     * `_nextHop` toward `_destinationNode` (`_peekNextNextHop`) and applies the SAME
     * three-part test the detection site used: (1) `Config::PATH_PASS_THROUGH_ENABLED`
     * leads so the NEXT_HOP lookup is skipped when pass-through is compiled out, (2) the
     * outbound edge B→C exists (`type != NONE` — a BLIND B→C is a valid pass-through
     * target, unlike the projection collapse which requires a LINE flank), and (3) the
     * shared ADR-023 `_passThroughHeadingOk(outbound, C)` collinear + same-direction
     * predicate.
     *
     * Pure over headings/geometry (no live yaw, no FSM mutation) — identical inputs at
     * dispatch and at the later detection tick (the FSM never re-plans between the two),
     * so latching the result at dispatch and consuming it at detection is behaviour-
     * identical to the old recompute, while giving the approach-ramp arm at dispatch the
     * pass-through answer it needs BEFORE the leading sensor sees the marker.
     *
     * @return `true` iff the A→B→C triple is collinear + same-direction — i.e. the hop
     *         passes THROUGH `_nextHop` and must keep full cruise (approach ramp
     *         suppressed).
     *
     * @note Distinct from `_evaluateProjectionPassThrough()` (the distance-gated
     *       `line–proj–line` collapse), which additionally requires the outbound edge to
     *       be LINE and C to be a LINE junction node. This intersection-gated variant
     *       gates only on edge existence + heading, exactly as the old detection peek.
     */
    bool _evaluateLineFollowPassThrough() const;

    /**
     * @brief ADR-059 blind-chain walk-ahead: try to fold the BLIND dead-reckon hop being
     *        dispatched (hop 0: `cur -> _nextHop`) plus the collinear BLIND hops beyond it
     *        into ONE chained run, filling the fixed `_chain` run buffer.
     *
     * Called from `_dispatchEdgeAfterTurn`'s `BLIND_DEAD_RECKON` arm when no run is in
     * flight and `Config::PATH_BLIND_CHAIN_ENABLED` is set. Walks `NEXT_HOP` forward from
     * `_nextHop` toward `_destinationNode`, admitting each candidate hop only when ALL of
     * the eligibility guards hold (AC7 / Error Handling — any miss ends the chain BEFORE
     * the unsafe hop, which then dispatches as a normal braked hop):
     *
     *   - the walked node is not the goal and not a `NodeType::RESET` node (the wall-zero
     *     interlude requires a stop);
     *   - the outbound edge exists, is `EdgeType::BLIND`, resolves to a plain dead-reckon
     *     (a `resolveStopAtLine(...)` seek is chain-ineligible) under a PROFILED bundle
     *     (`SYNC_TSHAPE` / `GYRO_TSHAPE` — `RAW_FLAT` has no ramps to chain);
     *   - no `Routes::PivotTurn` latches at the joint or onto the hop's destination
     *     (defensive — a validator-passed pivot is never collinear);
     *   - the hop is collinear + same-direction per the PURE
     *     `passThroughHeadingEligible(...)` predicate (a direction flip or a bend ends
     *     the chain);
     *   - the hop is `>= Config::TSHAPE_MIN_MOVE_CM` (below it the Drive builder silently
     *     degrades the profile);
     *   - the run stays within the caps: `<= Config::MAX_CHAIN_HOPS` hops and a summed
     *     commanded distance `<= Config::PATH_BLIND_CHAIN_MAX_CM`. A CAP truncation of an
     *     otherwise-eligible hop is LOGGED over Serial (never silent); the ineligibility
     *     guards above are ordinary chain ends and stay quiet.
     *
     * A walk yielding fewer than 2 hops is NOT a chain — `_chain` stays inactive and the
     * caller takes the ordinary single-hop path (AC12: non-chain hops are byte-identical
     * to pre-ADR-059). Otherwise the per-hop `(startPwm, endPwm, brake)` plan is resolved
     * by the pure two-pass `resolveChainSpeeds(...)` (ChainSpeedPolicy.h) with @p entryPwm
     * as the seed and the run is activated (`_chain.hopIndex = 0`).
     *
     * @param cur        The hop-0 departure node (`Localization.getCurrentNode()`).
     * @param reverse    Hop 0's resolved `decision.reverse` — latched into the run so a
     *                   mid-run direction flip is detected as a divergence.
     * @param hop0Mode   Hop 0's resolved motion bundle (`decision.driveMode`).
     * @param hop0Speed  Hop 0's resolved `SpeedTier` (`_resolveSpeed(edge)`) — latched into
     *                   the run so a later hop of a DIFFERENT tier ends the chain (the run
     *                   shares one cruise; issue #97). SPEED only: a same-tier SYNC<->GYRO
     *                   mode mix still chains.
     * @param hop0DistCm Hop 0's COMMANDED distance (`decision.commandedDistanceCm` — the
     *                   pivot-trimmed value actually dispatched, so the speed plan is
     *                   resolved over the distances really driven).
     * @param cruisePwm  The signed run cruise (±the hop's resolved `SpeedTier` PWM by
     *                   direction; NORMAL == the former `Config::SPEED_NORMAL`).
     * @param entryPwm   The signed PWM the chassis is physically at when the run begins:
     *                   the ADR-023 captured LineFollower cruise after a LINE/projection
     *                   pass-through, else ±`Config::TSHAPE_START_SPEED` (AC5 — seeding
     *                   the floor after a pass-through would re-dip and lurch).
     *
     * @note Also requires hop 0 itself to be profiled, `>= TSHAPE_MIN_MOVE_CM`, and NOT
     *       driving into an approaching pivot (`_approachingPivot` active means the
     *       chassis must stop δ short of `_nextHop` and pivot there).
     */
    void _chainTryStart(uint8_t cur, bool reverse, Routes::BlindMode hop0Mode,
                        Routes::SpeedTier hop0Speed, float hop0DistCm, int16_t cruisePwm,
                        int16_t entryPwm);

    /**
     * @brief Reset the ADR-059 blind-chain run buffer to the inactive state.
     *
     * Called at every point a run must not survive: boot (`begin`), `abort()`, a fault
     * (`_enterFault`), DEQUEUE of a fresh goal (defensive — a run never spans goals), a
     * real pre-edge turn firing mid-run (`_dispatchTurnIfNeeded` — the chassis stops, so
     * the chained speed plan is void), a divergent dispatch (defensive), and the run's
     * own FINAL-hop dispatch (the ordinary braked stop completes the run).
     */
    void _chainReset();

    /**
     * @brief Compute the effective dead-reckon distance for a DISTANCE-terminated
     *        hop, applying the per-turn pivot trims (ADR-036).
     *
     * Called only on distance-terminated dispatches (BLIND dead-reckon
     * `driveDist`, and a LINE edge into a projection DESTINATION node — the
     * distance-gated follow). Subtracts the two RESOLVED pivot
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
     * @brief Resolve the per-edge lateral LINE-follow MODE for the given edge (issue #156).
     *
     * The runtime analogue of the file-local `_resolveSpeed` mapping, but with NO mission-wide
     * pipeline default: `Routes::LineFollowMode::DEFAULT` (absence of an authored override) always
     * resolves to `MIDDLE` — centered follow, offset 0 — since there is no `PIPELINE_CONFIG`
     * follow-offset knob (ADR-069). A concrete `MIDDLE`/`LEFT`/`RIGHT` passes through unchanged.
     *
     * The resolved mode is threaded DIRECTLY into the `LINE_FOLLOW_INTERSECTION` and
     * `PROJECTION_FOLLOW` builder chains via
     * `.setLineFollowOffset(sign * lineFollowModeToSetpoint(mode))`, where
     * `sign = decision.reverse ? -1 : +1` composes the reverse-hop flip
     * (`WROMath.h::lineFollowModeToSetpoint` owns the forward-frame magnitude). It is
     * deliberately NOT a field of `EdgeDispatchQuery`/`EdgeDispatchDecision`, whose frozen
     * 7-field decision + truth table stay untouched. The parity stream records this resolved
     * mode (PRE-reverse-sign) as the `follow_mode` wire field.
     *
     * @param edge The directed edge being dispatched (`getEdge(cur, _nextHop)`).
     * @return The resolved concrete mode (`MIDDLE`, `LEFT`, or `RIGHT`; never `DEFAULT`).
     */
    Routes::LineFollowMode _resolveLineFollowMode(const Routes::EdgeInfo& edge);

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

    /// @brief Whether the IN-FLIGHT return re-acquires via the forward line seek (PR1b).
    ///
    /// Set by `_dispatchReturnMove(...)` when entering `RETURN_TO_NODE`: `true` only when the
    /// anchor is a LINE node AND the return distance is at least the front sensor's axle lead
    /// (`Config::LINE_SENSOR_AXLE_OFFSET_CM`) — a `Drive.stopAtLine` seek re-acquires the line and
    /// re-anchors via `onIntersection`. It is `false` for a blind-only anchor AND for a SHORT LINE
    /// return whose front sensor already sits past the node line (#14): both dead-reckon a forward
    /// `Drive.driveDist` and re-anchor via `clearOffset`. Read only by `_dispatchReturnMoveAfterTurn`
    /// and the `RETURN_TO_NODE` state body. Meaningless outside `RETURN_TO_NODE`; defaults to `false`.
    bool _returnUsesLineSeek = false;

    /// @brief True while a return-to-node prologue is in flight (PR1b, issue #36, #6 refactor).
    ///
    /// Set by `_dispatchReturnMove(...)` when a return is dispatched; cleared by the
    /// `RETURN_TO_NODE` arrive state once the return move settles (and on begin/abort/fault).
    /// Read by the SHARED `TURNING` state to route the post-turn dispatch to
    /// `_dispatchReturnMoveAfterTurn()` (the return seek/drive) instead of
    /// `_dispatchEdgeAfterTurn()` (a normal hop). Because the return reuses the same pre-turn
    /// machinery as every hop (`_dispatchTurnIfNeeded`), the pre-edge turn can never be
    /// skipped on a return — the structural fix for the LINE-return-without-turn bug.
    bool _returnActive = false;

    /// @brief Clamped return-travel distance (cm) latched for the in-flight return prologue.
    ///
    /// Set by `_dispatchReturnMove(...)` (the offset magnitude clamped to
    /// `Config::RETURN_DISTANCE_CAP_CM`) and consumed by `_dispatchReturnMoveAfterTurn()` once
    /// the shared pre-turn has faced the chassis at the return heading. Meaningless outside a
    /// return; defaults to 0.
    float _returnDistCm = 0.0f;

    /// @brief Whether a continuous lost-line window is currently being timed (issue #146).
    ///
    /// Set the first `LINE_FOLLOW` tick `LineFollower.isLineLost()` reports a loss OUTSIDE a
    /// deliberate dead-reckon window (`_passThrough` / `_projPassThrough` clear); cleared the
    /// instant the line reacquires, the FSM leaves `LINE_FOLLOW` (state-exit), or the goal is
    /// aborted / faulted / re-begun. While set, `_lineLostSinceMs` holds the loss-start clock;
    /// the escalation to `Fault::LINE_LOST` fires once the elapsed time reaches
    /// `Config::LINE_LOST_FAULT_MS`. Meaningless outside a timed loss; defaults to `false`.
    bool _lineLostTiming = false;

    /// @brief `millis()` stamp of the first tick of the in-flight continuous loss (issue #146).
    ///
    /// Latched together with `_lineLostTiming` on the first sustained-loss tick and compared
    /// rollover-safe against `Config::LINE_LOST_FAULT_MS` on each subsequent lost tick
    /// (`(uint32_t)(millis() - _lineLostSinceMs) >= Config::LINE_LOST_FAULT_MS`). Only read while
    /// `_lineLostTiming` is set, so the reset to 0 on begin/abort/fault/reacquire is defensive.
    uint32_t _lineLostSinceMs = 0;

    /// @brief `millis()` deadline for the in-flight SETTLE dwell (ADR-061).
    ///
    /// Stamped `millis() + Config::HOP_SETTLE_MS` on SETTLE entry by `_enterSettle(...)`.
    /// Compared rollover-safe in the `SETTLE` state: `(int32_t)(millis() - _settleDeadlineMs)
    /// >= 0`. Meaningless outside SETTLE; the state itself is the real guard, so the reset to 0
    /// on begin/abort/fault is defensive only.
    uint32_t _settleDeadlineMs = 0;

    /// @brief Resume target latched for the in-flight SETTLE dwell (ADR-061).
    ///
    /// Set by `_enterSettle(...)`: `PLAN` for a braked arrival's pre-maneuver dwell (dwell-1,
    /// also covers goal arrival), `EDGE_AFTER_TURN` for a cold pre-edge turn's turn->move dwell
    /// (dwell-2). Read once by the `SETTLE` state to pick `PLAN_NEXT_HOP` vs
    /// `_dispatchEdgeAfterTurn()` when the deadline elapses.
    SettleResume _settleResume = SettleResume::PLAN;

    /// @brief Per-arrival latch guarding the wall-zero reset interlude (PR2, AC9).
    ///
    /// Holds the node id whose reset interlude is currently in flight (the one that
    /// dispatched `Drive.wallStop` in `WALL_STOP`), or `FieldMap::NODE_NONE` when no
    /// reset is active. The `PLAN_NEXT_HOP` reset hook only enters the interlude when
    /// the just-arrived node is a `Routes::NodeType::RESET` node AND `_resetNode` does
    /// NOT already equal it — so the one-shot `Drive.wallStop` fires exactly ONCE per
    /// arrival (mirrors the pivot-latch discipline: a re-entry or a second tick of
    /// `PLAN_NEXT_HOP` after `REANCHOR` success does not re-fire the push, because
    /// `_resetNode` is still set to that node). Set in `WALL_STOP` (on dispatch) and
    /// re-armed (cleared to `NODE_NONE`) on a fresh `DEQUEUE`, on `abort()`, and on
    /// `_enterFault(...)`. A successful `REANCHOR` deliberately LEAVES it set to the
    /// reset node so the immediate return to `PLAN_NEXT_HOP` does not re-trigger; it
    /// naturally re-arms once the FSM advances off that node (a later goal that
    /// re-visits the same reset node clears it at DEQUEUE first).
    uint8_t _resetNode = FieldMap::NODE_NONE;

    /// @brief True only on the FIRST `_onPlanNextHop()` of a goal — before any hop has run.
    ///
    /// The wall-zero reset interlude fires on ARRIVAL at a reset node, i.e. only AFTER a real
    /// hop has driven the chassis onto it. But `_onPlanNextHop()` also runs once at the very
    /// start of a goal (the first plan after `DEQUEUE`, possibly after the return-to-node
    /// prologue), when `cur` is still the start/current node and no hop has moved the chassis.
    /// Firing the interlude there would (a) wall-push at mission start if the START node is typed
    /// `reset`, and (b) wall-push on a zero-hop `goTo(R)` while already on `R` — both with zero
    /// prior motion, and NEITHER modelled by the sim (which only checks for a reset node AFTER a
    /// hop). This flag is set at `DEQUEUE` and cleared on the first `_onPlanNextHop()`, so the
    /// interlude is gated to genuine post-hop arrivals and firmware/sim stay in lockstep. The
    /// matching authoring guard (a START node may not be typed `reset`) lives in the validator.
    bool _planIsFirstAfterDequeue = false;

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

    /// @brief `line(A)–projection(B)–line(C)` pass-through eligibility for the
    ///        in-flight LINE→projection follow (T19, issue #64).
    ///
    /// Latched in `_dispatchEdgeAfterTurn`'s LINE→projection branch from
    /// `_evaluateProjectionPassThrough()` BEFORE the `PROJECTION_FOLLOW` distance
    /// gate is armed: when set, the gate is armed with `setBrakeOnGate(false)` so
    /// the distance-gate trip latches `isDistanceReached()` WITHOUT self-disabling
    /// or braking the follower, keeping the A→C follow continuous across the
    /// marker-less projection node B. Consumed by the `PROJECTION_FOLLOW` state on
    /// the gate trip: when true it stamps `onDeadReckonedAdvance(B)` mid-follow and
    /// continues line-following the B→C hop (re-using the already-enabled
    /// LineFollower fast-path via `_passThrough`), where `isAtIntersection()` fires
    /// at the real junction C; when false it stops at B (today's per-hop behavior).
    /// Reset on begin/abort/fault and consumed (set false) once the gate trips.
    /// Distinct from `_passThrough` (the intersection-gated LINE_OVERSHOOT flag):
    /// this one fires off the DISTANCE gate at a projection node, NEVER
    /// `Intersection.isAtIntersection()`. See ADR-023 and issue #64.
    bool _projPassThrough = false;

    /// @brief Dispatch-time pass-through latch for the in-flight LINE→line (junction) hop
    ///        (F3 — evaluate-once hoist for the anticipatory approach ramp).
    ///
    /// Set in `_dispatchEdgeAfterTurn`'s `LINE_FOLLOW_INTERSECTION` arm from
    /// `_evaluateLineFollowPassThrough()` — the collinear `line(A)–line(B)–line(C)` peek
    /// that historically ran at intersection-detect in the `LINE_FOLLOW` state. Hoisting
    /// it to dispatch lets the arm decide whether THIS hop TERMINATES at `_nextHop` (arm
    /// the approach ramp) or PASSES THROUGH it (keep full cruise — ramp suppressed), a
    /// decision the ramp needs BEFORE the leading sensor sees the marker. The `LINE_FOLLOW`
    /// state consumes this SAME latch into `_passThrough` at detection instead of
    /// recomputing, so the schedule armed at dispatch always agrees with the branch taken
    /// at detection (the peek is pure over headings/geometry and the FSM never re-plans
    /// between the two, so the value is identical). Reset on begin/abort/fault, mirroring
    /// `_projPassThrough`. Distinct from `_passThrough` (the intersection-detect flag
    /// consumed by `LINE_OVERSHOOT` and the next dispatch). See ADR-023.
    bool _lineFollowPassThrough = false;

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
    ///
    /// @note `dir` is the traversal direction resolved for the pivot's
    ///       APPROACH hop (u→v). Force-SAME uses it to require the departure
    ///       hop (v→w) to match, and to pick the flipped wheel + the
    ///       direction-resolved overshoot budget (issue #54, ADR-071).
    struct PivotRef
    {
        bool             active  = false;
        Routes::TurnMode wheel   = Routes::TurnMode::DEFAULT;
        float            trim_cm = 0.0f;
        Direction        dir     = Direction::AUTO;
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

    /// @brief ADR-059 blind-chain run buffer — FIXED-SIZE, no heap (Data Model).
    ///
    /// Filled once per run by `_chainTryStart` (the walk-ahead) and consumed hop-by-hop
    /// by `_dispatchEdgeAfterTurn`'s `BLIND_DEAD_RECKON` arm as each hop of the run
    /// re-enters the NORMAL per-hop FSM cycle (PLAN_NEXT_HOP -> dispatch ->
    /// BLIND_FORWARD -> stamp node). The buffer is the ONLY cross-hop state the feature
    /// adds, and it lives here in PathTracker — never in `LineFollower`/`Drive`
    /// (ADR-050/ADR-059 scope rule).
    ///
    ///  - `node[0..hopCount]`     — the walked node ids (`node[0]` = run start).
    ///  - `hopDistCm[k]`          — hop k's walk-time COMMANDED distance (hop 0 is the
    ///                              pivot-trimmed dispatch value; later hops carry the
    ///                              raw edge length — no pivots exist mid-run).
    ///  - `speed[k]`              — the `resolveChainSpeeds` plan: signed
    ///                              `(startPwm, endPwm)` + `brake` (true only on the
    ///                              final hop).
    ///  - `hopIndex`              — the hop most recently DISPATCHED (in flight).
    ///  - `reverse`               — hop 0's resolved direction, for divergence detection.
    ///  - `speedTier`             — hop 0's resolved `SpeedTier`; the walk-ahead admits a
    ///                              hop only when its resolved tier matches (issue #97), and
    ///                              the dispatch re-check drops the run on a divergent tier.
    ///  - `dispatchEncSum`        — `Drive.getEncL() + getEncR()` snapshotted at the
    ///                              in-flight hop's dispatch; with `dispatchCmdCm` it
    ///                              yields the MEASURED inter-hop boundary carry that is
    ///                              subtracted from the next hop's commanded distance
    ///                              (AC9 — measured, never an assumed constant).
    ///  - `dispatchCmdCm`         — the distance actually commanded for the in-flight
    ///                              hop (post carry-subtraction).
    ///
    /// `active == false` gates every read, so stale array contents are harmless. The
    /// run retires at its FINAL hop's dispatch (that hop is an ordinary braked stop); a
    /// fresh run may re-form on the far side (cap overflow is not a fault).
    struct ChainRun
    {
        bool              active         = false;
        bool              reverse        = false;
        Routes::SpeedTier speedTier      = Routes::SpeedTier::DEFAULT;
        uint8_t           hopCount       = 0;
        uint8_t           hopIndex       = 0;
        uint8_t           node[Config::MAX_CHAIN_HOPS + 1]  = {};
        float             hopDistCm[Config::MAX_CHAIN_HOPS] = {};
        ChainHopSpeed     speed[Config::MAX_CHAIN_HOPS]     = {};
        int32_t           dispatchEncSum = 0;
        float             dispatchCmdCm  = 0.0f;
    };
    ChainRun _chain;

    FaultInfo _fault;
    void (*_tickCallback)()        = nullptr;
    bool  _tickCallbackWarningEmitted = false;
};

extern PathTrackerClass PathTracker;

#endif    // PATHTRACKER_H
