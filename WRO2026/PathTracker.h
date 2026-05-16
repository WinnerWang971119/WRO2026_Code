/**
 * @file PathTracker.h
 * @brief Non-blocking PathTracker FSM + fluent goTo() builder for the WRO 2026 R4 runtime.
 * @author WRO 2026
 *
 * @details
 * `PathTrackerClass` consumes the precomputed `Routes::NEXT_HOP` table and dispatches one
 * edge at a time onto `Drive` and `LineFollower` to navigate the chassis from the current
 * node to a goal node. It owns:
 *
 *  - A FIFO queue of pending goal entries (`Config::PATH_QUEUE_CAPACITY` slots).
 *  - A per-edge state machine driven from `update()` (cooperative tick).
 *  - A fluent `goTo(node)` builder that, by default, sync-blocks at end-of-statement until
 *    every queued goal has been reached. Chain `.async()` on the builder to opt out so a
 *    higher-level mission scheduler can poll `isIdle()` instead.
 *  - A latched fault state (`Fault::NOT_ZEROED`, `UNREACHABLE`, `DRIVE_FAULT`, `BAD_NODE`)
 *    surfaced via `hasFault()` / `getFault()`. `clearFault()` re-arms the queue but does
 *    not restart motion.
 *
 * Two iron rules (mirroring `Drive`'s discipline):
 *
 *  1. **`update()` is non-blocking and never issues a sync verb.** Every Drive call from
 *     PathTracker is `.async()`. Every LineFollower call applies its state via the builder
 *     destructor (which mutates state synchronously but issues no spin). The only sync site
 *     in PathTracker is the goTo() builder destructor's `_drainBlocking()` — and that
 *     site sits OUTSIDE the cooperative-tick chain by construction.
 *  2. **Localization is the only writer of `_currentNode`.** PathTracker calls
 *     `Localization.onIntersection(...)` after a LINE edge or `onDeadReckonedAdvance(...)`
 *     after a PROJECTION/BLIND edge. PathTracker NEVER writes heading state — every turn
 *     dispatches `Drive.turnToYaw(...)` against the IMU as the source of truth.
 *
 * Heading model: float degrees in `[0, 360)`, CCW positive, `0 deg = +x` (East). PathTracker
 * computes a target world-frame heading from `Routes::EDGES`, converts to IMU frame via
 * `Localization::toImuYaw(...)`, and dispatches an absolute turn. Each turn re-anchors to
 * IMU truth so float drift across many turns does not accumulate.
 *
 * Lifecycle:
 *  - `setup()`: `Localization.begin()` then `PathTracker.begin()` (after Phase 1 modules);
 *    `PathTracker.setTickCallback(cooperativeTick)` once the cooperative tick exists.
 *  - `loop()`: `PathTracker.update()` is part of the cooperative tick chain — call it AFTER
 *    `Intersection.update()` so its read-and-clear edge consumption sees this tick's event.
 *  - Demo: BTN_UP rising-edge calls `Localization.zeroHeadingFromCurrent()` then chains
 *    `PathTracker.goTo("C"); PathTracker.goTo("G");`. Each goTo() blocks until done.
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
 * Sync-blocking by default: the destructor calls `_applyGoToBuilder(blocking=true)` which
 * enqueues the goal then drains the queue (via the registered cooperative tick callback)
 * until everything pending has completed. Chain `.async()` to opt out — the destructor
 * still enqueues, but returns immediately so the caller can `update()` and poll
 * `PathTrackerClass::isIdle()` themselves.
 *
 * Copy and move are deleted: the builder is a stack-temporary at the call site (RAII),
 * stored only via NRVO. `[[nodiscard]]` warns if the caller stores it without using it.
 */
class [[nodiscard]] PathTrackerGoToBuilder
{
public:
    /**
     * @brief Construct a builder bound to @p owner with the resolved goal id.
     *
     * @param owner   PathTracker singleton whose state is mutated on destruction.
     * @param goalId  Target node id; `FieldMap::NODE_NONE` indicates a name-lookup miss
     *                that the destructor will surface as `Fault::BAD_NODE`.
     */
    PathTrackerGoToBuilder(PathTrackerClass& owner, uint8_t goalId);

    PathTrackerGoToBuilder(const PathTrackerGoToBuilder&)            = delete;
    PathTrackerGoToBuilder& operator=(const PathTrackerGoToBuilder&) = delete;
    PathTrackerGoToBuilder(PathTrackerGoToBuilder&&)                 = delete;
    PathTrackerGoToBuilder& operator=(PathTrackerGoToBuilder&&)      = delete;

    /**
     * @brief Destructor — applies the staged goal exactly once (RAII auto-execute).
     *
     * Calls `PathTrackerClass::_applyGoToBuilder(_goalId, _blocking)`.
     */
    ~PathTrackerGoToBuilder();

    /**
     * @brief Opt out of the default sync-blocking semantics.
     *
     * After this is chained, the destructor enqueues the goal and returns immediately.
     * Caller is responsible for ticking `cooperativeTick()` and polling
     * `PathTrackerClass::isIdle()` to detect completion.
     *
     * @return `*this` for fluent chaining.
     */
    PathTrackerGoToBuilder& async();

private:
    PathTrackerClass& _owner;
    uint8_t           _goalId;
    bool              _blocking = true;
    bool              _executed = false;
};

/**
 * @brief Singleton state machine that dispatches one edge at a time to Drive/LineFollower.
 *
 * Memory footprint: queue (`PATH_QUEUE_CAPACITY * sizeof(GoalEntry)`) + ~32 bytes of FSM
 * state. Negligible on the R4's 32 KB SRAM.
 */
class PathTrackerClass
{
    friend class PathTrackerGoToBuilder;

public:
    /**
     * @brief Tagged classification for a latched PathTracker fault.
     */
    enum class Fault : uint8_t
    {
        NONE         = 0,    ///< No fault.
        NOT_ZEROED   = 1,    ///< goTo() called before `Localization::zeroHeadingFromCurrent()`.
        UNREACHABLE  = 2,    ///< `Routes::NEXT_HOP[from][goal] == NEXT_HOP_NONE`.
        DRIVE_FAULT  = 3,    ///< `Drive::hasFault()` observed during dispatch.
        BAD_NODE     = 4,    ///< Goal id out of range / name not found.
        DRAIN_TIMEOUT = 5    ///< `_drainBlocking()` exceeded its bounded budget.
    };

    /**
     * @brief Captured details of the last latched fault.
     */
    struct FaultInfo
    {
        Fault   kind     = Fault::NONE;
        uint8_t fromNode = FieldMap::NODE_NONE;
        uint8_t goalNode = FieldMap::NODE_NONE;
    };

    /**
     * @brief Reset the FSM, queue, and fault to their power-on state.
     *
     * Must be called from `setup()` AFTER `Localization.begin()`.
     *
     * @return `true` always.
     */
    bool begin();

    /**
     * @brief Cooperative tick — must be called every iteration of the main `loop()`.
     *
     * Non-blocking. Advances the per-edge state machine by at most one transition per
     * call; idempotent in `IDLE` / `FAULT`. See class-level docblock for the iron rules
     * the body honours.
     */
    void update();

    /**
     * @brief Register the cooperative-tick function the goTo() drain spin should call.
     *
     * `_drainBlocking()` must keep ALL cooperative subsystems alive during its spin —
     * otherwise `EventTrace` doesn't render, `LineFollower` doesn't poll the tracer,
     * `Drive` doesn't dispatch. Register a function that ticks every module (typically
     * `cooperativeTick()` defined in the sketch) once after `setup()`.
     *
     * @param cb  Pointer to the cooperative-tick function. Pass `nullptr` to clear.
     * @note If no callback is registered, `_drainBlocking()` falls back to ticking only
     *       `update()` and emits a one-time Serial warning at first use.
     * @warning The registered callback's call chain MUST NOT issue sync-blocking
     *          PathTracker `goTo(...)` calls (i.e. no nested goTo without `.async()`).
     */
    void setTickCallback(void (*cb)());

    /**
     * @brief `true` iff the queue is empty AND the FSM is in `IDLE` AND no fault is set.
     *
     * The right query for the auto-planner's "is it safe to issue the next goal?" check.
     */
    bool isIdle() const;

    /**
     * @brief Build a goal-traversal command bound to @p goalId.
     *
     * @param goalId  Target node id (`< Routes::N_NODES`). Out-of-range values surface
     *                as `Fault::BAD_NODE` from the builder destructor.
     * @return        Fluent builder; sync-blocking by default. Chain `.async()` to opt out.
     */
    [[nodiscard]] PathTrackerGoToBuilder goTo(uint8_t goalId);

    /**
     * @brief Convenience overload: resolve a goal node by name via `FieldMap::findNodeByName`.
     *
     * @param goalName  Null-terminated C string. `nullptr` or unknown names surface as
     *                  `Fault::BAD_NODE`.
     * @return          Fluent builder; sync-blocking by default.
     */
    [[nodiscard]] PathTrackerGoToBuilder goTo(const char* goalName);

    /**
     * @brief `true` if a fault is currently latched.
     */
    bool hasFault() const;

    /**
     * @brief Snapshot of the latched fault. Undefined when `!hasFault()`.
     */
    FaultInfo getFault() const;

    /**
     * @brief Clear the latched fault so the queue is re-armed.
     *
     * @note Does NOT restart motion. The queue was emptied at fault time; the caller is
     *       responsible for re-issuing whatever follow-up goTos it wants.
     */
    void clearFault();

private:
    /**
     * @brief Per-edge state machine.
     */
    enum class State : uint8_t
    {
        IDLE,                   ///< Queue empty, no goal in flight.
        DEQUEUE,                ///< Pop next goal entry, capture into `_currentGoal`.
        PLAN_NEXT_HOP,          ///< Look up `Routes::NEXT_HOP[from][goal]`; dispatch.
        TURNING,                ///< Awaiting `Drive::isIdle()` after `turnToYaw().async()`.
        LINE_FOLLOW,            ///< Awaiting `Intersection::isAtIntersection()` (read-and-clear).
        PROJECTION_FOLLOW,      ///< Awaiting `LineFollower::isDistanceReached()` (read-and-clear).
        BLIND_FORWARD,          ///< Awaiting `Drive::isIdle()` after `driveDist().async()`.
        GOAL_BLIND_DISPATCH,    ///< Issue next BlindMove from `_activeGoalApproach->blind_moves`.
        GOAL_BLIND_AWAIT,       ///< Awaiting `Drive::isIdle()` after a blind step.
        FAULT                   ///< Terminal sink; queue cleared.
    };

    /**
     * @brief Per-goal queue entry. (Future: per-goal flags like `silent`, `priority`.)
     */
    struct GoalEntry
    {
        uint8_t goalId;
    };

    // ---- queue plumbing ----
    bool _enqueue(uint8_t goalId);    ///< Returns false on overflow.
    bool _pop(GoalEntry& out);        ///< Returns false on empty.
    void _clearQueue();

    // ---- FSM helpers ----
    void _enterFault(Fault kind, uint8_t from, uint8_t to);
    void _onPlanNextHop();
    void _dispatchEdgeAfterTurn();    ///< Called from TURNING when Drive becomes idle.
    void _onGoalBlindDispatch();      ///< Issue next BlindMove or transition to IDLE.

    // ---- drain ----
    void _drainBlocking();

    // ---- builder dispatch ----
    /**
     * @brief Internal apply-hook called by `PathTrackerGoToBuilder::~`.
     *
     * Public-ish via `friend class PathTrackerGoToBuilder` (declared at the top of this
     * class) — do not call directly from user code.
     *
     * @param goalId   Resolved goal id (`FieldMap::NODE_NONE` -> `Fault::BAD_NODE`).
     * @param blocking If true, drain the queue before returning.
     */
    void _applyGoToBuilder(uint8_t goalId, bool blocking);

    GoalEntry _queue[Config::PATH_QUEUE_CAPACITY];
    uint8_t   _qHead  = 0;
    uint8_t   _qTail  = 0;
    uint8_t   _qCount = 0;

    State    _state         = State::IDLE;
    uint8_t  _currentGoal   = FieldMap::NODE_NONE;
    uint8_t  _nextHop       = FieldMap::NODE_NONE;
    Routes::EdgeType _activeEdgeType = Routes::EdgeType::NONE;
    const Routes::GoalApproach* _activeGoalApproach = nullptr;
    uint8_t  _blindIndex    = 0;

    FaultInfo _fault;
    void (*_tickCallback)() = nullptr;
    bool _tickCallbackWarningEmitted = false;
};

/**
 * @brief Single global instance — defined in PathTracker.cpp.
 */
extern PathTrackerClass PathTracker;

#endif    // PATHTRACKER_H
