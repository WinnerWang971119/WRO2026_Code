/**
 * @file Tasks.h
 * @brief Tasks framework — registry-driven, member-fn-ptr-dispatched task bodies.
 * @author WRO 2026
 *
 * @details
 * The Tasks framework lets Mission steps invoke named, parameterised "tasks"
 * (e.g. ``demoEcho``, eventually ``mosaicPlace`` / ``parkInBay`` / ...) without
 * Mission needing to know anything about each task's implementation. A task is
 * a non-static member function on :class:`TasksClass` with the signature
 *
 * @code
 *   bool TasksClass::<name>(const TaskArgs& args);
 * @endcode
 *
 * It receives a 16-byte ``TaskArgs`` POD union packed by the operator-facing
 * ``Mission.task<Args...>(name, args...)`` template (added in a later T-task)
 * and returns ``true`` on success / ``false`` on "yield without abort".
 *
 * The single source of truth for valid task names is the static
 * ``_registry`` array in :file:`Tasks.cpp`. ``runTask`` performs a ``strcmp``
 * linear scan (registry is bounded by ``Config::MAX_TASKS = 16``, so a hash
 * map would be wasteful) and dispatches through a pointer-to-member-function.
 *
 * Lifecycle invariants:
 *   - ``_inTask`` is owned EXCLUSIVELY by ``runTask`` via an RAII scope guard.
 *     No other method (including ``abort()`` / ``clearAbort()``) touches it.
 *     ``runTask`` also READS it on entry to reject reentrant dispatch (a task
 *     body calling ``runTask`` again) — tasks are non-reentrant leaf
 *     operations; see ADR-026.
 *   - ``_aborted`` is a sticky latch settable via ``abort()`` and readable
 *     via :func:`isAborted`. It is cleared by either ``begin()`` (setup-time)
 *     or ``clearAbort()`` (called from ``Mission.start()`` before each new
 *     mission run) so an abort latched during run N does not bleed into
 *     run N+1.
 *   - There is no persistent active-task FSM state in TasksClass for this
 *     plan — task bodies run to completion inside ``runTask``. Multi-tick
 *     cooperative tasks land in a later iteration; for now ``update()`` is
 *     a deliberate no-op so the singleton can be ticked from ``loop()``
 *     without churn.
 *
 * @note The POD layer (``TaskArgs`` / ``TaskFn`` / ``TaskRegistryEntry``) lives
 *       in :file:`TasksTypes.h` to break the otherwise-cyclic dependency
 *       between :file:`Mission.h` (which needs the arg-union and registry-entry
 *       types to declare the ``TASK`` step variant) and :file:`Tasks.h`
 *       (which adds the class on top of those PODs).
 */

#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include "TasksTypes.h"

class TasksClass
{
public:
    /**
     * @brief Reset the task-runner state. Call from the sketch's ``setup()``.
     *
     * @return ``true`` always; signature matches the rest of the project's
     *         ``begin()`` convention so failures can be added later without
     *         touching call sites.
     */
    bool begin();

    /**
     * @brief Cooperative tick. Call from ``loop()`` alongside the other modules.
     *
     * Housekeeping only; no FSM state in this plan. Kept as a non-trivial
     * entry point so the call-site list in ``cooperativeTick()`` does not
     * churn when a future iteration adds multi-tick task support.
     */
    void update();

    /**
     * @brief Register the cooperative-tick callback driven during blocking task waits.
     *
     * Some task bodies (``servoTest``, ``home``) block until an actuator settles. While
     * they spin they must keep every module ticking — otherwise EventTrace stops
     * rendering, the belt's own ``update()`` stops advancing, etc. — so the wait invokes
     * this callback each iteration (mirroring ``ManipulatorClass::setTickCallback`` and
     * ``Drive``/``PathTracker``/``LineFollower``). The sketch passes ``cooperativeTick``
     * here in ``setup()`` after every module's ``begin()`` has run.
     *
     * @param cb Cooperative-tick function pointer, or ``nullptr`` to clear it (the
     *           blocking waits then degrade to ticking only ``Manipulator.update()`` and
     *           warn once).
     *
     * @note ``cooperativeTick()`` calls ``Mission.update()``, which short-circuits while
     *       ``Tasks.inTask()`` is true, so driving it from inside a task wait does NOT
     *       re-enter the task.
     */
    void setTickCallback(void (*cb)());

    /**
     * @brief Look up ``name`` in the static registry and dispatch its body.
     *
     * @param name  Null-terminated task name. Resolved by ``strcmp`` linear
     *              scan over ``_registry``.
     * @param args  Packed argument union. The body owns the unpack ABI for
     *              its specific task (typically ``reinterpret_cast`` to a
     *              per-task ``struct``).
     *
     * @return The body's return value; or ``false`` if ``name`` is unknown
     *         (an ``[Tasks] runTask: unknown name '<name>'`` warning is
     *         printed to Serial); or ``false`` if called reentrantly (see the
     *         reentrancy note below).
     *
     * @note **Non-reentrant.** If ``runTask`` is invoked while a task body is
     *       already in flight (``_inTask`` is ``true``), it refuses: it prints
     *       a ``[Tasks] runTask: refusing reentrant call to '<name>'`` warning
     *       and returns ``false`` WITHOUT resolving ``name`` or arming the
     *       guard. Tasks are leaf operations — share logic between them via
     *       private helpers, not by calling ``runTask`` from inside a task.
     *       See ADR-026.
     *
     * @note ``_inTask`` is set ``true`` for the duration of dispatch via an
     *       RAII scope guard, so it is correctly reset even if the body
     *       returns through every path. ``abort()`` does NOT clear
     *       ``_inTask`` — only the guard does.
     */
    bool runTask(const char* name, const TaskArgs& args);

    /**
     * @brief Linear-scan ``_registry`` for an entry whose name matches ``name``.
     *
     * @param name  Null-terminated task name.
     * @return Pointer to the matching ``TaskRegistryEntry``, or ``nullptr``
     *         when no entry's ``name`` field compares equal under ``strcmp``.
     *
     * @note Exposed publicly so future call sites (e.g. a Mission
     *       pre-validation pass at queue-time) can check name validity
     *       without paying the dispatch cost.
     */
    const TaskRegistryEntry* findTask(const char* name) const;

    /**
     * @brief Latch a request to stop the current task framework.
     *
     * Sets ``_aborted = true``. The flag is sticky for the duration of the
     * current mission and is reset by either ``begin()`` (setup-time) or
     * ``clearAbort()`` (called from ``Mission.start()`` before each new
     * mission run). Does NOT clear ``_inTask`` — that is owned solely by
     * ``runTask``'s scope guard so the in-flight body still gets to unwind
     * cleanly.
     */
    void abort();

    /**
     * @brief Clear the sticky ``_aborted`` latch without otherwise resetting
     *        Tasks state.
     *
     * Called by ``Mission.start()`` so the abort latch from a previous
     * mission does not bleed into the next run. Does NOT touch ``_inTask``.
     */
    void clearAbort();

    /** @brief ``true`` after ``abort()`` has been called and before the next ``begin()`` / ``clearAbort()``. */
    bool isAborted() const;

    /** @brief ``true`` while ``runTask`` is currently dispatching a body. */
    bool inTask() const;

    /**
     * @brief Static registry of all known task names + their member-fn pointers.
     *
     * Defined in ``Tasks.cpp`` — adding a new task is a single-line append to
     * the initializer there. ``_registry_count`` auto-derives from ``sizeof``,
     * and a file-scope ``static_assert`` rejects the build if the count
     * exceeds ``Config::MAX_TASKS``.
     *
     * @note Public so the bounds ``static_assert`` and any future external
     *       validators (e.g. a Mission queue-time pre-check) can read the
     *       size without the entries needing to grow a friend declaration.
     *       The members are ``const`` — privacy here gives no additional
     *       safety, only friction.
     */
    static const TaskRegistryEntry _registry[];
    static const size_t            _registry_count;

private:
    /**
     * @brief Smoke-test task body — prints the first two ``uint8`` arg slots.
     *
     * @param args  Packed argument union; reads ``args.u8[0]`` and ``args.u8[1]``.
     * @return Always ``true``.
     *
     * @note Serves as the registry's first row so the rest of the plumbing
     *       (Mission's ``TASK`` step, ``runTask`` dispatch, registry lookup)
     *       has a deterministic, side-effect-only target during bring-up.
     */
    bool demoEcho(const TaskArgs& args);

    /**
     * @brief Always-yield task body — returns ``false`` so the FSM's YIELD
     *        path (``TASK YIELD %s`` event + Serial line + ``_advanceStep``)
     *        has a production-side exercise.
     *
     * @param args  Ignored.
     * @return Always ``false``.
     *
     * @note Without this, the YIELD path is exercised only by the host-test
     *       mirror's stub and any regression in Mission's TASK case that
     *       mis-handles ``ok==false`` would ship undetected.
     */
    bool debugYield(const TaskArgs& args);

    /**
     * @brief Drive the sorter servo to an absolute angle and block until it settles.
     *
     * Bring-up helper: sets `Manipulator.sorter` to `args.u8[0]` degrees and polls
     * `Manipulator.update()` until the servo's settle timer reports done. Used to sweep
     * and confirm sorter bin angles by hand at the bench.
     *
     * @param args `u8[0]` = target angle in degrees.
     * @return Always `true`.
     */
    bool servoTest(const TaskArgs& args);

    /**
     * @brief Home the conveyor belt via `ManipBelt::home()`, then return.
     *
     * Thin wrapper: calls `Manipulator.belt.home()` — which stalls to the LEFT/HOME
     * end-stop, re-zeros the encoder, and drives on to the MID rest position — and polls
     * `Manipulator.update()` until the whole sequence lands. The belt's own per-stage
     * watchdogs bound the moves, so the wait always terminates.
     *
     * @param args Ignored.
     * @return Always `true`.
     */
    bool home(const TaskArgs& args);

    bool test1(const TaskArgs& args);

    /**
     * @brief Line-follow a short fixed distance — a smoke test for the line + drive path.
     *
     * Bring-up helper: enables the LineFollower with a hard-coded 50 cm distance target and
     * blocks (driving the cooperative tick) until the distance gate trips. The arg is
     * ignored — the 50 cm target is fixed in the body, not taken from `args`.
     *
     * @param args Ignored.
     * @return `true` once the 50 cm gate trips; `false` if the follow times out or Drive was
     *         not idle when the task ran.
     */
    bool testChassis(const TaskArgs& args);

    /**
     * @brief Bring-up nav demo: hop a fixed node sequence via PathTracker.
     *
     * @param args Ignored.
     * @return Always `true`.
     */
    bool testOne(const TaskArgs& args);

    bool _inTask  = false;
    bool _aborted = false;

    /**
     * @brief Cooperative-tick callback driven during blocking task waits (``servoTest`` /
     *        ``home``). ``nullptr`` until the sketch registers ``cooperativeTick`` via
     *        ``setTickCallback`` in ``setup()``.
     */
    void (*_tickCallback)() = nullptr;
};

extern TasksClass Tasks;

#endif    // TASKS_H
