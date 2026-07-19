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

class ManipServo;    // fwd decl only — _waitForServo takes a reference; the full
                     // type is pulled in by Tasks.cpp.

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
    * Some task bodies (``servoTest``, ``home``, ``idleWait``) block until an actuator
    * settles or a cooperative timeout elapses. While they spin they must keep every
    * module ticking — otherwise EventTrace stops rendering, the belt's own ``update()``
    * stops advancing, etc. — so the wait invokes this callback each iteration
    * (mirroring ``ManipulatorClass::setTickCallback`` and
    * ``Drive``/``PathTracker``/``LineFollower``). The sketch passes
    * ``cooperativeTick`` here in ``setup()`` after every module's ``begin()`` has run.
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

private:
    /**
     * @brief Static registry of all known task names + their member-fn pointers.
     *
     * Defined in ``Tasks.cpp`` — adding a new task is a single-line append to
     * the initializer there. ``_registry_count`` auto-derives from ``sizeof``,
     * and a file-scope ``static_assert`` rejects the build if the count
     * exceeds ``Config::MAX_TASKS``.
     *
     * @note Kept private because the registry is an implementation detail of
     *       ``TasksClass`` lookup and dispatch.
     */
    static const TaskRegistryEntry _registry[];

public:
    /**
    * @brief Number of entries in ``_registry``.
    *
    * Exposed so file-scope validation in ``Tasks.cpp`` can enforce the
    * ``Config::MAX_TASKS`` cap without exposing the array itself.
    */
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
     * @brief Grab the object to the bin in one call.
     * @param bin  which bin it is sorting to.
     * @note This is NOT a task that can be called at the mission.
     */
    bool _grabToBin(uint8_t bin);

    /**
     * @brief Capture one mosaic observation into the `Vision` latch, with a single retry.
     *
     * Thin wrapper over `Vision.readMosaic()`: on a fault it logs, clears the fault, and
     * retries exactly once. The captured colours are NOT returned — they live in the
     * `Vision` latch (the single source of truth); read them per-cell with
     * `Vision.getColor(cell)` after this returns `true`.
     *
     * @return `true` if a mosaic frame was captured (either attempt); `false` if both the
     *         initial read and the retry faulted, in which case the latch is unchanged.
     * @note The chassis MUST be fully idle before calling — see `Vision::readMosaic`. This is
     *       a command-time BLOCKING call and is NOT a task callable from the mission DSL.
     */
    bool _visionRead();

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

    /// @brief Test-support stub for Mission's TASK YIELD path: prints its
    ///        signature and returns false so a real Mission/Tasks integration can
    ///        exercise the yield (not abort) branch end-to-end.
    bool debugYield(const TaskArgs& args);

    /// @brief Cooperative idle-hold for `args.u32[0]` ms while the chassis is
    ///        stationary; refuses (returns false) if any motion is active.
    bool idleWait(const TaskArgs& args);

    bool testOne(const TaskArgs& args);

    /**
     * @brief Bring-up CTPS demo.
     *
     * @param args Ignored.
     * @return Always `true`.
     */
    bool ctpsExample1(const TaskArgs& args);

    /**
     * @brief Empty no-op test hook.
     *
     * Reserved for quick Mission/task plumbing tests. Does no motion, no manipulator actuation,
     * and simply returns success.
     *
     * @param args Ignored.
     * @return Always ``true``.
     */
    bool emptyTest(const TaskArgs& args);

    /**
     * @brief Belt-only bring-up: start the belt home sequence.
     *
     * Calls `Manipulator.belt.home()` (stall into the LEFT/HOME end-stop, re-zero the encoder,
     * then drive to the MID rest position) so the operator can watch the belt seek home without
     * any grabber / sorter / dumper motion. `belt.home()` is non-blocking, so the sequence
     * continues via `Manipulator.update()` after this returns.
     *
     * @param args Ignored.
     * @return Always ``true``.
     */
    bool beltTest(const TaskArgs& args);

    /**
     * @brief Primitive-handle bench test for every manipulator actuator.
     *
     * Drives the three servos and the belt DIRECTLY through `Manipulator`'s public primitive
     * handles (`grabber` / `sorter` / `dumper` / `belt`) so bring-up can verify each actuator
     * without going through the semantic verbs.
     *
     * @param args u8[0] = 1 to enable belt test; otherwise ignored.
     * @return ``true`` when every primitive movement completed; ``false`` on a belt/servo timeout.
     */
    bool manipDirectTest(const TaskArgs& args);

    /**
     * @brief Go to the place and read the mosaic colors.
     * @param args  Ignored.
     * @return `true` if read successfully; `false` if the read failed (after one retry).
     * @note Read the colors using the `Vision` latch. use .getColor(cell) to read the color of each cell after this returns `true`.
     */
    bool readMosaic(const TaskArgs& args);

    /**
     * @brief Get and put the rectangle shovel
     * @param args  Ignored.
     * @return true when finish. 
     */
    bool rectangleShovel(const TaskArgs& args);

    /**
     * @brief Get and put the triangle shovel
     * @param args  Ignored.
     * @return true when finish.
     */
    bool triangleShovel(const TaskArgs& args);

    /**
     * @brief Get and put the pot
     * @param args  Ignored.
     * @return true when finish.
     */
    bool pot(const TaskArgs& args);

    /**
     * @brief Get and put the mosiac
     * @param args  Ignored.
     * @return true when finish.
     * @note Need the helper _mosiacAlgo to get the mosiac fastest
     */
    bool mosaic(const TaskArgs& args);

    /**
     * @brief Get and put the debries
     * @param args  Ignored.
     * @return true when finish.
     */
    bool debries(const TaskArgs& args);

    /**
     * @brief Stationary cooperative delay helper for task bodies.
     *
     * @param ms    Timeout in milliseconds.
     * @param site  Short caller tag for diagnostics.
     * @return ``true`` when the timeout elapses; ``false`` if the chassis is moving at
     *         entry or the task framework is aborted mid-wait.
     */
    bool _waitStationaryMs(uint32_t ms,
                          const __FlashStringHelper* site = F("waitStationaryMs"));

    // --- Cooperative-wait helpers -------------------------------------------
    // Shared by the bring-up tasks (``emptyTest`` / ``actuatorSweep`` /
    // ``actuatorStep`` / ``manipDirectTest``) that block until an actuator
    // settles. Each keeps every module ticking during the wait via the
    // registered cooperative-tick callback (falling back to Drive + Manipulator
    // ``update()`` with a one-shot warning when none is set), mirroring
    // ``Manipulator::_drain``.

    /** @brief Drive one cooperative-tick iteration during a blocking wait. */
    void _pumpTick();

    /**
     * @brief Spin ``_pumpTick()`` for @p durationMs, servicing every module.
     * @param durationMs Wall-clock duration to wait, in milliseconds.
     */
    void _waitMs(uint32_t durationMs);

    /**
     * @brief Block until @p servo settles or @p timeoutMs elapses.
     * @param servo     Servo joint to wait on.
     * @param timeoutMs Give-up timeout in milliseconds.
     * @param label     Short site label printed on timeout.
     * @return ``true`` once the servo reports done; ``false`` on timeout.
     */
    bool _waitForServo(ManipServo& servo, uint32_t timeoutMs, const char* label);

    /**
     * @brief Block until the belt settles or @p timeoutMs elapses.
     * @param timeoutMs Give-up timeout in milliseconds.
     * @param label     Short site label printed on timeout.
     * @return ``true`` once the belt reports done; ``false`` on timeout.
     */
    bool _waitForBelt(uint32_t timeoutMs, const char* label);

    /**
     * @brief Get the mosaic fastest
     * @param args  Ignored.
     * @return true when finish.
     * @note This is a helper function for the mosaic task.
     */
    bool _mosaicAlgo(const TaskArgs& args);

    bool _inTask  = false;
    bool _aborted = false;

    /**
     * @brief Cooperative-tick callback driven during blocking task waits (``servoTest`` /
     *        ``home``). ``nullptr`` until the sketch registers ``cooperativeTick`` via
     *        ``setTickCallback`` in ``setup()``.
     */
    void (*_tickCallback)() = nullptr;

    /**
     * @brief One-shot latch so the "no tick callback set" warning printed by
     *        ``_pumpTick`` fires at most once per run.
     */
    bool _noCallbackWarned = false;
};

extern TasksClass Tasks;

#endif    // TASKS_H
