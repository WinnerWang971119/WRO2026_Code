/**
 * @file Mission.h
 * @brief Fully-manual mission DSL — goTo / wait / print / beep / led, queued + ticked.
 * @author WRO 2026
 *
 * @details
 * PR3 ships ``Mission`` as the operator-authored sequencer. The operator
 * writes the mission as a chain of builder calls in ``setup()``:
 *
 * @code
 *   Mission
 *       .goTo("A")
 *       .beep(1000, 200)
 *       .print("at A")
 *       .wait(500)
 *       .goTo("B")
 *       .led(0, 255, 0, 300)
 *       .goTo("C")
 *       .start();              // arm; returns immediately
 * @endcode
 *
 * ``start()`` is **non-blocking**. The mission advances entirely from
 * ``Mission.update()`` ticked from ``loop()`` alongside Drive,
 * LineFollower, Intersection, PathTracker, EventTrace.
 *
 * @warning **All Mission verbs are queue-only.** Calling ``Mission.goTo``,
 *          ``wait``, ``print``, ``beep``, ``led``, or ``task`` from inside a
 *          task body APPENDS the step to the queue and runs it AFTER the
 *          current TASK step's ``_advanceStep`` — it is NOT a synchronous
 *          call. Task bodies that need to drive the chassis, beep, or check
 *          state must use the underlying singletons directly: ``PathTracker``,
 *          ``Drive``, ``LineFollower``, ``MiniR4.Buzzer``, ``MiniR4.LED``.
 *          The ``task`` verb additionally guards against nested ``task()``
 *          enqueues (printing a Serial warning); the other verbs do not
 *          guard, so the queue-append happens silently. See ADR-025.
 *
 * Fault behaviour (PathTracker-fault OR ``goTo`` to an unknown node):
 *   1. The remaining queue is dropped immediately and the chassis is
 *      braked via ``Drive.stop(true)``.
 *   2. Mission transitions to ``FAULT_BEEPING``. Five ``MISSION_FAULT_BEEPS``
 *      cycles of 1 kHz ``MiniR4.Buzzer.beep(...)`` (the library owns the
 *      buzzer pin); the on-board RGB LEDs flash red in sync — on during
 *      the beep half-cycle, off during the gap.
 *   3. After the last cycle, Mission transitions to ``FAULT_AWAIT_RESET``,
 *      holds the LEDs solid red, and awaits a ``BTN_DOWN`` rising edge.
 *   4. On press: LEDs clear, ``PathTracker.clearFault()``, Mission
 *      returns to ``IDLE``.
 *
 * Fixed-size step queue (``Config::MAX_MISSION_STEPS``); overflow drops
 * the new step and prints a Serial warning.
 */

#ifndef MISSION_H
#define MISSION_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <type_traits>

#include "Config.h"
#include "NavTypes.h"
#include "Tasks.h"

class MissionClass
{
public:
    bool begin();

    /**
     * @brief Cooperative tick. Call from ``loop()`` alongside the other modules.
     *
     * Non-blocking. Advances at most one queue step per call; ``goTo``
     * steps spend many ticks awaiting PathTracker.isIdle().
     */
    void update();

    /**
     * @brief Enqueue a ``goTo("name")`` step. Resolves the name to a node id now.
     *
     * @note ``nodeName`` is stored as a pointer for the eventual Serial /
     *       OLED log line and must remain valid until the step executes.
     *       Passing a string literal (e.g. ``Mission.goTo("A")``) is the
     *       intended idiom and satisfies this naturally; do NOT pass a
     *       pointer to a stack buffer or a temporary ``String.c_str()``.
     */
    MissionClass& goTo(const char* nodeName);

    /** @brief Enqueue a non-blocking wait. */
    MissionClass& wait(uint32_t ms);

    /**
     * @brief Enqueue a Serial + OLED log entry.
     *
     * @note ``msg`` is stored as a pointer and must remain valid until the
     *       step executes. Same lifetime rule as :func:`goTo` — pass a
     *       string literal, not a stack buffer.
     */
    MissionClass& print(const char* msg);

    /**
     * @brief Enqueue a buzzer tone. Non-blocking via ``MiniR4.Buzzer.beep``.
     */
    MissionClass& beep(uint16_t freq_hz, uint32_t duration_ms);

    /**
     * @brief Enqueue an RGB LED step.
     *
     * @param duration_ms  0 holds the colour until the next ``led``
     *                     step. Non-zero clears after the timeout.
     */
    MissionClass& led(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms = 0);

    /**
     * @brief Force the direction of travel for EVERY hop on the path to the
     *        most-recently-enqueued ``goTo`` step's goal.
     *
     * @note No-op + Serial warning if the most-recent step is not a GOTO.
     *       Mission state is not corrupted on misuse. Defaults
     *       (``Direction::AUTO``) when never called preserve pre-T9 behaviour.
     *
     * @return ``*this`` for verb chaining.
     */
    MissionClass& setRouteDirection(Direction d);

    /**
     * @brief Force the direction of travel for the FINAL hop into the goal
     *        of the most-recently-enqueued ``goTo`` step.
     *
     * @note Same misuse semantics as :func:`setRouteDirection`. Wins over
     *       :func:`setRouteDirection` for the final hop only.
     *
     * @return ``*this`` for verb chaining.
     */
    MissionClass& setArrivalDirection(Direction d);

    /**
     * @brief Force stop-at-line behaviour for the BLIND hops on the path to
     *        the goal of the most-recently-enqueued ``goTo`` step.
     *
     * @note Same misuse semantics as :func:`setRouteDirection` (no-op + Serial
     *       warning if the most-recent step is not a GOTO, or if its push
     *       overflowed the queue). Forwarded to
     *       ``PathTracker.goTo(...).setStopAtLine(...)`` at GOTO execute: the
     *       call-site ON/OFF wins over each BLIND edge's BlindMode override,
     *       AUTO defers to the edge. Defaults (``StopAtLine::AUTO``) when never
     *       called preserve the per-edge BlindMode behaviour. See ADR-027.
     *
     * @param s  ON / OFF / AUTO — see :type:`StopAtLine`.
     *
     * @return ``*this`` for verb chaining.
     */
    MissionClass& setStopAtLine(StopAtLine s);

    /**
     * @brief Enqueue a named-task step. Packs ``args...`` into the step's
     *        16-byte ``TaskArgs`` POD via ``memcpy`` and stores the task name
     *        as a pointer (same string-literal lifetime rule as :func:`print`
     *        and :func:`goTo` — pass a string literal, not a stack buffer).
     *
     * @tparam Args  Zero or more trivially-copyable argument types. The total
     *               size ``sizeof...(args)`` summed by a fold expression must
     *               fit in ``Config::MAX_TASK_ARG_BYTES`` (16); a
     *               ``static_assert`` enforces this at compile time with the
     *               message ``"Mission.task args exceed MAX_TASK_ARG_BYTES"``.
     *               A second ``static_assert`` enforces
     *               ``std::is_trivially_copyable_v<Args>`` per arg — types
     *               with non-trivial copy constructors cannot survive the
     *               ``memcpy`` pack-and-unpack.
     * @param name   Registry key; resolved against ``Tasks._registry`` at
     *               BOTH queue time (here — bad names set ``_buildError`` and
     *               cause ``start()`` to refuse to arm) AND execute time
     *               (as a defence-in-depth fault check).
     * @param args   Argument pack. **Recommended:** pass a single per-task
     *               ``struct <Name>Args { ... };`` value so the byte layout
     *               matches the task body's ``reinterpret_cast<const
     *               NameArgs*>`` unpack. Mixing raw scalar types is only safe
     *               when the resulting byte layout is byte-identical to a
     *               contiguous ``uint8_t[N]`` (i.e. no padding). See the
     *               struct-padding warning on ``TaskArgs`` in TasksTypes.h.
     *
     * @return ``*this`` for verb chaining.
     *
     * @note Nested calls are forbidden — if invoked while ``Tasks.inTask()``
     *       returns ``true`` (i.e. from inside a task body), the call is
     *       ignored and a warning is printed to Serial. Use this for the
     *       operator-authored mission DSL, not from inside task bodies
     *       themselves. See the queue-only warning at the top of this header
     *       for why the OTHER verbs (``goTo``, ``wait``, ``print``, ``beep``,
     *       ``led``) do NOT guard against nested calls — they queue silently.
     */
    template <typename... Args>
    MissionClass& task(const char* name, Args... args);

    /**
     * @brief Arm the mission. Returns immediately; subsequent ``update()``
     *        calls drain the queue. Logs a Serial message on transition.
     *
     * @warning **Calling ``start()`` while a mission is already running
     *          silently rewinds the queue index to 0 and re-runs the
     *          entire queue from the first step.** The FSM is moved
     *          unconditionally into ``RUNNING``; there is no guard
     *          against re-entry. The intended usage pattern is one
     *          ``buildMission(); Mission.start();`` per power cycle,
     *          gated externally (the .ino uses ``_missionArmed`` for
     *          this). If you need to re-arm a finished mission, this
     *          is fine; if you need to abort + replace a running
     *          mission, call ``Mission.abort()`` first.
     * @warning Calling ``start()`` while in ``FAULT_BEEPING`` or
     *          ``FAULT_AWAIT_RESET`` also rewinds to ``RUNNING`` and
     *          the next tick will immediately re-fault if the
     *          underlying PathTracker fault is still latched. Clear
     *          the fault (or call ``Mission.abort()``) first.
     */
    void start();

    /**
     * @brief Real safety stop: cancel the in-flight PathTracker FSM,
     *        disable LineFollower, brake the chassis, empty the queue,
     *        return Mission to ``IDLE``, and clear any latched fault.
     *
     * Symmetric with ``Drive.stop(true)`` but at the mission level —
     * use this anywhere you need a hard cancel of every cooperative
     * subsystem that could re-issue motion on the next tick.
     */
    void abort();

    /** @brief ``true`` when no mission is running and no fault is latched. */
    bool isIdle() const;

    /** @brief ``true`` while the halt-beep-flash sub-FSM is active. */
    bool hasFault() const;

private:
    enum class StepType : uint8_t
    {
        GOTO,
        WAIT,
        PRINT,
        BEEP,
        LED,
        TASK,    ///< new (T4)
    };

    enum class State : uint8_t
    {
        IDLE,
        RUNNING,
        FAULT_BEEPING,
        FAULT_AWAIT_RESET
    };

    enum class StepState : uint8_t
    {
        ENTER,
        ACTIVE
    };

    // NOTE: Step is a flat struct paying the worst-case size for every slot —
    // a 20-byte growth in this PR (TaskArgs + task_name) times
    // MAX_MISSION_STEPS=64 = ~1.28 KB of mostly-unused queue storage when the
    // mission is dominated by GOTO/WAIT/PRINT steps. A tagged-union refactor
    // (one payload per StepType) is the right shape but is multi-PR; tracked
    // as a backlog item for the Phase 6 sweep. The 32 KB SRAM budget is
    // comfortable today.
    struct Step
    {
        StepType    type;
        uint32_t    duration_ms;       ///< wait / beep / led duration
        uint16_t    freq_hz;           ///< beep frequency
        uint8_t     goal;              ///< goto: resolved node id
        uint8_t     r;
        uint8_t     g;
        uint8_t     b;
        const char* text;              ///< print / goto name (for logging)
        Direction   route_direction;   ///< goto: per-step setRouteDirection override (default AUTO).
        Direction   arrival_direction; ///< goto: per-step setArrivalDirection override (default AUTO).
        StopAtLine  stop_at_line;      ///< goto: per-step setStopAtLine override (default AUTO).
        // T4: task step
        const char* task_name = nullptr;    ///< TASK: registry key (string-literal lifetime, same rule as `text`)
        TaskArgs    task_args{};            ///< TASK: packed args
    };

    bool _push(const Step& s);
    void _advanceStep();
    void _emitFault();
    void _enterFaultBeeping(uint32_t now);
    void _setLeds(uint8_t r, uint8_t g, uint8_t b);
    void _beep(uint16_t freq_hz, uint16_t duration_ms);

    /**
     * @brief Recursively ``memcpy`` each argument into ``dst``.
     *
     * Member function so the inline ``task<Args...>`` template can access the
     * private ``Step`` / ``StepType`` types from the same scope. ``memcpy`` is
     * used instead of a placement-new / aliased pointer write because the R4's
     * ARM Cortex-M4 has weak alignment guarantees on unaligned word stores —
     * ``memcpy`` is always safe regardless of the run-time alignment of ``dst``.
     */
    template <typename T, typename... Rest>
    void _packArgs(uint8_t* dst, const T& head, const Rest&... rest);

    /** @brief Base case for the variadic ``_packArgs`` recursion. */
    void _packArgs(uint8_t* /*dst*/);

    Step _queue[Config::MAX_MISSION_STEPS];
    uint8_t _count = 0;
    uint8_t _idx   = 0;

    State     _state     = State::IDLE;
    StepState _stepState = StepState::ENTER;
    uint32_t  _stepStart_ms = 0;

    uint8_t  _faultBeepIdx       = 0;
    uint32_t _faultPhaseStart_ms = 0;
    bool     _faultLedOn         = false;
    bool     _btnPrev            = false;

    /**
     * @brief Sticky build-error latch set by ``task<>()`` when the supplied
     *        registry name does not resolve at queue time. ``start()`` checks
     *        this and refuses to arm the mission if set, surfacing the typo
     *        before any chassis motion happens. Cleared by ``begin()`` and
     *        the fault-clear path (``FAULT_AWAIT_RESET`` → ``IDLE``).
     */
    bool _buildError = false;

    /// True iff the most recent step-pushing verb (goTo/wait/print/beep/led)
    /// successfully enqueued a step. setRouteDirection / setArrivalDirection
    /// MUST consult this flag before mutating `_queue[_count - 1]` — without
    /// it, a chained modifier after a dropped overflow step silently rewrites
    /// the PREVIOUS step (often another GOTO) and corrupts an unrelated goal's
    /// direction. Reset by every step-pushing verb so the modifier can only
    /// ever target the immediately-preceding push.
    bool     _lastPushOk         = false;
};

extern MissionClass Mission;

// -----------------------------------------------------------------------------
// Inline template definitions
//
// The variadic ``task<Args...>`` verb and its ``_packArgs`` recursion helpers
// must be visible at every instantiation point, so their bodies live in the
// header. They sit after the class body so they may freely refer to the global
// ``Tasks`` singleton declared in ``Tasks.h`` (included above).
// -----------------------------------------------------------------------------

template <typename... Args>
MissionClass& MissionClass::task(const char* name, Args... args)
{
    // Binary right fold with `0` as the identity already produces 0 for an
    // empty pack — no ternary guard needed. (Unary right fold `(sizeof(Args)
    // + ...)` would be ill-formed for the empty pack; the `+ 0` initializer
    // is what makes this expression safe.)
    constexpr size_t TOTAL = (sizeof(Args) + ... + 0);
    static_assert(TOTAL <= Config::MAX_TASK_ARG_BYTES,
                  "Mission.task args exceed MAX_TASK_ARG_BYTES");

    // Non-trivially-copyable arg types cannot survive the memcpy pack-and-
    // unpack flow inside _packArgs / task body — they would invoke copy
    // constructors that are erased by the byte-level copy. Forcing a TC
    // requirement at compile time prevents callers from accidentally passing
    // e.g. a std::string or a class with a vtable.
    static_assert((std::is_trivially_copyable_v<Args> && ...),
                  "Mission.task args must be trivially-copyable");

    if (Tasks.inTask())
    {
        Serial.println(F("[Mission] task() called from inside a task body; ignored"));
        return *this;
    }

    // Queue-time registry validation: catch typos before any chassis motion.
    // findTask is the same linear strcmp scan runTask uses at execute time;
    // the redundancy is acceptable because it's a one-time queue-time cost
    // and it turns a mid-route fault into a pre-arm refusal.
    if (Tasks.findTask(name) == nullptr)
    {
        Serial.print(F("[Mission] task('"));
        Serial.print(name == nullptr ? "(null)" : name);
        Serial.println(F("') unknown at queue time; start() will refuse to arm"));
        _buildError = true;
        return *this;
    }

    Step s{};
    s.type      = StepType::TASK;
    s.task_name = name;
    _packArgs(s.task_args.bytes, args...);
    _push(s);
    return *this;
}

template <typename T, typename... Rest>
inline void MissionClass::_packArgs(uint8_t* dst, const T& head, const Rest&... rest)
{
    memcpy(dst, &head, sizeof(T));
    _packArgs(dst + sizeof(T), rest...);
}

inline void MissionClass::_packArgs(uint8_t* /*dst*/) {}

#endif    // MISSION_H
