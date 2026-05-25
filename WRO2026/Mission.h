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

#include "Config.h"
#include "NavTypes.h"

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
        LED
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
    };

    bool _push(const Step& s);
    void _advanceStep();
    void _emitFault();
    void _enterFaultBeeping(uint32_t now);
    void _setLeds(uint8_t r, uint8_t g, uint8_t b);
    void _beep(uint16_t freq_hz, uint16_t duration_ms);

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
};

extern MissionClass Mission;

#endif    // MISSION_H
