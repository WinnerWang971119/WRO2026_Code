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
 * On any PathTracker fault mid-``goTo``, Mission enters a fault state:
 *   1. Five 500-ms beeps at 1 kHz on pin 6.
 *   2. Both RGB LEDs flash red, then hold red.
 *   3. The remaining queue is dropped; Mission idles awaiting
 *      ``BTN_DOWN``.
 *   4. On press: LEDs clear, PathTracker fault is cleared, Mission
 *      returns to IDLE.
 *
 * Fixed-size step queue (``Config::MAX_MISSION_STEPS``); overflow drops
 * the new step and prints a Serial warning.
 */

#ifndef MISSION_H
#define MISSION_H

#include <Arduino.h>
#include <stdint.h>

#include "Config.h"

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
     * @brief Enqueue a ``goTo("name")`` step. Resolves name to node id now.
     */
    MissionClass& goTo(const char* nodeName);

    /** @brief Enqueue a non-blocking wait. */
    MissionClass& wait(uint32_t ms);

    /** @brief Enqueue a Serial + OLED log entry. ``msg`` must remain valid. */
    MissionClass& print(const char* msg);

    /** @brief Enqueue a buzzer tone. Non-blocking via ``tone()``. */
    MissionClass& beep(uint16_t freq_hz, uint32_t duration_ms);

    /**
     * @brief Enqueue an RGB LED step.
     *
     * @param duration_ms  0 holds the colour until the next ``led``
     *                     step. Non-zero clears after the timeout.
     */
    MissionClass& led(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms = 0);

    /**
     * @brief Arm the mission. Returns immediately; subsequent ``update()``
     *        calls drain the queue. Logs a Serial message on transition.
     */
    void start();

    /** @brief Drop the entire queue and return to IDLE. Stops Drive. */
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
        uint32_t    duration_ms;    ///< wait / beep / led duration
        uint16_t    freq_hz;        ///< beep frequency
        uint8_t     goal;           ///< goto: resolved node id
        uint8_t     r;
        uint8_t     g;
        uint8_t     b;
        const char* text;           ///< print / goto name (for logging)
    };

    bool _push(const Step& s);
    void _advanceStep();
    void _emitFault();
    void _setLeds(uint8_t r, uint8_t g, uint8_t b);

    Step _queue[Config::MAX_MISSION_STEPS];
    uint8_t _count = 0;
    uint8_t _idx   = 0;

    State     _state     = State::IDLE;
    StepState _stepState = StepState::ENTER;
    uint32_t  _stepStart_ms = 0;

    uint8_t  _faultBeepIdx       = 0;
    uint32_t _faultPhaseStart_ms = 0;
    bool     _btnPrev            = false;
};

extern MissionClass Mission;

#endif    // MISSION_H
