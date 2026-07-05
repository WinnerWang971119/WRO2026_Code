/**
 * @file StateMachine.h
 * @brief Minimal header-only templated state-machine helper for WRO 2026 task modules.
 * @author Diego Wang
 *
 * @note This header is intentionally not instantiated in Phase 1. It is authored here so that
 *       Phase 4 task modules can include it without modification. Because StateMachine is a
 *       template class, the entire implementation must reside in this header.
 */

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>

/**
 * @brief Lightweight, header-only templated state machine.
 *
 * Tracks the current state of type @p State, the wall-clock timestamp of the most
 * recent transition, and a single-iteration "changed" flag that lets callers react
 * exactly once to each transition without polling a delta.
 *
 * Typical usage pattern inside a task's `update()` method:
 * @code
 *   if (_sm.changed()) {
 *       _sm.ack();          // consume the flag
 *       onEnter(_sm.current());
 *   }
 *   if (_sm.elapsed() > TIMEOUT_MS) {
 *       _sm.enter(NEXT_STATE);
 *   }
 * @endcode
 *
 * @tparam State An enum (or any cheap copy-assignable type) whose values represent
 *               the discrete states of the machine.
 *
 * @note `millis()` wraps after ~49.7 days. `elapsed()` uses unsigned subtraction so
 *       it handles the wrap correctly as long as no single state lasts longer than
 *       the wrap period.
 */
template<typename State>
class StateMachine
{
public:
    /**
     * @brief Constructs the state machine and immediately enters the initial state.
     *
     * Records `millis()` as the entry time and sets the changed flag so the first
     * call to `changed()` returns `true`, allowing callers to run entry logic
     * unconditionally on the first update cycle.
     *
     * @param initial The starting state of the machine.
     */
    StateMachine(State initial) : _state(initial), _enteredAt(millis()), _changed(true) {}

    /**
     * @brief Transitions the machine to state @p s.
     *
     * Updates the current state, captures the entry timestamp via `millis()`, and
     * asserts the changed flag. Transitioning to the same state is permitted and
     * will reset the elapsed timer and re-assert the flag — this allows callers to
     * "restart" a timed phase without leaving the state.
     *
     * @param s The target state to transition into.
     */
    void enter(State s)
    {
        _state     = s;
        _enteredAt = millis();
        _changed   = true;
    }

    /**
     * @brief Returns the current state of the machine.
     *
     * @return The active state value of type @p State.
     */
    State current() const
    {
        return _state;
    }

    /**
     * @brief Returns the number of milliseconds elapsed since the last call to `enter()`.
     *
     * Uses unsigned 32-bit subtraction, which correctly handles the `millis()` rollover
     * at 2^32 − 1 ms (~49.7 days).
     *
     * @return Elapsed time in milliseconds as a `uint32_t`.
     */
    uint32_t elapsed() const
    {
        return static_cast<uint32_t>(millis() - _enteredAt);
    }

    /**
     * @brief Returns whether the state changed on the most recent call to `enter()`.
     *
     * The flag is set to `true` inside `enter()` and remains `true` until the caller
     * explicitly clears it with `ack()`. This lets a single state transition be
     * detected across exactly one update iteration.
     *
     * @return `true` if `enter()` has been called and the flag has not yet been
     *         cleared by `ack()`; `false` otherwise.
     */
    bool changed() const
    {
        return _changed;
    }

    /**
     * @brief Acknowledges and clears the changed flag.
     *
     * Call this after processing the entry logic triggered by `changed()`. Subsequent
     * calls to `changed()` will return `false` until the next `enter()` call.
     *
     * @note Calling `ack()` when the flag is already `false` is a no-op.
     */
    void ack()
    {
        _changed = false;
    }

private:
    /** @brief The currently active state. */
    State    _state;

    /** @brief The `millis()` timestamp recorded when the current state was entered. */
    uint32_t _enteredAt;

    /** @brief True from the moment `enter()` is called until `ack()` clears the flag. */
    bool     _changed;
};

#endif // STATEMACHINE_H
