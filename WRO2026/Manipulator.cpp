/**
 * @file Manipulator.cpp
 * @brief Out-of-line members of `ManipulatorClass` — the manipulator-subsystem facade.
 *
 * @note The semantic verbs (`grab`, `release`, `dump`, `sortTo`, `reset`), the `_action`
 *       factory, and the `ManipAction` builder are all defined inline in `Manipulator.h`.
 *       Only the heavy lifecycle / blocking-spin members live here: the ctor (so the member
 *       initialiser list can name the `Config::*` tables in one place), `begin()`, `update()`,
 *       `isDone()`, `setTickCallback()`, and the `_drain` / `_maskDone` drain machinery.
 *
 * @note `<Arduino.h>` (millis / delay / Serial) is pulled in transitively through
 *       `Manipulator.h` -> `ManipServo.h` / `ManipBelt.h`, so it is not re-included here.
 */

#include "Manipulator.h"

/// @brief Single global instance declared `extern` in Manipulator.h.
ManipulatorClass Manipulator;

// -----------------------------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------------------------

// Member initialiser list order MUST match the declaration order in Manipulator.h
// (grabber, sorter, dumper, belt) or the compiler warns -Wreorder. Each servo stores its
// `Config::ServoCfg` BY CONST REFERENCE, so we pass the named `constexpr` (which has static
// lifetime) rather than a temporary — a temporary would dangle the moment the ctor returns.
ManipulatorClass::ManipulatorClass()
    : grabber(Config::GRABBER),
      sorter(Config::SORTER),
      dumper(Config::DUMPER),
      belt(Config::BELT_STATIONS, Config::BELT_STATION_COUNT, Config::BELT_M_ID)
{
    // No hardware work — the actuators only capture their config here. Hardware bring-up is
    // deferred to begin(), which must run after MiniR4.begin() has returned.
}

// -----------------------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------------------

bool ManipulatorClass::begin()
{
    // Configure each servo's hardware direction + seed its move-timer shadow, then command
    // each to its home pose so the manipulator boots into a known safe pose (AC8).
    grabber.begin();
    sorter.begin();
    dumper.begin();

    grabber.home();
    sorter.home();
    dumper.home();

    // Sanctioned setup-time blocking spin (same rationale as Drive::calibrateMotorBias's
    // explicit delay): begin() runs BEFORE any cooperative tick callback is registered — the
    // sketch calls setTickCallback() only after every module's begin() — so we cannot route
    // this wait through _drain()'s callback. A plain millis()/delay() poll is the only option
    // this early, and the caller is off the cooperative tick chain by definition. The three
    // servos are pure timers (no per-tick work), so we just wait out their settle budgets.
    // Reuse BELT_RESET_TIMEOUT_MS as the generic setup-home ceiling so a wedged servo timer
    // can never hang the boot path.
    const uint32_t start = millis();
    while (!(grabber.isDone() && sorter.isDone() && dumper.isDone())
           && (uint32_t)(millis() - start) < Config::BELT_RESET_TIMEOUT_MS)
    {
        delay(2);
    }
    if (!(grabber.isDone() && sorter.isDone() && dumper.isDone()))
    {
        // Servo completion is a software timer estimate, not a hardware confirmation, so a
        // settle timeout here is non-fatal — warn and continue. begin()'s success/failure is
        // governed by the belt, which has real feedback.
        Serial.println(F("[Manipulator] begin(): servo home settle timed out"));
    }

    // Belt configures the motor and stall-homes into its end-stop, blocking-bounded inside
    // ManipBelt::begin() (it self-bounds with BELT_RESET_TIMEOUT_MS and plain delay for the
    // same pre-tick-callback reason). Its bool is the sole failable init here.
    const bool beltOk = belt.begin();
    return beltOk;
}

void ManipulatorClass::update()
{
    // Only the belt has a non-blocking state machine to advance; the three servos are pure
    // software timers (isDone() reads the timer directly) and need no per-tick work.
    belt.update();
}

bool ManipulatorClass::isDone() const
{
    return grabber.isDone() && sorter.isDone() && dumper.isDone() && belt.isDone();
}

void ManipulatorClass::setTickCallback(void (*cb)())
{
    _tickCallback = cb;
}

// -----------------------------------------------------------------------------------------
// Drain machinery (RAII wait for the blocking-by-default verbs)
// -----------------------------------------------------------------------------------------

bool ManipulatorClass::_maskDone(uint8_t mask) const
{
    // Check ONLY the actuators named in `mask` — an unmasked actuator's state is ignored so a
    // verb that touched one joint does not block on an unrelated in-flight one. An empty mask
    // is vacuously done (the && of an empty set), so the drain is an immediate no-op.
    if ((mask & M_GRAB) && !grabber.isDone())
    {
        return false;
    }
    if ((mask & M_SORT) && !sorter.isDone())
    {
        return false;
    }
    if ((mask & M_DUMP) && !dumper.isDone())
    {
        return false;
    }
    if ((mask & M_BELT) && !belt.isDone())
    {
        return false;
    }
    return true;
}

void ManipulatorClass::_drain(uint8_t mask)
{
    // Empty mask → nothing was commanded; the wait is an immediate no-op (mirrors the
    // already-done semantics of sortTo()'s out-of-range guard).
    if (mask == 0)
    {
        return;
    }

    // Sanctioned blocking site (mirrors Drive::_drainBlocking). The blocking caller is off the
    // cooperative tick chain by definition, so this spin does not violate the non-blocking
    // update()-chain iron rule. Every module must keep ticking during the wait — otherwise
    // EventTrace stops rendering, the belt's own update() stops advancing, etc. — so we invoke
    // the registered cooperative-tick callback each iteration; absent one we degrade to ticking
    // only this module's update() and warn once.
    //
    // Bounded-spin guard: a wedged servo timer or a stalled belt must never trap the mission.
    // The ceiling is Config::BELT_HOME_TIMEOUT_MS — the longest a single masked actuator can
    // legitimately take is the belt's full home sequence (stall + reset + drive to MID, which
    // reset() fires), so the ceiling must cover BOTH stages; by then the belt's own watchdogs
    // have forced its state machine to IDLE and the servo timers have long since elapsed, so a
    // healthy drain converges well inside it. On expiry we stop the belt (a safe brake via a
    // zero-speed run, finalised on the next update()) and warn so a stuck actuator can never hang
    // the drain indefinitely.
    static bool noCallbackWarned = false;
    const uint32_t deadline = millis() + (uint32_t) Config::BELT_HOME_TIMEOUT_MS;

    while (!_maskDone(mask))
    {
        if ((int32_t)(millis() - deadline) >= 0)
        {
            Serial.println(F("[Manipulator] _drain: timeout — stopping belt"));
            belt.runFor(0, 0);    // zero-speed run → setSpeed(0); update() brakes + idles it
            return;
        }

        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            if (!noCallbackWarned)
            {
                Serial.println(F("[Manipulator] _drain: no tick callback set — only "
                                 "Manipulator.update() will tick during blocks. Call "
                                 "Manipulator.setTickCallback(...) in setup()."));
                noCallbackWarned = true;
            }
            update();
        }
        delay(2);
    }
}
