/**
 * @file Manipulator.cpp
 * @brief Out-of-line members of `ManipulatorClass` — the manipulator-subsystem facade.
 *
 * @note This translation unit owns the manipulator implementation surface: the `ManipAction`
 *       builder, the semantic verbs (`grab`, `release`, `dump`, `dumpReset`, `sortTo`, `beltTo`,
 *       `reset`, `boxOpen`, `boxClose`), the `_action` factory, and the lifecycle /
 *       blocking-spin members.
 *
 * @note `<Arduino.h>` (millis / delay / Serial) is pulled in transitively through
 *       `Manipulator.h` -> `ManipServo.h` / `ManipBelt.h`, so it is not re-included here.
 */

#include "Manipulator.h"
#include "ManipBelt.h"
#include "ManipServo.h"

/// @brief Single global instance declared `extern` in Manipulator.h.
ManipulatorClass Manipulator;

// -----------------------------------------------------------------------------------------
// ManipAction builder
// -----------------------------------------------------------------------------------------

ManipAction::ManipAction(ManipulatorClass& m, uint8_t mask) : _m(m), _mask(mask)
{
}

ManipAction::ManipAction(ManipAction&& other) noexcept
    : _m(other._m), _mask(other._mask), _blocking(other._blocking), _executed(other._executed)
{
    other._executed = true;
}

ManipAction& ManipAction::async()
{
    _blocking = false;
    return *this;
}

ManipAction& ManipAction::rampMs(uint16_t ms)
{
    // The verb already armed the move at the servo default rate; retarget the pacing of every
    // servo in the mask before the end-of-statement drain (no update() fires in between).
    _m._applyStepMs(_mask, ms);
    return *this;
}

ManipAction& ManipAction::instant()
{
    return rampMs(0);
}

ManipAction::~ManipAction()
{
    if (_executed)
    {
        return;
    }
    _executed = true;
    if (_blocking)
    {
        _m._drain(_mask);
    }
}

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
// Semantic verbs
// -----------------------------------------------------------------------------------------

ManipAction ManipulatorClass::grab()
{
    // Open fully to home first, THEN close — a self-contained capture stroke, so a grab always
    // runs the full open->close regardless of the grabber's prior pose. The two moves are a
    // fire-and-forget two-stop sequence (ManipServo::homeThenClose): home fires now, close is
    // queued and fired by update() once home settles. The returned action drains on M_GRAB, whose
    // isDone() stays false until the SECOND move settles — so a blocking grab() waits for the whole
    // stroke, and grab().async() returns immediately while update() advances it.
    grabber.homeThenClose();
    return _action(M_GRAB);
}

ManipAction ManipulatorClass::release()
{
    grabber.open();
    return _action(M_GRAB);
}

ManipAction ManipulatorClass::dump()
{
    dumper.open();
    return _action(M_DUMP);
}

ManipAction ManipulatorClass::dumpReset()
{
    dumper.home();
    return _action(M_DUMP);
}

ManipAction ManipulatorClass::sortTo(uint8_t bin)
{
    if (bin >= Config::SORTER_BIN_COUNT)
    {
        Serial.print(F("[Manipulator] sortTo: bin out of range: "));
        Serial.println((int)bin);
        return _action(0);    // empty mask → drain is an immediate no-op
    }
    sorter.setAngle(Config::SORTER_BIN_ANGLE[bin]);
    return _action(M_SORT);
}

ManipAction ManipulatorClass::reset()
{
    grabber.home();
    sorter.home();
    dumper.home();
    belt.home();
    return _action(M_GRAB | M_SORT | M_DUMP | M_BELT);
}

ManipAction ManipulatorClass::beltTo(Config::BeltStation station)
{
    // Belt analogue of sortTo: fire the station move now, wait on the belt unless .async().
    // toStation() bounds-checks the station and warns on an out-of-range value.
    belt.toStation(station);
    return _action(M_BELT);
}

ManipAction ManipulatorClass::boxOpen()
{
    //Move belt to open the rear box
    belt.toStation(Config::BeltStation::REAR_BOX_OPEN);
    return _action(M_BELT);
}

ManipAction ManipulatorClass::boxClose()
{
    //Move belt to close the rear box
    belt.toStation(Config::BeltStation::REAR_BOX_CLOSE);
    return _action(M_BELT);
}

ManipAction ManipulatorClass::_action(uint8_t mask)
{
    return ManipAction(*this, mask);
}

void ManipulatorClass::_applyStepMs(uint8_t mask, uint16_t stepMs)
{
    // Retarget only the SERVO joints the verb touched — the belt is not step-paced this way.
    if (mask & M_GRAB)
    {
        grabber.setStepMs(stepMs);
    }
    if (mask & M_SORT)
    {
        sorter.setStepMs(stepMs);
    }
    if (mask & M_DUMP)
    {
        dumper.setStepMs(stepMs);
    }
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
    // ManipBelt::begin() (it self-bounds with BELT_HOME_TIMEOUT_MS and plain delay for the
    // same pre-tick-callback reason). Its bool is the sole failable init here.
    const bool beltOk = belt.begin();
    return beltOk;
}

void ManipulatorClass::update()
{
    // Advance each servo's optional two-stop sequence (e.g. grab()'s home->close) — update() is a
    // no-op for a servo with no queued follow-up. Then advance the belt's non-blocking state
    // machine. Servos without a pending follow-up cost only a timer compare here.
    grabber.update();
    sorter.update();
    dumper.update();
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
    // The ceiling depends on whether the belt is in the mask. If it is, the longest a single
    // masked actuator can legitimately take is the belt's full home sequence (stall + reset +
    // settle + drive to MID, which home() chains), so the ceiling is Config::BELT_HOME_TIMEOUT_MS
    // to cover all of those phases. A servo-only drain (no belt in the mask) is sized to each
    // masked servo's own estimated remaining move time: an INSTANT move settles inside the
    // Config::MANIP_SERVO_DRAIN_TIMEOUT_MS floor, while a `stepMs > 0` ramp (e.g. a slowed
    // grabber) can legitimately crawl for longer, so the ceiling grows to cover it
    // (worst servo budget + Config::MANIP_SERVO_DRAIN_MARGIN_MS). By the deadline the belt's own
    // watchdogs have forced its state machine to IDLE and the servo ramps/timers have elapsed, so
    // a healthy drain converges well inside it. On expiry we stop the belt (a safe brake via a
    // zero-speed run, finalised on the next update()) and warn so a stuck actuator can never hang
    // the drain indefinitely.
    static bool noCallbackWarned = false;
    // A belt-masked drain uses the belt-home ceiling (its home() chains stall+reset+MID). A
    // servo-only drain derives its ceiling from the masked servos' own estimated remaining move
    // time: a `stepMs > 0` ramp (e.g. the grabber) can legitimately run far longer than the
    // MANIP_SERVO_DRAIN_TIMEOUT_MS floor, so use max(floor, worst servo budget + margin) — an
    // instant move stays capped at the floor, a slow ramp gets exactly the time it needs.
    uint32_t ceiling;
    if (mask & M_BELT)
    {
        ceiling = (uint32_t) Config::BELT_HOME_TIMEOUT_MS;
    }
    else
    {
        uint32_t servoBudget = 0;
        if (mask & M_GRAB)
        {
            const uint32_t b = grabber.estimatedRemainingMs();
            if (b > servoBudget) { servoBudget = b; }
        }
        if (mask & M_SORT)
        {
            const uint32_t b = sorter.estimatedRemainingMs();
            if (b > servoBudget) { servoBudget = b; }
        }
        if (mask & M_DUMP)
        {
            const uint32_t b = dumper.estimatedRemainingMs();
            if (b > servoBudget) { servoBudget = b; }
        }
        ceiling = servoBudget + (uint32_t) Config::MANIP_SERVO_DRAIN_MARGIN_MS;
        if (ceiling < (uint32_t) Config::MANIP_SERVO_DRAIN_TIMEOUT_MS)
        {
            ceiling = (uint32_t) Config::MANIP_SERVO_DRAIN_TIMEOUT_MS;
        }
    }
    const uint32_t deadline = millis() + ceiling;

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
        // The spin must not block with a fixed delay — yield() services the R4 background
        // (USB/Wi-Fi) cooperatively, matching LineFollower::waitUntilDistanceReached and
        // Drive::_drainBlocking. Wall-clock progress comes from the registered tick callback
        // (which advances real millis() via the modules it spins), not from sleeping here.
        yield();
    }
}
