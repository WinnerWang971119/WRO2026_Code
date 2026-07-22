/**
 * @file Manipulator.h (CTPS HOST MOCK)
 * @brief Recording mock of the Manipulator facade for the CTPS dump.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `Manipulator.h` (quote-included; force-included by the T4
 * cmake wiring). The include guard is the REAL one (`MANIPULATOR_H`) so the real
 * header no-ops if reached afterwards.
 *
 * Mirrors the production verb surface so a real task body that actuates the
 * manipulator links and records against this mock unchanged:
 *   - Semantic verbs (`grab` / `release` / `dump` / `sortTo` / `reset` / `boxOpen` /
 *     `boxClose`) return a no-op `ManipAction` builder (the recording happens in the
 *     verb body, so an `.async()` chain or a bare statement both record exactly once).
 *     Each is captured as a `manip` op (method name + arg; the arg-less verbs record
 *     `0`, `sortTo` records the bin) into the shared `g_ctpsLog`, preserving execution
 *     order relative to surrounding `task_start` / `drive` / `task_end` records.
 *   - The primitive actuator handles (`grabber` / `sorter` / `dumper` / `belt`)
 *     are exposed as no-op stubs — NOT recording seams. The competition mission
 *     drives the manipulator through the verbs above; only bring-up task bodies
 *     (`servoTest` / `home`) poke a raw joint, and they are never part of a
 *     recorded run. But the dump compiles the WHOLE of `Tasks.cpp` as one TU, so
 *     those handles must exist to link — hence the `ManipServo` / `ManipBelt`
 *     stubs below (guarded with the production guards so the real, `MiniR4`-bound
 *     actuator headers no-op if reached). They record nothing.
 *
 * The `setLift` / `setGripper` / `setArm` setters are FIXTURE-ONLY: they do not
 * exist on the production facade. They are kept because the `ctps_dump`
 * self-test (`recordManipDriveSegment`) drives them to synthesise `manip` ops
 * (AC4) without depending on any registered task body.
 */

#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <stdint.h>

#include "ctps_record.h"

// ---------------------------------------------------------------------------
// Raw-joint actuator stubs. Guarded with the PRODUCTION guards (`MANIPSERVO_H` /
// `MANIPBELT_H`) so the real `ManipServo.h` / `ManipBelt.h` (which bind the
// `MiniR4` RC + DC channels) no-op if reached. Bring-up task bodies
// (`servoTest` / `home`) poke these directly; they are pure no-ops and record
// nothing (raw-joint moves are not part of a recorded competition trace).
// ---------------------------------------------------------------------------

#ifndef MANIPSERVO_H
#define MANIPSERVO_H
/** @brief No-op mock of one RC-servo joint (grabber / sorter / dumper). */
class ManipServo
{
public:
    ManipServo() = default;

    void     open() {}
    void     openForWhite() {}
    void     close() {}
    void     home() {}
    void     setAngle(uint16_t deg) { _shadow = deg; }
    void     setAngle(uint16_t deg, uint16_t /*stepMs*/) { _shadow = deg; }
    void     setStepMs(uint16_t /*stepMs*/) {}
    bool     isDone() const { return true; }
    uint16_t commandedAngle() const { return _shadow; }

private:
    uint16_t _shadow = 0;
};
#endif    // MANIPSERVO_H

#ifndef MANIPBELT_H
#define MANIPBELT_H
/** @brief No-op mock of the conveyor-belt actuator. */
class ManipBelt
{
public:
    ManipBelt() = default;

    bool begin() { return true; }
    template<typename T>
    void toStation(T) {}
    void home() {}
    void update() {}
    bool isDone() const { return true; }
};
#endif    // MANIPBELT_H

/**
 * @brief No-op mock of the production `ManipAction` RAII-wait builder.
 *
 * The real builder fires its actuator setpoint in the verb body and waits at
 * end-of-statement; here the verb has already recorded the `manip` op, so this
 * builder only has to satisfy the call-as-temporary surface. Move-only
 * like the real handle (copy + assigns deleted, move ctor kept so
 * the verb can return it by value on pre-C++17 compilers). `async()` is a no-op.
 */
class ManipAction
{
public:
    ManipAction() = default;

    ManipAction(const ManipAction&)            = delete;
    ManipAction& operator=(const ManipAction&) = delete;
    ManipAction& operator=(ManipAction&&)      = delete;
    ManipAction(ManipAction&&)                 = default;

    /** @brief Gate the (mock) wait — no-op; the verb already recorded. */
    ManipAction& async() { return *this; }
    /** @brief Override the (mock) servo step rate — no-op; the verb already recorded. */
    ManipAction& rampMs(uint16_t /*ms*/) { return *this; }
    /** @brief Run the (mock) move at full speed — no-op; the verb already recorded. */
    ManipAction& instant() { return *this; }
};

/**
 * @brief Recording mock of the manipulator facade.
 */
class ManipulatorClass
{
public:
    /** @brief No-op bring-up; always reports success. */
    bool begin() { return true; }

    /** @brief No-op cooperative tick. */
    void update() {}

    /** @brief No-op tick-callback registration (matches the real facade surface). */
    void setTickCallback(void (*)()) {}

    // --- Production semantic verbs (record a `manip` op, return a no-op action) ---

    /** @brief Record a `grab` manip op. */
    ManipAction grab()
    {
        ctps::recordManip("grab", 0);
        return ManipAction{};
    }

    /** @brief Record a `release` manip op. */
    ManipAction release()
    {
        ctps::recordManip("release", 0);
        return ManipAction{};
    }

    /** @brief Record a `dump` manip op. */
    ManipAction dump()
    {
        ctps::recordManip("dump", 0);
        return ManipAction{};
    }

    /** @brief Record a `dumpReset` manip op. */
    ManipAction dumpReset()
    {
        ctps::recordManip("dumpReset", 0);
        return ManipAction{};
    }

    /** @brief Record a `sortTo` manip op carrying the bin index. */
    ManipAction sortTo(uint8_t bin)
    {
        ctps::recordManip("sortTo", bin);
        return ManipAction{};
    }

    /**
     * @brief Record a `beltTo` manip op carrying the station index.
     * @note Templated on the station type (like `ManipBelt::toStation`) so this mock
     *       stays free of a `Config.h` dependency; the scoped-enum station casts to its
     *       underlying index.
     */
    template<typename T>
    ManipAction beltTo(T station)
    {
        ctps::recordManip("beltTo", static_cast<uint8_t>(station));
        return ManipAction{};
    }

    /** @brief Record a `reset` manip op. */
    ManipAction reset()
    {
        ctps::recordManip("reset", 0);
        return ManipAction{};
    }

    /** @brief Record a `boxOpen` manip op. */
    ManipAction boxOpen()
    {
        ctps::recordManip("boxOpen", 0);
        return ManipAction{};
    }

    /** @brief Record a `boxClose` manip op. */
    ManipAction boxClose()
    {
        ctps::recordManip("boxClose", 0);
        return ManipAction{};
    }

    // --- Fixture-only setters (NOT on the production facade; see file docstring) ---

    /** @brief Record a `setLift` manip op (fixture self-test only). */
    void setLift(uint8_t height) { ctps::recordManip("setLift", height); }

    /** @brief Record a `setGripper` manip op (fixture self-test only). */
    void setGripper(uint8_t angle) { ctps::recordManip("setGripper", angle); }

    /** @brief Record a `setArm` manip op (fixture self-test only). */
    void setArm(uint8_t angle) { ctps::recordManip("setArm", angle); }

    /** @brief Always done — the mock motion completes instantly. */
    bool isDone() const { return true; }

    // --- Raw-joint handles (bring-up tasks only; no-op, record nothing) ---

    ManipServo grabber;
    ManipServo sorter;
    ManipServo dumper;
    ManipBelt  belt;
};

/** @brief Single global instance — defined in ctps_record.cpp. */
extern ManipulatorClass Manipulator;

#endif    // MANIPULATOR_H
