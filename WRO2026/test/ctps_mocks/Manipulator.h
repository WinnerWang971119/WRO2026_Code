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
 *   - Semantic verbs (`grab` / `release` / `dump` / `sortTo` / `reset`) return a
 *     no-op `ManipAction` builder (the recording happens in the verb body, so an
 *     `.async()` chain or a bare statement both record exactly once). Each is
 *     captured as a `manip` op (method name + arg; the arg-less verbs record `0`,
 *     `sortTo` records the bin) into the shared `g_ctpsLog`, preserving execution
 *     order relative to surrounding `task_start` / `drive` / `task_end` records.
 *   - The primitive actuator handles (`grabber` / `sorter` / `dumper` / `belt`)
 *     are intentionally NOT mocked: production task bodies use the verbs, only
 *     calibration / bring-up code pokes a raw joint, and pulling them in would
 *     drag `ManipServo` / `ManipBelt` / `Config` into the dump TU for no gain.
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

/**
 * @brief No-op mock of the production `ManipAction` RAII-wait builder.
 *
 * The real builder fires its actuator setpoint in the verb body and waits at
 * end-of-statement; here the verb has already recorded the `manip` op, so this
 * builder only has to satisfy the call-as-temporary surface. Move-only and
 * `[[nodiscard]]` like the real handle (copy + assigns deleted, move ctor kept so
 * the verb can return it by value on pre-C++17 compilers). `async()` is a no-op.
 */
class [[nodiscard]] ManipAction
{
public:
    ManipAction() = default;

    ManipAction(const ManipAction&)            = delete;
    ManipAction& operator=(const ManipAction&) = delete;
    ManipAction& operator=(ManipAction&&)      = delete;
    ManipAction(ManipAction&&)                 = default;

    /** @brief Gate the (mock) wait — no-op; the verb already recorded. */
    ManipAction& async() { return *this; }
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

    /** @brief Record a `sortTo` manip op carrying the bin index. */
    ManipAction sortTo(uint8_t bin)
    {
        ctps::recordManip("sortTo", bin);
        return ManipAction{};
    }

    /** @brief Record a `reset` manip op. */
    ManipAction reset()
    {
        ctps::recordManip("reset", 0);
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
};

/** @brief Single global instance — defined in ctps_record.cpp. */
extern ManipulatorClass Manipulator;

#endif    // MANIPULATOR_H
