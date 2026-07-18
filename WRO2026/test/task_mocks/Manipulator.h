/**
 * @file Manipulator.h (HOST MOCK)
 * @brief Self-contained no-op `ManipulatorClass` mock for the Tasks host harness.
 *
 * Shadows the production `WRO2026/Manipulator.h` (same `MANIPULATOR_H` guard,
 * force-included by the `test_tasks_dispatch` wiring) so a task body that drives
 * the manipulator links against this stub instead of `Manipulator.cpp` (which,
 * with `ManipServo` / `ManipBelt`, binds the `MiniR4` RC + DC channels).
 *
 * Provides minimal no-op mocks of `ManipServo` / `ManipBelt` / `ManipAction`
 * (guarded with the real `MANIPSERVO_H` / `MANIPBELT_H` names so the production
 * actuator headers no-op if reached), then the `ManipulatorClass` facade with
 * its coordinated verbs and public actuator handles. Every verb is a no-op;
 * `isDone()` reports instant completion so a blocking verb advances on host.
 */

// NOTE: SAME include guard (`MANIPULATOR_H`) as the production header.
#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <stdint.h>

#include "Config.h"    // real — Config::BeltStation for ManipBelt::toStation

// ---------------------------------------------------------------------------
// Minimal actuator stubs. Guarded with the PRODUCTION guards so the real
// `ManipServo.h` / `ManipBelt.h` (which pull in `MiniR4`) no-op if reached.
// ---------------------------------------------------------------------------

#ifndef MANIPSERVO_H
#define MANIPSERVO_H
/** @brief No-op mock of one RC-servo joint (grabber / sorter / dumper). */
class ManipServo
{
public:
    ManipServo() = default;

    void     open() {}
    void     close() {}
    void     home() {}
    void     setAngle(uint16_t deg) { _shadow = deg; }
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
    void toStation(Config::BeltStation /*s*/) {}
    void beltTo(int32_t /*absDeg*/) {}
    void runFor(uint32_t /*ms*/, int16_t /*speed*/) {}
    void home() {}
    void update() {}
    bool isDone() const { return true; }
};
#endif    // MANIPBELT_H

/**
 * @brief No-op mock of `ManipAction`.
 *
 * Move-only to match the real coordinated-verb handle; the
 * real dtor drains the action unless `.async()` was called — the mock dtor does
 * nothing.
 */
class ManipAction
{
public:
    ManipAction() = default;
    ManipAction(ManipAction&&) noexcept = default;

    ManipAction(const ManipAction&)            = delete;
    ManipAction& operator=(const ManipAction&) = delete;
    ManipAction& operator=(ManipAction&&)      = delete;

    ManipAction& async() { return *this; }
};

/**
 * @brief No-op mock of the Manipulator facade.
 */
class ManipulatorClass
{
public:
    ManipulatorClass() = default;

    // --- lifecycle ---
    bool begin() { return true; }
    void update() {}
    void setTickCallback(void (*)()) {}

    // --- state query ---
    bool isDone() const { return true; }

    // --- coordinated semantic verbs ---
    ManipAction grab() { return ManipAction{}; }
    ManipAction release() { return ManipAction{}; }
    ManipAction dump() { return ManipAction{}; }
    ManipAction dumpReset() { return ManipAction{}; }
    ManipAction sortTo(uint8_t /*bin*/) { return ManipAction{}; }
    ManipAction beltTo(Config::BeltStation /*station*/) { return ManipAction{}; }
    ManipAction reset() { return ManipAction{}; }
    ManipAction boxOpen() { return ManipAction{}; }
    ManipAction boxClose() { return ManipAction{}; }

    // --- public actuator handles (calibration / bring-up) ---
    ManipServo grabber;
    ManipServo sorter;
    ManipServo dumper;
    ManipBelt  belt;
};

// Single global instance — `inline` (force-included only into `test_tasks_dispatch`).
// See `task_mocks/PathTracker.h` for why this is `inline` rather than `extern` + mocks.cpp.
inline ManipulatorClass Manipulator;

#endif    // MANIPULATOR_H
