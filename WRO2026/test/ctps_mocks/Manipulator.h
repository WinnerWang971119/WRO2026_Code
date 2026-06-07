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
 * Records the three manipulator commands as `manip` ops (method + arg) into the
 * shared `g_ctpsLog`, preserving execution order relative to surrounding
 * `task_start` / `drive` / `task_end` records. There is NO `grab()` / `release()`
 * — the real `ManipulatorClass` exposes only `begin` / `update` / `setLift` /
 * `setGripper` / `setArm` / `isDone`.
 *
 * NOTE: the real `Mission.cpp` / `Tasks.cpp` do not include this header — it is
 * needed only by the AC4 fixture task TU (T4) that drives the manipulator. It is
 * mocked here so that fixture compiles + records against the recording sink.
 */

#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <stdint.h>

#include "ctps_record.h"

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

    /** @brief Record a `setLift` manip op. */
    void setLift(uint8_t height) { ctps::recordManip("setLift", height); }

    /** @brief Record a `setGripper` manip op. */
    void setGripper(uint8_t angle) { ctps::recordManip("setGripper", angle); }

    /** @brief Record a `setArm` manip op. */
    void setArm(uint8_t angle) { ctps::recordManip("setArm", angle); }

    /** @brief Always done — the mock motion completes instantly. */
    bool isDone() const { return true; }
};

/** @brief Single global instance — defined in ctps_record.cpp. */
extern ManipulatorClass Manipulator;

#endif    // MANIPULATOR_H
