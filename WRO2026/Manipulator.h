#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <Arduino.h>
#include "Config.h"

/**
 * @brief Phase 1 placeholder for the robot's manipulator subsystem.
 *
 * This class reserves the module slot and defines the public API that the rest of the
 * codebase (StateMachine, mission runners, etc.) will call in Phase 3.  The underlying
 * mechanism — lift geometry, gripper design, arm kinematics, servo/motor mapping — is
 * **undecided** and is tracked in DECISIONS.md.  Until the physical mechanism is built
 * and those decisions are recorded, every method in this class is a no-op stub that
 * compiles cleanly and returns safe sentinel values.
 *
 * When Phase 3 begins:
 *  1. Identify the hardware assigned to lift, gripper, and arm in DECISIONS.md.
 *  2. Add the required `MiniR4.RC*` / `MiniR4.M*` calls inside each method body.
 *  3. Remove the Phase 3 `@todo` notes from this header once fully implemented.
 *
 * @note Do **not** add hardware calls here until the mechanism is finalized; premature
 *       abstraction will be ripped out and rewritten anyway.
 */
class ManipulatorClass
{
public:
    /**
     * @brief Initialize the manipulator subsystem.
     *
     * In Phase 3 this will configure servo/motor directions, set safe home positions,
     * and verify hardware readiness.  Currently a no-op that always reports success so
     * that `setup()` can call it unconditionally without branching.
     *
     * @return `true` always (stub).
     *
     * @todo Phase 3 — implement after mechanism finalized: configure RC servos / DC
     *       motors, command home position, check MMLower::RESULT for errors.
     */
    bool begin();

    /**
     * @brief Periodic update tick — call from `loop()` once per iteration.
     *
     * In Phase 3 this will advance any non-blocking motion sequences (e.g. profiled
     * lifts, timed gripper open/close) and update internal state flags consumed by
     * `isDone()`.  Currently a no-op.
     *
     * @todo Phase 3 — implement after mechanism finalized: drive any async motion
     *       profiles and refresh the done-flag.
     */
    void update();

    /**
     * @brief Command the lift to a target height index or millimetre offset.
     *
     * The meaning of @p height (raw encoder count, millimetre value, or discrete level
     * index) depends on the physical design and will be specified in DECISIONS.md during
     * Phase 3.  Currently a no-op; the parameter is suppressed to avoid compiler
     * warnings.
     *
     * @param height  Target height (units TBD in Phase 3).
     *
     * @todo Phase 3 — implement after mechanism finalized: translate @p height to servo
     *       PWM / motor encoder target and start the motion.
     */
    void setLift(uint8_t height);

    /**
     * @brief Command the gripper to a target angle.
     *
     * Full-open and full-close angles will be calibrated constants in Config.h once the
     * mechanism is built.  Currently a no-op; the parameter is suppressed to avoid
     * compiler warnings.
     *
     * @param angle  Target servo angle in degrees (0–180, meaning TBD in Phase 3).
     *
     * @todo Phase 3 — implement after mechanism finalized: write @p angle to the
     *       assigned `MiniR4.RC*` servo channel.
     */
    void setGripper(uint8_t angle);

    /**
     * @brief Command the arm to a target angle.
     *
     * Joint range and zero reference will be established once the arm geometry is
     * decided.  Currently a no-op; the parameter is suppressed to avoid compiler
     * warnings.
     *
     * @param angle  Target servo angle in degrees (0–180, meaning TBD in Phase 3).
     *
     * @todo Phase 3 — implement after mechanism finalized: write @p angle to the
     *       assigned `MiniR4.RC*` servo channel and update the done-flag accordingly.
     */
    void setArm(uint8_t angle);

    /**
     * @brief Query whether the last commanded motion has completed.
     *
     * In Phase 3 this will return `false` while an async motion (lift travel, gripper
     * squeeze, arm swing) is in progress and `true` once the mechanism reaches its
     * target.  Currently always returns `true` so that callers do not block waiting for
     * a stub.
     *
     * @return `true` always (stub).
     *
     * @todo Phase 3 — implement after mechanism finalized: return the real in-motion
     *       flag driven by `update()`.
     */
    bool isDone() const;
};

/** @brief Singleton instance — include this header and call `Manipulator.begin()` in `setup()`. */
extern ManipulatorClass Manipulator;

#endif    // MANIPULATOR_H
