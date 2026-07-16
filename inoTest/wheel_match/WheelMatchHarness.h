/**
 * @file WheelMatchHarness.h
 * @brief Declares the lifted wheel-match test (weight vs mechanical drag).
 * @author WRO 2026
 *
 * @details
 * See `WheelMatchHarness.cpp` for the full description. Kept in its own header + `.cpp` pair for
 * the same ctags/auto-prototype reason `TurnProbeHarness.cpp` documents.
 */

#ifndef WHEEL_MATCH_HARNESS_H
#define WHEEL_MATCH_HARNESS_H

/**
 * @brief With the robot LIFTED (wheels off the ground, spinning free), drive both wheels at a
 *        series of matched PWM levels and compare their encoder rates.
 *
 * Unloaded, both wheels see no ground friction, so any rate difference is the motor + drivetrain
 * itself. Matched rates -> the on-ground turn imbalance is the WEIGHT loading the heavy wheel
 * (compensate in software). A persistently slower left even lifted -> mechanical DRAG on the left
 * (a tight bearing / rub / misalignment), better fixed at the mechanism. Reads the correct,
 * swapped encoder channel per wheel (left -> M3, right -> M2; see enc_check).
 *
 * @param tickCb Cooperative tick, pumped during the drives and the BTN_UP wait.
 */
void runWheelMatch(void (*tickCb)());

#endif    // WHEEL_MATCH_HARNESS_H
