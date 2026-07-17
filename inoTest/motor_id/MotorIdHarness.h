/**
 * @file MotorIdHarness.h
 * @brief Declares the drive-each-motor identify test (power channel -> wheel -> encoder).
 * @author WRO 2026
 *
 * @details
 * See `MotorIdHarness.cpp`. Kept in its own header + `.cpp` pair for the ctags/auto-prototype
 * reason `TurnProbeHarness.cpp` documents.
 */

#ifndef MOTOR_ID_HARNESS_H
#define MOTOR_ID_HARNESS_H

/**
 * @brief Drive motor channel M2, then M3 (per-motor `setPower`, no DriveDC group), and report
 *        which encoder channel responds to each — so it cross-checks the power->wheel->encoder
 *        chain against the hand-spin result (left wheel = M3 enc, right = M2 enc).
 *
 * @param tickCb Unused (kept for a uniform signature with the other harnesses).
 */
void runMotorId(void (*tickCb)());

#endif    // MOTOR_ID_HARNESS_H
