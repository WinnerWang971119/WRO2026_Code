/**
 * @file MotorMatchHarness.h
 * @brief Declares the spare-motor characterizer + best-pair picker (inoTest/motor_match).
 * @author WRO 2026
 *
 * @details
 * See `MotorMatchHarness.cpp`. Kept in its own header + `.cpp` pair for the ctags/auto-prototype
 * reason `TurnProbeHarness.cpp` documents.
 */

#ifndef MOTOR_MATCH_HARNESS_H
#define MOTOR_MATCH_HARNESS_H

/**
 * @brief Interactive bench routine: plug in one spare motor at a time, spin it at several PWM
 *        levels, record its speed (persisted to EEPROM across resets), then compute the best
 *        MATCHED pair (closest speed at equal power, preferring the faster pair).
 *
 * Runs forever, driven by the two on-board buttons (record / redo / compute / erase). Never
 * returns, so the sketch's `loop()` stays empty.
 *
 * @param tickCb Unused (kept for a uniform signature with the other harnesses).
 */
void runMotorMatch(void (*tickCb)());

#endif    // MOTOR_MATCH_HARNESS_H
