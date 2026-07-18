/**
 * @file SpeedTestHarness.h
 * @brief Declares the per-motor closed-loop setSpeed test (does it regulate under the enc swap?).
 * @author WRO 2026
 *
 * @details
 * See `SpeedTestHarness.cpp`. Kept in its own header + `.cpp` pair for the ctags/auto-prototype
 * reason `TurnProbeHarness.cpp` documents.
 */

#ifndef SPEED_TEST_HARNESS_H
#define SPEED_TEST_HARNESS_H

/**
 * @brief Command each motor's per-motor closed-loop `setSpeed()` at several levels and measure the
 *        DRIVEN wheel's actual rate (via its correct, crossed encoder), to settle whether per-motor
 *        speed control regulates or saturates given the swapped encoder wiring.
 *
 * If the measured rate scales with the commanded speed, the loop reads correct feedback and works
 * (closed-loop per-wheel is viable). If it saturates at max regardless of the command, the loop is
 * reading the wrong wheel and per-motor closed-loop is unusable.
 *
 * @param tickCb Unused (kept for a uniform signature with the other harnesses).
 */
void runSpeedTest(void (*tickCb)());

#endif    // SPEED_TEST_HARNESS_H
