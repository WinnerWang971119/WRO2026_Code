/**
 * @file EncIdHarness.h
 * @brief Declares the hand-spin encoder identify test (definitive wheel -> encoder mapping).
 * @author WRO 2026
 *
 * @details
 * See `EncIdHarness.cpp`. Kept in its own header + `.cpp` pair for the ctags/auto-prototype reason
 * `TurnProbeHarness.cpp` documents.
 */

#ifndef ENC_ID_HARNESS_H
#define ENC_ID_HARNESS_H

/**
 * @brief With the motors OFF, continuously print both encoder counters and flag which one moves,
 *        so spinning a wheel BY HAND directly reveals that wheel's encoder channel.
 *
 * No motor is ever driven and no DriveDC/config is involved — this is the pure physical
 * wheel -> encoder wiring, self-verified by the operator. Runs forever (RESET to stop).
 *
 * @param tickCb Unused (kept for a uniform signature with the other harnesses).
 */
void runEncId(void (*tickCb)());

#endif    // ENC_ID_HARNESS_H
