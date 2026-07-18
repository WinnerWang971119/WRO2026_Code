/**
 * @file EncCheckHarness.h
 * @brief Declares the encoder-read ground-truth diagnostic.
 * @author WRO 2026
 *
 * @details
 * See `EncCheckHarness.cpp` for the full description. Kept in its own header + `.cpp` pair for the
 * same ctags/auto-prototype reason `TurnProbeHarness.cpp` documents.
 */

#ifndef ENC_CHECK_HARNESS_H
#define ENC_CHECK_HARNESS_H

/**
 * @brief Drive each wheel four ways and print EVERY encoder read, so we can see which one actually
 *        tracks a spinning wheel (per-motor `M2/M3` counter/degrees vs the `DriveDC` group counter,
 *        under grouped `Move` vs per-motor `setPower` drive).
 *
 * Gated on BTN_UP. Deliberately does NOT pump `cooperativeTick()` during the drive windows, so the
 * encoder reads are the only lower-MCU traffic — that isolates whether a stuck read is a grouping
 * issue or bus contention. Watch the robot AND the numbers: for each labelled test, note which
 * encoder columns change while the wheel is visibly turning.
 *
 * @param tickCb Cooperative tick (used only for the BTN_UP wait, not during the timed drives).
 */
void runEncCheck(void (*tickCb)());

#endif    // ENC_CHECK_HARNESS_H
