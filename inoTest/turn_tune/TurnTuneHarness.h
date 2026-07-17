/**
 * @file TurnTuneHarness.h
 * @brief Declares the closed-loop turn-accuracy bench harness used to tune the PD turn gains.
 * @author WRO 2026
 *
 * @details
 * See `TurnTuneHarness.cpp` for the full description. This declaration is the only thing
 * `turn_tune.ino` needs. Kept in its own header + `.cpp` pair (not inlined into the `.ino`)
 * for the same reason `turn_probe` is: the harness's file-local helpers live in an anonymous
 * namespace, and Arduino's `.ino`-only ctags auto-prototype pass does not understand internal
 * linkage — it would emit mismatched global-scope forward declarations and the link would
 * fail. A `.cpp` in the sketch folder is compiled as an ordinary translation unit with no such
 * rewriting. See `TurnTuneHarness.cpp`'s header for the full explanation.
 */

#ifndef TURN_TUNE_HARNESS_H
#define TURN_TUNE_HARNESS_H

/**
 * @brief Run the full turn-accuracy sweep once, blocking until every trial has run and the
 *        summary has printed.
 *
 * Gated behind a single `BTN_UP` prompt so the operator can place the chassis in a clear
 * circle first, then runs `TURN_TUNE_TRIALS` in-place turns through the REAL `Drive.turnDeg()`
 * PD path — alternating direction each trial to keep the USB tether from winding up — and
 * reports the resting IMU residual, post-brake coast, and settle time per trial plus a
 * run-summary. The gains in force (`Config::TURN_*`) are echoed at the top of the run so each
 * log records exactly which build produced its numbers.
 *
 * @param tickCb Cooperative-tick function pumped during every blocking wait. Pass
 *               `cooperativeTick` from `turn_tune.ino`: the sync-blocking `Drive.turnDeg()`
 *               spins re-enter it every iteration, so it MUST be the sketch's real
 *               `cooperativeTick()` (the same module set a real nav turn pays), not a partial
 *               substitute, or the measured turn dynamics are not representative.
 */
void runTurnTune(void (*tickCb)());

#endif    // TURN_TUNE_HARNESS_H
