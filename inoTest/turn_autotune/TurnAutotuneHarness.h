/**
 * @file TurnAutotuneHarness.h
 * @brief Declares the on-robot Twiddle auto-tuner for the turn PD gains (plan T13).
 * @author WRO 2026
 *
 * @details
 * See `TurnAutotuneHarness.cpp` for the full description. This declaration is the only thing
 * `turn_autotune.ino` needs. Kept in its own header + `.cpp` pair (not inlined into the `.ino`)
 * for the same ctags/auto-prototype reason `TurnProbeHarness.cpp` documents — see that header.
 */

#ifndef TURN_AUTOTUNE_HARNESS_H
#define TURN_AUTOTUNE_HARNESS_H

/**
 * @brief Run the on-robot PD gain search once, blocking until it converges, hits the iteration
 *        cap, or the operator aborts with BTN_DOWN.
 *
 * Gated behind a single `BTN_UP` prompt so the operator can place the chassis in a clear circle.
 * Runs a Twiddle (coordinate-ascent) search over the turn PD gains: each candidate gain vector is
 * scored by running N alternating +/- turns through a faithful RUNTIME-gain replica of
 * `Drive::_updateTurnProfile` (the same `TurnPDPolicy.h` / `TurnSignPolicy.h` decision math
 * production uses, driving `MiniR4.DriveDC.Move` directly), and the search keeps the vector that
 * minimises a weighted cost of residual + overshoot + settle-time + timeout penalty. `Drive.cpp`
 * is never modified — the winning gains are printed for the operator to paste into
 * `WRO2026/config/ConfigDrive.h`, then validated on the real `Drive.turnDeg()` path with
 * `inoTest/turn_tune`.
 *
 * @param tickCb Cooperative-tick function pumped once per replica turn-loop iteration (and during
 *               every blocking wait). Pass `cooperativeTick` from `turn_autotune.ino`: pumping the
 *               full module set each iteration makes the replica's tick period match a real nav
 *               turn's, so the time-normalized `Kd` (and thus the tuned gains) transfer. It MUST
 *               be the sketch's real `cooperativeTick()`, not a partial substitute.
 */
void runTurnAutotune(void (*tickCb)());

#endif    // TURN_AUTOTUNE_HARNESS_H
