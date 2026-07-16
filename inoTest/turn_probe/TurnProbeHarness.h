/**
 * @file TurnProbeHarness.h
 * @brief Declares the five-phase turn creep-floor / tick-period / IMU-noise bench probe.
 * @author WRO 2026
 *
 * @details
 * See `TurnProbeHarness.cpp` for the full description. This declaration is the only thing
 * `turn_probe.ino` needs: it is kept in its own header + `.cpp` pair (not inlined into the
 * `.ino`) so the probe's file-local helpers can live inside an anonymous namespace without
 * tripping Arduino's `.ino`-only ctags auto-prototype pass — see the `.cpp` file header for
 * the full explanation.
 */

#ifndef TURN_PROBE_HARNESS_H
#define TURN_PROBE_HARNESS_H

/**
 * @brief Run the full five-phase bench probe once, blocking until every phase has run and
 *        the final summary has printed.
 *
 * Phases run (d) IMU noise -> (c) idle tick re-confirm -> (a1) breakaway sweep -> (a2)
 * kinetic sweep -> (b) omega + turn tick, each gated behind its own `BTN_UP` prompt so the
 * operator can reposition the chassis between phases. See `TurnProbeHarness.cpp` for the
 * full phase-by-phase contract.
 *
 * @param tickCb Cooperative-tick function pumped during every blocking wait inside the
 *               probe. Pass `cooperativeTick` from `turn_probe.ino` — phase (b) counts this
 *               callback's own iterations to measure the TURN-representative tick, so it
 *               MUST be the sketch's real `cooperativeTick()`, not a partial substitute.
 */
void runTurnProbe(void (*tickCb)());

#endif    // TURN_PROBE_HARNESS_H
