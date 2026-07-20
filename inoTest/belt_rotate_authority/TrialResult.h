/**
 * @file TrialResult.h
 * @brief One trial's measured outcome for `belt_rotate_authority.ino`.
 *
 * Lives in its own header purely because of the Arduino build: the IDE auto-generates
 * function prototypes and splices them in directly after the sketch's last preprocessor
 * directive — i.e. ABOVE any type declared in the `.ino` body. A helper returning or taking
 * this struct therefore fails to compile unless the type arrives via an `#include`.
 */

#ifndef TRIAL_RESULT_H
#define TRIAL_RESULT_H

#include <Arduino.h>

/**
 * @brief Outcome of one probe trial.
 *
 * @note The headline number is `travelDeg` against `commandedDeg`, NOT an average speed.
 *       An earlier revision of this probe reported mean deg/s over the whole watchdog
 *       window, which cannot distinguish a move that crawls from one that sprints and then
 *       quits — the belt does the latter, and the mean hid it. `movingMs` (how long the belt
 *       was actually turning) is what separates the two.
 */
struct TrialResult
{
    uint16_t commandedDeg  = 0;       ///< `degree` argument handed to `rotateFor`.
    int8_t   commandSign   = +1;      ///< Sign of the speed handed to `rotateFor`.
    int32_t  travelDeg     = 0;       ///< Signed encoder travel actually delivered.
    bool     completed     = false;   ///< `ChkRotateEnd` latched before the window closed.
    uint32_t elapsedMs     = 0;       ///< Total window the trial occupied.
    uint32_t movingMs      = 0;       ///< Time from command until motion last seen.
    float    peakDegPerSec = 0.0f;    ///< Fastest sampled interval — the available authority.
};

#endif    // TRIAL_RESULT_H
