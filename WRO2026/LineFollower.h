/**
 * @file LineFollower.h
 * @brief Outer P+D line-following controller riding on top of `Drive.setSpeed`.
 * @author WRO 2026
 *
 * @details
 * `LineFollowerClass` is the singleton that converts the weighted error reported by the
 * MATRIX Line Tracer 10CH sensor (`MatrixLineTracer::getError()`, range `[-4.5, +4.5]`) into
 * differential left/right PWM commands sent to `Drive.setSpeed(L, R)`. It is deliberately the
 * *outer* loop: it commands raw PWM, NOT `MoveSync`, so the lower-MCU's wheel-balance PID does
 * not stack on top of this one and oscillate (see `Drive::setSpeed` doxygen).
 *
 * Tuning is sourced exclusively from `Config::LINE_KP`, `Config::LINE_KI` (currently 0 — see
 * `update()` note), `Config::LINE_KD`, `Config::LINE_MAX_CORR`, `Config::CRUISE_SPEED`, and
 * `Config::LINE_THRESHOLD`. Hardware wiring is sourced from `Config::LINE_I2C_PORT`. Lost-line
 * behaviour is dispatched on `Config::LOST_LINE_ACTION`.
 *
 * The owned `MatrixLineTracer _tracer` instance is exposed through `getTracer()` so that the
 * Intersection module (T7) can read junction state from the *same* sensor without issuing
 * duplicate I2C transactions.
 *
 * Lifecycle:
 *  - `begin()` once from `setup()` after `MiniR4.begin()` and `Drive.begin()`.
 *  - `update()` every iteration of the main `loop()`. Non-blocking. Short-circuits if disabled.
 *  - `setEnabled(true)` to start following; `setEnabled(false)` to stop and brake.
 *
 * @note When `setEnabled(false)` is called, the chassis is actively braked and the
 *       derivative-term history (`_lastErr`) is cleared so the D-term does not carry into the
 *       next enable cycle.
 */

#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>
#include <MatrixMiniR4.h>            // brings in Wire1
#include "MiniR4_MXLineTracer.h"     // MatrixLineTracer
#include "Config.h"

/**
 * @brief Singleton outer-loop line follower over `MatrixLineTracer` and `Drive.setSpeed`.
 *
 * Owns one `MatrixLineTracer` instance, runs a P+D controller every `update()` tick, and
 * drives the chassis at `Config::CRUISE_SPEED ± corr`. The integral term is intentionally
 * absent in Phase 1 — see `update()` doxygen.
 */
class LineFollowerClass
{
public:
    /**
     * @brief Wire up the tracer to Wire1 + the configured mux channel and initialise it.
     *
     * Sets `_tracer._pWire = &Wire1` and `_tracer._ch = Config::LINE_I2C_PORT`, calls
     * `_tracer.begin()`, then pushes `Config::LINE_THRESHOLD` via `setThreshold()`. Resets the
     * controller state (`_enabled`, `_lostLine`, `_lastErr`).
     *
     * @return `true` (the underlying `MatrixLineTracer::begin()` returns void; we surface a
     *         `bool` for symmetry with the other `*::begin()` methods in this codebase).
     * @note Must be called from `setup()` AFTER `MiniR4.begin()` so `Wire1` is alive.
     */
    bool begin();

    /**
     * @brief Cooperative tick — call every iteration of the main `loop()`.
     *
     * Behaviour:
     *  - If `!_enabled`, returns immediately (short-circuit).
     *  - If `!_tracer.isOnline()` (line lost beneath every channel), sets `_lostLine = true`
     *    and dispatches `Config::LOST_LINE_ACTION`:
     *      - `STOP`  → `Drive.stop(true)`  — active brake.
     *      - `COAST` → `Drive.stop(false)` — release power, roll to a stop.
     *      - `CRAWL` → `Drive.setSpeed(CRUISE_SPEED/2, CRUISE_SPEED/2)` — straight creep
     *                  forward (no steering) so the strip can be reacquired.
     *  - Otherwise (line online): clears `_lostLine`, samples `_tracer.getError()`, runs a P+D
     *    controller `corr = LINE_KP*err + LINE_KD*(err - _lastErr)`, clamps `corr` to
     *    `[-LINE_MAX_CORR, +LINE_MAX_CORR]`, then commands
     *    `Drive.setSpeed(CRUISE_SPEED + corr, CRUISE_SPEED - corr)` with each channel
     *    re-clamped to `[-100, +100]` (the DriveDC PWM range).
     *
     * @note Non-blocking. Contains NO `delay()`. Safe to call at any cadence.
     * @note Phase 1 deliberately omits the integral term — `Config::LINE_KI` is unused. If
     *       integral action is needed later, add an `_errSum` accumulator with appropriate
     *       windup guard (clamp `_errSum` and/or only integrate when `|err|` is small).
     */
    void update();

    /**
     * @brief Enable or disable closed-loop line following.
     *
     * @param on `true` to start following on the next `update()`; `false` to stop now.
     * @note Disabling immediately calls `Drive.stop(true)` (active brake) and resets
     *       `_lastErr = 0.0f` so the D-term does not jump on the next enable. The lost-line
     *       flag is intentionally NOT cleared here — callers (e.g. EventTrace) may still want
     *       to react to the trailing edge of `isLineLost()`.
     */
    void setEnabled(bool on);

    /**
     * @brief Whether closed-loop line following is currently active.
     * @return `true` if `update()` will run the controller; `false` if it short-circuits.
     */
    bool isEnabled() const;

    /**
     * @brief Whether the line was lost beneath every channel on the most recent `update()`.
     * @return `true` if the controller is in the lost-line branch.
     * @note Polled by EventTrace (T8) to push `LINE LOST` / `LINE OK` transitions.
     */
    bool isLineLost() const;

    /**
     * @brief Most recent weighted error sample stored by the controller.
     *
     * @return The last value returned by `MatrixLineTracer::getError()` while the line was
     *         online (range `[-4.5, +4.5]`). Reset to `0.0f` by `begin()`, by
     *         `setEnabled(false)`, and on every entry to the lost-line branch of `update()` —
     *         so a caller observing the value immediately after the line is dropped will see
     *         `0.0f`, not a stale sample.
     * @note Useful as telemetry for OLED tuning views.
     */
    float getRawError() const;

    /**
     * @brief Pointer to the owned `MatrixLineTracer` so the Intersection module can share it.
     *
     * Intersection (T7) calls this from its own `begin()` to bind onto the *same* sensor
     * instance — duplicating the driver would issue duplicate I2C transactions on the same
     * Wire1 mux channel.
     *
     * @return Non-owning pointer to the tracer. Lifetime equals this singleton's lifetime.
     */
    MatrixLineTracer* getTracer();

private:
    MatrixLineTracer _tracer;            ///< Owned sensor driver shared with Intersection (T7).
    bool             _enabled  = false;  ///< When false, `update()` is a no-op.
    bool             _lostLine = false;  ///< Last-tick lost-line state (no channel online).
    float            _lastErr  = 0.0f;   ///< Previous error sample, for the D-term.
};

/**
 * @brief Single global instance — defined in LineFollower.cpp.
 */
extern LineFollowerClass LineFollower;

#endif    // LINEFOLLOWER_H
