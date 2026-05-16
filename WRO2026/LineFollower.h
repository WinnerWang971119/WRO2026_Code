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
 * `update()` note), `Config::LINE_KD`, `Config::LINE_MAX_CORR`, the active speed preset
 * (resolved by `_presetToPwm()` from `Config::SPEED_SLOW/NORMAL/FAST`), and
 * `Config::LINE_THRESHOLD`. Hardware wiring is sourced from `Config::LINE_I2C_PORT`. Lost-line
 * behaviour is dispatched on `Config::LOST_LINE_ACTION`.
 *
 * The owned `MatrixLineTracer _tracer` instance is exposed through `getTracer()` so that the
 * Intersection module can read junction state from the *same* sensor without issuing
 * duplicate I2C transactions.
 *
 * Lifecycle:
 *  - `begin()` once from `setup()` after `MiniR4.begin()` and `Drive.begin()`.
 *  - `update()` every iteration of the main `loop()`. Non-blocking. Short-circuits if disabled.
 *  - `setEnabled(true)` to start following; `setEnabled(false)` to stop and brake (default).
 *  - `followDistance(cm)` / `waitUntilDistanceReached()` when a caller needs the line-follow
 *    segment to finish before returning motor ownership to the next blocking `Drive.*` move.
 *
 * Fluent API:
 *  The verb method `setEnabled(bool)` returns a `[[nodiscard]]` builder
 *  (`LineFollowerEnableBuilder`). Setters such as `.setBrake(bool)` and `.setPreset(...)` are
 *  staged on the builder; the staged state is applied exactly once when the builder destructs
 *  at end-of-statement (RAII auto-execute). See `LineFollowerEnableBuilder` for examples.
 *
 * @note When `setEnabled(false)` is called with the default brake behaviour, the chassis is
 *       actively braked and the derivative-term history (`_lastErr`) is cleared so the D-term
 *       does not carry into the next enable cycle.
 */

#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>
#include <MatrixMiniR4.h>    // Brings in `MatrixLineTracer` transitively via MiniR4I2C.h.
                             // The Arduino 1.5 library spec only adds `<lib>/src/` to the
                             // include path — not nested subdirectories like
                             // `src/Modules/Sensors/` — so `<MiniR4_MXLineTracer.h>` is not
                             // portable across build environments. The vendor facade
                             // `<MatrixMiniR4.h>` already declares the type, which is all
                             // we need for the `MatrixLineTracer*` member below.
                             // (See code review F9.)
#include "Config.h"

// Forward declaration so the builder can hold a reference to the owner type.
class LineFollowerClass;

/**
 * @brief RAII builder returned by `LineFollowerClass::setEnabled(bool)` for fluent
 *        configuration of the enable transition.
 *
 * @details
 * The builder stages a target state — enable/disable, optional brake-on-disable override,
 * optional cruise-preset override — and applies all of it exactly once when its destructor
 * runs at end-of-statement. This lets callers chain knobs without an unreadable positional
 * argument list:
 *
 * @code
 * LineFollower.setEnabled(true);                                         // turn on, default cruise
 * LineFollower.setEnabled(true).setPreset(Config::SpeedPreset::FAST);    // turn on, FAST cruise
 * LineFollower.setEnabled(false);                                        // turn off, active brake
 * LineFollower.setEnabled(false).setBrake(false);                        // turn off, coast
 * LineFollower.setEnabled(false).setBrake(false).setPreset(Config::SpeedPreset::SLOW);
 * @endcode
 *
 * Defaults:
 *  - When `setEnabled(true)` is invoked without `.setPreset(...)`, the current preset is left
 *    UNCHANGED — re-enabling the controller does not silently reset cruise.
 *  - When `setEnabled(false)` is invoked without `.setBrake(...)`, the brake defaults to
 *    `true`, matching the historical Phase 1 behaviour.
 *  - `tShape` is **on by default** on `setEnabled(true)` — cruise ramps from 0 up to the
 *    resolved preset over `Config::TSHAPE_ACCEL_PWM_PER_CM`. See DECISIONS.md
 *    "Sync + T-shape as default move profile" for rationale.
 *
 * Lifetime / safety:
 *  - The builder is `[[nodiscard]]`: storing the temporary in a variable without using it
 *    triggers a compiler warning, since the staged state would never apply.
 *  - Copy is disabled outright. Move is disabled too — chain temporaries are constructed in
 *    place via NRVO, so a move constructor is not needed and removing it eliminates any risk
 *    of a double-execute.
 *
 * @note A single builder applies its state to the owner exactly once via the
 *       `LineFollowerClass::_applyEnableBuilder(...)` backdoor, then sets `_executed = true`
 *       so a second destructor invocation (defensive) does nothing.
 */
class [[nodiscard]] LineFollowerEnableBuilder
{
public:
    /**
     * @brief Construct the builder with the target enable state.
     *
     * @param owner          Reference to the owning `LineFollowerClass`. Must outlive `this`,
     *                       which is trivially true since the builder is a stack temporary
     *                       created from a method call on the singleton.
     * @param targetEnabled  `true` to transition into enabled, `false` to transition into
     *                       disabled when the destructor runs.
     * @note No state is mutated on the owner here — all changes are deferred to `~`.
     */
    LineFollowerEnableBuilder(LineFollowerClass& owner, bool targetEnabled);

    LineFollowerEnableBuilder(const LineFollowerEnableBuilder&)            = delete;
    LineFollowerEnableBuilder& operator=(const LineFollowerEnableBuilder&) = delete;
    LineFollowerEnableBuilder(LineFollowerEnableBuilder&&)                 = delete;
    LineFollowerEnableBuilder& operator=(LineFollowerEnableBuilder&&)      = delete;

    /**
     * @brief Destructor — applies the staged transition exactly once.
     *
     * @note Calls `LineFollowerClass::_applyEnableBuilder(...)` with the staged values, then
     *       sets `_executed` so a (defensive) re-entrant destructor call would be a no-op.
     */
    ~LineFollowerEnableBuilder();

    /**
     * @brief Override the brake-on-disable behaviour.
     *
     * @param brake  `true` to actively brake (`Drive.stop(true)`) on disable; `false` to skip
     *               the stop call entirely so the chassis keeps whatever speeds were last
     *               latched on `Drive`. The path planner is then responsible for issuing the
     *               next move.
     * @return       `*this` for fluent chaining.
     * @note Only meaningful when the parent `setEnabled(false)`. Calling `.setBrake()` after
     *       `setEnabled(true)` is silently ignored — the enable path never calls
     *       `Drive.stop(...)`.
     */
    LineFollowerEnableBuilder& setBrake(bool brake);

    /**
     * @brief Override the cruise speed preset as part of this transition.
     *
     * @param preset One of `Config::SpeedPreset::{SLOW, NORMAL, FAST}`. Resolved to a PWM
     *               value via `LineFollowerClass::_presetToPwm()` on the next `update()` tick.
     * @return       `*this` for fluent chaining.
     * @note If this method is NOT called on the builder, the owner's current preset is left
     *       unchanged. This means re-enabling the controller does not reset cruise.
     */
    LineFollowerEnableBuilder& setPreset(Config::SpeedPreset preset);

    /**
     * @brief Enable trapezoidal velocity profile on this enable transition.
     *
     * @note **`tShape` is on by default** — this setter is idempotent and exists for
     *       explicit-readability. See DECISIONS.md "Sync + T-shape as default move profile"
     *       for rationale.
     *
     * On a `setEnabled(true)` call, the line follower's cruise PWM ramps from 0 up to the
     * resolved preset over an implicit ramp distance: `cruise / accel`, clamped to
     * `[Config::TSHAPE_RAMP_MIN_CM, Config::TSHAPE_RAMP_MAX_CM]`. The ramp is purely forward
     * (line following has no natural "end of move", so there is no ramp-down) and is
     * consumed once: after the ramp distance is traveled, subsequent ticks use the full
     * preset cruise. Disabling the controller cancels the ramp.
     *
     * Acceleration defaults to `Config::TSHAPE_ACCEL_PWM_PER_CM`. Override via `.setAccel()`.
     * Has no effect on a `setEnabled(false)` transition.
     *
     * @return `*this` for fluent chaining.
     */
    LineFollowerEnableBuilder& tShape();

    /**
     * @brief Override the per-PWM-per-cm acceleration rate for this enable's ramp.
     *
     * Implies `tShape()`. If `tShape()` was not also chained, this setter has no effect on
     * the disable path; on the enable path it triggers the ramp.
     *
     * @param pwmPerCm PWM units gained per cm of travel during the ramp.
     * @return `*this` for chaining.
     */
    LineFollowerEnableBuilder& setAccel(float pwmPerCm);

    /**
     * @brief Override the line-follow direction as part of this transition.
     *
     * @param reverse `true` to follow the line while driving BACKWARDS (cruise PWM negated;
     *                P+D mix unchanged because the differential-drive geometry is symmetric
     *                — a CW body rotation moves the front-mounted tracer the same direction
     *                regardless of motion direction). `false` for the default forward case.
     * @return        `*this` for fluent chaining.
     * @note Assumes the line tracer is FRONT-mounted on the chassis. The tracer itself is
     *       not moved or re-bound — only the sign of cruise flips. The line-error sign
     *       does NOT need to flip for differential drive, see DECISIONS.md for the
     *       kinematics derivation.
     * @note If this method is NOT called on the builder, the owner's current direction is
     *       left unchanged — re-enabling the controller does not silently reset direction.
     */
    LineFollowerEnableBuilder& setReverse(bool reverse);

    /**
     * @brief Stop after the chassis has travelled @p cm centimetres along the line.
     *
     * Used by `PathTracker` to traverse PROJECTION edges (line segments terminated by a
     * waypoint that has no physical intersection marker, so the runtime stops by encoder
     * distance). When the gate trips, `update()` self-disables (with active brake by
     * default) and latches a one-shot `isDistanceReached()` flag the consumer reads to
     * advance its FSM.
     *
     * @param cm  Target travel in centimetres. Non-positive values are silently clamped
     *            to zero — the gate trips on the first tick, equivalent to "disable now."
     * @return    `*this` for fluent chaining.
     * @note Implies `setEnabled(true)` — has no effect on a `setEnabled(false)` transition.
     * @note The gate measures the absolute average of the two wheels' encoder deltas via
     *       `encDegToCm(...)` (`WROMath.h`). Encoder counters are sampled at the moment
     *       the builder applies (in `_applyEnableBuilder`).
     * @note A second `setEnabled(true).setDistanceTarget(cm)` after the gate trips
     *       re-arms cleanly — the controller resamples the encoder counters as its new
     *       baseline.
     */
    LineFollowerEnableBuilder& setDistanceTarget(float cm);

private:
    LineFollowerClass&  _owner;                ///< Owner whose state is mutated on destruction.
    bool                _targetEnabled;        ///< Final value of `_owner._enabled`.
    bool                _brake               = true;   ///< Brake on disable (only used if !enable).
    bool                _hasPresetOverride   = false;  ///< True iff `_presetOverride` is meaningful.
    Config::SpeedPreset _presetOverride{};             ///< Pending preset; only honored when override flag set.
    bool                _tShape              = true;   ///< default ON — cruise-ramp on a setEnabled(true) transition.
                                                        ///< See DECISIONS.md "Sync + T-shape as default move profile".
    bool                _accelExplicit       = false;  ///< Reserved for parity with the Drive builders.
    float               _accel               = Config::TSHAPE_ACCEL_PWM_PER_CM; ///< Ramp rate.
    bool                _hasReverseOverride  = false;  ///< True iff `_reverseOverride` is meaningful.
    bool                _reverseOverride     = false;  ///< Pending reverse flag; only honored when override flag set.
    bool                _hasDistanceTarget   = false;  ///< True iff `_distanceTargetCm` is meaningful.
    float               _distanceTargetCm    = 0.0f;   ///< Pending distance gate (cm); only honored when flag set.
    bool                _executed            = false;  ///< Guards against repeated apply.
};

/**
 * @brief Singleton outer-loop line follower over `MatrixLineTracer` and `Drive.setSpeed`.
 *
 * Owns one `MatrixLineTracer` instance, runs a P+D controller every `update()` tick, and
 * drives the chassis at `_presetToPwm(_preset) ± corr`. The integral term is intentionally
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
     *      - `CRAWL` → `Drive.setSpeed(cruise/2, cruise/2)` — straight creep forward (no
     *                  steering) so the strip can be reacquired. `cruise` is resolved from
     *                  the current preset via `_presetToPwm()`.
     *  - Otherwise (line online): clears `_lostLine`, samples `_tracer.getError()`, runs a P+D
     *    controller `corr = LINE_KP*err + LINE_KD*(err - _lastErr)`, clamps `corr` to
     *    `[-LINE_MAX_CORR, +LINE_MAX_CORR]`, then commands
     *    `Drive.setSpeed(cruise + corr, cruise - corr)` with each channel re-clamped to
     *    `[-100, +100]` (the DriveDC PWM range). `cruise` is sourced from
     *    `_presetToPwm(_preset)`, NOT the historical hardcoded `Config::CRUISE_SPEED`.
     *
     * @note Non-blocking. Contains NO `delay()`. Safe to call at any cadence.
     * @note Phase 1 deliberately omits the integral term — `Config::LINE_KI` is unused. If
     *       integral action is needed later, add an `_errSum` accumulator with appropriate
     *       windup guard (clamp `_errSum` and/or only integrate when `|err|` is small).
     * @note The `Drive.setSpeed(L, R)` call below applies motor-bias calibration
     *       (Strategy B) automatically — see DECISIONS.md. If oscillation persists
     *       after calibration, consider adding Strategy C (R4-side encoder-rate
     *       PID) as a runtime layer on top.
     */
    void update();

    /**
     * @brief Begin a fluent enable/disable transition.
     *
     * Returns a `[[nodiscard]]` `LineFollowerEnableBuilder` that stages the target state. The
     * transition is applied exactly once when the builder destructs at end-of-statement, so
     * the bare expression `LineFollower.setEnabled(true);` Just Works as a one-liner.
     *
     * @param on  `true` to transition into enabled (next `update()` tick will run the P+D
     *            controller); `false` to transition into disabled (controller short-circuits).
     * @return    A new `LineFollowerEnableBuilder` whose destructor applies the transition.
     *            Chain `.setBrake(bool)` and/or `.setPreset(Config::SpeedPreset)` on it to
     *            override defaults.
     * @note Disabling defaults to active brake (`Drive.stop(true)`) — pass `.setBrake(false)`
     *       to suppress the brake and coast on whatever speeds Drive last latched.
     * @note Disabling always resets `_lastErr = 0.0f` and clears `_lostLine`, regardless of
     *       the brake choice. The brake choice ONLY affects whether `Drive.stop(...)` fires.
     * @note On enable, the cruise preset is left unchanged unless `.setPreset(...)` is also
     *       chained — re-enabling does not silently reset cruise to a default.
     */
    LineFollowerEnableBuilder setEnabled(bool on);

    /**
     * @brief Direct (non-fluent) cruise-preset setter.
     *
     * Use this for the simple 1-arg case where you want to retune cruise without toggling
     * enabled state. The same effect is achievable via the fluent builder, e.g.
     * `setEnabled(isEnabled()).setPreset(p)`, but this method is shorter for that case.
     *
     * @param preset One of `Config::SpeedPreset::{SLOW, NORMAL, FAST}`.
     * @note Takes effect on the next `update()` tick — the active tick (if any) is not
     *       re-evaluated, since `update()` reads `_preset` at the top of its body.
     */
    void setPreset(Config::SpeedPreset preset);

    /**
     * @brief Read the active cruise preset.
     *
     * @return The `Config::SpeedPreset` currently used by `update()` to compute cruise PWM.
     */
    Config::SpeedPreset getPreset() const;

    /**
     * @brief Direct (non-fluent) reverse-direction setter.
     *
     * @param reverse `true` to follow the line backwards; `false` for forward (default).
     * @note Takes effect on the next `update()` tick. Assumes a front-mounted tracer.
     */
    void setReverse(bool reverse);

    /**
     * @brief Read the active line-follow direction.
     * @return `true` if the controller is currently following the line in reverse; `false`
     *         for the default forward case.
     */
    bool getReverse() const;

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

    /**
     * @brief Read-and-clear: was the distance gate tripped since the last query?
     *
     * `update()` sets the internal flag exactly once when the encoder-distance gate fires
     * and self-disables the controller. The flag stays set until `isDistanceReached()` is
     * called, at which point it is cleared. Mirrors `Intersection::isAtIntersection()`'s
     * read-and-clear contract — the consumer (PathTracker's PROJECTION_FOLLOW state) is
     * the single owner of the latched event.
     *
     * @return `true` exactly once after each gate trip; `false` otherwise.
     * @note Intentionally non-`const` because it mutates `_distanceReached`.
     */
    bool isDistanceReached();

    /**
     * @brief Sync-blocking helper: follow the current line for @p cm, then return.
     *
     * Convenience wrapper for the common "arm + poll to completion" pattern:
     *
     * @code
     * LineFollower.followDistance(50);
     * Drive.driveDist(30);
     * LineFollower.followDistance(50);
     * @endcode
     *
     * Equivalent to:
     *
     * @code
     * LineFollower.setEnabled(true).setDistanceTarget(cm);
     * return LineFollower.waitUntilDistanceReached();
     * @endcode
     *
     * Uses the currently configured preset + reverse direction. If the caller needs custom
     * settings, stage them first with `setPreset(...)`, `setReverse(...)`, or the fluent
     * builder, then call `waitUntilDistanceReached()`.
     *
     * @param cm Target travel in centimetres.
     * @return `true` if the distance gate tripped; `false` on timeout or if a bounded Drive
     *         move already owns the chassis.
     * @warning Do NOT call from inside any module's `update()` method. This helper spins with
     *          `delay(2)` and is reserved for top-level orchestration only.
     */
    bool followDistance(float cm);

    /**
     * @brief Block until the currently-armed distance gate trips.
     *
     * This is the lower-level half of `followDistance(cm)`. Use it when you need extra builder
     * knobs before starting the segment, e.g.
     *
     * @code
     * LineFollower.setEnabled(true)
     *             .setPreset(Config::SpeedPreset::FAST)
     *             .setDistanceTarget(50);
     * const bool ok = LineFollower.waitUntilDistanceReached();
     * @endcode
     *
     * The wait is bounded by `Config::LINE_FOLLOW_WAIT_TIMEOUT_MS`. On timeout, or if the
     * helper detects that a bounded Drive move already owns the chassis, the follower is
     * disabled with active brake and `false` is returned.
     *
     * @return `true` if the distance gate tripped; `false` otherwise.
     * @warning Do NOT call from inside any module's `update()` method. This helper spins with
     *          `delay(2)` and is reserved for top-level orchestration only.
     */
    bool waitUntilDistanceReached();

    /**
     * @brief Internal apply-hook called by `LineFollowerEnableBuilder::~`.
     *
     * Public only because the builder is a free-standing class; conceptually private. Do not
     * call from user code — invoke `setEnabled(...)` instead and let the builder destructor
     * dispatch here.
     *
     * @param targetEnabled       Final value of `_enabled` after the transition.
     * @param brake               Whether to call `Drive.stop(true)` on a disable transition;
     *                            ignored when `targetEnabled == true`.
     * @param hasPresetOverride   `true` if `preset` should overwrite the active preset.
     * @param preset              New preset; honored only when `hasPresetOverride` is true.
     * @param tShape              On an enable transition, ramps cruise from 0 up to the
     *                            resolved preset over an implicit distance — see
     *                            `LineFollowerEnableBuilder::tShape()`.
     * @param accel               Ramp rate in PWM-per-cm; used only when `tShape == true` AND
     *                            `targetEnabled == true`.
     * @param hasReverseOverride  `true` if `reverse` should overwrite the active direction.
     * @param reverse             New reverse flag; honored only when `hasReverseOverride`.
     * @param hasDistanceTarget   `true` if `distanceTargetCm` should arm the gate.
     * @param distanceTargetCm    Target travel in cm; honored only when flag set AND
     *                            `targetEnabled == true`. Encoder baseline is captured here.
     */
    void _applyEnableBuilder(bool                targetEnabled,
                             bool                brake,
                             bool                hasPresetOverride,
                             Config::SpeedPreset preset,
                             bool                tShape,
                             float               accel,
                             bool                hasReverseOverride,
                             bool                reverse,
                             bool                hasDistanceTarget,
                             float               distanceTargetCm);

private:
    // `presetToPwm` lives in `WROMath.h` — see Drive.h's matching note. Internal callers
    // here invoke the free function directly.

    MatrixLineTracer    _tracer;                                  ///< Owned sensor driver shared with Intersection (T7).
    bool                _enabled  = false;                        ///< When false, `update()` is a no-op.
    bool                _lostLine = false;                        ///< Last-tick lost-line state (no channel online).
    float               _lastErr  = 0.0f;                         ///< Previous error sample, for the D-term.
    Config::SpeedPreset _preset   = Config::SpeedPreset::NORMAL;  ///< Active cruise preset; sourced by `update()`.
    bool                _reverse  = false;                        ///< When true, `update()` negates cruise to follow line in reverse.

    // Trapezoidal cruise-ramp state (`.tShape()` on the enable builder).
    bool    _useTShape       = false;    ///< If true, cruise PWM ramps over distance after enable.
    float   _tShapeAccel     = Config::TSHAPE_ACCEL_PWM_PER_CM;
    int32_t _tShapeStartEnc  = 0;        ///< Encoder reading at enable.
    int16_t _tShapeStartPwm  = 0;        ///< Starting cruise (typically 0).
    int16_t _tShapeTargetPwm = 0;        ///< Final cruise (resolved from preset).

    // Distance-target gate state (`.setDistanceTarget(cm)` on the enable builder, used by
    // PathTracker's PROJECTION_FOLLOW state). When armed, `update()` evaluates the average
    // absolute encoder delta after the PID branch and self-disables (active brake) on hit.
    bool    _distanceTargetActive = false;    ///< Gate armed.
    float   _distanceTargetCm     = 0.0f;     ///< Target travel from baseline.
    int32_t _distanceStartEncL    = 0;        ///< Left encoder reading at arm.
    int32_t _distanceStartEncR    = 0;        ///< Right encoder reading at arm.
    bool    _distanceReached      = false;    ///< One-shot latch; cleared by isDistanceReached().
};

/**
 * @brief Single global instance — defined in LineFollower.cpp.
 */
extern LineFollowerClass LineFollower;

#endif    // LINEFOLLOWER_H
