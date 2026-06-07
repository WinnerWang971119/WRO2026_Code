/**
 * @file LineFollower.h
 * @brief Outer P+D line-following controller riding on top of `Drive.setSpeed`.
 * @author WRO 2026
 *
 * @details
 * `LineFollowerClass` is the singleton that converts the weighted error reported by the
 * 8-channel reflectance line array (`WROLineArrayClass::getError()`, range `[-3.5, +3.5]`)
 * into differential left/right PWM commands sent to `Drive.setSpeed(L, R)`. It is deliberately
 * the *outer* loop: it commands raw PWM, NOT `MoveSync`, so the lower-MCU's wheel-balance PID
 * does not stack on top of this one and oscillate (see `Drive::setSpeed` doxygen).
 *
 * Tuning is sourced exclusively from `Config::LINE_KP`, `Config::LINE_KI` (currently 0 — see
 * `update()` note), `Config::LINE_KD`, `Config::LINE_MAX_CORR`, and the active speed preset
 * (resolved by `_presetToPwm()` from `Config::SPEED_SLOW/NORMAL/FAST`). Hardware wiring,
 * per-channel calibration baselines, and the firmware-level illumination/normalization for
 * the underlying MXColor boards live entirely in `WROLineArrayClass` (see `WROLineArray.h`
 * for the two-layer calibration contract). Lost-line behaviour is dispatched on
 * `Config::LOST_LINE_ACTION`.
 *
 * The shared `WROLineArrayClass` singleton (`LineArray`) is exposed through `getArray()` so
 * that the Intersection module can read junction state from the *same* sensor without
 * issuing duplicate I2C transactions on the same Wire1 boards.
 *
 * Lifecycle:
 *  - `begin()` once from `setup()` after `MiniR4.begin()` and `Drive.begin()`.
 *  - `update()` every iteration of the main `loop()`. Non-blocking. Short-circuits if disabled.
 *  - `setEnabled(true)` to start following; `setEnabled(false)` to stop and brake (default).
 *  - `followDistance(cm)` / `waitUntilDistanceReached()` when a caller needs the line-follow
 *    segment to finish before returning motor ownership to the next blocking `Drive.*` move.
 *  - `followUntilIntersection()` / `waitUntilIntersection()` when a caller wants the follower
 *    to own the motors until the next fresh intersection edge, then stop.
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
#include <MatrixMiniR4.h>
#include "Config.h"
#include "WROLineArray.h"    // Brings in `WROLineArrayClass` and the `LineArray` singleton —
                             // the WRO 2026 8-channel reflectance backend that replaces the
                             // vendor `MatrixLineTracer`. `LineFollower` and `Intersection`
                             // both hold a non-owning `WROLineArrayClass*` onto this
                             // singleton; the controller-side surface (getError(),
                             // isOnline(), getJunctionType()) is preserved so the P+D
                             // mathematics here did not change.

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
     * @brief Enable the trapezoidal cruise ramp-UP on this enable transition.
     *
     * @note **`tShape` is on by default** — this setter is idempotent and exists for
     *       explicit-readability. It is effectively an alias of `setTShapeRampUp(true)`
     *       and affects ONLY the ramp-up phase. The symmetric ramp-DOWN before a
     *       distance gate is controlled by `setTShapeRampDown(bool)` (also on by
     *       default). See DECISIONS.md "LineFollower distance-gated stops use a
     *       symmetric trapezoidal profile" for rationale.
     *
     * On a `setEnabled(true)` call, the line follower's cruise PWM ramps from 0 up to the
     * resolved preset over an implicit ramp distance: `cruise / accel`, clamped to
     * `[Config::TSHAPE_RAMP_MIN_CM, Config::TSHAPE_RAMP_MAX_CM]`. With a distance target
     * also armed, the ramp-up is additionally clamped to half the distance target so it
     * cannot overlap the ramp-down zone. The ramp is consumed once: after the ramp
     * distance is traveled, subsequent ticks use the full preset cruise. Disabling the
     * controller cancels the ramp.
     *
     * Acceleration defaults to `Config::TSHAPE_ACCEL_PWM_PER_CM`. Override via `.setAccel()`.
     * Has no effect on a `setEnabled(false)` transition.
     *
     * @return `*this` for fluent chaining.
     */
    LineFollowerEnableBuilder& tShape();

    /**
     * @brief Opt out of the cruise ramp-up on this enable transition.
     *
     * @param on  `true` (default) keeps the trapezoidal ramp from 0 → cruise; `false`
     *            instantly applies full cruise from the first post-enable tick.
     * @return `*this` for fluent chaining.
     *
     * @note Default is `true` for ergonomic continuity with `tShape()`. The
     *       enable path automatically skips the ramp-up reseed when the controller
     *       was already enabled (re-arm semantics), so callers chaining
     *       `setDistanceTarget(...)` on an already-running follower do not need to
     *       set this explicitly. Provided for manual sketches that want instant cruise.
     */
    LineFollowerEnableBuilder& setTShapeRampUp(bool on);

    /**
     * @brief Opt out of the cruise ramp-down before the distance gate.
     *
     * @param on  `true` (default) ramps cruise from its current value down to
     *            `Config::PWM_MIN_MAGNITUDE` over the approach distance, then
     *            fires active brake at the gate. `false` holds full cruise until
     *            the gate trips and brakes from cruise directly (legacy behaviour).
     * @return `*this` for fluent chaining.
     *
     * @note Only meaningful when `setDistanceTarget(...)` is also chained.
     *       For a `LINE_OVERSHOOT`-style 5 cm hop, ramp distance is fitted to the
     *       available distance (see implementation note in `LineFollower.cpp`).
     */
    LineFollowerEnableBuilder& setTShapeRampDown(bool on);

    /**
     * @brief Opt out of BOTH cruise ramps in one call — run this enable session flat.
     *
     * Convenience over `setTShapeRampUp(false).setTShapeRampDown(false)`: clears the
     * ramp-up (0 → cruise on the enable transition) AND the ramp-down (cruise →
     * `Config::PWM_MIN_MAGNITUDE` before a distance gate). The result is full cruise PWM
     * from the first post-enable tick through to the gate brake.
     *
     * Provided for fluent parity with `DriveDistBuilder::flat()` / `DriveTimeBuilder::flat()`
     * (ADR-033). Unlike the Drive builders — where `tShape` is a single flag — LineFollower
     * has two independent ramp phases, so this method is sugar over the two existing
     * setters rather than a new capability.
     *
     * @return `*this` for fluent chaining.
     * @note **Ordering matters.** `tShape()`/`setAccel(...)` re-enable the ramp-up if chained
     *       AFTER `.flat()`. Treat `.flat()` as mutually exclusive with the ramp setters.
     */
    LineFollowerEnableBuilder& flat();

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
     * @note On 2-board configurations (`Config::LINE_BOARD_COUNT == 2`), this flips
     *       `WROLineArrayClass`'s active board to the rear — see
     *       `WROLineArray.h::setReverseActive()` and DECISIONS.md. The actual wiring will
     *       land in T5.
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

    /**
     * @brief Override the lost-line policy for this enable session — dead-reckon vs.
     *        `Config::LOST_LINE_ACTION`.
     *
     * @param on  When `on == true` AND `setEnabled(true)` is the parent transition, the
     *            controller dead-reckons symmetric cruise PWM whenever `_array->isOnline()`
     *            returns false during this enable session, instead of dispatching
     *            `Config::LOST_LINE_ACTION`. Default `false` preserves today's
     *            LOST_LINE_ACTION behaviour (STOP / COAST / CRAWL). Used by PathTracker for
     *            the 5 cm `LINE_OVERSHOOT` window so the chassis advances cleanly even when
     *            the line physically ends at the node. Has no effect on `setEnabled(false)`
     *            transitions.
     * @return    `*this` for fluent chaining.
     * @note If this method is NOT called on the builder, the owner's current overshoot-mode
     *       value is left unchanged across re-enables (mirrors the `setPreset(...)` /
     *       `setReverse(...)` override pattern). See ADR-023 for the pass-through-collinear
     *       intersections design and ADR-018 for the underlying lost-line dispatch model.
     */
    LineFollowerEnableBuilder& setOvershootMode(bool on);

    /**
     * @brief Override whether the distance-gate trip self-disables the controller and
     *        actively brakes.
     *
     * @param on  When `on == false` AND a distance gate is armed via
     *            `setDistanceTarget(cm)`, the gate trip latches `_distanceReached = true`
     *            and clears `_distanceTargetActive` + ramp state, but does NOT self-disable
     *            the controller and does NOT call `Drive.stop`. Used by PathTracker for
     *            pass-through traversals where motor ownership must stay with LineFollower
     *            across the 5 cm overshoot. Default `true` preserves today's brake-on-gate
     *            behaviour (gate trip → self-disable + active brake).
     * @return    `*this` for fluent chaining.
     * @note If this method is NOT called on the builder, the owner's current brake-on-gate
     *       value is left unchanged across re-enables (mirrors the `setPreset(...)` /
     *       `setReverse(...)` override pattern). See ADR-023 for the pass-through-collinear
     *       intersections design.
     */
    LineFollowerEnableBuilder& setBrakeOnGate(bool on);

private:
    LineFollowerClass&  _owner;                ///< Owner whose state is mutated on destruction.
    bool                _targetEnabled;        ///< Final value of `_owner._enabled`.
    bool                _brake               = true;   ///< Brake on disable (only used if !enable).
    bool                _hasPresetOverride   = false;  ///< True iff `_presetOverride` is meaningful.
    Config::SpeedPreset _presetOverride{};             ///< Pending preset; only honored when override flag set.
    bool                _tShapeRampUp        = true;   ///< default ON — cruise ramp-up from 0 on a setEnabled(true) transition.
                                                        ///< See DECISIONS.md "LineFollower distance-gated stops use a symmetric trapezoidal profile".
    bool                _tShapeRampDown      = true;   ///< default ON — cruise ramp-down to PWM_MIN_MAGNITUDE before the distance gate.
                                                        ///< Only meaningful with `setDistanceTarget(...)` also chained.
    bool                _accelExplicit       = false;  ///< Reserved for parity with the Drive builders.
    float               _accel               = Config::TSHAPE_ACCEL_PWM_PER_CM; ///< Ramp rate.
    bool                _hasReverseOverride  = false;  ///< True iff `_reverseOverride` is meaningful.
    bool                _reverseOverride     = false;  ///< Pending reverse flag; only honored when override flag set.
    bool                _hasDistanceTarget   = false;  ///< True iff `_distanceTargetCm` is meaningful.
    float               _distanceTargetCm    = 0.0f;   ///< Pending distance gate (cm); only honored when flag set.
    bool                _hasOvershootModeOverride = false;  ///< True iff `_overshootModeOverride` is meaningful.
    bool                _overshootModeOverride    = false;  ///< Pending overshoot-mode flag; only honored when override flag set.
    bool                _hasBrakeOnGateOverride   = false;  ///< True iff `_brakeOnGateOverride` is meaningful.
    bool                _brakeOnGateOverride      = true;   ///< Pending brake-on-gate flag; only honored when override flag set. Default true matches owner default.
    bool                _executed            = false;  ///< Guards against repeated apply.
};

/**
 * @brief Singleton outer-loop line follower over `WROLineArrayClass` and `Drive.setSpeed`.
 *
 * Holds a non-owning pointer to the `LineArray` singleton (bound in `begin()` and shared
 * with `Intersection` to avoid duplicate I2C transactions), runs a P+D controller every
 * `update()` tick, and drives the chassis at `_presetToPwm(_preset) ± corr`. The integral
 * term is intentionally absent in Phase 1 — see `update()` doxygen. When the underlying
 * line array classifies the current tick as a junction (LEFT/RIGHT/T_OR_CROSS), the P+D
 * mix is bypassed and both wheels run at symmetric cruise to suppress centroid-skew wobble
 * across the junction — see `isPidFrozen()`.
 */
class LineFollowerClass
{
public:
    /**
     * @brief Bind onto the `LineArray` singleton and reset controller state.
     *
     * Captures `_array = &LineArray` (non-owning) so this controller and `Intersection`
     * both sample the same `WROLineArrayClass` instance — no duplicate I2C transactions on
     * the same Wire1 boards. Resets the controller state (`_enabled`, `_lostLine`,
     * `_lastErr`). All hardware initialisation (I2C port selection, firmware-level
     * illumination via `MatrixColor::setLight(...)`, software per-channel calibration)
     * lives in `WROLineArrayClass::begin()` — see `WROLineArray.h` for the two-layer
     * calibration contract.
     *
     * @return `true` once `_array` is bound. The underlying `WROLineArrayClass::begin()`
     *         must already have been called by the sketch — `LineFollower::begin()` does
     *         not call it implicitly, since the lifecycle order is strict: `LineArray`
     *         comes up BEFORE the controllers that depend on it.
     * @note Must be called from `setup()` AFTER `MiniR4.begin()` (so `Wire1` is alive)
     *       AND AFTER `LineArray.begin()` (so the singleton is hardware-initialised).
     */
    bool begin();

    /**
     * @brief Cooperative tick — call every iteration of the main `loop()`.
     *
     * Behaviour:
     *  - If `!_enabled`, returns immediately (short-circuit).
     *  - If `!_array->isOnline()` (line lost beneath every channel of the active board),
     *    sets `_lostLine = true` and dispatches `Config::LOST_LINE_ACTION`:
     *      - `STOP`  → `Drive.stop(true)`  — active brake.
     *      - `COAST` → `Drive.stop(false)` — release power, roll to a stop.
     *      - `CRAWL` → `Drive.setSpeed(cruise/2, cruise/2)` — straight creep forward (no
     *                  steering) so the strip can be reacquired. `cruise` is resolved from
     *                  the current preset via `_presetToPwm()`.
     *  - Otherwise (line online): clears `_lostLine`, then samples
     *    `_array->getJunctionType()` ONCE to drive the junction-freeze gate. When the
     *    classifier returns anything other than NONE/UNKNOWN (LEFT, RIGHT, or T_OR_CROSS),
     *    the P+D math is bypassed and both wheels are commanded at the same `cruise` PWM
     *    (deadband-snapped) to suppress centroid-skew wobble across the junction —
     *    `isPidFrozen()` exposes this state. On the first non-frozen tick after a freeze,
     *    `_lastErr` is reset to 0 to prevent a derivative kick. Otherwise (no junction),
     *    samples `_array->getError()` (range `[-3.5, +3.5]` for the 8-channel array), runs
     *    a P+D controller `corr = LINE_KP*err + LINE_KD*(err - _lastErr)`, clamps `corr` to
     *    `[-LINE_MAX_CORR, +LINE_MAX_CORR]`, then commands
     *    `Drive.setSpeed(cruise + corr, cruise - corr)` with each channel re-clamped to
     *    `[-100, +100]` (the DriveDC PWM range). `cruise` is sourced from
     *    `_presetToPwm(_preset)`, NOT the historical hardcoded `Config::CRUISE_SPEED`.
     *    The distance-gate evaluation runs unconditionally after either branch so the
     *    encoder gate keeps ticking during freeze.
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
     * @note On 2-board configurations (`Config::LINE_BOARD_COUNT == 2`), this flips
     *       `WROLineArrayClass`'s active board to the rear — see
     *       `WROLineArray.h::setReverseActive()` and DECISIONS.md. The actual wiring will
     *       land in T5.
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
     * @brief Whether the most recent update() tick froze the P+D mix due to a
     *        junction classification (LEFT, RIGHT, or T_OR_CROSS).
     *
     * When the underlying `WROLineArrayClass::getJunctionType()` returns anything
     * other than NONE/UNKNOWN, the weighted-center error is unreliable (the
     * centroid math is skewed by the cross-bar's black region), so the P+D
     * controller is bypassed for that tick and both wheels run at symmetric
     * cruise PWM. This accessor exposes that state for Serial debug, future
     * telemetry, and unit-test-style inspection. The freeze is purely internal
     * to update(); no builder setter or mode flag is added.
     *
     * @return `true` if the freeze branch ran on the last update() tick; `false`
     *         otherwise (including whenever the controller is disabled — the
     *         disable path clears the flag explicitly).
     * @note See DECISIONS.md "LineFollower PID freeze during junction dwell" and
     *       issue #22 for the design rationale.
     * @note EventTrace does NOT consume this accessor today; it exists for
     *       Serial-side tuning visibility without contending with the OLED ring.
     */
    bool isPidFrozen() const;

    /**
     * @brief Most recent weighted error sample stored by the controller.
     *
     * @return The last value returned by `WROLineArrayClass::getError()` while the line was
     *         online (range `[-3.5, +3.5]` for the 8-channel array). Reset to `0.0f` by
     *         `begin()`, by `setEnabled(false)`, and on every entry to the lost-line branch
     *         of `update()` — so a caller observing the value immediately after the line is
     *         dropped will see `0.0f`, not a stale sample.
     * @note Useful as telemetry for OLED tuning views.
     */
    float getRawError() const;

    /**
     * @brief Pointer to the shared `WROLineArrayClass` singleton so Intersection can read
     *        from the same sensor without duplicate I2C transactions.
     *
     * Intersection (T7) calls this from its own `begin()` to bind onto the *same* sensor
     * instance — both modules share the singleton, so a single `getError()` /
     * `isOnline()` / `getJunctionType()` sample on either side issues exactly one set of
     * I2C transactions against the active MXColor board.
     *
     * @return Non-owning pointer to the `WROLineArrayClass` singleton (== `&LineArray`).
     *         Lifetime equals the singleton's.
     */
    WROLineArrayClass* getArray();

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
     * @brief Register the cooperative-tick callback invoked from the sync-blocking follow
     *        helpers (`waitUntilDistanceReached` and `waitUntilIntersection`) so other
     *        modules keep running during a blocked line-follow wait.
     *
     * Mirrors `DriveClass::setTickCallback` exactly — same signature, same fallback
     * semantics. The sketch registers `cooperativeTick` here in `setup()` next to the
     * matching `Drive` / `PathTracker` registrations.
     *
     * @param cb Pointer to the cooperative-tick function. Pass `nullptr` to clear.
     *
     * @note If no callback is registered, the blocking helpers fall back to ticking only
     *       `LineFollower.update()` (and `Intersection.update()` for the intersection
     *       wait) and emit a one-time Serial warning at first use. Safe degradation, but
     *       means `LineArray.update()` will NOT run during the wait — flipping
     *       `Config::LINE_ARRAY_TICK_CACHE_ENABLED` to `true` without a registered
     *       callback would freeze sensor reads to whatever was cached on entry.
     * @warning The registered callback's call chain MUST NOT issue sync-blocking
     *          `followDistance` / `waitUntilDistanceReached` / `followUntilIntersection`
     *          / `waitUntilIntersection` calls. Doing so would re-enter the blocking
     *          spin from inside its own callback and recurse indefinitely. Today no
     *          `update()` does this; if a future module's tick needs to issue
     *          line-follow waits, it must do so off-tick.
     */
    void setTickCallback(void (*cb)());

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
     * @warning Do NOT call from inside any module's `update()` method. This helper is
     *          intentionally sync-blocking and reserved for top-level orchestration only.
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
     * @warning Do NOT call from inside any module's `update()` method. This helper is
     *          intentionally sync-blocking and reserved for top-level orchestration only.
     */
    bool waitUntilDistanceReached();

    /**
     * @brief Sync-blocking helper: follow the current line until the next fresh intersection.
     *
     * Convenience wrapper for:
     *
     * @code
     * LineFollower.setEnabled(true);
     * return LineFollower.waitUntilIntersection();
     * @endcode
     *
     * On success, the helper disables the follower with the default brake behaviour, so motor
     * ownership is cleanly handed back to the next blocking `Drive.*` call.
     *
     * @return `true` if a fresh intersection edge was observed; `false` on timeout or if a
     *         bounded Drive move already owns the chassis.
     * @warning Do NOT call from inside any module's `update()` method. This helper is
     *          intentionally sync-blocking and reserved for top-level orchestration only.
     */
    bool followUntilIntersection();

    /**
     * @brief Block until the next fresh intersection edge is observed.
     *
     * Use this after manually enabling the follower or staging other options first, e.g.
     *
     * @code
     * LineFollower.setEnabled(true).setPreset(Config::SpeedPreset::FAST);
     * const bool ok = LineFollower.waitUntilIntersection();
     * @endcode
     *
     * The helper clears one stale latched edge at entry so it waits for the next newly-detected
     * intersection after the call starts. On success it disables the follower with the default
     * brake behaviour. The wait is bounded by `Config::LINE_FOLLOW_WAIT_TIMEOUT_MS`.
     *
     * @return `true` if a fresh intersection edge was observed; `false` otherwise.
     * @warning Do NOT call from inside any module's `update()` method. This helper is
     *          intentionally sync-blocking and reserved for top-level orchestration only.
     */
    bool waitUntilIntersection();

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
     * @param tShapeRampUp        On an enable transition that is a true off→on transition,
     *                            ramps cruise from 0 up to the resolved preset over an
     *                            implicit distance — see `LineFollowerEnableBuilder::tShape()`.
     *                            Re-enabling a running follower never reseeds the ramp-up
     *                            regardless of this flag.
     * @param tShapeRampDown      On an enable transition with a distance target armed, ramps
     *                            cruise from the current preset's PWM down to
     *                            `Config::PWM_MIN_MAGNITUDE` over the approach distance —
     *                            see `LineFollowerEnableBuilder::setTShapeRampDown(bool)`.
     * @param accel               Ramp rate in PWM-per-cm; used by both ramp phases when
     *                            `targetEnabled == true`.
     * @param hasReverseOverride  `true` if `reverse` should overwrite the active direction.
     * @param reverse             New reverse flag; honored only when `hasReverseOverride`.
     * @param hasDistanceTarget   `true` if `distanceTargetCm` should arm the gate.
     * @param distanceTargetCm    Target travel in cm; honored only when flag set AND
     *                            `targetEnabled == true`. Encoder baseline is captured here.
     * @param hasOvershootModeOverride  `true` if `overshootMode` should overwrite the owner's
     *                            `_overshootMode`. Absent → owner value preserved across
     *                            re-enables (mirrors the preset/reverse override pattern).
     * @param overshootMode       New overshoot-mode flag; honored only when
     *                            `hasOvershootModeOverride`. When true AND
     *                            `targetEnabled == true`, `update()` dead-reckons symmetric
     *                            cruise on `isOnline() == false` instead of dispatching
     *                            `Config::LOST_LINE_ACTION`. See ADR-023 / ADR-018.
     * @param hasBrakeOnGateOverride  `true` if `brakeOnGate` should overwrite the owner's
     *                            `_brakeOnGate`. Absent → owner value preserved across
     *                            re-enables.
     * @param brakeOnGate         New brake-on-gate flag; honored only when
     *                            `hasBrakeOnGateOverride`. When false AND a distance gate
     *                            is armed, the gate trip latches `_distanceReached` without
     *                            self-disabling the controller or calling `Drive.stop`.
     *                            See ADR-023.
     */
    void _applyEnableBuilder(bool                targetEnabled,
                             bool                brake,
                             bool                hasPresetOverride,
                             Config::SpeedPreset preset,
                             bool                tShapeRampUp,
                             bool                tShapeRampDown,
                             float               accel,
                             bool                hasReverseOverride,
                             bool                reverse,
                             bool                hasDistanceTarget,
                             float               distanceTargetCm,
                             bool                hasOvershootModeOverride,
                             bool                overshootMode,
                             bool                hasBrakeOnGateOverride,
                             bool                brakeOnGate);

private:
    // `presetToPwm` lives in `WROMath.h` — see Drive.h's matching note. Internal callers
    // here invoke the free function directly.

    WROLineArrayClass*  _array    = nullptr;                      ///< Pointer to the singleton WROLineArrayClass shared with Intersection (T7).
    bool                _enabled  = false;                        ///< When false, `update()` is a no-op.
    bool                _lostLine = false;                        ///< Last-tick lost-line state (no channel online).
    /// @brief True on the most recent update() tick where the junction classifier
    ///        (LEFT/RIGHT/T_OR_CROSS) suppressed the P+D mix. Reset to false on the
    ///        first non-junction tick AND in the disable path and lost-line branch
    ///        (so no flag leaks across an enable/disable cycle or a line-lost
    ///        transition). See `isPidFrozen()` for the public accessor.
    bool                _pidFrozen = false;
    float               _lastErr  = 0.0f;                         ///< Previous error sample, for the D-term.
    /// @brief Set on the lost-line and junction-freeze-exit branches so the first online
    ///        non-junction tick after a recovery seeds `_lastErr` with the fresh error
    ///        (making that tick's `dErr` ~0) instead of differencing against the reset 0,
    ///        which would inject a one-tick `LINE_KD*err` derivative kick. Consumed and
    ///        cleared by the P+D block in `update()`. See trace finding #19.
    bool                _seedLastErr = false;
    Config::SpeedPreset _preset   = Config::SpeedPreset::NORMAL;  ///< Active cruise preset; sourced by `update()`.
    bool                _reverse  = false;                        ///< When true, `update()` negates cruise to follow line in reverse.
    bool                _overshootMode = false;                   ///< When true, isOnline()==false dead-reckons symmetric cruise (no LOST_LINE_ACTION). See ADR-023.
    bool                _brakeOnGate   = true;                    ///< When false, distance-gate trip latches _distanceReached without self-disabling. See ADR-023.

    // Trapezoidal cruise-ramp state (`.tShape()` on the enable builder).
    bool    _useTShape       = false;    ///< If true, cruise PWM ramps over distance after enable.
    float   _tShapeAccel     = Config::TSHAPE_ACCEL_PWM_PER_CM;
    int32_t _tShapeStartEnc  = 0;        ///< Encoder reading at enable.
    int16_t _tShapeStartPwm  = 0;        ///< Starting cruise (typically 0).
    int16_t _tShapeTargetPwm = 0;        ///< Final cruise (resolved from preset).

    // Trapezoidal cruise ramp-DOWN state (used when a distance target is armed and
    // `_useTShapeRampDown` is set). The ramp-down lerps cruise from `_tShapeRampDownStart`
    // down to `Config::PWM_MIN_MAGNITUDE` (sign-corrected for reverse) as the chassis
    // approaches the distance gate, so the active brake fires at floor PWM rather than from
    // full cruise. See DECISIONS.md "LineFollower distance-gated stops use a symmetric
    // trapezoidal profile".
    bool    _useTShapeRampDown   = false;    ///< Set when ramp-down should apply this distance run.
    int16_t _tShapeRampDownStart = 0;        ///< Cruise PWM captured at gate-arm time.

    // Distance-target gate state (`.setDistanceTarget(cm)` on the enable builder, used by
    // PathTracker's PROJECTION_FOLLOW state). When armed, `update()` evaluates the average
    // absolute encoder delta after the PID branch and self-disables (active brake) on hit.
    bool    _distanceTargetActive = false;    ///< Gate armed.
    float   _distanceTargetCm     = 0.0f;     ///< Target travel from baseline.
    int32_t _distanceStartEncL    = 0;        ///< Left encoder reading at arm.
    int32_t _distanceStartEncR    = 0;        ///< Right encoder reading at arm.
    bool    _distanceReached      = false;    ///< One-shot latch; cleared by isDistanceReached().

    /// @brief Optional cooperative-tick callback invoked from the sync-blocking follow
    ///        helpers so other modules (notably `LineArray` when tick-cache mode is
    ///        enabled) keep running during the wait. Registered via `setTickCallback()`.
    ///        `nullptr` until set; falls back to ticking `LineFollower.update()` (and
    ///        `Intersection.update()` in the intersection-wait path) alone with a one-time
    ///        Serial warning.
    void (*_tickCallback)() = nullptr;
    /// @brief One-shot guard so the missing-callback warning is printed only once per
    ///        registration cycle (cleared by `setTickCallback`).
    bool _tickCallbackWarningEmitted = false;
};

/**
 * @brief Single global instance — defined in LineFollower.cpp.
 */
extern LineFollowerClass LineFollower;

#endif    // LINEFOLLOWER_H
