#ifndef CONFIG_NAV_H
#define CONFIG_NAV_H

#include <stdint.h>

/**
 * @brief PathTracker navigation configuration — reopens `namespace Config` per ADR-031.
 *
 * Holds the PathTracker goal queue + drain timeout + heading tolerance, the pass-through
 * collinear-intersection toggle, the bidirectional auto-selection kill-switch, and the
 * Localization operator-offset integrator constants (ADR-042). Self-contained (only
 * `<stdint.h>`): every value here is a literal with no cross-file constexpr dependency.
 */
namespace Config
{
    // =========================================================================================
    // PathTracker queue + drain timeout + heading tolerance (Phase 2 PR2).
    //
    // PathTracker.goTo() returns a fluent builder that auto-enqueues a goal on the FIFO. The
    // destructor sync-blocks by default (drains the queue via the registered cooperative tick
    // callback) so source-order chains like
    //
    //   PathTracker.goTo("C");
    //   PathTracker.goTo("G");
    //
    // execute one-after-the-other with no caller polling. Chain `.async()` on the builder to
    // opt out (auto-planner case where the caller ticks loop() and polls PathTracker.isIdle()).
    //
    // See docs/plans/2026-05-06-phase2-pr2-path-tracker-runtime.md and
    // user_fluent_api_preference.md for the rationale.
    // =========================================================================================

    /**
     * @brief Maximum number of pending goTo() entries the PathTracker queue can hold.
     *
     * Bumped 8→32 to accommodate missions chaining 10+ destinations; trivial RAM cost
     * (32 bytes since GoalEntry/NavEntry is a single uint8_t). Overflow drops the new
     * entry and prints a Serial warning — that is a caller bug.
     */
    constexpr uint8_t PATH_QUEUE_CAPACITY = 32;

    /**
     * @brief Per-goal worst-case ceiling used by `PathTracker._drainBlocking` to bound
     *        the synchronous spin (mirrors `DRIVE_DRAIN_TIMEOUT_MS_PER_CMD`).
     *
     * 60 s per goal accommodates a worst-case multi-edge traverse on the toy graph at slow
     * cruise. Drain budget is `(qCount + 1) * PATH_DRAIN_TIMEOUT_MS_PER_GOAL`. On expiry,
     * the chassis is stopped, the queue cleared, and a fault is latched.
     */
    constexpr uint32_t PATH_DRAIN_TIMEOUT_MS_PER_GOAL = 60000;

    /**
     * @brief Grace window (ms) of CONTINUOUS lost-line before PathTracker escalates to
     *        `PathTrackerClass::Fault::LINE_LOST` (issue #146).
     *
     * While the FSM is line-following (`State::LINE_FOLLOW`), PathTracker polls
     * `LineFollower.isLineLost()` each tick. A loss that persists for at least this many
     * milliseconds — measured from the first tick the line drops and reset the instant it
     * reacquires — is treated as the chassis having wandered off the line, and PathTracker
     * latches `Fault::LINE_LOST`. A shorter, transient loss (a scuffed segment, a junction
     * saturation) never trips the fault.
     *
     * @note This is a BOUNDED fault ESCALATION only, not the deferred ADR-009 lost-line
     *       sweep/reacquire/resume recovery (see docs/decisions/009-*.md). The poll runs
     *       ONLY in `State::LINE_FOLLOW`; `State::LINE_OVERSHOOT` is a deliberate dead-reckon
     *       past the line and is not polled, and the poll is suppressed during a
     *       PathTracker-owned pass-through window (`_passThrough` / `_projPassThrough`).
     * @note TODO bench-tune. 500 ms is a placeholder: long enough to ride out a saturated
     *       junction or a brief scuff at cruise, short enough to catch a genuine departure
     *       before the chassis drives far off-map. Retune once the real segment quality and
     *       cruise speed are measured.
     * @note Consumer: `PathTracker`'s `LINE_FOLLOW` lost-line escalation (issue #146).
     */
    constexpr uint32_t LINE_LOST_FAULT_MS = 500;   // TODO bench-tune

    /**
     * @brief Settle dwell (ms) held after a stopped hop before the next maneuver.
     *
     * After the chassis brakes to rest at a node, `PathTracker` holds the FSM idle in
     * `State::SETTLE` (brakes engaged, non-blocking, `Drive` untouched) for this many
     * milliseconds before dispatching the next pre-edge turn/move — giving residual
     * inertia time to dissipate before a turn or drive re-anchors against it.
     *
     * @note `0` DISABLES the feature entirely: `State::SETTLE` is never entered, and the
     *       dispatched primitive sequence is byte-identical to pre-feature. This is the
     *       single rollback knob.
     * @note Settle is SKIPPED (regardless of this value) on: a LINE/projection
     *       pass-through, an ADR-059 blind-chain intermediate hop, a goal's first-plan
     *       pre-maneuver dwell (standing start), arrival at a `RESET` node, and the
     *       return-to-node prologue's own turn/move (and the wall interludes). See
     *       ADR-061 for the full fire/skip matrix.
     * @note **Compile-time override:** guarded by `#ifndef HOP_SETTLE_MS_DEFAULT` so a
     *       host test can pin it target-wide via `-DHOP_SETTLE_MS_DEFAULT=0` (disable) or
     *       another value without editing this file — same idiom as
     *       `PATH_PASS_THROUGH_ENABLED_DEFAULT` above.
     * @note CTPS (`tools/path_planner/ctps/track_builder.py`) parses this value directly
     *       from this source file via `config_parse` (never hand-mirrored, ADR-058) to
     *       render a matching dwell keyframe at each settle seam. See ADR-061.
     * @note Consumer: `PathTracker`'s SETTLE state (ADR-061).
     */
#ifndef HOP_SETTLE_MS_DEFAULT
#define HOP_SETTLE_MS_DEFAULT 100
#endif
    constexpr uint32_t HOP_SETTLE_MS = HOP_SETTLE_MS_DEFAULT;   // TODO bench-tune

    /**
     * @brief Tolerance (degrees) below which `PathTracker` skips an absolute turn dispatch.
     *
     * Avoids a sub-degree twitch at every edge join when consecutive edges happen to be
     * colinear (e.g. mid-segment projection-node hand-off). 0.5 deg is well below the
     * `Drive.turnToYaw` resolvable range.
     */
    constexpr float NAV_TURN_TOLERANCE_DEG = 0.5f;

    /**
     * @brief Whether PathTracker passes straight through collinear intersections
     *        without braking.
     *
     * When `true` (default), at each LINE-edge intersection PathTracker peeks the
     * next-hop edge after `_nextHop` (toward `_destinationNode`). If the next edge's
     * heading delta vs. the inbound edge is within `NAV_TURN_TOLERANCE_DEG` and the
     * intersected node is not the final destination, the chassis traverses the node
     * at cruise PWM with no ramp-down → brake → ramp-up cycle. See ADR-023
     * "Pass-through collinear intersections" for the full rule.
     *
     * @note Setting this to `false` reverts to today's stop-and-go behavior at every
     *       intersection regardless of geometry. Useful as the Level-1 rollback knob
     *       if pass-through regresses on hardware. The companion
     *       `LineFollower::setOvershootMode(true)` (project-wide overshoot-mode rule
     *       per ADR-023) remains active independently of this flag — it is rolled
     *       back via code change, not a Config toggle.
     * @note The Python sim (`tools/path_planner/sim/runtime.py`) hand-mirrors this
     *       constant. A pytest verifies parity via regex grep on this header.
     * @note **Compile-time override:** the default value below is guarded by an
     *       `#ifndef PATH_PASS_THROUGH_ENABLED_DEFAULT` so a build can flip the
     *       behavior via `-DPATH_PASS_THROUGH_ENABLED_DEFAULT=false` on the
     *       compiler command line — used by the `WRO2026/test/` host-compile
     *       harness to exercise both branches of the AC7b behavioral toggle
     *       without editing this file. The public `Config::PATH_PASS_THROUGH_ENABLED`
     *       symbol is unchanged; only its initializer is overridable.
     */
#ifndef PATH_PASS_THROUGH_ENABLED_DEFAULT
#define PATH_PASS_THROUGH_ENABLED_DEFAULT true
#endif
    constexpr bool PATH_PASS_THROUGH_ENABLED = PATH_PASS_THROUGH_ENABLED_DEFAULT;

    /// Kill-switch for bidirectional line-following auto-selection.
    ///
    /// Scope: gates ONLY the auto branch of `PathTracker::_resolveDirection`
    /// (Step 5 — turn-saving REAR selection when |body delta| exceeds
    /// `BIDIRECTIONAL_TURN_THRESHOLD_DEG`). Operator overrides (per-edge
    /// `direction_override`, pipeline default, builder `setRouteDirection` /
    /// `setArrivalDirection`) remain effective regardless of this flag.
    ///
    /// Renamed from `BIDIRECTIONAL_ENABLED` (PR #34 review) — the original
    /// name was misleading ("master kill-switch") because overrides bypass
    /// it; the `_AUTO_` infix matches the actual scope.
#ifndef BIDIRECTIONAL_AUTO_ENABLED_DEFAULT
#define BIDIRECTIONAL_AUTO_ENABLED_DEFAULT true
#endif
    constexpr bool BIDIRECTIONAL_AUTO_ENABLED = BIDIRECTIONAL_AUTO_ENABLED_DEFAULT;

    /// Angular threshold (degrees) above which PathTracker selects REAR
    /// line-following instead of performing a wide on-the-spot turn.
    constexpr float BIDIRECTIONAL_TURN_THRESHOLD_DEG = 90.0f;

    // `Config::IMU_YAW_SIGN` (the app->STM32 write-path yaw sign, applied only by the retired
    // `YawSource::toHwHeading`) was RETIRED in issue #169 (ADR-075): GYRO-straight heading-hold
    // moved to an R4-side PD, so nothing writes a heading target to the STM32 gyro PID anymore.
    // The surviving read-path sign is `Config::GYRO_YAW_SIGN`, declared below (ADR-074).

    // =========================================================================================
    // External gyro (MatrixColor board) raw-read config (ADR-074).
    //
    // The external gyro is a THIRD, physically independent I2C board masquerading as an MXColor
    // device (addr 0x22) that packs a signed 16-bit heading into its G/R registers. `YawSource`
    // is the sole reader of these five constants, decoding via
    // `GyroDecodePolicy.h::decodeGyroYawDeg(hiG, loR, GYRO_SCALE, GYRO_YAW_SIGN)`.
    //
    // `GYRO_YAW_SIGN` converts the gyro's raw decoded heading into the app CCW-positive frame. It
    // is now the SOLE yaw sign: the app -> STM32-raw write-path sign `IMU_YAW_SIGN` was retired
    // with `YawSource::toHwHeading` and the `MoveGyro*` write path (issue #169, ADR-075). The
    // external gyro is its own device with its own handedness, distinct from the STM32 IMU the
    // old read-path sign served — exactly the "third frame crossing" ADR-045's Trigger to revisit
    // anticipated. Apply this sign only inside its one documented adapter point.
    // =========================================================================================

    /**
     * @brief I2C port label the external gyro's `MatrixColor` board answers on.
     *
     * A port LABEL (I2C1..I2C4), matching the `LINE_FRONT_I2C_PORT` / `LINE_REAR_I2C_PORT`
     * convention — the 0-based PCA9548A mux channel is `GYRO_I2C_PORT - 1`; do NOT pre-convert
     * here.
     *
     * @note Default `3` (I2C3) is the mentor's wiring; the line boards sit on I2C1/I2C4 so
     *       there is no conflict out of the box. A rewire is a one-constant change.
     * @note A `static_assert` in `ConfigChecks.h` enforces the value is in `[1,4]` and distinct
     *       from both `LINE_FRONT_I2C_PORT` and (when `LINE_BOARD_COUNT == 2`)
     *       `LINE_REAR_I2C_PORT`.
     * @note Consumer: `YawSource` (the I2C mux wiring in `begin()`).
     */
#ifndef GYRO_I2C_PORT_DEFAULT
#define GYRO_I2C_PORT_DEFAULT 3
#endif
    constexpr uint8_t GYRO_I2C_PORT = GYRO_I2C_PORT_DEFAULT;

    /**
     * @brief Positive scale calibration applied to the gyro's raw decoded heading.
     *
     * @note Measured on this chassis's gyro via the `inoTest/gyro_calib/` SCALE phase (2026-07),
     *       replacing the mentor's `0.997f` placeholder. Re-run that sketch and re-paste if the
     *       gyro board is swapped.
     * @note Deliberately carries NO value-pinning tripwire (unlike a tuned drive constant): this
     *       constant is *meant* to be overwritten by the calibration output, not defended once
     *       set to a specific number.
     * @note A `static_assert` in `ConfigChecks.h` enforces only a sane band around `1.0f` — a
     *       structural sanity gate, not a pin to this specific placeholder value.
     * @note Consumer: `GyroDecodePolicy.h::decodeGyroYawDeg`, via `YawSource`.
     */
#ifndef GYRO_SCALE_DEFAULT
#define GYRO_SCALE_DEFAULT 0.9945f
#endif
    constexpr float GYRO_SCALE = GYRO_SCALE_DEFAULT;

    /**
     * @brief Sign that converts the external gyro's raw decoded heading into the app
     *        CCW-positive frame.
     *
     * Applied EXCLUSIVELY inside `GyroDecodePolicy.h::decodeGyroYawDeg` (via `YawSource`); no
     * other module may apply it — a secondary application would double-negate the sign and
     * reverse all gyro-sourced heading math, the same one-place rule ADR-045 established for the
     * (now-retired) STM32 write-path sign.
     *
     * @note Confirmed `-1.0f` on this chassis's gyro via the `inoTest/gyro_calib/` SIGN phase
     *       (2026-07): the mentor's block code carries no sign, so handedness could only be read
     *       off the bench. Re-run that sketch and re-paste if the gyro board is swapped.
     * @note A `static_assert` in `ConfigChecks.h` enforces the value is exactly `+1.0f` or
     *       `-1.0f` — a pure-sign structural gate.
     * @note Consumer: `GyroDecodePolicy.h::decodeGyroYawDeg`, via `YawSource` only.
     */
#ifndef GYRO_YAW_SIGN_DEFAULT
#define GYRO_YAW_SIGN_DEFAULT (-1.0f)
#endif
    constexpr float GYRO_YAW_SIGN = GYRO_YAW_SIGN_DEFAULT;

    /**
     * @brief Plausibility rate bound (deg/s) for the gyro's torn-read gate.
     *
     * The rate `GyroDecodePolicy.h::gyroReadPlausible` would use to reject a new reading whose
     * time-normalized angular step from the last good read exceeds it — a defence against a torn
     * I2C read decoding to a wildly-wrong-but-in-range heading.
     *
     * @note Default `1000.0f` deg/s is roughly 8 deg per ~8 ms cooperative tick — well above any
     *       rotation rate this chassis can physically achieve.
     * @warning **Currently UNCONSUMED at runtime.** The plausibility gate was deferred to bench
     *          (issue #170); `YawSource` gates reads by dead-byte detection only. `gyroReadPlausible`
     *          remains as pure, unit-tested code, and this constant is reserved for re-adding the
     *          gate if the bench shows a real torn read.
     */
#ifndef GYRO_MAX_STEP_DEG_PER_S_DEFAULT
#define GYRO_MAX_STEP_DEG_PER_S_DEFAULT 1000.0f
#endif
    constexpr float GYRO_MAX_STEP_DEG_PER_S = GYRO_MAX_STEP_DEG_PER_S_DEFAULT;

    /**
     * @brief Staleness backstop (ms) for `YawSource`'s tick-scoped yaw cache.
     *
     * `YawSource::update()` refreshes the cache once per cooperative tick; a consumer that reads
     * before the first tick, or after `update()` has stopped being called, would otherwise be
     * served an arbitrarily stale value forever. `readYawDeg` refreshes inline whenever the
     * cache is absent or older than this many milliseconds.
     *
     * @note Default `12` ms is roughly 1.5x the measured ~8 ms cooperative tick — enough slack
     *       to absorb normal tick jitter without masking a genuinely stuck `update()` loop.
     * @note Consumer: `YawSource::readYawDeg` (the inline refresh-on-stale path).
     */
#ifndef GYRO_CACHE_MAX_AGE_MS_DEFAULT
#define GYRO_CACHE_MAX_AGE_MS_DEFAULT 12
#endif
    constexpr uint8_t GYRO_CACHE_MAX_AGE_MS = GYRO_CACHE_MAX_AGE_MS_DEFAULT;

    // =========================================================================================
    // Localization operator-offset integrator (ADR-042).
    //
    // The `Localization` 2D operator-offset delta (Pillar 1) accumulates how far the chassis
    // is hand-driven away from its last anchored node, gated by per-channel deadbands and
    // bounded by a return trigger / distance cap + off-map policy. Folded here from the former
    // standalone `ConfigLocalization.h` so the per-domain split stays one file per concern.
    //
    // @note **This sub-section is Pillar 1 (operator-offset) only.** The Pillar 2 wall-reset /
    //       wall-normal constants live in their own "Wall-normal strict sanity check" section
    //       further down this file — keep the two groups separate; do not interleave them here.
    // @note Some constants below have no consumer in PR1a (operator-offset integrator); they
    //       are authored for PR1b (return-to-node prologue) so that PR needs no Config change.
    //       Each such constant is marked `Forward-looking: no consumer in PR1a`.
    // @note **Every numeric value here is bench-tunable** — treat the defaults as starting
    //       points until measured on the physical chassis.
    // =========================================================================================

    /**
     * @brief Per-tick net translation (cm) below which wheel motion is treated as encoder noise.
     *
     * The offset integrator accumulates wheel-encoder deltas each cooperative tick.
     * When the per-tick net translation magnitude (|(ΔL + ΔR) / 2|) falls below this
     * floor, the tick is silently dropped from the translation channel — it is too
     * small to distinguish real motion from encoder quantisation noise. The rotation
     * channel is gated INDEPENDENTLY via `OFFSET_ROTATION_DEADBAND_DEG`.
     *
     * @note TODO: bench-tune. Suggested: 0.2f cm (~half a wheel-encoder count at
     *       typical cruise). Tighten toward 0 to capture slower drift; loosen to
     *       reduce noise accumulation.
     * @note Consumer: the offset integrator's translation channel (PR1a).
     */
    constexpr float OFFSET_TRANSLATION_DEADBAND_CM = 0.2f;

    /**
     * @brief Per-tick IMU-yaw delta (deg) below which rotation is treated as IMU noise.
     *
     * When the per-tick absolute IMU-yaw change falls below this floor, the tick is
     * silently dropped from the rotation channel. Gated INDEPENDENTLY of the
     * translation deadband — a tick can contribute to translation without contributing
     * to rotation, and vice versa.
     *
     * @note TODO: bench-tune. Suggested: 0.5f deg (~one IMU LSB at typical noise floor).
     *       Tighten toward 0 to capture slow yaw creep; loosen to suppress sensor drift
     *       during straight-line driving.
     * @note Consumer: the offset integrator's rotation channel (PR1a).
     */
    constexpr float OFFSET_ROTATION_DEADBAND_DEG = 0.5f;

    /**
     * @brief Offset magnitude (cm) above which a `goTo()` call first returns to the
     *        anchored node before planning the requested path.
     *
     * When the accumulated operator offset vector's magnitude exceeds this threshold,
     * PathTracker inserts a dead-reckoned return move back onto the anchored node before
     * dispatching the caller's goal. Below this threshold `goTo()` behaves exactly as
     * today — no return move is inserted.
     *
     * @note TODO: bench-tune. Suggested: 3.0f cm (roughly half the axle-offset measurement
     *       uncertainty). Raise to tolerate larger pushes before triggering a return;
     *       lower to recover from smaller deviations.
     * @note Must be greater than `OFFSET_TRANSLATION_DEADBAND_CM` — if the trigger sits
     *       below or at the deadband, accumulated offset can never reach the trigger and
     *       the return-to-node branch is permanently unreachable. Enforced by static_assert
     *       below.
     * @note Forward-looking: no consumer in PR1a. Used by the PR1b return-to-node prologue.
     */
    constexpr float RETURN_TRIGGER_THRESHOLD_CM = 3.0f;

    /**
     * @brief Maximum offset magnitude (cm) that can be trusted for a dead-reckoned return.
     *
     * Above this cap the robot is too far off-node for dead reckoning to reliably drive
     * back — the accumulated position error makes the return vector untrustworthy. When
     * the offset exceeds this cap, the PR1b off-map policy decides whether to fault or to
     * attempt the return anyway.
     *
     * @note TODO: bench-tune. Suggested: 50.0f cm (roughly one node-spacing on a typical
     *       WRO field). Raise if the robot is allowed to wander further before needing to
     *       recover; lower to be more conservative about trusting dead reckoning.
     * @note Must be ≥ `RETURN_TRIGGER_THRESHOLD_CM` — you can only trigger a return for
     *       offsets you still trust to drive back. Enforced by static_assert below.
     * @note Forward-looking: no consumer in PR1a. Used by the PR1b return-to-node prologue.
     */
    constexpr float RETURN_DISTANCE_CAP_CM = 50.0f;

    // ---------------------------------------------------------------------------------------
    // Off-map gate policy (PR1b — read by the return-to-node prologue)
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Policy applied when the accumulated offset exceeds `RETURN_DISTANCE_CAP_CM`.
     *
     * - `FAULT`       — refuse to plan; latch a localization fault and surface it to the
     *                   caller. Safer default: avoids driving the chassis further off-map
     *                   on a vector the firmware can no longer trust.
     * - `BEST_EFFORT` — log a Serial warning and attempt the return anyway, using the
     *                   capped vector as a best-effort dead-reckoned target. Useful for
     *                   bench/debug sessions where losing the robot to a fault is worse
     *                   than a possibly imprecise recovery.
     */
    enum class OffMapGate : uint8_t
    {
        FAULT,
        BEST_EFFORT
    };

    /**
     * @brief Active off-map gate policy.
     *
     * @note TODO: bench-tune. Default `FAULT` (refuse-and-fault when offset > cap).
     *       Override to `OffMapGate::BEST_EFFORT` for bench use once tested.
     */
    constexpr OffMapGate OFF_MAP_GATE = OffMapGate::FAULT;

    // ---------------------------------------------------------------------------------------
    // Wall-normal strict sanity check (Pillar 2 — consumed by Localization::reanchorAtWall)
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Master switch for the wall-normal sanity check in `reanchorAtWall`.
     *
     * When `false` (default), `Localization::reanchorAtWall(...)` ALWAYS re-anchors against
     * the wall's ground truth — it trusts that a preceding `Drive.wallStop` left the chassis
     * square to the wall. When `true`, `reanchorAtWall` first compares the live IMU heading
     * against the expected wall normal (`facing`) and REFUSES to re-anchor (no mutation,
     * returns `false`) when the absolute angular difference EXCEEDS
     * `WALL_NORMAL_TOLERANCE_DEG` — the squaring is then untrustworthy and re-zeroing the
     * pose against it would inject a worse error than the drift it tried to correct.
     *
     * @note Default `false`: the check is a guard against a botched wall-square, not part of
     *       the nominal flow. Enable it once bench testing characterises how square
     *       `wallStop` actually leaves the chassis and what tolerance flags a real failure
     *       (vs. normal squaring slop) without false-rejecting good re-anchors.
     * @note TODO: bench-tune (the enable decision, paired with `WALL_NORMAL_TOLERANCE_DEG`).
     * @note Consumer: `Localization::reanchorAtWall` (Pillar 2).
     * @note Override-hook: guarded by `#ifndef WALL_NORMAL_STRICT_ENABLED_DEFAULT` so the
     *       SHIPPED source default stays `false` while the strict-ON host test
     *       (`test_reanchor_at_wall_strict`) flips it target-wide via
     *       `-DWALL_NORMAL_STRICT_ENABLED_DEFAULT=true` (same idiom as
     *       `PATH_PASS_THROUGH_ENABLED_DEFAULT` above) — avoids the cross-TU `constexpr`
     *       override trap without a per-target Config shadow header.
     */
#ifndef WALL_NORMAL_STRICT_ENABLED_DEFAULT
#define WALL_NORMAL_STRICT_ENABLED_DEFAULT false
#endif
    constexpr bool WALL_NORMAL_STRICT_ENABLED = WALL_NORMAL_STRICT_ENABLED_DEFAULT;

    /**
     * @brief Tolerance (degrees) for the wall-normal sanity check, when enabled.
     *
     * The maximum absolute angular difference, computed wrap-around-correct over `[0, 360)`,
     * between the live IMU heading and the expected wall normal that still lets
     * `reanchorAtWall` proceed. The comparison is `> tolerance` REJECTS: a difference exactly
     * EQUAL to the tolerance still re-anchors (the tolerance value is inclusive / "just
     * inside"), and only a strictly greater difference rejects. Inert while
     * `WALL_NORMAL_STRICT_ENABLED == false`.
     *
     * @note TODO: bench-tune. Suggested: 10.0f deg — wide enough to absorb the squaring slop
     *       a real `wallStop` leaves, tight enough to catch a chassis that stopped against a
     *       corner / obstacle instead of the intended wall. Tighten once the squaring
     *       repeatability is measured.
     * @note Consumer: `Localization::reanchorAtWall` (Pillar 2).
     */
    constexpr float WALL_NORMAL_TOLERANCE_DEG = 10.0f;

    /**
     * @brief Distance (cm) from the rear wall-contact face to the wheel axle (the tracked
     *        reference point).
     *
     * A wall-zero presses the chassis REAR against a known wall; the believed position must
     * therefore land on the AXLE, which sits this far IN FRONT of the contact face — not on the
     * wall surface. `Localization::reanchorAtWall` subtracts this from the wall coordinate so the
     * re-zeroed axis reads the axle, and the PathTracker reset interlude then drives
     * `standoff - WALL_CONTACT_OFFSET_CM` forward back onto the node.
     *
     * @note TODO: measure at bring-up — the empty-box rest geometry sets it (the variable / loaded
     *       box case is deferred to issue #77). Hand-mirrored in
     *       `tools/path_planner/sim/runtime.py` (added together in this change); keep in lockstep.
     * @note Consumed by `Localization::reanchorAtWall` (margin subtraction) and the PathTracker
     *       wall-depart move, both landing in this same change (Pillar 2).
     */
    constexpr float WALL_CONTACT_OFFSET_CM = 8.0f;   // TODO measure at bring-up

    /**
     * @brief Minimum node-to-wall standoff (cm) a reset node may be authored at.
     *
     * The pivot sweep radius (axle -> farthest corner, ~12.5 cm) plus a safety margin. A reset
     * node must sit at least this far from its wall so BOTH the in-place SQUARE turn that
     * reorients the chassis rear-to-wall AND any post-re-anchor pivot clear the wall. Together
     * with the static_assert below it also guarantees the return distance
     * `standoff - WALL_CONTACT_OFFSET_CM >= 0`.
     *
     * @note Enforced OFFLINE by `tools/path_planner/validator.py`, which hand-mirrors this value
     *       (Python cannot include this header) — keep the two in lockstep. The firmware does not
     *       read it at runtime; it is an authoring guardrail (and the static_assert anchor).
     * @note TODO: bench-tune once the pivot sweep radius is measured.
     */
    constexpr float WALL_PIVOT_CLEARANCE_CM = 14.0f;   // TODO bench-tune (12.5 pivot radius + safety)

    // ---------------------------------------------------------------------------------------
    // Line-arrival heading reset (wall-free) — ADR-052. Consumed by
    // `Localization::reanchorHeading` and `PathTracker::_maybeHeadingReset`; mirrored in
    // `tools/path_planner/sim/runtime.py` (T7) for sim parity.
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Master enable for the wall-free heading reset on a collinear PROJECTION
     *        pass-through.
     *
     * When `true` (default), `PathTracker::_maybeHeadingReset` fires
     * `Localization::reanchorHeading` whenever the just-completed arrival was a collinear
     * pass-through through a `NodeType::PROJECTION` node — a clean heading sample, since the
     * line-follow PID never freezes at a projection pass-through (no junction dwell). The reset
     * re-anchors the IMU-yaw offset to the incoming LINE edge's already-known `heading_deg`,
     * body-corrected, bounding IMU-yaw drift for free with no wall maneuver and no motion.
     *
     * @note Default `true`: projection pass-throughs are the low-risk case (clean heading), so
     *       the reset is expected to be safe and beneficial out of the box.
     * @note Consumer: `PathTracker::_maybeHeadingReset` (ADR-052). Mirrored in
     *       `tools/path_planner/sim/runtime.py` for parity.
     * @note **Compile-time override:** guarded by `#ifndef LINE_HEADING_RESET_ON_PROJECTION_DEFAULT`
     *       so a host test can flip it target-wide via
     *       `-DLINE_HEADING_RESET_ON_PROJECTION_DEFAULT=false` without editing this file — same
     *       idiom as `WALL_NORMAL_STRICT_ENABLED_DEFAULT` above.
     */
#ifndef LINE_HEADING_RESET_ON_PROJECTION_DEFAULT
#define LINE_HEADING_RESET_ON_PROJECTION_DEFAULT true
#endif
    constexpr bool LINE_HEADING_RESET_ON_PROJECTION = LINE_HEADING_RESET_ON_PROJECTION_DEFAULT;

    /**
     * @brief Enable for the wall-free heading reset on a collinear physical-junction (LINE)
     *        pass-through.
     *
     * When `true`, `PathTracker::_maybeHeadingReset` also fires `Localization::reanchorHeading`
     * on a collinear pass-through through a `NodeType::LINE` node (a physical intersection).
     * Unlike a projection pass-through, the junction PID-freeze window (see
     * `project-junction-pid-freeze`) can inject a small heading wiggle while the chassis
     * traverses the dwell, so this knob ships OFF for v1 until bench data characterises the
     * freeze-window envelope.
     *
     * @note Default `false`: the junction freeze-window envelope is unmeasured; ships OFF for
     *       v1. Flip ON once bench data (gathered via the Serial skip/fire log while this knob
     *       is ON) shows the wiggle is small enough that frequent slightly-noisy binds strictly
     *       dominate rare clean ones on a long run.
     * @note Consumer: `PathTracker::_maybeHeadingReset` (ADR-052). Mirrored in
     *       `tools/path_planner/sim/runtime.py` for parity.
     * @note **Compile-time override:** guarded by `#ifndef LINE_HEADING_RESET_ON_JUNCTION_DEFAULT`
     *       so a host test can flip it target-wide via
     *       `-DLINE_HEADING_RESET_ON_JUNCTION_DEFAULT=true` without editing this file — same
     *       idiom as `WALL_NORMAL_STRICT_ENABLED_DEFAULT` above.
     */
#ifndef LINE_HEADING_RESET_ON_JUNCTION_DEFAULT
#define LINE_HEADING_RESET_ON_JUNCTION_DEFAULT false
#endif
    constexpr bool LINE_HEADING_RESET_ON_JUNCTION = LINE_HEADING_RESET_ON_JUNCTION_DEFAULT;

    /**
     * @brief Master switch for the strict sanity gate in `Localization::reanchorHeading`.
     *
     * When `true` (default), `reanchorHeading` compares the live IMU heading against the
     * pre-bind belief and REFUSES to bind (no mutation, returns `false`) when the wrap-correct
     * absolute difference EXCEEDS `LINE_HEADING_RESET_TOLERANCE_DEG`. This is a gross-wrong-state
     * detector (perpendicular / kidnapped / stale-flag cases at ~90-180 deg) — at the ~10 deg
     * default it does NOT discriminate line-follow wiggle (+/-1-3 deg) from IMU drift
     * (+/-1-3 deg/min), and it (by design) passes a parallel wrong line where the bind is
     * coincidentally still correct.
     *
     * @note Default `true`: unlike the wall-normal strict check (which defaults OFF pending
     *       bench characterisation of `wallStop` squaring), this gate defaults ON — a
     *       heading-reset bind runs unattended mid-mission with no operator confirmation, so
     *       guarding against a gross misread is the safer default.
     * @note Consumer: `Localization::reanchorHeading` (ADR-052).
     * @note **Compile-time override:** guarded by
     *       `#ifndef LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT` so a host test can flip it
     *       target-wide via `-DLINE_HEADING_RESET_STRICT_ENABLED_DEFAULT=false` without editing
     *       this file — same idiom as `WALL_NORMAL_STRICT_ENABLED_DEFAULT` above.
     */
#ifndef LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT
#define LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT true
#endif
    constexpr bool LINE_HEADING_RESET_STRICT_ENABLED = LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT;

    /**
     * @brief Tolerance (degrees) for the strict sanity gate in `Localization::reanchorHeading`,
     *        when enabled.
     *
     * The maximum absolute angular difference, computed wrap-around-correct over `[0, 360)`,
     * between the pre-bind heading belief and the incoming edge's heading that still lets
     * `reanchorHeading` proceed. The comparison is `> tolerance` REJECTS: a difference exactly
     * EQUAL to the tolerance still binds (the tolerance value is inclusive / "just inside"), and
     * only a strictly greater difference rejects — matching the `WALL_NORMAL_TOLERANCE_DEG`
     * boundary convention. Inert while `LINE_HEADING_RESET_STRICT_ENABLED == false`.
     *
     * @note TODO: bench-tune. Ships as a documented default (10 deg) to be bench-calibrated
     *       later (see ADR-052's trigger-to-revisit).
     * @note Consumer: `Localization::reanchorHeading` (ADR-052).
     * @note **Compile-time override:** guarded by
     *       `#ifndef LINE_HEADING_RESET_TOLERANCE_DEG_DEFAULT` so a host test can flip it
     *       target-wide via `-DLINE_HEADING_RESET_TOLERANCE_DEG_DEFAULT=5.0f` without editing
     *       this file — same idiom as the other `_DEFAULT` hooks in this file. Range is enforced
     *       by a `static_assert` in `ConfigChecks.h` (included last, alongside the other
     *       value-range tripwires).
     */
#ifndef LINE_HEADING_RESET_TOLERANCE_DEG_DEFAULT
#define LINE_HEADING_RESET_TOLERANCE_DEG_DEFAULT 10.0f
#endif
    constexpr float LINE_HEADING_RESET_TOLERANCE_DEG = LINE_HEADING_RESET_TOLERANCE_DEG_DEFAULT;

    // Sanity static_asserts (intra-Localization; keep them here next to the constants,
    // NOT in ConfigChecks.h — they reference only the operator-offset symbols above).

    static_assert(OFFSET_TRANSLATION_DEADBAND_CM > 0.0f,
                  "Config::OFFSET_TRANSLATION_DEADBAND_CM must be positive — a zero or negative "
                  "deadband disables the encoder-noise gate and causes every idle tick to accumulate "
                  "phantom offset.");

    static_assert(OFFSET_ROTATION_DEADBAND_DEG > 0.0f,
                  "Config::OFFSET_ROTATION_DEADBAND_DEG must be positive — a zero or negative "
                  "deadband disables the IMU-noise gate and causes every idle tick to accumulate "
                  "phantom yaw offset.");

    static_assert(RETURN_TRIGGER_THRESHOLD_CM > 0.0f,
                  "Config::RETURN_TRIGGER_THRESHOLD_CM must be positive — a zero or negative "
                  "threshold would fire the return-to-node prologue on every goTo() call, even "
                  "when the robot is already on-node.");

    static_assert(RETURN_DISTANCE_CAP_CM > 0.0f,
                  "Config::RETURN_DISTANCE_CAP_CM must be positive — a zero or negative cap "
                  "would fire the off-map gate on every offset, making the return-to-node "
                  "prologue unreachable.");

    static_assert(RETURN_TRIGGER_THRESHOLD_CM <= RETURN_DISTANCE_CAP_CM,
                  "Config::RETURN_TRIGGER_THRESHOLD_CM must be <= RETURN_DISTANCE_CAP_CM - "
                  "the return-to-node prologue should only fire for offsets that the firmware "
                  "can still trust to dead-reckon back from. A trigger above the cap would "
                  "immediately invoke the off-map gate on every return attempt.");

    static_assert(WALL_NORMAL_TOLERANCE_DEG > 0.0f,
                  "Config::WALL_NORMAL_TOLERANCE_DEG must be positive — a zero or negative "
                  "tolerance would reject EVERY re-anchor when the strict check is enabled "
                  "(no real wall-square lands bit-exact on the normal), making "
                  "reanchorAtWall permanently fail under WALL_NORMAL_STRICT_ENABLED.");

    static_assert(WALL_NORMAL_TOLERANCE_DEG < 180.0f,
                  "Config::WALL_NORMAL_TOLERANCE_DEG must be < 180 — the wrap-around angular "
                  "difference is bounded by 180 deg, so a tolerance >= 180 can never be "
                  "exceeded and the strict check would accept any heading, defeating it.");

    static_assert(WALL_PIVOT_CLEARANCE_CM >= WALL_CONTACT_OFFSET_CM,
                  "Config::WALL_PIVOT_CLEARANCE_CM must be >= WALL_CONTACT_OFFSET_CM, else an "
                  "authored reset node can pass the clearance floor yet command a negative return "
                  "distance (Drive.driveDist REJECTS a negative distance with no command — the "
                  "robot never leaves the wall, then pivots into it).");

    static_assert(WALL_CONTACT_OFFSET_CM > 0.0f,
                  "Config::WALL_CONTACT_OFFSET_CM must be positive — the wheel axle sits in front "
                  "of the rear contact face, so the margin subtracted in reanchorAtWall is "
                  "strictly positive; zero or negative would place belief on or past the wall.");

    // ---------------------------------------------------------------------------------------
    // Blind-chain continuous motion (ADR-059) — consumed by `PathTracker`'s walk-ahead +
    // `ChainSpeedPolicy.h`; mirrored in `tools/path_planner/sim/runtime.py` for parity.
    //
    // COUPLED SITES (all three constants below):
    //   - `PathTracker`'s walk-ahead chain builder (decides how many hops to fold into one
    //     chained run and where the run buffer is sized).
    //   - `ChainSpeedPolicy.h`'s forward/backward two-pass resolver (the `entryPwm` seed and
    //     the per-hop `(startPwm, endPwm)` it emits assume the run stays inside these caps).
    //   - `tools/path_planner/sim/runtime.py` (hand-mirrored copies of all three; a regex-grep
    //     parity test verifies they match this header — same idiom as `PATH_PASS_THROUGH_ENABLED`
    //     above).
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Master kill-switch for blind-node continuous-motion chaining.
     *
     * When `true` (default), `PathTracker` walks ahead across consecutive collinear BLIND
     * hops and dispatches them as one chained run — `setStartSpeed`/`setEndSpeed` +
     * `setBrake(false)` on every hop but the last — instead of braking to rest at each
     * intermediate node. When `false`, PathTracker reverts to today's behavior: every blind
     * hop is its own braked stop-and-go move, regardless of collinearity.
     *
     * @note Rollback knob: flip to `false` if blind chaining regresses on hardware (e.g. the
     *       inter-hop boundary-carry subtraction under-corrects and the chain drifts off-node
     *       over N hops). This is the single flag that restores the pre-ADR-059 behavior.
     * @note Consumer: `PathTracker`'s walk-ahead chain builder (ADR-059).
     * @note The Python sim (`tools/path_planner/sim/runtime.py`) hand-mirrors this constant.
     *       A pytest verifies parity via regex grep on this header (same idiom as
     *       `PATH_PASS_THROUGH_ENABLED_DEFAULT` above).
     * @note **Compile-time override:** guarded by `#ifndef PATH_BLIND_CHAIN_ENABLED_DEFAULT`
     *       so a host test can flip it target-wide via
     *       `-DPATH_BLIND_CHAIN_ENABLED_DEFAULT=false` without editing this file — same idiom
     *       as `PATH_PASS_THROUGH_ENABLED_DEFAULT` above.
     * @warning Changing the default here changes on-hardware behavior for every mission that
     *          contains a collinear blind run. Re-verify AC7 (TC5: chaining disabled stops at
     *          every node) before shipping a default flip.
     */
#ifndef PATH_BLIND_CHAIN_ENABLED_DEFAULT
#define PATH_BLIND_CHAIN_ENABLED_DEFAULT true
#endif
    constexpr bool PATH_BLIND_CHAIN_ENABLED = PATH_BLIND_CHAIN_ENABLED_DEFAULT;

    /**
     * @brief Maximum total run distance (cm) a blind chain may span before it is forced to
     *        end (that hop brakes; a fresh chain re-forms after).
     *
     * Bounds how far the chassis dead-reckons without an intermediate braked correction.
     *
     * @note Owner decision (2026-07): this cap is now a LOOSE runaway guard, not a tight
     *       error budget. It is deliberately NOT tied to `STOP_AT_LINE_ARM_WINDOW_CM` or any
     *       other seek-arm-window concern — the practical bound on how long a blind chain can
     *       run is `MAX_CHAIN_HOPS`, not this distance cap. Set to 1000.0f (10 m, larger than
     *       any physical collinear run on a WRO field) so the cap never binds in practice; if
     *       bench testing later shows it needs to be tighter, lower it then.
     * @note Consumer: `PathTracker`'s walk-ahead chain builder (caps the run BEFORE
     *       dispatch) and `ChainSpeedPolicy.h` (the backward pass assumes the run fits this
     *       cap). Mirrored in `tools/path_planner/sim/runtime.py` for parity.
     * @note Overflow is not a fault: a run exceeding this cap ends AT the cap (that hop is
     *       dispatched as an ordinary braked stop) and PathTracker logs the truncation over
     *       Serial; a new chain may re-form on the far side. In practice `MAX_CHAIN_HOPS`
     *       will end a chain long before this distance cap is ever reached.
     * @warning If bench testing shows a blind chain that ends into a `Drive.stopAtLine()`
     *          seek drifts past that seek's `STOP_AT_LINE_ARM_WINDOW_CM` arm window, do NOT
     *          re-tie this general cap to the arm window again — add a SEPARATE, tighter
     *          bound that applies only to a chain ending into a seek. A `static_assert` in
     *          `ConfigChecks.h` enforces only a loose sanity range on this constant now (no
     *          longer arm-window-derived).
     * @note **Compile-time override:** guarded by `#ifndef PATH_BLIND_CHAIN_MAX_CM_DEFAULT`
     *       so a host test can override it target-wide via e.g.
     *       `-DPATH_BLIND_CHAIN_MAX_CM_DEFAULT=8.0f` without editing this file (same idiom
     *       as `PATH_BLIND_CHAIN_ENABLED_DEFAULT` above) — kept so existing host-test targets
     *       that need a tighter cap to exercise truncation (TC5/TC6-style tests) still can.
     */
#ifndef PATH_BLIND_CHAIN_MAX_CM_DEFAULT
#define PATH_BLIND_CHAIN_MAX_CM_DEFAULT 1000.0f
#endif
    constexpr float PATH_BLIND_CHAIN_MAX_CM = PATH_BLIND_CHAIN_MAX_CM_DEFAULT;

    /**
     * @brief Compile-time cap on the number of hops a single blind chain run may contain.
     *
     * Sizes the fixed-size firmware run buffer `PathTracker` uses to hold the walked-ahead
     * `hopDistCm[]` and resolved `ChainSpeedPolicy` output — a plain stack/member array, no
     * heap allocation, so this constant is the array dimension, not a runtime-checked limit.
     *
     * @note Sizing rule: 8 comfortably covers the toy-field longest collinear blind run
     *       (bench-observed 3-4 hops) with headroom for a denser real-field layout, while
     *       keeping the run buffer small (8 * sizeof(hop entry) is a few dozen bytes on an
     *       Arduino Uno R4). Raise only if a real mission legitimately needs a longer
     *       uninterrupted blind run AND `PATH_BLIND_CHAIN_MAX_CM` is raised in lockstep
     *       (both caps gate the same run — whichever is hit first ends the chain).
     * @note Consumer: `PathTracker`'s fixed-size chain run buffer (array dimension) and its
     *       walk-ahead loop (loop bound). Mirrored in `tools/path_planner/sim/runtime.py` for
     *       parity (the sim has no fixed-buffer constraint but honors the same cap so its
     *       output matches firmware — ADR-050 parity).
     * @note Overflow is not a fault: a run exceeding this hop count ends AT the cap (that hop
     *       is dispatched as an ordinary braked stop) and PathTracker logs the truncation; a
     *       new chain may re-form on the far side.
     * @warning This is an ARRAY DIMENSION, not just a tuning knob — raising it grows
     *          `PathTracker`'s per-instance RAM footprint by one run-buffer entry per unit.
     *          A `static_assert` in `ConfigChecks.h` enforces a sane range
     *          (`>= 2 && <= 32`); re-check available RAM before raising past the current
     *          value on a memory-constrained target.
     * @note **Compile-time override:** guarded by `#ifndef MAX_CHAIN_HOPS_DEFAULT` so a host
     *       test can pin it target-wide via `-DMAX_CHAIN_HOPS_DEFAULT=2` (e.g. to exercise
     *       the hop-cap truncation, TC6) without editing this file — same idiom as
     *       `PATH_BLIND_CHAIN_MAX_CM_DEFAULT` above. Being an array dimension, the override
     *       must be applied target-wide (`target_compile_definitions`), never per-TU.
     */
#ifndef MAX_CHAIN_HOPS_DEFAULT
#define MAX_CHAIN_HOPS_DEFAULT 8
#endif
    constexpr uint8_t MAX_CHAIN_HOPS = MAX_CHAIN_HOPS_DEFAULT;

}    // namespace Config

#endif    // CONFIG_NAV_H
