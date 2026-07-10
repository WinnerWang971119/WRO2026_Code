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

    // =========================================================================================
    // IMU yaw sign — hardware CW-positive → app CCW-positive conversion (ADR-045).
    //
    // The STM32 coprocessor reports yaw in a CW-positive frame; the app world frame is
    // CCW-positive (standard mathematical convention).  A single multiplicative sign
    // applied at the two adapter points in `YawSource` bridges the two frames:
    //
    //   • STM32 → app  : `YawSource::readYawDeg()`  returns  raw * IMU_YAW_SIGN
    //   • app  → STM32 : `YawSource::toHwHeading()`  converts appHeading * IMU_YAW_SIGN
    //
    // ALL other modules (PathTracker, Drive.turnToYaw, Localization, this Config file)
    // work EXCLUSIVELY in the app CCW-positive frame and must NEVER apply this sign
    // themselves — doing so would double-negate the conversion.
    //
    // Default `-1.0f` matches the current competition chassis (CW-positive raw IMU).
    // The bench / test rover may differ — override at compile time via
    //   `-DIMU_YAW_SIGN_DEFAULT=1.0f`
    // without editing this file.  A static_assert in ConfigChecks.h enforces the value
    // is exactly ±1.0f (that tripwire lives in ConfigChecks.h — just referenced here).
    // =========================================================================================

    /**
     * @brief Per-chassis sign that converts raw STM32 IMU yaw (CW-positive) into the
     *        app/world frame (CCW-positive).
     *
     * The STM32 coprocessor emits yaw values in a clockwise-positive convention.
     * The application convention — used throughout PathTracker, Drive.turnToYaw,
     * Localization, and all authored field headings — is counter-clockwise-positive
     * (standard mathematical).  Multiplying by `IMU_YAW_SIGN` at exactly two points
     * inside `YawSource` bridges the frames:
     *
     *   - **STM32 → app** (`YawSource::readYawDeg`):
     *       `appYaw = rawYaw * IMU_YAW_SIGN`
     *   - **app → STM32** (`YawSource::toHwHeading`):
     *       `hwHeading = appHeading * IMU_YAW_SIGN`
     *
     * **No other module applies this sign.** `YawSource` is the sole conversion
     * boundary; any secondary application would double-negate the sign and reverse
     * all heading math.
     *
     * @note Default `-1.0f` is correct for the current competition chassis where the
     *       raw IMU yaw increases clockwise.  If a second chassis has a CCW-positive
     *       raw IMU, override to `1.0f` via `-DIMU_YAW_SIGN_DEFAULT=1.0f` at the
     *       compiler command line — this leaves the shipped source unchanged.
     * @note A `static_assert` in `ConfigChecks.h` enforces the value is exactly
     *       `+1.0f` or `-1.0f` at compile time.  Any other value (e.g. a scale
     *       factor accidentally placed here) is rejected immediately.
     * @note **Compile-time override:** guarded by `#ifndef IMU_YAW_SIGN_DEFAULT` so
     *       a build or host test can pass `-DIMU_YAW_SIGN_DEFAULT=1.0f` to exercise
     *       the opposite-sign chassis without editing this file — same idiom as
     *       `PATH_PASS_THROUGH_ENABLED_DEFAULT` above.
     * @note Consumer: `YawSource` only — see note above.
     */
#ifndef IMU_YAW_SIGN_DEFAULT
#define IMU_YAW_SIGN_DEFAULT (-1.0f)
#endif
    constexpr float IMU_YAW_SIGN = IMU_YAW_SIGN_DEFAULT;

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
     * Bounds how far the chassis dead-reckons without an intermediate braked correction. A
     * chain immediately preceding a `Drive.stopAtLine()` seek ends before the seek (seek
     * nodes are chain-ineligible — see ADR-059), so the residual dead-reckoning error
     * accumulated over the chain must still land within the FOLLOWING seek's
     * `STOP_AT_LINE_ARM_WINDOW_CM` arm window, or the seek's `T_OR_CROSS` detector can arm too
     * late and miss the real line (→ runs to max → `LINE_NOT_FOUND`).
     *
     * @note Sizing rule (load-bearing): MUST be `<= STOP_AT_LINE_ARM_WINDOW_CM`
     *       (`ConfigDrive.h`, currently 10.0f cm) — that is the full error budget the
     *       following seek can absorb. Sized conservatively at 8.0f cm (a 2 cm margin under
     *       the 10 cm window) so a chain's residual dead-reckon error, even near the
     *       worst-case end of the seek's own tolerance, still leaves room for the seek's
     *       normal under-travel slack. Do not raise this to equal the window exactly —
     *       the margin is intentional headroom, not slack to reclaim.
     * @note Consumer: `PathTracker`'s walk-ahead chain builder (caps the run BEFORE
     *       dispatch) and `ChainSpeedPolicy.h` (the backward pass assumes the run fits this
     *       cap). Mirrored in `tools/path_planner/sim/runtime.py` for parity.
     * @note Overflow is not a fault: a run exceeding this cap ends AT the cap (that hop is
     *       dispatched as an ordinary braked stop) and PathTracker logs the truncation over
     *       Serial; a new chain may re-form on the far side.
     * @warning Raising this value without re-checking `STOP_AT_LINE_ARM_WINDOW_CM` breaks the
     *          seek arm-window budget invariant this constant exists to protect. A
     *          `static_assert` in `ConfigChecks.h` enforces
     *          `PATH_BLIND_CHAIN_MAX_CM <= STOP_AT_LINE_ARM_WINDOW_CM` at compile time, but
     *          that only catches the boundary case — it does NOT prove the chain's actual
     *          dead-reckoning error stays under budget on hardware. Re-verify AC9 (bench) on
     *          any change.
     * @note **Compile-time override:** guarded by `#ifndef PATH_BLIND_CHAIN_MAX_CM_DEFAULT`
     *       so a host test can widen it target-wide via
     *       `-DPATH_BLIND_CHAIN_MAX_CM_DEFAULT=10.0f` without editing this file (same idiom
     *       as `PATH_BLIND_CHAIN_ENABLED_DEFAULT` above). Load-bearing for the blind-chain
     *       host ctest: with the shipped 8.0f cap and `TSHAPE_MIN_MOVE_CM` (5.0f), two
     *       eligible hops always sum past the cap, so a test target must widen it (within
     *       the `ConfigChecks.h` arm-window ceiling) for a chain to form at all.
     */
#ifndef PATH_BLIND_CHAIN_MAX_CM_DEFAULT
#define PATH_BLIND_CHAIN_MAX_CM_DEFAULT 8.0f
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
