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
    // Localization operator-offset integrator (ADR-042).
    //
    // The `Localization` 2D operator-offset delta (Pillar 1) accumulates how far the chassis
    // is hand-driven away from its last anchored node, gated by per-channel deadbands and
    // bounded by a return trigger / distance cap + off-map policy. Folded here from the former
    // standalone `ConfigLocalization.h` so the per-domain split stays one file per concern.
    //
    // @note **Pillar 1 only.** Do NOT add wall-reset / wall-normal constants here — those
    //       belong to Pillar 2.
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
     */
    constexpr bool WALL_NORMAL_STRICT_ENABLED = false;

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

}    // namespace Config

#endif    // CONFIG_NAV_H
