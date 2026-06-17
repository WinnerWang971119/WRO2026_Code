#ifndef CONFIG_NAV_H
#define CONFIG_NAV_H

#include <stdint.h>

/**
 * @brief PathTracker navigation configuration — reopens `namespace Config` per ADR-031.
 *
 * Holds the PathTracker goal queue + drain timeout + heading tolerance, the pass-through
 * collinear-intersection toggle, and the bidirectional auto-selection kill-switch.
 * Self-contained (only `<stdint.h>`): every value here is a literal with no cross-file
 * constexpr dependency.
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
}    // namespace Config

#endif    // CONFIG_NAV_H
