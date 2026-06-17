#ifndef CONFIG_LOCALIZATION_H
#define CONFIG_LOCALIZATION_H

#include <stdint.h>

/**
 * @brief Localization subsystem configuration — reopens `namespace Config` per ADR-031.
 *
 * Holds the operator-offset integrator constants (Pillar 1) consumed by the
 * localization upgrade. Included by `Config.h` after `ConfigVision.h` and
 * before `ConfigChecks.h`, so the static_assert validators in ConfigChecks.h
 * see all symbols from this file.
 *
 * @note **Pillar 1 only.** Do NOT add wall-reset / wall-normal constants here —
 *       those belong to Pillar 2 and are owned by a later task (T15).
 *
 * @note Some constants below have no consumer in the first PR that lands this
 *       file (PR1a, operator-offset integrator). They are authored here so that
 *       PR1b (return-to-node prologue) can source them without a second Config
 *       change mid-flight. Each such constant is individually marked as
 *       `@note Forward-looking: no consumer in PR1a`.
 *
 * @note **Every numeric value here is bench-tunable.** Constants carry a
 *       `TODO: bench-tune` annotation — do not treat the suggested defaults
 *       as final until measured on the physical chassis.
 */
namespace Config
{
    // ---------------------------------------------------------------------------------------
    // Operator-offset integrator — deadband thresholds (PR1a consumers)
    // ---------------------------------------------------------------------------------------

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

    // ---------------------------------------------------------------------------------------
    // Return-to-node prologue — trigger and cap (PR1b consumers, forward-looking)
    // ---------------------------------------------------------------------------------------

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
     * the offset exceeds this cap, the off-map gate policy (`OFF_MAP_GATE`) decides
     * whether to fault or to attempt the return anyway.
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
    // Off-map gate policy (PR1b consumer, forward-looking)
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
     *
     * @note Default is `FAULT` (safer). Switch to `BEST_EFFORT` for bench sessions or
     *       when empirical testing shows dead reckoning is still reliable beyond the cap.
     * @note Forward-looking: no consumer in PR1a. Used by the PR1b return-to-node prologue.
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
     * @note Forward-looking: no consumer in PR1a.
     */
    constexpr OffMapGate OFF_MAP_GATE = OffMapGate::FAULT;

    // ---------------------------------------------------------------------------------------
    // Sanity static_asserts (intra-Localization; DO NOT move these to ConfigChecks.h)
    // ---------------------------------------------------------------------------------------

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

    static_assert(OFFSET_TRANSLATION_DEADBAND_CM < RETURN_TRIGGER_THRESHOLD_CM,
                  "Config::OFFSET_TRANSLATION_DEADBAND_CM must be strictly less than "
                  "RETURN_TRIGGER_THRESHOLD_CM — if the deadband sits at or above the trigger, "
                  "wheel motion is filtered out before it can accumulate to the trigger value, "
                  "making the return-to-node prologue permanently unreachable.");

    static_assert(RETURN_TRIGGER_THRESHOLD_CM <= RETURN_DISTANCE_CAP_CM,
                  "Config::RETURN_TRIGGER_THRESHOLD_CM must be <= RETURN_DISTANCE_CAP_CM - "
                  "the return-to-node prologue should only fire for offsets that the firmware "
                  "can still trust to dead-reckon back from. A trigger above the cap would "
                  "immediately invoke the off-map gate on every return attempt.");

}    // namespace Config

#endif    // CONFIG_LOCALIZATION_H
