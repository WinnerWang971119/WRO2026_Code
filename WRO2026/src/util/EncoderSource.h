/**
 * @file EncoderSource.h
 * @brief Sole adapter over the lower-MCU per-motor encoder + drive task-done reads.
 * @author WRO 2026
 *
 * @details
 * `EncoderSourceClass` is the single owner of all lower-MCU encoder-degree and
 * drive-task-done I/O. The vendor read paths (`MiniR4DC::getDegrees` /
 * `MiniR4DriveDC::isPrevTaskDone`) DISCARD the `MMLower` RESULT, so a lower-MCU comms
 * timeout surfaces as a silent `0` (encoder) or a fabricated `true` (task-done) — a
 * corrupted encoder delta or a phantom "move complete". `EncoderSource` reads the vendor
 * two-channel APIs (`mmL.GetEncoderDegrees` / `mmL.Get_Drive_isTaskDone`) directly so the
 * RESULT is visible, and offers two read policies (mirroring `YawSource`, ADR-045):
 *
 *  - **Surfacing** (`readDegrees`): retry-once, then leave the caller's out-param UNTOUCHED
 *    on a hard failure. For fresh baseline / arm / calibration reads that MUST be accurate —
 *    a stale `_profileStartEnc`-style baseline would corrupt every later delta, so the caller
 *    refuses to arm rather than arm on garbage.
 *  - **Last-good** (`readDegreesOr`): return the centralized last-good on a hard failure. For
 *    continuous completion checks where a one-tick-stale value is safe.
 *
 * A per-motor last-good cache (populated on every successful read, invalidated by
 * `resetCache`) backs the last-good policy. `resetCache` MUST be called wherever a
 * `resetCounter()` re-zeros a motor, else a post-reset read could serve the pre-reset value.
 * A genuine `0` returned with a SUCCESS result is always cached/served as `0` — a success-0
 * is never mistaken for a failure.
 *
 * Single-owner rule (per ADR-066): no other module reads `mmL.GetEncoderDegrees` /
 * `mmL.Get_Drive_isTaskDone` or interprets the encoder / task-done RESULT. `Drive`,
 * `LineFollower` (via `Drive.getEncL/getEncR`), and `ManipBelt` route all such I/O here.
 *
 * This header is intentionally LIGHT — it pulls in only `<stdint.h>` so dependents
 * (`Drive.cpp`, `ManipBelt.h`) can include it cheaply without dragging in MatrixMiniR4 or
 * `Config.h`. The lower-MCU dependency lives entirely in `EncoderSource.cpp`.
 */

#ifndef ENCODER_SOURCE_H
#define ENCODER_SOURCE_H

#include <stdint.h>

/**
 * @brief Sole adapter over the lower-MCU per-motor encoder + drive task-done reads —
 *        surfaces the `MMLower` RESULT the vendor read paths discard (ADR-066).
 *
 * @note Single global instance `EncoderSource` (declared `extern` below, defined in the
 *       .cpp). Construct nothing else; all consumers share this instance so the last-good
 *       cache is shared across every reader of a given motor id.
 * @note No other module performs raw encoder / task-done I/O or interprets its RESULT —
 *       see the file header and ADR-066.
 */
class EncoderSourceClass
{
public:
    /**
     * @brief Lower-MCU drive-instance id passed to the vendor task-done API.
     *
     * `MiniR4.DriveDC` is `MiniR4DriveDC<1>`, whose `_id` is `1`; the vendor
     * `isPrevTaskDone()` forwards that id to `mmL.Get_Drive_isTaskDone(_id, ...)`. Named
     * here so `isPrevTaskDone()` passes the same instance the differential-drive base uses.
     */
    static constexpr uint8_t DRIVE_INSTANCE_ID = 1;

    /**
     * @brief Read a motor's cumulative encoder degrees, surfacing the lower-MCU result.
     *
     * Reads via `mmL.GetEncoderDegrees` (retry-once after a short backoff on a transient
     * non-OK result, per the MMLower-caller convention). The value is written to @p out
     * **only** on a successful read, and the per-motor last-good cache is refreshed.
     *
     * @param motorId  M-bus channel `1..4`. An out-of-range id fails without a bus read.
     * @param[out] out Encoder degrees (signed). Left UNTOUCHED on a hard failure (both
     *                 attempts non-OK) so the caller's prior value is preserved — a failed
     *                 baseline never arms a move on `0`-derived garbage.
     * @return `true` if the read returned OK (possibly after one retry); `false` on a hard
     *         failure or an invalid @p motorId, with @p out untouched.
     * @note Use for fresh baseline / arm / calibration reads that must be accurate.
     */
    bool readDegrees(uint8_t motorId, int32_t& out);

    /**
     * @brief Read a motor's encoder degrees, degrading to the last-good value on failure.
     *
     * On a successful read returns (and caches) the fresh value. On a hard failure returns
     * the centralized last-good for this motor if one exists, else @p fallback.
     *
     * @param motorId  M-bus channel `1..4`.
     * @param fallback Value returned only when the read fails AND no last-good is cached
     *                 (e.g. the very first read fails, or the cache was just reset).
     * @return Fresh degrees on success; else the last-good; else @p fallback.
     * @note Use for continuous completion checks where a one-tick-stale value is safe.
     */
    int32_t readDegreesOr(uint8_t motorId, int32_t fallback);

    /**
     * @brief Invalidate a motor's last-good cache after its encoder is re-zeroed.
     *
     * Call wherever a `resetCounter()` re-zeros the motor: otherwise a post-reset read that
     * fails could serve the pre-reset value through `readDegreesOr`.
     *
     * @param motorId M-bus channel `1..4`. An out-of-range id is ignored.
     */
    void resetCache(uint8_t motorId);

    /**
     * @brief Report whether the previous lower-MCU drive task has completed.
     *
     * Reads `mmL.Get_Drive_isTaskDone(DRIVE_INSTANCE_ID, ...)` and gates on the RESULT:
     * any non-`Drive_RESULT::OK` read is treated as "not done" (a transient miss must never
     * fabricate a `true` that would hand off a move that never finished). On an OK read the
     * vendor done-bit convention is honored (`isEnd[0] == true` means "still running").
     *
     * @return `true` only on an OK read whose done-bit reports the task finished.
     */
    bool isPrevTaskDone();

private:
    /// Number of addressable M-bus encoder channels (`M1..M4`).
    static constexpr uint8_t MOTOR_COUNT = 4;

    /// Per-motor last-good cache slot.
    struct CacheSlot
    {
        int32_t lastGood = 0;        ///< Last successfully-read degrees.
        bool    valid    = false;    ///< `true` once a successful read has populated `lastGood`.
    };

    /// Last-good cache, indexed by motor id `1..4` (`[0]` unused).
    CacheSlot _cache[MOTOR_COUNT + 1];

    /// @return `true` if @p motorId is an addressable channel `1..4`.
    static bool _validId(uint8_t motorId) { return motorId >= 1 && motorId <= MOTOR_COUNT; }
};

/// Single global instance — the sole owner of lower-MCU encoder / task-done I/O (defined in the .cpp).
extern EncoderSourceClass EncoderSource;

#endif    // ENCODER_SOURCE_H
