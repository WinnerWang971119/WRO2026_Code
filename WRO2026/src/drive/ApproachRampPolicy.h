#ifndef APPROACH_RAMP_POLICY_H
#define APPROACH_RAMP_POLICY_H

#include <stdint.h>

#include "TShapeProfilePolicy.h"

/**
 * @file ApproachRampPolicy.h
 * @brief Pure distance-domain "anticipatory line-approach ramp" math — resolve a ramp-down
 *        schedule from a PREDICTED sensor-crossing distance, then look up the commanded
 *        magnitude for a traveled distance along that schedule (2026-07-17 anticipatory
 *        line-approach-ramp plan, T2).
 *
 * Both reactive-stop consumers — the `LineFollower` junction hop (`PathTracker`'s
 * `LINE_FOLLOW_INTERSECTION`) and `Drive.stopAtLine`'s SEEK/ARMED seek — predict how far the
 * leading sensor board must still travel before it reaches the target line, then use this
 * policy to ramp cruise DOWN to the arrival floor BEFORE that predicted point instead of
 * reacting only after the physical line is seen. Physical detection stays the authoritative
 * stop (it re-anchors `Localization`); this policy only shapes the commanded magnitude leading
 * up to it and bounds how far the robot may crawl at the floor before a hard fault fires.
 *
 * This header COMPOSES `TShapeProfilePolicy.h`'s `resolveTShapeRampCm` for the ramp LENGTH —
 * it does not re-derive the no-ramp-epsilon / clamp arithmetic. The only new math here is the
 * three phase boundaries (`rampStartCm`, `floorStartCm`, `overrunLimitCm`) and one linear
 * interpolation across the ramp phase.
 *
 * This header is deliberately self-contained (only `<stdint.h>` plus the sibling
 * `TShapeProfilePolicy.h`; zero Arduino / MatrixMiniR4 / Config.h surface) so a host TU can
 * include it standalone. The `Config::LINE_APPROACH_*` / `Config::TSHAPE_*` knobs are passed
 * in as parameters by the caller — same convention as `TShapeProfilePolicy.h`.
 *
 * BEHAVIOR-IDENTITY CONTRACT — do NOT "improve" the arithmetic. The golden vectors and the
 * Python float32 mirror (`tools/path_planner/ctps/profile.py`) are pinned to the exact
 * operations below:
 *   - All real arithmetic is 32-bit `float` (phase-boundary subtraction/addition, the ramp
 *     ratio divide, the interpolation scale).
 *   - The interpolation applies an INNER truncating `int16_t` cast to the scaled PWM delta
 *     BEFORE adding the endpoint PWM (truncation toward zero, so a negative delta rounds up) —
 *     the SAME shape as `tShapeSpeedAt`'s ramp-up branch in `TShapeProfilePolicy.h`.
 *   - Phase boundaries are compared with `<` (never `<=`) so each traveled distance maps to
 *     exactly one phase, in the order CRUISE, RAMP, FLOOR_HOLD, OVERRUN.
 *
 * @note `deadbandSnap` is intentionally NOT applied here — `approachSpeedAt`'s returned
 *       magnitude is pre-snap and unsigned; the caller applies direction sign and snaps the
 *       dispatched PWM downstream (mirrors `TShapeProfilePolicy.h`).
 * @note `accel` MUST be > 0 (passed straight through to `resolveTShapeRampCm`) — this header
 *       performs no guard of its own; standalone host callers must enforce the same
 *       precondition as `TShapeProfilePolicy.h`.
 * @note `resolveApproachSchedule` does NO sensor-offset math on `predictedCm` — the caller
 *       (PathTracker / Drive) pre-subtracts the direction-aware axle offset before calling in,
 *       so `predictedCm` is already the sensor-crossing distance (TC17).
 */

/**
 * @brief Named phase of the approach-ramp schedule at a given traveled distance.
 *
 * Order matches the physical sequence of a terminating line hop: full cruise, ramping down,
 * holding the arrival floor, then (only on a missed line) an overrun crawl at the same floor.
 */
enum class ApproachPhase : uint8_t
{
    CRUISE,        ///< Traveled distance is before the ramp — commanded magnitude is cruise.
    RAMP,          ///< Traveled distance is inside the ramp — linearly interpolating down.
    FLOOR_HOLD,    ///< Traveled distance is at/after the predicted crossing, within bounds.
    OVERRUN        ///< Traveled distance is past the bounded overrun limit — caller faults.
};

/**
 * @brief Resolved approach-ramp schedule: three phase-boundary distances plus the cruise/floor
 *        magnitudes, and whether the schedule is usable at all.
 *
 * Field order and types are the T2 contract (2026-07-17 anticipatory-line-approach-ramp plan,
 * Contracts section) — the Python mirror (`profile.py`) and the parity trace record depend on
 * this exact shape. All distances share one unit (cm from both current callers); `approachSpeedAt`
 * is otherwise unit-agnostic.
 */
struct ApproachSchedule
{
    float   rampStartCm;      ///< Traveled distance where the ramp begins (`floorStartCm -
                               ///< rampLenCm`). `<= 0.0f` is one of the two degrade-to-flat
                               ///< conditions (see `active`).
    float   floorStartCm;     ///< Traveled distance where the floor hold begins
                               ///< (`predictedCm - marginCm`).
    float   overrunLimitCm;   ///< Traveled distance where `OVERRUN` begins
                               ///< (`predictedCm + maxMarginCm`).
    int16_t cruiseMag;        ///< Cruise magnitude (unsigned, `>= 0`).
    int16_t floorMag;         ///< Arrival/hold magnitude (unsigned, `>= 0`; caller passes in
                               ///< `Config::PWM_MOVING_MIN`).
    bool    active;           ///< `false` => the schedule is degenerate (short hop or a
                               ///< disabled-feature caller passing a tiny `predictedCm`); the
                               ///< caller ignores every other field and runs its existing flat
                               ///< path. The policy never fabricates a partial ramp.
};

/**
 * @brief Commanded magnitude + phase for one traveled distance along a resolved schedule.
 */
struct ApproachSpeed
{
    int16_t       pwmMag;    ///< Commanded magnitude for this traveled distance (pre-snap,
                              ///< unsigned — caller applies direction sign and `deadbandSnap`).
    ApproachPhase phase;     ///< Which phase produced `pwmMag` — `CRUISE`, `RAMP`,
                              ///< `FLOOR_HOLD`, or `OVERRUN`.
};

/**
 * @brief Resolve the approach-ramp schedule from a predicted sensor-crossing distance.
 *
 * `floorStartCm = predictedCm - marginCm`; the ramp length composes `resolveTShapeRampCm`
 * (`TShapeProfilePolicy.h`) exactly as the T-shape profile resolves a single side's ramp, so
 * `rampStartCm = floorStartCm - rampLenCm`; `overrunLimitCm = predictedCm + maxMarginCm`.
 *
 * The schedule degrades to `active = false` when it is degenerate: either the ramp would have
 * to start at or before the beginning of the move (`rampStartCm <= 0.0f` — there is no room for
 * a full ramp before the floor hold), or the predicted distance itself is below the minimum
 * meaningful move (`predictedCm < minMoveCm`, mirroring `TShapeProfilePolicy.h`'s own
 * `TSHAPE_MIN_MOVE_CM` floor). The caller (LineFollower / Drive.stopAtLine) then runs its
 * existing flat-cruise-until-detected path unchanged — this is the documented short-hop known
 * limit (AC7), not an error.
 *
 * @param predictedCm  Predicted distance (cm) the leading sensor still has to travel before it
 *                      crosses the target line. Already sensor-frame — the caller has
 *                      pre-subtracted the direction-aware axle offset (TC17: this function does
 *                      no offset math of its own).
 * @param marginCm     Distance (cm) BEFORE `predictedCm` where the floor hold begins —
 *                      `Config::LINE_APPROACH_MARGIN_CM`.
 * @param maxMarginCm  Distance (cm) PAST `predictedCm` before `OVERRUN` begins —
 *                      `Config::LINE_APPROACH_MAX_MARGIN_CM`.
 * @param cruiseMag    Cruise magnitude (unsigned, `>= 0`) — carried into the schedule unchanged.
 * @param floorMag     Arrival/hold magnitude (unsigned, `>= 0`) — `Config::PWM_MOVING_MIN`.
 * @param accel        Ramp slope in PWM per cm — `Config::TSHAPE_ACCEL_PWM_PER_CM`. Must be > 0
 *                      (see the file-level precondition note); forwarded to
 *                      `resolveTShapeRampCm` unchanged.
 * @param rampMin      Per-side ramp floor (cm) — `Config::TSHAPE_RAMP_MIN_CM`.
 * @param rampMax      Per-side ramp ceiling (cm) — `Config::TSHAPE_RAMP_MAX_CM`.
 * @param noRampEps    No-ramp hysteresis (PWM) — `Config::TSHAPE_NO_RAMP_EPSILON_PWM`.
 * @param minMoveCm    Minimum meaningful predicted distance (cm) below which the schedule
 *                      degrades to flat — `Config::TSHAPE_MIN_MOVE_CM`. Passed in rather than
 *                      hard-coded because this header takes no `Config.h` dependency.
 * @return The resolved `ApproachSchedule`; check `active` before using the other fields.
 */
inline ApproachSchedule resolveApproachSchedule(float predictedCm, float marginCm,
                                                float maxMarginCm, int16_t cruiseMag,
                                                int16_t floorMag, float accel, float rampMin,
                                                float rampMax, int16_t noRampEps,
                                                float minMoveCm)
{
    ApproachSchedule out;

    out.floorStartCm = predictedCm - marginCm;

    const float rampLenCm = resolveTShapeRampCm(cruiseMag, floorMag, accel, rampMin, rampMax,
                                                noRampEps);
    out.rampStartCm = out.floorStartCm - rampLenCm;

    out.overrunLimitCm = predictedCm + maxMarginCm;
    out.cruiseMag       = cruiseMag;
    out.floorMag        = floorMag;
    out.active          = !(out.rampStartCm <= 0.0f || predictedCm < minMoveCm);

    return out;
}

/**
 * @brief Commanded magnitude + phase at a traveled distance along a resolved approach schedule.
 *
 * Verbatim four-way phase split:
 *  - `traveledCm < s.rampStartCm` -> `{cruiseMag, CRUISE}`.
 *  - `s.rampStartCm <= traveledCm < s.floorStartCm` -> linear interp `cruiseMag` -> `floorMag`
 *    across `[rampStartCm, floorStartCm]`, phase `RAMP`.
 *  - `s.floorStartCm <= traveledCm < s.overrunLimitCm` -> `{floorMag, FLOOR_HOLD}`.
 *  - `traveledCm >= s.overrunLimitCm` -> `{floorMag, OVERRUN}`.
 *
 * @note `OVERRUN` is PINNED to return `floorMag`, the same magnitude as `FLOOR_HOLD` — past the
 *       overrun limit the policy keeps commanding the floor; STOPPING the chassis is the
 *       caller's fault action (`LINE_NOT_FOUND` / `DRIVE_FAULT`), never this policy's.
 * @note Caller must not invoke this on a schedule with `active == false` — the boundaries are
 *       meaningless on a degenerate schedule; the caller runs its flat path instead.
 *
 * @param s          A schedule from `resolveApproachSchedule` (assumed `active == true`).
 * @param traveledCm Distance traveled so far (same unit as the schedule's boundaries).
 * @return `{pwmMag, phase}` — unsigned magnitude (pre-`deadbandSnap`) and the phase that
 *         produced it.
 */
inline ApproachSpeed approachSpeedAt(const ApproachSchedule& s, float traveledCm)
{
    if (traveledCm < s.rampStartCm)
    {
        return ApproachSpeed{ s.cruiseMag, ApproachPhase::CRUISE };
    }

    if (traveledCm < s.floorStartCm)
    {
        // Ramp down: linear interp from cruise -> floor across [rampStartCm, floorStartCm).
        const float rampLenCm = s.floorStartCm - s.rampStartCm;
        const float into      = traveledCm - s.rampStartCm;
        const float ratio     = rampLenCm == 0.0f ? 1.0f : into / rampLenCm;
        const int16_t pwmMag  = (int16_t) (s.cruiseMag
                                           + (int16_t) ((s.floorMag - s.cruiseMag) * ratio));
        return ApproachSpeed{ pwmMag, ApproachPhase::RAMP };
    }

    if (traveledCm < s.overrunLimitCm)
    {
        return ApproachSpeed{ s.floorMag, ApproachPhase::FLOOR_HOLD };
    }

    return ApproachSpeed{ s.floorMag, ApproachPhase::OVERRUN };
}

#endif    // APPROACH_RAMP_POLICY_H
