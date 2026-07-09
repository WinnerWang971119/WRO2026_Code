#ifndef TSHAPE_PROFILE_POLICY_H
#define TSHAPE_PROFILE_POLICY_H

#include <stdint.h>

/**
 * @file TShapeProfilePolicy.h
 * @brief Pure distance-domain T-shape (trapezoidal velocity profile) math — per-side ramp
 *        resolution, triangle fallback with reduced peak, half-of-total cap, and the
 *        speed-at-traveled interpolation (issue #130).
 *
 * Extracted VERBATIM from `DriveClass::_execProfileMove` (distance variant: ramp resolution +
 * triangle fallback + half-cap + peak PWM) and `DriveClass::_profileSpeedAt` (distance branch).
 * Drive.cpp delegates to these functions; the time-domain branch, the cm → encoder-degree
 * conversions (`cmToEncDeg`), and the profile state-machine wiring stay in Drive.cpp.
 *
 * Drive.cpp is NOT host-compiled (heavy MatrixMiniR4 deps), so this math previously had no
 * automated coverage. Promoting it to header-only `inline` free functions lets production
 * (Drive.cpp), the host golden-vector dump TU, and the pytest parity mirror
 * (`tools/path_planner/ctps/profile.py`, pinned via the golden vectors) exercise the SAME
 * arithmetic — the established `TurnSignPolicy.h` / `PivotTrimPolicy.h` pattern, which removes
 * the drift risk of hand-copied replicas.
 *
 * This header is deliberately self-contained (only `<stdint.h>`; zero Arduino / MatrixMiniR4 /
 * Config.h surface) so a host TU can include it standalone. The `Config::TSHAPE_*` knobs are
 * passed in as parameters by the caller.
 *
 * BEHAVIOR-IDENTITY CONTRACT — do NOT "improve" the arithmetic. The golden vectors and the
 * Python float32 mirror are pinned to the exact operations below:
 *   - All real arithmetic is 32-bit `float` (ramp divide, triangle rescale, interpolation).
 *   - The triangle peak is a truncating int32 divide, `(start + end + cruise) / 3`, cast to
 *     `int16_t`, then clamped toward cruise by two sign-aware compares.
 *   - The interpolation applies an INNER truncating `int16_t` cast to the scaled PWM delta
 *     BEFORE adding the endpoint PWM (truncation toward zero, so a negative delta rounds up).
 *   - Clamp order: no-ramp epsilon → [rampMin, rampMax] → triangle rescale → half-of-total cap.
 *
 * @note `deadbandSnap` is intentionally NOT applied here — the returned setpoints are pre-snap;
 *       Drive.cpp's `_emitContinuous` snaps each dispatched PWM downstream.
 * @note `accel` MUST be > 0 for both resolvers — `_execProfileMove` bails out (rc `0x01`)
 *       before this math is reached, so the policy performs no guard of its own; standalone
 *       host callers must enforce the same precondition.
 */

/**
 * @brief Resolved per-side ramp distances plus the (possibly triangle-reduced) peak PWM.
 *
 * Field order and types are the T2 contract (2026-07-09 CTPS firmware-motion-profile plan,
 * Contracts section). The distances are in whatever unit the resolver was fed — cm from
 * `resolveTShapeRamps`; `tShapeSpeedAt` is unit-agnostic and only needs all its distance
 * arguments to share one unit.
 */
struct TShapeRamps
{
    float   accelCm;    ///< Ramp-up distance (`0.0f` = no-ramp path — skip the ramp entirely).
    float   decelCm;    ///< Ramp-down distance (`0.0f` = no-ramp path).
    int16_t peakPwm;    ///< Signed peak PWM: cruise on the trapezoid path, reduced on the
                        ///< triangle fallback.
};

/**
 * @brief Resolve ONE side's ramp distance (cm): no-ramp epsilon, then `|cruise - endpoint| /
 *        accel`, clamped into `[rampMin, rampMax]`.
 *
 * Verbatim from `_execProfileMove`'s `resolveRamp` lambda. Returns `0.0f` on the no-ramp path
 * (a chained move whose endpoint speed is already within `noRampEps` of cruise) — the caller
 * treats a zero ramp as "skip the ramp entirely", NOT as "clamp up to rampMin".
 *
 * Shared by BOTH `_execProfileMove` variants: the distance variant consumes the cm directly
 * (via `resolveTShapeRamps` below); the time variant converts the cm to ms in Drive.cpp
 * (`cmToMs`).
 *
 * @param cruise    Signed cruise PWM (sign = direction).
 * @param endpoint  Signed endpoint PWM for this side (startSpeed or endSpeed).
 * @param accel     Ramp slope in PWM per cm — `Config::TSHAPE_ACCEL_PWM_PER_CM`. Must be > 0
 *                  (see the file-level precondition note).
 * @param rampMin   Floor on the ramp distance (cm) — `Config::TSHAPE_RAMP_MIN_CM`.
 * @param rampMax   Ceiling on the ramp distance (cm) — `Config::TSHAPE_RAMP_MAX_CM`.
 * @param noRampEps No-ramp hysteresis (PWM) — `Config::TSHAPE_NO_RAMP_EPSILON_PWM`.
 * @return Ramp distance in cm; `0.0f` when `|cruise - endpoint| < noRampEps`.
 */
inline float resolveTShapeRampCm(int16_t cruise, int16_t endpoint, float accel, float rampMin,
                                 float rampMax, int16_t noRampEps)
{
    // The original `abs(cruise - endpoint)` operates on int (int16_t promotes before the
    // subtraction). The explicit int32_t ternary below computes the identical value without
    // depending on the Arduino `abs` macro — this header is Arduino-free.
    const int32_t delta    = (int32_t) cruise - (int32_t) endpoint;
    const int32_t deltaMag = (delta > 0) ? delta : -delta;
    if (deltaMag < noRampEps)
    {
        return 0.0f;
    }
    float raw = (float) deltaMag / accel;
    if (raw < rampMin)
    {
        raw = rampMin;
    }
    if (raw > rampMax)
    {
        raw = rampMax;
    }
    return raw;
}

/**
 * @brief Resolve BOTH ramp distances + the peak PWM for a distance-bounded T-shape move.
 *
 * Mirrors `_execProfileMove`'s distance variant verbatim: per-side ramp resolution
 * (`resolveTShapeRampCm`), the triangle fallback with the averaged reduced peak, and the
 * half-of-total cap. Also applies `_execProfileMove`'s defensive non-negative floor on
 * `totalCm` (WARNING-2 fix) so the standalone policy is well-defined over its whole input
 * domain; the floor is idempotent when called from Drive.cpp, which floors before calling.
 *
 * @param totalCm   Total travel (cm). Negative values floor to 0.
 * @param cruise    Signed cruise PWM (sign carries direction; preserved into `peakPwm`).
 * @param startSpd  Signed PWM at move start.
 * @param endSpd    Signed PWM at move end.
 * @param accel     Ramp slope in PWM per cm — `Config::TSHAPE_ACCEL_PWM_PER_CM`. Must be > 0
 *                  (see the file-level precondition note).
 * @param rampMin   Per-side ramp floor (cm) — `Config::TSHAPE_RAMP_MIN_CM`.
 * @param rampMax   Per-side ramp ceiling (cm) — `Config::TSHAPE_RAMP_MAX_CM`.
 * @param noRampEps No-ramp hysteresis (PWM) — `Config::TSHAPE_NO_RAMP_EPSILON_PWM`.
 * @return `{accelCm, decelCm, peakPwm}` — per-side ramp distances in cm (each ≤ `totalCm / 2`
 *         after the cap) and the signed peak PWM.
 */
inline TShapeRamps resolveTShapeRamps(float totalCm, int16_t cruise, int16_t startSpd,
                                      int16_t endSpd, float accel, float rampMin, float rampMax,
                                      int16_t noRampEps)
{
    // Defensive non-negative clamp on the move bound — mirrors `_execProfileMove`'s WARNING-2
    // floor (a negative totalCm would produce negative ramp distances downstream).
    if (totalCm < 0.0f)
    {
        totalCm = 0.0f;
    }

    float accelCm = resolveTShapeRampCm(cruise, startSpd, accel, rampMin, rampMax, noRampEps);
    float decelCm = resolveTShapeRampCm(cruise, endSpd, accel, rampMin, rampMax, noRampEps);

    TShapeRamps out;

    // Triangle fallback: if the two FULL-peak ramps don't fit in the move, collapse the cruise
    // plateau and reduce the peak so the speed-vs-distance integral still equals the cruise *
    // total requested. The simple averaged-peak heuristic keeps the math easy and bounded:
    // peak <= cruise always (clamp). The triangle test MUST run on the UN-capped ramp
    // distances — applying the half-of-total cap first floors each ramp at totalCm*0.5, so
    // their sum could never exceed `totalCm` and this branch was unreachable (finding #15).
    // After this branch the ramps are rescaled to sum to `totalCm`, so the half-cap below is a
    // no-op on this path; it only guards the non-triangle case where the ramps fit overall but
    // one side alone would still over-run half the move.
    if (accelCm + decelCm > totalCm)
    {
        int16_t peak = (int16_t) (((int32_t) startSpd + (int32_t) endSpd + (int32_t) cruise)
                                  / 3);
        // Peak must NEVER exceed the requested cruise.
        if (peak > cruise && cruise >= 0)
        {
            peak = cruise;
        }
        if (peak < cruise && cruise < 0)
        {
            peak = cruise;
        }
        out.peakPwm = peak;

        if (accelCm + decelCm > 0.0f)
        {
            const float scale = totalCm / (accelCm + decelCm);
            accelCm *= scale;
            decelCm *= scale;
        }
    }
    else
    {
        out.peakPwm = cruise;
    }

    // Half-of-total cap — must not exceed half the move distance on either side. Runs AFTER
    // the triangle test (finding #15). In the triangle branch the ramps were rescaled to sum
    // to totalCm, but an asymmetric pair can still leave one side above totalCm*0.5, so this
    // cap may still trim the larger ramp; that only reshapes the apex (a small plateau at the
    // reduced peak instead of a pure triangle) and is harmless because move completion is
    // bound-gated on the encoder total, not on the velocity integral. In the full-trapezoid
    // branch it bounds a lone over-long ramp so a cruise plateau still exists.
    if (accelCm > totalCm * 0.5f)
    {
        accelCm = totalCm * 0.5f;
    }
    if (decelCm > totalCm * 0.5f)
    {
        decelCm = totalCm * 0.5f;
    }

    out.accelCm = accelCm;
    out.decelCm = decelCm;
    return out;
}

/**
 * @brief PWM setpoint at a traveled distance along the resolved T-shape profile.
 *
 * Verbatim from `_profileSpeedAt`'s distance branch:
 *  - Below the accel threshold: linear interp from `startSpd` → `r.peakPwm`.
 *  - Above the decel threshold: linear interp from `r.peakPwm` → `endSpd`.
 *  - In between: hold at `r.peakPwm`.
 *
 * Unit-agnostic: `traveledCm`, `totalCm`, `r.accelCm`, and `r.decelCm` only have to share one
 * distance unit. Drive.cpp feeds encoder-degree magnitudes cast to float (all exact integers
 * ≤ UINT16_MAX, far below float's 2^24 exact-integer ceiling, so every compare / subtraction /
 * divide sees exactly the operands the previous inline int32 branch saw — bit-identical); the
 * host golden dump and the Python mirror feed cm.
 *
 * @param traveledCm Distance traveled so far (same unit as `totalCm` / the ramps).
 * @param totalCm    Total move distance.
 * @param r          Resolved ramps + peak from `resolveTShapeRamps`.
 * @param startSpd   Signed PWM at move start.
 * @param endSpd     Signed PWM at move end.
 * @return Signed PWM setpoint (pre-`deadbandSnap`).
 */
inline int16_t tShapeSpeedAt(float traveledCm, float totalCm, const TShapeRamps& r,
                             int16_t startSpd, int16_t endSpd)
{
    if (traveledCm < r.accelCm)
    {
        // Ramp up: linear interp from start → peak.
        const float ratio = r.accelCm == 0.0f ? 1.0f : traveledCm / r.accelCm;
        return (int16_t) (startSpd + (int16_t) ((r.peakPwm - startSpd) * ratio));
    }
    else if (totalCm > r.decelCm && traveledCm >= (totalCm - r.decelCm))
    {
        // Ramp down: linear interp from peak → end.
        const float into  = traveledCm - (totalCm - r.decelCm);
        const float ratio = r.decelCm == 0.0f ? 1.0f : into / r.decelCm;
        return (int16_t) (r.peakPwm + (int16_t) ((endSpd - r.peakPwm) * ratio));
    }
    return r.peakPwm;
}

#endif    // TSHAPE_PROFILE_POLICY_H
