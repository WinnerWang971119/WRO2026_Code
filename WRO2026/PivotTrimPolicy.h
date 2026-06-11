#ifndef PIVOT_TRIM_POLICY_H
#define PIVOT_TRIM_POLICY_H

/**
 * @file PivotTrimPolicy.h
 * @brief Single source of truth for the per-turn pivot trim on a DISTANCE-
 *        terminated hop (ADR-036, issue #44).
 *
 * `resolvePivotTrim(...)` answers one question per dead-reckon hop: given the raw
 * edge length and the two RESOLVED pivot trims latched at plan time, what
 * effective distance does the chassis drive? It subtracts the departing
 * (outgoing) over-travel of the pivot just left and the approaching trim of the
 * pivot being entered, then clamps to a tiny positive floor so a bad table can
 * never command a non-positive drive:
 *
 *     effective = rawDistanceCm - departingTrimCm - approachingTrimCm
 *     drive     = max(effective, minDriveCm)
 *
 * On a chain `u->v->w->x` the hop `v->w` nets
 * `length - delta_departing - delta_approaching` (exactly what the validator's
 * chained sum-check guards). `clamped` reports whether the floor fired so the
 * caller can emit its own underflow trace -- this header stays I/O-free.
 *
 * Previously this lived inlined in PathTracker.cpp's dispatch, beyond the reach
 * of the host-compile suite (which does not link PathTracker.cpp). Promoting it
 * to a header-only `inline` free function lets production and a host test link
 * the SAME symbol -- the established `DirectionPolicy.h` / `StopAtLinePolicy.h`
 * pattern (PR #34), which killed the per-test replicas those decisions used to
 * need.
 *
 * Cross-language parity: keep step-for-step identical with
 * `tools/path_planner/sim/runtime.py::PathTrackerSim._pivot_trim_distance`.
 */

/**
 * @brief Outcome of the pivot-trim decision for one DISTANCE-terminated hop.
 *
 * @note `effectiveCm` is the pre-clamp value (`raw - trims`); the caller reads it
 *       only for the underflow trace. `driveCm` is the value to actually drive
 *       (`>= minDriveCm`).
 */
struct PivotTrimResolution
{
    float driveCm;        ///< distance to drive: max(effective, floor).
    float effectiveCm;    ///< pre-clamp distance (raw - trims); for the trace.
    bool  clamped;        ///< true if the floor clamp fired (caller traces it).
};

/**
 * @brief Resolve the effective dead-reckon distance after the two pivot trims.
 *
 * @param rawDistanceCm     The directed edge length (`edge.distance_cm`).
 * @param departingTrimCm   Outgoing trim of the pivot being LEFT, or 0 if none.
 * @param approachingTrimCm Approach trim of the pivot being ENTERED, or 0 if none.
 * @param minDriveCm        `Config::PIVOT_TRIM_MIN_DRIVE_CM` underflow floor.
 * @return `{driveCm, effectiveCm, clamped}` per the formula above.
 */
inline PivotTrimResolution resolvePivotTrim(
    float rawDistanceCm,
    float departingTrimCm,
    float approachingTrimCm,
    float minDriveCm)
{
    PivotTrimResolution result;
    result.effectiveCm = rawDistanceCm - departingTrimCm - approachingTrimCm;
    if (result.effectiveCm < minDriveCm)
    {
        result.driveCm = minDriveCm;
        result.clamped = true;
    }
    else
    {
        result.driveCm = result.effectiveCm;
        result.clamped = false;
    }
    return result;
}

#endif    // PIVOT_TRIM_POLICY_H
