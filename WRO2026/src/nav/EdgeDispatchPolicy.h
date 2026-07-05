#ifndef EDGE_DISPATCH_POLICY_H
#define EDGE_DISPATCH_POLICY_H

#include <stdint.h>

#include "NavTypes.h"
#include "PivotTrimPolicy.h"
#include "StopAtLinePolicy.h"
#include "routes.h"

/**
 * @file EdgeDispatchPolicy.h
 * @brief Single source of truth for the TERMINAL one-hop dispatch classification
 *        on a routed edge (issue #108).
 *
 * `resolveEdgeDispatch(...)` answers one question per hop: given the already-
 * resolved inputs for an edge — its type, the destination node type, the resolved
 * travel direction, the resolved line/blind modes, the stop-at-line decision, the
 * pivot trims, and the projection pass-through lookahead — which terminal
 * primitive does the FSM dispatch, and with what commanded parameters? The output
 * is a classification, NOT a full descriptor:
 *
 *   - `LINE_FOLLOW_INTERSECTION` — LINE edge into a LINE (junction) node: follow
 *     to the next physical intersection, then the LINE_OVERSHOOT roll + re-anchor.
 *     Sensor-terminated, so `commandedDistanceCm` is 0 here — the overshoot advance
 *     is computed in the `LINE_FOLLOW` state, not in this classifier.
 *   - `PROJECTION_FOLLOW` — LINE edge into a PROJECTION node: distance-gated follow
 *     using the pivot-trimmed edge length; advances the node by dead-reckoning.
 *   - `STOP_AT_LINE_SEEK` — BLIND edge that seeks a line (`stopAtLine.seek`): the
 *     seek drives the FULL `rawDistanceCm` (the crossing, not the distance,
 *     terminates it) and rolls a (pivot-trimmed) overshoot to land the axle on the
 *     node; re-anchors on the intersection.
 *   - `BLIND_DEAD_RECKON` — BLIND edge with no seek: dead-reckons the pivot-trimmed
 *     distance, then advances the node by dead-reckoning.
 *
 * The resolver COMPOSES the existing pure policies — it calls `resolvePivotTrim`
 * (PivotTrimPolicy.h) for the dead-reckon / projection commanded distance and
 * `resolveSensorOvershootTrim` for the seek overshoot, and consumes the
 * `StopAtLineResolution` from `resolveStopAtLine` (StopAtLinePolicy.h) — but
 * introduces NO new cross-hop state. Direction resolution, the projection
 * pass-through lookahead, the pivot 3-latch carry, the GYRO heading target, the
 * velocity-profile start-speed split, and the `PIVOT_TRIM_UNDERFLOW` fault all stay
 * FSM-side (PathTracker.cpp). The fault is intentionally NOT a decision field: the
 * classifier clamps (via the composed policies) and the FSM re-derives the clamp at
 * the dispatch site to raise the fault, consistent with the fault-FSM structural
 * exclusion from the parity schema.
 *
 * Payload rule (memory-critical, `gotcha-sim-primitive-payload-is-command-not-
 * displacement`): `commandedDistanceCm` is the COMMANDED value — full
 * `rawDistanceCm` for a sensor-terminated seek, the pivot-trimmed distance for a
 * dead-reckon / projection follow — NEVER the physical displacement. The physical
 * trim lives only in the pose integration.
 *
 * Trim-clamp arithmetic (frozen contract — both languages implement this
 * identically): for dead-reckon / projection hops `commandedDistanceCm =
 * max(rawDistanceCm - departingTrimCm - approachingTrimCm, minDriveCm)` (the
 * post-clamp value the CSV truth-table compares row-for-row).
 *
 * Cross-language parity: keep this step-for-step identical with
 * `tools/path_planner/sim/runtime.py::resolve_edge_dispatch` — same input fields,
 * same enum values, same `{primitive, reverse, driveMode, commandedDistanceCm,
 * overshootCm, reanchor, brakeOnGate}` output. The fixture
 * `tools/path_planner/tests/fixtures/edge_dispatch_truth_table.csv` is the
 * normative source both transcribe; both surfaces are verified against it row-for-
 * row (Python TC1 in `test_runtime_sim.py` and the C++ harness
 * `test_edge_dispatch_truth_table.cpp`), mirroring the `resolveStopAtLine` /
 * #115 pattern. This field set is the frozen contract that lets the C++ and Python
 * resolvers be authored in parallel — any field added here is a contract change
 * that updates the spec and re-syncs the Python mirror, never a silent addition.
 */

/**
 * @brief Already-resolved inputs to the terminal one-hop dispatch classifier.
 *
 * @note The classifier computes NO cross-hop state — every field is resolved by
 *       the FSM before the query is built. Field order mirrors the Python mirror's
 *       `EdgeDispatchQuery` so the two surfaces read identically.
 */
struct EdgeDispatchQuery
{
    Routes::EdgeType     edgeType;          ///< LINE | BLIND (NONE is never routed here).
    Routes::NodeType     destNodeType;      ///< Dest node type: LINE | PROJECTION.
    Direction            direction;         ///< FRONT | REAR (already resolved; never AUTO).
    float                rawDistanceCm;      ///< edge.distance_cm.
    Routes::LineMode     lineMode;          ///< Resolved line mode for the edge (never DEFAULT).
    StopAtLineResolution stopAtLine;        ///< resolveStopAtLine(...) result; only read for BLIND.
    bool                 projPassThrough;    ///< FSM lookahead; only meaningful LINE->projection.
    float                departingTrimCm;    ///< _departingPivot.trim_cm or 0.
    float                approachingTrimCm;  ///< _approachingPivot.trim_cm or 0.
    float                axleOffsetCm;       ///< Direction-aware LINE_SENSOR_AXLE_OFFSET[_REAR]_CM.
    float                minDriveCm;        ///< Config::PIVOT_TRIM_MIN_DRIVE_CM.
};

/**
 * @brief Terminal one-hop dispatch primitive selected by `resolveEdgeDispatch`.
 */
enum class DispatchPrimitive : uint8_t
{
    LINE_FOLLOW_INTERSECTION,  ///< LINE -> line: follow to junction, then overshoot.
    PROJECTION_FOLLOW,         ///< LINE -> projection: distance-gated follow.
    STOP_AT_LINE_SEEK,         ///< BLIND + seek.
    BLIND_DEAD_RECKON          ///< BLIND, no seek.
};

/**
 * @brief How the FSM re-anchors `Localization` at the end of the hop.
 */
enum class ReanchorKind : uint8_t
{
    ON_INTERSECTION,          ///< Verified re-anchor on a physical line crossing.
    ON_DEAD_RECKONED_ADVANCE  ///< Dead-reckoned advance (no junction to re-anchor against).
};

/**
 * @brief Outcome of the terminal one-hop dispatch classification.
 *
 * @note Exactly these seven output fields — the frozen contract (see the file
 *       docstring). `commandedDistanceCm` is the post-clamp COMMANDED value, never
 *       the physical displacement. `PIVOT_TRIM_UNDERFLOW` is NOT a field: the FSM
 *       re-derives the clamp at the dispatch site to raise the fault.
 */
struct EdgeDispatchDecision
{
    DispatchPrimitive primitive;            ///< Which terminal primitive to dispatch.
    bool              reverse;              ///< direction == REAR.
    Routes::BlindMode driveMode;            ///< Blind primitives: stopAtLine.motionMode; line: DEFAULT (unused).
    float             commandedDistanceCm;  ///< Post-clamp commanded value (NOT displacement).
    float             overshootCm;          ///< Seek overshoot channel (0 for non-seek).
    ReanchorKind      reanchor;            ///< Intersection re-anchor vs dead-reckoned advance.
    bool              brakeOnGate;          ///< false ONLY on projection pass-through.
};

/**
 * @brief Classify the terminal one-hop dispatch for a routed edge.
 *
 * @param q Already-resolved inputs for this hop (@ref EdgeDispatchQuery).
 * @return The `{primitive, reverse, driveMode, commandedDistanceCm, overshootCm,
 *         reanchor, brakeOnGate}` classification per the frozen contract above.
 *
 * @note `q.stopAtLine` is read only for BLIND edges; `q.projPassThrough` only for a
 *       LINE -> projection hop. `q.lineMode` is carried by the FSM, not the
 *       decision (line primitives take their motion profile from the edge directly).
 */
inline EdgeDispatchDecision resolveEdgeDispatch(const EdgeDispatchQuery& q)
{
    EdgeDispatchDecision d{};
    d.reverse = (q.direction == Direction::REAR);

    if (q.edgeType == Routes::EdgeType::LINE)
    {
        if (q.destNodeType == Routes::NodeType::PROJECTION)
        {
            // LINE -> projection: distance-gated follow over the pivot-trimmed length.
            d.primitive = DispatchPrimitive::PROJECTION_FOLLOW;
            d.commandedDistanceCm =
                resolvePivotTrim(q.rawDistanceCm, q.departingTrimCm, q.approachingTrimCm,
                                 q.minDriveCm)
                    .driveCm;
            d.overshootCm = 0.0f;
            d.reanchor    = ReanchorKind::ON_DEAD_RECKONED_ADVANCE;
            d.brakeOnGate = !q.projPassThrough;
            d.driveMode   = Routes::BlindMode::DEFAULT;    // unused for line primitives
        }
        else
        {
            // LINE -> line (junction): sensor-terminated; the LINE_OVERSHOOT advance is
            // computed in the LINE_FOLLOW state, NOT here, so commandedDistanceCm is 0.
            d.primitive           = DispatchPrimitive::LINE_FOLLOW_INTERSECTION;
            d.commandedDistanceCm = 0.0f;
            d.overshootCm         = 0.0f;
            d.reanchor            = ReanchorKind::ON_INTERSECTION;
            d.brakeOnGate         = true;    // unused for the intersection-gated follow
            d.driveMode           = Routes::BlindMode::DEFAULT;    // unused for line primitives
        }
    }
    else    // BLIND
    {
        if (q.stopAtLine.seek)
        {
            // BLIND + seek: the seek drives the FULL raw distance (sensor-terminated);
            // the overshoot rolls the axle onto the node, pivot-trimmed.
            d.primitive           = DispatchPrimitive::STOP_AT_LINE_SEEK;
            d.commandedDistanceCm = q.rawDistanceCm;
            d.overshootCm =
                resolveSensorOvershootTrim(q.axleOffsetCm, q.approachingTrimCm, q.minDriveCm)
                    .overshootCm;
            d.reanchor    = ReanchorKind::ON_INTERSECTION;
            d.brakeOnGate = true;
            d.driveMode   = q.stopAtLine.motionMode;
        }
        else
        {
            // BLIND, no seek: dead-reckon the pivot-trimmed distance.
            d.primitive = DispatchPrimitive::BLIND_DEAD_RECKON;
            d.commandedDistanceCm =
                resolvePivotTrim(q.rawDistanceCm, q.departingTrimCm, q.approachingTrimCm,
                                 q.minDriveCm)
                    .driveCm;
            d.overshootCm = 0.0f;
            d.reanchor    = ReanchorKind::ON_DEAD_RECKONED_ADVANCE;
            d.brakeOnGate = true;
            d.driveMode   = q.stopAtLine.motionMode;
        }
    }

    return d;
}

#endif    // EDGE_DISPATCH_POLICY_H
