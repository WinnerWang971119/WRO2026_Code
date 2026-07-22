#ifndef PATH_TRACKER_PARITY_TRACE_H
#define PATH_TRACKER_PARITY_TRACE_H

/**
 * @file PathTrackerParityTrace.h
 * @brief Compile-time-gated NDJSON trace emit facility for the sim<->firmware FSM
 *        parity harness (issue #108).
 *
 * The whole facility is wrapped in `#ifdef PT_PARITY_TRACE`. The firmware build
 * (`arduino-cli compile`) and the default host ctest suite do NOT define it, so
 * this header is empty there and every trace call site in PathTracker.cpp compiles
 * to nothing — the binary is byte-for-byte behaviorally identical. Only the
 * dedicated parity-dump target defines `PT_PARITY_TRACE` (with
 * `PARITY_SCHEMA_VERSION=4`).
 *
 * When enabled, `pt_parity::emit(line)` writes one NDJSON record line to stdout.
 * PathTracker.cpp builds the per-hop records and calls it at each FSM commit point;
 * the parity driver (T6) includes this header to emit its own `goal_start` /
 * `goal_done` framing records. Floats are formatted as fixed 6 decimals (`%.6f`)
 * by the record builders so the text is stable for the cross-language comparator.
 *
 * ===========================================================================================
 * ADR-080 wire schema — the blind-run records (firmware side; `sim/parity.py` mirrors these
 * field sets byte-for-byte in `TRACE_RECORD_FIELDS`, which is STRICT: an unknown OR missing
 * field of a known kind raises `ParitySchemaError`). ADR-080 (blind-run coalescing) bumps the
 * schema to v4: `blind_dead_reckon` gains required `node` and `cruise_pwm` fields, and a
 * collinear BLIND run of N >= 2 hops is now ONE record (dist_cm = the summed run distance,
 * `chain` = true) followed by N-1 `blind_pass` records (one per intermediate node), emitted at
 * arrival in run order. A single BLIND hop is ONE `blind_dead_reckon` with `chain` = false.
 *
 * 1. `blind_dead_reckon` — ONE per BLIND run. Exact serialization (`_ptEmitBlindDeadReckon`,
 *    PathTracker.cpp):
 *
 *      {"kind":"blind_dead_reckon","reverse":<bool>,"drive_mode":"<SYNC_TSHAPE|GYRO_TSHAPE|
 *       RAW_FLAT>","node":"<final node>","dist_cm":<%.6f>,"cruise_pwm":<%d>,"start_pwm":<%d>,
 *       "end_pwm":<%d>,"chain":<bool>,
 *       "reanchor":"ON_DEAD_RECKONED_ADVANCE"}
 *
 *    - `node`      : the run's final node (or the ordinary destination for a single hop).
 *    - `dist_cm`   : the DISPATCHED distance — `totalCm` (Σ hop distances) for a coalesced
 *                    run, else the single hop's pivot-trimmed distance. No boundary-carry
 *                    term (ADR-080 dropped it; one braked move has no inter-hop carry).
 *    - `cruise_pwm`: signed resolved cruise passed to `Drive.setPower(...)`. This is explicit
 *                    because it is not recoverable from a cold run's start/end endpoints.
 *    - `start_pwm` : signed PWM the combined move's tShape starts at (`%d`, int16):
 *                    ±TSHAPE_START_SPEED, or the ADR-023 captured LineFollower cruise after a
 *                    pass-through. RAW_FLAT: its flat start (cruise, or the captured cruise).
 *    - `end_pwm`   : signed PWM the plan ends at (`%d`, int16). Braked (every coalesced run
 *                    and every non-RAW single hop): `0`. RAW_FLAT: cruise.
 *    - `chain`     : `true`/`false` — whether this record is a COALESCED run (>= 2 hops,
 *                    dist_cm == totalCm) versus a single hop. NOTE: the run flag key is
 *                    `chain`, deliberately NOT any sub-field named "kind" — `kind` is the
 *                    record-type discriminator the parity loader keys on.
 *
 * 2. `blind_pass` — one per INTERMEDIATE node of a coalesced run (node[1..hopCount-1]).
 *    Emitted at arrival in `BLIND_FORWARD`, immediately BEFORE that node's
 *    `onDeadReckonedAdvance` stamp, so a run reads as one `blind_dead_reckon` followed by its
 *    per-node passes in run order. The run's FINAL node is the braked arrival and emits NO
 *    pass record. Exact serialization (`_ptEmitBlindPass`, PathTracker.cpp):
 *
 *      {"kind":"blind_pass","node":"<node name>"}
 *
 * ===========================================================================================
 * issue #156 / parity v3 wire schema — the two LINE-follow records gain a lateral `follow_mode`
 * field and RENAME the historically-misnamed `line_mode` (which has always held the resolved
 * SPEED tier, never a follow mode) to `speed`. Both changes land in ONE field-set bump.
 * `sim/parity.py` mirrors these field sets byte-for-byte in `TRACE_RECORD_FIELDS` (STRICT: an
 * unknown OR missing field of a known kind raises `ParitySchemaError`), so both sides must land
 * the bump together — the coordinated `PARITY_SCHEMA_VERSION` stamp is owned by the host-test /
 * dump surfaces, not this header.
 *
 * 3. `line_follow` — field-set BUMP (`line_mode`→`speed` rename + new `follow_mode`). Exact
 *    serialization (`_ptEmitLineFollow`, PathTracker.cpp):
 *
 *      {"kind":"line_follow","reverse":<bool>,"speed":"<SLOW|NORMAL|FAST>",
 *       "follow_mode":"<MIDDLE|LEFT|RIGHT>"}
 *
 *    - `speed`       : the resolved `Routes::SpeedTier` member name (`_ptLineModeName`) — same
 *                      value as the old `line_mode`, corrected key.
 *    - `follow_mode` : the RESOLVED `Routes::LineFollowMode` member name (`_ptFollowModeName`),
 *                      one of MIDDLE/LEFT/RIGHT — never "DEFAULT" (`_resolveLineFollowMode`
 *                      resolves DEFAULT→MIDDLE upstream). It is the MODE, not the float offset
 *                      handed to `LineFollower.setLineFollowOffset(...)` (which is the mapped
 *                      setpoint, dispatched direction-independent — ADR-069 amendment).
 *
 * 4. `projection_follow` — field-set BUMP (same `line_mode`→`speed` rename + new `follow_mode`).
 *    Exact serialization (`_ptEmitProjectionFollow`, PathTracker.cpp):
 *
 *      {"kind":"projection_follow","reverse":<bool>,"speed":"<SLOW|NORMAL|FAST>",
 *       "follow_mode":"<MIDDLE|LEFT|RIGHT>","dist_cm":<%.6f>,"pass_through":<bool>,
 *       "reanchor":"ON_DEAD_RECKONED_ADVANCE"}
 *
 *    `speed`/`follow_mode` carry the same meaning as on `line_follow`; the remaining fields
 *    (`dist_cm`, `pass_through`, `reanchor`) are unchanged from v2.
 */

#ifdef PT_PARITY_TRACE

#include <cstdio>

namespace pt_parity
{
    /**
     * @brief Emit one NDJSON trace line to stdout, followed by a newline.
     *
     * @param ndjsonLine A complete, NUL-terminated JSON object (no trailing
     *                    newline — this function appends it).
     */
    inline void emit(const char* ndjsonLine)
    {
        std::fputs(ndjsonLine, stdout);
        std::fputc('\n', stdout);
    }
}    // namespace pt_parity

#endif    // PT_PARITY_TRACE

#endif    // PATH_TRACKER_PARITY_TRACE_H
