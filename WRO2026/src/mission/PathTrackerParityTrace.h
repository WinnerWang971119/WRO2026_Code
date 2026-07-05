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
 * `PARITY_SCHEMA_VERSION=1`).
 *
 * When enabled, `pt_parity::emit(line)` writes one NDJSON record line to stdout.
 * PathTracker.cpp builds the per-hop records and calls it at each FSM commit point;
 * the parity driver (T6) includes this header to emit its own `goal_start` /
 * `goal_done` framing records. Floats are formatted as fixed 6 decimals (`%.6f`)
 * by the record builders so the text is stable for the cross-language comparator.
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
