#ifndef STOP_AT_LINE_POLICY_H
#define STOP_AT_LINE_POLICY_H

#include <stdint.h>

#include "Config.h"
#include "NavTypes.h"
#include "routes.h"

/**
 * @file StopAtLinePolicy.h
 * @brief Single source of truth for the stop-at-line seek/degrade decision on a
 *        BLIND hop.
 *
 * `resolveStopAtLine(...)` answers one question per hop: does this BLIND edge run
 * a `Drive.stopAtLine` seek, and if not, which motion bundle does the
 * dead-reckoned fallback use? The rule:
 *
 *   1. seekRequested: a call-site `StopAtLine::ON` always asks for the seek; a
 *      call-site `StopAtLine::AUTO` asks only when the edge itself is
 *      `Routes::BlindMode::STOP_AT_LINE`; `StopAtLine::OFF` never asks.
 *   2. boardOK: a line board must face the travel direction — the front board
 *      always faces forward, but a rear seek needs a second board
 *      (`lineBoardCount >= 2`).
 *   3. seek = seekRequested AND boardOK.
 *   4. When seeking, the motion bundle is the edge's mode, except a bare
 *      `STOP_AT_LINE` edge has no real velocity profile of its own and seeks
 *      under `SYNC_TSHAPE`.
 *   5. When not seeking (OFF, or a board-degrade), the hop dead-reckons: a
 *      `STOP_AT_LINE` edge falls back to the pipeline blind mode; any residual
 *      `STOP_AT_LINE` (e.g. a `STOP_AT_LINE` pipeline default) defensively
 *      degrades to `SYNC_TSHAPE`. The returned `motionMode` is therefore never
 *      `STOP_AT_LINE` and never `DEFAULT`.
 *
 * Previously this precedence lived inlined in PathTracker.cpp's dispatch, which
 * forced the host-compile test suite to maintain a byte-for-byte
 * `replica_resolveStopAtLine`. Promoting it to a header-only `inline` free
 * function eliminates that drift risk: production and tests link the same
 * symbol via include. This mirrors the established `DirectionPolicy.h` pattern
 * (whose own promotion killed `replica_resolveDirection`). The Python sim does
 * the same — its `resolve_stop_at_line` is a module-level public helper called
 * by both the runtime FSM and the offline compiler. See the Option-B plan and
 * PR #34 review for the full rationale.
 *
 * Cross-language parity: keep this step-for-step identical with
 * `tools/path_planner/sim/runtime.py::resolve_stop_at_line`. The fixture
 * `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv` (T0) is the
 * normative source both transcribe; TC13 enforces parity.
 */

/**
 * @brief Outcome of the stop-at-line decision for one BLIND hop.
 *
 * @note `motionMode` is always a concrete velocity bundle — never
 *       `Routes::BlindMode::STOP_AT_LINE` and never `Routes::BlindMode::DEFAULT`.
 *       When `seek` is true it is the seek's drive mode; when false it is the
 *       dead-reckoned fallback's drive mode.
 */
struct StopAtLineResolution
{
    bool              seek;          ///< true -> dispatch the Drive.stopAtLine seek.
    Routes::BlindMode motionMode;    ///< Motion bundle; NEVER STOP_AT_LINE or DEFAULT.
};

/**
 * @brief Resolve whether a BLIND hop seeks a line and which motion bundle it runs.
 *
 * @param callFlag       PathTracker's `_activeStopAtLine` (ON/OFF/AUTO).
 * @param edgeMode       `_resolveBlindMode(edge)` — already DEFAULT->pipeline
 *                       resolved, so never `Routes::BlindMode::DEFAULT`.
 * @param pipelineBlind  `Routes::PIPELINE_CONFIG.blind` — the mission-wide blind
 *                       default, used as the dead-reckon fallback for a
 *                       `STOP_AT_LINE` edge that does not seek.
 * @param direction      `_resolvedDirection` — the post-`resolveDirection`
 *                       travel direction (FRONT/REAR; never AUTO here).
 * @param lineBoardCount `Config::LINE_BOARD_COUNT` — board count gates a rear seek.
 * @return The `{seek, motionMode}` decision per the T0 truth table.
 */
inline StopAtLineResolution resolveStopAtLine(
    StopAtLine        callFlag,
    Routes::BlindMode edgeMode,
    Routes::BlindMode pipelineBlind,
    Direction         direction,
    uint8_t           lineBoardCount)
{
    StopAtLineResolution result;

    // Step 1: who asked for the seek? ON always; AUTO only on a STOP_AT_LINE
    // edge; OFF never.
    const bool seekRequested =
        (callFlag == StopAtLine::ON)
        || (callFlag == StopAtLine::AUTO && edgeMode == Routes::BlindMode::STOP_AT_LINE);

    // Step 2: a line board must face the travel direction. Front always; rear
    // needs a second board.
    const bool boardOK = (direction == Direction::FRONT) || (lineBoardCount >= 2);

    // Step 3: the seek runs only when requested and physically possible.
    result.seek = seekRequested && boardOK;

    if (result.seek)
    {
        // Step 4: keep the edge's typed mode; a bare STOP_AT_LINE edge seeks
        // under SYNC_TSHAPE (it has no velocity profile of its own).
        result.motionMode = (edgeMode == Routes::BlindMode::STOP_AT_LINE)
                                ? Routes::BlindMode::SYNC_TSHAPE
                                : edgeMode;
    }
    else
    {
        // Step 5: dead-reckon fallback. A STOP_AT_LINE edge falls back to the
        // pipeline blind mode; any residual STOP_AT_LINE defensively degrades to
        // SYNC_TSHAPE so the returned mode is never STOP_AT_LINE.
        const Routes::BlindMode base = (edgeMode == Routes::BlindMode::STOP_AT_LINE)
                                           ? pipelineBlind
                                           : edgeMode;
        result.motionMode = (base == Routes::BlindMode::STOP_AT_LINE)
                                ? Routes::BlindMode::SYNC_TSHAPE
                                : base;
    }

    return result;
}

#endif    // STOP_AT_LINE_POLICY_H
