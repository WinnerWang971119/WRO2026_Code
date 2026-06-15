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
 * dead-reckoned fallback use? The seek decision follows a strict precedence
 * ladder; the first rung that matches decides `requested`:
 *
 *   1. Call-site `StopAtLine::ON` / `StopAtLine::OFF` (`q.callFlag`) — a per-hop
 *      verb beats everything below it.
 *   2. Per-edge `Routes::StopAtLineMode::ON` / `OFF` (`q.edgeOverride`) — the
 *      Routes `stop_at_line_override` baked by the graph builder.
 *   3. Legacy force-on: the resolved edge mode (`q.edgeMode`) is itself
 *      `Routes::BlindMode::STOP_AT_LINE` (the historical "STOP_AT_LINE tag means
 *      seek" behaviour, kept for back-compat).
 *   4. Auto-rule: `q.autoRuleActive` (the caller's
 *      `autoSeekEnabled && nextHopIsProjection`) — a blind hop feeding a
 *      projection seeks a line first to re-anchor. This rung sets
 *      `autoTriggered` so the seek can be flagged a soft fallback.
 *   5. Otherwise `requested = false` — a call-site/edge `AUTO`/`DEFAULT` with no
 *      rule active never seeks. There is intentionally NO pipeline-default seek
 *      rung: the resolved `edgeMode` already encodes the pipeline default, so
 *      the degenerate `edgeMode != STOP_AT_LINE, pipelineBlind == STOP_AT_LINE`
 *      case still yields `seek = false` (full-domain parity with the shipping
 *      resolver).
 *
 * boardOK: a line board must face the travel direction — the front board always
 * faces forward, but a rear seek needs a second board (`lineBoardCount >= 2`).
 * `seek = requested && boardOK`.
 *
 * softFallback: `seek && autoTriggered` — true ONLY when the seek was raised by
 * the auto rung (step 4) AND a board could service it (boardOK). A board-degraded
 * auto case (`seek` false) carries no soft fallback, and any higher-precedence
 * rung (call / edge / legacy) yields `softFallback` false. The firmware uses this
 * flag to decide whether a missing line is tolerable (soft) or a hard fault.
 *
 * motionMode: when seeking, the motion bundle is the edge's mode, except a bare
 * `STOP_AT_LINE` edge has no real velocity profile of its own and seeks under
 * `SYNC_TSHAPE`. When not seeking, the hop dead-reckons: a `STOP_AT_LINE` edge
 * falls back to the pipeline blind mode; any residual `STOP_AT_LINE` (e.g. a
 * `STOP_AT_LINE` pipeline default) defensively degrades to `SYNC_TSHAPE`. The
 * returned `motionMode` is therefore never `STOP_AT_LINE` and never `DEFAULT`.
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
 * `tools/path_planner/sim/runtime.py::resolve_stop_at_line` — same precedence
 * ladder, same `edgeOverride` / `autoRuleActive` inputs, same `softFallback`
 * output. The fixture
 * `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv` is the
 * normative source both transcribe; TC13 enforces parity across the full domain.
 */

/**
 * @brief Inputs to the stop-at-line decision for one BLIND hop.
 *
 * @note Field order mirrors the Python signature
 *       `resolve_stop_at_line(call_flag, edge_override, edge_mode,
 *       pipeline_blind, direction, line_board_count, auto_rule_active)` so the
 *       two surfaces read identically.
 */
struct StopAtLineQuery
{
    StopAtLine             callFlag;          ///< NavTypes call-site flag (ON/OFF/AUTO).
    Routes::StopAtLineMode edgeOverride;      ///< Per-edge override (DEFAULT/ON/OFF).
    Routes::BlindMode      edgeMode;          ///< Resolved blind mode (never DEFAULT).
    Routes::BlindMode      pipelineBlind;     ///< Routes::PIPELINE_CONFIG.blind.
    Direction              direction;         ///< FRONT / REAR (never AUTO).
    uint8_t                lineBoardCount;    ///< Config::LINE_BOARD_COUNT.
    bool                   autoRuleActive;    ///< autoSeekEnabled && nextHopIsProjection.
};

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
    bool              seek;            ///< true -> dispatch the Drive.stopAtLine seek.
    Routes::BlindMode motionMode;     ///< Motion bundle; NEVER STOP_AT_LINE or DEFAULT.
    bool              softFallback;    ///< true ONLY for auto-rule-triggered seeks.
};

/**
 * @brief Resolve whether a BLIND hop seeks a line and which motion bundle it runs.
 *
 * @param q Inputs for this hop (@ref StopAtLineQuery):
 *          - `callFlag`       PathTracker's `_activeStopAtLine` (ON/OFF/AUTO).
 *          - `edgeOverride`   the per-edge `stop_at_line_override`
 *                             (DEFAULT/ON/OFF) baked into Routes by the graph
 *                             builder.
 *          - `edgeMode`       `_resolveBlindMode(edge)` — already DEFAULT->pipeline
 *                             resolved, so never `Routes::BlindMode::DEFAULT`.
 *          - `pipelineBlind`  `Routes::PIPELINE_CONFIG.blind` — the mission-wide
 *                             blind default, used as the dead-reckon fallback for
 *                             a `STOP_AT_LINE` edge that does not seek.
 *          - `direction`      `_resolvedDirection` — the post-`resolveDirection`
 *                             travel direction (FRONT/REAR; never AUTO here).
 *          - `lineBoardCount` `Config::LINE_BOARD_COUNT` — gates a rear seek.
 *          - `autoRuleActive` the caller's
 *                             `autoSeekEnabled && nextHopIsProjection` — true when
 *                             the field-wide auto-seek toggle is on AND this blind
 *                             hop feeds a projection edge.
 * @return The `{seek, motionMode, softFallback}` decision per the T0 truth table.
 *         `seek` is true iff the `Drive.stopAtLine` seek should be dispatched;
 *         `motionMode` is the seek's drive mode (when seeking) or the dead-reckon
 *         fallback's drive mode (when not) — always a concrete bundle, never
 *         `STOP_AT_LINE` or `DEFAULT`; `softFallback` is true iff the seek was
 *         raised by the auto rung AND a board could service it.
 */
inline StopAtLineResolution resolveStopAtLine(const StopAtLineQuery& q)
{
    StopAtLineResolution result;

    // Step 1: precedence ladder for who asked. Call-site ON/OFF beats the
    // per-edge override, which beats the legacy STOP_AT_LINE tag, which beats
    // the auto-rule. AUTO call + DEFAULT edge + no rule active never seeks.
    bool requested     = false;
    bool autoTriggered = false;
    if (q.callFlag == StopAtLine::ON)
    {
        requested = true;
    }
    else if (q.callFlag == StopAtLine::OFF)
    {
        requested = false;
    }
    else if (q.edgeOverride == Routes::StopAtLineMode::ON)
    {
        requested = true;
    }
    else if (q.edgeOverride == Routes::StopAtLineMode::OFF)
    {
        requested = false;
    }
    else if (q.edgeMode == Routes::BlindMode::STOP_AT_LINE)    // legacy force-on
    {
        requested = true;
    }
    else if (q.autoRuleActive)
    {
        requested     = true;
        autoTriggered = true;
    }
    else    // AUTO call / DEFAULT edge, no rule active: never seek
    {
        requested = false;
    }

    // Step 2: a line board must face the travel direction. Front always; rear
    // needs a second board.
    const bool boardOK = (q.direction == Direction::FRONT) || (q.lineBoardCount >= 2);

    // Step 3: the seek runs only when requested and physically possible.
    result.seek = requested && boardOK;

    // Step 3b: a soft fallback is an auto-raised seek that a board can service.
    // A board-degraded auto case (seek false) or any higher-precedence rung
    // (autoTriggered false) never carries it.
    result.softFallback = result.seek && autoTriggered;

    if (result.seek)
    {
        // Step 4: keep the edge's typed mode; a bare STOP_AT_LINE edge seeks
        // under SYNC_TSHAPE (it has no velocity profile of its own).
        result.motionMode = (q.edgeMode == Routes::BlindMode::STOP_AT_LINE)
                                ? Routes::BlindMode::SYNC_TSHAPE
                                : q.edgeMode;
    }
    else
    {
        // Step 5: dead-reckon fallback. A STOP_AT_LINE edge falls back to the
        // pipeline blind mode; any residual STOP_AT_LINE defensively degrades to
        // SYNC_TSHAPE so the returned mode is never STOP_AT_LINE.
        const Routes::BlindMode base = (q.edgeMode == Routes::BlindMode::STOP_AT_LINE)
                                           ? q.pipelineBlind
                                           : q.edgeMode;
        result.motionMode = (base == Routes::BlindMode::STOP_AT_LINE)
                                ? Routes::BlindMode::SYNC_TSHAPE
                                : base;
    }

    return result;
}

#endif    // STOP_AT_LINE_POLICY_H
