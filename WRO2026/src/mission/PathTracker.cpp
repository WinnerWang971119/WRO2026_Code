/**
 * @file PathTracker.cpp
 * @brief Implementation of the PathTracker FSM (PR3 + pipeline-modes update).
 * @author WRO 2026
 */

#include "PathTracker.h"

#include <Arduino.h>
#include <math.h>

#include "../nav/DirectionPolicy.h"
#include "../drive/Drive.h"
#include "../nav/EdgeDispatchPolicy.h"
#include "../util/EventTrace.h"
#include "../drive/Intersection.h"
#include "../drive/LineFollower.h"
#include "../nav/Localization.h"
#include "PathTrackerParityTrace.h"
#include "../nav/PivotTrimPolicy.h"
#include "../nav/ReturnToNodePolicy.h"
#include "../nav/ReversePivotPolicy.h"
#include "../nav/StopAtLinePolicy.h"
#include "../util/WROMath.h"    // encDegToCm — the ADR-059 measured boundary-carry conversion

PathTrackerClass PathTracker;

// =========================================================================================
// Mode resolution helpers (file-local).
//
// Each `Routes::EdgeInfo` carries per-category override fields
// (`speed_override`, `blind_mode_override`, `direction_override`,
// `stop_at_line_override`). The value `DEFAULT` means "inherit from
// `Routes::PIPELINE_CONFIG.<category>`"; anything else wins over the
// pipeline-wide default. These helpers resolve DEFAULT → concrete at
// edge-dispatch time so the FSM never has to chase the override semantics
// inline. The pre-edge turn wheel is resolved per turn from the
// `Routes::PivotTurn` table (ADR-036), not from an `EdgeInfo` field.
// =========================================================================================

namespace
{
    /// Resolve the per-edge cruise SPEED tier for the given edge (both LINE and
    /// BLIND). `SpeedTier::DEFAULT` inherits the mission-wide `PIPELINE_CONFIG.speed`.
    Routes::SpeedTier _resolveSpeed(const Routes::EdgeInfo& edge)
    {
        return (edge.speed_override == Routes::SpeedTier::DEFAULT)
            ? Routes::PIPELINE_CONFIG.speed
            : edge.speed_override;
    }

    /// Resolve the BLIND-edge motion-profile mode for the given edge.
    Routes::BlindMode _resolveBlindMode(const Routes::EdgeInfo& edge)
    {
        return (edge.blind_mode_override == Routes::BlindMode::DEFAULT)
            ? Routes::PIPELINE_CONFIG.blind
            : edge.blind_mode_override;
    }

    // NOTE: the production direction-resolution helper has been promoted to
    // `DirectionPolicy.h::resolveDirection` (header-only inline) so both
    // production and the host-compile test target link the same definition.
    // See ADR-024 and the PR #34 review.

    /// Emit the standard REVERSE_UNAVAILABLE diagnostic. Shared by both guard sites
    /// (the load-bearing pre-turn guard in `_onPlanNextHop` and the defensive post-turn
    /// guard in `_dispatchEdgeAfterTurn`) so the two messages cannot drift; @p site names
    /// which guard fired. The caller still issues `_enterFault(...)`.
    ///
    /// Issue #64: the only reverse-unservable edge type is now `LINE` (a REAR line-follow
    /// needs the rear board; BLIND REAR is permitted on a 1-board chassis). The
    /// `reverseUnavailable(...)` predicate is scoped to `EdgeType::LINE`, so this helper is
    /// only ever reached with a LINE edge — the label is unconditional (no edge-type param).
    void _printReverseUnavailable(uint8_t from, uint8_t to, const __FlashStringHelper* site)
    {
        Serial.print(F("[PathTracker] REVERSE_UNAVAILABLE: edge "));
        Serial.print(F("LINE"));
        Serial.print(F(" at u="));
        Serial.print(from);
        Serial.print(F(" v="));
        Serial.print(to);
        Serial.print(F(" requires rear board ("));
        Serial.print(site);
        Serial.println(F(")"));
    }

    /**
     * @brief Map a `Routes::TurnMode` to the wheel-pattern byte consumed by
     *        `TurnDegBuilder::setMode(uint8_t)` /
     *        `TurnToYawBuilder::setMode(uint8_t)`.
     *
     * Byte encoding matches the custom TurnGyro state machine introduced by the
     * 2026-05-17 PR4 plan (see `Drive.h::TurnDegBuilder::setMode` and
     * `Config::TURN_MODE` for the authoritative contract):
     *
     *   0 = COUNTER_ROT — both wheels spin opposite directions; tightest in-place rotation.
     *   1 = PIVOT_LEFT  — left wheel braked, right drives; chassis pivots around left wheel.
     *   2 = PIVOT_RIGHT — right wheel braked, left drives; chassis pivots around right wheel.
     *
     * @note Intentionally distinct from the legacy `MiniR4.DriveDC.TurnGyro` byte values —
     *       the lower-MCU TurnGyro is no longer called.
     * @note `TurnMode::DEFAULT` should not reach this helper (callers resolve the pre-turn
     *       wheel — pivot wheel or pipeline default — to a concrete value first). It is
     *       mapped to counter-rotation defensively.
     */
    uint8_t _turnModeToByte(Routes::TurnMode m)
    {
        switch (m)
        {
            case Routes::TurnMode::COUNTER_ROT: return 0;
            case Routes::TurnMode::PIVOT_LEFT:  return 1;
            case Routes::TurnMode::PIVOT_RIGHT: return 2;
            case Routes::TurnMode::DEFAULT:     return 0;    // defensive — see doc above
            default:                            return 0;
        }
    }

    /**
     * @brief Map a resolved `Routes::BlindMode` motion bundle to the `DriveMode`
     *        the `Drive.stopAtLine` seek runs under.
     *
     * @param m The `StopAtLineResolution::motionMode` from `resolveStopAtLine(...)`.
     *          Always one of the three concrete velocity bundles — `SYNC_TSHAPE`,
     *          `GYRO_TSHAPE`, or `RAW_FLAT` — so `STOP_AT_LINE` and `DEFAULT` never
     *          reach this helper.
     * @return The matching `DriveMode` (`SYNC`/`GYRO`/`RAW`).
     * @note The `default` returns `DriveMode::SYNC` purely defensively; the resolver
     *       guarantees `motionMode` is never `STOP_AT_LINE`/`DEFAULT`.
     */
    DriveMode blindModeToDriveMode(Routes::BlindMode m)
    {
        switch (m)
        {
            case Routes::BlindMode::SYNC_TSHAPE: return DriveMode::SYNC;
            case Routes::BlindMode::GYRO_TSHAPE: return DriveMode::GYRO;
            case Routes::BlindMode::RAW_FLAT:    return DriveMode::RAW;
            default:                             return DriveMode::SYNC;    // defensive — see doc above
        }
    }

    /// Print the shared over-cap diagnostic prefix for the two off-map gate branches (#12):
    /// "[PathTracker] return-to-node: offset <mag> cm exceeds cap <cap> cm; OFF_MAP_GATE=".
    /// Each branch appends only its gate-specific tail, so a later format change touches one
    /// place instead of drifting between the FAULT and BEST_EFFORT messages.
    void _printReturnOverCapPrefix(float mag)
    {
        Serial.print(F("[PathTracker] return-to-node: offset "));
        Serial.print(mag);
        Serial.print(F(" cm exceeds cap "));
        Serial.print(Config::RETURN_DISTANCE_CAP_CM);
        Serial.print(F(" cm; OFF_MAP_GATE="));
    }

    /// Print the shared overshoot-trim underflow trace prefix for the two sensor-anchored
    /// pivot paths (#6): the LINE overshoot reduction and the STOP_AT_LINE seek. Each caller
    /// appends its own tail (the LINE path clamps and continues; the seek path faults — they
    /// diverge only there), so a later format change touches one place. @p adrTag names the
    /// ADR governing that path (ADR-036 LINE / ADR-044 seek). Mirrors the
    /// `_printReturnOverCapPrefix` prefix-helper pattern.
    void _printOvershootTrimUnderflow(const __FlashStringHelper* adrTag, float offset,
                                      float delta)
    {
        Serial.print(F("[PathTracker] "));
        Serial.print(adrTag);
        Serial.print(F(": overshoot trim underflow (offset="));
        Serial.print(offset);
        Serial.print(F(" delta="));
        Serial.print(delta);
        Serial.print(F("); "));
    }

    /// Print the ADR-059 blind-chain CAP-truncation diagnostic (Error Handling: "no silent
    /// cap"). Fired by the walk-ahead ONLY when an otherwise-ELIGIBLE hop is refused by a run
    /// cap (`MAX_CHAIN_HOPS` or `PATH_BLIND_CHAIN_MAX_CM`) — ordinary chain ends (bend, seek,
    /// goal, ...) are silent, so this line never spams a normal route. @p reason names the cap
    /// that fired ("hop cap" / "distance cap"); @p nextNode is the first hop LEFT OUT of the
    /// run (it dispatches later as a normal braked hop; a fresh chain may re-form there).
    void _printChainCapTruncation(const __FlashStringHelper* reason, uint8_t hops,
                                  float totalCm, uint8_t nextNode)
    {
        Serial.print(F("[PathTracker] ADR-059: chain capped ("));
        Serial.print(reason);
        Serial.print(F(") at "));
        Serial.print(hops);
        Serial.print(F(" hops / "));
        Serial.print(totalCm);
        Serial.print(F(" cm before node "));
        Serial.println(FieldMap::getNode(nextNode).name);
    }
}    // namespace

// =========================================================================================
// Parity trace helpers (issue #108) — compiled in ONLY when PT_PARITY_TRACE is defined.
//
// Each builder serializes one normalized NDJSON record (the cross-language wire schema
// in the parity plan) and emits it via `pt_parity::emit`. Floats are fixed 6-decimal
// (`%.6f`) to match the comparator. Node names (not ids) and the enum string spellings
// the `resolveStopAtLine` CSV already uses keep the C++ and Python streams aligned.
// The whole block — and every call site — vanishes from the firmware build, so the
// trace can never affect timing or behavior on the robot.
// =========================================================================================

#ifdef PT_PARITY_TRACE
namespace
{
    const char* _ptWheelName(Routes::TurnMode m)
    {
        switch (m)
        {
            case Routes::TurnMode::COUNTER_ROT: return "COUNTER_ROT";
            case Routes::TurnMode::PIVOT_LEFT:  return "PIVOT_LEFT";
            case Routes::TurnMode::PIVOT_RIGHT: return "PIVOT_RIGHT";
            case Routes::TurnMode::DEFAULT:
            default:                            return "COUNTER_ROT";    // byte 0, see _turnModeToByte
        }
    }

    const char* _ptDriveModeName(Routes::BlindMode m)
    {
        switch (m)
        {
            case Routes::BlindMode::SYNC_TSHAPE: return "SYNC_TSHAPE";
            case Routes::BlindMode::GYRO_TSHAPE: return "GYRO_TSHAPE";
            case Routes::BlindMode::RAW_FLAT:    return "RAW_FLAT";
            default:                             return "SYNC_TSHAPE";    // resolver guarantees concrete
        }
    }

    /// Spell a resolved `Routes::SpeedTier` for the parity `speed` wire key (RENAMED from the
    /// historically-misnamed `line_mode` in the issue-#156 v3 bump — this helper's name is kept to
    /// limit churn, but it produces the SPEED tier, not a follow mode). The value is the SpeedTier
    /// member name (SLOW/NORMAL/FAST) so the sim's parity mirror stays byte-compatible.
    const char* _ptLineModeName(Routes::SpeedTier m)
    {
        switch (m)
        {
            case Routes::SpeedTier::NORMAL: return "NORMAL";
            case Routes::SpeedTier::SLOW:   return "SLOW";
            case Routes::SpeedTier::FAST:   return "FAST";
            case Routes::SpeedTier::DEFAULT:
            default:                        return "NORMAL";    // resolved before reaching here
        }
    }

    /// Spell a RESOLVED `Routes::LineFollowMode` for the parity `follow_mode` wire key
    /// (issue #156). The value is the LineFollowMode member name (MIDDLE/LEFT/RIGHT) — the MODE,
    /// recorded PRE-reverse-sign, never the signed float offset. `_resolveLineFollowMode` resolves
    /// DEFAULT->MIDDLE upstream, so DEFAULT is never reached here (defensive fallback to MIDDLE).
    const char* _ptFollowModeName(Routes::LineFollowMode m)
    {
        switch (m)
        {
            case Routes::LineFollowMode::MIDDLE: return "MIDDLE";
            case Routes::LineFollowMode::LEFT:   return "LEFT";
            case Routes::LineFollowMode::RIGHT:  return "RIGHT";
            case Routes::LineFollowMode::DEFAULT:
            default:                             return "MIDDLE";    // resolved before reaching here
        }
    }

    const char* _ptBool(bool b) { return b ? "true" : "false"; }

    const char* _ptNodeName(uint8_t id) { return FieldMap::getNode(id).name; }

    /// `turn` / `return_turn` — {body_heading_deg, wheel}.
    void _ptEmitTurn(const char* kind, float bodyHeadingDeg, Routes::TurnMode wheel)
    {
        char line[256];
        std::snprintf(line, sizeof(line),
                      "{\"kind\":\"%s\",\"body_heading_deg\":%.6f,\"wheel\":\"%s\"}", kind,
                      (double) bodyHeadingDeg, _ptWheelName(wheel));
        pt_parity::emit(line);
    }

    /// `line_follow` — {reverse, speed, follow_mode} (parity v3, issue #156). `speed` is the
    /// resolved SpeedTier member name — RENAMED from the historically-misnamed `line_mode`, which
    /// always held the speed tier. `follow_mode` is the RESOLVED LineFollowMode member name
    /// (MIDDLE/LEFT/RIGHT) — the MODE, recorded PRE-reverse-sign, never the signed float offset.
    void _ptEmitLineFollow(bool reverse, Routes::SpeedTier speed, Routes::LineFollowMode followMode)
    {
        char line[256];
        std::snprintf(line, sizeof(line),
                      "{\"kind\":\"line_follow\",\"reverse\":%s,\"speed\":\"%s\","
                      "\"follow_mode\":\"%s\"}",
                      _ptBool(reverse), _ptLineModeName(speed), _ptFollowModeName(followMode));
        pt_parity::emit(line);
    }

    /// `line_overshoot` — {advance_cm, pass_through, reanchor:"ON_INTERSECTION"}.
    void _ptEmitLineOvershoot(float advanceCm, bool passThrough)
    {
        char line[256];
        std::snprintf(line, sizeof(line),
                      "{\"kind\":\"line_overshoot\",\"advance_cm\":%.6f,\"pass_through\":%s,"
                      "\"reanchor\":\"ON_INTERSECTION\"}",
                      (double) advanceCm, _ptBool(passThrough));
        pt_parity::emit(line);
    }

    /// `projection_follow` — {reverse, speed, follow_mode, dist_cm, pass_through,
    /// reanchor:"ON_DEAD_RECKONED_ADVANCE"} (parity v3, issue #156). `speed` is the resolved
    /// SpeedTier member name — RENAMED from the historically-misnamed `line_mode`. `follow_mode`
    /// is the RESOLVED LineFollowMode member name (MIDDLE/LEFT/RIGHT) — the MODE, recorded
    /// PRE-reverse-sign, never the signed float offset.
    void _ptEmitProjectionFollow(bool reverse, Routes::SpeedTier speed,
                                 Routes::LineFollowMode followMode, float distCm, bool passThrough)
    {
        char line[256];
        std::snprintf(line, sizeof(line),
                      "{\"kind\":\"projection_follow\",\"reverse\":%s,\"speed\":\"%s\","
                      "\"follow_mode\":\"%s\",\"dist_cm\":%.6f,\"pass_through\":%s,"
                      "\"reanchor\":\"ON_DEAD_RECKONED_ADVANCE\"}",
                      _ptBool(reverse), _ptLineModeName(speed), _ptFollowModeName(followMode),
                      (double) distCm, _ptBool(passThrough));
        pt_parity::emit(line);
    }

    /// `stop_at_line_seek` — {reverse, drive_mode, dist_cm, overshoot_cm,
    /// reanchor:"ON_INTERSECTION"}.
    void _ptEmitStopAtLineSeek(bool reverse, Routes::BlindMode driveMode, float distCm,
                               float overshootCm)
    {
        char line[256];
        std::snprintf(line, sizeof(line),
                      "{\"kind\":\"stop_at_line_seek\",\"reverse\":%s,\"drive_mode\":\"%s\","
                      "\"dist_cm\":%.6f,\"overshoot_cm\":%.6f,\"reanchor\":\"ON_INTERSECTION\"}",
                      _ptBool(reverse), _ptDriveModeName(driveMode), (double) distCm,
                      (double) overshootCm);
        pt_parity::emit(line);
    }

    /// `blind_dead_reckon` — {reverse, drive_mode, dist_cm, start_pwm, end_pwm, chain,
    /// reanchor:"ON_DEAD_RECKONED_ADVANCE"}. ADR-059 field-set bump (strict schema — the
    /// three new fields are emitted on EVERY blind_dead_reckon record, chained or not; the
    /// exact wire contract lives in PathTrackerParityTrace.h's "ADR-059 wire schema" block).
    /// `start_pwm`/`end_pwm` are the RESOLVED velocity-PLAN endpoints (ChainHopSpeed for a
    /// chained hop; the plain-tShape / RAW_FLAT plan otherwise), NOT any degraded dispatch
    /// detail, so the sim (which has no encoder noise) emits identical values. NOTE the
    /// chain flag key is `chain` — NEVER a sub-field named "kind"
    /// (memory `gotcha-parity-record-kind-is-discriminator`).
    void _ptEmitBlindDeadReckon(bool reverse, Routes::BlindMode driveMode, float distCm,
                                int16_t startPwm, int16_t endPwm, bool chain)
    {
        char line[256];
        std::snprintf(line, sizeof(line),
                      "{\"kind\":\"blind_dead_reckon\",\"reverse\":%s,\"drive_mode\":\"%s\","
                      "\"dist_cm\":%.6f,\"start_pwm\":%d,\"end_pwm\":%d,\"chain\":%s,"
                      "\"reanchor\":\"ON_DEAD_RECKONED_ADVANCE\"}",
                      _ptBool(reverse), _ptDriveModeName(driveMode), (double) distCm,
                      (int) startPwm, (int) endPwm, _ptBool(chain));
        pt_parity::emit(line);
    }

    /// `blind_pass` — {node}. ADR-059: emitted at each NON-braking mid-hop node boundary of
    /// a blind-chain run — the tick the chained hop's move retires in `BLIND_FORWARD`,
    /// immediately BEFORE the `onDeadReckonedAdvance` stamp, so the record sits between the
    /// two hops' `blind_dead_reckon` records in the stream (the sim mirrors the same
    /// ordering; the tag maps to the sim's `ACT_BLIND_PASS = "blind_pass"`). The run's FINAL
    /// (braked) arrival emits NO pass record. Deliberately minimal: the chained speeds
    /// already ride the surrounding drive records.
    void _ptEmitBlindPass(uint8_t node)
    {
        char line[256];
        std::snprintf(line, sizeof(line), "{\"kind\":\"blind_pass\",\"node\":\"%s\"}",
                      _ptNodeName(node));
        pt_parity::emit(line);
    }

    /// `return_move` — {mode:"seek"|"forward", node, dist_cm, heading_world_deg}.
    void _ptEmitReturnMove(const char* mode, uint8_t node, float distCm, float headingWorldDeg)
    {
        char line[256];
        std::snprintf(line, sizeof(line),
                      "{\"kind\":\"return_move\",\"mode\":\"%s\",\"node\":\"%s\","
                      "\"dist_cm\":%.6f,\"heading_world_deg\":%.6f}",
                      mode, _ptNodeName(node), (double) distCm, (double) headingWorldDeg);
        pt_parity::emit(line);
    }

    /// `wall_stop` / `wall_reanchor` / `wall_depart` — {node, axis_idx, <coordKey>}.
    /// `axis_idx`: 0 = x, 1 = y. `coordKey` is "wall_coord" (stop/reanchor) or
    /// "node_coord" (depart).
    void _ptEmitWall(const char* kind, uint8_t node, uint8_t axisIdx, const char* coordKey,
                     float coord)
    {
        char line[256];
        std::snprintf(line, sizeof(line),
                      "{\"kind\":\"%s\",\"node\":\"%s\",\"axis_idx\":%u,\"%s\":%.6f}", kind,
                      _ptNodeName(node), (unsigned) axisIdx, coordKey, (double) coord);
        pt_parity::emit(line);
    }

    /// `heading_reset` — {node, pass_kind, heading_deg}. Emitted ONLY on a successful bind, so
    /// its mere presence means the reset bound — there is NO `bound`/`result` field. NOTE the
    /// proj/junction tag is carried in `pass_kind`, NOT `kind`: `kind` is the record-type
    /// discriminator ("heading_reset") the parity loader keys on, and reusing it here would emit
    /// a duplicate `"kind"` key (invalid JSON). @p kind is the proj/junction pass tag string.
    void _ptEmitHeadingReset(uint8_t node, const char* kind, float headingDeg)
    {
        char line[256];
        std::snprintf(line, sizeof(line),
                      "{\"kind\":\"heading_reset\",\"node\":\"%s\",\"pass_kind\":\"%s\","
                      "\"heading_deg\":%.6f}",
                      _ptNodeName(node), kind, (double) headingDeg);
        pt_parity::emit(line);
    }
}    // namespace
#endif    // PT_PARITY_TRACE

// =========================================================================================
// PathTrackerGoToBuilder
// =========================================================================================

PathTrackerGoToBuilder::PathTrackerGoToBuilder(PathTrackerClass& owner, uint8_t goalId)
    : _owner(owner)
    , _goalId(goalId)
{
}

PathTrackerGoToBuilder::~PathTrackerGoToBuilder()
{
    if (_executed)
    {
        return;
    }
    _owner._applyGoToBuilder(
        _goalId, _blocking, _routeDirection, _arrivalDirection, _stopAtLine);
    _executed = true;
}

PathTrackerGoToBuilder& PathTrackerGoToBuilder::async()
{
    _blocking = false;
    return *this;
}

PathTrackerGoToBuilder& PathTrackerGoToBuilder::setRouteDirection(Direction d)
{
    _routeDirection = d;
    return *this;
}

PathTrackerGoToBuilder& PathTrackerGoToBuilder::setArrivalDirection(Direction d)
{
    _arrivalDirection = d;
    return *this;
}

PathTrackerGoToBuilder& PathTrackerGoToBuilder::setStopAtLine(StopAtLine s)
{
    _stopAtLine = s;
    return *this;
}

// =========================================================================================
// PathTrackerClass — lifecycle
// =========================================================================================

bool PathTrackerClass::begin()
{
    _qHead  = 0;
    _qTail  = 0;
    _qCount = 0;
    _state  = State::IDLE;
    _destinationNode         = FieldMap::NODE_NONE;
    _nextHop                 = FieldMap::NODE_NONE;
    _activeEdgeType          = Routes::EdgeType::NONE;
    _passThrough             = false;
    _projPassThrough         = false;
    _lineFollowPassThrough   = false;
    _resolvedDirection       = Direction::AUTO;
    _resolvedBodyHeading     = 0.0f;
    _returnUsesLineSeek      = false;
    _returnActive            = false;
    _returnDistCm            = 0.0f;
    // #146: no lost-line escalation window in flight at boot.
    _lineLostTiming          = false;
    _lineLostSinceMs         = 0;
    // ADR-061: no settle dwell in flight at boot (the SETTLE state is the real guard; this is
    // defensive parity with the other latch resets).
    _settleDeadlineMs        = 0;
    _settleResume            = SettleResume::PLAN;
    // PR2: no wall-zero reset interlude in flight at boot.
    _resetNode               = FieldMap::NODE_NONE;
    // ADR-036: no pivot latched at boot.
    _resetPivotLatch();
    // ADR-059: no blind-chain run in flight at boot.
    _chainReset();
    _activeRouteDirection    = Direction::AUTO;
    _activeArrivalDirection  = Direction::AUTO;
    _activeStopAtLine        = StopAtLine::AUTO;
    _fault = FaultInfo{};
    return true;
}

void PathTrackerClass::setTickCallback(void (*cb)())
{
    _tickCallback                = cb;
    _tickCallbackWarningEmitted  = false;
}

bool PathTrackerClass::isIdle() const
{
    return (_qCount == 0) && (_state == State::IDLE) && (_fault.kind == Fault::NONE);
}

bool PathTrackerClass::hasFault() const
{
    return _fault.kind != Fault::NONE;
}

PathTrackerClass::FaultInfo PathTrackerClass::getFault() const
{
    return _fault;
}

void PathTrackerClass::clearFault()
{
    _fault = FaultInfo{};
    if (_state == State::FAULT)
    {
        _state = State::IDLE;
    }
    // ADR-067: cascade the clear to Drive. A latched Drive fault (e.g. a return seek that ran to
    // its max distance and latched `DriveClass::Fault::LINE_NOT_FOUND`, surfacing this tracker's
    // `Fault::DRIVE_FAULT`) is NOT touched by clearing our own fault, and `Drive.stop()` does not
    // clear it either. Without this the very next `goTo()` dispatch is frozen by Drive's fault
    // gate and the shared Drive-fault guard immediately re-trips `Fault::DRIVE_FAULT`, leaving the
    // robot unrecoverable without a reboot. A no-op when Drive carries no latched fault.
    Drive.clearFault();
}

void PathTrackerClass::abort()
{
    // Stop the chassis first. Disable LineFollower BEFORE Drive.stop(true)
    // because the follower writes to Drive every tick — if it is still
    // enabled after Drive.stop, the next cooperativeTick() iteration's
    // LineFollower.update() re-issues Drive.setSpeed(...) and undoes the
    // halt. Mirror of Mission::abort()'s ordering (Mission.cpp:166).
    LineFollower.setEnabled(false);
    Drive.stop(true);

    _clearQueue();
    _state                  = State::IDLE;
    _destinationNode        = FieldMap::NODE_NONE;
    _nextHop                = FieldMap::NODE_NONE;
    _activeEdgeType         = Routes::EdgeType::NONE;
    _passThrough            = false;
    _projPassThrough        = false;
    _lineFollowPassThrough  = false;
    _resolvedDirection      = Direction::AUTO;
    _resolvedBodyHeading    = 0.0f;
    _returnUsesLineSeek     = false;
    _returnActive           = false;
    _returnDistCm           = 0.0f;
    // #146: a hard-cancel drops any in-flight lost-line escalation window so a later goal cannot
    // inherit its accrued loss time.
    _lineLostTiming         = false;
    _lineLostSinceMs        = 0;
    // ADR-061: a hard-cancel drops any in-flight settle dwell so a later goal cannot inherit a
    // stale deadline.
    _settleDeadlineMs       = 0;
    _settleResume           = SettleResume::PLAN;
    // PR2: drop any in-flight wall-zero reset interlude so a re-arm on a later goal
    // re-fires the wall push.
    _resetNode              = FieldMap::NODE_NONE;
    // ADR-036: drop any latched pivot so a mid-pivot abort cannot leak the
    // outgoing trim into the next goal's first dead-reckon drive.
    _resetPivotLatch();
    // ADR-059: drop any in-flight blind-chain run — the chassis was hard-stopped, so the
    // chained speed plan (and its boundary-carry baseline) is void.
    _chainReset();
    _activeRouteDirection   = Direction::AUTO;
    _activeArrivalDirection = Direction::AUTO;
    _activeStopAtLine       = StopAtLine::AUTO;
    _fault                  = FaultInfo{};
    // ADR-067: cascade the fault clear to Drive. `Drive.stop(true)` above halts the chassis but
    // does NOT clear a latched Drive fault, so a hard-cancel that leaves Drive faulted would freeze
    // the next `goTo()` dispatch behind Drive's fault gate. A no-op when Drive is already clean.
    Drive.clearFault();
}

const char* PathTrackerClass::faultMnemonic(Fault kind)
{
    switch (kind)
    {
        case Fault::NONE:                 return "OK";
        case Fault::NOT_ZEROED:           return "NZ";
        case Fault::UNREACHABLE:          return "UNR";
        case Fault::DRIVE_FAULT:          return "DRV";
        case Fault::BAD_NODE:             return "BAD";
        case Fault::DRAIN_TIMEOUT:        return "DRN";
        case Fault::REVERSE_UNAVAILABLE:  return "REV";
        case Fault::OFF_NODE:             return "OFN";
        case Fault::WALL_NOT_SQUARE:      return "WSQ";
        case Fault::PIVOT_TRIM_UNDERFLOW: return "PTU";
        case Fault::LINE_LOST:            return "LLN";
        case Fault::PIVOT_DIRECTION_MISMATCH: return "PDM";
    }
    return "??";
}

// =========================================================================================
// Queue
// =========================================================================================

bool PathTrackerClass::_enqueue(uint8_t goalId, Direction routeDir, Direction arrivalDir)
{
    if (_qCount >= Config::PATH_QUEUE_CAPACITY)
    {
        Serial.println(F("[PathTracker] queue overflow; goTo dropped"));
        return false;
    }
    _queue[_qTail].nodeId            = goalId;
    _queue[_qTail].route_direction   = routeDir;
    _queue[_qTail].arrival_direction = arrivalDir;
    // Default the call-site stop-at-line override to AUTO so a recycled FIFO
    // slot never carries a stale value. The real value (when the operator
    // chained `.setStopAtLine(...)`) is written into this same slot by
    // `_applyGoToBuilder` immediately after this returns — `_enqueue` keeps its
    // 3-arg signature, mirroring the queue plumbing for the Direction overrides.
    _queue[_qTail].stop_at_line      = StopAtLine::AUTO;
    _qTail  = (uint8_t) ((_qTail + 1) % Config::PATH_QUEUE_CAPACITY);
    _qCount++;
    return true;
}

bool PathTrackerClass::_pop(NavEntry& out)
{
    if (_qCount == 0)
    {
        return false;
    }
    out    = _queue[_qHead];
    _qHead = (uint8_t) ((_qHead + 1) % Config::PATH_QUEUE_CAPACITY);
    _qCount--;
    return true;
}

void PathTrackerClass::_clearQueue()
{
    _qHead  = 0;
    _qTail  = 0;
    _qCount = 0;
}

// =========================================================================================
// Builder dispatch
// =========================================================================================

void PathTrackerClass::_applyGoToBuilder(
    uint8_t    goalId,
    bool       blocking,
    Direction  routeDir,
    Direction  arrivalDir,
    StopAtLine stopAtLine)
{
    // No-op when a fault is already latched. This MUST run before the
    // goal-id and isZeroed() checks below: those branches call
    // `_enterFault(...)`, which would overwrite the existing
    // `_fault.kind`/`fromNode`/`destNode` and destroy the original
    // diagnostic. Guarding first preserves the latched fault so the
    // operator still sees what tripped it (clear with clearFault()).
    if (_fault.kind != Fault::NONE)
    {
        Serial.print(F("[PathTracker] goTo while faulted (kind="));
        Serial.print(faultMnemonic(_fault.kind));
        Serial.println(F("); clear with clearFault()"));
        return;
    }
    if (goalId == FieldMap::NODE_NONE || goalId >= Routes::N_NODES)
    {
        _enterFault(Fault::BAD_NODE, Localization.getCurrentNode(), goalId);
        return;
    }
    if (!Localization.isZeroed())
    {
        _enterFault(Fault::NOT_ZEROED, Localization.getCurrentNode(), goalId);
        return;
    }
    // Slot that `_enqueue` is about to fill (it advances `_qTail` past it on
    // success). Captured BEFORE the call so the call-site stop-at-line override
    // can be written into the same NavEntry without widening `_enqueue`.
    const uint8_t slot = _qTail;
    if (!_enqueue(goalId, routeDir, arrivalDir))
    {
        return;
    }
    _queue[slot].stop_at_line = stopAtLine;
    if (blocking)
    {
        _drainBlocking();
    }
}

// =========================================================================================
// Drain spin
// =========================================================================================

void PathTrackerClass::_drainBlocking()
{
    const uint32_t budget = (uint32_t) (_qCount + 1) * Config::PATH_DRAIN_TIMEOUT_MS_PER_GOAL;
    const uint32_t deadline = millis() + budget;

    if (_tickCallback == nullptr && !_tickCallbackWarningEmitted)
    {
        Serial.println(
            F("[PathTracker] no tick callback registered; other modules will freeze "
              "during drain."));
        _tickCallbackWarningEmitted = true;
    }

    while (true)
    {
        if (_fault.kind != Fault::NONE)
        {
            return;
        }
        if (_qCount == 0 && _state == State::IDLE)
        {
            return;
        }
        if ((int32_t) (millis() - deadline) > 0)
        {
            Drive.stop(true);
            _enterFault(Fault::DRAIN_TIMEOUT, Localization.getCurrentNode(), _destinationNode);
            return;
        }
        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            update();
        }
        delay(2);
    }
}

// =========================================================================================
// FSM tick
// =========================================================================================

void PathTrackerClass::update()
{
    _stepFsm();

    // Push nav activity DOWN to the offset integrator (Localization is below us): it
    // accumulates operator offset only while we are NOT executing a hop. A latched
    // FAULT counts as not-driving so an operator can hand-correct and the next goTo
    // still returns. Idle also counts as not-driving.
    Localization.setNavActive(_state != State::IDLE && _state != State::FAULT);
}

void PathTrackerClass::_stepFsm()
{
    // ADR-038: a missed stop-at-line seek latches `DriveClass::Fault::LINE_NOT_FOUND`
    // and `Drive.hasFault()` goes true; for a normal (projection) hop the shared
    // Drive-fault guard below catches it and hard-faults into `Fault::DRIVE_FAULT`
    // (no soft fallback — a missed seek is always a hard fault).
    //
    // RETURN_TO_NODE is EXEMPT (alongside IDLE / FAULT): a `LINE_NOT_FOUND` on the
    // line re-acquire seek must instead route to the off-map gate (Fault::OFF_NODE
    // under FAULT, accept-dead-reckon under BEST_EFFORT) — see the plan's
    // "Line-return success check". If this guard fired for RETURN_TO_NODE it would
    // mislabel that as DRIVE_FAULT, so the RETURN_TO_NODE state body inspects
    // `Drive.hasFault()` / `Drive.isLineFound()` itself (issue #36, PR1b).
    if (_state != State::IDLE && _state != State::FAULT && _state != State::RETURN_TO_NODE
        && Drive.hasFault())
    {
        Drive.stop(true);
        _enterFault(Fault::DRIVE_FAULT, Localization.getCurrentNode(), _destinationNode);
        return;
    }

    switch (_state)
    {
        case State::IDLE:
        {
            if (_qCount > 0)
            {
                _state = State::DEQUEUE;
            }
            return;
        }

        case State::DEQUEUE:
        {
            NavEntry e{};
            if (!_pop(e))
            {
                _state = State::IDLE;
                return;
            }
            _destinationNode        = e.nodeId;
            _activeRouteDirection   = e.route_direction;
            _activeArrivalDirection = e.arrival_direction;
            _activeStopAtLine       = e.stop_at_line;
            // PR2: re-arm the wall-zero reset latch for the fresh goal so a reset node
            // re-visited by a later goTo(...) fires its wall push again.
            _resetNode              = FieldMap::NODE_NONE;
            // PR2 (#3): the wall-zero interlude fires only on a post-hop arrival. Mark this
            // goal's FIRST plan so `_onPlanNextHop` skips the interlude when no hop has run yet
            // (a START reset node, or a zero-hop goTo(R) while already on R). Cleared on the
            // first plan below. Matches the sim, which only checks for a reset node after a hop.
            _planIsFirstAfterDequeue = true;
            // ADR-036: each goal starts with a clean pivot latch. A pivot whose
            // FINAL hop was the previous goal's last hop left `_pendingPivot`
            // active with no following hop to consume it — clear all three here
            // so a pivot never fires across a `goTo(...)` boundary and a stop at
            // the pivot node degrades to a normal turn.
            _resetPivotLatch();
            // ADR-059: defensive — a blind-chain run retires at its own final-hop
            // dispatch and its walk never extends past the goal, so no run can be
            // live here; clear anyway so a run could never span a goTo() boundary
            // (mirrors the pivot-latch discipline above).
            _chainReset();

            // Return-to-node prologue (issue #36, PR1b). Evaluate the accumulated
            // operator offset BEFORE planning: a non-trivial offset drives the
            // chassis back onto the anchored node first (RETURN_TO_NODE), an
            // over-cap offset honors the off-map gate, and an offset≈0 falls
            // through to PLAN_NEXT_HOP unchanged (AC5 — a true no-op). The helper
            // sets `_state` to one of {RETURN_TO_NODE, PLAN_NEXT_HOP, FAULT}.
            const Offset2D off = Localization.getOffsetFromNode();
            const float    mag = sqrtf(off.x * off.x + off.y * off.y);
            _evaluateReturnGate(off, mag);
            return;
        }

        case State::RETURN_TO_NODE:
        {
            // The return move was dispatched `.async()` in `_dispatchReturnMove`.
            // This state is EXEMPT from the generic Drive-fault guard at the top of
            // `_stepFsm()` so a LINE re-acquire's `LINE_NOT_FOUND` can route to the
            // off-map gate (Fault::OFF_NODE) instead of being mislabeled DRIVE_FAULT.
            // The trade-off: this state body must inspect the Drive fault itself for
            // BOTH node kinds — a blind dead-reckon drive that faults for any reason
            // must NOT be silently accepted as a successful return.
            if (!Drive.isIdle())
            {
                return;    // seek / dead-reckon drive still in flight
            }

            // The return move (and its shared pre-turn) has settled — the prologue is no
            // longer in flight regardless of the outcome below. Clear the routing flag so a
            // later TURNING tick (the next goal's normal hop) is never mis-routed to the
            // return dispatch.
            _returnActive = false;

            if (_returnUsesLineSeek)
            {
                // Line-seek return: the `Drive.stopAtLine` seek either re-acquired
                // the line (axle now on the node — a true re-localisation) or ran the
                // full max distance without a crossing and latched
                // `Fault::LINE_NOT_FOUND` (stop + clear queue, so `isIdle()` is now
                // true). A LINE_NOT_FOUND routes to the off-map gate (NOT DRIVE_FAULT)
                // per the plan: under FAULT raise `Fault::OFF_NODE`; under BEST_EFFORT
                // log and accept the dead-reckoned position (the chassis drove back to
                // ~the node — re-anchor anyway via `clearOffset()` so the goal can
                // still plan).
                if (Drive.hasFault())
                {
                    // The `stopAtLine` seek's EXPECTED fault is `LINE_NOT_FOUND` (no crossing
                    // within the max distance), which routes to the off-map gate. Discriminate
                    // it explicitly (#3): any OTHER Drive fault here (e.g. a future
                    // `DISPATCH_FAILED`) is a genuine hardware/dispatch failure, NOT an off-map
                    // condition — surface it as `DRIVE_FAULT` instead of mislabeling it
                    // `OFF_NODE` (FAULT gate) or silently accepting a never-completed return
                    // (BEST_EFFORT). `Drive.getFault().kind` (not `isLineFound()`) is the right
                    // probe: `isLineFound()` is false for ANY fault and cannot tell the two
                    // apart. (Drive's no-line outcome agrees: `isLineFound() == false`.)
                    if (Drive.getFault().kind != DriveClass::Fault::LINE_NOT_FOUND)
                    {
                        Drive.stop(true);
                        _enterFault(Fault::DRIVE_FAULT, Localization.getCurrentNode(),
                                    _destinationNode);
                        return;
                    }

                    if (Config::OFF_MAP_GATE == Config::OffMapGate::FAULT)
                    {
                        Drive.stop(true);
                        _enterFault(Fault::OFF_NODE, Localization.getCurrentNode(),
                                    _destinationNode);
                        return;
                    }
                    // BEST_EFFORT: clear the latched Drive fault so the next hop's
                    // dispatch is not immediately tripped by the generic guard, accept
                    // the dead-reckoned pose (re-anchor on the SAME node), and plan.
                    Serial.println(
                        F("[PathTracker] return-to-node: LINE_NOT_FOUND, BEST_EFFORT "
                          "accepting dead-reckoned position"));
                    Drive.clearFault();
                    Localization.clearOffset();
                    _state = State::PLAN_NEXT_HOP;
                    return;
                }

                // Success: the seek detected the crossing line and rolled the axle
                // onto it. Re-anchor the SAME node (we returned TO it, not advanced)
                // via `onIntersection(...)` — a verified physical re-localisation,
                // which also clears the operator offset (Localization side).
                Localization.onIntersection(Localization.getCurrentNode());
                _state = State::PLAN_NEXT_HOP;
                return;
            }

            // Dead-reckon return (a blind-only anchor OR a short LINE return, #14): the return was
            // a dead-reckoned turn + drive (no line seek). A `Drive.hasFault()` here is a genuine
            // drive failure (not a LINE_NOT_FOUND — neither `turnToYaw` nor `driveDist` seeks a line), so it
            // is a real DRIVE_FAULT, NOT the off-map gate. Routing it through
            // `_enterFault(DRIVE_FAULT, ...)` here is necessary because the generic
            // guard at the top is suppressed for this state; without it the blind
            // success path below would silently accept a faulted (incomplete) return.
            if (Drive.hasFault())
            {
                Drive.stop(true);
                _enterFault(Fault::DRIVE_FAULT, Localization.getCurrentNode(),
                            _destinationNode);
                return;
            }

            // Success: the dead-reckoned inverse drive IS the re-anchor. `clearOffset()`
            // zeroes the offset and re-snapshots the integrator baselines without
            // changing the node id (we returned to the same node). ADR-001: a blind
            // node's own position is only dead-reckoned, so a dead-reckoned return is no
            // worse than how the robot reached it; the distance cap bounds the error.
            Localization.clearOffset();
            _state = State::PLAN_NEXT_HOP;
            return;
        }

        case State::PLAN_NEXT_HOP:
        {
            _onPlanNextHop();
            return;
        }

        case State::TURNING:
        {
            if (Drive.isIdle())
            {
                // The shared pre-edge turn just finished. Route the post-turn dispatch by
                // context: a return prologue (`_returnActive`) dispatches the return
                // seek/drive; a normal hop dispatches its edge. The return reuses this SAME
                // state and the SAME pre-turn, so a return's pre-edge turn can never be
                // skipped (issue #36, #6 refactor — the structural fix for the
                // LINE-return-without-turn bug).
                if (_returnActive)
                {
                    // Return-prologue's own pre-edge turn never settles (dwell-2 is scoped to
                    // normal hops; the return interlude owns its own timing). Dispatch the
                    // return seek/drive directly, as today.
                    _dispatchReturnMoveAfterTurn();
                }
                else if (Config::HOP_SETTLE_MS > 0)
                {
                    // ADR-061 dwell-2: a cold pre-edge turn on a NORMAL hop just braked to
                    // rest. Settle before dispatching the move (the turn->move dwell). NO
                    // RESET check here — this is a turn->move dwell, not an arrival dwell —
                    // and NOT gated on `_planIsFirstAfterDequeue` (that flag was already
                    // cleared in `_onPlanNextHop` before TURNING ran, so the first hop's cold
                    // turn settles like any other). `_dispatchEdgeAfterTurn()` runs when the
                    // dwell elapses (SettleResume::EDGE_AFTER_TURN).
                    _enterSettle(SettleResume::EDGE_AFTER_TURN);
                }
                else
                {
                    _dispatchEdgeAfterTurn();
                }
            }
            return;
        }

        case State::LINE_FOLLOW:
        {
            // -----------------------------------------------------------------
            // #146 — bounded lost-line fault ESCALATION (NOT the deferred ADR-009
            // sweep/reacquire/resume recovery). Poll `LineFollower.isLineLost()`
            // ONLY here in `LINE_FOLLOW`: `LINE_OVERSHOOT` is a deliberate
            // dead-reckon PAST the line (its overshoot-mode arm expects the sensor
            // to go offline as the line ends at the node), so polling it would be
            // incoherent. Suppress the poll during a PathTracker-owned pass-through
            // dead-reckon window (`_passThrough` / `_projPassThrough`) — those are
            // the FSM's OWN flags, so the escalation never reaches into
            // LineFollower's private `_overshootMode` (cross-lane). A loss that
            // persists for `Config::LINE_LOST_FAULT_MS` of CONTINUOUS
            // line-following outside such a window means the chassis wandered off
            // the line; escalate to `Fault::LINE_LOST`. Any reacquire (or a
            // suppressed tick) resets the timer, so only a genuine sustained
            // departure faults; the timer is also cleared on state-exit (below),
            // abort, and fault.
            if (_passThrough || _projPassThrough || !LineFollower.isLineLost())
            {
                _lineLostTiming = false;
            }
            else if (!_lineLostTiming)
            {
                _lineLostTiming  = true;
                _lineLostSinceMs = millis();
            }
            else if ((uint32_t) (millis() - _lineLostSinceMs) >= Config::LINE_LOST_FAULT_MS)
            {
                // Read `_destinationNode` as the fault's `to` BEFORE `_enterFault`
                // clears it (the argument is evaluated first). Mirrors the
                // DRIVE_FAULT dispatch pattern.
                _enterFault(Fault::LINE_LOST, Localization.getCurrentNode(), _destinationNode);
                return;
            }

            // On intersection-detect: do NOT advance Localization yet. The line
            // sensor sits `Config::LINE_SENSOR_AXLE_OFFSET_CM` ahead of the wheel
            // axle, so the chassis center is still that distance short of the
            // junction node. Re-arm the LineFollower with a distance gate equal
            // to that offset and wait for `isDistanceReached()` in LINE_OVERSHOOT
            // before calling `Localization.onIntersection(...)`.
            //
            // The controller is already enabled; re-issuing `setEnabled(true)`
            // is the cleanest way to arm the distance target via the builder
            // chain (it does NOT reset preset / direction / `_lastErr`).
            if (Intersection.isAtIntersection())
            {
                // -----------------------------------------------------------------
                // Pass-through decision (F3 — evaluate-once). The collinear
                // `line(A)–line(B)–line(C)` peek was HOISTED to dispatch
                // (`_dispatchEdgeAfterTurn`'s `LINE_FOLLOW_INTERSECTION` arm) and
                // latched into `_lineFollowPassThrough`, because the anticipatory
                // approach ramp is armed at dispatch and must know THERE whether this
                // hop terminates at `_nextHop` (arm the ramp) or passes collinearly
                // through it (keep full cruise). The peek is pure over headings/geometry
                // and the FSM never re-plans between dispatch and this detect tick, so
                // consuming the latch here is behaviour-identical to the old recompute.
                // Consume it into `_passThrough`, which `LINE_OVERSHOOT` and the next
                // dispatch read (brakeOnGate / tShapeRampDown / start-speed). See
                // ADR-023 and issue #19.
                // -----------------------------------------------------------------
                _passThrough = _lineFollowPassThrough;

                // Arm the overshoot. Always set overshootMode=true (project-wide rule
                // per ADR-023: the 5cm window dead-reckons through line-offline
                // conditions). brakeOnGate and tShapeRampDown depend on _passThrough.
                //
                // `.setReverse(...)` is chained explicitly (PR #34 review). The
                // previous code relied on LineFollower's sticky _reverse state —
                // `setEnabled(true)` without `setReverse` left whatever was last
                // configured. That works today, but a future change to
                // LineFollower::_applyEnableBuilder that reset _reverse on
                // re-enable would silently break the overshoot window on REAR
                // LINE hops (overshoot dead-reckons forward instead of reverse).
                // Making the direction explicit at every re-arm point removes
                // the implicit coupling.
                const bool overshootReverse = (_resolvedDirection == Direction::REAR);
                // Dead-reckon with the axle offset of the board that LED this hop. A REAR
                // hop tracked with the front board's offset lands off-node and feeds that
                // error into the next dead-reckoned hop, so a reverse overshoot uses the
                // rear board's own offset. Single-sourced through the shared policy
                // (ADR-071, #54) so this LINE-overshoot budget cannot drift from the seek
                // budget, the offline validator, or the sim.
                const float overshootAxleOffsetCm =
                    pivotOvershootBudgetCm(_resolvedDirection);

                // ----------------------------------------------------------------
                // ADR-036 — LINE-incoming pivot trim (per-turn compensation).
                //
                // This LINE hop drives INTO `_nextHop`. If `_nextHop` is a pivot
                // node, the approaching pivot was resolved at THIS hop's plan-time
                // dispatch (`_onPlanNextHop`'s one-hop lookahead) into
                // `_approachingPivot`, which survives into this LINE_FOLLOW state.
                //
                // A LINE edge re-anchors on the junction line and cannot stop
                // short, so instead of an approach-distance trim we shorten the
                // overshoot advance by δ — the axle settles δ short of the node.
                // The OUTGOING hop's dead-reckon trim is NOT latched here: the
                // one-hop carry is already owned by `_pendingPivot` → next hop's
                // `_departingPivot`. This is the LINE row of the regime matrix.
                //
                // The validator guarantees δ ≤ the overshoot budget (the FRONT
                // sensor-to-axle offset; pivots are FRONT-only in v1), so the
                // reduced target stays positive; a non-positive result means a
                // config/validator mismatch, so clamp defensively to a tiny
                // positive advance rather than command a zero/negative distance
                // into the gate.
                const float inboundPivotTrim =
                    _approachingPivot.active ? _approachingPivot.trim_cm : 0.0f;
                const SensorOvershootResolution lineOvershoot =
                    resolveSensorOvershootTrim(
                        overshootAxleOffsetCm,
                        inboundPivotTrim,
                        Config::PIVOT_TRIM_MIN_DRIVE_CM);
                if (lineOvershoot.clamped)
                {
                    // A LINE-incoming pivot's δ is ALWAYS validator-visible (no
                    // call-site escape, unlike the seek), so the budget check
                    // (validator.py "sensor/LINE" regime) already rejects δ>offset
                    // here — this clamp is defensive only (a table built without
                    // the validator). Clamp + warn rather than fault.
                    _printOvershootTrimUnderflow(F("ADR-036"), overshootAxleOffsetCm,
                                                 inboundPivotTrim);
                    Serial.println(F("clamping. Check validator/config."));
                }
                const float overshootTargetCm = lineOvershoot.overshootCm;

                LineFollower.setEnabled(true)
                            .setTShapeRampUp(false)
                            .setTShapeRampDown(!_passThrough)
                            .setOvershootMode(true)
                            .setReverse(overshootReverse)
                            .setBrakeOnGate(!_passThrough)
                            .setDistanceTarget(overshootTargetCm);
#ifdef PT_PARITY_TRACE
                _ptEmitLineOvershoot(overshootTargetCm, _passThrough);
#endif
                // #146: leaving LINE_FOLLOW (state-exit) — clear the lost-line timer so a
                // later re-entry cannot inherit stale accrued loss time. LINE_OVERSHOOT is a
                // deliberate dead-reckon and is not polled.
                _lineLostTiming = false;
                _state          = State::LINE_OVERSHOOT;
            }
            else if (LineFollower.approachPhase() == ApproachPhase::OVERRUN)
            {
                // AC6 — bounded-overrun fault. The LineFollower junction hop had NO
                // late-line bound before this feature (an unbounded follow). The
                // anticipatory approach ramp advanced at the kinetic floor past
                // `predicted + Config::LINE_APPROACH_MAX_MARGIN_CM` without the junction
                // marker ever classifying — a missed line is a hard fault (ADR-039), not
                // an unbounded crawl. Enter the SAME Drive-fault path the seek/BLIND miss
                // uses: `_enterFault` disables the follower and `Drive.stop(true)` halts
                // the chassis. `isAtIntersection()` above stays the authoritative arrival
                // (it re-anchors Localization) and wins in the same tick — the plain
                // overshoot re-arm it dispatches carries no `setApproachTarget`, so the
                // schedule is disarmed and `approachPhase()` can never read OVERRUN once
                // detection has fired. Note (S3): a target marker that is simply ABSENT
                // drives `!isOnline()` → `LOST_LINE_ACTION` inside the follower first;
                // this OVERRUN bound specifically guards the "on a line but the marker
                // never classifies past the overrun limit" case.
                _enterFault(Fault::DRIVE_FAULT, Localization.getCurrentNode(),
                            _destinationNode);
            }
            return;
        }

        case State::LINE_OVERSHOOT:
        {
            // Wait until the chassis has rolled forward by LINE_SENSOR_AXLE_OFFSET_CM
            // past the intersection-detect point. `isDistanceReached()` is read-and-clear;
            // the LineFollower's internal gate handling differs based on `_passThrough`:
            //
            //   - Stop-and-go (_passThrough == false): LineFollower's update() armed
            //     `_brakeOnGate = true` (default), so when the gate trips it self-disables
            //     via _applyEnableBuilder with active brake. By the time we observe
            //     isDistanceReached() here, the chassis is already braked and the
            //     controller is disabled. Same as pre-issue-19 behavior.
            //
            //   - Pass-through (_passThrough == true): LineFollower's update() armed
            //     `_brakeOnGate = false`, so the gate trip latches `_distanceReached`
            //     without disabling the controller. The chassis continues at cruise PWM
            //     under LineFollower's symmetric output. _dispatchEdgeAfterTurn (T6/T7)
            //     consumes _passThrough to decide whether to keep LineFollower live (LINE
            //     next), re-arm for a new distance target (PROJECTION next), or release
            //     with no brake before handing off to Drive.driveDist (BLIND next).
            //
            // Either way, advance Localization to the intersected node. _passThrough is
            // consumed by the upcoming PLAN_NEXT_HOP → _dispatchEdgeAfterTurn dispatch.
            //
            // See ADR-023 and issue #19.
            if (LineFollower.isDistanceReached())
            {
                Localization.onIntersection(_nextHop);
                // ADR-061 dwell-1: a `_passThrough` arrival is a NON-braked continuing
                // pass-through (LineFollower stays live at cruise) — no settle, plan straight
                // through. A non-pass arrival braked to rest — settle before the next maneuver
                // (subject to the enabled + non-RESET gate). Order is load-bearing: the
                // `onIntersection` stamp above ran FIRST.
                if (_passThrough)
                {
                    _state = State::PLAN_NEXT_HOP;
                }
                else
                {
                    _settleOrPlanAfterArrival(_nextHop);
                }
            }
            return;
        }

        case State::PROJECTION_FOLLOW:
        {
            // A LINE edge into a projection DESTINATION node (issue #64): the
            // projection node sits on the line but at NO physical intersection
            // marker, so the line-follow is DISTANCE-terminated — the runtime stops
            // when the encoder distance gate (armed with the pivot-trimmed edge
            // distance in `_dispatchEdgeAfterTurn`) trips, NOT when an intersection
            // is detected. There is no junction to re-anchor against, so advance the
            // node by dead-reckoning (`onDeadReckonedAdvance`), mirroring the sim's
            // `follow_line_to_distance` + `on_dead_reckoned_advance` (T5).
            //
            // THE PASS-THROUGH TRIGGER IS THIS DISTANCE GATE (T19, issue #64).
            // For a `line(A)–projection(B)–line(C)` triple routed PAST B, the A→B
            // hop runs THIS state (projection node B has no junction marker, so
            // `Intersection.isAtIntersection()` never fires at B — it only fires at
            // the real junction C). The pass-through MUST therefore fire off this
            // DISTANCE gate, NOT the intersection gate of the `LINE_FOLLOW` state.
            // Eligibility was decided + latched at arm time (`_projPassThrough`) in
            // `_dispatchEdgeAfterTurn`, which also armed the gate with
            // `brakeOnGate == false` so the trip does NOT self-disable/brake the
            // follower — the A→C follow stays continuous across B.
            if (LineFollower.isDistanceReached())
            {
                if (_projPassThrough)
                {
                    // PASS-THROUGH: do NOT stop and do NOT re-enable the follower (a
                    // fresh enable would re-seed the tShape ramp / deadband-snap the
                    // PWM floor and lurch). The follower is still running (the gate
                    // was armed brake-on-gate=false). Stamp the node advance at B
                    // MID-FOLLOW — a dead-reckon advance (no junction re-anchor here;
                    // C re-anchors), preserving the one-advance-per-node invariant —
                    // then continue line-following the B→C hop. Set `_passThrough` so
                    // the upcoming PLAN_NEXT_HOP → `_dispatchEdgeAfterTurn` LINE→line
                    // branch hits the already-enabled LineFollower fast-path (no
                    // ramp-up reseed) and cruise PWM is preserved across B. The B→C
                    // pre-edge turn is a no-op because A→B→C are collinear (the gate
                    // guaranteed it), so `_dispatchTurnIfNeeded` returns false and
                    // never disables the follower. At C, `LINE_FOLLOW` +
                    // `Intersection.isAtIntersection()` re-anchors normally. Mirrors
                    // the sim's distance-gated mid-follow advance (T20).
                    Localization.onDeadReckonedAdvance(_nextHop);
                    _projPassThrough = false;    // consumed (re-evaluated per hop)
                    _passThrough     = true;     // keep the follower live into B→C
                    _state           = State::PLAN_NEXT_HOP;
                    return;
                }

                // Normal per-hop path: stop at the projection node B. Disable the
                // follower (active brake), dead-reckon the advance, then plan the
                // next hop (or finish if B is the goal). This is the fallback for a
                // `goTo(B)` and for any triple that failed the pass-through gate
                // (BLIND flank, non-collinear, projection/blind C, …).
                LineFollower.setEnabled(false);
                Localization.onDeadReckonedAdvance(_nextHop);
                _projPassThrough = false;    // consumed (defensive; already false here)
                // ADR-061 dwell-1: this branch braked to rest at B — settle before the next
                // maneuver (subject to the enabled + non-RESET gate). The pass-through branch
                // above is a non-braked continue and never reaches here. Order is load-bearing:
                // the `onDeadReckonedAdvance` stamp above ran FIRST.
                _settleOrPlanAfterArrival(_nextHop);
            }
            return;
        }

        case State::BLIND_FORWARD:
        {
            if (Drive.isIdle())
            {
                // ADR-059: a still-ACTIVE chain run here means the hop that just retired
                // was a NON-final chained hop (the final hop retires the run at its own
                // dispatch), i.e. this node boundary is a non-braking pass — the chained
                // move ended at its planned endPwm with `setBrake(false)` and the next
                // chained hop re-drives the wheels at that same PWM one plan tick later.
                // Emit the `blind_pass` parity record BEFORE the advance stamp so the
                // record sits between the two hops' blind_dead_reckon records (the sim
                // mirrors this ordering). The node is still stamped exactly ONCE via the
                // unchanged onDeadReckonedAdvance below (AC3) — blind nodes have no
                // positional re-anchor, chained or not.
                //
                // ADR-061: capture the chain-active flag OUTSIDE the trace block (the
                // `_chain.active` read below drives the settle decision, not just the trace).
                // A still-active chain is a non-final chained pass (no brake) -> NO settle;
                // a non-chain hop (or the final chained hop, whose dispatch already retired
                // the run) braked to rest -> settle.
                const bool chainActive = _chain.active;
#ifdef PT_PARITY_TRACE
                if (chainActive)
                {
                    _ptEmitBlindPass(_nextHop);
                }
#endif
                Localization.onDeadReckonedAdvance(_nextHop);
                // Order is load-bearing: the `onDeadReckonedAdvance` stamp above ran FIRST.
                if (chainActive)
                {
                    _state = State::PLAN_NEXT_HOP;
                }
                else
                {
                    _settleOrPlanAfterArrival(_nextHop);
                }
            }
            return;
        }

        case State::STOP_AT_LINE:
        {
            // Drive.stopAtLine(...) seek dispatched in `_dispatchEdgeAfterTurn`.
            //
            // No-line / max-distance fault: when the seek runs the full max
            // distance without a crossing, Drive latches `Fault::LINE_NOT_FOUND`
            // and `Drive.hasFault()` goes true. That is caught by the shared
            // Drive-fault guard at the TOP of update() (same as BLIND_FORWARD),
            // which stops the chassis and enters `Fault::DRIVE_FAULT` — so this
            // state body only has to handle the success path. No dead-reckon
            // fallback (ADR-027 / AC7).
            //
            // Success: the seek detected the crossing line, rolled the axle onto
            // it, and active-braked — a true physical re-localisation event. Unlike
            // BLIND_FORWARD (which dead-reckons via `onDeadReckonedAdvance`), advance
            // the node with `onIntersection(...)` so this hop RE-ANCHORS the discrete
            // position on the real line (the "better localization" of issue #27).
            //
            // Issue #64: when `_nextHop` is a projection node, this is the
            // BLIND→projection auto-seek's FULL-ADVANCE re-anchor. `onIntersection`
            // sets the current node to the projection node and clears the operator
            // offset, treating the seek-stop as the projection node's mapped position
            // (there is no junction marker; the seek lands on the projection node's
            // own line). Same call, no special branch — the auto-rule that routed the
            // hop here lives in `_dispatchEdgeAfterTurn`.
            if (Drive.isIdle())
            {
                Localization.onIntersection(_nextHop);
                // ADR-061 dwell-1: a STOP_AT_LINE seek always brakes to rest — settle before
                // the next maneuver (subject to the enabled + non-RESET gate). Order is
                // load-bearing: the `onIntersection` stamp above ran FIRST.
                _settleOrPlanAfterArrival(_nextHop);
            }
            return;
        }

        case State::WALL_STOP:
        {
            // Wall-zero reset interlude, phase 1 (PR2, AC9). The `Drive.wallStop(...)`
            // push was dispatched `.async()` by `_dispatchWallStop` when this state was
            // entered (latched on `_resetNode`, so it fires exactly once per arrival).
            // The push is stall/timeout-terminated (T14): it ends idle on EITHER a
            // confirmed wall stall (back-off + brake) OR the non-latching advisory
            // timeout. A wall-stop timeout does NOT set `Drive.hasFault()`, so the
            // generic Drive-fault guard at the top of `_stepFsm()` does NOT fire here —
            // the advisory is inspected in `REANCHOR` (AC10: assume-at-wall, continue).
            // Just wait for the push to settle, then re-square.
            if (Drive.isIdle())
            {
                _state = State::REANCHOR;
            }
            return;
        }

        case State::REANCHOR:
        {
            // Wall-zero reset interlude, phase 2 (PR2, AC9–AC11). The chassis is now
            // pressed against (and backed a hair off) the wall. Re-anchor the world
            // pose against the wall's ground truth.
            const uint8_t cur = Localization.getCurrentNode();

            // AC10 — surface the wall-stop advisory but DO NOT halt. A timeout means the
            // push never stalled (never reached the wall); we still re-anchor and
            // continue ("assume at wall"). The advisory goes to EventTrace + Serial so
            // the operator can see it on the OLED ring; it is NOT a fault.
            if (Drive.wallStopTimedOut())
            {
                EventTrace.pushEvent("WALL no-stall, assume@wall");
                Serial.print(F("[PathTracker] WALL_STOP advisory: no stall at node "));
                Serial.print(cur);
                Serial.println(F("; assuming at wall and re-anchoring"));
            }

            // Re-read the reset record (axis / wall_coord / facing). A null here means
            // the node lost its reset record between arrival and now — defensively skip
            // the re-anchor and resume planning rather than dereference null.
            const Routes::ResetNode* info = FieldMap::getResetInfo(cur);
            if (info == nullptr)
            {
                Serial.print(F("[PathTracker] REANCHOR: node "));
                Serial.print(cur);
                Serial.println(F(" has no reset record; skipping re-anchor"));
                _state = State::PLAN_NEXT_HOP;
                return;
            }

            // AC9 — axis set + heading squared atomically inside reanchorAtWall (T15).
            const bool ok =
                Localization.reanchorAtWall(info->axis, info->wall_coord, (float) info->facing);

            // AC11 — the strict wall-normal check (only active under
            // Config::WALL_NORMAL_STRICT_ENABLED) rejected the squaring: the chassis is
            // too far off the expected wall normal to trust a re-zero. Fault instead of
            // re-anchoring against a bad pose.
            if (!ok)
            {
                EventTrace.pushEvent("WALL not square; WSQ");
                Serial.print(F("[PathTracker] REANCHOR: wall-normal check REJECTED at node "));
                Serial.print(cur);
                Serial.println(F("; raising WALL_NOT_SQUARE"));
                // `_enterFault` clears `_resetNode` (re-arm) along with the rest of the
                // in-flight goal state.
                _enterFault(Fault::WALL_NOT_SQUARE, cur, _destinationNode);
                return;
            }

#ifdef PT_PARITY_TRACE
            _ptEmitWall("wall_reanchor", cur, info->axis, "wall_coord", info->wall_coord);
#endif

            // Success: re-anchored. The chassis is still pressed against the wall (belief is the
            // axle, WALL_CONTACT_OFFSET_CM short of the wall surface), so BEFORE any planning —
            // which could dispatch a pivot that would clip the wall — drive FORWARD off the wall
            // back onto the node, giving the next hop clearance. `_resetNode == cur` stays latched
            // through WALL_DEPART so the interlude is not re-entered; it re-arms once the FSM
            // advances off this node (or on the next goal's DEQUEUE). The depart distance is the
            // node-to-wall standoff minus the contact offset (the axle is already that offset off
            // the wall). GYRO holds `_heldYaw` (the squared heading the wallStop SQUARE left set),
            // so the short forward move tracks straight away from the wall.
            const float nodeCoord = Localization.coordOnAxis(cur, info->axis);
            const float standoff  = fabsf(info->wall_coord - nodeCoord);
            const float departCm  = standoff - Config::WALL_CONTACT_OFFSET_CM;
            if (departCm > 0.0f)
            {
#ifdef PT_PARITY_TRACE
                _ptEmitWall("wall_depart", cur, info->axis, "node_coord", nodeCoord);
#endif
                Drive.driveDist(departCm).setMode(DriveMode::GYRO).async();
                _state = State::WALL_DEPART;
                return;
            }
            // Degenerate: standoff <= contact offset. The validator's clearance floor
            // (standoff >= WALL_PIVOT_CLEARANCE_CM >= WALL_CONTACT_OFFSET_CM, static_asserted) makes
            // this unreachable on a valid field; guard it anyway so a malformed table never
            // dispatches a non-positive driveDist. Nothing to drive: declare belief == node, plan.
            Localization.clearOffset();
            _state = State::PLAN_NEXT_HOP;
            return;
        }

        case State::WALL_DEPART:
        {
            // Wall-zero reset interlude, phase 3 (return-to-node). The `Drive.driveDist(...)`
            // dispatched on REANCHOR success drives the chassis FORWARD off the wall, back onto the
            // node (the `standoff - WALL_CONTACT_OFFSET_CM` the axle was short of it). It is a plain
            // bounded move, so a real Drive fault during it correctly trips the generic DRIVE_FAULT
            // guard at the top of `_stepFsm()` — this state is intentionally NOT exempt (unlike
            // RETURN_TO_NODE / the wallStop advisory). On a clean settle the axle is on the node:
            // clear the operator offset to declare belief == node, then resume the normal
            // post-arrival path. `_resetNode == cur` stays latched, so PLAN_NEXT_HOP does not
            // re-enter the interlude. A blind-only reset node has no line to re-acquire — this is a
            // pure dead-reckon return onto a known-distance target.
            if (!Drive.isIdle())
            {
                return;    // depart drive still in flight
            }
            Localization.clearOffset();
            _state = State::PLAN_NEXT_HOP;
            return;
        }

        case State::SETTLE:
        {
            // ADR-061: non-blocking post-stop dwell. Drive is idle/braked and NO Drive command
            // is issued here — the state is self-bounded by `_settleDeadlineMs`, so `update()`
            // returns each tick and the deadline compare resolves it. Rollover-safe compare
            // (matches the `_drainBlocking` deadline idiom): `(int32_t)(millis() - deadline)`
            // is non-negative once the deadline is reached, even across a `millis()` wrap.
            if ((int32_t) (millis() - _settleDeadlineMs) >= 0)
            {
                if (_settleResume == SettleResume::PLAN)
                {
                    _state = State::PLAN_NEXT_HOP;
                }
                else    // EDGE_AFTER_TURN
                {
                    _dispatchEdgeAfterTurn();
                }
            }
            return;
        }

        case State::FAULT:
        {
            return;
        }
    }
}

// =========================================================================================
// FSM helpers
// =========================================================================================

void PathTrackerClass::_enterFault(Fault kind, uint8_t from, uint8_t to)
{
    _fault.kind     = kind;
    _fault.fromNode = from;
    _fault.destNode = to;

    // Disable LineFollower BEFORE Drive.stop(true). If the follower is left
    // enabled (e.g., fault fired during LINE_FOLLOW or LINE_OVERSHOOT with
    // brakeOnGate=false), its next update() tick re-issues Drive.setSpeed()
    // and undoes the halt — making the fault non-fail-safe in exactly the
    // scenario it is meant to protect. Codex P1 (PR #34 review).
    LineFollower.setEnabled(false);
    Drive.stop(true);

    _clearQueue();
    _state                  = State::FAULT;
    _destinationNode        = FieldMap::NODE_NONE;
    _nextHop                = FieldMap::NODE_NONE;
    _activeEdgeType         = Routes::EdgeType::NONE;
    _activeRouteDirection   = Direction::AUTO;
    _activeArrivalDirection = Direction::AUTO;
    // A fault disables the follower above; drop all pass-through eligibility flags
    // so a latched value cannot leak into a post-clearFault() goal's first dispatch.
    _passThrough            = false;
    _projPassThrough        = false;
    _lineFollowPassThrough  = false;
    _returnUsesLineSeek     = false;
    _returnActive           = false;
    _returnDistCm           = 0.0f;
    // #146: a fault aborts the in-flight goal — drop any in-flight lost-line escalation window so
    // its accrued loss time cannot bleed into a post-clearFault() goal.
    _lineLostTiming         = false;
    _lineLostSinceMs        = 0;
    // ADR-061: a fault aborts the in-flight goal — drop any in-flight settle dwell so it
    // cannot bleed into a post-clearFault() goal.
    _settleDeadlineMs       = 0;
    _settleResume           = SettleResume::PLAN;
    // PR2: a fault aborts the in-flight goal — drop any in-flight wall-zero reset
    // interlude so it cannot bleed into a post-clearFault() goal.
    _resetNode              = FieldMap::NODE_NONE;
    // ADR-036: a fault aborts the in-flight goal — drop any latched pivot so it
    // cannot bleed into a post-clearFault() goal.
    _resetPivotLatch();
    // ADR-059: a fault stops the chassis — drop any in-flight blind-chain run so a stale
    // speed plan cannot bleed into a post-clearFault() goal's first blind dispatch.
    _chainReset();

    Serial.print(F("[PathTracker] FAULT "));
    Serial.print(faultMnemonic(kind));
    Serial.print(F(" from="));
    Serial.print(from);
    Serial.print(F(" to="));
    Serial.println(to);
}

void PathTrackerClass::_evaluateReturnGate(const Offset2D& off, float mag)
{
    // Pure gate decision shared with the host test (ReturnToNodePolicy.h). The
    // position offset governs the trigger/cap (the translation magnitude, NOT the
    // heading delta — see the Config docstrings), so `mag` is the only metric fed in.
    const ReturnAction action = resolveReturnAction(
        mag,
        Config::RETURN_TRIGGER_THRESHOLD_CM,
        Config::RETURN_DISTANCE_CAP_CM,
        Config::OFF_MAP_GATE);

    switch (action)
    {
        case ReturnAction::NONE:
        {
            // AC5: offset within trigger — behave exactly as today. No Drive
            // dispatch, no return move; plan the goal directly.
            _state = State::PLAN_NEXT_HOP;
            return;
        }

        case ReturnAction::OFF_MAP_FAULT:
        {
            // AC6: offset over cap, FAULT gate — refuse to plan. `_enterFault`
            // stops the chassis, clears the queue, and latches the diagnostic.
            _printReturnOverCapPrefix(mag);
            Serial.println(F("FAULT, refusing to plan"));
            _enterFault(Fault::OFF_NODE, Localization.getCurrentNode(), _destinationNode);
            return;
        }

        case ReturnAction::OFF_MAP_PROCEED:
        {
            // AC6: offset over cap, BEST_EFFORT gate — log and proceed with the
            // return anyway, treating it as a capped best-effort drive
            // (`_dispatchReturnMove` clamps the distance to the cap).
            _printReturnOverCapPrefix(mag);
            Serial.println(F("BEST_EFFORT, returning anyway (capped)"));
            _dispatchReturnMove(off, mag);
            return;
        }

        case ReturnAction::RETURN:
        {
            // AC4: trigger < offset ≤ cap — drive the inverse return onto the node.
            _dispatchReturnMove(off, mag);
            return;
        }
    }
}

void PathTrackerClass::_dispatchReturnMove(const Offset2D& off, float mag)
{
    const uint8_t cur = Localization.getCurrentNode();

    // Return-travel world heading: the direction from the current (hand-pushed) pose back to
    // the anchor, i.e. the heading of the inverse offset vector (-off.x, -off.y). World frame
    // is CCW-positive with 0° = +x (East), exactly atan2's convention, so atan2(-off.y, -off.x)
    // (degrees) gives the heading. The magnitude is > trigger (>= a few cm by Config), so the
    // vector is well-defined and atan2 never sees (0, 0). Use the Arduino `PI` macro (defined by
    // Arduino.h and the host mock), NOT M_PI which the host test mock does not provide —
    // matching the WROMath.h / Localization.cpp convention. The raw atan2 result is fed straight
    // to the shared turn / `toImuYaw`, which normalizes its own argument (Localization.cpp), so
    // no pre-normalization is needed here (#10) — only the log line below wants a positive value.
    const float returnHeadingDeg = atan2f(-off.y, -off.x) * (180.0f / (float) PI);

    // Clamp the drive distance to the trust cap so a BEST_EFFORT over-cap return (mag > cap)
    // cannot command an unbounded seek/drive; an in-trust return (mag ≤ cap) is unaffected.
    _returnDistCm =
        (mag > Config::RETURN_DISTANCE_CAP_CM) ? Config::RETURN_DISTANCE_CAP_CM : mag;

    // Normalize into [0, 360) for the LOG ONLY — the functional consumers (`_dispatchTurnIfNeeded`
    // and `_dispatchReturnMoveAfterTurn`) call `toImuYaw`, which normalizes internally.
    float returnHeadingLog = fmodf(returnHeadingDeg, 360.0f);
    if (returnHeadingLog < 0.0f)
    {
        returnHeadingLog += 360.0f;
    }
    Serial.print(F("[PathTracker] return-to-node: node="));
    Serial.print(cur);
    Serial.print(F(" dist="));
    Serial.print(_returnDistCm);
    Serial.print(F(" cm headingWorld="));
    Serial.println(returnHeadingLog);

    // Latch the in-flight return context for the shared TURNING / RETURN_TO_NODE path. The
    // return always drives FORWARD toward the anchor facing the return heading, so the resolved
    // direction is FRONT (the REVERSE_UNAVAILABLE guard is irrelevant) and the resolved body
    // heading IS the return heading.
    // A LINE anchor is re-acquired by driving FORWARD until the FRONT line sensor re-crosses the
    // node line. That sensor leads the wheel axle by `LINE_SENSOR_AXLE_OFFSET_CM`, so once the
    // return distance (the axle's distance back to the node) is shorter than that lead, the sensor
    // already sits PAST the node line at the start of the seek and a forward drive can never
    // re-cross it — the seek would just run to its max and fault `LINE_NOT_FOUND` (#14). Returns
    // shorter than the sensor lead therefore dead-reckon back exactly like a blind anchor (the
    // `RETURN_TO_NODE` blind success path re-anchors the SAME node via `clearOffset`).
    // `_returnDistCm` is the cap-clamped magnitude set just above.
    const bool nodeIsLine = (FieldMap::getNodeType(cur) == Routes::NodeType::LINE);
    _returnUsesLineSeek  = nodeIsLine && (_returnDistCm >= Config::LINE_SENSOR_AXLE_OFFSET_CM);
    _resolvedDirection   = Direction::FRONT;
    _resolvedBodyHeading = returnHeadingDeg;
    _returnActive        = true;

    // Dispatch the pre-edge turn through the SAME helper every hop uses — the structural fix
    // for the LINE-return-without-turn bug (the turn is no longer inlined per node kind, so it
    // cannot be omitted on one branch). A return is never pivot-adjacent, so the wheel is the
    // pipeline default. If a turn fires the FSM waits in TURNING and dispatches the move when it
    // completes; if the chassis is already aligned the move dispatches immediately.
    if (_dispatchTurnIfNeeded(returnHeadingDeg, _turnModeToByte(Routes::PIPELINE_CONFIG.turn)))
    {
#ifdef PT_PARITY_TRACE
        _ptEmitTurn("return_turn", returnHeadingDeg, Routes::PIPELINE_CONFIG.turn);
#endif
        _state = State::TURNING;
    }
    else
    {
        _dispatchReturnMoveAfterTurn();
    }
}

void PathTrackerClass::_dispatchReturnMoveAfterTurn()
{
#ifdef PT_PARITY_TRACE
    _ptEmitReturnMove(_returnUsesLineSeek ? "seek" : "forward", Localization.getCurrentNode(),
                      _returnDistCm, _resolvedBodyHeading);
#endif
    if (_returnUsesLineSeek)
    {
        // Line-seek return (LINE anchor, return distance >= the sensor lead). Re-acquire the
        // line: drive FORWARD toward the expected node crossing facing the (already-aligned)
        // return heading; the seek arms the T_OR_CROSS
        // detector near the endpoint, snaps on the line, and overshoots so the wheel AXLE lands
        // on the node (ADR-027) — the same primitive the STOP_AT_LINE hop uses. GYRO holds the
        // return heading so the chassis tracks straight back. `expectedCm` is the capped
        // magnitude; the seek's own STOP_AT_LINE_MAX_MARGIN_CM provides the search slack past
        // it, so the seek faults LINE_NOT_FOUND at the max distance rather than running forever.
        // RETURN_TO_NODE re-anchors the SAME node via onIntersection on success, or routes
        // LINE_NOT_FOUND to the off-map gate.
        //
        // Drain any stale Intersection edge so the return seek's own physical crossing cannot
        // leak a phantom nav edge into the next LINE hop (the forward seek drives over the
        // anchor node's line marking to re-acquire it while Intersection.update() independently
        // counts that crossing). RAII rule: one discarded-temporary .async() statement.
        (void) Intersection.isAtIntersection();

        const int16_t imuTarget = Localization.toImuYaw(_resolvedBodyHeading);
        // The return move carries no EdgeInfo, so seek at the mission-wide pipeline
        // speed (PIPELINE_CONFIG.speed → PWM) rather than the Drive default. NORMAL
        // stays byte-identical to the former default SPEED_NORMAL.
        Drive.stopAtLine(_returnDistCm)
            .setReverse(false)
            .setMode(DriveMode::GYRO)
            .setHeading(imuTarget)
            .setPower(presetToPwm(speedTierToPreset(Routes::PIPELINE_CONFIG.speed)))
            .async();
        _state = State::RETURN_TO_NODE;
        return;
    }

    // Dead-reckon return: a blind-only anchor (no line to snap onto) OR a short LINE return whose
    // front sensor already sits past the node line (#14) — both drive the inverse vector blind.
    // The chassis already faces the return heading (the shared pre-turn ran in TURNING), so this
    // is a plain forward driveDist. deadbandSnap floors any non-zero PWM to PWM_MIN_MAGNITUDE, so seed the
    // start speed (TSHAPE_START_SPEED) like the BLIND_FORWARD dispatch — do not ramp from 0.
    // SYNC + tShape + setPower + setStartSpeed mirrors the existing SYNC_TSHAPE BLIND dispatch
    // shape. RAII rule: one discarded-temporary .async() statement.
    Drive.driveDist(_returnDistCm)
        .setMode(DriveMode::SYNC)
        .setPower(presetToPwm(speedTierToPreset(Routes::PIPELINE_CONFIG.speed)))
        .setStartSpeed(Config::TSHAPE_START_SPEED)
        .tShape()
        .async();
    _state = State::RETURN_TO_NODE;
}

void PathTrackerClass::_enterSettle(SettleResume resume)
{
    // Non-blocking dwell (ADR-061). Stamp the rollover-safe deadline, latch the resume
    // target, and park in SETTLE. No Drive command is issued — Drive stays idle/braked, so
    // the dwell is self-bounded by `_settleDeadlineMs` and `update()` returns each tick.
    _settleDeadlineMs = millis() + Config::HOP_SETTLE_MS;
    _settleResume     = resume;
    _state            = State::SETTLE;
}

void PathTrackerClass::_settleOrPlanAfterArrival(uint8_t arrivedNode)
{
    // Shared dwell-1 decision for the four braked arrival states (ADR-061). The caller has
    // already advanced `Localization` for `arrivedNode` (order is load-bearing). Settle only
    // when the feature is enabled AND the arrival is not a RESET node — the wall-zero reset
    // interlude owns the dwell at a RESET boundary. Otherwise plan immediately, byte-identical
    // to the pre-feature transition.
    if (Config::HOP_SETTLE_MS > 0
        && FieldMap::getNodeType(arrivedNode) != Routes::NodeType::RESET)
    {
        _enterSettle(SettleResume::PLAN);
    }
    else
    {
        _state = State::PLAN_NEXT_HOP;
    }
}

bool PathTrackerClass::_maybeEnterReset(uint8_t cur)
{
    // Wall-zero reset hook (PR2, AC9). `_onPlanNextHop` is the single arrival
    // chokepoint: every hop's success path advances `Localization` and routes here, so
    // `cur` is the just-arrived node. If it carries a reset record and its interlude has
    // not already run for THIS arrival (latch guard), run the wall-stop interlude before
    // planning the next hop / declaring the goal reached — the arrival analogue of the
    // RETURN_TO_NODE prologue inserted at DEQUEUE.
    //
    // No change for non-reset nodes (AC5-style "no behavior change when not applicable"):
    // getNodeType returns LINE/BLIND, the first clause is false, and the caller proceeds
    // exactly as before. The `_resetNode == cur` guard makes the entry one-shot — after a
    // successful REANCHOR returns control here with `_resetNode` still set to `cur`, this
    // re-check is false, so the interlude is NOT re-entered (the wallStop push fired
    // once). It re-arms once the FSM advances off this node (or on the next DEQUEUE).
    if (FieldMap::getNodeType(cur) == Routes::NodeType::RESET && _resetNode != cur)
    {
        _dispatchWallStop(cur);
        return true;
    }
    return false;
}

void PathTrackerClass::_dispatchWallStop(uint8_t node)
{
    // Resolve the reset record for this node (axis / wall_coord / facing). The caller
    // confirmed `NodeType::RESET`, but a node can be tagged RESET yet (defensively) lack
    // a sparse `RESET_NODES` entry — if so, skip the push and re-anchor will likewise
    // no-op, so just plan on. This keeps the dispatch total even on a malformed table.
    const Routes::ResetNode* info = FieldMap::getResetInfo(node);
    if (info == nullptr)
    {
        Serial.print(F("[PathTracker] WALL_STOP: node "));
        Serial.print(node);
        Serial.println(F(" tagged RESET but has no reset record; skipping wall push"));
        // Latch it so `_maybeEnterReset` does not loop back in, then fall through to
        // planning via the caller's normal path.
        _resetNode = node;
        _state     = State::PLAN_NEXT_HOP;
        return;
    }

    // Advisory push distance: the absolute gap from the node's coordinate on `axis` to
    // the wall. The axis decode (0 = x, 1 = y — the cross-task ResetNode contract) lives in
    // `Localization::coordOnAxis`, not open-coded here. This distance is ADVISORY only —
    // `Drive.wallStop` terminates on a wheel stall or the advisory timeout (T14), never on
    // distance — but we pass a sensible value rather than 0.
    const float nodeCoord  = Localization.coordOnAxis(node, info->axis);
    const float distToWall = fabsf(info->wall_coord - nodeCoord);

    // (#1) Square-then-push, REVERSE. `wallStop` first turns in place to `facing` — its internal
    // SQUARE phase — which points the robot's FRONT away from the wall (so the REAR faces it),
    // then presses the rear STRAIGHT into the wall in RAW mode (open-loop + motor bias). Only the
    // rear of this chassis may take wall load; a front press would break the mechanism. RAW (not
    // GYRO) because once the chassis is squared a heading-hold PID would only fight the wall
    // contact as one wheel stalls first; an open-loop straight press with motor bias presses square
    // and stalls cleanly. The squaring corrects any arrival-heading error up front. `facing` is the
    // PARKED heading (front away from the wall); the node is authored so the wall is BEHIND along
    // `facing + 180`. Back-off is 0 — the post-re-anchor WALL_DEPART move provides all turn
    // clearance, so the chassis stays pressed (belief == axle) until WALL_DEPART pulls it off.
    const int16_t imuTarget = Localization.toImuYaw((float) info->facing);

    Serial.print(F("[PathTracker] WALL_STOP: node="));
    Serial.print(node);
    Serial.print(F(" axis="));
    Serial.print(info->axis);
    Serial.print(F(" wallCoord="));
    Serial.print(info->wall_coord);
    Serial.print(F(" facing="));
    Serial.print(info->facing);
    Serial.print(F(" distToWall="));
    Serial.println(distToWall);

    // Latch BEFORE dispatch so a second `_onPlanNextHop` tick cannot re-fire the push.
    _resetNode = node;

    // RAII builder rule: the whole chain is ONE discarded-temporary statement ending in
    // `.async()` — never split across statements or held in a local. `.async()` keeps
    // the FSM non-blocking; the WALL_STOP state polls `Drive.isIdle()`.
    Drive.wallStop(distToWall)
        .setReverse(true)
        .setMode(DriveMode::RAW)
        .setHeading(imuTarget)
        .setBackOff(0.0f)
        .async();
#ifdef PT_PARITY_TRACE
    _ptEmitWall("wall_stop", node, info->axis, "wall_coord", info->wall_coord);
#endif
    _state = State::WALL_STOP;
}

void PathTrackerClass::_maybeHeadingReset(uint8_t cur, bool firstPlan,
                                          float incomingBodyHeadingDeg)
{
    // Line-arrival heading reset (ADR-052). Fires only on a collinear pass-through arrival and
    // re-binds the IMU-yaw offset to the incoming LINE edge's already-known body heading, for
    // free — no wall, no maneuver. Heading-only, no motion, no fault, NO `_state` change: a
    // rejected sample or a failed IMU read soft-skips and the run continues. Full contract +
    // gating rationale in the header doxygen.

    // Mirror the wall hook's `!firstPlan` gate. `_passThrough` provably cannot be true on a
    // first plan (it is latched only on a real LINE-edge arrival), so this documents the
    // invariant rather than trusting the distant consume/clear sites forever.
    if (firstPlan)
    {
        return;
    }

    // Only a collinear pass-through arrival is eligible. A stop-and-go arrival already re-anchors
    // at the junction marker (or dead-reckons at a projection stop) through the existing hooks.
    if (!_passThrough)
    {
        return;
    }

    // Select the enabling knob + parity `pass_kind` tag by node category. Enumerate ALL FOUR
    // `Routes::NodeType`s with NO `default` so a future enum member trips `-Wswitch`. BLIND /
    // RESET are a defensive SILENT no-op: per ADR-048 no LINE edge is incident to a RESET node
    // and `_passThrough` is latched only on a LINE-edge arrival, so a pass-through into
    // BLIND/RESET is unreachable on any validator-passed field (exercised only under forced
    // state, TC11).
    bool        knob = false;
    const char* kind = nullptr;
    switch (FieldMap::getNodeType(cur))
    {
        case Routes::NodeType::PROJECTION:
            knob = Config::LINE_HEADING_RESET_ON_PROJECTION;
            kind = "proj";
            break;
        case Routes::NodeType::LINE:
            knob = Config::LINE_HEADING_RESET_ON_JUNCTION;
            kind = "junction";
            break;
        case Routes::NodeType::BLIND:
        case Routes::NodeType::RESET:
            return;    // defensive silent no-op (unreachable on a validator-passed field, ADR-048)
    }

    // Knob OFF => no IMU read, no log — this is what makes "off means off, no cost".
    if (!knob)
    {
        return;
    }

    // Seed the delta to a negative sentinel so a knob-ON skip can tell a FAILED IMU read (which
    // leaves the out-param UNTOUCHED) from a strict-gate reject (which SETS the out-param): a
    // real pre-bind delta is `fabsf(...)` and thus never negative. `reanchorHeading` returns
    // false for BOTH and does not say which — the sentinel is how we distinguish them below.
    float      delta = -1.0f;
    const bool bound = Localization.reanchorHeading(incomingBodyHeadingDeg, &delta);

    if (bound)
    {
        // FIRE: one Serial line (node / kind / bound heading / pre-bind delta) + one fixed OLED
        // push. The push is a FIXED string literal (static storage) per the wall precedent —
        // EventTrace may store the pointer, so never hand it a stack buffer.
        Serial.print(F("[PathTracker] heading reset: node "));
        Serial.print(FieldMap::getNode(cur).name);
        Serial.print(F(" ("));
        Serial.print(kind);
        Serial.print(F(") bound heading="));
        Serial.print(incomingBodyHeadingDeg);
        Serial.print(F(" preBindDelta="));
        Serial.println(delta);
        EventTrace.pushEvent("HDG reset");
#ifdef PT_PARITY_TRACE
        _ptEmitHeadingReset(cur, kind, incomingBodyHeadingDeg);    // record ONLY on a successful bind
#endif
    }
    else
    {
        // SKIP (knob was ON — knob-OFF returned above without a read): one loud Serial line so a
        // rejected/failed reset — the case the reset was most needed — is never silent. The
        // untouched-sentinel `delta < 0.0f` means the IMU read failed (no delta to report);
        // otherwise the strict gate rejected the sample and `delta` holds the offending value.
        Serial.print(F("[PathTracker] heading reset SKIP: node "));
        Serial.print(FieldMap::getNode(cur).name);
        Serial.print(F(" ("));
        Serial.print(kind);
        Serial.print(F(") "));
        if (delta < 0.0f)
        {
            Serial.println(F("IMU read failed; delta=n/a"));
        }
        else
        {
            Serial.print(F("strict-rejected (|belief - heading| > tolerance); delta="));
            Serial.println(delta);
        }
    }
}

void PathTrackerClass::_onPlanNextHop()
{
    const uint8_t cur = Localization.getCurrentNode();

    // Capture the body heading the chassis JUST arrived on for the line-arrival heading reset
    // (ADR-052), BEFORE anything below can overwrite `_resolvedBodyHeading` (the next hop
    // overwrites it at `_resolvedBodyHeading = bodyTarget;` further down). Passed explicitly
    // into `_maybeHeadingReset` so the bind target never depends on member-write ordering.
    const float incomingBody = _resolvedBodyHeading;

    // (#3) Capture-and-clear the "first plan of this goal" flag. The interlude below fires only
    // on a genuine post-hop arrival, never on the first plan (when `cur` is still the start node
    // and no hop has moved the chassis) — see `_planIsFirstAfterDequeue`.
    const bool firstPlan     = _planIsFirstAfterDequeue;
    _planIsFirstAfterDequeue = false;

    // Wall-zero reset hook (PR2, AC9): if the just-arrived node is a RESET node and its
    // interlude has not yet run, divert into the wall-stop / re-anchor interlude BEFORE
    // any planning or goal-reached decision. Returns control to this method (via
    // REANCHOR → PLAN_NEXT_HOP) once the re-anchor completes. A non-reset node is a true
    // no-op here — the FSM continues exactly as before (AC5-style). The `!firstPlan` gate
    // (#3) keeps the interlude off the zero-hop cases (START reset node / goTo(R) while on R)
    // so the firmware fires it only after a real hop, exactly as the sim does.
    if (!firstPlan && _maybeEnterReset(cur))
    {
        return;    // _state is now WALL_STOP; the interlude will route back here
    }

    // Line-arrival heading reset hook (ADR-052): on a collinear pass-through arrival, re-bind
    // heading for free to the incoming edge's known heading, bounding IMU-yaw drift. Fired here
    // — after the wall hook, before the destination check and the next hop's direction
    // resolution — so the next hop plans against the freshly-corrected heading. It is void and
    // never changes `_state`, so the FSM continues exactly as before. `incomingBody` was
    // captured at the top before any dispatch overwrites `_resolvedBodyHeading`.
    _maybeHeadingReset(cur, firstPlan, incomingBody);

    // Already at the destination — done.
    if (cur == _destinationNode)
    {
        _destinationNode = FieldMap::NODE_NONE;
        // ADR-036: the goal is complete. If its final hop departed a pivot node
        // `_pendingPivot` is still active with no following hop to consume it —
        // drop all three so a later `goTo(...)` (or the next queued goal, on the
        // DEQUEUE path) does not inherit a stale latch.
        _resetPivotLatch();
        _state           = (_qCount > 0) ? State::DEQUEUE : State::IDLE;
        return;
    }

    const uint8_t nxt = FieldMap::getNextHop(cur, _destinationNode);
    if (nxt == FieldMap::NODE_NONE)
    {
        _enterFault(Fault::UNREACHABLE, cur, _destinationNode);
        return;
    }

    const Routes::EdgeInfo& edge = FieldMap::getEdge(cur, nxt);
    if (edge.type == Routes::EdgeType::NONE)
    {
        _enterFault(Fault::UNREACHABLE, cur, nxt);
        return;
    }

    _nextHop        = nxt;
    _activeEdgeType = edge.type;

    // -----------------------------------------------------------------------
    // Per-turn pivot latch contract (ADR-036, supersedes ADR-035).
    //
    // A pivot is a property of a 3-node turn `(u, v, w)`, not of an edge. Snapshot
    // the carry into `_departingPivot` BEFORE the one-hop lookahead overwrites
    // `_pendingPivot` — the single carry member is sufficient ONLY because the
    // departing pivot is read out first. Both records survive the TURNING state
    // into the drive dispatch (and `_approachingPivot` into the LINE_FOLLOW
    // overshoot-arm, a different FSM state).
    //
    // Last hop: no pivot can latch onto the FINAL node mid-turn, so skip the
    // forward lookahead. This never skips CONSUMING an already-pending latch — a
    // pivot `(u,v,w)` whose goal is `w` still consumes its wheel + trim on the
    // final `v→w` hop via `_departingPivot`.
    // -----------------------------------------------------------------------
    const bool isLastHop = (nxt == _destinationNode);

    _departingPivot = _pendingPivot;    // snapshot the carry FIRST
    if (isLastHop)
    {
        _approachingPivot = PivotRef{};    // no pivot can latch onto the final node
    }
    else
    {
        const uint8_t nextNext = FieldMap::getNextHop(nxt, _destinationNode);
        const Routes::PivotTurn* p = FieldMap::findPivotTurn(cur, nxt, nextNext);
        _approachingPivot =
            p ? PivotRef{true, p->wheel, p->trim_cm} : PivotRef{};
    }
    _pendingPivot = _approachingPivot;    // carry for the NEXT hop

    const float currentWorld = Localization.getHeadingDeg();

    // -----------------------------------------------------------------------
    // Bidirectional direction resolution.
    //
    // Resolve per-hop precedence: per-edge override → pipeline default →
    // builder arrival override (last-hop only) → builder route override →
    // auto (turn-saving comparison against BIDIRECTIONAL_TURN_THRESHOLD_DEG,
    // gated by 2-board chassis + kill-switch). Then compute the body-target
    // heading the chassis must face: equal to edge.heading_deg for FRONT,
    // or (edge.heading_deg + 180°) mod 360 for REAR (so the rear board ends
    // up pointing along the edge direction).
    //
    // The skip-turn check below MUST compare against bodyTarget — not the raw
    // edge.heading_deg — otherwise a chassis already aligned with the raw
    // forward heading would short-circuit a REAR hop into a forward-going
    // LineFollower dispatch.
    //
    // `isLastHop` was hoisted above for the pivot latch contract.
    // -----------------------------------------------------------------------

    // Builder overrides cached on DEQUEUE for the in-flight goal (T8).
    // Both default to Direction::AUTO for bare `goTo(...)` calls — the
    // `_resolveDirection` precedence layers below per-edge / pipeline
    // overrides only kick in when the operator chained
    // `.setRouteDirection(...)` / `.setArrivalDirection(...)`.
    const Direction builderArrival = _activeArrivalDirection;
    const Direction builderRoute   = _activeRouteDirection;

    // `edge.direction_override` is a per-DIRECTED-edge value: this hop reads
    // `EDGES[cur][nxt]`. A field_data `direction:` override is baked onto the
    // forward (u->v) edge only, so a hop the planner routes the other way
    // (v->u) reads DEFAULT here and falls through to the AUTO heuristic below.
    // That is intentional — front/rear depends on the entry direction, so an
    // override authored for u->v must NOT bind v->u (mirroring it would reverse
    // the chassis when forward is correct). Author a swapped EDGES entry to
    // force the reverse sense. See graph_builder `_emit_directed_pair` warning.
    Direction resolved = resolveDirection(
        edge.direction_override,
        Routes::PIPELINE_CONFIG.direction,
        builderArrival,
        builderRoute,
        currentWorld,
        edge.heading_deg,
        isLastHop,
        Config::BIDIRECTIONAL_AUTO_ENABLED,
        Config::LINE_BOARD_COUNT);

    // -----------------------------------------------------------------------
    // Force-SAME: a pivot commits to the direction it APPROACHED in (issue #54,
    // ADR-071, supersedes the v1 force-FRONT). A single-wheel pivot is a rigid
    // 3-node maneuver, so its departure `v→w` MUST leave in the same direction
    // its approach `u→v` arrived — otherwise δ and the (possibly flipped) wheel
    // are wrong. This is the direct generalization of the old force-FRONT, which
    // forced BOTH legs FRONT.
    //
    // The APPROACH hop resolves its direction FREELY above (it may be REAR now),
    // and we STAMP that resolved direction onto the latch so the departure can
    // adopt it next hop. The DEPARTURE hop (`_departingPivot.active`) is FORCED
    // to the stored approach direction. A binding higher-precedence override on
    // the departure that truly contradicts the approach direction cannot be
    // reconciled, so it hard-faults (`PIVOT_DIRECTION_MISMATCH`) rather than run
    // a mis-trimmed maneuver — the validator rejects the statically-visible case
    // offline; this is the runtime backstop. Ordinary AUTO never contradicts:
    // force-SAME just pins the departure's AUTO rung to the approach direction.
    // -----------------------------------------------------------------------
    if (_departingPivot.active)
    {
        // A binding direction override on the DEPARTURE edge `v→w`, derived with
        // the SAME precedence resolveDirection uses so this "would a rung force a
        // contradicting direction?" test matches what resolveDirection actually
        // returns: per-edge `direction_override` (step 1) → pipeline direction
        // default (step 2) → builder ARRIVAL override, last hop only (step 3) →
        // builder ROUTE override, all hops (step 4). The short-circuit order below
        // mirrors resolveDirection exactly; the AUTO heuristic (step 5) is NOT
        // binding, so force-SAME is free to pin the AUTO rung to the approach
        // direction.
        //
        // Arrival/route are included even though they are RUNTIME-only builder
        // overrides: for a pivot `(u,v,w)` whose goal is `w`, the departure `v→w`
        // is the last hop, so a `.setArrivalDirection(...)` (or a
        // `.setRouteDirection(...)`) can genuinely resolve the departure to a
        // direction that contradicts the approach. Unlike per-edge/pipeline
        // overrides, builder overrides are invisible to the offline validator (NO
        // static backstop), so this runtime check is the only guard against
        // silently overwriting the operator's explicit arrival/route direction
        // with a mis-trimmed pivot.
        Direction bindingDir = Direction::AUTO;
        if (edge.direction_override == Routes::DirectionMode::FORCE_FORWARD)
        {
            bindingDir = Direction::FRONT;
        }
        else if (edge.direction_override == Routes::DirectionMode::FORCE_REVERSE)
        {
            bindingDir = Direction::REAR;
        }
        else if (Routes::PIPELINE_CONFIG.direction == Routes::DirectionMode::FORCE_FORWARD)
        {
            bindingDir = Direction::FRONT;
        }
        else if (Routes::PIPELINE_CONFIG.direction == Routes::DirectionMode::FORCE_REVERSE)
        {
            bindingDir = Direction::REAR;
        }
        else if (isLastHop && builderArrival != Direction::AUTO)
        {
            bindingDir = builderArrival;
        }
        else if (builderRoute != Direction::AUTO)
        {
            bindingDir = builderRoute;
        }

        if (bindingDir != Direction::AUTO && bindingDir != _departingPivot.dir)
        {
            Serial.print(F("[PathTracker] ADR-071: pivot direction mismatch at v="));
            Serial.print(cur);
            Serial.print(F(" approach="));
            Serial.print(static_cast<uint8_t>(_departingPivot.dir));
            Serial.print(F(" departure-pin="));
            Serial.println(static_cast<uint8_t>(bindingDir));
            _enterFault(Fault::PIVOT_DIRECTION_MISMATCH, cur, nxt);
            return;
        }

        // Force-SAME: the departure adopts the pivot's approach direction. The
        // REAR +180 body flip is applied below when `bodyTarget` is computed, and
        // the wheel flip when `resolvedTurnMode` is resolved.
        resolved = _departingPivot.dir;
    }

    // Stamp the resolved (final) direction of THIS hop onto BOTH latch copies.
    // `_pendingPivot` was copied from `_approachingPivot` above BEFORE the
    // direction was resolved, so writing only one would leave the departing carry
    // at AUTO and force-SAME would misfire (spec latch-ordering note). When this
    // hop is a pivot-approach, `resolved` is its approach direction; when it is
    // also a chained pivot's departure, `resolved` is the forced value, so the
    // next pivot inherits a consistent direction.
    _approachingPivot.dir = resolved;
    _pendingPivot.dir     = resolved;

    // ------------------------------------------------------------------------
    // Early reverse-availability guard.
    //
    // Reject a REAR LINE hop on a 1-board chassis BEFORE the pre-edge turn is
    // dispatched, otherwise the chassis physically rotates 180° to face the
    // (unservable) REAR body-heading and then halts in fault — leaving the
    // robot pointing backward with REVERSE_UNAVAILABLE latched and the operator
    // forced to manually re-orient.
    //
    // Issue #64: the resolved PRIMITIVE — not a deleted projection edge type —
    // decides serviceability. `reverseUnavailable` keys on `edge.type`, which is
    // LINE or BLIND only now; a LINE edge into a projection DESTINATION node is
    // STILL a reverse line-follow (it tracks the line backward to its distance
    // gate), so it needs the rear board and is covered by the LINE clause.
    //
    // The auto branch of `_resolveDirection` short-circuits to FRONT when
    // `LINE_BOARD_COUNT < 2`, so this guard only fires for an operator-forced
    // REAR (per-edge override, pipeline default, or builder route/arrival).
    // BLIND REAR is permitted on 1-board chassis (no line sensor required),
    // so the check is scoped to LINE.
    //
    // The same guard is left in `_dispatchEdgeAfterTurn` as defensive
    // redundancy — if a future FSM change introduces a path into the
    // dispatcher that bypasses this early site, the late guard still fires.
    // ------------------------------------------------------------------------
    if (reverseUnavailable(resolved, edge.type, Config::LINE_BOARD_COUNT))
    {
        _printReverseUnavailable(cur, nxt, F("pre-turn guard"));
        _enterFault(Fault::REVERSE_UNAVAILABLE, cur, nxt);
        return;
    }

    float bodyTarget = edge.heading_deg;
    if (resolved == Direction::REAR)
    {
        bodyTarget = fmodf(bodyTarget + 180.0f, 360.0f);
        if (bodyTarget < 0.0f) bodyTarget += 360.0f;
    }

    _resolvedDirection   = resolved;
    _resolvedBodyHeading = bodyTarget;

    // ADR-036: the pre-turn maneuver wheel comes from the DEPARTING pivot (the turn we are
    // pivoting through at `cur` this hop), else the pipeline default. A valid pivot's turn
    // (validator floor >= Config::PIVOT_MIN_TURN_DEG) always exceeds NAV_TURN_TOLERANCE_DEG
    // (0.5° by default), so the shared turn fires and the pivot wheel applies — no firmware
    // min-angle change is needed. The wheel and the δ come from the SAME resolved record, so
    // they cannot disagree.
    // ADR-071 (#54): a REAR departure rolls the driven wheel BACKWARD, so the
    // stored forward-sense wheel plants the opposite side — `resolvePivotWheel`
    // flips PIVOT_LEFT<->PIVOT_RIGHT for a REAR pivot (FRONT/AUTO unchanged). The
    // table is the wheel authority; the flip is a pure function of the resolved
    // approach direction latched on the departing pivot.
    const Routes::TurnMode resolvedTurnMode =
        _departingPivot.active
            ? resolvePivotWheel(_departingPivot.wheel, _departingPivot.dir)
            : Routes::PIPELINE_CONFIG.turn;
    const uint8_t turnModeByte = _turnModeToByte(resolvedTurnMode);

    // ------------------------------------------------------------------------
    // Commanded-delta sign tripwire (issue #54, ADR-071, TC16). The dispatched
    // pivot wheel is the stored table's authority (`resolvePivotWheel` above),
    // NEVER derived from the live delta — doing so reintroduces ADR-036
    // Blocker-1. This only ASSERTS the table and the geometry agree: a CCW body
    // turn (delta > 0) plants the LEFT wheel driving forward / the RIGHT wheel
    // driving in reverse, and a CW turn (delta < 0) the opposite — so the wheel
    // the delta sign implies (direction-corrected through the SAME policy) must
    // equal the dispatched wheel. A gross disagreement means the table and the
    // realised geometry diverged; fault rather than pivot the wrong way.
    //
    // Only meaningful past the turn tolerance: a valid pivot always turns well
    // past it, and the offline `_PIVOT_MAX_TURN_DEG = 179` cap precludes the
    // near-180 sign ambiguity, so a sub-tolerance |delta| (no meaningful sign)
    // is skipped rather than special-cased.
    if (_departingPivot.active)
    {
        const float pivotDelta = wrapDeltaDeg(bodyTarget - currentWorld);
        if (fabsf(pivotDelta) > Config::NAV_TURN_TOLERANCE_DEG)
        {
            const Routes::TurnMode expectedForwardWheel =
                (pivotDelta > 0.0f) ? Routes::TurnMode::PIVOT_LEFT
                                    : Routes::TurnMode::PIVOT_RIGHT;
            const Routes::TurnMode expectedWheel =
                resolvePivotWheel(expectedForwardWheel, _departingPivot.dir);
            if (resolvedTurnMode != expectedWheel)
            {
                Serial.print(F("[PathTracker] ADR-071: pivot commanded-delta sign "
                               "tripwire at v="));
                Serial.print(cur);
                Serial.print(F(" delta="));
                Serial.print(pivotDelta);
                Serial.print(F(" wheel="));
                Serial.print(turnModeByte);
                Serial.print(F(" expected="));
                Serial.println(_turnModeToByte(expectedWheel));
                _enterFault(Fault::PIVOT_DIRECTION_MISMATCH, cur, nxt);
                return;
            }
        }
    }

    // Dispatch the pre-edge turn through the SHARED helper — the same path the return prologue
    // uses (issue #36, #6). It owns the skip-when-aligned decision (NAV_TURN_TOLERANCE_DEG), the
    // LineFollower-vs-Drive writer guard (the yaw-vs-peek divergence case), and the pass-through
    // reset, so the pre-edge turn is structurally identical for every hop and for the return.
    if (_dispatchTurnIfNeeded(bodyTarget, turnModeByte))
    {
        if (_departingPivot.active)
        {
            Serial.print(F("[PathTracker] ADR-036: pivot turn at v="));
            Serial.print(cur);
            Serial.print(F(" wheel="));
            Serial.print(turnModeByte);
            Serial.print(F(" trim="));
            Serial.println(_departingPivot.trim_cm);
        }
#ifdef PT_PARITY_TRACE
        _ptEmitTurn("turn", bodyTarget, resolvedTurnMode);
#endif
        _state = State::TURNING;
    }
    else
    {
        _dispatchEdgeAfterTurn();
    }
}

bool PathTrackerClass::_dispatchTurnIfNeeded(float bodyHeadingWorld, uint8_t turnModeByte)
{
    const float currentWorld = Localization.getHeadingDeg();
    const float delta        = wrapDeltaDeg(bodyHeadingWorld - currentWorld);
    if (fabsf(delta) <= Config::NAV_TURN_TOLERANCE_DEG)
    {
        return false;    // already aligned — caller dispatches the move directly
    }

    // A real pre-edge turn is required: the wheels go to Drive.turnToYaw alone.
    //
    // Disable a live LineFollower (no brake) so the two writers never fight over the motors —
    // without this, LineFollower.update() would keep issuing Drive.setSpeed each tick while
    // Drive.turnToYaw owns the wheels. This also covers the yaw-vs-peek divergence guard
    // (ADR-023 follow-up): the LINE_OVERSHOOT pass-through peek classifies on EDGE headings
    // (FieldMap) while this decision uses the live IMU heading (Localization); if the IMU has
    // drifted, the peek may have left the controller enabled while the live delta now demands a
    // turn. The bodyTarget-vs-currentWorld delta is the authoritative trigger.
    if (LineFollower.isEnabled())
    {
        LineFollower.setEnabled(false).setBrake(false);
    }

    // Clear any pass-through eligibility the LINE_FOLLOW peek latched: a real turn means the
    // chassis stops, so the post-turn dispatch must soft-start from TSHAPE_START_SPEED rather
    // than assume cruise. Leaving `_passThrough` set would make a post-turn blind move run its
    // pass-through branch (cruise start-speed) from a dead stop with no soft-start ramp, jolting
    // the chassis. (PR#35 findings #32 / #90.)
    _passThrough = false;

    // ADR-059: same rule for a blind-chain run. The walk-ahead classified the joints on EDGE
    // headings, but this live check compares the IMU heading — if drift has opened a real
    // delta mid-run, the turn takes the wheels and the chassis stops, so the chained speed
    // plan (built for a no-stop run) is void. Drop it loudly; the post-turn dispatch is then
    // an ordinary braked soft-start hop, and a fresh chain may re-form on a later hop.
    if (_chain.active)
    {
        Serial.println(F("[PathTracker] ADR-059: pre-edge turn fired mid-chain; dropping run"));
        _chainReset();
    }

    Drive.turnToYaw(Localization.toImuYaw(bodyHeadingWorld)).setMode(turnModeByte).async();
    return true;
}

void PathTrackerClass::_resetPivotLatch()
{
    // ADR-036: all three latch members move together. Clearing only some would
    // leak a pivot across a goal boundary — the bleed these resets prevent.
    _pendingPivot     = PivotRef{};
    _departingPivot   = PivotRef{};
    _approachingPivot = PivotRef{};
}

float PathTrackerClass::_pivotTrimDistance(const Routes::EdgeInfo& edge)
{
    // ADR-036 (issue #44) — per-turn pivot compensation, distance-terminated side.
    // LOCAL effective distance only; EDGES[*][*].distance_cm is never mutated. The
    // one-hop carry / latch is owned by the `_pendingPivot` contract resolved in
    // `_onPlanNextHop()`; this wrapper only gathers the two already-resolved trims
    // and delegates the subtract-and-clamp to the shared `resolvePivotTrim` policy
    // (PivotTrimPolicy.h) so the host test links the SAME symbol.
    //
    // DEPARTING trim: this hop LEAVES a pivot node (`cur`) — its outgoing
    // over-travel δ, so the axle lands exactly on the node. APPROACHING trim: this
    // dead-reckon edge drives INTO a pivot node (`nxt`) — its δ, so the chassis
    // stops δ short and the pivot starts at the geometric pivot point. On a chain
    // u→v→w→x the hop v→w nets `length − δ_v(departing) − δ_w(approaching)`.
    const float departingTrim   = _departingPivot.active ? _departingPivot.trim_cm : 0.0f;
    const float approachingTrim =
        _approachingPivot.active ? _approachingPivot.trim_cm : 0.0f;

    const PivotTrimResolution trim = resolvePivotTrim(
        edge.distance_cm, departingTrim, approachingTrim, Config::PIVOT_TRIM_MIN_DRIVE_CM);

    // Defensive clamp trace. The validator guarantees δ leaves at least the floor
    // on every dead-reckon side (and the chained sum-check on a shared edge), so a
    // correctly-built table never clamps here. A clamp means a future bad table /
    // validator mismatch — trace it (the policy already degraded to the floor so
    // Drive.driveDist never sees a non-positive cm, which it rejects and no-ops).
    if (trim.clamped)
    {
        Serial.print(F("[PathTracker] ADR-036: dead-reckon trim underflow (raw="));
        Serial.print(edge.distance_cm);
        Serial.print(F(" eff="));
        Serial.print(trim.effectiveCm);
        Serial.println(F("); clamping. Check validator/config."));
    }

    return trim.driveCm;
}

Routes::LineFollowMode PathTrackerClass::_resolveLineFollowMode(const Routes::EdgeInfo& edge)
{
    // issue #156 / ADR-069 — resolve the per-edge lateral LINE-follow MODE. The runtime analogue
    // of the file-local `_resolveSpeed`, but with NO mission-wide pipeline default:
    // `LineFollowMode::DEFAULT` (absence of an authored override) always resolves to `MIDDLE`
    // (centered follow, offset 0) — there is no `Routes::PIPELINE_CONFIG` follow-offset field. A
    // concrete `MIDDLE`/`LEFT`/`RIGHT` passes through. The caller maps this to a signed
    // forward-frame float via `lineFollowModeToSetpoint` and composes the reverse-hop sign at the
    // dispatch site; the parity stream records the resolved mode itself.
    return (edge.line_follow_mode_override == Routes::LineFollowMode::DEFAULT)
        ? Routes::LineFollowMode::MIDDLE
        : edge.line_follow_mode_override;
}

uint8_t PathTrackerClass::_peekNextNextHop() const
{
    // Shared one-hop lookahead guard. A pass-through decision peeks the hop AFTER
    // `_nextHop` toward `_destinationNode` and needs three guards, kept in one place
    // here so they cannot drift:
    //   1. not-final  — `_nextHop` is the destination, so there is no next-next hop
    //                   (the final/goal hop must never crash or trigger the rule).
    //   2. NODE_NONE  — `getNextHop` returns no route from `_nextHop` to the goal.
    //   3. self-loop  — the lookahead returns `_nextHop` itself.
    // Any failed guard yields `FieldMap::NODE_NONE`; otherwise the valid next-next
    // node id is returned for the caller to classify.
    //
    // Two callers share this ONE guard set. Issue #64 retired the OLD second caller
    // (the BLIND auto-rule projection classification — now a LOCAL
    // `isProjectionNode(_nextHop)` read with no next-next peek), and T19 added a new
    // one: `_evaluateProjectionPassThrough()` for the distance-gated
    // `line–proj–line` collapse. The other caller is the intersection-gated
    // `LINE_FOLLOW` pass-through. Both conform to the same three guards, kept intact
    // and self-contained here.
    if (_nextHop == _destinationNode)
    {
        return FieldMap::NODE_NONE;
    }
    const uint8_t nextNext = FieldMap::getNextHop(_nextHop, _destinationNode);
    if (nextNext == FieldMap::NODE_NONE || nextNext == _nextHop)
    {
        return FieldMap::NODE_NONE;
    }
    return nextNext;
}

bool PathTrackerClass::passThroughHeadingEligible(float                   inboundBodyHeadingDeg,
                                                  Direction               inboundDirection,
                                                  const Routes::EdgeInfo& outboundEdge,
                                                  bool                    outboundIsLastHop,
                                                  Direction activeArrivalDirection,
                                                  Direction activeRouteDirection)
{
    // The PURE ADR-023 collinearity + same-direction test (ADR-059). Every input is an
    // explicit argument — no FSM member reads — so the blind-chain walk-ahead can call it
    // repeatedly for multi-hop lookahead with synthesized inbound headings, and the Python
    // sim mirrors THIS one function (full contract in the header doxygen).
    //
    // Compare BODY headings: re-resolve the outbound direction with the inbound body
    // heading as the current heading, rotate the outbound edge heading 180° for REAR, then
    // wrap-normalize the delta. A FRONT↔REAR flip would force a mid-flight LineFollower
    // board swap (or a blind-chain sign flip) — never a continuation, so the resolved
    // direction must match the inbound one too.
    const Direction nextDir = resolveDirection(
        outboundEdge.direction_override,
        Routes::PIPELINE_CONFIG.direction,
        activeArrivalDirection,
        activeRouteDirection,
        inboundBodyHeadingDeg,
        outboundEdge.heading_deg,
        outboundIsLastHop,
        Config::BIDIRECTIONAL_AUTO_ENABLED,
        Config::LINE_BOARD_COUNT);

    float outboundBody = outboundEdge.heading_deg;
    if (nextDir == Direction::REAR)
    {
        outboundBody = fmodf(outboundBody + 180.0f, 360.0f);
        if (outboundBody < 0.0f) outboundBody += 360.0f;
    }

    const float delta = wrapDeltaDeg(outboundBody - inboundBodyHeadingDeg);
    return (fabsf(delta) <= Config::NAV_TURN_TOLERANCE_DEG) && (nextDir == inboundDirection);
}

bool PathTrackerClass::_passThroughHeadingOk(const Routes::EdgeInfo& outbound,
                                             uint8_t next) const
{
    // LIVE-STATE ADAPTER over the pure predicate (ADR-059). Both pass-through gates (the
    // intersection-gated line–line–line collapse in LINE_FOLLOW and the distance-gated
    // line–proj–line collapse in `_evaluateProjectionPassThrough`) call this thin wrapper;
    // the heading/direction MATH lives only in `passThroughHeadingEligible`, which the
    // blind-chain walk-ahead calls directly — one predicate, three gates, no drift
    // (mirrors the Python sim's single `_should_pass_through()`). The next-next hop is
    // still part of the active goTo(), so the same cached route/arrival overrides apply
    // (arrival only binds when C is the final destination, via the isLastHop flag).
    return passThroughHeadingEligible(_resolvedBodyHeading, _resolvedDirection, outbound,
                                      (next == _destinationNode), _activeArrivalDirection,
                                      _activeRouteDirection);
}

bool PathTrackerClass::_evaluateProjectionPassThrough() const
{
    // `line(A)–projection(B)–line(C)` pass-through gate (T19, issue #64). Called at
    // LINE→projection arm time: B == `_nextHop`, `_resolvedBodyHeading` /
    // `_resolvedDirection` already hold the A→B hop's body heading + direction.
    //
    // ROUTING LOOKAHEAD ONLY. This decides WHETHER the distance-gated follow into B
    // collapses the triple into one continuous A→C follow; it does NOT change which
    // gate fires the state transition (always the A→B DISTANCE gate — a projection
    // node has no junction marker). Every condition must hold; any miss ⇒ stop at B.

    // (1) Compiled out → never pass through. Leads so the NEXT_HOP peek below is
    // skipped entirely on a build without pass-through (matches the LINE_FOLLOW
    // intersection-gated peek's `PATH_PASS_THROUGH_ENABLED &&` lead).
    if (!Config::PATH_PASS_THROUGH_ENABLED)
    {
        return false;
    }

    // (2) B must be a NON-goal projection node with a real next hop C. The shared
    // `_peekNextNextHop()` applies the not-final / NODE_NONE / self-loop guards —
    // its not-final guard (`_nextHop == _destinationNode`) is exactly the `goTo(B)`
    // case, so a goal-is-B route returns NODE_NONE here and correctly stops at B.
    const uint8_t c = _peekNextNextHop();
    if (c == FieldMap::NODE_NONE)
    {
        return false;
    }

    // (3) The outbound edge B→C must be a LINE edge (both flanks LINE). The inbound
    // A→B is LINE by construction — this method is only reached on the LINE→projection
    // dispatch path. A BLIND flank (B→C blind) is a normal stop at B.
    const Routes::EdgeInfo& outbound = FieldMap::getEdge(_nextHop, c);
    if (outbound.type != Routes::EdgeType::LINE)
    {
        return false;
    }

    // (4) C must be a LINE (junction) node so the B→C hop re-anchors at a real
    // intersection (`Intersection.isAtIntersection()` fires at C). A projection C
    // (B→C would itself be a distance-gated follow with no re-anchor) or any other
    // type is rejected — those are out of scope for the line-proj-line collapse.
    if (FieldMap::getNodeType(c) != Routes::NodeType::LINE)
    {
        return false;
    }

    // (5) A→B→C collinear AND same direction — the SAME ADR-023 heading check the
    // intersection-gated pass-through uses, factored into the one shared
    // `_passThroughHeadingOk()` predicate so the two sites cannot drift.
    return _passThroughHeadingOk(outbound, c);
}

bool PathTrackerClass::_evaluateLineFollowPassThrough() const
{
    // `line(A)–line(B)–line(C)` intersection-gated pass-through gate (F3 — evaluate-once
    // hoist). Called at LINE→line (junction) arm time in `_dispatchEdgeAfterTurn`:
    // B == `_nextHop`, `_resolvedBodyHeading` / `_resolvedDirection` already hold the
    // A→B hop's body heading + direction. This is the dispatch-time twin of the peek the
    // `LINE_FOLLOW` state historically ran at intersection-detect; the decision is hoisted
    // to dispatch so the approach ramp knows whether to arm (terminating hop) or stay at
    // full cruise (pass-through) BEFORE the leading sensor sees the marker.
    //
    // The SAME three-part test the old detection peek used — NOT the stricter
    // `_evaluateProjectionPassThrough` conditions (that gate additionally requires a LINE
    // outbound edge + a LINE junction C, because a projection collapse re-follows the line
    // across a marker-less node). Here a BLIND outbound B→C is a valid pass-through target
    // (the LINE→BLIND handoff releases the follower without braking downstream), so guard
    // only on edge EXISTENCE + heading.

    // (1) Compiled out → never pass through. Leads so the NEXT_HOP peek is skipped
    // entirely on a build without pass-through (matches the old detection peek's
    // `PATH_PASS_THROUGH_ENABLED ? ... : NODE_NONE` lead).
    const uint8_t nextNext =
        Config::PATH_PASS_THROUGH_ENABLED ? _peekNextNextHop() : FieldMap::NODE_NONE;
    if (nextNext == FieldMap::NODE_NONE)
    {
        return false;
    }

    // (2) The outbound edge B→C must exist. `_peekNextNextHop()` already applied the
    // not-final / NODE_NONE / self-loop guards, so a valid `nextNext` here means the goal
    // is beyond B; a missing edge is still a normal stop-at-B.
    const Routes::EdgeInfo& outbound = FieldMap::getEdge(_nextHop, nextNext);
    if (outbound.type == Routes::EdgeType::NONE)
    {
        return false;
    }

    // (3) A→B→C collinear AND same direction — the shared ADR-023 `_passThroughHeadingOk()`
    // predicate (also used by the distance-gated projection collapse), comparing BODY
    // headings so a FRONT↔REAR flip (which would swap the active line board mid-flight) is
    // never a continuation. See the old detection-site comment (PR #34 review; TC9 / TC9c).
    return _passThroughHeadingOk(outbound, nextNext);
}

void PathTrackerClass::_chainReset()
{
    // ADR-059: value-initialize the whole run buffer back to the inactive state.
    // `active == false` gates every read, so the array wipe is hygiene, not correctness.
    _chain = ChainRun{};
}

void PathTrackerClass::_chainTryStart(uint8_t cur, bool reverse, Routes::BlindMode hop0Mode,
                                      Routes::SpeedTier hop0Speed, float hop0DistCm,
                                      int16_t cruisePwm, int16_t entryPwm)
{
    // ADR-059 blind-chain walk-ahead (full contract in the header doxygen). On entry the
    // hop being dispatched (hop 0: `cur -> _nextHop`) is already classified
    // BLIND_DEAD_RECKON by the caller; this method decides whether hop 0 STARTS a chained
    // run and, if so, how far the run extends. Leaves `_chain` INACTIVE unless a genuine
    // >= 2-hop run forms (AC12: a non-chain hop takes the ordinary per-hop path untouched).

    // ---- Hop-0 eligibility. -------------------------------------------------------------
    // RAW_FLAT has no ramps to chain; a sub-TSHAPE_MIN_MOVE_CM hop silently degrades the
    // Drive profile (start/end speeds ignored); an active approaching pivot means the
    // chassis must stop delta short of `_nextHop` and pivot there — all three make hop 0 an
    // ordinary braked hop.
    if (hop0Mode == Routes::BlindMode::RAW_FLAT)
    {
        return;
    }
    if (hop0DistCm < Config::TSHAPE_MIN_MOVE_CM)
    {
        return;
    }
    if (_approachingPivot.active)
    {
        return;
    }

    _chain.node[0]      = cur;
    _chain.node[1]      = _nextHop;
    _chain.hopDistCm[0] = hop0DistCm;

    uint8_t count       = 1;             // hops admitted so far (hop 0 is in)
    float   totalCm     = hop0DistCm;    // summed COMMANDED run distance
    float   inboundBody = _resolvedBodyHeading;    // hop 0's body heading seeds the walk

    // ---- Walk NEXT_HOP forward, admitting one hop per iteration. --------------------------
    // Every `break` below ENDS the chain BEFORE the hop under evaluation (Error Handling):
    // that hop dispatches later as a normal braked hop. Only the two run CAPS log — an
    // ordinary ineligibility (goal / reset / seek / bend / flip / short hop) is silent.
    while (true)
    {
        const uint8_t a = _chain.node[count];    // node the run currently ends on

        // Goal node: a run may END on the goal (the final hop brakes there) but never
        // continues past it. RESET node: the wall-zero interlude fires on arrival and
        // needs a stopped chassis, so the run must brake there too.
        if (a == _destinationNode)
        {
            break;
        }
        if (FieldMap::getNodeType(a) == Routes::NodeType::RESET)
        {
            break;
        }

        const uint8_t b = FieldMap::getNextHop(a, _destinationNode);
        if (b == FieldMap::NODE_NONE || b == a)
        {
            break;    // no route / self-loop — same guards as _peekNextNextHop
        }

        const Routes::EdgeInfo& e = FieldMap::getEdge(a, b);
        if (e.type != Routes::EdgeType::BLIND)
        {
            break;    // a LINE hop is the line-follower's domain — the run ends before it
        }

        // Pivot guards (defensive: a validator-passed pivot turn is >= PIVOT_MIN_TURN_DEG,
        // never collinear, so the heading predicate below would reject these joints anyway).
        // A pivot AT the joint (prev, a, b) means the turn at `a` is a pivot maneuver; a
        // pivot ONTO b (a, b, c) means the a->b hop must stop delta short of b. Both brake.
        if (FieldMap::findPivotTurn(_chain.node[count - 1], a, b) != nullptr)
        {
            break;
        }
        if (b != _destinationNode)
        {
            const uint8_t c = FieldMap::getNextHop(b, _destinationNode);
            if (c != FieldMap::NODE_NONE && c != b
                && FieldMap::findPivotTurn(a, b, c) != nullptr)
            {
                break;
            }
        }

        // Collinear + same-direction — the PURE predicate (a bend or a FRONT<->REAR flip
        // ends the run; the direction therefore stays constant along the whole run).
        if (!passThroughHeadingEligible(inboundBody, _resolvedDirection, e,
                                        (b == _destinationNode), _activeArrivalDirection,
                                        _activeRouteDirection))
        {
            break;
        }

        // Seek / motion-bundle resolution — the SAME resolver inputs the dispatch site
        // builds, so walk-time and dispatch-time classification cannot diverge. A seek
        // (stopAtLine) hop is chain-ineligible (the run ends; the seek is a fresh braked
        // hop whose arm window must absorb the run's residual dead-reckon error). That is
        // no longer a compile-enforced PATH_BLIND_CHAIN_MAX_CM <= STOP_AT_LINE_ARM_WINDOW_CM
        // budget: the distance cap was decoupled (owner decision, ConfigNav.h) and
        // MAX_CHAIN_HOPS is now the practical bound. If a chain-into-seek drift shows up on
        // the bench, add a SEPARATE bound scoped to that handoff (do NOT re-tie the cap).
        const bool autoRule =
            Routes::BLIND_PROJECTION_AUTO_SEEK && FieldMap::isProjectionNode(b);
        const StopAtLineQuery sq{
            _activeStopAtLine,
            e.stop_at_line_override,
            _resolveBlindMode(e),
            Routes::PIPELINE_CONFIG.blind,
            _resolvedDirection,
            Config::LINE_BOARD_COUNT,
            autoRule};
        const StopAtLineResolution sr = resolveStopAtLine(sq);
        if (sr.seek)
        {
            break;
        }
        if (sr.motionMode == Routes::BlindMode::RAW_FLAT)
        {
            break;    // no ramps to chain
        }

        // SPEED-tier boundary (issue #97). The run merges every admitted hop into ONE
        // velocity profile at a single cruise, so a hop whose resolved SpeedTier (edge
        // override, else PIPELINE_CONFIG.speed) differs from hop 0's cannot share that
        // profile and ends the run before it. SPEED ONLY — a SYNC<->GYRO mode mix at the
        // SAME tier still chains (the mode-mixed run already chains today, and adding a
        // stop there would fight the fewer-stops preference).
        if (_resolveSpeed(e) != hop0Speed)
        {
            break;
        }

        // A hop below the profile floor cannot carry chained start/end speeds (the Drive
        // builder silently degrades it to a flat move) — the run ends before it.
        if (e.distance_cm < Config::TSHAPE_MIN_MOVE_CM)
        {
            break;
        }

        // ---- Run caps — LOGGED truncations, never silent (Error Handling). --------------
        // The hop under evaluation passed EVERY eligibility guard above, so refusing it
        // here is a genuine cap truncation: the run ends at the cap (its current final hop
        // brakes) and a fresh chain may re-form from `b`'s hop onward.
        if (count >= Config::MAX_CHAIN_HOPS)
        {
            _printChainCapTruncation(F("hop cap"), count, totalCm, b);
            break;
        }
        if (totalCm + e.distance_cm > Config::PATH_BLIND_CHAIN_MAX_CM)
        {
            _printChainCapTruncation(F("distance cap"), count, totalCm, b);
            break;
        }

        // ---- Admit the hop. --------------------------------------------------------------
        // Mid-run hops carry the raw edge length: no pivot can trim them (the pivot guards
        // above ended the run at any pivot-adjacent joint), so the walk-time distance
        // equals the dispatch-time `resolvePivotTrim` output exactly.
        _chain.hopDistCm[count] = e.distance_cm;
        _chain.node[count + 1]  = b;
        ++count;
        totalCm += e.distance_cm;

        // The next joint's inbound body heading is THIS hop's body heading (REAR rotates
        // 180°, matching `_onPlanNextHop`'s bodyTarget math) — pairwise joint tests, the
        // same tolerance semantics the per-hop skip-turn check applies at dispatch time.
        inboundBody = e.heading_deg;
        if (_resolvedDirection == Direction::REAR)
        {
            inboundBody = fmodf(inboundBody + 180.0f, 360.0f);
            if (inboundBody < 0.0f)
            {
                inboundBody += 360.0f;
            }
        }
    }

    if (count < 2)
    {
        return;    // not a chain — hop 0 dispatches on the ordinary per-hop path (AC12)
    }

    // ---- Resolve the per-hop speed plan (pure two-pass planner, ChainSpeedPolicy.h). -----
    // `entryPwm` seeds the forward pass (AC5: the ADR-023 captured cruise after a
    // pass-through, else the +/-TSHAPE_START_SPEED cold-start seed); the backward pass
    // sheds every node speed low enough that the final hop's ordinary cruise->floor->brake
    // stop fits. The final hop's plan is `{startPwm, 0, brake=true}` by construction.
    const ChainSpeedConfig cfg{Config::PWM_MIN_MAGNITUDE, Config::PWM_MAX,
                               Config::TSHAPE_ACCEL_PWM_PER_CM, Config::TSHAPE_MIN_MOVE_CM};
    resolveChainSpeeds(_chain.hopDistCm, count, cruisePwm, entryPwm, cfg, _chain.speed);

    _chain.active         = true;
    _chain.reverse        = reverse;
    _chain.speedTier      = hop0Speed;
    _chain.hopCount       = count;
    _chain.hopIndex       = 0;
    _chain.dispatchEncSum = 0;
    _chain.dispatchCmdCm  = 0.0f;

    Serial.print(F("[PathTracker] ADR-059: blind chain "));
    Serial.print(FieldMap::getNode(_chain.node[0]).name);
    Serial.print(F(" -> "));
    Serial.print(FieldMap::getNode(_chain.node[count]).name);
    Serial.print(F(": hops="));
    Serial.print(count);
    Serial.print(F(" dist="));
    Serial.print(totalCm);
    Serial.print(F(" cm entryPwm="));
    Serial.println(entryPwm);
}

void PathTrackerClass::_dispatchEdgeAfterTurn()
{
    const uint8_t cur = Localization.getCurrentNode();
    const Routes::EdgeInfo& edge = FieldMap::getEdge(cur, _nextHop);

    // -----------------------------------------------------------------------
    // Reverse-availability guard (T7 / issue #19) — defensive redundancy.
    //
    // Same condition as the early guard in `_onPlanNextHop` (see the block
    // beginning "Early reverse-availability guard"). The early guard is the
    // load-bearing site — it fires before the 180° pre-edge turn dispatches —
    // and this dispatcher-side check exists only to defend against future
    // FSM changes that might route into `_dispatchEdgeAfterTurn` without
    // re-running `_onPlanNextHop`.
    //
    // BLIND REAR is permitted on a 1-board chassis: dead-reckoned reverse
    // does not require a rear line sensor — only negative PWM.
    // -----------------------------------------------------------------------
    if (reverseUnavailable(_resolvedDirection, edge.type, Config::LINE_BOARD_COUNT))
    {
        _printReverseUnavailable(cur, _nextHop, F("post-turn guard"));
        _enterFault(Fault::REVERSE_UNAVAILABLE, cur, _nextHop);
        return;
    }

    // NONE / unknown edge type → unreachable. This replaces the old switch's
    // `case NONE: default:` arm: `resolveEdgeDispatch` only classifies LINE/BLIND
    // (its `else` is BLIND), so a non-routed edge must fault BEFORE the resolver.
    if (edge.type != Routes::EdgeType::LINE && edge.type != Routes::EdgeType::BLIND)
    {
        _enterFault(Fault::UNREACHABLE, cur, _nextHop);
        return;
    }

    // -----------------------------------------------------------------------
    // Build the resolved query for the terminal one-hop classifier
    // (`EdgeDispatchPolicy.h::resolveEdgeDispatch`, issue #108). Every input is
    // already resolved here — the classifier composes the existing pure policies
    // (`resolveStopAtLine` / `resolvePivotTrim` / `resolveSensorOvershootTrim`)
    // and owns the trim-clamp arithmetic, but introduces no new cross-hop state.
    // -----------------------------------------------------------------------

    // Direction-aware axle offset (the seek's overshoot channel), single-sourced
    // through the shared policy (ADR-071, #54): a REAR seek rolls the rear board's
    // own offset, so this seek budget cannot drift from the LINE-overshoot budget,
    // the offline validator, or the sim.
    const float axleOffsetCm = pivotOvershootBudgetCm(_resolvedDirection);
    const float departingTrimCm   = _departingPivot.active ? _departingPivot.trim_cm : 0.0f;
    const float approachingTrimCm = _approachingPivot.active ? _approachingPivot.trim_cm : 0.0f;

    // Hoist the (side-effect-free, `const`) projection pass-through lookahead so it is
    // a resolved query input. It is only MEANINGFUL on a LINE→projection hop (the
    // classifier reads it nowhere else); computing it unconditionally is harmless
    // (pure reads of NEXT_HOP + edge geometry) and keeps the FSM the single owner of
    // the lookahead. The original assignment to `_projPassThrough` happens in the
    // PROJECTION_FOLLOW branch below, unchanged.
    const bool projPassThrough = _evaluateProjectionPassThrough();

    EdgeDispatchQuery q{};
    q.edgeType          = edge.type;
    q.destNodeType      = FieldMap::getNodeType(_nextHop);
    q.direction         = _resolvedDirection;
    q.rawDistanceCm     = edge.distance_cm;
    q.speed             = _resolveSpeed(edge);
    q.projPassThrough   = projPassThrough;
    q.departingTrimCm   = departingTrimCm;
    q.approachingTrimCm = approachingTrimCm;
    q.axleOffsetCm      = axleOffsetCm;
    q.minDriveCm        = Config::PIVOT_TRIM_MIN_DRIVE_CM;

    if (edge.type == Routes::EdgeType::BLIND)
    {
        // -----------------------------------------------------------------
        // Effective stop-at-line resolution (ADR-027 / issue #27) — unchanged.
        //
        // `resolveStopAtLine(...)` folds the call-site flag, the per-edge BlindMode,
        // the pipeline blind default, the resolved direction, and the board count
        // into `{seek, motionMode}`. Issue #64 auto-rule (LOCAL): a BLIND edge whose
        // DESTINATION is a projection node seeks a line first to re-anchor; the
        // toggle leads the `&&` so `isProjectionNode` is skipped when compiled out.
        // -----------------------------------------------------------------
        const bool autoRuleActive =
            Routes::BLIND_PROJECTION_AUTO_SEEK && FieldMap::isProjectionNode(_nextHop);
        const StopAtLineQuery sq{
            _activeStopAtLine,
            edge.stop_at_line_override,
            _resolveBlindMode(edge),
            Routes::PIPELINE_CONFIG.blind,
            _resolvedDirection,
            Config::LINE_BOARD_COUNT,
            autoRuleActive};
        q.stopAtLine = resolveStopAtLine(sq);
    }
    else
    {
        // LINE edge: `stopAtLine` is unused by the classifier — default-construct it.
        q.stopAtLine = StopAtLineResolution{};
    }

    const EdgeDispatchDecision decision = resolveEdgeDispatch(q);

    // ADR-059 defensive: a blind-chain run only ever continues through BLIND_DEAD_RECKON
    // dispatches — the walk-ahead admitted exclusively plain-dead-reckon BLIND hops, so any
    // other primitive arriving while a run is live means the world diverged from the walk
    // (unreachable by construction on a static NEXT_HOP table). Drop the run loudly and let
    // the primitive dispatch normally rather than carry a stale speed plan forward.
    if (_chain.active && decision.primitive != DispatchPrimitive::BLIND_DEAD_RECKON)
    {
        Serial.println(F("[PathTracker] ADR-059: non-blind dispatch mid-chain; dropping run"));
        _chainReset();
    }

    switch (decision.primitive)
    {
        case DispatchPrimitive::LINE_FOLLOW_INTERSECTION:
        {
            // LINE → line (junction) node: follow to the next intersection. ADR-036:
            // a LINE outgoing edge applies no dead-reckon trim — it is sensor-
            // terminated and re-anchors on the junction line. The approach trim for a
            // LINE-incoming pivot is the overshoot-advance reduction in the LINE_FOLLOW
            // state; the outgoing carry is owned by the `_pendingPivot` latch.
            //
            // Drain any stale Intersection edge so the first follow tick does not skip
            // the segment. `.setReverse(...)` selects forward-vs-rear board + error
            // sign; `.setOvershootMode(false)` flips back to normal P+D after any
            // LINE_OVERSHOOT window. For a pass-through into this edge `setEnabled(true)`
            // hits the already-enabled fast-path (no ramp-up reseed); see ADR-023.
            (void) Intersection.isAtIntersection();
            const Config::SpeedPreset preset = speedTierToPreset(q.speed);
            // issue #156: resolve the per-edge lateral follow MODE and compose the reverse-hop
            // sign flip (LEFT stays left-of-TRAVEL on a rear-board hop). LineFollower stays
            // nav-agnostic — it takes the signed float, not the enum. `mode` is also read by the
            // parity-trace emit below, so it is resolved unconditionally.
            const Routes::LineFollowMode mode             = _resolveLineFollowMode(edge);
            const float                  sign             = decision.reverse ? -1.0f : 1.0f;
            const float                  lineFollowOffset = sign * lineFollowModeToSetpoint(mode);

            // F3 — evaluate-once pass-through hoist. Decide HERE (at dispatch) whether
            // this hop terminates at `_nextHop` or passes collinearly through it, and
            // latch the answer. `LINE_FOLLOW` consumes the SAME latch at
            // intersection-detect (no recompute), and the approach-ramp arm below reads
            // it too. Pure over headings/geometry — no live yaw. See ADR-023.
            _lineFollowPassThrough = _evaluateLineFollowPassThrough();

            // C1 — anticipatory approach ramp (the headline behaviour). On a TERMINATING
            // LINE hop (NOT a pass-through) with `Config::LINE_APPROACH_RAMP_ENABLED`,
            // predict how far the leading sensor board travels before it reaches the
            // target line and chain `.setApproachTarget(predictedCm)` so LineFollower
            // ramps cruise → the kinetic floor over that distance instead of holding full
            // cruise until detect. The prediction is the PIVOT-TRIMMED edge length
            // (`_pivotTrimDistance(edge)` — the SAME value `PROJECTION_FOLLOW` drives; NOT
            // `decision.commandedDistanceCm`, which is 0.0 by design for this
            // sensor-terminated primitive, so it would make the schedule degenerate and
            // the ramp a silent no-op) MINUS the direction-aware sensor-to-axle offset
            // `axleOffsetCm` (front 12.02 / rear 7.68, already resolved above). A
            // pass-through node keeps FULL cruise — the robot drives straight through it,
            // so the ramp is suppressed. When the ramp is disabled OR this is a
            // pass-through, the plain arm (no `setApproachTarget`) DISARMS any prior
            // schedule — byte-identical to today's flat behaviour. The lateral follow
            // offset (issue #156) is applied on BOTH arms; the soft-start ramp-up is left
            // ON (`setTShapeRampUp` default-true): T6 phase-gates the approach so ramp-up
            // and the anticipatory ramp-down coexist.
            if (Config::LINE_APPROACH_RAMP_ENABLED && !_lineFollowPassThrough)
            {
                const float predictedCm = _pivotTrimDistance(edge) - axleOffsetCm;
                LineFollower.setEnabled(true)
                            .setPreset(preset)
                            .setReverse(decision.reverse)
                            .setOvershootMode(false)
                            .setLineFollowOffset(lineFollowOffset)
                            .setApproachTarget(predictedCm);
            }
            else
            {
                LineFollower.setEnabled(true)
                            .setPreset(preset)
                            .setReverse(decision.reverse)
                            .setOvershootMode(false)
                            .setLineFollowOffset(lineFollowOffset);
            }
#ifdef PT_PARITY_TRACE
            _ptEmitLineFollow(decision.reverse, q.speed, mode);
#endif
            _state       = State::LINE_FOLLOW;
            _passThrough = false;    // consumed
            break;
        }

        case DispatchPrimitive::PROJECTION_FOLLOW:
        {
            // LINE → projection node: DISTANCE-gated follow. The gate uses the pivot-
            // trimmed edge length (`_pivotTrimDistance`, the `distance` regime); the
            // PROJECTION_FOLLOW state advances the node via `onDeadReckonedAdvance`
            // when it trips. T19 (issue #64) `line–projection–line` PASS-THROUGH: the
            // lookahead was hoisted above into `q.projPassThrough`; latch it now (the
            // original `_projPassThrough = _evaluateProjectionPassThrough()` point) so
            // the gate is armed with `brakeOnGate == false` and the A→C follow stays
            // continuous across the marker-less node B.
            //
            // `_pivotTrimDistance(edge)` fires the dead-reckon clamp underflow trace
            // and returns the SAME value as `decision.commandedDistanceCm` (both
            // delegate to `resolvePivotTrim` with identical inputs), so dispatching its
            // return is behavior-identical to the pre-refactor code while keeping the
            // trace; the dispatched `effectiveDistCm` is what the parity stream records.
            (void) Intersection.isAtIntersection();
            const Config::SpeedPreset preset = speedTierToPreset(q.speed);
            // issue #156: resolve the per-edge lateral follow MODE and compose the reverse-hop
            // sign flip, exactly as the LINE_FOLLOW_INTERSECTION path. LineFollower takes the
            // signed float; the parity stream records the resolved mode (PRE-reverse-sign).
            const Routes::LineFollowMode mode = _resolveLineFollowMode(edge);
            const float                  sign = decision.reverse ? -1.0f : 1.0f;
            _projPassThrough            = q.projPassThrough;
            const float effectiveDistCm = _pivotTrimDistance(edge);
            LineFollower.setEnabled(true)
                        .setPreset(preset)
                        .setReverse(decision.reverse)
                        .setOvershootMode(false)
                        .setTShapeRampDown(!_projPassThrough)
                        .setBrakeOnGate(decision.brakeOnGate)
                        .setDistanceTarget(effectiveDistCm)
                        .setLineFollowOffset(sign * lineFollowModeToSetpoint(mode));
#ifdef PT_PARITY_TRACE
            _ptEmitProjectionFollow(decision.reverse, q.speed, mode, effectiveDistCm,
                                    _projPassThrough);
#endif
            _state       = State::PROJECTION_FOLLOW;
            _passThrough = false;    // consumed
            break;
        }

        case DispatchPrimitive::STOP_AT_LINE_SEEK:
        {
            // Pivot-scoped REAR-seek serviceability (issue #54, ADR-071). A REAR
            // stop-at-line seek INTO a pivot re-anchors on the crossing line with the
            // REAR board (ADR-027); on a <2-board chassis that board is absent, so the
            // seek cannot land the axle δ-short and the pivot would fire off-target.
            // Fault here rather than seek forward-blind. Scoped to `_approachingPivot`
            // so it does NOT widen `reverseUnavailable` (which keys on EdgeType::LINE),
            // preserving the documented NON-pivot forced-REAR STOP_AT_LINE ->
            // dead-reckon-reverse degrade (a BLIND path legitimately runs open-loop
            // reverse on 1 board).
            if (_approachingPivot.active && decision.reverse && Config::LINE_BOARD_COUNT < 2)
            {
                _printReverseUnavailable(cur, _nextHop, F("pivot REAR-seek"));
                _enterFault(Fault::REVERSE_UNAVAILABLE, cur, _nextHop);
                return;
            }

            // BLIND + seek (ADR-027). A pass-through LINE->BLIND handoff leaves the
            // LineFollower enabled (T5's brake-on-gate=false path); release it WITHOUT
            // braking before handing the wheels to the Drive seek so the two writers
            // never fight over the motors.
            if (_passThrough)
            {
                LineFollower.setEnabled(false).setBrake(false);
            }

            // Drain any stale Intersection edge so the seek's own physical crossing
            // cannot leak a phantom nav edge into the next LINE hop (AC7 — the forward
            // seek drives over its target T_OR_CROSS while Intersection.update()
            // independently counts it).
            (void) Intersection.isAtIntersection();

            // PIVOT_TRIM_UNDERFLOW fault (ADR-044). The decision struct intentionally
            // carries no clamp flag, so re-derive the clamp at the dispatch site from
            // the SAME resolveSensorOvershootTrim inputs the classifier used for
            // `decision.overshootCm`. A seek into an approaching pivot must land the
            // axle delta short of the node; a forward-only seek with delta > offset has
            // no positive overshoot left to shorten, so it hard-faults rather than
            // firing the pivot off-target (ADR-039; #75). With no approaching pivot the
            // trim is 0 ⇒ overshoot stays the full direction-resolved offset and a
            // plain seek is unchanged (AC2).
            const SensorOvershootResolution seekOvershoot = resolveSensorOvershootTrim(
                axleOffsetCm, approachingTrimCm, Config::PIVOT_TRIM_MIN_DRIVE_CM);
            if (seekOvershoot.clamped)
            {
                _printOvershootTrimUnderflow(F("ADR-044"), axleOffsetCm, approachingTrimCm);
                Serial.println(F("delta exceeds offset; faulting (see #75)."));
                _enterFault(Fault::PIVOT_TRIM_UNDERFLOW, cur, _nextHop);
                return;
            }

            // ADR-038: a missed seek is always a hard fault (Drive's default max search
            // distance; no zero-margin override, no soft fallback). The seek drives the
            // FULL commanded distance (== edge.distance_cm — the crossing, not the
            // distance, terminates it) and rolls `decision.overshootCm` to land the axle
            // on the node. GYRO/SYNC come from the resolved bundle; the body heading
            // already accounts for REAR. The Drive builder is move-only, so the whole
            // chain is ONE discarded-temporary RAII `.async()` statement; the
            // STOP_AT_LINE state polls Drive.isIdle()/hasFault() for the outcome.
            Drive.stopAtLine(decision.commandedDistanceCm)
                .setReverse(decision.reverse)
                .setMode(blindModeToDriveMode(decision.driveMode))
                .setHeading(Localization.toImuYaw(_resolvedBodyHeading))
                .setPower(presetToPwm(speedTierToPreset(q.speed)))
                .setOvershoot(decision.overshootCm)
                .async();
#ifdef PT_PARITY_TRACE
            _ptEmitStopAtLineSeek(decision.reverse, decision.driveMode,
                                  decision.commandedDistanceCm, decision.overshootCm);
#endif
            _state       = State::STOP_AT_LINE;
            _passThrough = false;    // consumed
            break;
        }

        case DispatchPrimitive::BLIND_DEAD_RECKON:
        {
            // Pass-through LINE->BLIND handoff: when _passThrough is set the
            // LineFollower is still enabled (T5's brake-on-gate=false path), so:
            //   1. Capture the current cruise PWM BEFORE disabling LineFollower.
            //   2. Disable LineFollower WITHOUT braking — it releases the motors
            //      cleanly; the last latched speeds stay until the next setStartSpeed.
            //   3. Pass the captured cruise as the start speed so the chassis does not
            //      dip to the deadband floor first.
            // For non-pass-through, LineFollower was already disabled by T5's path and
            // startSpeed defaults to the existing TSHAPE_START_SPEED. See ADR-023.
            int16_t startSpeed = Config::TSHAPE_START_SPEED;
            if (_passThrough)
            {
                startSpeed = presetToPwm(LineFollower.getPreset());
                LineFollower.setEnabled(false).setBrake(false);
            }

            // Per-edge blind cruise magnitude (issue #97). Reuse THIS hop's already-
            // resolved SpeedTier (`q.speed`, the edge override else PIPELINE_CONFIG.speed)
            // as a PWM — the former hardcoded Config::SPEED_NORMAL is now a per-edge value.
            // NORMAL/absent stays byte-identical to the previous 70. Computed BEFORE the
            // seed clamp and the signed cruise so both derive from it.
            const int16_t cruiseMag = presetToPwm(speedTierToPreset(q.speed));

            // AC12 pass-through entry seed clamp: the captured inbound line cruise (e.g. a
            // FAST line at 90) must not enter a slower blind hop above its own cruise. Clamp
            // the UNSIGNED seed magnitude to the resolved cruise before the sign is applied,
            // so a FAST(90) line into a SLOW(60) blind enters at 60, not 90. A cold-start
            // seed (TSHAPE_START_SPEED) already sits below any tier, so this is a no-op there.
            if (startSpeed > cruiseMag)
            {
                startSpeed = cruiseMag;
            }

            // BLIND REAR (T7 / issue #19): sign-by-power is the convention —
            // `driveDist` rejects negative cm at Drive.h:1254 (positive distance,
            // signed PWM). Negate the cruise + start/end speeds when REAR so the
            // trapezoid ramps from a negative-direction PWM straight away
            // (Drive.h:1288-1296 documents the matching sign-by-cruise rule).
            const int16_t cruisePwm = decision.reverse ? (int16_t) -cruiseMag
                                                        : (int16_t)  cruiseMag;
            int16_t signedStartSpeed =
                decision.reverse ? (int16_t) -startSpeed : startSpeed;

            // `_pivotTrimDistance(edge)` fires the dead-reckon clamp underflow trace and
            // returns the SAME value as `decision.commandedDistanceCm` (both delegate to
            // `resolvePivotTrim` with identical inputs: edge.distance_cm minus the two
            // latched pivot trims, clamped to the floor). Dispatching its return is
            // behavior-identical to the pre-refactor code while keeping the trace;
            // the dispatched distance is the value the parity stream records.
            const float effectiveDistCm = _pivotTrimDistance(edge);

            // ---------------------------------------------------------------------------
            // ADR-059 blind-chain resolution. Exactly one of three paths applies:
            //   (a) a run is IN FLIGHT — this dispatch is its next hop: subtract the
            //       MEASURED inter-hop boundary carry (AC9) and take the hop's resolved
            //       (startPwm, endPwm, brake) from the run buffer;
            //   (b) no run + chaining enabled — try to START one via the walk-ahead
            //       (`_chainTryStart`); on success this hop is the run's first mid-hop;
            //   (c) chaining disabled / no eligible run — the ordinary per-hop dispatch
            //       below is UNTOUCHED (AC12; TC5: PATH_BLIND_CHAIN_ENABLED=false is
            //       byte-identical to today's stop-at-every-node behavior).
            // ---------------------------------------------------------------------------
            float   dispatchCm  = effectiveDistCm;
            bool    chainMid    = false;    // dispatch with chained endPwm + setBrake(false)
            int16_t chainEndPwm = 0;

            if (_chain.active)
            {
                const uint8_t k = (uint8_t) (_chain.hopIndex + 1);
                const bool    match =
                    (k < _chain.hopCount) && (cur == _chain.node[k])
                    && (_nextHop == _chain.node[k + 1])
                    && (decision.reverse == _chain.reverse)
                    && (q.speed == _chain.speedTier)
                    && (decision.driveMode != Routes::BlindMode::RAW_FLAT);
                if (match)
                {
                    // Measured inter-hop boundary carry (AC9). Between the previous hop's
                    // profile completing and THIS dispatch, the chassis keeps rolling
                    // (Drive's empty-queue chained-end path releases the brake and coasts
                    // — Drive.cpp finding #13 — for the ~2-3 FSM ticks of the
                    // BLIND_FORWARD -> PLAN_NEXT_HOP -> dispatch boundary, ~1.5-3 cm at
                    // cruise). That travel lands BEFORE the new move's encoder baseline,
                    // so subtract the MEASURED overshoot — travel since the previous
                    // hop's dispatch minus its commanded distance — from this hop's
                    // command. Dispatch-to-dispatch deltas tile the run's timeline with
                    // no gap or overlap, so the correction telescopes and the run's final
                    // node lands on-node with NO error accumulation over N hops. Never an
                    // assumed constant. A negative measurement (host mocks with parked
                    // encoders; an under-traveled move) clamps to 0 — never LENGTHEN a
                    // hop, and the parity harness (encoders parked) then records the
                    // planned distances exactly like the sim.
                    const int32_t encSum = Drive.getEncL() + Drive.getEncR();
                    const float   traveledCm =
                        fabsf(encDegToCm((encSum - _chain.dispatchEncSum) / 2));
                    float carryCm = traveledCm - _chain.dispatchCmdCm;
                    if (carryCm < 0.0f)
                    {
                        carryCm = 0.0f;
                    }
                    dispatchCm = effectiveDistCm - carryCm;
                    if (dispatchCm < Config::PIVOT_TRIM_MIN_DRIVE_CM)
                    {
                        dispatchCm = Config::PIVOT_TRIM_MIN_DRIVE_CM;
                    }

                    _chain.hopIndex  = k;
                    signedStartSpeed = _chain.speed[k].startPwm;
                    if (!_chain.speed[k].brake)
                    {
                        chainMid    = true;
                        chainEndPwm = _chain.speed[k].endPwm;
                    }
                    // else: the run's FINAL hop — the braked cruise->floor->brake stop
                    // (AC3), dispatched below with the chained start speed AND the tShape
                    // end target pinned to the FLOOR (see `chainFinalEndPwm`).
                }
                else
                {
                    // Divergence from the walked plan — unreachable by construction on a
                    // static NEXT_HOP table (a mid-run turn already drops the run in
                    // _dispatchTurnIfNeeded); guard it anyway. The chassis is physically
                    // near the previous hop's planned exit PWM, so seed the fallback
                    // braked hop from it (sign-checked against the new cruise) instead of
                    // re-dipping to the floor.
                    Serial.println(
                        F("[PathTracker] ADR-059: chain divergence at dispatch; "
                          "abandoning run"));
                    const int16_t heldPwm = _chain.speed[_chain.hopIndex].endPwm;
                    _chainReset();
                    if (heldPwm != 0 && ((heldPwm < 0) == (cruisePwm < 0)))
                    {
                        signedStartSpeed = heldPwm;
                    }
                }
            }
            else if (Config::PATH_BLIND_CHAIN_ENABLED)
            {
                _chainTryStart(cur, decision.reverse, decision.driveMode, q.speed,
                               effectiveDistCm, cruisePwm, signedStartSpeed);
                if (_chain.active)
                {
                    // Hop 0 of a fresh run. `speed[0].startPwm` is the entryPwm clamped
                    // into [PWM_MIN_MAGNITUDE, |cruise|] by the planner (== the ADR-023
                    // captured cruise after a pass-through, else the cold-start seed —
                    // AC5). hopCount >= 2, so hop 0 is always a non-braking mid-hop.
                    signedStartSpeed = _chain.speed[0].startPwm;
                    if (!_chain.speed[0].brake)
                    {
                        chainMid    = true;
                        chainEndPwm = _chain.speed[0].endPwm;
                    }
                }
            }

            const bool chainHop = _chain.active;

            // ADR-059 chain-FINAL hop (the run buffer's braked last hop; `chainHop` with
            // `chainMid` false): pin the braked tShape's end target to the FLOOR
            // (±PWM_MIN_MAGNITUDE), NOT 0. The braked-default endSpeed=0 asks the kernel
            // to shed cruise->0 — (85−0)/5 = 17 cm of ramp-down — which cannot fit a
            // short final hop, so the profile triangle-collapses
            // (TShapeProfilePolicy.h: peak = (85+0+85)/3 = 56) and the hop LAUNCHES
            // below the cruise the chassis physically enters at — the exact re-dip lurch
            // ADR-059 removes. Ending at the floor sizes the shed at
            // (cruise − PWM_MIN)/accel = 7 cm, which fits: the hop launches at the plan
            // start, sheds cruise->floor, and the brake (still default-on) completes
            // floor->rest — ADR-058's cruise->floor->brake stop shape. Non-chain braked
            // hops deliberately keep the endSpeed=0 default (they start from rest / a
            // low seed and do not triangle-collapse; changing them would alter existing
            // motion and re-pin goldens — AC12).
            // Revisit the final end speed if the robot cannot brake to rest from the
            // floor due to inertia on a short final hop (owner decision, Diego 2026-07).
            const bool    chainFinal = chainHop && !chainMid;
            const int16_t chainFinalEndPwm =
                decision.reverse ? (int16_t) -Config::PWM_MIN_MAGNITUDE
                                 : (int16_t) Config::PWM_MIN_MAGNITUDE;

            // RAW_FLAT start resolution, hoisted so the RAW dispatch branch and the parity
            // emit below cannot drift: pass-through is the one case where the inbound
            // LineFollower cruise can legitimately differ from cruisePwm (non-NORMAL
            // preset on the inbound LINE edge). Only read on the RAW_FLAT branch.
            const int16_t rawStart = _passThrough ? signedStartSpeed : cruisePwm;

            if (chainHop && dispatchCm < Config::TSHAPE_MIN_MOVE_CM)
            {
                // ADR-059 degraded chained stub: the measured-carry subtraction shrank
                // this hop below the profile floor (a planned >= TSHAPE_MIN_MOVE_CM hop
                // with a large boundary carry). The builder would silently degrade the
                // profiled dispatch to a FLAT move at POWER = cruise — a mid-run PWM step
                // to cruise (or a cruise-then-brake slam on the final hop). Dispatch the
                // stub explicitly FLAT at the hop's OWN planned PWM instead: a mid-hop
                // holds its planned exit speed (no brake, run continues), a final hop
                // crawls the residual at its low planned entry speed into the ordinary
                // brake. RAW_FLAT never chains, so only SYNC/GYRO reach this.
                const int16_t stubPwm = chainMid ? chainEndPwm : signedStartSpeed;
                Serial.print(F("[PathTracker] ADR-059: chained hop shrunk to "));
                Serial.print(dispatchCm);
                Serial.println(F(" cm (below profile floor); dispatching flat at plan PWM"));
                if (decision.driveMode == Routes::BlindMode::GYRO_TSHAPE)
                {
                    Drive.driveDist(dispatchCm)
                         .setMode(DriveMode::GYRO)
                         .setHeading(Localization.toImuYaw(_resolvedBodyHeading))
                         .setPower(stubPwm)
                         .flat()
                         .setBrake(!chainMid)
                         .async();
                }
                else
                {
                    Drive.driveDist(dispatchCm)
                         .setMode(DriveMode::SYNC)
                         .setPower(stubPwm)
                         .flat()
                         .setBrake(!chainMid)
                         .async();
                }
            }
            else
            {
                // The resolved motion bundle (`decision.driveMode`) maps onto one of three
                // Drive builder chains:
                //   - SYNC_TSHAPE : driveDist(cm).setMode(SYNC).tShape() (encoder-rate sync
                //                   PID + trapezoidal velocity profile)
                //   - GYRO_TSHAPE : driveDist(cm).setMode(GYRO).tShape() (straight-line yaw
                //                   lock; heading is the BODY heading, already REAR-rotated)
                //   - RAW_FLAT    : driveDist(cm).setMode(RAW).setStartSpeed(cruise)
                //                   .setEndSpeed(cruise) (open-loop PWM, no ramp)
                // DEFAULT/STOP_AT_LINE cannot reach this switch — the resolver guarantees a
                // concrete bundle; the SYNC_TSHAPE default below is belt-and-braces.
                //
                // ADR-059: a chained MID-hop (`chainMid`) additionally chains
                // `.setEndSpeed(chainEndPwm).setBrake(false)` so the chassis passes the
                // node boundary without braking to rest (AC3) — Drive's empty-queue
                // chained-end path then releases the brake and coasts for the short
                // boundary window until the next chained hop re-drives the wheels at that
                // same planned PWM. A chain FINAL hop (`chainFinal`) takes the braked
                // statement with its start speed seeded from the run plan and its tShape
                // end target pinned to the FLOOR (`chainFinalEndPwm` — see the block
                // above for why 0 triangle-collapses a short final hop); a non-chain hop
                // is byte-identical to pre-ADR-059 (AC12). RAII rule throughout: each
                // Drive chain is ONE discarded-temporary statement.
                switch (decision.driveMode)
                {
                    case Routes::BlindMode::GYRO_TSHAPE:
                    {
                        // GYRO holds the BODY heading — use `_resolvedBodyHeading` (already
                        // rotated 180 deg for REAR by `_onPlanNextHop`), not raw edge.heading_deg.
                        const int16_t imuTarget = Localization.toImuYaw(_resolvedBodyHeading);
                        if (chainMid)
                        {
                            Drive.driveDist(dispatchCm)    // carry-adjusted (AC9)
                                 .setMode(DriveMode::GYRO)
                                 .setHeading(imuTarget)
                                 .setPower(cruisePwm)
                                 .setStartSpeed(signedStartSpeed)    // ChainHopSpeed.startPwm
                                 .setEndSpeed(chainEndPwm)           // ChainHopSpeed.endPwm
                                 .setBrake(false)                    // never brake mid-run (AC3)
                                 .tShape()
                                 .async();
                        }
                        else if (chainFinal)
                        {
                            // ADR-059 chain-FINAL: braked stop shedding cruise->FLOOR
                            // (brake completes floor->rest) — see `chainFinalEndPwm`.
                            Drive.driveDist(dispatchCm)    // carry-adjusted (AC9)
                                 .setMode(DriveMode::GYRO)
                                 .setHeading(imuTarget)
                                 .setPower(cruisePwm)
                                 .setStartSpeed(signedStartSpeed)    // ChainHopSpeed.startPwm
                                 .setEndSpeed(chainFinalEndPwm)      // floor, not 0
                                 .tShape()
                                 .async();
                        }
                        else
                        {
                            Drive.driveDist(dispatchCm)    // ADR-036 pivot-trimmed (LOCAL)
                                 .setMode(DriveMode::GYRO)
                                 .setHeading(imuTarget)
                                 .setPower(cruisePwm)
                                 .setStartSpeed(signedStartSpeed)    // ADR-023 seed
                                 .tShape()
                                 .async();
                        }
                        break;
                    }

                    case Routes::BlindMode::RAW_FLAT:
                    {
                        // RAW_FLAT pins start=end=cruise for a flat open-loop profile (the "FLAT"
                        // in the name); `rawStart` (hoisted above) covers the pass-through case.
                        // Non-pass-through MUST keep start = cruisePwm — anything else
                        // introduces a ramp and breaks the FLAT contract for callers that picked
                        // RAW_FLAT precisely to avoid a ramp. See ADR-023 and issue #19.
                        // ADR-059: RAW_FLAT is chain-ineligible (no ramps to chain), so this
                        // branch is always a plain per-hop dispatch.
                        Drive.driveDist(dispatchCm)    // ADR-036 pivot-trimmed (LOCAL)
                             .setMode(DriveMode::RAW)
                             .setPower(cruisePwm)
                             .setStartSpeed(rawStart)
                             .setEndSpeed(cruisePwm)
                             .async();
                        break;
                    }

                    case Routes::BlindMode::SYNC_TSHAPE:
                    case Routes::BlindMode::DEFAULT:    // defensive — see comment above
                    default:
                    {
                        if (chainMid)
                        {
                            Drive.driveDist(dispatchCm)    // carry-adjusted (AC9)
                                 .setMode(DriveMode::SYNC)
                                 .setPower(cruisePwm)
                                 .setStartSpeed(signedStartSpeed)    // ChainHopSpeed.startPwm
                                 .setEndSpeed(chainEndPwm)           // ChainHopSpeed.endPwm
                                 .setBrake(false)                    // never brake mid-run (AC3)
                                 .tShape()
                                 .async();
                        }
                        else if (chainFinal)
                        {
                            // ADR-059 chain-FINAL: braked stop shedding cruise->FLOOR
                            // (brake completes floor->rest) — see `chainFinalEndPwm`.
                            Drive.driveDist(dispatchCm)    // carry-adjusted (AC9)
                                 .setMode(DriveMode::SYNC)
                                 .setPower(cruisePwm)
                                 .setStartSpeed(signedStartSpeed)    // ChainHopSpeed.startPwm
                                 .setEndSpeed(chainFinalEndPwm)      // floor, not 0
                                 .tShape()
                                 .async();
                        }
                        else
                        {
                            Drive.driveDist(dispatchCm)    // ADR-036 pivot-trimmed (LOCAL)
                                 .setMode(DriveMode::SYNC)
                                 .setPower(cruisePwm)
                                 .setStartSpeed(signedStartSpeed)
                                 .tShape()
                                 .async();
                        }
                        break;
                    }
                }
            }
#ifdef PT_PARITY_TRACE
            {
                // Wire fields (strict ADR-050 schema — see PathTrackerParityTrace.h's
                // "ADR-059 wire schema" block): start_pwm/end_pwm are the RESOLVED
                // velocity-PLAN endpoints, so the sim (carry-free) emits identical
                // values; dist_cm is the carry-adjusted COMMANDED distance (== the
                // planned distance whenever the measured carry is 0, as in the parity
                // harness). A braked hop (incl. the chain FINAL) records end_pwm = 0 —
                // the plan endpoint is rest; the chain-final dispatch pinning its tShape
                // end target to the floor (`chainFinalEndPwm`) is a DISPATCH detail,
                // deliberately NOT recorded here.
                int16_t emitStartPwm = signedStartSpeed;
                int16_t emitEndPwm   = chainMid ? chainEndPwm : (int16_t) 0;
                if (decision.driveMode == Routes::BlindMode::RAW_FLAT)
                {
                    emitStartPwm = rawStart;
                    emitEndPwm   = cruisePwm;
                }
                _ptEmitBlindDeadReckon(decision.reverse, decision.driveMode, dispatchCm,
                                       emitStartPwm, emitEndPwm, chainHop);
            }
#endif
            // ADR-059 run bookkeeping: snapshot the boundary baselines the NEXT hop's
            // measured-carry subtraction reads (encoder sum + the distance just
            // commanded), then retire the run when the hop just dispatched is its FINAL
            // (braked) hop — BLIND_FORWARD's pass emit keys on `_chain.active`, so the
            // final arrival correctly emits no `blind_pass`, and a fresh chain may
            // re-form on the far side.
            if (chainHop)
            {
                _chain.dispatchEncSum = Drive.getEncL() + Drive.getEncR();
                _chain.dispatchCmdCm  = dispatchCm;
                if ((uint8_t) (_chain.hopIndex + 1) >= _chain.hopCount)
                {
                    _chainReset();
                }
            }
            _state       = State::BLIND_FORWARD;
            _passThrough = false;    // consumed
            break;
        }
    }
}

// =========================================================================================
// goTo() public surface
// =========================================================================================

PathTrackerGoToBuilder PathTrackerClass::goTo(uint8_t goalId)
{
    return PathTrackerGoToBuilder(*this, goalId);
}

PathTrackerGoToBuilder PathTrackerClass::goTo(const char* goalName)
{
    const uint8_t id = FieldMap::findNodeByName(goalName);
    return PathTrackerGoToBuilder(*this, id);
}
