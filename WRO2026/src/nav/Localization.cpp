/**
 * @file Localization.cpp
 * @brief Implementation of the `Localization` singleton (see Localization.h).
 * @author WRO 2026
 */

#include "Localization.h"

#include <MatrixMiniR4.h>
#include <math.h>

#include "../drive/Drive.h"
#include "NavTypes.h"
#include "../util/WROMath.h"
#include "YawSource.h"
#include "routes.h"

LocalizationClass Localization;    // single global instance

bool LocalizationClass::begin()
{
    _currentNode  = Routes::START_NODE;
    _yawOffsetDeg = 0.0f;
    _zeroed       = false;
    // Re-seed the heading cache so a re-begin / re-zero between bench runs does not serve
    // the prior run's last heading. The stamp resets to 0 ("no live read yet") so a
    // staleness check treats the seed value as not-yet-refreshed.
    _lastGoodHeadingDeg = (float) Routes::START_HEADING_DEG;
    _lastGoodHeadingMs  = 0;
    // Reset the operator-offset integrator so a re-begin between bench runs starts from a clean
    // frame. Without this, getOffsetFromNode() would serve the prior run's displacement until the
    // next arrival, and a leaked _navActive == true would silently suppress all integration.
    // begin() takes NO IMU/encoder read (the contract forbids touching the IMU before zeroing) —
    // the baselines reset to their defaults here, and the first update() under the closed gate
    // (!_zeroed) re-snapshots them to live readings before any accumulation can occur.
    _offset      = Offset2D{};
    _lastEncL    = 0;
    _lastEncR    = 0;
    _lastYawDeg  = 0.0f;
    _navActive   = false;
    _gateWasOpen = false;
    return true;
}

bool LocalizationClass::_readImuYaw(float& yawOut)
{
    // Single-owner IMU read (ADR-045): YawSource performs the retry-once lower-MCU read
    // AND applies Config::IMU_YAW_SIGN, returning the corrected app-frame (CCW-positive) yaw.
    // On a hard failure it leaves its out-param untouched and returns false; mirror that here by
    // NOT writing yawOut unless the read succeeded. The world-frame math below this layer is
    // unchanged — it now just operates on the corrected yaw.
    double corrected = 0.0;
    if (!YawSource.readYawDeg(corrected))
    {
        return false;
    }
    yawOut = (float) corrected;
    return true;
}

bool LocalizationClass::zeroHeadingFromCurrent()
{
    // Read the IMU yaw NOW through the result-surfacing path. The value can lie anywhere in
    // (-360, +360) depending on the boot orientation and any accumulated rotation since
    // `MiniR4.begin()` ran.
    float imuYaw = 0.0f;
    if (!_readImuYaw(imuYaw))
    {
        // Transient lower-MCU timeout: do NOT bind a corrupt frame. Leave `_yawOffsetDeg`
        // and `_zeroed` untouched and report failure so the .ino BTN_UP retry path engages.
        return false;
    }

    // World-frame definition: world = normalize(imu - offset). At the zero moment we
    // want world == Routes::START_HEADING_DEG, so offset = imu - START_HEADING_DEG.
    _yawOffsetDeg = imuYaw - (float) Routes::START_HEADING_DEG;
    _zeroed       = true;

    // Re-zeroing rebinds the world frame, so any offset accumulated under the OLD frame is stale and
    // its rotation baseline (_lastYawDeg) now references a different yaw origin. Drop the offset and
    // force the gate-reopen edge — same reset begin() performs — so the next update() re-baselines
    // from a fresh live read in the NEW frame before it integrates. Without this, an idle re-zero or a
    // MISSION_ALLOW_REARM re-zero (gate already open, _gateWasOpen still true) would skip the reopen
    // edge and integrate the frame discontinuity against the stale baselines, reporting the prior
    // run's displacement instead of starting from zero. Only reached on a successful read — a failed
    // read returned above WITHOUT mutating any state, so the offset is never cleared on a no-op zero.
    _offset      = Offset2D{};
    _gateWasOpen = false;
    return true;
}

bool LocalizationClass::isZeroed() const
{
    return _zeroed;
}

uint8_t LocalizationClass::getCurrentNode() const
{
    return _currentNode;
}

float LocalizationClass::coordOnAxis(uint8_t nodeId, uint8_t axis) const
{
    // The single firmware-side home for the ResetNode axis encoding (0 = x, 1 = y — the T13
    // cross-task contract the emitter and sim mirror). Any non-1 axis falls back to x; the
    // caller passes a validated `info->axis`, so the fallback is defence-in-depth.
    const Routes::NodeInfo& node = FieldMap::getNode(nodeId);
    return (axis == 1) ? node.y_cm : node.x_cm;
}

float LocalizationClass::getHeadingDeg() const
{
    if (!_zeroed)
    {
        // Defensive: callers should not consult heading before zeroing, but if they
        // do, return the start heading rather than a garbage offset-relative value.
        return Routes::START_HEADING_DEG;
    }
    float heading = 0.0f;
    if (!_readWorldHeading(heading))
    {
        // Transient lower-MCU timeout: a 0.0-derived heading would be wrong by up to a
        // full revolution and can flip a turn decision. Serve the last good heading
        // instead (the cache is not refreshed on a failed read, so it stays valid).
        return _lastGoodHeadingDeg;
    }
    return heading;
}

bool LocalizationClass::_readWorldHeading(float& headingOut) const
{
    // Live IMU read -> world heading, refreshing the last-good cache + stamp ONLY on success.
    // Returns false on a transient lower-MCU failure WITHOUT touching headingOut or the cache, so a
    // caller can choose: serve the last-good value (getHeadingDeg) or skip the tick entirely
    // (_integrateOffset — the offset has no correction path, so a deposit at a stale heading is a
    // permanent error). Caller holds _zeroed; the world frame must be bound.
    float imuYaw = 0.0f;
    if (!_readImuYaw(imuYaw))
    {
        return false;
    }
    headingOut          = _normalize(imuYaw - _yawOffsetDeg);
    _lastGoodHeadingDeg = headingOut;
    _lastGoodHeadingMs  = millis();
    return true;
}

float LocalizationClass::getLastHeadingDeg() const
{
    // No I/O: return the value cached by the most recent successful getHeadingDeg() read.
    return _lastGoodHeadingDeg;
}

uint32_t LocalizationClass::getLastHeadingUpdateMs() const
{
    // No I/O: timestamp of the last OK getHeadingDeg() read (0 if none since begin()).
    return _lastGoodHeadingMs;
}

int16_t LocalizationClass::toImuYaw(float worldHeadingDeg) const
{
    const float w        = _normalize(worldHeadingDeg);
    const float imuFloat = w + _yawOffsetDeg;
    // Drive's `_normalizeYaw(int32_t)` further reduces to (-360, +360). Round to nearest
    // int16 here. roundf is closest-to-zero for half-values; that's fine for our use,
    // which never hits half-degree targets in practice (the toy fixture uses 45-deg
    // multiples; the real field is integer headings).
    const long rounded = lroundf(imuFloat);
    return (int16_t) rounded;
}

void LocalizationClass::onIntersection(uint8_t advanceTo)
{
    _currentNode = advanceTo;
    // Arriving on a node re-anchors the operator-offset frame. The baseline re-snapshot inside
    // clearOffset() is what wins the arrival-tick race: on the single tick where Drive finishes
    // the approach and PathTracker re-anchors, the very next update() integrates a zero delta, so
    // getOffsetFromNode() reads ~0 instead of a phantom equal to the final approach motion.
    clearOffset();
}

void LocalizationClass::onDeadReckonedAdvance(uint8_t advanceTo)
{
    _currentNode = advanceTo;
    // Same re-anchor as onIntersection — a dead-reckoned advance lands on a node too, so the
    // offset frame resets to zero with the baselines re-snapshotted (kills the arrival phantom).
    clearOffset();
}

void LocalizationClass::declarePosition(uint8_t nodeId)
{
    // Operator-recovery counterpart to onIntersection / onDeadReckonedAdvance: the same
    // unconditional low-level write (set node + re-anchor offset) but operator-initiated
    // rather than triggered by a sensed crossing. The heading offset is deliberately left
    // alone — the operator is only declaring graph position and zeroing the offset.
    //
    // Layering: this NEVER calls up into PathTracker (no clearFault here). The idle-OR-faulted
    // guard and the fault-clear live in the Mission layer above, which can reach both modules.
    _currentNode = nodeId;
    clearOffset();
}

void LocalizationClass::_bindHeadingFromYaw(float imuYaw, float headingDeg)
{
    // Single source of truth for the offset-bind invariant, shared by reanchorAtWall step (1) and
    // the heading-only reanchorHeading. Bind the offset so world == normalize(imu - offset) ==
    // headingDeg, then refresh the heading cache + stamp. The cache refresh is REQUIRED (not
    // cosmetic): a no-I/O getLastHeadingDeg() and the next _consumeOffset() — which baselines
    // _lastYawDeg off the cached heading — must see the POST-bind heading, never the stale pre-bind
    // value. Writes _yawOffsetDeg + the heading cache ONLY; never _offset or the integrator
    // baselines (the caller owns any offset re-anchor).
    _yawOffsetDeg       = imuYaw - headingDeg;
    _lastGoodHeadingDeg = _normalize(imuYaw - _yawOffsetDeg);
    _lastGoodHeadingMs  = millis();
}

bool LocalizationClass::reanchorAtWall(uint8_t axis, float wallCoord, float headingDeg)
{
    // --- Read the IMU ONCE up front (pre-mutation) --------------------------------------------
    // One raw-yaw sample backs BOTH the strict check (live heading vs. expected normal) and the
    // heading re-bind, so the two can never disagree across a second read. A failed read aborts
    // with NO mutation: re-binding _yawOffsetDeg against a stale yaw would skew every later
    // getHeadingDeg() by the staleness, which is worse than declining the re-anchor and letting
    // the caller fault.
    float imuYaw = 0.0f;
    if (!_readImuYaw(imuYaw))
    {
        return false;
    }

    // --- Strict wall-normal sanity check (default OFF) ----------------------------------------
    // When enabled, refuse to re-anchor if the chassis is not actually squared to the wall: the
    // current world heading must be within tolerance of the expected wall normal. Computed and
    // decided BEFORE any state write so a rejection leaves the pose byte-for-byte unchanged
    // (atomicity). Reuse wrapDeltaDeg for the wrap-around-correct signed difference rather
    // than a naive abs(a - b), which breaks across the 0/360 seam.
    if (Config::WALL_NORMAL_STRICT_ENABLED)
    {
        const float currentHeading = _normalize(imuYaw - _yawOffsetDeg);
        const float diff           = fabsf(wrapDeltaDeg(headingDeg - currentHeading));
        // Boundary: `> tolerance` REJECTS, so a difference exactly equal to the tolerance still
        // re-anchors ("just inside" is inclusive). Matches the Config docstring + AC11 boundary.
        if (diff > Config::WALL_NORMAL_TOLERANCE_DEG)
        {
            return false;
        }
    }

    // === Atomic re-anchor: no observation point from here to the end ==========================

    // (1) Square heading: bind the offset so getHeadingDeg() == headingDeg. Same definition as
    //     zeroHeadingFromCurrent (world = normalize(imu - offset)), but targeting the wall normal
    //     instead of the start heading. Also seeds the heading cache so a no-I/O getLastHeadingDeg()
    //     immediately reflects the squared heading instead of the pre-square value. The offset-bind
    //     + cache-refresh invariant lives in ONE place (_bindHeadingFromYaw), shared with the
    //     heading-only reanchorHeading; this call is behaviour-identical to the former inline lines.
    _bindHeadingFromYaw(imuYaw, headingDeg);

    // (2) Re-anchor the operator-offset frame to zero AND re-snapshot all three integrator
    //     baselines to the CURRENT readings (kills the arrival-tick phantom — same discipline as
    //     clearOffset / onIntersection). Done BEFORE writing the axis offset below: _consumeOffset
    //     zeroes the whole _offset, so the axis write must follow it, not precede it.
    _consumeOffset();

    // (3) Set the on-axis belief to the wall coordinate PULLED BACK by the rear-contact margin.
    //     The chassis presses its REAR face flat against the wall, but the tracked reference is the
    //     wheel AXLE, which sits Config::WALL_CONTACT_OFFSET_CM short of that contact face — on the
    //     NODE side of the wall. So the believed on-axis coordinate is the wall coordinate moved
    //     back toward the node by that margin, not the wall surface itself (the old belief ==
    //     wallCoord was optimistic by exactly this offset). Position is _currentNode + _offset (no
    //     stored absolute pose), so express it as an offset from the anchored node:
    //     _offset[axis] = (wallCoord - sign(gap)*offset) - nodeCoord, where gap = wallCoord -
    //     nodeCoord carries which side of the node the wall is on (so the axle lands on the node
    //     side regardless). The off-axis component and thetaDeg stay 0 from step (2) — the wall
    //     gives no off-axis info and heading was just re-bound, so the delta channel resets.
    const float nodeCoord  = coordOnAxis(_currentNode, axis);    // x=0 / y=1 decode (one home)
    const float gap        = wallCoord - nodeCoord;              // signed: which side the wall is on
    const float signGap    = (gap >= 0.0f) ? 1.0f : -1.0f;
    // Net belief on-axis == wallCoord - signGap*WALL_CONTACT_OFFSET_CM (axle on the node side).
    const float axleOffset = gap - signGap * Config::WALL_CONTACT_OFFSET_CM;
    if (axis == 0)
    {
        _offset.x = axleOffset;
    }
    else if (axis == 1)
    {
        _offset.y = axleOffset;
    }
    // Any other axis value: leave _offset at the zeroed state from step (2). The heading is still
    // squared (the wall normal is axis-independent). The caller passes a validated info->axis, so
    // this branch is defence-in-depth, not an expected path.

    return true;
}

bool LocalizationClass::reanchorHeading(float headingDeg, float* outPreBindDeltaDeg)
{
    // Heading-only re-anchor (ADR-052). Full contract in the header. Every guard below returns
    // false having mutated NOTHING; the bind at the end is the only state write, and it touches
    // _yawOffsetDeg + the heading cache ONLY — never _offset / the integrator baselines.

    // (1) Closed-gate invariant: only ever run while PathTracker is executing a hop
    //     (_navActive == true), so the operator-offset integrate gate is CLOSED and the later
    //     closed->open edge re-baselines from a live read in the NEW frame. Rebinding the world
    //     frame while the gate is OPEN (idle) would leave _gateWasOpen true, skip the reopen edge,
    //     and deposit the yaw discontinuity into the reported offset on the next update()
    //     (gotcha-localization-rezero-reopen-edge). Hard reject, not a warning — and this is why we
    //     must never clear _offset / _gateWasOpen here (that would MASK a future call-order bug).
    if (!_navActive)
    {
        return false;
    }

    // (2) World frame not yet bound: nothing to re-anchor against.
    if (!_zeroed)
    {
        return false;
    }

    // (3) Read the IMU ONCE. A failed read aborts with NO mutation and leaves outPreBindDeltaDeg
    //     UNTOUCHED (no read = no delta) — binding against a stale yaw would skew every later
    //     getHeadingDeg().
    float imuYaw = 0.0f;
    if (!_readImuYaw(imuYaw))
    {
        return false;
    }

    // (4) Pre-bind belief and wrap-correct delta. Reuse wrapDeltaDeg for the 0/360-seam-safe signed
    //     difference (a naive abs(a - b) breaks across the seam) — the SAME pattern reanchorAtWall
    //     uses for its strict check.
    const float belief = _normalize(imuYaw - _yawOffsetDeg);
    const float delta  = fabsf(wrapDeltaDeg(headingDeg - belief));

    // (5) Report the delta on read success — BEFORE the strict decision — so a strict-rejected call
    //     still surfaces its delta (PathTracker's skip log wants it). Only a FAILED read leaves the
    //     out-param untouched, which was handled by the early return above.
    if (outPreBindDeltaDeg != nullptr)
    {
        *outPreBindDeltaDeg = delta;
    }

    // (6) Strict gate (LINE-reset-specific knobs, NOT the wall ones): reject a gross-wrong-state
    //     sample. Boundary is inclusive — `> tolerance` rejects, a delta exactly equal to the
    //     tolerance still binds (matches reanchorAtWall). A rejection mutates NOTHING.
    if (Config::LINE_HEADING_RESET_STRICT_ENABLED &&
        delta > Config::LINE_HEADING_RESET_TOLERANCE_DEG)
    {
        return false;
    }

    // (7) Bind heading only via the shared core. No _offset / baseline touch — the arrival hook
    //     already zeroed _offset one call earlier, and this is a heading-only verb.
    _bindHeadingFromYaw(imuYaw, headingDeg);
    return true;
}

void LocalizationClass::update()
{
    // Two conditions open the integrate gate: PathTracker is not driving a hop (the operator may
    // be hand-driving) AND the world frame is bound (heading is meaningless before zeroing).
    const bool gateOpen = (!_navActive) && _zeroed;
    if (gateOpen)
    {
        if (!_gateWasOpen)
        {
            // Gate-reopen EDGE: re-baseline from a SINGLE live read so the first integrated delta
            // is ~0 (discards any motion that happened while the gate was closed). This is the only
            // heading read on the closed->open boundary — the closed branch no longer reads the IMU
            // every tick, so a multi-second mission no longer issues a blocking lower-MCU read per
            // cooperative tick on the hot path (and the EventTrace staleness flag works again). If
            // the read fails, leave the gate "not yet reopened" and retry next tick rather than
            // baselining off a stale heading, which would deposit the whole closed-period rotation
            // as a phantom on the first integrate.
            float heading = 0.0f;
            if (_readWorldHeading(heading))
            {
                _snapshotBaselines(heading);
                _gateWasOpen = true;
            }
            return;
        }
        _integrateOffset();
        return;
    }

    // Gate closed (PathTracker driving a hop, or pre-zero): do NOT accumulate and do NOT read the
    // IMU. Baselines are re-established at the reopen edge above, so there is no per-tick read here.
    _gateWasOpen = false;
}

Offset2D LocalizationClass::getOffsetFromNode() const
{
    // Read-only telemetry: no I/O, no mutation — just hand back the accumulated offset.
    return _offset;
}

void LocalizationClass::clearOffset()
{
    // Thin wrapper over the re-anchor helper (naming per the plan: integrate vs consume).
    _consumeOffset();
}

void LocalizationClass::setNavActive(bool active)
{
    _navActive = active;
}

void LocalizationClass::_integrateOffset()
{
    // Read the live world heading ONCE; both channels reference the same reading so the translation
    // decomposition and the rotation delta are consistent for this tick. On a transient lower-MCU
    // failure SKIP the whole tick rather than decomposing motion onto a stale heading: x/y has no
    // correction path, so a single deposit at the wrong angle is a permanent error. Leaving the
    // baselines unadvanced carries both channels' deltas across to the next good-read tick (the same
    // carry-across the sub-deadband branches use), so the motion is deferred, not lost.
    float heading = 0.0f;
    if (!_readWorldHeading(heading))
    {
        return;
    }

    // ---- Translation channel (gated independently of rotation) -------------------------------
    const int32_t encL = Drive.getEncL();
    const int32_t encR = Drive.getEncR();
    // Average the two wheel deltas in CENTIMETRES (not raw encoder degrees) so a veer with an odd
    // encoder-degree sum does not lose ~0.5 enc-deg to integer truncation on every crossing. A
    // pure on-the-spot turn — wheels equal-and-opposite — still nets ~zero. encDegToCm carries
    // the sign through.
    const float dCm = (encDegToCm(encL - _lastEncL) + encDegToCm(encR - _lastEncR)) * 0.5f;
    if (fabsf(dCm) >= Config::OFFSET_TRANSLATION_DEADBAND_CM)
    {
        // Above the deadband: decompose along the current world heading and advance the baselines.
        // Use the Arduino `PI` macro (defined by Arduino.h and the host mock) rather than M_PI,
        // which the host test mock does not provide — matching the existing WROMath.h convention.
        const float headingRad = heading * ((float) PI / 180.0f);
        _offset.x += dCm * cosf(headingRad);
        _offset.y += dCm * sinf(headingRad);
        _lastEncL = encL;
        _lastEncR = encR;
    }
    // Below the deadband: leave _offset AND the encoder baselines UNADVANCED, so slow sub-deadband
    // motion still accumulates across ticks until it crosses the floor. A stationary chassis never
    // crosses it (dCm stays ~0 forever), satisfying the no-phantom-drift requirement.

    // ---- Rotation channel (gated independently of translation) -------------------------------
    // wrapDeltaDeg (NavTypes.h) is the single source of truth for folding a heading difference
    // into (-180, 180]; reuse it instead of a private fmodf copy so the rotation channel matches
    // the convention used by PathTracker / DirectionPolicy. `+` is CCW, so a 359 -> 1 step yields
    // +2, not -358.
    const float dYaw = wrapDeltaDeg(heading - _lastYawDeg);
    if (fabsf(dYaw) >= Config::OFFSET_ROTATION_DEADBAND_DEG)
    {
        // A pure manual turnDeg nets ~zero translation but a real yaw change, so the rotation
        // channel must update thetaDeg even when the translation channel skipped this tick.
        _offset.thetaDeg += dYaw;
        _lastYawDeg = heading;
    }
    // Below the deadband: leave thetaDeg and the yaw baseline UNADVANCED (same sub-deadband
    // carry-across rationale as the translation channel).
}

void LocalizationClass::_consumeOffset()
{
    // Re-anchor: zero the offset and re-snapshot all three baselines so the next update() integrates
    // a zero delta. This is the arrival-tick phantom kill. The node-arrival hooks run on the hot FSM
    // path (PathTracker::_stepFsm, right after Drive active-brakes), so anchor the yaw baseline to
    // the CACHED heading (getLastHeadingDeg) — no blocking IMU read here. Correctness holds because
    // arrival always happens with the gate closed (navActive while a hop runs); the gate-reopen edge
    // re-baselines from a live read before any integrate consumes this baseline.
    _offset = Offset2D{};
    _snapshotBaselines(getLastHeadingDeg());
}

void LocalizationClass::_snapshotBaselines(float headingDeg)
{
    // Re-anchor all three integrator baselines to "now": current encoders + the caller-supplied
    // world heading. The heading is passed in (not read here) so each caller picks its source — the
    // gate-reopen edge passes a LIVE read (a fresh baseline), the arrival hook passes the CACHED
    // heading (no blocking read on the FSM hot path). The next update() then integrates a ~0 delta.
    _lastEncL   = Drive.getEncL();
    _lastEncR   = Drive.getEncR();
    _lastYawDeg = headingDeg;
}

float LocalizationClass::_normalize(float deg)
{
    // Handle arbitrary inputs: shift into [0, 360) by repeatedly adding/subtracting 360.
    // fmodf can return negative results for negative inputs, so the explicit guard
    // afterward ensures the final value is in the half-open interval.
    float r = fmodf(deg, 360.0f);
    if (r < 0.0f)
    {
        r += 360.0f;
    }
    // r is now in [0, 360]. Edge case: r == 360.0 exactly snaps back to 0.
    if (r >= 360.0f)
    {
        r -= 360.0f;
    }
    return r;
}
