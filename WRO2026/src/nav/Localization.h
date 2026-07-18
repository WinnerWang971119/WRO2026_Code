/**
 * @file Localization.h
 * @brief Single source of truth for `currentNode` and the world-frame yaw offset.
 * @author WRO 2026
 *
 * @details
 * `LocalizationClass` is the singleton that holds the robot's discrete position on the
 * field graph (`currentNode`, an id into `Routes::NODES`) and the runtime offset that
 * bridges Routes:: world-frame headings (CCW positive, `0 deg = +x` East, range
 * `[0, 360)`) to the IMU's local yaw frame.
 *
 * Single-writer rule (per ARCHITECTURE.md):
 *  - `_currentNode` is written ONLY by this class. PathTracker calls `onIntersection(...)`
 *    or `onDeadReckonedAdvance(...)` to advance the node id after a successful edge.
 *  - `_yawOffsetDeg` is written ONLY by `LocalizationClass` methods —
 *    `zeroHeadingFromCurrent()` (start-button frame bind) and `reanchorAtWall()` (wall-zero
 *    re-square). PathTracker / Drive NEVER poke heading state directly: every turn dispatches
 *    `Drive.turnToYaw(...)` against the IMU as the source of truth, so cumulative-relative
 *    drift is never accumulated. The write SET grew for the wall-reset feature (PR2), but the
 *    writes stay inside this class so the single-writer discipline holds (PR2 ADR T20
 *    supersedes ADR-005's narrower "only zeroHeadingFromCurrent" wording).
 *
 * Operator-relative reference frame (key convention):
 *  - The yaw offset is captured at the **start-button press**, not at `begin()`. The
 *    chassis is rarely at the right pose when `MiniR4.begin()` runs, so a `begin()`-time
 *    snapshot would bind the wrong frame. Power-on flow:
 *      1. `setup()` runs `Localization.begin()` -> `_zeroed = false`.
 *      2. Operator places the robot on `Routes::START_NODE` facing `START_HEADING_DEG`.
 *      3. Operator presses BTN_UP -> `.ino` calls `zeroHeadingFromCurrent()` -> reads the
 *         IMU yaw NOW, computes the offset, sets `_zeroed = true`.
 *      4. Subsequent goTo() calls succeed; goTo() before zero faults loud.
 *
 * Lifecycle:
 *  - `begin()` once in `setup()` after every `MiniR4.begin()` and the Phase 1 modules'
 *    `begin()`. The order is `Drive` -> `LineFollower` -> `Intersection` -> `Manipulator`
 *    -> `Localization` -> `PathTracker` -> `EventTrace`.
 *  - `update()` runs once per cooperative tick (after `PathTracker.update()`, so encoders are
 *    fresh and the nav-active flag is current). It integrates raw operator motion into a 2D
 *    offset-from-node while the gate is open (`!navActive && isZeroed()`); while the gate is
 *    closed it neither accumulates nor reads the IMU, re-baselining only on the closed->open edge.
 *    `getHeadingDeg()` reads the IMU live; on a transient lower-MCU read failure it falls back to
 *    the last good heading instead of a `0.0`-derived garbage value (see `getHeadingDeg`).
 *  - `getLastHeadingDeg()` returns that cached value with NO I/O — use it on telemetry /
 *    non-blocking paths that must not issue a bus transaction.
 */

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <Arduino.h>
#include <stdint.h>

#include "FieldMap.h"
#include "routes.h"

/**
 * @brief Read-only telemetry pose offset from the last anchored node.
 *
 * A small POD carried by value out of `LocalizationClass::getOffsetFromNode()`. It
 * accumulates how far the chassis has been hand-driven away from the node it was last
 * anchored to (`x`/`y` in centimetres along the world frame, `thetaDeg` a signed
 * heading delta in degrees). All three fields are `0` while the chassis sits on its
 * anchored node — a node arrival re-anchors and zeroes the offset.
 *
 * @note `thetaDeg` is a delta-from-anchor, NOT an authoritative heading. The single
 *       source of truth for heading stays live-from-IMU via `getHeadingDeg()`; this
 *       value exists only as recovery telemetry (ADR-005 single-writer rule).
 */
struct Offset2D
{
    float x        = 0.0f;    ///< World-frame +x displacement from the anchored node (cm).
    float y        = 0.0f;    ///< World-frame +y displacement from the anchored node (cm).
    float thetaDeg = 0.0f;    ///< Signed heading delta from the anchored node (degrees).
};

/**
 * @brief Singleton that holds the discrete current node + IMU-frame yaw offset.
 *
 * Heading is treated as a derived quantity — `getHeadingDeg()` reads the IMU live and
 * returns the world-frame value via `normalize(IMU_yaw - _yawOffsetDeg)`. Every OK read
 * also refreshes `_lastGoodHeadingDeg`; on a non-OK lower-MCU read `getHeadingDeg()`
 * returns that cached value rather than a `0.0`-derived garbage heading, and
 * `getLastHeadingDeg()` exposes the same cache for callers that must not issue I/O.
 */
class LocalizationClass
{
public:
    /**
     * @brief Seed `_currentNode = Routes::START_NODE`; clear `_zeroed`, the offset, and the
     *        integrator baselines.
     *
     * Must be called from `setup()` after every Phase 1 module's `begin()`. Does NOT
     * touch the IMU — `_yawOffsetDeg` stays `0.0f` until `zeroHeadingFromCurrent()` runs.
     * Also resets the operator-offset integrator (`_offset`, the encoder/yaw baselines, and the
     * `_navActive` gate) so a re-begin between bench runs starts from a clean frame rather than
     * serving the prior run's displacement or inheriting a stuck nav gate.
     *
     * ALSO the arm-time frame-rebind: the `.ino` BTN_UP handler calls this immediately before
     * `zeroHeadingFromCurrent()` on every arm. That pairing is the ONLY way `_currentNode` returns
     * to `Routes::START_NODE` for a `Config::MISSION_ALLOW_REARM` re-arm — `zeroHeadingFromCurrent()`
     * binds heading alone, so without this the re-armed run plans from the PREVIOUS run's final
     * node and `PathTracker` fake-arrives on a goal it believes it is already standing on. Call the
     * two together: this clears `_zeroed`, so `begin()` without a following zero leaves the chassis
     * unable to `goTo` (faults `NOT_ZEROED`).
     *
     * @return `true` always (defensive — kept for symmetry with the other modules' bool
     *         return convention so `.ino` can collect a single `_setupFailed` flag).
     */
    bool begin();

    /**
     * @brief Capture the IMU yaw NOW and bind the world-frame offset.
     *
     * Reads the corrected (app CCW-positive) IMU yaw through `YawSource` — the single owner of
    * the lower-MCU read, which applies `Config::IMU_YAW_SIGN` (ADR-045). On an OK read computes
     * `_yawOffsetDeg = imu_yaw - Routes::START_HEADING_DEG` and sets
     * `_zeroed = true`. Must be called once on the start-button press AFTER the operator has
     * physically placed the chassis on `Routes::START_NODE` facing the start heading.
     *
     * @return `true` on a successful read. Returns `false` on a non-OK IMU read (transient
     *         STM32 comms timeout) WITHOUT touching `_yawOffsetDeg` or `_zeroed`, so the
     *         `.ino` BTN_UP handler's retry path engages instead of latching a corrupt
     *         world-frame offset.
     * @note Calling this a second time is allowed — it re-binds the offset to whatever
     *       the chassis is currently pointing at. Useful for re-zeroing during testing
     *       between bench runs.
     */
    bool zeroHeadingFromCurrent();

    /**
     * @brief Whether `zeroHeadingFromCurrent()` has been called at least once.
     *
     * @return `true` once the operator has zero'd; `false` otherwise.
     * @note `PathTrackerClass::goTo(...)`'s builder destructor checks this and faults
     *       `Fault::NOT_ZEROED` on a `goTo` issued before zeroing.
     */
    bool isZeroed() const;

    /**
     * @brief Read the current node id (single source of truth).
     *
     * @return Node id in `[0, FieldMap::N_NODES)`. Initialised to `Routes::START_NODE`
     *         by `begin()`.
     */
    uint8_t getCurrentNode() const;

    /**
     * @brief Read a node's stored coordinate on one field axis (the x=0 / y=1 decode).
     *
     * Single home for the load-bearing `ResetNode::axis` encoding (`0 = x`, `1 = y` — the T13
     * cross-task contract the emitter and sim mirror). Callers that need "the coordinate of a
     * node on the reset axis" (PathTracker's wall-push distance, `reanchorAtWall`'s offset write)
     * route through here instead of open-coding `(axis == 0) ? x_cm : y_cm`, so the decode lives
     * in exactly one place on the firmware side.
     *
     * @param nodeId Node id in `[0, FieldMap::N_NODES)`.
     * @param axis   `0 = x`, `1 = y`. Any other value returns the x coordinate (defensive — the
     *               caller passes a validated `info->axis`).
     * @return The node's `x_cm` (axis 0) or `y_cm` (axis 1), in centimetres.
     */
    float coordOnAxis(uint8_t nodeId, uint8_t axis) const;

    /**
     * @brief Read the current world-frame heading in degrees (live IMU read).
     *
     * Issues a single corrected IMU read through `YawSource` (applies `Config::IMU_YAW_SIGN`,
    * ADR-045). On an OK read computes
     * `normalize(imu_yaw - _yawOffsetDeg)` to `[0, 360)` and refreshes the last-good cache.
     * On a NON-OK read (transient STM32 comms timeout) the underlying out-param is left
     * untouched by the library, so this method returns the LAST GOOD heading instead of a
     * `0.0`-derived garbage value. Returns `Routes::START_HEADING_DEG` if `!isZeroed()`
     * (defensive — caller should check `isZeroed()` before relying on this).
     *
     * @return Heading in `[0, 360)` degrees, CCW positive, `0 deg = +x` (East).
     * @note Performs I/O. Telemetry / non-blocking paths must use `getLastHeadingDeg()`.
     */
    float getHeadingDeg() const;

    /**
     * @brief Read the cached world-frame heading WITHOUT issuing any I/O.
     *
     * Returns the value refreshed by the most recent successful `getHeadingDeg()` read, or
     * `Routes::START_HEADING_DEG` if no successful read has occurred yet. This accessor
     * never touches the IMU / I2C bus, so it is safe on paths whose contract forbids
     * blocking on hardware (e.g. the EventTrace heartbeat).
     *
     * @return Last good heading in `[0, 360)` degrees, CCW positive, `0 deg = +x` (East).
     */
    float getLastHeadingDeg() const;

    /**
     * @brief `millis()` of the last successful live heading read; `0` if none yet.
     *
     * Lets a no-I/O consumer distinguish a freshly-read heading from a stale cached one.
     * `getLastHeadingDeg()` keeps returning a plausible value even when the IMU has been
     * failing or no navigation code has read the heading recently, so a telemetry path
     * (e.g. the EventTrace heartbeat) compares `millis() - getLastHeadingUpdateMs()`
     * against its own staleness budget to flag the difference.
     *
     * @return `millis()` captured at the last OK `getHeadingDeg()` read; `0` if no OK read
     *         has happened since `begin()` (which resets it).
     */
    uint32_t getLastHeadingUpdateMs() const;

    /**
     * @brief Convert a world-frame heading to the corrected app-frame yaw value
     *        `Drive.turnToYaw` expects.
     *
     * Result = `normalize(worldHeadingDeg + _yawOffsetDeg)` rounded to `int16_t`. This is in
     * the corrected app (CCW-positive) frame — the SAME frame `YawSource` reads return, NOT the
     * raw STM32 frame. The custom turn loop compares it directly against corrected reads; a GYRO
     * straight-line write converts it to the raw frame via `YawSource::toHwHeading` at the call
    * site (ADR-045). The `Drive` layer further normalises to `(-360, +360)` via `_normalizeYaw`.
     *
     * @param worldHeadingDeg  World-frame heading in `[0, 360)`. Out-of-range values are
     *                         normalised before the offset is applied.
     * @return Corrected app-frame yaw target as an `int16_t` (a gyro-PID write applies
     *         `toHwHeading` first; never pass this raw to the STM32).
     */
    int16_t toImuYaw(float worldHeadingDeg) const;

    /**
     * @brief Edge-arrival hook called by PathTracker after a LINE edge intersection event.
     *
     * @param advanceTo  Node id to set as the new `_currentNode`.
     * @note Does not validate @p advanceTo — caller (PathTracker) has just resolved it
     *       from `FieldMap::getNextHop(...)` so it is by construction a valid node id.
     */
    void onIntersection(uint8_t advanceTo);

    /**
     * @brief Edge-arrival hook called by PathTracker after a PROJECTION or BLIND edge.
     *
     * @param advanceTo  Node id to set as the new `_currentNode`.
     * @note Semantically distinct from `onIntersection(...)` because no physical
     *       re-localisation event fired — this is dead-reckoned advance. Future
     *       calibration nodes (Phase 2.5) may differentiate behaviour here.
     */
    void onDeadReckonedAdvance(uint8_t advanceTo);

    /**
     * @brief Operator-recovery re-anchor: declare the chassis to be on @p nodeId
     *        with a zero offset.
     *
     * Sets `_currentNode = nodeId` and clears the accumulated offset (re-snapshotting
     * the integrator baselines exactly like `clearOffset()`), making this a true
     * re-anchor at the declared node with zero offset. It is the operator-initiated
     * counterpart to the sensed-arrival hooks (`onIntersection` /
     * `onDeadReckonedAdvance`): those advance the node after a physically sensed
     * crossing, whereas this is an explicit operator declaration used to recover from
     * an `OFF_NODE` fault.
     *
     * @param nodeId  Node id to anchor to. NOT heavily validated here (mirrors
     *                `onIntersection`, which trusts its caller) — the Mission layer
     *                resolves the name and bounds-checks the id before calling.
     * @note Performs the unconditional low-level write ONLY — it does NOT reach UP into
     *       `PathTracker` (no `PathTracker.clearFault()`), preserving the module
     *       layering. The guard and the fault-clear live in the Mission layer above.
     * @note The heading offset (`_yawOffsetDeg`) is intentionally left UNTOUCHED — the
     *       operator is declaring graph position and zeroing the offset, not re-zeroing
     *       heading. Re-zero heading via `zeroHeadingFromCurrent()` if needed.
     */
    void declarePosition(uint8_t nodeId);

    /**
     * @brief Wall-zero re-anchor: adopt the ground truth a `Drive.wallStop` just established.
     *
     * After the chassis presses its REAR against a known wall, the tracked wheel axle sits
     * `Config::WALL_CONTACT_OFFSET_CM` short of the wall on @p axis (the rear contact face is at
     * the wall; the axle is that margin in front of it, on the node side), and the chassis should
     * be squared to the wall normal. This re-binds Localization's belief to that truth in ONE
     * atomic update (the wall-reset half of PR2, called by PathTracker from a
     * `FieldMap::getResetInfo(node)` record). It corrects both the accumulated heading drift
     * (against the wall normal) and the on-axis position drift (against the axle's wall-relative
     * coordinate) at once.
     *
     * Pose-model choice: position is `_currentNode` + `_offset` (there is NO stored absolute
     * pose in this class), so the on-axis belief is expressed as an offset from the anchored node:
     * `_offset[axis] = (wallCoord - sign(gap)*WALL_CONTACT_OFFSET_CM) - nodeCoord[axis]`, where
     * `gap = wallCoord - nodeCoord` carries which side the wall is on (so the axle always lands on
     * the node side), read via `FieldMap::getNode(_currentNode)`. The reset node's stored
     * coordinate generally differs from its wall coordinate (e.g. node Wall1 sits at y=139 but its
     * wall is at y=154), so this offset is non-zero by design and the net belief
     * `nodeCoord[axis] + _offset[axis]` equals `wallCoord - sign(gap)*WALL_CONTACT_OFFSET_CM` (the
     * axle, NOT the wall surface). The off-axis offset component and `thetaDeg` are zeroed
     * (the wall gives no off-axis information; heading is re-bound below, so the delta-channel
     * resets), and the integrator baselines are re-snapshotted so the next `update()` tick
     * integrates a zero delta (no arrival-tick phantom). `_currentNode` is left as-is — the
     * caller already anchored it to the reset node on arrival.
     *
     * Atomicity: the strict check (when enabled) and the single IMU read happen BEFORE any
     * state write, and a failed IMU read or a rejected strict check returns `false` having
     * mutated NOTHING. On the success path all of `_yawOffsetDeg`, `_offset`, and the three
     * integrator baselines are written in one straight-line sequence with no intervening
     * observation point, so no consumer can witness a half-anchored pose.
     *
     * Post-condition on a `true` return: `getHeadingDeg() == @p headingDeg` (modulo the live
     * IMU read it re-derives), `nodeCoord[axis] + getOffsetFromNode()[axis] == @p wallCoord -
     * sign(gap)*Config::WALL_CONTACT_OFFSET_CM` (the axle, on the node side of the wall), the
     * off-axis offset and `thetaDeg` are `0`, and the next `update()` tick integrates a
     * zero delta. On a `false` return the pose is byte-for-byte unchanged.
     *
     * Strict wall-normal sanity check (`Config::WALL_NORMAL_STRICT_ENABLED`, default OFF):
     * when enabled, the live IMU heading is compared against @p headingDeg BEFORE any
     * mutation. If the wrap-around-correct absolute angular difference EXCEEDS
     * `Config::WALL_NORMAL_TOLERANCE_DEG` the squaring is untrustworthy and the method rejects
     * (returns `false`, no mutation). A difference exactly equal to the tolerance still
     * re-anchors (boundary is inclusive; `> tolerance` rejects). When the flag is OFF the
     * method always re-anchors (subject only to a successful IMU read).
     *
     * @param axis       Reset axis: `0 = x`, `1 = y` (the T13 `Routes::ResetNode::axis`
     *                   contract). Any other value is treated as a no-op axis write (the
     *                   heading is still re-bound) — the caller passes a validated `info->axis`.
     * @param wallCoord  Ground-truth wall-surface coordinate on @p axis (cm). The believed axle
     *                   coordinate is pulled back from it by `Config::WALL_CONTACT_OFFSET_CM`.
     * @param headingDeg Expected world-frame wall normal (degrees, `[0, 360)`) — the heading
     *                   to re-bind to.
     * @return `true` if the pose was re-anchored; `false` if the strict check rejected the
     *         re-anchor OR the IMU read failed (in both cases NO state was mutated). PathTracker
     *         (T16) maps a `false` to a localization fault; `Localization` does NOT raise the
     *         fault itself (it sits below PathTracker — layering).
     * @note Performs I/O (one live IMU read). One-shot synchronous update — no loops, no
     *       blocking beyond the IMU read's own bounded retry.
     */
    bool reanchorAtWall(uint8_t axis, float wallCoord, float headingDeg);

    /**
     * @brief Heading-only re-anchor: re-bind the world-frame yaw offset to a known heading.
     *
     * Instantaneous, motion-free heading re-zero used at a collinear line-follow pass-through to
     * bound IMU-yaw drift for free (PathTracker calls it with the incoming edge's known
     * `heading_deg`, body-corrected — ADR-052). One IMU read + offset math; no Drive motion, no
     * fault. Unlike `reanchorAtWall`, this is HEADING-ONLY: it re-binds `_yawOffsetDeg` and
     * refreshes the heading cache/stamp, and touches NOTHING else — not `_offset`, not the
     * integrator baselines (`_lastEncL`/`_lastEncR`/`_lastYawDeg`), not `_currentNode`. The arrival
     * hook already zeroed `_offset` one call earlier; a heading-only verb that also cleared it would
     * MASK a future call-order bug (`gotcha-localization-rezero-reopen-edge`).
     *
     * Frame-rebind / integrate-gate invariant: this MUST be called only while the operator-offset
     * integrate gate is CLOSED (`_navActive == true`), which the `!_navActive` reject ENFORCES.
     * Rebinding the world frame while the gate is OPEN (idle) would leave `_gateWasOpen == true`,
     * skip the closed->open re-baseline edge, and deposit the yaw discontinuity into the reported
     * offset on the next `update()`. Because it only ever runs under `_navActive`, the later
     * closed->open edge re-baselines correctly from a live read in the new frame. This is why
     * `!_navActive` is a hard reject, not a warning, and why this method must NEVER clear `_offset`
     * / `_gateWasOpen` (that would mask a future call-order bug).
     *
     * Strict gate is LINE-RESET-SPECIFIC: it reads `Config::LINE_HEADING_RESET_STRICT_ENABLED` /
     * `Config::LINE_HEADING_RESET_TOLERANCE_DEG` — the line-arrival knobs, NOT the wall-reset ones.
     * A future NON-line caller must NOT be wired through this method and silently inherit the line
     * tolerance; give it its own re-anchor surface + gate instead.
     *
     * Sequence (order matters; every guard returns `false` having mutated NOTHING):
     *   1. `!_navActive`   -> `false` (closed-gate invariant above).
     *   2. `!_zeroed`      -> `false` (world frame not yet bound).
     *   3. failed IMU read -> `false`, and @p outPreBindDeltaDeg left UNTOUCHED (no read = no delta).
     *   4. On a successful read: if @p outPreBindDeltaDeg is non-null, set it to the wrap-correct
     *      pre-bind `|normalize(imuYaw - _yawOffsetDeg) - headingDeg|` NOW — BEFORE the strict
     *      decision, so a strict-rejected call still reports its delta.
     *   5. strict reject (`STRICT_ENABLED && delta > TOLERANCE_DEG`) -> `false`, no mutation. The
     *      boundary is inclusive: a delta exactly equal to the tolerance still binds (matches
     *      `reanchorAtWall`).
     *   6. Bind via `_bindHeadingFromYaw(imuYaw, headingDeg)` and return `true`.
     *
     * @param headingDeg         World-frame heading to re-bind to (degrees, `[0, 360)`).
     * @param outPreBindDeltaDeg Optional out-param. On a successful IMU read it is set to the
     *                           wrap-correct pre-bind `|belief - headingDeg|` (degrees) even when the
     *                           strict gate then rejects; on a FAILED read it is left UNTOUCHED.
     * @return `true` if the heading was re-bound; `false` (mutating nothing) on `!_navActive`,
     *         `!_zeroed`, a failed IMU read, or a strict-gate rejection.
     * @note Performs I/O (one live IMU read). PathTracker maps a `false` to a loud soft-skip (log +
     *       continue), never a fault — the run proceeds on the last good heading.
     */
    bool reanchorHeading(float headingDeg, float* outPreBindDeltaDeg = nullptr);

    /**
     * @brief Per-tick entry point for the operator-offset integrator (read-only telemetry).
     *
     * Reads the `Drive` encoders and the live IMU world heading, computes per-tick deltas, and
     * accumulates them into the stored offset when the integrate gate is open. The gate is open
     * ONLY when BOTH hold:
     *   1. `!_navActive` — PathTracker is NOT executing a hop (the operator may be hand-driving).
     *   2. `isZeroed()`  — the world frame is bound (heading is meaningless before zeroing).
     * When the gate is closed the offset is left untouched and NO IMU read is issued; the three
     * baselines (`_lastEncL`, `_lastEncR`, `_lastYawDeg`) are instead re-snapshotted ONCE on the
     * closed->open EDGE (from a single live read) so the first delta after the gate reopens is ~0.
     * This keeps a multi-second mission from issuing a blocking lower-MCU read on every cooperative
     * tick of the hot path.
     *
     * Translation and rotation accumulate on two INDEPENDENT deadband gates (see
     * `Config::OFFSET_TRANSLATION_DEADBAND_CM` / `OFFSET_ROTATION_DEADBAND_DEG`): a pure manual
     * turn updates `thetaDeg` while contributing ~zero translation, and a pure straight push
     * updates `x`/`y` without touching `thetaDeg`.
     *
     * @note Call site (added by a later task) runs this once per cooperative tick AFTER
     *       `PathTracker.update()`. PR1a only integrates + exposes — nothing acts on the offset.
     * @note On a transient lower-MCU read failure while accumulating, the tick is SKIPPED (no
     *       deposit, baselines unadvanced) so motion is never decomposed onto a stale heading; the
     *       deltas carry across to the next good-read tick.
     * @note Reads the lower `Drive` layer (allowed); Localization is never called UP into by Drive.
     */
    void update();

    /**
     * @brief Read the accumulated pose offset from the last anchored node (read-only telemetry).
     *
     * Pure accessor — issues no I/O and mutates no state. Returns the value built up by
     * `update()` since the last `clearOffset()` / node arrival.
     *
     * @return A by-value `Offset2D`: `x`/`y` in centimetres along the world frame, `thetaDeg` a
     *         signed heading delta in degrees. All-zero while the chassis is anchored on a node.
     */
    Offset2D getOffsetFromNode() const;

    /**
     * @brief Zero the accumulated offset AND re-snapshot the integrator baselines.
     *
     * Clears `x`/`y`/`thetaDeg` to `0` and re-snapshots `_lastEncL`/`_lastEncR`/`_lastYawDeg` to
     * the current encoder and heading readings, so the very NEXT `update()` integrates a zero
     * delta. Re-snapshotting the baselines is what kills the arrival-tick phantom: on the single
     * tick where Drive finishes the approach and PathTracker re-anchors, the offset reads ~0 even
     * though the wheels just moved.
     *
     * @note Called by the node-arrival hooks (`onIntersection` / `onDeadReckonedAdvance`); kept
     *       public so a later recovery verb can re-anchor explicitly.
     */
    void clearOffset();

    /**
     * @brief Push PathTracker's hop activity DOWN to the integrator gate.
     *
     * PathTracker calls this `true` while executing a hop and `false` on idle / done / FAULT. The
     * integrator's translation and rotation channels both gate on `!_navActive`, so motion that
     * PathTracker itself commands never grows the operator offset.
     *
     * @param active `true` while PathTracker is driving a hop; `false` otherwise.
     * @note PathTracker only CALLS this method — it never writes offset state directly
     *       (ADR-005 single-writer rule keeps all offset mutation inside this class).
     */
    void setNavActive(bool active);

private:
    /**
     * @brief Wrap a degree value into `[0, 360)`.
     *
     * Handles negative inputs (e.g. raw IMU yaw can be `-180`) by adding `360.0` until
     * the value lands in range. Returns `fmodf(deg + 360, 360)` semantics without the
     * IEEE float subtleties of `fmodf`.
     *
     * @param deg  Any finite float.
     * @return Equivalent value in `[0, 360)`.
     */
    static float _normalize(float deg);

    /**
     * @brief Read the corrected (app CCW-positive) IMU yaw via the single-owner `YawSource`.
     *
    * Thin adapter over `YawSource::readYawDeg` (ADR-045), which performs the result-surfacing
     * lower-MCU read (`mmL.GetIMUEuler`, retry-once) and applies `Config::IMU_YAW_SIGN` to
     * convert the hardware CW-positive raw yaw into the app frame. On a non-OK read `yawOut` is
     * left untouched, so callers can distinguish a real read from a silent comms timeout.
     *
     * @param yawOut  Set to the corrected app-frame yaw in degrees ONLY when the read returns OK.
     * @return `true` if the underlying lower-MCU read returned `MMLower::RESULT::OK`; `false` otherwise.
     */
    static bool _readImuYaw(float& yawOut);

    /**
     * @brief Bind the yaw offset so `getHeadingDeg() == headingDeg`, then refresh the heading cache.
     *
     * The single source of truth for the offset-bind invariant, shared by `reanchorAtWall` step (1)
     * and the public `reanchorHeading`. Both callers do their OWN single IMU read and their OWN
     * gate check, then hand the already-read raw yaw here so the bind math lives in exactly ONE
     * place — the single-writer class exists to protect this invariant, and two copies would drift.
     *
     * Sets `_yawOffsetDeg = imuYaw - headingDeg` (so world == `normalize(imu - offset)` ==
     * `headingDeg`), then refreshes `_lastGoodHeadingDeg = _normalize(imuYaw - _yawOffsetDeg)` and
     * stamps `_lastGoodHeadingMs = millis()`. The cache refresh is REQUIRED, not cosmetic: a no-I/O
     * `getLastHeadingDeg()` and the next `_consumeOffset()` (which baselines `_lastYawDeg` off the
     * cached heading) must see the POST-bind heading, never the stale pre-bind value.
     *
     * @param imuYaw     The corrected app-frame IMU yaw the caller already read (degrees).
     * @param headingDeg The world-frame heading to bind to (degrees).
     * @note Writes `_yawOffsetDeg` plus the heading cache/stamp ONLY. Touches neither `_offset` nor
     *       the integrator baselines — the caller owns any offset re-anchor.
     */
    void _bindHeadingFromYaw(float imuYaw, float headingDeg);

    /**
     * @brief Per-tick accumulation math for the offset integrator (both channels).
     *
     * Called by `update()` ONLY when the integrate gate is open. Decomposes the averaged
     * wheel-encoder delta along the current world heading into `x`/`y`, and the signed IMU-yaw
     * delta into `thetaDeg`, each behind its own deadband. A channel whose delta is below its
     * deadband leaves both its offset AND its baseline UNADVANCED, so slow sub-deadband motion can
     * still accumulate across ticks until it crosses the floor (a stationary chassis never does).
     */
    void _integrateOffset();

    /**
     * @brief Zero the offset and re-snapshot all three integrator baselines to current readings.
     *
     * The implementation behind `clearOffset()`. Reads the current `Drive` encoders and live IMU
     * heading into `_lastEncL`/`_lastEncR`/`_lastYawDeg` and zeroes `_offset`, so the next
     * `update()` sees a zero delta. (Naming per the plan: `_integrateOffset` accumulates;
     * `_consumeOffset` re-anchors.)
     */
    void _consumeOffset();

    /**
     * @brief Re-snapshot the integrator baselines: current `Drive` encoders + a caller-supplied
     *        world heading.
     *
     * The heading is passed in rather than read here so each caller picks its source: the
     * gate-reopen edge in `update()` passes a LIVE read (a fresh, accurate baseline), while the
     * arrival hook `_consumeOffset()` passes the CACHED heading (`getLastHeadingDeg()`) to avoid a
     * blocking IMU read on the PathTracker FSM hot path. Either way the next `update()` integrates
     * a ~0 delta.
     *
     * @param headingDeg World heading (degrees) to anchor `_lastYawDeg` to.
     */
    void _snapshotBaselines(float headingDeg);

    /**
     * @brief Live IMU read -> world heading with explicit success reporting.
     *
     * Shared core of `getHeadingDeg()` and the offset integrator. Reads the raw IMU yaw via
     * `_readImuYaw`, converts to the world frame (`normalize(imu - offset)`), and refreshes the
     * last-good cache + stamp — but ONLY on a successful read. Returns `false` on a transient
     * lower-MCU failure WITHOUT touching @p headingOut or the cache, so callers can choose between
     * serving the last-good value (`getHeadingDeg`) and skipping the tick (`_integrateOffset`,
     * whose offset has no correction path so a stale-heading deposit would be permanent).
     *
     * @param headingOut Set to the world heading in `[0, 360)` ONLY on a successful read.
     * @return `true` if the live read succeeded; `false` on a transient lower-MCU failure.
     * @note Caller must hold `_zeroed` (the world frame must be bound).
     */
    bool _readWorldHeading(float& headingOut) const;

    uint8_t _currentNode  = 0;        ///< Single source of truth for graph position.
    float   _yawOffsetDeg = 0.0f;     ///< Captured at zeroHeadingFromCurrent(); world = IMU - offset.
    bool    _zeroed       = false;    ///< Latched true on first zeroHeadingFromCurrent() call.

    /**
     * @brief World-frame heading from the last successful `getHeadingDeg()` read.
     *
     * Seeded to `Routes::START_HEADING_DEG` so a read before the first OK IMU sample still
     * returns a sane value. Refreshed on every OK read; served verbatim (no I/O) by
     * `getLastHeadingDeg()` and returned by `getHeadingDeg()` when a read fails. `begin()`
     * re-seeds it so a re-zero between bench runs does not serve the prior run's heading.
     */
    mutable float _lastGoodHeadingDeg = (float) Routes::START_HEADING_DEG;

    /// `millis()` of the last OK `getHeadingDeg()` read; `0` = none since `begin()`.
    mutable uint32_t _lastGoodHeadingMs = 0;

    /// Accumulated pose offset from the last anchored node; all-zero while anchored.
    Offset2D _offset;

    /// Last-tick left-encoder snapshot (degrees) — translation-channel delta baseline.
    int32_t _lastEncL = 0;

    /// Last-tick right-encoder snapshot (degrees) — translation-channel delta baseline.
    int32_t _lastEncR = 0;

    /// Last-tick world heading (degrees) — rotation-channel delta baseline.
    float _lastYawDeg = 0.0f;

    /// Pushed DOWN by PathTracker; `true` while it executes a hop. Closes the integrate gate.
    bool _navActive = false;

    /**
     * @brief Previous-tick state of the integrate gate (`!_navActive && _zeroed`).
     *
     * Lets `update()` re-baseline only on the closed->open EDGE instead of every closed tick, so a
     * running mission no longer issues a blocking IMU read per cooperative tick. Reset to `false`
     * by `begin()` so the first open tick after zeroing always re-baselines.
     */
    bool _gateWasOpen = false;
};

/**
 * @brief Single global instance — defined in Localization.cpp.
 */
extern LocalizationClass Localization;

#endif    // LOCALIZATION_H
