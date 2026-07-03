/**
 * @file LineFollower.h (CTPS HOST MOCK)
 * @brief Recording mock of the LineFollower facade for the CTPS dump.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `LineFollower.h` (quote-included; force-included by the T4
 * cmake wiring). The include guard is the REAL one (`LINEFOLLOWER_H`) so the
 * real header no-ops if reached afterwards.
 *
 * Line following is MOSTLY a no-op here — the P+D control loop produces no
 * command-stream entries the CTPS player consumes — but distance-gated follows
 * ARE a recording seam: they advance the shared chassis without advancing the
 * logical node, so they must appear in the trace as a `lineFollowDist` drive
 * move. Two producers record it:
 *   - The `setEnabled(true).setDistanceTarget(cm)` fluent builder records once
 *     from an `_executed`-guarded DESTRUCTOR (so chained setters — the distance
 *     target and `.setReverse(true)` — are seen before the record is emitted),
 *     and ONLY when the transition both enables the follower AND arms a distance
 *     gate. `setEnabled(false)` and a target-less `setEnabled(true)` record
 *     nothing.
 *   - The sync-blocking `followDistance(cm)` facade helper records
 *     `lineFollowDist` DIRECTLY. Unlike the real facade — which delegates to
 *     `setEnabled(true).setDistanceTarget(cm)` — the mock must NOT delegate, or
 *     the enable-builder destructor would emit a SECOND (duplicate) record for
 *     the same follow.
 *
 * The enable builder mirrors the real move-only `[[nodiscard]]` shape (deleted
 * copy + move) so the discarded-temporary call sites compile the same way they
 * do against the real header. It stays non-movable — CTPS only ever sees
 * discarded temporaries and C++17 guaranteed copy elision means no move is
 * invoked on the by-value return from `setEnabled` — with a `static_assert`
 * documenting the invariant and the `_executed` guard keeping the emit
 * idempotent regardless.
 */

#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <stdint.h>

#include <type_traits>

#include "Config.h"         // Config::SpeedPreset for the chain setters
#include "ctps_record.h"    // ctps::recordDriveMove — the lineFollowDist producer

class LineFollowerClass;

/**
 * @brief Recording mock of `LineFollowerEnableBuilder`.
 *
 * Move-only `[[nodiscard]]` like the real builder. Most setters are no-ops
 * returning `*this`; `setDistanceTarget` / `setReverse` capture the state the
 * `lineFollowDist` record needs. The captured state is emitted exactly once
 * from the `_executed`-guarded destructor, and only when the transition both
 * enables the follower and arms a distance gate. When `setReverse` is not
 * chained, the record's direction falls back to the facade's persistent
 * `_reverse` — mirrors the real `_hasReverseOverride` seam (LineFollower.cpp:610),
 * which preserves a prior `LineFollower.setReverse(true)`.
 */
class [[nodiscard]] LineFollowerEnableBuilder
{
public:
    /**
     * @brief Construct with the target enable state.
     * @param targetEnabled `true` for an enable transition, `false` for disable.
     *                      Recorded as `_enabled`; a disable never emits a record.
     */
    LineFollowerEnableBuilder(LineFollowerClass& owner, bool targetEnabled)
        : _owner(owner), _enabled(targetEnabled)
    {
    }

    LineFollowerEnableBuilder(const LineFollowerEnableBuilder&)            = delete;
    LineFollowerEnableBuilder& operator=(const LineFollowerEnableBuilder&) = delete;
    LineFollowerEnableBuilder(LineFollowerEnableBuilder&&)                 = delete;
    LineFollowerEnableBuilder& operator=(LineFollowerEnableBuilder&&)      = delete;

    /**
     * @brief Emit one `lineFollowDist` record iff the transition enables the
     *        follower AND a distance gate was armed.
     *
     * `_executed`-guarded so a (defensive) second destructor invocation is a
     * no-op. `setEnabled(false)` (`_enabled` false) and a target-less
     * `setEnabled(true)` (`_hasTarget` false) each record nothing. Defined
     * out-of-line below `LineFollowerClass` because it reads the facade's
     * persistent `_reverse` (via `getReverse()`) when this builder did not chain
     * `setReverse` — the facade type is incomplete at this point.
     */
    ~LineFollowerEnableBuilder();

    LineFollowerEnableBuilder& setBrake(bool) { return *this; }
    LineFollowerEnableBuilder& setPreset(Config::SpeedPreset) { return *this; }
    LineFollowerEnableBuilder& tShape() { return *this; }
    LineFollowerEnableBuilder& setTShapeRampUp(bool) { return *this; }
    LineFollowerEnableBuilder& setTShapeRampDown(bool) { return *this; }

    /** @brief Opt out of both cruise ramps (#89 missing-setter gap). No-op mock. */
    LineFollowerEnableBuilder& flat() { return *this; }

    LineFollowerEnableBuilder& setAccel(float) { return *this; }

    /**
     * @brief Capture the reverse-direction flag for this follow's record.
     *
     * Marks `_hasReverseOverride` so the destructor uses this value instead of
     * the facade's persistent `_reverse` — mirrors the real builder's
     * override-pair gating (LineFollower.cpp:610).
     */
    LineFollowerEnableBuilder& setReverse(bool reverse)
    {
        _reverse            = reverse;
        _hasReverseOverride = true;
        return *this;
    }

    /** @brief Arm the distance gate; captures the commanded distance for the record. */
    LineFollowerEnableBuilder& setDistanceTarget(float cm)
    {
        _distanceTarget = cm;
        _hasTarget      = true;
        return *this;
    }

    LineFollowerEnableBuilder& setOvershootMode(bool) { return *this; }
    LineFollowerEnableBuilder& setBrakeOnGate(bool) { return *this; }

private:
    LineFollowerClass& _owner;      ///< Facade that spawned this builder; source of the fallback reverse.
    bool  _enabled;                 ///< From the ctor's targetEnabled; a disable never records.
    float _distanceTarget = 0.0f;   ///< Commanded distance from setDistanceTarget(cm).
    bool  _hasTarget      = false;  ///< True iff setDistanceTarget() was called.
    bool  _reverse        = false;  ///< From setReverse(bool); valid iff _hasReverseOverride.
    bool  _hasReverseOverride = false;  ///< True iff setReverse() was chained on THIS builder.
    bool  _executed       = false;  ///< Guards the destructor emit against a double-record.
};

static_assert(!std::is_copy_constructible<LineFollowerEnableBuilder>::value
                  && !std::is_move_constructible<LineFollowerEnableBuilder>::value,
              "CTPS LineFollowerEnableBuilder must stay non-copyable and non-movable so its "
              "_executed-guarded destructor emits exactly once — the by-value return from "
              "setEnabled relies on C++17 guaranteed copy elision, not a move.");

/**
 * @brief Recording mock of the LineFollower facade.
 *
 * Every method is a no-op except the distance-gated follow seam:
 * `followDistance(cm)` records a `lineFollowDist` move directly, and the enable
 * builder (via `setEnabled`) records one from its destructor. `setReverse` /
 * `getReverse` keep a persistent facade-level `_reverse` so `followDistance`
 * captures the follow direction (the enable-builder path uses its own chained
 * `setReverse` when present, else falls back to this facade `_reverse`).
 */
class LineFollowerClass
{
public:
    /** @brief No-op bring-up; always reports success. */
    bool begin() { return true; }

    /** @brief No-op cooperative tick. */
    void update() {}

    /** @brief Return the recording enable builder (matches the real `[[nodiscard]]` verb). */
    [[nodiscard]] LineFollowerEnableBuilder setEnabled(bool on)
    {
        return LineFollowerEnableBuilder(*this, on);
    }

    void                setPreset(Config::SpeedPreset) {}
    Config::SpeedPreset getPreset() const { return Config::SpeedPreset::NORMAL; }

    /** @brief Persist the facade-level reverse flag read by `followDistance`. */
    void setReverse(bool r) { _reverse = r; }

    /** @brief Read the facade-level reverse flag (source for `followDistance`). */
    bool getReverse() const { return _reverse; }

    bool  isEnabled() const { return false; }
    bool  isLineLost() const { return false; }
    bool  isPidFrozen() const { return false; }
    float getRawError() const { return 0.0f; }
    bool  isDistanceReached() { return true; }
    void  setTickCallback(void (*)()) {}

    /**
     * @brief Record a distance-gated follow directly as a `lineFollowDist` move.
     *
     * Unlike the real facade (which delegates to
     * `setEnabled(true).setDistanceTarget(cm)`), the mock records here and does
     * NOT delegate — delegating would double-record via the enable-builder
     * destructor. Reverse is sourced from the persistent facade `_reverse`.
     *
     * @param cm Commanded follow distance in centimetres.
     * @return   Always `true` (mirrors the real success return).
     */
    bool followDistance(float cm)
    {
        ctps::recordDriveMove("lineFollowDist",
                              {cm, /*mode*/ nullptr, /*hasHeading*/ false, /*heading*/ 0.0f,
                               _reverse});
        return true;
    }

    bool waitUntilDistanceReached() { return true; }
    bool followUntilIntersection() { return true; }
    bool waitUntilIntersection() { return true; }

private:
    /// @brief Persistent facade-level reverse flag; set by `setReverse`, read by
    ///        `followDistance`. Separate from the enable-builder's own `_reverse`.
    bool _reverse = false;
};

/**
 * @brief Emit the enable-builder's `lineFollowDist` record (out-of-line so it
 *        may read the now-complete facade's persistent `_reverse`).
 *
 * Direction precedence mirrors the real `_applyEnableBuilder`: a chained
 * `setReverse` on this builder wins; otherwise fall back to the facade's
 * persistent `_reverse` (a prior `LineFollower.setReverse(true)`), which the
 * real builder preserves when `_hasReverseOverride` is false.
 */
inline LineFollowerEnableBuilder::~LineFollowerEnableBuilder()
{
    if (_executed)
    {
        return;
    }
    _executed = true;
    if (_enabled && _hasTarget)
    {
        const bool reverse = _hasReverseOverride ? _reverse : _owner.getReverse();
        ctps::recordDriveMove(
            "lineFollowDist",
            {_distanceTarget, /*mode*/ nullptr, /*hasHeading*/ false, /*heading*/ 0.0f, reverse});
    }
}

/** @brief Single global instance — defined in ctps_record.cpp. */
extern LineFollowerClass LineFollower;

#endif    // LINEFOLLOWER_H
