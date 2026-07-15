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
 * The enable builder mirrors the real move-only shape (deleted
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
 * @brief Map a `Config::SpeedPreset` to its command-stream `preset` string.
 *
 * Duplicated locally (rather than shared with `ctps_mocks/Drive.h`) because
 * each CTPS mock header is a self-contained shadow of its real counterpart —
 * see the file docstring. Keep byte-identical to `Drive.h`'s
 * `speedPresetToStr` and to the loader's preset whitelist.
 *
 * @param p The resolved speed preset.
 * @return `"slow"` / `"normal"` / `"fast"`.
 */
inline const char* lineFollowerPresetToStr(Config::SpeedPreset p)
{
    switch (p)
    {
        case Config::SpeedPreset::SLOW:
            return "slow";
        case Config::SpeedPreset::NORMAL:
            return "normal";
        case Config::SpeedPreset::FAST:
            return "fast";
    }
    return "";    // Unreachable: every SpeedPreset value is handled — satisfies -Wreturn-type.
}

/**
 * @brief Recording mock of `LineFollowerEnableBuilder`.
 *
 * Move-only like the real builder. Most setters are no-ops
 * returning `*this`; `setDistanceTarget` / `setReverse` / `setPreset` /
 * `setBrake` / `tShape` / `flat` capture the state the `lineFollowDist`
 * record needs. The captured state is emitted exactly once from the
 * `_executed`-guarded destructor, and only when the transition both enables
 * the follower and arms a distance gate. When `setReverse` is not chained,
 * the record's direction falls back to the facade's persistent `_reverse` —
 * mirrors the real `_hasReverseOverride` seam (LineFollower.cpp:610), which
 * preserves a prior `LineFollower.setReverse(true)`.
 */
class LineFollowerEnableBuilder
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

    /**
     * @brief Capture the brake flag for this follow's record.
     *
     * Was previously a no-op mock setter (issue #130 T8 fix) — a chained
     * `.setBrake(...)` now actually stores into `_brake`/`_hasBrake` before
     * the destructor emits.
     */
    LineFollowerEnableBuilder& setBrake(bool b)
    {
        _brake    = b;
        _hasBrake = true;
        return *this;
    }

    /**
     * @brief Capture the cruise preset for this follow's record.
     *
     * Was previously a no-op mock setter (issue #130 T8 fix) — a chained
     * `.setPreset(...)` now actually stores the "slow"/"normal"/"fast" string
     * before the destructor emits.
     */
    LineFollowerEnableBuilder& setPreset(Config::SpeedPreset p)
    {
        _preset = lineFollowerPresetToStr(p);
        return *this;
    }

    /**
     * @brief Capture the profiled flag (true) for this follow's record.
     *
     * Was previously a total no-op mock setter that silently DROPPED the
     * `profiled` knob (issue #130 T8 fix — the exact "no-op mock setter drops
     * the builder knob" trap) — a chained `.tShape()` now actually stores
     * `_profiled = true` before the destructor emits.
     */
    LineFollowerEnableBuilder& tShape()
    {
        _profiled    = true;
        _hasProfiled = true;
        return *this;
    }
    LineFollowerEnableBuilder& setTShapeRampUp(bool) { return *this; }
    LineFollowerEnableBuilder& setTShapeRampDown(bool) { return *this; }

    /**
     * @brief Capture the profiled flag (false) for this follow's record
     *        (#89 missing-setter gap; issue #130 T8 fix).
     *
     * Was previously a no-op mock setter — a chained `.flat()` now actually
     * stores `_profiled = false` before the destructor emits.
     */
    LineFollowerEnableBuilder& flat()
    {
        _profiled    = false;
        _hasProfiled = true;
        return *this;
    }

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
    const char* _preset   = nullptr;    ///< Set by setPreset(); nullptr = preset unset.
    bool  _brake          = false;  ///< Set by setBrake(); valid iff _hasBrake.
    bool  _hasBrake       = false;  ///< True iff setBrake() was called.
    bool  _profiled       = false;  ///< true=.tShape(), false=.flat(); valid iff _hasProfiled.
    bool  _hasProfiled    = false;  ///< True iff .tShape() or .flat() was called.
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

    /** @brief Return the recording enable builder (matches the real verb). */
    LineFollowerEnableBuilder setEnabled(bool on)
    {
        return LineFollowerEnableBuilder(*this, on);
    }

    /**
     * @brief Persist the facade-level cruise preset.
     *
     * Was previously a no-op mock setter (`getPreset()` always reported
     * `NORMAL`; issue #130 T8 fix). Now mirrors the persistent-`_reverse`
     * pattern below so `followDistance` — which the real facade documents as
     * using "the currently configured preset" — can report a real value
     * instead of fabricating one.
     */
    void setPreset(Config::SpeedPreset p) { _preset = p; }

    /** @brief Read the facade-level cruise preset (source for `followDistance`). */
    Config::SpeedPreset getPreset() const { return _preset; }

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
     * destructor. Reverse and preset are sourced from the persistent facade
     * state (`_reverse` / `_preset`) per the real facade's documented "uses the
     * currently configured preset + reverse direction" behavior; this one-shot
     * call carries no per-call `power`/`start_speed`/`end_speed`/`profiled`/
     * `brake` knobs of its own, so those stay unset (not fabricated).
     *
     * @param cm Commanded follow distance in centimetres.
     * @return   Always `true` (mirrors the real success return).
     */
    bool followDistance(float cm)
    {
        ctps::recordDriveMove(
            "lineFollowDist",
            {
                cm,                              // cm
                /*mode*/ nullptr,                // mode
                /*hasHeading*/ false,             // hasHeading
                /*heading*/ 0.0f,                 // heading
                _reverse,                         // reverse
                /*power*/ 0,                       // power (no per-call knob on followDistance)
                /*hasPower*/ false,                // hasPower
                lineFollowerPresetToStr(_preset), // preset (facade's persistent preset)
                /*startSpeed*/ 0,                  // startSpeed (LineFollower has no setStartSpeed)
                /*hasStartSpeed*/ false,           // hasStartSpeed
                /*endSpeed*/ 0,                    // endSpeed (LineFollower has no setEndSpeed)
                /*hasEndSpeed*/ false,             // hasEndSpeed
                /*profiled*/ false,                // profiled (no per-call knob on followDistance)
                /*hasProfiled*/ false,             // hasProfiled
                /*brake*/ false,                   // brake (no per-call knob on followDistance)
                /*hasBrake*/ false                 // hasBrake
            });
        return true;
    }

    bool waitUntilDistanceReached() { return true; }
    bool followUntilIntersection() { return true; }
    bool waitUntilIntersection() { return true; }

private:
    /// @brief Persistent facade-level reverse flag; set by `setReverse`, read by
    ///        `followDistance`. Separate from the enable-builder's own `_reverse`.
    bool _reverse = false;

    /// @brief Persistent facade-level cruise preset; set by `setPreset`, read by
    ///        `followDistance`. Separate from the enable-builder's own `_preset`.
    Config::SpeedPreset _preset = Config::SpeedPreset::NORMAL;
};

/**
 * @brief Emit the enable-builder's `lineFollowDist` record (out-of-line so it
 *        may read the now-complete facade's persistent `_reverse`).
 *
 * Direction precedence mirrors the real `_applyEnableBuilder`: a chained
 * `setReverse` on this builder wins; otherwise fall back to the facade's
 * persistent `_reverse` (a prior `LineFollower.setReverse(true)`), which the
 * real builder preserves when `_hasReverseOverride` is false. `preset` follows
 * the SAME precedence: a chained `setPreset` on this builder wins, else fall
 * back to the facade's persistent preset (`_owner.getPreset()`) — mirroring the
 * real `_applyEnableBuilder` keeping `_preset` when `hasPresetOverride` is false
 * (LineFollower.cpp:606-609), so a prior `LineFollower.setPreset(FAST)` survives
 * an enable that does not re-chain it. `brake` / `profiled` are this builder's
 * OWN captured state (no builder-vs-facade pairing) — power/startSpeed/endSpeed
 * stay unset since `LineFollowerEnableBuilder` has no such setters on the real
 * header.
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
        // Chained setPreset on this builder wins; else fall back to the facade's
        // persistent preset (mirrors the real `_applyEnableBuilder`, which keeps
        // `_preset` when `hasPresetOverride` is false). `_preset` is a non-null
        // "slow"/"normal"/"fast" string ONLY when setPreset was chained, so the
        // null check IS the builder's override flag.
        const char* const preset =
            _preset != nullptr ? _preset : lineFollowerPresetToStr(_owner.getPreset());
        ctps::recordDriveMove(
            "lineFollowDist",
            {
                _distanceTarget,   // cm
                /*mode*/ nullptr,  // mode
                /*hasHeading*/ false,  // hasHeading
                /*heading*/ 0.0f,      // heading
                reverse,                // reverse
                /*power*/ 0,             // power (LineFollowerEnableBuilder has no setPower)
                /*hasPower*/ false,      // hasPower
                preset,                  // preset
                /*startSpeed*/ 0,        // startSpeed (LineFollowerEnableBuilder has no setStartSpeed)
                /*hasStartSpeed*/ false, // hasStartSpeed
                /*endSpeed*/ 0,          // endSpeed (LineFollowerEnableBuilder has no setEndSpeed)
                /*hasEndSpeed*/ false,   // hasEndSpeed
                _profiled,               // profiled
                _hasProfiled,            // hasProfiled
                _brake,                  // brake
                _hasBrake                // hasBrake
            });
    }
}

/** @brief Single global instance — defined in ctps_record.cpp. */
extern LineFollowerClass LineFollower;

#endif    // LINEFOLLOWER_H
