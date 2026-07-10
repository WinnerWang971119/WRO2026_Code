/**
 * @file ManipServo.h
 * @brief Single RC-servo joint, instantiated three times (grabber / sorter / dumper) by the
 *        `Manipulator` facade.
 *
 * @details
 * `ManipServo` is a thin, header-only wrapper over one `MiniR4.RCx` channel. It exists so the
 * `Manipulator` facade can own three identical joints by value and drive them through a uniform
 * pose API (`open()` / `close()` / `home()` / `setAngle()`, plus the two-stop `moveThen()` /
 * `homeThenClose()`) without repeating the per-channel dispatch and the timer bookkeeping in
 * three places.
 *
 * **Fire-and-forget, timer-based completion.** The vendor RC API (`MiniR4RC`) exposes NO position
 * feedback — `setAngle()` commands the lower MCU and returns immediately; there is no way to read
 * the servo's actual angle or ask whether it has arrived. This class therefore models move
 * completion with a software timer: on each `setAngle()` it estimates the slew time from the
 * commanded angle delta (`cfg.msPerDeg`) plus a fixed settle dwell (`cfg.settleMs`), stamps the
 * issue time, and `isDone()` reports completion once that budget has elapsed. Callers poll
 * `isDone()` cooperatively; nothing here blocks.
 *
 * **Optional two-stop sequence.** `moveThen(first, second)` (and its `homeThenClose()` sugar)
 * commands `first` now and QUEUES `second` to fire automatically once `first` settles. Because the
 * servo is fire-and-forget, two back-to-back `setAngle()` calls would skip the intermediate pose
 * (the second write overrides the first before the servo moves), so the follow-up is deferred: the
 * facade advances it by calling `update()` each cooperative tick, and `isDone()` stays `false`
 * until the SECOND move settles. Any direct `setAngle()` (or pose verb) cancels a queued follow-up.
 *
 * **Hardware-owned direction.** A reversed servo is handled entirely in hardware: `begin()`
 * forwards `cfg.reverse` to `MiniR4RC::setHWDir()` exactly once. There is NO software angle
 * negation anywhere in this class — the commanded angle is always the raw value the caller asked
 * for, clamped to `[0, 180]`. `commandedAngle()` returns that raw shadow value.
 *
 * **Runtime channel dispatch.** Unlike most port-bound classes in this codebase, `ManipServo` is
 * NOT a template: the facade declares three instances of this one type, so the RC channel is a
 * runtime `uint8_t` (`cfg.rcId`) and the per-channel `MiniR4.RC1..RC4` dispatch happens through a
 * small private `switch`.
 */

#ifndef MANIPSERVO_H
#define MANIPSERVO_H

#include <Arduino.h>
#include <MatrixMiniR4.h>

#include "../../config/ConfigManipulator.h"

/**
 * @brief One RC-servo joint with fire-and-forget, timer-based move completion.
 *
 * Bound to a single `MiniR4.RCx` channel selected at runtime by `Config::ServoCfg::rcId`. Holds a
 * shadow copy of the last-commanded angle (the vendor API has no getter) and a millis-based budget
 * used by `isDone()`. See the file docblock for the fire-and-forget / hardware-direction contract.
 */
class ManipServo
{
public:
    /**
     * @brief Construct a joint bound to the channel and calibration in @p cfg.
     *
     * Stores @p cfg by const reference and seeds the angle shadow to `cfg.homeDeg`. Does NOT touch
     * hardware — call `begin()` once after `MiniR4.begin()` has returned.
     *
     * @param cfg Per-servo calibration (channel, key poses, slew rate, settle time, direction).
     *            MUST outlive this `ManipServo` (typically a `constexpr Config::ServoCfg`).
     */
    explicit ManipServo(const Config::ServoCfg& cfg)
        : _cfg(cfg), _shadow(cfg.homeDeg), _startMs(0), _estMs(0)
    {
    }

    /**
     * @brief Apply the hardware direction flag, seed the move timer, and command the home pose.
     *
     * Calls `MiniR4RC::setHWDir(cfg.reverse)` exactly once on the configured RC channel (hardware
     * owns direction — there is no software negation), re-seeds the angle shadow to `cfg.homeDeg`,
     * then issues `home()` so the servo drives to its rest pose at boot (AC8 known-safe state).
     *
     * @note Commands `cfg.homeDeg` via `home()`. Because the shadow was just re-seeded to
     *       `cfg.homeDeg`, the move budget is `0 * msPerDeg + cfg.settleMs`, so `isDone()` reports
     *       `true` only after `cfg.settleMs` (NOT immediately) — poll it before assuming the servo
     *       has settled.
     * @note Empirically the servo does not reach the home pose unless the angle is written here in
     *       `begin()` (the later `home()` in `Manipulator::begin()` is not sufficient on its own),
     *       so this write is left in place deliberately — "if it works, it works" — pending a
     *       hardware-level root cause for the duplicate home.
     * @note The vendor `setHWDir` returns `bool`; on a failed call this retries once after a 1 ms
     *       delay (the transient-failure retry pattern used throughout the MMLower callers).
     */
    void begin()
    {
        _setDir(_cfg.reverse);
        _shadow  = _cfg.homeDeg;
        _startMs = millis();
        _estMs   = 0;
        home();
    }

    /**
     * @brief Command an absolute servo angle and arm the completion timer.
     *
     * Clamps @p deg to `[0, 180]`, estimates the move budget as
     * `|clamped - shadow| * cfg.msPerDeg + cfg.settleMs`, writes the clamped angle to the lower MCU,
     * then updates the angle shadow and stamps the issue time. The write is fire-and-forget: it
     * returns immediately and completion is reported by `isDone()` once the budget elapses.
     *
     * @param deg Desired absolute angle in degrees. Values outside `[0, 180]` are clamped (the
     *            vendor servo range), not rejected.
     * @note No reverse transform — the raw clamped angle is sent as-is; direction is owned by the
     *       hardware flag applied in `begin()`.
     * @note The vendor `setAngle` returns `bool`; on a failed call this retries once after a 1 ms
     *       delay before giving up (transient-failure retry pattern).
     * @note A direct command CANCELS any queued `moveThen()` follow-up — the new setpoint wins.
     */
    void setAngle(uint16_t deg)
    {
        const uint16_t clamped = (deg > 180) ? 180 : deg;
        _estMs = (uint32_t)(fabs((float)clamped - (float)_shadow) * _cfg.msPerDeg) + _cfg.settleMs;
        _writeAngle(clamped);
        _shadow     = clamped;
        _startMs    = millis();
        _hasPending = false;    // a direct command overrides any queued two-stop follow-up
    }

    /**
     * @brief Command @p first now, then automatically command @p second once @p first settles.
     *
     * A two-stop, fire-and-forget sequence. @p first is issued immediately (via `setAngle()`); the
     * clamped @p second is QUEUED and fired by `update()` on the first tick after `first`'s
     * slew-plus-settle timer elapses. `isDone()` stays `false` until the SECOND move settles, so a
     * blocking drain on this servo waits for the whole sequence. Any direct `setAngle()` / pose verb
     * issued before the follow-up fires cancels it.
     *
     * @param first  Absolute angle to command now (clamped to `[0, 180]` by `setAngle()`).
     * @param second Absolute angle to command once @p first settles (clamped to `[0, 180]`).
     * @note Advanced only by `update()` — the owner must tick this servo each cooperative loop for
     *       the follow-up to fire.
     */
    void moveThen(uint16_t first, uint16_t second)
    {
        setAngle(first);    // fires `first` and clears any prior pending
        _pendingDeg = (second > 180) ? 180 : second;
        _hasPending = true;
    }

    /**
     * @brief Open fully to the home pose, then close — a self-contained capture stroke.
     *
     * Convenience over `moveThen(cfg.homeDeg, cfg.closeDeg)`: drives the joint to its widest/home
     * pose first, then clamps to close once home settles, so a capture always runs the full
     * open→close stroke regardless of the joint's prior position.
     *
     * @note Fire-and-forget two-stop sequence — see `moveThen()`. `isDone()` reports `true` only
     *       after the close settles; advanced by `update()`.
     */
    void homeThenClose() { moveThen(_cfg.homeDeg, _cfg.closeDeg); }

    /**
     * @brief Advance the optional two-stop sequence — fire the queued follow-up if the first
     *        move has settled. A no-op when no follow-up is pending.
     *
     * Must be called each cooperative tick by the owning facade so a `moveThen()` / `homeThenClose()`
     * sequence progresses. Pure servos with no queued follow-up do no work here.
     */
    void update()
    {
        if (_hasPending && _timerElapsed())
        {
            const uint16_t next = _pendingDeg;
            _hasPending         = false;
            setAngle(next);
        }
    }

    /**
     * @brief Move to the configured "open" pose (`cfg.openDeg`).
     * @note Fire-and-forget — poll `isDone()` for completion. See `setAngle()`.
     */
    void open() { setAngle(_cfg.openDeg); }

    /**
     * @brief Move to the configured "close" pose (`cfg.closeDeg`).
     * @note Fire-and-forget — poll `isDone()` for completion. See `setAngle()`.
     */
    void close() { setAngle(_cfg.closeDeg); }

    /**
     * @brief Move to the configured rest pose (`cfg.homeDeg`).
     * @note Fire-and-forget — poll `isDone()` for completion. See `setAngle()`.
     */
    void home() { setAngle(_cfg.homeDeg); }

    /**
     * @brief Whether the last commanded move's timer budget has elapsed.
     *
     * Because the vendor RC API has NO position feedback, completion is purely time-based: the move
     * is considered done once `cfg.msPerDeg * |delta| + cfg.settleMs` milliseconds have passed since
     * the issuing `setAngle()`. This is an estimate, not a confirmation — a stalled or unpowered
     * servo will still report done on schedule.
     *
     * @return `true` once the estimated slew-plus-settle budget has elapsed (rollover-safe) AND no
     *         `moveThen()` follow-up is still queued — a two-stop sequence is done only after its
     *         SECOND move settles.
     * @note The `millis() - _startMs` subtraction is computed in `uint32_t`, so it stays correct
     *       across the ~49.7-day `millis()` rollover.
     */
    bool isDone() const { return !_hasPending && _timerElapsed(); }

    /**
     * @brief The last angle commanded to the servo (clamped shadow value).
     *
     * @return The most recently written angle in `[0, 180]`; `cfg.homeDeg` before the first move.
     * @note This is the COMMANDED angle, not a measured one — the vendor API has no getter, so the
     *       servo's true position is unknown.
     */
    uint16_t commandedAngle() const { return _shadow; }

private:
    /**
     * @brief Write a clamped angle to the configured RC channel, retrying once on failure.
     *
     * Dispatches to `MiniR4.RC1..RC4.setAngle()` by `_cfg.rcId`. The vendor call returns `bool`; on
     * `false` this retries once after `delay(1)` (transient-failure retry pattern). An unknown
     * `rcId` is a no-op (the `default` branch) — channel validity is enforced upstream in `Config`.
     *
     * @param deg Clamped angle in `[0, 180]` to forward to the lower MCU.
     */
    void _writeAngle(uint16_t deg)
    {
        switch (_cfg.rcId)
        {
        case 1:
            if (!MiniR4.RC1.setAngle(deg)) { delay(1); MiniR4.RC1.setAngle(deg); }
            break;
        case 2:
            if (!MiniR4.RC2.setAngle(deg)) { delay(1); MiniR4.RC2.setAngle(deg); }
            break;
        case 3:
            if (!MiniR4.RC3.setAngle(deg)) { delay(1); MiniR4.RC3.setAngle(deg); }
            break;
        case 4:
            if (!MiniR4.RC4.setAngle(deg)) { delay(1); MiniR4.RC4.setAngle(deg); }
            break;
        default:
            break;
        }
    }

    /**
     * @brief Set the hardware direction on the configured RC channel, retrying once on failure.
     *
     * Dispatches to `MiniR4.RC1..RC4.setHWDir()` by `_cfg.rcId`. The vendor call returns `bool`; on
     * `false` this retries once after `delay(1)`. An unknown `rcId` is a no-op (`default`).
     *
     * @param dir Hardware direction flag forwarded to `MiniR4RC::setHWDir()` (NOT a software
     *            negation): `true` = forward, `false` = reverse.
     */
    void _setDir(bool dir)
    {
        switch (_cfg.rcId)
        {
        case 1:
            if (!MiniR4.RC1.setHWDir(dir)) { delay(1); MiniR4.RC1.setHWDir(dir); }
            break;
        case 2:
            if (!MiniR4.RC2.setHWDir(dir)) { delay(1); MiniR4.RC2.setHWDir(dir); }
            break;
        case 3:
            if (!MiniR4.RC3.setHWDir(dir)) { delay(1); MiniR4.RC3.setHWDir(dir); }
            break;
        case 4:
            if (!MiniR4.RC4.setHWDir(dir)) { delay(1); MiniR4.RC4.setHWDir(dir); }
            break;
        default:
            break;
        }
    }

    /**
     * @brief Whether the last commanded move's slew-plus-settle timer has elapsed (ignores any
     *        queued `moveThen()` follow-up). Rollover-safe `uint32_t` subtraction.
     *
     * `isDone()` layers the pending-follow-up check on top of this; `update()` uses it raw to know
     * when to fire the queued second move.
     */
    bool _timerElapsed() const { return (uint32_t)(millis() - _startMs) >= _estMs; }

    const Config::ServoCfg& _cfg;       ///< Per-servo calibration (channel, poses, slew, direction).
    uint16_t                _shadow;    ///< Last commanded angle in `[0, 180]` (no vendor getter).
    uint32_t                _startMs;   ///< `millis()` at the last `setAngle()` issue.
    uint32_t                _estMs;     ///< Estimated slew-plus-settle budget for the last move (ms).
    bool                    _hasPending = false;   ///< A `moveThen()` follow-up is queued.
    uint16_t                _pendingDeg = 0;        ///< Clamped angle to fire when `_hasPending`.
};

#endif    // MANIPSERVO_H
