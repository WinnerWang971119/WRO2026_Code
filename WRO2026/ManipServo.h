/**
 * @file ManipServo.h
 * @brief Single RC-servo joint, instantiated three times (grabber / sorter / dumper) by the
 *        `Manipulator` facade.
 *
 * @details
 * `ManipServo` is a thin, header-only wrapper over one `MiniR4.RCx` channel. It exists so the
 * `Manipulator` facade can own three identical joints by value and drive them through a uniform
 * pose API (`open()` / `close()` / `home()` / `setAngle()`) without repeating the per-channel
 * dispatch and the timer bookkeeping in three places.
 *
 * **Fire-and-forget, timer-based completion.** The vendor RC API (`MiniR4RC`) exposes NO position
 * feedback — `setAngle()` commands the lower MCU and returns immediately; there is no way to read
 * the servo's actual angle or ask whether it has arrived. This class therefore models move
 * completion with a software timer: on each `setAngle()` it estimates the slew time from the
 * commanded angle delta (`cfg.msPerDeg`) plus a fixed settle dwell (`cfg.settleMs`), stamps the
 * issue time, and `isDone()` reports completion once that budget has elapsed. Callers poll
 * `isDone()` cooperatively; nothing here blocks.
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

#include "ConfigManipulator.h"

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
     * @brief Apply the hardware direction flag and seed the move timer.
     *
     * Calls `MiniR4RC::setHWDir(cfg.reverse)` exactly once on the configured RC channel (hardware
     * owns direction — there is no software negation). Then re-seeds the angle shadow to
     * `cfg.homeDeg`, stamps the issue time, and zeroes the move budget so `isDone()` reports `true`
     * immediately after `begin()`.
     *
     * @note Does NOT command any angle — the servo holds whatever pose it powered up in until the
     *       first `setAngle()` / `open()` / `close()` / `home()`.
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
     */
    void setAngle(uint16_t deg)
    {
        const uint16_t clamped = (deg > 180) ? 180 : deg;
        _estMs = (uint32_t)(fabs((float)clamped - (float)_shadow) * _cfg.msPerDeg) + _cfg.settleMs;
        _writeAngle(clamped);
        _shadow  = clamped;
        _startMs = millis();
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
     * @return `true` once the estimated slew-plus-settle budget has elapsed (rollover-safe).
     * @note The `millis() - _startMs` subtraction is computed in `uint32_t`, so it stays correct
     *       across the ~49.7-day `millis()` rollover.
     */
    bool isDone() const { return (uint32_t)(millis() - _startMs) >= _estMs; }

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

    const Config::ServoCfg& _cfg;       ///< Per-servo calibration (channel, poses, slew, direction).
    uint16_t                _shadow;    ///< Last commanded angle in `[0, 180]` (no vendor getter).
    uint32_t                _startMs;   ///< `millis()` at the last `setAngle()` issue.
    uint32_t                _estMs;     ///< Estimated slew-plus-settle budget for the last move (ms).
};

#endif    // MANIPSERVO_H
