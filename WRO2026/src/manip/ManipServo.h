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
 * **Optional stepped ramp (speed control).** By default a move writes its full target angle in one
 * command and the servo slews at its own native speed. Set `cfg.stepMs > 0` (or pass a per-move
 * rate via `setAngle(deg, stepMs)` / the `Manipulator.<verb>().rampMs(ms)` builder) to CRAWL
 * instead: `update()` advances the commanded angle one degree every `stepMs` milliseconds until it
 * reaches the target, so a bigger `stepMs` is slower. This tames joints that slam too fast on the
 * mechanism (e.g. the grabber). `isDone()` stays `false` until the last step lands plus the settle
 * dwell, so a blocking verb waits out the whole ramp; the effective rate is floored by the
 * cooperative-tick period (~8 ms/deg).
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
        : _cfg(cfg), _shadow(cfg.homeDeg), _startMs(0), _estMs(0), _target(cfg.homeDeg),
          _stepMs(cfg.stepMs), _lastStepMs(0)
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
        _target  = _cfg.homeDeg;
        _stepMs  = _cfg.stepMs;
        _startMs = millis();
        _estMs   = 0;
        home();
    }

    /**
     * @brief Command an absolute servo angle at this servo's DEFAULT ramp rate (`cfg.stepMs`).
     *
     * Resets the active ramp rate to the configured default, then arms the move (see the two-arg
     * overload). Every top-level pose verb (`open`/`close`/`home`/`openForWhite`) routes through
     * here, so a prior per-move `.rampMs()` / `setStepMs()` override is naturally one-shot — the
     * next top-level command restores the default rate.
     *
     * @param deg Desired absolute angle in degrees. Values outside `[0, 180]` are clamped.
     * @note A direct command CANCELS any queued `moveThen()` follow-up — the new setpoint wins.
     */
    void setAngle(uint16_t deg) { setAngle(deg, _cfg.stepMs); }

    /**
     * @brief Command an absolute servo angle at an explicit ramp rate for THIS move.
     *
     * Clamps @p deg to `[0, 180]` and arms the move. With @p stepMs `== 0` the full target is
     * written in one command (INSTANT — the original behaviour) and completion is estimated from
     * `cfg.msPerDeg`. With @p stepMs `> 0` the servo CRAWLS: `update()` steps the commanded angle
     * one degree every @p stepMs milliseconds until it reaches the target, and `isDone()` stays
     * false until the last step lands plus `cfg.settleMs`. Fire-and-forget either way — the call
     * returns immediately; a blocking `Manipulator` verb waits it out via the drain.
     *
     * @param deg    Desired absolute angle in degrees (clamped to `[0, 180]`).
     * @param stepMs Milliseconds per 1° step for this move; `0` = instant. Bigger = slower.
     * @note No reverse transform — the raw clamped angle is sent as-is; direction is owned by the
     *       hardware flag applied in `begin()`.
     * @note A direct command CANCELS any queued `moveThen()` follow-up — the new setpoint wins.
     */
    void setAngle(uint16_t deg, uint16_t stepMs)
    {
        _stepMs     = stepMs;
        _hasPending = false;    // a direct command overrides any queued two-stop follow-up
        _armRamp((deg > 180) ? 180 : deg);
    }

    /**
     * @brief Override the ramp rate (ms per 1° step) for the IN-FLIGHT move, in place.
     *
     * Used by the `Manipulator.<verb>().rampMs(ms)` builder chain: the verb has already armed the
     * move at the default rate, and this re-arms the SAME target at @p stepMs. Re-arming (rather
     * than only flipping the rate) is what makes `.instant()` (`stepMs == 0`) actually write the
     * target in one command even when the move was originally armed as a ramp — otherwise an async
     * `.instant()` would still crawl one degree per tick. Leaves any queued two-stop follow-up
     * intact, so both stops share the rate; the next top-level `setAngle(deg)` restores the default.
     *
     * @param stepMs Milliseconds per 1° step; `0` = instant. Bigger = slower.
     */
    void setStepMs(uint16_t stepMs)
    {
        _stepMs = stepMs;
        _armRamp(_target);    // re-evaluate instant-vs-ramp for the current move at the new rate
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
     * @brief Advance an in-progress ramp by one step, then the optional two-stop follow-up.
     *
     * Two responsibilities, gated in order:
     *  1. **Ramp** — if a `stepMs > 0` move is still crawling (`_shadow != _target`), advance the
     *     commanded angle by one degree once the step interval has elapsed, and return. An instant
     *     (`stepMs == 0`) move never enters this branch (it wrote its target in `_armRamp`).
     *  2. **Two-stop follow-up** — only once the current stop has fully settled, fire a queued
     *     `moveThen()` / `homeThenClose()` second stop (arming its ramp at the SAME rate).
     *
     * Must be called each cooperative tick by the owning facade. A settled servo with no queued
     * follow-up costs only two compares here.
     */
    void update()
    {
        // 1) Advance an in-progress ramp — one degree per elapsed step interval.
        if (_shadow != _target)
        {
            if ((uint32_t)(millis() - _lastStepMs) >= _stepMs)
            {
                _shadow = (_target > _shadow) ? (uint16_t)(_shadow + 1) : (uint16_t)(_shadow - 1);
                _writeAngle(_shadow);
                _lastStepMs = millis();
            }
            return;    // still crawling — do not fire the two-stop follow-up yet
        }

        // 2) Ramp/settle complete — fire the queued second stop, keeping the active rate.
        if (_hasPending && _timerElapsed())
        {
            _hasPending = false;
            _armRamp(_pendingDeg);
        }
    }

    /**
     * @brief Move to the configured "open" pose (`cfg.openDeg`).
     * @note Fire-and-forget — poll `isDone()` for completion. See `setAngle()`.
     */
    void open() { setAngle(_cfg.openDeg); }

    /**
     * @brief Move the dumper to the named white-payload open pose.
     * @note Commands the independently configured `cfg.openForWhiteDeg` angle.
     * @note Fire-and-forget — poll `isDone()` for completion. See `setAngle()`.
     */
    void openForWhite() { setAngle(_cfg.openForWhiteDeg); }

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
     * @return `true` once the ramp has reached its target (`_shadow == _target`) AND the
     *         slew-plus-settle budget has elapsed (rollover-safe) AND no `moveThen()` follow-up is
     *         still queued — a two-stop sequence is done only after its SECOND move settles, and a
     *         `stepMs > 0` move is never "done" mid-crawl.
     * @note The `millis() - _startMs` subtraction is computed in `uint32_t`, so it stays correct
     *       across the ~49.7-day `millis()` rollover.
     */
    bool isDone() const { return !_hasPending && (_shadow == _target) && _timerElapsed(); }

    /**
     * @brief Upper-bound estimate of the milliseconds still needed for this servo to reach `isDone()`.
     *
     * Used by `Manipulator::_drain` to size a slow ramp's watchdog ceiling — an instant move
     * returns roughly its settle remainder, a `stepMs > 0` ramp returns the remaining step time
     * (plus any queued second stop), so the drain never cuts a legitimate slow move short.
     *
     * @return Estimated remaining move time in ms; `0` once already done.
     */
    uint32_t estimatedRemainingMs() const
    {
        if (isDone())
        {
            return 0;
        }
        const uint32_t elapsed = (uint32_t)(millis() - _startMs);
        uint32_t       rem     = (_estMs > elapsed) ? (_estMs - elapsed) : 0;
        // A ramp lagging behind its estimate (tick period > stepMs) needs the remaining steps.
        if (_shadow != _target)
        {
            const uint32_t stepsLeft =
                (_target > _shadow) ? (uint32_t)(_target - _shadow) : (uint32_t)(_shadow - _target);
            const uint32_t rampRem = stepsLeft * (uint32_t)_stepMs + _cfg.settleMs;
            if (rampRem > rem)
            {
                rem = rampRem;
            }
        }
        // Add a queued two-stop follow-up's own move time.
        if (_hasPending)
        {
            const uint32_t d = (_pendingDeg > _target) ? (uint32_t)(_pendingDeg - _target)
                                                       : (uint32_t)(_target - _pendingDeg);
            const uint32_t perDeg = (_stepMs > 0) ? (uint32_t)_stepMs : (uint32_t)_cfg.msPerDeg;
            rem += d * perDeg + _cfg.settleMs;
        }
        return rem;
    }

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

    /**
     * @brief Arm a move to the pre-clamped @p clamped target at the active rate `_stepMs`.
     *
     * Sets `_target`, stamps the completion budget, and either writes the full angle now
     * (`_stepMs == 0` or zero delta → INSTANT, preserving the original `msPerDeg`-based estimate)
     * or defers the motion to `update()`, which crawls one degree per `_stepMs` (budget
     * `delta * stepMs + settleMs`). Does NOT touch `_stepMs` or `_hasPending` — the callers own
     * those, so a two-stop follow-up keeps the current rate.
     *
     * @param clamped Absolute target angle already clamped to `[0, 180]`.
     */
    void _armRamp(uint16_t clamped)
    {
        _target = clamped;
        const uint16_t delta =
            (clamped > _shadow) ? (uint16_t)(clamped - _shadow) : (uint16_t)(_shadow - clamped);
        if (_stepMs == 0 || delta == 0)
        {
            // Instant: write the full target now; completion estimated from the native slew rate.
            _estMs   = (uint32_t)((float)delta * _cfg.msPerDeg) + _cfg.settleMs;
            _writeAngle(clamped);
            _shadow  = clamped;
            _startMs = millis();
        }
        else
        {
            // Ramp: `update()` steps `_shadow` toward `_target` one degree per `_stepMs`.
            _estMs      = (uint32_t)delta * (uint32_t)_stepMs + _cfg.settleMs;
            _startMs    = millis();
            _lastStepMs = millis();
        }
    }

    const Config::ServoCfg& _cfg;       ///< Per-servo calibration (channel, poses, slew, direction).
    uint16_t                _shadow;    ///< Last commanded angle in `[0, 180]` (no vendor getter).
    uint32_t                _startMs;   ///< `millis()` at the last move's arm.
    uint32_t                _estMs;     ///< Estimated slew/ramp-plus-settle budget for the last move (ms).
    uint16_t                _target;    ///< Ramp destination; `_shadow` crawls toward this.
    uint16_t                _stepMs;    ///< Active ms per 1° step (`0` = instant); reset to `cfg.stepMs`.
    uint32_t                _lastStepMs;///< `millis()` of the last ramp step (paces `update()`).
    bool                    _hasPending = false;   ///< A `moveThen()` follow-up is queued.
    uint16_t                _pendingDeg = 0;        ///< Clamped angle to fire when `_hasPending`.
};

#endif    // MANIPSERVO_H
