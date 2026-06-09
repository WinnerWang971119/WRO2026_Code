/**
 * @file ManipBelt.h
 * @brief Conveyor-belt actuator wrapper around a single LEGO encoder DC motor.
 *
 * @details
 * `ManipBelt` drives the manipulator conveyor belt. Unlike the RC servos
 * (`ManipServo`, which has no position feedback and fakes completion with a
 * timer), the belt motor is a LEGO encoder DC motor with REAL encoder feedback,
 * so this class can move it to absolute encoder targets and confirm completion
 * from the lower MCU's rotate-done bit.
 *
 * It supports two move modes, selected per station in `Config::BELT_STATIONS`:
 *
 *  - **POSITION** (`Config::BeltMode::POSITION`) — for *mid* stations that have a
 *    well-defined encoder target with free travel on both sides. The vendor
 *    `rotateFor(speed, degree)` primitive is **relative** ("rotate N degrees from
 *    here"), so an absolute target is reached by reading `getDegrees()`, computing
 *    `delta = absTarget - current`, and issuing `rotateFor(sign(delta) * |moveSpeed|,
 *    |delta|)`. Completion is read from the encoder via `ChkRotateEnd(bool&)`. A
 *    bounded watchdog (`Config::BELT_MOVE_TIMEOUT_MS`) forces completion if the
 *    move stalls, so a jammed belt never hangs the drain.
 *
 *  - **TIMED** (`Config::BeltMode::TIMED`) — for *end* stations that stall the belt
 *    against a hard mechanical stop. Commanding an encoder degree into an end-stop
 *    would never report "rotate done" (the encoder cannot reach the target once the
 *    belt is jammed), so instead the belt runs at a fixed `setSpeed(speed)` for a
 *    fixed duration and completion is purely timer-based. The deliberate stall at the
 *    end is the intended behavior — the hard stop, not the encoder, defines the pose.
 *
 * **Reset + home routines.** There is no limit switch. `reset()` is the low-level primitive:
 * a TIMED stall toward the LEFT/HOME end-stop (negative speed per the sign convention) followed
 * by `resetCounter()` (which re-establishes the encoder zero that makes POSITION targets
 * repeatable). `home()` builds on it — it calls `reset()` then chains an automatic move on to
 * the MID rest / "home" position via `toStation(MID)` once the reset settles. The whole
 * sequence is driven by `update()`; `begin()` runs `home()` blocking-bounded at setup time, so
 * the belt powers up LEFT end-stop → encoder zero → MID.
 *
 * **Sign convention** (from `ConfigManipulator.h`): `+speed` drives toward FAR_END,
 * `-speed` drives toward HOME.
 *
 * @note Non-template class storing a runtime `_motorId` (the facade owns a plain
 *       `ManipBelt belt;` member). Hardware dispatch to the four distinct
 *       `MiniR4.M1..M4` template instances goes through small private `switch`
 *       helpers — `MiniR4.M1..M4` are different *types* (`MiniR4DC<1..4>`), so they
 *       cannot be indexed by a runtime id.
 *
 * @note `begin()` runs BEFORE any cooperative tick callback is registered (the
 *       sketch registers callbacks after every `begin()`), so the homing wait uses
 *       plain `millis()`/`delay()` bounded by `Config::BELT_HOME_TIMEOUT_MS` — it
 *       does NOT spin a tick callback. This is a permitted setup-time blocking site.
 *
 * @warning The manipulator physically moves on power-up: `begin()` drives the belt into its
 *          LEFT/HOME end-stop, re-zeros, then runs it out to the MID rest position. Keep
 *          hands clear at boot.
 */

#ifndef MANIPBELT_H
#define MANIPBELT_H

#include <Arduino.h>
#include <MatrixMiniR4.h>

#include "ConfigManipulator.h"

/**
 * @brief Encoder-DC-motor conveyor-belt actuator with POSITION + TIMED move modes.
 *
 * @see File-level docblock for the POSITION-vs-TIMED rationale, the stall-home
 *      procedure, the watchdog, and the sign convention.
 */
class ManipBelt
{
public:
    /**
     * @brief Construct the belt actuator. Does NO hardware work.
     *
     * Stores the station table, its length, and the motor channel id. Hardware
     * configuration and the first home are deferred to `begin()` (which must run
     * after `MiniR4.begin()` has auto-initialised M1..M4).
     *
     * @param table    Pointer to the per-station command table (typically
     *                 `Config::BELT_STATIONS`). Not copied — must outlive this object
     *                 (a `constexpr` table has static lifetime, so this is safe).
     * @param count    Number of entries in @p table (typically
     *                 `Config::BELT_STATION_COUNT`).
     * @param motorId  M-bus channel the belt motor is wired to (1..4); dispatched via
     *                 the private `switch` helpers to `MiniR4.M1..M4`.
     */
    explicit ManipBelt(const Config::BeltStationCfg* table, uint8_t count, uint8_t motorId)
        : _table(table), _count(count), _motorId(motorId)
    {
    }

    /**
     * @brief Configure the motor and stall-home the belt. Blocking, setup-time only.
     *
     * Sequence:
     *   1. `setPPR_RPM(Config::BELT_PPR, Config::BELT_RPM)` — encoder resolution + RPM
     *      ceiling for the lower-MCU speed/position loops.
     *   2. `setReverse(false)` — placeholder hardware direction (the sign convention is
     *      carried by speed; flip this at bring-up if the motor wiring is reversed).
     *   3. A BLOCKING, wall-clock-bounded `home()` (stall toward HOME, `resetCounter()`,
     *      settle, then drive to MID), polled by `update()` + `delay(2)` until `isDone()`
     *      or the `Config::BELT_HOME_TIMEOUT_MS` ceiling.
     *
     * @return `true` if the motor configured and the belt homed within the timeout;
     *         `false` if a vendor call persistently failed or homing timed out.
     *
     * @note Plain `delay()`/`millis()` are used deliberately — no cooperative tick
     *       callback is registered this early (see file docblock). This is a sanctioned
     *       setup-time blocking site.
     * @note Vendor `bool` calls are retried once after a 1 ms `delay()` per the project
     *       return-convention rule before being treated as a hard failure.
     */
    bool begin()
    {
        bool ok = _setPPR_RPM(Config::BELT_PPR, Config::BELT_RPM);
        ok = _setReverse(false) && ok;    // placeholder HW direction; sign carries direction

        // Full home (stall to LEFT/HOME, reset, drive to MID), blocking-bounded. begin() runs
        // before any tick callback is registered, so we cannot rely on Drive-style _drain —
        // poll update() directly. The ceiling covers BOTH home stages (stall + MID move).
        home();
        const uint32_t start = millis();
        while (!isDone() && (uint32_t)(millis() - start) < Config::BELT_HOME_TIMEOUT_MS) {
            update();
            delay(2);
        }

        if (!isDone()) {
            // Home never completed within the ceiling — force the belt to a safe stop
            // and report failure so the boot path can flag it.
            _setBrake(true);
            _state = State::IDLE;
            Serial.println(F("[ManipBelt] begin(): home timed out"));
            return false;
        }

        return ok;
    }

    /**
     * @brief Drive the belt to a logical station, using that station's configured mode.
     *
     * Indexes `_table[(uint8_t)s]`. A POSITION station is reached via `beltTo(cfg.deg)`
     * (absolute encoder target); a TIMED station via `runFor(cfg.ms, cfg.speed)`.
     *
     * @param s Logical station to drive to (`Config::BeltStation`).
     *
     * @note Out-of-range stations are rejected with a Serial warning and treated as
     *       already-done (no actuator write), so a blocking caller never hangs on a bad
     *       index.
     */
    void toStation(Config::BeltStation s)
    {
        const uint8_t idx = (uint8_t)s;
        if (_table == nullptr || idx >= _count) {
            Serial.print(F("[ManipBelt] toStation: index out of range: "));
            Serial.println((int)idx);
            _state = State::IDLE;    // already-done so a blocking drain does not stall
            return;
        }

        const Config::BeltStationCfg& cfg = _table[idx];
        if (cfg.mode == Config::BeltMode::POSITION) {
            beltTo(cfg.deg);
        } else {
            runFor(cfg.ms, cfg.speed);
        }
    }

    /**
     * @brief Start an absolute POSITION move to @p absDeg encoder degrees.
     *
     * `rotateFor` is RELATIVE, so the absolute target is converted to a signed relative
     * delta: `delta = absDeg - getDegrees()`. A near-zero delta is a no-op (already
     * there → IDLE). Otherwise the direction is carried in the sign of the speed
     * (`+|BELT_MOVE_SPEED|` toward FAR_END, `-|BELT_MOVE_SPEED|` toward HOME) and the
     * magnitude is `|delta|`. The move runs non-blocking; poll `isDone()` / call
     * `update()`.
     *
     * @param absDeg Absolute encoder target, measured from the homed zero.
     *
     * @note Targets are only repeatable relative to the zero established by `home()` +
     *       `resetCounter()`.
     * @note The watchdog (`Config::BELT_MOVE_TIMEOUT_MS`, enforced in `update()`) bounds
     *       this move so a stall cannot hang the drain.
     */
    void beltTo(int32_t absDeg)
    {
        _resetPending   = false;    // a non-reset move must not inherit a stale reset/home latch
        _pendingStation = -1;

        const int32_t current = _getDegrees();
        const int32_t delta   = absDeg - current;

        if (labs(delta) <= POS_DEADBAND_DEG) {
            // Already at the target (within deadband) — nothing to do.
            _state = State::IDLE;
            return;
        }

        const int16_t  mag    = (int16_t)abs(Config::BELT_MOVE_SPEED);
        const int16_t  speed  = (delta >= 0) ? mag : (int16_t)(-mag);
        const uint16_t degree = (uint16_t)labs(delta);

        if (!_rotateFor(speed, degree)) {
            Serial.println(F("[ManipBelt] beltTo: rotateFor failed"));
            _state = State::IDLE;    // treat as done so a blocking drain does not hang
            return;
        }

        _state   = State::POS_MOVE;
        _startMs = millis();
    }

    /**
     * @brief Start a TIMED move: run at @p speed for @p ms, completion by timer.
     *
     * Used for end stations that stall against a hard stop (where an encoder target
     * would never report done). The belt runs at the signed @p speed (sign selects
     * direction per the convention) until @p ms have elapsed, then `update()` stops and
     * brakes it. Runs non-blocking; poll `isDone()` / call `update()`.
     *
     * @param ms    Run duration in milliseconds.
     * @param speed Signed PWM (`+` toward FAR_END, `-` toward HOME); magnitude is power.
     */
    void runFor(uint32_t ms, int16_t speed)
    {
        _resetPending   = false;    // a non-reset move must not inherit a stale reset/home latch
        _pendingStation = -1;

        if (!_setSpeed(speed)) {
            Serial.println(F("[ManipBelt] runFor: setSpeed failed"));
            _state = State::IDLE;    // treat as done so a blocking drain does not hang
            return;
        }

        _state   = State::TIMED_MOVE;
        _startMs = millis();
        _durMs   = ms;
    }

    /**
     * @brief Stall into the LEFT/HOME end-stop and re-zero the encoder there. NON-blocking.
     *
     * The reset primitive: it runs the belt toward HOME at `-|Config::BELT_STALL_SPEED|`
     * (negative speed per the sign convention) as a TIMED move for `Config::BELT_RESET_MS`,
     * jamming it against the mechanical LEFT/HOME end-stop. When that timer elapses,
     * `update()` brakes the belt and calls `resetCounter()` so `getDegrees()` reads zero —
     * the reference that makes absolute POSITION targets repeatable.
     *
     * @note Non-blocking on its own (the stall is driven to completion by `update()`, which
     *       brakes + `resetCounter()`s on TIMED completion).
     * @note There is no limit switch — the mechanical end-stop, not a sensor, defines the
     *       LEFT/HOME reference. The brief stall at that end is intended.
     * @note The watchdog is the TIMED timer itself: `Config::BELT_RESET_MS` bounds the stall,
     *       so a wedged belt can never hang the reset.
     * @note Clears any pending chained station — a bare `reset()` re-zeroes only; use `home()`
     *       to also drive on to the MID rest position afterward.
     */
    void reset()
    {
        const int16_t mag = (int16_t)abs(Config::BELT_STALL_SPEED);
        if (!_setSpeed((int16_t)(-mag))) {
            Serial.println(F("[ManipBelt] reset: setSpeed failed"));
            // Cannot drive — surface as "not resetting"; begin() will time out and report.
            _state          = State::IDLE;
            _resetPending   = false;
            _pendingStation = -1;
            return;
        }

        _state          = State::TIMED_MOVE;
        _startMs        = millis();
        _durMs          = Config::BELT_RESET_MS;
        _resetPending   = true;    // tells update() to brake + resetCounter() on completion
        _pendingStation = -1;      // a bare reset() chains nothing; home() arms this after
    }

    /**
     * @brief Full home routine: `reset()` then a chained `toStation(MID)`. The single
     *        operating-home primitive.
     *
     * Calls `reset()` to establish the LEFT/HOME zero reference (stall into the end-stop +
     * `resetCounter()`), then arms a chained move on to the MID rest / "home" position. Once
     * `update()` brakes the reset stall and re-zeroes the encoder, it dwells through
     * `State::SETTLE` and finally issues the MID move via `toStation(MID)`. `isDone()` stays
     * `false` until that chained move finishes.
     *
     * `begin()` calls this at boot, so the belt powers up: LEFT end-stop → encoder zero → MID.
     *
     * @note Non-blocking on its own (the whole sequence is driven by `update()`); `begin()`
     *       wraps it in a wall-clock-bounded blocking poll (`Config::BELT_HOME_TIMEOUT_MS`,
     *       which covers the stall + settle + MID move).
     * @note If `reset()` could not start the stall, or the station table has no MID entry, the
     *       chain is skipped (reset only).
     */
    void home()
    {
        reset();                                   // establish the LEFT/HOME zero reference
        const uint8_t midIdx = (uint8_t)Config::BeltStation::MID;
        if (_state == State::TIMED_MOVE && _table != nullptr && midIdx < _count) {
            _pendingStation = (int8_t)midIdx;      // ...then drive on to MID once the reset settles
        }
    }

    /**
     * @brief Cooperative tick — advance any in-flight move. NON-blocking.
     *
     * Must be called every iteration of the owning module's tick while a move may be in
     * flight. Behaviour by state:
     *   - **TIMED_MOVE** — when `millis() - _startMs >= _durMs`, brake the drive to a stop.
     *     For a reset stall, `resetCounter()` (re-zero); if a station was chained (`home()`),
     *     enter **SETTLE** rather than IDLE. Any other TIMED completion returns to IDLE.
     *   - **SETTLE** — after `Config::BELT_HOME_SETTLE_MS` from the braked stall, issue the
     *     chained move via `toStation()` and enter POS_MOVE/TIMED (or IDLE if nothing was
     *     chained). The dwell gives the lower MCU a clean tick gap to switch from the stall's
     *     `setSpeed()` into the rotate (a same-tick stop->rotate failed to engage on hardware).
     *   - **POS_MOVE** — read `ChkRotateEnd(bool&)`; if the lower MCU reports the rotate
     *     is done, brake and return to IDLE. Otherwise, if `millis() - _startMs >=
     *     Config::BELT_MOVE_TIMEOUT_MS`, the watchdog fires: brake, return to IDLE, and
     *     emit a Serial warning (a stalled POSITION move must never hang the drain).
     *   - **IDLE** — no-op.
     *
     * @note Contains no `delay()`. Safe to call at any cadence.
     */
    void update()
    {
        switch (_state) {
            case State::TIMED_MOVE: {
                if ((uint32_t)(millis() - _startMs) >= _durMs) {
                    _setBrake(true);    // stop the timed drive (stall or end-station run)
                    if (_resetPending) {
                        _resetPending = false;
                        _resetCounter();    // re-establish the encoder zero at LEFT/HOME
                        if (_pendingStation >= 0) {
                            // A station was chained (home()): defer it to SETTLE. We just braked
                            // the stall's setSpeed() drive, so give the lower MCU a clean tick gap
                            // before issuing the chained move (back-to-back stop->rotate in ONE
                            // tick is what failed on hardware — the rotate never engaged). The
                            // dwell mirrors how ordinary moves are separated across ticks.
                            _state   = State::SETTLE;
                            _startMs = millis();
                            break;
                        }
                    }
                    _state = State::IDLE;
                }
                break;
            }

            case State::SETTLE: {
                if ((uint32_t)(millis() - _startMs) >= Config::BELT_HOME_SETTLE_MS) {
                    // Settled. Drive on to the chained station via toStation() — reusing the
                    // normal path (no copy-pasted rotate math). toStation() reads getDegrees(),
                    // but the 150 ms settle gives the lower MCU a clean gap after the reset brake
                    // so the post-reset encoder read is stable (this replaces the old
                    // relative-rotate-of-known-degree approach). Capture and clear the pending
                    // station first so toStation() does not inherit a stale latch.
                    const int8_t s = _pendingStation;
                    _pendingStation = -1;
                    if (s >= 0) { toStation((Config::BeltStation)s); }   // -> POS_MOVE/TIMED or IDLE
                    else { _state = State::IDLE; }
                }
                break;
            }

            case State::POS_MOVE: {
                bool ended = false;
                _chkEnd(ended);
                if (ended) {
                    _setBrake(true);
                    _state = State::IDLE;
                } else if ((uint32_t)(millis() - _startMs) >= Config::BELT_MOVE_TIMEOUT_MS) {
                    // Watchdog: the encoder never reported done — assume a stall/jam and
                    // force completion so the drain cannot hang.
                    _setBrake(true);
                    _state = State::IDLE;
                    Serial.println(F("[ManipBelt] update: POSITION move watchdog tripped"));
                }
                break;
            }

            case State::IDLE:
            default:
                break;
        }
    }

    /**
     * @brief Whether the belt is idle (no move in flight).
     * @return `true` iff the internal state machine is IDLE.
     * @note For POSITION moves this becomes `true` when `ChkRotateEnd` reports done or
     *       the watchdog fires; for TIMED moves when the run timer elapses.
     */
    bool isDone() const { return _state == State::IDLE; }

private:
    /**
     * @brief Internal move state.
     *
     * `IDLE` = no move; `POS_MOVE` = an absolute POSITION move issued via `rotateFor`,
     * completion read from `ChkRotateEnd` (watchdog-bounded); `TIMED_MOVE` = a fixed-speed
     * run (station end-stop or homing), completion by timer.
     */
    enum class State : uint8_t
    {
        IDLE,
        POS_MOVE,
        TIMED_MOVE,
        SETTLE    ///< Dwell between the braked reset stall and the chained station move.
    };

    /// @brief Encoder-degree tolerance below which a POSITION move is treated as already
    ///        complete (avoids issuing a zero-length `rotateFor`).
    static constexpr int32_t POS_DEADBAND_DEG = 2;

    const Config::BeltStationCfg* _table   = nullptr;    ///< Per-station command table (not owned).
    uint8_t                       _count   = 0;          ///< Number of entries in `_table`.
    uint8_t                       _motorId = 0;          ///< M-bus channel (1..4) for the belt motor.

    State    _state   = State::IDLE;    ///< Current move state.
    uint32_t _startMs = 0;              ///< `millis()` at the start of the in-flight move.
    uint32_t _durMs   = 0;              ///< TIMED-move duration (ms); unused in POS_MOVE.
    bool     _resetPending = false;     ///< TIMED move is a reset stall → resetCounter() on done.
    int8_t   _pendingStation = -1;      ///< -1 = none; else a `Config::BeltStation` index to chain after the reset settles.

    // -------------------------------------------------------------------------------------
    // Vendor dispatch helpers.
    //
    // MiniR4.M1..M4 are distinct template instances (MiniR4DC<1>..MiniR4DC<4>), i.e. four
    // different *types* — they cannot be indexed by a runtime id, so each call is a small
    // switch over `_motorId`. Every vendor method returns `bool` except getDegrees()
    // (int32_t). Per the project return convention, a `false` is retried once after a 1 ms
    // delay before being reported up.
    // -------------------------------------------------------------------------------------

    /**
     * @brief Issue a RELATIVE rotate (`rotateFor`) on the bound motor. Retry-once.
     * @param speed  Signed PWM (sign carries direction).
     * @param degree Magnitude of rotation in degrees.
     * @return `true` if the lower MCU accepted the command.
     */
    bool _rotateFor(int16_t speed, uint16_t degree)
    {
        bool ok = false;
        switch (_motorId) {
            case 1: ok = MiniR4.M1.rotateFor(speed, degree); break;
            case 2: ok = MiniR4.M2.rotateFor(speed, degree); break;
            case 3: ok = MiniR4.M3.rotateFor(speed, degree); break;
            case 4: ok = MiniR4.M4.rotateFor(speed, degree); break;
            default: return false;
        }
        if (!ok) {
            delay(1);
            switch (_motorId) {
                case 1: ok = MiniR4.M1.rotateFor(speed, degree); break;
                case 2: ok = MiniR4.M2.rotateFor(speed, degree); break;
                case 3: ok = MiniR4.M3.rotateFor(speed, degree); break;
                case 4: ok = MiniR4.M4.rotateFor(speed, degree); break;
                default: return false;
            }
        }
        return ok;
    }

    /**
     * @brief Read the rotate-done bit (`ChkRotateEnd`) for the bound motor.
     *
     * `ChkRotateEnd` is an out-param API, not a predicate return: the `bool&` it fills is
     * the rotate-done state, while its own `bool` return only reports whether the read
     * succeeded. On a failed read, @p isEnd is left as the caller initialised it (the
     * caller seeds it `false`), so a transient bus error never spuriously ends the move.
     *
     * @param[out] isEnd Set to the rotate-done state on a successful read.
     * @return `true` if the state was read successfully.
     */
    bool _chkEnd(bool& isEnd)
    {
        switch (_motorId) {
            case 1: return MiniR4.M1.ChkRotateEnd(isEnd);
            case 2: return MiniR4.M2.ChkRotateEnd(isEnd);
            case 3: return MiniR4.M3.ChkRotateEnd(isEnd);
            case 4: return MiniR4.M4.ChkRotateEnd(isEnd);
            default: return false;
        }
    }

    /**
     * @brief Read cumulative encoder degrees for the bound motor.
     * @return Encoder degrees (signed); `0` for an invalid motor id.
     */
    int32_t _getDegrees()
    {
        switch (_motorId) {
            case 1: return MiniR4.M1.getDegrees();
            case 2: return MiniR4.M2.getDegrees();
            case 3: return MiniR4.M3.getDegrees();
            case 4: return MiniR4.M4.getDegrees();
            default: return 0;
        }
    }

    /**
     * @brief Set the closed-loop speed (`setSpeed`) of the bound motor. Retry-once.
     * @param speed Signed PWM in `[-100, 100]`.
     * @return `true` if the lower MCU accepted the command.
     */
    bool _setSpeed(int16_t speed)
    {
        bool ok = false;
        switch (_motorId) {
            case 1: ok = MiniR4.M1.setSpeed(speed); break;
            case 2: ok = MiniR4.M2.setSpeed(speed); break;
            case 3: ok = MiniR4.M3.setSpeed(speed); break;
            case 4: ok = MiniR4.M4.setSpeed(speed); break;
            default: return false;
        }
        if (!ok) {
            delay(1);
            switch (_motorId) {
                case 1: ok = MiniR4.M1.setSpeed(speed); break;
                case 2: ok = MiniR4.M2.setSpeed(speed); break;
                case 3: ok = MiniR4.M3.setSpeed(speed); break;
                case 4: ok = MiniR4.M4.setSpeed(speed); break;
                default: return false;
            }
        }
        return ok;
    }

    /**
     * @brief Engage or release the brake (`setBrake`) on the bound motor. Retry-once.
     * @param brake `true` = active brake; `false` = coast.
     * @return `true` if the lower MCU accepted the command.
     */
    bool _setBrake(bool brake)
    {
        bool ok = false;
        switch (_motorId) {
            case 1: ok = MiniR4.M1.setBrake(brake); break;
            case 2: ok = MiniR4.M2.setBrake(brake); break;
            case 3: ok = MiniR4.M3.setBrake(brake); break;
            case 4: ok = MiniR4.M4.setBrake(brake); break;
            default: return false;
        }
        if (!ok) {
            delay(1);
            switch (_motorId) {
                case 1: ok = MiniR4.M1.setBrake(brake); break;
                case 2: ok = MiniR4.M2.setBrake(brake); break;
                case 3: ok = MiniR4.M3.setBrake(brake); break;
                case 4: ok = MiniR4.M4.setBrake(brake); break;
                default: return false;
            }
        }
        return ok;
    }

    /**
     * @brief Reset the encoder counter (`resetCounter`) of the bound motor to zero.
     *        Retry-once.
     * @return `true` if the lower MCU accepted the command.
     */
    bool _resetCounter()
    {
        bool ok = false;
        switch (_motorId) {
            case 1: ok = MiniR4.M1.resetCounter(); break;
            case 2: ok = MiniR4.M2.resetCounter(); break;
            case 3: ok = MiniR4.M3.resetCounter(); break;
            case 4: ok = MiniR4.M4.resetCounter(); break;
            default: return false;
        }
        if (!ok) {
            delay(1);
            switch (_motorId) {
                case 1: ok = MiniR4.M1.resetCounter(); break;
                case 2: ok = MiniR4.M2.resetCounter(); break;
                case 3: ok = MiniR4.M3.resetCounter(); break;
                case 4: ok = MiniR4.M4.resetCounter(); break;
                default: return false;
            }
        }
        return ok;
    }

    /**
     * @brief Configure encoder PPR + max RPM (`setPPR_RPM`) for the bound motor.
     *        Retry-once.
     * @param ppr Encoder pulses per revolution.
     * @param rpm Motor max RPM.
     * @return `true` if the lower MCU accepted the command.
     */
    bool _setPPR_RPM(uint16_t ppr, uint16_t rpm)
    {
        bool ok = false;
        switch (_motorId) {
            case 1: ok = MiniR4.M1.setPPR_RPM(ppr, rpm); break;
            case 2: ok = MiniR4.M2.setPPR_RPM(ppr, rpm); break;
            case 3: ok = MiniR4.M3.setPPR_RPM(ppr, rpm); break;
            case 4: ok = MiniR4.M4.setPPR_RPM(ppr, rpm); break;
            default: return false;
        }
        if (!ok) {
            delay(1);
            switch (_motorId) {
                case 1: ok = MiniR4.M1.setPPR_RPM(ppr, rpm); break;
                case 2: ok = MiniR4.M2.setPPR_RPM(ppr, rpm); break;
                case 3: ok = MiniR4.M3.setPPR_RPM(ppr, rpm); break;
                case 4: ok = MiniR4.M4.setPPR_RPM(ppr, rpm); break;
                default: return false;
            }
        }
        return ok;
    }

    /**
     * @brief Set the hardware direction (`setReverse`) of the bound motor. Retry-once.
     * @param reverse `true` = reverse, `false` = forward.
     * @return `true` if the lower MCU accepted the command.
     */
    bool _setReverse(bool reverse)
    {
        bool ok = false;
        switch (_motorId) {
            case 1: ok = MiniR4.M1.setReverse(reverse); break;
            case 2: ok = MiniR4.M2.setReverse(reverse); break;
            case 3: ok = MiniR4.M3.setReverse(reverse); break;
            case 4: ok = MiniR4.M4.setReverse(reverse); break;
            default: return false;
        }
        if (!ok) {
            delay(1);
            switch (_motorId) {
                case 1: ok = MiniR4.M1.setReverse(reverse); break;
                case 2: ok = MiniR4.M2.setReverse(reverse); break;
                case 3: ok = MiniR4.M3.setReverse(reverse); break;
                case 4: ok = MiniR4.M4.setReverse(reverse); break;
                default: return false;
            }
        }
        return ok;
    }
};

#endif    // MANIPBELT_H
