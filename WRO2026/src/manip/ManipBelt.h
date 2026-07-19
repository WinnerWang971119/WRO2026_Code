/**
 * @file ManipBelt.h
 * @brief Conveyor-belt actuator wrapper around a single LEGO encoder DC motor.
 *
 * @details
 * `ManipBelt` drives the manipulator conveyor belt. Unlike the RC servos
 * (`ManipServo`, which has no position feedback and fakes completion with a
 * timer), the belt motor is a LEGO encoder DC motor with REAL encoder feedback,
 * so this class can move it to absolute encoder targets and confirm arrival from
 * the encoder itself.
 *
 * It supports two move modes, selected per station in `Config::BELT_STATIONS`:
 *
 *  - **POSITION** (`Config::BeltMode::POSITION`) — for *mid* stations that have a
 *    well-defined encoder target with free travel on both sides. The position loop is
 *    closed **on the R4**, in `update()`: each tick reads `getDegrees()`, computes
 *    `err = target - current`, and drives `setSpeed()` toward it — coarse
 *    (`Config::BELT_MOVE_SPEED`) while further than `Config::BELT_POS_SLOW_ZONE_DEG`,
 *    then creeping (`Config::BELT_POS_CREEP_SPEED`) so it can actually stop inside
 *    `Config::BELT_POS_TOL_DEG`. A stall detector and the wall-clock watchdog
 *    (`Config::BELT_MOVE_TIMEOUT_MS`) both force completion, so a jammed belt never
 *    hangs the drain.
 *
 *    The lower MCU's own position primitive (`rotateFor` / `ChkRotateEnd`) is
 *    deliberately NOT used. Bench measurement on this belt
 *    (`inoTest/belt_rotate_authority`) found it delivers only ~43% of the commanded
 *    degrees before stopping — flat across a 5x speed sweep, with the rotate-done bit
 *    never latching — while the very same motor tracks an R4-side loop over `setSpeed`
 *    to within tolerance every time. It is closed vendor firmware, so the primitive is
 *    replaced rather than repaired.
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

#include "../../config/ConfigManipulator.h"
#include "../util/EncoderSource.h"    // Single owner of lower-MCU encoder reads (ADR-066).

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
     *   2. `setReverse(Config::BELT_MOTOR_REVERSE)` — selects which physical travel direction
     *      the hardware treats as a positive speed command, so `reset()`'s negative homing
     *      command drives into the correct end-stop. The matching encoder READ frame is carried
     *      by `Config::BELT_DIR_SIGN` (applied in `_getDegrees()` only).
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
        // Select which physical direction the hardware calls positive, so reset()'s negative
        // homing command drives into the correct end-stop. The encoder read frame that matches
        // it is carried by BELT_DIR_SIGN, applied in _getDegrees() only.
        ok = _setReverse(Config::BELT_MOTOR_REVERSE) && ok;

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
     * Arms the R4-side position loop and returns immediately — no drive command is issued
     * here. A target already inside `Config::BELT_POS_TOL_DEG` is a no-op (already there →
     * IDLE). Otherwise `update()` takes over: every tick it re-reads the position, re-derives
     * the direction from `target - current`, and drives coarse-then-creep until the error is
     * inside tolerance. The move runs non-blocking; poll `isDone()` / call `update()`.
     *
     * Deriving the direction from a fresh reading each tick — rather than committing to one at
     * issue time — is what makes the loop self-correcting: overshoot, backlash and stiction are
     * all absorbed on the next tick.
     *
     * @param absDeg Absolute logical target (HOME = 0, FAR_END positive), measured from the
     *               zero established by `home()` + `resetCounter()`.
     *
     * @note Targets are only repeatable relative to that homed zero.
     * @note Both the stall detector and the wall-clock watchdog
     *       (`Config::BELT_MOVE_TIMEOUT_MS`, enforced in `update()`) bound this move, so a jam
     *       cannot hang the drain.
     */
    void beltTo(int32_t absDeg)
    {
        _resetPending   = false;    // a non-reset move must not inherit a stale reset/home latch
        _pendingStation = -1;

        const int32_t current = _getDegrees();
        const int32_t delta   = absDeg - current;

        // TEMP bring-up telemetry: shows the actual encoder frame at move time so we can tell
        // a sign-frame bug (current reads the wrong sign) from a stale-position one.
        Serial.print(F("[ManipBelt] beltTo cur="));
        Serial.print((long)current);
        Serial.print(F(" tgt="));
        Serial.print((long)absDeg);
        Serial.print(F(" delta="));
        Serial.println((long)delta);

        if (labs(delta) <= Config::BELT_POS_TOL_DEG) {
            // Already at the target (within tolerance) — nothing to do.
            _state = State::IDLE;
            return;
        }

        // Arm the loop. No drive command is issued here: update() owns every write, so the
        // approach/creep/stop decision is made in exactly one place against a fresh reading.
        _target       = absDeg;
        _lastCmd      = 0;
        _motionRefDeg = current;
        _state        = State::POS_MOVE;
        _startMs      = millis();
        _lastMotionMs = _startMs;
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
     *     chained). The dwell lets the belt relax off the end-stop and the encoder reading
     *     settle before the chained move samples it as its starting position.
     *   - **POS_MOVE** — run one iteration of the R4-side position loop: read the position,
     *     stop and brake if the error is inside `Config::BELT_POS_TOL_DEG`, otherwise drive
     *     `setSpeed` toward the target at coarse or creep speed. Ends early if the belt stops
     *     moving for `Config::BELT_POS_STALL_MS` (jam) or if `millis() - _startMs >=
     *     Config::BELT_MOVE_TIMEOUT_MS` (watchdog); either way it brakes, returns to IDLE, and
     *     emits a Serial warning — a stalled POSITION move must never hang the drain.
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
                            // the stall's setSpeed() drive against a hard stop, so dwell before
                            // the chained move samples the encoder — the belt relaxes off the
                            // end-stop by a few degrees once the drive releases, and that
                            // settled reading is the zero every absolute target is measured from.
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
                    // normal path, so there is exactly one place that turns a station into a
                    // move. The dwell has let the belt relax off the end-stop, so the position
                    // toStation() samples is stable. Capture and clear the pending station
                    // first so toStation() does not inherit a stale latch.
                    const int8_t s = _pendingStation;
                    _pendingStation = -1;
                    if (s >= 0) { toStation((Config::BeltStation)s); }   // -> POS_MOVE/TIMED or IDLE
                    else { _state = State::IDLE; }
                }
                break;
            }

            case State::POS_MOVE: {
                const uint32_t now = millis();
                const int32_t  cur = _getDegrees();
                const int32_t  err = _target - cur;

                if (labs(err) <= Config::BELT_POS_TOL_DEG) {
                    _setSpeed(0);
                    _setBrake(true);
                    _state = State::IDLE;
                    break;
                }

                // Stall detection measures against a reference that only advances when real
                // motion is seen — never against the previous tick. At creep speed the belt
                // covers well under BELT_POS_MOTION_EPS_DEG in one ~8 ms tick, so a per-tick
                // comparison would read a healthy creep as a jam.
                if (labs(cur - _motionRefDeg) >= Config::BELT_POS_MOTION_EPS_DEG) {
                    _motionRefDeg = cur;
                    _lastMotionMs = now;
                }

                const bool stalled  = (uint32_t)(now - _lastMotionMs) >= Config::BELT_POS_STALL_MS;
                const bool timedOut =
                    (uint32_t)(now - _startMs) >= Config::BELT_MOVE_TIMEOUT_MS;
                if (stalled || timedOut) {
                    // Jam or overrun. Force completion so a blocking drain cannot hang — the
                    // belt reports done either way, matching the pre-existing watchdog contract.
                    _setSpeed(0);
                    _setBrake(true);
                    _state = State::IDLE;
                    Serial.print(F("[ManipBelt] update: POSITION move "));
                    Serial.print(stalled ? F("stalled") : F("watchdog tripped"));
                    Serial.print(F(" at "));
                    Serial.print((long)cur);
                    Serial.print(F(" (target "));
                    Serial.print((long)_target);
                    Serial.println(F(")"));
                    break;
                }

                // Coarse until close, then creep. The creep phase is what lets the belt stop
                // inside BELT_POS_TOL_DEG at all — see the constant's docblock.
                const int16_t mag = (labs(err) > Config::BELT_POS_SLOW_ZONE_DEG)
                                        ? Config::BELT_MOVE_SPEED
                                        : Config::BELT_POS_CREEP_SPEED;
                const int16_t cmd = (err > 0) ? mag : (int16_t)(-mag);

                // Only write when the command actually changes. The lower MCU holds the last
                // speed setpoint, and every write is a blocking SoftwareSerial round trip — so
                // a healthy move costs three writes (coarse, creep, stop) instead of one per tick.
                if (cmd != _lastCmd) {
                    _setSpeed(cmd);
                    _lastCmd = cmd;
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
     * @note For POSITION moves this becomes `true` when the position error falls inside
     *       `Config::BELT_POS_TOL_DEG`, or when the stall detector or watchdog fires; for TIMED
     *       moves when the run timer elapses.
     */
    bool isDone() const { return _state == State::IDLE; }

private:
    /**
     * @brief Internal move state.
     *
     * `IDLE` = no move; `POS_MOVE` = an absolute POSITION move driven by the R4-side loop in
     * `update()`, completion on position tolerance (stall- and watchdog-bounded);
     * `TIMED_MOVE` = a fixed-speed run (station end-stop or homing), completion by timer.
     */
    enum class State : uint8_t
    {
        IDLE,
        POS_MOVE,
        TIMED_MOVE,
        SETTLE    ///< Dwell between the braked reset stall and the chained station move.
    };

    const Config::BeltStationCfg* _table   = nullptr;    ///< Per-station command table (not owned).
    uint8_t                       _count   = 0;          ///< Number of entries in `_table`.
    uint8_t                       _motorId = 0;          ///< M-bus channel (1..4) for the belt motor.

    State    _state   = State::IDLE;    ///< Current move state.
    uint32_t _startMs = 0;              ///< `millis()` at the start of the in-flight move.
    uint32_t _durMs   = 0;              ///< TIMED-move duration (ms); unused in POS_MOVE.
    bool     _resetPending = false;     ///< TIMED move is a reset stall → resetCounter() on done.
    int8_t   _pendingStation = -1;      ///< -1 = none; else a `Config::BeltStation` index to chain after the reset settles.

    // --- POSITION-move loop state (POS_MOVE only) ---
    int32_t  _target       = 0;    ///< Absolute logical target the loop is driving toward.
    int16_t  _lastCmd      = 0;    ///< Last speed written; suppresses redundant bus writes.
    int32_t  _motionRefDeg = 0;    ///< Position at the last CONFIRMED motion (stall reference).
    uint32_t _lastMotionMs = 0;    ///< `millis()` at the last confirmed motion.

    // -------------------------------------------------------------------------------------
    // Vendor dispatch helpers.
    //
    // MiniR4.M1..M4 are distinct template instances (MiniR4DC<1>..MiniR4DC<4>), i.e. four
    // different *types* — they cannot be indexed by a runtime id, so each command helper is a
    // small switch over `_motorId`. Every such vendor method returns `bool`; per the project
    // return convention, a `false` is retried once after a 1 ms delay before being reported up.
    // The encoder READ is the exception — it is delegated to `EncoderSource.readDegreesOr`
    // (ADR-066), which owns the RESULT-surfacing + last-good policy centrally.
    // -------------------------------------------------------------------------------------

    /**
     * @brief Read the belt's LOGICAL position for the bound motor (HOME = 0, FAR_END positive).
     *
     * Routes through EncoderSource (ADR-066): the bare vendor `getDegrees()` silently returns
     * `0` on a lower-MCU comms timeout, which would corrupt the position error the loop drives
     * on. The last-good cache returns the previous reading on a transient miss instead. Fallback
     * `0` matches the post-`resetCounter()` physical zero (the belt homes to 0 at `begin()`, so
     * the cache is warm in normal operation).
     *
     * @return Logical position in encoder degrees; `0` for an invalid motor id or a cold-cache
     *         read miss.
     *
     * @note `Config::BELT_DIR_SIGN` is applied HERE AND ONLY HERE. Applying it to the speed
     *       commands as well would cancel it out of the loop entirely: the drive direction comes
     *       from `target - current`, so scaling both the read and the command by the same factor
     *       leaves the chosen direction unchanged. Read-only is what makes it a working knob.
     * @note A stale last-good read cannot spoof arrival on its own — it also freezes the stall
     *       reference, so a wedged bus ends the move through the stall path rather than by
     *       reporting a false position match.
     */
    int32_t _getDegrees()
    {
        const int32_t raw = EncoderSource.readDegreesOr(_motorId, 0);
        return (int32_t)(Config::BELT_DIR_SIGN * raw);
    }

    /**
     * @brief Set the closed-loop speed (`setSpeed`) of the bound motor. Retry-once.
     * @param speed Signed PWM in `[-100, 100]`.
     * @return `true` if the lower MCU accepted the command.
     */
    bool _setSpeed(int16_t speed)
    {
        // NO sign folding here — Config::BELT_DIR_SIGN belongs to the READ only (see
        // _getDegrees). Applying it on both sides cancels it out of every position calculation
        // and silently turns the knob into a no-op. Which physical direction the hardware calls
        // positive is selected by Config::BELT_MOTOR_REVERSE instead.
        const int16_t s = speed;
        bool          ok = false;
        switch (_motorId) {
            case 1: ok = MiniR4.M1.setSpeed(s); break;
            case 2: ok = MiniR4.M2.setSpeed(s); break;
            case 3: ok = MiniR4.M3.setSpeed(s); break;
            case 4: ok = MiniR4.M4.setSpeed(s); break;
            default: return false;
        }
        if (!ok) {
            delay(1);
            switch (_motorId) {
                case 1: ok = MiniR4.M1.setSpeed(s); break;
                case 2: ok = MiniR4.M2.setSpeed(s); break;
                case 3: ok = MiniR4.M3.setSpeed(s); break;
                case 4: ok = MiniR4.M4.setSpeed(s); break;
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
        // Invalidate the last-good cache after re-zeroing (ADR-066): a post-reset read that
        // fails must not serve the pre-reset position through `readDegreesOr`.
        EncoderSource.resetCache(_motorId);
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
