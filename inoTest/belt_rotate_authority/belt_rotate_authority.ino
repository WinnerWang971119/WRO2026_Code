/**
 * @file belt_rotate_authority.ino
 * @brief Find out why an STM32 `rotateFor` POSITION move delivers only a fraction of the
 *        degrees it is asked for, while the `setSpeed` TIMED path runs the same belt smoothly.
 *
 * ## Established so far
 *
 * Run 1 (`rotateFor(maxSpeed, 180)` at maxSpeed 20/40/70/100, brake latched and released, link
 * polled and silent) delivered 58-100 deg of the 180 asked for, every time, with peak speed
 * 230-264 deg/s against a `setSpeed(18)` reference of 266 deg/s and `ChkRotateEnd` never
 * latching. So the move is not slow, it is TRUNCATED: the belt sprints at full reference speed
 * for a fraction of a second, covers part of the target, and stops until the caller's watchdog
 * fires. Travel was flat across a 5x speed sweep, which rules out anything a speed cap could
 * explain — brake drag, PID gain starvation, PPR/MaxRPM scaling of the SPEED loop.
 *
 * Run 2 swept the `degree` argument instead, and was CONTAMINATED: trials deliver unequal
 * distances, so merely alternating direction let the belt random-walk into an end-stop. Its
 * `moving` times scattered 52-2502 ms with no relation to the target, the reference itself
 * degraded from 399 to 310 deg over the same 1.5 s command, and one row peaked ABOVE the
 * reference — the signature of a pinned belt breaking free, not of steady motion. Alternating
 * direction does not hold position; only returning to a known position does.
 *
 * ## What this revision does about it
 *
 * `mapTravel()` runs first: it drives gently into each end-stop, records both limits in encoder
 * degrees, and derives the usable span. Every trial then re-centres via `returnTo()` — R4-side
 * position control over the `setSpeed` path, which is the one primitive proven to work on this
 * belt — so each rotate starts from the same place with equal room on both sides. A target that
 * would not fit inside the span is SKIPPED and reported as skipped rather than silently run
 * against a limit, since that is exactly how run 2 produced numbers that looked like data.
 *
 * Mapping also self-calibrates the command/encoder sign, so nothing here depends on
 * `Config::BELT_DIR_SIGN` or `BELT_MOTOR_REVERSE` being set consistently.
 *
 * ## Reading the result
 *
 *   - **constant RATIO** across the sweep -> units mismatch between the rotate controller's
 *     target and the reported encoder frame;
 *   - **constant DISTANCE** -> the move is aborted early, independent of the target;
 *   - **constant `moving` ms** -> the move is cut by TIME, not distance;
 *   - **small targets complete, large ones do not** -> an internal ceiling (timeout, or a
 *     saturating target field on the wire).
 *
 * The motor is configured exactly as `ManipBelt::begin()` leaves it, and each rotate reproduces
 * production's brake -> `BELT_HOME_SETTLE_MS` dwell -> rotate ordering, so the verdict transfers.
 *
 * @warning The belt physically moves throughout, and mapping deliberately drives it into both
 *          end-stops at `Config::BELT_STALL_SPEED`. Keep hands clear.
 *
 * Build / upload / monitor:  uv run inoTest/belt_rotate_authority/build.py
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "TrialResult.h"

// --- the swept variable -----------------------------------------------------

/// @brief `degree` arguments swept by the main battery. Spans a 24x range so a constant ratio
///        and a constant distance cannot be confused for one another. Targets that do not fit
///        in the mapped span are skipped, not clamped.
static constexpr uint16_t SWEEP_DEGREES[] = { 30, 60, 90, 180, 360, 720 };
static constexpr uint8_t  SWEEP_COUNT     = sizeof(SWEEP_DEGREES) / sizeof(SWEEP_DEGREES[0]);

/// @brief Speed cap held FIXED across the sweep — run 1 proved distance does not depend on it,
///        so it is a control variable here, not a treatment.
static constexpr int16_t TRIAL_SPEED = Config::BELT_MOVE_SPEED;

/// @brief Repeat count for the fixed-target repeatability check on BTN_DOWN.
static constexpr uint8_t  REPEAT_COUNT  = 4;
/// @brief Target used by the repeatability check.
static constexpr uint16_t REPEAT_DEGREE = 180;

// --- travel mapping ---------------------------------------------------------

/// @brief Speed used to feel for the end-stops. Deliberately the gentle stall speed the belt
///        already homes with, so mapping is no rougher than a normal `reset()`.
static constexpr int16_t MAP_SPEED = Config::BELT_STALL_SPEED;

/// @brief Stillness that counts as "this end-stop is reached" while mapping.
static constexpr uint32_t STALL_DETECT_MS = 500;

/// @brief Ceiling on one end-stop seek, so a slipping belt cannot spin here forever.
static constexpr uint32_t MAP_TIMEOUT_MS = 12000;

/// @brief Keep trials this far clear of either end-stop. Run 2's failure mode was arriving at
///        a limit mid-trial, which is indistinguishable from the truncation under test.
static constexpr int32_t EDGE_MARGIN_DEG = 40;

// --- re-centring ------------------------------------------------------------

/// @brief Coarse speed for the bulk of a return-to-centre move.
static constexpr int16_t RETURN_SPEED = 30;

/// @brief Creep speed used inside `RETURN_SLOW_ZONE_DEG` of the target.
///
/// A single-speed bang-bang loop cannot settle: at `RETURN_SPEED` the belt covers ~18 deg per
/// sample, so any tolerance tighter than that is overshot every iteration and the loop dithers
/// until it times out. The creep phase is what makes the tolerance reachable.
static constexpr int16_t RETURN_CREEP_SPEED = 8;

/// @brief Distance from the target at which the loop drops to `RETURN_CREEP_SPEED`.
static constexpr int32_t RETURN_SLOW_ZONE_DEG = 250;

/// @brief Position tolerance for the return-to-centre move. Must comfortably exceed the
///        per-sample travel at `RETURN_CREEP_SPEED` (~2-3 deg at `RETURN_SAMPLE_MS`).
static constexpr int32_t CENTER_TOL_DEG = 25;

/// @brief Sampling cadence while re-centring. Tighter than `SAMPLE_MS` so the creep phase can
///        stop within tolerance.
static constexpr uint32_t RETURN_SAMPLE_MS = 20;

/// @brief Ceiling on a return-to-centre move. Must cover a full half-span at creep-in speed —
///        ~1900 deg here, which is over 5 s of coarse travel before settling even starts.
static constexpr uint32_t RETURN_TIMEOUT_MS = 20000;

/// @brief Stillness that aborts a re-centre as jammed, rather than burning the full timeout.
static constexpr uint32_t RETURN_STALL_MS = 900;

// --- motion detection -------------------------------------------------------

/// @brief Per-sample travel below which the belt counts as stopped.
static constexpr int32_t MOTION_EPS_DEG = 2;

/// @brief How long the belt must be still before a trial concludes the move has ended.
static constexpr uint32_t MOTION_IDLE_MS = 1200;

/// @brief Hard ceiling per trial. Mirrors `Config::BELT_MOVE_TIMEOUT_MS`, so a trial that runs
///        out here is the same event as the production watchdog.
static constexpr uint32_t ROT_TIMEOUT_MS = Config::BELT_MOVE_TIMEOUT_MS;

/// @brief Encoder sampling cadence; sets the resolution of `movingMs` and the peak read.
static constexpr uint32_t SAMPLE_MS = 50;

/// @brief Dwell between trials so the belt fully stops and the lower MCU sees a clean gap.
static constexpr uint32_t TRIAL_GAP_MS = 400;

// --- reference --------------------------------------------------------------

/// @brief Reference run uses the TIMED path's speed — the one the operator reports as smooth.
static constexpr int16_t  REF_SPEED = Config::BELT_STALL_SPEED;
/// @brief Reference run duration, kept short enough to stay inside half the mapped span.
static constexpr uint32_t REF_MS    = 1200;

static bool _prevUp   = false;
static bool _prevDown = false;

/// @brief Encoder degrees at each mapped end-stop, and the derived safe centre/half-span.
static int32_t _limitLow  = 0;
static int32_t _limitHigh = 0;
static int32_t _center    = 0;
static int32_t _halfRoom  = 0;      ///< Usable travel from centre to either side, less margin.
static bool    _mapped    = false;

/// @brief `+1` if a POSITIVE speed command makes `getDegrees()` increase, `-1` if it decreases.
///
/// Measured during mapping rather than taken from `Config::BELT_DIR_SIGN` /
/// `BELT_MOTOR_REVERSE`, so the probe stays correct no matter how those are set.
static int8_t _cmdSign = +1;

/// @brief Alternates the trial direction in the ENCODER frame, so consecutive trials pull the
///        belt opposite ways about the centre.
static int8_t _dir = +1;

/// @brief Reference peak speed (deg/s) — the authority the motor demonstrably has.
static float _refDegPerSec = 0.0f;

// ---------------------------------------------------------------------------
// Per-channel dispatch. M1..M4 are distinct template types, so each op is a switch.
// ---------------------------------------------------------------------------

static int32_t readDeg(uint8_t id)
{
    switch (id)
    {
    case 1:  return MiniR4.M1.getDegrees();
    case 2:  return MiniR4.M2.getDegrees();
    case 3:  return MiniR4.M3.getDegrees();
    case 4:  return MiniR4.M4.getDegrees();
    default: return 0;
    }
}

static void resetDeg(uint8_t id)
{
    switch (id)
    {
    case 1:  MiniR4.M1.resetCounter(); break;
    case 2:  MiniR4.M2.resetCounter(); break;
    case 3:  MiniR4.M3.resetCounter(); break;
    case 4:  MiniR4.M4.resetCounter(); break;
    default: break;
    }
}

static void setSpeedMotor(uint8_t id, int16_t speed)
{
    switch (id)
    {
    case 1:  MiniR4.M1.setSpeed(speed); break;
    case 2:  MiniR4.M2.setSpeed(speed); break;
    case 3:  MiniR4.M3.setSpeed(speed); break;
    case 4:  MiniR4.M4.setSpeed(speed); break;
    default: break;
    }
}

static void brakeMotor(uint8_t id, bool brake)
{
    switch (id)
    {
    case 1:  MiniR4.M1.setBrake(brake); break;
    case 2:  MiniR4.M2.setBrake(brake); break;
    case 3:  MiniR4.M3.setBrake(brake); break;
    case 4:  MiniR4.M4.setBrake(brake); break;
    default: break;
    }
}

static void rotateForMotor(uint8_t id, int16_t speed, uint16_t degree)
{
    switch (id)
    {
    case 1:  MiniR4.M1.rotateFor(speed, degree); break;
    case 2:  MiniR4.M2.rotateFor(speed, degree); break;
    case 3:  MiniR4.M3.rotateFor(speed, degree); break;
    case 4:  MiniR4.M4.rotateFor(speed, degree); break;
    default: break;
    }
}

static bool chkEnd(uint8_t id)
{
    bool ended = false;
    switch (id)
    {
    case 1:  MiniR4.M1.ChkRotateEnd(ended); break;
    case 2:  MiniR4.M2.ChkRotateEnd(ended); break;
    case 3:  MiniR4.M3.ChkRotateEnd(ended); break;
    case 4:  MiniR4.M4.ChkRotateEnd(ended); break;
    default: break;
    }
    return ended;
}

static void setPPR_RPM(uint8_t id, uint16_t ppr, uint16_t rpm)
{
    switch (id)
    {
    case 1:  MiniR4.M1.setPPR_RPM(ppr, rpm); break;
    case 2:  MiniR4.M2.setPPR_RPM(ppr, rpm); break;
    case 3:  MiniR4.M3.setPPR_RPM(ppr, rpm); break;
    case 4:  MiniR4.M4.setPPR_RPM(ppr, rpm); break;
    default: break;
    }
}

static void setMotorReverse(uint8_t id, bool reverse)
{
    switch (id)
    {
    case 1:  MiniR4.M1.setReverse(reverse); break;
    case 2:  MiniR4.M2.setReverse(reverse); break;
    case 3:  MiniR4.M3.setReverse(reverse); break;
    case 4:  MiniR4.M4.setReverse(reverse); break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
// Position keeping. All of this runs over `setSpeed` — the primitive proven to work.
// ---------------------------------------------------------------------------

/// @brief Bring the motor to a full stop and leave the brake ENGAGED, as production does.
static void parkBraked(uint8_t id)
{
    setSpeedMotor(id, 0);
    brakeMotor(id, true);
    delay(TRIAL_GAP_MS);
}

/**
 * @brief Drive at @p speed until the belt stops moving (an end-stop) or the ceiling expires.
 *
 * @param id    Motor channel.
 * @param speed Signed speed command; the sign is a raw COMMAND sign, not an encoder-frame sign.
 * @return Encoder degrees where motion ceased.
 */
static int32_t driveUntilStall(uint8_t id, int16_t speed)
{
    brakeMotor(id, false);
    setSpeedMotor(id, speed);

    const uint32_t startMs      = millis();
    int32_t        lastDeg      = readDeg(id);
    uint32_t       lastMotionMs = startMs;

    while ((uint32_t)(millis() - startMs) < MAP_TIMEOUT_MS)
    {
        delay(SAMPLE_MS);
        const int32_t nowDeg = readDeg(id);
        if (labs(nowDeg - lastDeg) >= MOTION_EPS_DEG) { lastMotionMs = millis(); }
        lastDeg = nowDeg;
        if ((uint32_t)(millis() - lastMotionMs) >= STALL_DETECT_MS) { break; }
    }

    parkBraked(id);
    return readDeg(id);
}

/**
 * @brief Drive the belt to an absolute encoder target with an R4-side loop over `setSpeed`.
 *
 * Deliberately does NOT use `rotateFor` — that is the primitive under investigation, and the
 * probe must not depend on it to set up its own trials.
 *
 * @param id     Motor channel.
 * @param target Absolute encoder degrees to settle at.
 * @return `true` if the target was reached within `CENTER_TOL_DEG` before the timeout.
 */
static bool returnTo(uint8_t id, int32_t target)
{
    const uint32_t startMs = millis();
    brakeMotor(id, false);

    int32_t  lastDeg      = readDeg(id);
    uint32_t lastMotionMs = startMs;
    bool     stalled      = false;

    while ((uint32_t)(millis() - startMs) < RETURN_TIMEOUT_MS)
    {
        const int32_t now = readDeg(id);
        const int32_t err = target - now;
        if (labs(err) <= CENTER_TOL_DEG) { break; }

        // Two-phase approach: coarse until close, then creep. A single-speed loop overshoots
        // the tolerance every sample and never settles.
        const int16_t mag =
            (labs(err) > RETURN_SLOW_ZONE_DEG) ? RETURN_SPEED : RETURN_CREEP_SPEED;
        // _cmdSign converts "which way the encoder must go" into "which way to command".
        const int16_t cmd = (int16_t)(_cmdSign * ((err > 0) ? mag : (int16_t)(-mag)));
        setSpeedMotor(id, cmd);

        delay(RETURN_SAMPLE_MS);

        if (labs(readDeg(id) - lastDeg) >= MOTION_EPS_DEG) { lastMotionMs = millis(); }
        lastDeg = readDeg(id);
        if ((uint32_t)(millis() - lastMotionMs) >= RETURN_STALL_MS) { stalled = true; break; }
    }

    parkBraked(id);

    const int32_t residual = target - readDeg(id);
    const bool    ok       = labs(residual) <= CENTER_TOL_DEG;
    if (!ok)
    {
        Serial.print(F("   [returnTo] "));
        Serial.print(stalled ? F("STALLED") : F("timeout"));
        Serial.print(F(" — target="));
        Serial.print((long)target);
        Serial.print(F(" at="));
        Serial.print((long)readDeg(id));
        Serial.print(F(" residual="));
        Serial.println((long)residual);
    }
    return ok;
}

/**
 * @brief Map both end-stops, derive the safe centre and half-span, and learn the command sign.
 *
 * Feels for one limit, then the other, at `MAP_SPEED`. The encoder delta observed on the first
 * seek also fixes `_cmdSign`, so no `Config` sign constant is trusted.
 *
 * @return `true` if a usable span was found.
 */
static bool mapTravel()
{
    const uint8_t id = Config::BELT_M_ID;

    Serial.println(F("--- mapping travel (drives into BOTH end-stops) ---"));

    const int32_t before = readDeg(id);
    const int32_t endA   = driveUntilStall(id, +MAP_SPEED);
    _cmdSign = (endA >= before) ? +1 : -1;

    const int32_t endB = driveUntilStall(id, -MAP_SPEED);

    _limitLow  = (endA < endB) ? endA : endB;
    _limitHigh = (endA < endB) ? endB : endA;
    const int32_t span = _limitHigh - _limitLow;
    _center    = _limitLow + span / 2;
    _halfRoom  = span / 2 - EDGE_MARGIN_DEG;

    Serial.print(F("   limits: "));
    Serial.print((long)_limitLow);
    Serial.print(F(" .. "));
    Serial.print((long)_limitHigh);
    Serial.print(F("   span="));
    Serial.print((long)span);
    Serial.print(F("deg   centre="));
    Serial.print((long)_center);
    Serial.print(F("   usable each way="));
    Serial.print((long)_halfRoom);
    Serial.print(F("deg   cmdSign="));
    Serial.println((int)_cmdSign);

    if (_halfRoom <= EDGE_MARGIN_DEG)
    {
        Serial.println(F("   !! span too small to trial anything. Check the belt is free."));
        _mapped = false;
        return false;
    }

    if (!returnTo(id, _center))
    {
        Serial.println(F("   !! could not reach centre after mapping — aborting."));
        _mapped = false;
        return false;
    }
    _mapped = true;
    Serial.print(F("   centred at "));
    Serial.println((long)readDeg(id));
    Serial.println();
    return true;
}

// ---------------------------------------------------------------------------
// Trials.
// ---------------------------------------------------------------------------

/**
 * @brief Watch the encoder until the belt stops moving, `ChkRotateEnd` latches, or time runs out.
 *
 * @param id      Motor channel under test.
 * @param pollEnd `true` to poll `ChkRotateEnd` each sample (what production does).
 * @return Travel, `movingMs`, peak speed, and whether the rotate reported done.
 *
 * @note Returns as soon as the belt has been still for `MOTION_IDLE_MS`, so a truncated move
 *       costs ~1.5 s of bench time instead of the full watchdog window.
 */
static TrialResult watchMove(uint8_t id, bool pollEnd)
{
    TrialResult r;

    const int32_t  startDeg     = readDeg(id);
    const uint32_t startMs      = millis();
    int32_t        lastDeg      = startDeg;
    uint32_t       lastMs       = startMs;
    uint32_t       lastMotionMs = startMs;

    while ((uint32_t)(millis() - startMs) < ROT_TIMEOUT_MS)
    {
        delay(SAMPLE_MS);

        const int32_t  nowDeg = readDeg(id);
        const uint32_t nowMs  = millis();
        const uint32_t dt     = nowMs - lastMs;
        const int32_t  step   = nowDeg - lastDeg;

        if (labs(step) >= MOTION_EPS_DEG)
        {
            lastMotionMs = nowMs;
            if (dt > 0)
            {
                const float v = (float)labs(step) * 1000.0f / (float)dt;
                if (v > r.peakDegPerSec) { r.peakDegPerSec = v; }
            }
        }

        lastDeg = nowDeg;
        lastMs  = nowMs;

        if (pollEnd && chkEnd(id)) { r.completed = true; break; }
        if ((uint32_t)(nowMs - lastMotionMs) >= MOTION_IDLE_MS) { break; }
    }

    r.elapsedMs = millis() - startMs;
    r.movingMs  = lastMotionMs - startMs;
    r.travelDeg = readDeg(id) - startDeg;
    return r;
}

/// @brief Print one trial as delivered-vs-commanded, the metric that actually discriminates.
static void report(const TrialResult& r)
{
    const long travel = labs(r.travelDeg);

    Serial.print(F("   cmd="));
    Serial.print((int)r.commandSign > 0 ? F("+") : F("-"));
    Serial.print(F(" enc="));
    Serial.print(r.travelDeg >= 0 ? F("+") : F("-"));
    Serial.print(F("  asked="));
    Serial.print((int)r.commandedDeg);
    Serial.print(F("deg  got="));
    Serial.print(travel);
    Serial.print(F("deg  ratio="));
    Serial.print(r.commandedDeg ? (100.0f * (float)travel / (float)r.commandedDeg) : 0.0f, 0);
    Serial.print(F("%  moving="));
    Serial.print((long)r.movingMs);
    Serial.print(F("ms  peak="));
    Serial.print(r.peakDegPerSec, 0);
    Serial.print(F(" deg/s"));
    if (_refDegPerSec > 1.0f)
    {
        Serial.print(F(" ("));
        Serial.print((r.peakDegPerSec / _refDegPerSec) * 100.0f, 0);
        Serial.print(F("% ref)"));
    }
    Serial.print(F("  rotateEnd="));
    Serial.println(r.completed ? F("YES") : F("no"));
}

/**
 * @brief Re-centre, then issue one `rotateFor` and measure what it delivers.
 *
 * @param degree The swept variable: how many degrees the move is asked for.
 *
 * @note Skips (and says so) when @p degree exceeds the mapped room, rather than running a trial
 *       that would end against an end-stop. A limit hit looks exactly like the truncation under
 *       test, which is how run 2 produced numbers that were not data.
 */
static void rotateTrial(uint16_t degree)
{
    const uint8_t id = Config::BELT_M_ID;

    if (!_mapped) { Serial.println(F("   (not mapped — press BTN_UP to map first)")); return; }

    if ((int32_t)degree > _halfRoom)
    {
        Serial.print(F("   asked="));
        Serial.print((int)degree);
        Serial.print(F("deg  SKIPPED — only "));
        Serial.print((long)_halfRoom);
        Serial.println(F("deg of room each way; a limit hit would masquerade as truncation"));
        return;
    }

    if (!returnTo(id, _center))
    {
        Serial.println(F("   (could not re-centre — skipping this trial)"));
        return;
    }

    // Production dwells BELT_HOME_SETTLE_MS between the braked stop and the chained rotate.
    parkBraked(id);
    delay(Config::BELT_HOME_SETTLE_MS);
    brakeMotor(id, false);
    delay(20);

    // _dir is an ENCODER-frame direction; _cmdSign turns it into a command sign.
    const int16_t cmd = (int16_t)(_cmdSign * (int16_t)(_dir * TRIAL_SPEED));
    rotateForMotor(id, cmd, degree);

    TrialResult r  = watchMove(id, true);
    r.commandedDeg = degree;
    r.commandSign  = (cmd >= 0) ? +1 : -1;
    parkBraked(id);
    _dir = (int8_t)(-_dir);

    report(r);
}

/// @brief Reference: the TIMED/`setSpeed` path the operator reports as smooth. Establishes the
///        speed the motor demonstrably has, so peak numbers below have a yardstick.
static void referenceTrial()
{
    const uint8_t id = Config::BELT_M_ID;

    Serial.print(F("--- REF  setSpeed("));
    Serial.print((int)REF_SPEED);
    Serial.print(F(") for "));
    Serial.print((long)REF_MS);
    Serial.println(F(" ms  (known-good TIMED path, from centre)"));

    if (!returnTo(id, _center)) { Serial.println(F("   (could not re-centre)")); return; }

    brakeMotor(id, false);
    setSpeedMotor(id, (int16_t)(_cmdSign * (int16_t)(_dir * REF_SPEED)));

    const int32_t  startDeg = readDeg(id);
    const uint32_t startMs  = millis();
    float          peak     = 0.0f;
    int32_t        lastDeg  = startDeg;
    uint32_t       lastMs   = startMs;
    while ((uint32_t)(millis() - startMs) < REF_MS)
    {
        delay(SAMPLE_MS);
        const int32_t  nowDeg = readDeg(id);
        const uint32_t nowMs  = millis();
        const uint32_t dt     = nowMs - lastMs;
        if (dt > 0)
        {
            const float v = (float)labs(nowDeg - lastDeg) * 1000.0f / (float)dt;
            if (v > peak) { peak = v; }
        }
        lastDeg = nowDeg;
        lastMs  = nowMs;
    }
    const uint32_t elapsed = millis() - startMs;
    const long     travel  = labs(readDeg(id) - startDeg);
    parkBraked(id);
    _dir = (int8_t)(-_dir);

    _refDegPerSec = peak;

    Serial.print(F("   travel="));
    Serial.print(travel);
    Serial.print(F("deg in "));
    Serial.print((long)elapsed);
    Serial.print(F("ms   mean="));
    Serial.print((float)travel * 1000.0f / (float)elapsed, 0);
    Serial.print(F(" deg/s   peak="));
    Serial.print(peak, 0);
    Serial.println(F(" deg/s  <- the authority the motor HAS"));
    Serial.println();
}

void setup()
{
    MiniR4.begin();
    Serial.begin(115200);
    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // Configure the belt motor EXACTLY as ManipBelt::begin() leaves it, or the verdict does
    // not transfer to the firmware.
    setPPR_RPM(Config::BELT_M_ID, Config::BELT_PPR, Config::BELT_RPM);
    setMotorReverse(Config::BELT_M_ID, Config::BELT_MOTOR_REVERSE);
    resetDeg(Config::BELT_M_ID);

    Serial.println();
    Serial.println(F("=== belt rotateFor truncation probe ==="));
    Serial.print(F("motor M"));
    Serial.print((int)Config::BELT_M_ID);
    Serial.print(F("  PPR="));
    Serial.print((int)Config::BELT_PPR);
    Serial.print(F("  maxRPM="));
    Serial.print((int)Config::BELT_RPM);
    Serial.print(F("  reverse="));
    Serial.print(Config::BELT_MOTOR_REVERSE ? F("true") : F("false"));
    Serial.print(F("  speed="));
    Serial.println((int)TRIAL_SPEED);
    Serial.print(F("battery="));
    Serial.print(MiniR4.PWR.getBattVoltage(), 2);
    Serial.println(F(" V"));
    Serial.println();
    Serial.println(F("BTN_UP   = map travel, then degree sweep 30/60/90/180/360/720"));
    Serial.println(F("BTN_DOWN = 4x repeat at 180 (needs a map first)"));
    Serial.println(F("Every trial re-centres first, so the belt cannot drift into a limit."));
    Serial.println();
}

void loop()
{
    const bool upNow    = MiniR4.BTN_UP.getState();
    const bool downNow  = MiniR4.BTN_DOWN.getState();
    const bool upEdge   = upNow && !_prevUp;
    const bool downEdge = downNow && !_prevDown;
    _prevUp   = upNow;
    _prevDown = downNow;

    if (downEdge)
    {
        Serial.println();
        Serial.print(F("--- repeatability: "));
        Serial.print((int)REPEAT_COUNT);
        Serial.print(F("x rotateFor("));
        Serial.print((int)REPEAT_DEGREE);
        Serial.println(F(") ---"));
        for (uint8_t i = 0; i < REPEAT_COUNT; ++i) { rotateTrial(REPEAT_DEGREE); }
        Serial.println(F("   (spread here bounds how much to trust one sweep row)"));
        Serial.println();
        return;
    }

    if (!upEdge) { return; }

    Serial.println();
    if (!mapTravel()) { return; }

    referenceTrial();

    Serial.println(F("--- degree sweep (speed fixed; only the target varies) ---"));
    for (uint8_t i = 0; i < SWEEP_COUNT; ++i) { rotateTrial(SWEEP_DEGREES[i]); }

    Serial.println();
    Serial.println(F("Read 'ratio' across the rows:"));
    Serial.println(F("  roughly CONSTANT %    -> units mismatch (rotate target vs encoder frame)"));
    Serial.println(F("  roughly CONSTANT deg  -> aborted early, independent of the target"));
    Serial.println(F("  100% then falls off   -> an internal ceiling (timeout / target saturation)"));
    Serial.println(F("And 'moving': the same ms on every row means the cut is by TIME, not distance."));
    Serial.println();
    Serial.println(F("=== sweep complete ==="));
    Serial.println();
}
