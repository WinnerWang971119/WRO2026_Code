/**
 * @file gyro-move.ino
 * @brief Standalone serial diagnostic for the "GYRO move corrects to the wrong side / drifts a lot" bug.
 * @author WRO 2026
 *
 * @details
 * This sketch is INTENTIONALLY self-contained — it depends only on <MatrixMiniR4.h>, NOT on the
 * WRO2026 firmware sources. Flash it on its own (Arduino IDE: board "Arduino UNO R4 WiFi", open
 * this folder). It talks to the STM32 coprocessor exactly the way the real Drive layer does
 * (MiniR4.DriveDC.MoveGyro + the lower-MCU IMU read), so what you measure here is what the
 * firmware experiences.
 *
 * WHY THIS EXISTS
 * ---------------
 * The GYRO heading loop runs ON the STM32: MoveGyro(power, Target) sends an ABSOLUTE raw-frame
 * yaw target; the STM32 reads its own IMU and drives the wheel differential to hold it. Our app
 * layer (YawSource / Config::IMU_YAW_SIGN) only sign-converts the target on the way in and the
 * yaw on the way out — and for a "hold the CURRENT heading" move those two sign multiplies CANCEL.
 * So flipping IMU_YAW_SIGN cannot change a hold-current GYRO move. If it still drifts, the cause is
 * one of:
 *   (A) WRONG GAINS  — the firmware ships `using ActiveMotor = MatrixTT;` (test-rover gyro PID,
 *                      Kp=6.01/Kd=2.15) while the competition chassis is LegoEV3 (Kp=0.52/Kd=0.15).
 *                      Gains ~11x too hot => the STM32 over-corrects => overshoot/oscillation/weave.
 *   (B) WRONG SIGN   — the STM32 correction direction is inverted relative to this chassis's motor
 *                      wiring (or IMU mount) => monotonic runaway (the true "corrects to wrong side").
 *   (C) IMU read sign — Config::IMU_YAW_SIGN does not match the real raw-IMU rotation sense.
 *
 * This tool measures (A), (B) and (C) directly so we fix the right thing instead of guessing.
 *
 * SERIAL COMMANDS (115200 baud, newline-terminated or single chars)
 * -----------------------------------------------------------------
 *   ?    print this help + current state
 *   i    IMU SIGN check — streams raw STM32 yaw and corrected app yaw; rotate the robot by hand
 *   e    select LegoEV3 gain profile  (Kp 0.52 / Kd 0.15, PPR 360) — the competition chassis
 *   t    select MatrixTT gain profile (Kp 6.01 / Kd 2.15, PPR 545) — the test rover (current default)
 *   h    GYRO hold-current-heading test (drives forward, should go straight)
 *   o    GYRO off-target test (target = current + OFFSET deg, should curve to it and hold)
 *   +/-  raise / lower the test forward power
 *   [/]  shorten / lengthen the off-target OFFSET
 *   s    STOP motors now (also: any key during a test aborts it)
 *
 * SAFETY: h/o drive the chassis FORWARD. Put it on the floor with ~1.5 m clear ahead, or up on
 *         blocks (wheels free) if you only want to watch the yaw trace. i needs no motion.
 *
 * MIRRORED CONSTANTS: the gain/PPR/sign values below mirror WRO2026/config/ConfigDrive.h
 *                     (MatrixTT / LegoEV3 / ActiveMotor) and ConfigNav.h (IMU_YAW_SIGN). They are
 *                     duplicated here ONLY so this sketch stays standalone — if you retune the real
 *                     Config, update these too.
 */

#include <MatrixMiniR4.h>

// ============================================================================================
// Mirrored config (keep in sync with WRO2026/config/*) — see file header note.
// ============================================================================================

/// @brief Battery pack cell count — MUST match the physical battery (ConfigGlobal::BATT_CELLS).
static constexpr uint8_t kBattCells = 2;

/// @brief Drive pair + direction flags (ConfigDrive: MOTOR_LEFT_ID/RIGHT_ID + *_REVERSE).
static constexpr uint8_t kLeftId = 2, kRightId = 3;
static constexpr bool    kLeftReverse = false, kRightReverse = true;

/// @brief IMU yaw sign that converts raw STM32 (CW+) yaw to app (CCW+) frame (ConfigNav::IMU_YAW_SIGN).
static constexpr float kImuYawSign = -1.0f;

/// @brief One selectable chassis gyro profile (mirrors a Config::ActiveMotor candidate).
struct GyroProfile
{
    const char* name;
    float       kp, ki, kd;    ///< MoveGyro PID (setMoveGyroPID)
    uint16_t    ppr, rpm;      ///< encoder spec (setPPR_RPM)
};

/// @brief Test-rover profile — the value the firmware currently ships (`using ActiveMotor = MatrixTT;`).
static const GyroProfile kProfileTT  = {"MatrixTT (test rover)", 6.01f, 0.0f, 2.15f, 545, 200};

/// @brief Competition profile — what the Lofaix LegoEV3 chassis should use.
static const GyroProfile kProfileEV3 = {"LegoEV3 (competition)", 0.52f, 0.0f, 0.15f, 360, 300};

// ============================================================================================
// Diagnostic driver
// ============================================================================================

/**
 * @brief Owns the probe state (active profile, test power, off-target offset) and the three tests.
 *
 * Kept as a small class rather than loose globals so each test is a named method and the shared
 * yaw-read / heading-error helpers live in one place.
 */
class GyroProbe
{
public:
    void begin()
    {
        _active = kProfileTT;    // start on whatever the firmware currently ships, so the A/B is honest
        applyProfile(_active);
        printBanner();
        printHelp();
    }

    /// @brief Poll Serial each loop and dispatch single-char commands.
    void poll()
    {
        if (!Serial.available())
        {
            return;
        }
        const int c = Serial.read();
        switch (c)
        {
        case '?': printHelp(); break;
        case 'i': imuSignCheck(); break;
        case 'e': _active = kProfileEV3; applyProfile(_active); break;
        case 't': _active = kProfileTT;  applyProfile(_active); break;
        case 'h': gyroTest(0.0f, "HOLD-CURRENT"); break;
        case 'o': gyroTest(_offsetDeg, "OFF-TARGET"); break;
        case '+': _power = constrain(_power + 5, 30, 90); reportKnobs(); break;
        case '-': _power = constrain(_power - 5, 30, 90); reportKnobs(); break;
        case ']': _offsetDeg = constrain(_offsetDeg + 5, 5, 90); reportKnobs(); break;
        case '[': _offsetDeg = constrain(_offsetDeg - 5, 5, 90); reportKnobs(); break;
        case 's': case 'x': stopMotors(); Serial.println(F("[stop]")); break;
        case '\n': case '\r': case ' ': break;    // ignore whitespace
        default: Serial.print(F("[?] unknown cmd '")); Serial.print((char) c); Serial.println(F("' — press ? for help")); break;
        }
    }

private:
    GyroProfile _active   = kProfileTT;
    int16_t     _power    = 45;     ///< forward PWM for the drive tests (vendor range -100..100)
    int16_t     _offsetDeg = 20;    ///< off-target test heading offset (deg, app frame)

    // ---- lower-MCU IMU read (faithful mirror of YawSource: read raw, retry once, apply sign) ----

    /// @brief Read RAW STM32 yaw (CW-positive, no sign applied). Retries once on a transient miss.
    bool readRawYaw(double& out) const
    {
        double          r = 0, p = 0, y = 0;
        MMLower::RESULT res = mmL.GetIMUEuler(r, p, y);
        if (res != MMLower::RESULT::OK)
        {
            delay(2);
            res = mmL.GetIMUEuler(r, p, y);
        }
        if (res != MMLower::RESULT::OK)
        {
            return false;
        }
        out = y;
        return true;
    }

    /// @brief Corrected app-frame yaw = raw * IMU_YAW_SIGN (what readYawDeg() returns).
    bool readAppYaw(double& out) const
    {
        double raw = 0;
        if (!readRawYaw(raw))
        {
            return false;
        }
        out = kImuYawSign * raw;
        return true;
    }

    /// @brief app -> raw heading for MoveGyro's Target (what toHwHeading() does).
    static int16_t toHwHeading(double appDeg) { return (int16_t) (kImuYawSign * appDeg); }

    /// @brief Shortest signed difference a-b folded into (-180, +180].
    static float normDelta(float d)
    {
        while (d > 180.0f) d -= 360.0f;
        while (d <= -180.0f) d += 360.0f;
        return d;
    }

    /// @brief Discard Serial line-ending whitespace and report any real abort character.
    bool abortRequested() const
    {
        bool requested = false;
        while (Serial.available())
        {
            const int c = Serial.read();
            if (c != '\r' && c != '\n' && c != ' ')
            {
                Serial.print(F("[abort byte] dec="));
                Serial.print(c);
                Serial.print(F(" hex=0x"));
                Serial.println(c, HEX);
                requested = true;
            }
        }
        return requested;
    }

    // ---- actuator helpers ----

    void applyProfile(const GyroProfile& pr)
    {
        MiniR4.M2.setPPR_RPM(pr.ppr, pr.rpm);
        MiniR4.M3.setPPR_RPM(pr.ppr, pr.rpm);
        const bool ok = MiniR4.DriveDC.setMoveGyroPID(pr.kp, pr.ki, pr.kd);
        Serial.print(F("[profile] -> "));
        Serial.print(pr.name);
        Serial.print(F("  Kp=")); Serial.print(pr.kp);
        Serial.print(F(" Kd="));  Serial.print(pr.kd);
        Serial.print(F(" PPR=")); Serial.print(pr.ppr);
        Serial.println(ok ? F("  (PID pushed OK)") : F("  (PID push FAILED!)"));
    }

    void stopMotors()
    {
        MiniR4.DriveDC.Move(0, 0);
        MiniR4.DriveDC.brake(true);
    }

    // ---- Test 1: IMU sign ----

    /**
     * @brief Stream raw vs corrected yaw for ~10 s. Rotate the robot COUNTER-CLOCKWISE by hand.
     *
     * Verdict: in the app (CCW-positive) frame, a CCW rotation MUST make the corrected yaw INCREASE.
     * If it decreases, Config::IMU_YAW_SIGN has the wrong sense for this chassis.
     */
    void imuSignCheck()
    {
        Serial.println(F("\n=== IMU SIGN CHECK ==="));
        Serial.println(F("Rotate the robot COUNTER-CLOCKWISE (viewed from above) ~90 deg over the next 10 s."));
        Serial.print(F("Configured IMU_YAW_SIGN = ")); Serial.println(kImuYawSign);
        Serial.println(F("Expect: as you turn CCW, 'app' should INCREASE. If it DECREASES, the sign is wrong."));
        Serial.println(F("  t(ms)    raw(CW+)   app(CCW+)"));

        const uint32_t t0 = millis();
        double         appStart = 0;
        bool           haveStart = readAppYaw(appStart);
        while (millis() - t0 < 10000)
        {
            if (abortRequested()) { break; }    // any non-whitespace key aborts
            double raw = 0, app = 0;
            if (readRawYaw(raw) && readAppYaw(app))
            {
                Serial.print(F("  "));
                Serial.print(millis() - t0);
                Serial.print(F("\t"));
                Serial.print(raw, 2);
                Serial.print(F("\t"));
                Serial.println(app, 2);
            }
            delay(150);
        }
        double appEnd = 0;
        if (haveStart && readAppYaw(appEnd))
        {
            const float d = normDelta((float) (appEnd - appStart));
            Serial.print(F("net app delta = ")); Serial.print(d, 1); Serial.println(F(" deg"));
            Serial.println(d > 5.0f
                               ? F("VERDICT: app rose on CCW -> IMU_YAW_SIGN looks CORRECT.")
                               : (d < -5.0f
                                      ? F("VERDICT: app FELL on CCW -> IMU_YAW_SIGN is WRONG (flip it).")
                                      : F("VERDICT: too little rotation to tell — rerun and turn ~90 deg CCW.")));
        }
        Serial.println(F("=== end ===\n"));
    }

    // ---- Tests 2 & 3: gyro loop behaviour ----

    /**
     * @brief Drive forward under MoveGyro holding (current + offsetDeg), sampling heading error.
     *
     * @param offsetDeg 0 = hold current heading (should go straight); >0 = step the target (should
     *                  curve onto it and settle).
     *
     * Classifies the run from the error trace:
     *   - small, settled error              => HEALTHY loop
     *   - large error, many sign flips       => OSCILLATION (gains too hot — e.g. TT gains on EV3)
     *   - error grows, no sign flips         => RUNAWAY (correction sign inverted)
     */
    void gyroTest(float offsetDeg, const char* label)
    {
        double cur = 0;
        if (!readAppYaw(cur))
        {
            Serial.println(F("[err] cannot read IMU yaw — aborting test."));
            return;
        }
        const double target = cur + offsetDeg;

        Serial.print(F("\n=== GYRO ")); Serial.print(label); Serial.println(F(" TEST ==="));
        Serial.print(F("profile=")); Serial.print(_active.name);
        Serial.print(F("  power=")); Serial.print(_power);
        Serial.print(F("  offset=")); Serial.print(offsetDeg, 0);
        Serial.println(F(" deg"));
        Serial.print(F("start app yaw=")); Serial.print(cur, 1);
        Serial.print(F("  target app yaw=")); Serial.println(target, 1);
        Serial.println(F("Driving... (any key aborts).   t(ms)  appYaw  err"));

        const uint32_t t0 = millis();
        const uint32_t durMs = 2500;
        float          maxAbs = 0, firstAbs = -1, lastAbs = 0, prevErr = 0;
        uint8_t        signFlips = 0;
        bool           first = true;

        while (millis() - t0 < durMs)
        {
            if (abortRequested()) { Serial.println(F("[aborted]")); break; }

            double app = 0;
            if (!readAppYaw(app))
            {
                continue;    // transient miss — hold last command, do not act on a fake 0
            }
            const float err = normDelta((float) (target - app));

            MiniR4.DriveDC.MoveGyro(_power, toHwHeading(target));

            const float a = fabs(err);
            if (a > maxAbs) maxAbs = a;
            if (firstAbs < 0) firstAbs = a;
            lastAbs = a;
            if (!first && ((err > 0) != (prevErr > 0)) && fabs(err) > 1.0f) signFlips++;
            prevErr = err;
            first   = false;

            Serial.print(F("  ")); Serial.print(millis() - t0);
            Serial.print(F("\t")); Serial.print(app, 1);
            Serial.print(F("\t")); Serial.println(err, 1);
            delay(60);
        }

        stopMotors();

        Serial.print(F("maxErr=")); Serial.print(maxAbs, 1);
        Serial.print(F("  firstErr=")); Serial.print(firstAbs, 1);
        Serial.print(F("  lastErr=")); Serial.print(lastAbs, 1);
        Serial.print(F("  signFlips=")); Serial.println(signFlips);

        // Heuristic verdict — read the printed trace too; these thresholds are deliberately loose.
        if (maxAbs < 8.0f && lastAbs < 8.0f)
        {
            Serial.println(F("VERDICT: loop HEALTHY (held heading). These gains fit this chassis."));
        }
        else if (signFlips >= 3 && maxAbs >= 8.0f)
        {
            Serial.println(F("VERDICT: OSCILLATION — gains too hot for this chassis."));
            Serial.println(F("         If you're on the EV3 chassis with the TT profile, press 'e' and retest."));
        }
        else if (signFlips <= 1 && lastAbs > firstAbs + 8.0f)
        {
            Serial.println(F("VERDICT: RUNAWAY — correction drives the WRONG way (inverted sign)."));
            Serial.println(F("         Not fixable via IMU_YAW_SIGN (cancels here). Suspect motor wiring /"));
            Serial.println(F("         reverse flags or IMU mount vs the STM32 gyro-correction sign."));
        }
        else
        {
            Serial.println(F("VERDICT: AMBIGUOUS — eyeball the trace; try the other profile / a lower power."));
        }
        Serial.println(F("=== end ===\n"));
    }

    // ---- banners ----

    void printBanner()
    {
        Serial.println(F("\n############################################################"));
        Serial.println(F("#  WRO2026 GYRO move diagnostic (test/gyro-move)            #"));
        Serial.println(F("############################################################"));
        Serial.print(F("IMU_YAW_SIGN = ")); Serial.println(kImuYawSign);
        Serial.println(F("NOTE: firmware currently ships `using ActiveMotor = MatrixTT;`."));
        Serial.println(F("      If this chassis is the competition LegoEV3, that profile is WRONG"));
        Serial.println(F("      (gyro Kp ~11x too hot). Press 'e' to load EV3 gains, then 'h' to retest."));
    }

    void reportKnobs()
    {
        Serial.print(F("[knobs] power=")); Serial.print(_power);
        Serial.print(F("  offset=")); Serial.print(_offsetDeg); Serial.println(F(" deg"));
    }

    void printHelp()
    {
        Serial.println(F("\n-- commands --"));
        Serial.println(F("  ?  help + state        i  IMU sign check (rotate by hand)"));
        Serial.println(F("  e  EV3 gains           t  TT gains (current firmware default)"));
        Serial.println(F("  h  hold-current test   o  off-target test"));
        Serial.println(F("  +/- power             [/] offset       s/x  STOP"));
        Serial.print(F("[state] profile=")); Serial.print(_active.name);
        Serial.print(F("  power=")); Serial.print(_power);
        Serial.print(F("  offset=")); Serial.print(_offsetDeg); Serial.println(F(" deg"));
    }
};

GyroProbe probe;

void setup()
{
    MiniR4.begin();                          // STM32, IMU, motors, servos, OLED
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }   // give USB CDC a moment, but don't hang headless
    MiniR4.PWR.setBattCell(kBattCells);

    // Match the firmware's per-motor setup before DriveDC.begin (Drive.cpp:45-53).
    MiniR4.M2.setReverse(kLeftReverse);
    MiniR4.M3.setReverse(kRightReverse);

    const uint8_t rc = MiniR4.DriveDC.begin(kLeftId, kRightId, kLeftReverse, kRightReverse);
    if (rc != 0x00)
    {
        Serial.print(F("[FATAL] DriveDC.begin failed rc=0x"));
        Serial.println(rc, HEX);
    }

    probe.begin();
}

void loop()
{
    probe.poll();
}
