/**
 * @file belt_encoder_probe.ino
 * @brief Characterize the belt's encoder vs. the STM32 closed-loop `rotateFor`.
 *
 * Findings so far: the belt encoder is on `Config::BELT_M_ID` (co-located with power) but
 * counts OPPOSITE the motor (drive +30 -> count -290). `rotateFor` is closed-loop ON the
 * STM32 (drive at speed, count that channel's encoder to target), so a motor/encoder
 * disagreement makes it run away -> the belt's POSITION move hangs to the watchdog. Setting
 * `SetEncoderDir(REVERSE)` in firmware did NOT fix it.
 *
 * `SetEncoderDir(REVERSE)` did NOT rescue the loop (the STM32 rotate controller ignores it).
 * The remaining question: since the ENCODER is the reversed side, does flipping the MOTOR
 * instead — `setReverse(true)`, which uses the working `SetDCMotorDir` — bring motor + encoder
 * into agreement so `rotateFor` completes? BTN_UP runs the same `rotateFor(+ROT_SPEED, ROT_DEG)`
 * with the motor at its default direction, then with `setReverse(true)`, holding the encoder at
 * its default FORWARD sense so only the motor direction varies.
 *
 *   - completes under `setReverse(true)`  -> the fix is `setReverse(true)` + a BELT_DIR_SIGN /
 *                                            read-sign flip; fully software, no rewiring.
 *   - completes under NEITHER             -> `rotateFor` is unusable with this wiring; move
 *                                            position control onto the R4 (bypass rotateFor).
 *
 * BTN_DOWN keeps the open-loop nudge (drive the firing motor, read all four encoders) for
 * re-checking the channel/sign. It drives via `setPower` (raw PWM), NOT `setSpeed`: the latter
 * is STM32 closed-loop and therefore runs through the very encoder feedback under test, so on
 * a dead encoder it can output nothing and mimic a dead motor. `setup()` also applies
 * `setPPR_RPM` exactly as `ManipBelt::begin()` does, so the closed-loop test below runs in the
 * same motor configuration the firmware ships.
 *
 * @warning The belt physically moves on every test. Keep hands clear.
 *
 * Build / upload / monitor:  uv run inoTest/belt_encoder_probe/build.py
 */

#include <MatrixMiniR4.h>

#include "Config.h"

// --- open-loop nudge tunables (BTN_DOWN) ------------------------------------
static constexpr int16_t  NUDGE_POWER  = 30;    ///< open-loop PWM power for the read test.
static constexpr uint32_t NUDGE_MS     = 700;   ///< how long the nudge drives before braking.
static constexpr int32_t  MOVE_EPS_DEG = 5;     ///< |delta| below this counts as "no movement".

// --- closed-loop rotate test tunables (BTN_UP) ------------------------------
static constexpr int16_t  ROT_SPEED      = 30;     ///< rotateFor speed (PWM, positive).
static constexpr uint16_t ROT_DEG        = 360;    ///< rotateFor target magnitude (degrees).
static constexpr uint32_t ROT_TIMEOUT_MS = 5000;   ///< give up polling ChkRotateEnd after this.

static constexpr uint32_t STREAM_MS = 400;    ///< idle encoder-stream cadence.

static bool     _prevUp   = false;
static bool     _prevDown = false;
static uint32_t _lastStreamMs = 0;

// --- per-channel dispatch (M1..M4 are distinct types, so switch on the id) ---

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

/// @brief Drive the motor UNREGULATED (`setPower`, the EV3 "power" block).
///
/// Deliberately not `setSpeed`: that one is closed-loop ("the Lower MCU will control the
/// motor PWM to try keep the speed you set"), so it drives THROUGH the encoder feedback this
/// probe exists to characterize — with a dead or miswired encoder it can output nothing at
/// all, which reads identically to a dead motor. `setPower` is raw PWM and always turns a
/// healthy motor, so a still belt here means the MOTOR side is at fault, full stop.
static void powerMotor(uint8_t id, int16_t power)
{
    switch (id)
    {
    case 1:  MiniR4.M1.setPower(power); break;
    case 2:  MiniR4.M2.setPower(power); break;
    case 3:  MiniR4.M3.setPower(power); break;
    case 4:  MiniR4.M4.setPower(power); break;
    default: break;
    }
}

/// @brief Configure encoder PPR + max RPM, exactly as `ManipBelt::begin()` does.
///
/// `rotateFor` and `setSpeed` are both STM32 closed-loop commands scaled by this pair. Without
/// it the probe tests the motor in a different configuration than the firmware ships, and its
/// verdict does not transfer.
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

static void resetAll()
{
    for (uint8_t id = 1; id <= 4; ++id) { resetDeg(id); }
}

static void streamCounts()
{
    Serial.print(F("enc  "));
    for (uint8_t id = 1; id <= 4; ++id)
    {
        Serial.print('M');
        Serial.print((int)id);
        Serial.print(id == Config::BELT_M_ID ? F("*=") : F("="));
        Serial.print((long)readDeg(id));
        Serial.print(F("  "));
    }
    Serial.println();
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

/// @brief Flip the MOTOR power direction (setReverse), then run one rotateFor and report.
///
/// Holds the STM32 encoder at its default FORWARD sense and varies ONLY the motor direction,
/// so it isolates whether flipping the motor (the working SetDCMotorDir half of setReverse)
/// brings motor + encoder into agreement and lets the closed-loop rotate complete.
static void rotateTest(bool reverseMotor)
{
    const uint8_t fire = Config::BELT_M_ID;

    Serial.print(F(">> setReverse("));
    Serial.print(reverseMotor ? F("true ") : F("false"));
    Serial.print(F(")  then rotateFor(+"));
    Serial.print((int)ROT_SPEED);
    Serial.print(F(", "));
    Serial.print((int)ROT_DEG);
    Serial.println(F(")"));

    mmL.SetEncoderDir(fire, MMLower::DIR::FORWARD);    // hold encoder at default; vary only motor
    setMotorReverse(fire, reverseMotor);
    delay(50);
    resetDeg(fire);
    delay(50);

    rotateForMotor(fire, ROT_SPEED, ROT_DEG);
    const uint32_t start = millis();
    bool           ended = false;
    while ((uint32_t)(millis() - start) < ROT_TIMEOUT_MS)
    {
        if (chkEnd(fire)) { ended = true; break; }
        delay(20);
    }
    const uint32_t elapsed = millis() - start;
    powerMotor(fire, 0);
    brakeMotor(fire, true);
    setMotorReverse(fire, false);    // restore default motor direction
    delay(150);

    Serial.print(F("   completed="));
    Serial.print(ended ? F("YES") : F("NO (timeout)"));
    Serial.print(F("   elapsed="));
    Serial.print((long)elapsed);
    Serial.print(F("ms   finalDeg="));
    Serial.print((long)readDeg(fire));
    Serial.println(ended ? F("   <-- rotateFor WORKS this way") : F(""));
}

/// @brief Open-loop nudge: drive the firing motor UNREGULATED, read which encoder moved.
static void runNudge(int16_t power)
{
    const uint8_t fire = Config::BELT_M_ID;

    Serial.print(F(">> open-loop nudge (setPower) M"));
    Serial.print((int)fire);
    Serial.print(F(" at "));
    Serial.print((int)power);
    Serial.print(F(" for "));
    Serial.print((long)NUDGE_MS);
    Serial.println(F(" ms"));

    resetAll();
    powerMotor(fire, power);
    const uint32_t start = millis();
    while ((uint32_t)(millis() - start) < NUDGE_MS) { delay(20); }
    powerMotor(fire, 0);
    brakeMotor(fire, true);
    delay(150);

    uint8_t moved = 0;
    int32_t best  = 0;
    for (uint8_t id = 1; id <= 4; ++id)
    {
        const int32_t d = readDeg(id);
        Serial.print(F("   M"));
        Serial.print((int)id);
        Serial.print(id == fire ? F(" (firing)  delta=") : F("           delta="));
        Serial.println((long)d);
        if (labs(d) > labs(best)) { best = d; moved = id; }
    }
    if (moved == 0 || labs(best) < MOVE_EPS_DEG) { Serial.println(F("   -> belt did not move")); }
    else if (moved != fire)
    {
        Serial.print(F("   -> CHANNEL MISWIRE: encoder on M"));
        Serial.println((int)moved);
    }
    else
    {
        Serial.print(F("   -> encoder co-located, sign vs drive: "));
        Serial.println(((best > 0) == (power > 0)) ? F("SAME") : F("OPPOSITE"));
    }
    Serial.println();
}

void setup()
{
    MiniR4.begin();
    Serial.begin(115200);
    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // Match ManipBelt::begin()'s motor configuration — rotateFor and setSpeed are both STM32
    // closed-loop commands scaled by PPR/max-RPM, so without this the probe characterizes a
    // differently-configured motor than the firmware ships.
    setPPR_RPM(Config::BELT_M_ID, Config::BELT_PPR, Config::BELT_RPM);
    resetAll();

    Serial.println();
    Serial.println(F("=== belt encoder / rotateFor probe ==="));
    Serial.print(F("firing motor (Config::BELT_M_ID) = M"));
    Serial.println((int)Config::BELT_M_ID);
    Serial.print(F("PPR="));
    Serial.print((int)Config::BELT_PPR);
    Serial.print(F("  maxRPM="));
    Serial.println((int)Config::BELT_RPM);
    Serial.println(F("BTN_UP   = rotateFor completion test (motor default, then setReverse(true))"));
    Serial.println(F("BTN_DOWN = open-loop nudge via setPower (which encoder channel moves)"));
    Serial.println();
}

void loop()
{
    const bool upNow   = MiniR4.BTN_UP.getState();
    const bool downNow = MiniR4.BTN_DOWN.getState();
    const bool upEdge   = upNow && !_prevUp;
    const bool downEdge = downNow && !_prevDown;
    _prevUp   = upNow;
    _prevDown = downNow;

    if (upEdge)
    {
        Serial.println(F("\nStart with the belt near mid-travel (the two runs drive opposite ways)."));
        rotateTest(false);    // motor default
        rotateTest(true);     // motor reversed (candidate fix)
        Serial.println(F("(setReverse(true) YES -> the fix is setReverse + sign flip, all software.\n"
                         " both NO -> rotateFor is unusable; move position control onto the R4.)"));
        Serial.println();
        return;
    }
    if (downEdge) { runNudge(NUDGE_POWER); return; }

    if ((uint32_t)(millis() - _lastStreamMs) >= STREAM_MS)
    {
        _lastStreamMs = millis();
        streamCounts();
    }
}
