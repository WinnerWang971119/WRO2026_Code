/**
 * @file wheel_floor.ino
 * @brief Standalone probe for the PER-WHEEL breakaway/creep PWM floors, using each wheel's own
 *        encoder — the measurement behind the asymmetric-chassis turn fix (Option A).
 * @author WRO 2026
 *
 * @details
 * The chassis's heavy left side means a symmetric turn floor moves only the light right wheel; the
 * heavy left wheel stalls and the robot pivots instead of rotating about center. This probe drives
 * one wheel at a time through the turn's actuation (`MiniR4.DriveDC.Move`) and reads that wheel's
 * encoder (`MiniR4.M2/M3.getDegrees()`) to measure each wheel's OWN breakaway + creep floor,
 * automatically. The measurement + confirmation logic lives in `WheelFloorHarness.cpp` — see that
 * file's header for the full contract.
 *
 * `Drive.begin()` here configures `MiniR4.DriveDC` with `Config::MOTOR_LEFT_ID/RIGHT_ID` and the
 * reverse flags, so `DriveDC.Move` maps the wheels EXACTLY as the real turn does — that is why the
 * full module init below is kept (the same as `inoTest/turn_probe`), not trimmed.
 *
 * @note BUILD/FLASH/MONITOR: `uv run inoTest/wheel_floor/build.py` (add `--build-only` to compile
 *       without touching a connected board). Log STRAIGHT to a file, not `| tee`. BTN_UP starts,
 *       BTN_DOWN aborts.
 * @note Serial diagnostics run at 115200 baud; `setup()` waits up to 3 s for a USB CDC listener but
 *       does not hang a headless boot.
 * @note NOT in CI — the `inoTest/` diagnostic sketches never are.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "Drive.h"
#include "EventTrace.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Localization.h"
#include "Manipulator.h"
#include "Mission.h"
#include "PathTracker.h"
#include "Tasks.h"
#include "Vision.h"
#include "WheelFloorHarness.h"
#include "WROLineArray.h"

/**
 * @brief Cooperative tick — every module's `update()` once, same order as
 *        `WRO2026.ino::cooperativeTick()`. Keeps the OLED/Serial and idle modules serviced during
 *        the probe's blocking drive windows. (The probe does not need a control-representative
 *        tick, but mirroring the set keeps every module initialised and alive.)
 */
void cooperativeTick()
{
    LineArray.update();
    Drive.update();
    LineFollower.update();
    Intersection.update();
    Manipulator.update();
    Tasks.update();
    PathTracker.update();
    Localization.update();
    Mission.update();
    EventTrace.update();
}

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }    // give USB CDC a moment, but don't hang headless

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // All 11 begin()s, same order as WRO2026.ino::setup(). Drive.begin() is REQUIRED here — it
    // configures MiniR4.DriveDC with the project's left/right motor IDs and reverse flags, so the
    // probe's Move() drives the wheels identically to the real turn.
    if (!LineArray.begin())    { Serial.println(F("[wheel_floor] LineArray begin() FAILED")); }
    if (!Drive.begin())        { Serial.println(F("[wheel_floor] Drive begin() FAILED")); }
    if (!LineFollower.begin()) { Serial.println(F("[wheel_floor] LineFollower begin() FAILED")); }
    if (!Intersection.begin()) { Serial.println(F("[wheel_floor] Intersection begin() FAILED")); }
    if (!Manipulator.begin())  { Serial.println(F("[wheel_floor] Manipulator begin() FAILED")); }
    if (!Localization.begin()) { Serial.println(F("[wheel_floor] Localization begin() FAILED")); }
    if (!PathTracker.begin())  { Serial.println(F("[wheel_floor] PathTracker begin() FAILED")); }
    if (!Mission.begin())      { Serial.println(F("[wheel_floor] Mission begin() FAILED")); }
    if (!Tasks.begin())        { Serial.println(F("[wheel_floor] Tasks begin() FAILED")); }
    if (!Vision.begin())       { Serial.println(F("[wheel_floor] Vision begin() FAILED")); }
    if (!EventTrace.begin())   { Serial.println(F("[wheel_floor] EventTrace begin() FAILED")); }

    Drive.setTickCallback(cooperativeTick);
    PathTracker.setTickCallback(cooperativeTick);
    LineFollower.setTickCallback(cooperativeTick);
    Manipulator.setTickCallback(cooperativeTick);
    Tasks.setTickCallback(cooperativeTick);
    Vision.setTickCallback(cooperativeTick);

    // Gated on BTN_UP inside runWheelFloor(); nothing drives until the operator presses it. Blocks
    // for the whole probe, pumping cooperativeTick() throughout via the tickCb parameter.
    runWheelFloor(cooperativeTick);
}

void loop()
{
    // One-shot per power cycle ("press RESET to run again"). Keep ticking so Serial/OLED stay alive.
    cooperativeTick();
}
