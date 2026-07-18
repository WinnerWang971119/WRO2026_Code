/**
 * @file wheel_match.ino
 * @brief Lifted wheel-match test: is the turn imbalance the weight, or mechanical drag on the left?
 * @author WRO 2026
 *
 * @details
 * Drives both wheels at matched PWM with the robot LIFTED (no ground load) and compares each
 * wheel's encoder rate. Matched -> the on-ground turn imbalance is the weight; left slower even
 * unloaded -> mechanical drag on the left drivetrain. Logic lives in `WheelMatchHarness.cpp`.
 *
 * `Drive.begin()` here configures MiniR4.DriveDC (left=M2, right=M3, reverse flags) so `Move`
 * drives the wheels exactly as the turn does; encoders are read on the swapped channels
 * (left->M3, right->M2, see enc_check).
 *
 * @note BUILD/FLASH/MONITOR: `uv run inoTest/wheel_match/build.py`. Log to a file, not `| tee`.
 *       LIFT THE ROBOT before pressing BTN_UP.
 * @note NOT in CI.
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
#include "WheelMatchHarness.h"
#include "WROLineArray.h"

/** @brief Cooperative tick (BTN_UP wait + idle). Same module set/order as WRO2026.ino. */
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
    while (!Serial && millis() < 3000) { }

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // Drive.begin() configures MiniR4.DriveDC with the project's motor IDs + reverse flags so Move
    // maps the wheels as the turn does. The rest match WRO2026.ino::setup().
    if (!LineArray.begin())    { Serial.println(F("[wheel_match] LineArray begin() FAILED")); }
    if (!Drive.begin())        { Serial.println(F("[wheel_match] Drive begin() FAILED")); }
    if (!LineFollower.begin()) { Serial.println(F("[wheel_match] LineFollower begin() FAILED")); }
    if (!Intersection.begin()) { Serial.println(F("[wheel_match] Intersection begin() FAILED")); }
    if (!Manipulator.begin())  { Serial.println(F("[wheel_match] Manipulator begin() FAILED")); }
    if (!Localization.begin()) { Serial.println(F("[wheel_match] Localization begin() FAILED")); }
    if (!PathTracker.begin())  { Serial.println(F("[wheel_match] PathTracker begin() FAILED")); }
    if (!Mission.begin())      { Serial.println(F("[wheel_match] Mission begin() FAILED")); }
    if (!Tasks.begin())        { Serial.println(F("[wheel_match] Tasks begin() FAILED")); }
    if (!Vision.begin())       { Serial.println(F("[wheel_match] Vision begin() FAILED")); }
    if (!EventTrace.begin())   { Serial.println(F("[wheel_match] EventTrace begin() FAILED")); }

    Drive.setTickCallback(cooperativeTick);

    runWheelMatch(cooperativeTick);
}

void loop()
{
    cooperativeTick();
}
