/**
 * @file enc_check.ino
 * @brief Encoder-read ground-truth diagnostic: which encoder read tracks a spinning wheel?
 * @author WRO 2026
 *
 * @details
 * `inoTest/wheel_floor` read `M2/M3.getDegrees()` to detect wheel motion, but on the bench the
 * wheel turned while the read said "no". This sketch drives each wheel four ways and prints every
 * candidate encoder read so the working one is visible. Logic lives in `EncCheckHarness.cpp`.
 *
 * `Drive.begin()` here groups `MiniR4.DriveDC` exactly as production (so tests 1/2 use the turn's
 * real actuation, and we learn whether grouping suppresses the per-motor encoders).
 *
 * @note BUILD/FLASH/MONITOR: `uv run inoTest/enc_check/build.py`. Log to a file, not `| tee`.
 * @note NOT in CI — the `inoTest/` diagnostic sketches never are.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "Drive.h"
#include "EncCheckHarness.h"
#include "EventTrace.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Localization.h"
#include "Manipulator.h"
#include "Mission.h"
#include "PathTracker.h"
#include "Tasks.h"
#include "Vision.h"
#include "WROLineArray.h"

/**
 * @brief Cooperative tick — used only for the BTN_UP wait and idle loop (the diagnostic drives run
 *        WITHOUT it, on purpose). Same module set/order as WRO2026.ino::cooperativeTick().
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
    while (!Serial && millis() < 3000) { }

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // Drive.begin() is what configures MiniR4.DriveDC as a grouped pair (Set_Drive2Motor_PARAM),
    // so tests 1/2 drive exactly as the real turn does. The rest match WRO2026.ino::setup().
    if (!LineArray.begin())    { Serial.println(F("[enc_check] LineArray begin() FAILED")); }
    if (!Drive.begin())        { Serial.println(F("[enc_check] Drive begin() FAILED")); }
    if (!LineFollower.begin()) { Serial.println(F("[enc_check] LineFollower begin() FAILED")); }
    if (!Intersection.begin()) { Serial.println(F("[enc_check] Intersection begin() FAILED")); }
    if (!Manipulator.begin())  { Serial.println(F("[enc_check] Manipulator begin() FAILED")); }
    if (!Localization.begin()) { Serial.println(F("[enc_check] Localization begin() FAILED")); }
    if (!PathTracker.begin())  { Serial.println(F("[enc_check] PathTracker begin() FAILED")); }
    if (!Mission.begin())      { Serial.println(F("[enc_check] Mission begin() FAILED")); }
    if (!Tasks.begin())        { Serial.println(F("[enc_check] Tasks begin() FAILED")); }
    if (!Vision.begin())       { Serial.println(F("[enc_check] Vision begin() FAILED")); }
    if (!EventTrace.begin())   { Serial.println(F("[enc_check] EventTrace begin() FAILED")); }

    Drive.setTickCallback(cooperativeTick);

    runEncCheck(cooperativeTick);
}

void loop()
{
    cooperativeTick();
}
