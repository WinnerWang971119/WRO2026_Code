/**
 * @file enc_id.ino
 * @brief Hand-spin encoder identify: which encoder channel belongs to which wheel, motors OFF.
 * @author WRO 2026
 *
 * @details
 * The definitive check for the wheel->encoder mapping: drives nothing, prints both encoder
 * counters live, and you spin each wheel by hand to see which channel moves. Logic in
 * `EncIdHarness.cpp`. Minimal init -- only `MiniR4.begin()` is needed to bring up the encoders;
 * no `Drive`/nav stack, so no motor-drive config can confound the reading.
 *
 * @note BUILD/FLASH/MONITOR: `uv run inoTest/enc_id/build.py`. Log to a file, not `| tee`.
 * @note NOT in CI.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "EncIdHarness.h"
#include "EventTrace.h"

void setup()
{
    MiniR4.begin();    // brings up M1-M4 (motors + encoders); enough to read getCounter()

    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    if (!EventTrace.begin()) { Serial.println(F("[enc_id] EventTrace begin() FAILED")); }

    runEncId(nullptr);    // never returns -- live encoder monitor
}

void loop()
{
    // runEncId() loops forever; nothing here.
}
