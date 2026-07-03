/**
 * @file tick_bench.ino
 * @brief T1 RISK-RETIRING SPIKE — proves a standalone sketch compiles+links the REAL WRO2026 modules.
 * @author WRO 2026
 *
 * @details
 * This is the minimal T1 stub for the `cooperativeTick()` benchmark (issue #123). Its ONLY job is
 * to prove the build mechanic: an `inoTest/` sketch can pull in the real `WRO2026/*.cpp` module
 * sources (not host mocks, not a mirror) by treating `WRO2026/` as a legacy Arduino library. T3
 * replaces this stub with the full instrumented bench (TickProfiler segments, Serial report, etc.).
 *
 * It deliberately does the bare minimum: include `<MatrixMiniR4.h>` + three real WRO2026 headers by
 * their normal quote names, then reference two real module globals (`Drive`, `LineArray`) from
 * `setup()` so the linker MUST resolve them against the real `Drive.cpp` / `WROLineArray.cpp`
 * objects. `loop()` is empty. If this links, the whole approach is viable.
 *
 * BUILD MECHANIC — `WRO2026/` as a legacy (1.0) Arduino library
 * -------------------------------------------------------------
 * `WRO2026/` has no `src/` subdir, so arduino-cli treats it as a legacy 1.0 library: it compiles
 * the root `.cpp` files (Drive.cpp, WROLineArray.cpp, ...) and adds ONLY the root to the include
 * path — so `#include "Config.h"` and the transitive `#include "config/ConfigGlobal.h"` resolve,
 * and the `test/` host-mock dir is NOT on the include path (no header shadowing). The `.ino` files
 * in `WRO2026/` are ignored by the library compiler, so there is no `setup()`/`loop()` collision
 * with `WRO2026.ino`.
 *
 * @note BUILD: run from the repo root `C:/users/wangd/documents/wro2026/wro2026_code` —
 *       arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi inoTest/tick_bench --library WRO2026
 *       (verified clean with arduino-cli 1.5.1, arduino:renesas_uno core 1.6.0, MatrixMiniR4 1.2.2.)
 *
 * @note Serial diagnostics run at 115200 baud (matches WRO2026.ino).
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "Drive.h"
#include "WROLineArray.h"

/**
 * @brief Minimal bring-up: init the controller, then touch two real module globals so the linker
 *        pulls in the real `Drive.cpp` and `WROLineArray.cpp` objects.
 *
 * Order mirrors `WRO2026.ino::setup()`: `MiniR4.begin()` first, then Serial, then the battery-cell
 * count from the real `Config::`, then `LineArray.begin()` (needs `Wire1` alive) before
 * `Drive.begin()`.
 */
void setup()
{
    MiniR4.begin();
    Serial.begin(115200);
    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    LineArray.begin();
    Drive.begin();
}

/**
 * @brief Empty — this is a compile spike, not the full bench. T3 fleshes out `loop()`.
 */
void loop()
{
}
