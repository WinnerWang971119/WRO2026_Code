// =========================================================================================
// SCOPED test shadow of WRO2026/ConfigLocalization.h — strict wall-normal check FORCED ON.
//
// WHY THIS EXISTS
//   `Config::WALL_NORMAL_STRICT_ENABLED` is a hard `constexpr bool ... = false;` in the
//   production `ConfigLocalization.h` — it has NO `#ifndef <NAME>_DEFAULT` override hook (unlike
//   `PATH_PASS_THROUGH_ENABLED` / `BIDIRECTIONAL_AUTO_ENABLED` / `MISSION_ALLOW_REARM`). So it
//   CANNOT be flipped to `true` via `target_compile_definitions(... <NAME>_DEFAULT=true)`, and an
//   in-file `#define` in the test .cpp would NOT affect the separately-compiled `Localization.cpp`
//   TU (the known cross-TU Config-override trap). Editing the production header is out of scope for
//   T18 (test-only). The remaining clean, target-scoped lever is to SHADOW this sub-header for ONE
//   target via a force-include, so BOTH `Localization.cpp` and the test main in that target see the
//   strict flag ON.
//
// HOW IT IS WIRED (see CMakeLists.txt `test_reanchor_at_wall_strict`)
//   This file lives in its OWN dir (`wall_strict_mocks/`, NOT `mocks/`) so it shadows
//   `ConfigLocalization.h` for ONLY the strict-ON target — `mocks/` is on every target's include
//   path and a copy there would silently flip the strict flag for ALL targets (the
//   `mocks/`-shadows-everything trap). It is force-included FIRST into the strict-ON target with
//   the production include guard `CONFIG_LOCALIZATION_H` defined, so when `Config.h` later does
//   `#include "ConfigLocalization.h"` the real header no-ops.
//
// MINIMAL-BY-DESIGN (low drift)
//   `Localization.cpp` odr-uses EXACTLY four `Config::` symbols from this sub-header
//   (`WALL_NORMAL_STRICT_ENABLED`, `WALL_NORMAL_TOLERANCE_DEG`, `OFFSET_TRANSLATION_DEADBAND_CM`,
//   `OFFSET_ROTATION_DEADBAND_DEG` — verified by grep), and `ConfigChecks.h` references NONE of
//   this header's symbols. So this shadow provides ONLY those four constants. If a future edit
//   makes `Localization.cpp` read a NEW symbol from this sub-header, the strict-ON target fails to
//   compile with a missing-symbol error — a loud tripwire to extend this shadow, not a silent skew.
//   The four values mirror the production defaults verbatim EXCEPT the one under test
//   (`WALL_NORMAL_STRICT_ENABLED` = true here vs false in production).
// =========================================================================================

#ifndef CONFIG_LOCALIZATION_H
#define CONFIG_LOCALIZATION_H

#include <stdint.h>

namespace Config
{
    // The ONE difference from production: strict wall-normal sanity check FORCED ON so the
    // reanchorAtWall boundary (`diff > WALL_NORMAL_TOLERANCE_DEG` rejects) is exercised.
    constexpr bool WALL_NORMAL_STRICT_ENABLED = true;

    // Mirror of the production default (ConfigLocalization.h) — the boundary tolerance under test.
    constexpr float WALL_NORMAL_TOLERANCE_DEG = 10.0f;

    // Mirrors of the production deadbands — read by Localization::_integrateOffset(). Values match
    // the production defaults so the offset integrator behaves identically; only the strict flag
    // differs in this target.
    constexpr float OFFSET_TRANSLATION_DEADBAND_CM = 0.2f;
    constexpr float OFFSET_ROTATION_DEADBAND_DEG   = 0.5f;
}    // namespace Config

#endif    // CONFIG_LOCALIZATION_H
