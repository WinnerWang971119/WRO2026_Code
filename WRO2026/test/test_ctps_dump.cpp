/**
 * @file test_ctps_dump.cpp
 * @brief Host-compile test for the CTPS Vision mock + dump plumbing (task T22) — TC14/TC15.
 * @author WRO 2026
 *
 * @details
 * Compiled by the `test_ctps_dump` target (registered by T7 in `CMakeLists.txt`, guarded
 * on `EXISTS ${TEST_DIR}/test_ctps_dump.cpp`). That target force-includes the recording
 * `ctps_mocks/` surface (`Drive.h` / `PathTracker.h` / `EventTrace.h` / `LineFollower.h` /
 * `Localization.h` / `Manipulator.h`) PLUS the `ctps_dump`-only `Vision.h` shadow, links
 * `ctps_record.cpp` (the recorder + the `VisionClass Vision` global) and the issue #91
 * `fixtures/routes_fixture.cpp`, and defines `CTPS_RECORD`. This TU provides its OWN
 * `main()` — it does NOT link `ctps_dump.cpp`.
 *
 * Two case families:
 *   - **TC14a** — `Vision.getColor(i)` returns the injected per-cell colour; an
 *     out-of-range index returns `Config::Color::UNKNOWN` (the mock's bounds-checking
 *     contract, per `ctps_mocks/Vision.h`'s `getColor` docstring).
 *   - **TC14b** — `Vision.readMosaic()` + `_setFailAt(n)` (1-based fail index, confirmed
 *     against the mock's docstring: "the Nth call... 1-based") and the `read_mosaic` op
 *     recorded into `ctps::g_ctpsLog` on every call (success flag only — the mock never
 *     carries per-op colour data, see `Op::READ_MOSAIC`'s docstring in `ctps_record.h`).
 *
 * **TC15 (build proof).** The mere fact that this translation unit compiles and links —
 * with the `Vision.h` mock shadow force-included ahead of the real `Vision.h`, and the
 * full `ctps_record.cpp` recorder linked in — proves the mock shadow + recorder combo is
 * wired correctly for the `ctps_dump` target (which compiles the whole of `Tasks.cpp` and
 * would break if a future vision-consuming task could not resolve `Vision` against this
 * mock). No runtime assertion is needed for TC15; the ctest target itself (`ctps_dump_mock`)
 * existing and passing IS the proof.
 */

#include "TestAssert.h"

// Because the `test_ctps_dump` CMake target force-includes `ctps_mocks/Vision.h` ahead of
// any source parsing, `Vision`, `VisionClass`, and `Config::Color` are already visible here
// with no explicit `#include "Vision.h"` needed — mirroring how `test_ctps_mock_conformance`
// relies on its own force-included mock surface. `ctps_record.h` gives us `ctps::reset()`
// and `ctps::g_ctpsLog` directly (not transitively guaranteed), so it is included explicitly.
#include "ctps_record.h"

#include "config/ConfigVision.h"

#include <cstdint>

// ===========================================================================
// TC14a — getColor returns injected colors; out-of-range -> UNKNOWN.
// ===========================================================================

static void test_TC14a_getColor_returns_injected()
{
    TEST_SCOPE("TC14a: getColor returns injected colors; out-of-range -> UNKNOWN");

    ctps::reset();

    // A known 12-color array mixing YELLOW/BLUE/GREEN/WHITE across all cell indices.
    Config::Color colors[Config::VISION_CELL_COUNT] = {
        Config::Color::YELLOW, Config::Color::BLUE,   Config::Color::GREEN, Config::Color::WHITE,
        Config::Color::GREEN,  Config::Color::YELLOW, Config::Color::WHITE, Config::Color::BLUE,
        Config::Color::BLUE,   Config::Color::WHITE,  Config::Color::YELLOW, Config::Color::GREEN,
    };

    Vision._setInjectedColors(colors);

    for (uint8_t i = 0; i < Config::VISION_CELL_COUNT; ++i)
    {
        ASSERT_TRUE(Vision.getColor(i) == colors[i]);
    }

    // Out-of-range indices return the safe sentinel UNKNOWN — never a crash / garbage read.
    ASSERT_TRUE(Vision.getColor(Config::VISION_CELL_COUNT) == Config::Color::UNKNOWN);
    ASSERT_TRUE(Vision.getColor(200) == Config::Color::UNKNOWN);
}

// ===========================================================================
// TC14b — readMosaic + fail-at (1-based) + read_mosaic op recorded.
// ===========================================================================

static void test_TC14b_readMosaic_failAt_and_op_recorded()
{
    TEST_SCOPE("TC14b: readMosaic fail-at (1-based) + read_mosaic op recorded");

    // --- Sub-case 1: _setFailAt(1) -> the 1st readMosaic() is false, the 2nd is true. ---
    ctps::reset();
    Vision._setFailAt(1);

    const bool firstOk  = Vision.readMosaic();
    const bool secondOk = Vision.readMosaic();

    ASSERT_FALSE(firstOk);
    ASSERT_TRUE(secondOk);

    // Exactly two READ_MOSAIC records, first success==false, second success==true.
    {
        int readMosaicCount = 0;
        bool firstSuccess = true;    // sentinel — overwritten by the first match found
        bool secondSuccess = false;  // sentinel — overwritten by the second match found
        for (const auto& rec : ctps::g_ctpsLog)
        {
            if (rec.op == ctps::Op::READ_MOSAIC)
            {
                if (readMosaicCount == 0)
                {
                    firstSuccess = rec.success;
                }
                else if (readMosaicCount == 1)
                {
                    secondSuccess = rec.success;
                }
                ++readMosaicCount;
            }
        }
        ASSERT_EQ(readMosaicCount, 2);
        ASSERT_FALSE(firstSuccess);
        ASSERT_TRUE(secondSuccess);
    }

    // --- Sub-case 2: _setFailAt(0) (never) -> a fresh reset(), one readMosaic() -> true,
    //     one read_mosaic op recorded with success == true. ---
    ctps::reset();
    Vision._setFailAt(0);

    const bool ok = Vision.readMosaic();
    ASSERT_TRUE(ok);

    {
        int readMosaicCount = 0;
        bool success = false;
        for (const auto& rec : ctps::g_ctpsLog)
        {
            if (rec.op == ctps::Op::READ_MOSAIC)
            {
                success = rec.success;
                ++readMosaicCount;
            }
        }
        ASSERT_EQ(readMosaicCount, 1);
        ASSERT_TRUE(success);
    }
}

// ===========================================================================
// TC15 — build proof: this TU compiling + linking against the Vision.h mock shadow
// and the full ctps_record.cpp recorder IS the assertion. No runtime code needed —
// see the file docblock.
// ===========================================================================

// ===========================================================================
// Test entry point.
// ===========================================================================

int main()
{
    test_TC14a_getColor_returns_injected();
    test_TC14b_readMosaic_failAt_and_op_recorded();

    TEST_REPORT();
}
