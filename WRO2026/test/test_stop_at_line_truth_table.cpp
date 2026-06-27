/**
 * @file test_stop_at_line_truth_table.cpp
 * @brief Full-domain CSV parity harness for the production `resolveStopAtLine`.
 *
 * Drives every row of the shared truth-table fixture
 * `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv` through the
 * header-only production resolver `StopAtLinePolicy.h::resolveStopAtLine` and
 * asserts the returned `(seek, motionMode)` against the fixture's
 * `expect_seek` / `expect_motionMode` columns. Any future drift in the C++
 * resolver therefore turns CI red.
 *
 * This is the C++ mirror of Python's TC13
 * (`tools/path_planner/tests/test_runtime_sim.py::
 * test_tc13_resolve_stop_at_line_matches_fixture`): same fixture, same column
 * contract, same per-row mapping and comparison. The CSV is the normative
 * single source of truth that both surfaces transcribe independently.
 *
 * The resolver is a pure `inline` free function that reads nothing from the
 * route tables, so this target links no real `PathTracker`/`Drive`/`routes.cpp`
 * — only the header-only policy against the host mocks. The fixture path is
 * supplied by CMake as the compile definition `STOP_AT_LINE_TRUTH_TABLE_CSV`
 * (a string-literal absolute path); the test never hardcodes a path.
 *
 * Per-row comparison is MANUAL (not via the `ASSERT_*` macros): those print
 * their own uncapped `FAIL:` line per failure, which would defeat the ~20-row
 * report cap. Mismatches are accumulated and the first `kReportCap` printed in
 * full; a single final `ASSERT_TRUE(mismatchCount == 0)` records the verdict,
 * so the net assertion count is small and deterministic (1 load + 9 columns +
 * 1 row-count + 1 verdict), not 936 x 2.
 */

#include "TestAssert.h"

#include "Arduino.h"    // Mock — supplies Serial/millis that Config.h's <Arduino.h> include resolves.
#include "CsvTruthTable.h"
#include "StopAtLinePolicy.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

// ===========================================================================
// Row-count guard.
//
// 936 rows = base 864 (3 callFlag x 3 edgeOverride x 4 edgeMode x 3 pipelineBlind
//            x 2 direction x 2 lineBoardCount x 2 autoRuleActive)
//            + supplemental 72 (pipelineBlind = STOP_AT_LINE, edgeMode pinned).
// The generator tools/path_planner/tests/fixtures/gen_stop_at_line_truth_table.py
// is the single source of truth; this constant is a guard against a truncated or
// empty CSV vacuously passing, NOT a magic number. Mirrors Python TC13's
// _EXPECTED_ROW_COUNT.
// ===========================================================================

static constexpr std::size_t kExpectedRowCount = 936;

// ===========================================================================
// String -> enum maps.
//
// Each function hand-mirrors the production enum member names EXACTLY (the CSV
// uses those exact spellings). This deliberate independent transcription is how
// the generator and both language ports stay decoupled — a typo here surfaces
// as a parity failure, not a silent default. An unmapped token aborts loudly
// (mirrors Python's `_MAP[key]` KeyError); it is NEVER silently defaulted.
// ===========================================================================

/**
 * @brief Map a `callFlag` cell to `StopAtLine`. Aborts on an unknown token.
 */
static StopAtLine toCallFlag(const std::string& s)
{
    if (s == "AUTO")
    {
        return StopAtLine::AUTO;
    }
    if (s == "ON")
    {
        return StopAtLine::ON;
    }
    if (s == "OFF")
    {
        return StopAtLine::OFF;
    }
    std::fprintf(stderr, "FATAL: unmapped StopAtLine token '%s'\n", s.c_str());
    std::abort();
}

/**
 * @brief Map an `edgeOverride` cell to `Routes::StopAtLineMode`. Aborts on an
 *        unknown token.
 */
static Routes::StopAtLineMode toEdgeOverride(const std::string& s)
{
    if (s == "DEFAULT")
    {
        return Routes::StopAtLineMode::DEFAULT;
    }
    if (s == "ON")
    {
        return Routes::StopAtLineMode::ON;
    }
    if (s == "OFF")
    {
        return Routes::StopAtLineMode::OFF;
    }
    std::fprintf(stderr, "FATAL: unmapped Routes::StopAtLineMode token '%s'\n", s.c_str());
    std::abort();
}

/**
 * @brief Map an `edgeMode` / `pipelineBlind` / `expect_motionMode` cell to
 *        `Routes::BlindMode`. Aborts on an unknown token.
 *
 * @note Mirrors Python's `_BLIND_MODE_MAP`: only the four concrete bundles are
 *       accepted. `DEFAULT` never appears in these CSV columns, so it is
 *       intentionally rejected (loud) rather than mapped.
 */
static Routes::BlindMode toBlindMode(const std::string& s)
{
    if (s == "SYNC_TSHAPE")
    {
        return Routes::BlindMode::SYNC_TSHAPE;
    }
    if (s == "GYRO_TSHAPE")
    {
        return Routes::BlindMode::GYRO_TSHAPE;
    }
    if (s == "RAW_FLAT")
    {
        return Routes::BlindMode::RAW_FLAT;
    }
    if (s == "STOP_AT_LINE")
    {
        return Routes::BlindMode::STOP_AT_LINE;
    }
    std::fprintf(stderr, "FATAL: unmapped Routes::BlindMode token '%s'\n", s.c_str());
    std::abort();
}

/**
 * @brief Map a `direction` cell to `Direction`. Aborts on an unknown token.
 *
 * @note Mirrors Python's `_DIRECTION_MAP`: only FRONT / REAR appear in the
 *       fixture (the resolver receives a post-`resolveDirection` value, never
 *       `AUTO`), so anything else aborts loudly.
 */
static Direction toDirection(const std::string& s)
{
    if (s == "FRONT")
    {
        return Direction::FRONT;
    }
    if (s == "REAR")
    {
        return Direction::REAR;
    }
    std::fprintf(stderr, "FATAL: unmapped Direction token '%s'\n", s.c_str());
    std::abort();
}

/**
 * @brief Parse a boolean cell (`"true"` / `"false"`). Aborts on anything else.
 */
static bool toBool(const std::string& s)
{
    if (s == "true")
    {
        return true;
    }
    if (s == "false")
    {
        return false;
    }
    std::fprintf(stderr, "FATAL: unmapped bool token '%s' (expected true/false)\n", s.c_str());
    std::abort();
}

/**
 * @brief Reverse map a `Routes::BlindMode` to its member name for readable
 *        `got` / `expected` mismatch reporting.
 */
static const char* blindModeName(Routes::BlindMode m)
{
    switch (m)
    {
        case Routes::BlindMode::DEFAULT:
            return "DEFAULT";
        case Routes::BlindMode::SYNC_TSHAPE:
            return "SYNC_TSHAPE";
        case Routes::BlindMode::GYRO_TSHAPE:
            return "GYRO_TSHAPE";
        case Routes::BlindMode::RAW_FLAT:
            return "RAW_FLAT";
        case Routes::BlindMode::STOP_AT_LINE:
            return "STOP_AT_LINE";
    }
    return "<unknown>";
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: stop-at-line CSV parity — %zu rows through resolveStopAtLine "
                "(mirrors Python TC13)\n",
                kExpectedRowCount);

    // --- Load the fixture via the compile-defined absolute path. -----------
    wro_csv::Table table;
    std::string    err;
    const bool     loaded = wro_csv::load(STOP_AT_LINE_TRUTH_TABLE_CSV, table, err);

    TEST_SCOPE("CSV loads");
    if (!loaded)
    {
        std::fprintf(stderr,
                     "FAIL: could not load truth-table CSV '%s': %s\n",
                     STOP_AT_LINE_TRUTH_TABLE_CSV,
                     err.c_str());
    }
    ASSERT_TRUE(loaded);

    // --- Column presence by name (order-independent, loud on a rename). -----
    TEST_SCOPE("truth-table columns present by name");
    const int cCallFlag       = table.colIndex("callFlag");
    const int cEdgeOverride   = table.colIndex("edgeOverride");
    const int cEdgeMode       = table.colIndex("edgeMode");
    const int cPipelineBlind  = table.colIndex("pipelineBlind");
    const int cDirection      = table.colIndex("direction");
    const int cLineBoardCount = table.colIndex("lineBoardCount");
    const int cAutoRuleActive = table.colIndex("autoRuleActive");
    const int cExpectSeek     = table.colIndex("expect_seek");
    const int cExpectMotion   = table.colIndex("expect_motionMode");
    ASSERT_TRUE(cCallFlag >= 0);
    ASSERT_TRUE(cEdgeOverride >= 0);
    ASSERT_TRUE(cEdgeMode >= 0);
    ASSERT_TRUE(cPipelineBlind >= 0);
    ASSERT_TRUE(cDirection >= 0);
    ASSERT_TRUE(cLineBoardCount >= 0);
    ASSERT_TRUE(cAutoRuleActive >= 0);
    ASSERT_TRUE(cExpectSeek >= 0);
    ASSERT_TRUE(cExpectMotion >= 0);

    const bool allColumnsPresent = cCallFlag >= 0 && cEdgeOverride >= 0 && cEdgeMode >= 0 &&
                                   cPipelineBlind >= 0 && cDirection >= 0 && cLineBoardCount >= 0 &&
                                   cAutoRuleActive >= 0 && cExpectSeek >= 0 && cExpectMotion >= 0;

    // --- Row-count guard: a truncated / empty CSV must fail loudly. ---------
    TEST_SCOPE("row count guards against a truncated CSV");
    ASSERT_EQ(table.rows.size(), kExpectedRowCount);

    // --- Per-row parity loop (manual compare, report-all-capped). -----------
    std::size_t       mismatchCount = 0;
    const std::size_t kReportCap    = 20;

    // Guard the loop on a clean load + full column set so a missing column
    // reports via the assert above instead of aborting inside `wro_csv::field`.
    // A wrong row count is intentionally NOT a guard: the loop still runs over
    // whatever rows exist (the ASSERT_EQ already recorded the failure).
    if (loaded && allColumnsPresent)
    {
        for (std::size_t i = 0; i < table.rows.size(); ++i)
        {
            StopAtLineQuery q;
            q.callFlag      = toCallFlag(wro_csv::field(table, i, cCallFlag));
            q.edgeOverride  = toEdgeOverride(wro_csv::field(table, i, cEdgeOverride));
            q.edgeMode      = toBlindMode(wro_csv::field(table, i, cEdgeMode));
            q.pipelineBlind = toBlindMode(wro_csv::field(table, i, cPipelineBlind));
            q.direction     = toDirection(wro_csv::field(table, i, cDirection));
            q.lineBoardCount =
                static_cast<uint8_t>(std::stoi(wro_csv::field(table, i, cLineBoardCount)));
            q.autoRuleActive = toBool(wro_csv::field(table, i, cAutoRuleActive));

            const bool              expectSeek   = toBool(wro_csv::field(table, i, cExpectSeek));
            const Routes::BlindMode expectMotion = toBlindMode(wro_csv::field(table, i, cExpectMotion));

            const StopAtLineResolution r = resolveStopAtLine(q);

            if (r.seek != expectSeek || r.motionMode != expectMotion)
            {
                if (mismatchCount < kReportCap)
                {
                    std::fprintf(
                        stderr,
                        "MISMATCH row %zu: callFlag=%s edgeOverride=%s edgeMode=%s "
                        "pipelineBlind=%s direction=%s lineBoardCount=%s autoRuleActive=%s -> "
                        "got(seek=%d, motionMode=%s) expected(seek=%d, motionMode=%s)\n",
                        i,
                        wro_csv::field(table, i, cCallFlag).c_str(),
                        wro_csv::field(table, i, cEdgeOverride).c_str(),
                        wro_csv::field(table, i, cEdgeMode).c_str(),
                        wro_csv::field(table, i, cPipelineBlind).c_str(),
                        wro_csv::field(table, i, cDirection).c_str(),
                        wro_csv::field(table, i, cLineBoardCount).c_str(),
                        wro_csv::field(table, i, cAutoRuleActive).c_str(),
                        static_cast<int>(r.seek),
                        blindModeName(r.motionMode),
                        static_cast<int>(expectSeek),
                        blindModeName(expectMotion));
                }
                ++mismatchCount;
            }
        }
    }

    if (mismatchCount > kReportCap)
    {
        std::fprintf(stderr,
                     "... %zu additional mismatching rows suppressed (report cap %zu)\n",
                     mismatchCount - kReportCap,
                     kReportCap);
    }
    if (mismatchCount != 0)
    {
        std::fprintf(stderr,
                     "RESULT: %zu of %zu rows mismatched the production resolveStopAtLine\n",
                     mismatchCount,
                     table.rows.size());
    }

    // --- Single verdict assert (NOT a per-row ASSERT, to honor the cap). ----
    TEST_SCOPE("all 936 rows match the production resolver");
    ASSERT_TRUE(mismatchCount == 0);

    TEST_REPORT();
}
