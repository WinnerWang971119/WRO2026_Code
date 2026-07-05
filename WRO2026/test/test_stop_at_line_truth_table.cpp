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
#include "src/nav/StopAtLinePolicy.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <string>
#include <utility>

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
// String -> value maps.
//
// Each column transcriber hand-mirrors the production member names EXACTLY (the
// CSV uses those exact spellings) and delegates to mapToken(), which aborts
// loudly on any token it does not list. This deliberate independent
// transcription is how the generator and both language ports stay decoupled — a
// typo here surfaces as a parity failure, not a silent default.
//
// Strictness vs Python TC13: the enum-mapped columns mirror Python's
// `_MAP[key]` KeyError exactly — an unknown token aborts on both sides. The
// boolean columns (autoRuleActive, expect_seek) and lineBoardCount are
// deliberately STRICTER than Python: toBool accepts only "true"/"false" where
// Python TC13 uses `.strip().lower() == "true"` (any other cell silently
// becomes False), and toLineBoardCount accepts only "1"/"2" where Python uses
// `int()` (any integer parses). The current fixture carries only these exact
// tokens, so both surfaces agree on every row today; the stricter parse is
// intentional — a malformed cell (a capitalized "True", a 3-board count, an
// empty cell) fails this harness loudly instead of being silently coerced into
// a wrong-but-quiet parity pass.
// ===========================================================================

/**
 * @brief Map a CSV token @p s to its value via @p table, aborting on an unknown
 *        token.
 *
 * @details
 * The single loud-on-unknown mapper shared by the column transcribers below.
 * Mirrors Python TC13's dict-literal lookups (`_CALL_FLAG_MAP[...]`, etc.): a
 * token absent from @p table aborts with a named diagnostic rather than silently
 * defaulting, so a CSV / enum drift surfaces as a hard failure, never a
 * wrong-but-quiet pass.
 *
 * @tparam T    Mapped-to type (a Routes / policy enum, `bool`, or `uint8_t`).
 * @param s     The trimmed CSV cell.
 * @param table `{token, value}` pairs — the independent transcription of the
 *              production member names that keeps both language ports decoupled.
 * @param label Human-readable column / type name, used in the abort diagnostic.
 * @return The value paired with @p s.
 */
template <class T>
static T mapToken(const std::string&                               s,
                  std::initializer_list<std::pair<const char*, T>> table,
                  const char*                                      label)
{
    for (const auto& entry : table)
    {
        if (s == entry.first)
        {
            return entry.second;
        }
    }
    std::fprintf(stderr, "FATAL: unmapped %s token '%s'\n", label, s.c_str());
    std::abort();
}

/**
 * @brief Map a `callFlag` cell to `StopAtLine`. Aborts on an unknown token.
 */
static StopAtLine toCallFlag(const std::string& s)
{
    return mapToken<StopAtLine>(
        s,
        {{"AUTO", StopAtLine::AUTO}, {"ON", StopAtLine::ON}, {"OFF", StopAtLine::OFF}},
        "StopAtLine");
}

/**
 * @brief Map an `edgeOverride` cell to `Routes::StopAtLineMode`. Aborts on an
 *        unknown token.
 */
static Routes::StopAtLineMode toEdgeOverride(const std::string& s)
{
    return mapToken<Routes::StopAtLineMode>(
        s,
        {{"DEFAULT", Routes::StopAtLineMode::DEFAULT},
         {"ON", Routes::StopAtLineMode::ON},
         {"OFF", Routes::StopAtLineMode::OFF}},
        "Routes::StopAtLineMode");
}

// ---------------------------------------------------------------------------
// Single bidirectional Routes::BlindMode <-> name table.
//
// toBlindMode() reads it forward (skipping DEFAULT, which never appears in the
// edgeMode / pipelineBlind / expect_motionMode columns — mirrors Python's
// `_BLIND_MODE_MAP`, which lists only the four concrete bundles). blindModeName()
// reads it backward for mismatch diagnostics, with DEFAULT included so an
// unexpected resolver result still prints a real member name, not `<unknown>`.
// One table means a new Routes::BlindMode member is transcribed in exactly one
// place, not kept in sync by hand across a forward and a reverse list.
// ---------------------------------------------------------------------------

struct BlindModeEntry
{
    const char*       name;
    Routes::BlindMode mode;
};

static constexpr BlindModeEntry kBlindModes[] = {
    {"DEFAULT", Routes::BlindMode::DEFAULT},
    {"SYNC_TSHAPE", Routes::BlindMode::SYNC_TSHAPE},
    {"GYRO_TSHAPE", Routes::BlindMode::GYRO_TSHAPE},
    {"RAW_FLAT", Routes::BlindMode::RAW_FLAT},
    {"STOP_AT_LINE", Routes::BlindMode::STOP_AT_LINE},
};

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
    for (const auto& e : kBlindModes)
    {
        if (e.mode != Routes::BlindMode::DEFAULT && s == e.name)
        {
            return e.mode;
        }
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
    return mapToken<Direction>(
        s, {{"FRONT", Direction::FRONT}, {"REAR", Direction::REAR}}, "Direction");
}

/**
 * @brief Parse a boolean cell (`"true"` / `"false"`). Aborts on anything else.
 *
 * @note Deliberately stricter than Python TC13's `.strip().lower() == "true"`
 *       (which silently coerces any other cell to False): a malformed cell
 *       aborts loudly so fixture drift cannot pass quietly. See the section
 *       header above.
 */
static bool toBool(const std::string& s)
{
    return mapToken<bool>(s, {{"true", true}, {"false", false}}, "bool");
}

/**
 * @brief Parse a `lineBoardCount` cell (`"1"` / `"2"`). Aborts on anything else.
 *
 * @note Deliberately stricter than Python TC13's `int()` (which parses any
 *       integer): the fixture only ever carries 1 or 2 (Config::LINE_BOARD_COUNT),
 *       so anything else aborts loudly with a named diagnostic. See the section
 *       header above.
 */
static uint8_t toLineBoardCount(const std::string& s)
{
    return mapToken<uint8_t>(s, {{"1", uint8_t{1}}, {"2", uint8_t{2}}}, "lineBoardCount");
}

/**
 * @brief Reverse map a `Routes::BlindMode` to its member name for readable
 *        `got` / `expected` mismatch reporting.
 */
static const char* blindModeName(Routes::BlindMode m)
{
    for (const auto& e : kBlindModes)
    {
        if (e.mode == m)
        {
            return e.name;
        }
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
    // One list: each required column is resolved, asserted present, and folded
    // into allColumnsPresent in a single pass, so a column added or dropped by
    // issue #108's harnesses cannot leave a stale third site behind.
    TEST_SCOPE("truth-table columns present by name");
    int cCallFlag = -1, cEdgeOverride = -1, cEdgeMode = -1, cPipelineBlind = -1, cDirection = -1,
        cLineBoardCount = -1, cAutoRuleActive = -1, cExpectSeek = -1, cExpectMotion = -1;

    const struct
    {
        const char* name;
        int*        out;
    } required[] = {
        {"callFlag", &cCallFlag},
        {"edgeOverride", &cEdgeOverride},
        {"edgeMode", &cEdgeMode},
        {"pipelineBlind", &cPipelineBlind},
        {"direction", &cDirection},
        {"lineBoardCount", &cLineBoardCount},
        {"autoRuleActive", &cAutoRuleActive},
        {"expect_seek", &cExpectSeek},
        {"expect_motionMode", &cExpectMotion},
    };

    bool allColumnsPresent = true;
    for (const auto& rc : required)
    {
        *rc.out = table.colIndex(rc.name);
        if (*rc.out < 0)
        {
            std::fprintf(stderr, "FAIL: required column '%s' missing from CSV header\n", rc.name);
            allColumnsPresent = false;
        }
        ASSERT_TRUE(*rc.out >= 0);
    }

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
            q.lineBoardCount = toLineBoardCount(wro_csv::field(table, i, cLineBoardCount));
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
    TEST_SCOPE("all rows match the production resolver");
    ASSERT_TRUE(mismatchCount == 0);

    TEST_REPORT();
}
