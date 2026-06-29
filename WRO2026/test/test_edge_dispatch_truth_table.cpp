/**
 * @file test_edge_dispatch_truth_table.cpp
 * @brief Full-domain CSV parity harness for the production `resolveEdgeDispatch`.
 *
 * Drives every row of the shared truth-table fixture
 * `tools/path_planner/tests/fixtures/edge_dispatch_truth_table.csv` through the
 * header-only production resolver `EdgeDispatchPolicy.h::resolveEdgeDispatch` and
 * asserts the returned `{primitive, reverse, driveMode, commandedDistanceCm,
 * overshootCm, reanchor, brakeOnGate}` against the fixture's seven `expect_*`
 * columns. Any future drift in the C++ resolver therefore turns CI red.
 *
 * This is the C++ mirror of Python's TC1
 * (`tools/path_planner/tests/test_runtime_sim.py::
 * test_tc1_resolve_edge_dispatch_matches_fixture`): same fixture, same column
 * contract, same per-row mapping and comparison. The CSV is the normative single
 * source of truth that both surfaces transcribe independently, and the generator
 * `gen_edge_dispatch_truth_table.py` never calls either resolver — so a drift in
 * the table OR in either resolver is a hard failure, never a tautological pass.
 * Mirrors the #115 `resolveStopAtLine` pattern (`test_stop_at_line_truth_table.cpp`).
 *
 * The resolver is a pure `inline` free function: it COMPOSES the header-only
 * `resolvePivotTrim` / `resolveSensorOvershootTrim` (PivotTrimPolicy.h) and reads
 * only enum TYPES from `routes.h` — no route-table symbol is ODR-used — so this
 * target links no real `PathTracker`/`Drive`/`routes.cpp`, only `MOCK_SOURCES`
 * for the `Serial`/`g_mockMillis` externs the mock `Arduino.h` declares (pulled in
 * transitively via `Config.h`). The fixture path is supplied by CMake as the
 * compile definition `EDGE_DISPATCH_TRUTH_TABLE_CSV` (a string-literal absolute
 * path); the test never hardcodes a path.
 *
 * Per-row comparison is MANUAL (not via the `ASSERT_*` macros): those print
 * their own uncapped `FAIL:` line per failure (and `ASSERT_EQ` casts to
 * `long long`, which cannot show float fields). Mismatches are accumulated and
 * the first `kReportCap` printed in full; a single final `ASSERT_TRUE` records
 * the verdict, so the net assertion count is small and deterministic.
 *
 * Float fields (`commandedDistanceCm`, `overshootCm`) are parsed to numbers and
 * compared within `kDistTol` (the plan's `DIST_TOL`): the resolver computes in
 * `float`, the CSV carries fixed 6-decimal text, so they are NEVER compared as
 * strings.
 */

#include "TestAssert.h"

#include "Arduino.h"    // Mock — supplies Serial/millis that Config.h's <Arduino.h> include resolves.
#include "CsvTruthTable.h"
#include "EdgeDispatchPolicy.h"

#include <cmath>
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
// 864 rows = LINE block 432 (2 destNodeType x 2 direction x 3 rawDistanceCm
//            x 3 lineMode x 2 projPassThrough x 2 departingTrimCm
//            x 3 approachingTrimCm)
//          + BLIND block 432 (2 direction x 2 seek x 3 motionMode
//            x 3 rawDistanceCm x 2 departingTrimCm x 3 approachingTrimCm
//            x 2 axleOffsetCm).
// The generator tools/path_planner/tests/fixtures/gen_edge_dispatch_truth_table.py
// is the single source of truth; this constant is a guard against a truncated or
// empty CSV vacuously passing, NOT a magic number. Mirrors Python TC1's
// _EDGE_DISPATCH_EXPECTED_ROW_COUNT.
// ===========================================================================

static constexpr std::size_t kExpectedRowCount = 864;

// Float comparison tolerance (the plan's DIST_TOL): the resolver computes in
// float32 and the CSV carries fixed 6-decimal text, so the continuous commanded /
// overshoot fields are compared numerically within 1e-4 cm.
static constexpr float kDistTol = 1e-4f;

// ===========================================================================
// String -> value maps.
//
// Each column transcriber hand-mirrors the production member names EXACTLY (the
// CSV uses those exact spellings) and delegates to mapToken(), which aborts
// loudly on any token it does not list. This deliberate independent
// transcription is how the generator and both language ports stay decoupled — a
// typo here surfaces as a parity failure, not a silent default. Mirrors Python
// TC1's dict-literal `_MAP[key]` KeyError-on-unknown behaviour exactly.
// ===========================================================================

/**
 * @brief Map a CSV token @p s to its value via @p table, aborting on an unknown
 *        token.
 *
 * @tparam T    Mapped-to type (a Routes / policy enum, or `bool`).
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
 * @brief Map an `edgeType` cell to `Routes::EdgeType`. Aborts on an unknown token.
 *
 * @note Only LINE / BLIND appear (NONE is never routed), so anything else aborts.
 */
static Routes::EdgeType toEdgeType(const std::string& s)
{
    return mapToken<Routes::EdgeType>(
        s,
        {{"LINE", Routes::EdgeType::LINE}, {"BLIND", Routes::EdgeType::BLIND}},
        "Routes::EdgeType");
}

/**
 * @brief Map a `destNodeType` cell to `Routes::NodeType`. Aborts on an unknown token.
 *
 * @note Only LINE / PROJECTION appear in the fixture: the resolver tests only
 *       `== PROJECTION`, so LINE stands in for any non-projection dest (and is
 *       the pinned sentinel on a BLIND row, which the resolver ignores).
 */
static Routes::NodeType toNodeType(const std::string& s)
{
    return mapToken<Routes::NodeType>(
        s,
        {{"LINE", Routes::NodeType::LINE}, {"PROJECTION", Routes::NodeType::PROJECTION}},
        "Routes::NodeType");
}

/**
 * @brief Map a `direction` cell to `Direction`. Aborts on an unknown token.
 *
 * @note Only FRONT / REAR appear (the resolver receives a post-`resolveDirection`
 *       value, never AUTO), so anything else aborts loudly.
 */
static Direction toDirection(const std::string& s)
{
    return mapToken<Direction>(
        s, {{"FRONT", Direction::FRONT}, {"REAR", Direction::REAR}}, "Direction");
}

/**
 * @brief Map a `lineMode` cell to `Routes::LineMode`. Aborts on an unknown token.
 *
 * @note Only the three concrete presets appear (the resolved line mode is never
 *       DEFAULT); the field is carried by the query but never read by the
 *       decision, so this map exists purely to populate the query faithfully.
 */
static Routes::LineMode toLineMode(const std::string& s)
{
    return mapToken<Routes::LineMode>(
        s,
        {{"NORMAL", Routes::LineMode::NORMAL},
         {"SLOW", Routes::LineMode::SLOW},
         {"FAST", Routes::LineMode::FAST}},
        "Routes::LineMode");
}

// ---------------------------------------------------------------------------
// Single bidirectional Routes::BlindMode <-> name table.
//
// toBlindMode() reads it forward; blindModeName() reads it backward for mismatch
// diagnostics. The `motionMode` input column carries only the three concrete
// bundles; the `expect_driveMode` output column additionally carries DEFAULT
// (the line primitives' unused-mode sentinel). STOP_AT_LINE never appears in
// either column (the resolved motion bundle is never the bare seek tag), but it
// is kept in the table so a stray cell prints a real member name rather than
// `<unknown>`.
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
 * @brief Map a `motionMode` / `expect_driveMode` cell to `Routes::BlindMode`.
 *        Aborts on an unknown token.
 */
static Routes::BlindMode toBlindMode(const std::string& s)
{
    for (const auto& e : kBlindModes)
    {
        if (s == e.name)
        {
            return e.mode;
        }
    }
    std::fprintf(stderr, "FATAL: unmapped Routes::BlindMode token '%s'\n", s.c_str());
    std::abort();
}

/**
 * @brief Map an `expect_primitive` cell to `DispatchPrimitive`. Aborts on an
 *        unknown token.
 */
static DispatchPrimitive toPrimitive(const std::string& s)
{
    return mapToken<DispatchPrimitive>(
        s,
        {{"LINE_FOLLOW_INTERSECTION", DispatchPrimitive::LINE_FOLLOW_INTERSECTION},
         {"PROJECTION_FOLLOW", DispatchPrimitive::PROJECTION_FOLLOW},
         {"STOP_AT_LINE_SEEK", DispatchPrimitive::STOP_AT_LINE_SEEK},
         {"BLIND_DEAD_RECKON", DispatchPrimitive::BLIND_DEAD_RECKON}},
        "DispatchPrimitive");
}

/**
 * @brief Map an `expect_reanchor` cell to `ReanchorKind`. Aborts on an unknown token.
 */
static ReanchorKind toReanchor(const std::string& s)
{
    return mapToken<ReanchorKind>(
        s,
        {{"ON_INTERSECTION", ReanchorKind::ON_INTERSECTION},
         {"ON_DEAD_RECKONED_ADVANCE", ReanchorKind::ON_DEAD_RECKONED_ADVANCE}},
        "ReanchorKind");
}

/**
 * @brief Parse a boolean cell (`"true"` / `"false"`). Aborts on anything else.
 *
 * @note Matches Python TC1's strict `_edge_dispatch_bool`: a malformed cell
 *       aborts loudly so fixture drift cannot pass quietly.
 */
static bool toBool(const std::string& s)
{
    return mapToken<bool>(s, {{"true", true}, {"false", false}}, "bool");
}

/**
 * @brief Parse a float cell. Aborts if the cell is not a valid number.
 *
 * @note Uses `std::strtof` so the parse precision matches the resolver's `float`
 *       arithmetic. A cell that does not parse to a number aborts loudly rather
 *       than silently yielding 0.
 */
static float toFloat(const std::string& s)
{
    const char* begin = s.c_str();
    char*       end   = nullptr;
    const float value = std::strtof(begin, &end);
    if (end == begin)
    {
        std::fprintf(stderr, "FATAL: unparseable float cell '%s'\n", s.c_str());
        std::abort();
    }
    return value;
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

/**
 * @brief Reverse map a `DispatchPrimitive` to its member name for diagnostics.
 */
static const char* primitiveName(DispatchPrimitive p)
{
    switch (p)
    {
        case DispatchPrimitive::LINE_FOLLOW_INTERSECTION: return "LINE_FOLLOW_INTERSECTION";
        case DispatchPrimitive::PROJECTION_FOLLOW:        return "PROJECTION_FOLLOW";
        case DispatchPrimitive::STOP_AT_LINE_SEEK:        return "STOP_AT_LINE_SEEK";
        case DispatchPrimitive::BLIND_DEAD_RECKON:        return "BLIND_DEAD_RECKON";
    }
    return "<unknown>";
}

/**
 * @brief Reverse map a `ReanchorKind` to its member name for diagnostics.
 */
static const char* reanchorName(ReanchorKind r)
{
    switch (r)
    {
        case ReanchorKind::ON_INTERSECTION:          return "ON_INTERSECTION";
        case ReanchorKind::ON_DEAD_RECKONED_ADVANCE: return "ON_DEAD_RECKONED_ADVANCE";
    }
    return "<unknown>";
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: edge-dispatch CSV parity — %zu rows through resolveEdgeDispatch "
                "(mirrors Python TC1)\n",
                kExpectedRowCount);

    // --- Load the fixture via the compile-defined absolute path. -----------
    wro_csv::Table table;
    std::string    err;
    const bool     loaded = wro_csv::load(EDGE_DISPATCH_TRUTH_TABLE_CSV, table, err);

    TEST_SCOPE("CSV loads");
    if (!loaded)
    {
        std::fprintf(stderr,
                     "FAIL: could not load truth-table CSV '%s': %s\n",
                     EDGE_DISPATCH_TRUTH_TABLE_CSV,
                     err.c_str());
    }
    ASSERT_TRUE(loaded);

    // --- Column presence by name (order-independent, loud on a rename). -----
    TEST_SCOPE("truth-table columns present by name");
    int cEdgeType = -1, cDestNodeType = -1, cDirection = -1, cRawDistance = -1, cLineMode = -1,
        cSeek = -1, cMotionMode = -1, cProjPassThrough = -1, cDepartingTrim = -1,
        cApproachingTrim = -1, cAxleOffset = -1, cMinDrive = -1, cExpectPrimitive = -1,
        cExpectReverse = -1, cExpectDriveMode = -1, cExpectCommanded = -1, cExpectOvershoot = -1,
        cExpectReanchor = -1, cExpectBrakeOnGate = -1;

    const struct
    {
        const char* name;
        int*        out;
    } required[] = {
        {"edgeType", &cEdgeType},
        {"destNodeType", &cDestNodeType},
        {"direction", &cDirection},
        {"rawDistanceCm", &cRawDistance},
        {"lineMode", &cLineMode},
        {"seek", &cSeek},
        {"motionMode", &cMotionMode},
        {"projPassThrough", &cProjPassThrough},
        {"departingTrimCm", &cDepartingTrim},
        {"approachingTrimCm", &cApproachingTrim},
        {"axleOffsetCm", &cAxleOffset},
        {"minDriveCm", &cMinDrive},
        {"expect_primitive", &cExpectPrimitive},
        {"expect_reverse", &cExpectReverse},
        {"expect_driveMode", &cExpectDriveMode},
        {"expect_commandedDistanceCm", &cExpectCommanded},
        {"expect_overshootCm", &cExpectOvershoot},
        {"expect_reanchor", &cExpectReanchor},
        {"expect_brakeOnGate", &cExpectBrakeOnGate},
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
            EdgeDispatchQuery q;
            q.edgeType     = toEdgeType(wro_csv::field(table, i, cEdgeType));
            q.destNodeType = toNodeType(wro_csv::field(table, i, cDestNodeType));
            q.direction    = toDirection(wro_csv::field(table, i, cDirection));
            q.rawDistanceCm = toFloat(wro_csv::field(table, i, cRawDistance));
            q.lineMode     = toLineMode(wro_csv::field(table, i, cLineMode));
            q.stopAtLine.seek       = toBool(wro_csv::field(table, i, cSeek));
            q.stopAtLine.motionMode = toBlindMode(wro_csv::field(table, i, cMotionMode));
            q.projPassThrough  = toBool(wro_csv::field(table, i, cProjPassThrough));
            q.departingTrimCm  = toFloat(wro_csv::field(table, i, cDepartingTrim));
            q.approachingTrimCm = toFloat(wro_csv::field(table, i, cApproachingTrim));
            q.axleOffsetCm     = toFloat(wro_csv::field(table, i, cAxleOffset));
            q.minDriveCm       = toFloat(wro_csv::field(table, i, cMinDrive));

            const DispatchPrimitive expectPrimitive =
                toPrimitive(wro_csv::field(table, i, cExpectPrimitive));
            const bool expectReverse = toBool(wro_csv::field(table, i, cExpectReverse));
            const Routes::BlindMode expectDriveMode =
                toBlindMode(wro_csv::field(table, i, cExpectDriveMode));
            const float expectCommanded = toFloat(wro_csv::field(table, i, cExpectCommanded));
            const float expectOvershoot = toFloat(wro_csv::field(table, i, cExpectOvershoot));
            const ReanchorKind expectReanchor =
                toReanchor(wro_csv::field(table, i, cExpectReanchor));
            const bool expectBrake = toBool(wro_csv::field(table, i, cExpectBrakeOnGate));

            const EdgeDispatchDecision d = resolveEdgeDispatch(q);

            const bool match = (d.primitive == expectPrimitive) && (d.reverse == expectReverse) &&
                               (d.driveMode == expectDriveMode) &&
                               (std::fabs(d.commandedDistanceCm - expectCommanded) < kDistTol) &&
                               (std::fabs(d.overshootCm - expectOvershoot) < kDistTol) &&
                               (d.reanchor == expectReanchor) && (d.brakeOnGate == expectBrake);

            if (!match)
            {
                if (mismatchCount < kReportCap)
                {
                    std::fprintf(
                        stderr,
                        "MISMATCH row %zu: edgeType=%s destNodeType=%s direction=%s "
                        "rawDistanceCm=%s seek=%s motionMode=%s projPassThrough=%s "
                        "departingTrimCm=%s approachingTrimCm=%s axleOffsetCm=%s -> "
                        "got(primitive=%s reverse=%d driveMode=%s commandedDistanceCm=%.6f "
                        "overshootCm=%.6f reanchor=%s brakeOnGate=%d) "
                        "expected(primitive=%s reverse=%d driveMode=%s commandedDistanceCm=%.6f "
                        "overshootCm=%.6f reanchor=%s brakeOnGate=%d)\n",
                        i,
                        wro_csv::field(table, i, cEdgeType).c_str(),
                        wro_csv::field(table, i, cDestNodeType).c_str(),
                        wro_csv::field(table, i, cDirection).c_str(),
                        wro_csv::field(table, i, cRawDistance).c_str(),
                        wro_csv::field(table, i, cSeek).c_str(),
                        wro_csv::field(table, i, cMotionMode).c_str(),
                        wro_csv::field(table, i, cProjPassThrough).c_str(),
                        wro_csv::field(table, i, cDepartingTrim).c_str(),
                        wro_csv::field(table, i, cApproachingTrim).c_str(),
                        wro_csv::field(table, i, cAxleOffset).c_str(),
                        primitiveName(d.primitive),
                        static_cast<int>(d.reverse),
                        blindModeName(d.driveMode),
                        static_cast<double>(d.commandedDistanceCm),
                        static_cast<double>(d.overshootCm),
                        reanchorName(d.reanchor),
                        static_cast<int>(d.brakeOnGate),
                        primitiveName(expectPrimitive),
                        static_cast<int>(expectReverse),
                        blindModeName(expectDriveMode),
                        static_cast<double>(expectCommanded),
                        static_cast<double>(expectOvershoot),
                        reanchorName(expectReanchor),
                        static_cast<int>(expectBrake));
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
                     "RESULT: %zu of %zu rows mismatched the production resolveEdgeDispatch\n",
                     mismatchCount,
                     table.rows.size());
    }

    // --- Single verdict assert (NOT a per-row ASSERT, to honor the cap). ----
    TEST_SCOPE("all rows match the production resolver");
    ASSERT_TRUE(mismatchCount == 0);

    TEST_REPORT();
}
