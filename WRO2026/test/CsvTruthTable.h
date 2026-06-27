/**
 * @file CsvTruthTable.h
 * @brief Reusable CSV table loader for host-compile parity harnesses.
 *
 * @details
 * Namespace `wro_csv` provides a minimal, CSV-mechanics-only table loader
 * intended for the host `test/` suite.  It carries zero knowledge of
 * production enums or resolver logic — the test that `#include`s it owns all
 * enum mapping and assertion logic.
 *
 * Designed to be lifted by issue #108's per-resolver harnesses without
 * modification.  This header never runs on the robot: it is `test/` code
 * only and may use the C++ STL freely.
 *
 * @note Include this header from `test/` targets only.  Do NOT place it in
 *       any `mocks/` directory — the `mocks/` directories are on every
 *       host-test target's include path and a header placed there shadows
 *       real production headers for all targets.
 */

#ifndef CSV_TRUTH_TABLE_H
#define CSV_TRUTH_TABLE_H

#include <cassert>
#include <fstream>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// wro_csv — CSV mechanics only.  No production enum or resolver knowledge.
// ---------------------------------------------------------------------------

/**
 * @brief CSV mechanics namespace: column-name lookup, whitespace / CR trim,
 *        row iteration, and row count.
 *
 * @details
 * Typical usage:
 * @code
 *   wro_csv::Table t;
 *   std::string err;
 *   if (!wro_csv::load(MY_CSV_PATH, t, err)) { ... fail ... }
 *   int colSeek = t.colIndex("expect_seek");
 *   ASSERT_TRUE(colSeek >= 0);
 *   for (std::size_t i = 0; i < t.rows.size(); ++i) {
 *       const std::string& val = wro_csv::field(t, i, colSeek);
 *       ...
 *   }
 * @endcode
 */
namespace wro_csv
{

// ---------------------------------------------------------------------------
// Internal helpers — not part of the public API.
// ---------------------------------------------------------------------------

namespace detail
{

/**
 * @brief Trim leading and trailing ASCII whitespace (space, tab, CR, LF)
 *        from @p s.
 *
 * @param s Input string (may carry a stray `\r` from a CRLF checkout).
 * @return Copy of @p s with leading/trailing whitespace stripped.
 *
 * @note `\r` is treated as whitespace so that the last field on each line
 *       parses identically whether the working tree checks out CRLF (Windows)
 *       or LF (Linux CI).  `std::getline` strips `\n` but not `\r`, leaving
 *       a trailing `\r` in the last cell of every CRLF line.  Trimming it
 *       here ensures a cell like `"STOP_AT_LINE\r"` matches `"STOP_AT_LINE"`.
 */
inline std::string trim(const std::string& s)
{
    const std::string kWs = " \t\r\n";
    const std::size_t start = s.find_first_not_of(kWs);
    if (start == std::string::npos)
    {
        return "";
    }
    const std::size_t end = s.find_last_not_of(kWs);
    return s.substr(start, end - start + 1);
}

/**
 * @brief Split @p line on every `,` and trim each resulting cell.
 *
 * @param line Raw input line (already fetched by `std::getline`).
 * @return Vector of trimmed cell strings; one entry per comma-separated field.
 *
 * @note No quoted-field or embedded-comma handling is performed — the parity
 *       fixtures use neither.  Keeping this simple avoids over-engineering a
 *       full CSV parser.
 */
inline std::vector<std::string> splitLine(const std::string& line)
{
    std::vector<std::string> cells;
    std::size_t start = 0;
    while (true)
    {
        const std::size_t comma = line.find(',', start);
        if (comma == std::string::npos)
        {
            cells.push_back(trim(line.substr(start)));
            break;
        }
        cells.push_back(trim(line.substr(start, comma - start)));
        start = comma + 1;
    }
    return cells;
}

}    // namespace detail

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief Parsed CSV table with named-column lookup.
 *
 * @details
 * Populated by `wro_csv::load`.  All cells are trimmed of leading/trailing
 * whitespace and an optional trailing `\r` so the data is identical whether
 * the file was checked out with CRLF (Windows) or LF (Linux CI) line endings.
 *
 * Access cells by column name:
 * @code
 *   int col = table.colIndex("my_column");  // -1 if absent
 *   ASSERT_TRUE(col >= 0);
 *   const std::string& val = wro_csv::field(table, rowIdx, col);
 * @endcode
 */
struct Table
{
    /** @brief Column names from the header row, trimmed. */
    std::vector<std::string> header;

    /**
     * @brief Data rows; each inner vector holds one row's cells, trimmed.
     *        `rows[i][j]` is the cell at zero-based row `i`, column `j`.
     */
    std::vector<std::vector<std::string>> rows;

    /**
     * @brief Return the zero-based index of the column named @p name, or
     *        `-1` if no such column exists in the header.
     *
     * @param name Column name to search for (exact match, case-sensitive).
     * @return Index of the column in `header`, or `-1` if absent.
     *
     * @note Callers that require the column to be present should assert the
     *       return value before passing it to `wro_csv::field`.  `field`
     *       will abort via `assert` on a `-1` argument rather than silently
     *       indexing with -1, which is undefined behaviour.
     */
    int colIndex(const std::string& name) const
    {
        for (int i = 0; i < static_cast<int>(header.size()); ++i)
        {
            if (header[static_cast<std::size_t>(i)] == name)
            {
                return i;
            }
        }
        return -1;
    }
};

/**
 * @brief Load a CSV file at @p path into @p out.
 *
 * @details
 * Opens @p path with `std::ifstream`.  The first non-empty line is parsed as
 * the header row: columns are split on `,` and each column name is trimmed.
 * Each subsequent non-empty line is parsed as a data row and appended to
 * `out.rows`.  A trailing blank line at EOF (produced by a final newline) is
 * silently skipped and does not produce a spurious empty row.
 *
 * Every field is trimmed of leading/trailing ASCII whitespace, including
 * `\r`, so the parsed values are identical whether the file on disk uses
 * CRLF (Windows working tree) or LF (Linux CI checkout) line endings.
 * `std::getline` strips `\n` but not `\r`; the trim step removes the
 * residual `\r` so a cell like `"STOP_AT_LINE\r"` matches `"STOP_AT_LINE"`.
 *
 * No quoted-field or embedded-comma handling is performed.
 *
 * @param path Absolute or relative path to the CSV file.  Forward slashes
 *             are accepted on both Windows and Linux; CMake bakes the
 *             compile-definition path with forward slashes.
 * @param out  Output table; cleared on entry and populated on success.
 * @param err  Set to a diagnostic message on failure; unchanged on success.
 * @return `true` on success; `false` if the file cannot be opened, contains no
 *         non-empty lines (no header row), or has a data row whose cell count
 *         differs from the header (a ragged row fails loudly via @p err).
 *
 * @note `out` may be in a partial state if the file opens but is empty.
 *       Discard `out` on any `false` return.
 */
inline bool load(const char* path, Table& out, std::string& err)
{
    out.header.clear();
    out.rows.clear();

    std::ifstream file(path);
    if (!file.is_open())
    {
        err = std::string("wro_csv::load: cannot open '") + path + "'";
        return false;
    }

    std::string line;
    bool headerParsed = false;

    while (std::getline(file, line))
    {
        // Trim the whole line to detect blank lines (including CRLF-only
        // lines where std::getline leaves a trailing '\r').  A blank line
        // is skipped; this also silently drops the spurious empty row that
        // a final newline would otherwise produce at EOF.
        if (detail::trim(line).empty())
        {
            continue;
        }

        if (!headerParsed)
        {
            out.header  = detail::splitLine(line);
            headerParsed = true;
        }
        else
        {
            std::vector<std::string> cells = detail::splitLine(line);
            // Reject a ragged row (fewer / more cells than the header) loudly
            // rather than letting `field()` index out of range later. The current
            // parity fixtures are rectangular so this never trips, but the helper
            // is lifted unchanged by issue #108's harnesses, where a hand-edited
            // fixture could be ragged — so the guard upholds the helper's own
            // "fail loudly, never silently" contract.
            if (cells.size() != out.header.size())
            {
                err = std::string("wro_csv::load: data row ") +
                      std::to_string(out.rows.size()) + " has " + std::to_string(cells.size()) +
                      " cells but the header has " + std::to_string(out.header.size()) +
                      " columns, in '" + path + "'";
                return false;
            }
            out.rows.push_back(cells);
        }
    }

    if (!headerParsed)
    {
        err = std::string("wro_csv::load: no header row found in '") + path + "'";
        return false;
    }

    return true;
}

/**
 * @brief Return a const reference to the cell at @p row, column @p col.
 *
 * @details
 * Convenience accessor combining a column index (from `Table::colIndex`)
 * with a row index.  Aborts via `assert` if @p col is negative, so a `-1`
 * returned by `colIndex` for a missing column produces a loud diagnostic
 * instead of silently indexing with -1 (undefined behaviour).
 *
 * @param t   The parsed table populated by `wro_csv::load`.
 * @param row Zero-based row index into `t.rows`.
 * @param col Zero-based column index, as returned by `Table::colIndex`.
 *            Must be `>= 0`; asserts and aborts if negative.
 * @return Const reference to the trimmed cell string.
 *
 * @note No bounds-check is performed on @p row — out-of-range access is
 *       the caller's responsibility.  Verify the row count before entering
 *       the loop (e.g. `ASSERT_EQ(t.rows.size(), kExpectedRowCount)`).  The
 *       column index needs no per-row guard: `load()` rejects a ragged table,
 *       so any valid `colIndex` result is in range for every row.
 */
inline const std::string& field(const Table& t, std::size_t row, int col)
{
    assert(col >= 0 && "wro_csv::field: col is -1 — column missing from header");
    return t.rows[row][static_cast<std::size_t>(col)];
}

}    // namespace wro_csv

#endif    // CSV_TRUTH_TABLE_H
