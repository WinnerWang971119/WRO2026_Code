# Stop-at-line full-domain C++ CSV parity harness (#63) Plan

**Goal:** Give the C++ `resolveStopAtLine` the same mechanical safety-net the Python resolver already has — a host (ctest) test that drives **all 936 rows** of the shared `stop_at_line_truth_table.csv` through the real `StopAtLinePolicy.h::resolveStopAtLine`, asserting `(seek, motionMode)` per row — so any future drift in the C++ resolver turns CI red. Closes issue #63 (part of epic #107).

**Approach:** Add a new **standalone minimal** ctest target (`test_stop_at_line_truth_table`) that compiles only the new test `.cpp` + the header-only `StopAtLinePolicy.h` against the existing host mocks — no real `PathTracker`/`Drive`/`routes.cpp` link, because the resolver is a pure `inline` function. The test reads the CSV from a **compile-defined absolute path**, parses it by **column name** through a new **reusable, CSV-mechanics-only** helper header (`CsvTruthTable.h`) that issue #108 will reuse for the other resolvers, maps the enum-name strings to the production enums, and asserts every row with a **report-all-capped** mismatch report. The target stays in the default suite (CI + "Run All" run it) but carries a `parity` ctest **label** so it can be skipped/selected locally. Finally, restore the "full domain" wording in the two C++ docstrings the issue cites.

## Scope

- **In scope:**
  - New host test `WRO2026/test/test_stop_at_line_truth_table.cpp`: loads the CSV, asserts all 936 rows through the production `resolveStopAtLine`, reports mismatches report-all-capped.
  - New reusable header-only helper `WRO2026/test/CsvTruthTable.h`: CSV mechanics only (open file, parse header row, column-name lookup, CR/whitespace trim, row iteration, row-count). No resolver-specific logic — so #108's four resolvers can lift it.
  - `WRO2026/test/CMakeLists.txt`: new `add_executable` + `add_test` target, the CSV path baked via `target_compile_definitions`, and a `parity` ctest label. No `EXCLUDE_FROM_ALL`.
  - Restore the "every row / full domain" wording in `WRO2026/StopAtLinePolicy.h` and `WRO2026/test/test_path_tracker_stop_at_line.cpp` docstrings, cross-referencing the new harness.
  - A row-count guard (936) and string→enum maps that mirror Python's TC13.
  - After the work lands: a **comment** on epic #108 outlining the per-resolver broadening (feasibility table + that `CsvTruthTable.h` is reusable). Not separate GitHub issues.
- **Out of scope:**
  - Any change to `resolveStopAtLine` logic, the CSV fixture, or `gen_stop_at_line_truth_table.py` — the CSV is reused as-is.
  - Building full-domain harnesses for the other four resolvers (`DirectionPolicy`, `ReturnToNodePolicy`, `TurnSignPolicy`, `PivotTrimPolicy`) — that is epic #108.
  - Any `.github/workflows/ci.yml` change — the `host-tests` job already runs `ctest` over all targets, so the new test is auto-included.
  - Touching the existing real-FSM seek tests (TC-SL1/2/3) beyond the docstring restore.
  - A CI freshness check that the committed CSV matches the generator (Python TC13 doesn't regenerate either).

## Decisions

- **Standalone minimal target, not an extension of `test_path_tracker_stop_at_line.cpp`** — the resolver is header-only `inline`; the parity loop needs nothing from the heavy real-FSM link chain. This is also the clean unit #108 replicates per resolver. (User-chosen.)
- **CSV path via compile-definition** (`STOP_AT_LINE_TRUTH_TABLE_CSV="<abs path>"`) — robust regardless of ctest cwd/build-dir; CMake already reaches into `tools/path_planner/` via `TOY_ROUTES_DIR`. (User-chosen.)
- **Helper factors CSV mechanics only** — resolver-specific string→enum maps and the per-row assert stay in the test `.cpp`. Minimal, non-speculative; reusable for #108. (User-chosen.)
- **Report-all-capped mismatch report** — count every mismatch, print the first ~20 rows in full, fail with the total. Matches the repo's report-all `TestAssert` style and shows scale of breakage. (User-chosen.)
- **In CI + `parity` label, not `EXCLUDE_FROM_ALL`** — stays in the default suite so a resolver regression turns CI red (#63's purpose), but `ctest -LE parity` skips it locally and `ctest -L parity` / `ctest -R` runs only it. (User-chosen.)
- **#108 broadening recorded as a comment**, not sub-issues. (User-chosen.)
- **Phase 4.5 cross-vendor approach consult skipped** — single determined approach, test-only and low-risk (no money/security/irreversible dimension).
- **Host test code may use STL** (`<fstream>`/`<string>`/`<vector>`/`<unordered_map>`) — the firmware no-dynamic-allocation rule applies to the R4 firmware, not host `test/` code, which never runs on the robot (precedent: `ctps_dump.cpp`, `test_mission_dispatch.cpp`).

## Acceptance Criteria

- [ ] AC1: A new ctest target `test_stop_at_line_truth_table` builds on MSVC and g++ and reads `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv` via the compile-defined absolute path.
- [ ] AC2: The test drives **all 936 rows** through the production `resolveStopAtLine` and asserts `(seek, motionMode)` per row; all pass against the current resolver.
- [ ] AC3: A row-count guard asserts exactly **936** data rows — a truncated/empty CSV fails loudly (no vacuous pass), mirroring Python's `_EXPECTED_ROW_COUNT`.
- [ ] AC4: Columns are looked up **by name** from the parsed header (order-independent); an unknown column or an unmapped enum-name string fails loudly rather than silently defaulting.
- [ ] AC5: Each field is trimmed of trailing `\r`/whitespace so the CRLF CSV parses correctly under both MSVC (text mode) and g++/Linux CI.
- [ ] AC6: On mismatch the test prints the first ~20 mismatching rows in full (all 7 inputs + got vs expected) then fails with the total mismatch count.
- [ ] AC7: The target is in the default ctest suite (CI `host-tests` + "Run All" run it) and carries a `parity` label: `ctest -LE parity` skips it, `ctest -L parity` / `ctest -R test_stop_at_line_truth_table` runs only it. No `ci.yml` change.
- [ ] AC8: `CsvTruthTable.h` contains only CSV mechanics (load + header parse + name lookup + trim + row count) — zero resolver-specific logic — and lives in `WRO2026/test/` (not a `mocks/` dir).
- [ ] AC9: The "full domain" wording is restored in `StopAtLinePolicy.h` and `test_path_tracker_stop_at_line.cpp` docstrings, cross-referencing the new harness; no resolver logic changes anywhere.
- [ ] AC10: The new target links **no** real `PathTracker.cpp`/`Drive.cpp`/`routes.cpp`; full `ctest` passes locally with no regressions in the other targets.

## Data Model

The CSV column contract (the normative fixture, owned by `gen_stop_at_line_truth_table.py`; **do not change it here**). Value spellings are exact enum **member names**:

| Column | Maps to (C++) | Spellings |
|---|---|---|
| `callFlag` | `StopAtLine` | AUTO / ON / OFF |
| `edgeOverride` | `Routes::StopAtLineMode` | DEFAULT / ON / OFF |
| `edgeMode` | `Routes::BlindMode` | SYNC_TSHAPE / GYRO_TSHAPE / RAW_FLAT / STOP_AT_LINE |
| `pipelineBlind` | `Routes::BlindMode` | (same four) |
| `direction` | `Direction` | FRONT / REAR |
| `lineBoardCount` | `uint8_t` | 1 / 2 |
| `autoRuleActive` | `bool` | true / false |
| `expect_seek` | `bool` | true / false |
| `expect_motionMode` | `Routes::BlindMode` | (same four) |

The query/result types already exist in `StopAtLinePolicy.h`:
```cpp
struct StopAtLineQuery {
    StopAtLine             callFlag;
    Routes::StopAtLineMode edgeOverride;
    Routes::BlindMode      edgeMode;
    Routes::BlindMode      pipelineBlind;
    Direction              direction;
    uint8_t                lineBoardCount;
    bool                   autoRuleActive;
};
struct StopAtLineResolution { bool seek; Routes::BlindMode motionMode; };
inline StopAtLineResolution resolveStopAtLine(const StopAtLineQuery& q);
```

## Contracts & Interfaces

Single source of truth for the seams this plan introduces.

### Helper API — `WRO2026/test/CsvTruthTable.h` (owner: T1; consumers: T2, future #108)

Header-only, namespace `wro_csv`. CSV mechanics only — no enum/resolver knowledge.

```cpp
namespace wro_csv {
  struct Table {
      std::vector<std::string>              header;   // column names, trimmed
      std::vector<std::vector<std::string>> rows;     // each cell trimmed of CR/whitespace
      int colIndex(const std::string& name) const;    // index of column, or -1 if absent
  };
  // Loads `path`. Returns false + fills `err` on open/parse failure. Trims each
  // field of leading/trailing whitespace and trailing '\r' (CRLF-safe on Linux).
  bool load(const char* path, Table& out, std::string& err);

  // Convenience: field by column name for a given row index. Caller has validated
  // the column exists (colIndex != -1) and asserts on -1 before calling.
  const std::string& field(const Table& t, std::size_t row, int col);
}
```

### Compile-definition (owner: CMake/T3; consumer: T2)

- `STOP_AT_LINE_TRUTH_TABLE_CSV` — string literal absolute path to the fixture, baked by `target_compile_definitions`. The test references it as the only path source.

### ctest label (owner: CMake/T3)

- Label `parity` on `test_stop_at_line_truth_table`. Documented usage: `-LE parity` to skip, `-L parity` to run only.

### Row-count constant (owner: T2)

- `static constexpr std::size_t kExpectedRowCount = 936;` with a comment naming `gen_stop_at_line_truth_table.py` as the source of truth (mirrors Python `_EXPECTED_ROW_COUNT`).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/test/CsvTruthTable.h` (new) | T1 | T2 |
| `WRO2026/test/test_stop_at_line_truth_table.cpp` (new) | T2 | T3 |
| `WRO2026/test/CMakeLists.txt` | T3 | — |
| `WRO2026/StopAtLinePolicy.h` (docstring only) | T4 | — |
| `WRO2026/test/test_path_tracker_stop_at_line.cpp` (docstring only) | T4 | — |

No two tasks edit the same file.

## Error Handling

- **CSV missing / unreadable:** `wro_csv::load` returns false with a message; the test asserts loaded and fails with the path (mirrors Python's `_FIXTURE_PATH.exists()` guard).
- **Row count ≠ 936:** hard fail naming actual vs expected (truncation guard).
- **Unknown column name:** `colIndex` returns -1; the test asserts each required column is present before the loop and fails loudly.
- **Unmapped enum string:** the local string→enum lookup fails loudly (e.g. returns a sentinel the test asserts against / aborts) rather than silently mapping to a default — matches Python's `_MAP[key]` `KeyError`.
- **Row mismatch:** accumulate count; print first ~20 in full; `TEST_REPORT()` exits nonzero with the total.

## Testing Strategy

**Levels:** the deliverable *is* a host test. Verification is building + running it, a mutation sanity check, and a full-suite regression run.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | Build + run `test_stop_at_line_truth_table` against the current resolver | Integration | Exit 0; prints `PASS: <n> assertions` with n ≈ 1872 (936×2) |
| TC2 | Row-count guard with a truncated CSV (manual/local spot check, reverted) | Unit | Fails with "expected 936 … found N" |
| TC3 | Mutation sanity: temporarily flip one `expect_seek`/`expect_motionMode` cell | Unit | Test fails and prints that exact row's inputs + got/expected; revert after |
| TC4 | CRLF handling on Linux/g++ (CI run) | Integration | All 936 rows parse; no trailing-`\r` mismatch on the last column |
| TC5 | Label behavior: `ctest -LE parity` then `ctest -L parity` | Integration | First run excludes the target; second runs only it |
| TC6 | Full `ctest --test-dir build/test` | Integration | All existing targets still pass; new target passes |

**Test data:** the committed `stop_at_line_truth_table.csv` (936 rows). No new fixtures.
**Run command:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`. Local MSVC: drive the pre-configured VS-generator `build/` tree with the bundled `cmake --build … --config Debug` (do **not** use the vcvars/cmd bridge — see Notes).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Reusable CSV helper | — | med | `WRO2026/test/CsvTruthTable.h` (new) | Create header-only `namespace wro_csv` with `Table` (header + rows + `colIndex`), `load(path, out, err)`, and `field(...)` per the Contracts section. Open with `<fstream>`; split on `,`; **trim each field of leading/trailing whitespace and trailing `\r`** (CRLF-safe). No quoted-field/escaping handling needed (the fixture has none). No enum/resolver knowledge. Doxygen on the namespace + each function. Satisfies AC4, AC5, AC8. |
| T2 | Parity test main | T1 | med | `WRO2026/test/test_stop_at_line_truth_table.cpp` (new) | `#include "TestAssert.h"`, `"CsvTruthTable.h"`, `"StopAtLinePolicy.h"`. Define `kExpectedRowCount = 936` (comment: generator is SoT). Define local `static` string→enum maps for `StopAtLine` / `Routes::StopAtLineMode` / `Routes::BlindMode` / `Direction` mirroring the enum member names exactly; an unknown string aborts loudly. In `main()`: `wro_csv::load(STOP_AT_LINE_TRUTH_TABLE_CSV, …)`, assert loaded, assert each of the 9 columns present, assert `rows.size() == kExpectedRowCount`, then loop: build `StopAtLineQuery`, call `resolveStopAtLine`, compare to `expect_seek`/`expect_motionMode`; on mismatch increment a counter and (if under the ~20 cap) `fprintf` the full row (7 inputs + got vs expected). Use a per-iteration `TEST_SCOPE` carrying the row's inputs so a captured failure is identifiable. End with one `ASSERT_TRUE(mismatchCount == 0)` (or report total) + `TEST_REPORT()`. Satisfies AC2, AC3, AC4, AC6. |
| T3 | CMake target + label | T2 | med | `WRO2026/test/CMakeLists.txt` | Add `add_executable(test_stop_at_line_truth_table ${TEST_DIR}/test_stop_at_line_truth_table.cpp ${MOCK_SOURCES})` (mirror the lightweight `test_stop_at_line_armmath` block — `MOCK_SOURCES` for any extern `Serial`/`g_mockMillis` the mock `Arduino.h` declares). `target_include_directories(... PRIVATE ${COMMON_INCLUDES})` (mocks-first; **no** force-includes — Config.h pulls only `<Arduino.h>` + config sub-headers). Bake the CSV path: `set(STOP_AT_LINE_CSV ${SKETCH_DIR}/../tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv)` then `target_compile_definitions(... PRIVATE STOP_AT_LINE_TRUTH_TABLE_CSV="${STOP_AT_LINE_CSV}")`. `add_test(NAME stop_at_line_truth_table COMMAND test_stop_at_line_truth_table)` and `set_tests_properties(stop_at_line_truth_table PROPERTIES LABELS "parity")`. Satisfies AC1, AC7, AC10. |
| T4 | Restore docstrings | — | low | `WRO2026/StopAtLinePolicy.h`, `WRO2026/test/test_path_tracker_stop_at_line.cpp` | In `StopAtLinePolicy.h` (~line 64) change the "the C++ tests cover representative rows" clause to state both resolvers are now verified against the full domain (Python TC13 + the new `test_stop_at_line_truth_table.cpp` CSV harness). In `test_path_tracker_stop_at_line.cpp` update the TC-SL4 docblock (~line 344) and the `main()` "representative vectors" comment (~line 903) to note the full-domain parity now lives in the sibling harness and these in-FSM-target vectors are the kept representative subset. **Do NOT** touch the "representative lower-MCU seek" comments (~lines 41, 201) — those are about the seek call, not the resolver domain. No logic changes. Satisfies AC9. |
| T5 | Build + verify | T3, T4 | low | (no source edits) | Configure + build + run: confirm the new target passes (≈1872 assertions, AC2), do the TC3 mutation sanity (flip one cell → fails → revert), confirm `ctest -LE parity` skips and `ctest -L parity` runs only it (AC7), and run full `ctest` for no regressions (AC10). Report results. |

T1 and T4 have no blockers → parallel. T2 waits on T1. T3 waits on T2. T5 waits on T3 + T4.

## Notes for Implementer

- **The CSV is CRLF** (`\r\n`, verified via `od -c`). Under g++ on Linux CI, text mode does **not** strip the `\r`, so the last column per row (`expect_motionMode`) would carry a trailing `\r` and never match. Trimming `\r`/whitespace per field in `wro_csv::load` (T1) is load-bearing, not cosmetic — TC4 guards it.
- **Why the target is minimal (and avoids a known trap):** host FSM tests that link the real `routes.cpp` resolve nodes by hardcoded index and segfault when the node set changes (memory: *host-tests-link-production-routes*, issue #91). This target sidesteps that entirely — `resolveStopAtLine` reads nothing from the route tables, so we link no `routes.cpp` and need no fixture-routes shadow. Including production `routes.h` for the enum *types* is fine (no symbols ODR-used).
- **Do not put the helper in a `mocks/` dir.** `mocks/` (and the other mock dirs) are on every target's include path; a header there shadows the real header for *all* targets (memory: *mocks-dir-shadows-all-targets*). `CsvTruthTable.h` is a plain `test/` header included only by this test (and later #108 tests).
- **Compile-def path uses forward slashes.** CMake path variables use `/` even on Windows, so the baked string literal needs no backslash escaping and `fopen`/`ifstream` accept it on both OSes.
- **Local MSVC build:** the VS Build Tools toolchain is available but not on PATH; drive the pre-configured VS-generator `build/` tree with the bundled `cmake --build <dir> --config Debug` (MSBuild self-discovers the toolchain). Do **not** use a `cmd /c vcvars` bridge — it imports zero env vars here (memory: *host-build-vs-generator-not-vcvars-cmd*). Probe the toolchain via PowerShell `vswhere`.
- **String→enum independence:** the maps in T2 hand-mirror the enum member names (the CSV uses exact member-name spellings) — this is deliberate, mirroring how the generator and Python TC13 stay independent transcriptions. An unknown string must abort, never default.
- **No `routes.h`/`routes.cpp` regeneration** — this plan touches neither the emitter nor field data, so the CI routes-freshness gate is not in play.
- **#108 follow-up (after merge):** post a comment on epic #108 with the per-resolver feasibility table (StopAtLine exhaustive; Direction/ReturnToNode/TurnSign bucketed-enumerable; PivotTrim representative-only) and note `CsvTruthTable.h` is the reusable base. Not separate issues.
- **Rollback:** the change is additive (two new files + one CMake target) plus comment-only docstring edits; reverting the commit fully removes it with zero impact on firmware or other tests.
