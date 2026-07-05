# WRO2026 Sketch-Tree Reorg (issue #66) Plan

**Goal:** Move the 38 framework `.cpp`/`.h` modules out of the flat `WRO2026/` sketch root into domain subfolders under `WRO2026/src/`, keeping the operator-editable files at the root, so the sketch root stops being a wall of ~40 files and clearly separates "what the operator may modify" from "framework internals."

**Approach:** A two-tier split by **operator-editability**. The sketch root keeps exactly the files the operator touches during competition (`WRO2026.ino`, `Config.h` + `config/`, `MissionScript.{h,cpp}`, `Tasks.{h,cpp}` + `TasksTypes.h`); everything else moves into six `src/<domain>/` framework folders. Because Arduino resolves `#include "..."` only relative to the including file's own directory, every cross-boundary include is rewritten to its explicit relative form; the host-test harness is reconciled by rewriting its production-header includes to root-relative `"src/<domain>/…"` form (leaving every CMake `-I` list untouched), which keeps host include-resolution congruent with Arduino's. The change lands as layout-neutral prep commits followed by one atomic `git mv` move commit.

## Scope

- **In scope:**
  - Move 38 framework modules into `src/{drive,perception,nav,mission,manip,util}/` (exact mapping in Contracts).
  - Rewrite all ~73 cross-boundary production quote-includes + the host-harness/mock/fixture production includes to their correct relative/root-relative form.
  - Update `WRO2026/test/CMakeLists.txt` source paths (`-I` lists unchanged).
  - Move generated `routes.{h,cpp}` to `src/nav/`: update `header_writer.py` output path + coupled tests, `ci.yml` routes-diff, `.gitattributes`.
  - Harden the two "silent-green" guards (routes-freshness existence check; `check_host_coverage.py` recursive glob + path-keying + non-empty floors) and add a permanent include-resolvability checker.
  - Fix the off-CI seams: `test_ctps_mock_superset.py` repo-root landmark, `.vscode/c_cpp_properties.json`, `vision_bench.ino`, and the `--library WRO2026` diagnostic mechanic.
  - New ADR for the layout convention + user-facing doc file-tree updates.
- **Out of scope:**
  - Any behavioral / logic change to firmware, tests, or the planner — this is a pure move + path-reconciliation. Zero runtime behavior changes.
  - Deleting `StateMachine.h` (kept, relocated to `src/util/`).
  - Moving `Config.h`/`config/` or the operator root set (they stay at root — decided).
  - Rewriting the `sim/` "MUST mirror" provenance comments and archived `docs/` history (frozen — intentionally left to drift).
  - Any `field_data.json` / route-data **re-planning**: routes are only *relocated*. (Not byte-identical, though: the emitter's `#include` line is now derived from the output header's basename, so production `routes.cpp` stays `#include "routes.h"` while each generated fixture/toy `.cpp` emits its own sibling include — expected, gate-checked regenerations. See T8.)

## Decisions

- **Split axis = operator-editability, not pure technical domain** — the root becomes the operator's editable surface, `src/` signals "framework, don't touch." Chosen by the user as the organizing principle; it also solves the flat-wall problem.
- **Operator root set stays at root:** `WRO2026.ino` (forced by Arduino), `Config.h` + `config/`, `MissionScript.{h,cpp}`, `Tasks.{h,cpp}`, `TasksTypes.h` — the tunables + mission + task registration the operator edits in competition. Keeping `config/` at root also leaves the Python planner's `config_parse.py`/`validator.py` paths untouched.
- **Six framework domains:** `drive`, `perception`, `nav`, `mission`, `manip`, `util` (user pick, fewest cross-domain include rewrites). The five resolver policies live in `nav/` alongside their `NavTypes`+`routes` deps — **except `TurnSignPolicy.h`**, which is Drive-only and lives in `src/drive/` (the mapping in Contracts is the source of truth).
- **`routes.{h,cpp}` → `src/nav/`** (generated files nest with the rest; generator output path + CI freshness + `.gitattributes` follow). *Rejected alternative — leave the generated pair at the sketch root:* it would delete most of T8 and avoid the emitter-include seam + fixture/toy rebaselines, but it puts two non-operator-editable *generated* files at the root, contradicting the "root = operator-editable only" principle. Moving them honors that principle; the emitter fix (below) is the accepted cost. (Flagged for the user at approval.)
- **`Mission.{h,cpp}` engine → `src/mission/`**, split from the operator-authored `MissionScript.{h,cpp}` which stays at root.
- **`StateMachine.h` → `src/util/`** (kept as a shared helper, not deleted).
- **Harness reconciliation = rewrite includes, not `-I` dirs** (Fable veto adopted): rewrite every host-test production-header include to root-relative `"src/<domain>/…"` and leave all CMake `-I` lists unchanged. Adding `src/<domain>/` to the `-I` lists was rejected — it makes bare-name resolution order-dependent against five mock dirs carrying colliding basenames, and creates host-vs-Arduino divergence (bare names that resolve on host but never under Arduino). After the move, in a test file **bare `"Drive.h"` = mock, `"src/drive/Drive.h"` = real**.
- **Sequence = layout-neutral prep commits → one atomic move commit** (Fable refinement of the user's "atomic move" choice): prep makes the guards/tests layout-agnostic and builds the rewrite tooling while staying green under the *old* layout; the move then lands as a single reviewable `git mv` + scripted rewrite.
- **`git mv` for every file** to preserve `git log --follow` history.
- **Scripted rewrite from a single path-mapping table** (not hand edits) for a replayable, reviewable diff.
- **Docs scope = CI-red seams + a new ADR + user-facing file-trees**; skip `sim/` mirror comments and archived docs.
- **Approach consult:** Fable **vetoed two specifics** (harness `-I` reconciliation; missed breakage points) and endorsed the overall shape — both folded in. Codex **skipped** (usage limit until ~5:19 PM) — Fable-only consult this run.

## Acceptance Criteria

- [ ] AC1 — `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026` succeeds with all framework sources under `src/<domain>/`.
- [ ] AC2 — `cmake -S WRO2026/test -B build/test`, `cmake --build build/test`, `ctest --test-dir build/test --output-on-failure` all pass, **including** the compile-fail ctests (`tasks_overflow_compile_fail`, and any belt/servo compile-fail targets) which must still fail on their **intended `static_assert` message**, not an include-not-found.
- [ ] AC3 — the **parity** job (build `test_path_tracker_parity_dump` + `test_pathtracker_parity.py`) and the **ctps-e2e** job (build `ctps_dump` + the ctps pytest) both pass.
- [ ] AC4 — `uv run --extra dev pytest tools/path_planner/tests/` passes, including `test_header_writer.py` and `test_build_routes.py` updated for the new `src/nav/routes.*` path.
- [ ] AC5 — `routes.{h,cpp}` regenerated at `WRO2026/src/nav/`; `git diff --exit-code -- WRO2026/src/nav/routes.h WRO2026/src/nav/routes.cpp` is clean, and the CI routes-freshness step has a **file-existence guard** so a stale pathspec cannot pass vacuously.
- [ ] AC5b — the **fixture-freshness** gate (regenerated `routes_fixture.{h,cpp}` + `routes_reset_passthrough_forced.*`) and the **toy-golden** pytest (`test_routes_cpp_matches_golden` / `test_routes_h_matches_golden`) pass post-move; each generated `.cpp` carries its own sibling `#include` (from the emitter, not a hand-edit) and its `git diff` is clean.
- [ ] AC6 — `check_host_coverage.py` passes with recursive glob + path-keying + non-empty sanity floors; a **mutation test** (drop one `${SKETCH_DIR}/src/...` source from CMakeLists) makes it go **red**.
- [ ] AC7 — the include-resolvability checker passes (every `#include "..."` under `WRO2026/` resolves under the correct rule) and is wired into CI.
- [ ] AC8 — operator root set is exactly `{WRO2026.ino, Config.h, config/, MissionScript.{h,cpp}, Tasks.{h,cpp}, TasksTypes.h}`; every other framework module lives under `src/<domain>/` per the mapping.
- [ ] AC9 — history preserved: `git log --follow` traces at least one moved file across the rename.
- [ ] AC10 — ADR written + `DECISIONS.md` updated; user-facing doc file-trees updated; the `--library WRO2026` diagnostic replacement documented; `vision_bench.ino` compiles under the new layout.
- [ ] AC11 — the full CI matrix (all five jobs) is green on the PR.

## Contracts & Interfaces

This is a path-migration; the binding cross-task contract is the **file→location mapping** and the **include-form rules**. Every task that moves a file or rewrites an include conforms to these exactly.

### File → destination mapping (the single source of truth)

**Stay at sketch root (operator-editable — do NOT move):**
`WRO2026.ino`, `Config.h`, `config/` (all 8 sub-headers), `MissionScript.h`, `MissionScript.cpp`, `Tasks.h`, `Tasks.cpp`, `TasksTypes.h`.

**`src/drive/`** — `Drive.h`, `Drive.cpp`, `LineFollower.h`, `LineFollower.cpp`, `Intersection.h`, `Intersection.cpp`, `TurnSignPolicy.h`
**`src/perception/`** — `WROLineArray.h`, `WROLineArray.cpp`, `Vision.h`, `Vision.cpp`, `VisionFrame.h`
**`src/nav/`** — `routes.h`, `routes.cpp`, `FieldMap.h`, `NavTypes.h`, `Localization.h`, `Localization.cpp`, `YawSource.h`, `YawSource.cpp`, `DirectionPolicy.h`, `StopAtLinePolicy.h`, `EdgeDispatchPolicy.h`, `PivotTrimPolicy.h`, `ReturnToNodePolicy.h`
**`src/mission/`** — `Mission.h`, `Mission.cpp`, `PathTracker.h`, `PathTracker.cpp`, `PathTrackerParityTrace.h`
**`src/manip/`** — `Manipulator.h`, `Manipulator.cpp`, `ManipServo.h`, `ManipBelt.h`
**`src/util/`** — `WROMath.h`, `StateMachine.h`, `EventTrace.h`, `EventTrace.cpp`

### Include-form rules (apply uniformly; angle-bracket `<...>` includes NEVER change)

| Including file location | Target header | New include form |
|---|---|---|
| `src/<A>/` | same-domain header in `src/<A>/` | `"Header.h"` (bare — unchanged) |
| `src/<A>/` | header in sibling `src/<B>/` | `"../<B>/Header.h"` |
| `src/<A>/` | root header (`Config.h`, `Tasks.h`, `TasksTypes.h`, `MissionScript.h`) | `"../../Header.h"` |
| `src/<A>/` | a `config/` sub-header (e.g. `ManipServo.h`→`ConfigManipulator.h`, `Vision.h`/`VisionFrame.h`→`ConfigVision.h`) | `"../../config/ConfigX.h"` |
| root file (`.ino`, `MissionScript.cpp`, `Tasks.cpp`, `Tasks.h`, `TasksTypes.h`) | framework header in `src/<B>/` | `"src/<B>/Header.h"` |
| root file | root header | `"Header.h"` (bare — unchanged) |
| `config/` sub-header | sibling `config/` sub-header | `"ConfigX.h"` (bare — unchanged) |
| `Config.h` (root) | its `config/` sub-headers | `"config/ConfigX.h"` (unchanged) |
| **any `WRO2026/test/` file** (mock header, harness header, fixture TU) | a **production** header now under `src/<B>/` | `"src/<B>/Header.h"` (root-relative; resolves via existing `-I ${SKETCH_DIR}`) |

### Host-test shadow invariant (must remain true post-move)

- Mock force-includes are **unchanged**: mocks are force-included by absolute path (`${MOCKS_DIR}/…`, `${HAL_MOCKS_DIR}/…`, `${PATH_MOCKS_DIR}/…`, `${TASK_MOCKS_DIR}/…`, `${CTPS_MOCKS_DIR}/…`, `${MISSION_MOCKS_DIR}/…`) with include guards matching production (`DRIVE_H`, `WROLINEARRAY_H`, …). The real header (now at `src/<domain>/`) still no-ops when later included by any path.
- All CMake `-I` lists are **unchanged**. Only `${SKETCH_DIR}/<name>.cpp` source paths become `${SKETCH_DIR}/src/<domain>/<name>.cpp`.
- **Hand-written mock/harness/test files that include a real (unmocked) leaf header must be hand-rewritten** to the root-relative `"src/<domain>/…"` form — these are NOT shadowed, they pull the real header for real. Known set (verify at execute time): `mocks/Drive.h`→`WROMath.h`; `hal_mocks/Drive.h`→`WROMath.h`; `task_mocks/PathTracker.h`→`NavTypes.h`; `mission_mocks/PathTracker.h`→`NavTypes.h`; `ctps_mocks/PathTracker.h`→`FieldMap.h`,`NavTypes.h`,`routes.h`; `test_path_tracker_harness.h`→ real Drive/PathTracker/Localization/YawSource/FieldMap/Config; the ~25 hand-written test `.cpp` TUs (`test_line_follower_overshoot.cpp`, `test_vision.cpp`, `test_mission_dispatch.cpp`, the five policy tests, the reanchor/localization tests, `ctps_dump.cpp`, …).
- **Generated TUs get their `#include` from the emitter, NOT a hand-edit.** The fixture TUs (`routes_fixture.cpp`, `routes_reset_passthrough_forced.cpp`) and the toy golden (`routes_toy.cpp`) are emitter output. Do NOT hand-rewrite their `#include "routes.h"` to `"src/nav/routes.h"` — that self-reverts on the next regeneration and reddens the freshness/golden gates. Instead the emitter derives the include from the output header's basename (T8), so each emits its own sibling header (`routes_fixture.h` / `routes_reset_passthrough_forced.h` / `routes_toy.h`), resolving same-dir. **Before hand-editing any generated-looking `.cpp`, verify it is not emitter-generated.**

### Generator + CI path seams

- `tools/path_planner/header_writer.py`: `DEFAULT_HEADER_PATH = Path("WRO2026/src/nav/routes.h")`, `DEFAULT_SOURCE_PATH = Path("WRO2026/src/nav/routes.cpp")` (currently lines 90-91). CLI `--out-dir` help text updated.
- `tools/path_planner/header_writer.py:948` — the hardcoded `lines.append('#include "routes.h"')` becomes derived from the output header's basename (thread `header_path.name` into `_build_routes_cpp_lines` via `emit_routes`). Production `routes.cpp` still emits `#include "routes.h"` (byte-identical, resolves same-dir in `src/nav/`); each generated fixture/toy `.cpp` emits its own sibling (`routes_fixture.h` / `routes_reset_passthrough_forced.h` / `routes_toy.h`). This is the fix for the fixture-freshness and toy-golden breakage.
- `tools/path_planner/tests/test_header_writer.py` — (a) update the `DEFAULT_HEADER_PATH`/`SOURCE_PATH` value assertion to the new `src/nav/` paths; (b) realign the `emit_targets` fixture (lines ~172-174) to emit to `tmp_path/"routes_toy.h"` + `"routes_toy.cpp"` so the golden-comparison emit uses the **same basename** as `regenerate_goldens()` — otherwise `test_routes_cpp_matches_golden` diverges on the include line and fails even after a correct rebaseline. Rebaseline the toy golden via the module's `__main__`.
- `.github/workflows/ci.yml` also carries a **second** freshness gate at lines ~43-46 (`regenerate_routes_fixture.py` + `git diff --exit-code` on `WRO2026/test/fixtures/routes_fixture.{h,cpp}`) — the fixture stays under `test/fixtures/`, but its regenerated `.cpp` include line changes, so this gate must be re-run (green) inside the move commit.
- `.github/workflows/ci.yml`: routes-diff pathspec → `WRO2026/src/nav/routes.{h,cpp}`, preceded by a `test -f` existence guard.
- `.gitattributes`: `WRO2026/src/nav/routes.h`/`.cpp` eol=lf.
- `WRO2026/test/CMakeLists.txt`: the sole production-data reference `${SKETCH_DIR}/routes.cpp` (line ~1038) → `${SKETCH_DIR}/src/nav/routes.cpp`.

### Naming

- New ADR number: **next unused, resolved at execute/merge time.** On this branch the max file is `052`; `053` appears free here but is **claimed by the unmerged CTPS branch (#125)** whose `DECISIONS.md` entry exists. Assign the next genuinely-free number at merge time (likely **054**) and renumber to avoid collision, per the ADR-collision gotcha (renumber the branch that merges second, never one already on main).
- New tooling lives under `tools/` (e.g. `tools/reorg/` for the mapping+rewriter, `tools/path_planner/` or `WRO2026/test/` for the resolvability checker — implementer's call, keep it discoverable by CI).

## Error Handling

The migration's only dangerous failure class is **silent green** — a seam that passes CI while actually broken. Everything else (a wrong include, a wrong CMake source path) fails loudly at compile. Explicit mitigations:

- **Routes-freshness vacuous pass:** `git diff --exit-code -- <path>` exits 0 when the pathspec matches nothing. Guard with `test -f WRO2026/src/nav/routes.h` (and `.cpp`) before the diff; fail the job if the file is absent.
- **Coverage-guard vacuous pass:** a bug in the new recursive glob or the `LINK_RE` (which currently excludes `/`) yields an empty production or linked set → zero violations → green. Add sanity floors: the production set must be non-empty AND `linked ∩ production` must be non-empty, else raise a violation. Prove it with a mutation test (drop one linked source, expect red).
- **Includes hidden behind build flags:** includes behind `#ifdef PT_PARITY_TRACE` / `CTPS_RECORD` (and any debug-only flag) are compiled only by the target that defines them — one behind a flag no CI target defines is checked by nothing. The static include-resolvability checker walks every `#include "..."` regardless of flags and asserts the target file exists under the correct rule.
- **Off-CI mechanics:** the `--library WRO2026` diagnostic and `vision_bench.ino` are not CI-gated and break silently until used on the robot. Fix `vision_bench` and document the `--library` replacement (one `--library WRO2026/src/<domain>` flag per domain, legacy-layout compiles each dir's root) in the ADR.
- **Regeneration content drift (expected, gate-checked — not a surprise):** the emitted `#include` line is derived from the output header basename, so every generated `.cpp` changes its include on regeneration (production `routes.cpp` stays `#include "routes.h"`; the fixtures/toy gain their own sibling includes). Regenerate ALL generated artifacts — production routes, both fixtures, and the toy golden — inside the move commit and let the freshness/golden gates confirm them. If the banner additionally embeds the output path, that is a second drift source; verify and rebaseline the same way.

## Testing Strategy

**Levels:** compile (Arduino + host, MSVC & g++), host unit/integration (ctest), Python (pytest), static guards. All are existing suites — the move must keep every one green with zero behavioral change.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `arduino-cli compile … WRO2026` after the move | Compile (Arduino) | Green; framework `src/<domain>/*.cpp` compiled recursively (AC1) |
| TC2 | Full `ctest` incl. compile-fail targets | Host | All pass; compile-fail targets fail on the intended `static_assert` message, not include-not-found (AC2) |
| TC3 | Parity job: build `test_path_tracker_parity_dump` + `test_pathtracker_parity.py` | Integration | Trace parity unchanged (AC3) |
| TC4 | ctps-e2e: build `ctps_dump` + ctps pytest | Integration | ctps trace unchanged; `ctps_mocks` includes resolve (AC3) |
| TC5 | `pytest tools/path_planner/tests/` incl. `test_header_writer`, `test_build_routes` | Unit | Pass against new `src/nav/routes.*` path (AC4) |
| TC6 | Regenerate routes → `git diff --exit-code` on new path | Static/CI | Clean; existence guard fails if file absent (AC5) |
| TC6b | Regenerate fixtures + toy golden; run `test_routes_cpp_matches_golden` + compile `test_path_tracker_projection` | Static/CI | Fixture/toy diffs clean; golden tests green; projection target links (AC5b) |
| TC7 | `check_host_coverage.py` + mutation test | Static/CI | Passes normally; goes red when a linked src is removed (AC6) |
| TC8 | Include-resolvability checker over all `WRO2026/**` includes | Static/CI | Every `#include "..."` resolves under the correct rule (AC7) |
| TC9 | `git log --follow src/drive/Drive.cpp` | Static | History traces across the rename (AC9) |
| TC10 | Build under MSVC and g++ semantics | Compile (host) | Both green — guards against the documented MSVC/arduino-cli divergence |
| TC11 | `vision_bench.ino` compile under new include path | Compile | Green (AC10) |

**Test data / setup:** local MSVC toolchain (VS Build Tools; drive the pre-configured VS-generator `build/` tree with the bundled `cmake --build … --config Debug`, not vcvars-in-bash) and the bundled `arduino-cli` (renesas_uno core installed). **Run commands:** the five CI jobs locally — `arduino-cli compile … WRO2026`; `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`; the parity + ctps-e2e explicit builds; `uv run --extra dev pytest tools/path_planner/tests/`; `python WRO2026/test/check_host_coverage.py`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Path-mapping table + scripted include-rewriter | — | med | `tools/reorg/` (new) | Encode the file→destination mapping (Contracts) + the include-form rules as a small Python tool that (a) lists `git mv` operations and (b) rewrites `#include "..."` lines deterministically. **Test-file classification input:** for a `WRO2026/test/` file, whether a bare production include (e.g. `"PathTracker.h"`) means *mock* or *real* is NOT decidable from the file content alone — mock-intent in `test_mission_dispatch.cpp`, real-intent in the inversion targets. The tool must take per-target context (which mocks a target force-includes / its `-I` order from CMakeLists) or the repo's `// mock` / `// real` include-comment convention as the classification key; a bare include with no production counterpart at root after the move resolves only to a mock dir, so the tool rewrites to `"src/<domain>/…"` only the includes whose intent is *real*. Green/no-op under the old layout. Feeds every move task. |
| T2 | Make `check_host_coverage.py` layout-agnostic + hardened | — | high | `WRO2026/test/check_host_coverage.py` | Recursive glob of production `.cpp` under `WRO2026/` excluding `test/`; `LINK_RE` accepts `/`; the production-vs-linked comparison keys by repo-relative path (not basename, to avoid cross-domain collisions); add non-empty sanity floors (production set + `linked∩production`) and a duplicate-basename guard. **Keep the allowlist (`host_coverage_allowlist.json`) entries basename-keyed** so they stay layout-independent (no move-time re-path); match them against the basenames of the un-linked production set. Must pass under BOTH the current flat layout and the future nested layout. Satisfies AC6 error-handling. |
| T3 | Include-resolvability checker + CI wiring | — | med | `tools/…` (new), `.github/workflows/ci.yml` | New static checker. For **production** files under `WRO2026/` (excluding `test/`): strict Arduino rule — every `#include "..."` must resolve source-relative to the including file. For **`WRO2026/test/`** files: resolve against a candidate-root set (own dir, `${SKETCH_DIR}`, `${TEST_DIR}`, and the mock/fixture/toy dirs) since they legitimately reach `"Arduino.h"`, `"TestAssert.h"`, `"../Tasks.h"`, etc. — a sketch-dir-only rule would false-fail. Also assert the operator root set equals the AC8 whitelist. Wire a CI step. Lands green under the old layout. Satisfies AC7, AC8. |
| T4 | Decouple `test_build_routes.py` from the hardcoded routes path | — | low | `tools/path_planner/tests/test_build_routes.py` | Reference `header_writer.DEFAULT_HEADER_PATH`/`DEFAULT_SOURCE_PATH` instead of literal `WRO2026/routes.*`, so the test follows the constant. Green under old layout. |
| T5 | Scripted `git mv` + production include rewrite | T1, T2, T3, T4 | high | all 38 framework files (moved) + `WRO2026.ino`, `MissionScript.cpp`, `Tasks.h`, `Tasks.cpp`, `TasksTypes.h` (edited in place — root files stay put) | Run T1's tool: `git mv` every framework file to `src/<domain>/`; rewrite all cross-boundary production quote-includes per the include-form rules. `StateMachine.h`→`src/util/`. Preserves history. Satisfies AC8, AC9. |
| T6 | Rewrite hand-written host-test production includes | T1 | high | `WRO2026/test/mocks/`, `hal_mocks/`, `path_mocks/`, `task_mocks/`, `mission_mocks/`, `ctps_mocks/`, `test_path_tracker_harness.h`, the hand-written `WRO2026/test/*.cpp` TUs (incl. the `ctps_dump` driver) | Rewrite every **hand-written** `test/` file's include of a production header to root-relative `"src/<domain>/…"` (mock headers' own real-leaf includes + the ~25 hand-written test TUs — see the known set in Contracts; enumerate via T1's tool). **Do NOT touch the generated fixture/toy `.cpp` — those get their include from the emitter in T8** (verify per file whether emitter-generated before editing). Bare names remain the mocks. Leave force-include dirs + `-I` lists alone. |
| T7 | Update CMake source paths (`-I` unchanged) | T1 | high | `WRO2026/test/CMakeLists.txt` | For every `${SKETCH_DIR}/<name>.cpp` **whose module is in the move mapping** (the 9 host-linked modules that move: LineFollower, PathTracker, Drive, Localization, YawSource, routes, Manipulator, Vision, Mission), rewrite to `${SKETCH_DIR}/src/<domain>/<name>.cpp`. **Leave the `Tasks.cpp` and `MissionScript.cpp` refs untouched — those two stay at root.** ~78 of the ~82 `${SKETCH_DIR}/*.cpp` references move (incl. `routes.cpp` at ~1038). Do NOT touch any `target_include_directories` / `COMMON_INCLUDES`. |
| T8 | Move routes output + emitter include fix + CI freshness + regenerate all artifacts | T1, T3 | high | `tools/path_planner/header_writer.py`, `tools/path_planner/tests/test_header_writer.py`, `.github/workflows/ci.yml`, `.gitattributes`, `WRO2026/src/nav/routes.{h,cpp}`, `WRO2026/test/fixtures/routes_fixture.{h,cpp}`, `WRO2026/test/fixtures/routes_reset_passthrough_forced.{h,cpp}`, `tools/path_planner/tests/golden/routes_toy.{h,cpp}` | `DEFAULT_HEADER_PATH`/`SOURCE_PATH` → `src/nav/`; derive the emitted `#include` from the output header basename (`header_writer.py:948`); update the `test_header_writer` value assertion, realign its `emit_targets` fixture to the `routes_toy` basename, + CLI help. Regenerate ALL generated artifacts **inside the move commit**: production routes into `src/nav/` (`uv run … build_routes.py`), `regenerate_routes_fixture.py` (+ the reset-passthrough regen), and the toy golden via `test_header_writer` `__main__`. ci.yml: production routes-diff → new path + `test -f` existence guard, and confirm the fixture-freshness gate (lines ~43-46) stays green. `.gitattributes` routes paths. Satisfies AC5, AC5b. |
| T9 | Fix off-CI + editor seams | T1 | med | `tools/path_planner/tests/test_ctps_mock_superset.py`, `.vscode/c_cpp_properties.json`, `tools/vision_bench/vision_bench.ino` (+ its doc/flag) | Repo-root landmark `WRO2026/Drive.h`→ a stable post-move landmark; add `WRO2026/src/**` to IntelliSense `includePath`+`browse.path`; make `vision_bench` `src/perception`-aware. Satisfies AC10 (vision_bench). |
| T10 | ADR + docs + `--library` replacement | T1 | low | `docs/decisions/05N-*.md` (next unused), `DECISIONS.md`, `README.md`, `ARCHITECTURE.md`, `CLAUDE.md`, `AGENTS.md`, `docs/OPERATOR_GUIDE.md`, `tools/path_planner/README.md`, `WRO2026/test/README.md` | Write the layout ADR (root=operator / `src/`=framework; the Arduino source-relative-include rule; the root-relative harness-include convention; the `--library WRO2026/src/<domain>` diagnostic replacement). Update `DECISIONS.md` index + status table. Update the file-tree sections in the user-facing docs. Satisfies AC10. |
| T11 | Full local verification gate | T5, T6, T7, T8, T9, T10 | high | — (runs suites) | Run all five CI jobs locally + arduino-cli + full ctest (incl. compile-fail targets, which must still fail on their intended `static_assert`) + parity + ctps-e2e + `check_host_coverage` + the resolvability checker, under MSVC and g++. Run two **mutation checks**: (a) drop one `${SKETCH_DIR}/src/...` source → coverage guard goes red; (b) rename `src/nav/routes.h` → the routes-freshness existence guard fails (proves the guard path is right). Run one diagnostic compile with the documented `--library WRO2026/src/<domain>` per-domain flags to prove that mechanic. Confirm zero behavioral diff. Satisfies AC1–AC7, AC10, AC11. |

## Notes for Implementer

- **Atomicity:** T1–T4 are **prep commits**, each green under the *old* flat layout (land them first). T5–T10 are the **one atomic move commit** — stage them all and commit once (a partial move does not compile; do not commit T5–T10 individually). T11 verifies before the move commit is finalized; T3's CI wiring may be its own prep commit.
- **The mapping table (T1) is the contract** — every rewrite task consumes it; do not hand-derive paths per task.
- **`-I` lists are sacrosanct** — the whole point of the harness reconciliation is that CMake `-I` lists don't change. If you find yourself adding `src/<domain>/` to a `target_include_directories`, stop: rewrite the offending include to `"src/<domain>/…"` instead.
- **Never `#include` a `.cpp`** and never introduce a bare cross-domain include (`"Drive.h"` from another domain) in a production file — Arduino won't resolve it, and it would be caught only by arduino-cli, not host ctest.
- **Verify with arduino-cli, MSVC, AND g++.** Host ctest uses mocks + separate TUs and won't catch link/ODR or MSVC-only errors; arduino-cli is the only verifier of real source-relative resolution. Local MSVC build: drive the pre-configured VS-generator `build/` tree with the bundled `cmake --build … --config Debug` (MSBuild self-discovers the toolchain; the bash→cmd vcvars bridge is broken here).
- **Regeneration:** after moving `routes.*`, confirm the regenerated file differs from the moved original **only** by location (no banner path embed). If the banner embeds the path, expect a content delta and rebaseline the toy goldens via the emitter's `__main__`, then re-run the routes-freshness gate.
- **ADR number:** next unused at merge time (max file is `052`; `053` is claimed by the unmerged CTPS #125 branch, so likely `054`). Re-check `docs/decisions/` AND every open branch for a collision before finalizing; renumber the branch that merges second, never one already on main.
- **Rollback:** the move is a single commit of pure renames + path edits; `git revert` of that commit + the prep commits restores the flat layout cleanly. No data or behavioral state to unwind.
- **This branch is off `main`**, independent of the unmerged `feat/ctps-player-uiux-rework` work; if that branch also edits `ctps_mocks/` or the `ctps_dump` CMake target, expect a merge reconciliation there.
