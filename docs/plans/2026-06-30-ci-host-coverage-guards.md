# CI Guards: No-New-Mirror + Host-Coverage Manifest Plan

**Goal:** Add two cheap static CI guards so the host-test harness (epic #107) can't silently regress — one fails when a new `// MIRROR:` replica appears, the other fails when a production `WRO2026/*.cpp` ships with no host coverage.

**Approach:** One stdlib-only Python script (`WRO2026/test/check_host_coverage.py`) runs two pure text checks against the working tree and exits non-zero on any violation. Each check is gated by a committed, hand-maintained allowlist (mirroring ADR-050's self-auditing allowlist): the **mirror allowlist** pins which test files may hold `// MIRROR:` slices and how many; the **coverage allowlist** pins which production `.cpp` are allowed to be un-host-linked, each with a reason. The script is wired as a new step in the Python-capable `path-planner` CI job and its parsing logic is unit-tested by pytest in `tools/path_planner/tests/`. No firmware change; no de-mirroring or new host-linking — the guards only baseline and ratchet today's state.

## Scope

- **In scope:**
  - A stdlib-only Python guard script implementing both checks, exiting non-zero with clear per-violation messages.
  - Two committed allowlist data files (JSON): a mirror allowlist and a host-coverage allowlist, seeded to the current tree.
  - A new CI step in `.github/workflows/ci.yml` (`path-planner` job) that runs the guard.
  - pytest unit tests for the guard's parsing/comparison logic, in `tools/path_planner/tests/` (collected by the existing pytest job).
  - A "How to apply" note added to ADR-030 pointing at the guard + allowlists.
  - A comment on issue #66 flagging the guard's flat-layout/path assumptions as a sketch-reorg call-site.
- **Out of scope:**
  - De-mirroring `test_wall_stop_watchdog.cpp` or `test_tasks_dispatch_overflow.cpp` — the guard only baselines them; removal is separate work tracked under epic #107.
  - Host-linking any currently-unlinked `.cpp` (`EventTrace`, `Intersection`, `WROLineArray`) — the manifest only records/allowlists them.
  - Opening new GitHub tracking issues — entries reference existing issues (epic #107) where one fits; no new issues filed.
  - Any change to firmware sources or `test/CMakeLists.txt` target lists.
  - Invoking CMake or a compiler in the guard — both checks are pure static text parsing.

## Decisions

- **Mirror count semantics:** strict line-anchored regex `^\s*//+\s*MIRROR:` over `WRO2026/test/` `.cpp`/`.h` files. This counts the 7 real annotation slices and excludes the 6 prose/doc mentions that merely quote the pattern. — Chosen over a naive `// MIRROR:` substring (which would trip on doc edits) and over a single baseline integer (which loses per-file reasons/issue links).
- **Mirror baseline form:** an itemized per-file allowlist (`file → count → reason → issue`), not a single number — mirrors ADR-050's self-auditing allowlist. The observed per-file mirror map must **exactly equal** the committed allowlist (set + counts), so a rise, a drop, a new file, and a stale entry are all caught by one equality check.
- **Guard behavior:** **fail** the build on any violation, including a count **drop** below the allowlisted value. Drop-fail forces a dev to ratchet the allowlist down when they de-mirror — keeping it honest (the issue's "or warns" softer option was rejected; the coverage manifest already "fails" per the issue).
- **Coverage "linked" test:** a production `.cpp` counts as linked if its literal `${SKETCH_DIR}/<name>.cpp` path appears on any non-comment line of `test/CMakeLists.txt` — catches sources added via `add_executable` **or** `target_sources`/helper macros (robust against future CMake refactors). — Chosen over matching only inside `add_executable(...)` blocks (the issue's literal wording), which would miss `target_sources` additions.
- **Reverse-drift detection:** the coverage check also fails when an allowlisted file has since become linked, or no longer exists — keeps the allowlist from rotting (ADR-050 stale-entry rule).
- **Placement:** script + both JSON data files under `WRO2026/test/` (next to the CMakeLists they guard); unit tests under `tools/path_planner/tests/` (run for free by the existing pytest job).
- **CI home:** a new step in the `path-planner` job, which already has Python 3.12 on PATH. The guard is static (no build), so it does not need the `host-tests` build job; running it in the Python-ready job avoids adding a Python setup elsewhere.
- **Data format:** JSON (stdlib `json`), trivial to parse and unit-test.
- **Allowlists are hand-maintained**, never auto-regenerated — auto-deriving the baseline from the tree would always match and defeat the ratchet.
- **Tracking:** each allowlist entry carries a reason; the two mirror entries link epic #107; the `Intersection` coverage entry has no issue yet (field left null). No new issues filed.
- **Docs:** amend ADR-030 with a "How to apply" note rather than write a new ADR — the guards mechanically enforce ADR-030 (link real sources) + ADR-050 (self-auditing allowlist), so they extend an existing decision rather than introduce a new one.

## Acceptance Criteria

- [ ] AC1: `python WRO2026/test/check_host_coverage.py` exits 0 on the current tree (both guards green against reality — the seeded allowlists match the observed state).
- [ ] AC2: Adding a `// MIRROR:` annotation line to a test file not in the mirror allowlist makes the script exit non-zero with a message naming that file.
- [ ] AC3: Changing the mirror count in an allowlisted file (up or down) without updating the allowlist makes the script exit non-zero naming the file and the expected-vs-found counts. (AC covers both rise and the ratchet-down drop case.)
- [ ] AC4: The 6 prose/doc mentions of `// MIRROR:` (in `CMakeLists.txt`, `test_path_tracker_passthrough.cpp:11`, `test_path_tracker_stop_at_line.cpp:11`, `test_wall_stop_watchdog.cpp:10`) are **not** counted; the observed baseline is exactly 7 across 2 files.
- [ ] AC5: Adding a new `WRO2026/Foo.cpp` that is neither linked in `test/CMakeLists.txt` nor in the coverage allowlist makes the script exit non-zero naming `Foo.cpp`.
- [ ] AC6: A source linked only via `target_sources(...)` (not inside `add_executable(...)`) is detected as linked (no false "unlinked" failure).
- [ ] AC7: A stale coverage allowlist entry — a file that has since become host-linked, or that no longer exists in `WRO2026/` — makes the script exit non-zero.
- [ ] AC8: The new CI step in the `path-planner` job runs the guard and CI is green on this PR.
- [ ] AC9: ADR-030 carries a "How to apply" note pointing at the guard script + the two allowlists.
- [ ] AC10: A comment is posted on issue #66 listing the guard's flat-`WRO2026/*.cpp` glob + `${SKETCH_DIR}/X.cpp` parse as a sketch-reorg call-site to update.

## Data Model

Two committed JSON files under `WRO2026/test/`. Both use a top-level `{"entries": [...]}` envelope so the schema can grow without breaking the parser.

`WRO2026/test/mirror_allowlist.json` — one entry per test file permitted to hold `// MIRROR:` slices:

```json
{
  "entries": [
    {
      "file": "test_wall_stop_watchdog.cpp",
      "count": 6,
      "reason": "Drive wall-stop watchdog STATE wiring; pure primitives (stopAtLineProgressed/Stalled) shared via WROMath.h, not byte-copied",
      "issue": "#107"
    },
    {
      "file": "test_tasks_dispatch_overflow.cpp",
      "count": 1,
      "reason": "compile-time queue-overflow static_assert mirror of Mission.h:307-312",
      "issue": "#107"
    }
  ]
}
```

`WRO2026/test/host_coverage_allowlist.json` — one entry per production `.cpp` permitted to be un-host-linked:

```json
{
  "entries": [
    {
      "file": "EventTrace.cpp",
      "reason": "OLED/serial peripheral, mocked; no real EventTrace.cpp linked (test/CMakeLists.txt:299)",
      "issue": null
    },
    {
      "file": "Intersection.cpp",
      "reason": "mocked as an upper peripheral in PathTracker tests; no dedicated host test yet",
      "issue": null
    },
    {
      "file": "WROLineArray.cpp",
      "reason": "hardware I2C line-array leaf, replaced by a test-controlled mock",
      "issue": null
    }
  ]
}
```

Field semantics: `file` is a basename (no path). `count` (mirror only) is the exact expected number of strict-regex matches. `reason` is required, non-empty. `issue` is a string like `"#107"` or `null`.

## Contracts & Interfaces

Single source of truth for every seam more than one task touches.

### Shared types — the two JSON schemas

- **Mirror allowlist entry:** `{ "file": str(basename), "count": int>=1, "reason": str(non-empty), "issue": str|null }`. Owner: T2. Consumers: T1 (parses), T3 (fixtures assert against it).
- **Coverage allowlist entry:** `{ "file": str(basename), "reason": str(non-empty), "issue": str|null }`. Owner: T2. Consumers: T1 (parses), T3 (fixtures).
- Both files wrap entries in `{ "entries": [ ... ] }`.

### Signatures (guard module — `check_host_coverage.py`)

Functions are pure and parameterized on paths so pytest can drive them against temp fixtures, not just the real repo. Owner: T1; consumers: T3.

- `repo_root() -> Path` — resolves the repo toplevel from `__file__` (`Path(__file__).resolve().parents[2]`), so the script is cwd-independent.
- `count_mirrors(test_dir: Path) -> dict[str, int]` — returns `{basename: match_count}` for files under `test_dir` (recursive, `*.cpp`/`*.h`) with ≥1 match of `MIRROR_RE`. Files with 0 matches are omitted.
- `load_allowlist(path: Path) -> list[dict]` — reads a JSON allowlist file, returns `entries`.
- `check_mirrors(test_dir: Path, allowlist_path: Path) -> list[str]` — returns a list of violation messages (empty == pass). Fails when the observed `{file: count}` map does not exactly equal the allowlist `{file: count}` map.
- `linked_sources(cmakelists_path: Path) -> set[str]` — returns the set of `<name>.cpp` basenames matched by `LINK_RE` on non-comment text of the CMakeLists.
- `check_coverage(prod_dir: Path, cmakelists_path: Path, allowlist_path: Path) -> list[str]` — returns violation messages. Fails on un-linked-and-un-allowlisted production `.cpp`, and on stale allowlist entries (now-linked or non-existent).
- `main(argv=None) -> int` — runs both checks against `repo_root()`, prints results, returns `0` (clean) or `1` (any violation).

### Constants (owner: T1)

- `MIRROR_RE = re.compile(r"^\s*//+\s*MIRROR:")` — line-anchored; matches `//`/`///`-led annotation lines, excludes block-comment (` * ...`) and `#`-led and mid-line prose mentions.
- `LINK_RE = re.compile(r"\$\{SKETCH_DIR\}/([A-Za-z_][A-Za-z0-9_]*)\.cpp")` — captures the basename.
- Comment stripping for the coverage parser: for each CMakeLists line, drop everything from the first `#` to end-of-line before applying `LINK_RE` (handles whole-line and inline `#` comments).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/test/check_host_coverage.py` | T1 | T3 (imports), T4 (invokes) |
| `WRO2026/test/mirror_allowlist.json` | T2 | T1, T3 |
| `WRO2026/test/host_coverage_allowlist.json` | T2 | T1, T3 |
| `tools/path_planner/tests/test_host_coverage_guard.py` | T3 | — |
| `.github/workflows/ci.yml` | T4 | — |
| `docs/decisions/030-harness-links-real-sources-ci-enforced.md` | T5 | — |
| issue #66 (GitHub comment) | T6 | — |

### Naming

- Script path: `WRO2026/test/check_host_coverage.py` (exact — referenced by the CI step, the tests, and ADR-030).
- CI step name: `Host-coverage + mirror guards`.
- The coverage parser keys on the exact literal `${SKETCH_DIR}/` prefix as it appears in `test/CMakeLists.txt` (the var is `set(SKETCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)` at line 38).

## Error Handling

- **Allowlist file missing or malformed JSON:** the script prints an explicit error (`cannot read <path>: <reason>`) and exits non-zero — a broken guard must fail loud, never skip silently.
- **`test/CMakeLists.txt` missing:** same — exit non-zero with a clear message (the repo layout assumption is broken; this is also the #66-reorg tripwire).
- **Any check returns violations:** print each violation on its own line under a per-check header, print a one-line summary (`FAIL: N mirror, M coverage violations`), exit `1`.
- **Both checks clean:** print `OK: mirror baseline N across K files; P production .cpp linked, Q allowlisted`, exit `0`.
- **Both checks always run** (do not short-circuit after the first failing check), so a single CI run reports every violation at once.

## Testing Strategy

**Levels:** Unit (parsing/comparison logic against temp fixtures) + one live integration assertion against the real repo.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | Run both checks against the real repo tree | Integration | No violations; `main()` returns 0 (anti-tautology: seeded allowlists match reality) |
| TC2 | `count_mirrors` over the real `WRO2026/test/` | Unit | Returns exactly `{test_wall_stop_watchdog.cpp: 6, test_tasks_dispatch_overflow.cpp: 1}` (prose excluded) |
| TC3 | Mirror in a temp test file not in the allowlist | Unit | `check_mirrors` returns a violation naming the file |
| TC4 | Allowlisted file with count raised by 1 (temp fixture) | Unit | Violation: expected N, found N+1 |
| TC5 | Allowlisted file with count dropped (temp fixture) | Unit | Violation: stale entry, expected N, found M (ratchet-down enforced) |
| TC6 | Allowlist entry for a file with 0 mirrors / missing file | Unit | Violation: stale mirror allowlist entry |
| TC7 | Block-comment (` * \`// MIRROR:\``) and mid-line prose lines | Unit | `MIRROR_RE` does not match them (count unaffected) |
| TC8 | `linked_sources` over a temp CMakeLists with `add_executable` + `target_sources` refs | Unit | Both forms detected; basenames returned |
| TC9 | `${SKETCH_DIR}/X.cpp` appearing only inside a `#` comment / after inline `#` | Unit | Not counted as linked |
| TC10 | Production `.cpp` neither linked nor allowlisted (temp dirs) | Unit | `check_coverage` returns a violation naming the file |
| TC11 | Coverage allowlist entry for a file that is also linked | Unit | Violation: stale (now linked) |
| TC12 | Coverage allowlist entry for a non-existent file | Unit | Violation: stale (no longer exists) |
| TC13 | Missing/malformed allowlist JSON | Unit | Raises/returns a clear error; `main()` returns 1 |

**Test data:** `tmp_path` fixtures — synthesize a fake `prod_dir` with empty `.cpp` files, a fake `test_dir` with `.cpp` files containing controlled `// MIRROR:` lines, a fake `CMakeLists.txt` with controlled `${SKETCH_DIR}/X.cpp` refs, and small JSON allowlists. The guard module is loaded via `importlib.util.spec_from_file_location` from `WRO2026/test/check_host_coverage.py` (it is not an installed package). TC1/TC2 run against the real repo via `repo_root()`.

**Run command:** `uv run --extra dev pytest tools/path_planner/tests/test_host_coverage_guard.py -v` (and the script directly: `python WRO2026/test/check_host_coverage.py`).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Guard script | — | med | `WRO2026/test/check_host_coverage.py` | Implement the stdlib-only guard per Contracts & Interfaces: `repo_root`, `count_mirrors`, `load_allowlist`, `check_mirrors` (observed map must exactly equal allowlist map), `linked_sources` (strip `#` comments then `LINK_RE`), `check_coverage` (un-tracked + reverse-drift), `main` (run both, print, return 0/1). Use `MIRROR_RE`/`LINK_RE` constants. Both checks always run. Satisfies AC1-AC7. |
| T2 | Seed allowlists | — | low | `WRO2026/test/mirror_allowlist.json`, `WRO2026/test/host_coverage_allowlist.json` | Author the two JSON files exactly as in Data Model: mirror = `test_wall_stop_watchdog.cpp`(6,#107) + `test_tasks_dispatch_overflow.cpp`(1,#107); coverage = `EventTrace.cpp`, `Intersection.cpp`, `WROLineArray.cpp` with the stated reasons, `issue:null`. Conform to the Contracts schema. Satisfies AC4 (baseline=7) and the coverage seed. |
| T3 | Unit tests | T1, T2 | low | `tools/path_planner/tests/test_host_coverage_guard.py` | Implement TC1-TC13. Load the guard via `importlib` from its path. Use `tmp_path` fixtures for unit cases; TC1/TC2 assert against the real repo. Satisfies AC1-AC7 verification. |
| T4 | CI wiring | T1 | low | `.github/workflows/ci.yml` | Add a step named `Host-coverage + mirror guards` to the `path-planner` job (after the routes/fixture freshness steps): `run: python WRO2026/test/check_host_coverage.py`. Plain `python` (stdlib only; setup-python 3.12 is on PATH). Satisfies AC8. |
| T5 | Amend ADR-030 | — | low | `docs/decisions/030-harness-links-real-sources-ci-enforced.md` | Add a short "How to apply" bullet: the no-new-mirror + host-coverage guards (`WRO2026/test/check_host_coverage.py` + the two allowlists) mechanically enforce this ADR; baseline is itemized (ADR-050 style) and ratchets down as mirrors are removed. Do not renumber or change Status. Satisfies AC9. |
| T6 | Comment on #66 | — | low | issue #66 (GitHub) | At execute time, `gh issue comment 66` noting the guard hardcodes the flat `WRO2026/*.cpp` glob + `${SKETCH_DIR}/<name>.cpp` parse, so the sketch-tree reorg must update `check_host_coverage.py` (alongside the CMakeLists / header_writer.py / CI items already listed). Outward-facing write — performed once, after spec approval. Satisfies AC10. |

T1 and T2 run in parallel (different files; both conform to the Contracts schema). T3 waits for both. T4 and T5 wait only on T1 / nothing respectively. T6 is independent.

## Notes for Implementer

- **Anti-tautology is the point:** TC1/TC2 must run against the *real* repo so the seeded allowlists are proven to match the actual tree. If `count_mirrors` returns anything other than `{wall_stop:6, tasks_overflow:1}`, the regex or the seed is wrong — fix before merging.
- **`MIRROR_RE` is line-anchored on purpose.** Verify against the known cases: it must match `test_wall_stop_watchdog.cpp:85` (`/// MIRROR:`) and `:99/106/120/141/153` and `test_tasks_dispatch_overflow.cpp:39`, and must NOT match `test_wall_stop_watchdog.cpp:10` (`//   annotate ... \`// MIRROR:`), the ` * \`// MIRROR:\`` block-comment lines at `passthrough.cpp:11` / `stop_at_line.cpp:11`, or the `#`-led CMakeLists prose at lines 1153/1195/1446.
- **Scope the mirror sweep to `WRO2026/test/`**, not all of `WRO2026/` — production sources don't carry mirrors, and scanning the whole tree risks counting future doc prose elsewhere.
- **Comment stripping for coverage:** strip from the first `#` to EOL per CMakeLists line before `LINK_RE`. Line 80 (`# \`${SKETCH_DIR}/routes.cpp\`. ...`) and similar must not count as a link.
- **Production set = `WRO2026/*.cpp`** (non-recursive glob; the modules are flat today). Today that is 14 files; 11 must be linked, 3 allowlisted. `routes.cpp` is generated but IS linked — it stays in the watched set.
- **stdlib only** — `json`, `re`, `pathlib`, `sys`, `glob`/`Path.glob`. No third-party imports, so the CI step needs no `uv`/venv.
- **`repo_root()` via `__file__`**, not cwd — the CI step runs from repo root, but the pytest import resolves the module from `WRO2026/test/`; both must find the same files.
- **Don't auto-regenerate the allowlists.** They are deliberate ceilings; a regenerate-and-diff step (the routes-freshness pattern) would always match and defeat the ratchet.
- **#66 coupling:** the guard assumes the flat sketch layout. When #66 moves modules into `WRO2026/src/<domain>/`, both the production glob and the `${SKETCH_DIR}/X.cpp` form change — that's why T6 posts the heads-up comment.
- **Rollback:** the change is additive (new files + one CI step + one ADR bullet). Reverting the commit removes the guard with no firmware impact.
