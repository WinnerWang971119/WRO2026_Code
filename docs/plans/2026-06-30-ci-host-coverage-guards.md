# CI Guards: No-New-Mirror + Host-Coverage Manifest Plan

**Goal:** Add two cheap static CI guards so the host-test harness (epic #107) can't silently regress — one fails when a new `MIRROR:` replica annotation appears, the other fails when a production `WRO2026/*.cpp` ships with no host coverage.

**Approach:** One stdlib-only Python script (`WRO2026/test/check_host_coverage.py`) runs two pure text checks against the working tree and exits non-zero on any violation. Each check is gated by a committed, hand-maintained allowlist (modeled on ADR-050's self-auditing allowlist, with one deliberate relaxation noted in Decisions): the **mirror allowlist** pins which test files may hold `MIRROR:` slices and how many; the **coverage allowlist** pins which production `.cpp` are allowed to be un-host-linked, each with a reason. The mirror scan recognizes a genuine annotation in any comment style (leading or mid-line, line- or block-comment) as long as it cites a concrete reference; the coverage scan recognizes linkage only inside source-bearing CMake command bodies. The script is wired as a new step in the Python-capable `path-planner` CI job, and its parsing logic is unit-tested by pytest in `tools/path_planner/tests/`. No firmware change; no de-mirroring or new host-linking — the guards only baseline and ratchet today's state.

## Scope

- **In scope:**
  - A stdlib-only Python guard script implementing both checks, exiting non-zero with clear per-violation messages.
  - Two committed allowlist data files (JSON): a mirror allowlist and a host-coverage allowlist, seeded to the current tree.
  - A new CI step in `.github/workflows/ci.yml` (`path-planner` job) that runs the guard.
  - pytest unit tests for the guard's parsing/comparison logic, in `tools/path_planner/tests/` (collected by the existing pytest job).
  - A "How to apply" note added to ADR-030 pointing at the guard + allowlists (and a one-line correction of ADR-030's stale "sole remaining inline mirror" claim).
  - A comment on issue #66 flagging the guard's flat-layout/path assumptions as a sketch-reorg call-site.
- **Out of scope:**
  - De-mirroring `test_wall_stop_watchdog.cpp` or `test_tasks_dispatch_overflow.cpp` — the guard only baselines them; removal is separate work tracked under epic #107.
  - Host-linking any currently-unlinked `.cpp` (`EventTrace`, `Intersection`, `WROLineArray`) — the manifest only records/allowlists them.
  - Opening new GitHub tracking issues — entries reference existing issues (epic #107) where one fits; no new issues filed.
  - Editing the test files' bodies or comments (including the now-stale "Drive.cpp is NOT host-compiled" rationale docblock in `test_wall_stop_watchdog.cpp`), firmware sources, or `test/CMakeLists.txt` target lists.
  - Invoking CMake or a compiler in the guard — both checks are pure static text parsing (a deliberate user pin; see the residual limitation in Notes).

## Decisions

- **Mirror count semantics:** count a line as a mirror annotation when a comment marker (`//`/`///` line comment, or a `*`/`/*` block-comment line) carries the `MIRROR:` tag followed — anywhere on the line — by a **concrete reference**: a `.cpp`/`.h`:digit citation or a `Class::member`. This is position-independent, so leading **and** mid-line annotations count; the concrete-ref requirement excludes the convention-doc prose (which uses a `<line>` placeholder or backtick-quoted examples with no real citation). — Chosen over the original line-anchored `^\s*//+\s*MIRROR:` (which undercounted the mid-line annotations at `test_wall_stop_watchdog.cpp:112/132/148` and was trivially evadable) and over normalizing the test file's comments (which would leave the mid-line evasion open and edit a test out of scope).
- **True baseline = 10 real annotations:** 9 in `test_wall_stop_watchdog.cpp` (lines 85, 99, 106, 112, 120, 132, 141, 148, 153) + 1 in `test_tasks_dispatch_overflow.cpp` (line 39). Exactly **3** prose mentions are excluded: `test_wall_stop_watchdog.cpp:10` (`Drive.cpp:<line>` placeholder, no digit), `test_path_tracker_passthrough.cpp:11`, and `test_path_tracker_stop_at_line.cpp:11` (both ` *` block-comment, backtick-quoted, no concrete citation). (The 3 `MIRROR:` mentions in `test/CMakeLists.txt` are excluded by file type — the sweep only reads `.cpp`/`.h`.)
- **Ref-less-mirror hardening:** a `//`/`*` comment line that carries the `MIRROR:` tag but has **no** concrete reference and does **not** match a known-prose signature (a backtick-quoted comment marker `` `//`` immediately before `MIRROR:`, or a `<...>` placeholder after it) **fails** with "mirror annotation missing concrete reference — cite file:line". This closes the evasion where a new replica is tagged `// MIRROR: copied logic` without a citation, and enforces the documented convention that every mirror cites file:line.
- **Mirror baseline form:** an itemized per-file allowlist (`file → count → reason → issue`), not a single number — modeled on ADR-050's self-auditing allowlist. The observed per-file mirror map must **exactly equal** the committed allowlist (set + counts), so a rise, a drop, a new file, and a stale entry are all caught by one equality check.
- **Guard behavior:** **fail** the build on any violation, including a count **drop** below the allowlisted value. Drop-fail forces a dev to ratchet the allowlist down when they de-mirror — keeping it honest. (The issue's softer "or warns" option was rejected; the coverage manifest already "fails" per the issue.)
- **Coverage "linked" test:** a production `.cpp` counts as linked when its literal `${SKETCH_DIR}/<name>.cpp` path appears (on non-comment text) inside the **argument body of a source-bearing CMake command** — `add_executable(...)`, `target_sources(...)`, or `set(...)`. This catches sources listed in `add_executable`, in `target_sources`, and in the `set(VAR ...)` source variables the repo expands into targets (e.g. the COMMON sources `set()` at `test/CMakeLists.txt:93`), while excluding accidental mentions in `message()`, helper calls, bare lines, or comments. — Chosen over "any non-comment line" (which would mistake a `message()`/dead mention for linkage, per the Codex critic) and over the CMake File API (which would require running `cmake` in the guard, violating the pure-text pin). **Residual limitation, accepted:** pure text cannot distinguish a live source list from a dead one — a source inside an unused `set()` variable or an `if(FALSE)` block would still count as linked. Closing those needs partial CMake evaluation (out of scope); the primary guarded case — a new `.cpp` with zero mention in CMakeLists — is fully covered.
- **Reverse-drift detection:** the coverage check also fails when an allowlisted file has since become linked, or no longer exists — keeps the allowlist from rotting (ADR-050 stale-entry rule).
- **Keys are paths, not bare basenames:** the mirror map keys observations and allowlist entries by the file's **POSIX path relative to `WRO2026/test/`** (so two identically-named files in different subdirs cannot collide). The coverage map keys by basename because production sources are flat today and CMake references them by basename; nesting (via #66) is the migration trigger (see Notes + T6).
- **Strict allowlist schema validation:** `load_allowlist` validates each file before comparison — rejects a malformed envelope, duplicate `file` entries, missing/empty `reason`, a non-int or `< 1` `count`, and an `issue` that is neither a string nor `null`. A schema violation is a guard failure, not a silent skip.
- **Checks run independently:** the two checks never short-circuit each other. A malformed/missing allowlist (or missing `CMakeLists.txt`) is converted to a violation for **that** check while the other check still runs, so one CI run reports every problem at once.
- **Placement:** script + both JSON data files under `WRO2026/test/` (next to the CMakeLists they guard); unit tests under `tools/path_planner/tests/` (run for free by the existing pytest job).
- **CI home:** a new step in the `path-planner` job, which already has Python 3.12 on PATH. The guard is static (no build), so it does not need the `host-tests` build job.
- **Data format:** JSON (stdlib `json`), trivial to parse and unit-test.
- **Allowlists are hand-maintained**, never auto-regenerated — auto-deriving the baseline from the tree would always match and defeat the ratchet.
- **Tracking convention (a deliberate relaxation of ADR-050):** each allowlist entry requires a `reason`; a tracking `issue` is **optional** (`null` allowed). ADR-050 requires both reason *and* a tracking issue; this guard weakens that to reason-required because the user chose to link an existing issue only where one fits (epic #107 for the two mirrors) and `Intersection.cpp` has no dedicated issue yet. This relaxation is documented in the ADR-030 note (T5) so it is a stated convention, not an accident.
- **Docs:** amend ADR-030 with a "How to apply" note rather than write a new ADR — the guards mechanically enforce ADR-030 (link real sources) + ADR-050 (self-auditing allowlist).

## Acceptance Criteria

- [ ] AC1: `python WRO2026/test/check_host_coverage.py` exits 0 on the current tree — both guards green against reality (the seeded allowlists match the observed state: mirror baseline **10** across 2 files; 11 production `.cpp` linked, 3 allowlisted).
- [ ] AC2: The mirror sweep counts every genuine annotation regardless of comment style — a unit test proves (a) a mid-line `// ...prose... MIRROR: Foo.cpp:12` IS counted, (b) a block-comment ` * MIRROR: Foo.h:5` IS counted, and (c) the live tree yields exactly `{test_wall_stop_watchdog.cpp: 9, test_tasks_dispatch_overflow.cpp: 1}`.
- [ ] AC3: A prose mention with no concrete citation — `// MIRROR: Foo.cpp:<line>` (placeholder), a backtick-quoted `` `// MIRROR:` ``, or a ` *`-block reference with no `file:line` — is NOT counted. (The 3 real prose lines at `wall_stop:10`, `passthrough:11`, `stop_at_line:11` are excluded; the 3 `CMakeLists.txt` mentions are excluded by file type.)
- [ ] AC4: A `MIRROR:` tag in a `//`/`*` comment that has no concrete reference and is not a known-prose signature makes the script exit non-zero with "missing concrete reference".
- [ ] AC5: Adding a `MIRROR:` annotation (any comment style, with a citation) to a test file not in the mirror allowlist makes the script exit non-zero naming that file.
- [ ] AC6: Changing the mirror count in an allowlisted file (up or down) without updating the allowlist makes the script exit non-zero naming the file and the expected-vs-found counts (covers both rise and the ratchet-down drop case, and a stale entry whose file now has 0 / no longer exists).
- [ ] AC7: Adding a new `WRO2026/Foo.cpp` that is neither linked in a source-bearing CMake command nor in the coverage allowlist makes the script exit non-zero naming `Foo.cpp`.
- [ ] AC8: A source linked via `target_sources(...)` or a multi-line `set(VAR ...)` body is detected as linked; a `${SKETCH_DIR}/Foo.cpp` appearing only in a `message(...)`, a non-source command, a bare line, or a comment is NOT counted as linked.
- [ ] AC9: A stale coverage allowlist entry — a file that has since become host-linked, or that no longer exists in `WRO2026/` — makes the script exit non-zero.
- [ ] AC10: A malformed/duplicate allowlist (bad envelope, duplicate `file`, `count < 1`, empty `reason`, bad `issue` type) makes the script exit non-zero; and a malformed allowlist for one check does not suppress the other check's diagnostics (both reported in one run).
- [ ] AC11: The new CI step in the `path-planner` job runs the guard and CI is green on this PR (depends on the script + both allowlists + the tests all landing).
- [ ] AC12: ADR-030 carries a "How to apply" note pointing at the guard script + the two allowlists, **explicitly states the drop-fail/ratchet rule** (de-mirroring requires editing the allowlist down or CI breaks) and the reason-required/issue-optional relaxation, and corrects the stale "the sole remaining inline mirror is `test_wall_stop_watchdog`" line (the `test_tasks_dispatch_overflow` static-assert mirror also remains).
- [ ] AC13: A comment is posted on issue #66 listing the guard's flat-`WRO2026/*.cpp` glob + `${SKETCH_DIR}/X.cpp` parse as a sketch-reorg call-site to update.

## Data Model

Two committed JSON files under `WRO2026/test/`. Both use a top-level `{"entries": [...]}` envelope so the schema can grow without breaking the parser.

`WRO2026/test/mirror_allowlist.json` — one entry per test file permitted to hold `MIRROR:` slices (`file` is the path relative to `WRO2026/test/`):

```json
{
  "entries": [
    {
      "file": "test_wall_stop_watchdog.cpp",
      "count": 9,
      "reason": "Drive wall-stop watchdog STATE wiring; pure primitives (stopAtLineProgressed/Stalled) shared via WROMath.h, not byte-copied. De-mirror tracked under epic #107.",
      "issue": "#107"
    },
    {
      "file": "test_tasks_dispatch_overflow.cpp",
      "count": 1,
      "reason": "compile-time queue-overflow static_assert mirror of Mission.h:307-312. De-mirror tracked under epic #107.",
      "issue": "#107"
    }
  ]
}
```

`WRO2026/test/host_coverage_allowlist.json` — one entry per production `.cpp` permitted to be un-host-linked (`file` is a basename):

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

Field semantics: mirror `file` = POSIX path relative to `WRO2026/test/`; coverage `file` = basename (no path). `count` (mirror only) is the exact expected number of annotation matches, an int `>= 1`. `reason` is required, non-empty. `issue` is a string like `"#107"` or `null` (optional — a deliberate relaxation of ADR-050; see Decisions).

## Contracts & Interfaces

Single source of truth for every seam more than one task touches.

### Shared types — the two JSON schemas

- **Mirror allowlist entry:** `{ "file": str(test-relative POSIX path), "count": int>=1, "reason": str(non-empty), "issue": str|null }`. Owner: T2. Consumers: T1 (parses + validates), T3 (fixtures).
- **Coverage allowlist entry:** `{ "file": str(basename), "reason": str(non-empty), "issue": str|null }`. Owner: T2. Consumers: T1, T3.
- Both files wrap entries in `{ "entries": [ ... ] }`. Duplicate `file` values within a file are rejected.

### Signatures (guard module — `check_host_coverage.py`)

Functions are pure and parameterized on paths so pytest can drive them against temp fixtures, not just the real repo. Owner: T1; consumers: T3.

- `repo_root() -> Path` — resolves the repo toplevel from `__file__` (`Path(__file__).resolve().parents[2]`), so the script is cwd-independent.
- `count_mirrors(test_dir: Path) -> dict[str, int]` — recursively scans `*.cpp`/`*.h` under `test_dir` **line by line**; returns `{test_relative_posix_path: match_count}` for files with ≥1 match of `MIRROR_RE`. Files with 0 matches are omitted.
- `find_refless_mirrors(test_dir: Path) -> list[str]` — returns violation messages for comment lines carrying a `MIRROR:` tag that neither match `MIRROR_RE` (no concrete ref) nor `PROSE_RE` (known-prose signature). Empty == none.
- `load_allowlist(path: Path, *, with_count: bool) -> list[dict]` — reads + schema-validates a JSON allowlist (envelope, required fields, types, `count>=1` when `with_count`, no duplicate `file`); raises `AllowlistError` on any violation.
- `check_mirrors(test_dir: Path, allowlist_path: Path) -> list[str]` — returns violation messages (empty == pass). Fails when the observed `{file: count}` map does not exactly equal the allowlist map, OR `find_refless_mirrors` reports any, OR the allowlist fails to load/validate.
- `linked_sources(cmakelists_path: Path) -> set[str]` — returns the set of `<name>.cpp` basenames captured by `LINK_RE` within the argument bodies of source-bearing commands (`SOURCE_COMMANDS`), after comment-stripping. See `_command_bodies`.
- `_command_bodies(text: str, names: tuple[str, ...]) -> str` — returns the concatenated argument bodies of every top-level CMake command whose (case-insensitive) name is in `names`, by scanning for an identifier immediately followed by `(` and reading to the matching `)` with paren-depth tracking (so multi-line `set()`/`target_sources()` bodies are captured whole). Pure text; no cmake invocation.
- `check_coverage(prod_dir: Path, cmakelists_path: Path, allowlist_path: Path) -> list[str]` — returns violation messages. Fails on an un-linked-and-un-allowlisted production `.cpp`, on stale allowlist entries (now-linked or non-existent), or if the allowlist/CMakeLists fail to load.
- `main(argv=None) -> int` — runs **both** checks against `repo_root()` independently (a raised input error in one is caught and turned into that check's violation, the other still runs), prints all results, returns `0` (clean) or `1` (any violation).

### Constants (owner: T1)

- `MIRROR_RE = re.compile(r"(?://+|\*).*?\bMIRROR:.*?(?:\.(?:cpp|h):\d|\w::\w)")` — applied **per line**. Matches a `//`/`///` or `*`/`/*` comment line whose `MIRROR:` tag is followed by a concrete `.cpp/.h:<digit>` citation or a `Class::member`. Verified to match the 10 real annotations and exclude the 3 prose lines.
- `PROSE_RE = re.compile(r"`\s*//+\s*MIRROR:|MIRROR:[^\n]*<[^>]+>")` — the known-prose signature: a backtick-quoted comment marker immediately before `MIRROR:`, or a `<...>` placeholder after it. Used by `find_refless_mirrors` to whitelist documentation, so only genuinely ref-less *non-prose* `MIRROR:` lines fail.
- `LINK_RE = re.compile(r"\$\{SKETCH_DIR\}/([A-Za-z_][A-Za-z0-9_]*)\.cpp")` — captures the basename.
- `SOURCE_COMMANDS = ("add_executable", "target_sources", "set")` — the CMake commands whose argument bodies may legitimately list `${SKETCH_DIR}/*.cpp`.
- Comment stripping for the coverage parser: for each CMakeLists line, drop everything from the first `#` to end-of-line before `_command_bodies`/`LINK_RE` (handles whole-line and inline `#` comments).

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

- **Allowlist file missing or malformed JSON / schema-invalid:** `load_allowlist` raises `AllowlistError`; the owning check (`check_mirrors` / `check_coverage`) catches it, emits an explicit violation (`cannot use <path>: <reason>`), and returns. The **other** check still runs — `main` never aborts on one check's input error.
- **`test/CMakeLists.txt` missing:** `check_coverage` emits a clear violation (the repo layout assumption is broken; this is also the #66-reorg tripwire). The mirror check still runs.
- **Any check returns violations:** print each violation on its own line under a per-check header, print a one-line summary (`FAIL: N mirror, M coverage violations`), exit `1`.
- **Both checks clean:** print `OK: mirror baseline N across K files; P production .cpp linked, Q allowlisted`, exit `0`.
- **Both checks always run** — `main` invokes each in its own try/except so a single CI run reports every violation at once.

## Testing Strategy

**Levels:** Unit (parsing/comparison logic against temp fixtures) + a live integration assertion against the real repo.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | Run both checks against the real repo tree | Integration | No violations; `main()` returns 0 (anti-tautology: seeded allowlists match reality) |
| TC2 | `count_mirrors` over the real `WRO2026/test/` | Unit | Returns exactly `{test_wall_stop_watchdog.cpp: 9, test_tasks_dispatch_overflow.cpp: 1}` |
| TC3 | Mid-line `// ...prose... MIRROR: Foo.cpp:12` (temp file) | Unit | Counted (proves position-independence; the regression for the original bug) |
| TC4 | Block-comment ` * MIRROR: Foo.h:5` (temp file) | Unit | Counted |
| TC5 | Prose lines: `// MIRROR: Foo.cpp:<line>`, `` `// MIRROR:` ``, ` * \`// MIRROR:\` of \`X.cpp\`` | Unit | NOT counted (no concrete citation) |
| TC6 | `MIRROR:` in a `//` comment with no concrete ref and not known-prose (e.g. `// MIRROR: copied logic`) | Unit | `find_refless_mirrors` returns a "missing concrete reference" violation |
| TC7 | Mirror (cited) in a temp file not in the allowlist | Unit | `check_mirrors` returns a violation naming the file |
| TC8 | Allowlisted file with count raised by 1 / dropped (temp fixtures) | Unit | Violation: expected N, found M (both directions) |
| TC9 | Allowlist entry for a file with 0 mirrors / missing file | Unit | Violation: stale mirror allowlist entry |
| TC10 | `linked_sources`: `add_executable`, `target_sources`, and multi-line `set(VAR ...)` bodies each list a source | Unit | All three detected; basenames returned |
| TC11 | `${SKETCH_DIR}/Foo.cpp` only in `message(...)` / a bare line / a non-source command / a `#` comment | Unit | NOT counted as linked |
| TC12 | Production `.cpp` neither linked nor allowlisted (temp dirs) | Unit | `check_coverage` returns a violation naming the file |
| TC13 | Coverage allowlist entry for a file that is also linked | Unit | Violation: stale (now linked) |
| TC14 | Coverage allowlist entry for a non-existent file | Unit | Violation: stale (no longer exists) |
| TC15 | Schema-invalid allowlists: bad envelope, duplicate `file`, `count` 0/boolean, empty `reason`, `issue` wrong type | Unit | `load_allowlist` raises `AllowlistError` for each |
| TC16 | Malformed mirror JSON + a coverage violation present (and the mirror malformed-coverage-clean inverse) | Unit | `main` reports BOTH the malformed-input violation and the other check's diagnostics; returns 1 |

**Test data:** `tmp_path` fixtures — synthesize a fake `prod_dir` with empty `.cpp` files, a fake `test_dir` with `.cpp`/`.h` files containing controlled `MIRROR:` lines, a fake `CMakeLists.txt` with controlled command bodies, and small JSON allowlists. The guard module is loaded via `importlib.util.spec_from_file_location` from `WRO2026/test/check_host_coverage.py` (it is not an installed package). TC1/TC2 run against the real repo via `repo_root()`.

**Run command:** `uv run --extra dev pytest tools/path_planner/tests/test_host_coverage_guard.py -v` (and the script directly: `python WRO2026/test/check_host_coverage.py`).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Guard script | — | med | `WRO2026/test/check_host_coverage.py` | Implement the stdlib-only guard per Contracts & Interfaces: `repo_root`; `count_mirrors` (line-by-line `MIRROR_RE`, keyed by test-relative POSIX path); `find_refless_mirrors` (`MIRROR:` tag minus `MIRROR_RE` minus `PROSE_RE` → fail); `load_allowlist` (strict schema: envelope, fields, types, `count>=1`, no duplicates → `AllowlistError`); `check_mirrors` (observed map must exactly equal allowlist map + ref-less hardening); `_command_bodies` (paren-depth scan of `SOURCE_COMMANDS` bodies); `linked_sources` (comment-strip → `_command_bodies` → `LINK_RE`); `check_coverage` (un-tracked + reverse-drift); `main` (run both independently in try/except, print, return 0/1). Define `MIRROR_RE`/`PROSE_RE`/`LINK_RE`/`SOURCE_COMMANDS`. Satisfies AC1-AC10. |
| T2 | Seed allowlists | — | low | `WRO2026/test/mirror_allowlist.json`, `WRO2026/test/host_coverage_allowlist.json` | Author the two JSON files exactly as in Data Model: mirror = `test_wall_stop_watchdog.cpp`(count 9, #107) + `test_tasks_dispatch_overflow.cpp`(count 1, #107); coverage = `EventTrace.cpp`, `Intersection.cpp`, `WROLineArray.cpp` with the stated reasons, `issue:null`. Conform to the Contracts schema. Satisfies AC1 (baseline=10) + the coverage seed. |
| T3 | Unit tests | T1, T2 | low | `tools/path_planner/tests/test_host_coverage_guard.py` | Implement TC1-TC16. Load the guard via `importlib` from its path. Use `tmp_path` fixtures for unit cases; TC1/TC2 assert against the real repo. Satisfies AC1-AC10 verification. |
| T4 | CI wiring | T1, T2 | low | `.github/workflows/ci.yml` | Add a step named `Host-coverage + mirror guards` to the `path-planner` job (after the routes/fixture freshness steps): `run: python WRO2026/test/check_host_coverage.py`. Plain `python` (stdlib only; setup-python 3.12 is on PATH). Needs the allowlists (T2) present or the step fails. Satisfies AC11 (full green also depends on T3 landing in the same PR). |
| T5 | Amend ADR-030 | — | low | `docs/decisions/030-harness-links-real-sources-ci-enforced.md` | Add a "How to apply" bullet: the no-new-mirror + host-coverage guards (`WRO2026/test/check_host_coverage.py` + the two allowlists) mechanically enforce this ADR; the baseline is itemized (ADR-050 style). **Explicitly state the ratchet rule** (de-mirroring REQUIRES editing the allowlist count down, or CI fails on the drop) and the reason-required/issue-optional relaxation of ADR-050. **Correct the stale claim** that `test_wall_stop_watchdog` is "the sole remaining inline mirror" — `test_tasks_dispatch_overflow` also carries a (compile-time static_assert) mirror; both are now allowlisted. Do not renumber or change Status. Satisfies AC12. |
| T6 | Comment on #66 | — | low | issue #66 (GitHub) | At execute time, `gh issue comment 66` noting the guard hardcodes the flat `WRO2026/*.cpp` glob + `${SKETCH_DIR}/<name>.cpp` parse, so the sketch-tree reorg must update `check_host_coverage.py` (alongside the CMakeLists / header_writer.py / CI items already listed). Outward-facing write — performed once, after spec approval. Satisfies AC13. |

T1 and T2 run in parallel (different files; both conform to the Contracts schema). T3 waits for both. T4 waits for T1 + T2. T5 and T6 are independent.

## Notes for Implementer

- **Anti-tautology is the point:** TC1/TC2 must run against the *real* repo so the seeded allowlists are proven to match the actual tree. If `count_mirrors` returns anything other than `{test_wall_stop_watchdog.cpp: 9, test_tasks_dispatch_overflow.cpp: 1}`, the regex or the seed is wrong — fix before merging.
- **`MIRROR_RE` is position-independent and line-scoped.** Verify against the known cases: it must match `test_wall_stop_watchdog.cpp:85` (`/// MIRROR: Drive::...`), `:99/106/120/141/153` (leading `// MIRROR: Drive.cpp:NN`), **`:112/132/148` (mid-line `... MIRROR: Drive.cpp:NN`)**, and `test_tasks_dispatch_overflow.cpp:39`. It must NOT match `test_wall_stop_watchdog.cpp:10` (`Drive.cpp:<line>` placeholder), `test_path_tracker_passthrough.cpp:11` / `test_path_tracker_stop_at_line.cpp:11` (` *` block-comment, backtick-quoted, no `file:line`). Apply it line-by-line (iterate `splitlines()`), so `.` never crosses newlines — do not run it over the whole file.
- **The ref-less hardening is what closes the evasion the broadened regex would otherwise open:** because `MIRROR_RE` requires a concrete ref, a new replica tagged `// MIRROR: copied logic` (no citation) would slip past the count — `find_refless_mirrors` catches it. `PROSE_RE` keeps the 3 documentation lines from tripping it.
- **Coverage parser is scoped to source-bearing command bodies** (`add_executable`/`target_sources`/`set`), not "any non-comment line" — so a `${SKETCH_DIR}/X.cpp` in a `message()`/comment/dead line does not count as linkage. **Residual limitation, accepted (user pin: no cmake):** it cannot tell a live source list from a dead one — a source inside an unused `set()` variable or an `if(FALSE)` block still counts as linked. The primary guarded case (a new `.cpp` with zero mention) is fully covered.
- **Comment stripping first:** strip from the first `#` to EOL per CMakeLists line before `_command_bodies`. Line 80 (`# \`${SKETCH_DIR}/routes.cpp\`. ...`) and similar must not count.
- **Production set = `WRO2026/*.cpp`** (non-recursive glob; modules are flat today — 14 files; 11 linked, 3 allowlisted). `routes.cpp` is generated but IS linked — it stays in the watched set. Nesting via #66 (`WRO2026/src/<domain>/`) breaks both the glob and the `${SKETCH_DIR}/X.cpp` form; that's why T6 posts the heads-up comment. Do NOT make the glob recursive now — `WRO2026/**/*.cpp` would sweep the tests and mocks too.
- **stdlib only** — `json`, `re`, `pathlib`, `sys`. No third-party imports, so the CI step needs no `uv`/venv.
- **`repo_root()` via `__file__`**, not cwd — the CI step runs from repo root, but the pytest import resolves the module from `WRO2026/test/`; both must find the same files.
- **Don't auto-regenerate the allowlists.** They are deliberate ceilings; a regenerate-and-diff step (the routes-freshness pattern) would always match and defeat the ratchet.
- **Out-of-scope but noted:** `test_wall_stop_watchdog.cpp`'s "WHY AN INLINE MIRROR (not the real Drive.cpp) — Drive.cpp is NOT host-compiled by ANY target" docblock is now stale (Drive.cpp IS host-linked since #106). Do not fix it here; it belongs with the de-mirror work under #107.
- **Rollback:** the change is additive (new files + one CI step + one ADR bullet). Reverting the commit removes the guard with no firmware impact.
