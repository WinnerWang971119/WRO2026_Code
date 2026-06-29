# PR #116 Parity-Net Hardening Plan

**Goal:** Close four verified gaps from the PR #116 review (`feat/pathtracker-sim-parity`) where the new sim↔firmware parity safety net can silently mask a real divergence — the exact failure mode issue #108 exists to catch — plus the related allowlist-correctness and trace-fidelity defects.

**Approach:** Four independent, mostly mechanical fixes. (3) Make the C++ parity trace emit the value actually dispatched to `Drive`, not a separately-recomputed twin. (2) Make the divergence allowlist consult its `side` field and use a platform-stable case-sensitive glob. (4) Collapse the duplicated wall-reset axle geometry into one shared helper. (1) Seed the Python sim with the C++-realized (encoder-quantized) operator offset so `return_move.dist_cm` is actually compared instead of allowlisted away; the now-redundant allowlist entry is deleted and the existing stale-entry guard enforces that it stays gone. All approach forks were chosen by the user during triage; findings 5/6/7 are explicitly out of scope.

## Scope

- **In scope:**
  - **Finding 3** — `WRO2026/PathTracker.cpp`: the two `PT_PARITY_TRACE` emit hooks (BLIND_DEAD_RECKON, PROJECTION_FOLLOW) record `effectiveDistCm` (the dispatched value) instead of `decision.commandedDistanceCm`. Trace-only; robot behavior byte-for-byte unchanged.
  - **Finding 2** — `tools/path_planner/sim/parity.py`: `Allowlist.match` consults `entry.side`; switch `fnmatch` → `fnmatchcase`; caller `_diff_records` computes and passes the diverging side.
  - **Finding 4** — extract the wall-reset axle pull-back math into one helper in `runtime.py`, called by both `parity.py::_apply_wall` and `runtime.py::_dispatch_wall_reset`.
  - **Finding 1** — emit the realized encoder-quantized operator-offset magnitude from the C++ dump's `goal_start` record; bump `PARITY_SCHEMA_VERSION` 1→2; surface it through `load_cpp_ndjson`; seed the sim with it in `_run_sim_case`; delete the now-stale `return-move-offset-encoder-quantization` allowlist entry.
  - Build the parity dump binary locally and run the parity suite against it; run the full Python suite, the host ctest, and the routes_fixture freshness gate.
- **Out of scope (deferred, NOT in this PR):**
  - **Finding 5** — `lru_cache` on `parse_config_float` / `parse_track_cm_from_config` (perf only).
  - **Finding 6** — hoist a shared dump-runner util between `test_pathtracker_parity.py` and `ctps/__main__.py` (DRY only).
  - **Finding 7** — return copies of the module-level containers from `build_field_data` (latent, no live mutator).
  - Any firmware behavior change. Finding 3 is deliberately trace-only.

## Decisions

- **Finding 3 → emit the driven value** (user choice): change the two `_ptEmit*` calls to pass `effectiveDistCm`; do NOT re-route what `Drive` is fed. Lowest behavior-regression risk.
- **Finding 1 → seed sim from the realized offset** (user choice): align the operator-offset INPUT on both sides (legitimate — it is an input, not an FSM decision), then compare `return_move.dist_cm` honestly. Preferred over bounding the allowlist.
- **Bump `PARITY_SCHEMA_VERSION` 1→2** (user choice): adding a required `goal_start` field is exactly the emitter/loader desync the version guards; a stale local dump binary then fails with a clean `schema_version 1 != 2` rather than a cryptic field-mismatch. Change the C++ `-DPARITY_SCHEMA_VERSION` and the Python `PARITY_SCHEMA_VERSION` in lockstep.
- **`realized_offset_cm` on EVERY `goal_start`**: the loader validates the `goal_start` field set for exact equality (`_record_from_obj`, parity.py:404-413), so the field cannot be optional. Clean and non-offset curated cases emit `0.0`; the two offset cases emit the captured realized magnitude.
- **`load_cpp_ndjson` return shape**: introduce a small `CppCase` dataclass (`records: List[TraceRecord]`, `realized_offset_cm: float`) and return `Dict[str, CppCase]`. There is one production consumer (`parity_run`) plus loader unit tests; the new required field forces those test fixtures to change regardless, so a clean typed return is preferable to a side-channel dict.
- **Commits grouped by area** (user choice): three commits — finding 3 (C++ trace fidelity); findings 2+4 (Python parity-net correctness); finding 1 (cross-language return-distance coverage incl. allowlist removal). Match the existing `T-`/`test(path):` commit voice on this branch; no AI attribution.
- **Phase 4.5 cross-vendor approach consult skipped**: the two real approach forks were explicitly chosen by the user with full tradeoffs immediately before planning; findings 2 and 4 have a single prescribed fix. No open approach decision remained to consult on.

## Acceptance Criteria

- [ ] **AC1 (finding 3):** Both `PT_PARITY_TRACE` hooks in `PathTracker.cpp` (`_ptEmitBlindDeadReckon` ~2320, `_ptEmitProjectionFollow` ~2157) pass `effectiveDistCm`; the value dispatched to `Drive` is unchanged; the stale comments at ~2142-2146 / ~2253-2258 are corrected.
- [ ] **AC2 (finding 2A):** `Allowlist.match` takes the diverging side and suppresses a mismatch only when `entry.side` equals it; a `cpp`-scoped entry no longer mutes a `python`-side regression on the same `(case, kind, field)`.
- [ ] **AC3 (finding 2B):** `Allowlist.match` uses `fnmatchcase` (case-sensitive on every platform); behavior is identical on Windows and Linux for any selector.
- [ ] **AC4 (finding 4):** The axle pull-back formula (`gap = wall_coord - node_coord`; `sign`; `axle_coord = wall_coord - sign*WALL_CONTACT_OFFSET_CM`) exists in exactly one helper, called by both `_apply_wall` and `_dispatch_wall_reset`; the wall parity cases still pass pose cross-check.
- [ ] **AC5 (finding 1, plumbing):** Every `goal_start` record carries `realized_offset_cm`; `PARITY_SCHEMA_VERSION == 2` on both sides; for the two offset cases the sim is seeded with the realized magnitude parsed from the C++ stream, not the nominal table value.
- [ ] **AC6 (finding 1, coverage):** With the realized seed, `return_move.dist_cm` matches within `DIST_TOL` (1e-4); the `return-move-offset-encoder-quantization` entry is removed from `parity_allowlist.json`; `test_no_stale_allowlist_entries` passes (no stale entry), and `test_no_field_divergences` (or equivalent) passes with no unmatched `return_move.dist_cm` mismatch.
- [ ] **AC7 (verification):** `uv run --extra dev pytest tools/path_planner/tests/` is fully green **with the parity test actually executing** (dump binary built, `PARITY_DUMP_EXE` set) — not skipped.
- [ ] **AC8 (no regressions):** Host ctest passes (`ctest --test-dir build/test`); `uv run python tools/path_planner/build_routes.py` then `git diff --exit-code` on `routes.h/.cpp` is clean (freshness gate); the dump and ctest targets compile under the C++ toolchain.

## Contracts & Interfaces

Single source of truth for every seam more than one task touches.

### Signatures

- **`Allowlist.match(self, case_id: str, event_kind: str, field_path: str, side: str) -> Optional[AllowlistEntry]`** — owner: **T2** (parity.py). New trailing `side` param. Filter adds `and entry.side == side`, and the glob is `fnmatchcase(case_id, entry.case_selector)`. Consumer: `_diff_records` (test_pathtracker_parity.py, also edited in T2).
- **Diverging-side value** — `_diff_records` passes the side string. The allowlist vocabulary is `"cpp"` / `"python"` (`AllowlistEntry.side` docstring, parity.py:1086). A `cpp_value != sim_value` mismatch is attributed to **`"cpp"`** in this suite (the C++ stream is the side under test; the single shipped entry is `side: "cpp"`). Owner: **T2**. Pin the exact string to `"cpp"` and add a one-line comment explaining the mapping.
- **`_wall_axle_coord(wall_coord: float, node_coord: float) -> float`** (name at implementer's discretion) — owner: **T2**, lives in `runtime.py`. Returns `wall_coord - (1.0 if (wall_coord - node_coord) >= 0.0 else -1.0) * WALL_CONTACT_OFFSET_CM`. Called by `runtime.py::_dispatch_wall_reset` (~2381) and `parity.py::_apply_wall` (~937, which already imports `WALL_CONTACT_OFFSET_CM` from runtime, parity.py:100). Must preserve the existing numeric result exactly.
- **`load_cpp_ndjson(text: str) -> Dict[str, CppCase]`** — owner: **T3** (parity.py). Return type changes from `Dict[str, List[TraceRecord]]` to `Dict[str, CppCase]`.
- **`_run_sim_case(field, graph, next_hop, case, realized_offset_cm: float = 0.0) -> List[TraceRecord]`** — owner: **T3** (test_pathtracker_parity.py). When `case.offset is not None`, seed `sim.operator_move(realized_offset_cm)` instead of the nominal `case.offset[0]`. Keep the existing `dy == 0 and dtheta == 0` assertion against `case.offset`.

### Shared types

- **`CppCase`** (new `@dataclass`, parity.py, exported via `__all__`) — owner: **T3**:
  - `records: List[TraceRecord]` — the per-hop body (what the dict values used to be).
  - `realized_offset_cm: float` — parsed from this case's `goal_start.realized_offset_cm` (0.0 for clean / non-offset cases).
- **`TRACE_RECORD_FIELDS["goal_start"]`** — owner: **T3** — gains `"realized_offset_cm": float`. This automatically subjects the field to `_record_from_obj`'s strict exact-set validation.

### Naming (must match byte-for-byte across the C++ emitter and Python loader)

- JSON key: **`realized_offset_cm`** on the `goal_start` record. Emitted by C++ `emitGoalStart` with `%.6f`; declared in `TRACE_RECORD_FIELDS["goal_start"]` as `float`. Must be the EXACT same key string on both sides.
- `PARITY_SCHEMA_VERSION` literal **`2`** — set in `WRO2026/test/CMakeLists.txt` (`-DPARITY_SCHEMA_VERSION=2` on the dump target) and in `parity.py` (`PARITY_SCHEMA_VERSION: int = 2`).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/PathTracker.cpp` | T1 | — |
| `tools/path_planner/sim/runtime.py` | T2 | — |
| `tools/path_planner/sim/parity.py` | T2, then T3 | T3 reads T2's parity.py edits (serialized via blocker) |
| `tools/path_planner/tests/test_pathtracker_parity.py` | T2, then T3 | serialized via blocker |
| `WRO2026/test/test_path_tracker_parity_dump.cpp` | T3 | — |
| `WRO2026/test/CMakeLists.txt` | T3 | — |
| `tools/path_planner/tests/fixtures/parity_allowlist.json` | T3 | — |

`parity.py` and `test_pathtracker_parity.py` are each edited by both T2 and T3 — **T3 is blocked by T2** so they never run in parallel on the same file.

## Error Handling

- **`return_move.dist_cm` does NOT collapse after the realized seed** (residual divergence > `DIST_TOL`): treat as a real finding, not an allowlist candidate. STOP and report — do not re-add or broaden an allowlist entry to hide it. Investigate whether the C++ return distance has quantization beyond the offset (it should not: the dead-reckon distance is the offset magnitude; the seek distance is the commanded magnitude). If genuinely irreducible, surface to the user before deleting the entry.
- **Loader rejects the stream** (`ParitySchemaError: goal_start field mismatch`): a `goal_start` somewhere is missing/extra `realized_offset_cm` — confirm `emitGoalStart` emits it on ALL paths (clean + curated) and `TRACE_RECORD_FIELDS["goal_start"]` carries it.
- **`schema_version 1 != 2`**: a stale dump binary — rebuild the dump target before running the parity test.
- **Parity test skips locally** (no binary): expected without a build; CI fails hard. Build the dump and set `PARITY_DUMP_EXE` for a real local run (see Notes).
- **Allowlist `side` mapping wrong**: if `_diff_records` passes `"python"` while the entry says `"cpp"` (or vice-versa), a genuine divergence goes unmuted (loud test failure) — safe direction, but verify the mapping is `"cpp"` per the contract.

## Testing Strategy

**Levels:** the existing standing-guard pytest suite + host ctest are the verification surface; no new test framework. Add/adjust fixtures the contract changes force.

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | Full parity sweep with dump binary built | Integration | `test_pathtracker_parity.py` runs (not skipped); all standing guards green |
| TC2 | `return_move.dist_cm` on `off_n14_h0` / `off_n6_h0` | Integration | Matches within `DIST_TOL` once sim is seeded with the realized offset |
| TC3 | Stale-allowlist guard after entry deletion | Unit/Integration | `test_no_stale_allowlist_entries` passes; `parity_allowlist.json` has zero entries (or none stale) |
| TC4 | `Allowlist.match` with non-matching side | Unit | A `side="cpp"` entry does NOT suppress a `python`-side mismatch on the same `(case, kind, field)` |
| TC5 | `Allowlist.match` glob case-sensitivity | Unit | `fnmatchcase` yields identical matches regardless of OS; a case-mismatched selector does not match |
| TC6 | `goal_start` strict schema with new field | Unit | A `goal_start` without `realized_offset_cm` raises `ParitySchemaError`; with it (+ `schema_version=2`) parses |
| TC7 | Wall-reset pose cross-check after helper extraction | Integration | `reset_wall1` / `reset_wall2` cases still pass `_diff_pose`; numeric axle result unchanged |
| TC8 | Host ctest suite | Integration | `ctest` all-pass (finding 3's C++ edit + the dump compile) |
| TC9 | routes_fixture freshness | Integration | `build_routes.py` then `git diff --exit-code` clean |

**Test data:** the existing `field_fixture` + `routes_fixture`; the two pinned offset cases drive TC2/TC3. **Run commands:** see Notes for Implementer.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Trace records driven distance (finding 3) | — | low | `WRO2026/PathTracker.cpp` | In the two `PT_PARITY_TRACE` hooks, replace `decision.commandedDistanceCm` with `effectiveDistCm` (the value passed to `Drive`): `_ptEmitBlindDeadReckon(...)` (~2320, BLIND_DEAD_RECKON) and `_ptEmitProjectionFollow(...)` (~2157, PROJECTION_FOLLOW). Confirm `effectiveDistCm` (computed at ~2259 / ~2150 via `_pivotTrimDistance(edge)`) is in scope at each emit site. Do NOT change what `Drive` is dispatched. Correct the comments at ~2142-2146 and ~2253-2258 that claim `decision.commandedDistanceCm` is the recorded value. Satisfies AC1. |
| T2 | Allowlist side+glob and shared wall helper (findings 2+4) | — | low | `tools/path_planner/sim/parity.py`, `tools/path_planner/sim/runtime.py`, `tools/path_planner/tests/test_pathtracker_parity.py` | **Finding 2:** add a trailing `side: str` param to `Allowlist.match` (parity.py ~1126); filter requires `entry.side == side`; replace `fnmatch(...)` with `fnmatchcase(...)` (update the `from fnmatch import` at ~62). In `_diff_records` (test ~467) pass the diverging side `"cpp"` per the contract, with a one-line comment on the mapping. **Finding 4:** add a helper in `runtime.py` (e.g. `_wall_axle_coord(wall_coord, node_coord)`) returning `wall_coord - sign(gap)*WALL_CONTACT_OFFSET_CM`; call it from `_dispatch_wall_reset` (~2357-2381) and from `parity.py::_apply_wall` (~933-937). Preserve exact numerics. Satisfies AC2, AC3, AC4. |
| T3 | Realized-offset seed + allowlist removal (finding 1) | T2 | med | `WRO2026/test/test_path_tracker_parity_dump.cpp`, `WRO2026/test/CMakeLists.txt`, `tools/path_planner/sim/parity.py`, `tools/path_planner/tests/test_pathtracker_parity.py`, `tools/path_planner/tests/fixtures/parity_allowlist.json` | **C++:** in `runCuratedCase` capture `float realized = pushOperatorOffset(...)` (0.0 when no offset); add a `float realizedOffsetCm` param to `emitGoalStart` and emit `,"realized_offset_cm":%.6f`; `runCleanCase` passes `0.0`. Bump `-DPARITY_SCHEMA_VERSION` to `2` on the dump target in `WRO2026/test/CMakeLists.txt`. **parity.py:** `PARITY_SCHEMA_VERSION = 2`; add `"realized_offset_cm": float` to `TRACE_RECORD_FIELDS["goal_start"]`; add `CppCase` dataclass (`records`, `realized_offset_cm`), export it; change `load_cpp_ndjson` to return `Dict[str, CppCase]`, reading `realized_offset_cm` off each case's `goal_start` before it is consumed. **test:** in `parity_run` use `cpp_cases[case_id].records` and pass `.realized_offset_cm` into `_run_sim_case`; in `_run_sim_case` seed `sim.operator_move(realized_offset_cm)` for offset cases. Update every other `load_cpp_ndjson` caller / loader unit test / `goal_start` fixture for the new return type + required field. **allowlist:** after confirming the divergence collapses, delete the `return-move-offset-encoder-quantization` entry from `parity_allowlist.json`. Satisfies AC5, AC6. |
| T4 | Build + verify everything | T1, T3 | low | (no source edits) | Build the dump target, run the parity suite against it, run the full Python suite, host ctest, and the routes freshness gate. See Notes for the exact commands. If `return_move.dist_cm` does not collapse, follow Error Handling (stop + report, do not re-add an allowlist entry). Satisfies AC7, AC8. |

## Notes for Implementer

- **Build the dump locally (memory `gotcha-host-build-vs-generator-not-vcvars-cmd`):** do NOT shell vcvars through cmd. Drive the pre-configured VS-generator tree with the bundled cmake. Configure once if needed (`cmake -S WRO2026/test -B build/test`), then `cmake --build build/test --config Debug --target test_path_tracker_parity_dump`. The MSVC toolchain self-discovers (memory `host_cpp_toolchain_available`; probe with vswhere via PowerShell, not Bash).
- **Run the parity test for real:** point `PARITY_DUMP_EXE` at the built binary (e.g. `build/test/Debug/test_path_tracker_parity_dump.exe`), then `uv run --extra dev pytest tools/path_planner/tests/test_pathtracker_parity.py`. Bare `uv run pytest` fails (memory `gotcha-uv-run-extra-dev-pytest`) — always `--extra dev`.
- **Run the FULL pytest, not per-file** (memories `gotcha-strict-emitter-field-fixtures`, `gotcha-field-contract-change-breaks-cpp-and-py-fixtures`): the `goal_start` schema change + `load_cpp_ndjson` return-type change will break loader unit tests and any `goal_start`-building fixture across files. Grep for `load_cpp_ndjson` and `goal_start` usages and fix all of them; only the full run finds them.
- **The parity test is CI-gated-by-path** (memory `gotcha-ci-gated-test-broad-pytest-collection`): it `pytest.skip`s locally without a binary but `pytest.fail`s under `CI=true`. The broad pytest job `--ignore`s it; the dedicated `parity` job builds the binary and invokes it by path. Keep that split intact — do not make the broad job depend on the binary.
- **Host FSM tests link the real `routes_fixture.cpp` by node** (memories `gotcha-host-tests-link-production-routes`, `project-host-test-link-toy-routes`): finding 3's `PathTracker.cpp` edit recompiles into those targets, so run the full ctest, not just one target.
- **Sim payload is the COMMANDED value, not displacement** (memory `gotcha-sim-primitive-payload-is-command-not-displacement`): for the LINE-seek offset case (`off_n14_h0`) `return_move.dist_cm` is the commanded magnitude; for the BLIND case (`off_n6_h0`) it is the open-loop displacement = the offset magnitude. Both are direct functions of the offset, which is why seeding the realized offset collapses the divergence.
- **`%.6f` precision** on the emitted `realized_offset_cm` is ~1e-6 cm — comfortably inside `DIST_TOL = 1e-4`. The C++ FSM uses full-precision internally; the ~1e-6 emit rounding is negligible.
- **Commit order (grouped):** (a) finding 3 alone; (b) findings 2+4 together; (c) finding 1 (C++ + Python + allowlist deletion). Match the branch's `test(path):` / `fix(path):` voice from `git log -20 --oneline`; no AI attribution. After the set lands and verifies, `git push` to PR #116.
- **Rollback:** each fix is independent. If finding 1 cannot collapse the divergence cleanly, ship findings 2/3/4 (they are self-contained) and surface finding 1 for a decision rather than masking it.
