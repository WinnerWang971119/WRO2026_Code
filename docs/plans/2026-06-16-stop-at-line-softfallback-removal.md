# Stop-at-line soft-fallback removal + survivor cleanups

**Goal:** Remove the blind→projection stop-at-line "soft fallback" entirely so the auto-seek fires a *normal* `Drive.stopAtLine` (standard margin, hard-fault on a miss), eliminating the two latent soft-fallback bugs by deletion; defer the genuinely-wanted "drive back to the node on overshoot" recovery to a new issue; and land the survivor low-risk cleanups from the same review.

**Approach:** Strip the `softFallback` concept from both language resolvers and every transcription/consumer (C++ `StopAtLinePolicy.h`, Python `runtime.py`, the 936-row CSV + generator, both test suites), delete the firmware recovery branch + `_activeSoftFallback` + the `STOP_AT_LINE_FAULT_FALL_BACK` toggle + the zero-margin `setMaxDistance` builder branch, and supersede ADR-037's soft-fallback sub-decision with a new ADR-038. Keep the auto-FIRE (`Routes::BLIND_PROJECTION_AUTO_SEEK`, the per-edge `stop_at_line_override`, the resolver's `seek`/`motionMode` decision and precedence ladder) untouched. Fold the survivor cleanups (#4, #5, #6, #9, #10, #11, #12, #14) into the same files where they already get touched. Chosen approach = the Opus consult's **Option A** (full delete), staged as two commits (firmware+resolver+tests, then CSV+generator).

This plan supersedes the soft-fallback parts of `docs/plans/2026-06-15-issue43-auto-stop-at-line-projection.md` and the originating review of 15 findings (user request: "Fix bug with me. Please verify first then we decide what to fix.").

## Scope

- **In scope:**
  - Remove `softFallback` from `StopAtLineResolution` (C++) and the `resolve_stop_at_line` return (Python), plus the internal `autoTriggered`/`auto_triggered` + step-3b logic.
  - Delete the `PathTracker.cpp` soft-recovery branch and **all six** `_activeSoftFallback` write sites + the latch, the `_activeSoftFallback` member in `PathTracker.h`, and the `Config::STOP_AT_LINE_FAULT_FALL_BACK` toggle.
  - Collapse the two `Drive.stopAtLine(...)` builder chains (soft vs normal) into one normal seek — drop the `.setMaxDistance(edge.distance_cm)` zero-margin override so the auto-seek uses the standard `STOP_AT_LINE_MAX_MARGIN_CM` default.
  - Remove the `expect_softFallback` CSV column, the generator column + its soft-fallback self-check, and the soft-fallback assertions/tests in both `test_runtime_sim.py` and `test_path_tracker_stop_at_line.cpp` — **keeping** the auto-fire `seek=true` coverage.
  - Update `compiler.py`'s two unpack sites to the 2-tuple (would otherwise `ValueError`).
  - Write ADR-038 superseding ADR-037's soft-fallback sub-decision; update ADR-037 status + cross-links + DECISIONS.md.
  - Close obsolete issue #60; file two new issues: (a) position-aware come-back recovery, (b) CSV-driven C++ full-domain parity harness.
  - Survivor cleanups: #4 (peek precondition comment), #5 (cross-enum coercion doc), #6 (add `BLIND_PROJECTION_AUTO_SEEK` to two fixtures), #9 (ADR rung-5 wording), #10 (hoist compiler peek), #11 (fold the two override readers), #12 (extract the PathTracker next-hop projection peek helper), #14 (generator double-build).
- **Out of scope (deferred to new issues):**
  - The real come-back-on-overshoot recovery (position-aware return to the node when the seek overshoots, e.g. sensor failure) — design only; bench-gated geometry. New issue (a).
  - Building the CSV-driven C++ full-domain parity harness — new issue (b). This pass only corrects the false "every row" docstrings to "representative rows" for the C++ side.
  - Any change to `field_data` geometry, the emitter, or `routes.h/.cpp` (must stay byte-identical — freshness CI).

## Decisions

- **Full removal over deprecate-in-place** — a `softFallback` output with no consumer is exactly the "speculative abstraction" the codebase bans; deleting it restores the resolver to a clean `(seek, motionMode)` shape and removes both bugs by construction. (Opus consult Option A, endorsed.)
- **Keep the auto-fire** — `BLIND_PROJECTION_AUTO_SEEK` + per-edge `stop_at_line_override` + the resolver seek decision stay; only the *soft* behavior is removed. A blind hop feeding a projection still auto-fires a (now normal) `stopAtLine`.
- **Miss = hard fault for all seeks** — a missed line latches `Fault::DRIVE_FAULT` via the existing shared guard, identical to every other `stopAtLine`. The dev/competition "surface missed lines loudly" posture (previously documented on the deleted toggle) is preserved in ADR-038 and carried into the come-back issue.
- **Defer the come-back recovery** — it is hardware-geometry-coupled and bench-gated; it does not belong in a destructive cleanup. New issue captures the `LINE_NOT_FOUND`-fault-kind discrimination + dev/competition posture as design inputs.
- **New ADR-038, supersede ADR-037's sub-decision** — per the repo ADR convention (write the next number, mark the old one partially superseded, cross-link both ways). Do not rewrite ADR-037 in place beyond status + cross-link.
- **Retain `LINE_SENSOR_AXLE_OFFSET_REAR_CM` and `STOP_AT_LINE_MAX_MARGIN_CM`** — both have live non-soft-fallback users; removing the recovery branch must not remove either constant.

## Acceptance Criteria

- [ ] AC1 — The blind→projection auto-seek dispatches a single `Drive.stopAtLine(edge.distance_cm).setReverse(...).setMode(...).setHeading(...).async()` with **no** `.setMaxDistance(...)` override (uses the default `expected + STOP_AT_LINE_MAX_MARGIN_CM`). *(Inspection + arduino-cli-compile verified; the mock Drive has no fault surface, so no host unit-test asserts the absence of the builder call.)*
- [ ] AC2 — A missed line on any seek enters `Fault::DRIVE_FAULT` via the shared guard; no recovery branch and no `_activeSoftFallback` exist anywhere in `WRO2026/`.
- [ ] AC3 — `resolveStopAtLine` returns `{seek, motionMode}` (no `softFallback`); `resolve_stop_at_line` returns `(seek, motion_mode)`; no `softFallback`/`auto_triggered` token remains in either resolver.
- [ ] AC4 — `stop_at_line_truth_table.csv` has no `expect_softFallback` column; re-running the generator reproduces it byte-identical; the base/supplement/total counts stay **864 / 72 / 936** and the legacy-slice self-check stays 144.
- [ ] AC5 — `Config::STOP_AT_LINE_FAULT_FALL_BACK` is deleted and `grep` finds zero references to it. Each retained constant — `STOP_AT_LINE_MAX_MARGIN_CM`, `LINE_SENSOR_AXLE_OFFSET_CM`, `LINE_SENSOR_AXLE_OFFSET_REAR_CM` — still has ≥1 non-test reference after the change (`grep`-verifiable).
- [ ] AC6 — Host ctest (MSVC) passes; `arduino-cli compile` passes; `uv run --extra dev pytest tools/path_planner/tests/` passes; `build_routes.py` produces no diff (`git diff --exit-code` on `routes.h/.cpp` clean).
- [ ] AC7 — The auto-fire is intact: `BLIND_PROJECTION_AUTO_SEEK` still gates the auto-rule; the TC20 auto-fire tests still assert `seek=true` on a blind→projection hop.
- [ ] AC8 — ADR-038 exists (supersedes ADR-037's soft-fallback sub-decision), ADR-037 status/cross-links updated, DECISIONS.md index + status table updated; ADR-037's rung-5 wording corrected (#9).
- [ ] AC9 — Issue #60 closed with an explanatory comment; new issues filed for (a) come-back recovery and (b) CSV-driven C++ parity harness.
- [ ] AC10 — Survivor cleanups present: #4 comment, #5 doc (runtime + header_writer), #6 attr on both fixtures, #10 hoisted peek, #11 single shared override reader, #12 shared peek helper, #14 single `build_rows` call.

## Data Model (the new resolver contract)

C++ (`StopAtLinePolicy.h`):
```cpp
struct StopAtLineResolution
{
    bool              seek;        // dispatch the Drive.stopAtLine seek
    Routes::BlindMode motionMode;  // concrete bundle; NEVER STOP_AT_LINE or DEFAULT
};
// StopAtLineQuery is UNCHANGED (7 fields). resolveStopAtLine(query) still takes the
// same inputs; only the result drops `softFallback` and the internal `autoTriggered`
// + step-3b assignment. The precedence ladder (rungs 1-5) is unchanged.
```

Python (`runtime.py`):
```python
def resolve_stop_at_line(...) -> tuple[bool, BlindMode]:   # was tuple[bool, BlindMode, bool]
    ...
    return seek, motion_mode                                # drop soft_fallback
```

CSV header (`stop_at_line_truth_table.csv`):
```
callFlag,edgeOverride,edgeMode,pipelineBlind,direction,lineBoardCount,autoRuleActive,expect_seek,expect_motionMode
# (expect_softFallback column removed; 936 data rows retained)
```

## Error Handling

- **Missed line during the auto-seek:** the seek runs to `expected + STOP_AT_LINE_MAX_MARGIN_CM`, finds no `T_OR_CROSS`, and `Drive` latches `Fault::LINE_NOT_FOUND`. The shared guard at the top of `update()` (`PathTracker.cpp:539`) stops the chassis and enters `Fault::DRIVE_FAULT`. No dead-reckon fallback — loud by design.
- **Intermediate build state:** `Config.h` deleting the toggle while `PathTracker.cpp` still referenced it would break the build; the firmware task removes every reader, and verification compiles the final state. Any dangling `_activeSoftFallback` after the member is deleted is a compile error — the firmware task must remove all six write sites + the latch in the same change as the member.
- **Parity drift:** if the C++ and Python resolvers disagree after the edit, the CSV-driven Python TC13 (936 rows) and the C++ representative-vector tests fail. The generator self-checks refuse to emit a malformed table.

## Testing Strategy

**Levels:** host C++ unit (ctest), Python unit (pytest), Arduino compile (arduino-cli), generated-artifact freshness (build_routes + git diff).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `resolveStopAtLine` over representative vectors (TC1-TC13/TC20 minus softFallback) | Host C++ | `seek` + `motionMode` match; file compiles against the 2-field result |
| TC2 | Auto-rule seek on a blind→projection (FRONT 1-board; REAR 2-board) | Host C++ | `seek == true`, `motionMode` keeps the edge mode (auto-fire retained) |
| TC3 | Python TC13 drives all 936 CSV rows through `resolve_stop_at_line` | Python | `(seek, motion_mode)` matches every row; `_EXPECTED_ROW_COUNT == 936` |
| TC4 | Re-run `gen_stop_at_line_truth_table.py` | Python/CLI | CSV byte-identical to committed; counts 864/72/936; legacy slice 144 |
| TC5 | `compile_path` + `PathTrackerSim` byte-identical streams (TC10/TC12 random walks) | Python | Unchanged — 2-tuple unpack does not alter emitted primitives |
| TC6 | Firmware compiles after removal | arduino-cli | No dangling `_activeSoftFallback` / `STOP_AT_LINE_FAULT_FALL_BACK` refs; links clean |
| TC7 | `build_routes.py` then `git diff --exit-code` | CLI | No change to `routes.h/.cpp` (no field_data/emitter touched) |
| TC8 | Fixtures `bidirectional_field_data` / `collinear_4node` carry `BLIND_PROJECTION_AUTO_SEEK` | Python | A future emit_routes over them does not `AttributeError` (#6) |

**Test data:** existing fixtures + the regenerated CSV. **Run commands:** host ctest via the MSVC/vcvars64 + cmake/ninja flow; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`; `uv run --extra dev pytest tools/path_planner/tests/`; `uv run python tools/path_planner/tests/fixtures/gen_stop_at_line_truth_table.py`; `uv run python tools/path_planner/build_routes.py`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | C++ resolver removal | — | med | `WRO2026/StopAtLinePolicy.h` | Delete the `softFallback` field from `StopAtLineResolution`, the `autoTriggered` local, and the step-3b assignment. Keep the 7-field `StopAtLineQuery` and the rungs-1-5 ladder. Remove EVERY softFallback mention in BOTH doc blocks: the file-level docstring (~lines 10-71, incl. the parity claim ~67-70) AND the function-level Doxygen (`@return The {seek, motionMode, softFallback}...` ~line 127 + surrounding prose). Correct the C++ parity claim to "representative rows" (Python remains "full domain") (#1). Satisfies AC3, part of AC8(#9-adjacent wording). |
| T2 | Python resolver removal + fold readers + coercion doc | — | med | `tools/path_planner/sim/runtime.py` | In `resolve_stop_at_line`: drop `auto_triggered` + `soft_fallback`, change return to `(seek, motion_mode)` and its annotation; update its docstring + module docstring. Update the `_walk_hops` unpack (~:1255) to 2-tuple. Fold `_read_edge_stop_at_line_override` + `_read_edge_dir_override` into ONE shared coercion helper that returns the raw enum member incl. DEFAULT (NOT the DEFAULT→pipeline-collapsing `_resolve_override`) (#11). Add a comment documenting the cross-enum-int coercion hazard (latent; builder bakes only real members) (#5). Satisfies AC3, AC10(#11,#5). |
| T3 | Compiler 2-tuple + hoist peek + precondition comment | — | low | `tools/path_planner/compiler.py` | Update BOTH `resolve_stop_at_line` unpack sites (~:353, ~:411) to the 2-tuple. Hoist the duplicated `next_hop_is_projection`/`auto_rule_active` peek to the top of the per-hop loop body (or a small shared helper) — NOT literally "above the REAR/FRONT split"; computing it once per hop for all edge types is harmless (#10). Add a comment that `path[i+2]` requires a NEXT_HOP-consistent path and that the robot + `PathTrackerSim`/CTPS use the NEXT_HOP-table peek (#4). Satisfies AC3, AC10(#10,#4). |
| T4 | Header-writer coercion doc | — | low | `tools/path_planner/header_writer.py` | Add a comment on the int best-effort branch (~:214-218) documenting the cross-enum-int coercion hazard, mirroring T2's note (#5). Satisfies AC10(#5). |
| T5 | Generator: drop column + self-check + double-build | — | low | `tools/path_planner/tests/fixtures/gen_stop_at_line_truth_table.py` | Remove `expect_softFallback` from HEADER + `_compute_row` (drop `auto_triggered` + the soft-fallback cell), and delete the soft-fallback self-check block. Fix the double-build (#14): `write_csv` currently calls `_build_base_rows()` + `_build_supplemental_rows()` AND `build_rows()` (which re-calls both). Note the base/supplement split CANNOT be recovered from `build_rows()`'s combined list — so derive the two counts from the dimension products (the same arithmetic `build_rows`'s asserts already use) and call `build_rows()` exactly once for the row data. Update the module docstring. Keep counts 864/72/936 + the 144 legacy slice. Satisfies AC4, AC10(#14). |
| T6 | Regenerate CSV | T5 | low | `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv` | Run `uv run python tools/path_planner/tests/fixtures/gen_stop_at_line_truth_table.py`; commit the regenerated CSV (936 rows, no `expect_softFallback` column). Do not hand-edit. Satisfies AC4. |
| T7 | Firmware removal + peek helper | — | high | `WRO2026/PathTracker.cpp`, `WRO2026/PathTracker.h` | Delete the soft-recovery branch (~:469-537). Delete ALL SIX `_activeSoftFallback = false` writes (~:244 begin, ~:305 abort, ~:533 recovery, ~:578 DEQUEUE, ~:883 `_enterFault`, ~:908 goal-complete) and the latch `_activeSoftFallback = sal.softFallback` (~:1336), plus the member + Doxygen in `PathTracker.h` (~:346-358). Collapse the `if (sal.softFallback) {...} else {...}` stopAtLine chains (~:1374-1401) into the single `else`-branch chain (one discarded-temporary RAII statement — do NOT `auto b = ...`). Extract the next-hop projection peek (NODE_NONE/self-loop guard, ~:1307-1317) into a private helper and reuse it for the pass-through peek (~:626-633) (#12); guard so the peek short-circuits when `Routes::BLIND_PROJECTION_AUTO_SEEK` is compile-time false (#13). Do NOT touch `LINE_SENSOR_AXLE_OFFSET_REAR_CM` usage at the LINE_OVERSHOOT path (~:710). Satisfies AC1, AC2, AC7, AC10(#12,#13). |
| T8 | Config toggle removal | — | low | `WRO2026/Config.h` | Delete `STOP_AT_LINE_FAULT_FALL_BACK` + its Doxygen block (~:619-637). Leave `STOP_AT_LINE_MAX_MARGIN_CM` and both `LINE_SENSOR_AXLE_OFFSET_*` intact. No ConfigChecks.h tripwire exists for the toggle. Satisfies AC5. |
| T9 | Host C++ test update | T1, T7 | low | `WRO2026/test/test_path_tracker_stop_at_line.cpp` | Drop every `r.softFallback` assertion (TC1-TC20) by stripping just that line, NOT the whole test. KEEP the auto-fire tests (`...auto_rule_active_front...`, `...auto_rule_rear_2board...`) re-purposed to `seek==true`/`motionMode` only. KEEP `test_TC20_auto_rule_rear_1board_degrades_no_soft_fallback` but strip only its `ASSERT_FALSE(r.softFallback)` line — it retains a valid `seek==false` + `motionMode==SYNC_TSHAPE` auto-rule board-degrade vector. DELETE only `test_TC20_soft_fallback_only_for_auto_not_explicit_seeks` + its `main()` call (genuinely softFallback-only). Re-point the `// MIRROR: PathTracker.cpp:NNN` back-pointers (test lines ~139-140, ~197-199) to the post-T7 line numbers — T7's ~50-line deletion + chain-collapse shifts every cited site. Update file docstrings: drop softFallback, change the parity claim to "representative rows" (#1). `StopAtLineQuery{...}` literals stay valid (7 inputs unchanged). Satisfies AC1, AC7. |
| T10 | Python test update | T2, T6 | low | `tools/path_planner/tests/test_runtime_sim.py` | Drop the `row["expect_softFallback"]` access, the 3-tuple unpack, the soft-fallback assert clause, and the failure-message field (~:1855-1914). Keep `_EXPECTED_ROW_COUNT == 936` (row count unchanged). Satisfies AC3, AC4. |
| T11 | Fixture attr (#6) | — | low | `tools/path_planner/tests/fixtures/bidirectional_field_data.py`, `tools/path_planner/tests/fixtures/collinear_4node.py` | These fixtures expose `SimpleNamespace` objects from factory functions, NOT a module-level `field_data` — a bare module constant would be a no-op. Add `BLIND_PROJECTION_AUTO_SEEK=True` to EACH returned `SimpleNamespace(...)` (the 3 factories in `bidirectional_field_data.py` and `make_field_data` in `collinear_4node.py`). The strict readers that would `AttributeError` are `header_writer.py:425` (`field_data.BLIND_PROJECTION_AUTO_SEEK`) and `validator.py` (`getattr(..., "BLIND_PROJECTION_AUTO_SEEK")` with no default). Satisfies AC10(#6). |
| T12 | ADR-038 + ADR-037 + index | — | low | `docs/decisions/038-stop-at-line-normal-seek-no-soft-fallback.md` (new), `docs/decisions/037-blind-projection-auto-stop-at-line.md`, `DECISIONS.md` | Write ADR-038: auto-seek fires a normal `stopAtLine` (default margin), miss = hard fault, soft-fallback removed, come-back deferred to issue (a); preserve the dev/competition "fail loud" posture. ADR-037 is only PARTLY superseded (auto-rule / per-edge override / toggle survive), so keep its Status "Implemented" and add a body note "**Soft-fallback sub-decision superseded by ADR-038**" + cross-links (do NOT invent a non-standard Status string). Fix its rung-5 wording ("Pipeline blind default (no seek)" → the resolver's unconditional "never seek; no pipeline-default rung") (#9). In DECISIONS.md: add the ADR-038 index line + a status-table row (Implemented count 30→31; ADR-037 stays in the Implemented row). Confirm 038 is still free at write time (ADR-collision gotcha). Satisfies AC8. |
| T13 | GitHub issues | — | low | (GitHub via `gh`) | Close #60 with a comment (recovery removed; superseded by the pivot; link issue (a)). File issue (a): position-aware come-back-to-node recovery on seek overshoot — design inputs: signed corrective = `axle_offset − margin`; `Drive.getFault().kind == LINE_NOT_FOUND` already discriminates the fault (note it so the feature does not re-derive it); dev/competition fail-loud posture; bench-gated geometry; AND it will need host-test coverage when built (the debt #60 tracked moves here). File issue (b): CSV-driven C++ full-domain parity harness reading `stop_at_line_truth_table.csv` through the real `resolveStopAtLine` (restore "every row" in the C++ docstrings when it lands). Satisfies AC9. |
| T14 | Integration verification | T1,T2,T3,T4,T6,T7,T8,T9,T10,T11,T12 | med | — (build/test only) | Run: host ctest (MSVC/vcvars64 + cmake/ninja); `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`; `uv run --extra dev pytest tools/path_planner/tests/`; `uv run python tools/path_planner/build_routes.py` then `git diff --exit-code` on `routes.h/.cpp`; confirm the regenerated CSV matches T6. Note: `test_auto_stop_at_line_projection.py` exercises only `compile_path`/sim (not `resolve_stop_at_line` directly), so it is unaffected by the 2-tuple change — it must still pass. Fix any fallout. Satisfies AC6. |

## Notes for Implementer

- **The resolver call signature does NOT change** — `StopAtLineQuery` keeps its 7 fields; only the *result* struct/tuple shrinks. Every `StopAtLineQuery{...}` literal and every `resolve_stop_at_line(...)` argument list stays valid; you are only removing `.softFallback`/the third return element and the reads of it.
- **DO NOT delete** `Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM` (LINE_OVERSHOOT rear, `PathTracker.cpp:710`, + Python mirror) or `Config::STOP_AT_LINE_MAX_MARGIN_CM` (the Drive default the normal seek now relies on).
- **Firmware deletion is all-or-nothing:** removing the `_activeSoftFallback` member while leaving any of its six writes is a compile error. Remove member + all writes + the latch + the recovery branch together (T7).
- **RAII builder rule:** the collapsed `Drive.stopAtLine(...)` chain must be a single discarded-temporary statement — never `auto b = Drive.stopAtLine(...)...` (move-only builder; deleted copy ctor + early enqueue).
- **DictReader is by name**, so dropping the CSV column is safe for `runtime.py:1869`, but `test_runtime_sim.py` directly indexes `row["expect_softFallback"]` (~:1889) — that access must be removed in the same change as the CSV (T10 + T6 land together).
- **Generator row math is unchanged** — `softFallback` was a derived column, not an input dimension. Base 864 / supplement 72 / total 936 / legacy-slice 144 all stay; only the soft-fallback *self-check* block goes.
- **Parity is the safety net:** keep the C++ `resolveStopAtLine` and Python `resolve_stop_at_line` step-for-step identical (both `(seek, motionMode)`); the CSV-driven Python TC13 + the C++ representative vectors fail loudly on drift.
- **No routes/emitter change** — `BLIND_PROJECTION_AUTO_SEEK` and the `EdgeInfo.stop_at_line_override` field are KEPT and untouched, so `routes.h/.cpp` stay byte-identical (freshness CI).
- **Commit shape:** match the repo's Conventional-Commits voice (see recent `feat(tracker)/test(planner)` log); no AI attribution. Stage as two commits per the consult: (1) firmware + Config + resolver + tests; (2) CSV + generator. ADR + issues can ride either.
- **Rollback:** the change is additive-by-deletion on a feature branch; revert the two commits to restore. No data migration.
