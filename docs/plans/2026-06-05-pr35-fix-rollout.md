# PR #35 Trace-Findings Fix Rollout Plan

**Goal:** Land all ~65 accepted "fix now" findings from `docs/PR35_TRACE_FINDINGS.md` as four CI-gated PRs sequenced around the in-flight PR #37, optimizing for speed-to-done without forcing a blind, locally-uncompilable rebase war.

**Approach:** Batch by **actual file-overlap with the two open PRs (#35, #37)** and by verifiability, not by severity. A "conflict-free" wave (files touched by neither #35 nor #37) branches off `main` immediately and banks ~31 fixes in parallel with the open PRs. The remaining fixes wait for #37 to merge, then land as a pipeline-core wave, a harness-enforcement wave (which establishes the ctest CI gate), and a firmware wave (landed under that gate). One commit per finding, coupled pairs kept atomic/ordered, emitter changes regenerate routes + goldens in-commit. Chosen over a single monolithic PR after `/brainstorm` + `/debate` (monolith scored 4.25/10).

## Scope

- **In scope:**
  - Every accepted "fix now" decision in `PR35_TRACE_FINDINGS.md` (A/B decisions, plus the C-decisions that are still a real edit: #8, #27, #40, #79).
  - 3 ADRs for the cross-module rules these fixes establish (FieldMap bounds, validator anti-ambiguity, harness-enforced).
  - The `.github/workflows/ci.yml` ctest job (#75) and harness-quality fixes (#76/#77/#78/#80).
- **Out of scope:**
  - **Defer bucket** — #22, #23, #24, #25, #54, #55, #68, #69, #70, #89 (not touched).
  - **Bench-verify / Phase-3 bucket** — #74, #81, #82, #83, #84, #85, #86, #87, #88, #93 (folded into the doc's Phase-3 bench checklist; NOT coded now).
  - **True no-ops** — #3, #7, #9, #10, #11, #92 (Decision = leave as-is, no code change).
  - Merging #35 or #37 themselves (owned separately); this plan only sequences *around* them.
  - Any behavioral hardware verification (deferred to the Phase-3 bench).

## Decisions

- **Branch the conflict-free wave (PR-1) off `main` now**, before #35 merges — it touches files in neither #35 nor #37, so #35/#37 absorb it via a zero-conflict merge. (User-confirmed; #35 is currently merge-BLOCKED, so waiting would idle ~31 fixes.)
- **PR-1 is one combined PR** (offline Python + non-overlap firmware), not split — fewest round-trips; Python is pre-verified locally, the firmware edits are low-risk pre-existing-module changes.
- **The #75 ctest CI job lands post-#37, bundled with the harness-quality fixes (PR-3)**, so it enforces a *real* harness, not the partly-decorative current one (#76/#78).
- **#78 (link real `Tasks.cpp`/`Mission.cpp`) lands in PR-4** with the Mission/Tasks firmware fixes, not PR-3 — so the newly-real tests pass against fixed code. PR-3 establishes the ctest gate against the still-mirror-for-Tasks harness; PR-4 flips Tasks/Mission to real sources alongside their fixes.
- **PR-1 stays PURE CODE — no `DECISIONS.md`, `ARCHITECTURE.md`, or `Roadmap.md` edits** (all three are in BOTH #35 and #37; touching them would break PR-1's zero-conflict invariant).
- **3 ADRs, all in post-#37 waves**: ADR-028 (FieldMap accessor bounds — the *fix* #33 lands in PR-1, but the ADR file + `DECISIONS.md` index land in PR-2), ADR-029 (validator anti-ambiguity + input finiteness, PR-2), ADR-030 (C++ harness links real sources & is CI-enforced, PR-3). Numbers assigned above what #35 (025/026) and #37 (024/027) consume → 028/029/030; verify against `DECISIONS.md` on the post-#37 branch at execution.
- **Later waves branch off a main that already includes PR-1.** PR-2/PR-3/PR-4 branch only after BOTH #37 AND PR-1 have merged, so cross-wave same-file edits (planner.py in PR-1+PR-2; LineFollower.cpp in PR-1+PR-4; `test/CMakeLists.txt` in PR-3+PR-4; PathTracker in PR-3+PR-4) never diverge.
- **Pivot fixes (#36/#42/#94) stay in scope** (PR-2) — pivot use on the 2026 field is undecided; the fixes are cheap and correct latent bugs.
- **One commit per finding**, human-voice, Conventional-Commit style matching the repo (e.g. `fix(drive): brake on empty queue when profile ends at cruise (trace #13)`), no AI attribution.
- **Emitter-changing commits regenerate in-commit**: any commit altering routes/header emission (#42, #44, #48) runs `uv run python tools/path_planner/build_routes.py` and regenerates the toy goldens in the same commit, or CI's `git diff --exit-code` route-freshness gate fails.
- **Execute partitions agents by file, not by finding** — one agent owns all findings in a file; two tasks touching the same file get a blocking dependency.

## PR / Wave structure

| PR | Wave | Branch base | Gate to start | Contents |
|----|------|-------------|---------------|----------|
| PR-1 | Conflict-free | `main` (now) | none | offline Python (non-overlap) + firmware (non-overlap). **Pure code — no docs/ADR/DECISIONS.md.** |
| PR-2 | Pipeline core | post-#37 + post-PR-1 `main` | #37 & PR-1 merged | overlapping Python (compiler/header_writer/validator/pipeline_config/sim/planner) + regen + ADR-028 + ADR-029 |
| PR-3 | Harness enforcement | post-#37 + post-PR-1 `main` | #37 & PR-1 merged | ci.yml ctest job + CMake harness fixes + shared-guard extraction + ADR-030 |
| PR-4 | Firmware | post-#37 + post-PR-1 `main` | **PR-3 merged** | Mission/Drive/PathTracker/WROMath/Config/.ino/Tasks behavior fixes + #78 link-real |

## Finding → wave → file map

**PR-1 (conflict-free, off main now):**
- graph_builder.py: #37, #39, #40
- planner.py: #43
- visualizer.py: #52, #53
- pick_xy.py: #56, #57, #58, #59, #60
- editor/ (app.py, commands.py, canvas.py): #61, #62, #63, #64, #65, #66, #67
- tests: #71 (test_build_routes.py), #73 (pyproject.toml + test_visualizer.py)
- LineFollower.cpp: #17, #18, #19, #20
- Intersection.cpp/.h: #26
- EventTrace.cpp/.h + Localization.cpp/.h: #30 → #28 (order), #29, #27
- FieldMap.h: #33 (covers #31). **ADR-028 file + DECISIONS.md index deferred to PR-2** (DECISIONS.md is in #35/#37).
- test/README.md: #12

**PR-2 (pipeline core, post-#37 + post-PR-1):**
- validator.py + build_routes.py: #36 (move pivot check into validator, drop the uncaught ValueError path in build_routes), #38+#47 (cross-check, one commit), #46 (finite), #49 (reject unsafe names) + ADR-028 file + ADR-029 + DECISIONS.md index for both
- planner.py: #42 (pivot compensation, emitter-regen) → #94 (double-apply guard, after #42); #36 planner-side removal
- compiler.py: #44 (REAR heading, emitter-regen), #45 (type alias)
- header_writer.py: #48 (enum bodies from Python, emitter-regen), #50 (assert n_nodes<255)
- pipeline_config.py: #51 (reject self-referential DEFAULT)
- sim/runtime.py: #41 (accept string mode names)
- tests + golden: #72 (document regen workflow), #34 (cross-language parity golden), regen integration

**PR-3 (harness enforcement, post-#37):**
- .github/workflows/ci.yml: #75 (add ctest job) + ADR-030
- test/CMakeLists.txt: #76 (PASS_REGULAR_EXPRESSION), #80 (link routes.cpp)
- PathTracker.cpp/.h + test_path_tracker_bidirectional.cpp: #77 (promote guard to shared header)

**PR-4 (firmware, post-#37, after PR-3):**
- Mission.cpp/.h: #1, #2, #6
- Drive.cpp: #13, #14, #15, #16
- PathTracker.cpp/.h: #32 (covers #90), #91
- WROMath.h: #35
- Config.h + LineFollower.cpp: #21 (CRAWL_PWM)
- WRO2026.ino: #4, #5
- Tasks.cpp: #8
- test/CMakeLists.txt + test_tasks_dispatch.cpp: #78 (link real Tasks.cpp/Mission.cpp), #79 (little-endian comment)

## Acceptance Criteria

- [ ] **AC1** — PR-1 merges with green CI (pytest + arduino-compile); every PR-1 finding is its own commit citing the trace number; PR-1 contains no `docs/`, `DECISIONS.md`, `ARCHITECTURE.md`, or `Roadmap.md` edits.
- [ ] **AC2** — PR-1 edits only files in NEITHER #35 nor #37's changed set (verify each path against `gh pr diff 35 --name-only` and `gh pr diff 37 --name-only`); merging it produces zero conflicts in #35/#37.
- [ ] **AC3** — After #37 merges, PR-2 merges green; every emitter-changing commit (#42, #44, #48) includes regenerated `routes.h`/`routes.cpp` + `tools/path_planner/tests/golden/routes_toy.*` so CI's `git diff --exit-code` passes; ADR-029 present.
- [ ] **AC4** — PR-3 adds a ctest job to `ci.yml` that configures with cmake and runs ctest and passes; #76 asserts the `static_assert` text via `PASS_REGULAR_EXPRESSION`; #80 adds `routes.cpp` (or a toy def) to the bidirectional target; ADR-030 present.
- [ ] **AC5** — PR-4 merges green (pytest + arduino-compile + ctest); #78 links the real `Tasks.cpp`/`Mission.cpp` into the harness and the harness's dispatch tests pass against them; every firmware finding is its own commit. NOTE: the ctest gate covers PathTracker/Tasks/LineFollower-overshoot only — Drive/Mission-behavior/.ino/WROMath fixes are compile-only here, with motor/IMU behavior deferred to the Phase-3 bench (out of scope).
- [ ] **AC6 (couplings)** — #38 and #47 land as one commit; #32's commit resolves #90; #31 needs no separate commit (covered by #33); #42 lands before #94; #30 lands before #28; the PR-3 ctest gate lands before #78.
- [ ] **AC7 (no scope creep)** — no file/line in the Defer, bench-verify, or no-op buckets is modified; confirm by diffing touched findings against the doc's Decision lines.
- [ ] **AC8** — every accepted finding is resolved by exactly one landed commit across the four PRs (coupled pairs #38+#47 share one commit; #31 is covered by #33's commit; #90 is covered by #32's commit — these cite the covering finding). No accepted finding dropped, none double-fixed.

## Error Handling / contingencies

- **#37 lands new conflicts into a not-yet-merged PR-1:** PR-1 is constructed to touch neither #35 nor #37 files, so this cannot happen for PR-1. If a late edit pulls a PR-1 finding into an overlapping file, move that finding to PR-2/PR-4.
- **Emitter regen drift:** if `build_routes.py` output changes and the commit forgot to regen, CI's `git diff --exit-code` fails — re-run regen and amend that commit. Never commit an emitter change without the regenerated artifacts.
- **#78 real-link surfaces a failure the mirror hid:** that failure should correspond to an accepted firmware fix in PR-4 (e.g. #1). Order #78's commit after the relevant fix commit; if it surfaces an *unlisted* defect, stop and report rather than papering over it.
- **No local C++ compiler:** all firmware verification is CI-only (arduino-compile + ctest once PR-3 lands). Behavior correctness of motor/IMU/I2C paths is out of scope here and already tracked in the Phase-3 bench checklist.
- **ADR number collision:** #35/#37 introduce ADRs not yet on `main`. At execution, read `DECISIONS.md` on the target branch and assign the next free numbers; update both the ADR file and the `DECISIONS.md` index.

## Testing Strategy

**Levels:** Unit (pytest for offline Python), Compile (arduino-cli for firmware), Harness (ctest once PR-3 lands). Hardware behavior is out of scope (Phase-3 bench).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | graph_builder drops an obstacle-clipped LINE edge (#37) | Unit | emits a `UserWarning` like BLIND/EDGES; `_run` forwards `WARN:` to stderr |
| TC2 | edge grazing inflated-obstacle boundary at min clearance (#39) | Unit | `edge_clips_obstacle` returns False (interior-crossing, not touch) |
| TC3 | `compute_all_pairs_next_hop(cost_blind_multiplier<=0)` (#43) | Unit | raises `ValueError` |
| TC4 | `_is_number(float('nan'))` / Inf in field data (#46) | Unit | rejected; `validate()` raises `FieldDataError` |
| TC5 | EDGES pair conflicting with an existing LINE/BLIND pair (#38/#47) | Unit | `validate()` raises `FieldDataError` |
| TC6 | self-referential `PIPELINE_CONFIG.line = LineMode.DEFAULT` (#51) | Unit | rejected with a clear error |
| TC7 | edge `primitive_mode='NORMAL'` (string) through sim (#41) | Unit | `_resolve_override` resolves via `enum_cls[name]`, no `ValueError` |
| TC8 | `final_heading_after` on a REAR final hop (#44) | Unit | returns chassis heading (edge_heading+180), matches `compile_path` |
| TC9 | pivot compensation reaches emitted distance (#42) | Unit | regenerated routes.cpp EDGES carry the shortened `distance_cm` |
| TC10 | emitter byte-golden after regen (#42/#44/#48) | Unit | `routes_toy.*` golden matches regenerated output |
| TC11 | editor: drag released off-axes records a MoveNode (#61) | Unit | `_finish_move` runs; undo stack has the move |
| TC12 | editor: delete current start node (#62) | Unit | refused with a clear message; model stays saveable |
| TC13 | pick_xy: click while pan/zoom toolbar active (#59) | Unit | no coordinate printed, no marker added |
| TC14 | ctest job runs in CI (#75) | Harness | cmake configure + ctest exit 0 |
| TC15 | overflow WILL_FAIL test (#76) | Harness | passes only when the expected `static_assert` text appears |
| TC16 | real-linked Tasks/Mission dispatch tests (#78) | Harness | the harness's internal TC1..TC15 (in test_tasks_dispatch.cpp) run against real `Tasks.cpp`/`Mission.cpp` (mocked HW), pass post-fix |
| TC17 | firmware sketch still compiles after each firmware fix | Compile | `arduino-cli compile` exit 0 (CI) |

**Test data:** existing toy fixture for header/emitter goldens; `pytest.warns`/`catch_warnings` for warning assertions (and add the `[tool.pytest.ini_options]` block per #73). Run: `uv run pytest tools/path_planner/tests/` locally; CI runs pytest + arduino-compile, and ctest after PR-3.

## Tasks

Risk legend: firmware behavior fixes that can't be locally verified = **med/high**; offline Python with tests = **low/med**; docs/comments = **low**.

### PR-1 — conflict-free (off `main` now). All T1.x parallelizable (disjoint files).

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1.1 | graph_builder fixes | — | med | `tools/path_planner/graph_builder.py` | #37 warn on clipped LINE; #39 interior-crossing test; #40 loud doc on single-direction override. One commit each. Satisfies AC1; TC1, TC2. |
| T1.2 | planner guard | — | low | `tools/path_planner/planner.py` | #43 raise ValueError if `cost_blind_multiplier<=0` at entry. TC3. |
| T1.3 | visualizer fixes | — | low | `tools/path_planner/visualizer.py` | #52 drop `aspect` kwarg on imshow; #53 remove dead `label=`. |
| T1.4 | pick_xy fixes | — | low | `tools/path_planner/pick_xy.py` | #56 free `c` from keymap.back; #57 drop aspect; #58 guard field_data import (exit 2); #59 ignore clicks while toolbar active; #60 broaden imread except + guard overlay shape. TC13. |
| T1.5 | editor fixes | — | med | `tools/path_planner/editor/app.py`, `editor/commands.py`, `editor/canvas.py` | #61 off-axes release records MoveNode; #62 guard delete start node; #63 resolve pending-rename in delete guard; #64 anchor rename recompute to last Save; #65 forbid reusing pending-rename source; #66 guard width_mm<=0; #67 scope shortcuts to canvas focus. TC11, TC12. |
| T1.6 | test hardening | — | low | `tools/path_planner/tests/test_build_routes.py`, `tests/test_visualizer.py`, `pyproject.toml` | #71 base on toy fixture; #73 add `[tool.pytest.ini_options]`. |
| T1.7 | LineFollower fixes | — | high | `WRO2026/LineFollower.cpp` | #17 distance gate trips while line lost; #18 refresh LineArray in no-callback fallbacks (789 & 851); #19 seed `_lastErr` to fresh err one tick post-recovery; #20 both-wheel ramp-up average. Behavioral, compile-only (no local compiler, no ctest coverage for LineFollower except overshoot) — Phase-3 bench verifies behavior. |
| T1.8 | Intersection doc | — | low | `WRO2026/Intersection.cpp`, `WRO2026/Intersection.h` | #26 correct stale "default=false" comments to "holds only with cache enabled (shipped true)". |
| T1.9 | Localization + EventTrace | — | high | `WRO2026/Localization.cpp`, `WRO2026/Localization.h`, `WRO2026/EventTrace.cpp`, `WRO2026/EventTrace.h` | #30 return last-good heading + cache accessor on non-OK read; #29 surface IMU result, return false on bad read in `zeroHeadingFromCurrent`; #28 serve heartbeat from cached heading (depends on #30's cache — order #30→#28); #27 drop dead `_prevIntEdgeCount` priming + fix docs. One commit each, #30 before #28. Behavioral firmware, compile-only — Phase-3 bench verifies. |
| T1.10 | FieldMap bounds | — | high | `WRO2026/FieldMap.h` | #33 bounds-check `getNextHop`/`getEdge`/`getEdgeType`/`getNode`/`getNodeType` (return safe sentinel on OOB) — covers #31. **No DECISIONS.md/ADR edit here** (ADR-028 file + index land in PR-2/T2.1). Compile-only; header-only change. |
| T1.11 | README test count | — | low | `WRO2026/test/README.md` | #12 update to the 7 registered ctest cases + correct expected output. |
| T1.12 | PR-1 assembly | T1.1–T1.11 | low | — | Open PR-1 off main; run `uv run pytest tools/path_planner/tests/` green; confirm AC2 (no #35/#37 file overlap); push. |

### PR-2 — pipeline core (off post-#37 `main`; gate: #37 merged).

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T2.1 | validator anti-ambiguity + ADRs 028/029 | #37 & PR-1 merged | med | `tools/path_planner/validator.py`, `tools/path_planner/planner.py`, `tools/path_planner/build_routes.py`, `docs/decisions/028-fieldmap-accessor-bounds.md`, `docs/decisions/029-validator-anti-ambiguity.md`, `DECISIONS.md` | #36 move pivot out-degree check into validator (remove from planner.apply_pivot_compensations — touches planner.py, so blocks T2.2 — and drop the uncaught path in build_routes.py); #38+#47 EDGES-vs-LINE/BLIND cross-check (one commit); #46 require finite in `_is_number`; #49 reject unsafe node names. Write ADR-028 (FieldMap bounds, documenting PR-1's #33) + ADR-029 (validator anti-ambiguity); index both in DECISIONS.md. TC4, TC5. |
| T2.2 | planner pivot compensation | T2.1 | med | `tools/path_planner/planner.py` | #42 carry compensated distance to the emitted graph (single graph for cost+emit) — emitter change, regen in-commit; then #94 guard against double-applying compensation (after #42). TC9. Blocked by T2.1 (shared planner.py). |
| T2.3 | compiler fixes | #37 merged | med | `tools/path_planner/compiler.py` | #44 derive end pose from `compile_path` resolution (REAR +180) — emitter change, regen in-commit; #45 widen `Primitive` type alias to real tuple shapes. TC8. |
| T2.4 | header_writer fixes | #37 merged | med | `tools/path_planner/header_writer.py` | #48 generate C++ enum bodies from the Python enums — emitter change, regen in-commit; #50 assert `n_nodes < 255` in `emit_routes`. |
| T2.5 | pipeline_config guard | #37 merged | low | `tools/path_planner/pipeline_config.py` | #51 reject self-referential `*.DEFAULT` on category fields. TC6. |
| T2.6 | sim runtime mode names | #37 merged | low | `tools/path_planner/sim/runtime.py` | #41 accept string mode names via `enum_cls[name]` in `_resolve_override`. TC7. |
| T2.7 | regen + goldens + parity test | T2.2, T2.3, T2.4 | med | `WRO2026/routes.h`, `WRO2026/routes.cpp`, `tools/path_planner/tests/golden/routes_toy.*`, `tools/path_planner/tests/test_header_writer.py`, new parity test file | Run `uv run python tools/path_planner/build_routes.py`; regenerate toy goldens; #72 document the regenerate-golden workflow near the test; #34 add a cross-language parity golden (Python `resolve_direction` vs recorded C++ `resolveDirection`). Verify `git diff --exit-code` clean. TC10. |
| T2.8 | PR-2 assembly | T2.1–T2.7 | low | — | Open PR-2 off post-#37 main; pytest green; emitter commits carry regenerated artifacts; push. |

### PR-3 — harness enforcement (off post-#37 `main`; gate: #37 merged).

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T3.1 | ctest CI job + ADR-030 | #37 merged | med | `.github/workflows/ci.yml`, `docs/decisions/0NN-harness-enforced.md`, `DECISIONS.md` | #75 add a CI job that cmake-configures `WRO2026/test/` and runs ctest. Write ADR-030 (rule: the C++ host-compile harness links real sources and is CI-enforced). TC14. |
| T3.2 | CMake harness fixes | #37 merged | low | `WRO2026/test/CMakeLists.txt` | #76 add `PASS_REGULAR_EXPRESSION` on the overflow WILL_FAIL test; #80 add `routes.cpp` (or a toy routes def) to the bidirectional target so future fixtures link. TC15. |
| T3.3 | promote bidirectional guard | #37 merged | med | `WRO2026/PathTracker.cpp`, `WRO2026/PathTracker.h`, `WRO2026/test/test_path_tracker_bidirectional.cpp` | #77 extract `_dispatchEdgeAfterTurn` REVERSE_UNAVAILABLE guard to a shared header/function so test and production link the same symbol (remove the hand-copied replica). |
| T3.4 | PR-3 assembly | T3.1–T3.3 | low | — | Open PR-3; CI ctest job green; merge before PR-4 starts. |

### PR-4 — firmware (off post-#37 `main`; gate: **PR-3 merged**). T4.1–T4.7 parallel (disjoint files); T4.8 last.

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T4.1 | Mission fixes | PR-3 merged | med | `WRO2026/Mission.cpp`, `WRO2026/Mission.h` | #1 set `_lastPushOk = _push(s)` in `task()`; #2 LED `setColor(1,..)`+`setColor(2,..)` (drop invalid index 0); #6 clamp beep duration to uint16 ceiling at the verb. |
| T4.2 | Drive fixes | PR-3 merged | high | `WRO2026/Drive.cpp` | #13 brake on profile completion when queue empty & endSpeed!=0; #14 clamp SYNC/GYRO+continuous power via deadbandSnap; #15 run triangle test before half-caps; #16 validate `accel>0` at the builder. Compile-only; behavior to Phase-3 bench. |
| T4.3 | PathTracker fixes | PR-3 merged | high | `WRO2026/PathTracker.cpp`, `WRO2026/PathTracker.h` | #32 clear `_passThrough` on the turn-required branch (covers #90); #91 move the no-op fault guard to the top of `_applyGoToBuilder` so a latched fault is preserved. |
| T4.4 | WROMath clamp | PR-3 merged | low | `WRO2026/WROMath.h` | #35 clamp `cmToEncDeg` result to `UINT16_MAX` (mirror the negative guard). |
| T4.5 | CRAWL_PWM | PR-3 merged | low | `WRO2026/Config.h`, `WRO2026/LineFollower.cpp` | #21 add explicit `CRAWL_PWM` constant in Config; use it in the lost-line CRAWL branch instead of `cruise/2`. |
| T4.6 | .ino menu/button | PR-3 merged | med | `WRO2026/WRO2026.ino` | #4 gate EventTrace render while cal menu open (mark clean on entry / suppress while OPEN); #5 wait for both buttons released on menu entry. |
| T4.7 | Tasks comment | PR-3 merged | low | `WRO2026/Tasks.cpp` | #8 add a one-line comment at `isAborted()` pointing to the intended first consumer (staged hook, not enforced). |
| T4.8 | link real sources | T4.1, T4.3 | high | `WRO2026/test/CMakeLists.txt`, `WRO2026/test/test_tasks_dispatch.cpp` | #78 link real `Tasks.cpp`/`Mission.cpp` (mocked hardware) so TC1..TC15 run against production; #79 comment that the union-punning TC assumes a little-endian GCC/clang host. Blocked by T4.1/T4.3 so real-linked tests pass. TC16. |
| T4.9 | PR-4 assembly | T4.1–T4.8 | med | — | Open PR-4 off post-#37 main, after PR-3 merged; CI (pytest + arduino-compile + ctest) green; push. |

## Notes for Implementer

- **Verify overlap before every PR.** Re-run `gh pr diff 35 --name-only` and `gh pr diff 37 --name-only` at the time you branch; the open PRs may change. PR-1's invariant is "touches neither set."
- **Commit voice:** mirror the repo's Conventional-Commit style (`git log -20 --oneline`), cite the trace finding number, no AI attribution, no banned words.
- **Coupled-pair atomicity:** #38+#47 = one commit. #32's commit message notes it resolves #90. #31 gets no commit (covered by #33 — note this in #33's message). #42 must precede #94. #30 must precede #28.
- **Regen discipline:** after any commit to `compiler.py`/`header_writer.py`/`planner.py` emission paths, run `uv run python tools/path_planner/build_routes.py` and regenerate `tests/golden/routes_toy.*` in the *same* commit. CI fails otherwise.
- **ADR numbering** is provisional (028/029/030). At execution, read the target branch's `DECISIONS.md`, take the next free numbers in order, and update the index + status table.
- **Sequencing reality:** PR-1 can open immediately. PR-2/PR-3 wait for #37 to merge (they retarget to post-#37 main). PR-4 waits for PR-3 (so firmware lands under the ctest gate). If #35/#37 stall, PR-1 still delivers value alone.
- **Rollback:** each finding is one commit; `git revert <sha>` backs out a single fix. Each PR is independently revertible.
- **No local C++ compiler** — do not attempt to build firmware locally; rely on CI. Python is fully local-verifiable; pre-run pytest before pushing PR-1 and PR-2.
