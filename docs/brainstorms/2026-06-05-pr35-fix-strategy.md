# Brainstorm: PR #35 trace-findings fix strategy

**Seed (verbatim):** how do we fix. Should we plan a super big fix or we cut it into different parts?

**Context summary:**
- `docs/PR35_TRACE_FINDINGS.md` catalogs 94 adversarially-verified findings from a whole-repo trace
  surfaced by PR #35 (Tasks framework + `Mission.task()`). Severity split: bugs / suspicious / possible.
  Scope split: ~12 `PR-surface` (PR #35's changed files) vs the rest `pre-existing` (Drive, LineFollower,
  PathTracker, WROLineArray, offline `tools/path_planner/*`, editor GUI, tests).
- Diego has already filled in a **Decision:** line on every finding. Buckets observed:
  - **Fix now** (A/B with a real code change): the large majority — spans firmware (Drive, LineFollower,
    EventTrace, Localization, FieldMap, PathTracker, WROMath, Config), the offline Python pipeline
    (validator, graph_builder, planner, compiler, header_writer, pipeline_config, visualizer, pick_xy),
    the editor GUI, and tests/CI (#71-73, #75-78, #80).
  - **Do nothing / leave as-is:** #3, #7, #9, #10, #11, #79, #92.
  - **Defer:** #22, #23, #24, #25, #54, #55, #68, #69, #70, #89.
  - **Verify in Phase-3 bench (NOT coded now):** #74, #81-88, #93 (folded into the bench checklist table
    at the bottom of the doc).
- Two languages, many modules, mix of one-liners + docs-only + behavior-changing fixes. Several fixes are
  coupled (#31→#33, #38↔#47, #32↔#90, #75↔#78, #42↔#94).
- mem0: Diego prefers a centralized abstraction when a roadmap item shares the plumbing; matches task
  granularity to complexity (no over-decomposition); source-cited proof for load-bearing API decisions;
  fluent C++ DSL over JSON. No prior memory on this specific fix-rollout question.

**Clarified framing:** The per-finding *decisions* are already made — this is purely a **rollout/packaging
strategy** question for the ~60 accepted "fix now" findings (spanning R4 firmware, the offline Python
pipeline, the editor GUI, and tests/CI). Constraints from Diego:
1. **Scope = ALL fix-now decisions** (defer only the defer/bench-verify/do-nothing buckets).
2. **Branching:** merge PR #35 first, then do the fixes as **new PRs off main** — NOT folded into #35.
   **Caveat: another PR is already waiting/in-flight**, so sequencing must avoid conflicting with it.
3. **Priority = speed to done** — fewest round-trips, batch aggressively.
Environment: Diego works solo on Windows; **no host C++ compiler locally** (firmware only compiles in CI);
plan→execute with subagents; feature branches; human-voice commits (no AI attribution).

## Perspectives (round 1)

**DECISIVE DISCOVERY (all three agents, grounded via `gh`):** the in-flight "other PR" is **PR #37
`feat/stop-at-line` (+4487/−84), based on `feat/tasks-framework` (the #35 branch), NOT main.** When #35
merges, GitHub auto-retargets #37 to main. #37's changed files overlap the fix set in **13+ files**:
`Drive.cpp/.h`, `Mission.cpp/.h`, `PathTracker.cpp/.h`, `WROLineArray.h`, `WROMath.h`, `Config.h`,
`compiler.py`, `header_writer.py`, `pipeline_config.py`, `sim/runtime.py`, `validator.py`,
`test/CMakeLists.txt`, golden `routes_toy.*`. So a "batch off main" plan collides with #37 on the same hunks.

### Critic
1. The other PR (#37) IS the fix work's twin and collides on nearly every hot file → batching off main forces
   a three-way rebase war on files Diego can't compile-test locally. The dominant cost, not a footnote.
2. ~60 behavior-changing firmware edits with NO verifier in the loop: no host compiler, CI runs no ctest
   (#75/#78), no hardware until Phase 3. arduino-cli catches syntax/link, not semantic motion/nav bugs.
3. A big batch destroys bisectability exactly at the Phase-3 bench — a regression won't isolate to a commit,
   and deferred bench fixes (#81-88) land on top, so a symptom has two possible origins.
4. Mixing docs/one-liners (#12,#26,#35,#50) with dangerous flips (#13 runaway brake, #44 REAR 180°, #42 pivot)
   in one batch anaesthetizes solo review — the trivial volume hides the high-risk hunks.
5. Couplings (#31→#33, #38↔#47, #32↔#90, #42↔#94, #28→#30, #75↔#78) impose ordering that "batch by file/ship
   fast" actively fights — landing half a coupling produces wrong fixes (#94 before #42; #28 before #30).
6. #75/#78 (wire ctest into CI, link real Tasks/Mission) should land FIRST so the firmware batch is enforced;
   speed-first batching that defers them ships ~30 firmware fixes green against mirror-only tests.
7. "Fewest round-trips" is false economy when reviewer=author=conflict-resolver=one person with no compiler;
   binding constraint is human verification bandwidth + merge surface, both worse-than-linear in batch size.

### Feasibility — VERDICT: Conditional
- Must be true: #35 merges first; #37 sequenced deliberately; every fix PR gated on green CI; #75 lands in/
  before the firmware batch; regen step (`build_routes.py` + toy goldens) runs after any emitter-changing fix.
- No hard blockers (#37 is currently clean; base auto-retargets on #35 merge).
- Clean partition into ~4 PRs: **B1 offline editor/pick_xy/visualizer** (#52,53,56-67 — fully pytest-
  verifiable, ZERO overlap with #37 → land FIRST, in parallel with #37 in flight); **B2 pipeline core**
  (#36-51,71-73,94 — overlaps #37 → land AFTER #37); **B3 firmware** (#1,2,4,5,6,8,13-35,91 — overlaps #37 →
  land AFTER #37, gated by #75); **B4 test/CI** (#75,76,77,78,80 + #12,#34 — #75 first to enforce B3).
- Counts: ~33 pure-Python (CI-testable now), ~24 firmware (compile-only in CI), rest docs/harness.
- Subagents partitioned BY FILE not by finding (Drive agent owns all Drive findings, etc.).

### Thinker (branches)
1. **Layer-cut: Python-first, firmware-second** — land the locally-`pytest`-verifiable offline half fast;
   firmware (CI-compile-only) churns in parallel.
2. **CI harness as the literal first commit** — make #75(+76/77/78/80) PR-0; every later firmware fix is then
   enforced. Turns "no local compiler" into structure (harness = compiler-by-proxy).
3. **Coupling-graph cut: one PR per fix-cluster** — package by the doc's named edges, not by layer.
4. **Risk-tiered: trivial-sweep mega-PR + behavior-change individual PRs** — split on blast radius; batch the
   near-unreviewable-as-wrong one-liners maximally, isolate the genuinely behavioral fixes.
5. **Findings-doc-as-ledger + parallel subagent fan-out** — append a `Status:` token per Decision line;
   one execute-agent per disjoint module bucket; doc is both spec and done-tracker.
6. **In-flight-PR-aware staging by file disjointness** — start with `tools/path_planner/`-only fixes that
   can't intersect #37; hold firmware until #37 merges, rebase once. Makes #37 the scheduling pivot.
7. **Self-verifying-fix discipline** — no behavior fix merges without a failing→passing test (pytest / ctest).
8. **One squashed mega-PR, one commit per finding** (speculative) — maximal batch, per-commit changelog.
9. **Two-track by CI job: pipeline-track vs sketch-track concurrently** (speculative) — exploit CI's two jobs.
10. **ADR-anchored: write the cross-module ADRs first** (speculative) — #33/#31, #14/#85, #38/#47/#51, #75/#78.

## Live direction
Converging: **cut into parts, sequenced around PR #37** = B1 (offline/editor, no #37 overlap) + B4-#75
(CI harness) land NOW in parallel with #37 → #37 merges → B3 firmware + B2 pipeline rebase onto post-#37 main.
Blends Thinker #1+#2+#6 and Feasibility's batch plan. Still serves "speed to done" (B1 proceeds immediately,
no rebase war). <pending Diego's gate>

## Killed / parked

## Decisions & debate verdicts
- **Fork: one big comprehensive PR vs ~4 batched PRs sequenced around #37 (speed-to-done).**
  → `/debate` verdict: **Lean No on the monolith / prefer batching (4.25/10).**
  - Key resolution: the monolith's "one rebase past #37" advantage dissolves — sequencing the batches
    around #37 avoids the rebase in both plans. Counter-resolution (favoring monolith): per-finding revert +
    1:1 changelog come from *commit* boundaries, not PR boundaries → carry one-commit-per-finding discipline
    INSIDE each batch.
  - Batching wins on: bank the zero-overlap offline fixes now (in parallel with #37 in flight); land #75/#78
    to enforce firmware before firmware fixes; all-or-nothing CI avoided; Phase-3 bench stays bisectable.
