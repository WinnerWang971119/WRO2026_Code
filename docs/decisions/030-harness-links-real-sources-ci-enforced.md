# ADR-030: The host-compile harness links the real production symbols, CI-enforced

**Status:** Implemented (CI gate is live; `LineFollower`, `DirectionPolicy.resolveDirection`, the reverse-availability guard, real `Tasks.cpp` (#46 Phase 1), real `Mission.cpp` (#46 Phase 2), and real `PathTracker.cpp` / `Drive.cpp` (#106) are all linked from production. Two narrowly-scoped FSM mirror slices remain in `test_path_tracker_reset_arrival` + `test_return_to_node_dispatch`, tracked as follow-up #113 — see How to apply.)
**Date decided:** 2026-06 (PR-35 trace-findings fix rollout)
**Implemented:** 2026-06 (#106 — real `PathTracker.cpp` / `Drive.cpp` host-link)

## Rule

A C++ host test under `WRO2026/test/` asserts against the **same symbol the
firmware compiles**, not a hand-copied replica of it. When a piece of logic must
be exercised on a headless host, promote it to a header-only `inline` free
function (the `DirectionPolicy.h` pattern) and `#include` it from both the
production translation unit and the test — do not byte-copy it into the test.

This is CI-enforced. The `host-tests` job (name "Host tests") in
`.github/workflows/ci.yml` cmake-configures `WRO2026/test`, builds the harness,
and runs `ctest` on every push to `main` and every pull request. A new test
target must be reachable by that job to count.

## Why

A host-compile harness that asserts against a hand-copied replica can pass green
while production drifts: the replica freezes the logic at copy time, so a real
regression in the firmware symbol is invisible to the test. Linking the actual
production symbol makes the test fail the moment production changes behavior,
which is the only thing that makes the harness worth running.

A harness that nothing ever compiles is decorative. Local-only host tests rot
because no gate forces them to keep building. Wiring `ctest` into CI on every
push and PR turns the harness from documentation into an enforced contract.

## How to apply

- **New shared logic:** factor it into a header-only `inline` like
  `WRO2026/DirectionPolicy.h::resolveDirection` (the precedent — it replaced the
  former `replica_resolveDirection` byte-copy) or `reverseUnavailable` in the
  same header (finding #77, this wave). Production `.cpp` and the test both
  `#include` it, so there is one definition to drift from.
- **New host test:** add the target to `WRO2026/test/CMakeLists.txt` so the
  `host-tests` ctest job picks it up. A test the job cannot reach is not enforced.
- **Tasks / Mission:** real `Tasks.cpp` is linked by `test_tasks_dispatch` (#46
  Phase 1) and real `Mission.cpp` by `test_mission_dispatch` (#46 Phase 2), both
  via the shared `hal_mocks/` HAL and per-target mocks; the `TasksMirror` /
  `MissionMirror` inline replicas are deleted.
- **PathTracker / Drive (#106):** the five named mirror tests —
  `test_path_tracker_passthrough`, `_pass_through_projection`, `_projection`,
  `_stop_at_line`, `_bidirectional` — now host-link the real `PathTracker.cpp` +
  `Drive.cpp` (plus real `Localization.cpp` / `YawSource.cpp`), so their per-edge
  FSM and Drive builder/queue/profile logic is asserted against production code in
  CI. Every `// MIRROR:` slice in those five is deleted. The trust boundary
  inverts vs `test_mission_dispatch`: these targets do NOT force-include a `Drive`
  or `PathTracker` mock (the real headers win); they mock only the `MatrixMiniR4`
  facade + the line-sensor leaf + the upper peripherals (`Intersection` /
  `LineFollower`) via `path_mocks/`, and link a dedicated `path_mocks/path_mocks.cpp`
  for facade globals (NOT `hal_mocks.cpp`, which defines a mock `Drive` that would
  collide with the real one).
- **Remaining FSM mirrors (follow-up):** `test_path_tracker_reset_arrival` +
  `test_return_to_node_dispatch` STILL carry narrowly-scoped PathTracker FSM
  `// MIRROR:` slices (over real `Localization` + the fixture routes). They were
  out of #106's named five; de-mirroring them is a tracked fast-follow now that
  real `PathTracker.cpp` is host-linkable (#113). So the rule
  is in force for all new and rewritten targets, with these two slices the last
  known exception.

## Trigger to revisit

- The harness grows a dependency that cannot be mocked on a headless host (e.g.
  a symbol that pulls in `MatrixMiniR4` hardware state that has no mock surface)
  — a narrowly-scoped mirror may be the lesser evil, documented as such.
- A test genuinely needs to diverge from production (e.g. asserting a property
  the production symbol deliberately does not expose) — record why the divergence
  is intentional rather than letting a silent replica creep back in.

## See also

- [ADR-024 — Bidirectional line following (`resolveDirection` shared-symbol precedent)](./024-bidirectional-line-following.md)
- `.github/workflows/ci.yml` — the `host-tests` job (added by PR #41, finding #75).
- `WRO2026/DirectionPolicy.h` — the header-only `inline` shared-symbol pattern.
- `WRO2026/test/CMakeLists.txt` — the ctest target list the job runs.
- `WRO2026/test/test_mission_dispatch.cpp` — the Phase 2 target linking real `Mission.cpp`.
- Findings #75 (CI gate), #77 (`reverseUnavailable` promoted), #78 (real `Tasks`/`Mission` linking, landed #46); issue #106 (`PathTracker`/`Drive` host-link, landed — the five named mirror tests now link real sources; `reset_arrival` + `return_to_node_dispatch` FSM slices tracked as #113).
