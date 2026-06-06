# ADR-030: The host-compile harness links the real production symbols, CI-enforced

**Status:** Partially implemented (CI gate is live, and `LineFollower` / `DirectionPolicy.resolveDirection` / the reverse-availability guard are linked from production; real `Tasks.cpp` / `Mission.cpp` linking is scheduled for PR-4, finding #78)
**Date decided:** 2026-06 (PR-35 trace-findings fix rollout)

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
- **Remaining gap:** `WRO2026/test/test_tasks_dispatch.cpp` still exercises
  `TasksMirror` / `MissionMirror` inline reimplementations rather than the real
  `Tasks.cpp` / `Mission.cpp`. PR-4 (finding #78) will link the production
  sources and delete those mirrors; until then Tasks/Mission dispatch is tested
  against mirrors and this rule is only partially in force.

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
- Findings #75 (CI gate), #77 (`reverseUnavailable` promoted), #78 (real `Tasks`/`Mission` linking, PR-4).
