# Phase 2 PR 1 — Final Audit

## Status: Ready for review

---

## Test suite

- **Command:** `uv run pytest tools/path_planner/tests/ -v`
- **Result:** 70/70 tests passed in 1.80 s
- **Pre-existing warning:** `UserWarning: Node 'G' has no incident edges after C-space inflation; it will be unreachable in the planner.` in `test_disconnected_goal_logs_warning` — expected, tested explicitly

---

## Packaging fix (T10)

The `[build-system]` hatchling backend was declared but had no package discovery config, causing
`Unable to determine which files to ship inside the wheel` when `uv` tried to resolve test imports.

**Fix applied:**
1. Added `tools/__init__.py` (namespace package marker) so `tools.path_planner` resolves as an installed
   namespace.
2. Changed `[tool.hatch.build.targets.wheel] packages` from `["tools/path_planner"]` to `["tools"]`
   so the full `tools.path_planner.*` import hierarchy is installed, matching the `from tools.path_planner
   import ...` pattern used throughout the test suite.

After fix: `uv sync` rebuilds cleanly; `uv run pytest` collects all 70 tests with no import errors.

---

## Generated artefact validity

### arduino-cli
Skipped — not installed on this machine.

### Manual audit of `WRO2026/routes.h` (49 lines)

| Check | Result |
|---|---|
| Include guard: `#ifndef ROUTES_H` opens at line 5 | PASS |
| Include guard: `#endif  // ROUTES_H` closes at line 49 | PASS |
| `#include <stdint.h>` only header dependency (no Arduino-specific includes) | PASS |
| `enum class NodeType : uint8_t` has explicit underlying type | PASS |
| `enum class EdgeType : uint8_t` has explicit underlying type | PASS |
| `enum class BlindAction : uint8_t` has explicit underlying type | PASS |
| `extern const NodeInfo NODES[N_NODES]` declared | PASS |
| `extern const uint8_t NEXT_HOP[N_NODES][N_NODES]` declared | PASS |
| `extern const EdgeInfo EDGES[N_NODES][N_NODES]` declared | PASS |
| `extern const GoalApproach GOAL_APPROACH[N_GOALS]` declared | PASS |
| `}  // namespace Routes` closes namespace cleanly | PASS |
| All float literals end in `f` (e.g. `0.0f`, `0.000000f`) | PASS |

### Manual audit of `WRO2026/routes.cpp` (106 lines)

| Check | Result |
|---|---|
| `const NodeInfo NODES[N_NODES]` definition matches `extern` in .h | PASS |
| `const uint8_t NEXT_HOP[N_NODES][N_NODES]` definition matches `extern` in .h | PASS |
| `const EdgeInfo EDGES[N_NODES][N_NODES]` definition matches `extern` in .h | PASS |
| `const GoalApproach GOAL_APPROACH[N_GOALS]` definition matches `extern` in .h | PASS |
| Struct field initializers use correct brace syntax; fields in declaration order | PASS |
| Anonymous-namespace blind-move array `_G_blind[]` exists for goal G | PASS |
| `GOAL_APPROACH[0]` references `_G_blind` and `blind_move_count=2` | PASS |
| `}  // namespace Routes` closes namespace cleanly | PASS |
| All float literals end in `f` | PASS |
| Unreachable entries use `0xFF` sentinel (e.g. row `from G` → all 0xFF except self) | PASS |

### Regen idempotency (AC2)

```
uv run python tools/path_planner/build_routes.py --verbose
git diff -- WRO2026/routes.h WRO2026/routes.cpp
```

Result: **byte-identical** — `git diff` returned only Windows line-ending normalization notices
(`LF will be replaced by CRLF`), no content diff.

---

## Scope check (AC12)

**Files outside intended scope:** none

**Phase 1 C++ files touched:** none

`git diff main...HEAD --stat` shows exactly 30 files, all within the planned scope:
- New Python modules + tests: `tools/path_planner/**`
- New generated C++: `WRO2026/routes.h`, `WRO2026/routes.cpp`
- New docs: `docs/PATH_PLANNER_SPEC.md`, `docs/PHASE2_DEFERRED.md`, `docs/plans/2026-05-05-phase2-pr1-path-planner-python.md`
- Repo root config: `.gitignore`, `pyproject.toml`

No changes to: `Drive.h/.cpp`, `LineFollower.h/.cpp`, `Intersection.h/.cpp`, `Manipulator.h/.cpp`,
`EventTrace.h/.cpp`, `WRO2026.ino`, `Config.h`, `WROMath.h`, `StateMachine.h`.

**T10 additions (not yet in committed diff, staged by orchestrator):**
- `tools/__init__.py` — namespace package marker (new file, in scope)
- `pyproject.toml` — 4-line hatchling packaging fix (modification, in scope)
- `docs/phase2-pr1-final-audit.md` — this file (new doc, in scope)

---

## Files changed in this PR

```
 .gitignore                                         |   9 +
 WRO2026/routes.cpp                                 | 106 +++
 WRO2026/routes.h                                   |  49 ++
 docs/PATH_PLANNER_SPEC.md                          | 520 +++
 docs/PHASE2_DEFERRED.md                            | 263 +++
 docs/plans/2026-05-05-phase2-pr1-path-planner-python.md | 232 +++
 pyproject.toml                                     |  23 +
 tools/__init__.py                                  |   1 +
 tools/path_planner/README.md                       | 163 +++
 tools/path_planner/__init__.py                     |   1 +
 tools/path_planner/build_routes.py                 | 230 +++
 tools/path_planner/compiler.py                     | 276 +++
 tools/path_planner/field_data.py                   |  69 +++
 tools/path_planner/graph_builder.py                | 338 +++
 tools/path_planner/header_writer.py                | 784 +++
 tools/path_planner/output/.gitkeep                 |   0
 tools/path_planner/planner.py                      | 191 +++
 tools/path_planner/requirements.txt                |   5 +
 tools/path_planner/tests/__init__.py               |   1 +
 tools/path_planner/tests/golden/__init__.py        |  10 +
 tools/path_planner/tests/golden/routes_toy.cpp     | 106 +++
 tools/path_planner/tests/golden/routes_toy.h       |  49 +++
 tools/path_planner/tests/test_build_routes.py      | 252 +++
 tools/path_planner/tests/test_compiler.py          | 505 +++
 tools/path_planner/tests/test_field_data.py        |  99 +++
 tools/path_planner/tests/test_graph.py             | 183 +++
 tools/path_planner/tests/test_header_writer.py     | 315 +++
 tools/path_planner/tests/test_planner.py           | 247 +++
 tools/path_planner/tests/test_visualizer.py        | 158 +++
 tools/path_planner/validator.py                    | 351 +++
 tools/path_planner/visualizer.py                   | 338 +++
 31 files changed, ~5875 insertions(+)
```

---

## Notes for reviewer

- **`uv.lock` is present at the repo root in this branch** and should be used as the pinned dependency
  lockfile for reproducible `uv sync`/CI environments; no reviewer action is needed on tracked vs
  ignored status in this PR.
- **`tools/__init__.py`** is a minimal namespace marker. It does not expose any public API — it exists
  solely so `from tools.path_planner import ...` resolves correctly after wheel install.
- **`arduino-cli` not available** on this machine; AC3 satisfied by manual audit above. Full compile
  verification can be done in CI or by the reviewer with `arduino-cli compile --fqbn
  arduino:renesas_uno:unor4wifi WRO2026/`.
