# CTPS support for direct `Localization.declarePosition()` Plan

**Goal:** Make a **direct** `Localization.declarePosition(...)` call show up in the CTPS dump so the player snaps the robot to the declared node — closing the gap where only `Mission.declarePosition(...)` was recorded.

**Approach:** Move the CTPS recording seam down to the `Localization` facade. The CTPS mock's `declarePosition(uint8_t)` becomes the single recording funnel (resolving id→name via `FieldMap`), the `declarePosition(const char*)` overload delegates to it (mirroring real `Localization.cpp`), and the now-redundant `#ifdef CTPS_RECORD` hook in `Mission.cpp` is removed so the Mission path still records exactly once. No Python/player changes — the existing `declare_position` op + `track_builder` belief-reset already snap the robot to the node.

## Scope

- **In scope:**
  - CTPS mock `WRO2026/test/ctps_mocks/Localization.h`: record on **both** `declarePosition` overloads (uint8 funnel + name delegates), mirroring real firmware semantics.
  - Remove the `Mission.cpp` `#ifdef CTPS_RECORD` `recordDeclarePosition` hook (single-seam; avoids double-count).
  - Docstring updates on the mock and on `ctps::recordDeclarePosition` in `ctps_record.h`.
  - One new host-ctest case (in the existing `ctps_dump_mock` target) proving the emit.
- **Out of scope:**
  - Any Python change (`trace_loader.py`, `track_builder.py`, renderer, goldens) — the `declare_position` op is already fully supported end-to-end.
  - Any new trace field/label distinguishing "operator-declared" vs "mission-declared" (user chose: same marker, just plug the hole).
  - Changing the recorder entry point `ctps::recordDeclarePosition` or the JSON wire shape (`{"op":"declare_position","node":<name>}`) — unchanged.
  - Firmware behavior of `Localization`/`Mission` on the real robot (the removed hook is `#ifdef CTPS_RECORD`, compiled out of the sketch — zero firmware effect).

## Decisions

- **Single recording seam at the `Localization` mock, not the Mission layer** — Capturing **both** overloads (user requirement) means recording at the `uint8_t` funnel. Since `Mission.declarePosition(name)` internally calls `Localization.declarePosition(id)` (`Mission.cpp:253`), keeping the Mission hook too would double-record. So the Mission hook is removed. This mirrors the repo's existing precedent (goto is recorded by the `PathTracker.goTo` mock only, never also by `Mission.cpp`).
- **The seam move lands as ONE atomic task/commit** — recording-in-mock (`Localization.h`) and hook-removal (`Mission.cpp`) are two halves of one seam change. Splitting them across commits leaves a bad intermediate state either way (hook-first → Mission-path declares record *nowhere*; mock-first → Mission-path *double*-records), and no self-test would catch it. They ship together (task T1).
- **uint8 overload resolves id→name via `FieldMap::getNode(id).name`** — the wire op carries a node **name**; the uint8 path only has an id. `routes.cpp`/`routes_fixture.cpp` is linked into every affected target, so `FieldMap` resolves.
- **Unknown node records nothing** (user choice, mirrors firmware) — the name overload returns `false` and records nothing when `findNodeByName` yields `NODE_NONE`; the uint8 overload records only when `id < Routes::N_NODES` (the single canonical guard — `FieldMap::getNode` returns the empty-name sentinel exactly when the id is out of range).
- **Test reuses the `ctps_dump_mock` ctest** (`test_ctps_dump.cpp`) — it already force-includes the CTPS mock `Localization`, links `routes_fixture.cpp`, includes `ctps_record.h` (`reset()`/`g_ctpsLog`), and builds with `CTPS_RECORD`. No new CMake target. Anti-drift coverage for the two mock overloads is added to the existing `test_ctps_mock_conformance` TU (which today only smoke-calls `recordDeclarePosition`).
- **Only one reasonable approach** — the "additive, name-overload-only" alternative was ruled out by the "capture both overloads" requirement (it can't see a direct `uint8_t` call), so Phase 4 presents no fork.

## Acceptance Criteria

- [ ] AC1: In the CTPS dump, a direct `Localization.declarePosition("Inter1")` (name overload) records exactly one `Op::DECLARE_POSITION` with `target == "Inter1"`, and the name overload returns `true`. (Test: TC1)
- [ ] AC2: A direct `Localization.declarePosition(id)` (uint8 overload) for a valid `id` records exactly one `Op::DECLARE_POSITION` whose `target` equals `FieldMap::getNode(id).name`. (Test: TC2)
- [ ] AC3: `Mission.declarePosition("X")` still yields exactly **one** `declare_position` op (no double-count) after the seam move. **Review-verified, not test-verified** — `test_ctps_dump.cpp` does not link `Mission.cpp` and `ctps_dump --self-test` never calls `declarePosition`, so this is guaranteed by construction (T1 is atomic: mock records once per call + Mission hook removed) and confirmed by code inspection, not an automated assertion.
- [ ] AC4: An unknown node name records **nothing** and the name overload returns `false`; an out-of-range `id` (`>= Routes::N_NODES`) records **nothing**. (Tests: TC3 name path, TC5 uint8 path)
- [ ] AC5: `commandStreamToJson(...)` serializes the recorded op as `{"op": "declare_position", "node": "<name>"}` (unchanged wire shape; loadable by the existing `trace_loader.py`). (Test: TC4)
- [ ] AC6: Both mock overloads are chained in `test_ctps_mock_conformance.cpp` so a future real-vs-mock signature drift fails the build. (Test: TC7)
- [ ] AC7: Full host `ctest` passes — including all three targets that force-include the mock (`ctps_dump`, `test_ctps_mock_conformance`, `test_ctps_dump`) — and `arduino-cli compile` still succeeds. (Test: TC6)

## Contracts & Interfaces

Single source of truth for the seams this change touches.

### CTPS mock `Localization` (owner: T1) — `WRO2026/test/ctps_mocks/Localization.h`

Both methods stay **header-inline** (no new translation unit), matching the existing no-op mock:

```cpp
class LocalizationClass
{
public:
    // Records the re-anchor (single recording funnel). Mirrors real Localization.cpp:
    // unconditional low-level write, but the dump records only when id < Routes::N_NODES
    // (FieldMap::getNode returns the empty-name sentinel exactly when it is out of range).
    void declarePosition(uint8_t nodeId)
    {
        if (nodeId < Routes::N_NODES)
        {
            ctps::recordDeclarePosition(FieldMap::getNode(nodeId).name);
        }
    }

    // Mirrors real: resolve name -> id; if NODE_NONE return false (record nothing);
    // else delegate to declarePosition(id) (which records) and return true.
    bool declarePosition(const char* nodeName)
    {
        const uint8_t id = FieldMap::findNodeByName(nodeName);
        if (id == FieldMap::NODE_NONE) { return false; }
        declarePosition(id);
        return true;
    }

    // UNCHANGED — identity world->IMU yaw in the CTPS frame.
    int16_t toImuYaw(float worldHeadingDeg) const { return (int16_t) worldHeadingDeg; }
};
extern LocalizationClass Localization;   // defined in ctps_record.cpp (unchanged)
```

New includes in the mock: `#include "src/nav/FieldMap.h"` (id→name + `Routes::N_NODES`, via `SKETCH_DIR`) and `#include "ctps_record.h"` (via `CTPS_MOCKS_DIR`). The sibling mock `ctps_mocks/PathTracker.h` already includes exactly these, so the include pattern is pre-proven for these targets. Rewrite the file/class docstring: it currently claims "both re-anchor forms are no-ops here" — replace with a statement that the mock records the re-anchor as a `declare_position` op via the single uint8 funnel.

### Recorder entry point (owner: T1) — `WRO2026/test/ctps_mocks/ctps_record.{h,cpp}`

- `void ctps::recordDeclarePosition(const char* name);` — **unchanged signature/behavior.** Appends one `Op::DECLARE_POSITION` with `target = name`. Serialized by `commandStreamToJson` as `"op": "declare_position", "node": <target>`. T1 only updates docs (no logic change):
  - the `recordDeclarePosition` Doxygen `@brief` in `ctps_record.h` — say it is invoked by the CTPS `Localization` mock funnel, not the `Mission.cpp` hook;
  - the file-level "Recording division (Option A)" map in `ctps_record.h` (lines ~22–35) — add a line noting `declare_position` is recorded by the `Localization` mock;
  - the stale `ctps_record.cpp` comment at ~lines 198–199 ("No-op Localization mock singleton (Mission.declarePosition target)") — the global is still defined here, but it is no longer a no-op.

### Mission seam (owner: T1) — `WRO2026/src/mission/Mission.cpp`

- Remove the block at ~`Mission.cpp:255-261`:
  ```cpp
  #ifdef CTPS_RECORD
      ctps::recordDeclarePosition(name);
  #endif
  ```
  The recording now happens one line earlier, inside `Localization.declarePosition(id)` (`Mission.cpp:253`) via the mock funnel. No other op is emitted between those lines, so stream order is unchanged. Also drop the now-unused `#include`/forward-decl of `ctps` in `Mission.cpp` **only if** removing the hook leaves it unreferenced (check the other `#ifdef CTPS_RECORD` hooks in that file first — they likely still use it, so keep it).

### Wire op (unchanged; consumed by existing Python)

- `{"op": "declare_position", "node": "<node name>"}` → `trace_loader.py` parses `node`; `track_builder.py` applies `sim.localization.declare_position(node)` (belief reset → robot snaps to node) + emits the localization-lane `declare_position` marker. No change.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/test/ctps_mocks/Localization.h` | T1 | T2 |
| `WRO2026/src/mission/Mission.cpp` | T1 | — |
| `WRO2026/test/ctps_mocks/ctps_record.h` | T1 | — |
| `WRO2026/test/ctps_mocks/ctps_record.cpp` | T1 | — |
| `WRO2026/test/test_ctps_dump.cpp` | T2 | — |
| `WRO2026/test/ctps_mocks/test_ctps_mock_conformance.cpp` | T2 | — |

## Error Handling

- **Unknown node name (name overload):** `findNodeByName` → `NODE_NONE` → return `false`, record nothing. (Matches real `Localization.cpp:204-214`.)
- **Out-of-range id (uint8 overload):** `FieldMap::getNode(id)` returns the safe sentinel (empty name); the mock records nothing when the resolved name is empty (or `id >= Routes::N_NODES`). Keeps the trace free of `"node":""` ops (which `trace_loader.py` would otherwise warn-and-drop).
- **Include/link fan-out:** the mock is force-included into 3 targets; T4 runs the full `ctest` so a compile break in any of them fails loudly rather than only in `ctps_dump`.

## Testing Strategy

**Levels:** Unit (host C++ ctest), plus full-suite + firmware-compile regression.

| ID  | Test Case | Type | File | Expected Behavior |
|-----|-----------|------|------|-------------------|
| TC1 | `Localization.declarePosition("Inter1")` (name overload), then scan `g_ctpsLog` | Unit | `test_ctps_dump.cpp` | Returns `true`; exactly one `Op::DECLARE_POSITION` with `target == "Inter1"` |
| TC2 | `id = FieldMap::findNodeByName("Corner"); Localization.declarePosition(id)` | Unit | `test_ctps_dump.cpp` | Exactly one `Op::DECLARE_POSITION` with `target == "Corner"` |
| TC3 | `Localization.declarePosition("NopeNode")` (unknown name) | Unit | `test_ctps_dump.cpp` | Returns `false`; `g_ctpsLog` gains **no** new record |
| TC4 | `commandStreamToJson("Start", 0.0)` after TC1 | Unit | `test_ctps_dump.cpp` | Output contains `"op": "declare_position"` and `"node": "Inter1"` |
| TC5 | `Localization.declarePosition(0xFF)` (out-of-range id) | Unit | `test_ctps_dump.cpp` | `g_ctpsLog` gains **no** new record (exercises the `id < Routes::N_NODES` guard) |
| TC6 | Full `ctest` across all targets that force-include the mock + `arduino-cli compile` of the sketch | Integration | — | All ctests pass (no fan-out compile break); firmware compiles (Mission hook was `#ifdef CTPS_RECORD`, compiled out — zero firmware delta) |
| TC7 | Chain both overloads on the mock in the conformance TU (`Localization.declarePosition((uint8_t)0); bool ok = Localization.declarePosition("N1");`) | Unit (compile) | `test_ctps_mock_conformance.cpp` | Target builds; a future real-vs-mock signature drift on either overload fails the build |

**Test data:** `ctps::reset()` before each case (as the existing `test_ctps_dump.cpp` cases do); node names come from `routes_fixture.cpp` (`"Inter1"` idx 16, `"Corner"` idx 7, no `"NopeNode"`). Use `ASSERT_TRUE`/`ASSERT_FALSE`/`ASSERT_EQ` from `TestAssert.h`. TC5 counts records before/after to assert no append.
**Run command:**
```
cmake -S WRO2026/test -B build/test && cmake --build build/test
ctest --test-dir build/test --output-on-failure
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/
```

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Move the recording seam (atomic) | — | med | `WRO2026/test/ctps_mocks/Localization.h`, `WRO2026/src/mission/Mission.cpp`, `WRO2026/test/ctps_mocks/ctps_record.h`, `WRO2026/test/ctps_mocks/ctps_record.cpp` | **(a) Mock** — implement both overloads per the Contracts snippet (header-inline): `declarePosition(uint8_t)` records `ctps::recordDeclarePosition(FieldMap::getNode(id).name)` only when `id < Routes::N_NODES`; `declarePosition(const char*)` resolves via `FieldMap::findNodeByName`, returns `false` + records nothing on `NODE_NONE`, else delegates to the uint8 overload and returns `true`. Add `#include "src/nav/FieldMap.h"` and `#include "ctps_record.h"`; keep `toImuYaw`; rewrite the file/class docstring (drop "both re-anchor forms are no-ops here"). **(b) Mission** — delete the `#ifdef CTPS_RECORD ... ctps::recordDeclarePosition(name); #endif` block at ~`Mission.cpp:255-261` (recording now happens via the mock funnel at line 253); keep the `ctps` include + the seven other `#ifdef CTPS_RECORD` hooks that still use it. **(c) Docs** — update the `recordDeclarePosition` `@brief` + the "Recording division (Option A)" map in `ctps_record.h`, and the stale no-op comment at `ctps_record.cpp:~198-199`. Ships as ONE commit (no bad intermediate state — see Decisions). Satisfies AC1, AC2, AC4; AC3 by construction. med risk = mock is force-included into 3 targets (include-order/fan-out). |
| T2 | Host tests for the emit + anti-drift | T1 | low | `WRO2026/test/test_ctps_dump.cpp`, `WRO2026/test/ctps_mocks/test_ctps_mock_conformance.cpp` | In `test_ctps_dump.cpp`: add a `test_*_declare_position_*` function wired into `main()` implementing TC1–TC5 (name overload records `"Inter1"` + returns true; uint8 overload records resolved name for `"Corner"`; unknown `"NopeNode"` → false + no record; `commandStreamToJson` emits `"op": "declare_position"` / `"node": "Inter1"`; `declarePosition(0xFF)` → no record). `ctps::reset()` per case; `TestAssert.h` macros; match the existing `test_TC14*` style (count `g_ctpsLog` before/after for the no-record cases). In `test_ctps_mock_conformance.cpp`: add a chained-call block for both `Localization` overloads (TC7) next to the existing `recordDeclarePosition("N1")` smoke call. Satisfies AC5, AC6 (conformance), and tests AC1/AC2/AC4. |
| T3 | Verify build + full suite | T1, T2 | low | — | Run the full `ctest` (all three mock-force-include targets + the new cases) and `arduino-cli compile`. Optionally render a quick dump and eyeball the localization-lane marker. Satisfies AC7. |

## Notes for Implementer

- **Today, no CTPS-linked real TU calls `Localization.declarePosition` directly** — the only real-TU call site across `Mission.cpp` / `Tasks.cpp` / `MissionScript.cpp` is `Mission.cpp:253` (the uint8 overload, inside `Mission.declarePosition`). This is why the existing `ctps_dump --fixture`/`--self-test` output is unchanged by T1 (no new op is emitted by the current fixture mission). The moment a task body adds a direct `Localization.declarePosition(...)` call, that guarantee flips and a new `declare_position` op will appear in that mission's dump — which is exactly the intended behavior.
- **Do not perturb `buildMissionFixture()` / the `--self-test` op stream** — it has many existing self-test assertions; the new coverage lives in `test_ctps_dump.cpp` (the `ctps_dump_mock` ctest), which does **not** link `Mission.cpp`, so AC3's "single-count for the Mission path" is verified by construction (hook removed) rather than by a fixture assertion.
- **Include resolution:** from `ctps_mocks/`, `#include "src/nav/FieldMap.h"` resolves via `SKETCH_DIR` and `#include "ctps_record.h"` via `CTPS_MOCKS_DIR` — the same include-dir set every affected target already has. `FieldMap.h` is header-only over `routes.h` (host-safe; no hardware).
- **Force-include order is already correct:** `PathTracker.h` (force-included before `Localization.h`) pulls `routes.h` first, and for the fixture targets `routes_fixture.h` is force-included before the mocks — so adding `FieldMap.h` to `Localization.h` does not change which `routes.h` wins. The conformance test never calls `declarePosition` (only `recordDeclarePosition` directly), so it only needs the new includes to **compile**.
- **Round-trip stability:** `Mission.declarePosition(name)` validates name→id, then the mock resolves id→name; `FieldMap::getNode(FieldMap::findNodeByName(name)).name == name` for a valid node, so the recorded name matches the caller's.
- **No new op / no 3-place contract to extend:** `declare_position` already exists across recorder → loader → renderer. This change adds only a new **emit site**, so the loader whitelist and renderer are untouched.
- **Rollback:** revert T1/T2 to restore the no-op mock + Mission hook; the wire format is unchanged, so no trace/golden compatibility concerns either way.
