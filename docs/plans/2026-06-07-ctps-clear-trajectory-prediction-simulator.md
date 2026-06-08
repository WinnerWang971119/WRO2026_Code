# CTPS (Clear Trajectory Prediction Simulator) v1 Plan

**Goal:** Give the operator a smooth, scrubbable 2D player that animates the robot driving the *actual committed* Mission (route + task sub-commands) over the field map — so authoring logic can be verified without a physical run, saving dry-run time at competition.

**Approach:** Phased Mechanism #4 ("host-build recording mocks → JSON"). A new host-compile target links the **real** `Mission.cpp` + `Tasks.cpp` against **recording mocks** of the hardware facades; running it executes the mission and captures every `goTo` / `Drive` / `Manipulator` / `wait` call **in execution order** as a JSON command stream. A Python package expands each `goTo` through the **existing parity-locked `PathTrackerSim`** (raw `Drive` moves apply directly to `VirtualChassis`), assembles a `(timestamp→pose)+(timestamp→event)` track, and emits a **self-contained `ctps.html`** (field render + embedded track + vanilla-JS transport bar) for genuinely smooth play/pause/seek. The mission is single-source (real C++); there is **no** Python re-implementation of mission/task bodies.

## Scope

- **In scope:**
  - Extract `buildMission()` from `WRO2026.ino` into a host-compilable TU (`MissionScript.h/.cpp`), included by the sketch — firmware behavior unchanged.
  - Recording-mock layer (new `WRO2026/test/ctps_mocks/`) that lets real `Mission.cpp` + `Tasks.cpp` compile + run on host and capture an ordered command stream. Additive `MockSerial` integer-`print` + capture buffer in `mocks/Arduino.h`.
  - `ctps_dump` C++ target + main: runs the mission to completion against recording mocks, serializes the command stream to JSON on stdout.
  - Python `ctps` package: trace loader, track builder (reuses `PathTrackerSim` + `VirtualChassis`), self-contained HTML/Canvas player, and a one-command orchestrator that shells through `vcvars64` → cmake build → dump → expand → emit + open `ctps.html`.
  - Trajectory "T" rendered at the **top of the map**; **side panel** narrates the current sub-command (task name, `goTo` target, `Manipulator` command, `wait`), time-aligned to playback.
  - **Drift flag:** side-panel warning when chassis pose diverges from the logical `current_node` beyond a threshold (catches the raw-move-then-`goTo` desync class).
  - VS Code task entry + `.gitignore` for generated artifacts.
  - Tests: hermetic Python tests over JSON fixtures (no compiler needed) + a C++ dump test in the host harness.
  - Concrete `#46 Phase 1` closure: real-link `Tasks.cpp` into `test_tasks_dispatch` using the new `MockSerial` capture.
- **Out of scope:**
  - **Fault FSM modeling** — CTPS does not simulate the Mission/PathTracker fault FSM (beep/flash/await-reset). The drift flag is the only **physical** problem surfaced; queue-time `build_error` and `UNREACHABLE` are shown as informational banners, not as fault-FSM simulation.
  - **Physical fidelity** — no T-shape/accel/encoder/IMU/slip model; playback timing is constant-velocity for smoothness, not prediction of real durations.
  - **In-player editing / two-trace compare / multi-scenario** — v1 animates one committed mission from `START_NODE`.
  - **LRS ingestion (issue #23)** — the track schema + player are kept source-agnostic so a future logged trace can drop in, but no LRS adapter is built.
  - **Live sensor branching** — any sensor-dependent task branch resolves on the recording mock's deterministic default value (camera is Phase 4); CTPS shows that one branch.
  - **Full `#46 Phase 2`** (Mission.cpp *behavioral assertions* in `test_tasks_dispatch`) — enabled by the new mock layer but not wired in v1.
  - **Manipulator/Drive narration in the *real-mission* trace** — the committed `buildMission()` and its only registered tasks (`demoEcho`, `debugYield`) emit no `Manipulator`/`Drive` calls, so the v1 real-mission trace contains only `goto`/`wait`/`print`/`beep`/`led`/`task_*` records. Manipulator/Drive narration + the drift flag are demonstrated only via the synthetic `buildMissionFixture()` in the dump test, and will appear in a real-mission trace once real Manipulator/Drive tasks are authored (Phase 5/6).

## Decisions

- **Ingestion = Mechanism #4, phased.** Recording at the hardware-facade boundary is robust to helpers/branches/loops inside task bodies; hand-mirror (#1) and regex-scrape (#3) are not. (Brainstorm 2026-06-07.) This **supersedes ADR-025's sim-parity sub-decision** (which anticipated a Python mirror of each task body) — see T12.
- **Player = self-contained HTML/Canvas** (`ctps.html`), committed-to now (matplotlib spike skipped). `/debate` verdict Lean Yes (6.5/10): Canvas + `requestAnimationFrame` + O(1) seek is the lower-risk path to the operator's hard "zero-lag scrub" requirement. Accepted cost: the JS renderer sits outside the TC12/TC18 parity suite.
- **Mission single-source = real C++.** `buildMission()` extracted to `MissionScript.cpp`, compiled into both the firmware and the dump. Honors ADR-025 (no JSON authoring; JSON is a *derived* artifact like `routes.h`).
- **Trajectory reuse = existing `PathTrackerSim`.** Python expands `goTo` targets through the parity-locked sim; the C++ dump emits only the high-level command stream. Clean labor split: C++ = "what commands, in order"; Python = "what geometry/animation".
- **Recording mocks isolated in `ctps_mocks/` and force-included.** Quote-form includes (`"Drive.h"`, `"PathTracker.h"`, …) resolve source-relative regardless of `-I` order, so shadowing requires **force-include** (`/FI` MSVC, `SHELL:-include` gcc) via the existing `add_mock_force_includes()` mechanism — matching include guards alone are NOT sufficient. Only `<MatrixMiniR4.h>` (angle-included) is shadowed by include-path order. The dump's include dir is `ctps_mocks/` **only** (not `mocks/`, whose empty `MatrixMiniR4.h` would otherwise win). This keeps the existing assertion `mocks/` (and the 7 passing host tests) untouched; only `MockSerial` gains additive overloads.
- **`mission_trace.json` is gitignored build output** — consumed only by the Python player (not by firmware, unlike `routes.h`), so no CI freshness gate.

## Acceptance Criteria

- [ ] **AC1** — `uv run python -m tools.path_planner.ctps` (on a machine with the VS Build Tools toolchain) builds the dump, generates `ctps.html`, and opens it; the robot animates driving the committed mission across the field map with working play / pause / seek.
- [ ] **AC2** — The trajectory "T" renders at the top of the field map; the side panel shows the current sub-command (task name, `goTo` target, `Manipulator` command — e.g. `Manipulator: setGripper(180)` — and `wait`) time-aligned to the playhead.
- [ ] **AC3a** (automated) — Seek is O(1)-class: the generated `player.js` resolves a seek by a bounded index lookup / binary search into the baked keyframe track + lerp between adjacent keyframes — never a per-frame full-array scan. Verified by static inspection of the emitted JS (TC12).
- [ ] **AC3b** (manual) — Scrubbing/seeking *feels* smooth with no perceptible lag (Canvas + rAF playback). Operator-verified; not automated.
- [ ] **AC4** — Manip + raw `Drive` + arbitrary-helper recording in real execution order is satisfied by the synthetic `buildMissionFixture()` exercised in the C++ dump test (TC8) — **not** by the real mission.
- [ ] **AC5** — A raw `Drive` move followed by `goTo()` renders the chassis-vs-node drift (robot ends offset from the node marker); the side panel emits a drift warning when divergence exceeds `DRIFT_THRESHOLD_CM`.
- [ ] **AC6** — The animated mission is the real `buildMission()` (from `MissionScript.cpp`), compiled into both firmware and dump, with no Python re-implementation. This is a separate artifact from the AC4 fixture trace. The firmware sketch still compiles (arduino-compile CI green).
- [ ] **AC7** — The Track schema and HTML player are source-agnostic (documented contract) so a future LRS logged trace can feed the same player; no LRS ingestion exists in v1.
- [ ] **AC8** — Python tests pass hermetically with **no** C++ compiler (fixtures only); the C++ dump test passes in the host harness locally (MSVC) and in CI (g++). All 7 pre-existing host tests still pass.
- [ ] **AC9** — `test_tasks_dispatch` links and asserts against the **real** `Tasks.cpp` registry/dispatch via `MockSerial` capture (closes #46 Phase 1).
- [ ] **AC10** — v1's real-mission trace contains only `goto`/`wait`/`print`/`beep`/`led`/`task_start`/`task_end` records (no `op:manip`/`op:drive`), reflecting the committed mission's actual calls.

## Data Model

### A. Command-stream JSON — contract between the C++ dump and the Python loader

`mission_trace.json` (stdout of `ctps_dump`):

```jsonc
{
  "schema_version": 1,
  "start_node": "START",         // field_data.START_NODE at dump time (informational only;
  "start_heading_deg": 90.0,     //   the Python side seeds the chassis from field_data, the
                                 //   single source of truth — if they disagree, field_data wins
                                 //   and the loader warns)
  "build_error": false,          // mirrors Mission._buildError (bad task name → true)
  "commands": [
    { "seq": 0, "op": "goto",       "target": "A" },
    { "seq": 1, "op": "wait",       "ms": 500 },
    { "seq": 2, "op": "print",      "text": "at A" },
    { "seq": 3, "op": "task_start", "name": "demoEcho", "args_hex": "072a000000000000000000000000000000" },
    { "seq": 4, "op": "manip",      "method": "setGripper", "arg": 180 },   // fixture only (not real mission)
    { "seq": 5, "op": "drive",      "method": "driveDist", "cm": 15.0 },    // fixture only
    { "seq": 6, "op": "drive",      "method": "turnDeg",   "deg": -90.0 },  // relative turn
    { "seq": 7, "op": "goto",       "target": "B" },
    { "seq": 8, "op": "manip",      "method": "setLift", "arg": 120 },
    { "seq": 9, "op": "task_end",   "name": "demoEcho",  "result": "ok" }, // ok|yield
    { "seq": 10,"op": "beep",       "freq_hz": 1000, "ms": 200 },
    { "seq": 11,"op": "led",        "r": 0, "g": 255, "b": 0, "ms": 300 }
  ]
}
```

- `op` vocabulary (v1): `goto`, `task_start`, `task_end`, `manip`, `drive`, `wait`, `print`, `beep`, `led`. Unknown ops are logged and skipped by the loader (forward-compat).
- `manip.method` (v1): the **real** `ManipulatorClass` API — `setLift`, `setGripper`, `setArm` (each with a `uint8` `arg`), plus `begin`/`update`/`isDone` if ever recorded. **There is no `grab()`/`release()`** — those are not real API and will not appear in any trace.
- `drive.method` (v1 geometry-affecting): `driveDist` (`cm`, relative, → `VirtualChassis.forward/reverse`), `turnDeg` (`deg`, relative, → `turn_relative`), `turnToYaw` (`deg`, absolute world heading, → `turn_to_heading`). Other Drive primitives are captured as `{method, raw_args}` and ignored for geometry with a loader warning (documented extension point).
- `args_hex` is the raw 16-byte `TaskArgs` (hex, **host-endian** — both CI g++ and local MSVC are little-endian, consistent with the existing TC3 union-punning assumption; v1 does not decode per-task structs).

### B. Track JSON — contract between the Python track builder and the HTML player (embedded in `ctps.html`)

```jsonc
{
  "schema_version": 1,
  "field": {
    "nodes": { "A": [120.0, 40.0], "B": [200.0, 40.0] },
    "obstacles": [ [x_min, y_min, x_max, y_max, "kind"] ],
    "bounds": [x_min, y_min, x_max, y_max]
  },
  "duration_s": 12.34,
  "keyframes": [ { "t": 0.0, "x": 0.0, "y": 0.0, "h": 90.0 }, /* … pose at each primitive boundary */ ],
  "events":    [ { "t": 1.20, "type": "task_start", "label": "demoEcho" },
                 { "t": 3.40, "type": "manip",      "label": "setGripper(180)" },
                 { "t": 5.10, "type": "drift_warn", "label": "chassis 18.2 cm off node B" } ]
}
```

- The player interpolates pose **linearly between keyframes** at `requestAnimationFrame` time (x/y lerp; heading lerps on the shortest arc). Keyframes are sparse (one per primitive endpoint) → small JSON, smooth render. Keyframe poses are taken **from the `VirtualChassis` snapshot `step_through` yields**, not recomputed from a primitive's offset argument.
- `event.type` (v1): `task_start`, `task_end`, `goto`, `manip`, `wait`, `drift_warn`, `print`, `beep`, `led`.

### C. Python timing config (`ctps/config.py`) — visual, NOT physical

```python
SIM_DRIVE_CM_S    = 30.0   # cm/s used to assign drive/line primitive durations
SIM_TURN_DEG_S    = 180.0  # deg/s used to assign turn durations
MANIP_NOMINAL_S   = 0.5    # narration dwell for a Manipulator command
DRIFT_THRESHOLD_CM = 10.0  # chassis-vs-node divergence that raises drift_warn
                           # Rationale: after a CLEAN goto the chassis lands ON the node
                           # (the LINE_OVERSHOOT step advances the axle onto the node), so
                           # drift ≈ 0 — NOT ≈5 cm. A raw Drive move between hops injects a
                           # real offset that goto does not correct (localization desync).
                           # 10 cm sits comfortably above float/rounding noise + any benign
                           # dead-reckon residual, while still catching a raw-move desync.
                           # Drift is measured AFTER each goto's final hop completes,
                           # comparing the chassis pose to NODES[current_node].
```

## API / Module Contracts

### C++ (`WRO2026/`)
- `MissionScript.h`: `void buildMission();` (declaration). `MissionScript.cpp`: the body moved verbatim from the .ino. The .ino `#include "MissionScript.h"` and deletes its local `buildMission()`.
- `WRO2026/test/ctps_mocks/` recording headers (include guards MUST match the real ones **and** be force-included so they shadow): `MatrixMiniR4.h` (recording `MiniR4` with `LED`/`Buzzer`/`BTN`/`OLED`; shadowed by include-path order since it is angle-included), `Drive.h` (recording fluent-builder — reproduce ALL five move-only `[[nodiscard]]` builder types: `driveDist`/`turnDeg`/`turnToYaw`/`stop`/`stopAtLine` as applicable — plus `isIdle`→done, `hasFault`→false), `PathTracker.h` (recording `goTo`→records target+returns true, `isIdle`→true, `hasFault`→false, `clearFault`), `EventTrace.h` (recording or no-op `pushEvent`). `FieldMap.h` + `routes.h`/`routes.cpp` are real (pure const data, no hardware) and linked, not mocked. `LineFollower.h` needs a mock surface (Mission.cpp includes it). All recording mocks append to a shared `g_ctpsLog` (ordered vector of records) defined in `ctps_mocks/ctps_record.cpp`.
  - **`ctps_mocks/Manipulator.h`** is required ONLY when a fixture-task TU is added that `#include "Manipulator.h"`; that TU must include the header so the mock shadows it. It is NOT part of the real `Mission.cpp`/`Tasks.cpp` compile chain in v1 (Manipulator is included only by the .ino and `Manipulator.cpp`). The baseline v1 dump links without a Manipulator mock since no real task body references it.
  - **No `ctps_mocks/Localization.h`** — `Mission.cpp` does not include `Localization.h`, and `PathTracker.cpp` (which does use it) is shadowed by the mock, so `Localization` is not odr-used by the dump link.
- `ctps_dump.cpp` `int main()`: `…begin()` all singletons → install no-op tick callback → `buildMissionFixture()` (the dump-test mission) or `buildMission()` (the real mission, for the production run) → `Mission.start()` → loop `Mission.update()` until `Mission.isIdle()` or `CTPS_MAX_TICKS` (error if exceeded) → print `g_ctpsLog` as the command-stream JSON.

### Python (`tools/path_planner/ctps/`)
- `trace_loader.load(path|str) -> Trace` — parse + validate command-stream JSON; skip unknown ops with a warning.
- `track_builder.build(trace, field_data) -> Track` — build the graph (`build_graph`) + all-pairs `compute_all_pairs_next_hop`, construct **one** `VirtualChassis.from_field_data` + **one** `PathTrackerSim`, call `localization.zero_heading_from_current()` once before the first hop, then iterate commands **on that single long-lived sim/chassis** (never reconstruct per `goto`, or accumulated drift resets): `goto` → `PathTrackerSim.step_through(target)`, reading each yielded `(primitive, snapshot)` and using `snapshot` x/y/heading for keyframes; `drive` → apply to the shared `VirtualChassis` (`driveDist`→forward/reverse, `turnDeg`→turn_relative, `turnToYaw`→turn_to_heading) WITHOUT advancing `current_node`; `manip/wait/print/beep/led/task_*` → events; after each `goto`'s final hop, compare chassis pose vs `NODES[current_node]` → emit `drift_warn` past `DRIFT_THRESHOLD_CM`. Assign timestamps via `config` speeds.
- `player_html.render(track, out_path) -> Path` — emit a single self-contained `ctps.html` (field SVG + embedded Track JSON + `assets/player.js`). The seek path in `player.js` MUST be a bounded index/binary-search + lerp, never a per-frame full-array scan (AC3a).
- `__main__.py` — orchestrate build+dump+expand+emit+open; `--no-open`, `--out`, `--trace <existing.json>` (skip the C++ build and render an existing dump) flags.

## Error Handling

- **No host C++ toolchain found** (vcvars64 missing): `__main__` prints a clear message with the `--trace <file>` fallback (render a pre-generated dump) and exits non-zero. Python tests never invoke the compiler.
- **`build_error: true` in the dump** (bad task name at queue time): player still renders the steps enqueued before the error and shows an informational banner "mission build error — trace truncated" (NOT a fault-FSM simulation).
- **Dump exceeds `CTPS_MAX_TICKS`** (mission never goes idle): dump exits non-zero with a diagnostic; orchestrator surfaces it.
- **Unreachable goal** (`PathTrackerSim` raises `UnreachableGoalError`): track builder ends the trajectory at the fault point and emits an informational `event` `{type:"goto", label:"UNREACHABLE A→B"}`; playback stops there. This is a banner, not fault-FSM modeling.
- **Unknown `op` / `drive.method`**: loader logs a warning and skips; geometry continues from the last known pose.

## Testing Strategy

**Levels:** Unit (Python, hermetic) + Integration (C++ dump in host harness).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `track_builder` on a fixture with two `goto`s | Unit (py) | Keyframes match `PathTrackerSim` yielded snapshots; monotonic non-decreasing `t`; `duration_s` = last `t`. (AC1, AC10) |
| TC2 | Fixture: `task_start`→`manip setGripper`→`goto`→`manip setLift`→`task_end` | Unit (py) | Events emitted in order with correct labels (`setGripper(180)` etc.); side-panel labels present. (AC2) |
| TC3 | Fixture: `drive driveDist 15` then `goto B` from a node | Unit (py) | Chassis pose after `goto`'s final hop is offset from `NODES["B"]` by ≈ the raw-move vector; a `drift_warn` event is emitted. (AC5) |
| TC4 | Fixture: `drive driveDist 2` (small) then `goto` | Unit (py) | Divergence < threshold → **no** `drift_warn` (negative path). (AC5) |
| TC5 | `player_html.render` on a Track fixture | Unit (py) | Output is one self-contained `.html`; embeds the exact Track JSON; contains the transport-bar JS markers; no external `src`/`href` to local files. (AC1) |
| TC6 | Track schema round-trips an LRS-shaped synthetic track (poses+events, no command stream) | Unit (py) | `player_html.render` accepts it unchanged → proves source-agnostic player. (AC7) |
| TC7 | `trace_loader` on JSON with an unknown `op` and unknown `drive.method` | Unit (py) | Loads, warns, skips the unknown entries, does not raise. |
| TC8 | `ctps_dump` on `buildMissionFixture()` (goto + a task calling `Manipulator.setGripper` + raw `driveDist`/`turnDeg` + a helper) | Integration (cpp) | stdout JSON command stream matches the expected order/targets/args (normalize/​document little-endian `args_hex`); process exits 0. (AC4, AC6) |
| TC9 | `ctps_dump` termination | Integration (cpp) | Mission drains to idle within `CTPS_MAX_TICKS`; no infinite loop. |
| TC10 | `test_tasks_dispatch` against real `Tasks.cpp` | Integration (cpp) | Real registry dispatch, RAII reentrancy guard, unknown-name handling, and yield-via-`debugYield` assertions pass via `MockSerial` capture. (Tasks-owned behavior only; Mission-owned abort ordering is out of scope — that needs Mission.cpp, #46 Phase 2.) (AC9) |
| TC11 | Full host harness | Integration (cpp) | All pre-existing 7 ctests still pass after mock/CMake changes. (AC8) |
| TC12 | Static inspection of emitted `player.js` | Unit (py) | The seek path uses a bounded index lookup / binary search + lerp between adjacent keyframes (e.g. a `Math.floor`/bisect index bounded by keyframe count); it contains **no** unbounded per-frame scan over the full keyframe array. (AC3a) |

**Test data:** Python fixtures in `tools/path_planner/tests/fixtures/ctps_*.json` (hand-authored command streams + a Track fixture + an LRS-shaped track). The C++ dump test uses a dedicated `buildMissionFixture()` in the test TU (which `#include "Manipulator.h"` so the recording mock shadows it) so it is independent of the evolving production `buildMission()`.
**Run commands:** `uv run pytest tools/path_planner/tests/ -k ctps` (Python); `cmake -S WRO2026/test -B build/test -G Ninja && cmake --build build/test && ctest --test-dir build/test` inside `vcvars64` (C++).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Extract `buildMission()` to a host-compilable TU | — | med | `WRO2026/MissionScript.h`, `WRO2026/MissionScript.cpp`, `WRO2026/WRO2026.ino` | Move the `buildMission()` body (currently `WRO2026.ino:165`) verbatim into `MissionScript.cpp`; declare in `MissionScript.h`; `#include "MissionScript.h"` in the .ino and remove the local definition. No behavior change. Verify the sketch still compiles (arduino-compile). Satisfies AC6. |
| T2 | `MockSerial` integer-print + capture buffer | — | low | `WRO2026/test/mocks/Arduino.h` | Add `print(uint8_t)`, `print(int)`, `print(unsigned)`, `println(...)` overloads and an appendable capture buffer with an accessor + reset. **Additive only** — must not change existing overloads or break overload resolution for the 5 existing force-include consumers (`test_line_follower_overshoot`, both passthrough, both bidirectional, `test_path_tracker_stop_at_line`, `test_tasks_dispatch`). T2's own gate: all pre-existing host tests still pass (TC11). Enables AC4/AC9 capture. |
| T3 | CTPS recording-mock layer | — | high | `WRO2026/test/ctps_mocks/MatrixMiniR4.h`, `ctps_mocks/Drive.h`, `ctps_mocks/PathTracker.h`, `ctps_mocks/EventTrace.h`, `ctps_mocks/LineFollower.h`, `ctps_mocks/ctps_record.h`, `ctps_mocks/ctps_record.cpp` (+ `ctps_mocks/Manipulator.h` for the fixture path) | New mock headers whose include guards MATCH the real headers. **Quote-included headers (`Drive.h`, `PathTracker.h`, `EventTrace.h`, `LineFollower.h`) are shadowed only via force-include** (`/FI` MSVC, `SHELL:-include` gcc) wired in T4 — matching guards + include path are NOT sufficient. `<MatrixMiniR4.h>` (angle) is shadowed by include-path order. Each records its calls (target/method/args, in order) into shared `g_ctpsLog`; all "is-done?" queries report instant completion (`isIdle`→true, `hasFault`→false). Reproduce ALL five move-only `[[nodiscard]]` Drive builder types so discarded-temporary call sites compile. Signatures must match the real facades exactly. Read the real headers to mirror signatures. `Manipulator.h` mock is added only with the fixture task that includes it. Satisfies AC4. |
| T4 | `ctps_dump` main + cmake target + ctest | T1, T2, T3 | med | `WRO2026/test/ctps_dump.cpp`, `WRO2026/test/CMakeLists.txt` | Add `ctps_dump` executable linking real `Mission.cpp` + `Tasks.cpp` + `MissionScript.cpp` + `routes.cpp` + `ctps_mocks/ctps_record.cpp`, include dir `ctps_mocks/` **only** (exclude `mocks/`). Apply `add_mock_force_includes`-style `/FI`+`SHELL:-include` for every quote-included shadowed header (`Drive.h`, `PathTracker.h`, `EventTrace.h`, `LineFollower.h`). `main()` runs `buildMissionFixture()` to idle (cap `CTPS_MAX_TICKS`) and prints command-stream JSON. Register a ctest (TC8/TC9) asserting the JSON for the fixture. Satisfies AC4, AC6; partial AC8. |
| T5 | `#46 Phase 1`: real-link `Tasks.cpp` into `test_tasks_dispatch` | T2, T4 | med | `WRO2026/test/test_tasks_dispatch.cpp`, `WRO2026/test/CMakeLists.txt` | Link `${SKETCH_DIR}/Tasks.cpp` into the `test_tasks_dispatch` target; rewire the registry/dispatch/reentrancy/unknown-name/yield TCs onto the real `Tasks` singleton, asserting via `MockSerial` capture. Tasks-owned behavior only (no Mission abort-ordering — that's #46 Phase 2). Blocked by T4 (shared `CMakeLists.txt`). Satisfies AC9, TC10. |
| T6 | Python `ctps` package skeleton + config + loader | — | low | `tools/path_planner/ctps/__init__.py`, `ctps/config.py`, `ctps/trace_loader.py` | Create the package; `config.py` timing/threshold constants; `trace_loader.load()` parsing + validation per Command-stream schema, skipping unknown ops with warnings. Satisfies TC7. |
| T7 | Track builder (reuses `PathTrackerSim`/`VirtualChassis`) | T6 | med | `tools/path_planner/ctps/track_builder.py` | Implement `build(trace, field_data) -> Track`: build graph + next_hop, construct ONE `VirtualChassis` + ONE `PathTrackerSim`, `zero_heading_from_current()` once, then iterate commands on that single sim (never reconstruct per `goto`). Expand `goto` via `step_through`, using the YIELDED snapshot poses for keyframes; apply raw `drive` to the shared chassis WITHOUT advancing `current_node`; assign timestamps from `config`; emit events; emit `drift_warn` when chassis-vs-`current_node` divergence > `DRIFT_THRESHOLD_CM` (measured after each goto's final hop). Satisfies AC2, AC5, AC10; TC1–TC4. |
| T8 | HTML/Canvas player (transport bar) | T7 | med | `tools/path_planner/ctps/player_html.py`, `tools/path_planner/ctps/assets/player.js` | `render(track, out_path)`: emit one self-contained `ctps.html` — field SVG/Canvas (nodes/obstacles/bounds from `track.field`), trajectory "T" at top, side panel, embedded Track JSON, and `player.js` (rAF playback, play/pause, **bounded index/binary-search seek + keyframe lerp incl. shortest-arc heading** — never a per-frame full scan). Satisfies AC1, AC2, AC3a, AC3b, AC7; TC5, TC6, TC12. |
| T9 | One-command orchestrator | T4, T7, T8 | med | `tools/path_planner/ctps/__main__.py` | Shell `vcvars64` → cmake configure/build `ctps_dump` → run it → capture JSON → `track_builder.build` → `player_html.render` → open in browser. Flags: `--no-open`, `--out`, `--trace <file>` (skip build, render existing dump). Clear error if toolchain missing (point to `--trace`). Satisfies AC1; error-handling section. |
| T10 | Python tests | T7, T8 | low | `tools/path_planner/tests/test_ctps_track_builder.py`, `tools/path_planner/tests/test_ctps_player_html.py`, `tools/path_planner/tests/fixtures/ctps_*.json` | Implement TC1–TC7 + TC12 against hand-authored fixtures; no compiler invoked. Wire into the existing pytest run. Satisfies AC8 (Python half), AC3a. |
| T11 | VS Code task + gitignore | T9 | low | `.vscode/tasks.json`, `.gitignore` | Add a "CTPS: build & view" task running the T9 command (use a non-empty sentinel for any optional `${config:...}` arg per the Windows empty-arg gotcha). Gitignore `tools/path_planner/output/ctps*.html` and `**/mission_trace.json`. Satisfies AC1 (ergonomics). |
| T12 | Docs: new ADR + amend ADR-025 + issue update | T4, T8 | low | `docs/decisions/0NN-ctps-derived-mission-trace.md`, `docs/decisions/025-tasks-as-blocking-methods.md`, `DECISIONS.md` | New ADR: "CTPS ingests a derived mission trace from a host build of the real C++ (Mechanism #4); player is HTML/Canvas; faults/physics out of scope." **Amend ADR-025's sim-parity sub-decision** (the "Python mirror of each task body" direction): mark that paragraph superseded by the new ADR and cross-link both. Update `DECISIONS.md` index + status table. |

**Parallelism:** T1, T2, T3, T6 start immediately (no blockers). T7 follows T6; T8 follows T7; T4 follows T1+T2+T3; T9 joins T4+T7+T8; T5 after T4; T10 after T7+T8; T11 after T9; T12 after T4+T8. `CMakeLists.txt` is touched only by T4 then T5 (serialized — T5's "Blocked By" includes T4). `mocks/Arduino.h` is touched only by T2. No two parallel tasks share a file.

## Notes for Implementer

- **Recording mocks are the hard part (T3).** They must mirror the *exact* method signatures of the real `Drive` / `PathTracker` / `MiniR4` / `EventTrace` / `LineFollower` headers or real `Mission.cpp`/`Tasks.cpp` won't compile. Read each real header first. **Quote-included headers shadow only via force-include** (`/FI`, `SHELL:-include`) — set up exactly like the existing `add_mock_force_includes()` in `WRO2026/test/CMakeLists.txt`; include-path order alone does NOT shadow them. The dump's include dir must be `ctps_mocks/` only (the empty `mocks/MatrixMiniR4.h` would win otherwise).
- **The fluent `Drive` builder is move-only RAII** (deleted copy ctor, enqueue-on-destruct). Reproduce all five builder types as `[[nodiscard]]` move-only temporaries; never bind to a named `auto` local. Real `Mission.cpp`/fixture code calling `Drive.driveDist(15)` as a discarded temporary won't compile against a copyable mock.
- **`Manipulator` has no `grab()`/`release()`.** The real `ManipulatorClass` (a Phase-1 no-op stub) exposes only `begin`/`update`/`setLift(uint8_t)`/`setGripper(uint8_t)`/`setArm(uint8_t)`/`isDone`. Narrate those (`setGripper(180)` etc.). It is not in the real `Mission.cpp`/`Tasks.cpp` compile chain — its mock is only needed for the fixture task TU that includes it.
- **Instant-completion is the trick.** Task bodies "fake-block" by draining a cooperative tick until `isIdle()`. The recording mocks must report done immediately (and `hasFault()`→false) so the drain exits in ≤1 iteration; otherwise the dump hangs. Install a no-op tick callback.
- **Drift falls out naturally — keep ONE shared `VirtualChassis`.** `PathTrackerSim.step_through` drives from the chassis's actual pose but advances `current_node` separately; a raw `drive` advances the chassis but not the node. Use a single long-lived sim/chassis across the whole track (never reconstruct per `goto`, or the offset resets). After a *clean* goto the chassis is at the node (drift ≈ 0); a raw move injects the offset. Measure drift after each goto's final hop; threshold (10 cm) is above float noise, below a meaningful raw-move desync.
- **Read poses from the yielded snapshot, not the primitive's offset arg.** `step_through` yields `(primitive, snapshot)`; the snapshot is the authoritative pose. Note `step_through` hard-codes the FRONT `LINE_SENSOR_AXLE_OFFSET_CM` in the overshoot *primitive's value* even on REAR hops (a latent sim quirk) — the snapshot pose is still correct, so always key off the snapshot.
- **Turn semantics:** `turnDeg` is relative (`turn_relative`); `turnToYaw` is absolute world heading (`turn_to_heading`). Do not collapse them — they move the chassis differently.
- **Hermetic Python tests.** TC1–TC7 + TC12 must never shell out to a compiler — they consume committed JSON fixtures. Only the C++ dump test (TC8–TC11) needs the toolchain; CI runs it on g++, locally it runs on MSVC via `vcvars64` (verified working).
- **v1 demonstrates on the current mission** (`goTo` + `demoEcho`, which emits no manip/drive). Real manipulator/drive tasks animate automatically once authored in Phase 5/6 — no CTPS change needed. The T4 fixture (`buildMissionFixture()`) includes a synthetic task that calls `Manipulator.setGripper` + a raw `Drive` move so the side-panel narration + drift flag are exercised before those real tasks exist.
- **Rollback:** every change is additive (new files, new cmake target, new Python package) except the T1 extraction (mechanical move) and the T2 additive overloads. If CTPS is shelved, delete `ctps/`, `ctps_mocks/`, the `ctps_dump` target, and revert T1; the firmware and existing tests are unaffected.
- Heading/coordinate convention matches `chassis.py`: degrees in `[0,360)`, `0°=+x` (East), CCW positive.
