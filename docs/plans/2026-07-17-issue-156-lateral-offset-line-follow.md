# LineFollower Lateral Offset-Follow (issue #156) Plan

**Goal:** Make a LINE edge followable biased LEFT or RIGHT of the line centerline (not only centered/`MIDDLE`), so a mission can hug one side of a line to clear an obstacle or align a manipulator — completing the surface ADR-064 reserved but left inert.

**Approach:** Setpoint-bias. In `LineFollower::update()` the P+D loop already drives the sensor centroid `getError()` (range `[-3.5,+3.5]`, weighted, DARK-line) to 0. Add a signed offset to the *proportional* term only — `corr = LINE_KP*(err - offset) + LINE_KD*(err - _lastErr)` — so the loop instead holds the line off-center under the sensor, shifting the chassis laterally. `offset` is a per-edge property carried by a new 8th `EdgeInfo` field (`line_follow_mode_override: LineFollowMode`), emitted by `header_writer`, resolved in `PathTracker`, mapped to a signed float, and passed into the follower as a nav-agnostic bias. Everything else (P+D structure, T-shape ramps, junction-freeze, distance gate, lost-line) is unchanged.

## Scope

- **In scope:**
  - LEFT/RIGHT lateral-offset behavior in `LineFollower` (derivative-on-measurement setpoint bias).
  - New generated C++ `Routes::LineFollowMode` enum + an 8th `EdgeInfo.line_follow_mode_override` field, emitted by `header_writer` and consumed by `PathTracker`'s `LINE_FOLLOW_INTERSECTION` and `PROJECTION_FOLLOW` dispatch.
  - `Config::LINE_FOLLOW_OFFSET_ERROR` (float, error/centroid units) + a `static_assert` magnitude cap.
  - Remove the `LEFT`/`RIGHT` rejection guard in `validator.py`; make LINE `mode` **directional** (per-directed-edge) in `graph_builder` while BLIND `mode` stays bidirectional; enable LEFT/RIGHT in the editor Mode bar + a directional-semantics help note.
  - **Full `makeEdge()` adoption**: a `constexpr makeEdge(...)` factory emitted into `routes.h` (and the shadow-routes fixture headers), used by the emitter for every `EDGES` cell AND by every hand-written init; guarded by a `constexpr`-evaluability canary and a CI flash-size gate proving the `EDGES` array stayed in `.rodata`.
  - **Full sim/parity/CTPS modeling**: `resolve_line_follow_mode` in `sim/runtime.py`; a `follow_mode` field on the `line_follow`/`projection_follow` parity records with a `PARITY_SCHEMA_VERSION` 2→3 bump (renaming the misnamed `line_mode`→`speed` in the same bump); a **parity-safe visualization-only** geometric lateral hug-side render in CTPS (ADR-054 chassis-nudge pattern — never touches the parity-locked sim pose).
  - `build_routes.py` lint warning when a biased LINE edge's reverse traversal is actually used (`NEXT_HOP`) with no companion authored entry.
  - Regenerate `routes.h/.cpp`; update the 3 host-test shadow-routes fixtures + `routes_toy` golden; add/adapt host ctest + Python + editor tests; `arduino-cli compile`.
  - ADR-069 superseding ADR-064's "LINE Mode surface-only, no field" clause; DECISIONS.md updates; close issue #156.
- **Out of scope:**
  - Per-edge offset *magnitude* authoring — the amount is one global `Config` constant; the per-edge mode only picks the sign (LEFT/RIGHT). A numeric per-edge offset would need its own ADR.
  - A `goTo(...)` call-site override enum (NavTypes mirror) — the mode is edge-authored only, like `speed`/`blind_mode`.
  - Changes to BLIND `mode` handling or the `SpeedTier`/`stop_at_line` axes.
  - Edge-following (tracking a line boundary) — infeasible on a thin line where edge ≈ centroid.
  - Compensating the biased axle position in `Localization` or the sim pose (see Decisions).

## Decisions

- **Setpoint-bias, not edge-follow or dead-reckon** — the 8-channel centroid already produces the exact control variable; on a thin dark line an "edge" coincides with the centroid within a channel, and dead-reckoned lateral displacement is rejected by ADR-001 and ignores the line. Fable + Codex both endorse.
- **Derivative on measurement, offset on the P term only** — `corr = LINE_KP*(getError() - offset) + LINE_KD*(getError() - _lastErr)`, `_lastErr` stays the **raw** centroid. This preserves the `getRawError()` telemetry contract AND means a mid-follow mode change injects no derivative kick (the P-step is intentional and bounded, ≈ `LINE_KP*offset` ≈ 8.7 PWM at offset 1.5). Both consultants; Codex flagged the raw-`_lastErr` contract explicitly.
- **`Config::LINE_FOLLOW_OFFSET_ERROR = 1.5f`** in centroid-index units, symmetric (LEFT=+, RIGHT=−), a bench-tune starting point. `static_assert(0.0f <= LINE_FOLLOW_OFFSET_ERROR && LINE_FOLLOW_OFFSET_ERROR <= 1.5f)` — 1.5 leaves ≥2 error units of margin before the outer channel and keeps `LINE_KP*offset` well under the `LINE_MAX_CORR=20` clamp. Not cm: error units don't map to a fixed distance without sensor pitch/height/line-width, measured on the mat.
- **A1 (accepted): LEFT = chassis rides left of the line** — `LineFollowMode::LEFT` → `offset = +LINE_FOLLOW_OFFSET_ERROR` (line settles right-of-center under the sensor); RIGHT mirrors. A labeling convention pinned in ADR-069, trivially reversible (one sign). Bench-verify polarity on front AND rear boards.
- **A2 (refined): the bias is persistent owner state, applied whenever the online non-junction P+D path runs** — not "excluded from LINE_OVERSHOOT". It is set at `LINE_FOLLOW_INTERSECTION`/`PROJECTION_FOLLOW`, preserved across the overshoot re-arm (override-pair pattern), and simply inert while the sensor is offline (no P+D) or frozen at a junction (symmetric cruise). Codex corrected the original "sensor offline in overshoot" framing.
- **A3 (accepted): DEFAULT == MIDDLE (centered), no pipeline default** — `_resolveLineFollowMode` maps `DEFAULT`→`MIDDLE`→offset 0; there is no `PIPELINE_CONFIG` follow-offset field (no mission-wide "everything hugs left" knob). Codex point 7.
- **A4 (accepted): junction-freeze + lost-line unchanged** — the bias only shifts the P setpoint on the online, non-junction tick.
- **Forward-only / per-directed-edge LINE mode** (user pick, both endorse) — LEFT/RIGHT is asymmetric under reversal, so it belongs to the authored directed edge like `direction`/`stop_at_line`. `graph_builder` splits `mode`: directional for LINE, bidirectional for BLIND. A companion reverse entry is authored for the reverse traversal; `build_routes.py` warns if a used reverse traversal lacks one.
- **Travel-relative reverse sign** (user pick, both recommend) — LEFT always = left of the direction of motion; `PathTracker` negates the resolved offset when the hop uses the rear board (`decision.reverse`), so the sensor-frame mirroring of the rear array is compensated. Exact polarity host-tested + bench-confirmed.
- **LineFollower is nav-agnostic** — its builder takes a signed float `setLineFollowOffset(float)`, not the `Routes::LineFollowMode` enum, so the drive layer keeps zero dependency on generated nav data (mirrors how `Direction` is separate from `Routes::DirectionMode`). `PathTracker` owns mode→offset mapping + reverse sign; parity/CTPS record the mode enum from `PathTracker`.
- **Full `makeEdge()` adoption + size gate** (user pick) — uniform factory everywhere so a future field #9 touches only the factory signature; a correct `constexpr` factory keeps the 1521-cell `const EDGES` array in flash `.rodata` exactly as today's aggregates. Guarded by a compile-time `constexpr` canary and an `arduino-cli` `.data`/`.bss` size check gating that nothing moved flash→RAM.
- **Full geometric CTPS render, parity-safe** (user pick) — a visualization-only lateral chassis nudge on the biased leg via the ADR-054 pattern (the pivot wheel-arc precedent), mapping the offset to cm through a documented display constant; the parity-locked `LocalizationSim` pose is never touched. Both consultants vetoed a render that *alters the pose*; this honors "draw it offset" without that.
- **Parity v3 bump** — add `follow_mode` to `line_follow`/`projection_follow`; rename the misnamed `line_mode` (which holds the speed tier) → `speed` in the same bump, since a later rename would need another bump.
- **Accepted approximation, not modeled** — a biased arrival lands the axle ≈1–1.5 cm laterally off the node vs. what `Localization`/the sim assume; this is inside junction-arrival tolerance. Do NOT compensate it in `Localization` or the sim (the project has a scar from adding per-junction sim offsets). State it in the ADR; bench-gate `getJunctionType()` classification + `isLineLost()` at full offset before a scored run relies on it.
- **ADR number 069** — 066/067 (fix/issues-141-159-hardening) and 068 (feat/turn-pd-controller) are claimed on unmerged branches; re-verify no new collision at author time and renumber if needed.

## Acceptance Criteria

- [ ] **AC1** — A LINE `EDGES` entry with `mode: LEFT` follows biased so the line settles right-of-center under the sensor (chassis left of line); `mode: RIGHT` mirrors it; `mode: MIDDLE`/absent follows centered (byte-identical steering to today). Verified by a host LineFollower test asserting the emitted L/R for a mocked `getError()`.
- [ ] **AC2** — The bias is applied via `corr = LINE_KP*(err - offset) + LINE_KD*(err - _lastErr)` with `_lastErr` = raw centroid: `getRawError()` still returns the raw sample, and a mid-follow `MIDDLE→LEFT` mode change produces the bounded P-step with **no** derivative kick (host test).
- [ ] **AC3** — `LineFollowMode::LEFT` on a REAR-board (reverse) hop negates the offset so it stays left-of-travel (host test pins the composition `PathTracker` passes; polarity bench-confirmed).
- [ ] **AC4** — Mode lifecycle: a cold `setEnabled(true)` with no override follows centered (offset 0); a re-arm on an already-enabled follower preserves the current offset; `setEnabled(false)` resets the offset to 0 so `followDistance()`/`followUntilIntersection()` never inherit a stale bias (host tests).
- [ ] **AC5** — `mode: LEFT|RIGHT` on a LINE edge validates (the issue-#156 guard is gone) and emits `EdgeInfo.line_follow_mode_override = LineFollowMode::LEFT|RIGHT`; `mode: MIDDLE` emits `MIDDLE`; absence emits `DEFAULT`. `mode: LEFT` on a BLIND edge is still rejected (wrong-type diagnostic).
- [ ] **AC6** — Generated `EdgeInfo` has exactly 8 fields, `line_follow_mode_override: LineFollowMode` last; `test_header_writer.py`'s field-count assertion updated 7→8 and last-field check updated; NO `offsetof` static_assert added (the pytest freezing its absence stays green); an `--warnings all` build is clean (no `missing-field-initializers`).
- [ ] **AC7** — LINE `mode` is per-directed-edge: authoring `mode: LEFT` on `u→v` does NOT mirror to `v→u`; BLIND `mode` still mirrors to both directions (graph_builder + validator tests).
- [ ] **AC8** — `makeEdge(...)` is emitted for every generated `EDGES` cell AND used by `FieldMap::noEdge()` + the 2 hand-authored fixtures; the emitted `constexpr`-canary `static_assert` compiles; the `arduino-cli` size check shows the `EDGES` array in program storage (flash), `.data`/`.bss` (RAM) unchanged within tolerance vs. an 8-field aggregate baseline.
- [ ] **AC9** — `resolve_line_follow_mode(edge, cfg)` resolves DEFAULT→MIDDLE (directional) in the sim; the `line_follow`/`projection_follow` parity records carry `follow_mode`, `line_mode` is renamed `speed`, `PARITY_SCHEMA_VERSION` is 3 on both C++ and Python, and the parity trace-diff CI job is green.
- [ ] **AC10** — CTPS renders a parity-safe lateral hug-side offset for a LEFT/RIGHT leg (visualization-only nudge; the `LocalizationSim` pose is unchanged); the CTPS mock accepts `setLineFollowOffset(...)` so `ctps_dump` runs; the mode threads through the recorder + loader-whitelist + renderer 3-place contract.
- [ ] **AC11** — Editor Mode bar offers `MIDDLE`/`LEFT`/`RIGHT` for LINE edges + a help note that LINE mode is directional (not BLIND-style symmetric); round-trips through `to_json_dict`/`to_module_proxy` + `validate()`.
- [ ] **AC12** — `build_routes.py` emits a loud warning if a LINE edge with `mode: LEFT|RIGHT` is traversed in reverse by any `NEXT_HOP` route with no companion authored entry.
- [ ] **AC13** — `uv run python tools/path_planner/build_routes.py` regenerates `routes.h/.cpp`; the routes-freshness CI check passes. Full `pytest`, full host `ctest` (all targets), `arduino-cli compile`, and the parity job are all green.
- [ ] **AC14** — ADR-069 committed (supersedes ADR-064's surface-only clause; records the setpoint-bias law, directional+travel-relative semantics, makeEdge full adoption + size gate, parity v3, CTPS parity-safe render, accepted axle-offset approximation, bench items); DECISIONS.md index + status table + ADR-064 cross-link/status updated; #156 closed.

## Data Model

### Enums

`tools/path_planner/pipeline_config.py::LineFollowMode` already exists (Python validator vocabulary). This PR **emits it to C++** for the first time:

```python
class LineFollowMode(IntEnum):   # now emitted into generated routes.h
    DEFAULT = 0   # inherit == MIDDLE (no PIPELINE_CONFIG default)
    MIDDLE  = 1   # centered follow  (offset 0)
    LEFT    = 2   # chassis rides left of line  (offset = +LINE_FOLLOW_OFFSET_ERROR)
    RIGHT   = 3   # chassis rides right of line (offset = -LINE_FOLLOW_OFFSET_ERROR)
```

Add `LineFollowMode` to `header_writer._MODE_ENUMS` so `enum class LineFollowMode : uint8_t { ... }` auto-emits into `namespace Routes` (routes.h) alongside `SpeedTier`/`BlindMode`.

### `EdgeInfo` (generated `routes.h` + 3 shadow-routes fixture headers) — grows 7 → 8 fields

```cpp
struct EdgeInfo {
    EdgeType       type;
    float          distance_cm;
    float          heading_deg;
    SpeedTier      speed_override;
    BlindMode      blind_mode_override;
    DirectionMode  direction_override;
    StopAtLineMode stop_at_line_override;
    LineFollowMode line_follow_mode_override;   // NEW, last field
};
```

Cell size 16 → 20 bytes (5 trailing 1-byte enums padded to 4-byte struct alignment). `1521 * 20 ≈ 30 KB` in flash `.rodata` (256 KB flash) — no SRAM impact. No `offsetof` static_assert (a pytest freezes its absence).

### `makeEdge()` factory (emitted into `routes.h` + each shadow-routes fixture header)

```cpp
constexpr EdgeInfo makeEdge(
    EdgeType       type,
    float          distance_cm            = 0.0f,
    float          heading_deg            = 0.0f,
    SpeedTier      speed_override          = SpeedTier::DEFAULT,
    BlindMode      blind_mode_override     = BlindMode::DEFAULT,
    DirectionMode  direction_override      = DirectionMode::DEFAULT,
    StopAtLineMode stop_at_line_override   = StopAtLineMode::DEFAULT,
    LineFollowMode line_follow_mode_override = LineFollowMode::DEFAULT)
{
    return EdgeInfo{ type, distance_cm, heading_deg, speed_override,
                     blind_mode_override, direction_override,
                     stop_at_line_override, line_follow_mode_override };
}
// Constexpr-evaluability canary (keeps the const EDGES array in flash .rodata):
static_assert(makeEdge(EdgeType::NONE).type == EdgeType::NONE,
              "makeEdge must be constexpr-evaluable");
```

Emitter emits `makeEdge(EdgeType::LINE, 17.2f, 90.0f, ... )` per cell; a `NONE` cell is `makeEdge(EdgeType::NONE)`. Single-return, C++14-compatible (MSVC host tests compile it).

### `Config` (`ConfigLine.h`; assert in `ConfigChecks.h`)

```cpp
// ConfigLine.h
constexpr float LINE_FOLLOW_OFFSET_ERROR = 1.5f;   // centroid-index units; LEFT=+, RIGHT=-. TODO: bench-tune.
// ConfigChecks.h
static_assert(LINE_FOLLOW_OFFSET_ERROR >= 0.0f && LINE_FOLLOW_OFFSET_ERROR <= 1.5f,
              "LINE_FOLLOW_OFFSET_ERROR must stay in [0,1.5] error units (>=2 units margin, under LINE_MAX_CORR)");
```

## Contracts & Interfaces

Single source of truth for every cross-task seam. Conform byte-for-byte.

### Names (must match across Python, C++, editor, fixtures)
- Enum: `LineFollowMode { DEFAULT, MIDDLE, LEFT, RIGHT }` — Python `pipeline_config.LineFollowMode` (source), generated C++ `Routes::LineFollowMode` (emitted).
- `EdgeInfo` field: `line_follow_mode_override` (8th, last).
- JSON key: `mode` (LINE vocab `MIDDLE|LEFT|RIGHT`; directional for LINE, bidirectional for BLIND — same key, type-switched symmetry).
- Config: `Config::LINE_FOLLOW_OFFSET_ERROR` (float, error units).
- Factory: `Routes::makeEdge(...)` (8 params, trailing defaults).
- Parity wire fields: `follow_mode` (new), `speed` (renamed from `line_mode`); `PARITY_SCHEMA_VERSION = 3`.

### Signatures
- `float lineFollowModeToSetpoint(Routes::LineFollowMode mode)` in `WRO2026/src/util/WROMath.h` — returns `+LINE_FOLLOW_OFFSET_ERROR` (LEFT), `-` (RIGHT), `0` (MIDDLE/DEFAULT), forward-frame. Owner: T-wromath; consumers: T-pathtracker.
- `LineFollowerEnableBuilder& setLineFollowOffset(float offset)` + owner `_lineFollowOffset` (float, default 0) + `_hasLineFollowOffsetOverride` — `WRO2026/src/drive/LineFollower.{h,cpp}`. Applied only on the online non-junction P+D tick. Owner: T-linefollower; mirrored in all mocks by T-mocks. LineFollower stays nav-agnostic (float only, no `Routes::` param).
- `Routes::LineFollowMode PathTrackerClass::_resolveLineFollowMode(const Routes::EdgeInfo& edge)` in `PathTracker.cpp` (DEFAULT→MIDDLE; NO pipeline default) — sibling of `_resolveSpeed`. Threaded DIRECTLY into the two LINE builder chains via `.setLineFollowOffset(sign * lineFollowModeToSetpoint(mode))` where `sign = decision.reverse ? -1 : +1`. NOT added to `EdgeDispatchQuery`/`EdgeDispatchDecision` (the frozen 7-field decision + truth table are untouched). Owner: T-pathtracker.
- Header emit `_resolve_overrides(edge_attrs) -> (speed, blind, direction, stop_at_line, line_follow_mode)` (now a 5-tuple; routes LINE `mode`→line_follow_mode_override, ignores BLIND `mode` for this slot). Owner: T-headerwriter; consumers: T-regen, T-fixtures.
- Sim `resolve_line_follow_mode(edge, cfg) -> LineFollowMode` in `sim/runtime.py` (DEFAULT→MIDDLE, directional) — sibling of `resolve_blind_mode`. Owner: T-sim.
- graph_builder: LINE `mode` moves to directional handling; BLIND `mode` stays in `_BIDIRECTIONAL_OPTIONAL_KEYS`. The `mode` key's symmetry is now edge-type-conditional. Owner: T-graph.
- Parity: `_ptEmitLineFollow(reverse, speed, follow_mode)` + `_ptEmitProjectionFollow(...)` (C++ `PathTracker.cpp`/`PathTrackerParityTrace.h`) and Python `_make_line_follow`/`_make_projection_follow` + `TRACE_RECORD_FIELDS` (`sim/parity.py`) move in lockstep to schema v3. Owner: T-parity.
- CTPS: resolved `line_follow_mode` recorded via `ctps_record` Record field → loader whitelist → renderer + `track_builder` visualization-only lateral nudge; `ctps_mocks/LineFollower.h` gets a recording `setLineFollowOffset`. Owner: T-ctps.

### File ownership
| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/config/ConfigLine.h`, `config/ConfigChecks.h` | T-config | T-linefollower, T-wromath |
| `WRO2026/src/util/WROMath.h` | T-wromath | T-pathtracker |
| `WRO2026/src/drive/LineFollower.{h,cpp}` | T-linefollower | T-mocks (surface) |
| `WRO2026/src/mission/PathTracker.{h,cpp}`, `PathTrackerParityTrace.h` | T-pathtracker | T-parity |
| `WRO2026/src/nav/FieldMap.h` (`noEdge`) | T-noedge | — |
| `WRO2026/src/nav/routes.{h,cpp}` (generated) | T-regen | firmware compile |
| `tools/path_planner/validator.py` | T-validator | — |
| `tools/path_planner/graph_builder.py` | T-graph | — |
| `tools/path_planner/header_writer.py` | T-headerwriter | T-regen, T-fixtures |
| `tools/path_planner/sim/runtime.py` | T-sim | T-parity |
| `tools/path_planner/sim/parity.py` | T-parity | — |
| `tools/path_planner/build_routes.py` | T-lint | — |
| `WRO2026/test/fixtures/routes_fixture.{h,cpp}` (regen), `routes_speed_chain.{h,cpp}`, `routes_reset_passthrough_forced.{h,cpp}` | T-fixtures | host tests |
| `WRO2026/test/{path,task,mission,ctps}_mocks/LineFollower.h` (+ verify `mocks/`) | T-mocks | host targets |
| `WRO2026/test/*.cpp` + `CMakeLists.txt` | T-hosttests (owns CMake appends) | — |
| `tools/path_planner/tests/test_*.py` + goldens | T-pytests | — |
| `tools/path_planner/ctps/*` + `ctps_mocks/LineFollower.h` + `layout.json` | T-ctps | — |
| `tools/path_planner/editor/panels.py` | T-editor | — |
| `docs/decisions/069-*.md`, `DECISIONS.md` | T-adr | — |

### Naming
- LINE `mode` tags: `MIDDLE`, `LEFT`, `RIGHT` (exact). BLIND `mode` unchanged. Retired: the issue-#156 `LEFT/RIGHT` reject + its `# TODO(issue #156)` comment.

## Error Handling

- **`mode: LEFT|RIGHT` on a BLIND edge** → still rejected (wrong-type diagnostic naming the valid BlindMode tags) — unchanged behavior; only the LINE-side guard is removed.
- **`mode` unknown tag on LINE** → precise diagnostic listing `MIDDLE|LEFT|RIGHT`.
- **Biased LINE edge traversed in reverse with no companion entry** → `build_routes.py` prints a loud warning naming the edge (build succeeds; it is an authoring smell, not a hard error).
- **`makeEdge` not constexpr-evaluable** → the emitted `static_assert` canary fails the build; the CI size gate additionally fails if `EDGES` moved to RAM.
- **Sensor dropout / junction during a biased follow** → unchanged: the P+D block runs only when `isOnline()` (which implies a non-zero centroid, since `_isOnlineFromCache` and `_computeError` share the `> LINE_CHANNEL_ON_RATIO` rule), so a zero-weight read never reaches the biased P term; junction-freeze runs symmetric cruise; lost-line dispatches `LOST_LINE_ACTION`/overshoot dead-reckon.

## Testing Strategy

**Levels:** Unit (validator, header emit, graph symmetry, sim resolve, WROMath map, editor), Integration (host FSM ctest, LineFollower behavior ctest, arduino compile, routes freshness, flash-size gate), Parity (sim↔firmware trace-diff v3), Visualization (CTPS).

| ID | Test Case | Type | Expected |
|----|-----------|------|----------|
| TC1 | LEFT bias, mocked `getError()` sweep | Unit (C++, real LineFollower) | L/R reflect `corr = KP*(err-1.5)+KD*(err-lastErr)`; RIGHT mirrors; MIDDLE == baseline |
| TC2 | Mid-follow `MIDDLE→LEFT` mode change | Unit (C++) | P steps by ≈KP*offset; D term has NO kick (`_lastErr` raw); `getRawError()` still raw |
| TC3 | LEFT + reverse (rear board) | Unit (C++) | `PathTracker` passes negated offset (`setLineFollowOffset` receives `-1.5*sign`) |
| TC4 | Lifecycle: cold enable / re-arm / disable | Unit (C++) | cold→0; re-arm preserves; disable resets 0 (no stale bias into `followDistance`) |
| TC5 | `mode: LEFT` LINE emit | Unit (py) | `EdgeInfo` cell `line_follow_mode_override = LineFollowMode::LEFT`; RIGHT/MIDDLE/absent likewise |
| TC6 | `mode: LEFT` on BLIND | Unit (py) | Rejected (wrong-type) |
| TC7 | LINE `mode` directional; BLIND `mode` bidirectional | Unit (py) | LINE `u→v` LEFT not on `v→u`; BLIND `mode` on both |
| TC8 | `EdgeInfo` 8 fields + makeEdge | Unit (py) + C++ | field-count 8, last = line_follow_mode_override; no offsetof assert; canary compiles; `--warnings all` clean |
| TC9 | Flash-size gate | Integration | `arduino-cli` reports `EDGES` in program storage; RAM `.data/.bss` ~unchanged vs 8-field aggregate baseline |
| TC10 | `resolve_line_follow_mode` | Unit (py) | DEFAULT→MIDDLE; LEFT/RIGHT pass through; directional |
| TC11 | Parity v3: line_follow/projection_follow | Parity | `follow_mode` present, `line_mode`→`speed`, schema 3, trace-diff green |
| TC12 | build_routes reverse-traversal lint | Unit (py) | Warning fires for a used unpaired biased reverse traversal; silent otherwise |
| TC13 | CTPS parity-safe render | Unit (py) + eyeball | Lateral nudge drawn on a LEFT/RIGHT leg; `LocalizationSim` pose unchanged; `ctps_dump` runs |
| TC14 | Editor Mode bar LEFT/RIGHT + round-trip | Unit (py) | LEFT/RIGHT selectable; help note present; `to_json_dict`→`validate`→`to_module_proxy` clean |
| TC15 | Routes freshness + full pytest + full ctest + arduino compile | Integration | `build_routes` then `git diff --exit-code` clean; all suites green |

**Test data:** add LEFT/RIGHT LINE cells to `routes_speed_chain` (the per-edge fixture) + a Python fixture; regenerate header goldens (`test_header_writer.py` `regenerate_goldens()`) + `routes_fixture` (`regenerate_routes_fixture.py`) + `routes_toy`. **Run:** `uv run --extra dev pytest tools/path_planner/tests/`; `cmake --build build/test && ctest --test-dir build/test`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`. Worktree: `uv sync --project tools/path_planner` first (worktrees don't copy `.venv`).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T-config | Config constant + assert | — | low | `ConfigLine.h`, `ConfigChecks.h` | Add `LINE_FOLLOW_OFFSET_ERROR = 1.5f` + the `[0,1.5]` `static_assert` (ConfigChecks last). Satisfies AC (config surface). |
| T-validator | Remove LEFT/RIGHT guard | — | med | `validator.py` | Delete the `# TODO(issue #156)` LEFT/RIGHT reject (validator.py:1245-1252); accept `MIDDLE\|LEFT\|RIGHT` on LINE; keep BLIND wrong-type reject. Satisfies AC5. |
| T-graph | Directional LINE mode | — | med | `graph_builder.py` | Split `mode` symmetry by edge type: LINE `mode` → directional (forward-only, like `direction`); BLIND `mode` stays bidirectional. Satisfies AC7. |
| T-headerwriter | Emit enum + field + makeEdge | — | high | `header_writer.py` | Import `LineFollowMode`; add to `_MODE_ENUMS`; `_LINE_FOLLOW_MODE_DEFAULT` sentinel; `_resolve_overrides` → 5-tuple + LINE `mode` branch; emit 8th struct field; emit `constexpr makeEdge(...)` + canary; rewrite `_edge_cell`/`_none_edge_cell` to emit `makeEdge(...)` for ALL cells (full adoption). Satisfies AC5,AC6,AC8. |
| T-sim | Sim resolver | — | high | `sim/runtime.py` | Add `resolve_line_follow_mode(edge,cfg)` (DEFAULT→MIDDLE, directional); thread into LINE HopPlan/dispatch. Satisfies AC9. |
| T-wromath | Mode→setpoint helper | T-config, T-regen | low | `WROMath.h` | `float lineFollowModeToSetpoint(Routes::LineFollowMode)` reading `LINE_FOLLOW_OFFSET_ERROR`. Host-testable. Satisfies AC1. |
| T-linefollower | Offset bias in LineFollower | T-config | high | `LineFollower.{h,cpp}` | Add `_lineFollowOffset` (float,0) + `_hasLineFollowOffsetOverride`; `setLineFollowOffset(float)` builder knob + apply-path (cold→0, re-arm→preserve, disable→reset 0, mirror `setPreset`/`setReverse` override-pair); in `update()` online non-junction P+D apply `corr = KP*(err - _lineFollowOffset) + KD*(err - _lastErr)`, `_lastErr` raw. Satisfies AC1,AC2,AC4. |
| T-regen | Regenerate routes | T-validator,T-graph,T-headerwriter,T-sim | med | `routes.{h,cpp}` | `uv run python build_routes.py`; commit `routes.h/.cpp`. Freshness gate. Unblocks firmware/WROMath compile. |
| T-noedge | FieldMap noEdge → makeEdge | T-regen | low | `FieldMap.h` | `noEdge()` → `Routes::makeEdge(Routes::EdgeType::NONE)`. |
| T-fixtures | Host shadow-routes fixtures | T-headerwriter,T-regen | high | `test/fixtures/routes_{fixture,speed_chain,reset_passthrough_forced}.{h,cpp}` | Regenerate `routes_fixture` (`regenerate_routes_fixture.py`); hand-edit the 2 hand-authored fixtures: add 8th struct field + `LineFollowMode` enum + `makeEdge` factory to each shadow header, convert cells to `makeEdge(...)`, add a LEFT/RIGHT case to `routes_speed_chain`; regen `routes_toy` golden. Resolve real-routes nodes by NAME. |
| T-pathtracker | Resolve + thread + parity emit | T-wromath,T-linefollower,T-regen | high | `PathTracker.{h,cpp}`, `PathTrackerParityTrace.h` | `_resolveLineFollowMode(edge)` (DEFAULT→MIDDLE); chain `.setLineFollowOffset(sign*lineFollowModeToSetpoint(mode))`, `sign=decision.reverse?-1:+1`, into `LINE_FOLLOW_INTERSECTION` (2680) + `PROJECTION_FOLLOW` (2712); do NOT add to `EdgeDispatchQuery`/`Decision`. Parity: `_ptEmitLineFollow`/`_ptEmitProjectionFollow` gain `follow_mode`, rename `line_mode`→`speed`, bump schema v3. Satisfies AC1,AC3,AC9. |
| T-mocks | LineFollower mock surface | T-linefollower | low | `test/{path,task,mission,ctps}_mocks/LineFollower.h` (+ verify `mocks/`) | Add `setLineFollowOffset(float)` to each mock builder (no-op; ctps mock records). Keeps every PathTracker/tasks/CTPS target compiling. |
| T-parity | Python parity v3 | T-sim,T-pathtracker | high | `sim/parity.py` | `TRACE_RECORD_FIELDS`: add `follow_mode`, rename `line_mode`→`speed` on `line_follow`/`projection_follow`; `_make_*` + normalizer emit `follow_mode` from `resolve_line_follow_mode`; `PARITY_SCHEMA_VERSION=3`. Satisfies AC9. |
| T-lint | Reverse-traversal lint | T-graph,T-regen | med | `build_routes.py` | Warn when a `NEXT_HOP`-used reverse traversal of a `LEFT/RIGHT` LINE edge has no companion authored entry. Satisfies AC12. |
| T-ctps | CTPS parity-safe render | T-pathtracker | high | `ctps/*`, `ctps_mocks/LineFollower.h`, `layout.json` | Record `line_follow_mode` (recorder Record field + loader whitelist + renderer); visualization-only lateral chassis nudge on the biased leg (ADR-054 pattern, `track_builder`), offset→cm display constant; `LocalizationSim` pose untouched; ctps mock records `setLineFollowOffset`. Satisfies AC10. |
| T-editor | Editor Mode bar | T-validator | med | `editor/panels.py` | `_LINE_MODE_TAGS += (LEFT, RIGHT)`; directional help note. Satisfies AC11. |
| T-hosttests | Host ctest + size gate + arduino | T-pathtracker,T-fixtures,T-mocks,T-regen | high | `test/*.cpp`, `CMakeLists.txt` | Add TC1-TC4 (real LineFollower.cpp target), TC8 layout, TC9 flash-size gate, PathTracker thread test + `routes_speed_chain` LEFT/RIGHT case; adapt every EdgeInfo-touching target; full `ctest` + `arduino-cli compile`. Orchestrator owns CMake appends. Satisfies AC1-4,AC6,AC8,AC13. |
| T-pytests | Python tests + goldens | T-validator,T-graph,T-headerwriter,T-sim,T-parity,T-lint,T-fixtures,T-editor | med | `tests/test_*.py`, goldens | TC5-TC7,TC10-TC14; flip `test_field_data.py` TC7 rejection→acceptance; update `test_header_writer.py` 7→8 field-count; regenerate goldens; full `pytest`. Satisfies AC5-7,AC9-14. |
| T-adr | ADR-069 + close issue | T-validator,T-headerwriter,T-linefollower,T-pathtracker | low | `docs/decisions/069-*.md`, `DECISIONS.md` | Write ADR-069 (verify no 069 collision first); supersede ADR-064 surface-only clause; update index/status table/cross-links; close #156. Satisfies AC14. |

## Notes for Implementer

- **Worktree first:** work in `/Users/diego/Documents/WRO2026/wt-156-lateral-offset` (branch `feat/156-lateral-offset-line-follow`, off main). Run `uv sync --project tools/path_planner` there — worktrees don't copy the gitignored `.venv` (`[[gotcha-worktree-venv-uv-sync]]`).
- **The dropout phantom is already prevented** — do NOT add a redundant guard: the biased P term runs only inside the `runOnlinePath` block, and `isOnline()` ⟹ some channel `> LINE_CHANNEL_ON_RATIO` ⟹ `_computeError` totalWeight > 0, so `getError()` there is a real centroid, never the zero-weight 0. Just keep the offset out of the junction-freeze (symmetric) and lost-line (offline) branches.
- **`_lastErr` stays raw.** The offset touches ONLY the P term. This is what preserves `getRawError()` AND kills the mode-change D-kick (Codex). Do not fold the offset into `_lastErr` or the derivative.
- **LineFollower takes a float, not the enum** — keep the drive layer free of any `Routes::` include. `PathTracker` (nav-aware) does mode→float via `WROMath.lineFollowModeToSetpoint` and composes the reverse sign; parity/CTPS record the mode enum from `PathTracker`.
- **EdgeInfo fan-out (`[[gotcha-edgeinfo-field-addition]]`):** the 8th field touches BOTH emitter cell builders (`_edge_cell` + `_none_edge_cell`), `FieldMap::noEdge()`, and all 3 shadow-routes fixture tables (struct def + every cell + each fixture's own `makeEdge`/`LineFollowMode`). No `offsetof` assert exists — do NOT add one; a pytest freezes its absence.
- **Fixture/parity fan-out needs the FULL suites** — a new emitter-read field breaks `SimpleNamespace` fixtures across several modules and a validator-rule change breaks BOTH Python and host-C++ fixtures; only the full `pytest` AND full `ctest` surface them all (`[[gotcha-strict-emitter-field-fixtures]]`, `[[gotcha-field-contract-change-breaks-cpp-and-py-fixtures]]`). Host FSM tests resolve real-routes nodes by NAME (`[[gotcha-host-tests-link-production-routes]]`).
- **makeEdge must stay `constexpr` + single-return** (MSVC host tests compile it). The canary `static_assert` + the CI `.data`/`.bss` size check are the two mechanical guards that a mistake demotes to a build error, not a silent flash→RAM regression / brick.
- **Parity v3 is a lockstep bump** — the C++ emitters (`PathTrackerParityTrace.h` + `PathTracker.cpp` `_ptEmit*`) and Python (`sim/parity.py` `TRACE_RECORD_FIELDS` + `_make_*`) must land the same field-set + version in one change, or `ParitySchemaError`/version-mismatch reddens the parity job (`[[gotcha-parity-record-kind-is-discriminator]]`).
- **CTPS render is visualization-only** — the lateral nudge lives in the render/`track_builder` layer (ADR-054 pivot-wheel-arc precedent). A raw x/y write must NEVER reach `LocalizationSim` (`[[gotcha-sim-chassis-axle-on-node]]`). Thread the mode through the full 4-surface CTPS knob contract (mock → record → loader → renderer) so it is not silently dropped (`[[gotcha-ctps-mock-noop-setter-drops-builder-knob]]`).
- **CMake append coordination:** the orchestrator owns `CMakeLists.txt` edits; a new real-`.cpp`-linked target fans out to many targets (`[[gotcha-parallel-execute-cmakelists-append-coordination]]`).
- **Bench items (post-merge, on hardware):** verify LEFT/RIGHT polarity on front AND rear boards; bench-gate `getJunctionType()` classification + `isLineLost()` while riding at full offset (the crossbar channel pattern shifts); tune `LINE_FOLLOW_OFFSET_ERROR` on the mat. The ≈1–1.5 cm biased-arrival axle offset is accepted, within junction tolerance, and NOT compensated in `Localization`/sim.
- **ADR number:** verify no branch has landed 069 at author time; renumber if so (`[[gotcha-adr-number-collision]]`).
- **Rollback:** additive — revert = restore the validator guard + drop the field/enum/makeEdge + the `SPEED_NORMAL`-style regenerate; no data migration (the real field has no LINE `mode` overrides yet).
