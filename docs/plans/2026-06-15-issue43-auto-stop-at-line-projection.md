# Auto stop-at-line on a BLIND hop that precedes a PROJECTION follow (issue #43)

**Goal:** Make a BLIND hop automatically run a stop-at-line seek when the hop that
follows it (resolved at runtime via `NEXT_HOP[next][goal]`) is a `PROJECTION` edge, so
the chassis re-anchors on a real line before the projection follow begins instead of
dead-reckoning into it. On by default, with a field-wide toggle and per-edge overrides.

**Approach:** Extend the single shared resolver (`StopAtLinePolicy.h::resolveStopAtLine`
+ its byte-parity Python mirror `sim/runtime.py::resolve_stop_at_line`) with one new
input `auto_rule_active` and one new output `soft_fallback`, and switch its argument list
to an input struct. The caller (C++ `PathTracker` blind dispatch, Python `_walk_hops`,
offline `compile_path`) owns the projection peek and folds the field-wide toggle into
`auto_rule_active = auto_seek_enabled && next_hop_is_projection` — so the resolver stays
route-agnostic. Add a dedicated per-edge `stop_at_line` override to `EdgeInfo`, a
field-wide toggle in `field_data.json` emitted as `Routes::BLIND_PROJECTION_AUTO_SEEK`,
and a Config-gated soft-fallback (default off / hard-fault) for auto-triggered seeks that
miss the line. Regenerate routes, extend the truth table via a generator, and add an
end-to-end synthetic fixture. GUI authoring is deferred to issue #52.

## Scope

- **In scope:**
  - New resolver precedence (highest first): call-site `StopAtLine ON/OFF` → per-edge
    dedicated `stop_at_line` override `ON/OFF` → legacy `BlindMode::STOP_AT_LINE` force-on
    → auto-rule (`auto_rule_active`) → pipeline blind default.
  - Resolver signature: input struct `StopAtLineQuery` (adds `edgeStopAtLineOverride`
    + `autoRuleActive`); output struct gains `soft_fallback`. C++ and Python kept
    step-for-step identical.
  - `next_hop_is_projection` computed at the C++ blind dispatch (mirror the existing
    pass-through next-hop peek), in the Python sim `_walk_hops`, and in `compile_path`.
  - New `Routes::StopAtLineMode { DEFAULT, ON, OFF }` enum + `EdgeInfo.stop_at_line_override`
    field with full pipeline plumbing (pipeline_config, validator, graph_builder,
    header_writer, FieldMap::noEdge). Authored in `field_data.py` EDGES (Python) only.
  - Field-wide toggle: top-level bool `blind_projection_auto_seek` in `field_data.json`
    (default true), read by `field_data.py`, emitted as standalone constant
    `Routes::BLIND_PROJECTION_AUTO_SEEK`.
  - Soft-fallback for auto-seeks: `Config::STOP_AT_LINE_FAULT_FALL_BACK` (default
    **false** = hard-fault). When true, an auto-triggered seek that misses the line
    degrades to dead-reckon-to-node instead of faulting. The soft seek caps its max
    distance at the node, but because the post-detect overshoot roll runs ONLY on
    detection (never on a fault), a no-line fault stops with the *sensor* — not the axle —
    at the cap, leaving the axle ~`LINE_SENSOR_AXLE_OFFSET_CM` short of the node. The soft
    recovery must therefore reconcile the axle to the node with an explicit corrective
    advance (bench-derived geometry), not rely on the cap alone. Explicit/ON/legacy seeks
    always hard-fault.
  - Decoupled semantics: motion mode (`SYNC`/`GYRO`/`RAW`) is orthogonal to the seek
    decision; only the dedicated per-edge OFF or call-site OFF suppresses the auto-rule.
  - Truth-table generator emitting the full-combinatorial CSV (864 rows) + updated TC13
    (Python) + updated C++ resolver tests. New end-to-end synthetic fixture (blind →
    projection) with sim/compiler parity.
  - Regenerate `routes.h`/`routes.cpp`; keep CI green; new ADR; comment on issue #52.

- **Out of scope:**
  - Tk editor GUI for the toggle or per-edge override — deferred to issue #52 (the
    dedicated `EdgeInfo` field is added now precisely so #52 is just UI wiring).
  - Putting per-edge overrides into `field_data.json` (EDGES-in-JSON is blocked by
    ADR-022 enum representation; part of #52).
  - Removing/deprecating `BlindMode::STOP_AT_LINE` (kept for backward compat).
  - Reverse rear-board physical bench, pivot/bidirectional logic, new motion modes.
  - Flipping `STOP_AT_LINE_FAULT_FALL_BACK` on for competition (a later, deliberate flip).

## Decisions

- **Resolver stays route-agnostic via `auto_rule_active`** — the caller computes
  `auto_seek_enabled && next_hop_is_projection`; the resolver never learns the word
  "projection." (Opus consult #1: shrinks resolver inputs and truth table; matches the
  existing pass-through-peek pattern where the peek lives at the call site.)
- **Resolver uses an input struct `StopAtLineQuery`** — 7 inputs is past the project's
  3+-knob threshold; a POD input → existing `StopAtLineResolution` output. (Opus #2;
  matches the user's fluent/struct preference; lets the CSV reader bind by column name.)
- **Dedicated per-edge `stop_at_line` override (new `EdgeInfo` field), not the BlindMode
  tag** — decouples seek from motion so a future GUI maps one field = motion, one field
  = seek. Kept this issue (not deferred) per the user's choice to pay the abstraction now
  for smoother #52 work. *(Opus #6 flagged this as the highest-churn, no-current-author
  part and suggested deferring; overridden deliberately by the user.)* Decoupling is
  **partial**: the legacy `BlindMode::STOP_AT_LINE` motion tag still force-seeks (kept for
  backward compat), so motion and seek are not 100% orthogonal. The dedicated
  `stop_at_line=OFF` rung sits ABOVE the legacy tag, so OFF suppresses a legacy
  force-on. The ADR records this partial-decoupling honestly.
- **Per-edge `stop_at_line` is DIRECTIONAL, not mirrored** — like `direction_override`
  (baked onto the forward `u→v` edge only, never mirrored to `v→u`; see the
  single-direction EDGES convention). The seek-before-projection intent is inherently
  directional (whether the next hop is a projection depends on travel direction), so
  mirroring would wrongly suppress/force the seek on the reverse hop. To control the
  reverse sense, author a swapped EDGES entry. (Critic Axis-1.)
- **Field-wide toggle in `field_data.json`, emitted as standalone `Routes::BLIND_PROJECTION_AUTO_SEEK`**
  — it travels with the field (not firmware), so it belongs in generated `routes.h`, not
  `Config.h` (unlike the firmware-sourced `PATH_PASS_THROUGH_ENABLED` /
  `BIDIRECTIONAL_AUTO_ENABLED`). Standalone constant, not a `PipelineConfig` field
  (PipelineConfig is a 5-enum motion bundle whose `__post_init__` rejects non-enum/DEFAULT
  members). The ADR records why it diverges from the Config.h bools. (Opus #5.)
- **Soft-fallback default OFF (`STOP_AT_LINE_FAULT_FALL_BACK = false`)** — during
  development (not at comp) the team wants loud hard faults to surface problems; the soft
  path is implemented but gated off, to be flipped on at competition for graceful
  degradation. The max-distance cap removes the `MAX_MARGIN` over-search, but does NOT
  place the axle on the node: on fault there is no overshoot roll and the sensor leads the
  axle by `LINE_SENSOR_AXLE_OFFSET_CM`, so the recovery owes an explicit corrective
  advance of that offset (exact value bench-verified) before `onDeadReckonedAdvance`.
  (User directive + Opus GATE 1, corrected by the critic/judge after tracing `Drive.cpp`
  detect/overshoot geometry.)
- **Source-parity test for the toggle** — a dedicated test asserts the emitted
  `Routes::BLIND_PROJECTION_AUTO_SEEK` literal equals `field_data`'s bool, on top of the
  CI routes-freshness gate, so C++ and the Python sim can't silently feed the resolver
  different toggle values. (Opus GATE 2.)
- **Truth table: full-combinatorial 864-row CSV from a checked-in generator** — the
  generator encodes the precedence as an ordered rule list independent of the resolver's
  if/elif, reproduces the existing 144 rows as a slice, and is the normative source TC13
  and the C++ hand cases both transcribe. Exhaustive (no future coverage gap) + a real
  independent check. (User: "which is best… don't want it to break one day"; Opus #7
  sizing.)

## Acceptance Criteria

- [ ] A blind hop whose resolved next hop is `PROJECTION` runs a stop-at-line seek by
      default (toggle on, no per-edge/call override). [TC-AUTO-FRONT, TC-E2E]
- [ ] BLIND→LINE, BLIND→BLIND, and a final blind hop (no next hop) do NOT trigger the
      auto-rule. [TC-NOAUTO, TC-E2E-final]
- [ ] Field-wide toggle off (`blind_projection_auto_seek=false`) disables the rule
      field-wide and restores exact dead-reckon behavior. [TC-TOGGLE-OFF]
- [ ] Per-edge `stop_at_line=OFF` and call-site `StopAtLine::OFF` win over the auto-rule
      (no seek) and over a legacy `BlindMode::STOP_AT_LINE` tag; per-edge `ON` / call `ON`
      force the seek. [TC-PRECEDENCE]
- [ ] Board gate respected: a REAR auto-seek needs `LINE_BOARD_COUNT>=2`, else degrades
      to dead-reckon reverse. [TC-REAR-1BOARD, TC-REAR-2BOARD]
- [ ] Soft-fallback: with `STOP_AT_LINE_FAULT_FALL_BACK=false` (default) an auto-seek that
      misses the line hard-faults `LINE_NOT_FOUND`; with it true the recovery calls
      `Drive.clearFault()`, issues an explicit corrective advance to reconcile the axle to
      the node (no overshoot rolls on a fault, and the sensor leads the axle), then
      `onDeadReckonedAdvance` + continues. [TC-SOFT-OFF, TC-SOFT-ON] (numeric axle-on-node
      geometry is a hardware bench item) [TC-AUTO-PIPELINE-STOP for full-domain parity]
- [ ] `resolveStopAtLine` (C++) and `resolve_stop_at_line` (Python) agree on every
      generated truth-table row, including `soft_fallback`. [TC13, TC13-CPP]
- [ ] `Routes::BLIND_PROJECTION_AUTO_SEEK` equals `field_data`'s bool (source parity).
      [TC-TOGGLE-PARITY]
- [ ] `routes.h`/`routes.cpp` regenerated and committed; toy goldens unchanged (no
      BLIND→PROJECTION pair added to the toy field); `build_routes` + `git diff --exit-code`
      CI gate green. [TC-FRESHNESS]
- [ ] Full pytest suite, host ctest, and `arduino-cli` compile all pass.

## Data Model

### New enum (`pipeline_config.py` + emitted into `routes.h`)

```python
class StopAtLineMode(IntEnum):
    """Per-edge stop-at-line override, decoupled from BlindMode (the motion bundle)."""
    DEFAULT = 0   # inherit: defer to legacy BlindMode tag / auto-rule / pipeline
    ON      = 1   # force the seek on this edge
    OFF     = 2   # force dead-reckon (no seek) on this edge
```

```cpp
// routes.h (emitted by header_writer)
enum class StopAtLineMode : uint8_t { DEFAULT = 0, ON = 1, OFF = 2 };
```

`stop_at_line_override` is a **directional** per-edge attribute: it is baked onto the
forward `u→v` directed edge only and NOT mirrored onto `v→u` (same convention as
`direction_override`). Control the reverse sense with a swapped EDGES entry.

### `EdgeInfo` (emitted into `routes.h`) — new field appended

```cpp
struct EdgeInfo {
    EdgeType       type;
    float          distance_cm;
    float          heading_deg;
    LineMode       line_mode_override;
    BlindMode      blind_mode_override;
    ProjectionMode projection_mode_override;
    DirectionMode  direction_override;
    StopAtLineMode stop_at_line_override;   // NEW (append; DEFAULT=0 = inherit)
};
```

### Field-wide toggle

- `field_data.json` top-level: `"blind_projection_auto_seek": true`
- `field_data.py`: `BLIND_PROJECTION_AUTO_SEEK = _data.get("blind_projection_auto_seek", True)`
- `routes.h` (emitted): `constexpr bool BLIND_PROJECTION_AUTO_SEEK = true;`

### Resolver contract (both languages, byte-parity)

```cpp
struct StopAtLineQuery {
    StopAtLine        callFlag;               // NavTypes call-site flag (AUTO/ON/OFF)
    StopAtLineMode    edgeOverride;           // Routes per-edge override (DEFAULT/ON/OFF)
    Routes::BlindMode edgeMode;               // resolved blind mode (never DEFAULT)
    Routes::BlindMode pipelineBlind;          // Routes::PIPELINE_CONFIG.blind
    Direction         direction;              // FRONT / REAR (never AUTO)
    uint8_t           lineBoardCount;         // Config::LINE_BOARD_COUNT
    bool              autoRuleActive;         // caller: autoSeekEnabled && nextHopIsProjection
};

struct StopAtLineResolution {
    bool              seek;
    Routes::BlindMode motionMode;             // never STOP_AT_LINE / DEFAULT
    bool              softFallback;           // NEW: true ONLY for auto-rule-triggered seeks
};
```

### Resolution algorithm (the single normative rule both languages + the generator encode)

```
auto_triggered = false
if    callFlag == ON:                      requested = true
elif  callFlag == OFF:                     requested = false
elif  edgeOverride == ON:                  requested = true
elif  edgeOverride == OFF:                 requested = false
elif  edgeMode == STOP_AT_LINE:            requested = true                 # legacy force-on
elif  autoRuleActive:                      requested = true; auto_triggered = true
else:                                      requested = false   # AUTO, no rule active: never seek.
                                                               # Pipeline-default seek is already handled by the
                                                               # resolved edgeMode==STOP_AT_LINE rung above (edgeMode
                                                               # is DEFAULT->pipeline resolved before the resolver).

board_ok      = (direction == FRONT) || (lineBoardCount >= 2)
seek          = requested && board_ok
soft_fallback = seek && auto_triggered

# motionMode (unchanged from today):
if seek:   motionMode = (edgeMode == STOP_AT_LINE) ? SYNC_TSHAPE : edgeMode
else:
    base = (edgeMode == STOP_AT_LINE) ? pipelineBlind : edgeMode
    motionMode = (base == STOP_AT_LINE) ? SYNC_TSHAPE : base
```

Existing 144 truth-table rows are the slice `edgeOverride=DEFAULT, autoRuleActive=false`
(→ `soft_fallback=false`). With the final rung returning `false`, the algorithm matches
the shipping resolver across its FULL domain (not just the fixture) — including the
degenerate `callFlag=AUTO, autoRuleActive=false, edgeMode != STOP_AT_LINE,
pipelineBlind=STOP_AT_LINE` case, which both old and new return `seek=false`. A truth-table
row asserts this. `pipelineBlind` still feeds the `motionMode` dead-reckon-degrade leg
(unchanged).

## Error Handling

- **Auto-seek misses the line (`LINE_NOT_FOUND`), `STOP_AT_LINE_FAULT_FALL_BACK=false`
  (default):** unchanged — the shared Drive-fault guard at the top of
  `PathTracker::update()` latches `DRIVE_FAULT` (loud halt). This is the dev-time default.
- **Auto-seek misses the line, toggle true:** the shared Drive-fault guard, before
  entering `DRIVE_FAULT`, detects `_state == STOP_AT_LINE && _activeSoftFallback &&
  Config::STOP_AT_LINE_FAULT_FALL_BACK && fault == LINE_NOT_FOUND`, then: (1) calls
  `Drive.clearFault()` explicitly (Drive latches the fault; without this the next move
  silently no-ops) and confirms the chassis is stopped; (2) issues an explicit corrective
  `Drive.driveDist(...)` for the residual axle-to-node distance — the soft seek capped its
  max at `distance_cm`, but no overshoot rolled on the fault and the sensor leads the axle
  by `LINE_SENSOR_AXLE_OFFSET_CM`, so the axle is short by ~that offset (exact correction
  bench-derived from `Drive`'s detect/overshoot geometry, not asserted); (3) asserts
  LineFollower is disabled (already disabled on the blind-seek path; replicate any other
  `_enterFault` teardown the bypass skips); (4) calls
  `Localization.onDeadReckonedAdvance(_nextHop)` and transitions to `PLAN_NEXT_HOP`.
- **Explicit / call-ON / legacy `BlindMode::STOP_AT_LINE` seek misses:** always hard-faults
  (`softFallback=false`), regardless of the Config toggle.
- **REAR auto-seek on a 1-board chassis:** `board_ok=false` → `seek=false` → dead-reckon
  reverse (`ACT_BLIND_REVERSE` offline; `Drive.driveDist` reverse on firmware). No fault.
  Note: `soft_fallback = seek && auto_triggered`, so a board-degraded REAR case (seek
  false) never carries `soft_fallback` — the soft path can only fire after `board_ok`
  passed.
- **Final blind hop / non-projection next hop:** `autoRuleActive=false` → no auto-seek.

## Testing Strategy

**Levels:** Unit (resolver both languages), Integration (sim/compiler parity on a fixture
graph), Build (routes freshness + arduino-cli compile).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC13 | Python `resolve_stop_at_line` over every generated CSV row | Unit | `(seek, motion, soft_fallback)` matches each row; row count == new total |
| TC13-CPP | C++ `resolveStopAtLine` over representative rows incl. all new dims | Unit | Matches the truth table; all hand cases updated to struct input |
| TC-AUTO-FRONT | AUTO edge, `autoRuleActive=true`, FRONT, board≥1 | Unit | `seek=true, soft_fallback=true, motion=pipeline/edge` |
| TC-NOAUTO | AUTO edge, `autoRuleActive=false` (next hop LINE/BLIND/none) | Unit | `seek=false, soft_fallback=false` (baseline slice) |
| TC-TOGGLE-OFF | `autoRuleActive=false` via toggle off, next hop IS projection | Unit | `seek=false` — identical to dead-reckon |
| TC-PRECEDENCE | call OFF>edge ON; edge OFF>auto; call ON>edge OFF; **edge OFF>legacy `BlindMode::STOP_AT_LINE` tag** | Unit | Higher rung wins; `soft_fallback=false` whenever a non-auto rung decides; edge OFF suppresses a legacy force-on |
| TC-AUTO-PIPELINE-STOP | `callFlag=AUTO, autoRuleActive=false, edgeMode != STOP_AT_LINE, pipelineBlind=STOP_AT_LINE` | Unit | `requested=false` (full-domain parity with the shipping resolver) |
| TC-REAR-1BOARD | auto-rule, REAR, board=1 | Unit | `seek=false` (degrade), `soft_fallback=false` |
| TC-REAR-2BOARD | auto-rule, REAR, board=2 | Unit | `seek=true, soft_fallback=true` |
| TC-SOFT-OFF | firmware: auto-seek miss, toggle false | Integration (host) | enters `DRIVE_FAULT` (no recovery) |
| TC-SOFT-ON | firmware: auto-seek miss, toggle true | Integration (host) | `Drive.clearFault()` invoked + corrective advance invoked + `onDeadReckonedAdvance` + `PLAN_NEXT_HOP`. Host test asserts the FSM transition + that `clearFault` and the corrective advance are CALLED; it CANNOT verify the geometry (sim never models a missing line / sensor-axle offset) — numeric axle-on-node correctness is a bench item |
| TC-E2E | synthetic graph blind→projection: sim `go_to` vs `compile_path` | Integration | Both emit `ACT_STOP_AT_LINE` for the blind hop; byte-identical streams |
| TC-E2E-final | synthetic graph blind hop is the goal (no next) | Integration | Emits `ACT_FORWARD` (dead-reckon), not `ACT_STOP_AT_LINE` |
| TC-TOGGLE-PARITY | parse `routes.h` constant vs `field_data` bool | Unit | Equal |
| TC-FRESHNESS | `build_routes` then `git diff --exit-code` | Build | No diff; toy goldens unchanged |
| TC-EDGEINFO | `FieldMap::getEdge(...).stop_at_line_override` on an authored edge | Unit (host) | Returns the authored `StopAtLineMode`; `noEdge()` returns `DEFAULT` |
| TC-COMPILE | `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` | Build | Compiles + links clean — the only check that catches an `EdgeInfo` layout/ODR miss (host ctest only warns) |

**Test data:** the checked-in generator produces the CSV via a **base + targeted**
construction (NOT a brute 1152-row cross-product): (a) a base matrix with `pipelineBlind`
in `{SYNC,GYRO,RAW}` × `edgeOverride(3)` × `autoRuleActive(2)` = 864 rows, and (b) a
targeted supplemental block that varies `pipelineBlind` over all four values (adding
`STOP_AT_LINE`) ONLY across cells where it can change an output — i.e. `edgeMode==STOP_AT_LINE`
on the non-seek dead-reckon-degrade leg (covering `base==STOP_AT_LINE → motionMode=SYNC_TSHAPE`).
The generator encodes precedence + `motionMode` as an independent ordered rule list (not
re-derived from the resolver), sets `soft_fallback` true only when the auto rung fires AND
`board_ok`, and **self-asserts the exact emitted row count** (documented in a comment as
`base + supplemental`). A new synthetic fixture graph (2–3 nodes: a blind edge into a node
that starts a projection edge) lives under `tools/path_planner/tests/` and is consumed by
a new test module (NOT appended to `test_runtime_sim.py`, to avoid a parallel-edit conflict).
**Run command:** `uv run pytest tools/path_planner/tests/`; host `ctest`; `arduino-cli
compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Add `StopAtLineMode` enum | — | low | `tools/path_planner/pipeline_config.py` | Add `class StopAtLineMode(IntEnum){DEFAULT,ON,OFF}` + export in `__all__`. No `PipelineConfig` field (toggle is field_data, not pipeline). Satisfies data-model enum. |
| T2 | Config soft-fallback knob | — | low | `WRO2026/Config.h` | Add `constexpr bool STOP_AT_LINE_FAULT_FALL_BACK = false;` near the other `STOP_AT_LINE_*` knobs (~L554-617) with Doxygen explaining default=hard-fault, flip true at comp. Satisfies soft-fallback AC. |
| T3 | Field-wide toggle storage (offline) | T1 | low | `tools/path_planner/field_data.json`, `tools/path_planner/field_data.py` | Add top-level `"blind_projection_auto_seek": true` to JSON; in field_data.py expose `BLIND_PROJECTION_AUTO_SEEK = _data.get("blind_projection_auto_seek", True)`. (The per-edge `stop_at_line` key on EDGES dicts is a plain convention consumed by T4/T5 — no separate shape task.) |
| T4 | Validator: toggle + per-edge field | T3 | med | `tools/path_planner/validator.py` | Validate `blind_projection_auto_seek` is a bool (optional, default true). In `_validate_edges_field`, accept optional `stop_at_line` key on an EDGES entry: a `StopAtLineMode` member or `"on"/"off"` string (mirror the `direction` string tolerance, since this is a directional override); reject other types/values. |
| T5 | Graph builder: extract per-edge override | T1 | med | `tools/path_planner/graph_builder.py` | Add `stop_at_line` to the **directional** attr extractor (`_extract_directional_attrs`, NOT the bidirectional one — like `direction`, bake onto the forward `u→v` edge only, never mirror to `v→u`), setting `stop_at_line_override` on the graph edge (coerce string→`StopAtLineMode`). |
| T6 | Header writer: emit enum, field, toggle constant | T1, T5 | high | `tools/path_planner/header_writer.py` | Emit `enum class StopAtLineMode`; append `StopAtLineMode stop_at_line_override;` to the `EdgeInfo` struct definition (~L441-449) AND to BOTH per-cell builders `_edge_cell` and `_none_edge_cell` (~L273-302) — the new field is resolved like `direction_override` (a directional slot read from the graph edge, DEFAULT when absent), NOT via the `primitive_mode` slot map (`_PRIMITIVE_MODE_SLOT_BY_EDGE_TYPE`, which is irrelevant here). Emit a standalone `constexpr bool BLIND_PROJECTION_AUTO_SEEK = <from field_data>;` inside the `namespace Routes` block (`emit_routes` already has `field_data` in scope, ~L734 — thread it to the routes.h emit function). Keep `_emit_pipeline_config` unchanged. NOTE: there is NO `EdgeInfo` `offsetof`/`static_assert` layout guard in the codebase (it was removed earlier) — the field-addition checklist here is: struct def + both cell builders + `FieldMap::noEdge` (T7). |
| T7 | FieldMap noEdge positional init | T6 | low | `WRO2026/FieldMap.h` | Add the trailing `Routes::StopAtLineMode::DEFAULT` to the `noEdge()` positional `EdgeInfo` initializer (~L43-51) so the appended field is initialized; confirm `getEdge`/`getEdgeType` need no signature change. |
| T8 | Regenerate routes | T3,T4,T5,T6 | low | `WRO2026/routes.h`, `WRO2026/routes.cpp` (generated) | Run `uv run python tools/path_planner/build_routes.py`. Verify `EdgeInfo` field, `StopAtLineMode` enum, and `BLIND_PROJECTION_AUTO_SEEK` appear; toy `EDGES`/`NEXT_HOP` otherwise unchanged. Commit generated output with the source changes (CI freshness gate). |
| T9 | C++ resolver: struct + new logic | T8 | high | `WRO2026/StopAtLinePolicy.h` | Add `StopAtLineQuery` input struct; add `softFallback` to `StopAtLineResolution`; reimplement `resolveStopAtLine(const StopAtLineQuery&)` per the resolution algorithm (precedence + `auto_triggered`/`soft_fallback`). Update the header docstring + parity note. |
| T10 | Python resolver + sim dispatch | T1 | high | `tools/path_planner/sim/runtime.py` | Mirror T9: `resolve_stop_at_line` new signature (`edge_override`, `auto_rule_active` in; `soft_fallback` out) — keep step-for-step identical to T9. In `_walk_hops` BLIND branch, read the edge's `stop_at_line_override`, compute `next_next` type → `next_hop_is_projection`, `auto_rule_active = field_data.BLIND_PROJECTION_AUTO_SEEK && next_hop_is_projection`, pass into the resolver; emission stream unchanged for non-auto rows. |
| T11 | Offline compiler blind branch | T10 | med | `tools/path_planner/compiler.py` | In `compile_path` FRONT+REAR BLIND branches (~L328, L366), compute `next_hop_is_projection` from the next hop's edge type `graph.edges[path[i+1], path[i+2]]["type"] == EDGE_PROJECTION` — **guard `i+2 < len(path)`** (false when the blind hop is the last/second-last hop, avoiding IndexError). Read the edge `stop_at_line_override`; `auto_rule_active = field_data.BLIND_PROJECTION_AUTO_SEEK && next_hop_is_projection` (read the toggle from the now-used `field_data` param, currently `# noqa: ARG001`). Pass both into `resolve_stop_at_line`. Keep byte-identical to `_walk_hops`. |
| T12 | PathTracker dispatch + soft-fallback | T8, T9, T7, T2 | high | `WRO2026/PathTracker.cpp` | At the BLIND dispatch (~L1193): compute `nextHopIsProjection` via `FieldMap::getNextHop(_nextHop,_destinationNode)` then `getEdge(_nextHop,nextNext).type == PROJECTION` (mirror pass-through peek L546-552); build `StopAtLineQuery{ _activeStopAtLine, edge.stop_at_line_override, _resolveBlindMode(edge), PIPELINE_CONFIG.blind, _resolvedDirection, LINE_BOARD_COUNT, Routes::BLIND_PROJECTION_AUTO_SEEK && nextHopIsProjection }`. Latch `_activeSoftFallback = sal.softFallback`. When dispatching a soft seek, add `.setMaxDistance(edge.distance_cm)` (caps the over-search — does NOT alone land the axle on the node). In the shared Drive-fault guard at top of `update()` (~L463), BEFORE `DRIVE_FAULT`: if `_state==STOP_AT_LINE && _activeSoftFallback && Config::STOP_AT_LINE_FAULT_FALL_BACK && getFault().kind==LINE_NOT_FOUND`, then (a) call `Drive.clearFault()` explicitly (else next move no-ops) + ensure stopped, (b) issue a corrective `Drive.driveDist` for the residual axle-to-node offset (no overshoot rolls on a fault + sensor leads axle by `LINE_SENSOR_AXLE_OFFSET_CM`; exact value bench-verified — firmware sub-task behind a bench gate), (c) assert LineFollower disabled (replicate `_enterFault` teardown the bypass skips), (d) `Localization.onDeadReckonedAdvance(_nextHop)`, `_state=PLAN_NEXT_HOP`. Reset `_activeSoftFallback` on dequeue/fault/goal-complete (alongside `_resetPivotLatch`). |
| T13 | Truth-table generator + CSV | T9, T10 | med | `tools/path_planner/tests/fixtures/gen_stop_at_line_truth_table.py`, `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv` | New generator encoding precedence + `motionMode` as an ordered rule list INDEPENDENT of the resolver. Columns: `callFlag,edgeOverride,edgeMode,pipelineBlind,direction,lineBoardCount,autoRuleActive,expect_seek,expect_motionMode,expect_softFallback`. Emit **base + targeted**: base = `pipelineBlind∈{SYNC,GYRO,RAW}` × all other dims (864 rows); targeted supplement = `pipelineBlind=STOP_AT_LINE` ONLY where it changes an output (`edgeMode==STOP_AT_LINE`, non-seek degrade leg). Set `soft_fallback` true only when the auto rung fires AND `board_ok`. The generator **self-asserts the exact emitted row count** (comment documents `base + supplemental`). Assert it reproduces the prior 144 as a sub-slice. Regenerate the CSV. |
| T14 | Update TC13 (Python) | T13, T10 | low | `tools/path_planner/tests/test_runtime_sim.py` | Migrate the reader from POSITIONAL to **named (DictReader by column)** binding; pass `edge_override`/`auto_rule_active`; assert `soft_fallback`; set `_EXPECTED_ROW_COUNT` to the generator's emitted count (not a hardcoded guess); extend the input maps (`_STOP_AT_LINE_MODE_MAP`, etc.). |
| T15 | Update C++ resolver tests | T13, T9 | med | `WRO2026/test/test_path_tracker_stop_at_line.cpp` | Convert ALL `resolveStopAtLine` call sites (~16, not 21 — earlier line-grep over-counted) to the `StopAtLineQuery` struct; add cases for `edgeOverride ON/OFF`, `autoRuleActive` true/false, board-degrade, edge-OFF-beats-legacy-tag, and `softFallback` (auto vs explicit/legacy). Re-sync the file's stale `MIRROR PathTracker.cpp:NNN` back-pointer comments (currently cite old line numbers) to the post-T12 lines. |
| T16 | E2E synthetic fixture + parity test | T10, T11 | med | `tools/path_planner/tests/test_auto_stop_at_line_projection.py`, `tools/path_planner/tests/fixtures/` (new fixture) | Build a small graph: blind edge into a node that begins a projection edge. Assert `PathTrackerSim.go_to` and `compile_path` both emit `ACT_STOP_AT_LINE` for the blind hop (auto on) and `ACT_FORWARD` when the blind hop is the goal / toggle off. Byte-identical streams. (TC-E2E / TC-E2E-final / TC-TOGGLE-OFF.) |
| T17 | Toggle source-parity test | T8 | low | `tools/path_planner/tests/test_header_writer.py` (or a new test) | Parse `Routes::BLIND_PROJECTION_AUTO_SEEK` from generated `routes.h` and assert it equals `field_data.BLIND_PROJECTION_AUTO_SEEK`. (TC-TOGGLE-PARITY; Opus GATE 2.) |
| T18 | ADR | T9, T12 | low | `docs/decisions/037-blind-projection-auto-stop-at-line.md`, `DECISIONS.md` | New ADR-037: the auto-rule, decoupled per-edge `stop_at_line`, field-sourced toggle in routes.h (and why NOT Config.h), soft-fallback default-off. Add to the DECISIONS.md index + status table. Renumber if a parallel PR already claimed 037. |
| T19 | Comment on issue #52 | — | low | (GitHub) | Add a comment to issue #52 describing the deferred GUI work this issue ENABLES but does NOT do: an editor checkbox for `blind_projection_auto_seek` and a per-edge `stop_at_line` control, which will require extending FieldModel's two serialization surfaces + a JSON representation for EDGES (ADR-022 territory). Make clear the firmware/sim field exists now so #52 is UI wiring only. |
| T20 | Full regression | T12,T14,T15,T16,T17,T8 | med | (build) | Run `uv run pytest tools/path_planner/tests/`, host `ctest`, and `arduino-cli compile`. Fix any link/ODR/parity failures. Confirms all ACs + CI gates. |

## Notes for Implementer

- **Byte-parity is sacred.** T9 (C++) and T10 (Python) resolvers must be step-for-step
  identical; the generator (T13) is a *third, independent* transcription. If the three
  disagree, TC13/TC13-CPP catch it. Do not generate the CSV by calling the resolver.
- **EdgeInfo-field-addition checklist (T6/T7):** appending a field touches the struct
  definition, BOTH per-cell emitter builders (`_edge_cell`, `_none_edge_cell`), and the
  `FieldMap::noEdge()` positional init. There is NO `offsetof`/`static_assert` layout guard
  to update (it was removed earlier — verified). The host ctest may not catch a layout
  miss (only warns); rely on `arduino-cli` (T20 / TC-COMPILE) to catch ODR/link issues. A
  miss silently corrupts every edge read.
- **Soft-fallback axle reconciliation (T12) — BENCH GATE:** the cap
  `setMaxDistance(edge.distance_cm)` only removes the `STOP_AT_LINE_MAX_MARGIN_CM`
  over-search; it does NOT put the axle on the node. The overshoot roll
  (`LINE_SENSOR_AXLE_OFFSET_CM`) fires only in the OVERSHOOT state on detection — every
  fault path skips it, and the sensor leads the axle, so on a no-line fault the axle stops
  ~`LINE_SENSOR_AXLE_OFFSET_CM` SHORT of the node. The recovery MUST issue an explicit
  corrective `driveDist` (or read Drive's traveled distance) to land the axle on the node
  before `onDeadReckonedAdvance`; the exact correction is bench-verified on hardware, not
  asserted. This is a firmware sub-task behind a bench gate (the path is default-OFF).
- **Recovery bypasses `_enterFault` (T12):** the soft recovery does not go through
  `_enterFault`, so it MUST call `Drive.clearFault()` explicitly (the only fault reset —
  otherwise the next move silently no-ops) and replicate any teardown `_enterFault` would
  do. LineFollower is already disabled on the blind-seek dispatch path, but assert it
  rather than assume it.
- **Decoupling is partial:** an explicit `BlindMode::STOP_AT_LINE` motion tag still
  force-seeks (legacy, kept for back-compat). Full motion/seek orthogonality is not a
  goal this issue; the dedicated `stop_at_line=OFF` rung sits above the legacy tag so OFF
  always wins.
- **`_activeSoftFallback` lifetime:** latch on seek dispatch, reset on dequeue / fault /
  goal-complete (alongside the existing `_resetPivotLatch` reset points) so it can't bleed
  across goals.
- **Toggle single-source:** T6 emits `BLIND_PROJECTION_AUTO_SEEK` from the same
  `field_data` value the Python sim (T10/T11) reads. The CI `build_routes` + `git diff`
  gate plus T17 guard against drift.
- **Decoupling reminder:** an explicit `BlindMode` motion tag (GYRO/RAW/SYNC) does NOT
  suppress the auto-rule. Suppress with per-edge `stop_at_line=OFF` or call-site OFF only.
  `BlindMode::STOP_AT_LINE` remains a force-on motion tag for backward compat.
- **No toy golden churn:** do not add a BLIND→PROJECTION pair to the toy field; exercise
  the new path only via the T16 synthetic fixture, so the committed toy goldens stay put.
- **ADR-022 boundary:** the per-edge override is authored in `field_data.py` EDGES
  (Python) this issue; EDGES-in-JSON (needed for the GUI) is deferred to #52.
- **Rollback:** the feature is null-safe by construction — toggle off restores exact
  prior behavior, and `STOP_AT_LINE_FAULT_FALL_BACK=false` keeps today's hard-fault. If a
  field regresses, set `blind_projection_auto_seek=false` and regenerate.
