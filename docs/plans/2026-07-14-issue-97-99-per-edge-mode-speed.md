# Per-Edge Mode × Speed Orthogonalization (issues #97 + #99) Plan

**Goal:** Split the overloaded per-edge `primitive_mode` override into two orthogonal axes — **Speed** (uniform, every edge type) and **Mode** (type-switched) — so a BLIND hop can carry its own speed (the #97 gap) and the editor exposes both controls per edge (the #99 goal), while collapsing two legacy warts (STOP_AT_LINE-inside-mode, and the type-switched `primitive_mode` key) in the one PR where the JSON break is free.

**Approach:** Extend the existing `EDGES`-override overlay pipeline (field_data.json → validator → graph_builder → header_writer → `EdgeInfo` → firmware). Replace the `line_mode_override` slot with a unified `speed_override` (`SpeedTier`) consumed by **both** edge types (blind speed stops being hardcoded to `SPEED_NORMAL`). Keep `blind_mode_override` as the BLIND Mode, minus `STOP_AT_LINE` (folded onto the existing `stop_at_line` axis by emitter/fixture lowering — runtime byte-identical). Reserve a LINE Mode **surface only** (`mode: MIDDLE` accepted; `LEFT`/`RIGHT` validator-rejected until the LineFollower lateral-offset follow lands) with **no** new `EdgeInfo` field. Net: `EdgeInfo` stays 7 fields (a rename, not a growth). Hard-remove `primitive_mode` with a targeted migration error.

## Scope

- **In scope:**
  - Rename/repurpose `EdgeInfo.line_mode_override: LineMode` → `speed_override: SpeedTier`; make **both** LINE and BLIND hops resolve speed from it (blind stops using hardcoded `Config::SPEED_NORMAL`).
  - New `SpeedTier {DEFAULT, SLOW, NORMAL, FAST}` enum (replaces `LineMode`); resolves to the existing `Config::SPEED_SLOW/NORMAL/FAST` (60/70/90) through one mapping next to `presetToPwm`.
  - JSON `EDGES` schema: remove `primitive_mode`; add uniform `speed` key + type-switched `mode` key (both bidirectional). `direction` / `stop_at_line` unchanged (forward-only).
  - BLIND `mode` vocabulary = `{SYNC_TSHAPE, GYRO_TSHAPE, RAW_FLAT}` — **`STOP_AT_LINE` removed** from authoring; a seek is authored solely via `stop_at_line: on`. Emitter + fixtures lower the old `STOP_AT_LINE` intent to `blind_mode=SYNC_TSHAPE + stop_at_line=ON` (byte-identical `EdgeInfo` bits). ADR-048's `STOP_AT_LINE + stop_at_line='off'` contradiction rule dissolves (inexpressible).
  - LINE `mode` **surface only**: Python `LineFollowMode {DEFAULT, MIDDLE, LEFT, RIGHT}` for validator vocabulary; validator accepts `MIDDLE` (== default, emits nothing), **hard-rejects `LEFT`/`RIGHT`** with a marked "remove when offset-follow lands (issue #NNN)" guard. No C++ enum, no `EdgeInfo` field, no firmware consumption this PR.
  - `PIPELINE_CONFIG`: rename `line`→`speed` (default `NORMAL`, now applies to both edge types); keep `blind`/`turn`/`direction`. JSON key `pipeline_line_mode`→`pipeline_speed`.
  - Firmware: resolve per-edge speed for LINE (unchanged mechanism, renamed) and BLIND (new); replace the two `SPEED_NORMAL` hardcodes; extend blind-chain eligibility so a chain joins hops only when resolved **(mode, speed)** match.
  - Editor: two controls per edge — **Speed** (uniform) + **Mode** (type-switched: LINE→`MIDDLE`+inherit; BLIND→`SYNC_TSHAPE`/`GYRO_TSHAPE`/`RAW_FLAT`+inherit). Drop the ADR-048 save-mirror.
  - Migrate every fixture/golden/test off `primitive_mode` (LINE speed→`speed`; BLIND mode→`mode`; BLIND `STOP_AT_LINE`→`stop_at_line: on`).
  - Sim (`sim/runtime.py`) + CTPS parity for per-edge blind speed.
  - ADR-064 (supersedes ADR-047's `primitive_mode`-encoding clause + ADR-048's contradiction rule). Close #97 as absorbed; file the lateral-follow follow-up issue.
- **Out of scope:**
  - LineFollower lateral-offset following (LEFT/RIGHT behavior) + its `line_follow_mode_override` `EdgeInfo` field — deferred to the lateral-follow PR (tracked issue). This PR only reserves the JSON/editor surface.
  - Numeric per-edge PWM authoring (`speed_pwm`) — deferred; tiers keep field data chassis-agnostic (two chassis: TT vs Lofaix).
  - Separate blind-vs-line PWM constants — the single `SpeedTier→SpeedPreset` mapping is the future split seam; not pre-split now.
  - Firmware-internal removal of the now-unauthorable `BlindMode::STOP_AT_LINE` seek-ladder layer (ADR-046) — dead but harmless; a later cleanup.
  - Per-edge `turn` / new drive modes.

## Decisions

- **Merge #97 + #99 into one feature; close #97 as absorbed** — both hinge on the same per-edge data-shape decision; splitting the shape twice would rework the same field.
- **Speed = one shared axis** (`SpeedTier`, one `speed` key, one `speed_override` field, both edge types) — chosen over separate line/blind speed fields (which would leave one field always `DEFAULT`, the same dead-weight shape being removed). Fable-endorsed.
- **Blind speed reuses the line presets** (`SPEED_SLOW/NORMAL/FAST` = 60/70/90 via `presetToPwm`) — per user ("same as ConfigDrive/ConfigLine"); `NORMAL` stays byte-identical to today's hardcoded 70. One `SpeedTier→SpeedPreset` mapping, not a second PWM table.
- **LINE Mode = surface-only** — user chose the middle ground: land the `mode` key + editor bar now (LINE accepts `MIDDLE` only), but no C++ field (EdgeInfo stays 7 fields). Avoids Fable's "inert 8th field pays the full fan-out" objection while still shipping #99's two-axis surface. LEFT/RIGHT + the field land in the lateral-follow PR.
- **STOP_AT_LINE folded onto `stop_at_line`** — user reconsidered Fable's veto and took the fold: the JSON break is free now (only fixtures author EDGES), a second breaking migration later is not. Runtime byte-identical via emitter/fixture lowering; ADR-048's contradiction rule dissolves.
- **Hard-remove `primitive_mode`, no alias** — only fixtures author EDGES today and the routes-freshness CI gate makes stale generated code unmergeable; an alias would be dead code guarding data that does not exist. Refinement (Fable a): the validator special-cases the retired key with a targeted "split into `mode`+`speed`, see ADR-064" message rather than a generic unknown-key error.
- **Adopt the non-conflicting Fable wins:** blind-chain "same resolved (mode, speed)" eligibility rule (correctness); thread the `line`→`speed` rename to **both** resolution surfaces (firmware + `sim/runtime.py`); CTPS's 3-place knob contract for the now-variable blind speed. The `constexpr makeEdge()` construction factory is included as optional hardening (T13) since every positional `EdgeInfo` init is touched by the rename anyway.
- **ADR number = 064** (highest committed is 063; verify no unmerged-branch collision at author time).
- **Codex approach consult unavailable** (CLI/account rejected all models) — proceeded on the Fable consult alone.

## Acceptance Criteria

- [ ] AC1 — A BLIND `EDGES` entry with `speed: SLOW` cruises at `SPEED_SLOW`(60); `FAST`→`SPEED_FAST`(90); absent/`NORMAL`→70 (byte-identical to today). Verified in firmware resolution + sim parity.
- [ ] AC2 — A LINE `EDGES` entry with `speed: FAST` sets the LineFollower `FAST` preset (unchanged mechanism, via `speed` not `primitive_mode`).
- [ ] AC3 — `mode: SYNC_TSHAPE|GYRO_TSHAPE|RAW_FLAT` on a BLIND edge selects the drive mode as before; `mode: STOP_AT_LINE` is **rejected** by the validator with a message pointing to `stop_at_line: on` + ADR-064.
- [ ] AC4 — `mode: MIDDLE` on a LINE edge validates and emits nothing (no `EdgeInfo` field); `mode: LEFT|RIGHT` on a LINE edge is **rejected** with a "not yet implemented — issue #NNN" message.
- [ ] AC5 — `primitive_mode` anywhere in `EDGES` is **rejected** with a targeted migration message naming `mode`+`speed` and ADR-064 (not a generic unknown-key error).
- [ ] AC6 — Generated `EdgeInfo` has exactly 7 fields with `speed_override: SpeedTier` in the former `line_mode_override` slot; the `offsetof` layout static_assert passes; `FieldMap::noEdge()` + `routes_fixture` inits compile with no missing-field-initializer warning under `--warnings all`.
- [ ] AC7 — A blind run of two adjacent chainable hops with **different** resolved speed tiers does **not** merge into one velocity profile — the chain ends at the tier boundary (stop-then-relaunch). Same-tier hops still chain.
- [ ] AC8 — `PIPELINE_CONFIG.speed` (default `NORMAL`) is honored as the mission-wide default for both LINE and BLIND edges whose `speed` is absent, at **both** the firmware and the sim resolution surfaces (parity test green).
- [ ] AC9 — `uv run python tools/path_planner/build_routes.py` regenerates `routes.h/.cpp`; the routes-freshness CI check passes (regenerate-then-`git diff --exit-code`).
- [ ] AC10 — Full `pytest`, full host `ctest` (all targets), `arduino-cli compile`, and the parity job are all green. The editor's two bars round-trip through `to_json_dict`/`to_module_proxy` and `validate()`.
- [ ] AC11 — ADR-064 committed; DECISIONS.md index + status table updated; ADR-047/048 cross-links + statuses updated; #97 closed as absorbed; lateral-follow issue filed and its number wired into the AC4 rejection comment.

## Data Model

### Enums (`tools/path_planner/pipeline_config.py` → emitted into generated `routes.h`)

```python
class SpeedTier(IntEnum):          # REPLACES LineMode; consumed by BOTH edge types
    DEFAULT = 0   # inherit PIPELINE_CONFIG.speed at C++ dispatch
    SLOW    = 1   # -> Config::SPEED_SLOW   (60)
    NORMAL  = 2   # -> Config::SPEED_NORMAL (70)
    FAST    = 3   # -> Config::SPEED_FAST   (90)

class LineFollowMode(IntEnum):     # PYTHON-ONLY this PR (validator vocabulary). NOT emitted to C++.
    DEFAULT = 0   # inherit (== MIDDLE today)
    MIDDLE  = 1   # centered follow (only authorable value this PR)
    LEFT    = 2   # RESERVED — validator-rejected until offset-follow lands (issue #NNN)
    RIGHT   = 3   # RESERVED — validator-rejected until offset-follow lands (issue #NNN)

class BlindMode(IntEnum):          # UNCHANGED enum; STOP_AT_LINE no longer JSON-authorable
    DEFAULT      = 0
    SYNC_TSHAPE  = 1
    GYRO_TSHAPE  = 2
    RAW_FLAT     = 3
    STOP_AT_LINE = 4   # kept for firmware seek-ladder (ADR-046); NOT emitted from any EDGES cell
```

`DirectionMode` / `StopAtLineMode` unchanged.

### `EdgeInfo` (generated `routes.h` + `WRO2026/test/fixtures/routes_fixture.h`) — stays 7 fields

```cpp
struct EdgeInfo {
    EdgeType       type;
    float          distance_cm;
    float          heading_deg;
    SpeedTier      speed_override;        // WAS: LineMode line_mode_override
    BlindMode      blind_mode_override;   // unchanged
    DirectionMode  direction_override;    // unchanged
    StopAtLineMode stop_at_line_override; // unchanged
};
```

Field count, order, and byte offsets are unchanged (a plain-enum→plain-enum rename), so the `offsetof` static_assert and every positional init keep the same arity — only the type/default token changes (`LineMode::DEFAULT` → `SpeedTier::DEFAULT`).

### `PIPELINE_CONFIG` (`pipeline_config.py`, emitted)

```
line -> speed   (SpeedTier, default NORMAL, applies to LINE and BLIND)
blind           (BlindMode, default SYNC_TSHAPE)   unchanged
turn, direction                                    unchanged
```

### JSON `EDGES` entry schema (field_data.json)

| Key | Values | Symmetry | Notes |
|-----|--------|----------|-------|
| `type`,`u`,`v` | required | — | unchanged |
| `distance_cm`,`heading_deg` | optional | — | derived from coords when absent; unchanged |
| `speed` | `SLOW`\|`NORMAL`\|`FAST` | **bidirectional** | both edge types; absent = inherit |
| `mode` | LINE: `MIDDLE` · BLIND: `SYNC_TSHAPE`\|`GYRO_TSHAPE`\|`RAW_FLAT` | **bidirectional** | type-switched; absent = inherit; LINE `LEFT`/`RIGHT` rejected; BLIND `STOP_AT_LINE` rejected |
| `direction` | `front`\|`rear` | forward-only | unchanged |
| `stop_at_line` | `on`\|`off` | forward-only | **now the sole seek axis** |
| ~~`primitive_mode`~~ | — | — | **removed**; targeted migration error |

## Contracts & Interfaces

Single source of truth for every cross-task seam. Conform byte-for-byte.

### Names (must match across Python, C++, editor, fixtures)
- Enum: `SpeedTier` (values `DEFAULT/SLOW/NORMAL/FAST`), `BlindMode` (unchanged), Python-only `LineFollowMode`.
- JSON keys: `speed`, `mode` (both bidirectional); `direction`, `stop_at_line` (forward-only); top-level `pipeline_speed` (was `pipeline_line_mode`).
- `EdgeInfo` field: `speed_override` (was `line_mode_override`).
- `PIPELINE_CONFIG` field: `speed` (was `line`).
- Retired token: `primitive_mode` (validator special-case), authorable `BlindMode::STOP_AT_LINE` (validator/editor reject), `LineFollowMode::{LEFT,RIGHT}` (validator reject).

### Signatures
- `SpeedTier -> Config::SpeedPreset` mapping: one function `speedTierToPreset(SpeedTier)` in `WRO2026/src/util/WROMath.h`, next to `presetToPwm` — owner: T6; consumers: T7. Do **not** add a second PWM table.
- Firmware resolver `_resolveSpeed(const Routes::EdgeInfo&) -> Routes::SpeedTier` (was `_resolveLineMode`), falls back to `PIPELINE_CONFIG.speed` — owner: T7.
- Header emit `_resolve_overrides(edge_attrs) -> (speed_override, blind_override, direction_override, stop_at_line_override)` (drops the line slot; routes `speed`→speed_override for any type, `mode`→blind_override on BLIND only, ignores LINE `mode`) — owner: T4; consumers: T-regen, T12.
- graph_builder `_BIDIRECTIONAL_OPTIONAL_KEYS = ("speed", "mode")` — owner: T3.
- Blind-chain eligibility predicate gains a resolved-`SpeedTier` compare alongside the existing mode compare in `_chainTryStart` / the walk-ahead — owner: T7; mirror in T5 (sim) + T14 (CTPS run-grouper).

### File ownership
| File | Owner | Consumers |
|------|-------|-----------|
| `tools/path_planner/pipeline_config.py` | T1 | T2,T3,T4,T5,T9,T10 |
| `tools/path_planner/validator.py` | T2 | — |
| `tools/path_planner/graph_builder.py` | T3 | — |
| `tools/path_planner/header_writer.py` | T4 | T-regen |
| `tools/path_planner/compiler.py` + `sim/runtime.py` | T5 | — |
| `WRO2026/src/util/WROMath.h` | T6 | T7 |
| `WRO2026/src/mission/PathTracker.{h,cpp}` | T7 | — |
| `WRO2026/src/nav/FieldMap.h` (`noEdge` init) | T8 | — |
| `WRO2026/src/nav/routes.{h,cpp}` (generated) | T-regen | firmware compile |
| `tools/path_planner/editor/panels.py` | T9 | — |
| `tools/path_planner/editor/commands.py` | T10 | — |
| Python fixtures (`tests/fixtures/*.py`,`*.json`) | T11 | T12b |
| `WRO2026/test/fixtures/routes_fixture.{h,cpp}` | T12 | host tests |
| Python tests (`tests/test_*.py`) | T12b | — |
| Host tests (`WRO2026/test/*.cpp`, CMake) | T14host | — |
| CTPS (recorder/loader/renderer/profile) | T14 | — |
| ADR-064 + DECISIONS.md + issues | T15 | — |

## Error Handling (validator rejections — ADR-029 single gate; editor mirrors are fast feedback only)

- **`primitive_mode` present** → `FieldDataError` naming `EDGES[i]`: "`primitive_mode` was split into `mode` + `speed` (ADR-064)". Special-cased before the generic unknown-key path.
- **`mode: STOP_AT_LINE` on BLIND** → reject: "STOP_AT_LINE folded into the `stop_at_line` axis (ADR-064); author `stop_at_line: on`".
- **`mode: LEFT|RIGHT` on LINE** → reject: "LineFollower offset-follow not yet implemented (issue #NNN); remove this guard when it lands". Marked with a `# TODO(issue #NNN)` comment.
- **`mode` wrong-type tag** (e.g. `SYNC_TSHAPE` on LINE, `MIDDLE` on BLIND) → category error naming valid tags for the derived type.
- **`speed` = unknown / literal `DEFAULT`** → reject (DEFAULT == absence; omit the key).
- **`speed`/`mode` unknown tag** → precise diagnostic listing valid tags.
- Both `_validate_edges_field` **and** `_build_authored_graph` tolerate the new keys (geometry gates unchanged; verify neither key-whitelists in a way that drops the new keys).

## Testing Strategy

**Levels:** Unit (validator, header emit, editor commands), Integration (host FSM ctest, arduino compile, routes freshness), Parity (sim↔firmware).

| ID | Test Case | Type | Expected |
|----|-----------|------|----------|
| TC1 | BLIND edge `speed: SLOW` → firmware/sim cruise | Parity | Resolves to `SPEED_SLOW` (60); NORMAL/absent = 70 byte-identical |
| TC2 | LINE edge `speed: FAST` emit | Unit | `EdgeInfo.speed_override = SpeedTier::FAST` |
| TC3 | `mode: SYNC_TSHAPE` BLIND emit | Unit | `blind_mode_override = SYNC_TSHAPE` |
| TC4 | `mode: STOP_AT_LINE` BLIND | Unit | Rejected; message names `stop_at_line: on` |
| TC5 | Old `primitive_mode: STOP_AT_LINE` fixture lowered to `stop_at_line: on` | Unit | Emitted `EdgeInfo` bits byte-identical to pre-change golden (`blind=SYNC`, `stop=ON`) |
| TC6 | `mode: MIDDLE` LINE | Unit | Validates; emits nothing (no field) |
| TC7 | `mode: LEFT` LINE | Unit | Rejected; "not yet implemented (issue #NNN)" |
| TC8 | `primitive_mode` anywhere | Unit | Rejected; targeted ADR-064 migration message |
| TC9 | `EdgeInfo` layout | Unit (C++) | `offsetof` static_assert holds; clean `--warnings all` build |
| TC10 | Blind chain, tier boundary | Integration (host) | Chain ends at the boundary; same-tier hops chain |
| TC11 | `PIPELINE_CONFIG.speed` default both surfaces | Parity | Absent-speed edge uses NORMAL in firmware AND sim |
| TC12 | Editor two-bar round-trip | Unit | `speed`+`mode` set → `to_json_dict` entry → `validate()` passes → `to_module_proxy` carries EDGES |
| TC13 | `speed`+`mode` bidirectional | Unit | Both forwarded to reverse directed edge; `direction`/`stop_at_line` forward-only |
| TC14 | Routes freshness | Integration | `build_routes.py` then `git diff --exit-code` clean |
| TC15 | Full pytest + full ctest + arduino compile | Integration | All green (fixture migration complete) |

**Test data:** migrate `field_data_stop_at_line.json` + every `SimpleNamespace`/toy fixture off `primitive_mode`; regenerate `test_header_writer.py` goldens via its `regenerate_goldens()` helper. **Run:** `uv run --extra dev pytest tools/path_planner/tests/`; `ctest --test-dir build/test`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Enums + PIPELINE_CONFIG | — | high | `pipeline_config.py` | Add `SpeedTier` (replace `LineMode`) + `LineFollowMode`; rename `PipelineConfig.line`→`speed` (default NORMAL) + `from_field_data`/`_SPECS`/`_CATEGORY_FIELDS` (`pipeline_line_mode`→`pipeline_speed`, keep DEFAULT-ban). Update `__all__`. Satisfies contract for AC1/AC8. |
| T2 | Validator mode/speed | T1 | high | `validator.py` | Rewrite `_validate_edges_field`: drop `primitive_mode` (targeted ADR-064 error), add `speed` (both types) + type-switched `mode` (LINE→`LineFollowMode`: accept MIDDLE, reject LEFT/RIGHT w/ `# TODO(issue #NNN)`; BLIND→`BlindMode`: reject STOP_AT_LINE). Delete ADR-048 contradiction check. Verify `_build_authored_graph` tolerates new keys. AC3/AC4/AC5. |
| T3 | graph_builder forwarding | T1 | med | `graph_builder.py` | `_BIDIRECTIONAL_OPTIONAL_KEYS=("speed","mode")`; ensure `speed`/`mode` copy to both directions; `direction`/`stop_at_line` stay directional. AC13. |
| T4 | header_writer emit | T1 | high | `header_writer.py` | Rework `_resolve_overrides`: `speed`→`speed_override` (any type), `mode`→`blind_mode_override` on BLIND, ignore LINE `mode`. Emit `SpeedTier` enum (drop `LineMode`); keep `BlindMode` w/ comment. `_edge_cell`/`_none_edge_cell` emit `speed_override`. Emit `PIPELINE_CONFIG.speed`. AC6/AC2/AC3. |
| T5 | compiler + sim resolve | T1 | high | `compiler.py`, `sim/runtime.py` | `resolve_line_mode`→`resolve_speed`; resolve blind speed per-edge in the sim dispatch (was constant); mirror blind-chain tier compare (T7); keep `resolve_blind_mode` STOP_AT_LINE dead path. Un-hardcode any `SPEED_NORMAL` sim mirror. AC1/AC8/AC11-parity. |
| T6 | speedTierToPreset helper | T1 | low | `WROMath.h` | Add `Config::SpeedPreset speedTierToPreset(SpeedTier)` next to `presetToPwm`. Single mapping; no second PWM table. |
| T7 | Firmware consumption + chain | T6, T4 | high | `PathTracker.{h,cpp}` | Rename `_resolveLineMode`→`_resolveSpeed`, `_lineModeToPreset`→use `speedTierToPreset`; LINE path via `speed_override` (2610/2675/2705). BLIND: replace hardcoded `SPEED_NORMAL` (1639, 2806-2807) with resolved-tier PWM in SYNC/GYRO/RAW branches. Extend `_chainTryStart` + walk-ahead eligibility to compare resolved `SpeedTier`. AC1/AC7. |
| T8 | FieldMap noEdge | T4 | low | `FieldMap.h` | Update `noEdge()` positional init: `LineMode::DEFAULT`→`SpeedTier::DEFAULT`. |
| T9 | Editor two bars | T1 | med | `editor/panels.py` | Replace `_PRIMITIVE_VALUES_BY_TYPE` combobox with two rows: Speed (`SLOW/NORMAL/FAST`+inherit, uniform) + Mode (LINE→`MIDDLE`+inherit; BLIND→`SYNC_TSHAPE/GYRO_TSHAPE/RAW_FLAT`+inherit). `_handle_save_override` emits `speed`+`mode`. Drop ADR-048 save-mirror. |
| T10 | Editor command keys | T1 | med | `editor/commands.py` | `_OVERRIDE_VALUE_KEYS=("speed","mode","direction","stop_at_line")`; `SetEdgeOverride` fields `speed`/`mode` (drop `primitive_mode`); `_build_entry` handles new keys, preserves geometry (R5). |
| T11 | Python fixtures migration | T2, T4 | med | `tests/fixtures/*.json`,`*.py` | Migrate `field_data_stop_at_line.json` + every `SimpleNamespace`/toy fixture off `primitive_mode` (LINE speed→`speed`; BLIND mode→`mode`; `STOP_AT_LINE`→`stop_at_line: on`). Add the `pipeline_speed` attr to every emitter-read fixture. |
| T12 | Host EdgeInfo fixture | T4 | high | `test/fixtures/routes_fixture.{h,cpp}` | Rename `line_mode_override`→`speed_override` (`SpeedTier`); update `offsetof` static_assert token; migrate toy EDGES cells + any `STOP_AT_LINE` cell to (SYNC+stop_at_line). |
| T12b | Python tests | T2,T3,T4,T5,T9,T10,T11 | med | `tests/test_field_data.py`,`test_header_writer.py`,`test_compiler.py`,`test_editor_*.py` | Add TC1-TC8,TC12,TC13; regenerate header goldens (`regenerate_goldens()`); migrate every `primitive_mode`/`STOP_AT_LINE` authoring in tests. Run FULL pytest (fixtures fan out across modules). |
| T13 | (Optional) makeEdge factory | T8, T12 | low | `routes.h`(template), `FieldMap.h`, `routes_fixture.cpp` | Add `constexpr Routes::makeEdge(type,dist,heading, speed=DEFAULT, blind=DEFAULT, dir=DEFAULT, stop=DEFAULT)`; convert hand-written positional inits to it so the lateral-follow PR's field addition is source-compatible. Drop if it complicates the diff. |
| T14 | CTPS blind-speed parity | T5, T7 | med | `ctps/` recorder/loader/renderer/`profile.py` | Thread resolved blind speed into recorded dispatch (3-place: recorder + loader whitelist + renderer); mirror the chain tier-boundary in the run-grouper; verify the config-derived motion profile uses per-edge cruise. |
| T14host | Host tests + compile | T7, T12 | high | `WRO2026/test/*.cpp`, CMake | Adapt ~20 FSM targets to `speed_override`; add TC9,TC10; resolve real-routes nodes by name; run FULL ctest + `arduino-cli compile`. |
| T-regen | Regenerate routes | T4, T5, T7 | med | `WRO2026/src/nav/routes.{h,cpp}` | `uv run python tools/path_planner/build_routes.py`; commit `routes.h/.cpp` with the sources. Freshness gate (TC14). |
| T15 | ADR-064 + issues | T2, T4, T7 | low | `docs/decisions/064-*.md`, `DECISIONS.md` | Write ADR-064 (supersede ADR-047 encoding clause + ADR-048 contradiction rule; update both statuses/cross-links + index + status table). Close #97 as absorbed; file the lateral-follow issue; wire its number into the T2 `LEFT/RIGHT` reject comment + AC4. |

## Notes for Implementer

- **Fan-out is a RENAME, not a growth.** `EdgeInfo` keeps 7 fields — every positional init changes only the `LineMode::DEFAULT`→`SpeedTier::DEFAULT` token. Still grep every hand-written `EdgeInfo` init (`FieldMap::noEdge`, `routes_fixture.cpp`, any toy/shadow routes TU) — a missed one compiles silently (value-inits to 0) and only warns under `--warnings all` ([[gotcha-edgeinfo-field-addition]]).
- **Two enforcement/resolution surfaces, always:** validator geometry lives in `_validate_edges_field` AND `_build_authored_graph`; `PIPELINE_CONFIG` + blind-speed resolution live in firmware AND `sim/runtime.py` (the parity test catches drift). The `line`→`speed` rename and blind-chain tier rule must land in both.
- **Fixture fan-out needs the FULL suites.** A new emitter-read attr (`pipeline_speed`) breaks every `SimpleNamespace` fixture across several modules; a contract change breaks BOTH Python AND host C++ fixtures. Only the full `pytest` + full `ctest` surface them all — expect 2-3 whack-a-mole rounds. Host FSM tests link real/fixture routes by NAME, not index.
- **STOP_AT_LINE fold is contract-only, runtime byte-identical.** The firmware already handles `blind=SYNC + stop_at_line=ON`; the fold just stops emitting `blind=STOP_AT_LINE`. Pin the equivalence with a golden (TC5). The C++ `BlindMode::STOP_AT_LINE` member stays (ADR-046 seek ladder references it) but is emitted from no EDGES cell — comment it.
- **LINE `mode` is genuinely inert this PR** (only `MIDDLE`, == default). It exists to establish the JSON key + editor bar so the lateral-follow PR is purely additive. `MIDDLE` maps to omit/inherit; the validator still *accepts* an explicit `MIDDLE` to document intent, but the emitter produces no field for it. Do not add a C++ enum or `EdgeInfo` field for it.
- **Enum numeric values are cosmetic** — everything serializes by NAME (`SpeedTier::NORMAL`), so choosing `SpeedTier {DEFAULT=0,SLOW=1,NORMAL=2,FAST=3}` (vs old `LineMode` numbering) is safe; keep `SpeedTier` the same underlying int size as the old `LineMode` so `EdgeInfo` layout/offsets are unchanged.
- **Config constant tripwire:** `SPEED_SLOW/NORMAL/FAST` are unchanged, so no drive-constant retune fan-out — but if the blind path ever wants distinct tiers, `speedTierToPreset` is the single split seam, not new EDGES fields.
- **ADR number:** verify no unmerged branch already claimed 064 before committing ([[gotcha-adr-number-collision]]); renumber the unmerged one if so.
- **Rollback:** the change is additive-then-swap; revert = restore `primitive_mode` handling + `line_mode_override` + the two `SPEED_NORMAL` hardcodes + regenerate. No data migration to undo (real field has no EDGES).
