# Per-Edge Mode × Speed Orthogonalization (issues #97 + #99) Plan

**Goal:** Split the overloaded per-edge `primitive_mode` override into two orthogonal axes — **Speed** (uniform, every edge type) and **Mode** (type-switched) — so a BLIND hop can carry its own speed (the #97 gap) and the editor exposes both controls per edge (the #99 goal), while collapsing two legacy warts (STOP_AT_LINE-inside-mode, and the type-switched `primitive_mode` key) in the one PR where the JSON break is free.

**Approach:** Extend the existing `EDGES`-override overlay pipeline (field_data.json → validator → graph_builder → header_writer → `EdgeInfo` → firmware). Replace the `line_mode_override` slot with a unified `speed_override` (`SpeedTier`) consumed by **both** edge types (blind speed stops being hardcoded to `SPEED_NORMAL`, including on the seek dispatch). Keep `blind_mode_override` as the BLIND Mode, minus `STOP_AT_LINE` (folded onto the existing `stop_at_line` axis by lowering to explicit `mode: SYNC_TSHAPE + stop_at_line: on`). Reserve a LINE Mode **surface only** (`mode: MIDDLE` accepted; `LEFT`/`RIGHT` validator-rejected until the LineFollower lateral-offset follow lands) with **no** new `EdgeInfo` field. Net: `EdgeInfo` stays 7 fields (a rename, not a growth). Hard-remove `primitive_mode` with a targeted migration error.

## Scope

- **In scope:**
  - Rename/repurpose `EdgeInfo.line_mode_override: LineMode` → `speed_override: SpeedTier`; make **both** LINE and BLIND hops resolve speed from it. No raw `Config::SPEED_NORMAL` literal remains in any blind dispatch: edge cruise, **the seek dispatch** (`Drive.stopAtLine(...).setPower(...)`), and the pass-through entry seed all derive from a resolved `SpeedTier`.
  - New `SpeedTier {DEFAULT, SLOW, NORMAL, FAST}` enum (replaces `LineMode`); resolves to the existing `Config::SPEED_SLOW/NORMAL/FAST` (60/70/90) through one mapping next to `presetToPwm`.
  - JSON `EDGES` schema: remove `primitive_mode`; add uniform `speed` key + type-switched `mode` key (both bidirectional). `direction` / `stop_at_line` unchanged (forward-only).
  - BLIND `mode` vocabulary = `{SYNC_TSHAPE, GYRO_TSHAPE, RAW_FLAT}` — **`STOP_AT_LINE` removed** from both per-edge AND pipeline authoring; a seek is authored solely via `stop_at_line: on`. Legacy `STOP_AT_LINE` intent lowers to **explicit** `mode: SYNC_TSHAPE + stop_at_line: on`. ADR-048's `STOP_AT_LINE + stop_at_line='off'` contradiction rule dissolves (inexpressible).
  - LINE `mode` **surface only**: Python `LineFollowMode {DEFAULT, MIDDLE, LEFT, RIGHT}` for validator vocabulary; validator accepts `MIDDLE` (== default, emits nothing), **hard-rejects `LEFT`/`RIGHT`** with a marked "remove when offset-follow lands (issue #NNN)" guard. No C++ enum, no `EdgeInfo` field, no firmware consumption this PR.
  - `PIPELINE_CONFIG`: rename `line`→`speed` (default `NORMAL`, now applies to both edge types); keep `blind`/`turn`/`direction`. JSON key `pipeline_line_mode`→`pipeline_speed`.
  - Firmware: resolve per-edge speed for LINE (unchanged mechanism, renamed) and BLIND (new, incl. seek); replace all `SPEED_NORMAL` hardcodes; extend blind-chain eligibility so a chain joins hops only when the resolved **speed tier** matches (mode mixing is unchanged from today).
  - Editor: two controls per edge — **Speed** (uniform) + **Mode** (type-switched: LINE→`MIDDLE`+inherit; BLIND→`SYNC_TSHAPE`/`GYRO_TSHAPE`/`RAW_FLAT`+inherit). Drop the ADR-048 save-mirror. PIPELINE DEFAULTS panel: `pipeline_speed` rename + blind vocab minus `STOP_AT_LINE`.
  - Migrate every fixture/golden/test off `primitive_mode` (LINE speed→`speed`; BLIND mode→`mode`; BLIND `STOP_AT_LINE`→`mode: SYNC_TSHAPE, stop_at_line: on`).
  - Sim (`sim/runtime.py`, `sim/parity.py`) + CTPS parity for per-edge blind speed.
  - ADR-064 (supersedes ADR-047's `primitive_mode`-encoding clause + ADR-048's contradiction rule). Close #97 as absorbed; file the lateral-follow follow-up issue.
- **Out of scope:**
  - LineFollower lateral-offset following (LEFT/RIGHT behavior) + its `line_follow_mode_override` `EdgeInfo` field — deferred to the lateral-follow PR (tracked issue). This PR only reserves the JSON/editor surface. **The `constexpr makeEdge()` construction factory (to soften that PR's field-addition fan-out) is recommended for that PR, not this rename-only one.**
  - Numeric per-edge PWM authoring (`speed_pwm`) — deferred; tiers keep field data chassis-agnostic (two chassis: TT vs Lofaix).
  - Separate blind-vs-line PWM constants — the single `SpeedTier→SpeedPreset` mapping is the future split seam; not pre-split now.
  - Firmware-internal removal of the now-unauthorable `BlindMode::STOP_AT_LINE` seek-ladder layer (ADR-038/039, `StopAtLinePolicy.h`) — dead but harmless; a later cleanup.
  - Per-edge `turn` / new drive modes.

## Decisions

- **Merge #97 + #99 into one feature; close #97 as absorbed** — both hinge on the same per-edge data-shape decision; splitting the shape twice would rework the same field.
- **Speed = one shared axis** (`SpeedTier`, one `speed` key, one `speed_override` field, both edge types) — chosen over separate line/blind speed fields (which would leave one field always `DEFAULT`, the same dead-weight shape being removed). Fable-endorsed.
- **Blind speed reuses the line presets** (`SPEED_SLOW/NORMAL/FAST` = 60/70/90 via `presetToPwm`) — per user ("same as ConfigDrive/ConfigLine"); `NORMAL` stays byte-identical to today's hardcoded 70. One `SpeedTier→SpeedPreset` mapping, not a second PWM table.
- **LINE Mode = surface-only** — user chose the middle ground: land the `mode` key + editor bar now (LINE accepts `MIDDLE` only), but no C++ field (EdgeInfo stays 7 fields). Avoids the "inert 8th field pays the full fan-out" objection while still shipping #99's two-axis surface. LEFT/RIGHT + the field land in the lateral-follow PR.
- **STOP_AT_LINE folded onto `stop_at_line`, lowered to explicit `SYNC_TSHAPE + on`** — user reconsidered and took the fold: the JSON break is free now (only fixtures author EDGES), a second breaking migration later is not. The fold is **NOT unconditionally byte-identical**: the pre-fold degraded-seek path (REAR seek on a 1-board chassis, or call-site/override OFF) dead-reckoned under `PIPELINE_CONFIG.blind` (`StopAtLinePolicy.h:182-183`); the explicit `SYNC_TSHAPE` lowering pins that degrade to `SYNC`, matching the non-degraded seek mode a bare `STOP_AT_LINE` edge already runs (`StopAtLinePolicy.h:173-174`; `blindModeToDriveMode` defaults to SYNC, `PathTracker.cpp:147-156`). The one behavior that changes: a degraded legacy-`STOP_AT_LINE` hop under a **non-SYNC** pipeline blind default previously ran that non-SYNC mode and now runs SYNC. This is an **accepted, intentional pin** (seek and its own fallback kept on one motion mode), documented in ADR-064 — not an incidental equivalence. Today's default is SYNC, so current behavior is unchanged. Pinned by a `resolveStopAtLine` resolution-equivalence test (Opus judge, SYNTHESIS ruling).
- **Blind-chain rule restricts on SPEED only, not mode** — the chain merges hops into one velocity profile, so a differing speed tier must break it; but mode mixing (SYNC↔GYRO) already chains today and stays chainable (a mode-match restriction would add stops against the fewer-stops preference). RAW_FLAT / seek already break the chain as before.
- **Hard-remove `primitive_mode`, no alias** — only fixtures author EDGES today and the routes-freshness CI gate makes stale generated code unmergeable. The validator special-cases the retired key with a targeted "split into `mode`+`speed`, see ADR-064" message rather than a generic unknown-key error.
- **ADR number = 064** (highest committed is 063; verify no unmerged-branch collision at author time).
- **Codex approach + spec consult unavailable** (CLI/account rejected all models) — proceeded on the Fable consult + Fable critic + Opus judge alone.

## Acceptance Criteria

- [ ] AC1 — A BLIND `EDGES` entry with `speed: SLOW` cruises at `SPEED_SLOW`(60); `FAST`→`SPEED_FAST`(90); absent/`NORMAL`→70 (byte-identical to today). Holds on the plain cruise **and** the seek dispatch (`speed: SLOW + stop_at_line: on` seeks at 60, not 70). Verified in firmware + sim parity.
- [ ] AC2 — A LINE `EDGES` entry with `speed: FAST` sets the LineFollower `FAST` preset (unchanged mechanism, via `speed` not `primitive_mode`).
- [ ] AC3 — `mode: SYNC_TSHAPE|GYRO_TSHAPE|RAW_FLAT` on a BLIND edge selects the drive mode as before; `mode: STOP_AT_LINE` (per-edge **or** as `pipeline_blind_mode`) is **rejected** with a message pointing to `stop_at_line: on` + ADR-064.
- [ ] AC4 — `mode: MIDDLE` on a LINE edge validates and emits nothing (no `EdgeInfo` field); `mode: LEFT|RIGHT` on a LINE edge is **rejected** with a "not yet implemented — issue #NNN" message.
- [ ] AC5 — `primitive_mode` anywhere in `EDGES` is **rejected** with a targeted migration message naming `mode`+`speed` and ADR-064 (not a generic unknown-key error).
- [ ] AC6 — Generated `EdgeInfo` has exactly 7 fields with `speed_override: SpeedTier` in the former `line_mode_override` slot (byte layout unchanged: same field count/order, `SpeedTier` same underlying int size as `LineMode`). Every hand-written positional `EdgeInfo` init is updated; the `--warnings all` build is clean (no `missing-field-initializers`). **No `offsetof` static_assert is added** — none exists today and a pytest freezes its absence.
- [ ] AC7 — A blind run of two adjacent chainable hops with **different** resolved speed tiers does **not** merge into one velocity profile — the chain ends at the tier boundary. Same-tier hops still chain, including across a SYNC↔GYRO mode boundary (mode mixing unchanged).
- [ ] AC8 — `PIPELINE_CONFIG.speed` (default `NORMAL`) is honored as the mission-wide default for both LINE and BLIND edges whose `speed` is absent, at **both** the firmware and the sim resolution surfaces (parity test green).
- [ ] AC9 — `uv run python tools/path_planner/build_routes.py` regenerates `routes.h/.cpp`; the routes-freshness CI check passes (regenerate-then-`git diff --exit-code`).
- [ ] AC10 — Full `pytest`, full host `ctest` (all targets), `arduino-cli compile`, and the parity job are all green. The editor's two bars round-trip through `to_json_dict`/`to_module_proxy` and `validate()` (incl. the `pipeline_speed` rename in `editor/model.py`).
- [ ] AC11 — ADR-064 committed; DECISIONS.md index + status table updated; ADR-047/048 cross-links + statuses updated; #97 closed as absorbed; lateral-follow issue filed and its number wired into the AC4 rejection comment.
- [ ] AC12 — The pass-through entry seed into a blind hop is clamped so entry speed ≤ the hop's resolved cruise (a `FAST` LINE edge into a `SLOW` BLIND edge does not enter above 60), at both firmware and sim.

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
    STOP_AT_LINE = 4   # kept for firmware seek-ladder (ADR-038/039, StopAtLinePolicy.h); emitted from NO EDGES cell
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

Field count, order, and byte offsets are unchanged (a plain-enum→plain-enum rename of the same underlying size). **There is no `offsetof` static_assert** — it was removed with the pivot fields and a pytest (`test_header_writer.py`) freezes its absence; do not add one. The layout guard is the positional-init arity + a clean `--warnings all` build (`missing-field-initializers` catches a dropped field). Grep every hand-written init and update the token only (`LineMode::DEFAULT` → `SpeedTier::DEFAULT`).

### `PIPELINE_CONFIG` (`pipeline_config.py`, emitted)

```
line -> speed   (SpeedTier, default NORMAL, applies to LINE and BLIND)
blind           (BlindMode, default SYNC_TSHAPE; STOP_AT_LINE rejected as a default)
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
- `EdgeDispatchQuery.lineMode` (`WRO2026/src/nav/EdgeDispatchPolicy.h:85`) → renamed `speed`, typed `Routes::SpeedTier`.
- Retired token: `primitive_mode` (validator special-case), authorable `BlindMode::STOP_AT_LINE` per-edge **and** pipeline (validator/editor reject), `LineFollowMode::{LEFT,RIGHT}` (validator reject).

### Signatures
- `SpeedTier -> Config::SpeedPreset` mapping: one function `speedTierToPreset(SpeedTier)` in `WRO2026/src/util/WROMath.h`, next to `presetToPwm` — owner: T6; consumers: T7. No second PWM table.
- Firmware resolver `_resolveSpeed(const Routes::EdgeInfo&) -> Routes::SpeedTier` (was `_resolveLineMode`), falls back to `PIPELINE_CONFIG.speed` — owner: T7.
- STOP_AT_LINE lowering (canonical, one form everywhere): legacy `primitive_mode: STOP_AT_LINE` → `{"mode": "SYNC_TSHAPE", "stop_at_line": "on"}` → emitted cell `(blind_mode_override=SYNC_TSHAPE, stop_at_line_override=ON)`. Cited by T4/T11/T12/TC5.
- Header emit `_resolve_overrides(edge_attrs) -> (speed_override, blind_override, direction_override, stop_at_line_override)` (drops the line slot; routes `speed`→speed_override for any type, `mode`→blind_override on BLIND only, ignores LINE `mode`) — owner: T4; consumers: T-regen, T12.
- graph_builder `_BIDIRECTIONAL_OPTIONAL_KEYS = ("speed", "mode")` — owner: T3.
- Blind-chain eligibility predicate gains a resolved-`SpeedTier` compare (SPEED only; do NOT add a mode compare) in `_chainTryStart` / the walk-ahead — owner: T7; mirror in T5 (sim) + T14 (CTPS run-grouper).
- Editor save seam (pin so T9/T10 don't diverge): `panels._handle_save_override` → `app` callback `on_set_edge_override(edge_type, u, v, speed, mode, direction, stop_at_line)` → `commands.SetEdgeOverride(edge_type, u, v, speed=None, mode=None, direction=None, stop_at_line=None)`. `editor/app.py` owned by T9; `commands.py` by T10; both conform to this signature.

### File ownership
| File | Owner | Consumers |
|------|-------|-----------|
| `tools/path_planner/pipeline_config.py`, `field_data.py` (loader) | T1 | T2,T3,T4,T5,T9,T10,T-model |
| `tools/path_planner/validator.py` | T2 | — |
| `tools/path_planner/graph_builder.py` | T3 | — |
| `tools/path_planner/header_writer.py` | T4 | T-regen |
| `tools/path_planner/compiler.py`, `sim/runtime.py`, `sim/parity.py` | T5 | — |
| `WRO2026/src/util/WROMath.h` | T6 | T7 |
| `WRO2026/src/mission/PathTracker.{h,cpp}`, `WRO2026/src/nav/EdgeDispatchPolicy.h` | T7 | — |
| `WRO2026/src/nav/FieldMap.h` (`noEdge` init) | T8 | — |
| `WRO2026/src/nav/routes.{h,cpp}` (generated) | T-regen | firmware compile |
| `tools/path_planner/editor/panels.py`, `editor/app.py` | T9 | — |
| `tools/path_planner/editor/commands.py` | T10 | — |
| `tools/path_planner/editor/model.py`, `loader.py`, `migrate.py` | T-model | — |
| Python fixtures (`tests/fixtures/*.py`,`*.json`), `WRO2026/test/fixtures/field_fixture.py` | T11 | T12b |
| `WRO2026/test/fixtures/routes_fixture.{h,cpp}`, `routes_reset_passthrough_forced.{h,cpp}` | T12 | host tests |
| Python tests (`tests/test_*.py`) | T12b | — |
| Host tests (`WRO2026/test/*.cpp`, CMake) + `arduino-cli compile` | T14host | — |
| CTPS (recorder/loader/renderer/`profile.py`/`track_builder.py`) | T14 | — |
| ADR-064 + DECISIONS.md + issues | T15 | — |

## Error Handling (validator rejections — ADR-029 single gate; editor mirrors are fast feedback only)

- **`primitive_mode` present** → `FieldDataError` naming `EDGES[i]`: "`primitive_mode` was split into `mode` + `speed` (ADR-064)". Special-cased before the generic unknown-key path.
- **`mode: STOP_AT_LINE` on BLIND, or `pipeline_blind_mode: STOP_AT_LINE`** → reject: "STOP_AT_LINE folded into the `stop_at_line` axis (ADR-064); author `stop_at_line: on`".
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
| TC4 | `mode: STOP_AT_LINE` (per-edge) + `pipeline_blind_mode: STOP_AT_LINE` | Unit | Both rejected; message names `stop_at_line: on` + ADR-064 |
| TC5 | Lowered `mode: SYNC_TSHAPE + stop_at_line: on` vs legacy STOP_AT_LINE | Unit | `resolveStopAtLine` resolution-equivalence: `{seek, motionMode}` equal across all four degrade triggers (seek, REAR-1board, call-OFF, override-OFF), with motionMode pinned SYNC. **Not** a bit-compare against a pre-fold golden. |
| TC6 | `mode: MIDDLE` LINE | Unit | Validates; emits nothing (no field) |
| TC7 | `mode: LEFT` LINE | Unit | Rejected; "not yet implemented (issue #NNN)" message content |
| TC8 | `primitive_mode` anywhere | Unit | Rejected; targeted ADR-064 migration message content |
| TC9 | `EdgeInfo` layout | Unit (C++) + pytest | Clean `--warnings all` build; every hand-written init updated; the pytest freezing `offsetof`-absence stays green (no assert added) |
| TC10 | Blind chain: (a) tier boundary breaks; (b) same-tier SYNC↔GYRO still chains; (c) `speed` on a seek edge | Integration (host) + Parity | (a) chain ends at boundary; (b) chains; (c) seek cruises at the resolved tier |
| TC11 | `PIPELINE_CONFIG.speed` default both surfaces | Parity | Absent-speed edge uses NORMAL in firmware AND sim |
| TC12 | Editor two-bar + pipeline round-trip | Unit | `speed`+`mode` set → `to_json_dict` → `validate()` passes → `to_module_proxy` carries EDGES + `pipeline_speed` |
| TC13 | `speed`+`mode` bidirectional; wrong-type-tag; `speed: DEFAULT`/unknown | Unit | Both forwarded to reverse edge; `direction`/`stop_at_line` forward-only; wrong-type + DEFAULT + unknown each rejected with the specified message |
| TC14 | Pass-through entry seed clamp | Parity | FAST-line → SLOW-blind: entry ≤ 60 in firmware AND sim |
| TC15 | Routes freshness + full pytest + full ctest + arduino compile | Integration | `build_routes.py` then `git diff --exit-code` clean; all suites green |

**Test data:** migrate `field_data_stop_at_line.json` + every `SimpleNamespace`/toy fixture + `field_fixture.py` off `primitive_mode`; regenerate `test_header_writer.py` goldens via its `regenerate_goldens()` helper. **Run:** `uv run --extra dev pytest tools/path_planner/tests/`; `ctest --test-dir build/test`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T15a | File lateral-follow issue | — | low | (GitHub) | File "LineFollower lateral offset-follow: implement LEFT/RIGHT LineFollowMode + wire line_follow_mode_override EdgeInfo field". Capture its number for T2's reject comment. Runs first so `#NNN` is real. |
| T1 | Enums + PIPELINE_CONFIG + loader | — | high | `pipeline_config.py`, `field_data.py` | Add `SpeedTier` (replace `LineMode`) + `LineFollowMode`; rename `PipelineConfig.line`→`speed` (default NORMAL) + `from_field_data`/`_SPECS`/`_CATEGORY_FIELDS` + `__all__`; rename `field_data.py:93` loader `pipeline_line_mode`→`pipeline_speed`. |
| T2 | Validator mode/speed | T1, T15a | high | `validator.py` | Rewrite `_validate_edges_field`: drop `primitive_mode` (targeted ADR-064 error), add `speed` (both types) + type-switched `mode` (LINE→accept MIDDLE, reject LEFT/RIGHT w/ `# TODO(issue #<T15a>)`; BLIND→reject STOP_AT_LINE). Reject `pipeline_blind_mode: STOP_AT_LINE` (validator.py:1748-1798) + rename the `PIPELINE_LINE_MODE` spec row → `pipeline_speed` (validator.py:1798). Delete ADR-048 contradiction check. Verify `_build_authored_graph` tolerates new keys. |
| T3 | graph_builder forwarding | T1 | med | `graph_builder.py` | `_BIDIRECTIONAL_OPTIONAL_KEYS=("speed","mode")`; `speed`/`mode` copy to both directions; `direction`/`stop_at_line` stay directional. |
| T4 | header_writer emit | T1 | high | `header_writer.py` | Rework `_resolve_overrides`: `speed`→`speed_override` (any type), `mode`→`blind_mode_override` on BLIND, ignore LINE `mode`. Emit `SpeedTier` enum (drop `LineMode`); keep `BlindMode` w/ "no EDGES cell emits STOP_AT_LINE" comment. `_edge_cell`/`_none_edge_cell` emit `speed_override`. Emit `PIPELINE_CONFIG.speed`. Do NOT add an offsetof assert (a pytest freezes its absence). |
| T5 | compiler + sim resolve | T1 | high | `compiler.py`, `sim/runtime.py`, `sim/parity.py` | `resolve_line_mode`→`resolve_speed`; resolve blind speed per-edge in the sim dispatch incl. seek (was constant `SPEED_NORMAL` at runtime.py; parity.py:102-112 imports + RAW_FLAT expectations 963-976); mirror the SPEED-only chain compare + pass-through seed clamp (runtime.py:3409-3412); keep `resolve_blind_mode` STOP_AT_LINE dead path. |
| T6 | speedTierToPreset helper | T1 | low | `WROMath.h` | Add `Config::SpeedPreset speedTierToPreset(SpeedTier)` next to `presetToPwm`. Single mapping; no second PWM table. |
| T7 | Firmware consumption + chain | T6, T-regen | high | `PathTracker.{h,cpp}`, `EdgeDispatchPolicy.h` | Rename `_resolveLineMode`→`_resolveSpeed`, use `speedTierToPreset`; rename `EdgeDispatchQuery.lineMode`→`speed:SpeedTier`. LINE path via `speed_override` (2610/2675/2705). BLIND: replace `SPEED_NORMAL` — edge cruise (2806-7) + **seek dispatch** `.setPower(resolvedPwm)` (2768-73; `Drive.stopAtLine` default `_power=SPEED_NORMAL`, Drive.h:2979) use per-edge tier; RETURN/recovery moves (1621-25, 1639) use `PIPELINE_CONFIG.speed`. Clamp pass-through entry seed (2794-8) to resolved cruise. Extend `_chainTryStart`+walk-ahead eligibility with a resolved-`SpeedTier` compare (SPEED only; keep existing mode handling). |
| T8 | FieldMap noEdge | T4 | low | `FieldMap.h` | Update `noEdge()` positional init token: `LineMode::DEFAULT`→`SpeedTier::DEFAULT`. |
| T9 | Editor two bars + pipeline panel | T1 | med | `editor/panels.py`, `editor/app.py` | Replace `_PRIMITIVE_VALUES_BY_TYPE` with two rows: Speed (uniform) + Mode (LINE→`MIDDLE`+inherit; BLIND→`SYNC/GYRO/RAW`+inherit, no STOP_AT_LINE). `_handle_save_override` emits `speed`+`mode` via the pinned callback. Drop ADR-048 save-mirror. PIPELINE DEFAULTS: `pipeline_speed` rename + blind vocab minus STOP_AT_LINE. Own `app.py` callback seam (app.py:1502-1524). |
| T10 | Editor command keys | T1 | med | `editor/commands.py` | `_OVERRIDE_VALUE_KEYS=("speed","mode","direction","stop_at_line")`; `SetEdgeOverride` fields `speed`/`mode` (drop `primitive_mode`) per the pinned signature; `_build_entry` handles new keys, preserves geometry (R5). |
| T-model | Editor model serialization | T1 | med | `editor/model.py`, `loader.py`, `migrate.py` | Rename `pipeline_line_mode`→`pipeline_speed` dataclass field + `to_json_dict`/`to_module_proxy` (both surfaces — the dual-serialization trap); update the `primitive_mode`/`pipeline_line_mode` key whitelists in `loader.py:32,129,154` + `migrate.py:259`. AC10 round-trip. |
| T11 | Fixtures migration | T2, T4 | med | `tests/fixtures/*.json`,`*.py`, `WRO2026/test/fixtures/field_fixture.py` | Migrate `field_data_stop_at_line.json` + every `SimpleNamespace`/toy fixture + `field_fixture.py:123,310-311` off `primitive_mode` (LINE speed→`speed`; BLIND mode→`mode`; `STOP_AT_LINE`→canonical `mode: SYNC_TSHAPE, stop_at_line: on`). Add `pipeline_speed` to every emitter-read fixture. |
| T12 | Host EdgeInfo fixture | T4 | high | `test/fixtures/routes_fixture.{h,cpp}`, `routes_reset_passthrough_forced.{h,cpp}` | Rename `line_mode_override`→`speed_override` (`SpeedTier`) in both shadow TUs (10 occurrences in the reset-passthrough TU); migrate toy EDGES cells + any `STOP_AT_LINE` cell to the canonical lowering. |
| T12b | Python tests | T2,T3,T4,T5,T9,T10,T-model,T11 | med | `tests/test_field_data.py`,`test_header_writer.py`,`test_compiler.py`,`test_editor_*.py` | Add TC1-TC8,TC12,TC13; regenerate header goldens (`regenerate_goldens()`); migrate every `primitive_mode`/`STOP_AT_LINE` authoring. Run FULL pytest (fixtures fan out across modules). |
| T14 | CTPS blind-speed parity | T5, T7 | med | `ctps/` recorder/loader/renderer/`profile.py`/`track_builder.py` | Thread resolved blind speed into recorded dispatch (3-place: recorder + loader whitelist + renderer); blind cruise PWM is picked at `track_builder.py:608-615` + `profile.py:1904/1954` — thread both; mirror the SPEED-only chain break in the run-grouper. |
| T-regen | Regenerate routes | T1,T2,T3,T4,T5 | med | `WRO2026/src/nav/routes.{h,cpp}` | `uv run python tools/path_planner/build_routes.py`; commit `routes.h/.cpp`. Freshness gate (TC15). Unblocks firmware compile. |
| T14host | Host tests + arduino compile | T7, T12, T-regen | high | `WRO2026/test/*.cpp`, CMake | Adapt ~20 FSM targets to `speed_override`; add TC9,TC10,TC14; resolve real-routes nodes by name. Run FULL ctest + `arduino-cli compile`. |
| T15 | ADR-064 + issues | T2, T4, T7, T15a | low | `docs/decisions/064-*.md`, `DECISIONS.md` | Write ADR-064 (supersede ADR-047 encoding clause + ADR-048 contradiction rule; document the STOP_AT_LINE fold's SYNC pin + the one changed degrade case; update both statuses/cross-links + index + status table). Close #97 as absorbed; patch the `#NNN` placeholder (T15a's number) into the T2 reject comment. |

## Notes for Implementer

- **Fan-out is a RENAME, not a growth.** `EdgeInfo` keeps 7 fields — every positional init changes only the `LineMode::DEFAULT`→`SpeedTier::DEFAULT` token. There is **no** `offsetof` static_assert (removed with the pivot fields; a pytest freezes its absence — do not add one). Still grep every hand-written `EdgeInfo` init (`FieldMap::noEdge`, `routes_fixture.cpp`, `routes_reset_passthrough_forced.cpp`, any toy/shadow routes TU) — a missed one value-inits to 0 and only warns under `--warnings all` ([[gotcha-edgeinfo-field-addition]]).
- **STOP_AT_LINE fold is NOT unconditionally byte-identical.** It pins the degraded-seek dead-reckon (REAR on a 1-board chassis, or call-site/override OFF) to `SYNC_TSHAPE`, matching the non-degraded seek mode a bare `STOP_AT_LINE` edge already runs (`StopAtLinePolicy.h:173-174`; `blindModeToDriveMode` defaults SYNC, `PathTracker.cpp:147-156`). It differs from pre-fold **only** for a field whose `PIPELINE_CONFIG.blind != SYNC_TSHAPE` (pre-fold degrade inherited that non-SYNC mode; `StopAtLinePolicy.h:182-183`). Today's default is SYNC, so behavior is unchanged today. Document it in ADR-064 as an intentional pin and name that one changed case; pin it with the TC5 resolution-equivalence test (`{seek, motionMode}`), not a bit-compare golden. Lower to **explicit** `mode: SYNC_TSHAPE + stop_at_line: on` everywhere (one form: T4/T11/T12/TC5) — never bare `stop_at_line: on` (that would leave `blind_mode_override=DEFAULT` and re-introduce the `pipelineBlind`-inherited fork). Also reject `pipeline_blind_mode: STOP_AT_LINE` so the `pipelineBlind==STOP_AT_LINE` rung (`StopAtLinePolicy.h:185-187`) is fully unauthorable.
- **Blind chain restricts on SPEED tier only.** The current chain (`_chainTryStart` 2346, dispatch match 2834-2841) rejects only RAW_FLAT + seeks; SYNC↔GYRO mixed runs chain today and must keep chaining. Add ONLY a resolved-tier compare — a mode-equality restriction would add stops against the fewer-stops preference ([[project-fewer-stops-better-friction-drift]]).
- **All blind PWM derives from a resolved tier.** Sites: edge cruise (2806-7) + seek dispatch (`Drive.stopAtLine(...).setPower(...)`, default `_power=SPEED_NORMAL` at Drive.h:2979, sites ~2768-73/1621-25) use the per-edge tier; RETURN/recovery moves (1639 `_dispatchReturnMoveAfterTurn`, and the return-side line-seek) have no `EdgeInfo`, so use `PIPELINE_CONFIG.speed`. No raw `SPEED_NORMAL` literal survives in a blind dispatch. The pass-through entry seed (2794-8) is clamped to the resolved cruise so entry ≤ cruise.
- **Two resolution surfaces, always:** validator geometry lives in `_validate_edges_field` AND `_build_authored_graph`; `PIPELINE_CONFIG` + blind-speed resolution + the seed clamp + the chain rule live in firmware AND `sim/runtime.py` (parity test catches drift). `sim/parity.py` imports the renamed resolver — it breaks on ImportError until updated. Editor model serializes through `to_json_dict` AND `to_module_proxy` — the `pipeline_speed` rename touches both ([[gotcha-fieldmodel-dual-serialization]]).
- **Fixture fan-out needs the FULL suites.** A new emitter-read attr (`pipeline_speed`) breaks every `SimpleNamespace` fixture across several modules; a contract change breaks BOTH Python AND host C++ fixtures. Only the full `pytest` + full `ctest` surface them all — expect 2-3 whack-a-mole rounds ([[gotcha-strict-emitter-field-fixtures]], [[gotcha-field-contract-change-breaks-cpp-and-py-fixtures]]). Host FSM tests link real/fixture routes by NAME, not index ([[gotcha-host-tests-link-production-routes]]).
- **LINE `mode` is genuinely inert this PR** (only `MIDDLE`, == default). It exists to establish the JSON key + editor bar so the lateral-follow PR is purely additive. `MIDDLE` maps to omit/inherit; the validator accepts an explicit `MIDDLE` to document intent, but the emitter produces no field. Do not add a C++ enum or `EdgeInfo` field.
- **Enum numeric values are cosmetic** — everything serializes by NAME (`SpeedTier::NORMAL`), so `SpeedTier {DEFAULT=0,SLOW=1,NORMAL=2,FAST=3}` is safe; keep `SpeedTier` the same underlying int size as the old `LineMode` so `EdgeInfo` layout/offsets are unchanged.
- **ADR number:** verify no unmerged branch already claimed 064 ([[gotcha-adr-number-collision]]); renumber the unmerged one if so.
- **`#NNN` ordering:** T15a files the lateral-follow issue first; T2 writes the reject message against that number (or a `#NNN` placeholder that T15 patches). Do not leave a dangling placeholder.
- **Rollback:** the change is additive-then-swap; revert = restore `primitive_mode` handling + `line_mode_override` + the `SPEED_NORMAL` sites + regenerate. No data migration to undo (real field has no EDGES).
