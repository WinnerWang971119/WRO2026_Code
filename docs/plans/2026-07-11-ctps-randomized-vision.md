# CTPS Randomized Vision Plan

**Goal:** Give CTPS a "randomized vision" feature that mimics the real WRO mosaic-read: a configurable 4×3 mosaic that is re-rolled every time you open CTPS, a robot-attached forward vision cone, a player-side `readMosaic()` simulation (true iff all 12 cells fall inside the cone), and the dormant dump-injection plumbing that will let the robot's *path* vary with the roll once a vision-consuming mission exists.

> **Note on user request 4(d)** ("if it cannot see, fire read mosaic but return false; if it can read return true"): this is satisfied **at the player HUD layer only** — the geometric read prediction. The C++ `Vision` mock's return value is intentionally **decoupled** from cone geometry (thin mock, ADR-049): the mock returns the injected result and the mission drives to a deliberate read pose, while the HUD independently reports whether the cone actually covers all 12 cells. A mock-says-ok / HUD-says-cone-misses divergence is a **useful "your camera mount or read pose is wrong" signal**, not a bug. The user chose "plumbing, path varies later" with full knowledge the current mission reads no vision; this is that choice made explicit.

**Approach:** Three layers on top of the existing CTPS replay pipeline. **(1) Authoring** — a new top-level `mosaic` block and a `robot.vision` block in `layout.json`, authored in the offline editor (a new Mosaic tab + vision-cone controls on the Robot tab). **(2) Roll + carry** — the Python orchestrator rolls the mosaic once (seeded), passes the colors to `ctps_dump`, and the dump **echoes** the roll (colors + seed + a geometry fingerprint) into its trace JSON so the mosaic is carried by the single trace artifact (no drift; consistent on every path including `--trace`). **(3) Render + read** — Python `resolve_cells()` (unit-tested) turns the geometry + rolled colors into 12 world-positioned cells embedded as `const MOSAIC`; a new `assets/vision.js` (shipped on every page, like the cargo assets) draws the cells + the robot-attached cone as registered layers and runs the per-cell cone-visibility predicate into a read HUD. The C++ side gains a thin `Vision` mock and a `read_mosaic` trace op — landed and CI-proven now via the dump self-test, dormant until a Phase-6 mission actually calls `Vision.readMosaic()`.

This is the scope the user chose: **CTPS visualization sim + dump-injection plumbing** — not a demo vision-mission. It follows the Fable approach consult (trace-carried roll over dual distribution; unified pinned-cells schema; header-parse color guard; per-cell predicate; land the mock now) and the Fable critic + Opus judge rulings (unconditional vision assets; Python-tested geometry; trace-carried geometry fingerprint). Codex consult/critic unavailable (CLI model-version error).

## Scope

- **In scope:**
  - `layout.json` schema: a top-level `mosaic` block (grid geometry + gaps + palette + per-cell pins + counts + mode + optional seed) and a `robot.vision` block (robot-local forward cone).
  - Editor authoring: a new **Mosaic** tab and **vision-cone controls on the Robot tab**; serialize both to `layout.json`.
  - Python roll + resolve: a seeded, single-source `roll_mosaic()` (colors) and a unit-tested `resolve_cells()` (geometry → 12 world-positioned cells); the orchestrator resolves the seed and passes the roll to `ctps_dump`.
  - Dump-injection plumbing: a **thin** `ctps_mocks/Vision.h` mock; `ctps_dump` argv (`--mosaic`, `--mosaic-seed`, `--mosaic-geom-hash`, `--mosaic-fail-at`); the dump **echoes** a top-level `mosaic` block; a `read_mosaic` trace op (recorder + loader whitelist + track event), CI-exercised via the dump self-test synthesis.
  - Player render: `assets/vision.js` (shipped unconditionally, shared with the editor) — draw + per-cell cone predicate + cone/mosaic layers + a read HUD in a visible panel; `MOSAIC_META` + `const MOSAIC` emitted from Python when a mosaic is present.
  - Tests: `test_ctps_mosaic.py`, `test_ctps_layout.py` additions, `test_ctps_trace_loader.py` additions, a color-enum parse-guard pytest, seeded render goldens (mosaic-present region asserts **and** the mosaic-free/TC9 invariants), host C++ ctest for the mock + dump argv + echo + `read_mosaic` op + the malformed-argv exit.
  - ADR-060 + doc updates (DECISIONS.md, ARCHITECTURE.md, CLAUDE.md/AGENTS.md module lists).
- **Out of scope (deliberately deferred):**
  - **A vision-consuming mission/task** (the piece that makes the recorded *path* actually vary) — Roadmap Phase-6 (P6.1/P6.2), ADR-025 mission logic. The plumbing is built and CI-proven but dormant until a mission calls `Vision.readMosaic()`.
  - **A geometry-gated dump** (the mock re-evaluating the cone and re-running the mission). Rejected: heavy, no convergence guarantee, duplicates the predicate, violates ADR-033. The thin mock is intentional (see the Goal note).
  - **A player-side re-roll button.** Re-roll = re-run `python -m tools.path_planner.ctps` (a browser re-roll would desync the overlay from the recorded run).
  - **Any firmware change.** The real `Vision` already returns `false` on a bad read.
  - **Mosaic sizes other than 12 cells.** Pinned to `rows*cols == 12` (== `Config::VISION_CELL_COUNT`).

## Decisions

- **Trace-carried roll (single carrier).** Python rolls once (seeded); the colors + seed + a geometry fingerprint go to `ctps_dump`; the dump echoes them into a top-level `mosaic` JSON block. The rendered mosaic is resolved from (trace colors × layout geometry). Rationale: the alternative (Python feeding both the player embed and the dump separately) is an unguarded drift pair and makes `--trace` incoherent. — *Fable consult VETO adopted.*
- **Unconditional vision assets; only DATA is conditional.** `vision.js`, the `#ctps-vision` panel markup, and the cone/mosaic trail-layer registration ship on **every** player page (mirroring ADR-056's cargo assets, which the in-browser `store.loadLayout` upload leg needs). Only the emitted DATA is conditional: `MOSAIC_META` / `const MOSAIC` / the trace `mosaic` roll are present only when a mosaic exists. A layout with `mosaic`/`robot.vision` uploaded into a mosaic-free-emitted page therefore renders fully. — *Opz judge (C2) CRITIC_WINS.*
- **Geometry authority is Python, tested.** `mosaic.resolve_cells(geom, color_names) → [{index, x_cm, y_cm, color}]` computes the 12 world cell centres (the load-bearing cell→`getColor` mapping) + colors; unit-tested with pinned vectors. The player draws these embedded resolved cells directly (like field objects, which already carry world positions). The editor computes cell positions in JS for live authoring preview only (a non-load-bearing mirror, eyeball-verified against the same pinned vectors). — *Fable critic W7.*
- **Trace-carried geometry fingerprint.** The dump echoes a `geom_hash` alongside colors+seed; `player_html` compares it to the current layout geometry's hash and badges a mismatch (mirrors the existing "trace recorded against a different field" guard). Closes the `--trace` colors-on-wrong-grid hole. — *Fable critic W6.*
- **Unified pinned-cells schema.** `mosaic.cells[12]` (each `null`=rolled or a pinned color) + `mosaic.counts` (quotas for the null cells) + a `mode` label. `mode:"directed"` requires all 12 pinned; `mode:"random"` allows some pins with `counts` summing to the number of nulls. One roll function, one validator; "mostly-directed with 2 random cells" comes free. — *Fable consult #2.*
- **Read succeeds iff all 12 cells are in the cone** (user Q2), but the predicate returns **per-cell booleans** so the HUD can say which cell is outside. `readMosaicWouldSucceed = all(bool[12])`. — *Fable consult #4.*
- **Vision cone lives in `robot.vision` (layout.json), not `ConfigVision.h`** — robot-local geometry like `grab_zone`; the firmware never consumes FOV geometry.
- **Thin C++ Vision mock; no cone geometry in C++** (ADR-049). See the Goal note on 4(d).
- **Color contract by header-parse, not a hand-synced triplicate.** Canonical names+letters+hex single-sourced in `mosaic.py`, emitted as `MOSAIC_META` (kills the JS copy), parsed letter→`Config::Color` in the C++ mock against the real enum (kills the C++ copy). One pytest regex-parses `enum class Color` out of `ConfigVision.h` and asserts the Python names/order (the `parse_track_cm_from_config` precedent). — *Fable consult #3.*
- **Uniform grid gaps.** `row_gap_cm` / `col_gap_cm` are single scalars for the whole mosaic (the natural reading of "the row gap and the column gap"; a normal uniform grid). Per-row/per-column arrays are a possible future extension; the scalar form is assumed here. The user may override. — *Fable critic W2.*
- **Opt-in DATA.** The orchestrator injects a mosaic (and emits `MOSAIC_META`/`const MOSAIC`) **only when the layout has a `mosaic` block**; no block ⇒ no `--mosaic` arg ⇒ the dump JSON and the emitted DATA are unchanged from today. (The vision ASSETS still ship — see the unconditional-assets decision.)
- **Seed determinism.** All randomness in one Python function taking an explicit `random.Random(seed)`. Default = fresh entropy per invocation; the used seed is echoed into the trace + shown in a player badge (reproduce with the orchestrator's `--seed`). Tests pin the seed.
- **Land the mock + `read_mosaic` op now, CI-proven via the dump self-test** — `ctps_dump` compiles all of `Tasks.cpp`, so a real Phase-6 vision task would break the dump build unless the mock already exists; the `read_mosaic` 3-place contract is exercised by synthetic emission so it lands verified. — *Fable consult #6.*
- **UI/UX gate not run generatively.** CTPS is internal tooling with a frozen design language (ADR-053 `tokens.css`); the new panels **must match the existing CTPS tokens/components**. A dedicated CSS-owner task (covering **both** `components.css` for the player and `editor.css` for the editor) + a localhost eyeball step are included (the `ctps-plan-css-owner-gap` gotcha). — *Fable critic W12.*

## Acceptance Criteria

- [ ] **AC1** — A `layout.json` with a valid `mosaic` block and `robot.vision` block loads/validates; a malformed one raises `LayoutError` loud with a precise key path. *(T2, T19)*
- [ ] **AC2** — `roll_mosaic(config, Random(seed))` is deterministic; random-mode output honours the counts multiset over the null cells and keeps every pinned cell; directed-mode output equals the pinned cells verbatim. *(T1, T18)*
- [ ] **AC3** — Validation rejects: `rows*cols != 12`; `cells` length ≠ 12; a pin/palette/counts color not in the palette; random-mode `sum(counts) != #null cells`; directed-mode with any null cell; `half_angle_deg` outside (0,90); `range_cm <= 0`; a negative or non-integer seed. *(T1/T2, T18/T19)*
- [ ] **AC4** — The Python color name list matches `Config::Color` (order + names, `UNKNOWN` stripped) via the header-parse guard; drift fails CI. *(T3)*
- [ ] **AC5** — `resolve_cells(geom, colors)` returns 12 world-positioned cells whose centres match pinned expected vectors for a rotated, unequal-gap config; index `i = row*cols+col`, row 0 at `+y`. *(T1, T18)*
- [ ] **AC6** — `ctps_dump --mosaic <letters> --mosaic-seed <n> --mosaic-geom-hash <h>` echoes a top-level `mosaic` block (`{cells:[names×12], seed, geom_hash, injected:true}`); without `--mosaic` the JSON is byte-identical to today (no `mosaic` key). A malformed `--mosaic` (bad letter / wrong length) exits non-zero with a stderr message and emits no partial block. *(T5, T6, T22)*
- [ ] **AC7** — The thin `Vision` mock: `getColor(cell)` returns the injected color; `readMosaic()` returns `true` (or `false` on the `--mosaic-fail-at` Nth call); each `readMosaic()` records a `read_mosaic` op. The `ctps_dump` build (compiling all of `Tasks.cpp`) succeeds with the mock shadowing the real `Vision.h`. *(T4, T6, T7, T22)*
- [ ] **AC8** — `trace_loader` whitelists/validates the top-level `mosaic` block and the `read_mosaic` op; an unknown/malformed one is skipped-with-warning, never fatal. *(T8, T20)*
- [ ] **AC9** — `visibleCells(pose, visionCfg, cells)` returns a 12-element boolean array; a cell is visible iff it is forward of the apex, within `half_angle_deg` of robot-forward, and within `range_cm` (or any distance when `range_cm` is null). `readMosaicWouldSucceed` = all true. *(T11, T18 mirror + T24 boundary eyeball)*
- [ ] **AC10** — With a mosaic + cone, the rendered player draws the 12 colored cells (from `const MOSAIC`), the robot-attached cone (moving with `poseAt(t)`), and a read HUD in the **visible** `#ctps-vision` panel updating per frame ("k/12 — cell j outside cone", green at 12/12); a no-roll trace draws the grid outline with a "no roll recorded" badge; a geometry-hash mismatch shows a badge. *(T10, T11, T12, T24)*
- [ ] **AC11** — The editor's Mosaic tab authors the grid geometry/gaps/palette/pins/counts/mode and the Robot tab authors the cone; Save serializes both into `layout.json`; reload round-trips them; an uploaded `mosaic`/`robot.vision` layout renders fully in a player page emitted without a mosaic (upload-leg coverage). *(T13–T17, T21)*
- [ ] **AC12** — A **mosaic-free** render emits no `MOSAIC_META`, no `const MOSAIC`, no trace-mosaic roll; the TC9 `PLAYER_META` literal is unchanged; the full existing player + editor suites stay green (the new **unconditional** `vision.js` / `#ctps-vision` / layer-registration additions do not red the frozen literal/substring tests; any test freezing the exact asset list or page markup is updated to include them). *(T0, T10, T21)*
- [ ] **AC13** — Full pytest and host C++ ctest suites pass on Windows and Linux/macOS (LF-normalized goldens). *(all test tasks + CI)*

## Data Model

### `layout.json` additions (schema_version stays 1; both blocks optional)

```jsonc
{
  "schema_version": 1,
  "objects": [ /* unchanged */ ],
  "robot": {
    "grab_zone": { /* unchanged */ }, "box_zone": { /* unchanged */ }, "drop_points": [ /* unchanged */ ],
    "vision": {                       // NEW — optional; robot-LOCAL (+x forward, +y left)
      "origin_x_cm": 12.0,            // cone apex, forward of robot centre
      "origin_y_cm": 0.0,             // lateral offset
      "half_angle_deg": 25.0,         // half-FOV; rays at ±this from forward; must be in (0, 90)
      "range_cm": null               // null/absent = unbounded (drawn as long rays); else > 0
    }
  },
  "mosaic": {                          // NEW — optional; top-level; WORLD frame (cm)
    "mode": "random",                 // "random" | "directed"
    "center_x_cm": 100.0, "center_y_cm": 75.0, "heading_deg": 0.0,   // seat + orientation
    "rows": 4, "cols": 3,             // rows*cols MUST == 12
    "cell_w_cm": 5.0, "cell_h_cm": 5.0,
    "row_gap_cm": 1.0, "col_gap_cm": 1.0,     // uniform gaps (single scalars)
    "palette": ["YELLOW", "BLUE", "GREEN", "WHITE"],
    "cells": [null,"YELLOW",null,null,null,null, null,"BLUE",null,null,null,null], // null=rolled, or a pinned palette color
    "counts": { "YELLOW": 3, "BLUE": 3, "GREEN": 4 },   // random: quota for NULL cells; sum == #nulls
    "seed": null                      // null = fresh per invocation; else a fixed non-negative int
  }
}
```

- **Cell ordering:** `i = row*cols + col`, `row ∈ [0,rows)` top→bottom, `col ∈ [0,cols)` left→right.
- **directed mode:** every `cells[i]` non-null; `counts` absent/ignored. **random mode:** `counts` present; `sum(counts) == count(null cells)`; pinned cells keep their color; null cells get the shuffled `counts` multiset.

### Python (`tools/path_planner/ctps/mosaic.py`, NEW — single source)

```python
COLOR_NAMES  = ("YELLOW", "BLUE", "GREEN", "WHITE")            # order/name must match Config::Color (minus UNKNOWN)
COLOR_LETTER = {"YELLOW":"Y","BLUE":"B","GREEN":"G","WHITE":"W"}
COLOR_HEX    = {"YELLOW":"#F2C200","BLUE":"#2563EB","GREEN":"#16A34A","WHITE":"#F5F5F5"}   # display only

def roll_mosaic(cfg, rng) -> list[str]: ...           # 12 color names by cell index; directed=pins; random=pins+shuffled pool
def resolve_cells(geom, color_names) -> list[dict]:   # [{index, x_cm, y_cm, color}] — world centres + colors; UNIT-TESTED (AC5)
    # local_x = (col - (cols-1)/2)*(cell_w_cm + col_gap_cm)
    # local_y = ((rows-1)/2 - row)*(cell_h_cm + row_gap_cm)          # row 0 at +y "top"
    # world   = (center_x_cm, center_y_cm) + Rot(heading_deg)·(local_x, local_y)
def geom_hash(geom) -> str: ...                        # stable fingerprint of the geometry params (for the trace echo)
```

### Trace additions (echoed by the dump, whitelisted by the loader)

```jsonc
"mosaic": { "cells": ["YELLOW", ...12...], "seed": 12345, "geom_hash": "ab12…", "injected": true }   // top-level, present iff dump ran with --mosaic
{ "seq": N, "op": "read_mosaic", "success": true }                                                    // command op; on failure success:false
```

### Embedded player constants (present only when a mosaic is resolved)

```jsonc
const MOSAIC_META = { "names":["YELLOW","BLUE","GREEN","WHITE"], "hex":{"YELLOW":"#F2C200", ...} };
const MOSAIC = { "cells":[{"index":0,"x_cm":…,"y_cm":…,"color":"YELLOW"}, …12…], "seed":12345, "mode":"random",
                 "geom_hash":"ab12…", "geom_match": true };   // geom_match=false → the "geometry changed since recording" badge
// robot.vision (cone) comes from the layout embed (const OBJECTS.robot.vision), NOT the trace.
```

## Contracts & Interfaces

Binding source of truth for every cross-task seam. A task conforms exactly; it does not redefine.

- **Unconditional-asset invariant:** `vision.js`, the `#ctps-vision` panel markup, and the cone/mosaic layer registration are on **every** player page (ADR-056 upload-leg rule). `MOSAIC_META` / `const MOSAIC` / trace `mosaic` are the only conditional artifacts. Owner: T10.
- **Schema keys:** the `mosaic` + `robot.vision` blocks exactly as in **Data Model**. Owner: `layout.py` (T2). Consumers: editor serialize (T16), orchestrator (T10), C++ dump argv (T6).
- **Cell ordering + geometry:** `i = row*cols+col` (row top→bottom, col left→right); the frame-local + world formula in **Data Model**. The **authoritative** implementation is Python `mosaic.resolve_cells` (T1, unit-tested per AC5); the editor's JS `mosaicCellCenters(geom)` (T11/T14) is a preview mirror validated against the same pinned vectors. The **same index `i`** is the position in `cells[]`, the `counts`-fill order, the trace `mosaic.cells[]`, `MOSAIC.cells[].index`, and the firmware `Vision.getColor(i)` index. **This ordering *defines* the intended cell 0** (row-major, top-left, world frame, camera viewing the mosaic from the robot's approach side — recorded in ADR-060); the bench-taught `VISION_CELL_TO_SLOT` must be made to match it, not vice-versa.
- **Color contract:** `COLOR_NAMES`/`COLOR_LETTER`/`COLOR_HEX` in `mosaic.py` (T1); emitted as `MOSAIC_META` (T10/T13); parsed letter→`Config::Color` against the real enum in the C++ mock (T4); guarded by `test_color_enum_parity` regex-parsing `enum class Color` from `ConfigVision.h` (T3).
- **Dump argv grammar (owner `ctps_dump.cpp` T6):** the existing `strcmp(argv[1])` mode selector is unchanged; the mosaic flags are order-free options that compose with `--fixture`. `--mosaic <12 letters>` (letters per `COLOR_LETTER`) sets the mock injection + is echoed (as names). `--mosaic-seed <int>` and `--mosaic-geom-hash <str>` are echoed verbatim (Python owns both; the dump does not roll or hash). `--mosaic-fail-at <int>` = the Nth `readMosaic()` returns false (0/absent = never). Absent `--mosaic-seed` ⇒ the `seed` key is omitted from the echo. No `--mosaic` ⇒ no injection, no echo. A malformed `--mosaic` exits non-zero.
- **Reproduce flag:** the **user-facing** flag is the orchestrator's `--seed`; the orchestrator resolves the seed and passes it to the dump as `--mosaic-seed`. Never document `--mosaic-seed` as a user flag.
- **Trace shapes:** top-level `mosaic` block + `read_mosaic` op exactly as in **Data Model**. `op:"read_mosaic"` is the discriminator (never a sub-field). The `read_mosaic` op carries **`success` only** (the colors are the top-level `mosaic.cells` on success — no redundant per-op color copy). Owner: recorder `ctps_record.{h,cpp}` (T5); whitelist `trace_loader.py` (T8); event `track_builder.py` (T9).
- **JS predicate API (owner `assets/vision.js` T11; consumers HUD T11, editor draw T14):**
  - `visibleCells(pose, visionCfg, cells) -> bool[12]` — apex = `localToWorld(pose, origin_x_cm, origin_y_cm)`; per cell, in robot-local: `localVec.x > 0` AND `abs(degrees(atan2(localVec.y, localVec.x))) <= half_angle_deg` AND (`range_cm` null OR `hypot(localVec) <= range_cm`). *(degrees on both sides.)*
  - `readMosaicWouldSucceed(bools) -> bool` = all true.
  - `mosaicCellCenters(geom) -> [{index,x_cm,y_cm}]` (editor preview mirror of `resolve_cells` geometry).
  - `drawVisionCone(ctx, view, pose, visionCfg, project)`, `drawMosaicCells(ctx, view, cells, project)` — reuse `CTPS_OBJ.project`/`localToWorld`.
- **Player DOM/CSS seam:** panel id `#ctps-vision` (a **visible** panel sibling to `#ctps-cargo`, never `#ctps-hud`). Layer chips `data-layer="vision-cone"` / `data-layer="mosaic"`. Player classes `ctps-vision-*` in `components.css`; editor Mosaic-tab / pin-grid / cone-field classes `ctps-mosaic-*` in `editor.css`. CSS custom properties, no inline styles (ADR-053). Owners: markup T10 (player) / T13 (editor); behaviour T11/T14; style T12 (both sheets).
- **Pin-grid interaction (owner T15):** a 4×3 clickable grid; clicking a cell cycles `unset(rolled) → palette[0] → … → palette[n-1] → unset`. "Unset/rolled" renders as an outlined empty cell, visually distinct from a filled WHITE (which is a swatch with a border). Basic left/right-arrow focus optional.
- **C++ mock surface (owner `ctps_mocks/Vision.h` T4):** match the real `VisionClass` subset a mission calls: `begin()`, `start()`, `readMosaic()`, `getColor(uint8_t)`, `getRawHue(uint8_t)`, `hasFault()`, `getFault()`, `clearFault()`, `setTickCallback(void(*)())`, the `Fault`/`FaultInfo` types, `extern VisionClass Vision;`. Test-set hooks: inject colors, set fail-at. Records `read_mosaic` on each `readMosaic()`.

### File ownership (every file has exactly one writer task)
| File | Owner task | Consumers (read only) |
|------|-----------|-----------------------|
| `tools/path_planner/ctps/mosaic.py` (NEW) | T1 | T2, T8, T10, T18, T3 |
| `tools/path_planner/ctps/layout.py` | T2 | T13, T16, T19, T10 |
| `tools/path_planner/ctps/trace_loader.py` | T8 | T9, T20 |
| `tools/path_planner/ctps/track_builder.py` | T9 | T10, T21 |
| `tools/path_planner/ctps/player_html.py` + `__main__.py` | T10 | T11, T12, T21 |
| `tools/path_planner/ctps/layout_editor_html.py` | T13 | T14–T17, T21 |
| `tools/path_planner/ctps/assets/vision.js` (NEW) | T0 stub → T11 fleshes | T14 |
| `tools/path_planner/ctps/assets/store.js` | T11 | — |
| `tools/path_planner/ctps/assets/components.css` + `tokens.css` + `editor.css` | T12 | — |
| `tools/path_planner/ctps/assets/editor_core.js` | T14 | T15, T17 |
| `tools/path_planner/ctps/assets/editor_tools.js` | T15 | T17 |
| `tools/path_planner/ctps/assets/editor_io.js` | T16 | T17 |
| `tools/path_planner/ctps/assets/editor_boot.js` | T17 | — |
| `WRO2026/test/ctps_mocks/Vision.h` (NEW) | T4 | T6, T7 |
| `WRO2026/test/ctps_mocks/ctps_record.{h,cpp}` | T5 | T6 |
| `WRO2026/test/ctps_dump.cpp` | T6 | T22 |
| `WRO2026/test/CMakeLists.txt` | T7 | T22 |

*(No source change to `assets/objects_draw.js` — `vision.js` reuses its existing `CTPS_OBJ` exports.)*

## Error Handling

- **Malformed `mosaic`/`robot.vision`:** `layout.py.validate` raises `LayoutError` LOUD with the key path; the in-browser editor ingest is tolerant (skip+`console.warn`).
- **Unknown/malformed trace `mosaic` block or `read_mosaic` op:** `trace_loader` skips-with-`UserWarning`, never fatal.
- **`ctps_dump --mosaic` bad letter / wrong length:** clear stderr, exit non-zero, no partial `mosaic` block (tested — TC16b).
- **No `mosaic` block in the trace (old trace / dump ran without `--mosaic`):** the player draws the mosaic grid outline from the layout geometry (if present) with a "no roll recorded" badge; never synthesizes colors client-side. No geometry either ⇒ nothing renders (feature inactive).
- **Geometry-hash mismatch (`--trace` against a re-seated/re-gapped mosaic):** `player_html` sets `MOSAIC.geom_match=false`; the player shows a "geometry changed since this trace was recorded" badge.
- **Cone without mosaic (or vice-versa):** draw whichever is present; HUD reads "no mosaic to read" / "no vision cone configured"; no crash.
- **`half_angle_deg` outside (0,90) or `range_cm <= 0`:** rejected by `layout.py` with a fix hint.

## Testing Strategy

**Levels:** Unit (Python roll/resolve/validators/color guard; a Python mirror of the JS predicate), Integration (trace_loader round-trip; host C++ dump argv→echo→op), Golden/region (seeded mosaic-present region asserts + mosaic-free invariants, matching the repo's literal/substring convention — **not** full-page byte goldens), Manual (localhost eyeball).

**No-JS-runner note:** `vision.js` has no automated runner. The load-bearing geometry (`resolve_cells`) is Python and unit-tested (AC5). The `visibleCells` predicate is JS; T18 provides a Python **mirror** (pinned vectors) whose docstring explicitly names the mirror-vs-`vision.js` drift gap, and T24 must include a boundary case (a cell *just* outside the half-angle) that exercises the real JS. Keep the predicate point-in-sector so both stay credible.

| ID | Test Case | Type | Expected |
|----|-----------|------|----------|
| TC1 | `roll_mosaic` same seed twice | Unit | Identical 12-color output |
| TC2 | `roll_mosaic` random, counts {Y:3,B:3,G:4,W:2}, no pins | Unit | Output multiset == counts; all filled |
| TC3 | `roll_mosaic` random, 2 pins + counts summing to 10 | Unit | Pins kept; nulls == remaining multiset |
| TC4 | `roll_mosaic` directed (all 12 pinned) | Unit | Output == pins verbatim; seed irrelevant |
| TC5 | `resolve_cells` rotated + unequal gaps, pinned expected vectors | Unit | 12 centres match; index/`+y`-top/gap-axis correct |
| TC6 | validate `rows*cols!=12` / `cells` len≠12 / directed with null / random `sum(counts)!=#nulls` / bad palette color | Unit | `LayoutError` each, key path |
| TC7 | validate `half_angle_deg`=0/95, `range_cm`=0, seed=-1/1.5 | Unit | `LayoutError` each |
| TC8 | color-enum parity vs `ConfigVision.h` (regex targets the enum body between braces; ignores the surrounding `#pragma push_macro("WHITE")`/`#undef`) | Unit | `COLOR_NAMES` == parsed enum names/order |
| TC9 | `visibleCells` dead-ahead in range / behind / outside half-angle / beyond range; `readMosaicWouldSucceed` 12/12 vs 11/12 | Unit (py mirror) | true / false as specified |
| TC10 | `trace_loader` valid `mosaic` block + `read_mosaic` op | Integration | Parsed onto Trace |
| TC11 | `trace_loader` malformed `mosaic`/`read_mosaic` | Integration | Skipped-with-warning |
| TC12 | `ctps_dump --mosaic YGBW… --mosaic-seed 7 --mosaic-geom-hash h` | Host C++ | JSON `mosaic:{cells,seed:7,geom_hash:"h",injected:true}` |
| TC13 | `ctps_dump` no `--mosaic` | Host C++ | JSON byte-identical to baseline (no `mosaic` key) |
| TC14 | Vision mock `getColor`/`readMosaic`/`--mosaic-fail-at 1`; `read_mosaic` op recorded | Host C++ | Colors returned; 1st read false; op present |
| TC15 | `ctps_dump` builds with the Vision mock shadow (full `Tasks.cpp`) | Build | Link+run OK; real `Vision.h` not leaked |
| TC16 | `ctps_dump --mosaic ZZZ…` (bad letters) / wrong length | Host C++ | Exit non-zero, stderr, no partial block |
| TC17 | Player render, seeded mosaic+cone | Region golden | `MOSAIC_META`/`const MOSAIC`/`#ctps-vision` regions stable (seed pinned) |
| TC18 | Player render, **no** mosaic block | Region golden | No `MOSAIC_META`/`const MOSAIC`; TC9 `PLAYER_META` literal unchanged; existing suite green |
| TC19 | Player render, mosaic **geometry present but trace has no roll** | Region golden | Grid-outline markup + "no roll recorded" badge |
| TC20 | Editor render + Save round-trip of `mosaic`+`robot.vision` | Golden/Unit | Save re-`validate`s and equals input |
| TC21 | Upload a `mosaic`/`robot.vision` layout into a mosaic-free-emitted player page | Golden/markup | `vision.js`+`#ctps-vision` present (unconditional) so the upload leg renders |

**Run:** `uv run --extra dev pytest tools/path_planner/tests/` and `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T0 | `vision.js` stub | — | low | `tools/path_planner/ctps/assets/vision.js` (NEW) | Create `vision.js` as a minimal valid stub (a no-op IIFE registering nothing) so every task that inlines it finds a file. Unblocks the include-adders (T10/T13) without waiting on the full implementation (judge C1). |
| T1 | Mosaic roll + resolve + color contract | — | med | `tools/path_planner/ctps/mosaic.py` (NEW) | `COLOR_NAMES`/`COLOR_LETTER`/`COLOR_HEX` (single source), a `MosaicConfig` dataclass, `roll_mosaic(cfg,rng)` (directed=pins; random=pins+seeded shuffle of `counts` into nulls), `resolve_cells(geom,colors)` (world centres per the Data Model formula + colors), `geom_hash(geom)`, and pure validators reused by `layout.py`. Satisfies AC2, AC5. |
| T2 | `layout.json` schema: `mosaic` + `robot.vision` | T1 | med | `tools/path_planner/ctps/layout.py` | `MosaicConfig`/`VisionCone` dataclasses + strict `_validate_mosaic`/`_validate_vision` (all AC3 rules), wired into `validate`/`load`/`to_dict`; both optional; reuse T1 validators. Satisfies AC1, AC3. |
| T3 | Color-enum parity guard | T1 | low | `tools/path_planner/tests/test_ctps_color_enum_parity.py` (NEW) | Regex-parse the `enum class Color` **body between braces** out of `WRO2026/config/ConfigVision.h` (ignore the surrounding `#pragma push_macro("WHITE")`/`#undef`; strip `UNKNOWN`) and assert it equals `mosaic.COLOR_NAMES` in name+order. Satisfies AC4. |
| T4 | Thin `Vision` mock | — | high | `WRO2026/test/ctps_mocks/Vision.h` (NEW) | Thin mock of the real `VisionClass` surface (Contracts): inject colors + fail-at; `getColor` returns injected; `readMosaic` returns true / false-at-N and records a `read_mosaic` op via `ctps_record`; parse letters→`Config::Color` against the real `ConfigVision.h` (compiled in). Record + test-set only (ADR-049). Satisfies AC7. |
| T5 | Trace `mosaic` echo + `read_mosaic` op (recorder) | — | high | `WRO2026/test/ctps_mocks/ctps_record.{h,cpp}` | Add `Op::READ_MOSAIC` + `recordReadMosaic(success)` (success only — no per-op color copy) + `setInjectedMosaic(colors[12], seed?, geomHash?)`; serialize the op case + the top-level `mosaic` echo (omit `seed` when unset) in `commandStreamToJson`. Satisfies AC6, AC8 (producer). |
| T6 | `ctps_dump` argv + wiring + self-test | T4, T5, T7 | high | `WRO2026/test/ctps_dump.cpp` | Parse `--mosaic`/`--mosaic-seed`/`--mosaic-geom-hash`/`--mosaic-fail-at` (order-free, composes with `--fixture`; bad letters/length → non-zero exit + stderr); set the mock injection + `setInjectedMosaic` echo; add a synthetic `read_mosaic` emission in the self-test segment so the op pipeline is CI-proven with no real consumer. No `--mosaic` ⇒ no echo. Satisfies AC6, AC7. |
| T7 | CMake: mock shadow (ctps_dump target ONLY) + test target | T4 | med | `WRO2026/test/CMakeLists.txt` | Force-include / include-path `ctps_mocks/Vision.h` for the **`ctps_dump` target only** (do NOT add it to the shared `add_ctps_mock_force_includes`, which `test_ctps_mock_conformance` mirrors — ADR-049/051 scope). Register the T22 host test target. Owns all `CMakeLists.txt` edits. Satisfies AC7 (build). |
| T8 | `trace_loader`: whitelist `mosaic` + `read_mosaic` | T1 | med | `tools/path_planner/ctps/trace_loader.py` | Add `read_mosaic` to `_KNOWN_OPS` (fields: `success`), parse/validate the top-level `mosaic` block (tolerant skip-with-warning); expose both on the `Trace`. Satisfies AC8 (consumer). |
| T9 | `track_builder`: carry `mosaic` + emit `read_mosaic` events | T8 | med | `tools/path_planner/ctps/track_builder.py` | Pass the trace `mosaic` block onto the `Track`; emit a `read_mosaic` timeline event when present. No new timeline lane for the dormant case. |
| T10 | Orchestrator roll + player embed + HUD markup | T2, T9, T1, T0 | med | `tools/path_planner/ctps/__main__.py`, `tools/path_planner/ctps/player_html.py` | `__main__`: resolve seed (`--seed`/`mosaic.seed`/fresh), roll via `mosaic.roll_mosaic`, pass `--mosaic`/`--mosaic-seed`/`--mosaic-geom-hash` to the dump **only when the layout has a `mosaic` block**. `player_html`: `resolve_cells(layout geom × trace colors)` → embed `const MOSAIC` (+ `geom_match` from comparing the trace `geom_hash`) + `MOSAIC_META` (both conditional on a mosaic); **unconditionally** add the `#ctps-vision` panel markup + `data-layer` chips + the `vision.js` include (ADR-056 upload-leg). Sole writer of both files. Satisfies AC6, AC10, AC12. |
| T11 | `vision.js`: predicate + layers + HUD | T0, T10 | high | `tools/path_planner/ctps/assets/vision.js`, `assets/store.js` | Flesh out the stub: `visibleCells`, `readMosaicWouldSucceed`, `mosaicCellCenters` (editor mirror), `drawVisionCone`/`drawMosaicCells` (reuse `CTPS_OBJ.project`/`localToWorld`); register the per-frame cone layer + the mosaic-cells layer (ADR-053 registries, off `poseAt(t)` — NOT the resolve-once scene); update `#ctps-vision` per frame + the no-roll / geom-mismatch badges. `store.js`: hold + expose the mosaic/vision embed + upload-leg loadLayout. Satisfies AC9, AC10. |
| T12 | Player + editor CSS | — | low | `tools/path_planner/ctps/assets/components.css`, `assets/tokens.css`, `assets/editor.css` | Own `ctps-vision-*` (player, `components.css`) + `ctps-mosaic-*` (editor Mosaic tab / 12-cell pin grid / counts inputs / cone fields, `editor.css`) + shared tokens (`tokens.css`), CSS custom properties, match existing CTPS look. Closes the CSS-owner gap for BOTH pages. |
| T13 | Editor: Mosaic tab + Robot-tab cone markup | T2, T0 | med | `tools/path_planner/ctps/layout_editor_html.py` | Add a third **Mosaic** tab (center/heading/rows/cols/cell size/row+col gap/palette/mode + a 4×3 pin grid + counts) and vision-cone inputs on the Robot tab; emit `MOSAIC_META`; **unconditionally** include `vision.js`; extend `reembed_layout` for the mosaic/vision embed. Sole writer of this file. |
| T14 | Editor core: mosaic + cone state + draw | T11, T13 | med | `tools/path_planner/ctps/assets/editor_core.js` | Default mosaic/vision state (mosaic default seat = **field centre** per the field bounds); draw the mosaic grid (reuse `vision.js` `mosaicCellCenters`/`drawMosaicCells`) on the Mosaic tab and the cone (`drawVisionCone`, pose identity) on the Robot tab; extend bounds to include the mosaic. |
| T15 | Editor tools: select/edit mosaic + cone | T14 | med | `tools/path_planner/ctps/assets/editor_tools.js` | Property-panel editing of mosaic geometry/gaps/palette/mode, the per-cell pin grid (cycle per the Contracts interaction), and the cone fields; numeric-input-first. |
| T16 | Editor IO: serialize + ingest | T2, T14 | med | `tools/path_planner/ctps/assets/editor_io.js` | Serialize `mosaic` + `robot.vision` into the schema on Save; tolerant ingest on load. Satisfies AC11. |
| T17 | Editor boot wiring | T13, T14, T15, T16 | low | `tools/path_planner/ctps/assets/editor_boot.js` | Register the Mosaic tab, bind its + the cone inputs to state/redraw/serialize; auto-load from the embedded layout. |
| T18 | `test_ctps_mosaic.py` (+ predicate mirror) | T1 | low | `tools/path_planner/tests/test_ctps_mosaic.py` (NEW) | TC1–TC5 (roll + resolve pinned vectors) + TC6/TC7 (validation) + TC9 (a Python **mirror** of `visibleCells`/`readMosaicWouldSucceed`, docstring naming the mirror-vs-`vision.js` drift gap). Satisfies AC2, AC3, AC5, AC9. |
| T19 | `test_ctps_layout.py` additions | T2 | low | `tools/path_planner/tests/test_ctps_layout.py` | Schema-validation cases for `mosaic` + `robot.vision` (valid + each rejection incl. seed). Satisfies AC1, AC3. |
| T20 | `test_ctps_trace_loader.py` additions | T8 | low | `tools/path_planner/tests/test_ctps_trace_loader.py` | TC10, TC11 — valid + malformed `mosaic` block and `read_mosaic` op. Satisfies AC8. |
| T21 | Render goldens (region + mosaic-free + upload) | T10, T11, T12, T13, T14, T15, T16, T17 | med | `tools/path_planner/tests/test_ctps_player_html.py`, `test_ctps_layout_editor.py` | TC17 (seeded mosaic+cone region asserts), TC18 (mosaic-free: no data + TC9 literal + suite green; update any frozen asset-list/markup test for the new unconditional additions), TC19 (no-roll grid outline), TC20 (editor round-trip), TC21 (upload-leg). Region/substring asserts, not full-page byte goldens; LF-normalized. Blocked by all render-affecting tasks (they all inline into the pages). Satisfies AC11, AC12, AC13. |
| T22 | Host C++ ctest: mock + dump | T6, T7 | med | `WRO2026/test/test_ctps_dump*.cpp` (NEW; registered by T7) | TC12–TC16 — argv→echo, no-arg baseline, mock colors/fail-at/`read_mosaic`, shadowed full-`Tasks.cpp` build, malformed-argv exit. Satisfies AC6, AC7, AC13. |
| T23 | ADR-060 + doc updates | — | low | `docs/decisions/060-ctps-randomized-vision.md` (NEW), `DECISIONS.md`, `ARCHITECTURE.md`, `CLAUDE.md`, `AGENTS.md` | Write ADR-060 (trace-carried roll + geometry fingerprint; unified pinned-cells schema; Python-tested geometry; unconditional vision assets; player-side per-cell predicate; thin dormant Vision mock + `read_mosaic` op; header-parse color guard; **the cell-ordering-defines-cell-0 + camera-viewing-side convention** the bench mapping must match). Verify no ADR-number collision before commit. Update DECISIONS.md index+table and the CTPS/vision module lines. |
| T24 | Localhost eyeball verification | T11, T12, T21, T17 | med | (no source; verification) | Serve the rendered player + editor via `python -m http.server` → 127.0.0.1 (claude-in-chrome blocks `file://`); confirm the cone follows the robot across scrub incl. a **boundary case** (a cell just inside vs just outside the half-angle), the mosaic cells color correctly, `#ctps-vision` updates per frame + greens at 12/12, the no-roll + geom-mismatch badges show, and the editor Mosaic tab + pin grid + Robot cone author + Save round-trip and match the player render. Fix any unstyled-panel / collapsed-canvas issues. |

## Notes for Implementer

- **Branch hygiene:** the working tree is on `feat/better-manip-verb-and-small-fixes` with unrelated uncommitted changes to `WRO2026/Tasks.cpp` / `Tasks.h`. This feature branches off **`main`** as `feat/ctps-randomized-vision` and must **not** include those two files.
- **Unconditional assets, conditional data.** `vision.js` + `#ctps-vision` markup + layer registration ship on every page (the upload leg needs them, ADR-056). Only `MOSAIC_META`/`const MOSAIC`/trace `mosaic` are conditional. Verify TC18 (mosaic-free: no data, TC9 literal unchanged) AND TC21 (upload leg) early — they are the guards that keep existing goldens green while not regressing the upload path.
- **Geometry is Python-authoritative + tested.** `resolve_cells` (T1) is the source of truth for the cell→`getColor` mapping; the editor's JS `mosaicCellCenters` is a preview mirror — they must agree (T24 eyeball). Pin the same expected vectors for both.
- **`ctps_dump` compiles ALL of `Tasks.cpp`.** T4/T7 (mock + its ctps_dump-only CMake shadow) must land before/with T6, or the dump build breaks (the `ctps-dump-compiles-all-tasks` gotcha). Probe the shadow include order (SKETCH_DIR vs mock dir) per the host-link-inversion gotcha. Do NOT add `Vision.h` to the shared `add_ctps_mock_force_includes` (it would drag the mock into `test_ctps_mock_conformance`).
- **Byte-frozen surfaces:** do not touch the seek core / `renderer.js` / the `PLAYER_META` literal. New draw code registers a **layer** (ADR-053). The cone is a per-frame overlay off `poseAt(t)`, NOT part of the resolve-once cargo scene (ADR-056).
- **HUD anchoring:** put status in the visible `#ctps-vision` panel, never `#ctps-hud` (`hidden`/0×0 — the `ctps-stage-hud-hidden-by-default` gotcha). Watch the collapsed-canvas-in-unfocused-tab gotcha when eyeballing.
- **Cross-platform:** LF-normalize any hashed/golden output (the CRLF-vs-LF gotcha); the C++ dump build path differs on Windows (vcvars+Ninja) vs POSIX (plain cmake).
- **Color source of truth:** never hardcode a color name/hex/letter outside `mosaic.py` / `MOSAIC_META` / the real `Config::Color`. T3 fails CI on drift.
- **Cell-0 convention:** the sim's row-major/top-left/world-frame ordering *defines* cell 0; the bench-taught `VISION_CELL_TO_SLOT` must be made to match it, and the camera viewing-side assumption is recorded in ADR-060. Do not assume the placeholder identity mapping is authoritative.
- **Rollback:** the feature is additive; reverting the `__main__.py` injection + the emitted `const MOSAIC` disconnects the read/render while the (always-present) vision assets sit inert.
- **CONTRIBUTING:** Conventional-Commit style; labels `area: sim/ctps` + `area: vision`; the PR body needs a `Doxygen reviewed for affected APIs before implementation.` line and a "how it affects the robot: no runtime effect (tooling; plumbing latent until a vision-consuming mission exists)" note.
