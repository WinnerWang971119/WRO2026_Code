# CTPS Object Authoring + Cargo Animation Plan (#57)

**Goal:** Make the CTPS player animate a manipulator cargo model — grab tiles into
a magazine, capture cargo in a rear box, dump/open them back onto the field — over
a field of hand-placed objects authored in a new offline editor page.

**Approach:** A **new standalone offline editor page** (`layout_editor.html`, two
tabs: Field + Robot) authors one `layout.json` (field objects + robot manipulator
zones), saved via Blob download / loaded via FileReader. The **existing player only
consumes** that layout (embedded at emit time and/or uploaded in-browser) and
extends purely through ADR-053's registries — a new JS module adds an objects
layer, a zones overlay, and the two-tab cargo panel, driven by a **resolve-once
event-indexed scene timeline** computed from the trace's manipulator events. The
pinned player seek core (`player.js`) and `renderer.js` are **not refactored** —
shared logic ships as *new* files both pages inline; the ~40-line world→pixel
transform is duplicated rather than extracted. The only firmware change is two new
`Manipulator` verbs (`boxOpen`/`boxClose`) landed on the production facade and the
recording mock together.

This plan supersedes the vaguer wording of issue #57; the mechanism below is the
refined design agreed with the user.

## Scope

- **In scope:**
  - New offline editor page `layout_editor.html` with a **Field tab** (map
    backdrop; place Rect/Circle objects — each named, colored, typed
    Tile/Cargo/Fixed; rubber-band draw + precise numeric dims) and a **Robot tab**
    (default robot box; draw the grab zone, rear-box zone, and 4 drop points).
  - `layout.json` schema + strict Python validation + tolerant JS parse.
  - Save/load: Blob `<a download>` + `<input type=file>`/FileReader (the `file://`
    persistence path — no localStorage / File System Access API).
  - Player consumes the layout two ways: **emit-time embed** (`player_html.render`
    gains an `objects=` arg → inline `const OBJECTS`) **and** in-browser upload.
  - Player object layer + zones overlay (faint, robot-local, toggled by new
    OBJECTS / ZONES chips), plus the two-tab cargo panel (Magazine 4×3 vertical
    lanes + Inventory rear box), the HUD holding line, and the Holding timeline
    lane — all fed by a resolve-once scene timeline.
  - Cargo semantics: `grab` (nearest Tile in grab zone → in-hand), `sortTo(n)`
    (in-hand → magazine lane n), `dump` (front tile of each non-empty lane → its
    drop point), `boxClose`/`boxOpen` (capture/drop Cargo at the rear-box zone),
    `reset` (home; no cargo change). Held objects leave the field into the sidebar
    inventory and reappear at drop points.
  - Two new firmware verbs `boxOpen`/`boxClose` on `Manipulator` + the ctps
    recording mock; structured `method`/`arg` keys added to the `manip` track
    event. Synthetic fixtures + tests.
  - ADR-056 documenting the object/cargo model + the two-page split.

- **Out of scope:**
  - Physics, collision, stacking realism (spatial capture/pickup are simple
    center-in-zone / nearest tests).
  - Real mission animation now — no task body emits these verbs yet; v1 is
    exercised by synthetic fixtures and goes live automatically once task bodies
    call the verbs. (Not a "deferred unbuilt feature" dodge — the player-side
    feature is fully built and tested against fixtures.)
  - Manipulator calibration / numeric servo setpoints (ADR-032 bring-up).
  - Editing field *geometry* (nodes/edges/map) — that stays in the Tk field editor.
  - The color-cell mosaic object (explicitly cut by the user; objects are
    Rect/Circle only).
  - Refactoring `renderer.js`/`player.js` or extracting their field-draw/transform
    (duplicate instead).
  - Editor niceties deferred if pressed: multi-level undo, per-object
    `inspectorFields` in the player, image/silhouette robot import.

## Decisions

- **Two offline pages, not one author-mode player** — user-chosen; keeps the
  frozen player core + its exact test contracts (TC9/TC-STRUCT/TC-NOINLINE) out of
  editor churn. (Fable consult: ENDORSED.)
- **Share only NEW files; do not refactor `renderer.js`** — the world→pixel
  transform (~40 lines) is duplicated in the editor; object/zone drawing is a new
  shared `objects_draw.js` both pages inline; emit scaffolding (lint / `<script>`
  wrap / asset read) is a new shared `_page_emit.py`. Extracting from `renderer.js`
  (adjacent to the byte-frozen seek core) is high-risk for ~40 lines of stable math.
  (Fable VETO of "extraction-first" — adopted.)
- **Cargo = resolve-once event-indexed scene timeline, not fold-on-scrub** —
  object positions change only *at* manip events, so on layout-load a single pure
  pass (events × `poseAt(t)` × zone geometry) precomputes a scene snapshot per
  manip-event index; scrubbing is a pure bisect (like the Log panel). Eliminates
  the backward-seek incremental-state bug family the text-only pytest harness can't
  see, and runs the spatial geometry in exactly one place. (Fable VETO of
  fold-on-scrub — adopted.)
- **The op contract is 3-place but the loader leg is already open** —
  `trace_loader._KNOWN_OPS` already contains `manip`, and `manip.method` is NOT
  whitelisted, so `grab`/`release`/`dump`/`sortTo`/`boxOpen`/`boxClose` method
  strings flow through untouched. Four of the five existing verbs already record in
  the mock. Net new C++ = `boxOpen`/`boxClose` only, which per ADR-051 must land on
  the production `Manipulator` facade AND the mock together.
- **`track_builder` adds structured `method`/`arg` to the `manip` event** — today
  it flattens to a display `label`; the player must never parse labels. Additive:
  manip-free traces stay byte-identical.
- **Additive, conditional emission** — `const OBJECTS` and any new `PLAYER_META`
  key appear only when a layout is present (the `pivots` precedent). Layout-free
  traces emit byte-identically → TC9's exact-meta literal stays green.
- **Player-first phasing** — schema+fixtures → player consume+animate → editor last
  (the player validates the schema before the editor is built against it).
- **Codex approach consult skipped** — Codex runtime did not return
  (`/codex:setup` to enable). Proceeded on the Fable consult alone.

## Acceptance Criteria

- [ ] AC1 — `layout_editor_html.render()` emits a single self-contained offline
  `.html` that passes the self-containment lint and opens on `file://` with no
  network/local dependency.
- [ ] AC2 — Field tab: rubber-band-draw a Rect and a Circle, set name / type
  (Tile/Cargo/Fixed) / color / precise dims + x,y; each renders at the correct
  world position over the field backdrop.
- [ ] AC3 — Robot tab: draw the grab zone, the rear-box zone, and 4 per-lane drop
  points on the default robot box; they persist into `layout.json` in robot-local
  coordinates.
- [ ] AC4 — Save downloads a schema-valid `layout.json` (Blob); Load restores it
  (FileReader); a save→load round-trip is loss-free.
- [ ] AC5 — The player, given a layout (embedded `const OBJECTS` OR uploaded),
  renders Tile/Cargo/Fixed objects at their world positions; OBJECTS/ZONES chips
  toggle the objects layer and the zones overlay.
- [ ] AC6 — On a `grab` event the nearest Tile whose center is inside the
  world-projected grab zone leaves the field into the in-hand slot; a `grab` with
  no Tile in the zone is a logged no-op (nothing grabbed).
- [ ] AC7 — `sortTo(n)` moves the in-hand tile into magazine lane `n` (front-to-
  back); `dump` ejects the front (bottom) tile of each non-empty lane to that
  lane's world drop point, and the object reappears on the field there.
- [ ] AC8 — `boxClose` captures the nearest Cargo overlapping the world-projected
  rear-box zone (capacity 1; extra overlaps ignored + logged); `boxOpen` drops the
  held Cargo back on the field at the rear-box zone.
- [ ] AC9 — The right-rail cargo panel shows two tabs (Magazine 4×3 vertical lanes,
  Inventory rear box) with live counts; the HUD holding line and the Holding
  timeline lane reflect the same state.
- [ ] AC10 — Scrubbing to any `t` (forward or backward, and after a jump) shows the
  exact cargo + object placement for that instant; no stale/incremental drift.
- [ ] AC11 — A layout-free trace emits byte-identically to before this change
  (no `const OBJECTS`; `PLAYER_META` literal unchanged → TC9 green).
- [ ] AC12 — `boxOpen`/`boxClose` exist on the production `Manipulator` facade AND
  the ctps recording mock (conformance guard passes; `ctps_dump` compiles); the
  loader accepts them with no whitelist change; `manip` track events carry
  structured `method`/`arg`.
- [ ] AC13 — No JS asset uses a static inline `style=` or `el.style.prop =`
  (TC-NOINLINE), and no asset trips the self-containment lint.

## UI/UX Specification

> Carried from the confirmed mockup (rendered in the real CTPS tokens). Full design
> — drawing, description, hex tokens, states, responsive — lives in the sidecar
> `docs/plans/2026-07-06-ctps-object-cargo.ui.md`. The executor reproduces THIS; do
> not re-design. This feature EXTENDS ADR-053's design system; it introduces no new
> palette (every color is an existing `tokens.css` custom property).

**Chosen base:** the existing CTPS "Instrument" dark player, extended.
**Grafted in:** none (single confirmed direction).

### Layout (drawing)

```
Right rail — Holding panel becomes a two-tab CARGO panel:
  [ Magazine 7/12 ] [ Inventory 1/1 ]        (count badges on the tabs)
  ── active tab underline = --ctps-hold ──
  MAGAZINE: 4 VERTICAL lanes (0–3), stack of 3, front tile at BOTTOM
     0    1    2    3
    [B3] [  ] [  ] [  ]
    [G2] [C5] [  ] [O7]
    [A1] [Y4] [  ] [W6]
     ↓    ↓    ↓    ↓     sortTo(n) fills lane n · dump ejects the front of each
  INVENTORY: rear box (cap 1) — hold card: ▪ name … [onrobot]

Field (player canvas): Tile squares (filled), Cargo (--ctps-hold outline),
  Fixed (faint dashed) + faint robot-local ZONES overlay (grab / rear-box /
  4 drop points) that moves with the robot; new OBJECTS + ZONES layer chips.

Editor (layout_editor.html): [Field][Robot] tabs · Rect/Circle tools ·
  Properties panel (name/type/color/precise dims) · Save/Load buttons.
```

### Description

The single scaffolded Holding panel becomes a two-tab cargo panel (Magazine +
Inventory), one tab per on-robot store, following the existing tab idiom but as its
own rail section (not the log/inspector tab row, not registered via `panelTabs`).
The magazine is 4 vertical lanes of 3, front tile at the bottom (dump side). On the
field, objects render by type and grabbed objects vanish into the sidebar; a faint
zone overlay shows the manipulator geometry moving with the robot. The editor is a
separate offline two-tab page authoring the same `layout.json`. Full behavior +
states in the sidecar.

### Color codes

See the sidecar table. Cargo accent is `--ctps-hold #5bc8e0`; robot stays
`--ctps-accent #ffd24a`; object fills are the authored per-object hex applied via
`setProperty` (never a hard-coded literal). No new palette hues.

## Data Model

### `layout.json` (schema_version 1)

```jsonc
{
  "schema_version": 1,
  "objects": [
    {
      "id": "o1",            // stable unique string
      "name": "green tile",  // user text (shown in inventory/log)
      "shape": "rect",        // "rect" | "circle"
      "type": "tile",         // "tile" | "cargo" | "fixed"
      "color": "#57b36b",     // authored hex
      "x_cm": 40.0, "y_cm": 120.0,   // world center (cm), same frame as field.nodes
      "heading_deg": 0.0,     // rect orientation (ignored for circle)
      "w_cm": 5.0, "h_cm": 5.0,      // rect dims (present for shape=="rect")
      "radius_cm": 3.0        // circle radius (present for shape=="circle")
    }
  ],
  "robot": {                  // robot-LOCAL frame: origin = robot center,
                              // +x = heading-forward, +y = robot-left; the player
                              // maps each to world via poseAt(t) (rotate+translate)
    "grab_zone": { "x_cm": -1.0, "y_cm": 8.0, "w_cm": 6.0, "h_cm": 14.0 },
    "box_zone":  { "x_cm": -9.0, "y_cm": 0.0, "w_cm": 6.0, "h_cm": 16.0 },
    "drop_points": [ {"x_cm": 4, "y_cm": -6}, {"x_cm": 4, "y_cm": -2},
                     {"x_cm": 4, "y_cm": 2},  {"x_cm": 4, "y_cm": 6} ]  // one per lane 0–3
  }
}
```

- `objects[]` is validated loud in Python (`ctps/layout.py`), tolerant in JS
  (skip+warn a malformed object, never throw — mirrors the loader convention).
- Magazine geometry (4 lanes × 3) and rear-box capacity (1) are fixed player
  constants in `ctps/config.py` mirrored as JS constants — NOT in `layout.json`.
- Robot-local coordinate frame is defined once here and in Contracts; the editor
  Robot tab and the player resolver both use it.

### `manip` track event (additive keys)

`track_builder` currently emits `{ "t", "type": "manip", "label": "grab(0)" }`.
Add `"method"` and `"arg"`:

```jsonc
{ "t": 8.42, "type": "manip", "method": "grab", "arg": 0, "label": "grab(0)" }
```

`label` is kept for display/back-compat; the player reads `method`/`arg` and never
parses `label`. Additive → manip-free traces byte-identical.

### Resolved scene timeline (player, in-memory only — not serialized)

Computed once on layout-load by `cargo_resolve(track, objects, robotCfg)`:

```jsonc
// one entry per manip event, in event order:
{
  "eventIndex": 12,          // index into track.events of the manip op
  "t": 12.7,
  "inHand": "o5" | null,     // object id staged at grabber
  "magazine": [[ "o1","o2","o3" ], [ "o4" ], [], [ "o6","o7" ]],  // lane→[ids], front last
  "rearBox": "o9" | null,    // Cargo id held (capacity 1)
  "placed": { "o8": { "x_cm": 55, "y_cm": 30 } }  // objects moved to a NEW field spot (dropped/dumped)
}
```

- Objects not in `inHand`/`magazine`/`rearBox` and not in `placed` render at their
  authored `x_cm,y_cm`; objects in a store are omitted from the field; objects in
  `placed` render at the placed position.
- Scrub: `findEventIndex(t)` → bisect the scene array → render that snapshot. Exact,
  since object state changes only at manip events.

## Contracts & Interfaces

Single source of truth for every cross-task seam.

### Shared new modules (owned once, consumed by both pages / multiple tasks)

- `tools/path_planner/ctps/layout.py` — owner: T2. `load(source) -> Layout`,
  `Layout.to_dict()`, strict `validate()`. Consumers: T3 (editor emitter), T4
  (player embed), tests.
- `tools/path_planner/ctps/_page_emit.py` — owner: T1. Extracted shared emit
  helpers `read_asset(path)`, `assert_self_contained(sources)`, `script(inner)`,
  `FORBIDDEN_SUBSTRINGS`. Consumers: `player_html.py` (T1 rewires to import these,
  behavior-identical) and `layout_editor_html.py` (T7).
- `assets/objects_draw.js` — owner: T5. Pure canvas draws, transform-agnostic
  (takes a `project(view,x,y)` fn): `CTPS_OBJ.drawObject(ctx, view, obj, project,
  opts)`, `CTPS_OBJ.drawZones(ctx, view, pose, robotCfg, project)`. Consumers: T5
  (player object/zone layers) and T8 (editor canvas). Exposed on a small global
  (`window.CTPS_OBJ`) so both pages' non-CTPS editor namespace can call it.

### Player-side seams (ADR-053 registries — new module `assets/cargo.js`, added to `player_html._ASSET_JS` before `boot.js`)

- Store (extends `store.js`, owner T6): new keys + actions, each `notify([...])`:
  - `store.objects` (array, from `const OBJECTS`), `store.robotCfg` (robot section),
    `store.scene` (current resolved snapshot), `store.layers.objects`/`layers.zones`
    (booleans), `store.activeCargoTab` (`"magazine"|"inventory"`).
  - `store.loadLayout(layoutDict)`, `store.setCargoTab(name)`,
    `store.toggleLayer("objects"|"zones")` (reuse existing `setLayer`).
  - A controller subscribes to `["t"]`, bisects the scene timeline, and only when
    the scene index CHANGES calls an internal `setScene(snapshot)` → `notify(["scene"])`
    (so the static objects layer repaints on cargo events, not every frame).
- Layers (`CTPS.layers.register`, owner T5): descriptor shape is the existing
  `{ id, canvasId, visible(state), draw(ctx,view,state), invalidateOn:[keys] }`.
  - `{ id:"objects", canvasId:"static", visible: s=>s.layers.objects,
      draw:<objects_draw>, invalidateOn:["scene","layers"] }`
  - `{ id:"zones", canvasId:"actors", visible: s=>s.layers.zones,
      draw:<zones from pose>, invalidateOn:["t","scene","layers"] }`
- Render/inverse: the player reuses `renderer.js`'s `_project` for object world→px
  (objects layer draws in the renderer's `draw(ctx,view,state)` callback, which
  receives `view`). No pixel→world inverse is needed in the player (no dragging).
- Cargo panel + HUD + timeline lane (owner T6): replaces the holding shell in
  `panels.js` (`renderHoldingShell` #57 seam) with the two-tab cargo panel;
  `timeline.js` `buildHoldingLane` (the reserved `[]` #57 seam) returns held-period
  spans from the scene timeline; the `#ctps-hud-holding` line updates from
  `store.scene`. `_BODY_MARKUP` in `player_html.py` swaps the `.ctps-panel--holding`
  block for the cargo-panel markup (keeps `#ctps-command`/`#ctps-log` → TC-STRUCT
  unaffected).
- Emitter (`player_html.render`, owner T4): new kwarg `objects: dict|None=None`.
  When truthy, emits `const OBJECTS = <json>` (own `<script>`, `</` escaped) and
  the layout validates through `layout.py`. `PLAYER_META` gets a new key ONLY when
  objects present (gate like `render_mode`), else byte-identical. `_ASSET_JS` gains
  `cargo.js` + `objects_draw.js` (before `boot.js`).

### Op contract (3-place; loader leg already open)

- Recorder mock (`WRO2026/test/ctps_mocks/Manipulator.h`, owner T10): add
  `boxOpen()` / `boxClose()` → `ctps::recordManip("boxOpen"|"boxClose", 0)`. Verbs
  `grab`/`release`/`dump`/`sortTo`/`reset` already record — unchanged.
- Production facade (`WRO2026/src/manip/Manipulator.h/.cpp`, owner T10): add
  `boxOpen()` / `boxClose()` verbs (thin `ManipAction` scaffolds, uncalibrated,
  matching ADR-032) so the ADR-051 conformance guard (mock ⊆ real) holds.
- Loader (`trace_loader.py`): **no change** — `manip` is a known op; `manip.method`
  is not whitelisted, so all six verbs pass through.
- Builder (`track_builder.py`, owner T9): add `method`/`arg` to the manip event.
- Player method → cargo action mapping (owner T5, the resolver): `grab`→pick,
  `sortTo(n)`→file lane n, `dump`→eject fronts, `boxClose`→capture, `boxOpen`→drop,
  `release`/`reset`→log only (no cargo change).

### Robot-local frame (used by editor Robot tab + player resolver)

Origin = robot center; `+x` = heading-forward; `+y` = robot-left. Zone/point world
position at time `t`: `world = pose.xy + Rot(pose.heading)·local.xy`. The grab zone
sits at `+y` (left). Defined once here.

### Naming (must match across tasks)

- Trace op values: `grab`, `release`, `dump`, `sortTo`, `reset`, `boxOpen`,
  `boxClose` (exact strings, recorder ↔ resolver).
- Object types: `"tile"`, `"cargo"`, `"fixed"`. Shapes: `"rect"`, `"circle"`.
- Cargo tabs: `"magazine"`, `"inventory"`. Layer ids: `"objects"`, `"zones"`.
- Layout keys exactly as in the Data Model block.

### File ownership

| File | Owner task | Consumers |
|------|-----------|-----------|
| `ctps/_page_emit.py` | T1 | player_html.py, layout_editor_html.py |
| `ctps/layout.py` | T2 | T3, T4, tests |
| `ctps/config.py` (append magazine/rearbox consts) | T2 | T5, T6 |
| `ctps/track_builder.py` (manip method/arg) | T9 | T5 |
| `ctps/player_html.py` (objects arg, `_ASSET_JS`, `_BODY_MARKUP`) | T4 | T5, T6 |
| `assets/objects_draw.js` | T5 | T5, T8 |
| `assets/cargo.js` (new player module) | T5→T6 | — (T5 layers/resolver, T6 panel; split by function within the file, T6 blocked by T5) |
| `assets/store.js` (new keys/actions) | T6 | T5, T6 |
| `assets/panels.js`, `assets/timeline.js`, `assets/chrome.js` (chips) | T6 | — |
| `ctps/layout_editor_html.py` + `assets/editor_*.js` + `editor.css` | T7, T8 | — |
| `WRO2026/src/manip/Manipulator.{h,cpp}`, `test/ctps_mocks/Manipulator.h` | T10 | — |

> `cargo.js` is touched by both T5 (layers + resolver) and T6 (panel wiring); T6 is
> **Blocked By T5** so they never edit it in parallel.

## Error Handling

- **Malformed `layout.json` (upload)** — JS `loadLayout` validates tolerantly:
  skip a bad object with `console.warn`, keep the rest; a structurally invalid file
  (not JSON / missing `objects`) shows an inline "couldn't read layout" message and
  leaves the current scene intact.
- **Malformed layout at emit time** — `layout.py.validate()` raises loud
  (validator-style) so a bad authored file fails the render, not the field-day page.
- **`grab` with no Tile in zone** — logged no-op; in-hand unchanged.
- **`sortTo(n)` full lane / no in-hand / bad `n`** — logged no-op, cargo unchanged.
- **`boxClose` with box full / no Cargo in zone** — logged no-op.
- **`dump` on empty magazine** — logged no-op (no ejection).
- **Missing/partial `robot` config** — objects still render; grab/dump/box become
  logged no-ops (no zones) rather than throwing; a one-line HUD note flags "no robot
  zones."
- **Layout present but trace has no manip events** — objects render statically; the
  cargo panel shows empty stores.
- **Self-containment / inline-style violations** — caught at emit time by the lint
  and by TC-NOINLINE; a failing asset blocks the render.

## Testing Strategy

**Levels:** Unit (Python: schema, resolver-adjacent builder, emitter), Integration
(end-to-end render), Host C++ (facade+mock conformance), JS (text-level: asset
inlining, no-inline-style, self-containment). The JS scene resolver has no
pytest-level unit harness (JS is tested as text), so it is a single pure function
exercised by a deterministic synthetic fixture whose expected transitions are
asserted via the Python end-to-end text checks + manual verification; keep it
inspectable.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `layout.py` loads a valid layout | Unit | round-trips via `to_dict`; fields typed |
| TC2 | `layout.py` rejects bad object (missing type / bad shape / out-of-range) | Unit | raises with a specific message |
| TC3 | `track_builder` emits `method`/`arg` on manip events | Unit | manip event dict has both keys; `label` unchanged |
| TC4 | manip-free trace track is byte-identical to pre-change | Unit | golden unchanged (no new keys) |
| TC5 | `player_html.render(objects=None)` — byte-identical, no `const OBJECTS`, TC9 meta literal intact | Unit | existing TC9 still passes |
| TC6 | `player_html.render(objects=<layout>)` embeds `const OBJECTS`, adds the gated meta key, inlines `cargo.js`+`objects_draw.js` | Unit | OBJECTS round-trips; assets present |
| TC7 | `layout_editor_html.render()` is self-contained + offline | Unit | passes lint; no forbidden substrings |
| TC8 | every editor asset is inlined (TC-ASSETS analogue) | Unit | each `editor_*.js`/`editor.css` present verbatim |
| TC9 | no inline `style=` / `el.style.prop=` in any new asset (player + editor) | Unit | TC-NOINLINE analogue passes |
| TC10 | End-to-end: synthetic trace (grab→sortTo→dump→boxClose→boxOpen) + layout renders + inlines text | Integration | page emits; scene text present |
| TC11 | Resolver scene fixture: known trace+layout → expected snapshot per manip event (front-to-back fill, dump ejects fronts, capacity limits) | Integration | asserted transitions match |
| TC12 | `boxOpen`/`boxClose` on production facade ⊆ mock (conformance) + `ctps_dump` compiles | Host C++ | `test_ctps_mock_superset` passes; build clean |
| TC13 | `pivots` + `objects` co-present render (self-containment with both overlays) | Integration | no lint trip; both inline |

**Test data:** hand-written Data-Model-A fixtures under
`tools/path_planner/tests/fixtures/` — `ctps_cargo_layout.json` (a layout),
`ctps_cargo_trace.json` (grab/sortTo/dump/box sequence). Reuse the existing
emitter-test regex extractors for `const OBJECTS`.
**Run command:** `uv run --extra dev pytest tools/path_planner/tests/` and
`ctest --test-dir build/test --output-on-failure`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Extract shared emit scaffold | — | med | `ctps/_page_emit.py`, `ctps/player_html.py` | Move `_read_asset`/`_assert_self_contained`/`_script`/`_FORBIDDEN_SUBSTRINGS` into `_page_emit.py`; rewire `player_html.py` to import them, behavior-identical. Satisfies the shared-scaffold contract. Run full `test_ctps_player_html.py` — must stay green. |
| T2 | `layout.json` schema + validation | — | med | `ctps/layout.py`, `ctps/config.py`, `tests/test_ctps_layout.py` | Implement `Layout`/`load`/`validate`/`to_dict` per the Data Model; add magazine (4×3) + rear-box (1) constants to `config.py`. Satisfies AC4 (schema), TC1/TC2. |
| T3 | Cargo fixtures | T2 | low | `tests/fixtures/ctps_cargo_layout.json`, `tests/fixtures/ctps_cargo_trace.json` | Author a valid layout (tiles/cargo/fixed + robot zones) and a Data-Model-A trace running grab→sortTo(0..)→dump→boxClose→boxOpen. Feeds TC10/TC11. |
| T4 | Player emitter: `objects=` embed | T1, T2 | med | `ctps/player_html.py`, `tests/test_ctps_player_html.py` | Add `objects` kwarg → `const OBJECTS` (own script, `</` escaped), validate via `layout.py`; gate a new `PLAYER_META` key on presence; add `cargo.js`+`objects_draw.js` to `_ASSET_JS`; swap the holding markup for the cargo-panel markup keeping `#ctps-command`/`#ctps-log`. Satisfies AC5/AC11, TC5/TC6. |
| T5 | Player object layer + zones + scene resolver | T4 | high | `assets/objects_draw.js`, `assets/cargo.js` (resolver+layers), `assets/store.js` | Implement `objects_draw.js` (draw object/zones, transform-agnostic); `cargo_resolve()` (resolve-once event-indexed scene timeline per the Data Model + method→action map); register `objects`/`zones` layers; add store keys (`objects`,`robotCfg`,`scene`,`layers.objects/zones`) + the t→scene controller. Satisfies AC5/AC6/AC7/AC8/AC10, TC11. |
| T6 | Cargo panel + HUD + holding lane | T5 | med | `assets/cargo.js` (panel), `assets/panels.js`, `assets/timeline.js`, `assets/chrome.js`, `ctps/player_html.py` (markup) | Replace `renderHoldingShell` with the two-tab cargo panel (Magazine vertical lanes + Inventory rear box) reading `store.scene`; wire `store.setCargoTab`; fill `buildHoldingLane` from the scene; update `#ctps-hud-holding`; add OBJECTS/ZONES chips in `chrome.js`. Satisfies AC9, matches the UI sidecar. |
| T7 | Editor page emitter | T1, T2 | high | `ctps/layout_editor_html.py`, `tests/test_ctps_layout_editor.py` | New emitter (uses `_page_emit.py`): inline `tokens.css`+`editor.css`, the editor JS assets, and `const FIELD` (field backdrop from `field_data`) + optional `const MAP_IMAGE`; own asset tuple + own lint. Satisfies AC1, TC7/TC8. |
| T8 | Editor interactions (two tabs, draw, props, save/load) | T5, T7 | high | `assets/editor_core.js`, `assets/editor_tools.js`, `assets/editor_io.js`, `assets/editor_boot.js`, `assets/editor.css` | Duplicated transform + pixel→world inverse; field backdrop draw; Field/Robot tabs; Rect/Circle rubber-band + select/drag/delete; Properties panel (name/type/color/precise dims); Blob save + FileReader load of `layout.json`. Reuses `objects_draw.js` from T5. Satisfies AC2/AC3/AC4/AC13. |
| T9 | `track_builder` structured manip keys | — | low | `ctps/track_builder.py`, `tests/test_ctps_track_builder.py` | Add `method`/`arg` to the manip event; keep `label`. Update `test_tc2_task_manip_events_in_order` for the additive keys; ensure manip-free golden unchanged. Satisfies AC12 (builder leg), TC3/TC4. |
| T10 | Firmware `boxOpen`/`boxClose` + mock | — | med | `WRO2026/src/manip/Manipulator.h/.cpp`, `WRO2026/test/ctps_mocks/Manipulator.h` | Add `boxOpen()`/`boxClose()` verbs to the production facade (thin scaffolds per ADR-032) AND the recording mock (`recordManip`). Run host ctest (`test_ctps_mock_superset`) + confirm `ctps_dump` compiles. Satisfies AC12 (C++ leg), TC12. |
| T11 | ADR-056 + DECISIONS index | T5, T7 | low | `docs/decisions/056-ctps-object-cargo.md`, `DECISIONS.md` | Write ADR-056 (object/cargo model, two-page split, resolve-once scene timeline, additive emission, boxOpen/boxClose contract); add the index row + bump the status table. |

Parallelizable: T1, T2, T9, T10 have no blockers (run first). T3 follows T2. T4
follows T1+T2. T5 follows T4. T6 follows T5 (same file `cargo.js`). T7 follows
T1+T2. T8 follows T5 (needs `objects_draw.js`) + T7. T11 follows T5+T7.

## Notes for Implementer

- **Never touch `player.js`'s pinned functions** (`findKeyframeIndex`/`poseAt`/
  `findEventIndex`/`wrapDeltaDeg`) or reindent them — TC12 (the emitter's, distinct
  from this plan's TC12) watches that file. Consume them via `CTPS.engine`.
- **Do not refactor `renderer.js`.** Duplicate the ~40-line transform in the editor;
  the player reuses `_project` via the `view` passed into a registered layer's
  `draw`.
- **HiDPI:** draw the objects/zones layers by clear+redraw each repaint (the layer
  system already does per-layer redraws); never `getImageData`/`putImageData`
  snapshot in CSS px (device-px ghost trap).
- **Per-frame styling** only via `el.style.setProperty('--foo', …)` against a
  `tokens.css`-declared custom property; object fill = authored hex set the same
  way. A static inline `style=` or `el.style.prop =` fails TC-NOINLINE.
- **Additive-or-nothing emission:** with `objects=None` the emitted bytes must not
  change at all (no `const OBJECTS`, meta literal intact) — verify against TC9
  before/after.
- **Loader needs no change** — resist adding `boxOpen`/`boxClose` to any whitelist;
  `manip.method` is unfiltered by design.
- **`cargo.js` is shared by T5/T6** — T6 is blocked by T5; within the file keep the
  resolver+layers (T5) and the panel wiring (T6) in clearly separated sections.
- **Rollback:** the feature is fully gated on a layout being present; reverting is
  removing `cargo.js`/`objects_draw.js` from `_ASSET_JS` and the `objects` kwarg —
  layout-free renders are unaffected either way.
- Branch off `main` as `feat/ctps-object-cargo`; regenerate nothing in
  `routes.h/.cpp` (no field-data change). Read `CONTRIBUTING.md` before the PR;
  `Closes #57`.
