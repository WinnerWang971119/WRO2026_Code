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
  the mock. Net new C++ = `boxOpen`/`boxClose` only. They must land on **both** the
  production `Manipulator` facade (so a task body calling them compiles on the robot
  via arduino-cli) **and** the recording mock (so `ctps_dump`, which compiles the
  whole `Tasks.cpp`, builds when a task body calls them). This is **not** ADR-051:
  that guard covers the `Drive`/`LineFollower` fluent-builder surface, not
  `Manipulator` verbs, and the mock already carries `setLift`/`setGripper`/`setArm`
  that production lacks — the two are independent build requirements.
- **`track_builder` adds structured `method`/`arg` to the `manip` event** — today
  it flattens to a display `label`; the player must never parse labels. Additive:
  manip-free traces stay byte-identical.
- **Conditional emission** — only `const OBJECTS` is conditional (emitted when a
  layout is present; the `Track.to_dict` pivots-**omission** precedent). `PLAYER_META`
  is NOT changed — the player detects a layout via `typeof OBJECTS`. The cargo assets +
  panel markup ship on every page (the upload leg needs them), so a layout-free page is
  not byte-identical; the invariant is: no `const OBJECTS`, TRACK round-trips, and the
  `PLAYER_META` literal is unchanged (suite TC9 green).
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
- [ ] AC8 — `boxClose` captures the nearest Cargo whose **center is inside** the
  world-projected rear-box zone (capacity 1; extra in-zone Cargo ignored + logged);
  `boxOpen` drops the held Cargo back on the field at the rear-box zone. (Both grab
  and boxClose use the same center-in-zone predicate.)
- [ ] AC9 — The right-rail cargo panel shows two tabs (Magazine 4×3 vertical lanes,
  Inventory rear box) with live counts; the HUD holding line and the Holding
  timeline lane reflect the same state.
- [ ] AC10 — Scrubbing to any `t` (forward or backward, and after a jump) shows the
  exact cargo + object placement for that instant; no stale/incremental drift.
- [ ] AC11 — A layout-free render (`objects=None`) emits **no `const OBJECTS`**, the
  TRACK JSON round-trips unchanged, and `PLAYER_META` is unchanged (suite TC9 stays
  green). Note: the new inert assets (`cargo.js`/`objects_draw.js`) and the
  cargo-panel markup DO change the page bytes for every render — the guarantee is
  TRACK + `PLAYER_META`, not a byte-identical page.
- [ ] AC12 — `boxOpen`/`boxClose` exist on the production `Manipulator` facade (compiles
  under `arduino-cli`) AND the ctps recording mock (`ctps_dump` compiles); the loader
  accepts them with no whitelist change; `manip` track events carry structured
  `method`/`arg`. (No ADR-051 conformance claim — that guard is Drive/LineFollower only.)
- [ ] AC13 — No JS asset uses a static inline `style=` or `el.style.prop =`
  (TC-NOINLINE), and no asset trips the self-containment lint.

> **Verification split.** ACs 1, 4 (Python leg), 5 (embed), 11, 12, 13 are covered
> by automated Python/host-C++ tests. The **runtime-behavior** ACs — 2, 3, 4 (draw +
> Blob/FileReader legs), 6, 7, 8, 9, 10 — run only in the browser (the pytest harness
> tests JS as text, and mirroring the resolver in Python is banned by ADR-033), so
> they are **manual-verification** against the open-the-page checklist in Testing
> Strategy. The scene resolver is kept a single pure function so it *could* later gain
> a node smoke test (see Testing Strategy).

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
- Magazine geometry and rear-box capacity are fixed player constants in
  `ctps/config.py` — `MAGAZINE_LANES = 4`, `MAGAZINE_DEPTH = 3`, `REAR_BOX_CAP = 1`
  — mirrored as JS constants in `cargo.js`, NOT in `layout.json`.
- Robot-local coordinate frame is defined once here and in Contracts; the editor
  Robot tab and the player resolver both use it.
- The editor Properties panel edits every schema field: `w_cm`/`h_cm` for a rect,
  `radius_cm` for a circle, and `heading_deg` via a rotation input (rect only;
  hidden for circles). No schema field is unauthorable.
- The magazine cell's short label is the first two non-space characters of the
  object `name` (uppercased); the inventory hold card shows the full `name`.

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
  "magazine": [[ "o1","o2","o3" ], [ "o4" ], [], [ "o6","o7" ]],  // lane→[ids], FIFO: index 0 = oldest = front/dump side
  "rearBox": "o9" | null,    // Cargo id held (capacity 1)
  "placed": { "o8": { "x_cm": 55, "y_cm": 30 } }  // objects moved to a NEW field spot (dropped/dumped)
}
```

- Objects not in `inHand`/`magazine`/`rearBox` and not in `placed` render at their
  authored `x_cm,y_cm`; objects in a store are omitted from the field; objects in
  `placed` render at the placed position.
- Scrub: `findEventIndex(t)` → bisect the scene array → render that snapshot. Exact,
  since object state changes only at manip events.
- **Magazine ordering is FIFO** (decided during execution, supersedes any earlier
  "front last" wording here and in AC7): each lane array is ordered oldest-first, so
  index 0 is the front / bottom / dump side. `sortTo(n)` appends (the newest tile
  lands on top); `dump` ejects index 0 (the oldest) first. This matches a physical
  gravity lane loaded from the top and emptied from the bottom. See ADR-056.

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

### Player-side seams (ADR-053 registries — new `assets/objects_draw.js` then `assets/cargo.js`, both added to `player_html._ASSET_JS` in that order, before `boot.js`)

- **Asset load order (pinned):** `objects_draw.js` **before** `cargo.js` — `cargo.js`'s
  layer descriptors reference `CTPS_OBJ.drawObject`/`drawZones` at IIFE time.
- **Store (extends `store.js`, owner T5 — T5 owns EVERY store.js change here; T6 only
  consumes):** new keys + actions, each `notify([...])`:
  - `store.objects` (from `const OBJECTS`), `store.robotCfg` (robot section),
    `store.scene` (current resolved snapshot + index), `store.layers.objects` (default
    **on**) / `store.layers.zones` (default **off**), `store.activeCargoTab`
    (`"magazine"` default | `"inventory"`).
  - `store.loadLayout(layoutDict)` → set `objects`/`robotCfg`, resolve the scene
    timeline, set the initial `scene`, `notify(["objects","robotCfg","scene"])`.
  - `store.setCargoTab(name)` → `notify(["activeCargoTab"])`.
  - Layer visibility reuses the EXISTING `store.toggleLayer`/`setLayer` (store.js:122-129)
    — not redefined; `layers.objects`/`layers.zones` are just new keys in `store.layers`.
  - A controller subscribes to `["t"]`, bisects the scene timeline, and only when the
    scene **index changes** calls an internal `setScene(snapshot)` → `notify(["scene"])`,
    so the panel/HUD/holding-lane update on cargo events, not every frame.
- **Layers (`CTPS.layers.register`, owner T5):** existing descriptor shape
  `{ id, canvasId, visible(state), draw(ctx,view,state), invalidateOn:[keys] }`. Both new
  layers register on the **`trail`** canvas — it sits BELOW the `actors` (robot) canvas in
  the stack, so objects/zones draw UNDER the robot without touching `renderer.js`'s
  registration order (a module loading after renderer.js cannot register before the
  `actors` layer):
  - `{ id:"objects", canvasId:"trail", visible: s=>s.layers.objects, draw:<CTPS_OBJ.drawObject over scene>, invalidateOn:["scene","layers"] }`
  - `{ id:"zones", canvasId:"trail", visible: s=>s.layers.zones, draw:<CTPS_OBJ.drawZones from pose(t)>, invalidateOn:["t","scene","layers"] }`
  Both draw by clear+redraw each trail repaint (cheap for a small vector scene; dodges the
  device-px `getImageData` ghost trap). The trail canvas already repaints on `["t","layers"]`.
- **Projection (this is the load-bearing seam — do it here, once):** `objects_draw.js`
  OWNS the world→pixel formula `project(view, x, y)` (transform-agnostic — applied to
  whatever `view` it is handed). The player's registered layer `draw(ctx, view, state)`
  receives the renderer's `view` and passes it straight to `CTPS_OBJ`; the editor computes
  its OWN `view` (a duplicated `_computeView`, ~40 lines) and passes that. `renderer.js` is
  NOT changed and gains no export — `CTPS.renderer` stays `{resize, hitTest}`. The
  pixel→world inverse lives ONLY in the editor (T8); the player never drags.
- **Cargo panel + chips + upload markup (owner T4, in `player_html._BODY_MARKUP`):** swaps
  the `.ctps-panel--holding` block for the two-tab cargo-panel markup, adds `OBJECTS`/`ZONES`
  chip buttons to the existing `.ctps-layer-chips` row, and adds a `<input type=file>`+button
  for layout upload — keeping `#ctps-command`/`#ctps-log` so TC-STRUCT is unaffected.
  `chrome.js` needs **no** change: it wires every `.ctps-layer-chips .ctps-chip` generically
  via `data-layer` (chrome.js:285, 324-340), so the new chips work by markup alone.
- **DOM contract for that markup (owner T4; queried by T6):** `#ctps-cargo` (panel),
  `.ctps-cargo-tab[data-cargo-tab="magazine"|"inventory"]` (tabs), `#ctps-cargo-count-mag`
  / `#ctps-cargo-count-inv` (badges), `#ctps-mag-grid` (magazine lanes), `#ctps-inv-list`
  (rear-box card), `#ctps-layout-file` (`<input type=file>`), chips
  `.ctps-chip[data-layer="objects"|"zones"]`.
- **Cargo panel + upload + HUD + holding-lane BEHAVIOR (owner T6):**
  - `panels.js` `renderHoldingShell` (#57 seam) → the two-tab cargo panel reading
    `store.scene` + `store.activeCargoTab`; tab clicks call `store.setCargoTab`.
  - `timeline.js` `buildHoldingLane` (the reserved `[]` #57 seam) → held-period spans, PLUS
    a **rebuild path**: the dock builds lanes once at boot (timeline.js:459-489), so T6 adds
    a lane rebuild on `notify(["scene"])`/layout-load, else the holding lane is permanently
    empty for embedded and uploaded layouts alike.
  - `#ctps-hud-holding` updates from `store.scene`.
  - `#ctps-layout-file`'s `change` handler reads the file (FileReader) and calls
    `store.loadLayout(JSON.parse(text))` — the in-browser upload leg of AC5.
- **Emitter (`player_html.render`, owner T4):** new kwarg `objects: dict|None=None`. When
  truthy, validates via `layout.py` and emits `const OBJECTS = <json>` (own `<script>`,
  `</` escaped). **`PLAYER_META` is NOT changed** — the player detects a layout via
  `typeof OBJECTS !== "undefined"`, so suite TC9's exact-literal assertion is untouched.
  `const OBJECTS` is emitted only when objects are present (the `Track.to_dict` **pivots
  omission** precedent — NOT the always-present `render_mode` key). `_ASSET_JS` gains
  `objects_draw.js` then `cargo.js` (both before `boot.js`).

### Op contract (loader leg already open — NOT ADR-051)

- Loader (`trace_loader.py`): **no change** — `manip` is a known op and `manip.method`
  is not whitelisted, so all six verbs pass through untouched.
- Recorder mock (`WRO2026/test/ctps_mocks/Manipulator.h`, owner T10): add `boxOpen()` /
  `boxClose()` → `ctps::recordManip("boxOpen"|"boxClose", 0)`; the other five verbs
  already record. Needed so `ctps_dump` (which compiles ALL of `Tasks.cpp`) builds when
  a task body calls the new verbs.
- Production facade (`WRO2026/src/manip/Manipulator.{h,cpp}`, owner T10): add `boxOpen()`
  / `boxClose()` verbs — thin `ManipAction` scaffolds with an empty actuator mask
  (uncalibrated, matching ADR-032's "verbs ship as scaffolds"; the rear-box actuator is
  TBD at bring-up, so they fire nothing yet). Needed so a task body calling them compiles
  on the robot (arduino-cli). Plain build requirement — **not** ADR-051 (that guard
  covers `Drive`/`LineFollower` builders, not `Manipulator` verbs).
- Builder (`track_builder.py`, owner T9): add `method`/`arg` to the manip event (additive).
- Player method → cargo action (owner T5, the resolver): `grab`→pick nearest Tile whose
  center is in the grab zone into the 1-tile in-hand slot; `sortTo(n)`→file the in-hand
  tile into lane n; `dump`→eject the front tile of each non-empty lane; `boxClose`→capture
  the nearest Cargo whose center is in the box zone; `boxOpen`→drop the held Cargo;
  `release`→drop the in-hand tile back at the grab zone (evicts it, so grab/release leaves
  nothing stranded); `reset`→log only, no cargo change.

### Robot-local frame (used by editor Robot tab + player resolver)

Origin = robot center; `+x` = heading-forward; `+y` = robot-left. Zone/point world
position at time `t`: `world = pose.xy + Rot(pose.heading)·local.xy`. The grab zone
sits at `+y` (left). Defined once here. The editor Robot tab draws its default robot
box at `2·robot_radius_cm` per side (the player's `_robotSideCm`, from `field_data`).

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
| `assets/objects_draw.js` | T5 | T5 (player layers), T8 (editor) |
| `assets/cargo.js` (new player module) | T5→T6 | — (T5 = resolver+layers+store-wiring; T6 = panel+upload+lane behavior, in a separate section of the file, Blocked By T5) |
| `assets/store.js` (new keys/actions) | T5 | T6 |
| `assets/panels.js`, `assets/timeline.js` (the #57 seams only) | T6 | — |
| `ctps/layout_editor_html.py` + `assets/editor_*.js` + `editor.css` | T7, T8 | — |
| `WRO2026/src/manip/Manipulator.{h,cpp}`, `test/ctps_mocks/Manipulator.h` | T10 | — |

> `cargo.js` is touched by both T5 (resolver + layers + store wiring) and T6 (panel +
> upload + lane behavior); T6 is **Blocked By T5** so they never edit it in parallel, and
> each keeps to a clearly-labelled section of the file. `chrome.js` is NOT edited (the new
> chips are generic `data-layer` buttons). T6's edits to `panels.js`/`timeline.js` are
> confined to the reserved #57 seams whose docstrings already anticipate them (the
> ADR-053-sanctioned exception to "don't edit another feature's file").

## Error Handling

- **Malformed `layout.json` (upload)** — JS `loadLayout` validates tolerantly:
  skip a bad object with `console.warn`, keep the rest; a structurally invalid file
  (not JSON / missing `objects`) shows an inline "couldn't read layout" message and
  leaves the current scene intact.
- **Malformed layout at emit time** — `layout.py.validate()` raises loud
  (validator-style) so a bad authored file fails the render, not the field-day page.
- **`grab` with no Tile in zone** — logged no-op; in-hand unchanged.
- **`grab` with the in-hand slot already occupied** — logged no-op (only one tile in
  hand at a time; `sortTo` or `release` must clear it first).
- **`release` with an empty in-hand slot** — logged no-op (nothing to drop).
- **`sortTo(n)` full lane / no in-hand / bad `n`** — logged no-op, cargo unchanged.
- **`boxClose` with box full / no Cargo center in zone** — logged no-op.
- **`dump` on empty magazine** — logged no-op (no ejection).
- **Missing/partial `robot` config** — objects still render; grab/dump/box become
  logged no-ops (no zones) rather than throwing; a one-line HUD note flags "no robot
  zones."
- **Layout present but trace has no manip events** — objects render statically; the
  cargo panel shows empty stores.
- **Self-containment / inline-style violations** — caught at emit time by the lint
  and by TC-NOINLINE; a failing asset blocks the render.

## Testing Strategy

**Levels:** Unit (Python: schema, builder, emitter), Integration (Python: end-to-end
render), Host C++ / compile (the two new verbs on both surfaces), JS (text-level: asset
inlining, no-inline-style, self-containment), and a **manual browser checklist** for
every runtime behavior. The pytest harness tests JS **as text**, and re-implementing the
resolver in Python just to test it would fork render logic into a second drift-prone
copy (the cargo resolver is a player-render concern, not a firmware-FSM mirror like the
sanctioned `sim/`). So resolver / cargo / scrub behavior is NOT pytest-coverable. `cargo_resolve` is kept a single pure DOM-free function so it *could*
gain a `node` smoke test later (a conscious future decision, not v1). Plan-local ids are
prefixed **`P-TC`** to avoid colliding with the emitter suite's own `TC*` (whose `TC9` is
the exact-`PLAYER_META`-literal assert this plan must keep green).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| P-TC1 | `layout.py` loads a valid layout | Unit | round-trips via `to_dict`; fields typed |
| P-TC2 | `layout.py` rejects a bad object (missing type / bad shape / out-of-range) | Unit | raises with a specific message |
| P-TC3 | `track_builder` emits `method`/`arg` on manip events | Unit | manip event dict has both keys; `label` unchanged |
| P-TC4 | manip-free trace track unchanged | Unit | golden byte-identical (no new keys) |
| P-TC5 | `render(objects=None)` — no `const OBJECTS`; TRACK round-trip + `PLAYER_META` literal unchanged | Unit | suite `TC9` still passes; no `OBJECTS` in output |
| P-TC6 | `render(objects=<layout>)` embeds `const OBJECTS` (round-trips), inlines `objects_draw.js`+`cargo.js` in order; `PLAYER_META` STILL unchanged | Unit | `OBJECTS` extractable; assets present; meta literal identical |
| P-TC7 | `layout_editor_html.render()` self-contained + offline | Unit | passes lint; no forbidden substrings |
| P-TC8 | every editor asset inlined (TC-ASSETS analogue) | Unit | each `editor_*.js`/`editor.css` verbatim |
| P-TC9 | no inline `style=` / `el.style.prop=` in any new player+editor asset | Unit | TC-NOINLINE analogue passes |
| P-TC10 | End-to-end: synthetic trace (grab→sortTo→dump→boxClose→boxOpen) + layout renders | Integration | `const OBJECTS` round-trips; the embedded TRACK events carry `method`/`arg` for the fixture sequence; page passes the lint |
| P-TC11 | Resolver scene fixture — documented expected snapshot per manip event (front-to-back fill, dump ejects fronts, capacity, scrub-back exactness) | **Manual** (browser) | expected transitions written in the fixture comment; verified via the checklist below |
| P-TC12 | `boxOpen`/`boxClose` compile on the production facade (`arduino-cli`) and in the mock (`ctps_dump` build) | Host C++ / compile | both builds clean; a task body calling them records a `manip` op |
| P-TC13 | `pivots` + `objects` co-present render | Integration | no lint trip; both inline |

**Manual browser checklist (covers ACs 2, 3, 6, 7, 8, 9, 10 — the runtime legs):** open
the emitted page on `file://` and confirm — objects render by type; OBJECTS/ZONES chips
toggle; `grab` pulls the nearest Tile in the zone off the field into the in-hand slot;
`sortTo(n)` fills lane `n` bottom-up; `dump` drops the front of each lane at its drop
point; `boxClose`/`boxOpen` capture/drop a Cargo at the rear box; the two-tab counts + HUD
+ holding lane track state; scrubbing backward to any `t` restores the exact placement.
Editor: draw a Rect + Circle, set name/type/color/precise dims + rotation, Save →
`layout.json` downloads, Load it back → identical.

**Test data:** hand-written Data-Model-A fixtures under `tools/path_planner/tests/fixtures/`
— `ctps_cargo_layout.json` (a layout with robot zones) and `ctps_cargo_trace.json`
(grab/sortTo/dump/box sequence). Reuse the existing emitter-test regex extractors for
`const OBJECTS`.
**Run command:** `uv run --extra dev pytest tools/path_planner/tests/`,
`ctest --test-dir build/test --output-on-failure`, and
`arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` (P-TC12).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Extract shared emit scaffold | — | med | `ctps/_page_emit.py`, `ctps/player_html.py` | Move `_read_asset`/`_assert_self_contained`/`_script`/`_FORBIDDEN_SUBSTRINGS` into `_page_emit.py`; rewire `player_html.py` to import them, behavior-identical. Satisfies the shared-scaffold contract. Run full `test_ctps_player_html.py` — must stay green. |
| T2 | `layout.json` schema + validation | — | med | `ctps/layout.py`, `ctps/config.py`, `tests/test_ctps_layout.py` | Implement `Layout`/`load`/`validate`/`to_dict` per the Data Model; add magazine (4×3) + rear-box (1) constants to `config.py`. Satisfies AC4 (schema), P-TC1/P-TC2. |
| T3 | Cargo fixtures | T2 | low | `tests/fixtures/ctps_cargo_layout.json`, `tests/fixtures/ctps_cargo_trace.json` | Author a valid layout (tiles/cargo/fixed + robot zones) and a Data-Model-A trace running grab→sortTo(0..)→dump→boxClose→boxOpen. Feeds P-TC10/P-TC11. |
| T4 | Player emitter + markup: `objects=` embed, cargo-panel/chips/upload markup | T1, T2 | med | `ctps/player_html.py`, `tests/test_ctps_player_html.py` | Add `objects` kwarg → `const OBJECTS` (own script, `</` escaped), validate via `layout.py`; add `objects_draw.js` then `cargo.js` to `_ASSET_JS` (before `boot.js`); in `_BODY_MARKUP` swap the `.ctps-panel--holding` block for the two-tab cargo-panel markup (DOM ids per Contracts), add OBJECTS/ZONES `data-layer` chips to `.ctps-layer-chips`, and add the `#ctps-layout-file` upload input — keep `#ctps-command`/`#ctps-log`. **Do NOT touch `PLAYER_META`.** Satisfies AC5(embed)/AC11, P-TC5/P-TC6. |
| T5 | Object layer + zones + scene resolver + store | T4, T9 | high | `assets/objects_draw.js`, `assets/cargo.js` (resolver+layers+store-wiring section), `assets/store.js` | `objects_draw.js` owns `project(view,x,y)` + `drawObject`/`drawZones` on `window.CTPS_OBJ` (transform-agnostic); `cargo_resolve()` = resolve-once event-indexed scene timeline (Data Model + method→action map, reads `method`/`arg`); register `objects`/`zones` layers on the **`trail`** canvas; add ALL store keys/actions (`objects`,`robotCfg`,`scene`,`layers.objects`(on)/`layers.zones`(off),`activeCargoTab`,`loadLayout`,`setCargoTab`) + the t→scene controller. Satisfies AC6/AC7/AC8/AC10 (manual), P-TC11. |
| T6 | Cargo panel + upload + HUD + holding-lane behavior | T5 | med | `assets/cargo.js` (panel/upload/lane section), `assets/panels.js`, `assets/timeline.js` | In `cargo.js`'s panel section: fill the two-tab cargo panel (Magazine vertical lanes + Inventory rear box) from `store.scene`/`store.activeCargoTab`, wire tab clicks → `store.setCargoTab`, wire `#ctps-layout-file` → FileReader → `store.loadLayout`, update `#ctps-hud-holding`. `panels.js`: replace the `renderHoldingShell` #57 seam. `timeline.js`: fill `buildHoldingLane` from the scene AND add a lane rebuild on `notify(["scene"])`/layout-load (the dock builds once at boot). No `chrome.js` change. Satisfies AC9, matches the UI sidecar. |
| T7 | Editor page emitter | T1, T2 | high | `ctps/layout_editor_html.py`, `tests/test_ctps_layout_editor.py` | New emitter (uses `_page_emit.py`): inline `tokens.css`+`editor.css`, the editor JS assets, and `const FIELD` (field backdrop from `field_data`) + optional `const MAP_IMAGE`; own asset tuple + own lint. **Reproduce sidecar Change 3; do not re-design.** Satisfies AC1, P-TC7/P-TC8. |
| T8 | Editor interactions (two tabs, draw, props, save/load) | T5, T7 | high | `assets/editor_core.js`, `assets/editor_tools.js`, `assets/editor_io.js`, `assets/editor_boot.js`, `assets/editor.css` | Duplicated `_computeView` + pixel→world inverse; field backdrop draw; Field/Robot tabs; Rect/Circle rubber-band + select/drag/delete; Properties panel (name/type/color/`w`,`h` or `radius`/`heading` rotation/`x`,`y`); Blob save (`a.href = url` with spaces — the self-containment-lint trap) + FileReader load of `layout.json`. Reuses `CTPS_OBJ` from T5. **Reproduce sidecar Change 3; do not re-design.** Satisfies AC2/AC3/AC4/AC13. |
| T9 | `track_builder` structured manip keys | — | low | `ctps/track_builder.py`, `tests/test_ctps_track_builder.py` | Add `method`/`arg` to the manip event; keep `label`. Update `test_tc2_task_manip_events_in_order` for the additive keys; ensure manip-free golden unchanged. Satisfies AC12 (builder leg), P-TC3/P-TC4. |
| T10 | Firmware `boxOpen`/`boxClose` + mock | — | med | `WRO2026/src/manip/Manipulator.h/.cpp`, `WRO2026/test/ctps_mocks/Manipulator.h` | Add `boxOpen()`/`boxClose()` verbs to the production facade (thin empty-mask scaffolds per ADR-032; rear-box actuator TBD at bring-up) so task bodies compile on-robot, AND to the recording mock (`recordManip`) so `ctps_dump` compiles. Verify by `arduino-cli compile` (production) + the `ctps_dump` build (mock) — **not** `test_ctps_mock_superset` (that guard covers the Drive/LineFollower builders, not Manipulator verbs). Satisfies AC12 (C++ leg), P-TC12. |
| T11 | ADR-056 + DECISIONS index | T5, T7 | low | `docs/decisions/056-ctps-object-cargo.md`, `DECISIONS.md` | Write ADR-056 (object/cargo model, two-page split, resolve-once scene timeline, additive emission, boxOpen/boxClose contract); add the index row + bump the status table. |

Parallelizable: T1, T2, T9, T10 have no blockers (run first). T3 follows T2. T4
follows T1+T2. T5 follows T4+T9 (needs the emitter seam + the `method`/`arg` keys).
T6 follows T5 (same file `cargo.js`). T7 follows T1+T2. T8 follows T5 (needs
`objects_draw.js`) + T7. T11 follows T5+T7.

## Notes for Implementer

- **Never touch `player.js`'s pinned functions** (`findKeyframeIndex`/`poseAt`/
  `findEventIndex`/`wrapDeltaDeg`) or reindent them — the emitter suite's `TC12`
  (`test_tc12_player_js_seek…`) watches that file, distinct from this plan's `P-TC12`.
  Consume them via `CTPS.engine`.
- **Do not refactor `renderer.js`** (no export either). `objects_draw.js` owns a
  transform-agnostic `project(view,x,y)` (a duplicate of the ~40-line formula); the
  player's registered layer `draw(ctx,view,state)` gets `view` and passes it in; the
  editor computes its own `view`. `CTPS.renderer` stays `{resize, hitTest}`.
- **HiDPI:** draw the objects/zones layers by clear+redraw each repaint (the layer
  system already does per-layer redraws); never `getImageData`/`putImageData`
  snapshot in CSS px (device-px ghost trap).
- **Per-frame styling** only via `el.style.setProperty('--foo', …)` against a
  `tokens.css`-declared custom property; object fill = authored hex set the same
  way. A static inline `style=` or `el.style.prop =` fails TC-NOINLINE.
- **Conditional emission (not byte-identical page):** `cargo.js`/`objects_draw.js` and
  the cargo-panel markup ship on EVERY page (the in-browser upload leg needs them), so
  the page is not byte-identical to pre-change. What must stay invariant with
  `objects=None`: no `const OBJECTS`, the TRACK JSON round-trips, and `PLAYER_META`
  equals the pre-change literal (suite TC9) — verify TC9 before/after.
- **Resolver automation:** `cargo_resolve` is pure and DOM-free; the ONLY path to
  automating its correctness later is a `node` smoke test. It is not pytest/ctest-
  coverable (JS-as-text harness), so v1 verifies it by the manual browser checklist.
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
