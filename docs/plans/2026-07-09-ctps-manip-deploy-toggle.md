# CTPS DEPLOY Toggle — Manipulator-Area Deploy Overlay Plan

**Goal:** Add a new `DEPLOY` layer chip to the CTPS player that draws each manipulator area (grab zone, rear-box zone, dump drop-points) only while that actuator is "deployed" at the current playhead, hidden the rest of the time — so scrubbing the timeline shows the manipulator's live reach, not a static always-on overlay.

**Approach:** Player-side only. Add a `deploy` layer to the store, a `DEPLOY` chip in the layer-chips row (chrome.js wires it generically), a filled-accent masked draw in `objects_draw.js`, and a deploy-state resolver + `deploy` canvas layer in `cargo.js` (which already owns the static `zones` layer). Box visibility reuses the store's already-resolved cargo scene (`scene.rearBox`); grab/dump flashes come from the fixed track events, windowed to the manip dwell (precomputed once at ingest). No changes to `track_builder.py`, the Track/trace schema, or the Python emitter data model. The existing static `ZONES` overlay is unchanged.

**Base branch:** `feat/ctps-firmware-motion-profile`. **Work branch:** `feat/ctps-manip-zone-deploy-toggle` (already created).

## Scope

- **In scope:**
  - A new `DEPLOY` chip in `.ctps-layer-chips` (after `ZONES`), default OFF, wired by the existing generic `data-layer` handler in `chrome.js` (no `chrome.js` edit).
  - A new `deploy` boolean in `store.layers`.
  - A new `deploy` canvas layer (trail canvas) that draws only the currently-deployed manipulator areas at the interpolated robot pose.
  - A deploy-state resolver `_deployMaskAt(t, state) -> { grab, box, dump }`.
  - A filled-accent masked draw `CTPS_OBJ.drawDeployedZones(ctx, view, pose, robotCfg, project, mask)`.
  - Python markup + JS-presence test assertions; manual localhost eyeball.
- **Out of scope:**
  - Any change to `track_builder.py`, the Track/trace schema, `layout.py`, or the compiler/emitter data model (the trace already carries structured `manip` `method`/`arg`).
  - Changing the existing static `ZONES` chip/layer behavior (stays an always-on dashed cyan overlay).
  - New servo/position telemetry in the trace (there is none; "deployed" is derived from existing events + resolved scene).
  - A JS unit-test runner (none exists in the repo; logic is verified by markup assertions + eyeball).
  - Editing the offline layout editor (`editor_*.js`) — the deploy overlay is player-only. Note: `objects_draw.js` is a **shared** asset inlined into BOTH the player and the editor HTML (objects_draw.js:4-7), so T3's edits ride into the editor emit too; they are inert there because no editor code calls `drawDeployedZones` (the editor draws its own zones), and the `_drawZoneRect` refactor keeps existing output identical. Editor coverage rides on the full-suite run (TC5) + a one-line editor eyeball (T6).

## Decisions

- **Chip label = `DEPLOY`**; it is a **separate** chip — the existing `ZONES` static overlay is untouched — Distinguishes "show while deployed" from "show all reach zones always."
- **Default OFF; overlap allowed** — When both `ZONES` and `DEPLOY` are on, both overlays draw (static cyan dashed reach zones + the yellow live fill on top). They are independent toggles; no suppression logic.
- **Active visual = filled accent glow (CANVAS OVERLAY only)** — Deployed areas render on the canvas as a translucent `--ctps-accent` (#ffd24a) fill + a solid `--ctps-accent` outline, visually distinct from `ZONES`' dashed `--ctps-hold` (#5bc8e0) cyan outline. The **chip button itself** uses the stock `.ctps-chip` / `.ctps-chip--active` style like every other layer chip — the accent glow is the overlay treatment, not a bespoke chip style.
- **Flash window = `[ev.t, ev.t + MANIP_NOMINAL_S]` (0.5s)** — The grab/dump flashes span exactly the manip event's on-screen dwell (verified: `track_builder.py:1378-1387` emits the event at the pre-dwell clock `t`, then dwells `MANIP_NOMINAL_S`). `MANIP_NOMINAL_S` is mirrored as a JS constant in `cargo.js`, single-sourced from `ctps/config.py` (same pattern as `MAGAZINE_LANES`).
- **Per-actuator "deployed" semantics (NOT one uniform extended/closed rule)** — Each area is shown when it is operationally meaningful:
  - **grab area (`grab_zone`)** ← flashes on **`release`** events (grabber open — reaching out to place a tile). Deliberately NOT `grab`.
  - **box area (`box_zone`)** ← shown while the rear box is **closed** (`boxClose`→`boxOpen`). **Update (2026-07-11):** this is now keyed to the `boxClose`/`boxOpen` **events**, not the cargo model's `scene.rearBox`. The original scene-bisect coupling required a Cargo object to lie in the box zone at `boxClose` time (else `rearBox` stayed null and the overlay never showed); on real mission traces the layout's cargo rarely aligns with the box-close pose, so the box overlay was permanently dark. Keying it to the events makes it reflect the actuator on any trace.
  - **dump area (`drop_points`)** ← flashes on **`dump`** events.
- ~~**Box state via the existing scene bisect** — `_deployMaskAt` reads `state.scene.rearBox`~~ **Superseded (2026-07-11):** box now reads a cached `boxClose`→`boxOpen` event-span list (`windows.box`, `_tInAnyBoxSpan`), start-inclusive / end-exclusive (close shows, open clears), built alongside the grab/dump flash windows in `_buildDeployWindows`. `_deployMaskAt` no longer reads `state.scene`.
- **Grab/dump state from track events, precomputed once** — The `release`/`dump` flash windows `[ev.t, ev.t+0.5s]` are derived from `CTPS.engine.events` and never change after ingest, so they are computed **once** (cached at first use / init) into two sorted window lists, matching `chrome.js`'s `findWorstDrift` compute-once idiom (chrome.js:298-300) rather than re-scanning every trail repaint. `_deployMaskAt(t)` then does a bounded membership check against the cached windows; independent of the loaded layout.
- **All three areas are event-derived (2026-07-11)** — Originally grab/dump came from event flash windows and box from the scene bisect. Box is now an event **span** (`boxClose`→`boxOpen`) built in the same pass, so all three read only `CTPS.engine.events` and none touches the cargo model. A grab/dump flash EXPIRES at `ev.t+0.5s` (a window with no event at its end); a box span is bounded by two real events, so it needs no synthetic end marker.
- **Legacy `label`-only traces draw nothing** — The grab/dump/box detection is `method`-gated (as is the existing cargo resolver, cargo.js:493-498). A legacy trace whose `manip` events carry only `label` and no structured `method` produces no overlay for any area. This is intended, not a gap.
- **Code placement: extend `cargo.js` + `objects_draw.js`** — No new asset file, no `player_html.py` concat-order change (the new draw/resolver ride existing already-inlined assets).
- **Refactor `_drawZoneRect` to reuse a new path-only helper `_pathZoneRectWorld`** — so the rotated-rect corner math has one copy shared by the static `zones` draw and the new deployed draw; `_drawZoneRect`'s stroked output is unchanged.
- **Testing = Python markup + JS-presence assertions + manual eyeball** — Matches how the existing `zones`/`objects` layers are covered (no JS unit runner).

## Acceptance Criteria

- [ ] **AC1** — A `DEPLOY` chip (`id="ctps-layer-deploy"`, `data-layer="deploy"`) renders in `.ctps-layer-chips` after the `ZONES` chip with class **exactly `ctps-chip`** and NOT `ctps-chip--active` (default not-active — unlike the MAP/TRAIL/… chips which ship `--active`). (T2)
- [ ] **AC2** — `store.layers.deploy` defaults `false`; clicking the chip toggles it via the existing generic handler, and the `--active` class mirrors the store. It is independent of `ZONES` (both can be on at once). (T1, T2)
- [ ] **AC3** — With `DEPLOY` on and a layout carrying `robot` zones loaded, scrubbing shows: the **box area** filled across each `boxClose`→`boxOpen` span (close shows, open clears — event-driven, **not** gated on `scene.rearBox`, updated 2026-07-11); the **grab area** filled for ~0.5s at each `release` event; the **dump drop-points** filled for ~0.5s at each `dump` event. Each is hidden at all other times. (T3, T4)
- [ ] **AC4** — Deployed areas render as a translucent `--ctps-accent` fill + solid `--ctps-accent` outline (drop-points as filled accent dots), following the interpolated robot pose — visually distinct from the static `ZONES` dashed cyan overlay. (T3, T4)
- [ ] **AC5** — With no loaded layout, or a layout with no `robot` section (or all mask flags false at `t`), the `DEPLOY` layer draws nothing and raises no error. (T4)
- [ ] **AC6** — The static `ZONES` overlay, every other layer, and a manip-free / layout-free page are visually identical with no behavioral change (the AC10 byte-pin covers only `player.js`/`renderer.js`, both untouched here); the existing `TC-CARGO-STRUCT` markup test still passes. (T3, T4, T5)
- [ ] **AC7** — The full CTPS pytest suite passes, with new `DEPLOY` markup + JS-presence assertions added. (T5)

## UI/UX Specification

> The visual language is **inherited from the existing CTPS design system** (the `.ctps-chip` component + the canvas zone-draw idiom + the fixed `tokens.css` palette), so the Phase 1.5 three-direction design gate was intentionally not run — there is no open design space. The micro visual choices below are pinned as exact values. **No sidecar `.ui.md`** — nothing was generated by the front-end-design skill.

**Chosen base:** existing CTPS "Instrument" dark console. **Grafted in:** none.

### Layout (drawing)

```
Header layer-chips row (top-right overlay, .ctps-layer-chips flex row):

  [ MAP ][ TRAIL ][ PATH ][ NODES ][ OBJECTS ][ ZONES ][ DEPLOY ]
                                                          └ new, class .ctps-chip
                                                            (.ctps-chip--active when store.layers.deploy)

Canvas stage (trail canvas), DEPLOY on, mid-scrub while rear box holds cargo
and a dump just fired:

        ╭───────────╮   box_zone: solid #ffd24a outline,
        │▓▓▓▓▓▓▓▓▓▓▓│   translucent #ffd24a fill  (rear box holding)
        ╰───────────╯
             ▛▀▜   robot marker (existing)
             ▙▄▟
        ● ● ● ●      drop_points: filled #ffd24a dots (dump flash, ~0.5s)

  (grab_zone would appear as the same filled-accent rect only during a release flash)
  Static ZONES (if also on): dashed #5bc8e0 outlines underneath.
```

### Description

The `DEPLOY` chip is the 7th chip in the existing top-right layer row, identical in size/typography to its siblings (`.ctps-chip`), gaining `.ctps-chip--active` when enabled. On the trail canvas, the `deploy` layer draws above the static `objects`/`zones` layers (registration order), so its accent fill sits on top. Each deployed area is a filled-accent shape at the interpolated robot pose: `grab_zone`/`box_zone` as rotated rects, `drop_points` as small filled dots. Nothing is drawn when the layer is off, no layout is loaded, or no area is deployed at the current `t`.

### Color codes

| Token | Hex | Use |
|-------|-----|-----|
| `--ctps-accent` | #ffd24a | DEPLOY overlay fill (translucent) + outline + drop-point dots |
| `--ctps-hold` | #5bc8e0 | (unchanged) static ZONES dashed outline |
| `--ctps-faint` | #565349 | (unchanged) fixed-object outline |

### Type & spacing tokens

Chip: unchanged `.ctps-chip` component (its `--ctps-font-ui` sizing / `--ctps-radius-chip` 3px / `--ctps-raised` fill are inherited as-is). No new CSS.

### States (pinned canvas constants)

- **Chip default / hover / focus / active** — reuse `.ctps-chip` / `.ctps-chip--active` (unchanged component).
- **Overlay empty (no area deployed / no layout / no robot section)** — layer draws nothing.
- **Overlay active** — filled-accent fill at `DEPLOY_FILL_ALPHA = 0.22`, then a solid outline at full alpha with `DEPLOY_STROKE_W = 2` (matches the existing `ZONE_STROKE_W`); dump drop-points as filled accent dots of radius `DEPLOY_DOT_R = 4`. These three named constants are the pinned exact values (no `≈`, no inline literals).

### Responsive

The chip row is an existing flex container (`.ctps-layer-chips`); a 7th chip flows in the same row. Eyeball that the row does not overflow the stage at the narrowest supported width (no test asserts the row width).

## Data Model

No persisted/JSON data model change. One new in-memory store key and one transient render struct:

```js
// store.layers  (store.js) — one new boolean, default false
layers: { map: true, trail: true, path: true, nodes: true,
          objects: true, zones: false, deploy: false }

// deploy mask (transient, produced per-render by cargo.js, consumed by objects_draw.js)
mask = { grab: boolean, box: boolean, dump: boolean }
```

## Contracts & Interfaces

Single source of truth for every cross-file seam. Each consumer conforms byte-for-byte.

### Shared names / strings

- **Layer id / store key / chip attribute = `"deploy"`** — the store key `store.layers.deploy` (owner T1), the chip `data-layer="deploy"` (owner T2), and the layer registration `id: "deploy"` (owner T4) MUST all be the exact string `deploy`. `chrome.js`'s generic `data-layer` wiring binds the chip to the store key by this string; the layer's `visible()` reads `state.layers.deploy`.

### Signatures

- `CTPS_OBJ.drawDeployedZones(ctx, view, pose, robotCfg, project, mask)` — owner: **T3** (`objects_draw.js`); consumer: **T4** (`cargo.js` `_drawDeployLayer`). Draws only the areas whose `mask` flag is true (`mask.grab`→`robotCfg.grab_zone`, `mask.box`→`robotCfg.box_zone`, `mask.dump`→`robotCfg.drop_points`) in the filled-accent style. No-ops on a missing `robotCfg`/`pose`/`mask` or an absent zone.
- `_deployMaskAt(t, state) -> { grab, box, dump }` — owner: **T4** (`cargo.js`, module-private, not exposed on `CTPS`). `grab` = a cached `release` window covers `t`; `dump` = a cached `dump` window covers `t`; `box` = `!!(state.scene && state.scene.rearBox)`. The `release`/`dump` window lists are built once from `CTPS.engine.events` (cached; events never change after ingest), so `_deployMaskAt` does a bounded membership check, not a per-call full scan.

### Mask shape (the T3↔T4 seam)

`{ grab: boolean, box: boolean, dump: boolean }` — produced by `_deployMaskAt` (T4), consumed by `drawDeployedZones` (T3). Both files use exactly these three keys.

### Layer registration (owner T4)

`CTPS.layers.register({ id: "deploy", canvasId: "trail", visible: state => !!(state.layers && state.layers.deploy), draw: _drawDeployLayer, invalidateOn: ["t", "scene", "layers"] })` — registered AFTER the `zones` layer so the accent fill paints above the static overlay.

### Constants (named, no magic literals)

- `MANIP_NOMINAL_S = 0.5` in `cargo.js` — mirror of `ctps/config.py::MANIP_NOMINAL_S`, with a single-source comment (same discipline as the `MAGAZINE_LANES`/`MAGAZINE_DEPTH`/`REAR_BOX_CAP` mirrors already in `cargo.js`).
- `DEPLOY_FILL_ALPHA = 0.22`, `DEPLOY_STROKE_W = 2`, `DEPLOY_DOT_R = 4` in `objects_draw.js` (top-of-IIFE named constants, alongside the existing `ZONE_*` constants) — owner **T3**. No inline numeric literals in the draw body.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `tools/path_planner/ctps/assets/store.js` | T1 | — |
| `tools/path_planner/ctps/player_html.py` | T2 | — |
| `tools/path_planner/ctps/assets/objects_draw.js` | T3 | T4 (runtime) |
| `tools/path_planner/ctps/assets/cargo.js` | T4 | — |
| `tools/path_planner/tests/test_ctps_player_html.py` | T5 | — |

## Error Handling

- **No layout / no `robot` section:** `_drawDeployLayer` returns early on `!state.robotCfg`; the chip still toggles. (AC5)
- **No area deployed at `t`:** `_deployMaskAt` returns all-false → `_drawDeployLayer` returns before drawing (trail canvas already cleared). (AC5)
- **Missing individual zone** (e.g. `grab_zone` absent but mask.grab true): `drawDeployedZones` skips that area (guarded per-zone), draws the others.
- **`scene` null (before first manip event / no layout):** `box` flag is false; no error.
- **Malformed / non-finite zone dims:** sanitized upstream by two different paths — the in-browser UPLOAD leg runs `_coerceRobotCfg` (drops bad zones), while the EMBEDDED `const OBJECTS` leg (cargo.js:616-622 hands `OBJECTS` straight to `store.loadLayout`) is validated at emit time by `layout.py`'s strict `validate`. Either way `drawDeployedZones` should still guard each zone per-area (those guards are load-bearing, not belt-and-suspenders — the embedded path never touches `_coerceRobotCfg`).

## Testing Strategy

**Levels:** Unit (Python markup/emit assertions) + Manual (browser eyeball). No JS unit runner exists — the deploy logic is ES5 inlined into the emitted HTML and is verified by presence + eyeball, matching the existing `zones`/`objects` coverage.

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | Render any track → chip markup | Unit | `html` contains `data-layer="deploy"`, `>DEPLOY<`, and `id="ctps-layer-deploy"`; the deploy button carries class **exactly `ctps-chip`** (assert `id="ctps-layer-deploy" class="ctps-chip"` and NOT `ctps-chip--active`). (AC1) |
| TC2 | Render → store default | Unit | Emitted store JS contains `deploy: false` inside the `layers` literal. (AC2) |
| TC3 | Render → deploy layer + draw present in inlined JS | Unit | `html` contains `drawDeployedZones` and the `id: "deploy"` layer registration. (AC3, AC4) |
| TC4 | Existing `TC-CARGO-STRUCT` regression | Unit | Still green — `data-layer="objects"`/`"zones"`, cargo panel ids unaffected. (AC6) |
| TC5 | Full CTPS suite | Unit | `pytest tools/path_planner/tests/` green (no exhaustive-chip-set assertion breaks). (AC7) |
| TC6 | Eyeball deploy behavior | Manual | Serve over localhost, render `ctps_cargo_trace.json` with a robot-zone layout, toggle `DEPLOY`: box area fills while holding cargo, dump dots flash at each dump, grab rect flashes at each release; hidden otherwise; ZONES (if on) coexists. (AC3–AC6) |

**Test data:** existing fixtures — any track for TC1–TC4 (e.g. `fixtures/ctps_lrs_track.json`); for TC6 the cargo demo path (`fixtures/ctps_cargo_trace.json` rendered via `player_html.render(track_dict, out, objects=layout)` — see the CTPS cargo-demo render recipe memory). A `release` event is needed to eyeball the grab flash specifically; if the demo trace lacks one, verify box + dump from it and add a scratch `release` to a copy for the grab check.

**Run command:**
`uv run --extra dev pytest tools/path_planner/tests/test_ctps_player_html.py` then the full `uv run --extra dev pytest tools/path_planner/tests/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Add `deploy` store layer default | — | low | `tools/path_planner/ctps/assets/store.js` | In the `layers` object literal (store.js:48-49) add `deploy: false` (keep `zones: false`). Nothing else. Satisfies AC2. |
| T2 | Add `DEPLOY` chip markup | — | low | `tools/path_planner/ctps/player_html.py` | After the `ZONES` `<button>` (player_html.py:178-179) add `<button id="ctps-layer-deploy" class="ctps-chip" type="button" data-layer="deploy">DEPLOY</button>` — class is exactly `ctps-chip` (default OFF), NOT `ctps-chip--active`. Optionally extend the `_BODY_MARKUP` chip-row docstring comment (~player_html.py:126-136) to mention DEPLOY. **Do NOT touch the `PLAYER_META` constant (player_html.py:496-502) or its `lanes` list — it is coupled to frozen test literals.** No `chrome.js` change (generic `data-layer` wiring). Satisfies AC1. |
| T3 | Filled-accent masked draw in objects_draw.js | — | med | `tools/path_planner/ctps/assets/objects_draw.js` | (a) Add the named constants `DEPLOY_FILL_ALPHA = 0.22`, `DEPLOY_STROKE_W = 2`, `DEPLOY_DOT_R = 4` at the top of the IIFE (beside `ZONE_*`); no inline literals in the draw body. (b) Add `accent: _cssVar("--ctps-accent")` to `_resolveTokens`. (c) Extract the rotated-rect corner→path building of `_drawZoneRect` into a new path-only helper `_pathZoneRectWorld(ctx, view, pose, zone, project)` (moveTo/lineTo/closePath, no stroke); make `_drawZoneRect` call it then `stroke()` (output byte-identical). (d) Add + export `drawDeployedZones(ctx, view, pose, robotCfg, project, mask)` per the Contracts signature: for `mask.grab`/`mask.box`, path the zone via `_pathZoneRectWorld` then `fill()` at `globalAlpha = DEPLOY_FILL_ALPHA`, reset `globalAlpha = 1`, `lineWidth = DEPLOY_STROKE_W`, `stroke()`, all `--ctps-accent`; for `mask.dump`, draw each `drop_points` entry as a filled accent dot (reuse `_pathCircle`, radius `DEPLOY_DOT_R`). Wrap in `save/restore`, guard `!robotCfg||!pose||!mask` and each absent zone. ES5 only. Satisfies AC4, AC5. |
| T4 | Deploy resolver + `deploy` layer in cargo.js | T3 | med | `tools/path_planner/ctps/assets/cargo.js` | Add `var MANIP_NOMINAL_S = 0.5;` mirror (single-source comment → `ctps/config.py`). Build the `release`/`dump` flash-window lists **once** (cached in a module var, lazily on first `_deployMaskAt` call, since `CTPS.engine.events` never changes after ingest): each `manip` event with `method===METHOD_RELEASE`→a release window `[ev.t, ev.t+MANIP_NOMINAL_S]`, `method===METHOD_DUMP`→a dump window; mirror `chrome.js`'s compute-once `findWorstDrift` idiom, NOT a per-repaint full scan. Add `_deployMaskAt(t, state)` returning `{grab,box,dump}`: `grab` = a cached release window covers `t`, `dump` = a cached dump window covers `t`, `box = !!(state.scene && state.scene.rearBox)`. Add `_drawDeployLayer(ctx,view,state)`: return on `!OBJ.drawDeployedZones || !state.robotCfg`; compute mask; return if all false; `pose = CTPS.engine.poseAt(state.t)`; call `OBJ.drawDeployedZones(ctx,view,pose,state.robotCfg,OBJ.project,mask)`. Register the `deploy` layer per Contracts (AFTER the `zones` register at ~cargo.js:603-609 so it paints on top). Keep `_deployMaskAt` module-private (do not expose on `CTPS`). `METHOD_RELEASE`/`METHOD_DUMP` already defined (cargo.js:63,65). Satisfies AC3, AC5. |
| T5 | Tests: markup + JS-presence assertions | T1, T2, T3, T4 | low | `tools/path_planner/tests/test_ctps_player_html.py` | Add a `test_deploy_chip_and_layer_present` covering TC1–TC3: render a fixture track, assert `data-layer="deploy"`, `>DEPLOY<`, `id="ctps-layer-deploy"`, the negative `'id="ctps-layer-deploy" class="ctps-chip"'` (exact, no `--active`), `deploy: false`, `drawDeployedZones`, and `id: "deploy"` all appear in `html`. Do not weaken existing tests. Then run the full suite (TC4/TC5). Satisfies AC7, AC6, AC1, AC2. |
| T6 | Manual localhost eyeball | T1, T2, T3, T4, T5 | low | (no code) | Per TC6 + the eyeball memory: render `ctps_cargo_trace.json` with a robot-zone layout to an HTML file, `python -m http.server`, open `127.0.0.1` (claude-in-chrome blocks `file://`), toggle `DEPLOY`, scrub, and confirm: (a) box-hold fill across `boxClose`→`boxOpen`, (b) dump flashes, (c) release flash (add a scratch `release` if the trace lacks one), (d) all-hidden when idle, (e) ZONES coexistence when both on. ALSO: (f) render a **layout-free** page (no `objects=`), toggle DEPLOY, confirm it draws nothing and the console is clean (AC5); (g) confirm the 7-chip row does not overflow the stage at the narrowest supported width (Responsive); (h) one-line editor check — open the emitted layout-editor HTML and confirm its zones/objects render unchanged (shared `objects_draw.js`). Report findings. Satisfies AC3–AC6. |

## Notes for Implementer

- **ES5 only** in the JS assets (`var`/`function`, no arrow/let/const), no DOM inline styles; canvas colors resolved once via `getComputedStyle` tokens (extend `_resolveTokens`), never a hard-coded hex.
- **Draw order matters:** register `deploy` AFTER `zones` so the yellow accent fill paints above the cyan dashed static overlay on the shared `trail` canvas.
- **`globalAlpha` discipline:** set fill alpha → `fill()` → reset `globalAlpha = 1` → `stroke()`, all inside one `save()/restore()`. Reset before `stroke()` so the **outline is fully opaque**, not translucent (this is the reason — not "leaking into later repaints"; the `save()/restore()` plus each repaint's fresh `clearRect` already isolate canvas state).
- **Keep `_drawZoneRect` output identical** after extracting `_pathZoneRectWorld` — the static `zones` layer must stay byte-stable (AC6).
- **`box` uses the resolved scene**, so it shows across the whole `boxClose`→`boxOpen` span automatically via the store's `t`→scene bisect (the scene snapshot between the two events carries `rearBox`). No interval math needed for the box.
- **grab flashes on `release`, not `grab`** (per Decisions) — and `release` is a no-op in the cargo model and rare in comp traces, so the grab flash may seldom appear; that is intended.
- **Do not touch** `track_builder.py`, the Track/trace schema, `layout.py`, the compiler, or `chrome.js`.
- **Regenerating routes is NOT needed** (no field/graph change).
- **Rollback:** the feature is additive and gated behind a default-off chip; reverting the five edits removes it with no effect on any existing trace/layer.
- No existing test asserts an *exhaustive* layer-chip set or the store `layers` literal (only per-chip `data-layer=` presence, test_ctps_player_html.py:1224-1225) — verified — so the new chip/key won't red an existing assertion.
- `objects_draw.js` is shared with the offline layout editor, but the editor never calls `drawDeployedZones`, so the addition is inert there; the `_drawZoneRect` refactor must keep byte-identical stroked output for the editor's zone draw.
