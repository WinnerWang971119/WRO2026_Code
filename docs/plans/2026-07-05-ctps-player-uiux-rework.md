# CTPS Player UI/UX Rework Plan

**Goal:** Re-skin and re-lay-out the CTPS trajectory player to match the handed-off
high-fidelity design ("option 3a Instrument", a warm-dark console), over the *existing*
Track data and playback engine, while restructuring the source into modular assets and
leaving cheap, invisible seams for three feature issues (#57, #55, #23).

**Approach:** Keep the existing single-file, offline, vanilla-JS Canvas player as the
substrate — its bounded seek (`findKeyframeIndex`/`poseAt`), world↔pixel transform, and
event model are sound and test-locked. Split the runtime into an ordered set of
`assets/*.js` + `*.css` files that `player_html.render()` concatenates and inlines into
the one self-contained HTML output (exactly how it already inlines `player.js`). Layout
and behavior come from design 3a. Everything that would need new trace data is a
reserved shell or a code-only seam. This is issue **#125** (UI/UX only); features #57 /
#55 / #23 remain their own issues. Both Fable and Codex independently endorsed this
approach.

## Scope

- **In scope:**
  - Full visual + layout rework of the Player to design 3a: header (wordmark, trace
    filename, static `PREDICTED` badge, drift chip, `? Help`), canvas stage (dimmed map
    scrim, bounds, full + traveled trajectory, nodes with halos, **box** robot, scale
    bar, MAP/TRAIL/NODES layer chips, ⛶ Full-view), right rail (Current command / HOLDING
    [reserved] / tabbed Log+Inspector), transport bar (⏮ play-pause ⏭, drift-jump, seek
    slider with drift ticks, time readout, 0.5–4× speed), timeline dock (TASK / GOTO /
    HOLDING / DRIFT lanes + playhead + drag-scrub), Full-view mode, Help/legend modal.
  - Wire to **existing** Track data: robot pose/heading + full & traveled trajectory
    (keyframes), Current command + Log (events ≤ playhead), TASK lane (task_start/end
    spans), GOTO lane (goto spans; see the emit below), DRIFT lane/chip/ticks (drift_warn),
    Inspector (pose/heading/nearest node/
    drift-check), node **click-selection** (canvas hit-test → inspector).
  - Source restructure: modular `assets/*.js` + `*.css` concatenated at emit; a Python-
    emitted `PLAYER_META` inline constant; an emit-time self-containment lint; layer /
    render-mode / timeline-lane / panel-tab / inspector-field **registries**.
  - Full keymap (Space, `[` `]`, Shift+←/→ event-step, Home/End, `D`, `F`, `?`; plain ←/→
    = the native range's built-in frame-step on the focused `#ctps-seek`).
  - **Minimal `track_builder` GOTO-span emit** (Python-only): a successful `goto` emits a
    `goto` event with `t` / `t_end` / `from->target`, so the GOTO lane draws spans and the
    Log + Current command caption gotos (today it emits nothing on success). No C++.
  - Preserve/extend the emitter test contract; a new ADR for the conventions introduced.

- **Out of scope (reserved seams — no visible UI, or empty shell):**
  - **#57** — object authoring + real cargo: HOLDING panel + HOLDING lane ship as empty
    shells (design "empty" slots); no grab/put wiring, no object placement/save.
  - **#55** — pivot wheel-arc: only the render-mode registry + `box` default marker
    ship; no arc animation.
  - **#23** — LRS/telemetry: source badge is static `PREDICTED`; source-agnostic ingest,
    lane registry, tab registry, inspector-field, render-mode registry are the seams; no
    telemetry UI, no source switcher.
  - Any **C++/recorder** change — the GOTO-span emit is a Python-only `track_builder`
    addition (additive, schema-tolerant), NOT a firmware/recorder change. Also out:
    broadening drift to raw drive commands; runtime tweak controls for accent/map-dim.

## Decisions

- **Modular source, single-file output (Fable+Codex endorsed).** `player_html.render()`
  concatenates ordered `_ASSET_CSS` + `_ASSET_JS` tuples into the existing inline
  `<style>`/`<script>` slots. Output stays one self-contained offline HTML; no bundler,
  no ES modules, no external refs. Rejected: extend one giant `player.js` in place — at
  ~1.5–2.5k lines it turns every future feature issue into a merge conflict in one file.
- **Pin the seek core.** `findKeyframeIndex` / `poseAt` / `findEventIndex` /
  `wrapDeltaDeg` stay in `assets/player.js` with their current declaration style and
  indentation; `PLAYER_JS_PATH` keeps pointing there. TC12's regex is brittle to exactly
  this refactor — do not move what it watches.
- **Emit-time self-containment lint.** With many concatenated assets, "no `src=`/`href=`/
  `http(s)` substring" cannot stay a whitespace accident; `render()` raises if any asset
  or the final HTML contains a forbidden substring.
- **Stacked-canvas layer pipeline + registries**, replacing the single `getImageData`
  static-layer snapshot (which fights the traveled overlay + layer toggles and already
  caused one HiDPI ghosting bug). Layer / render-mode / lane / tab / inspector-field are
  registries so #23/#55/#57 are additive registrations.
- **One store + event-cursor service.** All mutable UI state in one small store with
  `subscribe`; one `findEventIndex(t)` per refresh feeds every stateful panel via a
  forward/rebuild cursor; Log renders only its ~10-line window. Fixes today's O(n)
  `rebuildLog`. HOLDING registers a no-op consumer (the reserved seam).
- **CSS classes + custom properties, never inline styles.** JS sets one custom property
  per animated axis (`--ctps-frac`, …) via `setProperty`; all styling lives in the
  stylesheet. This carve-out (dynamic values via CSS vars ≠ inline-style violation) is
  documented in a new ADR.
- **Seek = styled native `<input type=range id=ctps-seek>`** + overlay for accent fill +
  drift ticks. Keeps a11y/keyboard and the test markers while matching 3a.
- **`PLAYER_META`** inline constant (emitted by Python) carries render-time knowledge:
  source badge (`PREDICTED`), trace filename, lane manifest, tab manifest, default render
  mode. #23's predicted/logged distinction is a `render(source=…)` kwarg later — it is
  not discoverable in JS.
- **Robot = box** (chassis rectangle + heading nose), overriding 3a's triangle, to suit
  the #55 pivot seam.
- **GOTO lane live** via a minimal Python `track_builder` goto-span emit (success →
  `t`/`t_end`/`from->target`; UNREACHABLE stays a fault pin). **HOLDING reserved** (empty
  shells). TASK + DRIFT live. (I first reserved GOTO after finding no successful-goto event
  in `track_builder`; per user direction we add the small Python emit to make it live — no
  C++, additive + schema-tolerant.)

## Acceptance Criteria

- [ ] `player_html.render()` emits ONE self-contained `.html` that opens on `file://`
      with no external `src=`/`href=`/`http(s)`; the design 3a layout renders (header,
      canvas stage, right rail, transport, timeline dock). (AC-selfcontained)
- [ ] The existing emitter tests (`test_ctps_player_html.py` TC5/TC6/TC12 + the three map
      tests) pass unchanged: `id="ctps-play"`, an `<input type="range">`, `id="ctps-time"`,
      `<canvas>`, `const TRACK = {…};` byte round-trip, bounded-seek. (AC-contract)
- [ ] `render()` raises a clear error if any **asset or the HTML template** (NOT the embedded
      TRACK/META/MAP_IMAGE JSON values) contains `src=`, `href=`, `http://`, `https://`,
      `data:image`, or `</script>`. (AC-lint)
- [ ] `const PLAYER_META = {…};` is embedded and round-trips via `json.loads` like TRACK;
      carries source `"PREDICTED"`, filename, lane + tab manifests, default render mode
      `"box"`. (AC-meta)
- [ ] Scrubbing drives every region from playhead `t`: box robot pose/heading, traveled
      polyline (start→playhead), Current command (last event ≤ t), Log window, Inspector
      values, drift-check — with backward-seek rebuild bounded (no full-array per-frame
      scan; TC12 style). Behavior verified manually (T16); text-heuristics TC-TRAIL/TC-NOINLINE. (AC-scrub)
- [ ] Transport: play/pause, 0.5×/1×/2×/4×, Home/End, ⚑‹/⚑› drift jumps, event stepping,
      seek slider with red drift ticks; full keymap works. Verified manually (T16). (AC-transport)
- [ ] Timeline dock: TASK spans (task_start/end) and DRIFT pins+uncorrected-span
      (drift_warn) render from real events; GOTO renders live spans (goto success events,
      `t`→`t_end`) plus UNREACHABLE fault pins; HOLDING is an empty reserved lane; playhead line +
      drag-to-scrub work. (AC-timeline)
- [ ] Right rail: Current command panel; Log/Inspector tabs (Log click-to-seek; Inspector
      shows pose/heading/nearest node/drift-check for the selected entity); clicking a node
      on canvas selects it and swaps the inspector; HOLDING panel is an empty reserved
      shell. Verified manually (T16). (AC-rail)
- [ ] Full-view collapses rail + dock, canvas fills width, on-canvas HUD (current command +
      a reserved `holding 0` chip — no cargo data yet) shows; layer chips MAP/TRAIL/NODES
      toggle their layers; drift chip hidden when no drift. Verified manually (T16). (AC-fullview)
- [ ] Robot renders as an accent box with a heading indicator via the render-mode registry
      (`box` default); the registry admits a future `wheel-arc` mode without touching
      playback. Verified manually (T16). (AC-robotbox)
- [ ] A new ADR documents the CSS-custom-property styling convention + the self-containment
      lint + the modular-asset/registry/`PLAYER_META` structure; `DECISIONS.md` updated.
      (AC-adr)
- [ ] `uv run --extra dev pytest tools/path_planner/tests/test_ctps_player_html.py` is
      green; `python -m tools.path_planner.ctps --fixture` renders and opens a page that
      looks like 3a. (AC-verify)

## UI/UX Specification

> Carried verbatim from the Phase-1.5 external design handoff (option 3a "Instrument").
> The executor reproduces THIS; it does not re-design. Full design + exact tokens + the
> mockup's value source-of-truth live in the sidecar
> [`docs/plans/2026-07-05-ctps-player-uiux-rework.ui.md`](./2026-07-05-ctps-player-uiux-rework.ui.md).

**Chosen base:** 3a "Instrument" (dark monochrome-plus-accent console).
**Grafted in:** none (single handed-off direction).
**Deviations (authoritative):** robot = box (not triangle); Canvas-2D (not SVG);
CSS classes + custom properties (not inline styles); seek = styled native range input;
icons = unicode glyphs / inline SVG paths (no `href`); HOLDING reserved (GOTO live via a small `track_builder` emit).

### Layout (drawing)

```
┌──────────────────────────────────────────────────────────────────────────┐  #161614
│ CTPS  mission_04.trace  [PREDICTED]            ▲drift 12.4cm@46.5s   ? Help │  header, border #31302a
├──────────────────────────────────────────────────────────────┬───────────┤
│  CANVAS STAGE  #1e1d1a                        MAP TRAIL NODES  │ CURRENT    │  right rail 288px
│    dimmed map scrim rgba(22,22,20,.48) + bounds #3f3d35        │ COMMAND    │
│    full path #e8e5db@60% · traveled #ffd24a · nodes+halo       │ ────────── │
│    ▮ box robot (accent, dark outline) + heading nose           │ HOLDING    │  [reserved shell]
│    50cm scale bar                                    ⛶ Full     │ n/3 empty  │
│                                                                │ ────────── │
│                                                                │ Log|Insp   │  tabbed, accent underline
│                                                                │ …events…   │
├──────────────────────────────────────────────────────────────┴───────────┤
│ ⏮ ❚❚ ⏭ │ ⚑‹ ⚑› │ ●━━━━━━━━━━━━━━━━━━━━━━ (accent fill, red drift tick) │ 42.18/96.40s │ 0.5× [1×] 2× 4× │  transport
├──────────────────────────────────────────────────────────────────────────┤
│ TASK  │ ▭ T1 collect ▭ T2 deliver ▭ T3 return          │ ruler 0–90s      │  timeline dock
│ GOTO  │ ▭ HOME->J1 ▭ J1->J2 … spans + UNREACHABLE ▲ pin │                  │
│ HOLDING (reserved shell)                                │                  │  holdColor #5bc8e0
│ DRIFT │ ▲──────red uncorrected span──────▶              │  playhead │      │  #ff6b4a
└──────────────────────────────────────────────────────────────────────────┘
```

> **Illustrative-only in the mockup (do not reproduce literally):** the HOLDING `n/3`
> count + named cargo cards (reserved — renders as `0/3` + dashed empty slots), and the
> fixed `0–90 s` timeline ruler (compute the ruler from the trace's actual `duration_s`).

### Description

Fixed dark console filling the viewport; column flex `header → stage row (flex 1) →
transport bar → timeline dock`. Every region derives from the playhead `t` (bounded
bisect + lerp for pose; last event ≤ t for command/log; drift_warn for drift). Right rail
hides in Full view; the canvas then fills the width and an on-canvas HUD shows the current
command + holding count. Layer chips toggle map/trail/nodes. Tabs switch Log ⇄ Inspector.
No animation beyond playback; state changes are instant. See the sidecar for the full
per-region prose, states, and the mockup's exact values.

### Color codes

| Token | Hex | Use |
|---|---|---|
| bg | `#161614` | app background |
| panel | `#1e1d1a` | stage, rail, transport, dock |
| raised | `#26251f` | chips, current-event pill, buttons |
| border | `#31302a` / `#3f3d35` | subtle / strong |
| text | `#e8e5db` / `#a5a193` / `#918d7f` / `#565349` | primary / secondary / muted / faint |
| accent | `#ffd24a` | playhead, robot, trail, seek fill, active tab |
| hold | `#5bc8e0` | cargo (reserved) |
| drift | `#ff6b4a` (warn border `#4a2e26`) | drift chip/pins/ticks/lane |

### Type & spacing tokens
UI = `ui-sans-serif, system-ui`; data/log = `ui-monospace, Consolas, monospace`. No
webfonts. Panels radius 6px; chips/buttons 3–4px; borders 1px. Map dim `0.48` (fixed).

### States
default · playing · paused · seeking · no-drift (chip+lane hidden) · no-map (vector
fallback) · empty/short trace · Full view · tab Log/Inspector · selection robot/node.

### Responsive
Console fills the viewport; canvas resizes to its box (existing devicePixelRatio-aware
resize). Mockup's 1180×756 is only the capture size.

## Data Model

**Track (Data Model B — ingested; schema-version tolerant; `goto` now carries an optional `t_end`):**
```
{
  schema_version: int,   // stays 1 — adding optional t_end is backward-compatible
  field: { nodes: {name:[x,y]}, obstacles: [...], bounds: [x0,y0,x1,y1] },  // LOWERCASE, cm; player.js reads field.nodes/obstacles/bounds
  duration_s: float,
  keyframes: [ {t: float, x: float, y: float, h: float} ],   // pose; bounded-bisect + lerp
  events:    [ {t: float, type: str, label: str, t_end?: float} ]  // t_end on goto spans
}
```
Event `type` values: `task_start`, `task_end`, `drive`, `manip`, `wait`, `print`, `beep`,
`led`, `drift_warn`, and `goto` — emitted on **success** (NEW: `t`, `t_end`, label
`from->target`) and on **UNREACHABLE fault** (no `t_end`, label `UNREACHABLE …`). No
grab/put or item identity in `manip` (`f"{method}({arg})"`).

**PLAYER_META (NEW — emitted by Python, inline constant, round-trips like TRACK):**
```
{
  source: "PREDICTED",                 // #23 seam: "LOGGED" later via render(source=…)
  filename: "<trace basename>",        // header
  lanes: ["task","goto","holding","drift"],   // timeline lane manifest (order)
  tabs:  ["log","inspector"],          // rail tab manifest
  render_mode: "box"                   // default robot render mode (#55 seam)
}
```

**Store (NEW — client state, never persisted):**
```
{ t, playing, speed, activeTab: "log"|"inspector", fullView: bool,
  layers: {map, trail, nodes}, selection: {kind:"robot"|"node", id?} }
```
Derived per refresh (never stored): pose/heading, traveled path, current event, log
window, held items (reserved → empty), drift-check result, nearest node.

## Contracts & Interfaces

Single source of truth for every cross-task seam. Owner produces it; consumers conform
byte-for-byte.

### `window.CTPS` namespace (owner: **T1**, consumers: T3–T10)
```
window.CTPS = {
  engine: { findKeyframeIndex, poseAt, findEventIndex, wrapDeltaDeg, track, keyframes, events }, // player.js, PINNED
  layers, renderModes, timelineLanes, panelTabs, inspectorFields,       // registries
  store,                                                                // set by T3
  renderer,                                                             // set by T4 (resize/worldToPixel/hitTest)
  _inits: [ initFn ], _errors: [ str ],                                 // boot runs _inits in load order; surfaces _errors
}
```
Each registry exposes `register(descriptor)` + iteration; `register` **records a duplicate
`id` into `CTPS._errors`** (does NOT throw — each JS asset is its own `<script>`, and boot
surfaces `_errors` via `console.error`, so one bad module can't kill the page). Each module
IIFE, on load, pushes its `init(ctx)` onto `CTPS._inits` and/or registers descriptors.
`ctx = { store, dom, track, meta }`, `track === TRACK`.
**Engine ingestion (T1, PINNED):** `player.js` keeps its top-of-IIFE ingestion —
`track = (typeof TRACK!=='undefined')?TRACK:null; keyframes = track&&track.keyframes||[];
events = track&&track.events||[];` — and `engine`'s functions close over `keyframes`/`events`
(byte-stable for TC12). `CTPS` + `engine` are created **unconditionally** (even when `TRACK`
is undefined) so boot/modules always find the namespace; `engine` re-exposes
`track`/`keyframes`/`events` so other files reuse the same arrays without re-reading `ctx`.

### Descriptor interfaces (owners as noted; consumed by boot/renderer/panels/timeline)
- **layer** (T4): `{ id, canvasId, visible(state), draw(ctx, view, state), invalidateOn: [keys] }` — layers are draw-units mapped onto a backing canvas via `canvasId`. **`map`, `bounds`, `trajectory`, `nodes` are SEPARATE layers** (each its own `visible(state)`) though all share `canvasId:"static"`; `trail`→`"trail"`, `actors`→`"actors"`. Flipping a layer's visible re-renders only that canvas — this is what lets the MAP/NODES/TRAIL chips toggle independently.
- **renderMode** (T4): `{ id, draw(ctx, pose, view, chassis) }` — `box` is default (T4); `wheel-arc` reserved (#55)
- **renderer surface** (T4): `CTPS.renderer = { resize(), worldToPixel(x,y)->{px,py}, hitTest(px,py)->{kind:"node"|"robot", id?}|null }` — boot wires `window "resize" → CTPS.renderer.resize()`; T8 uses `hitTest`/`worldToPixel`.
- **timelineLane** (T6): `{ id, label, kind: "span"|"point", color, build(track)->[{t0,t1?,label,cls}] }` — `task`,`goto`,`drift` live (`goto` = spans `t`→`t_end` + UNREACHABLE pin); `holding` reserved (empty build)
- **panelTab** (T7): `{ id, label, render(el, state) }` — `log`, `inspector`; telemetry reserved (#23)
- **inspectorField** (T8): `{ id, label, value(state)->str, tone?(state)->"drift"|null }` — pose/heading/nearest/drift-check. (Panels can only set CSS classes, so the descriptor returns a semantic **tone** hint, NOT a hex — `"drift"` maps to `.ctps-inspector-value--drift`; avoids a 3-place color-literal coupling.)

### Event-cursor service (owner: **T3**, in `store.js`)
`CTPS.cursor()` (NO arg — uses `engine.findEventIndex`, which closes over the module
`events`; do not thread an events array through `findEventIndex(t)`) → `{ at(t) -> idx;
step(prevIdx, idx, {forward(slice), rebuild(idx)}) }`. Consumers (Log T7, HOLDING reserved
T7) register forward/rebuild callbacks; one bisect per refresh feeds all; Log rebuild
renders only the ~10-line window.
**Store subscribe (T3):** `store.subscribe(fn)` calls `fn(changedKeys, state)`; a layer's
`invalidateOn:[keys]` intersects `changedKeys` to decide whether to redraw its canvas.

### DOM ids / markers (test contract — owner: **T2** template; consumers style/hydrate)
`#ctps-play` (play/pause) · `<input type="range" id="ctps-seek">` · `#ctps-time` (readout) ·
`<canvas id="ctps-canvas">` (actors) + stacked `ctps-canvas-static` / `ctps-canvas-trail` ·
`#ctps-command` · `#ctps-log`.
- **Asserted TODAY** by `test_ctps_player_html.py`: `id="ctps-play"`, `<input`,
  `type="range"`, `id="ctps-time"`, `<canvas`. Keep these.
- **Asserted only AFTER T14 lands** (TC-STRUCT / TC-SEEKINPUT): `id="ctps-seek"`,
  `#ctps-command`, `#ctps-log`, the stacked-canvas ids. A mid-execution task must not
  "verify" these against the current test file — they aren't asserted until T14.
Do not rename any without updating T14.

### CSS custom properties & token vars (owner: **T11** defaults; JS writes only via `setProperty`)
- **Dynamic (JS-written):** `--ctps-frac` (playhead 0–1) — **set on
  `document.documentElement`** so the seek fill (T13) and timeline playhead (T6) both read
  it. JS NEVER sets `el.style.<prop>` (the ADR-053 carve-out — one custom property per
  animated axis).
- **Static token vars** (T11 defines; T12/T13 style against them; the **renderer T4 resolves
  ALL canvas colors from them** via `getComputedStyle(document.documentElement)
  .getPropertyValue(...)` at init/resize — NO hard-coded hex in JS, keeping no-magic-numbers
  single-source): `--ctps-bg` `#161614` · `--ctps-panel` `#1e1d1a` · `--ctps-raised`
  `#26251f` · `--ctps-border` `#31302a` · `--ctps-border-strong` `#3f3d35` · `--ctps-text`
  `#e8e5db` · `--ctps-text-2` `#a5a193` · `--ctps-muted` `#918d7f` · `--ctps-faint`
  `#565349` · `--ctps-accent` `#ffd24a` · `--ctps-hold` `#5bc8e0` · `--ctps-drift`
  `#ff6b4a` · `--ctps-warn-border` `#4a2e26` · `--ctps-map-dim` `0.48`.

### CSS class inventory (owner: **T2** markup + dynamic-DOM tasks; consumed by T12/T13)
Pin the cross-task class names so parallel markup + styling agree. Root `.ctps-app`;
full-view toggles `.ctps-app.is-fullview` (T9 sets; T12's collapse rules key on it). The
five container classes **TC-STRUCT asserts**: `.ctps-header`, `.ctps-stage`, `.ctps-rail`,
`.ctps-transport`, `.ctps-timeline`. Dynamic DOM: `.ctps-lane` / `.ctps-lane-label` /
`.ctps-span` / `.ctps-pin` (T6); `.ctps-log-line` (+ `--current` / `--drift`) (T7);
`.ctps-tab` (+ `--active`) (T7); `.ctps-chip` (+ `--active`) (T4/T9); `.ctps-hud` (T9);
`.ctps-modal` (T9 help); `.ctps-hold-card` / `.ctps-hold-empty` (T7, reserved). Any new
class is added HERE first, never invented locally.

### Emitter (owner: **T2**, in `player_html.py`)
`_ASSET_CSS: tuple[Path,...]` = (`tokens.css`, `layout.css`, `components.css`) → one
`<style>`. `_ASSET_JS: tuple[Path,...]` = (`player.js`, `store.js`, `renderer.js`,
`transport.js`, `timeline.js`, `panels.js`, `selection.js`, `chrome.js`, `boot.js`) —
**order is the load contract; `player.js` first, `boot.js` last. Each JS asset is emitted in
its OWN `<script>` element** so a throwing top-level IIFE in one module can't abort the
others. Signature keeps the existing param name: `render(track, out_path, map_png=None,
source="PREDICTED")`. It emits `const TRACK`, `const MAP_IMAGE`, `const PLAYER_META` **each
in its own `<script>`** — so `_TRACK_RE`'s `...;</script>` terminates at TRACK alone (a
same-script `PLAYER_META` would be swallowed and break TC5/TC6/map round-trips), and a
sibling regex can pull `PLAYER_META`. `_assert_self_contained(...)` lints the **assets + the
HTML template string ONLY** (never the embedded TRACK/META/MAP_IMAGE JSON values, which
carry arbitrary user text such as `print` labels): raises `ValueError` on `src=`, `href=`,
`http://`, `https://`, `data:image`, or `</script>` in any asset/template. The MAP_IMAGE
`data:image/png` value is exempt (it is a JSON value, not an asset).

### File ownership
| File | Owner | Consumers |
|---|---|---|
| `tools/path_planner/ctps/assets/player.js` | T1 | all JS modules (namespace/engine) |
| `…/assets/store.js` | T3 | T4–T9 |
| `…/assets/renderer.js` | T4 | T8 (transform/hit-test), boot |
| `…/assets/transport.js` | T5 | boot |
| `…/assets/timeline.js` | T6 | boot |
| `…/assets/panels.js` | T7 | boot |
| `…/assets/selection.js` | T8 | boot |
| `…/assets/chrome.js` | T9 | boot |
| `…/assets/boot.js` | T10 | — |
| `…/assets/tokens.css` | T11 | T12, T13 |
| `…/assets/layout.css` | T12 | — |
| `…/assets/components.css` | T13 | — |
| `tools/path_planner/ctps/track_builder.py` | T17 | T6 / T14 (goto events) |
| `tools/path_planner/ctps/player_html.py` | T2 | T14 |
| `tools/path_planner/tests/test_ctps_player_html.py` | T14 | — |
| `docs/decisions/053-*.md`, `DECISIONS.md` | T15 | — |

## Error Handling

- **Forbidden substring in an asset or the HTML template** (`src=`/`href=`/`http(s)`/`data:image`/`</script>`)
  → `render()` raises `ValueError` naming the asset + substring (fail loud at emit; CI/test
  catches it). The embedded TRACK/META/MAP_IMAGE JSON values are NOT linted (they carry
  arbitrary user text).
- **Missing asset file in a tuple** → `render()` raises `FileNotFoundError` with the path.
- **Duplicate registry id** → `register()` records it into `CTPS._errors` (does NOT throw —
  each JS asset is its own `<script>`); boot surfaces `_errors` via `console.error`. Non-fatal.
- **Registry key in `PLAYER_META` with no implementation** → boot logs a
  `console.warn("CTPS: no <kind> registered for '<id>'")` and skips it (mirrors the
  existing `unknown drive.method` 3-place-contract gotcha) — never throws in the player.
- **No map** → existing `MAP_IMAGE = null` vector fallback. **No drift events** → drift
  chip + DRIFT lane hidden. **Empty/short trace** → guards as today (`keyframes||[]`).
- **Backward seek** → cursor `rebuild(idx)` reconstructs only bounded windows; no full
  per-frame scan.

## Testing Strategy

**Levels:** Unit (Python emitter text assertions; JS is exercised as text like the
existing suite — no browser). Integration/manual (render + open, visual check vs 3a).

| ID | Test Case | Type | Expected Behavior |
|---|---|---|---|
| TC5 | render is self-contained (existing) | Unit | passes unchanged: markers present, TRACK round-trips, no src/href/http |
| TC6 | LRS-shaped dict round-trips (existing) | Unit | passes unchanged |
| TC12 | bounded-seek in player.js (existing) | Unit | passes unchanged: `findKeyframeIndex` bisect + `poseAt` delegates, in `player.js` |
| TC-map1/2/3 | map data-URI / null / missing (existing) | Unit | pass unchanged |
| TC-META | `PLAYER_META` embedded + round-trips | Unit | `const PLAYER_META = {…};` parses; has source/filename/lanes/tabs/render_mode |
| TC-LINT-pos | asset with `src=`/`href=`/`http(s)`/`data:image`/`</script>` | Unit | `_assert_self_contained` raises ValueError naming the asset + offending substring |
| TC-LINT-neg | clean assets + data:URI map | Unit | no raise; the MAP_IMAGE `data:image/png;base64,` VALUE is allowed (it's exempt — a JSON value, not an asset) |
| TC-STRUCT | 3a structural markers present | Unit | the five container classes `.ctps-header/.ctps-stage/.ctps-rail/.ctps-transport/.ctps-timeline` + `#ctps-command`/`#ctps-log` in HTML |
| TC-ASSETS | every `_ASSET_JS`/`_ASSET_CSS` file exists + inlined | Unit | each file's content substring appears in HTML; missing file → FileNotFoundError |
| TC-SEEKINPUT | seek is a native range with id | Unit | `<input type="range" id="ctps-seek"` present (a11y/test contract) |
| TC-TRAIL | trail rebuild is bounded | Unit | `renderer.js` trail path references `findKeyframeIndex`; no `for (...keyframes.length...)` full scan in it |
| TC-GOTOEND | goto lane reads spans | Unit | `timeline.js` goto lane `build` references `t_end` (span) and handles the UNREACHABLE label (pin) |
| TC-NOINLINE | ADR-053 styling carve-out | Unit | no `.style.<prop> =` assignment in any JS asset except `.style.setProperty(` |
| TC-DUPREG | duplicate registry id is non-fatal | Unit | the `register` path has no top-level `throw` — it pushes to `_errors`; boot references `_errors` |

**Test data:** existing fixtures (`ctps_two_goto.json`, `ctps_lrs_track.json`,
`toy_field_data`); add an asset-with-`src=` temp file for TC-LINT-pos.
**Run command:** `uv run --extra dev pytest tools/path_planner/tests/test_ctps_player_html.py`
then `python -m tools.path_planner.ctps --fixture` for the visual check.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Core: pinned seek + namespace + registries | — | high | `assets/player.js` | Rewrite `player.js` to the slim core: keep `findKeyframeIndex`/`poseAt`/`findEventIndex`/`wrapDeltaDeg` **byte-stable** (same decls + 4-space IIFE indent, so TC12 passes). **Keep the TRACK/keyframes/events ingestion and create `CTPS` + `engine` UNCONDITIONALLY** (even if TRACK undefined — see Contracts → Engine ingestion). Add `window.CTPS` with `engine` (re-exposing track/keyframes/events) + empty `layers`/`renderModes`/`timelineLanes`/`panelTabs`/`inspectorFields` registries (each `register(desc)` **records dup id into `_errors`, no throw** + iteration), `_inits:[]`, `_errors:[]`. Remove the old rendering/transport/narration (moves to T4–T9). Satisfies AC-contract, AC-robotbox seam. |
| T2 | Emitter pipeline + template + PLAYER_META + lint | — | high | `player_html.py` | New 3a HTML template (containers per the drawing; required markers `#ctps-play`, `<input type=range id=ctps-seek>`, `#ctps-time`, `<canvas id=ctps-canvas>` + stacked `ctps-canvas-static/-trail`, `#ctps-command`, `#ctps-log`, header/stage/rail/transport/timeline scaffolds — all class-based). Add `_ASSET_CSS`/`_ASSET_JS` tuples (see Contracts) concatenated into the `<style>`/`<script>` slots. Emit `const TRACK`, `const MAP_IMAGE`, `const PLAYER_META` **each in its OWN `<script>`**, and each JS asset in its own `<script>` (Contracts → Emitter — this stops `_TRACK_RE` swallowing PLAYER_META). Add `source="PREDICTED"` kwarg; keep param name `out_path`. Keep `PLAYER_JS_PATH`→`player.js` and map inlining. Add `_assert_self_contained(...)` that lints the assets + template string ONLY (NOT the TRACK/META/MAP_IMAGE JSON values) and raises on `src=`/`href=`/`http(s)`/`data:image`/`</script>`; call before write. Satisfies AC-selfcontained, AC-lint, AC-meta, AC-contract. |
| T3 | Store + event-cursor service | T1 | med | `assets/store.js` | `CTPS.store` = `{t,playing,speed,activeTab,fullView,layers:{map,trail,nodes},selection}` + `subscribe(fn)` (calls `fn(changedKeys, state)`) + small action helpers (setT/seek/togglePlay/setSpeed/toggleLayer/select/setTab/toggleFull). `CTPS.cursor()` service (NO arg — uses `engine.findEventIndex`; see Contracts): one bisect per refresh, forward/rebuild dispatch to registered consumers. Push init onto `_inits`. Satisfies AC-scrub. |
| T4 | Layered canvas renderer + render-mode registry | T1, T3 | high | `assets/renderer.js` | Stacked canvases (`static`, `trail`, `actors`) via one shared resize (devicePixelRatio-aware) + world↔pixel transform (port from `player.js`). Register SEPARATE layers (Contracts → layer): `map` (scrim `--ctps-map-dim`), `bounds`, `trajectory` (full path), `nodes` (dots+halo) — all `canvasId:static`, each with its own `visible(state)` bound to store.layers so the MAP/NODES/TRAIL chips toggle independently; `trail` (`canvasId:trail`; traveled start→playhead in accent — incremental on forward play, rebuild to `findKeyframeIndex(t)` on backward seek, bounded); `actors` (`canvasId:actors`; box robot + selection halo + scale bar + full-view HUD). **Resolve ALL canvas colors from the `--ctps-*` custom properties via `getComputedStyle(document.documentElement)` at init/resize — no hard-coded hex.** Register `renderMode` `box` (rotated chassis rect + heading nose) as default; leave `wheel-arc` unregistered (#55 seam). Expose `CTPS.renderer = {resize, worldToPixel, hitTest}` for boot/T8. Satisfies AC-scrub, AC-robotbox, AC-fullview(canvas). |
| T5 | Transport + keymap | T1, T3 | med | `assets/transport.js` | Wire `#ctps-play`, native `#ctps-seek` (input→seek; reflect `t`→value + `--ctps-frac`), `#ctps-time` readout, speed segmented 0.5–4×, ⏮/⏭ Home/End, ⚑‹/⚑› next/prev drift (bisect over drift_warn). rAF play loop advancing `t` by real elapsed×speed (reuse existing clock). Keymap: Space, `[` `]`, Shift+←/→ (event step), Home/End, `D`, `F`, `?`. All via store actions. Satisfies AC-transport. |
| T6 | Timeline dock + lane registry | T1, T3 | med | `assets/timeline.js` | DOM dock (label col + lane col + ruler); playhead div positioned by `--ctps-frac`; drag-to-scrub → store.seek. Register lanes: `task` (spans from task_start/task_end), `drift` (pin at each drift_warn + red uncorrected span to end), `goto` (**live**: spans from goto success events `t`→`t_end`, plus a UNREACHABLE fault pin), `holding` (reserved: build() returns []). Spans as %-positioned divs (kind span/point). Satisfies AC-timeline. |
| T7 | Right rail: command + log/inspector tabs + holding shell | T1, T3 | med | `assets/panels.js` | Current-command panel (`#ctps-command` + detail; last event ≤ t; drift events in `#ff6b4a`). Tabbed panel: register `log` tab (`#ctps-log`; ~10-line window ≤ t via cursor; click line→seek; current-event pill; drift lines red) and `inspector` tab (renders registered inspectorFields for store.selection). HOLDING panel = reserved empty shell — dashed `.ctps-hold-empty` slots + a static `0 / 3` count; NO cargo data (the mockup's named cards + `n/3` are illustrative), register a no-op cursor consumer. Tab underline follows active. Satisfies AC-rail. |
| T8 | Canvas selection → inspector | T1, T3, T4 | med | `assets/selection.js` | Click on canvas → `renderer.hitTest` → nearest node within radius (else robot) → `store.select`. Register inspectorFields: `pose` (x,y cm), `heading` (°), `nearest` (node+dist), `drift-check` (last drift_warn ≤ t result, OK `#e8e5db` / off-node `#ff6b4a`). Robot is default selection. Satisfies AC-rail(selection). |
| T9 | Chrome: header + full-view + help | T1, T3 | med | `assets/chrome.js` | Header wiring: filename + `PREDICTED` badge from PLAYER_META; drift chip (worst drift_warn, click→seek, hidden when none); layer chips MAP/TRAIL/NODES→store.layers. Full-view toggle (⛶): store.fullView adds `.is-fullview` to the root (T12 collapses rail+dock), canvas fills, on-canvas `.ctps-hud` (current command + a reserved `holding 0` chip — no cargo data yet). `? Help` opens a `.ctps-modal` listing the keymap + color legend (unicode/inline-SVG only, no `href`). Satisfies AC-fullview. |
| T10 | Boot / bootstrap | T1, T3 | med | `assets/boot.js` | On DOMContentLoaded: read `TRACK` + `PLAYER_META`, build `ctx={store,dom,track,meta}`, run `CTPS._inits` in load order, validate each `PLAYER_META.lanes/tabs/render_mode` has a registered impl (else `console.warn`), log any `CTPS._errors` via `console.error`, wire `window "resize" → CTPS.renderer.resize()`, seek to 0. No feature-specific switches. |
| T11 | Design tokens stylesheet | — | low | `assets/tokens.css` | `:root` custom properties for ALL token vars in Contracts (`--ctps-bg/panel/raised/border/border-strong/text/text-2/muted/faint/accent/hold/drift/warn-border/map-dim`), plus the dynamic `--ctps-frac:0` default, type stacks, spacing, radii — per the sidecar token table. |
| T12 | Layout stylesheet | T2, T11 | low | `assets/layout.css` | App/header/stage-row/rail(288px)/transport/timeline flex layout; full-view collapse rules (rail+dock hidden, canvas fills); responsive fill-viewport + canvas box. Classes only. |
| T13 | Components stylesheet | T2, T11 | med | `assets/components.css` | Chips, buttons, tabs (accent underline), panels/cards, lanes + spans + playhead, HUD, help modal, drift styling, and the **styled native range** (`#ctps-seek` track transparent + accent fill overlay driven by `--ctps-frac` + light thumb via `::-webkit-slider-thumb`/`::-moz-range-thumb`; drift ticks as overlay divs). **No `data:image` backgrounds** (the emitter lint bans `data:image` in assets) — pure CSS shapes. Exact values from the sidecar/mockup. Classes only; no inline styles. |
| T14 | Emitter tests (keep + extend) | T1,T2,T3,T4,T5,T6,T7,T8,T9,T10,T11,T12,T13,T17 | med | `tools/path_planner/tests/test_ctps_player_html.py` | Keep TC5/TC6/TC12 + 3 map tests green against the new pipeline (adjust ONLY incidental text that legitimately moved, never the asserted markers/semantics). Add TC-META, TC-LINT-pos, TC-LINT-neg, TC-STRUCT, TC-ASSETS, TC-SEEKINPUT, TC-TRAIL, TC-GOTOEND, TC-NOINLINE, TC-DUPREG (see Testing Strategy). Satisfies AC-contract, AC-lint, AC-meta. |
| T15 | ADR-053 + DECISIONS index | — | low | `docs/decisions/053-ctps-player-modular-shell.md`, `DECISIONS.md` | Write ADR-053 (verify 053 is free at write time; else next number): CTPS player = modular concatenated assets + registries + `PLAYER_META`; dynamic styling via CSS custom properties set through `setProperty` is NOT an inline-style violation; emit-time self-containment lint. **Cite the #55 direction** so the single-entry `renderModes` registry isn't "simplified away" as speculative. Add the index line + status-table bump in `DECISIONS.md`. Satisfies AC-adr. |
| T16 | End-to-end verify | T14, T15 | med | (runs commands) | Run `uv run --extra dev pytest tools/path_planner/tests/test_ctps_player_html.py` (green) and `python -m tools.path_planner.ctps --fixture` (or `--trace` a fixture); open the page and confirm it looks like 3a (header/stage/rail/transport/timeline, box robot, dark palette, scrub drives all regions). Fix any integration wiring gaps found (in the owning file). Satisfies AC-verify. |
| T17 | track_builder GOTO-span emit | — | med | `tools/path_planner/ctps/track_builder.py` (+ affected fixtures/goldens) | In the successful-`goto` branch (`track_builder.py` ~L426): capture `t_start = t` BEFORE `step_through`, and after the hops emit `events.append({"t": t_start, "type": "goto", "t_end": t, "label": f"{from_node}->{target}"})` (reuse the already-captured `from_node`; keeps events sorted — `t_start` ≥ all prior events, `t_end` ≥ `t_start`). Leave UNREACHABLE as-is (no `t_end`). Update the "emits NO discrete event" comment. Run the FULL `uv run --extra dev pytest tools/path_planner/tests/` and update any Track-output goldens/fixtures the new event ripples into (watch sim-parity #108, strict-emitter fixtures). Additive + schema-tolerant; NO C++. Lights up GOTO lane spans + Log + Current command. Satisfies AC-timeline (GOTO live). |

## Notes for Implementer

- **TC12 is indentation/file-sensitive.** Its regex isolates `poseAt` ending on a
  4-space-indented `}` inside the `player.js` IIFE and requires `poseAt` to reference
  `findKeyframeIndex`. Keep the seek core's declarations and indentation exactly; do not
  relocate them out of `player.js`. Re-run TC12 after any `player.js` edit.
- **The self-containment bans are literal substrings** (in ASSETS + template only):
  `src=`, `href=`, `http://`, `https://`, `data:image`, `</script>`. `img.src = MAP_IMAGE;`
  (spaces) is fine — it's `src=` (no space) that's banned; never write an attribute
  `src=`/`href=`, no `<use href>`/`<image href>`, no `data:image` CSS background in an asset
  (use pure CSS shapes for the seek thumb/ticks), and no literal `</script>` inside a JS/CSS
  asset string. The emit lint (T2) is the backstop.
- **The MAP_IMAGE `data:image/png;base64,…` VALUE is exempt** — it's a JSON value, not an
  asset, so the map path stays as today (the `data:image` ban applies only to assets).
- **Keymap:** plain ←/→ is the native `<input type=range>` step on the focused `#ctps-seek`
  (no custom handler); Shift+←/→ is the event-step handler in T5.
- **`str.format` in the template:** asset contents are substituted as *values* (their
  `{`/`}` are inert). Do not add new literal `{`/`}` into `_HTML_TEMPLATE` unescaped, and
  don't put `<style>`/`<script>` braces in the template that collide with `.format`.
- **Bounded rebuild-on-seek:** the traveled overlay and the log window must be O(Δ) on
  forward play and O(window) on backward seek via the cursor/bisect — never a per-frame
  full `keyframes`/`events` scan (the invariant TC12 guards for pose).
- **No inline styles:** all styling in the stylesheets; JS sets only CSS custom properties
  via `setProperty`. This is the ADR-053 carve-out — cite it in module headers.
- **Reserved seams are registrations, not dead markup:** HOLDING = empty lane `build()`
  + no-op cursor consumer + empty panel shell; GOTO = live span `build()` (success `t_end`
  spans + UNREACHABLE pin, fed by T17); `wheel-arc`
  = an unregistered render mode; LRS = the `source` kwarg + the registries + the static
  badge. None add visible feature UI.
- **GOTO emit (T17) ripples the built Track's `events`** — run the FULL ctps pytest, not
  just the player test, and update Track-output goldens; watch sim-parity (#108) and
  strict-emitter fixtures. Additive + schema-tolerant (consumers ignore `t_end`).
- **Rollback:** the change is confined to `tools/path_planner/ctps/` (+ its test + ADR +
  one additive `track_builder` goto emit); the C++/recorder and every other tool are
  untouched. Revert the directory to fall back to the current player.
- **Parallelism:** T1, T2, T11, T15, T17 have no blockers. T3 after T1; T4–T9 after T1+T3
  (T8 also after T4); T10 after T1+T3; T12/T13 after T2+T11; T14 after all
  asset+emitter+T17 tasks; T16 last.
