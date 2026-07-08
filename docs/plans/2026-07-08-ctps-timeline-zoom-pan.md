# CTPS Timeline Zoom + Move Bar Plan

**Goal:** Make the CTPS player's bottom timeline dock navigable like a real video/DAW editor — a zoom control and a horizontal "move bar" (pan) — so a long trace is readable instead of cramming every span/pin into the full-width `0..duration` track.

**Approach:** Keep the proven percent-of-track positioning of spans/pins/ticks unchanged. Nest the ruler + all lanes + playhead inside ONE horizontal scroll viewport whose inner content track width = `duration × pixels-per-second`, driven by a single CSS custom property `--ctps-content-w`. The lane-label column stays visually pinned via `position: sticky; left: 0` while the track scrolls under it. A new asset `timeline_view.js` owns zoom state, the custom themed scrollbar, wheel/keyboard/follow behavior, and re-anchoring; `timeline.js` keeps owning lane/ruler DOM (with a stable ruler track element and a `rebuildRuler` hook for zoom-aware tick density). Chosen structural approach: **Scroll viewport + CSS content width** (Phase 4).

## Scope

- **In scope:**
  - A zoom widget docked top-right of the timeline dock: `−` / `+` buttons, a zoom slider, a `Fit` button, and a live `N.N×` readout.
  - `Ctrl`/`⌘` + mouse-wheel over the timeline to zoom, anchored at the cursor.
  - Keyboard: `+`/`=` zoom in, `-` zoom out, `0` fit-to-width (wired into the existing transport keymap).
  - A custom themed horizontal scrollbar (thumb) below the lanes for panning; consistent on Windows/macOS. Native scroll gestures (trackpad, shift-wheel) still work with the native scrollbar hidden; plain vertical wheel over the dock pans horizontally.
  - Playhead lives inside the scrolling content and pans/zooms with it; the frozen label gutter (TASK/GOTO/…) never scrolls.
  - Follow-the-playhead during playback (auto-scroll to keep the playhead visible), with manual pan temporarily suspending follow.
  - Zoom range: minimum = fit-to-width (never zoom out past the whole trace), maximum capped so a short window (~1.5 s) fills the view; default on load = fit-to-width (1×).
  - Zoom-aware ruler: tick density recomputed on zoom so tick spacing stays readable on screen.
  - Emitter structural-marker tests for the new DOM + asset registration.
- **Out of scope:**
  - The CTPS **layout editor** (`layout_editor_html.py` / `editor.css`) — player only.
  - The transport bar's top full-width seek slider and its red drift ticks — they stay a full-duration overview (the always-visible minimap) and do NOT zoom.
  - Any change to `track_builder`, the recorded trace JSON, `PLAYER_META`'s lane list, or the cargo/scene model.
  - Vertical zoom / lane-height resizing, per-lane collapse, timeline row reordering.
  - A minimap overview strip (considered, not chosen).

## Decisions

- **Structure: one scroll viewport, sticky gutter, dedicated content wrapper** — a `#ctps-tl-content` element (width = gutter + gap + `--ctps-content-w`) holds `#ctps-ruler`, `#ctps-lanes`, and `#ctps-playhead`; it is the scrolled content, so the playhead scrolls/zooms with the track for free. The gutter cells (`.ctps-lane-label`) pin via `position: sticky; left: 0`. This preserves the existing percent math (`pctOf`, `renderLaneItems`, `appendTick`) verbatim — only the container scaffolding and the playhead/scrub geometry change.
- **Percent positioning preserved** — spans/pins/ticks stay positioned as a percent of their track column, which is now `--ctps-content-w` wide instead of `1fr`. No pixel recompute of individual items (rejected Approach B).
- **New asset over bloating timeline.js** — zoom/pan is a distinct concern; it goes in a new `timeline_view.js` (registered in `_ASSET_JS` after `timeline.js`, before `boot.js`), matching ADR-053's modular-asset design. `timeline.js` stays the lane/ruler DOM owner.
- **`--ctps-content-w` as the single width knob** — one custom property (a CSS length, e.g. `"640px"`) set via `setProperty` on `.ctps-timeline` drives the grid track column, the content wrapper width, and the playhead calc. This is the ADR-053 setProperty carve-out (same as span `left`/`width`), not an inline-style violation.
- **Native scroll kept, native scrollbar hidden** — `overflow-x: auto` on the viewport so trackpad/shift-wheel momentum still works; the native scrollbar is visually hidden (`scrollbar-width: none` + `::-webkit-scrollbar { display: none }`) and a custom thumb synced to `scrollLeft` is the visible control. Best cross-OS consistency without losing native gestures.
- **Zoom anchoring** — button/slider/keyboard zoom keeps the playhead fixed on screen; `Ctrl`/`⌘`+wheel keeps the point under the cursor fixed.
- **Follow re-arm policy** — follow is active while `store.playing`; a manual pan during playback suspends it for the rest of that run; it re-arms when playback (re)starts or on `Fit`/`Home`.
- **No new ADR** — this is a feature inside the player, not a project-wide rule; it extends ADR-053's existing conventions (modular assets + setProperty carve-out). The `timeline.js` docstring and this plan record the design.
- **Verification** — Python emitter tests (structural markers, self-containment, asset inclusion) plus serving the emitted HTML over `127.0.0.1` (`python -m http.server`) and eyeballing/screenshotting on a long-duration trace. There is no JS unit-test runner for the player.

## Acceptance Criteria

- [ ] AC1 — On load, a trace fits entirely in the dock width at 1× (fit-to-width); no horizontal scroll appears until the user zooms in.
- [ ] AC2 — Clicking `+` / pressing `+`/`=` zooms in; the content track widens, spans/pins spread out, and the scrollbar thumb shrinks accordingly. `−` reverses it. Zoom never goes below fit-to-width.
- [ ] AC3 — `Ctrl`/`⌘` + wheel over the timeline zooms; the trace point under the cursor stays under the cursor after the zoom.
- [ ] AC4 — Button/slider/keyboard zoom keeps the playhead at the same on-screen x position before and after the zoom step.
- [ ] AC5 — The `Fit` button and the `0` key both reset to fit-to-width (1×) and scroll to the start.
- [ ] AC6 — The lane-label column (TASK/GOTO/MAGAZINE/REAR BOX/DRIFT) stays visually pinned on the left while the track is scrolled; labels are opaque (do not show spans bleeding through).
- [ ] AC7 — Dragging the custom scrollbar thumb pans the ruler + all lanes + playhead together, perfectly in sync; plain vertical wheel over the dock also pans horizontally.
- [ ] AC8 — Dragging on the lane/ruler area still scrubs the playhead (the existing gesture), and the scrubbed time is correct regardless of the current scroll offset and zoom level. Pointer-downs on the toolbar, the sticky labels, and the scrollbar do NOT scrub.
- [ ] AC9 — During playback while zoomed in, the view auto-scrolls to keep the playhead on screen; a manual pan during playback stops the auto-scroll until playback restarts.
- [ ] AC10 — Ruler tick labels stay readable at every zoom level (density recomputed on zoom); the drag-to-scrub mapping remains correct after any ruler rebuild.
- [ ] AC11 — The zoom readout shows the current factor as `N.N×`; the slider position reflects the current zoom and vice-versa.
- [ ] AC12 — Empty/degenerate trace (`duration_s ≤ 0`) or a trace that already fits: zoom controls are inert/disabled and the scrollbar thumb is hidden or full-width; nothing throws.
- [ ] AC13 — The transport bar's top seek slider + drift ticks are unchanged (full-duration overview); zooming the dock does not affect them.
- [ ] AC14 — The emitted page stays fully self-contained (passes `assert_self_contained`); `timeline_view.js` is inlined verbatim in its own `<script>`; the page opens on `file://` with no network.
- [ ] AC15 — Returning from full-view (timeline was hidden) and window resizes recompute fit-to-width and clamp zoom/scroll so nothing is left mis-sized.

## UI/UX Specification

The design is fully constrained by the existing CTPS "Instrument" dark console token system (`tokens.css`); no new palette is introduced. There is no separate `…ui.md` sidecar — the visual language is inherited verbatim from the existing dock; only the additive controls below are new. All colors are existing tokens.

### Layout (drawing)

```
.ctps-timeline (panel: --ctps-panel #1e1d1a, border --ctps-border #31302a, radius 6px)
┌──────────────────────────────────────────────────────────── toolbar (right) ─┐
│                                          [−]  ⟨────●────⟩  [+]   [Fit]   2.0× │  .ctps-tl-toolbar
├───────┬───────────────────────────────────────────────────────────────────────┤
│ TASK  │┃ ▓▓▓▓   ░░░░░░   ▓▓▓                                        (scrolls →) │
│ GOTO  │┃ ▓▓ ▓▓▓ ▓                                                              │  #ctps-tl-viewport
│ MAG   │┃  ▬▬▬▬▬▬▬▬                                                             │   (overflow-x:auto,
│ R.BOX │┃  ▬▬                                                                   │    native bar hidden)
│ DRIFT │┃ ▲  ▭▭▭                                                                │  #ctps-tl-content
│ ═════ │┃ 0s    2s    4s    6s    8s   10s   12s ...     (ruler, zoom-aware)    │   width = gutter+gap
│(sticky│┃                                                                       │   + --ctps-content-w
│ gutter│┃                          │ playhead (--ctps-frac)                     │
├───────┴───────────────────────────────────────────────────────────────────────┤
│        ◀───────────[▓▓▓▓▓▓▓▓▓▓]────────────────────────────────▶               │  .ctps-tl-scroll
│                     custom thumb #ctps-tl-thumb (= scrollLeft/scrollWidth)      │   (thumb only)
└─────────────────────────────────────────────────────────────────────────────────┘
     ↑ frozen 52px gutter        ↑ ┃ = viewport left edge; content wider than viewport when zoomed
```

### Description

- **Toolbar** (`.ctps-tl-toolbar`): a thin row at the top of the dock, right-aligned. Buttons reuse the chip/transport look (`--ctps-raised` fill, `--ctps-border-strong` border, `--ctps-text-2` glyphs; active/hover states like the existing `.ctps-transport-btn`/`.ctps-chip`). The slider is a native `<input type="range">` styled minimally against the tokens. The `N.N×` readout uses `--ctps-font-mono`, `--ctps-faint`.
- **Viewport** (`#ctps-tl-viewport`): the horizontal scroll region. `overflow-x: auto; overflow-y: hidden`. Native scrollbar hidden. Contains one content wrapper.
- **Content** (`#ctps-tl-content`): `position: relative; width: calc(var(--ctps-lane-gutter) + var(--ctps-lane-gap) + var(--ctps-content-w))`. Holds `#ctps-ruler`, `#ctps-lanes`, `#ctps-playhead`. This is the element that scrolls.
- **Sticky gutter**: each `.ctps-lane-label` cell is `position: sticky; left: 0; z-index: 2; background: var(--ctps-panel)`, so labels pin to the viewport's left edge while the track scrolls under them. The ruler's empty gutter cell is sticky too.
- **Custom scrollbar** (`.ctps-tl-scroll` + `#ctps-tl-thumb`): a track strip below the viewport; the thumb width = `clientWidth/scrollWidth`, left = `scrollLeft/scrollWidth`, both written via `setProperty`. Thumb fill `--ctps-border-strong`, track `--ctps-raised`; hover brightens. When the content fits (zoom = 1×), the thumb is hidden (or full-width inert).
- **Playhead**: unchanged look (1.5px `--ctps-text` line with glow); now positioned inside `#ctps-tl-content`, so it scrolls/zooms with the track.

### Color codes (all existing tokens — no new values)

| Token | Hex | Use here |
|-------|-----|----------|
| `--ctps-panel` | `#1e1d1a` | dock + sticky-label background |
| `--ctps-raised` | `#26251f` | zoom buttons, scrollbar track |
| `--ctps-border` | `#31302a` | dividers |
| `--ctps-border-strong` | `#3f3d35` | scrollbar thumb, button borders |
| `--ctps-text-2` | `#a5a193` | button glyphs, labels |
| `--ctps-faint` | `#565349` | zoom readout, ruler ticks |
| `--ctps-accent` | `#ffd24a` | slider filled track / focus ring, current-span highlight (existing) |

### Type & spacing tokens

Fonts/spacing/radii all from `tokens.css` (`--ctps-font-ui`, `--ctps-font-mono`, `--ctps-space-*`, `--ctps-radius-chip`). Toolbar buttons ~18–20px square, 9.5px UI font, matching existing chips.

### States

- **Default / Hover / Focus** — buttons and thumb reuse the existing chip/transport hover+focus treatment (accent focus ring).
- **Zoom at fit-to-width (min)** — `−` and `Fit` are disabled/inert; thumb hidden; slider at 0.
- **Zoom at max** — `+` disabled/inert.
- **Follow active (playing)** — no distinct visual; the view simply tracks the playhead.
- **Empty/degenerate trace** — the whole zoom widget + scrollbar are disabled/hidden.

### Responsive

The dock spans the app width. On window/dock resize (and on returning from full-view), fit-to-width is recomputed and the current zoom factor + scroll are re-clamped/re-anchored. Below fit-to-width is never allowed.

## Contracts & Interfaces

Single source of truth for the seams between tasks. Each task conforms to these exact names/shapes.

### DOM contract (owned by T1 in `_BODY_MARKUP`; consumed by T2 CSS, T3 view, T4 timeline, T6 tests)

The `.ctps-timeline` inner markup becomes (ids/classes are the binding contract):

```html
<div class="ctps-timeline">
  <div class="ctps-tl-toolbar">
    <button id="ctps-zoom-out"  class="ctps-zoom-btn" type="button" aria-label="Zoom out">&#8722;</button>
    <input  id="ctps-zoom"      class="ctps-zoom-slider" type="range"
            min="0" max="1000" step="1" value="0" aria-label="Zoom">
    <button id="ctps-zoom-in"   class="ctps-zoom-btn" type="button" aria-label="Zoom in">+</button>
    <button id="ctps-zoom-fit"  class="ctps-zoom-fit" type="button">Fit</button>
    <span   id="ctps-zoom-level" class="ctps-zoom-level">1.0&#215;</span>
  </div>
  <div id="ctps-tl-viewport" class="ctps-tl-viewport">
    <div id="ctps-tl-content" class="ctps-tl-content">
      <div id="ctps-ruler"    class="ctps-ruler"></div>
      <div id="ctps-lanes"    class="ctps-lanes"></div>
      <div id="ctps-playhead" class="ctps-playhead"></div>
    </div>
  </div>
  <div class="ctps-tl-scroll">
    <div id="ctps-tl-thumb" class="ctps-tl-scroll-thumb"></div>
  </div>
</div>
```

- `#ctps-ruler`, `#ctps-lanes`, `#ctps-playhead` ids are **preserved** (existing consumers keep working). They move inside `#ctps-tl-content`.

### CSS custom property (owned by T3 `timeline_view.js`; consumed by T2 CSS + T4 ruler-step)

- `--ctps-content-w` — a CSS length string (e.g. `"640px"`) = the **track column** width (excludes gutter/gap). Written via `.ctps-timeline` element `setProperty('--ctps-content-w', px + 'px')`. Read by:
  - `.ctps-ruler` / `.ctps-lanes` `grid-template-columns: var(--ctps-lane-gutter) var(--ctps-content-w)` (T2),
  - `#ctps-tl-content` `width: calc(var(--ctps-lane-gutter) + var(--ctps-lane-gap) + var(--ctps-content-w))` (T2),
  - `.ctps-playhead` `left: calc(var(--ctps-lane-gutter) + var(--ctps-lane-gap) + var(--ctps-frac,0) * var(--ctps-content-w))` (T2),
  - `timeline.js` ruler-step compute reads it back with `getComputedStyle(el).getPropertyValue('--ctps-content-w')` (T4).
- Existing `--ctps-frac` (owned by `transport.js`) is unchanged. `--ctps-lane-gutter` (52px) and `--ctps-lane-gap` are unchanged.

### JS API: `CTPS.timelineView` (owned by T3 `timeline_view.js`; consumed by T5 `transport.js` keymap)

```
CTPS.timelineView = {
  zoomIn(),    // one step in, playhead-anchored; no-op at max
  zoomOut(),   // one step out, playhead-anchored; no-op at fit
  fit(),       // reset to fit-to-width (1x) + scroll to 0
}
```

Assigned during `timeline_view.js`'s `init(ctx)`. `transport.js` calls each guarded (`if (CTPS.timelineView && typeof CTPS.timelineView.zoomIn === 'function') …`), exactly like it already guards `CTPS.help.open`.

### JS API: `CTPS.timeline.rebuildRuler` (owned by T4 `timeline.js`; consumed by T3 `timeline_view.js`)

```
CTPS.timeline = {
  rebuildRuler(pxPerSec),  // repaint ruler tick children at a step chosen for the given
                           // pixels-per-second (readable on-screen density). Preserves the
                           // stable #ctps-ruler-track element so drag-to-scrub keeps its ref.
}
```

- `timeline.js` gives `.ctps-ruler-track` the id `ctps-ruler-track` and builds it ONCE (stable); `rebuildRuler` clears only its tick children and re-appends. `timeline_view.js` calls `rebuildRuler(currentPxPerSec)` after each committed zoom change.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `tools/path_planner/ctps/player_html.py` | T1 | T6 |
| `tools/path_planner/ctps/assets/components.css` | T2 | — |
| `tools/path_planner/ctps/assets/layout.css` | T2 | — |
| `tools/path_planner/ctps/assets/timeline_view.js` (new) | T3 | — |
| `tools/path_planner/ctps/assets/timeline.js` | T4 | — |
| `tools/path_planner/ctps/assets/transport.js` | T5 | — |
| `tools/path_planner/tests/test_ctps_player_html.py` | T6 | — |

No two tasks edit the same file.

### Naming (must match across tasks)

- Ids: `ctps-tl-viewport`, `ctps-tl-content`, `ctps-tl-thumb`, `ctps-ruler-track`, `ctps-zoom`, `ctps-zoom-in`, `ctps-zoom-out`, `ctps-zoom-fit`, `ctps-zoom-level`.
- Classes: `ctps-tl-toolbar`, `ctps-tl-viewport`, `ctps-tl-content`, `ctps-tl-scroll`, `ctps-tl-scroll-thumb`, `ctps-zoom-btn`, `ctps-zoom-slider`, `ctps-zoom-fit`, `ctps-zoom-level`.
- Custom property: `--ctps-content-w`.
- Keymap keys: `+`, `=`, `-`, `0`.

## Zoom / pan math (Notes for `timeline_view.js`, T3)

Let `dur = track.duration_s`, `gutter = --ctps-lane-gutter (52px)`, `gap = --ctps-lane-gap`.

- **Track view width**: `trackView = viewport.clientWidth - gutter - gap` (the visible track area right of the sticky gutter). Recompute on init + resize.
- **Fit**: `fitContentW = max(1, trackView)`; `pxPerSecFit = fitContentW / dur` (guard `dur > 0`). At zoom `z ≥ 1`, `contentW = fitContentW * z`, written to `--ctps-content-w`. `z = 1` ⇒ content exactly fills the track view (no scroll).
- **Max zoom**: `MIN_VISIBLE_SECONDS = 1.5`; `zoomMax = clamp(dur / MIN_VISIBLE_SECONDS, 1, ABSOLUTE_MAX)` with `ABSOLUTE_MAX = 200`. If `dur ≤ MIN_VISIBLE_SECONDS` (already fits densely) ⇒ `zoomMax = 1` ⇒ zoom disabled.
- **Slider mapping**: slider `[0..1000]` ↔ `z ∈ [1 .. zoomMax]` on a log scale: `z = zoomMax^(v/1000)`; inverse `v = 1000 * log(z)/log(zoomMax)`.
- **Button/key step**: multiply/divide `z` by `1.5` per press, clamped to `[1, zoomMax]`.
- **Playhead-anchored zoom** (buttons/slider/keys): let `frac = clamp(store.t / dur, 0, 1)`, `oldPlayheadX = gutter + gap + frac*oldContentW` (content coords). After setting `contentW`, `newScrollLeft = (gutter + gap + frac*newContentW) - (oldPlayheadX - oldScrollLeft)`. Clamp to `[0, scrollWidth - clientWidth]`.
- **Cursor-anchored zoom** (Ctrl/⌘+wheel): `cursorContentX = oldScrollLeft + (evt.clientX - viewportRect.left)`; `cursorFrac = clamp((cursorContentX - gutter - gap) / oldContentW, 0, 1)`. After zoom, `newScrollLeft = (gutter + gap + cursorFrac*newContentW) - (evt.clientX - viewportRect.left)`. Clamp.
- **Pan**: `viewport.scrollLeft = x` (a DOM property, not an inline style). Native `scroll` event → update thumb via `setProperty`. Thumb drag → set `scrollLeft`. Vertical wheel (no Ctrl) over the viewport → `scrollLeft += evt.deltaY`, `preventDefault`.
- **Follow** (subscribe `["t"]`): while `store.playing` and not suspended, if the playhead's viewport-x is within `EDGE_MARGIN` of the right edge or left of the visible track, page the view so the playhead sits at ~⅓ from the left. A manual pan sets `followSuspended = true`; re-arm on `playing` false→true, on `fit()`, and on transport Home.
- **Resize** (`ResizeObserver` on `#ctps-tl-viewport`, coalesced on rAF, + `window` resize fallback — mirror `renderer.js`): recompute `trackView`/`fitContentW`, re-apply the current `z` (clamped to the new `zoomMax`), re-anchor scroll, resync slider/readout/thumb, and `CTPS.timeline.rebuildRuler(pxPerSec)`.
- **Guards**: `dur ≤ 0` or `zoomMax ≤ 1` ⇒ set `contentW = fitContentW`, disable slider/buttons (`.disabled` + a `--disabled` class), hide the thumb; never throw.

## Testing Strategy

**Levels:** Unit (Python emitter) + manual integration (served HTML eyeball). No JS unit runner exists for the player.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `render()` emits the new timeline markup | Unit | HTML contains `id="ctps-tl-viewport"`, `id="ctps-tl-content"`, `id="ctps-tl-thumb"`, `id="ctps-zoom"`, `id="ctps-zoom-in"`, `id="ctps-zoom-out"`, `id="ctps-zoom-fit"`, `id="ctps-zoom-level"`, `class="ctps-tl-toolbar"`, `class="ctps-tl-scroll"`, and still `class="ctps-timeline"`. |
| TC2 | `timeline_view.js` registered + inlined | Unit | `timeline_view.js` is in `_ASSET_JS`; its exact on-disk content appears verbatim in the page (extend/rely on existing TC-ASSETS loop). |
| TC3 | Preserved ids still present | Unit | `id="ctps-ruler"`, `id="ctps-lanes"`, `id="ctps-playhead"` still in the HTML (moved, not removed). |
| TC4 | Self-containment holds | Unit | `render()` on a real map still passes `assert_self_contained`; no forbidden substring in the new JS/CSS. (existing TC-LINT-neg stays green.) |
| TC5 | Contract markers present in assets | Unit | `--ctps-content-w` appears in both `components.css` and `timeline_view.js`; `CTPS.timelineView` appears in `timeline_view.js`; `rebuildRuler` appears in `timeline.js` and `timeline_view.js`. |
| TC6 | Existing structural + round-trip tests | Unit | The full `test_ctps_player_html.py` suite (TC-STRUCT, TRACK round-trip, PLAYER_META lanes, TC-ASSETS) stays green. |
| TC7 | Long-trace zoom in/out | Manual | Serve a long-duration trace; `+`/`−`/slider/`Ctrl`+wheel change zoom; content widens/narrows; scrollbar thumb resizes; never below fit. |
| TC8 | Playhead + cursor anchoring | Manual | Button zoom keeps playhead at the same screen x; `Ctrl`+wheel keeps the point under the cursor fixed. |
| TC9 | Pan sync + scrub coexistence | Manual | Thumb drag / vertical wheel pans ruler+lanes+playhead in sync; dragging the lanes still scrubs to the correct time at any scroll/zoom; toolbar/labels/scrollbar clicks don't scrub. |
| TC10 | Follow + fit + keys | Manual | Playing while zoomed auto-scrolls to keep the playhead visible; manual pan suspends it; `0`/`Fit` resets; `+`/`=`/`−` keys zoom. |
| TC11 | Degenerate + resize | Manual | Empty trace: controls inert, no throw. Toggle full-view off/on and resize the window: fit recomputes, nothing mis-sized. |

**Test data:** an existing trace fixture with a non-trivial duration — e.g. `tools/path_planner/tests/fixtures/ctps_lrs_track.json` for emitter tests; `ctps_cargo_trace.json` (via `player_html.render`, see the cargo demo recipe) or a freshly built long mission trace for the manual eyeball, rendered to a scratchpad `.html` and served with `python -m http.server` on `127.0.0.1` (claude-in-chrome blocks `file://`).

**Run command:** `uv run --extra dev pytest tools/path_planner/tests/test_ctps_player_html.py` (Python); manual serve + browser for TC7–TC11.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T3 | New `timeline_view.js` zoom/pan controller | — | high | `tools/path_planner/ctps/assets/timeline_view.js` | Create the module (own `<script>`, IIFE, `"use strict"`, vanilla JS, ADR-053 setProperty-only geometry). Register `init(ctx)` via `CTPS._inits`. In init, query the DOM contract elements; compute `trackView`/`fitContentW`/`zoomMax` per the Zoom/pan math section; own zoom state, the custom scrollbar thumb (width/left via `setProperty`, thumb drag → `scrollLeft`, native `scroll` → thumb sync), `Ctrl`/⌘+wheel (cursor-anchored zoom) + plain wheel (horizontal pan, `preventDefault`), the zoom slider/`+`/`−`/`Fit` widget wiring, playhead-anchored zoom, follow-the-playhead (subscribe `["t"]`, suspend on manual pan), and a `ResizeObserver` (coalesced on rAF, mirror renderer.js) + window-resize fallback. Set `--ctps-content-w` on `.ctps-timeline` via `setProperty`; after each committed zoom call `CTPS.timeline.rebuildRuler(pxPerSec)`. Expose `CTPS.timelineView = { zoomIn, zoomOut, fit }`. Guard `dur ≤ 0` / already-fits (disable controls, hide thumb, never throw). Break the work into named helpers (fit compute, applyZoom, anchor math, scrollbar sync, follow, wheel). Satisfies AC1–AC12, AC15. Do NOT edit any other file — this task creates only this file. |
| T1 | Restructure timeline markup + register asset | T3 | med | `tools/path_planner/ctps/player_html.py` | In `_BODY_MARKUP`, replace the `.ctps-timeline` inner block with the DOM contract markup (toolbar + viewport + content wrapping the preserved `#ctps-ruler`/`#ctps-lanes`/`#ctps-playhead` + custom scrollbar). Use numeric HTML entities for glyphs (`&#8722;` minus, `&#215;` ×), class-based only, no inline styles, no forbidden substrings. Add `_ASSETS_DIR / "timeline_view.js"` to `_ASSET_JS` immediately AFTER `timeline.js` and BEFORE `panels.js`. Do not touch `PLAYER_META`. Satisfies AC13 (leaves the transport seek slider untouched), AC14. |
| T2 | Timeline dock CSS: viewport, sticky gutter, scrollbar, zoom widget, playhead calc | — | med | `tools/path_planner/ctps/assets/components.css`, `tools/path_planner/ctps/assets/layout.css` | components.css: make `.ctps-timeline` a vertical flex (toolbar / viewport / scrollbar rows); `.ctps-tl-toolbar` right-aligned flex row; `.ctps-tl-viewport { overflow-x:auto; overflow-y:hidden; scrollbar-width:none }` + `.ctps-tl-viewport::-webkit-scrollbar { display:none }`; `#ctps-tl-content { position:relative; width: calc(var(--ctps-lane-gutter) + var(--ctps-lane-gap) + var(--ctps-content-w)) }`; change `.ctps-ruler`/`.ctps-lanes` `grid-template-columns` from `... 1fr` to `var(--ctps-lane-gutter) var(--ctps-content-w)`; make `.ctps-lane-label { position:sticky; left:0; z-index:2; background:var(--ctps-panel) }`; update `.ctps-playhead` `left` calc to the content-relative form in Contracts (drop the old ±2×padding basis); style `.ctps-tl-scroll` track + `#ctps-tl-thumb` (width/left via custom props the JS writes), `.ctps-zoom-btn`/`.ctps-zoom-slider`/`.ctps-zoom-fit`/`.ctps-zoom-level` reusing existing tokens + a `--disabled` treatment. layout.css: ensure `.ctps-timeline` height stays auto and the viewport doesn't grow unbounded (min-width:0). Provide `--ctps-content-w` fallback so pre-hydration render isn't broken (e.g. default via the JS on init; CSS may assume it's set). Satisfies AC6, and the visual half of AC1–AC11. Do NOT edit JS. |
| T4 | timeline.js: stable ruler track + `rebuildRuler` + scrub rescope | — | med | `tools/path_planner/ctps/assets/timeline.js` | Give the ruler track element the id `ctps-ruler-track`. Split ruler building so the `.ctps-ruler-track` (and gutter cell) are created ONCE and stay stable; add `rebuildRuler(pxPerSec)` that clears only the tick children and repaints at a step chosen so ticks are ~90–120px apart on screen for the given `pxPerSec` (reuse `TICK_STEP_CANDIDATES`; compute from `--ctps-content-w`/duration). Expose `CTPS.timeline = { rebuildRuler }`. Rescope `wireScrub` to bind on `#ctps-tl-viewport` (not the whole dock) and ignore pointer-downs whose target is inside a `.ctps-lane-label`, `.ctps-tl-toolbar`, or `.ctps-tl-scroll`; keep resolving the fraction from the stable `#ctps-ruler-track` rect (works at any scroll/zoom). Ensure `buildLanesDom`/`renderLaneItems`/`pctOf` are otherwise unchanged and now render inside `#ctps-tl-content`. Update the file docstring for the new structure. Satisfies AC8, AC10. Do NOT edit other files. |
| T5 | transport.js: zoom keymap | — | low | `tools/path_planner/ctps/assets/transport.js` | In `onKeyDown`, add cases for `+`, `=`, `-`, `0` that call `CTPS.timelineView.zoomIn()` / `zoomIn()` / `zoomOut()` / `fit()` respectively, each guarded like `openHelp` (`if (CTPS.timelineView && typeof …==='function')`), with `preventDefault()`. Keep the existing `ArrowLeft/Right`-for-range and ctrl/meta/alt and help-open guards intact; `-`/`+`/`0`/`=` must not fire while a text input is focused or the help modal is open. Update the module header's key list. Satisfies AC2, AC5. Do NOT edit other files. |
| T6 | Emitter structural + contract tests | T1, T3 | low | `tools/path_planner/tests/test_ctps_player_html.py` | Add tests: TC1 (new markup ids/classes present), TC3 (preserved ids), TC5 (contract markers `--ctps-content-w` in components.css + timeline_view.js, `CTPS.timelineView` in timeline_view.js, `rebuildRuler` in timeline.js + timeline_view.js). Confirm the existing TC-ASSETS loop now also covers `timeline_view.js` (TC2) and TC-STRUCT/round-trip/TC-LINT stay green (TC4, TC6). Run `uv run --extra dev pytest tools/path_planner/tests/test_ctps_player_html.py`. Satisfies AC14 and pins the DOM/asset contract. |
| T7 | Serve + eyeball verification (long trace) | T1, T2, T3, T4, T5 | low | (scratchpad only — no repo files) | Render a long-duration trace to a scratchpad `.html` via `player_html.render(...)` (use `ctps_cargo_trace.json` or a built long mission trace; the cargo demo recipe), serve with `python -m http.server` on `127.0.0.1`, and walk TC7–TC11 by hand (zoom in/out via buttons/slider/`Ctrl`-wheel/keys; playhead + cursor anchoring; thumb + wheel pan in sync; scrub still correct; follow during playback; `Fit`/`0`; empty-trace + resize + full-view-return). Capture a screenshot. Report any AC that fails. |

## Notes for Implementer

- **ADR-053 is law here:** JS writes geometry ONLY via `setProperty` (custom props / `left` / `width`) — never `el.style.<prop> = …`. `viewport.scrollLeft = x` is a DOM property, not an inline style, so it is allowed. Do not introduce any of the forbidden substrings (`src=`, `href=`, `http://`, `https://`, `data:image`, `</script>`) in the new JS/CSS — the emit-time lint will reject the whole page.
- **Stable ruler track element is critical:** `wireScrub` captures the ruler track's rect. If `rebuildRuler` recreated the track element, the scrub reference would go stale (a known-class gotcha). Rebuild only the tick CHILDREN; keep `#ctps-ruler-track` itself.
- **Playhead containing block:** the playhead must be a child of `#ctps-tl-content` (a positioned, content-width element that is itself scrolled) so it pans with the track. An absolute child of the *viewport* would NOT scroll — do not anchor it there.
- **Sticky label opacity:** labels need an opaque `--ctps-panel` background and `z-index:2` or spans will bleed through them while scrolled. The playhead never enters the gutter (its left starts at `gutter+gap`), so playhead `z-index:3` over labels is fine.
- **Preserve percent math:** do NOT convert spans/pins/ticks to pixel positioning. `pctOf` stays; the only change is the track column is now `--ctps-content-w` wide instead of `1fr`.
- **Init ordering:** `timeline_view.js` runs after `timeline.js` (its `_ASSET_JS` position), so `CTPS.timeline.rebuildRuler` and the built DOM exist when `timeline_view`'s init runs. `boot.js` stays last.
- **Follow vs scrub:** follow only auto-scrolls during playback; the existing drag-to-scrub gesture is unchanged except for the viewport rescope. A scrub-drag is not a "manual pan" for follow-suspend purposes (it moves the playhead, not the view).
- **Verify on a genuinely long trace** — the placeholder field's default mission may be short; build or reuse a long trace so zoom actually overflows the viewport. (`field_data` is a test map; verify the built behavior anyway.)
- **Rollback:** the change is additive and contained to the player's timeline dock + one new asset; reverting the 6 edited/created files restores the prior full-width dock. No data, trace, or firmware surface is touched.
```
