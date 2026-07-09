# CTPS Timeline Zoom + Move Bar Plan

**Goal:** Make the CTPS player's bottom timeline dock navigable like a real video/DAW editor — a zoom control and a horizontal "move bar" (pan) — so a long trace is readable instead of cramming every span/pin into the full-width `0..duration` track.

**Approach:** Keep the proven percent-of-track positioning of spans/pins/ticks unchanged. Nest the ruler + all lanes + playhead inside ONE horizontal scroll viewport whose inner content track width = `duration × pixels-per-second`, driven by a single CSS custom property `--ctps-content-w`. Restructure the dock body into a vertical stack of full-content-width **flex rows** (one per lane, plus the ruler row); each row's label cell is `position: sticky; left: 0`, so labels pin correctly (the row is the label's full-width containing block) while the track scrolls under them. A new asset `timeline_view.js` owns zoom state, the custom themed scrollbar, wheel/keyboard/follow behavior, and re-anchoring; `timeline.js` keeps owning lane/ruler DOM (now building rows, with stable per-row track elements and a windowed `rebuildRuler` hook for zoom-aware tick density). Chosen structural approach: **Scroll viewport + CSS content width** (Phase 4).

## Scope

- **In scope:**
  - A zoom widget docked top-right of the timeline dock: `−` / `+` buttons, a zoom slider, a `Fit` button, and a live `N.N×` readout.
  - `Ctrl`/`⌘` + mouse-wheel over the timeline to zoom, anchored at the cursor.
  - Keyboard: `+`/`=` zoom in, `-` zoom out, `0` fit-to-width (wired into the existing transport keymap); the help modal's shortcut list is updated to match.
  - A custom themed horizontal scrollbar (thumb) below the lanes for panning; consistent on Windows/macOS. Native scroll gestures (trackpad, shift-wheel) still work with the native scrollbar hidden; plain vertical-wheel over the viewport pans horizontally.
  - Playhead lives inside the scrolling content and pans/zooms with it; the label gutter (TASK/GOTO/…) stays visually pinned.
  - Follow-the-playhead during playback (auto-scroll to keep the playhead visible), with manual pan temporarily suspending follow.
  - Zoom range: minimum = fit-to-width (never zoom out past the whole trace), maximum capped so a short window (~1.5 s) fills the view; default on load = fit-to-width (1×).
  - Zoom-aware, windowed ruler: tick density recomputed on zoom so tick spacing stays readable, and only ticks near the visible window are rendered (bounded node count on long traces).
  - Emitter structural-marker tests for the new DOM + asset registration.
- **Out of scope:**
  - The CTPS **layout editor** (`layout_editor_html.py` / `editor.css` / `editor_*.js`). It emits its OWN asset set and does not inline the player's `timeline.js` / `components.css` / `layout.css`, so these changes do not reach it. (Verified: `layout_editor_html.py` has its own `_ASSET_JS`/`_ASSET_CSS` tuples.)
  - The transport bar's top full-width seek slider and its red drift ticks — they stay a full-duration overview (the always-visible minimap) and do NOT zoom.
  - Any change to `track_builder`, the recorded trace JSON, `PLAYER_META`'s lane list, or the cargo/scene model.
  - Vertical zoom / lane-height resizing, per-lane collapse, timeline row reordering.
  - A minimap overview strip (considered, not chosen).

## Decisions

- **Structure: one scroll viewport, per-row flex, sticky gutter** — the dock body is a vertical stack of `.ctps-tl-row` flex rows inside `#ctps-tl-content` (width = gutter + gap + `--ctps-content-w`). Each row spans the full content width and holds `[.ctps-lane-label (sticky left:0)] [track element (width = --ctps-content-w)]`. The ruler is the first row. The playhead is a single absolute child of `#ctps-tl-content` spanning all rows. **Why rows, not the old 2-column grid:** a grid item's containing block is its grid area, so a 52px label in a 52px column has zero sticky travel and would scroll off. A full-content-width flex row gives its sticky label full horizontal travel, and label/track auto-align as flex siblings — while preserving the existing `gutter + gap + content-w` width formula so all zoom/scroll anchor math is unchanged. (Fable critic C2; judge ruled CRITIC_WINS and confirmed this fix over the fixed-column-outside alternative.)
- **Percent positioning preserved** — spans/pins/ticks stay positioned as a percent of their track element, which is now `--ctps-content-w` wide. `pctOf` / `renderLaneItems` / `appendTick` are unchanged (Approach B pixel-recompute rejected).
- **New asset over bloating timeline.js** — zoom/pan is a distinct concern; it goes in a new `timeline_view.js` (registered in `_ASSET_JS` after `timeline.js`, before `panels.js`), matching ADR-053's modular-asset design.
- **`--ctps-content-w` as the single width knob** — one custom property (a CSS length, e.g. `"640px"`) set via `setProperty` on `.ctps-timeline` drives the row track width and the playhead calc. ADR-053 setProperty carve-out (same as span `left`/`width`), not an inline-style violation. CSS supplies a fallback (`var(--ctps-content-w, 100%)` for row tracks, `var(--ctps-content-w, 0px)` inside the content-width calc) so if `timeline_view.js` ever throws, the dock degrades to a full-width, non-zooming timeline instead of collapsing — preserving boot.js's "one bad module can't take down the page" posture.
- **Native scroll kept, native scrollbar hidden** — `overflow-x: auto` on the viewport so trackpad/shift-wheel momentum still works; the native scrollbar is visually hidden (`scrollbar-width: none` + `::-webkit-scrollbar { display: none }`) and a custom thumb synced to `scrollLeft` (via `--ctps-tl-thumb-w` / `--ctps-tl-thumb-x`) is the visible control.
- **Zoom anchoring** — button/slider/keyboard zoom keeps the playhead fixed on screen; `Ctrl`/`⌘`+wheel keeps the point under the cursor fixed. Both zoom paths `preventDefault` (wheel listeners registered `{ passive: false }`) so the browser never page-zooms alongside.
- **Follow re-arm policy** — `timeline_view.js` subscribes to `["t","playing"]`. Follow is active while `store.playing`; a manual pan during playback suspends it for the rest of that run; it re-arms on the `playing` false→true edge and on `fit()`. (No "Home" re-arm — Home is just `store.seek(0)` with no distinguishing signal.) A manual pan is detected from user-input handlers (wheel-pan, thumb drag) plus a "not-self-initiated" guard on the native `scroll` event, so the module's own programmatic paging never falsely suspends follow.
- **CSS ownership split** — layout/position/flex scaffolding for `.ctps-timeline` and its rows lives in `layout.css` (which owns the page-container layout); the panel *look* (colors, borders, scrollbar/thumb, zoom widget) lives in `components.css`. Neither file restates a property the other owns.
- **No new ADR** — this is a feature inside the player, not a project-wide rule; it extends ADR-053's conventions (modular assets + setProperty carve-out). The `timeline.js` docstring and this plan record the design.
- **Verification** — Python emitter tests (structural markers, self-containment, asset inclusion) plus serving the emitted HTML over `127.0.0.1` (`python -m http.server`) and eyeballing/screenshotting on a long-duration trace. There is no JS unit-test runner for the player.

## Acceptance Criteria

- [ ] AC1 — On load, a trace fits entirely in the dock width at 1× (fit-to-width); no horizontal scroll appears until the user zooms in (`Math.floor` the fit width so a 1px rounding surplus never shows a thumb at 1×).
- [ ] AC2 — Clicking `+` / pressing `+`/`=` zooms in; the content track widens, spans/pins spread out, and the scrollbar thumb shrinks accordingly. `−` reverses it. Zoom never goes below fit-to-width.
- [ ] AC3 — `Ctrl`/`⌘` + wheel over the timeline zooms; the trace point under the cursor stays under the cursor after the zoom; the browser does not page-zoom.
- [ ] AC4 — Button/slider/keyboard zoom keeps the playhead at the same on-screen x position before and after the zoom step.
- [ ] AC5 — When zoomed in, the `Fit` button and the `0` key both reset to fit-to-width (1×) and scroll to the start. (At fit-to-width the Fit control is already inert.)
- [ ] AC6 — The lane-label column (TASK/GOTO/MAGAZINE/REAR BOX/DRIFT) stays visually pinned on the left while the track is scrolled at any zoom; labels are opaque (no spans bleed through) and sit above the playhead line.
- [ ] AC7 — Dragging the custom scrollbar thumb pans the ruler + all lanes + playhead together, perfectly in sync; plain vertical-wheel over the viewport also pans horizontally; native trackpad horizontal (`deltaX`) gestures still pan.
- [ ] AC8 — Dragging on the lane/ruler track still scrubs the playhead (the existing gesture), and the scrubbed time is correct regardless of the current scroll offset and zoom level. Pointer-downs on the toolbar, the sticky labels, and the scrollbar do NOT scrub.
- [ ] AC9 — During playback while zoomed in, the view auto-scrolls to keep the playhead on screen; a manual pan during playback stops the auto-scroll until playback restarts.
- [ ] AC10 — Ruler tick labels stay readable at every zoom level (density recomputed on zoom) and only ticks near the visible window are rendered (bounded count); the drag-to-scrub mapping stays correct after any ruler rebuild.
- [ ] AC11 — The zoom readout shows the current factor as `N.N×`; moving the slider updates zoom and readout, and button/key/wheel zoom moves the slider — two-way in sync.
- [ ] AC12 — Empty/degenerate trace (`duration_s ≤ 0`) or a trace that already fits: the zoom controls are disabled and the scrollbar thumb is hidden; nothing throws.
- [ ] AC13 — The transport bar's top seek slider + drift ticks are unchanged (full-duration overview); zooming the dock does not move or resize them.
- [ ] AC14 — The emitted page stays fully self-contained (passes `assert_self_contained`); `timeline_view.js` is inlined verbatim in its own `<script>`; the page opens on `file://` with no network.
- [ ] AC15 — Returning from full-view (timeline was hidden) and window resizes recompute fit-to-width and clamp zoom/scroll so nothing is left mis-sized.

## UI/UX Specification

The design is fully constrained by the existing CTPS "Instrument" dark console token system (`tokens.css`); no new palette is introduced. There is no separate `…ui.md` sidecar — the visual language is inherited verbatim from the existing dock; only the additive controls below are new.

### Layout (drawing)

```
.ctps-timeline (panel: --ctps-panel #1e1d1a, border --ctps-border #31302a, radius 6px; vertical flex)
┌──────────────────────────────────────────────────────────── toolbar (right) ─┐
│                                          [−]  ⟨────●────⟩  [+]   [Fit]   2.0× │  .ctps-tl-toolbar
├────────────────────────────────────────────────────────────────────────────── ┤
│ #ctps-tl-viewport  (overflow-x:auto, native scrollbar hidden) │                │
│ ┌─────┬──────────────────────────────────────────────────────── scrolls → ──┐ │
│ │ ═   │ 0s    2s    4s    6s    8s   10s   12s ...   (ruler row, windowed)   │ │  ← first .ctps-tl-row
│ │TASK │ ▓▓▓▓   ░░░░░░   ▓▓▓                                                  │ │  ← one .ctps-tl-row / lane
│ │GOTO │ ▓▓ ▓▓▓ ▓                                                             │ │
│ │ MAG │  ▬▬▬▬▬▬▬▬                                                            │ │
│ │R.BOX│  ▬▬                                                                  │ │
│ │DRIFT│ ▲  ▭▭▭                                                              │ │
│ └─────┴───────────────────────│ playhead (--ctps-frac, absolute, spans rows)─┘ │
│  ↑ sticky label cell (flex 0 0 52px, position:sticky left:0, opaque)           │
├──────────────────────────────────────────────────────────────────────────────┤
│        ◀───────────[▓▓▓▓▓▓▓▓▓▓]────────────────────────────────▶               │  .ctps-tl-scroll
│                     custom thumb #ctps-tl-thumb (w/x = scroll ratios)           │
└────────────────────────────────────────────────────────────────────────────────┘
   ┃ each .ctps-tl-row is display:flex, width = gutter+gap+--ctps-content-w (wider than viewport when zoomed)
```

### Description

- **Toolbar** (`.ctps-tl-toolbar`): a thin row at the top of the dock, right-aligned. Buttons reuse the chip/transport look (`--ctps-raised` fill, `--ctps-border-strong` border, `--ctps-text-2` glyphs; hover/focus/active states like the existing `.ctps-transport-btn`/`.ctps-chip`). The slider is a native `<input type="range">` styled minimally against the tokens. The `N.N×` readout uses `--ctps-font-mono`, `--ctps-faint`.
- **Viewport** (`#ctps-tl-viewport`): the horizontal scroll region. `overflow-x: auto; overflow-y: hidden`. Native scrollbar hidden. Contains one content wrapper.
- **Content** (`#ctps-tl-content`): `position: relative; width: calc(var(--ctps-lane-gutter) + var(--ctps-lane-gap) + var(--ctps-content-w, 0px))`. Holds the `.ctps-tl-row` stack + `#ctps-playhead`. This element is what scrolls.
- **Rows** (`.ctps-tl-row`): `display: flex; width: 100%` (i.e. the full content width). Child 1 = `.ctps-lane-label` (`flex: 0 0 var(--ctps-lane-gutter); position: sticky; left: 0; z-index: 3; background: var(--ctps-panel)`). Child 2 = the track element (`.ctps-lane` for lanes / `#ctps-ruler-track` for the ruler), `flex: 0 0 var(--ctps-content-w, 100%)`, the positioning context for its percent-placed children. Because the row spans the full content width, the sticky label has full horizontal travel and stays pinned + opaque while scrolled.
- **Custom scrollbar** (`.ctps-tl-scroll` + `#ctps-tl-thumb`): a track strip below the viewport; the thumb width = `--ctps-tl-thumb-w` (= `clientWidth/scrollWidth`), left = `--ctps-tl-thumb-x` (= `scrollLeft/scrollWidth`), both written via `setProperty`. Thumb fill `--ctps-border-strong`, track `--ctps-raised`; hover brightens. When the content fits (zoom = 1×), the thumb is hidden.
- **Playhead**: unchanged look (1.5px `--ctps-text` line with glow); now an absolute child of `#ctps-tl-content` at `z-index: 2` (below the sticky labels so it never paints over them), so it scrolls/zooms with the track.

### Color codes (all existing tokens — no new values)

| Token | Hex | Use here |
|-------|-----|----------|
| `--ctps-panel` | `#1e1d1a` | dock + sticky-label background |
| `--ctps-raised` | `#26251f` | zoom buttons, scrollbar track |
| `--ctps-border` | `#31302a` | dividers |
| `--ctps-border-strong` | `#3f3d35` | scrollbar thumb, button borders |
| `--ctps-text-2` | `#a5a193` | button glyphs, labels |
| `--ctps-faint` | `#565349` | zoom readout, ruler ticks |
| `--ctps-accent` | `#ffd24a` | slider filled track / focus ring |

### Type & spacing tokens

Fonts/spacing/radii all from `tokens.css` (`--ctps-font-ui`, `--ctps-font-mono`, `--ctps-space-*`, `--ctps-radius-chip`). Toolbar buttons ~18–20px square, 9.5px UI font (matching the lane labels; note `.ctps-chip` is 10px — use the label size for these).

### States

- **Default / Hover / Focus** — buttons and thumb reuse the existing chip/transport hover+focus treatment (accent focus ring).
- **Zoom at fit-to-width (min)** — `−` and `Fit` are disabled (`disabled` attr + a `ctps-tl--nozoom` class treatment where relevant); thumb hidden; slider at 0.
- **Zoom at max** — `+` disabled.
- **Follow active (playing)** — no distinct visual; the view simply tracks the playhead.
- **Empty/degenerate trace** — the whole zoom widget + scrollbar are disabled/hidden (`ctps-tl--nozoom` on `.ctps-timeline`).

### Responsive

The dock spans the app width. On window/dock resize (and on returning from full-view), fit-to-width is recomputed and the current zoom factor + scroll are re-clamped/re-anchored. Below fit-to-width is never allowed.

## Contracts & Interfaces

Single source of truth for the seams between tasks. Each task conforms to these exact names/shapes.

### DOM contract (owned by T1 in `_BODY_MARKUP`; consumed by T2 CSS, T3 view, T4 timeline, T6 tests)

The `.ctps-timeline` inner markup becomes (ids/classes are the binding contract). Note: the `.ctps-tl-row` **rows** are built by `timeline.js` (T4) at runtime; the static markup ships the toolbar, viewport, empty content wrapper (with the preserved `#ctps-ruler`/`#ctps-lanes` containers T4 fills), playhead, and scrollbar:

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
      <div id="ctps-ruler"    class="ctps-ruler"></div>     <!-- T4 builds the ruler row inside -->
      <div id="ctps-lanes"    class="ctps-lanes"></div>     <!-- T4 builds one .ctps-tl-row / lane inside -->
      <div id="ctps-playhead" class="ctps-playhead"></div>
    </div>
  </div>
  <div class="ctps-tl-scroll">
    <div id="ctps-tl-thumb" class="ctps-tl-scroll-thumb"></div>
  </div>
</div>
```

- `#ctps-ruler`, `#ctps-lanes`, `#ctps-playhead` ids are **preserved**; they move inside `#ctps-tl-content`. T4 appends a `.ctps-tl-row` for the ruler inside `#ctps-ruler` (or makes `#ctps-ruler` itself the ruler row) and one `.ctps-tl-row` per lane inside `#ctps-lanes`.

### CSS custom properties (owned by T3 `timeline_view.js` via `setProperty`; consumed by T2 CSS)

- `--ctps-content-w` — a CSS length string (e.g. `"640px"`) = the **track element** width (excludes gutter/gap). Written on the `.ctps-timeline` element. Read by: the `.ctps-lane` / `#ctps-ruler-track` `flex-basis`/`width`, and `#ctps-tl-content` width calc, and `.ctps-playhead` `left` calc, all in T2. Every CSS use supplies a fallback (`var(--ctps-content-w, 100%)` for track basis; `var(--ctps-content-w, 0px)` inside calc) so a pre-hydration or failed-init state degrades to a full-width, non-zooming dock.
- `--ctps-tl-thumb-w`, `--ctps-tl-thumb-x` — CSS percentages for the custom scrollbar thumb width and left, written on `#ctps-tl-thumb`. Consumed by T2's `.ctps-tl-scroll-thumb` rule.
- Existing `--ctps-frac` (owned by `transport.js`), `--ctps-lane-gutter` (52px), `--ctps-lane-gap` are unchanged.
- `.ctps-playhead` `left` (T2) = `calc(var(--ctps-lane-gutter) + var(--ctps-lane-gap) + var(--ctps-frac,0) * var(--ctps-content-w, 0px))`, measured inside `#ctps-tl-content`.

### JS API: `CTPS.timelineView` (owned by T3 `timeline_view.js`; consumed by T5 `transport.js` keymap)

```
CTPS.timelineView = {
  zoomIn(),    // one step in, playhead-anchored; no-op at max
  zoomOut(),   // one step out, playhead-anchored; no-op at fit
  fit(),       // reset to fit-to-width (1x) + scroll to 0 + re-arm follow
}
```

Assigned during `timeline_view.js`'s `init(ctx)`. `transport.js` calls each guarded (`if (CTPS.timelineView && typeof CTPS.timelineView.zoomIn === 'function') …`), exactly like it already guards `CTPS.help.open`.

### JS API: `CTPS.timeline.rebuildRuler` (owned by T4 `timeline.js`; consumed by T3 `timeline_view.js`)

```
CTPS.timeline = {
  rebuildRuler(pxPerSec, viewT0, viewT1),
    // Repaint ONLY the ruler tick children whose time falls within [viewT0, viewT1]
    // (the visible window) plus a one-screen margin, at a step chosen so ticks are
    // ~90-120px apart on screen for the given pixels-per-second. Preserves the stable
    // #ctps-ruler-track element (only its tick children change) so drag-to-scrub keeps
    // its ref. pxPerSec is the SOLE density input — timeline.js does NOT read back
    // --ctps-content-w via getComputedStyle.
}
```

- `timeline.js` gives the ruler track element the id `ctps-ruler-track` and builds it ONCE (stable); `rebuildRuler` clears/re-appends only its tick children. `timeline_view.js` calls it (rAF-coalesced) after each committed zoom **and** on pan, passing the current `pxPerSec` and visible `[viewT0, viewT1]`.

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `tools/path_planner/ctps/player_html.py` | T1 | T6 |
| `tools/path_planner/ctps/assets/components.css` | T2 | — |
| `tools/path_planner/ctps/assets/layout.css` | T2 | — |
| `tools/path_planner/ctps/assets/timeline_view.js` (new) | T3 | — |
| `tools/path_planner/ctps/assets/timeline.js` | T4 | — |
| `tools/path_planner/ctps/assets/transport.js` | T5 | — |
| `tools/path_planner/ctps/assets/chrome.js` | T5 | — |
| `tools/path_planner/tests/test_ctps_player_html.py` | T6 | — |

No two tasks edit the same file.

### Naming (must match across tasks)

- Ids: `ctps-tl-viewport`, `ctps-tl-content`, `ctps-tl-thumb`, `ctps-ruler-track`, `ctps-zoom`, `ctps-zoom-in`, `ctps-zoom-out`, `ctps-zoom-fit`, `ctps-zoom-level`.
- Classes: `ctps-tl-toolbar`, `ctps-tl-viewport`, `ctps-tl-content`, `ctps-tl-row`, `ctps-tl-scroll`, `ctps-tl-scroll-thumb`, `ctps-zoom-btn`, `ctps-zoom-slider`, `ctps-zoom-fit`, `ctps-zoom-level`, `ctps-tl--nozoom` (disabled/degenerate treatment on `.ctps-timeline`).
- Custom properties: `--ctps-content-w`, `--ctps-tl-thumb-w`, `--ctps-tl-thumb-x`.
- Keymap keys: `+`, `=`, `-`, `0`.

## Zoom / pan math (Notes for `timeline_view.js`, T3)

Let `dur = track.duration_s`, `gutter = --ctps-lane-gutter (52px)`, `gap = --ctps-lane-gap`.

- **Track view width**: `trackView = viewport.clientWidth - gutter - gap` (visible track area right of the sticky gutter). Recompute on init + resize.
- **Fit**: `fitContentW = Math.max(1, Math.floor(trackView))`; `pxPerSecFit = fitContentW / dur` (guard `dur > 0`). At zoom `z ≥ 1`, `contentW = fitContentW * z`, written to `--ctps-content-w`. `z = 1` ⇒ content fills the track view, no scroll.
- **Max zoom**: `MIN_VISIBLE_SECONDS = 1.5`; `zoomMax = clamp(dur / MIN_VISIBLE_SECONDS, 1, ABSOLUTE_MAX)` with `ABSOLUTE_MAX = 200`. If `dur ≤ MIN_VISIBLE_SECONDS` ⇒ `zoomMax = 1` ⇒ zoom disabled. (Effect note: for traces longer than `1.5 × 200 = 300 s` the `200×` cap binds first, so the min window exceeds 1.5 s on very long traces — acceptable.)
- **Slider mapping**: slider `[0..1000]` ↔ `z ∈ [1 .. zoomMax]` on a log scale: `z = zoomMax^(v/1000)`; inverse `v = 1000 * log(z)/log(zoomMax)`. Guard `zoomMax > 1` before dividing by `log(zoomMax)` (when `zoomMax = 1` the controls are disabled and the mapping is not evaluated).
- **Button/key step**: multiply/divide `z` by `1.5` per press, clamped to `[1, zoomMax]`.
- **Playhead-anchored zoom** (buttons/slider/keys): `frac = clamp(store.t / dur, 0, 1)`; `oldPlayheadX = gutter + gap + frac*oldContentW`. After setting `contentW`: `newScrollLeft = (gutter + gap + frac*newContentW) - (oldPlayheadX - oldScrollLeft)`. Clamp to `[0, scrollWidth - clientWidth]`.
- **Cursor-anchored zoom** (Ctrl/⌘+wheel): `cursorContentX = oldScrollLeft + (evt.clientX - viewportRect.left)`; `cursorFrac = clamp((cursorContentX - gutter - gap) / oldContentW, 0, 1)`. After zoom: `newScrollLeft = (gutter + gap + cursorFrac*newContentW) - (evt.clientX - viewportRect.left)`. Clamp. `evt.preventDefault()` (listener `{ passive: false }`).
- **Pan**: set `viewport.scrollLeft` (a DOM property, not an inline style). Wheel handler (`{ passive: false }`): if `evt.ctrlKey || evt.metaKey` → zoom (above); else if `Math.abs(evt.deltaY) > Math.abs(evt.deltaX)` → horizontal pan by a `deltaMode`-normalized `deltaY` (LINE→×~16, PAGE→×clientWidth), `preventDefault`; otherwise let native `deltaX` scroll through (trackpad horizontal). Thumb drag → set `scrollLeft`. On native `scroll`, update `--ctps-tl-thumb-w`/`-x`.
- **Follow** (subscribe `["t","playing"]`): while `store.playing` and `!followSuspended`, if the playhead's viewport-x is within `EDGE_MARGIN` of the right edge or left of the visible track, page the view so the playhead sits ~⅓ from the left. `followSuspended` is set by user-input handlers (wheel-pan, thumb drag) and by a native `scroll` that was NOT self-initiated (set a `_selfScroll` flag around programmatic `scrollLeft` writes and clear it on the next `scroll`). Re-arm (`followSuspended = false`) on the `playing` false→true edge and in `fit()`.
- **Resize** (`ResizeObserver` on `#ctps-tl-viewport`, coalesced on rAF, + `window` resize fallback — mirror `renderer.js` lines 948–966): recompute `trackView`/`fitContentW`, re-apply the current `z` (clamped to the new `zoomMax`), re-anchor scroll, resync slider/readout/thumb, and `rebuildRuler(pxPerSec, viewT0, viewT1)`.
- **Ruler coupling**: after any committed zoom OR pan (rAF-coalesced), call `CTPS.timeline.rebuildRuler(pxPerSec, viewT0, viewT1)` where `viewT0 = scrollLeft/pxPerSec`-adjusted for the gutter and `viewT1 = viewT0 + trackView/pxPerSec`. This bounds tick node count on long traces.
- **Guards**: `dur ≤ 0` or `zoomMax ≤ 1` ⇒ set `contentW = fitContentW`, add `ctps-tl--nozoom` to `.ctps-timeline`, disable slider/buttons (`disabled`), hide the thumb; never throw.

## Testing Strategy

**Levels:** Unit (Python emitter) + manual integration (served HTML eyeball). No JS unit runner exists for the player.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `render()` emits the new timeline markup | Unit | HTML contains `id="ctps-tl-viewport"`, `id="ctps-tl-content"`, `id="ctps-tl-thumb"`, `id="ctps-zoom"`, `id="ctps-zoom-in"`, `id="ctps-zoom-out"`, `id="ctps-zoom-fit"`, `id="ctps-zoom-level"`, `class="ctps-tl-toolbar"`, `class="ctps-tl-scroll"`, and still `class="ctps-timeline"`. |
| TC2 | `timeline_view.js` registered + inlined | Unit | `timeline_view.js` is in `_ASSET_JS`; its exact on-disk content appears verbatim in the page (existing TC-ASSETS loop iterates `_ASSET_JS`, so it is auto-covered). |
| TC3 | Preserved ids still present | Unit | `id="ctps-ruler"`, `id="ctps-lanes"`, `id="ctps-playhead"` still in the HTML (moved, not removed). |
| TC4 | Self-containment holds | Unit | `render()` with a real map still passes `assert_self_contained`; no forbidden substring in the new JS/CSS. (existing TC-LINT-neg stays green.) |
| TC5 | Contract markers present in assets | Unit | `--ctps-content-w` appears in both `components.css` and `timeline_view.js`; `--ctps-tl-thumb-w` in both; `CTPS.timelineView` in `timeline_view.js`; `rebuildRuler` in `timeline.js` and `timeline_view.js`. |
| TC6 | Existing structural + round-trip tests | Unit | The full `test_ctps_player_html.py` suite (TC-STRUCT, TRACK round-trip, PLAYER_META lanes, TC-ASSETS/TC-NOINLINE) stays green. |
| TC7 | Long-trace zoom in/out + readout/slider sync | Manual | Serve a long-duration trace; `+`/`−`/slider/`Ctrl`+wheel change zoom; content widens/narrows; thumb resizes; never below fit; readout `N.N×` and slider position track each other both ways (AC11). |
| TC8 | Playhead + cursor anchoring | Manual | Button zoom keeps playhead at same screen x; `Ctrl`+wheel keeps the point under the cursor fixed; no browser page-zoom. |
| TC9 | Pan sync + sticky gutter + scrub coexistence | Manual | Thumb drag / vertical-wheel / trackpad-`deltaX` pans ruler+lanes+playhead in sync; labels stay pinned + opaque + above the playhead while scrolled (AC6); dragging the lanes still scrubs to the correct time at any scroll/zoom; toolbar/labels/scrollbar clicks don't scrub. |
| TC10 | Follow + fit + keys + seek-slider untouched | Manual | Playing while zoomed auto-scrolls to keep the playhead visible; manual pan suspends it; `0`/`Fit` resets; `+`/`=`/`−` keys zoom; the transport top seek slider + drift ticks do not move (AC13). |
| TC11 | Degenerate + resize + full-view | Manual | Empty trace: controls disabled, thumb hidden, no throw. Toggle full-view off/on and resize the window: fit recomputes, nothing mis-sized. |

**Test data:** an existing trace fixture with a non-trivial duration — e.g. `tools/path_planner/tests/fixtures/ctps_lrs_track.json` for emitter tests; `ctps_cargo_trace.json` (via `player_html.render`, see the cargo demo recipe) or a freshly built long mission trace for the manual eyeball, rendered to a scratchpad `.html` and served with `python -m http.server` on `127.0.0.1` (claude-in-chrome blocks `file://`).

**Run command:** `uv run --extra dev pytest tools/path_planner/tests/test_ctps_player_html.py` (Python); manual serve + browser for TC7–TC11.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T3 | New `timeline_view.js` zoom/pan controller | — | high | `tools/path_planner/ctps/assets/timeline_view.js` | Create the module (own `<script>`, IIFE, `"use strict"`, vanilla JS, ADR-053 setProperty-only geometry; no forbidden substrings). Register `init(ctx)` via `CTPS._inits`. In init, query the DOM contract elements; compute `trackView`/`fitContentW`/`zoomMax` per the Zoom/pan math section; own zoom state, the custom scrollbar thumb (`--ctps-tl-thumb-w`/`-x` via `setProperty`, thumb drag → `scrollLeft`, native `scroll` → thumb sync + follow-suspend-unless-self-scroll), a single `wheel` listener `{passive:false}` (Ctrl/⌘ → cursor-anchored zoom + preventDefault; else `|deltaY|>|deltaX|` → deltaMode-normalized horizontal pan + preventDefault; else pass `deltaX` through), the zoom slider/`+`/`−`/`Fit` widget wiring, playhead-anchored zoom, follow-the-playhead (subscribe `["t","playing"]`; suspend on manual pan; re-arm on play edge + `fit()`), and a `ResizeObserver` (rAF-coalesced) + window-resize fallback. Set `--ctps-content-w` on `.ctps-timeline`; after each committed zoom AND pan (rAF-coalesced) call `CTPS.timeline.rebuildRuler(pxPerSec, viewT0, viewT1)`. Expose `CTPS.timelineView = { zoomIn, zoomOut, fit }`. Guard `dur ≤ 0` / `zoomMax ≤ 1` (add `ctps-tl--nozoom`, disable controls, hide thumb, never throw). Break into named helpers (fit compute, applyZoom, anchor math, scrollbar sync, follow, wheel, rulerSync). Satisfies AC1–AC12, AC15. Creates only this file. |
| T1 | Restructure timeline markup + register asset | T3 | med | `tools/path_planner/ctps/player_html.py` | In `_BODY_MARKUP`, replace the `.ctps-timeline` inner block with the DOM contract markup (toolbar + viewport + `#ctps-tl-content` wrapping the preserved `#ctps-ruler`/`#ctps-lanes` + `#ctps-playhead` + custom scrollbar). Numeric HTML entities for glyphs (`&#8722;` minus, `&#215;` ×), class-based only, no inline styles, no forbidden substrings. Add `_ASSETS_DIR / "timeline_view.js"` to `_ASSET_JS` immediately AFTER `timeline.js` and BEFORE `panels.js`. Do not touch `PLAYER_META`. Satisfies AC13 (transport seek slider untouched), AC14. |
| T2 | Timeline dock CSS: rows, sticky gutter, viewport, scrollbar, zoom widget, playhead calc | — | med | `tools/path_planner/ctps/assets/components.css`, `tools/path_planner/ctps/assets/layout.css` | **layout.css** (owns dock layout): make `.ctps-timeline` a vertical flex (toolbar / viewport / scrollbar rows); `.ctps-tl-viewport { overflow-x:auto; overflow-y:hidden }`; `#ctps-tl-content { position:relative; width: calc(var(--ctps-lane-gutter) + var(--ctps-lane-gap) + var(--ctps-content-w, 0px)) }`; `.ctps-tl-row { display:flex; width:100% }`; the row track element (`.ctps-lane` / `#ctps-ruler-track`) `flex: 0 0 var(--ctps-content-w, 100%)`; `.ctps-lane-label { flex:0 0 var(--ctps-lane-gutter); position:sticky; left:0; z-index:3 }`; keep `.ctps-timeline` height auto, `min-width:0`. **components.css** (owns look): `.ctps-lane-label` opaque `background:var(--ctps-panel)`; hide native scrollbar (`.ctps-tl-viewport{scrollbar-width:none}` + `.ctps-tl-viewport::-webkit-scrollbar{display:none}`); update `.ctps-playhead` `left` to the content-relative calc (Contracts) + `z-index:2` (below labels); `.ctps-tl-toolbar` right-aligned flex; `.ctps-tl-scroll` track + `.ctps-tl-scroll-thumb` (width `var(--ctps-tl-thumb-w)`, left `var(--ctps-tl-thumb-x)`, hidden when `ctps-tl--nozoom`); `.ctps-zoom-btn`/`.ctps-zoom-slider`/`.ctps-zoom-fit`/`.ctps-zoom-level` reusing tokens; `.ctps-tl--nozoom` disabled treatment. Remove the old 2-column `grid-template-columns` rules on `.ctps-ruler`/`.ctps-lanes` and the old `.ctps-lane-label--*` grid assumptions (keep the color modifiers). Satisfies AC6 + the visual half of AC1–AC11. Do NOT edit JS. |
| T4 | timeline.js: build rows, stable ruler track, windowed `rebuildRuler`, scrub rescope | — | med | `tools/path_planner/ctps/assets/timeline.js` | Change `buildLanesDom` to append one `.ctps-tl-row` per lane (label cell + `.ctps-lane` track) instead of alternating flat grid siblings; `renderLaneItems`/`pctOf` unchanged (percent of the `.ctps-lane` track). Build the ruler as a `.ctps-tl-row` whose track element has id `ctps-ruler-track`, created ONCE and stable; add `rebuildRuler(pxPerSec, viewT0, viewT1)` that clears/re-appends ONLY tick children within `[viewT0, viewT1]`+margin, at a step from `pxPerSec` (reuse `TICK_STEP_CANDIDATES`, ~90–120px on-screen spacing). Do NOT read `--ctps-content-w` via getComputedStyle — `pxPerSec` is the sole density input. Expose `CTPS.timeline = { rebuildRuler }`. Rescope `wireScrub` to bind on `#ctps-tl-viewport` and ignore pointer-downs whose target is inside a `.ctps-lane-label`, `.ctps-tl-toolbar`, or `.ctps-tl-scroll`; keep resolving the fraction from the stable `#ctps-ruler-track` rect (correct at any scroll/zoom). Update the file docstring for the row structure. Satisfies AC8, AC10. Do NOT edit other files. |
| T5 | transport.js zoom keymap + chrome.js help rows | — | low | `tools/path_planner/ctps/assets/transport.js`, `tools/path_planner/ctps/assets/chrome.js` | transport.js: in `onKeyDown`, add cases for `+`, `=`, `-`, `0` calling `CTPS.timelineView.zoomIn()` / `zoomIn()` / `zoomOut()` / `fit()`, each guarded like `openHelp` (`if (CTPS.timelineView && typeof …==='function')`) with `preventDefault()`; keep the existing ArrowLeft/Right-for-range, ctrl/meta/alt, text-input, and help-open guards intact (zoom keys must not fire in a text input or behind the help modal). Update the module header key list. chrome.js: add rows to `buildHelpModal`'s keymap list (near lines 159–165) — `["+ / −", "Zoom timeline in / out"]`, `["0", "Fit timeline to width"]`. Satisfies AC2, AC5. Do NOT edit other files. |
| T6 | Emitter structural + contract tests | T1, T2, T3, T4 | low | `tools/path_planner/tests/test_ctps_player_html.py` | Add: TC1 (new markup ids/classes present), TC3 (preserved ids), TC5 (contract markers — `--ctps-content-w` in components.css + timeline_view.js, `--ctps-tl-thumb-w` in both, `CTPS.timelineView` in timeline_view.js, `rebuildRuler` in timeline.js + timeline_view.js). Rely on the existing TC-ASSETS loop for TC2, and TC-STRUCT/round-trip/TC-LINT for TC4/TC6. Run `uv run --extra dev pytest tools/path_planner/tests/test_ctps_player_html.py`. Blocked by all of T1–T4 because TC5 reads markers written in their files. Satisfies AC14; pins the DOM/asset contract. |
| T7 | Serve + eyeball verification (long trace) | T1, T2, T3, T4, T5 | low | (scratchpad only — no repo files) | Render a long-duration trace to a scratchpad `.html` via `player_html.render(...)` (use `ctps_cargo_trace.json` or a built long mission trace; the cargo demo recipe), serve with `python -m http.server` on `127.0.0.1`, walk TC7–TC11 by hand (zoom via buttons/slider/`Ctrl`-wheel/keys; playhead + cursor anchoring; thumb + wheel + trackpad pan in sync; sticky labels pinned/opaque; scrub still correct; follow during playback; `Fit`/`0`; empty-trace + resize + full-view-return). Capture a screenshot. Report any AC that fails. |

## Notes for Implementer

- **ADR-053 is law here:** JS writes geometry ONLY via `setProperty` (custom props / `left` / `width`) — never `el.style.<prop> = …`. `viewport.scrollLeft = x` is a DOM property, not an inline style, so it is allowed. Do not introduce any forbidden substring (`src=`, `href=`, `http://`, `https://`, `data:image`, `</script>`) in the new JS/CSS — the emit-time lint rejects the whole page.
- **Sticky labels need a full-width row:** a grid item's containing block is its 52px grid area, so sticky-in-a-grid-cell has zero travel (the original design bug). Each `.ctps-tl-row` must span the full content width so its sticky label travels the whole scroll. This is why the dock is per-row flex, not the old 2-column grid.
- **Z-order:** sticky labels `z-index:3` (opaque `--ctps-panel` bg), playhead `z-index:2`, spans/ticks `z-auto (0)`. The playhead must sit BELOW the labels — while scrolled, a playhead whose viewport-x falls in the left 52px strip would otherwise paint over a pinned label. (The earlier "playhead never enters the gutter" reasoning was in content coords and is wrong under scroll.)
- **Stable ruler track element:** `wireScrub` captures the ruler track's rect. If `rebuildRuler` recreated the track element, the scrub ref would go stale (a known-class gotcha). Rebuild only the tick CHILDREN; keep `#ctps-ruler-track` itself.
- **Playhead containing block:** the playhead must be a child of `#ctps-tl-content` (a positioned, content-width element that is itself scrolled) so it pans with the track. An absolute child of the *viewport* would NOT scroll — do not anchor it there.
- **Preserve percent math:** do NOT convert spans/pins/ticks to pixel positioning. `pctOf` stays; the only change is the track element is now `--ctps-content-w` wide.
- **Wheel handling:** register `wheel` `{ passive: false }`; normalize `deltaMode` (Firefox reports LINE — `deltaY≈3`; multiply to px). Ctrl/⌘-wheel zoom AND vertical-wheel pan both `preventDefault` (Ctrl+wheel is the browser page-zoom gesture); let native `deltaX` (trackpad horizontal) scroll through.
- **Follow self-cancel guard:** wrap programmatic `scrollLeft` writes with a `_selfScroll` flag so the module's own paging doesn't register as a "manual pan" and suspend follow. Only user-input pans (wheel, thumb, native non-self scroll) suspend it.
- **`--ctps-content-w` fallback owner is T2 (CSS):** use `var(--ctps-content-w, 100%)` / `, 0px)` in every usage so a pre-hydration render or a thrown `timeline_view.js` init degrades to a full-width, non-zooming dock instead of collapsing the layout (matches boot.js's per-init isolation). No `tokens.css` edit needed.
- **Init ordering:** `timeline_view.js` runs after `timeline.js` (its `_ASSET_JS` position), so `CTPS.timeline.rebuildRuler` and the built rows exist when `timeline_view`'s init runs. `boot.js` stays last.
- **Follow vs scrub:** follow only auto-scrolls during playback; the drag-to-scrub gesture is unchanged except for the viewport rescope. A scrub-drag moves the playhead, not the view, so it is not a "manual pan" for follow-suspend.
- **Verify on a genuinely long trace** — the placeholder field's default mission may be short; build or reuse a long trace so zoom actually overflows the viewport. (`field_data` is a test map; verify the built behavior anyway.)
- **Rollback:** the change is additive and contained to the player's timeline dock + one new asset; reverting the edited/created files restores the prior full-width dock. No data, trace, or firmware surface is touched.
