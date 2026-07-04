# CTPS Player UI/UX — design of record (sidecar)

Carried verbatim from the external design handoff (`CTPS UIUX Functional Scoping.zip`,
handoff `design_handoff_ctps_player`, **option 3a "Instrument"**). This is the pixel
source of truth; the executor reproduces THIS, and does not re-design. The main spec
links here.

> **Base direction:** 3a "Instrument" (dark, monochrome-plus-accent console).
> **Grafted in:** none (single handed-off direction; 1a/1b/1c/2a/2b in the mockup are
> historical explorations, ignored).

## Deliberate deviations from the mockup (authoritative — apply these)

1. **Robot marker = BOX, not the triangle in 3a.** A chassis rectangle with a small
   heading indicator (front nose), accent fill + 1.5px dark outline. Rationale: reads
   correctly when #55 later animates a single-wheel pivot *about a corner wheel*; a
   triangle can't show which wheel it swings around. (User override.)
2. **Canvas is Canvas-2D, not SVG.** The mockup draws the stage in SVG only to
   represent it; production uses the existing layered Canvas-2D renderer + world↔pixel
   transform.
3. **CSS classes + custom properties, never inline element styles.** The mockup uses
   inline styles; translate every one into the stylesheet. Dynamic values (playhead
   fraction, fills) are set via one CSS custom property per animated axis
   (`el.style.setProperty('--ctps-frac', …)`), never `el.style.<prop> = …`.
4. **Seek control = a styled native `<input type="range" id="ctps-seek">`** (accent
   fill + light thumb + red drift ticks via an overlay layer), not the mockup's pure
   div slider — keeps keyboard/a11y and the emitter test contract.
5. **Icons = unicode glyphs** (⏮ ⏭ ❚❚ ⚑ ▲ ⛶) or inline SVG *paths* — never `<use href>`
   or `<image href>` (the offline self-containment ban forbids `href=`/`src=`).
6. **GOTO timeline lane = LIVE** via a small Python `track_builder` goto-span emit — a
   successful `goto` emits no event today (`track_builder.py:426`), so we add a `goto`
   event with `t`/`t_end`/`from->target` (no C++). **HOLDING lane + panel = reserved empty
   shells** (manip events carry no grab/put + item identity yet). TASK, GOTO, and DRIFT
   lanes are live.

## Design tokens (final — use exact hex)

| Token | Hex | Use |
|---|---|---|
| bg page/card | `#161614` | app background |
| panel bg | `#1e1d1a` | canvas stage, rail panels, transport, dock |
| raised / chip | `#26251f` | chips, current-event pill, buttons |
| border subtle | `#31302a` | dividers, panel borders |
| border strong | `#3f3d35` | bounds frame, badge, task spans |
| text primary | `#e8e5db` | commands, headings, active chip text |
| text secondary | `#a5a193` | readouts, older log lines |
| text muted | `#918d7f` | labels, filename, meta |
| text faint | `#565349` | empty slots, ruler |
| accent | `#ffd24a` | playhead, robot, trail, seek fill, active tab underline |
| accent (alt) | `#e0a43c` · `#4fd6a3` · `#ff8a5b` | tweakable alternates (ship `#ffd24a`) |
| hold color | `#4fd6a3` | all cargo/holding (reserved) — soft fill @14% α, border @60% α |
| drift / warn | `#ff6b4a` | drift chip/pins/ticks/lane; warn border `#4a2e26` |

- **Fonts:** UI = `ui-sans-serif, system-ui`; data/readouts/log = `ui-monospace, Consolas, monospace`. No webfonts.
- **Radii:** panels 6px, buttons/chips 3–4px. Borders 1px.
- **Map dim scrim:** `rgba(22,22,20, 0.48)` default (fixed; not user-adjustable this PR).

## Layout, behavior, states, responsive

Full prose is the handoff README (reproduced below verbatim). Summary: fixed dark
console filling the viewport; column flex `header → stage row (flex 1) → transport bar
→ timeline dock`. Stage row = canvas stage (flex 1) + right rail (288px, hidden in
Full view). Right rail = Current command panel / HOLDING panel [reserved] / tabbed
Log+Inspector panel. Transport = ⏮ play-pause ⏭ · drift-jump ⚑‹ ⚑› · seek slider · time
readout · speed 0.5–4×. Timeline dock = TASK / GOTO / HOLDING / DRIFT lanes + playhead.
Full-view = rail+dock collapse, canvas fills, on-canvas HUD (command + holding count).

**States:** default / playing / paused / seeking; no-drift (drift chip + lane hidden);
no-map (vector fallback); empty/short trace; Full view; tab = Log vs Inspector;
selection = robot (default) vs node. No animations beyond playback; state changes are
instant.

**Responsive:** the console fills the viewport and the canvas resizes to its box
(existing devicePixelRatio-aware resize). The mockup's 1180×756 card is only the
capture size.

**Keymap:** Space play/pause · `[` `]` speed · Shift+←/→ step event · Home/End · `D`
next drift · `F` full view · `?` help. Step-by-frame ←/→ exists in the engine (no
dedicated buttons).

---

## Handoff README (verbatim)

_The original `design_handoff_ctps_player/README.md`, unmodified:_

```md
# Handoff: CTPS Player — Instrument shell (design 3a)
[... the full README text is the authoritative behavioral description; see the
handoff bundle. Key sections — Design tokens, Header, Stage row, Right rail,
Transport bar, Timeline dock, Full view mode, Interactions & Behavior, State
Management, Data model — are all reproduced above and in the main spec's UI/UX
Specification section. ...]
```

## Mockup markup (option 3a — exact value source of truth)

The exact spans/positions/inline values for option 3a live in the handoff's
`Player Explorations.dc.html`, `<div id="3a">` (the block from the header row through
the timeline dock). Its inline styles are the source of truth for exact px/color
values; `{{ … }}` placeholders in that block mark the scrub-derived bindings
(traveledPoints, robotTransform, currentCmd, heldItems, logLines, pctStr, timeStr,
playheadLeft, layer-chip and full-view state). Reproduce those values in the class-based
stylesheet.
