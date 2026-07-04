# Handoff: CTPS Player — Instrument shell (design 3a)

## Overview
Redesigned playback screen ("Player") for CTPS — Clear Trajectory Prediction Simulator. The design being handed off is **option 3a "Instrument"** from `Player Explorations.dc.html`: a dark, monochrome-plus-accent console with a live canvas stage, right rail (current command / holding / tabbed Log+Inspector), transport bar, multi-lane timeline dock, and a Full-view mode. Options 1a/1b/1c/2a/2b in the same file are historical explorations — ignore them except as reference.

## About the Design Files
The files in this bundle are **design references created in HTML** — a prototype showing intended look and behavior, not production code to copy directly. The target codebase is the existing CTPS emitter (`ctps/player_html.py` + `ctps/assets/player.js`, vanilla JS + Canvas, self-contained single-file output, offline / file:// safe, no external fonts or CDNs). Recreate the design in that environment using its existing patterns: layered canvas renderer, bounded seek (`poseAt` / `findKeyframeIndex`), transport state machine, incremental log narration. Keep the reserved seams (#57 author/objects, #55 wheel-arc, #23 LRS/telemetry) invisible — pure code seams, no UI.

## Fidelity
**High-fidelity.** Colors, spacing, typography, and layout are final. Recreate pixel-perfectly. All interactive behavior demonstrated in the prototype (scrub-driven robot/log/holding, tab switching, full view) is intended behavior.

## Screen: Player (single view)

Fixed dark console, fills viewport. Column flex: header → stage row (flex 1) → transport bar → timeline dock.

### Design tokens
- Background page/card: `#161614`
- Panel background: `#1e1d1a`
- Panel raised / chip: `#26251f`
- Border: `#31302a` (subtle) · `#3f3d35` (stronger)
- Text primary: `#e8e5db` · secondary: `#a5a193` · muted: `#918d7f` · faint: `#565349`
- Accent (playback, robot, trail, seek fill, active tab underline): `#ffd24a` (tweakable: `#e0a43c`, `#4fd6a3`, `#ff8a5b`)
- Hold color (everything cargo-related): `#4fd6a3` (tweakable: `#5bc8e0`, `#ffd24a`, `#ff8a5b`)
  - soft fill: hold color at 14% alpha · border: 60% alpha
- Drift / warning: `#ff6b4a`, warning border `#4a2e26`
- Fonts: UI = system sans (`ui-sans-serif, system-ui`); data/readouts/log = `ui-monospace, Consolas, monospace`. No webfonts (offline constraint).
- Radii: panels 6px, buttons/chips 3–4px. Borders 1px.

### Header (row, 10px 14px padding, bottom border #31302a)
- "CTPS" — 12px bold, letter-spacing .12em
- Trace filename — mono 12px `#918d7f`
- "PREDICTED" badge — 10px semibold, 3px 8px, 1px border `#3f3d35`, radius 3, color `#918d7f`. (This is the trace-source indicator; #23 LRS seam will add more values later — keep it a single static badge for now.)
- Spacer (flex 1)
- Drift chip — mono 11px `#ff6b4a`, border `#4a2e26`, radius 3, ▲ glyph + `drift 12.4 cm @ 46.5 s`. Shows worst drift_warn; clicking should seek to it. Hidden when no drift events.
- "? Help" button — 11px `#918d7f`, border `#31302a`. Opens keymap/legend overlay (light design — not mocked; simple modal listing shortcuts + color legend).

### Stage row (flex, gap 8, padding 8)
**Canvas stage** (flex 1, `#1e1d1a`, border, radius 6, relative):
- Field: printed map image (`map.png`, 2000×968) drawn full-bleed within field bounds, dimmed by a `rgba(22,22,20, mapDim)` scrim — default mapDim 0.48, user-adjustable 0–0.85. 1.5px bounds frame `#3f3d35`.
- Full trajectory polyline: `#e8e5db` @ 60% opacity, 2px.
- Traveled portion (start → playhead): accent color, 2.5px, drawn over the full path.
- Nodes: 4px dots `#e8e5db` + mono 11px labels with a 3px dark halo (paint-order stroke `#161614`) so they read over the map. Toggleable layer.
- Robot: accent triangle (≈26×18px at 1:1), dark 1.5px outline, positioned/rotated by bounded-bisect lerp over keyframes — same algorithm as existing `findKeyframeIndex`/`poseAt`. Heading = direction of last moving segment.
- Scale bar bottom-left: 50 cm, mono 10px `#918d7f`.
- Layer chips top-right: MAP / TRAIL / NODES — 10px, 3px 8px, radius 3. Active = light bg `#e8e5db` + dark text `#161614`; inactive = `#26251f` bg, `#918d7f` text, border `#31302a`.
- **⛶ Full view** switch bottom-right — chip style (`#26251f` bg, `#c9c5b8` text). Replaces zoom +/− cluster entirely (per review). Toggles Full view (below). Active state inverts (light bg, dark text, label "🗗 Exit full view").

**Right rail** (fixed 288px, column, gap 8) — hidden in Full view:
1. **Current command** panel: label ("CURRENT COMMAND", 10px uppercase `#918d7f`), command mono 15px `#e8e5db` (drift events render `#ff6b4a`), detail line mono 11px `#918d7f` (e.g. "hop 2/3 · LINE edge"). Driven by last event ≤ playhead.
2. **Holding** panel (cargo rail): header "HOLDING" + `n / 3` count. One card per held item: 1px border holdColor@60%, fill holdColor@14%, radius 5, 8px 10px padding; name mono 12.5px holdColor, meta right-aligned 10px `#918d7f` ("grab 14.6 s @ J2"). Remaining capacity renders as dashed `#31302a` "empty" slots. Items appear on `manip grab` events ≤ playhead and disappear on `put` — replayed from the event stream, not stored state.
3. **Log / Inspector tabbed panel** (flex 1): tab row bottom-bordered; active tab 11px semibold `#e8e5db` with 2px accent underline; inactive `#918d7f`.
   - **Log tab**: last ~10 events ≤ playhead, mono 11.5px, line-height 1.65, newest at bottom (`justify-content:flex-end`). Current event highlighted (`#26251f` bg pill, text `#e8e5db`); drift lines `#ff6b4a`; older lines `#a5a193`. Clicking a line seeks to its `t`.
   - **Inspector tab**: "SELECTED · robot" header; 2-col mono 12px grid: pose `(x, y) cm`, heading `°`, nearest node + distance, **drift chk** — result of the last post-goto drift check ≤ playhead: `OK · < 10 cm` (`#e8e5db`) or `12.4 cm off J6` (`#ff6b4a`). NOT a continuous drift value (that only exists with a recorded actual trace — LRS seam #23). Selecting a node/entity on canvas swaps inspector content (hit-test + inspector-renderer registry seam).

### Transport bar (row, gap 10, panel style, margins 0 8px)
- ⏮ / **play-pause** (accent bg, dark glyph, min-width 64, weight 700) / ⏭ — 28px tall; step-by-frame buttons intentionally removed (per review), Home/End jumps stay.
- Divider, then drift jumps: ⚑‹ ⚑› — `#ff6b4a` text, `#4a2e26` border.
- Seek slider (flex 1): 4px track `#31302a`, accent fill to playhead, 13px light thumb with 2px accent ring, 2×8px `#ff6b4a` tick at each drift event.
- Time readout mono 13px `#a5a193`: `42.18 / 96.40 s`.
- Speed selector: segmented 0.5× / 1× / 2× / 4×, 11px; active = `#e8e5db` bg + `#161614` text; container `#161614` bg + border.

### Timeline dock (panel, margins 8, relative)
Grid: 52px label column + flex lane column, 4px row gap. Mono 9px time ruler (0–90 s) on top, faint `#565349`.
- **TASK** lane (18px): spans `#26251f` bg, `#3f3d35` border, 9.5px `#a5a193` labels (T1 collect / T2 deliver / T3 return).
- **GOTO** lane (18px): spans `#4a483f` on `#161614`.
- **HOLDING** lane (34px, label in holdColor): one rounded span (12px tall, radius 6) per item from grab→put, holdColor border/fill/text, stacked in 2 sub-rows for overlaps. Mirrors the Holding panel.
- **DRIFT** lane (18px, label `#ff6b4a`): drift is a **discrete post-goto check** — ▲ pin at each `drift_warn` t; from that pin a red span (`#ff6b4a` @14% fill, 55% border) runs to end-of-track labeled "12.4 cm off node · uncorrected" (a goto reveals drift but never corrects it; close the span if a future re-localization command exists).
- Playhead: 1.5px `#e8e5db` vertical line across all lanes, subtle glow; dragging in the dock scrubs.

### Full view mode
Toggled by ⛶ switch. Right rail + timeline dock collapse; canvas stage fills the width; transport bar remains. HUD appears top-left on canvas, stacked chips (bg `rgba(22,22,20,.88)`, border `#31302a`, radius 4):
- Current command — mono 13px, command color
- `holding n / 3` — mono 11px, holdColor
(Drift chip deliberately excluded per review.)

## Interactions & Behavior
- **Scrubbing is the spine**: every region derives from playhead t — robot pose (bounded bisect + lerp between keyframes), traveled polyline, current command (last event ≤ t), log window, holding replay (grab/put), inspector values, drift-check result. Rebuild-on-seek must stay bounded (existing incremental-forward + rebuild pattern).
- Play/pause (Space), speed 0.5–4× (`[` `]`), step by event (Shift+←/→), Home/End, D = next drift, F = full view, ? = help. Step-by-frame ←/→ exists in the engine but has no dedicated buttons.
- Click log line / node marker / drift chip / drift pin → seek.
- Layer chips toggle map/trail/node layers (layer registry).
- Tabs: Log ⇄ Inspector, accent underline follows.
- No animations beyond playback itself; state changes are instant.

## State Management
- `t` (playhead s), `playing`, `speed`, `activeTab` (log|inspector), `fullView`, layer flags (`map`, `trail`, `nodes`), `selectedEntity` (default robot).
- Derived per frame from Track data (never stored): pose/heading, traveled path, current event, log lines, held items, drift-check result.
- Track ingest stays schema-version-tolerant (additive keys survive).

## Data model (drives everything)
Matches `track_builder.py` output: `keyframes: [{t, x, y, h}]`, `events: [{t, type: task|goto|manip|drift_warn|…, label}]`, field (NODES, OBSTACLES, FIELD_MM), `duration_s`. Manip labels encode grab/put + item name; drift_warn label: `chassis {d:.1f} cm off node {node}`, threshold `DRIFT_THRESHOLD_CM = 10`.

## Assets
- `map.png` — printed mat image (user-supplied, 2000×968). Embed via the existing single-file inlining path.
- No icon fonts; glyphs are unicode (⏮ ⏭ ❚❚ ⚑ ▲ ⛶). Replace with tiny inline SVG if unicode rendering is inconsistent.

## Files
- `Player Explorations.dc.html` — all explorations; **option 3a (section id="3a", top of file) is the design of record.** Its inline styles are the source of truth for exact values; its logic class shows reference implementations of scrub-derived state (bisect, event replay, holding reconstruction).
- `uploads/map.png` — mat image.
