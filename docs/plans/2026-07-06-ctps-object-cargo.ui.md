# CTPS #57 — UI/UX sidecar (object authoring + cargo inventory)

Design language is **fixed by ADR-053** (the CTPS "Instrument" dark theme). This
feature EXTENDS that system; it does not redesign it. No new palette — every
color is an existing `tokens.css` custom property. Interactive mockup (confirmed
by the user): the `#57` cargo-panel artifact rendered in the real tokens.

The executor reproduces THIS. Do not re-design layout, colors, or states.

---

## Color codes (existing `tokens.css` — do not add new palette hues)

| Token | Hex | Use in #57 |
|-------|-----|------------|
| `--ctps-bg` | `#161614` | player ground, rail ground |
| `--ctps-panel` | `#1e1d1a` | cargo panel, command panel |
| `--ctps-raised` | `#26251f` | tile badges, hold card, chips |
| `--ctps-border` | `#31302a` | panel/lane borders |
| `--ctps-border-strong` | `#3f3d35` | active tab, empty-cell dashed border |
| `--ctps-text` | `#e8e5db` | active tab text, object names |
| `--ctps-text-2` | `#a5a193` | detail lines |
| `--ctps-muted` | `#918d7f` | labels, inactive tabs, lane numbers |
| `--ctps-faint` | `#565349` | fill hints, dump arrows |
| `--ctps-accent` | `#ffd24a` | robot, trail, playhead (unchanged) |
| `--ctps-hold` | `#5bc8e0` | **cargo** — active cargo tab underline, hold card accent, `onrobot` tag, zone overlay, holding timeline span, drop points |
| `--ctps-drift` | `#ff6b4a` | drift (unchanged) |

Per-frame dynamic values use `element.style.setProperty('--foo', …)` only
(ADR-053 carve-out); never a static inline `style=` or `el.style.prop =`.
Object fill colors are the **authored per-object hex** from `layout.json`, applied
via `setProperty` on a custom property, not a hard-coded literal.

Type: the CTPS offline system stack — UI (`--ctps-font-ui`) for uppercase
letter-spaced labels, mono (`--ctps-font-mono`) for counts/data (with
`tabular-nums` on the count badges).

---

## Change 1 — the right rail: `Holding` panel → two-tab **cargo panel**

The single scaffolded Holding panel (`.ctps-panel--holding`, `#ctps-hold-count`,
`#ctps-hold-list`) becomes a two-tab cargo panel, one tab per on-robot store.
Count badges live ON the tabs so both are visible at a glance. Follows the
existing tab idiom (like Log/Inspector), but is its own rail section — NOT the
log/inspector tab row, and NOT registered through `panelTabs`.

```
┌ CARGO PANEL (rail) ─────────────────────────┐
│ [ Magazine  7/12 ]   [ Inventory  1/1 ]      │  ← two tabs, count badges
│───────────────────────  active underline ────│    (active = --ctps-hold)
│  MAGAZINE tab (default):                      │
│    4 lanes × 3, VERTICAL lanes, front@bottom  │
│      0    1    2    3     ← lane index (0–3)   │
│    [B3] [  ] [  ] [  ]                         │
│    [G2] [C5] [  ] [O7]                         │
│    [A1] [Y4] [  ] [W6]                         │
│     ↓    ↓    ↓    ↓      ← dump ejects front  │
│    sortTo(n) fills lane n                      │
│                                               │
│  INVENTORY tab (rear box, capacity 1):        │
│    ┌──────────────────────────────┐           │
│    │ ▪ shovel            onrobot   │  hold card│
│    └──────────────────────────────┘           │
│    closed — captures Cargo in the rear-box zone│
└───────────────────────────────────────────────┘
```

- **Magazine tab** — `4 lanes × 3`, drawn as **4 vertical columns** (index
  `0–3`), each a stack of 3 cells with the **front tile at the bottom** (a `↓`
  under each lane marks the dump-eject side). `sortTo(n)` fills lane `n`
  bottom-up. Filled cell = the tile's authored color + a short mono label; empty
  cell = a thin dashed outline (`--ctps-border-strong`). Cells are small (~24px),
  compact and centered — not stretched across the panel.
- **Inventory tab** — the **rear box** (capacity 1). A `.ctps-hold-card` (existing
  CSS, `--ctps-hold` left accent) with a color swatch, the object name, and an
  **`onrobot`** pill. Empty/closed state shows a one-line hint.
- Both counts also mirror into the stage **HUD** (`#ctps-hud-holding` →
  `magazine 7/12  rear box 1/1`) and the **Holding timeline lane** (a
  `--ctps-hold` span over each held period).

## Change 2 — on the field (player canvas)

- **Objects layer** — Tile (filled colored square/circle), Cargo (hollow
  `--ctps-hold` outline), Fixed (faint dashed `--ctps-faint`) drawn at their
  world positions; a grabbed object is omitted (it's in the sidebar).
- **Zones overlay** — faint, robot-local, moves with the robot pose: the **grab
  zone** (left), the **rear-box zone** (rear), and the **per-lane drop points**
  (dots, `--ctps-hold`). Toggled by a new **ZONES** layer chip.
- **OBJECTS** layer chip toggles the objects layer, matching the existing
  MAP/TRAIL/PATH/NODES chip row.

## Change 3 — the editor page (`layout_editor.html`, two tabs)

A separate offline page in the same dark theme. No trace/transport/timeline.

```
┌ layout_editor.html ─────────────────────────────────────┐
│ [ Field ] [ Robot ]   tools: ▭ Rect  ● Circle   ⭳Save ⭱Load│
│──────────────────────────────────────────────────────────│
│  Field tab: map backdrop + place objects   │ Properties   │
│    rubber-band draw → name / type / color   │  name  […]   │
│    Tile / Cargo / Fixed                     │  type  ▾     │
│                                             │  color ▮     │
│  Robot tab (no map): default robot box      │  x/y  [ ][ ] │
│    draw grab zone · rear-box zone ·         │  w/h  [ ][ ] │
│    4 drop points                            │  (precise)   │
└──────────────────────────────────────────────────────────┘
```

- **Field tab** — map/field backdrop; rubber-band-draw a Rect or Circle, then set
  name, type (Tile/Cargo/Fixed), color, and precise numeric dims + x/y in the
  Properties panel. Click to select, drag to move, delete key removes.
- **Robot tab** — no map; the default robot box centered; draw the grab zone,
  rear-box zone, and 4 drop points (robot-local rectangles/points).
- **Save** → Blob `<a download>` of `layout.json`. **Load** → `<input type=file>`
  + FileReader. (localStorage / File System Access API are dead on `file://`.)

## States

- **Cargo tab** — Magazine / Inventory; active tab: `--ctps-text` text +
  `--ctps-hold` underline; inactive: `--ctps-muted`.
- **Magazine cell** — empty (dashed) / full (authored color + label).
- **Hold card** — present (name + `onrobot`) / empty (closed hint).
- **Object (canvas)** — on-field / grabbed (omitted) / selected (halo, editor
  only) / drop (brief fade-in at the drop point).
- **Zone chip / OBJECTS chip** — on (`--ctps-hold` / `--ctps-text`) / off
  (`--ctps-muted`).
- **Editor tool** — Rect / Circle / Select; drawing (rubber-band) / idle.

## Responsive

Follows the player's existing behavior: the rail collapses under the stage below
~760px; the magazine lane row stays centered and does not stretch; wide content
scrolls within its own container, never the page body.
