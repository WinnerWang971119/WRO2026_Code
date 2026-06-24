# UI/UX design — Issue #79 EDGES overrides + tunable-constants inspector

Sidecar for `2026-06-23-issue-79-edges-json-overrides-gui.md`. Carried verbatim from the
Phase 1.5 design gate. The executor reproduces THIS — do not re-design.

## Toolkit & constraints

- Plain **tkinter / ttk** (NOT customtkinter — the customtkinter restyle is issue #83, out of scope here).
- Right inspector panel is a fixed **312 px** band (`panels.py` `_inspector_frame`).
- Reuse the existing **drafting-paper theme** (`editor/theme.py`) and the house widget
  factories (`_themed_frame`, `_themed_label`, `_themed_button`, `_ghost_button`,
  `_themed_entry(mono=…)`, ttk styles `Editor.TButton` / `Editor.TCombobox` /
  `Editor.TSpinbox` / `Editor.Mono.TSpinbox`).
- Reuse the existing **collapsible section** helper (`_collapsible_section(master, key, title, start_open)`)
  and the **Pivots list** idiom (scrollable rows + per-row delete + add-row).

## Chosen base + grafts

- **Base:** Direction **C — Override-list manager.** A collapsible `EDGE OVERRIDES` list in the
  inspector that mirrors the existing **Pivots** list: one row per edge that carries an override,
  add / edit / delete rows. Decoupled from canvas selection. Reuses the proven pivots add-row +
  list + delete idiom, so **no new canvas edge-selection subsystem** is introduced. Matches sibling
  issue #87 ("surface per-edge settings in the inspector").
- **Grafted from A (via the EXISTING Edge mode):** clicking an existing edge in the editor's
  current Edge mode **locates / highlights** that edge's override row (and pre-fills the add-row's
  `u` / `v` / `type` from the clicked segment). This reuses the editor's existing `_edge_selection`
  edge-mode click handling — it does NOT add a new selection model. User's words: "C as base but
  can click A. Just use the edge mode we already have."

## Layout (drawing)

ASCII mockup of the right inspector with the new sections. Hex codes annotate the major surfaces.

```
+-- PROPERTIES --------------------------------+   bg/surface  #EADFC8
| ...existing Nodes list + Node inspector...   |
| ...existing Reset-spec / Pivots collapsibles |
| -------------------------------------------- |   border      #D7C9AC (GRID)
| v EDGE OVERRIDES                      (2)     |   header text #2B2622 (INK), chevron #C56B45 (TERRACOTTA)
|   +--------------------------------------+    |
|   | B->G  [BLIND]  seek=on          e  x |    |   row text #2B2622; BLIND tag tinted #B0413E (BRICK)
|   | A->C  [LINE]   FAST              e  x |    |   LINE tag tinted #2B2622 (INK)
|   +--------------------------------------+    |
|   + add override                              |   ghost button: rest #6E6457, hover #C56B45
|   +-- editor (row "e" / add) -------------+   |
|   |  Edge    u[B v]  v[G v]               |   |   u/v comboboxes Editor.TCombobox
|   |  Type    BLIND   (auto, read-only)    |   |   type DERIVED from the existing segment, locked
|   |  Primitive  [ STOP_AT_LINE        v]  |   |   BlindMode tags for blind / LineMode tags for line
|   |  Direction  [ default             v]  |   |   default | front | rear
|   |  Stop@line  [ default             v]  |   |   default | on | off
|   |  (distance / heading: auto-derived)   |   |   NOT shown - geometry tracks node coords
|   |  [ save ]   [ cancel ]                |   |
|   +---------------------------------------+   |
| -------------------------------------------- |
| v PIPELINE DEFAULTS                           |   new collapsible
|   Line       [ NORMAL              v]         |   LineMode (no DEFAULT)
|   Blind      [ SYNC_TSHAPE         v]         |   BlindMode (no DEFAULT)
|   Turn       [ COUNTER_ROT         v]         |   TurnMode (no DEFAULT)
|   Direction  [ DEFAULT             v]         |   DirectionMode (DEFAULT allowed)
| -------------------------------------------- |
| v GLOBAL KNOBS  (existing, extended)          |
|   [x] Blind-decay   k[..] exp[..] dref[..]    |   existing
|   robot_radius_cm [ 9.0 ]                     |   existing
|   [x] blind_projection_auto_seek              |   NEW field-wide toggle
|   Field size  W[ 2362 ] H[ 1143 ] mm          |   NEW FIELD_MM (dimensions only; no MAP_PNG)
| -------------------------------------------- |
| ...existing Start pose + action buttons...    |
+----------------------------------------------+
```

## Description

- **EDGE OVERRIDES** is a new collapsible directly below the existing Pivots section. The header
  shows a count badge `(N)` like other sections. Each row reads `u->v  [TYPE]  <tag summary>` with
  an edit (`e`) and delete (`x`) affordance, exactly like a pivot row. "+ add override" opens the
  inline editor sub-frame (same pattern as the pivots add-row).
- **The inline editor** sets the edge by `u` / `v` comboboxes; **`type` is auto-derived** from the
  matching existing `LINE_SEGMENTS` / `BLIND_EDGES` pair and shown read-only (an operator cannot pick
  a type that conflicts with the segment — this prevents a Save-time anti-ambiguity rejection). The
  three override controls are `primitive_mode` (a combobox whose values switch on the derived type:
  `LineMode` tags for a line edge, `BlindMode` tags for a blind edge — `DEFAULT` is presented as
  "default (inherit)" meaning the key is omitted), `direction` (`default` / `front` / `rear`), and
  `stop_at_line` (`default` / `on` / `off`). Distance / heading are never shown — they are derived
  from node coordinates at build time, so an override always tracks the nodes (no stale geometry).
- **Edge-mode graft:** when the operator clicks an existing edge in the editor's current Edge mode,
  the panel scrolls to + highlights that edge's override row if one exists, otherwise pre-fills the
  add-row's `u` / `v` / `type` from the clicked segment so the operator can attach an override to the
  edge they just clicked. No new selection state — this rides the existing `_edge_selection`.
- **PIPELINE DEFAULTS** is a new collapsible with four comboboxes for the field-wide pipeline
  modes (`line` / `blind` / `turn` / `direction`). `line` / `blind` / `turn` cannot be `DEFAULT`
  (matches `PipelineConfig.__post_init__`); `direction` may be `DEFAULT`. Leaving them unset keeps the
  field absent and the planner falls back to `pipeline_config.py` defaults.
- **GLOBAL KNOBS** (existing) gains a `blind_projection_auto_seek` checkbutton and a `FIELD_MM`
  width/height entry pair. `MAP_PNG` is intentionally NOT surfaced (user: "Only need field dimensions").

## Color codes (existing theme tokens — no new colors)

| Token | Hex | Use |
|-------|-----|-----|
| PAPER | `#F3EAD8` | page / plotting background |
| PAPER_LIGHT | `#FAF4E6` | entry field background |
| SURFACE | `#EADFC8` | inspector panel surface |
| INK | `#2B2622` | body / header text, LINE tag |
| INK_MUTED | `#6E6457` | secondary text, ghost button at rest |
| TERRACOTTA | `#C56B45` | accent: chevrons, active/hover, selection ring |
| BRICK | `#B0413E` | BLIND edge tint, invalid badge |
| OCHRE | `#C8951F` | projection node tint (not used by edges) |
| TEAL | `#2A8C82` | reset node tint (not used by edges) |
| GRID | `#D7C9AC` | section dividers |
| VALID | `#5C8A3A` | validation-OK badge |

## Type & spacing tokens

- Fonts: `FONT_SANS` (IBM Plex Sans → Segoe UI) for labels/headers; `FONT_MONO` (IBM Plex Mono →
  Consolas) for numeric entries (distance/coords/dimensions). Title 16 pt, section header 9 pt
  small-caps, UI 10 pt — all from `theme.py`.
- Spacing: match the existing reset-spec / pivots rows (`side="top", fill="x"` rows; left label +
  right control; 6 px inter-control padding).

## States

- **Default:** EDGE OVERRIDES list shows existing override rows (or an empty hint "no edge overrides").
  PIPELINE DEFAULTS shows current values or the inherited default greyed.
- **Add/edit (focused):** inline editor sub-frame expanded; `type` read-only; combobox values
  context-switched by derived edge type.
- **Hover:** ghost "+ add override" and per-row `e`/`x` tint TERRACOTTA `#C56B45` on hover.
- **Loading (open file):** sections populate from the loaded `FieldModel`; absent keys → empty/inherit.
- **Empty:** no overrides → list shows the empty hint and the add-row only.
- **Error (Save-time validate fail):** the existing status-strip validation badge turns INVALID
  `#B0413E` with the validator's message; the offending override row is tinted BRICK. Obvious
  edit-time errors (e.g. an edge with no matching segment) are rejected inline before dispatch
  (mirrors the existing inline-reject pattern in `app.py`).

## Responsive

- The inspector is a fixed 312 px band; sections are vertically stacked and individually
  collapsible. The EDGE OVERRIDES list and PIPELINE DEFAULTS sections collapse to keep the band
  scannable on short windows (same `_toggle_collapsible` mechanism as today).
