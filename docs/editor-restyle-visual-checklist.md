# Editor restyle — manual visual checklist

The de-lag restyle (Issue #78, wave W3: T10–T13) repaints the live field editor
with the warm "drafting-table" palette defined in
`tools/path_planner/editor/theme.py`. The automated guard
(`tests/test_editor_panels_smoke.py`, TC11) proves the panel *contract* survived
the restyle, but the *look* can only be eyeballed. Run the editor and walk this
list before signing off on AC9.

How to run:

```
uv run python tools/path_planner/edit_field.py
```

(Use the same field JSON you normally edit, or migrate `field_data.py` first.)

## Palette — no default grey anywhere (AC9)

- [ ] The right-hand control panels are the warm `SURFACE` cream, not the
  default ttk grey.
- [ ] The matplotlib map area (figure + axes facecolors, grid, spines, ticks)
  is the warm paper palette, not white/grey.
- [ ] The root window / chrome (window background behind the widgets) is warm,
  not default grey — there is no grey gutter peeking out between the canvas and
  the panels.

## Node + edge palette

- [ ] Line nodes render in `INK` (warm near-black).
- [ ] Blind nodes render in `BRICK` (warm red).
- [ ] Reset nodes render in `TEAL`.
- [ ] The selected node shows a `TERRACOTTA` selection ring.
- [ ] Line edges and blind edges are themed (line = ink, blind = brick), not a
  flat black/red.

## Fonts (fallback chains resolve; no hard-fail on a missing font)

- [ ] Display / heading text resolves down the chain
  (Fraunces → Cambria → Georgia → serif).
- [ ] UI text resolves down the sans chain
  (IBM Plex Sans → Segoe UI → Helvetica → sans-serif).
- [ ] Numeric fields (node x/y, max-edge-length, reset wall-coord / facing,
  global knobs, start heading) render in the mono chain
  (IBM Plex Mono → Cascadia Mono → Consolas → monospace).
- [ ] Nothing hard-fails or shows a `Tk` error if a preferred bundled font is
  absent — the editor still launches and reads cleanly on the generic tail.

## Mode toolbar

- [ ] The active mode row shows the `TERRACOTTA` accent strip on its left edge.
- [ ] Switching modes moves the accent strip to the newly active row and clears
  it from the previously active one (the inactive rows show a same-width
  surface-toned strip so the rows stay aligned).

## Status surface

- [ ] The status surface is visibly bigger / clearer than before and reads as a
  distinct sunk plane at the floor of the column.
- [ ] It shows the current mode line, the current selection line, the
  free-form status line, and the validation badge.
- [ ] The validation badge is the largest, boldest line; it turns `VALID`
  (green) / `INVALID` (brick) and is muted until the first validation result.

## Decisions to confirm (flagged during review)

These four points were called out during review as deliberate interpretations
that only a human eye can ratify. Confirm each is the intended look, or note
the requested change as a follow-up.

1. **Mode-toolbar interpretation.** T11/T13 implemented the "left mode toolbar"
   as a *conservative in-panel restyle*: a terracotta accent strip on the
   active mode's row inside the existing **Mode** panel. The window layout
   (matplotlib canvas on the left, control column on the right) is unchanged —
   there is no separate left-edge toolbar docked against the window frame.
   - [ ] Confirm the in-panel accent strip is the intended look, **or** note
     that a separate left-edge toolbar is wanted (would be a follow-up).

2. **Status font size.** `_FONT_SIZE_STATUS` currently equals the UI font size
   (`_FONT_SIZE_UI`); the code comment implied the status surface "reads a touch
   larger," but the value is the same. The badge is genuinely larger
   (`_FONT_SIZE_BADGE`).
   - [ ] Confirm the status-line size is acceptable, **or** request a bump so
     the whole surface reads larger than the body UI text.

3. **Read-only / disabled fields render on the `SURFACE` tone.** The read-only
   name entry and the disabled coord / reset / knob fields use `SURFACE`
   (`readonlybackground` / `disabledbackground`) so they recede into the panel,
   while editable fields sit on the lighter `PAPER_LIGHT`.
   - [ ] Confirm the read-only name field and the disabled coord fields still
     read as *fields* (boxes you could type into when enabled), not as flat
     labels that have lost their field affordance.

4. **Validation-badge lifecycle.** The badge turns green/`VALID` after a
   successful Save and **stays** `VALID` after a later successful inline edit,
   until the next Save. It only drops to `INVALID` on a rejected edit or a
   failed Save — it does not reset to a neutral/dirty state when the model is
   edited.
   - [ ] Confirm "stale-VALID-until-next-Save" is acceptable, **or** request a
     dirty → neutral reset so an unsaved edit no longer reads as validated.

## UI restyle + projection/reset (this change)

These items cover the restyle layout and the projection-edge / reset-autofill
interactions that the unit tests cannot eyeball. The logic underneath is unit
tested (`tests/test_editor_projection.py`, `tests/test_editor_reset_autofill.py`,
and the TC9/TC10 additions in `tests/test_editor_panels_smoke.py` /
`tests/test_editor_inspector.py`); this list is the look-and-feel sign-off.

- [ ] The window renders as a **3-pane layout** (map canvas, control column,
  status surface) and nothing overlaps or clips at the default window size.
- [ ] The **active mode** shows the terracotta accent bar on its row plus the
  mode tint, and the bar moves when you switch modes.
- [ ] The **Save & Build** button is visible (terracotta primary), and clicking
  it regenerates `WRO2026/routes.h` + `routes.cpp` (check the file mtimes or a
  `git status` after a build).
- [ ] The map background is the **warm cream** paper tone, not grey/white.
- [ ] The **add-node dialog offers `reset`** in the Type combo; picking `reset`
  and creating the node **auto-opens the reset section** with an **auto-filled
  wall coordinate + facing** already populated.
- [ ] Pick a source node, then a destination, and press **P**: a **teal
  projection arrow** is drawn from `u` to `v`.
- [ ] Pressing **?** or **F1** opens the **keyboard-shortcuts dialog**; it lists
  the bindings and dismisses on Escape / Close / the window "X".
- [ ] **Position-change reset recompute (deferred TC6 visual half):** with a
  reset node selected, **changing its axis** re-suggests a fresh wall + facing
  (the recompute), while **hand-editing only the wall or facing** (axis
  unchanged) keeps your typed values (no clobber). The dispatch logic for both
  branches is unit tested in `tests/test_editor_reset_autofill.py` (TC6); this
  checkbox is the on-screen confirmation that the panel reflects it.
