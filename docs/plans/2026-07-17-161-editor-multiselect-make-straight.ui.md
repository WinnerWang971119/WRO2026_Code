# UI/UX design record — editor multi-select + "Make straight" (issue #161)

Sidecar for `2026-07-17-161-editor-multiselect-make-straight.md`. Phase 1.5 design gate,
`--auto` mode: three directions generated, the recommended base auto-picked, no grafts.

**Chosen base: Direction B — "Straightedge". Grafted in: none.**

---

## Brief constraints (these override any aesthetic instinct)

The editor is a **shipped Tk + matplotlib desktop app** with an established visual language in
`tools/path_planner/editor/theme.py` (653 lines). The brief pins the palette, so the design gate's
job here is **interaction design, not visual identity**. No new colour language, no new fonts, no
new spacing scale. Every token below already exists.

A note on calibration: the front-end-design skill warns that AI design clusters on "warm cream
background + terracotta accent". This editor *is* that look already — `PAPER #F3EAD8` +
`TERRACOTTA #C56B45` — but it is the shipped product's identity, authored before this task, not a
default reached for here. The brief's own words win: reuse it exactly.

### Existing tokens (`editor/theme.py`)

| Token | Hex | Existing role |
|---|---|---|
| `PAPER` | `#F3EAD8` | canvas / page background |
| `PAPER_LIGHT` | `#FAF4E6` | raised surface |
| `SURFACE` | `#EADFC8` | panel + ghost-button background |
| `INK` | `#2B2622` | primary text |
| `INK_MUTED` | `#6E6457` | section headers, captions |
| `TERRACOTTA` | `#C56B45` | `SELECTION_RING`, `ACCENT_BAR`, `DIRTY_DOT`, `GHOST_BUTTON_FG_ACCENT` |
| `TEAL` | `#2A8C82` | `PROPOSED_EDGE` — "this is a proposal, not a fact" |
| `BRICK` | `#B0413E` | `INVALID` |
| `OCHRE` | `#C8951F` | accent |
| `GRID` | `#D7C9AC` | grid lines |
| `SPINE` | `#9C8C72` | `HAIRLINE`, `GHOST_BUTTON_BORDER` |
| `VALID` | `#5C8A3A` | valid badge |
| `ACCENT_SOFT` | `#E7C9B8` | soft accent fill |
| chevrons | `◢` open / `▸` collapsed | collapsible section headers |

### Existing precedent worth cloning

- **Selection ring:** terracotta open circle — `s=140`, `linewidth=2.5`, `zorder=8` (`canvas.py:134-138`).
- **Obstacle rubber-band:** dotted rectangle, hardcoded `"blue"`, `linewidth=1.5`, `zorder=5`
  (`canvas.py:186-190`) with a full blit fast-path (`begin_obstacle_drag` / `update_obstacle_preview`
  / `end_obstacle_drag`). This is the structural template for the marquee. Its `"blue"` is
  **off-palette** — a pre-existing inconsistency, deliberately not touched by this work.
- **Ghost button:** `SURFACE` bg, `SPINE` 1px border, `INK_MUTED` fg, `TERRACOTTA` fg on hover.
- **Collapsible section:** `EDGE OVERRIDES` — chevron + `INK_MUTED` header + indented rows.

---

## Direction A — "Align section"

Follows existing precedent exactly. No new visual ideas; the safest possible read of the issue.

```
┌─ Field Editor ────────────────────────────────────────────────────┐
│ [Sel] Move  Add  Edge  Obst  Strt                    ⬤ valid      │
├────────────────────────────────────┬──────────────────────────────┤
│  canvas — PAPER #F3EAD8            │ NODES                        │
│                                    │ ┌──────────────────────────┐ │
│    ◎ M     ◎ MB2    ◎ MG2          │ │ M   MB2   MG2   MW2  ... │ │
│                                    │ └──────────────────────────┘ │
│   ╭┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈╮            │                              │
│   ┊  marquee, TERRACOTTA ┊         │ ◢ ALIGN                      │
│   ┊  #C56B45 dotted 1.5  ┊         │   6 nodes selected           │
│   ╰┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈╯            │   Axis  (•) Auto             │
│                                    │         ( ) X   ( ) Y        │
│   ◎ = selected (SELECTION_RING,    │         ( ) Best fit         │
│       existing token, unchanged)   │   [ Make straight ]          │
│                                    │ ▸ EDGE OVERRIDES             │
├────────────────────────────────────┴──────────────────────────────┤
│ Selected 6 nodes.                                                 │
└───────────────────────────────────────────────────────────────────┘
```

- **Marquee:** dotted `TERRACOTTA` rectangle, cloning the obstacle blit trio.
- **Selection:** existing terracotta rings, now list-valued at the render call site.
- **Control:** a collapsible `ALIGN` section — count label, axis radio, `Make straight` ghost button.
- **Feedback:** status line only.
- **Cost:** lowest. **Risk:** lowest.
- **Weakness — the disqualifying one:** the operator gets a button and no information. At the
  editor's zoom a 0.016 cm residual on a 185 cm field is sub-pixel: they cannot see that a run
  needs straightening, cannot see which node is the outlier, and cannot tell the click did
  anything. The feature's whole subject is a defect too small to see, and A renders none of it.

---

## Direction B — "Straightedge" ← **CHOSEN BASE**

A is the floor; B adds the one element that makes the invisible legible. The metaphor is the
drafting table the editor already evokes (paper ground, ink text, terracotta marks): lay a
straightedge across the run and show what it would touch.

```
┌─ Field Editor ────────────────────────────────────────────────────┐
│ [Sel] Move  Add  Edge  Obst  Strt                    ⬤ valid      │
├────────────────────────────────────┬──────────────────────────────┤
│  canvas — PAPER #F3EAD8            │ NODES                        │
│                                    │ ┌──────────────────────────┐ │
│    ◎ M      50.484                 │ │ M      ▓ selected        │ │
│    ┊                               │ │ MB2    ▓ selected        │ │
│    ◎ MB2    50.484                 │ │ MG2    ▓ selected        │ │
│    ┊  ← guide line, TEAL #2A8C82   │ │ MW2    ▓ selected        │ │
│    ○ MY2    50.500  ⇠ 0.0133 off   │ │ MY2    ▓ selected        │ │
│    ┊     ghost dot at target,      │ │ i1     ▓ selected        │ │
│    ◎ MG2    50.484   TEAL hollow   │ └──────────────────────────┘ │
│    ┊                               │                              │
│    ◎ i1     50.484                 │ ◢ ALIGN                      │
│                                    │   6 nodes selected           │
│   ╭┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈╮            │   Axis  (•) Auto — X         │
│   ┊  marquee, TERRACOTTA ┊         │         ( ) X   ( ) Y        │
│   ┊  #C56B45 dotted 1.5  ┊         │         ( ) Best fit         │
│   ╰┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈╯            │   ┌────────────────────────┐ │
│                                    │   │ max off-axis  0.0133cm │ │
│   ◎ = selected (SELECTION_RING)    │   │ x = 50.4867            │ │
│   ○ = ghost target (will move)     │   └────────────────────────┘ │
│                                    │   [ Make straight ]          │
│                                    │ ▸ EDGE OVERRIDES             │
├────────────────────────────────────┴──────────────────────────────┤
│ 6 nodes selected — max 0.0133 cm off axis.                        │
└───────────────────────────────────────────────────────────────────┘
```

> Note: under mean-snap all six nodes move to the shared mean `x = 50.4867`; the five near-`50.484`
> nodes shift ~0.0027 cm and their ghost dots coincide with their rings at this zoom, so only MY2's
> ghost is visibly displaced. The status reports the **max** residual, not a "1 off" count.

### The signature — the residual readout + ghost dots

This is the one memorable element, and the one risk taken. Everything else stays quiet.

- **`TEAL` guide line** through the selection at the resolved axis. `TEAL` is already
  `PROPOSED_EDGE` — the token *already means* "proposed, not actual". Reusing it for the guide is
  structure encoding truth, not decoration: same meaning, same colour.
- **Hollow `TEAL` ghost dots** at each mover's target. Under mean-snap essentially every node moves
  to the shared mean, so most ghosts sit within a pixel of their ring; the one visibly displaced
  ghost marks the outlier. A node already exactly at its target (only after a prior alignment) gets
  no ghost — so a genuine "nothing to fix here" is still visible before the operator commits.
- **Residual readout** — `max off-axis 0.0133cm` + the target coordinate. This is the number the
  operator cannot get any other way. It is also the number that makes the *result* verifiable: it
  reads `0.0000cm` after a successful alignment.
- **The `Auto` radio label reveals its own resolution** — `Auto — X`. Auto-detection that won't
  say what it detected is a magic box; naming the resolved axis makes the automation auditable, and
  costs one label.

### Why B over A and C

The brief's subject is a defect measured in hundredths of a centimetre. Any direction that renders
only a control and not the measurement leaves the operator clicking blind — unable to answer "does
this run need it?" or "did it work?". B answers both. Its cost is one extra static artist on a
render path that already exists, and one label — genuinely small for the only element that makes the
feature legible.

The restraint check (remove one accessory): an earlier pass had the guide line **animate** into
place on selection. Cut — the guide is a measuring instrument, and instruments do not perform. It
appears when the selection does, and holds still.

### States

- **Default (<2 selected):** section visible, controls disabled, readout blank.
  Status: `Select 2 or more nodes to align them.`
- **Ready (≥2 selected, residual > 0):** count + resolved axis + max residual + target coord;
  guide + ghosts drawn; button enabled.
- **No-op (≥2 selected, already collinear):** readout `max off-axis 0.0000cm`; no ghosts drawn;
  button enabled, and a click reports `Already straight. Nothing moved.` without burning an undo entry.
- **Hover / focus:** existing ghost-button styles (`GHOST_BUTTON_FG_ACCENT` terracotta on hover).
  Keyboard focus ring unchanged.
- **Refusal:** `Make straight: alignment would leave the field. Nothing moved.`
- **Marquee active:** dotted terracotta rectangle on the blit path; rings update only on release.
- **Success:** `Made 6 nodes straight on x = 50.4867.`

### Copy rules applied

Active voice; the action keeps its name end to end — the button says `Make straight`, the success
line says `Made 6 nodes straight`. The refusal states what happened and what did *not* happen
(`Nothing moved.`) rather than apologizing or going vague. The empty state is an instruction, not a
mood: `Select 2 or more nodes to align them.` Nothing does double duty — the label labels, the
readout measures, the status reports.

---

## Direction C — "Context menu"

Minimal. No new panel real estate; the editor already has a right-click context menu
(`_context_delete`).

```
┌─ Field Editor ────────────────────────────────────────────────────┐
│ [Sel] Move  Add  Edge  Obst  Strt                    ⬤ valid      │
├────────────────────────────────────┬──────────────────────────────┤
│    ◎ M                             │ NODES                        │
│    ◎ MB2   ┌────────────────────┐  │ ┌──────────────────────────┐ │
│    ◎ MY2   │ Make straight     ▸│  │ │ M   MB2   MG2   MW2  ... │ │
│    ◎ MG2   │ Delete         (6) │  │ └──────────────────────────┘ │
│    ◎ i1    └────────────────────┘  │                              │
│              ↳ Auto                │ ▸ EDGE OVERRIDES             │
│                Align X             │                              │
│                Align Y             │  (no new panel real estate)  │
│                Best fit            │                              │
├────────────────────────────────────┴──────────────────────────────┤
│ 6 nodes selected.                                                 │
└───────────────────────────────────────────────────────────────────┘
```

- **Cost:** lowest panel footprint; reuses the existing context menu.
- **Strength:** the action appears exactly where the selection is — no cross-screen trip to a panel.
- **Weakness:** same blindness as A (no residual, no preview), **plus** the axis choice is buried in
  a submenu the operator must open to discover, and a menu cannot show a live readout. Discoverability
  is also worse: nothing on screen advertises that make-straight exists.

---

## Decision

**Base: B — Straightedge. Grafts: none** (`--auto` takes the recommended base as-is).

C's "action at the selection" idea is genuinely good and is the obvious follow-up if operators find
the panel trip annoying — but it is additive to B (the same `_handle_align_request` would back a
context-menu entry), so nothing is lost by not grafting it now. A is fully contained in B.
