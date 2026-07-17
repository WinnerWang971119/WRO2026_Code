# Editor multi-node selection + "Make straight" alignment — Plan (issue #161)

**Goal:** Let the field-editor operator select several nodes at once (rubber-band marquee, shift/ctrl-click, or the node list) and snap them onto a shared line in one undoable action, so runs that are *meant* to be collinear are exactly collinear instead of a few hundredths of a centimetre off.

**Approach:** Extend the existing `Mode.SELECT` (never `Mode.MOVE`) with a marquee that clones the obstacle rubber-band's blit fast-path, and hold the selection in one ordered list behind a single writer funnel. Alignment geometry lives in a new pure, Tk-free `editor/align.py` (mirroring `auto_edges.py` / `projection_edges.py`); the editor computes the target coordinates and dispatches an existing `MacroCommand([MoveNode, ...])`, so **no new command class is needed** and one Ctrl-Z restores the whole run.

## Scope

- **In scope:**
  - Multi-node selection in `Mode.SELECT`: rubber-band marquee, shift-click (add), ctrl-click (toggle), plus an `EXTENDED` node listbox.
  - Selected nodes render with the existing terracotta `SELECTION_RING` (today `_selected_node` is **never drawn** — see Decisions).
  - A `Make straight` action: align the selection to a common X, a common Y, or a best-fit line, snapping to the **mean** of the selection.
  - Auto axis detection (smaller spread wins) with an explicit X / Y / Best-fit override.
  - A live guide line + ghost target dots + a max-residual readout (the Phase 1.5 "Straightedge" design).
  - **Multi-nudge**: arrow keys move the *whole* selection as a rigid body (user-requested; replaces the multi-drag/multi-delete/distribute options that were offered and declined).
  - Host-side tests: pure geometry, injected handlers (Style A), real-Tk marquee blit (Style B).

- **Out of scope:**
  - **Editing `field_data.json` / regenerating `routes.h`+`.cpp`.** User chose "tool only". No CI routes-freshness coupling in this PR. Straightening the real field's known-off columns is a separate change.
  - Multi-drag (dragging the selection with the mouse), multi-delete, distribute-evenly — offered, explicitly not selected.
  - Re-deriving `distance_cm` / `heading_deg` on `EDGES` overrides (see Decisions — pre-existing, inherited).
  - Any change to `Mode.MOVE`, `Mode.OBSTACLE`, or the obstacle rubber-band's off-palette `"blue"`.
  - `commands.py` changes — `MacroCommand` + `MoveNode` already compose the operation.

## Decisions

- **Snap target = mean of the selection** — user-selected. Minimizes total travel and privileges no node. Consequence, accepted: every selected node's coordinate changes in the JSON diff, and the run lands on a value nobody measured.
- **Axis = auto-detect + override** — user-selected. `AUTO` picks the axis with the **smaller spread** (a column snaps X, a row snaps Y). Exact tie → `X`, deterministic and documented.
- **Scope = make-straight + multi-nudge only** — user-selected via a write-in ("multimove up down left right by arrow keys"). Multi-drag / multi-delete / distribute dropped.
- **Field data untouched** — user-selected "tool only".
- **`FIT` = orthogonal (total-least-squares) regression through the centroid** — chosen because the TLS line passes through the mean by construction, so `FIT` is the same "snap to the mean" rule generalized to a diagonal. On an axis-aligned run it is *identical* to `X`/`Y`; on exactly 2 nodes it is a **no-op** (the line already passes through both, so each rounded target equals its 4dp-clean input) — surfaced as a status message, not an undo entry.
- **`FIT` moves the endpoints too; the issue's literal "best-fit line through the endpoints" is deliberately not implemented** — auto-resolved. The issue text says "a best-fit line **through the endpoints**", but the user's chosen snap target is the *mean of the selection*, which pins the centroid, not the extremes. Anchoring on the two endpoints (the "Line through the endpoints" option offered during planning) was declined. `FIT` therefore projects every node — endpoints included — onto the TLS line through the centroid, consistent with mean-snap.
- **Rounding is applied to the ALIGNED coordinate(s) only, to 4 dp (`ALIGN_ROUND_DP = 4`)** — auto-resolved. `align-X` rounds only x and copies y through byte-for-byte; `align-Y` rounds only y; `FIT` rounds both projected coords. Rounding *every* coordinate would corrupt the untouched axis (real y values are full-precision floats, e.g. `MB2.y = 60.95324525661604`) and violate AC5. Rationale for 4 dp: the raw mean is `50.48667530970902`; unrounded values churn the JSON diff, and 1e-4 cm is ~1 µm — far below any physical or planner-relevant scale. For `X`/`Y`/`AUTO` every node receives the *identical* rounded scalar → exactly collinear. For `FIT`, rounding leaves ≤1e-4 cm residual ⇒ <0.0003° over a 20 cm edge, ~1700× under `NAV_TURN_TOLERANCE_DEG`.
- **Under mean-snap, essentially every node is a mover — this is intended, not a rounding artifact.** The 5 "aligned" nodes on the real x≈50.5 column sit at `x = 50.484010371650825`; the mean target is `50.4867`, so they move too (by ~0.0027 cm). The `max off-axis` residual (the outlier MY2's `0.0133 cm`) is the headline signal, and the "Already straight. Nothing moved." no-op state fires only when every node already equals the rounded target — rare on real data, common right after a prior make-straight. The ghost dots of near-stationary movers coincide with their rings at any practical zoom, so the *visibly* displaced ghost marks the true outlier.
- **Make-straight refuses rather than shears** — auto-resolved. For `X`/`Y`/`AUTO` the mean of in-bounds values is provably in-bounds, so a clamp can never fire. For `FIT`, if any projected point would land outside the field, **abort the whole operation** with a status message and move nothing. Silently clamping one node would break the exact-collinearity guarantee the feature exists to provide.
- **Multi-nudge clamps the delta as a rigid body, never per-node** — auto-resolved, and load-bearing. `_nudge_selected` clamps each node independently today; applied to a selection, a single node hitting a wall would shear the run and *un-straighten* what make-straight just fixed. The effective delta is clamped once against the selection's bounding box; if it collapses to zero, no undo entry is burned.
- **Selection state = one ordered list behind one writer** — `_selected_nodes: list[str]` with the invariant `_selected_node == _selected_nodes[-1] or None`. All four existing `_selected_node` writers are rewritten to funnel through `_set_selection()`. Rejected: a `@property`-backed `_selected_node` (cleaner on paper, but `EditorApp.__new__` injection tests never run `__init__`, so the getter would `AttributeError`); rejected: a `Selection` class (speculative abstraction for one consumer, against the repo's plain style).
- **Multi-select extends `Mode.SELECT` only** — auto-resolved. `Mode.MOVE`'s left-click arms a node drag; overloading it with a marquee collides. `SELECT` already means "latch, don't move".
- **Reset-typed nodes are moved like any other node, with no extra pre-check** — auto-resolved, matching precedent: `_finish_move`, `_nudge_selected`, and the inspector coord entry all skip `_warn_if_reset_too_close`. `_refresh_view` → `_revalidate()` runs the full validator live on every edit, so an invalid wall spec surfaces in the validation badge immediately.
- **The issue's "Scope" bullets carry two stale premises** — recorded so the executor doesn't chase them:
  - *"mirror the moved coordinates into both serialization paths (`to_json_dict` + `to_module_proxy`)"* — nothing to do. `FieldModel.nodes` is the **single** holder of x/y; both paths are pure pass-through. Writing `model.nodes[name] = (x, y)` is complete.
  - *"Keep EDGES/projection-edge deltas in sync when node positions move"* — nothing to do. `ProjectionDelta` was removed in issue #64 and `_pending_renames`/`ast_rewrite` retired in #79 (verified: every remaining grep hit is a docstring recording the retirement). `EDGES` entries key on `(type, u, v)`, which a coordinate move never changes.
- **Honest rationale correction.** The issue says these residuals "show up downstream as sub-tolerance turns on edges that should be straight." Measured against the real `field_data.json`: the x≈50.5 column is off by 0.016 cm (**0.014°** over its 65.9 cm run) and the x≈117.7 column by 0.047 cm (**0.077°** over 35.5 cm). Both are far **under** `NAV_TURN_TOLERANCE_DEG` (0.5°), so today they are already treated as pass-throughs and do **not** split runs. The real value is (a) authoring hygiene — the data says what the operator meant; (b) removing the latent hazard in `gotcha-axis-aligned-fixtures-hide-subtolerance-turns`, where any collinearity gate using an epsilon tighter than 0.5° mis-splits a run on the real field while every axis-aligned toy fixture stays green. Do not oversell this as a live bug fix.

## Acceptance Criteria

- [ ] AC1 — In `Mode.SELECT`, dragging on empty canvas draws a dotted terracotta marquee and, on release, selects every node whose coordinate falls inside the box.
- [ ] AC2 — Shift-click adds a node to the selection; ctrl-click toggles one; a plain click on a node replaces the selection with just that node; a plain click on empty canvas clears it.
- [ ] AC3 — Every selected node draws the terracotta `SELECTION_RING` on the canvas (this also fixes the pre-existing gap where single `Mode.SELECT` selection was invisible).
- [ ] AC4 — With ≥2 nodes selected, the `ALIGN` inspector section is enabled and reports the live count, the resolved axis, and the max residual in cm; with <2 selected it is disabled.
- [ ] AC5 — `Make straight` on the real x≈50.5 column (`M, MB2, MG2, MW2, MY2, i1`) with `AUTO` sets every node's x to exactly `50.4867` and leaves every y untouched.
- [ ] AC6 — `Make straight` is exactly **one** undo entry: one Ctrl-Z restores every moved node's original coordinates byte-for-byte.
- [ ] AC7 — `Make straight` that would move nothing (already collinear, or `FIT` on 2 nodes) reports a status message and creates **no** undo entry.
- [ ] AC8 — `AUTO` resolves to align-X when `spread_x <= spread_y`, else align-Y; an exact tie resolves to X.
- [ ] AC9 — Arrow keys move **all** selected nodes by the same delta as one undo entry; when the delta is clamped by the field edge, every node shifts by the *same* reduced delta (relative geometry preserved — no shear).
- [ ] AC10 — A `FIT` alignment that would push any node outside the field bounds moves **nothing** and reports it.
- [ ] AC11 — Only the aligned axis is rounded (to 4 dp); for `X`/`Y`/`AUTO` all aligned nodes share one identical scalar and every untouched-axis coordinate is byte-identical to input.
- [ ] AC13 — When ≥2 nodes are selected with a nonzero residual, the canvas draws a teal guide line and hollow teal ghost dots at the movers' targets; when the selection drops below 2 or is cleared, both disappear.
- [ ] AC14 — After undoing a make-straight, the ALIGN readout, guide line, and ghost dots recompute to reflect the reverted model (they do not show a stale post-alignment residual).
- [ ] AC12 — `uv run --extra dev pytest tools/path_planner/tests/` passes, including under `xvfb-run` as CI invokes it.

## UI/UX Specification

> Carried verbatim from the Phase 1.5 design gate. The executor reproduces THIS — do not
> re-design. Full raw output lives in the sidecar
> `docs/plans/2026-07-17-161-editor-multiselect-make-straight.ui.md`.

**Chosen base:** Direction B — "Straightedge"
**Grafted in:** none (`--auto` mode auto-picks the recommended base with no graft)

**Why B:** the residuals this feature exists to remove are 0.016–0.047 cm on a ~185 cm field — invisible at any practical zoom. Directions A and C give the operator a button but no way to know a run *needs* straightening or that it *worked*. B's residual readout + ghost target dots make an invisible problem legible, at the cost of one extra artist on the existing blit path.

**Palette is pinned by the shipped `editor/theme.py`. No new colour language.** One deliberate deviation: the node marquee uses `TERRACOTTA`, not the obstacle rubber-band's hardcoded `"blue"` — a different colour encodes a different verb (select vs. create). The obstacle box is not touched.

### Layout (drawing)

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

(All six snap to the mean `x = 50.4867`; the five near-`50.484` nodes move ~0.0027 cm — their ghost dots land on their own rings at this zoom, so only MY2's ghost is visibly displaced. The `max off-axis` readout, not a node count, is the honest signal.)

### Description

- **Marquee.** Press on empty canvas in `Mode.SELECT` arms it; motion updates a dotted `TERRACOTTA` rectangle on the blit fast-path; release selects every node inside and tears the artist down. Beat-for-beat the obstacle rubber-band lifecycle (`begin_obstacle_drag` / `update_obstacle_preview` / `end_obstacle_drag`), with a different colour and a different verb.
- **Selection rings.** Every selected node gets the existing terracotta `SELECTION_RING` — `canvas.render(highlighted_nodes=[...])` and `_draw_highlight` are already list-valued, so this is a one-line union at the single `render` call site.
- **Guide line.** A `TEAL` line — same token as `PROPOSED_EDGE`, and the same meaning: *this is where things would go, not where they are.* For `X`/`Y` it spans the selection's extent on the other axis; for `FIT` it spans the projected extremes.
- **Ghost dots.** Hollow `TEAL` dots at each mover's target. Under mean-snap essentially *every* node moves (to the shared mean), so most ghosts sit within a pixel of their own ring — the one **visibly** displaced ghost marks the true outlier, and the `max off-axis` readout is the primary quantitative signal. A node already exactly at its target (only after a prior alignment) gets no ghost.
- **ALIGN section.** A collapsible inspector section (`◢`/`▸` chevrons, matching `EDGE OVERRIDES`). Live count, an axis radio whose `Auto` label reveals the resolved axis (`Auto — X`), a readout box with the max residual and the target coordinate, and a ghost `Make straight` button. Disabled below 2 nodes.
- **Copy.** Active voice, the action keeps its name: the button says `Make straight`, the status line after it says `Made 6 nodes straight on x = 50.4867.` A refusal states what happened and why: `Make straight: alignment would leave the field. Nothing moved.` Empty state: `Select 2 or more nodes to align them.`

### Colour codes

All tokens already exist in `editor/theme.py`. **No new values.**

| Token | Hex | Use here |
|---|---|---|
| `theme.PAPER` | `#F3EAD8` | canvas background (unchanged) |
| `theme.SURFACE` | `#EADFC8` | ALIGN section + ghost-button background |
| `theme.SELECTION_RING` / `TERRACOTTA` | `#C56B45` | selection rings; marquee edge |
| `theme.TEAL` (= `PROPOSED_EDGE`) | `#2A8C82` | guide line; ghost target dots |
| `theme.INK` | `#2B2622` | readout values, labels |
| `theme.INK_MUTED` | `#6E6457` | section header, residual caption |
| `theme.SPINE` | `#9C8C72` | readout box hairline, ghost-button border |
| `theme.VALID` / `INVALID` | `#5C8A3A` / `#B0413E` | status badge (unchanged) |

### Type & spacing tokens

Inherited from `theme.py` / the existing ttk styles — the ALIGN section reuses `EDGE OVERRIDES`' header style, row padding, and chevron glyphs (`◢` open, `▸` collapsed). No new fonts, sizes, or spacing values.

### States

- **Default (<2 selected):** section visible, controls disabled, readout blank, status `Select 2 or more nodes to align them.`
- **Ready (≥2 selected, residual > 0):** count + resolved axis + max residual + target coord; guide + ghosts drawn; button enabled.
- **No-op (≥2 selected, already at the rounded target):** readout shows `max off-axis 0.0000cm`; no ghosts drawn; button enabled but a click reports `Already straight. Nothing moved.` and burns no undo entry.
- **Success:** status `Made 6 nodes straight on x = 50.4867.`
- **Hover / focus:** existing ghost-button styles (`GHOST_BUTTON_FG_ACCENT` terracotta on hover). Keyboard focus ring unchanged.
- **Refusal:** status `Make straight: alignment would leave the field. Nothing moved.`
- **Marquee active:** dotted terracotta rectangle; rings update only on release.

### Responsive

Not applicable — fixed-layout Tk desktop window. The ALIGN section follows the existing 3-pane inspector column and collapses with its chevron like every sibling section.

## Data Model

No schema change. `field_data.json` is untouched by this PR (Decisions: "tool only").

`FieldModel.nodes: dict[str, tuple[float, float]]` is the **single** holder of node coordinates; `to_json_dict` and `to_module_proxy` are pure pass-through. A make-straight writes `model.nodes[name] = (x, y)` via `MoveNode` and nothing else needs updating.

## Contracts & Interfaces

Single source of truth for every seam. A task that touches a seam conforms to the exact name and signature here; it does not redefine it.

### Shared types — `editor/align.py` (NEW, pure, no Tk/matplotlib import)

```python
class AlignAxis(Enum):
    AUTO = "auto"
    X = "x"
    Y = "y"
    FIT = "fit"

ALIGN_ROUND_DP: int = 4

@dataclass(frozen=True)
class AlignPreview:
    """Render payload for the guide line + ghost targets. None when <2 nodes."""
    guide: tuple[tuple[float, float], tuple[float, float]]  # (x0,y0),(x1,y1) in cm
    ghosts: dict[str, tuple[float, float]]                  # movers only -> target xy
```

### Signatures

| Signature | Owner | Consumers |
|---|---|---|
| `resolve_axis(points: dict[str, tuple[float,float]], axis: AlignAxis) -> AlignAxis` — resolves `AUTO` to `X`/`Y` by smaller spread; tie → `X`. `X`/`Y`/`FIT` pass through. | T1 | T5, T6 |
| `align_points(points: dict[str, tuple[float,float]], axis: AlignAxis) -> dict[str, tuple[float,float]]` — target coords for **every** input node (unchanged ones included). Rounds **only the aligned axis** to `ALIGN_ROUND_DP`: `X` rounds x and copies y verbatim, `Y` rounds y and copies x verbatim, `FIT` rounds both projected coords. Raises `ValueError` if `len(points) < 2`. | T1 | T5, T6 |
| `max_residual_cm(points: dict[str, tuple[float,float]], axis: AlignAxis) -> float` — largest Euclidean distance any node must travel. Drives the readout. | T1 | T5, T6 |
| `build_preview(points: dict[str, tuple[float,float]], axis: AlignAxis) -> AlignPreview \| None` | T1 | T4, T5 |
| `FieldCanvas.find_nodes_in_rect(x0: float, y0: float, x1: float, y1: float) -> list[str]` — inclusive box test over `_last_model.nodes`, returned in model dict order. `[]` if `render` never ran. | T2 | T5, T8 |
| `FieldCanvas.begin_marquee_drag() -> None` / `update_marquee_preview(x0, y0, x1, y1) -> None` / `end_marquee_drag() -> None` — clones the obstacle rubber-band trio; `TERRACOTTA`, dotted, `linewidth 1.5`, own zorder + own blit background. | T2 | T5, T8 |
| `FieldCanvas.render(model, *, highlighted_nodes: Optional[list[str]] = None, proposed_edges: Optional[list[tuple[str,str,str]]] = None, align_preview: AlignPreview \| None = None) -> None` — params are **keyword-only** (the real signature already has `*`; do not change the existing two defaults, both `None`); `align_preview` is the one new keyword-only param, default `None`. Existing call sites unaffected. | T2 | T4 |
| `FieldCanvas.on_click: Optional[Callable[[float, float, int, Optional[str]], None]]` — **widened**: 4th arg is matplotlib's `event.key` (`None`, `"shift"`, `"control"`, …). | T2 | T4, T5 |
| `FieldCanvas.on_release: Optional[Callable[[float, float, int, Optional[str]], None]]` — **widened** identically. | T2 | T4, T5 |
| `SidePanels.on_align_request: Optional[Callable[[str], None]]` — fired by the `Make straight` button. Arg is an `AlignAxis` **value string** (`"auto"`/`"x"`/`"y"`/`"fit"`), never the enum (panels must not import `align`). | T3 | T5 |
| `SidePanels.on_axis_change: Optional[Callable[[str], None]]` — fired when the axis radio flips. Same value-string arg. Lets the app recompute the readout, `Auto — <resolved>` label, guide line, and ghosts for the newly chosen axis *without* running an alignment. | T3 | T5 |
| `SidePanels.on_nodes_select: Optional[Callable[[list[str]], None]]` — full listbox selection, listbox order. | T3 | T5 |
| `SidePanels.set_align_state(count: int, axis_label: str, residual_cm: Optional[float], target_label: str, enabled: bool) -> None` — writes the ALIGN readout. `residual_cm=None` blanks it. | T3 | T5 |
| `EditorApp._align_axis: AlignAxis` — the operator's current axis choice (default `AlignAxis.AUTO`), updated by the `on_axis_change` handler. Read wherever ALIGN state / preview is computed. | T5 | T5 |
| `EditorApp._compute_align_state() -> None` — recomputes `set_align_state` + the `align_preview` from the **live** `_selected_nodes` and `_align_axis`. Called from `_refresh_view()` so undo/redo, deletes, and any model change keep the readout and overlay fresh (fixes the stale-readout-after-undo hazard). | T4 | T5 |
| `EditorApp._set_selection(names: list[str]) -> None` — **the only writer** of selection state. Dedupes preserving order, sets `_selected_nodes` + `_selected_node` (`= names[-1] or None`), mirrors `_panels._selected_node_var`, updates the selection display, calls `_refresh_view()` (which in turn calls `_compute_align_state()`). | T4 | T5 |

### File ownership

| File | Owner task | Consumer tasks |
|---|---|---|
| `tools/path_planner/editor/align.py` (new) | T1 | T4, T5, T6 |
| `tools/path_planner/editor/canvas.py` | T2 | T5, T8 |
| `tools/path_planner/editor/panels.py` | T3 | T5 |
| `tools/path_planner/editor/app.py` | T4, then T5 (**strictly serialized — T5 blocked by T4**) | — |
| `tools/path_planner/tests/test_editor_align.py` (new) | T6 | — |
| `tools/path_planner/tests/test_editor_multiselect.py` (new) | T7 | — |
| `tools/path_planner/tests/test_editor_marquee_blit.py` (new) | T8 | — |
| `tools/path_planner/tests/test_editor_canvas_blit.py` (existing — update 3-arg `on_release` injection) | T2 | — |
| `tools/path_planner/tests/test_editor_select_mode.py` (existing — update `_on_canvas_click` call signature) | T5 | — |

`commands.py`, `model.py`, `history.py`, `theme.py`, `field_data.json`, `routes.h/.cpp` — **not touched by any task.**

### Naming (exact strings that must match across tasks)

- `AlignAxis` values: `"auto"`, `"x"`, `"y"`, `"fit"` — the wire form between `panels` and `app` (both `on_align_request` and `on_axis_change`).
- `EditorApp._selected_nodes` (list), `EditorApp._selected_node` (primary), `EditorApp._set_selection`, `EditorApp._align_axis`, `EditorApp._compute_align_state`.
- `EditorApp._marquee_anchor`, `EditorApp._marquee_cursor` — mirroring `_obstacle_anchor` / `_obstacle_cursor`; both **must** be registered in `_clear_transient_state` (`app.py:1782-1806`).
- Status strings, verbatim: `Made {n} nodes straight on {axis} = {value}.` / `Made {n} nodes straight on the best-fit line.` (FIT — no single aligned scalar to substitute into the `{axis} = {value}` form) / `Already straight. Nothing moved.` / `Make straight: alignment would leave the field. Nothing moved.` / `Select 2 or more nodes to align them.`

## Error Handling

- **<2 nodes selected:** ALIGN controls disabled; `align_points` raises `ValueError` if called anyway — caught by `_dispatch`'s existing `(ValueError, KeyError, IndexError)` net and surfaced on the status line.
- **Alignment is a no-op** (already collinear, or `FIT` on 2 nodes): dispatch **nothing**. Status `Already straight. Nothing moved.` No undo entry (mirrors `_finish_move`'s and `_nudge_selected`'s skip-if-unchanged discipline).
- **`FIT` would push a node out of bounds:** abort entirely, move nothing, status `Make straight: alignment would leave the field. Nothing moved.` Never partially clamp — that would shear the run.
- **Multi-nudge delta clamps to zero** (selection already flush against the wall): no dispatch, no undo entry.
- **A selected node was deleted underneath the selection:** `_set_selection` filters against `model.nodes` before use; `MoveNode.apply` raises `KeyError` on an unknown name, `MacroCommand.apply` rolls back the applied prefix, `_dispatch` catches it and reports. Model is never left half-moved.
- **Marquee released outside the axes:** reuse `_clamped_world_coords` (`canvas.py:928`), same as the obstacle path (#61).
- **Reset node moved into its wall:** not pre-checked (matches every existing move path). `_refresh_view` → `_revalidate()` flags it live in the validation badge.

## Testing Strategy

**Levels:** Unit (pure geometry), Integration (injected handlers), GUI smoke (real Tk).

| ID | Test Case | Type | Expected Behavior |
|---|---|---|---|
| TC1 | `resolve_axis` on the real x≈50.5 column (`spread_x=0.016`, `spread_y=65.92`) with `AUTO` | Unit | Returns `AlignAxis.X` |
| TC2 | `resolve_axis` with `spread_x == spread_y` exactly | Unit | Returns `AlignAxis.X` (documented tie-break) |
| TC3 | `align_points` on the real column, `AUTO` | Unit | Every node's x is exactly `50.4867`; every y **byte-identical** to the full-precision input (proves round-only-aligned-axis) |
| TC4 | `align_points` with `X` on 4dp-clean nodes already sharing one x (e.g. all at `x=50.0`) | Unit | Returns coords equal to input (the caller detects a no-op); a full-precision shared-x set instead grid-snaps to the rounded mean and is **not** a no-op — assert that too |
| TC5 | `align_points` with `FIT` on exactly 2 4dp-clean nodes | Unit | Returns both inputs unchanged (line passes through both) |
| TC6 | `align_points` with `FIT` on a diagonal run with one outlier | Unit | Outlier projects onto the TLS line; line passes through the centroid |
| TC7 | `align_points` rounding | Unit | Every returned coord equals `round(v, 4)`; for `X`/`Y` all share one identical scalar |
| TC8 | `align_points` with 1 node / 0 nodes | Unit | Raises `ValueError` |
| TC9 | `max_residual_cm` on the real x≈117.7 column with `AUTO`, residual measured to the **raw (unrounded) mean** | Unit | `≈0.0288465` within 1e-6 (pin the reference as the raw mean, not the rounded target — they differ by ~1.2e-5) |
| TC10 | `build_preview` on an already-collinear selection | Unit | `ghosts == {}` (nothing to draw) |
| TC11 | `find_nodes_in_rect` covering 3 of 6 nodes | Unit (Style A, fake model) | Returns exactly those 3, in model dict order |
| TC12 | `find_nodes_in_rect` with inverted box (x1<x0, y1<y0) | Unit | Normalizes; same result as the non-inverted box |
| TC13 | `_set_selection(["A","B","C"])` | Style A | `_selected_nodes == ["A","B","C"]`, `_selected_node == "C"`, panel var == `"C"` |
| TC14 | `_set_selection([])` | Style A | `_selected_nodes == []`, `_selected_node is None`, panel var == `""` |
| TC15 | `_set_selection` with a duplicate (`["A","B","A"]`) | Style A | Dedupes preserving **first** order → `["A","B"]` … assert the documented rule |
| TC16 | `_set_selection` naming a node absent from the model | Style A | Filtered out; never reaches `_selected_nodes` |
| TC17 | Shift-click on an unselected node | Style A | Appends; prior selection retained |
| TC18 | Ctrl-click on an already-selected node | Style A | Removes it; the rest retained |
| TC19 | Plain click on a node with 3 already selected | Style A | Replaces selection with just that node |
| TC20 | Plain click on empty canvas | Style A | Clears the selection |
| TC21 | Marquee release selecting 4 nodes | Style A | `_set_selection` called with those 4; `end_marquee_drag` called exactly once |
| TC22 | Make-straight on 6 nodes | Style A | `_dispatch` called once with a `MacroCommand` holding exactly 6 `MoveNode`s |
| TC23 | Make-straight when already collinear | Style A | `_dispatch` **not** called; status `Already straight. Nothing moved.` |
| TC24 | Make-straight `FIT` that would exit the field | Style A | `_dispatch` not called; status is the refusal string; model unmutated |
| TC25 | Undo after make-straight | Unit (real `History` + real `FieldModel`) | One `undo()` restores all 6 original coords exactly |
| TC26 | `MacroCommand` rollback when one `MoveNode` names a deleted node | Unit | Raises; every already-applied node restored to its original coord |
| TC27 | Arrow-key nudge with 3 selected, clear of the walls | Style A | One `MacroCommand` of 3 `MoveNode`s, all shifted by the identical delta |
| TC28 | Arrow-key nudge where one node is flush against x=0, nudging left | Style A | **Rigid**: delta clamps to 0 → no dispatch. Asserts no shear (the regression guard for AC9) |
| TC29 | Arrow-key nudge where the delta partially clamps | Style A | Every node moves by the same reduced delta; relative spacing byte-identical |
| TC30 | Arrow-key nudge with exactly 1 selected | Style A | Behaves exactly as today (single `MoveNode`, not a `MacroCommand`) — back-compat guard |
| TC31 | `_clear_transient_state` after a mode switch mid-marquee | Style A | `_marquee_anchor` / `_marquee_cursor` are `None`; selection preserved |
| TC32 | `render` call site | Style A | `highlighted_nodes` is the union of `_edge_selection` and `_selected_nodes` |
| TC33 | Marquee blit lifecycle | **Style B (`tk_root`)** | `begin_` adds exactly one animated artist; `update_` blits without a full redraw; `end_` removes it and leaves the axes artist count unchanged |
| TC34 | `SidePanels` ALIGN section: construction + control wiring | **Style B (`tk_root`)** | Section builds; controls disabled at 0 selected; `set_align_state(6, "Auto — X", 0.0133, "x = 50.4867", True)` populates and enables; clicking `Make straight` fires `on_align_request` with the current radio value; flipping the radio fires `on_axis_change` with the new value |
| TC35 | Listbox `EXTENDED` round-trip | **Style B (`tk_root`)** | Selecting 3 rows fires `on_nodes_select` with those 3; `refresh(model)` restores all 3 highlighted and fires **no** callback (`_suppress_callbacks`) |
| TC36 | `render(align_preview=...)` draws then clears the overlay | **Style B (`tk_root`)** | With a preview, one teal guide-line artist + one ghost-dot collection are added; `render(align_preview=None)` leaves neither (verifies AC13) |
| TC37 | ALIGN readout after undo of a make-straight | Style A (fake canvas/panels recording `set_align_state` args) | After `_dispatch(make-straight)` then `_undo()`, `_compute_align_state` fires again and the recorded residual reflects the reverted model, not the post-alignment `0.0` (verifies AC14) |

**Test data:** the pure-geometry cases (TC1–TC10) use the **real** coordinates from `field_data.json` (the x≈50.5 and x≈117.7 columns) as literals — they are the actual motivating defect, and per `gotcha-axis-aligned-fixtures-hide-subtolerance-turns` an axis-aligned toy fixture would hide exactly the sub-tolerance behaviour this feature targets. Include at least one **diagonal** fixture for `FIT`. Handler tests use `EditorApp.__new__(EditorApp)` + `SimpleNamespace` + the `_FakeVar` helper (copy it from `test_editor_select_mode.py:53`). Real-Tk tests take the **shared** `tk_root` from `tests/conftest.py` — never define a new root.

**Run command:** `uv run --extra dev pytest tools/path_planner/tests/` (CI runs it under `xvfb-run -a`).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Alignment geometry module | — | med | `tools/path_planner/editor/align.py` (new) | Create the pure, Tk/matplotlib-free module. Implement `AlignAxis`, `ALIGN_ROUND_DP=4`, `AlignPreview`, `resolve_axis`, `align_points`, `max_residual_cm`, `build_preview` exactly per Contracts & Interfaces. `AUTO`: `spread_x <= spread_y` → X else Y. `X`/`Y`: set the axis coord to `round(mean, 4)`. `FIT`: orthogonal/total-least-squares line through the centroid — `theta = 0.5*atan2(2*Sxy, Sxx-Syy)`, direction `u=(cos θ, sin θ)`, project `p' = c + ((p-c)·u)u`, then round. Raise `ValueError` for `len(points) < 2`. Module docstring must state the tie-break, the 4 dp rule, and that `FIT` on 2 nodes is a no-op. Follow `auto_edges.py` / `projection_edges.py` for style. Satisfies AC5, AC8, AC11. |
| T2 | Canvas: modifier plumbing, marquee, rect hit-test, align preview | — | med | `tools/path_planner/editor/canvas.py`, `tools/path_planner/tests/test_editor_canvas_blit.py` | (a) Widen `on_click` / `on_release` to `(x, y, button, key)` — pass `event.key` from `_on_button_press` / `_on_button_release` (~`canvas.py:979`). Update the internal call sites **and** the existing test that injects a 3-arg `on_release` (`test_editor_canvas_blit.py:421` sets `lambda x, y, b: …` and asserts a 3-tuple — it will `TypeError` after the widen; update it to the 4-arg form and 4-tuple assertion). (b) Add `find_nodes_in_rect` (inclusive, normalizes inverted boxes, model dict order, `[]` before first render). (c) Add `begin_marquee_drag` / `update_marquee_preview` / `end_marquee_drag` cloning the obstacle rubber-band trio (`begin_obstacle_drag` `canvas.py:608` / `update_obstacle_preview` `canvas.py:661` / `end_obstacle_drag` `canvas.py:723`) with its own preview artist, own blit background, and own `_MARQUEE_PREVIEW_*` constants: edgecolor `theme.SELECTION_RING`, linestyle `":"`, linewidth `1.5`, its own zorder. **Do not** touch the obstacle path or its `"blue"`. (d) Add keyword-only `align_preview: AlignPreview | None = None` to `render`; when set, draw the `theme.TEAL` guide line and hollow `theme.TEAL` ghost dots at each target (static artists in the normal render, not the blit path). Satisfies AC1, AC3 (rendering half), AC13. |
| T3 | Panels: ALIGN section + EXTENDED listbox | — | high | `tools/path_planner/editor/panels.py` | (a) Add the collapsible `ALIGN` section per the UI/UX Specification: count label, axis radio (`Auto — <resolved>` / `X` / `Y` / `Best fit`), readout box (max residual + target coord), `Make straight` ghost button. Reuse `EDGE OVERRIDES`' header/chevron/padding styles. (b) Add the `on_align_request` (button), `on_axis_change` (radio flip), and `on_nodes_select` callback slots and `set_align_state` per Contracts. The radio must fire `on_axis_change` with the value string on every change so the app can refresh the readout/preview without running an alignment. (c) Switch `_node_listbox` from `tk.SINGLE` to `tk.EXTENDED` (`panels.py:1478`) and extend `refresh` (`panels.py:2403-2532`) to restore the **full** selection, not just `_selected_node_var`. **Danger:** `refresh` must stay wrapped in `_suppress_callbacks` (it must never fire `on_*`), and the `_edges_signature` dirty-check that skips override-row rebuild on a pure node move (R11 / `test_editor_override_interaction.py` TC18) must keep working. Keep `_selected_node_var` as the single-node **inspector anchor** — the inspector still edits one node. Satisfies AC4. |
| T4 | App: selection state + single writer funnel + render union | T1, T2, T3 | high | `tools/path_planner/editor/app.py` | Add `_selected_nodes: list[str]` and `_align_axis: AlignAxis = AlignAxis.AUTO` next to `_selected_node` (`app.py:487`). Implement `_set_selection(names)` per Contracts as the **only** writer: filter against `model.nodes`, dedupe preserving first order, set both fields (`_selected_node = names[-1] or None`), mirror `_panels._selected_node_var`, update the selection display, call `_refresh_view()`. Implement `_compute_align_state()` (builds `set_align_state` args + the `align_preview` from `_selected_nodes` + `_align_axis` via `align.build_preview`/`max_residual_cm`/`resolve_axis`) and **call it from `_refresh_view()`** so undo/redo and deletes keep the readout + overlay fresh. Rewrite all four existing `_selected_node` writers (`app.py:1047`, `2004`, `856`, `2675`) to funnel through `_set_selection`. Union `_selected_nodes` into the single `render` call site (`app.py:911-916`) so selection is finally drawn, and pass the computed `align_preview`. Add `_marquee_anchor` / `_marquee_cursor` and register both in `_clear_transient_state` (`app.py:1782-1806`). Update the widened `on_click`/`on_release` handler signatures from T2. Satisfies AC3, AC14; verified by TC32. Blocked by T1 (imports `align`), T2 (canvas API), T3 (`set_align_state`). |
| T5 | App: marquee + modifier handlers, axis-change, make-straight dispatch, multi-nudge | T1, T3, T4 | high | `tools/path_planner/editor/app.py`, `tools/path_planner/tests/test_editor_select_mode.py` | (a) `Mode.SELECT` press on empty canvas → `begin_marquee_drag` + anchor; motion → `update_marquee_preview`; release → `end_marquee_drag` + `find_nodes_in_rect` → `_set_selection`. Press **on a node** keeps today's latch behaviour. Update the existing `Mode.SELECT` click test that calls `_on_canvas_click(...)` with the old signature (`test_editor_select_mode.py:280-282`) to the widened `(x, y, button, key)` form. (b) Modifier clicks: shift = add, ctrl = toggle, plain = replace (per AC2), reading the new `key` arg. Modifier-click on **empty** canvas = no-op (does not clear); a plain empty click clears. (c) `_handle_axis_change(axis_value)`: set `_align_axis`, call `_compute_align_state()` (refresh readout + preview, no dispatch). (d) `_handle_align_request(axis_value)`: build `points` from `_selected_nodes`, call `align_points`, drop nodes whose target equals current, **no-op → status `Already straight. Nothing moved.` + no dispatch**, out-of-bounds under `FIT` → refuse + no dispatch, else `_dispatch(MacroCommand([MoveNode(n, xy) …]))`. Mirror the `len(steps) > 1` degenerate-collapse idiom at `app.py:2415-2420`. (e) Rewrite `_nudge_selected` (`app.py:834-873`) for the rigid multi-nudge: compute the desired `(dx, dy)`, clamp the **delta once** against the selection's bounding box (`dx_eff = clamp(dx, -min(x_i), width - max(x_i))`, same for y), zero delta → no dispatch, 1 node → single `MoveNode` (back-compat, TC30), ≥2 → one `MacroCommand`. Use the exact status strings from Contracts → Naming. Satisfies AC2, AC5, AC6, AC7, AC9, AC10. |
| T6 | Tests: alignment geometry | T1 | low | `tools/path_planner/tests/test_editor_align.py` (new) | TC1–TC10 + TC12. Pure — no Tk, no matplotlib import dance needed (`align.py` imports neither). Use the real `field_data.json` column coordinates as literals plus one diagonal fixture for `FIT`. |
| T7 | Tests: selection, handlers, make-straight, multi-nudge | T5 | low | `tools/path_planner/tests/test_editor_multiselect.py` (new) | TC11, TC13–TC32, TC37. Style A: `EditorApp.__new__(EditorApp)` + injected `SimpleNamespace` collaborators; copy the `_FakeVar` helper and the matplotlib Agg import dance from `test_editor_select_mode.py:31-70` verbatim. TC25/TC26 use a **real** `History` + `FieldModel` (no mock) to prove undo atomicity. TC28 is the rigid-clamp/no-shear regression guard — do not omit it. TC37 records `set_align_state` args across dispatch+undo. |
| T8 | Tests: marquee blit + ALIGN panel smoke + preview render | T2, T3 | low | `tools/path_planner/tests/test_editor_marquee_blit.py` (new) | TC33–TC36. Style B: take the **shared** session `tk_root` from `tests/conftest.py` — never define a new root (a second session fixture = a second `tk.Tk()` = intermittent `TclError` and green-by-silent-skip; see `conftest.py:1-15`). Build `FieldCanvas(tk_root)` / `SidePanels(tk_root)` in function-scoped fixtures, mirroring `test_editor_canvas_blit.py:78-87` and `test_editor_panels_smoke.py:137-148`. TC36 asserts the guide/ghost artists appear and clear. |

**Parallelism:** T1, T2, T3 start together (three different files, no shared seam beyond the pinned contracts). T4 waits on **all three** — it imports `align` (T1), calls the new canvas API (T2), and calls `set_align_state` (T3). T5 waits on T1+T3+T4 and is **strictly serialized behind T4** because both own `app.py`. T2 and T5 each also edit one existing test file (`test_editor_canvas_blit.py`, `test_editor_select_mode.py`) — no other task touches those, so no conflict. T6 unblocks with T1; T8 with T2+T3; T7 last.

## Notes for Implementer

- **`commands.py` needs no change.** `MacroCommand([MoveNode(n, xy), …])` already gives one atomic, single-undo op with prefix rollback. The issue asked for "a Make straight command in `editor/commands.py`"; that is satisfied by composition. Do not add an `AlignNodes` command — computing geometry inside `apply` would couple the geometry to the model and complicate `revert`, against the established "compute in the app, dispatch `MoveNode`" pattern (`_finish_move`, `_nudge_selected`, the inspector coord entry all do exactly this).
- **Two of the issue's Scope bullets are already satisfied by the current architecture** — see Decisions. Do not go looking for `ProjectionDelta`, `_proj_delta`, `_pending_renames`, or `ast_rewrite.py`; all four are retired and every remaining grep hit is a docstring saying so. There is no coordinate→EDGES cascade to maintain.
- **Inherited, not introduced:** a hand-authored `EDGES` entry carrying an explicit `distance_cm` / `heading_deg` goes stale after any node move. Drag, nudge, and the inspector coord entry all already do this; make-straight inherits the behaviour. Out of scope here — do not "fix" it in this PR.
- **`_dispatch` only catches `ValueError`, `KeyError`, `IndexError`** (`app.py:976-996`). Anything else escapes to Tk and kills the handler silently. Raise only those three from new code.
- **`panels.refresh` must never fire callbacks** — it is wrapped in `_suppress_callbacks = True`. Switching the listbox to `EXTENDED` (T3) is the highest-risk edit in this plan: the restore path at `panels.py:2431` and the R11 `_edges_signature` dirty-check both have regression history. Run `test_editor_override_interaction.py` and `test_editor_panels_smoke.py` after touching it.
- **Register every new transient in `_clear_transient_state`** (`app.py:1782-1806`) or marquee state bleeds across mode switches.
- **The matplotlib import dance is mandatory** in any test module that imports `editor.app` / `editor.canvas`: pin `Agg`, no-op `matplotlib.use` around the import, restore. `editor/canvas.py` calls `matplotlib.use("TkAgg")` at import time and raises headless. `align.py` imports neither, so `test_editor_align.py` needs none of this.
- **Never define a second `tk_root`.** One shared session fixture lives in `tests/conftest.py`. A per-module root causes intermittent `TclError` and silent skips that read as green.
- **Run the FULL suite, not just the new files:** `uv run --extra dev pytest tools/path_planner/tests/` — bare `uv run pytest` resolves to system Python and dies on `ModuleNotFoundError` (pytest/libcst are in the `dev` extra).
- **No `routes.h`/`routes.cpp` regeneration.** This PR does not touch `field_data.json`, so the CI freshness gate is not in play. If you find yourself running `build_routes.py`, you have left the scope.
- **Rounding is applied once, inside `align_points`** — never round again at the call site, or `X`/`Y` alignment can drift off the shared scalar.
- **Rollback:** every task is additive except T3's `SINGLE`→`EXTENDED` flip and T4's rewrite of the four `_selected_node` writers. If T3 destabilizes `refresh`, revert the listbox to `tk.SINGLE` and keep the ALIGN section — multi-select still works via marquee and modifier clicks, and AC4 still holds.
- **Branch:** `feat/161-editor-multiselect-straight`, worktree `/Users/diego/Documents/WRO2026/wt-161-editor-multiselect`, based on `origin/main`. Commit style per `CONTRIBUTING.md`: `feat(editor): …`, lowercase imperative, no AI-attribution trailer. The PR body needs the line `Doxygen reviewed for affected APIs before implementation.` (read the target modules' docstrings first — the repo's documentation-first rule).
