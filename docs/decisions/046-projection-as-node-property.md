# ADR-046: Projection is a node property, not an edge type

**Status:** Implemented
**Date decided:** 2026-06 (issue #64, Phase 3)

## Rule

`projection` is a mutually-exclusive **node** type — `NodeType { LINE, BLIND, RESET, PROJECTION }`
— not an edge type. Edges are **LINE or BLIND only**; the per-hop primitive falls out of
`(edge_type, destination_node_type)`. A projection node sits on a line but not at a junction:
you can line-follow across it, but you can only dead-reckon (by encoder distance) *to* it, so
there is a real line to stop on but no junction marker to identify it.

Per-hop primitive (source node type never matters):

| Edge | → line node | → projection node | → blind node |
|------|-------------|-------------------|--------------|
| **LINE** | line-follow until junction (+ re-anchor) | line-follow + dead-reckon to distance | illegal (validator rejects) |
| **BLIND** | dead-reckon (seek via explicit override) | **auto-seek stop-at-line + full-advance re-anchor** | pure dead-reckon |

- **Auto-seek is a local destination-node read.** `auto_rule_active = (edge.type == BLIND) ∧
  (dest node is PROJECTION) ∧ BLIND_PROJECTION_AUTO_SEEK`, computed by the caller with **no**
  `NEXT_HOP[next][goal]` peek. The route-relative `next_hop_is_projection` lookup (transcribed
  across firmware, compiler, and sim) is gone.
- **The resolver stays frozen.** `resolveStopAtLine` / `resolve_stop_at_line` and the 936-row
  truth-table CSV are route-agnostic and **unchanged** — only the caller's `auto_rule_active`
  input is now computed differently. If the CSV golden regenerates, the resolver was touched.
- **BLIND→projection** = auto-seek + full-advance re-anchor (current node = projection node,
  operator offset cleared). **LINE→projection** = distance-terminated follow + a dead-reckon
  `onDeadReckonedAdvance`; its motion profile comes from the LINE edge's `line_mode_override`.
- **A projection node is NOT a blind-decay anchor** (conservative): `planner._ANCHOR_NODE_TYPES`
  stays `{line, reset}`. Routing cost follows the incident edge type — LINE→proj is factor-1,
  BLIND→proj is decay-priced. **Production routing is unchanged this phase** (the real field has
  zero projection nodes; only the toy field gains one); making the cost change live is a P3.5
  hand-off when the real field is re-encoded.
- **`line–proj–line` pass-through:** a non-goal projection node flanked by collinear LINE edges
  (ADR-023 heading check) collapses to one continuous A→C follow, fired off the **distance gate**
  — `Intersection.isAtIntersection()` never triggers at a projection node — with a mid-follow
  dead-reckon advance at B and a re-anchor at C's junction. A BLIND-flanked projection node is a
  normal stop.
- **RETURN_TO_NODE to a projection node** is a dead-reckon return (no junction to seek);
  `_returnUsesLineSeek` / `is_line_backed` already returns false for it.

The seek override surface (call-flag, per-edge `stop_at_line_override`, legacy
`BlindMode::STOP_AT_LINE`, the field-wide `BLIND_PROJECTION_AUTO_SEEK` toggle) survives unchanged;
only the auto-rule **trigger** is retargeted. `ProjectionMode`, `projection_mode_override`,
`PipelineConfig.projection`, `EdgeType::PROJECTION`, and `next_hop_is_projection` are deleted tree-wide.

## Why

"Projection is an edge type" was never a numbered decision — it was an **implicit convention**
introduced piecemeal across the pipeline. It forced a goal-relative `NEXT_HOP[next][goal]` peek to
decide a behavior that is really a property of *where you are arriving*, and that peek had to be
re-transcribed in firmware, compiler, and sim (three drift surfaces). Modeling projection as a node
type — mirroring the on-model `RESET` precedent — makes every per-hop behavior a local lookup of
`(edge_type, destination_node_type)`, deletes the route-relative peek, and lets the editor drop its
off-model `ProjectionDelta` store in favor of the normal node-type combobox. Mutual exclusivity of
the four node types makes a separate `no_marker` flag redundant.

(The issue #64 body cited "ADR-036/037" as the prior projection decisions; that was wrong — ADR-036
is per-turn pivot compensation and ADR-037 is HUE12 vision. No prior numbered ADR established the
edge model. This ADR documents/retires a convention, not a decision.)

## How to apply

- **Offline:** author a projection node with `NODE_TYPES[x] = "projection"`. The validator rejects
  an unknown node type, a LINE edge into a blind node, and a projection node as the start node;
  `_regime` returns `distance` for LINE→proj, `line` for LINE→line, `sensor` for BLIND→proj.
- **Firmware (`PathTracker`):** `_dispatchEdgeAfterTurn` branches on destination type — LINE→proj
  arms the distance gate + dead-reckon advance, BLIND→proj seeks + full-advance re-anchors. The
  auto-rule reads `FieldMap::getNodeType(_nextHop) == PROJECTION` locally; `_peekNextNextHop` is
  kept for the pass-through lookahead only, guards intact.
- **Pass-through:** the state transition is **distance-gated** at the projection node, not
  intersection-gated. A non-collinear or non-LINE-flanked candidate falls back to a normal stop.
- **Editor:** projection is set through the standard node-type combobox (the `reset`-node UX
  precedent, type-only, no sub-panel); the off-model machinery is gone.

## Trigger to revisit

- **Bench gate (required before any mission relies on a projection-node hop).** The `line–proj–line`
  pass-through and the projection re-anchor are *physical* behaviors — the auto-seek now re-anchors
  at the projection node's own line instead of a preceding junction, a deliberate relocation of where
  re-anchoring happens on the field. Verify on the real chassis, analogous to the B3.8 pivot gate,
  before trusting a projection hop in a scored run.
- **P3.5 real-field encoding.** When the real field gains projection nodes, the cost change (proj
  not an anchor; LINE→proj factor-1, BLIND→proj decay-priced) becomes live — re-validate real-field
  `NEXT_HOP` and choose LINE-vs-BLIND incidence intentionally per projection approach.
- A projection node needs to act as a blind-decay anchor — revisit `_ANCHOR_NODE_TYPES`
  (deliberately *not* changed here).

## Supersedes / amends

- **Supersedes** the ADR-038/ADR-039 **auto-rule sub-decision only** — the
  "next-hop-is-projection-*edge*" trigger becomes "destination-is-projection-*node*." The rest of
  both ADRs (the seek precedence ladder, the per-edge override, the field-sourced toggle, the
  hard-fault-on-miss posture) stays in force.
- **Retires the implicit "projection is an edge type" convention** (never a numbered ADR).
- **Amends [ADR-023](./023-pass-through-collinear-intersections.md):** its pass-through language is
  narrowed to the `line–proj–line` case, now fired off the distance gate rather than the
  intersection branch (a projection node has no junction marker).
- **Amends [ADR-044](./044-stop-at-line-incoming-pivot-trim.md):** the pivot regime is now derived
  from `(edge_type, destination_node_type)`; a pivot approaching a projection node draws its trim δ
  from the LINE→proj follow distance, not the overshoot budget.

## See also

- [ADR-038 — Blind→projection auto stop-at-line](./038-blind-projection-auto-stop-at-line.md) — auto-rule sub-decision (trigger) superseded here; ladder/override/toggle survive.
- [ADR-039 — Auto stop-at-line fires a normal seek; missed line is a hard fault](./039-stop-at-line-normal-seek-no-soft-fallback.md) — same auto-rule sub-decision retargeted; hard-fault-on-miss unchanged.
- [ADR-023 — Pass-through collinear intersections](./023-pass-through-collinear-intersections.md) — amended (line–proj–line, distance-gated).
- [ADR-044 — STOP_AT_LINE-incoming pivot trim](./044-stop-at-line-incoming-pivot-trim.md) — amended (regime from edge + destination).
- [ADR-043 — Wall-zero reset nodes](./043-wall-zero-reset-nodes.md) — the on-model node-type precedent this mirrors.
- [ADR-041 — Blind-decay routing](./041-blind-decay-routing.md) — the anchor-set / cost model a projection node deliberately does NOT join.
- `docs/plans/2026-06-22-issue64-projection-as-node-property.md` — the originating plan.
- Issue #64 (closed by this).
