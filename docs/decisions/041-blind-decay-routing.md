# ADR-041: Blind-decay routing — convex multiplicative penalty on blind-edge distance to nearest anchor

**Status:** Implemented
**Date decided:** 2026-06 (PR3)

## Rule

Blind edges in the offline planner carry a **static, multiplicative, convex** decay penalty keyed
on how deeply the edge's tail sits in anchor-free territory:

    weight(blind u -> v) = distance_cm * (1 + k * (d(u) / d_ref) ** exponent)

Line and projection edges are unchanged: `weight = distance_cm` (factor `1`). This **replaces**
the old flat `COST_BLIND_MULTIPLIER`; at `k = 0` (or `enabled = false`) it reduces to plain
`distance_cm` — a strict generalisation of the prior model.

`d(u)` is the **blind-subgraph distance from tail node `u` to its nearest anchor**, where an
anchor is any node whose type is **`"line"` OR `"reset"`** (not reset-only). The d-map is a
static node property computed once at table-build time via a multi-source Dijkstra over the
blind subgraph seeded from every line and reset node. One flat `NEXT_HOP[from][to]` table is
emitted; no firmware change beyond regenerated values.

## Why

**Line anchors, not just reset anchors.** A line intersection re-anchors localization via
`onIntersection` (ADR-001/005), clearing accumulated blind drift exactly as a reset node does.
Measuring anchor-distance to reset nodes only would misprice routes on the line backbone —
treating them as "reset-far" — and a worked example showed the planner preferring an all-blind
shortcut over a longer but self-correcting line route. With line nodes counted as anchors the
planner correctly prefers the line backbone; reset nodes then matter specifically in line-free
blind pockets where no line anchor is reachable over blind edges.

**Multiplicative, not additive.** The prior model is already per-cm multiplicative. An additive
`length + k*d²` term would swamp short edges, vanish on long ones, and make `k` non-portable
across maps. The multiplicative form keeps `k` dimensionless and map-portable.

**Static and path-independent by design.** `d` is a node property, not path history. The planner
runs standard Dijkstra over a single weighted copy of the graph; no augmented state, no runtime
counter, no new firmware dimension. Routing toward anchors is an **emergent effect** — anchors
create a low-`d` basin — not a semantic "reset the accumulator on arrival."

## How to apply

**Knobs** (`field_data.json` top-level scalars, per ADR-022; never `Config.h`; the firmware
does not compute decay):

| Attribute             | Default | Constraint       |
|-----------------------|---------|------------------|
| `BLIND_DECAY_ENABLED` | `true`  | boolean          |
| `BLIND_DECAY_K`       | `0.05`  | `k >= 0`         |
| `BLIND_DECAY_EXPONENT`| `2.0`   | `exponent >= 1`  |
| `BLIND_DECAY_D_REF`   | `50.0`  | `d_ref > 0` (cm) |

Default `k`, `exponent`, and `d_ref` are placeholder values to be bench-tuned on hardware.

**Honest tail-approximation caveat.** The penalty is keyed on the edge **tail** (a static
potential field). This means it is directionally naive: traversing toward an anchor can cost more
than traversing away on the same edge, and the first blind hop off an anchor receives zero penalty
(`d(u) = 0`). Both are accepted: the model's job is to discourage deep blind **chains**, not single
bounded hops, and line/projection edges (the cheap, self-correcting backbone) dominate routing on
any well-designed field anyway.

## Trigger to revisit

Real-field runs show the directional naivety of the tail-approximation actually causing
mis-routing — the planner routing through a deep-blind path when a line route of comparable
total length exists. At that point, graduate to the deferred augmented-state
`(node, blind_bucket)` path-dependent model with a runtime `blind_accumulated_cm` counter and a
3-D `NEXT_HOP[from][blind_bucket][to]` table (PHASE2_DEFERRED §8.5 "exact" form).

## See also

- [ADR-001](./001-no-wall-follow-no-dead-reckon.md) — line intersections re-anchor localization; the ADR honesty rule that motivates counting line nodes as decay anchors
- [ADR-005](./005-localization-single-source-of-truth.md) — `onIntersection` clears accumulated drift; justifies line nodes as anchors
- [ADR-022](./022-field-data-json-loader.md) — decay knobs live in `field_data.json`; `field_data.py` is a loader, not the data source
- [ADR-043](./043-wall-zero-reset-nodes.md) — the `"reset"` node kind that anchors deep blind-only pockets the line graph cannot reach
- [PHASE2_DEFERRED.md §8.5](../PHASE2_DEFERRED.md) — the path-dependent augmented-state upgrade this ADR defers
