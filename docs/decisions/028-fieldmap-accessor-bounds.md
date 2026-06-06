# ADR-028: FieldMap accessors bounds-check every index and return a safe sentinel

**Status:** Implemented
**Date decided:** 2026-06 (PR-2 trace findings, #33 / #31)

## Rule

Every `FieldMap` accessor that indexes the auto-generated `Routes::` tables
bounds-checks its node/edge index against `N_NODES` and returns a safe sentinel
on out-of-range input instead of reading past the fixed-size array:

- `getNextHop(from, goal)` returns `NODE_NONE` (`0xFF`) if `from` or `goal` is `>= N_NODES`.
- `getEdge(from, to)` returns the function-local `noEdge()` sentinel (an `EdgeInfo`
  with `EdgeType::NONE` and all distance/heading/override fields zeroed) if either index is out of range.
- `getEdgeType(from, to)` delegates to `getEdge`, so an out-of-range query reports `EdgeType::NONE`.
- `getNode(id)` returns the `noNode()` sentinel (zeroed coordinates, `NodeType::LINE`, empty-string name) if `id >= N_NODES`.
- `getNodeType(id)` delegates to `getNode`.
- `findNodeByName(name)` null-guards `name` and returns `NODE_NONE` when the name is unknown.

The sentinels are function-local statics, so the const-reference returns stay
valid and the header is ODR-safe.

## Why

`Localization` is the single writer of `_currentNode` (ADR-005), and `FieldMap`
uses that id directly as an unchecked array index (`EDGES[from][to]`,
`NODES[id]`, `NEXT_HOP[from][goal]`). Before this rule the accessors trusted
every caller; an id of `NODE_NONE` (`0xFF`) or any `id >= N_NODES` read out of
bounds on the generated tables (#33). The `Localization` writers themselves
accept any `uint8_t` with no range check (#31), so the safe boundary is the
accessor, not each call site. Guarding once at the indexing layer covers every
consumer, including those writers, at zero cost in a header-only inline.

`getNodeType` is a deliberate exception worth noting: `NodeType` has no dedicated
"none" value, so an out-of-range id reports `NodeType::LINE`, indistinguishable
from a real LINE node. Callers that must tell the two apart range-check the id
first or inspect `getNode().name`.

## How to apply

- Read the graph only through `FieldMap` accessors; never index `Routes::NODES`,
  `Routes::EDGES`, or `Routes::NEXT_HOP` directly from a consumer.
- Treat `NODE_NONE`, an `EdgeType::NONE` edge, or an empty-name node as "absent"
  and branch on it rather than dereferencing blindly.
- New accessors over `Routes::` data follow the same pattern: check against
  `N_NODES`, return a function-local static sentinel on miss.

## Trigger to revisit

- A consumer needs to distinguish an out-of-range node from a real LINE node by
  type alone — that requires a dedicated `NodeType::NONE` and changes the enum.
- The graph grows past 254 nodes, at which point `NODE_NONE` (`0xFF`) can no
  longer be both the unreachable sentinel and out of the valid id range.

## See also

- [ADR-005 — `Localization` is the single source of truth for current node + heading](./005-localization-single-source-of-truth.md)
- `WRO2026/FieldMap.h` — the inline accessors and `noEdge()` / `noNode()` sentinels.
- `WRO2026/routes.h` / `routes.cpp` — the auto-generated `Routes::` tables the accessors guard.
- `docs/PR35_TRACE_FINDINGS.md` — findings #33 (unguarded accessors) and #31 (unchecked Localization writers).
