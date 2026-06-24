# ADR-048: Reset nodes are blind-incident only; STOP_AT_LINE + stop_at_line='off' is rejected

**Status:** Implemented
**Date decided:** 2026-06 (PR #98 triage, issue #79 follow-up, Phase 3)

## Rule

Two field-data contract rules the validator (`tools/path_planner/validator.py`) now enforces:

1. **Reset nodes are BLIND-incident only.** A `LINE` (line-follow) edge may not be incident to a
   `"reset"` node, exactly as it may not be incident to a `"blind"` node. A reset node is reached
   by dead reckoning and presses a wall *off* the line backbone, so it carries no line under the
   sensor. A `LINE` edge into a `"projection"` node stays **legal** (ADR-046) — only `blind` and
   `reset` endpoints are off-line and rejected. Enforced for both `LINE_SEGMENTS` pairs and
   `EDGES`-table `LINE` overrides (one helper, `_reject_line_follow_into_blind`, now rejects both
   kinds). Defense-in-depth mirrors the predicate in the editor (`auto_edges` classifies a
   reset-incident candidate `"blind"`; edge-mode `L` refuses a reset-incident pair).

2. **`STOP_AT_LINE` + `stop_at_line='off'` is a contradiction and is rejected.** An `EDGES` entry
   whose `primitive_mode` resolves to `BlindMode.STOP_AT_LINE` (the seek bundle) together with a
   `stop_at_line` of `OFF` (force dead-reckon, no seek) is rejected with a `FieldDataError` naming
   the `EDGES[i]` index and both halves of the contradiction. This is the **only** primitive/stop
   combination rejected; every other pairing (`STOP_AT_LINE` + `ON`, `SYNC_TSHAPE` + `OFF`, …)
   stays legal. Mirrored by the inline editor Save for fast operator feedback.

## Why

Both rules close a *silent* mis-behavior that the per-edge override overlay (ADR-047) newly made
expressible from the GUI:

- A `LINE` edge into a reset node is not just physically impossible — a reset node IS a blind-decay
  anchor (`planner._ANCHOR_NODE_TYPES` is `{line, reset}`). Reaching it over a line-follow seam
  would make it advertise itself as a line-backbone anchor it is not, the same correctness trap
  ADR-041 documents for a `LINE`→`blind` seam. The existing `LINE`→`blind` rejection already
  covered the blind case; the reset case was an unguarded gap surfaced once the editor could author
  reset nodes inline.

- `STOP_AT_LINE` and `stop_at_line='off'` directly oppose: the motion bundle asks for the seek the
  per-edge override forbids. The resolver would silently let the `OFF` override win and dead-reckon
  an edge the author asked to stop on — a quiet behavior change with no diagnostic. Failing it loud
  at author time (the single anti-ambiguity gate, ADR-029) is cheaper than chasing a missed
  re-anchor on the field.

## How to apply

- **Offline (validator):** the rules run inside `validate()` before any emit. The reset-incidence
  check is in `_reject_line_follow_into_blind` (gated on `_RESET_NODE_TYPE` alongside
  `_BLIND_NODE_TYPE`), called from the `LINE_SEGMENTS` loop and the `EDGES` `LINE`-entry branch.
  The contradiction check is in `_validate_edges_field`, after both `primitive_mode` and
  `stop_at_line` are shape-validated, resolving each field's string-tag and enum-instance forms.
- **Composition with reset rules.** This composes with the existing `_validate_reset_nodes` gates
  (a reset node validly sitting on a BLIND backbone still passes): the incidence check only rejects
  a `LINE` edge, never a `BLIND` one. The host-test fixture (`routes_fixture.cpp`, reset nodes
  Wall1/Wall2 reached only by BLIND) and the reset emit fixture both validate unchanged.
- **GUI (defense-in-depth):** the editor mirrors both predicates — `auto_edges._classify_edge_type`
  returns `"blind"` for a reset-or-blind endpoint, the edge-mode `L` guard refuses a reset-incident
  pair, and `panels._handle_save_override` rejects the STOP_AT_LINE/off pairing with a status
  message and no dispatch. The validator stays the authoritative single gate (ADR-029); the GUI
  guards are fast feedback, not a second source of truth.

## Trigger to revisit

- A reset node ever gains a legitimate line-follow approach (e.g. a wall-zero node that also sits
  on a junction) — re-open the incidence rule; today's anchor model assumes a reset node is reached
  blind.
- A future `BlindMode` member is added whose seek semantics are conditional, making the
  `STOP_AT_LINE` + `OFF` contradiction one of several — generalize the per-edge contradiction check
  rather than special-casing each pair.

## See also

- [ADR-047 — JSON-native EDGES overrides + field-sourced PIPELINE_CONFIG](./047-json-edges-overrides-and-pipeline-config.md) — the override overlay these rules guard.
- [ADR-046 — Projection is a node property](./046-projection-as-node-property.md) — a `LINE`→projection edge stays legal; only blind/reset are off-line.
- [ADR-043 — Wall-zero reset nodes](./043-wall-zero-reset-nodes.md) — the reset node kind whose blind-only incidence this rule pins.
- [ADR-041 — Blind-decay routing](./041-blind-decay-routing.md) — the `{line, reset}` anchor set a `LINE`→reset seam would corrupt.
- [ADR-029 — The validator is the single anti-ambiguity gate](./029-validator-anti-ambiguity.md) — both rejections land here, GUI guards are mirrors.
- `docs/plans/2026-06-24-pr98-triage-fixes.md` — originating plan (U2 / U5 / R8).
- PR #98 (closed by this).
