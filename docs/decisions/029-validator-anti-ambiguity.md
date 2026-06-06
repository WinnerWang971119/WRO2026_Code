# ADR-029: The validator is the single anti-ambiguity and input-finiteness gate

**Status:** Implemented
**Date decided:** 2026-06 (PR-2 trace findings, #36 / #38 / #46 / #47 / #49)

## Rule

`tools/path_planner/validator.py` is the single gate that rejects ambiguous or
unsafe field data before the graph builder, planner, and emitter see it. It
rejects:

- **(a)** a pivot-tagged edge whose destination does not have exactly one forward
  out-edge — this check moved out of `planner.py` so it fails as a clean
  `FieldDataError` at validation time, not as a raw `ValueError` traceback (#36).
- **(b)** an `EDGES` pair that conflicts with an existing `LINE_SEGMENTS` or
  `BLIND_EDGES` pair, closing the bypass where an `EDGES` entry could silently
  re-type a line/blind segment (#38 / #47).
- **(c)** any non-finite number (`NaN` / `Inf`) in any numeric field, via a
  finiteness check in `_is_number` so every numeric gate is covered in one place (#46).
- **(d)** node names carrying quotes, backslashes, or non-printable characters
  that would produce a malformed C++ string literal in the generated `routes.cpp` (#49).

Every downstream stage may therefore assume validated, unambiguous, finite, safe
input and skip re-checking it.

## Why

The pipeline is a one-way fan-out: validator → graph_builder → planner →
compiler → header_writer. When a correctness check lived in a *later* stage, a
fully "valid" `field_data` could pass `validate()` and then die deep in the
pipeline — the pivot out-degree check raised an uncaught `ValueError` from the
planner with no controlled exit code and no written `routes.h` (#36), `NaN`
slipped past every geometry gate and surfaced as corrupt Dijkstra edge weights
(#46), and an unsafe node name failed at arduino-compile on a do-not-hand-edit
generated file instead of pointing at the source (#49). Anti-ambiguity is the
same story: the validator already rejected a pair appearing in both
`LINE_SEGMENTS` and `BLIND_EDGES`, but `EDGES` bypassed that rule and let
`networkx` silently overwrite the edge type and geometry by emission order
(#38 / #47). Concentrating these checks in the validator makes the failure a
clean `FieldDataError` at the earliest layer, pointing at the field data, and
lets every later stage trust its input.

## How to apply

- Any new "this input is ambiguous, non-finite, or would break a downstream
  literal" check belongs in `validator.py`, not in the stage that happens to trip
  over it. If a later stage can raise on bad data, promote the check upstream.
- Reject with `FieldDataError` so `build_routes.py` reports it on the documented
  data-error exit path rather than as a traceback.
- Downstream stages (`graph_builder`, `planner`, `compiler`, `header_writer`) do
  not re-validate finiteness, name safety, or pair uniqueness; they assume the
  validator already enforced them.

## Trigger to revisit

- A downstream stage gains a new failure mode on otherwise-valid input — add the
  guard to the validator rather than letting that stage raise.
- The field-data schema absorbs string-keyed `EDGES` overrides (the ADR-022
  trigger); the anti-ambiguity cross-check must then understand override-vs-redefine semantics.

## See also

- [ADR-022 — field_data geometry lives in JSON; field_data.py is a loader](./022-field-data-json-loader.md)
- `tools/path_planner/validator.py` — the single validation gate.
- `tools/path_planner/build_routes.py` — orchestrator; surfaces `FieldDataError` as the data-error exit.
- `docs/PR35_TRACE_FINDINGS.md` — findings #36, #38, #46, #47, #49.
