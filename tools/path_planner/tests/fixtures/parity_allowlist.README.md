# PathTracker sim↔firmware parity — divergence allowlist

`parity_allowlist.json` is the committed safety valve for the PathTracker
sim↔firmware FSM parity check (issue #108). It is loaded by
`tools/path_planner/sim/parity.py::load_allowlist` and consumed by the T8 parity
test (`tools/path_planner/tests/test_pathtracker_parity.py`).

The whole point of the epic is that the Python `PathTrackerSim` and the real C++
`PathTracker` firmware **agree**. The allowlist is therefore for **known,
unavoidable, tightly-scoped divergences only** — never a whole case, never a
whole trace, never a `*` / `*` / `*` triple wildcard.

## File shape (why there are no comments)

`load_allowlist` requires the top level to be a **JSON list** of entry objects,
and each entry object must carry **exactly** these seven keys (no more, no less):

```
id, case_selector, event_kind, field_path, side, reason, tracking_issue
```

JSON has no comment syntax, the root is a bare list (so there is no place for a
top-level documentation key), and a stray key on any entry is a hard load
failure. A documentation-only "sentinel" entry is **not allowed** either: it
would match zero mismatches and the T8 stale-entry guard fails CI on any entry
that matches nothing. That is why this prose lives in this sibling README, not in
the data file.

## Discipline (what keeps it honest)

- **Stale = fail.** An entry that matches zero mismatches across a whole run
  fails CI (`Allowlist.stale_entries`). Seed only divergences you are confident
  occur.
- **Scope tightly.** Each entry pins a specific `(case_selector, event_kind,
  field_path)` triple so it cannot accidentally mute anything else. A mismatch
  that matches more than one entry is an ambiguous (over-broad) allowlist and
  also fails.
- **Real reason + tracking issue** on every entry.

## Seeded entries

### `return-move-offset-encoder-quantization`

- **Scope:** `case_selector="*__off_*"` (the offset-tagged curated cases only),
  `event_kind="return_move"`, `field_path="dist_cm"`, `side="cpp"`.
- **Why:** The return-to-node prologue fires only on a non-trivial accumulated
  operator offset, which the curated `__off_*` cases pre-load identically on both
  sides. The firmware manufactures that offset through the **encoder
  integrator** — fixed 2 cm incremental deposits, each leg round-tripped through
  cm→encoder-degrees→cm — so the commanded return distance is **quantized** and
  can differ from the sim by more than `DIST_TOL`. The sim applies the offset
  analytically via `operator_move`, so its `dist_cm` is exact. Only the C++
  commanded `dist_cm` of the `return_move` record diverges; the pose still
  re-anchors onto the same node.
- **Status / caveat:** This is the one concrete a-priori divergence the design
  predicts, but its exact magnitude depends on the encoder round-trip. T8 is
  where it is validated against the **live** diff. If the round-trip turns out
  exact (no mismatch beyond `DIST_TOL`), this entry is **stale** and must be
  removed — the stale-entry guard will force that.

## Deliberately NOT allowlisted (structural exclusions — no entry needed)

These are **not in the trace schema at all**, so they can never produce a
mismatch and must **not** have an entry (an entry would be stale and fail CI):

- **Fault FSM (ADR-033).** The parity driver drives only the nominal SUCCESS
  path, so no fault records are ever emitted. The host-unreachable
  `LINE_NOT_FOUND` / `DRIVE_FAULT` branches (memory
  `gotcha-host-drive-only-line-not-found-fault`) never reach the trace. No
  fault dimension exists on the wire → no entry.
- **Physical / timing fidelity (constant-velocity sim).** Timing is structurally
  excluded from the trace schema (the records carry decisions + commanded
  params, not durations), so it is excluded by construction, not allowlisted. No
  entry.
