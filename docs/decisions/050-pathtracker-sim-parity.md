# ADR-050: PathTracker sim↔firmware FSM parity is a live trace-diff, CI-enforced

**Status:** Implemented
**Date decided:** 2026-06 (Phase 3 / testing, issue #108)

## Rule

The Python `PathTrackerSim` must make the same per-hop FSM decisions — and reach
the same pose — as the real C++ `PathTracker` across the whole dispatch surface,
proven mechanically in CI. A dedicated `parity` job drives the **real** C++ FSM
over the `routes_fixture` (exhaustive all-pairs `goTo` + curated specials),
emits a normalized semantic per-hop trace, and diffs it **live** against the
sim's stream over the **same single-sourced** fixture. There is **no committed
C++ trace golden** — a golden could launder a real regression through a routine
"golden update," the exact silent drift #108 exists to catch.

Load-bearing pieces:

- **Single-sourced + gated fixture (the #91 reversal).** `routes_fixture` is
  single-sourced from an importable `field_fixture` module through the real emit
  pipeline and promoted from frozen to CI-gated (`git diff --exit-code` after
  regen). This **re-couples** the fixture to `Config.h` track width — a
  track-width or field edit now restales the committed fixture and fails the
  gate. Accepted as desirable; see *Supersedes* below.
- **`EdgeDispatchPolicy.h` = terminal one-hop classifier.** Header-only,
  composes the existing `resolveStopAtLine` / `resolvePivotTrim` /
  `resolveSensorOvershootTrim` policies over already-resolved inputs and owns the
  post-clamp trim arithmetic. It introduces **no** cross-hop state: pass-through
  lookahead, the pivot 3-latch carry, return-to-node, and the reset interlude
  stay FSM-side and are covered by the trace, not the resolver. Mirrored in
  Python (`resolve_edge_dispatch`); guarded by a standalone CSV truth-table.
- **Trace schema = a 3-place contract** (C++ emitter / Python loader / Python sim
  normalizer). An unknown record `kind`/field is a loud failure, never a silent
  skip.
- **Three comparison buckets, every field assigned.** EXACT (enums, bools, names,
  reanchor kind, ordering), TOLERANT-NUMERIC (distance at `1e-4`, angular circular
  at `1e-4`), POSE-TOLERANT (`x,y` at `1e-3`, heading circular). An unbucketed
  field fails.
- **Pose = cumulative cross-check, not an oracle.** One shared `VirtualChassis`
  integrates both sides; its non-redundant signal is sub-tolerance drift
  accumulation + heading lever-arm amplification — not reanchor or coordinate
  bugs (owned by the decision compare and the fixture validator).
- **Self-auditing allowlist.** A committed data file scoped to
  `(case_selector, event_kind, field_path)` + reason + tracking issue; a stale
  (unmatched) entry fails CI; never allowlist a whole case.
- **Read-only parity accessors.** `PathTracker::state()` +
  `resolvedBodyHeadingDeg()` (the `State` enum promoted to public) are the only
  introspection surface; the generic driver dispatches off observed FSM state and
  never re-derives a decision.

## Why

The sim already mirrored `resolveStopAtLine` (#63/#115), but nothing proved the
*rest* of the FSM agreed — so the sim could silently diverge from the robot on
every other dispatch branch. A live trace-diff against the real symbol (the
ADR-030 discipline) is the only check that fails the moment production drifts; a
committed golden would not. The classifier is kept terminal because widening it
would either duplicate the existing pure policies or smuggle FSM state into a
"pure" function.

## How to apply

- **Adding a dispatch branch:** extend `EdgeDispatchPolicy.h` and its Python
  mirror together (the CSV truth-table is the cross-language drift guard), add a
  trace record kind to all three schema places, and assign every new field a
  comparison bucket.
- **A real, intentional divergence:** add a scoped allowlist entry with a reason
  and tracking issue — never mute a whole case or trace.
- **Editing the fixture field or `Config.h` track width:** regenerate
  `routes_fixture` and commit it; the freshness gate will otherwise fail.
- **Structural exclusions (not allowlisted):** physical/timing fidelity and the
  fault FSM are excluded from the trace schema by construction (ADR-033
  constant-velocity sim), not muted entry-by-entry.

## Trigger to revisit

- Cross-hop coverage needs the terminal classifier widened (it would then start
  carrying FSM state — re-open the terminal-classifier decision instead).
- The sim must model timing or the fault FSM — the structural exclusion (and
  ADR-033's constant-velocity assumption) would have to be lifted.
- The `routes_fixture`↔`Config.h` coupling proves too noisy in practice (a
  track-width edit churns the fixture more than it is worth) — reconsider the #91
  reversal and a separate parity fixture.

## Supersedes

- **Supersedes the issue-#91 freeze/decouple convention.** #91 froze
  `routes_fixture` and decoupled it from `field_data.json` / `Config.h` so
  unrelated edits could not silently break it; there was **no numbered ADR** for
  that decision — it lived in the `regenerate_routes_fixture.py` docstring + the
  issue. This ADR records the deliberate re-coupling: the fixture is now
  single-sourced and CI-gated, so it always reflects real track geometry, and a
  `Config.h` track-width edit *should* restale it. The first regen is a real
  one-time diff (the field move changes the SHA stamp), not byte-stable.

## See also

- [ADR-030 — Host harness links the real production symbols, CI-enforced](./030-harness-links-real-sources-ci-enforced.md) — the parity dump links real `PathTracker`/`Drive`/`Localization`/`YawSource` the #30 way.
- [ADR-033 — CTPS ingests a derived mission trace; no Python re-impl](./033-ctps-derived-mission-trace.md) — the constant-velocity sim whose fault-FSM + timing exclusion this trace inherits; the `drive.method` 3-place-contract precedent.
- [ADR-027 — `Drive.stopAtLine()` unified resolver](./027-drive-stop-at-line.md) — `resolveStopAtLine`, one of the policies the classifier composes.
- [ADR-036 — Per-turn pivot compensation](./036-per-turn-pivot-compensation.md) and [ADR-044 — STOP_AT_LINE-incoming pivot trim](./044-stop-at-line-incoming-pivot-trim.md) — the pivot trims the classifier composes.
- [ADR-046 — Projection is a node property](./046-projection-as-node-property.md) — the `(edge_type, dest_node_type)` per-hop primitive the classifier keys on.
- `docs/plans/2026-06-28-pathtracker-sim-parity.md` — the originating plan.
- Issues #108 (this parity check), #91 (the freeze/decouple convention reversed here).
