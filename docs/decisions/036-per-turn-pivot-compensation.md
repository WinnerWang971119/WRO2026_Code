# ADR-036: Per-turn pivot compensation — a pivot is a 3-node turn, not a per-edge field

**Status:** Implemented
**Date decided:** 2026-06 (issue #44, Phase 3)

## Rule

A single-wheel pivot is a property of a **3-node turn** `(u, v, w)`: arrive from `u`,
pivot at `v`, leave to `w`. It is authored as a node-name triple in
`field_data.PIVOT_TURNS` and fires only when the route actually takes that turn, so a
node `v` may carry a pivot on one continuation and a normal turn on another.

The offline pipeline derives both the wheel and the over-travel `δ` per triple, from
geometry alone (authors never type a wheel):

```
θ     = normalize(heading(v→w) − heading(u→v))   into (−180, 180]
δ     = (T/2) · tan(|θ|/2)                         # carried over from ADR-035, unchanged
wheel = PIVOT_LEFT  if θ > 0
        PIVOT_RIGHT  if θ < 0
```

These records emit as a sparse, sorted `PivotTurn[]` table (`N_PIVOT_TURNS` records),
not a dense `TURNS[N][N][N]` array. The firmware resolves the turn per hop via a
one-hop lookahead (`findPivotTurn(cur, nxt, getNextHop(nxt, dest))`) and a single
`_pendingPivot` latch. **One lookup decides both the maneuver wheel and the trim, so
they cannot disagree** — this closes the Blocker-1 disagreement bug. `pivot_trim_cm`
and `pre_turn_mode_override` are removed from `EdgeInfo` entirely.

The runtime trim mechanics (the regime matrix, the LINE-overshoot reduction, the
dead-reckon approach/departure subtraction, the one-hop carry) are carried over from
ADR-035 unchanged; only the *source* of `δ` (per-turn table vs per-edge field) and the
*wheel resolution* (one lookup) change.

## The sign(θ) → wheel mapping, derived

The mapping is **not** self-evident, so it is derived here from the wheel-byte
semantics rather than asserted by convention.

1. **Heading frame** (graph_builder `_heading_deg`): headings are `atan2(dy, dx) % 360`,
   so `[0, 360)` with `0°` = East (+x) and **CCW positive**. `θ` is the inbound→outbound
   heading change in that frame; `θ > 0` is a left (CCW) turn.
2. **Wheel-byte semantics** (PathTracker.cpp:151, the `_turnModeToByte` contract):
   `1 = PIVOT_LEFT — left wheel braked, right drives; chassis pivots around left wheel.`
3. **Physical motion:** the right (driven) wheel rolls forward while the left wheel is
   braked, so the chassis rotates about the left contact point. The driven side swings
   forward and the nose rotates toward the left, i.e. **counter-clockwise**.
4. **Sign:** a CCW nose swing is a positive heading change in the CCW-positive frame, so
   `PIVOT_LEFT ⇒ θ > 0`. Combined with the forward-travel assumption (a pivot is
   FRONT-only / forward traversal, so the nose direction is the heading direction),
   therefore `θ > 0 ⇒ PIVOT_LEFT` and, by mirror symmetry, `θ < 0 ⇒ PIVOT_RIGHT`.

The chain is code-citation → physical motion → sign, auditable end to end.

## Min-angle / IMU-heading assumption

The pre-turn maneuver dispatches only when `|bodyTarget − currentWorld| >
NAV_TURN_TOLERANCE_DEG` (PathTracker.cpp:951). The validator rejects any authored pivot
whose geometric `|θ|` is below `Config::PIVOT_MIN_TURN_DEG` (a tunable knob, default
5°, single-sourced into the validator via `parse_pivot_min_turn_deg_from_config`), so a
real pivot clears that gate. This replaced an earlier `≥ 3 × NAV_TURN_TOLERANCE_DEG`
derived floor: the derived form silently scaled with the tolerance and baked in a
fixed margin assumption, whereas an explicit configurable angle is re-tuned directly
when the chassis or tolerance changes. Clearing the gate still assumes post-arrival IMU
heading matches the inbound edge heading within tolerance — the same assumption the
existing yaw-vs-peek divergence guard (PathTracker.cpp:953) manages; if IMU drift
exceeds it, that guard, not the pivot logic, is the mitigation.

## Why

Per-turn dissolves Blocker 1: the trim and the maneuver share one resolved record and
cannot drift apart. It also deletes the old single-forward-out-edge landing requirement
(finding #36) — a turn binds `(u, v, w)`, so `v` is free to have other continuations.
The table is sparse (a handful of records), not dense `N³` — a `TURNS[N][N][N]` array is
~125 KB at `N=50` on a 256 KB-flash board, infeasible. The `(T/2)·tan(θ/2)` magnitude,
its first-principles proof, and the full regime matrix are unchanged from
[ADR-035](./035-two-sided-pivot-compensation.md); see it for the derivation rather than
repeating it here.

## How to apply

- **Authoring:** `pivot_turns` in `field_data.json` (array of 3-element node-name
  arrays); re-exported as `field_data.PIVOT_TURNS`.
- **Offline:** `planner.build_pivot_turns(graph, track_cm)` derives `δ`
  (`compute_pivot_compensation`, reused) and the wheel (`sign(θ)`);
  `header_writer` emits the sorted sparse `PivotTurn[]` (`max(1, count)` elements so a
  pivot-free field still compiles).
- **Firmware latch contract:** at plan time, snapshot `_departingPivot = _pendingPivot`
  **before** the lookahead overwrites `_pendingPivot`; resolve `_approachingPivot` fresh;
  both survive `TURNING` into the drive. Departing trim (outgoing) and approaching trim
  (approach) come from two distinct resolved records, so a chain `u→v→w→x` never loses or
  misroles a `δ` on the shared edge. A pivot-adjacent hop is **forced FRONT** (the wheel
  and `δ` are derived for forward traversal); a silent downgrade to a counter-rotation is
  rejected.
- **FRONT-only in v1** — reverse (REAR-traversal) pivots are deferred to #54.

## Trigger to revisit

- Reverse pivots (REAR traversal) are needed — #54. **Implemented by
  [ADR-073](./073-reverse-pivots.md) (#54).**
- A chained sequence longer than two consecutive pivots is needed (the latch carries one
  hop; longer chains would stack more records).
- Hardware bench B3.8 shows residual scrub error after `δ` is applied — apply a measured
  `Config::PIVOT_SCRUB_FACTOR` correction (the ADR-035 idealization caveat).
- `LINE_SENSOR_AXLE_OFFSET_CM` changes on either chassis — re-run the validator's
  LINE-incoming overshoot-budget check against every line-incoming pivot.

## See also

- [ADR-035 — Two-sided pivot compensation `δ = (T/2)·tan(θ/2)`](./035-two-sided-pivot-compensation.md) (**Superseded by this ADR**; keeps the `δ` proof + regime matrix).
- [ADR-023 — Pass-through collinear intersections + overshoot-mode probe](./023-pass-through-collinear-intersections.md) — the `LINE_OVERSHOOT` advance the approach trim reduces.
- [ADR-027 — `Drive.stopAtLine()` re-anchors localization; arm-window rule](./027-drive-stop-at-line.md) — the arm-window the approach trim adjusts on `STOP_AT_LINE` blind edges.
- [ADR-028 — FieldMap accessors bounds-check every index](./028-fieldmap-accessor-bounds.md) — `findPivotTurn` follows this on out-of-range indices.
- [ADR-029 — The validator is the single anti-ambiguity gate](./029-validator-anti-ambiguity.md) — extended with the per-turn fail-fasts (replacing the #36 out-degree check).
- [ADR-073 — Reverse (REAR-traversal) single-wheel pivots](./073-reverse-pivots.md) — lifts the FRONT-only restriction below via force-SAME + a runtime wheel-flip policy.
- `docs/plans/2026-06-11-per-turn-pivot-compensation.md` — the originating plan.
- Issues #44 (closed by this), #54 (reverse pivots, closed by ADR-073), #52 (editor authoring).
