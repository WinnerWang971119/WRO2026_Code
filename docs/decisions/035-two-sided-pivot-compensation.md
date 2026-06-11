# ADR-035: Two-sided pivot compensation with `delta = (T/2)*tan(theta/2)`

**Status:** Superseded by [ADR-036](./036-per-turn-pivot-compensation.md)
**Date decided:** 2026-06 (issue #44, Phase 3)

> **Superseded by [ADR-036](./036-per-turn-pivot-compensation.md).** The per-edge
> `pivot_trim_cm` / `pre_turn_mode_override` model below is replaced by a per-turn
> `(u, v, w)` model with a sparse `PivotTurn[]` table and a single firmware lookup. The
> `delta = (T/2)*tan(theta/2)` magnitude, its derivation, the regime matrix, and the
> dead-reckon / LINE mechanics carry over to ADR-036 unchanged. The body below is kept
> verbatim as history.

## Rule

A single-wheel pivot at node `v` introduces forward over-travel on **both** the incoming
and outgoing adjacent drives. The corrected magnitude is:

```
delta = (T/2) * tan(|theta|/2)
```

where `T` is the chassis track width and `theta` is the interior turn angle. This value
applies equally to both sides; one `pivot_trim_cm` field on the pivot-tagged edge stores it.

`delta` is stored on the **pivot-tagged edge `u→v`**, not on the following edge `v→w`,
and never folded into any shared `EDGES[*][*].distance_cm` cell. The compiler emits
`pivot_trim_cm` (0.0 on non-pivot edges). The firmware and sim carry `delta` one hop and
apply the trim at dispatch:

- **Dead-reckoned incoming (blind / projection):** shorten the approach drive to
  `L_uv - delta` (stop `delta` short of `v`); latch `delta`; shorten the following
  distance-terminated drive to `L_vw - delta`. Axle lands on `w` with zero residual.
- **Line incoming (re-anchors on the junction line at `v`):** reduce the `LINE_OVERSHOOT`
  advance by `delta` (valid while `delta <= LINE_SENSOR_AXLE_OFFSET_CM`); then apply the
  outgoing trim. Equivalent to the dead-reckoned case within the overshoot budget.
- **Sensor-terminated outgoing edges receive no trim.** They self-correct on the physical
  line; the trim is correctly inert there.

The validator fails fast on degenerate pivots (theta approaching 180), `delta >= edge length`
on a dead-reckon side, and `delta > LINE_SENSOR_AXLE_OFFSET_CM` on a line-incoming side.

## Why

Issue #44: the previous mechanism called `apply_pivot_compensations` and mutated
`EDGES[v][w].distance_cm` globally. A non-pivot route through the same `v→w` edge
read the shortened distance and under-drove, an aliasing bug with no fix that did not also
break the pivot case. Storing `delta` on the **pivot edge** and carrying it one hop to the
outgoing drive removes the aliasing at the root: a non-pivot route through `v→w` reads
`pivot_trim_cm = 0.0` and drives the full original distance.

The previous magnitude `T * sin^2(theta/2)` is correct only at theta=90 (gives 0.5T).
The corrected formula `(T/2) * tan(theta/2)` is derived from first principles.

Single-wheel pivot: one wheel is braked and the chassis rotates about that wheel's ground
contact `P`. The axle centre sits on the axle at distance `r = T/2` from `P`, so during the
turn the axle centre traces a circular arc of radius `r` about `P`. Let `h_in` and `h_out`
be the unit incoming and outgoing drive directions; the heading turns by the interior angle
`theta`, so the angle between `h_in` and `h_out` is `theta`.

Stop the approach so the axle centre is `delta` short of the node along the inbound edge, at
`A = v - delta * h_in`, and require the pivot to finish `delta` past the node along the
outbound edge, at `B = v + delta * h_out`. Both points lie on the radius-`r` arc about `P`:
before the turn the axle is perpendicular to `h_in`, so `PA` is perpendicular to `h_in` with
length `r`; after the turn `PB` is perpendicular to `h_out` with length `r`.

The chord is `B - A = delta * (h_in + h_out)`. Two unit vectors `theta` apart sum to a vector
of length `2 * cos(theta/2)`, so `|AB| = 2 * delta * cos(theta/2)`. The same chord subtends
the heading change `theta` at the arc centre `P`, so it also has length `2r * sin(theta/2)`.
Equating:

```
2 * delta * cos(theta/2) = 2r * sin(theta/2)
delta = r * tan(theta/2) = (T/2) * tan(theta/2)
```

Equivalently: the radius-`r` arc is tangent to both edge directions (`PA` perpendicular to
`h_in` at A, `PB` perpendicular to `h_out` at B), so `delta` is the tangent length from `v`
to each tangency point, `r * tan(theta/2)`.

Verification at known angles:
- `theta=90`: `delta = (T/2) * tan(45) = (T/2) * 1.0 = 0.5T`. Matches the old value at 90.
- `theta=60`: `delta = (T/2) * tan(30) ~= 0.289T`. Old value (0.25T) was wrong here.
- `theta=120`: `delta = (T/2) * tan(60) ~= 0.866T`. Old value (0.75T) was wrong here.

## How to apply

**Offline:** `apply_pivot_compensations` in `planner.py` computes `delta` from `Config::TRACK_MM`
(single source, code-genned into the planner; no second track literal). It stores the result
in `graph.edges[u][v]['pivot_trim_cm']` without touching `distance_cm`. `header_writer.py`
emits `pivot_trim_cm` on every `EdgeInfo` cell (0.0 default).

**Firmware (PathTracker):** when the current hop's edge has `pivot_trim_cm = delta > 0`:
trim the approach drive (`driveDist`/`stopAtLine` arm-window) or reduce the `LINE_OVERSHOOT`
advance; latch `delta` in a one-hop carried member; on the next hop's drive, subtract `delta`
from distance-terminated primitives (`driveDist`, `FOLLOW_DIST` gate) and no-op on
`FOLLOW_INT` / `STOP_AT_LINE` outgoing edges. Clear `delta` on abort or after the final hop.

**Sim (sim/runtime.py):** mirrors the firmware dispatch byte-for-byte; `_POSITION_TOL_CM`
and drift == 0 are preserved.

**Regime matrix** (incoming edge type x outgoing edge type):

```
incoming \ outgoing | LINE / STOP_AT_LINE (sensor)     | BLIND-plain / PROJECTION (distance)
--------------------+----------------------------------+-------------------------------------
LINE (re-anchor)    | overshoot-reduce in; outgoing    | overshoot-reduce in; outgoing drive
                    | trim no-ops (self-corrects @w)   | trimmed by delta
BLIND / PROJECTION  | approach trim; outgoing no-ops   | approach trim + outgoing drive trim
                    | (self-corrects @w)               | (exact, zero residual)
```

## Trigger to revisit

- Hardware bench (B3.x-pivot) shows residual positional error larger than the measured
  tolerance even after `delta` is applied, suggesting wheel scrub on the braked contact
  invalidates the rigid-body model; at that point the emitted `delta` needs a
  correction factor derived from the bench data.
- A future chained-pivot sequence (two consecutive pivots with no intervening dead-reckon
  segment) is needed; the current "carry one hop" model would need to stack two `delta`
  values.
- `LINE_SENSOR_AXLE_OFFSET_CM` is changed on either chassis: the validator's overshoot-budget
  check must immediately re-run against any line-incoming pivot in `field_data`.

## See also

- [ADR-023 — pass-through collinear intersections + overshoot-mode probe](./023-pass-through-collinear-intersections.md) — the `LINE_OVERSHOOT` advance that ADR-035 reduces.
- [ADR-027 — `Drive.stopAtLine()` re-anchors localization; arm-window rule](./027-drive-stop-at-line.md) — the arm-window that the approach trim adjusts on `STOP_AT_LINE`-typed blind edges.
- [ADR-022 — field_data geometry lives in JSON; field_data.py is a loader](./022-field-data-json-loader.md) — the `EDGES` table that `pivot_trim_cm` does NOT mutate.
- [ADR-029 — validator is the single anti-ambiguity gate](./029-validator-anti-ambiguity.md) — extended here with pivot-regime fail-fasts.
- `docs/plans/2026-06-10-issue44-pivot-two-sided-compensation.md` — the originating plan.
- Issue #44 — the aliasing bug this ADR closes.
