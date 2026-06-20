# ADR-044: STOP_AT_LINE-incoming pivot trim — a seek into a pivot lands the axle δ-short

**Status:** Implemented
**Date decided:** 2026-06 (issue #65, Phase 3)

## Rule

The pivot regime matrix (ADR-035, carried by ADR-036) gains a third incoming row. A
`STOP_AT_LINE` *seek* that arrives into a pivot node `v` now reduces its post-detect
overshoot by the approaching pivot trim `δ`: the seek's `Drive.stopAtLine` overshoot
drops from `LINE_SENSOR_AXLE_OFFSET_CM` to `offset − δ`, clamped to
`PIVOT_TRIM_MIN_DRIVE_CM`. The axle therefore settles `δ` short of `v` — mirroring the
LINE-incoming overshoot-reduction path — instead of rolling fully onto the node and
landing the pivot off-target (a `LINE_NOT_FOUND` on the outgoing follow).

**D1 — approaching trim ONLY.** The reduction subtracts the *approaching* pivot trim
and never the departing trim. A seek is sensor-terminated: it stops on the line at `v`
regardless of where it started, so a departing pivot at `u` is auto-absorbed and the
post-detect overshoot depends only on the approaching pivot at `v`. Never subtract the
departing trim on the seek path.

The fix keys on `_approachingPivot.active`, so it is **rung-agnostic** across the
ADR-038 seek precedence ladder: (1) call-site `setStopAtLine(ON)`, (2) per-edge
`stop_at_line_override`, (3) legacy `BlindMode::STOP_AT_LINE`, (4) the auto-rule
`BLIND_PROJECTION_AUTO_SEEK && nextHopIsProjection`, (5) never. Every rung reaches the
same seek dispatch, so the trim covers all of them — including the user's
`u→v` BLIND, `v→w` PROJECTION auto-rule case (rung 4).

## Why

Issue #65: the regime matrix had only two incoming rows — LINE and BLIND/PROJECTION —
and **no row for a `STOP_AT_LINE` seek incoming**. A seek re-anchors longitudinally
(axle on the node) like the LINE path, but unlike the LINE path it never reduced its
post-detect overshoot by `δ`. The axle rolled fully onto `v`, the pivot fired from the
wrong origin, and the outgoing follow could miss its line. Routing the seek overshoot
through the same δ-reduction the LINE path already uses closes the gap.

`δ > offset` (pivots sharper than ~106° on our track) cannot be corrected by a
forward-only seek — there is no positive overshoot left to shorten. The validator
**hard-rejects** that case offline; the general fix (a reverse "re-seat" / signed
overshoot, which would also cover the LINE follow) is **deferred to issue #75**.

## How to apply

- **Firmware (`PathTracker.cpp`):** in the BLIND seek dispatch, when
  `_approachingPivot.active`, set the seek overshoot to
  `resolveSensorOvershootTrim(seekAxleOffsetCm, _approachingPivot.trim_cm,
  PIVOT_TRIM_MIN_DRIVE_CM).overshootCm`, where `seekAxleOffsetCm` is direction-resolved
  (FRONT/REAR) like the LINE path — force-FRONT keeps it FRONT for every pivot seek. When
  the floor clamp fires (δ > offset) the dispatch **hard-faults**
  (`Fault::PIVOT_TRIM_UNDERFLOW`) instead of dispatching the seek off-target (D4). The
  inline LINE overshoot reduction calls the same helper, **passing its direction-resolved
  offset** (FRONT or REAR) so the REAR LINE-into-pivot path stays byte-identical; the LINE
  clamp stays a defensive warn-and-clamp (always validator-covered — no call-site escape —
  so it cannot land off-target the way the seek can).
- **Shared helper:** `resolveSensorOvershootTrim` lives in `PivotTrimPolicy.h` (a thin
  wrapper over `resolvePivotTrim(axleOffsetCm, 0.0f, approachingTrimCm, minDriveCm)`,
  departing fixed at 0). Its **Python twin** in `sim/runtime.py` routes both the LINE and
  the seek overshoot reductions through the same formula — cross-language parity is the
  guard that the sim stays a faithful oracle.
- **Validator (`validator.py`):** the δ ≤ offset overshoot-budget check is extended from
  `in_regime == "line"` to `in_regime in ("line", "sensor")`. The `_regime` helper folds
  the auto-rule (`auto_seek_enabled and next_hop_is_projection`) into the `"sensor"`
  class, so one change budgets authored `STOP_AT_LINE`, per-edge override, AND auto-rule
  `BLIND→PROJECTION` pivots. A δ>offset `"sensor"`-incoming pivot fails fast naming the
  escape (soften the turn, approach via a dead-reckon BLIND leg, or issue #75).
- **D4 — boundary.** The validator computes `next_hop_is_projection` offline, so it sees
  authored `STOP_AT_LINE`, per-edge override, and the auto-rule. The ONE case it cannot
  see is a pure call-site `setStopAtLine(ON)` on an otherwise non-sensor edge (no
  projection continuation, no override). For `δ ≤ offset` the firmware fix lands the axle
  δ-short there too (it keys on `_approachingPivot.active`, not the rung). For `δ > offset`
  — where a forward-only seek has no positive overshoot left to shorten — the dispatch
  **hard-faults** (`Fault::PIVOT_TRIM_UNDERFLOW`) rather than clamp to the floor and fire
  the pivot from an off-target origin. That matches ADR-039's "no silent degradation on a
  seek": a missed line hard-faults, and so does a seek that cannot land where the pivot
  needs it. The reverse re-seat that would actually compensate `δ > offset` is issue #75.

## The audit — pivot × stop-mechanism matrix (issue #65 deliverable)

**Incoming approach trim (does the axle land δ-short of `v`?):**

| Incoming `u→v` | Re-anchors at `v`? | δ-short approach trim? | Verdict |
|---|---|---|---|
| LINE follow | yes (junction line) | yes — overshoot reduced by δ | OK |
| BLIND dead-reckon | no | yes — `driveDist` shortened by δ (`_pivotTrimDistance`) | OK |
| PROJECTION | no (`onDeadReckonedAdvance`) | yes — `driveDist` shortened by δ; encoder-gated, trim bites before the drive | OK |
| **STOP_AT_LINE seek** (any rung: legacy edge / per-edge override / auto-rule BLIND→PROJECTION) | yes (line) | **NO — axle rolled fully onto `v`** | **was BUG → fixed (D1–D3)** |

**Adjacent interaction cases:**

| Case | Verdict | Why |
|---|---|---|
| Departing-pivot-into-seek (arm-window) | OK | δ_departing auto-absorbed by sensor termination; the line still appears inside the seek's arm window and under the max margin. Revisit only if the arm window is tightened below the max authored departing δ. |
| Chained pivots across a seek (latch carry) | OK | `_pendingPivot → _departingPivot` is latched at plan-time in `_onPlanNextHop`; no STOP_AT_LINE / seek path clears it. |
| Pure call-site `setStopAtLine(ON)` into pivot | Covered (δ≤offset) / hard fault (δ>offset) | Reachable; the validator can't see the call-site-only override. For δ≤offset the firmware fix lands it δ-short; for δ>offset the dispatch hard-faults `Fault::PIVOT_TRIM_UNDERFLOW` instead of landing off-target (D4). |
| RETURN_TO_NODE (#36) seek onto a pivot node | OK | The return seek precedes pivot resolution and re-acquires the same node; no δ owed. |
| Pivot node that is also a wall-zero reset node (ADR-040) | OK | The WALL_STOP→REANCHOR interlude re-anchors `v` on arrival, erasing any approach error before the pivot fires; the incoming trim is belt-and-braces. |
| REAR / 2-board seek into a pivot | Unreachable | FRONT forced on any pivot-adjacent hop before the seek dispatches; the validator also rejects authored REAR on a pivot leg. |

## Trigger to revisit

- A pivot needs `δ > offset` (sharper than ~106° on our track) reached via a seek or a
  LINE follow — the forward-only overshoot reduction can't compensate it; pick up the
  reverse "re-seat" / signed-overshoot generalization tracked in **issue #75**.
- The seek arm window (`STOP_AT_LINE_ARM_WINDOW_CM`) is tightened below the max authored
  departing δ — re-check the departing-pivot-into-seek auto-absorption case.
- `LINE_SENSOR_AXLE_OFFSET_CM` changes on either chassis — the validator's δ ≤ offset
  budget must re-run against every `"line"`- and `"sensor"`-incoming pivot.
- REAR-traversal pivots (#54) land — the FRONT-only single-offset assumption on the seek
  path no longer holds; re-derive the offset per direction.

## See also

- [ADR-035 — Two-sided pivot compensation `δ = (T/2)·tan(θ/2)`](./035-two-sided-pivot-compensation.md) — the regime matrix this ADR extends originates here (kept for the δ proof).
- [ADR-036 — Per-turn pivot compensation `(u, v, w)`](./036-per-turn-pivot-compensation.md) — carries the regime matrix forward; **extended by this ADR** with the STOP_AT_LINE-incoming row.
- [ADR-038 — Blind→projection auto stop-at-line; precedence ladder](./038-blind-projection-auto-stop-at-line.md) — the seek precedence ladder this fix is rung-agnostic across.
- [ADR-039 — Auto stop-at-line fires a normal seek; a missed line is a hard fault](./039-stop-at-line-normal-seek-no-soft-fallback.md) — the seek the trim adjusts; LINE_NOT_FOUND still hard-faults.
- [ADR-029 — The validator is the single anti-ambiguity gate](./029-validator-anti-ambiguity.md) — extended here with the δ ≤ offset budget on `"sensor"`-incoming pivots.
- `docs/plans/2026-06-19-issue65-pivot-seek-reanchor.md` — the originating plan.
- Issues #65 (closed by this), #75 (reverse re-seat for δ>offset), #54 (REAR pivots).
