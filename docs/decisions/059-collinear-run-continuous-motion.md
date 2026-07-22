# ADR-059: Collinear line and projection runs keep continuous motion

**Status:** Implemented; BLIND subdecision superseded by ADR-080
**Date decided:** 2026-07 (phase)

## Rule

LINE and PROJECTION pass-through runs keep one continuous velocity profile: one ramp-up at
the run start, cruise through intermediate nodes, and one ramp-down/brake at the terminal
stop. `LineFollower` and `Drive` hold no cross-hop run state. `PROJECTION_FOLLOW` uses
`.setTShapeRampDown(!projPassThrough)` so a line-projection-line continuation does not shed
to the floor at the projection node.

ADR-080 supersedes this ADR's former BLIND per-hop velocity chain. A compatible BLIND run
now coalesces into one braked distance command; `ChainSpeedPolicy`, its Python mirror,
goldens, measured boundary-carry subtraction, and final-hop stub handling are retired.

The CTPS run grouper still renders a continuous line/projection run as one velocity profile.
Zero-motion records such as heading resets, declarations, manipulator actions, and waits are
transparent to that render grouping.

## Why

LINE pass-through already lets the live follower carry cruise through a collinear junction,
but the original CTPS playback rendered a false stop at every intermediate node. Projection
pass-through also disabled the brake without disabling T-shape ramp-down, causing a real
speed dip. Treating the full line/projection continuation as one render profile and suppressing
the internal projection ramp-down makes both firmware and playback match the intended motion.

The original BLIND design tried to preserve per-hop dispatch while planning velocity across
the boundaries. Hardware later showed that its short final hop lacked enough runway to shed
cruise and that boundary carry accumulated. ADR-080 adopted one combined move, removing the
need for that policy while preserving this ADR's line/projection behavior.

## How to apply

- Assemble pass-through runs only in `PathTracker`; split them at pivots, seeks, resets,
  goals, direction changes, or non-collinear bends.
- For `PROJECTION_FOLLOW`, keep ramp-down disabled only while the destination projection is
  pass-through eligible. The terminal hop retains the ordinary floor-to-brake stop.
- In CTPS, group line/projection geometry between braked stops and consume recorded motion
  data. Do not infer a BLIND coalescing decision in the renderer; ADR-080's strict trace is
  authoritative for blind motion.
- Keep `NAV_TURN_TOLERANCE_DEG` as the shared geometry threshold. Do not introduce a second
  pass-through tolerance without a new field or hardware requirement.

## Trigger to revisit

Revisit if a mission needs an explicit stop at an otherwise-collinear pass-through node, or
if projection hardware requires a distinct speed profile rather than the line-follow cruise.

## See also

- [ADR-080 — Blind-run coalescing](./080-blind-run-coalescing.md) — supersedes the BLIND
  per-hop velocity-chain subdecision.
- [ADR-023 — Pass-through collinear intersections](./023-pass-through-collinear-intersections.md)
- [ADR-046 — Projection is a node property](./046-projection-as-node-property.md)
- [ADR-058 — CTPS renders the firmware motion profile](./058-ctps-firmware-motion-profile.md)
- `docs/plans/2026-07-10-collinear-run-continuous-motion.md`
