# ADR-052: Wall-free heading-only re-anchor at collinear line pass-throughs, gated by two independent knobs plus a strict sanity check

**Status:** Implemented
**Date decided:** 2026-07 (Phase 3, issue #73)

## Rule

`LocalizationClass::reanchorHeading(headingDeg)` re-zeros the robot's IMU-yaw belief to a
known incoming edge's `heading_deg` whenever it line-follows straight through a collinear
pass-through node — no wall, no maneuver, no motion. `PathTracker` fires it at the single
arrival chokepoint (`_onPlanNextHop` → `_maybeHeadingReset`), on a collinear pass-through
only, gated by two independent `Config` knobs — `LINE_HEADING_RESET_ON_PROJECTION` (default
ON) and `LINE_HEADING_RESET_ON_JUNCTION` (default OFF) — plus a strict sanity gate
(`LINE_HEADING_RESET_STRICT_ENABLED` default ON, `_TOLERANCE_DEG` default 10°). It is
instantaneous (one IMU read + offset math); a failed read or a strict-gate rejection
soft-skips — returns `false`, mutates nothing, no fault — and the run continues.

## Why

Long runs accumulate IMU-yaw drift; the wall re-anchor (ADR-042/043) bounds it but needs an
authored reset node and a maneuver. A pass-through node's incoming LINE edge already carries
a known `heading_deg` (ADR-046) for free — binding to it costs one IMU read with no motion
and no field authoring beyond placing a projection node. Projection pass-throughs default ON
because the line-follow PID never freezes there, so the sample is clean; physical-junction
pass-throughs default OFF because the junction PID-freeze window (ADR-020) can inject a small
wiggle into the sample, and that envelope is unmeasured pending bench data. The strict gate
exists only to catch a gross-wrong-state sample (roughly 90–180° off — a kidnapped or
perpendicular read), not to discriminate line-follow wiggle from IMU drift; both live in the
same few-degree band at a 10° tolerance.

`reanchorHeading` is heading-only by design: it must never touch `_offset` or the integrator
baselines, and it rejects on `!_navActive` to keep the closed-integrate-gate invariant intact
(`gotcha-localization-rezero-reopen-edge`) — rebinding the world frame while that gate is open
would deposit the discontinuity into `thetaDeg` on the next update. The heading-bind core,
`_bindHeadingFromYaw`, is shared with `reanchorAtWall` so the offset-bind invariant has one
implementation, but each caller keeps its own IMU read and its own gate: a wall rejection
faults, a line rejection soft-skips, and merging the two would either double-read the IMU or
apply the wrong failure semantics to one of them.

## How to apply

Projection-node placement (an existing authored node property, ADR-046) is the opt-in — no
`field_data`, `routes`, or `ResetNode` change is needed to enable this on a given field.
`tools/path_planner/sim/{runtime,parity}.py` mirrors the fire decision and emits a sim-only
`heading_reset` parity record (three-place contract, modeled on `ACT_REANCHOR`) on the accept
path only; the record's pass-kind field is named `pass_kind` (`"proj"` | `"junction"`), not
`kind` — `kind` is reserved as the record-type discriminator. Observability is a Serial
fire/skip log on the enabled paths only (knob-off stays silent, no forced IMU read); the
parity record fires only on a successful bind, so the driftless sim (which never rejects)
matches the firmware's accept path exactly.

## Trigger to revisit

- Bench data characterizes the junction freeze-window wiggle envelope — flip
  `LINE_HEADING_RESET_ON_JUNCTION` ON by default.
- Broaden to an every-LINE-arrival reset, or replace the discrete bind with a continuous
  complementary-filter heading trim.
- Add a settledness sample-gate before binding.
- Add always-on (feature-off) drift telemetry — today logging only happens when the relevant
  knob is ON, since logging while off would force an IMU read on every pass-through even with
  the feature disabled.

## See also

- [ADR-042 — `Localization` tracks a 2D operator-offset delta from the last anchored node](./042-operator-offset-localization.md)
- [ADR-043 — Wall-zero reset nodes](./043-wall-zero-reset-nodes.md) — `reanchorAtWall`, the sibling verb sharing `_bindHeadingFromYaw`.
- [ADR-045 — IMU yaw sign normalization](./045-imu-yaw-sign-normalization.md) — `YawSource`, the sole owner of raw IMU-yaw I/O this reset reads through.
- [ADR-046 — Projection is a node property](./046-projection-as-node-property.md) — the opt-in placement this rule binds to.
- [ADR-048 — Reset nodes are blind-incident only](./048-reset-blind-incidence-and-stop-at-line-contradiction.md) — why the BLIND/RESET no-op switch branch in `_maybeHeadingReset` is defense-in-depth, unreachable on a validator-passed field.
- Modules: `WRO2026/Localization.{h,cpp}` (`reanchorHeading`, `_bindHeadingFromYaw`), `WRO2026/PathTracker.{h,cpp}` (`_maybeHeadingReset`), `WRO2026/config/ConfigNav.h` (the four knobs), `tools/path_planner/sim/{runtime,parity}.py` (the mirror).
- Issue #73.
