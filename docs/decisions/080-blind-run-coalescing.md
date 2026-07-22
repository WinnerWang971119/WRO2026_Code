# ADR-080: A collinear blind run coalesces into one braked `driveDist` over the whole run

**Status:** Implemented
**Date decided:** 2026-07 (phase)

## Rule

A strictly-collinear BLIND dead-reckon run of >= 2 hops — every hop collinear and
same-direction within `Config::NAV_TURN_TOLERANCE_DEG`, sharing one `SpeedTier`, no
pivots, no stop-at-line seek, profiled SYNC/GYRO (never `RAW_FLAT`) — drives as **one**
braked `Drive.driveDist(totalCm).setMode(runMode).tShape()` over the summed hop distance,
not N per-hop moves. On arrival `PathTracker` advances `Localization` through every node of
the run (`onDeadReckonedAdvance` per node) and settles at the final node. Gated by
`Config::PATH_BLIND_CHAIN_ENABLED`; capped by `MAX_CHAIN_HOPS` + `PATH_BLIND_CHAIN_MAX_CM`;
the collinearity gate stays `NAV_TURN_TOLERANCE_DEG`. LINE/PROJECTION collinear pass-through
is unchanged — they re-seek the line at each node and self-correct. This replaces ADR-059's
per-hop blind velocity chain and retires ADR-079's boundary-carry subtraction and stub.

## Why

Bench-confirmed on hardware. With the ADR-059 per-hop chain, the run's final short hop
(6-9 cm) enters at cruise (~67 PWM) and cannot decelerate to rest inside its length — the
real chassis decelerates slower than `TSHAPE_ACCEL_PWM_PER_CM=3.5` assumes, so even a 9.3 cm
`goTo("MWD")` overshot — and each per-hop braked stop coasts ~1 cm across the node boundary,
which accumulated (~8 cm up the mosaic column) into the top wall. One combined move over the
whole run gives the entire run as deceleration runway (a hand-measured 100 cm move landed at
99.98 cm) and, having no inter-hop boundary, leaves no coast to accumulate — fixing the
deceleration and the carry at once. This reopens ADR-059's rejected run-coalescing: its
rejection rested on parity-granularity cost plus a physics claim ("short combined runs won't
land <1 cm") the bench has now refuted, firing ADR-059's own "rejected tradeoff shifts"
trigger.

## How to apply

- `PathTracker::_chainTryStart` — the reused ADR-059 walk-ahead — now fills the run's ordered
  `node[]` and `totalCm` only; it no longer resolves a per-hop velocity plan.
- The `BLIND_DEAD_RECKON` dispatch arm emits ONE braked `driveDist(totalCm)`; `BLIND_FORWARD`
  advances `Localization` through every intermediate node and emits one `blind_pass` parity
  record per node.
- The combined move uses hop 0's resolved mode and body heading. Blind runs are GYRO on this
  chassis; SYNC is unused (crossed encoder cables), so hop 0's mode covers the whole move.
- The parity/sim/CTPS mirrors emit ONE `blind_dead_reckon` (`chain=true`, `dist_cm=totalCm`)
  plus one `blind_pass` per intermediate node — byte-identical firmware/sim under the ADR-050
  strict schema.
- The bench is the arbiter: land <1 cm from the run's final node, clear of the wall.

## Trigger to revisit

- The combined braked move itself cannot land a run's final node within tolerance — e.g. a
  very short 2-hop run, or a real-chassis decel worse than the runway provides.
- A same-tier mixed SYNC/GYRO collinear blind run becomes real (impossible today — SYNC is
  broken/unused, so the run takes hop 0's mode); it would need a per-run mode-resolution rule.

## See also

- [ADR-059 — Collinear runs chain velocity across hops](./059-collinear-run-continuous-motion.md) — **reopened**: its rejected run-coalescing alternative is adopted here; the blind per-hop velocity chain is retired.
- [ADR-079 — Braked-blind boundary carry generalizes beyond chains; chained final-hop stub floors](./079-braked-blind-carry-and-stub-guard.md) — **superseded by this ADR**: the carry subtraction and stub-guard it added are removed, subsumed by coalescing.
- [ADR-023 — Pass-through collinear intersections](./023-pass-through-collinear-intersections.md) — the collinearity predicate and cruise-carry the run walk-ahead reuses.
- [ADR-050 — PathTracker sim↔firmware FSM parity](./050-pathtracker-sim-parity.md) — the strict trace schema the one-record-per-run emit must satisfy on both sides.
- [ADR-058 — CTPS renders the firmware's T-shape motion profile](./058-ctps-firmware-motion-profile.md) / [ADR-061 — Hop settle wait](./061-hop-settle-wait.md) — the `cruise→floor→brake` stop + settle shape the combined move lands on.
- `ChainSpeedPolicy.h` + its `chain_speed_dump`/goldens/`chain_speed.py` isolated tests are now UNUSED by the runtime (dead code — retire in a follow-up), since coalescing dropped the per-hop velocity plan they served.
- `docs/plans/2026-07-22-blind-run-coalescing.md` — the originating spec.
