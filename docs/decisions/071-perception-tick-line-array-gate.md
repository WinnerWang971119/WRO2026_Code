# ADR-071: Gate LineArray/LineFollower/Intersection tick updates on line-follow-or-seek liveness

**Status:** Implemented
**Date decided:** 2026-07 (issue #164)

## Rule
In `cooperativeTick()`, the three line-sensor consumer updates — `LineArray.update()`,
`LineFollower.update()`, `Intersection.update()` — run only when
`LineFollower.isEnabled() || Drive.isSeekingLine()` (the predicate is resolved once per tick
into a single `const bool` and reused for all three gates). `Drive.update()`,
`Localization.update()`, and every other module tick stay unconditional. The gate sits behind
a compile-time toggle, `Config::LINE_ARRAY_TICK_GATE_ENABLED` (default `true`); when `false`
the predicate constant-folds to `true`, restoring the legacy always-on path with no other
code change.

## Why
The line boards sit on the slow Wire1 I2C bus; `LineArray.update()`'s read costs ~11 ms in
cache mode against an ~8 ms / 124 Hz tick budget. During a pure GYRO/RAW/SYNC profile move, a
`wallStop` push, or pre-mission idle, nothing reads the line cache — `LineFollower.update()`
already short-circuits when disabled, and `Drive`'s only line-data consumer,
`_updateStopAtLine()`, only fires mid-seek. Paying the I2C cost every tick regardless of
whether anything consumed it was pure waste; gating it on actual consumer liveness reclaims
that budget without touching any consumer's internals.

## How to apply
- **Freshness is structural, not an extra refresh.** `cooperativeTick()` runs
  `LineArray.update()` first among the gated three and every consumer after it, so any tick
  the predicate is true refreshes the cache before it is read.
- **Load-bearing internal order:** inside `Drive::update()`, `_updateStopAtLine()` must run
  before `_dispatchNext()`. A queued `stopAtLine` popped this tick arms `_stopState` only
  after the seek sub-FSM has already run, so it consumes no line sample until the following
  tick — whose tick-top predicate is by then true and refreshes first. Do not reorder this;
  the T5 host test asserts it.
- **Reopen drain (state-keyed).** On the gate's closed→open edge, arm a drain that
  read-and-clears `Intersection` every open tick until the departure marker has stably
  released (`Intersection.isJunctionActive()` == `false`), discarding a departure-junction
  latch that would otherwise surface as a phantom PathTracker arrival. Keying off the
  debounced marker state — not a fixed tick count — closes the flicker case where sensor
  noise (e.g. NONE→T→T over the first open ticks) delays the debounced edge past a
  fixed-count window. A settle floor of `Config::INTERSECTION_DEBOUNCE_TICKS` keeps the drain
  armed across a leading tick-1 dropout before the marker asserts and bounds the no-marker
  case (an axle-offset arrival whose sensor sits past the node) so the drain still ends
  promptly. The residual now needs ≥`INTERSECTION_DEBOUNCE_TICKS` consecutive leading
  dropouts at the exact reopen instant — not physically the departure geometry.
- **Reclaimed budget** is exactly the single `LineArray` I2C read; `LineFollower.update()`
  already no-ops when disabled, and the `Intersection` saving is a cheap cache-read skip, not
  an I2C cost.
- **Gate × cache interaction:** with `LINE_ARRAY_TICK_CACHE_ENABLED` true (shipped),
  gate-closed skips the I2C read plus both cheap consumer updates. With cache mode false,
  `LineArray.update()` is already a no-op, but the gate still skips the
  `LineFollower`/`Intersection` inline getter reads — so gating is never counterproductive
  under either cache setting. No `static_assert` couples the two toggles; they vary
  independently.

## Trigger to revisit
A new consumer of the line cache appears outside `LineFollower` or the `stopAtLine` seek (the
predicate would need widening); `INTERSECTION_DEBOUNCE_TICKS` semantics change; or intersection
detection moves off the follow/seek liveness signal entirely.

## See also
- `DriveClass::isSeekingLine()` (T1) — the seek-liveness predicate input.
- `IntersectionClass::isJunctionActive()` — the non-consuming debounced-marker query the
  reopen drain keys "drain until released" off.
- ADR-027 (`Drive.stopAtLine()` / the seek FSM this gate keys off).
- The `LineArray` tick-cache (PR #20, `LINE_ARRAY_TICK_CACHE_ENABLED`).
- Issue #164.
- The `feat/line-approach-ramp` branch (line-approach ramp; ADR number assigned when it
  merges, per the ADR-collision rule the unmerged branch renumbers): touches the same
  SEEK/ARMED states with schedule/ramp math but does not change `_stopState` IDLE/non-IDLE
  transitions, so this gate's predicate stays valid alongside it.
