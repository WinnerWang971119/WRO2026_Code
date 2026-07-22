# ADR-079: Braked-blind boundary carry generalizes beyond chains; chained final-hop stub floors, not planned-PWM brakes

**Status:** Implemented
**Date decided:** 2026-07 (phase)

## Rule

The ADR-059 measured inter-hop boundary-carry subtraction generalizes from
chained runs only to **any consecutive braked blind dead-reckon hops**, via a
chain-lifecycle-independent carry state owned by `PathTracker`
(`_brakedBlindCarryActive`/`_brakedBlindEncSum`/`_brakedBlindCmdCm`/`_brakedBlindReverse`).
It snapshots on every braked blind dispatch (chain-entry, chain-final, and
non-chain alike) and is read-and-subtracted only on a contiguous same-direction
braked blind hop that the in-chain carry block is not already compensating
(the `!match` guard is the sole double-subtract interlock — SNAPSHOT still
runs either way). It resets to inactive on: a turn, a non-blind hop
(LINE/PROJECTION/seek/STOP_AT_LINE), a wall/line re-anchor, a fault, a return
to `IDLE`, a fresh-goTo origin bind (the `IDLE→PLAN_NEXT_HOP` transition only —
a within-goTo continuation re-entry does not reset), a direction flip (which
re-arms at carry=0 rather than sign-flipping), and a non-braked dispatch
(mid-chain flow-through, `RAW_FLAT`); the chain-divergence fallback explicitly
invalidates it at the divergence site, not via `_chainReset()`.

Separately, a **chained-run final hop** that this carry subtraction shrinks
below `TSHAPE_MIN_MOVE_CM` now dispatches its degraded stub flat at the motor
floor (`±PWM_MIN_MAGNITUDE`, sign-resolved by direction), not at the plan start
PWM the backward pass planned against the hop's full geometric length. A
mid-chain stub is unaffected — it still flows at its planned `chainEndPwm`.

Run-coalescing (one combined `driveDist` per collinear blind run) remains
**rejected as primary** — ADR-059's parity-granularity ground still holds and
its four-surface fan-out is unchanged. It is recorded here as a documented
bench fallback, ordered behind a `ChainSpeedPolicy` terminal brake-to-rest
reserve (`R ≈ 4 cm`), should bench data show this fix's residual insufficient.

## Why

The ~8 cm blind-column overshoot (`goTo("MWU")` up the mosaic column) was two
sequencing defects, not a per-hop landing defect — a short single hop already
lands accurately on this chassis (Diego, bench). With chaining off, the
measured-carry subtraction lived only inside the chain's `if (_chain.active)`
block, so the plain per-hop braked path commanded the full geometric distance
every hop; the ~1–3 cm boundary carry each braked stop leaves (chassis rolling
on the last PWM across the node boundary, discarded when
`Localization::onDeadReckonedAdvance()` re-anchors to the exact node) then
accumulated uncompensated over the 8-hop column. With chaining on, the
chain's backward pass planned the final hop's entry PWM (~67) against its full
length; subtracting the measured carry from that hop's *distance* then dropped
it below `TSHAPE_MIN_MOVE_CM`, and the degraded-stub branch dispatched the
shrunk hop flat at that planned ~67 PWM, braking from speed into the wall
instead of crawling in. Fixing both in place preserves hop-granular parity and
needs no new firmware surface: the sim/parity harness parks encoders (carry
always 0) and the stub PWM is a dispatch detail the parity emit never records
(it emits plan endpoints), so both fixes are byte-invisible to the trace and
must be pinned by host ctests asserting the real `MiniR4.DriveDC` setpoint.

## How to apply

- The generalized carry reuses the **same shared helper** as the in-chain
  carry arithmetic — extracted once, never copied — so the two paths cannot
  drift apart (the `gotcha-drive-constant-retune-fanout` family of bug).
- The stub floor pin mirrors the existing `chainFinalEndPwm` floor pin and is
  deliberately **not** recorded in the parity trace (`#ifdef PT_PARITY_TRACE`
  still emits the plan `signedStartSpeed`/`0`); correctness is proven only by
  a host ctest on the real dispatched setpoint, never the plan record.
- Bench acceptance — land <1 cm from the run's final node, clear of the wall,
  both chaining on and off — is the arbiter; a green host-test suite or a
  green parity job is necessary but not sufficient evidence either fix works.

## Trigger to revisit

- Bench shows the stub-guard still leaves a residual isolated to the
  chain-final hop (AC1 fails while AC2 passes) → apply the `ChainSpeedPolicy`
  terminal brake-to-rest reserve (`R ≈ 4 cm`) as the first fallback before
  touching coalescing.
- The terminal reserve also proves insufficient → reopen run-coalescing
  (Option A), the same coalescing tradeoff ADR-059 already defers against its
  parity-granularity cost.
- Bench shows the residual is a constant per-hop bias independent of hop count
  rather than an accumulating carry — that falsifies the carry mechanism
  itself and the fix target must be revisited before either fallback.

## See also

- [ADR-059 — Collinear runs chain velocity across hops instead of stopping at every node](./059-collinear-run-continuous-motion.md) — amended: this ADR generalizes its in-chain boundary-carry subtraction to non-chain braked blind hops and floors the chained final hop's degraded stub instead of braking it from plan PWM.
- [ADR-058 — CTPS playback renders the firmware's T-shape motion profile](./058-ctps-firmware-motion-profile.md) — the `TShapeProfilePolicy` floor/brake shape the stub-guard now matches on a degraded final hop.
- [ADR-018 — LineFollower distance-gated stops use symmetric trapezoidal + active brake](./018-linefollower-distance-gated-stops.md) — the `cruise→floor→brake` stop shape this ADR's stub now also lands on.
- Memory: `project-fewer-stops-better-friction-drift` — the bench rationale ADR-059 already cites for treating blind sequencing defects as high priority.
- `docs/plans/2026-07-22-blind-column-overshoot.md` — the originating plan (Decisions + Defect Analysis + Contracts sections).
