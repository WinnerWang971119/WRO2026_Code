# ADR-061: Hop settle wait

**Status:** Implemented
**Date decided:** 2026-07 (phase, issue #122)

## Rule

After a braked arrival at a node — and after a cold pre-edge turn on a normal
hop — `PathTracker` holds a non-blocking `State::SETTLE` dwell of
`Config::HOP_SETTLE_MS` before the next maneuver, so residual inertia
dissipates before the re-anchor turn/drive. Eligibility is governed by the
**Settle-fire matrix** (`docs/plans/2026-07-12-hop-settle-wait.md`): it fires
on braked LINE/projection/stop-at-line/non-chain-blind arrivals and on cold
normal-hop turns (including the final turn into goal arrival); it is
**skipped** on pass-throughs, blind-chain intermediate hops, a goal's first
plan (standing start), `RESET`-node arrivals, and the return-to-node/wall-zero
interludes. `HOP_SETTLE_MS == 0` disables the feature entirely — dispatch is
byte-identical to pre-feature.

## Why

Residual inertia at a braked stop corrupts the immediately-following
re-anchor turn/drive: the chassis is still settling (frame flex, wheel
rebound) when the next precision maneuver begins, and that error compounds
into the next node's arrival. An explicit non-blocking FSM state — not an
inline `millis()` gate inside `PLAN_NEXT_HOP`/`TURNING` — matches the
codebase's per-state FSM discipline, is observable via `state()` and
`EventTrace`, and is the natural home for the parity-dump driver and host
test-harness per-state helpers. An inline gate would need a re-entry latch to
avoid re-firing arrival hooks and would hide the dwell from `state()`
entirely.

## How to apply

- Firmware: `PathTrackerClass::State::SETTLE` in `PathTracker.cpp`'s
  `_stepFsm()` — entered via `_enterSettle(SettleResume)` from the four
  braked arrival states (through `_settleOrPlanAfterArrival`) and from the
  `TURNING`-idle branch on a cold, non-return-prologue turn. Rollover-safe
  `millis()` compare, resuming to `PLAN_NEXT_HOP` or the post-turn edge
  dispatch.
- CTPS: the dwell is rendered at the **render layer only**
  (`tools/path_planner/ctps/track_builder.py`) — never in the parity-locked
  `PathTrackerSim`. `ctps_dump` runs against a mock `PathTracker` that never
  executes the real FSM, so the dwell can only originate downstream; the
  render layer already owns synthetic time (`append_dwell_keyframe`,
  `MANIP_NOMINAL_S`) and already classifies braked vs pass-through hops
  (ADR-059's run-grouper). The parity trace excludes timing by construction
  (ADR-050), so `SETTLE` emits no trace record and moves no pose.
- `HOP_SETTLE_MS` is **parsed from the Config source** by
  `tools/path_planner/config_parse.py` into `ProfileConfig.hop_settle_ms` —
  never a hand-mirrored literal (ADR-058 discipline). CTPS therefore renders
  the shipped source default; a firmware `-DHOP_SETTLE_MS_DEFAULT=<n>`
  compile override is invisible to the parser and to CTPS.
- Eligibility is guarded by a **paired tether**, not a policy header: one
  host ctest asserts the firmware fires SETTLE exactly on the Settle-fire
  matrix, and one pytest asserts the CTPS render dwell fires on the same
  matrix. Both cite the matrix directly so a future edit updates both sides
  together; the eligibility rules live on FSM *transitions*, not resolved
  per-hop inputs, so a full `SettlePolicy.h` truth-table is disproportionate
  today.

## Trigger to revisit

- The deferred wall-zero reset interlude (`WALL_STOP → REANCHOR →
  WALL_DEPART`) or the return-to-node prologue gain their own settle —
  both skip settle today and would need their own eligibility rows.
- A per-edge `HOP_SETTLE` override is needed — v1 ships only the global
  `Config::HOP_SETTLE_MS` constant; per-edge tuning belongs to the #104
  EDGES-fields family.
- Either of the above lands: re-open whether the eligibility rules graduate
  from FSM transitions to a dedicated policy header instead.

## See also

- [ADR-050 — PathTracker sim↔firmware FSM parity](./050-pathtracker-sim-parity.md) — the timing-excluded-by-construction trace schema `SETTLE` relies on to stay parity-invisible.
- [ADR-058 — CTPS playback renders the firmware's T-shape motion profile](./058-ctps-firmware-motion-profile.md) — the `config_parse`-sourced, never-hand-mirrored Config discipline this ADR's `hop_settle_ms` follows.
- [ADR-059 — Collinear runs chain velocity across hops](./059-collinear-run-continuous-motion.md) — the braked-vs-pass-through classification the Settle-fire matrix and the CTPS run-grouper both consume.
- [ADR-042 — Operator-offset `Localization` delta](./042-operator-offset-localization.md) / [ADR-043 — Wall-zero reset nodes](./043-wall-zero-reset-nodes.md) — the reset/wall interlude this ADR's eligibility rules deliberately skip.
- `docs/plans/2026-07-12-hop-settle-wait.md` — the originating plan, incl. the full Settle-fire matrix.
- Issue #122.
