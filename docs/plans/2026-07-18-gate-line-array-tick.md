# Gate LineArray / LineFollower reads in cooperativeTick (issue #164)

**Goal:** Stop spending the slow Wire1 line-sensor I2C read on ticks where nothing
consumes it — skip `LineArray.update()` / `LineFollower.update()` / `Intersection.update()`
when the robot is not line-following AND not running a `stopAtLine` seek — while
guaranteeing `stopAtLine` and junction detection always see a fresh (this-tick) line sample.

**Approach:** Add one tick-top predicate in `cooperativeTick()` —
`lineArrayNeeded = LineFollower.isEnabled() || Drive.isSeekingLine()` — and gate the three
perception `update()` calls behind it. `Drive.update()` and `Localization.update()` stay
unconditional. Freshness is structural: `cooperativeTick()` already calls
`LineArray.update()` FIRST and every consumer after it, so on any tick the predicate is true
the cache is refreshed at the top before any consumer reads it. On the gate's closed→open
edge, a one-shot Intersection-edge drain discards any departure-junction latch so it cannot
be read as a phantom arrival. Gating is behind a new compile-time toggle
`Config::LINE_ARRAY_TICK_GATE_ENABLED` (default `true`) so the always-read path is one flag
away.

## Scope

- **In scope:**
  - New public predicate `DriveClass::isSeekingLine() const` returning `_stopState != StopAtLineState::IDLE` (the in-flight `stopAtLine` seek).
  - New config toggle `Config::LINE_ARRAY_TICK_GATE_ENABLED` (`constexpr bool`, default `true`) next to `LINE_ARRAY_TICK_CACHE_ENABLED` in `ConfigLine.h`.
  - Gate `LineArray.update()`, `LineFollower.update()`, `Intersection.update()` in `cooperativeTick()` behind a `lineArrayNeededThisTick()` helper; keep `Drive.update()` / `Localization.update()` and everything else unconditional.
  - On the gate's closed→open transition, drain any stale Intersection edge latch (one read-and-clear) so a departure/transition junction cannot be consumed as a phantom arrival.
  - Debug observable: count `LineArray.update()` invocations vs total ticks and surface it in the Serial heartbeat, so AC1 (gate actually closes) is verifiable on hardware.
  - `tick_bench` enhancement: report whole-tick time with the three line segments run vs skipped so the reclaimed budget is a printed number.
  - Host ctest for `Drive.isSeekingLine()` (idle → false, seek in flight → true, after completion/fault → false), plus an assertion that no line sample is consumed on the seek's dispatch tick.
  - ADR documenting the perception-tick gating contract + the freshness guarantee; index it in `DECISIONS.md`.

- **Out of scope:**
  - Any change to `LineFollower`, `WROLineArray`, or `Intersection` internals — the fix is a gate at the call site plus a reopen drain, not a behavior change inside those modules.
  - Touching the `stopAtLine` seek FSM, the approach ramp, or `wallStop` (a `wallStop` push uses the encoder stall watchdog, not the line sensor).
  - Gating `Drive.update()` / `Localization.update()` / `Vision` — these either drive the FSMs every tick or don't consume the line array.
  - Retuning any line gain / threshold (that is the separate `test/tune-robot-everything` work this branch is stacked on).

## Decisions

- **Predicate = `LineFollower.isEnabled() || Drive.isSeekingLine()`, evaluated once at tick-top.** A repo-wide sweep of the sketch tree (`grep -E 'LineArray\.(getError|isOnline|getJunctionType|readActiveBoard|readBoard)'` and the Intersection consumers) confirms these are the ONLY live consumers of the line cache: `LineFollower.update()` reads the getters only when enabled (`LineFollower.h:384` short-circuit); `Drive._updateStopAtLine()` reads `LineArray.getJunctionType()` directly in its ARMED state (`Drive.cpp:1285`); `Intersection.update()` feeds PathTracker's only real edge consume, which lives in `State::LINE_FOLLOW` (`PathTracker.cpp:944`) — every other `isAtIntersection()` call is a `(void)` drain. `LineArray.calibrate()` (`WRO2026.ino:306`) and the blocking-follow-helper `LineArray.update()` calls (`LineFollower.cpp:877/945`) run only during active follow/idle, not in the gated tick path. No consumer needs the line array during a pure GYRO/RAW/SYNC move or pre-mission idle.
- **Freshness comes from tick order, not an extra refresh — and it has one load-bearing dependency.** `cooperativeTick()` runs `LineArray.update()` first and all consumers after, so a true predicate at tick-top refreshes the cache before any read. The dispatch-tick case is covered by `Drive::update()`'s internal order: `_updateStopAtLine()` (`Drive.cpp:136`) runs BEFORE `_dispatchNext()` (`Drive.cpp:149`), so a *queued* `stopAtLine` popped this tick arms `_stopState` AFTER the seek FSM already ran — it consumes no line sample until the next tick, whose tick-top predicate is true and refreshes first. **This ordering is load-bearing**: if anyone reorders `_dispatchNext()` ahead of the sub-FSM updates, the guarantee breaks — so it is asserted in the host test (T5) and stated in the ADR.
- **The #149 no-op-flip skip does not reintroduce staleness.** `setReverseActive()` DOES refresh the cache on a *real* board flip when cache mode is on (added in commit `59b3839`, `#149`) and skips the refresh only on a no-op same-direction flip. But this gate does not depend on that: because `LineArray.update()` itself refreshes every needed tick at tick-top, a reverse seek whose flip is a no-op still gets a fresh sample from `update()` before `_updateStopAtLine()` reads. The `#149` mechanism and this gate are independent safety nets, not a chain.
- **Gate × cache-toggle interaction.** The reclaim is `LineArray.update()`'s I2C read (~11 ms in cache mode). With `LINE_ARRAY_TICK_CACHE_ENABLED == true` (shipped) the gate skips that read plus the two cheap consumer updates. With cache `false`, `LineArray.update()` is already a no-op but the gate still skips `LineFollower`/`Intersection` inline getter reads, so gating stays a net win — it is never counterproductive. No `static_assert` couples the two toggles (gating is valid under either cache mode); the four-combination matrix is documented in the ADR.
- **Toggle `LINE_ARRAY_TICK_GATE_ENABLED`, default `true`** (operator choice). `false` makes the predicate constant-fold to `true`, restoring today's unconditional three-update path. `constexpr` so the off-branch is dead-code-eliminated — mirrors `LINE_ARRAY_TICK_CACHE_ENABLED`.
- **Predicate kept inline in `WRO2026.ino`** as a small `static` helper, not extracted to a policy header. It is a two-input boolean OR; its only non-trivial input, the Drive seek state, is host-tested via `isSeekingLine()`. A policy header for an OR would be the speculative abstraction the coding rules forbid.
- **`tick_bench` gets a gated-vs-ungated measurement** driven by a bench-local runtime bool (NOT the `constexpr` toggle — a compile-time constant cannot measure both paths in one run), alternating measurement windows so both whole-tick times print.
- **Base / integration:** worktree branched off `test/tune-robot-everything` (operator is merging that to `main` later; the issue's line citations and `LINE_ARRAY_TICK_CACHE_ENABLED == true` match that branch). PR targets `test/tune-robot-everything` (or `main` after that merge). See Notes for the `feat/line-approach-ramp` overlap.

## Assumptions (auto-recorded, --auto)

- `Drive.cpp`'s only line-array DATA READ is `getJunctionType()` in `_updateStopAtLine()` (`Drive.cpp:1285`). Its other `LineArray.*` calls are `setReverseActive()` board-selects (`Drive.cpp:1348/1519/1726`) — one of which (`1519`) is a `wallStop` cleanup — that issue NO junction read. So a pure `wallStop` push touches `LineArray` only to reset the active board, never to read line state; leaving the gate closed during `wallStop` is correct. ✔ verified in code.
- `LINE_ARRAY_TICK_CACHE_ENABLED == true` on both `main` and `test/tune-robot-everything`, so the cache-mode stale-read guard is the live concern. ✔ verified.
- Nothing consumes the line array during pre-mission idle or the calibration menu — `WROLineArray::calibrate()` reads boards directly, not via `update()`. Closing the gate at idle is safe and a small budget win. ✔ verified via sweep.
- A `stopAtLine` seek always arms on clean line (`armWithin` cm before the expected crossing), never sitting on a junction. The one case where a consumer sees a junction the moment the gate opens is a follow hop *departing* the node it just reached — handled by the reopen drain (see Error Handling), not by this assumption.

## Acceptance Criteria

- [ ] AC1 — On a pure `GYRO`/`RAW`/`SYNC` move with line-follow off and no seek in flight, the gate closes: `LineArray.update()`, `LineFollower.update()`, `Intersection.update()` are skipped. Observable via the heartbeat's `update()`-count-vs-ticks readout (near-zero updates during such a move).
- [ ] AC2 — A `stopAtLine` seek (forward and, hardware permitting, reverse) still stops on the line: `LineArray.update()` runs every tick `Drive.isSeekingLine()` is true, so `_updateStopAtLine()`'s `getJunctionType()` reads a fresh, this-tick `T_OR_CROSS` sample.
- [ ] AC3 — With line-follow enabled, all three perception updates run, the follower sees a fresh error sample, and intersection rising edges are still detected/consumed by PathTracker.
- [ ] AC4 — On the closed→open transition the first consuming read is a fresh sample, never a pre-gap stale cache; and a departure-junction latch formed on the first open tick is drained, not consumed as a phantom arrival.
- [ ] AC5 — With `LINE_ARRAY_TICK_GATE_ENABLED == false`, the predicate constant-folds to `true` and the three updates are unconditional (checkable by compile + code review), matching today's behavior.
- [ ] AC6 — `tick_bench` prints whole-tick time for the gate-open vs gate-closed path, quantifying the reclaimed budget (the `LineArray` I2C segment).
- [ ] AC7 — Host ctest: `Drive.isSeekingLine()` is `false` before any seek, `true` while a `stopAtLine` seek is in flight, and `false` again once it completes or faults `LINE_NOT_FOUND`; and no line sample is consumed on the seek's dispatch tick.
- [ ] AC8 — `arduino-cli compile` and the full `ctest` suite pass.

## Contracts & Interfaces

Single source of truth for every seam more than one task touches.

### Signatures

- `bool DriveClass::isSeekingLine() const` — returns `_stopState != StopAtLineState::IDLE`. **Owner:** T1. **Consumers:** T3 (`cooperativeTick` gate), T5 (host test). **Contract:** keys off the seek FSM state only, so it stays valid regardless of any approach-ramp sub-phase layered inside SEEK/ARMED by a parallel branch.
- `constexpr bool Config::LINE_ARRAY_TICK_GATE_ENABLED = true;` — **Owner:** T2. **Consumers:** T3. When `false`, the predicate constant-folds to `true` and the three updates run unconditionally.
- `static bool lineArrayNeededThisTick();` in `WRO2026.ino` = `LineFollower.isEnabled() || Drive.isSeekingLine()` (or `true` when the toggle is off). **Owner:** T3. Internal to the sketch. T4 mirrors its *shape* with a bench-local runtime bool (it does NOT consume the `constexpr` toggle).

### File ownership

| File | Owner task | Consumer tasks |
|------|-----------|----------------|
| `WRO2026/src/drive/Drive.h`, `Drive.cpp` | T1 | T3, T5 |
| `WRO2026/config/ConfigLine.h` | T2 | T3 |
| `WRO2026/WRO2026.ino` | T3 | — |
| `inoTest/tick_bench/tick_bench.ino` | T4 | — |
| `WRO2026/test/<new seek-predicate test>.cpp` | T5 | — |
| `WRO2026/test/CMakeLists.txt` (append) | orchestrator-coordinated | T5 |
| `docs/decisions/ADR-0NN-*.md`, `DECISIONS.md` | T6 | — |

### Naming

- Predicate method: `isSeekingLine` (not `isStopAtLineSeeking` — shorter, and unambiguous since `wallStop` is a distinct verb that does not seek the line).
- Config constant: `LINE_ARRAY_TICK_GATE_ENABLED` (parallels `LINE_ARRAY_TICK_CACHE_ENABLED`).

## Error Handling

- **Predicate wrongly closes the gate during a seek** → skipped/stale line read → seek never confirms a crossing → `LINE_NOT_FOUND` fault at max distance. Guarded by keying the predicate on `Drive.isSeekingLine()` (host-tested, AC7) plus the tick-order freshness proof (AC2/AC4).
- **Phantom junction on gate reopen (frozen-Intersection hazard).** When a follow hop departs a junction node the robot reached with the gate closed (a blind/GYRO arrival, or a `wallStop` re-anchor onto a node — nodes are junctions), the first gate-open tick runs `Intersection.update()` fresh over the departure marker and latches a `T_OR_CROSS` edge that PathTracker's `State::LINE_FOLLOW` consume (`PathTracker.cpp:944`) would read as an instant arrival at the next node. PathTracker's existing stale-edge drain (commit `3254df5`) can be out-ordered by the gate (Intersection ticks at step 4, before PathTracker at step 7). **Mitigation (as shipped — state-keyed drain):** on the gate's closed→open edge, `cooperativeTick()` arms a drain that read-and-clears `Intersection.isAtIntersection()` after `Intersection.update()` **every** open tick until the departure marker has stably released (`Intersection.isJunctionActive()` == `false`), with a settle floor of `Config::INTERSECTION_DEBOUNCE_TICKS` ticks. A fixed-count window is insufficient: `Intersection` latches the departure edge only once its debounce streak reaches `INTERSECTION_DEBOUNCE_TICKS`, and sensor flicker (e.g. NONE→T→T) can push that latch past a fixed window. Keying the drain off the debounced marker state instead of a tick count closes that flicker case; the settle floor bridges a leading tick-1 dropout and bounds the no-marker (axle-offset) arrival so the drain still ends promptly. Safe because a genuine next-node arrival cannot occur within one debounce window of follow start. (Residual now requires ≥`INTERSECTION_DEBOUNCE_TICKS` consecutive leading dropouts at the exact reopen instant — not physically the departure geometry; resolved via option (b), decided in the PR discussion.)
- **I2C read failure on a gate-open tick** → unchanged from today: `_refreshCache()` sets `_lastOnline = false`, getters report lost-line, `LineFollower` dispatches `Config::LOST_LINE_ACTION`.
- **Gate closed at idle / during a `wallStop` / during a pure profile move** → no line consumer active → safe by construction; the intended budget win.

## Testing Strategy

**Levels:** Host unit (ctest) for the Drive predicate + dispatch-tick ordering; on-hardware for the gate itself (the sketch's `cooperativeTick()` is not host-compiled); `arduino-cli compile` for build integrity; `tick_bench` for the budget claim.

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | `isSeekingLine()` before any seek | Host unit | `false` |
| TC2 | `isSeekingLine()` while a `stopAtLine` seek is in flight | Host unit | `true` |
| TC3 | `isSeekingLine()` after the seek confirms a crossing | Host unit | `false` |
| TC4 | `isSeekingLine()` after the seek maxes out → `LINE_NOT_FOUND` | Host unit | `false` |
| TC5 | No `getJunctionType()` read on the tick a queued `stopAtLine` is dispatched | Host unit | Line sample consumed 0 times that tick (asserts the `_updateStopAtLine`-before-`_dispatchNext` order) |
| TC6 | Reverse `stopAtLine` seek through a mocked rear board | Host unit | `isSeekingLine()` true during, false after; stops on mocked `T_OR_CROSS` (host stand-in for TC10 when the rear board is unwired) |
| TC7 | Gate closed on a pure GYRO move (no follow, no seek) | On-hardware | Heartbeat shows near-zero `LineArray.update()` calls; move unaffected (AC1) |
| TC8 | Forward `stopAtLine` under the gate | On-hardware | Stops on the line (AC2) |
| TC9 | `LINE_ARRAY_TICK_GATE_ENABLED = false` regression | Compile + review | Predicate folds to `true`; three updates unconditional (AC5) |
| TC10 | Reverse `stopAtLine` under the gate (rear board) | On-hardware | Stops on the line; no stale front-board frame (AC4). **Precondition:** rear board physically wired (`LINE_BOARD_COUNT == 2`, ADR-024); if unwired, TC6 is the standing proof. |
| TC11 | `tick_bench` gate-open vs gate-closed | On-hardware | Prints two whole-tick times; delta ≈ `LineArray` segment (AC6) |
| TC12 | Follow hop departing a junction node reached blind | On-hardware | No phantom instant-arrival; robot follows to the real next node (reopen-drain, AC4) |

**Test data:** The host tests reuse the existing real-`Drive.cpp` + mocked-`WROLineArray` regime that `test_path_tracker_stop_at_line` (`WRO2026/test/CMakeLists.txt`) already scripts — advance the mocked M2/M3 encoders past the arm threshold (so the seek reaches ARMED rather than stall-faulting) and step the mock clock for the TC4 max-out path. No new fixtures.
**Run command:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Add `Drive::isSeekingLine()` | — | low | `WRO2026/src/drive/Drive.h`, `WRO2026/src/drive/Drive.cpp` | Add a public `bool isSeekingLine() const` next to `isLineFound()` (Drive.h ~line 497). Body: `return _stopState != StopAtLineState::IDLE;`. Full Doxygen `@brief/@return/@note` matching the module tone; `@note` that `wallStop` is separate and does NOT open the line gate, and that this keys off the seek FSM state (robust to any approach-ramp sub-phase). Pure query, no state change. Satisfies AC2/AC7 (predicate input). |
| T2 | Add `LINE_ARRAY_TICK_GATE_ENABLED` toggle | — | low | `WRO2026/config/ConfigLine.h` | Add `constexpr bool LINE_ARRAY_TICK_GATE_ENABLED = true;` immediately after `LINE_ARRAY_TICK_CACHE_ENABLED` (~line 248) with a Doxygen block: when `true`, `cooperativeTick()` skips the three line-consumer updates on ticks with no follow/seek; when `false` the predicate folds to `true` and they run unconditionally (legacy). While here, correct the now-stale `LINE_ARRAY_TICK_CACHE_ENABLED` doc (it still says "default `false`" though the value is `true`). Keep all prose in the Doxygen block; do NOT add a trailing `//` comment to any `#define` line (config-parse regex breaks on a trailing comment on a `#define X_DEFAULT` line — not an issue for `constexpr`, but do not introduce one). Satisfies AC5. |
| T3 | Gate the three perception updates in `cooperativeTick()` + reopen drain + observable | T1, T2 | high | `WRO2026/WRO2026.ino` | Add `static bool lineArrayNeededThisTick()` returning, when `Config::LINE_ARRAY_TICK_GATE_ENABLED`, `LineFollower.isEnabled() \|\| Drive.isSeekingLine()`, else `true`. In `cooperativeTick()` resolve it ONCE into `const bool lineNeeded` and wrap `LineArray.update()` (149), `LineFollower.update()` (152), `Intersection.update()` (153) in `if (lineNeeded)`. Track the previous tick's value in a `static bool`; on the closed→open edge, after `Intersection.update()`, do a one-shot `(void) Intersection.isAtIntersection()` drain (Error Handling → phantom junction). Add a debug counter of `LineArray.update()` invocations vs total ticks and surface it in the Serial heartbeat (behind the existing heartbeat cadence, no new `delay`). Leave `Drive.update()`, `Localization.update()`, and the rest unconditional. Update the doxygen block (129-148) to describe the gate, the tick-order freshness guarantee, and the load-bearing `_updateStopAtLine`-before-`_dispatchNext` dependency. Do NOT change the segment set (no `SEGMENT_COUNT` bump). Satisfies AC1/AC3/AC4/AC5. |
| T4 | `tick_bench` gated-vs-ungated measurement | T3 | low | `inoTest/tick_bench/tick_bench.ino` | Extend `benchTick()`/the summary so the bench reports whole-tick time twice — running the `LineArray`/`LineFollower`/`Intersection` segments vs skipping them — driven by a bench-local runtime `bool` toggled between measurement windows (NOT the `constexpr` config toggle, which cannot measure both paths in one run). Mirror T3's predicate shape. Keep the "KEEP IN SYNC WITH cooperativeTick()" note accurate. Bench-only; no production weight. Satisfies AC6. |
| T5 | Host ctest for `Drive::isSeekingLine()` + dispatch-tick order | T1 | low | `WRO2026/test/test_drive_seek_predicate.cpp` (new), `WRO2026/test/CMakeLists.txt` (append) | New host test linking the REAL `Drive.cpp` (mirror the mock regime of the existing `test_path_tracker_stop_at_line` target — advance mocked encoders past the arm threshold, step the mock clock). Drive a forward seek `NONE`→`T_OR_CROSS` and a reverse seek through a mocked rear board; assert `isSeekingLine()` across TC1–TC4/TC6 and assert no `getJunctionType()` consume on the dispatch tick (TC5). Append one `add_executable` + `add_test` — the execute orchestrator owns the `CMakeLists.txt` append to avoid racing the parallel `feat/line-approach-ramp` appends (their targets: `test_approach_ramp_policy`, `approach_ramp_dump`, `approach_ramp_freshness`). Satisfies AC7. |
| T6 | ADR for the perception-tick gating contract | T3 | low | `docs/decisions/ADR-0NN-perception-tick-gate.md` (new), `DECISIONS.md` | Write an ADR (next free number read from `DECISIONS.md` at implement time; if `feat/line-approach-ramp` grabbed the same number, this unmerged branch renumbers — known ADR-collision rule). Rule = the three line-consumer updates are gated on `LineFollower.isEnabled() \|\| Drive.isSeekingLine()`; freshness guaranteed by the `LineArray`-first tick order + the `_updateStopAtLine`-before-`_dispatchNext` order; reopen drain kills the phantom junction; the budget win is the single `LineArray` I2C read; the 4-way gate×cache matrix. Toggle `LINE_ARRAY_TICK_GATE_ENABLED`. Add the index line + status-table bump in `DECISIONS.md`. |

Parallelism: T1 and T2 run concurrently. T3 waits for both. T4 waits for T3. T5 waits for T1 (parallel to T3/T4). T6 waits for T3.

## Notes for Implementer

- **Resolve the predicate once per tick** into `const bool lineNeeded` and reuse it for all three `if`s so they cannot disagree within a tick.
- **`Drive.update()` must stay unconditional** — it drives the profile/seek/wallStop FSMs and the queue; `_updateStopAtLine()` there reads the (now-guaranteed-fresh) cache. Only `LineArray`/`LineFollower`/`Intersection` updates are gated. Do NOT reorder `_dispatchNext()` ahead of the sub-FSM updates in `Drive::update()` — the dispatch-tick freshness proof depends on that order (TC5 guards it).
- **`Localization.update()` stays unconditional** — the .ino comment at 158-164 already spells out why (encoder integrator + nav gate).
- **Parallel branch `feat/line-approach-ramp` (in progress in `wt-line-approach-ramp`).** It edits `Drive.cpp`, `Drive.h`, and `WRO2026/test/CMakeLists.txt` — the same files as T1/T5. The overlap is **textual only**: it adds `ApproachRampPolicy` schedule members and ramp math inside the SEEK/ARMED states but does NOT change the `_stopState` IDLE/non-IDLE transitions, so `isSeekingLine()` stays correct after it lands. Whichever branch merges second resolves the `Drive.h`/`Drive.cpp` conflicts (place `isSeekingLine()` beside `isLineFound()`, independent of the ramp) and the `CMakeLists.txt` append (distinct target names). No design change needed; do not wait on that branch.
- **Do not extract the predicate to a policy header** — it is a two-input OR; the reviewable/testable part is `isSeekingLine()`, host-tested. Keep the helper `static` in the sketch.
- **Rollback:** flip `LINE_ARRAY_TICK_GATE_ENABLED` to `false` — restores the always-read path with no other change.
- Host-mock reminder: the integration test links the REAL `Drive.cpp`; a `LineFollower`-only mock regime would mock Drive and give a false green. Link the real Drive, per the existing `test_path_tracker_stop_at_line` setup.
