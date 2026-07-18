# Issues #141–159 hardening batch — Plan

**Goal:** Fix the 14 open, actionable issues in the range #141–159 — a firmware/perception/planner/sim/config hardening sweep that removes several latent-but-real robustness hazards ahead of the 2026-07-21 regional — on one worktree branch, with each issue (or #154 sub-item) as its own commit.

**Approach:** Each issue has a settled fix direction, verified against current source by four code-mapping passes and hardened by two independent adversarial critics (Fable + Codex). Group the work by the file each fix touches so parallel executors never collide: internally-serialized lanes for the four shared files (`Drive.cpp`, `PathTracker.cpp`, `sim/runtime.py`, `validator.py`), plus independent singletons. Two fixes establish a new cross-module contract and get an ADR (#141 → ADR-066 `EncoderSource`, mirroring ADR-045 `YawSource`; #145 → ADR-067 fault-clear cascade). No fix changes `header_writer.py`/`field_data`, so `routes.h/.cpp` stay byte-identical — a final `build_routes.py` freshness check proves it.

## Scope

- **In scope (14 issues):**
  - **Firmware/nav:** #141 (EncoderSource: timeout-as-zero), #142 (turn passed-target guard), #145 (fault-clear cascade), #146 (lost-line fault escalation), #149 (line-board stale-cache on flip), #150 (drain ignores latched fault), #153 (fabricated 0.0 yaw on turn dispatch), #154-#16 (wallStop SQUARE fault guard).
  - **Config:** #152 (junction-knob + turn-ladder static_asserts, stale comments, honest `MISSION_ALLOW_REARM` docs + boot banner, optional servo-drain assert).
  - **Vision:** #154-#18 (`getRawHue` sentinel `0` → named `RAW_HUE_NONE`).
  - **Editor:** #154-#19 (JSON round-trip drops explicitly-authored default keys).
  - **Planner/sim/ctps:** #147 (turn-gate parity — full fix), #148 (validator PIPELINE_* resolution — **resolved by closing as obsolete**: ADR-064/#158 dissolved its premise; see Decisions), #151 (swapped-pair `EDGES` heading clobber guard), #157 (CTPS RETURN speed), #159 (sim pass-through entry-seed tier), #154-#17 (ResetNode facing — round in sim), #154-#20 (mosaic loader whitelist).
  - **Resolution note:** 13 issues land as code; **#148 is resolved by closing it** as superseded by ADR-064 (#158), with an explanatory comment — no code change (see Decisions).
- **#154 sub-items** (5): #16 wallStop (firmware, T4), #17 facing (sim, T10), #18 Vision (T14), #19 editor (T18), #20 mosaic (T17). Each commits separately referencing #154; the issue closes when all five land.
- **Out of scope (decided in planning):**
  - **#155** (extract Drive sub-FSMs to `*Policy.h`) — deferred; its premise ("Drive.cpp is not host-compiled") is obsolete (Drive.cpp *is* host-compiled — 24 targets link it), so #142's guard lands with a direct Drive host test instead.
  - **#156** (LEFT/RIGHT lateral offset-follow) — genuinely-unbuilt deferred feature; no current mission needs it.
  - **#143, #144** (merged PRs) and **#158** (the open per-edge-speed PR = the base branch itself) — not fix targets.
  - **#141 plausibility gate** (rejecting corrupted-but-complete UART frames) — the issue explicitly scopes this out.
  - Flipping `MISSION_ALLOW_REARM` to `false` — a deliberate separate pre-competition action; this batch keeps the bench `true` value and only corrects the docs + adds a boot banner.
  - **Firmware wall-depart at NORMAL** (#157): the firmware's `WALL_DEPART` move drives at the Drive NORMAL default (no `.setPower`), unlike the return moves that use `PIPELINE_CONFIG.speed`. #157 makes CTPS *mirror the firmware*, so CTPS keeps `ACT_WALL_DEPART` at NORMAL. Whether firmware wall-depart *should* also use pipeline speed is a separate firmware question, not this batch.

## Decisions

- **Base branch:** worktree `fix/issues-141-159-hardening` is cut from `feat/per-edge-mode-speed` (not `main`), because #157/#159 (and part of #147) patch code that only exists on that branch (#158/ADR-064). Rebase onto `main` after #158 merges.
- **Worktree:** all work happens in `/Users/diego/Documents/WRO2026/wt-fix-141-159`.
- **PR shape:** one PR off the branch (explicitly overriding CONTRIBUTING's "one concern per branch" for this batch), one conventional commit per issue / per #154 sub-item, so review goes commit-by-commit. The PR description carries the required `Doxygen reviewed for affected APIs before implementation.` line.
- **#141 EncoderSource design (refined after review):** a `src/util/EncoderSourceClass` singleton `EncoderSource` (util = below every consumer: `Drive`, `LineFollower`, `ManipBelt`; avoids a manip→nav/drive cross-domain edge; naming mirrors `YawSourceClass`/`YawSource`). It owns a **centralized per-motor last-good cache + validity**. Two read forms: `readDegrees(id, out)` (surfacing — leaves `out` untouched on double failure, retry-once) for **baseline/arm/calibration** reads that MUST be fresh; `readDegreesOr(id, fallback)` (returns last-good) for **continuous completion** reads where a one-tick stale value is safe. A baseline/arm/calibration read that fails is **not** seeded with stale data — the caller retries/skips. `resetCache(id)` invalidates the cache and is called wherever `resetCounter()` re-zeros a motor. A genuine `0` with a success result is always kept.
- **#141 task-done:** `isPrevTaskDone()` wraps the vendor `MiniR4.DriveDC.isPrevTaskDone()` → `MMLower::Get_Drive_isTaskDone(uint8_t, bool*)` returning `MMLower::Drive_RESULT`; treat any non-`Drive_RESULT::OK` as "not done". Drive instance id is a named `DRIVE_INSTANCE_ID`.
- **#146 grace window:** new `Config::LINE_LOST_FAULT_MS = 500` (placeholder, `// TODO bench-tune`) in `ConfigNav.h`; the timer starts on first sustained loss, resets on reacquire, and is cleared on state-exit/abort/fault. #146 adds a **bounded lost-line fault escalation** in PathTracker; the full ADR-009 sweep/reacquire/resume recovery stays deferred (this does not claim to implement ADR-009).
- **#147:** full fix — align the sim + compiler turn-emission gate to `NAV_TURN_TOLERANCE_DEG`, applying ONE logical-heading rule (advance heading to the edge when a sub-tolerance turn is skipped) consistently across `_emit_turn_to`, `compile_path`, AND `final_heading_after`, so cumulative sub-tolerance hops cannot become a compiler-only turn. Add a parametric parity test over every mirrored constant; add near-collinear single-hop AND multi-hop-cumulative fixtures. `runtime.py`'s second `TURN_EPSILON_DEG` use (the primitive-apply path) stays at `1e-6` intentionally.
- **#148 (closed as obsolete):** on this post-ADR-064 base the divergence #148 describes **no longer reproduces** — the seek classification depends only on `stop_at_line_override` (already returned by `_edge_attrs`) and `auto_rule_active`, not the singleton-vs-field args #148 targets, and `STOP_AT_LINE` is unauthorable as `mode`/`PIPELINE_BLIND_MODE` (verified against `resolve_stop_at_line`'s "intentionally NO pipeline-default seek rung" contract; confirmed by both critics + the Opus judge). Per the review and the user's decision, **#148 is resolved by closing it** as superseded by ADR-064 (#158) — no code. The batch posts a closing comment on the GitHub issue explaining the premise no longer holds. (The reduced defensive-alignment + regime-parity-test option was considered and declined.)
- **#152 `MISSION_ALLOW_REARM`:** keep the value `true`; rewrite the three docs (ConfigMission.h, OPERATOR_GUIDE.md) to state the truth (ships `true` for bench, MUST flip `false` before competition) and add a self-silencing boot `[WARN]` banner.
- **#154-#17 facing:** round facing identically in the sim/`LocalizationSim` (`int(round(x)) % 360`) to mirror the int16 firmware; no struct/emitter/routes change.
- **#154-#20 mosaic:** reject the **whole** mosaic block (warn once, `trace.mosaic = None`) if any cell is invalid — skipping a single cell would break the fixed 12-cell index contract. Validate against `mosaic.COLOR_NAMES ∪ {mosaic.AIR_NAME}` (not a literal `"UNKNOWN"`).
- **New ADRs:** ADR-066 (`EncoderSource`), ADR-067 (Drive fault-clear cascade). Next free numbers (065 is latest). A dedicated final task (T19) owns the shared `DECISIONS.md` index/status-table edits so the two ADR tasks don't collide on it.
- **No emitter change / routes regen:** confirmed `build_routes.py` runs validator → graph_builder → planner → header_writer (it never imports `compiler.py`), and `routes.cpp` holds only `PIPELINE_CONFIG/NODES/NEXT_HOP/EDGES/PIVOT_TURNS/RESET_NODES` — no compiled turn primitives. So #147's compiler-gate change cannot alter `routes.cpp`. A final `build_routes.py` + `git diff --exit-code` gate proves byte-identity.

## Acceptance Criteria

- [ ] **#141** — Encoder/task-done reads route through `EncoderSource`: a timed-out read never reads as `0`/`true`; continuous checks get last-good; baseline/arm/calibration reads that fail do NOT arm on stale data; `resetCache` fires wherever `resetCounter` does; the task-done bit treats any non-`Drive_RESULT::OK` as "not done". A genuine success-`0` is preserved. Host tests prove: last-good on a mocked continuous timeout; no-arm on a mocked baseline timeout; cache-invalidation after reset; non-OK task-done → not-done.
- [ ] **#142** — An in-flight turn whose heading crosses the tolerance band between ticks brakes on the next tick (worst-case one tick of overshoot), in **any** active turn state (CRUISE or SLOW). Host tests drive a SLOW straddle, a CRUISE-to-opposite-side jump, and a ±180° wrap crossing.
- [ ] **#145** — After any latched Drive fault, BTN_DOWN reset, `Mission::abort()`, and `PathTracker::abort()` all clear `Drive._fault` via the cascade; the next `goTo()` does not re-trip `DRIVE_FAULT`. `declarePosition`'s now-redundant explicit `Drive.clearFault()` + stale comment are removed. Host test proves the cascade.
- [ ] **#146** — A line lost during `LINE_FOLLOW` outside a deliberate dead-reckon window (`_passThrough`/`_projPassThrough` clear) raises `Fault::LINE_LOST` after `LINE_LOST_FAULT_MS` of continuous loss; a pass-through / overshoot hop does NOT fault; a transient loss that reacquires before the window resets the timer. Host tests prove fault, suppression, and reacquire-reset.
- [ ] **#149** — After `setReverseActive` flips the active board, same-tick `getError`/`isOnline`/`getJunctionType` reflect the **new** board. Verified by `arduino-cli` compile + a specified bench procedure (distinct front/rear line patterns, cache enabled, one flip, expected outputs).
- [ ] **#150** — A sync Drive verb issued while faulted returns immediately, preserving the original `_fault.kind` (no `DRAIN_TIMEOUT` clobber, no full-budget spin). Host test proves early-out + kind preserved.
- [ ] **#153** — A turn/hold dispatched at an IMU-read failure uses `_heldYaw`, not `0.0`, in `_execTurnDeg`, `_execTurnToYaw`, `holdYaw`, AND `getYaw`. Host test proves the fallback.
- [ ] **#154-#16** — `_updateWallStop` SQUARE→PUSH aborts (`_endWallStop(true)`) if a fault is latched during square-up. Host test proves it.
- [ ] **#152** — `static_assert`s guard the junction knobs (`LEFT_HI ≤ 7`, `RIGHT_LO ∈ [1,7]`, `T_MIN_ON ∈ [1,8]`) — with `#ifndef *_DEFAULT` override hooks added so a compile-fail probe can trip them — and a correct turn-ladder invariant (`0 < TURN_SLOW_SPEED_PWM ≤ TURN_CRUISE_PWM ≤ PWM_MAX`, `TURN_CRUISE_PWM ≥ PWM_MIN_MAGNITUDE`, `TURN_TOLERANCE_DEG < TURN_SLOW_ZONE_DEG`). The two stale comments are corrected; `MISSION_ALLOW_REARM` docs (×3) tell the truth; a boot `[WARN]` banner prints when it is `true`. Firmware still compiles; a per-boundary compile-fail probe proves each new assert fires.
- [ ] **#154-#18** — `Vision::getRawHue` returns a named `RAW_HUE_NONE` (`0xFF`) for no-data/bad-cell, never colliding with a genuine hue; `Vision.h` + the ADR-028-citing comment are updated. Host test asserts the sentinel is distinct from hue 0.
- [ ] **#154-#19** — An editor open→save round-trip preserves an explicitly-authored `pivot_turns: []` or `blind_projection_auto_seek: true` (default-valued but present). Test round-trips a field that authors both and asserts byte-identity of those keys.
- [ ] **#147** — sim + compiler gate at `NAV_TURN_TOLERANCE_DEG`, one logical-heading rule applied to `_emit_turn_to`/`compile_path`/`final_heading_after`; parametric parity test pins every mirrored `runtime.py` constant; single-hop and multi-hop-cumulative near-collinear fixtures pass; TC12/TC18 byte-parity preserved (or any golden shift surfaced/justified).
- [ ] **#148** — Closed on GitHub as obsolete (superseded by ADR-064/#158) with a comment explaining the premise no longer holds. No code change (see Decisions).
- [ ] **#151** — The validator errors loudly when two swapped-pair `EDGES` entries would clobber each other's `heading_deg`, naming both entries; tests cover both an explicit-vs-explicit and an explicit-vs-derived conflict (wrap-aware), and consistent pairs pass.
- [ ] **#157** — CTPS renders `ACT_RETURN_FORWARD` and `ACT_RETURN_STOP_AT_LINE` at `pipeline_cfg.speed` (matching firmware `_dispatchReturnMoveAfterTurn`) and keeps `ACT_WALL_DEPART` at NORMAL (matching firmware). A render test over a non-NORMAL `pipeline_speed` covers all three tags.
- [ ] **#159** — The sim pass-through entry seed derives from the inbound line hop's resolved tier, then clamps to this hop's cruise. Tests cover FAST→FAST (seed 90), FAST→SLOW (clamp down), and SLOW→FAST.
- [ ] **#154-#17** — An authored fractional facing is quantized identically in sim and firmware. Tests cover a rounding tie (`x.5`) and a `359.7° → 0°` wrap.
- [ ] **#154-#20** — On any invalid mosaic cell, `trace_loader` warns once and sets `trace.mosaic = None` (whole block rejected); valid blocks (colors + `AIR_NAME`) pass. Both doc surfaces (`mosaic.py`, `trace_loader.py`) are corrected. Test asserts the None-on-invalid behavior.

## Contracts & Interfaces

Every fix owns its files/symbols; the true cross-task seams are the new symbols below and the shared-file lanes (**File ownership**).

### New symbols (each owned by one task)

- **`EncoderSourceClass` / `EncoderSource`** (T5, new `WRO2026/src/util/EncoderSource.{h,cpp}`) — sole owner of lower-MCU encoder + task-done reads, mirroring `YawSource`:
  - `bool readDegrees(uint8_t motorId, int32_t& out)` — surfacing; retry-once; leaves `out` untouched on double failure. For fresh baselines/arm/calibration.
  - `int32_t readDegreesOr(uint8_t motorId, int32_t fallback)` — returns last-good on double failure. For continuous completion checks.
  - `void resetCache(uint8_t motorId)` — invalidate last-good after a `resetCounter()`.
  - `bool isPrevTaskDone()` — true only on `Drive_RESULT::OK` done-bit (drive instance `DRIVE_INSTANCE_ID`).
  - `extern EncoderSourceClass EncoderSource;`. T5 is the only producer and only consumer (it routes all sites).
- **`Fault::LINE_LOST = 10`** (T7) — new value in the `PathTracker` `Fault` enum (`PathTracker.h`; current max is 9), with a mnemonic row in `PathTracker.cpp`'s table. Owned by T7.
- **`Config::LINE_LOST_FAULT_MS = 500`** (T7) — `ConfigNav.h` (kept out of `ConfigLine.h`/`ConfigChecks.h` to stay clear of the T15 config lane). Owned by T7.
- **`Config::VISION_RAW_HUE_NONE = 0xFF`** (T14) — named sentinel replacing the literal in `getRawHue` (`ConfigVision.h` or a Vision-local constant). Owned by T14.
- **`DRIVE_INSTANCE_ID`** (T5) — named constant for the lower-MCU drive instance passed to the task-done API. Owned by T5.
- **ADR-066 / ADR-067** — T5 writes `docs/decisions/066-*.md`, T6 writes `docs/decisions/067-*.md` (distinct files, no conflict). The shared `DECISIONS.md` index/status-table edit is owned solely by **T19**.

### Existing symbols consumed read-only (not new seams)

- `speedTierToPreset`, `presetToPwm`, `Routes::PIPELINE_CONFIG.speed`, `PipelineConfig.from_field_data`, `resolve_blind_mode`, `resolve_stop_at_line`, `LineFollower.getPreset()`, `MiniR4.M*.getDegrees`, `mmL`/`Get_Drive_isTaskDone`, PathTracker `_passThrough`/`_projPassThrough`/`isLineLost` — all pre-existing.

### File ownership (the lanes — same file ⇒ dependency, never parallel)

| File | Owner chain (in order) |
|------|------------------------|
| `WRO2026/src/drive/Drive.cpp` | T1 (#142) → T2 (#150) → T3 (#153) → T4 (#154-16) → T5 (#141) |
| `WRO2026/src/mission/PathTracker.cpp` | T6 (#145) → T7 (#146) |
| `tools/path_planner/sim/runtime.py` | T8 (#147) → T9 (#159) → T10 (#154-17) |
| `tools/path_planner/validator.py` | T12 (#151) only (#148 closed — no task) |
| `WRO2026/src/util/EncoderSource.{h,cpp}` (new), `WRO2026/src/drive/LineFollower.cpp`, `WRO2026/src/manip/ManipBelt.h`, `ARCHITECTURE.md` | T5 only |
| `WRO2026/src/mission/Mission.cpp` | T6 only |
| `WRO2026/src/perception/WROLineArray.{cpp,h}` | T13 (#149) only |
| `WRO2026/src/perception/Vision.{cpp,h}` | T14 (#154-18) only |
| config headers + `WRO2026.ino` + `OPERATOR_GUIDE.md` | T15 (#152) only |
| `tools/path_planner/compiler.py` | T8 (#147) only |
| `tools/path_planner/ctps/track_builder.py` | T16 (#157) only |
| `tools/path_planner/ctps/{trace_loader.py,mosaic.py}` | T17 (#154-20) only |
| `tools/path_planner/editor/model.py` | T18 (#154-19) only |
| `docs/decisions/066-*.md` | T5 · `docs/decisions/067-*.md` | T6 |
| **`DECISIONS.md`** (index + status table) | **T19 only** (blocked by T5, T6) |
| **`WRO2026/test/CMakeLists.txt`** (shared, append-only registration) | T1–T7, T15 each append a self-contained `add_executable`/`add_test` block; the execute orchestrator applies these appends **serially** (distinct end-of-file blocks, never the same hunk). See coordination note. |

`ConfigNav.h` is touched only by T7 (`LINE_LOST_FAULT_MS`); the T15 config lane touches disjoint headers, so T7 and T15 run in parallel.

**CMakeLists.txt coordination:** the only cross-lane shared file besides `DECISIONS.md` is the test registration in `WRO2026/test/CMakeLists.txt`. Each test-adding task appends its own target block at the end; because the appends are disjoint and additive, the orchestrator serializes them at commit time. If the execute layer cannot serialize appends to one worktree, register all new targets in a final T19-adjacent step instead. Name every test file/target explicitly in each task (no `test/…` placeholders).

## Error Handling

- **EncoderSource (#141):** continuous reads return last-good on double failure (no fault). Baseline/arm/calibration reads that fail do **not** arm on stale data — the caller retries/skips (a bad baseline would corrupt every later delta). Cache is invalidated on every `resetCounter`. A persistently dead link still surfaces via the existing move/stall/drain watchdogs.
- **Lost-line fault (#146):** fires only in `LINE_FOLLOW`, outside `_passThrough`/`_projPassThrough`, after `LINE_LOST_FAULT_MS` of continuous loss; the timer resets on reacquire and clears on state-exit/abort/fault. Not polled in `LINE_OVERSHOOT` (that state is a deliberate dead-reckon past the line).
- **Drain while faulted (#150):** early-out preserves the original `_fault.kind`.
- **wallStop fault during square-up (#154-16):** `_endWallStop(true)` and return.
- **Config static_asserts (#152):** a violated invariant is a compile error. The turn-ladder assert must express the true invariant (`TURN_SLOW_SPEED_PWM` is intentionally below the deadband floor — see Notes) or it breaks the build.
- **Mosaic loader (#154-20):** any invalid cell → warn once, whole block `None` (never raise, never a partial 11-cell mosaic).

## Testing Strategy

**Levels:** host C++ ctest (firmware — Drive/PathTracker/LineFollower are host-compiled), pytest (Python), `arduino-cli` compile (all firmware), specified bench procedure (#149 only).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | #141: continuous read mocked to time out twice | Unit (host) | `readDegreesOr` returns caller's last-good; a later success-`0` returns `0` |
| TC1b | #141: baseline/arm read mocked to fail | Unit (host) | `readDegrees` returns false, `out` untouched; caller does NOT arm on stale data |
| TC1c | #141: `resetCache` after a `resetCounter` | Unit (host) | next `readDegreesOr` cannot serve the pre-reset value |
| TC2 | #141: task-done read returns non-`Drive_RESULT::OK` | Unit (host) | `isPrevTaskDone()` false; `_dispatchNext` does not fire. (Mock adds both `GetEncoderDegrees` and the task-done API.) |
| TC3 | #142: SLOW straddle (+1.5°→−1.5°); CRUISE-to-opposite-side jump; ±180° wrap crossing | Unit (host, Drive) | BRAKE on the straddle tick in every active state; no extra revolution |
| TC4 | #145: latch fault, BTN_DOWN, re-`goTo` | Integration (host) | `Drive._fault` cleared; no `DRIVE_FAULT` on next dispatch |
| TC5 | #146: line lost mid-`LINE_FOLLOW` > `LINE_LOST_FAULT_MS`; transient loss that reacquires; loss cleared across abort | Integration (host) | fault on sustained loss; timer resets on reacquire; timer cleared on abort |
| TC6 | #146: "lost" during a pass-through/overshoot hop | Integration (host) | no fault (suppressed) |
| TC7 | #150: sync verb enqueued while faulted | Unit (host) | `_drainBlocking` returns immediately; `_fault.kind` unchanged |
| TC8 | #153: dispatch turn/hold/getYaw with a mocked IMU read failure | Unit (host) | target from `_heldYaw`, not `0.0` |
| TC9 | #154-16: fault latched during SQUARE | Unit (host) | `_endWallStop(true)`; PUSH never runs |
| TC10 | #152: build with each junction/turn-ladder boundary out of range (via added `*_DEFAULT` hooks) | Compile-fail probe | each assert message fires (`PASS_REGULAR_EXPRESSION`) |
| TC11 | #154-18: `getRawHue` no-read / bad cell | Unit (host) | returns `RAW_HUE_NONE`, distinct from hue 0 |
| TC11b | #154-19: round-trip a field authoring `pivot_turns: []` + `blind_projection_auto_seek: true` | Unit (pytest) | both keys survive open→save byte-identically |
| TC12 | #147: parametric parity — each mirrored `runtime.py` constant vs its header | Unit (pytest) | every mirrored constant equals its parsed header value |
| TC13 | #147: single near-collinear hop (δ ∈ (1e-6, 0.5°]) AND a multi-hop chain whose per-hop δ ≤ 0.5° but cumulative > 0.5° | Unit (pytest) | sim + compiler emit no spurious `ACT_TURN_REL`; heading advanced consistently; no cumulative compiler-only turn |
| TC16 | #151: swapped-pair with explicit-vs-explicit and explicit-vs-derived conflicting `heading_deg` | Unit (pytest) | validator errors (wrap-aware), names both entries; consistent pairs pass |
| TC17 | #157: render `pipeline_speed: FAST` trace exercising `ACT_RETURN_FORWARD`, `ACT_RETURN_STOP_AT_LINE`, `ACT_WALL_DEPART` | Unit (pytest) | both return tags render at 90; wall-depart stays at NORMAL |
| TC18 | #159: FAST→FAST (seed 90); FAST→SLOW (clamp to 60); SLOW→FAST | Unit (pytest) | seed = inbound tier clamped to hop cruise |
| TC19 | #154-17: facing `45.5°` (tie) and `359.7°` (wrap) | Unit (pytest) | sim rounds to match emitted int16 (46, 0) |
| TC20 | #154-20: mosaic block with one unknown-color cell | Unit (pytest) | one warning; `trace.mosaic is None` (whole block rejected) |

**Test data:** add `GetEncoderDegrees` **and** the task-done API to `test/hal_mocks/MatrixMiniR4.h` (and any mock surface that compiles `ManipBelt.h`/`LineFollower.cpp` — see T5 note); new near-collinear single + multi-hop fixtures; a `pipeline_speed: FAST` fixture (one owner — created by T8, consumed by T9/T16 via distinct filenames if needed); existing `routes_toy`/pivot-seek fixtures.
**Run commands:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure` · `uv run --extra dev pytest tools/path_planner/tests/` · `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` · final: `uv run python tools/path_planner/build_routes.py && git diff --exit-code -- WRO2026/src/nav/routes.h WRO2026/src/nav/routes.cpp`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | #142 turn passed-target | — | high | `src/drive/Drive.cpp`, `test/CMakeLists.txt`(+block), `test/…` | In `_updateTurnProfile` (~:1102), before the band checks, in **any** active turn state: if `turnSign(rawDelta) != _turnDeltaSign`, go straight to BRAKE. Add TC3 (SLOW + CRUISE + wrap). High risk = motor-safety stop condition. Satisfies AC #142. |
| T2 | #150 drain fault early-out | T1 | low | `src/drive/Drive.cpp`, `test/…` | Top of `_drainBlocking` loop (~:1817): `if (hasFault()) { _clearQueue(); return; }` — preserve original kind. TC7. Satisfies AC #150. |
| T3 | #153 fabricated yaw | T2 | low | `src/drive/Drive.cpp`, `test/…` | Replace `readYawDegOr(0.0)` with `readYawDegOr(_heldYaw)` in `_execTurnDeg` (:353), `_execTurnToYaw` (:413), `holdYaw` (:515-519), AND `getYaw` (:512). TC8. Satisfies AC #153. |
| T4 | #154-16 wallStop guard | T3 | low | `src/drive/Drive.cpp`, `test/…` | Top of `_updateWallStop` SQUARE branch (~:1397): `if (hasFault()) { _endWallStop(true); return; }`. TC9. Satisfies AC #154-16. |
| T5 | #141 EncoderSource | T4 | high | `src/util/EncoderSource.{h,cpp}` (new), `src/drive/Drive.cpp`, `src/drive/LineFollower.cpp`, `src/manip/ManipBelt.h`, `ARCHITECTURE.md`, `docs/decisions/066-*.md`, `test/hal_mocks/MatrixMiniR4.h` (+ other mock surfaces), `test/CMakeLists.txt`, `test/…` | Create `EncoderSourceClass`/`EncoderSource` (util) per Decisions: centralized per-motor cache, `readDegrees`(surfacing)/`readDegreesOr`(last-good)/`resetCache`/`isPrevTaskDone`(`Drive_RESULT::OK`, `DRIVE_INSTANCE_ID`). Route sites by policy: continuous checks → `readDegreesOr(last-good)` (Drive.cpp `_updateProfile` :721, `_updateStopAtLine` :1201, LineFollower distance gate :531-532 + ramp-down :279-280, ManipBelt `beltTo` :194); baseline/arm/calibration → `readDegrees` surfacing (Drive.cpp arm seeds :1039/:1711, wallStop SQUARE seed :1408, `calibrateMotorBias` :559-560); `resetCache` at every `resetCounter` (Drive.cpp :506/:528, ManipBelt `_resetCounter` :566+); task-done via `isPrevTaskDone()` (:120); `getEncL/R` (:496/:501). Add `GetEncoderDegrees` + the task-done API to the HAL mock and every mock surface that compiles `ManipBelt.h`/`LineFollower.cpp` (hal_mocks, mocks, ctps_mocks as needed); gate on FULL ctest. TC1/TC1b/TC1c/TC2. Write ADR-066 (DECISIONS.md index is T19). Update ARCHITECTURE.md module map + dependency graph. Satisfies AC #141. |
| T6 | #145 fault-clear cascade | — | high | `src/mission/PathTracker.cpp`, `src/mission/Mission.cpp`, `docs/decisions/067-*.md`, `test/CMakeLists.txt`, `test/…` | `PathTracker::clearFault()` (:490) + `PathTracker::abort()` (:499) call `Drive.clearFault()`; add it to `Mission::abort()` (:198-223). Remove the now-redundant explicit `Drive.clearFault()` in `declarePosition` (:272) and fix its stale comment (:266-271). Write ADR-067 (index is T19). TC4. Satisfies AC #145. |
| T7 | #146 lost-line fault | T6 | high | `src/mission/PathTracker.cpp`, `src/mission/PathTracker.h`, `config/ConfigNav.h`, `docs/decisions/009-*.md` (status note), `test/CMakeLists.txt`, `test/…` | Add `Fault::LINE_LOST = 10` + mnemonic; `Config::LINE_LOST_FAULT_MS = 500` (`// TODO`) in ConfigNav.h. Poll `LineFollower.isLineLost()` in `LINE_FOLLOW` (:932) ONLY (not LINE_OVERSHOOT); suppress via PathTracker-owned `_passThrough`/`_projPassThrough` (do NOT read `LineFollower._overshootMode` — private, cross-lane); fault after `LINE_LOST_FAULT_MS` continuous loss; reset timer on reacquire; clear on state-exit/abort/fault; reuse `_enterFault` (:1411). Note in ADR-009 this is fault-escalation, not the deferred sweep/reacquire recovery. TC5/TC6. Satisfies AC #146. |
| T8 | #147 turn-gate parity | — | high | `tools/path_planner/sim/runtime.py`, `tools/path_planner/compiler.py`, `tools/path_planner/tests/…` (parity test + fixtures) | Change the sim `_emit_turn_to` gate (:2541) and compiler gates (:202/:369) from `1e-6` to `NAV_TURN_TOLERANCE_DEG`. Apply ONE logical-heading rule (advance heading to the edge when a sub-tolerance turn is skipped) in `_emit_turn_to`, `compile_path`, AND `final_heading_after`, so cumulative sub-tolerance hops never become a compiler-only turn. Leave `runtime.py`'s second `TURN_EPSILON_DEG` use (:2513) at `1e-6`. Fix the misleading `_emit_turn_to` docstring. Parametric parity test over every mirrored constant (via `config_parse`). Single + multi-hop-cumulative fixtures (TC12/TC13). Keep TC12/TC18 byte-parity; flag any golden shift. Satisfies AC #147. |
| T9 | #159 pass-through seed | T8 | med | `tools/path_planner/sim/runtime.py`, `tools/path_planner/tests/…` | Carry the inbound line hop's resolved tier through `_walk_hops` alongside `pass_through_active`; seed `entry_mag = min(inbound_tier_pwm, cruise_mag)` (:3479-3483). TC18 (FAST→FAST, FAST→SLOW, SLOW→FAST). Update the known-limitation comment (:3462-3478). Satisfies AC #159. |
| T10 | #154-17 facing round | T9 | low | `tools/path_planner/sim/runtime.py`, `tools/path_planner/tests/…` | In `_dispatch_wall_reset` (:2711)/`LocalizationSim.reanchor_at_wall`, quantize facing with `int(round(x)) % 360` (matches header_writer.py:884). TC19 (tie + wrap). Satisfies AC #154-17. |
| T12 | #151 swapped-pair guard | — | low | `tools/path_planner/validator.py`, `tools/path_planner/tests/…` | In `_validate_edges_field` (:888), error when two swapped-pair `EDGES` entries would clobber each other's `heading_deg` (wrap-aware: explicit heading vs the `_reverse_heading` the pair would write). Name both entries. TC16 (explicit-vs-explicit + explicit-vs-derived). Satisfies AC #151. |
| T13 | #149 line-board cache | — | med | `src/perception/WROLineArray.{cpp,h}` | In `setReverseActive` (:244-254), when `_activeBoard` changes and `LINE_ARRAY_TICK_CACHE_ENABLED`, call `_refreshCache()`. `arduino-cli` compile; add the specified bench procedure to the PR (not host-compiled). Satisfies AC #149. |
| T14 | #154-18 Vision sentinel | — | low | `src/perception/Vision.{cpp,h}`, `config/ConfigVision.h` (const), `test/test_vision.cpp` | Add `VISION_RAW_HUE_NONE = 0xFF`; `getRawHue` (:539) returns it for `!_hasRead`/OOB; update Vision.h doc + the ADR-028-citing comment. TC11. Satisfies AC #154-18. |
| T15 | #152 config hardening | — | med | `config/ConfigChecks.h`, `config/ConfigLine.h`, `config/ConfigDrive.h`, `config/ConfigMission.h`, `config/ConfigManipulator.h`, `WRO2026.ino`, `docs/OPERATOR_GUIDE.md`, `test/CMakeLists.txt`, `test/…` | Add `#ifndef *_DEFAULT` override hooks to the junction knobs, then junction asserts (`LEFT_HI ≤ 7`, `RIGHT_LO ∈ [1,7]`, `T_MIN_ON ∈ [1,8]`) + the correct turn-ladder invariant (`0 < TURN_SLOW_SPEED_PWM ≤ TURN_CRUISE_PWM ≤ PWM_MAX`, `TURN_CRUISE_PWM ≥ PWM_MIN_MAGNITUDE`, `TURN_TOLERANCE_DEG < TURN_SLOW_ZONE_DEG`) in ConfigChecks.h. Fix stale comments (ConfigDrive.h:484-486, ConfigLine.h:255-256). Rewrite ConfigMission.h:35/43-47 + OPERATOR_GUIDE.md:668-670 (ships `true` for bench, flip before comp). Add a self-silencing boot `[WARN]` banner in setup() after Serial.begin (:181). Optional: servo-drain assert via a constexpr reducer over the servo structs. Per-boundary compile-fail probe (TC10). Do NOT change the `MISSION_ALLOW_REARM` value. Satisfies AC #152. |
| T16 | #157 CTPS RETURN speed | — | low | `tools/path_planner/ctps/track_builder.py`, `tools/path_planner/tests/…` | Render `ACT_RETURN_FORWARD` **and** `ACT_RETURN_STOP_AT_LINE` at `_blind_cruise_pwm(cfg, pipeline_cfg.speed)` (thread the `pipeline_cfg` resolved in `build()` :1338 into `_primitive_drive_speed`); leave `ACT_WALL_DEPART` at NORMAL (mirrors firmware). Update the `_PER_EDGE_SPEED_TAGS` doc block (:508-512). TC17 (all three tags). Satisfies AC #157. |
| T17 | #154-20 mosaic whitelist | — | low | `tools/path_planner/ctps/{trace_loader.py,mosaic.py}`, `tools/path_planner/tests/…` | In `_parse_mosaic` (:624-636), validate each cell against `mosaic.COLOR_NAMES ∪ {mosaic.AIR_NAME}`; on any invalid cell warn once and set the whole block to `None` (preserve the 12-cell contract). Correct BOTH docstrings (mosaic.py:16-17, trace_loader.py:235-238). TC20. Satisfies AC #154-20. |
| T18 | #154-19 editor round-trip | — | low | `tools/path_planner/editor/model.py`, `tools/path_planner/tests/…` | In `to_json_dict` (:529-540), preserve an explicitly-authored `pivot_turns: []` / `blind_projection_auto_seek: true` on save (track source-presence in `from_json_dict`, or always emit the keys). TC11b. Satisfies AC #154-19. |
| T19 | ADR index update | T5, T6 | low | `DECISIONS.md` | Add the ADR-066 and ADR-067 index lines (correct sections) and bump the status-summary table. Sole owner of `DECISIONS.md` so T5/T6 don't collide on it. |

## Notes for Implementer

- **Lane serialization:** three files are shared and their tasks are chained (Drive.cpp T1→T2→T3→T4→T5; PathTracker.cpp T6→T7; sim/runtime.py T8→T9→T10). `validator.py` now has a single task (T12/#151) since #148 is closed without code. The Drive lane deliberately does the **cheap guards first** and the big EncoderSource (#141) **last**, so a stall in the module work never strands the trivial safety fixes. #141 is P1 — it must still land; it is last only for scheduling safety.
- **#141 vendor reach:** the firmware reads `MiniR4.M2.getDegrees()` (bare int32). To surface the RESULT, `EncoderSource` calls the vendor two-channel `mmL.GetEncoderDegrees(id, degs)`; the task-done bit is `MMLower::Get_Drive_isTaskDone(uint8_t, bool*)` → `Drive_RESULT`. The vendor `MatrixMiniR4` source IS present locally (arduino-cli resolves it; Codex read `MMLower.h`) — confirm `mmL` reachability from `src/util/` and mirror the exact signatures in the mock.
- **#141 last-good is safe ONLY for continuous checks.** A failed baseline/arm/calibration read must NOT arm on stale data (a bad `_profileStartEnc`/`_stopStartEnc` corrupts every later delta), and the cache must be invalidated after `resetCounter` (else a post-reset read serves the pre-reset value). This split is why there are two read forms.
- **#152 turn-ladder assert (do NOT paste the issue's literal assert):** `TURN_CRUISE_PWM=50`, `TURN_SLOW_SPEED_PWM=20`, `PWM_MIN_MAGNITUDE=35`. `TURN_SLOW_SPEED_PWM` is intentionally a below-floor setpoint that `deadbandSnap` raises to `PWM_MIN_MAGNITUDE` at emit. Assert `0 < TURN_SLOW_SPEED_PWM ≤ TURN_CRUISE_PWM ≤ PWM_MAX`, `TURN_CRUISE_PWM ≥ PWM_MIN_MAGNITUDE`, `TURN_TOLERANCE_DEG < TURN_SLOW_ZONE_DEG` — NOT `TURN_SLOW_SPEED_PWM ≥ PWM_MIN_MAGNITUDE` (false, breaks the build). Fix the "set equal to PWM_MIN_MAGNITUDE" comment to describe the snap-up.
- **#152 compile-fail probe** needs `#ifndef *_DEFAULT` override hooks on the junction knobs (they are plain `constexpr` today) — add them following the existing `IMU_YAW_SIGN`/`HOP_SETTLE_MS` probe pattern, else `-D..._DEFAULT` is a no-op.
- **#146 poll site:** only `LINE_FOLLOW`. `LINE_OVERSHOOT` unconditionally arms overshoot mode (PathTracker :1057), so a "lost line" there is the expected dead-reckon — polling it would be incoherent. Use PathTracker-owned `_passThrough`/`_projPassThrough` for suppression; never reach into `LineFollower`'s private `_overshootMode`.
- **#148 (no task — closed as obsolete):** the task table intentionally has no T11; #148 is resolved by closing the GitHub issue as superseded by ADR-064/#158 (`gh issue close 148 --comment "..."`), posted as a batch step. T12 (#151) is unblocked and is the sole `validator.py` task.
- **#157**: firmware `WALL_DEPART` (PathTracker.cpp ~:1347) drives at the Drive NORMAL default (no `.setPower`); the two return moves (:1611-1616 seek, :1628-1633 dead-reckon) use `PIPELINE_CONFIG.speed`. Mirror that exactly.
- **No routes regen:** `build_routes.py` never imports `compiler.py`; the final freshness gate proves `routes.h/.cpp` are unchanged. If a diff appears, stop and re-scope.
- **Rollback:** every task is one commit; revert the offending commit. The two ADRs are additive docs.
- **Regional is 2026-07-21** — the P1 firmware lanes (Drive lane + PathTracker lane) carry the runtime-relevant fixes; the planner/sim/config work is latent (no current-field runtime effect).
