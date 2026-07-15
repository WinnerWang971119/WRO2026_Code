# Issues #141–159 hardening batch — Plan

**Goal:** Fix the 14 open, actionable issues in the range #141–159 — a firmware/perception/planner/sim/config hardening sweep that removes several latent-but-real robustness hazards ahead of the 2026-07-21 regional — on one worktree branch with one commit per issue.

**Approach:** Each issue has a settled fix direction (verified against current source by four code-mapping passes). Group the work by the file each fix touches so parallel executors never collide: five lanes that run concurrently, each internally serialized where fixes share a file (`Drive.cpp`, `PathTracker.cpp`, `sim/runtime.py`, `validator.py`). Two fixes establish a new cross-module contract and get an ADR (#141 → ADR-066 `EncoderSource`, mirroring ADR-045 `YawSource`; #145 → ADR-067 fault-clear cascade). No fix changes the emitter or `field_data`, so `routes.h/.cpp` stay byte-identical (no regen, CI freshness gate untouched).

## Scope

- **In scope (14 issues):**
  - **Firmware/nav:** #141 (EncoderSource: timeout-as-zero), #142 (turn passed-target guard), #145 (fault-clear cascade), #146 (lost-line fault), #149 (line-board stale-cache on flip), #150 (drain ignores latched fault), #153 (fabricated 0.0 yaw on turn dispatch), #154-#16 (wallStop SQUARE fault guard).
  - **Config:** #152 (junction-knob + turn-ladder static_asserts, stale comments, honest `MISSION_ALLOW_REARM` docs + boot banner, optional servo-drain assert).
  - **Vision:** #154-#18 (`getRawHue` sentinel `0` → `0xFF`).
  - **Planner/sim/ctps:** #147 (turn-gate parity — full fix), #148 (validator PIPELINE_* resolution), #151 (swapped-pair `EDGES` heading clobber guard), #157 (CTPS RETURN speed), #159 (sim pass-through entry-seed tier), #154-#17 (ResetNode facing — round in sim), #154-#20 (mosaic loader whitelist).
- **Out of scope (decided in planning):**
  - **#155** (extract Drive sub-FSMs to `*Policy.h`) — deferred; its motivation ("Drive.cpp is not host-compiled") is already partly obsolete (Drive.cpp *is* host-compiled), and #142's guard lands with a direct Drive host test instead.
  - **#156** (LEFT/RIGHT lateral offset-follow) — genuinely-unbuilt deferred feature; no current mission needs it.
  - **#143, #144** (merged PRs) and **#158** (the open per-edge-speed PR = the base branch itself) — not fix targets.
  - **#141 plausibility gate** (rejecting corrupted-but-complete UART frames) — the issue explicitly scopes this out; this batch fixes only the timeout-treated-as-valid hazard.
  - Flipping `MISSION_ALLOW_REARM` to `false` — a deliberate separate pre-competition action; this batch keeps the bench `true` value and only corrects the docs + adds a boot banner.

## Decisions

- **Base branch:** worktree `fix/issues-141-159-hardening` is cut from `feat/per-edge-mode-speed` (not `main`), because #157/#159/#148 (and the SpeedTier part of #147) patch code that only exists on that branch (#158/ADR-064). Rebase onto `main` after #158 merges.
- **Worktree:** all work happens in `/Users/diego/Documents/WRO2026/wt-fix-141-159`.
- **PR shape:** one PR off the branch, one conventional commit per issue, so review can go issue-by-issue.
- **#141 fallback:** on a double lower-MCU read failure, `EncoderSource` returns the **last-good** value and raises **no** new fault (mirrors `YawSource`). A genuine `0` with `RESULT::OK` is always kept.
- **#146 grace window:** new `Config::LINE_LOST_FAULT_MS = 500` (placeholder, `// TODO bench-tune`), suppressed during any deliberate overshoot/dead-reckon window.
- **#147:** full fix — align the sim+compiler turn-emission gate to `NAV_TURN_TOLERANCE_DEG`, advance sim heading on pass-through so no micro-turn accumulates, add a parametric parity test over every mirrored constant, add a near-collinear fixture. Pin live-vs-latent first; flag any committed-golden shift.
- **#152 `MISSION_ALLOW_REARM`:** keep the value `true`; rewrite the three docs to state the truth (ships `true` for bench, MUST flip `false` before competition) and add a self-silencing boot `[WARN]` banner.
- **#154-#17 facing:** round facing identically in the sim/`LocalizationSim` to mirror the int16 firmware; no struct/emitter/routes change.
- **New ADRs:** ADR-066 (`EncoderSource`), ADR-067 (Drive fault-clear cascade). #146 implements ADR-009's already-deferred lost-line recovery branch (status note only, no new ADR).
- **No routes regen:** confirmed no fix touches `header_writer.py`/`field_data.json`, so `routes.h/.cpp` are unchanged.

## Acceptance Criteria

- [ ] **#141** — A lower-MCU encoder/task-done read that times out no longer reads as `0`/`true`: reads route through `EncoderSource` (retry-once, last-good on double failure); the task-done bit treats any non-OK `Drive_RESULT` as "not done". A genuine `RESULT::OK` `0` is preserved. Host test proves last-good is served on a mocked timeout.
- [ ] **#142** — An in-flight turn whose heading crosses the tolerance band between two ticks brakes on the next tick (worst-case one tick of overshoot), not after a full extra revolution. Host test drives a straddle case.
- [ ] **#145** — After any latched Drive fault, the operator's BTN_DOWN reset (and `Mission::abort()` / `PathTracker::abort()`) clears `Drive._fault`; the next `goTo()` does not re-trip `DRIVE_FAULT`. Host test proves the cascade.
- [ ] **#146** — A line lost during `LINE_FOLLOW`/`LINE_OVERSHOOT` outside a deliberate dead-reckon window raises `Fault::LINE_LOST` after `LINE_LOST_FAULT_MS`, giving the standard beep/OLED; a deliberate overshoot hop does **not** fault. Host test proves both.
- [ ] **#149** — After `setReverseActive` flips the active board, same-tick reads (`getError`/`isOnline`/`getJunctionType`) reflect the **new** board, not the previous board's cached sample. Verified by `arduino-cli` compile + a bench note (WROLineArray is not host-compiled).
- [ ] **#150** — A sync Drive verb issued while `Drive._fault` is latched returns immediately, preserving the original fault kind (no `DRAIN_TIMEOUT` clobber, no full-budget spin). Host test proves early-out + kind preserved.
- [ ] **#153** — A turn/hold dispatched at the instant an IMU read fails uses `_heldYaw` (last-known target), not a fabricated `0.0`. Host test proves the fallback.
- [ ] **#154-#16** — `_updateWallStop` SQUARE→PUSH aborts (`_endWallStop(true)`) if a fault is latched during square-up, instead of pushing at an un-squared heading. Host test proves it.
- [ ] **#152** — `static_assert`s guard the junction knobs (`LEFT_HI ≤ 7`, `RIGHT_LO ∈ [1,7]`, `T_MIN_ON ∈ [1,8]`) and a correct turn-ladder invariant; the two stale comments (`ConfigDrive.h`, `ConfigLine.h`) are corrected; `MISSION_ALLOW_REARM` docs (×3) tell the truth and a boot `[WARN]` banner prints when it is `true`. The firmware still compiles; a compile-fail probe proves at least one new assert fires.
- [ ] **#154-#18** — `Vision::getRawHue` returns `0xFF` (device NONE) for no-data/bad-cell, never colliding with a genuine red hue; `Vision.h` documents it. Host test asserts the sentinel.
- [ ] **#147** — The sim (`_emit_turn_to`) and compiler turn-emission gate at `NAV_TURN_TOLERANCE_DEG` (0.5°), with sim heading advanced on pass-through so no residual micro-turn accumulates; a parametric parity test pins every mirrored `runtime.py` constant to its header; a near-collinear (delta ∈ (1e-6, 0.5°]) fixture exercises the band. TC12/TC18 byte-parity preserved (or any golden shift surfaced and justified).
- [ ] **#148** — `_validate_pivot_turns` resolves the incoming-leg seek regime with the edge's actual `mode` + a field-sourced `PipelineConfig.from_field_data(field_data)`, matching the compiler/runtime. A `mode: STOP_AT_LINE`-equivalent blind-seek pivot leg and a field-set `PIPELINE_BLIND_MODE` case are covered by tests.
- [ ] **#151** — The validator errors loudly when two swapped-pair `EDGES` entries would clobber each other's `heading_deg` geometry, naming both entries; a test proves the rejection and that consistent pairs still pass.
- [ ] **#157** — CTPS renders `ACT_RETURN_FORWARD`/`ACT_WALL_DEPART` at the field-sourced `pipeline_cfg.speed`, matching the firmware. A render test over a non-NORMAL `pipeline_speed` proves it.
- [ ] **#159** — The sim pass-through entry seed derives from the inbound line hop's resolved tier (then clamps to this hop's cruise), so FAST-line-into-FAST-blind seeds at 90, not 70. A parity/render test over a non-NORMAL line-into-blind pass-through proves it.
- [ ] **#154-#17** — An authored fractional reset-node facing is quantized identically in the sim and firmware (no sub-degree sim/firmware divergence). A test asserts the sim rounds to the emitted int16.
- [ ] **#154-#20** — `trace_loader` validates each mosaic cell against `mosaic.COLOR_NAMES ∪ {"UNKNOWN"}` (warn-and-skip otherwise); the `mosaic.py` docstring matches the real behavior. A test feeds an unknown color and asserts the warn-and-skip.

## Contracts & Interfaces

Every fix owns its files and symbols end-to-end; the only true cross-task seams are the new symbols below and the shared-file lanes (pinned in **File ownership**). No task redefines another's symbol.

### New symbols (each owned by one task)

- **`EncoderSource`** (T1, new `WRO2026/src/nav/EncoderSource.h/.cpp`) — sole owner of lower-MCU encoder + task-done reads, mirroring `YawSource`:
  - `bool readDegrees(uint8_t motorId, int32_t& out)` — surfacing form; leaves `out` untouched on failure (retry-once inside), returns false on double failure.
  - `int32_t readDegreesOr(uint8_t motorId, int32_t fallback)` — convenience; returns `fallback` on double failure. Callers pass their **last-good** value as `fallback`.
  - `bool isPrevTaskDone()` — returns true only on a `RESULT::OK` done-bit; any non-OK `Drive_RESULT` → false.
  - Exposed as `extern EncoderSource Encoder;` (ADR-002 singleton pattern). T1 is the only producer and the only consumer (it routes all sites); no other task references it.
- **`Fault::LINE_LOST`** (T7) — new value `= 10` in the `PathTracker` `Fault` enum (`PathTracker.h`), with a mnemonic row in `PathTracker.cpp` (the `:544-553` table). Owned by T7; no other task consumes it.
- **`Config::LINE_LOST_FAULT_MS = 500`** (T7) — new constant in `ConfigNav.h` (nav-FSM timing; kept out of `ConfigLine.h`/`ConfigChecks.h` to avoid the #152 lane). Owned by T7.
- **ADR-066 / ADR-067** — next free numbers (065 is latest). T1 writes 066, T6 writes 067. No renumber risk.

### Existing symbols consumed read-only (not new seams)

- `speedTierToPreset`, `presetToPwm`, `Routes::PIPELINE_CONFIG.speed`, `PipelineConfig.from_field_data`, `resolve_blind_mode`, `resolve_stop_at_line`, `LineFollower.getPreset()`, `mmL.GetEncoderDegrees` — all pre-existing; #147/#148/#157/#159/#141 read them, none redefine them.

### File ownership (the lanes — same file ⇒ dependency, never parallel)

| File | Owner chain (in order) |
|------|------------------------|
| `WRO2026/src/drive/Drive.cpp` | T1 (#141) → T2 (#142) → T3 (#150) → T4 (#153) → T5 (#154-16) |
| `WRO2026/src/mission/PathTracker.cpp` | T6 (#145) → T7 (#146) |
| `tools/path_planner/sim/runtime.py` | T11 (#147) → T12 (#159) → T13 (#154-17) |
| `tools/path_planner/validator.py` | T14 (#148) → T15 (#151) |
| `WRO2026/src/drive/LineFollower.cpp`, `ManipBelt.h` | T1 only |
| `WRO2026/src/mission/Mission.cpp` | T6 only |
| `WRO2026/src/perception/WROLineArray.{cpp,h}` | T8 (#149) only |
| `WRO2026/src/perception/Vision.{cpp,h}` | T9 (#154-18) only |
| config headers + `WRO2026.ino` + `OPERATOR_GUIDE.md` | T10 (#152) only |
| `tools/path_planner/compiler.py` | T11 (#147) only |
| `tools/path_planner/ctps/track_builder.py` | T16 (#157) only |
| `tools/path_planner/ctps/trace_loader.py` + `mosaic.py` | T17 (#154-20) only |

`ConfigNav.h` is touched only by T7 (`LINE_LOST_FAULT_MS`); the #152 lane (T10) touches `ConfigLine.h/ConfigDrive.h/ConfigMission.h/ConfigManipulator.h/ConfigChecks.h/.ino/OPERATOR_GUIDE.md` — disjoint from T7, so T7 and T10 run in parallel.

## Error Handling

- **EncoderSource double failure (#141):** return last-good, no fault. A persistently dead STM32 link still surfaces via the existing move/stall/drain watchdogs — it is not this layer's job to fault.
- **Lost-line fault (#146):** only fires outside a deliberate dead-reckon window (`_overshootMode` / `_passThrough` / `_projPassThrough` all clear) and only after `LINE_LOST_FAULT_MS`; a transient 1–2 tick blip at ~35 Hz does not fault.
- **Drain while faulted (#150):** early-out preserves the original `_fault.kind`; never overwrite with `DRAIN_TIMEOUT`.
- **wallStop fault during square-up (#154-16):** `_endWallStop(true)` (brake) and return; do not advance to PUSH.
- **Config static_asserts (#152):** a violated invariant is a **compile error** (loud, pre-flight), not a runtime surprise. The turn-ladder assert must express a *true* invariant (see Notes) or it breaks the current build.
- **Mosaic loader (#154-20):** unknown cell color → `warnings.warn` + skip that cell, never raise.

## Testing Strategy

**Levels:** host C++ ctest (firmware where compiled), pytest (Python), `arduino-cli` compile (all firmware), bench note (#149 only).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | #141: mock `GetEncoderDegrees` returns timeout twice for a site read | Unit (host) | `readDegreesOr` returns the caller's last-good, not `0`; a subsequent `RESULT::OK` `0` returns `0` |
| TC2 | #141: task-done read returns non-OK `Drive_RESULT` | Unit (host) | `isPrevTaskDone()` returns false; `_dispatchNext` does not fire |
| TC3 | #142: two ticks straddle the tolerance band (+1.5° then −1.5°) | Unit (host, Drive) | turn transitions to BRAKE on the straddle tick; no extra revolution |
| TC4 | #145: latch a Drive fault, press BTN_DOWN, re-`goTo` | Integration (host) | `Drive._fault` cleared by reset; no `DRIVE_FAULT` on next dispatch |
| TC5 | #146: line lost mid-`LINE_FOLLOW`, wait > `LINE_LOST_FAULT_MS` | Integration (host) | `Fault::LINE_LOST` entered; beep/OLED path taken |
| TC6 | #146: line "lost" during a deliberate overshoot hop | Integration (host) | no fault (suppressed) |
| TC7 | #150: sync verb enqueued while faulted | Unit (host) | `_drainBlocking` returns immediately; `_fault.kind` unchanged; queue cleared |
| TC8 | #153: dispatch turn with a mocked IMU read failure | Unit (host) | target derived from `_heldYaw`, not `0.0` |
| TC9 | #154-16: fault latched during SQUARE | Unit (host) | `_endWallStop(true)`; PUSH never runs |
| TC10 | #152: build with an out-of-range junction knob (`-D..._DEFAULT=<bad>`) | Compile-fail probe | matching `static_assert` message fires (`PASS_REGULAR_EXPRESSION`) |
| TC11 | #154-18: `getRawHue` with no read / bad cell | Unit (host) | returns `0xFF`, distinct from a valid hue 0 |
| TC12 | #147: parametric parity — each mirrored `runtime.py` constant vs its header | Unit (pytest) | every mirrored constant equals its parsed header value |
| TC13 | #147: near-collinear hop, delta ∈ (1e-6, 0.5°] | Unit (pytest) | sim + compiler emit **no** `ACT_TURN_REL`; heading advanced; parity holds |
| TC14 | #148: pivot leg = blind seek (`stop_at_line: on`) with θ giving trim > offset | Unit (pytest) | validator classifies "sensor" regime and rejects (matches runtime overshoot) |
| TC15 | #148: field sets `PIPELINE_BLIND_MODE` non-default | Unit (pytest) | validator regime tracks the field value, not the singleton |
| TC16 | #151: swapped-pair entries with conflicting explicit `heading_deg` | Unit (pytest) | validator errors, names both entries; consistent pairs pass |
| TC17 | #157: render a trace with `pipeline_speed: FAST` and a RETURN move | Unit (pytest) | `ACT_RETURN_FORWARD` renders at 90, not 70 |
| TC18 | #159: FAST line into FAST blind pass-through | Unit (pytest) | entry seed = 90 (clamped to hop cruise), not 70 |
| TC19 | #154-17: authored facing 45.7° | Unit (pytest) | sim reads 46 (matches emitted int16), not 45.7 |
| TC20 | #154-20: trace mosaic cell = an unknown string | Unit (pytest) | `warnings.warn` + cell skipped; no KeyError downstream |

**Test data:** existing host harness force-include mocks (add `GetEncoderDegrees` to `test/hal_mocks/MatrixMiniR4.h` for TC1/TC2); a new near-collinear fixture for TC13; a `pipeline_speed: FAST` fixture for TC17/TC18; existing `routes_toy`/pivot-seek fixtures for TC14–16.
**Run commands:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure` · `uv run --extra dev pytest tools/path_planner/tests/` · `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | #141 EncoderSource | — | high | `src/nav/EncoderSource.{h,cpp}` (new), `src/drive/Drive.cpp`, `src/drive/LineFollower.cpp`, `src/manip/ManipBelt.h`, `test/hal_mocks/MatrixMiniR4.h`, `test/…` (new host test), `docs/decisions/066-*.md`, `DECISIONS.md` | Create `EncoderSource` (extern singleton `Encoder`) mirroring `YawSource`: `readDegrees(id,out)` surfacing + retry-once, `readDegreesOr(id,fallback)` last-good, `isPrevTaskDone()` OK-only. Route the call sites: Drive.cpp `getEncL/R` (:496/:501), `_updateProfile` (:721), `_updateStopAtLine` (:1201), `_updateWallStop` (:1408/:1421), `isPrevTaskDone` (:120), `calibrateMotorBias` (:559-560), profile/stop arm seeds (:1039/:1711); LineFollower.cpp distance gate (:531-532) + ramp-down (:279-280); ManipBelt.h `beltTo` (:194) + `_getDegrees` dispatcher. Each site passes its own last-good as fallback. Add `GetEncoderDegrees` to the HAL mock; write TC1/TC2. Write ADR-066, update DECISIONS.md index + status table. Satisfies AC #141. |
| T2 | #142 turn passed-target | T1 | med | `src/drive/Drive.cpp`, `test/…` | In `_updateTurnProfile` (:1102-1116), before the band checks: if in SLOW/approach and `turnSign(rawDelta) != _turnDeltaSign`, go straight to BRAKE. Add TC3 (straddle). Satisfies AC #142. |
| T3 | #150 drain fault early-out | T2 | low | `src/drive/Drive.cpp`, `test/…` | At the top of `_drainBlocking` loop (:1817): `if (hasFault()) { _clearQueue(); return; }` — preserve original kind. Add TC7. Satisfies AC #150. |
| T4 | #153 fabricated yaw | T3 | low | `src/drive/Drive.cpp`, `test/…` | Replace `readYawDegOr(0.0)` with `readYawDegOr(_heldYaw)` in `_execTurnDeg` (:353), `_execTurnToYaw` (:413), `holdYaw` (:515-519). Add TC8. Satisfies AC #153. |
| T5 | #154-16 wallStop guard | T4 | low | `src/drive/Drive.cpp`, `test/…` | Top of `_updateWallStop` SQUARE branch (:1397): `if (hasFault()) { _endWallStop(true); return; }`. Add TC9. Satisfies AC #154-16. |
| T6 | #145 fault-clear cascade | — | high | `src/mission/PathTracker.cpp`, `src/mission/Mission.cpp`, `test/…`, `docs/decisions/067-*.md`, `DECISIONS.md` | Have `PathTracker::clearFault()` (:490) and `PathTracker::abort()` (:499) call `Drive.clearFault()`; add it to `Mission::abort()` (:198-223). Verify the BTN_DOWN reset (Mission.cpp:641-643) now clears Drive via the cascade. Write ADR-067 (fault-clear cascade / single recovery contract), update DECISIONS.md. Add TC4. Satisfies AC #145. |
| T7 | #146 lost-line fault | T6 | high | `src/mission/PathTracker.cpp`, `src/mission/PathTracker.h`, `config/ConfigNav.h`, `docs/decisions/009-*.md` (status note), `test/…` | Add `Fault::LINE_LOST = 10` + mnemonic. Add `Config::LINE_LOST_FAULT_MS = 500` (`// TODO bench-tune`) to ConfigNav.h. In `LINE_FOLLOW` (:932) and `LINE_OVERSHOOT` (:1069): poll `LineFollower.isLineLost()`; if lost AND not in a dead-reckon window (`!_overshootMode`/`!_passThrough`/`!_projPassThrough`) for ≥ `LINE_LOST_FAULT_MS`, `_enterFault(Fault::LINE_LOST, …)`. Add TC5/TC6. Note ADR-009 status. Satisfies AC #146. |
| T8 | #149 line-board cache | — | med | `src/perception/WROLineArray.{cpp,h}` | In `setReverseActive` (:244-254), when `_activeBoard` actually changes and `LINE_ARRAY_TICK_CACHE_ENABLED`, call `_refreshCache()` (re-reads the new board). Verify via `arduino-cli` compile; add a bench-verify note (not host-compiled). Satisfies AC #149. |
| T9 | #154-18 Vision sentinel | — | low | `src/perception/Vision.{cpp,h}`, `test/test_vision.cpp` | `getRawHue` (:539) returns `0xFF` for `!_hasRead`/OOB; document in Vision.h (:180-192). Add TC11. Satisfies AC #154-18. |
| T10 | #152 config hardening | — | med | `config/ConfigChecks.h`, `config/ConfigLine.h`, `config/ConfigDrive.h`, `config/ConfigMission.h`, `config/ConfigManipulator.h`, `WRO2026.ino`, `docs/OPERATOR_GUIDE.md`, `test/…` (compile-fail probe) | Add junction-knob asserts (`LEFT_HI ≤ 7`, `RIGHT_LO ∈ [1,7]`, `T_MIN_ON ∈ [1,8]`) and a **correct** turn-ladder invariant (see Notes — the issue's literal assert breaks the build) to ConfigChecks.h. Fix stale comments at ConfigDrive.h:484-486 and ConfigLine.h:256. Rewrite ConfigMission.h:35/43-47 + OPERATOR_GUIDE.md:668-670 to state `true`-for-bench truth; add a self-silencing boot `[WARN]` banner in setup() after Serial.begin (:181). Optional: servo-drain assert (constexpr reducer over the servo structs). Add a compile-fail probe (TC10). Do **not** change the `MISSION_ALLOW_REARM` value. Satisfies AC #152. |
| T11 | #147 turn-gate parity | — | high | `tools/path_planner/sim/runtime.py`, `tools/path_planner/compiler.py`, `tools/path_planner/tests/…` (new parity test + fixture) | Change the sim `_emit_turn_to` gate (:2541) and compiler gate (:369, :202) from `1e-6` to `NAV_TURN_TOLERANCE_DEG`, advancing sim heading on pass-through so no micro-turn accumulates (mirror PR35 Option); keep TC12/TC18 byte-parity (pin live-vs-latent first; flag any golden shift). Fix the misleading `_emit_turn_to` docstring (:2525-2528). Add a parametric parity test parsing each mirrored constant via `config_parse` and asserting the `runtime.py` mirror equals it (covers the untested list: `LINE_SENSOR_AXLE_OFFSET_CM/_REAR_CM`, `WALL_CONTACT_OFFSET_CM`, `WALL_STOP_BACK_OFF_CM`, `SPEED_NORMAL/SLOW/FAST`, `TSHAPE_START_SPEED`, `TSHAPE_MIN_MOVE_CM`, `PIVOT_TRIM_MIN_DRIVE_CM`). Add a near-collinear fixture (TC13). Satisfies AC #147. |
| T12 | #159 pass-through seed | T11 | med | `tools/path_planner/sim/runtime.py`, `tools/path_planner/tests/…` | Carry the inbound line hop's resolved tier through `_walk_hops` alongside `pass_through_active`; seed `entry_mag = min(inbound_tier_pwm, cruise_mag)` (:3479-3483) instead of `min(SPEED_NORMAL, cruise_mag)`. Add TC18 (FAST→FAST). Update the known-limitation comment (:3462-3478). Satisfies AC #159. |
| T13 | #154-17 facing round | T12 | low | `tools/path_planner/sim/runtime.py`, `tools/path_planner/tests/…` | In `_dispatch_wall_reset` (:2711) / `LocalizationSim.reanchor_at_wall`, quantize `facing` with the same `int(round(...)) % 360` the emitter uses (header_writer.py:884). Add TC19. Satisfies AC #154-17. |
| T14 | #148 validator PIPELINE_* | — | med | `tools/path_planner/validator.py`, `tools/path_planner/tests/…` | Make `_edge_attrs` (:558-567) also return `data.get("mode")`; resolve `cfg = PipelineConfig.from_field_data(field_data)` once in `_validate_pivot_turns`; change the `_regime` `resolve_stop_at_line` call (:617-625) to `resolve_blind_mode(edge_attrs, cfg)` + `cfg.blind`. Add TC14/TC15. Satisfies AC #148. |
| T15 | #151 swapped-pair guard | T14 | low | `tools/path_planner/validator.py`, `tools/path_planner/tests/…` | In `_validate_edges_field` (:888), error when two swapped-pair `EDGES` entries would clobber each other's `heading_deg` (explicit heading on one inconsistent with the `_reverse_heading` the pair would write for the same directed edge). Name both entries. Add TC16. Satisfies AC #151. |
| T16 | #157 CTPS RETURN speed | — | low | `tools/path_planner/ctps/track_builder.py`, `tools/path_planner/tests/…` | Render `ACT_RETURN_FORWARD`/`ACT_WALL_DEPART` at `_blind_cruise_pwm(cfg, pipeline_cfg.speed)` (thread the already-resolved `pipeline_cfg` from `build()` :1338 into `_primitive_drive_speed`). Add TC17. Satisfies AC #157. |
| T17 | #154-20 mosaic whitelist | — | low | `tools/path_planner/ctps/trace_loader.py`, `tools/path_planner/ctps/mosaic.py`, `tools/path_planner/tests/…` | In `_parse_mosaic` (:624-636) validate each cell against `mosaic.COLOR_NAMES ∪ {"UNKNOWN"}` (warn-and-skip otherwise); correct the `mosaic.py:16-17` docstring. Add TC20. Satisfies AC #154-20. |

## Notes for Implementer

- **Drive.cpp lane (T1–T5) is serial** — five issues share `Drive.cpp`; each is blocked by the previous so no two editors collide. Same for PathTracker.cpp (T6→T7), sim/runtime.py (T11→T12→T13), validator.py (T14→T15).
- **#141 vendor reach:** the current firmware reads `MiniR4.M2.getDegrees()` (bare int32, no RESULT). To surface the RESULT, `EncoderSource` must call the vendor's two-channel `mmL.GetEncoderDegrees(id, degs)` directly (as the issue specifies) — confirm `mmL` is reachable from `src/nav/` and add the matching mock method. If `mmL` is not directly reachable, fall back to detecting the vendor wrapper's ambiguity another way, but prefer the RESULT-surfacing call.
- **#152 turn-ladder assert (do NOT paste the issue's literal assert):** actual values are `TURN_CRUISE_PWM=50`, `TURN_SLOW_SPEED_PWM=20`, `PWM_MIN_MAGNITUDE=35`. The issue's `static_assert(TURN_SLOW_SPEED_PWM >= PWM_MIN_MAGNITUDE …)` is **false** (20 < 35) and would break the build. `TURN_SLOW_SPEED_PWM` is intentionally a below-floor setpoint that `deadbandSnap` raises to `PWM_MIN_MAGNITUDE` at emit. Assert the *true* invariant (e.g. `TURN_CRUISE_PWM >= PWM_MIN_MAGNITUDE` and `TURN_SLOW_SPEED_PWM <= TURN_CRUISE_PWM`) and fix the stale "set equal to PWM_MIN_MAGNITUDE" comment to describe the snap-up behavior. Confirm the build compiles before committing.
- **#142:** the issue's prose says the SLOW state is floored to 50; the real floor is `PWM_MIN_MAGNITUDE=35`. This does not change the passed-target fix (sign-flip detection is independent of the floor value).
- **#146 suppression:** the deliberate dead-reckon window is exactly when `LineFollower._overshootMode` is set (armed by PathTracker at :1057). Gate the new fault on that plus `State::LINE_OVERSHOOT`/`_passThrough`/`_projPassThrough` so an intentional overshoot never faults. `_enterFault` (:1411) already disables LineFollower before `Drive.stop` — reuse it, do not hand-roll the beep.
- **#147 live-vs-latent:** confirm first whether the "advance heading to the edge on pass-through" fix already landed (`runtime.py:1824` pass-through *decision* uses `NAV_TURN_TOLERANCE_DEG`). If it did, the residual `1e-6` gate only bites non-pass-through sub-0.5° cases; if not, it bites collinear hops. Move the sim and compiler gates together to preserve TC12/TC18 byte-parity. Toy fixtures are axis-aligned (exact 0°/90°), so existing goldens should not shift — verify.
- **#149** is not host-compiled (`WROLineArray.cpp` is always mocked), so it gets `arduino-cli` compile coverage only; leave a bench-verify note in the PR.
- **No routes regen:** none of these fixes touch `header_writer.py`/`field_data.json`; if a diff appears in `routes.h/.cpp`, something went wrong — stop and re-scope.
- **Rollback:** every task is one commit; revert the offending commit. The two ADRs are additive docs.
- **Regional is 2026-07-21** — prioritize the P1 firmware lanes (T1–T7) if time is tight; the P2/P3 planner/sim/config work is latent (no current-field runtime effect).
