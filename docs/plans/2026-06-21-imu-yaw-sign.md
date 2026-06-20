# IMU Yaw Sign Normalization (CCW-positive) Plan

**Goal:** Make the robot's app/world frame uniformly CCW-positive — so `turnDeg(90)` turns 90° left and world navigation is un-mirrored — by correcting the hardware-measured CW-positive IMU yaw through a single per-chassis sign constant.

**Approach:** Introduce a `YawSource` module that OWNS all raw IMU yaw I/O. It applies `Config::IMU_YAW_SIGN` (= −1 for the current chassis) once on every read (STM32→app, via the result-surfacing `mmL.GetIMUEuler`) and once on every heading target written to the STM32 gyro PID (app→STM32, `MoveGyro*`). The turn-direction sign math is promoted into a header-only `TurnSignPolicy.h` (mirroring `DirectionPolicy.h`/`PivotTrimPolicy.h`) so it is host-testable. `Drive` and `Localization` consume only these. The motor/wheel pattern and the Python planner/sim are untouched.

## Background (verified)

- **Hardware fact (bench-measured):** rotating the chassis counter-clockwise *decreases* the raw IMU yaw → the raw IMU is **CW-positive**. The motors are correct: forward = `+ +`, and the counter-rotation pattern `L = −sign·pwm, R = +sign·pwm` physically produces CCW for a positive command.
- **App convention:** the Python planner emits world headings as `atan2(dy,dx) % 360` = CCW-positive, 0° = +x (East); `Localization` documents the same. So the raw IMU currently disagrees with the app by a sign.
- **Consequence today:** the custom R4 turn loop drives CCW expecting yaw to rise; yaw falls; it commits to the wrong direction and spins the long way / never settles. World nav would be mirrored.
- **Two IMU crossings (both must be bridged):**
  - **Read (STM32→app):** 6 sites — Localization `_readImuYaw` ×1 (uses `mmL.GetIMUEuler`), and Drive's `_execTurnDeg` (`:342`), `_execTurnToYaw` (`:408`), `getYaw` (`:508`), `holdYaw` (`:513`), `_updateTurnProfile` (`:1081`) (these use `MiniR4.Motion.getEuler`, a thin vendor wrapper over `mmL.GetIMUEuler` — value-equivalent on success).
  - **Write (app→STM32):** heading target to the STM32 gyro PID, which holds against the STM32's OWN raw IMU — `MoveGyro` (`Drive.cpp:172` forwardGyro, `:687` `_emitContinuous`), `MoveGyroDegs` (`:260`), `MoveGyroTime` (`:311`). The single `_emitContinuous:687` GYRO emitter serves three channels: `_profileHeading` (`:777`), `_stopHeading` (`:1238`, stopAtLine), `_wallHeading` (`:1417/:1486`, wallStop) — all carry app-frame (`toImuYaw`) headings.
- **Lower-MCU `TurnGyro` is dead** (zero call sites in `WRO2026/`; replaced by the custom `_updateTurnProfile`), so the turn loop writes no yaw to the STM32 — read-side correction alone fixes turn direction.
- **Host-test reality (load-bearing):** `Drive.cpp` is **NOT** host-compiled (`test/CMakeLists.txt`: "Drive.cpp is NOT host-compiled (heavy MatrixMiniR4 deps)"; `SUT_SOURCES` = `LineFollower.cpp` only). Drive-touching tests force-include a no-op mock `test/mocks/Drive.h`, and the mock `MatrixMiniR4.h` has no `Motion` / no `DriveDC`. So the Drive turn path and the GYRO-write call sites are **unreachable from host tests** — only `Localization.cpp` and new self-contained modules (`YawSource`, `TurnSignPolicy`) are host-testable. This drives the test strategy below.

## Scope

- **In scope:**
  - New `YawSource` module: `WRO2026/YawSource.h` + `WRO2026/YawSource.cpp`.
  - New header-only `WRO2026/TurnSignPolicy.h`: the turn-sign capture + counter-rot/pivot wheel-pattern math, extracted from `Drive.cpp` so it is host-testable (repo pattern: `DirectionPolicy.h`).
  - `Config::IMU_YAW_SIGN` in `ConfigLocalization.h` (overridable `#ifndef IMU_YAW_SIGN_DEFAULT` guard, default `-1.0f`) + a `ConfigChecks.h` static_assert tripwire.
  - Route Localization's `_readImuYaw` and all 5 Drive read sites through `YawSource` (sign applied once, in floating point before any int cast).
  - Wrap the 4 STM32 gyro-heading writes with `YawSource::toHwHeading()` (× `IMU_YAW_SIGN`).
  - Have `Drive.cpp` call `TurnSignPolicy` for the sign capture + wheel pattern (no behavior change to the pattern itself).
  - Host tests: dedicated `test_yaw_source` (read sign + `toHwHeading` value) and `test_turn_sign_policy` (wheel-pattern direction); plus the Localization read-sign assertions.
  - Update the legacy tests that bake the old sign by negating their non-zero mock-yaw inputs and re-deriving any sign-sensitive assertions.
  - Doc fixes: rewrite `Drive.cpp:1128-1138` "flip the wheel signs" comment to point at `IMU_YAW_SIGN`/`YawSource`; fix the stale `Drive.h:345` `turnDeg` "non-blocking" → "blocking" comment (evidence: builder default is `_blocking = true` — `Drive.h:2213-2215` class doc "Sync-blocking by default", `:2316` `setBlocking` note "default is now true (sync)").
  - ADR-042 + `DECISIONS.md` index/status; `ARCHITECTURE.md` dependency graph + module table.
- **Out of scope:**
  - The motor/wheel turn pattern values (confirmed correct; `TurnSignPolicy` extracts them verbatim — no value change).
  - The Python planner and sim (live in the world frame, never touch raw IMU — must remain byte-for-byte unchanged; AC8).
  - The other chassis's sign value (sets its own `IMU_YAW_SIGN_DEFAULT` override later).
  - Hardening dispatch-time IMU read failures beyond today's behavior (the per-tick turn loop gains graceful skip-on-fail; dispatch sites keep best-effort).
  - Reflashing/remapping the STM32 firmware IMU axis (vendor black box).
  - Building a host harness that compiles real `Drive.cpp` (large new effort; not justified — the *logic* is host-guarded via `YawSource` + `TurnSignPolicy`, the *wiring* via arduino-cli + bench).

## Decisions

- **Bidirectional sign bridge, shipped atomically.** Read-flip and write-flip are coupled: the write-side `×SIGN` is valid *only because* the read-flip redefines `_yawOffsetDeg` and `_heldYaw` into the corrected frame. They land together or the frame is internally inconsistent.
- **`YawSource` owns I/O on `mmL.GetIMUEuler`** (result-surfacing), not `Motion.getEuler` (which discards the RESULT). Unifies the read path and surfaces transient timeouts to the Drive turn loop. NOTE: this does **not** make the Drive path host-testable (Drive.cpp isn't compiled on host) — that was a wrong earlier assumption; host coverage comes from the extracted policy + YawSource unit tests.
- **Turn-sign logic promoted to `TurnSignPolicy.h`** (host-tested) so the direction fix has an automated regression guard, since `Drive.cpp` itself is bench/compile-only. (Phase 5.7 judge synthesis.)
- **Sign in floating point before any int16 cast.** `YawSource::readYawDeg` returns `double`; callers cast once.
- **`toHwHeading` is pure `×SIGN`, no offset** — every value reaching a gyro write (`_heldYaw`, and `cmd.heading`/`yaw` from `toImuYaw`) is already in the corrected app frame; app↔raw differ by sign alone. Precondition: nothing ever passes a *raw* heading into a `*Gyro*` target (guard-commented at the call sites).
- **Test strategy: negate legacy inputs + dedicated tests** (over insulating with `+1`), so the real production sign is exercised end-to-end where reachable.
- **`IMU_YAW_SIGN` overridable** via `#ifndef IMU_YAW_SIGN_DEFAULT` (two-chassis reality + per-target test override via `target_compile_definitions` — a per-file `#define` won't reach the separately-compiled `YawSource.cpp`), default `-1.0f`.
- **Tripwire placement:** the `±1` static_assert goes in `ConfigChecks.h` (coupled-sites style, matching its Vision guard at `:85`), even though `ConfigLocalization.h` keeps its own intra-module asserts — the IMU-sign assert is about cross-module call sites, so ConfigChecks is the right home.
- **Approach consult (Phase 4.5):** Opus endorsed the bidirectional bridge + single-owner wrapper.

## Acceptance Criteria

- [ ] **AC1 — Turn direction (logic host-guarded; wiring bench).** `TurnSignPolicy` maps a positive yaw delta to the CCW wheel pattern (`L<0, R>0`) and negative to CW; combined with AC2's read sign this means `turnDeg(90)` rotates CCW/left. *Host:* `test_turn_sign_policy`. *Bench/compile:* the robot physically turns left for `turnDeg(90)` and the loop converges.
- [ ] **AC2 — Read sign (host).** After `zeroHeadingFromCurrent()` at raw yaw 0 (offset 0), raw −90 → `getHeadingDeg() == 90`; raw +90 → `270`. World heading increases under CCW rotation. (`Localization.cpp` is host-compiled.)
- [ ] **AC3 — GYRO write conversion (value host-guarded; wiring bench).** `YawSource::toHwHeading(H) == IMU_YAW_SIGN·H` exactly (host). *Bench/compile:* a GYRO straight-line drive holds its commanded heading on the robot (the real `MoveGyroDegs`/`_emitContinuous` sites receive the converted value).
- [ ] **AC4 — Config guard.** `IMU_YAW_SIGN` defaults to `-1.0f`, is overridable via `-D IMU_YAW_SIGN_DEFAULT=...`, and the `ConfigChecks.h` static_assert rejects any value other than `±1.0f` (compile-fail test).
- [ ] **AC5 — Single owner.** No call to `Motion.getEuler` or `mmL.GetIMUEuler` remains outside `YawSource` (grep-clean across `WRO2026/`, excluding the mock); no MoveGyro heading write bypasses `YawSource::toHwHeading`; no turn-sign/wheel-pattern math remains inline in `Drive.cpp` outside a `TurnSignPolicy` call.
- [ ] **AC6 — Green.** Full `ctest` passes, full `uv run --extra dev pytest` passes, `arduino-cli compile` succeeds.
- [ ] **AC7 — Docs.** ADR-042 exists and is indexed (status table bumped); `ARCHITECTURE.md` shows `YawSource` + `TurnSignPolicy`; the two stale comments are fixed.
- [ ] **AC8 — Sim untouched.** `git diff` shows zero changes under `tools/path_planner/`.

## Frame invariant (the load-bearing contract)

Let `r` = raw STM32 yaw (CW-positive). Define corrected app reading `c = IMU_YAW_SIGN · r` (CCW-positive). Then:

- `Localization`: `world = normalize(c − offset)`, `offset = c_zero − START_HEADING` (captured in the `c` frame).
- `toImuYaw(world) = world + offset` → a value in the **`c` frame** (NOT raw).
- Custom turn loop reads `c`, commands raw `Move(L,R)` (loop decides direction via `TurnSignPolicy`) — consistent, no STM32 write.
- STM32 gyro holds `r`. We pass it `toHwHeading(c_target) = IMU_YAW_SIGN · c_target = r_target`. Pure sign — `c↔r` is sign-only and the offset is already folded into `c_target`.
- **All three GYRO heading channels carry `c`-frame values:** `_profileHeading`, `_stopHeading`, `_wallHeading` are each set from `cmd.heading` (`_headingSet ? _heading : _heldYaw`), where `_heading` is `toImuYaw` output and `_heldYaw` is a corrected read / turn target. So the single `toHwHeading` wrap at `_emitContinuous:687` (plus the bounded `MoveGyroDegs`/`MoveGyroTime` sites) covers straight-line, stop-at-line, and wall-stop alike.

## Error Handling

- **Transient IMU read timeout (read path):** `YawSource::readYawDeg` retries once after a short delay then returns `false`; out-param untouched. `Localization` serves last-good heading (unchanged). `_updateTurnProfile` SKIPS the tick on `false` — a deliberate **behavior change** vs today's "act on the vendor's 0.0" (a skipped tick holds the previous `Move(L,R)` one more cycle); bench-verify mid-turn robustness. Dispatch sites + `getYaw`/`holdYaw` use `readYawDegOr(fallback)`, preserving today's best-effort.
- **Invalid `IMU_YAW_SIGN`:** compile-time static_assert (build fails).
- **Raw heading into a gyro write (precondition violation):** guard comment at the 4 sites; if a future caller passes a raw heading, `toHwHeading` double-corrects — the bench GYRO-hold check is the backstop.

## Testing Strategy

**Levels:** host unit (ctest, the reachable logic), planner/sim (pytest, must stay green untouched), firmware compile (arduino-cli), bench (the Drive wiring + physical direction).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | Zero at raw 0; set raw −90; read world | host (Localization) | `getHeadingDeg() == 90` |
| TC2 | Zero at raw 0; set raw +90; read world | host (Localization) | `getHeadingDeg() == 270` |
| TC3 | `YawSource::readYawDeg` with `mmL.mockYawDeg=-90` | host (YawSource) | returns `+90` (sign applied); `false` + untouched on `mockResult=ERROR` |
| TC4 | `YawSource::toHwHeading(H)` for several H | host (YawSource) | `== IMU_YAW_SIGN·H` exactly (e.g. H=90 → −90; H=−90 → 90) |
| TC5 | `TurnSignPolicy` wheel pattern, COUNTER_ROT, +delta | host (policy) | `L<0, R>0` (CCW); −delta → `L>0, R<0`; PIVOT_LEFT/RIGHT variants match `Drive.cpp:1147-1154` |
| TC6 | `TurnSignPolicy` sign capture | host (policy) | `sign(+90)=+1`, `sign(−90)=−1`, boundary at ±180 fold matches `_normalizeYaw` |
| TC7 | `IMU_YAW_SIGN = 0.0f` via `-D` override target | host (compile-fail) | static_assert fires; build fails (REQUIRED — AC4 is binding) |
| TC8 | Legacy offset/reanchor tests, inputs negated | host | world-frame assertions hold under production sign (re-derived, see T8) |
| TC9 | Full sim parity suite | pytest | unchanged — all pass with zero sim edits |
| TC10 | `arduino-cli compile` | compile | links clean; no stray `Motion.getEuler`/`GetIMUEuler` (AC5 grep) |
| TC11 | Bench: forward, `turnDeg(±90)`, GYRO straight, stopAtLine, wall-zero | manual | turns go the labeled direction + converge; GYRO/stop/wall hold heading |

**Test data:** `mmL.mockYawDeg` (raw) + `mmL.mockResult`; `TurnSignPolicy` + `YawSource` tested as pure/self-contained units (no Drive harness). **Run:** `ctest`; `uv run --extra dev pytest tools/path_planner/tests/`; `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Config constant | — | low | `WRO2026/ConfigLocalization.h` | Add `IMU_YAW_SIGN` with `#ifndef IMU_YAW_SIGN_DEFAULT` guard (default `-1.0f`) + Doxygen noting it is consumed ONLY by `YawSource`. (The guard pattern matches `PATH_PASS_THROUGH_ENABLED` in `Config.h:1062` — different file, same idiom.) Satisfies AC4. |
| T2 | ConfigChecks tripwire | T1 | low | `WRO2026/ConfigChecks.h` | `static_assert(IMU_YAW_SIGN == 1.0f \|\| IMU_YAW_SIGN == -1.0f, ...)` + coupled-sites comment naming `YawSource::readYawDeg`/`toHwHeading`. Satisfies AC4. |
| T3 | YawSource module | T1 | high | `WRO2026/YawSource.h`, `WRO2026/YawSource.cpp` | Implement the API (below). `readYawDeg` mirrors today's `_readImuYaw` (retry-once via `mmL.GetIMUEuler`; `IMU_READ_RETRY_DELAY_MS` moves here) then `yawOut = IMU_YAW_SIGN * (float)yaw` on OK; returns `double`-precision via `readYawDegOr`. `toHwHeading` = `(int16_t)(IMU_YAW_SIGN * appHeadingDeg)`. Float before any cast. Singleton `extern`. Satisfies AC2/AC3/AC5. |
| T4 | TurnSignPolicy header | — | med | `WRO2026/TurnSignPolicy.h` | Header-only inline functions extracted from `Drive.cpp`: (a) `turnSign(int32_t deltaNorm) -> int8_t` (sign capture, today at `_execTurnDeg`/`_execTurnToYaw`); (b) `turnWheelPattern(int8_t sign, int16_t pwm, uint8_t mode) -> {L,R}` (the COUNTER_ROT/PIVOT mapping at `Drive.cpp:1143-1160`). VERBATIM logic — no value change. Doxygen per repo style. Satisfies AC1. |
| T5 | Localization wiring | T3 | high | `WRO2026/Localization.cpp` | `_readImuYaw` body → `return YawSource.readYawDeg(yawOut);`; remove local `IMU_READ_RETRY_DELAY_MS`; `#include "YawSource.h"`. Call sites unchanged. Satisfies AC2/AC5. |
| T6 | Drive wiring + comment fixes | T3, T4 | high | `WRO2026/Drive.cpp`, `WRO2026/Drive.h` | Route 5 reads through `YawSource` (`_updateTurnProfile` uses bool API + skip-on-fail; others `readYawDegOr`). Replace inline sign capture + wheel pattern with `TurnSignPolicy` calls. Wrap the 4 MoveGyro heading args (`:172,:260,:311,:687`) with `YawSource::toHwHeading(...)` + a precondition guard comment. `#include` both new headers. Rewrite `:1128-1138` comment to cite `IMU_YAW_SIGN`/`YawSource`. Fix `Drive.h:345` "non-blocking"→"blocking". Satisfies AC1/AC3/AC5/AC7. |
| T7 | Host tests + CMake | T3, T4, T5 | med | `WRO2026/test/CMakeLists.txt`, `WRO2026/test/test_yaw_source.cpp`, `WRO2026/test/test_turn_sign_policy.cpp` | Add `YawSource.cpp` to EVERY target that compiles `Localization.cpp` (else they fail to link after T5). Add self-contained `test_yaw_source` (TC3/TC4, links `YawSource.cpp` + mock `MatrixMiniR4.h`) and `test_turn_sign_policy` (TC5/TC6, header-only, no mocks — like `test_pivot_trim_policy`) targets + `add_test`. Add the `IMU_YAW_SIGN=0` compile-fail target (TC7) via `target_compile_definitions(... IMU_YAW_SIGN_DEFAULT=0.0f)` + `PASS_REGULAR_EXPRESSION` on the static_assert message. CMakeLists is edited ONLY here (no parallel conflict). Satisfies AC1/AC2/AC3/AC4. |
| T8 | Negate legacy inputs + re-derive | T5, T7 | med | `WRO2026/test/test_localization_offset.cpp`, `WRO2026/test/test_reanchor_at_wall.cpp`, `WRO2026/test/test_reanchor_at_wall_strict.cpp`, `WRO2026/test/test_path_tracker_reset_arrival.cpp` | Negate each non-zero `mmL.mockYawDeg = X` → `-X`. For each test, VERIFY it zeroes at raw 0 (offset 0, so negation preserves world-frame assertions); where a test zeroes at non-zero raw OR asserts against a non-negated world constant (e.g. strict `facing=90`, boundary cases at `reanchor_at_wall_strict:205-210`), RE-DERIVE the expected value, don't blindly keep it. These tests use the mock Drive (not real Drive.cpp), so they don't depend on T6. Run FULL ctest to catch any other target setting a non-zero mock yaw. Satisfies AC6/TC8. |
| T9 | ADR-042 + index | — | low | `docs/decisions/042-imu-yaw-sign-normalization.md`, `DECISIONS.md` | Rule: all raw IMU yaw I/O goes through `YawSource`, which applies the per-chassis `IMU_YAW_SIGN` on read and on STM32 gyro-heading write; turn-sign math lives in `TurnSignPolicy`. Add to index + bump status table. 042 is next free (041 highest; 039 skipped); watch unmerged-branch collision. Satisfies AC7. |
| T10 | ARCHITECTURE update | — | low | `ARCHITECTURE.md` | Add `YawSource` (lowest app layer over vendor IMU; read by Drive + Localization) + `TurnSignPolicy` to the dependency graph + module table. Satisfies AC7. |
| T11 | Final verification | T2, T6, T7, T8, T9, T10 | med | — (no source edits) | Build + full `ctest`; `uv run --extra dev pytest tools/path_planner/tests/`; `arduino-cli compile`. Confirm AC5 grep-clean and AC8 (`git diff` clean under `tools/path_planner/`). Satisfies AC5/AC6/AC8. |

## Notes for Implementer

- **Atomicity:** T5 + T6 (reads) and T6 (writes) must all be present before the suite is meaningful — a half-applied frame is internally inconsistent. Execute order (T3/T4 → T5/T6 → T7/T8 → T11) enforces it; never land a partial.
- **Drive.cpp is not host-tested** — its correctness rests on `TurnSignPolicy`/`YawSource` host tests (the logic), arduino-cli compile (it builds), careful review (the glue), and the bench (TC11). Do NOT assume a host test will catch a wiring slip in T6; review it like it's unguarded, because the glue is.
- **`toHwHeading` direction is the silent foot-gun:** `SIGN·SIGN = 1`, so a double-application looks like a no-op until the robot veers. TC4 pins the value; the bench GYRO-hold (TC11) is the wiring backstop. Add the precondition guard comment at all 4 sites.
- **Float-before-cast** in `YawSource`; `readYawDeg` returns `double`. `getYaw()` keeps precision: `double y; return YawSource.readYawDeg(y) ? y : 0.0;`.
- **Don't build a Motion mock / a Drive host harness** — out of scope; the logic is covered by the two new self-contained test targets.
- **`#include` rules:** Drive.cpp / Localization.cpp include `YawSource.h`; Drive.cpp includes `TurnSignPolicy.h`; never `#include` a `.cpp`.
- **CMake:** `YawSource.cpp` must be added to every target compiling `Localization.cpp` (and the dedicated `test_yaw_source`). Missing one = link error in that target only.
- **Run the FULL suites**, not per-file (a Config/contract change historically breaks fixtures across multiple host AND python targets that only a full run surfaces). `uv run --extra dev pytest`.
- **Rollback:** set `IMU_YAW_SIGN_DEFAULT` back to `+1.0f` to neutralize behavior without reverting structure, or revert the branch. Sim/planner untouched → no `routes.h` regeneration.
- **Background-job execution:** code-writer sub-agents AUTHOR + RETURN content; the orchestrator applies all writes in the worktree (bg-isolation blocks sub-agent writes); use absolute worktree paths.

## API

`WRO2026/YawSource.h`:

```cpp
class YawSourceClass
{
public:
    /// Result-surfacing read. mmL.GetIMUEuler (retry-once on transient timeout), applies
    /// Config::IMU_YAW_SIGN, writes corrected yaw (deg) to @p yawOut ONLY on success.
    /// Returns false on hard failure (yawOut untouched).
    bool   readYawDeg(double& yawOut) const;
    /// Fire-and-forget convenience: corrected yaw on success, else @p fallback.
    double readYawDegOr(double fallback) const;
    /// App-frame heading -> STM32 raw-frame heading for a MoveGyro* target (pure ×SIGN).
    static int16_t toHwHeading(int16_t appHeadingDeg);
};
extern YawSourceClass YawSource;
```

`ConfigLocalization.h`:
```cpp
#ifndef IMU_YAW_SIGN_DEFAULT
#define IMU_YAW_SIGN_DEFAULT (-1.0f)   // current chassis: raw IMU is CW-positive
#endif
constexpr float IMU_YAW_SIGN = IMU_YAW_SIGN_DEFAULT;
```
