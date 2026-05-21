# Issue #15 — Switch Motor Profile from Matrix TT to LEGO EV3 (with TT↔LEGO Toggle)

**Goal:** Reflect reality in `Config.h` — Lofaix uses LEGO EV3 motors, not Matrix TT — and add a single-line compile-time toggle so the test rover (TT) and Lofaix (EV3) can swap without scattered edits.

**Approach:** Introduce two `constexpr` structs (`MatrixTT`, `LegoEV3`) in `namespace Config` carrying the vendor-cited PPR/RPM/PID values. Add `using ActiveMotor = LegoEV3;` as the single edit point. Keep the existing flat constants (`MOTOR_PPR`, `MOVESYNC_KP`, …) as one-line forwarders so `Drive.cpp` callsites are untouched. Update `MOTOR_LEFT/RIGHT_REVERSE` documentation to make clear it's a chassis-wiring property (orthogonal to motor type). Reframe the TT-flavored comments throughout `Config.h`, flip the dev-target wording in ADR-011, generalize ADR-017, mark Phase 7 P7.0b done in Roadmap.md, and add ADR-022 documenting the new mechanism.

## Scope

- **In scope:**
  - `WRO2026/Config.h` — new `MatrixTT` / `LegoEV3` structs, `ActiveMotor` alias, flat forwarders, generalized comments, one-line "re-run calibrateMotorBias() on swap" reminder.
  - `docs/decisions/011-motor-bias-strategy-b.md` — flip "test rover (TT, current dev target)" wording. Rule unchanged.
  - `docs/decisions/017-sync-plus-tshape-default-profile.md` — generalize "two TT motors" → "two drive motors". Rule unchanged.
  - `Roadmap.md` — Phase 7 paragraph (lines 33-41), exit gate at line 230, task row P7.0b (line 248), risk row #12 (line 356).
  - `docs/decisions/022-dual-motor-profile.md` — new ADR documenting the constexpr-struct + alias mechanism.
  - `DECISIONS.md` — one-line index entry for ADR-022 + status-summary bump.
- **Out of scope:**
  - `Drive.calibrateMotorBias()` workflow or any `MOTOR_BIAS_L/R` values — per-chassis, requires hardware.
  - `LINE_KP` / `LINE_KD` retune — explicit Phase 7 exit gate P7.0d.
  - Any `Drive.cpp` logic changes — the flat-forwarder design keeps callsites byte-identical.
  - Static-assert linking `ActiveMotor` to `MOTOR_BIAS_L/R` calibration state — user explicitly declined the "bulletproof" option.
  - Parallel `ChassisProfile` struct for bias scalars — user explicitly declined.

## Decisions

- **Toggle mechanism:** `constexpr struct + using` alias inside `namespace Config`. Single-line edit to swap chassis.
- **Default after this PR:** `using ActiveMotor = LegoEV3;` (Lofaix is the real competition chassis per `project_two_chassis.md`).
- **TurnGyro Kp for LEGO:** LE-F variant (`Kp=22.75, Ki=0.02, Kd=1.54`) per vendor example `MiniR4_DriveDC_Test.ino:61`.
- **Flat constants:** kept as one-line forwarders to `ActiveMotor::...` so `Drive.cpp` is untouched.
- **Reverse flags:** stay as standalone constants — wiring property, not motor-type property. Existing static_asserts at `Config.h:76-83` preserved.
- **Compile-time guard:** one Doxygen note next to `using ActiveMotor = ...;` reminding the operator to re-run `Drive.calibrateMotorBias()` after a swap. No new static_asserts.
- **ADR strategy:** update ADR-011/017 in-place (descriptive context, not the rule); add ADR-022 for the new mechanism.

## Acceptance Criteria

- [ ] `Config.h` defines `Config::MatrixTT` and `Config::LegoEV3` structs each carrying `PPR`, `RPM`, `MOVESYNC_KP/KI/KD`, `MOVEGYRO_KP/KI/KD`, `TURNGYRO_KP/KI/KD` with vendor-cited values.
- [ ] `Config::ActiveMotor` resolves to `Config::LegoEV3` after this PR.
- [ ] `Config::MOTOR_PPR`, `Config::MOTOR_RPM`, `Config::MOVESYNC_K{P,I,D}`, `Config::MOVEGYRO_K{P,I,D}`, `Config::TURNGYRO_K{P,I,D}` resolve to `ActiveMotor::*` values via flat forwarders.
- [ ] `Drive.cpp` source is unchanged (no diff in the file).
- [ ] All TT-specific comments in `Config.h` (header doxygen at lines 16-19, motor-profile block 91-99, lines 101/104/181/552/577) are reworded to be motor-type-neutral, with one source-cited Doxygen comment near `ActiveMotor` pointing at MiniR4DriveDC.h docstrings + vendor example.
- [ ] `docs/decisions/011-motor-bias-strategy-b.md` line 12 wording flipped: LEGO EV3 (Lofaix) is the active dev target after this PR; TT is the rover-side fallback. Re-calibration warning intact.
- [ ] `docs/decisions/017-sync-plus-tshape-default-profile.md` line 23 generalized.
- [ ] `Roadmap.md`: Phase 7 paragraph (~lines 33-41) reframed (motor swap done; Phase 7 = calibration + line-PID retune only); exit gate line 230 reworded as "Lofaix verified on hardware"; P7.0b row marked done; risk row #12 reworded to cite the new struct-alias mechanism.
- [ ] `docs/decisions/022-dual-motor-profile.md` exists with the standard ADR shape (Status / Date decided / Rule / Why / How to apply / Trigger to revisit / See also). Cites the vendor source lines.
- [ ] `DECISIONS.md` has a new index entry for ADR-022 and the status-summary table bumped.
- [ ] `uv run pytest tools/path_planner/tests/` passes (smoke check; Python is unaffected).
- [ ] `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` compiles cleanly (if `arduino-cli` available locally; otherwise fall back to documented grep audit).

## Data Model

```cpp
// WRO2026/Config.h — additions

namespace Config
{
    /**
     * @brief Vendor PID + encoder defaults for the Matrix TT motor.
     * @note Source: MatrixMiniR4 MiniR4DriveDC.h PID docstrings (setMoveSyncPID, setMoveGyroPID,
     *       setTurnGyroPID) + examples/4-DriveDC Functions/MiniR4_DriveDC_Test/MiniR4_DriveDC_Test.ino:64-74.
     */
    struct MatrixTT
    {
        static constexpr uint16_t PPR = 545;
        static constexpr uint16_t RPM = 200;
        static constexpr float    MOVESYNC_KP = 0.02f;
        static constexpr float    MOVESYNC_KI = 0.0f;
        static constexpr float    MOVESYNC_KD = 0.04f;
        static constexpr float    MOVEGYRO_KP = 6.01f;
        static constexpr float    MOVEGYRO_KI = 0.0f;
        static constexpr float    MOVEGYRO_KD = 2.15f;
        static constexpr float    TURNGYRO_KP = 22.75f;
        static constexpr float    TURNGYRO_KI = 0.08f;
        static constexpr float    TURNGYRO_KD = 0.25f;
    };

    /**
     * @brief Vendor PID + encoder defaults for the LEGO EV3 motor (LE-F TurnGyro variant).
     * @note Source: MiniR4DriveDC.h PID docstrings + MiniR4_DriveDC_Test.ino:52-62.
     *       LE-F = "LEGO EV3 Fast" Kp (22.75). LE-S variant uses Kp=5.25 — not used here.
     */
    struct LegoEV3
    {
        static constexpr uint16_t PPR = 360;
        static constexpr uint16_t RPM = 300;
        static constexpr float    MOVESYNC_KP = 0.002f;
        static constexpr float    MOVESYNC_KI = 0.0f;
        static constexpr float    MOVESYNC_KD = 0.004f;
        static constexpr float    MOVEGYRO_KP = 0.52f;
        static constexpr float    MOVEGYRO_KI = 0.0f;
        static constexpr float    MOVEGYRO_KD = 0.15f;
        static constexpr float    TURNGYRO_KP = 22.75f;   // LE-F variant
        static constexpr float    TURNGYRO_KI = 0.02f;
        static constexpr float    TURNGYRO_KD = 1.54f;
    };

    /**
     * @brief Single edit point to swap the chassis motor model.
     *
     * Default is LegoEV3 (Lofaix competition robot). Switch to MatrixTT for the
     * test rover. The flat constants below (MOTOR_PPR, MOVESYNC_K*, …) forward
     * to ActiveMotor::* so call sites do not change.
     *
     * @warning Swapping motors does NOT recalibrate MOTOR_BIAS_L/R — those are
     *          per-physical-chassis measurements and MUST be re-captured via
     *          Drive.calibrateMotorBias() after a swap. See ADR-011 + ADR-022.
     */
    using ActiveMotor = LegoEV3;

    constexpr uint16_t MOTOR_PPR    = ActiveMotor::PPR;
    constexpr uint16_t MOTOR_RPM    = ActiveMotor::RPM;
    constexpr float    MOVESYNC_KP  = ActiveMotor::MOVESYNC_KP;
    constexpr float    MOVESYNC_KI  = ActiveMotor::MOVESYNC_KI;
    constexpr float    MOVESYNC_KD  = ActiveMotor::MOVESYNC_KD;
    constexpr float    MOVEGYRO_KP  = ActiveMotor::MOVEGYRO_KP;
    constexpr float    MOVEGYRO_KI  = ActiveMotor::MOVEGYRO_KI;
    constexpr float    MOVEGYRO_KD  = ActiveMotor::MOVEGYRO_KD;
    constexpr float    TURNGYRO_KP  = ActiveMotor::TURNGYRO_KP;
    constexpr float    TURNGYRO_KI  = ActiveMotor::TURNGYRO_KI;
    constexpr float    TURNGYRO_KD  = ActiveMotor::TURNGYRO_KD;
}
```

## Error Handling

- **Misconfigured `ActiveMotor` alias** (e.g., typo): compile error from the unresolved type — fails fast at build, not at runtime. Acceptable.
- **Forgot to re-run `calibrateMotorBias()` after chassis swap**: silent — chassis drifts. Mitigation: Doxygen warning at the alias declaration + ADR-022 runbook step. Bulletproof static_assert was declined as out of scope.
- **TT PID values applied to LEGO motor (or vice versa) due to wrong alias**: chassis moves badly. Detected only by operator observation. Acceptable risk for a single-line edit.

## Testing Strategy

**Levels:** smoke (Python pytest), compile (Arduino), grep audit (manual).

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `uv run pytest tools/path_planner/tests/` | Smoke | All existing tests pass (Python unaffected by Arduino-side change). |
| TC2 | `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` | Compile | Compiles clean. If `arduino-cli` not installed, fall back to TC3+TC4. |
| TC3 | `grep -rn "Config::MOTOR_PPR\|Config::MOTOR_RPM\|Config::MOVESYNC\|Config::MOVEGYRO\|Config::TURNGYRO" WRO2026/` | Audit | Same callsite count and locations as on `main` — no `Drive.cpp` diff. |
| TC4 | `grep -rn "TT-motor\|TT motor\|TT motors" WRO2026/ docs/` | Audit | Only references remaining are inside ADR-011 (now reframed) and ADR-022 (intentional). No bare "TT motor" claims as the "current" default. |
| TC5 | Manual diff of `Drive.cpp` against `main` | Audit | Zero changes. |
| TC6 | Flip `using ActiveMotor = MatrixTT;` locally, rebuild, revert | Compile | Compiles cleanly both ways. Confirms toggle mechanism actually toggles. |

**Run command:** `uv run pytest tools/path_planner/tests/ && arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` (or grep fallback per TC3-TC5).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Add `MatrixTT` / `LegoEV3` structs + `ActiveMotor` alias + flat forwarders | — | med | `WRO2026/Config.h` | Insert the two `struct` definitions and `using ActiveMotor = LegoEV3;` immediately above the existing `// Motor profile + PID gains` section. Replace the existing `constexpr uint16_t MOTOR_PPR = 545;` block (lines ~101-126) with the eleven one-line forwarders shown in the Data Model section. Add the Doxygen `@warning` about `calibrateMotorBias()` to the `ActiveMotor` declaration. Vendor source citation (`MiniR4DriveDC.h` docstrings + `MiniR4_DriveDC_Test.ino:52-74`) in each struct's Doxygen. Satisfies AC 1, 2, 3. |
| T2 | Reword TT-flavored comments in `Config.h` | T1 | low | `WRO2026/Config.h` | Update: file header doxygen lines 16-19 (drop "target TT-motors", reframe as "tuned via `ActiveMotor` selector"); block comment lines 91-99 (rename "Motor profile + PID gains (TT-motor)" → "Motor profile + PID gains (sourced from `ActiveMotor`)" and update the PID-default citations to mention both TT and LE rows from the vendor docstrings); brief comments at line 101 / 104 (drop "TT motor" — say "active motor profile"); block comment lines 181-193 (drop "TT motors" → "DC drive motors"); block comment lines 552-567 (drop "TT-motors of the same model" → "DC motors of the same model"); line 577 (drop "on TT motors" → "on typical DC drive motors"). Preserve all rationale and structure. Satisfies AC 5. |
| T3 | Reword ADR-011 line 12 | — | low | `docs/decisions/011-motor-bias-strategy-b.md` | Change "test rover with TT motors (current dev target, Config.h is tuned for this)" to acknowledge LEGO EV3 (Lofaix) is the now-active `ActiveMotor`, with TT remaining the alternate setting for the test rover. Keep the cross-chassis re-calibration warning verbatim. Add a one-line "See also: ADR-022" pointer at the bottom of the ADR. Rule statement unchanged. Satisfies AC 6. |
| T4 | Generalize ADR-017 line 23 | — | low | `docs/decisions/017-sync-plus-tshape-default-profile.md` | Change "the two TT motors" to "the two drive motors". No other change. Satisfies AC 7. |
| T5 | Update Roadmap.md Phase 7 narrative + tables | — | low | `Roadmap.md` | Edits: (a) lines 33-41 — rewrite the "Two physical chassis" paragraph to say the `Config.h` motor swap landed in this PR (cite issue #15), and Phase 7 now only does `calibrateMotorBias()` re-run + `LINE_KP/KD` retune + B6.1 ten-trial battery on Lofaix; (b) line 230 exit gate — reword from "Config.h updated for EV3 PPR / RPM (issue #15)" to "ActiveMotor verified set to LegoEV3 on flashed Lofaix (issue #15 closed in prep)"; (c) line 248 — append "(done in #15 PR)" to the P7.0b row description and update the status column if there is one; (d) line 356 risk row #12 — keep the risk text but reword the mitigation column from "static_assert or per-chassis preprocessor flag" to "single-line `using ActiveMotor = ...;` toggle in Config.h + Doxygen warning about MOTOR_BIAS (see ADR-022)". Satisfies AC 8. |
| T6 | Create ADR-022 | T1 | low | `docs/decisions/022-dual-motor-profile.md` | New file using the standard ADR template per CLAUDE.md "Documenting design decisions (ADRs)" section. Status: Implemented. Date decided: 2026-05 (Phase 2). Rule: "Motor-type-dependent constants (PPR/RPM/PID gains) live in per-motor `constexpr struct`s inside `Config`; the active chassis selects one via `using ActiveMotor = ...;`. To swap chassis, edit only that alias." Why: cite issue #15, the dual-chassis fact (test rover + Lofaix), the scattered TT assumptions before this PR. How to apply: list the swap protocol (edit alias → reflash → re-run `Drive.calibrateMotorBias()` → re-tune `LINE_KP/KD`). Trigger to revisit: add a third motor model, or move bias scalars into the profile. See also: ADR-011 (per-chassis bias), ADR-017 (drive-pair asymmetry). Cite vendor source files. Stay ~30-60 lines. Satisfies AC 9. |
| T7 | Add ADR-022 to DECISIONS.md index | T6 | low | `DECISIONS.md` | One new line in the appropriate section (likely the Drive/Config section) referencing ADR-022. Bump the status-summary table by one row. Match the existing index entry style exactly. Satisfies AC 10. |
| T8 | Verify build + grep audit | T1, T2 | low | (no file changes) | Run `uv run pytest tools/path_planner/tests/` (TC1), then `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` if available (TC2). Run TC3 + TC4 + TC5 grep audits and confirm `Drive.cpp` byte-identical to `main`. Run TC6 by locally flipping the alias to `MatrixTT`, rebuilding (or recompiling Config.h via a quick standalone test), then reverting. Log results inline in the implementation report. Satisfies AC 11, 12. |

**Dependency graph:**
- T1 has no blocker (Config.h structural change).
- T2 depends on T1 (same file, sequential edit).
- T3, T4, T5, T6 can all run in parallel with T1/T2 (different files).
- T6 depends on T1 only insofar as the ADR cites the struct names — minor dependency, can run in parallel if struct names are pinned in this spec.
- T7 depends on T6 (must exist before being indexed).
- T8 depends on T1 + T2 (Config.h must be final before audit).

Parallelizable: {T3, T4, T5, T6} after T1 starts. T7 strictly after T6.

## Notes for Implementer

- **Do not introduce a new `MotorProfile` sub-namespace** — user explicitly chose flat `Config::` scope. Structs are defined directly inside `namespace Config { ... }`.
- **Preserve byte-for-byte:** the `MOTOR_LEFT/RIGHT_ID`, `MOTOR_LEFT/RIGHT_REVERSE` block + `static_assert`s at `Config.h:30-89`. Motor reversal is a chassis-wiring property; do not touch.
- **Do not touch `Drive.cpp`.** If anything makes it look like Drive.cpp needs an edit, that's a sign the forwarders are wrong.
- **Source-citation discipline:** every PID value pasted into the structs must trace to a specific line in either `MiniR4DriveDC.h` (PID docstrings on `setMoveSyncPID` / `setMoveGyroPID` / `setTurnGyroPID`) or the vendor example `MiniR4_DriveDC_Test.ino` (lines 52-74). See `feedback_source_cited_proof.md`.
- **Doxygen `@warning` on the alias:** must call out that `MOTOR_BIAS_L/R` do not auto-flip with a chassis swap. This is the "minimal compile-time guard" the user picked.
- **Roadmap edit caution:** `Roadmap.md` is long. Re-read lines 33-41, 219-260, and 350-360 before editing to avoid orphaning forward-references. The task table on lines 246-252 has a header — preserve the column alignment.
- **ADR-022 line budget:** stay 30-60 lines per the CLAUDE.md ADR template. The "Why" section is where the dual-chassis context lives; the "How to apply" section is the runbook.
- **Rollback plan:** if anything breaks at compile or behaviorally, the rollback is a single `git revert` of this PR. Drive.cpp untouched means there's no half-state risk.
- **Commit per task:** match the project's git-workflow rule — one commit per task (so eight commits, plus a final "audit + verify" commit if T8 surfaces anything).
