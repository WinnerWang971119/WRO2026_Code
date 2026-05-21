# WRO 2026 Mosaic Masters — Roadmap

This roadmap is the sequence axis of the project. Phases are linear —
finish one, start the next, as fast as possible. Internal phase timing
is deliberately not budgeted; only the external WRO competition dates
are fixed. The roadmap schedules the work whose shape is described in
[ARCHITECTURE.md](./ARCHITECTURE.md) and whose rationale is recorded in
the ADRs under [docs/decisions/](./docs/decisions/) (indexed in
[DECISIONS.md](./DECISIONS.md)). When this document and either of those
disagree, the architecture / decisions docs win and this file gets
updated.

## 1. Big Picture

The robot competes in WRO RoboMission Senior 2026 ("Mosaic Masters"): one
2-minute autonomous run on a fixed field, scoring up to 233 points by
delivering three tools, placing colored mosaic tiles into a randomized 3×3
frame, delivering colored cement blocks, and returning the brick trowel to
the start zone. Three deadlines: regional `2026-07-21`, national
`2026-08-24`, world final `2026-11-30`. The team is 2-3 people running
two parallel tracks: mechanical (chassis, manipulator, sensor mounts) and
software (modules in [ARCHITECTURE.md](./ARCHITECTURE.md)).

**Status snapshot.** The **Foundation** (original Phases 0-2) is closed
and shipped to `main`. The drivebase facade, the line-follow +
intersection + custom 8-channel sensor stack, the offline Python path
planner, the on-robot `Localization` + `PathTracker`, and the Mission
operator DSL are all live. Issues #16 (8-ch line sensor) and #22
(junction PID freeze) have landed. Manipulator v0 mechanism is in
progress on the mech track; perception (camera) and the Tasks layer are
not yet started. Phase 3 is next.

**Two physical chassis.** Phases 3-6 run on the **test rover** (TT
motors; switch the alias to `MatrixTT` in `Config.h`). The **competition
robot Lofaix** (LEGO EV3 motors) must be assembled and brought up before
Phase 7's code freeze. The EV3 model swap landed in the issue-#15 PR:
`Config.h` now defaults to `using ActiveMotor = LegoEV3;`
(see [ADR-022](./docs/decisions/022-dual-motor-profile.md)). Bring-up
now only requires: re-running `Drive.calibrateMotorBias()` on Lofaix
(TT scalars do not transfer —
see [ADR-011](./docs/decisions/011-motor-bias-strategy-b.md)),
re-tuning `LINE_KP` / `LINE_KD` on Lofaix's geometry, and re-passing
the Phase 6 B6.1 ten-trial battery on Lofaix to confirm the regional
target still holds.

## 2. Phase Sequence

| Phase | Goal | Status |
|---|---|---|
| **Foundation** | Drive / LineFollower / Intersection / WROLineArray / FieldMap / Localization / PathTracker / Mission DSL / Python path planner / CI | shipped |
| 3 | Hardware verification of the shipped stack + real 2026 field encoded | next |
| 4 | Perception: one camera module (3×3 mosaic + single-cell color) — runs in parallel with mech building manipulator v0 | pending |
| 5 | Manipulator software on v0 mechanism + first end-to-end Task | pending |
| 6 | Tasks layer + Mission scheduler + recoveries + full 2-minute run | pending |
| 7 | Lofaix bring-up + regional sprint (compete `2026-07-21`) | pending |
| 8 | National refinement (compete `2026-08-24`) | pending |
| 9 | World final prep (compete `2026-11-30`) | pending |

Phases are linear — finish one, start the next. Phase 3 is the
load-bearing one: it tests the assumption that the simulator-verified
stack actually works on the real chassis. Only external competition
dates are fixed; internal phase duration is "as fast as possible," not
budgeted.

## 3. Foundation (shipped, retained for context)

The work in `main` at the start of Phase 3 is captured by:
- [ARCHITECTURE.md](./ARCHITECTURE.md) — module map as it stands today.
- [DECISIONS.md](./DECISIONS.md) + [docs/decisions/](./docs/decisions/) — every load-bearing decision (ADR-001 through ADR-020).
- [docs/archive/plans/](./docs/archive/plans/) — the seven plan docs whose work landed.
- [docs/archive/](./docs/archive/) — Phase 1 verification, Phase 2 PR1/PR2 audits, the original PATH_PLANNER_SPEC.

Foundation deliverables:

| Module | Status | Source of truth |
|---|---|---|
| `Drive` (fluent builders, queue, T-shape, bias) | shipped | ADRs 011-017 |
| `LineFollower` (P+D, distance gate, junction freeze) | shipped | ADRs 013, 018, 020 |
| `Intersection` (rising-edge, JunctionType, edge counter) | shipped | — |
| `WROLineArray` (8-ch over MXColor I2C) | shipped | ADR-019 |
| `Localization` (single source of truth) | shipped | ADR-005 |
| `FieldMap` (header-only over routes.h) | shipped | — |
| `routes.h/.cpp` (auto-generated) | shipped | — |
| `PathTracker` (FSM + fluent `goTo`) | shipped | — |
| `Mission` (operator DSL) | shipped | — |
| `EventTrace` (OLED + Serial heartbeat) | shipped | ADR-010 |
| `WROMath`, `StateMachine` | shipped (latter unused) | — |
| Python path planner pipeline | shipped | `tools/path_planner/README.md` |
| GitHub Actions CI | shipped | `.github/workflows/` |

## 4. Phase 3 — Hardware Verification + Real-Field Encoding

**Goal:** Prove the shipped stack works on the real chassis (not just the
simulator), and replace the toy A-G+P1 graph with the actual 2026 field
layout. This phase is the moment of truth — every later phase assumes the
foundation is reliable on hardware.

**Exit criteria (hard gates):**
- [ ] Real 2026 field measured (via `pick_xy.py`), encoded in
      `field_data.py`, `routes.h/.cpp` regenerated, `map.png` reviewed.
- [ ] `Drive.driveDist(cm)` accuracy ±2 cm on a 2 m straight (SYNC + tShape default).
- [ ] `Drive.turnDeg(90)` accuracy ±2° over 20 trials.
- [ ] Line cruise ±5 mm lateral error at NORMAL speed on a 2 m straight.
- [ ] Intersection detection ≥19/20 on T- and +-intersections at NORMAL.
- [ ] `PathTracker.goTo(name)` traverses any-to-any pair on the toy
      printout AND on the real-field printout (≤1 missed intersection per
      10 traversals).
- [ ] Lost-line and missed-intersection recoveries trigger and bound in lab tests.
- [ ] OLED `EventTrace` + Serial heartbeat readable end-to-end during a 60 s drive.

**Tracks running in parallel:**
- Mechanical: manipulator v0 build (lift + gripper + arm geometry). Mount color sensor and camera in *placeholder* positions so geometry can be validated; final mounts come in Phase 4/5.
- Software: hardware verification + real-field encoding only. No new modules.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P3.1 | Bench-test `Drive.driveDist` / `turnDeg` accuracy on real chassis | sw | — | calibrated drive |
| P3.2 | Run `calibrateMotorBias()`; hand-edit `Config::MOTOR_BIAS_*` | sw | P3.1 | matched motors |
| P3.3 | Tune `LINE_KP` / `LINE_KD` on practice strip; verify junction freeze | sw | P3.1 | tuned line PID |
| P3.4 | Run `pick_xy.py` against the real WRO 2026 mat photo | sw | — | real coordinates |
| P3.5 | Rewrite `field_data.py` with real 2026 nodes, edges, obstacles | sw | P3.4 | real field encoded |
| P3.6 | Regenerate `routes.h/.cpp`, review `map.png`, commit together | sw | P3.5 | real routes shipped |
| P3.7 | Hardware verify `PathTracker` end-to-end (toy + real-field printout) | sw | P3.3, P3.6 | nav metrics |
| P3.8 | Verify lost-line + missed-intersection recoveries in lab | sw | P3.7 | recovery v1 |
| P3.9 | Manipulator v0 build (mech track, parallel) | mech | — | mech v0 |

**Pull-forward candidates from PHASE2_DEFERRED:** If P3.7 exposes drift on
the blind edges of the real field, consider pulling forward calibration
nodes + blind-cost tiers + `wall_zero` (ADR-aligned with Phase 2.5
deferred work) into a Phase 3.5 sub-task before Phase 4 starts.

## 5. Phase 4 — Perception

**Goal:** Read individual cell colors AND the 3×3 mosaic pattern
reliably under workshop lighting — both through a **single camera
module**. Per [ADR-021](./docs/decisions/021-camera-as-sole-color-sensor.md)
there is no dedicated color sensor. The camera module runs independently
of the manipulator mechanism, so it lands while mech finishes manipulator v0.

**Exit criteria (hard gates):**
- [ ] Camera hardware selected and decision logged in [DECISIONS.md](./DECISIONS.md) (ADR-006 implementation closes).
- [ ] Camera module's single-cell color read returns `{Y, B, G, W, NONE}` correctly on ≥18/20 samples per color under workshop lighting (replaces what was the ColorReader benchmark, B4.1).
- [ ] Camera module's `capturePattern(out[9])` classifies all 9 cells correctly on ≥9/10 captures under workshop lighting (B4.2).
- [ ] Module exposes the `update()` cooperative-tick interface and respects Rule 1 (no `delay()` in tick paths).
- [ ] One calibration table in `Config.h` covers both use cases (single-cell and 3×3).

**Tracks running in parallel:**
- Mechanical: finalize manipulator v0; design permanent camera mount that serves both 3×3-mosaic view AND single-cell view at the cement-block pick point (geometry locks in Phase 5).
- Software: camera module (`CameraMosaic` / `CameraColor` / `Camera` — name decided when written) covering both operations; one calibration baseline committed to `Config.h` like `LINE_CALIB_*`.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P4.1 | Decide camera (HuskyLens vs raw) — write the ADR-006 implementation closure | sw | P3.7 | camera selection |
| P4.2 | Mount camera in placeholder position for calibration (both use cases) | both | P4.1 | placeholder mount |
| P4.3 | Camera module — `capturePattern(out[9])` for the 3×3 mosaic | sw | P4.1, P4.2 | mosaic read |
| P4.4 | Camera module — `readSingleColor() → ColorEnum` for cement / tile | sw | P4.3 | single-cell read |
| P4.5 | Workshop-lighting calibration table — single source for both reads | sw | P4.3, P4.4 | calibration data |
| P4.6 | Manipulator v0 final assembly + bench range-of-motion check | mech | P3.9 | mech v0 ready |

## 6. Phase 5 — Manipulator + First Task

**Goal:** Drive the manipulator v0 mechanism from software; build and run
one end-to-end Task (probably tool delivery as the simplest) that
composes PathTracker + Manipulator + camera module.

**Exit criteria (hard gates):**
- [ ] `Manipulator.setLift(h)` / `setGripper(angle)` / `setArm(angle)` non-blocking; reach commanded position within tolerance + timeout.
- [ ] Permanent color + camera mounts installed; calibration tables re-baselined.
- [ ] First `Tasks` FSM (tool delivery) reaches `isDone()` in ≥4/5 lab trials.
- [ ] Mission DSL gains a `.task("name", args)` verb that dispatches into the Tasks layer.

**Tracks running in parallel:**
- Mechanical: finalize permanent sensor mounts; lock manipulator geometry; spare-parts ordering.
- Software: `Manipulator.h/.cpp` real implementation; `Tasks.h/.cpp` skeleton + first task FSM; Mission DSL extension.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P5.1 | Replace `Manipulator` stub with real `setLift/setGripper/setArm` against v0 | sw | P4.7 | manipulator sw v1 |
| P5.2 | Tune servo ranges, neutral positions, grip force | both | P5.1 | tuned manipulator |
| P5.3 | Install permanent camera mount (serves both 3×3 + single-cell views); re-baseline calibration | both | P5.2 | locked geometry |
| P5.4 | `Tasks.h/.cpp` framework (`start/update/isDone/abort` per scoring action) | sw | P5.1 | tasks framework |
| P5.5 | First task FSM — tool delivery (drive → grip → drive → drop → verify) | sw | P5.4 | tool task |
| P5.6 | Mission DSL `.task("name", args)` verb + dispatcher | sw | P5.4 | DSL extension |
| P5.7 | End-to-end tool delivery on test mat, 5 trials | both | P5.5, P5.6 | first scoring run |

## 7. Phase 6 — Tasks Layer Complete + Mission Scheduler + Recoveries

**Goal:** Every scoring action implemented as a Task; Mission scheduler
picks task order; 2-minute time guard + return-to-start works; the third
named failure recovery (failed grip) lands.

**Exit criteria (hard gates):**
- [ ] `Tasks` covers: mosaic pattern read, mosaic tile placement (×9 attempts), cement delivery, brick-trowel return, tool delivery (already in Phase 5).
- [ ] `Mission` scheduler runs greedy score-per-second ordering and aborts overrunning tasks (ADR-008 implementation closes).
- [ ] 2-minute time guard triggers at `t = 100 s` to start return-to-start.
- [ ] Failed-grip recovery (ADR-009 third branch) triggers and yields in lab tests.
- [ ] Full 2-minute run on practice field scores meeting the **regional target**: mosaic placement working (≥6 of 9 tiles) + cement delivery + tool delivery, succeeding on ≥8 of 10 lab attempts.

**Tracks running in parallel:**
- Mechanical: tuning only — weights, grip pads, no rebuilds.
- Software: all remaining Task FSMs, Mission scheduler, recovery branches.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P6.1 | Tasks — mosaic pattern read (uses CameraMosaic) | sw | P5.5 | mosaic read task |
| P6.2 | Tasks — mosaic tile placement (composes pattern + manipulator) | sw | P6.1 | mosaic place task |
| P6.3 | Tasks — cement delivery (uses camera `readSingleColor()` for block ID) | sw | P5.5 | cement task |
| P6.4 | Tasks — brick-trowel return | sw | P5.5 | return task |
| P6.5 | Mission scheduler — greedy score/sec + per-task estimates | sw | P6.2, P6.3, P6.4 | scheduler |
| P6.6 | Mission 2-minute time guard + forced return-to-start at t=100s | sw | P6.5 | time guard |
| P6.7 | Failed-grip recovery branch in Tasks | sw | P5.1 | recovery 3 |
| P6.8 | Ten full-run scoring sessions; require ≥8/10 success on regional-target scope | both | P6.6, P6.7 | run metrics |

## 8. Phase 7 — Lofaix Bring-up + Regional Sprint

**Compete date:** `2026-07-21` (regional, fixed by WRO).

**Goal:** Move the working software stack from the test rover onto the
competition robot **Lofaix** (LEGO EV3 motors), confirm the regional
target still holds on Lofaix, then compete at regional. No code rewrites
in the sprint window — only constants tunable.

**Exit criteria (hard gates):**
- [ ] Lofaix mechanically assembled and powered (mech track must deliver before this phase starts).
- [ ] `Config::ActiveMotor` confirmed as `LegoEV3` on flashed Lofaix (constants landed in issue #15 PR; see [ADR-022](./docs/decisions/022-dual-motor-profile.md)).
- [ ] `Drive.calibrateMotorBias()` re-run on Lofaix, scalars committed.
- [ ] `LINE_KP` / `LINE_KD` re-tuned on Lofaix.
- [ ] B6.1 ten-trial battery passes on Lofaix at the same 8/10 success rate.
- [ ] Code frozen before venue practice. Only color baselines and PID gains may change after freeze.
- [ ] On-site lighting calibration recorded for the camera module (single table, both use cases).
- [ ] Battery cells charged, spares present, `setBattCell(2)` confirmed.
- [ ] At least one full practice run executed at the venue before competition.

**Tracks running in parallel:**
- Mechanical: Lofaix assembly must complete BEFORE phase entry; in-phase mech work is visual inspection, fastener check, spare parts staging.
- Software: chassis swap recalibration, re-run regional benchmark on Lofaix, then re-baseline color thresholds against venue lighting.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P7.0a | Lofaix mech assembly complete | mech | (parallel to Phase 5/6) | Lofaix ready |
| P7.0b | Swap `Config.h` motor constants (PPR / RPM / reverse flags) for EV3 (done in #15 PR) | sw | P7.0a, issue #15 | EV3 Config |
| P7.0c | Re-run `calibrateMotorBias()` on Lofaix; commit new scalars | sw | P7.0b | Lofaix bias |
| P7.0d | Re-tune `LINE_KP` / `LINE_KD` on Lofaix | sw | P7.0c | Lofaix line PID |
| P7.0e | Re-run B6.1 ten-trial battery on Lofaix; confirm ≥8/10 success | both | P7.0d | Lofaix qualified |
| P7.1 | Code freeze, tag commit | sw | P7.0e | regional tag |
| P7.2 | Pack spares, batteries, cables | mech | — | go-bag |
| P7.3 | Venue practice run + on-site lighting calibration | both | P7.1 | tuned baselines |
| P7.4 | Compete | both | P7.3 | regional result |

**Sequence note:** P7.0a-P7.0e (Lofaix bring-up) should land early in
this phase so the bulk of the time goes to venue prep + compete. If
Lofaix mech slips, the bring-up tasks compress the freeze window, but
the regional compete date `2026-07-21` is fixed.

## 9. Phase 8 — National Refinement

**Compete date:** `2026-08-24` (national, fixed by WRO).

**Goal:** Convert regional learnings into measurable score gains; compete
at national.

**Exit criteria (hard gates):**
- [ ] Written post-mortem with the weakest task identified.
- [ ] Redundancy paths added to `FieldMap` for tasks that failed at regional.
- [ ] Recovery branches tightened (timeouts, bounded sweep widths).
- [ ] National run completed; result recorded.

**Tracks running in parallel:**
- Mechanical: address regional breakage / wear; reinforce weak joints.
- Software: post-mortem-driven fixes, `FieldMap` rev bump if topology changes, recovery tuning.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P8.1 | Regional post-mortem, identify weakest task | both | Phase 7 done | post-mortem |
| P8.2 | Add redundancy edges in `field_data.py` for risky paths; regen routes | sw | P8.1 | routes rev 2 |
| P8.3 | Tighten recovery timeouts in lost-line / missed-intersection / failed-grip | sw | P8.1 | recovery v2 |
| P8.4 | Pull forward Phase 2.5 deferred items if regional exposed drift (calibration nodes, blind-cost tiers, wall_zero) | sw | P8.1 | drift mitigation |
| P8.5 | Mechanical reinforcement on wear points | mech | P8.1 | hardened mech |
| P8.6 | Ten full runs; require improvement vs regional B7.1 baseline | both | P8.3 | improved metrics |
| P8.7 | Code freeze for national, tag commit | sw | P8.6 | national tag |
| P8.8 | Compete at national | both | P8.7 | national result |

## 10. Phase 9 — World Final Prep

**Compete date:** `2026-11-30` (world final, fixed by WRO).

**Goal:** Maximize score per run by re-ordering the task queue, shaving
seconds, and hardening the robot for travel and repeated handling.

**Exit criteria (hard gates):**
- [ ] Documented A/B experiments showing the chosen task order beats alternatives on score/run.
- [ ] Robot survives a simulated travel cycle (drop test from carry height, vibration soak) with no functional loss.
- [ ] Spare-parts kit complete (motors, servos, sensor boards, batteries, fasteners).
- [ ] Optional: replay/log tooling shipped if [ADR-010](./docs/decisions/010-lightweight-logging-only.md) is revisited.

**Tracks running in parallel:**
- Mechanical: robustness, travel case, spares inventory.
- Software: scheduler tuning, scoring optimization, optional logging.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P9.1 | A/B task-order experiments via Mission config | sw | P8.8 | best ordering |
| P9.2 | Shave seconds — speed up turns, increase line cruise speed | sw | P9.1 | faster run |
| P9.3 | Mechanical robustness pass; travel case | mech | P8.5 | travel-ready |
| P9.4 | Spare-parts kit assembled and inventoried | mech | — | spares kit |
| P9.5 | (Optional) replay/log tooling, gated on team capacity | sw | — | replay tool |
| P9.6 | Code freeze for world final, tag commit | sw | P9.2 | worlds tag |
| P9.7 | Compete at world final | both | P9.6 | worlds result |

## 11. Open Decisions Log

Closed decisions are recorded in [DECISIONS.md](./DECISIONS.md). The
remaining open items below must be closed before their phase's software
can lock in.

| Decision | Owner | Decide-by | Options | Notes |
|---|---|---|---|---|
| Camera | sw lead | Phase 4 entry | HuskyLens (built-in color recog, faster) ; raw camera + on-device pixel classify | Drives the camera module impl; lighting calibration required either way (ADR-006). Camera is the **sole** color sensor — no dedicated ColorReader (ADR-021). |
| Manipulator mechanism geometry | mech lead | Phase 5 entry | TBD lift / gripper / arm geometry on 3 RC channels | Locks `Manipulator.setLift/setGripper/setArm` parameters (ADR-007). |

### Closed in Foundation

| Decision | Closed | Outcome |
|---|---|---|
| Line sensor | 2026-05 | Custom `WROLineArray` — 8 ch / board on MXColor I2C boards, 1–2 boards (ADR-019). |
| Line PID source | 2026-05 | Own P+D in `LineFollower.update()`, junction-freeze added in issue #22 (ADR-020). |
| Motor / DriveDC API | 2026-05 | `MiniR4.DriveDC` wrapped by `Drive` fluent-builder facade with FIFO queue + T-shape (ADRs 011-017). |
| STM32 firmware version | 2026-05 | ≥ v6.0 confirmed at boot (`MiniR4.begin()` halts if older). |

## 12. Risk Register

| # | Risk | Likelihood | Impact | Mitigation | Track |
|---|---|---|---|---|---|
| 1 | Manipulator slips on tool / tile / cement, repeatedly | M | H | Failed-grip recovery (ADR-009); grip-pad tuning in Phase 5/6; one retry then yield. | mech + sw |
| 2 | Manipulator v0 mechanism slips past Phase 5 entry | M | H | Phase 4 (perception) and Phase 5 (manipulator sw) are scoped to run independently so a mech slip only delays Phase 5; software can draft `Manipulator` against expected RC servo IDs from `Config` as soon as Phase 4 ships. | mech |
| 3 | Line sensor unreliable on competition mat surface | L | H | Two-layer calibration (firmware illumination + per-channel software baselines, ADR-019); re-baseline on-site (Phase 7.3). | sw |
| 4 | Camera lighting shift between practice and venue | H | H | On-site re-baseline in Phase 7.3 / Phase 8.6; geometry fixed by line-follow stop, not odometry (ADR-006). | sw |
| 5 | 2S battery sag causes brownout under high motor draw | M | M | `MiniR4.PWR.setBattCell(2)` correct; charged spares in go-bag; monitor `MiniR4.PWR` voltage on OLED. | mech |
| 6 | `field_data.py` revision mismatch vs official 2026 layout | M | H | Encoded from real WRO 2026 mat in P3.4/P3.5 via `pick_xy.py`; SHA256-stamped `routes.h` banner catches stale headers; review against official PDF before each competition freeze. | sw |
| 7 | A blocking task breaks the 2-minute time guard | M | H | Iron rule 1 (no `delay()` in `update()`); Mission scheduler aborts overrunning tasks (ADR-008). | sw |
| 8 | Practice field arrives late, blocking Phase 6 / Phase 8 tuning | M | M | Use printed reduced-scale field (P3.4) until full-scale arrives; budget tuning into Phase 8 buffer weeks. | mech |
| 9 | Encoder slip / dead-reckoning drift during a long task | M | M | Architecture rejects pure dead-reckoning (ADR-001); `Localization` re-anchors at every intersection. If Phase 3 verification exposes blind-edge drift, pull forward Phase 2.5 calibration nodes / `wall_zero`. | sw |
| 10 | `Localization` written from outside the module | L | H | Coding criteria checklist in [ARCHITECTURE.md](./ARCHITECTURE.md); ADR-005; enforced in code review. | sw |
| 11 | Lofaix (competition robot) not mech-ready by Phase 7 entry | M | H | Mech track must deliver Lofaix as a parallel deliverable during Phases 5-6. If slip, P7.0a-P7.0e compress the freeze window, but the `2026-07-21` regional date is fixed. Worst case: compete on the test rover with TT-tuned `Config.h`. | mech |
| 12 | TT-tuned `Config.h` constants silently misapplied to Lofaix EV3 motors | L | H | Single-line `using ActiveMotor = ...;` toggle in `Config.h` makes the swap one explicit edit + reflash; Doxygen `@warning` on the alias declaration reminds the operator to re-run `Drive.calibrateMotorBias()` after the swap (see [ADR-022](./docs/decisions/022-dual-motor-profile.md)). | sw |

## 13. Test Fixture & Measurement Plan

Each phase exits on numeric, repeatable benchmarks. Tests are run on
hardware. IDs are referenced from each phase's "Exit criteria" gates.

| Phase | Benchmark ID | Setup | Pass criterion |
|---|---|---|---|
| 3 | B3.1 | `Drive.driveDist(200).setMode(SYNC).tShape()` on flat floor | ±2 cm at endpoint over 10 trials |
| 3 | B3.2 | `Drive.turnDeg(90)` from rest, IMU yaw logged | Final heading within ±2° on 19/20 turns |
| 3 | B3.3 | 2 m line on white substrate, cruise speed | Lateral error ±5 mm at midpoint; no loss-of-line |
| 3 | B3.4 | T- and +-intersections, 20 trials each | ≥19/20 correct detections per type |
| 3 | B3.5 | `PathTracker.goTo(name)`, 10 random A→B traversals on real-field printout | ≤1 missed intersection across all 10 runs |
| 3 | B3.6 | Inject lost-line + missed-intersection in lab | Recoveries trigger and bound within timeout |
| 4 | B4.1 | Each color sample (Y/B/G/W) shown to the camera's `readSingleColor()` 20× under workshop lighting | ≥18/20 correct per color |
| 4 | B4.2 | Mosaic pattern card under workshop lighting, robot at fixed stop | All 9 cells correct on ≥9/10 captures |
| 5 | B5.1 | Manipulator `setLift / setGripper / setArm` range sweep | Reaches commanded position within tolerance + timeout |
| 5 | B5.2 | Tool delivery Task end-to-end on test mat, 5 trials | ≥4/5 complete with score awarded |
| 6 | B6.1 | Full 2-minute run on practice field, 10 trials | ≥8/10 succeed on regional-target scope (mosaic ≥6/9 tiles + cement + tool) |
| 6 | B6.2 | Inject failed-grip in lab | Retry-once then yield, scheduler advances |
| 7 | B7.1 | Venue practice run after lighting recalibration | One full run completed; matches B6.1 success rate |
| 8 | B8.1 | Ten full runs after post-mortem fixes | Mean score strictly higher than B7.1 baseline |
| 9 | B9.1 | A/B task-ordering experiments via Mission config, 5 runs each | Chosen ordering beats alternatives on mean score/run |

## 14. Glossary

Maps WRO 2026 Mosaic Masters terminology to the code modules in
[ARCHITECTURE.md](./ARCHITECTURE.md). When unsure which module owns a
concept, check this table first.

| WRO term | Code module |
|---|---|
| Field (printed playing surface with black lines) | `field_data.py` → `routes.h` → `FieldMap` |
| Intersection / waypoint (line crossing used as checkpoint) | `Intersection`, `Localization` |
| Mosaic frame (3×3 fixture holding the pattern) | camera module, `Tasks` |
| Mosaic tile (single colored piece placed into one cell) | `Tasks`, `Manipulator` |
| Tools (the three deliverable items) | `Tasks`, `Manipulator` |
| Cement blocks (colored blocks delivered to deposit zones) | `Tasks`, camera module (`readSingleColor`), `Manipulator` |
| Brick trowel (returned to start zone at end) | `Tasks`, `Mission` |
| Deposit zones (color-matched drop locations) | `field_data.py` / `routes.h`, `Tasks` |
| 2-minute run (full autonomous round, scored up to 233) | `Mission` |
