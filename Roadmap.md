# WRO 2026 Mosaic Masters — Roadmap

This roadmap is the time axis of the project. It schedules the work whose
shape is described in [ARCHITECTURE.md](./ARCHITECTURE.md) and whose rationale
is recorded in [DECISIONS.md](./DECISIONS.md). When this document and either
of those disagree, the architecture / decisions docs win and this file gets
updated.

## 1. Big Picture

The robot competes in WRO RoboMission Senior 2026 ("Mosaic Masters"): one
2-minute autonomous run on a fixed field, scoring up to 233 points by
delivering three tools, placing colored mosaic tiles into a randomized 3x3
frame, delivering colored cement blocks, and returning the brick trowel to
the start zone. We have three deadlines — regional on `2026-07-21`, national
on `2026-08-24`, world final at the end of November `2026-11-30`. The team is
2-3 people running two parallel tracks: a mechanical track (chassis,
manipulator, sensor mounts) and a software track (modules in
[ARCHITECTURE.md](./ARCHITECTURE.md)). The hard constraint at `2026-05-03` is
that nothing is built yet — no chassis, no manipulator, no software — so
Phase 0 is pure hardware bring-up and the open decisions in
[DECISIONS.md](./DECISIONS.md) must be closed before software can lock in.

## 2. Timeline at a Glance

| Phase | Dates | Goal | Must-ship-by |
|---|---|---|---|
| 0 | `2026-05-03` -> `2026-05-17` | Hardware bring-up + close open decisions | `2026-05-17` |
| 1 | `2026-05-17` -> `2026-06-07` | Drive base follows lines and turns | `2026-06-07` |
| 2 | `2026-06-07` -> `2026-06-21` | Robot navigates from any node to any node | `2026-06-21` |
| 3 | `2026-06-21` -> `2026-07-05` | Color, mosaic camera, manipulator, first task | `2026-07-05` |
| 4 | `2026-07-05` -> `2026-07-19` | Full mission with scheduler + recovery | `2026-07-19` |
| 5 | `2026-07-19` -> `2026-07-21` | Regional sprint, on-site calibration | `2026-07-21` |
| 6 | `2026-07-22` -> `2026-08-24` | Post-regional refinement, national run | `2026-08-24` |
| 7 | `2026-08-25` -> `2026-11-30` | World final preparation | `2026-11-30` |

- TimeLine can change and you don't have to take it too seriously.

## 4. Phase 1 — Drive & Line Following (`2026-05-17` -> `2026-06-07`, 3 weeks)

**Goal:** Robot follows a single black line at competition speed, detects
intersections reliably, and turns 90 degrees accurately using the IMU.

**Exit criteria (hard gates to next phase):**
- [ ] Follows a 2 m straight line within +/-5 mm lateral error at cruise speed.
- [ ] Detects T-intersections and +-intersections at >=95% over 20 trials each.
- [ ] 90-degree turn lands within +/-3 degrees of target heading (IMU yaw).
- [ ] No `delay()` calls inside any `update()` path (per ARCHITECTURE.md rule 1).
- [ ] `Drive`, `LineFollower`, `Intersection` modules compile standalone.

**Tracks running in parallel:**
- Mechanical: mount line sensor at front, mount color sensor in reading position, manipulator v0 fabrication starts.
- Software: implement `WROChassis` wrapping `MiniR4.DriveDC`; `Drive.turnDeg` over `MoveGyro`; `LineFollower` PID; `Intersection` detector; OLED event trace.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P1.1 | Mount line sensor at front, set ride height | mech | P0.7 | sensor mounted |
| P1.2 | Implement `WROChassis` / `Drive` wrapping `MiniR4.DriveDC` | sw | P0.6 | Drive module |
| P1.3 | `Drive.turnDeg` using IMU yaw (`MoveGyro`-family) | sw | P1.2 | turn primitive |
| P1.4 | `LineFollower.update()` PID, no `delay()` | sw | P1.1, P1.2 | line PID |
| P1.5 | Tune `KP_LINE`, `KD_LINE` on practice strip | sw | P1.4 | tuned gains |
| P1.6 | `Intersection.isAtIntersection()` from sensor array | sw | P1.4 | intersection event |
| P1.7 | Manipulator v0 build (lift + gripper + arm) | mech | P0.8 | mech v0 |
| P1.8 | OLED event trace for line/intersection/turn events | sw | P1.6 | debug visibility |

**Benchmarks for exit:** see Test Fixture Plan section 13 — B1.1, B1.2, B1.3.

## 5. Phase 2 — Map & Navigation (`2026-06-07` -> `2026-06-21`, 2 weeks)

**Goal:** Robot navigates from any node to any node on the official 2026
Mosaic Masters field graph using line following + intersection counting.

**Exit criteria (hard gates to next phase):**
- [ ] `FieldMap` encodes the official 2026 layout (nodes, edges, cardinal directions, costs).
- [ ] `Pathfinder` runs Dijkstra over a fixed array of size `MAX_NODES`, no dynamic allocation.
- [ ] `Localization.currentNode` and `heading` are the single source of truth — no other module writes them.
- [ ] `PathTracker` consumes a node sequence and emits drive/turn commands.
- [ ] On a printed reduced-scale field, `<=1` missed intersection per 10 traversals.

**Tracks running in parallel:**
- Mechanical: manipulator v0 assembled, bench-tested through full lift/gripper/arm range.
- Software: `FieldMap`, `Pathfinder`, `Localization`, `PathTracker`.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P2.1 | Encode 2026 field as `FieldMap` const arrays | sw | — | FieldMap rev1 |
| P2.2 | `Pathfinder.findPath` Dijkstra over fixed array | sw | P2.1 | path output |
| P2.3 | `Localization` — currentNode, heading, intersection/turn-done hooks | sw | P1.6, P1.3 | single source of truth |
| P2.4 | `PathTracker.start` / `update` / `isDone` | sw | P2.2, P2.3 | path executor |
| P2.5 | Print reduced-scale field for bench testing | mech | P2.1 | test mat |
| P2.6 | Bench manipulator v0 — full range, repeatability check | mech | P1.7 | mech ready |
| P2.7 | Any-node-to-any-node test, log misses on OLED | sw | P2.4, P2.5 | nav metrics |

**Benchmarks for exit:** see Test Fixture Plan section 13 — B2.1, B2.2.

## 6. Phase 3 — Perception & First Task (`2026-06-21` -> `2026-07-05`, 2 weeks)

**Goal:** Read mosaic colors and a single cell color reliably; execute the
first end-to-end task (tool delivery) using `Manipulator` primitives.

**Exit criteria (hard gates to next phase):**
- [ ] `ColorReader.read()` returns `{Y, B, G, W, NONE}` correctly on >=18/20 samples per color under workshop lighting.
- [ ] `CameraMosaic.capturePattern(out[9])` classifies all 9 cells correctly under workshop lighting.
- [ ] `Manipulator.setLift / setGripper / setArm` non-blocking, position reached within tolerance.
- [ ] `Tasks` (tool delivery) executes end-to-end >=4/5 trials.

**Tracks running in parallel:**
- Mechanical: finalize manipulator, finalize permanent color/camera mounts, fix sensor geometry to the navigation stop point per [DECISIONS.md](./DECISIONS.md) "Mosaic pattern reading".
- Software: `ColorReader`, `CameraMosaic`, `Manipulator`, first `Tasks` state machine.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P3.1 | Final manipulator build, weight + grip pads selected | mech | P2.6 | mech final |
| P3.2 | Permanent color sensor + camera mounts | mech | P3.1 | sensor geometry locked |
| P3.3 | `ColorReader.read()` -> ColorEnum | sw | P3.2 | color module |
| P3.4 | `CameraMosaic.capturePattern(out[9])` -> 9 cells | sw | P3.2 | mosaic read |
| P3.5 | `Manipulator.setLift/setGripper/setArm` + `update()` | sw | P3.1 | actuator module |
| P3.6 | `Tasks` — tool delivery state machine | sw | P3.5, P2.4 | first task |
| P3.7 | End-to-end tool delivery on test mat | both | P3.6 | first scoring run |

**Benchmarks for exit:** see Test Fixture Plan section 13 — B3.1, B3.2, B3.3.

## 7. Phase 4 — Full Mission & All Tasks (`2026-07-05` -> `2026-07-19`, 2 weeks)

**Goal:** All scoring tasks implemented, scheduled by `Mission`, with the
2-minute time guard and the three failure recoveries from
[DECISIONS.md](./DECISIONS.md).

**Exit criteria (hard gates to next phase):**
- [ ] `Tasks` covers tool delivery (3x), mosaic placement, cement delivery, brick-trowel return.
- [ ] `Mission` runs greedy score-per-second ordering and aborts overrunning tasks.
- [ ] 2-minute guard triggers at `t = 100 s` to start return-to-start.
- [ ] Lost-line, missed-intersection, failed-grip recoveries each trigger correctly in lab tests.
- [ ] Full 2-minute run scores `>=100` pts on practice field on 3 of 5 runs.

**Tracks running in parallel:**
- Mechanical: tuning only — weights, grip pads, no rebuilds.
- Software: `Tasks` for every scoring action, `Mission` scheduler, recovery branches.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P4.1 | `Tasks` — mosaic placement (uses `CameraMosaic` output) | sw | P3.4, P3.6 | mosaic task |
| P4.2 | `Tasks` — cement delivery (color match) | sw | P3.3, P3.6 | cement task |
| P4.3 | `Tasks` — brick-trowel return | sw | P3.6 | return task |
| P4.4 | `Mission` — task queue + greedy score/sec scheduler | sw | P4.1, P4.2, P4.3 | scheduler |
| P4.5 | `Mission` 2-minute time guard + return-to-start | sw | P4.4 | time guard |
| P4.6 | Lost-line recovery branch in `LineFollower` | sw | P1.4 | recovery 1 |
| P4.7 | Missed-intersection recovery in `PathTracker` | sw | P2.4 | recovery 2 |
| P4.8 | Failed-grip recovery in `Tasks` | sw | P3.6 | recovery 3 |
| P4.9 | Five full-run scoring sessions, log score per run | both | P4.5 | run metrics |

**Benchmarks for exit:** see Test Fixture Plan section 13 — B4.1, B4.2.

## 8. Phase 5 — Regional Sprint (`2026-07-19` -> `2026-07-21`, 2 days)

**Goal:** Compete at regional. No code rewrites — only constants tunable.

**Exit criteria (hard gates to next phase):**
- [ ] Code frozen on `2026-07-19`. Only color baselines and PID gains may change.
- [ ] On-site lighting calibration recorded for `ColorReader` and `CameraMosaic`.
- [ ] Battery cells charged, spares present, `setBattCell(2)` confirmed.
- [ ] At least one full practice run executed at the venue before competition.

**Tracks running in parallel:**
- Mechanical: visual inspection, fastener check, spare parts staged.
- Software: re-baseline color thresholds against venue lighting.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P5.1 | Code freeze, tag commit | sw | P4.9 | regional tag |
| P5.2 | Pack spares, batteries, cables | mech | — | go-bag |
| P5.3 | Venue practice run + lighting calibration | both | P5.1 | tuned baselines |
| P5.4 | Compete | both | P5.3 | regional result |

**Benchmarks for exit:** see Test Fixture Plan section 13 — B5.1.

## 9. Phase 6 — National Refinement (`2026-07-22` -> `2026-08-24`, ~5 weeks)

**Goal:** Convert regional learnings into measurable score gains; compete at
national.

**Exit criteria (hard gates to next phase):**
- [ ] Written post-mortem with the weakest task identified.
- [ ] Redundancy paths added to `FieldMap` for tasks that failed at regional.
- [ ] Failure recovery branches tightened (timeouts, bounded sweep widths).
- [ ] National run completed; result recorded.

**Tracks running in parallel:**
- Mechanical: address any breakage / wear from regional; reinforce weak joints.
- Software: post-mortem-driven fixes, `FieldMap` rev bump if topology changes, recovery tuning.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P6.1 | Regional post-mortem, identify weakest task | both | Phase 5 done | post-mortem |
| P6.2 | Add redundancy edges in `FieldMap` for risky paths | sw | P6.1 | FieldMap rev2 |
| P6.3 | Tighten recovery timeouts in lost-line / missed-intersection | sw | P6.1 | recovery v2 |
| P6.4 | Mechanical reinforcement on wear points | mech | P6.1 | hardened mech |
| P6.5 | Five full-run scoring sessions; require improvement vs Phase 4 | both | P6.3 | improved metrics |
| P6.6 | Code freeze for national, tag commit | sw | P6.5 | national tag |
| P6.7 | Compete at national | both | P6.6 | national result |

**Benchmarks for exit:** see Test Fixture Plan section 13 — B6.1.

## 10. Phase 7 — World Final Prep (`2026-08-25` -> `2026-11-30`, ~14 weeks)

**Goal:** Maximize score per run by re-ordering the task queue, shaving
seconds, and hardening the robot for travel and repeated handling.

**Exit criteria (hard gates to next phase):**
- [ ] Documented experiments showing the chosen task order beats alternatives on score/run.
- [ ] Robot survives a simulated travel cycle (drop test from carry height, vibration soak) with no functional loss.
- [ ] Spare-parts kit complete (motors, servos, sensor boards, batteries, fasteners).
- [ ] Optional: replay/log tooling shipped if [DECISIONS.md](./DECISIONS.md) "What is deliberately not in scope" is revisited.

**Tracks running in parallel:**
- Mechanical: robustness, travel case, spares.
- Software: scheduler tuning, scoring optimization, optional logging.

**Tasks (ordered):**

| ID | Task | Track | Blocked By | Output |
|----|------|-------|------------|--------|
| P7.1 | A/B task-order experiments via `Mission` config | sw | P6.7 | best ordering |
| P7.2 | Shave seconds — speed up turns, line cruise speed | sw | P7.1 | faster run |
| P7.3 | Mechanical robustness pass, travel case | mech | P6.4 | travel-ready |
| P7.4 | Spare-parts kit assembled and inventoried | mech | — | spares kit |
| P7.5 | (Optional) replay/log tooling, gated on team capacity | sw | — | replay tool |
| P7.6 | Code freeze for world final, tag commit | sw | P7.2 | worlds tag |
| P7.7 | Compete at world final | both | P7.6 | worlds result |

**Benchmarks for exit:** see Test Fixture Plan section 13 — B7.1.

## 11. Open Decisions Log

These are the open items from [ARCHITECTURE.md](./ARCHITECTURE.md) section
"Open items to confirm before coding" and [DECISIONS.md](./DECISIONS.md). All
must be closed before software can rely on a fixed sensor or mechanism. Decide-by
dates fall in Phase 0 or early Phase 1.

| Decision | Owner | Decide-by | Options | Notes |
|---|---|---|---|---|
| Camera | sw lead | `2026-05-12` | HuskyLens (built-in color recog, faster) ; raw camera + on-device pixel classify | Drives `CameraMosaic` impl; lighting calibration required either way (per [DECISIONS.md](./DECISIONS.md)). |
| Line sensor | sw lead | `2026-05-12` | MXLineTracer (10ch I2C) ; custom analog array | Affects `LineFollower` and `Intersection` interfaces. |
| Color sensor | sw lead | `2026-05-12` | MXColorV3 ; HTColV2 ; Vernier | `ColorReader.read()` -> `{Y,B,G,W,NONE}` regardless. |
| Manipulator v0 mechanism | mech lead | `2026-05-17` | TBD lift / gripper / arm geometry on 3 RC channels | Locks `Manipulator` setLift/setGripper/setArm parameters. |
| Line PID source | sw lead | `2026-05-24` | Library line-tracker helper ; custom PID | If lib helper proves usable, write less code; if not, `LineFollower` owns PID per [DECISIONS.md](./DECISIONS.md). |

## 12. Risk Register

| # | Risk | Likelihood | Impact | Mitigation | Track |
|---|---|---|---|---|---|
| 1 | Manipulator slips on tool / tile / cement, repeatedly | M | H | Failed-grip recovery (sec 4 of [DECISIONS.md](./DECISIONS.md)); grip-pad tuning in Phase 4; one retry then yield. | mech + sw |
| 2 | STM32 firmware `< v6.0` halts `MiniR4.begin()` at OLED prompt | L | H | Verify in P0.4; if low, flash before Phase 1 starts. | sw |
| 3 | Line sensor unreliable on competition mat surface | M | H | Decide sensor in P0; bench against printed mat in P2.5; fall back to alternative sensor option from sec 11. | sw |
| 4 | Camera lighting shift between practice and venue | H | H | On-site re-baseline in P5.3 / P6.5; geometry fixed by line-follow stop, not odometry (per [DECISIONS.md](./DECISIONS.md)). | sw |
| 5 | 2S battery sag causes brownout under high motor draw | M | M | `MiniR4.PWR.setBattCell(2)` correct; charged spares in go-bag; monitor `MiniR4.PWR` voltage on OLED. | mech |
| 6 | `FieldMap` revision mismatch vs official 2026 layout | M | H | Encode from rule book in P2.1; rev-tag the array; review against official PDF before each competition freeze. | sw |
| 7 | A blocking task breaks the 2-minute time guard | M | H | Iron rule 1 (no `delay()` in `update()`); `Mission` aborts overrunning tasks (per [DECISIONS.md](./DECISIONS.md)). | sw |
| 8 | Practice field arrives late, blocking Phase 4 / Phase 6 tuning | M | M | Use printed reduced-scale field (P2.5) until full-scale arrives; budget tuning into Phase 6 buffer weeks. | mech |
| 9 | Encoder slip / dead-reckoning drift during a long task | M | M | Architecture rejects pure dead-reckoning ([DECISIONS.md](./DECISIONS.md)); `Localization` re-anchors at every intersection. | sw |
| 10 | `Localization` written from outside the module | M | H | Coding criteria checklist in [ARCHITECTURE.md](./ARCHITECTURE.md); enforce in code review. | sw |

## 13. Test Fixture & Measurement Plan

Each phase exits on numeric, repeatable benchmarks. Tests are run on
hardware. IDs are referenced from each phase's "Benchmarks for exit" line.

| Phase | Benchmark ID | Setup | Pass criterion |
|---|---|---|---|
| 0 | B0.1 | `MiniR4_LibraryTest` flashed; serial open | Runs to completion, no firmware halt prompt; STM32 firmware string `>= v6.0`. |
| 0 | B0.2 | `MiniR4.DriveDC.Move(50,50)` for 2 s on bench rollers | Bot rolls forward straight; both wheels respond; no brownout. |
| 0 | B0.3 | `setBattCell(2)` then read `MiniR4.PWR` voltage at idle and under stall | Voltage in 2S range; no controller reset under stall load. |
| 1 | B1.1 | 2 m straight black line on white substrate, cruise speed | Lateral error `+/- 5 mm` at midpoint; no loss-of-line event. |
| 1 | B1.2 | T- and +-intersections, 20 trials each | `>= 19/20` correct detections per type. |
| 1 | B1.3 | `Drive.turnDeg(90)` from rest, IMU yaw logged | Final heading within `+/- 3 deg` of target on 10 of 10 turns. |
| 2 | B2.1 | Reduced-scale printed field, 10 random A->B traversals | `<= 1` missed intersection across all 10 runs. |
| 2 | B2.2 | `Pathfinder.findPath` over `MAX_NODES`-size graph | Returns optimal path on 20 random pairs; no dynamic allocation observed. |
| 3 | B3.1 | Each color sample (Y/B/G/W) shown to `ColorReader` 20x under workshop lighting | `>= 18/20` correct per color. |
| 3 | B3.2 | Mosaic pattern card under workshop lighting, robot at fixed stop | All 9 cells classified correctly on 10 of 10 captures. |
| 3 | B3.3 | Tool delivery task end-to-end on test mat, 5 trials | `>= 4/5` complete with score awarded. |
| 4 | B4.1 | Full 2-minute run on practice field, 5 trials | `>= 100` pts on 3 of 5 runs. |
| 4 | B4.2 | Inject lost-line, missed-intersection, failed-grip in lab | Each recovery triggers and bounds within timeout; run continues. |
| 5 | B5.1 | Venue practice run after lighting recalibration | One full run completed without loss-of-line or color misclassification. |
| 6 | B6.1 | Five full runs after post-mortem fixes | Mean score strictly higher than Phase 4 B4.1 baseline. |
| 7 | B7.1 | A/B task-ordering experiments via `Mission` config, 5 runs each | Chosen ordering beats alternatives on mean score/run. |

## 14. Glossary

Maps WRO 2026 Mosaic Masters terminology to the code modules in
[ARCHITECTURE.md](./ARCHITECTURE.md). When unsure which module owns a
concept, check this table first.

| WRO term | Code module |
|---|---|
| Field (the printed playing surface with black lines) | `FieldMap` |
| Intersection / waypoint (line crossing used as a checkpoint) | `Intersection`, `Localization` |
| Mosaic frame (the 3x3 fixture holding the pattern) | `CameraMosaic`, `Tasks` |
| Mosaic tile (single colored piece placed into one cell) | `Tasks`, `Manipulator` |
| Tools (the three deliverable items) | `Tasks`, `Manipulator` |
| Cement blocks (colored blocks delivered to deposit zones) | `Tasks`, `ColorReader`, `Manipulator` |
| Brick trowel (returned to start zone at end) | `Tasks`, `Mission` |
| Deposit zones (color-matched drop locations) | `FieldMap`, `Tasks` |
| 2-minute run (full autonomous round, scored up to 233) | `Mission` |
