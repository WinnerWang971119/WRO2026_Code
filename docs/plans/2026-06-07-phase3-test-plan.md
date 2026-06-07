# Phase 3 — Test Plan ("everything that needs testing")

**Phase 3 goal (Roadmap §4):** prove the simulator-verified stack actually works on
the **real test rover** (TT motors), and replace the toy `A–G + P1` graph with the
**real 2026 field**. This is the load-bearing phase — every later phase assumes the
foundation is reliable on hardware.

This document is the bench checklist. It collects every Phase 3 test from the
Roadmap exit gates (§4), the benchmark table (§13, IDs `B3.1`–`B3.7`), the risk
register (§12), and the open issues that bear on Phase 3 verification. Each item
says **where it runs** (host/CI vs. real chassis), the **pass criterion**, and the
**config knob** it tunes or proves.

Legend:
- 🔧 **HW** — must be run on the real rover. This is the actual Phase 3 work.
- 💻 **CI** — already covered by host ctest / pytest; do **not** re-do on the bench, just confirm green before the session.
- 📐 **DATA** — offline correctness of the generated field/routes (host).

---

## 0. Pre-session gate (run before touching hardware)

All of these are 💻 CI and must be green at the commit you flash:

- [ ] `uv run pytest tools/path_planner/tests/` — full Python suite passes (~24 test files: validator, graph, Dijkstra, compiler, header writer, sim parity, editor).
- [ ] `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure` — host C++ ctests pass (stop-at-line arm math, pass-through, bidirectional, tasks dispatch, line overshoot). Host toolchain is available locally (MSVC via vcvars64).
- [ ] `uv run python tools/path_planner/build_routes.py` then `git diff --exit-code` on `routes.h/.cpp` — generated routes are fresh (CI fails on stale routes).
- [ ] Arduino compile check passes (`arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` or IDE Verify).
- [ ] `Config::BATT_CELLS == 2` matches the physical 2S battery; `MiniR4.PWR.setBattCell(2)` confirmed (Risk #5).
- [ ] `using ActiveMotor = ...` in `Config.h` is set to the **test rover** profile (`MatrixTT`), not `LegoEV3` (Roadmap §1; Risk #12).

> Why split this way: the logic of every primitive (arm window, resolver
> precedence, FSM transitions, header emission) is already proven on host. Phase 3
> is **only** about the physical layer the host can't see — encoder→cm scaling,
> motor matching, sensor reflectance, line geometry, and end-to-end timing. Don't
> burn bench time re-checking logic.

---

## 1. Real-field encoding (📐 DATA — exit gate #1)

Replace the toy graph with the real 2026 mat. Tasks P3.4–P3.6.

| ID | Test | Pass criterion |
|----|------|----------------|
| F1 | Run `pick_xy.py` against the real WRO 2026 mat photo (`tools/path_planner/maps/`) | Every node/intersection coordinate measured; saved to `field_data.json` via the editor |
| F2 | Rewrite the field (nodes, line segments, blind edges, obstacles) for the real layout | `field_data.json` + `EDGES` (Python) reflect the official 2026 field |
| F3 | `uv run pytest tools/path_planner/tests/` after the rewrite | Validator passes (no ambiguity, finite inputs — ADR-029); planner builds NEXT_HOP for all pairs |
| F4 | `build_routes.py` regenerate `routes.h/.cpp`; review `map.png` (visualizer) | `map.png` matches the physical mat; SHA256 banner stamped; routes + goldens committed together (Risk #6) |
| F5 | Any-to-any reachability on the real graph (sim) | `sim/` PathTracker mirror traverses every goal pair without dead-ends |

**Watch during F2 (latent bugs that activate with the real field):**
- **Issue #44** — pivot compensation writes a *global* following-edge distance; under-drives any route that reaches the landing node *without* executing the preceding pivot. Inert today (empty `EDGES`). **If the real field authors a pivot edge whose following edge is plain-`BLIND` or `PROJECTION` and whose landing node is `START_NODE`/a mission goal → silent under-shoot.** Verify before shipping any pivot.
- **Issue #43** — auto stop-at-line before a `PROJECTION` follow is not yet implemented; a blind hop landing at a projection segment starts line-following from a dead-reckoned guess unless tagged `STOP_AT_LINE`. If the real field has blind→projection transitions, decide per-edge tagging.

---

## 2. Drive primitive accuracy (🔧 HW — exit gates #2, #3)

The core "does odometry survive contact with reality" gates. Tasks P3.1, P3.2.

| ID | Benchmark | Setup | Pass criterion | Tunes |
|----|-----------|-------|----------------|-------|
| B3.1 | `driveDist` accuracy | `Drive.driveDist(200).setMode(SYNC).tShape()` on flat floor, 10 trials | **±2 cm** at endpoint over 10 trials | encoder→cm scaling, `WROMath::cmToEncDeg` |
| B3.2 | `turnDeg` accuracy | `Drive.turnDeg(90)` from rest, IMU yaw logged, 20 trials | final heading **±2°** on **≥19/20** | `Config::TURN_PID_*` |
| — | Motor bias calibration (P3.2) | Run `Drive.calibrateMotorBias()`, copy printed scalars into `Config::MOTOR_BIAS_L/R` | straight drive tracks straight; both wheels matched | `MOTOR_BIAS_L`, `MOTOR_BIAS_R` (default 1.0/1.0 — must be measured) |

**Notes**
- SYNC + tShape is the default profile (ADR-017); test in that mode, it's what missions use.
- `calibrateMotorBias()` is a permitted setup-time blocking site; TT scalars do **not** transfer to Lofaix (Phase 7 re-runs it — ADR-011).
- The turn FSM has **no async stall watchdog** (Roadmap §4 follow-up): a jammed `.async()` turn can hang the FSM. Note any hang observed; it's a tracked hardening item, not a Phase 3 blocker unless it reproduces.

---

## 3. Line following (🔧 HW — exit gate #4)

Task P3.3. Tune the app-side P+D (the only PID we own; ADR motor PIDs stay on STM32).

| ID | Benchmark | Setup | Pass criterion | Tunes |
|----|-----------|-------|----------------|-------|
| B3.3 | Line cruise accuracy | 2 m line on white substrate, NORMAL cruise speed | lateral error **±5 mm** at midpoint; **no loss-of-line** | `Config::LINE_KP` (8.0), `LINE_KD` (4.0) |
| — | Sensor calibration | `WROLineArray.calibrate()` on the real mat surface | per-channel baselines captured into `LINE_CALIB_LOW/HIGH[2][8]` | two-layer calib (firmware illumination + software baselines, ADR-019) |
| — | Junction PID freeze | Drive a line through a `+`/`T` while following | PID output frozen across the junction; no lurch (ADR-020) | — |
| — | Lost-line behavior | Remove the line mid-follow | acts per `Config::LOST_LINE_ACTION` (= `STOP`); no runaway | `LOST_LINE_ACTION` |

**Risk #3:** line sensor may read differently on the competition mat surface —
re-baseline `LINE_CALIB_*` on the actual mat, and again on-site in Phase 7.3.

---

## 4. Intersection detection (🔧 HW — exit gate #5)

| ID | Benchmark | Setup | Pass criterion |
|----|-----------|-------|----------------|
| B3.4 | T- and +-intersection detection | T- and +-intersections, 20 trials each at NORMAL speed | **≥19/20** correct detections **per type**; correct `JunctionType` classification |

Confirms `Intersection` rising-edge + debounce + `WROLineArray::getJunctionType()`
classification on real geometry. This is the sensor edge that `Localization`
re-anchors on, so it gates B3.5.

---

## 5. Stop-at-line re-anchor (🔧 HW — exit gate via P3.10 / B3.7)

ADR-027. The cheapest blind-drift mitigation (Risk #9). Task P3.10.

| ID | Benchmark | Setup | Pass criterion | Tunes |
|----|-----------|-------|----------------|-------|
| B3.7 | Forward stop-at-line | `Drive.stopAtLine(cm)` toward a taped perpendicular line, **with one intervening line crossed before the arm window**, 10 trials | stops **±2 cm** of the taped line; **zero false-triggers** on the intervening line; `LINE_NOT_FOUND` fault when the line is removed | `STOP_AT_LINE_ARM_WINDOW_CM` (10), `STOP_AT_LINE_MAX_MARGIN_CM` (15), `STOP_AT_LINE_DETECT_TICKS` (2), `STOP_AT_LINE_STALL_TIMEOUT_MS` (1500) |
| — | Axle overshoot tune | Same rig, measure where the wheel axle lands | axle centers on the line | `LINE_SENSOR_AXLE_OFFSET_CM` (5.0) |
| — | Arm-window sizing | Inject deliberate under-travel | window absorbs worst-case under-travel, still arms on the true line | `STOP_AT_LINE_ARM_WINDOW_CM` (must exceed worst-case under-travel drift) |
| — | Flag drifting blind hops | After B3.5, mark blind edges that drift as `blind_mode: STOP_AT_LINE` in `field_data` | drifting blind hops re-anchor on a physical crossing | edge `BlindMode::STOP_AT_LINE` |

**Already CI-tested (💻 — don't re-bench):** the arm-window math
(`stopAtLineSeekMaxedOut`/`stopAtLineStalled` in `WROMath.h`), the FSM phases, and
the ON/OFF/AUTO resolver parity (`StopAtLinePolicy.h` vs `sim/runtime.py`, TC13)
are host-tested. The bench only proves the **physical** stop accuracy and timing.

**Deferred:** reverse stop-at-line (`B3.7-reverse`) — logic is CI-tested, but the
physical rear-board bench waits until the **rear line board is wired** (1-board
rover degrades a forced-REAR hop to a reverse dead-reckon, no fault).

---

## 6. PathTracker end-to-end navigation (🔧 HW — exit gate #6)

Task P3.7. The integration test — composes Drive + LineFollower + Intersection +
Localization + FieldMap.

| ID | Benchmark | Setup | Pass criterion |
|----|-----------|-------|----------------|
| B3.5 (toy) | `PathTracker.goTo(name)` on the **toy** printout | 10 random A→B traversals | **≤1 missed intersection** across all 10 runs |
| B3.5 (real) | `PathTracker.goTo(name)` on the **real-field** printout | 10 random A→B traversals | **≤1 missed intersection** across all 10 runs |

Run both: the toy graph isolates the FSM, the real graph proves the encoded field.
`Localization` must re-anchor at every intersection (ADR-001 rejects pure
dead-reckoning; ADR-005 single-writer). Watch for blind-edge drift here → it's the
trigger for the §8 pull-forward decision.

---

## 7. Recoveries (🔧 HW — exit gate #7)

Task P3.8. Two of the three named failure modes (ADR-009); failed-grip waits for
the manipulator (Phase 5/6).

| ID | Benchmark | Setup | Pass criterion |
|----|-----------|-------|----------------|
| B3.6 (lost-line) | Inject a lost line mid-follow | recovery triggers and **bounds within timeout**; no runaway |
| B3.6 (missed-int) | Inject a missed intersection | recovery triggers and bounds within timeout |
| — | Fault surfacing | Force `LINE_NOT_FOUND` / nav faults | `PathTracker` fault state set; mnemonic (NZ/UNR/DRV/BAD/DRN) to Serial; `FAIL PATH` on OLED |

---

## 8. Observability (🔧 HW — exit gate #8)

| ID | Test | Pass criterion |
|----|------|----------------|
| O1 | OLED `EventTrace` ring + Serial heartbeat during a 60 s drive | both readable end-to-end; 4-event ring updates; 500 ms heartbeat steady (ADR-010) |

This is the operator's only window into the run — verify it before relying on it to
debug the gates above.

---

## 9. Pull-forward decision (gate after B3.5)

If §6 (B3.5) exposes accumulated **blind-edge drift** on the real field, decide
whether to pull forward the Phase 2.5 deferred items **before** Phase 4
(Roadmap §4 "Pull-forward candidates"; Risk #9; Issue #36):

- [ ] Calibration nodes + blind-cost tiers + constrained Dijkstra (`PHASE2_DEFERRED.md`)
- [ ] `wall_zero` primitive (needs a mechanical bumper/limit switch first — Issue #36 "BlindWall Reset")
- [ ] Decay-cost path biasing toward reset points (Issue #36)

Decision rule: pull forward **only if** drift on a multi-blind route causes
mis-localization that B3.7 stop-at-line can't already absorb. Otherwise stay
deferred.

---

## 10. Pulled-forward software (💻 CI now / 🔧 bench later)

The Manipulator API + Config split (Issues #25, #39; plan
`docs/plans/2026-06-07-manipulator-api-and-config-split.md`) was pulled forward
from Phase 5 and is **structure-only** — the mechanism is CAD-done, not assembled,
so numeric calibration is `TODO: measure at bring-up`.

**What is testable now (💻 CI — host ctest, AC1–AC13 / TC1–TC16 in the plan):**
- [ ] Config split compiles; `ConfigChecks.h` included last; all includers build (AC1).
- [ ] Compile-fail asserts fire on ID collisions (`-DBELT_M_ID_DEFAULT=2`, `-DSORTER_RC_ID_DEFAULT=1`) with the exact `static_assert` message (AC2, TC13/TC14).
- [ ] `ManipServo` clamp + timer `isDone()`; `ManipBelt` POSITION/TIMED modes + watchdog + stall-home; `Manipulator` blocking/`.async()` verbs; `begin()` bounded home (TC1–TC12, TC15–TC16), under `-fsanitize=address,undefined`.

**What needs hardware (🔧 — deferred to Phase 5 bring-up, NOT this bench unless the mechanism is assembled):**
- Real servo angles (home/open/close/detent), `msPerDeg`, settle times.
- Belt PPR/RPM, station positions, home stall behavior, move/home timeouts.
- The thin verb→actuator choreography (`grab`/`release`/`sortTo`/`dump`/`reset`).
- B5.1 manipulator range sweep (Phase 5 benchmark, not Phase 3).

---

## 11. Out of scope for Phase 3 (don't test now)

- Camera / perception (Phase 4 — B4.1 color, B4.2 mosaic).
- Tasks FSMs, Mission scheduler, 2-minute time guard, failed-grip recovery (Phase 6 — B6.x).
- Lofaix (EV3) bring-up: re-run `calibrateMotorBias()`, re-tune `LINE_KP/KD`, re-pass B6.1 (Phase 7 — B7.1).
- Open enhancement issues not gating Phase 3: #40 (FHT fault handling), #33 (CTPS sim), #23 (log replay), #31 (routes.h merge tooling), #46 (host harness real-source linking — CI hygiene).

---

## Exit summary — Phase 3 is done when

All Roadmap §4 hard gates pass:

- [ ] F1–F5 real field encoded, routes regenerated, `map.png` reviewed.
- [ ] B3.1 driveDist ±2 cm / B3.2 turnDeg ±2° (19/20).
- [ ] B3.3 line cruise ±5 mm; B3.4 intersection ≥19/20 per type.
- [ ] B3.5 PathTracker ≤1 missed intersection / 10 runs (toy **and** real).
- [ ] B3.6 lost-line + missed-intersection recoveries bound.
- [ ] B3.7 stop-at-line ±2 cm, zero false-triggers, faults on removed line.
- [ ] O1 EventTrace + heartbeat readable over 60 s.
- [ ] §9 pull-forward decision recorded (pull forward or stay deferred, with reason).
</content>
</invoke>
