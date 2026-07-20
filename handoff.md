# Handoff — turn PD never settles (bounces, times out every turn)

**Branch:** `test/tune-robot-third-time` · **HEAD:** `e75f487`
**Date:** 2026-07-20
**Chassis:** Lofaix LEGO-EV3 comp robot (NOT the TT test rover)

---

## 1. Symptom

Every `turnDeg` / `turnToYaw` oscillates around the target and never satisfies the settle
debounce. It runs the full `TURN_TIMEOUT_MS` (3000 ms), then brakes and reports done
**without faulting** (non-faulting by design), so the mission continues from a wrong heading.
Operator described it as "bouncing", "damps and damps", and later "super slow".

---

## 2. Root cause — almost certainly a yaw/wheel SIGN INVERSION

A per-tick trace was captured (`turn.log`, untracked, in repo root). It is unambiguous:

```
t_ms, yaw,  err,  dt_ms, cmd_raw, cmd_out, moveL, moveR, broke, inband
   0,  0.0, 90.0,   14,    180,      65,    -65,    65,    0,     0
  67, -1.0, 91.0,   16,    186,      65,    -65,    65,    0,     0
  99, -2.0, 92.0,   16,    188,      65,    -65,    65,    1,     0
 131, -3.0, 93.0,   16,    186,      65,    -65,    65,    1,     0
```

Target `+90°`. Wheels commanded at **full authority** in the correct `COUNTER_ROT` pattern
(`L = -65, R = +65`). And `yaw` moves **0 → −1 → −2 → −3** while `err` grows **90 → 91 → 92 → 93**.

The chassis rotates *away* from the target. The loop is **positive feedback**. All six trials
timed out, mirrored in both directions:

```
trial,dir,cmd,exitDelta,restDelta,overshoot,coast,durMs,flag
1, 1, 90, -88.51, -88.51,  -1.49, 0.00, 3735, TIMEOUT
2,-1,-90,  91.49,  91.49,   1.49, 0.00, 3745, TIMEOUT
3, 1, 90, -72.60, -72.60, -17.40, 0.00, 3740, TIMEOUT
```

Commanded `+90` → ends ~88° on the wrong side. Commanded `−90` → ends ~91° on the wrong side.

**No gain value can fix a sign inversion.** This is why sweeping `TURN_KD` from 0 to 5 changed
nothing — the operator did that exhaustively and correctly concluded gains were not the variable.

### The fix

`turnWheelPattern(sign=+1, ...)` emits `L = -pwm, R = +pwm` and expects yaw to **increase**.
On this robot that wheel pattern **decreases** the measured yaw. Exactly one of these two is
inverted — flip **one**, never both (flipping both cancels and looks like the constant "does
nothing"):

| Candidate | Current | Notes |
|---|---|---|
| `Config::GYRO_YAW_SIGN` (`ConfigNav.h:244`) | `-1.0f` | **Primary suspect.** Applied once in `GyroDecodePolicy.h::decodeGyroYawDeg`. Determined by `inoTest/gyro_calib`'s SIGN phase — likely run with the operator rotating the wrong way. |
| `MOTOR_LEFT_REVERSE` / `MOTOR_RIGHT_REVERSE` (`ConfigDrive.h:83,88`) | `true` / `false` | Less likely — this pair is validated by straight-line driving working, and is documented as correct for Lofaix (the TT rover uses the opposite pair). Changing it would break straight moves. |

**Recommended:** flip `GYRO_YAW_SIGN` to `+1.0f`, reflash `inoTest/turn_tune --define TURN_TRACE`,
and confirm in the trace that `err` now *shrinks* monotonically from the first tick.

**Arbiter if unsure:** re-run `inoTest/gyro_calib` SIGN phase and follow its prompt exactly —
it reports whether the raw count increases under CCW rotation.

> ⚠️ `GYRO_YAW_SIGN` is also read by `Localization`. Flipping it inverts the world-heading frame
> for nav as well. After the flip, re-verify a full mission hop, not just an isolated turn.

---

## 3. Ruled out — do NOT re-investigate

Each of these was tested or verified this session:

- **Gains.** `TURN_KD` swept 0 → 5 (operator, on hardware). `TURN_KP` set to the mentor's proven
  `2.0`. No effect. `Kd = 0` (pure P) still bounces.
- **Exit condition.** `TURN_SETTLE_SAMPLES` 2 → 1 and `TURN_TOLERANCE_DEG` → 1.0, matching the
  mentor's single-sample exit exactly. No effect.
- **Pivot geometry.** Route table emits `COUNTER_ROT` / `DEFAULT` only — zero `PIVOT_LEFT` /
  `PIVOT_RIGHT`. Both wheels are always driven.
- **Two-writer motor fight.** `_dispatchTurnIfNeeded` disables `LineFollower` before dispatch;
  all four `setEnabled(true)` sites are in edge-dispatch paths unreachable during `State::TURNING`.
  Guard verified sound.
- **Gyro decode / scale.** Matches the mentor's math exactly. `gyro_calib` run and passing,
  `GYRO_SCALE = 0.9945` inside the `(0.9, 1.1)` sanity band.
- **Battery** and **`TURN_TRACE` build overhead** — both checked by the operator, both fine.
- **Dispatch parameters.** Mission uses default power (65) and the same mode as `turn_tune`;
  one-shot dispatch, no re-dispatch loop within an edge.
- **The three commits that landed mid-session** (`fada883`, `87a5ec4`, `bc82ee8` — ConfigUi,
  ConsoleTypes, docs). Header-only, nothing wired into the tick, zero runtime cost.

---

## 4. Facts established this session (worth keeping)

- **The external gyro's wire resolution is 1 degree per count**, not 0.01. `gyro_calib.ino:81`:
  *"10 turns -> 3600 raw counts."* The chip may resolve 0.01° internally, but the two-byte
  protocol `(G*256 + R) − 32768` spends its whole 16-bit range on ±32768 **degrees**. So the
  smallest heading change the firmware can observe is `1 × GYRO_SCALE` = **0.9945°**.
- **Measured turn tick is ~16–17 ms**, per the trace `dt_ms` column — not the ~22.66 ms recorded
  in `ConfigDrive.h`. Anything derived from the tick should be re-checked against this.
- **`TURN_MIN_RESOLVABLE_DEG` was wrong.** It held `0.4423` (motion per tick, measured at the old
  creep of 38). The binding limit is the larger of motion-per-tick and the sensor quantum, i.e.
  `0.9945`. Corrected this session; compile-time only, never read at runtime.

---

## 5. Open items (independent of the sign bug)

### 5a. `NAV_TURN_TOLERANCE_DEG` is unsatisfiable — real bug, not yet fixed

```
TURN_TOLERANCE_DEG     = 1.0    // turn controller exits with up to 1.0 deg residual
NAV_TURN_TOLERANCE_DEG = 0.5    // but PathTracker only SKIPS a turn if delta <= 0.5 deg
gyro quantum           = 0.9945 // 0.5 deg is below one sensor count
```

The turn controller's guaranteed residual exceeds the nav layer's skip threshold, and the
threshold is finer than the sensor can measure. So `_dispatchTurnIfNeeded` fires a turn at
colinear joins it was explicitly written to skip (*"Avoids a sub-degree twitch at every edge
join"*). Each spurious dispatch costs: `LineFollower` disabled, `_passThrough = false`,
`_chainReset()`, and a `HOP_SETTLE_MS` (100 ms) dwell, then a soft-start from
`TSHAPE_START_SPEED` instead of cruise.

**No `static_assert` couples the two** — that's the missing tripwire.

Fix: raise `NAV_TURN_TOLERANCE_DEG` to `>= TURN_TOLERANCE_DEG` (suggest 1.5), and add the assert.
Tightening `TURN_TOLERANCE_DEG` instead is **not possible** — 0.5° is below the sensor quantum,
so the turn could never settle.

**Blast radius — four surfaces, not a one-liner:** the constant is mirrored in
`tools/path_planner/sim/runtime.py` and consumed by `compiler.py`, `validator.py`, and
`ctps/track_builder.py`. Needs the firmware change + sim mirror + `build_routes.py` regeneration
+ full `pytest` and `ctest`.

### 5b. `turn_pd_integration` host test — 1 failing, stale fixtures

Baseline at committed values is **83/83 green**. Currently 82/83. Four assertions fail, all
fixture arithmetic, no firmware regression:

- **3 assertions** (`test_turn_pd_integration.cpp:294,300`) from the operator's
  `TURN_SLEW_PWM_PER_TICK 10→20` and `TURN_BREAKAWAY_PWM 46→50`. The test hard-codes "4 ramp
  ticks at `power=90` won't reach the ceiling", true only at the old values.
- **1 assertion** (`:654`, TC15) from `TURN_SETTLE_SAMPLES 2→1`. The harness teleports yaw to the
  target on tick 1; with a single-sample exit the turn correctly completes without commanding
  motion, so the test's `moved` probe (which exists to catch an unrelated `power=0` sentinel bug)
  no longer trips.

**Not patched deliberately** — the right fix is deriving expectations from `Config` rather than
hard-coding tick counts. That's a decision for whoever owns the test.

### 5c. Left wheel reported dead — never resolved

Mid-session the operator reported *"the left isn't moving"*, later *"robot going super slow"*.
Battery and trace overhead were both ruled out by the operator. **This was never diagnosed.**
It may be a second, independent fault, or it may be the sign bug being misread.

```
uv run inoTest/motor_id/build.py     # drives M2 then M3, reports wheel + encoder per port
```

Note the trace shows `moveL=-65, moveR=+65` being *commanded* — the firmware is asking for both
wheels. Whether both deliver is a drivetrain question the trace cannot answer.

### 5d. `TURN_KD` is `0.03`; the mentor's equivalent is `0.06`

Irrelevant until the sign bug is fixed, but worth normalising afterwards.

---

## 6. Mentor reference (ground truth for this chassis family)

Source: `/Users/diego/Documents/WRO2026/Meat_code/` — `meat.ino` plus three screenshots
(`messageImage_*.jpg`). **Outside the repo, not in CI.**

`M_Turn_angle(power, angle)` decoded:

```
error_old = 0;  error_new = angle - Gyro3D;  timer1.reset()
repeat until (|error_new| < 1) OR (timer1 > 3000):
    Gyro_Read;  error_new = angle - Gyro3D;  wrap to +/-180
    error = error_new - error_old              // RAW per-iteration diff, no dt
    speed = 2*error_new + 10*error             // Kp=2, Kd_raw=10 (~0.06 time-normalized)
    clamp to +/-power
    if 0 < speed <  20: speed =  20            // floor == threshold (continuous, no cliff)
    if 0 > speed > -20: speed = -20
    Move(speed, -speed);  error_old = error_new
Stop(Brake)
```

Key structural differences from ours: his creep threshold **equals** the floor (20) so there is no
5-PWM discontinuity; he has **no slew limiter** and **no two-level breakaway** (his `error_old = 0`
seeding makes tick 1 saturate, which *is* his launch kick); and he exits on the **first** in-band
sample.

His **straight-line** hold `gyro_move` uses `Kp=1.5, Kd_raw=8` — do not confuse the two. Our
`TURN_KP` was found at `1.5` (his *straight* gain) at session start; it has been corrected to `2.0`.

`Gyro_Read`: `(G*256 + R) − 32768`, `* 0.997`, `mod 360`, wrap to `(−180, +180]`. Ours matches,
with a bench-measured `0.9945` in place of his `0.997`.

---

## 7. Tree state — everything below is UNCOMMITTED

```
 M WRO2026/config/ConfigDrive.h
 M WRO2026/src/drive/Drive.cpp
 M inoTest/build_common.py
 M WRO2026/config/ConfigManipulator.h        <- not from this workstream
 M docs/brainstorms/2026-07-20-issue40-...   <- not from this workstream
?? turn.log                                  <- the trace capture; untracked
?? handoff.md                                <- this file
```

### Current turn constants

| Constant | Value | Changed this session by |
|---|---|---|
| `TURN_MODE` | 0 (COUNTER_ROT) | — |
| `TURN_KP` | 2.0 | operator (was 1.5) |
| `TURN_KD` | 0.03 | operator (was 0.1) |
| `TURN_CRUISE_PWM` | 65 | — |
| `TURN_CREEP_THRESH_PWM` | 25 | — |
| `TURN_CREEP_PWM` | 20 | — (operator confirms 20 is correct) |
| `TURN_BREAKAWAY_PWM` | 50 | operator (was 46) |
| `TURN_BREAKAWAY_MOTION_DEG` | 1.0 | — |
| `TURN_SLEW_PWM_PER_TICK` | 20 | operator (was 10) |
| `TURN_SETTLE_SAMPLES` | **1** | assistant (was 2) |
| `TURN_TOLERANCE_DEG` | 1.0 | operator (was 0.8) |
| `TURN_TIMEOUT_MS` | 3000 | — |
| `TURN_MIN_RESOLVABLE_DEG` | **0.9945** | assistant (was 0.4423) |
| `NAV_TURN_TOLERANCE_DEG` | 0.5 | — (see 5a) |
| `GYRO_SCALE` | 0.9945 | — |
| `GYRO_YAW_SIGN` | **−1.0** | — (see §2) |

### Assistant-authored changes

- **`Drive.cpp`** — per-tick turn trace, **124 insertions / 0 deletions**, entirely inside
  `#ifdef TURN_TRACE`. Production build behaviourally identical; costs 2.9 kB SRAM only when
  enabled. Revert cleanly with `git checkout WRO2026/src/drive/Drive.cpp`.
- **`build_common.py`** — new repeatable `--define NAME[=VALUE]` passthrough, emitted as a single
  `compiler.cpp.extra_flags` build property (Arduino CLI keeps only the last value for a repeated
  property key, so they must be combined). Useful beyond this session.
- **`ConfigDrive.h`** — `TURN_SETTLE_SAMPLES` 2→1 and `TURN_MIN_RESOLVABLE_DEG` 0.4423→0.9945,
  both with rewritten docstrings explaining the reasoning.

### Build status

- `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` — **clean**, with and
  without `-DTURN_TRACE`.
- `ctest` — **82/83**, only `turn_pd_integration` (see 5b).

---

## 8. Tools

```bash
# per-tick turn trace (CSV dumped at turn end; buffered in RAM so recording
# does not perturb the tick — but the DUMP blocks ~0.6 s per turn)
uv run inoTest/turn_tune/build.py --define TURN_TRACE > turn.log     # NOT | tee

uv run inoTest/motor_id/build.py     # which motor / which encoder per port
uv run inoTest/gyro_calib/build.py   # SIGN / SCALE / DRIFT phases  <-- arbiter for §2
uv run inoTest/turn_probe/build.py   # open-loop creep + breakaway + omega + tick
uv run inoTest/tick_bench/build.py   # cooperative tick period
```

Trace columns: `t_ms, yaw, err, dt_ms, cmd_raw, cmd_out, moveL, moveR, broke, inband`.
Read in this order: `dt_ms` (real tick) → `yaw` vs `moveL/moveR` (does it respond, right
direction?) → `err` (does it shrink?) → `inband` → `cmd_raw` vs `cmd_out` (how much the
clamp/slew/creep reshaped the PD).

---

## 9. What I got wrong — so it isn't repeated

1. **Tuned constants for most of the session instead of instrumenting first.** The trace that
   found the bug took ~20 minutes to write and answered it in one run. It should have been the
   *first* action once "Kd = 0 still bounces" was known — that fact alone rules out the entire
   gain axis and should have redirected to signal integrity immediately.
2. **Claimed `TURN_KD = 0.1` was a noise ceiling** on hand-wavy grounds. The measured numbers
   (0.100° IMU noise, ~16–22 ms tick) don't support it. Corrected mid-session, but it cost the
   operator bench time.
3. **Recommended widening `TURN_TOLERANCE_DEG` to 1.0 without checking `NAV_TURN_TOLERANCE_DEG`.**
   That doubled a pre-existing mismatch (§5a).
4. **Attached the nav-tolerance finding to a "slow turn" symptom it cannot explain.** Tolerance
   governs when a turn *stops*, not how fast it *rotates*. The operator correctly rejected it.
5. **Never followed up on the reported dead left wheel** (§5c) while continuing to propose config
   changes on top of an unexplained drivetrain symptom.

---

## 10. Suggested order of work

1. Flip `GYRO_YAW_SIGN` → `+1.0f`. Reflash `turn_tune --define TURN_TRACE`. Confirm `err` shrinks
   monotonically from tick 1 and trials report `SETTLED`, not `TIMEOUT`. **Everything else is
   blocked behind this.**
2. Run `motor_id` to close out the left-wheel question (§5c).
3. Re-verify a full mission hop — `GYRO_YAW_SIGN` also feeds `Localization`'s world frame.
4. Normalise `TURN_KD` to 0.06 and re-baseline gains against a working loop.
5. Fix `NAV_TURN_TOLERANCE_DEG` (§5a) across all four surfaces.
6. Repair or re-derive the `turn_pd_integration` fixtures (§5b).
7. Decide whether the `TURN_TRACE` block stays in `Drive.cpp` (it's inert without the flag) or is
   reverted.
