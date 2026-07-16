# line_pid_tune — LineFollower P+D auto-tuner

An on-robot, OLED-guided auto-tuner for the line-follower's P+D gains. It finds good
`LINE_KP` / `LINE_KD` (front board, forward) **and** `LINE_KP_REAR` / `LINE_KD_REAR`
(rear board, reverse) in one session, driven entirely from the robot's two buttons — no
laptop needed while tuning.

Companion to [`junction_monitor`](../junction_monitor) (which watches the sensor) — this one
drives the chassis and scores how well it tracks.

## Run

```
uv run inoTest/line_pid_tune/build.py
```

Compiles, uploads to the connected R4, and opens the serial monitor at 115200. Add
`--build-only` to compile without flashing, or `--port <dev>` if auto-detect fails.

## What it does

It runs a **twiddle** (coordinate-descent) optimiser: each run drives a flat 50 cm line-follow
at cruise `SPEED_NORMAL` (70), scores the run, and the optimiser nudges the gains toward the
lowest score until it converges. Gains change in RAM between runs — **no reflash per trial.**

With two line boards it runs **two independent optimisers at once** (parallel ping-pong):
forward runs tune the FRONT set, reverse runs tune the REAR set. The two boards sit at
different distances from the axle (12.02 cm vs 7.68 cm = different lookahead), so each direction
needs its own gains — see [ADR-066](../../docs/decisions/066-line-follower-front-rear-gains.md).
With one board it tunes the front set only.

## Operator flow

The OLED tells you, before every run, how to place the robot and which way it will drive. Each
optimiser's evaluation is three guided placements:

| Placement | How to set the robot |
|---|---|
| `VERY OFF ~60deg` | angled ~45–70° across the line — big recovery |
| `A LITTLE OFF` | small angle / small lateral offset |
| `ALMOST ON LINE` | nearly centred — tests steady-state wobble / overshoot |

The schedule interleaves forward + reverse, so the robot **shuttles on one 50 cm stretch** (a
forward run ends downrange; the paired reverse run drives it back). Per run:

1. OLED shows `FWD VERY OFF ~60deg` (or `REV …`) + the gains about to be tested.
2. Place the robot that way on a straight 50 cm+ line.
3. Press **BTN_UP** → it drives 50 cm, scoring as it goes, then brakes.
4. Repeat. Each direction converges on its own; when **both** are done the OLED shows `TUNED`
   and Serial prints the paste blocks.

### Buttons

- **BTN_UP** — run the next guided placement (or restart after `TUNED`).
- **BTN_DOWN** — finish now and show the best-so-far for both directions.

## Reading the result

On convergence (or BTN_DOWN) Serial prints a ready-to-paste block:

```
constexpr float LINE_KP = 5.600f;
constexpr float LINE_KD = 1.400f;
constexpr float LINE_KP_REAR = 4.200f;
constexpr float LINE_KD_REAR = 1.800f;
```

Paste into [`WRO2026/config/ConfigLine.h`](../../WRO2026/config/ConfigLine.h) and reflash the
main firmware.

## The cost function

Each run accumulates three numbers tick-by-tick, then folds them into one score:

```
cost = 50·meanSqErr + 3·oscCount + 10·lostTicks   (+ 5000 if the run was cut short)
```

- **meanSqErr** — mean of `getError()²` over online ticks. Rewards tracking (Kp).
- **oscCount** — hysteresis-gated sign flips of the error (`|err| > 0.4`). Rewards damping (Kd).
- **lostTicks** — ticks spent off the line.
- **abort penalty** — a run that can't finish 50 cm (6 s timeout or 1.2 s sustained line-loss)
  is cut short for safety and scored +5000, so the optimiser avoids gains that fly off.

Every run prints its breakdown to Serial, e.g.
`cost=142.3 ( meanSqErr=1.204 osc=9 lostTicks=3 ticks=118 )`, so a score is never a black box.
The weights are named constants at the top of the sketch — reshape them if the search leans
wrong (wobbly-but-on-line → raise `OSC_WEIGHT`; loose-but-smooth → raise `MSE_WEIGHT`).

## Notes & caveats

- **The gains transfer 1:1.** The sketch reproduces `LineFollower.cpp`'s online P+D law exactly
  (same error source, ±`LINE_MAX_CORR` clamp, `L=cruise+corr`/`R=cruise-corr` mix, cruise negated
  in reverse, rear board selected via `setReverseActive`, `Drive.setSpeed` deadband). If that law
  changes in production, mirror it in `stepControl()`.
- **Calibrate the rear board first.** Reverse runs read the rear board — if `LINE_CALIB` row 1 is
  still at defaults, the reverse scores are noise. Run the main firmware's LINE calibration
  (hold BTN_DOWN ≥ 2 s) before trusting the `REV` results.
- **Ki is fixed at 0** — the P+D law has no integral term. Only Kp and Kd are searched.
- **It's a local search.** With only three placements per evaluation and noisy reverse runs, a
  direction can converge on a lucky round. If a converged set looks off on the robot, re-run.
- **Not in CI.** Like every `inoTest/` sketch, `arduino-cli` is the only check that it links.
