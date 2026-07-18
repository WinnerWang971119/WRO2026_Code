# line_pid_manual — live LineFollower P+D tuner

The hand-driven companion to [`line_pid_tune`](../line_pid_tune) (the auto-tuner). Instead of a
twiddle search, **you** set the gains: type them in the serial monitor, run a line-follow, read
the score, tweak, run again. Gains live in RAM — **no reflash between trials.**

Use this when you want to feel out the gains yourself, sanity-check a value, or explore beyond
where the auto-tuner converged.

## Run

```
uv run inoTest/line_pid_manual/build.py
```

Compiles, uploads, opens the serial monitor at 115200. `--build-only` to compile without flashing.

## Serial commands

Type in the monitor and press Enter:

| Command | Effect |
|---|---|
| `kp <v>` | set Kp of the **active** direction |
| `kd <v>` | set Kd of the **active** direction |
| `c <v>` | set cruise PWM (default `SPEED_NORMAL` = 70) |
| `dist <v>` | set run distance in cm (default 50) |
| `f` | select **forward** (front board) |
| `r` | select **reverse** (rear board) |
| `go` | run one line-follow with the current gains |
| `stop` | abort a run in progress / stop the motors |
| `?` | print current settings + help |

**Buttons:** BTN_UP = run · BTN_DOWN = toggle direction. Either button also aborts a run in
progress.

`kp`/`kd` edit whichever direction is selected, so the flow is: `f` or `r` to pick the side,
`kp`/`kd` to set its gains, `go` to test. It holds a separate FRONT and REAR set (ADR-066).

## Reading a run

After each run it prints the score so you can compare trials by hand:

```
[man] done cost=142.3 ( meanSqErr=1.204 osc=9 lostTicks=3 peakErr=2.80 ticks=118 )
```

Lower cost = better. `meanSqErr` = tracking tightness (Kp), `osc` = oscillation (Kd), `lostTicks`
= time off the line, `peakErr` = worst single excursion. See the
[auto-tuner README](../line_pid_tune/README.md) for the full cost breakdown.

## Notes

- Same P+D law as production (transfers 1:1 to `Config::LINE_KP`/`LINE_KD` and the `_REAR` set).
- Reverse runs read the REAR board — calibrate it first or the reverse score is noise.
- SAFETY: every run self-terminates on distance, a 6 s timeout, or a 1.2 s sustained line-loss.
- Not in CI (like every `inoTest/` sketch).
