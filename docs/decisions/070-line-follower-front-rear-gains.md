# ADR-070: LineFollower uses a separate P+D gain set per board (front vs rear)

**Status:** Implemented
**Date decided:** 2026-07 (Phase 3 bring-up / tuning)

## Rule
The outer line-follow P+D controller selects its gains by the leading board: forward follow
(front board) uses `Config::LINE_KP` / `LINE_KD`; reverse follow (rear board, `_reverse == true`)
uses `Config::LINE_KP_REAR` / `LINE_KD_REAR`. `LINE_MAX_CORR` and the speed preset stay shared.
The rear gains default to the front values, so behaviour is unchanged until the rear set is tuned.

## Why
The line sensor's distance from the wheel axle is the controller's *lookahead*, and the two
boards do not share it: the front sits `LINE_SENSOR_AXLE_OFFSET_CM` (12.02 cm) ahead of the axle,
the rear `LINE_SENSOR_AXLE_OFFSET_REAR_CM` (6.0 cm). A longer lookahead anticipates heading error
and rides smooth; a shorter one has less preview, so the *same* Kp/Kd that track cleanly forward
over-correct and swing the chassis off the line in reverse. This showed up on the bench as smooth
forward follow and oscillating, line-losing reverse follow at one shared gain set — the classic
signature of one controller driving two different lever arms. A single set cannot satisfy both;
each direction needs its own tune.

## How to apply
- Tune each set independently with the `inoTest/line_pid_tune` ping-pong tuner: its forward runs
  optimise the FRONT set, its reverse runs the REAR set (two independent twiddle searches).
- Keep the bench-tuned REAR values in `Config::LINE_KP_REAR` / `LINE_KD_REAR`
  (`WRO2026/config/ConfigLine.h`) synchronized with any future tuner result.
- The selection lives in `LineFollowerClass::update()`'s online P+D block; it keys off the same
  `_reverse` flag that already selects the board (`setReverseActive`) and negates cruise, so no
  new state is introduced. The correction sign is unchanged — the rear board's error mapping
  carries the direction (see ADR-062), so only the gain magnitudes differ.

## Trigger to revisit
If a future chassis mounts the two boards at the same axle distance (equal lookahead), the split
becomes redundant and `LINE_KP_REAR` / `LINE_KD_REAR` can collapse back to the front set. Also
revisit if an integral term is ever added — Ki would need the same per-board treatment.

## See also
- ADR-062 (dark-line polarity; the rear board needs no `getError()` negation, only its own gains).
- ADR-024 (bidirectional line-following / rear board).
- ADR-069 (lateral-offset line-follow); shares the same `corr` line — the board-selected gains
  here multiply both the offset-biased P term and the D term.
- `WRO2026/src/drive/LineFollower.cpp` P+D block; `WRO2026/config/ConfigLine.h` gain constants;
  `inoTest/line_pid_tune/line_pid_tune.ino` (the per-direction tuner).
