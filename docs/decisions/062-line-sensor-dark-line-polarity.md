# ADR-062: Line sensor follows a DARK line — on-line-ness is `(HIGH - raw) / (HIGH - LOW)`

**Status:** Implemented
**Date decided:** 2026-07 (Phase 3 / LEGO-EV3 bring-up)

## Rule
The MXColor reflectance channels read HIGH over white and LOW over the (dark) line. A
channel's "on-line-ness" is therefore `(HIGH - raw) / (HIGH - LOW)` — 1.0 at the calibrated
dark endpoint, 0.0 at the calibrated white endpoint. `Config::LINE_CALIB_LOW` is the
per-channel DARK/on-line value (the sweep minimum); `LINE_CALIB_HIGH` is the WHITE/off-line
value (the sweep maximum). All three `WROLineArray` predicates — weighted error, on-line, and
junction classification — derive their score from the single `_onnessNorm(raw, low, high)`
helper so their polarity cannot drift apart.

## Why
The original `WROLineArray` normalised as `(raw - LOW) / (HIGH - LOW)` and treated a HIGH score
as on-line, i.e. it assumed the followed line reads BRIGHTER than the background. The physical
WRO field is a BLACK line on a white mat, where the sensor reads the line LOW — the opposite
premise. The result was a silently inverted controller: the weighted centroid locked onto the
white background instead of the line, so every steering correction pointed AWAY from the line
(the "opposite correction" bug), and `isOnline()` / junction detection were inverted too.
Pure recalibration could not fix it — `calibrate()` captures per-channel min→LOW / max→HIGH,
so on black-on-white it still fed white into the "on-line" side. The known-good competition
reference (`Meat_code/meat.ino`, follower `MBC_black`, marker detection `light < 4`) confirms
dark = low; its motor wiring and `Move(power+speed, power-speed)` mix are identical to ours, so
the error-term polarity was the sole divergence.

## How to apply
- Compute per-channel on-line-ness only through `WROLineArrayClass::_onnessNorm`; never inline a
  `(raw - LOW)` form. Callers keep the dead-channel guard (`LOW >= HIGH` → skip) so the helper's
  denominator stays positive.
- Treat `LINE_CALIB_LOW` as the dark endpoint and `LINE_CALIB_HIGH` as the white endpoint. The
  `calibrate()` min→LOW / max→HIGH capture and the `{0}` / `{9}` safe defaults already satisfy
  this — an uncalibrated channel fails safe to off-line over a bright surface.
- The `_sensorWeights` `{-3.5 … +3.5}` (ch0 left … ch7 right) and the `LineFollower`
  `L = cruise + corr / R = cruise - corr` mix are correct as-is once polarity is right; do NOT
  "fix" a steering inversion by negating `getError()` — that leaves on-line/junction inverted.

## Trigger to revisit
A future field or sensor where the followed feature reads BRIGHTER than its background (e.g. a
white/reflective line on a dark mat), or an MXColor firmware change that inverts the raw
reflectance scale. Either flips the premise and requires re-deriving `_onnessNorm`.

## See also
- ADR-019 (custom 8-channel `WROLineArray` replaces `MatrixLineTracer`).
- ADR-020 (junction PID freeze), ADR-024 (bidirectional line following).
- `WRO2026/src/perception/WROLineArray.{h,cpp}`, `WRO2026/config/ConfigLine.h`.
