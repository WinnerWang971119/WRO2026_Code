# ADR-044: IMU yaw sign normalization — all raw IMU yaw I/O flows through YawSource

**Status:** Implemented
**Date decided:** 2026-06 (Phase 2)

## Rule

All raw IMU yaw I/O goes through the single `YawSource` module, which applies the per-chassis
`Config::IMU_YAW_SIGN` once on every read (STM32 CW-positive raw → app CCW-positive frame, via
`YawSource::readYawDeg` / `readYawDegOr`) and once on every heading target written to the STM32
gyro PID (`YawSource::toHwHeading` — app → raw, pure ×SIGN). The turn-direction sign and
wheel-pattern math live in the header-only host-tested `TurnSignPolicy.h`. No module outside
`YawSource` calls `MiniR4.Motion.getEuler`, `mmL.GetIMUEuler`, or applies the sign itself; no
`MoveGyro*` heading argument bypasses `toHwHeading`.

## Why

The hardware IMU is bench-measured CW-positive (CCW rotation decreases raw yaw), while the Python
planner emits world headings as `atan2(dy, dx) % 360` (CCW-positive, 0° = East) and `Localization`
documents the same convention. Without correction the custom R4 turn loop commands CCW motion
expecting yaw to rise, but yaw falls — it commits to the wrong direction and spins to the opposite
heading. World navigation would be spatially mirrored. Applying a single `±1` constant at one
ownership boundary fixes both the read path and the STM32 gyro-PID write atomically; a half-applied
bridge would leave the app frame internally inconsistent.

Extracting the turn-sign and wheel-pattern math to `TurnSignPolicy.h` gives the direction fix an
automated host regression guard. `Drive.cpp` is not host-compiled (heavy MatrixMiniR4 deps; see
`test/CMakeLists.txt`), so the only way to machine-verify the direction logic is to isolate it in a
header-only policy that the host harness can reach directly.

## How to apply

- **Reading IMU yaw:** call `YawSource.readYawDeg(yawOut)` (result-surfacing, retry-once, returns
  `double`) or the convenience wrapper `YawSource.readYawDegOr(fallback)`. These are the only entry
  points for `mmL.GetIMUEuler`. `Localization::_readImuYaw` delegates to `readYawDeg`; the five
  Drive read sites (`_execTurnDeg`, `_execTurnToYaw`, `getYaw`, `holdYaw`, `_updateTurnProfile`)
  use `readYawDegOr` or the bool form.
- **Writing a gyro heading target:** wrap every `MoveGyro*` heading argument with
  `YawSource::toHwHeading(appHeadingDeg)` (static, pure `IMU_YAW_SIGN × arg`). The four sites are
  `Drive.cpp` at the `forwardGyro` emit, `MoveGyroDegs`, `MoveGyroTime`, and the unified
  `_emitContinuous` GYRO path. Add a precondition guard comment at each site: passing a raw heading
  here double-corrects.
- **Turn direction and wheel pattern:** call the free functions `normalizeTurnDelta`, `turnSign`,
  and `turnWheelPattern` (declared in `TurnSignPolicy.h` — un-namespaced inline free functions, the
  `PivotTrimPolicy.h` convention, not a `TurnSignPolicy::` class/namespace) from `Drive.cpp`; the
  logic is extracted verbatim — no value change to the pattern.
- **Config:** `Config::IMU_YAW_SIGN` lives in `config/ConfigNav.h`, guarded by
  `#ifndef IMU_YAW_SIGN_DEFAULT` (default `-1.0f` for the current CW-positive chassis); the
  two-chassis override is `-DIMU_YAW_SIGN_DEFAULT=1.0f` at `target_compile_definitions` scope
  (a per-file `#define` does not reach the separately compiled `YawSource.cpp`). The `ConfigChecks.h`
  static_assert rejects any value other than `±1.0f` — a `0.0f` or floating value is a compile
  error.

## Trigger to revisit

- A chassis whose raw IMU axis is reflashed or remapped by an STM32 firmware update that changes
  the yaw sign convention — adjust `IMU_YAW_SIGN_DEFAULT` or query the new convention.
- Adding a third frame crossing (e.g. a processed Euler from a second IMU, or a fusion filter that
  exposes its own sign) — extend `YawSource` rather than adding a new ad-hoc sign correction.
- The STM32 `TurnGyro` path is revived and written to from R4 side — the write-side `toHwHeading`
  call sites must be audited.

## See also

- [ADR-005](./005-localization-single-source-of-truth.md) — `Localization` is the single source of
  truth for heading; `YawSource` sits beneath it as the single I/O boundary for raw IMU reads.
- [ADR-004](./004-pid-stays-on-stm32.md) — motor-speed/position PID stays on the STM32; the
  `toHwHeading` conversion exists because the STM32 gyro PID holds against its own raw IMU frame.
- [ADR-036](./036-per-turn-pivot-compensation.md) — the `DirectionPolicy.h` / `PivotTrimPolicy.h`
  header-only host-test-extraction pattern that `TurnSignPolicy.h` follows.
- [ADR-031](./031-config-split-per-domain.md) — per-domain Config split; `IMU_YAW_SIGN` lives in
  `config/ConfigNav.h`; the `±1` tripwire is in `ConfigChecks.h` (cross-module guard).
