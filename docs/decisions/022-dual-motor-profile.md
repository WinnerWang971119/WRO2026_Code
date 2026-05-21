# ADR-022: Dual motor profile via constexpr struct + ActiveMotor alias

**Status:** Implemented
**Date decided:** 2026-05 (Phase 2)

## Rule

Motor-type-dependent constants (PPR, RPM, and all PID gains) live in per-motor `constexpr struct`s inside `namespace Config` (`MatrixTT` and `LegoEV3`). The active chassis selects one via a single `using ActiveMotor = ...;` alias. The flat constants (`Config::MOTOR_PPR`, `Config::MOVESYNC_KP`, …) forward to `ActiveMotor::*` so all call sites in `Drive.cpp` are untouched when the chassis swaps. To swap chassis, edit only that one alias.

## Why

The project maintains two physical chassis: a TT-motor test rover and the LEGO EV3 competition robot "Lofaix" (see `project_two_chassis.md` user context). Before this ADR, `Config.h` was hard-coded for the TT rover across nine separate `constexpr` declarations spread over two blocks, with TT-specific wording in block comments and the file-level Doxygen. Any chassis swap required locating and editing nine values in two distinct locations — easy to half-apply, with no compile-time signal if one was missed.

Issue #15 was raised specifically because the documented motor type in `Config.h` did not match reality: Lofaix (LEGO EV3) is the actual competition hardware. The struct + alias mechanism collapses the swap to one line, makes each motor model's PID and encoder constants an explicit named table, and keeps a Doxygen `@warning` on the `ActiveMotor` declaration to remind the operator that `MOTOR_BIAS_L/R` must be re-captured separately (those are per-physical-chassis measurements, not per-motor-model — see ADR-011). Each struct's Doxygen cites the vendor source: `MatrixMiniR4 MiniR4DriveDC.h` PID docstrings for `setMoveSyncPID`, `setMoveGyroPID`, and `setTurnGyroPID`, plus `examples/4-DriveDC Functions/MiniR4_DriveDC_Test/MiniR4_DriveDC_Test.ino` lines 52–74.

## How to apply

To swap from one chassis to the other:

1. Edit `using ActiveMotor = ...;` in `WRO2026/Config.h` to either `MatrixTT` or `LegoEV3`.
2. Re-flash the R4.
3. Re-run `Drive.calibrateMotorBias()` on the new chassis and paste the new `MOTOR_BIAS_L`/`MOTOR_BIAS_R` scalars into `Config.h` — motor-bias scalars are per-physical-chassis, not per-motor-model (see ADR-011). This is Roadmap Phase 7 step P7.0c.
4. Re-tune `LINE_KP` / `LINE_KD` against the new chassis geometry. This is Roadmap Phase 7 step P7.0d.

The `@warning` on the `ActiveMotor` declaration in `Config.h` surfaces step 3 at the edit point. Steps 3 and 4 are not automated; they require hardware.

## Trigger to revisit

A third motor model is added (e.g. a brushed mid-mount motor), OR `MOTOR_BIAS_L/R` scalars start needing to live alongside the PID gains — for example, if the project ends up maintaining multiple calibrated bias sets per motor model. At that point this ADR should be superseded by one that introduces a parallel `ChassisProfile` struct (explicitly out of scope here).

## See also

- [ADR-011](./011-motor-bias-strategy-b.md) — per-chassis motor-bias calibration; explains why bias scalars stay separate from the motor-model profile.
- [ADR-017](./017-sync-plus-tshape-default-profile.md) — SYNC + tShape default profile; consumes the dispatched PID gains.
- `Roadmap.md` Phase 7 (P7.0c / P7.0d) — the calibration and line-PID retune steps that follow the alias swap.
- `WRO2026/Config.h` — `struct MatrixTT`, `struct LegoEV3`, `using ActiveMotor`, and the flat forwarder constants.
- Vendor source: `C:\Users\wangd\Documents\WRO2026\MatrixMiniR4\src\Modules\MiniR4DriveDC.h` (PID defaults in docstrings of `setMoveSyncPID`/`setMoveGyroPID`/`setTurnGyroPID`) and `examples/4-DriveDC Functions/MiniR4_DriveDC_Test/MiniR4_DriveDC_Test.ino` lines 52–74.
