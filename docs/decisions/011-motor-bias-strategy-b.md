# ADR-011: Static motor-bias scalar (Strategy B); encoder-rate PID (Strategy C) on the shelf

**Status:** Implemented (Strategy B). C/D not built.
**Date decided:** 2026-05 (Phase 1)

## Rule

Manufacturer mismatch between the two chassis DC motors is compensated by **a static calibrated bias scalar** (`Config::MOTOR_BIAS_L` / `MOTOR_BIAS_R`) applied inside `Drive` to every raw-PWM path (`setSpeed`, `driveDist`, `driveTime` in `DriveMode::RAW`).

Calibration is a manual one-shot: `Drive.calibrateMotorBias()` spins both motors for ~2 s wheels-free, reads the encoder deltas, prints two scalars to Serial. Diego hand-edits `Config.h` and re-flashes. **No EEPROM persistence, no auto-recalibration** — the source file is the single source of truth.

The rule is model-neutral: it applies to whatever DC motor pair the chassis is currently running. The project uses two physical chassis — a **test rover** with TT motors (current dev target, `Config.h` is tuned for this) and a **competition robot "Lofaix"** with LEGO EV3 motors. The scalars do not carry between chassis; `calibrateMotorBias()` must be re-run on each one, and `Config.h` re-flashed accordingly.

Sync, Gyro, and Turn paths do **not** apply the bias — they close the loop on the STM32, and a feedforward scalar on top of an inner PID would fight the regulator.

## Why

Two DC motors of the same part number can differ by 10-20% in RPM at the same PWM, regardless of model (TT, EV3, or otherwise). That asymmetry wrecks the outer P+D line-following loop — the controller chases both the line error and the mechanical bias, and they confound each other into oscillation.

Strategy B is the simplest fix that addresses the dominant disturbance (static manufacturer mismatch). Encoder-rate PID (Strategy C) is dormant — would add roughly one UART round-trip per tick if turned on; we are not building it until B proves insufficient. Strategy A (switch line-follower to MoveSync) was rejected on read of `MMLower::Set_Drive_MoveSync_Func` — the STM32 PID slot most plausibly drives `error = rate_L - rate_R` toward zero, which would force shared rates and cancel the asymmetric L:R mix the outer P+D produces.

## How to apply

- Calibration: lift chassis → operator menu (BTN_DOWN long-press) → MOTOR → confirm with BTN_UP.
- Edit `Config::MOTOR_BIAS_L` / `MOTOR_BIAS_R` with the printed scalars; reflash.
- Every raw-PWM Drive call inside the codebase applies `_applyBias()`. Sync/Gyro/Turn carry a `// No _applyBias here — STM32 closes the loop.` comment so the discipline is visible at the call site.

## Trigger to revisit

If the chassis still oscillates after calibration produces good scalars, layer Strategy C (encoder-rate PID on the R4 side). If C is insufficient, escalate to Strategy D (cascaded gyro into the line-follow control law) — but D restructures the whole loop and stays last-resort.

When switching from the test rover to Lofaix (or vice versa), re-run `calibrateMotorBias()` from scratch — TT scalars do not transfer to EV3, and motor PPR / RPM in `Config.h` will also need updating. See issue #15 for the EV3 model swap plan.

## See also

- `WRO2026/Drive.h` — `calibrateMotorBias`, `_applyBias`
- `WRO2026/Config.h` — `MOTOR_BIAS_L` / `MOTOR_BIAS_R`
