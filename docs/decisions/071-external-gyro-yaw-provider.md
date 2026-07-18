# ADR-071: External gyro yaw provider — two independent frame-crossing signs

**Status:** Implemented
**Date decided:** 2026-07 (Phase 2 hardening)

## Rule

`YawSource` now bridges the app-frame yaw across **two physically independent device frames**,
each with its own sign constant applied exactly once: `Config::GYRO_YAW_SIGN` on the external-gyro
**read** (external gyro raw → app CCW-positive, folded into the scale multiply inside
`GyroDecodePolicy.h::decodeGyroYawDeg`, before the wrap), and `Config::IMU_YAW_SIGN` on the STM32
gyro-PID **write** (`YawSource::toHwHeading`, app → STM32 raw, unchanged). One constant can no
longer serve both directions, because the external gyro and the STM32 IMU are separate boards with
independent, unrelated handedness — collapsing them into a single sign would double-correct one
path or leave the other uncorrected.

## Why

This is precisely the "third frame crossing" ADR-045's *Trigger to revisit* anticipated: "adding a
third frame crossing (e.g. a processed Euler from a second IMU, or a fusion filter that exposes its
own sign) — extend `YawSource` rather than adding a new ad-hoc sign correction." The yaw **read**
provider moved from the on-board STM32 IMU to an external MXColor-protocol gyro board (I2C addr
`0x22`, mux channel `Config::GYRO_I2C_PORT - 1`), read via the same `MatrixColor` driver
`WROLineArray` already drives — no new I2C class. `YawSource` decodes the board's raw signed 16-bit
heading directly from its two colour registers (`G` high byte, `R` low byte), rather than reading
STM32 Euler yaw.

Reads are gated ONLY by dead-byte detection (`G == 0 && R == 0` ⇒ hard failure, `yawOut`
untouched) — a runtime plausibility/torn-read gate was deferred to bench verification: the
known-good `meat.ino` reference ships with no such gate, and an earlier attempt at one rejected the
FSM host-test mocks that teleport heading between hops. That gap is tracked by **#170**.
(Background only: the *retired* hue-derived STM32 read had its own failure mode — `RGB2HSV` set
`h = 0` on a degenerate/gray reading, aliasing a legitimate 0° heading. The two-register raw read
sidesteps that class of bug entirely; it is not the reason for this ADR.)

## How to apply

- **External-gyro read:** `GYRO_YAW_SIGN` / `GYRO_SCALE` are measured on the bench with the
  standalone `inoTest/gyro_calib` sketch and pasted into `config/ConfigNav.h`. The sign is applied
  exactly once, inside `GyroDecodePolicy.h::decodeGyroYawDeg`, in the same multiply as the scale and
  before the wrap — never at the call site.
- **STM32 write:** `IMU_YAW_SIGN` and `YawSource::toHwHeading` are unchanged from ADR-045; every
  `MoveGyro*` heading argument still routes through `toHwHeading`.
- **Containment:** the external-gyro transport, cache, and both signs stay inside `YawSource`. No
  other module reads the gyro board, reads the STM32 IMU for yaw, or applies either sign — `Drive`
  and `Localization` keep consuming the app-frame CCW-positive yaw through the unchanged
  `readYawDeg` / `readYawDegOr` contract.

## Trigger to revisit

- The STM32 gyro-PID **write** path still steers on the STM32's own onboard IMU while yaw is now
  **read** from the external gyro — an accepted, tracked split-brain (**#169**): a `MoveGyro*`
  target derived from the external gyro's zero is serviced against the STM32's own zero, so the
  first GYRO move after this change is wrong by a large angle, not a slow drift. The fix (an R4-side
  heading hold that replaces the STM32 gyro-PID write) is in progress on another branch; when it
  lands, `toHwHeading` and `IMU_YAW_SIGN` likely retire and this ADR should be revisited.
- If bench testing (#170) shows an actual torn read (a garbled high/low byte pair spanning two
  samples), re-add the plausibility gate — `GyroDecodePolicy.h::gyroReadPlausible` already exists,
  is pure, and is unit-tested; it is simply not called by `YawSource` yet.

## See also

- [ADR-045](./045-imu-yaw-sign-normalization.md) — the single-owner `YawSource` / one-sign-per-frame
  rule this ADR extends to a second, independent device frame; not superseded.
- [ADR-019](./019-8channel-custom-line-sensor.md) — the `WROLineArray` / `MatrixColor` MXColor
  protocol driver this ADR's gyro read reuses.
- Issue #169 — GYRO write-path split-brain (STM32 IMU write vs. external-gyro read), tracked
  follow-up.
- Issue #170 — bench verification of the deferred torn-read plausibility gate.
