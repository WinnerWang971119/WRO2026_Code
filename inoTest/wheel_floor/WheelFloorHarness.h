/**
 * @file WheelFloorHarness.h
 * @brief Declares the per-wheel breakaway/creep floor probe (asymmetric-chassis fix).
 * @author WRO 2026
 *
 * @details
 * See `WheelFloorHarness.cpp` for the full description. This declaration is the only thing
 * `wheel_floor.ino` needs. Kept in its own header + `.cpp` pair (not inlined into the `.ino`) for
 * the same ctags/auto-prototype reason `TurnProbeHarness.cpp` documents — see that header.
 */

#ifndef WHEEL_FLOOR_HARNESS_H
#define WHEEL_FLOOR_HARNESS_H

/**
 * @brief Measure each wheel's own breakaway and kinetic-creep PWM floor, using that wheel's
 *        encoder as the motion sensor, then confirm both move at equal rate about center.
 *
 * Gated behind a single `BTN_UP` prompt. Drives one wheel at a time through the SAME actuation the
 * turn uses (`MiniR4.DriveDC.Move`) and reads that wheel's rotation directly
 * (`MiniR4.M2/M3.getDegrees()`), so the measurement is per-wheel and needs no human judgement —
 * the encoder decides whether the wheel actually turned (the IMU could not tell the two wheels
 * apart). Reports four numbers (LEFT/RIGHT x breakaway/creep) to paste into `ConfigDrive.h`, plus
 * a counter-rotation confirmation that both wheels spin at ~equal rate. `BTN_DOWN` aborts.
 *
 * @param tickCb Cooperative-tick function pumped during every blocking wait, so the OLED/Serial
 *               and the idle modules stay serviced. Pass `cooperativeTick` from `wheel_floor.ino`.
 */
void runWheelFloor(void (*tickCb)());

#endif    // WHEEL_FLOOR_HARNESS_H
