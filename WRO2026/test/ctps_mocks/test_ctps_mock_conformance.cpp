/**
 * @file test_ctps_mock_conformance.cpp
 * @brief Compile-time mock-conformance guard for the CTPS recording builders (task T5).
 * @author WRO 2026
 *
 * @details
 * The compile-time half of the issue #89 anti-drift guard. This translation unit
 * chains EVERY current real fluent-builder setter — with its REAL argument type —
 * on a discarded temporary of each CTPS recording mock builder (`ctps_mocks/Drive.h`
 * + `ctps_mocks/LineFollower.h`). If a mock setter is REMOVED, RENAMED, or has its
 * signature made incompatible with the real call, THIS target fails to build and
 * ctest reports it (`ctps_mock_conformance`). It is an always-built ctest target, so
 * both `cmake --build` and `ctest` exercise it.
 *
 * Companion guard: the Python name-superset test (task T12) is the "real-grew" half —
 * it catches a real builder GAINING a setter the mock lacks. This TU cannot catch
 * that case: it exercises only the setters that already exist on the mock surface and
 * stays green until a human adds the new call here (the same manual step as updating
 * the mock). The two guards together deliver "can't drift again".
 *
 * KNOWN LIMIT: this TU verifies call-site ASSIGNABILITY, not exact parameter types.
 * An implicitly-convertible argument (for example a `float` where the real setter
 * takes `int`, or the reverse) still compiles, so a benign type widening/narrowing on
 * a mock setter can slip through. That is acceptable for a thin mock that only records
 * the COMMANDED value — the exact param type is not load-bearing for the trace.
 *
 * The `setMode` argument type is deliberately split: `turnDeg` / `turnToYaw` take
 * `setMode(uint8_t)`, while the four linear-move builders take `setMode(DriveMode)`.
 *
 * The mock builder destructors call `ctps::recordDriveMove`, so this target links
 * `ctps_record.cpp` (the mock `Drive` / `LineFollower` globals + the recorder) and the
 * issue #91 `routes_fixture.cpp` (the force-included mock `PathTracker.h` transitively
 * pulls `routes.h`). Each chain is one discarded-temporary statement, exactly as
 * `ctps_dump.cpp` relies on (RAII auto-execute at end-of-statement — ADR-065).
 *
 * @note HOST-ONLY translation unit — compiled into `test_ctps_mock_conformance` only,
 *       never the Arduino sketch.
 *
 * @note issue #130 (task T8): `setPower` / `setPreset` / `setStartSpeed` /
 *       `setEndSpeed` / `tShape` / `flat` / `setBrake` now STORE their argument
 *       into the mock builder's state (previously several of these were total
 *       no-ops) so the recorder can serialize the additive `power`/`preset`/
 *       `start_speed`/`end_speed`/`profiled`/`brake` `drive` JSON fields. The
 *       setter names/signatures chained below are UNCHANGED by that fix, so
 *       this TU required no edits to the chains themselves — it still only
 *       proves compile-time conformance (per the KNOWN LIMIT above), not that
 *       a setter actually stores; that runtime-capture guarantee is verified
 *       by re-reading each setter body (see the T8 task notes), not by this TU.
 */

#include "ctps_record.h"

// Recording mock facades. The `ctps_mocks/`-first include order (and the T5 cmake
// force-include wiring) resolves each of these to the mock builder surface under test;
// the mock include guards match the production ones, so the force-included copies win.
#include "Config.h"          // Config::SpeedPreset for the setPreset(...) chain setters.
#include "Drive.h"           // ctps_mocks — the six recording Drive builders + DriveMode.
#include "LineFollower.h"    // ctps_mocks — the recording LineFollowerEnableBuilder.

/**
 * @brief Chain every real setter on each mock builder to pin the mock surface.
 *
 * Each statement obtains a mock builder from a facade verb, chains every real setter
 * with its real argument type, and discards the temporary (which records once from its
 * `_executed`-guarded destructor). A removed / renamed / signature-incompatible mock
 * setter makes the corresponding chain fail to compile — the whole point of the guard.
 *
 * @return Always 0 — the guard is the COMPILE, not the runtime behaviour.
 */
int main()
{
    // DriveDistBuilder (Drive.driveDist) — 12 setters; setMode takes DriveMode.
    Drive.driveDist(30.0f)
        .setPower(int16_t(50))
        .setPreset(Config::SpeedPreset::NORMAL)
        .setMode(DriveMode::GYRO)
        .setBrake(true)
        .setBlocking(true)
        .async()
        .setHeading(int16_t(0))
        .tShape()
        .flat()
        .setAccel(1.0f)
        .setStartSpeed(int16_t(0))
        .setEndSpeed(int16_t(0));

    // DriveTimeBuilder (Drive.driveTime) — same 12 setters as DriveDist; setMode DriveMode.
    Drive.driveTime(1.0f)
        .setPower(int16_t(50))
        .setPreset(Config::SpeedPreset::NORMAL)
        .setMode(DriveMode::GYRO)
        .setBrake(true)
        .setBlocking(true)
        .async()
        .setHeading(int16_t(0))
        .tShape()
        .flat()
        .setAccel(1.0f)
        .setStartSpeed(int16_t(0))
        .setEndSpeed(int16_t(0));

    // TurnDegBuilder (Drive.turnDeg) — 6 setters; setMode takes uint8_t (NOT DriveMode).
    Drive.turnDeg(90)
        .setPower(int16_t(50))
        .setPreset(Config::SpeedPreset::NORMAL)
        .setMode(uint8_t(2))
        .setBrake(true)
        .setBlocking(true)
        .async();

    // TurnToYawBuilder (Drive.turnToYaw) — same 6 as TurnDeg; setMode takes uint8_t.
    Drive.turnToYaw(90)
        .setPower(int16_t(50))
        .setPreset(Config::SpeedPreset::NORMAL)
        .setMode(uint8_t(2))
        .setBrake(true)
        .setBlocking(true)
        .async();

    // StopAtLineBuilder (Drive.stopAtLine) — 11 setters; setMode takes DriveMode.
    Drive.stopAtLine(20.0f)
        .setPower(int16_t(50))
        .setPreset(Config::SpeedPreset::NORMAL)
        .setMode(DriveMode::GYRO)
        .setHeading(int16_t(0))
        .setArmWithin(5.0f)
        .setMaxDistance(40.0f)
        .setOvershoot(2.0f)
        .setReverse(true)
        .setBrake(true)
        .setBlocking(true)
        .async();

    // WallStopBuilder (Drive.wallStop) — 10 setters; setMode takes DriveMode.
    Drive.wallStop(10.0f)
        .setPower(int16_t(50))
        .setPreset(Config::SpeedPreset::NORMAL)
        .setMode(DriveMode::GYRO)
        .setHeading(int16_t(0))
        .setReverse(true)
        .setBackOff(3.0f)
        .setTimeout(uint32_t(4000))
        .setBrake(true)
        .setBlocking(true)
        .async();

    // LineFollowerEnableBuilder (LineFollower.setEnabled) — 12 setters.
    LineFollower.setEnabled(true)
        .setBrake(true)
        .setPreset(Config::SpeedPreset::NORMAL)
        .tShape()
        .setTShapeRampUp(true)
        .setTShapeRampDown(true)
        .flat()
        .setAccel(1.0f)
        .setReverse(true)
        .setDistanceTarget(90.0f)
        .setOvershootMode(true)
        .setBrakeOnGate(true)
        .setApproachTarget(30.0f);

    // recordDeclarePosition (issue #130, task T8) — link-and-run smoke coverage
    // for the new declare_position recorder entry point, consistent with this
    // TU's "chain and discard" style: the guard is the COMPILE (and, here,
    // successful link) of the call, not a runtime assertion.
    ctps::recordDeclarePosition("N1");

    return 0;
}
