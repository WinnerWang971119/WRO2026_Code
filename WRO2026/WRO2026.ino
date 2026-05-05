/**
 * @file WRO2026.ino
 * @brief WRO 2026 Mosaic Masters — Phase 1 entry sketch.
 * @author WRO 2026
 *
 * @details
 * Phase 1 deliverable: a thin entry that wires every module together. `setup()` follows the
 * documented MatrixMiniR4 initialisation order; `loop()` delegates to `cooperativeTick()` —
 * a clean tick across Drive -> LineFollower -> Intersection -> Manipulator -> EventTrace,
 * with NO business logic. The same `cooperativeTick()` is registered with
 * `Drive.setTickCallback(...)` so that during a Drive sync-blocking spin, every other module
 * keeps running.
 *
 * Define `DEMO_MODE` at compile time (e.g. via the IDE custom-build flags or by uncommenting
 * the line below) to enable a tiny smoke routine triggered by `BTN_UP`.
 *
 * @note No business logic in `loop()`. Per ARCHITECTURE.md iron rule 1, every module's
 *       `update()` is non-blocking; the loop just dispatches them via `cooperativeTick()`.
 * @note This sketch contains no `delay()` itself. The Drive sync-default path uses
 *       `delay(2)` inside `_drainBlocking` — the documented allowed-delay site for
 *       synchronous-orchestration paths, off the cooperative tick chain.
 */

// #define DEMO_MODE   // Uncomment to enable the BTN_UP smoke routine.

#include <MatrixMiniR4.h>

#include "Config.h"
#include "Drive.h"
#include "EventTrace.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Manipulator.h"

#ifdef DEMO_MODE
namespace
{
    /**
     * @brief Distance, in centimetres, that the DEMO_MODE smoke routine drives forward.
     *
     * Lifted out of `loop()` so the sketch obeys the project's strict no-magic-numbers rule
     * even for demo-only constants.
     */
    static constexpr float DEMO_DIST_CM = 30.0f;

    /**
     * @brief Relative yaw delta, in degrees, that the DEMO_MODE smoke routine turns through
     *        after the straight-line segment.
     */
    static constexpr int16_t DEMO_TURN_DEG = 90;

    bool _demoArmed = false;    ///< Latches the one-shot demo so re-presses do nothing.
    bool _btnPrev   = false;    ///< Previous BTN_UP sample, for rising-edge detection.
}    // namespace
#endif    // DEMO_MODE

/**
 * @brief Cooperative tick — calls every module's `update()` exactly once.
 *
 * Used as both the body of `loop()` and as the registered tick callback for
 * `Drive.setTickCallback()` so that during a sync-blocking Drive command, every
 * other module continues to run (EventTrace renders, LineFollower polls the
 * tracer, Intersection latches edges, Manipulator advances its state machine).
 *
 * Order matters — see `loop()` doxygen for the rationale.
 */
void cooperativeTick()
{
    Drive.update();
    LineFollower.update();
    Intersection.update();
    Manipulator.update();
    EventTrace.update();
}

/**
 * @brief Arduino entry: bring up the lower MCU and every WRO module in dependency order.
 *
 * Order is fixed and must not be reshuffled:
 *   1. `MiniR4.begin()`                    — lower MCU, IMU, motors, servos, OLED.
 *   2. `Serial.begin(115200)`              — telemetry console.
 *   3. `MiniR4.PWR.setBattCell(...)`       — must match the physical pack.
 *   4. `Drive.begin()`                     — drivebase + PIDs + reverse flags.
 *   5. `LineFollower.begin()`              — MXLineTracer + outer-loop PID.
 *   6. `Intersection.begin()`              — pattern detector (reuses tracer).
 *   7. `Manipulator.begin()`               — Phase-3 stub kept in the chain.
 *   8. `EventTrace.begin()`                — OLED ring buffer; pushed last so the boot
 *                                            event is the first line on screen.
 */
void setup()
{
    // 1. Bring up the lower MCU, IMU, motors, servos, OLED, and the on-board LEDs.
    MiniR4.begin();

    // 2. Open the serial console for any ad-hoc telemetry.
    Serial.begin(115200);

    // 3. Tell the power module how many cells are physically installed (must match the pack).
    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // 4. Initialise our modules in dependency order.
    Drive.begin();
    LineFollower.begin();
    Intersection.begin();
    Manipulator.begin();
    EventTrace.begin();

    // Wire up the tick callback so Drive's sync-blocking builders keep every other
    // module alive during their spin. Without this, EventTrace would not render and
    // LineFollower would not poll the tracer while a Drive command is in flight.
    Drive.setTickCallback(cooperativeTick);

    // 5. Surface a boot marker so the OLED + serial confirm a clean cold-start.
    EventTrace.pushEvent("BOOT OK");
    Serial.println("WRO2026 Phase 1 ready.");
}

/**
 * @brief Arduino main loop: one cooperative tick across every module.
 *
 * The body is just `cooperativeTick();` — the same function is also registered with
 * `Drive.setTickCallback()` so it runs during sync-blocking Drive commands too. Order of
 * the calls inside `cooperativeTick()` matters:
 *   - `Drive.update()`        — flushes any completed bounded-task flags first so downstream
 *                               modules see fresh state.
 *   - `LineFollower.update()` — reads the tracer once and computes the corr.
 *   - `Intersection.update()` — reuses the same tracer read on the same tick.
 *   - `Manipulator.update()`  — currently a stub; kept in the chain so Phase 3 just fills it
 *                               in without touching this file.
 *   - `EventTrace.update()`   — samples this tick's truth from each module above.
 *
 * No business logic lives here. The optional DEMO_MODE block is the only exception. Its
 * `Drive.driveDist(...)` / `Drive.turnDeg(...)` calls are now sync-blocking by default — the
 * registered tick callback keeps every other module alive during those blocks, so the demo
 * runs to completion in a single `loop()` iteration.
 */
void loop()
{
    cooperativeTick();

#ifdef DEMO_MODE
    // Tiny smoke routine: on a BTN_UP rising edge, drive a short straight then a 90-degree
    // turn. Verifies wiring + PIDs + the line-following enable path without needing a field.
    //
    // `_demoArmed` makes the demo a one-shot per power-cycle; pressing BTN_UP again after
    // the first activation does nothing. This is intentionally defensive — re-firing the
    // demo while a bounded task from the previous run is still pending would race the
    // shared task-done flag inside `DriveClass`. Both verbs sync-block by default; BTN_UP
    // polling resumes naturally once they return.
    const bool btnNow = MiniR4.BTN_UP.getState();
    if (btnNow && !_btnPrev && !_demoArmed)
    {
        _demoArmed = true;
        EventTrace.pushEvent("DEMO RUN");
        Drive.holdYaw();
        LineFollower.setEnabled(true);
        Drive.driveDist(DEMO_DIST_CM).setPower(Config::CRUISE_SPEED).setMode(DriveMode::GYRO);
        Drive.turnDeg(DEMO_TURN_DEG);    // sync relative turn — defaults to Config::TURN_POWER, mode 0.
    }
    _btnPrev = btnNow;
#endif    // DEMO_MODE
}
