#ifndef CONFIG_MISSION_H
#define CONFIG_MISSION_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Mission DSL + Tasks configuration — reopens `namespace Config` per ADR-031.
 *
 * Holds the Mission step-queue size + re-arm hook, the Tasks registry sizing
 * (`MAX_TASKS` / `MAX_TASK_ARG_BYTES`, both `size_t`), and the halt-on-fault beep
 * sequence. `<stddef.h>` is pulled in for the `size_t` constants.
 */
namespace Config
{
    // =========================================================================================
    // Mission DSL (PR3).
    //
    // Mission is a thin, fully-manual scheduler the operator hand-writes in
    // WRO2026.ino. Each `Mission.<verb>(...)` call enqueues one step; the
    // queue advances entirely from `Mission.update()` in the cooperative
    // tick. No automatic planning — that is intentional for PR3.
    //
    // On PathTracker fault, Mission halts the chassis, beeps five long
    // tones, flashes the RGB LEDs red, and waits for BTN_DOWN to clear.
    // =========================================================================================

    /** @brief Maximum number of pending Mission steps (no dynamic alloc). */
    constexpr uint8_t MAX_MISSION_STEPS = 64;

    /**
     * @brief Bench-test switch: re-fire the mission with a fresh BTN_UP press
     *        instead of one run per power cycle.
     *
     * Production / competition default is ``false`` — BTN_UP arms the hand-authored
     * mission exactly once (see WRO2026.ino), so a stray press cannot restart the
     * route. When ``true`` (bench), once the mission returns to idle (a clean finish
     * or a cleared fault, never mid-run) a fresh BTN_UP rising edge re-arms and
     * re-runs ``buildMission()``. The ``_missionArmed`` latch is kept, so the
     * BTN_DOWN calibration menu stays suppressed after the first arm exactly as in
     * production.
     *
     * @note Override-hook: guarded by ``#ifndef MISSION_ALLOW_REARM_DEFAULT`` so the
     *       SHIPPED source default is the competition-safe ``false`` — merging this
     *       branch can never silently ship the bench value. Enable on the bench with
     *       ``-DMISSION_ALLOW_REARM_DEFAULT=true`` (arduino-cli:
     *       ``--build-property build.extra_flags=-DMISSION_ALLOW_REARM_DEFAULT=true``).
     */
#ifndef MISSION_ALLOW_REARM_DEFAULT
#define MISSION_ALLOW_REARM_DEFAULT false
#endif
    constexpr bool MISSION_ALLOW_REARM = MISSION_ALLOW_REARM_DEFAULT;

    // ---------------------------------------------------------------------------------------
    // Tasks framework
    // ---------------------------------------------------------------------------------------

    /** @brief Maximum number of task types that can be registered in the Tasks registry. */
    constexpr size_t MAX_TASKS = 16;

    /** @brief Maximum byte size of the serialised argument payload stored in a TaskArgs. */
    constexpr size_t MAX_TASK_ARG_BYTES = 16;

    /** @brief Tone frequency used for the halt-on-fault beep sequence. */
    constexpr uint16_t MISSION_FAULT_BEEP_HZ = 1000;

    /** @brief Duration of each on/off pulse in the halt-on-fault beep sequence. */
    constexpr uint16_t MISSION_FAULT_BEEP_MS = 500;

    /** @brief Number of beeps emitted before Mission awaits BTN_DOWN to clear. */
    constexpr uint8_t MISSION_FAULT_BEEPS = 5;

    /**
     * @brief Upper bound (~65 s) on a Mission ``beep`` step's duration, in ms.
     *
     * The audible tone is emitted through ``MiniR4.Buzzer.Tone`` via the
     * ``uint16_t``-duration ``_beep`` wrapper, while the BEEP advance gate
     * compares against the stored duration. Clamping the stored value to the
     * ``uint16_t`` range at the ``beep`` verb keeps the audio clock and the
     * step clock in agreement — without it, a duration above 65535 ms would
     * silence the tone after ``duration_ms % 65536`` ms while the step kept
     * blocking for the full duration.
     */
    constexpr uint32_t MISSION_BEEP_MAX_MS = 65535;
}    // namespace Config

#endif    // CONFIG_MISSION_H
