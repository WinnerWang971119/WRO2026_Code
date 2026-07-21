#ifndef CONFIG_GLOBAL_H
#define CONFIG_GLOBAL_H

#include <stdint.h>

/**
 * @brief Cross-cutting global constants — reopens `namespace Config` per ADR-031 / Issue #39.
 *
 * Holds the battery cell count, the project-wide PWM saturation/deadband bounds shared by
 * every actuator path, the OLED EventTrace ring capacity + heartbeat interval, and the boot
 * feedback (buzzer + RGB LED) tones. These symbols are depended on by the domain sub-headers
 * (e.g. ConfigDrive.h's `TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE`), so this header is the first
 * sub-header pulled in by the `Config.h` umbrella.
 */
namespace Config
{
    // ---------------------------------------------------------------------------------------
    // Battery
    // ---------------------------------------------------------------------------------------

    /** @brief Battery cell count passed to `MiniR4.PWR.setBattCell()`. MUST match the pack. */
    constexpr uint8_t BATT_CELLS = 3;

    /**
     * @brief Boot-time minimum pack voltage (V). Below this, setup() fails the boot.
     *
     * `setup()` reads `MiniR4.PWR.getBattVoltage()` once (with a single retry on a
     * transient read miss) and, if the measured pack voltage is below this floor,
     * folds a `BATT LOW` failure into the same `BOOT FAIL` path as a failed module
     * begin() — red RGB LEDs + one long low tone + the OLED event ring. This keeps
     * a run from starting on a sagging pack, which drifts every open-loop move.
     *
     * @note Sized for the 3S competition pack (12.6 V full, 11.1 V nominal): 11.0 V
     *       is a "get off the field" floor above the 10.2 V empty point. A loaded
     *       serviceable pack sags below 11.9 V on the bench, so the old 11.9 V floor
     *       failed boot on a good battery; 11.0 V clears that while still catching an
     *       empty pack. MUST be re-chosen if `BATT_CELLS` changes — a 2S pack can
     *       never reach it.
     */
    constexpr float BATT_MIN_VOLTAGE_V = 11.0f;

    // =============================================================================
    // PWM bounds (saturation cap + motor deadband floor).
    //
    // Two physical properties of the DC drive motors (applies to both `MatrixTT` and `LegoEV3` profiles):
    //   * Saturation: PWM above ~95% gives no headroom for closed-loop corrections
    //     — anything beyond gets clipped by the lower MCU.
    //   * Deadband: PWM below ~20% can't break static friction; the motor stays
    //     still even though a non-zero command was issued. Any P+D correction in
    //     the (0, 20) range would be silently lost without compensation.
    //
    // `deadbandSnap()` (defined in WROMath.h) takes a raw PWM and returns a value
    // in {0} U [PWM_MIN_MAGNITUDE, PWM_MAX] (with mirrored negative range): zero
    // stays zero, non-zero values below the floor snap UP to the floor magnitude,
    // values above the cap clamp DOWN to the cap. Apply this at every actuator
    // boundary where a computed PWM hits the lower MCU.
    // =============================================================================

    /** @brief Saturation cap — any |PWM| above this gets clipped to this magnitude. */
    constexpr int16_t PWM_MAX = 95;

    /**
     * @brief Motor deadband floor — non-zero |PWM| below this can't break stiction.
     *
     * `deadbandSnap()` snaps non-zero commands UP to this floor (preserving sign)
     * so P+D corrections always result in motion. Zero commands pass through —
     * the caller intended to stop, deadband is irrelevant.
     */
    constexpr int16_t PWM_MIN_MAGNITUDE = 45;

#ifndef PWM_MOVING_MIN_DEFAULT
#define PWM_MOVING_MIN_DEFAULT 35
#endif
    /** @brief Kinetic (moving) deadband floor for the tShape/line-stop decel tail.
     *  @warning Load-bearing: consumed by Drive::_updateProfile decel, Drive::setSpeed,
     *           LineFollower stop ramp-down, ctps/profile.py, sim/runtime.py. Must stay
     *           <= PWM_MIN_MAGNITUDE. == PWM_MIN_MAGNITUDE disables the feature (identity).
     *  @note The offline parser (config_parse.py) requires the #define value to end the
     *        line — keep the bench-tune TODO on the constexpr below, not the #define. */
    constexpr int16_t PWM_MOVING_MIN = PWM_MOVING_MIN_DEFAULT;    // TODO bench-tune (Phase 3)

    // ---------------------------------------------------------------------------------------
    // OLED event trace
    // ---------------------------------------------------------------------------------------

    /** @brief Number of recent events kept in the EventTrace ring buffer for the OLED. */
    constexpr uint8_t EVENT_TRACE_CAPACITY = 4;

    /** @brief Interval at which EventTrace emits its `HB ...` status heartbeat. */
    constexpr uint32_t MISSION_HEARTBEAT_MS = 500;

    // ---------------------------------------------------------------------------------------
    // Boot feedback (buzzer + on-board RGB LED)
    //
    // The last block of setup() signals boot status. A successful boot (every module
    // begin() returned true) chirps BOOT_OK_BEEPS short bright tones and lights both
    // on-board RGB LEDs green; a failed boot emits one long low tone and lights them
    // red. Tones are emitted via MiniR4.Buzzer.Tone, which wraps the non-blocking
    // Arduino tone() — so the OK sequence delays BEEP_MS + GAP_MS between chirps to
    // keep them audibly separate. This is safe because setup() is fully blocking and
    // runs before the cooperative tick begins.
    // ---------------------------------------------------------------------------------------

    /** @brief Number of short chirps emitted on a successful boot. */
    constexpr uint8_t BOOT_OK_BEEPS = 3;

    /** @brief Frequency (Hz) of each boot-OK chirp. */
    constexpr uint16_t BOOT_OK_BEEP_HZ = 2000;

    /** @brief On-duration (ms) of each boot-OK chirp. */
    constexpr uint16_t BOOT_OK_BEEP_MS = 150;

    /** @brief Silent gap (ms) between consecutive boot-OK chirps. */
    constexpr uint16_t BOOT_OK_BEEP_GAP_MS = 100;

    /** @brief Frequency (Hz) of the single long boot-FAIL tone (low/ominous). */
    constexpr uint16_t BOOT_FAIL_BEEP_HZ = 600;

    /** @brief Duration (ms) of the single long boot-FAIL tone. */
    constexpr uint16_t BOOT_FAIL_BEEP_MS = 2000;

    // ---------------------------------------------------------------------------------------
    // Task result feedback (buzzer + on-board RGB LED)
    //
    // Tasks::_signalSuccess / _signalFail give a task an audible + visual result cue.
    // SUCCESS = one short bright chirp + green LEDs; FAIL = TASK_FAIL_BEEPS low tones
    // (same low pitch as the boot-fail tone) + red LEDs. Unlike the boot feedback these
    // run under the cooperative tick, so the on-times and gaps are spun via Tasks::_waitMs
    // (which keeps every module ticking), never a raw delay().
    // ---------------------------------------------------------------------------------------

    /** @brief Frequency (Hz) of the task-success chirp (bright/high). */
    constexpr uint16_t TASK_OK_BEEP_HZ = 2000;

    /** @brief On-duration (ms) of the task-success chirp. */
    constexpr uint16_t TASK_OK_BEEP_MS = 150;

    /** @brief Number of low tones emitted on task failure. */
    constexpr uint8_t TASK_FAIL_BEEPS = 3;

    /** @brief Frequency (Hz) of each task-fail tone (low/ominous, like boot-fail). */
    constexpr uint16_t TASK_FAIL_BEEP_HZ = 600;

    /** @brief On-duration (ms) of each task-fail tone. */
    constexpr uint16_t TASK_FAIL_BEEP_MS = 200;

    /** @brief Silent gap (ms) between consecutive task-fail tones. */
    constexpr uint16_t TASK_FAIL_BEEP_GAP_MS = 120;
}    // namespace Config

#endif    // CONFIG_GLOBAL_H
