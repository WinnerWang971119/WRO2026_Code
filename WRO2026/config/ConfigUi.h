#ifndef CONFIG_UI_H
#define CONFIG_UI_H

#include <stdint.h>

/**
 * @brief Operator diagnostics console timing/threshold constants — reopens `namespace Config`.
 *
 * Holds the OLED repaint cadence, the sensor-view sampling cadences, the menu long-press
 * threshold, the SELF TEST dwell/timeout budget, the battery-read retry policy, and the
 * gyro-drift warning threshold consumed by the operator diagnostics console (`OperatorConsole`
 * / `OperatorPanel`). Timing and thresholds only — screen/row geometry (`FRAME_ROWS`,
 * `TEXT_COLS`, ...) deliberately does NOT live here; it lives in
 * `src/operator/ConsoleTypes.h`, owned by a different layer.
 *
 * @note `<stdint.h>`-only, no `Config.h` and no `src/` include. `OperatorConsole` is a pure,
 *       host-testable core that includes this header (and `ConfigGlobal.h`) directly, never
 *       the `Config.h` umbrella — the umbrella pulls `<Arduino.h>`, which would break the
 *       core's host-linkability. Included by `Config.h` before `ConfigChecks.h` so the
 *       static_assert validators in `ConfigChecks.h` see these symbols.
 */
namespace Config
{
    // ---------------------------------------------------------------------------------------
    // OLED repaint / sensor-view sampling cadence
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Minimum interval (ms) between OLED repaints while the diagnostics console is open.
     *
     * The OLED shares Wire1 with the line array and the external gyro, so repainting on every
     * tick would starve those reads of bus time. 200 ms keeps the display visibly responsive
     * to cursor moves and live-view updates while leaving the bus free for sensor traffic the
     * rest of the tick.
     */
    constexpr uint32_t UI_REPAINT_INTERVAL_MS = 200;   // OLED shares Wire1 with line + gyro

    /**
     * @brief Refresh cadence (ms) for the PUSHED-group sample fields (battery, gyro, and the
     *        line-derived telemetry `junctionType` / `isOnline` / centroid error).
     *
     * @note Does NOT gate button polling or the boot report. Buttons are polled EVERY tick —
     *       before the closed-console early-return — so no edge is ever dropped; throttling
     *       them to this cadence would cost up to 250 ms of button latency. The boot report is
     *       pushed once at init via `setBootReport()`, not on any cadence.
     *
     * The adapter refreshes the PUSHED group — the data the console consumes without issuing
     * its own read request — on this cadence while the console is open. 250 ms is fast enough
     * for a human operator to perceive as live without re-sampling every tick.
     */
    constexpr uint32_t UI_VIEW_SAMPLE_MS = 250;   // PUSHED-group cadence

    /**
     * @brief Minimum interval (ms) between successive `READ_MOSAIC` dispatches from VISION VIEW.
     *
     * `Vision::readMosaic()` is a blocking, bus-bound read that can take up to ~240 ms to
     * complete. 750 ms leaves a comfortable margin above that worst case so a new request is
     * never issued while the previous one could still be in flight, and keeps the console
     * responsive to button input between vision samples.
     */
    constexpr uint32_t UI_VISION_SAMPLE_MS = 750;   // readMosaic() blocks up to ~240 ms

    // ---------------------------------------------------------------------------------------
    // Menu entry
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Long-press duration (ms) on `BTN_DOWN` required to open the diagnostics menu.
     *
     * @note `BTN_DOWN`, never `BTN_UP` — `BTN_UP` is the mission-arm button and must stay
     *       exclusively on the arm path (AC18/AC23). Mirrors the pre-console menu trigger.
     *
     * Unchanged from today's hard-coded `2000` in the pre-console calibration menu — kept
     * identical so the operator's muscle memory for entering the menu is not disturbed by
     * this refactor.
     */
    constexpr uint32_t UI_MENU_LONGPRESS_MS = 2000;   // unchanged from today's hard-coded 2000

    // ---------------------------------------------------------------------------------------
    // GYRO VIEW
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Drift magnitude (deg) from the view-entry baseline at which GYRO VIEW flags drift.
     *
     * Chosen as a small, easily bench-observable threshold: a healthy stationary gyro should
     * not accumulate more than a couple of degrees of apparent drift over a normal
     * diagnostics-view dwell, so 2.0 deg separates a genuinely drifting sensor from ordinary
     * read noise without the operator having to watch the raw yaw number.
     */
    constexpr float UI_GYRO_DRIFT_WARN_DEG = 2.0f;

    // ---------------------------------------------------------------------------------------
    // SELF TEST sequencer
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Minimum dwell (ms) SELF TEST holds on a check after it reaches a verdict.
     *
     * This is a MINIMUM readability floor, never a substitute for waiting on a real verdict —
     * each check still judges strictly on data freshness first. 300 ms is long enough for an
     * operator to read a one-line PASS/FAIL result before the sequencer advances to the next
     * check.
     */
    constexpr uint32_t UI_SELFTEST_STEP_MS = 300;   // MINIMUM post-verdict dwell

    /**
     * @brief Per-check freshness deadline (ms) before SELF TEST fails a check closed.
     *
     * If a check has not observed fresh data (a new sample sequence number, or an
     * execution-done notification) within this many milliseconds of entering the check, the
     * check FAILs naming the subsystem rather than stalling indefinitely. Sized comfortably
     * above one battery retry cycle plus two view-sample periods (see the paired static_assert
     * in `ConfigChecks.h`) so a healthy-but-slow read is never failed spuriously.
     */
    // per-check freshness deadline
    constexpr uint32_t UI_SELFTEST_CHECK_TIMEOUT_MS = 1500;

    // ---------------------------------------------------------------------------------------
    // Battery read retry
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Delay (ms) before re-attempting a failed battery voltage read.
     *
     * Mirrors the settle delay `setup()` already uses around `MiniR4.PWR.getBattVoltage()`,
     * converted from a blocking `delay(50)` into a `millis()`-deadline retry so the console's
     * `update()` path never blocks.
     */
    constexpr uint32_t UI_BATT_RETRY_MS = 50;

    /**
     * @brief Number of battery-read attempts (initial read plus retries) before giving up.
     *
     * Two attempts — the initial read plus one retry — matches the single-retry pattern
     * `setup()` already uses for a transient battery-read miss.
     */
    constexpr uint8_t UI_BATT_RETRY_ATTEMPTS = 2;   // initial read + one retry

}    // namespace Config

#endif    // CONFIG_UI_H
