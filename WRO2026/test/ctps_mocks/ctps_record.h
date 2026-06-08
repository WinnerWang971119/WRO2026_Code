/**
 * @file ctps_record.h
 * @brief Shared recording sink for the CTPS host-build mocks (Mechanism #4, task T3).
 * @author WRO 2026
 *
 * @details
 * CTPS (Clear Trajectory Prediction Simulator) links the REAL `Mission.cpp` +
 * `Tasks.cpp` + `MissionScript.cpp` + `routes.cpp` on a host x86 toolchain
 * against recording mocks of every hardware facade. Running the resulting
 * `ctps_dump` executes a mission to completion and captures an ORDERED stream
 * of high-level commands (`goto` / `drive` / `manip` / `wait` / `print` /
 * `beep` / `led` / `task_start` / `task_end`) into the shared `g_ctpsLog`.
 * Task T4 serializes that log to the Data-Model-A command-stream JSON via
 * `commandStreamToJson(...)`.
 *
 * This is HOST-ONLY code: it freely uses `std::vector` / `std::string`, which
 * are unavailable / undesirable on the firmware target. None of this header is
 * ever compiled into the Arduino sketch — it is reached only from the CTPS
 * recording mocks (which themselves only shadow the real headers during the
 * dump build) and from `ctps_dump.cpp`.
 *
 * Recording division (Option A — see the CTPS plan, task T3):
 *   - Mission-level verbs with no clean facade (`wait` / `print` / `beep` /
 *     `led` / `task_start` / `task_end` / `build_error`) are recorded by a
 *     small `#ifdef CTPS_RECORD` hook inside `Mission.cpp`.
 *   - `goto` is recorded by the recording `PathTracker.goTo` mock (single seam;
 *     Mission.cpp must NOT also record it or it would double-count).
 *   - Raw `drive` moves are recorded by the recording `Drive` builder mocks.
 *   - `manip` calls are recorded by the recording `Manipulator` mock.
 *
 * Every recorder appends exactly one `Record` to `g_ctpsLog` and stamps it with
 * the next value of a shared monotonic sequence counter, so execution order is
 * preserved across all seams (e.g. `task_start` → [drive/manip emitted by the
 * task body] → `task_end` interleave naturally).
 */

#ifndef CTPS_RECORD_H
#define CTPS_RECORD_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ctps
{
    /**
     * @brief Operation discriminator for a captured command-stream entry.
     *
     * Mirrors the `op` vocabulary of Data Model A. Each `Record` carries only
     * the fields relevant to its `op`; the others stay at their defaults and
     * are not emitted by `commandStreamToJson`.
     */
    enum class Op : uint8_t
    {
        GOTO,          ///< Navigation hop. Field: `target`.
        TASK_START,    ///< Task dispatch begin. Fields: `name`, `args` (16 raw bytes).
        TASK_END,      ///< Task dispatch end. Fields: `name`, `result`.
        MANIP,         ///< Manipulator command. Fields: `method`, `arg`.
        DRIVE,         ///< Raw drive primitive. Fields: `method`, plus `cm`/`deg` per method.
        WAIT,          ///< Mission wait. Field: `ms`.
        PRINT,         ///< Mission print. Field: `text`.
        BEEP,          ///< Mission beep. Fields: `freq_hz`, `ms`.
        LED,           ///< Mission LED. Fields: `r`, `g`, `b`, `ms`.
    };

    /**
     * @brief One captured command-stream entry.
     *
     * A flat record paying the worst-case field set for every `op`; only the
     * fields named in `Op`'s docstring for a given `op` are populated and
     * emitted. `seq` is assigned in append order by the recorder helpers.
     */
    struct Record
    {
        uint32_t    seq     = 0;             ///< Shared monotonic sequence number (append order).
        Op          op      = Op::PRINT;     ///< Which operation this record captures.

        // String payloads. Copied into owning std::strings so the record stays
        // valid regardless of the lifetime of the source pointer (Mission
        // stores step text/name by pointer into static storage, but copying is
        // the robust choice and costs nothing on the host).
        std::string target;                  ///< GOTO: destination node name.
        std::string text;                    ///< PRINT: message text.
        std::string name;                    ///< TASK_START / TASK_END: task name.
        std::string method;                  ///< MANIP / DRIVE: facade method name.

        // Scalar payloads.
        uint32_t    ms      = 0;             ///< WAIT / BEEP / LED: duration in ms.
        uint16_t    freq_hz = 0;             ///< BEEP: tone frequency in Hz.
        uint8_t     r       = 0;             ///< LED: red channel.
        uint8_t     g       = 0;             ///< LED: green channel.
        uint8_t     b       = 0;             ///< LED: blue channel.
        bool        result  = false;         ///< TASK_END: task body return value (true == ok).
        float       cm      = 0.0f;          ///< DRIVE: distance for driveDist / stopAtLine.
        float       deg     = 0.0f;          ///< DRIVE: angle for turnDeg / turnToYaw.
        uint8_t     arg     = 0;             ///< MANIP: command argument (servo angle / level).

        // TASK_START: the raw 16-byte packed TaskArgs blob, captured verbatim
        // so the loader can decode per-task structs later. Always exactly
        // Config::MAX_TASK_ARG_BYTES (16) bytes for a TASK_START record.
        std::vector<uint8_t> args;           ///< TASK_START: raw packed argument bytes.
    };

    /**
     * @brief The single shared, ordered capture buffer.
     *
     * Every recording mock + the Mission.cpp hook append here. Defined in
     * `ctps_record.cpp`. Cleared by `reset()`.
     */
    extern std::vector<Record> g_ctpsLog;

    /**
     * @brief Sticky build-error flag, set by `recordBuildError()`.
     *
     * Mirrors `MissionClass::_buildError`: the Mission.cpp hook calls
     * `recordBuildError()` from `start()`'s build-error branch. Serialized into
     * the `build_error` field of the command-stream JSON. Defined in
     * `ctps_record.cpp`; cleared by `reset()`.
     */
    extern bool g_ctpsBuildError;

    // -------------------------------------------------------------------------
    // Recording API — one helper per captured op. Each appends exactly one
    // Record to g_ctpsLog with the next sequence number.
    // -------------------------------------------------------------------------

    /**
     * @brief Record a navigation hop (recorded by the PathTracker.goTo mock only).
     * @param target Destination node name (resolved from the goal index via FieldMap).
     *               `nullptr` is stored as an empty string.
     */
    void recordGoto(const char* target);

    /**
     * @brief Record a Mission `wait` step.
     * @param ms Wait duration in milliseconds.
     */
    void recordWait(uint32_t ms);

    /**
     * @brief Record a Mission `print` step.
     * @param text Message text. `nullptr` is stored as an empty string.
     */
    void recordPrint(const char* text);

    /**
     * @brief Record a Mission `beep` step.
     * @param freq_hz Tone frequency in Hz.
     * @param ms      Tone duration in milliseconds.
     */
    void recordBeep(uint16_t freq_hz, uint32_t ms);

    /**
     * @brief Record a Mission `led` step.
     * @param r  Red channel (0-255).
     * @param g  Green channel (0-255).
     * @param b  Blue channel (0-255).
     * @param ms Hold duration in milliseconds (0 = hold until next LED step).
     */
    void recordLed(uint8_t r, uint8_t g, uint8_t b, uint32_t ms);

    /**
     * @brief Record a task-dispatch begin (Mission.cpp hook, before runTask).
     * @param name Task registry name. `nullptr` is stored as an empty string.
     * @param args Pointer to the packed argument bytes (Mission's `TaskArgs::bytes`).
     *             May be `nullptr`, in which case `n` zero bytes are stored.
     * @param n    Number of argument bytes (always `sizeof(TaskArgs::bytes)` == 16).
     */
    void recordTaskStart(const char* name, const uint8_t* args, size_t n);

    /**
     * @brief Record a task-dispatch end (Mission.cpp hook, after runTask returns).
     * @param name Task registry name. `nullptr` is stored as an empty string.
     * @param ok   The task body's return value (`true` == "ok", `false` == "yield").
     */
    void recordTaskEnd(const char* name, bool ok);

    /**
     * @brief Record a Manipulator command (recorded by the Manipulator mock).
     * @param method Facade method name: a production verb (`grab` / `release` /
     *               `dump` / `sortTo` / `reset`) or a fixture-only setter
     *               (`setLift` / `setGripper` / `setArm`). `nullptr` is stored as
     *               an empty string.
     * @param arg    Command argument (`sortTo` bin / servo angle / lift level; the
     *               arg-less verbs record `0`).
     */
    void recordManip(const char* method, uint8_t arg);

    /**
     * @brief Record a `Drive.driveDist(cm)` raw move (recorded by the Drive mock).
     * @param cm Requested distance in centimetres.
     */
    void recordDriveDist(float cm);

    /**
     * @brief Record a `Drive.turnDeg(deg)` relative turn (recorded by the Drive mock).
     * @param deg Relative yaw delta in degrees.
     */
    void recordTurnDeg(float deg);

    /**
     * @brief Record a `Drive.turnToYaw(deg)` absolute turn (recorded by the Drive mock).
     * @param deg Absolute world heading in degrees.
     */
    void recordTurnToYaw(float deg);

    /**
     * @brief Record a `Drive.stop(...)` continuous-stop (recorded by the Drive mock).
     *
     * Emitted with `method == "stop"` and no geometry argument — the loader
     * treats it as a non-geometry drive primitive.
     */
    void recordDriveStop();

    /**
     * @brief Record a `Drive.stopAtLine(cm)` seek (recorded by the Drive mock).
     * @param cm Nominal expected distance to the crossing line, in centimetres.
     */
    void recordStopAtLine(float cm);

    /**
     * @brief Set the sticky build-error flag (Mission.cpp hook, `start()` branch).
     */
    void recordBuildError();

    /**
     * @brief Clear the log, the sequence counter, and the build-error flag.
     *
     * Call once before running a mission so a fresh dump starts from `seq == 0`.
     */
    void reset();

    /**
     * @brief Serialize `g_ctpsLog` to the Data-Model-A command-stream JSON.
     *
     * @param startNode       Name of the start node (informational `start_node`).
     *                        `nullptr` is emitted as the empty string.
     * @param startHeadingDeg Start heading in degrees (informational
     *                        `start_heading_deg`).
     * @return A complete, valid JSON document (object with `schema_version`,
     *         `start_node`, `start_heading_deg`, `build_error`, and the ordered
     *         `commands` array). `args_hex` is the lowercase hex of the 16 raw
     *         `TaskArgs` bytes in host-endian order.
     */
    std::string commandStreamToJson(const char* startNode, double startHeadingDeg);

}    // namespace ctps

#endif    // CTPS_RECORD_H
