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
 *   - Raw `drive` moves (`driveDist` / `stopAtLine` / `wallStop` /
 *     `lineFollowDist`) are recorded by the recording `Drive` and
 *     `LineFollower` builder mocks through the unified
 *     `recordDriveMove(method, DriveMoveParams)` producer, which also carries
 *     the optional `mode` / `heading` / `reverse` payload for GYRO / reverse
 *     moves.
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

#include "Config.h"   // Config::Color for the injected-mosaic echo (setInjectedMosaic).

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
        GOTO,              ///< Navigation hop. Fields: `target`, forced `arrival`/`route` direction.
        TASK_START,        ///< Task dispatch begin. Fields: `name`, `args` (16 raw bytes).
        TASK_END,          ///< Task dispatch end. Fields: `name`, `result`.
        MANIP,             ///< Manipulator command. Fields: `method`, `arg`.
        DRIVE,             ///< Raw drive move: `method`, `cm`/`deg`, `mode`/`heading`/`reverse`,
                           ///< the optional per-command speed knobs (`power`/`preset`/
                           ///< `start_speed`/`end_speed`/`profiled`/`brake`), plus the
                           ///< optional `follow_offset` lateral hug bias (lineFollowDist only).
        WAIT,              ///< Mission wait. Field: `ms`.
        PRINT,             ///< Mission print. Field: `text`.
        BEEP,              ///< Mission beep. Fields: `freq_hz`, `ms`.
        LED,               ///< Mission LED. Fields: `r`, `g`, `b`, `ms`.
        DECLARE_POSITION,  ///< Localization re-anchor. Field: `node` (reuses `target`; target node name).
        READ_MOSAIC,       ///< Vision mosaic read. Field: `success` only (colors live in the top-level `mosaic` echo).
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
        std::string arrival;                 ///< GOTO: forced arrival direction "front"|"rear"; empty = AUTO.
        std::string route;                   ///< GOTO: forced route direction "front"|"rear"; empty = AUTO.
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
        bool        success = false;         ///< READ_MOSAIC: readMosaic() return value (true == read ok).
        float       cm      = 0.0f;          ///< DRIVE: distance for the linear drive moves.
        bool        hasCm   = false;         ///< DRIVE: true iff a linear move carries `cm` (gates cm emission).
        float       deg     = 0.0f;          ///< DRIVE: angle for turnDeg / turnToYaw.
        uint8_t     arg     = 0;             ///< MANIP: command argument (servo angle / level).

        // DRIVE detail — carried by recordDriveMove; each key is emitted only
        // when its builder setter fired (see commandStreamToJson).
        std::string mode;                    ///< DRIVE: "raw"|"sync"|"gyro"; empty = unset.
        float       heading    = 0.0f;       ///< DRIVE: held heading in degrees (gyro moves).
        bool        hasHeading = false;      ///< DRIVE: true iff setHeading() was called.
        bool        reverse    = false;      ///< DRIVE: reverse-direction flag (linear moves).

        // DRIVE per-command speed knobs — carried by recordDriveMove; each key is
        // emitted only when its "was-set" sentinel is true (or, for `preset`, when
        // non-empty) — see commandStreamToJson. The recorder captures FACTS ONLY
        // (whether a setter fired and what value it carried); default resolution
        // happens Python-side.
        int16_t     power         = 0;       ///< DRIVE: .setPower() PWM override.
        bool        hasPower      = false;   ///< DRIVE: true iff setPower() was called.
        std::string preset;                  ///< DRIVE: "slow"|"normal"|"fast" from .setPreset(); empty = unset.
        int16_t     startSpeed    = 0;       ///< DRIVE: .setStartSpeed() PWM.
        bool        hasStartSpeed = false;   ///< DRIVE: true iff setStartSpeed() was called.
        int16_t     endSpeed      = 0;       ///< DRIVE: .setEndSpeed() PWM.
        bool        hasEndSpeed   = false;   ///< DRIVE: true iff setEndSpeed() was called.
        bool        profiled      = false;   ///< DRIVE: true=.tShape(), false=.flat(); valid iff hasProfiled.
        bool        hasProfiled   = false;   ///< DRIVE: true iff .tShape() or .flat() was called.
        bool        brake         = false;   ///< DRIVE: .setBrake() flag; valid iff hasBrake.
        bool        hasBrake      = false;   ///< DRIVE: true iff setBrake() was called.
        float       lineFollowOffset    = 0.0f;   ///< DRIVE: .setLineFollowOffset() signed lateral hug bias (centroid units, +LEFT/-RIGHT); valid iff hasLineFollowOffset.
        bool        hasLineFollowOffset = false;   ///< DRIVE: true iff setLineFollowOffset() was called (gates follow_offset emission).

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
     * @param target  Destination node name (resolved from the goal index via FieldMap).
     *                `nullptr` is stored as an empty string.
     * @param arrival Forced arrival-direction tag ("front"/"rear"), or `nullptr`/""
     *                for AUTO (the setter was not called); serialized only when set.
     * @param route   Forced route-direction tag ("front"/"rear"), or `nullptr`/""
     *                for AUTO; serialized only when set.
     */
    void recordGoto(const char* target, const char* arrival, const char* route);

    /**
     * @brief Record a Mission `declarePosition(name)` re-anchor op (Mission.cpp
     *        hook, success path only — after the PathTracker-busy and
     *        unknown-node guards have both passed).
     * @param name Target node name. `nullptr` is stored as an empty string.
     */
    void recordDeclarePosition(const char* name);

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
     * @brief Record a `Vision.readMosaic()` call (recorded by the Vision mock).
     *
     * Appends a single `Op::READ_MOSAIC` Record carrying @p success only — the
     * rolled colors are echoed once at the top level via @ref setInjectedMosaic,
     * so a per-op color copy would be redundant. `"op": "read_mosaic"` is the
     * record-type discriminator in the serialized stream; the record never
     * introduces a sub-field also named `op`/`kind`.
     *
     * @param success The `readMosaic()` return value (`true` == read ok).
     */
    void recordReadMosaic(bool success);

    /**
     * @brief Set the injected mosaic roll for the top-level `mosaic` trace echo.
     *
     * Called by `ctps_dump` when it runs with `--mosaic`. Stores the 12 injected
     * cell colors (echoed as color NAMES), the optional roll seed, and the
     * geometry fingerprint. Calling this arms the top-level `mosaic` block in
     * @ref commandStreamToJson; when it is never called (dump ran without
     * `--mosaic`), no `mosaic` key is emitted and the JSON is byte-identical to
     * the pre-mosaic output.
     *
     * @param colors   The 12 injected cell colors, indexed `i = row*cols + col`.
     * @param hasSeed  `true` iff a roll seed was supplied (`--mosaic-seed`
     *                 present). When `false`, the `seed` key is OMITTED from the
     *                 echo (@p seed is ignored).
     * @param seed     The roll seed, echoed verbatim; meaningful only when
     *                 @p hasSeed is `true`.
     * @param geomHash The geometry fingerprint, echoed verbatim. `nullptr` is
     *                 stored as an empty string.
     */
    void setInjectedMosaic(const Config::Color colors[12], bool hasSeed, long seed,
                           const char* geomHash);

    /**
     * @brief Captured fluent-builder state for one linear DRIVE move.
     *
     * Filled by the recording Drive / LineFollower builder mocks at their
     * destructor emission point and handed to @ref recordDriveMove. Only the
     * members relevant to the move's method are meaningful; an unset member
     * stays at the caller-supplied default and is suppressed by the serializer.
     *
     * The trailing six members are the per-command speed knobs (issue #130,
     * task T8): each pairs a captured value with a "was-set" sentinel (or, for
     * `preset`, an empty-string sentinel) so the recorder captures FACTS ONLY —
     * whether a builder setter fired and what value it carried — and never
     * bakes in a firmware default. Python resolves effective defaults later.
     * A builder that never chains any of these six setters serializes byte-
     * identical to the pre-T8 output.
     */
    struct DriveMoveParams
    {
        float       cm;            ///< Commanded distance in centimetres.
        const char* mode;          ///< "raw"|"sync"|"gyro"; nullptr/"" if setMode() unset.
        bool        hasHeading;    ///< true iff setHeading() was called on the builder.
        float       heading;       ///< Held heading in degrees (honored only under mode "gyro").
        bool        reverse;       ///< true iff setReverse(true) was called (reverse-direction move).
        int16_t     power;         ///< .setPower() PWM override; valid iff hasPower.
        bool        hasPower;      ///< true iff setPower() was called on the builder.
        const char* preset;        ///< "slow"|"normal"|"fast" from .setPreset(); nullptr/"" if unset.
        int16_t     startSpeed;    ///< .setStartSpeed() PWM; valid iff hasStartSpeed.
        bool        hasStartSpeed; ///< true iff setStartSpeed() was called on the builder.
        int16_t     endSpeed;      ///< .setEndSpeed() PWM; valid iff hasEndSpeed.
        bool        hasEndSpeed;   ///< true iff setEndSpeed() was called on the builder.
        bool        profiled;      ///< true=.tShape(), false=.flat(); valid iff hasProfiled.
        bool        hasProfiled;   ///< true iff .tShape() or .flat() was called on the builder.
        bool        brake;         ///< .setBrake() flag; valid iff hasBrake.
        bool        hasBrake;      ///< true iff setBrake() was called on the builder.

        // Line-follow lateral hug-side bias (issue #156) — the CTPS-specific
        // recording seam. Only the recording ctps_mocks LineFollower enable
        // builder fills these (the path/task/mission mocks are no-ops); the Drive
        // mocks never set an offset. Default member initializers keep every
        // existing 16-field positional aggregate init (Drive.h / the
        // followDistance path) warning-clean under -Wmissing-field-initializers —
        // an omitted trailing member with a default initializer is not "missing".
        float       lineFollowOffset    = 0.0f;   ///< .setLineFollowOffset() signed lateral hug bias (centroid units, +LEFT/-RIGHT); valid iff hasLineFollowOffset.
        bool        hasLineFollowOffset = false;  ///< true iff setLineFollowOffset() was called on the builder.
    };

    /**
     * @brief Record one linear DRIVE move (the producer for `driveDist` /
     *        `stopAtLine` / `wallStop` / `lineFollowDist`).
     *
     * Appends a single `Op::DRIVE` Record carrying @p method plus the captured
     * @p p fields. `mode` / `heading` / `reverse` / `power` / `preset` /
     * `start_speed` / `end_speed` / `profiled` / `brake` are serialized only
     * when set (non-empty mode/preset, the matching `has___` sentinel, or
     * `reverse == true`); a bare distance move with none of them set stays
     * byte-identical to the legacy output.
     *
     * @param method Drive method name (`driveDist` / `stopAtLine` / `wallStop` /
     *               `lineFollowDist`). `nullptr` is stored as an empty string.
     * @param p      Captured builder state (distance + optional mode/heading/
     *               reverse/power/preset/startSpeed/endSpeed/profiled/brake).
     */
    void recordDriveMove(const char* method, const DriveMoveParams& p);

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
