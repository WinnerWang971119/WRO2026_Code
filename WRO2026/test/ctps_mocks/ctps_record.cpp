/**
 * @file ctps_record.cpp
 * @brief Definitions for the CTPS recording sink + the host-shim globals.
 * @author WRO 2026
 *
 * @details
 * Owns the storage for the shared capture buffer (`g_ctpsLog`), the build-error
 * flag, the monotonic sequence counter, and the two host-shim globals declared
 * `extern` in `Arduino.h` (the mock clock `g_ctpsMillis` and the no-op `Serial`
 * instance). Implements every `ctps::record*` helper and the Data-Model-A JSON
 * serializer.
 *
 * HOST-ONLY translation unit — compiled into `ctps_dump` only, never the sketch.
 */

#include "ctps_record.h"

// The recording-mock singletons are defined in this single shared mock TU (the
// one ctps_record.cpp that T4 links into the dump). Each mock header declares
// its global `extern`; the matching definition lives here so exactly one
// definition exists across the link. These headers pull in the REAL Config.h /
// FieldMap.h / routes.h / NavTypes.h (linked, not mocked) during the dump build.
#include "Arduino.h"        // MockSerial / mock-clock declarations this TU defines
#include "Drive.h"          // DriveClass mock
#include "EventTrace.h"     // EventTraceClass mock
#include "LineFollower.h"   // LineFollowerClass mock
#include "Localization.h"   // LocalizationClass mock
#include "Manipulator.h"    // ManipulatorClass mock
#include "MatrixMiniR4.h"   // MatrixMiniR4 facade (MiniR4 global)
#include "PathTracker.h"    // PathTrackerClass mock

#include <cstdio>       // snprintf for float formatting

namespace
{
    /// @brief Shared monotonic sequence counter — stamps every appended record.
    uint32_t g_ctpsSeq = 0;

    /**
     * @brief Append a string to @p out with JSON string-escaping (no surrounding quotes).
     *
     * Escapes the control characters and the two structural characters that
     * would otherwise break the document: `"` and `\`. Other control bytes
     * (< 0x20) are emitted as `\u00XX`. The command-stream payloads are short
     * ASCII identifiers / log lines, so this minimal escaper is sufficient and
     * always produces valid JSON.
     */
    void appendEscaped(std::string& out, const std::string& s)
    {
        static const char* kHex = "0123456789abcdef";
        for (char ch : s)
        {
            const unsigned char c = static_cast<unsigned char>(ch);
            switch (c)
            {
                case '"':  out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\b': out += "\\b";  break;
                case '\f': out += "\\f";  break;
                case '\n': out += "\\n";  break;
                case '\r': out += "\\r";  break;
                case '\t': out += "\\t";  break;
                default:
                    if (c < 0x20)
                    {
                        out += "\\u00";
                        out += kHex[(c >> 4) & 0x0F];
                        out += kHex[c & 0x0F];
                    }
                    else
                    {
                        out += static_cast<char>(c);
                    }
                    break;
            }
        }
    }

    /**
     * @brief Append a JSON `"key": "value"` string member (value escaped).
     */
    void appendStrMember(std::string& out, const char* key, const std::string& value)
    {
        out += '"';
        out += key;
        out += "\": \"";
        appendEscaped(out, value);
        out += '"';
    }

    /**
     * @brief Format a float as a compact, round-trippable JSON number.
     *
     * Uses `%g` so integral values render as `15` and fractional values keep
     * enough precision (`-90.0` → `-90`, `15.5` → `15.5`). The loader parses
     * these as plain JSON numbers.
     */
    std::string floatToJson(float v)
    {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(v));
        return std::string(buf);
    }

    /**
     * @brief Append the 16 raw TaskArgs bytes as a lowercase hex string (host-endian).
     */
    void appendArgsHex(std::string& out, const std::vector<uint8_t>& args)
    {
        static const char* kHex = "0123456789abcdef";
        for (uint8_t byte : args)
        {
            out += kHex[(byte >> 4) & 0x0F];
            out += kHex[byte & 0x0F];
        }
    }

    /**
     * @brief Stamp @p rec with the next sequence number and append it to the log.
     */
    void push(ctps::Record&& rec)
    {
        rec.seq = g_ctpsSeq++;
        ctps::g_ctpsLog.push_back(std::move(rec));
    }
}    // namespace

// =============================================================================
// Host-shim globals declared `extern` in Arduino.h.
// =============================================================================

/// @brief Mock millisecond clock. The dump advances it each tick via
///        `mockAdvanceMillis(...)` so Mission's WAIT / BEEP / LED `now - start
///        >= duration` gates complete on the next tick (instant-completion).
uint32_t g_ctpsMillis = 0;

/// @brief No-op Serial instance — the dump needs no Serial text output.
MockSerial Serial;

/// @brief No-op MatrixMiniR4 facade instance backing the `MiniR4` global.
MatrixMiniR4 MiniR4;

// -----------------------------------------------------------------------------
// Recording-mock facade singletons. Each mock header declares these `extern`;
// these are their single definitions for the dump link.
// -----------------------------------------------------------------------------

/// @brief Recording Drive mock singleton.
DriveClass Drive;

/// @brief Recording PathTracker mock singleton.
PathTrackerClass PathTracker;

/// @brief No-op LineFollower mock singleton.
LineFollowerClass LineFollower;

/// @brief No-op EventTrace mock singleton.
EventTraceClass EventTrace;

/// @brief Recording Manipulator mock singleton.
ManipulatorClass Manipulator;

/// @brief No-op Localization mock singleton (Mission.declarePosition target).
LocalizationClass Localization;

// =============================================================================
// ctps:: recording sink.
// =============================================================================

namespace ctps
{
    std::vector<Record> g_ctpsLog;
    bool                g_ctpsBuildError = false;

    void recordGoto(const char* target, const char* arrival, const char* route)
    {
        Record rec;
        rec.op      = Op::GOTO;
        rec.target  = (target != nullptr) ? target : "";
        rec.arrival = (arrival != nullptr) ? arrival : "";
        rec.route   = (route != nullptr) ? route : "";
        push(std::move(rec));
    }

    void recordWait(uint32_t ms)
    {
        Record rec;
        rec.op = Op::WAIT;
        rec.ms = ms;
        push(std::move(rec));
    }

    void recordPrint(const char* text)
    {
        Record rec;
        rec.op   = Op::PRINT;
        rec.text = (text != nullptr) ? text : "";
        push(std::move(rec));
    }

    void recordBeep(uint16_t freq_hz, uint32_t ms)
    {
        Record rec;
        rec.op      = Op::BEEP;
        rec.freq_hz = freq_hz;
        rec.ms      = ms;
        push(std::move(rec));
    }

    void recordLed(uint8_t r, uint8_t g, uint8_t b, uint32_t ms)
    {
        Record rec;
        rec.op = Op::LED;
        rec.r  = r;
        rec.g  = g;
        rec.b  = b;
        rec.ms = ms;
        push(std::move(rec));
    }

    void recordTaskStart(const char* name, const uint8_t* args, size_t n)
    {
        Record rec;
        rec.op   = Op::TASK_START;
        rec.name = (name != nullptr) ? name : "";
        rec.args.assign(n, 0);
        if (args != nullptr)
        {
            for (size_t i = 0; i < n; ++i)
            {
                rec.args[i] = args[i];
            }
        }
        push(std::move(rec));
    }

    void recordTaskEnd(const char* name, bool ok)
    {
        Record rec;
        rec.op     = Op::TASK_END;
        rec.name   = (name != nullptr) ? name : "";
        rec.result = ok;
        push(std::move(rec));
    }

    void recordManip(const char* method, uint8_t arg)
    {
        Record rec;
        rec.op     = Op::MANIP;
        rec.method = (method != nullptr) ? method : "";
        rec.arg    = arg;
        push(std::move(rec));
    }

    void recordDriveMove(const char* method, const DriveMoveParams& p)
    {
        Record rec;
        rec.op         = Op::DRIVE;
        rec.method     = (method != nullptr) ? method : "";
        rec.cm         = p.cm;
        rec.hasCm      = true;
        rec.mode       = (p.mode != nullptr) ? p.mode : "";
        rec.hasHeading = p.hasHeading;
        rec.heading    = p.heading;
        rec.reverse    = p.reverse;
        push(std::move(rec));
    }

    void recordTurnDeg(float deg)
    {
        Record rec;
        rec.op     = Op::DRIVE;
        rec.method = "turnDeg";
        rec.deg    = deg;
        push(std::move(rec));
    }

    void recordTurnToYaw(float deg)
    {
        Record rec;
        rec.op     = Op::DRIVE;
        rec.method = "turnToYaw";
        rec.deg    = deg;
        push(std::move(rec));
    }

    void recordDriveStop()
    {
        Record rec;
        rec.op     = Op::DRIVE;
        rec.method = "stop";
        push(std::move(rec));
    }

    void recordBuildError()
    {
        g_ctpsBuildError = true;
    }

    void reset()
    {
        g_ctpsLog.clear();
        g_ctpsSeq        = 0;
        g_ctpsBuildError = false;
    }

    std::string commandStreamToJson(const char* startNode, double startHeadingDeg)
    {
        std::string out;
        out.reserve(256 + g_ctpsLog.size() * 64);

        out += "{\n";
        out += "  \"schema_version\": 1,\n";

        out += "  ";
        appendStrMember(out, "start_node", (startNode != nullptr) ? startNode : "");
        out += ",\n";

        {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%g", startHeadingDeg);
            out += "  \"start_heading_deg\": ";
            out += buf;
            out += ",\n";
        }

        out += "  \"build_error\": ";
        out += (g_ctpsBuildError ? "true" : "false");
        out += ",\n";

        out += "  \"commands\": [";
        for (size_t i = 0; i < g_ctpsLog.size(); ++i)
        {
            const Record& rec = g_ctpsLog[i];
            out += (i == 0) ? "\n" : ",\n";
            out += "    { \"seq\": ";
            out += std::to_string(rec.seq);
            out += ", ";

            switch (rec.op)
            {
                case Op::GOTO:
                    out += "\"op\": \"goto\", ";
                    appendStrMember(out, "target", rec.target);
                    // Optional forced-direction keys — each emitted only when the
                    // builder's setArrivalDirection / setRouteDirection fired (a
                    // non-AUTO override). A direction-free goto stays byte-identical
                    // to the legacy output (mirrors the DRIVE detail keys).
                    if (!rec.arrival.empty())
                    {
                        out += ", ";
                        appendStrMember(out, "arrival", rec.arrival);
                    }
                    if (!rec.route.empty())
                    {
                        out += ", ";
                        appendStrMember(out, "route", rec.route);
                    }
                    break;

                case Op::WAIT:
                    out += "\"op\": \"wait\", \"ms\": ";
                    out += std::to_string(rec.ms);
                    break;

                case Op::PRINT:
                    out += "\"op\": \"print\", ";
                    appendStrMember(out, "text", rec.text);
                    break;

                case Op::BEEP:
                    out += "\"op\": \"beep\", \"freq_hz\": ";
                    out += std::to_string(rec.freq_hz);
                    out += ", \"ms\": ";
                    out += std::to_string(rec.ms);
                    break;

                case Op::LED:
                    out += "\"op\": \"led\", \"r\": ";
                    out += std::to_string(static_cast<unsigned>(rec.r));
                    out += ", \"g\": ";
                    out += std::to_string(static_cast<unsigned>(rec.g));
                    out += ", \"b\": ";
                    out += std::to_string(static_cast<unsigned>(rec.b));
                    out += ", \"ms\": ";
                    out += std::to_string(rec.ms);
                    break;

                case Op::TASK_START:
                    out += "\"op\": \"task_start\", ";
                    appendStrMember(out, "name", rec.name);
                    out += ", \"args_hex\": \"";
                    appendArgsHex(out, rec.args);
                    out += '"';
                    break;

                case Op::TASK_END:
                    out += "\"op\": \"task_end\", ";
                    appendStrMember(out, "name", rec.name);
                    out += ", ";
                    appendStrMember(out, "result", rec.result ? "ok" : "yield");
                    break;

                case Op::MANIP:
                    out += "\"op\": \"manip\", ";
                    appendStrMember(out, "method", rec.method);
                    out += ", \"arg\": ";
                    out += std::to_string(static_cast<unsigned>(rec.arg));
                    break;

                case Op::DRIVE:
                    out += "\"op\": \"drive\", ";
                    appendStrMember(out, "method", rec.method);
                    // Linear moves carry a distance; turns carry an angle; stop
                    // (and driveTime) carry neither. Gate cm emission on the
                    // record's own `hasCm` flag (set by the sole producer,
                    // recordDriveMove) rather than a method-name list — a new
                    // linear method must not be able to silently drop cm by being
                    // forgotten here (the original wallStop bug).
                    if (rec.hasCm)
                    {
                        out += ", \"cm\": ";
                        out += floatToJson(rec.cm);
                    }
                    else if (rec.method == "turnDeg" || rec.method == "turnToYaw")
                    {
                        out += ", \"deg\": ";
                        out += floatToJson(rec.deg);
                    }
                    // Optional drive-detail keys — each emitted only when its
                    // builder setter fired (see recordDriveMove).
                    if (!rec.mode.empty())
                    {
                        out += ", ";
                        appendStrMember(out, "mode", rec.mode);
                    }
                    if (rec.hasHeading)
                    {
                        out += ", \"heading\": ";
                        out += floatToJson(rec.heading);
                    }
                    if (rec.reverse)
                    {
                        out += ", \"reverse\": true";
                    }
                    break;
            }

            out += " }";
        }
        out += (g_ctpsLog.empty() ? "]\n" : "\n  ]\n");
        out += "}\n";

        return out;
    }

}    // namespace ctps
