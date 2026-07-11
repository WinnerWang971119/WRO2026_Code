/**
 * @file Vision.h (TASK-BODY HOST MOCK)
 * @brief No-op stand-in for the Vision mosaic-colour facade in `test_tasks_dispatch`.
 * @author WRO 2026
 *
 * @details
 * Shadows the production `WRO2026/src/perception/Vision.h` (same `VISION_H` guard) so a
 * `Tasks.cpp` task body that drives the `Vision` facade links WITHOUT dragging in the real,
 * `Serial1`-bound `Vision.cpp`. The `Vision` global is `inline`-defined here, so no extra
 * TU is linked — the same pattern as the sibling `task_mocks/` PathTracker / LineFollower /
 * Manipulator mocks.
 *
 * Thin and always-succeeds: `readMosaic()` returns `true` and never latches a fault. The
 * `Fault` / `FaultInfo` types and the `getColor` / `getRawHue` / fault-surface getters mirror
 * the real facade so any call site (e.g. `TasksClass::_visionRead`) compiles unchanged.
 */

// NOTE: SAME include guard (`VISION_H`) as the production header, so the real Vision.h
// no-ops if reached after this force-included shadow.
#ifndef VISION_H
#define VISION_H

#include <stdint.h>

// The REAL colour enum, single-sourced from ConfigVision.h (resolved via the `${SKETCH_DIR}`
// include path) — a hand-copied list would drift from `Config::Color`.
#include "config/ConfigVision.h"

/**
 * @brief No-op / record mock of the Vision mosaic-colour facade.
 */
class VisionClass
{
public:
    /** @brief Source of a Vision-layer fault (mirrors the production enum). */
    enum class Fault : uint8_t
    {
        NONE         = 0,
        READ_TIMEOUT = 1,
        BAD_CHECKSUM = 2,
        BAD_CELL     = 3
    };

    /** @brief Captured details of the last Vision fault (mirrors the production struct). */
    struct FaultInfo
    {
        Fault   kind         = Fault::NONE;
        uint8_t firstBadCell = 0xFF;
        uint8_t rawValue     = 0;
    };

    /** @brief No-op UART bring-up; always reports success. */
    bool begin() { return true; }

    /** @brief No-op camera wake. */
    void start() {}

    /** @brief Always reports a successful capture; the mock never faults. */
    bool readMosaic() { return true; }

    /**
     * @brief Latched colour for cell @p cell (UNKNOWN until injected via `_setInjectedColors`).
     * @return `Config::Color::UNKNOWN` for an out-of-range index (mirrors the real sentinel).
     */
    Config::Color getColor(uint8_t cell) const
    {
        if (cell >= Config::VISION_CELL_COUNT)
        {
            return Config::Color::UNKNOWN;
        }
        return _colors[cell];
    }

    /** @brief The thin mock carries no hue data — always `0` (the real safe sentinel). */
    uint8_t getRawHue(uint8_t /*cell*/) const { return 0; }

    /** @brief The mock never latches a fault. */
    bool hasFault() const { return false; }

    /** @brief Always the NONE default (the mock never faults). */
    FaultInfo getFault() const { return FaultInfo{}; }

    /** @brief No-op (nothing to clear). */
    void clearFault() {}

    /** @brief No-op tick-callback registration (matches the real facade surface). */
    void setTickCallback(void (*)()) {}

    /**
     * @brief Inject the per-cell colours returned by `getColor()` (fixture-only hook).
     * @param colors Source array of exactly `Config::VISION_CELL_COUNT` colours.
     */
    void _setInjectedColors(const Config::Color colors[Config::VISION_CELL_COUNT])
    {
        for (uint8_t i = 0; i < Config::VISION_CELL_COUNT; ++i)
        {
            _colors[i] = colors[i];
        }
    }

private:
    Config::Color _colors[Config::VISION_CELL_COUNT] = {};   ///< Injected per-cell colour latch (UNKNOWN by default).
};

/**
 * @brief Single global instance — `inline` so no separate TU is linked.
 */
inline VisionClass Vision;

#endif    // VISION_H
