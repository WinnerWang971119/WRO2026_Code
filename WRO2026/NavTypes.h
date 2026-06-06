#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <stdint.h>

/**
 * @file NavTypes.h
 * @brief Navigation-direction types shared by PathTracker and Mission.
 *
 * Standalone header — avoids circular include between Mission.h and
 * PathTracker.h (which transitively pulls in FieldMap.h + routes.h).
 */

/**
 * @brief Direction the chassis traverses an edge.
 *
 * @note Direction::AUTO and Routes::DirectionMode::DEFAULT both encode
 *       "no override" but are distinct C++ types — never compare across them.
 */
enum class Direction : uint8_t
{
    AUTO  = 0,    ///< Defer to per-edge override → auto algorithm.
    FRONT = 1,    ///< Force forward (chassis front leads).
    REAR  = 2,    ///< Force reverse (chassis rear leads; LINE/PROJECTION use rear board).
};

/**
 * @brief Wrap a heading delta (degrees) into the half-open range (-180, 180].
 *
 * Single source of truth for the `while (d > 180) d -= 360; while (d <= -180) d += 360;`
 * loop that the direction-resolution policy (DirectionPolicy.h) and PathTracker's
 * pass-through / turn-needed checks each previously inlined.
 *
 * Kept in this dependency-light header (rather than WROMath.h, which pulls in
 * `Arduino.h` + `Config.h`) so `DirectionPolicy.h` — compiled directly by the host-test
 * target — gains the helper without new heavy includes.
 *
 * @param deltaDeg Any finite heading difference in degrees.
 * @return Equivalent delta in (-180, 180]  (so -180 maps to +180, +180 stays +180).
 */
inline float wrapDeltaDeg(float deltaDeg)
{
    while (deltaDeg > 180.0f) { deltaDeg -= 360.0f; }
    while (deltaDeg <= -180.0f) { deltaDeg += 360.0f; }
    return deltaDeg;
}

#endif    // NAV_TYPES_H
