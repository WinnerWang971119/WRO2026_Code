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
 * @brief Call-site override for stop-at-line behaviour on a BLIND hop.
 *
 * @note Tri-state precedence: a call-site ON/OFF wins over the per-edge
 *       BlindMode override; AUTO defers to the edge (mirrors Direction::AUTO).
 *       StopAtLine::AUTO and Routes::BlindMode::DEFAULT both encode "no
 *       override" but are distinct C++ types — never compare across them.
 */
enum class StopAtLine : uint8_t
{
    AUTO = 0,    ///< Defer to the per-edge BlindMode override.
    ON   = 1,    ///< Force stop-at-line on this hop's BLIND edge (overrides edge).
    OFF  = 2,    ///< Force-disable; traverse the BLIND edge as a normal dead-reckoned hop.
};

#endif    // NAV_TYPES_H
