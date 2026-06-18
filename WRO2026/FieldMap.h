/**
 * @file FieldMap.h
 * @brief Header-only namespace of inline accessors over the auto-generated `Routes::` data.
 * @author WRO 2026
 *
 * @details
 * `FieldMap` is the read-only abstraction the rest of the runtime uses
 * to consult the precomputed graph. Owns no state, no `begin()`, no
 * `update()`, no `extern` instance — every getter is an inline lookup
 * into the `Routes::` arrays defined in `routes.cpp`.
 *
 * PR3 simplification: the planner now emits only `NODES`, `NEXT_HOP`,
 * and `EDGES`. The PR2 `GoalApproach` struct + `BlindAction` enum are
 * gone — off-line goals are first-class blind nodes in the graph and
 * the per-node `Mission` DSL drives any post-arrival actions.
 */

#ifndef FIELDMAP_H
#define FIELDMAP_H

#include <stdint.h>
#include <string.h>

#include "routes.h"

namespace FieldMap
{
    /** @brief Number of nodes in the field graph (re-export of `Routes::N_NODES`). */
    constexpr uint8_t N_NODES = Routes::N_NODES;

    /** @brief Sentinel for "not found / unreachable" (mirrors `Routes::NEXT_HOP_NONE`). */
    constexpr uint8_t NODE_NONE = Routes::NEXT_HOP_NONE;

    /**
     * @brief Safe sentinel returned when `getEdge()` is asked for an out-of-range index.
     *
     * `EdgeType::NONE` (= 0) marks "no edge"; distance/heading/overrides are all zeroed so the
     * dispatcher treats it as a missing edge rather than a real maneuver. Function-local static
     * so the const-reference return stays valid for the caller's lifetime (ODR-safe in a header).
     */
    inline const Routes::EdgeInfo& noEdge()
    {
        static const Routes::EdgeInfo kNoEdge = {
            Routes::EdgeType::NONE,        // type
            0.0f,                          // distance_cm
            0.0f,                          // heading_deg
            Routes::LineMode::DEFAULT,     // line_mode_override
            Routes::BlindMode::DEFAULT,    // blind_mode_override
            Routes::ProjectionMode::DEFAULT, // projection_mode_override
            Routes::DirectionMode::DEFAULT,  // direction_override
            Routes::StopAtLineMode::DEFAULT  // stop_at_line_override
        };
        return kNoEdge;
    }

    /**
     * @brief Safe sentinel returned when `getNode()` is asked for an out-of-range id.
     *
     * Coordinates are zeroed, `type` is `NodeType::LINE` (the enum has no dedicated "none"
     * value), and `name` is the empty string so consumers that `strcmp` the name never
     * dereference a null pointer. Function-local static for a valid const-reference return.
     */
    inline const Routes::NodeInfo& noNode()
    {
        static const Routes::NodeInfo kNoNode = {
            0.0f,                  // x_cm
            0.0f,                  // y_cm
            Routes::NodeType::LINE, // type
            ""                     // name
        };
        return kNoNode;
    }

    /**
     * @brief Read the next-hop entry from the Dijkstra-precomputed table.
     *
     * @return `NODE_NONE` if either index is out of range (defensive — guards `NODE_NONE`
     *         (0xFF) and any id `>= N_NODES` from reading past the fixed-size table).
     */
    inline uint8_t getNextHop(uint8_t from, uint8_t goal)
    {
        if (from >= N_NODES || goal >= N_NODES)
        {
            return NODE_NONE;
        }
        return Routes::NEXT_HOP[from][goal];
    }

    /**
     * @brief Read the directed edge metadata between two nodes.
     *
     * @return A zeroed `EdgeInfo` (`EdgeType::NONE`) if either index is out of range.
     */
    inline const Routes::EdgeInfo& getEdge(uint8_t from, uint8_t to)
    {
        if (from >= N_NODES || to >= N_NODES)
        {
            return noEdge();
        }
        return Routes::EDGES[from][to];
    }

    /**
     * @brief Convenience accessor for the edge type only.
     *
     * @return `EdgeType::NONE` if either index is out of range.
     */
    inline Routes::EdgeType getEdgeType(uint8_t from, uint8_t to)
    {
        // Delegate the bounds check to getEdge(): noEdge().type is EdgeType::NONE, so an
        // out-of-range query still reports NONE without duplicating the guard here.
        return getEdge(from, to).type;
    }

    /**
     * @brief Read a node descriptor by id.
     *
     * @return The `noNode()` sentinel if `id` is out of range.
     */
    inline const Routes::NodeInfo& getNode(uint8_t id)
    {
        if (id >= N_NODES)
        {
            return noNode();
        }
        return Routes::NODES[id];
    }

    /**
     * @brief Convenience accessor for the node category only.
     *
     * @return The `noNode()` sentinel's type (`NodeType::LINE`) if `id` is out of range.
     */
    inline Routes::NodeType getNodeType(uint8_t id)
    {
        // Delegate the bounds check to getNode(). NodeType has no dedicated "none" value,
        // so an out-of-range id reports noNode().type (NodeType::LINE) — indistinguishable
        // from a real LINE node. Callers that must tell the two apart should range-check
        // the id (`id < N_NODES`) before calling, or use getNode() and inspect the name.
        return getNode(id).type;
    }

    /**
     * @brief Resolve a node name to its id (linear scan, returns NODE_NONE if not found).
     */
    inline uint8_t findNodeByName(const char* name)
    {
        if (name == nullptr)
        {
            return NODE_NONE;
        }
        for (uint8_t i = 0; i < N_NODES; ++i)
        {
            if (strcmp(Routes::NODES[i].name, name) == 0)
            {
                return i;
            }
        }
        return NODE_NONE;
    }

    /**
     * @brief Resolve the single-wheel pivot record for a turn (arrive-from `u`, pivot-at
     *        `v`, leave-to `w`).
     *
     * @param u Node index the robot arrives from.
     * @param v Node index the robot pivots at.
     * @param w Node index the robot leaves to.
     * @return Pointer to the matching `Routes::PivotTurn`, or `nullptr` if any index is out
     *         of range or no record exists for the turn.
     *
     * @note Bounds-checks every index (ADR-028) and scans only the first `N_PIVOT_TURNS`
     *       truth records, never the padded array size. When no real pivots exist
     *       (`N_PIVOT_TURNS == 0`) the loop runs zero times, so the zeroed `{0,0,0,...}`
     *       placeholder element is never matched and a `(0,0,0)` query returns `nullptr`.
     *       Bounded iteration count, no dynamic allocation (ARCHITECTURE iron rules).
     */
    inline const Routes::PivotTurn* findPivotTurn(uint8_t u, uint8_t v, uint8_t w)
    {
        if (u >= N_NODES || v >= N_NODES || w >= N_NODES)
        {
            return nullptr;
        }
        for (uint8_t i = 0; i < Routes::N_PIVOT_TURNS; ++i)
        {
            const Routes::PivotTurn& rec = Routes::PIVOT_TURNS[i];
            if (rec.u == u && rec.v == v && rec.w == w)
            {
                return &Routes::PIVOT_TURNS[i];
            }
        }
        return nullptr;
    }
}    // namespace FieldMap

#endif    // FIELDMAP_H
