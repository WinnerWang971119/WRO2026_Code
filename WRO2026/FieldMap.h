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
     * @brief Read the next-hop entry from the Dijkstra-precomputed table.
     */
    inline uint8_t getNextHop(uint8_t from, uint8_t goal)
    {
        return Routes::NEXT_HOP[from][goal];
    }

    /**
     * @brief Read the directed edge metadata between two nodes.
     */
    inline const Routes::EdgeInfo& getEdge(uint8_t from, uint8_t to)
    {
        return Routes::EDGES[from][to];
    }

    /**
     * @brief Convenience accessor for the edge type only.
     */
    inline Routes::EdgeType getEdgeType(uint8_t from, uint8_t to)
    {
        return Routes::EDGES[from][to].type;
    }

    /**
     * @brief Read a node descriptor by id.
     */
    inline const Routes::NodeInfo& getNode(uint8_t id)
    {
        return Routes::NODES[id];
    }

    /**
     * @brief Convenience accessor for the node category only.
     */
    inline Routes::NodeType getNodeType(uint8_t id)
    {
        return Routes::NODES[id].type;
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
}    // namespace FieldMap

#endif    // FIELDMAP_H
