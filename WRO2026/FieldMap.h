/**
 * @file FieldMap.h
 * @brief Header-only namespace of inline accessors over the auto-generated `Routes::` data.
 * @author WRO 2026
 *
 * @details
 * `FieldMap` is the read-only abstraction the rest of Phase 2's runtime
 * (`Localization`, `PathTracker`) uses to consult the precomputed graph. It owns no state,
 * has no `begin()`, no `update()` and no `extern` instance — every getter is an inline
 * lookup into the `Routes::` arrays defined in `routes.cpp`.
 *
 * Why a wrapper at all (instead of consumers using `Routes::` directly):
 *  - One linear scan helper for goal-id -> `Routes::GoalApproach` lookup; the runtime needs
 *    this on every goal arrival and a wrapper avoids duplicating the loop body.
 *  - One node-name -> id helper for the `goTo("name")` overload on `PathTrackerClass`.
 *  - Insulates consumers from any future schema bumps to the auto-generated header — a
 *    rename to `EDGES_X` or a swap from `uint8_t` to `uint16_t` ids only touches this file.
 *
 * Why header-only:
 *  - Every accessor is a one-line `Routes::...[...]` read, so a `.cpp` would just push the
 *    same code into a separate translation unit with link overhead and zero benefit.
 *  - Matches the MatrixMiniR4 style for thin wrappers (most `Modules/` headers are
 *    header-only template / inline classes — see CLAUDE.md "Code conventions").
 *
 * @note `FieldMap` reads the `goal_node` field PR 2 added to `Routes::GoalApproach` so
 *       `getGoalApproach(goalId)` can resolve a goal's blind-moves entry without a parallel
 *       array. The schema bump is documented in
 *       `docs/plans/2026-05-06-phase2-pr2-path-tracker-runtime.md`.
 */

#ifndef FIELDMAP_H
#define FIELDMAP_H

#include <stdint.h>
#include <string.h>

#include "routes.h"

namespace FieldMap
{
    /**
     * @brief Number of nodes in the field graph (re-export of `Routes::N_NODES`).
     */
    constexpr uint8_t N_NODES = Routes::N_NODES;

    /**
     * @brief Number of goal-approach entries (re-export of `Routes::N_GOALS`).
     */
    constexpr uint8_t N_GOALS = Routes::N_GOALS;

    /**
     * @brief Sentinel returned by lookup helpers when a node is not found / not reachable.
     *
     * Mirrors `Routes::NEXT_HOP_NONE` (`0xFF`) so a single sentinel value works for every
     * node-id-shaped query in the runtime.
     */
    constexpr uint8_t NODE_NONE = Routes::NEXT_HOP_NONE;

    /**
     * @brief Read the next-hop entry from the Dijkstra-precomputed table.
     *
     * @param from  Source node id.
     * @param goal  Destination node id.
     * @return The id of the node to move to next on a shortest path from @p from to
     *         @p goal, or `NODE_NONE` if the destination is unreachable.
     * @note Caller must ensure @p from and @p goal are valid ids (`< N_NODES`); a stray
     *       value will index out of the static array. The runtime upstream
     *       (`PathTrackerClass`) checks bounds at the call site.
     */
    inline uint8_t getNextHop(uint8_t from, uint8_t goal)
    {
        return Routes::NEXT_HOP[from][goal];
    }

    /**
     * @brief Read the directed edge metadata between two nodes.
     *
     * @param from  Source node id.
     * @param to    Target node id.
     * @return Reference to the `Routes::EdgeInfo` cell. The cell's `type` field is
     *         `Routes::EdgeType::NONE` if no direct edge exists.
     * @note Bounds are not checked — caller pre-validates ids.
     */
    inline const Routes::EdgeInfo& getEdge(uint8_t from, uint8_t to)
    {
        return Routes::EDGES[from][to];
    }

    /**
     * @brief Convenience accessor for the edge type only.
     *
     * @param from  Source node id.
     * @param to    Target node id.
     * @return The `Routes::EdgeType` between the pair (`NONE` if no direct edge).
     */
    inline Routes::EdgeType getEdgeType(uint8_t from, uint8_t to)
    {
        return Routes::EDGES[from][to].type;
    }

    /**
     * @brief Read a node descriptor by id.
     *
     * @param id  Node id (`< N_NODES`).
     * @return Reference to the static `Routes::NodeInfo` for @p id.
     */
    inline const Routes::NodeInfo& getNode(uint8_t id)
    {
        return Routes::NODES[id];
    }

    /**
     * @brief Convenience accessor for the node category only.
     *
     * @param id  Node id (`< N_NODES`).
     * @return One of `Routes::NodeType::{LINE, PROJECTION, GOAL}`.
     */
    inline Routes::NodeType getNodeType(uint8_t id)
    {
        return Routes::NODES[id].type;
    }

    /**
     * @brief Find the `Routes::GoalApproach` entry whose `goal_node == id`.
     *
     * @param goalId  Node id of a `Routes::NodeType::GOAL` node.
     * @return Pointer to the matching entry, or `nullptr` if @p goalId is not a goal node
     *         (or `N_GOALS == 0`).
     * @note Linear scan over `Routes::GOAL_APPROACH[N_GOALS]`. `N_GOALS` is small (1 on the
     *       PR1 toy fixture; expected ≤ 10 on the real 2026 field), so the cost is bounded.
     *       The caller may cache the pointer for the duration of a single goal traversal.
     */
    inline const Routes::GoalApproach* getGoalApproach(uint8_t goalId)
    {
        for (uint8_t i = 0; i < N_GOALS; ++i)
        {
            if (Routes::GOAL_APPROACH[i].goal_node == goalId)
            {
                return &Routes::GOAL_APPROACH[i];
            }
        }
        return nullptr;
    }

    /**
     * @brief Resolve a node name to its id.
     *
     * @param name  Null-terminated C string. Compared via `strcmp` against the `name` field
     *              of every `Routes::NODES` entry.
     * @return The matching node id, or `NODE_NONE` if no match.
     * @note Linear scan — fine at human-issue rate for the `goTo("name")` PathTracker
     *       overload (toy graph `N_NODES = 7`; real field expected `<= 20`).
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
