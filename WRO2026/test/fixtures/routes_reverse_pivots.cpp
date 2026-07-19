// routes_reverse_pivots.cpp - HAND-AUTHORED, NOT AUTO-GENERATED, NOT VALIDATOR-PASSED.
// See routes_reverse_pivots.h for the six reverse-pivot components this table carries and why it
// is deliberately not single-sourced from field_fixture.py / the emit pipeline.
//
// Six disjoint chains (no edge between them), each hand-picked to mirror the geometry of the
// matching tools/path_planner/tests/fixtures/pivot_field_data.py factory:
//
//   indices  0- 3  ArS0 -> ArL0 -> ArL1 -> ArL2   (authored-REAR,        make_authored_rear_pivot_field)
//   indices  4- 6  RaU  -> RaV  -> RaW            (AUTO-REAR approach,   make_auto_rear_approach_pivot_field)
//   indices  7- 9  BcU  -> BcV  -> BcW            (binding-contradiction, make_binding_contradiction_pivot_field)
//   indices 10-12  RnU  -> RnV  -> RnW            (reset-node reject,    make_reset_node_pivot_field)
//   indices 13-15  RnfU -> RnfV -> RnfW           (reset-node FRONT-pinned accept, make_reset_node_pivot_field_front_pinned)
//   indices 16-19  CrU  -> CrV  -> CrW  -> CrX    (chained-reverse pair, make_chained_reverse_pivot_field)

#include "routes_reverse_pivots.h"

namespace Routes {
    const PipelineConfig PIPELINE_CONFIG = {
        SpeedTier::NORMAL,
        BlindMode::SYNC_TSHAPE,
        TurnMode::COUNTER_ROT,
        DirectionMode::DEFAULT,
    };

    const NodeInfo NODES[N_NODES] = {
        { -40.000000f,   0.000000f, NodeType::BLIND, "ArS0" },    // index  0
        {   0.000000f,   0.000000f, NodeType::LINE,  "ArL0" },    // index  1
        {  60.000000f,   0.000000f, NodeType::LINE,  "ArL1" },    // index  2
        {  89.018331f,  27.530646f, NodeType::BLIND, "ArL2" },    // index  3
        {   0.000000f,   0.000000f, NodeType::BLIND, "RaU"  },    // index  4
        {  -6.945927f,  39.392310f, NodeType::BLIND, "RaV"  },    // index  5
        { -37.587705f,  13.680806f, NodeType::BLIND, "RaW"  },    // index  6
        {   0.000000f,   0.000000f, NodeType::BLIND, "BcU"  },    // index  7
        {  40.000000f,   0.000000f, NodeType::BLIND, "BcV"  },    // index  8
        {  40.000000f,  40.000000f, NodeType::BLIND, "BcW"  },    // index  9
        {   0.000000f,   0.000000f, NodeType::BLIND, "RnU"  },    // index 10
        {   0.000000f,  60.000000f, NodeType::RESET, "RnV"  },    // index 11
        {  40.000000f,  60.000000f, NodeType::BLIND, "RnW"  },    // index 12
        {   0.000000f,   0.000000f, NodeType::BLIND, "RnfU" },    // index 13
        {   0.000000f,  60.000000f, NodeType::RESET, "RnfV" },    // index 14
        {  40.000000f,  60.000000f, NodeType::BLIND, "RnfW" },    // index 15
        {   0.000000f,   0.000000f, NodeType::BLIND, "CrU"  },    // index 16
        {   0.000000f,  60.000000f, NodeType::BLIND, "CrV"  },    // index 17
        {  60.000000f,  60.000000f, NodeType::BLIND, "CrW"  },    // index 18
        {  60.000000f,   0.000000f, NodeType::BLIND, "CrX"  },    // index 19
    };

    const uint8_t NEXT_HOP[N_NODES][N_NODES] = {
        // [from][to] = next-hop node index; 0xFF = unreachable/unused. Each row only carries
        // forward hops within its own disjoint chain (mirrors routes_speed_chain.h); every
        // cross-component and backward pair is unused by any test.
        {    0,    1,    1,    1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // ArS0 ( 0)
        { 0xFF,    1,    2,    2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // ArL0 ( 1)
        { 0xFF, 0xFF,    2,    3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // ArL1 ( 2)
        { 0xFF, 0xFF, 0xFF,    3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // ArL2 ( 3)
        { 0xFF, 0xFF, 0xFF, 0xFF,    4,    5,    5, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // RaU  ( 4)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,    5,    6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // RaV  ( 5)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,    6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // RaW  ( 6)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,    7,    8,    8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // BcU  ( 7)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,    8,    9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // BcV  ( 8)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,    9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // BcW  ( 9)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   10,   11,   11, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // RnU  (10)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   11,   12, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // RnV  (11)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   12, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // RnW  (12)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   13,   14,   14, 0xFF, 0xFF, 0xFF, 0xFF },    // RnfU (13)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   14,   15, 0xFF, 0xFF, 0xFF, 0xFF },    // RnfV (14)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   15, 0xFF, 0xFF, 0xFF, 0xFF },    // RnfW (15)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   16,   17,   17,   17 },    // CrU  (16)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   17,   18,   18 },    // CrV  (17)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   18,   19 },    // CrW  (18)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   19 },    // CrX  (19)
    };

    const EdgeInfo EDGES[N_NODES][N_NODES] = {
        { // ArS0 (0)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 40.000000f, 0.000000f),    // -> ArL0
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // ArL0 (1)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::LINE, 60.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT,
                      DirectionMode::FORCE_REVERSE),    // -> ArL1: LINE, sensor-terminated incoming leg of the authored-REAR pivot, pinned REAR.
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // ArL1 (2)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 40.000000f, 43.493015f, SpeedTier::DEFAULT, BlindMode::DEFAULT,
                      DirectionMode::FORCE_REVERSE),    // -> ArL2: BLIND, dead-reckoned outgoing leg, pinned REAR (both legs REAR, AC5).
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // ArL2 (3)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // RaU (4)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 40.000000f, 100.000000f),    // -> RaV: BLIND, no override -- entry delta (100 deg from START_HEADING_DEG=0) resolves
            // REAR under BIDIRECTIONAL_AUTO (AC1's AUTO-resolved, not authored, REAR approach).
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // RaV (5)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 40.000000f, 220.000000f),    // -> RaW: BLIND, no override -- the 120 deg pivot's departure; force-SAME must hold this
            // REAR (matching RaU->RaV's approach) even though its own naive AUTO delta
            // (wrap(120-180) = -60) would independently resolve FRONT (AC2 coherent-run proof).
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // RaW (6)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // BcU (7)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 40.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT,
                      DirectionMode::FORCE_FORWARD),    // -> BcV: BLIND, pinned FRONT (binding approach direction).
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // BcV (8)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 40.000000f, 90.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT,
                      DirectionMode::FORCE_REVERSE),    // -> BcW: BLIND, pinned REAR (binding departure direction) -- CONTRADICTS the FRONT-pinned
            // approach BcU->BcV. Drives PIVOT_DIRECTION_MISMATCH at the departure dispatch (AC3, TC4).
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // BcW (9)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // RnU (10)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 60.000000f, 90.000000f),    // -> RnV: BLIND, no override -- reverse-reachable approach into the RESET node RnV
            // (2-board default chassis, not FRONT-pinned). Expected REJECT offline (AC7b).
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // RnV (11)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 40.000000f, 0.000000f),    // -> RnW: BLIND, no override.
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // RnW (12)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // RnfU (13)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 60.000000f, 90.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT,
                      DirectionMode::FORCE_FORWARD),    // -> RnfV: BLIND, pinned FRONT -- the SAME geometry as RnU->RnV, but statically
            // FRONT-pinned, so the reset-node pivot at RnfV is the ACCEPTED counterpart.
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // RnfV (14)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 40.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT,
                      DirectionMode::FORCE_FORWARD),    // -> RnfW: BLIND, pinned FRONT.
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // RnfW (15)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // CrU (16)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 60.000000f, 90.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT,
                      DirectionMode::FORCE_REVERSE),    // -> CrV: BLIND, pinned REAR -- pivot 1's (CrU, CrV, CrW) REAR approach.
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
        { // CrV (17)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 60.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT,
                      DirectionMode::FORCE_REVERSE),    // -> CrW: BLIND, pinned REAR -- pivot 1's forced-REAR departure AND pivot 2's
            // (CrV, CrW, CrX) REAR approach (the shared edge, chained end to end).
            makeEdge(EdgeType::NONE),
        },
        { // CrW (18)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::BLIND, 60.000000f, 270.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT,
                      DirectionMode::FORCE_REVERSE),    // -> CrX: BLIND, pinned REAR -- pivot 2's forced-REAR departure.
        },
        { // CrX (19)
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
            makeEdge(EdgeType::NONE),
        },
    };

    const PivotTurn PIVOT_TURNS[7] = {
        // sorted by (u, v, w) node index
        // 3.59 = (LINE_SENSOR_AXLE_OFFSET_REAR_CM - PIVOT_TRIM_MIN_DRIVE_CM)/2 at 7.68/0.5;
        // regenerate the trim + ArL2 coords/heading on a REAR-offset retune.
        { 1, 2, 3, TurnMode::PIVOT_LEFT, 3.590000f },       // authored-REAR (ArL0, ArL1, ArL2)
        { 4, 5, 6, TurnMode::PIVOT_LEFT, 15.588457f },      // AUTO-REAR approach (RaU, RaV, RaW)
        { 7, 8, 9, TurnMode::PIVOT_LEFT, 9.000000f },       // binding-contradiction (BcU, BcV, BcW)
        { 10, 11, 12, TurnMode::PIVOT_RIGHT, 9.000000f },   // reset-node reject (RnU, RnV, RnW)
        { 13, 14, 15, TurnMode::PIVOT_RIGHT, 9.000000f },   // reset-node FRONT-pinned (RnfU, RnfV, RnfW)
        { 16, 17, 18, TurnMode::PIVOT_RIGHT, 9.000000f },   // chained pivot 1 (CrU, CrV, CrW)
        { 17, 18, 19, TurnMode::PIVOT_RIGHT, 9.000000f },   // chained pivot 2 (CrV, CrW, CrX)
    };

    const ResetNode RESET_NODES[2] = {
        // sorted by node index; axis 0 = x, 1 = y
        { 11, 1, 80.000000f, 270 },    // RnV  -- reverse-reachable, expected reject (AC7b)
        { 14, 1, 80.000000f, 270 },    // RnfV -- FRONT-pinned, expected accept
    };
}  // namespace Routes
