// routes_speed_chain.cpp - HAND-AUTHORED, NOT AUTO-GENERATED, NOT VALIDATOR-PASSED.
// See routes_speed_chain.h for the topology + why this table is not single-sourced.

#include "routes_speed_chain.h"

namespace Routes {
    const PipelineConfig PIPELINE_CONFIG = {
        SpeedTier::NORMAL,
        BlindMode::SYNC_TSHAPE,
        TurnMode::COUNTER_ROT,
        DirectionMode::DEFAULT,
    };

    const NodeInfo NODES[N_NODES] = {
        { 0.000000f, 0.000000f, NodeType::BLIND, "Q0" },    // index 0
        { 25.000000f, 0.000000f, NodeType::BLIND, "Q1" },    // index 1
        { 45.000000f, 0.000000f, NodeType::BLIND, "Q2" },    // index 2
        { 65.000000f, 0.000000f, NodeType::BLIND, "Q3" },    // index 3
        { 0.000000f, 40.000000f, NodeType::BLIND, "Sk0" },    // index 4
        { 30.000000f, 40.000000f, NodeType::LINE, "SkL" },    // index 5
        { 0.000000f, 80.000000f, NodeType::LINE, "Ln0" },    // index 6
        { 30.000000f, 80.000000f, NodeType::LINE, "Ln1" },    // index 7
        { 50.000000f, 80.000000f, NodeType::BLIND, "Lb2" },    // index 8
    };

    const uint8_t NEXT_HOP[N_NODES][N_NODES] = {
        {    0,    1,    1,    1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // from Q0 (index 0)
        { 0xFF,    1,    2,    2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // from Q1 (index 1)
        { 0xFF, 0xFF,    2,    3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // from Q2 (index 2)
        { 0xFF, 0xFF, 0xFF,    3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },    // from Q3 (index 3)
        { 0xFF, 0xFF, 0xFF, 0xFF,    4,    5, 0xFF, 0xFF, 0xFF },    // from Sk0 (index 4)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,    5, 0xFF, 0xFF, 0xFF },    // from SkL (index 5)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,    6,    7,    7 },    // from Ln0 (index 6)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,    7,    8 },    // from Ln1 (index 7)
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,    8 },    // from Lb2 (index 8)
    };

    const EdgeInfo EDGES[N_NODES][N_NODES] = {
        { // from Q0 (index 0)
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q0
            { EdgeType::BLIND, 25.000000f, 0.000000f, SpeedTier::NORMAL, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q2
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q3
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Sk0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to SkL
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Lb2
        },
        { // from Q1 (index 1)
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q1
            { EdgeType::BLIND, 20.000000f, 0.000000f, SpeedTier::NORMAL, BlindMode::GYRO_TSHAPE, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q2
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q3
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Sk0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to SkL
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Lb2
        },
        { // from Q2 (index 2)
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q2
            { EdgeType::BLIND, 20.000000f, 0.000000f, SpeedTier::SLOW, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q3
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Sk0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to SkL
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Lb2
        },
        { // from Q3 (index 3)
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q2
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q3
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Sk0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to SkL
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Lb2
        },
        { // from Sk0 (index 4)
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q2
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q3
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Sk0
            { EdgeType::BLIND, 30.000000f, 0.000000f, SpeedTier::SLOW, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::ON }, // to SkL
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Lb2
        },
        { // from SkL (index 5)
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q2
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q3
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Sk0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to SkL
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Lb2
        },
        { // from Ln0 (index 6)
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q2
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q3
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Sk0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to SkL
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln0
            { EdgeType::LINE, 30.000000f, 0.000000f, SpeedTier::FAST, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Lb2
        },
        { // from Ln1 (index 7)
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q2
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q3
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Sk0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to SkL
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln1
            { EdgeType::BLIND, 20.000000f, 0.000000f, SpeedTier::SLOW, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Lb2
        },
        { // from Lb2 (index 8)
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q2
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Q3
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Sk0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to SkL
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln0
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Ln1
            { EdgeType::NONE, 0.000000f, 0.000000f, SpeedTier::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to Lb2
        },
    };

    const PivotTurn PIVOT_TURNS[1] = { { 0, 0, 0, TurnMode::DEFAULT, 0.000000f } };
    const ResetNode RESET_NODES[1] = { { 0, 0, 0.000000f, 0 } };
}  // namespace Routes
