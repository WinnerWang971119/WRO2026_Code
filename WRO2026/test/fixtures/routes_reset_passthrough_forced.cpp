// routes_reset_passthrough_forced.cpp - HAND-WRITTEN, NOT AUTO-GENERATED, NOT VALIDATOR-PASSED.
// See the header docblock (routes_reset_passthrough_forced.h) for why this table deliberately
// violates ADR-048 and is intentionally NOT single-sourced from `field_fixture.py`.
//
// Topology: `PassA --LINE(heading 0)--> PassResetB --LINE(heading 0)--> PassC`, fully collinear
// so a `goTo(PassC)` from `PassA` lets the real intersection-gated LINE pass-through gate
// (`PathTrackerClass::_passThroughHeadingOk`) legitimately latch `_passThrough == true` while
// sailing through `PassResetB` — a RESET-typed node, which is exactly the state ADR-048
// guarantees can never happen on a validator-passed field.

#include "routes_reset_passthrough_forced.h"

namespace Routes {
    const PipelineConfig PIPELINE_CONFIG = {
        LineMode::NORMAL,
        BlindMode::SYNC_TSHAPE,
        TurnMode::COUNTER_ROT,
        DirectionMode::DEFAULT,
    };

    const NodeInfo NODES[N_NODES] = {
        {  0.000000f, 0.000000f, NodeType::LINE,  "PassA"      },    // index 0
        { 40.000000f, 0.000000f, NodeType::RESET, "PassResetB" },    // index 1
        { 80.000000f, 0.000000f, NodeType::LINE,  "PassC"      },    // index 2
    };

    const uint8_t NEXT_HOP[N_NODES][N_NODES] = {
        // [from][to] = next-hop node index; 0xFF = unreachable
        {    0,    1,    1 },    // from PassA (index 0)
        {    0,    1,    2 },    // from PassResetB (index 1)
        { 0xFF, 0xFF,    2 },    // from PassC (index 2) — unused, TC11 never starts here
    };

    const EdgeInfo EDGES[N_NODES][N_NODES] = {
        { // from PassA (index 0)
            { EdgeType::NONE, 0.000000f, 0.000000f, LineMode::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to PassA
            { EdgeType::LINE, 40.000000f, 0.000000f, LineMode::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to PassResetB
            { EdgeType::NONE, 0.000000f, 0.000000f, LineMode::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to PassC
        },
        { // from PassResetB (index 1)
            { EdgeType::NONE, 0.000000f, 0.000000f, LineMode::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to PassA
            { EdgeType::NONE, 0.000000f, 0.000000f, LineMode::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to PassResetB
            { EdgeType::LINE, 40.000000f, 0.000000f, LineMode::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to PassC
        },
        { // from PassC (index 2)
            { EdgeType::NONE, 0.000000f, 0.000000f, LineMode::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to PassA
            { EdgeType::NONE, 0.000000f, 0.000000f, LineMode::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to PassResetB
            { EdgeType::NONE, 0.000000f, 0.000000f, LineMode::DEFAULT, BlindMode::DEFAULT, DirectionMode::DEFAULT, StopAtLineMode::DEFAULT }, // to PassC
        },
    };

    const PivotTurn PIVOT_TURNS[1] = {
        // placeholder: N_PIVOT_TURNS == 0, never read at runtime.
        { 0, 0, 0, TurnMode::DEFAULT, 0.000000f },
    };

    const ResetNode RESET_NODES[1] = {
        // placeholder: N_RESET_NODES == 0, never read at runtime — PassResetB is DELIBERATELY
        // left without a reset record (see the header docblock).
        { 0, 0, 0.000000f, 0 },
    };
}  // namespace Routes
