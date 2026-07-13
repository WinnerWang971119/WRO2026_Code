// routes_speed_chain.h - HAND-AUTHORED, NOT AUTO-GENERATED, NOT VALIDATOR-PASSED.
//
// Per-edge mode/speed orthogonalization (issues #97 + #99, ADR-064). This tiny
// shadow field carries the per-edge `speed`/`mode`/`stop_at_line` combinations the
// single-sourced `routes_fixture` (field_fixture.py) does not author, so the
// blind-chain speed-tier gate + seek-speed + pass-through seed-clamp can be driven
// through the REAL PathTracker/Drive/Localization FSM (test_path_tracker_speed_chain.cpp,
// TC10a/b/c + TC14). Three disjoint collinear runs (headings 0):
//   * Q0 ->(NORMAL,SYNC) Q1 ->(NORMAL,GYRO) Q2 ->(SLOW) Q3  -- tier gate (a/b)
//   * Sk0 ->(SLOW, stop_at_line=on) SkL                     -- seek cruises at SLOW (c)
//   * Ln0 ->(LINE,FAST) Ln1 ->(BLIND,SLOW) Lb2              -- FAST-line -> SLOW-blind (TC14)
//
// Like `routes_reset_passthrough_forced.{h,cpp}`, this is NOT single-sourced from
// `field_fixture.py`, NOT governed by ADR-050, and must NEVER be run through the emit
// pipeline. It shares the production `ROUTES_H` include guard so force-including it
// ahead of the sketch `routes.h` shadows the production data for its one CMake target
// (`test_path_tracker_speed_chain`). Nodes are resolved BY NAME in the test.

#ifndef ROUTES_H
#define ROUTES_H

#include <stdint.h>

namespace Routes {
    constexpr uint8_t N_NODES = 9;

    constexpr bool BLIND_PROJECTION_AUTO_SEEK = true;

    enum class NodeType : uint8_t { LINE = 0, BLIND = 1, RESET = 2, PROJECTION = 3 };
    enum class EdgeType : uint8_t { NONE = 0, LINE = 1, BLIND = 2 };

    enum class SpeedTier     : uint8_t { DEFAULT = 0, SLOW = 1, NORMAL = 2, FAST = 3 };
    enum class BlindMode     : uint8_t { DEFAULT = 0, SYNC_TSHAPE = 1, GYRO_TSHAPE = 2, RAW_FLAT = 3, STOP_AT_LINE = 4 };
    enum class TurnMode      : uint8_t { DEFAULT = 0, COUNTER_ROT = 1, PIVOT_LEFT = 2, PIVOT_RIGHT = 3 };
    enum class DirectionMode : uint8_t { DEFAULT = 0, FORCE_FORWARD = 1, FORCE_REVERSE = 2 };

    enum class StopAtLineMode : uint8_t { DEFAULT = 0, ON = 1, OFF = 2 };

    struct PipelineConfig {
        SpeedTier     speed;
        BlindMode     blind;
        TurnMode      turn;
        DirectionMode direction;
    };
    extern const PipelineConfig PIPELINE_CONFIG;

    struct NodeInfo { float x_cm; float y_cm; NodeType type; const char* name; };
    struct EdgeInfo {
        EdgeType       type;
        float          distance_cm;
        float          heading_deg;
        SpeedTier      speed_override;
        BlindMode      blind_mode_override;
        DirectionMode  direction_override;
        StopAtLineMode stop_at_line_override;
    };

    struct PivotTurn { uint8_t u; uint8_t v; uint8_t w; TurnMode wheel; float trim_cm; };
    struct ResetNode { uint8_t node; uint8_t axis; float wall_coord; int16_t facing; };

    extern const NodeInfo NODES[N_NODES];
    extern const uint8_t  NEXT_HOP[N_NODES][N_NODES];
    extern const EdgeInfo EDGES   [N_NODES][N_NODES];

    constexpr uint8_t N_PIVOT_TURNS = 0;
    extern const PivotTurn PIVOT_TURNS[1];

    constexpr uint8_t N_RESET_NODES = 0;
    extern const ResetNode RESET_NODES[1];

    constexpr uint8_t NEXT_HOP_NONE = 0xFF;
    constexpr uint8_t START_NODE = 0;
    constexpr float   START_HEADING_DEG = 0.000000f;
}  // namespace Routes
#endif  // ROUTES_H
