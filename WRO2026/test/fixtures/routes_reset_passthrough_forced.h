// routes_reset_passthrough_forced.h - HAND-WRITTEN, NOT AUTO-GENERATED, NOT VALIDATOR-PASSED.
//
// TC11 / AC9 (ADR-052, docs/plans/2026-07-03-line-arrival-heading-reset.md) exists to prove
// `PathTrackerClass::_maybeHeadingReset`'s BLIND/RESET no-op branch is a true no-op WHEN
// REACHED. Per ADR-048, that branch is UNREACHABLE on any validator-passed field: no LINE edge
// may be incident to a RESET node, and `_passThrough` is latched only on a LINE-edge arrival —
// so a real `field_data.json` -> validator -> graph_builder -> build_routes.py pipeline could
// never emit a table that reaches it. This tiny 3-node table is therefore DELIBERATELY
// hand-authored to violate ADR-048 on purpose (`PassA -> PassResetB` is a LINE edge straight
// into a RESET-typed node) — exactly the "hand-edited routes table that bypasses the validator"
// case the plan's Decisions section names as one of the branch's two real-world triggers. TC11
// drives the REAL PathTracker/Drive/Localization FSM over this table (no private-member access,
// no PathTracker.h/.cpp edit) so the real intersection-gated LINE pass-through gate
// (`_passThroughHeadingOk`) legitimately latches `_passThrough == true` while crossing
// `PassResetB` — a state ADR-048 guarantees can never happen on a real field.
//
// NOT single-sourced from `field_fixture.py` and NOT governed by ADR-050 (which covers
// `routes_fixture.{h,cpp}` only). NEVER run this through `regenerate_routes_fixture.py` or
// `tools/path_planner/build_routes.py` — both would (correctly) reject this topology. Used by
// exactly ONE CMake target: `test_path_tracker_heading_reset_forced_reset`
// (`WRO2026/test/CMakeLists.txt`).
//
// Shares the production `ROUTES_H` include guard so force-including this header ahead of the
// sketch `routes.h` shadows the production data for that one target only — the same shadow
// trick `routes_fixture.h` / `routes_toy.h` use.

#ifndef ROUTES_H
#define ROUTES_H

#include <stdint.h>

namespace Routes {
    constexpr uint8_t N_NODES = 3;

    // Field-wide: auto stop-at-line seek on a BLIND hop that precedes a PROJECTION. Unused by
    // this table (no PROJECTION node), kept for ABI parity with the production header.
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

    struct PivotTurn {
        uint8_t  u;
        uint8_t  v;
        uint8_t  w;
        TurnMode wheel;
        float    trim_cm;
    };

    struct ResetNode {
        uint8_t  node;
        uint8_t  axis;
        float    wall_coord;
        int16_t  facing;
    };

    extern const NodeInfo NODES[N_NODES];
    extern const uint8_t  NEXT_HOP[N_NODES][N_NODES];
    extern const EdgeInfo EDGES   [N_NODES][N_NODES];

    // No pivots on this table.
    constexpr uint8_t N_PIVOT_TURNS = 0;
    extern const PivotTurn PIVOT_TURNS[1];

    // `PassResetB` is DELIBERATELY absent from `RESET_NODES` (`N_RESET_NODES == 0`):
    // `FieldMap::getResetInfo(PassResetB)` then returns `nullptr`, so
    // `PathTrackerClass::_dispatchWallStop` takes its own defensive "tagged RESET but has no
    // reset record" branch — a single tick, no `Drive.wallStop` dispatch — which is the
    // simplest way past the wall-zero interlude (`_maybeEnterReset`) into a SECOND
    // `_onPlanNextHop()` call with `_resetNode == cur`, the precondition for
    // `_maybeHeadingReset` to actually run and take the BLIND/RESET branch.
    constexpr uint8_t N_RESET_NODES = 0;
    extern const ResetNode RESET_NODES[1];

    constexpr uint8_t NEXT_HOP_NONE = 0xFF;
    constexpr uint8_t START_NODE = 0;
    constexpr float   START_HEADING_DEG = 0.000000f;
}  // namespace Routes
#endif  // ROUTES_H
