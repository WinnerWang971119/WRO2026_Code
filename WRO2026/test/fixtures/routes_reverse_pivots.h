// routes_reverse_pivots.h - HAND-AUTHORED, NOT AUTO-GENERATED, NOT VALIDATOR-PASSED.
//
// Reverse (REAR-traversal) single-wheel pivot fixtures (issue #54, ADR-073, T9). See
// docs/plans/2026-07-19-issue54-reverse-pivots.md, "Testing Strategy: Test data" for the six
// triples this table mirrors from the Python side (tools/path_planner/tests/fixtures/pivot_field_data.py):
//
//   * authored-REAR   -- ArS0 -> ArL0 --LINE,"rear"--> ArL1 --BLIND,"rear"--> ArL2
//                        (pivot (ArL0, ArL1, ArL2), both legs pinned REAR, delta inside the REAR
//                        overshoot budget minus the trim floor -- AC5);
//   * AUTO-REAR approach + coherent-run -- RaU -> RaV -> RaW, no authored override anywhere
//                        (pivot (RaU, RaV, RaW); RaU->RaV's entry delta is 100 deg, strictly > the
//                        90 deg threshold and NOT axis-aligned, so it genuinely resolves REAR under
//                        `BIDIRECTIONAL_AUTO`; the 120 deg turn's naive post-REAR departure delta
//                        (wrap(120-180) = -60) would independently resolve FRONT, so force-SAME
//                        holding the departure REAR is the coherent-run proof -- AC1, AC2, TC3);
//   * binding-contradiction -- BcU --BLIND,"front"--> BcV --BLIND,"rear"--> BcW
//                        (pivot (BcU, BcV, BcW): approach pinned FRONT, departure pinned REAR --
//                        a statically-visible contradiction; the firmware backstop must fault
//                        `PIVOT_DIRECTION_MISMATCH` -- AC3, TC4);
//   * reset-node (reject)      -- RnU -> RnV [RESET, y-wall] -> RnW, no override (reverse-reachable);
//   * reset-node (FRONT-pinned, accept) -- RnfU --BLIND,"front"--> RnfV [RESET] --BLIND,"front"--> RnfW;
//   * chained-reverse pair     -- CrU --BLIND,"rear"--> CrV --BLIND,"rear"--> CrW --BLIND,"rear"--> CrX
//                        (pivots (CrU, CrV, CrW) and (CrV, CrW, CrX) sharing edge CrV->CrW, both
//                        legs of both pivots pinned REAR end to end).
//
// Every edge is authored ONE-DIRECTION ONLY (mirrors routes_speed_chain.h): `NEXT_HOP` and `EDGES`
// carry the forward sense of each disjoint chain; the reverse/backward and every cross-component
// pair is `0xFF` / `EdgeType::NONE` (unused by any test). Nodes are resolved BY NAME
// (`FieldMap::findNodeByName`, issue #91) -- seat the chassis at any component's first node via the
// harness `seatAt(name, ...)` helper before driving a `goTo` through it; do not assume indices.
//
// NOT single-sourced from `field_fixture.py` (T5 owns that file's biased parity edge) and NOT run
// through `regenerate_routes_fixture.py` / `tools/path_planner/build_routes.py` -- the
// binding-contradiction and reverse-reachable reset-node components are BUILT to be rejected by the
// offline validator, so neither script could ever emit this table. Shares the production `ROUTES_H`
// include guard so force-including this header ahead of the sketch `routes.h` shadows the production
// data for whichever CMake target(s) link it (T10 wires the CMakeLists appends).

#ifndef ROUTES_H
#define ROUTES_H

#include <stdint.h>

namespace Routes {
    constexpr uint8_t N_NODES = 20;

    constexpr bool BLIND_PROJECTION_AUTO_SEEK = true;

    enum class NodeType : uint8_t { LINE = 0, BLIND = 1, RESET = 2, PROJECTION = 3 };
    enum class EdgeType : uint8_t { NONE = 0, LINE = 1, BLIND = 2 };

    enum class SpeedTier      : uint8_t { DEFAULT = 0, SLOW = 1, NORMAL = 2, FAST = 3 };
    enum class BlindMode      : uint8_t { DEFAULT = 0, SYNC_TSHAPE = 1, GYRO_TSHAPE = 2, RAW_FLAT = 3, STOP_AT_LINE = 4 };
    enum class TurnMode       : uint8_t { DEFAULT = 0, COUNTER_ROT = 1, PIVOT_LEFT = 2, PIVOT_RIGHT = 3 };
    enum class DirectionMode  : uint8_t { DEFAULT = 0, FORCE_FORWARD = 1, FORCE_REVERSE = 2 };
    enum class LineFollowMode : uint8_t { DEFAULT = 0, MIDDLE = 1, LEFT = 2, RIGHT = 3 };

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
        LineFollowMode line_follow_mode_override;
    };

    // makeEdge(...): shadow-fixture mirror of the production factory
    // (issue #156) so every EDGES cell below is built the same way and
    // the shadow ABI stays identical. Trailing defaults fill the
    // unspecified overrides; a NONE cell is makeEdge(EdgeType::NONE).
    // The static_assert canary fails the build if makeEdge ever stops
    // being constexpr.
    constexpr EdgeInfo makeEdge(
        EdgeType       type,
        float          distance_cm              = 0.0f,
        float          heading_deg              = 0.0f,
        SpeedTier      speed_override           = SpeedTier::DEFAULT,
        BlindMode      blind_mode_override      = BlindMode::DEFAULT,
        DirectionMode  direction_override       = DirectionMode::DEFAULT,
        StopAtLineMode stop_at_line_override    = StopAtLineMode::DEFAULT,
        LineFollowMode line_follow_mode_override = LineFollowMode::DEFAULT)
    {
        return EdgeInfo{ type, distance_cm, heading_deg, speed_override,
                         blind_mode_override, direction_override,
                         stop_at_line_override, line_follow_mode_override };
    }
    static_assert(makeEdge(EdgeType::NONE).type == EdgeType::NONE,
                  "makeEdge must be constexpr-evaluable");

    // Single-wheel pivot turn (ADR-036/ADR-073). The FORWARD-sense wheel + the
    // direction-invariant trim; `ReversePivotPolicy::resolvePivotWheel` flips
    // the wheel at runtime for a REAR-resolved traversal -- this table never
    // changes shape for #54 (only PathTracker's runtime `PivotRef` gains `dir`).
    struct PivotTurn {
        uint8_t  u;
        uint8_t  v;
        uint8_t  w;
        TurnMode wheel;
        float    trim_cm;
    };

    // Wall-zero reset node (PR2).
    struct ResetNode {
        uint8_t  node;
        uint8_t  axis;
        float    wall_coord;
        int16_t  facing;
    };

    extern const NodeInfo NODES[N_NODES];
    extern const uint8_t  NEXT_HOP[N_NODES][N_NODES];
    extern const EdgeInfo EDGES   [N_NODES][N_NODES];

    // Seven pivots: authored-REAR (1), AUTO-REAR-approach/coherent-run (1),
    // binding-contradiction (1), reset-node reject + FRONT-pinned accept (2),
    // chained-reverse pair (2).
    constexpr uint8_t N_PIVOT_TURNS = 7;
    extern const PivotTurn PIVOT_TURNS[7];

    // Two reset (wall-zero) nodes: the reject variant's RnV and the
    // FRONT-pinned accept variant's RnfV.
    constexpr uint8_t N_RESET_NODES = 2;
    extern const ResetNode RESET_NODES[2];

    constexpr uint8_t NEXT_HOP_NONE = 0xFF;
    constexpr uint8_t START_NODE = 0;
    constexpr float   START_HEADING_DEG = 0.000000f;
}  // namespace Routes
#endif  // ROUTES_H
