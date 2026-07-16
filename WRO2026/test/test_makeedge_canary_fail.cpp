/**
 * @file test_makeedge_canary_fail.cpp
 * @brief TC-canary / AC8 — compile-fail target proving the emitted `makeEdge(...)` constexpr
 *        canary `static_assert` FIRES (and prints its own message) when the factory contract
 *        is broken.
 *
 * @details
 * `header_writer.py` emits, into `routes.h` (and every shadow-routes fixture header), a
 * `constexpr EdgeInfo makeEdge(...)` factory guarded by a compile-time canary:
 *
 *     static_assert(makeEdge(EdgeType::NONE).type == EdgeType::NONE,
 *                   "makeEdge must be constexpr-evaluable");
 *
 * The canary keeps the ~30 KB `const EDGES[N][N]` array in flash `.rodata`: a factory that is
 * not usable in a constant expression forces `EDGES` into RAM (blowing the 32 KB SRAM budget),
 * so the build MUST fail loudly rather than silently regress. This target reproduces the emitted
 * factory + canary verbatim against a DELIBERATELY-BROKEN body so the canary fires, then asserts
 * (via `PASS_REGULAR_EXPRESSION`, NEVER `WILL_FAIL`) that the build output carries the canary's
 * OWN message — proving the guard is wired and its diagnostic reaches the operator.
 *
 * WHY A VALUE BREAK, NOT A DROPPED `constexpr`:
 *   A genuinely non-`constexpr` factory does trip the SAME `static_assert` LINE, but the compiler
 *   then reports "static assertion expression is not an integral constant expression"
 *   (clang) / "non-constant condition for static assertion" (g++) and does NOT print the
 *   user message — so `PASS_REGULAR_EXPRESSION "makeEdge must be constexpr-evaluable"` would not
 *   match, and would red the host suite on the local clang toolchain. To exercise the canary's
 *   MESSAGE path deterministically on every compiler, the factory below stays `constexpr` (so the
 *   condition is a valid, evaluated compile-time constant) but violates the value contract — it
 *   ignores `type` and always stamps `EdgeType::LINE`, so `makeEdge(EdgeType::NONE).type ==
 *   EdgeType::NONE` is a constant `false` and the `static_assert` fires WITH its message on both
 *   clang and g++. The plan's TC-canary explicitly permits "or otherwise trips the emitted canary".
 *
 * The CMake target is `EXCLUDE_FROM_ALL` + `EXCLUDE_FROM_DEFAULT_BUILD` (never part of the normal
 * build) and self-contained (only `<stdint.h>`), so an unrelated header typo cannot mask the
 * intended diagnostic — the exact isolation `test_imu_yaw_sign_fail.cpp` uses.
 */

#include <stdint.h>

namespace Routes
{
enum class EdgeType : uint8_t { NONE = 0, LINE = 1, BLIND = 2 };
enum class SpeedTier : uint8_t { DEFAULT = 0 };
enum class BlindMode : uint8_t { DEFAULT = 0 };
enum class DirectionMode : uint8_t { DEFAULT = 0 };
enum class StopAtLineMode : uint8_t { DEFAULT = 0 };
enum class LineFollowMode : uint8_t { DEFAULT = 0, MIDDLE = 1, LEFT = 2, RIGHT = 3 };

/// @brief Mirror of the emitted 8-field `EdgeInfo` (issue #156, `line_follow_mode_override` last).
struct EdgeInfo
{
    EdgeType       type;
    float          distance_cm;
    float          heading_deg;
    SpeedTier      speed_override;
    BlindMode      blind_mode_override;
    DirectionMode  direction_override;
    StopAtLineMode stop_at_line_override;
    LineFollowMode line_follow_mode_override;
};

/// @brief DELIBERATELY-BROKEN factory: still `constexpr` (so the canary condition evaluates as a
///        compile-time constant, printing the assert message) but it ignores `type` and always
///        stamps `EdgeType::LINE`, so the emitted canary below is a constant `false`.
constexpr EdgeInfo makeEdge(
    EdgeType /*type*/,
    float          distance_cm              = 0.0f,
    float          heading_deg              = 0.0f,
    SpeedTier      speed_override           = SpeedTier::DEFAULT,
    BlindMode      blind_mode_override      = BlindMode::DEFAULT,
    DirectionMode  direction_override       = DirectionMode::DEFAULT,
    StopAtLineMode stop_at_line_override    = StopAtLineMode::DEFAULT,
    LineFollowMode line_follow_mode_override = LineFollowMode::DEFAULT)
{
    return EdgeInfo{ EdgeType::LINE, distance_cm, heading_deg, speed_override,
                     blind_mode_override, direction_override,
                     stop_at_line_override, line_follow_mode_override };
}

// The EMITTED canary, verbatim from header_writer / routes.h. It FIRES here because the broken
// factory above violates the value contract, printing "makeEdge must be constexpr-evaluable".
static_assert(makeEdge(EdgeType::NONE).type == EdgeType::NONE,
              "makeEdge must be constexpr-evaluable");
}    // namespace Routes

int main()
{
    return 0;
}
