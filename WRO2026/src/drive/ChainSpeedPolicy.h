#ifndef CHAIN_SPEED_POLICY_H
#define CHAIN_SPEED_POLICY_H

#include <stdint.h>

/**
 * @file ChainSpeedPolicy.h
 * @brief Pure two-pass velocity planner for a collinear "chain" of hops driven as ONE
 *        continuous motion — a single ramp-up, cruise through the intermediate nodes, and a
 *        single ramp-down/stop at the run end, instead of stopping and re-accelerating at each
 *        node (ADR-059, the collinear-run continuous-motion plan).
 *
 * The planner is the forward+backward junction scheme from GRBL / Marlin: a FORWARD pass bounds
 * each node's speed by what the acceleration allows from the run entry, and a BACKWARD pass
 * sheds every node's speed low enough to reach the motor deadband floor (`pwmMin`) by the run
 * end. Chaining the backward pass through the ALREADY-reduced downstream node (not a per-hop
 * `pwmMin + accel*distToEnd` closed form) is what keeps every commanded decel <= `accel` and
 * pulls the shed into the PRIOR hop when the final hop is short — avoiding the "half-clamp slam"
 * a decel-only formula produces (ADR-059 Decisions).
 *
 * The final hop is the ORDINARY `cruise->floor->brake` stop (ADR-058): its output end speed is
 * `0` with `brake=true`. The backward pass guarantees the entry to the final hop is low enough
 * that the firmware's `(cruise - pwmMin)/accel` shed-to-floor fits within it.
 *
 * The planner works entirely in the MAGNITUDE domain (absolute PWM); the sign of every output
 * comes from `cruisePwm` (a reverse-cruise run has a negative `cruisePwm` and yields negative
 * `startPwm`/`endPwm`). This keeps the two-pass math sign-free and matches the way the firmware
 * carries direction in the sign of the commanded PWM.
 *
 * This header is deliberately self-contained (only `<stdint.h>`; zero Arduino / MatrixMiniR4 /
 * Config.h surface) so a host golden-dump TU and the pytest float32 mirror
 * (`tools/path_planner/ctps/chain_speed.py`) can include / reproduce it standalone — the same
 * `TShapeProfilePolicy.h` pattern. The `Config` knobs (`PWM_MIN_MAGNITUDE`, `PWM_MAX`,
 * `TSHAPE_ACCEL_PWM_PER_CM`, `TSHAPE_MIN_MOVE_CM`) are passed in as a `ChainSpeedConfig` by the
 * caller.
 *
 * BEHAVIOR-IDENTITY CONTRACT — do NOT "improve" the arithmetic. The golden vectors and the
 * Python float32 mirror (`chain_speed.py`, mirroring `profile.py`'s `_f32` / `_trunc_i16`) are
 * pinned to the exact operations below:
 *   - Every node speed is carried as a 32-bit `float` through BOTH passes (the `accel*hopDist`
 *     products, the additions, and the `min` compares are all float32).
 *   - The clamp bounds (`pwmMin`, `|cruisePwm|`) and `|entryPwm|` are integer magnitudes
 *     promoted to `float` before they enter the float32 domain.
 *   - Each node speed is truncated to `int16_t` toward zero ONLY at the output boundary
 *     (`out[k].startPwm` / `out[k].endPwm`), then the run sign is multiplied in. Truncation
 *     toward zero (not round) mirrors `profile.py`'s `_trunc_i16`.
 *   - `accel <= 0` is clamped to a tiny positive slope (`ACCEL_EPS`) so no pass ever divides by
 *     zero or fails to shed; a zero/negative `hopDistCm[k]` floors to `0`.
 *
 * @note This header plans SPEEDS only — it never touches encoder distances, the boundary-carry
 *       subtraction, or the FSM dispatch (those stay in `PathTracker`). It is a pure function of
 *       its arguments.
 * @note `deadbandSnap` is NOT applied here — the returned setpoints are pre-snap; `Drive.cpp`
 *       snaps each dispatched PWM downstream, exactly as for `TShapeProfilePolicy`.
 */

/**
 * @brief The Config knobs the chain planner needs, passed in by the caller.
 *
 * Kept as a POD so the host golden TU can build it from the real firmware `Config::*` constants
 * and the pytest mirror can build it from the parsed header values, without either depending on
 * the other's surface.
 */
struct ChainSpeedConfig
{
    int16_t pwmMin;         ///< Motor deadband floor magnitude — `Config::PWM_MIN_MAGNITUDE`.
    int16_t pwmMax;         ///< Saturation cap magnitude — `Config::PWM_MAX` (defensive clamp).
    float   accelPwmPerCm;  ///< Ramp slope in PWM per cm — `Config::TSHAPE_ACCEL_PWM_PER_CM`.
    float   minMoveCm;      ///< Profiled-move floor (cm) — `Config::TSHAPE_MIN_MOVE_CM`
                            ///< (informational; the caller gates chain eligibility on it).
};

/**
 * @brief One resolved hop of a chain: the signed start/end PWM setpoints + the brake flag.
 *
 * `brake` is `true` ONLY on the final hop (`out[hopCount-1]`), whose `endPwm` is `0` — the
 * ordinary `cruise->floor->brake` stop. Every mid-hop has `brake=false` and a non-zero `endPwm`
 * so the chassis passes through the node without braking to rest.
 */
struct ChainHopSpeed
{
    int16_t startPwm;  ///< Signed PWM commanded at the hop start (sign = run direction).
    int16_t endPwm;    ///< Signed PWM commanded at the hop end; `0` on the final hop.
    bool    brake;     ///< `true` only on the final hop.
};

/**
 * @brief Smallest positive accel slope used when the caller passes `accel <= 0`.
 *
 * The firmware bails on `accel <= 0` before this policy is reached, but a standalone host caller
 * (or a mis-configured mirror) must not divide the accel-limited passes into a zero slope — so
 * the policy clamps up to this tiny positive value, matching `profile.py`'s `1e-6` defensive
 * floor. Exact power-of-two-free but far below any real retune, so it never perturbs a valid
 * config.
 */
static constexpr float CHAIN_ACCEL_EPS = 1e-6f;

/**
 * @brief Two-pass junction planner over a collinear run of `hopCount` hops (`hopCount+1` nodes).
 *
 * @details
 * Node speeds `v[0..hopCount]` are resolved in the magnitude domain, then mapped to per-hop
 * signed `(startPwm, endPwm)`:
 *
 *   1. **Seed.** `v[0] = clamp(|entryPwm|, pwmMin, |cruise|)`; every interior/target node
 *      (`v[1..hopCount]`) seeds to `|cruise|`.
 *   2. **Forward pass (accel limit).** For `k = 0..hopCount-1`:
 *      `v[k+1] = min(v[k+1], v[k] + accel*hopDist[k])`. No commanded exit exceeds what the
 *      acceleration allows from the entry: `vExit[k] <= vEntry[k] + accel*hopDist[k]`.
 *   3. **Backward pass (decel to floor).** The run must reach the FLOOR (`pwmMin`) by the run
 *      end — the final hop is the ordinary `cruise->floor->brake` stop, so the terminal node
 *      target is `pwmMin`, not `0`. Seed `v[hopCount] = min(v[hopCount], pwmMin)`, then for
 *      `k = hopCount-1 .. 0`: `v[k] = min(v[k], v[k+1] + accel*hopDist[k])`. Chaining through
 *      the ALREADY-reduced downstream node `v[k+1]` (not a per-hop `pwmMin + accel*distToEnd`
 *      closed form) guarantees every hop's decel `v[k] - v[k+1] <= accel*hopDist[k]` (AC4, no
 *      half-clamp slam) and pulls the shed into the PRIOR hop for a short final hop. On the
 *      monotone case this equals `pwmMin + accel*distToEnd(k)`; on a run whose forward pass left
 *      an interior node below cruise it stays accel-bounded where the closed form would not.
 *   4. **Clamp.** Every node speed stays within `[pwmMin, |cruise|]` by construction (the seeds
 *      are in range and both passes only ever LOWER a node, never below the `pwmMin` floor the
 *      backward pass targets).
 *   5. **Map to hops.** For `k = 0..hopCount-2`: `out[k] = { sign*v[k], sign*v[k+1], false }`.
 *      Final hop: `out[hopCount-1] = { sign*v[hopCount-1], 0, true }` — end speed `0`, brake
 *      `true`. `v[hopCount]` (the terminal node) is used only as the backward-pass seed that
 *      forces `v[hopCount-1]` low enough for the firmware's `cruise->floor` shed to fit inside
 *      the final hop; it is NOT emitted directly.
 *
 * @note **Degenerate inputs** (Error Handling): `hopCount < 2` is NOT a chain — the function
 *       returns immediately leaving `out` untouched, and the caller takes the ordinary per-hop
 *       path (TC1's single-hop case). A zero/negative `hopDistCm[k]` floors to `0` (that hop
 *       contributes no accel/decel room). `accel <= 0` clamps to `CHAIN_ACCEL_EPS`. Nothing here
 *       divides, so there is no divide-by-zero to guard beyond the accel floor.
 *
 * @param hopDistCm  Array of `hopCount` per-hop distances (cm). Negative/zero entries floor to 0.
 * @param hopCount   Number of hops in the run (>= 2 for a chain; < 2 leaves `out` untouched).
 * @param cruisePwm  Signed run cruise PWM (sign carries the run direction into every output).
 * @param entryPwm   Signed PWM the chassis is physically at when the run begins — the captured
 *                   cruise after a pass-through, else +/-`TSHAPE_START_SPEED` on a cold start.
 * @param cfg        The `pwmMin` / `pwmMax` / `accel` knobs (see @ref ChainSpeedConfig).
 * @param out        Output array of `hopCount` @ref ChainHopSpeed; filled for `hopCount >= 2`.
 */
inline void resolveChainSpeeds(const float* hopDistCm, uint8_t hopCount, int16_t cruisePwm,
                               int16_t entryPwm, const ChainSpeedConfig& cfg, ChainHopSpeed* out)
{
    // hopCount < 2 is not a chain — the caller uses the ordinary per-hop path. Leave `out`
    // untouched (the single-hop case, TC1).
    if (hopCount < 2)
    {
        return;
    }

    // Run sign comes from the cruise PWM; the two-pass math is done in the magnitude domain.
    // A cruise of exactly 0 has no direction, so treat it as forward (+1) — a degenerate input
    // the caller never produces (a chain always cruises), guarded only so the sign is defined.
    const int16_t sign = (cruisePwm < 0) ? (int16_t) -1 : (int16_t) 1;

    // Integer magnitudes promoted to float once, up front — the clamp bounds and the entry seed
    // enter the float32 domain here (see the file-level BEHAVIOR-IDENTITY note).
    const int32_t cruiseMagI = (cruisePwm < 0) ? -(int32_t) cruisePwm : (int32_t) cruisePwm;
    const int32_t entryMagI  = (entryPwm < 0) ? -(int32_t) entryPwm : (int32_t) entryPwm;
    const float   cruiseMag  = (float) cruiseMagI;
    const float   floorMag   = (float) cfg.pwmMin;

    // Defensive accel floor: never divide the accel-limited passes into a zero/negative slope.
    const float accel = (cfg.accelPwmPerCm > 0.0f) ? cfg.accelPwmPerCm : CHAIN_ACCEL_EPS;

    // Node-speed buffer, sized to the compile-time cap. `hopCount` is a uint8_t bounded by the
    // caller's MAX_CHAIN_HOPS run buffer; CHAIN_MAX_NODES gives the local array a fixed size with
    // headroom so this header carries no heap and no VLA. If a caller ever exceeds the cap it is
    // clamped here so the local write stays in bounds (belt-and-braces — the caller caps first).
    static const uint8_t CHAIN_MAX_NODES = 64;
    uint8_t              nodeCount        = (uint8_t) (hopCount + 1);
    if (nodeCount > CHAIN_MAX_NODES)
    {
        nodeCount = CHAIN_MAX_NODES;
    }
    const uint8_t hops = (uint8_t) (nodeCount - 1);
    float         v[CHAIN_MAX_NODES];

    // ---- 1. Seed. -------------------------------------------------------------------------
    // v[0] = clamp(|entry|, pwmMin, |cruise|); interior/target nodes seed to |cruise|.
    float v0 = (float) entryMagI;
    if (v0 < floorMag)
    {
        v0 = floorMag;
    }
    if (v0 > cruiseMag)
    {
        v0 = cruiseMag;
    }
    v[0] = v0;
    for (uint8_t k = 1; k < nodeCount; ++k)
    {
        v[k] = cruiseMag;
    }

    // ---- 2. Forward pass (accel limit): v[k+1] = min(v[k+1], v[k] + accel*hopDist[k]). -----
    for (uint8_t k = 0; k < hops; ++k)
    {
        float d = hopDistCm[k];
        if (d < 0.0f)
        {
            d = 0.0f;
        }
        const float reachable = v[k] + accel * d;
        if (reachable < v[k + 1])
        {
            v[k + 1] = reachable;
        }
    }

    // ---- 3. Backward pass (decel to floor): terminal node -> pwmMin, chain through v[k+1]. --
    if (floorMag < v[hops])
    {
        v[hops] = floorMag;
    }
    for (int16_t k = (int16_t) (hops - 1); k >= 0; --k)
    {
        float d = hopDistCm[k];
        if (d < 0.0f)
        {
            d = 0.0f;
        }
        const float sheddable = v[k + 1] + accel * d;
        if (sheddable < v[k])
        {
            v[k] = sheddable;
        }
    }

    // ---- 5. Map node speeds to per-hop signed (startPwm, endPwm). Truncate toward zero at the
    //         output boundary, then multiply the run sign in (see BEHAVIOR-IDENTITY note). -----
    for (uint8_t k = 0; k < hops; ++k)
    {
        const int16_t startMag = (int16_t) v[k];
        out[k].startPwm        = (int16_t) (sign * startMag);
        if (k + 1 == hops)
        {
            // Final hop: the ordinary cruise->floor->brake stop (end speed 0, brake true).
            out[k].endPwm = 0;
            out[k].brake  = true;
        }
        else
        {
            const int16_t endMag = (int16_t) v[k + 1];
            out[k].endPwm        = (int16_t) (sign * endMag);
            out[k].brake         = false;
        }
    }
}

#endif    // CHAIN_SPEED_POLICY_H
