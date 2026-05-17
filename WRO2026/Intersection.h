/**
 * @file Intersection.h
 * @brief Edge-triggered junction/intersection detector built on `MatrixLineTracer`.
 * @author WRO 2026
 *
 * @details
 * `IntersectionClass` is the singleton that turns the raw, polled
 * `MatrixLineTracer::getJunctionType()` reading into a one-shot, edge-triggered event the rest
 * of the firmware can consume cleanly. Without edge detection, naive callers polling
 * `getJunctionType()` would see the same intersection reported on every loop tick for as long as
 * the chassis straddles the marker — leading to duplicate path-tracker decisions, OLED log
 * spam, and stuck state-machine transitions.
 *
 * Design rules:
 *  - **Shares the LineFollower's tracer pointer.** `begin()` calls `LineFollower.getTracer()`
 *    and binds onto the same `MatrixLineTracer` instance, so we issue zero extra I2C
 *    transactions on the Wire1 mux channel — every `update()` reuses the sample LineFollower
 *    just took. Caller MUST invoke `LineFollower.begin()` before `Intersection.begin()`.
 *  - **`update()` is non-blocking** — no `delay()`, no I2C wait loops. Cooperative tick.
 *  - **Edge-triggered consumption** — `isAtIntersection()` returns `true` exactly once per
 *    junction *entry* (rising edge of "at-junction"), then auto-clears.
 *  - **`JunctionType` enum mirrors the library's 0..4 values exactly**, so a plain
 *    `static_cast<JunctionType>(raw)` is the only conversion ever needed.
 *
 */

#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <Arduino.h>
#include <MatrixMiniR4.h>    // brings in `MatrixLineTracer` transitively (see code review F9
                             // for why we don't include `<MiniR4_MXLineTracer.h>` directly).

/**
 * @brief Junction type mirror of `MatrixLineTracer::getJunctionType()` return values.
 *
 * Underlying values are stable (must match the library's 0..4 convention exactly) so that the
 * raw `uint8_t` returned by the driver can be converted with a plain
 * `static_cast<JunctionType>(raw)` and round-tripped back the same way.
 *
 * @note `UNKNOWN` (4) is reported by the sensor when *something* was detected at the line
 *       array but it could not be classified. The detector treats UNKNOWN the same as NONE
 *       (i.e. no edge fires) to avoid acting on noisy / ambiguous samples.
 */
enum class JunctionType : uint8_t
{
    NONE       = 0,    ///< No junction under the sensor.
    LEFT       = 1,    ///< Left branch detected.
    RIGHT      = 2,    ///< Right branch detected.
    T_OR_CROSS = 3,    ///< T-junction or 4-way crossroads.
    UNKNOWN    = 4     ///< Sensor reported a junction but could not classify it.
};

/**
 * @brief Singleton edge-triggered intersection detector.
 *
 * Owns no hardware of its own — borrows the `MatrixLineTracer*` exposed by `LineFollower` so
 * both modules read the same I2C sample. State machine on each `update()`:
 *
 *  - Sample `raw = _tracer->getJunctionType()`.
 *  - `atJunctionNow = (raw != NONE && raw != UNKNOWN)`.
 *  - If `atJunctionNow && !_wasAtJunction`  →  rising edge: latch `_lastType`, set
 *    `_edgeFired = true`.
 *  - Always update `_wasAtJunction = atJunctionNow`.
 *
 * `isAtIntersection()` is the read-and-clear consumer for `_edgeFired`. `getLastType()` is a
 * pure getter — it survives consumption of the edge, so callers can inspect the type after
 * acknowledging the event.
 */
class IntersectionClass
{
public:
    /**
     * @brief Bind to LineFollower's shared tracer and reset detector state.
     *
     * @return `true` if `LineFollower.getTracer()` returned a non-null pointer; `false`
     *         otherwise (defensive — should never happen if `LineFollower.begin()` was called
     *         first, in which case `update()` and friends will silently no-op).
     * @note Must be called from `setup()` AFTER `LineFollower.begin()` so the underlying
     *       `MatrixLineTracer` has been initialised on Wire1 + the configured mux channel.
     */
    bool begin();

    /**
     * @brief Cooperative tick — call every iteration of the main `loop()`.
     *
     * Samples the shared tracer, runs the rising-edge detector, and (on a rising edge) latches
     * the captured junction type into `_lastType` and raises `_edgeFired`. Does not consume
     * the flag — that is `isAtIntersection()`'s job.
     *
     * @note Non-blocking. Contains NO `delay()`. Safe to call at any cadence.
     * @note Edge detection: returns rising edge of `(raw != NONE && raw != UNKNOWN)`. Treats
     *       both NONE and UNKNOWN as "no junction" so a momentary unclassified sample doesn't
     *       fire.
     * @note Should be called AFTER `LineFollower.update()` in the main loop so both modules
     *       observe the same I2C cycle's worth of state on the shared tracer.
     */
    void update();

    /**
     * @brief Read-and-clear consumer for the edge-triggered junction event.
     *
     * @return `true` if a rising edge has been latched since the last call; `false` otherwise.
     *         Always clears the internal flag before returning.
     * @note Read-and-clear. Returns true exactly once per junction entry. Subsequent calls
     *       return false until the sensor leaves the junction and re-enters. Reserved for
     *       the **single** owning consumer (e.g. the navigation/path-tracker layer). Other
     *       observers (telemetry, OLED logging) should use `getEdgeCount()` instead — see
     *       code review F6.
     * @note Intentionally non-`const` because it mutates `_edgeFired`.
     */
    bool isAtIntersection();

    /**
     * @brief Non-consuming, multi-observer rising-edge counter.
     *
     * Strictly increments by exactly one each time `update()` detects a rising edge of the
     * "at-junction" condition. Observers that need to react to edges WITHOUT contending
     * with the navigation layer's read-and-clear `isAtIntersection()` should cache the
     * value returned here and fire whenever a new tick produces a different value.
     *
     * @return Total number of rising edges since `begin()`. Wraps after `2^32` events,
     *         which is irrelevant in any realistic competition run.
     * @note Use this in EventTrace and any future passive observer. The single consuming
     *       consumer should still use `isAtIntersection()` because that doubles as its
     *       state-machine arming bit.
     * @note See code review F6.
     */
    uint32_t getEdgeCount() const;

    /**
     * @brief Type classification captured at the most recent rising edge.
     *
     * @return The `JunctionType` latched the last time `update()` saw a NONE/UNKNOWN → real
     *         junction transition. `JunctionType::NONE` until the first rising edge.
     * @note Stays valid after `isAtIntersection()` consumes the edge — useful for retrieving
     *       the junction type after acknowledging the event.
     */
    JunctionType getLastType() const;

private:
    MatrixLineTracer* _tracer        = nullptr;                ///< Shared tracer from LineFollower.
    bool              _wasAtJunction = false;                  ///< Debounced at-junction state.
    bool              _edgeFired     = false;                  ///< Latched rising edge, cleared on read.
    JunctionType      _lastType      = JunctionType::NONE;     ///< Type captured at the last edge.
    uint32_t          _edgeCount     = 0;                      ///< Strictly-increasing edge counter (F6).
    uint8_t           _validStreak   = 0;                      ///< Consecutive valid-junction ticks (F7).
    uint8_t           _noneStreak    = 0;                      ///< Consecutive NONE ticks (F7).
};

/**
 * @brief Single global instance — defined in Intersection.cpp.
 */
extern IntersectionClass Intersection;

#endif    // INTERSECTION_H
