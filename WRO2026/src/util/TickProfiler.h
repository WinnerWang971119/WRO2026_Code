/**
 * @file TickProfiler.h
 * @brief Header-only, fixed-size min/avg/max timing accumulator for per-segment and whole-tick
 *        `cooperativeTick()` benchmarking.
 *
 * @details
 * `TickProfiler<MAX_SEGMENTS>` is the pure-arithmetic engine behind `inoTest/tick_bench/`: it
 * accumulates one whole-tick duration plus `MAX_SEGMENTS` named per-segment durations (all in
 * microseconds), tracks running min/avg/max for each, and classifies the measured rate against
 * two Hz thresholds. It carries NO Arduino dependency in the accumulate/getter path — only
 * `<stdint.h>` — so it host-compiles standalone and is exercised by `WRO2026/test/test_tick_profiler.cpp`
 * with injected sample values (the host cannot time a real tick, but it CAN verify the arithmetic).
 *
 * Design rules (see the 2026-07-03 cooperativetick-benchmark plan, T2):
 *  - **Fixed-size, no dynamic allocation.** Every array is `[MAX_SEGMENTS]`, a compile-time
 *    template parameter. No `new`, no STL containers, no `delay()`.
 *  - **Segments registered once, reused across windows.** `addSegment()` is called once per
 *    module in `setup()`; `reset()` zeroes the accumulators but preserves the registered names
 *    and count, so the bench sketch registers exactly once and reports/resets repeatedly.
 *  - **Output is a member template.** `report()` is `template<typename OutT> void report(OutT&, ...)`
 *    so the stream type is DEPENDENT — `Serial` on device, a duck-typed capture struct on host —
 *    with no `Print` base needed at all. There is deliberately no `class Print;` forward decl:
 *    an inline `report(Print&)` would access members of an incomplete, non-dependent type and be
 *    rejected by g++ and MSVC `/permissive-` (the host harness builds with `/permissive-`), and
 *    the host mock (`WRO2026/test/mocks/Arduino.h`) has NO Arduino `Print` base class at all —
 *    only a standalone `MockSerial`.
 *
 * @note Integer averages are `sum / count`. With the bench's `WINDOW_TICKS <= 200` and per-tick
 *       durations well under ~50 ms, both the whole-tick and per-segment `sum` accumulate to at
 *       most `200 * 50000 = 10,000,000` us — far below the `uint32_t` ceiling (~4.29e9). No
 *       overflow guard is needed for the intended window sizes.
 * @note `benchTick()` (in the bench sketch, not here) wraps each of the 10 segments plus the
 *       whole tick in a `micros()` call pair, i.e. ~2*(segments+1) `micros()` calls per tick —
 *       a handful of microseconds of self-overhead that is INSIDE the measured whole-tick number,
 *       not subtracted out. This is negligible next to the millisecond-scale I2C work being
 *       measured (e.g. the LineArray read), so it is not compensated for.
 */

#ifndef TICK_PROFILER_H
#define TICK_PROFILER_H

#include <stdint.h>

/** @brief Sentinel returned by addSegment() when the slot table is full; a no-op sampleSegment idx. */
static constexpr uint8_t  TICK_INVALID_SEGMENT = 0xFF;
/** @brief "no sample yet" minimum sentinel; getters map it to 0 on an empty window. */
static constexpr uint32_t TICK_US_MIN_SENTINEL = 0xFFFFFFFFu;

/** @brief Verdict of a measured average tick time against two Hz thresholds. */
enum class TickVerdict : uint8_t
{
    GOOD,
    MARGINAL,
    TOO_LONG
};

/**
 * @brief Fixed-size accumulator of per-segment + whole-tick timing (min/avg/max microseconds).
 * @tparam MAX_SEGMENTS Compile-time upper bound on named segments (no dynamic allocation; sized
 *                      to the bench's `SEGMENT_COUNT`).
 * @note The accumulate/getter path uses only `<stdint.h>` arithmetic — no Arduino I/O type — so
 *       this header host-compiles standalone. Output goes through the member template `report()`.
 */
template <uint8_t MAX_SEGMENTS>
class TickProfiler
{
public:
    /**
     * @brief Construct an empty profiler: no segments registered, all accumulators at rest.
     * @note Equivalent to a freshly-registered `reset()` state with `_segCount == 0`. The bench
     *       constructs one instance, then calls `addSegment()` once per module in `setup()`.
     */
    TickProfiler()
        : _segCount(0), _tickMin(TICK_US_MIN_SENTINEL), _tickSum(0), _tickMax(0), _tickCount(0)
    {
        for (uint8_t i = 0; i < MAX_SEGMENTS; ++i)
        {
            _names[i]      = nullptr;
            _segMin[i]     = TICK_US_MIN_SENTINEL;
            _segSum[i]     = 0;
            _segMax[i]     = 0;
            _segSamples[i] = 0;
        }
    }

    /**
     * @brief Register a named segment slot.
     * @param name Human-readable segment label (e.g. `"LineArray"`); stored by pointer, not
     *             copied — callers must pass a string literal or otherwise-stable storage, which
     *             every intended caller (the bench sketch, `addSegment("LineArray")` style) does.
     * @return The segment's index (`0` .. `MAX_SEGMENTS - 1`), stable for the profiler's lifetime;
     *         `TICK_INVALID_SEGMENT` if `MAX_SEGMENTS` slots are already registered.
     * @note Registration is intended to happen once, in `setup()`, before any `sampleSegment()`
     *       call. `reset()` does NOT re-register — the count and names persist across windows.
     */
    uint8_t addSegment(const char* name)
    {
        if (_segCount >= MAX_SEGMENTS)
        {
            return TICK_INVALID_SEGMENT;
        }
        const uint8_t idx = _segCount;
        _names[idx]       = name;
        _segMin[idx]       = TICK_US_MIN_SENTINEL;
        _segSum[idx]       = 0;
        _segMax[idx]       = 0;
        _segSamples[idx]   = 0;
        ++_segCount;
        return idx;
    }

    /**
     * @brief Feed one segment's measured duration (microseconds) for the current tick.
     * @param idx  Segment index returned by `addSegment()`.
     * @param dtUs Measured duration in microseconds.
     * @note No-op (bounds-checked, defensive) when `idx == TICK_INVALID_SEGMENT` or
     *       `idx >= ` the number of registered segments — this cannot fire at runtime when the
     *       caller sizes `MAX_SEGMENTS` to match its registered segment count, but is host-tested
     *       so an out-of-range index never writes out of bounds.
     */
    void sampleSegment(uint8_t idx, uint32_t dtUs)
    {
        if (idx == TICK_INVALID_SEGMENT || idx >= _segCount)
        {
            return;
        }
        if (dtUs < _segMin[idx])
        {
            _segMin[idx] = dtUs;
        }
        if (dtUs > _segMax[idx])
        {
            _segMax[idx] = dtUs;
        }
        _segSum[idx] += dtUs;
        ++_segSamples[idx];
    }

    /**
     * @brief Feed the whole-tick measured duration (microseconds) and advance the tick count.
     * @param dtUs Measured whole-tick duration in microseconds.
     */
    void sampleTick(uint32_t dtUs)
    {
        if (dtUs < _tickMin)
        {
            _tickMin = dtUs;
        }
        if (dtUs > _tickMax)
        {
            _tickMax = dtUs;
        }
        _tickSum += dtUs;
        ++_tickCount;
    }

    /** @brief Number of whole-tick samples accumulated since the last `reset()`. */
    uint32_t tickCount() const
    {
        return _tickCount;
    }

    /**
     * @brief Minimum whole-tick duration (microseconds) observed since the last `reset()`.
     * @return `0` on an empty window (the internal sentinel is never exposed).
     */
    uint32_t tickMinUs() const
    {
        return (_tickMin == TICK_US_MIN_SENTINEL) ? 0 : _tickMin;
    }

    /**
     * @brief Average whole-tick duration (microseconds), integer `sum / count`.
     * @return `0` when `tickCount() == 0` (guarded against divide-by-zero).
     */
    uint32_t tickAvgUs() const
    {
        return (_tickCount == 0) ? 0 : (_tickSum / _tickCount);
    }

    /** @brief Maximum whole-tick duration (microseconds) observed since the last `reset()`. */
    uint32_t tickMaxUs() const
    {
        return _tickMax;
    }

    /**
     * @brief Minimum duration (microseconds) observed for segment `idx` since the last `reset()`.
     * @param idx Segment index returned by `addSegment()`.
     * @return `0` on an empty window or an out-of-range `idx` (sentinel / bounds mapped to 0).
     */
    uint32_t segMinUs(uint8_t idx) const
    {
        if (idx >= _segCount)
        {
            return 0;
        }
        return (_segMin[idx] == TICK_US_MIN_SENTINEL) ? 0 : _segMin[idx];
    }

    /**
     * @brief Average duration (microseconds) for segment `idx`, integer `sum / samples`.
     * @param idx Segment index returned by `addSegment()`.
     * @return `0` when the segment has zero samples this window or `idx` is out of range
     *         (guarded against divide-by-zero).
     */
    uint32_t segAvgUs(uint8_t idx) const
    {
        if (idx >= _segCount || _segSamples[idx] == 0)
        {
            return 0;
        }
        return _segSum[idx] / _segSamples[idx];
    }

    /**
     * @brief Maximum duration (microseconds) for segment `idx` since the last `reset()`.
     * @param idx Segment index returned by `addSegment()`.
     * @return `0` for an out-of-range `idx`.
     */
    uint32_t segMaxUs(uint8_t idx) const
    {
        if (idx >= _segCount)
        {
            return 0;
        }
        return _segMax[idx];
    }

    /**
     * @brief Effective whole-tick rate in Hz, derived from the current average.
     * @return `1,000,000 / tickAvgUs()`, integer division; `0` when `tickAvgUs() == 0`
     *         (guarded against divide-by-zero on an empty window).
     */
    uint32_t effectiveHz() const
    {
        const uint32_t avg = tickAvgUs();
        return (avg == 0) ? 0 : (1000000UL / avg);
    }

    /**
     * @brief Classify `effectiveHz()` against two thresholds.
     * @param floorHz Minimum acceptable rate; below this is `TOO_LONG`.
     * @param goodHz  Rate at or above which the tick is `GOOD`.
     * @return `TickVerdict::GOOD` when `hz >= goodHz`; `TickVerdict::MARGINAL` when
     *         `floorHz <= hz < goodHz`; `TickVerdict::TOO_LONG` otherwise (including the
     *         `hz == 0` empty-window case).
     */
    TickVerdict classify(uint32_t floorHz, uint32_t goodHz) const
    {
        const uint32_t hz = effectiveHz();
        if (hz >= goodHz)
        {
            return TickVerdict::GOOD;
        }
        if (hz >= floorHz)
        {
            return TickVerdict::MARGINAL;
        }
        return TickVerdict::TOO_LONG;
    }

    /**
     * @brief Print the full summary (per-segment + whole-tick + Hz + verdict) to `out`.
     * @tparam OutT Any type exposing Arduino-style `print()`/`println()` overloads for
     *              `const char*` and `uint32_t` — `Serial` on device, a duck-typed capture struct
     *              in the host ctest. The stream type is a template parameter (not `Print&`) so
     *              this header stays Arduino-I/O-free until `report()` is instantiated.
     * @param out      Destination stream.
     * @param floorHz  Passed through to `classify()`.
     * @param goodHz   Passed through to `classify()`.
     * @note Emits the EXACT line grammar pinned in the benchmark plan's Contracts -> Naming
     *       section (byte-for-byte, asserted by the host test):
     *         - `-- tick bench: <N> ticks --`
     *         - one `  SEG <name> us min=<min> avg=<avg> max=<max>` line per registered segment,
     *           in registration order
     *         - `TICK us min=<min> avg=<avg> max=<max> | ~<hz> Hz | <VERDICT>` last
     *       All `<min>/<avg>/<max>` print as integer microseconds, `0` (never the raw sentinel)
     *       on an empty window. Uses only `out.print()` / `out.println()` — no printf, no String.
     */
    template <typename OutT>
    void report(OutT& out, uint32_t floorHz, uint32_t goodHz) const
    {
        out.print("-- tick bench: ");
        out.print(tickCount());
        out.println(" ticks --");

        for (uint8_t i = 0; i < _segCount; ++i)
        {
            out.print("  SEG ");
            out.print(_names[i]);
            out.print(" us min=");
            out.print(segMinUs(i));
            out.print(" avg=");
            out.print(segAvgUs(i));
            out.print(" max=");
            out.println(segMaxUs(i));
        }

        out.print("TICK us min=");
        out.print(tickMinUs());
        out.print(" avg=");
        out.print(tickAvgUs());
        out.print(" max=");
        out.print(tickMaxUs());
        out.print(" | ~");
        out.print(effectiveHz());
        out.print(" Hz | ");
        out.println(verdictName(classify(floorHz, goodHz)));
    }

    /**
     * @brief Zero every accumulator and the tick count; registered segment names and count survive.
     * @note Whole-tick and every registered segment's min resets to `TICK_US_MIN_SENTINEL`, max to
     *       `0`, sum/count/samples to `0`. Segments registered via `addSegment()` before this call
     *       remain registered (same index, same name) so the bench can register once in `setup()`
     *       and `reset()` every window without re-registering.
     */
    void reset()
    {
        _tickMin   = TICK_US_MIN_SENTINEL;
        _tickSum   = 0;
        _tickMax   = 0;
        _tickCount = 0;
        for (uint8_t i = 0; i < _segCount; ++i)
        {
            _segMin[i]     = TICK_US_MIN_SENTINEL;
            _segSum[i]     = 0;
            _segMax[i]     = 0;
            _segSamples[i] = 0;
        }
    }

private:
    /**
     * @brief Map a `TickVerdict` to its exact report-grammar token.
     * @param v Verdict to render.
     * @return `"GOOD"`, `"MARGINAL"`, or `"TOO_LONG"`; falls back to `"TOO_LONG"` for any
     *         unrecognised value (defensive default — calms the compiler on the no-default
     *         switch and survives a future enum extension that forgets to update this function,
     *         matching the `presetToPwm` precedent in `WROMath.h`).
     */
    static const char* verdictName(TickVerdict v)
    {
        switch (v)
        {
        case TickVerdict::GOOD:
            return "GOOD";
        case TickVerdict::MARGINAL:
            return "MARGINAL";
        case TickVerdict::TOO_LONG:
            return "TOO_LONG";
        }
        return "TOO_LONG";
    }

    /** @brief Registered segment labels, indexed 0.._segCount-1; stored by pointer, not copied. */
    const char* _names[MAX_SEGMENTS];

    /** @brief Number of segments registered via `addSegment()` so far (0..MAX_SEGMENTS). */
    uint8_t _segCount;

    /** @brief Per-segment running minimum (microseconds); `TICK_US_MIN_SENTINEL` until sampled. */
    uint32_t _segMin[MAX_SEGMENTS];

    /** @brief Per-segment running sum (microseconds) since the last `reset()`. */
    uint32_t _segSum[MAX_SEGMENTS];

    /** @brief Per-segment running maximum (microseconds) since the last `reset()`. */
    uint32_t _segMax[MAX_SEGMENTS];

    /**
     * @brief Per-segment sample count since the last `reset()`.
     *
     * Tracked independently of `_tickCount` (rather than assuming every segment is sampled every
     * tick) so `segAvgUs()`/`segMinUs()` are guarded even if a caller's per-tick loop skips a
     * segment on some pass (e.g. a conditional module).
     */
    uint32_t _segSamples[MAX_SEGMENTS];

    /** @brief Whole-tick running minimum (microseconds); `TICK_US_MIN_SENTINEL` until sampled. */
    uint32_t _tickMin;

    /** @brief Whole-tick running sum (microseconds) since the last `reset()`. */
    uint32_t _tickSum;

    /** @brief Whole-tick running maximum (microseconds) since the last `reset()`. */
    uint32_t _tickMax;

    /** @brief Whole-tick sample count since the last `reset()`. */
    uint32_t _tickCount;
};

#endif    // TICK_PROFILER_H
