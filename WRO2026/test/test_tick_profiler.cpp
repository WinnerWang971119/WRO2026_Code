/**
 * @file test_tick_profiler.cpp
 * @brief Host-compile tests for `WRO2026/TickProfiler.h` — the pure min/avg/max
 *        accumulation math behind `inoTest/tick_bench/` (#123, T4).
 *
 * `TickProfiler<MAX_SEGMENTS>` carries NO Arduino dependency in its accumulate/
 * getter path (only `<stdint.h>`), and `report()` is a member template on the
 * stream type — so a duck-typed `CaptureStream` (no `Print` base; the harness's
 * `mocks/Arduino.h` has none) exercises it here with zero mock linkage. This
 * target therefore compiles ONLY this .cpp — no production `.cpp`, no
 * `MOCK_SOURCES` — see the 2026-07-03 cooperativetick-benchmark plan, T4 and
 * the Contracts -> Naming section for the exact `report()` grammar asserted by
 * TC7.
 *
 * Covers TC1-TC7 from the plan's Testing Strategy:
 *   TC1 - sampleSegment/sampleTick fed known dts -> exact per-segment + whole-
 *         tick min/max/avg.
 *   TC2 - effectiveHz() from a known avg (28571 us -> 35 Hz).
 *   TC3 - classify(30, 45) at 50/35/20 Hz -> GOOD/MARGINAL/TOO_LONG.
 *   TC4 - zero-tick window: no div-by-zero, min maps to 0 (never the sentinel),
 *         verdict TOO_LONG, report() prints min=0/max=0.
 *   TC5 - reset() zeroes accumulators but keeps registered segment names/count.
 *   TC6 - addSegment() overflow -> TICK_INVALID_SEGMENT; sampleSegment() on
 *         that sentinel is a no-op (no OOB write, no crash).
 *   TC7 - report()'s exact line grammar, byte-for-byte, in registration order.
 */

#include "TestAssert.h"

#include "TickProfiler.h"

#include <string>

// ===========================================================================
// CaptureStream — duck-typed host stand-in for `Serial`. `report()` is a
// member template on its stream type, so this needs no `Print` base (there is
// none in this harness). Overloads match EXACTLY what `TickProfiler::report()`
// calls: `print(const char*)`, `print(uint32_t)`, `println(const char*)`, and
// `println(uint32_t)` (the per-segment line ends with `println(segMaxUs(i))`,
// a `uint32_t`, not a string).
// ===========================================================================

struct CaptureStream
{
    std::string buf;

    void print(const char* s)
    {
        buf += s;
    }

    void print(uint32_t v)
    {
        buf += std::to_string(v);
    }

    void println(const char* s)
    {
        buf += s;
        buf += "\n";
    }

    void println(uint32_t v)
    {
        buf += std::to_string(v);
        buf += "\n";
    }
};

// ===========================================================================
// TC1 - known dt vectors -> exact per-segment and whole-tick min/max/avg.
// ===========================================================================

static void test_TC1_segment_and_tick_min_avg_max()
{
    TEST_SCOPE("TC1: sampleSegment/sampleTick accumulate exact min/avg/max");

    TickProfiler<3> prof;
    const uint8_t   segLineArray = prof.addSegment("LineArray");
    const uint8_t   segDrive     = prof.addSegment("Drive");
    const uint8_t   segIdle      = prof.addSegment("LineFollower");    // never sampled below

    // LineArray: {2000, 3000, 2500} us -> min=2000, max=3000, avg=7500/3=2500.
    prof.sampleSegment(segLineArray, 2000u);
    prof.sampleSegment(segLineArray, 3000u);
    prof.sampleSegment(segLineArray, 2500u);
    ASSERT_EQ(prof.segMinUs(segLineArray), (uint32_t) 2000u);
    ASSERT_EQ(prof.segMaxUs(segLineArray), (uint32_t) 3000u);
    ASSERT_EQ(prof.segAvgUs(segLineArray), (uint32_t) 2500u);

    // Drive: single sample -> min == max == avg == that one value.
    prof.sampleSegment(segDrive, 500u);
    ASSERT_EQ(prof.segMinUs(segDrive), (uint32_t) 500u);
    ASSERT_EQ(prof.segMaxUs(segDrive), (uint32_t) 500u);
    ASSERT_EQ(prof.segAvgUs(segDrive), (uint32_t) 500u);

    // LineFollower: never sampled this window -> guarded zero, not garbage.
    ASSERT_EQ(prof.segMinUs(segIdle), (uint32_t) 0u);
    ASSERT_EQ(prof.segMaxUs(segIdle), (uint32_t) 0u);
    ASSERT_EQ(prof.segAvgUs(segIdle), (uint32_t) 0u);

    // Whole tick: {10000, 15000, 12000} us -> min=10000, max=15000,
    // avg=37000/3=12333 (integer sum/count, truncated).
    prof.sampleTick(10000u);
    prof.sampleTick(15000u);
    prof.sampleTick(12000u);
    ASSERT_EQ(prof.tickCount(), (uint32_t) 3u);
    ASSERT_EQ(prof.tickMinUs(), (uint32_t) 10000u);
    ASSERT_EQ(prof.tickMaxUs(), (uint32_t) 15000u);
    ASSERT_EQ(prof.tickAvgUs(), (uint32_t) 12333u);
}

// ===========================================================================
// TC2 - effectiveHz() from a known average.
// ===========================================================================

static void test_TC2_effective_hz_from_known_average()
{
    TEST_SCOPE("TC2: effectiveHz() == integer 1e6 / avg");

    // A single 28571 us sample makes avg == 28571 exactly (sum/count == 28571/1).
    TickProfiler<1> prof;
    prof.sampleTick(28571u);
    ASSERT_EQ(prof.tickAvgUs(), (uint32_t) 28571u);
    ASSERT_EQ(prof.effectiveHz(), (uint32_t) 35u);    // 1,000,000 / 28571 == 35.00028 -> 35
}

// ===========================================================================
// TC3 - classify(30, 45) at three known rates.
// ===========================================================================

static void test_TC3_classify_at_three_rates()
{
    TEST_SCOPE("TC3: classify(30, 45) -> GOOD / MARGINAL / TOO_LONG");

    // 50 Hz: avg = 1,000,000 / 50 = 20000 us exactly -> hz = 50 -> GOOD (>= goodHz).
    TickProfiler<1> profGood;
    profGood.sampleTick(20000u);
    ASSERT_TRUE(profGood.classify(30u, 45u) == TickVerdict::GOOD);

    // 35 Hz: avg = 28571 us (as in TC2) -> hz = 35 -> MARGINAL (>= floorHz, < goodHz).
    TickProfiler<1> profMarginal;
    profMarginal.sampleTick(28571u);
    ASSERT_TRUE(profMarginal.classify(30u, 45u) == TickVerdict::MARGINAL);

    // 20 Hz: avg = 1,000,000 / 20 = 50000 us exactly -> hz = 20 -> TOO_LONG (< floorHz).
    TickProfiler<1> profTooLong;
    profTooLong.sampleTick(50000u);
    ASSERT_TRUE(profTooLong.classify(30u, 45u) == TickVerdict::TOO_LONG);
}

// ===========================================================================
// TC4 - zero-tick window: no div-by-zero, sentinel never leaks, report() is safe.
// ===========================================================================

static void test_TC4_zero_tick_window()
{
    TEST_SCOPE("TC4: empty window guards div-by-zero and never exposes the raw sentinel");

    TickProfiler<2> prof;
    prof.addSegment("SegA");
    prof.addSegment("SegB");
    // No sampleTick()/sampleSegment() calls at all this window.

    ASSERT_EQ(prof.effectiveHz(), (uint32_t) 0u);
    ASSERT_EQ(prof.tickAvgUs(), (uint32_t) 0u);
    ASSERT_EQ(prof.tickMinUs(), (uint32_t) 0u);    // sentinel-mapped, NOT 0xFFFFFFFF
    ASSERT_TRUE(prof.classify(30u, 45u) == TickVerdict::TOO_LONG);

    CaptureStream cap;
    prof.report(cap, 30u, 45u);    // must not crash / divide by zero

    // Exact grammar on an empty window: every min/avg/max prints 0, hz prints 0.
    const std::string expected = "-- tick bench: 0 ticks --\n"
                                  "  SEG SegA us min=0 avg=0 max=0\n"
                                  "  SEG SegB us min=0 avg=0 max=0\n"
                                  "TICK us min=0 avg=0 max=0 | ~0 Hz | TOO_LONG\n";
    ASSERT_TRUE(cap.buf == expected);
    ASSERT_TRUE(cap.buf.find("min=0") != std::string::npos);
    ASSERT_TRUE(cap.buf.find("max=0") != std::string::npos);
}

// ===========================================================================
// TC5 - reset() zeroes accumulators but keeps registered segment names/count.
// ===========================================================================

static void test_TC5_reset_preserves_registration()
{
    TEST_SCOPE("TC5: reset() zeroes counters but segment names/count survive");

    TickProfiler<2> prof;
    const uint8_t   a = prof.addSegment("Alpha");
    const uint8_t   b = prof.addSegment("Beta");

    prof.sampleSegment(a, 1000u);
    prof.sampleSegment(b, 2000u);
    prof.sampleTick(3000u);
    ASSERT_EQ(prof.tickCount(), (uint32_t) 1u);    // sanity: accumulated before reset

    prof.reset();

    ASSERT_EQ(prof.tickCount(), (uint32_t) 0u);
    ASSERT_EQ(prof.tickMinUs(), (uint32_t) 0u);
    ASSERT_EQ(prof.tickMaxUs(), (uint32_t) 0u);
    ASSERT_EQ(prof.tickAvgUs(), (uint32_t) 0u);
    ASSERT_EQ(prof.segMinUs(a), (uint32_t) 0u);
    ASSERT_EQ(prof.segAvgUs(a), (uint32_t) 0u);
    ASSERT_EQ(prof.segMaxUs(a), (uint32_t) 0u);
    ASSERT_EQ(prof.segMinUs(b), (uint32_t) 0u);
    ASSERT_EQ(prof.segAvgUs(b), (uint32_t) 0u);
    ASSERT_EQ(prof.segMaxUs(b), (uint32_t) 0u);

    // Names/registration survive reset() -> a follow-up report() still lists them,
    // in the original registration order (Alpha before Beta).
    CaptureStream cap;
    prof.report(cap, 30u, 45u);
    const std::size_t posAlpha = cap.buf.find("SEG Alpha");
    const std::size_t posBeta  = cap.buf.find("SEG Beta");
    ASSERT_TRUE(posAlpha != std::string::npos);
    ASSERT_TRUE(posBeta != std::string::npos);
    ASSERT_TRUE(posAlpha < posBeta);
}

// ===========================================================================
// TC6 - addSegment() overflow -> TICK_INVALID_SEGMENT; sampleSegment() no-ops.
// ===========================================================================

static void test_TC6_add_segment_overflow_is_safe()
{
    TEST_SCOPE("TC6: addSegment() beyond MAX_SEGMENTS returns TICK_INVALID_SEGMENT, no OOB write");

    TickProfiler<2> prof;
    const uint8_t   idx0 = prof.addSegment("A");
    const uint8_t   idx1 = prof.addSegment("B");
    ASSERT_EQ(idx0, (uint8_t) 0u);
    ASSERT_EQ(idx1, (uint8_t) 1u);

    // The table is full (MAX_SEGMENTS == 2) -> the third registration fails.
    const uint8_t idx2 = prof.addSegment("C");
    ASSERT_EQ(idx2, TICK_INVALID_SEGMENT);

    // Sampling the sentinel index must be a no-op: no crash, no OOB write, and
    // the already-registered segments stay untouched.
    prof.sampleSegment(TICK_INVALID_SEGMENT, 9999u);
    ASSERT_EQ(prof.segMinUs(idx0), (uint32_t) 0u);
    ASSERT_EQ(prof.segMaxUs(idx0), (uint32_t) 0u);
    ASSERT_EQ(prof.segMinUs(idx1), (uint32_t) 0u);
    ASSERT_EQ(prof.segMaxUs(idx1), (uint32_t) 0u);

    // Reading the sentinel index itself is also bounds-checked -> 0, not garbage.
    ASSERT_EQ(prof.segMinUs(TICK_INVALID_SEGMENT), (uint32_t) 0u);
    ASSERT_EQ(prof.segAvgUs(TICK_INVALID_SEGMENT), (uint32_t) 0u);
    ASSERT_EQ(prof.segMaxUs(TICK_INVALID_SEGMENT), (uint32_t) 0u);

    // The real segments still work normally after the failed overflow add.
    prof.sampleSegment(idx1, 1234u);
    ASSERT_EQ(prof.segMinUs(idx1), (uint32_t) 1234u);
    ASSERT_EQ(prof.segMaxUs(idx1), (uint32_t) 1234u);
}

// ===========================================================================
// TC7 - report()'s exact line grammar, byte-for-byte, in registration order.
//
// Source of truth: the benchmark plan's Contracts -> Naming section:
//   - `-- tick bench: <N> ticks --`
//   - one `  SEG <name> us min=<min> avg=<avg> max=<max>` line per segment,
//     in REGISTRATION order
//   - `TICK us min=<min> avg=<avg> max=<max> | ~<hz> Hz | <VERDICT>` last
// ===========================================================================

static void test_TC7_report_grammar_exact()
{
    TEST_SCOPE("TC7: report() emits the exact Contracts->Naming grammar, in registration order");

    TickProfiler<2> prof;
    const uint8_t   idxLineArray = prof.addSegment("LineArray");
    const uint8_t   idxDrive     = prof.addSegment("Drive");

    // Tick 1.
    prof.sampleSegment(idxLineArray, 3000u);
    prof.sampleSegment(idxDrive, 500u);
    prof.sampleTick(4000u);

    // Tick 2.
    prof.sampleSegment(idxLineArray, 5000u);
    prof.sampleSegment(idxDrive, 700u);
    prof.sampleTick(6000u);

    // LineArray: min=3000, max=5000, avg=(3000+5000)/2=4000.
    // Drive:     min=500,  max=700,  avg=(500+700)/2=600.
    // Whole tick: min=4000, max=6000, avg=(4000+6000)/2=5000 ->
    //   effectiveHz = 1,000,000 / 5000 = 200 -> classify(30,45): 200 >= 45 -> GOOD.
    CaptureStream cap;
    prof.report(cap, 30u, 45u);

    const std::string expected = "-- tick bench: 2 ticks --\n"
                                  "  SEG LineArray us min=3000 avg=4000 max=5000\n"
                                  "  SEG Drive us min=500 avg=600 max=700\n"
                                  "TICK us min=4000 avg=5000 max=6000 | ~200 Hz | GOOD\n";
    ASSERT_TRUE(cap.buf == expected);

    // Belt-and-suspenders: assert the ordering explicitly too (LineArray, the
    // first-registered segment, must appear before Drive, and the TICK summary
    // line must come last).
    const std::size_t posLineArray = cap.buf.find("SEG LineArray");
    const std::size_t posDrive     = cap.buf.find("SEG Drive");
    const std::size_t posTick      = cap.buf.find("TICK us min=");
    ASSERT_TRUE(posLineArray != std::string::npos);
    ASSERT_TRUE(posDrive != std::string::npos);
    ASSERT_TRUE(posTick != std::string::npos);
    ASSERT_TRUE(posLineArray < posDrive);
    ASSERT_TRUE(posDrive < posTick);
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: TickProfiler min/avg/max accumulation math (TC1-TC7)\n");

    test_TC1_segment_and_tick_min_avg_max();
    test_TC2_effective_hz_from_known_average();
    test_TC3_classify_at_three_rates();
    test_TC4_zero_tick_window();
    test_TC5_reset_preserves_registration();
    test_TC6_add_segment_overflow_is_safe();
    test_TC7_report_grammar_exact();

    TEST_REPORT();
}
