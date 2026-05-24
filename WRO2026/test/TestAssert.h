/**
 * @file TestAssert.h
 * @brief Tiny assertion helpers for the host-compile test harness.
 *
 * No GoogleTest / Catch2 dependency — each test is a standalone `main()` that
 * exits nonzero on the first failing assert. CMake's `add_test` interprets a
 * nonzero exit code as failure.
 *
 * On success the test prints `PASS: <test name>`; on failure it prints a
 * one-line `FAIL:` message with the file/line and the failing expression, then
 * exits with status 1.
 */

#ifndef WRO_TEST_ASSERT_H
#define WRO_TEST_ASSERT_H

#include <cstdio>
#include <cstdlib>

// ---------------------------------------------------------------------------
// Per-test counters — incremented by the macros, reported by TEST_REPORT().
// ---------------------------------------------------------------------------

namespace wro_test
{
    inline int& failCounter()
    {
        static int n = 0;
        return n;
    }
    inline int& assertCounter()
    {
        static int n = 0;
        return n;
    }
    inline const char*& currentScope()
    {
        static const char* s = "(unknown)";
        return s;
    }
}    // namespace wro_test

// ---------------------------------------------------------------------------
// ASSERT macros — each one increments the per-test counters and prints on
// failure but does NOT abort, so a single test main can report multiple
// failures in one run. The final `TEST_REPORT()` exits nonzero if any
// failed.
// ---------------------------------------------------------------------------

#define ASSERT_TRUE(expr)                                                                          \
    do                                                                                             \
    {                                                                                              \
        wro_test::assertCounter()++;                                                               \
        if (!(expr))                                                                               \
        {                                                                                          \
            wro_test::failCounter()++;                                                             \
            std::fprintf(stderr,                                                                   \
                         "FAIL: [%s] %s:%d: ASSERT_TRUE(%s)\n",                                    \
                         wro_test::currentScope(),                                                 \
                         __FILE__,                                                                 \
                         __LINE__,                                                                 \
                         #expr);                                                                   \
        }                                                                                          \
    } while (0)

#define ASSERT_FALSE(expr)                                                                         \
    do                                                                                             \
    {                                                                                              \
        wro_test::assertCounter()++;                                                               \
        if ((expr))                                                                                \
        {                                                                                          \
            wro_test::failCounter()++;                                                             \
            std::fprintf(stderr,                                                                   \
                         "FAIL: [%s] %s:%d: ASSERT_FALSE(%s)\n",                                   \
                         wro_test::currentScope(),                                                 \
                         __FILE__,                                                                 \
                         __LINE__,                                                                 \
                         #expr);                                                                   \
        }                                                                                          \
    } while (0)

#define ASSERT_EQ(actual, expected)                                                                \
    do                                                                                             \
    {                                                                                              \
        wro_test::assertCounter()++;                                                               \
        const auto _a = (actual);                                                                  \
        const auto _e = (expected);                                                                \
        if (!(_a == _e))                                                                           \
        {                                                                                          \
            wro_test::failCounter()++;                                                             \
            std::fprintf(stderr,                                                                   \
                         "FAIL: [%s] %s:%d: ASSERT_EQ(%s, %s) — got %lld, expected %lld\n",        \
                         wro_test::currentScope(),                                                 \
                         __FILE__,                                                                 \
                         __LINE__,                                                                 \
                         #actual,                                                                  \
                         #expected,                                                                \
                         (long long) _a,                                                           \
                         (long long) _e);                                                          \
        }                                                                                          \
    } while (0)

#define ASSERT_GE(actual, threshold)                                                               \
    do                                                                                             \
    {                                                                                              \
        wro_test::assertCounter()++;                                                               \
        const auto _a = (actual);                                                                  \
        const auto _t = (threshold);                                                               \
        if (!(_a >= _t))                                                                           \
        {                                                                                          \
            wro_test::failCounter()++;                                                             \
            std::fprintf(stderr,                                                                   \
                         "FAIL: [%s] %s:%d: ASSERT_GE(%s, %s) — got %lld, threshold %lld\n",       \
                         wro_test::currentScope(),                                                 \
                         __FILE__,                                                                 \
                         __LINE__,                                                                 \
                         #actual,                                                                  \
                         #threshold,                                                               \
                         (long long) _a,                                                           \
                         (long long) _t);                                                          \
        }                                                                                          \
    } while (0)

#define ASSERT_GT(actual, threshold)                                                               \
    do                                                                                             \
    {                                                                                              \
        wro_test::assertCounter()++;                                                               \
        const auto _a = (actual);                                                                  \
        const auto _t = (threshold);                                                               \
        if (!(_a > _t))                                                                            \
        {                                                                                          \
            wro_test::failCounter()++;                                                             \
            std::fprintf(stderr,                                                                   \
                         "FAIL: [%s] %s:%d: ASSERT_GT(%s, %s) — got %lld, threshold %lld\n",       \
                         wro_test::currentScope(),                                                 \
                         __FILE__,                                                                 \
                         __LINE__,                                                                 \
                         #actual,                                                                  \
                         #threshold,                                                               \
                         (long long) _a,                                                           \
                         (long long) _t);                                                          \
        }                                                                                          \
    } while (0)

#define TEST_SCOPE(name) wro_test::currentScope() = (name)

#define TEST_REPORT()                                                                              \
    do                                                                                             \
    {                                                                                              \
        if (wro_test::failCounter() == 0)                                                          \
        {                                                                                          \
            std::printf("PASS: %d assertions\n", wro_test::assertCounter());                       \
            return 0;                                                                              \
        }                                                                                          \
        std::fprintf(stderr,                                                                       \
                     "RESULT: %d/%d assertions failed\n",                                          \
                     wro_test::failCounter(),                                                      \
                     wro_test::assertCounter());                                                   \
        return 1;                                                                                  \
    } while (0)

#endif    // WRO_TEST_ASSERT_H
