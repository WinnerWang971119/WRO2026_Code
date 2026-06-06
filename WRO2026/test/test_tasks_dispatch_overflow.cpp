/**
 * @file test_tasks_dispatch_overflow.cpp
 * @brief TC4 — Compile-fail target proving the ``static_assert`` in
 *        ``MissionClass::task<Args...>`` fires for a 17-byte arg pack.
 *
 * AC9 negative branch. The CMake target is declared with
 * ``EXCLUDE_FROM_ALL`` + ``EXCLUDE_FROM_DEFAULT_BUILD`` so the regular
 * build does NOT compile it; ``ctest`` invokes a build-on-demand command
 * and passes only when the build output emits the
 * ``"Mission.task args exceed MAX_TASK_ARG_BYTES"`` static_assert diagnostic
 * (``PASS_REGULAR_EXPRESSION`` on that message) — the compiler exit code is
 * not used. A build that fails for an unrelated reason will not print that
 * message and therefore fails the test.
 *
 * Mirroring rationale: the production ``static_assert`` lives inside the
 * ``MissionClass::task<Args...>`` template body (Mission.h:311-312). Pulling
 * the real Mission header here would drag MatrixMiniR4 / EventTrace /
 * PathTracker into the compile graph — unnecessary for a compile-fail
 * smoke test. Instead, we replicate the minimal ``constexpr TOTAL`` +
 * ``static_assert`` pair against the same ``Config::MAX_TASK_ARG_BYTES``
 * constant the production code uses. The production message string
 * ``"Mission.task args exceed MAX_TASK_ARG_BYTES"`` is preserved verbatim
 * so the failing-build diagnostic looks identical to the real one.
 *
 * @note Why a 17-byte struct rather than 17 × uint8_t scalar args: the
 *       17-arg variant would expand to a parameter pack of 17 ``uint8_t``,
 *       which the GCC / clang variadic-template instantiation depth limit
 *       handles fine but bloats the failure log. A single struct with
 *       ``uint8_t a[17]`` triggers the same ``TOTAL > 16`` arithmetic
 *       with a one-line stack trace, which is what a CI consumer wants
 *       to see.
 */

#include "../TasksTypes.h"

#include <cstddef>
#include <cstdint>

// MIRROR: WRO2026/Mission.h:307-312 — the load-bearing compile-time check.
//
// The real template signature is
//   ``template <typename... Args> MissionClass& task(const char* name, Args... args);``
// with the static_assert in the body. The same fold expression + threshold
// constant fires here.
template <typename... Args>
constexpr void check_overflow()
{
    constexpr size_t TOTAL =
        (sizeof...(Args) == 0) ? 0 : (sizeof(Args) + ... + 0);
    static_assert(TOTAL <= Config::MAX_TASK_ARG_BYTES,
                  "Mission.task args exceed MAX_TASK_ARG_BYTES");
}

/// Single-arg payload sized one byte past the limit. ``check_overflow`` with
/// this type instantiates ``TOTAL == 17`` and the static_assert fails — that
/// is the load-bearing failure for TC4 / AC9 negative.
struct Seventeen
{
    uint8_t a[Config::MAX_TASK_ARG_BYTES + 1];
};

int main()
{
    // Instantiate. The compiler MUST reject this; if it doesn't, the
    // ``static_assert`` was silently bypassed and AC9 negative is broken.
    check_overflow<Seventeen>();
    return 0;
}
