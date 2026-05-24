/**
 * @file MatrixMiniR4.h (HOST MOCK)
 * @brief Empty shim — the real header pulls in hardware drivers we never call.
 *
 * `LineFollower.h` includes `<MatrixMiniR4.h>` for two reasons:
 *  1. To make `Config::SpeedPreset` resolution feel "native" (not actually
 *     needed — `Config.h` doesn't depend on this header).
 *  2. To compile, since `WROLineArray.h` (transitively included) declares a
 *     `MatrixColor _boards[2]` member.
 *
 * Our mock `WROLineArray.h` does NOT have `MatrixColor` members, so this header
 * can stay empty. It exists only to satisfy the `#include <MatrixMiniR4.h>`
 * directive in `LineFollower.h` and `Intersection.h`.
 */

#ifndef WRO_TEST_MOCK_MATRIXMINIR4_H
#define WRO_TEST_MOCK_MATRIXMINIR4_H

// Intentionally empty. The real library exposes the `MiniR4` global and the
// `MatrixColor` driver, neither of which is exercised by the test surface.

#endif    // WRO_TEST_MOCK_MATRIXMINIR4_H
