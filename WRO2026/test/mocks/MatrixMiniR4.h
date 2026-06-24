/**
 * @file MatrixMiniR4.h (HOST MOCK — forwarding shim)
 * @brief Forwards to the unified host-test MiniR4 facade in hal_mocks/.
 *
 * The full facade now lives in hal_mocks/MatrixMiniR4.h (the one shared mock-HAL
 * trust boundary, ADR-049). This shim keeps the legacy mocks/-path consumers
 * (manipulator / localization / yaw_source / reanchor) pointed at that single
 * definition — it is a strict superset of the old MockMatrixMiniR4, so those
 * targets compile unchanged. The forwarded header carries the real MATRIXMINIR4_H
 * guard, so do NOT add a second guard here.
 *
 * The relative include resolves source-relative to THIS file (mocks/), so the
 * legacy targets do NOT need hal_mocks/ on their include path.
 */
#include "../hal_mocks/MatrixMiniR4.h"
