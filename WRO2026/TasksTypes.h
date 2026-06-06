/**
 * @file TasksTypes.h
 * @brief Shared POD types for the Tasks framework (TaskArgs / TaskFn / TaskRegistryEntry).
 * @author WRO 2026
 *
 * @details
 * Thin dependency-free header included by both ``Mission.h`` and ``Tasks.h``.
 * Placing the POD layer here avoids a cyclic dependency: ``Mission.h`` needs the
 * arg-union type and the registry-entry type to declare the ``TASK`` step variant,
 * but it must NOT pull in the full ``TasksClass`` declaration. ``Tasks.h`` adds the
 * class on top of this header.
 *
 * @note ``Config::MAX_TASK_ARG_BYTES`` is defined in ``Config.h`` (added in the
 *       same PR as this file by Task T2). The ``static_assert`` below will fail to
 *       compile if T2 has not yet been committed; once T2 lands the constant is
 *       available and the assertion enforces the 16-byte POD contract.
 */

#ifndef TASKS_TYPES_H
#define TASKS_TYPES_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include "Config.h"    // Config::MAX_TASK_ARG_BYTES

class TasksClass;    // forward — full declaration lives in Tasks.h

/**
 * @brief Fixed-size POD for task argument storage.
 *
 * Operator-facing call sites use ``Mission.task<Args...>(name, args...)`` which
 * packs typed args via ``memcpy`` into ``TaskArgs::bytes``. Recommended layout:
 * define a per-task ``struct <Name>Args { ... };`` and pass a single struct value
 * so the byte layout matches the body's ``reinterpret_cast<const NameArgs*>``
 * unpack — mixing raw types (e.g. ``uint8_t`` + ``uint32_t``) is only safe when
 * the layout is byte-identical to a contiguous ``uint8_t[N]``.
 *
 * @warning **Type-punning gotcha.** Writing through ``.bytes`` (which is how
 *          ``MissionClass::_packArgs`` packs args via ``memcpy``) and then
 *          reading through ``.u16`` / ``.u32`` / ``.i32`` / ``.f32`` is
 *          strictly speaking undefined behaviour per ISO C++ [class.union]/4.
 *          GCC and clang document a relaxed extension that makes the read
 *          well-defined in practice, which is what the firmware target
 *          (avr-g++) and the host-compile tests (gcc/clang/MSVC) all rely on.
 *          If you ever cross-compile this code with a toolchain that does
 *          NOT honour the GCC union-aliasing extension, switch to
 *          ``memcpy(&dst_typed, &args.bytes[K], sizeof(dst_typed))`` reads.
 *          The unsigned-char view (``.u8`` / ``.bytes``) is always safe via
 *          the strict-aliasing exception for character types.
 *
 * @warning **Struct-padding gotcha.** When ``Mission.task("foo", FooArgs{...})``
 *          is called with a struct type that has padding (e.g.
 *          ``struct { uint8_t a; uint32_t b; }`` has 3 padding bytes between
 *          ``a`` and ``b``), the ``memcpy`` inside ``_packArgs`` copies the
 *          padding bytes verbatim. The receiver-side read of those padding
 *          offsets returns whatever was on the caller's stack at the time
 *          (indeterminate). Either (a) zero-init the struct at the call site
 *          (``FooArgs args{}; args.a = 1; ...``) or (b) use a packed struct
 *          (``__attribute__((packed))``) to eliminate the padding.
 *
 * @note 16 bytes (``Config::MAX_TASK_ARG_BYTES``) covers all foreseeable task
 *       signatures for Phase 5/6 (e.g. ``struct MosaicPlaceArgs { uint8_t cell;
 *       uint8_t color; uint16_t timeout_ms; }`` fits easily). Compile-time overflow
 *       is enforced by the ``static_assert`` in ``Mission.h``'s ``task<Args...>``
 *       template (AC9).
 */
union TaskArgs
{
    uint8_t  bytes[Config::MAX_TASK_ARG_BYTES];
    uint8_t  u8[Config::MAX_TASK_ARG_BYTES / sizeof(uint8_t)];
    uint16_t u16[Config::MAX_TASK_ARG_BYTES / sizeof(uint16_t)];
    uint32_t u32[Config::MAX_TASK_ARG_BYTES / sizeof(uint32_t)];
    int32_t  i32[Config::MAX_TASK_ARG_BYTES / sizeof(int32_t)];
    float    f32[Config::MAX_TASK_ARG_BYTES / sizeof(float)];
};

static_assert(sizeof(TaskArgs) == Config::MAX_TASK_ARG_BYTES, "TaskArgs layout");

// Cortex-M4 uses 4-byte alignment for the widest TaskArgs member (uint32_t /
// int32_t / float). A host compiler that aligned the union differently (e.g.
// 8-byte for double-ABI hosts) would still pass the size check above while
// silently giving the host-mirror tests a different memory layout than the
// on-target firmware — making the host mirror miss layout drift. The
// alignment assertion below binds the host tests to the M4 layout too.
static_assert(alignof(TaskArgs) == 4, "TaskArgs alignment must match Cortex-M4 (4)");

/**
 * @brief Pointer-to-member-function type for a task body.
 *
 * Bodies are non-static member functions on ``TasksClass`` so they share the
 * singleton's private helpers (e.g. future failure-flag access). A body receives
 * the packed argument union by const reference and returns ``true`` to signal
 * "advance to the next Mission step" or ``false`` to signal "yield" (Mission
 * advances without aborting the queue).
 */
using TaskFn = bool (TasksClass::*)(const TaskArgs&);

/**
 * @brief Compile-time registry row.
 *
 * The ``Tasks.cpp`` registry array is the single source of truth for valid task
 * names. ``Mission.task("name", ...)`` resolves the name against this table at
 * execute time; an unknown name triggers ``MissionClass::_enterFaultBeeping``.
 */
struct TaskRegistryEntry
{
    const char* name;
    TaskFn      fn;
};

#endif    // TASKS_TYPES_H
