/**
 * @file EventTrace.h (HAL MOCK — hal_mocks/)
 * @brief Host-test mock of the EventTrace OLED ring buffer with ordered pushEvent capture.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `EventTrace.h` using the REAL include guard (`EVENTTRACE_H`) so the
 * production header no-ops if it is reached afterwards via a transitive include chain.
 * Force-include this file (via `/FI` or `-include`) in CMake targets that need it.
 *
 * **Fidelity — where this mock can lie**
 *
 * | Aspect                        | Fidelity level                                      |
 * |-------------------------------|-----------------------------------------------------|
 * | `MAX_LINE_LEN` / `LINE_HEIGHT_PX` | Exact match — production code sizes buffers from these |
 * | `pushEvent` truncation rule   | Exact match (≤ 19 usable chars + NUL, nullptr/empty ignored) |
 * | `pushEvent` ordering          | Exact match — monotonic FIFO capture ring           |
 * | `update()` edge-detection     | Not modelled — no OLED, Drive, or LineFollower here |
 * | OLED rendering                | Not modelled — host has no hardware                 |
 * | `Config::EVENT_TRACE_CAPACITY`| Not honoured — capture ring uses its own `MOCK_RING_CAPACITY` |
 * | Dirty-flag / heartbeat        | Not modelled — irrelevant on host                   |
 *
 * The capture ring is a fixed-size array (`MOCK_RING_CAPACITY` = 16 entries) that wraps on
 * overflow, matching the behaviour of the real ring but with a larger window that is more
 * useful for multi-event test assertions. `mockEvents()` returns a pointer to the flat ring
 * so tests can snapshot or iterate; `mockCount()` returns the number of entries captured
 * since the last `mockReset()`, clamped to `MOCK_RING_CAPACITY`.
 *
 * Does NOT `#include` the real `EventTrace.h` or `Config.h` — this is a standalone
 * re-declaration that compiles without Arduino or project headers.
 */

#ifndef EVENTTRACE_H
#define EVENTTRACE_H

#include <stdint.h>
#include <string.h>

/**
 * @brief Host-test mock of the OLED event-trace sink with ordered capture.
 *
 * Provides the same public surface as the production `EventTraceClass` plus a
 * `mock*` test API for inspecting captured events.
 */
class EventTraceClass
{
public:
    // -----------------------------------------------------------------------------------------
    // Constants that match the real production values exactly.
    // -----------------------------------------------------------------------------------------

    /** @brief Width budget per OLED line in characters, including the trailing NUL (matches real). */
    static constexpr uint8_t MAX_LINE_LEN = 20;

    /** @brief Pixel height of one text-size-1 line on the Adafruit GFX font (matches real). */
    static constexpr uint8_t LINE_HEIGHT_PX = 8;

    // -----------------------------------------------------------------------------------------
    // Mock-only constant.
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Capture ring capacity for the host mock.
     *
     * Deliberately larger than `Config::EVENT_TRACE_CAPACITY` (4) so multi-step test
     * scenarios can capture a full sequence without evicting earlier events mid-assertion.
     * Must be a power of two for the wrap mask to work correctly.
     */
    static constexpr uint8_t MOCK_RING_CAPACITY = 16;

    // -----------------------------------------------------------------------------------------
    // Production surface (called by Mission.cpp, Tasks.cpp, etc.)
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Initialise the mock: clear the capture ring and reset counters.
     *
     * @return Always `true` — mirroring the production signature that reserves the right to
     *         return `false` on a future hardware failure path.
     */
    bool begin()
    {
        mockReset();
        return true;
    }

    /**
     * @brief No-op cooperative tick.
     *
     * The production implementation polls Drive/LineFollower for edge-triggered events and
     * repaints the OLED. Neither subsystem nor OLED exists on the host, so this is empty.
     */
    void update() {}

    /**
     * @brief Capture an event string into the ordered mock ring.
     *
     * @param msg NUL-terminated C string. `nullptr` and the empty string (`""`) are silently
     *            ignored, matching the real contract. Strings longer than `MAX_LINE_LEN - 1`
     *            characters are truncated to exactly 19 usable characters and NUL-terminated.
     *
     * Appends `msg` to the capture ring at `_head`, then advances `_head` with wrap. If the
     * ring is already at `MOCK_RING_CAPACITY` entries, the oldest entry is overwritten (same
     * policy as the production ring). `_count` is clamped to `MOCK_RING_CAPACITY`.
     */
    void pushEvent(const char* msg)
    {
        if (msg == nullptr || msg[0] == '\0')
        {
            return;
        }

        // Truncate to MAX_LINE_LEN - 1 usable characters, then NUL-terminate.
        strncpy(_events[_head], msg, MAX_LINE_LEN - 1);
        _events[_head][MAX_LINE_LEN - 1] = '\0';

        _head = static_cast<uint8_t>((_head + 1) & (MOCK_RING_CAPACITY - 1));

        if (_count < MOCK_RING_CAPACITY)
        {
            ++_count;
        }
    }

    /**
     * @brief Clear the production-visible event state.
     *
     * Delegates to `mockReset()` so both the production call sites and test teardown reach
     * the same clearing path.
     */
    void clear()
    {
        mockReset();
    }

    // -----------------------------------------------------------------------------------------
    // Test surface (used only from test files, never from production code).
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Return a pointer to the raw ordered capture ring.
     *
     * @return Pointer to `char[MOCK_RING_CAPACITY][MAX_LINE_LEN]` — the flat ring storage.
     *         The oldest entry still in the ring is at index
     *         `(_head - _count + MOCK_RING_CAPACITY) & (MOCK_RING_CAPACITY - 1)`.
     *         Iterate `_count` entries forward from that index (with wrap) for FIFO order.
     *
     * @note The pointer is valid until `mockReset()` or the object is destroyed. Tests that
     *       need a stable snapshot should copy the strings before calling any method that
     *       modifies the ring.
     */
    const char (*mockEvents() const)[MAX_LINE_LEN]
    {
        return _events;
    }

    /**
     * @brief Return the number of events captured since the last `mockReset()`.
     *
     * @return Value in `[0, MOCK_RING_CAPACITY]`. When the ring has wrapped, this equals
     *         `MOCK_RING_CAPACITY` and the oldest event has been overwritten.
     */
    uint8_t mockCount() const
    {
        return _count;
    }

    /**
     * @brief Return the Nth captured event string in FIFO order (0 = oldest still in ring).
     *
     * @param index Zero-based index. Must be less than `mockCount()`. Passing an out-of-range
     *              index returns an empty string (`""`).
     * @return NUL-terminated C string, truncated to at most `MAX_LINE_LEN - 1` characters.
     */
    const char* mockEventAt(uint8_t index) const
    {
        if (index >= _count)
        {
            return "";
        }
        uint8_t oldest =
            static_cast<uint8_t>((_head - _count + MOCK_RING_CAPACITY) & (MOCK_RING_CAPACITY - 1));
        uint8_t slot = static_cast<uint8_t>((oldest + index) & (MOCK_RING_CAPACITY - 1));
        return _events[slot];
    }

    /**
     * @brief Reset all capture state: zero the ring, set head and count to zero.
     *
     * Call this in test `setUp` / `TearDown` to start each test with a clean slate.
     * Also called by `begin()` and `clear()` so the production-facing methods reach the
     * same reset path.
     */
    void mockReset()
    {
        memset(_events, 0, sizeof(_events));
        _head  = 0;
        _count = 0;
    }

private:
    /**
     * @brief Fixed-size ordered capture ring.
     *
     * `_events[slot]` holds a NUL-terminated string of at most `MAX_LINE_LEN - 1` chars.
     * Slot layout: newest entry is at `_events[(_head - 1) & (MOCK_RING_CAPACITY - 1)]`;
     * oldest entry still in the ring is at
     * `_events[(_head - _count + MOCK_RING_CAPACITY) & (MOCK_RING_CAPACITY - 1)]`.
     */
    char _events[MOCK_RING_CAPACITY][MAX_LINE_LEN];

    /** @brief Next write index into `_events` (0..MOCK_RING_CAPACITY-1). */
    uint8_t _head = 0;

    /** @brief Number of valid entries captured since the last `mockReset()` (0..MOCK_RING_CAPACITY). */
    uint8_t _count = 0;
};

/**
 * @brief Single global instance — defined in the test translation unit that includes this header.
 */
extern EventTraceClass EventTrace;

#endif    // EVENTTRACE_H
