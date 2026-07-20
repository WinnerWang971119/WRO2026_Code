/**
 * @file Vision.cpp
 * @brief Out-of-line members of `VisionClass` — the mosaic-colour-identification facade.
 * @author WRO 2026
 *
 * @details
 * `readMosaic()` is the only non-trivial member. It owns the command-time BLOCKING UART read
 * (ADR-003 / ADR-037 sanctioned blocking site) and follows a strict fresh-frame discipline:
 * flush the streamed backlog, discard `Config::VISION_FLUSH_DISCARD_FRAMES` post-flush frames,
 * then latch the next one. The wait loop mirrors `Drive::_drainBlocking()` exactly — every
 * iteration calls the registered cooperative-tick callback (so the OLED / heartbeat / other
 * modules stay alive) and then `delay(small)`. The `delay()` is LOAD-BEARING: on the host
 * mock `millis()` advances only inside `delay()`, so a tight spin would hang the timeout tests
 * forever and on hardware would starve every cooperative module for up to the read timeout.
 *
 * The HUE12 begins emitting frames once it is kicked with the wake command; `begin()` sends it
 * via `start()` at boot, and `start()` is callable on demand if the camera needs re-kicking
 * after a long idle. Once awake the camera streams continuously, so `readMosaic()` flushes the
 * streamed backlog at the start of every attempt before latching a fresh frame.
 *
 * Parsing, unpacking, and classification are NOT reimplemented here — they live in the pure,
 * host-testable free functions in `VisionFrame.h` (ADR-030 host-link test seam). This file is
 * the hardware-facing UART glue only. `readMosaic()` is decomposed into a few private helpers
 * (`_flushRx`, `_waitTick`, `_classifyCandidate`) for readability; `Vision.h` declares the
 * public surface, those helpers, and the latch members.
 *
 * The vendor `MiniR4SmartCamReader` / `MiniR4.Vision` are deliberately NOT used (its parser
 * runs off the end of a non-void function on a misaligned start, overruns a fixed stack buffer
 * on a corrupt length byte, and false-syncs on a `0xAA`-as-data byte — see the plan). We talk
 * to the camera UART directly via `Config::VISION_SERIAL` (see ConfigVision.h — it resolves
 * to `Serial2` on the R4 WiFi, the D0/D1 header socket; `MiniR4.Uart` is the same port).
 * We do NOT `#include <MatrixMiniR4.h>` from here.
 */

#include "Vision.h"

#include "VisionFrame.h"

#include <Arduino.h>    // VISION_SERIAL (HUE12 UART RX), Serial (USB warning), millis(), delay()

/// @brief Single global instance declared `extern` in Vision.h.
VisionClass Vision;

// -----------------------------------------------------------------------------------------
// Compile-time invariants (defence-in-depth over ConfigChecks.h; these are the exact
// relationships readMosaic() relies on locally, so keep them adjacent to the code).
// -----------------------------------------------------------------------------------------

static_assert(Config::VISION_CELL_COUNT > 0, "VISION_CELL_COUNT must be positive");
static_assert(Config::VISION_FRAME_WORDS > 0, "VISION_FRAME_WORDS must be positive");

// -----------------------------------------------------------------------------------------
// File-local constants — no magic numbers. Frame size, discard count, timeout, baud, and the
// cell/slot counts all come from Config / VisionFrame; only the small wait granularity and the
// retry count are local to the read algorithm.
// -----------------------------------------------------------------------------------------

namespace
{
    /// @brief Wait-loop sleep granularity in ms. The ONE inline timing literal allowed by the
    ///        plan — small enough to keep the cooperative tick responsive, large enough that the
    ///        bounded wait makes monotonic `millis()` progress on host (the mock clock advances
    ///        inside `delay()`). Mirrors `Drive::_drainBlocking`'s `delay(2)`.
    constexpr uint32_t kWaitGranularityMs = 2u;

    /// @brief Maximum read attempts per `readMosaic()` call: one initial + one retry. A
    ///        READ_TIMEOUT or BAD_CHECKSUM on the first attempt retries once (AC6); a second
    ///        failure surfaces the fault. BAD_CELL never reaches this loop bound — it returns
    ///        immediately (a static scene re-read would only reproduce the same UNKNOWN cell).
    /// @note  Because of this single retry, the WORST-CASE blocking time of `readMosaic()` is the
    ///        full retry budget — `kMaxAttempts * Config::VISION_READ_TIMEOUT_MS` (2 × 120 ms =
    ///        240 ms at today's config), not one `VISION_READ_TIMEOUT_MS`. Each attempt re-arms
    ///        its own per-attempt deadline, so two consecutive transport timeouts stack.
    constexpr uint8_t kMaxAttempts = 2u;

    /// @brief Full HUE12 frame size in bytes: header + length + 2 bytes/word + checksum.
    ///        Computed from Config so it tracks `VISION_FRAME_WORDS` — never hardcode 21.
    ///        Used to size the read buffer AND to disambiguate READ_TIMEOUT vs BAD_CHECKSUM.
    constexpr size_t kFrameBytes =
        1u /* header */ + 1u /* length */ + 2u * static_cast<size_t>(Config::VISION_FRAME_WORDS)
        + 1u /* checksum */;

    /// @brief How many full frames the local read buffer can hold before it must compact.
    ///        Sized for a few frames so a valid frame that straddles two read bursts is never
    ///        truncated, yet bounded so an unending stream can never overflow it (we slide off
    ///        consumed/leading bytes once it fills — see the compaction logic in `readMosaic`).
    constexpr size_t kBufferFrames = 4u;

    /// @brief Fixed read-buffer capacity in bytes. A never-ending stream cannot overflow this:
    ///        when it fills without yielding a parse, the oldest bytes are dropped so at least
    ///        `kFrameBytes - 1` trailing bytes are retained to complete a straddling frame.
    constexpr size_t kBufferCapacity = kBufferFrames * kFrameBytes;
}    // namespace

// The compaction slide in readMosaic() keeps the last (kFrameBytes - 1) trailing bytes when the
// buffer fills, which is only safe if the buffer can hold at least one whole frame. Guard that
// invariant at compile time so a future shrink of kBufferFrames cannot silently break it. Placed
// after the namespace so both named constants are in scope.
static_assert(kFrameBytes <= kBufferCapacity, "RX buffer must hold at least one full frame");

// -----------------------------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------------------------

VisionClass::VisionClass()
{
    // Latch starts in the documented safe pre-read state: every cell UNKNOWN, every raw hue 0,
    // no successful read yet, no fault. Getters are therefore safe to call before begin() /
    // before the first successful readMosaic(). No hardware work here — the UART is opened in
    // begin(), which must run after MiniR4.begin() has returned.
    for (uint8_t c = 0; c < Config::VISION_CELL_COUNT; ++c)
    {
        _colors[c] = Config::Color::UNKNOWN;
        _rawHue[c] = 0;
    }
    _hasRead            = false;
    _fault.kind         = Fault::NONE;
    _fault.firstBadCell = 0xFF;
    _fault.rawValue     = 0;
    _tickCallback       = nullptr;
}

// -----------------------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------------------

bool VisionClass::begin()
{
    // Open the HUE12 UART. Baud comes from Config (bench-verified at T8) — never hardcode 9600.
    VISION_SERIAL.begin(Config::VISION_UART_BAUD);

    // Reset the latch to its safe pre-read state. Re-running begin() must not leave a stale read
    // visible, so we clear `_hasRead` and reset every cell exactly as the constructor does.
    for (uint8_t c = 0; c < Config::VISION_CELL_COUNT; ++c)
    {
        _colors[c] = Config::Color::UNKNOWN;
        _rawHue[c] = 0;
    }
    _hasRead = false;

    // Clear any latched fault — the module boots ready for the first readMosaic().
    _fault.kind         = Fault::NONE;
    _fault.firstBadCell = 0xFF;
    _fault.rawValue     = 0;

    // We do NOT flush pending UART bytes here (the device may already be streaming) — readMosaic()
    // owns its own flush-and-discard sequence. See the @note on begin() in Vision.h.

    // Kick the HUE12 so a cold-booted camera begins emitting frames (vendor SmartCamReader
    // contract). Done LAST so the UART is already open before the wake command is written.
    start();

    // Always true on this platform; declared `bool` for parity with Drive/Manipulator begin() so
    // the call site can guard on the return value if the platform ever changes.
    return true;
}

// -----------------------------------------------------------------------------------------
// Command — wake the camera
// -----------------------------------------------------------------------------------------

void VisionClass::start()
{
    // Pure TX: write the wake command to the HUE12. `VISION_SERIAL.print(const char*)` works on the
    // real HardwareSerial and on the host mock (a no-op there). No read, flush, or latch mutation.
    VISION_SERIAL.print(Config::VISION_START_CMD);
}

// -----------------------------------------------------------------------------------------
// Read-loop helpers — decomposed out of readMosaic() for readability (behaviour identical).
// -----------------------------------------------------------------------------------------

void VisionClass::_flushRx()
{
    // Step 1 — Flush the streamed backlog (AC7). The HUE12 streams continuously, so at the stop the
    // RX buffer holds frames captured WHILE MOVING. Draining them here guarantees the latched frame
    // begins after the flush, so it reflects the robot's current field position. (The caller must
    // additionally ensure Drive.isIdle() — that contract lives on Vision.h's @note.)
    while (VISION_SERIAL.available() > 0)
    {
        VISION_SERIAL.read();
    }
}

void VisionClass::_waitTick()
{
    // Wait-loop step (LOAD-BEARING). No bytes available yet: tick every other module, then sleep
    // one granularity. NEVER spin without delay() — on host the mock clock only advances inside
    // delay() (a tight spin hangs the timeout tests forever), and on hardware the tick re-entry is
    // what keeps the OLED / heartbeat alive. Mirrors Drive::_drainBlocking.
    if (_tickCallback != nullptr)
    {
        _tickCallback();
    }
    else
    {
        // No callback registered: still perform a bare bounded wait, but warn ONCE so the operator
        // knows other modules freeze during the read. Static-bool guard matches the
        // Drive/Manipulator "no tick registered" convention.
        static bool noCallbackWarned = false;
        if (!noCallbackWarned)
        {
            Serial.println(F("[Vision] readMosaic: no tick callback set — only the UART "
                              "poll runs during the read. Call "
                              "Vision.setTickCallback(...) in setup()."));
            noCallbackWarned = true;
        }
    }
    delay(kWaitGranularityMs);
}

bool VisionClass::_classifyCandidate(
    const uint16_t words[Config::VISION_FRAME_WORDS],
    Config::Color  outColors[Config::VISION_CELL_COUNT],
    uint8_t        outHue[Config::VISION_CELL_COUNT],
    uint8_t&       firstBadCell,
    uint8_t&       badRaw) const
{
    // Unpack into per-SLOT hue / sta arrays (device order S1..S12).
    uint8_t hue[Config::VISION_CELL_COUNT];
    uint8_t sta[Config::VISION_CELL_COUNT];
    VisionFrame::unpackFrame(words, hue, sta);

    // Classify into CELL-indexed local buffers, applying the strict guard, so the caller touches
    // the latch ONLY after every active cell validates. This is what makes AC4's "no overwrite on
    // failure" hold (the caller commits the latch only on a true return).
    for (uint8_t c = 0; c < Config::VISION_CELL_COUNT; ++c)
    {
        // DIRECTION MATTERS (TC19): the unpacked arrays are indexed BY SLOT, the result is stored
        // BY CELL. `VISION_CELL_TO_SLOT[c]` is the HUE12 slot that observes cell `c`. A transposed
        // map (indexing by cell) is exactly what TC19 catches.
        const uint8_t       slot = Config::VISION_CELL_TO_SLOT[c];
        const Config::Color col  = VisionFrame::classifyHue(hue[slot], sta[slot]);

        if (col == Config::Color::UNKNOWN)
        {
            // Strict guard (AC4): any active cell that fails to classify aborts the whole read.
            // Record the first offending CELL index and the raw hue of its mapped slot. The caller
            // does NOT write the latch on a false return.
            firstBadCell = c;
            badRaw       = hue[slot];
            return false;
        }

        outColors[c] = col;
        outHue[c]    = hue[slot];
    }

    // Every active cell validated.
    return true;
}

// -----------------------------------------------------------------------------------------
// Command — the core blocking read
//
// Structure:
//   - Outer retry-once loop over READ_TIMEOUT / BAD_CHECKSUM (AC6).
//   - Per attempt: flush (AC7) → accumulate into a bounded buffer, ticking + delay()ing while
//     waiting (AC1) → parse frames, discard VISION_FLUSH_DISCARD_FRAMES, latch the next (AC7) →
//     unpack + cell→slot map + classify + strict guard (AC1, AC4).
//   - BAD_CELL returns immediately (no retry, no latch overwrite). A success returns true with
//     the latch committed. A transport fault retries once, then latches the fault and returns
//     false without touching the colour/hue latch.
//
// The byte-accumulation / parse drain stays inline (it reads as one cohesive loop); the flush,
// the wait-tick, and the candidate classify+guard are factored into private helpers.
// -----------------------------------------------------------------------------------------

bool VisionClass::readMosaic()
{
    for (uint8_t attempt = 0; attempt < kMaxAttempts; ++attempt)
    {
        // ---------------------------------------------------------------------------------
        // Step 1 — Flush the streamed backlog (AC7), so the latched frame begins strictly after
        // the flush and reflects the robot's current field position.
        // ---------------------------------------------------------------------------------
        _flushRx();

        // ---------------------------------------------------------------------------------
        // Step 2 — Accumulate bytes into a FIXED, overflow-proof local buffer and parse frames
        // out of it as they complete. The first VISION_FLUSH_DISCARD_FRAMES valid frames are
        // counted and dropped; the next valid frame is the latch candidate.
        // ---------------------------------------------------------------------------------
        uint8_t buf[kBufferCapacity];
        size_t  len          = 0;       // valid bytes currently in buf
        size_t  framesParsed = 0;       // valid frames seen so far this attempt (incl. discarded)
        uint8_t lastByteRead = 0;       // most recent RX byte — reported on BAD_CHECKSUM
        bool    anyByteRead  = false;   // whether ANY byte was read this attempt

        // totalBytesSeen is a MONOTONIC count of every byte read this attempt. It is NOT buf
        // `len` (which shrinks on compaction), so it is the stable basis for the
        // timeout-vs-checksum disambiguation at the deadline.
        size_t totalBytesSeen = 0;

        // Outcome flags for this attempt, resolved below the read loop.
        bool    sawCandidate = false;   // a latch candidate was processed (success OR bad-cell)
        bool    latched      = false;   // the candidate validated and the latch was committed
        bool    badCell      = false;   // the candidate tripped the strict guard
        uint8_t badCellIndex = 0xFF;    // first offending CELL index (BAD_CELL only)
        uint8_t badCellRaw   = 0;       // raw hue of the offending cell's mapped slot (BAD_CELL)

        // Per-attempt deadline. `millis()` is unsigned; the `(int32_t)(now - deadline) >= 0`
        // form is wrap-safe, matching Drive::_drainBlocking and Manipulator::_drain.
        const uint32_t deadline =
            millis() + static_cast<uint32_t>(Config::VISION_READ_TIMEOUT_MS);

        while (!sawCandidate)
        {
            // Deadline check FIRST so an attempt that never accumulates a frame still terminates.
            if (static_cast<int32_t>(millis() - deadline) >= 0)
            {
                break;
            }

            if (VISION_SERIAL.available() > 0)
            {
                // Set when read() returns < 0 despite available() > 0 (the queue and the read
                // disagree). We then fall through to the wait branch so a delay() (+ tick) runs
                // before re-polling — never a tight spin that would hang the host mock clock.
                bool readUnderrun = false;

                // Drain everything currently readable into the buffer, compacting as needed so a
                // never-ending stream cannot overflow `buf`.
                while (VISION_SERIAL.available() > 0)
                {
                    if (len >= kBufferCapacity)
                    {
                        // Buffer full without a parse: slide off the leading bytes, keeping the
                        // last (kFrameBytes - 1) so a frame straddling the boundary is not
                        // truncated. Any genuine frame fits in kFrameBytes <= kBufferCapacity, so
                        // this never discards a completable frame.
                        const size_t keep = kFrameBytes - 1u;
                        const size_t drop = len - keep;
                        for (size_t i = 0; i < keep; ++i)
                        {
                            buf[i] = buf[drop + i];
                        }
                        len = keep;
                    }

                    const int b = VISION_SERIAL.read();
                    if (b < 0)
                    {
                        // available() and read() disagreed (cannot happen on this mock/hardware,
                        // but guard rather than store -1 as 0xFF). Treat it like "no byte this
                        // iteration": stop draining and fall through to the wait branch so a
                        // delay() (+ tick) runs before re-polling, instead of looping straight
                        // back into the available() > 0 branch and tight-spinning.
                        readUnderrun = true;
                        break;
                    }
                    buf[len++]   = static_cast<uint8_t>(b);
                    lastByteRead = static_cast<uint8_t>(b);
                    anyByteRead  = true;
                    ++totalBytesSeen;
                }

                // Try to extract every complete valid frame currently in the buffer. Each accepted
                // frame is counted; the buffer is compacted by sliding off the consumed prefix so
                // the next parse starts at the byte after the accepted frame.
                uint16_t words[Config::VISION_FRAME_WORDS];
                size_t   consumed = 0;
                while (!sawCandidate && VisionFrame::parseFrame(buf, len, words, consumed))
                {
                    ++framesParsed;

                    if (framesParsed > Config::VISION_FLUSH_DISCARD_FRAMES)
                    {
                        // -------------------------------------------------------------------
                        // Step 3 — Latch-candidate processing. The (DISCARD+1)-th valid frame
                        // began strictly after the flush, so it is the fresh frame to latch
                        // (AC7). Map + classify + strict-guard into LOCAL arrays; the latch is
                        // committed below ONLY on a full pass, so AC4's "no overwrite on failure"
                        // is visible right here at the call site.
                        // -------------------------------------------------------------------
                        sawCandidate = true;

                        Config::Color cellColors[Config::VISION_CELL_COUNT];
                        uint8_t       cellHue[Config::VISION_CELL_COUNT];

                        const bool allValid =
                            _classifyCandidate(words, cellColors, cellHue, badCellIndex, badCellRaw);

                        if (allValid)
                        {
                            // Every active cell validated — commit the latch (past the guard, so no
                            // partial write is possible) and mark the module read.
                            for (uint8_t c = 0; c < Config::VISION_CELL_COUNT; ++c)
                            {
                                _colors[c] = cellColors[c];
                                _rawHue[c] = cellHue[c];
                            }
                            _hasRead = true;
                            latched  = true;
                        }
                        else
                        {
                            // Strict-guard failure — `_classifyCandidate` recorded the first bad
                            // CELL index and its raw mapped-slot hue. Flag it for outcome resolution
                            // below; the latch was NOT touched.
                            badCell = true;
                        }
                        break;    // candidate handled — leave the parse loop
                    }

                    // Discarded frame — compact and keep scanning for the next complete frame.
                    if (consumed >= len)
                    {
                        len = 0;
                    }
                    else
                    {
                        const size_t remaining = len - consumed;
                        for (size_t i = 0; i < remaining; ++i)
                        {
                            buf[i] = buf[consumed + i];
                        }
                        len = remaining;
                    }
                }

                // No (further) complete frame in the buffer right now (or the candidate was just
                // handled). Loop condition re-checks `sawCandidate`; if not set we may have more
                // bytes already available (drain again) or we wait below for the next batch.
                // On a read() underrun, DON'T continue — fall through to the wait branch so a
                // delay() (+ tick) runs before re-polling, avoiding a tight spin.
                if (!readUnderrun)
                {
                    continue;
                }
            }

            // -----------------------------------------------------------------------------
            // Wait-loop step (LOAD-BEARING). No bytes available yet: tick every other module (or
            // warn once if no callback is set), then sleep one granularity inside `_waitTick()`.
            // The `delay()` it performs is what advances `millis()` on host and keeps cooperative
            // modules alive on hardware.
            // -----------------------------------------------------------------------------
            _waitTick();
        }

        // ---------------------------------------------------------------------------------
        // Step 4 — Resolve this attempt's outcome.
        // ---------------------------------------------------------------------------------
        if (latched)
        {
            // Success (AC1). Clear any residual fault so a recovered read presents cleanly.
            _fault.kind         = Fault::NONE;
            _fault.firstBadCell = 0xFF;
            _fault.rawValue     = 0;
            return true;
        }

        if (badCell)
        {
            // Strict-guard failure (AC4). The scene is static at a stop, so a re-read would return
            // the same UNKNOWN cell — do NOT retry. The latch was NOT modified above, so the prior
            // successful read (or the safe pre-read defaults) remains visible.
            _fault.kind         = Fault::BAD_CELL;
            _fault.firstBadCell = badCellIndex;
            _fault.rawValue     = badCellRaw;
            return false;
        }

        // ---------------------------------------------------------------------------------
        // Step 5 — Deadline hit without a latch candidate. Disambiguate the transport fault with
        // a single deterministic rule (T7's TC15 / TC16 are written against EXACTLY this):
        //
        //   * BAD_CHECKSUM is reserved for the case where bytes flowed but NOTHING ever
        //     validated: at least one full frame's worth of bytes was accumulated this attempt
        //     (totalBytesSeen >= kFrameBytes) yet parseFrame never accepted a single frame
        //     (framesParsed == 0), so every byte run failed the header/length/XOR validation
        //     →  BAD_CHECKSUM (TC15: a corrupt frame-sized blob first, a good frame on the retry).
        //
        //   * Otherwise it is a READ_TIMEOUT. This now also covers the slow-but-healthy device:
        //     if parseFrame DID accept at least one frame this attempt (framesParsed > 0, e.g. the
        //     discarded post-flush frames) the stream is proven to be flowing AND validating, so a
        //     deadline miss before the latch candidate completed is a too-short window vs the
        //     emission rate — a timeout, not a checksum failure. The starved case (a drip-feed
        //     that never completes one frame, totalBytesSeen < kFrameBytes, framesParsed == 0)
        //     also lands here (TC16: persistent timeout; the loop still terminates because delay()
        //     advances millis()).
        //
        // The byte boundary is `>= kFrameBytes` measured on the MONOTONIC byte counter, NOT on the
        // current buffer length — compaction shrinks `len`, so only the cumulative count is a
        // stable signal. framesParsed is reset to 0 at the top of each attempt, so a prior
        // attempt's accepted-frame count can never leak into this decision.
        // ---------------------------------------------------------------------------------
        const bool isChecksumFail =
            (anyByteRead && totalBytesSeen >= kFrameBytes && framesParsed == 0u);

        if (attempt + 1u < kMaxAttempts)
        {
            // Transport fault on a non-final attempt — retry once (AC6). The next iteration
            // re-flushes and re-reads from scratch.
            continue;
        }

        // Second (final) transport failure — surface the matching fault (AC6). The latch is NOT
        // updated (per ADR-037: a false return is authoritative; stale latch contents do not
        // become current).
        if (isChecksumFail)
        {
            _fault.kind         = Fault::BAD_CHECKSUM;
            _fault.firstBadCell = 0xFF;
            _fault.rawValue     = lastByteRead;    // last raw byte — diagnostic for the failure
        }
        else
        {
            _fault.kind         = Fault::READ_TIMEOUT;
            _fault.firstBadCell = 0xFF;
            _fault.rawValue     = 0;    // starved — no meaningful raw byte to report
        }
        return false;
    }

    // Unreachable — the loop always returns on the final attempt. Present so a future change to
    // kMaxAttempts cannot fall off the end of this non-void function.
    _fault.kind         = Fault::READ_TIMEOUT;
    _fault.firstBadCell = 0xFF;
    _fault.rawValue     = 0;
    return false;
}

// -----------------------------------------------------------------------------------------
// Getters (pure, bounds-checked, safe-sentinel — AC3)
// -----------------------------------------------------------------------------------------

Config::Color VisionClass::getColor(uint8_t cell) const
{
    // Out-of-range index OR no successful read yet → the documented safe sentinel. Never indexes
    // out of bounds (ADR-028). Pure: no I/O, no fault mutation.
    if (cell >= Config::VISION_CELL_COUNT || !_hasRead)
    {
        return Config::Color::UNKNOWN;
    }
    return _colors[cell];
}

uint8_t VisionClass::getRawHue(uint8_t cell) const
{
    // Same guard as getColor() — sentinel is the named Config::VISION_RAW_HUE_NONE (0xFF) for
    // the raw-hue axis (ADR-028), never a bare literal. It can never collide with a genuinely
    // latched hue: classifyHue() maps a raw 255 (device "no detection") to Color::UNKNOWN, so
    // any candidate frame containing it fails _classifyCandidate's strict guard and is rejected
    // before _rawHue is ever committed.
    if (cell >= Config::VISION_CELL_COUNT || !_hasRead)
    {
        return Config::VISION_RAW_HUE_NONE;
    }
    return _rawHue[cell];
}

// -----------------------------------------------------------------------------------------
// Fault surface (mirrors Drive / PathTracker)
// -----------------------------------------------------------------------------------------

bool VisionClass::hasFault() const
{
    return _fault.kind != Fault::NONE;
}

VisionClass::FaultInfo VisionClass::getFault() const
{
    return _fault;
}

void VisionClass::clearFault()
{
    // Reset ONLY the fault record (kind / firstBadCell / rawValue) to its zero-defaults. The
    // colour / hue latch is deliberately untouched so the last successful read stays accessible
    // after clearing a fault (see the @note on clearFault() in Vision.h).
    _fault.kind         = Fault::NONE;
    _fault.firstBadCell = 0xFF;
    _fault.rawValue     = 0;
}

// -----------------------------------------------------------------------------------------
// Callback
// -----------------------------------------------------------------------------------------

void VisionClass::setTickCallback(void (*cb)())
{
    _tickCallback = cb;
}
