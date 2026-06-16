/**
 * @file vision_bench.ino
 * @brief Standalone HUE12 bring-up / diagnostic sketch (NOT flight code).
 *
 * @details
 * This is a bench tool the operator flashes to the real R4 to bring up the
 * HUE12 UART camera before the firmware `Vision` module is trusted. It does ONE
 * job: pull bytes off the camera's UART, decode them with the SAME parser the
 * firmware uses (`VisionFrame::parseFrame` / `VisionFrame::unpackFrame`), and
 * print every slot's raw hue/sta plus a measured frame rate over the USB serial
 * monitor. From those numbers the operator fills in the three placeholder
 * `ConfigVision.h` constants that cannot be known without hardware.
 *
 * It deliberately reuses `VisionFrame::` rather than reimplementing the frame
 * contract: the wire format, the XOR checksum, the little-endian word packing,
 * and the hue/sta nibble split must have exactly ONE definition in the repo
 * (ADR-030). A second copy here would drift and turn this bench tool into a
 * source of false readings. So this sketch links the real header.
 *
 * ---------------------------------------------------------------------------
 * ON-HARDWARE BRING-UP PROCEDURE
 * ---------------------------------------------------------------------------
 *
 * (a) CONFIRM BAUD AND MEASURE THE FRAME EMISSION RATE (Hz).
 *     Power the HUE12, wire its TX/RX to the R4's D0/D1 UART socket, flash this
 *     sketch, and open the USB serial monitor at 115200 baud. Each second the
 *     sketch prints `fps=<N>` — the number of complete, checksum-valid frames
 *     decoded in the last ~1 s window. That N is the camera's EMISSION RATE.
 *       - If `fps` is a stable non-zero number, the configured UART baud
 *         (`Config::VISION_UART_BAUD`, currently 9600) is correct.
 *       - If `fps` is 0 or you see only `(no frames this second)`, the baud is
 *         likely wrong (or see note (d) — check the physical socket FIRST).
 *     The measured rate feeds `Config::VISION_READ_TIMEOUT_MS`: that timeout
 *     MUST be at least `(Config::VISION_FLUSH_DISCARD_FRAMES + 1)` emission
 *     periods so a single blocking read can flush, discard the stale frames,
 *     and still capture one fresh frame. With the placeholder
 *     `VISION_FLUSH_DISCARD_FRAMES = 1`, that is >= 2 frame periods; e.g. at a
 *     measured 45 fps (~22 ms/frame) the timeout must be >= ~44 ms — the
 *     placeholder 120 ms is comfortably safe, but VERIFY against the real rate.
 *
 * (b) MAP EACH GAME-MOSAIC CELL TO ITS HUE12 SLOT INDEX.
 *     `Config::VISION_CELL_TO_SLOT` is an identity placeholder `{0,1,...,11}`.
 *     The real mapping is taught here: cover all cells, then expose exactly ONE
 *     known mosaic cell to the camera at a time and watch which `slot <i>` line
 *     changes its hue away from the NONE code. That slot index `i` is the value
 *     `VISION_CELL_TO_SLOT[cell]` for that game cell. Record the slot index for
 *     all `Config::VISION_CELL_COUNT` cells and transcribe them into the array.
 *     NOTE: this sketch prints by RAW UNPACKED SLOT INDEX (the hue/sta array
 *     position 0..VISION_CELL_COUNT-1), NOT by game cell — that is exactly the
 *     identifier you need to populate `VISION_CELL_TO_SLOT`.
 *
 * (c) READ RAW HUES OF KNOWN-COLOR TILES TO CALIBRATE `Config::HUE_BANDS`.
 *     `HUE_BANDS` ships with placeholder ranges. Place a known Yellow / Green /
 *     Blue game tile under a slot, read its steady `hue=<rawHue>` from that
 *     slot's line, and repeat across several samples and lighting conditions.
 *     Set each band's `lo`/`hi` to bracket the observed chromatic hues (the
 *     hue axis is 0..239 chromatic; 240..245 is device White, 246..250 device
 *     Black, 255 device NONE — those are handled in `classifyHue`, NOT in
 *     `HUE_BANDS`). Also sanity-check `Config::VISION_WHITE_STA_MAX` against the
 *     `sta=<rawSta>` of a desaturated/white tile (sta is a 4-bit value, 0..15).
 *
 * (d) IF YOU GET NO BYTES, SUSPECT THE WIRING BEFORE THE BAUD.
 *     On the MatrixMiniR4, `MiniR4.Uart` IS `Serial1` (MatrixMiniR4.h:114 —
 *     `UART Uart = UART(UART2_TX_PIN, UART2_RX_PIN)`). This sketch talks to
 *     `Serial1` directly. If `fps` stays 0 and no `slot` table ever prints,
 *     check the physical D0/D1 socket and the TX<->RX cross-wiring (camera TX
 *     must reach R4 RX) BEFORE assuming a baud mismatch. A dead socket and a
 *     wrong baud look identical from the serial monitor. If the wiring and baud
 *     check out but `fps` still stays 0, the camera may need the start command
 *     to begin emitting — this bench now sends `Config::VISION_START_CMD` once at
 *     setup(), matching the firmware, so a cold-boot-idle camera is kicked awake.
 *
 * ---------------------------------------------------------------------------
 * HOW TO COMPILE / FLASH
 * ---------------------------------------------------------------------------
 *
 * The frame contract lives in the sibling `WRO2026/` sketch dir. Per ADR-030
 * we do NOT copy `VisionFrame.h` / `ConfigVision.h` into this folder — the
 * bench links the real sources. Reach them with an include-path build flag:
 *
 *   arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
 *     --build-property "compiler.cpp.extra_flags=-I<abs path>/WRO2026" \
 *     tools/vision_bench/
 *
 * Replace `<abs path>` with the absolute path to the repo's `WRO2026` sketch
 * directory (the one holding `VisionFrame.h` and `ConfigVision.h`). The Arduino
 * IDE requires the sketch folder name to match the `.ino` basename, so this
 * file MUST stay `tools/vision_bench/vision_bench.ino`. To flash, swap
 * `compile` for `upload` and add `--port <COMx>`.
 *
 * @note This is a diagnostic tool, not flight code, so it skips the full
 *       strict-OOP module treatment. It still sources every frame-contract
 *       number from `VisionFrame::` / `Config::` — no magic 9, 21, or 0xAA
 *       appears here.
 */

// We talk to Serial1 directly and deliberately do NOT include <MatrixMiniR4.h>
// — exactly like the firmware Vision.cpp (the reader uses the bare UART, not the
// vendor MiniR4SmartCamReader). Skipping it also dodges a real macro clash: the
// vendor's MiniR4OLED.h pulls in the Adafruit SSD1306 backwards-compat macro
// `#define WHITE 1`, which would mangle ConfigVision.h's `enum class Color { ...
// WHITE }` into `enum class Color { ... 1 }` and fail to compile. The Renesas
// core's <Arduino.h> already provides Serial / Serial1 / millis() / F().
#include <Arduino.h>

// Reached via the arduino-cli `-I<abs>/WRO2026` build flag (see HOW TO COMPILE
// above) — never copied into this folder (ADR-030). VisionFrame.h transitively
// includes ConfigVision.h, but we include it explicitly because we reference
// Config:: constants directly. We do NOT include the heavy Config.h: every
// constant this bench needs lives in the self-contained ConfigVision.h.
#include "ConfigVision.h"
#include "VisionFrame.h"

namespace
{
    /// @brief One complete HUE12 frame in bytes: header + length + data + checksum.
    ///        Derived from the Config word count — never hardcode 21.
    constexpr size_t kFrameBytes =
        1u /* header */ + 1u /* length */ + 2u * Config::VISION_FRAME_WORDS + 1u /* checksum */;

    /// @brief RX accumulation buffer capacity: room for a few full frames plus a
    ///        partial, so a misaligned start can still be resynced. Bounded so a
    ///        never-completing stream cannot overflow — we compact/slide on the
    ///        consumed prefix (the same shape the real reader uses), and as a hard
    ///        backstop we drop the oldest byte if the buffer ever fills without
    ///        yielding a frame (a corrupt/garbage stream must not wedge the loop).
    constexpr size_t kRxCapacity = kFrameBytes * 4u;

    /// @brief Rolling window length for the frames-per-second counter.
    constexpr unsigned long kFpsWindowMs = 1000u;

    /// @brief Serial monitor baud for USB diagnostic output (host side).
    constexpr unsigned long kUsbBaud = 115200u;

    /// @brief RX accumulation buffer and the count of valid bytes currently in it.
    uint8_t _rx[kRxCapacity];
    size_t  _rxLen = 0;

    /// @brief Frame-rate bookkeeping: frames seen since the window opened, and
    ///        the millis() timestamp the current window started at.
    unsigned long _framesThisWindow = 0;
    unsigned long _windowStartMs     = 0;

    /**
     * @brief Drain every available UART byte into the RX buffer, dropping the
     *        oldest bytes if the buffer is full so the stream can never wedge.
     */
    void pumpUartBytes()
    {
        while (Serial1.available() > 0)
        {
            if (_rxLen >= kRxCapacity)
            {
                // Buffer full and no frame extracted yet — slide off the oldest
                // byte to make room. A genuine frame is only kFrameBytes long, so
                // a full buffer with no frame means we are mid-garbage; advancing
                // by one preserves resync on a later 0xAA without overflowing.
                for (size_t i = 1; i < _rxLen; ++i)
                {
                    _rx[i - 1] = _rx[i];
                }
                --_rxLen;
            }

            const int b = Serial1.read();
            if (b < 0)
            {
                break;    // available() lied; nothing to read.
            }
            _rx[_rxLen++] = static_cast<uint8_t>(b);
        }
    }

    /**
     * @brief Compact the RX buffer by discarding its first @p consumed bytes,
     *        sliding any remaining tail down to the front.
     *
     * @param consumed Number of leading bytes to drop (clamped to `_rxLen`).
     */
    void compactRx(size_t consumed)
    {
        if (consumed >= _rxLen)
        {
            _rxLen = 0;
            return;
        }
        const size_t remaining = _rxLen - consumed;
        for (size_t i = 0; i < remaining; ++i)
        {
            _rx[i] = _rx[consumed + i];
        }
        _rxLen = remaining;
    }

    /**
     * @brief Print the per-slot raw hue/sta table for one decoded frame over USB.
     *
     * Prints all `Config::VISION_CELL_COUNT` unpacked indices as slot ids — the
     * bench maps cells->slots, so the raw unpacked array index IS the slot id the
     * operator records into `VISION_CELL_TO_SLOT`.
     *
     * @param words The Config::VISION_FRAME_WORDS decoded words from parseFrame().
     */
    void printFrame(const uint16_t words[Config::VISION_FRAME_WORDS])
    {
        uint8_t hue[Config::VISION_CELL_COUNT];
        uint8_t sta[Config::VISION_CELL_COUNT];
        VisionFrame::unpackFrame(words, hue, sta);

        Serial.println(F("---- frame ----"));
        for (uint8_t i = 0; i < Config::VISION_CELL_COUNT; ++i)
        {
            Serial.print(F("slot "));
            Serial.print(i);
            Serial.print(F(" : hue="));
            Serial.print(hue[i]);
            Serial.print(F(" : sta="));
            Serial.println(sta[i]);
        }
    }

    /**
     * @brief Emit the frames-per-second line once per rolling window and reset it.
     *
     * Called every loop iteration; only prints when the window has elapsed so the
     * operator can read a stable emission rate. Prints a distinct line when zero
     * frames were decoded so a silent link is obvious rather than ambiguous.
     */
    void reportFpsIfWindowElapsed()
    {
        const unsigned long now = millis();
        if (now - _windowStartMs < kFpsWindowMs)
        {
            return;
        }

        if (_framesThisWindow == 0)
        {
            Serial.println(F("fps=0 (no frames this second — check baud / D0-D1 wiring)"));
        }
        else
        {
            Serial.print(F("fps="));
            Serial.println(_framesThisWindow);
        }

        _framesThisWindow = 0;
        _windowStartMs    = now;
    }
}    // namespace

void setup()
{
    // USB diagnostic output to the serial monitor (host side).
    Serial.begin(kUsbBaud);

    // HUE12 camera UART. Serial1 IS MiniR4.Uart (MatrixMiniR4.h:114). Use the
    // Config baud so this bench and the firmware agree on one source of truth.
    Serial1.begin(Config::VISION_UART_BAUD);

    // Wake the camera (vendor SmartCamReader contract) — a cold-booted HUE12 may stay idle
    // until kicked with this command, so the bench sends it once like the firmware does.
    Serial1.print(Config::VISION_START_CMD);

    _rxLen            = 0;
    _framesThisWindow = 0;
    _windowStartMs    = millis();

    Serial.println(F("vision_bench: HUE12 bring-up diagnostic"));
    Serial.print(F("UART baud="));
    Serial.print(Config::VISION_UART_BAUD);
    Serial.print(F("  frame_words="));
    Serial.print(Config::VISION_FRAME_WORDS);
    Serial.print(F("  slots="));
    Serial.println(Config::VISION_SLOT_COUNT);
}

void loop()
{
    // 1. Pull whatever the camera has streamed into the accumulation buffer.
    pumpUartBytes();

    // 2. Extract every complete, checksum-valid frame currently buffered. Each
    //    parseFrame success advances `consumed` past the accepted frame; we
    //    compact and loop so a burst of buffered frames is fully drained. On a
    //    parse failure the `&&` short-circuits and the loop exits WITHOUT
    //    compacting, leaving the buffered bytes in place for the next pump to
    //    complete; a stream that never yields a frame is bounded by the
    //    oldest-byte slide in pumpUartBytes (the real reader resyncs similarly).
    uint16_t words[Config::VISION_FRAME_WORDS];
    size_t   consumed = 0;
    while (_rxLen >= kFrameBytes && VisionFrame::parseFrame(_rx, _rxLen, words, consumed))
    {
        printFrame(words);
        ++_framesThisWindow;
        compactRx(consumed);
    }

    // If no frame was found but the buffer is short of a full frame, leave the
    // bytes in place to be completed by the next pump. parseFrame only sets
    // consumed = _rxLen on a scan that found no header room, which is harmless
    // here because we re-pump before the next parse attempt.

    // 3. Report the measured emission rate once per second.
    reportFpsIfWindowElapsed();
}
