/**
 * @file vision_read.ino
 * @brief Live HUE12 read-out for hue-band tuning (NOT flight code).
 *
 * @details
 * Prints, a couple of times per second, the most recent decoded frame as a per-slot table of the
 * RAW `hue`/`sta` AND the colour the CURRENT `Config::HUE_BANDS` / `Config::VISION_WHITE_STA_MAX`
 * classify it as — using the exact same `VisionFrame::classifyHue()` the firmware uses. That side
 * by side is what you tune against:
 *
 *   slot  4 : hue=132 sta=11 -> GREEN
 *   slot  5 : hue= 38 sta= 9 -> YELLOW
 *   slot  6 : hue=210 sta= 2 -> UNKNOWN      <- a tile that no band covers yet
 *
 * TUNING LOOP
 *   1. Cover all slots, then expose ONE known-colour tile at a time.
 *   2. Read its steady `hue=`/`sta=` and see how the current bands label it.
 *   3. Edit `Config::HUE_BANDS` (add/adjust a band's lo/hi) or `VISION_WHITE_STA_MAX`, re-flash,
 *      and repeat until every tile classifies correctly. RED has no band yet: its hue wraps the
 *      0/239 seam, so read both ends off real red tiles before adding it.
 *   The printout is by RAW UNPACKED SLOT INDEX (device order), the same identifier used to build
 *   `Config::VISION_CELL_TO_SLOT` — not by game cell.
 *
 * Build/flash/monitor with the shared uv driver:
 *     uv run tools/vision_read/build.py
 *
 * Diagnostic tool, not flight code (same licence the sibling `vision_bench.ino` header claims);
 * it still sources the port, baud, frame contract, and classifier from `Config::` / `VisionFrame::`.
 */

#include <Arduino.h>

#include "config/ConfigVision.h"
#include "src/perception/VisionFrame.h"

namespace
{
    /// @brief USB serial-monitor baud (host side).
    constexpr unsigned long kUsbBaud = 115200u;

    /// @brief Print the latest frame at most this often — the camera runs ~16 fps, far too fast to
    ///        read every frame; a steady tile only needs a refresh a couple of times per second.
    constexpr unsigned long kPrintIntervalMs = 500u;

    /// @brief One complete HUE12 frame in bytes: header + length + data + checksum (from Config).
    constexpr size_t kFrameBytes =
        1u /* header */ + 1u /* length */ + 2u * Config::VISION_FRAME_WORDS + 1u /* checksum */;

    /// @brief RX accumulation buffer capacity: a few frames plus a partial, bounded so a
    ///        never-completing stream cannot overflow (oldest byte is slid off when full).
    constexpr size_t kRxCapacity = kFrameBytes * 4u;

    uint8_t _rx[kRxCapacity];
    size_t  _rxLen = 0;

    uint16_t      _lastWords[Config::VISION_FRAME_WORDS];
    bool          _haveFrame   = false;
    unsigned long _lastPrintMs = 0;

    /// @brief Stable short name for a classified colour.
    const char* colorName(Config::Color c)
    {
        switch (c)
        {
        case Config::Color::UNKNOWN:
            return "UNKNOWN";
        case Config::Color::YELLOW:
            return "YELLOW";
        case Config::Color::BLUE:
            return "BLUE";
        case Config::Color::GREEN:
            return "GREEN";
        case Config::Color::WHITE:
            return "WHITE";
        case Config::Color::RED:
            return "RED";
        }
        return "?";
    }

    /// @brief Drain every available UART byte into the RX buffer, dropping the oldest byte if it
    ///        fills so a garbage stream can never wedge the loop (mirrors vision_bench).
    void pumpUartBytes()
    {
        while (VISION_SERIAL.available() > 0)
        {
            if (_rxLen >= kRxCapacity)
            {
                for (size_t i = 1; i < _rxLen; ++i)
                {
                    _rx[i - 1] = _rx[i];
                }
                --_rxLen;
            }
            const int b = VISION_SERIAL.read();
            if (b < 0)
            {
                break;
            }
            _rx[_rxLen++] = static_cast<uint8_t>(b);
        }
    }

    /// @brief Discard the first @p consumed bytes, sliding the tail to the front.
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

    /// @brief Right-align a 0..255 value in a 3-char field so the columns line up.
    void printPad3(uint8_t v)
    {
        if (v < 100)
        {
            Serial.print(' ');
        }
        if (v < 10)
        {
            Serial.print(' ');
        }
        Serial.print(v);
    }

    /// @brief Print the latest frame as a per-slot raw + classified table.
    void printLatestFrame()
    {
        uint8_t hue[Config::VISION_CELL_COUNT];
        uint8_t sta[Config::VISION_CELL_COUNT];
        VisionFrame::unpackFrame(_lastWords, hue, sta);

        Serial.println(F("---- frame ----"));
        for (uint8_t s = 0; s < Config::VISION_CELL_COUNT; ++s)
        {
            const Config::Color colour = VisionFrame::classifyHue(hue[s], sta[s]);
            Serial.print(F("slot "));
            printPad3(s);
            Serial.print(F(" : hue="));
            printPad3(hue[s]);
            Serial.print(F(" sta="));
            printPad3(sta[s]);
            Serial.print(F(" -> "));
            Serial.println(colorName(colour));
        }
    }
}    // namespace

void setup()
{
    Serial.begin(kUsbBaud);

    VISION_SERIAL.begin(Config::VISION_UART_BAUD);
    VISION_SERIAL.print(Config::VISION_START_CMD);

    _rxLen       = 0;
    _haveFrame   = false;
    _lastPrintMs = millis();

    Serial.println(F("vision_read: raw hue/sta + current-band classification, per slot"));
    Serial.print(F("port via Config::VISION_SERIAL @ baud="));
    Serial.println(Config::VISION_UART_BAUD);
}

void loop()
{
    // 1. Pull streamed bytes and keep only the MOST RECENT complete frame (a steady scene means
    //    the last frame in the buffer is the freshest reading to classify).
    pumpUartBytes();

    uint16_t words[Config::VISION_FRAME_WORDS];
    size_t   consumed = 0;
    while (_rxLen >= kFrameBytes && VisionFrame::parseFrame(_rx, _rxLen, words, consumed))
    {
        for (uint8_t i = 0; i < Config::VISION_FRAME_WORDS; ++i)
        {
            _lastWords[i] = words[i];
        }
        _haveFrame = true;
        compactRx(consumed);
    }

    // 2. Print the latest frame on a readable cadence.
    const unsigned long now = millis();
    if (_haveFrame && (now - _lastPrintMs) >= kPrintIntervalMs)
    {
        _lastPrintMs = now;
        printLatestFrame();
    }
}
