/**
 * @file vision_probe.ino
 * @brief Dual-UART raw-byte probe at 115200 — confirms which port the HUE12 camera streams on.
 *
 * @details
 * The vendor `MiniR4SmartCamReader::Begin()` opens the camera on `Serial1` at **115200**
 * (`Serial1.begin(115200)`), reading the same `0xAA`/length/checksum frame contract we use. Our
 * ConfigVision placeholder baud (9600) is therefore wrong, which is why nothing framed at 9600.
 * This probe opens BOTH candidate ports at 115200 and, once per second, prints each port's raw
 * byte count, how many were the `0xAA` frame header, and a hex preview:
 *
 *   - `Serial1` = UART1 = pins 22/23 — the vendor's camera port (expected winner).
 *   - `Serial2` = UART2 = pins 1/0 = the D0/D1 header socket.
 *
 * Reading:
 *   - `S1 bytes>0` with periodic `hdr0xAA` and hex starting `AA 09 ...` -> the camera is on
 *     Serial1 at 115200. Fix: VISION_SERIAL = Serial1, VISION_UART_BAUD = 115200.
 *   - `S2` lights up instead -> camera is on Serial2 at 115200.
 *   - both silent / garbage -> physical (power / socket / TX<->RX), unchanged from before.
 *
 * A single fixed baud on both ports (no per-loop `Serial.end()`/`begin()` cycling): the earlier
 * baud-sweep variant re-opened the UARTs every loop, which can wedge the R4 and print nothing.
 *
 * Build/flash/monitor with the shared uv driver:
 *     uv run tools/vision_probe/build.py
 *
 * Diagnostic tool, not flight code (same licence the sibling `vision_bench.ino` header claims);
 * it still sources the wake command and frame header from `Config::` / `VisionFrame::`.
 *
 * @note Writing the wake command to `Serial1` also pokes the R4 WiFi ESP32 UART. Harmless for a
 *       bench probe (WiFi is never initialised here).
 */

#include <Arduino.h>

#include "config/ConfigVision.h"
#include "src/perception/VisionFrame.h"    // VisionFrame::FRAME_HEADER (the 0xAA contract byte)

namespace
{
    /// @brief USB serial-monitor baud (host side).
    constexpr unsigned long kUsbBaud = 115200u;

    /// @brief Camera-link baud under test — the vendor SmartCamReader value, NOT the (wrong)
    ///        ConfigVision placeholder. Hardcoded here because this probe exists to test it.
    constexpr unsigned long kLinkBaud = 115200u;

    /// @brief Reporting window: summarise each port once per second.
    constexpr unsigned long kWindowMs = 1000u;

    /// @brief How many leading bytes of each window to echo as hex.
    constexpr size_t kPreviewBytes = 24u;

    /// @brief Per-port running tally for the current 1 s window.
    struct PortStat
    {
        arduino::HardwareSerial& uart;                  ///< The UART being probed (Serial1/Serial2).
        const char*              name;                  ///< Label printed on the monitor.
        unsigned long            bytes;                 ///< Raw bytes seen this window.
        unsigned long            headers;               ///< How many equalled FRAME_HEADER (0xAA).
        uint8_t                  preview[kPreviewBytes]; ///< First bytes of the window (hex dump).
        size_t                   previewLen;            ///< Valid entries in `preview`.
    };

    PortStat s1{Serial1, "S1(22/23)", 0, 0, {}, 0};
    PortStat s2{Serial2, "S2(D0/D1)", 0, 0, {}, 0};

    /// @brief Drain everything currently readable on a port into its tally.
    void drainPort(PortStat& p)
    {
        while (p.uart.available() > 0)
        {
            const int b = p.uart.read();
            if (b < 0)
            {
                break;
            }
            ++p.bytes;
            if (static_cast<uint8_t>(b) == VisionFrame::FRAME_HEADER)
            {
                ++p.headers;
            }
            if (p.previewLen < kPreviewBytes)
            {
                p.preview[p.previewLen++] = static_cast<uint8_t>(b);
            }
        }
    }

    /// @brief Print one port's window summary, then reset its tally for the next window.
    void reportPort(PortStat& p)
    {
        Serial.print(p.name);
        Serial.print(F(" bytes="));
        Serial.print(p.bytes);
        Serial.print(F(" hdr0xAA="));
        Serial.print(p.headers);
        Serial.print(F(" hex:"));
        for (size_t i = 0; i < p.previewLen; ++i)
        {
            Serial.print(' ');
            if (p.preview[i] < 0x10)
            {
                Serial.print('0');
            }
            Serial.print(p.preview[i], HEX);
        }
        Serial.println();

        p.bytes      = 0;
        p.headers    = 0;
        p.previewLen = 0;
    }

    unsigned long _windowStartMs = 0;
}    // namespace

void setup()
{
    Serial.begin(kUsbBaud);

    // Open BOTH candidate camera ports at the vendor link baud (115200).
    Serial1.begin(kLinkBaud);
    Serial2.begin(kLinkBaud);

    // Wake the camera on whichever port it lives on (vendor SmartCamReader contract).
    Serial1.print(Config::VISION_START_CMD);
    Serial2.print(Config::VISION_START_CMD);

    _windowStartMs = millis();

    Serial.print(F("vision_probe: dual-UART raw probe @ baud="));
    Serial.println(kLinkBaud);
    Serial.println(F("A steady stream with periodic hdr0xAA on one port = that is the camera."));
}

void loop()
{
    drainPort(s1);
    drainPort(s2);

    const unsigned long now = millis();
    if (now - _windowStartMs >= kWindowMs)
    {
        _windowStartMs = now;
        reportPort(s1);
        reportPort(s2);
        Serial.println(F("----"));
    }
}
