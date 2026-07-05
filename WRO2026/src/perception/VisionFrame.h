#ifndef VISION_FRAME_H
#define VISION_FRAME_H

#include <stddef.h>
#include <stdint.h>

#include "../../config/ConfigVision.h"

/**
 * @file VisionFrame.h
 * @brief Single source of truth for the HUE12 wire-frame parser, the frame
 *        unpacker, and the per-cell hue classifier.
 *
 * These three `inline` free functions are PURE: no I/O, no globals, no statics,
 * no `Serial`. They consume only their arguments and the compile-time constants
 * from `ConfigVision.h`. This is deliberate — per ADR-030 they are the
 * host-link test seam for the vision subsystem, so the offline test suite (T7)
 * and the bench harness (T8) link the very same symbols the firmware
 * (`Vision.cpp`, T4) runs, with zero hardware in the loop. The hardware-facing
 * UART read loop lives in `Vision.cpp`; everything that can be reasoned about
 * without a serial port lives here.
 *
 * Safety note: `parseFrame()` is the safety-critical boundary. It treats `buf`
 * as untrusted bytes off the wire and NEVER dereferences past `buf[len - 1]`.
 * A corrupt length byte (e.g. 200) must not provoke a read past the buffer;
 * every index is bounds-checked against `len` before use. The test suite runs
 * this path under AddressSanitizer, so any overrun fails the build.
 *
 * Wire contract (mirrors the vendor framing of the HUE12 `.mbr4` program):
 *
 *   [0xAA] [length] [word0_lo word0_hi] ... [wordN-1_lo wordN-1_hi] [checksum]
 *
 *   - `length` MUST equal `Config::VISION_FRAME_WORDS` (9).
 *   - Each word is little-endian: `word[k] = lo | (hi << 8)`.
 *   - A full frame is `1 (header) + 1 (length) + 2*length (data) + 1 (checksum)`
 *     bytes — 21 bytes for the 9-word contract.
 *   - Checksum seed is `0xAA ^ length`; the seed then XORs in EVERY one of the
 *     `2*length` data bytes (both the low and high byte of each word). The
 *     trailing checksum byte must equal that running XOR. The seed covers
 *     header+length; the running XOR covers data bytes only.
 */
namespace VisionFrame
{
    /**
     * @brief The HUE12 frame header / resync byte. The one literal that
     *        legitimately lives here — every other framing parameter (word
     *        count, cell count, sta gate, hue bands) comes from
     *        `ConfigVision.h`.
     */
    static constexpr uint8_t FRAME_HEADER = 0xAA;

    /**
     * @brief Scan `buf[0..len)` for the first complete, checksum-valid HUE12
     *        frame, resyncing on the header byte.
     *
     * The scan walks forward looking for a `FRAME_HEADER` (0xAA). At each
     * candidate header it attempts to validate a full frame in place. A
     * candidate is REJECTED — and the scan resumes from the byte AFTER that
     * header — when any of the following hold:
     *   - fewer than `1 + 1 + 2*VISION_FRAME_WORDS + 1` bytes remain in `buf`
     *     (a full frame cannot fit), OR
     *   - the length byte != `Config::VISION_FRAME_WORDS`, OR
     *   - the trailing XOR checksum does not match.
     * This rejects a `0xAA`-as-data false header and a corrupt length byte
     * without ever reading past `buf[len - 1]`.
     *
     * On the first fully-valid frame it writes the nine little-endian words
     * into @p outWords, sets @p consumed to the index one past the accepted
     * frame's checksum byte (so the caller can compact the buffer), and returns
     * `true`. If no complete valid frame exists it returns `false` and sets
     * @p consumed to `len` (the caller may safely drop the whole buffer; no
     * partial frame was started that is worth preserving — a genuine partial at
     * the tail would have failed the "enough bytes remain" guard and the caller
     * re-reads more bytes regardless).
     *
     * @param buf       Untrusted byte buffer off the UART. May be `nullptr`
     *                  only when @p len is 0.
     * @param len       Number of valid bytes in @p buf.
     * @param outWords  Receives the `Config::VISION_FRAME_WORDS` decoded words
     *                  on success; left untouched on failure.
     * @param consumed  Set to one-past the accepted checksum on success, or to
     *                  @p len on failure.
     * @return `true` if a valid frame was found and decoded; `false` otherwise.
     *
     * @note Bounds safety: the per-candidate check `(len - i) < kFrameBytes`
     *       guarantees indices `i .. i + kFrameBytes - 1` are all `< len`
     *       before any dereference, so no read crosses `buf[len - 1]`.
     * @note Checksum seed is `FRAME_HEADER ^ length`; data bytes only are XORed
     *       in afterward (low then high byte of each word).
     */
    inline bool parseFrame(
        const uint8_t* buf,
        size_t         len,
        uint16_t       outWords[Config::VISION_FRAME_WORDS],
        size_t&        consumed)
    {
        // Full frame layout: header + length + (2 bytes per word) + checksum.
        // Computed from Config — never hardcode 9 or 21.
        const size_t kDataBytes  = static_cast<size_t>(Config::VISION_FRAME_WORDS) * 2u;
        const size_t kFrameBytes = 1u /* header */ + 1u /* length */ + kDataBytes + 1u /* csum */;

        // Walk forward looking for a header byte. Resync advances one byte at a
        // time so a 0xAA appearing inside data does not desync the stream.
        for (size_t i = 0; i < len; ++i)
        {
            if (buf[i] != FRAME_HEADER)
            {
                continue;
            }

            // Candidate header at i. Reject if a full frame cannot fit. This is
            // the load-bearing bounds guard: it proves i .. i+kFrameBytes-1 are
            // all < len before any further indexing below.
            if ((len - i) < kFrameBytes)
            {
                // Not enough bytes for a frame starting here. A header this
                // close to the tail with no room cannot be validated; the scan
                // continues, but no later i can fit either, so this loop ends.
                continue;
            }

            const uint8_t length = buf[i + 1];
            if (length != Config::VISION_FRAME_WORDS)
            {
                // Corrupt / unexpected length — reject and resync from i+1.
                continue;
            }

            // Seed covers header + length; running XOR covers data bytes only.
            uint8_t      check     = static_cast<uint8_t>(FRAME_HEADER ^ length);
            const size_t dataStart = i + 2u;
            for (size_t d = 0; d < kDataBytes; ++d)
            {
                check ^= buf[dataStart + d];
            }

            const uint8_t checksum = buf[dataStart + kDataBytes];
            if (check != checksum)
            {
                // Checksum mismatch — reject and resync from i+1.
                continue;
            }

            // Valid frame. Decode the little-endian words: word[k] = lo | hi<<8.
            for (size_t k = 0; k < Config::VISION_FRAME_WORDS; ++k)
            {
                const size_t p = dataStart + 2u * k;
                outWords[k] =
                    static_cast<uint16_t>(buf[p] | (static_cast<uint16_t>(buf[p + 1]) << 8));
            }

            // One past the checksum byte.
            consumed = dataStart + kDataBytes + 1u;
            return true;
        }

        // No complete valid frame in the buffer.
        consumed = len;
        return false;
    }

    /**
     * @brief Unpack a decoded HUE12 word frame into per-cell hue and saturation
     *        arrays.
     *
     * Packing layout (defined by the `.mbr4` contract program):
     *   - Words 0..5 carry the 12 hues, two per word:
     *       hue[2*k]     = word[k] & 0xFF;          // k = 0..5 -> hue[0,2,4,..,10]
     *       hue[2*k + 1] = (word[k] >> 8) & 0xFF;   // k = 0..5 -> hue[1,3,5,..,11]
     *   - Words 6..8 carry the 12 saturations, four 4-bit nibbles per word:
     *       sta[j] = (word[6 + j/4] >> (4 * (j % 4))) & 0x0F;   // j = 0..11
     *
     * Exactly `Config::VISION_CELL_COUNT` (12) hues and 12 stas are written.
     *
     * @param words  The `Config::VISION_FRAME_WORDS` decoded words from
     *               `parseFrame()`.
     * @param hue    Receives `Config::VISION_CELL_COUNT` hue bytes (0..255).
     * @param sta    Receives `Config::VISION_CELL_COUNT` saturation nibbles
     *               (0..15).
     *
     * @note Hue and sta cell counts both equal `Config::VISION_CELL_COUNT`; the
     *       loop bounds derive from Config, never a hardcoded 12 or 9.
     */
    inline void unpackFrame(
        const uint16_t words[Config::VISION_FRAME_WORDS],
        uint8_t        hue[Config::VISION_CELL_COUNT],
        uint8_t        sta[Config::VISION_CELL_COUNT])
    {
        // Hues: two per word, low byte = even cell, high byte = odd cell.
        for (uint8_t k = 0; 2u * k < Config::VISION_CELL_COUNT; ++k)
        {
            hue[2u * k]      = static_cast<uint8_t>(words[k] & 0xFF);
            hue[2u * k + 1u] = static_cast<uint8_t>((words[k] >> 8) & 0xFF);
        }

        // Saturations: four 4-bit nibbles per word, starting at word index
        // (VISION_CELL_COUNT / 2) which is the first word past the hue words.
        const uint8_t staWordBase = static_cast<uint8_t>(Config::VISION_CELL_COUNT / 2u);
        for (uint8_t j = 0; j < Config::VISION_CELL_COUNT; ++j)
        {
            const uint16_t word  = words[staWordBase + j / 4u];
            const uint8_t  shift = static_cast<uint8_t>(4u * (j % 4u));
            sta[j]               = static_cast<uint8_t>((word >> shift) & 0x0F);
        }
    }

    /**
     * @brief Classify one HUE12 cell reading into a `Config::Color`.
     *
     * Resolution order is fixed and LOAD-BEARING — the function returns at the
     * first matching step:
     *   1. `hue == 255`            -> UNKNOWN  (device "NONE" / no detection)
     *   2. `246 <= hue <= 250`     -> UNKNOWN  (device Black)
     *   3. `240 <= hue <= 245`     -> WHITE    (device achromatic White)
     *   3a. `hue > 239` (i.e. 251..254) -> UNKNOWN  (reserved device codes;
     *                              255 / 246..250 / 240..245 already returned
     *                              above, so the only codes left here are the
     *                              undefined 251..254 — fence them off so they
     *                              never reach the step-4 sta gate and get
     *                              mis-classified as WHITE)
     *   4. chromatic (`hue <= 239`) AND `sta < Config::VISION_WHITE_STA_MAX`
     *                              -> WHITE    (desaturated / washed out)
     *   5. chromatic `hue` inside a `Config::HUE_BANDS[]` entry (`lo <= hue <= hi`,
     *      inclusive)             -> that entry's colour
     *   6. otherwise               -> UNKNOWN  (band gap / uncalibrated)
     *
     * Steps 1–2 (bad device codes) MUST precede the step-4 saturation gate:
     * a low-saturation Black reading (hue 248, sta 1) returns UNKNOWN, never
     * WHITE. Putting the device-code checks first is what makes that hold (TC21).
     * Step 3a likewise fences the reserved 251..254 band off the sta gate.
     *
     * @param hue  Cell hue byte (device range: 0..239 chromatic, 240..245 White,
     *             246..250 Black, 255 None).
     * @param sta  Cell saturation nibble (0..15).
     * @return The classified `Config::Color`; `Config::Color::UNKNOWN` when the
     *         reading is a bad device code or falls outside every hue band.
     *
     * @note The hue-band scan iterates `HUE_BANDS` by its real length
     *       (`sizeof(HUE_BANDS) / sizeof(HUE_BANDS[0])`), so adding or removing
     *       a band at bench T8 needs no change here.
     */
    inline Config::Color classifyHue(uint8_t hue, uint8_t sta)
    {
        // Step 1: device NONE code — no detection.
        if (hue == 255)
        {
            return Config::Color::UNKNOWN;
        }

        // Step 2: device Black — must come BEFORE the saturation gate so a
        // low-sta Black does not leak through as White (TC21).
        if (hue >= 246 && hue <= 250)
        {
            return Config::Color::UNKNOWN;
        }

        // Step 3: device-side achromatic White.
        if (hue >= 240 && hue <= 245)
        {
            return Config::Color::WHITE;
        }

        // Step 3a: reserved device codes. 240..245 (White), 246..250 (Black), and 255 (None) all
        // returned above, so the ONLY codes > 239 reaching here are the undefined 251..254. They
        // are NOT chromatic, so fence them off as UNKNOWN before the step-4 sta gate — otherwise a
        // low-sta reserved code would leak through as WHITE.
        if (hue > 239)
        {
            return Config::Color::UNKNOWN;
        }

        // Step 4: chromatic but desaturated -> White. hue <= 239 is guaranteed
        // here because every code >= 240 was handled above.
        if (sta < Config::VISION_WHITE_STA_MAX)
        {
            return Config::Color::WHITE;
        }

        // Step 5: chromatic hue inside a calibrated band (inclusive bounds).
        const size_t bandCount = sizeof(Config::HUE_BANDS) / sizeof(Config::HUE_BANDS[0]);
        for (size_t b = 0; b < bandCount; ++b)
        {
            const Config::HueBand& band = Config::HUE_BANDS[b];
            if (hue >= band.lo && hue <= band.hi)
            {
                return band.color;
            }
        }

        // Step 6: band gap / uncalibrated.
        return Config::Color::UNKNOWN;
    }
}    // namespace VisionFrame

#endif    // VISION_FRAME_H
