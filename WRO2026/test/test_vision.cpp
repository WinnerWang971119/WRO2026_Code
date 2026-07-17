/**
 * @file test_vision.cpp
 * @brief Host-compile tests for the HUE12 Vision module (T7) — TC1..TC21.
 *
 * Links the production `Vision.cpp` (T4), the header-only `VisionFrame.h` (T2), and
 * `ConfigVision.h` (T1) against the host mocks under `WRO2026/test/mocks/`:
 *   - `<Arduino.h>`  -> mock clock (`g_mockMillis`, advanced only by `delay()`), the
 *                       capturing USB `Serial`, and the streaming `Serial1` UART mock
 *                       (`mockSerial1Feed` / `mockSerial1Clear` / drip-feed / `mockTickCount`).
 * The global `Vision` is defined in `Vision.cpp` (linked by CMake).
 *
 * Two test families:
 *   - Unit: the pure `VisionFrame::{parseFrame,unpackFrame,classifyHue}` free functions
 *     (TC1..TC11, TC17, TC21) — no `Serial1`, no clock.
 *   - Integration: `Vision::readMosaic()` driven over the mock `Serial1` (TC12..TC16,
 *     TC18, TC19, TC20) — frames delivered DURING the wait (after the read's own flush),
 *     either via a registered tick callback that feeds bytes or via drip-feed.
 *
 * ## readMosaic delivery model (load-bearing — see the Vision.cpp author's note)
 * `readMosaic()` FLUSHES `Serial1`'s RX at the start of every attempt, then discards
 * `VISION_FLUSH_DISCARD_FRAMES` (=1) valid frames and latches the NEXT one. So bytes fed
 * with drip OFF *before* the call are flushed away. For a happy-path success the fresh
 * frames must be delivered AFTER the flush — done here by registering a tick callback that
 * injects the frame bytes on the first spin iteration (the verified approach). A success
 * needs at least `VISION_FLUSH_DISCARD_FRAMES + 1` = 2 valid frames after the flush: the
 * first is discarded, the SECOND is latched, so the latched colours are the 2nd frame's.
 *
 * ## TC19 cell->slot override
 * The non-identity `VISION_CELL_TO_SLOT` is injected via the `VISION_CELL_TO_SLOT_DEFAULT`
 * macro hook in `ConfigVision.h`. It is defined as a TARGET-WIDE compile definition in
 * `CMakeLists.txt` (not in this file) — `Vision.cpp` reads the map at RUNTIME inside
 * `readMosaic()`, so BOTH this TU and `Vision.cpp`'s TU must compile against the SAME
 * non-identity map. A per-file `#define` here would only affect this TU and leave
 * `Vision.cpp` on the identity map, producing a spurious cell/slot mismatch. The map is a
 * self-inverse permutation, cell c -> slot (11 - c): every slot 0..11 is referenced exactly
 * once, so the ConfigChecks.h uniqueness/range static_asserts still hold. TC19 asserts
 * getColor(c) == classify(frame slot 11 - c), which a transposed map would fail.
 */

#include "TestAssert.h"

#include "Arduino.h"

// Production code under test (pulls ConfigVision.h transitively):
#include "src/perception/VisionFrame.h"
#include "src/perception/Vision.h"

#include <cstddef>
#include <cstdint>
#include <vector>

// ===========================================================================
// Frame-builder helpers — construct the on-wire HUE12 byte stream.
//
// Wire contract (VisionFrame.h docblock):
//   [0xAA] [length==VISION_FRAME_WORDS] [w0_lo w0_hi] ... [w8_lo w8_hi] [xor_checksum]
//   - word[k]      = hue[2k] | (hue[2k+1] << 8)            for k = 0..5   (12 hues)
//   - word[6+j/4] |= (sta[j] & 0xF) << (4 * (j % 4))       for j = 0..11  (12 stas, nibbles)
//   - checksum seed = 0xAA ^ length, then XOR every one of the 2*length data bytes.
// ===========================================================================

namespace
{
    /// Full HUE12 frame size in bytes — header + length + 2 bytes/word + checksum.
    /// Derived from Config so it tracks VISION_FRAME_WORDS (never hardcode 21).
    constexpr std::size_t kFrameBytes =
        1u + 1u + 2u * static_cast<std::size_t>(Config::VISION_FRAME_WORDS) + 1u;

    /// Pack the 12 hues / 12 stas into the VISION_FRAME_WORDS decoded words, applying the
    /// EXACT inverse of `VisionFrame::unpackFrame`'s split.
    void packWords(
        const uint8_t hue[Config::VISION_CELL_COUNT],
        const uint8_t sta[Config::VISION_CELL_COUNT],
        uint16_t      words[Config::VISION_FRAME_WORDS])
    {
        for (uint8_t k = 0; k < Config::VISION_FRAME_WORDS; ++k)
        {
            words[k] = 0;
        }

        // Hues: two per word, low byte = even cell, high byte = odd cell (words 0..5).
        for (uint8_t k = 0; 2u * k < Config::VISION_CELL_COUNT; ++k)
        {
            words[k] = static_cast<uint16_t>(hue[2u * k]
                                             | (static_cast<uint16_t>(hue[2u * k + 1u]) << 8));
        }

        // Stas: four 4-bit nibbles per word, starting at word (VISION_CELL_COUNT / 2).
        const uint8_t staWordBase = static_cast<uint8_t>(Config::VISION_CELL_COUNT / 2u);
        for (uint8_t j = 0; j < Config::VISION_CELL_COUNT; ++j)
        {
            const uint8_t shift = static_cast<uint8_t>(4u * (j % 4u));
            words[staWordBase + j / 4u] |= static_cast<uint16_t>((sta[j] & 0x0F) << shift);
        }
    }

    /// Build the full on-wire byte stream for one frame from 12 hues + 12 stas.
    /// `corruptChecksum == true` flips the trailing checksum byte (valid framing, bad XOR).
    std::vector<uint8_t> buildFrame(
        const uint8_t hue[Config::VISION_CELL_COUNT],
        const uint8_t sta[Config::VISION_CELL_COUNT],
        bool          corruptChecksum = false)
    {
        uint16_t words[Config::VISION_FRAME_WORDS];
        packWords(hue, sta, words);

        std::vector<uint8_t> out;
        out.reserve(kFrameBytes);

        const uint8_t length = Config::VISION_FRAME_WORDS;
        out.push_back(VisionFrame::FRAME_HEADER);    // 0xAA
        out.push_back(length);

        // Seed covers header + length; running XOR covers data bytes only.
        uint8_t check = static_cast<uint8_t>(VisionFrame::FRAME_HEADER ^ length);
        for (uint8_t k = 0; k < Config::VISION_FRAME_WORDS; ++k)
        {
            const uint8_t lo = static_cast<uint8_t>(words[k] & 0xFF);
            const uint8_t hi = static_cast<uint8_t>((words[k] >> 8) & 0xFF);
            out.push_back(lo);
            out.push_back(hi);
            check = static_cast<uint8_t>(check ^ lo ^ hi);
        }

        if (corruptChecksum)
        {
            check = static_cast<uint8_t>(check ^ 0xFF);    // guaranteed-wrong checksum
        }
        out.push_back(check);
        return out;
    }

    /// Convenience: build a frame where every cell has the same hue + sta.
    std::vector<uint8_t> buildUniformFrame(uint8_t hue, uint8_t sta, bool corrupt = false)
    {
        uint8_t h[Config::VISION_CELL_COUNT];
        uint8_t s[Config::VISION_CELL_COUNT];
        for (uint8_t c = 0; c < Config::VISION_CELL_COUNT; ++c)
        {
            h[c] = hue;
            s[c] = sta;
        }
        return buildFrame(h, s, corrupt);
    }

    // -----------------------------------------------------------------------
    // Representative hues for each colour (interior of each placeholder band),
    // and a high sta so the white STA-gate never triggers.
    // -----------------------------------------------------------------------
    constexpr uint8_t kHueYellow = 32;    // inside HUE_BANDS[0] {20,45}
    constexpr uint8_t kHueGreen  = 110;   // inside HUE_BANDS[1] {80,140}
    constexpr uint8_t kHueBlue   = 170;   // inside HUE_BANDS[2] {141,200}
    constexpr uint8_t kHueBlack  = 248;   // device Black (246..250)
    constexpr uint8_t kStaHigh   = 15;    // >= VISION_WHITE_STA_MAX — chromatic, not washed out

    // -----------------------------------------------------------------------
    // Tick-callback frame injection. readMosaic() flushes RX at attempt start, so the
    // fresh frames must arrive AFTER the flush — i.e. from inside the wait loop. We register
    // a callback that, on its FIRST invocation only, feeds a pre-staged byte stream into the
    // (now-flushed) RX with drip OFF, so the whole batch is `available()` on the next poll.
    // The one-shot guard prevents re-feeding on every spin (which would never let the buffer
    // drain to a clean candidate). A function-static holds the staged bytes since the C tick
    // callback signature (`void(*)()`) takes no argument.
    // -----------------------------------------------------------------------
    std::vector<uint8_t> g_injectBytes;    // bytes the next read's callback will deliver
    bool                 g_injectFired = false;

    void stageInjection(const std::vector<uint8_t>& bytes)
    {
        g_injectBytes = bytes;
        g_injectFired = false;
    }

    void injectTickCallback()
    {
        if (g_injectFired)
        {
            return;    // deliver exactly once, after the flush
        }
        g_injectFired = true;
        if (!g_injectBytes.empty())
        {
            mockSerial1Feed(g_injectBytes.data(), g_injectBytes.size());
        }
    }

    /// Reset the whole mock world and re-init Vision for a fresh scenario.
    /// Clears both Serial1 RX queues, the USB capture buffer, the clock, the tick counter,
    /// disables drip-feed, clears the staged injection, drops any tick callback, then
    /// re-`begin()`s Vision (which resets the latch + fault and re-opens Serial1).
    void resetWorld()
    {
        mockSerial1Clear();
        mockSerial1SetDripFeed(0);    // drip OFF by default
        Serial.clearCapture();
        mockResetMillis();
        mockResetTickCount();

        g_injectBytes.clear();
        g_injectFired = false;

        Vision.setTickCallback(nullptr);
        Vision.begin();               // re-opens Serial1, resets latch + fault, clears _hasRead
        Vision.clearFault();          // defensive — begin() already clears it

        // begin() does NOT flush RX, but resetWorld already cleared it above. Re-clear the
        // capture so a begin()-time warning (there is none today) can't leak into a TC20 count.
        Serial.clearCapture();
    }
}    // namespace

// ===========================================================================
// TC1 — classifyHue at each chromatic band midpoint -> the matching Color.
// (AC2)
// ===========================================================================

static void test_TC1_classify_band_midpoints()
{
    TEST_SCOPE("TC1: classifyHue band midpoints Y/G/B -> matching Color");

    ASSERT_TRUE(VisionFrame::classifyHue(kHueYellow, kStaHigh) == Config::Color::YELLOW);
    ASSERT_TRUE(VisionFrame::classifyHue(kHueGreen, kStaHigh) == Config::Color::GREEN);
    ASSERT_TRUE(VisionFrame::classifyHue(kHueBlue, kStaHigh) == Config::Color::BLUE);
}

// ===========================================================================
// TC2 — classifyHue device-side achromatic White band 240..245 -> WHITE.
// (AC2)
// ===========================================================================

static void test_TC2_classify_white_band()
{
    TEST_SCOPE("TC2: classifyHue 240..245 -> WHITE");

    // High sta so this is NOT the STA-gate path — pure device-White band.
    ASSERT_TRUE(VisionFrame::classifyHue(240, kStaHigh) == Config::Color::WHITE);
    ASSERT_TRUE(VisionFrame::classifyHue(242, kStaHigh) == Config::Color::WHITE);
    ASSERT_TRUE(VisionFrame::classifyHue(245, kStaHigh) == Config::Color::WHITE);
}

// ===========================================================================
// TC3 — chromatic hue with sta < VISION_WHITE_STA_MAX -> WHITE (desaturated gate).
// (AC2)
// ===========================================================================

static void test_TC3_classify_low_sta_chromatic_is_white()
{
    TEST_SCOPE("TC3: chromatic hue + sta < VISION_WHITE_STA_MAX -> WHITE");

    // A hue that WOULD be GREEN at high sta becomes WHITE when desaturated.
    ASSERT_TRUE(VisionFrame::classifyHue(kHueGreen, 0) == Config::Color::WHITE);
    ASSERT_TRUE(VisionFrame::classifyHue(kHueGreen, Config::VISION_WHITE_STA_MAX - 1)
                == Config::Color::WHITE);
    // At exactly the threshold it is NOT washed out — falls through to the band (GREEN).
    ASSERT_TRUE(VisionFrame::classifyHue(kHueGreen, Config::VISION_WHITE_STA_MAX)
                == Config::Color::GREEN);
}

// ===========================================================================
// TC4 — device NONE (255) and device Black (246..250) -> UNKNOWN.
// (AC2)
// ===========================================================================

static void test_TC4_classify_none_and_black_unknown()
{
    TEST_SCOPE("TC4: hue 255 and 246..250 -> UNKNOWN");

    // High sta so neither result comes from the STA-gate.
    ASSERT_TRUE(VisionFrame::classifyHue(255, kStaHigh) == Config::Color::UNKNOWN);
    ASSERT_TRUE(VisionFrame::classifyHue(246, kStaHigh) == Config::Color::UNKNOWN);
    ASSERT_TRUE(VisionFrame::classifyHue(248, kStaHigh) == Config::Color::UNKNOWN);
    ASSERT_TRUE(VisionFrame::classifyHue(250, kStaHigh) == Config::Color::UNKNOWN);
}

// ===========================================================================
// TC5 — band-gap and inclusive boundary correctness (lo, hi, lo-1, hi+1).
// (AC2)
// ===========================================================================

static void test_TC5_classify_band_boundaries_and_gap()
{
    TEST_SCOPE("TC5: band boundaries inclusive (lo,hi) + gaps -> UNKNOWN");

    // YELLOW band {20,45}: lo and hi inclusive; lo-1 and hi+1 outside.
    ASSERT_TRUE(VisionFrame::classifyHue(20, kStaHigh) == Config::Color::YELLOW);    // lo
    ASSERT_TRUE(VisionFrame::classifyHue(45, kStaHigh) == Config::Color::YELLOW);    // hi
    ASSERT_TRUE(VisionFrame::classifyHue(19, kStaHigh) == Config::Color::UNKNOWN);   // lo-1 (gap)
    ASSERT_TRUE(VisionFrame::classifyHue(46, kStaHigh) == Config::Color::UNKNOWN);   // hi+1 (gap)

    // GREEN band {80,140}: lo/hi inclusive.
    ASSERT_TRUE(VisionFrame::classifyHue(80, kStaHigh) == Config::Color::GREEN);
    ASSERT_TRUE(VisionFrame::classifyHue(140, kStaHigh) == Config::Color::GREEN);

    // BLUE band {141,200}: 141 is the first BLUE (hi GREEN is 140 — adjacent, no gap),
    // 200 is the last BLUE, 201 is the gap before device White at 240.
    ASSERT_TRUE(VisionFrame::classifyHue(141, kStaHigh) == Config::Color::BLUE);
    ASSERT_TRUE(VisionFrame::classifyHue(200, kStaHigh) == Config::Color::BLUE);
    ASSERT_TRUE(VisionFrame::classifyHue(201, kStaHigh) == Config::Color::UNKNOWN);   // gap

    // A gap strictly between two bands (between YELLOW hi=45 and GREEN lo=80).
    ASSERT_TRUE(VisionFrame::classifyHue(60, kStaHigh) == Config::Color::UNKNOWN);
}

// ===========================================================================
// TC6 — unpackFrame of a known 9-word frame -> exact 12 hues + 12 stas.
// (AC1)
// ===========================================================================

static void test_TC6_unpackframe_exact()
{
    TEST_SCOPE("TC6: unpackFrame 9 words -> exact 12 hues + 12 stas");

    // Distinct, non-symmetric values so a swapped lo/hi byte or nibble is caught.
    uint8_t hueIn[Config::VISION_CELL_COUNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    uint8_t staIn[Config::VISION_CELL_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

    uint16_t words[Config::VISION_FRAME_WORDS];
    packWords(hueIn, staIn, words);

    uint8_t hueOut[Config::VISION_CELL_COUNT];
    uint8_t staOut[Config::VISION_CELL_COUNT];
    VisionFrame::unpackFrame(words, hueOut, staOut);

    for (uint8_t c = 0; c < Config::VISION_CELL_COUNT; ++c)
    {
        ASSERT_EQ(hueOut[c], hueIn[c]);
        ASSERT_EQ(staOut[c], staIn[c]);
    }
}

// ===========================================================================
// TC7 — parseFrame of a clean valid frame -> true, words correct, consumed past frame.
// (AC5)
// ===========================================================================

static void test_TC7_parseframe_clean_valid()
{
    TEST_SCOPE("TC7: parseFrame clean valid frame -> true, words + consumed correct");

    uint8_t hueIn[Config::VISION_CELL_COUNT] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120};
    uint8_t staIn[Config::VISION_CELL_COUNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    uint16_t expectWords[Config::VISION_FRAME_WORDS];
    packWords(hueIn, staIn, expectWords);

    const std::vector<uint8_t> frame = buildFrame(hueIn, staIn);
    ASSERT_EQ(frame.size(), kFrameBytes);

    uint16_t outWords[Config::VISION_FRAME_WORDS];
    std::size_t consumed = 0;
    const bool ok = VisionFrame::parseFrame(frame.data(), frame.size(), outWords, consumed);

    ASSERT_TRUE(ok);
    for (uint8_t k = 0; k < Config::VISION_FRAME_WORDS; ++k)
    {
        ASSERT_EQ(outWords[k], expectWords[k]);
    }
    // consumed == one past the checksum byte == full frame length here (frame starts at 0).
    ASSERT_EQ(consumed, kFrameBytes);
}

// ===========================================================================
// TC8 — buffer starting MID-frame (garbage prefix) then a full frame -> resyncs.
// (AC5)
// ===========================================================================

static void test_TC8_parseframe_resyncs_after_garbage()
{
    TEST_SCOPE("TC8: parseFrame mid-frame garbage then a full frame -> resyncs to it");

    // Leading garbage with NO 0xAA so it cannot false-sync — the parser must skip it.
    std::vector<uint8_t> buf = {0x11, 0x22, 0x33, 0x44, 0x55};

    uint8_t hueIn[Config::VISION_CELL_COUNT] = {5, 15, 25, 35, 45, 55, 65, 75, 85, 95, 105, 115};
    uint8_t staIn[Config::VISION_CELL_COUNT] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    const std::vector<uint8_t> frame = buildFrame(hueIn, staIn);
    buf.insert(buf.end(), frame.begin(), frame.end());

    uint16_t expectWords[Config::VISION_FRAME_WORDS];
    packWords(hueIn, staIn, expectWords);

    uint16_t outWords[Config::VISION_FRAME_WORDS];
    std::size_t consumed = 0;
    const bool ok = VisionFrame::parseFrame(buf.data(), buf.size(), outWords, consumed);

    ASSERT_TRUE(ok);
    for (uint8_t k = 0; k < Config::VISION_FRAME_WORDS; ++k)
    {
        ASSERT_EQ(outWords[k], expectWords[k]);
    }
    // The real frame started after the 5-byte garbage prefix.
    ASSERT_EQ(consumed, (std::size_t) 5 + kFrameBytes);
}

// ===========================================================================
// TC9 — a 0xAA appearing as a DATA byte before the real header -> no false-sync.
// (AC5)
// ===========================================================================

static void test_TC9_parseframe_no_false_sync_on_aa_data()
{
    TEST_SCOPE("TC9: 0xAA-as-data before real header -> no false-sync, returns real frame");

    // A lone 0xAA followed by a non-length byte: the candidate at index 0 must be rejected
    // (length mismatch), the scan resumes from index 1, and the real frame is found.
    std::vector<uint8_t> buf = {VisionFrame::FRAME_HEADER, 0x77, 0x01};

    uint8_t hueIn[Config::VISION_CELL_COUNT] = {12, 24, 36, 48, 60, 72, 84, 96, 108, 120, 132, 144};
    uint8_t staIn[Config::VISION_CELL_COUNT] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    const std::vector<uint8_t> frame = buildFrame(hueIn, staIn);
    const std::size_t prefix = buf.size();
    buf.insert(buf.end(), frame.begin(), frame.end());

    uint16_t expectWords[Config::VISION_FRAME_WORDS];
    packWords(hueIn, staIn, expectWords);

    uint16_t outWords[Config::VISION_FRAME_WORDS];
    std::size_t consumed = 0;
    const bool ok = VisionFrame::parseFrame(buf.data(), buf.size(), outWords, consumed);

    ASSERT_TRUE(ok);
    for (uint8_t k = 0; k < Config::VISION_FRAME_WORDS; ++k)
    {
        ASSERT_EQ(outWords[k], expectWords[k]);
    }
    ASSERT_EQ(consumed, prefix + kFrameBytes);
}

// ===========================================================================
// TC10 — corrupt length byte (200) -> rejected, no read past len (ASan-clean).
// (AC5)
// ===========================================================================

static void test_TC10_parseframe_corrupt_length_no_overrun()
{
    TEST_SCOPE("TC10: length=200 corrupt -> rejected, no overrun (ASan-clean)");

    // A header + a wildly-too-large length byte in a 6-byte buffer. Here the too-short guard
    // `(len - i) < kFrameBytes` short-circuits BEFORE the length byte is even read, so the 200 is
    // never inspected — this sub-case proves the tail guard rejects (and never indexes past
    // buf[len-1]). Under -fsanitize=address an out-of-bounds read would trap.
    std::vector<uint8_t> buf = {VisionFrame::FRAME_HEADER, 200, 0x01, 0x02, 0x03, 0x04};

    uint16_t outWords[Config::VISION_FRAME_WORDS];
    std::size_t consumed = 0;
    const bool ok = VisionFrame::parseFrame(buf.data(), buf.size(), outWords, consumed);

    ASSERT_FALSE(ok);
    // No valid frame -> consumed set to len (caller may drop the whole buffer).
    ASSERT_EQ(consumed, buf.size());

    // Also exercise the "header too close to the tail to fit a frame" guard with a
    // correct-looking length but insufficient bytes — must not overrun either.
    std::vector<uint8_t> shortBuf = {VisionFrame::FRAME_HEADER, Config::VISION_FRAME_WORDS, 0x00};
    std::size_t consumed2 = 0;
    const bool ok2 = VisionFrame::parseFrame(shortBuf.data(), shortBuf.size(), outWords, consumed2);
    ASSERT_FALSE(ok2);
    ASSERT_EQ(consumed2, shortBuf.size());

    // Now actually drive the LENGTH-VALUE rejection path. A buffer >= kFrameBytes long clears the
    // too-short guard at i=0, so parseFrame READS the length byte (200), rejects it
    // (200 != VISION_FRAME_WORDS), and resyncs from i+1. The remaining bytes carry no 0xAA, so the
    // scan finds no later candidate header and returns false with consumed == len — and the tail
    // guard still bounds every candidate, so there is NO read past buf[len-1] (ASan-checked).
    const std::size_t kFullFrameLen = kFrameBytes;    // 1 + 1 + 2*VISION_FRAME_WORDS + 1
    std::vector<uint8_t> fullLenBuf(kFullFrameLen, 0x01);    // 0x01 != 0xAA -> no false-sync
    fullLenBuf[0] = VisionFrame::FRAME_HEADER;
    fullLenBuf[1] = 200;    // length value, now actually read and rejected
    std::size_t consumed3 = 0;
    const bool ok3 = VisionFrame::parseFrame(fullLenBuf.data(), fullLenBuf.size(), outWords,
                                             consumed3);
    ASSERT_FALSE(ok3);
    ASSERT_EQ(consumed3, fullLenBuf.size());
}

// ===========================================================================
// TC11 — valid header + length, WRONG XOR checksum -> not accepted.
// (AC5)
// ===========================================================================

static void test_TC11_parseframe_wrong_checksum_rejected()
{
    TEST_SCOPE("TC11: valid header/length, wrong XOR -> not accepted");

    const std::vector<uint8_t> frame = buildUniformFrame(kHueGreen, kStaHigh, /*corrupt=*/true);

    uint16_t outWords[Config::VISION_FRAME_WORDS];
    std::size_t consumed = 0;
    const bool ok = VisionFrame::parseFrame(frame.data(), frame.size(), outWords, consumed);

    ASSERT_FALSE(ok);
    // No accepted frame in the buffer -> consumed == len.
    ASSERT_EQ(consumed, frame.size());
}

// ===========================================================================
// TC12 — readMosaic happy path -> true; all 12 getColor + getRawHue match the frame.
// (AC1)
//
// Two valid frames delivered AFTER the flush (via the tick callback): the first is the
// discarded post-flush frame, the SECOND is latched. We make the two frames carry the SAME
// colours so the assertion is unambiguous regardless of which is latched, but set per-cell
// distinct hues so a wrong cell index would be caught.
// ===========================================================================

static void test_TC12_readmosaic_happy_path()
{
    TEST_SCOPE("TC12: readMosaic happy path -> true; 12 getColor + getRawHue match");

    resetWorld();

    // Per-cell hues cycling Y/G/B (all valid, no White/Black) so each cell classifies and a
    // transposed index would mismatch. Identity map is active in this TU? No — TC19's override
    // is compile-time global. So cell c reads slot (11 - c). Build the FRAME so that the
    // MAPPED slot for cell c carries cell c's intended colour. Simpler: make every slot the
    // same colour family per its index and derive the expected per-cell value via the map.
    const uint8_t palette[3] = {kHueYellow, kHueGreen, kHueBlue};

    uint8_t hue[Config::VISION_CELL_COUNT];
    uint8_t sta[Config::VISION_CELL_COUNT];
    for (uint8_t slot = 0; slot < Config::VISION_CELL_COUNT; ++slot)
    {
        hue[slot] = palette[slot % 3];
        sta[slot] = kStaHigh;
    }
    const std::vector<uint8_t> frame = buildFrame(hue, sta);

    // Stage TWO copies (discard 1, latch the 2nd). Both identical -> latch is deterministic.
    std::vector<uint8_t> twoFrames(frame);
    twoFrames.insert(twoFrames.end(), frame.begin(), frame.end());
    stageInjection(twoFrames);

    Vision.setTickCallback(injectTickCallback);
    const bool ok = Vision.readMosaic();

    ASSERT_TRUE(ok);
    ASSERT_FALSE(Vision.hasFault());

    // getColor(cell)/getRawHue(cell) reflect the MAPPED slot (cell -> slot (11 - cell)).
    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        const uint8_t slot = Config::VISION_CELL_TO_SLOT[cell];
        ASSERT_EQ(Vision.getRawHue(cell), hue[slot]);
        ASSERT_TRUE(Vision.getColor(cell) == VisionFrame::classifyHue(hue[slot], sta[slot]));
    }

    Vision.setTickCallback(nullptr);
}

// ===========================================================================
// TC13 — stale backlog: VISION_FLUSH_DISCARD_FRAMES + 1 STALE frames pre-fed (drip OFF ->
//        flushed away), then 2 FRESH frames delivered during the wait -> latches the FRESH
//        colours, not the stale. (AC7)
//
// Pre-feeding exactly ONE stale frame would NOT discriminate a broken (no-op) flush: that
// single stale frame would simply become the one post-flush discard, and the first fresh frame
// would still latch -> the fresh colour either way. To make the flush observable we pre-feed
// VISION_FLUSH_DISCARD_FRAMES + 1 = 2 stale frames. A working flush eats BOTH, then discards
// fresh frame #1 and latches fresh frame #2 -> BLUE. A broken flush discards only stale #1,
// latches stale #2 -> YELLOW, and this assertion FAILS. The +1 makes the stale count exceed
// the discard budget, so a surviving stale frame would reach the latch.
// ===========================================================================

static void test_TC13_readmosaic_latches_fresh_not_stale()
{
    TEST_SCOPE("TC13: stale backlog flushed; latches the fresh frame, not the stale one");

    resetWorld();

    // STALE frames: every cell YELLOW. Fed BEFORE readMosaic with drip OFF -> they sit in RX and
    // a working flush eats them at attempt start. Count = VISION_FLUSH_DISCARD_FRAMES + 1 so that
    // if the flush were broken at least one stale frame would survive the discard budget and
    // reach the latch (making the BLUE assertion below fail). Derived from Config, not hardcoded.
    const std::vector<uint8_t> staleFrame = buildUniformFrame(kHueYellow, kStaHigh);
    const std::size_t staleCount =
        static_cast<std::size_t>(Config::VISION_FLUSH_DISCARD_FRAMES) + 1u;
    for (std::size_t n = 0; n < staleCount; ++n)
    {
        mockSerial1Feed(staleFrame.data(), staleFrame.size());
    }

    // FRESH frames: every cell BLUE. Delivered AFTER the flush via the callback (2 copies:
    // discard 1, latch the 2nd).
    const std::vector<uint8_t> freshFrame = buildUniformFrame(kHueBlue, kStaHigh);
    std::vector<uint8_t> twoFresh(freshFrame);
    twoFresh.insert(twoFresh.end(), freshFrame.begin(), freshFrame.end());
    stageInjection(twoFresh);

    Vision.setTickCallback(injectTickCallback);
    const bool ok = Vision.readMosaic();

    ASSERT_TRUE(ok);
    ASSERT_FALSE(Vision.hasFault());

    // Latched colour is the FRESH (BLUE), proving the stale (YELLOW) backlog was flushed.
    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        ASSERT_TRUE(Vision.getColor(cell) == Config::Color::BLUE);
        ASSERT_EQ(Vision.getRawHue(cell), kHueBlue);
    }

    Vision.setTickCallback(nullptr);
}

// ===========================================================================
// TC14 — an active cell = Black(248) -> false, BAD_CELL, firstBadCell set,
//        prior latch unchanged.
// (AC4)
// ===========================================================================

static void test_TC14_readmosaic_bad_cell_preserves_latch()
{
    TEST_SCOPE("TC14: active cell Black(248) -> BAD_CELL, false, prior latch unchanged");

    resetWorld();

    // --- First, a GOOD read so there is a known prior latch (all GREEN). ---
    const std::vector<uint8_t> goodFrame = buildUniformFrame(kHueGreen, kStaHigh);
    std::vector<uint8_t> twoGood(goodFrame);
    twoGood.insert(twoGood.end(), goodFrame.begin(), goodFrame.end());
    stageInjection(twoGood);
    Vision.setTickCallback(injectTickCallback);
    ASSERT_TRUE(Vision.readMosaic());
    ASSERT_FALSE(Vision.hasFault());
    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        ASSERT_TRUE(Vision.getColor(cell) == Config::Color::GREEN);
    }

    // --- Now a frame where exactly one SLOT reads device Black(248). The cell whose map
    // points at that slot must trip the strict guard. With the (11 - cell) map, slot 5 is
    // observed by cell 6. ---
    uint8_t hue[Config::VISION_CELL_COUNT];
    uint8_t sta[Config::VISION_CELL_COUNT];
    for (uint8_t slot = 0; slot < Config::VISION_CELL_COUNT; ++slot)
    {
        hue[slot] = kHueGreen;
        sta[slot] = kStaHigh;
    }
    const uint8_t badSlot = 5;
    hue[badSlot] = kHueBlack;    // device Black -> classifyHue UNKNOWN -> BAD_CELL

    // Find which CELL maps to badSlot (the firstBadCell the guard reports). The guard scans
    // cells 0..N ascending and reports the FIRST cell that maps to an UNKNOWN slot.
    uint8_t expectBadCell = 0xFF;
    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        if (Config::VISION_CELL_TO_SLOT[cell] == badSlot)
        {
            expectBadCell = cell;
            break;
        }
    }

    const std::vector<uint8_t> badFrame = buildFrame(hue, sta);
    std::vector<uint8_t> twoBad(badFrame);
    twoBad.insert(twoBad.end(), badFrame.begin(), badFrame.end());
    stageInjection(twoBad);
    Vision.setTickCallback(injectTickCallback);

    const bool ok = Vision.readMosaic();

    ASSERT_FALSE(ok);
    ASSERT_TRUE(Vision.hasFault());
    const VisionClass::FaultInfo f = Vision.getFault();
    ASSERT_TRUE(f.kind == VisionClass::Fault::BAD_CELL);
    ASSERT_EQ(f.firstBadCell, expectBadCell);
    ASSERT_EQ(f.rawValue, kHueBlack);    // raw hue of the offending mapped slot

    // Prior latch (all GREEN) is unchanged — the failed read did NOT overwrite it.
    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        ASSERT_TRUE(Vision.getColor(cell) == Config::Color::GREEN);
        ASSERT_EQ(Vision.getRawHue(cell), kHueGreen);
    }

    Vision.setTickCallback(nullptr);
}

// ===========================================================================
// TC15 — bad-checksum-then-good (retry once) -> true, no fault.
// (AC6)
//
// Attempt 1 delivers a corrupt frame-sized blob (>= one frame's bytes, bad XOR ->
// framesParsed == 0). Since the attempt's deadline elapses with no candidate, readMosaic
// retries. Attempt 2 (after its own flush) delivers 2 good frames -> success, no fault.
//
// Delivery (DETERMINISTIC, millis-gated): the wait-loop callback delivers the corrupt blob
// once while the mock clock is still inside attempt 1's window (millis < VISION_READ_TIMEOUT_MS),
// and the two good frames once the clock has crossed into attempt 2's window
// (millis >= VISION_READ_TIMEOUT_MS). The per-attempt flush at the top of readMosaic() drains
// any leftover corrupt bytes before attempt 2 reads, so only the good frames are parsed in
// attempt 2. millis is monotonic and advanced only by the wait loop's own delay(), so the two
// windows never overlap — no fragile RX-empty heuristics.
//
// Window math: attempt 1 deadline = millis()(==0) + 120 = 120; it breaks at millis >= 120.
// Attempt 2 then computes deadline = millis()(~120) + 120 = ~240. So "millis < 120" uniquely
// identifies attempt 1 and "millis >= 120" attempt 2.
// ===========================================================================

namespace
{
    std::vector<uint8_t> g_tc15Corrupt;       // attempt-1 payload (corrupt blob)
    std::vector<uint8_t> g_tc15Good;          // attempt-2 payload (two good frames)
    bool                 g_tc15CorruptFed = false;
    bool                 g_tc15GoodFed    = false;

    void tc15TickCallback()
    {
        const uint32_t now = millis();
        if (now < Config::VISION_READ_TIMEOUT_MS)
        {
            // Attempt-1 window: feed the corrupt blob exactly once.
            if (!g_tc15CorruptFed && !g_tc15Corrupt.empty())
            {
                mockSerial1Feed(g_tc15Corrupt.data(), g_tc15Corrupt.size());
                g_tc15CorruptFed = true;
            }
        }
        else
        {
            // Attempt-2 window: feed the two good frames exactly once.
            if (!g_tc15GoodFed && !g_tc15Good.empty())
            {
                mockSerial1Feed(g_tc15Good.data(), g_tc15Good.size());
                g_tc15GoodFed = true;
            }
        }
    }
}    // namespace

static void test_TC15_bad_checksum_then_good_retries()
{
    TEST_SCOPE("TC15: bad-checksum first attempt, good second -> true, no fault (retry)");

    resetWorld();

    // Attempt-1 payload: a corrupt frame-sized blob (valid framing, wrong XOR) -> framesParsed
    // == 0, attempt 1 ends in a deadline miss -> retry.
    g_tc15Corrupt    = buildUniformFrame(kHueBlue, kStaHigh, /*corrupt=*/true);
    g_tc15CorruptFed = false;

    // Attempt-2 payload: two GOOD frames (discard 1, latch the 2nd) -> attempt 2 succeeds.
    const std::vector<uint8_t> good = buildUniformFrame(kHueBlue, kStaHigh);
    g_tc15Good = good;
    g_tc15Good.insert(g_tc15Good.end(), good.begin(), good.end());
    g_tc15GoodFed = false;

    Vision.setTickCallback(tc15TickCallback);
    const bool ok = Vision.readMosaic();

    ASSERT_TRUE(ok);
    ASSERT_FALSE(Vision.hasFault());
    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        ASSERT_TRUE(Vision.getColor(cell) == Config::Color::BLUE);
    }

    Vision.setTickCallback(nullptr);
}

// ===========================================================================
// TC16 — persistent timeout: drip-feed bytes that NEVER complete a frame on BOTH
//        attempts -> false, READ_TIMEOUT, hasFault/clearFault, loop terminates.
// (AC6)
//
// We pre-queue FEWER than one frame's bytes into the drip-held pending queue and drip 1
// byte/tick. They release per delay() tick but never form a full frame, so parseFrame never
// accepts. With totalBytesSeen < kFrameBytes and framesParsed == 0 the disambiguation lands
// on READ_TIMEOUT (per the Vision.cpp Step-5 rule). The loop terminates because delay()
// advances millis() to each attempt's deadline.
// ===========================================================================

static void test_TC16_persistent_timeout()
{
    TEST_SCOPE("TC16: drip-feed never completes a frame -> READ_TIMEOUT, terminates");

    resetWorld();

    // Enable drip-feed at 1 byte/tick, then queue a partial frame (header + a few bytes,
    // strictly fewer than kFrameBytes). With drip ON, feed() lands them in _pending; they
    // release one per delay() tick but can never form a full frame.
    mockSerial1SetDripFeed(1);
    std::vector<uint8_t> partial = {VisionFrame::FRAME_HEADER, Config::VISION_FRAME_WORDS,
                                    0x01, 0x02, 0x03};
    mockSerial1Feed(partial.data(), partial.size());

    // No tick callback needed — the bare bounded wait + delay() drip pump advances millis()
    // and releases the (insufficient) bytes. The loop MUST terminate at the deadline.
    const bool ok = Vision.readMosaic();

    ASSERT_FALSE(ok);
    ASSERT_TRUE(Vision.hasFault());
    ASSERT_TRUE(Vision.getFault().kind == VisionClass::Fault::READ_TIMEOUT);

    // clearFault() resets the fault record.
    Vision.clearFault();
    ASSERT_FALSE(Vision.hasFault());
    ASSERT_TRUE(Vision.getFault().kind == VisionClass::Fault::NONE);

    mockSerial1SetDripFeed(0);    // restore default for later tests
}

// ===========================================================================
// TC17 — getColor/getRawHue before any read, and out-of-range index -> safe sentinel,
//        no crash.
// (AC3)
// ===========================================================================

static void test_TC17_getters_pre_read_and_out_of_range()
{
    TEST_SCOPE("TC17: pre-read + out-of-range getters -> UNKNOWN / 0, no crash");

    resetWorld();    // begin() clears _hasRead -> pre-read state

    // Pre-read: every in-range cell is the safe sentinel.
    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        ASSERT_TRUE(Vision.getColor(cell) == Config::Color::UNKNOWN);
        ASSERT_EQ(Vision.getRawHue(cell), Config::VISION_RAW_HUE_NONE);
    }

    // Out-of-range indices (== count, and well past) -> sentinel, no OOB access (ASan-clean).
    ASSERT_TRUE(Vision.getColor(Config::VISION_CELL_COUNT) == Config::Color::UNKNOWN);
    ASSERT_EQ(Vision.getRawHue(Config::VISION_CELL_COUNT), Config::VISION_RAW_HUE_NONE);
    ASSERT_TRUE(Vision.getColor(200) == Config::Color::UNKNOWN);
    ASSERT_EQ(Vision.getRawHue(255), Config::VISION_RAW_HUE_NONE);
}

// ===========================================================================
// T14 (#154-18) — getRawHue() no-data sentinel: named Config::VISION_RAW_HUE_NONE (0xFF),
// distinct from a genuine hue-0 reading, returned for both the pre-read and out-of-range-cell
// cases (TC17 already drives those two paths against the sentinel above; this test additionally
// pins the sentinel's VALUE and proves it cannot be confused with an actual hue-0 cell).
// ===========================================================================

static void test_T14_getRawHue_sentinel_distinct_from_hue_zero()
{
    TEST_SCOPE("T14 (#154-18): getRawHue sentinel distinct from hue 0, returned on no-read");

    // The sentinel itself must never equal a real hue-0 reading.
    ASSERT_EQ(Config::VISION_RAW_HUE_NONE, (uint8_t) 0xFF);
    ASSERT_TRUE(Config::VISION_RAW_HUE_NONE != (uint8_t) 0);

    resetWorld();    // begin() clears _hasRead -> pre-read state, no fault

    // Pre-read (no successful readMosaic() yet): every cell reads the sentinel, not 0.
    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        ASSERT_EQ(Vision.getRawHue(cell), Config::VISION_RAW_HUE_NONE);
    }

    // A genuine successful read with an actual hue-0-adjacent chromatic value must NOT be
    // confused with the sentinel: latch a real frame, then confirm the in-range cells read
    // back their real (non-sentinel) hue while an out-of-range index still reads the sentinel.
    const std::vector<uint8_t> frame = buildUniformFrame(kHueGreen, kStaHigh);
    std::vector<uint8_t> twoFrames(frame);
    twoFrames.insert(twoFrames.end(), frame.begin(), frame.end());
    stageInjection(twoFrames);
    Vision.setTickCallback(injectTickCallback);
    ASSERT_TRUE(Vision.readMosaic());
    Vision.setTickCallback(nullptr);

    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        ASSERT_EQ(Vision.getRawHue(cell), kHueGreen);
        ASSERT_TRUE(Vision.getRawHue(cell) != Config::VISION_RAW_HUE_NONE);
    }

    // Out-of-range cell index still returns the sentinel even after a successful read.
    ASSERT_EQ(Vision.getRawHue(Config::VISION_CELL_COUNT), Config::VISION_RAW_HUE_NONE);
    ASSERT_EQ(Vision.getRawHue(255), Config::VISION_RAW_HUE_NONE);
}

// ===========================================================================
// TC18 — tick callback registered + drip-fed read -> mockTickCount > 0 during the wait.
// (Plan: AC6/AC7 support; proves the wait loop iterated.)
//
// Drip-feed the two good frames byte-by-byte so the read MUST spin the wait loop (each
// delay() tick releases one byte and bumps mockTickCount). The callback is registered so the
// loop calls it every iteration; after the read returns true, the tick counter is > 0.
// ===========================================================================

static void test_TC18_tick_callback_invoked_during_wait()
{
    TEST_SCOPE("TC18: drip-fed read spins the wait loop -> mockTickCount > 0");

    resetWorld();

    // Build 2 good frames and queue them ALL into the drip-held pending queue, released 4
    // bytes/tick so the read needs many ticks to assemble both frames -> the wait loop spins.
    const std::vector<uint8_t> good = buildUniformFrame(kHueGreen, kStaHigh);
    std::vector<uint8_t> twoGood(good);
    twoGood.insert(twoGood.end(), good.begin(), good.end());

    mockSerial1SetDripFeed(4);
    mockSerial1Feed(twoGood.data(), twoGood.size());

    // A do-nothing tick callback (the drip pump runs inside delay(), independent of the
    // callback). Registering it proves the callback path is exercised without freezing.
    Vision.setTickCallback([]() { /* no-op: drip pump runs in delay() */ });

    mockResetTickCount();
    const bool ok = Vision.readMosaic();

    ASSERT_TRUE(ok);
    ASSERT_FALSE(Vision.hasFault());
    // The wait loop ran at least one delay() tick to release the drip bytes.
    ASSERT_GT(mockTickCount, (uint32_t) 0);

    Vision.setTickCallback(nullptr);
    mockSerial1SetDripFeed(0);
}

// ===========================================================================
// TC19 — non-identity VISION_CELL_TO_SLOT -> getColor(cell) returns the MAPPED slot's
//        colour (proves cell->slot direction; catches a transposed map).
// (AC3)
//
// The TU-wide override maps cell c -> slot (11 - c). We build a frame where each SLOT has a
// DISTINCT colour family, then assert getColor(cell) equals the classification of the slot
// VISION_CELL_TO_SLOT[cell] — NOT slot==cell. A transposed implementation (indexing the
// frame by cell) would return the wrong colour for every non-fixed-point cell.
// ===========================================================================

static void test_TC19_cell_to_slot_direction()
{
    TEST_SCOPE("TC19: non-identity cell->slot map -> getColor reads the mapped slot");

    resetWorld();

    // Per-SLOT colours: slot s gets palette[s % 3] (all chromatic & valid). Distinct enough
    // that the (11 - c) permutation produces a different per-cell pattern than identity.
    const uint8_t palette[3] = {kHueYellow, kHueGreen, kHueBlue};
    uint8_t hue[Config::VISION_CELL_COUNT];
    uint8_t sta[Config::VISION_CELL_COUNT];
    for (uint8_t slot = 0; slot < Config::VISION_CELL_COUNT; ++slot)
    {
        hue[slot] = palette[slot % 3];
        sta[slot] = kStaHigh;
    }
    const std::vector<uint8_t> frame = buildFrame(hue, sta);
    std::vector<uint8_t> twoFrames(frame);
    twoFrames.insert(twoFrames.end(), frame.begin(), frame.end());
    stageInjection(twoFrames);

    Vision.setTickCallback(injectTickCallback);
    const bool ok = Vision.readMosaic();
    ASSERT_TRUE(ok);

    // Direction proof: for each cell, the latched value equals the MAPPED slot's value.
    // Also assert at least one cell differs from the identity expectation so the test would
    // FAIL against a transposed map (cell != mapped slot for all but the fixed point — there
    // is no fixed point in the (11 - c) permutation, so every cell differs).
    bool anyDiffersFromIdentity = false;
    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        const uint8_t mappedSlot = Config::VISION_CELL_TO_SLOT[cell];
        ASSERT_EQ(Vision.getRawHue(cell), hue[mappedSlot]);
        ASSERT_TRUE(Vision.getColor(cell)
                    == VisionFrame::classifyHue(hue[mappedSlot], sta[mappedSlot]));
        if (hue[mappedSlot] != hue[cell])
        {
            anyDiffersFromIdentity = true;
        }
    }
    ASSERT_TRUE(anyDiffersFromIdentity);

    Vision.setTickCallback(nullptr);
}

// ===========================================================================
// TC20 — read with NO tick callback registered, across TWO reads -> the one-time Serial
//        warning appears EXACTLY once.
// (AC6)
//
// With no callback the wait loop emits a one-time warning (static-bool guarded in Vision.cpp)
// then does a bare bounded wait. We drive TWO reads with no frames delivered (drip OFF, RX
// empty) so each read spins its wait loop and hits the warning path; the static guard means
// the captured USB Serial buffer contains the warning substring EXACTLY once across both.
// Both reads time out (no frames) — expected; we only assert on the warning count.
//
// ORDERING: the warning's static-bool guard is PROCESS-GLOBAL and one-shot, so this test must
// run before ANY other no-callback readMosaic (see the note in main()). If TC16 ran first the
// warning would already be latched and this count would be 0.
// ===========================================================================

static void test_TC20_no_callback_warns_once()
{
    TEST_SCOPE("TC20: no tick callback across two reads -> one-time warning exactly once");

    resetWorld();
    Vision.setTickCallback(nullptr);    // explicit: no callback

    // No frames delivered: both reads time out, each spinning the warning-emitting wait loop.
    Serial.clearCapture();
    (void) Vision.readMosaic();         // 1st read — emits the warning (first time)
    (void) Vision.readMosaic();         // 2nd read — warning already latched, must NOT repeat

    // Count occurrences of the warning's stable substring in the captured USB output.
    const std::string& cap = Serial.captured();
    const char* needle = "no tick callback set";
    std::size_t count = 0;
    std::size_t pos = cap.find(needle, 0);
    while (pos != std::string::npos)
    {
        ++count;
        pos = cap.find(needle, pos + 1);
    }
    ASSERT_EQ(count, (std::size_t) 1);
}

// ===========================================================================
// TC21 — classifyHue precedence: hue 248 (device Black) with sta 1 (below the white gate)
//        -> UNKNOWN (bad device code checked BEFORE the STA-gate, not WHITE).
// (AC2)
// ===========================================================================

static void test_TC21_classify_bad_code_before_sta_gate()
{
    TEST_SCOPE("TC21: hue 248 + sta 1 -> UNKNOWN (bad-code precedes STA-gate)");

    // sta 1 < VISION_WHITE_STA_MAX(3): a naive STA-gate-first impl would return WHITE.
    // The fixed order checks device Black (246..250) FIRST, so the result is UNKNOWN.
    ASSERT_TRUE(VisionFrame::classifyHue(248, 1) == Config::Color::UNKNOWN);
    // Same precedence for the device NONE code.
    ASSERT_TRUE(VisionFrame::classifyHue(255, 1) == Config::Color::UNKNOWN);

    // Reserved codes 251..254 are undefined: 240..245 (White), 246..250 (Black) and 255 (None)
    // are handled by earlier steps, so a code in 251..254 must NOT leak to WHITE through the
    // step-4 sta gate. It returns UNKNOWN regardless of saturation — both below the white gate
    // (sta 1) and above it (sta 15).
    ASSERT_TRUE(VisionFrame::classifyHue(252, 1) == Config::Color::UNKNOWN);
    ASSERT_TRUE(VisionFrame::classifyHue(252, 15) == Config::Color::UNKNOWN);
}

// ===========================================================================
// Test entry point.
// ===========================================================================

int main()
{
    // Unit — pure VisionFrame functions.
    test_TC1_classify_band_midpoints();
    test_TC2_classify_white_band();
    test_TC3_classify_low_sta_chromatic_is_white();
    test_TC4_classify_none_and_black_unknown();
    test_TC5_classify_band_boundaries_and_gap();
    test_TC6_unpackframe_exact();
    test_TC7_parseframe_clean_valid();
    test_TC8_parseframe_resyncs_after_garbage();
    test_TC9_parseframe_no_false_sync_on_aa_data();
    test_TC10_parseframe_corrupt_length_no_overrun();
    test_TC11_parseframe_wrong_checksum_rejected();

    // Integration — readMosaic over the mock Serial1.
    //
    // ORDERING NOTE: TC20 asserts the no-tick-callback warning fires EXACTLY once. That
    // warning is gated by a PROCESS-GLOBAL static-bool in Vision.cpp (it latches on the first
    // no-callback wait). TC16 also runs readMosaic WITHOUT a callback, so it would trip and
    // latch that warning first — leaving TC20 to capture zero occurrences. TC20 must therefore
    // run BEFORE TC16 (and before any other no-callback readMosaic). Every other integration
    // test here registers a non-null callback, so none of them touch the warning path.
    test_TC12_readmosaic_happy_path();
    test_TC13_readmosaic_latches_fresh_not_stale();
    test_TC14_readmosaic_bad_cell_preserves_latch();
    test_TC15_bad_checksum_then_good_retries();
    test_TC17_getters_pre_read_and_out_of_range();
    test_T14_getRawHue_sentinel_distinct_from_hue_zero();
    test_TC18_tick_callback_invoked_during_wait();
    test_TC19_cell_to_slot_direction();
    test_TC20_no_callback_warns_once();    // MUST precede TC16 (shared one-time-warning static)
    test_TC16_persistent_timeout();

    // Unit — precedence.
    test_TC21_classify_bad_code_before_sta_gate();

    TEST_REPORT();
}
