#ifndef CONFIG_VISION_H
#define CONFIG_VISION_H

#include <stdint.h>

/**
 * @brief Vision subsystem configuration — reopens `namespace Config` per ADR-031.
 *
 * Holds the HUE12 camera constants, hue-band calibration table, and mosaic
 * cell-to-slot mapping for the WRO 2026 colour-identification task. Included
 * by `Config.h` after `ConfigManipulator.h` and before `ConfigChecks.h`, so
 * the static_assert validators in ConfigChecks.h see all symbols from this file.
 *
 * @note **Every numeric value here is a placeholder pending bench calibration.**
 *       Constants marked `TODO` must be measured on the physical HUE12 camera
 *       at hardware bring-up (bench task T8) before competition use.
 * @note `VISION_CELL_TO_SLOT` uses the `_DEFAULT` override idiom so the TC19
 *       compile-time test can inject a non-identity map without editing this file. It
 *       defines the scalar `VISION_CELL_TO_SLOT_REVERSE_FOR_TEST` (not a brace-init
 *       `-DVISION_CELL_TO_SLOT_DEFAULT={...}`, which macOS `/bin/sh` brace-expands and
 *       corrupts); see the mapping's docblock below. Mirrors `Config.h`'s
 *       `PATH_PASS_THROUGH_ENABLED` and `ConfigManipulator.h`'s `SORTER_RC_ID` / `BELT_M_ID`.
 */

/**
 * @brief The `HardwareSerial` port the HUE12 camera is wired to.
 *
 * Bench-measured 2026-07: the camera streams valid frames (`0xAA` header, length 9, checksum)
 * on **`Serial1` at 115200 baud** (see `VISION_UART_BAUD`). A dual-UART baud-sweep probe saw a
 * steady ~16 fps of `0xAA`-headed frames on `Serial1` and NOTHING on `Serial2`. This matches the
 * vendor driver `MiniR4SmartCamReader`, whose `Begin()` does `Serial1.begin(115200)`.
 *
 * On the MATRIX Mini R4 the physical "UART" socket is routed to `Serial1`. The D0/D1 header
 * socket (`Serial2` = UART2 = pins 1/0) is NOT the camera and reads dead-silent. Do NOT change
 * this to `Serial2`: an earlier commit did exactly that — inferring the port from `MiniR4.Uart`
 * instead of measuring — and it broke the read. Selecting the port through this one macro keeps
 * `Vision.cpp` and `tools/vision_bench/vision_bench.ino` on a single source of truth.
 *
 * @note Guarded with `#ifndef` so a host/bench build can override the port from the command line.
 *       The host tests feed the mock `Serial1` directly (`mockSerial1Feed`), which this macro
 *       already selects — no alias needed.
 */
#ifndef VISION_SERIAL
#define VISION_SERIAL Serial1
#endif

namespace Config
{
    // -----------------------------------------------------------------------------------------
    // HUE12 UART contract
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Total number of hardware detection slots exported by the HUE12 camera (S1..S12).
     *
     * This is a hardware constant defined by the `.mbr4` contract program; do NOT change it
     * unless the camera firmware is reprogrammed. A frame from the HUE12 contains exactly
     * `VISION_SLOT_COUNT` slot readings.
     */
    constexpr uint8_t VISION_SLOT_COUNT = 12;   // HUE12 S1..S12 — hardware contract

    /**
     * @brief Number of active game-mosaic cells the robot must identify.
     *
     * The WRO 2026 mosaic is a 4×3 grid = 12 cells (confirmed 2026-06-13). Kept as its own
     * named constant (separate from `VISION_SLOT_COUNT`) so cell-count uses read clearly, but
     * the two are currently pinned together: `ConfigChecks.h` static_asserts
     * `VISION_CELL_COUNT == VISION_SLOT_COUNT`, an even count, and `VISION_FRAME_WORDS` derived
     * from it. A real layout change therefore means reprogramming the camera and updating those
     * coupled constants in lock-step — not a one-line edit to this value alone.
     */
    constexpr uint8_t VISION_CELL_COUNT = 12;   // 4×3 mosaic

    /**
     * @brief Number of packed 16-bit words per HUE12 UART frame.
     *
     * @note This is NOT the slot count or cell count — it is the wire framing parameter
     *       that determines when a complete frame has been received. Bench-confirmed: the wire
     *       shows `AA 09 ...` — an `0xAA` header followed by a length byte of 9.
     */
    constexpr uint8_t VISION_FRAME_WORDS = 9;   // packed words per frame; bench-confirmed (AA 09)

    /**
     * @brief UART baud rate for the HUE12 camera link.
     *
     * Bench-confirmed 2026-07 at 115200 — matches the vendor `MiniR4SmartCamReader::Begin()`
     * (`Serial1.begin(115200)`). The camera streamed clean framed data only at this baud; 9600
     * (the former placeholder) produced framing garbage.
     */
    constexpr uint32_t VISION_UART_BAUD = 115200;   // vendor SmartCamReader baud; bench-confirmed

    /**
     * @brief Per-read-attempt timeout in milliseconds for the HUE12 UART reader.
     *
     * Must be at least `(VISION_FLUSH_DISCARD_FRAMES + 1)` emission periods long to guarantee at
     * least one fresh frame is available after flush. Bench-measured rate is ~16 fps (~63 ms /
     * frame), so DISCARD+1 = 2 frames ≈ 125 ms; 250 ms leaves comfortable margin (and the reader
     * retries once on top of this).
     */
    constexpr uint16_t VISION_READ_TIMEOUT_MS = 250;   // >= (DISCARD+1) frame periods @ ~16 fps

    /**
     * @brief Number of frames discarded after a UART flush before latching the mosaic result.
     *
     * Dropping stale frames after flush ensures the latched snapshot was captured with the
     * camera in the current field position, not before the robot arrived.
     */
    constexpr uint8_t VISION_FLUSH_DISCARD_FRAMES = 1;   // frames dropped after flush before latch

    /**
     * @brief Wake / start command string written to the HUE12 over Serial1.
     *
     * The HUE12 is from the same device family as the vendor `MiniR4SmartCamReader`, whose
     * `SmartCamReader()` writes `Serial1.write("start")` whenever it has received no valid data
     * (`MiniR4SmartCamReader.h`: the `else { if (!hasReceivedValidData) Serial1.write("start"); }`
     * branch). On a cold boot the camera may sit idle until it is kicked with this command, so the
     * firmware sends it once from `Vision::begin()` and on demand via `Vision::start()`.
     *
     * @note Stored as a named constant so the wire string lives in exactly one place (no magic
     *       strings); mirrors the vendor contract verbatim.
     */
    constexpr char VISION_START_CMD[] = "start";   // HUE12 wake command — vendor SmartCamReader contract

    // -----------------------------------------------------------------------------------------
    // Colour classification
    // -----------------------------------------------------------------------------------------

    /** @brief Identified mosaic cell colour. `UNKNOWN` means classification failed. */
    // `WHITE` collides with the Adafruit backwards-compat `#define WHITE SSD1306_WHITE`
    // in MiniR4OLED.h, which is active in any TU that includes <MatrixMiniR4.h> before
    // this header (e.g. WRO2026.ino, EventTrace.cpp). Guard only the declaration: save
    // and #undef the macro for the enum, then restore it so OLED code calling
    // setTextColor(WHITE) still compiles. No <MatrixMiniR4.h>-including TU names
    // Config::Color::WHITE, so restoring the macro afterwards is safe.
#pragma push_macro("WHITE")
#undef WHITE
    enum class Color : uint8_t
    {
        UNKNOWN = 0,
        YELLOW,
        BLUE,
        GREEN,
        WHITE,
        RED
    };
#pragma pop_macro("WHITE")

    /**
     * @brief Sentinel raw-hue byte `Vision::getRawHue()` returns for "no data" — before the
     *        first successful `readMosaic()`, or for an out-of-range cell index.
     *
     * @note Never collides with a genuinely latched hue. The raw-hue axis is 0..239
     *       (chromatic) / 240..245 (device White) / 246..250 (device Black) / 255 (device
     *       "no detection"). `VisionFrame::classifyHue()` maps device NONE (255) to
     *       `Color::UNKNOWN`, which trips `VisionClass::_classifyCandidate()`'s strict guard
     *       and rejects the WHOLE candidate frame before `_rawHue` is ever committed — so
     *       `0xFF` can never appear as a genuinely-latched value.
     */
    constexpr uint8_t VISION_RAW_HUE_NONE = 0xFF;   // getRawHue() no-data / out-of-range sentinel

    /**
     * @brief Saturation threshold below which a chromatic hue reading is re-classified as White.
     *
     * The HUE12 outputs a saturation (sta) value in the range [0..15]. When `sta < VISION_WHITE_STA_MAX`
     * the pixel is achromatic (grey / white) regardless of the hue value, and is classified as
     * `Color::WHITE`. Must be in [0..15]. TODO: calibrate at bench T8.
     */
    constexpr uint8_t VISION_WHITE_STA_MAX = 3;   // sta < this → White; TODO calibrate T8

    /**
     * @brief One entry in the chromatic hue-band lookup table.
     *
     * The HUE12 hue axis is 0..239 (chromatic range). Ranges are inclusive: a reading `h` maps
     * to `color` when `lo <= h && h <= hi`. Achromatic White (device side 240..245) is handled
     * separately in `classifyHue()` and does NOT appear in `HUE_BANDS`.
     */
    struct HueBand
    {
        uint8_t lo;      ///< Inclusive lower bound on hue (0..239).
        uint8_t hi;      ///< Inclusive upper bound on hue (0..239).
        Color   color;   ///< Colour assigned to a reading in [lo, hi].
    };

    /**
     * @brief Chromatic hue-band lookup table, scanned linearly for classification.
     *
     * Bands are placeholders — calibrate the boundary values at bench task T8 using the
     * actual WRO 2026 game tiles under competition lighting. Gaps between bands (e.g. between
     * YELLOW hi and GREEN lo) are classified as `Color::UNKNOWN` by `classifyHue()`.
     *
     * @note Device-side achromatic White (hue 240..245) is handled before this table is
     *       consulted, so `WHITE` does NOT need a band entry here.
     */
    // @note `Color::RED` is a valid classification target but has NO band yet:
    //       its hue wraps across the 0/239 seam, so its bounds must be measured
    //       on the real tiles at bench T8. Until then a red reading falls in no
    //       band and classifies as `Color::UNKNOWN` (same as any other gap).
    constexpr HueBand HUE_BANDS[] = {
        {  20,  50, Color::YELLOW },   // bench-tuned
        {  70, 140, Color::GREEN  },   // bench-tuned (green dips to ~79; lo lowered off 80)
        { 141, 200, Color::BLUE   },   // placeholder — calibrate
    };

    // -----------------------------------------------------------------------------------------
    // UNKNOWN-cell masking (mosaic robustness)
    // -----------------------------------------------------------------------------------------

    /**
     * @brief When true, a mosaic read TOLERATES a few UNKNOWN cells instead of failing outright,
     *        and the mosaic planner injects `VISION_MASK_FALLBACK_COLOR` for each UNKNOWN cell.
     *
     * Scoring rationale: a tile placed in ANY bin scores (5 pts) versus 0 for an un-built cell, so
     * masking a handful of unreadable / surprise-colour cells and placing a fallback tile is a net
     * gain. Vision still REPORTS the cell as `UNKNOWN` (`getColor()` / VISION VIEW stay honest, so
     * hue tuning is not hidden); only the mosaic sort substitutes the fallback. Leave `false` for
     * the strict default contract — any UNKNOWN cell => `Fault::BAD_CELL` (ADR-037).
     *
     * @note Uses the `_DEFAULT` override idiom (like `VISION_CELL_TO_SLOT`) so a host-test target
     *       or a bench build can force it on with `-DVISION_MASK_UNKNOWN_ENABLED_DEFAULT=true`
     *       without editing this file.
     */
#ifndef VISION_MASK_UNKNOWN_ENABLED_DEFAULT
#define VISION_MASK_UNKNOWN_ENABLED_DEFAULT false
#endif
    constexpr bool VISION_MASK_UNKNOWN_ENABLED = VISION_MASK_UNKNOWN_ENABLED_DEFAULT;

    /**
     * @brief Colour the mosaic planner substitutes for an UNKNOWN cell while masking is on.
     *
     * MUST be one of the four wall-supplied station colours (YELLOW / BLUE / GREEN / WHITE) so a
     * real tile can actually be fetched for it — enforced in `ConfigChecks.h`.
     *
     * @note `WHITE` collides with MiniR4OLED.h's `#define WHITE SSD1306_WHITE` in any TU that
     *       includes `<MatrixMiniR4.h>` before this header, so guard the reference to the
     *       enumerator by name exactly as the `enum class Color` declaration above does.
     */
#pragma push_macro("WHITE")
#undef WHITE
    constexpr Color VISION_MASK_FALLBACK_COLOR = Color::WHITE;   // must be a wall station colour
#pragma pop_macro("WHITE")

    /**
     * @brief Maximum UNKNOWN cells a masked read tolerates. Above this the frame is a genuine
     *        failure (`Fault::BAD_CELL`), not masked — a mostly-unknown board means the camera or
     *        lighting is wrong, not that one or two tiles are surprises.
     */
    constexpr uint8_t VISION_MASK_MAX_UNKNOWN_CELLS = 2;   // above this => real failure

    // -----------------------------------------------------------------------------------------
    // Mosaic cell-to-slot mapping
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Maps each active mosaic cell index [0..VISION_CELL_COUNT-1] to the HUE12 slot
     *        (0-based, range 0..VISION_SLOT_COUNT-1) that observes it.
     *
     * The identity mapping `{0,1,…,11}` is a placeholder — the real mapping must be taught
     * at bench bring-up by pointing each cell at the camera and recording which slot index
     * the HUE12 reports. Slots not referenced here are unused and excluded from framing guards.
     *
     * @note **TC19 override hook:** the initializer is guarded by
     *       `#ifndef VISION_CELL_TO_SLOT_DEFAULT`. The host test does not pass the brace-init
     *       list on the command line — `-DVISION_CELL_TO_SLOT_DEFAULT={...}` is unquoted and
     *       macOS `/bin/sh` brace-expands the `{a,b,c}`, corrupting the value. Instead it defines
     *       the scalar `VISION_CELL_TO_SLOT_REVERSE_FOR_TEST`, which selects the reversed
     *       (self-inverse) mapping below with no brace/comma on the command line. A direct
     *       `-DVISION_CELL_TO_SLOT_DEFAULT={...}` still wins where the shell does not brace-expand.
     *       Mirrors the `PATH_PASS_THROUGH_ENABLED_DEFAULT` / `SORTER_RC_ID_DEFAULT` idiom.
     */
#ifndef VISION_CELL_TO_SLOT_DEFAULT
#ifdef VISION_CELL_TO_SLOT_REVERSE_FOR_TEST
#define VISION_CELL_TO_SLOT_DEFAULT \
    { 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 }
#else
#define VISION_CELL_TO_SLOT_DEFAULT \
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
#endif
#endif
    constexpr uint8_t VISION_CELL_TO_SLOT[VISION_CELL_COUNT] = VISION_CELL_TO_SLOT_DEFAULT;

}    // namespace Config

#endif    // CONFIG_VISION_H
