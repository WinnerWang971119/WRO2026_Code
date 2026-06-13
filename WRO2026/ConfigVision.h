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
 *       compile-time test can inject a collision via `-DVISION_CELL_TO_SLOT_DEFAULT=...`
 *       without editing this file (mirrors `Config.h`'s `PATH_PASS_THROUGH_ENABLED`
 *       and `ConfigManipulator.h`'s `SORTER_RC_ID` / `BELT_M_ID` idiom).
 */
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
     * The WRO 2026 mosaic is a 4×3 grid = 12 cells (confirmed 2026-06-13). Split from
     * `VISION_SLOT_COUNT` so a layout change is a one-line edit to this constant only.
     */
    constexpr uint8_t VISION_CELL_COUNT = 12;   // 4×3 mosaic

    /**
     * @brief Number of packed 16-bit words per HUE12 UART frame.
     *
     * @note This is NOT the slot count or cell count — it is the wire framing parameter
     *       that determines when a complete frame has been received. TODO: verify at bench T8.
     */
    constexpr uint8_t VISION_FRAME_WORDS = 9;   // packed words per frame; TODO verify T8

    /**
     * @brief UART baud rate for the HUE12 camera link.
     *
     * Must match the `.mbr4` contract program's configured baud. TODO: bench-verify at T8.
     */
    constexpr uint32_t VISION_UART_BAUD = 9600;   // contract-program baud; TODO bench-verify T8

    /**
     * @brief Per-read-attempt timeout in milliseconds for the HUE12 UART reader.
     *
     * Must be at least `(VISION_FLUSH_DISCARD_FRAMES + 1)` emission periods long to
     * guarantee at least one fresh frame is available after flush. TODO: tune at bench T8.
     */
    constexpr uint16_t VISION_READ_TIMEOUT_MS = 120;   // >= (DISCARD+1) emission periods; TODO T8

    /**
     * @brief Number of frames discarded after a UART flush before latching the mosaic result.
     *
     * Dropping stale frames after flush ensures the latched snapshot was captured with the
     * camera in the current field position, not before the robot arrived.
     */
    constexpr uint8_t VISION_FLUSH_DISCARD_FRAMES = 1;   // frames dropped after flush before latch

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
        WHITE
    };
#pragma pop_macro("WHITE")

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
    constexpr HueBand HUE_BANDS[] = {
        {  20,  45, Color::YELLOW },   // placeholder — calibrate at T8
        {  80, 140, Color::GREEN  },   // placeholder — calibrate at T8
        { 141, 200, Color::BLUE   },   // placeholder — calibrate at T8
    };

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
     *       `#ifndef VISION_CELL_TO_SLOT_DEFAULT` so a compile-time test can force a slot
     *       collision via `-DVISION_CELL_TO_SLOT_DEFAULT={...}` without editing this file.
     *       This mirrors the `PATH_PASS_THROUGH_ENABLED_DEFAULT` / `SORTER_RC_ID_DEFAULT`
     *       idiom used elsewhere in Config.
     */
#ifndef VISION_CELL_TO_SLOT_DEFAULT
#define VISION_CELL_TO_SLOT_DEFAULT \
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
#endif
    constexpr uint8_t VISION_CELL_TO_SLOT[VISION_CELL_COUNT] = VISION_CELL_TO_SLOT_DEFAULT;

}    // namespace Config

#endif    // CONFIG_VISION_H
