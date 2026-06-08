/**
 * @file MatrixMiniR4.h (CTPS HOST MOCK)
 * @brief No-op recording mock of the MatrixMiniR4 facade for the CTPS dump.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `<MatrixMiniR4.h>` (angle-included, so it is shadowed by
 * include-path order — the dump's only include dir is `ctps_mocks/`). The
 * include guard is the REAL one (`MATRIXMINIR4_H`) so that if the real header
 * is ever reached after this one, it no-ops.
 *
 * Exposes the `MiniR4` global with EXACTLY the members the real Mission.cpp /
 * EventTrace reference, all as no-ops:
 *   - `MiniR4.LED.setColor(idx, r, g, b)`  — NO-OP (LED steps are recorded by
 *                                            the Mission.cpp `#ifdef CTPS_RECORD`
 *                                            hook, not here).
 *   - `MiniR4.Buzzer.Tone(freq, ms)`       — NO-OP (BEEP steps recorded by the
 *                                            Mission hook).
 *   - `MiniR4.BTN_UP.getState()`  → false  — no button is ever pressed in the
 *   - `MiniR4.BTN_DOWN.getState()`→ false    headless dump.
 *   - `MiniR4.OLED.*`                       — NO-OP (EventTrace is mocked no-op,
 *                                            so the OLED is barely touched).
 *   - `MiniR4.PWR.setBattCell(cell)` → true — battery config is a no-op.
 *   - `MiniR4.begin()`               → true — facade bring-up is a no-op.
 *
 * Member names / shapes mirror the real `MatrixMiniR4.h` so call sites compile
 * unchanged.
 */

#ifndef MATRIXMINIR4_H
#define MATRIXMINIR4_H

#include <stdint.h>

/**
 * @brief No-op mock of the on-board RGB LED pair (`MiniR4.LED`).
 *
 * `setColor` mirrors the real `MiniR4LED::setColor(idx, r, g, b)` signature and
 * return type but does nothing — LED steps are captured by the Mission.cpp
 * recording hook.
 */
class CtpsMockLED
{
public:
    /** @brief No-op color set; always reports success. */
    bool setColor(uint8_t /*idx*/, uint8_t /*r*/, uint8_t /*g*/, uint8_t /*b*/) { return true; }

    /** @brief No-op packed-color set; always reports success. */
    bool setColor(uint8_t /*idx*/, uint32_t /*rgb*/) { return true; }

    /** @brief No-op brightness set. */
    void setBrightness(uint8_t /*idx*/, uint8_t /*brightness*/) {}
};

/**
 * @brief No-op mock of the on-board buzzer (`MiniR4.Buzzer`).
 *
 * `Tone` mirrors `MiniR4BUZZER::Tone(frequency, duration)` but does nothing —
 * BEEP steps are captured by the Mission.cpp recording hook.
 */
class CtpsMockBuzzer
{
public:
    /** @brief No-op tone. */
    void Tone(uint16_t /*frequency*/, uint32_t /*duration*/) {}

    /** @brief No-op stop-tone. */
    void NoTone(void) {}
};

/**
 * @brief No-op mock of an on-board button (`MiniR4.BTN_UP` / `BTN_DOWN`).
 *
 * `getState()` mirrors `MiniR4BTN<ID>::getState()` and always returns `false`
 * (no button is pressed in the headless dump). Templated on the same `ID`
 * parameter as the real class so the member shapes match.
 */
template <uint8_t ID>
class CtpsMockBTN
{
public:
    /** @brief Always `false` — no button press in the headless dump. */
    bool getState(void) { return false; }
};

/**
 * @brief No-op mock of the on-board OLED (`MiniR4.OLED`).
 *
 * Covers the Adafruit_SSD1306-style surface EventTrace / Mission could touch.
 * EventTrace is itself a no-op mock, so in practice nothing here runs — these
 * exist only so any stray OLED call site compiles.
 */
class CtpsMockOLED
{
public:
    void clearDisplay() {}
    void display() {}
    void setTextSize(uint8_t /*size*/) {}
    void setTextColor(uint16_t /*color*/) {}
    void setCursor(int16_t /*x*/, int16_t /*y*/) {}
    void print(const char* /*s*/) {}
    void println(const char* /*s*/) {}
    void println() {}
};

/**
 * @brief No-op mock of the power-management facade (`MiniR4.PWR`).
 */
class CtpsMockPower
{
public:
    /** @brief No-op battery-cell config; always reports success. */
    bool setBattCell(uint8_t /*cell*/) { return true; }
};

/**
 * @brief No-op mock of the top-level MatrixMiniR4 facade.
 *
 * Holds only the members the CTPS compile chain (Mission.cpp / EventTrace)
 * references. The real facade has many more modules; they are omitted because
 * no CTPS-linked TU touches them.
 */
class MatrixMiniR4
{
public:
    /** @brief No-op facade bring-up; always reports success. */
    bool begin() { return true; }

    CtpsMockPower      PWR;        ///< Power management (no-op).
    CtpsMockLED        LED;        ///< On-board RGB LED pair (no-op).
    CtpsMockBuzzer     Buzzer;     ///< On-board buzzer (no-op).
    CtpsMockOLED       OLED;       ///< On-board OLED (no-op).
    CtpsMockBTN<2>     BTN_UP;     ///< Up button — always returns not-pressed.
    CtpsMockBTN<1>     BTN_DOWN;   ///< Down button — always returns not-pressed.
};

/** @brief Single global facade instance — defined in ctps_record.cpp. */
extern MatrixMiniR4 MiniR4;

#endif    // MATRIXMINIR4_H
