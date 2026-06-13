# Vision Module (HUE12) Plan — Issue #38

**Goal:** Build a firmware `Vision` module that reads the color mosaic from the HUE12 board in one shot and exposes per-cell colors to the mission, with on-R4 hue classification and a hardware-independent test path.

**Approach:** A capture-once / latch / pure-getter module. `readMosaic()` is a one-shot command called at a line-anchored full stop (ADR-006): it flushes the streaming UART, reads one fresh at-rest frame through our **own bounded-resync parser** (the vendor parser is unsafe — see Decisions), unpacks the packed 9-word frame into 12 hues + 12 saturations, classifies each active mosaic cell on the R4 via a `Config::HUE_BANDS` table, and latches the result. `getColor(cell)` / `getRawHue(cell)` are pure getters over the latch. The read blocks but spins the registered `cooperativeTick` while waiting (the sanctioned Drive-drain pattern), so the OLED/heartbeat stay alive. Frame format, baud (9600), packing, and the active-cell count are reverse-engineered / provisional and are captured as `Config` constants so a bench surprise is a one-line change.

## Scope

- **In scope:**
  - `Vision.h` / `Vision.cpp` — `begin()`, `readMosaic()` (command), `getColor(cell)` / `getRawHue(cell)` (pure getters), `hasFault()` / `getFault()` / `clearFault()`, `setTickCallback()`. **No `update()`** and **not** in `cooperativeTick()` (no async work).
  - `VisionFrame.h` — header-only pure free functions: `parseFrame()` (bounded resync + header/length/XOR validate → 9 words), `unpackFrame()` (9 words → 12 hue + 12 sta), `classifyHue()` (hue+sta → `Config::Color`). The ADR-030 host-link test seam.
  - `ConfigVision.h` — reopens `namespace Config`, `#include`d at the bottom of `Config.h` (mirrors `ConfigManipulator.h`, ADR-031): `enum class Color`, `HueBand`/`HUE_BANDS[]`, `VISION_SLOT_COUNT`, `VISION_CELL_COUNT`, `VISION_CELL_TO_SLOT[]`, `VISION_UART_BAUD`, `VISION_FRAME_WORDS`, `VISION_READ_TIMEOUT_MS`, `VISION_WHITE_STA_MAX`, `VISION_FLUSH_DISCARD_FRAMES`. New invariants go in `ConfigChecks.h` (ADR-031).
  - `WRO2026.ino` wiring — `#include "Vision.h"`, `Vision.begin()` in the `setup()` ok-chain + boot-fail branch + `Vision.setTickCallback(cooperativeTick)`.
  - Host C++ ctest — `test/test_vision.cpp` + `CMakeLists.txt` target (with ASan/UBSan, like `test_manipulator`) + an additive streaming-`Serial1` mock in `test/mocks/Arduino.h`.
  - Bench-test sketch — `tools/vision_bench/vision_bench.ino`, a standalone diagnostic that prints each slot's raw hue/sta so the operator measures the frame emission rate, maps cell→slot, and reads values for `HUE_BANDS` calibration.
  - Doc/ADR updates — ADR-006, ADR-021, ADR-003 (blocking list), new **ADR-037**, `DECISIONS.md`, `ARCHITECTURE.md`, `Roadmap.md`, `CLAUDE.md`/`AGENTS.md` module lists (see T9).
- **Out of scope (tracked, not built here):**
  - Python sim vision twin — the sim is navigation-only and nothing consumes mosaic colors yet (ADR-033). Revisit when Tasks land in the sim (Phase 5/6).
  - The `mosaicPlace` Task and any Mission DSL `.task()` wiring — Phase 6.
  - A separate single-block / cement-block color read — **deferred, not "served."** The strict all-active-cell guard would fault a held-block pose (backdrop slots read NONE/Black), so a guard-bypassed single-region variant is a distinct future method. ADR-021 is updated to say single-block is deferred, not that `readMosaic` covers it.
  - Physical camera mount geometry — mech track.
  - Per-cell fault bitmask and the ≤6-per-color plausibility check — considered, declined for surface-area reasons.
  - Final calibrated `HUE_BANDS` values + the real cell→slot order — placeholders here; settled by the bench task on hardware.
- **Open questions:**
  - **RESOLVED (user, 2026-06-13) — active mosaic cell count = 12 (4×3).** The HUE12 device has 12 slots, all active. The repo Roadmap/ADR-006 (3×3 = 9) are now the stale source and are corrected in T9. The slot/cell split is retained so a future game-layout change is a one-line edit.
  - Frame **emission rate** (Hz) is still unverified — see Decisions / T8.

## Decisions

- **Own the parser; do not reuse `MiniR4SmartCamReader`.** The vendor parser runs off the end of a non-void function on a misaligned start (UB, and the *common* case against a continuously streaming device), overruns a 40-byte stack buffer on an unvalidated length byte (up to 510 bytes), false-syncs on `0xAA`-as-data (hue 170 = blue can forge a header), and busy-waits with no tick hook (so the chosen tick-re-entry could not happen). We write a ~60-line bounded-resync parser instead.
- **Frame contract (from the `.mbr4`, bench-verify):** one frame = `0xAA`, length byte `== VISION_FRAME_WORDS` (9), then 9 little-endian uint16 words, then a 1-byte XOR checksum. **Checksum coverage:** seed `check = 0xAA ^ length`, then XOR every one of the `2*length` data bytes; the trailing byte must equal `check` (matches the vendor scheme, MiniR4SmartCamReader.h:59,69 — XOR over data bytes only rejects every real frame). Words 0–5 carry the 12 hues two-per-word (`hue[2k]=word[k]&0xFF`, `hue[2k+1]=word[k]>>8`); words 6–8 carry the 12 saturations four-per-nibble (`sta[j]=(word[6+j/4]>>4*(j%4))&0x0F`, range 0–15). `VISION_CELL_COUNT` (active cells) and `VISION_SLOT_COUNT`/`VISION_FRAME_WORDS` are distinct numbers.
- **Baud = 9600 default.** Counter-evidence is acknowledged: the vendor's own reader **for this device** (`MiniR4SmartCamReader::Begin`) hardcodes 115200, which contradicts the facade docstring "mVision (UART 9600)". The `.mbr4` contract program (UART 9600) is the tiebreaker, bench-verified first thing in T8. `begin()` calls `Serial1.begin(Config::VISION_UART_BAUD)` directly. A frame is ~22 ms at 9600.
- **Fresh-frame discipline.** The device streams continuously, so at the stop the RX buffer holds frames captured while moving. `readMosaic()` flushes RX, then discards `VISION_FLUSH_DISCARD_FRAMES` (=1) fully-parsed frames and latches the next — guaranteeing the latched frame *began after the flush*. (It does not by itself guarantee the chassis was at rest; that is a caller contract — call `readMosaic()` only after `Drive.isIdle()`, documented as a `@note` on the method.)
- **Capture execution = blocking + tick re-entry** (user choice; Fable-endorsed). `readMosaic()` is synchronous but spins the registered `cooperativeTick` in its byte-wait loops, exactly like `Drive::_drainBlocking`. It is added to the ADR-003 / rule-1 permitted command-time-blocking list. Called only at a deliberate full stop.
- **Classification on the R4, in `Config::HUE_BANDS`** (one table, ADR-021). `classifyHue(hue, sta)` resolves in this fixed order: (1) `hue == 255` → `UNKNOWN` (device NONE); (2) `246 ≤ hue ≤ 250` → `UNKNOWN` (device Black); (3) `240 ≤ hue ≤ 245` → `WHITE`; (4) chromatic `hue ≤ 239` with `sta < VISION_WHITE_STA_MAX` → `WHITE` (desaturated); (5) chromatic `hue` in a `HUE_BANDS` entry → that color; (6) otherwise → `UNKNOWN` (band gap / uncalibrated). Bad device codes are caught *before* the STA-gate, so a low-sta Black stays bad. Single editable, version-controlled, sim-mirrorable table.
- **Strict framing guard, cell-driven.** A mosaic cell must be Y/B/G/W. The guard iterates the `VISION_CELL_COUNT` *active* cells via `VISION_CELL_TO_SLOT` only; if any active cell classifies `UNKNOWN` (device NONE/Black, or an uncalibrated band gap) the **whole** read faults with `Fault::BAD_CELL` carrying `firstBadCell` + the raw hue. Slots not referenced by `VISION_CELL_TO_SLOT` are unused and excluded — so the design does not break if `VISION_CELL_COUNT < VISION_SLOT_COUNT`.
- **Fault surface mirrors Drive/PathTracker** — a `Fault` enum + `FaultInfo`, surfaced via `hasFault()/getFault()/clearFault()`; `readMosaic()` returns `false` on failure. Vision does not beep on its own; the caller decides. Retry-once on a failed read before faulting. On failure the latch keeps its prior contents (caller treats the `false` return as authoritative — see ADR-037 for the getter-staleness window).
- **`Color` enum = `{ UNKNOWN, YELLOW, BLUE, GREEN, WHITE }`** in `ConfigVision.h`. `UNKNOWN` = pre-read / out-of-range / unclassifiable sentinel; no `NONE`/`BLACK` members (those device codes map to `UNKNOWN` and fault, never latch as a color).
- **Safe-sentinel getters.** `getColor(cell)` → `UNKNOWN` and `getRawHue(cell)` → `0` for an out-of-range index or before any successful read (mirrors ADR-028 FieldMap bounds-checking). Pure, no I/O. `getRawSta` is not exposed (no consumer; raw sta is used transiently inside classify).
- **No `update()` on Vision.** There is no in-flight async work; a do-nothing `update()` would only invite future I/O into a tick path. Vision is wired into `setup()` (`begin()` + `setTickCallback`) but not into `cooperativeTick()`. This is consistent with rule 1 *given* T9 adds `readMosaic` to the sanctioned command-time-blocking list.

## Data Model

```cpp
// ConfigVision.h  (reopens namespace Config; #included by Config.h before ConfigChecks.h)
namespace Config {

enum class Color : uint8_t { UNKNOWN = 0, YELLOW, BLUE, GREEN, WHITE };

constexpr uint8_t  VISION_SLOT_COUNT  = 12;   // HUE12 hardware slots S1..S12 (.mbr4 contract)
// Active game-mosaic cells: 4x3=12, confirmed by the user 2026-06-13 (repo Roadmap/ADR-006
// previously said 3x3=9 and are corrected in T9). Split kept so a layout change is one line.
constexpr uint8_t  VISION_CELL_COUNT  = 12;   // active mosaic cells; was 9 in ADR-006

constexpr uint8_t  VISION_FRAME_WORDS       = 9;    // packed words per frame (NOT cell/slot count)
constexpr uint32_t VISION_UART_BAUD         = 9600; // contract-program baud; bench-verify (T8)
constexpr uint16_t VISION_READ_TIMEOUT_MS   = 120;  // per attempt; >= (DISCARD+1) emission periods.
                                                    // NOTE: emission RATE (Hz) is unverified -> T8.
constexpr uint8_t  VISION_WHITE_STA_MAX     = 3;    // sta (0..15) below which a chromatic hue reads White
constexpr uint8_t  VISION_FLUSH_DISCARD_FRAMES = 1; // frames dropped after flush before latching

// Raw device hue -> game color. Bands are placeholders; calibrated on hardware (bench task).
struct HueBand { uint8_t lo; uint8_t hi; Color color; };  // inclusive [lo,hi], chromatic 0..239
constexpr HueBand HUE_BANDS[] = {
    { 20,  45, Color::YELLOW },   // placeholders
    { 80, 140, Color::GREEN  },
    {141, 200, Color::BLUE   },
};
// Device-side achromatic White band 240..245 is handled in classifyHue, not in HUE_BANDS.

// Active mosaic cell index -> the HUE12 slot (0..VISION_SLOT_COUNT-1) that reads it.
// Identity is a PLACEHOLDER pending bench cell->slot teaching. Length is VISION_CELL_COUNT.
// Slots not referenced here are unused and excluded from the strict framing guard.
constexpr uint8_t VISION_CELL_TO_SLOT[VISION_CELL_COUNT] = {0,1,2,3,4,5,6,7,8,9,10,11};

}  // namespace Config

// ConfigChecks.h additions (ADR-031): static_assert HUE_BANDS entries 0..239 & lo<=hi;
// VISION_CELL_TO_SLOT entries < VISION_SLOT_COUNT and unique; VISION_CELL_COUNT <= VISION_SLOT_COUNT;
// VISION_WHITE_STA_MAX <= 15.
```

```cpp
// Vision.h
class VisionClass {
public:
    enum class Fault : uint8_t { NONE = 0, READ_TIMEOUT, BAD_CHECKSUM, BAD_CELL };
    struct FaultInfo { Fault kind = Fault::NONE; uint8_t firstBadCell = 0xFF; uint8_t rawValue = 0; };

    VisionClass();
    bool          begin();                          // Serial1.begin(VISION_UART_BAUD); clears latch+fault
    bool          readMosaic();                      // one-shot command; true on success, else sets fault
    Config::Color getColor(uint8_t cell) const;      // pure; UNKNOWN if pre-read / out-of-range
    uint8_t       getRawHue(uint8_t cell) const;     // pure; 0 if pre-read / out-of-range
    bool          hasFault() const;
    FaultInfo     getFault() const;
    void          clearFault();
    void          setTickCallback(void (*cb)());
private:
    Config::Color _colors[Config::VISION_CELL_COUNT];
    uint8_t       _rawHue[Config::VISION_CELL_COUNT];
    bool          _hasRead = false;
    FaultInfo     _fault;
    void (*_tickCallback)() = nullptr;
};
extern VisionClass Vision;
```

```cpp
// VisionFrame.h — header-only pure functions (no I/O), the host-test seam
namespace VisionFrame {
// Scan buf[0..len) for a valid frame (resync on 0xAA, length==VISION_FRAME_WORDS, XOR ok).
// Writes the 9 words; sets consumed past the accepted frame; returns false if none complete.
inline bool parseFrame(const uint8_t* buf, size_t len, uint16_t outWords[Config::VISION_FRAME_WORDS],
                       size_t& consumed);
inline void unpackFrame(const uint16_t words[Config::VISION_FRAME_WORDS],
                        uint8_t hue[Config::VISION_CELL_COUNT], uint8_t sta[Config::VISION_CELL_COUNT]);
inline Config::Color classifyHue(uint8_t hue, uint8_t sta);  // fixed order in Decisions
}
```

## Acceptance Criteria

- [ ] **AC1** — `readMosaic()` at a stop reads one fresh frame, classifies it, latches 12 cell colors, and returns `true` on success. *(T4)*
- [ ] **AC2** — `classifyHue` follows the fixed order: chromatic hue → its band color; 240–245 → `WHITE`; chromatic + `sta < VISION_WHITE_STA_MAX` → `WHITE`; 255 / 246–250 / band-gap → `UNKNOWN`. *(T2)*
- [ ] **AC3** — `getColor(cell)` returns the latched color; `UNKNOWN` before any successful read or for an out-of-range index; `getRawHue(cell)` likewise returns `0`. Neither crashes. *(T4)*
- [ ] **AC4** — A read where any *active* cell classifies `UNKNOWN` faults (`Fault::BAD_CELL`, `firstBadCell`+raw set), returns `false`, and does not overwrite the prior latch. *(T4)*
- [ ] **AC5** — `parseFrame` validates header / length(==9) / XOR(seed `0xAA^length`), resyncs on a misaligned start, rejects a `0xAA`-as-data false header, and rejects a corrupt length without reading past the buffer. *(T2)*
- [ ] **AC6** — A checksum/timeout failure retries once; a second failure sets the matching `Fault` and returns `false`; `hasFault/getFault/clearFault` behave. *(T4)*
- [ ] **AC7** — `readMosaic` flushes streamed backlog and latches a fresh frame, discarding `VISION_FLUSH_DISCARD_FRAMES` stale frames first. *(T4)*
- [ ] **AC8** — Vision is wired into `setup()` (`#include`, `begin()` in the ok-chain, boot-fail branch, `setTickCallback`) and is **not** in `cooperativeTick()`; the sketch compiles via `arduino-cli`. *(T5)*
- [ ] **AC9** — `vision_bench.ino` prints per-slot raw hue/sta over USB `Serial`, enabling emission-rate measurement, cell→slot mapping, and `HUE_BANDS` calibration. *(T8)*

**AC → TC map:** AC1 → TC6,TC12 · AC2 → TC1,TC2,TC3,TC4,TC5,TC21 · AC3 → TC17,TC19 · AC4 → TC14 · AC5 → TC7,TC8,TC9,TC10,TC11 · AC6 → TC15,TC16,TC20 · AC7 → TC13 · AC8 → build-only (arduino-cli compile in CI; no host TC) · AC9 → hardware/compile-only (no host TC).

## Error Handling

- **Misaligned / mid-frame start:** `parseFrame` resyncs by scanning for `0xAA`; a frame that doesn't validate (bad length or XOR) is skipped and the scan continues within the buffer. Never reads past `len`.
- **Corrupt length byte:** `parseFrame` rejects any `length != VISION_FRAME_WORDS`; the fixed read buffer is sized for one max frame, so no overrun is possible (asserted under ASan in TC10).
- **Checksum failure / timeout:** `readMosaic` retries once; a second failure sets `Fault::BAD_CHECKSUM` / `Fault::READ_TIMEOUT` and returns false.
- **NONE/Black/uncalibrated cell:** any active cell classifying `UNKNOWN` sets `Fault::BAD_CELL` with `firstBadCell` + `rawValue` (raw hue), returns false; the latch is left at its prior contents.
- **Pre-read / out-of-range getter:** returns `Color::UNKNOWN` / `0`; never indexes out of bounds.
- **Stale streamed data:** flushed before every read; only a post-flush frame is latched.
- **`setTickCallback` not registered:** `readMosaic` still completes (bare bounded wait) but emits a one-time Serial warning, matching the Drive/Manipulator convention.
- **Emission-rate unknown:** if `VISION_READ_TIMEOUT_MS` is shorter than `(VISION_FLUSH_DISCARD_FRAMES + 1)` emission periods, every read spuriously times out. T8 measures the rate before this constant is trusted.

## Testing Strategy

**Levels:** Unit (pure `VisionFrame` functions) + Integration (host `readMosaic` over a mock `Serial1`). All host-side; no hardware. The `test_vision` target builds with `-fsanitize=address,undefined` on GNU/Clang (same as `test_manipulator`) so the no-overrun claims are machine-checked. Run: `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`.

**Host-clock note:** the mock `millis()` advances only via `delay()` / explicit helpers. `readMosaic`'s wait loop MUST call `delay(small)` every iteration (mirroring `Drive.cpp` drain), else timeout tests spin forever. The mock supports a drip-feed mode (bytes released from inside the registered tick callback / per `delay()` tick) so TC16/TC18-style cases actually exercise the wait loop.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `classifyHue` each band midpoint (Y/G/B) | Unit | Returns the matching `Color` |
| TC2 | `classifyHue` hue in 240–245 | Unit | Returns `WHITE` |
| TC3 | `classifyHue` chromatic hue, `sta < VISION_WHITE_STA_MAX` | Unit | Returns `WHITE` (STA-gate) |
| TC4 | `classifyHue` hue 255 and hue in 246–250 | Unit | Returns `UNKNOWN` (device NONE/Black) |
| TC5 | `classifyHue` chromatic band gap, and band boundary hues (lo, hi, lo-1, hi+1) | Unit | Inclusive bounds correct; gaps → `UNKNOWN` |
| TC6 | `unpackFrame` known 9 words | Unit | 12 hues (byte split) + 12 stas (nibble split) exactly correct |
| TC7 | `parseFrame` clean valid frame | Unit | true, 9 words correct, `consumed` past frame |
| TC8 | `parseFrame` buffer starting mid-frame then a full frame | Unit | Resyncs, returns the valid frame |
| TC9 | `parseFrame` `0xAA` as a data byte before the real header | Unit | No false-sync; returns the real frame |
| TC10 | `parseFrame` length byte = 200 (corrupt) | Unit | Rejected, no read past `len`, no overrun (ASan clean) |
| TC11 | `parseFrame` valid header, wrong XOR | Unit | Frame not accepted (skips to next / false) |
| TC12 | `readMosaic` happy path (mock streams valid frames) | Integration | true; 12 `getColor` + `getRawHue` match the frame |
| TC13 | `readMosaic` stale backlog: old frame then fresh frames | Integration | Latches the fresh frame, not the stale one |
| TC14 | `readMosaic` an active cell = Black(248) | Integration | false; `Fault::BAD_CELL`, `firstBadCell` set; prior latch unchanged |
| TC15 | `readMosaic` first attempt bad checksum, second good | Integration | true (retry-once); no fault |
| TC16 | `readMosaic` persistent timeout (drip-feed never completes a frame) | Integration | false; `Fault::READ_TIMEOUT`; `hasFault()` true; `clearFault()` clears; loop terminates |
| TC17 | `getColor`/`getRawHue` before any read, and out-of-range index | Unit | `UNKNOWN` / `0`, no crash |
| TC18 | `setTickCallback` registered, drip-fed read | Integration | Mock counts callback invocations > 0 during the wait |
| TC19 | Non-identity `VISION_CELL_TO_SLOT` (test override): cell→slot direction | Unit/Integration | `getColor(cell)` returns the color from the *mapped slot*, proving direction (catches a transposed map) |
| TC20 | Read with NO tick callback registered, across two reads | Integration | Completes; one-time Serial warning appears exactly once |
| TC21 | `classifyHue` precedence: hue 248 with `sta = 1` | Unit | Returns `UNKNOWN` (bad-code checked before STA-gate, not `WHITE`) |

**Test data:** the mock `Serial1` exposes a settable streaming RX queue (`mockSerial1Feed(bytes,n)` / `mockSerial1Clear()`) plus a drip-feed mode; helper builders construct valid/stale/corrupt frames from a 12-color array (and emit the correct XOR). TC19 uses a test-only override of `VISION_CELL_TO_SLOT` via the repo's `#ifndef ..._DEFAULT` override idiom. `resetWorld()` clears the queue + re-`begin()`s Vision before each scenario.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | `ConfigVision.h` + checks + include | — | low | `WRO2026/ConfigVision.h`, `WRO2026/Config.h`, `WRO2026/ConfigChecks.h` | Create `ConfigVision.h` reopening `namespace Config` with the `Color` enum, `HueBand`/`HUE_BANDS[]` (placeholder bands), `VISION_CELL_TO_SLOT[]` (identity placeholder), and all `VISION_*` constants per the Data Model **including the `VISION_SLOT_COUNT` vs `VISION_CELL_COUNT` split**. Add `#include "ConfigVision.h"` beside `#include "ConfigManipulator.h"` in `Config.h` (before `ConfigChecks.h`). Add the `static_assert`s listed in the Data Model to `ConfigChecks.h` (ADR-031). Cell count = 12 confirmed by user 2026-06-13. Satisfies the one-Config-table decision (ADR-021). |
| T2 | `VisionFrame.h` pure functions | T1 | high | `WRO2026/VisionFrame.h` | Header-only `parseFrame` (bounded resync: scan for `0xAA`, require `length==VISION_FRAME_WORDS`, XOR check seeded `0xAA^length` over all `2*length` data bytes, never read past `len`, set `consumed`), `unpackFrame` (byte-split hues words 0–5, nibble-split stas words 6–8 per the Decisions formulas), `classifyHue` (the 6-step fixed order). All `inline`, no I/O, no globals. Mirror the `DirectionPolicy.h` header-only-policy precedent (ADR-030). Satisfies AC2, AC5. |
| T3 | `Vision.h` class | T1, T2 | low | `WRO2026/Vision.h` | Declare `VisionClass` per the Data Model: `Fault` enum + `FaultInfo`, `begin/readMosaic/getColor/getRawHue/hasFault/getFault/clearFault/setTickCallback`, private `_colors`/`_rawHue` latch + `_hasRead` + `_fault` + `_tickCallback`, `extern VisionClass Vision;`. Include guard `VISION_H`, full Doxygen. Add a `@note` to `readMosaic` requiring the chassis be idle (`Drive.isIdle()`) at the call site. **No `update()`.** |
| T4 | `Vision.cpp` implementation | T3 | high | `WRO2026/Vision.cpp` | Define the singleton. `begin()`: `Serial1.begin(Config::VISION_UART_BAUD)`, init latch to `UNKNOWN`/0, clear fault. `readMosaic()`: flush RX; collect bytes into a fixed buffer, **calling `_tickCallback()` then `delay(small)` every wait iteration** (bare wait + one-time warning if null) under `VISION_READ_TIMEOUT_MS`; use `VisionFrame::parseFrame` discarding `VISION_FLUSH_DISCARD_FRAMES` frames then latch the next; `unpackFrame`; map each active cell via `VISION_CELL_TO_SLOT` and `classifyHue`; strict guard → `Fault::BAD_CELL` on any active-cell `UNKNOWN`; retry-once on checksum/timeout; set fault + return false on second failure; on success store `_colors/_rawHue`, set `_hasRead`, clear fault, return true. Getters pure + bounds-checked. Satisfies AC1, AC3, AC4, AC6, AC7. |
| T5 | Wire into the sketch | T3 | low | `WRO2026/WRO2026.ino` | Add `#include "Vision.h"` to the sketch include block. Add `const bool visionOk = Vision.begin();` into the `setup()` ok-chain, `&& visionOk` in `_setupFailed`, and a boot-fail branch (`if(!visionOk){ Serial.println(...); EventTrace.pushEvent("FAIL VIS"); }`) matching the existing per-module pattern. Add `Vision.setTickCallback(cooperativeTick);` beside the other registrations. Do **not** add `Vision.update()` to `cooperativeTick()`. Satisfies AC8. |
| T6 | Streaming `Serial1` mock | — | med | `WRO2026/test/mocks/Arduino.h` (+ `mocks.cpp` if needed) | Add an additive mock `Serial1` (a `HardwareSerial`-like object) with a settable streaming RX queue: `available()`, `read()`, `write()`/`print()` no-ops, `begin(baud)` records baud, helpers `mockSerial1Feed(const uint8_t*, size_t)` / `mockSerial1Clear()`, and a **drip-feed mode** that releases queued bytes from inside the tick callback / per `delay()` tick. Provide a callback-invocation counter for TC18. Keep additive so other targets are unaffected (mocks/ shadows all targets). |
| T7 | Host tests + ctest target | T2, T4, T6 | med | `WRO2026/test/test_vision.cpp`, `WRO2026/test/CMakeLists.txt` | Implement TC1–TC21 against real `Vision.cpp` + `VisionFrame.h` + the mock. Add `add_executable(test_vision ...)` linking `${SKETCH_DIR}/Vision.cpp` + `${MOCK_SOURCES}`, mocks dir first on the include path, the `-fsanitize=address,undefined` block (copy from `test_manipulator`), and `add_test`. Frame-builder helpers for valid/stale/corrupt streams (with correct XOR). TC19 uses the `#ifndef VISION_CELL_TO_SLOT_DEFAULT` override hook. Verifies every AC via its mapped TCs. |
| T8 | Bench-test sketch | T2 | med | `tools/vision_bench/vision_bench.ino` | Standalone sketch: `Serial1.begin(9600)`, loop reading frames via `VisionFrame::parseFrame`/`unpackFrame`, print `slot index : rawHue : rawSta` for all slots each frame over USB `Serial`, plus a frames/sec counter. Document the on-hardware steps in a header comment: (a) confirm baud + **measure emission rate** (feeds `VISION_READ_TIMEOUT_MS`); (b) map cell→slot to fix `VISION_CELL_TO_SLOT`; (c) read hues to calibrate `HUE_BANDS`; (d) note `MiniR4.Uart`≡`Serial1` so a dead D0/D1 socket isn't misdiagnosed as a baud problem. Resolve the Arduino include reach into `WRO2026/VisionFrame.h` via `arduino-cli --build-property "compiler.cpp.extra_flags=-I<sketchdir>/WRO2026"` (no copy/replica — ADR-030). Satisfies AC9. |
| T9 | ADR + doc updates | — | low | `docs/decisions/006-mosaic-via-camera.md`, `docs/decisions/021-camera-as-sole-color-sensor.md`, `docs/decisions/003-arduino-style-modules.md`, `docs/decisions/037-hue12-vision-capture-once.md`, `DECISIONS.md`, `ARCHITECTURE.md`, `Roadmap.md`, `CLAUDE.md`, `AGENTS.md` | Update ADR-006 (record the slot-vs-cell distinction; 9→12 active cells, confirmed by the user 2026-06-13, cited to `WRO2026_HighSchool_Topic_Drawer.html`; line-anchored capture stands). Update ADR-021 (`readMosaic`+`getColor` replace `capturePattern(out[9])`/`readSingleColor`; single-block read **deferred**, not served; STA-gated White; one `HUE_BANDS` table). Add `Vision::readMosaic` to the permitted-blocking lists in **both** ADR-003 and `ARCHITECTURE.md` (rule 1). Write **ADR-037** (capture-once latch + on-device slot teaching + R4 hue-band classification; 4th sanctioned command-time blocking site; getter-staleness-after-failed-read window). Add ADR-037 to `DECISIONS.md` index + status table. Reconcile **all** stale 3×3/9-cell language in `Roadmap.md` via a grep sweep (`3×3|9 cells|6 of 9|capturePattern|readSingleColor`, lines ~17,53,157,165-166,180-181,229,233,244-246,408,411,430). Add the Vision module to the `CLAUDE.md` "What's already built" list and mirror into `AGENTS.md`. Verify ADR-037 doesn't collide with any unmerged branch's number before committing. |

## Notes for Implementer

- **Cell count = 12, confirmed (user, 2026-06-13).** The `VISION_SLOT_COUNT`/`VISION_CELL_COUNT` split is kept so a future layout change is a one-line edit, but 12 is the committed value.
- **The `.mbr4`-derived facts (frame=9 packed words, baud=9600, packing formulas, emission rate) are reverse-engineered, not datasheet-confirmed.** They are isolated in `ConfigVision.h` + `VisionFrame.h` so the bench task (T8) can correct any one with a constant/formula tweak and no API churn. Do not scatter `9`, `9600`, or `0xAA` elsewhere.
- **`VISION_CELL_COUNT` ≠ `VISION_FRAME_WORDS` ≠ `VISION_SLOT_COUNT`.** The length-byte check is `== 9`; cell/getter loops bound on `VISION_CELL_COUNT`; the device has `VISION_SLOT_COUNT` slots. Mixing them is the most likely bug.
- **The wait loop must `delay(small)` every iteration** (and call `_tickCallback`). On the host the mock clock only advances via `delay()`, so a tight spin both hangs the timeout tests and burns the MCU; on hardware the tick re-entry is what keeps the OLED/heartbeat alive. Mirror `Drive::_drainBlocking` (`WRO2026/Drive.cpp`). The Notes here state the rule as "never spin without `delay()` + tick", not the reverse.
- **Fresh-frame discipline** (flush + discard `VISION_FLUSH_DISCARD_FRAMES`) is correctness, not nicety — skipping it latches motion-time data and every getter then lies. Pair it with the `Drive.isIdle()` caller contract.
- **STA is 4-bit (0–15)**, not 0–255 — the white gate threshold lives on that scale; don't compare it against a 0–255 value.
- **classify precedence** is bad-codes (255, 246–250) FIRST, then the STA-gate, then bands — so a low-sta Black is not reclassified White (TC21).
- The vendor `MiniR4SmartCamReader` and `MiniR4.Vision` are **not used**; Vision talks to `Serial1` directly. Do not `#include <MatrixMiniR4.h>` from `Vision.cpp` for the reader. Note `MiniR4.Uart`≡`Serial1` (MatrixMiniR4.h:114) when bench-debugging the physical socket.
- Rollback: the module is self-contained and not in `cooperativeTick()`; reverting T5 (one `setup()` edit) fully disconnects it without touching other modules.
