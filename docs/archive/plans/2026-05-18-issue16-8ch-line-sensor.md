# 8-Channel Custom Line Sensor — Issue #16 Plan

**Goal:** Replace the vendor `MatrixLineTracer` (10-ch I²C) with a custom 8-channel reflectance array (one or two MXColor-protocol boards packing two channels per CMYK byte), behind a stable `WROLineArray` interface that `LineFollower` and `Intersection` consume in place of the old tracer.

**Approach:** Introduce one new class, `WROLineArray`, that owns 1 or 2 `MatrixColor` instances (front + optional rear), unpacks the 4 CMYK bytes per board into 8 reflectance channels (each 0–9, higher = darker), applies per-channel software calibration on top of the firmware-level hardware calibration, and exposes the same three-method surface `MatrixLineTracer` did: `getError()`, `isOnline()`, `getJunctionType()`. The active board auto-follows `LineFollower::setReverse()`. `LineFollower._tracer` and `Intersection._tracer` change type from `MatrixLineTracer*` to `WROLineArray*`; the rest of the controller logic — the fluent builder, the distance gate, the debounce, and the lost-line dispatch — is untouched. A new BTN_DOWN-triggered software-calibration routine prints per-channel min/max to Serial; Diego hand-edits the values into `Config.h` and re-flashes, mirroring the existing `Drive.calibrateMotorBias()` workflow.

## Scope

- **In scope:**
  - New module `WROLineArray.h` / `WROLineArray.cpp`.
  - Support 1 or 2 sensor boards via `Config::LINE_BOARD_COUNT ∈ {1, 2}`. With 2 boards: front + rear, active board follows direction.
  - Per-board, per-channel software calibration baselines stored in `Config::LINE_CALIB_LOW[B][8]` / `LINE_CALIB_HIGH[B][8]`. Calibration routine prints values to Serial in paste-ready format.
  - Invocation of the firmware-level hardware-calibration / illumination command on each board at `WROLineArray::begin()`, so the on-board LED and any firmware-normalization layer are in a known steady state before the software calibration is ever sampled.
  - Junction detection derived from raw 8-channel readings using the stricter rule (see Decisions).
  - `LineFollower._tracer` and `Intersection._tracer` retyped to `WROLineArray`; `LineFollower::setReverse()` wired through to switch the active board.
  - Removal of `Config::LINE_I2C_PORT` and `Config::LINE_THRESHOLD` (replaced by the new per-board constants).
  - One bring-up sketch entry in `WRO2026.ino` that runs the software-calibration routine on a BTN_DOWN long-press during pre-mission idle.
  - DECISIONS.md addition documenting the swap, the inferred packed-byte format, and the two-layer calibration discipline.
- **Out of scope:**
  - Keeping the old 10-ch `MatrixLineTracer` code path. The user chose "abstract behind a sensor interface" specifically to swap, not to coexist — we delete the vendor backend rather than gate it behind a flag.
  - EEPROM-stored calibration (rejected for the same hand-edit reasons as `MOTOR_BIAS_*`).
  - Sensor-fusion across the two boards (active board is single-source-of-truth at any given moment).
  - Adaptive (per-tick) thresholding.
  - Per-channel weighting tuning UI — weights are compile-time constants symmetric around the centre.
  - The mechanical mounting of the front and rear boards.

## Decisions

- **C++ shape: concrete class with no virtual dispatch** — `LineFollower._tracer` and `Intersection._tracer` change to `WROLineArray*` directly. No `ILineSensor` virtual base. The "interface" is the stable trio of method signatures, not a vtable. Rationale: Arduino RAM cost discipline, the project uses concrete singletons everywhere, and the swap is compile-time-only — runtime polymorphism would buy nothing. The trio of method names (`getError` / `isOnline` / `getJunctionType`) is preserved exactly so any future second backend can drop in.
- **Polarity: 0–9, higher = darker (on line)** — confirmed by user, matches the literal reading of meat.ino thresholds like `light1 < 4` meaning "channel saw white". A channel is considered ON-LINE when `(value - calib_low) / (calib_high - calib_low) > LINE_CHANNEL_ON_RATIO` (default 0.5).
- **Channel index 0 = leftmost (forward-facing chassis)** — same as meat.ino convention. Channels 0..7 are unpacked from the four CMYK bytes (tens / ones digits). The exact CMYK→channel-index map is **inferred from meat.ino** (see `light_8()` lines 670–684) and must be **validated against the actual hardware** by TC9 — this is the source-cited-proof gate: the meat.ino author's variable assignments are the only evidence we have for the packing order, the MXColor firmware docs do not state it.
- **Two-layer calibration:** the MXColor-protocol board exposes a *firmware-level* hardware calibration / illumination layer (analogous to `MatrixLineTracer::startCalibration()` / `endCalibration()` on the 10-ch vendor sensor, and to `MatrixColor::setLight()` / `setWhiteBalance()` on TCS34725-class boards) that normalizes raw photoreceptor readings **inside the sensor itself** before the I²C byte is ever sent. Our software per-channel `LINE_CALIB_LOW/HIGH` arrays sit **on top of** that, compensating for the residual per-channel variance the firmware cannot see. Both layers run; they are NOT redundant. The hardware-layer command is invoked once per board in `WROLineArray::begin()` (volatile — resets on power cycle); the software-layer baselines are hand-edited into `Config.h` and survive re-flash. `WROLineArray.h` documents this contract so a future maintainer does not delete one layer thinking it is duplicated by the other.
- **Weights are symmetric and fixed at compile time** — `{-3.5, -2.5, -1.5, -0.5, +0.5, +1.5, +2.5, +3.5}`, mirroring the vendor's 10-ch scheme rescaled to 8 channels. This narrows the error range to `[-3.5, +3.5]` vs the vendor's `[-4.5, +4.5]`; `Config::LINE_KP` may need a one-shot retune at first hardware test, but that is normal Phase 1 tuning.
- **Junction rule (stricter):** evaluate in this priority order to ensure mutual exclusivity:
  1. `T_OR_CROSS` if ≥7 of 8 channels are on-line.
  2. `LEFT` if ch0..ch4 all on-line AND ch5..ch7 all off-line.
  3. `RIGHT` if ch3..ch7 all on-line AND ch0..ch2 all off-line.
  4. `NONE` otherwise.
  The asymmetric overlap on ch3..ch4 is by design (per user's wording); the strict opposite-side-off requirement eliminates false fires mid-segment when 5 contiguous centre channels straddle a fat line. `UNKNOWN` is not emitted by `WROLineArray` — we derive the verdict from clean data. `Intersection`'s existing UNKNOWN handling becomes structurally unreachable (left intact for safety; flagged in T7).
- **Active board switching auto-follows `LineFollower::setReverse(bool)`** — when 2 boards are configured AND reverse=true, `WROLineArray` reads from the rear board. Single-board configs ignore the flag. `LineFollower::_applyEnableBuilder` calls `_array->setReverseActive(_reverse)` whenever it touches `_reverse`. No separate `LineArray.setActive(...)` API — one source of truth for direction.
- **Active board offline → lost-line** — `WROLineArray::isOnline()` returns false; the existing `Config::LOST_LINE_ACTION` (STOP/COAST/CRAWL) dispatches as today. No automatic fall-back to the inactive board (would deliver opposite-signed error).
- **Software calibration via BTN_DOWN long-press + Serial print, hand-edited to Config** — mirrors `Drive.calibrateMotorBias()`. No EEPROM, no auto-baseline on boot. Deterministic and reviewable from source. The routine spins for ~5 s while the operator manually sweeps the bar between full-white and over-the-line samples, captures per-channel min/max, and prints a Config-paste-ready block.

## Acceptance Criteria

- [ ] AC1: `WROLineArray.h` compiles standalone (only depends on `Arduino.h`, `MatrixMiniR4.h`, `Config.h`).
- [ ] AC2: With a known black line under channel index `i`, `getError()` returns a value with the sign matching `_sensorWeights[i]` and magnitude ≥ `|_sensorWeights[i]| / 2`.
- [ ] AC3: With every channel below threshold, `getError()` returns 0.0f AND `isOnline()` returns false.
- [ ] AC4: `getJunctionType()` returns the values enumerated in Decisions for the four exact predicates, with priority order respected.
- [ ] AC5: When `Config::LINE_BOARD_COUNT == 2` and `LineFollower.setReverse(true)`, every subsequent `getError()` / `isOnline()` / `getJunctionType()` reads from the rear board's I²C address; `setReverse(false)` reverts.
- [ ] AC6: `WROLineArray::calibrate()` prints, for each configured board, 8 `LOW` and 8 `HIGH` values in a paste-ready `constexpr uint8_t LINE_CALIB_LOW[...] = { ... };` format on Serial at 115200 baud.
- [ ] AC7: After hand-editing the software-calibration values, `LineFollower` follows a 2 m straight line at `Config::SpeedPreset::NORMAL` with ≤ ±5 mm lateral error (re-validates Phase 1 benchmark B1.1 against the new sensor).
- [ ] AC8: `Intersection.isAtIntersection()` fires exactly once per physical T-intersection traversed at SLOW preset across 10 trials (re-validates B1.2 against the new sensor).
- [ ] AC9: All references to `Config::LINE_I2C_PORT` and `Config::LINE_THRESHOLD` are removed; no compile error.
- [ ] AC10: `WROLineArray::begin()` invokes the firmware-level illumination / hardware-calibration command on every configured board exactly once. The on-board LED comes on (or transitions to its configured steady state) by the time `begin()` returns, regardless of any subsequent `update()` activity.

## Data Model

### Config additions (new constants)

```cpp
// ---- 8-channel line array (replaces LINE_I2C_PORT / LINE_THRESHOLD) ----

/** Number of MXColor-protocol boards installed. 1 = front-only, 2 = front + rear. */
constexpr uint8_t LINE_BOARD_COUNT = 1;    // user edits to 2 when rear board installed

/** I²C port (1..4) on Wire1 carrying the FRONT board. */
constexpr uint8_t LINE_FRONT_I2C_PORT = 1;

/** I²C port (1..4) on Wire1 carrying the REAR board. Ignored when LINE_BOARD_COUNT == 1. */
constexpr uint8_t LINE_REAR_I2C_PORT = 4;

/** Per-board per-channel calibrated WHITE (off-line) value. Hand-edited from calibrate() output. */
constexpr uint8_t LINE_CALIB_LOW[2][8]  = {
    {0,0,0,0,0,0,0,0},   // front — TODO: paste calibrate() output
    {0,0,0,0,0,0,0,0},   // rear  — TODO: paste calibrate() output (ignored if LINE_BOARD_COUNT==1)
};

/** Per-board per-channel calibrated DARK (on-line) value. */
constexpr uint8_t LINE_CALIB_HIGH[2][8] = {
    {9,9,9,9,9,9,9,9},   // front — TODO
    {9,9,9,9,9,9,9,9},   // rear  — TODO
};

/** Channel ON-LINE threshold as a fraction of (high - low). 0.5 = midpoint. */
constexpr float LINE_CHANNEL_ON_RATIO = 0.5f;

/** Min channels on-line to declare T_OR_CROSS junction (priority 1). */
constexpr uint8_t LINE_JUNCTION_T_MIN_ON = 7;

/** Channel indices for LEFT junction predicate (priority 2): all of [0..LEFT_HI] ON and all of [LEFT_HI+1..7] OFF. */
constexpr uint8_t LINE_JUNCTION_LEFT_HI  = 4;    // ch0..4 must be ON
constexpr uint8_t LINE_JUNCTION_RIGHT_LO = 3;    // ch3..7 must be ON, ch0..2 must be OFF

/** On-board LED brightness applied at begin() via the firmware-level setLight() command. */
constexpr uint8_t LINE_LED_PWM = 200;
```

### New class: WROLineArray

```cpp
class WROLineArray {
public:
    bool begin();                                  // sets up MatrixColor instances, invokes firmware-level
                                                   //   hardware calibration / setLight() on each board,
                                                   //   defaults active = front
    void update();                                 // optional cache refresh; LineFollower calls it implicitly

    // The "interface" — exact method names mirror MatrixLineTracer for substitutability:
    float   getError();                            // weighted center error, range [-3.5, +3.5]; 0 when offline
    bool    isOnline();                            // true if any channel above ON threshold (active board)
    uint8_t getJunctionType();                     // 0=NONE, 1=LEFT, 2=RIGHT, 3=T_OR_CROSS (per Decisions)

    // New: active-board control. Called by LineFollower._applyEnableBuilder.
    void    setReverseActive(bool reverse);        // 2-board: rear when reverse, front otherwise. 1-board: no-op.
    bool    getReverseActive() const;

    // Raw access for diagnostics / bringup / Intersection debounce.
    bool    readActiveBoard(uint8_t out[8]);       // returns 0..9 per channel; false if I²C read failed
    bool    readBoard(uint8_t boardIdx, uint8_t out[8]);   // 0=front, 1=rear

    // Software calibration routine. Spins for ~5 s, captures per-channel min/max, prints Config-paste block.
    // The HARDWARE calibration is invoked separately in begin() — these two layers are NOT redundant.
    void    calibrate();                           // sync-blocking. Call from sketch top-level only.

private:
    MatrixColor _boards[2];                        // front, rear; _boards[1] inert when LINE_BOARD_COUNT == 1
    uint8_t     _activeBoard = 0;                  // 0 or 1
    uint8_t     _lastChannels[8] = {0};            // cached for getJunctionType after getError
    bool        _lastOnline = false;

    // Packed-byte unpacking: byte / 10 → first channel, byte % 10 → second. Order inferred
    // from meat.ino light_8(); VALIDATED by bringup TC9 before merge.
    bool        _readBoardInternal(uint8_t boardIdx, uint8_t out[8]);
    float       _computeError(const uint8_t channels[8]) const;
    bool        _isOnlineFromCache(const uint8_t channels[8]) const;
    uint8_t     _classifyJunction(const uint8_t channels[8]) const;
};

extern WROLineArray LineArray;
```

### LineFollower changes (field + one wiring point)

```cpp
// LineFollower.h — only the type of _tracer changes.
class LineFollowerClass {
    // ...
    WROLineArray* _array;     // was: MatrixLineTracer _tracer;
    // ...
};

// LineFollower.cpp — begin() sets _array = &LineArray; _applyEnableBuilder() propagates _reverse:
void LineFollowerClass::_applyEnableBuilder(...) {
    // ... existing logic ...
    if (hasReverseOverride) { _reverse = reverse; }
    _array->setReverseActive(_reverse);   // NEW: one line, wherever _reverse is written.
    // ... rest unchanged ...
}
```

### Intersection changes (pointer-type swap only)

```cpp
class IntersectionClass {
    WROLineArray* _array = nullptr;     // was: MatrixLineTracer* _tracer = nullptr;
    // ...
};
// update() body: replace _tracer->getJunctionType() with _array->getJunctionType().
// The UNKNOWN (4) branch becomes structurally unreachable but the validation code stays
// intact in case a future backend re-introduces ambiguous samples.
```

## Wire Protocol (inferred — must be validated)

Per board, one I²C transaction reads 4 CMYK bytes via `MatrixColor::getColor(ColorType)`. Each byte packs two channel values as decimal digits:

```
byte = (channel_a * 10) + channel_b      where channel_a, channel_b ∈ [0..9]
channel_a = byte / 10                    (tens digit)
channel_b = byte % 10                    (ones digit)
```

The four CMYK bytes therefore contribute 8 channels. The mapping inferred from meat.ino `light_8()` (lines 670–684) is:

| CMYK byte | tens digit (channel) | ones digit (channel) |
|-----------|----------------------|----------------------|
| C         | 0 (leftmost)         | 1                    |
| M         | 2                    | 3                    |
| Y         | 4                    | 5                    |
| K         | 6                    | 7 (rightmost)        |

**This mapping is the only inferred contract in the design.** TC9 of the test plan validates it by physically covering one channel at a time and confirming the expected `_lastChannels[i]` index drops. If the mapping is wrong, the fix is local to `WROLineArray::_readBoardInternal` — no caller code changes.

## Error Handling

- **I²C read failure (active board not responding):** `readActiveBoard()` returns false; `getError()` returns 0.0f, `isOnline()` returns false, `getJunctionType()` returns NONE. LineFollower's existing `LOST_LINE_ACTION` dispatch fires (STOP/COAST/CRAWL per Config). No fall-back to the inactive board — would deliver opposite-signed error.
- **Calibration values inverted (HIGH ≤ LOW for a channel):** treated as a "dead channel" — never reports on-line; never contributes to the weighted error. Logged once to Serial at `begin()` so a botched calibration paste is loud.
- **`Config::LINE_BOARD_COUNT == 2` but rear board never responds at begin():** `WROLineArray::begin()` logs a one-time warning to Serial but does not halt — single-board missions can still run on the front board if the operator forgets to install the rear.
- **Hardware-calibration / `setLight()` command fails for a board:** logged to Serial; `begin()` still returns true so the rest of the sketch boots. The on-board LED simply stays in its power-on default state — readings remain usable, just less reliable until a power-cycle clears it.
- **Junction predicate ambiguity:** the priority order in Decisions guarantees at most one verdict per tick. No `UNKNOWN` is emitted.
- **`Intersection` debounce streaks** (`INTERSECTION_DEBOUNCE_TICKS` / `_RELEASE_TICKS`) carry over unchanged — they protect against any backend's transient flicker.

## Testing Strategy

**Levels:** Hardware sketch tests only (project convention — no unit-test framework on Arduino). Each TC is verified by flashing a small sketch and visually / serially checking output.

| ID   | Test Case                                                            | Type      | Expected Behavior                                                                                          |
|------|----------------------------------------------------------------------|-----------|------------------------------------------------------------------------------------------------------------|
| TC1  | All channels report value 0 (bar lifted in ambient white)            | Hardware  | `getError()` → 0.0f; `isOnline()` → false; `getJunctionType()` → NONE                                       |
| TC2  | Bar centered over line (ch3+ch4 above threshold)                     | Hardware  | `getError()` ≈ 0; `isOnline()` → true; `getJunctionType()` → NONE                                           |
| TC3  | Line under ch0 only                                                  | Hardware  | `getError()` ≈ -3.5; `getJunctionType()` → NONE                                                            |
| TC4  | Line under ch7 only                                                  | Hardware  | `getError()` ≈ +3.5                                                                                         |
| TC5  | All 8 channels covered (full T-junction simulation)                  | Hardware  | `getJunctionType()` → T_OR_CROSS                                                                            |
| TC6  | ch0..ch4 covered, ch5..ch7 clear                                     | Hardware  | `getJunctionType()` → LEFT                                                                                  |
| TC7  | ch3..ch7 covered, ch0..ch2 clear                                     | Hardware  | `getJunctionType()` → RIGHT                                                                                 |
| TC8  | ch0..ch5 covered (overlap, ch5 not in either strict predicate)       | Hardware  | `getJunctionType()` → NONE (no rule fires — by design)                                                      |
| TC9  | Channel-position validation: cover each channel index in turn        | Hardware  | The dropped channel index in `readActiveBoard()` output matches the physical position from left to right   |
| TC10 | 2-board config, `setReverseActive(true)` then read                    | Hardware  | I²C transaction goes to `LINE_REAR_I2C_PORT`; `setReverseActive(false)` reverts                            |
| TC11 | `calibrate()` invoked via BTN_DOWN long-press, sweep bar 5 s         | Hardware  | Serial prints `LINE_CALIB_LOW` and `LINE_CALIB_HIGH` blocks in paste-ready C++ form                         |
| TC12 | 2 m straight line at NORMAL preset                                   | Hardware  | Lateral error ≤ ±5 mm (B1.1 re-validation)                                                                  |
| TC13 | T-intersection × 10 at SLOW preset                                   | Hardware  | `Intersection.isAtIntersection()` fires exactly once per junction (B1.2 re-validation)                      |
| TC14 | Force I²C read failure on active board (unplug mid-run)              | Hardware  | LOST_LINE_ACTION (STOP) fires; no recovery attempted; clean restart on plug-back                            |
| TC15 | Reverse-direction follow with 2 boards installed                     | Hardware  | `setReverse(true)` activates rear board; line-follow still works backwards                                  |
| TC16 | Hardware calibration / `setLight()` ran in `begin()`                  | Hardware  | Each configured board's on-board LED comes on at the configured `LINE_LED_PWM` brightness by `begin()` exit |

**Test data:** A printed 2 m straight strip + one T-junction strip from the Phase 1 P1.4 bench fixture. No fake data sources — Arduino sketches run on real hardware.

**Run command:** Arduino IDE → Verify, then Upload. Manual observation per TC.

## Tasks

| ID  | Task | Blocked By | Risk | Files | Description |
|-----|------|------------|------|-------|-------------|
| T1  | Add new Config constants for 8-ch sensor | — | low | `WRO2026/Config.h` | Add `LINE_BOARD_COUNT`, `LINE_FRONT_I2C_PORT`, `LINE_REAR_I2C_PORT`, `LINE_CALIB_LOW[2][8]`, `LINE_CALIB_HIGH[2][8]`, `LINE_CHANNEL_ON_RATIO`, `LINE_JUNCTION_T_MIN_ON`, `LINE_JUNCTION_LEFT_HI`, `LINE_JUNCTION_RIGHT_LO`, `LINE_LED_PWM`. Remove `LINE_I2C_PORT` and `LINE_THRESHOLD`. Doxygen each constant explaining the 0..9 range, the two-layer calibration (firmware vs software) and the paste workflow. Satisfies AC9. |
| T2  | Write `WROLineArray.h` | T1 | med | `WRO2026/WROLineArray.h` | Declare class per the Data Model. Full doxygen on every public method matching the existing project style. Class-header doxygen MUST document the two-layer calibration (hardware in `begin()`, software in `Config::LINE_CALIB_*`) so a future maintainer does not delete one thinking it is redundant. Mark the byte-packing inference with a `@note` pointing to TC9. Declare `extern WROLineArray LineArray;`. Satisfies AC1. |
| T3  | Implement `WROLineArray.cpp` including calibrate() and hardware-layer init | T2 | high | `WRO2026/WROLineArray.cpp` | Implement `begin()` (init both `MatrixColor` instances, invoke the firmware-level hardware-calibration / illumination command — e.g. `_boards[i].setLight(true, false, Config::LINE_LED_PWM)` — on every configured board exactly once, log if rear missing). Implement `_readBoardInternal()` (unpack 4 CMYK bytes per the table), `_computeError()` (weighted center, normalize per software calibration, ratio-threshold), `_isOnlineFromCache()`, `_classifyJunction()` (priority T → LEFT → RIGHT → NONE per Decisions), `setReverseActive()`, and `calibrate()` (5 s sweep, Serial print in paste-ready C++ format). Include the `_sensorWeights[8] = {-3.5..+3.5}` const. Satisfies AC2, AC3, AC4, AC5 (mechanism), AC6, AC10, plus TC1–TC8, TC10, TC11, TC14, TC16. |
| T4  | Retype `LineFollower._tracer` to `WROLineArray*` and wire setReverse | T2 | med | `WRO2026/LineFollower.h` | Change `MatrixLineTracer _tracer;` to `WROLineArray* _array = nullptr;`. Rename `getTracer()` → `getArray()` returning `WROLineArray*`. Update doxygen for `setReverse(bool)` to mention active-board switching. No logic changes. |
| T5  | Port `LineFollower.cpp` to the new array | T3, T4 | med | `WRO2026/LineFollower.cpp` | Replace `_tracer.begin()` / `_tracer._pWire` / `_tracer._ch` / `_tracer.setThreshold(...)` block in `begin()` with `_array = &LineArray;` (binding only — `LineArray.begin()` is called by the sketch). Replace `_tracer.isOnline()` and `_tracer.getError()` calls in `update()` with `_array->isOnline()` / `_array->getError()`. In `_applyEnableBuilder`, after every write to `_reverse` (including `hasReverseOverride` branch and the direct `setReverse()` non-fluent setter), call `_array->setReverseActive(_reverse)`. Update `getTracer()` → `getArray()`. Satisfies AC5 (wiring), AC7 (when calibration values are in). |
| T6  | Retype `Intersection._tracer` to `WROLineArray*` | T2 | low | `WRO2026/Intersection.h` | Change `MatrixLineTracer* _tracer = nullptr;` to `WROLineArray* _array = nullptr;`. No public-API changes. |
| T7  | Port `Intersection.cpp` to the new array | T3, T5, T6 | low | `WRO2026/Intersection.cpp` | Replace `LineFollower.getTracer()` with `LineFollower.getArray()` and store as `_array`. Replace `_tracer->getJunctionType()` with `_array->getJunctionType()`. Keep the UNKNOWN / range-validation branches structurally intact (defensive, documented as currently-unreachable). Satisfies AC8. |
| T8  | Wire `WRO2026.ino` for begin() and calibration trigger | T5, T7 | med | `WRO2026/WRO2026.ino` | Add `#include "WROLineArray.h"`. Add `LineArray.begin();` in `setup()` after `MiniR4.begin()` and BEFORE `LineFollower.begin()` (LineFollower binds to it). Add a pre-mission idle handler: if `BTN_DOWN` is held for ≥2 s before the mission start, invoke `LineArray.calibrate()` and reset. Do not break the existing DEMO_MODE / mission boot path. |
| T9  | Hardware bring-up + calibration | T8 | med | `(no source files)` | Flash, confirm LED on each configured board lit at boot (TC16), lift bar, hold BTN_DOWN 2 s, sweep bar over white + a black-line sample for 5 s, copy printed `LINE_CALIB_LOW` / `LINE_CALIB_HIGH` blocks into `Config.h` (manual edit), re-flash, run TC9 to validate the byte→channel mapping inference, run TC12 (B1.1) and TC13 (B1.2). If TC9 fails, fix the unpacking table in `_readBoardInternal()` and re-run. Satisfies AC2, AC4, AC7, AC8, AC10 with real hardware. |
| T10 | Update DECISIONS.md with the sensor swap rationale | T9 | low | `DECISIONS.md` | New section titled "8-channel custom line sensor replaces MXLineTracer". Cover: (a) why the swap (custom-for-competition hardware), (b) the abstraction shape (concrete class, no virtual), (c) the inferred byte-packing format and the validation gate (TC9), (d) the front+rear / setReverse() routing decision, (e) the two-layer calibration discipline (firmware in `begin()` + software in `Config::LINE_CALIB_*`), (f) the calibration workflow modelled on `Drive.calibrateMotorBias()`. |

### Dependency / parallelism notes

- T1 unblocks everything; nothing else can compile against the new constants until it lands.
- After T2, T4 and T6 (header changes) can run in parallel — both consume `WROLineArray.h`.
- T3 unblocks T5, T7, T8 (link-time dependency on the impl). T3 itself parallelizes with T4 and T6 because none of them touch the same files.
- T5 needs both T3 (impl) and T4 (header); T7 needs T3, T5, T6.
- T8 is the integration point — sequenced after T5 and T7.
- T9 is the only on-hardware task and is the gate before T10.
- No two tasks edit the same file (verified above).

## Notes for Implementer

- **Source-cited proof discipline:** the byte-packing table in `_readBoardInternal()` is the only inferred contract in this design. Mark it with a `@note` comment that cross-references TC9 and meat.ino lines 670–684. If TC9 fails, only the table needs editing — no caller-side changes.
- **Two-layer calibration:** the MXColor-protocol board ships with a firmware-level calibration / illumination layer (LED brightness, internal normalization). Invoke whatever the actual firmware exposes (the visible analogue is `MatrixColor::setLight(...)`) in `WROLineArray::begin()` so the on-board state is deterministic by the time `update()` runs. Our software per-channel `LINE_CALIB_LOW/HIGH` baselines sit on top of that, compensating for residual per-channel variance. Both layers run; do not delete one thinking it duplicates the other. Document this contract in the `WROLineArray.h` class header and reference it from any place that touches calibration.
- **`MatrixColor::getColor()` blocking behaviour:** `MatrixColor::i2cReadData()` uses synchronous `_pWire->requestFrom()` with no explicit timeout; on the I²C bus this is typically a few hundred µs per byte. Calling `readActiveBoard()` is therefore four blocking I²C transactions ≈ 1–2 ms per `update()` tick. This is within the existing tick budget — `LineFollower.update()` was already paying the same cost on the 10-ch tracer (which read all 10 sensors per `getError()`).
- **Reverse-direction error sign:** the `LineFollower.update()` P+D mix is direction-symmetric (per the existing reverse-line-follow design); we do NOT need to negate `getError()` when the rear board is active. The rear board's leftmost channel (ch0) is still on the chassis's *left* when reversing (rear-mounted bar's left = chassis left), so weights stay the same. Worth confirming in TC15 during T9.
- **Dead-channel handling:** if calibration produced `LOW[i] >= HIGH[i]`, `_computeError()` treats channel `i` as permanently off (zero contribution). Critical so a botched paste does not silently fight the controller.
- **The `Intersection` UNKNOWN branch becomes structurally unreachable** with the new backend (our `_classifyJunction` never emits UNKNOWN). T7 leaves the code intact for safety and documents this fact in a `@note`. If a future backend reintroduces ambiguous samples, the existing handling already covers them.
- **No changes to the fluent builder** (`LineFollowerEnableBuilder`), the distance gate, the trapezoidal ramps, the lost-line dispatch, the `MatrixColor` library itself, or the path planner. The blast radius is intentionally narrow.
- **Rollback plan:** the swap deletes the `MatrixLineTracer*` field but does not touch the `MatrixMiniR4` vendor source. Reverting is one git revert of T4/T5/T6/T7 + restoring `Config::LINE_I2C_PORT` and `Config::LINE_THRESHOLD` (T1). `WROLineArray.h/.cpp` can stay in-tree as orphan files. The whole change is one PR.
- **Phase 1 benchmarks are re-validated**, not replaced. AC7 and AC8 reuse the B1.1 / B1.2 numeric criteria from Roadmap.md §13, which is the right hurdle to clear since this is a sensor swap, not a behavioural change.
