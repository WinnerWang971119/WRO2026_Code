# External Gyro Yaw Source Migration Plan

**Issue:** [#163](https://github.com/WinnerWang971119/WRO2026_Code/issues/163) — *nav: migrate yaw source from on-board STM32 IMU to an external gyro (contained to YawSource)*

**Goal:** Move the robot's yaw **read** provider from the on-board STM32 IMU to the external
MXColor-protocol gyro on I2C3, contained to `YawSource`, and ship a standalone calibration
sketch that measures the gyro's scale, handedness, and zero-rate drift.

**Approach:** Extract the pure decode math into a header-only `GyroDecodePolicy.h` that takes
scale and sign as **parameters** (the `TurnSignPolicy.h` / `PivotTrimPolicy.h` precedent, and the
same reasoning ADR-045 used to make its direction logic host-verifiable). `YawSource.cpp` owns the
I²C transport, a tick-scoped cache, and read validation; `YawSource.h` stays light (`<stdint.h>`
only) so `Drive.h` / `Localization.cpp` keep including it cheaply. Three measured constants
(`GYRO_SCALE`, `GYRO_YAW_SIGN`, `GYRO_I2C_PORT`) land in `ConfigNav.h`. The calibration sketch
`#include`s the same policy header and varies the constants as parameters, so the protocol math
exists in exactly one place and the bench tool adds zero production weight.

`toHwHeading` is **unchanged** — it still converts app→STM32-raw for `MoveGyro*`. Replacing that
write path is explicitly out of scope (tracked by a new follow-up issue, T10).

---

## Scope

- **In scope:**
  - `src/nav/GyroDecodePolicy.h` — header-only raw decode, wrap, plausibility gate, dead-byte detect.
  - `YawSource` — new `begin()` / `update()` / `zeroHardware()` / `isHealthy()`; `readYawDeg` /
    `readYawDegOr` re-pointed at the external gyro. Public read signatures unchanged.
  - `Config` — `GYRO_I2C_PORT`, `GYRO_SCALE`, `GYRO_YAW_SIGN`, `GYRO_MAX_STEP_DEG_PER_S`,
    `GYRO_CACHE_MAX_AGE_MS` + `ConfigChecks.h` static_asserts.
  - Sketch wiring — `YawSource.begin()` in `setup()`, `YawSource.update()` first in `cooperativeTick()`.
  - `inoTest/gyro_calib/` — standalone calibration sketch (scale, sign, zero-rate drift) that
    Serial-prints paste-ready constants.
  - Host tests — new `GyroDecodePolicy` pure-math target; reworked `test_yaw_source`; a `MatrixColor`
    stub **plus a `mockYawDeg` bridge** in the shared HAL mock so the 17 existing yaw-driving suites
    survive unchanged.
  - ADR-071 (two-sign split / second frame crossing) + `DECISIONS.md`, `ARCHITECTURE.md`, `CLAUDE.md` updates.
  - A follow-up GitHub issue for the GYRO **write** path.

- **Out of scope:**
  - **Replacing the GYRO move / `MoveGyro*` write path** — already in progress on another branch;
    this plan only files the issue. `toHwHeading` and `IMU_YAW_SIGN` stay exactly as they are.
  - **Any change to `Drive` / `Localization` consumers** — they keep consuming app-frame CCW-positive
    yaw through the unchanged read signatures. (Enforced: a diff touching `Drive.cpp` or
    `Localization.cpp` means the seam leaked.)
  - **Runtime drift compensation** — the sketch reports °/min; nothing subtracts it. Drift is
    temperature- and run-dependent, and a wrong constant is worse than none.
  - **Noise / resolution characterization** — explicitly dropped from the calibration scope.
  - **A code guard on the GYRO split-brain** — see Decisions; documented + issue-tracked only.
  - **The mentor's `M_Turn_angle` gain retune** (Kp 1→2, Kd 3→10, floor 15→20, debounce dropped) —
    belongs to the turn / GYRO-move work on the other branch. Recorded below only because it makes
    an existing memory stale.
  - Removing the STM32 IMU as a device (it still backs `MoveGyro*`).

---

## Decisions

- **Protocol source of truth:** the screenshots in `WRO2026/Meat_code/` (dated 2026-07-17), **not**
  either `.mbr4` on disk — both files predate them and use the older `getH()` read. Confirmed by
  decoding both files: `wro20260714_大通道gyro+16路2ms_RT.mbr4` reads `MXcolor_getColor(I2C3, H)`
  ×0.996; `wro2026junior_gyro.mbr4` doesn't use the external gyro at all (only vendor
  `MoveGyroPID`/`TurnGyroPID`). The screenshots' two-register decode is also technically stronger —
  see *Why the protocol changed* below.
- **Driver:** reuse the vendor `MatrixColor` class (`Modules/Sensors/MiniR4ColorSensorExt.h`,
  addr `0x22`) — the same class `WROLineArray` already drives. No new driver. This answers the
  issue's "search the library first" directive.
  ⚠️ **Not** `MiniR4_MXColorV3` — that is a TCS34725 *colour* sensor with a near-identical name and
  a normalising `getR()`; it is a decoy and must not be used here.
- **Decode math location:** header-only `GyroDecodePolicy.h`, scale/sign as params — so the
  calibration sketch reuses the *exact* production math instead of duplicating it, and the math is
  host-testable without any mock.
- **Sign modelling:** add `GYRO_YAW_SIGN` (external gyro → app) and keep `IMU_YAW_SIGN` (app →
  STM32 raw, for `toHwHeading`). The two devices have physically independent handedness, so one
  constant can no longer serve both reads and writes. This is precisely the "third frame crossing"
  ADR-045's *Trigger to revisit* anticipated → ADR-071.
- **Caching:** tick-scoped cache inside `YawSource` (`update()` reads once per tick; all consumers
  serve from it), with a staleness backstop so a consumer that reads before/without a tick still
  gets a fresh value rather than a stale one forever. Measured tick is ~8 ms and a gyro read is
  ~4–6 ms with 2–3 reads/tick realistic, so an uncached read path would balloon the tick.
- **Failure detection:** `G == 0 && R == 0` → hard failure (`readYawDeg` returns `false`, `yawOut`
  untouched — the existing contract), plus a `begin()` connectivity pre-flight. The vendor read has
  no error channel, so a dead bus otherwise decodes to a plausible-looking heading. **The host mock
  bridge also maps the legacy `mmL.mockResult = ERROR` failure-injection to this dead-byte pattern**,
  so the 17 FSM suites' 'failed read deposits nothing' assertions keep working unchanged.
- **Torn reads: DEFERRED to bench (revised 2026-07-18, user decision).** The plan originally added a
  time-normalized plausibility gate + streak recovery to reject torn/impossible yaw jumps. In host
  testing it rejected the FSM mocks that teleport the heading between hops, and its necessity is
  unproven — the known-good `meat.ino` uses NO such gate and works on the real robot. **Dropped from
  this scope; dead-byte detection is kept.** A torn read is re-added only if the bench actually shows
  one (tracked by **#170**). `GyroDecodePolicy.h::gyroReadPlausible` stays (pure, unit-tested
  by T6) but is no longer CALLED by `YawSource`.
- **Hardware zero:** expose `YawSource::zeroHardware()` (a `getColor(K)` read-with-side-effect),
  call it once in `begin()`, and let the calibration sketch drive it too. `Localization`'s
  software offset remains the app-level zero — unchanged.
- **Drift:** report only. No runtime compensation.
- **Legacy host-test yaw surface:** 17 existing suites script yaw through `mmL.mockYawDeg`, a
  surface T4 retires. Rather than sweep 16 files, the HAL mock **bridges** it: `MatrixColor::getColor`
  synthesizes the gyro bytes from `mmL.mockYawDeg`, so those suites keep passing untouched. See T3.
- **GYRO split-brain interim:** **document + file an issue; no code guard.** *User's explicit,
  informed call* — the GYRO-move replacement is already in progress on another branch, so a guard
  would be thrown away. The hazard is written up in *Notes for Implementer* and in the follow-up
  issue. (Recorded here so this reads as a deliberate decision, not an oversight — reviewers should
  weigh it against that context.)
- **Assumptions carried unconfirmed** (the assumption batch went unanswered — proceeding on best
  judgment, per `--auto`):
  1. The gyro is wired to **I2C3** (mentor's port). Line boards are on I2C1/I2C4, so no conflict.
     Encoded as `Config::GYRO_I2C_PORT` so a rewire is a one-constant change.
  2. Raw counts are **degrees** (1 count ≈ 1°) — the only reading under which `raw × 0.997` then
     `mod 360` is meaningful. Sets effective resolution at ~1° (which the host-test bridge inherits).
  3. The gyro is a **third, separate board**, not fused with the line array.
  4. **Hardware-in-hand is UNCONFIRMED.** The issue says it "may become blocked if the hardware
     isn't in hand yet". AC-11/AC-12 (bench) are therefore gated — see *Acceptance Criteria*.
  5. **⚠️ It is UNKNOWN whether the gyro board tolerates `MatrixColor::begin()`.** That method
     ID-checks `Device_ID == 0x43` and, on match, writes a CONFIG **reset** (`0x10`, 500 ms, then
     `setting`). The mentor's flow never calls `begin()` — only `getColor`. If the board doesn't
     answer `0x43`, `begin()` returns `false` on healthy hardware; if it does answer but the reset
     has side effects (re-zero / bias re-estimation), `begin()` corrupts gyro state. T4 must
     therefore **not** hard-depend on the vendor `begin()` — see T4's two-path pre-flight.

### Why the protocol changed (background, load-bearing)

The gyro board masquerades as an MXColor board and stuffs a signed 16-bit heading into the
R/G/B registers. The two mentor revisions read it differently:

| | Old (`.mbr4`, on disk) | New (screenshots, **authoritative**) |
|---|---|---|
| Read | `getH()` → `updateHSV()` → `RGB2HSV(r,g,b).h` | `getColor(G)*256 + getColor(R) − 32768` |
| Scale | ×0.996 | ×0.997 |
| Tail | `floor()` | none (keeps float precision) |
| Cost | 3 register reads (~6–9 ms) | 2 register reads (~4–6 ms) |

`getH()` derives hue **on the R4** from three 8-bit registers through float max/min/delta math.
Its failure mode is *worse* than a sentinel: `RGB2HSV` (`MiniR4ColorSensorExt.cpp`) sets `h = 0`
when the channels are equal (`delta < 0.00001`), so a gray/degenerate reading silently **aliases a
legitimate 0° heading**. (Note: the "−1 if undefined" docstring belongs to the *other* class,
`MiniR4_MXColorV3::getH()`, and to `getColorID()`'s `COLOR_NONE` — **not** to this one. Do not
repeat that claim in ADR-071 or the issue body.) The two-register read recovers the board's true
signed 16-bit value directly: full precision, no hue-math artifacts, no gray aliasing, and one
fewer I²C transaction.

The same screenshots also show `M_Turn_angle` retuned versus the `.mbr4`. Out of scope here, but it
means the `reference-mentor-turn-pd-mbr4` memory is now partly stale (see *Notes for Implementer*).

---

## Acceptance Criteria

- [ ] **AC-1** `YawSource.readYawDeg()` returns the external gyro's heading in the app CCW-positive
      frame, wrapped to (−180°, 180°], with no change to its signature or `bool`/`yawOut-untouched`
      contract.
- [ ] **AC-2** `readYawDeg()` returns `false` and leaves `yawOut` untouched when the gyro reads
      all-zero bytes (dead bus).
- [ ] **AC-3 (revised — plausibility gate DEFERRED).** `readYawDeg()` is gated ONLY by dead-byte
      detection this scope (AC-2). The runtime plausibility gate is deferred to bench (see Decisions);
      `GyroDecodePolicy.h::gyroReadPlausible` remains as pure, unit-tested code (T6 TC9–TC12) but
      `YawSource` does not call it. Re-adding it is tracked by **#170**.
- [ ] **AC-4** `GyroDecodePolicy.h` decodes `(G,R)` → app degrees using `scale` and `sign` **passed
      as parameters** (no `Config` include), and is linked by production, the host test, and the sketch.
- [ ] **AC-5** The gyro device is read **at most once per cache window**; in the mock-clock host test,
      N consumer reads within one tick produce **exactly one** device read.
- [ ] **AC-6** `YawSource::begin()` returns `false` when **no device answers on the selected mux
      channel**, and the sketch's boot health gate surfaces it rather than booting `BOOT OK`.
      *(Scoped deliberately: a line board on a mis-set channel is itself an MXColor device and would
      answer — port misconfiguration is caught by the AC-9 static_assert, not at runtime.)*
- [ ] **AC-7** `YawSource::zeroHardware()` issues the `getColor(K)` read and is called once by
      `begin()`.
- [ ] **AC-8** `toHwHeading()` and `Config::IMU_YAW_SIGN` are byte-for-byte unchanged; no file under
      `src/drive/` or `src/nav/Localization.*` is modified.
- [ ] **AC-9** `ConfigChecks.h` rejects at compile time: a `GYRO_YAW_SIGN` other than exactly `±1.0f`;
      a `GYRO_SCALE` outside a sane band; a `GYRO_I2C_PORT` outside 1..4 or equal to
      `LINE_FRONT_I2C_PORT` / `LINE_REAR_I2C_PORT`.
- [ ] **AC-10** The calibration sketch Serial-prints paste-ready `constexpr` lines for `GYRO_SCALE`
      and `GYRO_YAW_SIGN`, plus a drift figure in °/min.
- [ ] **AC-11** *(bench — gated on hardware in hand)* The sketch's `GYRO_SCALE` lands inside the
      `ConfigChecks` band and is **repeatable across 3 runs to within ±0.005**. Proximity to the
      mentor's 0.997 is recorded as an observation, **not** a pass condition (our board may
      legitimately differ).
- [ ] **AC-12** *(bench — gated on hardware in hand)* With the measured constants pasted in, the
      robot's reported heading tracks a known 90° rotation to within ±2°.
- [ ] **AC-13** Full `ctest` is green — all **32** targets that link `YawSource.cpp` build and pass,
      including the 17 legacy suites that drive yaw via `mmL.mockYawDeg` (preserved by the T3 bridge).
- [ ] **AC-14** `arduino-cli compile` is green for `WRO2026/` and for `inoTest/gyro_calib/`.
- [ ] **AC-15** A follow-up issue exists for the GYRO write path, carrying the split-brain analysis. **(DONE: #169.)**

> **If hardware is not in hand:** AC-11/AC-12 are deferred, the PR is marked as such, and
> `GYRO_SCALE` / `GYRO_YAW_SIGN` ship at their documented placeholder defaults flagged
> **UNVERIFIED**. Everything else still gates.

---

## Wire Protocol / Decode Contract

**Device:** `MatrixColor`, I²C addr `0x22`, on mux channel `Config::GYRO_I2C_PORT - 1`
(⚠️ port label ≠ mux channel: I2C1→ch0 … I2C4→ch3).

**Registers** (`ColorType` → `ColorRegType`): `R = 4` → `Device_RED`, `G = 5` → `Device_GREEN`,
`K = 10` → `Device_BLACK`. Each `getColor()` is a fresh `i2cMUXSelect()` + 1-byte read (~2–3 ms).

**Read** (G = high byte, R = low byte):

```
raw16   = (uint16_t)G * 256u + (uint16_t)R          // 0 .. 65535
biased  = (int32_t)raw16 - 32768                    // -32768 .. 32767, units = degrees
appDeg  = GYRO_YAW_SIGN * GYRO_SCALE * biased       // app frame, unwrapped
wrapped = fmodf(appDeg, 360.0f)                     // C truncated fmod -> (-360, 360)
if (wrapped < -180.0f || wrapped == -180.0f) wrapped += 360.0f;
else if (wrapped > 180.0f)                   wrapped -= 360.0f;
// result range: (-180, 180]     (note: exactly -180 maps to +180)
```

The sign is folded into the same multiply as the scale and applied **before** the wrap. This is not
merely equivalent to post-wrap negation, it is strictly better: `sign * wrap(x)` can emit −180 (out
of the canonical range) when `wrap(x) == 180`, whereas `wrap(sign * x)` is always canonical. This is
the single application point of `GYRO_YAW_SIGN` (ADR-045's one-place rule, now per-frame — ADR-071).

**Zero:** `getColor(K)` — a **read with a side effect**. The returned byte is discarded.

**Dead-byte detect:** `G == 0 && R == 0`. Note this aliases `biased == -32768`, a value that is
legitimate in principle (~91 CW turns of accumulation) but unreachable in a 2-minute run given the
`begin()` hardware zero. Document the aliasing; do not "fix" it.

---

## Contracts & Interfaces

Single source of truth for every seam. A task conforms to the exact names/signatures here.

### `src/nav/GyroDecodePolicy.h` — owner **T2**; consumers **T4** (YawSource), **T6** (test), **T8** (sketch)

Un-namespaced inline free functions (the `TurnSignPolicy.h` / `PivotTrimPolicy.h` convention — **not**
a class or namespace). Header includes `<stdint.h>` + `<math.h>` **only** — no `Config.h`, no Arduino,
no MatrixMiniR4. This is what makes it linkable from the host test and the sketch alike.

```cpp
/// Reconstruct the board's biased 16-bit raw count. UNWRAPPED and unscaled — the
/// calibration sketch accumulates THIS across N*360 turns to derive the scale.
inline int32_t gyroRawBiased(uint8_t hiG, uint8_t loR);

/// Wrap a degree value to (-180, 180]. Exactly -180 maps to +180.
inline float gyroWrap180(float deg);

/// Decode the board's two bytes into an app-frame heading in (-180, 180].
/// Built on gyroRawBiased() — the sketch and production share that one decode.
/// @param scale Positive scale calibration (Config::GYRO_SCALE at the call site).
/// @param sign  Exactly +1.0f or -1.0f (Config::GYRO_YAW_SIGN at the call site).
inline float decodeGyroYawDeg(uint8_t hiG, uint8_t loR, float scale, float sign);

/// True when both bytes are zero — the vendor's indistinguishable-from-dead-bus pattern.
inline bool gyroBytesDead(uint8_t hiG, uint8_t loR);

/// Shortest signed angular difference a - b, in (-180, 180].
inline float gyroShortestDelta(float aDeg, float bDeg);

/// Time-normalized plausibility gate: rejects a step faster than maxRateDegPerS.
/// @param elapsedMs Milliseconds since the last good read. A caller passing 0 must be
///                  treated as "no bound" (accept) rather than dividing by zero.
inline bool gyroReadPlausible(float newDeg, float lastDeg, uint32_t elapsedMs, float maxRateDegPerS);
```

### `src/nav/YawSource.h` — owner **T4**; consumers **T5** (sketch wiring), **T7** (tests)

Header stays light: `#include <stdint.h>` only. The `MatrixColor` instance and cache live as
file-statics in `YawSource.cpp`, which is what keeps the SDK out of `Drive.h`.

```cpp
class YawSourceClass
{
public:
    bool           begin();                            // NEW  — wire+mux, pre-flight, zeroHardware
    void           update();                           // NEW  — tick-scoped cache refresh
    void           zeroHardware();                     // NEW  — getColor(K), discards the byte
    bool           isHealthy() const;                  // NEW  — last read passed dead+plausibility gates
    bool           readYawDeg(double& yawOut) const;   // UNCHANGED signature + contract
    double         readYawDegOr(double fallback) const;// UNCHANGED
    static int16_t toHwHeading(int16_t appHeadingDeg); // UNCHANGED — still app -> STM32 raw
};
extern YawSourceClass YawSource;
```

`readYawDeg` stays `const` and serves the file-static cache; it refreshes inline when the cache is
absent or older than `Config::GYRO_CACHE_MAX_AGE_MS`. **Do not** widen the signature — 32 host
targets and both consumers bind to it.

### `config/ConfigNav.h` — owner **T1**; consumers **T4**, **T8**

⚠️ Per the config-parse regex gotcha: the `#ifndef X_DEFAULT` / `#define X_DEFAULT <value>` lines
must carry **no trailing `//` comment**; put prose on the `constexpr` line instead.

| Constant | Type | Default | Meaning |
|---|---|---|---|
| `GYRO_I2C_PORT` | `uint8_t` | `3` | Port label; mux channel is `PORT - 1` |
| `GYRO_SCALE` | `float` | `0.997f` | Positive scale cal; **UNVERIFIED** placeholder (mentor's value on his board) |
| `GYRO_YAW_SIGN` | `float` | `-1.0f` | Exactly ±1. **UNVERIFIED — the placeholder is a coin-flip pending AC-11**; the mentor's block code carries no sign, so his handedness cannot be read off it. |
| `GYRO_MAX_STEP_DEG_PER_S` | `float` | `1000.0f` | Plausibility rate bound (≈8°/tick at 8 ms; well above any real chassis) |
| `GYRO_CACHE_MAX_AGE_MS` | `uint8_t` | `12` | Staleness backstop (~1.5 ticks) |

`ConfigChecks.h` (owner **T1**) — mirror the existing `GRABBER_RC_ID != SORTER_RC_ID` /
`BELT_M_ID != MOTOR_LEFT_ID` distinctness pattern already in that file:
- `static_assert(GYRO_YAW_SIGN == 1.0f || GYRO_YAW_SIGN == -1.0f)`
- a sane-band assert on `GYRO_SCALE` (e.g. `0.9f < GYRO_SCALE && GYRO_SCALE < 1.1f`)
- `static_assert(GYRO_I2C_PORT >= 1 && GYRO_I2C_PORT <= 4)`
- `static_assert(GYRO_I2C_PORT != LINE_FRONT_I2C_PORT)` and, when `LINE_BOARD_COUNT == 2`,
  `!= LINE_REAR_I2C_PORT` — **this is the real defence against a port misconfig**, since a line
  board on a mis-set channel would happily answer the runtime ID check.

**No value-pinning tripwire on `GYRO_SCALE`** — unlike a tuned drive constant it is *meant* to be
replaced by the calibration output.

### HAL mock — owner **T3**; consumers **T4**, **T7**, and all 32 YawSource-linked targets

⚠️ **`test/mocks/MatrixMiniR4.h` is a forwarding shim** whose entire body is
`#include "../hal_mocks/MatrixMiniR4.h"` (the shared mock-HAL trust boundary, ADR-049). The unified
facade lives in `hal_mocks/` and the shim carries no guard of its own. Therefore:

- **Add the types to `test/hal_mocks/MatrixMiniR4.h` ONLY.** Adding them to both headers would
  redefine `ColorType` / `MatrixColor` for the 6 legacy-regime targets. The earlier "26 → hal_mocks,  6 → mocks" split describes *include paths*, not *definitions* — every target ultimately sees the
  `hal_mocks/` facade.
- **Make all state `static inline` members (C++17) — do NOT add `extern` globals to `mocks.cpp` /
  `hal_mocks.cpp`.** The file already sets the precedent (`inline MockMMLower mmL;` — *"inline so no
  mock source file changes"*), the harness is `CMAKE_CXX_STANDARD 17`, and critically
  **SIX targets link BOTH `mocks.cpp` and `hal_mocks.cpp`** (`test_mock_hal`, `test_line_follower_offset`,
  `test_return_to_node_policy`, `test_turn_imu_watchdog`, `test_makeedge_canary_fail`,
  `test_mission_dispatch`), so a definition in each `.cpp` would be a duplicate symbol in all six.
- **`TwoWire` / `Wire1` do not exist anywhere in the mock world** (verified: zero hits across
  `mocks/` + `hal_mocks/`). T4 assigns `_pWire = &Wire1`, so T3 must add them or all 32 targets fail
  to compile.

```cpp
// hal_mocks/MatrixMiniR4.h  (ONLY — mocks/MatrixMiniR4.h forwards here)
class TwoWire { public: void begin() {} };
inline TwoWire Wire1;

typedef enum __ColorType { R = 4, G, B, C, M, Y, K } ColorType;

class MatrixColor {
public:
    uint8_t  _ch = 0;
    TwoWire* _pWire = nullptr;

    // --- test-settable state (static inline: no .cpp, no duplicate symbols) ---
    static inline bool     mockBeginOk      = true;
    static inline bool     mockUseRawBytes  = false;  // false => bridge from mmL.mockYawDeg
    static inline uint8_t  mockG            = 0;
    static inline uint8_t  mockR            = 0;
    static inline uint32_t mockKReadCount   = 0;      // getColor(K) calls  -> proves zeroHardware
    static inline uint32_t mockDeviceReads  = 0;      // incremented ONCE PER G-REGISTER READ,
                                                      // i.e. once per logical device sample (AC-5)
    static inline void resetMock();

    bool    begin() { return mockBeginOk; }
    void    setLight(bool, bool, uint8_t) {}
    uint8_t getColor(ColorType c);
};
```

**The `mockYawDeg` bridge (load-bearing).** 17 existing suites — `test_localization_offset`, both
`reanchor_at_wall`, `reanchor_heading`, and twelve `test_path_tracker_*` — script yaw **only** via
`mmL.mockYawDeg` (the documented *"park mockYawDeg at edge heading EACH tick"* pattern). T4 retires
that surface, which would strand all 17 with no owning task. Instead, since `mmL` is an inline global
in this same header, `MatrixColor::getColor` **synthesizes the bytes from it** when
`mockUseRawBytes == false`:

```
biased = lround(mmL.mockYawDeg * IMU_YAW_SIGN / (GYRO_YAW_SIGN * GYRO_SCALE))
raw16  = (uint16_t)(biased + 32768)
G -> raw16 >> 8      R -> raw16 & 0xFF
```

so that `readYawDeg()` still yields `mockYawDeg * IMU_YAW_SIGN` — exactly what
`test_path_tracker_harness.h` documents — and the 17 suites pass **unchanged**. TC11/TC12 set
`mockUseRawBytes = true` to inject dead/torn bytes directly.

- The 32 YawSource targets pin `GYRO_SCALE=1.0f` and `GYRO_YAW_SIGN=<IMU_YAW_SIGN's value>` via
  **`target_compile_definitions`** (an in-file `#define` is TU-local and would not reach the
  separately compiled `YawSource.cpp`), making the bridge exact.
- **Accepted consequence:** the bridge quantizes to whole degrees (1 count ≈ 1°), so a parked
  non-integer heading round-trips with up to ±0.5° error. This is the device's real resolution. If a
  suite proves tighter than that, **raise that suite's tolerance** — do not fake sub-degree precision
  in the mock. Flag any such suite in the PR.

### Naming (must match exactly across tasks)

`GyroDecodePolicy.h`, `gyroRawBiased`, `gyroWrap180`, `decodeGyroYawDeg`, `gyroBytesDead`,
`gyroShortestDelta`, `gyroReadPlausible`, `YawSourceClass::{begin,update,zeroHardware,isHealthy}`,
`Config::{GYRO_I2C_PORT,GYRO_SCALE,GYRO_YAW_SIGN,GYRO_MAX_STEP_DEG_PER_S,GYRO_CACHE_MAX_AGE_MS}`,
`MatrixColor::{mockBeginOk,mockUseRawBytes,mockG,mockR,mockKReadCount,mockDeviceReads,resetMock}`,
ctest names `gyro_decode_policy` / `yaw_source`, sketch dir `inoTest/gyro_calib/`.

### File ownership

| File | Owner | Consumers |
|---|---|---|
| `WRO2026/config/ConfigNav.h`, `config/ConfigChecks.h` | T1 | T4, T8 |
| `WRO2026/src/nav/GyroDecodePolicy.h` | T2 | T4, T6, T8 |
| `WRO2026/test/hal_mocks/MatrixMiniR4.h` | T3 | T4, T7, all 32 targets |
| `WRO2026/src/nav/YawSource.h`, `YawSource.cpp` | T4 | T5, T7 |
| `WRO2026/WRO2026.ino` | T5 | — |
| `WRO2026/test/CMakeLists.txt` | **T6 → T7 → T3b** (serialized — see Task Table) | — |
| `WRO2026/test/test_gyro_decode_policy.cpp` | T6 | — |
| `WRO2026/test/test_yaw_source.cpp` | T7 | — |
| `inoTest/gyro_calib/gyro_calib.ino` | T8 | — |
| `docs/decisions/070-*.md`, `DECISIONS.md`, `ARCHITECTURE.md`, `CLAUDE.md` | T9 | — |

---

## Error Handling

- **No device on the selected mux channel at boot** → `YawSource::begin()` returns `false`; the
  sketch's existing `_setupFailed` boot health gate reports a hard boot failure. Never boot
  `BOOT OK` with a dead gyro.
- **Dead bus mid-run (all-zero bytes)** → `readYawDeg` returns `false`, `yawOut` untouched.
  `Localization::getHeadingDeg()` already falls back to its last-good heading; `Drive` read sites
  already use `readYawDegOr`. No consumer change needed — this is why the contract is preserved.
- **Torn read (high/low bytes from different samples) → NOT guarded this scope (deferred to bench).**
  The runtime plausibility gate was dropped (see Decisions) because it rejected the FSM test mocks
  and its necessity is unproven (`meat.ino` omits it). If the bench shows a torn read (a ~255° one-
  tick glitch), re-add `gyroReadPlausible` (already present, pure, unit-tested) with a recovery rule
  so a read gap can't latch it out. Tracked by **#170**.
- **`isHealthy()`** → reflects whether the last device read was non-dead (not all-zero bytes); it no
  longer tracks a plausibility streak.
- **Cache stale (no `update()` this tick)** → `readYawDeg` refreshes inline past
  `GYRO_CACHE_MAX_AGE_MS`. Correctness never depends on the tick wiring being right.
- **`zeroHardware()` on a dead bus** → the `getColor(K)` byte is discarded, so failure is invisible
  here by construction; `begin()`'s pre-flight runs *before* it and is the real gate.

---

## Testing Strategy

**Levels:** Unit (host, pure math) · Integration (host, real `YawSource.cpp` + mock device) · Bench (hardware).

| ID | Test Case | Type | Expected Behavior |
|---|---|---|---|
| TC1 | `decodeGyroYawDeg(0x80,0x00,1.0,1.0)` (raw 32768 → biased 0) | Unit | `0.0°` |
| TC2 | Known raw → degrees at `scale=0.997`, `sign=+1` | Unit | Matches hand-computed value |
| TC3 | `sign=-1.0` mirrors TC2's result | Unit | Exact negation (pre-wrap) |
| TC4 | Wrap boundaries: `+180` stays `+180`; `-180` → `+180`; `>180` → −360; `<−180` → +360 | Unit | Result always in (−180, 180] |
| TC5 | `gyroRawBiased` at extremes: `(0x00,0x00)`→−32768; `(0xFF,0xFF)`→32767 | Unit | Exact; computed in `int32_t`, no overflow |
| TC6 | Large accumulation (biased near ±32767) scaled then wrapped | Unit | Correct wrapped value |
| TC7 | `gyroBytesDead(0,0)` true; `(0,1)`/`(1,0)` false | Unit | — |
| TC8 | `gyroShortestDelta(179,-179)` | Unit | `−2°` (not `358°`) |
| TC9 | `gyroReadPlausible(-179, 179, 8ms, 1000)` | Unit | `true` — the legitimate wrap is NOT rejected |
| TC10 | `gyroReadPlausible(100, 0, 8ms, 1000)` | Unit | `false` — 100° in 8 ms exceeds the rate bound |
| TC11 | Same 100° step but `elapsedMs = 500` | Unit | `true` — the bound scales with time (proves no latch-out) |
| TC12 | `gyroReadPlausible(..., elapsedMs = 0, ...)` | Unit | `true` — no divide-by-zero, treated as unbounded |
| TC13 | `readYawDeg` with bridged `mmL.mockYawDeg` | Integration | `true`; `yawOut == decodeGyroYawDeg(G,R,GYRO_SCALE,GYRO_YAW_SIGN)` — the sign is applied **once**, inside the policy |
| TC14 | `readYawDeg` with `mockUseRawBytes`, `mockG=0, mockR=0` | Integration | `false`; `yawOut` **untouched** — seed it non-zero and assert it survives |
| TC14b | `readYawDeg` with `mmL.mockResult = ERROR` (bridge → dead bytes) | Integration | `false`; `yawOut` untouched — proves the legacy failure-injection still fails a read |
| TC15 | ~~Implausible jump rejected~~ **DROPPED** (gate descoped 2026-07-18) | — | Plausibility deferred to bench; `gyroReadPlausible` still unit-tested by TC9–TC12 |
| TC16 | ~~Streak recovery~~ **DROPPED** (gate descoped 2026-07-18) | — | Streak recovery removed with the gate |
| TC17 | 3 consumer reads within one tick | Integration | `mockDeviceReads` increments **exactly once** (AC-5) |
| TC18 | Read with no `update()` and a stale cache | Integration | Refreshes inline; returns fresh value |
| TC19 | `begin()` with `mockBeginOk=false` | Integration | Returns `false` |
| TC20 | `begin()` on success (seed plausible non-zero G/R first, or the pre-flight fails before the zero fires) | Integration | Returns `true`; `mockKReadCount == 1` |
| TC21 | `toHwHeading(+90)` / `(−90)` | Integration | Unchanged: `IMU_YAW_SIGN × arg` (the existing TC4 block, kept verbatim) |
| TC22 | Compile-fail: `-DGYRO_YAW_SIGN_DEFAULT=0.5f` | Build | Build fails with the static_assert text (AC-9). Use `PASS_REGULAR_EXPRESSION` on the FULL message — **never `WILL_FAIL`** |
| TC23 | Full suite after T3 + T4 | Integration | All **32** YawSource-linked targets build + pass, incl. the 17 bridged legacy suites (AC-13) |
| TC24 | Operator N×360° turns, 3 runs | Bench | `GYRO_SCALE` in-band and repeatable to ±0.005 (AC-11) |
| TC25 | Known 90° rotation with measured constants | Bench | Reported heading within ±2° (AC-12) |
| TC26 | Stationary drift log | Bench | Prints a °/min figure; recorded in the PR |
| TC27 | Boot with the gyro unplugged | Bench | OLED/Serial shows a hard boot failure, **not** `BOOT OK` (AC-6) |

**Test data:** `GyroDecodePolicy` tests need **no mocks at all** (pure math, no Arduino/Config
surface) — mirror the `test_turn_sign_policy` target, which links nothing but the sketch include dir.
`YawSource` tests script `MatrixColor::mock*` statics directly and call `resetMock()` in setup.

**Run commands:**
```
cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/
uv run python inoTest/gyro_calib/build.py --build-only
```

---

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Gyro Config constants + checks | — | med | `WRO2026/config/ConfigNav.h`, `WRO2026/config/ConfigChecks.h` | Add the five constants per the Contracts table, each `#ifndef X_DEFAULT`-guarded. **No trailing `//` on the `#define` lines** (breaks the Python config-parse regex) — prose goes on the `constexpr` line. Add all four static_asserts from the Contracts section to `ConfigChecks.h` (which must stay last in the umbrella), mirroring the existing `GRABBER_RC_ID != SORTER_RC_ID` distinctness pattern. Doxygen both measured constants **UNVERIFIED — pending gyro_calib bench (AC-11)**. Satisfies AC-9. |
| T2 | `GyroDecodePolicy.h` pure math | — | med | `WRO2026/src/nav/GyroDecodePolicy.h` | Implement the six inline free functions exactly as pinned in Contracts & Interfaces. Include `<stdint.h>` + `<math.h>` ONLY — no `Config.h`, no Arduino, no MatrixMiniR4 (that is what makes it host- and sketch-linkable). `decodeGyroYawDeg` MUST be built on `gyroRawBiased` so the sketch and production share one decode. Compute in `int32_t` before any float. Apply `sign` in the same multiply as `scale`, **before** the wrap. `gyroReadPlausible` is **time-normalized** (`maxRateDegPerS × elapsedMs/1000`) and treats `elapsedMs == 0` as unbounded — no divide-by-zero. Doxygen every function. Satisfies AC-4. |
| T3 | `MatrixColor` + `TwoWire`/`Wire1` stub and `mockYawDeg` bridge in the HAL mock | — | **high** | `WRO2026/test/hal_mocks/MatrixMiniR4.h` | Add exactly what the Contracts "HAL mock" block pins. **`hal_mocks/` ONLY** — `mocks/MatrixMiniR4.h` is a forwarding shim (ADR-049); touching both redefines the types. Use `static inline` members and `inline TwoWire Wire1;` — **no `.cpp` edits** (mirrors `inline MockMMLower mmL;`), which also avoids duplicate symbols in the SIX targets that link BOTH `mocks.cpp` and `hal_mocks.cpp`. Implement the `mmL.mockYawDeg` → bytes bridge so the 17 legacy yaw suites survive untouched. Keep the mock THIN: scripted bytes + the bridge, never board behaviour. ⚠️ The vendor enum injects unscoped single-letter globals (`R,G,B,C,M,Y,K`) into every host target — same collision class as the known `WHITE` macro gotcha; if a target breaks on one of those identifiers, that is why. **High risk: fans out to all 32 targets that link `YawSource.cpp`.** Run FULL `ctest` before handing off. |
| T3b | Pin `GYRO_SCALE` / `GYRO_YAW_SIGN` on the 32 targets **(REQUIRED — not an optimization; 6 FSM suites fail without it via 0.997°/count quantization vs the 0.5° turn tolerance, proven during T4)** | T3 | med | `WRO2026/test/CMakeLists.txt` | Add `target_compile_definitions(<t> PRIVATE GYRO_SCALE_DEFAULT=1.0f GYRO_YAW_SIGN_DEFAULT=<IMU_YAW_SIGN value>)` to the 32 targets linking `YawSource.cpp`, so the T3 bridge round-trips exactly. An in-file `#define` is TU-local and will NOT reach the separately compiled `YawSource.cpp` — it must be a compile definition. **Serialized last on `CMakeLists.txt`** (after T6, T7). Then run FULL `ctest` (AC-13). |
| T4 | `YawSource` read-path migration | T1, T2, T3 | **high** | `WRO2026/src/nav/YawSource.h`, `WRO2026/src/nav/YawSource.cpp` | Re-point `readYawDeg`/`readYawDegOr` at the external gyro via `MatrixColor` (file-static in the .cpp; `_pWire = &Wire1`, `_ch = Config::GYRO_I2C_PORT - 1` — **port label ≠ mux channel**). Add `update()` (tick cache), `zeroHardware()` (`getColor(K)`, discard byte), `isHealthy()`. **`begin()` must NOT hard-depend on the vendor `MatrixColor::begin()`** (assumption #5: it ID-checks `0x43` and writes a CONFIG reset; the mentor never calls it and the board may not implement it). Implement the pre-flight as: attempt `MatrixColor::begin()`, but treat a `false` as *inconclusive* and fall through to the authoritative check — **a `getColor` G/R pair that is not all-zero**. Only an all-zero pair fails `begin()`. Leave a `@note` that whether the vendor `begin()`/CONFIG-reset is safe on this board is UNVERIFIED pending bench. Read G then R back-to-back — **nothing may tick between them** or the mux re-points. Gate every read through `gyroBytesDead` ONLY (**plausibility gate DROPPED this scope per user decision 2026-07-18** — do not call `gyroReadPlausible`, do not add streak state). **`toHwHeading` and the `readYawDeg` signature are untouchable.** Keep `YawSource.h` at `<stdint.h>` only. Satisfies AC-1,2,3,5,6,7,8. |
| T5 | Sketch wiring | T4 | med | `WRO2026/WRO2026.ino` | Call `YawSource.begin()` in `setup()` **before** `Drive.begin()` and `Localization.begin()` (both read yaw), folding its result into the existing `_setupFailed` boot health gate. Call `YawSource.update()` **first** in `cooperativeTick()` so consumers see a fresh value in the same tick. Update the required-init-sequence comment. Satisfies AC-6. |
| T6 | `GyroDecodePolicy` host test + CMake target | T2 | low | `WRO2026/test/test_gyro_decode_policy.cpp`, `WRO2026/test/CMakeLists.txt` | Cover TC1–TC12. Mirror the `test_turn_sign_policy` target: pure math, **no mocks, no SUT sources** — just the sketch include dir. Register as ctest name `gyro_decode_policy`. Add the TC22 compile-fail target using `PASS_REGULAR_EXPRESSION` on the full static_assert message (**never `WILL_FAIL`**). **Owns the first `CMakeLists.txt` append**; T7 then T3b follow. |
| T7 | Rework `test_yaw_source` | T4, T6 | med | `WRO2026/test/test_yaw_source.cpp`, `WRO2026/test/CMakeLists.txt` | Cover TC13, TC14, TC17–TC21 against the real `YawSource.cpp` + the T3 mock. **TC15/TC16 (the plausibility-gate + streak-recovery tests) are DROPPED — the gate was descoped 2026-07-18.** Add a case that a `mmL.mockResult = ERROR` bridged read makes `readYawDeg` return false with `yawOut` untouched (mirrors the retired path; the T3 bridge now honors mockResult). **The existing TC3 blocks (`readYawDeg` vs `mmL.mockYawDeg`, `getImuEulerCallCount`) are SUPERSEDED by TC13–TC15 and must be replaced** — they assert against the retired STM32 read path and cannot pass after T4. **The TC4 `toHwHeading` block is kept verbatim** (TC21) — it pins the untouched write path. For TC14/TC15 seed `yawOut` non-zero and assert it **survives**. For TC17 assert `mockDeviceReads == 1` across 3 reads. ALSO adapt two legacy suites the read-migration's new semantics broke (both test-tree, both root-caused during T4): (a) `test/drive_hardening/test_drive_hardening.cpp` — its `tickUntilFault` relied on the RETIRED read path's internal `delay(IMU_READ_RETRY_DELAY_MS)` to advance the mock clock so the `TURN_IMU_STALL_TIMEOUT_MS` watchdog trips; the gyro read has no such delay, so make `tickUntilFault` advance the mock clock itself (`mockAdvanceMillis`) per tick; (b) `test/test_reanchor_at_wall_strict.cpp` — it asserts sub-degree yaw boundaries (99.5/100.5) the whole-degree mock bridge cannot express; move those boundary values to whole degrees (per the mock's own doc: raise the suite's granularity, never fake sub-degree precision in the mock). |
| T8 | `gyro_calib` standalone sketch | T1, T2 | med | `inoTest/gyro_calib/gyro_calib.ino` | Standalone bench sketch (NOT production, no runtime weight). `#include` `GyroDecodePolicy.h` and vary `scale`/`sign` as **params** — never duplicate the decode; use `gyroRawBiased()` for the accumulation. Drive `MatrixColor` directly on `Config::GYRO_I2C_PORT`. Three phases under the established operator UX (BTN_UP advances, OLED prompts): (1) **sign** — prompt one CCW nudge, report measured handedness as ±1; (2) **scale** — operator hand-turns N×360° against a fixed mark, accumulate **unwrapped** `gyroRawBiased` deltas, then `scale = (N×360)/rawAccumulated`; (3) **drift** — hold still, log heading vs time, report °/min. Serial-print paste-ready `constexpr` lines + the drift figure. **No `build.py`** — on this base `inoTest/` sketches are bare `.ino` files (see `inoTest/tick_bench/tick_bench.ino`, `inoTest/gyro-move.ino`), NOT the `build.py` wrapper (that convention lives only on an unmerged branch). Follow the tick_bench.ino convention: document the exact `arduino-cli compile` command in the sketch's file header, reaching `GyroDecodePolicy.h` and `Config.h` via `--library WRO2026 --library WRO2026/src/nav` (each `src/<domain>` is a flat legacy-1.0 library root, so includes stay bare filenames: `#include "GyroDecodePolicy.h"`, `#include "Config.h"`). ⚠️ The sketch must tolerate the vendor `begin()` returning `false` (assumption #5) — proceed to raw reads regardless and say so on the OLED. Satisfies AC-10,11,12. |
| T9 | ADR-071 + doc updates | T4 | low | `docs/decisions/071-external-gyro-yaw-provider.md`, `DECISIONS.md`, `ARCHITECTURE.md`, `CLAUDE.md` | Write ADR-071 (~30–60 lines, repo shape): the rule is *two independent frame-crossing signs — `GYRO_YAW_SIGN` on the external-gyro read, `IMU_YAW_SIGN` on the STM32 write — each applied exactly once*. Cite ADR-045's "third frame crossing" trigger and cross-link both ways (ADR-045 is **extended, not superseded**). Reference the split-brain follow-up issue **#169** in the ADR's "See also" / trigger section. Do **not** repeat the false "getH() returns −1 on gray" claim — the real defect is `h = 0` aliasing a legitimate 0° heading. Add the one-line `DECISIONS.md` index entry + bump the status table. Update `ARCHITECTURE.md`'s YawSource module row and `CLAUDE.md`'s YawSource bullet. **Re-verify the next free ADR number at execution time** (`ls docs/decisions/` + check other branches) — 070 is this plan's snapshot, and the parallel GYRO-move branch may have taken it; if so, renumber the unmerged one. |
| T10 | File the GYRO write-path issue | — | low | *(none — `gh issue create`)* | **Read `CONTRIBUTING.md` first** (project rule for any issue/PR). Open an issue: *"nav: GYRO moves steer by the STM32 IMU while yaw is read from the external gyro (split-brain)"*. Body must carry the full analysis: `toHwHeading`'s 4 `MoveGyro*` sites (Drive.cpp:181, 270, 322, 720) close their loop on the STM32's **own** IMU; `MMLower` exposes no heading-injection API (only `SetIMUToZero` + a target); `Drive.cpp:530` seeds `_heldYaw` from `readYawDegOr`, so post-migration the STM32 gets a target in the **external gyro's zero** but servos against **its own** zero → an immediate large heading error, not slow drift; `BlindMode::GYRO_TSHAPE` is planner-selectable and used for wall-depart + line-seek. Note the fix is in progress on another branch, and reference this plan + ADR-071. Label `bug`, `P1: high`, `area: nav`. Satisfies AC-15. **DONE — filed as #169.** |

**Parallelism:** T1, T2, T3, T10 start immediately (no blockers). T4 is the join point. T6 runs as
soon as T2 lands; T5/T7/T9 follow T4; T8 needs only T1+T2 so it can run alongside T4. The
`CMakeLists.txt` chain is strictly **T6 → T7 → T3b**.

---

## Notes for Implementer

- **Fire this in a git worktree.** The worktree won't carry the gitignored `.venv` — run
  `uv sync --project <worktree>` before any `uv run` (the `gyro_calib` build wrapper needs it).
- **The `CMakeLists.txt` seam.** T6, T7 and T3b all append to `WRO2026/test/CMakeLists.txt` and are
  strictly serialized so two parallel agents never race on it. If the executor fans out, the
  **orchestrator owns the appends** — never let two code-writers edit it concurrently.
- **The 32-target blast radius is the single biggest risk.** `YawSource.cpp` is linked by 32 host
  targets. T3 must land before T4 compiles anywhere. After T3, after T4, and after T3b, run the
  **full** `ctest` — not just `yaw_source`. Also sanity-check `ctps_mocks/`: `ctps_dump` compiles the
  whole `Tasks.cpp`, so if any task path reaches `YawSource` it needs the stub too (verify by build,
  don't assume).
- **Do not use `MiniR4_MXColorV3`.** It is a TCS34725 colour sensor whose name is one character off
  the right class. The gyro speaks the `MatrixColor` protocol in
  `Modules/Sensors/MiniR4ColorSensorExt.h` (addr `0x22`). Getting this wrong yields a device that
  ID-checks against `0x44` and never responds.
- **`getColor(G)` then `getColor(R)` must be adjacent.** Each call re-runs `i2cMUXSelect()`, so if
  anything else touches the bus between them the second read lands on the wrong board. Never insert
  a tick callback, `delay`, or log between the two reads.
- **The vendor read blocks.** `i2cReadData` does `delay(1)` twice plus a 300 µs mux select, so the
  pair costs ~4–6 ms against a **measured** ~8 ms tick. That measurement is real (`inoTest/tick_bench`,
  2026-07-16: avg 8056 µs) — do **not** re-derive the tick from per-read costs; an earlier estimate
  was off by ~3.5×. If the tick regresses, run `tick_bench` and use the number.
- **Upside:** the STM32 path we're leaving (`mmL`, SoftwareSerial @57600) busy-spins up to 100 ms in
  `WaitData` on a garbled read. Moving the read to I²C removes that worst-case tick stall — and that
  same 100 ms stall is exactly why the plausibility gate must be time-normalized.
- **⚠️ Known hazard shipped on purpose — GYRO split-brain.** After T4, a `Drive.driveDist(x).setMode(GYRO)`
  will hand the STM32 a target derived from the **external gyro's** zero while the STM32 servos on
  **its own** IMU. The zeros are unrelated, so the first GYRO move is wrong by a large angle —
  immediately, not gradually. This is **accepted for this scope** by explicit user decision (the
  replacement is in flight on another branch); T10 files it. Do **not** add a guard, and do **not**
  quietly "fix" it by touching `Drive`. If you find yourself editing `src/drive/`, stop — the seam
  has leaked and AC-8 fails. Reviewers will (correctly) flag this; the rebuttal is the recorded
  decision, not a counter-argument.
- **`reference-mentor-turn-pd-mbr4` memory is now partly stale** — it documents the `.mbr4`'s old
  `getH()` read and old turn gains, both superseded by the 2026-07-17 screenshots. Refresh it when
  this lands.
- **Rollback:** every change is additive except `YawSource.cpp`'s read body. Reverting T4's
  `readYawDeg`/`readYawDegOr` internals to `mmL.GetIMUEuler` restores the STM32 provider with no
  consumer change — which is the whole point of the ADR-045 single-owner boundary this plan preserves.
