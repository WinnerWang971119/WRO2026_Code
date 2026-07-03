# On-device `cooperativeTick()` Benchmark Plan

**Goal:** Give the team a Serial-printed, on-device measurement of `cooperativeTick()` loop time — whole-tick min/avg/max µs, effective Hz with a good/marginal/too-long verdict, and a per-module breakdown — so "the tick is too long" becomes a concrete, measurable optimization target and a regression can be spotted when a new module or per-tick read lands. Closes issue #123.

**Approach:** A standalone diagnostic sketch under `inoTest/tick_bench/` (the existing `inoTest/` convention, matching `gyro-move.ino`) that reuses the **real** `WRO2026/` module sources — not host mocks, not a mirror — so it measures the real `update()` bodies and their vendor I2C cost. The min/avg/max accumulation logic is extracted into a header-only `WRO2026/TickProfiler.h` (pure arithmetic, no Arduino I/O in the accumulate path) so it can be unit-tested in the existing host ctest harness, while the actual timing runs only on hardware. The bench sketch wires the real modules exactly as `WRO2026.ino::setup()` does, then loops a `benchTick()` that calls each module's `update()` once — in `cooperativeTick()` order — each wrapped in a profiler segment.

## Scope

- **In scope:**
  - New `WRO2026/TickProfiler.h` — header-only, fixed-size, no dynamic allocation, no `delay()`; tracks per-segment and whole-tick min/avg/max in µs; getters; a `classify(floorHz, goodHz)` verdict; a member-template `report(OutT&)`; `reset()`.
  - New `inoTest/tick_bench/tick_bench.ino` — standalone sketch that reuses the real WRO2026 modules, runs the instrumented tick, prints a Serial summary every N ticks (then resets the window), and accepts a tiny single-char Serial command set (`?` help/state, `r` reset window now).
  - New host ctest `WRO2026/test/test_tick_profiler.cpp` (+ CMake target) exercising the profiler's pure accumulation math against injected samples.
  - A one-line maintainer breadcrumb comment in `WRO2026/WRO2026.ino` near `cooperativeTick()` reminding that module additions must be mirrored into the bench sketch.
  - The arduino-cli build command (how the bench sketch pulls in the real WRO2026 sources) documented in the sketch's header docblock.
- **Out of scope (with reason):**
  - A CI job that compiles or runs the bench sketch. The issue explicitly scopes CI *structural* guards ("count I2C reads / `update()` calls per tick against a budget") as a **separate follow-up**, and the sibling diagnostic sketches (`inoTest/gyro-move.ino`, `tools/vision_bench/vision_bench.ino`) are not CI-compiled either — the convention is that diagnostic sketches are manual. The arduino-cli command is documented so a CI compile-guard can be added later. (Tracked as a follow-up note below.)
  - Measuring the tick while a mission / blocking Drive move is *active*. The dominant per-tick cost (LineArray I2C, ~2–3 ms/read) is incurred every tick regardless of motion, so an idle bench tick is representative for the headline number. (Caveat: `Localization.update()` reads the IMU only while nav is INACTIVE — its operator-offset gate is open at idle, closed during a mission — so the idle split modestly OVER-weights `Localization` and under-weights the active-move work in `Drive`/`LineFollower`. The report docblock states this; it does not move the dominant `LineArray` finding.) A serial-triggered "measure under an active move" mode is a possible later extension, noted but not built.
  - Any change to the production control loop, tick order, or module behavior. The real `WRO2026.ino` loop is untouched except for a comment breadcrumb.
  - Structural per-tick I2C-read counting / budget assertions (the issue's own deferred follow-up).

## Decisions

- **Location & shape — separate sketch in `inoTest/`:** Per the user's Q1 answer ("2 at /inotest"): build it as option 2, a standalone diagnostic sketch in the existing `inoTest/` dir, rather than a compile-time flag inside `WRO2026.ino`. Keeps the production sketch essentially untouched; matches the established `gyro-move.ino` diagnostic-sketch convention.
- **Reuse the REAL modules, not a mirror (auto-resolved, forced by the issue):** Issue #123 states the benchmark "needs the real modules + real hardware" and that host mocks report "meaningless near-zero time." A self-contained mirror (the `gyro-move.ino` style) therefore cannot satisfy it. The bench sketch compiles the real `WRO2026/*.cpp`. This is the one place the new sketch diverges from the `gyro-move.ino` "only depends on MatrixMiniR4.h" precedent — deliberately, because measuring the real `update()` bodies is the entire point.
- **Build mechanic — treat `WRO2026/` as a legacy Arduino library:** Primary approach: `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi inoTest/tick_bench --library <repo>/WRO2026`. This adds the WRO2026 root to the include path (so `#include "Config.h"` / `#include "config/…"` resolve) and compiles its root `.cpp` files as library sources; `.ino` files are ignored by the library compiler, so there is no `setup()`/`loop()` collision with `WRO2026.ino`. Fallback if `--library` misbehaves: a small **copy** step — a `tick_bench/copy_sources` script (or an arduino-cli `sketch.yaml` profile) that stages the WRO2026 `.cpp/.h` into a **gitignored** `tick_bench/src/` (Arduino compiles `src/**`). A `src/`-**symlink** is explicitly NOT the fallback — Windows symlink creation needs Developer Mode / elevation and git checks symlinks out as plain text by default, so it is unreliable on this machine. **T1 retires this risk before any other work.**
- **`TickProfiler.h` lives in `WRO2026/`, not the bench folder:** It is a reusable header-only helper (peer of `WROMath.h` / the currently-unused `StateMachine.h`). Placing it in `WRO2026/` lets the host ctest harness include it by the normal path and lets a future in-loop use (if the team later wants always-on aggregate telemetry) reuse it, at the cost of one inert header added to the sketch. The bench sketch includes it via the `--library` include path.
- **Split pure math from Serial I/O so it is host-testable:** The accumulate/getter/classify path uses only `uint32_t` arithmetic (host-compilable); `report()` takes an Arduino `Print&` so the on-device path prints to `Serial` and the (optional) host path can pass a mock. This does not contradict the issue's "host can't measure timing" — the host test injects *known* sample values and asserts the arithmetic, it does not time a real tick.
- **Thresholds — 30 Hz floor / 45 Hz good (Q2):** `classify()` returns TOO_LONG below 30 Hz, MARGINAL between 30–45 Hz, GOOD at ≥45 Hz. The current ~35 Hz I2C-capped rate lands in the MARGINAL band, so any new per-tick read that pushes below 30 Hz flags immediately.
- **Reporting cadence — every N ticks, reset each report (Q3):** Accumulate over `WINDOW_TICKS` (default 200), print one summary block, then `reset()` the window. Matches the issue's "print every N iters" example and gives a deterministic sample count per report. `WINDOW_TICKS` and the two Hz thresholds are named `constexpr` **local to the bench sketch** (bench-only tunables; keeps `WRO2026/config/` untouched, consistent with `gyro-move.ino` defining its own local constants). No-magic-numbers still honored — they are named constants.
- **Segment set = the 10 `cooperativeTick()` members, in order:** `LineArray, Drive, LineFollower, Intersection, Manipulator, Tasks, PathTracker, Localization, Mission, EventTrace`. `Vision` is intentionally excluded from the *per-tick* set — it is capture-once (`readMosaic()` from a task), not ticked in `cooperativeTick()`. Note the asymmetry: the bench still calls `Vision.begin()` in `setup()` (11 `begin()`s total, matching `WRO2026.ino`), but registers/measures only the 10 ticked modules as segments.
- **The bench registers NO tick callbacks (deliberate divergence from `setup()`):** `WRO2026.ino::setup()` registers `cooperativeTick` on 6 modules via `setTickCallback` so sync-blocking drains keep everyone alive. The bench issues no blocking Drive move (no mission, no `goTo`), so no drain ever fires — and registering `benchTick` would risk re-entering the profiler mid-segment and corrupting spans. So the bench runs all 11 `begin()`s in setup order but calls **zero** `setTickCallback`. This is the one intentional wiring divergence from `setup()`; T3 and the report docblock note it.
- **Drift mitigation is a human breadcrumb (X-macro alternative considered, deferred):** Two sketches independently list the tick order, so a module added to the real tick could be missed in the bench. Chosen mitigation: a comment breadcrumb in both files (and this spec). A stronger alternative was considered and rejected *for now*, not dismissed as impossible: a shared `WRO2026/TickModules.h` defining a `TICK_MODULES(X)` X-macro expanded by BOTH `cooperativeTick()` and the bench's register/measure loop, which would turn drift into a compile error. It is deferred because it rewrites the production `cooperativeTick()` body to expand the macro (a real change to runtime firmware, which the user's "separate sketch, minimal-touch" choice deliberately avoids) and couples the cal-menu-owning `.ino` to the bench. Recorded as a follow-up; if drift ever bites, adopt it.

## Acceptance Criteria

- [ ] `inoTest/tick_bench/tick_bench.ino` compiles clean via `arduino-cli` for `arduino:renesas_uno:unor4wifi`, pulling in the **real** `WRO2026/*.cpp` (no mocks), using the documented `--library` command. (AC-build)
- [ ] On device, the sketch runs all **11** module `begin()`s (including `Vision.begin()`) in the same order as `WRO2026.ino::setup()` and registers **zero** tick callbacks, then loops a `benchTick()` calling each of the **10** ticked modules' `update()` once per pass in `cooperativeTick()` order (Vision is not ticked). (AC-wire; the 10-module order is made observable by TC9's ordered-`SEG` assertion)
- [ ] Every `WINDOW_TICKS` passes, a Serial summary prints: whole-tick `min / avg / max` µs, effective Hz (from avg), and a `GOOD / MARGINAL / TOO_LONG` verdict against the 45/30 Hz thresholds; the window then resets. (AC-report)
- [ ] The same summary includes a **per-module** line for each of the 10 segments with its `min / avg / max` µs, so the dominant segment is visible at a glance. (AC-breakdown)
- [ ] `WRO2026/TickProfiler.h` performs no dynamic allocation and contains no `delay()`; all storage is fixed-size; the accumulate/getter path has no Arduino I/O dependency. (AC-rules; verified by review + the host compile of `test_tick_profiler`, which links `TickProfiler.h` with no Arduino runtime)
- [ ] `WINDOW_TICKS` and both Hz thresholds are named `constexpr` (no magic numbers); the exact `--library` build command is documented in the sketch header docblock. (AC-constants; verified by review)
- [ ] Serial command `?` prints help + the current window state (ticks-so-far, N, thresholds); `r` resets the window immediately. (AC-serialcmd; `r` by TC10, `?` by TC11)
- [ ] Host ctest `test_tick_profiler` passes: injected dt samples yield the expected per-segment and whole-tick min/avg/max, effective Hz, verdict, `reset()`, empty-window (min→0), and `addSegment` overflow behavior; `report()` output matches the Contracts→Naming grammar. (AC-hosttest)
- [ ] `WRO2026/WRO2026.ino` carries a breadcrumb comment at `cooperativeTick()` telling maintainers to mirror module add/remove into `inoTest/tick_bench/`, and its stale `cooperativeTick()` docblock order list (missing `Tasks`) is corrected. (AC-breadcrumb)

## Contracts & Interfaces

Single source of truth for the seams between tasks. The `TickProfiler` API is produced by T2 and consumed byte-for-byte by the bench sketch (T3) and the host test (T4).

### `WRO2026/TickProfiler.h` — public API (owner: T2; consumers: T3, T4)

```cpp
#ifndef TICK_PROFILER_H
#define TICK_PROFILER_H

#include <stdint.h>

/** @brief Sentinel returned by addSegment() when the slot table is full; a no-op sampleSegment idx. */
static constexpr uint8_t  TICK_INVALID_SEGMENT = 0xFF;
/** @brief "no sample yet" minimum sentinel; getters map it to 0 on an empty window. */
static constexpr uint32_t TICK_US_MIN_SENTINEL = 0xFFFFFFFFu;

/** @brief Verdict of a measured average tick time against two Hz thresholds. */
enum class TickVerdict : uint8_t { GOOD, MARGINAL, TOO_LONG };

/**
 * @brief Fixed-size accumulator of per-segment + whole-tick timing (min/avg/max µs).
 * @tparam MAX_SEGMENTS compile-time upper bound on named segments (no dynamic alloc).
 * @note The accumulate/getter path uses only <stdint.h> arithmetic — NO Arduino I/O type — so
 *       the header host-compiles standalone. Output goes through the member template report(),
 *       whose stream type is DEPENDENT, so `Serial` (device) and a duck-typed host capture object
 *       both work with no `Print` base at all. (There is deliberately no `class Print;` forward
 *       decl: an inline `report(Print&)` would access members of an incomplete, non-dependent
 *       type and be rejected by g++ and MSVC `/permissive-`.)
 */
template <uint8_t MAX_SEGMENTS>
class TickProfiler
{
public:
    /** @brief Register a named segment slot; returns its index (0..MAX_SEGMENTS-1), or TICK_INVALID_SEGMENT if full. */
    uint8_t addSegment(const char* name);

    /** @brief Feed one segment's measured duration (µs) for the current tick. idx==TICK_INVALID_SEGMENT is a no-op. */
    void sampleSegment(uint8_t idx, uint32_t dtUs);

    /** @brief Feed the whole-tick measured duration (µs) and advance the tick count. */
    void sampleTick(uint32_t dtUs);

    // ---- pure getters (host-tested; no Arduino I/O) ----
    uint32_t tickCount()      const;   ///< ticks accumulated since last reset()
    uint32_t tickMinUs()      const;   ///< 0 on an empty window (sentinel mapped)
    uint32_t tickAvgUs()      const;   ///< sum / count; 0 when count == 0
    uint32_t tickMaxUs()      const;
    uint32_t segMinUs(uint8_t idx) const;   ///< 0 on an empty window
    uint32_t segAvgUs(uint8_t idx) const;
    uint32_t segMaxUs(uint8_t idx) const;
    uint32_t effectiveHz()    const;   ///< 1e6 / tickAvgUs(); 0 when avg == 0
    TickVerdict classify(uint32_t floorHz, uint32_t goodHz) const;

    /**
     * @brief Print the full summary (per-segment + whole-tick + Hz + verdict) to `out`.
     * @tparam OutT any object exposing Arduino-style print()/println() — `Serial` on device,
     *         a host capture object in the ctest. Member template ⇒ dependent stream type, so no
     *         `Print` base is needed and the header stays Arduino-I/O-free until instantiated.
     * @note Emits the EXACT line grammar pinned in Contracts → Naming; min/max print as 0 on an
     *       empty window (never the raw sentinel).
     */
    template <typename OutT>
    void report(OutT& out, uint32_t floorHz, uint32_t goodHz) const;

    /** @brief Zero every accumulator and tick count (segment names + registration survive). */
    void reset();
};
#endif // TICK_PROFILER_H
```

- `classify`: `hz = effectiveHz()`; `hz >= goodHz` → `GOOD`; `hz >= floorHz` → `MARGINAL`; else `TOO_LONG` (0 Hz / empty window ⇒ `TOO_LONG`).
- `reset()` preserves registered segment names/count (so the bench registers once in `setup()`), resetting min (→ `TICK_US_MIN_SENTINEL`) / max (→ 0) / sum / count.
- Averages are integer `sum / count`. With `WINDOW_TICKS ≤ 200` and per-tick times well under ~50 ms, both whole-tick and per-segment `sum` stay far below the `uint32_t` ceiling; T2 adds a `@note` documenting the bound.

### Bench-sketch tunables (owner: T3) — named `constexpr`, local to `tick_bench.ino`

- `WINDOW_TICKS = 200` — ticks per report window.
- `TICK_FLOOR_HZ = 30`, `TICK_GOOD_HZ = 45` — passed into `classify()` / `report()`.
- `SEGMENT_COUNT = 10` — the `MAX_SEGMENTS` template arg and the number of registered modules.
- Shared values (`BATT_CELLS`, motor IDs, etc.) come from the real `Config::` — not re-declared.

### File ownership

| File | Owner task | Consumer / editor tasks |
|------|-----------|------------------------|
| `WRO2026/TickProfiler.h` | T2 | T3, T4 (include only) |
| `inoTest/tick_bench/tick_bench.ino` | T1 (stub) → T3 (full) | — |
| `WRO2026/test/test_tick_profiler.cpp` | T4 | — |
| `WRO2026/test/CMakeLists.txt` | T4 | — |
| `WRO2026/WRO2026.ino` | T5 (comment only) | — |

### Naming (must match across tasks — single source of truth for the log grammar)

- Header guard `TICK_PROFILER_H`; type `TickProfiler<MAX_SEGMENTS>`; enum `TickVerdict{GOOD,MARGINAL,TOO_LONG}`; constants `TICK_INVALID_SEGMENT`, `TICK_US_MIN_SENTINEL`.
- Bench sketch folder + primary `.ino` basename must match: `inoTest/tick_bench/tick_bench.ino`.
- **Exact Serial report grammar** — produced by `report()` (T2), asserted byte-for-byte by T4/TC7, scraped by operators. Defined ONCE here so T2 and T4 cannot drift:
  - Window header line: `-- tick bench: <N> ticks --`
  - Per-segment line, one per registered module, **in registration order**: `  SEG <name> us min=<min> avg=<avg> max=<max>`
  - Whole-tick line, last: `TICK us min=<min> avg=<avg> max=<max> | ~<hz> Hz | <VERDICT>`
  - `<VERDICT>` is exactly one of `GOOD` / `MARGINAL` / `TOO_LONG`. All `<min>/<avg>/<max>` are integer µs and print `0` (not the sentinel) on an empty window; `<hz>` is integer Hz.

## Data Model

Not applicable — this is firmware instrumentation, no persisted data or schema. The only "state" is the `TickProfiler`'s fixed-size accumulators (see the API above).

## Error Handling

- **`addSegment` overflow** (more than `MAX_SEGMENTS` registered): returns `TICK_INVALID_SEGMENT`; `sampleSegment(TICK_INVALID_SEGMENT, …)` is a no-op (bounds-checked). The bench registers exactly `SEGMENT_COUNT` and sizes the template to match, so this cannot fire at runtime — it is a defensive guard, host-tested.
- **`begin()` failure of a real module in the bench `setup()`:** print a one-line `[bench] <module> begin() FAILED` to Serial and continue (unlike the mission sketch, the bench does not need every module healthy to produce useful timing; a failed board simply shows near-zero time for its segment). Do **not** halt.
- **Division by zero in `effectiveHz()` / `tickAvgUs()`:** guarded — return 0 when `count == 0` (e.g. a report requested before any tick). Host-tested.
- **Vendor read returning 0 as a "value" vs a failure:** out of scope here — the bench measures *time spent in* `update()`, not the read's correctness, so a silent all-zero read still contributes its real duration. (See the known vendor-read-no-error gotcha; it does not affect timing.)
- **Serial not connected (headless power-on):** `setup()` uses the same `while (!Serial && millis() < 3000)` bounded wait as `gyro-move.ino` so a headless boot does not hang; summaries are simply produced with no listener.

## Testing Strategy

**Levels:** Host unit test (profiler math, CI-enforced) + arduino-cli compile check + on-device manual verification (the actual timing — hardware only).

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | `sampleSegment`/`sampleTick` fed a known set of dts, then getters read | Host unit | `min`/`max`/`avg`(=sum/count) exact for each segment and whole-tick |
| TC2 | `effectiveHz()` from a known avg (e.g. avg=28571 µs) | Host unit | Returns 35 (1e6/avg, integer) |
| TC3 | `classify(30, 45)` at avg→50 Hz, 35 Hz, 20 Hz | Host unit | `GOOD`, `MARGINAL`, `TOO_LONG` respectively |
| TC4 | `classify`/`effectiveHz`/`tickMinUs`/`report` on a zero-tick window | Host unit | `effectiveHz()==0`, `tickAvgUs()==0`, `tickMinUs()==0` (sentinel mapped, not `0xFFFFFFFF`), verdict `TOO_LONG`, `report()` prints `min=0 … max=0`, no div-by-zero |
| TC5 | `reset()` after accumulating | Host unit | counts→0, min→`TICK_US_MIN_SENTINEL`, max→0, avg→0; segment names/count retained |
| TC6 | `addSegment` beyond `MAX_SEGMENTS`; then `sampleSegment(TICK_INVALID_SEGMENT,…)` | Host unit | Returns `TICK_INVALID_SEGMENT`; the stray sample is a no-op, no OOB write |
| TC7 | `report(cap, 30, 45)` over a known window, `cap` = duck-typed host capture object (exposes `print`/`println`; NO `Print` base — none exists in the harness) | Host unit | Output matches the Contracts→Naming grammar exactly: `-- tick bench: <N> ticks --`, one `  SEG <name> us min=.. avg=.. max=..` line per segment **in registration order**, then `TICK us … | ~..Hz | <VERDICT>` with the correct verdict token |
| TC8 | arduino-cli compile of the bench sketch with `--library WRO2026` | Compile | Exit 0; real WRO2026 `.cpp` objects link; no multiple-definition, no missing `setup()`/`loop()` |
| TC9 | Flash to device, observe Serial | On-device (manual) | A summary block prints every `WINDOW_TICKS`; the 10 `SEG` lines appear in exactly the documented `cooperativeTick()` order (registration order == print order, so wiring order is observable); whole-tick avg is a plausible ~20–35 ms (~30–50 Hz); `LineArray` is the dominant segment |
| TC10 | Send `r` over Serial mid-window | On-device (manual) | Window resets immediately; next block reflects only post-reset ticks |
| TC11 | Send `?` over Serial | On-device (manual) | Prints help text + current window state (ticks-so-far, N, floor/good Hz); does NOT reset the window |
| TC12 | Boot the bench with one line board disconnected | On-device (manual) | `[bench] LineArray begin() FAILED` prints; the sketch does NOT halt; summary blocks still print (that segment reports near-zero/degraded time) |

**Test data:** TC1–TC7 use a tiny host **capture struct** (a `CaptureStream` with Arduino-style `print()/println()` overloads that append to a `std::string`; NO Arduino `Print` base — none exists in `mocks/`) plus hand-authored dt vectors; no hardware. TC8 uses the real repo tree + arduino-cli. TC9–TC12 need the assembled chassis with the line board(s) wired.
**Run commands:** Host — `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`. Compile — the documented `arduino-cli compile … --library …WRO2026` line. On-device — flash `inoTest/tick_bench/` via Arduino IDE / arduino-cli, open Serial @ 115200.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|-----------|------|-------|-------------|
| T1 | Prove the real-source build mechanic | — | high | `inoTest/tick_bench/tick_bench.ino` | Create a minimal stub sketch: `#include <MatrixMiniR4.h>` + a handful of real WRO2026 headers (`Config.h`, `Drive.h`, `WROLineArray.h`) and empty `setup()`/`loop()` that reference one real module. Get it to compile via `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi inoTest/tick_bench --library <repoRoot>/WRO2026`. If `--library` fails, establish the `src/`-symlink fallback and document whichever works. Record the exact working command in the sketch header docblock. Satisfies AC-build. This retires the plan's central risk — do it first. |
| T2 | Write `WRO2026/TickProfiler.h` | — | med | `WRO2026/TickProfiler.h` | Header-only `template<uint8_t MAX_SEGMENTS> class TickProfiler` per the Contracts API exactly (guard `TICK_PROFILER_H`, `TickVerdict`, named `TICK_INVALID_SEGMENT` / `TICK_US_MIN_SENTINEL` — **NO `Print` forward-decl**). Fixed arrays sized by `MAX_SEGMENTS`; min inits to `TICK_US_MIN_SENTINEL`, getters map it to `0` on an empty window; integer avg = sum/count with a `@note` on the `uint32_t` sum bound; guarded zero-count getters; `classify` per the two-threshold rule. **`report()` is a member template** `template<typename OutT> void report(OutT&, floorHz, goodHz)` emitting the EXACT Contracts→Naming grammar (member template ⇒ dependent stream type ⇒ compiles under g++/MSVC `/permissive-`, needs no `Print` base). No `delay()`, no dynamic alloc, Doxygen on every public member; add a one-line `@note` that the ~2·(segments+1) `micros()` calls/tick are µs-scale, negligible vs the ms-scale I2C measured. Satisfies AC-rules, AC-constants. |
| T3 | Flesh out the bench sketch | T1, T2 | med | `inoTest/tick_bench/tick_bench.ino` | Replace the T1 stub with the full sketch: include `<MatrixMiniR4.h>`, `TickProfiler.h`, and the real module headers. `setup()` mirrors `WRO2026.ino::setup()` — `MiniR4.begin()`, `Serial.begin(115200)`, bounded `!Serial` wait, `setBattCell(Config::BATT_CELLS)`, all **11** `.begin()`s in the same order **including `Vision.begin()`**, `[bench] <module> begin() FAILED` on any false (no halt). **Register NO tick callbacks** (deliberate divergence from setup() — see Decisions; the bench runs no blocking move so none are needed and they would risk profiler re-entrancy). Register the **10** ticked segments only (`addSegment("LineArray")` … `"EventTrace"`; Vision is not ticked). `loop()` = `benchTick()`: for each of the 10 modules, `t=micros(); Module.update(); prof.sampleSegment(i, micros()-t);` in exact `cooperativeTick()` order, wrap the whole pass with `sampleTick`; when `prof.tickCount() >= WINDOW_TICKS`, `prof.report(Serial, TICK_FLOOR_HZ, TICK_GOOD_HZ); prof.reset();`. Poll Serial: `?` → print help + current window state (ticks-so-far, N, thresholds); `r` → reset window now. Local `constexpr WINDOW_TICKS/TICK_FLOOR_HZ/TICK_GOOD_HZ/SEGMENT_COUNT`. Header docblock: purpose, the exact arduino-cli build command from T1, the 115200 Serial note, the idle-vs-active `Localization` caveat, and a **keep-in-sync-with-cooperativeTick** warning. Satisfies AC-wire, AC-report, AC-breakdown, AC-constants, AC-serialcmd. |
| T4 | Host ctest for the profiler math | T2 | low | `WRO2026/test/test_tick_profiler.cpp`, `WRO2026/test/CMakeLists.txt` | Add a tiny local `CaptureStream` struct (Arduino-style `print()/println()` overloads appending to a `std::string`; **NO `Print` base** — the harness has none, only `MockSerial`) and TC1–TC7: feed known dt vectors; assert per-segment + whole-tick min/avg/max, `effectiveHz`, `classify` at three rates + zero-count, `tickMinUs()==0` + `report()` `min=0/max=0` on an empty window, `reset()` semantics, `addSegment` overflow → `TICK_INVALID_SEGMENT` + no-op sample, and that `report(cap,…)` matches the Contracts→Naming grammar (ordered `SEG` lines + `TICK` line + verdict token). Register a `test_tick_profiler` target in `WRO2026/test/CMakeLists.txt` modeled on `test_stop_at_line_armmath` (links `MOCK_SOURCES` + `COMMON_INCLUDES` only, no production `.cpp`); must be warning-clean under the harness's `-Wall -Wextra` / `/W3 /permissive-`. Satisfies AC-hosttest. |
| T5 | Breadcrumb + stale-comment fix in the real sketch | — | low | `WRO2026/WRO2026.ino` | Add a short comment at `cooperativeTick()` (~L136): "If you add/remove a module here, mirror it in `inoTest/tick_bench/tick_bench.ino` so the loop-time benchmark stays representative (#123)." AND fix the stale `cooperativeTick()` docblock order list (~L133-134), which omits `Tasks` — the real order is `… Manipulator → Tasks → PathTracker → …`. Comments only, no logic change. Independent of the other tasks (the bench path is pinned in Naming). Satisfies AC-breadcrumb. |

**Parallelism:** T1, T2, and T5 have no blockers → all three can start day one (T5 only edits `WRO2026.ino`, which no other task touches). T4 starts as soon as T2 lands (parallel to T3). T3 waits for both T1 (mechanic proven) and T2 (profiler exists). No two parallel tasks share a file (T1 and T3 both own `tick_bench.ino` but are ordered T1→T3; T5 is the sole editor of `WRO2026.ino`).

## Notes for Implementer

- **Do T1 first and for real.** The whole approach rests on `arduino-cli --library WRO2026` compiling the real modules into a sibling sketch. If neither `--library` nor the copy-sources-into-a-gitignored-`src/` fallback (NOT a symlink — unreliable on Windows, see Decisions) works cleanly, stop and surface it — the fallback-of-last-resort (bench sketch `#include`-ing `.cpp` files) is **banned** in this repo (multiple-definition; see the "never #include a .cpp in the sketch" gotcha) and would mean rethinking the location decision, not silently working around it.
- **arduino-cli is the only tool that catches this class of error** — host ctest uses mocks and separate TUs, so link/ODR/missing-symbol problems from the real-source reuse only show up under arduino-cli. Verify T1 and T3 with an actual compile, not by inspection.
- **`Vision` is not ticked** in `cooperativeTick()` (it is capture-once via `readMosaic()` from a task). Do not add it to the bench's per-tick loop — the 10 segments are exactly the modules in `cooperativeTick()`.
- **Keep the two tick orders in sync.** The bench replicates `cooperativeTick()`'s order and `setup()`'s `begin()` order by hand. The T5 breadcrumb is the only guard; if you touch `cooperativeTick()` in a future change, mirror it.
- **`LineArray.update()` does the I2C read every tick** when `Config::LINE_ARRAY_TICK_CACHE_ENABLED` is true (it is), so the dominant per-tick cost is captured even with no mission running — an idle bench tick is representative. Expect `LineArray` to dominate the per-module breakdown; that is the intended headline finding.
- **Idle-tick interpretation caveat (put it in the report docblock).** `Localization.update()` issues an IMU read only while nav is INACTIVE — its operator-offset gate is open at idle, closed during a mission — so the idle bench slightly OVER-reports `Localization` and under-reports the extra active-move work in `Drive`/`LineFollower`. Say this next to the summary so a reader doesn't mistake the idle split for the in-mission split. It does not change the `LineArray`-dominates headline.
- **Profiler self-overhead is negligible but bounded.** `benchTick()` adds ~2·(segments+1) = ~22 `micros()` calls per tick (~a few µs total) around ms-scale work; it is inside the measured whole-tick number, not subtracted. Note this in the `TickProfiler.h` docblock so the small constant offset isn't mistaken for module cost.
- **`report()` is a member template on the stream type** (`template<typename OutT> void report(OutT&, …)`), so the same code prints to `Serial` on device and to a duck-typed `CaptureStream` in the host test — with no `Print` base (the harness has none). Do NOT add a `class Print;` forward-decl or hardcode `Serial` inside `TickProfiler.h`: a non-template `report(Print&)` would access an incomplete non-dependent type and be rejected by g++ / MSVC `/permissive-`, and hardcoding `Serial` drags Arduino I/O into the header and breaks host compilation.
- **Config constants for the bench are local** to `tick_bench.ino` (like `gyro-move.ino`'s local `kBattCells`), not added to `WRO2026/config/`. This keeps the production Config untouched. Shared hardware values (battery cells, motor IDs) are read from the real `Config::`.
- **Follow-up (out of scope, note in the PR / a tracking issue):** (1) an optional CI arduino-cli *compile* check of `inoTest/tick_bench/` to catch API-drift bit-rot; (2) the issue's own deferred item — a structural per-tick guard that counts I2C reads / `update()` calls against a budget; (3) an "active-move" measurement mode; (4) an X-macro `WRO2026/TickModules.h` shared between `cooperativeTick()` and the bench to make module-list drift a compile error (see Decisions — deferred because it touches production firmware). None are built here.
- **Rollback:** every deliverable except the T5 one-line comment is a new file; reverting is deleting `WRO2026/TickProfiler.h`, `inoTest/tick_bench/`, `WRO2026/test/test_tick_profiler.cpp`, the CMake target line, and the breadcrumb. No production runtime path changes.
