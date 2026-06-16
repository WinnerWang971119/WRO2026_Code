# Brainstorm: Issue #38 — Implement Vision (HUE12 sensor)

**Seed (verbatim):** "Issue34. Things you need are here C:\Users\wangd\Documents\WRO2026\Vision"
(Resolved: #34 is a merged PR; the intended target is **Issue #38 "Implement Vision"**, body:
"We got the vision board and a doc. Now we have to program it. Fundamental: Getcolor(x, y),
getMosaic color (There is a contract). Important!")

**Context summary:**
- Hardware: **HUE12 視覺感應器** (MATRIX vision board), v19.5 Auto-Bind. PC-Free, taught via 2 back
  buttons. Talks **UART (Serial1 @ 115200)** to the R4 — matches the library's
  `MiniR4SmartCamReader` (0xAA header / length / LE words / XOR checksum framing).
- HUE12 native protocol: **12 slots (S1–S12)**, each a *learned color target*. Per slot it returns
  **HUE (0–255)** + **STA (saturation)**. HUE bands: 0–239 standard hue, 240–245 White, 246–250
  Black, 255 = slot disabled / NONE (not found). Pybricks exposes `getHue(slot)` / `getSta(slot)`.
- Slot teaching: SET X → SET Y → TOL (tolerance). CAM SETUP = global brightness/contrast/sat/
  exposure/flash/flip/AWB. All **auto-saved to on-device memory** (persistent, PC-free).
- Game mosaic (from `WRO2026_HighSchool_Topic_Drawer.html`): **12 cells (4×3)**, colors
  {Yellow, Blue, Green, White}, ≤6 of any one color, 1.5 cm black grid lines. 12 cells ≈ 12 slots.
- Existing design intent: **ADR-006** (read 3×3 mosaic by camera at a line-anchored stop) and
  **ADR-021** (camera is the SOLE color sensor — no dedicated ColorReader; one calib table for
  {Y,B,G,W}). Both **Pending implementation**. Sketched API: `capturePattern(out[9])`,
  `readSingleColor()→ColorEnum`. Note: ADRs assume 9 cells; game art now shows 12.
- Integration surface: Tasks framework (`Tasks.h`/`TasksTypes.h`) already anticipates a
  `mosaicPlace` task with `struct MosaicPlaceArgs { uint8_t cell; uint8_t color; ... }`.
- **Core tension:** the issue wants `getColor(x,y)` + `getMosaicColor()`, but HUE12 natively gives
  per-slot HUE/STA — NOT arbitrary pixel reads or a grid scan. How those map is the open question,
  and "There is a contract" signals Diego has a specific mapping in mind.

**Clarified framing:** Design the **full HUE12 vision module** for the WRO2026 robot — the read API,
HUE→{Yellow,Blue,Green,White} classification, calibration location (on-device button-taught slots vs
Config baselines), Tasks-framework integration, and camera mount geometry. The public surface is
`getColor(cell)` (single cell index → classified color; the issue's "(x,y)" collapses to a cell id)
and `getMosaicColor()` (whole 12-cell mosaic → 12 colors). The **slot↔cell mapping is deliberately
open** — how the 12 HUE12 slots (each a learned HUE/STA target) realize per-cell reads is the central
question the brainstorm must widen. Must honor ADR-006 (line-anchored capture) and ADR-021 (camera is
the sole color sensor, one calibration table), and fit the established cooperativeTick / fluent-builder
/ strict-OOP conventions.

## Perspectives (round 1)
### Critic
1. **Slot-semantics ambiguity is unresolvable from the documented packet, and the likeliest reading makes `getColor(cell)` impossible.** Packet is HUE+STA only, no X/Y. A slot taught by a fixed SET X/SET Y + TOL most plausibly reports color at a *pinned screen point*, not a roving detector — which reintroduces the mm-positioning problem ADR-006 dodges with line-anchoring. The public API presupposes a clean cell→slot bijection that isn't proven.
2. **PC-Free on-device teaching is irreproducible — collides with Config-baseline / CI / sim.** All calibration (12 slots × SET X/Y/TOL + CAM globals) lives in opaque device memory, taught by button presses. Can't commit, diff, regenerate in CI, or feed the Python sim. An accidental long-press turns slots off silently; nothing in the toolchain detects drift.
3. **A single HUE byte can't reliably separate {Y,B,G,W} under venue lighting.** Firmware already concedes white isn't a hue (reserved band 240–245) → White depends on a hidden STA threshold you can't set in Config. Workshop≠venue drift can collapse pale-yellow↔white; classification happens *inside* firmware so no R4 post-processing recourse. ADR-021 B4.1 wants ≥18/20 per color.
4. **Library reader is blocking, single-frame, self-contradicting on baud.** `SmartCamReader()` spins up to 500 ms and drains RX each read — violates cooperativeTick unless reimplemented (can't edit read-only lib). Facade documents "mVision (UART 9600)" while reader hardcodes 115200 — concrete proof name/comment inference is unreliable here.
5. **No honest sim/CTPS mirror for opaque-firmware hardware.** Closed classification + venue-specific button state = nothing to mirror. Any sim stand-in is invented; a `mosaicPlace` Task can pass CI and still fail on hardware for reasons the sim structurally can't model. False confidence on the most failure-prone subsystem.
6. **9-vs-12 cell mismatch invalidates the geometry both ADRs assume.** ADRs are built for 3×3 (`capturePattern(out[9])`); art is 4×3=12. Changes cell pitch, required FOV, per-cell angular size (more bleed across 1.5cm lines), mount distance — reopens the "mount geometry locked" Phase-3 gate. Foundation is a documented contradiction.
7. **Issue scope is premature: it freezes API + geometry while the one hinge is open.** Can't responsibly design mount/Config/Tasks/sim until a 30-min bench test resolves fixed-point vs roving-blob. Risks elaborate abstraction a single observation invalidates (CLAUDE.md rule 9, no speculative abstraction).

### Feasibility — **VERDICT: Conditional**
**Must-be-true:**
- The 12 slots must be reprogrammed from "12 colors" to "12 fixed cell regions." Native model is slot=learned color (roving blob, no X/Y in packet). "One capture → 12 cells in order" only works if each slot is a fixed crosshair window AND v19.5 supports fixed-window-per-slot mode. **This is the single unproven hinge — settle on the physical board or by inspecting the `.mbr4`.** Fallback: per-cell physical aiming (12 passes), slow + geometry-fragile.
- **The `.mbr4` block program is the real contract, not the manual.** mVision is OpenMV-based (STM32H7, LAB thresholding); the SPIKE example shows `DataToSend` is *programmable* (≤8 items, ≤32767 each) — the packet is whatever the program emits, so per-slot X/Y is technically emittable. "(There is a contract)" almost certainly = this program's chosen payload. Confirm by reading `DataToSend`.
- A 12-item frame must fit the reader's 40-byte buffer → max 20 LE-uint16 words. 12×hue fits; 12×(hue+sta+x+y)=48 does NOT (needs custom reader or multi-frame).
- On-site recalibration must come from on-device button teaching (PC-free), contradicting ADR-021's single Config table. Design must choose where the {Y,B,G,W} baseline lives.

**Hard blockers:** None unconditional (the hinge has a feasible fallback → Conditional not Blocked).
**Solvable:** UART clear — Serial1 = RA4M1's lone D0/D1 UART, `MiniR4.Uart`≡Serial1, nothing else in repo uses it, line sensor is on I2C/Wire1, **no contention** (one constraint: "start" handshake + 0xAA/XOR framing must match `.mbr4`). Latency fits (valid frame <4ms @115200; risk is *blocking* in cooperativeTick → wrap as one-shot at a full stop). Classification workable: 4 colors hue-separable, White(240–245)+NONE(255) free, use STA as tiebreak (White=low STA); needs venue bench test. Sim/ctest mirror straightforward — vision is stateless bytes→ColorEnum, a `FakeVision`/injected-hue seam feeds canned 12-cell frames; `TasksTypes.h` already anticipates `MosaicPlaceArgs{cell,color,...}`.
**Evidence:** Manual images resolve the ambiguity *against* a native grid read — SET X/SET Y are the teaching crosshair's axes; RUN MODE streams HUE+STA with no position word → slot=learned color, matching HuskyLens. UART path unambiguous. Could NOT confirm from any source whether v19.5 pins each slot to a fixed window for a single positional capture — must test board or read `.mbr4` `DataToSend`.

### Thinker (branches)
1. **Slot-as-position, geometry baked at teach time** — teach 12 slots as fixed aim points over the 4×3; R4 never computes geometry; `getMosaicColor()`=read 12 HUE words, `getColor(cell)`=slot[cell]. Needs a teaching jig + documented teach order; on-site recalib = "re-aim 12 dots."
2. **Slot-as-color, one roving detector per palette entry** — teach 4 slots (Y/B/G/W); needs a second localization mechanism unless a slot reports blob X/Y centroid.
3. **Hybrid: 12 position slots + Config HUE→Color LUT on R4** — HUE12 owns *where*, R4 owns *what* (`Config::HUE_BANDS[]` + White/NONE specials). One editable table in Config, recompile-free on-site edit vs 48 button presses.
4. **Two-layer calibration stolen from WROLineArray** — layer1 = on-device taught targets (set once per venue), layer2 = software per-slot HUE trim in Config re-bakeable in seconds; a `capture-baseline` routine snapshots all 12 HUEs.
5. **routes.h-style codegen for sim parity** — offline tool emits SHA-stamped `vision_calib.h` (taught refs + band edges); firmware + Python sim read the same source → simulated read matches robot; wire into build CLI + CI freshness check.
6. **Don't read 12 — read N cells the task needs, on demand** — `getColor(cell)` is the primitive, `getMosaicColor()` a convenience loop; optimize the single-slot poll for the active cell; dovetails ADR-021 single-block read.
7. **Camera never moves — index mosaic by robot pose, not slot** — `cell` = positional index into approach; a stop↔(slots-in-view→cells) table tiles 12 cells across multiple line-anchored stops; narrow-FOV camera composes the mosaic in the R4.
8. **HuskyLens-mode analogy: MOSAIC mode vs BLOCK mode** — module built around explicit taught modes; cross-mode calls fault; addresses ADR-021's "one optical setup serves mosaic AND single-block."
9. **Self-calibrating from ≤6-of-any-color constraint** — snapshot 12 raw HUEs, k=4 cluster on the HUE ring (pull White 240–245 out first), assign Y/B/G/W relatively → no per-venue baseline; reject reads that violate ≤6. (speculative)
10. **Voting / temporal fusion over streamed frames** — accumulate K frames in cooperativeTick, return modal color per cell, STA as confidence gate; no-quorum cell → retry/fault/unknown.
11. **NONE/Black as a first-class geometry signal** — expected Black(grid lines)/NONE(out of frame) pattern becomes `isFramedCorrectly()` runtime guard for ADR-006's geometry. (speculative)
12. **EventTrace-backed vision debug overlay** — pipe 12-cell read to OLED glyph grid + Serial; gives on-site calibration an immediate no-laptop feedback loop.

## Live direction
**Capture-once / latch / pure-getter model** (user-chosen, 2026-06-12).
- The HUE12 **device** owns cell selection / slot config (taught on-device with the buttons). Slot N ↔
  cell N is configured on the board; the only unknown is which physical cell index N refers to in the
  real world — resolved by a **simple bench test program**, not a design question (dissolves Critic #1).
- The robot drives to the **best-view line-anchored stop** (ADR-006), then **captures ONCE**: one UART
  read of all 12 cells, classify, store in a member array.
- Public surface is then pure getters over the stored snapshot: `getColor(cell)` → stored color,
  `getMosaicColor()` → stored 12-array. No I/O in the getters; callable any time after capture.
- Because capture is a deliberate one-shot at a full stop (not a per-tick poll), a bounded blocking
  read there is acceptable — softens Critic #4 (still bound it + retry-once + fault on failure).

**Assumed defaults (state, not re-litigate):** getter before any capture → `UNKNOWN` + `hasCapture()`
false; a new capture overwrites the latch; capture failure retries once then faults (repo convention).

**Forks — RESOLVED (2026-06-12):**
1. **Classification → on the R4, `Config::HUE_BANDS[]` table** (hue→{Y,B,G,W} + White 240–245 /
   NONE 255 specials). Version-controlled, sim-reproducible, one table per ADR-021. Chosen because the
   device emits a raw hue per cell *regardless*, so on-device classification saves no work and only
   forfeits git/CI/sim visibility. Hybrid trim deferred until a cell proves consistently off.
2. **Capture → single frame + retry-once now**, fault on second failure. Documented fallback: STA-gated
   vote over a few dwell frames if it misfires often at the venue (a known escape hatch, not a rewrite).

**Carried-forward flags:**
- Baud contradiction: facade documents "mVision UART 9600" but `MiniR4SmartCamReader` hardcodes
  `Serial1.begin(115200)`. Verify on hardware before trusting either.
- 9-vs-12 cells: ADR-006/021 assume 3×3=9 (`capturePattern(out[9])`); topic art is 4×3=12. Size the
  module to 12 (Config-parameterized cell count) and update both ADRs.
- Prerequisite task: bench-test program to map slot/cell index → real-world cell position.
- ADR-021 single-block read (`readSingleColor`): the same `getColor`/hue-band path can serve a held
  block's color — fold in or note as a near-term follow-on.

## Killed / parked

## Decisions & debate verdicts
