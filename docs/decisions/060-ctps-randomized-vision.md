# ADR-060: CTPS randomized vision — trace-carried roll, unified pinned-cells schema, thin dormant mock

**Status:** Partially implemented
**Date decided:** 2026-07 (phase)

## Rule

CTPS gains a randomized 4×3 mosaic + a robot-attached forward vision cone as a
visualization + dump-injection feature, layered without touching the pinned
seek core. Python is the **single roll carrier**: it rolls the mosaic once
(seeded), and the colors + seed + a geometry fingerprint (`geom_hash`) are
passed to `ctps_dump`, which **echoes** them into a top-level trace `mosaic`
block — never rolled independently for the player embed and the dump. The
player resolves the rendered mosaic from (trace colors × layout geometry) and
badges a `geom_hash` mismatch the same way it already badges a trace recorded
against a different field.

The mosaic schema is a **unified pinned-cells** shape: `mosaic.cells[12]`
(`null` = rolled, or a pinned color name) + `mosaic.counts` (quota for the null
cells) + a `mode` label. `mode:"directed"` requires all 12 cells pinned;
`mode:"random"` allows any subset pinned, with `counts` summing to the number
of null cells. One roll function and one validator cover both, and "mostly
directed with N random cells" falls out for free.

Cell→world geometry (`resolve_cells`) is **Python-authoritative and unit
tested** — it is the load-bearing cell→`getColor` mapping; the editor's JS
mirror is a preview convenience only, validated against the same pinned
vectors. Vision assets (`vision.js`, the `#ctps-vision` panel, the cone/mosaic
layer registrations) ship **unconditionally** on every player page, mirroring
ADR-056's cargo-asset rule — only the rolled DATA (`MOSAIC_META`, `const
MOSAIC`, the trace `mosaic` block) is conditional on a `mosaic` block existing
in `layout.json`. The player-side read predicate is per-cell:
`visibleCells(pose, visionCfg, cells) -> bool[12]`, and
`readMosaicWouldSucceed` is the AND of all twelve, so the HUD can report which
specific cell falls outside the cone.

The C++ side gets a thin, record-and-test-set `Vision` mock (ADR-049 fidelity)
plus a `read_mosaic` trace op, landed now and CI-proven via the `ctps_dump`
self-test synthesis, but dormant: no mission calls `Vision.readMosaic()` yet.
Canonical color names/letters/hex are single-sourced in `mosaic.py`; a pytest
regex-parses the real `enum class Color` out of `ConfigVision.h` and asserts
the Python list matches it in name and order, so the mock's letter→enum
parsing and the JS/emitted `MOSAIC_META` copy can never drift from the
firmware's actual enum.

**Cell-ordering convention (binding on the firmware):** cell index
`i = row*cols + col`, row 0 at `+y` ("top"), row-major, top-left-origin, in
the mosaic's own world frame, with the camera assumed to view the mosaic from
the robot's line-of-approach side. This ordering is *defined* by the sim
(`resolve_cells`) — it is not an arbitrary choice up for negotiation at bench
time. The bench-taught `Config::VISION_CELL_TO_SLOT` mapping must be made to
match this convention, not the reverse.

## Why

A dual-roll design (Python rolling once for the player embed and again,
separately, for the dump argv) is an unguarded drift pair, and it makes
`--trace`-only replay incoherent — there would be no way to know which roll a
loaded trace actually reflects. Carrying the roll on the trace closes that
hole and gives `--trace` the same coherence guarantee TC9-class tests already
assume for the rest of the player. The geometry fingerprint closes the
adjacent hole: a trace recorded against one mosaic seat/gap config, replayed
against a `layout.json` that has since moved the mosaic, would otherwise
silently render colors on the wrong grid.

A unified pinned/random schema was chosen over two separate mosaic shapes
because "some cells pinned, rest random" is the general case a competition
rehearsal actually wants (verify a specific failure cell while randomizing the
rest), and a single validator is cheaper to keep correct than two.

Landing the `Vision` mock and `read_mosaic` op now — before any mission
consumes them — is forced by a build-ordering fact, not scope creep:
`ctps_dump` compiles the whole of `Tasks.cpp`, so the day a real vision-reading
task lands, the dump build breaks unless the mock already exists and the trace
op already has a producer/whitelist/consumer path proven end-to-end. Building
it now, CI-proven via synthetic self-test emission, means the eventual
Phase-6 mission-consuming task is additive, not a scramble to unbreak `ctps
--dump`.

## How to apply

- `tools/path_planner/ctps/mosaic.py` is the single source of `COLOR_NAMES`/
  `COLOR_LETTER`/`COLOR_HEX`, `roll_mosaic()`, `resolve_cells()`, and
  `geom_hash()`. Never hardcode a color name/hex/letter anywhere else —
  `test_ctps_color_enum_parity.py` fails CI on drift against
  `WRO2026/config/ConfigVision.h`.
- The dump argv grammar (`--mosaic`, `--mosaic-seed`, `--mosaic-geom-hash`,
  `--mosaic-fail-at`) is order-free and composes with `--fixture`; the
  **user-facing** reproduce flag is the orchestrator's `--seed` — never
  document `--mosaic-seed` as a user-facing flag. No `--mosaic` ⇒ no
  injection, no echo, byte-identical dump JSON to today.
- `assets/vision.js` + the `#ctps-vision` panel markup + the `vision-cone`/
  `mosaic` layer registrations are added to the **unconditional** asset/markup
  set in `player_html.py`, alongside the ADR-056 cargo assets — a
  mosaic-free-emitted page still renders an uploaded `mosaic`/`robot.vision`
  layout fully.
- The `Vision` mock lives in `WRO2026/test/ctps_mocks/Vision.h`, force-included
  only for the `ctps_dump` target (never the shared
  `add_ctps_mock_force_includes`, which `test_ctps_mock_conformance` mirrors —
  this is not an ADR-051 conformance surface). It records call args and
  test-settable fail-at state; it does not model cone geometry — the mock's
  `readMosaic()` return value and the player HUD's independent geometric
  cone-coverage prediction are allowed to diverge, and a divergence is a
  signal about the authored read pose, not a bug (ADR-049 thin-mock rule).
- `Vision.getColor(i)` must be indexed per the cell-ordering convention above;
  when a Phase-6 mission wires up the real bench camera, tune
  `Config::VISION_CELL_TO_SLOT` to match `resolve_cells`'s ordering, not the
  other way around.

## Trigger to revisit

- A Phase-6 mission actually calls `Vision.readMosaic()` and drives to a
  vision-consuming node — at that point the thin mock's recorded surface may
  need to grow, but must not gain multi-tick behavioral modeling (ADR-049).
- A geometry-gated dump (the mock re-evaluating the cone and re-running the
  mission to converge on a read) is requested — this was evaluated and
  rejected here as heavy, non-convergent, and a duplicate of the player's
  predicate; revisit only if a concrete mission need can't be met by the
  current dormant plumbing.
- Mosaic sizes other than 12 cells are requested — today's schema is pinned to
  `rows*cols == 12 == Config::VISION_CELL_COUNT`; loosening that is a schema
  change, not a bugfix.
- The bench camera's physical mounting makes the "approach-side view" cell-0
  assumption wrong for a real field geometry — revisit the ordering
  convention itself, not just `VISION_CELL_TO_SLOT`.

## See also

- [ADR-033 — CTPS ingests a derived mission trace recorded from the real C++](./033-ctps-derived-mission-trace.md) — the trace-carry precedent this ADR's `mosaic` echo follows; no Python re-implementation of task bodies.
- [ADR-049 — Host-test mock HAL as the shared trust boundary](./049-host-test-mock-hal.md) — the thin record-and-test-set fidelity rule the `Vision` mock follows, and the dedicated-dir shadowing rule (`ctps_mocks/`, not the shared mock dir).
- [ADR-053 — CTPS player is modular concatenated assets + extension registries](./053-ctps-player-modular-shell.md) — `vision.js` registers as a layer via these registries; no edits to the pinned seek core.
- [ADR-056 — CTPS object/cargo model authored on a separate offline editor page](./056-ctps-object-cargo.md) — the unconditional-assets/conditional-DATA precedent (cargo) this ADR's vision assets mirror exactly.
- `docs/plans/2026-07-11-ctps-randomized-vision.md` — the originating plan.
