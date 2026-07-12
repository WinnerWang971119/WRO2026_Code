# ADR-063: Mosaic random-fill relaxes the counts-sum invariant — under-fill → air, over-fill → random subset

**Status:** Implemented
**Date decided:** 2026-07 (phase)

## Rule

In CTPS random-mode mosaic rolls, `mosaic.counts` may sum to ANY non-negative
number — it is no longer required to equal the null-cell count (relaxing
ADR-060's "`counts` summing to the number of null cells"). `roll_mosaic`
resolves the two off-exact cases deterministically:

- **under-fill** (`sum(counts) < #null cells`): the leftover null cells become
  **air** — the empty-cell sentinel `AIR_NAME = "UNKNOWN"` (the firmware
  `Config::Color::UNKNOWN=0`). Air is padded into the pool BEFORE the shuffle so
  it lands at random positions among the null cells, not always at the tail.
- **over-fill** (`sum(counts) > #null cells`): the shuffled pool is dealt into
  the null cells and the excess is dropped — the kept subset is a uniform random
  sample of the counts multiset.

Exact fill (`sum == #nulls`) is byte-identical to pre-ADR-063. Pinned cells are
always kept verbatim; air is a roll-time fill ONLY — never an authorable /
pinnable palette value — and directed mode is unchanged.

## Why

Authoring a random mosaic previously forced the operator to make the per-colour
quotas sum to exactly the number of null cells, an arithmetic chore that made
"put a few coloured tiles down, leave the rest empty" or "throw a big pile of
tiles at it and randomize" awkward to express. Relaxing the sum lets the counts
state intent directly: under-supply means empty cells, over-supply means a
random draw. "Air" reuses the existing `UNKNOWN=0` enumerant rather than adding
a colour, so the change is confined to the roll plus the two validators and
never touches the firmware enum, `MOSAIC_META`, or the header-parse colour-enum
parity guard.

## How to apply

- The roll and the sum relaxation live in `tools/path_planner/ctps/mosaic.py`
  (`roll_mosaic`, `_validate_mode_specific`). `AIR_NAME` / `AIR_LETTER` and the
  single-sourced `color_to_letter()` wire join live there too — never
  re-hardcode the air letter.
- `AIR_NAME` is kept OUT of `COLOR_NAMES` / `COLOR_HEX`: the player renders an
  air cell through its existing unknown-colour outline fallback, and the
  enum-parity guard (which strips `UNKNOWN`) stays green.
- The `--mosaic` wire gains one letter: `A → Config::Color::UNKNOWN` in
  `ctps_dump.cpp::letterToColor`; `ctps_record` already echoes `UNKNOWN` as
  `"UNKNOWN"`. The orchestrator joins the roll via `mosaic.color_to_letter`.
- The editor's JS validator mirror (`editor_io.js`) drops the sum-equality
  rejection; the counts readout (`editor_tools.js`) is informational
  (under-fill → "N air", over-fill → "trims to N, random"), never an error tint.

## Trigger to revisit

- A mission or authoring need calls for a directly **pinnable** air/empty cell
  (today air is a roll-time fill only) — that is a schema + validator + editor
  change, not covered here.
- Mosaic sizes other than 12 cells are requested — still pinned to
  `rows*cols == 12 == Config::VISION_CELL_COUNT` (see ADR-060).

## See also

- [ADR-060 — CTPS randomized vision](./060-ctps-randomized-vision.md) — the roll / schema this amends; its "`counts` summing to the number of null cells" clause is superseded by the rule above.
- `tools/path_planner/tests/test_ctps_mosaic.py` — under/over-fill + `color_to_letter` coverage; `WRO2026/test/CMakeLists.txt::ctps_dump_mosaic_air` — the C++ air-letter round-trip ctest.
