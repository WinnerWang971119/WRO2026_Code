# ADR-055: Sketch root is operator-editable only; framework modules live under `WRO2026/src/<domain>/`

**Status:** Implemented
**Date decided:** 2026-07 (issue #66)

## Rule

The `WRO2026/` sketch root holds exactly the operator-editable set — `WRO2026.ino`,
`Config.h` + `config/`, `MissionScript.{h,cpp}`, `Tasks.{h,cpp}`, `TasksTypes.h` — and
nothing else. Every framework module (39 files: `Drive`, `LineFollower`, `Intersection`,
the six resolver policies, `WROLineArray`, `Vision`, `VisionFrame`, `routes`, `FieldMap`,
`NavTypes`, `Localization`, `YawSource`, `Mission`, `PathTracker`, `PathTrackerParityTrace`,
`Manipulator`, `ManipServo`, `ManipBelt`, `WROMath`, `StateMachine`, `EventTrace`,
`TickProfiler`) lives under one of six `WRO2026/src/<domain>/` folders — `drive`,
`perception`, `nav`, `mission`, `manip`, `util`. New framework code goes under
`src/<domain>/`; it never lands at the root again.

(Number skip: ADR-054 is claimed by the concurrent unmerged `feat/ctps-pivot-wheel-arc`
branch, whose plan already cites "Recorded in ADR-054" — per the project's ADR-collision
convention, the branch merging second takes the next free number, so this one starts at 055.)

## Why

Before this change the sketch root was a flat wall of ~40 files with no signal about
which ones an operator should touch at the field versus which ones are framework
internals nobody should hand-edit under competition time pressure. Splitting by
**operator-editability**, rather than pure technical domain, makes the root itself answer
"can I edit this file at the field?" — anything at `WRO2026/` root is fair game, anything
under `src/` is framework.

The split only works because Arduino resolves a quote-include (`#include "..."`) relative
to the including file's own directory first, before falling back to the library search
path. Moving a file breaks every include that used to resolve by sitting in the same flat
directory, so each cross-boundary include had to be rewritten to its explicit relative
form instead of staying bare. The host-test harness needed the same reconciliation: its
mocks and hand-written test TUs already shadow production headers by bare name
(`"Drive.h"` means "give me the mock"), so pointing an include at the real header post-move
needed a form that could never collide with that shadow — the root-relative
`"src/<domain>/…"` form.

## How to apply

- Six domains, exact mapping in `tools/reorg/mapping.py` (the source of truth): `drive`
  (Drive, LineFollower, Intersection, TurnSignPolicy), `perception` (WROLineArray, Vision,
  VisionFrame), `nav` (routes, FieldMap, NavTypes, Localization, YawSource, and the five
  nav-owned resolver policies — DirectionPolicy, StopAtLinePolicy, EdgeDispatchPolicy,
  PivotTrimPolicy, ReturnToNodePolicy), `mission` (Mission, PathTracker,
  PathTrackerParityTrace), `manip` (Manipulator, ManipServo, ManipBelt), `util` (WROMath,
  StateMachine, EventTrace, TickProfiler).
- Include forms, by the including file's location: `src/<A>/` → same-domain header is bare
  `"Header.h"` (unchanged); `src/<A>/` → sibling `src/<B>/` is `"../<B>/Header.h"`;
  `src/<A>/` → root header (`Config.h`, `Tasks.h`, `TasksTypes.h`, `MissionScript.h`) is
  `"../../Header.h"`; `src/<A>/` → a `config/` sub-header (e.g. `ManipServo.h` →
  `ConfigManipulator.h`, `Vision.h`/`VisionFrame.h` → `ConfigVision.h`) is
  `"../../config/ConfigX.h"`; a root file (`.ino`, `MissionScript.cpp`, `Tasks.{h,cpp}`,
  `TasksTypes.h`) → framework header is `"src/<B>/Header.h"`; root-to-root and
  `config/`-to-`config/` stay bare.
- **Host-test harness convention:** in a `WRO2026/test/` file, a bare `"Drive.h"` still
  resolves to the **mock** (via the existing force-includes and `-I` order); the
  root-relative `"src/drive/Drive.h"` resolves to the **real** header through the
  unchanged `-I ${SKETCH_DIR}`. Every CMake `-I` list is untouched — only `#include` lines
  moved. A hand-written mock or test TU that genuinely needs the real, unmocked header
  (e.g. `mocks/Drive.h` reaching `WROMath.h`) uses the root-relative form.
- **`--library` diagnostic replacement:** arduino-cli decides recursive 1.5-format vs. flat
  legacy 1.0-format by whether a `library.properties` file is present. `WRO2026/` has none,
  so gaining a `src/` subfolder changes nothing about its format — `arduino-cli ...
  --library WRO2026` still resolves as a flat legacy library rooted at exactly `WRO2026/`,
  and the root files (`Config.h`, `Tasks.h`, `TasksTypes.h`, ...) stay on the include path
  exactly as before. What that one flag does NOT do is recurse into `src/<domain>/` — each
  domain folder is itself a `library.properties`-less flat library, so reaching a framework
  header there needs its own `--library WRO2026/src/<domain>` flag alongside the root one. A
  diagnostic sketch that wires up several modules needs the root flag plus one
  `--library WRO2026/src/<domain>` flag per domain it touches. `inoTest/tick_bench/tick_bench.ino`
  is the reference: it touches `drive`, `perception`, `nav`, `mission`, and `manip`, plus its
  own `util`-domain `TickProfiler.h` dependency, and compiles clean with `--library WRO2026
  --library WRO2026/src/drive --library WRO2026/src/perception --library WRO2026/src/nav
  --library WRO2026/src/mission --library WRO2026/src/manip --library WRO2026/src/util`.
  `tools/vision_bench/vision_bench.ino` is the one exception: it keeps the raw
  `-I<abs>/WRO2026` extra-flags mechanic instead of `--library`, because its includes are
  path-qualified (`config/ConfigVision.h`, `src/perception/VisionFrame.h`) and arduino-cli's
  `--library` usage scan only recognizes bare-basename `#include`s, so a path-qualified
  include never registers as "used" and the flag would get silently dropped. The raw `-I`
  adds the path unconditionally with no such scan, and it also reaches the root-level
  `config/ConfigVision.h` in the same stroke, which `--library WRO2026/src/perception` alone
  would not provide.

## Trigger to revisit

If a framework header genuinely needs to live in more than one domain (a cross-cutting
concern that doesn't fit `drive`/`perception`/`nav`/`mission`/`manip`/`util`), revisit
whether six domains still partition the tree cleanly. If Arduino tooling ever adds a
stable way to append extra include-search directories to a sketch build, the
source-relative include rewrite may become unnecessary.

## See also

- [`docs/plans/2026-07-05-sketch-src-reorg.md`](../plans/2026-07-05-sketch-src-reorg.md) — the full migration plan (issue #66).
- [ADR-030 — Host-compile harness links the real production symbols](./030-harness-links-real-sources-ci-enforced.md) — the harness this ADR's include convention keeps congruent with Arduino's own resolution rule.
- [ADR-049 — Host-test mock HAL as the shared trust boundary](./049-host-test-mock-hal.md) — the mock-vs-real shadowing this ADR's harness convention preserves.
