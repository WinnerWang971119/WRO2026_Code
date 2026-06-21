# Issue #51 — Cross-Platform Dev Tooling (Mac + Windows) Plan

**Goal:** Make the WRO2026 developer tooling run identically on macOS, Linux, and
Windows so a Mac contributor can build/run everything, fix the reported CTPS
orchestrator bug (#51), and ship a single Mac+Windows setup guide.

**Approach:** Three pillars. (1) **CTPS fix** — branch the orchestrator on OS:
pick the executable name by platform and, on non-Windows, build `ctps_dump` with
a plain default-generator `cmake` invocation (compiler from PATH) instead of
vcvars/vswhere; locate the built binary generator-agnostically. (2) **Drop the
PowerShell dependency from the dev workflow** — convert the Python VS Code tasks
to `uv run`, port the Arduino task logic to a cross-platform Python helper, delete
both `.ps1` wrappers, and fix the Windows-only VS Code config. (3) **Document** —
a full cross-platform `docs/SETUP.md`. The whole change is dev-tooling + docs;
no robot firmware or path-planner pipeline logic is touched.

This approach was endorsed by an independent Opus architecture consult, with five
hardening amendments folded in (binary-location robustness, macOS login-shell PATH,
macOS serial-port dedupe, UTF-8 subprocess captures, and a Tk-on-Mac doc preflight).

## Scope

- **In scope:**
  - `tools/path_planner/ctps/__main__.py` — OS-branched build/run, platform exe
    name, generator-agnostic binary discovery, UTF-8 subprocess captures, clear
    "no toolchain" error on non-Windows.
  - `WRO2026/test/CMakeLists.txt` — pin `RUNTIME_OUTPUT_DIRECTORY` on the
    `ctps_dump` target so its binary lands at the build-tree root under every
    generator. (Test-harness build wiring, **not** firmware or pipeline logic.)
  - `tools/arduino_cli_task.py` — **new** cross-platform port of
    `arduino_cli_task.ps1` (verify/upload/list-boards/monitor + serial-port
    auto-detect with macOS `cu.`/`tty.` dedupe + baud + friendly errors).
  - `.vscode/tasks.json` — all tasks drop `pwsh`; Python tasks use `uv run`;
    Arduino tasks call the new helper; macOS/Linux variants run through a login
    shell so `uv`/`arduino-cli` resolve when VS Code is launched from the Dock.
  - `.vscode/settings.json` — remove `wro2026.pathPlanner.pythonPath`; change
    `cmake.sourceDirectory` from the hard-coded absolute path to
    `${workspaceFolder}/WRO2026/test`.
  - `.vscode/c_cpp_properties.json` — add a macOS configuration entry.
  - Delete `tools/path_planner_task.ps1` and `tools/arduino_cli_task.ps1`.
  - New pytest coverage for the CTPS OS-branching and the Arduino helper.
  - `docs/SETUP.md` — full Mac+Windows dev setup; README links to it. Includes a
    **"What was broken on Mac" audit section** (user ask #1) — every Mac breakage
    found, mapped to its fix (see the audit list below).
  - `.gitattributes` — add LF normalization for the repo's text types
    (`*.py`/`*.md`/`*.json`, etc.); the file today only carries golden-routes
    rules, so generic types are uncovered.

- **Out of scope:**
  - Robot firmware (`WRO2026/*.ino`, `*.cpp`, `*.h`) and path-planner pipeline
    logic (`validator/graph_builder/planner/compiler/header_writer`). Untouched.
  - Adding macOS/Windows runners to CI — CI stays Linux-only; the OS branches are
    covered by mock-based unit tests (per the testing decision).
  - Rewriting `tools/path_planner/README.md` setup (it already has a Linux/macOS
    venv branch); `docs/SETUP.md` becomes the canonical guide and may link to it.
  - Any change to CTPS output semantics, the `--trace` flow, or the rendered HTML.

## Decisions

- **Fix breadth:** Everything fully cross-platform (user choice) — CTPS + all VS
  Code tasks/config + Arduino helper + tests + guide.
- **Arduino tasks:** Port `arduino_cli_task.ps1` to `tools/arduino_cli_task.py`
  (preserves auto-port-detect, baud, friendly errors); delete both `.ps1` files.
- **CTPS non-Windows build:** Same default-generator / PATH-compiler approach as
  CI (no `-G Ninja`), scoped to the `ctps_dump` target. Windows keeps vcvars+Ninja
  unchanged. (CI builds *all* targets with no `--target`; we build only `ctps_dump`
  — same toolchain path, narrower scope.)
- **Setup guide:** `docs/SETUP.md`, full dev setup for both OSes; README links it.
- **C++ IntelliSense:** Add a second macOS `configurations[]` entry; keep Windows.
- **Testing:** Unit-test the OS branches by mocking `sys.platform`/`subprocess`
  (CI is Linux-only); no real compiler/board needed.
- **Opus consult amendments (mandatory):**
  - Binary location must be generator-agnostic — pin CMake `RUNTIME_OUTPUT_DIRECTORY`
    **and** glob for `ctps_dump[.exe]` (build root + `Debug/`/`Release/`) in Python.
  - macOS/Linux VS Code tasks run through a **login shell** or `uv`/`arduino-cli`
    won't be found when VS Code is Dock-launched.
  - `arduino_cli_task.py` dedupes macOS `cu.`/`tty.` serial pairs (prefer `cu.*`).
  - All `subprocess.run` text captures pin `encoding="utf-8"`; keep the Windows
    (`shell=True` cmd string) and non-Windows (argv list, `shell=False`) build
    paths as **separate** branches.
  - `docs/SETUP.md` documents the uv-managed-Python Tk preflight and the
    "`uv: command not found` from the Dock" symptom.

## Mac breakage audit (deliverable #1)

The full enumeration of what breaks a Mac contributor today, each mapped to the
task that fixes it. This list is reproduced in `docs/SETUP.md` so user ask #1 is a
discoverable artifact, not just background.

| # | Breakage (file) | Effect on Mac | Fixed by |
|---|-----------------|---------------|----------|
| 1 | `ctps/__main__.py:83` `_DUMP_EXE` = `ctps_dump.exe` | binary never found; CTPS build-and-view dead | T1 |
| 2 | `ctps/__main__.py:240` toolchain only via vswhere/vcvars | "no host C++ toolchain found" on every non-Windows host | T1 |
| 3 | `.vscode/settings.json:2` `pythonPath` = `.venv/Scripts/python.exe` | Build Routes/Editor/Sim tasks can't find Python (`.venv/bin/python` on Mac) | T4 (uv run) + T5 |
| 4 | `.vscode/settings.json:18` `cmake.sourceDirectory` = hard-coded `C:/Users/wangd/...` | broken on **every** non-wangd machine, Mac included | T5 |
| 5 | `.vscode/c_cpp_properties.json` `${env:LOCALAPPDATA}`/`${env:USERPROFILE}`/`.exe` | C++ IntelliSense dead on Mac (builds unaffected) | T6 |
| 6 | `.vscode/tasks.json` 8 tasks invoke `pwsh` | tasks need PowerShell installed; and even then the Windows-venv path (#3) breaks them | T4 (+ T7 deletes the wrappers) |
| 7 | GUI/Dock-launched VS Code lacks profile PATH | `uv`/`arduino-cli` not found from a Dock launch | T4 login-shell variants + T10 docs |
| 8 | macOS serial nodes appear as `cu.*` **and** `tty.*` | false "multiple devices" on one board | T3 dedupe |
| 9 | Editor needs a Tk-enabled Python | `ModuleNotFoundError: _tkinter` on pyenv/bare Python | T10 docs (uv Python / `brew install python-tk`) |

**Confirmed already cross-platform (no work needed):** `_open_in_browser`
branches on `sys.platform`; `WRO2026/test/CMakeLists.txt` has `if(MSVC)/else()`
branches with no hard-coded paths; CI builds the host tests on Ubuntu with plain
`cmake` (the model for the Mac CTPS path); `visualizer.py` uses `Agg`;
`tools/path_planner/README.md` already has a Linux/macOS venv branch; all path
handling uses `pathlib`; `build/` is already gitignored.

## Acceptance Criteria

- [ ] **AC1** On macOS/Linux, `python -m tools.path_planner.ctps --fixture` builds
      `ctps_dump` via plain `cmake` (no vcvars), renders the HTML, and exits 0.
- [ ] **AC2** On Windows, CTPS behaves exactly as before (vcvars+Ninja), exits 0.
- [ ] **AC3** CTPS resolves `ctps_dump` (no extension) on non-Windows and
      `ctps_dump.exe` on Windows, and finds the binary whether it lands at the
      build root or in a `Debug/`/`Release/` subdir.
- [ ] **AC4** When `cmake` or a C++ compiler is missing on non-Windows, CTPS prints
      a clear message naming the missing tool and the `--trace` fallback, and
      exits non-zero with no Python traceback.
- [ ] **AC5** `--trace <file>` renders on every OS, requiring no compiler (unchanged).
- [ ] **AC6** With **no `pwsh` installed**, every VS Code task runs: Build Routes,
      Editor, Sim: Run PathTracker Demo, Sim: Plot PathTracker Demo, and CTPS via
      `uv run`; Arduino verify/upload/list-boards/monitor via the Python helper.
      **Static half (CI-enforced by TC19):** `tasks.json` contains no `pwsh` string
      and every planner/Arduino task routes through `uv run`/the helper. **Manual
      half (Mac gate):** launched from the macOS Dock, a planner task resolves
      `uv`/`arduino-cli` — verified by hand, and T7 must not merge before this passes.
- [ ] **AC7** `arduino_cli_task.py` filters `board list` by the configured FQBN so
      unrelated serial devices (a second Arduino, a USB dongle, a Bluetooth `cu.*`
      node) don't trip the multiple-boards error; it uploads/monitors to the single
      auto-detected board; on macOS it collapses `cu.`/`tty.` pairs for one physical
      board and prefers `/dev/cu.*`; it errors clearly on zero boards, genuinely-
      multiple boards, and a missing `arduino-cli`.
- [ ] **AC8** `.vscode/settings.json` contains no `wro2026.pathPlanner.pythonPath`,
      and `cmake.sourceDirectory` is `${workspaceFolder}/WRO2026/test`.
- [ ] **AC9** `.vscode/c_cpp_properties.json` has a macOS configuration entry
      (`~/Library/Arduino15` paths) alongside the Windows one.
- [ ] **AC10** New pytest cases pass on Linux CI and exercise both OS branches of
      CTPS (mocking `sys.platform`) and the Arduino resolvers (canned JSON,
      including the macOS `cu.`/`tty.` dedupe case).
- [ ] **AC11** `uv run --extra dev pytest tools/path_planner/tests/` is green, and
      the full host `ctest` still passes (the CMake change breaks no target).
- [ ] **AC12** `docs/SETUP.md` covers prerequisites, uv, arduino-cli + core/libs,
      tests, sim, editor, CTPS, host C++ tests, and the Arduino flash/serial flow
      for both Mac and Windows — including the uv-Python Tk preflight and the
      Dock-PATH symptom. README links to it.
- [ ] **AC13** `tools/path_planner_task.ps1` and `tools/arduino_cli_task.ps1` are
      deleted, and no file references them (CI-checkable via `git grep`).
- [ ] **AC14** The task orchestration survives the rework: the Verify→Build Routes
      and Upload→Verify `dependsOn` edges, `dependsOrder: "sequence"`, and the
      default build group (`group.isDefault`) are unchanged, so Ctrl+Shift+B still
      triggers the default build and the verify→upload chain still runs in order.
- [ ] **AC15** `docs/SETUP.md` reproduces the Mac breakage audit (every breakage
      from the audit table, incl. the absolute `cmake.sourceDirectory` line, mapped
      to its fix), satisfying user ask #1 as a discoverable artifact.

## Data Model / Contracts

### `tools/arduino_cli_task.py` — CLI contract

Invoked from `tasks.json` as:

```
uv run python tools/arduino_cli_task.py <action> \
    --cli <cliPath> --fqbn <fqbn> --sketch <sketchPath> \
    [--port <port|auto>] [--baud <baud>]
```

`action ∈ {verify, upload, list-boards, monitor}`. Mirrors the `.ps1` switch.
Reads the same `wro2026.arduino.*` settings values, passed as flags by the task.

### Pure, unit-testable functions (no subprocess in the resolvers)

```python
def resolve_serial_port(detected: list[dict], configured: str | None) -> str:
    """Return the serial port to use.

    detected: parsed detected_ports from `arduino-cli board list --json --fqbn
        <fqbn>` — i.e. ALREADY filtered by dispatch to ports whose identified
        board matches the UNO R4 WiFi FQBN (the .ps1 did this via --fqbn). Each
        item is shaped {"port": {"address": "...", ...}, ...}.
    configured: the wro2026.arduino.port value ("auto"/"" => autodetect).

    - configured set and != "auto"  -> return it.
    - autodetect: collect addresses; on macOS collapse cu./tty. pairs that share
      the same usbmodem/usbserial suffix, preferring the cu.* node (a tiebreaker
      AFTER the FQBN filter, not a substitute for it).
    - exactly one -> return it; zero -> raise; >1 distinct -> raise (list them).
    """

def resolve_baud(configured: str | None) -> str:
    """Return baud or raise if unset (monitor only)."""

def dispatch(action: str, cli: str, fqbn: str, sketch: str,
             port: str | None, baud: str | None) -> int:
    """Resolve arduino-cli on PATH (clear error if absent), then shell out for
    the action. For upload/monitor, run `board list --json --fqbn <fqbn>` and
    pass its detected_ports to resolve_serial_port. Returns the child exit code.
    Only this fn touches subprocess."""

def main(argv: list[str] | None = None) -> int: ...
```

### CTPS binary discovery (replaces the single `_DUMP_EXE` constant)

```python
def _dump_exe_name() -> str:
    return "ctps_dump.exe" if sys.platform.startswith("win") else "ctps_dump"

def _find_dump_exe() -> Optional[Path]:
    """Glob the build dir for the dump binary, generator-agnostically:
    search _BUILD_DIR and its Debug/ and Release/ subdirs for the platform
    exe name; return the newest match, or None if not found."""
```

### CTPS build branch (separate Windows / non-Windows paths)

```python
def _build_dump_windows(vcvars: Path) -> int:  # existing shell=True cmd path, unchanged
def _build_dump_posix() -> int:
    """argv list, shell=False:
       cmake -S <_TEST_DIR> -B <_BUILD_DIR>
       cmake --build <_BUILD_DIR> --target ctps_dump
    Returns exit code; OSError (cmake/compiler absent) is caught by the caller."""
```

## Error Handling

- **Non-Windows, `cmake` not on PATH:** `_build_dump_posix` raises `FileNotFoundError`
  (from `subprocess`); caller prints `ctps: cmake not found on PATH. Install CMake
  and a C++ compiler (g++/clang).` + the `--trace` hint; returns non-zero.
- **Non-Windows, configure/build fails (no compiler, CMake error):** non-zero
  return code surfaces with the build output above + the `--trace` hint (mirrors
  the existing Windows message).
- **Build succeeds but binary not found (odd generator):** `_find_dump_exe()`
  returns `None`; raise the existing "build reported success but the dump
  executable is missing at <build dir>" message, now naming the searched dir.
- **`arduino-cli` not found:** `dispatch` raises with the install hint
  (`brew install arduino-cli` / `winget install ArduinoSA.CLI`).
- **Zero boards / multiple distinct boards:** clear messages, as the `.ps1` did;
  multiple lists the addresses and points at `wro2026.arduino.port`.
- **macOS Tk missing for the Editor task:** documented in SETUP.md (use
  uv-managed Python; `brew install python-tk` fallback) — no code change.
- **`uv` not on PATH from Dock-launched VS Code:** addressed by the login-shell
  task variant; SETUP.md documents the symptom and the `code .` fallback.

## Testing Strategy

**Levels:** Unit (mock `sys.platform`/`subprocess`), plus the existing host
`ctest` and the full pytest suite as regression. CI stays Linux-only.

| ID  | Test Case | Type | Expected Behavior |
|-----|-----------|------|-------------------|
| TC1 | `_dump_exe_name()` with `sys.platform="win32"` | Unit | returns `ctps_dump.exe` |
| TC2 | `_dump_exe_name()` with `sys.platform="darwin"`/`"linux"` | Unit | returns `ctps_dump` (no ext) |
| TC3 | `_find_dump_exe()` finds binary at build root | Unit | returns that path |
| TC4 | `_find_dump_exe()` finds binary in `Debug/` subdir only | Unit | returns the subdir path |
| TC5 | `_find_dump_exe()` with no binary | Unit | returns `None` |
| TC6 | non-Windows build branch chosen when `sys.platform != win` | Unit | calls `_build_dump_posix` (cmake argv), not vcvars |
| TC7 | non-Windows, `subprocess` raises `FileNotFoundError` (no cmake) | Unit | prints cmake-not-found + `--trace` hint, returns non-zero, no traceback |
| TC8 | Windows branch still calls `_find_vcvars`/`_build_dump_windows` | Unit | vcvars path unchanged |
| TC9 | `--trace` flow on non-win (mock platform) | Unit | renders, never invokes a compiler |
| TC10 | `resolve_serial_port` with explicit non-"auto" port | Unit | returns it verbatim |
| TC11 | `resolve_serial_port` single board | Unit | returns its address |
| TC12 | `resolve_serial_port` macOS `cu.usbmodemX` + `tty.usbmodemX` | Unit | collapses to the single `cu.*` address |
| TC13 | `resolve_serial_port` zero boards | Unit | raises with the "connect a board" message |
| TC14 | `resolve_serial_port` two genuinely-distinct boards | Unit | raises, lists both addresses |
| TC15 | `resolve_baud` unset | Unit | raises "set baudRate" |
| TC16 | `dispatch` with `arduino-cli` absent (mock resolver) | Unit | raises install-hint error |
| TC17 | `dispatch` verify (mock subprocess) builds `compile --fqbn <fqbn> <sketch>` | Unit | correct argv, returns child code |
| TC18 | Full host `ctest` after CMake `RUNTIME_OUTPUT_DIRECTORY` pin | Integration | all existing targets still pass |
| TC19 | Parse `tasks.json`; assert no `command`/`args` contains `pwsh` and every planner/Arduino task routes through `uv run`/the helper | Unit | passes (CI-enforces AC6's static half) |
| TC20 | `resolve_serial_port` with the R4 plus an unrelated `cu.Bluetooth-*`/USB-serial device in the FQBN-filtered list | Unit | resolves to the single R4 address, no "multiple devices" |
| TC21 | non-Windows: `cmake` present but configure/build exits non-zero (compiler absent) | Unit | prints build-failed + `--trace` hint, returns non-zero, no traceback (distinct from the `FileNotFoundError` path in TC7) |
| TC22 | `git grep` / repo scan for `path_planner_task.ps1` / `arduino_cli_task.ps1` references | Unit | no references remain (AC13) |

**Test data:** canned `arduino-cli board list --json` dicts as Python literals
(incl. the macOS `cu.`/`tty.` pair). `monkeypatch.setattr(sys, "platform", ...)`
for OS branching; `monkeypatch`/`unittest.mock` for `subprocess.run`.
**Run command:** `uv run --extra dev pytest tools/path_planner/tests/ -v`
and `ctest --test-dir build/test --output-on-failure`.

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | CTPS orchestrator cross-platform fix | — | high | `tools/path_planner/ctps/__main__.py` | Add `_dump_exe_name()` + `_find_dump_exe()` (glob root + `Debug/`/`Release/`). Split build into `_build_dump_windows` (existing `shell=True` vcvars/Ninja path, unchanged) and `_build_dump_posix` (argv list, `shell=False`: `cmake -S <_TEST_DIR> -B <_BUILD_DIR>` then `cmake --build <_BUILD_DIR> --target ctps_dump`). In `_produce_trace_json`, branch on `sys.platform`: Windows keeps `_find_vcvars`; non-Windows skips vcvars and calls `_build_dump_posix`, catching `FileNotFoundError` for a clean "cmake not found" + `--trace` hint. Pin `encoding="utf-8"` on every `subprocess.run` text capture. Replace `_DUMP_EXE` usage with `_find_dump_exe()`. Keep `_open_in_browser` as-is. Satisfies AC1–AC5. |
| T2 | Pin ctps_dump output dir in CMake (future-proofing) | — | low | `WRO2026/test/CMakeLists.txt` | On the `ctps_dump` target, `set_target_properties(... RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})` plus the `_DEBUG`/`_RELEASE` multi-config variants = `${CMAKE_BINARY_DIR}`, so the binary lands at the build root even under a multi-config generator (Xcode/MSBuild). NOTE: under the default single-config generators (Make on Mac/Linux, Ninja on Windows) the binary already lands there — **T1's glob is what makes AC3 pass**; this pin only removes the multi-config edge case. Change nothing else; do not touch other targets. Satisfies AC11; backstops AC3. |
| T3 | New cross-platform Arduino helper | — | med | `tools/arduino_cli_task.py` | Port `arduino_cli_task.ps1` to Python per the Contracts section: pure `resolve_serial_port` (FQBN-filtered input, then macOS `cu.`/`tty.` dedupe preferring `cu.*`), `resolve_baud`, and `dispatch` (only fn touching `subprocess`; resolves `arduino-cli` via `shutil.which` with a clear install hint; for upload/monitor runs `board list --json --fqbn <fqbn>` and feeds detected_ports to the resolver — **preserving the .ps1's FQBN filter**). `main(argv)` with argparse: `<action> --cli --fqbn --sketch [--port] [--baud]`. Stdlib only, UTF-8, Doxygen-style docstrings. Satisfies AC7. |
| T4 | Rework VS Code tasks (drop pwsh) | T3 | med | `.vscode/tasks.json` | All four planner tasks — Build Routes, Editor: Open Field Editor GUI, Sim: Run PathTracker Demo, Sim: Plot PathTracker Demo — → `uv run python <script> [args]` (drop the `${config:...pythonPath}` arg; keep the script paths + `sim.goals` + `--plot`). CTPS already uses `uv run` — keep. Arduino tasks → `uv run python tools/arduino_cli_task.py <action> --cli ${config:wro2026.arduino.cliPath} --fqbn ... --sketch ... [--port ...] [--baud ...]`. **Preserve the existing orchestration exactly:** keep `dependsOn` (Verify→'Path Planner: Build Routes'; Upload→Verify), `dependsOrder: "sequence"`, and `group: {kind: "build", isDefault: true}` on the tasks that have them today — change only `command`/`args`. Add `osx`/`linux` task variants that run through a login shell (`options.shell` = `${env:SHELL}` with **`-lc`** — login, non-interactive, so PATH is sourced without interactive-shell hangs/prompt noise) so Dock-launched VS Code resolves `uv`/`arduino-cli`. No `pwsh` anywhere. Satisfies AC6, AC14. |
| T5 | Fix VS Code settings | — | low | `.vscode/settings.json` | Remove `wro2026.pathPlanner.pythonPath`. Change `cmake.sourceDirectory` to `${workspaceFolder}/WRO2026/test`. Keep `wro2026.arduino.*`, `sim.*`, pytest keys. Satisfies AC8. |
| T6 | Add macOS C/C++ IntelliSense config | — | low | `.vscode/c_cpp_properties.json` | Add a second `configurations[]` entry "Arduino UNO R4 WiFi (macOS)" using `${env:HOME}/Library/Arduino15/...` for the renesas core + tools and `${env:HOME}/Documents/Arduino/libraries/**`, `intelliSenseMode` `macos-gcc-arm`, `arm-none-eabi-g++` with no `.exe`. **Do NOT hard-copy the Windows `7-2017q4` toolchain version segment** — it drifts with the installed core; use a glob (`.../arm-none-eabi-gcc/*/bin/arm-none-eabi-g++`) for `compilerPath`/include paths, or note in SETUP.md that the contributor sets the version to match their installed core. Keep the Windows entry. Satisfies AC9. |
| T7 | Delete the PowerShell wrappers | T4 | low | `tools/path_planner_task.ps1`, `tools/arduino_cli_task.ps1` | Delete both files after T4 stops referencing them. `git grep` the repo to confirm no references remain (TC22). **Gate:** do not land this until AC6's manual Mac half is hand-verified — once the `.ps1` are gone, the only validation that the new tasks actually launch is manual. Satisfies AC13. |
| T8 | Unit tests — CTPS OS branching | T1 | low | `tools/path_planner/tests/test_ctps_cross_platform.py` | New file: TC1–TC9. Mock `sys.platform`; build a temp `_BUILD_DIR` with a dummy binary for `_find_dump_exe`; mock `subprocess.run`/`_find_vcvars`. Follow the headless pattern — no real compiler. Satisfies AC10. |
| T9 | Unit tests — Arduino helper | T3 | low | `tools/path_planner/tests/test_arduino_cli_task.py` | New file: TC10–TC17. Canned board-list dicts (incl. macOS `cu.`/`tty.` pair); mock `subprocess`/`shutil.which`. Satisfies AC10. |
| T10 | Write docs/SETUP.md | T4 | low | `docs/SETUP.md` | Full Mac+Windows guide per the Notes section: prerequisites (Homebrew/winget, Python 3.11+, uv, arduino-cli + renesas_uno core + MatrixMiniR4/ArduinoBLE/PubSubClient), running tests/sim/editor/CTPS, host C++ tests, VS Code tasks, Arduino flash/serial. **Reproduce the Mac breakage audit table (deliverable #1, AC15).** Include the uv-Python Tk preflight (`python -c "import tkinter; tkinter.Tk()"`, `brew install python-tk` fallback) and the Dock-PATH note — **recommend launching VS Code via `code .` from a terminal as the primary path** (inherits the shell PATH), with the login-shell task variants as the secondary mechanism. Satisfies AC12, AC15. |
| T11 | Link SETUP.md from README | T10 | low | `README.md` | Add a "Setup / onboarding" link to `docs/SETUP.md` near "How to use"; correct the "VS Code tasks (PowerShell-driven)" line to reflect the cross-platform `uv run`/Python tasks. Satisfies AC12. |
| T12 | Add LF normalization | — | low | `.gitattributes` | Add `*.py`, `*.md`, `*.json` (and the repo's other text types) as LF-normalized — today the file carries only golden-routes rules, so these are uncovered. Run `git add --renormalize .` so existing tracked files pick up the rule (the real risk is the Windows devs' CRLF working copies, not a fresh Mac clone). |

**Parallel-startable (no blockers):** T1, T2, T3, T5, T6, T12.
**Serialized:** T4←T3; T7←T4; T8←T1; T9←T3; T10←T4; T11←T10.
No two tasks edit the same file.

## Notes for Implementer

- **Windows must not regress.** The only change to the Windows CTPS path is the
  `_DUMP_EXE` constant → `_find_dump_exe()` lookup; vcvars discovery and the
  `shell=True` cmd build string stay byte-for-byte. Verify a Windows CTPS run
  still works (local MSVC toolchain is available — see project memory).
- **CMakeLists change is in scope and safe:** it is host-test-harness build wiring
  under `WRO2026/test/`, not robot firmware (`.ino/.cpp/.h`) or pipeline logic.
  It only pins where one target's binary lands; run the full `ctest` to confirm no
  target breaks (the compile-fail ctests use `$<CONFIG>`, empty under single-config
  generators — unaffected).
- **VS Code login-shell detail (the day-one Mac trap):** Dock-launched VS Code does
  NOT inherit the shell profile PATH, so a bare `command: "uv"` fails with
  `uv: command not found`. **Primary recommendation:** launch VS Code via `code .`
  from a terminal (inherits PATH) — document this first in SETUP.md. **Secondary
  mechanism:** OS-keyed task overrides — on `osx`/`linux` set `options.shell` to the
  user's `$SHELL` with `-lc` (login, NON-interactive; avoid `-i`, which can hang or
  spew prompt noise and whose rc may guard PATH setup behind an interactive check)
  and pass the command as a shell string. Test on the real Mac; this is the highest-
  risk item because CI can't catch it.
- **macOS serial nodes:** one physical board surfaces as both `/dev/cu.usbmodemX`
  and `/dev/tty.usbmodemX`. Upload/monitor through `cu.*` (non-blocking). The
  dedupe must collapse a pair sharing the `usbmodem*`/`usbserial*` suffix; only
  raise "multiple devices" for genuinely distinct boards.
- **Tk on macOS:** the Editor needs a Tk-enabled Python. uv-managed CPython bundles
  Tk; a pyenv/bare framework build often does not (`ModuleNotFoundError: _tkinter`).
  Steer Mac users to uv's Python in SETUP.md; `brew install python-tk` is the
  Homebrew-Python fallback. No code change.
- **Encoding:** pin `encoding="utf-8"` on every `subprocess.run(text=True, ...)` in
  `__main__.py` (the `--trace` read already does). Keeps the dump JSON deterministic
  across the Windows ANSI code page vs UTF-8.
- **Keep resolvers pure** in `arduino_cli_task.py` so tests feed canned JSON with no
  subprocess mocking; only `dispatch` shells out. Three module-level functions +
  `main(argv)` — no class, per the repo's no-speculative-abstraction rule.
- **Branch hygiene:** land this on a fresh feature branch off `main`
  (e.g. `fix/issue-51-ctps-cross-platform`), NOT in the current
  `issue-78-editor-delag-ux` worktree, which has unrelated uncommitted edits.
- **Worktree venv gotcha:** if executed in a worktree, run `uv sync` there first
  (the `.venv` is per-checkout and gitignored) before the pytest run.
- **Binary-location impact:** T2 changes where a `cmake --build` drops `ctps_dump`
  (pinned to the build root for multi-config generators). Anyone relying on a
  generator-specific subdir path should use the build root. `build/` is already
  gitignored, so nothing new is committed. (Ignore the stale `__main__.py:80`
  comment "gitignored by T11" — it references the original CTPS plan's numbering,
  not this spec's T11.)
- **Rollback:** the change is additive + isolated to tooling; reverting the commit
  restores the Windows-only behavior with no firmware impact.
