# Developer setup — macOS & Windows

How to get the WRO2026 developer tooling running on a fresh machine. The robot
firmware builds and flashes the same way everywhere (Arduino + `arduino-cli`);
the offline Python pipeline (path planner, sim, editor, CTPS) runs on macOS,
Linux, and Windows. Linux works too — it follows the macOS steps with your
distro's package manager.

> Only the host tooling differs by OS. Nothing about the robot, the field, or
> the generated `routes.*` is platform-specific.

## What used to be Windows-only (and is now fixed)

This guide came out of issue #51. For the record, here is everything that broke
a non-Windows contributor before this change, and where it was fixed:

| # | What broke | Effect on macOS/Linux | Fixed |
|---|------------|-----------------------|-------|
| 1 | CTPS hard-coded `ctps_dump.exe` | binary never found | `ctps/__main__.py` picks the name by platform |
| 2 | CTPS only located the toolchain via `vswhere`/`vcvars64.bat` | "no host C++ toolchain found" | non-Windows builds with plain `cmake` |
| 3 | `settings.json` `pythonPath` = `.venv/Scripts/python.exe` | planner VS Code tasks couldn't find Python | tasks now use `uv run`; the setting was removed |
| 4 | `settings.json` `cmake.sourceDirectory` was a hard-coded `C:/Users/...` path | broke on every other machine | now `${workspaceFolder}/WRO2026/test` |
| 5 | `c_cpp_properties.json` used `%LOCALAPPDATA%`/`%USERPROFILE%`/`.exe` | C++ IntelliSense dead | a macOS configuration entry was added |
| 6 | every VS Code task shelled out through `pwsh` | required PowerShell + a Windows venv layout | tasks run via `uv run`; the `.ps1` wrappers were deleted |
| 7 | a Dock-launched VS Code has no shell `PATH` | `uv`/`arduino-cli` not found | login-shell task variants + the `code .` guidance below |
| 8 | one board shows as both `/dev/cu.*` and `/dev/tty.*` on macOS | false "multiple devices" | the Arduino helper de-dupes the pair |
| 9 | the editor needs a Tk-enabled Python | `ModuleNotFoundError: _tkinter` | use the uv-managed Python (see the Tk preflight) |

## 1. Prerequisites

### macOS

```bash
# Command Line Tools give you git, clang, and make
xcode-select --install

# Homebrew (https://brew.sh) if you don't have it, then:
brew install uv cmake arduino-cli
```

- `uv` — the Python project/venv manager this repo uses.
- `cmake` + a C++ compiler (clang from the Command Line Tools) — only needed
  for the CTPS build-and-view flow and the host C++ tests. `cmake` is **not**
  bundled with the Command Line Tools, so install it from Homebrew.
- `arduino-cli` — compile/upload/monitor the firmware.

### Windows

```powershell
winget install astral-sh.uv
winget install ArduinoSA.CLI      # arduino-cli on PATH
winget install Git.Git
```

- A host C++ toolchain for the CTPS build / host tests is **optional**. If you
  have Visual Studio Build Tools 2022 with the C++ workload, CTPS finds it
  automatically via `vswhere` → `vcvars64.bat`. Without it, use the CTPS
  `--trace` flow (below), which needs no compiler.

### Linux (Ubuntu/Debian)

```bash
sudo apt-get install -y build-essential cmake xvfb
# uv: https://docs.astral.sh/uv/getting-started/installation/ (pipx install uv works too)
# arduino-cli: https://arduino.github.io/arduino-cli/latest/installation/
```

`xvfb` is only needed to run the Tk-based editor tests headlessly (see Tests).

### Arduino core + libraries (all platforms)

```bash
arduino-cli core update-index
arduino-cli core install arduino:renesas_uno
arduino-cli lib install MatrixMiniR4 ArduinoBLE PubSubClient
```

### Apple Silicon note for UNO R4 / `renesas_uno`

On Apple Silicon Macs, the Arduino Renesas UNO core currently pulls an
Intel-only GNU Arm toolchain (`arm-none-eabi-gcc 7-2017q4`). If Rosetta 2 is
not installed, `arduino-cli compile` fails with:

```text
bad CPU type in executable
```

Install Rosetta once, then rerun the core/tool build:

```bash
softwareupdate --install-rosetta --agree-to-license
```

## 2. Get the code and the Python environment

```bash
git clone https://github.com/WinnerWang971119/WRO2026_Code.git
cd WRO2026_Code
uv sync --extra dev      # creates .venv and installs locked deps incl. pytest
```

`uv sync` reads `pyproject.toml` + `uv.lock`, so every contributor gets the same
dependency graph. The `.venv` is per-checkout and gitignored — if you use a git
**worktree**, run `uv sync` inside it too.

### Editor Tk preflight (macOS)

The field editor uses Tkinter. uv-managed CPython bundles Tk, so `uv run` is the
reliable way to launch it. Verify Tk works:

```bash
uv run python -c "import tkinter; tkinter.Tk(); print('tk ok')"
```

If you instead use a Homebrew Python and this fails with `ModuleNotFoundError:
_tkinter`, install Tk for it: `brew install python-tk`.

## 3. Everyday commands

Run these from the repo root. `uv run` auto-activates the project's `.venv`.

| Task | Command |
|------|---------|
| Regenerate `routes.h`/`routes.cpp` | `uv run python tools/path_planner/build_routes.py` |
| Run the Python test suite | `uv run --extra dev pytest tools/path_planner/tests/ -v` |
| Run the PathTracker sim | `uv run python tools/path_planner/sim/demo.py --goals Start,finial` |
| Plot the sim | `uv run python tools/path_planner/sim/demo.py --goals Start,finial --plot` |
| Open the field editor | `uv run python tools/path_planner/edit_field.py` |
| CTPS — build + render + open | `uv run python -m tools.path_planner.ctps` |

On **Linux**, the editor's Tk tests need a display; run the suite under xvfb
(this is what CI does):

```bash
xvfb-run -a uv run --no-sync pytest tools/path_planner/tests/ -v
```

### CTPS (Clear Trajectory Prediction Simulator)

```bash
uv run python -m tools.path_planner.ctps             # build ctps_dump, render, open
uv run python -m tools.path_planner.ctps --fixture   # dump the synthetic fixture mission
uv run python -m tools.path_planner.ctps --trace run.json --no-open   # no compiler needed
```

- On **macOS/Linux** the build uses plain `cmake` with the default generator and
  whatever C++ compiler is on `PATH` (clang/g++). If `cmake` or a compiler is
  missing, CTPS prints a clear message and points you at `--trace`.
- On **Windows** CTPS locates MSVC automatically (`vswhere` → `vcvars64.bat`).
- `--trace <existing_dump.json>` skips the build entirely and works on every OS
  with no compiler.

### Host C++ tests

```bash
cmake -S WRO2026/test -B build/test
cmake --build build/test
ctest --test-dir build/test --output-on-failure
```

On macOS/Linux this uses clang/g++ from PATH; on Windows run it from a
Developer Command Prompt (or let CTPS's flow drive it).

## 4. VS Code tasks

The tasks in `.vscode/tasks.json` run through `uv run` (and `arduino-cli`) — no
PowerShell required on any OS. Available tasks: **Path Planner: Build Routes**,
**Editor: Open Field Editor GUI**, **Sim: Run/Plot PathTracker Demo**, **CTPS:
build & view**, and **Arduino: Verify/Upload/List Boards/Monitor Serial**.
`Ctrl/Cmd+Shift+B` runs *Arduino: Verify* (which first runs *Build Routes*).

### macOS: "uv: command not found" when running a task

A VS Code launched from the **Dock/Finder** does not inherit your shell's
`PATH`, so a task can fail with `uv: command not found` even though `uv` works
in your terminal.

- **Primary fix:** launch VS Code from a terminal so it inherits `PATH`:
  ```bash
  cd WRO2026_Code && code .
  ```
- The tasks also include a login-shell fallback (`osx`/`linux` run through
  `$SHELL -lc`), which sources your profile. If your shell only sets `PATH` in a
  non-login rc file, prefer the `code .` launch above.

## 5. Flashing the robot

Use the **Arduino: …** VS Code tasks, or `arduino-cli` directly:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026
arduino-cli board list
arduino-cli upload  --fqbn arduino:renesas_uno:unor4wifi --port <PORT> WRO2026
arduino-cli monitor --port <PORT> --config baudrate=115200
```

- **macOS:** the board enumerates as `/dev/cu.usbmodemXXXX` (use the `cu.` node,
  not `tty.`). The Upload/Monitor tasks auto-detect the port and collapse the
  `cu.`/`tty.` pair, so a single board never trips a "multiple devices" error.
  If you have more than one serial device, set `wro2026.arduino.port` in
  `.vscode/settings.json` to the exact `/dev/cu.*` path.
- **Windows:** the board is a `COMx` port; the UNO R4 WiFi is plug-and-play on
  Windows 10+.
- The monitor baud (`115200`) must match `Serial.begin(...)` in the firmware;
  garbled text usually means a baud mismatch.

## 6. C++ IntelliSense (VS Code)

`.vscode/c_cpp_properties.json` ships two configurations:

- **Arduino UNO R4 WiFi** — Windows (`%LOCALAPPDATA%/Arduino15`).
- **Arduino UNO R4 WiFi (macOS)** — `~/Library/Arduino15`.

Pick yours from the C/C++ configuration selector in the VS Code status bar
(bottom-right). Both pin the `arm-none-eabi-gcc` toolchain version (`7-2017q4`)
that the current `renesas_uno` core ships. If IntelliSense can't find the
compiler after a core update, open `c_cpp_properties.json` and change that
version segment to the one under
`Arduino15/packages/arduino/tools/arm-none-eabi-gcc/` on your machine. This
affects editor IntelliSense only — actual compiles via `arduino-cli` are
unaffected.
