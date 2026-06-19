# PATH-resolved arduino-cli for VS Code tasks (issue #68) Plan

**Goal:** Stop hardcoding the Arduino IDE's private bundled `arduino-cli.exe` path in the VS Code tasks and default to the standalone `arduino-cli` resolved from PATH, while keeping the setting overridable to an absolute binary path.

**Approach:** Change the `wro2026.arduino.cliPath` default in `.vscode/settings.json` from the long IDE-internal path to the bare command `arduino-cli`, and replace the `Test-Path $CliPath` existence guard in `tools/arduino_cli_task.ps1` (which is `$false` for a bare PATH command) with a `Get-Command` resolve that accepts both a bare command and an absolute path, then reassigns `$CliPath` to the resolved `.Source`. This is the implementation the issue specifies; the sub-decisions (doc placement, hardening, verification, error wording) were resolved in planning Q&A.

## Scope

- **In scope:**
  - `.vscode/settings.json` — set `wro2026.arduino.cliPath` default to `"arduino-cli"`, and add a `//` (JSONC) comment above it documenting the IDE-bundled fallback path for people who don't install the standalone CLI.
  - `tools/arduino_cli_task.ps1` — replace the `Test-Path`-based existence guard (lines 25-27) with a `Get-Command` resolve that handles both a bare command on PATH and an absolute path, reassigning `$CliPath` to the resolved absolute `.Source`, and throwing a clear install/set-setting message when nothing resolves.
- **Out of scope:**
  - `.vscode/tasks.json` — the 4 Arduino tasks pass `${config:wro2026.arduino.cliPath}` through positionally; no edit needed.
  - `.github/workflows/ci.yml` — CI uses `arduino/setup-arduino-cli@v2` and calls `arduino-cli` directly; it never routes through this script or settings file, so it is unaffected.
  - Other `wro2026.*` settings, and the `port` / `baudRate` empty-arg sentinel handling (`"auto"`) — left as-is.
  - Converting the script to a Pester test (none exist in the repo; verification is manual).
  - Writing an ADR — this is a one-PR tooling tweak, not a cross-module design rule.

## Decisions

- **Default value `"arduino-cli"`** — bare command resolved from PATH; the standalone CLI (`ArduinoSA.CLI` 1.5.1, confirmed at `C:\Program Files\Arduino CLI\arduino-cli.exe`) survives IDE updates and matches the invocation CI and the docs already assume.
- **Doc placement: settings.json `//` comment** — closest to where someone editing the setting would look. VS Code reads `settings.json` as JSONC, so the comment is valid (the file currently has none; adding one makes it JSONC).
- **Hardening: lightly hardened** — the issue's `Get-Command` + `$CliPath = $resolved.Source`, plus `-CommandType Application` so the resolve only matches executables (a bare name on PATH or an absolute path, incl. forward-slash) and is never shadowed by an alias/function/cmdlet whose `.Source` is empty, and `Select-Object -First 1` so multiple application matches can't make `$CliPath` an array. No `version` probe of the resolved binary (that extra check was the rejected "fully hardened" option — not warranted for a dev-only resolve).
- **Error message: winget hint + point to setting** — `"arduino-cli not found ('$CliPath'). Install it (winget install ArduinoSA.CLI) or set wro2026.arduino.cliPath to the binary path."` Matches how it was installed; does not embed the long IDE path in the throw.
- **Verification: branch-test the three resolve paths + one full compile** — proves PATH-resolve, absolute-path back-compat, and the not-found throw, then confirms the resolved binary actually builds the sketch end-to-end.
- **Fable approach consult (plan Phase 4.5) skipped** — single reasonable approach + trivial, reversible, low-risk change; the skill permits skipping in exactly this case.

## Acceptance Criteria

- [ ] With `wro2026.arduino.cliPath` = `arduino-cli`, invoking `tools/arduino_cli_task.ps1 verify arduino-cli <fqbn> WRO2026` resolves the CLI from PATH and compiles (exit 0). (Satisfied by T1, T2; verified by T3 / TC1)
- [ ] An absolute path in the setting still works — `Get-Command` resolves it and `& $CliPath` runs. (Satisfied by T2; verified by T3 / TC2)
- [ ] A missing or misspelled command fails with a non-zero exit and a message that **contains** `arduino-cli not found`, `winget install ArduinoSA.CLI`, and `wro2026.arduino.cliPath` — **not** a confusing `Test-Path` throw — and no compile is attempted. (Satisfied by T2; verified by T3 / TC3)
- [ ] `.vscode/settings.json` default is `"arduino-cli"` with a `//` comment recording the IDE-bundled fallback path, and the file still parses (comments stripped → valid JSON) and loads in VS Code. (Satisfied by T1; verified by T3 / TC4a, TC4b)
- [ ] No changes to `tasks.json`, `ci.yml`, or other `wro2026.*` settings; the CI compile job is unaffected. (Scope guard; verified by T3 / TC5)
- [ ] **Both** the `Arduino: Verify` and `Arduino: Upload` tasks resolve the CLI and run with `cliPath = arduino-cli` (the issue names both; Upload also hits the second `& $CliPath` call site at line ~54). (Satisfied by T2; verified by T3 / TC4b, TC7)

## Data Model (if applicable)

Not applicable — no data structures introduced.

## API Contracts (if applicable)

Not applicable. The only "contract" is the script's parameter surface, which is unchanged: `arduino_cli_task.ps1` still takes `-Action -CliPath -Fqbn -SketchPath [-Port] [-BaudRate]`. `$CliPath` semantics widen from "must be an existing filesystem path" to "a bare command on PATH **or** an absolute path"; the downstream `& $CliPath ...` call sites are unchanged once `$CliPath` holds the resolved absolute `.Source`.

## Error Handling

- **`cliPath` resolves to nothing** (not on PATH and not an existing file): `throw "arduino-cli not found ('$CliPath'). Install it (winget install ArduinoSA.CLI) or set wro2026.arduino.cliPath to the binary path."` — fails fast before any task action runs (replaces the old `Test-Path` throw). `$ErrorActionPreference = 'Stop'` is already set.
- **`cliPath` collides with an alias/function/cmdlet**: `-CommandType Application` filters to the executable, so `$CliPath = $resolved.Source` is never set to an alias's empty `.Source` (which would make `& $CliPath` fail with a confusing empty-command error — the very failure the issue removes). If multiple application matches remain, `Select-Object -First 1` keeps `$CliPath` a single string so `& $CliPath` does not receive an array.
- **Resolved binary runs but the compile/upload fails**: unchanged — `Invoke-ArduinoCli` already propagates a non-zero `$LASTEXITCODE` via `exit $LASTEXITCODE`.
- **`cliPath` set to `""`**: out of the happy path — `cliPath` must never be empty. An empty `${config:...}` is dropped entirely by VS Code shell tasks on Windows; for this leading **Mandatory** positional that either shifts later positionals (the fqbn lands in `-CliPath`) or trips the Mandatory-parameter binder. Either way it fails before the resolve; the default `"arduino-cli"` and any override keep it non-empty.

## Testing Strategy

**Levels:** Integration (manual — there is no host test runner for the PowerShell tasks; the repo's pytest suite does not cover `.vscode/` or `tools/*.ps1`).

**Precondition:** TC1/TC2/TC7 require the standalone `arduino-cli` to be installed and resolvable on PATH (confirmed: `ArduinoSA.CLI` 1.5.1 at `C:\Program Files\Arduino CLI\arduino-cli.exe`). If absent, `winget install ArduinoSA.CLI` first, or the full-compile cases cannot pass.

| ID  | Test Case                                                                 | Type        | Expected Behavior                                                                 |
|-----|---------------------------------------------------------------------------|-------------|-----------------------------------------------------------------------------------|
| TC1 | Run script `verify` with bare `arduino-cli` (PATH resolve) + full compile | Integration | `Get-Command -CommandType Application` resolves to `C:\Program Files\Arduino CLI\arduino-cli.exe`; sketch compiles; exit 0 |
| TC2 | Run script `verify` with an absolute path to arduino-cli                  | Integration | Resolves the absolute path (incl. forward slashes); `& $CliPath` runs; exit 0 (back-compat) |
| TC3 | Run script `verify` with a bogus value (e.g. `arduino-cli-nope`)          | Integration | Non-zero exit; **no** compile attempted; stderr message **contains** `arduino-cli not found`, `winget install ArduinoSA.CLI`, and `wro2026.arduino.cliPath` (asserts the clear message, not merely that *a* throw fired) |
| TC4a| Strip `//` lines from `settings.json` and `JSON.parse`/`ConvertFrom-Json` the rest | Unit | Parses as valid JSON; `wro2026.arduino.cliPath` == `arduino-cli` (JSONC comment doesn't corrupt the file) |
| TC4b| Run the actual `Arduino: Verify` **and** `Arduino: Upload` VS Code tasks once | Integration (manual) | VS Code substitutes `${config:wro2026.arduino.cliPath}` → `arduino-cli`; both tasks resolve the CLI and run (the issue names both tasks) |
| TC5 | Confirm `tasks.json`, `ci.yml`, other `wro2026.*` settings unchanged       | Unit (diff) | `git diff --stat` shows only `.vscode/settings.json` + `tools/arduino_cli_task.ps1` edited; the plan doc is the only added/untracked file (`git status`) |
| TC6 | Set `cliPath` to a name that is an alias but **not** an application        | Integration | `-CommandType Application` filters the alias out → not-found throw fires; `$CliPath` is **never** set to the alias's empty `.Source` (guards the Axis-3 regression) |
| TC7 | Run script `list-boards` (and a dry `upload` path) with bare `arduino-cli` | Integration | Exercises the **second** `& $CliPath` site (`board list --json` in `Resolve-SerialPort`, line ~54) with the resolved value; command runs (board-detect output or the documented "no board" throw), proving Upload's call site works too |

**Test data:** No fixtures. Use the live PATH `arduino-cli` for TC1/TC7, its resolved `.Source` (or the IDE-bundled path if present) as the absolute path for TC2, a deliberately-wrong string for TC3, and a temporary alias (`Set-Alias arduino-cli Get-Date`) for TC6.

**Run command:**
```powershell
pwsh -NoProfile -ExecutionPolicy Bypass -File tools/arduino_cli_task.ps1 verify arduino-cli arduino:renesas_uno:unor4wifi WRO2026
```
(Repeat with an absolute path and a bogus value for TC2/TC3. Calling the script directly skips the VS Code `dependsOn: Path Planner: Build Routes` step — that is fine for exercising the resolve; the full compile confirms end-to-end.)

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Default cliPath to PATH command + document fallback | — | low | `.vscode/settings.json` | Change line 7 value from the IDE-bundled absolute path to `"arduino-cli"`. Add a `//` JSONC comment on the line directly above documenting the IDE-bundled fallback path (`C:/Users/<you>/AppData/Local/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe`) for users who prefer not to install the standalone CLI. Touch nothing else in the file. Satisfies AC4. |
| T2 | Replace Test-Path guard with Get-Command resolve | — | low | `tools/arduino_cli_task.ps1` | Replace lines 25-27 (the `if (-not (Test-Path $CliPath)) { throw ... }` block) with: `$resolved = Get-Command $CliPath -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1` then `if (-not $resolved) { throw "arduino-cli not found ('$CliPath'). Install it (winget install ArduinoSA.CLI) or set wro2026.arduino.cliPath to the binary path." }` then `$CliPath = $resolved.Source`. `-CommandType Application` is required so an alias/function/cmdlet shadowing the name can't be picked (its empty `.Source` would make `& $CliPath` fail with a confusing empty-command error). Keep `Set-StrictMode`, `$ErrorActionPreference`, and `$repoRoot` above it unchanged. Leave `& $CliPath @Arguments` (line ~39) and `& $CliPath board list ...` (line ~54) untouched — they now receive the resolved absolute path. Satisfies AC1, AC2, AC3. |
| T3 | Verify resolve branches + full compile + both call sites + diff scope | T1, T2 | low | — | Precondition: `arduino-cli` on PATH (else `winget install ArduinoSA.CLI`). Run TC1 (bare `arduino-cli` + full compile, expect exit 0), TC2 (absolute path, expect exit 0), TC3 (bogus value — assert non-zero exit, no compile, **and** the message text contains `arduino-cli not found` / `winget install ArduinoSA.CLI` / `wro2026.arduino.cliPath`), TC6 (alias-only name → not-found throw, `$CliPath` never empty), TC7 (`list-boards` exercises the second `& $CliPath` site used by Upload). Confirm TC4a (settings.json minus `//` lines is valid JSON), TC4b (run the real `Arduino: Verify` + `Arduino: Upload` VS Code tasks once), and TC5 (`git diff --stat` only the two source files; plan doc the only added file). Report each result. Satisfies AC1-AC6. |

T1 and T2 edit different files with no interdependency — they run in parallel. T3 verifies both and is blocked by both.

## Notes for Implementer

- **`Get-Command -CommandType Application` resolves both forms:** a bare `arduino-cli` resolves via PATH; an absolute path (including forward slashes, as the old default used) returns an `ApplicationInfo` whose `.Source` is the full path. `-CommandType Application` is load-bearing, not cosmetic — without it, command precedence (Alias > Function > Cmdlet > Application) means an alias named `arduino-cli` would resolve first and `.Source` would be empty, so `$CliPath = $resolved.Source` would set `$CliPath = ''`. After `$CliPath = $resolved.Source`, the existing `& $CliPath ...` invocations work unchanged. `-ErrorAction SilentlyContinue` returns `$null` when nothing resolves, which the `if (-not $resolved)` guard catches even under `Set-StrictMode -Version Latest`.
- **Order matters:** the resolve block must stay where the old guard was — before `Resolve-SketchDir`, the helper functions, and the `switch ($Action)` — so a bad `cliPath` fails before any action runs.
- **JSONC comment:** `settings.json` currently has zero comments. Adding a `//` line is safe for VS Code (it parses settings as JSONC) but would break a strict `JSON.parse`; nothing in this repo parses `.vscode/settings.json` with a strict parser (CI doesn't read it), so this is fine. Keep the rest of the file as plain JSON.
- **Don't touch `tasks.json`:** the `${config:wro2026.arduino.cliPath}` substitutions on lines 100/128/153/175 are pass-through and already correct.
- **`cliPath` must never be empty** — VS Code drops an empty `${config:...}` shell-task arg entirely on Windows (shifting later positional args), and an empty value would also fail `Get-Command`. The default and any override stay non-empty. (Same class of gotcha as the `port`/`baudRate` `"auto"` sentinel.)
- **Memory follow-up (post-merge):** the `arduino-cli-headless-available` project memory cites the IDE-bundled path as *the* local arduino-cli; after this lands, the standalone PATH CLI is the primary and that path is the fallback. Update that memory in the plan-skill Learning phase / on merge.
- **Commit style:** Conventional Commits to match the repo (recent log: `feat(...)`, `docs(...)`, `test(...)`); a tooling change fits `chore(vscode):` or `build:`. No AI attribution in the commit or PR.
- **Rollback:** trivially reversible — revert the two one-area edits; no migration, no state.
