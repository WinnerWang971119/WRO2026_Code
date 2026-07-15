"""Tests for the cross-platform arduino-cli task driver (tools/arduino_cli_task.py).

The two ``resolve_*`` helpers (plus ``resolve_cli`` and ``_needs_autodetect``)
are PURE, so they are exercised directly with canned inputs. ``dispatch`` is the
only function that spawns subprocesses; its tests mock ``act.subprocess.run`` and
``act.shutil.which`` so no real ``arduino-cli`` is ever launched. The argv
assertions reuse the SUT's own ``_resolve_sketch_dir`` to stay deterministic and
platform-independent.
"""

from __future__ import annotations

import subprocess

import pytest

from tools import arduino_cli_task as act


# ---------------------------------------------------------------------------
# resolve_serial_port -- pure, fed canned ``detected_ports`` lists.
# ---------------------------------------------------------------------------


def test_resolve_serial_port_explicit_non_auto_returned_verbatim() -> None:
    """TC10: an explicit, non-'auto' configured port wins regardless of detection."""
    detected = [{"port": {"address": "/dev/ttyACM5"}}]
    assert act.resolve_serial_port(detected, "/dev/ttyACM0") == "/dev/ttyACM0"


def test_resolve_serial_port_single_board_autodetected() -> None:
    """TC11: a single detected board resolves to its address under 'auto'."""
    detected = [{"port": {"address": "/dev/ttyACM0"}}]
    assert act.resolve_serial_port(detected, "auto") == "/dev/ttyACM0"


def test_resolve_serial_port_macos_cu_tty_pair_collapses_to_cu() -> None:
    """TC12: a macOS cu./tty. pair sharing a suffix collapses to the cu.* node."""
    detected = [
        {"port": {"address": "/dev/cu.usbmodem1101"}},
        {"port": {"address": "/dev/tty.usbmodem1101"}},
    ]
    assert act.resolve_serial_port(detected, "auto") == "/dev/cu.usbmodem1101"


def test_resolve_serial_port_zero_boards_raises() -> None:
    """TC13: no detected board raises ResolveError mentioning the empty case."""
    with pytest.raises(act.ResolveError, match="No Arduino board detected"):
        act.resolve_serial_port([], "auto")


def test_resolve_serial_port_two_distinct_boards_raises_listing_both() -> None:
    """TC14: two genuinely-distinct boards raise, listing BOTH addresses."""
    detected = [
        {"port": {"address": "/dev/ttyACM0"}},
        {"port": {"address": "/dev/ttyACM1"}},
    ]
    with pytest.raises(act.ResolveError) as excinfo:
        act.resolve_serial_port(detected, "auto")
    message = str(excinfo.value)
    assert "/dev/ttyACM0" in message
    assert "/dev/ttyACM1" in message


# ---------------------------------------------------------------------------
# resolve_baud -- pure.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("configured", [None, ""])
def test_resolve_baud_blank_raises(configured) -> None:
    """TC15: a None or empty baud raises ResolveError pointing at baudRate."""
    with pytest.raises(act.ResolveError, match="baudRate"):
        act.resolve_baud(configured)


def test_resolve_baud_non_blank_returned_verbatim() -> None:
    """TC15 (positive): a configured baud is returned verbatim."""
    assert act.resolve_baud("115200") == "115200"


# ---------------------------------------------------------------------------
# resolve_cli -- pure aside from a single shutil.which lookup (monkeypatched).
# ---------------------------------------------------------------------------


def test_resolve_cli_not_found_raises_with_install_hint(monkeypatch) -> None:
    """TC16: a missing cli (not a file, not on PATH) raises with the install hint."""
    monkeypatch.setattr(act.shutil, "which", lambda _cmd: None)
    with pytest.raises(act.ResolveError) as excinfo:
        act.resolve_cli("arduino-cli")
    message = str(excinfo.value)
    assert "arduino-cli not found" in message
    assert "cliPath" in message


def test_resolve_cli_found_on_path_returned(monkeypatch) -> None:
    """TC16 (positive): a PATH hit is returned as the resolved executable."""
    monkeypatch.setattr(act.shutil, "which", lambda _cmd: "/usr/local/bin/arduino-cli")
    assert act.resolve_cli("arduino-cli") == "/usr/local/bin/arduino-cli"


# ---------------------------------------------------------------------------
# _needs_autodetect -- pure.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("configured", [None, "", "   ", "auto"])
def test_needs_autodetect_true_for_blank_or_auto(configured) -> None:
    """BONUS: None/blank/whitespace/'auto' all require auto-detection."""
    assert act._needs_autodetect(configured) is True


def test_needs_autodetect_false_for_explicit_port() -> None:
    """BONUS: an explicit port needs no detection."""
    assert act._needs_autodetect("/dev/ttyACM9") is False


# ---------------------------------------------------------------------------
# Apple Silicon / Renesas Uno diagnostic hint.
# ---------------------------------------------------------------------------


def test_apple_silicon_renesas_hint_matches_bad_cpu(monkeypatch) -> None:
    """Apple Silicon + Renesas Uno + bad CPU type gets a Rosetta hint."""
    monkeypatch.setattr(act.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(act.platform, "machine", lambda: "arm64")

    hint = act._apple_silicon_renesas_hint(
        "arduino:renesas_uno:unor4wifi",
        "",
        "fork/exec ... arm-none-eabi-g++: bad CPU type in executable",
    )

    assert hint is not None
    assert "Rosetta" in hint
    assert "Renesas Uno" in hint


def test_apple_silicon_renesas_hint_ignores_other_platforms(monkeypatch) -> None:
    """Non-macOS hosts should not get the Rosetta-specific hint."""
    monkeypatch.setattr(act.platform, "system", lambda: "Linux")
    monkeypatch.setattr(act.platform, "machine", lambda: "x86_64")

    assert act._apple_silicon_renesas_hint(
        "arduino:renesas_uno:unor4wifi",
        "",
        "bad CPU type in executable",
    ) is None


# ---------------------------------------------------------------------------
# dispatch -- the only function that spawns subprocesses; mock them.
# ---------------------------------------------------------------------------


def _fake_cli(monkeypatch, path: str = "/fake/bin/arduino-cli") -> str:
    """Pin ``resolve_cli`` to a fake executable path so no PATH lookup happens."""
    monkeypatch.setattr(act, "resolve_cli", lambda _cli: path)
    return path


def test_dispatch_verify_runs_compile_argv(monkeypatch) -> None:
    """TC17: verify runs ``<cli> compile --fqbn <fqbn> <sketchDir>`` and returns 0."""
    fake_cli = _fake_cli(monkeypatch)
    fqbn = "arduino:renesas_uno:unor4wifi"
    sketch = "WRO2026"
    expected_sketch_dir = str(act._resolve_sketch_dir(sketch))

    calls: list[list[str]] = []

    def fake_run(args, *_args, **_kwargs):
        calls.append(list(args))
        return subprocess.CompletedProcess(args, returncode=0)

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    rc = act.dispatch(
        action="verify",
        cli="arduino-cli",
        fqbn=fqbn,
        sketch=sketch,
        port=None,
        baud=None,
    )

    assert rc == 0
    assert calls == [[fake_cli, "compile", "--fqbn", fqbn, expected_sketch_dir]]


def test_dispatch_verify_bad_cpu_prints_rosetta_hint(monkeypatch, capsys) -> None:
    """A bad-CPU compile failure on Apple Silicon prints the targeted hint."""
    _fake_cli(monkeypatch)
    fqbn = "arduino:renesas_uno:unor4wifi"

    monkeypatch.setattr(act.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(act.platform, "machine", lambda: "arm64")

    def fake_run(args, *_args, **_kwargs):
        return subprocess.CompletedProcess(
            args,
            returncode=1,
            stdout="Used platform ...\n",
            stderr="fork/exec arm-none-eabi-g++: bad CPU type in executable\n",
        )

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    rc = act.dispatch(
        action="verify",
        cli="arduino-cli",
        fqbn=fqbn,
        sketch="WRO2026",
        port=None,
        baud=None,
    )

    captured = capsys.readouterr()
    assert rc == 1
    assert "bad CPU type in executable" in captured.err
    assert "Rosetta" in captured.err


def test_dispatch_upload_auto_filters_by_fqbn_and_resolves(monkeypatch) -> None:
    """TC20: upload with port='auto' runs an FQBN-filtered ``board list`` first.

    The board-list argv must carry ``--fqbn`` and the configured fqbn (proving
    unrelated devices are filtered out by arduino-cli before the resolver runs),
    and the upload must target the single detected /dev/ttyACM0.
    """
    fake_cli = _fake_cli(monkeypatch)
    fqbn = "arduino:renesas_uno:unor4wifi"
    sketch = "WRO2026"
    expected_sketch_dir = str(act._resolve_sketch_dir(sketch))

    calls: list[list[str]] = []

    def fake_run(args, *_args, **kwargs):
        calls.append(list(args))
        # The board-list invocation is the JSON-capturing call.
        if "board" in args and "list" in args:
            return subprocess.CompletedProcess(
                args,
                returncode=0,
                stdout='{"detected_ports":[{"port":{"address":"/dev/ttyACM0"}}]}',
            )
        # The actual upload (streamed I/O) returns success.
        return subprocess.CompletedProcess(args, returncode=0)

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    rc = act.dispatch(
        action="upload",
        cli="arduino-cli",
        fqbn=fqbn,
        sketch=sketch,
        port="auto",
        baud=None,
    )

    assert rc == 0
    assert len(calls) == 2

    board_list_call, upload_call = calls
    # The board-list call is FQBN-filtered: unrelated devices never reach the resolver.
    assert "board" in board_list_call
    assert "list" in board_list_call
    assert "--fqbn" in board_list_call
    assert fqbn in board_list_call

    # The R4 (the only fqbn match) is resolved and handed to upload.
    assert upload_call == [
        fake_cli,
        "upload",
        "--fqbn",
        fqbn,
        "--port",
        "/dev/ttyACM0",
        expected_sketch_dir,
    ]


def test_dispatch_upload_explicit_port_skips_board_list(monkeypatch) -> None:
    """BONUS: an explicit port makes upload skip ``board list`` entirely (lazy detect)."""
    fake_cli = _fake_cli(monkeypatch)
    fqbn = "arduino:renesas_uno:unor4wifi"
    sketch = "WRO2026"
    expected_sketch_dir = str(act._resolve_sketch_dir(sketch))

    calls: list[list[str]] = []

    def fake_run(args, *_args, **_kwargs):
        calls.append(list(args))
        return subprocess.CompletedProcess(args, returncode=0)

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    rc = act.dispatch(
        action="upload",
        cli="arduino-cli",
        fqbn=fqbn,
        sketch=sketch,
        port="/dev/ttyACM9",
        baud=None,
    )

    assert rc == 0
    # No board-list invocation at all -- lazy detection is bypassed for explicit ports.
    assert all(not ("board" in call and "list" in call) for call in calls)
    assert calls == [
        [
            fake_cli,
            "upload",
            "--fqbn",
            fqbn,
            "--port",
            "/dev/ttyACM9",
            expected_sketch_dir,
        ]
    ]


def test_dispatch_monitor_runs_monitor_argv(monkeypatch) -> None:
    """Monitor resolves the port + baud and builds the monitor argv.

    Covers the only ``baudrate=<baud>`` f-string and the wiring of
    ``resolve_baud`` into ``dispatch``.
    """
    fake_cli = _fake_cli(monkeypatch)
    fqbn = "arduino:renesas_uno:unor4wifi"

    calls: list[list[str]] = []

    def fake_run(args, *_args, **_kwargs):
        calls.append(list(args))
        if "board" in args and "list" in args:
            return subprocess.CompletedProcess(
                args,
                returncode=0,
                stdout='{"detected_ports":[{"port":{"address":"/dev/ttyACM0"}}]}',
            )
        return subprocess.CompletedProcess(args, returncode=0)

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    rc = act.dispatch(
        action="monitor",
        cli="arduino-cli",
        fqbn=fqbn,
        sketch="WRO2026",
        port="auto",
        baud="115200",
    )

    assert rc == 0
    monitor_call = calls[-1]
    assert monitor_call == [
        fake_cli,
        "monitor",
        "--port",
        "/dev/ttyACM0",
        "--config",
        "baudrate=115200",
    ]


def test_dispatch_list_boards_runs_board_list_argv(monkeypatch) -> None:
    """list-boards runs ``<cli> board list`` and returns its exit code."""
    fake_cli = _fake_cli(monkeypatch)

    calls: list[list[str]] = []

    def fake_run(args, *_args, **_kwargs):
        calls.append(list(args))
        return subprocess.CompletedProcess(args, returncode=0)

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    rc = act.dispatch(
        action="list-boards",
        cli="arduino-cli",
        fqbn="arduino:renesas_uno:unor4wifi",
        sketch="WRO2026",
        port=None,
        baud=None,
    )

    assert rc == 0
    assert calls == [[fake_cli, "board", "list"]]


def test_dispatch_upload_board_list_failure_raises(monkeypatch) -> None:
    """A non-zero ``board list`` during auto-detect raises ResolveError."""
    _fake_cli(monkeypatch)

    def fake_run(args, *_args, **_kwargs):
        if "board" in args and "list" in args:
            return subprocess.CompletedProcess(
                args, returncode=1, stdout="", stderr="boom"
            )
        return subprocess.CompletedProcess(args, returncode=0)

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    with pytest.raises(act.ResolveError, match="board list failed"):
        act.dispatch(
            action="upload",
            cli="arduino-cli",
            fqbn="arduino:renesas_uno:unor4wifi",
            sketch="WRO2026",
            port="auto",
            baud=None,
        )


# ---------------------------------------------------------------------------
# upload-monitor -- kill any open monitor, flash, reopen the monitor.
# ---------------------------------------------------------------------------


def test_dispatch_upload_monitor_kills_flashes_then_monitors(monkeypatch) -> None:
    """upload-monitor sequences kill -> detect -> upload -> re-detect -> monitor."""
    fake_cli = _fake_cli(monkeypatch)
    fqbn = "arduino:renesas_uno:unor4wifi"
    sketch = "WRO2026"
    expected_sketch_dir = str(act._resolve_sketch_dir(sketch))

    monkeypatch.setattr(act.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(act.time, "sleep", lambda _s: None)

    calls: list[list[str]] = []

    def fake_run(args, *_args, **_kwargs):
        calls.append(list(args))
        if "board" in args and "list" in args:
            return subprocess.CompletedProcess(
                args,
                returncode=0,
                stdout='{"detected_ports":[{"port":{"address":"/dev/ttyACM0"}}]}',
            )
        return subprocess.CompletedProcess(args, returncode=0)

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    rc = act.dispatch(
        action="upload-monitor",
        cli="arduino-cli",
        fqbn=fqbn,
        sketch=sketch,
        port="auto",
        baud="115200",
    )

    assert rc == 0
    # kill -> board list -> upload -> board list (re-detect) -> monitor.
    assert calls[0] == ["pkill", "-f", "arduino-cli monitor"]
    verbs = [call[1] if call[0] == fake_cli else call[0] for call in calls]
    assert verbs == ["pkill", "board", "upload", "board", "monitor"]
    assert calls[2] == [
        fake_cli,
        "upload",
        "--fqbn",
        fqbn,
        "--port",
        "/dev/ttyACM0",
        expected_sketch_dir,
    ]
    assert calls[-1] == [
        fake_cli,
        "monitor",
        "--port",
        "/dev/ttyACM0",
        "--config",
        "baudrate=115200",
    ]


def test_dispatch_upload_monitor_failed_flash_skips_monitor(monkeypatch) -> None:
    """A non-zero upload exit is propagated and no monitor is launched."""
    fake_cli = _fake_cli(monkeypatch)

    monkeypatch.setattr(act.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(act.time, "sleep", lambda _s: None)

    calls: list[list[str]] = []

    def fake_run(args, *_args, **_kwargs):
        calls.append(list(args))
        if "board" in args and "list" in args:
            return subprocess.CompletedProcess(
                args,
                returncode=0,
                stdout='{"detected_ports":[{"port":{"address":"/dev/ttyACM0"}}]}',
            )
        if "upload" in args:
            return subprocess.CompletedProcess(
                args, returncode=1, stdout="", stderr="flash failed"
            )
        return subprocess.CompletedProcess(args, returncode=0)

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    rc = act.dispatch(
        action="upload-monitor",
        cli="arduino-cli",
        fqbn="arduino:renesas_uno:unor4wifi",
        sketch="WRO2026",
        port="auto",
        baud="115200",
    )

    assert rc == 1
    assert all("monitor" not in call for call in calls if call[0] == fake_cli)


def test_dispatch_upload_monitor_missing_baud_fails_before_flash(monkeypatch) -> None:
    """A blank baud aborts BEFORE any subprocess runs -- nothing gets flashed."""
    _fake_cli(monkeypatch)

    calls: list[list[str]] = []

    def fake_run(args, *_args, **_kwargs):
        calls.append(list(args))
        return subprocess.CompletedProcess(args, returncode=0)

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    with pytest.raises(act.ResolveError, match="baud"):
        act.dispatch(
            action="upload-monitor",
            cli="arduino-cli",
            fqbn="arduino:renesas_uno:unor4wifi",
            sketch="WRO2026",
            port="auto",
            baud="  ",
        )

    assert calls == []


def test_kill_monitor_windows_uses_powershell_stop_process(monkeypatch) -> None:
    """On Windows the kill goes through PowerShell Stop-Process (no pkill)."""
    monkeypatch.setattr(act.platform, "system", lambda: "Windows")

    calls: list[list[str]] = []

    def fake_run(args, *_args, **_kwargs):
        calls.append(list(args))
        # A stopped ProcessId on stdout means "killed at least one".
        return subprocess.CompletedProcess(args, returncode=0, stdout="4242\n")

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    killed = act._kill_monitor_processes()

    assert killed is True
    assert len(calls) == 1
    assert calls[0][0] == "powershell"
    script = calls[0][-1]
    assert "Stop-Process" in script
    assert "arduino-cli*" in script


def test_kill_monitor_posix_no_match_returns_false(monkeypatch) -> None:
    """pkill exit 1 (nothing matched) reads as 'no monitor was running'."""
    monkeypatch.setattr(act.platform, "system", lambda: "Darwin")

    def fake_run(args, *_args, **_kwargs):
        return subprocess.CompletedProcess(args, returncode=1)

    monkeypatch.setattr(act.subprocess, "run", fake_run)

    assert act._kill_monitor_processes() is False


def test_resolve_port_after_reset_retries_until_board_returns(monkeypatch) -> None:
    """Empty detections right after a flash are polled until re-enumeration."""
    attempts = iter([[], [], [{"port": {"address": "/dev/ttyACM0"}}]])
    monkeypatch.setattr(
        act, "_list_detected_ports", lambda _cli, _fqbn: next(attempts)
    )

    sleeps: list[float] = []
    monkeypatch.setattr(act.time, "sleep", lambda s: sleeps.append(s))

    port = act._resolve_port_after_reset("/fake/bin/arduino-cli", "fqbn", "auto")

    assert port == "/dev/ttyACM0"
    assert len(sleeps) == 2  # one wait between each of the three attempts


def test_resolve_port_after_reset_exhausted_raises(monkeypatch) -> None:
    """A board that never re-enumerates surfaces the final ResolveError."""
    monkeypatch.setattr(act, "_list_detected_ports", lambda _cli, _fqbn: [])
    monkeypatch.setattr(act.time, "sleep", lambda _s: None)

    with pytest.raises(act.ResolveError, match="No Arduino board detected"):
        act._resolve_port_after_reset("/fake/bin/arduino-cli", "fqbn", "auto")


def test_resolve_port_after_reset_explicit_port_returns_immediately(monkeypatch) -> None:
    """An explicit configured port skips detection entirely (no board list)."""

    def _boom(_cli, _fqbn):
        raise AssertionError("board list must not run for an explicit port")

    monkeypatch.setattr(act, "_list_detected_ports", _boom)
    monkeypatch.setattr(act.time, "sleep", lambda _s: None)

    port = act._resolve_port_after_reset("/fake/bin/arduino-cli", "fqbn", "COM5")

    assert port == "COM5"
