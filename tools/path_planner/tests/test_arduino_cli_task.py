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
