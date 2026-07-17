#!/usr/bin/env python3
"""Build, upload, and monitor the MAIN WRO2026 sketch.

    uv run tools/flash/flash_wro2026.py [--build-only] [--port auto] [--baud 115200]
        [--no-monitor] [--log path/to/session.csv]

Unlike the ``inoTest/`` diagnostic wrappers, this driver compiles the real
``WRO2026/`` sketch as-is -- its ``src/<domain>`` sources are part of the
sketch itself, so the compile needs no ``--library`` flags at all. It is
self-contained: paths resolve from this file's own location, so it works
from any current directory and needs nothing from ``inoTest/build_common.py``.

Design, ported and adapted from the ``inoTest/build_common.py`` driver:
  - ``--port auto`` filters ``arduino-cli board list --json --fqbn <fqbn>``
    down to the single matching board, erroring clearly on zero or several.
  - After upload, the UNO R4 drops off USB on reset and re-enumerates under
    a (possibly new, especially on macOS) port name; the monitor waits for
    that re-enumeration instead of opening a dead handle.
  - Any stale ``arduino-cli monitor`` process is closed before upload --
    an open monitor holds the serial port and the upload would fail.

Extras this driver adds on top of that shape:
  - ``--no-monitor`` uploads without attaching a monitor afterward.
  - ``--log <path>`` tees the monitor stream to a file line-by-line, so a
    one-shot hardware run's Serial output survives a scrollback limit.

No new dependency is introduced: the monitor shells out to ``arduino-cli
monitor`` (per project convention -- see ``pyproject.toml``, which has no
serial library) rather than adding a ``pyserial`` dependency.
"""

from __future__ import annotations

import argparse
import json
import ntpath
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

DEFAULT_CLI = "arduino-cli"
DEFAULT_FQBN = "arduino:renesas_uno:unor4wifi"
DEFAULT_BAUD = 115200

# The UNO R4 drops off USB on reset and re-enumerates a few seconds later
# (macOS may assign the /dev/cu.usbmodem* node a new suffix); poll instead
# of opening the monitor on a dead handle.
REENUMERATION_TIMEOUT_SECONDS = 20.0
REENUMERATION_POLL_SECONDS = 0.5

# A stale monitor from a previous session holds the port and would fail the
# next upload; give the OS a moment to release it after closing one.
MONITOR_CLOSE_SETTLE_SECONDS = 0.25

# Grace period before escalating a Ctrl-C'd monitor child from terminate to kill.
MONITOR_TERMINATE_TIMEOUT_SECONDS = 5.0


class BoardListError(RuntimeError):
    """Raised when ``arduino-cli board list`` cannot provide a usable board."""


@dataclass(frozen=True)
class BoardPort:
    """A serial port address and the hardware identity reported for it."""

    address: str
    hardware_id: str | None = None


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command-line overrides for the compiler, board, and monitor.

    :param argv: explicit argv (excluding the program name); ``None`` reads
        :data:`sys.argv` via argparse's default.
    :return: the parsed namespace.
    """
    parser = argparse.ArgumentParser(
        description="Build, upload, and monitor the main WRO2026 sketch."
    )
    parser.add_argument(
        "--build-only",
        action="store_true",
        help="compile without detecting a board, uploading, or opening a monitor",
    )
    parser.add_argument(
        "--port",
        default="auto",
        help="serial port to upload to, or 'auto' to detect it (default: auto)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"serial monitor baud rate (default: {DEFAULT_BAUD})",
    )
    parser.add_argument(
        "--cli",
        default=DEFAULT_CLI,
        help="arduino-cli executable or path (default: arduino-cli)",
    )
    parser.add_argument(
        "--fqbn",
        default=DEFAULT_FQBN,
        help=f"Arduino board FQBN (default: {DEFAULT_FQBN})",
    )
    parser.add_argument(
        "--no-monitor",
        action="store_true",
        help="upload but do not attach a serial monitor afterward",
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=None,
        help="tee the monitor stream to this file, line by line",
    )
    args = parser.parse_args(argv)

    if args.baud <= 0:
        parser.error("--baud must be greater than zero")
    if args.build_only and args.log is not None:
        print(
            "warning: --log is unused with --build-only (no monitor runs)",
            file=sys.stderr,
        )
    if args.no_monitor and args.log is not None:
        print(
            "warning: --log is unused with --no-monitor (no monitor runs)",
            file=sys.stderr,
        )
    return args


def _sketch_dir() -> Path:
    """Return the ``WRO2026/`` sketch directory, resolved from this file's location."""
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "WRO2026"


def _resolve_cli(cli: str) -> str | None:
    """Resolve the arduino-cli executable to an existing file or a PATH entry.

    :param cli: the configured cli path or bare command name.
    :return: the resolved executable path, or ``None`` when it cannot be found.
    """
    candidate = Path(cli)
    if candidate.is_file():
        return str(candidate)
    return shutil.which(cli)


def _board_list(cli: str, fqbn: str) -> list[dict[str, Any]]:
    """Return the FQBN-filtered ``detected_ports`` entries from ``board list``.

    :param cli: the resolved arduino-cli executable path.
    :param fqbn: fully-qualified board name to filter by.
    :return: the raw ``detected_ports`` entries (each a dict).
    :raises BoardListError: on a nonzero exit or malformed/unexpected JSON.
    """
    result = subprocess.run(
        [cli, "board", "list", "--json", "--fqbn", fqbn],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or "no diagnostic output"
        raise BoardListError(
            f"arduino-cli board list failed with exit code {result.returncode}: {detail}"
        )

    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise BoardListError(f"arduino-cli board list returned malformed JSON: {error}") from error

    if not isinstance(payload, dict) or not isinstance(payload.get("detected_ports"), list):
        raise BoardListError(
            "arduino-cli board list returned unexpected JSON: "
            "expected an object containing a detected_ports array"
        )
    entries = payload["detected_ports"]
    if not all(isinstance(entry, dict) for entry in entries):
        raise BoardListError(
            "arduino-cli board list returned unexpected JSON: detected_ports contains a non-object"
        )
    return entries


def _macos_twin_key(address: str) -> str:
    """Collapse matching macOS ``/dev/cu.*`` and ``/dev/tty.*`` device names.

    A USB-serial adapter exposes both a callout (``cu.``) and a dial-in
    (``tty.``) node for the same physical device; addresses on other
    platforms (e.g. Windows ``COM3``) pass through unchanged.
    """
    for prefix in ("/dev/cu.", "/dev/tty."):
        if address.startswith(prefix):
            return address.removeprefix(prefix)
    return address


def _prefer_port(current: BoardPort, candidate: BoardPort) -> BoardPort:
    """Prefer a macOS callout (``cu.``) device over its matching ``tty.`` twin."""
    if candidate.address.startswith("/dev/cu.") and current.address.startswith("/dev/tty."):
        return candidate
    return current


def _matching_ports(cli: str, fqbn: str) -> list[BoardPort]:
    """Find and deduplicate serial ports whose matching boards include ``fqbn``.

    :param cli: the resolved arduino-cli executable path.
    :param fqbn: fully-qualified board name to filter by.
    :return: the deduplicated list of candidate ports.
    :raises BoardListError: propagated from :func:`_board_list`, or raised
        directly when a matching board is missing a port address.
    """
    ports: dict[str, BoardPort] = {}
    for entry in _board_list(cli, fqbn):
        matching_boards = entry.get("matching_boards", [])
        port = entry.get("port")
        if not isinstance(matching_boards, list) or not isinstance(port, dict):
            continue
        if not any(
            isinstance(board, dict) and board.get("fqbn") == fqbn for board in matching_boards
        ):
            continue
        address = port.get("address")
        if not isinstance(address, str) or not address:
            raise BoardListError(
                "arduino-cli board list returned a matching board without a valid port address"
            )
        hardware_id = port.get("hardware_id")
        candidate = BoardPort(
            address=address,
            hardware_id=hardware_id if isinstance(hardware_id, str) else None,
        )
        key = _macos_twin_key(address)
        ports[key] = _prefer_port(ports[key], candidate) if key in ports else candidate
    return list(ports.values())


def _detect_one_board(cli: str, fqbn: str) -> BoardPort:
    """Return the only connected board matching ``fqbn``.

    :raises BoardListError: with an actionable message when zero or several
        boards match.
    """
    ports = _matching_ports(cli, fqbn)
    if not ports:
        raise BoardListError(
            f"no connected board matches {fqbn!r}; connect the robot or pass --port explicitly"
        )
    if len(ports) > 1:
        addresses = ", ".join(sorted(port.address for port in ports))
        raise BoardListError(
            f"multiple connected boards match {fqbn!r}: {addresses}; choose one with --port"
        )
    return ports[0]


def _is_cli_monitor_command(command: str, cli: str) -> bool:
    """Return whether a process command line is the configured CLI running monitor."""
    try:
        tokens = shlex.split(command, posix=os.name != "nt")
    except ValueError:
        return False
    if not tokens:
        return False
    executable = tokens[0].strip('"')
    executable_name = ntpath.basename(executable).lower()
    cli_name = ntpath.basename(cli).lower()
    return executable_name == cli_name and "monitor" in tokens[1:]


def _running_processes() -> list[tuple[int, str]]:
    """List (pid, command line) pairs using built-in operating-system tools.

    Uses PowerShell's CIM process listing on Windows and ``ps`` elsewhere;
    returns an empty list (best-effort, never raises) if neither is available.
    """
    if os.name == "nt":
        powershell = shutil.which("powershell") or shutil.which("pwsh")
        if powershell is None:
            return []
        script = (
            "Get-CimInstance Win32_Process | Where-Object {$_.CommandLine} | "
            "ForEach-Object { \"{0}`t{1}\" -f $_.ProcessId,$_.CommandLine }"
        )
        command = [powershell, "-NoProfile", "-Command", script]
    else:
        command = ["ps", "-axo", "pid=,command="]

    result = subprocess.run(command, check=False, capture_output=True, text=True)
    if result.returncode != 0:
        return []
    processes: list[tuple[int, str]] = []
    for line in result.stdout.splitlines():
        fields = line.strip().split(maxsplit=1)
        if len(fields) == 2 and fields[0].isdigit():
            processes.append((int(fields[0]), fields[1]))
    return processes


def _close_existing_monitors(cli: str) -> None:
    """Best-effort close of any running ``arduino-cli monitor`` processes.

    An open monitor holds the serial port and would fail the next upload.
    Failures to signal a process are swallowed -- no monitor running is the
    normal case, not an error.
    """
    closed = 0
    for pid, command in _running_processes():
        if pid == os.getpid() or not _is_cli_monitor_command(command, cli):
            continue
        try:
            if os.name == "nt":
                subprocess.run(
                    ["taskkill", "/PID", str(pid), "/F"],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            else:
                os.kill(pid, signal.SIGTERM)
            closed += 1
        except (OSError, subprocess.SubprocessError):
            continue
    if closed:
        print(f"Closed {closed} existing arduino-cli monitor process(es).")
        time.sleep(MONITOR_CLOSE_SETTLE_SECONDS)


def _wait_for_board(cli: str, fqbn: str, uploaded: BoardPort) -> BoardPort:
    """Poll until the just-uploaded board has re-enumerated after reset.

    Prefers matching by ``hardware_id`` (stable across a port-name change);
    falls back to the same address (collapsing macOS cu./tty. twins), then
    to "exactly one board detected" if neither identifies the original port.

    :raises BoardListError: when the board never re-enumerates in time.
    """
    deadline = time.monotonic() + REENUMERATION_TIMEOUT_SECONDS
    last_error: BoardListError | None = None
    while time.monotonic() < deadline:
        try:
            ports = _matching_ports(cli, fqbn)
            last_error = None
        except BoardListError as error:
            last_error = error
            time.sleep(REENUMERATION_POLL_SECONDS)
            continue

        if uploaded.hardware_id:
            same_hardware = [port for port in ports if port.hardware_id == uploaded.hardware_id]
            if len(same_hardware) == 1:
                return same_hardware[0]
        same_address = [
            port for port in ports if _macos_twin_key(port.address) == _macos_twin_key(uploaded.address)
        ]
        if len(same_address) == 1:
            return same_address[0]
        if len(ports) == 1:
            return ports[0]
        time.sleep(REENUMERATION_POLL_SECONDS)

    detail = f" Last board-list error: {last_error}" if last_error else ""
    raise BoardListError(
        f"board did not re-enumerate within {REENUMERATION_TIMEOUT_SECONDS:g} seconds after upload."
        f"{detail}"
    )


def _compile_command(cli: str, fqbn: str, upload_port: str | None) -> list[str]:
    """Build the ``arduino-cli compile`` command for the WRO2026 sketch.

    No ``--library`` flags: the sketch's ``src/<domain>`` sources compile as
    part of the sketch itself (unlike the standalone ``inoTest/`` diagnostics).

    :param upload_port: when given, appends ``--upload --port <port>`` so the
        same command both compiles and flashes; ``None`` compiles only.
    """
    command = [cli, "compile", "--fqbn", fqbn]
    if upload_port is not None:
        command.extend(("--upload", "--port", upload_port))
    command.append(str(_sketch_dir()))
    return command


def _run_monitor(cli: str, port: str, baud: int, log_path: Path | None) -> int:
    """Stream ``arduino-cli monitor`` output to stdout, and optionally to a log file.

    Reads the child's combined stdout/stderr line by line so each line reaches
    the terminal and the log file as it arrives, rather than only at exit --
    important for a one-shot hardware run whose output must not be lost.

    :return: the child process exit code, or 130 if interrupted with Ctrl-C.
    """
    command = [cli, "monitor", "--port", port, "--config", f"baudrate={baud}"]
    log_file = None
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = log_path.open("w", encoding="utf-8", errors="replace")
        print(f"Logging monitor output to {log_path}")

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    try:
        assert process.stdout is not None
        for line in process.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            if log_file is not None:
                log_file.write(line)
                log_file.flush()
        return process.wait()
    except KeyboardInterrupt:
        process.terminate()
        try:
            process.wait(timeout=MONITOR_TERMINATE_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        print("\nMonitor stopped.")
        return 130
    finally:
        if log_file is not None:
            log_file.close()


def main(argv: list[str] | None = None) -> int:
    """Compile, optionally upload, then monitor; return the resulting exit code."""
    args = parse_args(argv)

    cli = _resolve_cli(args.cli)
    if cli is None:
        print(
            f"error: Arduino CLI executable not found: {args.cli!r}\n"
            "Install arduino-cli or pass its path with --cli.",
            file=sys.stderr,
        )
        return 127

    if args.build_only:
        return subprocess.run(_compile_command(cli, args.fqbn, None), check=False).returncode

    try:
        board = (
            _detect_one_board(cli, args.fqbn)
            if args.port == "auto"
            else BoardPort(address=args.port)
        )
    except BoardListError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    _close_existing_monitors(cli)
    print(f"Uploading to {board.address}...")
    result = subprocess.run(_compile_command(cli, args.fqbn, board.address), check=False)
    if result.returncode != 0:
        return result.returncode
    print("Upload complete.")

    if args.no_monitor:
        return 0

    print("Waiting for the board to re-enumerate...")
    try:
        monitor_port = _wait_for_board(cli, args.fqbn, board)
    except BoardListError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    print(f"Opening {monitor_port.address} at {args.baud} baud (Ctrl-C to stop)...")
    return _run_monitor(cli, monitor_port.address, args.baud, args.log)


if __name__ == "__main__":
    sys.exit(main())
