#!/usr/bin/env python3
"""Shared build/upload/monitor driver for the ``inoTest/`` diagnostic sketches.

Each sketch keeps its own thin ``build.py`` that supplies a folder name and the set of
``WRO2026/src/<domain>`` libraries it needs, then calls :func:`main` here. After the
``src/<domain>/`` reorg, ``--library WRO2026`` reaches only root files such as
``Config.h``, so every used domain needs its own ``--library`` flag; that assembly lives
in :func:`_compile_command`.

Paths resolve from this file's own location, so the wrappers work on Windows and macOS
and from any current directory.
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
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DEFAULT_FQBN = "arduino:renesas_uno:unor4wifi"
REENUMERATION_TIMEOUT_SECONDS = 20.0
REENUMERATION_POLL_SECONDS = 0.5


class BoardListError(RuntimeError):
    """Raised when ``arduino-cli board list`` cannot provide valid JSON."""


@dataclass(frozen=True)
class BoardPort:
    """A serial port and the hardware identity reported for it."""

    address: str
    hardware_id: str | None = None


def parse_args(sketch_label: str) -> argparse.Namespace:
    """Parse command-line overrides for the compiler, board, and monitor."""
    parser = argparse.ArgumentParser(
        description=f"Build, upload, and monitor {sketch_label}."
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
        default=115200,
        help="serial monitor baud rate (default: 115200)",
    )
    parser.add_argument(
        "--cli",
        default="arduino-cli",
        help="arduino-cli executable or path (default: arduino-cli)",
    )
    parser.add_argument(
        "--fqbn",
        default=DEFAULT_FQBN,
        help=f"Arduino board FQBN (default: {DEFAULT_FQBN})",
    )
    parser.add_argument(
        "--define",
        action="append",
        default=[],
        metavar="NAME[=VALUE]",
        help=(
            "extra preprocessor define passed to the compile, repeatable "
            "(e.g. --define TURN_TRACE). Lets a bench run toggle a compile-time "
            "diagnostic without editing firmware source."
        ),
    )
    args = parser.parse_args()
    if args.baud <= 0:
        parser.error("--baud must be greater than zero")
    return args


def _board_list(cli: str, fqbn: str) -> list[dict[str, Any]]:
    """Return validated entries matching ``fqbn`` from ``arduino-cli board list``."""
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
    """Collapse matching macOS ``/dev/cu.*`` and ``/dev/tty.*`` device names."""
    for prefix in ("/dev/cu.", "/dev/tty."):
        if address.startswith(prefix):
            return address.removeprefix(prefix)
    return address


def _prefer_port(current: BoardPort, candidate: BoardPort) -> BoardPort:
    """Prefer a macOS callout device over its matching tty device."""
    if candidate.address.startswith("/dev/cu.") and current.address.startswith("/dev/tty."):
        return candidate
    return current


def _matching_ports(cli: str, fqbn: str) -> list[BoardPort]:
    """Find and deduplicate serial ports whose matching boards include ``fqbn``."""
    ports: dict[str, BoardPort] = {}
    for entry in _board_list(cli, fqbn):
        matching_boards = entry.get("matching_boards", [])
        port = entry.get("port")
        if not isinstance(matching_boards, list) or not isinstance(port, dict):
            continue
        if not any(
            isinstance(board, dict) and board.get("fqbn") == fqbn
            for board in matching_boards
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
    """Return the only board matching ``fqbn``, with actionable errors otherwise."""
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
    """Return whether a process command is the configured Arduino CLI running monitor."""
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
    """List process IDs and command lines using built-in operating-system tools."""
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
    """Best-effort close only existing ``arduino-cli monitor`` processes."""
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
        time.sleep(0.25)


def _wait_for_board(cli: str, fqbn: str, uploaded: BoardPort) -> BoardPort:
    """Poll until the uploaded board has re-enumerated after reset."""
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
            port
            for port in ports
            if _macos_twin_key(port.address) == _macos_twin_key(uploaded.address)
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


def _compile_command(
    cli: str,
    fqbn: str,
    upload_port: str | None,
    sketch_name: str,
    source_domains: Sequence[str],
    defines: Sequence[str] = (),
    *,
    sketch_dir: Path | None = None,
    includes: Sequence[str] = (),
) -> list[str]:
    """Build the single Arduino CLI compile command.

    Two include regimes:

    * The inoTest default — ``--library WRO2026`` reaches only the firmware root
      (``Config.h``, ``Tasks.cpp``); each ``src/<domain>`` the sketch pulls in needs its
      own ``--library`` flag on top.
    * The ``includes`` opt-in (vision_bench) — a sketch that reaches its firmware headers by
      path-qualified quoted include (``config/ConfigVision.h``, ``src/perception/...``)
      cannot use ``--library``: that flag's include scan only recognises bare basenames and
      silently drops the path. Such a sketch passes a raw ``-I`` root here instead and no
      ``--library`` flag is emitted.

    ``includes`` (as ``-I``) and ``defines`` (as ``-D``) are merged into ONE
    ``compiler.cpp.extra_flags`` build property: Arduino CLI keeps only the last value for a
    repeated property key, so emitting them separately would silently drop all but the last.

    ``sketch_dir`` overrides the default ``inoTest/<sketch_name>`` sketch path (used by
    sketches that live elsewhere, e.g. ``tools/vision_bench``).
    """
    repo_root = Path(__file__).resolve().parents[1]
    sketch = sketch_dir if sketch_dir is not None else repo_root / "inoTest" / sketch_name
    firmware = repo_root / "WRO2026"
    command = [cli, "compile", "--fqbn", fqbn]
    if upload_port is not None:
        command.extend(("--upload", "--port", upload_port))
    command.append(str(sketch))
    if includes:
        extra = [f"-I{include}" for include in includes]
    else:
        command.extend(("--library", str(firmware)))
        for domain in source_domains:
            command.extend(("--library", str(firmware / "src" / domain)))
        extra = []
    extra.extend(f"-D{define}" for define in defines)
    if extra:
        command.extend(("--build-property", f"compiler.cpp.extra_flags={' '.join(extra)}"))
    return command


def main(
    sketch_name: str,
    source_domains: Sequence[str] = (),
    *,
    sketch_dir: Path | None = None,
    includes: Sequence[str] = (),
    location: str | None = None,
) -> int:
    """Compile, optionally upload, then monitor; return the child command exit status.

    ``sketch_dir`` / ``includes`` let a sketch outside ``inoTest/`` (e.g. ``tools/vision_bench``)
    reuse this driver with a raw ``-I`` include instead of ``--library`` — see
    :func:`_compile_command`. ``location`` overrides the ``inoTest/<name>`` label in ``--help``.
    """
    args = parse_args(location or f"inoTest/{sketch_name}")
    cli = shutil.which(args.cli)
    if cli is None:
        print(
            f"error: Arduino CLI executable not found: {args.cli!r}\n"
            "Install arduino-cli or pass its path with --cli.",
            file=sys.stderr,
        )
        return 127

    if args.build_only:
        return subprocess.run(
            _compile_command(
                cli,
                args.fqbn,
                None,
                sketch_name,
                source_domains,
                args.define,
                sketch_dir=sketch_dir,
                includes=includes,
            ),
            check=False,
        ).returncode

    try:
        board = (
            _detect_one_board(cli, args.fqbn)
            if args.port == "auto"
            else BoardPort(address=args.port)
        )
    except BoardListError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    print(f"Uploading to {board.address}...")
    _close_existing_monitors(cli)
    result = subprocess.run(
        _compile_command(
            cli,
            args.fqbn,
            board.address,
            sketch_name,
            source_domains,
            args.define,
            sketch_dir=sketch_dir,
            includes=includes,
        ),
        check=False,
    )
    if result.returncode != 0:
        return result.returncode

    print("Upload complete; waiting for the board to re-enumerate...")
    try:
        monitor_port = _wait_for_board(cli, args.fqbn, board)
    except BoardListError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    print(f"Opening {monitor_port.address} at {args.baud} baud (Ctrl-C to stop)...")
    monitor_command = [
        cli,
        "monitor",
        "--port",
        monitor_port.address,
        "--config",
        f"baudrate={args.baud}",
    ]
    try:
        return subprocess.run(monitor_command, check=False).returncode
    except KeyboardInterrupt:
        return 130
