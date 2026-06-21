#!/usr/bin/env python3
"""Cross-platform driver for the VS Code arduino-cli tasks.

This is the cross-platform Python port of the former PowerShell task
wrapper. It is invoked from the workspace's VS Code tasks via::

    uv run python tools/arduino_cli_task.py <action> \
        --cli <cliPath> --fqbn <fqbn> --sketch <sketchPath> \
        [--port <port>] [--baud <baud>]

``<action>`` is one of ``verify``, ``upload``, ``list-boards``,
``monitor``. The behaviour mirrors the PowerShell script exactly:

  verify       <cli> compile --fqbn <fqbn> <sketchDir>
  upload       resolve the serial port, then
               <cli> upload --fqbn <fqbn> --port <port> <sketchDir>
  list-boards  <cli> board list
  monitor      resolve the serial port + baud, then
               <cli> monitor --port <port> --config baudrate=<baud>

Serial-port resolution filters ``arduino-cli board list`` by the
configured FQBN so unrelated serial devices (a second Arduino, a USB
dongle, a Bluetooth ``cu.*`` node) do not trip the multiple-boards
error. On macOS a ``/dev/cu.*`` and ``/dev/tty.*`` node that share the
same ``usbmodem*`` / ``usbserial*`` suffix are collapsed to the single
``cu.*`` node before the count is checked.

Design:
  The two ``resolve_*`` helpers are PURE -- they take already-collected
  inputs and never touch the filesystem or a subprocess, so they are
  unit-testable in isolation. :func:`dispatch` is the only function that
  spawns ``arduino-cli``; it gathers the FQBN-filtered detected ports and
  feeds them to :func:`resolve_serial_port`.

Error handling:
  Resolution failures (missing arduino-cli, no/ambiguous board, no baud)
  raise :class:`ResolveError`. :func:`main` is the single boundary that
  catches it, prints the message to stderr, and returns a nonzero exit
  code. The exit code of a successfully-launched ``arduino-cli`` child
  is propagated verbatim as this script's exit code.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Optional

# macOS serial-device node prefixes. A USB-serial adapter exposes both a
# callout (cu.) and a dial-in (tty.) node for the same physical device;
# the part after the prefix (e.g. "usbmodem1101") is identical for a pair.
_MACOS_CU_PREFIX = "/dev/cu."
_MACOS_TTY_PREFIX = "/dev/tty."


class ResolveError(Exception):
    """Raised when a required value (cli, port, baud) cannot be resolved.

    Carries a user-facing message identical to the corresponding
    ``throw`` in the PowerShell reference script. Caught once in
    :func:`main`, which prints the message to stderr and returns a
    nonzero exit code.
    """


def resolve_cli(cli: str) -> str:
    """Resolve the arduino-cli executable.

    :param cli: the configured cli path or bare command name. If it names
        an existing file it is used as-is; otherwise it is looked up on
        ``PATH`` via :func:`shutil.which`.
    :return: the resolved absolute (or PATH-located) executable path.
    :raises ResolveError: when neither an existing file nor a PATH entry
        matches ``cli``.
    """
    candidate = Path(cli)
    if candidate.is_file():
        return str(candidate)

    found = shutil.which(cli)
    if found:
        return found

    raise ResolveError(
        f"arduino-cli not found ('{cli}'). Install it "
        "(macOS: brew install arduino-cli; "
        "Windows: winget install ArduinoSA.CLI) "
        "or set wro2026.arduino.cliPath to the binary path."
    )


def _macos_suffix(address: str) -> Optional[str]:
    """Return the device suffix for a macOS cu./tty. node, else ``None``.

    The "suffix" is the part of the address after the ``/dev/cu.`` or
    ``/dev/tty.`` prefix (e.g. ``usbmodem1101``). Addresses that are not
    macOS-style callout/dial-in nodes return ``None`` and are never
    collapsed.

    :param address: a serial-port address.
    :return: the shared suffix, or ``None`` if the address is not a
        ``/dev/cu.*`` or ``/dev/tty.*`` node.
    """
    if address.startswith(_MACOS_CU_PREFIX):
        return address[len(_MACOS_CU_PREFIX):]
    if address.startswith(_MACOS_TTY_PREFIX):
        return address[len(_MACOS_TTY_PREFIX):]
    return None


def _dedupe_macos_pairs(addresses: list[str]) -> list[str]:
    """Collapse macOS cu./tty. node pairs that share a device suffix.

    For each suffix seen on a ``/dev/cu.*`` node, the matching
    ``/dev/tty.*`` node (same suffix) is dropped, preferring the
    callout (cu.) node. Two different suffixes are never collapsed, and
    addresses that are not macOS-style nodes pass through untouched.

    :param addresses: candidate serial-port addresses (any platform).
    :return: the de-duplicated address list, order-preserving.
    """
    # Suffixes for which a cu.* node exists -- their tty.* twin is redundant.
    cu_suffixes = {
        _macos_suffix(addr)
        for addr in addresses
        if addr.startswith(_MACOS_CU_PREFIX)
    }

    result: list[str] = []
    for addr in addresses:
        if addr.startswith(_MACOS_TTY_PREFIX) and _macos_suffix(addr) in cu_suffixes:
            # A cu.* twin exists; prefer it and drop this tty.* node.
            continue
        result.append(addr)
    return result


def resolve_serial_port(
    detected: list[dict], configured: Optional[str]
) -> str:
    """Resolve the serial port to use, given pre-detected ports.

    PURE: takes the already-FQBN-filtered ``detected_ports`` list (from
    ``arduino-cli board list --json --fqbn <fqbn>``) and never touches a
    subprocess.

    :param detected: the ``detected_ports`` list, each item shaped
        ``{"port": {"address": "..."}, ...}``. Items lacking a non-empty
        ``port.address`` are ignored.
    :param configured: the configured port. When set, non-blank, and not
        the sentinel ``"auto"``, it is returned verbatim (no detection).
    :return: the resolved serial-port address.
    :raises ResolveError: when no board is detected, or more than one
        distinct device remains after the macOS cu./tty. tiebreaker.
    """
    if configured is not None and configured.strip() and configured != "auto":
        return configured

    # Collect non-empty addresses, defending against malformed entries.
    addresses: list[str] = []
    for entry in detected:
        if not isinstance(entry, dict):
            continue
        port = entry.get("port")
        if not isinstance(port, dict):
            continue
        address = port.get("address")
        if isinstance(address, str) and address:
            addresses.append(address)

    # Collapse macOS cu./tty. twins (AFTER the fqbn filter, not instead
    # of it), then reduce to the distinct set for the count check.
    addresses = _dedupe_macos_pairs(addresses)
    distinct = sorted(set(addresses))

    if len(distinct) == 1:
        return distinct[0]

    if not distinct:
        raise ResolveError(
            "No Arduino board detected. "
            "Connect the board or set wro2026.arduino.port."
        )

    raise ResolveError(
        "Multiple serial devices detected: "
        f"{', '.join(distinct)}. Set wro2026.arduino.port."
    )


def resolve_baud(configured: Optional[str]) -> str:
    """Resolve the monitor baud rate.

    PURE: no subprocess, no filesystem.

    :param configured: the configured baud rate.
    :return: the baud rate verbatim when non-blank.
    :raises ResolveError: when ``configured`` is ``None`` or blank.
    """
    if configured is not None and configured.strip():
        return configured

    raise ResolveError(
        "No baud rate configured. Set wro2026.arduino.baudRate."
    )


def _resolve_sketch_dir(sketch: str) -> Path:
    """Resolve the sketch directory relative to the repo root.

    :param sketch: the sketch path. Absolute paths are used as-is;
        relative paths are joined onto the repo root
        (``<repo>/`` -- two parents up from this file).
    :return: the resolved sketch path.
    """
    repo_root = Path(__file__).resolve().parents[1]
    candidate = Path(sketch)
    if candidate.is_absolute():
        return candidate
    return (repo_root / candidate).resolve()


def _list_detected_ports(cli: str, fqbn: str) -> list[dict]:
    """Run ``arduino-cli board list`` filtered by FQBN and parse the ports.

    The ``--fqbn`` filter makes arduino-cli return only boards matching
    the configured FQBN, so unrelated serial devices are excluded before
    :func:`resolve_serial_port` ever sees them.

    :param cli: the resolved arduino-cli executable path.
    :param fqbn: the fully-qualified board name to filter by.
    :return: the ``detected_ports`` list (``[]`` when absent or null).
    :raises ResolveError: when the listing command exits nonzero.
    """
    completed = subprocess.run(
        [cli, "board", "list", "--json", "--fqbn", fqbn],
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    if completed.returncode != 0:
        message = (completed.stderr or completed.stdout or "").strip()
        raise ResolveError(
            "arduino-cli board list failed "
            f"(exit {completed.returncode})"
            + (f": {message}" if message else ".")
        )

    parsed = json.loads(completed.stdout)
    detected = parsed.get("detected_ports")
    if not isinstance(detected, list):
        return []
    return detected


def _run_cli(cli: str, args: list[str]) -> int:
    """Run an arduino-cli subcommand, streaming its I/O to this process.

    :param cli: the resolved arduino-cli executable path.
    :param args: subcommand arguments (e.g. ``["compile", ...]``).
    :return: the child process exit code.
    """
    completed = subprocess.run([cli, *args])
    return completed.returncode


def _needs_autodetect(configured: Optional[str]) -> bool:
    """Return whether the serial port must be auto-detected.

    Mirrors the configured-port guard in :func:`resolve_serial_port`: a
    blank/absent value or the ``"auto"`` sentinel means "detect"; any other
    value is an explicit port that needs no ``board list`` at all. Keeping the
    listing lazy means an explicitly-configured upload/monitor never aborts on
    a transient ``board list`` failure (matching the PowerShell reference).

    :param configured: the configured port value.
    :return: ``True`` when auto-detection (and thus a ``board list``) is needed.
    """
    return configured is None or not configured.strip() or configured == "auto"


def dispatch(
    action: str,
    cli: str,
    fqbn: str,
    sketch: str,
    port: Optional[str],
    baud: Optional[str],
) -> int:
    """Resolve inputs and run the arduino-cli command for ``action``.

    This is the ONLY function that spawns subprocesses. For ``upload`` /
    ``monitor`` it runs the FQBN-filtered ``board list`` and feeds the
    detected ports to :func:`resolve_serial_port`.

    :param action: one of ``verify``, ``upload``, ``list-boards``,
        ``monitor``.
    :param cli: configured arduino-cli path or command name.
    :param fqbn: fully-qualified board name.
    :param sketch: sketch path (absolute, or relative to the repo root).
    :param port: configured serial port (``None``/blank/``auto`` ->
        auto-detect).
    :param baud: configured monitor baud rate.
    :return: the arduino-cli child process exit code.
    :raises ResolveError: on any unresolved cli / port / baud, or a
        failing ``board list``.
    :raises ValueError: on an unknown ``action`` (guarded by argparse).
    """
    resolved_cli = resolve_cli(cli)

    if action == "verify":
        sketch_dir = _resolve_sketch_dir(sketch)
        return _run_cli(
            resolved_cli,
            ["compile", "--fqbn", fqbn, str(sketch_dir)],
        )

    if action == "list-boards":
        return _run_cli(resolved_cli, ["board", "list"])

    if action == "upload":
        sketch_dir = _resolve_sketch_dir(sketch)
        detected = (
            _list_detected_ports(resolved_cli, fqbn)
            if _needs_autodetect(port)
            else []
        )
        upload_port = resolve_serial_port(detected, port)
        print(f"Uploading to {upload_port}")
        return _run_cli(
            resolved_cli,
            ["upload", "--fqbn", fqbn, "--port", upload_port, str(sketch_dir)],
        )

    if action == "monitor":
        detected = (
            _list_detected_ports(resolved_cli, fqbn)
            if _needs_autodetect(port)
            else []
        )
        monitor_port = resolve_serial_port(detected, port)
        monitor_baud = resolve_baud(baud)
        print(f"Monitoring {monitor_port} at {monitor_baud} baud")
        return _run_cli(
            resolved_cli,
            ["monitor", "--port", monitor_port, "--config", f"baudrate={monitor_baud}"],
        )

    raise ValueError(f"Unknown action: {action!r}")


def main(argv: Optional[list[str]] = None) -> int:
    """Argparse entry point.

    :param argv: optional explicit argv list (excluding the program
        name). When ``None`` (the default) :mod:`argparse` reads from
        :data:`sys.argv`.
    :return: process exit code -- the arduino-cli child exit code on
        success, or ``1`` when an input cannot be resolved.
    """
    parser = argparse.ArgumentParser(
        prog="arduino_cli_task.py",
        description="Cross-platform driver for the VS Code arduino-cli tasks.",
    )
    parser.add_argument(
        "action",
        choices=("verify", "upload", "list-boards", "monitor"),
        help="The arduino-cli task to run.",
    )
    parser.add_argument(
        "--cli",
        required=True,
        help="arduino-cli path or command name.",
    )
    parser.add_argument(
        "--fqbn",
        required=True,
        help="Fully-qualified board name (e.g. arduino:renesas_uno:unor4wifi).",
    )
    parser.add_argument(
        "--sketch",
        required=True,
        help="Sketch path (absolute, or relative to the repo root).",
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port. Blank or 'auto' triggers FQBN-filtered auto-detect.",
    )
    parser.add_argument(
        "--baud",
        default=None,
        help="Monitor baud rate (required for the 'monitor' action).",
    )
    args = parser.parse_args(argv)

    try:
        return dispatch(
            action=args.action,
            cli=args.cli,
            fqbn=args.fqbn,
            sketch=args.sketch,
            port=args.port,
            baud=args.baud,
        )
    except ResolveError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
