"""Command-stream JSON loader and validator for the CTPS playback engine.

Parses the JSON produced by the WRO 2026 Mission trace emitter, validates
the schema version, and converts each command entry into a typed
:class:`Command` dataclass for downstream consumption.

Supported schema version: **1**.

Unknown ``op`` values and unknown ``drive.method`` values are **skipped**
with a :class:`UserWarning` and do NOT raise, so the simulator degrades
gracefully when new ops are added to the firmware before the Python side is
updated.

Typical usage::

    from tools.path_planner.ctps.trace_loader import load

    trace = load("my_run.json")          # from a file path
    trace = load(raw_json_string)        # from an already-read string
    trace = load(parsed_dict)            # from an already-parsed dict

    for cmd in trace.commands:
        print(cmd.seq, cmd.op, cmd.fields)
"""

from __future__ import annotations

import json
import os
import warnings
from dataclasses import dataclass, field
from typing import Optional

# ---------------------------------------------------------------------------
# Schema contract
# ---------------------------------------------------------------------------

#: The only schema version this loader accepts.
SUPPORTED_SCHEMA_VERSION: int = 1

#: Known v1 ``op`` values.  An entry NOT in this set is skipped with a
#: :class:`UserWarning`.
_KNOWN_OPS: frozenset[str] = frozenset(
    {
        "goto",
        "task_start",
        "task_end",
        "manip",
        "drive",
        "wait",
        "print",
        "beep",
        "led",
    }
)

#: Recognised ``drive.method`` values.  A ``drive`` command whose ``method``
#: is NOT in this set is skipped with a :class:`UserWarning`.
#:
#: ``driveDist`` / ``turnDeg`` / ``turnToYaw`` move the chassis; ``stopAtLine``
#: and ``wallStop`` also move it — their ``cm`` is the nominal expected advance,
#: which :mod:`track_builder` replays as a forward push estimate. ``stop``
#: carries no geometry — the recorder emits it (``recordDriveStop``) to
#: preserve execution order, and :mod:`track_builder` renders it as a
#: non-geometry timeline event.
_KNOWN_DRIVE_METHODS: frozenset[str] = frozenset(
    {
        "driveDist",
        "turnDeg",
        "turnToYaw",
        "stop",
        "stopAtLine",
        "wallStop",
    }
)


# ---------------------------------------------------------------------------
# Public dataclasses
# ---------------------------------------------------------------------------


@dataclass
class Command:
    """A single command entry from the trace command-stream.

    :param seq: Zero-based sequence index from the JSON ``seq`` field.
    :param op: Operation name (e.g. ``"goto"``, ``"drive"``, ``"beep"``).
    :param target: Destination node name; set for ``goto`` commands.
    :param ms: Duration in milliseconds; set for ``wait``, ``beep``, and
        ``led`` commands.
    :param text: Human-readable string payload; set for ``print`` commands.
    :param name: Task name; set for ``task_start`` / ``task_end`` commands.
    :param args_hex: Raw 16-byte :class:`TaskArgs` as host-endian hex; set
        for ``task_start`` commands.  Not decoded in schema v1.
    :param method: Method name string; set for ``manip`` and ``drive``
        commands.
    :param arg: Uint8 argument; set for ``manip`` commands.
    :param cm: Distance in centimetres; set for ``drive`` ``driveDist``,
        ``stopAtLine``, and ``wallStop`` commands.
    :param deg: Angle in degrees; set for ``drive`` ``turnDeg`` /
        ``turnToYaw`` commands.
    :param freq_hz: Tone frequency in Hz; set for ``beep`` commands.
    :param r: Red channel (0–255); set for ``led`` commands.
    :param g: Green channel (0–255); set for ``led`` commands.
    :param b: Blue channel (0–255); set for ``led`` commands.
    :param result: Outcome string (``"ok"`` or ``"yield"``); set for
        ``task_end`` commands.
    :param fields: Forward-compatible catch-all for any key present in the
        raw JSON entry that is not captured by the named fields above.
        Also populated with ``{"method": ..., "raw_args": ...}`` when a
        ``drive`` command has an unrecognised method.
    """

    seq: int
    op: str

    # -- op-specific optional fields ----------------------------------------
    target:   Optional[str]   = None   # goto
    ms:       Optional[int]   = None   # wait, beep, led
    text:     Optional[str]   = None   # print
    name:     Optional[str]   = None   # task_start, task_end
    args_hex: Optional[str]   = None   # task_start
    method:   Optional[str]   = None   # manip, drive
    arg:      Optional[int]   = None   # manip
    cm:       Optional[float] = None   # drive driveDist
    deg:      Optional[float] = None   # drive turnDeg / turnToYaw
    freq_hz:  Optional[int]   = None   # beep
    r:        Optional[int]   = None   # led
    g:        Optional[int]   = None   # led
    b:        Optional[int]   = None   # led
    result:   Optional[str]   = None   # task_end

    # -- forward-compatible extras -------------------------------------------
    fields: dict = field(default_factory=dict)


@dataclass
class Trace:
    """A fully-parsed and validated command-stream trace.

    :param schema_version: Always ``1`` for traces accepted by this loader.
    :param start_node: The node name the robot started from (informational;
        the track builder seeds from ``field_data`` as the authoritative
        source).
    :param start_heading_deg: Robot heading at trace start, in degrees
        (CCW from +x, any wrap accepted).
    :param build_error: ``True`` when the Mission reported a build error
        (e.g. an unknown task name).  The command list may be partial.
    :param commands: Ordered list of parsed :class:`Command` objects.
        Unknown ops and unknown ``drive.method`` values are excluded.
    """

    schema_version: int
    start_node: str
    start_heading_deg: float
    build_error: bool
    commands: list[Command]


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _parse_command(raw: object, index: int) -> Optional[Command]:
    """Convert one raw JSON command dict to a :class:`Command`.

    :param raw: A single entry from the ``commands`` JSON array.
    :param index: Zero-based position in the array, used in warning messages
        when ``seq`` is unavailable.
    :return: A :class:`Command` instance, or ``None`` if the entry is
        structurally invalid, its ``op`` is unrecognised, or its
        ``drive.method`` is unrecognised — all of which are skipped with a
        :class:`UserWarning`.
    """
    # Guard: entry must be a mapping.
    if not isinstance(raw, dict):
        warnings.warn(
            f"ctps.trace_loader: command at index {index} is not a dict "
            f"(got {type(raw).__name__!r}) — skipped.",
            UserWarning,
            stacklevel=4,
        )
        return None

    # Guard: required keys must be present.
    if "seq" not in raw or "op" not in raw:
        missing = [k for k in ("seq", "op") if k not in raw]
        warnings.warn(
            f"ctps.trace_loader: command at index {index} missing required "
            f"key(s) {missing!r} — skipped.",
            UserWarning,
            stacklevel=4,
        )
        return None

    seq: int = int(raw["seq"])
    op: str  = str(raw["op"])

    # ----- unknown op -------------------------------------------------------
    if op not in _KNOWN_OPS:
        warnings.warn(
            f"ctps.trace_loader: unknown op {op!r} at seq={seq} — skipped.",
            UserWarning,
            stacklevel=4,   # surface at the load() call site
        )
        return None

    # ----- collect named fields ---------------------------------------------
    named_keys = {
        "seq", "op",
        "target", "ms", "text", "name", "args_hex",
        "method", "arg", "cm", "deg",
        "freq_hz", "r", "g", "b", "result",
    }
    extra_fields: dict = {k: v for k, v in raw.items() if k not in named_keys}

    cmd = Command(
        seq=seq,
        op=op,
        target=raw.get("target"),
        ms=raw.get("ms"),
        text=raw.get("text"),
        name=raw.get("name"),
        args_hex=raw.get("args_hex"),
        method=raw.get("method"),
        arg=raw.get("arg"),
        cm=raw.get("cm"),
        deg=raw.get("deg"),
        freq_hz=raw.get("freq_hz"),
        r=raw.get("r"),
        g=raw.get("g"),
        b=raw.get("b"),
        result=raw.get("result"),
        fields=extra_fields,
    )

    # ----- drive: validate method -------------------------------------------
    if op == "drive":
        method = raw.get("method", "")
        if method not in _KNOWN_DRIVE_METHODS:
            warnings.warn(
                f"ctps.trace_loader: unknown drive.method {method!r} at seq={seq} — skipped.",
                UserWarning,
                stacklevel=4,
            )
            # Capture for forward-compatibility inspection but exclude from
            # the geometry-affecting command list by returning None.
            return None

    return cmd


def _parse_dict(data: object) -> Trace:
    """Parse and validate an already-decoded JSON object.

    :param data: Top-level value from the command-stream JSON.  Must be a
        mapping; anything else raises immediately.
    :return: A validated :class:`Trace`.
    :raises ValueError: if ``data`` is not a dict, if ``schema_version`` is
        missing or not equal to :data:`SUPPORTED_SCHEMA_VERSION`, or if
        ``commands`` is present but is not a list.
    """
    if not isinstance(data, dict):
        raise ValueError(
            f"ctps.trace_loader: top-level JSON value must be a dict, "
            f"got {type(data).__name__!r}."
        )

    raw_version = data.get("schema_version")
    if raw_version is None:
        raise ValueError(
            "ctps.trace_loader: missing 'schema_version' field in command-stream JSON."
        )
    version = int(raw_version)
    if version != SUPPORTED_SCHEMA_VERSION:
        raise ValueError(
            f"ctps.trace_loader: unsupported schema_version={version!r}; "
            f"expected {SUPPORTED_SCHEMA_VERSION}."
        )

    start_node: str          = str(data.get("start_node", ""))
    start_heading_deg: float = float(data.get("start_heading_deg", 0.0))
    build_error: bool        = bool(data.get("build_error", False))

    raw_commands = data.get("commands", [])
    if not isinstance(raw_commands, list):
        raise ValueError(
            f"ctps.trace_loader: 'commands' must be a list, "
            f"got {type(raw_commands).__name__!r}."
        )
    commands: list[Command] = []
    for idx, raw_cmd in enumerate(raw_commands):
        parsed = _parse_command(raw_cmd, idx)
        if parsed is not None:
            commands.append(parsed)

    return Trace(
        schema_version=version,
        start_node=start_node,
        start_heading_deg=start_heading_deg,
        build_error=build_error,
        commands=commands,
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def load(source: "str | os.PathLike | dict") -> Trace:
    """Load and validate a command-stream trace.

    Accepts three input forms:

    * A :class:`pathlib.Path` or any :class:`os.PathLike` object — read as
      UTF-8 JSON.
    * A ``str`` that is an existing file path — read as UTF-8 JSON.
    * A ``str`` that is a raw JSON document (detected by the absence of an
      existing file at that path) — parsed directly.
    * A ``dict`` — treated as an already-parsed JSON object.

    :param source: Input source — file path, raw JSON string, or parsed dict.
    :return: A validated :class:`Trace` with unknown ops/methods excluded.
    :raises ValueError: if the decoded JSON is not a dict, if
        ``schema_version`` is missing or unsupported, or if ``commands`` is
        present but is not a list.
    :raises json.JSONDecodeError: if the JSON cannot be parsed.
    """
    if isinstance(source, dict):
        return _parse_dict(source)

    if isinstance(source, os.PathLike):
        # Always treat PathLike objects as file paths.
        path = os.fspath(source)
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
        return _parse_dict(data)

    # source is a str — disambiguate: file path vs raw JSON document.
    assert isinstance(source, str)
    if os.path.exists(source):
        with open(source, encoding="utf-8") as fh:
            data = json.load(fh)
        return _parse_dict(data)

    # Not an existing path — parse as a raw JSON string.
    data = json.loads(source)
    return _parse_dict(data)
