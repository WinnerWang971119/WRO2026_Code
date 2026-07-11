"""Command-stream JSON loader and validator for the CTPS playback engine.

Parses the JSON produced by the WRO 2026 Mission trace emitter, validates
the schema version, and converts each command entry into a typed
:class:`Command` dataclass for downstream consumption.

Supported schema version: **1**.

Unknown ``op`` values and unknown ``drive.method`` values are **skipped**
with a :class:`UserWarning` and do NOT raise, so the simulator degrades
gracefully when new ops are added to the firmware before the Python side is
updated.

A trace may also carry a top-level ``mosaic`` block (a sibling of the
``commands`` stream, echoed by ``ctps_dump`` when run with ``--mosaic``) and
``read_mosaic`` command ops. Both are parsed tolerantly: absent is normal
(``trace.mosaic`` is ``None``, no warning), and malformed is skipped with a
:class:`UserWarning` — never fatal.

Typical usage::

    from tools.path_planner.ctps.trace_loader import load

    trace = load("my_run.json")          # from a file path
    trace = load(raw_json_string)        # from an already-read string
    trace = load(parsed_dict)            # from an already-parsed dict

    for cmd in trace.commands:
        print(cmd.seq, cmd.op, cmd.fields)

    if trace.mosaic is not None:
        print(trace.mosaic.cells, trace.mosaic.geom_hash)
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
        "declare_position",
        "read_mosaic",
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
#: non-geometry timeline event. ``lineFollowDist`` is a distance-gated
#: LineFollower follow that also moves the chassis (a forward/reverse
#: advance), same geometry family as ``driveDist`` / ``stopAtLine`` /
#: ``wallStop``.
_KNOWN_DRIVE_METHODS: frozenset[str] = frozenset(
    {
        "driveDist",
        "turnDeg",
        "turnToYaw",
        "stop",
        "stopAtLine",
        "wallStop",
        "lineFollowDist",
    }
)

#: Recognised ``drive.mode`` strings.  A ``drive`` command whose ``mode`` is a
#: string NOT in this set — or is not a string at all — is warned about and
#: treated as unset (:mod:`track_builder` then renders along the current
#: facing rather than silently applying GYRO).
_KNOWN_DRIVE_MODES: frozenset[str] = frozenset(
    {
        "raw",
        "sync",
        "gyro",
    }
)

#: Recognised ``drive.preset`` strings (mirrors ``Drive::setPreset``).  A
#: ``drive`` command whose ``preset`` is a string NOT in this set — or is not
#: a string at all — is warned about and treated as unset (the resolver then
#: falls back to the NORMAL default).
_KNOWN_DRIVE_PRESETS: frozenset[str] = frozenset(
    {
        "slow",
        "normal",
        "fast",
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
    :param arrival: Forced arrival direction (``"front"`` / ``"rear"``); set for
        ``goto`` commands when ``setArrivalDirection`` was called. ``None`` when
        the key is absent (AUTO — defer to the sim's direction resolution).
    :param route: Forced route direction (``"front"`` / ``"rear"``); set for
        ``goto`` commands when ``setRouteDirection`` was called. ``None`` when
        the key is absent (AUTO).
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
        ``stopAtLine``, ``wallStop``, and ``lineFollowDist`` commands.
    :param deg: Angle in degrees; set for ``drive`` ``turnDeg`` /
        ``turnToYaw`` commands.
    :param mode: Drive velocity mode (``"raw"`` / ``"sync"`` / ``"gyro"``);
        set for ``drive`` commands when ``setMode`` was called. ``None`` when
        unset — the renderer then uses the current facing.
    :param heading: Held heading in degrees; set for ``drive`` commands when
        ``setHeading`` was called (honoured only under ``mode == "gyro"``).
    :param reverse: Reverse-direction flag; set for ``drive`` commands when
        ``setReverse`` was called. ``None`` when the key is absent.
    :param power: ``.setPower`` override (PWM); set for ``drive`` commands.
        ``None`` when unset — the resolver uses the preset/default cruise.
    :param preset: Speed preset (``"slow"`` / ``"normal"`` / ``"fast"``); set
        for ``drive`` commands when ``.setPreset`` was called. ``None`` when
        unset — the resolver defaults to NORMAL.
    :param start_speed: ``.setStartSpeed`` override (PWM); set for ``drive``
        commands. ``None`` when unset — the resolver applies the
        sign-by-cruise default.
    :param end_speed: ``.setEndSpeed`` override (PWM); set for ``drive``
        commands. ``None`` when unset — the resolver applies the default
        brake rule.
    :param profiled: ``True`` for ``.tShape()``, ``False`` for ``.flat()``;
        set for ``drive`` commands. ``None`` when unset — the resolver uses
        the builder default (tShape on for ``driveDist``).
    :param brake: ``.setBrake`` flag; set for ``drive`` commands. ``None``
        when unset — the resolver defaults to ``True``.
    :param node: Node name; set for ``declare_position`` commands.
    :param freq_hz: Tone frequency in Hz; set for ``beep`` commands.
    :param r: Red channel (0–255); set for ``led`` commands.
    :param g: Green channel (0–255); set for ``led`` commands.
    :param b: Blue channel (0–255); set for ``led`` commands.
    :param result: Outcome string (``"ok"`` or ``"yield"``); set for
        ``task_end`` commands.
    :param success: Read outcome; set for ``read_mosaic`` commands. ``True``
        unless the dump was run with ``--mosaic-fail-at`` targeting this
        call. ``"op"`` remains the sole discriminator — ``success`` is only
        ever a payload bool, never treated as a second op tag.
    :param fields: Forward-compatible catch-all for any key present in the
        raw JSON entry that is not captured by the named fields above.
        Unknown ops and unrecognised ``drive.method`` commands are dropped
        entirely (``_parse_command`` returns ``None``), so they never reach
        ``fields``.
    """

    seq: int
    op: str

    # -- op-specific optional fields ----------------------------------------
    target:      Optional[str]   = None   # goto
    arrival:     Optional[str]   = None   # goto: forced arrival direction "front"|"rear"
    route:       Optional[str]   = None   # goto: forced route direction "front"|"rear"
    ms:          Optional[int]   = None   # wait, beep, led
    text:        Optional[str]   = None   # print
    name:        Optional[str]   = None   # task_start, task_end
    args_hex:    Optional[str]   = None   # task_start
    method:      Optional[str]   = None   # manip, drive
    arg:         Optional[int]   = None   # manip
    cm:          Optional[float] = None   # drive driveDist
    deg:         Optional[float] = None   # drive turnDeg / turnToYaw
    mode:        Optional[str]   = None   # drive: "raw"|"sync"|"gyro"
    heading:     Optional[float] = None   # drive: held heading (gyro)
    reverse:     Optional[bool]  = None   # drive: reverse-direction flag
    power:       Optional[int]   = None   # drive: .setPower override (PWM)
    preset:      Optional[str]   = None   # drive: "slow"|"normal"|"fast"
    start_speed: Optional[int]   = None   # drive: .setStartSpeed override (PWM)
    end_speed:   Optional[int]   = None   # drive: .setEndSpeed override (PWM)
    profiled:    Optional[bool]  = None   # drive: true=.tShape, false=.flat
    brake:       Optional[bool]  = None   # drive: .setBrake flag
    node:        Optional[str]   = None   # declare_position
    freq_hz:     Optional[int]   = None   # beep
    r:           Optional[int]   = None   # led
    g:           Optional[int]   = None   # led
    b:           Optional[int]   = None   # led
    result:      Optional[str]   = None   # task_end
    success:     Optional[bool]  = None   # read_mosaic

    # -- forward-compatible extras -------------------------------------------
    fields: dict = field(default_factory=dict)


@dataclass
class MosaicTrace:
    """The top-level ``mosaic`` echo block from a ``--mosaic``-injected dump.

    A sibling of the ``commands`` stream (not an op) — see the ``Trace
    additions`` section of the CTPS randomized-vision plan. Present only when
    ``ctps_dump`` was run with ``--mosaic``; absent for an old trace or a dump
    run without the flag (``Trace.mosaic`` is ``None`` in that case, with no
    warning).

    :param cells: Exactly 12 color-name strings, cell-index order
        (``i = row*cols + col``). Not validated against the color palette
        here — the loader is tolerant; a bad name is a rendering concern, not
        a load-time error.
    :param seed: The seed used to roll the mosaic, if one was echoed
        (``--mosaic-seed`` was passed to the dump). ``None`` when absent —
        this is a normal, unwarned case (the dump omits the key when no seed
        was resolved).
    :param geom_hash: Geometry fingerprint string echoed by the dump, for the
        player's "geometry changed since this trace was recorded" guard.
    :param injected: Always ``True`` for a block that reached this dataclass
        (the dump only echoes ``mosaic`` when injection occurred).
    """

    cells: list[str]
    geom_hash: str
    injected: bool
    seed: Optional[int] = None


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
    :param mosaic: The top-level ``mosaic`` echo block, or ``None`` when
        absent (no ``--mosaic`` on the dump) or malformed (skipped with a
        :class:`UserWarning` — never fatal).
    """

    schema_version: int
    start_node: str
    start_heading_deg: float
    build_error: bool
    commands: list[Command]
    mosaic: Optional[MosaicTrace] = None


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
        "target", "arrival", "route", "ms", "text", "name", "args_hex",
        "method", "arg", "cm", "deg",
        "mode", "heading", "reverse",
        "power", "preset", "start_speed", "end_speed", "profiled", "brake",
        "node",
        "freq_hz", "r", "g", "b", "result",
        "success",
    }
    extra_fields: dict = {k: v for k, v in raw.items() if k not in named_keys}

    cmd = Command(
        seq=seq,
        op=op,
        target=raw.get("target"),
        arrival=raw.get("arrival"),
        route=raw.get("route"),
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

    # ----- drive: validate method + mode/heading/reverse --------------------
    if op == "drive":
        method = raw.get("method", "")
        if method not in _KNOWN_DRIVE_METHODS:
            warnings.warn(
                f"ctps.trace_loader: unknown drive.method {method!r} at seq={seq} — skipped.",
                UserWarning,
                stacklevel=4,
            )
            # Drop unrecognised drive methods entirely (forward-compat): the
            # renderer has no geometry for them, so exclude from the command
            # list by returning None.
            return None

        # mode: must be a string AND in the known set. Type-check that it is a
        # string BEFORE the set-membership test so a non-string (e.g. a list)
        # warns rather than raising. An invalid mode is treated as unset — the
        # renderer uses the current facing rather than silently applying GYRO.
        if "mode" in raw:
            raw_mode = raw["mode"]
            if isinstance(raw_mode, str) and raw_mode in _KNOWN_DRIVE_MODES:
                cmd.mode = raw_mode
            else:
                warnings.warn(
                    f"ctps.trace_loader: malformed drive.mode {raw_mode!r} at "
                    f"seq={seq} — treated as unset.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.mode = None

        # heading: accept only a real int/float; a non-numeric value warns and is
        # dropped. A JSON boolean is excluded explicitly — `bool` subclasses
        # `int`, so `isinstance(True, (int, float))` is truthy and would silently
        # store a heading of 1.0 (this mirrors the strict `reverse` guard below).
        # Absent → leave None.
        if "heading" in raw:
            raw_heading = raw["heading"]
            if isinstance(raw_heading, (int, float)) and not isinstance(raw_heading, bool):
                cmd.heading = raw_heading
            else:
                warnings.warn(
                    f"ctps.trace_loader: non-numeric drive.heading "
                    f"{raw_heading!r} at seq={seq} — dropped.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.heading = None

        # reverse: accept ONLY a JSON boolean. Never bool(raw.get("reverse")) —
        # the string "false" is truthy under Python bool(), which would invert
        # intent. Any non-bool value warns and is treated as False. Absent →
        # leave None.
        if "reverse" in raw:
            raw_reverse = raw["reverse"]
            if isinstance(raw_reverse, bool):
                cmd.reverse = raw_reverse
            else:
                warnings.warn(
                    f"ctps.trace_loader: non-boolean drive.reverse "
                    f"{raw_reverse!r} at seq={seq} — treated as False.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.reverse = False

        # power: accept only a real int (not a bool — `bool` subclasses `int`,
        # so a stray `true`/`false` in this slot must NOT silently become
        # 1/0). A non-int-or-bool-int value warns and is treated as unset.
        # Absent → leave None.
        if "power" in raw:
            raw_power = raw["power"]
            if isinstance(raw_power, int) and not isinstance(raw_power, bool):
                cmd.power = raw_power
            else:
                warnings.warn(
                    f"ctps.trace_loader: non-numeric drive.power "
                    f"{raw_power!r} at seq={seq} — treated as unset.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.power = None

        # preset: must be a string AND in the known set. Type-check that it is
        # a string BEFORE the set-membership test so a non-string (e.g. a
        # list) warns rather than raising. An invalid preset is treated as
        # unset — the resolver defaults to NORMAL.
        if "preset" in raw:
            raw_preset = raw["preset"]
            if isinstance(raw_preset, str) and raw_preset in _KNOWN_DRIVE_PRESETS:
                cmd.preset = raw_preset
            else:
                warnings.warn(
                    f"ctps.trace_loader: malformed drive.preset {raw_preset!r} at "
                    f"seq={seq} — treated as unset.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.preset = None

        # start_speed: accept only a real int (bool excluded — see `power`).
        # Absent → leave None.
        if "start_speed" in raw:
            raw_start_speed = raw["start_speed"]
            if isinstance(raw_start_speed, int) and not isinstance(raw_start_speed, bool):
                cmd.start_speed = raw_start_speed
            else:
                warnings.warn(
                    f"ctps.trace_loader: non-numeric drive.start_speed "
                    f"{raw_start_speed!r} at seq={seq} — treated as unset.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.start_speed = None

        # end_speed: accept only a real int (bool excluded — see `power`).
        # Absent → leave None.
        if "end_speed" in raw:
            raw_end_speed = raw["end_speed"]
            if isinstance(raw_end_speed, int) and not isinstance(raw_end_speed, bool):
                cmd.end_speed = raw_end_speed
            else:
                warnings.warn(
                    f"ctps.trace_loader: non-numeric drive.end_speed "
                    f"{raw_end_speed!r} at seq={seq} — treated as unset.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.end_speed = None

        # cm / deg: accept only a real int/float (bool excluded — see `heading`).
        # `bool` subclasses `int`, so a JSON `true`/`false` in this slot would
        # otherwise coerce to 1.0/0.0 and drive a phantom distance (`cm`) or a
        # spurious 1° turn (`deg`). A non-numeric value warns and is dropped to
        # None so a malformed field warns-and-skips downstream (the `cm`/`deg`
        # None guards in track_builder) instead of silently moving. Absent → the
        # constructor already left it None.
        if "cm" in raw:
            raw_cm = raw["cm"]
            if isinstance(raw_cm, (int, float)) and not isinstance(raw_cm, bool):
                cmd.cm = raw_cm
            else:
                warnings.warn(
                    f"ctps.trace_loader: non-numeric drive.cm {raw_cm!r} at "
                    f"seq={seq} — dropped.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.cm = None

        if "deg" in raw:
            raw_deg = raw["deg"]
            if isinstance(raw_deg, (int, float)) and not isinstance(raw_deg, bool):
                cmd.deg = raw_deg
            else:
                warnings.warn(
                    f"ctps.trace_loader: non-numeric drive.deg {raw_deg!r} at "
                    f"seq={seq} — dropped.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.deg = None

        # profiled: accept ONLY a JSON boolean (mirrors the `reverse` guard —
        # never bool(raw.get(...)), a truthy non-bool must not silently
        # coerce). Any non-bool value warns and is treated as unset (the
        # builder default: tShape on for driveDist). Absent → leave None.
        if "profiled" in raw:
            raw_profiled = raw["profiled"]
            if isinstance(raw_profiled, bool):
                cmd.profiled = raw_profiled
            else:
                warnings.warn(
                    f"ctps.trace_loader: non-boolean drive.profiled "
                    f"{raw_profiled!r} at seq={seq} — treated as unset.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.profiled = None

        # brake: accept ONLY a JSON boolean (mirrors `reverse`/`profiled`).
        # Any non-bool value warns and is treated as unset (the resolver
        # default: brake=True). Absent → leave None.
        if "brake" in raw:
            raw_brake = raw["brake"]
            if isinstance(raw_brake, bool):
                cmd.brake = raw_brake
            else:
                warnings.warn(
                    f"ctps.trace_loader: non-boolean drive.brake "
                    f"{raw_brake!r} at seq={seq} — treated as unset.",
                    UserWarning,
                    stacklevel=4,
                )
                cmd.brake = None

    # ----- declare_position: validate node -----------------------------------
    if op == "declare_position":
        raw_node = raw.get("node")
        if isinstance(raw_node, str) and raw_node:
            cmd.node = raw_node
        else:
            warnings.warn(
                f"ctps.trace_loader: malformed declare_position.node "
                f"{raw_node!r} at seq={seq} — skipped.",
                UserWarning,
                stacklevel=4,
            )
            # A declare_position with no valid node name carries no usable
            # payload for track_builder (nothing to declare) — drop it
            # entirely, mirroring the unknown-drive-method skip above.
            return None

    # ----- read_mosaic: validate success --------------------------------------
    if op == "read_mosaic":
        # "op" is the sole discriminator; "success" is only ever a payload
        # bool. Accept ONLY a JSON boolean (mirrors the drive `reverse` /
        # `profiled` / `brake` guards — never bool(raw.get(...)), since a
        # stray non-bool must not silently coerce to True/False).
        raw_success = raw.get("success")
        if isinstance(raw_success, bool):
            cmd.success = raw_success
        else:
            warnings.warn(
                f"ctps.trace_loader: malformed read_mosaic.success "
                f"{raw_success!r} at seq={seq} — treated as unset.",
                UserWarning,
                stacklevel=4,
            )
            cmd.success = None

    return cmd


def _parse_mosaic(raw: object) -> Optional[MosaicTrace]:
    """Convert the raw top-level ``mosaic`` value to a :class:`MosaicTrace`.

    Tolerant like the rest of this loader: an absent block is normal (no
    warning, returns ``None``); a present-but-malformed block skips with a
    :class:`UserWarning` and also returns ``None``. Never raises.

    :param raw: The value at the top-level ``"mosaic"`` key (``None`` if the
        key was absent from the trace JSON).
    :return: A :class:`MosaicTrace`, or ``None`` if absent/malformed.
    """
    if raw is None:
        # No `--mosaic` on the dump (or an old trace) — normal, no warning.
        return None

    if not isinstance(raw, dict):
        warnings.warn(
            f"ctps.trace_loader: top-level 'mosaic' is not a dict "
            f"(got {type(raw).__name__!r}) — skipped.",
            UserWarning,
            stacklevel=4,
        )
        return None

    raw_cells = raw.get("cells")
    if (
        not isinstance(raw_cells, list)
        or len(raw_cells) != 12
        or not all(isinstance(c, str) for c in raw_cells)
    ):
        warnings.warn(
            f"ctps.trace_loader: malformed 'mosaic.cells' {raw_cells!r} "
            f"(expected a list of 12 color-name strings) — 'mosaic' block skipped.",
            UserWarning,
            stacklevel=4,
        )
        return None

    raw_geom_hash = raw.get("geom_hash")
    if not isinstance(raw_geom_hash, str) or not raw_geom_hash:
        warnings.warn(
            f"ctps.trace_loader: malformed 'mosaic.geom_hash' {raw_geom_hash!r} "
            f"(expected a non-empty string) — 'mosaic' block skipped.",
            UserWarning,
            stacklevel=4,
        )
        return None

    raw_injected = raw.get("injected")
    if not isinstance(raw_injected, bool):
        warnings.warn(
            f"ctps.trace_loader: malformed 'mosaic.injected' {raw_injected!r} "
            f"(expected a bool) — 'mosaic' block skipped.",
            UserWarning,
            stacklevel=4,
        )
        return None

    # seed is OPTIONAL — absent is normal (no --mosaic-seed on the dump), not
    # a warning. Present-but-wrong-type is malformed and skips the whole
    # block, consistent with every other field here. A JSON bool must be
    # rejected explicitly since `bool` subclasses `int`.
    seed: Optional[int] = None
    if "seed" in raw:
        raw_seed = raw["seed"]
        if isinstance(raw_seed, int) and not isinstance(raw_seed, bool):
            seed = raw_seed
        else:
            warnings.warn(
                f"ctps.trace_loader: malformed 'mosaic.seed' {raw_seed!r} "
                f"(expected an int or absent) — 'mosaic' block skipped.",
                UserWarning,
                stacklevel=4,
            )
            return None

    return MosaicTrace(
        cells=list(raw_cells),
        geom_hash=raw_geom_hash,
        injected=raw_injected,
        seed=seed,
    )


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

    mosaic = _parse_mosaic(data.get("mosaic"))

    return Trace(
        schema_version=version,
        start_node=start_node,
        start_heading_deg=start_heading_deg,
        build_error=build_error,
        commands=commands,
        mosaic=mosaic,
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
