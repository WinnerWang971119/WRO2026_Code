"""Hermetic tests for the CTPS command-stream loader.

Exercises :func:`tools.path_planner.ctps.trace_loader.load` against
hand-authored Data Model A command streams. No compiler, no sim — pure
parse/validate.

Covered cases:

* **TC7** — an unknown ``op`` and an unknown ``drive.method`` each warn with a
  :class:`UserWarning`, do NOT raise, are skipped, and leave every recognized
  command intact so the simulator degrades gracefully when the firmware adds
  ops ahead of the Python side.
* **NC6** — the loader populates ``Command.mode`` / ``.heading`` / ``.reverse``
  from the new optional JSON keys, and ``lineFollowDist`` is in the whitelist
  (not dropped).
* **NC7** — malformed values (non-string ``mode``, unknown-string ``mode``,
  non-numeric ``heading``, non-bool ``reverse``) each produce a
  :class:`UserWarning`, the command is NOT dropped, and the affected field is
  set to the safe default (``None`` for mode/heading, ``False`` for reverse).
* **NC8** — all 8 pre-existing v1 fixtures load without error and have
  ``mode``/``heading``/``reverse`` defaulting to ``None`` on every command
  (schema change is additive; no new required keys).
* **T9 (this task)** — the additive ``drive`` fields (``power``, ``preset``,
  ``start_speed``, ``end_speed``, ``profiled``, ``brake``) parse per the
  Contracts JSON seam; absent → ``None``; a bad-type int field warns and
  unsets; a JSON bool masquerading in an int slot is rejected (``bool``
  subclasses ``int`` in Python); the new ``declare_position`` op parses its
  ``node`` field and a malformed/missing ``node`` warns and is skipped.
* **TC20 (#154-20, this task)** — a top-level ``mosaic.cells`` entry outside
  the :data:`mosaic.COLOR_NAMES` ``∪`` {:data:`mosaic.AIR_NAME`}
  whitelist rejects the WHOLE block (``trace.mosaic is None``) with exactly
  one :class:`UserWarning`, never a partial 11-cell mosaic; a block whose
  cells are all real colours or the AIR sentinel loads untouched with no
  warning.
"""

from __future__ import annotations

import warnings
from pathlib import Path

import pytest

from tools.path_planner.ctps import mosaic
from tools.path_planner.ctps.trace_loader import load

_FIXTURES = Path(__file__).resolve().parent / "fixtures"


# ---------------------------------------------------------------------------
# TC7 — unknown op + unknown drive.method warn, skip, and never raise
# ---------------------------------------------------------------------------


def test_tc7_unknown_op_and_drive_method_warn_and_skip():
    """TC7: unknown op + unknown drive.method are skipped with a UserWarning.

    Key assertion: :func:`load` emits a :class:`UserWarning` (it does NOT
    raise), the unknown ``teleport`` op and the unknown ``drive.strafeLeft``
    method are dropped, and the four recognized commands (``task_start``,
    ``drive driveDist``, ``goto``, ``task_end``) survive in order.
    """
    source = _FIXTURES / "ctps_unknown_ops.json"

    with pytest.warns(UserWarning):
        trace = load(source)

    # Recognized commands survive; unknown entries are excluded.
    ops = [(cmd.op, cmd.method) for cmd in trace.commands]
    assert ops == [
        ("task_start", None),
        ("drive", "driveDist"),
        ("goto", None),
        ("task_end", None),
    ]

    # No unknown op leaked through.
    assert all(cmd.op != "teleport" for cmd in trace.commands)
    # No unknown drive method leaked through.
    assert all(
        not (cmd.op == "drive" and cmd.method == "strafeLeft")
        for cmd in trace.commands
    )

    # The recognized drive kept its geometry payload.
    drive = next(cmd for cmd in trace.commands if cmd.op == "drive")
    assert drive.method == "driveDist"
    assert drive.cm == pytest.approx(10.0)

    # The recognized goto kept its target.
    goto = next(cmd for cmd in trace.commands if cmd.op == "goto")
    assert goto.target == "B"


def test_tc7_warns_per_unknown_entry():
    """TC7 (detail): both unknowns warn — unknown op AND unknown drive.method.

    Key assertion: the recorded warnings name the unknown ``op`` and the
    unknown ``drive.method`` separately, confirming each skip path fires its
    own :class:`UserWarning` rather than a single blanket warning.
    """
    source = _FIXTURES / "ctps_unknown_ops.json"

    with pytest.warns(UserWarning) as record:
        load(source)

    messages = "\n".join(str(w.message) for w in record)
    assert "unknown op" in messages
    assert "teleport" in messages
    assert "unknown drive.method" in messages
    assert "strafeLeft" in messages


# ---------------------------------------------------------------------------
# NC6 — parse + whitelist: mode / heading / reverse populated from JSON
# ---------------------------------------------------------------------------


def test_nc6_mode_heading_reverse_populated():
    """NC6: mode/heading/reverse are parsed from the new optional JSON keys.

    Key assertions:
    * gyro driveDist gets ``mode == "gyro"`` and ``heading == 90.0``.
    * plain driveDist (no mode/heading keys) leaves them ``None``.
    * ``lineFollowDist`` commands are NOT dropped (they are in the whitelist).
    * reversed commands get ``reverse is True``.
    * the new keys do NOT leak into ``Command.fields``.
    """
    # -- ctps_gyro_heading.json --------------------------------------------------
    trace = load(_FIXTURES / "ctps_gyro_heading.json")
    assert len(trace.commands) == 3

    gyro_dd = trace.commands[0]   # driveDist cm=35, mode=gyro, heading=90
    assert gyro_dd.method == "driveDist"
    assert gyro_dd.cm == pytest.approx(35.0)
    assert gyro_dd.mode == "gyro"
    assert gyro_dd.heading == pytest.approx(90.0)
    assert gyro_dd.reverse is None
    assert gyro_dd.fields == {}

    plain_dd = trace.commands[1]  # driveDist cm=20, no mode/heading/reverse keys
    assert plain_dd.method == "driveDist"
    assert plain_dd.mode is None
    assert plain_dd.heading is None
    assert plain_dd.reverse is None
    assert plain_dd.fields == {}

    gyro_sal = trace.commands[2]  # stopAtLine cm=12, mode=gyro, heading=90
    assert gyro_sal.method == "stopAtLine"
    assert gyro_sal.mode == "gyro"
    assert gyro_sal.heading == pytest.approx(90.0)
    assert gyro_sal.reverse is None
    assert gyro_sal.fields == {}

    # -- ctps_line_follow.json ---------------------------------------------------
    trace_lf = load(_FIXTURES / "ctps_line_follow.json")
    assert len(trace_lf.commands) == 2  # lineFollowDist is whitelisted → not dropped

    lf_plain = trace_lf.commands[0]  # lineFollowDist cm=60, no reverse key
    assert lf_plain.method == "lineFollowDist"
    assert lf_plain.cm == pytest.approx(60.0)
    assert lf_plain.mode is None
    assert lf_plain.heading is None
    assert lf_plain.reverse is None
    assert lf_plain.fields == {}

    lf_rev = trace_lf.commands[1]  # lineFollowDist cm=30, reverse=true
    assert lf_rev.method == "lineFollowDist"
    assert lf_rev.cm == pytest.approx(30.0)
    assert lf_rev.reverse is True
    assert lf_rev.fields == {}

    # -- ctps_reverse.json -------------------------------------------------------
    trace_r = load(_FIXTURES / "ctps_reverse.json")
    assert len(trace_r.commands) == 4

    methods = [cmd.method for cmd in trace_r.commands]
    assert methods == ["lineFollowDist", "stopAtLine", "wallStop", "driveDist"]

    for cmd in trace_r.commands[:3]:  # lineFollowDist, stopAtLine, wallStop: reverse=true
        assert cmd.reverse is True, f"expected reverse=True on {cmd.method}"

    fwd = trace_r.commands[3]  # driveDist has no reverse key → remains None
    assert fwd.method == "driveDist"
    assert fwd.reverse is None


# ---------------------------------------------------------------------------
# NC7 — negative paths: malformed mode / heading / reverse warn + safe default
# ---------------------------------------------------------------------------


def _drive_trace(*cmds: dict) -> dict:
    """Return a minimal v1 trace dict containing the given drive command dicts."""
    return {
        "schema_version": 1,
        "start_node": "A",
        "start_heading_deg": 0.0,
        "build_error": False,
        "commands": list(cmds),
    }


def test_nc7_malformed_drive_fields_warn_and_default():
    """NC7: each malformed mode/heading/reverse value warns and falls to a safe default.

    Key assertions per case:
    * The command is NOT dropped (still appears in ``trace.commands``).
    * The affected field is the safe default: ``None`` for mode/heading,
      ``False`` for reverse.
    * A non-string ``mode`` value does NOT raise — it warns instead.
    * ``reverse="false"`` (a Python-truthy string, not a JSON bool) becomes
      ``False``, never ``True``.
    """
    # Case 1: non-string mode (a list) → warns, mode is None, command survives
    data = _drive_trace(
        {"seq": 0, "op": "drive", "method": "driveDist", "cm": 10.0, "mode": ["gyro"]}
    )
    with pytest.warns(UserWarning):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].mode is None

    # Case 2: unknown-string mode → warns, mode is None, command survives
    data = _drive_trace(
        {"seq": 0, "op": "drive", "method": "driveDist", "cm": 10.0, "mode": "turbo"}
    )
    with pytest.warns(UserWarning):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].mode is None

    # Case 3: non-numeric heading → warns, heading is None, command survives
    data = _drive_trace(
        {"seq": 0, "op": "drive", "method": "driveDist", "cm": 10.0, "heading": "north"}
    )
    with pytest.warns(UserWarning):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].heading is None

    # Case 4: reverse="false" (truthy string, not a JSON bool) → warns, False not True
    data = _drive_trace(
        {"seq": 0, "op": "drive", "method": "lineFollowDist", "cm": 30.0, "reverse": "false"}
    )
    with pytest.warns(UserWarning):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].reverse is False


# ---------------------------------------------------------------------------
# NC8 — 8 pre-existing v1 fixtures load unchanged (additive schema)
# ---------------------------------------------------------------------------

_PRE_EXISTING_CTPS_FIXTURES = [
    "ctps_drive_drift",
    "ctps_drive_no_drift",
    "ctps_lrs_track",
    "ctps_stop_at_line",
    "ctps_task_events",
    "ctps_track",
    "ctps_two_goto",
    "ctps_unknown_ops",
]


def test_nc8_pre_existing_fixtures_load_unchanged():
    """NC8: the 8 pre-existing v1 fixtures load without error after the schema change.

    Key assertions:
    * ``schema_version == 1`` for every fixture.
    * Every ``drive`` command has ``mode``, ``heading``, and ``reverse``
      defaulting to ``None`` — the schema change is purely additive.
    """
    for name in _PRE_EXISTING_CTPS_FIXTURES:
        path = _FIXTURES / f"{name}.json"
        # Silence warnings locally (ctps_unknown_ops.json emits UserWarning by
        # design; we do not assert on them here — TC7 covers that).
        with warnings.catch_warnings():
            warnings.simplefilter("always")
            trace = load(path)
        assert trace.schema_version == 1, f"{name}: unexpected schema_version"
        for cmd in trace.commands:
            if cmd.op == "drive":
                assert cmd.mode is None, f"{name} seq={cmd.seq}: mode should be None"
                assert cmd.heading is None, f"{name} seq={cmd.seq}: heading should be None"
                assert cmd.reverse is None, f"{name} seq={cmd.seq}: reverse should be None"


# ---------------------------------------------------------------------------
# goto forced-direction tags (setArrivalDirection / setRouteDirection)
# ---------------------------------------------------------------------------


def test_goto_forced_direction_tags_parse():
    """A goto with forced arrival/route directions parses into ``Command``;
    a direction-free goto leaves both ``None`` (AUTO).

    Guards the loader stage of the CTPS direction-override pipeline: the C++
    recorder emits ``"arrival"``/``"route"`` only for a non-AUTO override, and the
    loader must surface them for ``track_builder`` to forward to the sim.
    """
    trace = load(
        {
            "schema_version": 1,
            "start_node": "A",
            "start_heading_deg": 0.0,
            "build_error": False,
            "commands": [
                {
                    "seq": 0,
                    "op": "goto",
                    "target": "B",
                    "arrival": "front",
                    "route": "rear",
                },
                {"seq": 1, "op": "goto", "target": "C"},
            ],
        }
    )
    forced, bare = trace.commands
    assert (forced.arrival, forced.route) == ("front", "rear")
    assert (bare.arrival, bare.route) == (None, None)


# ---------------------------------------------------------------------------
# T9 — new `drive` fields: power / preset / start_speed / end_speed /
#      profiled / brake
# ---------------------------------------------------------------------------


def test_drive_new_fields_all_present_parse():
    """All six new ``drive`` fields parse into their ``Command`` attributes.

    Field names must match the Contracts table exactly: ``power``,
    ``preset``, ``start_speed``, ``end_speed``, ``profiled``, ``brake``.
    None of them leak into ``Command.fields`` (they are named fields, not
    forward-compat extras).
    """
    trace = load(
        _drive_trace(
            {
                "seq": 0,
                "op": "drive",
                "method": "driveDist",
                "cm": 40.0,
                "power": 180,
                "preset": "fast",
                "start_speed": 60,
                "end_speed": 0,
                "profiled": True,
                "brake": False,
            }
        )
    )
    assert len(trace.commands) == 1
    cmd = trace.commands[0]
    assert cmd.power == 180
    assert cmd.preset == "fast"
    assert cmd.start_speed == 60
    assert cmd.end_speed == 0
    assert cmd.profiled is True
    assert cmd.brake is False
    assert cmd.fields == {}


def test_drive_new_fields_absent_default_to_none():
    """Absent new fields default to ``None`` — additive schema, byte-identical
    to a pre-existing trace with none of them.
    """
    trace = load(
        _drive_trace({"seq": 0, "op": "drive", "method": "driveDist", "cm": 10.0})
    )
    assert len(trace.commands) == 1
    cmd = trace.commands[0]
    assert cmd.power is None
    assert cmd.preset is None
    assert cmd.start_speed is None
    assert cmd.end_speed is None
    assert cmd.profiled is None
    assert cmd.brake is None


def test_drive_new_int_fields_reject_bool_and_bad_type():
    """``power``/``start_speed``/``end_speed`` reject non-int values AND a
    JSON boolean in the slot (``bool`` subclasses ``int`` in Python — the
    existing loader guards already special-case this so a stray
    ``true``/``false`` does not silently become ``1``/``0``). Each bad value
    warns and the field is treated as unset (``None``); the command survives.
    """
    # power: string value warns + unsets.
    data = _drive_trace(
        {"seq": 0, "op": "drive", "method": "driveDist", "cm": 10.0, "power": "fast"}
    )
    with pytest.warns(UserWarning, match="drive.power"):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].power is None

    # power: bool-as-int is rejected, not coerced to 1/0.
    data = _drive_trace(
        {"seq": 0, "op": "drive", "method": "driveDist", "cm": 10.0, "power": True}
    )
    with pytest.warns(UserWarning, match="drive.power"):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].power is None

    # start_speed: float value warns + unsets (must be int per Contracts).
    data = _drive_trace(
        {
            "seq": 0,
            "op": "drive",
            "method": "driveDist",
            "cm": 10.0,
            "start_speed": 12.5,
        }
    )
    with pytest.warns(UserWarning, match="drive.start_speed"):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].start_speed is None

    # start_speed: bool-as-int is rejected.
    data = _drive_trace(
        {
            "seq": 0,
            "op": "drive",
            "method": "driveDist",
            "cm": 10.0,
            "start_speed": False,
        }
    )
    with pytest.warns(UserWarning, match="drive.start_speed"):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].start_speed is None

    # end_speed: non-numeric value warns + unsets.
    data = _drive_trace(
        {
            "seq": 0,
            "op": "drive",
            "method": "driveDist",
            "cm": 10.0,
            "end_speed": None,
        }
    )
    with pytest.warns(UserWarning, match="drive.end_speed"):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].end_speed is None

    # end_speed: bool-as-int is rejected.
    data = _drive_trace(
        {
            "seq": 0,
            "op": "drive",
            "method": "driveDist",
            "cm": 10.0,
            "end_speed": True,
        }
    )
    with pytest.warns(UserWarning, match="drive.end_speed"):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].end_speed is None


def test_drive_preset_unknown_string_and_non_string_warn_and_unset():
    """``preset`` must be one of ``"slow"``/``"normal"``/``"fast"``; an
    unknown string or a non-string value warns and unsets (falls back to the
    NORMAL default at the resolver stage), mirroring the existing ``mode``
    guard.
    """
    # Unknown preset string.
    data = _drive_trace(
        {
            "seq": 0,
            "op": "drive",
            "method": "driveDist",
            "cm": 10.0,
            "preset": "turbo",
        }
    )
    with pytest.warns(UserWarning, match="drive.preset"):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].preset is None

    # Non-string preset (a number).
    data = _drive_trace(
        {"seq": 0, "op": "drive", "method": "driveDist", "cm": 10.0, "preset": 3}
    )
    with pytest.warns(UserWarning, match="drive.preset"):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].preset is None


def test_drive_profiled_and_brake_reject_non_bool():
    """``profiled``/``brake`` accept ONLY a JSON boolean (never
    ``bool(raw.get(...))``, mirroring the ``reverse`` guard) — a truthy
    non-bool (e.g. the string ``"true"``) must not silently coerce. Each
    warns and the field is treated as unset (``None``), leaving the
    resolver's builder-default rule in force.
    """
    data = _drive_trace(
        {
            "seq": 0,
            "op": "drive",
            "method": "driveDist",
            "cm": 10.0,
            "profiled": "true",
        }
    )
    with pytest.warns(UserWarning, match="drive.profiled"):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].profiled is None

    data = _drive_trace(
        {"seq": 0, "op": "drive", "method": "driveDist", "cm": 10.0, "brake": 1}
    )
    with pytest.warns(UserWarning, match="drive.brake"):
        trace = load(data)
    assert len(trace.commands) == 1
    assert trace.commands[0].brake is None


def test_drive_new_fields_do_not_break_pre_existing_fixtures():
    """A drive command with none of the new fields loads with all six
    defaulting to ``None`` — confirms the pre-existing fixtures remain
    byte-identical after the schema addition (spot check via
    ``ctps_drive_no_drift.json``, one of the NC8 fixtures).
    """
    trace = load(_FIXTURES / "ctps_drive_no_drift.json")
    for cmd in trace.commands:
        if cmd.op == "drive":
            assert cmd.power is None
            assert cmd.preset is None
            assert cmd.start_speed is None
            assert cmd.end_speed is None
            assert cmd.profiled is None
            assert cmd.brake is None


# ---------------------------------------------------------------------------
# T9 — `declare_position` op
# ---------------------------------------------------------------------------


def test_declare_position_parses():
    """``declare_position`` is a recognized op; its ``node`` field parses
    into ``Command.node`` and the command is NOT dropped.
    """
    trace = load(
        {
            "schema_version": 1,
            "start_node": "A",
            "start_heading_deg": 0.0,
            "build_error": False,
            "commands": [
                {"seq": 0, "op": "declare_position", "node": "B"},
            ],
        }
    )
    assert len(trace.commands) == 1
    cmd = trace.commands[0]
    assert cmd.op == "declare_position"
    assert cmd.node == "B"
    assert cmd.fields == {}


def test_declare_position_missing_node_warns_and_skips():
    """A ``declare_position`` with no ``node`` key warns and is dropped
    entirely — it carries no usable payload for ``track_builder``.
    """
    data = {
        "schema_version": 1,
        "start_node": "A",
        "start_heading_deg": 0.0,
        "build_error": False,
        "commands": [
            {"seq": 0, "op": "declare_position"},
            {"seq": 1, "op": "goto", "target": "B"},
        ],
    }
    with pytest.warns(UserWarning, match="declare_position.node"):
        trace = load(data)
    # The malformed declare_position is skipped; the following goto survives.
    assert len(trace.commands) == 1
    assert trace.commands[0].op == "goto"


def test_declare_position_non_string_node_warns_and_skips():
    """A ``declare_position`` whose ``node`` is not a string (e.g. a number)
    warns and is dropped, same as a missing ``node``.
    """
    data = {
        "schema_version": 1,
        "start_node": "A",
        "start_heading_deg": 0.0,
        "build_error": False,
        "commands": [
            {"seq": 0, "op": "declare_position", "node": 42},
        ],
    }
    with pytest.warns(UserWarning, match="declare_position.node"):
        trace = load(data)
    assert len(trace.commands) == 0


def test_declare_position_empty_string_node_warns_and_skips():
    """An empty-string ``node`` is treated as malformed (no node name to
    declare against) — warns and is dropped.
    """
    data = {
        "schema_version": 1,
        "start_node": "A",
        "start_heading_deg": 0.0,
        "build_error": False,
        "commands": [
            {"seq": 0, "op": "declare_position", "node": ""},
        ],
    }
    with pytest.warns(UserWarning, match="declare_position.node"):
        trace = load(data)
    assert len(trace.commands) == 0


# ---------------------------------------------------------------------------
# T20 — top-level `mosaic` block + `read_mosaic` op (TC10 valid / TC11 malformed)
# ---------------------------------------------------------------------------


def _mosaic_trace(mosaic: dict, *cmds: dict) -> dict:
    """Return a minimal v1 trace dict carrying a top-level ``mosaic`` block."""
    return {
        "schema_version": 1,
        "start_node": "A",
        "start_heading_deg": 0.0,
        "build_error": False,
        "commands": list(cmds),
        "mosaic": mosaic,
    }


_VALID_MOSAIC_CELLS = [
    "YELLOW", "BLUE", "GREEN", "WHITE",
    "YELLOW", "BLUE", "GREEN", "WHITE",
    "YELLOW", "BLUE", "GREEN", "WHITE",
]


def test_tc10_valid_mosaic_and_read_mosaic_op_load_no_warning():
    """TC10: a well-formed top-level ``mosaic`` block + a ``read_mosaic`` op
    load onto the ``Trace`` with NO warning.

    Key assertions:
    * ``trace.mosaic`` is populated: ``cells`` (12, in order), ``seed``,
      ``geom_hash``, ``injected`` all match the input.
    * the ``read_mosaic`` command survives in ``trace.commands`` with
      ``success is True``.
    * no :class:`UserWarning` is raised for a fully well-formed trace.
    """
    data = _mosaic_trace(
        {
            "cells": _VALID_MOSAIC_CELLS,
            "seed": 12345,
            "geom_hash": "ab12cd34",
            "injected": True,
        },
        {"seq": 0, "op": "read_mosaic", "success": True},
    )

    with warnings.catch_warnings():
        warnings.simplefilter("error")  # promote any UserWarning to a failure
        trace = load(data)

    assert trace.mosaic is not None
    assert trace.mosaic.cells == _VALID_MOSAIC_CELLS
    assert trace.mosaic.seed == 12345
    assert trace.mosaic.geom_hash == "ab12cd34"
    assert trace.mosaic.injected is True

    assert len(trace.commands) == 1
    read_cmd = trace.commands[0]
    assert read_cmd.op == "read_mosaic"
    assert read_cmd.success is True


def test_tc10_valid_mosaic_seed_absent_no_warning():
    """TC10 (seed-absent variant): a ``mosaic`` block with no ``seed`` key
    loads with ``trace.mosaic.seed is None`` and NO warning — an absent seed
    is a normal case (the dump omits the key when ``--mosaic-seed`` wasn't
    passed), not a malformed one.
    """
    data = _mosaic_trace(
        {
            "cells": _VALID_MOSAIC_CELLS,
            "geom_hash": "ab12cd34",
            "injected": True,
        },
        {"seq": 0, "op": "read_mosaic", "success": False},
    )

    with warnings.catch_warnings():
        warnings.simplefilter("error")
        trace = load(data)

    assert trace.mosaic is not None
    assert trace.mosaic.seed is None
    assert trace.mosaic.cells == _VALID_MOSAIC_CELLS
    assert trace.mosaic.geom_hash == "ab12cd34"
    assert trace.mosaic.injected is True

    assert len(trace.commands) == 1
    assert trace.commands[0].success is False


@pytest.mark.parametrize(
    "bad_mosaic",
    [
        pytest.param("not-a-dict", id="mosaic-not-a-dict"),
        pytest.param(["a", "list"], id="mosaic-is-a-list"),
        pytest.param(
            {
                "cells": _VALID_MOSAIC_CELLS[:11],  # wrong length (11, not 12)
                "geom_hash": "ab12cd34",
                "injected": True,
            },
            id="cells-wrong-length",
        ),
        pytest.param(
            {
                "cells": [*_VALID_MOSAIC_CELLS[:11], 42],  # non-string entry
                "geom_hash": "ab12cd34",
                "injected": True,
            },
            id="cells-non-string-entry",
        ),
        pytest.param(
            {
                "cells": _VALID_MOSAIC_CELLS,
                "geom_hash": "",  # empty string
                "injected": True,
            },
            id="geom-hash-empty-string",
        ),
        pytest.param(
            {
                "cells": _VALID_MOSAIC_CELLS,
                "geom_hash": 12345,  # non-string
                "injected": True,
            },
            id="geom-hash-non-string",
        ),
        pytest.param(
            {
                "cells": _VALID_MOSAIC_CELLS,
                "geom_hash": "ab12cd34",
                "injected": "true",  # non-bool
            },
            id="injected-non-bool",
        ),
        pytest.param(
            {
                "cells": _VALID_MOSAIC_CELLS,
                "geom_hash": "ab12cd34",
                "injected": True,
                "seed": 3.5,  # non-int
            },
            id="seed-non-int-float",
        ),
        pytest.param(
            {
                "cells": _VALID_MOSAIC_CELLS,
                "geom_hash": "ab12cd34",
                "injected": True,
                "seed": True,  # JSON bool masquerading as int
            },
            id="seed-bool-as-int",
        ),
    ],
)
def test_tc11_malformed_mosaic_block_warns_and_skips_all_or_nothing(bad_mosaic):
    """TC11: a malformed top-level ``mosaic`` block warns and is skipped
    entirely — ``trace.mosaic is None`` (all-or-nothing, never half-parsed),
    and the loader NEVER raises.
    """
    data = _mosaic_trace(bad_mosaic)

    with pytest.warns(UserWarning):
        trace = load(data)

    assert trace.mosaic is None


def test_tc11_read_mosaic_success_missing_kept_with_success_none():
    """TC11: a ``read_mosaic`` op with a missing ``success`` key warns, but
    the command is KEPT (not dropped) with ``success is None`` — only the
    top-level ``mosaic`` block is all-or-nothing; individual ops degrade
    field-by-field like every other op in this loader.
    """
    data = {
        "schema_version": 1,
        "start_node": "A",
        "start_heading_deg": 0.0,
        "build_error": False,
        "commands": [
            {"seq": 0, "op": "read_mosaic"},
        ],
    }

    with pytest.warns(UserWarning, match="read_mosaic.success"):
        trace = load(data)

    assert len(trace.commands) == 1
    cmd = trace.commands[0]
    assert cmd.op == "read_mosaic"
    assert cmd.success is None


def test_tc11_read_mosaic_success_non_bool_warns_and_kept_with_success_none():
    """TC11: a ``read_mosaic`` op whose ``success`` is a non-bool (e.g. a
    JSON int, mirroring the ``bool`` subclasses ``int`` guard used
    throughout this loader) warns and is treated as unset — the command is
    still kept.
    """
    data = {
        "schema_version": 1,
        "start_node": "A",
        "start_heading_deg": 0.0,
        "build_error": False,
        "commands": [
            {"seq": 0, "op": "read_mosaic", "success": 1},
        ],
    }

    with pytest.warns(UserWarning, match="read_mosaic.success"):
        trace = load(data)

    assert len(trace.commands) == 1
    cmd = trace.commands[0]
    assert cmd.op == "read_mosaic"
    assert cmd.success is None


def test_tc11_loader_never_raises_on_any_malformed_mosaic_input():
    """TC11 (never-raises guard): every malformed-mosaic case above — plus a
    malformed ``read_mosaic`` op alongside it — is handled purely with
    warnings; ``load`` never raises for any of them.
    """
    data = _mosaic_trace(
        {"cells": "not-a-list", "geom_hash": 7, "injected": None, "seed": "nope"},
        {"seq": 0, "op": "read_mosaic", "success": "yes"},
    )

    with warnings.catch_warnings():
        warnings.simplefilter("always")
        trace = load(data)  # must not raise

    assert trace.mosaic is None
    assert len(trace.commands) == 1
    assert trace.commands[0].success is None


# ---------------------------------------------------------------------------
# TC20 (#154-20) — mosaic loader colour-name whitelist
# ---------------------------------------------------------------------------


def test_tc20_unknown_cell_color_warns_once_and_rejects_whole_block():
    """TC20: a ``mosaic.cells`` entry with one unknown colour name warns
    exactly once and sets ``trace.mosaic`` to ``None`` — never a partial
    11-cell mosaic, preserving the fixed 12-cell index contract.
    """
    bad_cells = list(_VALID_MOSAIC_CELLS)
    bad_cells[5] = "MAGENTA"  # not in COLOR_NAMES or AIR_NAME
    data = _mosaic_trace(
        {
            "cells": bad_cells,
            "geom_hash": "ab12cd34",
            "injected": True,
        }
    )

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        trace = load(data)

    mosaic_warnings = [w for w in caught if issubclass(w.category, UserWarning)]
    assert len(mosaic_warnings) == 1
    assert "mosaic.cells" in str(mosaic_warnings[0].message)
    assert trace.mosaic is None


def test_tc20_valid_block_with_air_sentinel_cells_passes_untouched():
    """TC20 (valid variant): a ``mosaic.cells`` block mixing real colour
    names with the AIR sentinel (:data:`mosaic.AIR_NAME`) is a fully valid
    block — it loads with NO warning and ``trace.mosaic`` populated
    unchanged.
    """
    cells_with_air = [
        "YELLOW", "BLUE", "GREEN", "WHITE",
        mosaic.AIR_NAME, mosaic.AIR_NAME, "RED", "YELLOW",
        "BLUE", "GREEN", "WHITE", mosaic.AIR_NAME,
    ]
    data = _mosaic_trace(
        {
            "cells": cells_with_air,
            "geom_hash": "ab12cd34",
            "injected": True,
        }
    )

    with warnings.catch_warnings():
        warnings.simplefilter("error")  # promote any UserWarning to a failure
        trace = load(data)

    assert trace.mosaic is not None
    assert trace.mosaic.cells == cells_with_air
