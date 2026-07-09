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
"""

from __future__ import annotations

import warnings
from pathlib import Path

import pytest

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
