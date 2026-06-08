"""Hermetic tests for the CTPS command-stream loader (T10: TC7).

Exercises :func:`tools.path_planner.ctps.trace_loader.load` against a
hand-authored Data Model A command stream that mixes recognized commands with
an unknown ``op`` and a ``drive`` carrying an unknown ``method``. No compiler,
no sim — pure parse/validate.

Covered case (CTPS plan, T10):

* **TC7** — an unknown ``op`` and an unknown ``drive.method`` each warn with a
  :class:`UserWarning`, do NOT raise, are skipped, and leave every recognized
  command intact so the simulator degrades gracefully when the firmware adds
  ops ahead of the Python side.
"""

from __future__ import annotations

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
