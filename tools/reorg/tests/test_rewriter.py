"""Tests for tools.reorg.rewriter -- the file-walking orchestration layer.

Two kinds of test here:

* Against the REAL repo tree (read-only: only `rewrite_*_tree` is called,
  never `apply_diffs` with `check=False`) -- proves the old-layout no-op
  property end-to-end over actual files, and that generated fixtures are
  never touched by the test-tree rewriter.
* Against a synthetic `tmp_path` tree built to look like the POST-MOVE layout
  -- proves `rewrite_production_tree(layout="new")` + `apply_diffs` actually
  writes the correct rewritten content, and that a second pass is a no-op
  (idempotency of the whole pipeline, not just the line-level function).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tools.reorg import mapping
from tools.reorg import rewriter as rw
from tools.reorg.test_includes import GENERATED_FILE_EXCLUSIONS


# ---------------------------------------------------------------------------
# Read-only checks against the real repo.
# ---------------------------------------------------------------------------


def test_rewrite_production_tree_old_layout_is_a_no_op_over_the_real_tree(sketch_root: Path) -> None:
    if (sketch_root / "src").is_dir():
        pytest.skip("pre-move only: this snapshots the flat pre-move tree; the reorg move has run")
    diffs = rw.rewrite_production_tree(sketch_root, layout="old")
    changed = [d for d in diffs if d.changed]
    assert changed == []
    # Sanity: we actually walked a non-trivial number of files (46 root + 8 config).
    assert len(diffs) == 54


def test_rewrite_production_tree_old_layout_never_writes(sketch_root: Path) -> None:
    """Computing diffs must never touch disk -- only apply_diffs(check=False) may."""
    before = {
        p: p.read_text(encoding="utf-8")
        for p in sketch_root.iterdir()
        if p.is_file() and p.suffix in (".h", ".cpp", ".ino")
    }
    rw.rewrite_production_tree(sketch_root, layout="old")
    after = {
        p: p.read_text(encoding="utf-8")
        for p in sketch_root.iterdir()
        if p.is_file() and p.suffix in (".h", ".cpp", ".ino")
    }
    assert before == after


def test_rewrite_test_tree_never_touches_generated_exclusions(test_root: Path) -> None:
    diffs = rw.rewrite_test_tree(test_root)
    touched_relpaths = {d.path.relative_to(test_root).as_posix() for d in diffs}
    assert touched_relpaths.isdisjoint(GENERATED_FILE_EXCLUSIONS)


def test_rewrite_test_tree_finds_the_expected_real_rewrites(sketch_root: Path, test_root: Path) -> None:
    if (sketch_root / "src").is_dir():
        pytest.skip("pre-move only: this snapshots the flat pre-move tree; the reorg move has run")
    diffs = rw.rewrite_test_tree(test_root)
    changed_by_relpath = {
        d.path.relative_to(test_root).as_posix(): d.rewritten for d in diffs if d.changed
    }
    assert "test_manipulator.cpp" in changed_by_relpath
    assert 'src/manip/Manipulator.h' in changed_by_relpath["test_manipulator.cpp"]
    assert "test_path_tracker_harness.h" in changed_by_relpath
    harness_rewritten = changed_by_relpath["test_path_tracker_harness.h"]
    assert 'src/drive/Drive.h' in harness_rewritten
    assert 'src/nav/Localization.h' in harness_rewritten
    assert 'src/mission/PathTracker.h' in harness_rewritten


# ---------------------------------------------------------------------------
# Synthetic post-move tree: apply_diffs actually writes; a second pass is a no-op.
# ---------------------------------------------------------------------------


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _build_new_layout_tree(tmp_path: Path) -> Path:
    """A minimal but representative post-move WRO2026/ tree under tmp_path."""
    sketch = tmp_path / "WRO2026"

    # Root-stay files.
    _write(sketch / "WRO2026.ino", '#include "Drive.h"\n#include "Config.h"\n')
    _write(sketch / "Config.h", '#include "config/ConfigDrive.h"\n')
    _write(sketch / "MissionScript.cpp", '#include "MissionScript.h"\n#include "Mission.h"\n')
    _write(sketch / "MissionScript.h", "// empty\n")
    _write(sketch / "Tasks.cpp", '#include "Tasks.h"\n#include "Manipulator.h"\n')
    _write(sketch / "Tasks.h", '#include "TasksTypes.h"\n')
    _write(sketch / "TasksTypes.h", '#include "Config.h"\n')

    # config/ sub-headers.
    _write(sketch / "config" / "ConfigDrive.h", "// tunables\n")
    for name in mapping.CONFIG_SUBHEADERS - {"ConfigDrive.h"}:
        _write(sketch / "config" / name, "// tunables\n")

    # src/drive movers.
    _write(
        sketch / "src" / "drive" / "Drive.cpp",
        '#include "Drive.h"\n#include "TurnSignPolicy.h"\n#include "YawSource.h"\n',
    )
    _write(sketch / "src" / "drive" / "Drive.h", '#include "Config.h"\n#include "WROMath.h"\n')
    _write(sketch / "src" / "drive" / "TurnSignPolicy.h", "// resolver\n")
    _write(sketch / "src" / "drive" / "LineFollower.h", "// stub\n")
    _write(sketch / "src" / "drive" / "LineFollower.cpp", '#include "LineFollower.h"\n')
    _write(sketch / "src" / "drive" / "Intersection.h", "// stub\n")
    _write(sketch / "src" / "drive" / "Intersection.cpp", '#include "Intersection.h"\n')

    # src/perception movers.
    _write(sketch / "src" / "perception" / "WROLineArray.h", '#include "Config.h"\n')
    _write(sketch / "src" / "perception" / "WROLineArray.cpp", '#include "WROLineArray.h"\n')
    _write(sketch / "src" / "perception" / "Vision.h", '#include "config/ConfigVision.h"\n')
    _write(sketch / "src" / "perception" / "Vision.cpp", '#include "Vision.h"\n')
    _write(sketch / "src" / "perception" / "VisionFrame.h", '#include "config/ConfigVision.h"\n')

    # src/nav movers (minimal set for this synthetic tree).
    _write(sketch / "src" / "nav" / "YawSource.h", "// stub\n")
    _write(sketch / "src" / "nav" / "YawSource.cpp", '#include "YawSource.h"\n#include "Config.h"\n')
    _write(sketch / "src" / "nav" / "FieldMap.h", '#include "routes.h"\n')
    _write(sketch / "src" / "nav" / "NavTypes.h", "// stub\n")
    _write(sketch / "src" / "nav" / "Localization.h", '#include "FieldMap.h"\n#include "routes.h"\n')
    _write(sketch / "src" / "nav" / "Localization.cpp", '#include "Localization.h"\n#include "Drive.h"\n')
    _write(sketch / "src" / "nav" / "routes.h", "// stub\n")
    _write(sketch / "src" / "nav" / "routes.cpp", '#include "routes.h"\n')
    _write(sketch / "src" / "nav" / "DirectionPolicy.h", '#include "Config.h"\n#include "NavTypes.h"\n#include "routes.h"\n')
    _write(sketch / "src" / "nav" / "StopAtLinePolicy.h", "// stub\n")
    _write(sketch / "src" / "nav" / "EdgeDispatchPolicy.h", "// stub\n")
    _write(sketch / "src" / "nav" / "PivotTrimPolicy.h", "// stub\n")
    _write(sketch / "src" / "nav" / "ReturnToNodePolicy.h", "// stub\n")

    # src/mission movers.
    _write(sketch / "src" / "mission" / "Mission.h", '#include "Config.h"\n#include "NavTypes.h"\n#include "Tasks.h"\n')
    _write(
        sketch / "src" / "mission" / "Mission.cpp",
        '#include "Mission.h"\n#include "Drive.h"\n#include "FieldMap.h"\n#include "PathTracker.h"\n',
    )
    _write(sketch / "src" / "mission" / "PathTracker.h", '#include "FieldMap.h"\n')
    _write(sketch / "src" / "mission" / "PathTracker.cpp", '#include "PathTracker.h"\n#include "Drive.h"\n')
    _write(sketch / "src" / "mission" / "PathTrackerParityTrace.h", "// stub\n")

    # src/manip movers.
    _write(sketch / "src" / "manip" / "Manipulator.h", '#include "Config.h"\n#include "ManipServo.h"\n#include "ManipBelt.h"\n')
    _write(sketch / "src" / "manip" / "Manipulator.cpp", '#include "Manipulator.h"\n')
    _write(sketch / "src" / "manip" / "ManipServo.h", '#include "config/ConfigManipulator.h"\n')
    _write(sketch / "src" / "manip" / "ManipBelt.h", '#include "config/ConfigManipulator.h"\n')

    # src/util movers.
    _write(sketch / "src" / "util" / "WROMath.h", '#include "Config.h"\n')
    _write(sketch / "src" / "util" / "StateMachine.h", "// stub\n")
    _write(sketch / "src" / "util" / "EventTrace.h", '#include "Config.h"\n')
    _write(
        sketch / "src" / "util" / "EventTrace.cpp",
        '#include "EventTrace.h"\n#include "Drive.h"\n#include "FieldMap.h"\n#include "Localization.h"\n#include "routes.h"\n',
    )

    return sketch


def test_new_layout_synthetic_tree_rewrites_correctly(tmp_path: Path) -> None:
    sketch = _build_new_layout_tree(tmp_path)
    diffs = rw.rewrite_production_tree(sketch, layout="new")
    by_path = {d.path: d for d in diffs}

    ino_diff = by_path[sketch / "WRO2026.ino"]
    assert ino_diff.rewritten == '#include "src/drive/Drive.h"\n#include "Config.h"\n'

    drive_cpp_diff = by_path[sketch / "src" / "drive" / "Drive.cpp"]
    assert drive_cpp_diff.rewritten == (
        '#include "Drive.h"\n#include "TurnSignPolicy.h"\n#include "../nav/YawSource.h"\n'
    )

    drive_h_diff = by_path[sketch / "src" / "drive" / "Drive.h"]
    assert drive_h_diff.rewritten == '#include "../../Config.h"\n#include "../util/WROMath.h"\n'

    manip_servo_diff = by_path[sketch / "src" / "manip" / "ManipServo.h"]
    assert manip_servo_diff.rewritten == '#include "../../config/ConfigManipulator.h"\n'

    mission_cpp_diff = by_path[sketch / "src" / "mission" / "Mission.cpp"]
    assert mission_cpp_diff.rewritten == (
        '#include "Mission.h"\n#include "../drive/Drive.h"\n#include "../nav/FieldMap.h"\n#include "PathTracker.h"\n'
    )

    mission_script_diff = by_path[sketch / "MissionScript.cpp"]
    assert mission_script_diff.rewritten == '#include "MissionScript.h"\n#include "src/mission/Mission.h"\n'

    tasks_cpp_diff = by_path[sketch / "Tasks.cpp"]
    assert tasks_cpp_diff.rewritten == '#include "Tasks.h"\n#include "src/manip/Manipulator.h"\n'


def test_apply_diffs_writes_then_second_pass_is_a_no_op(tmp_path: Path) -> None:
    sketch = _build_new_layout_tree(tmp_path)
    first_diffs = rw.rewrite_production_tree(sketch, layout="new")
    changed_paths = rw.apply_diffs(first_diffs, check=False)
    assert changed_paths, "sanity: the first pass should have changed something"

    second_diffs = rw.rewrite_production_tree(sketch, layout="new")
    still_changed = [d for d in second_diffs if d.changed]
    assert still_changed == [], "a second rewrite pass must be a no-op (idempotency)"


def test_apply_diffs_check_mode_reports_without_writing(tmp_path: Path) -> None:
    sketch = _build_new_layout_tree(tmp_path)
    original_text = (sketch / "WRO2026.ino").read_text(encoding="utf-8")
    diffs = rw.rewrite_production_tree(sketch, layout="new")
    changed_paths = rw.apply_diffs(diffs, check=True)
    assert (sketch / "WRO2026.ino") in changed_paths
    assert (sketch / "WRO2026.ino").read_text(encoding="utf-8") == original_text
