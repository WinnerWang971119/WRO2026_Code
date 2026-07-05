"""Tests for tools.reorg.mapping -- the file -> destination mapping table.

Proves mapping completeness: all root files classified, and every entry
matches the spec's "File -> destination mapping" table exactly (including the
two named exceptions: TurnSignPolicy.h -> drive, and the other four resolver
policies + routes/FieldMap/NavTypes/Localization/YawSource -> nav).

The spec's Contracts table (authored against the branch tip when written)
lists 38 movers / 45 total. This suite pins 39 movers / 46 total instead --
`TickProfiler.h` landed on this branch afterward via the already-merged PR
#124 and is classified into `src/util/` here (see `mapping.py`'s module note
for the full rationale). `test_45_root_files_matches_the_real_current_tree`
below is the drift detector: it fails loudly the moment the real tree and
this table disagree again.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tools.reorg import mapping


def test_mapping_has_exactly_46_entries() -> None:
    assert len(mapping.MAPPING) == 46


def test_exactly_39_move_and_7_stay() -> None:
    movers = [name for name, domain in mapping.MAPPING.items() if domain is not None]
    stayers = [name for name, domain in mapping.MAPPING.items() if domain is None]
    assert len(movers) == 39
    assert len(stayers) == 7


def test_root_stay_set_matches_spec() -> None:
    assert mapping.ROOT_STAY_FILES == {
        "WRO2026.ino",
        "Config.h",
        "MissionScript.h",
        "MissionScript.cpp",
        "Tasks.h",
        "Tasks.cpp",
        "TasksTypes.h",
    }
    for name in mapping.ROOT_STAY_FILES:
        assert mapping.domain_of(name) is None
        assert mapping.is_root_stay(name)
        assert not mapping.is_mover(name)


def test_config_subheaders_are_a_disjoint_set_of_8() -> None:
    assert len(mapping.CONFIG_SUBHEADERS) == 8
    assert mapping.CONFIG_SUBHEADERS.isdisjoint(mapping.MAPPING)
    for name in mapping.CONFIG_SUBHEADERS:
        assert mapping.is_config_subheader(name)


@pytest.mark.parametrize(
    "basename,expected_domain",
    [
        # src/drive -- 7, including the TurnSignPolicy.h exception.
        ("Drive.h", "drive"),
        ("Drive.cpp", "drive"),
        ("LineFollower.h", "drive"),
        ("LineFollower.cpp", "drive"),
        ("Intersection.h", "drive"),
        ("Intersection.cpp", "drive"),
        ("TurnSignPolicy.h", "drive"),
        # src/perception -- 5.
        ("WROLineArray.h", "perception"),
        ("WROLineArray.cpp", "perception"),
        ("Vision.h", "perception"),
        ("Vision.cpp", "perception"),
        ("VisionFrame.h", "perception"),
        # src/nav -- 13 (routes + FieldMap/NavTypes/Localization/YawSource + 4 other policies).
        ("routes.h", "nav"),
        ("routes.cpp", "nav"),
        ("FieldMap.h", "nav"),
        ("NavTypes.h", "nav"),
        ("Localization.h", "nav"),
        ("Localization.cpp", "nav"),
        ("YawSource.h", "nav"),
        ("YawSource.cpp", "nav"),
        ("DirectionPolicy.h", "nav"),
        ("StopAtLinePolicy.h", "nav"),
        ("EdgeDispatchPolicy.h", "nav"),
        ("PivotTrimPolicy.h", "nav"),
        ("ReturnToNodePolicy.h", "nav"),
        # src/mission -- 5.
        ("Mission.h", "mission"),
        ("Mission.cpp", "mission"),
        ("PathTracker.h", "mission"),
        ("PathTracker.cpp", "mission"),
        ("PathTrackerParityTrace.h", "mission"),
        # src/manip -- 4.
        ("Manipulator.h", "manip"),
        ("Manipulator.cpp", "manip"),
        ("ManipServo.h", "manip"),
        ("ManipBelt.h", "manip"),
        # src/util -- 5 (includes TickProfiler.h; see the module note).
        ("WROMath.h", "util"),
        ("StateMachine.h", "util"),
        ("EventTrace.h", "util"),
        ("EventTrace.cpp", "util"),
        ("TickProfiler.h", "util"),
    ],
)
def test_framework_file_domain(basename: str, expected_domain: str) -> None:
    assert mapping.domain_of(basename) == expected_domain
    assert mapping.is_mover(basename)


def test_domain_counts_per_bucket() -> None:
    counts = {domain: 0 for domain in mapping.DOMAINS}
    for domain in mapping.MAPPING.values():
        if domain is not None:
            counts[domain] += 1
    assert counts == {
        "drive": 7,
        "perception": 5,
        "nav": 13,
        "mission": 5,
        "manip": 4,
        "util": 5,
    }


def test_domain_of_raises_for_untracked_name() -> None:
    with pytest.raises(KeyError):
        mapping.domain_of("SomeFileNotInTheMapping.h")


def test_source_and_destination_paths() -> None:
    assert mapping.source_repo_path("Drive.cpp") == Path("WRO2026/Drive.cpp")
    assert mapping.destination_repo_path("Drive.cpp") == Path("WRO2026/src/drive/Drive.cpp")
    assert mapping.destination_repo_path("Config.h") == Path("WRO2026/Config.h")
    assert mapping.destination_repo_path("TurnSignPolicy.h") == Path(
        "WRO2026/src/drive/TurnSignPolicy.h"
    )


def test_46_root_files_matches_the_real_current_tree(sketch_root: Path) -> None:
    """The mapping's 46 classified names must equal the flat root listing today.

    This is the strongest possible completeness check: it fails loudly the
    moment the real tree and the hand-authored mapping table drift apart
    (a new root file added without updating the table, or vice versa).
    """
    if (sketch_root / "src").is_dir():
        pytest.skip("pre-move only: this snapshots the flat pre-move tree; the reorg move has run")
    actual_root_files = {
        entry.name
        for entry in sketch_root.iterdir()
        if entry.is_file() and entry.suffix in (".h", ".cpp", ".ino")
    }
    assert actual_root_files == set(mapping.MAPPING)


def test_8_config_subheaders_matches_the_real_current_tree(sketch_root: Path) -> None:
    actual_config_files = {
        entry.name
        for entry in (sketch_root / "config").iterdir()
        if entry.is_file() and entry.suffix == ".h"
    }
    assert actual_config_files == set(mapping.CONFIG_SUBHEADERS)
