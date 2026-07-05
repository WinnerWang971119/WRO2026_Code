"""Single source-of-truth file -> destination mapping (WRO2026 sketch-src reorg, issue #66).

See ``docs/plans/2026-07-05-sketch-src-reorg.md``, section "File -> destination
mapping (the single source of truth)" -- this module is a direct transcription of
that table. Every downstream reorg task (T5 production move, T6 test-include
rewrite, T7 CMake source paths, ...) must consume :data:`MAPPING` rather than
hand-deriving destination paths itself.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, FrozenSet, Optional

#: The name of the Arduino sketch directory at the repo root.
SKETCH_ROOT_NAME = "WRO2026"

#: The six framework domains under ``WRO2026/src/``.
DOMAINS: FrozenSet[str] = frozenset({"drive", "perception", "nav", "mission", "manip", "util"})

#: Operator-editable files that stay at the flat ``WRO2026/`` sketch root -- these
#: are the tunables + mission + task registration the operator edits during
#: competition, never moved by this migration.
ROOT_STAY_FILES: FrozenSet[str] = frozenset({
    "WRO2026.ino",
    "Config.h",
    "MissionScript.h",
    "MissionScript.cpp",
    "Tasks.h",
    "Tasks.cpp",
    "TasksTypes.h",
})

#: The 8 ``config/`` sub-headers. They already live in their own ``WRO2026/config/``
#: subdirectory today and stay there untouched -- they are NOT part of the 45-file
#: root classification below (that set is exactly the files sitting directly in the
#: flat ``WRO2026/`` listing today; ``config/`` is a subdirectory, not a root file).
CONFIG_SUBHEADERS: FrozenSet[str] = frozenset({
    "ConfigChecks.h",
    "ConfigDrive.h",
    "ConfigGlobal.h",
    "ConfigLine.h",
    "ConfigManipulator.h",
    "ConfigMission.h",
    "ConfigNav.h",
    "ConfigVision.h",
})

#: The framework files -> destination domain (Contracts: "File -> destination
#: mapping"). Note the two call-outs from Decisions: ``TurnSignPolicy.h`` is
#: Drive-only and lives in ``src/drive/`` (NOT alongside the other four resolver
#: policies in ``src/nav/``); the other four policies + routes/FieldMap/NavTypes/
#: Localization/YawSource all live in ``src/nav/``.
#:
#: The Contracts table (authored against the branch tip at the time) lists 38
#: files. ``TickProfiler.h`` landed on this branch afterward via the already-merged
#: PR #124 (a header-only, ``<stdint.h>``-only min/avg/max timing accumulator for
#: the `cooperativeTick()` bench, host-tested by ``test_tick_profiler.cpp``, no mock
#: counterpart anywhere under ``WRO2026/test/``) -- it is a framework file by every
#: measure the Contracts table uses (not operator-editable mission/tunable
#: surface), so it is classified here as the 39th mover, alongside ``WROMath.h`` /
#: ``StateMachine.h`` in ``src/util/``. Leaving it unmapped would orphan it at the
#: flat root after T5, violating AC8 ("every other framework module lives under
#: src/<domain>/"). See the T1 implementation report for the full rationale.
_FRAMEWORK_MAPPING: Dict[str, str] = {
    # src/drive -- 7 files.
    "Drive.h": "drive",
    "Drive.cpp": "drive",
    "LineFollower.h": "drive",
    "LineFollower.cpp": "drive",
    "Intersection.h": "drive",
    "Intersection.cpp": "drive",
    "TurnSignPolicy.h": "drive",
    # src/perception -- 5 files.
    "WROLineArray.h": "perception",
    "WROLineArray.cpp": "perception",
    "Vision.h": "perception",
    "Vision.cpp": "perception",
    "VisionFrame.h": "perception",
    # src/nav -- 13 files.
    "routes.h": "nav",
    "routes.cpp": "nav",
    "FieldMap.h": "nav",
    "NavTypes.h": "nav",
    "Localization.h": "nav",
    "Localization.cpp": "nav",
    "YawSource.h": "nav",
    "YawSource.cpp": "nav",
    "DirectionPolicy.h": "nav",
    "StopAtLinePolicy.h": "nav",
    "EdgeDispatchPolicy.h": "nav",
    "PivotTrimPolicy.h": "nav",
    "ReturnToNodePolicy.h": "nav",
    # src/mission -- 5 files.
    "Mission.h": "mission",
    "Mission.cpp": "mission",
    "PathTracker.h": "mission",
    "PathTracker.cpp": "mission",
    "PathTrackerParityTrace.h": "mission",
    # src/manip -- 4 files.
    "Manipulator.h": "manip",
    "Manipulator.cpp": "manip",
    "ManipServo.h": "manip",
    "ManipBelt.h": "manip",
    # src/util -- 5 files (includes TickProfiler.h -- see the module note above).
    "WROMath.h": "util",
    "StateMachine.h": "util",
    "EventTrace.h": "util",
    "EventTrace.cpp": "util",
    "TickProfiler.h": "util",
}

assert len(_FRAMEWORK_MAPPING) == 39, (
    f"expected exactly 39 framework files in the mapping (38 from the Contracts table + "
    f"TickProfiler.h), got {len(_FRAMEWORK_MAPPING)}"
)
assert set(_FRAMEWORK_MAPPING.values()) == set(DOMAINS), "every domain must have >=1 file, and no stray domain names"
assert not (set(_FRAMEWORK_MAPPING) & ROOT_STAY_FILES), "a file cannot both move and stay"
assert not (set(_FRAMEWORK_MAPPING) & CONFIG_SUBHEADERS), "config sub-headers are a disjoint set"
assert not (ROOT_STAY_FILES & CONFIG_SUBHEADERS), "root-stay files and config sub-headers are disjoint"

#: The full 45-entry root-file classification: ``None`` => stays at the sketch
#: root; else => the destination domain under ``src/<domain>/``. This is the
#: single contract every downstream task consumes.
MAPPING: Dict[str, Optional[str]] = {
    **{name: None for name in ROOT_STAY_FILES},
    **_FRAMEWORK_MAPPING,
}

#: Convenience view: just the 39 basenames that move.
FRAMEWORK_FILES: FrozenSet[str] = frozenset(_FRAMEWORK_MAPPING)

assert len(MAPPING) == 46, f"expected 46 classified root files, got {len(MAPPING)}"
assert len(ROOT_STAY_FILES) == 7
assert len(FRAMEWORK_FILES) == 39
assert set(MAPPING) == ROOT_STAY_FILES | FRAMEWORK_FILES


def domain_of(basename: str) -> Optional[str]:
    """Destination domain for one of the 45 classified root files.

    Returns ``None`` if `basename` stays at the sketch root. Raises ``KeyError``
    if `basename` is not one of the 45 files this migration classifies -- callers
    should only invoke this for files known to be in :data:`MAPPING`.
    """
    return MAPPING[basename]


def is_mover(basename: str) -> bool:
    """True if `basename` is one of the files this migration relocates."""
    return MAPPING.get(basename) is not None


def is_root_stay(basename: str) -> bool:
    """True if `basename` is one of the 7 operator files that stay at the root."""
    return basename in MAPPING and MAPPING[basename] is None


def is_config_subheader(basename: str) -> bool:
    """True if `basename` is one of the 8 ``config/`` sub-headers (never moved)."""
    return basename in CONFIG_SUBHEADERS


def source_repo_path(basename: str) -> Path:
    """Repo-relative path to `basename` as it sits TODAY, before the move.

    Every one of the classified files -- movers included -- is still
    physically at the flat sketch root today.
    """
    return Path(SKETCH_ROOT_NAME) / basename


def destination_repo_path(basename: str) -> Path:
    """Repo-relative path to `basename` AFTER the move.

    Root-stay files (domain ``None``) map to themselves (no move); the movers
    map to ``WRO2026/src/<domain>/<basename>``.
    """
    domain = MAPPING[basename]
    if domain is None:
        return Path(SKETCH_ROOT_NAME) / basename
    return Path(SKETCH_ROOT_NAME) / "src" / domain / basename
