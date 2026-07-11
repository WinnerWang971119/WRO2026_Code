"""Tests for tools.reorg.test_includes -- the WRO2026/test/ real-vs-mock classifier.

The exact "real" classification set below (`EXPECTED_REAL`) is a pinned,
hand-verified snapshot of every ambiguous-basename include currently in the
tree (cross-checked against WRO2026/test/CMakeLists.txt's per-target
`target_include_directories` / force-include recipes at authoring time). If
this test starts failing because the set changed, that means either (a) a new
ambiguous include was added and needs a `REAL_INCLUDE_MANIFEST` entry (if
real) or is correctly absorbed by the same-dir-sibling rule / stays mock by
default, or (b) an existing one moved -- either way it demands a human look,
which is exactly the point of pinning it exactly rather than just a count.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tools.reorg import test_includes as ti


# ---------------------------------------------------------------------------
# discover_mock_basenames / ambiguous_basenames against the real tree.
# ---------------------------------------------------------------------------


def test_discover_mock_basenames_matches_the_known_9(test_root: Path) -> None:
    # Every basename that is BOTH a migration-tracked mover AND shadowed by a
    # mock header somewhere under WRO2026/test/ today.
    expected = {
        "Drive.h",
        "Intersection.h",
        "WROLineArray.h",
        "EventTrace.h",
        "LineFollower.h",
        "Manipulator.h",
        "PathTracker.h",
        "Localization.h",
        "Vision.h",
    }
    mocked = ti.discover_mock_basenames(test_root)
    # discover_mock_basenames also picks up non-tracked vendor mocks (Arduino.h,
    # MatrixMiniR4.h) -- intersect with the mapping the way ambiguous_basenames does.
    from tools.reorg.mapping import MAPPING

    assert {name for name in mocked if name in MAPPING} == expected


def test_ambiguous_basenames_matches_the_known_9(test_root: Path) -> None:
    expected = {
        "Drive.h",
        "Intersection.h",
        "WROLineArray.h",
        "EventTrace.h",
        "LineFollower.h",
        "Manipulator.h",
        "PathTracker.h",
        "Localization.h",
        "Vision.h",
    }
    assert ti.ambiguous_basenames(test_root) == expected


def test_unambiguous_movers_have_no_mock_anywhere(test_root: Path) -> None:
    """Spot-check a handful of the 30 non-ambiguous movers: truly no mock exists."""
    ambiguous = ti.ambiguous_basenames(test_root)
    for basename in ("WROMath.h", "FieldMap.h", "NavTypes.h", "routes.h", "YawSource.h",
                      "StopAtLinePolicy.h", "PivotTrimPolicy.h", "VisionFrame.h"):
        assert basename not in ambiguous


# ---------------------------------------------------------------------------
# classify_include -- unit-level, with synthetic sibling sets.
# ---------------------------------------------------------------------------


def test_classify_unmapped_basename() -> None:
    result = ti.classify_include("some_file.cpp", frozenset(), "Arduino.h", frozenset({"Drive.h"}))
    assert result.classification == "unmapped"
    assert result.evidence == "unmapped"


def test_classify_unambiguous_mover_is_always_real() -> None:
    result = ti.classify_include("some_file.cpp", frozenset(), "WROMath.h", frozenset({"Drive.h"}))
    assert result.classification == "real"
    assert result.evidence == "unambiguous"


def test_classify_ambiguous_with_same_dir_sibling_is_mock() -> None:
    ambiguous = frozenset({"Drive.h"})
    result = ti.classify_include(
        "mocks/mocks.cpp", frozenset({"Drive.h", "mocks.cpp"}), "Drive.h", ambiguous
    )
    assert result.classification == "mock"
    assert result.evidence == "same-dir-sibling"


def test_classify_ambiguous_in_manifest_is_real() -> None:
    ambiguous = frozenset({"Manipulator.h"})
    result = ti.classify_include(
        "test_manipulator.cpp", frozenset({"test_manipulator.cpp"}), "Manipulator.h", ambiguous
    )
    assert result.classification == "real"
    assert result.evidence == "manifest"


def test_classify_ambiguous_with_no_evidence_defaults_to_mock() -> None:
    ambiguous = frozenset({"Drive.h"})
    result = ti.classify_include(
        "test_mission_dispatch.cpp", frozenset({"test_mission_dispatch.cpp"}), "Drive.h", ambiguous
    )
    assert result.classification == "mock"
    assert result.evidence == "default-mock"


# ---------------------------------------------------------------------------
# GENERATED_FILE_EXCLUSIONS + looks_emitter_generated
# ---------------------------------------------------------------------------


def test_generated_file_exclusions_are_excluded(test_root: Path) -> None:
    for relpath in ti.GENERATED_FILE_EXCLUSIONS:
        assert ti.is_excluded_generated(test_root / relpath, test_root)


def test_looks_emitter_generated_true_for_the_real_emitted_fixture(test_root: Path) -> None:
    assert ti.looks_emitter_generated(test_root / "fixtures" / "routes_fixture.cpp")
    assert ti.looks_emitter_generated(test_root / "fixtures" / "routes_fixture.h")


def test_looks_emitter_generated_is_fooled_by_the_hand_written_forced_fixture(test_root: Path) -> None:
    """Documents the known discrepancy: this file is EXCLUDED (Contracts says so) but its
    own banner says "HAND-WRITTEN, NOT AUTO-GENERATED" -- the content heuristic correctly
    (and deliberately) reports False here, which is exactly why GENERATED_FILE_EXCLUSIONS,
    not this heuristic, is the authoritative exclusion gate.
    """
    path = test_root / "fixtures" / "routes_reset_passthrough_forced.cpp"
    assert not ti.looks_emitter_generated(path)
    assert ti.is_excluded_generated(path, test_root)


def test_looks_emitter_generated_false_for_an_ordinary_hand_written_test(test_root: Path) -> None:
    assert not ti.looks_emitter_generated(test_root / "test_manipulator.cpp")


# ---------------------------------------------------------------------------
# audit_test_includes -- the exact pinned real-classification snapshot.
# ---------------------------------------------------------------------------

EXPECTED_REAL = {
    ("test_path_tracker_harness.h", "Drive.h"),
    ("test_path_tracker_harness.h", "Localization.h"),
    ("test_path_tracker_harness.h", "PathTracker.h"),
    ("test_path_tracker_bidirectional.cpp", "PathTracker.h"),
    ("test_line_follower_overshoot.cpp", "LineFollower.h"),
    ("test_manipulator.cpp", "Manipulator.h"),
    ("test_localization_offset.cpp", "Localization.h"),
    ("test_reanchor_at_wall.cpp", "Localization.h"),
    ("test_reanchor_at_wall_strict.cpp", "Localization.h"),
    ("test_reanchor_heading.cpp", "Localization.h"),
}


def test_audit_pinned_real_set(test_root: Path) -> None:
    entries = ti.audit_test_includes(test_root)
    real = {(e.test_relpath, e.included_basename) for e in entries if e.classification == "real"}
    assert real == EXPECTED_REAL


def test_audit_never_touches_generated_exclusions(test_root: Path) -> None:
    entries = ti.audit_test_includes(test_root)
    for e in entries:
        assert e.test_relpath not in ti.GENERATED_FILE_EXCLUSIONS


def test_audit_every_entry_is_ambiguous_and_classified(test_root: Path) -> None:
    ambiguous = ti.ambiguous_basenames(test_root)
    entries = ti.audit_test_includes(test_root)
    assert entries, "sanity: expected to find at least one ambiguous include in the tree"
    for e in entries:
        assert e.included_basename in ambiguous
        assert e.classification in ("real", "mock")
        assert e.evidence in ("same-dir-sibling", "manifest", "default-mock")


# ---------------------------------------------------------------------------
# rewrite_test_line -- end-to-end on real classifications.
# ---------------------------------------------------------------------------


def test_rewrite_test_line_rewrites_a_real_include_to_root_relative_src_form(test_root: Path) -> None:
    ambiguous = ti.ambiguous_basenames(test_root)
    line = '#include "Manipulator.h"\n'
    rewritten = ti.rewrite_test_line(
        line, "test_manipulator.cpp", frozenset({"test_manipulator.cpp"}), ambiguous
    )
    assert rewritten == '#include "src/manip/Manipulator.h"\n'


def test_rewrite_test_line_leaves_a_mock_include_unchanged(test_root: Path) -> None:
    ambiguous = ti.ambiguous_basenames(test_root)
    line = '#include "Drive.h"          // mock — Drive\n'
    rewritten = ti.rewrite_test_line(
        line, "test_mission_dispatch.cpp", frozenset({"test_mission_dispatch.cpp"}), ambiguous
    )
    assert rewritten == line


def test_rewrite_test_line_leaves_an_untracked_include_unchanged(test_root: Path) -> None:
    ambiguous = ti.ambiguous_basenames(test_root)
    line = '#include "TestAssert.h"\n'
    rewritten = ti.rewrite_test_line(line, "test_manipulator.cpp", frozenset(), ambiguous)
    assert rewritten == line


def test_rewrite_test_line_leaves_a_root_stay_target_unchanged() -> None:
    # Config.h is root-stay (never moves); even if it were somehow ambiguous+real
    # this must stay bare since target.kind != "src". Sanity over a real root include.
    ambiguous: frozenset = frozenset()
    line = '#include "Config.h"\n'
    rewritten = ti.rewrite_test_line(line, "test_manipulator_config_fail.cpp", frozenset(), ambiguous)
    assert rewritten == line
