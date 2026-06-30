"""Unit + integration tests for the host-coverage / no-new-mirror CI guard.

The guard under test is ``WRO2026/test/check_host_coverage.py`` (plan
``docs/plans/2026-06-30-ci-host-coverage-guards.md``). It is a standalone
stdlib-only script, not an installed package, so it is loaded here via
:mod:`importlib` from its path on disk.

This file lands in ``tools/path_planner/tests/``, which the CI ``path-planner``
job collects with a broad ``pytest tools/path_planner/tests/``. Everything the
two real-repo cases (TC1/TC2) touch -- the guard script, both allowlists, the
test ``.cpp`` files, and the ``CMakeLists`` -- is committed and present in every
checkout, so these tests pass unconditionally. There is deliberately NO
``CI``-env gate, ``pytest.skip``/``importorskip``, or platform guard.

Case map (Testing Strategy table TC1-TC16):

* TC1  -- ``main()`` returns 0 against the real repo (anti-tautology).
* TC2  -- ``count_mirrors`` over the real test tree returns the exact map.
* TC3  -- mid-line ``MIRROR:`` annotation IS counted (the original bug).
* TC4  -- block-comment ``*`` ``MIRROR:`` annotation IS counted.
* TC5  -- prose ``MIRROR:`` mentions (no concrete ref) are NOT counted.
* TC6  -- ref-less ``MIRROR:`` comment is flagged as a violation.
* TC7  -- a cited mirror in a file not in the allowlist fails.
* TC8  -- count raised / dropped vs the allowlist fails (both directions).
* TC9  -- allowlist entry for a file with 0 mirrors / missing file is stale.
* TC10 -- ``linked_sources`` sees ``add_executable``/``target_sources``/``set``.
* TC11 -- non-source mentions of a source are NOT counted as linked.
* TC12 -- a production ``.cpp`` neither linked nor allowlisted fails.
* TC13 -- a coverage entry that is now linked is stale.
* TC14 -- a coverage entry for a non-existent file is stale.
* TC15 -- schema-invalid allowlists each raise ``AllowlistError``.
* TC16 -- the two checks run independently (one's bad input never suppresses
  the other's diagnostics).
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


def _load_guard():
    """Load ``check_host_coverage`` from ``WRO2026/test/`` via importlib.

    Walks up from this file until the guard script is found, so the load is
    robust to the test-tree depth (``tools/path_planner/tests/``) rather than a
    hard-coded ``parents[N]`` index.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "WRO2026" / "test" / "check_host_coverage.py"
        if candidate.is_file():
            spec = importlib.util.spec_from_file_location(
                "check_host_coverage", candidate
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise FileNotFoundError(
        f"WRO2026/test/check_host_coverage.py not found above {here}"
    )


guard = _load_guard()


def _write_json(path: Path, obj) -> Path:
    """Serialize ``obj`` to ``path`` as JSON and return the path."""
    path.write_text(json.dumps(obj), encoding="utf-8")
    return path


# ---------------------------------------------------------------------------
# TC1 / TC2 -- live assertions against the REAL repo (anti-tautology).
# ---------------------------------------------------------------------------


def test_tc1_main_returns_zero_against_real_repo():
    """TC1: ``main()`` is green on the committed tree.

    The seeded allowlists must match reality -- both guards pass and ``main``
    returns 0. A non-zero here means the regex or a seed drifted, not the test.
    """
    assert guard.main([]) == 0, "guard must be green against the committed tree"


def test_tc2_count_mirrors_real_tree_is_exact():
    """TC2: the live mirror map is exactly the two baselined files."""
    test_dir = guard.repo_root() / "WRO2026" / "test"
    assert guard.count_mirrors(test_dir) == {
        "test_wall_stop_watchdog.cpp": 9,
        "test_tasks_dispatch_overflow.cpp": 1,
    }


# ---------------------------------------------------------------------------
# TC3 / TC4 -- genuine annotations are counted regardless of comment style.
# ---------------------------------------------------------------------------


def test_tc3_midline_mirror_is_counted(tmp_path):
    """TC3: a mid-line ``MIRROR:`` with a citation counts (position-independent)."""
    (tmp_path / "test_x.cpp").write_text(
        "    foo();  // some prose here MIRROR: Foo.cpp:12\n", encoding="utf-8"
    )
    assert guard.count_mirrors(tmp_path) == {"test_x.cpp": 1}


def test_tc4_block_comment_mirror_is_counted(tmp_path):
    """TC4: a ``*`` block-comment ``MIRROR:`` with a citation counts."""
    (tmp_path / "test_y.h").write_text(" * MIRROR: Foo.h:5\n", encoding="utf-8")
    assert guard.count_mirrors(tmp_path) == {"test_y.h": 1}


# ---------------------------------------------------------------------------
# TC5 -- prose mentions with no concrete citation are NOT counted.
# ---------------------------------------------------------------------------


def test_tc5_prose_mentions_are_not_counted(tmp_path):
    """TC5: a ``<line>`` placeholder and backtick-quoted markers do not count."""
    (tmp_path / "test_prose.cpp").write_text(
        "// MIRROR: Foo.cpp:<line>\n"  # placeholder, no concrete digit
        " * a `// MIRROR:` backtick-quoted marker\n"  # quoted marker, no ref
        " * `// MIRROR:` of `Foo.cpp`\n",  # block backtick, no file:line
        encoding="utf-8",
    )
    assert guard.count_mirrors(tmp_path) == {}


# ---------------------------------------------------------------------------
# TC6 -- a ref-less, non-prose MIRROR comment is a hard violation.
# ---------------------------------------------------------------------------


def test_tc6_refless_mirror_is_flagged(tmp_path):
    """TC6: ``// MIRROR: copied logic`` (no citation) fails the ref-less guard."""
    (tmp_path / "test_refless.cpp").write_text(
        "// MIRROR: copied logic\n", encoding="utf-8"
    )
    violations = guard.find_refless_mirrors(tmp_path)
    assert len(violations) == 1, violations
    assert "test_refless.cpp" in violations[0]
    assert "missing concrete reference" in violations[0]


# ---------------------------------------------------------------------------
# TC7 -- a cited mirror in a file outside the allowlist fails.
# ---------------------------------------------------------------------------


def test_tc7_cited_mirror_in_unlisted_file(tmp_path):
    """TC7: a new file with a cited mirror, absent from the allowlist, fails."""
    test_dir = tmp_path / "test"
    test_dir.mkdir()
    (test_dir / "test_new.cpp").write_text(
        "// MIRROR: Foo.cpp:12\n", encoding="utf-8"
    )
    allowlist = _write_json(tmp_path / "mirror.json", {"entries": []})

    violations = guard.check_mirrors(test_dir, allowlist)
    assert any(
        "test_new.cpp" in v and "not in the mirror allowlist" in v
        for v in violations
    ), violations


# ---------------------------------------------------------------------------
# TC8 -- count mismatch vs the allowlist, both directions.
# ---------------------------------------------------------------------------


def test_tc8_count_mismatch_both_directions(tmp_path):
    """TC8: an observed count above OR below the allowlisted value fails."""
    test_dir = tmp_path / "test"
    test_dir.mkdir()
    source = test_dir / "test_mirror.cpp"

    # Raised: two annotations observed, allowlist pins 1.
    source.write_text(
        "// MIRROR: Foo.cpp:1\n// MIRROR: Bar.cpp:2\n", encoding="utf-8"
    )
    raised_allowlist = _write_json(
        tmp_path / "raised.json",
        {"entries": [{"file": "test_mirror.cpp", "count": 1, "reason": "x", "issue": None}]},
    )
    raised = guard.check_mirrors(test_dir, raised_allowlist)
    assert any(
        "test_mirror.cpp" in v and "expected 1, found 2" in v for v in raised
    ), raised

    # Dropped: one annotation observed, allowlist pins 2.
    source.write_text("// MIRROR: Foo.cpp:1\n", encoding="utf-8")
    dropped_allowlist = _write_json(
        tmp_path / "dropped.json",
        {"entries": [{"file": "test_mirror.cpp", "count": 2, "reason": "x", "issue": None}]},
    )
    dropped = guard.check_mirrors(test_dir, dropped_allowlist)
    assert any(
        "test_mirror.cpp" in v and "expected 2, found 1" in v for v in dropped
    ), dropped


# ---------------------------------------------------------------------------
# TC9 -- a stale mirror allowlist entry (0 mirrors / missing file).
# ---------------------------------------------------------------------------


def test_tc9_stale_mirror_entry(tmp_path):
    """TC9: an allowlisted file with no observed mirrors is reported stale."""
    test_dir = tmp_path / "test"
    test_dir.mkdir()

    # Entry for a file that does not exist on disk.
    missing_allowlist = _write_json(
        tmp_path / "missing.json",
        {"entries": [{"file": "test_gone.cpp", "count": 3, "reason": "x", "issue": None}]},
    )
    missing = guard.check_mirrors(test_dir, missing_allowlist)
    assert any(
        "test_gone.cpp" in v and "stale mirror allowlist entry" in v and "found 0" in v
        for v in missing
    ), missing

    # Entry for a file that exists but carries zero MIRROR annotations.
    (test_dir / "test_empty.cpp").write_text("int main() {}\n", encoding="utf-8")
    empty_allowlist = _write_json(
        tmp_path / "empty.json",
        {"entries": [{"file": "test_empty.cpp", "count": 1, "reason": "x", "issue": None}]},
    )
    empty = guard.check_mirrors(test_dir, empty_allowlist)
    assert any(
        "test_empty.cpp" in v and "stale mirror allowlist entry" in v
        for v in empty
    ), empty


# ---------------------------------------------------------------------------
# TC10 / TC11 -- linkage detection across CMake command bodies.
# ---------------------------------------------------------------------------


def test_tc10_linked_sources_across_command_forms(tmp_path):
    """TC10: ``add_executable``, ``target_sources``, and a multi-line ``set``."""
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(
        "add_executable(prog ${SKETCH_DIR}/Alpha.cpp)\n"
        "target_sources(prog PRIVATE ${SKETCH_DIR}/Beta.cpp)\n"
        "set(COMMON_SOURCES\n"
        "    ${SKETCH_DIR}/Gamma.cpp\n"
        ")\n",
        encoding="utf-8",
    )
    assert guard.linked_sources(cmake) == {"Alpha.cpp", "Beta.cpp", "Gamma.cpp"}


def test_tc11_non_source_mentions_are_not_linked(tmp_path):
    """TC11: ``message()`` / bare line / non-source command / ``#`` comment."""
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(
        'message(STATUS "building ${SKETCH_DIR}/InMessage.cpp")\n'
        "${SKETCH_DIR}/BareLine.cpp\n"
        "custom_command(OUTPUT ${SKETCH_DIR}/NonSource.cpp)\n"
        "# ${SKETCH_DIR}/Commented.cpp\n",
        encoding="utf-8",
    )
    assert guard.linked_sources(cmake) == set()


# ---------------------------------------------------------------------------
# TC12 -- an uncovered production source fails coverage.
# ---------------------------------------------------------------------------


def test_tc12_uncovered_production_source(tmp_path):
    """TC12: a ``.cpp`` neither linked nor allowlisted is named in a violation."""
    prod_dir = tmp_path / "WRO2026"
    prod_dir.mkdir()
    (prod_dir / "Orphan.cpp").write_text("", encoding="utf-8")
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(
        "add_executable(t ${SKETCH_DIR}/Other.cpp)\n", encoding="utf-8"
    )
    allowlist = _write_json(tmp_path / "cov.json", {"entries": []})

    violations = guard.check_coverage(prod_dir, cmake, allowlist)
    assert any(
        "Orphan.cpp" in v and "neither host-linked nor in the coverage allowlist" in v
        for v in violations
    ), violations


# ---------------------------------------------------------------------------
# TC13 / TC14 -- reverse-drift: stale coverage allowlist entries.
# ---------------------------------------------------------------------------


def test_tc13_coverage_entry_now_linked(tmp_path):
    """TC13: an allowlisted file that has since become linked is stale."""
    prod_dir = tmp_path / "WRO2026"
    prod_dir.mkdir()
    (prod_dir / "Linked.cpp").write_text("", encoding="utf-8")
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(
        "add_executable(t ${SKETCH_DIR}/Linked.cpp)\n", encoding="utf-8"
    )
    allowlist = _write_json(
        tmp_path / "cov.json",
        {"entries": [{"file": "Linked.cpp", "reason": "x", "issue": None}]},
    )

    violations = guard.check_coverage(prod_dir, cmake, allowlist)
    assert any(
        "Linked.cpp" in v and "stale coverage allowlist entry" in v and "now" in v
        for v in violations
    ), violations


def test_tc14_coverage_entry_no_longer_exists(tmp_path):
    """TC14: an allowlisted file that no longer exists is stale."""
    prod_dir = tmp_path / "WRO2026"
    prod_dir.mkdir()  # no .cpp files at all
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(
        "add_executable(t ${SKETCH_DIR}/Real.cpp)\n", encoding="utf-8"
    )
    allowlist = _write_json(
        tmp_path / "cov.json",
        {"entries": [{"file": "Ghost.cpp", "reason": "x", "issue": None}]},
    )

    violations = guard.check_coverage(prod_dir, cmake, allowlist)
    assert any(
        "Ghost.cpp" in v and "stale coverage allowlist entry" in v and "no longer" in v
        for v in violations
    ), violations


# ---------------------------------------------------------------------------
# TC15 -- strict allowlist schema validation.
# ---------------------------------------------------------------------------


def test_tc15_schema_invalid_allowlists_raise(tmp_path):
    """TC15: each schema violation raises :class:`AllowlistError`."""

    def _expect_error(obj, *, with_count):
        path = _write_json(tmp_path / "bad.json", obj)
        with pytest.raises(guard.AllowlistError):
            guard.load_allowlist(path, with_count=with_count)

    # Bad envelope: object without an 'entries' key.
    _expect_error({"items": []}, with_count=False)
    # Bad envelope: top-level is a list, not an object.
    _expect_error([], with_count=False)
    # Duplicate 'file' across two entries.
    _expect_error(
        {
            "entries": [
                {"file": "A.cpp", "reason": "x", "issue": None},
                {"file": "A.cpp", "reason": "y", "issue": None},
            ]
        },
        with_count=False,
    )
    # count == 0 (must be >= 1).
    _expect_error(
        {"entries": [{"file": "A.cpp", "count": 0, "reason": "x", "issue": None}]},
        with_count=True,
    )
    # count is a bool (an int subclass that must be rejected).
    _expect_error(
        {"entries": [{"file": "A.cpp", "count": True, "reason": "x", "issue": None}]},
        with_count=True,
    )
    # Empty 'reason'.
    _expect_error(
        {"entries": [{"file": "A.cpp", "reason": "", "issue": None}]},
        with_count=False,
    )
    # 'issue' is neither a string nor null.
    _expect_error(
        {"entries": [{"file": "A.cpp", "reason": "x", "issue": 5}]},
        with_count=False,
    )


# ---------------------------------------------------------------------------
# TC16 -- the two checks never short-circuit each other.
# ---------------------------------------------------------------------------


def test_tc16_checks_run_independently(tmp_path):
    """TC16: one check's bad input never suppresses the other's diagnostics.

    ``main`` hard-codes :func:`repo_root` paths, so the both-checks-independent
    property is exercised at the ``check_mirrors``/``check_coverage`` level (the
    same functions ``main`` calls in separate ``try``/``except`` blocks). Each
    returns its own violation independently, so a single ``main`` run would
    report both and return 1 without mutating any committed file.
    """
    # A shared fake test tree with a cited (allowlist-worthy) mirror.
    test_dir = tmp_path / "test"
    test_dir.mkdir()
    (test_dir / "test_a.cpp").write_text(
        "// MIRROR: Foo.cpp:1\n", encoding="utf-8"
    )
    # A shared fake production tree with one uncovered source.
    prod_dir = tmp_path / "WRO2026"
    prod_dir.mkdir()
    (prod_dir / "Orphan.cpp").write_text("", encoding="utf-8")
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(
        "add_executable(t ${SKETCH_DIR}/Other.cpp)\n", encoding="utf-8"
    )

    # --- A: malformed mirror allowlist + a live coverage violation ---
    bad_mirror = tmp_path / "mirror_bad.json"
    bad_mirror.write_text("{not valid json", encoding="utf-8")
    mirror_a = guard.check_mirrors(test_dir, bad_mirror)
    assert mirror_a, "malformed mirror allowlist must yield a violation"
    assert any("cannot use" in v for v in mirror_a), mirror_a

    good_coverage = _write_json(tmp_path / "cov_ok.json", {"entries": []})
    coverage_a = guard.check_coverage(prod_dir, cmake, good_coverage)
    assert coverage_a, "uncovered source must still be reported"
    assert any("Orphan.cpp" in v for v in coverage_a), coverage_a

    # --- B (inverse): malformed coverage allowlist + a live mirror violation ---
    good_mirror = _write_json(tmp_path / "mirror_ok.json", {"entries": []})
    mirror_b = guard.check_mirrors(test_dir, good_mirror)
    assert mirror_b, "unlisted mirror must still be reported"
    assert any("test_a.cpp" in v for v in mirror_b), mirror_b

    bad_coverage = tmp_path / "cov_bad.json"
    bad_coverage.write_text("{nope", encoding="utf-8")
    coverage_b = guard.check_coverage(prod_dir, cmake, bad_coverage)
    assert coverage_b, "malformed coverage allowlist must yield a violation"
    assert any("cannot use" in v for v in coverage_b), coverage_b
