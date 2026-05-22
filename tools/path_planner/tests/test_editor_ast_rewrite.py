"""Tests for tools.path_planner.editor.ast_rewrite.rewrite_edges_uv.

TC7  — rewrite preserves comments + only the renamed string literals change.
TC8  — no-op when nothing matches preserves mtime (and empty / no-EDGES variants).
TC15 — rename cascade reaches disk (rename in BOTH u and v of same entry).
"""

from __future__ import annotations

import difflib
from pathlib import Path

import pytest

from tools.path_planner.editor.ast_rewrite import rewrite_edges_uv

# ---------------------------------------------------------------------------
# Shared source fixtures (constants, not pytest fixtures, for clarity)
# ---------------------------------------------------------------------------

# The canonical TC7 / TC8 / TC15-cross-entry source. Has comments that MUST
# survive the rewrite. Uses "E" as a v-value in two separate entries.
_SOURCE_TC7 = '''\
"""Toy field for AST-rewrite tests."""

# Stand-in for the real EDGES table. Comments here must survive.
EDGES = [
    # [0] PROJECTION demo
    {
        "type":         "EDGE_PROJECTION",
        "u":            "A",
        "v":            "P1",
        "distance_cm":  45.0,
        "heading_deg":  0.0,
    },
    # [1] LINE edge
    {
        "type":         "EDGE_LINE",
        "u":            "D",
        "v":            "E",
        "distance_cm":  114.02,
        "heading_deg":  322.13,
    },
    # [2] LINE edge
    {
        "type":         "EDGE_LINE",
        "u":            "C",
        "v":            "E",
        "distance_cm":  122.07,
        "heading_deg":  214.99,
    },
]
'''

# Source where one entry has u == v == "A" (tests single-pass, both keys).
_SOURCE_SAME_UV = '''\
"""Toy field — same u and v for cascade test."""

EDGES = [
    {
        "type": "EDGE_LINE",
        "u":    "A",
        "v":    "A",
        "distance_cm": 10.0,
        "heading_deg": 0.0,
    },
    {
        "type": "EDGE_LINE",
        "u":    "B",
        "v":    "C",
        "distance_cm": 20.0,
        "heading_deg": 90.0,
    },
]
'''

# Source with NO EDGES assignment at all.
_SOURCE_NO_EDGES = '"""empty"""\n'

# Source with mixed single and double quote styles.
_SOURCE_MIXED_QUOTES = '''\
"""Mixed-quote source."""

EDGES = [
    {"u": "A", "v": \'B\', "type": "X"},
]
'''


def _write(tmp_path: Path, name: str, content: str) -> Path:
    """Write *content* to *tmp_path/name* encoded as UTF-8 with LF newlines.

    Forces ``newline="\\n"`` to match what ``rewrite_edges_uv`` writes; without
    it, ``write_text`` would emit CRLF on Windows and downstream byte-diff
    assertions would see a wall of line-ending changes on top of the real edits.
    """
    p = tmp_path / name
    p.write_text(content, encoding="utf-8", newline="\n")
    return p


# ---------------------------------------------------------------------------
# TC7 — rewrite preserves comments, only renamed string literals change
# ---------------------------------------------------------------------------


def test_tc7_only_renamed_literals_change(tmp_path: Path) -> None:
    """Renames 'E' → 'Q' in EDGES v-values; all comments and other literals survive."""
    path = _write(tmp_path, "field_data.py", _SOURCE_TC7)
    original_bytes = path.read_bytes()

    result = rewrite_edges_uv(path, {"E": "Q"})

    assert result is True, "rewrite_edges_uv should return True when a change is made"

    new_bytes = path.read_bytes()
    new_text = new_bytes.decode("utf-8")
    original_text = original_bytes.decode("utf-8")

    # The two occurrences of "E" as v-values must become "Q".
    assert '"Q"' in new_text, 'Renamed value "Q" must appear in output'
    assert new_text.count('"Q"') == 2, "Exactly two v-values should be renamed to Q"

    # "E" as a bare letter (the old name) must not appear as a standalone string value.
    # The only "E" left should be inside "EDGE_LINE", "EDGE_PROJECTION" — not as "E".
    assert '"E"' not in new_text, (
        'The old bare string "E" must not appear in the output; only "Q" should.'
    )

    # All comments must survive verbatim.
    for comment in (
        "# Stand-in for the real EDGES table. Comments here must survive.",
        "# [0] PROJECTION demo",
        "# [1] LINE edge",
        "# [2] LINE edge",
    ):
        assert comment in new_text, f"Comment must survive: {comment!r}"

    # u-values ("A", "D", "C") must be untouched.
    assert '"A"' in new_text
    assert '"D"' in new_text
    assert '"C"' in new_text

    # v-value "P1" (entry [0]) must be untouched.
    assert '"P1"' in new_text

    # Use unified_diff to confirm the diff is exactly the two "E" → "Q" replacements.
    diff_lines = list(
        difflib.unified_diff(
            original_text.splitlines(keepends=True),
            new_text.splitlines(keepends=True),
            fromfile="original",
            tofile="rewritten",
        )
    )
    # Only lines that begin with "+" or "-" (excluding the +++/--- headers) are the
    # actual changes. Each "E" → "Q" swap affects exactly one line (the "v" line), so
    # we expect exactly 2 removed lines and 2 added lines.
    removed = [l for l in diff_lines if l.startswith("-") and not l.startswith("---")]
    added = [l for l in diff_lines if l.startswith("+") and not l.startswith("+++")]
    assert len(removed) == 2, f"Expected 2 removed lines, got {len(removed)}: {removed}"
    assert len(added) == 2, f"Expected 2 added lines, got {len(added)}: {added}"
    for line in removed:
        assert '"E"' in line, f'Removed line should contain "E": {line!r}'
    for line in added:
        assert '"Q"' in line, f'Added line should contain "Q": {line!r}'


# ---------------------------------------------------------------------------
# TC8 — no-op when nothing matches preserves mtime
# ---------------------------------------------------------------------------


def test_tc8_no_match_returns_false_preserves_mtime(tmp_path: Path) -> None:
    """No node named 'Z' exists; returns False and leaves file byte-identical with same mtime."""
    path = _write(tmp_path, "field_data.py", _SOURCE_TC7)
    original_bytes = path.read_bytes()
    mtime_before = path.stat().st_mtime_ns

    result = rewrite_edges_uv(path, {"Z": "Q"})

    assert result is False, "rewrite_edges_uv should return False on a no-op rename"
    assert path.read_bytes() == original_bytes, "File content must be byte-identical on no-op"
    assert path.stat().st_mtime_ns == mtime_before, "mtime must not change on no-op"


def test_tc8_empty_renames_returns_false_preserves_mtime(tmp_path: Path) -> None:
    """Empty renames dict is a guaranteed no-op; returns False, file unchanged, mtime preserved."""
    path = _write(tmp_path, "field_data.py", _SOURCE_TC7)
    original_bytes = path.read_bytes()
    mtime_before = path.stat().st_mtime_ns

    result = rewrite_edges_uv(path, {})

    assert result is False, "Empty renames dict must return False"
    assert path.read_bytes() == original_bytes, "File content must be byte-identical"
    assert path.stat().st_mtime_ns == mtime_before, "mtime must not change"


def test_tc8_no_edges_assignment_returns_false(tmp_path: Path) -> None:
    """File with no EDGES assignment at all returns False and is not written."""
    path = _write(tmp_path, "field_data.py", _SOURCE_NO_EDGES)
    original_bytes = path.read_bytes()
    mtime_before = path.stat().st_mtime_ns

    result = rewrite_edges_uv(path, {"A": "Z"})

    assert result is False, "No EDGES assignment → must return False"
    assert path.read_bytes() == original_bytes, "File content must be byte-identical"
    assert path.stat().st_mtime_ns == mtime_before, "mtime must not change"


# ---------------------------------------------------------------------------
# TC15 — rename cascade reaches disk (u and v of same entry both rewritten)
# ---------------------------------------------------------------------------


def test_tc15_same_uv_both_keys_rewritten(tmp_path: Path) -> None:
    """Entry with u == v == 'A': renaming 'A' → 'Z' rewrites both keys in a single pass."""
    path = _write(tmp_path, "field_data.py", _SOURCE_SAME_UV)

    result = rewrite_edges_uv(path, {"A": "Z"})

    assert result is True, "rewrite_edges_uv must return True when a change is written"

    new_text = path.read_text(encoding="utf-8")

    # Entry [0] must now have u=Z, v=Z.
    assert '"Z"' in new_text, 'Renamed value "Z" must appear in output'
    assert new_text.count('"Z"') == 2, "Both u and v in entry [0] should be 'Z'"

    # The old bare "A" string literal must not appear anywhere in EDGES u/v context.
    # The only remaining "A" in the file is inside the _SOURCE_SAME_UV docstring
    # ("same u and v"), which does not contain the bare literal "A". Confirm directly:
    assert '"A"' not in new_text, (
        'Old value "A" must not remain anywhere after rename to "Z"'
    )

    # Entry [1] (u=B, v=C) must be untouched.
    assert '"B"' in new_text
    assert '"C"' in new_text


def test_tc15_cross_entry_both_occurrences_renamed(tmp_path: Path) -> None:
    """Single rename 'E' → 'Q' rewrites BOTH [1].v and [2].v in a single pass."""
    path = _write(tmp_path, "field_data.py", _SOURCE_TC7)

    result = rewrite_edges_uv(path, {"E": "Q"})

    assert result is True

    new_text = path.read_text(encoding="utf-8")
    # Cross-entry: entry [1] and [2] both had v="E"; now both must be "Q".
    assert new_text.count('"Q"') == 2, (
        "Both cross-entry occurrences of v='E' must be renamed to 'Q'"
    )
    assert '"E"' not in new_text, "No bare 'E' string literal should remain"


# ---------------------------------------------------------------------------
# Quote-style preservation test
# ---------------------------------------------------------------------------


def test_unsafe_rename_value_raises_before_touching_disk(tmp_path: Path) -> None:
    """Rename values with quote / backslash characters must raise ValueError untouched."""
    path = _write(tmp_path, "field_data.py", _SOURCE_TC7)
    original_bytes = path.read_bytes()
    mtime_before = path.stat().st_mtime_ns

    for bad in ('A"B', "A'B", "A\\B"):
        with pytest.raises(ValueError):
            rewrite_edges_uv(path, {"E": bad})

        assert path.read_bytes() == original_bytes, (
            f"File must not be written when value {bad!r} is rejected"
        )
        assert path.stat().st_mtime_ns == mtime_before, (
            f"mtime must not change when value {bad!r} is rejected"
        )


def test_unsafe_rename_key_raises_before_touching_disk(tmp_path: Path) -> None:
    """Rename keys with quote / backslash characters must raise ValueError untouched."""
    path = _write(tmp_path, "field_data.py", _SOURCE_TC7)
    original_bytes = path.read_bytes()

    with pytest.raises(ValueError):
        rewrite_edges_uv(path, {'bad"key': "Z"})

    assert path.read_bytes() == original_bytes


def test_quote_style_preserved_after_rename(tmp_path: Path) -> None:
    """Rename preserves single-quote style for v and double-quote style for u."""
    path = _write(tmp_path, "field_data.py", _SOURCE_MIXED_QUOTES)

    result = rewrite_edges_uv(path, {"A": "Z", "B": "Y"})

    assert result is True, "rewrite_edges_uv must return True when changes are made"

    new_text = path.read_text(encoding="utf-8")

    # u was double-quoted "A" → must become double-quoted "Z".
    assert '"Z"' in new_text, 'Double-quoted "Z" must appear (u preserved double-quote style)'

    # v was single-quoted 'B' → must become single-quoted 'Y'.
    assert "'Y'" in new_text, "Single-quoted 'Y' must appear (v preserved single-quote style)"

    # Old values must not appear as standalone string literals.
    assert '"A"' not in new_text
    assert "'B'" not in new_text
