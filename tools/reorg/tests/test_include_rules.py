"""Tests for tools.reorg.include_rules -- the production include-form rewriter.

Covers every row of the spec's "Include-form rules" table, idempotency, and
the flat-layout no-op property (both as a pure-function property over
`locate_old_layout`, and as an end-to-end check against every real quote-include
currently in the WRO2026/ tree).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tools.reorg import include_rules as ir
from tools.reorg import mapping


# ---------------------------------------------------------------------------
# parse_include
# ---------------------------------------------------------------------------


def test_parse_include_matches_a_plain_quote_include() -> None:
    assert ir.parse_include('#include "Drive.h"\n') == ('#include ', "Drive.h", "\n")


def test_parse_include_preserves_leading_whitespace_and_trailing_comment() -> None:
    line = '    #include "WROLineArray.h"    // Direct include for IWYU clarity\n'
    prefix, path, suffix = ir.parse_include(line)
    assert path == "WROLineArray.h"
    assert prefix == '    #include '
    assert suffix == '    // Direct include for IWYU clarity\n'


def test_parse_include_ignores_angle_bracket_includes() -> None:
    assert ir.parse_include("#include <MatrixMiniR4.h>\n") is None


def test_parse_include_ignores_prose_mentioning_an_include_in_a_comment() -> None:
    # A real line from WROLineArray.h's docstring: a `//`-prefixed comment that
    # merely quotes an #include for documentation purposes must never match.
    line = '// (`#include "Sensors/MiniR4ColorSensorExt.h"`). The Arduino\n'
    assert ir.parse_include(line) is None


def test_parse_include_ignores_non_include_lines() -> None:
    assert ir.parse_include('int x = 5;\n') is None
    assert ir.parse_include('\n') is None


# ---------------------------------------------------------------------------
# locate_old_layout / locate_new_layout
# ---------------------------------------------------------------------------


def test_locate_old_layout_reports_root_for_a_mover() -> None:
    # Drive.cpp WILL move, but under the old (current) layout it still sits at
    # the flat root -- this is the crux of the no-op guarantee.
    assert ir.locate_old_layout("Drive.cpp") == ir.ROOT


def test_locate_old_layout_reports_root_for_a_root_stay_file() -> None:
    assert ir.locate_old_layout("Config.h") == ir.ROOT


def test_locate_old_layout_reports_config_for_a_config_subheader() -> None:
    assert ir.locate_old_layout("ConfigManipulator.h") == ir.CONFIG


def test_locate_old_layout_never_returns_a_src_location() -> None:
    for basename in mapping.FRAMEWORK_FILES:
        loc = ir.locate_old_layout(basename)
        assert loc.kind != "src", f"{basename} unexpectedly resolved to src under the OLD layout"


def test_locate_old_layout_raises_for_untracked_name() -> None:
    with pytest.raises(KeyError):
        ir.locate_old_layout("NotTracked.h")


def test_locate_new_layout_reports_src_for_every_mover() -> None:
    for basename, domain in mapping.MAPPING.items():
        loc = ir.locate_new_layout(basename)
        if domain is None:
            assert loc == ir.ROOT
        else:
            assert loc == ir.src(domain)


def test_locate_new_layout_reports_config_for_a_config_subheader() -> None:
    assert ir.locate_new_layout("ConfigVision.h") == ir.CONFIG


def test_locate_dispatches_by_layout_name() -> None:
    assert ir.locate("Drive.cpp", "old") == ir.ROOT
    assert ir.locate("Drive.cpp", "new") == ir.src("drive")


def test_locate_rejects_unknown_layout() -> None:
    with pytest.raises(ValueError):
        ir.locate("Drive.cpp", "sideways")


# ---------------------------------------------------------------------------
# rewrite_form -- one case per Contracts include-form-rules table row.
# ---------------------------------------------------------------------------


def test_row_same_domain_bare_unchanged() -> None:
    # Drive.cpp (src/drive) including TurnSignPolicy.h (also src/drive, the
    # documented exception) stays bare.
    assert ir.rewrite_form(ir.src("drive"), "TurnSignPolicy.h", ir.src("drive")) == "TurnSignPolicy.h"


def test_row_sibling_domain() -> None:
    # Drive.cpp (src/drive) including WROLineArray.h (src/perception).
    assert (
        ir.rewrite_form(ir.src("drive"), "WROLineArray.h", ir.src("perception"))
        == "../perception/WROLineArray.h"
    )


def test_row_src_to_root() -> None:
    # Drive.h (src/drive) including Config.h (root).
    assert ir.rewrite_form(ir.src("drive"), "Config.h", ir.ROOT) == "../../Config.h"


def test_row_src_to_config() -> None:
    # ManipServo.h (src/manip) including config/ConfigManipulator.h.
    assert (
        ir.rewrite_form(ir.src("manip"), "ConfigManipulator.h", ir.CONFIG)
        == "../../config/ConfigManipulator.h"
    )
    # Vision.h / VisionFrame.h (src/perception) including config/ConfigVision.h.
    assert (
        ir.rewrite_form(ir.src("perception"), "ConfigVision.h", ir.CONFIG)
        == "../../config/ConfigVision.h"
    )


def test_row_root_to_src() -> None:
    # WRO2026.ino including Drive.h (src/drive).
    assert ir.rewrite_form(ir.ROOT, "Drive.h", ir.src("drive")) == "src/drive/Drive.h"


def test_row_root_to_root_bare_unchanged() -> None:
    # Tasks.cpp including Tasks.h; TasksTypes.h including Config.h.
    assert ir.rewrite_form(ir.ROOT, "Tasks.h", ir.ROOT) == "Tasks.h"


def test_row_root_to_config() -> None:
    # Config.h including config/ConfigDrive.h.
    assert ir.rewrite_form(ir.ROOT, "ConfigDrive.h", ir.CONFIG) == "config/ConfigDrive.h"


def test_row_config_to_config_bare_unchanged() -> None:
    assert ir.rewrite_form(ir.CONFIG, "ConfigGlobal.h", ir.CONFIG) == "ConfigGlobal.h"


# ---------------------------------------------------------------------------
# rewrite_production_line -- end-to-end on real production lines.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "line,includer,expected",
    [
        # Drive.h: #include "Config.h" / #include "WROMath.h"
        ('#include "Config.h"\n', ir.src("drive"), '#include "../../Config.h"\n'),
        ('#include "WROMath.h"\n', ir.src("drive"), '#include "../util/WROMath.h"\n'),
        # Drive.cpp: #include "TurnSignPolicy.h" (same domain, bare) and
        # #include "YawSource.h" (nav, sibling) and #include "WROLineArray.h" (perception, sibling).
        (
            '#include "TurnSignPolicy.h"    // Turn-sign + counter-rotation wheel math\n',
            ir.src("drive"),
            '#include "TurnSignPolicy.h"    // Turn-sign + counter-rotation wheel math\n',
        ),
        ('#include "YawSource.h"\n', ir.src("drive"), '#include "../nav/YawSource.h"\n'),
        ('#include "WROLineArray.h"\n', ir.src("drive"), '#include "../perception/WROLineArray.h"\n'),
        # ManipServo.h: #include "config/ConfigManipulator.h"
        (
            '#include "config/ConfigManipulator.h"\n',
            ir.src("manip"),
            '#include "../../config/ConfigManipulator.h"\n',
        ),
        # Mission.cpp: #include "PathTracker.h" (same domain, bare) and
        # #include "Tasks.h" (root) and #include "ctps_record.h" (untracked, unchanged).
        ('#include "PathTracker.h"\n', ir.src("mission"), '#include "PathTracker.h"\n'),
        ('#include "Tasks.h"\n', ir.src("mission"), '#include "../../Tasks.h"\n'),
        ('#include "ctps_record.h"\n', ir.src("mission"), '#include "ctps_record.h"\n'),
        # MissionScript.cpp (root file, stays put): #include "Mission.h" (src/mission).
        ('#include "Mission.h"\n', ir.ROOT, '#include "src/mission/Mission.h"\n'),
        # WRO2026.ino: several framework headers.
        ('#include "Manipulator.h"\n', ir.ROOT, '#include "src/manip/Manipulator.h"\n'),
        ('#include "FieldMap.h"\n', ir.ROOT, '#include "src/nav/FieldMap.h"\n'),
        # Angle-bracket include: never touched.
        ('#include <MatrixMiniR4.h>\n', ir.ROOT, '#include <MatrixMiniR4.h>\n'),
    ],
)
def test_rewrite_production_line_new_layout(line: str, includer: ir.Location, expected: str) -> None:
    assert ir.rewrite_production_line(line, includer, layout="new") == expected


def test_rewrite_production_line_old_layout_is_always_identity() -> None:
    """Every one of the above lines, rewritten under the OLD layout, is unchanged.

    This is the load-bearing "no-op under the current flat layout" property:
    since `locate_old_layout` never reports a `src/<domain>` location, none of
    `rewrite_form`'s rewriting branches can ever fire -- every line comes back
    byte-identical, regardless of includer.
    """
    real_lines_and_includers = [
        ('#include "Config.h"\n', ir.ROOT),
        ('#include "WROMath.h"\n', ir.ROOT),
        ('#include "TurnSignPolicy.h"\n', ir.ROOT),
        ('#include "YawSource.h"\n', ir.ROOT),
        ('#include "config/ConfigManipulator.h"\n', ir.ROOT),
        ('#include "PathTracker.h"\n', ir.ROOT),
        ('#include "Mission.h"\n', ir.ROOT),
        ('#include "ctps_record.h"\n', ir.ROOT),
        ('#include <MatrixMiniR4.h>\n', ir.ROOT),
        ('#include "ConfigGlobal.h"\n', ir.CONFIG),
    ]
    for line, includer in real_lines_and_includers:
        assert ir.rewrite_production_line(line, includer, layout="old") == line


def test_rewrite_production_line_leaves_untracked_quote_includes_alone() -> None:
    line = '#include "TestAssert.h"\n'
    assert ir.rewrite_production_line(line, ir.src("drive"), layout="new") == line


def test_rewrite_production_line_is_idempotent_new_layout() -> None:
    line = '#include "WROLineArray.h"    // Direct include for IWYU clarity\n'
    once = ir.rewrite_production_line(line, ir.src("drive"), layout="new")
    twice = ir.rewrite_production_line(once, ir.src("drive"), layout="new")
    assert once == '#include "../perception/WROLineArray.h"    // Direct include for IWYU clarity\n'
    assert once == twice


def test_rewrite_production_line_is_idempotent_for_root_to_src() -> None:
    line = '#include "Drive.h"\n'
    once = ir.rewrite_production_line(line, ir.ROOT, layout="new")
    twice = ir.rewrite_production_line(once, ir.ROOT, layout="new")
    assert once == '#include "src/drive/Drive.h"\n'
    assert once == twice


# ---------------------------------------------------------------------------
# The flat-layout no-op property, proven against every real quote-include
# currently in the WRO2026/ tree (excluding test/, which is out of scope for
# the production rewriter).
# ---------------------------------------------------------------------------


def test_old_layout_is_a_true_no_op_over_the_real_tree(sketch_root: Path) -> None:
    """Rewriting every real production file today (layout="old") changes nothing.

    Read-only: this test never writes back to the real tree, it only computes
    what the rewrite WOULD produce and asserts it is byte-identical to the
    original for every file directly under WRO2026/ and WRO2026/config/.
    """
    if (sketch_root / "src").is_dir():
        pytest.skip("pre-move only: this snapshots the flat pre-move tree; the reorg move has run")
    root_files = [
        p
        for p in sketch_root.iterdir()
        if p.is_file() and p.suffix in (".h", ".cpp", ".ino") and p.name in mapping.MAPPING
    ]
    config_files = [
        p for p in (sketch_root / "config").iterdir() if p.is_file() and p.suffix == ".h"
    ]
    assert root_files, "sanity: expected to find the 45 root files"
    assert config_files, "sanity: expected to find the 8 config sub-headers"

    for path in root_files + config_files:
        includer = ir.locate_old_layout(path.name)
        original = path.read_text(encoding="utf-8")
        rewritten = "".join(
            ir.rewrite_production_line(line, includer, layout="old")
            for line in original.splitlines(keepends=True)
        )
        assert rewritten == original, f"{path} changed under the OLD layout (expected a no-op)"
