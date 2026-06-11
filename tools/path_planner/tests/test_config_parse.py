"""Tests for the shared Config.h ``constexpr float`` parser (``config_parse``).

``planner`` and ``validator`` both read chassis constants out of
``WRO2026/Config.h`` through :func:`tools.path_planner.config_parse.parse_config_float`.
These tests pin the anchoring (#8: a commented example must not shadow the real
declaration) and the no-stale-fallback policy (issue #44).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tools.path_planner.config_parse import parse_config_float


def test_parse_config_float_reads_indented_decl(tmp_path) -> None:
    """A namespaced / indented declaration is parsed: the anchor allows leading
    whitespace before ``constexpr``."""
    cfg = tmp_path / "Config.h"
    cfg.write_text(
        "namespace Config {\n    constexpr float TRACK_MM = 180.0f;\n}\n",
        encoding="utf-8",
    )
    assert parse_config_float("TRACK_MM", cfg) == pytest.approx(180.0)


def test_parse_config_float_skips_commented_decl(tmp_path) -> None:
    """A ``//``-commented example above the real decl must not shadow it (#8).

    The match is anchored to the first non-whitespace token on a line, so a
    commented-out declaration is no longer the first ``search`` hit; the real
    uncommented declaration below it wins.
    """
    cfg = tmp_path / "Config.h"
    cfg.write_text(
        "// constexpr float TRACK_MM = 999.0f;  // example, wrong width\n"
        "    // constexpr float TRACK_MM = 888.0f;\n"
        "namespace Config {\n    constexpr float TRACK_MM = 180.0f;\n}\n",
        encoding="utf-8",
    )
    assert parse_config_float("TRACK_MM", cfg) == pytest.approx(180.0)


def test_parse_config_float_missing_decl_raises(tmp_path) -> None:
    """No matching declaration -> ValueError, never a stale fallback (issue #44)."""
    cfg = tmp_path / "Config.h"
    cfg.write_text("namespace Config {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="TRACK_MM"):
        parse_config_float("TRACK_MM", cfg)


def test_parse_config_float_unreadable_header_raises() -> None:
    """A missing header -> OSError, never a stale fallback (issue #44)."""
    with pytest.raises(OSError):
        parse_config_float("TRACK_MM", Path("does/not/exist/Config.h"))
