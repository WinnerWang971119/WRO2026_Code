"""Tests for the shared Config.h ``constexpr`` parser (``config_parse``).

``planner`` and ``validator`` both read chassis constants out of
``WRO2026/Config.h`` through :func:`tools.path_planner.config_parse.parse_config_float`.
The CTPS motion-profile kernel additionally reads ``constexpr int16_t``/``uint8_t``
constants (including symbolic RHS like ``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE``)
through :func:`tools.path_planner.config_parse.parse_config_int`. These tests pin
the anchoring (#8: a commented example must not shadow the real declaration) and
the no-stale-fallback policy (issue #44).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tools.path_planner.config_parse import parse_config_float, parse_config_int


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


# ---------------------------------------------------------------------------------------
# parse_config_int (TC13): plain int parse, one-level identifier RHS, fail-loud on
# an expression, fail-loud on a two-level chain.
# ---------------------------------------------------------------------------------------


def test_parse_config_int_reads_plain_literal(tmp_path) -> None:
    """A plain ``constexpr int16_t NAME = <int>;`` declaration parses directly."""
    cfg = tmp_path / "ConfigDrive.h"
    cfg.write_text(
        "namespace Config {\n    constexpr int16_t SPEED_NORMAL = 85;\n}\n",
        encoding="utf-8",
    )
    assert parse_config_int("SPEED_NORMAL", cfg, search_headers=(cfg,)) == 85


def test_parse_config_int_reads_uint8_literal(tmp_path) -> None:
    """``constexpr uint8_t`` declarations are also matched (e.g. ``BATT_CELLS``)."""
    cfg = tmp_path / "ConfigGlobal.h"
    cfg.write_text(
        "namespace Config {\n    constexpr uint8_t BATT_CELLS = 2;\n}\n",
        encoding="utf-8",
    )
    assert parse_config_int("BATT_CELLS", cfg, search_headers=(cfg,)) == 2


def test_parse_config_int_reads_negative_literal(tmp_path) -> None:
    """A negative literal RHS parses (sign is part of the captured literal)."""
    cfg = tmp_path / "ConfigDrive.h"
    cfg.write_text(
        "namespace Config {\n    constexpr int16_t OFFSET_PWM = -20;\n}\n",
        encoding="utf-8",
    )
    assert parse_config_int("OFFSET_PWM", cfg, search_headers=(cfg,)) == -20


def test_parse_config_int_resolves_one_level_identifier_rhs(tmp_path) -> None:
    """``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE`` resolves across ConfigGlobal.h.

    Mirrors the real firmware shape: ``ConfigDrive.h`` declares
    ``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE`` and ``ConfigGlobal.h`` declares
    ``PWM_MIN_MAGNITUDE = 50`` as a plain literal. One identifier hop is
    resolved; the caller's ``config_path`` need not itself contain the literal.
    """
    global_h = tmp_path / "ConfigGlobal.h"
    global_h.write_text(
        "namespace Config {\n    constexpr int16_t PWM_MIN_MAGNITUDE = 50;\n}\n",
        encoding="utf-8",
    )
    drive_h = tmp_path / "ConfigDrive.h"
    drive_h.write_text(
        "namespace Config {\n"
        "    constexpr int16_t TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE;\n"
        "}\n",
        encoding="utf-8",
    )
    value = parse_config_int(
        "TSHAPE_START_SPEED", drive_h, search_headers=(drive_h, global_h)
    )
    assert value == 50


def test_parse_config_int_resolves_cruise_speed_alias(tmp_path) -> None:
    """``CRUISE_SPEED = SPEED_NORMAL`` resolves across ConfigDrive.h (the real shape)."""
    drive_h = tmp_path / "ConfigDrive.h"
    drive_h.write_text(
        "namespace Config {\n    constexpr int16_t SPEED_NORMAL = 85;\n}\n",
        encoding="utf-8",
    )
    line_h = tmp_path / "ConfigLine.h"
    line_h.write_text(
        "namespace Config {\n    constexpr int16_t CRUISE_SPEED = SPEED_NORMAL;\n}\n",
        encoding="utf-8",
    )
    value = parse_config_int(
        "CRUISE_SPEED", line_h, search_headers=(drive_h, line_h)
    )
    assert value == 85


def test_parse_config_int_expression_rhs_raises(tmp_path) -> None:
    """An expression RHS (``A + B``) is not a literal or bare identifier -> ValueError.

    No stale-literal fallback (issue #44): the parser must fail loud rather than
    guessing at operator precedence or silently ignoring the expression.
    """
    cfg = tmp_path / "ConfigDrive.h"
    cfg.write_text(
        "namespace Config {\n"
        "    constexpr int16_t A = 1;\n"
        "    constexpr int16_t B = 2;\n"
        "    constexpr int16_t SUM = A + B;\n"
        "}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="SUM"):
        parse_config_int("SUM", cfg, search_headers=(cfg,))


def test_parse_config_int_two_level_chain_raises(tmp_path) -> None:
    """A two-level identifier chain (RHS resolves to another identifier) fails loud.

    ``parse_config_int`` resolves exactly ONE level of symbolic RHS; if that
    resolves to a second bare identifier instead of a literal, it must raise
    rather than silently chasing the chain further (issue #44 policy, and the
    plan's explicit "fail loud on any expression or multi-level chain").
    """
    base_h = tmp_path / "ConfigGlobal.h"
    base_h.write_text(
        "namespace Config {\n    constexpr int16_t BASE_VALUE = 10;\n}\n",
        encoding="utf-8",
    )
    mid_h = tmp_path / "ConfigDrive.h"
    mid_h.write_text(
        "namespace Config {\n    constexpr int16_t MID_ALIAS = BASE_VALUE;\n}\n",
        encoding="utf-8",
    )
    top_h = tmp_path / "ConfigLine.h"
    top_h.write_text(
        "namespace Config {\n    constexpr int16_t TOP_ALIAS = MID_ALIAS;\n}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="TOP_ALIAS"):
        parse_config_int(
            "TOP_ALIAS", top_h, search_headers=(base_h, mid_h, top_h)
        )


def test_parse_config_int_skips_commented_decl(tmp_path) -> None:
    """A ``//``-commented example above the real decl must not shadow it (#8)."""
    cfg = tmp_path / "ConfigDrive.h"
    cfg.write_text(
        "// constexpr int16_t SPEED_NORMAL = 999;  // example, wrong value\n"
        "namespace Config {\n    constexpr int16_t SPEED_NORMAL = 85;\n}\n",
        encoding="utf-8",
    )
    assert parse_config_int("SPEED_NORMAL", cfg, search_headers=(cfg,)) == 85


def test_parse_config_int_missing_decl_raises(tmp_path) -> None:
    """No matching declaration -> ValueError, never a stale fallback (issue #44)."""
    cfg = tmp_path / "ConfigDrive.h"
    cfg.write_text("namespace Config {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="SPEED_NORMAL"):
        parse_config_int("SPEED_NORMAL", cfg, search_headers=(cfg,))


def test_parse_config_int_unresolvable_symbol_raises(tmp_path) -> None:
    """A symbolic RHS with no matching declaration anywhere in search_headers -> ValueError."""
    drive_h = tmp_path / "ConfigDrive.h"
    drive_h.write_text(
        "namespace Config {\n"
        "    constexpr int16_t TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE;\n"
        "}\n",
        encoding="utf-8",
    )
    global_h = tmp_path / "ConfigGlobal.h"
    global_h.write_text("namespace Config {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="PWM_MIN_MAGNITUDE"):
        parse_config_int(
            "TSHAPE_START_SPEED", drive_h, search_headers=(drive_h, global_h)
        )


def test_parse_config_int_unreadable_header_raises() -> None:
    """A missing header -> OSError, never a stale fallback (issue #44)."""
    with pytest.raises(OSError):
        parse_config_int("SPEED_NORMAL", Path("does/not/exist/ConfigDrive.h"))


def test_parse_config_int_uses_real_firmware_headers() -> None:
    """Sanity check against the real firmware headers (default search_headers).

    Pins the actual shapes this parser exists to serve: ``PWM_MIN_MAGNITUDE`` is
    a plain literal in ``ConfigGlobal.h``, ``TSHAPE_START_SPEED`` aliases it in
    ``ConfigDrive.h``, and ``CRUISE_SPEED`` aliases ``SPEED_NORMAL`` in
    ``ConfigLine.h``. If a firmware retune changes these shapes this test should
    fail loudly rather than silently pass on a stale assumption.
    """
    from tools.path_planner.config_parse import (
        DEFAULT_INT_SEARCH_HEADERS,
        _CONFIG_DIR,
    )

    pwm_min = parse_config_int(
        "PWM_MIN_MAGNITUDE", _CONFIG_DIR / "ConfigGlobal.h"
    )
    assert pwm_min == 50

    tshape_start = parse_config_int(
        "TSHAPE_START_SPEED",
        _CONFIG_DIR / "ConfigDrive.h",
        search_headers=DEFAULT_INT_SEARCH_HEADERS,
    )
    assert tshape_start == pwm_min

    cruise_speed = parse_config_int(
        "CRUISE_SPEED",
        _CONFIG_DIR / "ConfigLine.h",
        search_headers=DEFAULT_INT_SEARCH_HEADERS,
    )
    speed_normal = parse_config_int(
        "SPEED_NORMAL", _CONFIG_DIR / "ConfigDrive.h"
    )
    assert cruise_speed == speed_normal
