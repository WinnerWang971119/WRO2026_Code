"""Tests for the config_parse ``uint32_t`` / ``#define``-macro extension (T5, TC13).

``HOP_SETTLE_MS`` in ``WRO2026/config/ConfigNav.h`` differs from every constant
:mod:`tools.path_planner.config_parse` previously handled in two ways: it is a
``constexpr uint32_t`` (the type alternation was only ``int16_t``/``uint8_t``),
and its RHS is the symbolic identifier ``HOP_SETTLE_MS_DEFAULT``, which resolves
through a preprocessor ``#define HOP_SETTLE_MS_DEFAULT 100`` — not a
``constexpr`` declaration, the only symbolic-RHS shape the resolver previously
followed. These tests pin:

- the existing ``int16_t``/``uint8_t`` constexpr->constexpr path still resolves
  unchanged (regression guard: :func:`test_tshape_start_speed_still_resolves_unchanged`,
  :func:`test_plain_literal_constant_still_resolves_unchanged`);
- ``HOP_SETTLE_MS`` resolves through the real ``ConfigNav.h`` ``#define`` macro
  to ``100`` (:func:`test_hop_settle_ms_resolves_through_define_macro`), and
  that :func:`tools.path_planner.ctps.profile.load_profile_config` surfaces the
  same value on :class:`~tools.path_planner.ctps.profile.ProfileConfig`
  (:func:`test_load_profile_config_hop_settle_ms`);
- a ``#define`` whose own RHS is a second identifier (a two-level chain) fails
  loud, exactly like the existing ``constexpr`` two-level-chain guard
  (:func:`test_define_two_level_chain_raises`).
"""

from __future__ import annotations

import pytest

from tools.path_planner.config_parse import (
    DEFAULT_INT_SEARCH_HEADERS,
    _CONFIG_DIR,
    parse_config_int,
)
from tools.path_planner.ctps.profile import load_profile_config


# ---------------------------------------------------------------------------------------
# Regression guard: existing constants (constexpr->constexpr, plain literal) still parse
# unchanged after the uint32_t type alternation + #define fallback were added.
# ---------------------------------------------------------------------------------------


def test_tshape_start_speed_still_resolves_unchanged() -> None:
    """``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE`` still resolves constexpr->constexpr.

    This is the case the plan explicitly calls out as NOT a drop-in of the new
    ``#define`` fallback -- both sides of this chain are ``constexpr`` in the
    real headers, so the resolver must still find the match on the FIRST
    (``constexpr``) attempt and never fall through to the ``#define`` regex.
    """
    drive_h = _CONFIG_DIR / "ConfigDrive.h"
    global_h = _CONFIG_DIR / "ConfigGlobal.h"

    pwm_min = parse_config_int("PWM_MIN_MAGNITUDE", global_h)
    tshape_start = parse_config_int(
        "TSHAPE_START_SPEED", drive_h, search_headers=DEFAULT_INT_SEARCH_HEADERS
    )
    assert tshape_start == pwm_min


def test_plain_literal_constant_still_resolves_unchanged() -> None:
    """A plain ``constexpr int16_t NAME = <literal>;`` constant still parses directly."""
    global_h = _CONFIG_DIR / "ConfigGlobal.h"
    assert parse_config_int("PWM_MIN_MAGNITUDE", global_h) == 45


# ---------------------------------------------------------------------------------------
# HOP_SETTLE_MS: uint32_t type + #define-macro RHS resolution (the crux of T5).
# ---------------------------------------------------------------------------------------


def test_hop_settle_ms_resolves_through_define_macro() -> None:
    """``HOP_SETTLE_MS`` (uint32_t, RHS = ``HOP_SETTLE_MS_DEFAULT``) resolves to 100.

    ``HOP_SETTLE_MS_DEFAULT`` is a preprocessor ``#define`` in ``ConfigNav.h``,
    not a ``constexpr`` -- this is the shape the resolver could not previously
    match at all (neither the ``uint32_t`` type nor the ``#define`` RHS).
    """
    nav_h = _CONFIG_DIR / "ConfigNav.h"
    search = DEFAULT_INT_SEARCH_HEADERS + (nav_h,)
    assert parse_config_int("HOP_SETTLE_MS", nav_h, search) == 100


def test_load_profile_config_hop_settle_ms() -> None:
    """``ProfileConfig.hop_settle_ms`` is populated from the real ``ConfigNav.h``."""
    cfg = load_profile_config()
    assert cfg.hop_settle_ms == 100


# ---------------------------------------------------------------------------------------
# Fail-loud: a #define whose own RHS is another identifier is a two-level chain.
# ---------------------------------------------------------------------------------------


def test_define_two_level_chain_raises(tmp_path) -> None:
    """A ``#define`` RHS that is itself a bare identifier fails loud (issue #44).

    Mirrors the existing ``constexpr`` two-level-chain guard: the resolver
    follows exactly ONE level of symbolic RHS, whether that level is a
    ``constexpr`` declaration or a ``#define`` -- it must never chase a second
    hop, in either shape.
    """
    nav_h = tmp_path / "ConfigNav.h"
    nav_h.write_text(
        "namespace Config {\n"
        "#define HOP_SETTLE_MS_DEFAULT HOP_SETTLE_MS_ALIAS\n"
        "#define HOP_SETTLE_MS_ALIAS 100\n"
        "    constexpr uint32_t HOP_SETTLE_MS = HOP_SETTLE_MS_DEFAULT;\n"
        "}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="HOP_SETTLE_MS_DEFAULT"):
        parse_config_int("HOP_SETTLE_MS", nav_h, search_headers=(nav_h,))
