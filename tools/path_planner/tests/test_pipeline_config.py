"""TC6 — PipelineConfig self-referential DEFAULT validation.

Finding #51: setting a category field (line/blind/projection/turn) on
PipelineConfig to its own .DEFAULT sentinel is circular and must be rejected.
DirectionMode.DEFAULT is the legitimate intended default for the direction
field and must NOT be rejected.
"""

from __future__ import annotations

import pytest

from tools.path_planner.pipeline_config import (
    BlindMode,
    DirectionMode,
    LineMode,
    PipelineConfig,
    PIPELINE_CONFIG,
    ProjectionMode,
    TurnMode,
)


# ---------------------------------------------------------------------------
# Valid constructions — must not raise
# ---------------------------------------------------------------------------

def test_default_pipeline_config_is_valid() -> None:
    """The module-level PIPELINE_CONFIG singleton must construct without error."""
    cfg = PipelineConfig()
    assert cfg.line       == LineMode.NORMAL
    assert cfg.blind      == BlindMode.SYNC_TSHAPE
    assert cfg.projection == ProjectionMode.NORMAL
    assert cfg.turn       == TurnMode.COUNTER_ROT
    assert cfg.direction  == DirectionMode.DEFAULT


def test_module_level_singleton_is_valid() -> None:
    """PIPELINE_CONFIG at module level must already be valid (import-time check)."""
    assert PIPELINE_CONFIG is not None


def test_direction_default_is_accepted() -> None:
    """DirectionMode.DEFAULT is the legitimate default; must not raise."""
    cfg = PipelineConfig(direction=DirectionMode.DEFAULT)
    assert cfg.direction == DirectionMode.DEFAULT


def test_explicit_non_default_values_accepted() -> None:
    cfg = PipelineConfig(
        line=LineMode.FAST,
        blind=BlindMode.GYRO_TSHAPE,
        projection=ProjectionMode.SLOW,
        turn=TurnMode.PIVOT_LEFT,
        direction=DirectionMode.FORCE_FORWARD,
    )
    assert cfg.line == LineMode.FAST


# ---------------------------------------------------------------------------
# Invalid constructions — must raise ValueError naming the offending field
# ---------------------------------------------------------------------------

def test_line_default_raises() -> None:
    with pytest.raises(ValueError, match=r"PipelineConfig\.line"):
        PipelineConfig(line=LineMode.DEFAULT)


def test_blind_default_raises() -> None:
    with pytest.raises(ValueError, match=r"PipelineConfig\.blind"):
        PipelineConfig(blind=BlindMode.DEFAULT)


def test_projection_default_raises() -> None:
    with pytest.raises(ValueError, match=r"PipelineConfig\.projection"):
        PipelineConfig(projection=ProjectionMode.DEFAULT)


def test_turn_default_raises() -> None:
    with pytest.raises(ValueError, match=r"PipelineConfig\.turn"):
        PipelineConfig(turn=TurnMode.DEFAULT)


def test_error_message_mentions_circular() -> None:
    """Error message should explain that DEFAULT is circular in this context."""
    with pytest.raises(ValueError, match=r"circular"):
        PipelineConfig(line=LineMode.DEFAULT)
