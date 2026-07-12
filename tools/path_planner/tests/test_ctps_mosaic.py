"""Hermetic tests for the CTPS mosaic roll / resolve / validate module (T18).

Exercises :mod:`tools.path_planner.ctps.mosaic` directly (no editor, no player,
no dump) per the plan's Testing Strategy TC1-TC7 and TC9:

* **TC1-TC4** — :func:`roll_mosaic` determinism, random-mode counts, pins kept
  alongside a random fill, and directed-mode pins-verbatim (seed irrelevant).
* **TC5** — :func:`resolve_cells` against HAND-COMPUTED pinned expected vectors
  for a rotated (``heading_deg=90``), unequal-gap (``row_gap_cm != col_gap_cm``)
  config (AC5, the load-bearing geometry test).
* **TC6-TC7** — :func:`validate_mosaic_config` / :func:`validate_vision_cone`
  reject every AC3 rule violation with the offending key named in the message.
* **TC9** — a Python **mirror** of the JS ``visibleCells`` /
  ``readMosaicWouldSucceed`` predicate (see the module docstring below for the
  mirror-vs-``vision.js`` drift caveat required by the plan's No-JS-runner
  note).
"""

from __future__ import annotations

import math
import random
from typing import Any, Dict, List, Optional

import pytest

from tools.path_planner.ctps.mosaic import (
    AIR_LETTER,
    AIR_NAME,
    CELL_COUNT,
    COLOR_LETTER,
    COLOR_NAMES,
    MODE_DIRECTED,
    MODE_RANDOM,
    MosaicConfig,
    color_to_letter,
    resolve_cells,
    roll_mosaic,
    validate_mosaic_config,
    validate_vision_cone,
)

# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------


def _base_geom_kwargs() -> Dict[str, Any]:
    """Return the geometry-only kwargs shared by every :class:`MosaicConfig` fixture."""
    return {
        "center_x_cm": 100.0,
        "center_y_cm": 75.0,
        "heading_deg": 0.0,
        "rows": 4,
        "cols": 3,
        "cell_w_cm": 5.0,
        "cell_h_cm": 5.0,
        "row_gap_cm": 1.0,
        "col_gap_cm": 1.0,
    }


def _random_mosaic_config(
    counts: Dict[str, int],
    cells: Optional[List[Optional[str]]] = None,
    seed: Optional[int] = None,
) -> MosaicConfig:
    """Build a random-mode :class:`MosaicConfig` with the given counts/pins."""
    if cells is None:
        cells = [None] * CELL_COUNT
    return MosaicConfig(
        mode=MODE_RANDOM,
        palette=list(COLOR_NAMES),
        cells=list(cells),
        counts=dict(counts),
        seed=seed,
        **_base_geom_kwargs(),
    )


def _directed_mosaic_config(cells: List[str]) -> MosaicConfig:
    """Build a directed-mode :class:`MosaicConfig` with every cell pinned."""
    return MosaicConfig(
        mode=MODE_DIRECTED,
        palette=list(COLOR_NAMES),
        cells=list(cells),
        counts={},
        seed=None,
        **_base_geom_kwargs(),
    )


def _valid_mosaic_dict() -> Dict[str, Any]:
    """Return a fresh, valid decoded ``mosaic`` block dict (safe to mutate per-test)."""
    return {
        "mode": "random",
        "center_x_cm": 100.0,
        "center_y_cm": 75.0,
        "heading_deg": 0.0,
        "rows": 4,
        "cols": 3,
        "cell_w_cm": 5.0,
        "cell_h_cm": 5.0,
        "row_gap_cm": 1.0,
        "col_gap_cm": 1.0,
        "palette": ["YELLOW", "BLUE", "GREEN", "WHITE"],
        "cells": [None] * CELL_COUNT,
        "counts": {"YELLOW": 3, "BLUE": 3, "GREEN": 4, "WHITE": 2},
        "seed": None,
    }


def _valid_vision_dict() -> Dict[str, Any]:
    """Return a fresh, valid decoded ``robot.vision`` block dict."""
    return {
        "origin_x_cm": 12.0,
        "origin_y_cm": 0.0,
        "half_angle_deg": 25.0,
        "range_cm": None,
    }


# ---------------------------------------------------------------------------
# TC1 — roll_mosaic determinism (same seed -> identical output)
# ---------------------------------------------------------------------------


def test_tc1_roll_mosaic_same_seed_is_deterministic():
    """TC1: roll_mosaic with random.Random(seed) twice yields identical output."""
    counts = {"YELLOW": 3, "BLUE": 3, "GREEN": 4, "WHITE": 2}
    cfg = _random_mosaic_config(counts)

    first = roll_mosaic(cfg, random.Random(1234))
    second = roll_mosaic(cfg, random.Random(1234))

    assert first == second
    assert len(first) == CELL_COUNT


def test_tc1_roll_mosaic_different_seed_can_differ():
    """Sanity companion to TC1: a different seed is very likely to reorder the fill.

    Not itself an acceptance criterion, but guards against a roll_mosaic that
    silently ignores rng (which would make TC1 a false positive).
    """
    counts = {"YELLOW": 3, "BLUE": 3, "GREEN": 4, "WHITE": 2}
    cfg = _random_mosaic_config(counts)

    a = roll_mosaic(cfg, random.Random(1))
    b = roll_mosaic(cfg, random.Random(2))

    assert a != b


# ---------------------------------------------------------------------------
# TC2 — random mode, no pins, output multiset == counts
# ---------------------------------------------------------------------------


def test_tc2_roll_mosaic_random_no_pins_matches_counts_multiset():
    """TC2: random mode, counts={Y:3,B:3,G:4,W:2}, no pins -> output multiset == counts."""
    counts = {"YELLOW": 3, "BLUE": 3, "GREEN": 4, "WHITE": 2}
    cfg = _random_mosaic_config(counts)

    result = roll_mosaic(cfg, random.Random(42))

    assert len(result) == CELL_COUNT
    assert all(color is not None for color in result)
    for name, quota in counts.items():
        assert result.count(name) == quota


# ---------------------------------------------------------------------------
# TC3 — random mode, 2 pins + counts summing to 10 -> pins kept, nulls filled
# ---------------------------------------------------------------------------


def test_tc3_roll_mosaic_random_pins_kept_nulls_match_remaining_multiset():
    """TC3: 2 pins + counts summing to 10 -> pins kept at their indices; nulls == remaining multiset."""
    cells: List[Optional[str]] = [None] * CELL_COUNT
    cells[0] = "YELLOW"
    cells[5] = "WHITE"
    counts = {"YELLOW": 2, "BLUE": 3, "GREEN": 4, "WHITE": 1}  # sums to 10 nulls
    cfg = _random_mosaic_config(counts, cells=cells)

    result = roll_mosaic(cfg, random.Random(7))

    assert len(result) == CELL_COUNT
    # Pins kept verbatim at their indices.
    assert result[0] == "YELLOW"
    assert result[5] == "WHITE"
    # The remaining (null) cells' multiset matches counts exactly.
    null_indices = [i for i in range(CELL_COUNT) if i not in (0, 5)]
    filled = [result[i] for i in null_indices]
    assert len(filled) == 10
    for name, quota in counts.items():
        assert filled.count(name) == quota


# ---------------------------------------------------------------------------
# Roll — random UNDER-fill (sum(counts) < #nulls): leftover cells become AIR (ADR-063)
# ---------------------------------------------------------------------------


def test_roll_random_underfill_pads_leftover_cells_with_air():
    """Under-fill: sum(counts) < #nulls -> the leftover null cells become AIR.

    counts sum to 8 over 12 null cells -> exactly 4 AIR cells; each colour still
    appears exactly its quota.
    """
    counts = {"YELLOW": 2, "BLUE": 2, "GREEN": 2, "WHITE": 2}  # sum 8, 12 nulls
    cfg = _random_mosaic_config(counts)

    result = roll_mosaic(cfg, random.Random(3))

    assert len(result) == CELL_COUNT
    assert result.count(AIR_NAME) == CELL_COUNT - 8  # 4 air cells
    for name, quota in counts.items():
        assert result.count(name) == quota


def test_roll_random_underfill_keeps_pins_air_only_in_null_cells():
    """Under-fill with pins: pins kept verbatim; AIR only ever lands in null cells."""
    cells: List[Optional[str]] = [None] * CELL_COUNT
    cells[0] = "RED"
    cells[11] = "RED"
    counts = {"YELLOW": 1, "BLUE": 1}  # sum 2 over 10 null cells -> 8 air
    cfg = _random_mosaic_config(counts, cells=cells)

    result = roll_mosaic(cfg, random.Random(9))

    # Pins kept verbatim and never overwritten with air.
    assert result[0] == "RED"
    assert result[11] == "RED"
    null_indices = [i for i in range(CELL_COUNT) if i not in (0, 11)]
    filled = [result[i] for i in null_indices]
    assert filled.count(AIR_NAME) == 10 - 2  # 8 air among the 10 null cells
    assert filled.count("YELLOW") == 1
    assert filled.count("BLUE") == 1


def test_roll_random_underfill_is_deterministic():
    """Under-fill roll is deterministic for a fixed seed (air placement included)."""
    counts = {"YELLOW": 2, "BLUE": 2}  # sum 4, 12 nulls -> 8 air
    cfg = _random_mosaic_config(counts)

    assert roll_mosaic(cfg, random.Random(5)) == roll_mosaic(cfg, random.Random(5))


def test_roll_random_empty_counts_is_all_air():
    """sum(counts) == 0 -> every null cell becomes AIR (the extreme under-fill)."""
    cfg = _random_mosaic_config({})

    result = roll_mosaic(cfg, random.Random(1))

    assert result == [AIR_NAME] * CELL_COUNT


# ---------------------------------------------------------------------------
# Roll — random OVER-fill (sum(counts) > #nulls): keep a random subset (ADR-063)
# ---------------------------------------------------------------------------


def test_roll_random_overfill_keeps_exactly_null_count_no_air():
    """Over-fill: sum(counts) > #nulls -> exactly #nulls cells filled, no AIR.

    Each colour appears AT MOST its quota (the excess is dropped at random).
    """
    counts = {"YELLOW": 6, "BLUE": 6, "GREEN": 6}  # sum 18 over 12 null cells
    cfg = _random_mosaic_config(counts)

    result = roll_mosaic(cfg, random.Random(11))

    assert len(result) == CELL_COUNT
    assert AIR_NAME not in result
    assert all(color in counts for color in result)
    for name, quota in counts.items():
        assert result.count(name) <= quota


def test_roll_random_overfill_is_deterministic():
    """Over-fill roll is deterministic for a fixed seed (which units survive)."""
    counts = {"YELLOW": 8, "BLUE": 8}  # sum 16 over 12 null cells
    cfg = _random_mosaic_config(counts)

    assert roll_mosaic(cfg, random.Random(6)) == roll_mosaic(cfg, random.Random(6))


# ---------------------------------------------------------------------------
# color_to_letter — the single-sourced --mosaic wire alphabet incl. the AIR letter
# ---------------------------------------------------------------------------


def test_color_to_letter_maps_palette_and_air():
    """Each palette colour maps to its letter; the AIR sentinel maps to AIR_LETTER."""
    for name, letter in COLOR_LETTER.items():
        assert color_to_letter(name) == letter
    assert color_to_letter(AIR_NAME) == AIR_LETTER
    # A rolled list carrying air joins to a wire string without raising (the
    # orchestrator's _resolve_mosaic_args path).
    rolled = ["YELLOW", AIR_NAME, "RED"]
    assert "".join(color_to_letter(c) for c in rolled) == "Y" + AIR_LETTER + "R"


# ---------------------------------------------------------------------------
# TC4 — directed mode (all 12 pinned) -> output == pins verbatim, seed irrelevant
# ---------------------------------------------------------------------------


def test_tc4_roll_mosaic_directed_equals_pins_verbatim_seed_irrelevant():
    """TC4: directed mode -> output == pins verbatim; two different seeds -> same output."""
    pins = [
        "YELLOW", "BLUE", "GREEN", "WHITE",
        "YELLOW", "BLUE", "GREEN", "WHITE",
        "YELLOW", "BLUE", "GREEN", "WHITE",
    ]
    cfg = _directed_mosaic_config(pins)

    result_seed_a = roll_mosaic(cfg, random.Random(111))
    result_seed_b = roll_mosaic(cfg, random.Random(999))

    assert result_seed_a == pins
    assert result_seed_b == pins
    assert result_seed_a == result_seed_b


# ---------------------------------------------------------------------------
# TC5 — resolve_cells rotated + unequal-gap config vs HAND-COMPUTED vectors
# ---------------------------------------------------------------------------
#
# Config (rotated 90 deg CCW; the two PITCHES are deliberately UNEQUAL so a
# resolve_cells bug that swaps which pitch feeds local_x vs local_y is caught --
# equal pitches would make such a transpose invisible):
#   rows=4, cols=3, cell_w_cm=5.0, cell_h_cm=5.0, row_gap_cm=2.0, col_gap_cm=1.0
#   center=(100.0, 50.0), heading_deg=90.0
#   -> col_pitch = cell_w_cm + col_gap_cm = 6.0
#      row_pitch = cell_h_cm + row_gap_cm = 7.0   (6 != 7 -- axis swap is caught)
#
# Hand derivation from the Data Model formula (mosaic.py docstring):
#   col_mid = (cols-1)/2 = 1.0        row_mid = (rows-1)/2 = 1.5
#   local_x(col) = (col - 1.0) * col_pitch(6) -> col0=-6,   col1=0,  col2=+6
#   local_y(row) = (1.5 - row) * row_pitch(7) -> row0=+10.5, row1=+3.5,
#                                                 row2=-3.5,  row3=-10.5  (row0 at +y "top")
#   heading_deg=90 -> cos_t=0, sin_t=1 -> world_x = center_x - local_y = 100 - local_y
#                                          world_y = center_y + local_x = 50 + local_x
#
# Row0 (local_y=+10.5): world_x = 100-10.5 = 89.5;  cols -6/0/+6 -> world_y = 44/50/56
# Row1 (local_y=+3.5):  world_x = 100-3.5  = 96.5;  world_y = 44/50/56
# Row2 (local_y=-3.5):  world_x = 100+3.5  = 103.5; world_y = 44/50/56
# Row3 (local_y=-10.5): world_x = 100+10.5 = 110.5; world_y = 44/50/56
#
# i = row*cols + col (row-major, row0 top).
_TC5_EXPECTED_VECTORS: Dict[int, Dict[str, float]] = {
    0: {"x_cm": 89.5, "y_cm": 44.0},
    1: {"x_cm": 89.5, "y_cm": 50.0},
    2: {"x_cm": 89.5, "y_cm": 56.0},
    3: {"x_cm": 96.5, "y_cm": 44.0},
    4: {"x_cm": 96.5, "y_cm": 50.0},
    5: {"x_cm": 96.5, "y_cm": 56.0},
    6: {"x_cm": 103.5, "y_cm": 44.0},
    7: {"x_cm": 103.5, "y_cm": 50.0},
    8: {"x_cm": 103.5, "y_cm": 56.0},
    9: {"x_cm": 110.5, "y_cm": 44.0},
    10: {"x_cm": 110.5, "y_cm": 50.0},
    11: {"x_cm": 110.5, "y_cm": 56.0},
}


def test_tc5_resolve_cells_rotated_unequal_gaps_matches_hand_computed_vectors():
    """TC5: resolve_cells for a rotated + unequal-PITCH config matches pinned expected vectors (AC5)."""
    cfg = MosaicConfig(
        mode=MODE_DIRECTED,
        center_x_cm=100.0,
        center_y_cm=50.0,
        heading_deg=90.0,
        rows=4,
        cols=3,
        cell_w_cm=5.0,
        cell_h_cm=5.0,
        row_gap_cm=2.0,
        col_gap_cm=1.0,
        palette=list(COLOR_NAMES),
        cells=["YELLOW"] * CELL_COUNT,
        counts={},
        seed=None,
    )
    color_names = ["YELLOW"] * CELL_COUNT

    cells = resolve_cells(cfg, color_names)

    assert len(cells) == CELL_COUNT
    for cell in cells:
        index = cell["index"]
        expected = _TC5_EXPECTED_VECTORS[index]
        assert cell["x_cm"] == pytest.approx(expected["x_cm"], abs=1e-9)
        assert cell["y_cm"] == pytest.approx(expected["y_cm"], abs=1e-9)
        assert cell["color"] == "YELLOW"
    # Cells are returned in ascending index order.
    assert [cell["index"] for cell in cells] == list(range(CELL_COUNT))


# ---------------------------------------------------------------------------
# TC6 — validate_mosaic_config rejects each malformed shape
# ---------------------------------------------------------------------------


def test_tc6_rejects_rows_times_cols_not_12():
    """TC6: rows*cols != 12 raises naming rows/cols."""
    data = _valid_mosaic_dict()
    data["rows"] = 3
    data["cols"] = 3  # 3*3 = 9 != 12

    with pytest.raises(ValueError) as exc_info:
        validate_mosaic_config(data)

    message = str(exc_info.value)
    assert "rows" in message
    assert "cols" in message


def test_tc6_rejects_cells_length_not_12():
    """TC6: len(cells) != 12 raises naming 'cells'."""
    data = _valid_mosaic_dict()
    data["cells"] = [None] * 10

    with pytest.raises(ValueError) as exc_info:
        validate_mosaic_config(data)

    message = str(exc_info.value)
    assert "cells" in message
    assert "12" in message


def test_tc6_rejects_directed_mode_with_a_null_cell():
    """TC6: directed mode with any null cell raises."""
    data = _valid_mosaic_dict()
    data["mode"] = "directed"
    data["cells"] = ["YELLOW"] * 11 + [None]

    with pytest.raises(ValueError) as exc_info:
        validate_mosaic_config(data)

    message = str(exc_info.value)
    assert "directed" in message
    assert "null" in message


def test_tc6_accepts_random_mode_counts_any_sum():
    """TC6 (ADR-063): random mode accepts counts summing under, over, or to #nulls.

    The old ``sum(counts) == #nulls`` equality is gone: an under-fill leaves the
    leftover cells as air, an over-fill keeps a random subset, and both are
    resolved in ``roll_mosaic`` — so none of these raise at validation.
    """
    under = _valid_mosaic_dict()
    under["counts"] = {"YELLOW": 3, "BLUE": 3, "GREEN": 4, "WHITE": 1}  # sum 11 < 12
    validate_mosaic_config(under)  # must not raise

    over = _valid_mosaic_dict()
    over["counts"] = {"YELLOW": 6, "BLUE": 6, "GREEN": 6, "WHITE": 6}  # sum 24 > 12
    validate_mosaic_config(over)  # must not raise

    empty = _valid_mosaic_dict()
    empty["counts"] = {}  # sum 0 -> every null cell becomes air
    validate_mosaic_config(empty)  # must not raise


def test_tc6_rejects_bad_palette_color():
    """TC6: a palette entry not in COLOR_NAMES raises naming the offending value."""
    data = _valid_mosaic_dict()
    data["palette"] = ["YELLOW", "BLUE", "GREEN", "PURPLE"]

    with pytest.raises(ValueError) as exc_info:
        validate_mosaic_config(data)

    message = str(exc_info.value)
    assert "palette" in message
    assert "PURPLE" in message


def test_tc6_rejects_bad_pin_color():
    """TC6: a pinned cell color not in the palette raises naming the offending value."""
    data = _valid_mosaic_dict()
    data["cells"][0] = "PURPLE"
    # 11 nulls remain; adjust counts to keep the (unrelated) sum rule satisfied
    # so the pin-color rule is exercised in isolation.
    data["counts"] = {"YELLOW": 3, "BLUE": 3, "GREEN": 3, "WHITE": 2}

    with pytest.raises(ValueError) as exc_info:
        validate_mosaic_config(data)

    message = str(exc_info.value)
    assert "cells[0]" in message
    assert "PURPLE" in message


def test_tc6_rejects_bad_counts_color():
    """TC6: a counts key not in COLOR_NAMES raises naming the offending key."""
    data = _valid_mosaic_dict()
    data["counts"] = {"YELLOW": 3, "BLUE": 3, "GREEN": 4, "PURPLE": 2}

    with pytest.raises(ValueError) as exc_info:
        validate_mosaic_config(data)

    message = str(exc_info.value)
    assert "counts" in message
    assert "PURPLE" in message


# ---------------------------------------------------------------------------
# TC7 — validate_vision_cone / validate_mosaic_config numeric-range rejections
# ---------------------------------------------------------------------------


def test_tc7_rejects_half_angle_deg_zero():
    """TC7: half_angle_deg == 0 raises (must be strictly in (0, 90))."""
    data = _valid_vision_dict()
    data["half_angle_deg"] = 0.0

    with pytest.raises(ValueError) as exc_info:
        validate_vision_cone(data)

    assert "half_angle_deg" in str(exc_info.value)


def test_tc7_rejects_half_angle_deg_95():
    """TC7: half_angle_deg == 95 raises (outside (0, 90))."""
    data = _valid_vision_dict()
    data["half_angle_deg"] = 95.0

    with pytest.raises(ValueError) as exc_info:
        validate_vision_cone(data)

    assert "half_angle_deg" in str(exc_info.value)


def test_tc7_rejects_range_cm_zero():
    """TC7: range_cm == 0 raises (must be null or > 0)."""
    data = _valid_vision_dict()
    data["range_cm"] = 0.0

    with pytest.raises(ValueError) as exc_info:
        validate_vision_cone(data)

    assert "range_cm" in str(exc_info.value)


def test_tc7_rejects_seed_negative_one():
    """TC7: mosaic.seed == -1 raises (must be null or a non-negative int)."""
    data = _valid_mosaic_dict()
    data["seed"] = -1

    with pytest.raises(ValueError) as exc_info:
        validate_mosaic_config(data)

    assert "seed" in str(exc_info.value)


def test_tc7_rejects_seed_one_point_five():
    """TC7: mosaic.seed == 1.5 raises (a float, even if integral-looking, is rejected)."""
    data = _valid_mosaic_dict()
    data["seed"] = 1.5

    with pytest.raises(ValueError) as exc_info:
        validate_mosaic_config(data)

    assert "seed" in str(exc_info.value)


# ---------------------------------------------------------------------------
# TC9 — Python MIRROR of the JS visibleCells / readMosaicWouldSucceed predicate
# ---------------------------------------------------------------------------
#
# NOTE (No-JS-runner note, plan Testing Strategy): this is a HAND-MAINTAINED
# MIRROR of `tools/path_planner/ctps/assets/vision.js`'s `visibleCells` /
# `readMosaicWouldSucceed`. There is NO automated JS runner in this repo (CTPS
# ships plain <script> assets with no node/jsdom harness) -- this Python
# reimplementation is kept deliberately small and point-in-sector, following
# the Contracts "JS predicate API" spec EXACTLY, so it stays credible against
# vision.js. The mirror and vision.js CAN DRIFT if one is edited without the
# other: T24's localhost eyeball verification (which explicitly exercises a
# boundary case -- a cell just inside vs. just outside the half-angle) is what
# guards the REAL JS implementation; this mirror only guards the ALGORITHM.


def _local_to_world(
    pose_x_cm: float, pose_y_cm: float, pose_heading_deg: float, local_x_cm: float, local_y_cm: float
) -> tuple[float, float]:
    """Mirror of vision.js's localToWorld: rotate a robot-local offset by pose.heading and add pose position."""
    theta = math.radians(pose_heading_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    world_x = pose_x_cm + (local_x_cm * cos_t - local_y_cm * sin_t)
    world_y = pose_y_cm + (local_x_cm * sin_t + local_y_cm * cos_t)
    return world_x, world_y


def _visible_cells(
    pose: Dict[str, float], vision_cfg: Dict[str, Any], cells: List[Dict[str, float]]
) -> List[bool]:
    """Mirror of vision.js's visibleCells(pose, visionCfg, cells) -> bool[len(cells)].

    Per the Contracts "JS predicate API" spec: apex = localToWorld(pose,
    origin_x_cm, origin_y_cm); per cell, in robot-local coordinates (world
    delta rotated by Rot(-pose.heading)): visible iff local_x > 0 AND
    abs(degrees(atan2(local_y, local_x))) <= half_angle_deg AND (range_cm is
    None OR hypot(local_x, local_y) <= range_cm).
    """
    apex_x, apex_y = _local_to_world(
        pose["x_cm"], pose["y_cm"], pose["heading_deg"],
        vision_cfg["origin_x_cm"], vision_cfg["origin_y_cm"],
    )
    theta = math.radians(pose["heading_deg"])
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    half_angle_deg = vision_cfg["half_angle_deg"]
    range_cm = vision_cfg.get("range_cm")

    results: List[bool] = []
    for cell in cells:
        world_dx = cell["x_cm"] - apex_x
        world_dy = cell["y_cm"] - apex_y
        # Rot(-theta) . (world_dx, world_dy)
        local_x = world_dx * cos_t + world_dy * sin_t
        local_y = -world_dx * sin_t + world_dy * cos_t

        visible = local_x > 0
        if visible:
            angle_deg = math.degrees(math.atan2(local_y, local_x))
            visible = abs(angle_deg) <= half_angle_deg
        if visible and range_cm is not None:
            visible = math.hypot(local_x, local_y) <= range_cm
        results.append(visible)
    return results


def _read_mosaic_would_succeed(visibility: List[bool]) -> bool:
    """Mirror of vision.js's readMosaicWouldSucceed(bools) -> all(bools)."""
    return all(visibility)


_TC9_POSE = {"x_cm": 0.0, "y_cm": 0.0, "heading_deg": 0.0}
_TC9_VISION_CFG = {
    "origin_x_cm": 0.0,
    "origin_y_cm": 0.0,
    "half_angle_deg": 25.0,
    "range_cm": 100.0,
}


def test_tc9_cell_dead_ahead_in_range_is_visible():
    """TC9: a cell dead-ahead, within range and angle, is visible."""
    cell = {"x_cm": 50.0, "y_cm": 0.0}

    result = _visible_cells(_TC9_POSE, _TC9_VISION_CFG, [cell])

    assert result == [True]


def test_tc9_cell_directly_behind_is_not_visible():
    """TC9: a cell directly behind the apex (local_x <= 0) is never visible."""
    cell = {"x_cm": -50.0, "y_cm": 0.0}

    result = _visible_cells(_TC9_POSE, _TC9_VISION_CFG, [cell])

    assert result == [False]


def test_tc9_cell_just_outside_half_angle_is_not_visible():
    """TC9: a cell at 26 deg (outside a 25 deg half-angle) is not visible."""
    r = 50.0
    angle_deg = 26.0
    cell = {
        "x_cm": r * math.cos(math.radians(angle_deg)),
        "y_cm": r * math.sin(math.radians(angle_deg)),
    }

    result = _visible_cells(_TC9_POSE, _TC9_VISION_CFG, [cell])

    assert result == [False]


def test_tc9_cell_just_inside_half_angle_is_visible():
    """Companion to the above: a cell at 24 deg (inside a 25 deg half-angle) IS visible."""
    r = 50.0
    angle_deg = 24.0
    cell = {
        "x_cm": r * math.cos(math.radians(angle_deg)),
        "y_cm": r * math.sin(math.radians(angle_deg)),
    }

    result = _visible_cells(_TC9_POSE, _TC9_VISION_CFG, [cell])

    assert result == [True]


def test_tc9_cell_beyond_range_is_not_visible():
    """TC9: a cell dead-ahead but beyond range_cm is not visible."""
    cell = {"x_cm": 150.0, "y_cm": 0.0}  # range_cm=100.0

    result = _visible_cells(_TC9_POSE, _TC9_VISION_CFG, [cell])

    assert result == [False]


def test_tc9_read_mosaic_would_succeed_true_at_12_of_12():
    """TC9: readMosaicWouldSucceed is True when all 12 cells are visible."""
    visibility = [True] * CELL_COUNT

    assert _read_mosaic_would_succeed(visibility) is True


def test_tc9_read_mosaic_would_succeed_false_at_11_of_12():
    """TC9: readMosaicWouldSucceed is False when even one of 12 cells is not visible."""
    visibility = [True] * (CELL_COUNT - 1) + [False]

    assert _read_mosaic_would_succeed(visibility) is False


def test_tc9_unbounded_range_cm_none_does_not_reject_a_distant_cell():
    """TC9: range_cm=None (unbounded) never rejects on distance, only angle/forward."""
    vision_cfg = dict(_TC9_VISION_CFG)
    vision_cfg["range_cm"] = None
    cell = {"x_cm": 1000.0, "y_cm": 0.0}

    result = _visible_cells(_TC9_POSE, vision_cfg, [cell])

    assert result == [True]
