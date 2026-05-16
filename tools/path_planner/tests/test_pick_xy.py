"""Tests for tools.path_planner.pick_xy."""

from __future__ import annotations

import os
import types

import pytest

from tools.path_planner import field_data
from tools.path_planner import pick_xy


def test_field_extent_cm_2500_1000():
    """TC1: FIELD_MM=(2500,1000) → (250.0, 100.0) cm."""
    stub = types.SimpleNamespace(FIELD_MM=(2500, 1000))
    assert pick_xy._field_extent_cm(stub) == (250.0, 100.0)


def test_field_extent_cm_3000_2000():
    """TC2: FIELD_MM=(3000,2000) → (300.0, 200.0) cm."""
    stub = types.SimpleNamespace(FIELD_MM=(3000, 2000))
    assert pick_xy._field_extent_cm(stub) == (300.0, 200.0)


def test_format_stdout_rounds_to_three_dp():
    """TC3: _format_stdout(12.3456, 45.6789) → '(12.346, 45.679)'."""
    assert pick_xy._format_stdout(12.3456, 45.6789) == "(12.346, 45.679)"


def test_format_stdout_zero():
    """TC4: _format_stdout(0.0, 0.0) → '(0.000, 0.000)'."""
    assert pick_xy._format_stdout(0.0, 0.0) == "(0.000, 0.000)"


def test_format_label_rounds_to_one_dp():
    """TC5: _format_label(12.345, 45.678) → '(12.3, 45.7)'."""
    assert pick_xy._format_label(12.345, 45.678) == "(12.3, 45.7)"


def test_resolve_map_png_returns_absolute_path_to_maps_map_png():
    """TC6: _resolve_map_png(field_data) returns absolute path ending in maps/map.png."""
    resolved = pick_xy._resolve_map_png(field_data)
    assert resolved is not None
    assert resolved.is_absolute()
    # Normalise separators so the suffix check works on Windows + POSIX.
    suffix = os.path.join("maps", "map.png")
    assert str(resolved).endswith(suffix)


def test_resolve_map_png_none_when_missing():
    """TC7: _resolve_map_png on stub with MAP_PNG=None returns None."""
    stub = types.SimpleNamespace(MAP_PNG=None)
    assert pick_xy._resolve_map_png(stub) is None
