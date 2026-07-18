"""Hermetic tests for the CTPS field-object + manipulator layout schema.

Exercises :func:`tools.path_planner.ctps.layout.load` /
:func:`tools.path_planner.ctps.layout.validate` against a hand-authored
``layout.json``-shaped dict (schema_version 1). No editor, no player — pure
parse/validate.

Covered cases (CTPS object-cargo plan, T2):

* **P-TC1** — a valid layout round-trips via :meth:`Layout.to_dict`; every
  numeric field is a real ``float`` (not left as a JSON ``int``/``str``), and
  the rect/circle shape-dimension fields are mutually exclusive.
* **P-TC2** — a bad object (missing ``type``, a bad ``shape``, or an
  out-of-range dimension) raises :class:`LayoutError` with a specific,
  precisely-worded message naming the offending key path.

Also covers: :func:`load` accepting a file path (in addition to a dict), a
duplicate object id, a bad ``robot.drop_points`` count, and
:meth:`Layout.validate` re-validating after in-place mutation.
"""

from __future__ import annotations

import copy
import json

import pytest

from tools.path_planner.ctps.layout import LayoutError, load, validate

# ---------------------------------------------------------------------------
# Shared fixture: a valid, fully-populated layout dict (schema_version 1)
# ---------------------------------------------------------------------------


def _valid_layout_dict() -> dict:
    """Return a fresh, valid layout dict (safe to mutate per-test)."""
    return {
        "schema_version": 1,
        "objects": [
            {
                "id": "o1",
                "name": "green tile",
                "shape": "rect",
                "type": "tile",
                "color": "#57b36b",
                "x_cm": 40.0,
                "y_cm": 120.0,
                "heading_deg": 0.0,
                "w_cm": 5.0,
                "h_cm": 5.0,
            },
            {
                "id": "o2",
                "name": "red cargo",
                "shape": "circle",
                "type": "cargo",
                "color": "#c0392b",
                "x_cm": 60.0,
                "y_cm": 100.0,
                "heading_deg": 0.0,
                "radius_cm": 3.0,
            },
            {
                "id": "o3",
                "name": "pillar",
                "shape": "rect",
                "type": "fixed",
                "color": "#888888",
                "x_cm": 10.0,
                "y_cm": 10.0,
                "heading_deg": 45.0,
                "w_cm": 8.0,
                "h_cm": 8.0,
            },
        ],
        "robot": {
            "grab_zone": {"x_cm": -1.0, "y_cm": 8.0, "w_cm": 6.0, "h_cm": 14.0},
            "box_zone": {"x_cm": -9.0, "y_cm": 0.0, "w_cm": 6.0, "h_cm": 16.0},
            "drop_points": [
                {"x_cm": 4.0, "y_cm": -6.0},
                {"x_cm": 4.0, "y_cm": -2.0},
                {"x_cm": 4.0, "y_cm": 2.0},
                {"x_cm": 4.0, "y_cm": 6.0},
            ],
        },
    }


# ---------------------------------------------------------------------------
# P-TC1 — a valid layout round-trips via to_dict; fields typed
# ---------------------------------------------------------------------------


def test_p_tc1_valid_layout_round_trips_via_to_dict():
    """P-TC1: a valid layout loads, is fully typed, and round-trips via to_dict."""
    data = _valid_layout_dict()

    layout = load(copy.deepcopy(data))

    assert layout.schema_version == 1
    assert len(layout.objects) == 3

    # -- rect object: w_cm/h_cm typed float, radius_cm absent (None) ---------
    tile = layout.objects[0]
    assert tile.id == "o1"
    assert tile.shape == "rect"
    assert tile.type == "tile"
    assert isinstance(tile.x_cm, float)
    assert isinstance(tile.y_cm, float)
    assert isinstance(tile.heading_deg, float)
    assert isinstance(tile.w_cm, float)
    assert isinstance(tile.h_cm, float)
    assert tile.radius_cm is None

    # -- circle object: radius_cm typed float, w_cm/h_cm absent (None) -------
    cargo = layout.objects[1]
    assert cargo.shape == "circle"
    assert cargo.type == "cargo"
    assert isinstance(cargo.radius_cm, float)
    assert cargo.w_cm is None
    assert cargo.h_cm is None

    # -- robot config: zones + exactly 4 drop points, all typed float -------
    assert isinstance(layout.robot.grab_zone.w_cm, float)
    assert isinstance(layout.robot.box_zone.h_cm, float)
    assert len(layout.robot.drop_points) == 4
    assert all(isinstance(point.x_cm, float) for point in layout.robot.drop_points)
    assert all(isinstance(point.y_cm, float) for point in layout.robot.drop_points)

    # -- loss-free round-trip -------------------------------------------------
    assert layout.to_dict() == data
    # And the round-tripped dict itself re-validates and re-parses cleanly
    # (a save->load cycle through JSON text, not just the in-memory dict).
    reloaded = load(json.loads(json.dumps(layout.to_dict())))
    assert reloaded.to_dict() == data


def test_load_accepts_a_file_path_and_a_dict_identically(tmp_path):
    """load() accepts a path OR a dict, producing an identical Layout."""
    data = _valid_layout_dict()
    path = tmp_path / "layout.json"
    path.write_text(json.dumps(data), encoding="utf-8")

    from_path = load(path)
    from_dict = load(copy.deepcopy(data))

    assert from_path.to_dict() == from_dict.to_dict() == data


def test_load_accepts_a_raw_json_string():
    """load() also accepts an already-read raw JSON string (not a file path)."""
    data = _valid_layout_dict()
    layout = load(json.dumps(data))
    assert layout.to_dict() == data


# ---------------------------------------------------------------------------
# P-TC2 — rejects a bad object with a specific message
# ---------------------------------------------------------------------------


def test_p_tc2_rejects_object_missing_type():
    """P-TC2: an object missing 'type' raises naming the missing key."""
    data = _valid_layout_dict()
    del data["objects"][0]["type"]

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert "objects[0]" in message
    assert "missing required key" in message
    assert "type" in message


def test_p_tc2_rejects_object_bad_shape():
    """P-TC2: an object with an unrecognised 'shape' raises naming the value."""
    data = _valid_layout_dict()
    data["objects"][0]["shape"] = "triangle"

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert "objects[0].shape" in message
    assert "triangle" in message


def test_p_tc2_rejects_object_bad_type():
    """P-TC2: an object with an unrecognised 'type' raises naming the value."""
    data = _valid_layout_dict()
    data["objects"][0]["type"] = "boulder"

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert "objects[0].type" in message
    assert "boulder" in message


def test_p_tc2_rejects_object_out_of_range_rect_dimension():
    """P-TC2: a non-positive rect dimension raises naming the offending field."""
    data = _valid_layout_dict()
    data["objects"][0]["w_cm"] = -5.0

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert "objects[0].w_cm" in message
    assert "> 0" in message


def test_p_tc2_rejects_object_out_of_range_circle_radius():
    """P-TC2: a zero circle radius raises naming the offending field."""
    data = _valid_layout_dict()
    data["objects"][1]["radius_cm"] = 0.0

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert "objects[1].radius_cm" in message
    assert "> 0" in message


def test_p_tc2_rejects_object_non_finite_coordinate():
    """P-TC2: a NaN/Infinity coordinate raises rather than silently corrupting the scene."""
    data = _valid_layout_dict()
    data["objects"][0]["x_cm"] = float("nan")

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert "objects[0].x_cm" in message
    assert "finite" in message


def test_p_tc2_rejects_duplicate_object_id():
    """P-TC2: a repeated object id raises naming the duplicate."""
    data = _valid_layout_dict()
    data["objects"][1]["id"] = data["objects"][0]["id"]

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert "duplicate" in message.lower()
    assert "o1" in message


def test_p_tc2_rejects_bad_hex_color():
    """P-TC2: a malformed hex color raises naming the offending value."""
    data = _valid_layout_dict()
    data["objects"][0]["color"] = "green"

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert "objects[0].color" in message
    assert "green" in message


def test_p_tc2_rejects_wrong_drop_point_count():
    """P-TC2: robot.drop_points not matching MAGAZINE_LANES raises naming the count."""
    data = _valid_layout_dict()
    data["robot"]["drop_points"].pop()

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert "drop_points" in message
    assert "4" in message


def test_p_tc2_rejects_missing_robot_section():
    """P-TC2: a layout with no 'robot' section raises naming the missing section."""
    data = _valid_layout_dict()
    del data["robot"]

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    assert "robot" in str(exc_info.value)


def test_p_tc2_rejects_unsupported_schema_version():
    """P-TC2: an unsupported schema_version raises naming the expected value."""
    data = _valid_layout_dict()
    data["schema_version"] = 2

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert "schema_version" in message
    assert "2" in message


def test_layout_validate_method_raises_after_in_place_mutation():
    """Layout.validate() re-checks in-place mutation of an already-loaded Layout."""
    layout = load(_valid_layout_dict())
    layout.objects[0].shape = "triangle"

    with pytest.raises(LayoutError):
        layout.validate()


# ---------------------------------------------------------------------------
# Mosaic + robot.vision additions (T19 -- CTPS randomized-vision plan, AC1/AC3)
# ---------------------------------------------------------------------------
#
# These exercise the *layout.py boundary* only: that a valid `mosaic` /
# `robot.vision` block loads and round-trips, that an absent block is fully
# omitted (AC12 guard), and that every AC3 rule violation is surfaced as a
# `LayoutError` naming the key path. The rules THEMSELVES (rows*cols==12,
# counts-sum, palette membership, seed shape, half_angle_deg range, ...) are
# unit-tested directly against `mosaic.py` in test_ctps_mosaic.py (T18,
# TC6/TC7) -- this file only proves `layout.py` wires them up and re-raises
# with its `ctps.layout:` key-path framing.


def _random_mosaic_block() -> dict:
    """Return a fresh, valid *random*-mode `mosaic` block dict."""
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
        "cells": [None, "YELLOW", None, None, None, None, None, "BLUE", None, None, None, None],
        "counts": {"YELLOW": 3, "BLUE": 3, "GREEN": 4},
        "seed": 12345,
    }


def _directed_mosaic_block() -> dict:
    """Return a fresh, valid *directed*-mode `mosaic` block dict (all 12 pinned).

    Includes `counts` (summing to 0, matching 0 null cells) even though
    directed mode ignores it -- `MosaicConfig.to_dict` always emits a `counts`
    key, so a directed input that OMITS `counts` would fail a first-pass
    byte-identity round-trip (it normalizes missing `counts` to `{}`). Pinning
    `counts: {}` here up front keeps the round-trip assertion honest without
    relying on the idempotent-second-pass workaround.
    """
    return {
        "mode": "directed",
        "center_x_cm": 40.0,
        "center_y_cm": 20.0,
        "heading_deg": 0.0,
        "rows": 4,
        "cols": 3,
        "cell_w_cm": 5.0,
        "cell_h_cm": 5.0,
        "row_gap_cm": 1.0,
        "col_gap_cm": 1.0,
        "palette": ["YELLOW", "BLUE", "GREEN", "WHITE"],
        "cells": [
            "YELLOW", "BLUE", "GREEN", "WHITE",
            "YELLOW", "BLUE", "GREEN", "WHITE",
            "YELLOW", "BLUE", "GREEN", "WHITE",
        ],
        "counts": {},
        "seed": None,
    }


def _valid_vision_block() -> dict:
    """Return a fresh, valid `robot.vision` cone block dict."""
    return {
        "origin_x_cm": 12.0,
        "origin_y_cm": 0.0,
        "half_angle_deg": 25.0,
        "range_cm": 60.0,
    }


def _layout_with_mosaic_and_vision(mosaic_block: dict) -> dict:
    """Return a valid layout dict carrying the given `mosaic` block + a vision cone."""
    data = _valid_layout_dict()
    data["mosaic"] = mosaic_block
    data["robot"]["vision"] = _valid_vision_block()
    return data


# -- Case 1: valid mosaic (random + directed) + valid vision cone; round-trips --


def test_mosaic_and_vision_random_mode_round_trips_through_load_to_dict_load():
    """A layout with a valid RANDOM-mode mosaic + robot.vision loads and round-trips stably."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())

    layout = load(copy.deepcopy(data))

    assert layout.mosaic is not None
    assert layout.mosaic.mode == "random"
    assert layout.robot.vision is not None
    assert layout.robot.vision.half_angle_deg == 25.0
    assert layout.robot.vision.range_cm == 60.0

    assert layout.to_dict() == data

    # Round-trip through actual JSON text, then reload -- stable second pass.
    reloaded = load(json.loads(json.dumps(layout.to_dict())))
    assert reloaded.to_dict() == data


def test_mosaic_and_vision_directed_mode_round_trips_through_load_to_dict_load():
    """A layout with a valid DIRECTED-mode (all-pinned) mosaic + robot.vision round-trips.

    `counts: {}` is included explicitly in the directed input (see
    `_directed_mosaic_block`'s docstring) so first-pass byte-identity holds --
    this test does not rely on the idempotent-second-round-trip workaround.
    """
    data = _layout_with_mosaic_and_vision(_directed_mosaic_block())

    layout = load(copy.deepcopy(data))

    assert layout.mosaic is not None
    assert layout.mosaic.mode == "directed"
    assert layout.mosaic.cells == data["mosaic"]["cells"]
    assert layout.robot.vision is not None

    assert layout.to_dict() == data

    reloaded = load(json.loads(json.dumps(layout.to_dict())))
    assert reloaded.to_dict() == data


# -- Case 2: absent mosaic/vision -- still loads, to_dict() OMITS both keys (AC12) --


def test_mosaic_and_vision_absent_loads_and_to_dict_omits_both_keys():
    """A mosaic-free / vision-free layout loads; to_dict() omits both keys entirely.

    Byte-identical to the no-mosaic input -- this is the AC12 guard: a
    mosaic-free/vision-free layout must never gain a `"mosaic"` or
    `"vision"` key (nor emit either as an explicit `None`).
    """
    data = _valid_layout_dict()
    assert "mosaic" not in data
    assert "vision" not in data["robot"]

    layout = load(copy.deepcopy(data))

    assert layout.mosaic is None
    assert layout.robot.vision is None

    out = layout.to_dict()
    assert "mosaic" not in out
    assert "vision" not in out["robot"]
    assert out == data


# -- Case 3: rejections -- each raises LayoutError naming the key path ----------


def test_rejects_mosaic_rows_times_cols_not_12():
    """mosaic: rows*cols != 12 raises LayoutError naming rows/cols."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["mosaic"]["rows"] = 3
    data["mosaic"]["cols"] = 3  # 3*3 = 9 != 12

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "mosaic" in message
    assert "rows" in message
    assert "cols" in message


def test_rejects_mosaic_cells_length_not_12():
    """mosaic: cells length != 12 raises LayoutError naming 'cells'."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["mosaic"]["cells"] = [None] * 10

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "mosaic.cells" in message
    assert "12" in message


def test_rejects_mosaic_directed_mode_with_a_null_cell():
    """mosaic: directed mode with a null cell raises LayoutError."""
    data = _layout_with_mosaic_and_vision(_directed_mosaic_block())
    data["mosaic"]["cells"][-1] = None

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "mosaic" in message
    assert "directed" in message
    assert "null" in message


def test_accepts_mosaic_random_mode_counts_any_sum():
    """mosaic: random mode counts may sum under OR over #null cells (ADR-063) -> loads.

    layout.py wires up mosaic.validate_mosaic_config, whose counts-sum equality
    was relaxed in ADR-063 (under-fill leaves leftover cells as air, over-fill
    keeps a random subset). Neither an under-fill nor an over-fill raises a
    LayoutError — this proves the relaxation flows through the layout layer.
    """
    # 10 nulls in this fixture's cells; counts sum to 6 (under-fill).
    under = _layout_with_mosaic_and_vision(_random_mosaic_block())
    under["mosaic"]["counts"] = {"YELLOW": 3, "BLUE": 3}
    load(under)  # must not raise

    # counts sum to 16 (over-fill).
    over = _layout_with_mosaic_and_vision(_random_mosaic_block())
    over["mosaic"]["counts"] = {"YELLOW": 8, "BLUE": 8}
    load(over)  # must not raise


def test_rejects_mosaic_pin_color_not_in_palette():
    """mosaic: a pinned cell color not in the palette raises LayoutError naming it."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["mosaic"]["cells"][1] = "PURPLE"  # was "YELLOW", not in a shrunk palette below
    data["mosaic"]["palette"] = ["BLUE", "GREEN", "WHITE"]

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "mosaic.cells" in message
    assert "PURPLE" in message


def test_rejects_mosaic_counts_color_not_in_palette():
    """mosaic: a counts key not in the palette raises LayoutError naming it."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["mosaic"]["counts"] = {"YELLOW": 3, "BLUE": 3, "PURPLE": 4}

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "mosaic.counts" in message
    assert "PURPLE" in message


def test_rejects_mosaic_palette_color_not_in_color_names():
    """mosaic: a palette entry not in COLOR_NAMES raises LayoutError naming it."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["mosaic"]["palette"] = ["YELLOW", "BLUE", "GREEN", "PURPLE"]

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "mosaic.palette" in message
    assert "PURPLE" in message


def test_rejects_mosaic_seed_negative_one():
    """mosaic: seed == -1 raises LayoutError (must be null or non-negative int)."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["mosaic"]["seed"] = -1

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "mosaic.seed" in message


def test_rejects_mosaic_seed_one_point_five():
    """mosaic: seed == 1.5 raises LayoutError (a non-integer float is rejected)."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["mosaic"]["seed"] = 1.5

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "mosaic.seed" in message


def test_rejects_vision_half_angle_deg_zero():
    """robot.vision: half_angle_deg == 0 raises LayoutError (must be strictly in (0, 90))."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["robot"]["vision"]["half_angle_deg"] = 0.0

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "robot.vision.half_angle_deg" in message


def test_rejects_vision_half_angle_deg_95():
    """robot.vision: half_angle_deg == 95 raises LayoutError (outside (0, 90))."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["robot"]["vision"]["half_angle_deg"] = 95.0

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "robot.vision.half_angle_deg" in message


def test_rejects_vision_range_cm_zero():
    """robot.vision: range_cm == 0 raises LayoutError (must be null or > 0)."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["robot"]["vision"]["range_cm"] = 0.0

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "robot.vision.range_cm" in message


def test_rejects_vision_range_cm_negative():
    """robot.vision: a negative range_cm raises LayoutError (must be null or > 0)."""
    data = _layout_with_mosaic_and_vision(_random_mosaic_block())
    data["robot"]["vision"]["range_cm"] = -5.0

    with pytest.raises(LayoutError) as exc_info:
        load(data)

    message = str(exc_info.value)
    assert message.startswith("ctps.layout:")
    assert "robot.vision.range_cm" in message


# ---------------------------------------------------------------------------
# robot.axle_offset additions (T7 -- CTPS axle-body-offset plan, AC6)
# ---------------------------------------------------------------------------
#
# Exercises the layout.py boundary for the optional `robot.axle_offset` block:
# both the `validate()` path (`_validate_axle_offset`) and the `load()` /
# `_build_layout` construction path (`_build_axle_offset`) must agree, so a
# layout carrying an offset round-trips byte-for-byte through
# `load(dict).to_dict()`. TC5 below is the dual-path gate: if only `validate`
# learned the field, `_build_layout` would silently drop `axle_offset` from
# the built `Layout`'s `to_dict()` output.


def _valid_axle_offset_block() -> dict:
    """Return a fresh, valid `robot.axle_offset` block dict."""
    return {"forward_cm": 4.0, "lateral_cm": -1.5}


def _layout_with_axle_offset() -> dict:
    """Return a valid layout dict carrying a `robot.axle_offset` block."""
    data = _valid_layout_dict()
    data["robot"]["axle_offset"] = _valid_axle_offset_block()
    return data


# -- TC1: axle_offset present -- to_dict() includes both keys; validates clean --


def test_axle_offset_present_to_dict_includes_both_keys_and_validates():
    """TC1: RobotConfig.to_dict() with axle_offset includes both keys; validate passes."""
    data = _layout_with_axle_offset()

    layout = load(copy.deepcopy(data))

    assert layout.robot.axle_offset is not None
    assert layout.robot.axle_offset.forward_cm == 4.0
    assert layout.robot.axle_offset.lateral_cm == -1.5

    out = layout.robot.to_dict()
    assert out["axle_offset"] == {"forward_cm": 4.0, "lateral_cm": -1.5}

    validate(layout.to_dict())  # must not raise


# -- TC2: axle_offset absent -- to_dict() omits the key entirely ----------------


def test_axle_offset_absent_to_dict_omits_key():
    """TC2: RobotConfig.to_dict() with no axle_offset omits the key entirely.

    Byte-identical to the no-offset input -- an absent offset must never gain
    an `"axle_offset"` key (nor emit one as an explicit `None`).
    """
    data = _valid_layout_dict()
    assert "axle_offset" not in data["robot"]

    layout = load(copy.deepcopy(data))

    assert layout.robot.axle_offset is None
    out = layout.robot.to_dict()
    assert "axle_offset" not in out
    assert layout.to_dict() == data


# -- TC3: validate() directly on a layout dict with a valid axle_offset ---------


def test_validate_accepts_valid_axle_offset():
    """TC3: validate() on a layout dict with a valid robot.axle_offset passes."""
    data = _layout_with_axle_offset()
    validate(data)  # must not raise


# -- TC4: validate() rejects each malformed axle_offset shape, naming the path --


def test_rejects_axle_offset_not_a_dict():
    """TC4a: a non-dict robot.axle_offset raises naming the key path."""
    data = _layout_with_axle_offset()
    data["robot"]["axle_offset"] = "forward"

    with pytest.raises(LayoutError) as exc_info:
        validate(data)

    message = str(exc_info.value)
    assert "robot.axle_offset" in message
    assert "dict" in message


def test_rejects_axle_offset_missing_forward_cm():
    """TC4b: robot.axle_offset missing forward_cm raises naming the missing key."""
    data = _layout_with_axle_offset()
    del data["robot"]["axle_offset"]["forward_cm"]

    with pytest.raises(LayoutError) as exc_info:
        validate(data)

    message = str(exc_info.value)
    assert "robot.axle_offset" in message
    assert "forward_cm" in message


def test_rejects_axle_offset_missing_lateral_cm():
    """TC4b: robot.axle_offset missing lateral_cm raises naming the missing key."""
    data = _layout_with_axle_offset()
    del data["robot"]["axle_offset"]["lateral_cm"]

    with pytest.raises(LayoutError) as exc_info:
        validate(data)

    message = str(exc_info.value)
    assert "robot.axle_offset" in message
    assert "lateral_cm" in message


def test_rejects_axle_offset_non_finite_value():
    """TC4c: a non-finite (NaN) axle_offset value raises naming the key path."""
    data = _layout_with_axle_offset()
    data["robot"]["axle_offset"]["forward_cm"] = float("nan")

    with pytest.raises(LayoutError) as exc_info:
        validate(data)

    message = str(exc_info.value)
    assert "robot.axle_offset.forward_cm" in message
    assert "finite" in message


def test_rejects_axle_offset_infinite_value():
    """TC4c: an infinite axle_offset value raises naming the key path."""
    data = _layout_with_axle_offset()
    data["robot"]["axle_offset"]["lateral_cm"] = float("inf")

    with pytest.raises(LayoutError) as exc_info:
        validate(data)

    message = str(exc_info.value)
    assert "robot.axle_offset.lateral_cm" in message
    assert "finite" in message


def test_rejects_axle_offset_non_numeric_value():
    """TC4d: a non-numeric axle_offset value raises naming the key path."""
    data = _layout_with_axle_offset()
    data["robot"]["axle_offset"]["forward_cm"] = "far"

    with pytest.raises(LayoutError) as exc_info:
        validate(data)

    message = str(exc_info.value)
    assert "robot.axle_offset.forward_cm" in message
    assert "finite" in message


# -- TC5 (CRITICAL -- the dual-path gate): load().to_dict() round-trips ---------


def test_axle_offset_round_trips_through_load_to_dict_load():
    """TC5 (dual-path gate): load(dict_with_offset).to_dict() round-trips axle_offset.

    Catches a missing `_build_axle_offset` wiring in `_build_layout` -- if
    only `validate()` learned the field, the built `Layout`'s `to_dict()`
    would silently drop `axle_offset` even though `validate()` accepted it.
    """
    data = _layout_with_axle_offset()

    layout = load(copy.deepcopy(data))

    assert layout.robot.axle_offset is not None
    assert layout.to_dict() == data

    # Round-trip through actual JSON text, then reload -- stable second pass.
    reloaded = load(json.loads(json.dumps(layout.to_dict())))
    assert reloaded.to_dict() == data
