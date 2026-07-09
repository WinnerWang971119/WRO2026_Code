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

from tools.path_planner.ctps.layout import LayoutError, load

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
