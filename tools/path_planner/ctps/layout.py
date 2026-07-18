"""Field-object + manipulator layout schema and strict loader (schema_version 1).

Parses ``layout.json`` — the file authored by the offline ``layout_editor.html``
(Field tab: field objects; Robot tab: manipulator zones) — into a typed
:class:`Layout` for the CTPS player emitter (``player_html.render(objects=...)``)
to embed as ``const OBJECTS``.

Unlike :mod:`tools.path_planner.ctps.trace_loader` (which degrades gracefully on
an unrecognised trace entry), this loader is **strict**: :func:`validate` raises
:class:`LayoutError` LOUD on the first malformed field, with a precise key path
and a one-line description of the fix — a bad authored ``layout.json`` must fail
the render, not silently corrupt the field-day page (see the plan's Error
Handling section). The in-browser JS loader is the tolerant counterpart (skip +
``console.warn`` a malformed object, never throw) and is intentionally NOT
mirrored here.

Coordinate frames:

* Field objects (``objects[]``) live in the same world frame as
  ``field_data`` — centimetres, ``(x_cm, y_cm)`` world coordinates.
* The ``robot`` zones/drop-points are **robot-LOCAL**: origin = robot centre,
  ``+x`` = heading-forward, ``+y`` = robot-left. The player resolver maps each
  to world coordinates via ``world = pose.xy + Rot(pose.heading) . local.xy``.

Magazine geometry (lane count) and rear-box capacity are NOT authored in
``layout.json`` — they are fixed player constants in
:mod:`tools.path_planner.ctps.config` (``MAGAZINE_LANES`` / ``MAGAZINE_DEPTH`` /
``REAR_BOX_CAP``); :func:`validate` reads ``MAGAZINE_LANES`` to check
``robot.drop_points`` carries exactly one entry per lane.

The optional top-level ``mosaic`` block and the optional ``robot.vision`` cone
block (CTPS randomized-vision feature, ADR-060) are both validated by delegating
to :mod:`tools.path_planner.ctps.mosaic`'s pure validators
(:func:`~tools.path_planner.ctps.mosaic.validate_mosaic_config` /
:func:`~tools.path_planner.ctps.mosaic.validate_vision_cone`) — their rules are
NOT re-implemented here. Each raises a plain ``ValueError``; this module catches
it and re-raises as :class:`LayoutError` with this module's ``ctps.layout:``
key-path framing. Both blocks are fully optional: absent ⇒ no raise and no key
in :meth:`Layout.to_dict`.

Typical usage::

    from tools.path_planner.ctps.layout import load

    layout = load("layout.json")     # from a file path
    layout = load(raw_json_string)   # from an already-read JSON string
    layout = load(parsed_dict)       # from an already-parsed dict

    for obj in layout.objects:
        print(obj.id, obj.shape, obj.type, obj.x_cm, obj.y_cm)

    layout.to_dict()  # round-trips back to the schema_version 1 dict shape
"""

from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass, field as dc_field
from typing import Any, Optional

from tools.path_planner.ctps import mosaic
from tools.path_planner.ctps.config import MAGAZINE_LANES

__all__ = [
    "SUPPORTED_SCHEMA_VERSION",
    "SHAPE_RECT",
    "SHAPE_CIRCLE",
    "OBJECT_TYPE_TILE",
    "OBJECT_TYPE_CARGO",
    "OBJECT_TYPE_FIXED",
    "LayoutError",
    "RectZone",
    "DropPoint",
    "VisionCone",
    "AxleOffset",
    "RobotConfig",
    "LayoutObject",
    "Layout",
    "load",
    "validate",
]

# ---------------------------------------------------------------------------
# Schema contract
# ---------------------------------------------------------------------------

#: The only schema version this loader accepts.
SUPPORTED_SCHEMA_VERSION: int = 1

#: Object shapes — exact strings, shared with the player resolver / editor
#: (Contracts: Naming).
SHAPE_RECT: str = "rect"
SHAPE_CIRCLE: str = "circle"
_ALLOWED_SHAPES: frozenset[str] = frozenset({SHAPE_RECT, SHAPE_CIRCLE})

#: Object types — exact strings, shared with the player resolver / editor
#: (Contracts: Naming).
OBJECT_TYPE_TILE: str = "tile"
OBJECT_TYPE_CARGO: str = "cargo"
OBJECT_TYPE_FIXED: str = "fixed"
_ALLOWED_TYPES: frozenset[str] = frozenset(
    {OBJECT_TYPE_TILE, OBJECT_TYPE_CARGO, OBJECT_TYPE_FIXED}
)

#: Required keys on every ``objects[]`` entry, regardless of shape.
_OBJECT_REQUIRED_KEYS: tuple[str, ...] = (
    "id",
    "name",
    "shape",
    "type",
    "color",
    "x_cm",
    "y_cm",
    "heading_deg",
)

#: Required keys on a ``robot.grab_zone`` / ``robot.box_zone`` rect.
_ZONE_REQUIRED_KEYS: tuple[str, ...] = ("x_cm", "y_cm", "w_cm", "h_cm")

#: Required keys on a single ``robot.drop_points[]`` entry.
_DROP_POINT_REQUIRED_KEYS: tuple[str, ...] = ("x_cm", "y_cm")

#: Required keys on the optional ``robot.axle_offset`` block.
_AXLE_OFFSET_REQUIRED_KEYS: tuple[str, ...] = ("forward_cm", "lateral_cm")

#: Authored hex color: ``#RRGGBB`` (6 hex digits), case-insensitive.
_HEX_COLOR_RE = re.compile(r"^#[0-9A-Fa-f]{6}$")


class LayoutError(Exception):
    """Raised by :func:`validate` when a ``layout.json`` dict is malformed.

    Carries a single human-readable message that includes the offending key
    path (e.g. ``objects[2].shape``) and a one-line description of the fix.
    """


def _is_finite_number(value: Any) -> bool:
    """Return True for a finite ``int``/``float`` that is not a ``bool``.

    A JSON ``true``/``false`` decodes to Python ``bool``, a subclass of
    ``int`` — ``isinstance(True, (int, float))`` is truthy, so a stray
    boolean would otherwise silently pass as ``1``/``0``. NaN/Infinity are
    also rejected: they slip past every downstream range gate (``<= 0`` is
    False for NaN) with no diagnostic.
    """
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


# ---------------------------------------------------------------------------
# Public dataclasses
# ---------------------------------------------------------------------------


@dataclass
class RectZone:
    """An axis-aligned rectangle in the robot-local frame (centre + extents).

    :param x_cm: Centre X, robot-local (``+x`` = heading-forward).
    :param y_cm: Centre Y, robot-local (``+y`` = robot-left).
    :param w_cm: Width in centimetres (along local X); must be > 0.
    :param h_cm: Height in centimetres (along local Y); must be > 0.
    """

    x_cm: float
    y_cm: float
    w_cm: float
    h_cm: float

    def to_dict(self) -> dict[str, float]:
        """Return the schema dict for this zone."""
        return {"x_cm": self.x_cm, "y_cm": self.y_cm, "w_cm": self.w_cm, "h_cm": self.h_cm}


@dataclass
class DropPoint:
    """A single magazine-lane drop point in the robot-local frame.

    :param x_cm: X, robot-local (``+x`` = heading-forward).
    :param y_cm: Y, robot-local (``+y`` = robot-left).
    """

    x_cm: float
    y_cm: float

    def to_dict(self) -> dict[str, float]:
        """Return the schema dict for this drop point."""
        return {"x_cm": self.x_cm, "y_cm": self.y_cm}


@dataclass
class VisionCone:
    """The robot-attached forward vision cone (``robot.vision``), robot-local.

    All fields are in the robot-local frame — origin = robot centre, ``+x`` =
    heading-forward, ``+y`` = robot-left — matching the manipulator zones. The
    firmware never consumes this FOV geometry; it drives the CTPS player's
    per-cell cone-visibility predicate (ADR-060). The rule set lives in
    :func:`tools.path_planner.ctps.mosaic.validate_vision_cone`, not here.

    :param origin_x_cm: Cone apex X, robot-local (``+x`` = forward of centre).
    :param origin_y_cm: Cone apex Y, robot-local (``+y`` = robot-left).
    :param half_angle_deg: Half-FOV in degrees; rays run at ``±`` this from
        robot-forward. Must be strictly inside ``(0, 90)``.
    :param range_cm: Cone reach in centimetres; ``> 0`` when set. ``None``
        (unbounded) draws as long rays and admits any forward distance.
    """

    origin_x_cm: float
    origin_y_cm: float
    half_angle_deg: float
    range_cm: Optional[float] = None

    def to_dict(self) -> dict[str, Any]:
        """Return the schema dict for this vision cone.

        ``range_cm`` is always present in the emitted dict — ``None`` denotes an
        unbounded cone (unlike the optional top-level ``mosaic`` / ``robot.vision``
        blocks, which are omitted entirely from their parent dict when absent).
        """
        return {
            "origin_x_cm": self.origin_x_cm,
            "origin_y_cm": self.origin_y_cm,
            "half_angle_deg": self.half_angle_deg,
            "range_cm": self.range_cm,
        }


@dataclass
class AxleOffset:
    """The axle-to-body-centre offset (``robot.axle_offset``), robot-local.

    Robot-local frame — origin = axle (rotation centre), ``+x`` (``forward_cm``)
    = heading-forward, ``+y`` (``lateral_cm``) = robot-left — matching the
    manipulator zones / vision cone convention. The CTPS player draws the
    robot body (chassis square + heading arrow), the vision cone, the mosaic
    visibility HUD, and the manipulator zone overlays at the body centre —
    the axle shifted by this vector; the axle itself stays anchored on the
    graph node for the trajectory, trail, and node/hit anchoring. An absent
    ``axle_offset`` is equivalent to ``{0,0}`` — today's centred render.

    :param forward_cm: Robot-local +x offset; positive moves the body ahead
        of the axle.
    :param lateral_cm: Robot-local +y offset; positive moves the body to the
        robot's left.
    """

    forward_cm: float
    lateral_cm: float

    def to_dict(self) -> dict[str, float]:
        """Return the schema dict for this axle offset."""
        return {"forward_cm": self.forward_cm, "lateral_cm": self.lateral_cm}


@dataclass
class RobotConfig:
    """The ``robot`` section: manipulator zones, all in the robot-local frame.

    :param grab_zone: Where ``grab`` picks the nearest Tile whose centre falls
        inside.
    :param box_zone: Where ``boxClose``/``boxOpen`` capture/drop a Cargo.
    :param drop_points: Exactly :data:`~tools.path_planner.ctps.config.MAGAZINE_LANES`
        points, one per magazine lane (``dump`` ejects lane ``n``'s front tile
        to ``drop_points[n]``).
    :param vision: Optional robot-attached forward vision cone (``robot.vision``);
        ``None`` when the layout authors no cone.
    :param axle_offset: Optional axle-to-body-centre offset (``robot.axle_offset``);
        ``None`` when the layout authors no offset (equivalent to ``{0,0}``).
    """

    grab_zone: RectZone
    box_zone: RectZone
    drop_points: list[DropPoint]
    vision: Optional[VisionCone] = None
    axle_offset: Optional[AxleOffset] = None

    def to_dict(self) -> dict[str, Any]:
        """Return the schema dict for the ``robot`` section.

        The ``vision`` and ``axle_offset`` keys are each emitted only when
        configured — an absent cone/offset omits the key entirely (never
        ``"vision": None`` / ``"axle_offset": None``), so a cone-free,
        offset-free robot round-trips byte-identically to today.
        """
        out: dict[str, Any] = {
            "grab_zone": self.grab_zone.to_dict(),
            "box_zone": self.box_zone.to_dict(),
            "drop_points": [point.to_dict() for point in self.drop_points],
        }
        if self.vision is not None:
            out["vision"] = self.vision.to_dict()
        if self.axle_offset is not None:
            out["axle_offset"] = self.axle_offset.to_dict()
        return out


@dataclass
class LayoutObject:
    """A single field object (Tile / Cargo / Fixed), Rect or Circle.

    ``w_cm``/``h_cm`` are populated (as ``float``) for ``shape == "rect"`` and
    ``None`` for ``shape == "circle"``; ``radius_cm`` is the mirror image.
    Exactly one of the two shape-dimension pairs is non-``None``.

    :param id: Stable unique string identity (unique across the whole layout).
    :param name: User-authored display text (inventory card / log).
    :param shape: ``"rect"`` or ``"circle"``.
    :param type: ``"tile"``, ``"cargo"``, or ``"fixed"``.
    :param color: Authored ``#RRGGBB`` hex string.
    :param x_cm: World-frame centre X, centimetres.
    :param y_cm: World-frame centre Y, centimetres.
    :param heading_deg: Rect orientation in degrees (ignored when rendering a
        circle, but always present).
    :param w_cm: Rect width, centimetres — present only for ``shape=="rect"``.
    :param h_cm: Rect height, centimetres — present only for ``shape=="rect"``.
    :param radius_cm: Circle radius, centimetres — present only for
        ``shape=="circle"``.
    """

    id: str
    name: str
    shape: str
    type: str
    color: str
    x_cm: float
    y_cm: float
    heading_deg: float
    w_cm: Optional[float] = None
    h_cm: Optional[float] = None
    radius_cm: Optional[float] = None

    def to_dict(self) -> dict[str, Any]:
        """Return the schema dict for this object.

        Only the dimension keys matching ``shape`` are emitted — a rect never
        carries ``radius_cm`` and a circle never carries ``w_cm``/``h_cm``.
        """
        out: dict[str, Any] = {
            "id": self.id,
            "name": self.name,
            "shape": self.shape,
            "type": self.type,
            "color": self.color,
            "x_cm": self.x_cm,
            "y_cm": self.y_cm,
            "heading_deg": self.heading_deg,
        }
        if self.shape == SHAPE_RECT:
            out["w_cm"] = self.w_cm
            out["h_cm"] = self.h_cm
        else:
            out["radius_cm"] = self.radius_cm
        return out


@dataclass
class Layout:
    """A fully-parsed and validated ``layout.json`` (schema_version 1).

    :param schema_version: Always :data:`SUPPORTED_SCHEMA_VERSION` for a
        ``Layout`` produced by :func:`load`.
    :param objects: Field objects, in authored order.
    :param robot: Manipulator zone configuration.
    :param mosaic: Optional top-level randomized-vision mosaic block (ADR-060);
        ``None`` when the layout authors no mosaic.
    """

    schema_version: int
    objects: list[LayoutObject] = dc_field(default_factory=list)
    robot: Optional[RobotConfig] = None
    mosaic: Optional[mosaic.MosaicConfig] = None

    def to_dict(self) -> dict[str, Any]:
        """Return the schema_version 1 dict for this layout.

        Round-trips byte-for-byte (through :func:`json.dumps`/``json.loads``)
        with the dict :func:`load` produced this :class:`Layout` from. The
        ``mosaic`` key is emitted only when a mosaic is configured — an absent
        mosaic omits the key entirely (never ``"mosaic": None``), so a
        mosaic-free layout round-trips byte-identically to today.
        """
        out: dict[str, Any] = {
            "schema_version": self.schema_version,
            "objects": [obj.to_dict() for obj in self.objects],
            "robot": self.robot.to_dict() if self.robot is not None else None,
        }
        if self.mosaic is not None:
            out["mosaic"] = self.mosaic.to_dict()
        return out

    def validate(self) -> None:
        """Re-validate this layout's current field values (LOUD).

        Re-runs the full :func:`validate` schema check against
        ``self.to_dict()`` — useful after in-place mutation of a loaded
        :class:`Layout` (e.g. a caller editing fields directly) to confirm it
        is still emit-safe.

        :raises LayoutError: on the first malformed field, per :func:`validate`.
        """
        validate(self.to_dict())


# ---------------------------------------------------------------------------
# Strict validation (LOUD — validator-style)
# ---------------------------------------------------------------------------


def validate(data: Any) -> None:
    """Strictly validate a ``layout.json``-shaped dict against schema_version 1.

    Raises :class:`LayoutError` on the FIRST malformed entry, with a precise
    key path (e.g. ``objects[2].shape``) and a one-line fix hint — mirroring
    the top-level field-data validator's LOUD-raise convention
    (:mod:`tools.path_planner.validator`). Called internally by :func:`load`
    before any :class:`Layout` is constructed, so a bad file never produces a
    partially-built object.

    :param data: The already-JSON-decoded top-level value.
    :raises LayoutError: naming the offending key path on the first invalid
        field.
    """
    if not isinstance(data, dict):
        raise LayoutError(
            f"ctps.layout: top-level layout must be a dict; got {type(data).__name__!r}."
        )

    _validate_schema_version(data)
    _validate_objects(data)
    _validate_robot(data)
    _validate_top_level_mosaic(data)


def _validate_schema_version(data: dict) -> None:
    """Validate the top-level ``schema_version`` key."""
    if "schema_version" not in data:
        raise LayoutError("ctps.layout: missing 'schema_version' field.")
    version = data["schema_version"]
    if (
        not isinstance(version, int)
        or isinstance(version, bool)
        or version != SUPPORTED_SCHEMA_VERSION
    ):
        raise LayoutError(
            f"ctps.layout: unsupported schema_version={version!r}; "
            f"expected {SUPPORTED_SCHEMA_VERSION}."
        )


def _validate_objects(data: dict) -> None:
    """Validate the top-level ``objects`` list (each entry, plus id uniqueness)."""
    if "objects" not in data:
        raise LayoutError("ctps.layout: missing 'objects' list.")
    objects = data["objects"]
    if not isinstance(objects, list):
        raise LayoutError(
            f"ctps.layout: 'objects' must be a list; got {type(objects).__name__!r}."
        )

    seen_ids: set[str] = set()
    for i, obj in enumerate(objects):
        _validate_object_entry(obj, i, seen_ids)


def _validate_object_entry(obj: Any, i: int, seen_ids: set[str]) -> None:
    """Validate a single ``objects[i]`` entry (shape/type/color/dims/range)."""
    if not isinstance(obj, dict):
        raise LayoutError(f"ctps.layout: objects[{i}] must be a dict; got {obj!r}.")

    missing = [key for key in _OBJECT_REQUIRED_KEYS if key not in obj]
    if missing:
        raise LayoutError(
            f"ctps.layout: objects[{i}] missing required key(s) {missing!r} "
            f"-- every object needs {list(_OBJECT_REQUIRED_KEYS)!r}."
        )

    obj_id = obj["id"]
    if not isinstance(obj_id, str) or not obj_id:
        raise LayoutError(
            f"ctps.layout: objects[{i}].id must be a non-empty string; got {obj_id!r}."
        )
    if obj_id in seen_ids:
        raise LayoutError(
            f"ctps.layout: objects[{i}].id {obj_id!r} is a duplicate -- object "
            "ids must be unique across the whole layout."
        )
    seen_ids.add(obj_id)

    name = obj["name"]
    if not isinstance(name, str):
        raise LayoutError(f"ctps.layout: objects[{i}].name must be a string; got {name!r}.")

    shape = obj["shape"]
    if shape not in _ALLOWED_SHAPES:
        raise LayoutError(
            f"ctps.layout: objects[{i}].shape must be one of "
            f"{sorted(_ALLOWED_SHAPES)!r}; got {shape!r}."
        )

    obj_type = obj["type"]
    if obj_type not in _ALLOWED_TYPES:
        raise LayoutError(
            f"ctps.layout: objects[{i}].type must be one of "
            f"{sorted(_ALLOWED_TYPES)!r}; got {obj_type!r}."
        )

    color = obj["color"]
    if not isinstance(color, str) or not _HEX_COLOR_RE.match(color):
        raise LayoutError(
            f"ctps.layout: objects[{i}].color must be a '#RRGGBB' hex string; "
            f"got {color!r}."
        )

    for key in ("x_cm", "y_cm", "heading_deg"):
        value = obj[key]
        if not _is_finite_number(value):
            raise LayoutError(
                f"ctps.layout: objects[{i}].{key} must be a finite number; got {value!r}."
            )

    if shape == SHAPE_RECT:
        for key in ("w_cm", "h_cm"):
            if key not in obj:
                raise LayoutError(
                    f"ctps.layout: objects[{i}] has shape='rect' but is missing '{key}'."
                )
            value = obj[key]
            if not _is_finite_number(value) or value <= 0:
                raise LayoutError(
                    f"ctps.layout: objects[{i}].{key} must be a finite number > 0 "
                    f"for shape='rect'; got {value!r}."
                )
    else:
        if "radius_cm" not in obj:
            raise LayoutError(
                f"ctps.layout: objects[{i}] has shape='circle' but is missing 'radius_cm'."
            )
        radius = obj["radius_cm"]
        if not _is_finite_number(radius) or radius <= 0:
            raise LayoutError(
                f"ctps.layout: objects[{i}].radius_cm must be a finite number > 0 "
                f"for shape='circle'; got {radius!r}."
            )


def _validate_robot(data: dict) -> None:
    """Validate the top-level ``robot`` section (zones + drop points)."""
    if "robot" not in data:
        raise LayoutError("ctps.layout: missing 'robot' section.")
    robot = data["robot"]
    if not isinstance(robot, dict):
        raise LayoutError(f"ctps.layout: 'robot' must be a dict; got {type(robot).__name__!r}.")

    _validate_zone(robot, "grab_zone")
    _validate_zone(robot, "box_zone")

    if "drop_points" not in robot:
        raise LayoutError("ctps.layout: robot missing 'drop_points' list.")
    drop_points = robot["drop_points"]
    if not isinstance(drop_points, list):
        raise LayoutError(
            f"ctps.layout: robot.drop_points must be a list; "
            f"got {type(drop_points).__name__!r}."
        )
    if len(drop_points) != MAGAZINE_LANES:
        raise LayoutError(
            f"ctps.layout: robot.drop_points must have exactly {MAGAZINE_LANES} "
            f"entries (one per magazine lane); got {len(drop_points)}."
        )
    for i, point in enumerate(drop_points):
        if not isinstance(point, dict):
            raise LayoutError(
                f"ctps.layout: robot.drop_points[{i}] must be a dict; got {point!r}."
            )
        missing = [key for key in _DROP_POINT_REQUIRED_KEYS if key not in point]
        if missing:
            raise LayoutError(
                f"ctps.layout: robot.drop_points[{i}] missing required key(s) {missing!r}."
            )
        for key in _DROP_POINT_REQUIRED_KEYS:
            value = point[key]
            if not _is_finite_number(value):
                raise LayoutError(
                    f"ctps.layout: robot.drop_points[{i}].{key} must be a finite "
                    f"number; got {value!r}."
                )

    _validate_vision(robot)
    _validate_axle_offset(robot)


def _validate_vision(robot: dict) -> None:
    """Validate the optional ``robot.vision`` cone block.

    The cone is optional: an absent ``vision`` key returns immediately. When
    present, the AC3 cone rules are enforced by delegating to
    :func:`tools.path_planner.ctps.mosaic.validate_vision_cone` (whose
    ``ValueError`` messages already start ``robot.vision.<key>`` / ``'robot.vision'``)
    and re-raising as :class:`LayoutError` with this module's ``ctps.layout:``
    prefix — the rule text lives once, in ``mosaic.py``.

    :param robot: the already-validated ``robot`` section dict.
    :raises LayoutError: prefixed ``ctps.layout: robot.vision...`` on the first
        invalid cone field.
    """
    if "vision" not in robot:
        return
    try:
        mosaic.validate_vision_cone(robot["vision"])
    except ValueError as exc:
        raise LayoutError(f"ctps.layout: {exc}") from exc


def _validate_axle_offset(robot: dict) -> None:
    """Validate the optional ``robot.axle_offset`` block.

    The offset is optional: an absent ``axle_offset`` key returns immediately
    (absent is equivalent to ``{0,0}`` — today's centred render). When
    present it must be a dict carrying finite numeric ``forward_cm`` and
    ``lateral_cm`` -- both keys are required together.

    :param robot: the already-validated ``robot`` section dict.
    :raises LayoutError: prefixed ``ctps.layout: robot.axle_offset...`` on the
        first invalid field.
    """
    if "axle_offset" not in robot:
        return
    offset = robot["axle_offset"]
    if not isinstance(offset, dict):
        raise LayoutError(
            f"ctps.layout: robot.axle_offset must be a dict; got {type(offset).__name__!r}."
        )
    missing = [key for key in _AXLE_OFFSET_REQUIRED_KEYS if key not in offset]
    if missing:
        raise LayoutError(
            f"ctps.layout: robot.axle_offset missing required key(s) {missing!r}."
        )
    for key in _AXLE_OFFSET_REQUIRED_KEYS:
        value = offset[key]
        if not _is_finite_number(value):
            raise LayoutError(
                f"ctps.layout: robot.axle_offset.{key} must be a finite number; "
                f"got {value!r}."
            )


def _validate_top_level_mosaic(data: dict) -> None:
    """Validate the optional top-level ``mosaic`` block.

    The block is optional: an absent ``mosaic`` key returns immediately. When
    present, the AC3 mosaic rules are enforced by delegating to
    :func:`tools.path_planner.ctps.mosaic.validate_mosaic_config` (whose
    ``ValueError`` messages already start ``mosaic.<key>`` / ``'mosaic'``) and
    re-raising as :class:`LayoutError` with this module's ``ctps.layout:``
    prefix — the rule text lives once, in ``mosaic.py``. A non-dict ``mosaic``
    is rejected by the delegate itself, so no extra type-check is needed here.

    :param data: the already-top-level-checked ``layout.json`` dict.
    :raises LayoutError: prefixed ``ctps.layout: mosaic...`` on the first invalid
        mosaic field.
    """
    if "mosaic" not in data:
        return
    try:
        mosaic.validate_mosaic_config(data["mosaic"])
    except ValueError as exc:
        raise LayoutError(f"ctps.layout: {exc}") from exc


def _validate_zone(robot: dict, zone_key: str) -> None:
    """Validate one ``robot.grab_zone`` / ``robot.box_zone`` rect."""
    if zone_key not in robot:
        raise LayoutError(f"ctps.layout: robot missing '{zone_key}' section.")
    zone = robot[zone_key]
    if not isinstance(zone, dict):
        raise LayoutError(
            f"ctps.layout: robot.{zone_key} must be a dict; got {type(zone).__name__!r}."
        )
    missing = [key for key in _ZONE_REQUIRED_KEYS if key not in zone]
    if missing:
        raise LayoutError(
            f"ctps.layout: robot.{zone_key} missing required key(s) {missing!r}."
        )
    for key in ("x_cm", "y_cm"):
        value = zone[key]
        if not _is_finite_number(value):
            raise LayoutError(
                f"ctps.layout: robot.{zone_key}.{key} must be a finite number; "
                f"got {value!r}."
            )
    for key in ("w_cm", "h_cm"):
        value = zone[key]
        if not _is_finite_number(value) or value <= 0:
            raise LayoutError(
                f"ctps.layout: robot.{zone_key}.{key} must be a finite number > 0; "
                f"got {value!r}."
            )


# ---------------------------------------------------------------------------
# Validated-dict -> Layout construction
# ---------------------------------------------------------------------------


def _build_zone(zone: dict) -> RectZone:
    """Build a :class:`RectZone` from an already-validated zone dict."""
    return RectZone(
        x_cm=float(zone["x_cm"]),
        y_cm=float(zone["y_cm"]),
        w_cm=float(zone["w_cm"]),
        h_cm=float(zone["h_cm"]),
    )


def _build_drop_point(point: dict) -> DropPoint:
    """Build a :class:`DropPoint` from an already-validated drop-point dict."""
    return DropPoint(x_cm=float(point["x_cm"]), y_cm=float(point["y_cm"]))


def _build_vision(vision: dict) -> VisionCone:
    """Build a :class:`VisionCone` from an already-validated ``robot.vision`` dict.

    ``range_cm`` is preserved as ``None`` when absent / null (an unbounded cone);
    otherwise coerced to ``float``.
    """
    return VisionCone(
        origin_x_cm=float(vision["origin_x_cm"]),
        origin_y_cm=float(vision["origin_y_cm"]),
        half_angle_deg=float(vision["half_angle_deg"]),
        range_cm=None if vision.get("range_cm") is None else float(vision["range_cm"]),
    )


def _build_axle_offset(offset: dict) -> AxleOffset:
    """Build an :class:`AxleOffset` from an already-validated ``robot.axle_offset`` dict."""
    return AxleOffset(
        forward_cm=float(offset["forward_cm"]),
        lateral_cm=float(offset["lateral_cm"]),
    )


def _build_object(obj: dict) -> LayoutObject:
    """Build a :class:`LayoutObject` from an already-validated object dict."""
    shape = obj["shape"]
    is_rect = shape == SHAPE_RECT
    return LayoutObject(
        id=str(obj["id"]),
        name=str(obj["name"]),
        shape=shape,
        type=obj["type"],
        color=obj["color"],
        x_cm=float(obj["x_cm"]),
        y_cm=float(obj["y_cm"]),
        heading_deg=float(obj["heading_deg"]),
        w_cm=float(obj["w_cm"]) if is_rect else None,
        h_cm=float(obj["h_cm"]) if is_rect else None,
        radius_cm=float(obj["radius_cm"]) if not is_rect else None,
    )


def _build_layout(data: dict) -> Layout:
    """Build a fully-typed :class:`Layout` from an already-validated dict."""
    robot_data = data["robot"]
    vision_data = robot_data.get("vision")
    axle_offset_data = robot_data.get("axle_offset")
    robot = RobotConfig(
        grab_zone=_build_zone(robot_data["grab_zone"]),
        box_zone=_build_zone(robot_data["box_zone"]),
        drop_points=[_build_drop_point(point) for point in robot_data["drop_points"]],
        vision=_build_vision(vision_data) if vision_data is not None else None,
        axle_offset=_build_axle_offset(axle_offset_data) if axle_offset_data is not None else None,
    )
    objects = [_build_object(obj) for obj in data["objects"]]
    mosaic_data = data.get("mosaic")
    mosaic_cfg = (
        mosaic.MosaicConfig.from_dict(mosaic_data) if mosaic_data is not None else None
    )
    return Layout(
        schema_version=int(data["schema_version"]),
        objects=objects,
        robot=robot,
        mosaic=mosaic_cfg,
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def load(source: "str | os.PathLike | dict") -> Layout:
    """Load, strictly validate, and parse a ``layout.json`` document.

    Accepts three input forms:

    * A :class:`pathlib.Path` or any :class:`os.PathLike` object — read as
      UTF-8 JSON.
    * A ``str`` that is an existing file path — read as UTF-8 JSON.
    * A ``str`` that is a raw JSON document (detected by the absence of an
      existing file at that path) — parsed directly.
    * A ``dict`` — treated as an already-parsed JSON object.

    :func:`validate` runs against the raw decoded dict BEFORE any
    :class:`Layout` is constructed, so a malformed file raises
    :class:`LayoutError` loud rather than silently building a partial object.

    :param source: Input source — file path, raw JSON string, or parsed dict.
    :return: A fully-typed, schema-valid :class:`Layout`.
    :raises LayoutError: if the decoded data fails :func:`validate`.
    :raises json.JSONDecodeError: if the JSON cannot be parsed.
    """
    if isinstance(source, dict):
        data: Any = source
    elif isinstance(source, os.PathLike):
        # Always treat PathLike objects as file paths.
        with open(os.fspath(source), encoding="utf-8") as fh:
            data = json.load(fh)
    else:
        # source is a str -- disambiguate: file path vs raw JSON document.
        assert isinstance(source, str)
        if os.path.exists(source):
            with open(source, encoding="utf-8") as fh:
                data = json.load(fh)
        else:
            data = json.loads(source)

    validate(data)
    return _build_layout(data)
