"""Pipeline-wide motion mode defaults for the WRO 2026 path planner.

Each enum below selects which Drive/LineFollower API the runtime PathTracker
dispatches for one primitive category. The header_writer bakes the values of
`PIPELINE_CONFIG` into a generated `Routes::PIPELINE_CONFIG` struct in
`routes.cpp`; per-edge overrides in `field_data.EDGES` can override individual
fields on a per-edge basis (DEFAULT sentinel = inherit pipeline value).

To change the mission-wide default for any category, edit `PIPELINE_CONFIG`
below and re-run `uv run python tools/path_planner/build_routes.py`.

The DEFAULT=0 sentinel on every enum means "inherit PIPELINE_CONFIG.<category>"
at C++ dispatch time. Keep DEFAULT as the first value so a zero-initialized
EdgeInfo override field implicitly means "no override".
"""

from __future__ import annotations
from dataclasses import dataclass
from enum import IntEnum


class LineMode(IntEnum):
    """Cruise preset for an EDGE_LINE traversal (LineFollower setPreset)."""
    DEFAULT = 0  # inherit PIPELINE_CONFIG.line at runtime
    NORMAL  = 1
    SLOW    = 2
    FAST    = 3


class BlindMode(IntEnum):
    """Drive-mode bundle for an EDGE_BLIND traversal."""
    DEFAULT      = 0  # inherit PIPELINE_CONFIG.blind at runtime
    SYNC_TSHAPE  = 1  # DriveMode::SYNC + tShape (today's runtime default)
    GYRO_TSHAPE  = 2  # DriveMode::GYRO + tShape — straight-line yaw lock
    RAW_FLAT     = 3  # DriveMode::RAW + no profile — open-loop, no ramp
    STOP_AT_LINE = 4  # drive blind, arm near endpoint, stop on the crossing line, re-anchor


class TurnMode(IntEnum):
    """Wheel pattern for the new custom TurnGyro (replaces MiniR4.DriveDC.TurnGyro)."""
    DEFAULT     = 0  # inherit PIPELINE_CONFIG.turn at runtime
    COUNTER_ROT = 1  # both wheels, opposite directions; pivot around chassis center
    PIVOT_LEFT  = 2  # left wheel still; right wheel spins; chassis pivots around left wheel
    PIVOT_RIGHT = 3  # right wheel still; left wheel spins; chassis pivots around right wheel


class DirectionMode(IntEnum):
    """Travel-direction override for an edge traversal (forward vs reverse)."""
    DEFAULT       = 0  # defer to builder -> auto algorithm (inherit PIPELINE_CONFIG.direction)
    FORCE_FORWARD = 1  # force forward traversal regardless of heading delta
    FORCE_REVERSE = 2  # force reverse traversal regardless of heading delta


class StopAtLineMode(IntEnum):
    """Per-edge stop-at-line override, decoupled from BlindMode (the motion bundle)."""
    DEFAULT = 0   # inherit: defer to legacy BlindMode tag / auto-rule / pipeline
    ON      = 1   # force the seek on this edge
    OFF     = 2   # force dead-reckon (no seek) on this edge


@dataclass(frozen=True)
class PipelineConfig:
    """Mission-wide motion-mode defaults baked into routes.cpp at build time."""
    line:       LineMode       = LineMode.NORMAL
    blind:      BlindMode      = BlindMode.SYNC_TSHAPE
    turn:       TurnMode       = TurnMode.COUNTER_ROT
    direction:  DirectionMode  = DirectionMode.DEFAULT

    def __post_init__(self) -> None:
        # Category fields may not be set to their own DEFAULT sentinel: DEFAULT
        # means "inherit PIPELINE_CONFIG.<category> at runtime", which is
        # self-referential when applied to the pipeline config itself and silently
        # degrades to the C++ hardcoded fallback.  DirectionMode.DEFAULT is
        # intentional here (means "defer to builder auto algorithm") and is exempt.
        _CATEGORY_FIELDS: tuple[tuple[str, object], ...] = (
            ("line",       LineMode.DEFAULT),
            ("blind",      BlindMode.DEFAULT),
            ("turn",       TurnMode.DEFAULT),
        )
        for field_name, sentinel in _CATEGORY_FIELDS:
            if getattr(self, field_name) == sentinel:
                raise ValueError(
                    f"PipelineConfig.{field_name} may not be set to "
                    f"{type(sentinel).__name__}.DEFAULT: DEFAULT is an "
                    f"edge-level sentinel that means 'inherit "
                    f"PIPELINE_CONFIG.{field_name}', so setting the pipeline "
                    f"config itself to DEFAULT is circular."
                )

    @classmethod
    def from_field_data(cls, field_data) -> "PipelineConfig":
        """Build a :class:`PipelineConfig` from the field-sourced PIPELINE_* keys.

        Reads the four optional mission-wide motion-mode defaults the field data
        may carry (ADR-047) and resolves each into the matching enum member:

        * ``PIPELINE_LINE_MODE``      -> :class:`LineMode`
        * ``PIPELINE_BLIND_MODE``     -> :class:`BlindMode`
        * ``PIPELINE_TURN_MODE``      -> :class:`TurnMode`
        * ``PIPELINE_DIRECTION_MODE`` -> :class:`DirectionMode`

        Each attribute may be an enum member NAME (e.g. ``"FAST"``), an enum
        INSTANCE (legacy / programmatic authoring), or ``None``. ``None`` (the
        absent-key case) falls back to the matching field on the module-level
        :data:`PIPELINE_CONFIG` singleton, so a field that omits every key
        produces a config byte-identical to the singleton and committed
        ``routes.cpp`` stays unchanged. Reads use ``getattr(..., None)`` so the
        many ``SimpleNamespace`` test fixtures that omit these keys keep working.

        The constructed instance runs through :meth:`__post_init__`, so the
        same DEFAULT-sentinel rule the validator enforces (line/blind/turn may
        not be DEFAULT; direction may) is re-asserted here as a second gate.

        :param field_data: the ``tools.path_planner.field_data`` module (or any
            object exposing the optional ``PIPELINE_*`` attributes).
        :return: a fully-resolved :class:`PipelineConfig`.
        :raises KeyError: if a string name does not name a member of its enum
            (the validator catches this first in a normal build).
        :raises ValueError: from :meth:`__post_init__` on a circular DEFAULT.
        """
        # (attr, EnumCls, singleton-field) for each mission-wide category. The
        # singleton field supplies the inherit-on-absent fallback.
        _SPECS: tuple[tuple[str, type, str], ...] = (
            ("PIPELINE_LINE_MODE",      LineMode,      "line"),
            ("PIPELINE_BLIND_MODE",     BlindMode,     "blind"),
            ("PIPELINE_TURN_MODE",      TurnMode,      "turn"),
            ("PIPELINE_DIRECTION_MODE", DirectionMode, "direction"),
        )
        resolved: dict[str, object] = {}
        for attr, enum_cls, ctor_field in _SPECS:
            value = getattr(field_data, attr, None)
            if value is None:
                resolved[ctor_field] = getattr(PIPELINE_CONFIG, ctor_field)
            elif isinstance(value, enum_cls):
                resolved[ctor_field] = value
            else:
                # A bare member name; ``enum_cls[name]`` raises KeyError on an
                # unknown name (the validator rejects it before emit).
                resolved[ctor_field] = enum_cls[value]
        return cls(**resolved)


# Mission-wide default. Edit here to swap defaults globally. Per-edge overrides
# in field_data.EDGES take precedence over these.
PIPELINE_CONFIG = PipelineConfig()


__all__ = [
    "LineMode",
    "BlindMode",
    "TurnMode",
    "DirectionMode",
    "StopAtLineMode",
    "PipelineConfig",
    "PIPELINE_CONFIG",
]
