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
    DEFAULT     = 0  # inherit PIPELINE_CONFIG.blind at runtime
    SYNC_TSHAPE = 1  # DriveMode::SYNC + tShape (today's runtime default)
    GYRO_TSHAPE = 2  # DriveMode::GYRO + tShape — straight-line yaw lock
    RAW_FLAT    = 3  # DriveMode::RAW + no profile — open-loop, no ramp


class ProjectionMode(IntEnum):
    """Cruise preset for an EDGE_PROJECTION traversal (line-follow stopped by encoder distance)."""
    DEFAULT = 0  # inherit PIPELINE_CONFIG.projection at runtime
    NORMAL  = 1
    SLOW    = 2
    FAST    = 3


class TurnMode(IntEnum):
    """Wheel pattern for the new custom TurnGyro (replaces MiniR4.DriveDC.TurnGyro)."""
    DEFAULT     = 0  # inherit PIPELINE_CONFIG.turn at runtime
    COUNTER_ROT = 1  # both wheels, opposite directions; pivot around chassis center
    PIVOT_LEFT  = 2  # left wheel still; right wheel spins; chassis pivots around left wheel
    PIVOT_RIGHT = 3  # right wheel still; left wheel spins; chassis pivots around right wheel


@dataclass(frozen=True)
class PipelineConfig:
    """Mission-wide motion-mode defaults baked into routes.cpp at build time."""
    line:       LineMode       = LineMode.NORMAL
    blind:      BlindMode      = BlindMode.SYNC_TSHAPE
    projection: ProjectionMode = ProjectionMode.NORMAL
    turn:       TurnMode       = TurnMode.COUNTER_ROT


# Mission-wide default. Edit here to swap defaults globally. Per-edge overrides
# in field_data.EDGES take precedence over these.
PIPELINE_CONFIG = PipelineConfig()


__all__ = [
    "LineMode",
    "BlindMode",
    "ProjectionMode",
    "TurnMode",
    "PipelineConfig",
    "PIPELINE_CONFIG",
]
