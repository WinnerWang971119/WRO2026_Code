"""Virtual chassis: ``(x, y, heading_deg)`` bookkeeping for the runtime sim.

Every primitive transition mutates the state in place. No timing, no
encoder model, no IMU model — each primitive applies its full effect
atomically. This matches the PR 2 verification approach: prove that the
sequence of dispatched primitives produced by the runtime FSM is
identical to what :func:`tools.path_planner.compiler.compile_path`
emits for the same NEXT_HOP-walked path, and prove that running those
primitives through a pure-geometry chassis lands at the goal node's
``(x, y)`` within tolerance.

Heading convention (must match graph_builder / compiler / Localization):

* Continuous float degrees in ``[0, 360)``.
* ``0 deg = +x`` axis (East).
* CCW positive.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def _normalize(deg: float) -> float:
    """Wrap a degree value into ``[0, 360)``.

    Mirrors :func:`Localization::_normalize` on the C++ side. Any finite
    float is accepted; negative inputs are folded by adding 360 until in
    range.
    """
    r = deg % 360.0
    if r < 0.0:
        r += 360.0
    if r >= 360.0:
        r -= 360.0
    return r


@dataclass
class VirtualChassis:
    """Pure-geometry stand-in for the physical chassis.

    Use :meth:`from_field_data` to build an instance seeded at
    ``field_data.START_NODE`` facing ``field_data.START_HEADING_DEG``
    — that is the assumption the runtime makes about start pose.
    """

    x_cm: float
    y_cm: float
    heading_deg: float

    @classmethod
    def from_field_data(cls, field_data) -> "VirtualChassis":
        """Construct a chassis at ``field_data.START_NODE`` / ``START_HEADING_DEG``.

        :param field_data: validated field-data namespace exposing
            ``NODES``, ``START_NODE``, ``START_HEADING_DEG``.
        :return: a new chassis at the canonical start pose.
        :raises KeyError: if ``START_NODE`` is not in ``NODES``.
        """
        x, y = field_data.NODES[field_data.START_NODE]
        return cls(
            x_cm=float(x),
            y_cm=float(y),
            heading_deg=_normalize(float(field_data.START_HEADING_DEG)),
        )

    # -- primitive transitions ------------------------------------------------

    def turn_relative(self, deg: float) -> None:
        """Rotate the chassis in place by @p deg (CCW positive)."""
        self.heading_deg = _normalize(self.heading_deg + float(deg))

    def turn_to_heading(self, target_deg: float) -> None:
        """Rotate the chassis in place to absolute world-frame heading."""
        self.heading_deg = _normalize(float(target_deg))

    def follow_line_to_intersection(self, distance_cm: float) -> None:
        """Translate the chassis forward by @p distance_cm along its heading.

        The line follower physically tracks the line, so the heading at the
        end of the segment matches the edge's heading_deg — but
        ``compile_path`` always emits a ``turn_relative`` before each edge
        traversal to align the chassis to the edge heading first, so this
        method does not need to clamp heading itself.

        :param distance_cm: edge length in centimetres (positive).
        """
        self._advance(distance_cm)

    def follow_line_distance(self, cm: float) -> None:
        """Translate the chassis forward by @p cm along its heading.

        Used for PROJECTION edges where the chassis stops by encoder
        distance rather than physical intersection marker.
        """
        self._advance(cm)

    def follow_line_to_distance(self, cm: float) -> None:
        """Advance the chassis straight forward by @p cm.

        Mirrors the runtime EDGE_PROJECTION dispatch (line-follow stopped
        by encoder distance). The sim does not model lateral offset for
        line-following, so this is a thin wrapper over :meth:`forward`.

        :param cm: encoder-gated distance in centimetres (positive).
        """
        self.forward(cm)

    def forward(self, cm: float) -> None:
        """Translate the chassis forward by @p cm (open-loop blind move)."""
        self._advance(cm)

    def reverse(self, cm: float) -> None:
        """Translate the chassis backward by @p cm along its current heading.

        Mirrors :meth:`forward` but negates the displacement vector, so the
        chassis moves opposite to its heading without rotating. Used by the
        bidirectional line-following dispatch (T11+) where the runtime
        drives the rear sensor toward the next intersection instead of
        rotating in place.

        :param cm: distance in centimetres (positive); negative values
            simply re-invert the direction (move forward).
        """
        self._advance(-float(cm))

    # -- internal -------------------------------------------------------------

    def _advance(self, cm: float) -> None:
        rad = math.radians(self.heading_deg)
        self.x_cm += float(cm) * math.cos(rad)
        self.y_cm += float(cm) * math.sin(rad)

    # -- introspection --------------------------------------------------------

    def position(self) -> tuple:
        """Return the current ``(x_cm, y_cm)`` tuple."""
        return (self.x_cm, self.y_cm)

    def __repr__(self) -> str:    # pragma: no cover -- visual aid only
        return f"VirtualChassis(x={self.x_cm:.3f}, y={self.y_cm:.3f}, heading={self.heading_deg:.3f})"
