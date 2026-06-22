"""Reset-node wall auto-fill geometry helper.

This module is the Tk-free, matplotlib-free core that the live field
editor's reset-node auto-fill UI leans on. It holds a single pure function:

* :func:`reset_autofill` -- suggest a ``(wall_coord, facing)`` pair for a
  reset node against the nearer qualifying field wall.

It is imported by the commands layer (``SetResetSpec.autofilled``) and the
app (the reset-spec inspector handler), and is unit-tested directly, so it
stays dependency-light: stdlib only plus two cheap module-level constants
pulled from the planner package. **No Tk, no matplotlib.**

.. note::
   The projection-edge tooling that previously lived here
   (``read_projection_edges`` / ``compute_projection_geometry`` /
   ``reconcile`` and the off-model ``ProjectionDelta`` it fed) was removed
   when projection became an on-model node type (issue #64): a projection
   node is now set through the normal ``SetNodeType`` command and rides
   ``node_types`` through serialization, so the off-model store and its
   geometry helpers are gone. Only the reset-node auto-fill survives.

Conventions (inherited from ``field_data.py`` / ``graph_builder.py``):
    Origin at the field SW corner. X is East, Y is North. Distances in
    centimetres. Headings in degrees, ``[0, 360)``, CCW from the +x axis.
    ``FIELD_MM`` is ``(width_mm, height_mm)`` in millimetres -- divide by
    10 to get centimetres.
"""

from __future__ import annotations

from tools.path_planner.editor.model import RESET_AXIS_VALUES

# MUST mirror validator.WALL_PIVOT_CLEARANCE_CM -- the pivot-sweep radius
# plus safety margin a reset node needs to stand off the wall it re-zeros
# against. Imported directly (it is a cheap top-level float constant) so the
# value cannot silently desync from the Save-time validator that re-checks it.
from tools.path_planner.validator import WALL_PIVOT_CLEARANCE_CM

# The two axes a reset node may re-zero against (a wall parallel to one axis).
# Single-sourced from ``model.RESET_AXIS_VALUES`` (a PUBLIC constant -- not a
# leading-underscore private of another module), which mirrors
# ``validator._ALLOWED_RESET_AXES``.
_ALLOWED_AXES = RESET_AXIS_VALUES

__all__ = [
    "WALL_PIVOT_CLEARANCE_CM",
    "reset_autofill",
]


def reset_autofill(
    node_xy: tuple[float, float],
    axis: str,
    field_mm: tuple[float, float],
) -> tuple[float, float]:
    """Suggest a ``(wall_coord, facing)`` for a reset node on the given axis.

    A reset node re-zeros its localization against a field wall parallel to one
    axis. This picks the wall and the heading the chassis faces while standing
    off it, given the node's position and the field size.

    Wall candidates and facings, per axis:

    * ``axis == "x"`` -- candidate walls are ``0.0`` and ``field_w_cm``
      (``field_mm[0] / 10``); the relevant node coordinate is ``node_xy[0]``.
      The chosen facing points *away* from the wall: ``0.0 deg`` (front toward
      +x / East) when the wall is the low ``0.0`` wall, else ``180.0 deg``.
    * ``axis == "y"`` -- candidate walls are ``0.0`` and ``field_h_cm``
      (``field_mm[1] / 10``); the relevant node coordinate is ``node_xy[1]``.
      The facing is ``90.0 deg`` (toward +y / North) when the wall is ``0.0``,
      else ``270.0 deg``.

    Wall selection by standoff ``abs(wall - node_coord)``:

    * If exactly one wall has standoff ``>= WALL_PIVOT_CLEARANCE_CM``, choose it.
    * If BOTH qualify, prefer the **nearer** wall (smaller standoff) -- the
      shorter, more reliable re-zero approach.
    * If NEITHER qualifies (the node hugs the field on this axis), fall back to
      the wall with the **larger** standoff. This is best-effort: the Save-time
      validator re-checks the clearance against
      :data:`WALL_PIVOT_CLEARANCE_CM` and will still flag the node, so the
      author is not silently handed an unrealisable reset.

    Ties (both standoffs equal) resolve to the ``0.0`` wall, since it is
    enumerated first in every selection branch.

    :param node_xy: the reset node position ``(x_cm, y_cm)``.
    :param axis: ``"x"`` or ``"y"`` -- the axis whose wall the node re-zeros
        against.
    :param field_mm: outer field size ``(width_mm, height_mm)`` in millimetres.
    :return: ``(wall_coord, facing)`` -- the suggested wall coordinate in
        centimetres and the heading the chassis faces in degrees (``[0, 360)``).
    :raises ValueError: if ``axis`` is not ``"x"`` or ``"y"``.
    """
    if axis not in _ALLOWED_AXES:
        raise ValueError(
            f"reset_autofill: axis must be one of {_ALLOWED_AXES}; "
            f"got {axis!r}."
        )

    field_w_cm = field_mm[0] / 10.0
    field_h_cm = field_mm[1] / 10.0

    if axis == "x":
        node_coord = node_xy[0]
        far_wall = field_w_cm
        low_facing = 0.0  # front toward +x, away from the 0.0 wall
        high_facing = 180.0
    else:  # axis == "y"
        node_coord = node_xy[1]
        far_wall = field_h_cm
        low_facing = 90.0  # front toward +y, away from the 0.0 wall
        high_facing = 270.0

    # Candidate walls and their facings, low wall first so ties favor 0.0.
    candidates = (
        (0.0, low_facing),
        (far_wall, high_facing),
    )

    qualifying = [
        (wall, facing)
        for wall, facing in candidates
        if abs(wall - node_coord) >= WALL_PIVOT_CLEARANCE_CM
    ]

    if qualifying:
        # One or both walls clear the standoff. Prefer the nearer (smallest
        # standoff); the low wall wins a tie since it sorts first below.
        wall, facing = min(qualifying, key=lambda c: abs(c[0] - node_coord))
    else:
        # Neither wall clears the standoff (node hugs the field). Best-effort:
        # pick the wall with the LARGER standoff; the Save validator still
        # flags it against WALL_PIVOT_CLEARANCE_CM.
        wall, facing = max(candidates, key=lambda c: abs(c[0] - node_coord))

    return wall, facing
