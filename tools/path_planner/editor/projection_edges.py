"""Reset-node wall auto-fill geometry helper.

This module is the Tk-free, matplotlib-free core that the live field
editor's reset-node auto-fill UI leans on. It holds three pure functions:

* :func:`reset_autofill_auto` -- suggest a full ``(axis, wall_coord, facing)``
  spec for a reset node against the nearest of all four field walls.
* :func:`reset_autofill` -- suggest a ``(wall_coord, facing)`` pair for a
  reset node against the nearest field wall on a *given* axis.
* :func:`is_reset_wall_too_close` -- report whether a node stands off its
  chosen wall by less than the pivot-sweep clearance.

It is imported by the commands layer (``SetResetSpec.autofilled`` /
``autofilled_auto``) and the app (the reset-spec inspector handler), and is
unit-tested directly, so it stays dependency-light: stdlib only plus two
cheap module-level constants pulled from the planner package. **No Tk, no
matplotlib.**

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
    "reset_autofill_auto",
    "is_reset_wall_too_close",
]


# Axis ordering for the cross-axis tie-break: prefer ``"y"`` over ``"x"`` when
# two candidate walls stand off the node by the same distance.
_AXIS_RANK = {"y": 0, "x": 1}


def _wall_candidates(
    node_xy: tuple[float, float],
    field_mm: tuple[float, float],
) -> tuple[tuple[str, float, float, float], ...]:
    """Enumerate the four field walls a reset node could re-zero against.

    Shared by :func:`reset_autofill_auto` (which ranks all four) and
    :func:`reset_autofill` (which filters to one axis), so the two stay in
    lock-step on the wall coordinates, the away-from-wall facings, and the
    standoff metric.

    The facing points *away* from the wall (so the chassis backs onto it):
    x=0 -> 0.0 (+x / East), x=field_w -> 180.0 (-x / West), y=0 -> 90.0
    (+y / North), y=field_h -> 270.0 (-y / South).

    :param node_xy: the reset node position ``(x_cm, y_cm)``.
    :param field_mm: outer field size ``(width_mm, height_mm)`` in millimetres.
    :return: four ``(axis, wall_coord, facing, standoff)`` tuples, in the order
        ``x=0, x=field_w, y=0, y=field_h`` -- each axis's low wall first so a
        low-vs-far tie favors the ``0.0`` wall.
    """
    field_w_cm = field_mm[0] / 10.0
    field_h_cm = field_mm[1] / 10.0
    node_x, node_y = node_xy

    return (
        ("x", 0.0, 0.0, abs(0.0 - node_x)),
        ("x", field_w_cm, 180.0, abs(field_w_cm - node_x)),
        ("y", 0.0, 90.0, abs(0.0 - node_y)),
        ("y", field_h_cm, 270.0, abs(field_h_cm - node_y)),
    )


def reset_autofill_auto(
    node_xy: tuple[float, float],
    field_mm: tuple[float, float],
) -> tuple[str, float, float]:
    """Suggest a full reset spec against the nearest of all four field walls.

    A reset node re-zeros its localization against a field wall parallel to one
    axis. Unlike :func:`reset_autofill` (which is told the axis), this derives
    the axis from geometry: it ranks all four walls and returns the one the node
    sits closest to -- the wall the operator most plausibly placed it against.

    Standoff is ``abs(wall - node_coord)``; the relevant node coordinate is
    ``node_xy[0]`` for the x walls, ``node_xy[1]`` for the y walls. The chosen
    facing points *away* from the wall (x=0 -> 0.0, x=field_w -> 180.0,
    y=0 -> 90.0, y=field_h -> 270.0).

    Selection is ``min`` over the four candidates by the composite key
    ``(standoff, axis_rank, wall_rank)`` -- never an ``==`` float compare:

    * smallest standoff wins;
    * on an exact tie, ``axis_rank`` prefers ``"y"`` (0) over ``"x"`` (1);
    * within an axis, ``wall_rank`` prefers the low ``0.0`` wall (0) over the
      far wall (1).

    This **never raises** on a too-close wall: a node hugging a wall still
    returns that nearest wall (best-effort). The caller surfaces a warning and
    the Save-time validator re-checks the clearance against
    :data:`WALL_PIVOT_CLEARANCE_CM`, so the author is not silently handed an
    unrealisable reset -- see :func:`is_reset_wall_too_close`.

    :param node_xy: the reset node position ``(x_cm, y_cm)``.
    :param field_mm: outer field size ``(width_mm, height_mm)`` in millimetres.
    :return: ``(axis, wall_coord, facing)`` -- the axis (``"x"`` or ``"y"``),
        the suggested wall coordinate in centimetres, and the heading the
        chassis faces in degrees (``[0, 360)``).
    """
    candidates = _wall_candidates(node_xy, field_mm)

    # Composite key: nearest wall, then prefer y over x, then the low wall.
    axis, wall, facing, _standoff = min(
        candidates,
        key=lambda c: (c[3], _AXIS_RANK[c[0]], 0 if c[1] == 0.0 else 1),
    )
    return axis, wall, facing


def reset_autofill(
    node_xy: tuple[float, float],
    axis: str,
    field_mm: tuple[float, float],
) -> tuple[float, float]:
    """Suggest a ``(wall_coord, facing)`` for a reset node on the given axis.

    A reset node re-zeros its localization against a field wall parallel to one
    axis. This picks the wall and the heading the chassis faces while standing
    off it, given the node's position and the field size. The axis is supplied
    by the operator (the inspector combobox); :func:`reset_autofill_auto`
    derives the axis instead.

    Wall candidates and facings, per axis:

    * ``axis == "x"`` -- candidate walls are ``0.0`` and ``field_w_cm``
      (``field_mm[0] / 10``); the relevant node coordinate is ``node_xy[0]``.
      The chosen facing points *away* from the wall: ``0.0 deg`` (front toward
      +x / East) when the wall is the low ``0.0`` wall, else ``180.0 deg``.
    * ``axis == "y"`` -- candidate walls are ``0.0`` and ``field_h_cm``
      (``field_mm[1] / 10``); the relevant node coordinate is ``node_xy[1]``.
      The facing is ``90.0 deg`` (toward +y / North) when the wall is ``0.0``,
      else ``270.0 deg``.

    Wall selection: pick the **nearest** of the two walls on the axis (smallest
    standoff ``abs(wall - node_coord)``). A too-close near wall is *not* skipped
    for a far one -- the operator aimed at the nearer wall, and the Save-time
    validator re-checks the clearance against :data:`WALL_PIVOT_CLEARANCE_CM`.
    Ties (both standoffs equal) resolve to the ``0.0`` wall, since it sorts
    first in the candidate list.

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

    # The two candidates on this axis, low wall first so ties favor 0.0.
    on_axis = [
        (wall, facing, standoff)
        for cand_axis, wall, facing, standoff in _wall_candidates(node_xy, field_mm)
        if cand_axis == axis
    ]

    # Nearest wall on the axis; the low wall wins a tie since it sorts first.
    wall, facing, _standoff = min(on_axis, key=lambda c: c[2])
    return wall, facing


def is_reset_wall_too_close(
    node_xy: tuple[float, float],
    axis: str,
    wall_coord: float,
) -> bool:
    """Report whether a reset node stands off its wall by less than clearance.

    Single-sources the standoff check the editor and the Save-time validator
    share: the node clears the wall iff its standoff is at least
    :data:`WALL_PIVOT_CLEARANCE_CM` (the pivot-sweep radius plus margin). The
    relevant node coordinate is ``node_xy[0]`` for an x wall, ``node_xy[1]``
    for a y wall.

    :param node_xy: the reset node position ``(x_cm, y_cm)``.
    :param axis: ``"x"`` or ``"y"`` -- the axis the wall is parallel to.
    :param wall_coord: the wall coordinate in centimetres (e.g. ``0.0`` or the
        field width/height on that axis).
    :return: ``True`` iff the standoff is ``< WALL_PIVOT_CLEARANCE_CM`` (too
        close); ``False`` at exactly the clearance or beyond.
    :raises ValueError: if ``axis`` is not ``"x"`` or ``"y"``.
    """
    if axis not in _ALLOWED_AXES:
        raise ValueError(
            f"is_reset_wall_too_close: axis must be one of {_ALLOWED_AXES}; "
            f"got {axis!r}."
        )

    node_coord = node_xy[0] if axis == "x" else node_xy[1]
    return abs(wall_coord - node_coord) < WALL_PIVOT_CLEARANCE_CM
