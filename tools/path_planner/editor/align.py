"""Multi-node "Make straight" alignment geometry for the WRO 2026 field editor.

Given a selection of node names -> ``(x, y)`` positions (in centimetres), this
module computes where each node should move to make the selection exactly
collinear, snapped to the selection's own **mean** rather than to an arbitrary
reference node -- so straightening a run does not silently adopt one node's
placement error as the target for every other node.

Three alignment axes are supported, chosen via :class:`AlignAxis`:

* ``X`` -- every node's x becomes the mean x; y is untouched.
* ``Y`` -- every node's y becomes the mean y; x is untouched.
* ``FIT`` -- an orthogonal (total-least-squares) regression line through the
  selection's centroid; every node is projected onto that line. On an
  axis-aligned run this is identical to ``X``/``Y``. On exactly two nodes it
  is a **no-op**: the TLS line through two points passes through both of
  them exactly, so each rounded target equals its own 4-dp-clean input.
* ``AUTO`` resolves to ``X`` or ``Y`` by comparing the selection's spread on
  each axis: the axis with the **smaller** spread is the one to collapse (an
  exact tie resolves to ``X``) -- see :func:`resolve_axis`.

Rounding rule (load-bearing, see :data:`ALIGN_ROUND_DP`): only the *aligned*
coordinate(s) are rounded, to 4 decimal places. ``X`` rounds x and copies y
through byte-for-byte; ``Y`` rounds y and copies x through byte-for-byte;
``FIT`` rounds both projected coordinates. Rounding the untouched axis too
would corrupt full-precision data (real field y-values carry 14+ significant
digits) for no benefit. Rounding happens exactly once, inside
:func:`align_points` -- callers must never round again.

:func:`max_residual_cm` reports the largest distance any node would travel,
but measures against the **raw, unrounded** reference (the true mean, or the
true projected point for ``FIT``) -- not the rounded target. The two differ
by roughly 1e-5 cm on real field data, which would otherwise silently
under-report the residual that motivated this feature in the first place.

This module is pure logic: no GUI imports, no networkx/shapely, no file I/O
(mirrors :mod:`tools.path_planner.editor.auto_edges` and
:mod:`tools.path_planner.editor.projection_edges`). It is imported by
``editor/app.py`` (which computes target coordinates and dispatches a
``MacroCommand`` of ``MoveNode`` steps -- this module never mutates a model
or a command) and is unit-tested directly in
``tools/path_planner/tests/test_editor_align.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

__all__ = [
    "AlignAxis",
    "ALIGN_ROUND_DP",
    "AlignPreview",
    "resolve_axis",
    "align_points",
    "max_residual_cm",
    "build_preview",
]


class AlignAxis(Enum):
    """The axis (or auto-detection mode) a "Make straight" alignment uses.

    ``AUTO`` is resolved to ``X`` or ``Y`` by :func:`resolve_axis`; ``X``,
    ``Y``, and ``FIT`` are terminal choices that every other function in this
    module consumes directly.
    """

    AUTO = "auto"
    X = "x"
    Y = "y"
    FIT = "fit"


#: Decimal places the aligned coordinate(s) are rounded to. 1e-4 cm is ~1 um --
#: far below any physical or planner-relevant scale -- while still keeping the
#: aligned nodes' JSON diff small and stable across repeated alignments.
ALIGN_ROUND_DP: int = 4


@dataclass(frozen=True)
class AlignPreview:
    """Render payload for the live guide line + ghost targets.

    Built by :func:`build_preview`; ``None`` in place of an instance signals
    "fewer than 2 nodes selected -- draw nothing" to the canvas.
    """

    #: The guide line endpoints, ``((x0, y0), (x1, y1))``, in centimetres.
    guide: tuple[tuple[float, float], tuple[float, float]]
    #: Movers only -- nodes whose rounded target differs from their current
    #: position -- mapped to their target ``(x, y)``. A node already sitting
    #: exactly on its target is omitted so its ghost dot is never drawn on
    #: top of its own selection ring.
    ghosts: dict[str, tuple[float, float]]


def _spread(values: list[float]) -> float:
    """Return ``max(values) - min(values)``, or ``0.0`` for an empty list.

    Shared by :func:`resolve_axis` for the x/y spread comparison that decides
    ``AUTO``.

    :param values: A list of coordinate values (all x's, or all y's).
    :return: The spread (max - min), or ``0.0`` if *values* is empty.
    """
    if not values:
        return 0.0
    return max(values) - min(values)


def _fit_line(
    points: dict[str, tuple[float, float]],
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Fit an orthogonal (total-least-squares) line through *points*.

    The line passes through the centroid by construction, matching this
    module's "snap to the mean" rule generalized to a diagonal. The
    orientation is found from the centered second moments ``Sxx``/``Syy``/
    ``Sxy``: ``theta = 0.5 * atan2(2*Sxy, Sxx - Syy)`` gives the angle of the
    principal axis of minimum orthogonal residual.

    :param points: Node name -> ``(x, y)`` in centimetres. Must have at
        least 1 entry (callers with 2+ points are the only real callers).
    :return: ``(centroid, u)`` where ``centroid`` is ``(cx, cy)`` and ``u`` is
        the unit direction vector ``(cos(theta), sin(theta))`` of the fitted
        line.
    """
    n = len(points)
    cx = sum(x for x, _ in points.values()) / n
    cy = sum(y for _, y in points.values()) / n
    sxx = sum((x - cx) ** 2 for x, _ in points.values())
    syy = sum((y - cy) ** 2 for _, y in points.values())
    sxy = sum((x - cx) * (y - cy) for x, y in points.values())
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    u = (math.cos(theta), math.sin(theta))
    return (cx, cy), u


def _project_onto_line(
    point: tuple[float, float],
    centroid: tuple[float, float],
    u: tuple[float, float],
) -> tuple[float, float]:
    """Orthogonally project *point* onto the line through *centroid* along *u*.

    :param point: The ``(x, y)`` point to project, in centimetres.
    :param centroid: A point the line passes through, ``(cx, cy)``.
    :param u: The line's unit direction vector, ``(ux, uy)``.
    :return: The projected point ``p' = centroid + ((point - centroid) . u) * u``.
    """
    dx = point[0] - centroid[0]
    dy = point[1] - centroid[1]
    t = dx * u[0] + dy * u[1]
    return (centroid[0] + t * u[0], centroid[1] + t * u[1])


def resolve_axis(
    points: dict[str, tuple[float, float]], axis: AlignAxis
) -> AlignAxis:
    """Resolve ``AUTO`` to ``X`` or ``Y`` by comparing the selection's spread.

    The axis with the **smaller** spread is the one to collapse: a selection
    that is already nearly-vertical (small x spread, large y spread) should
    align on ``X``, snapping the nodes onto a shared x; a nearly-horizontal
    selection should align on ``Y``. An exact tie (``spread_x == spread_y``)
    resolves to ``X``.

    ``X``, ``Y``, and ``FIT`` pass through unchanged -- only ``AUTO`` is
    resolved here.

    :param points: Node name -> ``(x, y)`` in centimetres.
    :param axis: The operator's requested axis (possibly ``AUTO``).
    :return: ``AlignAxis.X`` or ``AlignAxis.Y`` when *axis* is ``AUTO``;
        otherwise *axis* unchanged.
    """
    if axis is not AlignAxis.AUTO:
        return axis
    spread_x = _spread([xy[0] for xy in points.values()])
    spread_y = _spread([xy[1] for xy in points.values()])
    return AlignAxis.X if spread_x <= spread_y else AlignAxis.Y


def align_points(
    points: dict[str, tuple[float, float]], axis: AlignAxis
) -> dict[str, tuple[float, float]]:
    """Compute the target ``(x, y)`` for every node in *points*.

    ``AUTO`` is resolved first via :func:`resolve_axis`. Every input node
    gets a target -- including ones that do not move -- so callers can diff
    against the input to find movers (see :attr:`AlignPreview.ghosts`).

    Rounding (to :data:`ALIGN_ROUND_DP`) happens exactly once, here, and only
    on the aligned axis/axes:

    * ``X`` -- every node's x becomes ``round(mean_x, ALIGN_ROUND_DP)``; y is
      copied through byte-for-byte (not rounded).
    * ``Y`` -- every node's y becomes ``round(mean_y, ALIGN_ROUND_DP)``; x is
      copied through byte-for-byte (not rounded).
    * ``FIT`` -- every node is projected onto the orthogonal-regression line
      through the centroid (see :func:`_fit_line`), then **both** projected
      coordinates are rounded to ``ALIGN_ROUND_DP``. On exactly 2 input
      nodes this is a no-op: the TLS line through 2 points passes through
      both exactly, so each rounded projection equals its 4-dp-clean input.

    :param points: Node name -> ``(x, y)`` in centimetres. Must have at
        least 2 entries.
    :param axis: The requested alignment axis (``AUTO`` is resolved
        internally).
    :return: Node name -> target ``(x, y)`` in centimetres, one entry per
        input node.
    :raises ValueError: if *points* has fewer than 2 entries.
    """
    if len(points) < 2:
        raise ValueError(
            f"align_points requires at least 2 points; got {len(points)}."
        )

    resolved = resolve_axis(points, axis)

    if resolved is AlignAxis.X:
        mean_x = sum(xy[0] for xy in points.values()) / len(points)
        target_x = round(mean_x, ALIGN_ROUND_DP)
        return {name: (target_x, xy[1]) for name, xy in points.items()}

    if resolved is AlignAxis.Y:
        mean_y = sum(xy[1] for xy in points.values()) / len(points)
        target_y = round(mean_y, ALIGN_ROUND_DP)
        return {name: (xy[0], target_y) for name, xy in points.items()}

    # FIT: orthogonal/total-least-squares projection through the centroid.
    centroid, u = _fit_line(points)
    targets: dict[str, tuple[float, float]] = {}
    for name, xy in points.items():
        px, py = _project_onto_line(xy, centroid, u)
        targets[name] = (round(px, ALIGN_ROUND_DP), round(py, ALIGN_ROUND_DP))
    return targets


def max_residual_cm(
    points: dict[str, tuple[float, float]], axis: AlignAxis
) -> float:
    """Return the largest Euclidean distance any node would travel.

    This drives the ALIGN inspector's readout. The reference is the **raw,
    unrounded** mean (for ``X``/``Y``) or the raw, unrounded projected point
    (for ``FIT``) -- deliberately *not* the rounded target that
    :func:`align_points` returns. The two differ by roughly 1e-5 cm on real
    field data; measuring against the rounded target would silently
    under-report the true residual.

    For ``X``/``Y`` the move is purely along the aligned axis (the other
    axis is untouched), so the Euclidean distance collapses to
    ``abs(aligned_coord - raw_mean)``.

    :param points: Node name -> ``(x, y)`` in centimetres. Must have at
        least 2 entries.
    :param axis: The requested alignment axis (``AUTO`` is resolved
        internally).
    :return: The largest distance, in centimetres, any node would travel
        under this alignment.
    :raises ValueError: if *points* has fewer than 2 entries.
    """
    if len(points) < 2:
        raise ValueError(
            f"max_residual_cm requires at least 2 points; got {len(points)}."
        )

    resolved = resolve_axis(points, axis)

    if resolved is AlignAxis.X:
        mean_x = sum(xy[0] for xy in points.values()) / len(points)
        return max(abs(xy[0] - mean_x) for xy in points.values())

    if resolved is AlignAxis.Y:
        mean_y = sum(xy[1] for xy in points.values()) / len(points)
        return max(abs(xy[1] - mean_y) for xy in points.values())

    # FIT: distance from each raw point to its own raw (unrounded) projection.
    centroid, u = _fit_line(points)
    max_dist = 0.0
    for xy in points.values():
        px, py = _project_onto_line(xy, centroid, u)
        dist = math.hypot(xy[0] - px, xy[1] - py)
        if dist > max_dist:
            max_dist = dist
    return max_dist


def build_preview(
    points: dict[str, tuple[float, float]], axis: AlignAxis
) -> AlignPreview | None:
    """Build the guide-line + ghost-target preview for a candidate alignment.

    :func:`align_points` supplies the rounded targets; this just packages
    them into the render payload the canvas overlay consumes, plus a guide
    line spanning the selection:

    * ``X``/``Y`` -- the guide spans the selection's extent on the *other*
      axis at the aligned coordinate (e.g. for ``X``, from the lowest to the
      highest input y at ``x = target_x``).
    * ``FIT`` -- the guide spans the two most extreme *projected* points
      along the fitted line's direction (found by ordering the raw points by
      their signed projection past the centroid).

    Ghosts include only movers (see :attr:`AlignPreview.ghosts`); a selection
    that is already exactly on its rounded target produces an empty
    ``ghosts`` dict (nothing to draw).

    :param points: Node name -> ``(x, y)`` in centimetres.
    :param axis: The requested alignment axis (``AUTO`` is resolved
        internally).
    :return: An :class:`AlignPreview`, or ``None`` if *points* has fewer than
        2 entries (nothing to preview).
    """
    if len(points) < 2:
        return None

    resolved = resolve_axis(points, axis)
    targets = align_points(points, resolved)

    if resolved is AlignAxis.X:
        aligned_x = next(iter(targets.values()))[0]
        ys = [xy[1] for xy in points.values()]
        guide = ((aligned_x, min(ys)), (aligned_x, max(ys)))
    elif resolved is AlignAxis.Y:
        aligned_y = next(iter(targets.values()))[1]
        xs = [xy[0] for xy in points.values()]
        guide = ((min(xs), aligned_y), (max(xs), aligned_y))
    else:
        # FIT: order the raw points by their signed projection past the
        # centroid along the fitted direction, then take the two extremes'
        # (already-rounded) targets as the guide endpoints.
        centroid, u = _fit_line(points)

        def _projection_scalar(name: str) -> float:
            px, py = points[name]
            return (px - centroid[0]) * u[0] + (py - centroid[1]) * u[1]

        ordered_names = sorted(points.keys(), key=_projection_scalar)
        guide = (targets[ordered_names[0]], targets[ordered_names[-1]])

    ghosts = {
        name: target
        for name, target in targets.items()
        if target != points[name]
    }
    return AlignPreview(guide=guide, ghosts=ghosts)
