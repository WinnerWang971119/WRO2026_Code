"""Pure geometry helpers for PROJECTION edges + reset auto-fill (T2).

This module is the Tk-free, matplotlib-free core that the live field
editor's projection-edge tooling and reset-node auto-fill UI lean on. It
holds four pure functions:

* :func:`read_projection_edges` -- pluck the ``EDGE_PROJECTION`` entries out
  of a field-data module's ``EDGES`` list (defensive against malformed
  entries and a missing ``EDGES`` attribute).
* :func:`compute_projection_geometry` -- derive ``(distance_cm,
  heading_deg)`` for a projection edge between two points.
* :func:`reconcile` -- fold a set of loaded projection edges with pending
  adds and removes into one deterministic render set.
* :func:`reset_autofill` -- suggest a ``(wall_coord, facing)`` pair for a
  reset node against the nearer qualifying field wall.

It is imported by the commands layer (T4), the canvas (T7), the app (T8),
and is unit-tested directly (T9), so it stays dependency-light: stdlib
:mod:`math` plus two cheap module-level constants pulled from the planner
package. **No Tk, no matplotlib.**

Conventions (inherited from ``field_data.py`` / ``graph_builder.py``):
    Origin at the field SW corner. X is East, Y is North. Distances in
    centimetres. Headings in degrees, ``[0, 360)``, CCW from the +x axis.
    ``FIELD_MM`` is ``(width_mm, height_mm)`` in millimetres -- divide by
    10 to get centimetres.
"""

from __future__ import annotations

from math import atan2, degrees, hypot

from tools.path_planner.editor.model import RESET_AXIS_VALUES
from tools.path_planner.graph_builder import EDGE_PROJECTION

# MUST mirror validator.WALL_PIVOT_CLEARANCE_CM -- the pivot-sweep radius
# plus safety margin a reset node needs to stand off the wall it re-zeros
# against. Imported directly (it is a cheap top-level float constant) so the
# value cannot silently desync from the Save-time validator that re-checks it.
from tools.path_planner.validator import WALL_PIVOT_CLEARANCE_CM

# Decimal places to which projection geometry is rounded. Trims float noise so
# the value the editor writes into source is stable across runs (a re-derived
# heading otherwise differs in the 12th decimal and churns the diff).
_GEOMETRY_ROUND_DP = 4

# The two axes a reset node may re-zero against (a wall parallel to one axis).
# Single-sourced from ``model.RESET_AXIS_VALUES`` (a PUBLIC constant -- not a
# leading-underscore private of another module), which mirrors
# ``validator._ALLOWED_RESET_AXES``.
_ALLOWED_AXES = RESET_AXIS_VALUES

# Required keys on a PROJECTION entry in a field-data ``EDGES`` list. An entry
# missing any of these is skipped rather than crashed on (a half-authored edge
# in a live-edited module should not take the whole read down).
_REQUIRED_EDGE_KEYS = ("u", "v", "distance_cm", "heading_deg")

__all__ = [
    "WALL_PIVOT_CLEARANCE_CM",
    "read_projection_edges",
    "compute_projection_geometry",
    "reconcile",
    "reset_autofill",
]


def read_projection_edges(
    field_data_module: object,
) -> list[tuple[str, str, float, float]]:
    """Return the PROJECTION edges authored in a field-data module's ``EDGES``.

    Iterates ``field_data_module.EDGES`` (a list of dicts) in source order and
    yields one ``(u, v, distance_cm, heading_deg)`` tuple for every entry whose
    ``type`` equals :data:`graph_builder.EDGE_PROJECTION`. Non-PROJECTION
    entries are ignored.

    The read is deliberately defensive so a half-authored or malformed module
    (the editor mutates ``field_data`` live) never crashes the caller:

    * a missing ``EDGES`` attribute yields ``[]``;
    * an ``EDGES`` value that is not iterable yields ``[]``;
    * a non-dict entry, or a PROJECTION entry missing any of the required keys
      (``u``, ``v``, ``distance_cm``, ``heading_deg``), is skipped.

    ``distance_cm`` and ``heading_deg`` are coerced to ``float``; an entry whose
    distance or heading is non-numeric is skipped rather than propagated as a
    bad render value.

    :param field_data_module: the ``tools.path_planner.field_data`` module, or
        any object exposing an ``EDGES`` list-of-dicts attribute (or none).
    :return: a list of ``(u, v, distance_cm, heading_deg)`` tuples in the source
        order of ``EDGES``; ``[]`` when ``EDGES`` is absent or empty.
    """
    edges = getattr(field_data_module, "EDGES", None)
    if edges is None:
        return []
    try:
        iterator = iter(edges)
    except TypeError:
        # EDGES is present but not iterable -- treat as no projection edges.
        return []

    projections: list[tuple[str, str, float, float]] = []
    for entry in iterator:
        if not isinstance(entry, dict):
            continue
        if entry.get("type") != EDGE_PROJECTION:
            continue
        if any(key not in entry for key in _REQUIRED_EDGE_KEYS):
            continue
        try:
            distance_cm = float(entry["distance_cm"])
            heading_deg = float(entry["heading_deg"])
        except (TypeError, ValueError):
            # A non-numeric distance/heading is a malformed entry -- skip it
            # rather than surface a bad value to the renderer.
            continue
        projections.append((entry["u"], entry["v"], distance_cm, heading_deg))
    return projections


def compute_projection_geometry(
    p_u: tuple[float, float],
    p_v: tuple[float, float],
) -> tuple[float, float]:
    """Return ``(distance_cm, heading_deg)`` for a projection edge ``u -> v``.

    The distance is the Euclidean length ``hypot(dx, dy)``; the heading is
    ``degrees(atan2(dy, dx)) % 360.0`` -- the module convention (``[0, 360)``,
    ``0 deg`` = +x / East, CCW positive), matching
    :func:`graph_builder._heading_deg`. Both results are rounded to
    :data:`_GEOMETRY_ROUND_DP` decimal places so the value the editor writes
    into ``field_data`` source is byte-stable across runs.

    :param p_u: the source point ``(x_cm, y_cm)``.
    :param p_v: the destination point ``(x_cm, y_cm)``.
    :return: ``(distance_cm, heading_deg)`` -- both ``float``, rounded to
        4 decimals.
    :raises ValueError: if ``p_u`` and ``p_v`` are the same point. A
        zero-length projection has no defined heading (``atan2(0, 0)`` is
        meaningless here) and is never a valid edge.
    """
    dx = p_v[0] - p_u[0]
    dy = p_v[1] - p_u[1]
    if dx == 0.0 and dy == 0.0:
        raise ValueError(
            f"compute_projection_geometry: p_u and p_v are identical "
            f"({p_u!r}); a zero-length projection edge has no heading. "
            f"Place the destination node away from the source."
        )
    distance_cm = round(hypot(dx, dy), _GEOMETRY_ROUND_DP)
    heading_deg = round(degrees(atan2(dy, dx)) % 360.0, _GEOMETRY_ROUND_DP)
    return distance_cm, heading_deg


def reconcile(
    loaded: list[tuple[str, str, float, float]],
    adds: list[tuple[str, str, float, float]],
    removes: list[tuple[str, str]],
) -> list[tuple[str, str, float, float]]:
    """Fold loaded projection edges with pending adds/removes into a render set.

    The render set is ``loaded`` plus ``adds``, minus every entry whose
    ``(u, v)`` pair appears in ``removes``. Matching -- for both removal and
    de-duplication -- is on the ``(u, v)`` first-two-elements only; the
    ``distance_cm`` / ``heading_deg`` payload is not compared, so an add that
    re-states an existing ``(u, v)`` *overrides* the loaded entry's geometry.

    Determinism (so the canvas paints the same order every frame and tests are
    stable):

    1. Surviving ``loaded`` entries come first, in their original source order.
       An entry is dropped if its ``(u, v)`` is in ``removes`` OR is overridden
       by a same-pair entry in ``adds``.
    2. Then the ``adds``, in their given order, each emitted once, skipping any
       whose ``(u, v)`` is in ``removes`` and any whose ``(u, v)`` was already
       emitted (an earlier add wins over a later duplicate).

    A pair listed in ``removes`` is excluded even if it appears only in
    ``adds`` -- a remove suppresses a pending add of the same pair.

    :param loaded: projection edges read from the field-data module
        (:func:`read_projection_edges` output), in source order.
    :param adds: pending projection edges to add this session, in user order.
        A same-``(u, v)`` add overrides the loaded entry.
    :param removes: ``(u, v)`` pairs to suppress; only the first two elements of
        each tuple are read, so a 4-tuple edge may be passed directly.
    :return: the de-duplicated, remove-filtered render set as a list of
        ``(u, v, distance_cm, heading_deg)`` tuples in the order described
        above.
    """
    removed_pairs = {(pair[0], pair[1]) for pair in removes}
    add_pairs = {(edge[0], edge[1]) for edge in adds}

    render: list[tuple[str, str, float, float]] = []
    emitted: set[tuple[str, str]] = set()

    # 1. Surviving loaded entries first (source order). Drop a loaded entry if
    #    it is removed, or if an add re-states the same pair (the add overrides).
    for edge in loaded:
        pair = (edge[0], edge[1])
        if pair in removed_pairs or pair in add_pairs:
            continue
        if pair in emitted:
            # Defensive: a duplicate pair already in `loaded` -- keep the first.
            continue
        render.append(edge)
        emitted.add(pair)

    # 2. Then the adds (user order), each pair once, skipping removed pairs.
    for edge in adds:
        pair = (edge[0], edge[1])
        if pair in removed_pairs or pair in emitted:
            continue
        render.append(edge)
        emitted.add(pair)

    return render


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
