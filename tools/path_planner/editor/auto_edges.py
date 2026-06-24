"""Auto-edge proposal helper for the WRO 2026 live field editor.

Given a :class:`~tools.path_planner.editor.model.FieldModel`, propose a list of
undirected edges to add to the field graph based on a Delaunay triangulation
of the node positions, with the following filters applied in order:

1. **Length cap** -- drop candidates longer than ``max_edge_length_cm``.
2. **Obstacle filter** -- drop candidates whose segment clips an inflated
   obstacle (re-using :func:`tools.path_planner.graph_builder.inflate_obstacles`
   and :func:`tools.path_planner.graph_builder.edge_clips_obstacle` so the
   editor and the static planner agree on what "clear" means).
3. **Skip pre-existing** -- drop candidates that already appear in
   ``model.line_segments`` or ``model.blind_edges`` (undirected match).
4. **Type classification** -- :func:`_classify_edge_type` assigns
   ``"blind"`` if either endpoint is a blind or reset node, otherwise
   ``"line"``.  Projection nodes are NOT treated as blind (ADR-046: a
   LINE edge into a projection node is legal).

The resulting list is deduplicated (undirected) and sorted lexicographically
by ``(min(a, b), max(a, b))`` so callers get a deterministic preview every
run.  Degenerate inputs (fewer than 3 nodes, all-collinear points, or every
candidate filtered out) return an empty list -- the caller is expected to
surface that as a status-bar message and refuse to enter preview mode
(AC11).

This module is pure logic: no GUI imports, no networkx, no file I/O.  The
:class:`~tools.path_planner.editor.commands.AddEdgesBatch` command consumes
the output to mutate the model under the undo/redo stack.
"""

from __future__ import annotations

from typing import Iterable, Literal

from shapely.geometry import MultiPoint
from shapely.ops import triangulate

from tools.path_planner.editor.model import FieldModel
from tools.path_planner.graph_builder import edge_clips_obstacle, inflate_obstacles

# Public type aliases -- exported so command/GUI layers can name them without
# re-declaring the tuple shape.
EdgeType = Literal["line", "blind"]
ProposedEdge = tuple[str, str, EdgeType]

# Tolerance (in centimetres) used when matching shapely-returned triangle
# vertices back to the original node coordinates.  shapely round-trips floats
# through GEOS, which can introduce sub-ULP differences; 1e-6 cm = 10 nm is
# far smaller than any real placement precision while still being looser than
# the noise floor.
_VERTEX_MATCH_TOLERANCE_CM = 1e-6

__all__ = ["EdgeType", "ProposedEdge", "propose_edges"]


def _classify_edge_type(
    node_types: dict[str, str], a: str, b: str
) -> EdgeType:
    """Classify an undirected edge as ``"line"`` or ``"blind"``.

    Rule: if either endpoint is typed ``"blind"`` **or** ``"reset"``, the
    edge is a blind edge; otherwise the edge is a line edge.  Reset nodes
    must be reached by BLIND edges (ADR-047/ADR-048), so a reset endpoint is
    treated identically to a blind one here.  Projection endpoints are NOT
    included in this set -- a LINE edge into a projection node is legal
    (ADR-046) and projection nodes deliberately remain ``"line"``-classified.

    Endpoints absent from *node_types* fall back to ``"line"`` so a
    half-populated model still classifies sensibly -- :func:`propose_edges`
    itself only ever sees endpoints that came out of ``model.nodes``, and the
    validator guarantees every node has a matching ``node_types`` entry, so
    the default arm is purely defensive.

    :param node_types: Mapping of node name to its type string (e.g.
        ``"line"``, ``"blind"``, ``"reset"``, ``"projection"``).
    :param a: First endpoint name.
    :param b: Second endpoint name.
    :return: ``"blind"`` if either endpoint is blind or reset, else ``"line"``.
    """
    ta = node_types.get(a, "line")
    tb = node_types.get(b, "line")
    if ta in ("blind", "reset") or tb in ("blind", "reset"):
        return "blind"
    return "line"


def _edge_in_list(
    edges: Iterable[tuple[str, str]], a: str, b: str
) -> bool:
    """Return ``True`` iff the undirected pair ``(a, b)`` is in *edges*.

    The check is undirected: ``("x", "y")`` and ``("y", "x")`` both count as
    matches.  This mirrors the helper in
    :mod:`tools.path_planner.editor.commands`; it is duplicated here to keep
    ``auto_edges`` free of cross-module private imports (per CLAUDE.md "no
    half-finished imports" rule).

    :param edges: The edge list to search (typically ``line_segments`` or
        ``blind_edges``).
    :param a: First endpoint name.
    :param b: Second endpoint name.
    :return: ``True`` iff the undirected pair appears in *edges*.
    """
    for u, v in edges:
        if (u == a and v == b) or (u == b and v == a):
            return True
    return False


def _name_for_vertex(
    px: float,
    py: float,
    points: list[tuple[str, float, float]],
) -> str | None:
    """Find the node name whose ``(x, y)`` is closest to ``(px, py)``.

    shapely's :func:`shapely.ops.triangulate` may round-trip vertex
    coordinates through GEOS, producing values that are *not* bit-exact
    matches to the originals.  Building a dict keyed on ``(x, y)`` floats is
    therefore unreliable; instead, every triangle vertex is matched back to
    the closest input node by Euclidean distance with a small tolerance.

    :param px: Triangle-vertex x-coordinate, in centimetres.
    :param py: Triangle-vertex y-coordinate, in centimetres.
    :param points: List of ``(name, x, y)`` for every input node.
    :return: The matching node name, or ``None`` if no node lies within
        :data:`_VERTEX_MATCH_TOLERANCE_CM` of ``(px, py)``.
    """
    best_name: str | None = None
    best_dist_sq = float("inf")
    for name, nx_, ny_ in points:
        dx = nx_ - px
        dy = ny_ - py
        dist_sq = dx * dx + dy * dy
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_name = name
    if best_dist_sq > _VERTEX_MATCH_TOLERANCE_CM * _VERTEX_MATCH_TOLERANCE_CM:
        return None
    return best_name


def propose_edges(
    model: FieldModel,
    max_edge_length_cm: float,
) -> list[ProposedEdge]:
    """Propose edges to add to *model* based on Delaunay triangulation.

    Pipeline (each step is documented inline):

    1. Collect every node's ``(name, x, y)``.  If there are fewer than three,
       return ``[]`` -- you need at least three non-collinear points for a
       valid triangulation.
    2. Run :func:`shapely.ops.triangulate` over the node positions.  shapely
       returns ``[]`` for collinear inputs and may raise on other degenerate
       configurations; both are caught and result in an empty proposal list.
    3. Map every triangle vertex back to a node name via
       :func:`_name_for_vertex`.
    4. Extract undirected pairs from each triangle's exterior ring (3 edges
       per triangle) and deduplicate.  Pairs are normalised so the smaller
       name is always first; this is what gives the final output its
       deterministic ``a < b`` ordering.
    5. Drop pairs whose Euclidean length exceeds ``max_edge_length_cm``.
    6. Drop pairs whose segment clips any inflated obstacle.  Obstacles are
       inflated by ``model.robot_radius_cm`` exactly the same way the static
       planner does in :mod:`tools.path_planner.graph_builder`.
    7. Drop pairs that already exist in ``model.line_segments`` or
       ``model.blind_edges`` (undirected match).
    8. Classify each surviving pair with :func:`_classify_edge_type`.
    9. Sort the survivors lexicographically by ``(a, b)`` and return.

    :param model: The in-memory field model to propose edges for.  Not
        mutated.
    :param max_edge_length_cm: Maximum allowed Euclidean length, in
        centimetres, for any proposed edge.
    :return: A deterministic, sorted list of
        ``(node_a, node_b, "line" | "blind")`` tuples with ``node_a <
        node_b``.  Empty on degenerate input or when every candidate is
        filtered out.
    """
    # Step 1: collect nodes.
    points: list[tuple[str, float, float]] = [
        (name, xy[0], xy[1]) for name, xy in model.nodes.items()
    ]
    if len(points) < 3:
        return []

    # Step 2: Delaunay.  shapely returns [] for collinear input; we still
    # wrap the call in a broad except so any GEOS-side degeneracy surfaces
    # as an empty proposal list rather than crashing the GUI.  AC11 covers
    # all three degenerate cases (< 3 nodes / collinear / all filtered) with
    # the same "do nothing, status-bar message" behavior, so falling back to
    # [] here is the right call.
    try:
        triangles = triangulate(
            MultiPoint([(x, y) for _, x, y in points])
        )
    except Exception:  # noqa: BLE001 -- shapely/GEOS errors aren't a known set
        return []
    if not triangles:
        return []

    # Steps 3 + 4: pull undirected, name-normalised pairs out of every
    # triangle.  A shapely Polygon's exterior is a closed ring of 4 vertices
    # (first == last); the 3 distinct sides are the 3 consecutive pairs of
    # the first 3 entries.
    pair_set: set[tuple[str, str]] = set()
    for triangle in triangles:
        coords = list(triangle.exterior.coords)
        # Defensive guard: a valid triangle exterior has 4 coords (closed
        # ring).  Skip anything else so we never index out of bounds.
        if len(coords) < 4:
            continue
        names: list[str] = []
        for px, py in coords[:3]:
            matched = _name_for_vertex(px, py, points)
            if matched is None:
                names = []
                break
            names.append(matched)
        if len(names) != 3:
            continue
        for i in range(3):
            a = names[i]
            b = names[(i + 1) % 3]
            if a == b:
                # Degenerate triangle (shouldn't happen for real Delaunay
                # output, but skip rather than emit a self-loop).
                continue
            ordered = (a, b) if a < b else (b, a)
            pair_set.add(ordered)

    if not pair_set:
        return []

    # Step 6 (prep): inflate obstacles once for the whole batch.  Passing
    # the same inflated polygons to every ``edge_clips_obstacle`` call
    # avoids re-buffering the rectangles on every candidate.
    inflated = inflate_obstacles(model.obstacles, model.robot_radius_cm)

    # Steps 5 + 6 + 7 + 8: filter and classify.  Iterate the sorted pair
    # set so the survivor list comes out pre-sorted -- step 9's final sort
    # then becomes a no-op, but we still call it explicitly so the
    # determinism guarantee in the docstring isn't load-bearing on the
    # ordering of ``sorted(set(...))``.
    proposed: list[ProposedEdge] = []
    max_len_sq = max_edge_length_cm * max_edge_length_cm
    for a, b in sorted(pair_set):
        ax, ay = model.nodes[a]
        bx, by = model.nodes[b]
        # Step 5: length cap (compare squared distances to avoid sqrt).
        dx = bx - ax
        dy = by - ay
        if dx * dx + dy * dy > max_len_sq:
            continue
        # Step 6: obstacle filter.
        if edge_clips_obstacle((ax, ay), (bx, by), inflated):
            continue
        # Step 7: skip pre-existing pairs in either list.
        if _edge_in_list(model.line_segments, a, b):
            continue
        if _edge_in_list(model.blind_edges, a, b):
            continue
        # Step 8: classify.
        kind = _classify_edge_type(model.node_types, a, b)
        proposed.append((a, b, kind))

    # Step 9: final deterministic sort.  Iterating ``sorted(pair_set)``
    # above already produces this order, but the explicit sort here makes
    # the public guarantee independent of the internal iteration order.
    proposed.sort(key=lambda edge: (edge[0], edge[1]))
    return proposed
