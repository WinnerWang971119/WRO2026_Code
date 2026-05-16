"""WRO 2026 field encoding (PR3).

PR3 simplification (vs PR1/PR2):

* No projection nodes. Every line-graph node is reached by line-following
  to the next intersection.
* No goal-approach blind sequences. Off-line positions are first-class
  nodes connected to the line graph by explicit ``BLIND_EDGES``.
* The robot's ``Mission`` DSL (``Mission.goTo("A").beep(...).wait(...)``)
  drives the sequence; the planner just answers "which next-hop gets me
  from X to Y."

Coordinate system: origin (0, 0) at field SW corner. X is East, Y is North.
Distances in centimetres. Headings in degrees, [0, 360), CCW from +x.

**Swap NODES / LINE_SEGMENTS / BLIND_EDGES / OBSTACLES for the real WRO 2026
Mosaic Masters field measurements before the run.** The values below are
a known-good placeholder layout the simulator + pytest suite verify against.
"""

from tools.path_planner.graph_builder import EDGE_LINE, EDGE_BLIND, EDGE_PROJECTION, Pivot
from tools.path_planner.pipeline_config import LineMode, BlindMode, ProjectionMode, TurnMode

# Outer field dimensions in millimetres. The visualizer overlays the
# matplotlib axes on map.png using this as the world extent so the
# pixel-to-cm scale is implicit (no separate constant needed).
FIELD_MM = (3000, 2000)

# Path (relative to tools/path_planner/) of the printed field PNG the
# visualizer drops in as the background. The image is purely cosmetic
# verification — the planner reads nothing from it.
MAP_PNG = "maps/map.png"

# name: (x_cm, y_cm)
NODES = {
    "A": (10.0,  10.0),
    "B": (200.0, 10.0),
    "C": (200.0, 150.0),
    "D": (10.0,  150.0),
    "E": (100.0, 80.0),
    "G": (250.0, 80.0),
    # P1: projection waypoint on the A→B line segment (no physical intersection
    # marker), used by the EDGES[0] PROJECTION demonstration entry below.
    "P1": (55.0, 10.0),
}

# Allowed values: "line" (reached by line-following), "blind" (reached by
# turn + forward dead-reckoning).
NODE_TYPES = {
    "A": "line", "B": "line", "C": "line", "D": "line", "E": "line",
    "G": "blind",
    "P1": "line",   # projection waypoint; no physical intersection cross
}

# Undirected; graph_builder produces two directed line edges per segment.
LINE_SEGMENTS = [
    ("A", "B"), ("B", "C"), ("C", "D"), ("D", "A"),
    ("A", "E"), ("B", "E"), ("C", "E"), ("D", "E"),
]

# Undirected blind hops. Each entry yields two directed EDGE_BLIND edges.
# Cost is scaled by COST_BLIND_MULTIPLIER so Dijkstra prefers line edges.
BLIND_EDGES = [
    ("B", "G"),
]

# Axis-aligned rectangles. Final tuple element is a free-form kind label.
# Inflated by ROBOT_RADIUS_CM before edges are intersected.
OBSTACLES = [
    # (x_min, y_min, x_max, y_max, kind)
]

ROBOT_RADIUS_CM = 8.0
COST_BLIND_MULTIPLIER = 5.0

START_NODE = "A"
START_HEADING_DEG = 0.0

# ---------------------------------------------------------------------------
# Rich edge list — demonstration fixtures for T11/T12/T13 tests.
#
# Schema: each dict MUST supply:
#   type          : EDGE_LINE | EDGE_BLIND | EDGE_PROJECTION
#   u             : source node name (must exist in NODES)
#   v             : destination node name (must exist in NODES)
#   distance_cm   : Euclidean length in centimetres (float)
#   heading_deg   : departure heading in degrees, CCW from +x, [0, 360)
#
# Optional overrides (absent = inherit the pipeline-wide PIPELINE_CONFIG
# default for that edge's category at C++ dispatch time):
#   primitive_mode : LineMode | BlindMode | ProjectionMode value
#   pre_turn_mode  : TurnMode value — wheel pattern for the approach turn
#   pivot          : Pivot enum value — single-wheel pivot selector
#
# graph_builder.build_graph() emits a bidirectional directed pair for each
# entry, forwarding all optional keys onto both directions verbatim.
# These entries are ADDITIVE alongside LINE_SEGMENTS / BLIND_EDGES, not a
# replacement; the planner merges all edge sources into one DiGraph.
# ---------------------------------------------------------------------------
EDGES = [
    # [0] PROJECTION demo — A to P1 along the A-B line strip; no intersection
    #     marker at P1 so the chassis stops by encoder distance, not by a
    #     detected cross. Exercises the EDGE_PROJECTION compile path (T8/T11).
    {
        "type":         EDGE_PROJECTION,
        "u":            "A",
        "v":            "P1",
        "distance_cm":  45.0,   # |P1 - A| = |(55-10, 10-10)| = 45 cm
        "heading_deg":  0.0,    # due East (+x direction)
    },

    # [1] LINE edge with a per-edge FAST preset — D to E at full cruise speed.
    #     Exercises the primitive_mode per-edge override path (T7/T12).
    #     Geometry: D(10,150)->E(100,80); dist≈114.02 cm, heading≈322.13° (NE quadrant CCW from +x).
    {
        "type":           EDGE_LINE,
        "u":              "D",
        "v":              "E",
        "distance_cm":    114.02,
        "heading_deg":    322.13,
        "primitive_mode": LineMode.FAST,
    },

    # [2] LINE edge with pivot=LEFT and matching pre_turn_mode=PIVOT_LEFT.
    #     C to E — demonstrates the pivot/pre_turn_mode relationship that T5
    #     compensation reads and T7 emits into routes.cpp. Having both set
    #     on the same entry makes the turn-geometry intent self-documenting.
    #     Geometry: C(200,150)->E(100,80); dist≈122.07 cm, heading≈214.99° (SW quadrant).
    {
        "type":           EDGE_LINE,
        "u":              "C",
        "v":              "E",
        "distance_cm":    122.07,
        "heading_deg":    214.99,
        "pre_turn_mode":  TurnMode.PIVOT_LEFT,
        "pivot":          Pivot.LEFT,
    },
]
