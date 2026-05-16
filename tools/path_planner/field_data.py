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

# Outer field dimensions in millimetres. The visualizer overlays the
# matplotlib axes on map.png using this as the world extent so the
# pixel-to-cm scale is implicit (no separate constant needed).
FIELD_MM = (2500, 1000)  # PLACEHOLDER — remeasure tomorrow (issue #10)

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
}

# Allowed values: "line" (reached by line-following), "blind" (reached by
# turn + forward dead-reckoning).
NODE_TYPES = {
    "A": "line", "B": "line", "C": "line", "D": "line", "E": "line",
    "G": "blind",
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
