"""Toy mixed-feature field for PR 1 testing.

Replaces in PR 3 with the real 2026 Mosaic Masters layout. PR 1 ships
this fixture so every downstream module (graph_builder, planner, compiler,
header_writer) has a stable, reviewable input that exercises every edge
type: line, projection, blind, plus an obstacle-forced detour.

Coordinate system: origin (0, 0) at field SW corner. X is East, Y is North.
Distances in centimetres. Headings in degrees, [0, 360), CCW from +x.
"""

# name: (x_cm, y_cm)
NODES = {
    "A":       (0.0,  0.0),
    "B":       (60.0, 0.0),
    "C":       (60.0, 60.0),
    "D":       (0.0,  60.0),
    "E":       (30.0, 30.0),     # center hub
    "_proj_G": (60.0, 30.0),     # waypoint on segment B-C
    "G":       (90.0, 30.0),     # off-line goal, blind-reachable from _proj_G
}

# Allowed values: "line", "projection", "goal".
NODE_TYPES = {
    "A": "line", "B": "line", "C": "line", "D": "line", "E": "line",
    "_proj_G": "projection",
    "G": "goal",
}

# Undirected; graph_builder produces two directed edges per segment.
LINE_SEGMENTS = [
    ("A", "B"), ("B", "C"), ("C", "D"), ("D", "A"),
    ("A", "E"), ("B", "E"), ("C", "E"), ("D", "E"),
]

# Projection nodes are manually placed on a specific line segment at a
# specific distance from one endpoint. The graph builder splits the
# parent segment at this point.
PROJECTIONS = {
    "_proj_G": {"on_segment": ("B", "C"), "from": "B", "distance_cm": 30.0},
}

# How to enter and act at each goal node. approach_heading_deg is the
# absolute heading the chassis must have when arriving at `entry`;
# blind_moves run after arrival.
#   ("turn_relative", deg)  -- relative turn, degrees, CCW positive
#   ("forward",       cm)   -- straight forward, centimetres
GOAL_APPROACH = {
    "G": {
        "entry": "_proj_G",
        "approach_heading_deg": 0.0,
        "blind_moves": [
            ("turn_relative", 0.0),
            ("forward",       30.0),
        ],
    },
}

# Axis-aligned rectangles. Final tuple element is a free-form kind label.
OBSTACLES = [
    # (x_min, y_min, x_max, y_max, kind)
    (40.0, 40.0, 50.0, 50.0, "wall"),
]

ROBOT_RADIUS_CM = 8.0
COST_BLIND_MULTIPLIER = 5.0

START_NODE = "A"
START_HEADING_DEG = 0.0
