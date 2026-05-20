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

from tools.path_planner.editor.loader import load_field_json
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION, Pivot  # noqa: F401
from tools.path_planner.pipeline_config import LineMode, BlindMode, ProjectionMode, TurnMode  # noqa: F401

_data = load_field_json()
FIELD_MM              = _data["FIELD_MM"]
MAP_PNG               = _data["MAP_PNG"]
ROBOT_RADIUS_CM       = _data["ROBOT_RADIUS_CM"]
COST_BLIND_MULTIPLIER = _data["COST_BLIND_MULTIPLIER"]
START_NODE            = _data["START_NODE"]
START_HEADING_DEG     = _data["START_HEADING_DEG"]
NODES                 = _data["NODES"]
NODE_TYPES            = _data["NODE_TYPES"]
LINE_SEGMENTS         = _data["LINE_SEGMENTS"]
BLIND_EDGES           = _data["BLIND_EDGES"]
OBSTACLES             = _data["OBSTACLES"]

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
    #{
    #    "type":         EDGE_PROJECTION,
    #    "u":            "A",
    #    "v":            "P1",
    #    "distance_cm":  45.0,   # |P1 - A| = |(55-10, 10-10)| = 45 cm
    #    "heading_deg":  0.0,    # due East (+x direction)
    #},
]
