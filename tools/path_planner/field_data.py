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

Source of truth: the geometry (NODES / NODE_TYPES / LINE_SEGMENTS / BLIND_EDGES /
OBSTACLES plus FIELD_MM, MAP_PNG, ROBOT_RADIUS_CM, COST_BLIND_MULTIPLIER,
START_NODE, START_HEADING_DEG) lives in :file:`field_data.json` and is the file
the live editor (`edit_field.py`) mutates. This module is a thin loader that
re-exposes those values under their historical attribute names so callers do
not need to know about the JSON layout. The rich ``EDGES`` table stays in
Python because per-edge entries reference enum values (``LineMode``,
``TurnMode``, ``Pivot``) that JSON cannot represent (see ADR-022).

Tests that need a stable graph shape import
:mod:`tools.path_planner.tests.fixtures.toy_field_data` instead of this module
so editor saves do not flip the planner suite red.
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
# Rich edge list — optional per-edge overrides for the live field graph.
#
# This list is ADDITIVE alongside the geometry loaded from field_data.json
# (LINE_SEGMENTS / BLIND_EDGES). Entries here let an author attach enum-typed
# overrides (primitive_mode / pre_turn_mode / pivot) to a specific edge that
# JSON cannot represent. Leaving the list empty is fine — every edge then
# inherits the pipeline-wide PIPELINE_CONFIG defaults at C++ dispatch time.
# The example below is commented out and uses placeholder node names; uncomment
# (and rename ``u`` / ``v`` to nodes that exist in ``NODES``) to attach overrides.
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
