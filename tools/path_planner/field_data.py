"""WRO 2026 field encoding (PR3).

PR3 simplification (vs PR1/PR2):

* Projection is a NODE property, not an edge type (issue #64): a
  ``"projection"``-typed node sits on a line but not at a junction, and the
  hop into it is encoder-distance- (LINE edge) or auto-seek- (BLIND edge)
  terminated. The production field below carries no projection nodes this
  phase (deferred to P3.5); see the EDGES authoring comment for how to add
  one when the time comes.
* No goal-approach blind sequences. Off-line positions are first-class
  nodes connected to the line graph by explicit ``BLIND_EDGES``.
* The robot's ``Mission`` DSL (``Mission.goTo("A").beep(...).wait(...)``)
  drives the sequence; the planner just answers "which next-hop gets me
  from X to Y."

Coordinate system: origin (0, 0) at field SW corner. X is East, Y is North.
Distances in centimetres. Headings in degrees, [0, 360), CCW from +x.

Source of truth: the geometry (NODES / NODE_TYPES / LINE_SEGMENTS / BLIND_EDGES /
OBSTACLES plus FIELD_MM, MAP_PNG, ROBOT_RADIUS_CM, the BLIND_DECAY_* knobs
(BLIND_DECAY_ENABLED / BLIND_DECAY_K / BLIND_DECAY_EXPONENT / BLIND_DECAY_D_REF),
START_NODE, START_HEADING_DEG) lives in :file:`field_data.json` and is the file
the live editor (`edit_field.py`) mutates. This module is a thin loader that
re-exposes those values under their historical attribute names so callers do
not need to know about the JSON layout. The rich ``EDGES`` table stays in
Python because per-edge entries reference enum values (``LineMode``,
``TurnMode``) that JSON cannot represent (see ADR-022).

Single-wheel pivots are authored per turn, not per edge: ``field_data.json``
carries a ``pivot_turns`` list of ``["<u>", "<v>", "<w>"]`` node-name triples,
re-exposed here as :data:`PIVOT_TURNS` (a list of ``(u, v, w)`` string tuples).

Tests that need a stable graph shape import
:mod:`tools.path_planner.tests.fixtures.toy_field_data` instead of this module
so editor saves do not flip the planner suite red.
"""

from tools.path_planner.editor.loader import load_field_json
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE  # noqa: F401
from tools.path_planner.pipeline_config import LineMode, BlindMode, TurnMode  # noqa: F401

_data = load_field_json()
FIELD_MM              = _data["FIELD_MM"]
MAP_PNG               = _data["MAP_PNG"]
ROBOT_RADIUS_CM       = _data["ROBOT_RADIUS_CM"]
BLIND_DECAY_ENABLED   = _data["BLIND_DECAY_ENABLED"]
BLIND_DECAY_K         = _data["BLIND_DECAY_K"]
BLIND_DECAY_EXPONENT  = _data["BLIND_DECAY_EXPONENT"]
BLIND_DECAY_D_REF     = _data["BLIND_DECAY_D_REF"]
START_NODE            = _data["START_NODE"]
START_HEADING_DEG     = _data["START_HEADING_DEG"]
NODES                 = _data["NODES"]
NODE_TYPES            = _data["NODE_TYPES"]
LINE_SEGMENTS         = _data["LINE_SEGMENTS"]
BLIND_EDGES           = _data["BLIND_EDGES"]
OBSTACLES             = _data["OBSTACLES"]

# Per-turn single-wheel pivots: a list of (u, v, w) node-name string tuples
# (the loader has already converted the JSON 3-arrays to tuples). Absent /
# empty means no pivots; the downstream planner/validator/header consume this.
PIVOT_TURNS           = _data.get("pivot_turns", [])

# Field-wide toggle: when True, a BLIND hop whose DESTINATION is a "projection"
# node auto-seeks the next line and re-anchors before stopping (the default).
# Set to False to restore exact dead-reckoning into projection nodes. (Issue #64
# retargeted this from the retired EDGE_PROJECTION edge type to the destination
# node type; the toggle name and value are unchanged.)
BLIND_PROJECTION_AUTO_SEEK = _data.get("blind_projection_auto_seek", True)

# Wall-zero reset nodes: a sparse table keyed by node name, mapping each
# "reset"-typed node to its {axis, wall_coord, facing} re-localization spec.
# Mirrors the PIVOT_TURNS precedent (a top-level sparse table, not a key buried
# inside a node). Absent / empty means no reset nodes; the downstream
# validator/header consume this.
RESET_NODES           = _data.get("RESET_NODES", {})

# ---------------------------------------------------------------------------
# Rich edge list — optional per-edge overrides for the live field graph.
#
# This list is ADDITIVE alongside the geometry loaded from field_data.json
# (LINE_SEGMENTS / BLIND_EDGES). Entries here let an author attach enum-typed
# overrides (primitive_mode) to a specific edge that JSON cannot represent.
# Leaving the list empty is fine — every edge then inherits the pipeline-wide
# PIPELINE_CONFIG defaults at C++ dispatch time. The real field carries no
# overrides today; append a dict per the schema below to attach one.
#
# Single-wheel pivots are NOT authored here — they live in the ``pivot_turns``
# list above as (u, v, w) node-name triples (a pivot is a property of a turn,
# not of one edge).
#
# Authoring a PROJECTION waypoint (issue #64): projection is a NODE property,
# not an edge type. A projection node sits ON a line but NOT at a junction —
# there is no cross marker beneath the strip. To author one, set its type in
# the geometry: ``NODE_TYPES["<name>"] = "projection"`` (and place the node on
# a line in ``NODES`` / ``LINE_SEGMENTS``). Termination of the hop INTO it is
# decided by the incident edge type:
#   * reached by a LINE edge  -> dead-reckon to the encoder distance (no cross
#     to detect, so the chassis stops by distance rather than an intersection);
#   * reached by a BLIND edge -> drive blind, then (when
#     BLIND_PROJECTION_AUTO_SEEK is True, the default) auto-seek the next line
#     and re-anchor before stopping.
# The per-hop primitive falls out of ``(edge_type, destination_node_type)``
# downstream — there is no projection EDGE type and no per-edge projection mode.
#
# Schema: each dict MUST supply:
#   type          : EDGE_LINE | EDGE_BLIND
#   u             : source node name (must exist in NODES)
#   v             : destination node name (must exist in NODES)
#   distance_cm   : Euclidean length in centimetres (float)
#   heading_deg   : departure heading in degrees, CCW from +x, [0, 360)
#
# Optional overrides (absent = inherit the pipeline-wide PIPELINE_CONFIG
# default for that edge's category at C++ dispatch time):
#   primitive_mode : LineMode | BlindMode value
#   direction      : "front" | "rear" — asymmetric per-directed-edge
#                    traversal override; applies only to the forward
#                    (u → v) edge, never mirrored to the reverse. Use
#                    a separate EDGES entry with u/v swapped to override
#                    the reverse direction too.
#
# graph_builder.build_graph() emits a bidirectional directed pair for each
# entry. ``primitive_mode`` is forwarded onto both directions; the
# direction-dependent override (``direction``) lands on the forward edge only.
# These entries are ADDITIVE alongside LINE_SEGMENTS / BLIND_EDGES, not a
# replacement; the planner merges all edge sources into one DiGraph.
# ---------------------------------------------------------------------------
EDGES = []
