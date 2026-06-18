"""Stable toy field-data fixture for the planner / compiler / sim tests.

This module mirrors the public attribute surface of
``tools.path_planner.field_data`` but loads its values from
``field_data_toy.json`` next to this file -- a frozen copy of the
original placeholder field (nodes A/B/C/D/E/G/P1) the test suite was
authored against.

Why this exists
---------------

``tools/path_planner/field_data.json`` is the *live* file the field
editor mutates. The test suite, however, asserts against specific node
names ("A", "B", "G", ...) and exact coordinates baked into the original
toy layout. Importing the live module into tests therefore makes the
entire planner test suite a regression bomb -- every editor save would
flip one or more tests red.

Tests that exercise the toy graph's *shape* import this module instead
of ``field_data``. Tests that exercise the live module itself
(``test_build_routes`` runs the CLI; ``test_header_writer`` re-emits
``routes.h``) continue to use ``tools.path_planner.field_data``.
"""

from __future__ import annotations

from pathlib import Path

from tools.path_planner.editor.loader import load_field_json

_FIXTURE_PATH: Path = Path(__file__).resolve().parent / "field_data_toy.json"
_data = load_field_json(_FIXTURE_PATH)

FIELD_MM = _data["FIELD_MM"]
MAP_PNG = _data["MAP_PNG"]
ROBOT_RADIUS_CM = _data["ROBOT_RADIUS_CM"]
BLIND_DECAY_ENABLED = _data["BLIND_DECAY_ENABLED"]
BLIND_DECAY_K = _data["BLIND_DECAY_K"]
BLIND_DECAY_EXPONENT = _data["BLIND_DECAY_EXPONENT"]
BLIND_DECAY_D_REF = _data["BLIND_DECAY_D_REF"]
START_NODE = _data["START_NODE"]
START_HEADING_DEG = _data["START_HEADING_DEG"]
NODES = _data["NODES"]
NODE_TYPES = _data["NODE_TYPES"]
LINE_SEGMENTS = _data["LINE_SEGMENTS"]
BLIND_EDGES = _data["BLIND_EDGES"]
OBSTACLES = _data["OBSTACLES"]
BLIND_PROJECTION_AUTO_SEEK = _data.get("blind_projection_auto_seek", True)

# Optional sparse tables (this toy field authors neither). Mirror field_data.py
# so the strict header_writer read (`field_data.RESET_NODES`) never AttributeErrors.
PIVOT_TURNS = _data.get("pivot_turns", [])
RESET_NODES = _data.get("RESET_NODES", {})

EDGES: list = []
