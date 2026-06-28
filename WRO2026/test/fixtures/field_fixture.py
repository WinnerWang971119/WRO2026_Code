#!/usr/bin/env python3
"""Single-sourced host-test fixture field (``routes_fixture``).

This module is the ONE definition of the host-test fixture field. It is consumed
by BOTH:

  * ``regenerate_routes_fixture.py`` (T3) -- which runs it through the real emit
    pipeline to produce the committed ``routes_fixture.{h,cpp}`` the C++ host FSM
    tests link; and
  * the PathTracker sim<->firmware parity pytest (T8) -- which imports
    :func:`build_field_data` to build the sim graph over the exact same field the
    firmware FSM is driven across, so the two sides cannot diverge on the map.

Previously this field lived inline in ``regenerate_routes_fixture.py``; it was
factored out here so a single import (from either tree) yields the identical
``field_data``-shaped namespace.

Issue #91: the firmware FSM-logic host tests (``reanchor_at_wall[_strict]``,
``path_tracker_reset_arrival``, ``return_to_node_dispatch``, and the ``ctps_dump``
self-test) assert PathTracker / Localization *logic* that is independent of which
field is loaded. They used to link the production ``WRO2026/routes.cpp``, so an
unrelated map edit silently broke them -- and once the production field collapses
to a single ``Start`` spawn node (projection refactor, #64) they lose the
LINE / BLIND / RESET nodes they resolve by name entirely.

The field below is a deliberately FEATURE-COMPLETE fixture graph those tests link
instead. It exercises every path-planner feature the codebase supports, so the
emitted ``routes_fixture.cpp`` is a hard, realistic stress map (not a 7-node toy):

  * all four node types -- LINE (junctions), BLIND (off-line dead-reckon),
    RESET (wall re-anchor, x-axis AND y-axis walls), PROJECTION (on-line waypoint);
  * both edge types -- LINE (line-follow) and BLIND (dead-reckon);
  * a collinear ``line-line-line`` triple (intersection-gated pass-through) and a
    collinear ``line-projection-line`` triple (distance-gated pass-through);
  * a LINE->projection hop (distance-terminated) and a BLIND->projection hop
    (auto-seek + re-anchor, ``BLIND_PROJECTION_AUTO_SEEK``);
  * two LINE->projection hops whose OUTBOUND flank makes the line-projection
    pass-through gate DECLINE -- a BLIND flank (``Inter2->ProjC->Cend``, condition 3)
    and a non-collinear LINE flank (``Inter3->ProjE->Efar``, condition 5) -- so both
    gate-decline branches are reachable by the real FSM (#112);
  * two single-wheel pivot turns -- one ``line`` regime (LINE legs) and one
    ``sensor`` regime (BLIND auto-seek into a projection node);
  * per-edge ``EDGES`` overrides -- ``primitive_mode`` (a ``LineMode`` and a
    ``BlindMode``), an asymmetric ``direction`` (rear) override, and a
    ``stop_at_line`` seek override;
  * blind-decay routing (the ``BLIND_DECAY_*`` knobs, with blind edges of varying
    length).

Stable-name contract (issue #91): NO test depends on this field's SHAPE. The host
tests resolve only the nodes they need BY NAME, with zero assumptions about node
indices, the total count, or which other nodes exist. The original four anchors
were ``Wall1`` (RESET), ``Inter1`` (LINE), ``Start`` (BLIND spawn), and ``final``
(BLIND), used by ``reanchor_at_wall[_strict]``, ``path_tracker_reset_arrival``,
``return_to_node_dispatch``, and the ``ctps_dump`` self-test.

Issue #106 added five PathTracker mirror tests that host-link the real
``PathTracker.cpp`` + ``Drive.cpp`` and consolidate onto this fixture, resolving
ten nodes BY NAME:

  * ``passthrough`` -- ``Inter2``, ``Inter3``, ``Inter4`` (line-line-line);
  * ``pass_through_projection`` -- ``Inter1``, ``ProjL``, ``Inter2`` (line-proj-line),
    plus ``ProjC`` / ``Cend`` (BLIND-flank decline) and ``ProjE`` / ``Efar``
    (non-collinear-LINE-flank decline), #112;
  * ``projection`` -- ``Inter1``, ``ProjL`` (LINE->proj), ``Corner``, ``ProjB``
    (BLIND->proj auto-seek);
  * ``stop_at_line`` -- ``Start``, ``Inter1`` (BLIND seek), ``Corner``, ``ProjB``,
    ``Zed`` (the sensor-regime pivot);
  * ``bidirectional`` -- ``Inter1``, ``Spur`` (rear-override LINE).

So the full stable-name contract is sixteen names: ``Start``, ``Inter1``,
``ProjL``, ``Inter2``, ``Inter3``, ``Inter4``, ``Corner``, ``Spur``, ``ProjB``,
``Zed`` (the #106 set), the #112 decline-branch nodes ``ProjC`` / ``Cend`` /
``ProjE`` / ``Efar``, plus the original ``Wall1`` (RESET) and ``final``
(BLIND). Renaming any of these breaks a test that resolves it by name; everything
else here (node count, the other nodes, coordinates that no test pins) may be
freely edited/extended without touching a single test. (Keep ``Wall1`` at its
coords + reset record: ``test_reanchor_at_wall`` checks the believed-coordinate
post-condition against ``wall_coord 154`` over node y ``139``.)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

_THIS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _THIS_DIR.parents[2]  # fixtures -> test -> WRO2026 -> repo root
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE  # noqa: E402
from tools.path_planner.pipeline_config import BlindMode, LineMode  # noqa: E402

# ---------------------------------------------------------------------------
# Node geometry. Field is 320 cm x 154 cm (FIELD_MM below). Origin SW; +x East,
# +y North; headings CCW from +x. The main line runs along y=40; a vertical line
# rises at x=260 (Inter4->Corner) and a short spur at x=60 (Inter1->Spur).
# ---------------------------------------------------------------------------
_NODES = {
    "Start":     (24.0, 24.0),    # BLIND spawn (START_NODE)
    "Inter1":    (60.0, 40.0),    # LINE  (test anchor)
    "ProjL":     (100.0, 40.0),   # PROJECTION on the main line (LINE-reached)
    "Inter2":    (140.0, 40.0),   # LINE
    "Inter3":    (200.0, 40.0),   # LINE
    "Inter4":    (260.0, 40.0),   # LINE
    "Corner":    (260.0, 100.0),  # LINE  (top of the x=260 vertical)
    "Spur":      (60.0, 80.0),    # LINE  (top of the x=60 spur; rear-override edge)
    "final":     (95.0, 70.0),    # BLIND (test anchor)
    "position1": (200.0, 110.0),  # BLIND
    "Wall1":     (165.0, 139.0),  # RESET, y-wall  (test anchor -- keep coords)
    "Wall2":     (60.0, 110.0),   # RESET, x-wall
    "ProjB":     (300.0, 100.0),  # PROJECTION reached by a BLIND edge (auto-seek)
    "Zed":       (300.0, 40.0),   # BLIND (outgoing leg of the sensor-regime pivot)
    # LINE-reached projection nodes whose OUTBOUND flank makes the line-projection
    # pass-through gate DECLINE -- the two branches no triple above exercises (#112):
    "ProjC":     (140.0, 10.0),   # PROJECTION (LINE-reached) with a BLIND outbound flank (cond-3)
    "Cend":      (180.0, 10.0),   # BLIND leaf goal past ProjC (forces proj != goal, isolates cond-3)
    "ProjE":     (200.0, 10.0),   # PROJECTION (LINE-reached) with a non-collinear LINE flank (cond-5)
    "Efar":      (240.0, 10.0),   # LINE leaf goal past ProjE (cond-4 passes; isolates cond-5)
}

_NODE_TYPES = {
    "Start": "blind",
    "Inter1": "line", "ProjL": "projection", "Inter2": "line",
    "Inter3": "line", "Inter4": "line", "Corner": "line", "Spur": "line",
    "final": "blind", "position1": "blind",
    "Wall1": "reset", "Wall2": "reset",
    "ProjB": "projection", "Zed": "blind",
    "ProjC": "projection", "Cend": "blind",
    "ProjE": "projection", "Efar": "line",
}

# LINE edges (line-follow). Never incident to a blind/reset node. The main line
# Inter1-ProjL-Inter2-Inter3-Inter4 is collinear (y=40), giving both pass-through
# triples; Inter4-Corner + Inter1-Spur add the turn + rear-override geometry.
_LINE_SEGMENTS = [
    ("Inter1", "ProjL"), ("ProjL", "Inter2"),   # line-proj-line (Inter1->ProjL->Inter2)
    ("Inter2", "Inter3"), ("Inter3", "Inter4"),  # line-line-line (Inter2->Inter3->Inter4)
    ("Inter4", "Corner"),                          # vertical (pivot outgoing leg)
    ("Inter1", "Spur"),                            # spur (rear-override edge)
    # LINE hops INTO projection nodes whose outbound flank declines pass-through (#112).
    # Both depart their main-line junction heading 270 (straight down, off the y=40 line)
    # so a test can seat the chassis facing 270 and skip the pre-turn.
    ("Inter2", "ProjC"),                           # LINE -> projection; ProjC's flank is BLIND (cond-3)
    ("Inter3", "ProjE"), ("ProjE", "Efar"),        # LINE -> projection -> LINE, 90deg bend (cond-5)
]

# BLIND edges (dead-reckon). Free to touch any node type.
_BLIND_EDGES = [
    ("Start", "Inter1"),       # spawn into the line graph
    ("final", "Inter2"),       # off-line waypoint onto the line
    ("final", "position1"),
    ("final", "Wall2"),        # reach the x-wall reset node
    ("position1", "Inter3"),
    ("position1", "Wall1"),    # reach the y-wall reset node
    ("Corner", "ProjB"),       # BLIND -> projection (auto-seek)
    ("ProjB", "Zed"),          # projection -> blind (sensor-pivot outgoing leg)
    ("ProjC", "Cend"),         # projection -> blind: the non-LINE outbound flank (cond-3 decline)
]

# Single-wheel pivots (u, v, w). One LINE-regime turn and one SENSOR-regime turn.
_PIVOT_TURNS = [
    ("Inter3", "Inter4", "Corner"),  # LINE->LINE +90deg (line regime, both legs)
    ("Corner", "ProjB", "Zed"),      # BLIND->projection seek then BLIND -90deg (sensor x distance)
]

# Wall-zero reset nodes. Wall1 presses a y-wall (north); Wall2 presses an x-wall
# (west). Each facing points AWAY from its wall so the rear can press it.
_RESET_NODES = {
    "Wall1": {"axis": "y", "wall_coord": 154.0, "facing": 270.0},
    "Wall2": {"axis": "x", "wall_coord": 25.0, "facing": 0.0},
}


def _edge(u: str, v: str, etype, **overrides) -> dict:
    """Build a rich-EDGES dict with geometry computed from the node coordinates.

    ``distance_cm`` / ``heading_deg`` are derived from ``_NODES`` so the override
    edge stays collinear/consistent with the geometry the host headers expect;
    ``overrides`` carries the enum/string override under test.
    """
    (ux, uy), (vx, vy) = _NODES[u], _NODES[v]
    dist = math.hypot(vx - ux, vy - uy)
    head = math.degrees(math.atan2(vy - uy, vx - ux)) % 360.0
    cell = {"type": etype, "u": u, "v": v, "distance_cm": dist, "heading_deg": head}
    cell.update(overrides)
    return cell


# Per-edge overrides (ADR-022): authored here in Python because they carry enum
# values JSON cannot represent. Additive over LINE_SEGMENTS / BLIND_EDGES (same
# type, so no conflict). None of these edges is a pivot leg (rear is rejected on
# pivot legs; pivots are FRONT-only in v1).
_EDGES = [
    _edge("Inter2", "Inter3", EDGE_LINE, primitive_mode=LineMode.FAST),       # LineMode override
    _edge("final", "position1", EDGE_BLIND, primitive_mode=BlindMode.GYRO_TSHAPE),  # BlindMode override
    _edge("Inter1", "Spur", EDGE_LINE, direction="rear"),                     # asymmetric direction override
    _edge("Start", "Inter1", EDGE_BLIND, stop_at_line="on"),                  # forced seek override
]


def build_field_data() -> SimpleNamespace:
    """The feature-complete host-test fixture field (see module docstring).

    Returns a ``field_data``-shaped :class:`~types.SimpleNamespace` carrying every
    attribute the emit pipeline (``validate`` / ``build_graph`` / ``emit_routes``)
    and the parity sim read. The single source of truth for the fixture map.
    """
    return SimpleNamespace(
        NODES=_NODES,
        NODE_TYPES=_NODE_TYPES,
        LINE_SEGMENTS=_LINE_SEGMENTS,
        BLIND_EDGES=_BLIND_EDGES,
        OBSTACLES=[],
        ROBOT_RADIUS_CM=15.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE="Start",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3200, 1540),
        MAP_PNG="maps/map.png",
        BLIND_PROJECTION_AUTO_SEEK=True,
        RESET_NODES=_RESET_NODES,
        PIVOT_TURNS=_PIVOT_TURNS,
        EDGES=_EDGES,
    )
