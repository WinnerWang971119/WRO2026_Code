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
  * per-edge ``EDGES`` overrides -- ``speed`` (a ``SpeedTier``) and ``mode``
    (a ``BlindMode``), an asymmetric ``direction`` (rear) override, and a
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

ADR-059 (issue #106/T10-T11) added a collinear BLIND-chain dead-end spur off
``final`` for the blind-chain host FSM test (``test_path_tracker_blind_chain``):
``final -> ChainA -> ChainB -> ChainC -> ChainD``, all BLIND edges, all
collinear at heading 0. It is a pure additive leaf (no edge back into the
existing graph), so it cannot shorten any existing ``NEXT_HOP`` pair. The same
test also uses an ISOLATED LINE->BLIND spanning collinear run (TC7, AC5's
entryPwm=captured-cruise case): ``LpIn -> LpJoin -> LpBlindA -> LpBlindB``, a
LINE pass-through junction (``LpJoin``) feeding directly into a 2-hop BLIND
chain, all collinear at heading 0, isolated at y=130 so it cannot interact with
any existing route.

ADR-059 also added chain-INELIGIBILITY probes (TC6): ``ChainTiny`` (a
sub-``TSHAPE_MIN_MOVE_CM`` hop off ``ChainC``), ``ChainPivPre`` (a collinear
predecessor into the EXISTING ``(Corner, ProjB, Zed)`` pivot), ``ResetPre`` (a
collinear predecessor into the EXISTING ``Wall2`` RESET node) with ``ResetBeyond``
(a leaf goal PAST ``Wall2`` so the walk reaches the RESET break rather than the
goal break), ``SeekPre`` (a collinear predecessor into the forced-seek
``Start->Inter1`` edge so the SEEK break is exercised), and
``DirFlipA``/``DirFlipB``/``DirFlipC`` (a direction-flip probe -- the
``DirFlipB->DirFlipC`` EDGES override forces REAR against the preceding
FRONT-resolved hop). ``ResetBeyond`` and ``SeekPre`` are pure additive leaves
(each reached by exactly one BLIND edge, with no edge OUT of it), so neither can
shorten an existing ``NEXT_HOP`` pair.

So the full stable-name contract is thirty-five names: ``Start``, ``Inter1``,
``ProjL``, ``Inter2``, ``Inter3``, ``Inter4``, ``Corner``, ``Spur``, ``ProjB``,
``Zed`` (the #106 set), the #112 decline-branch nodes ``ProjC`` / ``Cend`` /
``ProjE`` / ``Efar``, the ADR-059 blind-chain nodes ``ChainA`` / ``ChainB`` /
``ChainC`` / ``ChainD`` plus the spanning-run nodes ``LpIn`` / ``LpJoin`` /
``LpBlindA`` / ``LpBlindB`` plus the ineligibility probes ``ChainTiny`` /
``ChainPivPre`` / ``ResetPre`` / ``ResetBeyond`` / ``SeekPre`` / ``DirFlipA`` /
``DirFlipB`` / ``DirFlipC``,
plus the original ``Wall1`` (RESET) and ``final`` (BLIND). Renaming any of
these breaks a test that resolves it by name; everything else here (node
count, the other nodes, coordinates that no test pins) may be freely
edited/extended without touching a single test. (Keep ``Wall1`` at its coords
+ reset record: ``test_reanchor_at_wall`` checks the believed-coordinate
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
from tools.path_planner.pipeline_config import BlindMode, SpeedTier  # noqa: E402

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
    # ADR-059 (issue #106/T10-T11) collinear BLIND chain: a 3-hop dead-end spur off
    # `final`, all BLIND, collinear heading 0 (east). Hop lengths 20/20/8 cm -- the
    # short FINAL hop (8 cm, just over TSHAPE_MIN_MOVE_CM=5 and under the 7 cm
    # shed-to-floor room from cruise 85 -> floor 50 at accel 5 pwm/cm) exercises the
    # ChainSpeedPolicy backward pass pulling the shed into the PRIOR hop (AC4). A pure
    # dead-end branch off an existing BLIND leaf: it cannot shorten any existing
    # NEXT_HOP pair, so it is a purely additive fixture change (see field_fixture.py
    # module docstring's stable-name contract).
    "ChainA":    (120.0, 70.0),   # BLIND (chain hop 0 target: final->ChainA, 25 cm)
    "ChainB":    (140.0, 70.0),   # BLIND (chain mid node: ChainA->ChainB, 20 cm)
    "ChainC":    (160.0, 70.0),   # BLIND (chain mid node: ChainB->ChainC, 20 cm)
    "ChainD":    (168.0, 70.0),   # BLIND (chain END: ChainC->ChainD, 8 cm -- short final hop)
    # ADR-059 LINE->BLIND spanning collinear run (TC7): a SEPARATE, isolated
    # line-junction pass-through feeding directly into a 2-hop BLIND chain, all
    # collinear heading 0. `LpIn->LpJoin` is LINE; the intersection-gated
    # pass-through at `LpJoin` peeks the collinear onward BLIND edge
    # `LpJoin->LpBlindA` and fires (issue #106/T10-T11). Isolated at y=130 (clear
    # of every other node) so it cannot interact with any existing route.
    "LpIn":      (20.0, 130.0),   # LINE (pass-through predecessor)
    "LpJoin":    (45.0, 130.0),   # LINE junction (the pass-through gate fires here)
    "LpBlindA":  (65.0, 130.0),   # BLIND (chain hop 0 after the pass-through, 20 cm)
    "LpBlindB":  (85.0, 130.0),   # BLIND (chain END, 20 cm)
    # ADR-059 chain-ineligibility probes (TC6, issue #106/T10-T11):
    "ChainTiny":   (163.0, 70.0),   # BLIND (off ChainC, 3 cm -- BELOW TSHAPE_MIN_MOVE_CM=5)
    "ChainPivPre": (220.0, 100.0),  # BLIND (collinear predecessor into the EXISTING
                                    #   (Corner, ProjB, Zed) sensor-regime pivot -- the
                                    #   pivot-onto-b lookahead check ends the chain
                                    #   before it can admit Corner->ProjB)
    "ResetPre":    (73.17, 94.948), # BLIND (collinear predecessor into the EXISTING
                                    #   Wall2 RESET node, heading 131.186 deg -- with a
                                    #   goal BEYOND Wall2 (ResetBeyond), the chain walk
                                    #   admits ResetPre->Wall2 then hits the RESET-node
                                    #   check on Wall2 and ends there)
    "ResetBeyond": (30.0, 110.0),   # BLIND leaf goal PAST the Wall2 RESET node. A pure
                                    #   dead-end reached ONLY via Wall2->ResetBeyond, so
                                    #   `goTo(ResetBeyond)` from ResetPre makes Wall2 an
                                    #   INTERIOR walk node (not the goal) -- the walk then
                                    #   reaches the RESET-node break (PathTracker.cpp:2287)
                                    #   instead of the goal break. Additive leaf: no edge
                                    #   OUT of it, so it cannot shorten any existing
                                    #   NEXT_HOP pair (stable-name contract).
    "SeekPre":     (6.0, 16.0),     # BLIND collinear predecessor into the EXISTING
                                    #   forced-seek `Start->Inter1` edge (stop_at_line=on).
                                    #   Exactly collinear with Start->Inter1 (Start-SeekPre
                                    #   == 0.5*(Inter1-Start)), so `goTo(Inter1)` from
                                    #   SeekPre admits SeekPre->Start then reaches the SEEK
                                    #   break (PathTracker.cpp:2350) when Start->Inter1
                                    #   resolves to a stopAtLine seek. Additive leaf into
                                    #   Start (no edge OUT of SeekPre): cannot shorten any
                                    #   existing NEXT_HOP pair (stable-name contract).
    # Direction-flip probe (TC6): DirFlipA->DirFlipB resolves FRONT (AUTO); the
    # EDGES override below forces DirFlipB->DirFlipC REAR -- collinear heading 0
    # but a direction mismatch, so the chain walk ends at DirFlipB. Isolated at
    # y=115 (clear of every other node).
    "DirFlipA":  (20.0, 115.0),
    "DirFlipB":  (40.0, 115.0),
    "DirFlipC":  (60.0, 115.0),
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
    "ChainA": "blind", "ChainB": "blind", "ChainC": "blind", "ChainD": "blind",
    "LpIn": "line", "LpJoin": "line", "LpBlindA": "blind", "LpBlindB": "blind",
    "ChainTiny": "blind", "ChainPivPre": "blind", "ResetPre": "blind",
    "ResetBeyond": "blind", "SeekPre": "blind",
    "DirFlipA": "blind", "DirFlipB": "blind", "DirFlipC": "blind",
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
    # ADR-059 LINE->BLIND spanning collinear run (TC7): LpIn->LpJoin is the LINE
    # leg into the pass-through junction LpJoin (see _NODES comment above).
    ("LpIn", "LpJoin"),
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
    # ADR-059 collinear BLIND chain (issue #106/T10-T11) -- a dead-end spur off
    # `final`, all four hops collinear at heading 0 (see _NODES comment above).
    ("final", "ChainA"),
    ("ChainA", "ChainB"),
    ("ChainB", "ChainC"),
    ("ChainC", "ChainD"),
    # ADR-059 LINE->BLIND spanning collinear run (TC7): LpJoin's pass-through peek
    # continues onto this collinear BLIND leg, which then chains a further hop.
    ("LpJoin", "LpBlindA"),
    ("LpBlindA", "LpBlindB"),
    # ADR-059 chain-ineligibility probes (TC6, issue #106/T10-T11) -- see _NODES
    # comment above for the geometry rationale of each.
    ("ChainC", "ChainTiny"),          # <TSHAPE_MIN_MOVE_CM hop -- chain-ineligible
    ("ChainPivPre", "Corner"),        # collinear predecessor into the existing pivot
    ("ResetPre", "Wall2"),            # collinear predecessor into the existing RESET node
    ("Wall2", "ResetBeyond"),         # onward leaf PAST Wall2 -- makes Wall2 an interior
                                      #   walk node so the RESET break is reached (see _NODES)
    ("SeekPre", "Start"),             # collinear predecessor into the forced-seek Start->Inter1
    ("DirFlipA", "DirFlipB"), ("DirFlipB", "DirFlipC"),  # direction-flip probe (see _EDGES)
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
    _edge("Inter2", "Inter3", EDGE_LINE, speed=SpeedTier.FAST),               # SpeedTier override
    _edge("final", "position1", EDGE_BLIND, mode=BlindMode.GYRO_TSHAPE),      # BlindMode override
    _edge("Inter1", "Spur", EDGE_LINE, direction="rear"),                     # asymmetric direction override
    _edge("Start", "Inter1", EDGE_BLIND, stop_at_line="on"),                  # forced seek override
    # ADR-059 chain-ineligibility probe (TC6, issue #106/T10-T11): forces
    # DirFlipB->DirFlipC REAR while the preceding hop resolves FRONT (AUTO on a
    # 1-board default chassis), so the direction-flip guard in the blind-chain
    # walk-ahead ends the chain BEFORE admitting this hop. A dedicated, isolated
    # pair (NOT reusing ChainC->ChainD) so the override cannot perturb the main
    # chain's TC4/TC8/TC9 dispatch.
    _edge("DirFlipB", "DirFlipC", EDGE_BLIND, direction="rear"),
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
