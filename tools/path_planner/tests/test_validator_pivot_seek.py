"""TC10..TC12: validator gates for sensor-incoming single-wheel pivots (ADR-044).

Issue #65 tightened the per-turn pivot validator so a SENSOR-terminated
incoming leg -- a BLIND edge that SEEKS a line at runtime (via the legacy
``BlindMode.STOP_AT_LINE`` tag, a per-edge ``stop_at_line`` override of ON, or
the BLIND->PROJECTION auto-rule) -- is treated like a true LINE incoming for the
over-travel correction: it re-anchors on the line and reduces the post-detect
overshoot advance by ``delta``, so it now shares the same FRONT sensor-to-axle
overshoot budget (``Config::LINE_SENSOR_AXLE_OFFSET_CM = 12.02`` cm). The
pre-ADR-044 blanket exemption for sensor legs is gone.

Two complementary properties are pinned here:

* a sensor-incoming pivot WITHIN the budget still validates (TC11) and one
  OVER the budget is rejected from BOTH the auto-rule and the authored-override
  source (TC10) -- the tightening covers every seek trigger; and
* the budget tightening did NOT leak into the chained-pivot double-budget
  sum-check, which stays distance-only: a sensor leg is never entered into the
  shared-edge trim sum (TC12).

The reference geometry (track ``Config::TRACK_MM = 180`` mm = 18.0 cm) the
fixtures rely on:

  theta = 70 deg  -> delta ~= 6.30 cm   (<= 12.02 FRONT budget)
  theta = 90 deg  -> delta = 9.000 cm   (<= 12.02 FRONT budget)
  theta = 120 deg -> delta ~= 15.588 cm (> 12.02 FRONT budget -> REJECT)

All fixtures live in ``tests/fixtures/pivot_field_data.py``; the over-budget
REJECT pair comes from :func:`over_budget_sensor_fields` (deliberately excluded
from that module's ``all_fields()`` legality sweep), and the TC12 chained
construction is built directly via the fixture module's ``_make_field`` builder.
"""

from __future__ import annotations

import pytest

from tools.path_planner.tests.fixtures.pivot_field_data import (
    _make_field,
    make_sensor_authored_pivot_field,
    make_sensor_autorule_pivot_field,
    over_budget_sensor_fields,
)
from tools.path_planner.validator import FieldDataError, validate


# -- TC11: sensor-incoming pivots within the FRONT budget validate -------------
#
# A sensor-incoming pivot whose delta stays within the 12.02 cm FRONT
# sensor-to-axle overshoot budget is a legal field. Both seek-trigger sources
# are exercised: the BLIND->PROJECTION auto-rule (90 deg, delta = 9.0 cm) and an
# authored ``stop_at_line: "on"`` override (70 deg, delta ~= 6.30 cm). These are
# the two passing fixtures the fixture module also feeds into ``all_fields()``.


def test_sensor_autorule_pivot_within_budget_validates() -> None:
    """TC11: the BLIND->PROJECTION auto-rule sensor-incoming pivot validates.

    ``A -> B`` is a plain BLIND hop whose destination ``B`` is a PROJECTION node,
    so with ``BLIND_PROJECTION_AUTO_SEEK`` on (the default) the incoming leg seeks
    a line and classifies ``"sensor"``. The 90 deg pivot's delta = 9.0 cm fits the
    12.02 cm FRONT overshoot budget.

    Post-#54 (ADR-073) the incoming leg is FRONT-pinned so it budgets against that
    FRONT offset -- an unpinned sensor approach on the 2-board default budgets
    against the smaller 7.68 cm REAR offset, which delta = 9.0 cm overflows. The
    ``EDGES`` override is a matching-type overlay on the fixture's ``A->B``
    BLIND edge (the shared fixture stays untouched for T9's reverse-pivot work).
    """
    field = make_sensor_autorule_pivot_field()
    field.EDGES = list(getattr(field, "EDGES", [])) + [
        {"type": "BLIND", "u": "A", "v": "B", "direction": "front"},
    ]
    assert validate(field) is None


def test_sensor_authored_pivot_within_budget_validates() -> None:
    """TC11 (mirror): the authored ``stop_at_line:'on'`` sensor pivot validates.

    ``A -> B`` carries an explicit ``stop_at_line: "on"`` override, forcing the
    seek so the incoming leg classifies ``"sensor"`` independent of the
    auto-rule. The 70 deg pivot's delta ~= 6.30 cm <= the 12.02 cm FRONT budget,
    so ``validate`` returns ``None``. Pairing this with the auto-rule case proves
    a delta<=budget sensor-incoming pivot is legal regardless of which seek
    trigger fires.
    """
    assert validate(make_sensor_authored_pivot_field()) is None


# -- TC10: an over-budget sensor-incoming pivot is rejected from BOTH sources --
#
# ADR-044: a sensor-incoming pivot reduces the post-detect overshoot advance by
# delta exactly like a LINE incoming, so it is bound by the same FRONT
# sensor-to-axle budget (12.02 cm) -- it is no longer exempt. Both over-budget
# variants are an obtuse 120 deg pivot (delta ~= 15.588 cm > 12.02 cm). Running
# the SAME assertion over the auto-rule variant and the authored-override
# variant proves the tightening reaches every seek-trigger source, not just one.

_OVER_BUDGET_REJECT_PATTERN = (
    r"PIVOT_TURNS\[0\].*sensor/LINE overshoot budget.*issue #75"
)


@pytest.mark.parametrize(
    "field",
    over_budget_sensor_fields(),
    ids=["auto_rule_blind_to_projection", "authored_stop_at_line"],
)
def test_over_budget_sensor_pivot_rejected(field) -> None:
    """TC10: a 120 deg sensor-incoming pivot is rejected for BOTH seek sources.

    Each variant's incoming leg classifies ``"sensor"`` (one via the
    BLIND->PROJECTION auto-rule, one via an authored ``stop_at_line: "on"``),
    and each is an obtuse 120 deg pivot whose delta ~= 15.588 cm overflows the
    12.02 cm FRONT overshoot budget. The error names the ``PIVOT_TURNS[0]`` key
    path, the ``sensor/LINE overshoot budget`` it overflows, and the ``issue
    #75`` escape hint. Asserting BOTH variants reject proves the ADR-044
    tightening covers the auto-rule and the authored-override sources alike.
    """
    with pytest.raises(FieldDataError, match=_OVER_BUDGET_REJECT_PATTERN):
        validate(field)


# -- TC12: a departing-seek shared edge is NOT double-budgeted (D3) ------------
#
# The chained-pivot double-budget sum-check (validator step 5) records a per-
# triple delta against a shared directed edge ONLY when that leg is
# DISTANCE-terminated -- a ``"sensor"`` / ``"line"`` leg absorbs its correction
# on the line and has no edge-length budget to share, so it is never entered
# into ``out_edge_deltas`` / ``in_edge_deltas``. ADR-044 added a budget to the
# sensor-incoming SINGLE-side correction but must NOT have leaked into this
# chained sum: the sum stays distance-only.
#
# Construction (a clean shared-sensor-edge chain, verified before finalizing):
#
#       A(0,0) --BLIND,head 90--> B(0,60) --BLIND,head 0--> C(60,60)
#                                                  --BLIND,head 270--> D(60,0)
#
#   Two pivots ``(A, B, C)`` and ``(B, C, D)`` share the directed edge
#   ``B -> C``. Each is a 90 deg right turn (delta = 9.0 cm at track 18.0 cm).
#   The shared edge ``B -> C`` is authored via ``EDGES`` at 15.0 cm. If the
#   chained sum counted both trims it would see 9.0 + 9.0 = 18.0 cm against a
#   15.0 cm edge and REJECT (the dead-reckon control below does exactly that).
#   But ``B -> C`` carries ``stop_at_line: "on"``, so it classifies ``"sensor"``
#   for BOTH the first pivot's departing (outgoing) side and the second pivot's
#   approaching (incoming) side -- neither records into the sum -- so the chain
#   is NOT double-budgeted and the field validates. The flanking legs ``A -> B``
#   and ``C -> D`` are long (60 cm geometric) so no single-side gate trips first,
#   and the sensor-incoming single-side budget on ``B -> C`` (12.02 - 9.0 = 3.02
#   cm >= the 0.5 cm trim floor) is comfortably satisfied.

# Shared four-node geometry: two stacked 90 deg right turns at B and C.
_TC12_NODES = {
    "A": (0.0, 0.0),
    "B": (0.0, 60.0),
    "C": (60.0, 60.0),
    "D": (60.0, 0.0),
}
_TC12_FLANKING_BLIND_EDGES = [("A", "B"), ("C", "D")]
_TC12_PIVOT_TURNS = [("A", "B", "C"), ("B", "C", "D")]
# The shared edge is short enough that summing two 9.0 cm trims (18.0 cm) would
# overflow it -- the load-bearing length for the control vs. sensor contrast.
_TC12_SHARED_EDGE_CM = 15.0


def test_chained_pivot_shared_sensor_edge_not_double_budgeted() -> None:
    """TC12: a sensor-terminated shared edge is excluded from the chained sum.

    The shared edge ``B -> C`` carries ``stop_at_line: "on"`` so it classifies
    ``"sensor"`` for the departing ``(A, B, C)`` pivot and the approaching
    ``(B, C, D)`` pivot alike. A sensor leg never records its delta into the
    chained double-budget sum, so the 18.0 cm of combined trim is NOT charged
    against the 15.0 cm edge and the field validates. The dead-reckon control
    below uses the identical geometry with a plain BLIND shared edge and proves
    the same 15.0 cm length WOULD reject if the sensor leg were summed -- so the
    pass here is the sensor exclusion firing, not a trivially-legal field.
    """
    field = _make_field(
        nodes=_TC12_NODES,
        # All edges are BLIND dead-reckon, so every node is blind; the START node
        # A must be blind (the validator rejects an on-line start, issue #64).
        node_types={name: "blind" for name in _TC12_NODES},
        line_segments=[],
        blind_edges=_TC12_FLANKING_BLIND_EDGES,
        pivot_turns=_TC12_PIVOT_TURNS,
        start_node="A",
        start_heading_deg=90.0,  # facing along A->B
        edges=[
            {
                "type": "BLIND",
                "u": "B",
                "v": "C",
                "distance_cm": _TC12_SHARED_EDGE_CM,
                "heading_deg": 0.0,
                "stop_at_line": "on",  # forces the seek -> sensor-terminated
                # FRONT-pin the shared sensor edge (#54): the second pivot's
                # approach over B->C would otherwise budget its 9.0 cm delta
                # against the 7.68 cm REAR offset and reject before this test's
                # chained-sum exclusion is exercised. Pinning to the 12.02 cm
                # FRONT budget isolates the double-budget behavior under test.
                "direction": "front",
            }
        ],
    )
    assert validate(field) is None


def test_chained_pivot_shared_dead_reckon_edge_double_budgets() -> None:
    """TC12 (control): the same chain with a DEAD-RECKON shared edge rejects.

    Identical four-node geometry and the identical 15.0 cm shared edge, but
    ``B -> C`` is now a plain (non-seeking) BLIND edge, so it classifies
    ``"distance"`` for both pivots and DOES record into the chained sum. The two
    90 deg trims (9.0 + 9.0 = 18.0 cm) overflow the 15.0 cm edge and the
    distance-only chained sum-check rejects with the ``chain on the shared
    dead-reckoned edge`` diagnostic. This is the existing double-budget behavior
    the sensor variant above deliberately escapes -- together they pin that the
    ADR-044 single-side budget did not leak into the chained sum.
    """
    field = _make_field(
        nodes=_TC12_NODES,
        # All BLIND dead-reckon nodes; START A must be blind (issue #64).
        node_types={name: "blind" for name in _TC12_NODES},
        line_segments=[],
        blind_edges=_TC12_FLANKING_BLIND_EDGES,
        pivot_turns=_TC12_PIVOT_TURNS,
        start_node="A",
        start_heading_deg=90.0,
        edges=[
            {
                "type": "BLIND",
                "u": "B",
                "v": "C",
                "distance_cm": _TC12_SHARED_EDGE_CM,
                "heading_deg": 0.0,
            }
        ],
    )
    with pytest.raises(
        FieldDataError, match=r"chain on\s+the shared dead-reckoned edge"
    ):
        validate(field)
