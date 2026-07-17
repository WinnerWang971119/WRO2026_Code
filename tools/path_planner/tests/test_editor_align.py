"""Tests for the pure "Make straight" alignment geometry module.

Covers TC1-TC10 from the plan's Testing Strategy
(docs/plans/2026-07-17-161-editor-multiselect-make-straight.md). TC12
(``find_nodes_in_rect`` with an inverted box) belongs to the canvas, not
``align.py``, and is deliberately not tested here -- it lives in
``test_editor_marquee_blit.py`` alongside TC11.

``align.py`` imports neither Tk nor matplotlib, so this module needs none of
the Agg-backend import dance the GUI-adjacent test files require -- see
``test_editor_select_mode.py`` for that pattern when it is needed.

Fixtures use two real columns from ``tools/path_planner/field_data.json``
(the x~50.5 column: M/MB2/MG2/MW2/MY2/i1, and the x~117.7 column: CY3/bowlO/
dump1/dump2/dump3/i3) plus small synthetic fixtures for cases the real field
doesn't exercise (an exact spread tie, a diagonal FIT run with an outlier,
a Y-axis alignment). Per gotcha-axis-aligned-fixtures-hide-subtolerance-turns,
the real-column fixtures are deliberately NOT perfectly axis-aligned -- their
sub-tolerance residuals are the actual defect this feature targets.
"""

from __future__ import annotations

import math

import pytest

from tools.path_planner.editor.align import (
    AlignAxis,
    AlignPreview,
    align_points,
    build_preview,
    max_residual_cm,
    resolve_axis,
)

# ---------------------------------------------------------------------------
# Real field_data.json fixtures (tools/path_planner/field_data.json NODES)
# ---------------------------------------------------------------------------

# The x~50.5 column. Five nodes share the full-precision x
# 50.484010371650825; MY2 sits at the clean x=50.5. Raw mean x is
# 50.48667530970902, which rounds to 50.4867 (AC5's pinned value).
COLUMN_X50 = {
    "M": (50.484010371650825, 69.4522369016751),
    "MB2": (50.484010371650825, 60.95324525661604),
    "MG2": (50.484010371650825, 77.46557188130221),
    "MW2": (50.484010371650825, 93.24941350784047),
    "MY2": (50.5, 45.9),
    "i1": (50.484010371650825, 27.328592007243696),
}

# The x~117.7 column. Raw mean x is 117.71861203715137; the max residual to
# that raw mean (not the rounded target) is ~0.0288465 cm (TC9).
COLUMN_X117 = {
    "CY3": (117.7, 36.5),
    "bowlO": (117.74745853397538, 22.829197020208255),
    "dump1": (117.7, 58.3),
    "dump2": (117.74745853397538, 53.6110219368646),
    "dump3": (117.71675515495741, 49.197283615261114),
    "i3": (117.7, 27.3),
}


def _distance(p0: tuple[float, float], p1: tuple[float, float]) -> float:
    """Euclidean distance between two (x, y) points, in centimetres."""
    return math.hypot(p1[0] - p0[0], p1[1] - p0[1])


def _point_to_line_distance(
    point: tuple[float, float],
    line_a: tuple[float, float],
    line_b: tuple[float, float],
) -> float:
    """Perpendicular distance from *point* to the infinite line through *line_a*/*line_b*."""
    x0, y0 = point
    x1, y1 = line_a
    x2, y2 = line_b
    numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denominator = math.hypot(y2 - y1, x2 - x1)
    return numerator / denominator


# ---------------------------------------------------------------------------
# TC1 -- resolve_axis on the real x~50.5 column, AUTO
# ---------------------------------------------------------------------------


def test_resolve_axis_real_column_auto_picks_x() -> None:
    xs = [xy[0] for xy in COLUMN_X50.values()]
    ys = [xy[1] for xy in COLUMN_X50.values()]
    spread_x = max(xs) - min(xs)
    spread_y = max(ys) - min(ys)
    assert spread_x == pytest.approx(0.016, abs=1e-3)
    assert spread_y == pytest.approx(65.92, abs=1e-2)
    assert spread_x <= spread_y

    assert resolve_axis(COLUMN_X50, AlignAxis.AUTO) is AlignAxis.X


# ---------------------------------------------------------------------------
# TC2 -- resolve_axis tie-break
# ---------------------------------------------------------------------------


def test_resolve_axis_exact_tie_resolves_to_x() -> None:
    points = {"A": (0.0, 0.0), "B": (5.0, 5.0)}
    xs = [xy[0] for xy in points.values()]
    ys = [xy[1] for xy in points.values()]
    assert (max(xs) - min(xs)) == (max(ys) - min(ys))  # exact tie by construction

    assert resolve_axis(points, AlignAxis.AUTO) is AlignAxis.X


def test_resolve_axis_passes_through_non_auto() -> None:
    points = {"A": (0.0, 0.0), "B": (5.0, 5.0)}
    assert resolve_axis(points, AlignAxis.X) is AlignAxis.X
    assert resolve_axis(points, AlignAxis.Y) is AlignAxis.Y
    assert resolve_axis(points, AlignAxis.FIT) is AlignAxis.FIT


# ---------------------------------------------------------------------------
# TC3 -- align_points on the real column, AUTO: x exact, y byte-identical
# ---------------------------------------------------------------------------


def test_align_points_real_column_auto_rounds_only_x() -> None:
    result = align_points(COLUMN_X50, AlignAxis.AUTO)

    assert set(result.keys()) == set(COLUMN_X50.keys())
    for name, (x, y) in result.items():
        assert x == 50.4867
        # The untouched axis must be byte-identical to the full-precision input.
        assert y == COLUMN_X50[name][1]


# ---------------------------------------------------------------------------
# TC4 -- X-alignment: 4dp-clean shared-x is a no-op; full-precision shared-x
# grid-snaps and is NOT a no-op.
# ---------------------------------------------------------------------------


def test_align_points_x_4dp_clean_shared_x_is_noop() -> None:
    points = {"A": (50.0, 1.0), "B": (50.0, 2.0), "C": (50.0, 3.0)}

    result = align_points(points, AlignAxis.X)

    assert result == points


def test_align_points_x_full_precision_shared_x_grid_snaps_not_noop() -> None:
    # All three nodes share the exact full-precision x from the real
    # x~50.5 column (50.484010371650825); the mean of three identical
    # values is itself, so the rounded target is 50.484, distinct from the
    # unrounded 50.484010371650825 input -- a grid-snap, not a no-op.
    points = {
        "M": (50.484010371650825, 69.4522369016751),
        "MB2": (50.484010371650825, 60.95324525661604),
        "MG2": (50.484010371650825, 77.46557188130221),
    }

    result = align_points(points, AlignAxis.X)

    assert result != points
    for name, (x, y) in result.items():
        assert x == 50.484
        assert x != points[name][0]
        assert y == points[name][1]  # y untouched


# ---------------------------------------------------------------------------
# TC5 -- FIT on exactly 2 4dp-clean nodes is a no-op
# ---------------------------------------------------------------------------


def test_align_points_fit_two_nodes_is_noop() -> None:
    points = {"A": (0.0, 0.0), "B": (10.0, 10.0)}

    result = align_points(points, AlignAxis.FIT)

    assert result == points


# ---------------------------------------------------------------------------
# TC6 -- FIT on a diagonal run with one outlier: TLS line through centroid
# ---------------------------------------------------------------------------


# A diagonal run mostly on y=x, plus one outlier (D) well off the line.
DIAGONAL_WITH_OUTLIER = {
    "A": (0.0, 0.0),
    "B": (10.0, 10.0),
    "C": (20.0, 20.0),
    "D": (10.0, 14.0),
}


def test_align_points_fit_diagonal_outlier_projects_and_passes_centroid() -> None:
    result = align_points(DIAGONAL_WITH_OUTLIER, AlignAxis.FIT)

    assert set(result.keys()) == set(DIAGONAL_WITH_OUTLIER.keys())

    # The raw (unrounded) centroid of the input selection.
    n = len(DIAGONAL_WITH_OUTLIER)
    centroid = (
        sum(x for x, _ in DIAGONAL_WITH_OUTLIER.values()) / n,
        sum(y for _, y in DIAGONAL_WITH_OUTLIER.values()) / n,
    )

    # The fitted line passes through the centroid, so the centroid must lie
    # (within rounding tolerance) on the line formed by any two of the
    # returned (rounded) targets.
    line_a, line_b = result["A"], result["D"]
    assert _point_to_line_distance(centroid, line_a, line_b) < 1e-3

    # The outlier D was genuinely moved off its input position by the fit.
    assert _distance(result["D"], DIAGONAL_WITH_OUTLIER["D"]) > 1.0

    # Every returned target is collinear with every other (all lie on the
    # same fitted line, independently rounded).
    names = list(result.keys())
    p0, p1 = result[names[0]], result[names[1]]
    for name in names[2:]:
        assert _point_to_line_distance(result[name], p0, p1) < 1e-3


# ---------------------------------------------------------------------------
# TC7 -- rounding: every returned coord equals round(v, 4); X/Y share one
# identical scalar on the aligned axis.
# ---------------------------------------------------------------------------


def test_align_points_x_rounds_to_4dp_and_shares_one_scalar() -> None:
    result = align_points(COLUMN_X50, AlignAxis.X)

    xs = {round(x, 4) for x, _ in result.values()}
    assert xs == {50.4867}
    for x, _ in result.values():
        assert x == round(x, 4)


def test_align_points_y_rounds_to_4dp_and_shares_one_scalar() -> None:
    # A synthetic "row": distinct x, noisy y -- exercises the Y branch,
    # which the two real columns (both X-resolved) do not.
    row = {"P": (1.0, 50.0), "Q": (2.0, 50.1), "R": (3.0, 49.95)}

    result = align_points(row, AlignAxis.Y)

    ys = {round(y, 4) for _, y in result.values()}
    assert ys == {50.0167}
    for name, (x, y) in result.items():
        assert y == round(y, 4)
        assert x == row[name][0]  # x untouched, byte-identical


def test_align_points_fit_rounds_both_coords_to_4dp() -> None:
    result = align_points(DIAGONAL_WITH_OUTLIER, AlignAxis.FIT)

    for x, y in result.values():
        assert x == round(x, 4)
        assert y == round(y, 4)


# ---------------------------------------------------------------------------
# TC8 -- fewer than 2 points raises ValueError
# ---------------------------------------------------------------------------


def test_align_points_one_node_raises_value_error() -> None:
    with pytest.raises(ValueError):
        align_points({"A": (1.0, 1.0)}, AlignAxis.X)


def test_align_points_zero_nodes_raises_value_error() -> None:
    with pytest.raises(ValueError):
        align_points({}, AlignAxis.X)


def test_max_residual_cm_one_node_raises_value_error() -> None:
    with pytest.raises(ValueError):
        max_residual_cm({"A": (1.0, 1.0)}, AlignAxis.X)


def test_max_residual_cm_zero_nodes_raises_value_error() -> None:
    with pytest.raises(ValueError):
        max_residual_cm({}, AlignAxis.X)


# ---------------------------------------------------------------------------
# TC9 -- max_residual_cm on the real x~117.7 column, AUTO, vs the RAW mean
# ---------------------------------------------------------------------------


def test_max_residual_cm_real_column_auto_matches_raw_mean() -> None:
    # Sanity: AUTO must resolve to X on this column too (small x spread,
    # large y spread), otherwise this test would be pinning the wrong axis.
    assert resolve_axis(COLUMN_X117, AlignAxis.AUTO) is AlignAxis.X

    residual = max_residual_cm(COLUMN_X117, AlignAxis.AUTO)

    assert residual == pytest.approx(0.0288465, abs=1e-6)

    # The residual must be measured against the RAW (unrounded) mean, not
    # the rounded target -- the two differ by ~1.2e-5 cm on this data.
    raw_mean_x = sum(x for x, _ in COLUMN_X117.values()) / len(COLUMN_X117)
    rounded_target_x = round(raw_mean_x, 4)
    assert raw_mean_x != rounded_target_x
    expected_raw = max(abs(x - raw_mean_x) for x, _ in COLUMN_X117.values())
    expected_rounded = max(abs(x - rounded_target_x) for x, _ in COLUMN_X117.values())
    assert residual == pytest.approx(expected_raw, abs=1e-12)
    assert residual != pytest.approx(expected_rounded, abs=1e-6)


def test_max_residual_cm_x_axis_direct() -> None:
    residual = max_residual_cm(COLUMN_X50, AlignAxis.X)

    raw_mean_x = sum(x for x, _ in COLUMN_X50.values()) / len(COLUMN_X50)
    expected = max(abs(x - raw_mean_x) for x, _ in COLUMN_X50.values())
    assert residual == pytest.approx(expected, abs=1e-12)


def test_max_residual_cm_fit_diagonal_outlier() -> None:
    residual = max_residual_cm(DIAGONAL_WITH_OUTLIER, AlignAxis.FIT)

    # The outlier D is the point farthest from the fitted line, so it must
    # drive the reported max residual.
    assert residual > 2.0


# ---------------------------------------------------------------------------
# TC10 -- build_preview on an already-collinear selection: ghosts == {}
# ---------------------------------------------------------------------------


def test_build_preview_already_collinear_has_no_ghosts() -> None:
    # 4dp-clean, already sharing one x -- align_points is a no-op (TC4),
    # so nothing should be flagged as a mover.
    points = {"A": (50.0, 1.0), "B": (50.0, 2.0), "C": (50.0, 3.0)}

    preview = build_preview(points, AlignAxis.X)

    assert preview is not None
    assert preview.ghosts == {}


def test_build_preview_already_collinear_fit_has_no_ghosts() -> None:
    # Exactly on the y=x line already -- the TLS fit through the centroid
    # reproduces each input point, so FIT should also report no movers.
    points = {"A": (0.0, 0.0), "B": (10.0, 10.0), "C": (20.0, 20.0)}

    preview = build_preview(points, AlignAxis.FIT)

    assert preview is not None
    assert preview.ghosts == {}


def test_build_preview_returns_none_below_two_nodes() -> None:
    assert build_preview({}, AlignAxis.X) is None
    assert build_preview({"A": (1.0, 1.0)}, AlignAxis.X) is None


def test_build_preview_real_column_has_ghosts_and_guide() -> None:
    preview = build_preview(COLUMN_X50, AlignAxis.AUTO)

    assert isinstance(preview, AlignPreview)
    # Every node moves under mean-snap (Decisions: "essentially every node
    # is a mover"), so all six should appear as ghosts.
    assert set(preview.ghosts.keys()) == set(COLUMN_X50.keys())
    for name, target in preview.ghosts.items():
        assert target[0] == 50.4867
        assert target[1] == COLUMN_X50[name][1]

    # Guide line spans the selection's y-extent at the aligned x.
    (x0, y0), (x1, y1) = preview.guide
    assert x0 == x1 == 50.4867
    ys = [xy[1] for xy in COLUMN_X50.values()]
    assert {y0, y1} == {min(ys), max(ys)}
