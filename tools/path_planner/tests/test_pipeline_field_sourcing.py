"""Cross-surface ``PIPELINE_*`` agreement tests (issue #148, ADR-064).

Issue #148's clause (d) asks for ONE field that authors all four ``PIPELINE_*``
categories (speed / blind / turn / direction) to non-default values and proves
every surface that resolves a pipeline default -- ``validate()``, ``build_graph()``,
``compile_path()``, ``final_heading_after()``, and the emitted ``routes.cpp`` --
agrees on what that field means. :func:`test_tc3_all_four_pipeline_categories_agree_across_surfaces`
is that test.

The audit behind this file found exactly three field-sourcing gaps:
``validator._regime`` (fixed by T1, pinned here by TC5/TC6/TC7/TC8),
``compiler.final_heading_after`` (fixed by T2), and the ``build_routes.py``
diagnostic (fixed by T3, smoke-tested manually by T6). The remaining reads of
the module-level :data:`~tools.path_planner.pipeline_config.PIPELINE_CONFIG`
singleton are documented, deliberate fallbacks (default args, absent-key
degrades) -- see the allowlist table below (TC9-TC11, T5).

**TC9-TC11 (T5, near the bottom of this file)** are a static AST lint, not a
behavioral test: it walks every ``.py`` under ``tools/path_planner/`` (minus
``tests/``) and fails if a ``PIPELINE_CONFIG`` singleton read appears outside
the allowlist above. It is the actual answer to "was the audit complete?" --
TC1-TC8/TC12 only prove the three found gaps are fixed; TC9 is what proves a
fourth one is not still sitting somewhere in the ~40-file tree.

**TC1 and TC2 live in ``test_compiler.py``, not here.** T2 already added
``test_final_heading_after_pipeline_direction_default_matches_compile_path``
(TC1: a field-sourced ``PIPELINE_DIRECTION_MODE: FORCE_REVERSE`` makes
``final_heading_after`` agree with ``compile_path``) and
``test_final_heading_after_no_field_data_stays_front_resolved`` (TC2: the
``field_data=None`` control) to ``tools/path_planner/tests/test_compiler.py``,
beside the existing REAR-final-hop precedent pair they extend. They are not
reimplemented or moved here -- this file picks up at TC3.

**A related trap, noted but NOT fixed here (out of scope for this file):**
``tools/path_planner/tests/test_turn_gate_parity.py`` (see
``test_tc13_single_near_collinear_hop_emits_no_turn`` ~line 278,
``test_tc13_multi_hop_cumulative_no_compiler_only_turn`` ~line 311,
``test_tc13_skewed_collinear_fixture_emits_no_turn`` ~line 344, and
``test_tc13_turn_emission_boundary`` ~line 369) calls
``compile_path(path, ..., graph, field_data)`` WITH ``field_data`` and then
``final_heading_after(path, ..., graph)`` WITHOUT it. That is inert today only
because every fixture those tests build authors no ``PIPELINE_*`` key (both
calls fall back to the same singleton-equivalent config either way) -- it is
exactly the call shape #148 is about, one fixture attribute away from
re-creating the divergence inside a parity test that is supposed to prove the
two surfaces agree. Left alone because that file is out of scope for this task.
"""

from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace
from typing import List

import pytest

from tools.path_planner.compiler import (
    ACT_TURN_REL,
    Primitive,
    compile_path,
    final_heading_after,
    normalize_turn,
)
from tools.path_planner.graph_builder import EDGE_BLIND, build_graph
from tools.path_planner.header_writer import emit_routes
from tools.path_planner.pipeline_config import (
    PIPELINE_CONFIG,
    BlindMode,
    DirectionMode,
    PipelineConfig,
)
from tools.path_planner.planner import BlindDecayParams, compute_all_pairs_next_hop
from tools.path_planner.sim.runtime import (
    ACT_BLIND_REVERSE,
    NAV_TURN_TOLERANCE_DEG,
    Direction,
    _pass_through_heading_eligible,
)
from tools.path_planner.validator import FieldDataError, validate


def _build_field(**overrides) -> SimpleNamespace:
    """Return a self-contained field-data proxy that survives the REAL pipeline.

    ``validate()`` -> ``build_graph()`` -> ``compile_path()`` ->
    ``final_heading_after()`` -> ``emit_routes()`` all read this fixture, so
    every attribute the STRICT emitter reads (bare ``field_data.X``, no
    ``getattr`` default -- see ``header_writer.py``: ``NODES``, ``NODE_TYPES``,
    ``START_NODE``, ``START_HEADING_DEG``, ``BLIND_PROJECTION_AUTO_SEEK``,
    ``RESET_NODES``) must be present, modeled on
    ``test_header_writer.py::_small_pipeline_field`` and
    ``test_sim_pipeline_parity.py::_build_fixture``.

    Topology: ``S`` (blind spawn) -> ``A`` -> ``B`` -> ``C`` -> ``D``, all
    BLIND edges (no LINE_SEGMENTS needed -- an all-blind field keeps the
    fixture minimal). ``PIVOT_TURNS = [("A", "B", "C")]`` is a real 90 deg
    single-wheel pivot (reference geometry: track 18.0 cm -> delta 9.000 cm,
    comfortably inside both the 12.02 cm FRONT and the 7.68 cm REAR
    sensor-to-axle overshoot budgets, and inside the plain 100 cm leg length
    used for the DISTANCE-regime trim floor check).

    **Fixture hazard (read before touching the pivot legs) -- corrected:** a
    REAR-resolved pivot APPROACH budgets the smaller REAR sensor-to-axle
    offset instead of the larger FRONT one (see
    ``test_field_data.py:2352-2373`` for the shape of that failure), but that
    mechanism (validator.py check 4, ADR-044) only fires for a 'line' /
    'sensor'-terminated approach. This fixture's pivot legs are BLIND with no
    ``stop_at_line`` override and a non-``STOP_AT_LINE``
    ``PIPELINE_BLIND_MODE``, so they classify 'distance': check 4 never
    fires, and the 100 cm leg length absorbs the 9.0 cm delta trivially
    regardless of direction -- confirmed, a pin-free variant of THIS default
    fixture validates cleanly. The ``direction: "front"`` pins on both pivot
    legs (``A``->``B``, ``B``->``C``) are therefore NOT load-bearing for the
    default fixture (TC3/TC4/TC5/TC7). They ARE load-bearing in TC6, whose
    ``stop_at_line: "on"`` override deliberately flips the leg to
    'sensor'-terminated so check 4 DOES fire, and the pin is what keeps the
    budget on the larger FRONT offset instead of the smaller REAR one. TC8
    deliberately drops these pins (see its docstring) precisely because it
    needs the opposite: a 'sensor'-terminated, REAR-reachable leg, to
    discriminate the ``_pipeline_cfg`` degrade branch. Keep the pins on the
    default fixture for parity with TC6's geometry, but do not assume they
    are protecting this default fixture from a budget rejection -- they are
    not.

    The two edges NOT part of the pivot triple (``S``->``A`` and ``C``->``D``)
    carry no per-edge override at all, so they inherit whatever
    ``PIPELINE_DIRECTION_MODE`` / ``PIPELINE_BLIND_MODE`` the field authors --
    that is what lets TC3 observe the field-sourced blind mode in an emitted
    ``ACT_BLIND_REVERSE`` payload, and what makes the ``C``->``D`` hop (the
    LAST hop of the canonical path) resolve a genuinely field-dependent final
    heading rather than one dictated by the front-pinned pivot legs.

    :param overrides: attribute name -> value pairs applied via ``setattr``
        after the baseline is built. The four ``PIPELINE_*`` keys default to
        the Data Model table's NON-default values
        (``FAST`` / ``GYRO_TSHAPE`` / ``PIVOT_LEFT`` / ``FORCE_REVERSE``);
        pass ``None`` for a key to fall back to the ``pipeline_config.py``
        singleton for that category. ``EDGES`` / ``PIVOT_TURNS`` /
        ``NODES`` / ``NODE_TYPES`` are overridable the same way for the
        validator-only unit tests (TC5-TC8) that need a differently-shaped
        pivot leg without paying for the whole strict-emitter skeleton.
    :return: a ``SimpleNamespace`` field-data proxy.
    """
    edges = [
        {"type": EDGE_BLIND, "u": "S", "v": "A", "distance_cm": 100.0, "heading_deg": 0.0},
        {
            "type": EDGE_BLIND, "u": "A", "v": "B", "distance_cm": 100.0,
            "heading_deg": 90.0, "direction": "front",
        },
        {
            "type": EDGE_BLIND, "u": "B", "v": "C", "distance_cm": 100.0,
            "heading_deg": 180.0, "direction": "front",
        },
        {"type": EDGE_BLIND, "u": "C", "v": "D", "distance_cm": 100.0, "heading_deg": 90.0},
    ]
    field = SimpleNamespace(
        NODES={
            "S": (0.0, -100.0),
            "A": (0.0, 0.0),
            "B": (100.0, 0.0),
            "C": (100.0, 100.0),
            "D": (100.0, 200.0),
        },
        NODE_TYPES={"S": "blind", "A": "blind", "B": "blind", "C": "blind", "D": "blind"},
        LINE_SEGMENTS=[],
        BLIND_EDGES=[],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE="S",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        EDGES=edges,
        BLIND_PROJECTION_AUTO_SEEK=True,
        RESET_NODES={},
        PIVOT_TURNS=[("A", "B", "C")],
        PIPELINE_SPEED="FAST",
        PIPELINE_BLIND_MODE="GYRO_TSHAPE",
        PIPELINE_TURN_MODE="PIVOT_LEFT",
        PIPELINE_DIRECTION_MODE="FORCE_REVERSE",
    )
    for key, value in overrides.items():
        setattr(field, key, value)
    return field


def _final_body_heading(
    primitives: List[Primitive],
    start_heading_deg: float,
    raw_hop_headings_deg: List[float],
) -> float:
    """Replay only the ``ACT_TURN_REL`` steps to recover the final body heading.

    Mirrors the "one logical-heading" rule (#147) -- but ONLY for a fixture
    whose every hop-to-hop turn is strictly greater than
    :data:`tools.path_planner.sim.runtime.NAV_TURN_TOLERANCE_DEG` (0.5 deg).
    Under that condition ``compile_path`` emits an ``ACT_TURN_REL`` at every
    hop transition, so the running heading changes ONLY via an emitted delta
    and summing those over ``start_heading_deg`` recovers exactly the heading
    ``compile_path`` leaves the chassis on -- the same value
    :func:`final_heading_after` computes independently.

    **This is NOT a general invariant.** #147 lets ``compile_path`` advance
    the logical heading across a sub-tolerance ("near-collinear") hop WITHOUT
    emitting any ``ACT_TURN_REL`` for it -- see
    ``test_turn_gate_parity.py::test_tc13_single_near_collinear_hop_emits_no_turn``,
    which asserts ``_turns(compiled) == []`` for a hop whose
    ``final_heading_after`` delta is still non-zero. Replaying primitives
    alone would silently miss that invisible delta and read as a phantom T2
    regression against :func:`final_heading_after` -- exactly why the
    precondition below is asserted eagerly instead of left as a docstring
    warning.

    ``raw_hop_headings_deg`` is the per-hop RAW edge heading along the path
    (``graph[u][v]["heading_deg"]`` for each consecutive pair, in path
    order) -- a pre-direction-resolution approximation of the turn the
    compiler actually gates on, not the body-heading-resolved one
    :func:`final_heading_after` computes. It is exact for every fixture in
    this file: every raw delta here is exactly 90 deg, and a FRONT/REAR flip
    at a hop boundary maps a raw delta ``d`` to a body delta of
    ``180 - d`` -- still 90 deg either way, safely clear of the 0.5 deg
    tolerance. A future fixture that combines a direction flip with a raw
    turn near the tolerance boundary would need a body-heading-resolved
    check instead; this cheap raw check is not a substitute for that case.

    A narrower, single-purpose sibling of
    ``test_compiler.py::_replay_primitives`` (which also tracks the heading at
    every motion step); this file only ever needs the last one.
    """
    for previous_heading, next_heading in zip(
        raw_hop_headings_deg, raw_hop_headings_deg[1:]
    ):
        turn = normalize_turn(next_heading - previous_heading)
        assert abs(turn) > NAV_TURN_TOLERANCE_DEG, (
            f"_final_body_heading is only valid when every hop turn exceeds "
            f"NAV_TURN_TOLERANCE_DEG ({NAV_TURN_TOLERANCE_DEG} deg, see #147) "
            f"-- got a {turn:.3f} deg raw turn between headings "
            f"{previous_heading} -> {next_heading}. compile_path may advance "
            f"the logical heading across a hop like this WITHOUT emitting "
            f"ACT_TURN_REL, which this primitive replay cannot observe; use "
            f"final_heading_after directly instead of this helper."
        )
    heading = start_heading_deg
    for primitive in primitives:
        if primitive[0] == ACT_TURN_REL:
            heading = (heading + primitive[1]) % 360.0
    return heading


# -- TC3/TC4: the headline cross-surface agreement test -----------------------


def test_tc3_all_four_pipeline_categories_agree_across_surfaces(tmp_path: Path) -> None:
    """TC3 (#148 clause (d)): one field, one test, all four PIPELINE_* categories.

    The field sets every ``PIPELINE_*`` key to its Data Model non-default value
    (``FAST`` / ``GYRO_TSHAPE`` / ``PIVOT_LEFT`` / ``FORCE_REVERSE``) and drives
    it through ``validate()`` -> ``build_graph()`` -> ``compile_path()`` ->
    ``final_heading_after()`` -> ``emit_routes()`` in that order, asserting
    every surface agrees:

    * ``validate()`` accepts the field (no ``FieldDataError``).
    * the compiled primitives' ``ACT_BLIND_REVERSE`` payloads carry
      ``BlindMode.GYRO_TSHAPE`` (observable only on a REAR blind hop -- see
      ``compiler.py``'s FRONT branch, which does not carry the mode in its
      payload at all).
    * ``final_heading_after(..., field_data=field)`` equals the final heading
      ``compile_path`` actually drives (recovered via
      :func:`_final_body_heading`).
    * the emitted ``routes.cpp``'s ``const PipelineConfig PIPELINE_CONFIG = {``
      block carries ``FAST`` / ``GYRO_TSHAPE`` / ``PIVOT_LEFT`` /
      ``FORCE_REVERSE``.

    This test is discriminating for the T2 fix specifically: the canonical
    path's LAST hop (``C``->``D``) carries no per-edge direction override, so
    its resolved direction -- and therefore the final heading -- depends on
    whether ``final_heading_after`` is given this field's
    ``PIPELINE_DIRECTION_MODE: FORCE_REVERSE``. Calling it WITHOUT
    ``field_data`` on the same path returns a materially different heading
    (proven inline below), which is exactly the silent-180-degree divergence
    #148 found.
    """
    field = _build_field()
    assert validate(field) is None

    graph = build_graph(field)
    path = ["S", "A", "B", "C", "D"]
    start_heading = 0.0

    primitives = compile_path(path, start_heading, graph, field)
    blind_reverse_modes = [
        primitive[-1] for primitive in primitives if primitive[0] == ACT_BLIND_REVERSE
    ]
    assert blind_reverse_modes, "expected at least one ACT_BLIND_REVERSE primitive"
    assert all(mode == BlindMode.GYRO_TSHAPE for mode in blind_reverse_modes)

    raw_hop_headings_deg = [graph[u][v]["heading_deg"] for u, v in zip(path, path[1:])]
    compiled_final_heading = _final_body_heading(
        primitives, start_heading, raw_hop_headings_deg
    )
    agreed_heading = final_heading_after(path, start_heading, graph, field_data=field)
    assert agreed_heading == pytest.approx(compiled_final_heading, abs=1e-9)

    # Discriminating control: WITHOUT this field's field_data, the pipeline
    # direction default degrades to DirectionMode.DEFAULT (AUTO), so the
    # unpinned final hop C->D resolves differently and the heading diverges --
    # proving the assertion above actually depends on the T2 field-sourcing
    # fix, not on some incidental symmetry of the fixture.
    unsourced_heading = final_heading_after(path, start_heading, graph)
    assert unsourced_heading != pytest.approx(agreed_heading, abs=1e-9)

    header_path = tmp_path / "routes.h"
    source_path = tmp_path / "routes.cpp"
    next_hop = compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(field))
    emit_routes(field, graph, next_hop, header_path=header_path, source_path=source_path)
    cpp_text = source_path.read_text(encoding="utf-8")
    cfg_start = cpp_text.index("const PipelineConfig PIPELINE_CONFIG = {")
    cfg_end = cpp_text.index("};", cfg_start)
    cfg_block = cpp_text[cfg_start:cfg_end]
    assert "SpeedTier::FAST" in cfg_block
    assert "BlindMode::GYRO_TSHAPE" in cfg_block
    assert "TurnMode::PIVOT_LEFT" in cfg_block
    assert "DirectionMode::FORCE_REVERSE" in cfg_block


def test_tc4_absent_pipeline_keys_resolve_to_singleton_defaults_everywhere(
    tmp_path: Path,
) -> None:
    """TC4: the same pipeline, but the field omits all four PIPELINE_* keys.

    Every surface must independently fall back to the ``pipeline_config.py``
    singleton defaults (``NORMAL`` / ``SYNC_TSHAPE`` / ``COUNTER_ROT`` /
    ``DEFAULT``) -- the absent-key control that makes TC3 meaningful (a
    fixture that always emitted the same block regardless of the field would
    not prove anything).
    """
    field = _build_field(
        PIPELINE_SPEED=None,
        PIPELINE_BLIND_MODE=None,
        PIPELINE_TURN_MODE=None,
        PIPELINE_DIRECTION_MODE=None,
    )
    assert validate(field) is None

    graph = build_graph(field)
    path = ["S", "A", "B", "C", "D"]
    start_heading = 0.0

    primitives = compile_path(path, start_heading, graph, field)
    raw_hop_headings_deg = [graph[u][v]["heading_deg"] for u, v in zip(path, path[1:])]
    compiled_final_heading = _final_body_heading(
        primitives, start_heading, raw_hop_headings_deg
    )
    agreed_heading = final_heading_after(path, start_heading, graph, field_data=field)
    assert agreed_heading == pytest.approx(compiled_final_heading, abs=1e-9)

    header_path = tmp_path / "routes.h"
    source_path = tmp_path / "routes.cpp"
    next_hop = compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(field))
    emit_routes(field, graph, next_hop, header_path=header_path, source_path=source_path)
    cpp_text = source_path.read_text(encoding="utf-8")
    cfg_start = cpp_text.index("const PipelineConfig PIPELINE_CONFIG = {")
    cfg_end = cpp_text.index("};", cfg_start)
    cfg_block = cpp_text[cfg_start:cfg_end]
    assert "SpeedTier::NORMAL" in cfg_block
    assert "BlindMode::SYNC_TSHAPE" in cfg_block
    assert "TurnMode::COUNTER_ROT" in cfg_block
    assert "DirectionMode::DEFAULT" in cfg_block


# -- TC5-TC8: validator._regime unit-level pins (T1) ---------------------------


def test_tc5_blind_pivot_leg_per_edge_mode_and_field_blind_default_both_classify_distance() -> None:
    """TC5: a BLIND pivot leg's per-edge ``mode`` and the field ``PIPELINE_BLIND_MODE``
    both resolve to a non-seek :class:`BlindMode`, so both pivot legs classify
    ``'distance'`` (dead-reckoned, over-travel trim applies) -- neither is a
    seek trigger.

    The incoming leg (``A``->``B``) carries a per-edge ``mode: 'GYRO_TSHAPE'``;
    the field sets ``PIPELINE_BLIND_MODE: 'RAW_FLAT'``. Per T1, ``_regime``
    resolves the incoming leg's own effective mode via
    ``resolve_blind_mode(edge_data, cfg)`` (the edge's ``mode`` beats the field
    default) and the outgoing leg (no per-edge ``mode``) inherits the
    field-sourced ``RAW_FLAT`` -- neither is ``BlindMode.STOP_AT_LINE``, so
    ``resolve_stop_at_line``'s rung 3 never fires and both legs stay
    DISTANCE-terminated. This is honestly a same-verdict invariant, not a
    behavior-flip: ``STOP_AT_LINE`` is unreachable from authored field data
    (rejected at the EDGES-shape step, before this pass, and degraded away by
    ``_pipeline_cfg`` when it comes from the field default) -- see the plan's
    Decisions section. Its value is proving the T1 threading (the new 4th
    ``_edge_attrs`` tuple element, the widened ``_regime`` signature) runs
    end-to-end without raising and without silently reclassifying a plain
    dead-reckoned pivot as a seek, which is exactly the ADR-064 invariant.
    """
    field = _build_field(
        PIPELINE_BLIND_MODE="RAW_FLAT",
        EDGES=[
            {
                "type": EDGE_BLIND, "u": "A", "v": "B", "distance_cm": 100.0,
                "heading_deg": 90.0, "direction": "front", "mode": "GYRO_TSHAPE",
            },
            {
                "type": EDGE_BLIND, "u": "B", "v": "C", "distance_cm": 100.0,
                "heading_deg": 180.0, "direction": "front",
            },
        ],
    )
    assert validate(field) is None


def test_tc6_stop_at_line_axis_drives_sensor_regime_independent_of_blind_axis() -> None:
    """TC6: a per-edge ``stop_at_line: 'on'`` classifies 'sensor' regardless of
    ``PIPELINE_BLIND_MODE``.

    The incoming pivot leg (``A``->``B``) is deliberately only 2.0 cm long --
    too short to survive the DISTANCE-regime dead-reckon trim floor check
    (``length - delta < PIVOT_TRIM_MIN_DRIVE_CM``; 2.0 - 9.0 is deeply
    negative). A per-edge ``stop_at_line: 'on'`` override makes the leg
    SENSOR-terminated (rung 2 of ``resolve_stop_at_line`` -- an explicit
    per-edge override outranks everything, including the blind-mode axis),
    which takes no DISTANCE trim at all and instead budgets the (larger,
    FRONT-pinned) 12.02 cm sensor/LINE overshoot offset -- comfortably
    covering the 9.0 cm delta. ``PIPELINE_BLIND_MODE: 'GYRO_TSHAPE'`` is set
    on the field purely to prove the seek classification does not come from
    that axis.

    Discriminating for the ``stop_at_line`` -> sensor-regime MECHANISM (rung 2
    of ``resolve_stop_at_line`` outranks rung 3 regardless of the blind
    axis), NOT for the T1 field-sourcing fix: rung 2 is the same per-edge
    override check whether ``_regime`` is fed the field-sourced ``cfg`` or
    the pre-T1 bare ``PIPELINE_CONFIG`` singleton, so this test's verdict
    does not flip if T1 is reverted (same honesty as TC5's docstring; unlike
    TC8, which needs its unpinned pivot legs specifically to discriminate
    T1's ``_pipeline_cfg`` degrade branch). The control below discriminates
    the override itself: the SAME short leg WITHOUT the ``stop_at_line``
    override is classified 'distance' and rejects on the dead-reckon trim
    floor -- proving the override, not the fixture geometry, is what makes
    the pivot validate.
    """
    edges = [
        {
            "type": EDGE_BLIND, "u": "A", "v": "B", "distance_cm": 2.0,
            "heading_deg": 90.0, "direction": "front", "stop_at_line": "on",
        },
        {
            "type": EDGE_BLIND, "u": "B", "v": "C", "distance_cm": 100.0,
            "heading_deg": 180.0, "direction": "front",
        },
    ]
    field = _build_field(PIPELINE_BLIND_MODE="GYRO_TSHAPE", EDGES=edges)
    assert validate(field) is None

    # Control: drop the stop_at_line override on the identical short leg ->
    # falls back to 'distance' regime -> the 2.0 cm leg cannot absorb the
    # 9.0 cm delta and over-travel trim floor rejects it.
    control_edges = [dict(edge) for edge in edges]
    del control_edges[0]["stop_at_line"]
    control_field = _build_field(PIPELINE_BLIND_MODE="GYRO_TSHAPE", EDGES=control_edges)
    with pytest.raises(FieldDataError, match=r"INCOMING dead-reckoned edge length"):
        validate(control_field)


def test_tc7_malformed_pipeline_blind_mode_raises_precise_field_data_error() -> None:
    """TC7: a malformed ``PIPELINE_BLIND_MODE`` on a field with pivot turns fails
    with the precise ``FieldDataError`` from ``_validate_pipeline_config``, not
    a bare ``KeyError`` escaping the earlier pivot pass.

    Guards AC2 / the ADR-029 diagnostic contract: ``_pipeline_cfg()`` (T1) must
    swallow the ``KeyError`` from ``BlindMode['NONSENSE']`` and degrade to the
    singleton so the pivot pass (step 10) completes cleanly on default
    assumptions, letting ``_validate_pipeline_config`` (step 13) -- which runs
    AFTER the pivot pass -- report the one precise, key-pathed diagnostic.
    Without the T1 degrade, the bare ``KeyError`` would escape from inside
    ``_validate_pivot_turns`` instead.
    """
    field = _build_field(PIPELINE_BLIND_MODE="NONSENSE")
    with pytest.raises(FieldDataError, match=r"PIPELINE_BLIND_MODE"):
        validate(field)


def test_tc8_pipeline_blind_mode_stop_at_line_still_rejected_adr064() -> None:
    """TC8: ``PIPELINE_BLIND_MODE: 'STOP_AT_LINE'`` still raises the ADR-064
    ``FieldDataError`` ("author stop_at_line: on"), proving the T1 fix did not
    reopen the retired legacy-seek pipeline default.

    ``'STOP_AT_LINE'`` is a VALID :class:`BlindMode` member name, so
    ``PipelineConfig.from_field_data`` resolves it without raising --
    ``_pipeline_cfg()``'s exception guard alone would not catch this. The
    second, explicit ``cfg.blind is BlindMode.STOP_AT_LINE`` degrade (T1) is
    what keeps the pivot pass from misclassifying every non-overridden BLIND
    leg 'sensor' one pass early and tripping a REAR-impossible / budget
    rejection BEFORE step 13 gets to report the real ADR-064 message.

    **The pivot legs are deliberately UNPINNED here** (no per-edge
    ``direction`` key), unlike the default fixture. The default fixture's
    ``direction: "front"`` pins keep the budget check on the larger FRONT
    offset (12.02 cm) regardless of the degrade branch, so with the pins
    present this test would reach step 13 and report the ADR-064 error WITH
    or WITHOUT the degrade -- passing three ways and discriminating nothing.
    Dropping the pins lets both legs inherit the field's
    ``PIPELINE_DIRECTION_MODE: "FORCE_REVERSE"`` default, which makes the
    incoming leg reverse-reachable. WITH the degrade (current code): ``cfg``
    falls back to the singleton's non-``STOP_AT_LINE`` blind default, the
    legs classify 'distance', the sensor/LINE overshoot budget (check 4)
    never fires, and validation reaches step 13 to report the ADR-064
    message. WITHOUT the degrade (hypothetically): ``STOP_AT_LINE`` would
    resolve straight into ``cfg.blind``, the non-overridden legs would
    classify 'sensor', ``in_reverse_reachable`` would be True, and check 4
    would budget the smaller REAR offset -- ``7.68 - 9.0 cm delta < the
    0.5 cm trim floor`` -- raising a REAR-budget ``FieldDataError`` instead of
    the ADR-064 one. That is a different exception message, so this fixture
    genuinely discriminates the degrade branch (verified against the current
    code below; the pin-free field validates cleanly today and still reports
    the ADR-064 message, so this costs nothing).
    """
    edges = [
        {"type": EDGE_BLIND, "u": "S", "v": "A", "distance_cm": 100.0, "heading_deg": 0.0},
        {"type": EDGE_BLIND, "u": "A", "v": "B", "distance_cm": 100.0, "heading_deg": 90.0},
        {"type": EDGE_BLIND, "u": "B", "v": "C", "distance_cm": 100.0, "heading_deg": 180.0},
        {"type": EDGE_BLIND, "u": "C", "v": "D", "distance_cm": 100.0, "heading_deg": 90.0},
    ]
    field = _build_field(PIPELINE_BLIND_MODE="STOP_AT_LINE", EDGES=edges)
    with pytest.raises(FieldDataError, match=r"author stop_at_line: on"):
        validate(field)


# -- TC12: the sim/runtime.py allowlist entry is load-bearing ------------------


def test_tc12_pass_through_heading_eligible_none_pipeline_default_falls_back_to_singleton(
    monkeypatch,
) -> None:
    """TC12: ``_pass_through_heading_eligible(pipeline_default=None, ...)`` still
    falls back to :data:`PIPELINE_CONFIG.direction`, proving that allowlist
    entry is load-bearing and not vestigial dead code.

    An outbound edge collinear with the inbound heading (0 deg delta) is
    eligible under the singleton's ``DirectionMode.DEFAULT`` (AUTO resolves
    FRONT, matching ``inbound_direction=Direction.FRONT``) whether
    ``pipeline_default`` is omitted as ``None`` or passed explicitly as
    ``DirectionMode.DEFAULT`` -- the two must agree. Passing an explicit
    ``DirectionMode.FORCE_REVERSE`` instead flips the resolved outbound
    direction to REAR, which no longer matches the inbound FRONT direction and
    must become INeligible.

    That much alone does NOT prove the ``None`` branch reads the singleton at
    call time: since ``PIPELINE_CONFIG.direction IS DirectionMode.DEFAULT``
    today, ``pipeline_default=None`` and ``pipeline_default=DirectionMode.
    DEFAULT`` are behaviorally identical downstream regardless of whether the
    ``None``-fallback in ``sim/runtime.py`` (``pipeline_default if
    pipeline_default is not None else _pipeline_config.PIPELINE_CONFIG.
    direction``) exists at all or was deleted / hardcoded to
    ``DirectionMode.DEFAULT``. The discriminating arm monkeypatches the
    singleton itself to a non-default ``PipelineConfig`` and re-issues the
    ``pipeline_default=None`` call: if the fallback were deleted (or
    hardcoded), this call would keep resolving FRONT and stay eligible; since
    it does not, the ``None`` branch is proven to genuinely read
    ``PIPELINE_CONFIG.direction`` at call time.
    """
    assert PIPELINE_CONFIG.direction == DirectionMode.DEFAULT

    outbound_edge = {"heading_deg": 0.0}
    common_kwargs = dict(
        inbound_body_heading=0.0,
        inbound_direction=Direction.FRONT,
    )

    eligible_omitted = _pass_through_heading_eligible(
        outbound_edge, False, pipeline_default=None, **common_kwargs
    )
    eligible_explicit_default = _pass_through_heading_eligible(
        outbound_edge, False, pipeline_default=DirectionMode.DEFAULT, **common_kwargs
    )
    assert eligible_omitted is True
    assert eligible_omitted is eligible_explicit_default

    eligible_forced_reverse = _pass_through_heading_eligible(
        outbound_edge, False, pipeline_default=DirectionMode.FORCE_REVERSE, **common_kwargs
    )
    assert eligible_forced_reverse is False

    # Discriminating arm (review fix W2): monkeypatch the singleton itself so
    # the None-fallback path -- if it is actually wired up -- resolves a
    # NON-default direction. Precedent for monkeypatching PIPELINE_CONFIG in
    # a test file: test_field_data.py:1983,
    # test_pivot_turn_no_override_blind_classifies_sensor_under_stop_at_line_default.
    import tools.path_planner.pipeline_config as pc

    monkeypatch.setattr(
        pc, "PIPELINE_CONFIG", PipelineConfig(direction=DirectionMode.FORCE_REVERSE)
    )
    eligible_omitted_after_patch = _pass_through_heading_eligible(
        outbound_edge, False, pipeline_default=None, **common_kwargs
    )
    assert eligible_omitted_after_patch is False


# -- TC9-TC11: AST lint scan for singleton PIPELINE_CONFIG reads (T5, AC6) -----
#
# TC1-TC8/TC12 above prove the three known field-sourcing gaps (#148) are
# fixed. They do NOT stop a FOURTH site from drifting back to the module
# singleton -- this lint is that guard, and it is the real check on whether
# the audit that found exactly three gaps was actually complete. Two rules
# make it trustworthy:
#
# * AST, not regex. ``PIPELINE_CONFIG`` appears in ~40 docstrings, comments,
#   and error-message strings in ``validator.py`` alone (see TC11) -- a
#   substring/regex scan would drown in false positives or need a fragile
#   exclude-list. Only a real parse distinguishes an executable read from
#   prose.
# * Both node shapes are flagged, not just attribute reads. A default
#   argument (``cfg: PipelineConfig = PIPELINE_CONFIG``) and a bare-name
#   return (``return PIPELINE_CONFIG``) are ``ast.Name`` nodes, not
#   ``ast.Attribute`` nodes -- an attribute-only predicate would leave 7 of
#   the allowlist's 10 legitimate entries undetectable (dead weight) and
#   would miss a real regression laundered through an alias
#   (``c = PIPELINE_CONFIG; c.blind``).

# Scan root: every ``.py`` under ``tools/path_planner/`` EXCEPT ``tests/`` --
# test files legitimately monkeypatch and compare against the singleton (see
# ``test_field_data.py:1983``, ``test_sim_pipeline_parity.py:74``, and TC12
# above), so flagging them would be noise, not signal.
_LINT_REPO_ROOT = Path(__file__).resolve().parents[3]
_LINT_SCAN_ROOT = _LINT_REPO_ROOT / "tools" / "path_planner"
_LINT_EXCLUDED_DIR = _LINT_SCAN_ROOT / "tests"

#: Non-vacuity floor for the scan. ``Path.rglob`` on a NONEXISTENT directory
#: returns an empty iterator SILENTLY, and TC9's only assertion is
#: "no findings" -- so a wrong ``_LINT_SCAN_ROOT`` (e.g. after this file is
#: relocated one directory deeper, changing ``parents[3]``) would make TC9
#: pass vacuously and this guard would die without ever going red. That is
#: precisely the silent-drift failure the guard exists to prevent, so the
#: scan asserts it found a plausible number of files before trusting a clean
#: result. The tree has 45 today; the floor is deliberately slack so ordinary
#: churn never trips it, while a resolution bug (which yields 0) always does.
_LINT_MIN_SCANNED_FILES = 20

# The exact post-fix legitimate set, encoded verbatim from the plan's
# Contracts -> Naming table: (repo-relative path, enclosing qualified
# function name) pairs. Line-number-independent by construction, so it
# survives edits elsewhere in the file.
#
# A ``None`` function name would allowlist a WHOLE file. The plan's table used
# that form for ``pipeline_config.py``; the shipped set below deliberately does
# NOT -- it pins the one function instead (see the comment on that entry), so
# NO entry here is currently ``None`` and the ``(path, None)`` branch in
# :func:`_lint_allowlisted` is retained as unused mechanism. Do not cite this
# paragraph to re-widen an entry to whole-file: a file-wide suppression hides
# any FUTURE drifted read in that file, which is the exact failure this lint
# exists to catch. (``pipeline_config.py``'s own module-scope
# ``PIPELINE_CONFIG = PipelineConfig()`` needs no allowlisting at all -- it is
# a ``Store``, never flagged; see the visitor docstring below.)
#
# Qualified names below are shaped like Python's own ``__qualname__``: dotted
# through enclosing scopes, with a literal ``<locals>`` segment inserted
# wherever the immediately enclosing scope is a FUNCTION (never for a class or
# the module) -- precisely what a function-name-stack visitor produces
# naturally, with no special-casing.
#
# Exception: the visitor overrides neither ``visit_Lambda`` nor the
# comprehension scopes, so a read inside a lambda or a comprehension is
# attributed to the nearest enclosing NAMED function rather than to
# ``<locals>.<lambda>``. The effect is narrow and one-directional -- it can
# only widen an ALREADY-allowlisted function's suppression to a lambda nested
# inside it, never suppress a read in a function that is not on the list.
#
# The six ``sim/runtime.py`` entries are all default-argument reads
# (``cfg: PipelineConfig = PIPELINE_CONFIG``). The default expression
# evaluates exactly once, at module-IMPORT time, at MODULE scope -- but the
# ``Name`` node for it is a syntactic child of the ``FunctionDef`` it
# defaults for, and this visitor pushes the enclosing-scope stack before
# descending into ``node.args`` (see ``_visit_scoped`` below), so the
# finding is attributed to the function being defined, not to the module.
# That is a deliberate design choice, not an accident: it is exactly what
# makes these 6 entries encodable at all (a module-scope-only allowlist could
# not distinguish "this file's default-arg fallbacks" from "an actual
# module-level PIPELINE_CONFIG read"), and it is verified empirically below
# -- every one of the 10 entries is exercised by scanning the real tree
# (confirmed by hand against a probe run before this file was written: zero
# post-allowlist findings, zero unused/dead allowlist entries).
_LINT_ALLOWLIST: frozenset[tuple[str, str | None]] = frozenset(
    {
        # Narrower than the plan's table, which allowlisted this file WHOLE.
        # The file's only singleton READ is from_field_data's absent-key
        # fallback (its module-scope ``PIPELINE_CONFIG = PipelineConfig()`` is
        # a Store, never flagged), so pinning the function costs nothing today
        # and stops a genuinely drifted NEW read elsewhere in the file from
        # being waved through -- the widest hole in the original table.
        ("tools/path_planner/pipeline_config.py", "PipelineConfig.from_field_data"),
        ("tools/path_planner/sim/runtime.py", "resolve_speed"),
        ("tools/path_planner/sim/runtime.py", "resolve_blind_mode"),
        ("tools/path_planner/sim/runtime.py", "resolve_line_follow_mode"),
        ("tools/path_planner/sim/runtime.py", "_pass_through_heading_eligible"),
        ("tools/path_planner/sim/runtime.py", "PathTrackerSim.resolve_speed"),
        ("tools/path_planner/sim/runtime.py", "PathTrackerSim.resolve_blind_mode"),
        ("tools/path_planner/sim/runtime.py", "PathTrackerSim.resolve_line_follow_mode"),
        (
            "tools/path_planner/validator.py",
            "_validate_pivot_turns.<locals>._pipeline_direction_default",
        ),
        (
            "tools/path_planner/validator.py",
            "_validate_pivot_turns.<locals>._pipeline_cfg",
        ),
    }
)


class _PipelineConfigSingletonVisitor(ast.NodeVisitor):
    """Collect every disallowed ``PIPELINE_CONFIG`` singleton READ.

    Maintains a stack of ``(scope_kind, name)`` for every enclosing
    ``def``/``async def``/``class``, so each finding carries a
    ``__qualname__``-shaped enclosing function name (e.g.
    ``_validate_pivot_turns.<locals>._pipeline_direction_default`` or
    ``PathTrackerSim.resolve_speed``) -- exactly what
    :data:`_LINT_ALLOWLIST` keys on.

    Flags BOTH node shapes; an attribute-only predicate would leave most of
    the allowlist dead and miss a real regression laundered through an alias:

    (a) ``ast.Name`` with ``id == "PIPELINE_CONFIG"`` in a ``Load`` context.
        ``Store`` is skipped deliberately -- it is the singleton's own
        definition (``PIPELINE_CONFIG = PipelineConfig()``) in
        ``pipeline_config.py``, not a read. Catches default args
        (``cfg: PipelineConfig = PIPELINE_CONFIG``), ``return
        PIPELINE_CONFIG``, and aliasing (``c = PIPELINE_CONFIG; c.blind`` --
        the alias's own RHS ``Name`` is what gets flagged).
    (b) ``ast.Attribute`` with ``attr == "PIPELINE_CONFIG"`` -- the dotted
        ``_pipeline_config.PIPELINE_CONFIG.direction`` form in
        ``sim/runtime.py``'s ``_pass_through_heading_eligible``, where the
        singleton is reached through a module attribute, never as a bare
        ``Name`` at all, so predicate (a) alone would miss it.

    ``from ... import PIPELINE_CONFIG`` produces no ``Name`` node at all
    (import aliases are ``ast.alias`` nodes, not ``ast.Name``), so a bare
    import is never flagged. That is intentional: an import with no
    subsequent read is dead code, not a violation of this guard.
    """

    def __init__(self) -> None:
        self._scope_stack: List[tuple[str, str]] = []
        self.findings: List[tuple[int, str]] = []  # (lineno, qualname)

    def _current_qualname(self) -> str:
        """Render the enclosing-scope stack in ``__qualname__`` shape."""
        parts: List[str] = []
        previous_kind: str | None = None
        for kind, name in self._scope_stack:
            if previous_kind == "function":
                parts.append("<locals>")
            parts.append(name)
            previous_kind = kind
        return ".".join(parts)

    def _visit_scoped(
        self,
        node: ast.FunctionDef | ast.AsyncFunctionDef | ast.ClassDef,
        kind: str,
    ) -> None:
        self._scope_stack.append((kind, node.name))
        self.generic_visit(node)
        self._scope_stack.pop()

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        # Pushed BEFORE generic_visit descends into node.args, so a default
        # argument's Name node is attributed to this function -- see the
        # class docstring's note on the six sim/runtime.py entries.
        self._visit_scoped(node, "function")

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        self._visit_scoped(node, "function")

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        self._visit_scoped(node, "class")

    def visit_Name(self, node: ast.Name) -> None:
        if node.id == "PIPELINE_CONFIG" and isinstance(node.ctx, ast.Load):
            self.findings.append((node.lineno, self._current_qualname()))
        self.generic_visit(node)

    def visit_Attribute(self, node: ast.Attribute) -> None:
        if node.attr == "PIPELINE_CONFIG":
            self.findings.append((node.lineno, self._current_qualname()))
        self.generic_visit(node)


def _lint_allowlisted(repo_relative_path: str, qualname: str) -> bool:
    """True iff ``(repo_relative_path, qualname)`` is a documented fallback."""
    if (repo_relative_path, None) in _LINT_ALLOWLIST:
        return True
    return (repo_relative_path, qualname) in _LINT_ALLOWLIST


def _lint_format_finding(repo_relative_path: str, lineno: int, qualname: str) -> str:
    """Render one finding.

    MUST name the offending ``file:line`` and MUST contain the literal
    substring ``PipelineConfig.from_field_data`` so the fix is discoverable
    from the assertion failure alone, with no need to open this module (AC6).
    """
    location = f"{repo_relative_path}:{lineno}"
    enclosing = qualname if qualname else "<module scope>"
    return (
        f"{location} reads the PIPELINE_CONFIG singleton inside {enclosing}, "
        "outside the documented lint allowlist -- use "
        "PipelineConfig.from_field_data(field_data) instead (issue #148)."
    )


def _lint_scan_source(source: str, repo_relative_path: str) -> List[tuple[int, str]]:
    """Parse ``source`` and return non-allowlisted ``(lineno, qualname)`` findings.

    Shared by the real-tree scan (:func:`_lint_scan_tree`, TC9) and the
    synthetic sources TC10/TC11 build inline, so every test exercises the
    identical visitor + allowlist logic -- no separate "test-only" scan path
    to drift out of sync with the real one.

    An unparseable source fails LOUDLY, naming the file, rather than being
    silently skipped -- a skipped file is a hole in the guard, not a clean
    result.
    """
    try:
        tree = ast.parse(source, filename=repo_relative_path)
    except SyntaxError as exc:
        raise AssertionError(
            f"pipeline_config lint: could not parse {repo_relative_path}: {exc}"
        ) from exc
    visitor = _PipelineConfigSingletonVisitor()
    visitor.visit(tree)
    return [
        (lineno, qualname)
        for lineno, qualname in visitor.findings
        if not _lint_allowlisted(repo_relative_path, qualname)
    ]


def _lint_iter_scan_files() -> List[Path]:
    """Every ``.py`` under ``tools/path_planner/``, excluding ``tests/``.

    Asserts the scan root resolved to a real directory holding a plausible
    number of files BEFORE returning. Without this the guard can fail closed:
    ``rglob`` on a nonexistent path returns an empty iterator silently, TC9's
    "no findings" assertion then holds vacuously, and the lint is dead while
    CI stays green -- the same silent drift the lint exists to catch. See
    :data:`_LINT_MIN_SCANNED_FILES`.
    """
    assert _LINT_SCAN_ROOT.is_dir(), (
        f"pipeline_config lint: scan root {_LINT_SCAN_ROOT} is not a directory "
        "-- _LINT_REPO_ROOT resolved wrong (did this test file move? it uses "
        "Path(__file__).parents[3]). The scan would silently find nothing and "
        "TC9 would pass vacuously."
    )
    paths = sorted(
        path
        for path in _LINT_SCAN_ROOT.rglob("*.py")
        if _LINT_EXCLUDED_DIR not in path.parents
    )
    assert len(paths) >= _LINT_MIN_SCANNED_FILES, (
        f"pipeline_config lint: only {len(paths)} file(s) under "
        f"{_LINT_SCAN_ROOT}, below the non-vacuity floor of "
        f"{_LINT_MIN_SCANNED_FILES}. Either the scan root is wrong or the tree "
        "shrank drastically; a clean result here would be meaningless."
    )
    return paths


def _lint_scan_tree() -> List[str]:
    """Scan the real post-fix tree; return formatted findings (empty == clean)."""
    messages: List[str] = []
    for path in _lint_iter_scan_files():
        repo_relative_path = path.relative_to(_LINT_REPO_ROOT).as_posix()
        source = path.read_text(encoding="utf-8")
        for lineno, qualname in _lint_scan_source(source, repo_relative_path):
            messages.append(_lint_format_finding(repo_relative_path, lineno, qualname))
    return messages


def test_tc9_lint_scan_over_post_fix_tree_yields_zero_findings() -> None:
    """TC9: the scan over the REAL post-fix ``tools/path_planner/`` tree is clean.

    This is the actual audit-completeness check (AC6): TC1-TC8/TC12 prove the
    three T1-T3 fixes are correct, but only this whole-tree scan proves no
    site OUTSIDE the allowlist reads the singleton -- including files this
    plan never touched (``ctps/``, ``editor/``, ``sim/parity.py``,
    ``header_writer.py``, ``field_data.py``, ...).

    If this reddens on a site the allowlist does not cover, that means the
    audit missed a real site: either a genuine 4th field-sourcing defect, or
    a legitimate fallback the plan's Naming table missed. Either way, the fix
    is to investigate and (if legitimate) extend :data:`_LINT_ALLOWLIST` with
    a written justification -- never to loosen the visitor's node predicate.
    """
    findings = _lint_scan_tree()
    assert findings == [], "\n".join(findings)


def test_tc10_lint_flags_singleton_attribute_read_outside_allowlist() -> None:
    """TC10: a synthetic ``PIPELINE_CONFIG.blind`` read outside the allowlist
    is flagged exactly once, with a message naming the line number and citing
    the fix.

    A negative-path CONTENT assertion, not just "the scan found something":
    proves the finding is attributed to the RIGHT line and enclosing function,
    and that the rendered message is actionable on its own (AC6 requires the
    failure message to name ``file:line`` AND contain the literal substring
    ``PipelineConfig.from_field_data``).
    """
    source = (
        "def _not_on_the_allowlist():\n"
        "    x = PIPELINE_CONFIG.blind\n"
        "    return x\n"
    )
    findings = _lint_scan_source(source, "tools/path_planner/_tc10_synthetic.py")

    assert len(findings) == 1
    lineno, qualname = findings[0]
    assert lineno == 2
    assert qualname == "_not_on_the_allowlist"

    message = _lint_format_finding(
        "tools/path_planner/_tc10_synthetic.py", lineno, qualname
    )
    assert "tools/path_planner/_tc10_synthetic.py:2" in message
    assert "PipelineConfig.from_field_data" in message


def test_tc11_lint_ignores_docstring_and_comment_mentions() -> None:
    """TC11: ``PIPELINE_CONFIG`` mentioned ONLY in a docstring and a comment
    yields zero findings -- this is what proves the AST walk beats a regex.

    A regex for ``PIPELINE_CONFIG\\.\\w+`` (or even a bare substring check)
    would match both lines below. The AST walk does not, because neither a
    docstring (an ``ast.Constant`` string literal, not a ``Name``/``Attribute``
    node) nor a comment (not part of the AST at all -- the tokenizer discards
    it before parsing) is a node this visitor's predicates can match.
    """
    source = (
        '"""Module docstring mentioning PIPELINE_CONFIG.blind for prose only."""\n'
        "\n"
        "def _uses_the_name_only_in_prose():\n"
        '    """See PIPELINE_CONFIG.blind for background -- not a real read."""\n'
        "    # PIPELINE_CONFIG.blind is read at C++ dispatch time, not here.\n"
        "    return 42\n"
    )
    findings = _lint_scan_source(source, "tools/path_planner/_tc11_synthetic.py")
    assert findings == []


def test_tc9b_lint_fails_loudly_on_an_unparseable_file() -> None:
    """TC9b: an unparseable source raises, naming the file -- never skipped.

    The plan's Error Handling section calls this out explicitly: "Lint
    encounters an unparseable ``.py`` file: fail the test loudly with the
    filename rather than skipping it. A silently-skipped file is a hole in the
    guard." A ``try/except SyntaxError: continue`` would let a drifted site
    hide behind an unrelated syntax error, so the behavior is pinned here
    rather than left to inspection (issue #148).
    """
    with pytest.raises(AssertionError) as excinfo:
        _lint_scan_source("def broken(:\n", "tools/path_planner/_tc9b_synthetic.py")

    message = str(excinfo.value)
    assert "tools/path_planner/_tc9b_synthetic.py" in message
    assert "could not parse" in message


def test_tc9c_scan_tree_wires_the_formatter_end_to_end() -> None:
    """TC9c: ``_lint_scan_tree`` really walks files and formats findings.

    TC9 asserts the tree is CLEAN and TC10 calls :func:`_lint_format_finding`
    directly, so neither one exercises the wiring BETWEEN the file walk and the
    formatter -- a scan that silently visited nothing would satisfy both. This
    pins the two halves that TC9's green result depends on: the walk yields a
    plausible file set (including files this plan never touched), and every
    scanned path is expressed relative to the repo root, which is what makes a
    finding's ``file:line`` clickable (issue #148).
    """
    scanned = _lint_iter_scan_files()

    # The walk is non-vacuous and reaches beyond the three files T1-T3 touched.
    relative = {path.relative_to(_LINT_REPO_ROOT).as_posix() for path in scanned}
    assert "tools/path_planner/compiler.py" in relative
    assert "tools/path_planner/sim/runtime.py" in relative
    assert "tools/path_planner/header_writer.py" in relative
    assert "tools/path_planner/ctps/track_builder.py" in relative

    # ...and no test file leaks in (they may read the singleton legitimately).
    assert not [name for name in relative if "/tests/" in name]

    # The formatter's message shape is what a reader acts on: file:line plus
    # the fix. Built from the same helpers _lint_scan_tree composes.
    formatted = _lint_format_finding("tools/path_planner/compiler.py", 42, "compile_path")
    assert "tools/path_planner/compiler.py:42" in formatted
    assert "compile_path" in formatted
    assert "PipelineConfig.from_field_data" in formatted
